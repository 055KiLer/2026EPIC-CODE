/**
 * @file    motion_kinematics.c
 * @brief   CoreXY 运动学纯解算（Algorithm 层）
 */
#include "motion_kinematics.h"
#include "sort_config.h"

static int32_t Kin_FToPulse(float value) {
    return (int32_t)(value >= 0.0f ? (value + 0.5f) : (value - 0.5f));
}

/* 速度(mm/s) -> 电机转速(RPM)：每转毫米数 = 每转脉冲 / 每毫米脉冲 */
static float Kin_RpmForSpeed(float speed_mm_s) {
    float mm_per_rev = (float)CFG_PULSE_PER_REV_42 / CFG_XY_PULSE_PER_MM;
    float rpm        = speed_mm_s * 60.0f / mm_per_rev;

    if (rpm < 1.0f) {
        rpm = 1.0f;
    }
    if (rpm > 300.0f) {
        rpm = 300.0f; /* 上限保护，联调标定 */
    }
    return rpm;
}

uint32_t Kinematics_MoveTimeMs(int32_t pulse, float rpm) {
    if (pulse == 0 || rpm <= 0.0f) {
        return 0u;
    }

    int32_t  abs_pulse   = (pulse > 0) ? pulse : -pulse;
    float    pulse_per_s = rpm * (float)CFG_PULSE_PER_REV_42 / 60.0f;
    uint32_t time_ms     = (uint32_t)((float)abs_pulse / pulse_per_s * 1000.0f);

    return time_ms + CFG_MOVE_EXTRA_MS;
}

uint32_t Kinematics_TrayMoveTimeMs(uint32_t pulses, float rpm) {
    if (pulses == 0u || rpm <= 0.0f) {
        return 0u;
    }

    /* 速率只由 57 电机细分数和转速决定；减速比已算在 pulses 里 */
    float    pulse_per_s = rpm * (float)CFG_PULSE_PER_REV_57 / 60.0f;
    uint32_t time_ms     = (uint32_t)((float)pulses / pulse_per_s * 1000.0f);

    return time_ms + CFG_MOVE_EXTRA_MS;
}

/* 有符号地板除：C 的 / 向零截断，负脉冲（反向走）下会把整圈边界算错 */
static int32_t Kin_FloorDiv(int32_t a, int32_t b) {
    int32_t q = a / b;

    if (((a % b) != 0) && ((a < 0) != (b < 0))) {
        q--;
    }
    return q;
}

/* 向上取整的整数除法（反向走要用它数"进入凸块作用弧"的次数） */
static int32_t Kin_CeilDiv(int32_t a, int32_t b) {
    return -Kin_FloorDiv(-a, b);
}

/* 两个脉冲记账点之间**经过几个凸块**（= 索引沿的理论次数），双向都成立。
   口径（与开关的物理行为对齐）：
     - 记账原点 = 索引沿 ⇒ 凸块只在 k×一圈 的整数倍处（不是"半个标称圈"）
     - "经过" = **进入凸块作用弧**：正转数 (from, to] 内的整数倍（终点正好压上算一次、
       起点已经压着不算）；反转对称地数 [to, from) 内的整数倍
     - 单调正转时与旧实现 (to/REV − from/REV) 逐值相同 ⇒ 举证口径不变，只是多了反向 */
uint32_t Kinematics_TrayCrossingsBetween(int32_t from_pulses, int32_t to_pulses) {
    int32_t rev = (int32_t)CFG_TRAY_PULSES_PER_REV;

    if (to_pulses == from_pulses) {
        return 0u;
    }
    if (to_pulses > from_pulses) {
        return (uint32_t)(Kin_FloorDiv(to_pulses, rev) - Kin_FloorDiv(from_pulses, rev));
    }
    return (uint32_t)(Kin_CeilDiv(from_pulses, rev) - Kin_CeilDiv(to_pulses, rev));
}

/* 累计脉冲（**有符号**：双向走时可以为负）-> 当前对准投放点的盒号 */
uint8_t Kinematics_TrayBinOf(int32_t pulses) {
    int32_t rev    = (int32_t)CFG_TRAY_PULSES_PER_REV;
    int32_t per    = (int32_t)CFG_TRAY_PULSES_PER_BIN;
    int32_t in_rev = pulses % rev;

    if (in_rev < 0) {
        in_rev += rev;
    }
    return (uint8_t)(((in_rev + per / 2) / per) % (int32_t)CFG_TRAY_BINS);
}

uint8_t Kinematics_XyInEnvelope(float x_mm, float y_mm) {
    /* 必须"两个都成立才放行"：写成"任一不成立则拒绝"时 NaN 会让两侧比较同时为假。
       2026-10-01 起工作包络 = 台架同框第三象限：X ∈ [-135, 0]、Y ∈ [-187, 0] */
    if (x_mm >= CFG_WORK_X_MIN_MM && x_mm <= CFG_WORK_X_MAX_MM && y_mm >= CFG_WORK_Y_MIN_MM &&
        y_mm <= CFG_WORK_Y_MAX_MM) {
        return 1u;
    }
    return 0u;
}

void Kinematics_XYSolve(float delta_x, float delta_y, float speed_mm_s, int32_t *pulse_a,
                        int32_t *pulse_b, float *rpm_a, float *rpm_b, uint32_t *time_ms) {
    int32_t pa        = Kin_FToPulse((delta_x + delta_y) * CFG_XY_PULSE_PER_MM * CFG_AXIS_A_SIGN);
    int32_t pb        = Kin_FToPulse((delta_x - delta_y) * CFG_XY_PULSE_PER_MM * CFG_AXIS_B_SIGN);
    int32_t abs_a     = (pa > 0) ? pa : -pa;
    int32_t abs_b     = (pb > 0) ? pb : -pb;
    int32_t abs_max   = (abs_a > abs_b) ? abs_a : abs_b;
    float   rpm_full  = 0.0f;
    float   rpm_val_a = 0.0f;
    float   rpm_val_b = 0.0f;

    if (abs_max == 0) {
        *pulse_a = 0;
        *pulse_b = 0;
        *rpm_a   = 0.0f;
        *rpm_b   = 0.0f;
        *time_ms = 0u;
        return;
    }

    /* 按脉冲比例分配转速，使两轴同时到位（路径保持直线） */
    rpm_full  = Kin_RpmForSpeed(speed_mm_s);
    rpm_val_a = rpm_full * (float)abs_a / (float)abs_max;
    rpm_val_b = rpm_full * (float)abs_b / (float)abs_max;
    if (rpm_val_a < 1.0f) {
        rpm_val_a = 1.0f;
    }
    if (rpm_val_b < 1.0f) {
        rpm_val_b = 1.0f;
    }

    *pulse_a = pa;
    *pulse_b = pb;
    *rpm_a   = rpm_val_a;
    *rpm_b   = rpm_val_b;

    uint32_t time_a = Kinematics_MoveTimeMs(pa, rpm_val_a);
    uint32_t time_b = Kinematics_MoveTimeMs(pb, rpm_val_b);
    *time_ms        = (time_a > time_b) ? time_a : time_b;
}