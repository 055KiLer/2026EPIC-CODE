/**
 * @file    motion_app.c
 * @brief   运动执行器：解算 -> 下发 -> 等待 + 记账可信度
 * @note    1) 守护任务：唯一处置 EXTI 事件的地方（急停 + 作废记账）
 *          2) XY：回零标定一次 + 相对脉冲记账；只在"没被急停"时提交记账；
 *             绝对定位先过 Kinematics_XyInEnvelope
 *          3) 转盘：单向趋近消背隙；原点开关回零时是停止条件、定位时是每圈一次的索引沿
 *          竞态：s_xy_trusted / s_stop_xy_pending 由守护任务写、分拣任务读，故 volatile
 */
#include "motion_app.h"
#include "sort_config.h"
#include "motion_kinematics.h"
#include "zdt_motor.h"
#include "bsp_gpio.h"
#include <FreeRTOS.h>
#include <task.h>

static float            s_x          = 0.0f; /* 记账坐标（原点 = 双限位触发处） */
static float            s_y          = 0.0f;
static volatile uint8_t s_xy_trusted = 0u;

static uint8_t  s_bin              = 0u;
static uint8_t  s_tray_trusted     = 0u; /* 仅分拣任务读写，无需 volatile */
static uint8_t  s_tray_moving      = 0u;
static uint32_t s_tray_pulses      = 0u; /* 自回零起累计已发脉冲（单向只增） */
static uint32_t s_tray_pulses_mark = 0u;
static uint32_t s_tray_index_mark  = 0u;
static uint32_t s_tray_start_tick  = 0u;
static uint8_t  s_tray_pending_bin = 0u;

static volatile uint8_t s_stop_xy_pending = 0u;
static volatile uint8_t s_homing_xy       = 0u; /* 1 = 回零中：压限位是预期动作 */

/* 上电自检结果：bit0=A bit1=B bit2=转盘（1 = 没通过）；未启用自检时保持 0 */
static uint8_t s_selftest_fail_mask = 0u;

/* 单台探应答。使能帧不含运动指令，电机不会走；结果只记录，不阻断启动 */
static uint8_t Motion_ProbeAxis(Zdt_ID_e id) {
    return (Zdt_Probe(id, CFG_SELFTEST_TIMEOUT_MS) == ZDT_PROBE_OK) ? 1u : 0u;
}

void Motion_SelfCheck(void) {
    uint8_t mask = 0u;

    if (Motion_ProbeAxis(ZDT_ID_A) == 0u) {
        mask |= 0x01u;
    }
    if (Motion_ProbeAxis(ZDT_ID_B) == 0u) {
        mask |= 0x02u;
    }
    if (Motion_ProbeAxis(ZDT_ID_TRAY) == 0u) {
        mask |= 0x04u;
    }

    s_selftest_fail_mask = mask;
}

uint8_t Motion_SelfCheckOk(void) {
    return (s_selftest_fail_mask == 0u) ? 1u : 0u;
}

uint8_t Motion_SelfCheckFailMask(void) {
    return s_selftest_fail_mask;
}

/* 两轴一起停（CoreXY 单停一轴会让龙门歪），两帧都发出才返回 1。
   两帧都是广播停（Zdt_Stop 固定广播地址）；帧间留间隔——台架实测背靠背帧会被驱动器丢 */
static uint8_t Motion_StopXY(void) {
    uint8_t ok_a = Zdt_Stop(ZDT_ID_A);
    uint8_t ok_b;

    vTaskDelay(pdMS_TO_TICKS(CFG_ZDT_STOP_GAP_MS));
    ok_b = Zdt_Stop(ZDT_ID_B);

    return (ok_a && ok_b) ? 1u : 0u;
}

/* travel_mm<0 即退出限位 */
static void Motion_AxisDelta(uint8_t axis, float travel_mm, float *dx, float *dy) {
    if (axis == 0u) {
        *dx = travel_mm * CFG_HOME_DIR_X;
        *dy = 0.0f;
    } else {
        *dx = 0.0f;
        *dy = travel_mm * CFG_HOME_DIR_Y;
    }
}

static uint8_t Motion_AxisSend(Zdt_ID_e id, int32_t pulse, float rpm) {
    if (pulse == 0) {
        return 1u;
    }

    Zdt_DIR_e dir       = (pulse > 0) ? ZDT_DIR_CW : ZDT_DIR_CCW;
    uint32_t  abs_pulse = (uint32_t)((pulse > 0) ? pulse : -pulse);

    return Zdt_MoveRelative(id, dir, (uint16_t)rpm, CFG_ZDT_ACCEL, abs_pulse);
}

/* 解算并下发一段 XY 合成运动；返回 0 表示指令未全部生效 */
static uint8_t Motion_SendXY(float dx, float dy, float speed_mm_s, uint32_t *time_ms) {
    int32_t  pulse_a = 0;
    int32_t  pulse_b = 0;
    float    rpm_a   = 0.0f;
    float    rpm_b   = 0.0f;
    uint32_t est_ms  = 0u;

    Kinematics_XYSolve(dx, dy, speed_mm_s, &pulse_a, &pulse_b, &rpm_a, &rpm_b, &est_ms);

    uint8_t ok_a = Motion_AxisSend(ZDT_ID_A, pulse_a, rpm_a);
    /* ★ 台架实测（09-25/28）：A/B 背靠背下发（≈1.1ms）驱动器丢帧——归零"跑斜线"的
       根源之一。台架口径 3ms 等应答+7ms 空闲；产品暂不读应答，取 10ms 纯间隔 */
    vTaskDelay(pdMS_TO_TICKS(CFG_ZDT_FRAME_GAP_MS));
    uint8_t ok_b = Motion_AxisSend(ZDT_ID_B, pulse_b, rpm_b);

    *time_ms = est_ms;
    if (!ok_a || !ok_b) {
        /* 只发出去一轴 = 那一轴已经动了，随后停止的位置没人知道（bsp_uart.h 的发送契约
           要求调用方作废记账）。不置这一位，接下来每一件都按错误基准做相对移动 */
        (void)Motion_StopXY(); /* 只有一轴动起来会走歪 */
        s_xy_trusted = 0u;
        return 0u;
    }
    return 1u;
}

static uint8_t Motion_WaitLevel(uint8_t axis, uint8_t want_active, uint32_t timeout_ms) {
    uint32_t t0 = HAL_GetTick();

    while ((HAL_GetTick() - t0) < timeout_ms) {
        uint8_t active = (axis == 0u) ? Bsp_Gpio_LimitX() : Bsp_Gpio_LimitY();

        if (active == want_active) {
            return 1u;
        }
        vTaskDelay(pdMS_TO_TICKS(2u));
    }
    return 0u;
}

/* 限位 -> 急停 + 作废记账；转盘索引沿 -> 只计数，不处置成停机 */
static void Motion_ServeEvents(void) {
    uint32_t events = Bsp_Gpio_TakeEvents();

    /* 回零期间压限位是正常动作，事件直接丢弃，否则粗压阶段就被停掉、永远收不到触发沿 */
    if (s_homing_xy != 0u) {
        return;
    }

    if ((events & BSP_GPIO_EVT_XY_LIMIT) != 0u) {
        s_xy_trusted      = 0u;
        s_stop_xy_pending = 1u;
    }

    if (s_stop_xy_pending != 0u && Motion_StopXY()) {
        s_stop_xy_pending = 0u; /* 发不出去就 1ms 后重试 */
    }
}

void Motion_SafetyTask(void *argument) {
    (void)argument;

    for (;;) {
        Bsp_Gpio_Poll(); /* 限位消抖/闩锁的松开确认：连续 40ms 松开才武装下一次按压判定 */
        Motion_ServeEvents();
        vTaskDelay(pdMS_TO_TICKS(CFG_SAFETY_PERIOD_MS));
    }
}

void Motion_Init(void) {
    s_x               = 0.0f;
    s_y               = 0.0f;
    s_xy_trusted      = 0u;
    s_bin             = (uint8_t)CFG_TRAY_ZERO_BIN;
    s_tray_pulses     = 0u;
    s_tray_trusted    = 0u;
    s_tray_moving     = 0u;
    s_stop_xy_pending = 0u;
    s_homing_xy       = 0u;
    (void)Bsp_Gpio_TakeEvents();

#if CFG_SELFTEST_ON_BOOT
    /* 上电自检三台驱动器：只发使能帧读应答，不产生任何运动。
       失败结果由 Motion_SelfCheckOk()/FailMask() 给显示与流程用（不在这里阻断） */
    Motion_SelfCheck();
#endif
}

uint8_t Motion_XyIsTrusted(void) {
    return s_xy_trusted;
}

uint8_t Motion_MoveXYBlocking(float x_mm, float y_mm, float speed_mm_s) {
    if (!s_xy_trusted) {
        return 0u; /* 记账不可信时绝不能做绝对定位，交给调用方重新回零 */
    }
    /* 软件原点角在远离限位的一侧，触发点 = (CFG_HOME_X_MAX_MM, CFG_HOME_Y_MAX_MM)
       （09-28 台架实锤开关在最大行程端）；越界目标一律拒绝、一字节都不发，
       由调用方作废本件，绝不留下"半程撞限位"的状态 */
    if (!Kinematics_XyInEnvelope(x_mm, y_mm)) {
        return 0u;
    }

    uint32_t time_ms = 0u;

    if (!Motion_SendXY(x_mm - s_x, y_mm - s_y, speed_mm_s, &time_ms)) {
        return 0u;
    }
    if (time_ms > 0u) {
        vTaskDelay(pdMS_TO_TICKS(time_ms));
    }

    /* 只有"按估算走完且途中没撞限位"才允许提交记账 */
    if (!s_xy_trusted) {
        return 0u;
    }
    s_x = x_mm;
    s_y = y_mm;
    return 1u;
}

/* 单轴回零：退出窗口 -> 粗压 -> 脱开 -> 慢压取触发沿 */
static uint8_t Motion_HomeAxis(uint8_t axis) {
    float    dx      = 0.0f;
    float    dy      = 0.0f;
    uint32_t time_ms = 0u;

    for (uint8_t attempt = 0u; attempt < 2u; attempt++) {
        /* 0) 一上来就压在限位上时必须先退出，否则"已触发"会被当成回零成功 */
        if (!Motion_WaitLevel(axis, 0u, 200u)) {
            Motion_AxisDelta(axis, -(CFG_BACKOFF_MM * 4.0f + 10.0f), &dx, &dy);
            if (!Motion_SendXY(dx, dy, CFG_HOME_SLOW_MM_S, &time_ms)) {
                return 0u;
            }
            if (!Motion_WaitLevel(axis, 0u, time_ms + CFG_HOME_WAIT_EXTRA_MS)) {
                (void)Motion_StopXY();
                continue; /* 退不出窗口 = 开关卡住/接线故障 */
            }
            (void)Motion_StopXY();
        }

        /* 1) 粗压：估算行程给足余量，等触发 */
        Motion_AxisDelta(axis, CFG_HOME_TRAVEL_MM, &dx, &dy);
        if (!Motion_SendXY(dx, dy, CFG_HOME_SPEED_MM_S, &time_ms)) {
            return 0u;
        }
        if (!Motion_WaitLevel(axis, 1u, time_ms + CFG_HOME_WAIT_EXTRA_MS)) {
            (void)Motion_StopXY();
            continue;
        }
        (void)Motion_StopXY(); /* 触发即停，不留过压 */
        vTaskDelay(pdMS_TO_TICKS(50u));

        /* 2) 必须确认真的脱开才能去取第 3 步的沿：脱开量不够时第 3 步会"在仍压着时
           立刻触发"，原点被定义在开关窗口内任意一点，之后每次定位都带固定偏移——
           这比回零失败更危险，因为它看起来是成功的 */
        Motion_AxisDelta(axis, -CFG_BACKOFF_MM, &dx, &dy);
        if (!Motion_SendXY(dx, dy, CFG_HOME_SPEED_MM_S, &time_ms)) {
            return 0u;
        }
        if (!Motion_WaitLevel(axis, 0u, time_ms + CFG_HOME_WAIT_EXTRA_MS)) {
            (void)Motion_StopXY();
            continue;
        }
        (void)Motion_StopXY();

        /* 3) 慢速同方向再压：触发沿才是重复性最好的原点定义 */
        Motion_AxisDelta(axis, CFG_BACKOFF_MM * 2.0f + 5.0f, &dx, &dy);
        if (!Motion_SendXY(dx, dy, CFG_HOME_SLOW_MM_S, &time_ms)) {
            return 0u;
        }
        if (!Motion_WaitLevel(axis, 1u, time_ms + CFG_HOME_WAIT_EXTRA_MS)) {
            (void)Motion_StopXY();
            continue;
        }
        (void)Motion_StopXY();
        return 1u;
    }
    return 0u;
}

uint8_t Motion_HomeXY(void) {
    s_xy_trusted = 0u;
    s_homing_xy  = 1u; /* 独占两轴 */

    uint8_t ok_x = Motion_HomeAxis(0u);
    uint8_t ok_y = Motion_HomeAxis(1u);

    /* 顺序不能变：先清 homing 自己压限位留下的事件、再解除让位、最后提交坐标与可信标志，
       否则守护任务会把这次正常触发当成异常撞限位 */
    (void)Bsp_Gpio_TakeEvents();
    s_homing_xy = 0u;

    /* ★ 触发点在工作坐标系里 = (CFG_HOME_X_MAX_MM, CFG_HOME_Y_MAX_MM)（开关装在最大
       行程端，09-28 台架实锤；此前误置 0,0 会把整个工作区映射到限位之外） */
    s_x          = CFG_HOME_X_MAX_MM;
    s_y          = CFG_HOME_Y_MAX_MM;
    s_xy_trusted = (ok_x && ok_y) ? 1u : 0u;

    if (!s_xy_trusted) {
        return 0u;
    }

    /* 停在触发点上等于一直保持触发，之后任何抖动都会被当成撞限位 */
    if (!Motion_MoveXYBlocking(CFG_PARK_X_MM, CFG_PARK_Y_MM, CFG_HOME_SPEED_MM_S)) {
        s_xy_trusted = 0u; /* 连退到待机点都没成 = 位置未知 */
        return 0u;
    }
    return 1u;
}

/* 转盘按电机轴口径相对走 pulses 个脉冲 */
static uint8_t Motion_TrayMovePulses(Zdt_DIR_e dir, uint32_t pulses, uint16_t rpm) {
    return Zdt_MoveRelative(ZDT_ID_TRAY, dir, rpm, CFG_ZDT_ACCEL, pulses);
}

#define TRAY_DIR_FWD  ((Zdt_DIR_e)(uint8_t)CFG_TRAY_DIR_FORWARD)
#define TRAY_DIR_BACK ((Zdt_DIR_e)(uint8_t)(1u - (uint8_t)CFG_TRAY_DIR_FORWARD))

static uint8_t Motion_WaitTrayLevel(uint8_t want_active, uint32_t timeout_ms) {
    uint32_t t0 = HAL_GetTick();

    while ((HAL_GetTick() - t0) < timeout_ms) {
        if (Bsp_Gpio_TrayOrigin() == want_active) {
            return 1u;
        }
        vTaskDelay(pdMS_TO_TICKS(2u));
    }
    return 0u;
}

uint8_t Motion_TrayHome(void) {
    s_tray_trusted = 0u;
    s_tray_moving  = 0u;

    /* 可能是在一笔定位中途来重新回零的：先停稳 */
    (void)Zdt_Stop(ZDT_ID_TRAY);
    vTaskDelay(pdMS_TO_TICKS(50u));

    for (uint8_t attempt = 0u; attempt < 2u; attempt++) {
        /* 0) 凸块可能正压在开关上：先前进退出窗口，才取得到干净的上升沿 */
        if (Bsp_Gpio_TrayOrigin()) {
            if (!Motion_TrayMovePulses(TRAY_DIR_FWD, CFG_TRAY_BACKOFF_PULSES * 2u,
                                       CFG_TRAY_APPROACH_RPM)) {
                return 0u;
            }
            if (!Motion_WaitTrayLevel(0u, CFG_TRAY_APPROACH_TIMEOUT_MS)) {
                (void)Zdt_Stop(ZDT_ID_TRAY);
                continue;
            }
            (void)Zdt_Stop(ZDT_ID_TRAY);
        }

        /* 1) 粗寻标：超时必须覆盖一整圈 */
        if (!Zdt_SpeedMode(ZDT_ID_TRAY, (Zdt_DIR_e)CFG_TRAY_DIR_FORWARD, CFG_TRAY_SEEK_RPM,
                           CFG_ZDT_ACCEL)) {
            return 0u;
        }
        if (!Motion_WaitTrayLevel(1u, CFG_TRAY_HOME_TIMEOUT_MS)) {
            (void)Zdt_Stop(ZDT_ID_TRAY);
            continue;
        }
        (void)Zdt_Stop(ZDT_ID_TRAY);
        vTaskDelay(pdMS_TO_TICKS(50u));

        /* 2) 必须反向脱开：旋转轴只有一条沿，继续正转会把它甩到开关身后，
           慢速再压就得再等一整圈（远超 APPROACH 超时） */
        if (!Motion_TrayMovePulses(TRAY_DIR_BACK, CFG_TRAY_BACKOFF_PULSES, CFG_TRAY_APPROACH_RPM)) {
            return 0u;
        }
        if (!Motion_WaitTrayLevel(0u, CFG_TRAY_APPROACH_TIMEOUT_MS)) {
            (void)Zdt_Stop(ZDT_ID_TRAY);
            continue;
        }
        (void)Zdt_Stop(ZDT_ID_TRAY);

        /* 3) 慢速同方向再压：这条沿就是索引零位（与后续定位同方向，背隙已吃进） */
        if (!Zdt_SpeedMode(ZDT_ID_TRAY, (Zdt_DIR_e)CFG_TRAY_DIR_FORWARD, CFG_TRAY_APPROACH_RPM,
                           CFG_ZDT_ACCEL)) {
            return 0u;
        }
        if (!Motion_WaitTrayLevel(1u, CFG_TRAY_APPROACH_TIMEOUT_MS)) {
            (void)Zdt_Stop(ZDT_ID_TRAY);
            continue;
        }
        (void)Zdt_Stop(ZDT_ID_TRAY);
        vTaskDelay(pdMS_TO_TICKS(50u));

        /* 4) 以此为累计脉冲与索引计数的共同基准 */
        s_tray_pulses  = 0u;
        s_bin          = (uint8_t)CFG_TRAY_ZERO_BIN;
        s_tray_trusted = 1u;
        return 1u;
    }

    s_tray_trusted = 0u;
    return 0u;
}

uint8_t Motion_TrayIsTrusted(void) {
    return s_tray_trusted;
}

uint8_t Motion_TrayBin(void) {
    return s_bin;
}

uint32_t Motion_TrayGotoBin(uint8_t bin) {
    if (!s_tray_trusted || bin >= CFG_TRAY_BINS) {
        return 0u;
    }

    /* 单方向接近消背隙（跨零位自然多转一圈）。
       前提标定：CFG_TRAY_DIR_FORWARD 必须使"正转 -> 盒号递增"，反了会整圈镜像放错 */
    uint8_t steps = (uint8_t)((bin + CFG_TRAY_BINS - s_bin) % CFG_TRAY_BINS);

    if (steps == 0u) {
        s_tray_moving = 0u;
        return 0u; /* 已对位 */
    }

    uint32_t pulses = (uint32_t)steps * CFG_TRAY_PULSES_PER_BIN;

    s_tray_pulses_mark = s_tray_pulses;
    s_tray_index_mark  = Bsp_Gpio_TrayIndexCount();
    s_tray_start_tick  = HAL_GetTick();

    if (!Motion_TrayMovePulses(TRAY_DIR_FWD, pulses, CFG_TRAY_SPEED_RPM)) {
        s_tray_trusted = 0u; /* 指令没发出去 = 转盘根本不会动 */
        return 0u;
    }

    s_tray_pulses += pulses;
    s_tray_pending_bin = bin;
    s_tray_moving      = 1u;

    return Kinematics_TrayMoveTimeMs(pulses, (float)CFG_TRAY_SPEED_RPM);
}

uint8_t Motion_TrayFinish(uint32_t est_ms) {
    if (!s_tray_moving) {
        return s_tray_trusted;
    }
    s_tray_moving = 0u;

    uint32_t elapsed_ms = HAL_GetTick() - s_tray_start_tick;

    if (est_ms > elapsed_ms) {
        vTaskDelay(pdMS_TO_TICKS(est_ms - elapsed_ms));
    }

    /* 索引复核：本笔理论整圈数必须与实测上升沿数完全一致，对不上 = 减速箱丢步/堵转。
       检出能力上界（仿真实测）：一圈只有一个沿，只有把累计脉冲推过/推离整圈边界的那一笔
       才看得见，故上界是一段约一圈的【行程】；漏检窗口内落点偏差理论上界为半圈 = 3 格。
       本模块是滑移【探测器】不是【补偿器】：检出的靠作废重回零，检不出的靠定期重新回零兜底 */
    uint32_t expect = Kinematics_TrayIndexCrossings(s_tray_pulses_mark, s_tray_pulses);
    uint32_t actual = Bsp_Gpio_TrayIndexCount() - s_tray_index_mark;

    if (expect != actual) {
        s_tray_trusted = 0u;
        return 0u;
    }

    /* 第二条是记账之间的交叉验证：按符号累加的盒号 vs 按累计脉冲折算的盒号 */
    uint8_t pulse_bin =
        (uint8_t)((CFG_TRAY_ZERO_BIN + Kinematics_TrayBinOf(s_tray_pulses)) % CFG_TRAY_BINS);

    if (pulse_bin != s_tray_pending_bin) {
        s_tray_trusted = 0u;
        return 0u;
    }

    s_bin = s_tray_pending_bin;
    return 1u;
}