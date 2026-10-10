/**
 * @file    dbg_app.c
 * @brief   调试动作实现：X/Y 意图 -> 产品解算器 -> 两台电机帧 -> 等够估算时间
 * @note    "检验算法无误"指的就是这里调用 Kinematics_XYSolve 的那条路：
 *          本文件不复制、不改写任何运动学公式，解算与配速全部由产品源码完成，
 *          所以在这里量到的方向/距离/耗时就是对产品算法的直接检验。
 */
#include "dbg_app.h"
#include "dbg_config.h"
#include "dbg_console.h" /* 自动体检要知道有没有收到过命令 */
#include "dbg_gpio.h"
#include "dbg_print.h"
#include "dbg_uart.h"
#include "dbg_vision.h" /* stat 的 [VISION] 行 */
#include "motion_kinematics.h"
#include "servo_out.h"
#include "sort_config.h"
#include "zdt_bus.h"

#define DBG_APP_ACCEL DBG_CFG_ACCEL_DEFAULT

/* 体检/采纳时扫描的地址上限（总线上的驱动器地址都在 1~这个数之间） */
#define AUTOSCAN_ADDR_MAX 8u

/* 上一次运动的证据（算法校验用：估算 vs 实测） */
static uint32_t s_last_est_ms;
static uint32_t s_last_actual_ms;
static int32_t  s_last_pulse_a;
static int32_t  s_last_pulse_b;
static float    s_last_rpm_a;
static float    s_last_rpm_b;

/* 速度字段单位系数：1 = 字段即 RPM（产品口径），10 = 字段是 0.1RPM（物创口径） */
static uint16_t s_speed_scale = DBG_CFG_SPEED_SCALE_DEFAULT;

/* ★ 转盘的同类系数（**与 XY 独立**：57 驱动器与 42 可能不是同一口径，2026-09-30 加）：
   `tspu 10` 会把转盘所有速度字段 ×10，**并且把它的所有等待超时一起 ×10**——
   因为"慢 10 倍"正是 thome 相位1（超时 25s 名义、覆盖一圈）与相位2（1.5s 等脱开）会误报失败的原因 */
static uint16_t s_tray_scale = DBG_CFG_TRAY_SPEED_SCALE_DEFAULT;

/* ★ 转盘速度百分比（`tspd`，默认 100）：给三种标定转速统一乘倍数（台架"再快一倍"= 200） */
static uint16_t s_tray_speed_pct = DBG_CFG_TRAY_SPEED_PCT_DEFAULT;

/* ★ 位置记账（`p` 绝对定位用，2026-09-26 新增）：
   原点 = 归零完成后的驻留位（限位角内退 20mm），X≤0/Y≤0 = 第三象限（dbg_config.h §4c）。
   位置是**开环推算**：归零设定初值，之后每条成功的运动命令按位移累加；
   运动中限位触发（撞了）或手动 stop（半途停）都会作废记账，必须重新归零 */
static float   s_pos_x, s_pos_y;
static uint8_t s_x_homed, s_y_homed; /* 两轴各自归过零；都归完 = 记账可信 */

/* ★ 撞限位判定状态（2026-10-08 起判据 = 电平连续压住，见 Dbg_App_LimitWatch）：
   s_lim_low_since = "连续压住"的起点（0 = 此刻没压着）；s_lim_fired = 本次压住已处置过；
   s_lim_glitch_cnt = 电气毛刺累计（压了不到确认窗就松开 = 不停、不作废，只留证据） */
static uint32_t s_lim_low_since;
static uint8_t  s_lim_fired;
static uint32_t s_lim_glitch_cnt;

static void Dbg_App_LimitWatch(void); /* 前向声明：WaitMsStop（下面）也要每圈调 */

/* ★ 转盘记账（`thome`/`tb` 用，2026-09-30 新增，对齐产品 Motion_TrayHome/GotoBin 的账）：
   s_tray_pulses = 累计电机轴脉冲（回零清零，之后只按成功命令累加）；
   s_bin = 当前对准投放点的盒号；s_tray_trusted = 记账可信（软急停/复核失败都作废）。
   转格后的两条产品同款复核：索引沿计数（检出减速箱丢步/堵转）+ 累计脉冲折算盒号 */
static int32_t  s_tray_pulses; /* 有符号：双向走时为负（与产品同口径） */
static uint8_t  s_tray_bin;
static uint8_t  s_tray_trusted;

/* ★ 类别(1~6) -> 盒号(1~6，人类编号)：盒1 = 回零位对准投放位那格。抽签后改
   dbg_config.h §4d 的 DBG_CFG_CATEGORY_TO_BIN（与产品 sort_task.c 的 s_category_to_bin 同步）。
   内部盒下标仍是 0~5（= 盒号-1），人机接口全部显示 1~6 */
static const uint8_t s_cat_to_bin[CFG_NUM_CATEGORIES] = DBG_CFG_CATEGORY_TO_BIN;

static uint8_t Dbg_App_XyTrusted(void) {
    return (s_x_homed != 0u && s_y_homed != 0u) ? 1u : 0u;
}

/* 浮点打印成 d.d（一位小数；自写格式器没有 %f，负数靠整数部分带符号） */
static void Dbg_App_PrintF1(float v) {
    int32_t t = (int32_t)(v * 10.0f + ((v >= 0.0f) ? 0.5f : -0.5f));

    Dbg_Printf("%d.%d", (int)(t / 10), (int)((t < 0 ? -t : t) % 10));
}

/* ★ 示波器观测变量（dbg_app.h 有用法）：必须非 static 才进符号表、volatile 才不被优化。
   特意用独立标量而不是结构体——Watch/Logic Analyzer 只认单个变量名，成员路径在 LA
   里 cannot evaluate（2026-09-26 实测）。主循环每圈在 Dbg_App_Tick 里刷新 */
volatile uint32_t g_scope_tick = 0u;
volatile uint16_t g_claw_us    = 0u;
volatile uint16_t g_lift_us    = 0u;
volatile uint8_t  g_lim_x      = 0u;
volatile uint8_t  g_lim_y      = 0u;
volatile uint8_t  g_lim_tray   = 0u;
volatile uint32_t g_lim_x_cnt  = 0u;
volatile uint32_t g_lim_y_cnt  = 0u;
volatile uint32_t g_lim_tray_cnt = 0u;
volatile uint32_t g_move_est_ms = 0u;
volatile uint32_t g_move_act_ms = 0u;

#if DBG_CFG_BOOT_AUTOSCAN
static uint32_t s_autoscan_tick; /* 上次体检的时刻（没收到命令时定期重跑） */
#endif

/* 驱动器回包（ZDT 手册「命令返回」；物创 ZDT_drive.c 的 receive_* 表同此）：
     [地址][功能码][02][6B] = 命令已正确接收
     [地址][00][EE][6B]     = 校验错误
     [地址][00][E2][6B]     = 参数错误（转速/加速度/脉冲越界）
   回包通常 2ms 内到（物创那边就是用 2ms 超时收的），这里等 DBG_CFG_ACK_WAIT_MS */
static uint16_t Dbg_App_AckRead(uint8_t motor, uint8_t *buf, uint16_t cap) {
    uint32_t t0 = HAL_GetTick();
    uint16_t n  = 0u;

    while ((HAL_GetTick() - t0) < DBG_CFG_ACK_WAIT_MS) {
        while ((Dbg_Uart_MotorAvailable(motor) > 0u) && (n < cap)) {
            buf[n] = Dbg_Uart_MotorReadByte(motor);
            n++;
        }
        /* 凑齐一条以 6B 结尾的完整应答就提前返回 */
        if ((n >= 4u) && (buf[n - 1u] == 0x6Bu)) {
            break;
        }
    }

    /* 探测窗口结束就把该线接收关掉：它一直开着只会拾噪，把主循环饿死（见 dbg_uart.c） */
    Dbg_Uart_MotorDisarm(motor);
    return n;
}

/* 在回包里找形如 [x][code][6B] 的应答（容忍前面多一个杂字节） */
static uint8_t Dbg_App_AckHas(const uint8_t *buf, uint16_t n, uint8_t code) {
    for (uint16_t i = 0u; (uint16_t)(i + 2u) < n; i++) {
        if ((buf[i + 1u] == code) && (buf[i + 2u] == 0x6Bu)) {
            return 1u;
        }
    }
    return 0u;
}

/* 返回 1 = 收到 02 正确接收应答（调用方可据此拦截"帧没到驱动器"的运动）。
   一条线一台驱动器，回包不再互撞——广播帧的应答也能读到（历史遗留的"撞车"说明作废） */
static uint8_t Dbg_App_AckReport(uint8_t motor, const char *what, uint8_t addr) {
    uint8_t  buf[16];
    uint16_t n = Dbg_App_AckRead(motor, buf, sizeof(buf));

    Dbg_Printf("[ACK] %s线 %s 地址%u 回包 %u 字节:", Zdt_Bus_NameOf((Dbg_Motor_e)motor), what,
               (unsigned)addr, (unsigned)n);
    for (uint16_t i = 0u; i < n; i++) {
        Dbg_Printf(" %02x", (unsigned)buf[i]);
    }
    Dbg_Printf("\n");

    if (Dbg_App_AckHas(buf, n, 0x02u) != 0u) {
        Dbg_Printf("[ACK] ✓ 驱动器应答 02 = 命令已正确接收 → 帧确实到了驱动器里，\n");
        Dbg_Printf("[ACK]   所以问题不在 MCU 这一侧（不是接线/波特率/引脚），往驱动器参数想\n");
        return 1u;
    } else if (Dbg_App_AckHas(buf, n, 0xEEu) != 0u) {
        Dbg_Printf("[ACK] ✗ 驱动器回校验错误 EE：帧长或校验字节不对\n");
    } else if (Dbg_App_AckHas(buf, n, 0xE2u) != 0u) {
        Dbg_Printf("[ACK] ✗ 驱动器回参数错误 E2：参数越界（转速/加速度/脉冲）\n");
    } else if (n == 0u) {
        Dbg_Printf("[ACK] 没有回包 → 这一帧没有任何驱动器应答，按这个顺序查：\n");
        Dbg_Printf("[ACK]   ① 驱动器供电（电源灯亮不亮、量 V+ 到 GND）\n");
        Dbg_Printf("[ACK]   ② %s 线 TX 是否真的接到驱动器 RX、以及两边是否共地\n",
                   Zdt_Bus_NameOf((Dbg_Motor_e)motor));
        Dbg_Printf("[ACK]   ③ 波特率这一轮是 %u → `baud 9600` / 57600 换档再试\n",
                   (unsigned)Dbg_Uart_MotorBaud(motor));
        Dbg_Printf("[ACK]   ④ 驱动器的地址到底是不是 %u（`scan` 逐个试）\n", (unsigned)addr);
        Dbg_Printf("[ACK]   ⑤ 驱动器控制模式是否被设成了脉冲/方向（那样它就完全不收串口）\n");
        Dbg_Printf("[ACK]   注：个别固件把应答功能关掉了也会无回包 —— 那就只能靠 `en 0` 手拧判断\n");
    } else {
        Dbg_Printf("[ACK] 收到字节但不认识：可能波特率不对，或线上有干扰\n");
    }
    return 0u;
}

/* 探测某条线上的某地址（只有使能，没有运动指令，电机不会走） */
void Dbg_App_AskAck(uint8_t motor, uint8_t addr) {
    Dbg_Printf("[ACK] 探测 %s线 地址 %u（用使能帧，没有运动指令，电机不会走）\n",
               Zdt_Bus_NameOf((Dbg_Motor_e)motor), (unsigned)addr);
    Dbg_Uart_MotorFlush(motor); /* 只看这一帧的回包，别混进残留字节 */
    if (Zdt_Bus_EnableOn(motor, addr, 1u) == 0u) {
        Dbg_Printf("[FAIL] 使能帧没发出去\n");
        return;
    }
    Dbg_App_AckReport(motor, "使能", addr);
}

uint8_t Dbg_App_SendRaw(uint8_t motor, const uint8_t *bytes, uint16_t len) {
    if ((bytes == NULL) || (len == 0u) || (len > 32u)) {
        Dbg_Printf("[FAIL] 字节数必须在 1~32 之间\n");
        return 0u;
    }

    Dbg_Printf("[SEND] 发到 %s线，%u 字节:", Zdt_Bus_NameOf((Dbg_Motor_e)motor), (unsigned)len);
    for (uint16_t i = 0u; i < len; i++) {
        Dbg_Printf(" %02x", (unsigned)bytes[i]);
    }
    Dbg_Printf("\n");

    Dbg_Uart_MotorFlush(motor);
    if (Dbg_Uart_SendMotor(motor, bytes, len) == 0u) {
        Dbg_Printf("[FAIL] 没发出去（该线忙或句柄未初始化）\n");
        return 0u;
    }
    Dbg_App_AckReport(motor, "任意帧", (len >= 1u) ? bytes[0] : 0u);
    return 1u;
}

void Dbg_App_SetSpeedScale(uint16_t scale) {
    if (scale == 0u) {
        scale = 1u;
    }
    if (scale > 100u) {
        scale = 100u;
    }
    s_speed_scale = scale;

    Dbg_Printf("[OK] 速度字段系数 -> %u（帧内数值 = 名义转速 × %u）\n", (unsigned)scale,
               (unsigned)scale);
    if (scale == 1u) {
        Dbg_Printf("[INFO] 口径 = 产品（字段即 RPM）。实测比估算慢约 10 倍就改 `spu 10`\n");
    } else {
        Dbg_Printf("[INFO] 口径 = 物创（字段是 0.1RPM）。产品 code/ 按 RPM 算 —— 两边差 10 倍\n");
        Dbg_Printf("[INFO] 这条要写进待办：产品的速度与时间估算可能整体差 10 倍\n");
    }
}

uint16_t Dbg_App_SpeedScale(void) {
    return s_speed_scale;
}

/* 转盘速度系数（tspu 命令；1 = 字段即 RPM，10 = 0.1RPM——只影响转盘，XY 用 spu） */
void Dbg_App_SetTraySpeedScale(uint16_t scale) {
    if (scale == 0u) {
        scale = 1u;
    }
    if (scale > 100u) {
        scale = 100u;
    }
    s_tray_scale = scale;
    Dbg_Printf("[OK] 转盘速度系数 -> %u（转盘所有速度字段 ×%u；等待超时也一起 ×%u）\n",
               (unsigned)scale, (unsigned)scale, (unsigned)scale);
    if (scale == 1u) {
        Dbg_Printf("[INFO] 口径 = 字段即 RPM（产品口径）\n");
    } else {
        Dbg_Printf("[INFO] 口径 = 字段是 0.1RPM（物创口径）——若秒表复核为真，这条要写进产品待办\n");
    }
}

uint16_t Dbg_App_TraySpeedScale(void) {
    return s_tray_scale;
}

/* 转盘速度百分比（tspd）：100 = 产品标定的三种转速；台架试"再快一倍"用 200 */
void Dbg_App_SetTraySpeedPct(uint16_t pct) {
    if (pct < 20u) {
        pct = 20u;
    }
    if (pct > 400u) {
        pct = 400u;
    }
    s_tray_speed_pct = pct;
Dbg_Printf("[OK] 转盘速度 -> 标定值的 %u%%（粗寻标 %u / 对格 %u / 慢压 %u rpm@电机轴；"
               "定稿后写回产品 CFG_TRAY_*_RPM）\n",
               (unsigned)pct,
               (unsigned)((uint32_t)CFG_TRAY_SEEK_RPM * pct / 100u),
               (unsigned)((uint32_t)CFG_TRAY_SPEED_RPM * pct / 100u),
               (unsigned)((uint32_t)CFG_TRAY_APPROACH_RPM * pct / 100u));
}

uint16_t Dbg_App_TraySpeedPct(void) {
    return s_tray_speed_pct;
}

/* ---------- 时间接缝：只用 HAL_GetTick 忙等（本工程没有 RTOS） ---------- */
static void Dbg_App_WaitMs(uint32_t ms) {
    uint32_t t0 = HAL_GetTick();

    while ((HAL_GetTick() - t0) < ms) {
    }
}

/* ★ 可打断等待：期间敲任何控制台字节 → 返回 1，调用方必须 StopBoth/中止当前动作。
   背景（2026-09-28 用户实测）：jog/p/pick 的运动等待原来是纯忙等，期间敲的 stop
   全部堆在接收缓冲里、运动走完才被执行——感知就是"stop 没效果/所有回复到位后才来"。
   归零路径早有字节监听（WaitMoveWatch），普通运动路径缺这一块，此函数补齐 */
static uint8_t Dbg_App_WaitMsStop(uint32_t ms) {
    uint32_t t0 = HAL_GetTick();

    while ((HAL_GetTick() - t0) < ms) {
        Dbg_App_LimitWatch(); /* 等运动的这段时间也是"守护任务"：撞限位判据要盯着电平 */
        if (Dbg_Uart_Available() > 0u) {
            return 1u;
        }
    }
    return 0u;
}

static uint16_t Dbg_App_RpmToU16(float rpm) {
    if (rpm < 1.0f) {
        return 1u;
    }
    if (rpm > 300.0f) {
        return 300u;
    }
    return (uint16_t)rpm;
}

/* 名义转速 -> 帧内速度字段。`j` 不走这里，它要的是原始数值。
   16 字节 V2 协议下速度字段固定 0.1RPM（×10 在构帧里做），spu 系数不参与 */
static uint16_t Dbg_App_SpeedField(float rpm) {
    uint32_t field = (uint32_t)Dbg_App_RpmToU16(rpm) *
                     ((Zdt_Bus_Frame16() != 0u) ? 1u : (uint32_t)s_speed_scale);

    if (field > 65535u) {
        field = 65535u;
    }
    if (field == 0u) {
        field = 1u;
    }
    return (uint16_t)field;
}

void Dbg_App_DumpFrames(uint8_t addr, uint32_t pulse) {
    uint8_t  en[6];
    uint8_t  pos[16];
    uint16_t field   = Dbg_App_SpeedField(30.0f);
    uint16_t en_len  = Zdt_Bus_BuildEnable(addr, 1u, en);
    uint16_t pos_len = Zdt_Bus_BuildPosition(addr, (int32_t)pulse, field, DBG_APP_ACCEL, pos);

    Dbg_Printf("[DUMP] 使能帧 %u 字节（不发）：", (unsigned)en_len);
    for (uint16_t i = 0u; i < en_len; i++) {
        Dbg_Printf(" %02x", (unsigned)en[i]);
    }
    Dbg_Printf("\n");

    Dbg_Printf("[DUMP] 位置帧 %u 字节（不发）：", (unsigned)pos_len);
    for (uint16_t i = 0u; i < pos_len; i++) {
        Dbg_Printf(" %02x", (unsigned)pos[i]);
    }
    Dbg_Printf("\n");

    Dbg_Printf("[DUMP] 地址=%u 功能码=F3/FD 方向=0(按产品口径正=CW) 速度字段=%u 加速=%u 脉冲=%u",
               (unsigned)addr, (unsigned)field, (unsigned)DBG_APP_ACCEL, (unsigned)pulse);
    Dbg_Printf(" 模式=相对\n");
    Dbg_Printf("[DUMP] 结尾必须是 6b；位置帧 %u 字节（13=老协议 / 16=V2 协议，fmt 命令切换）\n",
               (unsigned)pos_len);

    /* 直接给出可粘贴的复现命令：`send` 发的是原始字节，不经任何换算 */
    Dbg_Printf("[DUMP] 复现使能帧：send");
    for (uint16_t i = 0u; i < en_len; i++) {
        Dbg_Printf(" %02x", (unsigned)en[i]);
    }
    Dbg_Printf("\n");
    Dbg_Printf("[DUMP] 复现位置帧：send");
    for (uint16_t i = 0u; i < pos_len; i++) {
        Dbg_Printf(" %02x", (unsigned)pos[i]);
    }
    Dbg_Printf("\n");
}

/* TX/RX 回环自检：两条线各测一遍（短接哪条线就只看那一线的结果） */
void Dbg_App_Loopback(uint16_t repeat) {
    uint8_t  frame[16];
    uint16_t len;

    if (repeat == 0u) {
        repeat = 3u;
    }
    if (repeat > 50u) {
        repeat = 50u;
    }

    len = Zdt_Bus_BuildPosition(DBG_CFG_ADDR_BROADCAST, 3200, 30u, DBG_APP_ACCEL, frame);
    if (len == 0u) {
        Dbg_Printf("[FAIL] 构帧失败\n");
        return;
    }

    Dbg_Printf("[LOOP] 把某条线的 TX 与 RX 短接（建议同时拔掉该线驱动器），看对应线的结果\n");
    for (uint8_t motor = 0u; motor < (uint8_t)DBG_MOTOR_COUNT; motor++) {
        uint16_t got = 0u;

        Dbg_Uart_MotorFlush(motor); /* 先把旧字节丢掉，只看这一轮的 */
        Dbg_Printf("[LOOP] %s线：发送 %u 次广播位置帧（%u 字节/帧，波特率 %u）...\n",
                   Zdt_Bus_NameOf((Dbg_Motor_e)motor), (unsigned)repeat, (unsigned)len,
                   (unsigned)Dbg_Uart_MotorBaud(motor));
        for (uint16_t i = 0u; i < repeat; i++) {
            (void)Dbg_Uart_SendMotor(motor, frame, len);
        }

        Dbg_App_WaitMs(100u); /* 等最后一个字节收完（13 字节 @115200 ≈ 1.2ms，给足余量） */

        Dbg_Printf("[LOOP] %s线收到 %u 字节：", Zdt_Bus_NameOf((Dbg_Motor_e)motor),
                   (unsigned)Dbg_Uart_MotorAvailable(motor));
        while (Dbg_Uart_MotorAvailable(motor) > 0u) {
            Dbg_Printf(" %02x", (unsigned)Dbg_Uart_MotorReadByte(motor));
            got++;
            if ((got % 16u) == 0u) {
                Dbg_Printf("\n");
            }
        }
        Dbg_Printf("\n");

        if (got == 0u) {
            Dbg_Printf("[LOOP] %s线一个字节都没回来 → **该线发送链路有问题**（TX/RX 短接了吗？"
                       "波特率 %u，换档 `baud`）\n",
                       Zdt_Bus_NameOf((Dbg_Motor_e)motor), (unsigned)Dbg_Uart_MotorBaud(motor));
        } else {
            Dbg_Printf("[LOOP] %s线 ✓ 收到自己发的字节 → MCU 该线发送链路（时钟/引脚/MSP/波特率）没问题\n",
                       Zdt_Bus_NameOf((Dbg_Motor_e)motor));
        }
    }
    Dbg_Printf("[LOOP] 收不到的那条线，问题在驱动器侧：供电、TX/RX 交叉、共地、地址、"
               "控制模式或固件版本\n");
}

uint8_t Dbg_App_SetBusBaud(uint32_t bps) {
    uint8_t ok = 1u;

    for (uint8_t motor = 0u; motor < (uint8_t)DBG_MOTOR_COUNT; motor++) {
        if (Dbg_Uart_SetMotorBaud(motor, bps) == 0u) {
            ok = 0u;
        }
    }
    if (ok == 0u) {
        Dbg_Printf("[FAIL] 不支持的波特率（ZDT 常用：9600/19200/38400/57600/115200/230400）\n");
        return 0u;
    }

    Dbg_Printf("[OK] 两条电机线波特率 -> %u\n", (unsigned)bps);
    Dbg_Printf("[INFO] 换完立刻试 `en 0` 然后手拧电机：抱死就说明这一档是对的\n");
    return 1u;
}

/* 上电把"先做什么/怎么看"打发出去：台架上不用翻文档 */
static void Dbg_App_PrintHints(void) {
    Dbg_Printf("[HINT] 先看上电自检那几行 [ACK]：有回包（含 02）= 链路通，电机不动就是驱动器参数/机械\n");
    Dbg_Printf("[HINT]   没回包 → ① 驱动器电源灯 ② `ack a`/`ack b` ③ `en 0` 后手拧两台（抱死=收到了帧）\n");
    Dbg_Printf("[HINT]   ④ `dump` 核帧字节 → `loop` 回环自检（短接该线 TX/RX）→ `baud 9600` 换档 → `scan` 找真 ID\n");
    Dbg_Printf("[HINT] 能动之后：`j a cw 30 3200 255`（正好一圈，秒表掐时间）→ `bx 50` 试 X → `x 50` / `y 50`\n");
    Dbg_Printf("[HINT] A/B 谁在左谁在右由装机决定：`x 30` 方向反了就改产品 CFG_AXIS_A/B_SIGN（两轴符号表见 README §4）\n");
    Dbg_Printf("[HINT] 转盘：`j t cw 60 3200 255` 先手动转一圈（=转盘18°）→ `thome` 回零 → `tb 1` 对格复核\n");
    Dbg_Printf("[HINT] 速度单位两条口径：`spu 1` 字段即 RPM（产品口径，默认）；`spu 10` 字段是 0.1RPM（物创口径）\n");
    Dbg_Printf("[HINT] 屏/视觉：`scrtest` 验屏（n1~n6 亮 1..6）→ `scr n1 5` 单写一行；"
               "`k230 live` 看视觉帧流，`k230 sim 3 60 45` 不接线先验解析与换算\n");
    Dbg_Printf("[HINT] 单件演练（对齐产品读帧纪律）：`read`（到投放点取目标，给 pick 坐标）"
               "→ `pick X Y 100 <形状>` → `tb <类别>` → `drop`\n");
    Dbg_Printf("[HINT] 想手动盘车：`dis 0`（广播失能，电机松力）；任意时刻 `stop` 停双轴；`help` 看全部命令\n");
}

/* 上电自检（2026-09-30 全广播定调版）：三条线各发一帧广播使能，把电机拉进使能态
   （产品上电也是广播使能三台）。★ 本批驱动器对广播**不回包**（ID 非 1 时广播无应答
   是实测现象），所以这里**不读应答**——没有 [ACK] 行不是故障。要单播诊断用 `ack`/`scan` */
static void Dbg_App_BusPresence(void) {
    for (uint8_t motor = 0u; motor < (uint8_t)DBG_MOTOR_COUNT; motor++) {
        (void)Zdt_Bus_EnableOn(motor, DBG_CFG_ADDR_BROADCAST, 1u);
        Dbg_App_WaitMs(5);
    }
    Dbg_Printf("[BOOT] 寻址 = 全广播 0x00（一条线一台，与产品同口径）；三条线已广播使能"
               "（本批驱动器对广播不回包，无 [ACK] 属正常）\n");
    Dbg_Printf("[BOOT] 想验驱动器会不会说话：`scan`（单播逐 ID 探测，对自己的 ID 会回 02）；"
               "运动流程不依赖应答。手动盘车先 `dis 0`\n");
}

void Dbg_App_Init(void) {
    Dbg_Printf("[BOOT] 固件版本戳 %s（stat 里也有这行；有 = 新版在板上）\n",
               DBG_CFG_FW_STAMP);
    Dbg_App_PrintConfig();
    Dbg_App_PrintHints();
    Dbg_App_BusPresence();
#if DBG_CFG_BOOT_AUTOSCAN
    Dbg_App_BusAutoScan();
    s_autoscan_tick = HAL_GetTick();
#endif
}

/* ★ 上电自动归零（1001b 起，`DBG_CFG_AUTO_HOME_ON_BOOT`）：横幅/帮助打完后等
   DBG_CFG_AUTO_HOME_SKIP_MS，期间控制台来**任意字节 = 本次跳过**（字节不丢，留在缓冲里
   照常被命令台执行，比如敲 stop/help）；窗口内安静 = 自动跑全回零 `home`。
   这是产品"上电即启动、首圈 Sort_EnsureCalibrated 自动归零"的台架同款——从此上电 =
   归零 = `p`/`pick`/`tb` 直接可用，不再需要先敲三条命令。
   烧录/换线等不想让电机自己动的情况：上电后窗口内随便发个字节（比如一个回车） */
void Dbg_App_AutoHomeIfEnabled(void) {
#if DBG_CFG_AUTO_HOME_ON_BOOT
    uint32_t t0 = HAL_GetTick();

    Dbg_Printf("[HOME] %ums 内敲任意键 = 跳过自动归零；否则自动执行 `home`"
               "（homex→homey→thome，最坏约半分钟）\n",
               (unsigned)DBG_CFG_AUTO_HOME_SKIP_MS);
    while ((HAL_GetTick() - t0) < DBG_CFG_AUTO_HOME_SKIP_MS) {
        if (Dbg_Uart_Available() > 0u) {
            Dbg_Printf("[HOME] 自动归零已跳过（本次上电不运动；想归零随时敲 `home`）\n");
            return;
        }
    }
    (void)Dbg_App_HomeAll();
#endif
}

/* 主循环每圈调用。作用：**在还没收到过任何控制台输入**时，每 10 秒重跑一次总线体检。
   为什么：上电那一刻没人在收串口时，报告就白打了；重跑之后"任意时刻打开串口都能看到\n一份完整报告"，于是"PC→板子这一路不通"也不再挡住定位。收到第一个字节就永久停掉 */
void Dbg_App_Tick(void) {
    /* 示波器观测变量每圈刷新（dbg_app.h）：Keil Watch / Logic Analyzer 直接按名字看，
       g_scope_tick 一直爬 = 固件活着（放最前，不受下面 autoscan 提前返回影响） */
    Dbg_Gpio_Poll(); /* 先维护"松开确认"，让下面的沿计数保持"按压次数"语义 */
    Dbg_App_LimitWatch(); /* 撞限位判据（电平连续压住）——纯忙等时段靠这里兜住 */
    g_scope_tick++;
    g_claw_us = Servo_Out_CurrentUs(DBG_SERVO_CLAW);
    g_lift_us = Servo_Out_CurrentUs(DBG_SERVO_LIFT);
    g_lim_x        = Dbg_Gpio_Level(DBG_LIMIT_X);
    g_lim_y        = Dbg_Gpio_Level(DBG_LIMIT_Y);
    g_lim_tray     = Dbg_Gpio_Level(DBG_LIMIT_TRAY);
    g_lim_x_cnt    = Dbg_Gpio_EdgeCount(DBG_LIMIT_X);
    g_lim_y_cnt    = Dbg_Gpio_EdgeCount(DBG_LIMIT_Y);
    g_lim_tray_cnt = Dbg_Gpio_EdgeCount(DBG_LIMIT_TRAY);
    g_move_est_ms  = s_last_est_ms;
    g_move_act_ms  = s_last_actual_ms;

#if DBG_CFG_BOOT_AUTOSCAN
    if (Dbg_Console_HasEverReceived() != 0u) {
        return;
    }
    if ((HAL_GetTick() - s_autoscan_tick) >= DBG_CFG_AUTOSCAN_REPEAT_MS) {
        s_autoscan_tick = HAL_GetTick();
        Dbg_App_BusAutoScan();
    }
#endif
}

/* ★ 上电自动体检：6 档波特率 × 地址 1~8，**两条线各自**扫，只打"有回包"的组合。
   为什么要有它：PC→板子（控制台接收）一旦不通，人就敲不进 `scan`/`ack`；
   而板子→PC 通常还活着 —— 让板子自己把结论打出来，照样能定位。
   一条线一台驱动器，应答不撞车，扫出来的就是干净结果 */
static const uint32_t s_autoscan_baud[] = {9600u, 19200u, 38400u, 57600u, 115200u, 230400u};
void Dbg_App_BusAutoScan(void) {
    uint8_t  n_baud = (uint8_t)(sizeof(s_autoscan_baud) / sizeof(s_autoscan_baud[0]));
    uint32_t orig[DBG_MOTOR_COUNT];
    uint8_t  hits = 0u;

    for (uint8_t motor = 0u; motor < (uint8_t)DBG_MOTOR_COUNT; motor++) {
        orig[motor] = Dbg_Uart_MotorBaud(motor);
    }

    Dbg_Printf("[SCAN2] 上电体检：%u 档波特率 × 地址 1~%u × A/B 两条线（只列有回包的组合，约 4 秒）\n",
               (unsigned)n_baud, (unsigned)AUTOSCAN_ADDR_MAX);

    for (uint8_t b = 0u; b < n_baud; b++) {
        for (uint8_t motor = 0u; motor < (uint8_t)DBG_MOTOR_COUNT; motor++) {
            if (s_autoscan_baud[b] != orig[motor]) {
                if (Dbg_Uart_SetMotorBaud(motor, s_autoscan_baud[b]) == 0u) {
                    continue;
                }
            }

            for (uint8_t addr = 1u; addr <= AUTOSCAN_ADDR_MAX; addr++) {
                uint8_t  buf[8];
                uint16_t n;

                Dbg_Uart_MotorFlush(motor);
                if (Zdt_Bus_EnableOn(motor, addr, 1u) == 0u) {
                    continue;
                }
                n = Dbg_App_AckRead(motor, buf, sizeof(buf));
                if (n == 0u) {
                    continue;
                }

                hits++;
                Dbg_Printf("[SCAN2] ✓ %s线 波特率 %u 地址 %u 回包 %u 字节:",
                           Zdt_Bus_NameOf((Dbg_Motor_e)motor), (unsigned)s_autoscan_baud[b],
                           (unsigned)addr, (unsigned)n);
                for (uint16_t i = 0u; i < n; i++) {
                    Dbg_Printf(" %02x", (unsigned)buf[i]);
                }
                Dbg_Printf("\n");
            }
        }
    }

    for (uint8_t motor = 0u; motor < (uint8_t)DBG_MOTOR_COUNT; motor++) {
        if (Dbg_Uart_MotorBaud(motor) != orig[motor]) {
            (void)Dbg_Uart_SetMotorBaud(motor, orig[motor]);
        }
    }
    Dbg_Printf("[SCAN2] 波特率已恢复默认\n");

    /* 顺手量一下控制台 RX 引脚（PA1）：控制台"只出不进"时，这一行能分清是线还是 MCU */
    Dbg_Uart_ProbeConsoleRx();

    if (hits == 0u) {
        Dbg_Printf("[SCAN2] 6 档 × 8 地址 × 2 线全部无回包 → 两种可能：\n");
        Dbg_Printf("[SCAN2]   ① 驱动器把应答关了（那就只能看电机转不转来判断）\n");
        Dbg_Printf("[SCAN2]   ② 驱动器没接在该线上（数据线松/没插到底）或控制模式不是串口\n");
    } else {
        Dbg_Printf("[SCAN2] 体检结束：%u 个组合有回包 —— 上面每行 ✓ 就是真实地址与波特率\n",
                   (unsigned)hits);
    }
}

void Dbg_App_PrintConfig(void) {
    Dbg_Printf("[CFG] 板：产品板（PCB 修复版），接线与产品工程一致（2026-09-30 同步）\n");
    Dbg_Printf("[CFG] 电机 A = USART3 PC10/PC11（M1 座）   电机 B = UART5 PC12/PD2（M2 座）\n");
    Dbg_Printf("[CFG] 转盘 T = USART1 PB6/PB7（M3 座）  1:20 减速  每格 %u 脉冲\n",
               (unsigned)CFG_TRAY_PULSES_PER_BIN);
    Dbg_Printf("[CFG] 寻址 = 三线全广播 0x00（一条线一台，产品同口径）；运动只发不收"
               "（本批驱动器广播不回包）\n");
    Dbg_Printf("[CFG] 电机线波特率 %u\n", (unsigned)DBG_CFG_MOTOR_BAUD);
    Dbg_Printf("[CFG] 控制台   UART4 PA0/PA1 @%u（产品板 CN1 预留口；TX↔RX 交叉、共地）\n",
               (unsigned)DBG_CFG_CONSOLE_BAUD);
    Dbg_Printf("[CFG] 屏线     USART6 PC6/PC7 @%u（CN2，只发：`scr`/`scrtest`）"
               "  视觉线 USART2 PD5/PD6 @%u（CN3，只收：`k230`）\n",
               (unsigned)DBG_CFG_SCREEN_BAUD, (unsigned)DBG_CFG_VISION_BAUD);
    Dbg_Printf("[CFG] 视觉换算（产品同款）：机械X=ORIGIN_X+(%d)×帧.y  机械Y=ORIGIN_Y+(%d)×帧.x"
               "  原点=(%d,%d)%s\n",
               (int)CFG_VISION_SIGN_VY_TO_X, (int)CFG_VISION_SIGN_VX_TO_Y,
               (int)CFG_VISION_ORIGIN_X_MM, (int)CFG_VISION_ORIGIN_Y_MM,
               ((CFG_VISION_ORIGIN_X_MM == 0.0f) && (CFG_VISION_ORIGIN_Y_MM == 0.0f)) ? "（未标定）" : "");
    Dbg_Printf("[CFG] 舵机 TIM4 50Hz：夹爪 H1/PB9(CH4) 升降 H4/PB8(CH3)——与产品 tim.c 同引脚\n");
    Dbg_Printf("[CFG] 档位 夹爪 开%u/合%u/半%u  升降 抬%u/伸%u (us)\n",
               (unsigned)Servo_Out_PosUs(DBG_SERVO_CLAW, SERVO_POS_0),
               (unsigned)Servo_Out_PosUs(DBG_SERVO_CLAW, SERVO_POS_1),
               (unsigned)Servo_Out_PosUs(DBG_SERVO_CLAW, SERVO_POS_2),
               (unsigned)Servo_Out_PosUs(DBG_SERVO_LIFT, SERVO_POS_0),
               (unsigned)Servo_Out_PosUs(DBG_SERVO_LIFT, SERVO_POS_1));
    Dbg_Printf("[CFG] 解算常量（来自 Config/sort_config.h）：%u 脉冲/mm  符号 A=%d B=%d  一圈=%u\n",
               (unsigned)CFG_XY_PULSE_PER_MM, (int)CFG_AXIS_A_SIGN, (int)CFG_AXIS_B_SIGN,
               (unsigned)CFG_PULSE_PER_REV_42);
    Dbg_Printf("[CFG] 保护：单段上限 %dmm  速度上限 %dmm/s\n", (int)DBG_CFG_MAX_MOVE_MM,
               (int)DBG_CFG_MAX_SPEED_MM_S);
    Dbg_Printf("[CFG] 速度系数：XY=%u（`spu`） 转盘=%u（`tspu`） 转盘节拍=%u%%（`tspd`）"
               "（1=字段即RPM，10=0.1RPM；转盘系数同时缩放超时，节拍反缩）\n",
               (unsigned)s_speed_scale, (unsigned)s_tray_scale, (unsigned)s_tray_speed_pct);
    Dbg_Printf("[CFG] 示波器观测变量：g_scope_tick/g_claw_us/g_lift_us/g_lim_x·y·tray(_cnt)/"
               "g_move_est·act_ms（Keil Watch/LA 直接按名字加；地址看 map）\n");
}

void Dbg_App_PrintStatus(void) {
    Dbg_Printf("[STAT] 舵机 夹爪=%u us  升降=%u us\n", (unsigned)Servo_Out_CurrentUs(DBG_SERVO_CLAW),
               (unsigned)Servo_Out_CurrentUs(DBG_SERVO_LIFT));
    Dbg_Printf("[STAT] 限位 X=%u Y=%u TRAY=%u（1=开 0=压下；`lim` 看沿计数）\n",
               (unsigned)Dbg_Gpio_Level(DBG_LIMIT_X), (unsigned)Dbg_Gpio_Level(DBG_LIMIT_Y),
               (unsigned)Dbg_Gpio_Level(DBG_LIMIT_TRAY));
    Dbg_Printf("[STAT] 固件版本戳 %s（对不上 = 板上不是这份固件）\n", DBG_CFG_FW_STAMP);
    Dbg_Printf("[STAT] %u 脉冲/mm  单段上限 %dmm\n", (unsigned)Dbg_App_PulsePerMm(),
               (int)DBG_CFG_MAX_MOVE_MM);
    Dbg_Printf("[STAT] A线 %u bps | B线 %u bps | T线 %u bps（三线全广播 0x00，只发不收）\n",
               (unsigned)Dbg_Uart_MotorBaud(0u), (unsigned)Dbg_Uart_MotorBaud(1u),
               (unsigned)Dbg_Uart_MotorBaud(2u));
    Dbg_Printf("[STAT] 加速字节=%u  XY系数=%u 转盘系数=%u 转盘节拍=%u%%  转盘 %s：盒号%u 累计%u 脉冲\n",
               (unsigned)DBG_APP_ACCEL, (unsigned)s_speed_scale, (unsigned)s_tray_scale,
               (unsigned)s_tray_speed_pct,
               (s_tray_trusted != 0u) ? "可信" : "未回零", (unsigned)s_tray_bin,
               (unsigned)s_tray_pulses);
    Dbg_Printf("[STAT] 上次运动 A=%d脉冲@%urpm B=%d脉冲@%urpm 估算%ums 实测%ums\n",
               (int)s_last_pulse_a, (unsigned)Dbg_App_RpmToU16(s_last_rpm_a), (int)s_last_pulse_b,
               (unsigned)Dbg_App_RpmToU16(s_last_rpm_b), (unsigned)s_last_est_ms,
               (unsigned)s_last_actual_ms);
    Dbg_Printf("[STAT] 示波器 tick=%u（g_scope_tick；Watch/LA 按名加，变量清单见 cfg）\n",
               (unsigned)g_scope_tick);
    Dbg_Printf("[STAT] UE 自愈计数：控制台=%u A线=%u B线=%u T线=%u 屏=%u 视觉=%u（应恒为 0；>0 = 串口使能位被外部因素清过）\n",
               (unsigned)Dbg_Uart_UeHealCount(0u), (unsigned)Dbg_Uart_UeHealCount(1u),
               (unsigned)Dbg_Uart_UeHealCount(2u), (unsigned)Dbg_Uart_UeHealCount(3u),
               (unsigned)Dbg_Uart_UeHealCount(4u), (unsigned)Dbg_Uart_UeHealCount(5u));
    Dbg_Vision_PrintStatusLine();
    Dbg_Printf("[STAT] 位置记账 (");
    Dbg_App_PrintF1(s_pos_x);
    Dbg_Printf(", ");
    Dbg_App_PrintF1(s_pos_y);
    Dbg_Printf(")  %s\n", (Dbg_App_XyTrusted() != 0u) ? "可信" : "不可信（先归零）");
}

uint8_t Dbg_App_EnableAddr(uint8_t addr, uint8_t enable) {
    /* 广播（0）与指定数字地址都走"三条线各发一遍"。★ 只发不收（本批驱动器广播不回包）；
       单播数字地址是诊断路径（驱动器对自己的 ID 会回包），要看回包用 `ack <地址>` */
    uint8_t ok = 1u;

    for (uint8_t motor = 0u; motor < (uint8_t)DBG_MOTOR_COUNT; motor++) {
        uint8_t one;

        one = Zdt_Bus_EnableOn(motor, addr, enable);
        Dbg_Printf("[%s] %s线 地址 %u%s %s\n", one ? "OK" : "FAIL",
                   Zdt_Bus_NameOf((Dbg_Motor_e)motor), (unsigned)addr,
                   (addr == DBG_CFG_ADDR_BROADCAST) ? "（广播）" : "（单播诊断）",
                   enable ? "使能" : "失能");
        ok = (ok && one) ? 1u : 0u;
    }

    return ok;
}

/* 单台使能/失能：只发这一条的线（en a / dis b / en t 走这里）；广播、不读应答 */
uint8_t Dbg_App_EnableOne(uint8_t motor, uint8_t enable) {
    uint8_t ok;

    ok = Zdt_Bus_EnableOn(motor, DBG_CFG_ADDR_BROADCAST, enable);
    Dbg_Printf("[%s] %s线 广播 %s（本批驱动器不回包，属正常）\n", ok ? "OK" : "FAIL",
               Zdt_Bus_NameOf((Dbg_Motor_e)motor), enable ? "使能" : "失能");
    return ok;
}

uint8_t Dbg_App_EnableBoth(uint8_t enable) {
    uint8_t ok = 1u;

    for (uint8_t i = 0u; i < (uint8_t)DBG_MOTOR_COUNT; i++) {
        ok = (ok && Dbg_App_EnableOne(i, enable)) ? 1u : 0u;
    }

    return ok;
}

/* 停止路径（2026-09-30 全广播定调版）：三条线各一帧广播停（地址+FE+98+00+6B，帧格式
   已对 Emm_V5.0 手册核过），帧间 5ms 防"背靠背丢帧"（09-25/28 台架教训）。
   ★ 不读应答：本批驱动器广播不回包，产品本来就是"只发不收"——停止帧有效性已由
   0928e 台架验证（stoptest/触发停真停）。E2（堵转保护中）的恢复只能断电或驱动器按键 */
static uint32_t s_stop_done_tick; /* 停止帧发出时刻（软急停延迟测量用） */

uint8_t Dbg_App_StopBoth(void) {
    for (uint8_t motor = 0u; motor < (uint8_t)DBG_MOTOR_COUNT; motor++) {
        (void)Zdt_Bus_StopOn(motor, DBG_CFG_ADDR_BROADCAST);
        Dbg_App_WaitMs(5);
    }
    s_stop_done_tick = HAL_GetTick();

    /* 半途停止 = 实际位置停在命令目标之间，开环记账不再可信 */
    s_x_homed = 0u;
    s_y_homed = 0u;
    s_tray_trusted = 0u;
    return 1u;
}

/* ★ 触发急停（回零内部用，2026-09-30 新增）：A/B 两条**独立**线各一帧广播停，不留间隔
   （5ms 间隔是旧"共用一条总线"时代的防丢帧措施，分线后不需要了）——
   好处有两个：①真急停更快（0.43ms/帧，两台都在 1ms 内收到）②**不动记账**
   （旧写法调 StopBoth 会把 s_x_homed/s_y_homed 清掉，导致 homex 完再 homey 就"记账不可信"，
   `p` 永远用不了——这是 09-30 台架实测暴露的 bug） */
static void Dbg_App_StopXYFast(void) {
    (void)Zdt_Bus_StopOn((uint8_t)DBG_MOTOR_A, DBG_CFG_ADDR_BROADCAST);
    s_stop_done_tick = HAL_GetTick();
    (void)Zdt_Bus_StopOn((uint8_t)DBG_MOTOR_B, DBG_CFG_ADDR_BROADCAST);
}

uint32_t Dbg_App_LastStopDoneTick(void) {
    return s_stop_done_tick;
}

/* ★ 撞限位判定（2026-10-08）：判据 = X/Y 电平**连续压住** ≥ DBG_CFG_LIMIT_CONFIRM_MS
   ——与产品 Motion_ServeEvents **同值同口径**，台架要能预演产品行为。缘由（现场两个现象）：
   ①"抓完之后有时自己回零校准"：EXTI 只认"武装状态下的一次下降沿"，舵机/电机换向的电流
     尖峰耦合进限位线就能造出假沿；旧口径"沿闩锁即作废"每来一次毛刺就白跑一次回零。
   ②"一上来连夹持点都到不了"（拆分口径的硬伤）：停止帧立即发、记账晚 40ms 才作废——
     毛刺被判成假之后记账被**保留**，可运动其实已被停帧截在半路 ⇒ 记账说"已到位"、
     实物停在半路 ⇒ 坐标静默平移（仿真 s23 实测：12 件里 41 次抓空、1 次非法落点）。
   真撞是机械事实：开关**一直被压住**（几百 ms 起，直到人挪开/重新归零）；电气毛刺 µs~ms
   级一晃就回。⇒ 连续压住 ≥ 窗 ⇒ "真撞"：急停 + 记账作废（**必须一起做**：停过的运动
   位置未知，只停不作废必然漂坐标）；窗内已松开 ⇒ "毛刺"：不停、不作废，只打一行 [INFO]。
   调用点：所有可打断等待（WaitMsStop，运动/舵机等待期间每圈）+ 主循环 Dbg_App_Tick
   （覆盖纯忙等时段），与产品"1ms 守护任务全程盯着"等价。
   只在记账可信时判定：没记账就没有"作废"可言，也免得一上电龙门正趴在开关上时刷屏 */
static void Dbg_App_LimitWatch(void) {
    uint32_t now     = HAL_GetTick();
    uint8_t  pressed = ((Dbg_Gpio_Level(DBG_LIMIT_X) == 0u) || (Dbg_Gpio_Level(DBG_LIMIT_Y) == 0u))
                           ? 1u
                           : 0u;

    if (Dbg_App_XyTrusted() == 0u) {
        s_lim_low_since = 0u;
        s_lim_fired     = 0u;
        return;
    }

    if (pressed == 0u) {
        if ((s_lim_fired == 0u) && (s_lim_low_since != 0u)) {
            /* 压了一下又松开、没到确认窗 = 电气毛刺：这就是现场"抓完自己回零"的源头，
               产品同口径会忽略它。打印实测时长，便于判断真有毛刺时该不该动确认窗 */
            s_lim_glitch_cnt++;
            Dbg_Printf("[INFO] 限位毛刺：只压住 %ums 就松开（< %ums 确认窗）——**不停、不作废**"
                       "（产品同口径，本件照常走完）；累计第 %u 次\n",
                       (unsigned)(now - s_lim_low_since), (unsigned)DBG_CFG_LIMIT_CONFIRM_MS,
                       (unsigned)s_lim_glitch_cnt);
        }
        s_lim_low_since = 0u;
        s_lim_fired     = 0u;
    } else if (s_lim_fired == 0u) {
        if (s_lim_low_since == 0u) {
            s_lim_low_since = now;
        } else if ((now - s_lim_low_since) >= DBG_CFG_LIMIT_CONFIRM_MS) {
            uint8_t x_p = (Dbg_Gpio_Level(DBG_LIMIT_X) == 0u) ? 1u : 0u;
            uint8_t y_p = (Dbg_Gpio_Level(DBG_LIMIT_Y) == 0u) ? 1u : 0u;

            s_lim_fired = 1u;
            Dbg_App_StopXYFast(); /* 产品同款：急停 */
            s_x_homed   = 0u;     /* 产品同款：记账作废（两者必须一起，见函数头） */
            s_y_homed   = 0u;
            Dbg_Printf("[WARN] 撞限位（%s%s 被连续压住 ≥%ums）= 真撞：已急停 + **XY 记账作废**"
                       "（停过的运动位置未知）——先看清是不是真压上了开关（`lim` 看电平/计数），"
                       "再 `homex`/`homey`\n",
                       (x_p != 0u) ? "X " : "", (y_p != 0u) ? "Y " : "",
                       (unsigned)DBG_CFG_LIMIT_CONFIRM_MS);
        }
    }
}

/* ---------- 投放点激光检测（2026-10-09；引脚/极性/参数见 dbg_config.h §3.6） ---------- */

/* µs 计时的"每 µs 多少 CPU 周期"：**用 HAL_GetTick 现场自校准**，不读全局 SystemCoreClock。
   台架实测吃过亏：首版用 SystemCoreClock/1e6 当除数，实测打印"宽 0 µs"——那个全局量的
   更新依赖 HAL 版本/调用路径，一不可信就表现为"宽度恒 0"，比不显示更误导。
   自校准用 1ms tick 当尺子（5ms 窗口，命令级可接受），量出来 ≈168 @168MHz */
static uint32_t s_us_div;

static uint8_t Dbg_App_DwtCalibrate(void) {
    uint32_t t0, t1, c0, c1, dtick, dcyc;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    t0 = HAL_GetTick();
    c0 = DWT->CYCCNT;
    while ((HAL_GetTick() - t0) < 5u) {
    }
    t1 = HAL_GetTick();
    c1 = DWT->CYCCNT;

    dtick    = t1 - t0;
    dcyc     = c1 - c0;
    s_us_div = 0u;
    if ((dtick != 0u) && (dcyc != 0u)) {
        s_us_div = dcyc / (dtick * 1000u);
    }
    return (s_us_div != 0u) ? 1u : 0u;
}

static uint32_t Dbg_App_Micros(void) {
    return (s_us_div != 0u) ? (uint32_t)(DWT->CYCCNT / s_us_div) : 0u;
}

/* ★ 落料事件捕获（`laser drop [秒]`）：µs 级快采样，把光路上的每一段"检测到"都量下来
   （段数 + 每段**周期数/µs**）。**本命令就是为用户那句"物块落下的时间很短"写的**
   （2026-10-09 上机实测：一次落料 ≈ **1 个采样点宽（µs 级）**，且没有毛刺干扰）。
   结论怎么用：**段数**才是判据的输入（判据 = "窗口内捕到一次检测"，产品侧用 EXTI 沿）；
   宽度只用于回答"这信号能不能靠电平轮询抓" —— 宽度 ≈0 µs/一个采样点 ⇒ **不能**，必须沿。
   用法两种都行：① 手动：从投放点上方松手丢一块货，让它穿过光路；
                 ② 从爪里：`pick <物块X Y>` 抓起来别松爪 → `drop` → `laser drop`
   裸忙等采样（本工程无 RTOS）；每 1ms 边界照看一次限位与软急停，任意键中止。
   ⚠️ 本函数只用整数打印——自带格式器不支持 %f（踩过：`%.1f` 打出来是 "?1f"） */
void Dbg_App_LaserCapture(uint32_t secs) {
    uint32_t t_end = HAL_GetTick() + secs * 1000u;
    uint32_t guard = HAL_GetTick();
    uint32_t n     = 0u;
    uint8_t  in    = 0u;
    uint32_t start_cyc = 0u;
    uint8_t  dwt_ok = Dbg_App_DwtCalibrate();

    Dbg_Printf("[LASER] 捕获 %us（%s，µs 级采样；宽度计时 %s，1µs≈%u 周期）：此刻 = %s\n",
               (unsigned)secs, DBG_CFG_LASER_NAME,
               (dwt_ok != 0u) ? "DWT 可用" : "**DWT 不可用**（只报段数）", (unsigned)s_us_div,
               (Dbg_Gpio_LaserPart() != 0u) ? "检测中" : "无检测");
    Dbg_Printf("[LASER] 现在把货从光路上放过去（手动丢 / `pick` 别松爪再跑本命令）……\n");

    while ((int32_t)(HAL_GetTick() - t_end) < 0) {
        if (Dbg_Gpio_LaserPart() != 0u) {
            if (in == 0u) {
                in        = 1u;
                start_cyc = DWT->CYCCNT;
            }
        } else if (in != 0u) {
            uint32_t cyc = DWT->CYCCNT - start_cyc;
            uint32_t us  = (s_us_div != 0u) ? (cyc / s_us_div) : 0u;

            in = 0u;
            n++;
            Dbg_Printf("[LASER] 事件 #%u：宽 %u 周期（%u µs）%s\n", (unsigned)n, (unsigned)cyc,
                       (unsigned)us,
                       (us == 0u) ? "  ← 极窄：等于一个采样点，电平轮询抓不到（要靠 EXTI 沿）"
                                  : "");
        }
        if (HAL_GetTick() != guard) { /* 每 1ms 边界照看一次：限位 + 软急停 */
            guard = HAL_GetTick();
            Dbg_App_LimitWatch();
            if (Dbg_Uart_Available() > 0u) {
                Dbg_Printf("[LASER] 收到控制台字节 —— 中止捕获（已量到 %u 段）\n", (unsigned)n);
                return;
            }
        }
    }
    if (in != 0u) { /* 窗口结束时还在检测中：报下限 */
        Dbg_Printf("[LASER] 窗口结束时**仍在检测中**（≥%u 周期）—— 光路一直被挡着？\n",
                   (unsigned)(DWT->CYCCNT - start_cyc));
        n++;
    }
    Dbg_Printf("[OK] 捕获结束：%u 段事件（**一次落料 = 1 段**）。0 段 = 光路上什么都没过"
               "（没丢货 / 没对准 / 模块没工作 / 线接错）\n",
               (unsigned)n);
    if (n != 0u) {
        Dbg_Printf("[LASER] 判据只认\"窗口内有没有捕到\"（产品侧 = EXTI 下降沿，见 §3.6）——\n"
                   "        宽度 ≥ 几 ms 时电平轮询也抓得住；若是上面标的\"极窄\"，就只有沿靠得住\n");
    }
}

/* ★ 投放点落料复核（`laser verify`）：**激光模块自身的验收工具**。
   ★ 2026-10-10 起产品不用它当成功判据了（爪下伸到投放点把光路挡死 ⇒ 判据换成视觉
   "两帧数量差 1"，见产品 sort_config.h 的"数量复核"一节）：本命令现在回答的是
   "这个模块 + 这个安装位置还能不能看见落料"——哪天把它挪到爪够不着的位置，先在这里验通，
   再把产品 CFG_LASER_ENABLE 改回 1。
   序列：静置 → 松爪前先看一眼电平 → **开爪落料** → 窗口内 µs 级密采样 ⇒ 判"捕到检测 / 夹空"。
   ⚠️ 本命令**会真的松爪**（爪里的货会落到投放点下方）；只想看信号而不动爪，用 `laser drop`。
   用法：`pick <物块X Y>` 抓起来别松爪 → `drop` 到投放点 → `laser verify`
   @retval 1 = 捕到检测  0 = 夹空（或期间被控制台字节打断） */
uint8_t Dbg_App_LaserVerifyDrop(void) {
    uint32_t t0;
    uint32_t guard;
    uint8_t  seen;
    uint32_t t_first = 0u;

    (void)Dbg_App_DwtCalibrate(); /* 只是让 s_us_div 就绪（本函数不用 µs，留着以后打印用） */

    Dbg_Printf("[LASER] 复核开始（%s）：静置 %ums → 开爪落料 → %ums 窗口内密采样找检测\n",
               DBG_CFG_LASER_NAME, (unsigned)DBG_CFG_LASER_SETTLE_MS,
               (unsigned)DBG_CFG_LASER_WAIT_MS);
    if (Dbg_App_WaitMsStop(DBG_CFG_LASER_SETTLE_MS) != 0u) {
        Dbg_Printf("[LASER] 静置期间收到控制台字节 —— 中止（未判定）\n");
        return 0u;
    }

    seen = (Dbg_Gpio_LaserPart() != 0u) ? 1u : 0u; /* 松爪前先看一眼（稳态电平型模块算证据） */
    if (seen != 0u) {
        Dbg_Printf("[LASER] 松爪前电平即\"检测到\"（光路看到爪里的货；按产品口径这已是证据）\n");
    }

    Dbg_Printf("[LASER] 松爪：夹爪 -> 全开 %u us（货落下，随后找那一下窄事件）\n",
               (unsigned)DBG_CFG_CLAW_OPEN_US);
    (void)Servo_Out_SetUs(DBG_SERVO_CLAW, DBG_CFG_CLAW_OPEN_US);

    /* 窗口内**裸忙等密采样**（µs 级；与 `laser drop` 同款）：窄事件必须靠这个才捕得到。
       每 1ms 边界照看一次限位与软急停；捕到第一段即退出 */
    t0    = HAL_GetTick();
    guard = t0;
    while ((seen == 0u) && ((HAL_GetTick() - t0) < DBG_CFG_LASER_WAIT_MS)) {
        if (Dbg_Gpio_LaserPart() != 0u) {
            seen    = 1u;
            t_first = HAL_GetTick();
            break;
        }
        if (HAL_GetTick() != guard) {
            guard = HAL_GetTick();
            Dbg_App_LimitWatch();
            if (Dbg_Uart_Available() > 0u) {
                Dbg_Printf("[LASER] 采样期间收到控制台字节 —— 中止（未判定）\n");
                return 0u;
            }
        }
    }
    /* 与产品一致：捕到会提前退出，这里把舵机行程时间补足 */
    if ((HAL_GetTick() - t0) < (uint32_t)DBG_CFG_SERVO_SETTLE_MS) {
        Dbg_App_WaitMs((uint32_t)DBG_CFG_SERVO_SETTLE_MS - (uint32_t)(HAL_GetTick() - t0));
    }

    if (seen != 0u) {
        Dbg_Printf("[OK] 判定：**捕到检测**（松爪后 %ums 内）—— 这个模块在这条安装位置看得见落料\n",
                   (unsigned)(t_first - t0));
        return 1u;
    }
    Dbg_Printf("[WARN] 判定：**没看见落料**（%ums 窗口内光路上什么都没过）—— 爪里明明有货却没看见 ⇒\n",
               (unsigned)DBG_CFG_LASER_WAIT_MS);
    Dbg_Printf("[WARN] 爪里明明有货 ⇒ 按序查：①`laser drop` 看段数（0 段 = 光路/模块/接线）"
               "②`laser` 看**原始电平**（判定反了改 §3.6 的极性宏）③光路对准 ④模块供电/共地 "
               "⑤输出类型（NPN 集电极开路才是\"导通=低\"）\n");
    return 0u;
}

/* ★ 激光实时观察（`laser watch [秒]`）：电平**变化**才打一行（5ms 采样）——对光/查静态遮挡用：
   用手挡一下光路，行数该跟着变。⚠️ **短脉冲量不到**（5ms 采样会漏），
   要量落料脉冲用 `laser drop`。期间限位与软急停照常盯（走 WaitMsStop）。任意键中止 */
void Dbg_App_LaserWatch(uint32_t secs) {
    uint32_t t0    = HAL_GetTick();
    uint32_t dur   = secs * 1000u;
    uint8_t  last  = Dbg_Gpio_LaserPart();
    uint32_t flips = 0u;

    Dbg_Printf("[LASER] 观察 %us（%s，每 %ums 采一次，只在变化时打印）：此刻 = %s（原始电平 %u）\n",
               (unsigned)secs, DBG_CFG_LASER_NAME, (unsigned)DBG_CFG_LASER_POLL_MS,
               (last != 0u) ? "检测中" : "无检测", (unsigned)Dbg_Gpio_LaserRaw());
    Dbg_Printf("[LASER] ⚠️ 短脉冲（落料那一下）本命令量不到——用 `laser drop`；本命令看的是"
               "对准/静态遮挡\n");
    while ((HAL_GetTick() - t0) < dur) {
        uint8_t now_lv = Dbg_Gpio_LaserPart();

        if (now_lv != last) {
            flips++;
            last = now_lv;
            Dbg_Printf("[LASER] t=%ums -> %s（原始电平 %u）\n", (unsigned)(HAL_GetTick() - t0),
                       (now_lv != 0u) ? "检测中" : "无检测", (unsigned)Dbg_Gpio_LaserRaw());
        }
        if (Dbg_App_WaitMsStop(DBG_CFG_LASER_POLL_MS) != 0u) {
            Dbg_Printf("[LASER] 收到控制台字节 —— 中止观察\n");
            return;
        }
    }
    Dbg_Printf("[OK] 观察结束：%us 内电平变化 %u 次。0 次 = 全程没动过（光路一直挡着/一直空着，"
               "或模块根本没工作）——用手挡一下光路该有 2 次以上\n",
               (unsigned)secs, (unsigned)flips);
}

/* ★ 停止帧台架实测：广播走 10mm@3mm/s，600ms 后（电机正在走）发停止序列。
   每帧应答判决现场打印——电机停没停 + 哪帧什么应答，一次实验定案：
   02 且停了 = 停止帧好使（此前失败=背靠背丢帧/堵转保护）；
   E2 = 堵转保护或未使能（手册原文，停不下来的直接原因）；
   无应答 = 帧没到驱动器。 */
uint8_t Dbg_App_StopTest(void) {
    Dbg_Printf("[STOPTEST] 广播走 10mm@3mm/s，600ms 后电机正在走时发停止序列——盯住电机\n");
    Dbg_Printf("[STOPTEST] （每次点击 X+ 挪约 2mm 是测试本身；此位移不进位置记账，用 `p` 前重新归零）\n");
    for (uint8_t motor = 0u; motor < (uint8_t)DBG_MOTOR_TRAY; motor++) {
        if (Zdt_Bus_MoveOn(motor, DBG_CFG_ADDR_BROADCAST, 10 * (int32_t)Dbg_App_PulsePerMm(),
                           Dbg_App_SpeedField(3.0f), DBG_APP_ACCEL) == 0u) {
            Dbg_Printf("[FAIL] 广播运动帧没发出去（%s线）\n", Zdt_Bus_NameOf((Dbg_Motor_e)motor));
            return 0u;
        }
    }
    Dbg_App_WaitMs(600);
    Dbg_Printf("[STOPTEST] 发停止（此刻电机应该在走）：\n");
    (void)Dbg_App_StopBoth();
    Dbg_Printf("[HUMAN] 电机停了没有？停了 = 停止帧好使（0928e 已验证过一轮）；没停 = 堵转保护/"
               "驱动器接受不执行——断电或驱动器按键清保护再试\n");
    return 1u;
}

uint8_t Dbg_App_BroadcastMove(int32_t mm, uint16_t rpm) {
    int32_t  pulse_signed;
    uint32_t est_ms;
    uint32_t t0;
    uint8_t  ok = 1u;

    if ((mm == 0) || (mm > (int32_t)DBG_CFG_MAX_MOVE_MM) || (mm < -(int32_t)DBG_CFG_MAX_MOVE_MM)) {
        Dbg_Printf("[FAIL] 位移非法（0 或超 %dmm）\n", (int)DBG_CFG_MAX_MOVE_MM);
        return 0u;
    }
    if ((rpm == 0u) || (rpm > 300u)) {
        rpm = 30u;
    }

    /* 两台同向同量：A = B = mm × 脉冲/mm，换算回去就是纯 X 位移 mm */
    pulse_signed = mm * (int32_t)Dbg_App_PulsePerMm();
    est_ms       = Kinematics_MoveTimeMs(pulse_signed, (float)rpm);

    Dbg_Printf("[INFO] 广播 X%+dmm（两台同向同量，各 %u 脉冲 @%urpm）估算 %u ms\n", (int)mm,
               (unsigned)((pulse_signed > 0) ? pulse_signed : -pulse_signed), (unsigned)rpm,
               (unsigned)est_ms);

    /* 广播运动只发 A/B 两条线（= 纯 X 测试）；转盘别被广播带着转，它走 thome/tb/j t。
       只发不收（本批驱动器广播不回包） */
    for (uint8_t motor = 0u; motor < (uint8_t)DBG_MOTOR_TRAY; motor++) {
        if (Zdt_Bus_MoveOn(motor, DBG_CFG_ADDR_BROADCAST, pulse_signed,
                           Dbg_App_SpeedField((float)rpm), DBG_APP_ACCEL) == 0u) {
            Dbg_Printf("[FAIL] 广播帧没发出去（%s线/供电问题）\n", Zdt_Bus_NameOf((Dbg_Motor_e)motor));
            ok = 0u;
        }
    }
    if (ok == 0u) {
        (void)Zdt_Bus_StopBoth();
        return 0u;
    }

    t0 = HAL_GetTick();
    if (Dbg_App_WaitMsStop(est_ms) != 0u) {
        (void)Zdt_Bus_StopBoth();
        Dbg_Printf("[WARN] 软急停（控制台字节）——广播运动中止于 %ums\n",
                   (unsigned)(HAL_GetTick() - t0));
        return 1u;
    }
    Dbg_Printf("[OK] 走完 实测 %u ms / 估算 %u ms\n", (unsigned)(HAL_GetTick() - t0),
               (unsigned)est_ms);
    Dbg_Printf("[HUMAN] 两台应同时同向转；龙门应整体平移 %dmm（方向对不对记下来）\n", (int)mm);
    return 1u;
}

/* 单台原始点动。motor：0=A 线 1=B 线 2=转盘线（addr 用该台登记地址）；-1 = 广播/数字地址，
   同一帧 A/B 两条线各发一遍（只有 ID 匹配的那台执行；转盘**不**含在广播里，它用 `j t`）。
   脉冲为正 = 按产品口径的 CW */
uint8_t Dbg_App_Jog(int8_t motor, uint8_t addr, uint8_t ccw, uint16_t rpm, uint8_t accel,
                    uint32_t pulse) {
    int32_t  signed_pulse = (ccw != 0u) ? -(int32_t)pulse : (int32_t)pulse;
    uint32_t est_ms;
    uint32_t t0;

    est_ms = Kinematics_MoveTimeMs(signed_pulse, (float)rpm);
    if (motor < 0) {
        Dbg_Printf("[INFO] 地址 %u（A/B 两线同发）%s %u 脉冲 %u rpm accel=%u  估算 %u ms\n",
                   (unsigned)addr, (ccw != 0u) ? "CCW" : "CW", (unsigned)pulse, (unsigned)rpm,
                   (unsigned)accel, (unsigned)est_ms);
    } else {
        Dbg_Printf("[INFO] %s线 地址 %u %s %u 脉冲 %u rpm accel=%u  估算 %u ms\n",
                   Zdt_Bus_NameOf((Dbg_Motor_e)motor), (unsigned)addr, (ccw != 0u) ? "CCW" : "CW",
                   (unsigned)pulse, (unsigned)rpm, (unsigned)accel, (unsigned)est_ms);
    }
    if (motor == (int8_t)DBG_MOTOR_TRAY) {
        Dbg_Printf("[INFO] 转盘：%u 脉冲 = 电机轴 1 圈 = 转盘 %u.%u°（总比 %u.%u = 皮带 %u:%u × 减速箱 1:%u）；对格用 `thome`+`tb`\n",
                   (unsigned)CFG_PULSE_PER_REV_57,
                   (unsigned)(3600u * CFG_TRAY_BELT_TEETH_MOTOR /
                              (CFG_TRAY_BELT_TEETH_GEAR * CFG_TRAY_GEAR_RATIO) / 10u),
                   (unsigned)(3600u * CFG_TRAY_BELT_TEETH_MOTOR /
                              (CFG_TRAY_BELT_TEETH_GEAR * CFG_TRAY_GEAR_RATIO) % 10u),
                   (unsigned)(CFG_TRAY_BELT_TEETH_GEAR * CFG_TRAY_GEAR_RATIO /
                              CFG_TRAY_BELT_TEETH_MOTOR),
                   (unsigned)((CFG_TRAY_BELT_TEETH_GEAR * CFG_TRAY_GEAR_RATIO * 10u /
                               CFG_TRAY_BELT_TEETH_MOTOR) % 10u),
                   (unsigned)CFG_TRAY_BELT_TEETH_MOTOR, (unsigned)CFG_TRAY_BELT_TEETH_GEAR,
                   (unsigned)CFG_TRAY_GEAR_RATIO);
    }
    /* `j` 是原始帧路径：不做速度系数换算，你给多少就发多少（便于逐字节对照参考固件） */
    Dbg_Printf("[INFO] 帧内速度字段 = %u（`j` 不做系数换算；当前系数 %u 只作用于 bx/x/y/xy/sq/cal）\n",
               (unsigned)rpm, (unsigned)s_speed_scale);

    {
        /* 广播只到 A/B（转盘不进广播，它用 `j t` 单独发） */
        uint8_t last = (motor < 0) ? (uint8_t)DBG_MOTOR_B : (uint8_t)motor;

        for (uint8_t m = 0u; m <= last; m++) {
            uint8_t ok;

            if ((motor >= 0) && (m != (uint8_t)motor)) {
                continue;
            }
            ok = Zdt_Bus_MoveOn(m, addr, signed_pulse, rpm, accel);
            if (ok == 0u) {
                Dbg_Printf("[FAIL] 帧没发出去（%s线 地址 %u 不会动）\n",
                           Zdt_Bus_NameOf((Dbg_Motor_e)m), (unsigned)addr);
                return 0u;
            }
            if (addr != DBG_CFG_ADDR_BROADCAST) {
                /* 单播数字地址 = 诊断路径：驱动器对自己的 ID 会回包；广播不回包（正常） */
                Dbg_App_AckReport(m, "位置", addr);
            }
        }
    }

    t0 = HAL_GetTick();
    if (Dbg_App_WaitMsStop(est_ms) != 0u) {
        (void)Zdt_Bus_StopBoth();
        Dbg_Printf("[WARN] 软急停（控制台字节）——jog 中止于 %ums\n",
                   (unsigned)(HAL_GetTick() - t0));
        return 1u;
    }
    Dbg_Printf("[OK] 等满 %u ms（到位判定本就是时间估算，无发送应答）\n", (unsigned)est_ms);
    if (pulse == (uint32_t)CFG_PULSE_PER_REV_42) {
        Dbg_Printf("[HUMAN] 这正好是电机一圈：秒表掐一下，约 %u ms 说明帧内速度字段就是 RPM，\n",
                   (unsigned)est_ms);
        Dbg_Printf("[HUMAN]   若实测约 10 倍（≈%u ms）则字段是 0.1RPM —— 改 `spu 10` 并记进待办\n",
                   (unsigned)(est_ms * 10u));
    }
    return 1u;
}

/* 解算 + 双帧下发（帧间节奏/应答报告/失败急停都在这），**不含"等到位"**——
   普通运动（ApplyXY：发完等估算时间）与回零演练（HomeRun：途中轮询限位裸沿）共用。
   返回 0 = 有帧没发出去（串口/UE 问题，已停双轴、位置作废）。
   ★ 2026-09-30 用户定调：全广播、只发不收（一条线一台；本批驱动器广播不回包；
   产品同口径）——此前的"单播应答硬门槛"废除。防单电机的残存保险：A/B 各走各的
   线（一路串口坏不拖累另一路）、帧间隔 10ms（驱动器消化期，09-25/28 台架教训）、
   归零四相的电平复核逻辑 */
static uint8_t Dbg_App_SendXY(float dx, float dy, uint16_t speed_mm_s, uint32_t *est_ms_out) {
    int32_t  pulse_a = 0;
    int32_t  pulse_b = 0;
    float    rpm_a   = 0.0f;
    float    rpm_b   = 0.0f;
    uint32_t est_ms  = 0u;

    Kinematics_XYSolve(dx, dy, (float)speed_mm_s, &pulse_a, &pulse_b, &rpm_a, &rpm_b, &est_ms);

    s_last_pulse_a = pulse_a;
    s_last_pulse_b = pulse_b;
    s_last_rpm_a   = rpm_a;
    s_last_rpm_b   = rpm_b;
    s_last_est_ms  = est_ms;

    Dbg_Printf("[INFO] 解算：A %s%u脉冲 %urpm | B %s%u脉冲 %urpm | 估算 %u ms\n",
               (pulse_a >= 0) ? "CW+" : "CCW-", (unsigned)((pulse_a >= 0) ? pulse_a : -pulse_a),
               (unsigned)Dbg_App_RpmToU16(rpm_a), (pulse_b >= 0) ? "CW+" : "CCW-",
               (unsigned)((pulse_b >= 0) ? pulse_b : -pulse_b), (unsigned)Dbg_App_RpmToU16(rpm_b),
               (unsigned)est_ms);

    /* 纯 X 走：A/B 同向同量；纯 Y 走：A/B 反向同量。这条规律就是耦合正确性的判据。
       全广播只发不收（2026-09-30 定调）；帧间隔 10ms = 驱动器消化期（09-25/28 教训） */
    {
        uint8_t ok_a = Zdt_Bus_MoveMotor(DBG_MOTOR_A, pulse_a, Dbg_App_SpeedField(rpm_a),
                                         DBG_APP_ACCEL);
        uint8_t ok_b;

        Dbg_App_WaitMs(10);
        ok_b = Zdt_Bus_MoveMotor(DBG_MOTOR_B, pulse_b, Dbg_App_SpeedField(rpm_b), DBG_APP_ACCEL);
        if ((ok_a == 0u) || (ok_b == 0u)) {
            (void)Zdt_Bus_StopBoth();
            Dbg_Printf("[FAIL] %s 台位置帧没发出去（串口/UE 问题）——已停双轴\n",
                       (ok_a == 0u) ? "A" : "B");
            return 0u;
        }
    }

    *est_ms_out = est_ms;
    return 1u;
}

/* 解算 + 下发 + 等够时间；普通 X/Y 运动走这条（MoveXY / Square / Goto）。
   成功后按位移累加位置记账；运动中限位触发 = 实际位置未知 → 记账作废 */
static uint8_t Dbg_App_ApplyXY(float dx, float dy, uint16_t speed_mm_s) {
    uint32_t est_ms = 0u;
    uint32_t t0;

    if (Dbg_App_SendXY(dx, dy, speed_mm_s, &est_ms) == 0u) {
        return 0u;
    }

    if (est_ms != 0u) {
        t0 = HAL_GetTick();
        if (Dbg_App_WaitMsStop(est_ms) != 0u) {
            /* 运动中软急停：半途位置未知，记账作废（StopBoth 里清 homed 标志） */
            (void)Dbg_App_StopBoth();
            s_last_actual_ms = HAL_GetTick() - t0;
            Dbg_Printf("[WARN] 运动中软急停（控制台字节）——实测 %ums，位置记账作废\n",
                       (unsigned)s_last_actual_ms);
            return 0u;
        }
        s_last_actual_ms = HAL_GetTick() - t0;
    }

    /* ★ 撞限位的作废不再在这里做（2026-10-08）：判据 = 电平**连续压住**，由
       Dbg_App_LimitWatch 在等待期间（含本函数上面那段 WaitMsStop）处置，与产品同口径。
       旧写法是"运动前后比沿计数，只要多一次按压就作废"——把电气毛刺也当成撞限位，
       现场表现 = 刚归零完就被判"记账不可信"，`p`/`pick`/`read` 全拒（用户报
       "一上来连夹持点都到不了"）。真撞由上面那个 watch 负责停 + 作废，毛刺则忽略 */
    s_pos_x += dx;
    s_pos_y += dy;

    Dbg_Printf("[OK] 走完 实测 %u ms / 估算 %u ms\n", (unsigned)s_last_actual_ms,
               (unsigned)est_ms);
    if (s_last_actual_ms > (est_ms + 300u)) {
        Dbg_Printf("[WARN] 实测明显超估算：可能堵转/丢步，或细分与 CFG_PULSE_PER_REV_42 不符\n");
    }

    return 1u;
}

/* 等一段已下发的运动走完，同时盯着两件事：
   返回 1 = 该路限位又触发了（*edge_tick=沿时刻）  0 = 正常走完（est+200ms）  -1 = 软急停
   （退离相靠它发现"退错方向反而压向开关"，软急停相靠它响应键盘） */
static int8_t Dbg_App_WaitMoveWatch(Dbg_Limit_e limit, uint32_t est_ms, uint32_t *edge_tick) {
    uint32_t t0 = HAL_GetTick();

    *edge_tick = 0u;
    while ((HAL_GetTick() - t0) < (est_ms + 200u)) {
        if (Dbg_Uart_Available() > 0u) {
            return -1;
        }
        *edge_tick = Dbg_Gpio_TakeRawEdge(limit);
        if (*edge_tick != 0u) {
            return 1;
        }
    }
    return 0;
}

/* ★ 回零（X/Y 共用，**两相制**——2026-09-30 用户定调"简化"）：
   相位1 粗压：向开关侧走，限位裸沿触发即停（停帧链延迟实测 1.3~1.5ms，10mm/s 过冲 ≈0.02mm）；
        停后**电平复核必须=压下**——假沿（噪声）当场作废、换一次尝试重来
   相位2 退离：退 DBG_CFG_HOME_BACKOFF_MM，**电平必须回 1**（开关真脱开）；
        没松开就自动再退一份（共 2×退距）；仍不松开才算失败（提示调大退距/查机械接线）
   本轴原点 = 相位2 停下来的驻留位（记账置 0,0）。
   ★ 相对旧四相删掉的：反向脱开确认相、5mm/s 慢压定原点相、退离看守——依次被
   电平复核 + 触发沿延迟账（过冲 ≪1mm）+ 2×退距自动加倍覆盖。
   起点已压在开关上时先退开（否则等不到下降沿）。开关装在最大行程端；
   途中敲任何控制台字节 = 软急停（字节留在缓冲里，停完会被执行） */
static uint8_t Dbg_App_HomeRun(uint8_t axis_y, uint16_t speed_mm_s, const char *name) {
    Dbg_Limit_e limit    = (axis_y != 0u) ? DBG_LIMIT_Y : DBG_LIMIT_X;
    int         dir      = (axis_y != 0u) ? (int)DBG_CFG_HOME_DIR_Y : (int)DBG_CFG_HOME_DIR_X;
    float       approach = (float)dir * DBG_CFG_HOME_TRAVEL_MM;
    float       back     = -(float)dir * DBG_CFG_HOME_BACKOFF_MM;
    uint32_t    est_ms   = 0u;
    uint32_t    edge_tick = 0u;
    int8_t      r;

    if (speed_mm_s == 0u) {
        speed_mm_s = (uint16_t)DBG_CFG_HOME_SPEED_DEFAULT;
    }
    if (speed_mm_s > (uint16_t)DBG_CFG_HOME_SPEED_MAX) {
        speed_mm_s = (uint16_t)DBG_CFG_HOME_SPEED_MAX;
        Dbg_Printf("[WARN] 速度收到上限 %u mm/s\n", (unsigned)speed_mm_s);
    }

    Dbg_Printf("[HOME] %s 回零（两相）：粗压@%umm/s 撞开关即停 → 退离 %dmm\n", name,
               (unsigned)speed_mm_s, (int)DBG_CFG_HOME_BACKOFF_MM);
    Dbg_Printf("[HOME] 手放电源开关上；途中敲任意键=软急停\n");

    for (uint8_t attempt = 1u; attempt <= 2u; attempt++) {
        /* 起点已压在开关上：先慢慢退开，否则等不到下降沿 */
        if (Dbg_Gpio_Level(limit) == 0u) {
            Dbg_Printf("[HOME] 尝试%u：起点已压在开关上，先退开 %dmm\n", (unsigned)attempt,
                       (int)DBG_CFG_HOME_BACKOFF_MM * 2);
            Dbg_App_WaitMs(30);
            Dbg_Gpio_ClearRawEdges();
            if (Dbg_App_SendXY(axis_y ? 0.0f : back * 2.0f, axis_y ? back * 2.0f : 0.0f,
                               (uint16_t)DBG_CFG_HOME_SLOW_MM_S, &est_ms) == 0u) {
                return 0u;
            }
            r = Dbg_App_WaitMoveWatch(limit, est_ms, &edge_tick);
            Dbg_App_StopXYFast();
            if (r < 0) {
                Dbg_Printf("[HOME] 软急停（退开）\n");
                return 0u;
            }
            if (Dbg_Gpio_Level(limit) != 1u) {
                Dbg_Printf("[WARN] 退开 %dmm 后开关仍未松开——开关粘死/机械卡住/信号线对地，重试\n",
                           (int)DBG_CFG_HOME_BACKOFF_MM * 2);
                continue;
            }
        }

        /* ---- 相位 1：粗压到触发即停 ---- */
        Dbg_App_WaitMs(30); /* 停帧消化（上一动作刚发过广播停，09-25/28 教训） */
        Dbg_Gpio_ClearRawEdges();
        if (Dbg_App_SendXY(axis_y ? 0.0f : approach, axis_y ? approach : 0.0f, speed_mm_s,
                           &est_ms) == 0u) {
            return 0u;
        }
        r = Dbg_App_WaitMoveWatch(limit, est_ms, &edge_tick);
        if (r < 0) {
            (void)Dbg_App_StopBoth();
            Dbg_Printf("[HOME] 软急停（粗压相）\n");
            return 0u;
        }
        Dbg_App_StopXYFast();
        if (r == 0) {
            Dbg_Printf("[WARN] %s 走满 %dmm 没等到触发沿：开关没被压到/方向反了（HOME_DIR 宏）/"
                       "线不对（`lim` 看电平与按压计数），换一次重来\n",
                       name, (int)DBG_CFG_HOME_TRAVEL_MM);
            continue;
        }
        Dbg_Printf("[HOME] 相位1 ✓ 触发，裸沿→停止 %ums（%umm/s 过冲 ≈ %uµm）\n",
                   (unsigned)(Dbg_App_LastStopDoneTick() - edge_tick), (unsigned)speed_mm_s,
                   (unsigned)((uint32_t)speed_mm_s * (Dbg_App_LastStopDoneTick() - edge_tick)));
        if (Dbg_Gpio_Level(limit) != 0u) {
            Dbg_Printf("[WARN] 触发后电平已回高 = 假沿（噪声），本次作废、换一次重来\n");
            continue;
        }

        /* ---- 相位 2：退离（没松开就自动再退一份，共 2×退距） ---- */
        for (uint8_t k = 0u; k < 2u; k++) {
            Dbg_App_WaitMs(30); /* 触发停止的停帧风暴消化后再发退离帧 */
            Dbg_Gpio_ClearRawEdges();
            if (Dbg_App_SendXY(axis_y ? 0.0f : back, axis_y ? back : 0.0f, speed_mm_s,
                               &est_ms) == 0u) {
                return 0u;
            }
            r = Dbg_App_WaitMoveWatch(limit, est_ms, &edge_tick);
            Dbg_App_StopXYFast();
            if (r < 0) {
                Dbg_Printf("[HOME] 软急停（退离相）\n");
                return 0u;
            }
            if (Dbg_Gpio_Level(limit) == 1u) {
                break; /* 已脱开 */
            }
            Dbg_Printf("[WARN] 退 %dmm 后仍未脱开%s\n", (int)DBG_CFG_HOME_BACKOFF_MM,
                       (k == 0u) ? "——自动再退一份（共 2×退距）" : "（已退 2×退距）");
        }
        if (Dbg_Gpio_Level(limit) != 1u) {
            Dbg_Printf("[FAIL] 退离 2×%dmm 后开关仍压着——退距太小/开关粘死/位置帧被丢；"
                       "调大 DBG_CFG_HOME_BACKOFF_MM 或查机械与接线后重跑\n",
                       (int)DBG_CFG_HOME_BACKOFF_MM);
            continue;
        }

        /* ---- 完成：本轴原点 = 驻留位（两轴都归完才能用 `p` 绝对定位） ---- */
        if (axis_y != 0u) {
            s_pos_y   = 0.0f;
            s_y_homed = 1u;
        } else {
            s_pos_x   = 0.0f;
            s_x_homed = 1u;
        }
        Dbg_Printf("[OK] %s 回零完成（第%u次）：原点 = 触发点内退 %dmm 的驻留位。"
                   "重复 3 次看原点重复性\n", name, (unsigned)attempt,
                   (int)DBG_CFG_HOME_BACKOFF_MM);
        return 1u;
    }

    Dbg_Printf("[FAIL] %s 回零两次尝试都失败——按上面每相的提示查（`lim` 看电平/按压计数）\n", name);
    return 0u;
}

uint8_t Dbg_App_HomeY(uint16_t speed_mm_s) {
    return Dbg_App_HomeRun(1u, speed_mm_s, "Y");
}

uint8_t Dbg_App_HomeX(uint16_t speed_mm_s) {
    return Dbg_App_HomeRun(0u, speed_mm_s, "X");
}

/* ★ 一键全回零（`home`，2026-10-01）：homex → homey → thome 顺序串跑——这就是产品
   上电 Sort_EnsureCalibrated 自动做的那一步（产品 Motion_HomeXY 也是 X 先 Y 后串行）。
   ★ 为什么不"两轴同时"回零：CoreXY 里纯 X 与纯 Y 各是一组 A/B 同/反向组合，某轴开关
   触发时必须两台都停——另一轴的回零运动就成了"被中断的运动"，接着再发相对运动正踩在
   byte[10]=0x00"相对上一目标位置"的坑上（0928g 归零跑野的同源风险）。串行 = 每相都从
   静止的单轴干净运动开始，判据简单、无交叉污染；台架与产品同口径。
   ★ 三步互不污染已核对：HomeRun 收尾用 StopXYFast（不碰记账）、TrayHome 只停 T 线；
   只有手动 `stop`（StopBoth）才清记账——那本来就该清 */
uint8_t Dbg_App_HomeAll(void) {
    Dbg_Printf("[HOME] ===== 全回零开始：homex → homey → thome（= 产品上电自动归零） =====\n");

    if (Dbg_App_HomeX(0u) == 0u) {
        Dbg_Printf("[HOME] X 未归零，全回零中止（Y/转盘未动；`lim` 核开关后单独重试 `homex`）\n");
        return 0u;
    }
    if (Dbg_App_HomeY(0u) == 0u) {
        Dbg_Printf("[HOME] Y 未归零，全回零中止（转盘未动；单独重试 `homey`）\n");
        return 0u;
    }
    if (Dbg_App_TrayHome(0u) == 0u) {
        Dbg_Printf("[HOME] 转盘未回零（XY 已可信；单独重试 `thome`）\n");
        return 0u;
    }
    Dbg_Printf("[HOME] ✓ 全回零完成：XY 记账 + 转盘记账可信，`p`/`drop`/`pick`/`tb` 可用\n");
    return 1u;
}

/* ================= 转盘 57 步进（2026-09-30 接入，对齐产品 Motion_TrayHome/GotoBin） ================= */

#define DBG_TRAY_DIR_FWD  ((uint8_t)CFG_TRAY_DIR_FORWARD)      /* 产品标定：正转方向字节 */
#define DBG_TRAY_DIR_BACK ((uint8_t)(1u - (uint8_t)CFG_TRAY_DIR_FORWARD))

/* 位置帧按"正脉冲=CW=dir 0x00"编方向；转盘的"正转"由 CFG_TRAY_DIR_FORWARD 标定，
   所以 FWD=0x00 时正脉冲、FWD=0x01 时负脉冲 —— 常量改这里自动跟着对 */
static int32_t Dbg_App_TraySigned(uint32_t pulses, uint8_t dir_byte) {
    return (dir_byte == 0x00u) ? (int32_t)pulses : -(int32_t)pulses;
}

/* 转盘按电机轴口径相对走 pulses 个脉冲（dir_byte = 产品标定的方向字节）。
   ★ 速度字段走转盘自己的系数（tspu），与 XY 的 spu 独立 */
/* 名义转速 -> 实际下发名义转速（乘百分比，上限 600 = 产品 Zdt_SpeedField 的同值上限；
   默认百分比 100 ⇒ 逐相与产品同速：寻标 480 / 对格 240 / 慢压 120 = 产品 CFG_TRAY_*_RPM） */
static uint16_t Dbg_App_TrayRpm(uint16_t nominal) {
    uint32_t v = (uint32_t)nominal * (uint32_t)s_tray_speed_pct / 100u;

    if (v == 0u) {
        v = 1u;
    }
    if (v > 600u) {
        v = 600u;
    }
    return (uint16_t)v;
}

/* 名义估算时间 -> 实际等待窗口：单位系数放大（慢 10 倍）、速度百分比缩小（快了不用等） */
static uint32_t Dbg_App_TrayWaitOf(uint32_t est_ms) {
    return est_ms * (uint32_t)s_tray_scale * 100u / (uint32_t)s_tray_speed_pct + 500u;
}

static uint16_t Dbg_App_TraySpeedField(uint16_t rpm) {
    uint32_t field = (uint32_t)rpm * (uint32_t)s_tray_scale;

    if (field > 65535u) {
        field = 65535u;
    }
    if (field == 0u) {
        field = 1u;
    }
    return (uint16_t)field;
}

static uint8_t Dbg_App_TrayMovePulses(uint8_t dir_byte, uint32_t pulses, uint16_t rpm) {
    return Zdt_Bus_MoveMotor(DBG_MOTOR_TRAY, Dbg_App_TraySigned(pulses, dir_byte),
                             Dbg_App_TraySpeedField(Dbg_App_TrayRpm(rpm)), DBG_APP_ACCEL);
}

/* 转盘的各种超时都要跟着系数走（慢 10 倍时名义超时覆盖不了一圈、脱开等待也等不到） */
static uint32_t Dbg_App_TrayTimeout(uint32_t base_ms) {
    return Dbg_App_TrayWaitOf(base_ms);
}

/* 等转盘原点电平到指定状态（want_pressed：1=压下 0=脱开）。
   返回 1=等到  0=超时  -1=软急停（控制台字节）。电平轮询 = 产品 WaitTrayLevel 同口径 */
static int8_t Dbg_App_TrayWaitLevel(uint8_t want_pressed, uint32_t timeout_ms) {
    uint32_t t0 = HAL_GetTick();

    while ((HAL_GetTick() - t0) < timeout_ms) {
        if (Dbg_Uart_Available() > 0u) {
            return -1;
        }
        {
            uint8_t pressed = (Dbg_Gpio_Level(DBG_LIMIT_TRAY) == 0u) ? 1u : 0u;

            if (pressed == want_pressed) {
                return 1;
            }
        }
    }
    return 0;
}

/* ★ 转盘回零（五相，逐相对齐产品 Motion_TrayHome）：
   相位0 凸块可能正压在开关上：先正转退出窗口（2×backoff），才取得到干净的沿
   相位1 粗寻标：速度模式正转 SEEK_RPM，等原点压下；超时必须覆盖一整圈（HOME_TIMEOUT=25000ms 名义、按节拍缩放后仍 > 一圈）
   相位2 反向脱开：位置模式倒转 BACKOFF_PULSES —— 旋转轴只有一条沿，继续正转会把它甩到
         开关身后，慢速再压就得再等一整圈（远超 APPROACH 超时）
   相位3 慢压定零：速度模式正转 APPROACH_RPM，压下沿 = 索引零位（与后续定位同向，背隙已吃进）
   两次尝试（产品同款）；途中敲任意键 = 软急停（记账作废）。
   ★ 顺带验证原点开关校准：五相全程走通 = PB0 电平/极性/位置都对 */
uint8_t Dbg_App_TrayHome(uint16_t seek_rpm) {
    int8_t r;

    if (seek_rpm == 0u) {
        seek_rpm = (uint16_t)CFG_TRAY_SEEK_RPM;
    }
    /* ★ 2026-10-10 删掉旧的"seek_rpm ≤ 300"钳位：产品寻标就是 CFG_TRAY_SEEK_RPM = 480，
       这里钳到 300 ⇒ 台架回零比产品慢一大截（整圈 7.5s vs 4.7s）、过冲量也不一样
       ⇒ 台架上校出来的东西不可迁移（用户现场："载物盘太慢，无法像工程代码正确校准"）。
       上限统一交给 Dbg_App_TrayRpm 的 600（= 产品 ZDT_SPEED_FIELD_MAX 同口径），
       手滑给个超大值也只会到 600 */

    s_tray_trusted = 0u;

    Dbg_Printf("[THOME] 转盘回零（五相）：凸块退出 → 粗寻标@%urpm → 反向脱开 → 慢压@%durpm 定零（实际值 = 名义×%u%%）\n",
               (unsigned)Dbg_App_TrayRpm(seek_rpm), (unsigned)Dbg_App_TrayRpm(CFG_TRAY_APPROACH_RPM),
               (unsigned)s_tray_speed_pct);
    Dbg_Printf("[THOME] 原点开关 PB0（`lim` 可看电平）；手放电源开关上；途中敲任意键=软急停\n");

    /* 可能是在一笔定位中途来重新回零的：先停稳（产品同款） */
    (void)Zdt_Bus_StopMotor(DBG_MOTOR_TRAY);
    Dbg_App_WaitMs(50);

    for (uint8_t attempt = 1u; attempt <= 2u; attempt++) {
        /* ---- 相位 0：凸块退出 ---- */
        if (Dbg_Gpio_Level(DBG_LIMIT_TRAY) == 0u) {
            Dbg_Printf("[THOME] 尝试%u 相位0：凸块正压在开关上，先正转退出窗口\n", (unsigned)attempt);
            if (Dbg_App_TrayMovePulses(DBG_TRAY_DIR_FWD, CFG_TRAY_BACKOFF_PULSES * 2u,
                                       (uint16_t)CFG_TRAY_APPROACH_RPM) == 0u) {
                Dbg_Printf("[FAIL] 退出帧没发出去\n");
                return 0u;
            }
            r = Dbg_App_TrayWaitLevel(0u, Dbg_App_TrayTimeout(CFG_TRAY_APPROACH_TIMEOUT_MS));
            (void)Zdt_Bus_StopMotor(DBG_MOTOR_TRAY);
            if (r < 0) {
                Dbg_Printf("[THOME] 软急停（相位0）\n");
                return 0u;
            }
            if (r == 0) {
                Dbg_Printf("[THOME] 相位0 超时未脱开——开关粘死/常压？`lim` 看电平，重试\n");
                continue;
            }
        }

        /* ---- 相位 1：粗寻标（速度模式） ---- */
        Dbg_Printf("[THOME] 尝试%u 相位1：粗寻标@%urpm（超时 %ums 覆盖一整圈）\n", (unsigned)attempt,
                   (unsigned)Dbg_App_TrayRpm(seek_rpm), (unsigned)Dbg_App_TrayTimeout(CFG_TRAY_HOME_TIMEOUT_MS));
        if (Zdt_Bus_SpeedOn((uint8_t)DBG_MOTOR_TRAY, Zdt_Bus_AddrOf(DBG_MOTOR_TRAY),
                            DBG_TRAY_DIR_FWD, Dbg_App_TraySpeedField(Dbg_App_TrayRpm(seek_rpm)),
                            DBG_APP_ACCEL) == 0u) {
            Dbg_Printf("[FAIL] 速度模式帧没发出去\n");
            return 0u;
        }
        r = Dbg_App_TrayWaitLevel(1u, Dbg_App_TrayTimeout(CFG_TRAY_HOME_TIMEOUT_MS));
        (void)Zdt_Bus_StopMotor(DBG_MOTOR_TRAY);
        if (r < 0) {
            Dbg_Printf("[THOME] 软急停（相位1）\n");
            return 0u;
        }
        if (r == 0) {
            Dbg_Printf("[THOME] 相位1 走满 %ums 没等到原点压下：凸块没装/开关线不对（`lim` 核）/ "
                       "速度比预期慢 10 倍（0.1RPM 口径——`j t cw 60 3200 255`+秒表判定，`spu 10` 切换）\n",
                       (unsigned)Dbg_App_TrayTimeout(CFG_TRAY_HOME_TIMEOUT_MS));
            continue;
        }
        Dbg_App_WaitMs(50);

        /* ---- 相位 2：反向脱开（自动加退 + 过冲回找，2026-10-01）----
         ★ 台架实测：凸块压住开关的弧宽 > 600 脉冲（1.8°）——退一份退不出来 = 开关一直
           压着 → 本轮作废 → 下轮"凸块退出"相正向 3.6° 退出来再找一圈 = 用户看到的
           "碰按钮后又多转一圈"。修法 = 退一份没脱开就**原地再退一份**（最多 4 份 =
           7.2°），而不是烧掉整次尝试重来
         ★ 过冲回找（二次提速后实机现象）：480rpm 撞停的过冲可能把凸块直接送过开关
           作用弧——停车点已在开关身后、电平=松开，"反向退到松开"会立即**假成功**，
           慢压相就得从另一侧追将近一整圈（"白转一圈"）。处置：停车点已松开时先反向
           慢转 DBG_CFG_TRAY_REENTRY_PULSES 回找窗口，压上即停再走常规退出；走满没压
           上 = 假沿（停车点在窗口前），直接进慢压相。两条路都不用降速 */
        {
            uint8_t cleared      = 0u;
            uint8_t skip_release = 0u;

            if (Dbg_Gpio_Level(DBG_LIMIT_TRAY) != 0u) { /* 台架口径 1=松开/0=压下（勿照抄产品的 1=压下）：已松开 = 停车点不在窗口内 */
                Dbg_Printf("[THOME] 相位2：停车点已是松开（过冲出窗口/假沿）——反向回找窗口 ≤%u 脉冲\n",
                           (unsigned)DBG_CFG_TRAY_REENTRY_PULSES);
                if (Dbg_App_TrayMovePulses(DBG_TRAY_DIR_BACK, DBG_CFG_TRAY_REENTRY_PULSES,
                                           (uint16_t)CFG_TRAY_APPROACH_RPM) == 0u) {
                    Dbg_Printf("[FAIL] 回找帧没发出去\n");
                    return 0u;
                }
                r = Dbg_App_TrayWaitLevel(1u, Dbg_App_TrayWaitOf(Kinematics_TrayMoveTimeMs(
                                                  DBG_CFG_TRAY_REENTRY_PULSES,
                                                  (float)CFG_TRAY_APPROACH_RPM)));
                (void)Zdt_Bus_StopMotor(DBG_MOTOR_TRAY);
                if (r < 0) {
                    Dbg_Printf("[THOME] 软急停（相位2 回找）\n");
                    return 0u;
                }
                if (r == 0) {
                    Dbg_Printf("[THOME] 相位2：回找没压上（假沿，停车点在窗口前）——直接进慢压相\n");
                    skip_release = 1u;
                } else {
                    Dbg_Printf("[THOME] 相位2：回进窗口（过冲已确认），按常规反向退出\n");
                }
            }
            if (skip_release == 0u) {
                Dbg_Printf("[THOME] 相位2：反向脱开 %u 脉冲@%durpm（没脱开自动加退，最多 4 份）\n",
                           (unsigned)CFG_TRAY_BACKOFF_PULSES,
                           (unsigned)Dbg_App_TrayRpm(CFG_TRAY_APPROACH_RPM));
                for (uint8_t k = 1u; k <= 4u; k++) {
                    if (Dbg_App_TrayMovePulses(DBG_TRAY_DIR_BACK, CFG_TRAY_BACKOFF_PULSES,
                                               (uint16_t)CFG_TRAY_APPROACH_RPM) == 0u) {
                        Dbg_Printf("[FAIL] 脱开帧没发出去\n");
                        return 0u;
                    }
                    r = Dbg_App_TrayWaitLevel(0u, Dbg_App_TrayTimeout(CFG_TRAY_APPROACH_TIMEOUT_MS));
                    (void)Zdt_Bus_StopMotor(DBG_MOTOR_TRAY);
                    if (r < 0) {
                        Dbg_Printf("[THOME] 软急停（相位2）\n");
                        return 0u;
                    }
                    if (r == 1) {
                        cleared = 1u;
                        break;
                    }
                    Dbg_Printf("[THOME]   退了 %u 份（%u 脉冲）仍压着，再加退一份\n", (unsigned)k,
                               (unsigned)(k * CFG_TRAY_BACKOFF_PULSES));
                }
                if (cleared == 0u) {
                    Dbg_Printf("[THOME] 相位2 退满 4×%u 脉冲（=7.2°）仍未脱开——开关粘死或凸块"
                               "作用宽度异常，`lim` 看电平，查机械后重试\n",
                               (unsigned)CFG_TRAY_BACKOFF_PULSES);
                    continue;
                }
            }
        }
        Dbg_App_WaitMs(50);

        /* ---- 相位 3：慢压定零（速度模式，压下沿 = 索引零位） ---- */
        Dbg_Printf("[THOME] 相位3：慢压@%durpm，压下沿 = 零位\n",
                   (unsigned)Dbg_App_TrayRpm(CFG_TRAY_APPROACH_RPM));
        if (Zdt_Bus_SpeedOn((uint8_t)DBG_MOTOR_TRAY, Zdt_Bus_AddrOf(DBG_MOTOR_TRAY),
                            DBG_TRAY_DIR_FWD,
                            Dbg_App_TraySpeedField(Dbg_App_TrayRpm((uint16_t)CFG_TRAY_APPROACH_RPM)),
                            DBG_APP_ACCEL) == 0u) {
            Dbg_Printf("[FAIL] 速度模式帧没发出去\n");
            return 0u;
        }
        r = Dbg_App_TrayWaitLevel(1u, Dbg_App_TrayTimeout(CFG_TRAY_APPROACH_TIMEOUT_MS));
        (void)Zdt_Bus_StopMotor(DBG_MOTOR_TRAY);
        if (r < 0) {
            Dbg_Printf("[THOME] 软急停（相位3）\n");
            return 0u;
        }
        if (r == 0) {
            Dbg_Printf("[THOME] 相位3 超时未压下——粗压那次可能是假沿，重试\n");
            continue;
        }
        Dbg_App_WaitMs(50);

        /* ---- 相位 4：对格偏移（2026-10-01 用户实测定稿）----
         凸块索引零位与格中心差 DBG_CFG_TRAY_HOME_OFFSET_DEG（现值 10°，宏在 dbg_config.h；
         首版 15° 实测偏多已废）：零位定在索引沿后，
         **再正向转这么多度**才是盒1 正对投放位——与 XY"归零后退 3mm 驻留"同一思路。
         ★ 脉冲记账的原点仍 = 索引沿（s_tray_pulses 从偏移量起算）：索引沿复核
           （TrayIndexCrossings 按 PER_REV 分界）与盒号折算（TrayBinOf(5000)=0）都保持正确。
         ★ 附带好处：驻留位在开关外，转盘平时**不压着原点开关**（无磨损、无沿计数噪声） */
        {
            uint32_t off_pulses =
                (uint32_t)DBG_CFG_TRAY_HOME_OFFSET_DEG * CFG_TRAY_PULSES_PER_REV / 360u;

            if (off_pulses != 0u) {
                Dbg_Printf("[THOME] 相位4：正向偏移 %u°（%u 脉冲）到盒1 对准位\n",
                           (unsigned)DBG_CFG_TRAY_HOME_OFFSET_DEG, (unsigned)off_pulses);
                if (Dbg_App_TrayMovePulses(DBG_TRAY_DIR_FWD, off_pulses,
                                           (uint16_t)CFG_TRAY_APPROACH_RPM) == 0u) {
                    Dbg_Printf("[FAIL] 偏移帧没发出去\n");
                    return 0u;
                }
                if (Dbg_App_WaitMsStop(Dbg_App_TrayWaitOf(Kinematics_TrayMoveTimeMs(
                        off_pulses, (float)CFG_TRAY_APPROACH_RPM))) != 0u) {
                    (void)Zdt_Bus_StopMotor(DBG_MOTOR_TRAY);
                    s_tray_trusted = 0u;
                    Dbg_Printf("[WARN] 软急停（相位4）——偏移没走完，重新 thome\n");
                    return 0u;
                }
                s_tray_pulses = (int32_t)off_pulses; /* 记账原点 = 索引沿，驻留位 = 零位 + 偏移 */
            } else {
                s_tray_pulses = 0;
            }
        }

        /* ---- 以此为累计脉冲与盒号的共同基准（产品同款） ---- */
        s_tray_bin     = (uint8_t)CFG_TRAY_ZERO_BIN;
        s_tray_trusted = 1u;
        Dbg_Printf("[OK] 转盘回零完成：盒%u 对准投放位（索引零位+%u°偏移，驻留不压开关）。"
                   "重复 3 次看零位重复性；之后 `tb <类别>` 对格\n",
                   (unsigned)(s_tray_bin + 1u), (unsigned)DBG_CFG_TRAY_HOME_OFFSET_DEG);
        return 1u;
    }

    Dbg_Printf("[FAIL] 转盘回零两次尝试都失败——按各相位打印的提示查；"
               "开关本身用 `lim` 验（未压=1 压下=0，按压计数 +1/次）\n");
    return 0u;
}

/* 当前转盘记账（`tb` 不带参数 / `stat` 共用） */
void Dbg_App_TrayPrint(void) {
    Dbg_Printf("[TRAY] %s 盒%u 对准投放位  累计 %u 脉冲  原点电平=%u（1=开 0=压下）\n",
               (s_tray_trusted != 0u) ? "记账可信" : "未回零/不可信", (unsigned)(s_tray_bin + 1u),
               (unsigned)s_tray_pulses, (unsigned)Dbg_Gpio_Level(DBG_LIMIT_TRAY));
    Dbg_Printf("[TRAY] 类别→盒映射（抽签后改 dbg_config.h §4d）："
               "1→%u 2→%u 3→%u 4→%u 5→%u 6→%u\n",
               (unsigned)s_cat_to_bin[0], (unsigned)s_cat_to_bin[1], (unsigned)s_cat_to_bin[2],
               (unsigned)s_cat_to_bin[3], (unsigned)s_cat_to_bin[4], (unsigned)s_cat_to_bin[5]);
    Dbg_Printf("[TRAY] 每格 %u 电机脉冲、一圈 %u（3200 × 皮带 %u/%u × 减速箱 1:%u ÷ %u 格）；正转方向字节=%u（CFG_TRAY_DIR_FORWARD）\n",
               (unsigned)CFG_TRAY_PULSES_PER_BIN, (unsigned)CFG_TRAY_PULSES_PER_REV,
               (unsigned)CFG_TRAY_BELT_TEETH_GEAR, (unsigned)CFG_TRAY_BELT_TEETH_MOTOR,
               (unsigned)CFG_TRAY_GEAR_RATIO, (unsigned)CFG_TRAY_BINS,
               (unsigned)DBG_TRAY_DIR_FWD);
    Dbg_Printf("[TRAY] 速度系数 %u（`tspu`）节拍 %u%%（`tspd`）；粗寻标/对格/慢压 = %u/%u/%u rpm@电机轴\n",
               (unsigned)s_tray_scale, (unsigned)s_tray_speed_pct,
               (unsigned)((uint32_t)CFG_TRAY_SEEK_RPM * s_tray_speed_pct / 100u),
               (unsigned)((uint32_t)CFG_TRAY_SPEED_RPM * s_tray_speed_pct / 100u),
               (unsigned)((uint32_t)CFG_TRAY_APPROACH_RPM * s_tray_speed_pct / 100u));
}

/* ★ 转到指定盒号（对齐产品 Motion_TrayGotoBin/TrayFinish）：
   单方向正转趋近消背隙（跨零位自然多转一圈），steps × 每格脉冲 @ CFG_TRAY_SPEED_RPM；
   走完做产品同款双复核：①索引沿计数 = 理论整圈数（检出减速箱丢步/堵转）
   ②累计脉冲折算盒号 = 目标盒号（检出方向标定反了——FWD 应使盒号递增）。
   任一复核不过 = 记账作废，重新 thome。人眼复核：对准的盒是不是真在投放位 */
/* ★ tb <类别1~6>：一句到位——类别号查表映射到物理盒再转过去。
   抓取流程里的用法 = `read` 给出类别 n → `tb n`（和产品 Sort_DoCycle 的
   s_category_to_bin[target->category-1] 同一张表的台架副本） */
uint8_t Dbg_App_TrayGotoCategory(uint8_t category) {
    if ((category < 1u) || (category > (uint8_t)CFG_NUM_CATEGORIES)) {
        Dbg_Printf("[FAIL] 类别范围 1~%u（tb 的参数 = 视觉类别 ID，不再是 0~5 盒下标）\n",
                   (unsigned)CFG_NUM_CATEGORIES);
        return 0u;
    }
    return Dbg_App_TrayGotoBin((uint8_t)(s_cat_to_bin[category - 1u] - 1u));
}

uint8_t Dbg_App_TrayGotoBin(uint8_t bin) {
    uint32_t pulses;
    uint32_t est_ms;
    uint32_t t0;
    int32_t  pulses_mark;
    uint32_t index_mark;
    int32_t  delta;

    if (s_tray_trusted == 0u) {
        Dbg_Printf("[FAIL] 转盘记账不可信——先 `thome` 回零\n");
        return 0u;
    }
    if (bin >= (uint8_t)CFG_TRAY_BINS) {
        Dbg_Printf("[FAIL] 盒号范围 1~%u（=类别 ID；映射表 dbg_config.h §4d）\n",
                   (unsigned)CFG_TRAY_BINS);
        return 0u;
    }

    /* ★ 2026-10-08 与产品同口径：**双向走最近路径**（6 格时 |delta| ≤ 3，3 格平局仍走正转）。
       CFG_TRAY_ALLOW_REVERSE = 0 则退回老口径"单向正转消背隙"（最多白转 5 格） */
    delta = (int32_t)((bin + (uint8_t)CFG_TRAY_BINS - s_tray_bin) % (uint8_t)CFG_TRAY_BINS);
    if (delta > (int32_t)((uint8_t)CFG_TRAY_BINS / 2u)) {
        delta -= (int32_t)CFG_TRAY_BINS;
    }
#if CFG_TRAY_ALLOW_REVERSE == 0u
    if (delta < 0) {
        delta += (int32_t)CFG_TRAY_BINS;
    }
#endif
    if (delta == 0) {
        Dbg_Printf("[OK] 已对准盒 %u\n", (unsigned)(bin + 1u));
        return 1u;
    }

    pulses = (uint32_t)((delta > 0) ? delta : -delta) * (uint32_t)CFG_TRAY_PULSES_PER_BIN;
    est_ms = Kinematics_TrayMoveTimeMs(pulses, (float)CFG_TRAY_SPEED_RPM);
    Dbg_Printf("[INFO] 转到盒%u：%s %d 格 = %u 电机脉冲 @%durpm（实际）%u ms 估算"
               "（名义口径，等待已按节拍缩放）\n",
               (unsigned)(bin + 1u), (delta > 0) ? "正转" : "反转", (int)delta, (unsigned)pulses,
               (unsigned)Dbg_App_TrayRpm(CFG_TRAY_SPEED_RPM), (unsigned)est_ms);

    pulses_mark = s_tray_pulses;
    index_mark  = Dbg_Gpio_EdgeCount(DBG_LIMIT_TRAY);

    if (Dbg_App_TrayMovePulses((delta > 0) ? DBG_TRAY_DIR_FWD : DBG_TRAY_DIR_BACK, pulses,
                               (uint16_t)CFG_TRAY_SPEED_RPM) == 0u) {
        s_tray_trusted = 0u; /* 指令没发出去 = 转盘根本不会动 */
        Dbg_Printf("[FAIL] 位置帧没发出去，转盘记账作废\n");
        return 0u;
    }

    t0 = HAL_GetTick();
    if (Dbg_App_WaitMsStop(Dbg_App_TrayWaitOf(est_ms)) != 0u) {
        (void)Zdt_Bus_StopMotor(DBG_MOTOR_TRAY);
        s_tray_trusted = 0u;
        Dbg_Printf("[WARN] 软急停（控制台字节）——转盘中止于 %ums，记账作废，重新 thome\n",
                   (unsigned)(HAL_GetTick() - t0));
        return 0u;
    }
    s_tray_pulses += (delta > 0) ? (int32_t)pulses : -(int32_t)pulses;

    /* 复核①：索引沿计数（dbg_gpio 的消抖按压计数，凸块每圈压一次 = 产品索引沿同口径） */
    {
        uint32_t expect = Kinematics_TrayCrossingsBetween(pulses_mark, s_tray_pulses);
        uint32_t actual = Dbg_Gpio_EdgeCount(DBG_LIMIT_TRAY) - index_mark;

        if (expect != actual) {
            s_tray_trusted = 0u;
            Dbg_Printf("[FAIL] 索引复核不过：理论 %u 次整圈 vs 实测 %u 次压下——减速箱丢步/堵转嫌疑，"
                       "记账作废，重新 thome\n",
                       (unsigned)expect, (unsigned)actual);
            return 0u;
        }
    }

    /* 复核②：按累计脉冲折算盒号（方向标定反向时这里现形） */
    {
        uint8_t pulse_bin =
            (uint8_t)((CFG_TRAY_ZERO_BIN + Kinematics_TrayBinOf(s_tray_pulses)) %
                      (uint8_t)CFG_TRAY_BINS);

        if (pulse_bin != bin) {
            s_tray_trusted = 0u;
            Dbg_Printf("[FAIL] 盒号复核不过：脉冲折算=盒%u 目标=盒%u —— CFG_TRAY_DIR_FORWARD 标反"
                       "（正转必须使盒号递增）或每格脉冲不对，记账作废\n",
                       (unsigned)pulse_bin, (unsigned)bin);
            return 0u;
        }
    }

    s_tray_bin = bin;
    Dbg_Printf("[OK] 已对准盒 %u（实际耗时 %ums，索引沿 %u 次全对上）。[HUMAN] 眼睛复核：该盒真在投放位吗？\n",
               (unsigned)bin, (unsigned)(HAL_GetTick() - t0),
               (unsigned)(Dbg_Gpio_EdgeCount(DBG_LIMIT_TRAY) - index_mark));
    return 1u;
}

uint8_t Dbg_App_MoveXY(int32_t dx_mm, int32_t dy_mm, uint16_t speed_mm_s) {
    if ((dx_mm == 0) && (dy_mm == 0)) {
        Dbg_Printf("[FAIL] 位移为 0，什么都不做\n");
        return 0u;
    }
    if ((dx_mm > (int32_t)DBG_CFG_MAX_MOVE_MM) || (dx_mm < -(int32_t)DBG_CFG_MAX_MOVE_MM) ||
        (dy_mm > (int32_t)DBG_CFG_MAX_MOVE_MM) || (dy_mm < -(int32_t)DBG_CFG_MAX_MOVE_MM)) {
        Dbg_Printf("[FAIL] 单段超上限 %dmm，拒绝执行（防手误）\n", (int)DBG_CFG_MAX_MOVE_MM);
        return 0u;
    }

    /* ★ 软限位（2026-09-30 新增）：归零后记账可信时，相对移动也不许走出工作区。
       没归零时无法判位（记账不可信），只受上面的单段上限约束——为的是"先把龙门从
       开关上挪开再归零"这类操作不被挡住 */
    if (Dbg_App_XyTrusted() != 0u) {
        float nx = s_pos_x + (float)dx_mm;
        float ny = s_pos_y + (float)dy_mm;

        if ((nx > DBG_CFG_XY_MAX_MM) || (nx < DBG_CFG_XY_X_MIN_MM) ||
            (ny > DBG_CFG_XY_MAX_MM) || (ny < DBG_CFG_XY_Y_MIN_MM)) {
            Dbg_Printf("[FAIL] 会走出工作区（目标 X=");
            Dbg_App_PrintF1(nx);
            Dbg_Printf(" Y=");
            Dbg_App_PrintF1(ny);
            Dbg_Printf("）——工作区 X ∈ [%d, 0]、Y ∈ [%d, 0]；要越过限位先 `homex`/`homey` "
                       "重定，或用 `bx`/`j` 原始帧（无记账、无软限位）。\n"
                       "       （贴边件的**绝对定位** `p`/`pick` 不在此列：它们走产品同款钳位"
                       "——延伸带/极限位带内钳到极限位照样夹）\n",
                       (int)DBG_CFG_XY_X_MIN_MM, (int)DBG_CFG_XY_Y_MIN_MM);
            return 0u;
        }
    }

    if (speed_mm_s == 0u) {
        speed_mm_s = (uint16_t)DBG_CFG_DEFAULT_SPEED_MM_S;
    }
    if (speed_mm_s > (uint16_t)DBG_CFG_MAX_SPEED_MM_S) {
        speed_mm_s = (uint16_t)DBG_CFG_MAX_SPEED_MM_S;
        Dbg_Printf("[WARN] 速度收到上限 %dmm/s\n", (int)DBG_CFG_MAX_SPEED_MM_S);
    }

    Dbg_Printf("[INFO] 意图 X%+dmm Y%+dmm @%dmm/s\n", (int)dx_mm, (int)dy_mm, (int)speed_mm_s);
    return Dbg_App_ApplyXY((float)dx_mm, (float)dy_mm, speed_mm_s);
}

uint8_t Dbg_App_Square(int32_t mm, uint16_t speed_mm_s) {
    Dbg_Printf("[INFO] 方块测试：%dmm 四段（X+ Y+ X- Y-），应回到起点附近\n", (int)mm);

    if (Dbg_App_MoveXY(mm, 0, speed_mm_s) == 0u) {
        return 0u;
    }
    if (Dbg_App_MoveXY(0, mm, speed_mm_s) == 0u) {
        return 0u;
    }
    if (Dbg_App_MoveXY(-mm, 0, speed_mm_s) == 0u) {
        return 0u;
    }
    return Dbg_App_MoveXY(0, -mm, speed_mm_s);
}

uint16_t Dbg_App_PulsePerMm(void) {
    return (uint16_t)CFG_XY_PULSE_PER_MM;
}

/* 打印当前记账位置与可信状态（`p` 不带参数 / `stat` 共用） */
void Dbg_App_PrintPosition(void) {
    Dbg_Printf("[POS] 当前 (");
    Dbg_App_PrintF1(s_pos_x);
    Dbg_Printf(", ");
    Dbg_App_PrintF1(s_pos_y);
    Dbg_Printf(")  %s  工作区 X ∈ [%d, %d]  Y ∈ [%d, %d]（第三象限，原点=归零驻留位）\n",
               (Dbg_App_XyTrusted() != 0u) ? "记账可信" : "记账不可信——先 homex + homey",
               (int)DBG_CFG_XY_X_MIN_MM, (int)DBG_CFG_XY_MAX_MM,
               (int)DBG_CFG_XY_Y_MIN_MM, (int)DBG_CFG_XY_MAX_MM);
}

/* ★ 产品同款目标钳位（2026-10-10 从 Dbg_App_Goto / read 的两份副本里收出来）：
   规则与产品 Code/Config + sort_config.h 的"够得着就去夹"那一套**逐条同序**，常量全部由
   dbg_config.h 引用产品值（台架与产品不会各说各话）：
     ① X < -X 延伸带（-160）            ⇒ 拒绝（连延伸也够不着）
     ② Y < 工作区下限（-187 = 投放点）   ⇒ 拒绝（-Y 端是机械极限，无延伸量）
     ③ X/Y > 上限 + 极限位夹取带（25mm） ⇒ 拒绝
     ④ 落在 [延伸带, X_MIN) 或 (上限, 上限+带] ⇒ **钳到 X_MIN / 上限** 照样伸爪
     ⑤ 钳完按保护边距（3mm，2026-10-10 5→3 = 产品同值）回拉：> 上限-3mm ⇒ 到 上限-3mm（永不朝回零开关顶满）
   只判定与钳位、**不打印**（`p`/`pick` 走 Dbg_App_Goto、`read` 只预演，两处打印口径不同）。
   @param x,y 进出参（就地钳位）  @retval 1 = 可用  0 = 超出产品可达范围（产品会整件作废） */
static uint8_t Dbg_App_ProdClamp(float *x, float *y) {
    float cx = *x;
    float cy = *y;

    if ((cx < DBG_CFG_PROD_X_CLAMP_MM) || (cy < DBG_CFG_PROD_WORK_Y_MIN) ||
        (cx > (DBG_CFG_PROD_WORK_X_MAX + DBG_CFG_PROD_CLAMP_BAND_MM)) ||
        (cy > (DBG_CFG_PROD_WORK_Y_MAX + DBG_CFG_PROD_CLAMP_BAND_MM))) {
        return 0u;
    }
    if (cx < DBG_CFG_PROD_WORK_X_MIN) {
        cx = DBG_CFG_PROD_WORK_X_MIN; /* -X 夹爪延伸带内：钳到软限位就近夹 */
    }
    if (cx > DBG_CFG_PROD_WORK_X_MAX) {
        cx = DBG_CFG_PROD_WORK_X_MAX; /* +X 极限位夹取带内：钳到上限照样伸爪 */
    }
    if (cy > DBG_CFG_PROD_WORK_Y_MAX) {
        cy = DBG_CFG_PROD_WORK_Y_MAX;
    }
    if (cx > (DBG_CFG_PROD_WORK_X_MAX - DBG_CFG_PROD_WORK_EDGE_MARGIN)) {
        cx = DBG_CFG_PROD_WORK_X_MAX - DBG_CFG_PROD_WORK_EDGE_MARGIN;
    }
    if (cy > (DBG_CFG_PROD_WORK_Y_MAX - DBG_CFG_PROD_WORK_EDGE_MARGIN)) {
        cy = DBG_CFG_PROD_WORK_Y_MAX - DBG_CFG_PROD_WORK_EDGE_MARGIN;
    }

    *x = cx;
    *y = cy;
    return 1u;
}

/* ★ 绝对定位：走到工作区坐标 (x, y)（第三象限，负数）。
   前提 = 两轴都归过零且中途没停过/没撞过（记账可信）。工作区：X ∈ [-135, 0]、Y ∈ [-187, 0]；
   投放点 Y = -187（2026-10-01 口径）。
   ★ 2026-10-08：越界不再一律拒绝，改成**与产品 Sort_DoCycle 同口径的钳位**（"够得着就去夹"）
   ——台架必须能预演产品行为。现场实测："`read` 能到投放位、`pick` 报目标越界 = 到不了夹持点"，
   而产品那边这些目标其实会钳位照夹 ⇒ 台架必须走同一条路。逐条与产品同序（见 dbg_config.h
   "产品工作包络与够得着就去钳位"一节）：
     ① X ∈ [-160, -135) ⇒ 钳到 -135（夹爪延伸带）
     ② X < -160 / Y < -187 ⇒ 拒绝（产品同样整件作废、机器一动不动）
     ③ X/Y > 上限 且 ≤ 上限+25mm ⇒ 钳到上限照样伸爪（极限位夹取带）；> 25mm ⇒ 拒绝
     ④ 钳完按保护边距回拉：> 上限-3mm ⇒ 到 上限-3mm（永不朝回零开关顶满） */
uint8_t Dbg_App_Goto(float x, float y, uint16_t speed_mm_s) {
    float dx;
    float dy;

    if (Dbg_App_XyTrusted() == 0u) {
        Dbg_Printf("[FAIL] 位置记账不可信——先 `homex` + `homey` 归零\n");
        return 0u;
    }

    /* 产品同款钳位（延伸带 / 极限位夹取带 / 保护边距，与产品 Sort_ClampTarget 同序）——
       带外即拒（产品同样整件作废、机器一动不动），带内钳完照走 */
    {
        float cx = x;
        float cy = y;

        if (Dbg_App_ProdClamp(&cx, &cy) == 0u) {
            Dbg_Printf("[FAIL] 目标 (");
            Dbg_App_PrintF1(x);
            Dbg_Printf(", ");
            Dbg_App_PrintF1(y);
            Dbg_Printf(") 超出产品可达范围（-X 延伸带 %d、Y 下限 %d、+侧极限位带 %dmm）"
                       "——产品同样整件作废、机器一动不动\n",
                       (int)DBG_CFG_PROD_X_CLAMP_MM, (int)DBG_CFG_PROD_WORK_Y_MIN,
                       (int)DBG_CFG_PROD_CLAMP_BAND_MM);
            return 0u;
        }
        if ((cx != x) || (cy != y)) {
            Dbg_Printf("[GOTO] 贴边目标已按产品同款钳位 → 实际落点 (");
            Dbg_App_PrintF1(cx);
            Dbg_Printf(", ");
            Dbg_App_PrintF1(cy);
            Dbg_Printf(")（延伸带/极限位带/保护边距；落极限位可能夹偏/抓空）\n");
        }
        x = cx;
        y = cy;
    }

    dx = x - s_pos_x;
    dy = y - s_pos_y;
    if ((dx > -0.5f) && (dx < 0.5f) && (dy > -0.5f) && (dy < 0.5f)) {
        Dbg_Printf("[OK] 已在目标位\n");
        return 1u;
    }
    if (speed_mm_s == 0u) {
        speed_mm_s = (uint16_t)DBG_CFG_DEFAULT_SPEED_MM_S;
    }
    if (speed_mm_s > (uint16_t)DBG_CFG_MAX_SPEED_MM_S) {
        speed_mm_s = (uint16_t)DBG_CFG_MAX_SPEED_MM_S;
    }

    Dbg_Printf("[GOTO] 从 (");
    Dbg_App_PrintF1(s_pos_x);
    Dbg_Printf(", ");
    Dbg_App_PrintF1(s_pos_y);
    Dbg_Printf(") → (");
    Dbg_App_PrintF1(x);
    Dbg_Printf(", ");
    Dbg_App_PrintF1(y);
    Dbg_Printf(") 位移 (");
    Dbg_App_PrintF1(dx);
    Dbg_Printf(", ");
    Dbg_App_PrintF1(dy);
    Dbg_Printf(") @%umm/s\n", (unsigned)speed_mm_s);

    return Dbg_App_ApplyXY(dx, dy, speed_mm_s);
}

/* ★ 抓取全流程（pick）：机构构造决定的时序（dbg_config.h §3 注释）——
   巡航必须全开 → 到位后初夹 → 下放 → 放到底才夹紧 → 抬回。
   has_xy=1 时先走到 (x,y)（需先归零，`p` 同一套记账），且**张开在移动之前**做：
   巡航不全开会撞托盘边/旁边物块。每步打印对应整机流程的哪一段。
   ★ shape（0~3，缺省 0）：0 = 默认方案（兜底值）；1=立方体 2=12面体 3=六棱柱 取
   DBG_CFG_GRIP_* 表对应值——与产品 Sort_Grab(shape) 同口径；越界兜底 0 */
uint8_t Dbg_App_Pick(float x, float y, uint16_t speed_mm_s, uint8_t has_xy, uint8_t shape) {
    static const uint16_t grip_half_us[CFG_NUM_SHAPES]  = DBG_CFG_GRIP_HALF_US;
    static const uint16_t grip_close_us[CFG_NUM_SHAPES] = DBG_CFG_GRIP_CLOSE_US;
    uint16_t              half_us;
    uint16_t              close_us;

    if (shape > (uint8_t)CFG_NUM_SHAPES) {
        shape = 0u; /* 越界兜底 = 默认方案 */
    }
    if (shape == 0u) { /* 0 = 默认方案（不在三种特调之内）：兜底值 */
        half_us  = DBG_CFG_CLAW_HALF_US;
        close_us = DBG_CFG_CLAW_CLOSE_US;
    } else { /* 1~3 = 特调方案查表（下标 = 形状-1） */
        half_us  = grip_half_us[shape - 1u];
        close_us = grip_close_us[shape - 1u];
    }

    Dbg_Printf("[PICK] 夹持方案：形状 %u（0=默认；1=立方体 2=12面体 3=六棱柱）"
               "—— 初夹 %u / 夹紧 %u us\n",
               (unsigned)shape, (unsigned)half_us, (unsigned)close_us);
    Dbg_Printf("[PICK] ①巡航位重申：夹爪张开 %u us（巡航不全开会撞托盘边/旁边物块）\n",
               (unsigned)DBG_CFG_CLAW_OPEN_US);
    (void)Servo_Out_SetUs(DBG_SERVO_CLAW, DBG_CFG_CLAW_OPEN_US);
    if (Dbg_App_WaitMsStop(DBG_CFG_SERVO_SETTLE_MS) != 0u) {
        Dbg_Printf("[PICK] 软急停（第①步等待中）——后续步骤未执行\n");
        return 0u;
    }

    if (has_xy != 0u) {
        float tx = x;
        float ty = y;

        /* ★ 夹爪延伸夹取带（2026-10-01，dbg_config.h §4c）：物块可出现在 X ∈ [X_CLAMP,
           X_MIN)——龙门停在 X_MIN 时夹爪构造仍够得着。目标 X 超出软限位就**钳位到 X_MIN
           就近夹取**（打印提示，不拒绝）；比 X_CLAMP 还远的照常拒绝 */
        if (tx < DBG_CFG_XY_X_MIN_MM) {
            if (tx < DBG_CFG_XY_X_CLAMP_MM) {
                Dbg_Printf("[FAIL] X ");
                Dbg_App_PrintF1(tx);
                Dbg_Printf(" 超出夹爪延伸带（下限 %d）——夹爪延伸也够不着，拒绝\n",
                           (int)DBG_CFG_XY_X_CLAMP_MM);
                return 0u;
            }
            Dbg_Printf("[PICK] X ");
            Dbg_App_PrintF1(tx);
            Dbg_Printf(" 在夹爪延伸带内——钳位到 %d 就近夹取（夹爪构造覆盖剩余距离）\n",
                       (int)DBG_CFG_XY_X_MIN_MM);
            tx = DBG_CFG_XY_X_MIN_MM;
        }
        if (Dbg_App_Goto(tx, ty, speed_mm_s) == 0u) {
            Dbg_Printf("[FAIL] 定位失败（未归零/越界/软急停），爪时序未执行\n");
            return 0u;
        }
    }

    Dbg_Printf("[PICK] ②到目标 XY、下放之前：夹爪 -> 初步夹住 %u us（形状 %u）\n",
               (unsigned)half_us, (unsigned)shape);
    (void)Servo_Out_SetUs(DBG_SERVO_CLAW, half_us);
    if (Dbg_App_WaitMsStop(DBG_CFG_SERVO_SETTLE_MS) != 0u) {
        Dbg_Printf("[PICK] 软急停（第②步等待中，爪=初夹）——未下放。手动 `l 0` 抬回\n");
        return 0u;
    }

    Dbg_Printf("[PICK] ③下放：升降 -> 最低 %u us\n", (unsigned)DBG_CFG_LIFT_DOWN_US);
    (void)Servo_Out_SetUs(DBG_SERVO_LIFT, DBG_CFG_LIFT_DOWN_US);
    if (Dbg_App_WaitMsStop(DBG_CFG_SERVO_SETTLE_MS * 2u) != 0u) { /* 升降行程比换档慢，给双倍 */
        Dbg_Printf("[PICK] 软急停（第③步下放等待中）——未夹紧。手动 `l 0` 抬回、`g 0` 张开\n");
        return 0u;
    }

    Dbg_Printf("[PICK] ④放到底之后才夹紧：夹爪 -> %u us（形状 %u）\n", (unsigned)close_us,
               (unsigned)shape);
    (void)Servo_Out_SetUs(DBG_SERVO_CLAW, close_us);
    if (Dbg_App_WaitMsStop(DBG_CFG_SERVO_SETTLE_MS + 200u) != 0u) {
        Dbg_Printf("[PICK] 软急停（第④步夹紧等待中，爪=夹紧在底部）——手动 `l 0` 抬回\n");
        return 0u;
    }

    Dbg_Printf("[PICK] ⑤抬回最高 %u us（物块随爪走，平移前确认已离托盘面）\n",
               (unsigned)DBG_CFG_LIFT_UP_US);
    (void)Servo_Out_SetUs(DBG_SERVO_LIFT, DBG_CFG_LIFT_UP_US);
    if (Dbg_App_WaitMsStop(DBG_CFG_SERVO_SETTLE_MS) != 0u) {
        Dbg_Printf("[PICK] 软急停（第⑤步抬回等待中）——时序实质已完成\n");
        return 0u;
    }

    Dbg_Printf("[OK] 抓取时序走完（形状 %u）：开%u 初夹%u 紧%u / 升降 %u~%u us。哪步撞/顶就回来改值\n",
               (unsigned)shape, (unsigned)DBG_CFG_CLAW_OPEN_US, (unsigned)half_us,
               (unsigned)close_us, (unsigned)DBG_CFG_LIFT_DOWN_US, (unsigned)DBG_CFG_LIFT_UP_US);
    return 1u;
}

uint8_t Dbg_App_CalMove(int32_t mm, uint16_t ppm) {
    int32_t  pulse;
    uint32_t est_ms;
    uint32_t t0;
    uint8_t  ok_a;
    uint8_t  ok_b;

    if (ppm == 0u) {
        ppm = Dbg_App_PulsePerMm();
    }
    if ((mm == 0) || (mm > (int32_t)DBG_CFG_MAX_MOVE_MM) || (mm < -(int32_t)DBG_CFG_MAX_MOVE_MM)) {
        Dbg_Printf("[FAIL] 标定位移非法（0 或超 %dmm）\n", (int)DBG_CFG_MAX_MOVE_MM);
        return 0u;
    }

    /* 标定专用：不走解算器，直接按"X 方向 = A/B 同向同量"给脉冲，口径清楚、好算 */
    pulse  = mm * (int32_t)ppm;
    est_ms = Kinematics_MoveTimeMs(pulse, 30.0f);

    Dbg_Printf("[INFO] 标定走 X%+dmm @ 假设 %u 脉冲/mm → A/B 各 %u 脉冲（同向）\n", (int)mm,
               (unsigned)ppm, (unsigned)((pulse >= 0) ? pulse : -pulse));

    ok_a = Zdt_Bus_MoveMotor(DBG_MOTOR_A, pulse, Dbg_App_SpeedField(30.0f), DBG_APP_ACCEL);
    ok_b = Zdt_Bus_MoveMotor(DBG_MOTOR_B, pulse, Dbg_App_SpeedField(30.0f), DBG_APP_ACCEL);
    if (ok_a == 0u || ok_b == 0u) {
        (void)Zdt_Bus_StopBoth();
        Dbg_Printf("[FAIL] 帧没发全，已停双轴\n");
        return 0u;
    }

    t0 = HAL_GetTick();
    if (Dbg_App_WaitMsStop(est_ms) != 0u) {
        (void)Zdt_Bus_StopBoth();
        Dbg_Printf("[WARN] 软急停（控制台字节）——标定走中止于 %ums\n",
                   (unsigned)(HAL_GetTick() - t0));
        return 0u;
    }
    Dbg_Printf("[OK] 走完 %u ms。用卷尺量实际位移 D，则新 ppm = %u × %d / D\n", (unsigned)est_ms,
               (unsigned)ppm, (int)mm);
    return 1u;
}

/* 运行时改 A/B/T 的登记地址（台架换电机是常事）：改完立刻探一次，省得改错了不知道。
   which 0=A 1=B 2=T；转盘允许 0（=恢复产品口径的广播地址） */
uint8_t Dbg_App_SetMotorAddr(uint8_t which, uint8_t addr) {
    Dbg_Motor_e motor;

    if (which == 0u) {
        motor = DBG_MOTOR_A;
    } else if (which == 1u) {
        motor = DBG_MOTOR_B;
    } else if (which == 2u) {
        motor = DBG_MOTOR_TRAY;
    } else {
        Dbg_Printf("[FAIL] 只能改 a / b / t 三台\n");
        return 0u;
    }

    if (addr == 0u) {
        Dbg_Printf("[OK] %s 的地址 -> 0（广播 = 运动常驻口径）\n", Zdt_Bus_NameOf(motor));
    } else {
        Dbg_Printf("[OK] %s 的地址 -> %u（仅影响单播诊断帧的匹配；运动/停止一律广播）\n",
                   Zdt_Bus_NameOf(motor), (unsigned)addr);
    }
    Zdt_Bus_SetAddr(motor, addr);
    Dbg_App_AskAck((uint8_t)motor, addr);
    return 1u;
}

/* 逐 ID 探测**指定线**上的驱动器：有回包的那个 ID 就是这条线上那台的地址。
   只发使能帧，全程没有任何运动指令 */
void Dbg_App_ScanId(uint8_t motor, uint8_t id_max) {
    if (id_max == 0u || id_max > 32u) {
        id_max = 8u;
    }

    Dbg_Printf("[SCAN] %s线：逐 ID 发使能帧，看谁应答（有回包的那个 ID 就是这台驱动器的地址）\n",
               Zdt_Bus_NameOf((Dbg_Motor_e)motor));
    for (uint8_t id = 1u; id <= id_max; id++) {
        Dbg_Uart_MotorFlush(motor);
        (void)Zdt_Bus_EnableOn(motor, id, 1u);
        Dbg_App_AckReport(motor, "探测", id);
    }
    Dbg_Printf("[SCAN] 结束：把应答过的地址 `id %s <地址>` 写进去，或等下次上电自检自动采纳\n",
               Zdt_Bus_NameOf((Dbg_Motor_e)motor));
    Dbg_Printf("[SCAN] 一个都不应答时用手拧法：逐个 `en <ID>`，哪台抱死就是哪个 ID（见 上台架步骤.md §3）\n");
}

void Dbg_App_MeasureFrame(uint8_t motor, uint8_t addr, uint16_t repeat) {
    uint32_t cyc_per_us = SystemCoreClock / 1000000u;
    uint32_t total      = 0u;
    uint8_t  frame[16];
    uint16_t len;

    if (repeat == 0u) {
        repeat = 10u;
    }

    /* 只构帧不发：测的是"这一帧在 115200 上真正占用的时间"，电机不会动 */
    len = Zdt_Bus_BuildPosition(addr, 3200, 30u, DBG_APP_ACCEL, frame);
    if (len == 0u) {
        Dbg_Printf("[FAIL] 构帧失败\n");
        return;
    }

    /* DWT 周期计数器：µs 级测量 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    Dbg_Printf("[TIM] %s线连发 %u 次位置帧（%u 字节）到地址 %u\n",
               Zdt_Bus_NameOf((Dbg_Motor_e)motor), (unsigned)repeat, (unsigned)len, (unsigned)addr);
    for (uint16_t i = 0u; i < repeat; i++) {
        uint32_t t0 = DWT->CYCCNT;

        (void)Dbg_Uart_SendMotor(motor, frame, len);
        total += (DWT->CYCCNT - t0);
    }

    Dbg_Printf("[TIM] 单帧平均 %u us（理论 %u 字节@115200 = %u us，含 HAL 开销）\n",
               (unsigned)(total / repeat / cyc_per_us), (unsigned)len,
               (unsigned)(len * 10u * 1000000u / 115200u));
}

/* ★ 读帧位取目标（`read [等待s]`，2026-10-01）——台架版的产品 Sort_PrepareRead + 取帧：
   走到投放点上方（= 读帧位，不遮挡相机；归零/待机位正压托盘上方，那时读到的坐标不可信）
   → 静默 CFG_VISION_SETTLE_MS（物块刚被拿走，上位机要 ≤2s 换目标）
   → 清一次缓存（丢掉挡相机期间产生的旧帧）→ 等一帧"新鲜"目标。
   ★ 作为抓取依据的坐标必须出自本命令；`k230` 快照随时可看，但不承担读帧纪律。
   目标不消费：同一目标重复 read 会重复给出，拿走货/上位机换目标后自然更新。
   任意键 = 中止等待（本命令只读不抓，不动爪不停车）
   ★ 2026-10-08 分诊（现场"夹爪不去抓 / 无新鲜帧"）：本命令的"先清缓存、再等新帧"就是产品
     Sort_PrepareRead 的同款口径。现场口径 = **上位机从上电起持续发流** ⇒ 机器报"没有可用的
     目标帧"时，只可能是三选一（不预设谁的问题）：①流里是状态 0x00 的帧（上位机在说"本帧无目标"）
     ②流里有目标帧但类别/形状越界（本机按设计一律丢弃：宁可不抓，绝不放错盒）
     ③帧没到本机（校验/帧长/波特率/接线）。
     所以清缓存前先记一笔"本来有没有目标 + 它多久前到的"，超时后按四种指纹给判词；
     再配 `k230`（链路 + 三类计数）与 `k230 raw`（逐帧 hex/解码/校验判决）一次定死 */
uint8_t Dbg_App_ReadTarget(uint16_t wait_s) {
    K230_Target_t before; /* 清缓存前缓存里的目标（可能没有） */
    K230_Target_t t;
    float         mx = 0.0f;
    float         my = 0.0f;
    uint32_t      t0;
    uint32_t      next_report;
    uint32_t      none0;
    uint32_t      bad0;
    uint32_t      before_age = 0u;
    uint8_t       before_had;
    uint8_t       dead_printed = 0u;

    if (wait_s == 0u) {
        wait_s = 10u;
    }
    if (wait_s > 120u) {
        wait_s = 120u;
    }

    if (Dbg_App_XyTrusted() == 0u) {
        Dbg_Printf("[FAIL] 位置记账不可信——先归零（上电 3s 不敲键自动归零，或敲 `home`）\n");
        return 0u;
    }
    if (Dbg_App_Goto(DBG_CFG_DROP_X_MM, DBG_CFG_DROP_Y_MM, 0u) == 0u) {
        Dbg_Printf("[FAIL] 走不到读帧位（投放点 %d,%d 被拒）\n", (int)DBG_CFG_DROP_X_MM,
                   (int)DBG_CFG_DROP_Y_MM);
        return 0u;
    }
    if (Dbg_App_WaitMsStop(CFG_VISION_SETTLE_MS) != 0u) {
        Dbg_Printf("[WARN] 静默期被中止\n");
        return 0u;
    }

    /* ★ 清缓存前先记一笔：区分"本来就有目标（流里刚有、现在断了）"与"一直没有目标"（见函数头分诊） */
    before_had = Dbg_Vision_TargetSnapshot(&before, &before_age);

    Dbg_Vision_Flush(); /* 旧帧作废：挡相机期间产生的坐标不能用 */

    Dbg_Printf("[READ] 已到读帧位 (%d,%d) + 静默 %ums，缓存已清，等目标 ≤%us（任意键中止）\n",
               (int)DBG_CFG_DROP_X_MM, (int)DBG_CFG_DROP_Y_MM, (unsigned)CFG_VISION_SETTLE_MS,
               (unsigned)wait_s);
    if (before_had != 0u) {
        Dbg_Printf("[READ] （清缓存前缓存里有：类别 %u，%ums 前）\n", (unsigned)before.category,
                   (unsigned)before_age);
    }

    t0          = HAL_GetTick();
    next_report = t0 + 1000u;
    none0       = Dbg_Vision_NoneCount();
    bad0        = Dbg_Vision_BadCount();

    for (;;) {
        if (Dbg_Uart_Available() > 0u) { /* 只排空不执行：中止等待不算软急停 */
            while (Dbg_Uart_Available() > 0u) {
                (void)Dbg_Uart_ReadByte();
            }
            Dbg_Printf("[READ] 已中止\n");
            return 0u;
        }

        Dbg_Vision_Poll(); /* 阻塞等待期间必须自己泵视觉线（主循环没在跑） */

        if (Dbg_Vision_GetTarget(&t) != 0u) {
            uint32_t age = 0u;

            (void)Dbg_Vision_TargetSnapshot(&t, &age); /* 同一个缓存条目：取它的"到帧年龄" */
            Dbg_Vision_ToMachine(&t, &mx, &my);
            Dbg_Printf("[READ] ✓ 目标：类别 %u 形状 %u 数量 %u（本帧 %ums 前到）  视觉(",
                       (unsigned)t.category, (unsigned)t.shape, (unsigned)t.count,
                       (unsigned)age);
            Dbg_App_PrintF1(t.x_mm);
            Dbg_Printf(", ");
            Dbg_App_PrintF1(t.y_mm);
            Dbg_Printf(") -> 机械(");
            Dbg_App_PrintF1(mx);
            Dbg_Printf(", ");
            Dbg_App_PrintF1(my);
            Dbg_Printf(")\n");
            Dbg_Printf("[READ] 下一步：`pick %d %d 100 %u` → `tb %u` → `drop`"
                       "（pick 末参=形状选夹持方案；tb 参数=本类别号，映射已含）\n",
                       (int)((mx >= 0.0f) ? (mx + 0.5f) : (mx - 0.5f)),
                       (int)((my >= 0.0f) ? (my + 0.5f) : (my - 0.5f)), (unsigned)t.shape,
                       (unsigned)t.category);

            /* ★ 2026-10-08：预演**产品**会不会要这个目标（现场"机器不动 / 到不了夹持点"
               最常见的一条）。判据 = 产品 Sort_DoCycle 的决策树，与 dbg_config.h
               "产品工作包络与够得着就去钳位"一节逐条同序：-X 延伸带到 -160、+侧极限位夹取带
               25mm、保护边距 3mm（2026-10-10 5→3）⇒ 只有**带外**才整件作废。
               ★ 台架 `p`/`pick` 现在用**同一套钳位**（见 Dbg_App_Goto）⇒ 打印钳位后的实际
                 落点，并给一条可直接照抄的 `pick`（现场"台架拒绝、产品会夹"对不上的坑） */
            {
                float   cx  = mx;
                float   cy  = my;
                uint8_t bad = 0u;

                /* 与 `p`/`pick` 走**同一个函数**（Dbg_App_ProdClamp）——
                   这份预演不再自己抄一遍规则（2026-10-10 去重） */
                bad = (Dbg_App_ProdClamp(&cx, &cy) == 0u) ? 1u : 0u;

                if (bad != 0u) {
                    Dbg_Printf("[READ] ⚠️ 该目标**超出产品可达范围**——工作区机械 X∈[%d,%d]、"
                               "Y∈[%d,%d]；-X 延伸带到 %d、+侧极限位夹取带 %dmm、保护边距 %dmm\n",
                               (int)DBG_CFG_PROD_WORK_X_MIN, (int)DBG_CFG_PROD_WORK_X_MAX,
                               (int)DBG_CFG_PROD_WORK_Y_MIN, (int)DBG_CFG_PROD_WORK_Y_MAX,
                               (int)DBG_CFG_PROD_X_CLAMP_MM, (int)DBG_CFG_PROD_CLAMP_BAND_MM,
                               (int)DBG_CFG_PROD_WORK_EDGE_MARGIN);
                    Dbg_Printf("[READ] ⚠️ ⇒ 产品固件会**按设计作废本件、机器一动不动**"
                               "（不是取帧/新鲜度问题）：先看物块是不是放在够不着的边上/托盘外，\n"
                               "         再核视觉→机械映射（原点 CFG_VISION_ORIGIN_*／符号 SIGN_*／"
                               "视觉侧比例）\n");
                } else {
                    Dbg_Printf("[READ] 产品/台架都会去抓：钳位后实际落点 (");
                    Dbg_App_PrintF1(cx);
                    Dbg_Printf(", ");
                    Dbg_App_PrintF1(cy);
                    Dbg_Printf(")");
                    if ((cx != mx) || (cy != my)) {
                        Dbg_Printf("（贴边钳位：+侧 ≤%dmm 钳到上限-保护边距 %dmm、"
                                   "-X ≤%d 钳到 %d）——爪落极限位，可能夹偏/抓空\n",
                                   (int)DBG_CFG_PROD_CLAMP_BAND_MM,
                                   (int)DBG_CFG_PROD_WORK_EDGE_MARGIN,
                                   (int)DBG_CFG_PROD_X_CLAMP_MM, (int)DBG_CFG_PROD_WORK_X_MIN);
                    } else {
                        Dbg_Printf("（坐标在区内，不钳位）\n");
                    }
                    Dbg_Printf("[READ] 照抄这条即可（台架与产品同口径）："
                               "`pick %d %d 100 %u`\n",
                               (int)((cx >= 0.0f) ? (cx + 0.5f) : (cx - 0.5f)),
                               (int)((cy >= 0.0f) ? (cy + 0.5f) : (cy - 0.5f)),
                               (unsigned)t.shape);
                }
            }
            return 1u;
        }

        if ((Dbg_Vision_LinkAlive() == 0u) && (dead_printed == 0u)) {
            dead_printed = 1u;
            Dbg_Printf("[READ] 链路无流量（>1s 无校验通过帧）——查接线/波特率/上位机在不在发；继续等\n");
        }

        if ((int32_t)(HAL_GetTick() - next_report) >= 0) {
            next_report += 1000u;
            Dbg_Printf("[READ] …等待中 无目标+%u 越界+%u 链路=%s\n",
                       (unsigned)(Dbg_Vision_NoneCount() - none0),
                       (unsigned)(Dbg_Vision_BadCount() - bad0),
                       (Dbg_Vision_LinkAlive() != 0u) ? "活" : "断");
        }

        if ((HAL_GetTick() - t0) > ((uint32_t)wait_s * 1000u)) {
            uint32_t none = Dbg_Vision_NoneCount() - none0;
            uint32_t bad  = Dbg_Vision_BadCount() - bad0;

            Dbg_Printf("[FAIL] %us 内没有新鲜目标（无目标 %u 帧 / 越界 %u 帧 / 链路=%s）\n",
                       (unsigned)wait_s, (unsigned)none, (unsigned)bad,
                       (Dbg_Vision_LinkAlive() != 0u) ? "活" : "断");
            if (before_had != 0u) {
                /* 清缓存前有目标、之后一帧没有：说明"这一小段流里没有可用目标帧"了 ——
                   不预设是谁的问题，把三种可能一起摆出来，让 k230/k230 raw 去定死 */
                Dbg_Printf("[FAIL] ★ 清缓存前缓存里{有}目标（类别 %u，%ums 前），清缓存后一帧新的都没有\n"
                           "         ⇒ 这一小段流里不再有可用目标帧。三种可能（`k230` 三类计数、`k230 raw` 逐帧 hex 定死）：\n"
                           "         ① 上位机改成在说\"本帧无目标\"（0x00）＝它不再把这一件当目标\n"
                           "         ② 之后的帧报了目标、但类别/形状越界（本机按设计丢弃）\n"
                           "         ③ 帧干脆不来了（链路）；*上位机若是\"发固定帧数就停\"的实现，一串播完也落到这里*\n",
                           (unsigned)before.category, (unsigned)before_age);
            } else if (none > 0u) {
                Dbg_Printf("[FAIL] ★ 上位机在发\"本帧无目标\"（0x00）——物块在托盘上却没被当成目标\n"
                           "       ⇒ 识别/目标挑选问题（典型：被龙门挡过之后没把物块认回来）；\n"
                           "         `k230 live 20` 盯着看：一帧目标都没有就是它\n");
            } else if (bad > 0u) {
                Dbg_Printf("[FAIL] ★ 上位机在报目标，但帧**越界**（类别须 1~%u、形状须 0~%u）\n"
                           "       ⇒ 两边编码口径不一致（不是没识别到）：查上位机的类别/形状编号\n"
                           "         （类别 = 屏上行号 = 盒号 1~%u；形状 0~3：0=默认方案）\n",
                           (unsigned)CFG_NUM_CATEGORIES, (unsigned)CFG_NUM_SHAPES,
                           (unsigned)CFG_NUM_CATEGORIES);
            } else {
                Dbg_Printf("[FAIL] ★ 线上一帧都没有（链路断）——查接线/波特率/上位机进程在不在发；\n"
                           "         想确认接收侧：`k230 sim 3 60 45` 本地注一帧应能立刻看到\n");
            }
            return 0u;
        }
    }
}
