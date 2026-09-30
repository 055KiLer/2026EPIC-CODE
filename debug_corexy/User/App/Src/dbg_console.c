/**
 * @file    dbg_console.c
 * @brief   串口命令控制台：行缓冲 -> 分词 -> 分派
 * @note    不做逐字符回显（多数串口终端自带本地回显，两边都回显会出现重影）；
 *          收到整行后打印 `> 原样` 让你核对板上实际收到了什么。
 */
#include "dbg_console.h"
#include "dbg_config.h"
#include "dbg_gpio.h"
#include "dbg_print.h"
#include "dbg_uart.h"
#include "dbg_app.h"
#include "servo_out.h"
#include "sort_config.h" /* CFG_TRAY_BINS：tb 的盒号范围 */
#include "zdt_bus.h"

/* 分词上限：`send` 一帧最多 32 字节 = 32 个 token + 命令名，留够余量 */
#define CMD_MAX_TOKEN 40u

typedef uint8_t (*Dbg_Cmd_Fn)(char *tok[], uint8_t n);
typedef struct {
    const char *name;
    const char *args;
    const char *help;
    Dbg_Cmd_Fn  fn;
} Dbg_Cmd_t;

static char s_line[DBG_CFG_CMD_MAX];
static uint8_t s_len;
static uint8_t s_got_any; /* 上电以来有没有收到过任何字节（自动体检用它决定要不要重跑） */

/* ---------------- 小工具 ---------------- */

static uint8_t Dbg_Cmd_StrEq(const char *a, const char *b) {
    while ((*a != '\0') && (*b != '\0')) {
        if (*a != *b) {
            return 0u;
        }
        a++;
        b++;
    }
    return (*a == *b) ? 1u : 0u;
}

/* 严格解析十进制整数：非法/溢出返回 0，不静默当成 0 */
static uint8_t Dbg_Cmd_ParseI32(const char *s, int32_t *out) {
    int32_t value = 0;
    uint8_t neg   = 0u;
    uint8_t digit = 0u;

    if (s == NULL || *s == '\0') {
        return 0u;
    }
    if (*s == '-') {
        neg = 1u;
        s++;
    } else if (*s == '+') {
        s++;
    }

    while (*s != '\0') {
        if ((*s < '0') || (*s > '9')) {
            return 0u;
        }
        if (value > ((2147483647 - (*s - '0')) / 10)) {
            return 0u; /* 溢出 */
        }
        value = (value * 10) + (int32_t)(*s - '0');
        digit = 1u;
        s++;
    }

    if (digit == 0u) {
        return 0u;
    }

    *out = neg ? -value : value;
    return 1u;
}

/* 地址参数：all（按配置的两台）/ 0 或 bc（广播 0x00）/ a / b / 1~255 */
static uint8_t Dbg_Cmd_ParseAddr(const char *s, uint8_t *addr, uint8_t *is_both) {
    int32_t value = 0;

    *is_both = 0u;

    if (Dbg_Cmd_StrEq(s, "all") != 0u) {
        *is_both = 1u; /* 按配置的 A/B 两个地址分别发 */
        *addr    = 0u;
        return 1u;
    }
    if ((Dbg_Cmd_StrEq(s, "bc") != 0u) || (Dbg_Cmd_StrEq(s, "bcast") != 0u)) {
        *addr = (uint8_t)DBG_CFG_ADDR_BROADCAST;
        return 1u;
    }
    if ((Dbg_Cmd_StrEq(s, "a") != 0u) || (Dbg_Cmd_StrEq(s, "A") != 0u)) {
        *addr = Zdt_Bus_AddrOf(DBG_MOTOR_A);
        return 1u;
    }
    if ((Dbg_Cmd_StrEq(s, "b") != 0u) || (Dbg_Cmd_StrEq(s, "B") != 0u)) {
        *addr = Zdt_Bus_AddrOf(DBG_MOTOR_B);
        return 1u;
    }
    if ((Dbg_Cmd_ParseI32(s, &value) != 0u) && (value >= 0) && (value <= 255)) {
        *addr = (uint8_t)value; /* 0 = 广播 */
        return 1u;
    }
    return 0u;
}

static uint8_t Dbg_Cmd_ParseServo(const char *s, Dbg_Servo_e *servo) {
    if (Dbg_Cmd_StrEq(s, "claw") != 0u) {
        *servo = DBG_SERVO_CLAW;
        return 1u;
    }
    if (Dbg_Cmd_StrEq(s, "lift") != 0u) {
        *servo = DBG_SERVO_LIFT;
        return 1u;
    }
    return 0u;
}

static uint16_t Dbg_Cmd_ParseU16(const char *s, uint8_t *ok) {
    int32_t value = 0;

    if ((Dbg_Cmd_ParseI32(s, &value) == 0u) || (value < 0) || (value > 65535)) {
        *ok = 0u;
        return 0u;
    }
    *ok = 1u;
    return (uint16_t)value;
}

/* ---------------- 命令实现 ---------------- */

static uint8_t Dbg_Cmd_En(char *tok[], uint8_t n) {
    uint8_t addr    = 0u;
    uint8_t is_both = 0u;

    if (n < 2u) {
        Dbg_Printf("[FAIL] 用法：en <all|0广播|a|b|t|地址>（不知道 ID 时用 en 0）\n");
        return 0u;
    }
    if ((Dbg_Cmd_StrEq(tok[1], "a") != 0u) || (Dbg_Cmd_StrEq(tok[1], "A") != 0u)) {
        return Dbg_App_EnableOne((uint8_t)DBG_MOTOR_A, 1u);
    }
    if ((Dbg_Cmd_StrEq(tok[1], "b") != 0u) || (Dbg_Cmd_StrEq(tok[1], "B") != 0u)) {
        return Dbg_App_EnableOne((uint8_t)DBG_MOTOR_B, 1u);
    }
    if ((Dbg_Cmd_StrEq(tok[1], "t") != 0u) || (Dbg_Cmd_StrEq(tok[1], "T") != 0u)) {
        return Dbg_App_EnableOne((uint8_t)DBG_MOTOR_TRAY, 1u);
    }
    if (Dbg_Cmd_ParseAddr(tok[1], &addr, &is_both) == 0u) {
        Dbg_Printf("[FAIL] 地址参数不合法：%s\n", tok[1]);
        return 0u;
    }
    return (is_both != 0u) ? Dbg_App_EnableBoth(1u) : Dbg_App_EnableAddr(addr, 1u);
}

static uint8_t Dbg_Cmd_Dis(char *tok[], uint8_t n) {
    uint8_t addr    = 0u;
    uint8_t is_both = 0u;

    if (n < 2u) {
        Dbg_Printf("[FAIL] 用法：dis <all|0广播|a|b|t|地址>\n");
        return 0u;
    }
    if ((Dbg_Cmd_StrEq(tok[1], "a") != 0u) || (Dbg_Cmd_StrEq(tok[1], "A") != 0u)) {
        return Dbg_App_EnableOne((uint8_t)DBG_MOTOR_A, 0u);
    }
    if ((Dbg_Cmd_StrEq(tok[1], "b") != 0u) || (Dbg_Cmd_StrEq(tok[1], "B") != 0u)) {
        return Dbg_App_EnableOne((uint8_t)DBG_MOTOR_B, 0u);
    }
    if ((Dbg_Cmd_StrEq(tok[1], "t") != 0u) || (Dbg_Cmd_StrEq(tok[1], "T") != 0u)) {
        return Dbg_App_EnableOne((uint8_t)DBG_MOTOR_TRAY, 0u);
    }
    if (Dbg_Cmd_ParseAddr(tok[1], &addr, &is_both) == 0u) {
        Dbg_Printf("[FAIL] 地址参数不合法：%s\n", tok[1]);
        return 0u;
    }
    return (is_both != 0u) ? Dbg_App_EnableBoth(0u) : Dbg_App_EnableAddr(addr, 0u);
}

static uint8_t Dbg_Cmd_Stop(char *tok[], uint8_t n) {
    (void)tok;
    (void)n;
    return Dbg_App_StopBoth();
}

static uint8_t Dbg_Cmd_StopTest(char *tok[], uint8_t n) {
    (void)tok;
    (void)n;
    return Dbg_App_StopTest();
}

/* `j`/`tim` 的电机选择参数：a/b/t = 指定那条线（用登记地址）；0~255 = 数字地址
   （A/B 两线同发，只有 ID 匹配的那台执行；0 = 广播）。返回 0 = 不是合法选择 */
static uint8_t Dbg_Cmd_ParseMotor(const char *s, int8_t *motor, uint8_t *addr) {
    int32_t value = 0;

    if ((Dbg_Cmd_StrEq(s, "a") != 0u) || (Dbg_Cmd_StrEq(s, "A") != 0u)) {
        *motor = 0;
        *addr  = Zdt_Bus_AddrOf(DBG_MOTOR_A);
        return 1u;
    }
    if ((Dbg_Cmd_StrEq(s, "b") != 0u) || (Dbg_Cmd_StrEq(s, "B") != 0u)) {
        *motor = 1;
        *addr  = Zdt_Bus_AddrOf(DBG_MOTOR_B);
        return 1u;
    }
    if ((Dbg_Cmd_StrEq(s, "t") != 0u) || (Dbg_Cmd_StrEq(s, "T") != 0u)) {
        *motor = (int8_t)DBG_MOTOR_TRAY;
        *addr  = Zdt_Bus_AddrOf(DBG_MOTOR_TRAY);
        return 1u;
    }
    if ((Dbg_Cmd_ParseI32(s, &value) != 0u) && (value >= 0) && (value <= 255)) {
        *motor = -1; /* A/B 两线同发 */
        *addr  = (uint8_t)value;
        return 1u;
    }
    return 0u;
}

static uint8_t Dbg_Cmd_Jog(char *tok[], uint8_t n) {
    int8_t   motor = -1;
    uint8_t  addr = 0u;
    int32_t  rpm  = 0;
    int32_t  pulse = 0;
    int32_t  accel = (int32_t)DBG_CFG_ACCEL_DEFAULT;
    uint8_t  ccw;

    if (n < 5u) {
        Dbg_Printf("[FAIL] 用法：j <a|b|地址> <cw|ccw> <rpm> <pulse> [accel]\n");
        return 0u;
    }
    if (Dbg_Cmd_ParseMotor(tok[1], &motor, &addr) == 0u) {
        Dbg_Printf("[FAIL] 电机选择不合法（a / b / 0~255 数字地址）\n");
        return 0u;
    }
    if (Dbg_Cmd_StrEq(tok[2], "cw") != 0u) {
        ccw = 0u;
    } else if (Dbg_Cmd_StrEq(tok[2], "ccw") != 0u) {
        ccw = 1u;
    } else {
        Dbg_Printf("[FAIL] 方向只能是 cw / ccw\n");
        return 0u;
    }
    if ((Dbg_Cmd_ParseI32(tok[3], &rpm) == 0u) || (rpm < 1) || (rpm > 300)) {
        Dbg_Printf("[FAIL] 转速范围 1~300 rpm\n");
        return 0u;
    }
    if ((Dbg_Cmd_ParseI32(tok[4], &pulse) == 0u) || (pulse < 1)) {
        Dbg_Printf("[FAIL] 脉冲数必须是正整数\n");
        return 0u;
    }
    if (n >= 6u) {
        int32_t parsed = 0;

        /* 解析失败或越界都回默认值：直接截断成 uint8_t 会把 300 静默变成 44 */
        if ((Dbg_Cmd_ParseI32(tok[5], &parsed) == 0u) || (parsed < 0) || (parsed > 255)) {
            Dbg_Printf("[WARN] accel 不合法，用默认 %u\n", (unsigned)DBG_CFG_ACCEL_DEFAULT);
        } else {
            accel = parsed;
        }
    }

    return Dbg_App_Jog(motor, (uint8_t)addr, ccw, (uint16_t)rpm, (uint8_t)accel, (uint32_t)pulse);
}

static uint8_t Dbg_Cmd_Move1(char *tok[], uint8_t n, uint8_t axis_y) {
    int32_t mm    = 0;
    int32_t speed = (int32_t)DBG_CFG_DEFAULT_SPEED_MM_S;

    if (n < 2u) {
        Dbg_Printf("[FAIL] 用法：%s <mm> [mm/s]\n", axis_y ? "y" : "x");
        return 0u;
    }
    if (Dbg_Cmd_ParseI32(tok[1], &mm) == 0u) {
        Dbg_Printf("[FAIL] 位移不合法\n");
        return 0u;
    }
    if ((n >= 3u) && (Dbg_Cmd_ParseI32(tok[2], &speed) == 0u)) {
        Dbg_Printf("[FAIL] 速度不合法\n");
        return 0u;
    }
    if (speed < 1) {
        speed = (int32_t)DBG_CFG_DEFAULT_SPEED_MM_S;
    }

    return axis_y ? Dbg_App_MoveXY(0, mm, (uint16_t)speed) : Dbg_App_MoveXY(mm, 0, (uint16_t)speed);
}

static uint8_t Dbg_Cmd_X(char *tok[], uint8_t n) {
    return Dbg_Cmd_Move1(tok, n, 0u);
}

static uint8_t Dbg_Cmd_Y(char *tok[], uint8_t n) {
    return Dbg_Cmd_Move1(tok, n, 1u);
}

static uint8_t Dbg_Cmd_Xy(char *tok[], uint8_t n) {
    int32_t dx    = 0;
    int32_t dy    = 0;
    int32_t speed = (int32_t)DBG_CFG_DEFAULT_SPEED_MM_S;

    if (n < 3u) {
        Dbg_Printf("[FAIL] 用法：xy <dx_mm> <dy_mm> [mm/s]\n");
        return 0u;
    }
    if ((Dbg_Cmd_ParseI32(tok[1], &dx) == 0u) || (Dbg_Cmd_ParseI32(tok[2], &dy) == 0u)) {
        Dbg_Printf("[FAIL] 位移不合法\n");
        return 0u;
    }
    if ((n >= 4u) && (Dbg_Cmd_ParseI32(tok[3], &speed) == 0u)) {
        Dbg_Printf("[FAIL] 速度不合法\n");
        return 0u;
    }
    if (speed < 1) {
        speed = (int32_t)DBG_CFG_DEFAULT_SPEED_MM_S;
    }

    return Dbg_App_MoveXY(dx, dy, (uint16_t)speed);
}

static uint8_t Dbg_Cmd_Bx(char *tok[], uint8_t n) {
    int32_t mm  = 0;
    int32_t rpm = 30;

    if (n < 2u) {
        Dbg_Printf("[FAIL] 用法：bx <mm> [rpm]（广播：两台同向同量 = 纯 X 平移）\n");
        return 0u;
    }
    if (Dbg_Cmd_ParseI32(tok[1], &mm) == 0u) {
        Dbg_Printf("[FAIL] 位移不合法\n");
        return 0u;
    }
    if ((n >= 3u) && (Dbg_Cmd_ParseI32(tok[2], &rpm) == 0u)) {
        Dbg_Printf("[FAIL] 转速不合法\n");
        return 0u;
    }
    if ((rpm < 1) || (rpm > 300)) {
        rpm = 30;
    }

    return Dbg_App_BroadcastMove(mm, (uint16_t)rpm);
}

static uint8_t Dbg_Cmd_Sq(char *tok[], uint8_t n) {
    int32_t mm    = 0;
    int32_t speed = (int32_t)DBG_CFG_DEFAULT_SPEED_MM_S;

    if (n < 2u) {
        Dbg_Printf("[FAIL] 用法：sq <mm> [mm/s]\n");
        return 0u;
    }
    if (Dbg_Cmd_ParseI32(tok[1], &mm) == 0u) {
        Dbg_Printf("[FAIL] 边长不合法\n");
        return 0u;
    }
    if ((n >= 3u) && (Dbg_Cmd_ParseI32(tok[2], &speed) == 0u)) {
        Dbg_Printf("[FAIL] 速度不合法\n");
        return 0u;
    }
    if (speed < 1) {
        speed = (int32_t)DBG_CFG_DEFAULT_SPEED_MM_S;
    }

    return Dbg_App_Square(mm, (uint16_t)speed);
}

/* ★ 绝对定位：p 打印位置；p <X> <Y> [mm/s] 走到第三象限坐标（需先归零，坐标为整数 mm） */
static uint8_t Dbg_Cmd_P(char *tok[], uint8_t n) {
    int32_t ix = 0;
    int32_t iy = 0;

    if (n == 1u) {
        Dbg_App_PrintPosition();
        return 1u;
    }
    if ((n < 3u) || (Dbg_Cmd_ParseI32(tok[1], &ix) == 0u) || (Dbg_Cmd_ParseI32(tok[2], &iy) == 0u)) {
        Dbg_Printf("[FAIL] 用法：p（打印位置）/ p <X> <Y> [mm/s]（负数坐标，先归零）\n");
        return 0u;
    }
    if (n >= 4u) {
        int32_t sp = 0;

        if ((Dbg_Cmd_ParseI32(tok[3], &sp) == 0u) || (sp < 0)) {
            Dbg_Printf("[FAIL] 速度不合法\n");
            return 0u;
        }
        return Dbg_App_Goto((float)ix, (float)iy, (uint16_t)sp);
    }
    return Dbg_App_Goto((float)ix, (float)iy, 0u);
}

/* 回零演练：向最大端慢速走，限位裸沿触发即停 + 电平复核 + 退离 */
static uint8_t Dbg_Cmd_Home(char *tok[], uint8_t n, uint8_t axis_y) {
    int32_t speed = 0;

    if ((n >= 2u) && ((Dbg_Cmd_ParseI32(tok[1], &speed) == 0u) || (speed < 0))) {
        Dbg_Printf("[FAIL] 用法：%s [mm/s]（默认 %u，上限 %u）；一行只跑一条命令——homex 和 homey 请分两行\n", axis_y ? "homey" : "homex",
                   (unsigned)DBG_CFG_HOME_SPEED_DEFAULT, (unsigned)DBG_CFG_HOME_SPEED_MAX);
        return 0u;
    }

    return axis_y ? Dbg_App_HomeY((uint16_t)speed) : Dbg_App_HomeX((uint16_t)speed);
}

static uint8_t Dbg_Cmd_Homey(char *tok[], uint8_t n) {
    return Dbg_Cmd_Home(tok, n, 1u);
}

static uint8_t Dbg_Cmd_Homex(char *tok[], uint8_t n) {
    return Dbg_Cmd_Home(tok, n, 0u);
}

/* 转盘回零演练：tb/thome，见 dbg_app.h 的注释（对齐产品 Motion_TrayHome/GotoBin） */
static uint8_t Dbg_Cmd_Thome(char *tok[], uint8_t n) {
    int32_t seek = 0;

    if ((n >= 2u) && ((Dbg_Cmd_ParseI32(tok[1], &seek) == 0u) || (seek < 0))) {
        Dbg_Printf("[FAIL] 用法：thome [粗寻标rpm]（缺省 = 产品 CFG_TRAY_SEEK_RPM）\n");
        return 0u;
    }
    return Dbg_App_TrayHome((uint16_t)seek);
}

/* ★ 一键到投放点（drop [mm/s]）：坐标记录在 dbg_config.h §4c（2026-09-30 实测） */
static uint8_t Dbg_Cmd_Drop(char *tok[], uint8_t n) {
    int32_t speed = 0;

    if (n >= 2u) {
        if ((Dbg_Cmd_ParseI32(tok[1], &speed) == 0u) || (speed < 0)) {
            Dbg_Printf("[FAIL] 用法：drop [mm/s]（缺省 = 默认速度；投放点在 dbg_config.h §4c）\n");
            return 0u;
        }
    }
    Dbg_Printf("[DROP] 投放点 (%d, %d) —— 转过去（需先 homex + homey）\n", (int)DBG_CFG_DROP_X_MM,
               (int)DBG_CFG_DROP_Y_MM);
    return Dbg_App_Goto(DBG_CFG_DROP_X_MM, DBG_CFG_DROP_Y_MM, (uint16_t)speed);
}

static uint8_t Dbg_Cmd_Tb(char *tok[], uint8_t n) {
    int32_t bin = 0;

    if (n < 2u) {
        Dbg_App_TrayPrint();
        return 1u;
    }
    if (Dbg_Cmd_ParseI32(tok[1], &bin) == 0u) {
        Dbg_Printf("[FAIL] 用法：tb（打印转盘记账）/ tb <盒号0~%u>\n",
                   (unsigned)(CFG_TRAY_BINS - 1u));
        return 0u;
    }
    return Dbg_App_TrayGotoBin((uint8_t)bin);
}

static uint8_t Dbg_Cmd_Cal(char *tok[], uint8_t n) {
    int32_t mm  = 0;
    int32_t ppm = (int32_t)Dbg_App_PulsePerMm();

    if (n < 2u) {
        Dbg_Printf("[FAIL] 用法：cal <mm> [pulses_per_mm]\n");
        return 0u;
    }
    if (Dbg_Cmd_ParseI32(tok[1], &mm) == 0u) {
        Dbg_Printf("[FAIL] 位移不合法\n");
        return 0u;
    }
    if ((n >= 3u) && ((Dbg_Cmd_ParseI32(tok[2], &ppm) == 0u) || (ppm < 1) || (ppm > 10000))) {
        Dbg_Printf("[FAIL] pulses/mm 范围 1~10000\n");
        return 0u;
    }

    return Dbg_App_CalMove(mm, (uint16_t)ppm);
}

static uint8_t Dbg_Cmd_Raw(char *tok[], uint8_t n) {
    Dbg_Servo_e servo;
    uint8_t     ok = 0u;
    uint16_t    us;
    uint16_t    applied;

    if ((n < 3u) || (Dbg_Cmd_ParseServo(tok[1], &servo) == 0u)) {
        Dbg_Printf("[FAIL] 用法：raw <claw|lift> <脉宽us>\n");
        return 0u;
    }
    us = Dbg_Cmd_ParseU16(tok[2], &ok);
    if (ok == 0u) {
        Dbg_Printf("[FAIL] 脉宽不合法\n");
        return 0u;
    }

    applied = Servo_Out_SetUs(servo, us);
    Dbg_Printf("[OK] %s 脉宽 -> %u us（安全窗 %u~%u）\n", Servo_Out_NameOf(servo),
               (unsigned)applied, (unsigned)DBG_CFG_SERVO_MIN_US,
               (unsigned)DBG_CFG_SERVO_MAX_US);
    return 1u;
}

/* 舵机顶死/嗡嗡响/发烫时的第一反应：停这一路 PWM（舵机随即松力） */
static uint8_t Dbg_Cmd_Sstop(char *tok[], uint8_t n) {
    Dbg_Servo_e servo;

    if ((n < 2u) || (Dbg_Cmd_ParseServo(tok[1], &servo) == 0u)) {
        Dbg_Printf("[FAIL] 用法：sstop <claw|lift>（停该路 PWM，舵机松力）\n");
        return 0u;
    }

    Servo_Out_Stop(servo);
    Dbg_Printf("[OK] %s 已停 PWM（松力、失去保持力）。再发任何档位/raw 命令会自动重新起\n",
               Servo_Out_NameOf(servo));
    return 1u;
}

/* 位置帧格式切换：13 = Emm_V5.0 老协议，16 = V2 说明书协议（0.1°/0.1RPM） */
static uint8_t Dbg_Cmd_Fmt(char *tok[], uint8_t n) {
    uint16_t v;
    uint8_t  ok = 0u;

    if (n >= 2u) {
        v = Dbg_Cmd_ParseU16(tok[1], &ok);
        if ((ok == 0u) || (v != 13u && v != 16u)) {
            Dbg_Printf("[FAIL] 用法：fmt <13|16>\n");
            return 0u;
        }
        Zdt_Bus_SetFrame16((v == 16u) ? 1u : 0u);
    }
    Dbg_Printf("[OK] 位置帧 = %u 字节（%s）\n", (unsigned)(Zdt_Bus_Frame16() ? 16u : 13u),
               (Zdt_Bus_Frame16() != 0u) ? "V2 协议：0.1°/0.1RPM，加速度 511RPM/s"
                                         : "老协议：脉冲/RPM");
    return 1u;
}

static uint8_t Dbg_Cmd_Pos(char *tok[], uint8_t n, Dbg_Servo_e servo) {
    int32_t pos = 0;

    if (n < 2u) {
        Dbg_Printf("[FAIL] 用法：%s <档位0~%u>\n", Servo_Out_NameOf(servo),
                   (unsigned)(Servo_Out_PosCount(servo) - 1u));
        return 0u;
    }
    if ((Dbg_Cmd_ParseI32(tok[1], &pos) == 0u) || (pos < 0) ||
        (pos >= (int32_t)Servo_Out_PosCount(servo))) {
        Dbg_Printf("[FAIL] 档位范围 0~%u\n", (unsigned)(Servo_Out_PosCount(servo) - 1u));
        return 0u;
    }
    if (Servo_Out_SetPos(servo, (uint8_t)pos) == 0u) {
        Dbg_Printf("[FAIL] 档位下发失败\n");
        return 0u;
    }

    Dbg_Printf("[OK] %s -> 档位 %u (%s) = %u us\n", Servo_Out_NameOf(servo), (unsigned)pos,
               Servo_Out_PosNameOf(servo, (uint8_t)pos), (unsigned)Servo_Out_CurrentUs(servo));
    return 1u;
}

static uint8_t Dbg_Cmd_G(char *tok[], uint8_t n) {
    return Dbg_Cmd_Pos(tok, n, DBG_SERVO_CLAW);
}

static uint8_t Dbg_Cmd_L(char *tok[], uint8_t n) {
    return Dbg_Cmd_Pos(tok, n, DBG_SERVO_LIFT);
}

/* id <a|b> <地址>：运行时改电机地址（台架换电机/换 ID 用，免重烧） */
static uint8_t Dbg_Cmd_Id(char *tok[], uint8_t n) {
    int32_t addr = 0;
    uint8_t which;

    if (n < 3u) {
        Dbg_Printf("[FAIL] 用法：id <a|b|t> <地址>（0=广播；仅影响单播诊断帧，运动一律广播）\n");
        return 0u;
    }
    if (Dbg_Cmd_StrEq(tok[1], "a") != 0u) {
        which = 0u;
    } else if (Dbg_Cmd_StrEq(tok[1], "b") != 0u) {
        which = 1u;
    } else if (Dbg_Cmd_StrEq(tok[1], "t") != 0u) {
        which = 2u;
    } else {
        Dbg_Printf("[FAIL] 只能写 a / b / t\n");
        return 0u;
    }
    if ((Dbg_Cmd_ParseI32(tok[2], &addr) == 0u) || (addr < 0) || (addr > 255)) {
        Dbg_Printf("[FAIL] 地址范围 0~255（0 = 广播，运动常驻口径）\n");
        return 0u;
    }

    return Dbg_App_SetMotorAddr(which, (uint8_t)addr);
}

static uint8_t Dbg_Cmd_Scan(char *tok[], uint8_t n) {
    int32_t id_max = 8;

    if ((n >= 2u) && (Dbg_Cmd_ParseI32(tok[1], &id_max) == 0u)) {
        Dbg_Printf("[FAIL] 用法：scan [最大ID，默认 8]\n");
        return 0u;
    }
    Dbg_App_ScanId((uint8_t)DBG_MOTOR_A, (uint8_t)id_max);
    Dbg_App_ScanId((uint8_t)DBG_MOTOR_B, (uint8_t)id_max);
    Dbg_App_ScanId((uint8_t)DBG_MOTOR_TRAY, (uint8_t)id_max);
    return 1u;
}

static uint8_t Dbg_Cmd_Tim(char *tok[], uint8_t n) {
    int8_t   motor = 0;
    uint8_t  addr  = Zdt_Bus_AddrOf(DBG_MOTOR_A);
    int32_t  repeat = 10;

    if (n >= 2u) {
        if (Dbg_Cmd_ParseMotor(tok[1], &motor, &addr) == 0u) {
            Dbg_Printf("[FAIL] 用法：tim [a|b|地址] [次数]\n");
            return 0u;
        }
    }
    if ((n >= 3u) && (Dbg_Cmd_ParseI32(tok[2], &repeat) == 0u)) {
        Dbg_Printf("[FAIL] 次数不合法\n");
        return 0u;
    }
    if (repeat < 1) {
        repeat = 1;
    }
    if (repeat > 1000) {
        repeat = 1000;
    }

    Dbg_App_MeasureFrame((uint8_t)motor, addr, (uint16_t)repeat);
    return 1u;
}

static uint8_t Dbg_Cmd_Baud(char *tok[], uint8_t n) {
    int32_t bps = 0;

    if (n < 2u) {
        Dbg_Printf("[INFO] 当前 A线 %u bps / B线 %u bps。用法：baud <bps>（两条线一起改）\n",
                   (unsigned)Dbg_Uart_MotorBaud(0u), (unsigned)Dbg_Uart_MotorBaud(1u));
        Dbg_Printf("[INFO] ZDT 常用档：9600 19200 38400 57600 115200 230400\n");
        return 0u;
    }
    if ((Dbg_Cmd_ParseI32(tok[1], &bps) == 0u) || (bps <= 0)) {
        Dbg_Printf("[FAIL] 波特率不合法\n");
        return 0u;
    }

    return Dbg_App_SetBusBaud((uint32_t)bps);
}

static uint8_t Dbg_Cmd_Loop(char *tok[], uint8_t n) {
    int32_t repeat = 3;

    if ((n >= 2u) && (Dbg_Cmd_ParseI32(tok[1], &repeat) == 0u)) {
        Dbg_Printf("[FAIL] 用法：loop [次数]\n");
        return 0u;
    }
    if (repeat < 1) {
        repeat = 1;
    }

    Dbg_App_Loopback((uint16_t)repeat);
    return 1u;
}

static uint8_t Dbg_Cmd_Dump(char *tok[], uint8_t n) {
    uint8_t addr    = Zdt_Bus_AddrOf(DBG_MOTOR_A);
    uint8_t is_both = 0u;
    int32_t pulse   = 3200;

    if (n >= 2u) {
        if (Dbg_Cmd_ParseAddr(tok[1], &addr, &is_both) == 0u) {
            Dbg_Printf("[FAIL] 地址不合法\n");
            return 0u;
        }
        if (is_both != 0u) {
            addr = Zdt_Bus_AddrOf(DBG_MOTOR_A); /* dump 只针对一个地址，all 就按 A */
        }
    }
    if ((n >= 3u) && (Dbg_Cmd_ParseI32(tok[2], &pulse) == 0u)) {
        Dbg_Printf("[FAIL] 脉冲数不合法\n");
        return 0u;
    }
    if (pulse < 0) {
        pulse = -pulse;
    }

    Dbg_App_DumpFrames(addr, (uint32_t)pulse);
    return 1u;
}

static uint8_t Dbg_Cmd_Cfg(char *tok[], uint8_t n) {
    (void)tok;
    (void)n;
    Dbg_App_PrintConfig();
    return 1u;
}

/* 软复位：重新跑一遍上电自检（改完配置/想重看 [ACK] 时不用手按复位键，
   也让"我这边"能自己重启板子，不必麻烦人到台前按按钮） */
static uint8_t Dbg_Cmd_Rst(char *tok[], uint8_t n) {
    uint32_t t0 = HAL_GetTick();

    (void)tok;
    (void)n;
    Dbg_Printf("[OK] 软复位：立刻重启，控制台会重新打一遍 [CFG]/[HINT]/[ACK] 自检\n");
    Dbg_Print_Flush();
    while ((HAL_GetTick() - t0) < 50u) { /* 等最后一个字节真的发完再复位 */
    }
    NVIC_SystemReset();
    return 1u; /* 走不到这里 */
}

/* ---------------- 新增：回包探测 / 原始字节 / 速度单位系数 ---------------- */

/* 严格两位十六进制（非法就整条拒绝，绝不"猜"半个字节） */
static uint8_t Dbg_Cmd_HexByte(const char *s, uint8_t *out) {
    uint8_t value = 0u;

    for (uint8_t i = 0u; i < 2u; i++) {
        char    c = s[i];
        uint8_t d;

        if ((c >= '0') && (c <= '9')) {
            d = (uint8_t)(c - '0');
        } else if ((c >= 'a') && (c <= 'f')) {
            d = (uint8_t)((c - 'a') + 10);
        } else if ((c >= 'A') && (c <= 'F')) {
            d = (uint8_t)((c - 'A') + 10);
        } else {
            return 0u;
        }
        value = (uint8_t)((value << 4) | d);
    }

    if (s[2] != '\0') {
        return 0u; /* 必须正好两位，"1" 或 "001" 都不收 */
    }
    *out = value;
    return 1u;
}

static uint8_t Dbg_Cmd_Send(char *tok[], uint8_t n) {
    uint8_t buf[32];
    uint8_t len  = 0u;
    uint8_t motor = 0u; /* 默认 A 线 */
    uint8_t first = 1u; /* hex 从第几个 token 开始 */

    /* 可选首参 a|b（单字符，不会与两位十六进制混淆）选线 */
    if (n >= 2u) {
        if (Dbg_Cmd_StrEq(tok[1], "a") != 0u || Dbg_Cmd_StrEq(tok[1], "A") != 0u) {
            motor = 0u;
            first = 2u;
        } else if (Dbg_Cmd_StrEq(tok[1], "b") != 0u || Dbg_Cmd_StrEq(tok[1], "B") != 0u) {
            motor = 1u;
            first = 2u;
        } else if (Dbg_Cmd_StrEq(tok[1], "t") != 0u || Dbg_Cmd_StrEq(tok[1], "T") != 0u) {
            motor = 2u;
            first = 2u;
        }
    }

    if (n <= first) {
        Dbg_Printf("[FAIL] 用法：send [a|b|t] <两位十六进制字节...>，例如 send a 01 F3 AB 01 00 6B\n");
        Dbg_Printf("[INFO] 原样发到该线（不经任何换算），并打回包；缺 a|b 时默认 A 线。`dump` 打印现成整行\n");
        return 0u;
    }
    if ((uint8_t)(n - first) > (uint8_t)sizeof(buf)) {
        Dbg_Printf("[FAIL] 最多 %u 字节\n", (unsigned)sizeof(buf));
        return 0u;
    }

    for (uint8_t i = first; i < n; i++) {
        if (Dbg_Cmd_HexByte(tok[i], &buf[len]) == 0u) {
            Dbg_Printf("[FAIL] 第 %u 个参数不是两位十六进制：%s\n", (unsigned)(i - first), tok[i]);
            return 0u;
        }
        len++;
    }

    return Dbg_App_SendRaw(motor, buf, len);
}

static uint8_t Dbg_Cmd_Ack(char *tok[], uint8_t n) {
    uint8_t addr    = 0u;
    uint8_t is_both = 0u;

    if (n < 2u) {
        /* 缺省：两台各探一次（各自的线、各自的登记地址） */
        for (uint8_t i = 0u; i < (uint8_t)DBG_MOTOR_COUNT; i++) {
            Dbg_App_AskAck(i, Zdt_Bus_AddrOf((Dbg_Motor_e)i));
        }
        return 1u;
    }
    if ((Dbg_Cmd_StrEq(tok[1], "a") != 0u) || (Dbg_Cmd_StrEq(tok[1], "A") != 0u)) {
        Dbg_App_AskAck((uint8_t)DBG_MOTOR_A, Zdt_Bus_AddrOf(DBG_MOTOR_A));
        return 1u;
    }
    if ((Dbg_Cmd_StrEq(tok[1], "b") != 0u) || (Dbg_Cmd_StrEq(tok[1], "B") != 0u)) {
        Dbg_App_AskAck((uint8_t)DBG_MOTOR_B, Zdt_Bus_AddrOf(DBG_MOTOR_B));
        return 1u;
    }
    if ((Dbg_Cmd_StrEq(tok[1], "t") != 0u) || (Dbg_Cmd_StrEq(tok[1], "T") != 0u)) {
        Dbg_App_AskAck((uint8_t)DBG_MOTOR_TRAY, Zdt_Bus_AddrOf(DBG_MOTOR_TRAY));
        return 1u;
    }
    if (Dbg_Cmd_ParseAddr(tok[1], &addr, &is_both) == 0u) {
        Dbg_Printf("[FAIL] 用法：ack [a|b|t|all|地址]\n");
        return 0u;
    }
    if (is_both != 0u) {
        for (uint8_t i = 0u; i < (uint8_t)DBG_MOTOR_COUNT; i++) {
            Dbg_App_AskAck(i, Zdt_Bus_AddrOf((Dbg_Motor_e)i));
        }
        return 1u;
    }
    /* 数字地址：两条线各发一遍，哪台的 ID 匹配哪台应答 */
    for (uint8_t i = 0u; i < (uint8_t)DBG_MOTOR_COUNT; i++) {
        Dbg_App_AskAck(i, addr);
    }
    return 1u;
}

static uint8_t Dbg_Cmd_Spu(char *tok[], uint8_t n) {
    int32_t scale = 0;

    if (n < 2u) {
        Dbg_Printf("[INFO] 当前速度字段系数 %u（1=字段即 RPM，10=字段是 0.1RPM）\n",
                   (unsigned)Dbg_App_SpeedScale());
        Dbg_Printf("[INFO] 用法：spu <1|10>（只影响 bx/x/y/xy/sq/cal 的解算结果，`j` 永远是原始值）\n");
        return 0u;
    }
    if ((Dbg_Cmd_ParseI32(tok[1], &scale) == 0u) || (scale < 1) || (scale > 100)) {
        Dbg_Printf("[FAIL] 系数范围 1~100\n");
        return 0u;
    }

    Dbg_App_SetSpeedScale((uint16_t)scale);
    return 1u;
}

/* 转盘速度字段系数（与 XY 的 spu 独立）；同时缩放转盘的所有等待超时 */
/* 转盘节拍：给三种标定转速统一乘倍数（100 = 产品标定；"再快一倍" = 200） */
static uint8_t Dbg_Cmd_Tspd(char *tok[], uint8_t n) {
    int32_t pct = 0;

    if (n < 2u) {
        Dbg_Printf("[INFO] 当前转盘节拍 %u%%（100 = 产品标定的 粗寻标120/对格60/慢压30 rpm）\n",
                   (unsigned)Dbg_App_TraySpeedPct());
        Dbg_Printf("[INFO] 用法：tspd <20~400>（只调节拍，等待超时自动反缩；定稿值写回产品 CFG_TRAY_*_RPM）\n");
        return 0u;
    }
    if ((Dbg_Cmd_ParseI32(tok[1], &pct) == 0u) || (pct < 20) || (pct > 400)) {
        Dbg_Printf("[FAIL] 百分比范围 20~400\n");
        return 0u;
    }
    Dbg_App_SetTraySpeedPct((uint16_t)pct);
    return 1u;
}

static uint8_t Dbg_Cmd_Tspu(char *tok[], uint8_t n) {
    int32_t scale = 0;

    if (n < 2u) {
        Dbg_Printf("[INFO] 当前转盘速度系数 %u（1=字段即RPM，10=字段是0.1RPM）\n",
                   (unsigned)Dbg_App_TraySpeedScale());
        Dbg_Printf("[INFO] 用法：tspu <1|10>（只影响转盘；XY 用 `spu`。判定：`j t cw 30 3200 255` 秒表 2s/20s）\n");
        return 0u;
    }
    if ((Dbg_Cmd_ParseI32(tok[1], &scale) == 0u) || (scale < 1) || (scale > 100)) {
        Dbg_Printf("[FAIL] 系数范围 1~100\n");
        return 0u;
    }
    Dbg_App_SetTraySpeedScale((uint16_t)scale);
    return 1u;
}

static uint8_t Dbg_Cmd_Stat(char *tok[], uint8_t n) {
    (void)tok;
    (void)n;
    Dbg_App_PrintStatus();
    return 1u;
}

/* 限位/原点开关自检：电平（未触发=1 压下=0）+ 按压计数（消抖+状态闩锁，只认"压下"
   方向，一个按压循环恒 +1，松开弹跳不计）。`lim clr` 清零。
   与产品回零同一条判据：电平证明"开关看得见"，计数证明"EXTI 事件进得来"——
   回零靠的是沿不是电平 */
static uint8_t Dbg_Cmd_Lim(char *tok[], uint8_t n) {
    if ((n >= 2u) && (Dbg_Cmd_StrEq(tok[1], "clr") != 0u)) {
        Dbg_Gpio_ClearEdges();
        Dbg_Printf("[OK] 按压计数已清零\n");
        return 1u;
    }

    Dbg_Printf("[LIM] 未触发=1 压下=0；计数只认\"压下\"（松开弹跳不计）。"
               "COM->GND、NO->信号脚，引脚见 dbg_config.h §3.5\n");
    for (uint8_t i = 0u; i < (uint8_t)DBG_LIMIT_COUNT; i++) {
        Dbg_Limit_e which = (Dbg_Limit_e)i;
        uint8_t     level = Dbg_Gpio_Level(which);
        uint32_t    edges = Dbg_Gpio_EdgeCount(which);

        Dbg_Printf("[LIM] %-6s %-4s 电平=%u(%s)  按压=%u  最近=", Dbg_Gpio_RoleOf(which),
                   Dbg_Gpio_NameOf(which), (unsigned)level, (level != 0u) ? "开" : "压下",
                   (unsigned)edges);
        if (edges == 0u) {
            Dbg_Printf("无\n");
        } else {
            Dbg_Printf("%ums\n", (unsigned)Dbg_Gpio_LastEdgeTick(which));
        }
    }
    return 1u;
}

/* ★ 抓取全流程：pick=原地爪时序；pick <X> <Y> [mm/s]=先定位（需先归零）再走爪时序 */
static uint8_t Dbg_Cmd_Pick(char *tok[], uint8_t n) {
    int32_t ix = 0;
    int32_t iy = 0;

    if (n >= 3u) {
        if ((Dbg_Cmd_ParseI32(tok[1], &ix) == 0u) || (Dbg_Cmd_ParseI32(tok[2], &iy) == 0u)) {
            Dbg_Printf("[FAIL] 用法：pick / pick <X> <Y> [mm/s]\n");
            return 0u;
        }
        if (n >= 4u) {
            int32_t sp = 0;

            if ((Dbg_Cmd_ParseI32(tok[3], &sp) == 0u) || (sp < 0)) {
                Dbg_Printf("[FAIL] 速度不合法\n");
                return 0u;
            }
            return Dbg_App_Pick((float)ix, (float)iy, (uint16_t)sp, 1u);
        }
        return Dbg_App_Pick((float)ix, (float)iy, 0u, 1u);
    }
    return Dbg_App_Pick(0.0f, 0.0f, 0u, 0u);
}

static uint8_t Dbg_Cmd_Help(char *tok[], uint8_t n);

/* 命令表：help 直接遍历它，加命令只改这一张表 */
static const Dbg_Cmd_t s_cmd[] = {
    {"help", "", "打印本表", Dbg_Cmd_Help},
    {"en", "<all|0|a|b|t|地址>", "使能（0 或 bc = 广播，不知道 ID 时用）", Dbg_Cmd_En},
    {"dis", "<all|0|a|b|t|地址>", "失能（0 或 bc = 广播）", Dbg_Cmd_Dis},
    {"stop", "", "立即停止：三条线广播 + A/B/T 各停一次（逐帧应答判决），全部记账作废", Dbg_Cmd_Stop},
    {"stoptest", "", "★停止帧实测：广播走10mm→运动中发停止→逐帧应答判决（盯电机停没停）", Dbg_Cmd_StopTest},
    {"bx", "<mm> [rpm]", "★广播走 X：两台同向同量，不需要知道 ID", Dbg_Cmd_Bx},
    {"j", "<a|b|t|地址> <cw|ccw> <rpm> <pulse> [accel]", "单台原始点动（a/b/t=那条线；数字地址=A/B两线同发）", Dbg_Cmd_Jog},
    {"x", "<mm> [mm/s]", "按 X 意图走（正=右）——需要 ID 正确", Dbg_Cmd_X},
    {"y", "<mm> [mm/s]", "按 Y 意图走（正=下）——需要 ID 正确", Dbg_Cmd_Y},
    {"xy", "<dx> <dy> [mm/s]", "X/Y 合成运动——需要 ID 正确", Dbg_Cmd_Xy},
    {"sq", "<mm> [mm/s]", "方块测试 X+ Y+ X- Y-", Dbg_Cmd_Sq},
    {"p", "<X> <Y> [mm/s]", "★绝对定位到第三象限坐标（负数；先归零；p 打印当前位置）", Dbg_Cmd_P},
    {"drop", "[mm/s]", "★一键到投放点（坐标=dbg_config.h §4c；需先归零）", Dbg_Cmd_Drop},
    {"homey", "[mm/s]", "★Y 轴回零演练：向最大端(Y+)走，限位裸沿触发即停+电平复核+退离；任意键=软急停", Dbg_Cmd_Homey},
    {"homex", "[mm/s]", "★X 轴回零演练：向最大端(X+)走，限位裸沿触发即停+电平复核+退离；任意键=软急停", Dbg_Cmd_Homex},
    {"thome", "[粗寻标rpm]", "★转盘回零（四相，对齐产品）：凸块退出→粗寻标→反向脱开→慢压定零；走通=原点开关PB0校准正确", Dbg_Cmd_Thome},
    {"tb", "[盒号0~5]", "★转盘对格：单向正转 steps×每格脉冲@60rpm，走完双复核（索引沿+脉冲折算盒号）；tb=打印记账", Dbg_Cmd_Tb},
    {"cal", "<mm> [pulses/mm]", "标定位移（只用 X 方向）", Dbg_Cmd_Cal},
    {"g", "<0开|1紧|2初夹>", "夹爪档位（巡航必须 0=全开；紧=放到底之后才允许）", Dbg_Cmd_G},
    {"l", "<0抬|1伸>", "升降档位（0=最高 1=最低）", Dbg_Cmd_L},
    {"pick", "[X Y [mm/s]]", "★抓取全流程：张开→[定位]→初夹→下放→夹紧→抬回（带坐标需先归零）", Dbg_Cmd_Pick},
    {"raw", "<claw|lift> <us>", "舵机原始脉宽（扫行程用）", Dbg_Cmd_Raw},
    {"sstop", "<claw|lift>", "停该路舵机 PWM（顶死/发烫时的第一反应）", Dbg_Cmd_Sstop},
    {"scan", "[最大ID]", "★A/B/T 三条线逐 ID 试使能+收回包，找出各自驱动器地址", Dbg_Cmd_Scan},
    {"tim", "[a|b|t|地址] [次数]", "测一帧的发送耗时", Dbg_Cmd_Tim},
    {"dump", "[地址] [脉冲]", "★把使能帧/位置帧的字节打出来（不发）+ 现成的 send 行", Dbg_Cmd_Dump},
    {"loop", "[次数]", "★TX/RX 回环自检（短接某线 TX/RX 后跑，A/B/T 三线各测一遍）", Dbg_Cmd_Loop},
    {"baud", "[bps]", "★改三条电机线波特率（驱动器被改过波特率时试）", Dbg_Cmd_Baud},
    {"ack", "[a|b|t|地址]", "★探测驱动器回包（有 02 = 链路通；缺省=三台都探；只发使能不运动）", Dbg_Cmd_Ack},
    {"id", "<a|b|t> <地址>", "★运行时改登记地址（t 允许 0=广播/产品口径）", Dbg_Cmd_Id},
    {"send", "[a|b|t] <hex...>", "★原样发一段十六进制到指定线（默认 a；并打回包）", Dbg_Cmd_Send},
    {"spu", "[1|10]", "★速度字段系数：1=字段即RPM 10=字段是0.1RPM", Dbg_Cmd_Spu},
    {"tspu", "[1|10]", "★转盘速度系数（与 spu 独立）；同时缩放转盘所有等待超时", Dbg_Cmd_Tspu},
    {"tspd", "<20~400>", "★转盘节拍百分比（100=产品标定；想快一倍用 200）", Dbg_Cmd_Tspd},
    {"fmt", "<13|16>", "★位置帧格式：16=V2说明书(0.1°/0.1RPM) 13=老协议(脉冲/RPM)", Dbg_Cmd_Fmt},
    {"lim", "[clr]", "★限位/原点开关：电平 + 按压计数（消抖+闩锁，只认压下；clr 清零）", Dbg_Cmd_Lim},
    {"cfg", "", "打印配置（含待核实项）", Dbg_Cmd_Cfg},
    {"rst", "", "★软复位：重跑上电自检（不用手按复位键）", Dbg_Cmd_Rst},
    {"stat", "", "状态汇总", Dbg_Cmd_Stat},
};

#define CMD_COUNT (sizeof(s_cmd) / sizeof(s_cmd[0]))

void Dbg_Console_PrintHelp(void) {
    Dbg_Printf("[HELP] 命令（一行一条，回车执行）：\n");
    for (uint32_t i = 0u; i < (uint32_t)CMD_COUNT; i++) {
        Dbg_Printf("[HELP]   %-6s %-38s %s\n", s_cmd[i].name, s_cmd[i].args, s_cmd[i].help);
    }
    Dbg_Printf("[HELP] 提示：纯 X 走 A/B 同向、纯 Y 走 A/B 反向 —— 不一致就是耦合或接线问题\n");
    Dbg_Printf("[HELP] 上电自检的 [ACK] 行有回包（含 02）就说明该线链路通；没回包先查供电与接线\n");
}

static uint8_t Dbg_Cmd_Help(char *tok[], uint8_t n) {
    (void)tok;
    (void)n;
    Dbg_Console_PrintHelp();
    return 1u;
}

/* ---------------- 行处理与分派 ---------------- */

static uint8_t Dbg_Console_Tokenize(char *line, char *tok[], uint8_t max) {
    uint8_t n = 0u;

    while ((*line != '\0') && (n < max)) {
        while ((*line == ' ') || (*line == '\t')) {
            *line = '\0';
            line++;
        }
        if (*line == '\0') {
            break;
        }
        tok[n] = line;
        n++;
        while ((*line != '\0') && (*line != ' ') && (*line != '\t')) {
            line++;
        }
    }

    return n;
}

static void Dbg_Console_Execute(char *line) {
    char   *tok[CMD_MAX_TOKEN];
    uint8_t n;

    Dbg_Printf("> %s\n", line);

    n = Dbg_Console_Tokenize(line, tok, CMD_MAX_TOKEN);
    if (n == 0u) {
        return;
    }

    for (uint32_t i = 0u; i < (uint32_t)CMD_COUNT; i++) {
        if (Dbg_Cmd_StrEq(tok[0], s_cmd[i].name) != 0u) {
            (void)s_cmd[i].fn(tok, n);
            Dbg_Printf("[READY]\n");
            return;
        }
    }

    Dbg_Printf("[FAIL] 未知命令：%s（`help` 看命令表）\n[READY]\n", tok[0]);
}

uint8_t Dbg_Console_HasEverReceived(void) {
    return s_got_any;
}

void Dbg_Console_Poll(void) {
    static uint8_t prev_cr; /* 行结束宽容：\r\n、单独 \r、单独 \n 都认 */

    /* 先自愈接收：串口出错后可能一直不收了，现象就是"板子还在打、命令一个都不进" */
    Dbg_Uart_EnsureRxArmed();

    while (Dbg_Uart_Available() > 0u) {
        char c = (char)Dbg_Uart_ReadByte();

        s_got_any = 1u;

#if DBG_CFG_ECHO_RX
        /* 逐字符回显：可打印字符原样，不可见字节打转义。
           排查"PC 发命令板上没反应"时，这一行能直接分辨：
             看到字符 = PC→板 的线通了；看到 <0x0d>/<0x0a> = 行结束到了板上；
             什么都没看到 = 那根线没通（或助手没发出去） */
        if ((c >= 0x20) && (c < 0x7F)) {
            Dbg_Printf("%c", c);
        } else {
            Dbg_Printf("<0x%02x>", (unsigned)(uint8_t)c);
        }
        Dbg_Print_Flush(); /* 每个字节立刻送出：保证回显顺序与命令输出不混在一起 */
#endif

        /* 串口助手的"发送新行"有的发 \r\n、有的只发 \r：两种都必须当整行处理，
           否则敲了命令毫无反应，而现象和"线没接好"一模一样 */
        if ((c == '\r') || (c == '\n')) {
            if ((c == '\n') && (prev_cr != 0u)) {
                prev_cr = 0u;
                continue; /* 这是 \r\n 的第二个字节，行已经执行过了 */
            }
            prev_cr = (c == '\r') ? 1u : 0u;

            s_line[s_len] = '\0';
            if (s_len > 0u) {
                Dbg_Console_Execute(s_line);
            }
            s_len = 0u;
            continue;
        }

        prev_cr = 0u;

        if ((c == 8) || (c == 127)) { /* 退格：本侧只回显，不做行编辑 */
            if (s_len > 0u) {
                s_len--;
            }
            continue;
        }

        if (s_len < (uint8_t)(DBG_CFG_CMD_MAX - 1u)) {
            s_line[s_len] = c;
            s_len++;
        }
        /* 超长直接丢弃，不截断执行半条命令 */
    }
}