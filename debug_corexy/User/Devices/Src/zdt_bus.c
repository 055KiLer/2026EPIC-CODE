/**
 * @file    zdt_bus.c
 * @brief   ZDT 闭环步进 · 每台一路 UART 的帧构造与发送
 * @note    帧格式来源：产品 `zdt_motor.c`（对 Emm_V5.0 手册）＋ 物创项目的
 *          `drive/stepper_drive.c`（同批驱动器实测过），两者除地址字段外逐字节一致。
 *          位置帧 13 字节：脉冲后还有【运动模式】【多机同步】两个字节，
 *          少发一个字节会让驱动器此后一帧都不执行。
 *          2026-09-30 起与产品接线一致：A=USART3、B=UART5，各占一路 UART；
 *          帧内地址字节仍是驱动器 ID（一条线一台，ID 未核时用广播 0x00）。
 */
#include "zdt_bus.h"
#include "dbg_config.h"
#include "sort_config.h" /* CFG_PULSE_PER_REV_42：16 字节帧的角度换算要用 */
#include "dbg_uart.h"
#include "dbg_print.h"

#define ZDT_CHKSUM 0x6Bu

/* 寻址定调（2026-09-30 用户确认）：一条线一台驱动器，**全广播 0x00**（产品同口径），
   不再做单播/地址采纳；`id a/b/t` 仅实验用。本批驱动器对广播不回包 → 运动路径只发不收 */
static uint8_t s_addr[DBG_MOTOR_COUNT] = {DBG_CFG_ADDR_BROADCAST, DBG_CFG_ADDR_BROADCAST,
                                          DBG_CFG_ADDR_BROADCAST};
static const char *const s_name[DBG_MOTOR_COUNT] = {"A", "B", "T"};

/* 位置帧格式：0 = 13 字节老协议，1 = 16 字节 V2 协议（详见 zdt_bus.h 的注释） */
static uint8_t s_frame16;

void Zdt_Bus_SetFrame16(uint8_t use16) {
    s_frame16 = (use16 != 0u) ? 1u : 0u;
}

uint8_t Zdt_Bus_Frame16(void) {
    return s_frame16;
}

uint8_t Zdt_Bus_AddrOf(Dbg_Motor_e motor) {
    return (motor < DBG_MOTOR_COUNT) ? s_addr[motor] : 0u;
}

void Zdt_Bus_SetAddr(Dbg_Motor_e motor, uint8_t addr) {
    if (motor < DBG_MOTOR_COUNT) {
        s_addr[motor] = addr;
    }
}

const char *Zdt_Bus_NameOf(Dbg_Motor_e motor) {
    return (motor < DBG_MOTOR_COUNT) ? s_name[motor] : "?";
}

uint16_t Zdt_Bus_BuildEnable(uint8_t addr, uint8_t enable, uint8_t *out) {
    if (out == NULL) {
        return 0u;
    }

    out[0] = addr;
    out[1] = 0xF3u;
    out[2] = 0xABu;
    out[3] = enable ? 0x01u : 0x00u;
    out[4] = 0x00u; /* 多机同步 */
    out[5] = ZDT_CHKSUM;

    return 6u;
}

uint16_t Zdt_Bus_BuildPosition(uint8_t addr, int32_t pulse_signed, uint16_t speed_rpm, uint8_t accel,
                               uint8_t *out) {
    uint8_t  dir;
    uint32_t pulse;

    if (out == NULL) {
        return 0u;
    }

    /* 产品口径：正脉冲 = CW = dir 0x00 */
    dir   = (pulse_signed > 0) ? 0x00u : 0x01u;
    pulse = (uint32_t)((pulse_signed > 0) ? pulse_signed : -pulse_signed);

    if (s_frame16 != 0u) {
        /* V2 协议 16 字节（说明书 7.4.1 梯形曲线位置模式）：
           [addr][FD][符号][加速2B][减速2B][速度2B=0.1RPM][角度4B=0.1°][相对][同步][6B]
           单位换算：角度(0.1°) = 脉冲 × 3600 ÷ 每圈脉冲；速度(0.1RPM) = RPM × 10。
           加速度沿用说明书示例 511RPM/s（45rpm 约 0.09s 加速完成） */
        uint32_t angle = pulse * 3600u / (uint32_t)CFG_PULSE_PER_REV_42;
        uint16_t spd16 = (uint16_t)(speed_rpm * 10u);

        (void)accel;
        out[0]  = addr;
        out[1]  = 0xFDu;
        out[2]  = dir;
        out[3]  = (uint8_t)(DBG_CFG_ACC16_RPM_PS >> 8);
        out[4]  = (uint8_t)DBG_CFG_ACC16_RPM_PS;
        out[5]  = (uint8_t)(DBG_CFG_ACC16_RPM_PS >> 8);
        out[6]  = (uint8_t)DBG_CFG_ACC16_RPM_PS;
        out[7]  = (uint8_t)(spd16 >> 8);
        out[8]  = (uint8_t)spd16;
        out[9]  = (uint8_t)(angle >> 24);
        out[10] = (uint8_t)(angle >> 16);
        out[11] = (uint8_t)(angle >> 8);
        out[12] = (uint8_t)angle;
        out[13] = 0x00u; /* 相对位置运动 */
        out[14] = 0x00u; /* 多机同步 = 立即执行 */
        out[15] = ZDT_CHKSUM;
        return 16u;
    }

    out[0]  = addr;
    out[1]  = 0xFDu;
    out[2]  = dir;
    out[3]  = (uint8_t)(speed_rpm >> 8);
    out[4]  = (uint8_t)(speed_rpm);
    out[5]  = accel;
    out[6]  = (uint8_t)(pulse >> 24);
    out[7]  = (uint8_t)(pulse >> 16);
    out[8]  = (uint8_t)(pulse >> 8);
    out[9]  = (uint8_t)(pulse);
    out[10] = DBG_CFG_POS_MODE_RELATIVE; /* 运动模式：00 = 相对上一目标位置 */
    out[11] = 0x00u;                     /* 多机同步 = 立即执行 */
    out[12] = ZDT_CHKSUM;

    return 13u;
}

uint8_t Zdt_Bus_EnableOn(uint8_t motor, uint8_t addr, uint8_t enable) {
    uint8_t  frame[6];
    uint16_t len = Zdt_Bus_BuildEnable(addr, enable, frame);

    if (len == 0u) {
        return 0u;
    }

    return Dbg_Uart_SendMotor(motor, frame, len);
}

uint8_t Zdt_Bus_MoveOn(uint8_t motor, uint8_t addr, int32_t pulse_signed, uint16_t speed_rpm,
                       uint8_t accel) {
    uint8_t  frame[16];
    uint16_t len;

    if (pulse_signed == 0) {
        return 1u; /* 零脉冲不下发，视为成功 */
    }

    /* 位置帧 13 字节：脉冲后还有【运动模式】【多机同步】两个字节。
       少发一个字节会让驱动器此后一帧都不执行，而本侧只发不收、账面看不出来 */
    len = Zdt_Bus_BuildPosition(addr, pulse_signed, speed_rpm, accel, frame);
    if (len == 0u) {
        return 0u;
    }

    return Dbg_Uart_SendMotor(motor, frame, len);
}

uint8_t Zdt_Bus_SpeedOn(uint8_t motor, uint8_t addr, uint8_t ccw, uint16_t speed_rpm,
                        uint8_t accel) {
    uint8_t frame[8] = {addr, 0xF6u, ccw, 0u};

    frame[3] = (uint8_t)(speed_rpm >> 8);
    frame[4] = (uint8_t)(speed_rpm);
    frame[5] = accel;
    frame[6] = 0x00u;
    frame[7] = ZDT_CHKSUM;

    return Dbg_Uart_SendMotor(motor, frame, sizeof(frame));
}

uint8_t Zdt_Bus_StopOn(uint8_t motor, uint8_t addr) {
    uint8_t frame[5] = {addr, 0xFEu, 0x98u, 0x00u, ZDT_CHKSUM};

    return Dbg_Uart_SendMotor(motor, frame, sizeof(frame));
}

/* 物创项目 `ZDT_drive.c` 验证过的帧：清当前位置为零点 */
uint8_t Zdt_Bus_ZeroPositionOn(uint8_t motor, uint8_t addr) {
    uint8_t frame[5] = {addr, 0x93u, 0x88u, 0x01u, ZDT_CHKSUM};

    return Dbg_Uart_SendMotor(motor, frame, sizeof(frame));
}

uint8_t Zdt_Bus_EnableMotor(Dbg_Motor_e motor, uint8_t enable) {
    return Zdt_Bus_EnableOn((uint8_t)motor, Zdt_Bus_AddrOf(motor), enable);
}

uint8_t Zdt_Bus_MoveMotor(Dbg_Motor_e motor, int32_t pulse_signed, uint16_t speed_rpm,
                          uint8_t accel) {
    return Zdt_Bus_MoveOn((uint8_t)motor, Zdt_Bus_AddrOf(motor), pulse_signed, speed_rpm, accel);
}

uint8_t Zdt_Bus_StopMotor(Dbg_Motor_e motor) {
    return Zdt_Bus_StopOn((uint8_t)motor, Zdt_Bus_AddrOf(motor));
}

uint8_t Zdt_Bus_StopBoth(void) {
    uint8_t ok = 1u;

    for (uint8_t i = 0u; i < (uint8_t)DBG_MOTOR_COUNT; i++) {
        ok = (ok && Zdt_Bus_StopMotor((Dbg_Motor_e)i)) ? 1u : 0u;
    }
    return ok;
}

void Zdt_Bus_PanicStop(void) {
    /* 这条是故障路径：tick 可能已经停了（故障优先级高于 SysTick，它进不来），
       所以必须用**不读 tick** 的发送 —— Dbg_Uart_SendMotor 是寄存器级轮询 +
       纯计数超时，正好满足。全广播寻址（2026-09-30 定调）：每条线一帧广播停即可 */
    for (uint8_t i = 0u; i < (uint8_t)DBG_MOTOR_COUNT; i++) {
        uint8_t bcast[5] = {DBG_CFG_ADDR_BROADCAST, 0xFEu, 0x98u, 0x00u, ZDT_CHKSUM};

        (void)Dbg_Uart_SendMotor(i, bcast, sizeof(bcast));
    }
}
