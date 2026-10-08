/**
 * @file    zdt_motor.c
 * @brief   ZDT Emm 系专用串口指令（广播地址 0x00，帧尾固定 0x6B）
 * @note    帧结构 [地址][功能码][参数...][0x6B]，多字节参数大端。本模块不保存位置
 */
#include "zdt_motor.h"
#include "bsp_uart.h"
#include "sort_config.h"

#define ZDT_ADDR_BROADCAST 0x00u
#define ZDT_CHECKSUM       0x6Bu

/* 只写语义编号，物理口由 bsp_uart.c 的 s_huart[] 决定 */
static const UART_ID_e s_uart[ZDT_ID_NUM] = {(UART_ID_e)CFG_UART_MOTOR_A, (UART_ID_e)CFG_UART_MOTOR_B,
                                             (UART_ID_e)CFG_UART_TRAY};

static uint8_t Zdt_Send(Zdt_ID_e id, const uint8_t *frame, uint16_t len) {
    if (id >= ZDT_ID_NUM) {
        return 0u;
    }
    return Bsp_Uart_Send(s_uart[id], frame, len);
}

/* 名义转速 -> 帧内速度字段（乘单位系数，见 sort_config.h 的 CFG_ZDT_SPEED_FIELD_SCALE）。
   集中在这一处换算：算法层继续用真实 RPM，设备层负责适应驱动器口径。
   上限 600：XY 侧另有解算器 300 上限，这里管的是转盘（二次提速后最高寻标 480，
   旧 300 钳位会把寻标静默砍到 300——台架 Dbg_App_TrayRpm 同口径放宽） */
#define ZDT_SPEED_FIELD_MAX 600u

static uint16_t Zdt_SpeedField(uint16_t speed_rpm) {
    uint32_t field;

    if (speed_rpm > ZDT_SPEED_FIELD_MAX) {
        speed_rpm = (uint16_t)ZDT_SPEED_FIELD_MAX;
    }
    field = (uint32_t)speed_rpm * (uint32_t)CFG_ZDT_SPEED_FIELD_SCALE;

    if (field > 65535u) {
        field = 65535u;
    }
    if (field == 0u) {
        field = 1u; /* 速度 0 在部分固件里等于"停止"，运动指令必须给非零 */
    }
    return (uint16_t)field;
}

void Zdt_Init(void) {
    Zdt_Enable(ZDT_ID_A, 1u);
    Zdt_Enable(ZDT_ID_B, 1u);
    Zdt_Enable(ZDT_ID_TRAY, 1u);
}

uint8_t Zdt_Enable(Zdt_ID_e id, uint8_t enable) {
    uint8_t frame[6] = {ZDT_ADDR_BROADCAST, 0xF3u, 0xABu, enable ? 0x01u : 0x00u, 0x00u, ZDT_CHECKSUM};

    return Zdt_Send(id, frame, sizeof(frame));
}

uint8_t Zdt_MoveRelative(Zdt_ID_e id, Zdt_DIR_e dir, uint16_t speed_rpm, uint8_t accel, uint32_t pulse) {
    if (pulse == 0u) {
        return 1u; /* 零脉冲不下发，视为成功 */
    }

    /* Emm_V5.0 位置帧是 13 字节：脉冲后还有【运动模式】和【多机同步】两个字节。
       少发同步字节会让 0x6B 落进驱动器期望"多机同步"的位置、下一条帧的地址字节被当成
       校验字节吞掉 —— 后果不是"少一字节"，而是整条总线此后一直错位、一帧都不执行 */
    uint8_t frame[13] = {ZDT_ADDR_BROADCAST, 0xFDu, (uint8_t)dir, 0u};
    uint16_t field    = Zdt_SpeedField(speed_rpm);

    frame[3]  = (uint8_t)(field >> 8);
    frame[4]  = (uint8_t)(field);
    frame[5]  = accel;
    frame[6]  = (uint8_t)(pulse >> 24);
    frame[7]  = (uint8_t)(pulse >> 16);
    frame[8]  = (uint8_t)(pulse >> 8);
    frame[9]  = (uint8_t)(pulse);
    frame[10] = (uint8_t)CFG_ZDT_POS_MODE; /* 运动模式：00 = 相对上一目标位置、02 = 相对当前位置（见 sort_config.h 该常量一节） */
    frame[11] = 0x00u; /* 多机同步 = 立即执行 */
    frame[12] = ZDT_CHECKSUM;

    return Zdt_Send(id, frame, sizeof(frame));
}

uint8_t Zdt_SpeedMode(Zdt_ID_e id, Zdt_DIR_e dir, uint16_t speed_rpm, uint8_t accel) {
    uint8_t  frame[8] = {ZDT_ADDR_BROADCAST, 0xF6u, (uint8_t)dir, 0u};
    uint16_t field    = Zdt_SpeedField(speed_rpm);

    frame[3] = (uint8_t)(field >> 8);
    frame[4] = (uint8_t)(field);
    frame[5] = accel;
    frame[6] = 0x00u;
    frame[7] = ZDT_CHECKSUM;

    return Zdt_Send(id, frame, sizeof(frame));
}

/* "驱动器自检"（发使能帧读回包的 Zdt_Probe）已删除：本批驱动器对广播不回包
   （台架 2026-09-30 定稿），等回包只会永远误报失败；本层与电控文档 §4.3 一致，
   主控只发不收 */

uint8_t Zdt_Stop(Zdt_ID_e id) {
    uint8_t frame[5] = {ZDT_ADDR_BROADCAST, 0xFEu, 0x98u, 0x00u, ZDT_CHECKSUM};

    return Zdt_Send(id, frame, sizeof(frame));
}

/* 故障路径：三台各发一次停止帧，用不依赖 HAL/tick 的发送（见 Bsp_Uart_SendDirect）。
   为什么必须有：转盘是以**速度模式**持续转的，主控一旦进 HardFault 就没有任何人再发指令，
   它会一直转到有人断电为止；而本函数是故障处理里唯一还能起作用的东西 */
void Zdt_PanicStopAll(void) {
    for (uint8_t i = 0u; i < (uint8_t)ZDT_ID_NUM; i++) {
        uint8_t frame[5] = {ZDT_ADDR_BROADCAST, 0xFEu, 0x98u, 0x00u, ZDT_CHECKSUM};

        (void)Bsp_Uart_SendDirect(s_uart[i], frame, sizeof(frame));
    }
}