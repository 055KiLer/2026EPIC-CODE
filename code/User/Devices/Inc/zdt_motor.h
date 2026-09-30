/**
 * @file    zdt_motor.h
 * @brief   ZDT 张大头闭环步进电机驱动（Device 层，协议见电控文档 §4.4）
 * @note    主控只发不收，无到位反馈；speed 与 pulse 均为「电机轴」口径，
 *          带减速箱时换算只体现在 pulse 里
 */
#ifndef __ZDT_MOTOR_H
#define __ZDT_MOTOR_H

#include "main.h"

typedef enum {
    ZDT_ID_A = 0, /* CoreXY A（USART3）    */
    ZDT_ID_B,     /* CoreXY B（UART5）     */
    ZDT_ID_TRAY,  /* 转盘 1:20（USART1）   */
    ZDT_ID_NUM
} Zdt_ID_e;

typedef enum {
    ZDT_DIR_CW  = 0x00,
    ZDT_DIR_CCW = 0x01
} Zdt_DIR_e;

/* Zdt_Probe 的返回值（驱动器应答见 zdt_motor.c 的说明） */
typedef enum {
    ZDT_PROBE_OK = 1,        /* 回了 02：命令已正确接收 */
    ZDT_PROBE_NO_REPLY = 0,  /* 一个字节都没有：数据线/地址/波特率/供电 */
    ZDT_PROBE_REJECT = -1    /* 回了 EE/E2 或非标准字节：被拒（控制模式=脉冲+方向、校验、参数） */
} Zdt_Probe_e;

/* 上电时序第一步：依次使能三台 */
void Zdt_Init(void);

/**
 * @brief  驱动器自检：发使能帧并读回包（不含运动指令，电机不会走）
 * @param  timeout_ms 等回包的时长（台架实测回包通常 <2ms）
 * @note   用途：上电时判"这一路驱动器在不在、收不收"。两种静默失效只有它能发现：
 *         ① 完全不应答（数据线/地址/波特率）② 只应答不执行运动（控制模式=脉冲+方向）
 */
Zdt_Probe_e Zdt_Probe(Zdt_ID_e id, uint32_t timeout_ms);

/* 以下均返回 1=帧已发出  0=发送失败（电机不会动，调用方须作废记账） */
uint8_t Zdt_Enable(Zdt_ID_e id, uint8_t enable);
uint8_t Zdt_MoveRelative(Zdt_ID_e id, Zdt_DIR_e dir, uint16_t speed_rpm, uint8_t accel, uint32_t pulse);
uint8_t Zdt_SpeedMode(Zdt_ID_e id, Zdt_DIR_e dir, uint16_t speed_rpm, uint8_t accel);
uint8_t Zdt_Stop(Zdt_ID_e id);

/**
 * @brief  故障路径：给三台各发一次停止帧（不依赖 HAL/tick，可在异常处理里调用）
 * @note   转盘是速度模式持续转的，主控进异常后没人再发指令 → 不停就一直转。故异常处理
 *         先调本函数再死循环（对应《调试与推进计划》A 组遗留 #3）
 */
void Zdt_PanicStopAll(void);

#endif