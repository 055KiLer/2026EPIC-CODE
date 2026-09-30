/**
 * @file    zdt_bus.h
 * @brief   ZDT 闭环步进驱动 · 每台一路 UART 的发送层（本调试工程专用）
 * @note    2026-09-30 起接线与产品工程一致：**一台驱动器一条 UART**
 *          （A=USART3/M1，B=UART5/M2），帧格式与产品 `code/User/Devices/zdt_motor.c`
 *          逐字节一致（位置帧 13 字节）。帧内地址字节仍是该台驱动器的 ID——
 *          驱动器只执行"自己 ID 或广播 0x00"的帧；一条线只有一台，所以 ID 配错时
 *          用广播也能让它动（这是"换了驱动器还没核 ID"阶段的救命路径）。
 *          方向约定沿用产品：**脉冲为正 = CW = dir 0x00**。
 *          命名约定：`*_On(motor, addr, ...)` = 发到指定电机线、地址字节用实参；
 *          `*Motor(motor, ...)` = 发到指定电机线、地址字节用该台登记地址。
 */
#ifndef __ZDT_BUS_H
#define __ZDT_BUS_H

#include "main.h"

typedef enum {
    DBG_MOTOR_A = 0,    /* CoreXY A（产品板 M1 / USART3 PC10/PC11） */
    DBG_MOTOR_B,        /* CoreXY B（产品板 M2 / UART5  PC12/PD2） */
    DBG_MOTOR_TRAY,     /* 转盘 57（产品板 M3 / USART1 PB6/PB7，1:20 减速） */
    DBG_MOTOR_COUNT
} Dbg_Motor_e;

/* 地址表在 Config/dbg_config.h（DBG_CFG_MOTOR_A_ID / _B_ID），上电自检会自动采纳 */
uint8_t     Zdt_Bus_AddrOf(Dbg_Motor_e motor);
const char *Zdt_Bus_NameOf(Dbg_Motor_e motor);

/* 运行时改某台的地址（台架换电机/换 ID 用；改完 `ack a/b` 验一下更稳） */
void Zdt_Bus_SetAddr(Dbg_Motor_e motor, uint8_t addr);

/* 位置帧格式运行时切换（2026-09-25 台架实测：V2 固件对 13 字节位置帧回 EE=格式错）：
   0 = 13 字节（Emm_V5.0 老协议：[addr][FD][dir][速度2B][加速度1B][脉冲4B][模式][同步][6B]）
   1 = 16 字节（V2 说明书 7.4.1：[addr][FD][符号][加速2B][减速2B][速度2B=0.1RPM]
       [角度4B=0.1°][相对/绝对][多机同步][6B]）
   控制台 `fmt 13|16` 切换；x/y/xy/sq/cal/bx/dump/tim 全部跟随当前格式 */
void    Zdt_Bus_SetFrame16(uint8_t use16);
uint8_t Zdt_Bus_Frame16(void);

/* 以下均返回 1=整帧已发出  0=发送失败（那台不会动，调用方须作废记账）
   motor：0 = A 线（USART3），1 = B 线（UART5），2 = 转盘线（USART1）；
   addr：该线驱动器的 ID，0x00 = 广播（产品三台全用广播） */
uint8_t Zdt_Bus_EnableOn(uint8_t motor, uint8_t addr, uint8_t enable);
uint8_t Zdt_Bus_MoveOn(uint8_t motor, uint8_t addr, int32_t pulse_signed, uint16_t speed_rpm,
                       uint8_t accel);
uint8_t Zdt_Bus_SpeedOn(uint8_t motor, uint8_t addr, uint8_t ccw, uint16_t speed_rpm, uint8_t accel);
uint8_t Zdt_Bus_StopOn(uint8_t motor, uint8_t addr);
uint8_t Zdt_Bus_ZeroPositionOn(uint8_t motor, uint8_t addr);

/**
 * @brief  只构帧不发（测发送耗时、对帧格式用）
 * @param  out 至少 16 字节
 * @retval 帧长（13 或 16，跟随 Zdt_Bus_SetFrame16）；参数非法返回 0
 */
uint16_t Zdt_Bus_BuildPosition(uint8_t addr, int32_t pulse_signed, uint16_t speed_rpm, uint8_t accel,
                               uint8_t *out);

/* 只构使能帧不发（6 字节） */
uint16_t Zdt_Bus_BuildEnable(uint8_t addr, uint8_t enable, uint8_t *out);

/* 本工程的三台便捷封装：按登记地址使能/点动/停 */
uint8_t Zdt_Bus_EnableMotor(Dbg_Motor_e motor, uint8_t enable);
uint8_t Zdt_Bus_MoveMotor(Dbg_Motor_e motor, int32_t pulse_signed, uint16_t speed_rpm,
                          uint8_t accel);
uint8_t Zdt_Bus_StopMotor(Dbg_Motor_e motor);

/* 全部电机一起停：A/B/转盘三条线各发一帧登记地址的停止帧（三条线独立，互不丢帧） */
uint8_t Zdt_Bus_StopBoth(void);

/**
 * @brief  故障处理专用：不依赖 HAL_GetTick 地连发停止帧
 * @note   HardFault 里 tick 可能已停，用带重试的发送会卡死；
 *          这里每条线"登记地址 + 广播"各裸发一次就返回（广播保证"ID 没核过"的
 *          驱动器也停得下来；这也是产品 A 组遗留「fault handler 不停机」的样板实现：
 *          转盘以速度模式跑着时进 HardFault，若只是 while(1)，电机会一直转到比赛结束）
 */
void Zdt_Bus_PanicStop(void);

#endif
