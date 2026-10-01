/**
 * @file    dbg_uart.h
 * @brief   六路串口：控制台 + 电机 A/B/转盘三条线 + 屏线(USART6) + 视觉线(USART2)，各自带接收环形缓冲
 * @note    2026-09-30 起接线与产品工程一致：三台驱动器**各占一路 UART**
 *          （转盘=USART1/M3，A=USART3/M1，B=UART5/M2），不再是一条总线挂多台。
 *          2026-10-01 接入产品另两条链路（屏=CN2/USART6，视觉=CN3/USART2），与产品同口同脚。
 *          电机线编号约定：0 = A（USART3），1 = B（UART5），2 = 转盘（USART1），
 *          与 zdt_bus.h 的 Dbg_Motor_e 数值一致（BSP 层不反向包含 Devices 头，用数字传参）
 */
#ifndef __DBG_UART_H
#define __DBG_UART_H

#include "main.h"

#define DBG_UART_RX_BUF_SIZE 128u /* 2 的幂 */

/* 六路串口都挂接收中断 */
void Dbg_Uart_Init(void);

/* ---- 控制台 ---- */
uint16_t Dbg_Uart_Available(void);
uint8_t  Dbg_Uart_ReadByte(void);
uint8_t  Dbg_Uart_SendConsole(const uint8_t *data, uint16_t len);

/* ---- 电机线（motor：0 = A/USART3，1 = B/UART5，2 = 转盘/USART1）---- */
uint16_t Dbg_Uart_MotorAvailable(uint8_t motor);
uint8_t  Dbg_Uart_MotorReadByte(uint8_t motor);
void     Dbg_Uart_MotorFlush(uint8_t motor);

/**
 * @brief  重新打开某条电机线的接收（含错误中断）并重置噪声预算
 * @note   电机线接收会被自动关掉：RX 引脚悬空拾噪时，错误中断反复重挂会形成风暴、
 *         把主循环与 SysTick 一起饿死（现象是"卡死、命令全不回"）。
 *         `Dbg_Uart_MotorFlush()` 内部已经会调它，所以要看回包的命令无需自己调
 */
void     Dbg_Uart_MotorArm(uint8_t motor);

/**
 * @brief  关掉某条电机线的接收 —— 这是电机线的常态（只发的，只在探测回包时临时打开）
 */
void     Dbg_Uart_MotorDisarm(uint8_t motor);

/* ---- 屏线（USART6/CN2）：只发为主，Flush/Read 供回环自检与杂音观察 ---- */
uint16_t Dbg_Uart_ScreenAvailable(void);
uint8_t  Dbg_Uart_ScreenReadByte(void);
void     Dbg_Uart_ScreenFlush(void);
uint8_t  Dbg_Uart_SendScreen(const uint8_t *data, uint16_t len);

/* ---- 视觉线（USART2/CN3）：只收（产品同口径，本侧不发）---- */
uint16_t Dbg_Uart_VisionAvailable(void);
uint8_t  Dbg_Uart_VisionReadByte(void);
void     Dbg_Uart_VisionFlush(void);

/* 中断入口：由 Core/Src/stm32f4xx_it.c 里的 USARTx_IRQHandler 调用。
   之所以自写而不用 HAL_UART_IRQHandler：HAL 在 RxState 不是 BUSY_RX 时
   既不清标志也不回调，只要 RXNEIE 还开着就会无限重进中断、把 CPU 吃干 */
void     Dbg_Uart_IsrPushMotor(uint8_t motor, uint8_t byte, uint32_t sr);
void     Dbg_Uart_IsrPushConsole(uint8_t byte, uint32_t sr);
void     Dbg_Uart_IsrPushScreen(uint8_t byte, uint32_t sr);
void     Dbg_Uart_IsrPushVision(uint8_t byte, uint32_t sr);

/* ★ 控制台 RX 线路体检：PA1 当输入采样，量"起点电平 + 边沿数"（见 dbg_uart.c）*/
void     Dbg_Uart_ProbeConsoleRx(void);

/**
 * @brief  某条电机线发送整帧（阻塞；寄存器级轮询，不依赖 HAL_GetTick）
 * @param  motor 0 = A 线，1 = B 线，2 = 转盘线
 * @retval 1=整帧已发出  0=超时/未初始化（该帧对应电机会不动，调用方须作废记账）
 */
uint8_t Dbg_Uart_SendMotor(uint8_t motor, const uint8_t *data, uint16_t len);

/**
 * @brief  运行时改某条电机线的波特率（重新初始化该 UART）
 * @note   驱动器被改过波特率时用：控制台不受影响，所以能一条条试过去。
 *         内部会在重初始化后**重新挂上接收中断**（DeInit 会把它清掉）
 * @retval 1=已切换  0=参数不支持
 */
uint8_t Dbg_Uart_SetMotorBaud(uint8_t motor, uint32_t bps);

/* 某条电机线当前波特率（打印用） */
uint32_t Dbg_Uart_MotorBaud(uint8_t motor);

/* ★ 接收自愈：主循环每圈调一次；哪一路没挂在收就重新挂上。
   串口因噪声/溢出出错后可能一直停摆（现象是"板子还在打、命令却一个都不进"），
   这一句就是那种"必须复位才能恢复"怪象的根治手段。
   同时自愈 CR1 的 UE/RE/TE（实测两路 UE 曾同时被清零、收发全停）。
   屏/视觉线被噪声预算关掉后由这里到期自动重开（电机线按需重开） */
void Dbg_Uart_EnsureRxArmed(void);

/* UE 自愈计数（which: 0=控制台 1=A线 2=B线 3=转盘线 4=屏线 5=视觉线）：正常应为 0；
   >0 = 该路 CR1 曾被外部因素破坏并被自愈救回——次数增长说明问题在复发，需要继续抓根因 */
uint16_t Dbg_Uart_UeHealCount(uint8_t which);

#endif
