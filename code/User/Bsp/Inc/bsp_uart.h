/**
 * @file    bsp_uart.h
 * @brief   6 路串口：环形缓冲接收 + 阻塞发送（Bsp 层）
 * @note    接收纪律（2026-10-01 对齐台架）：只武装真正消费接收的线路（视觉线）；
 *          电机线广播不回包、屏只发不收、UART4 预留，RX 一律不开（悬空拾噪 =
 *          中断风暴 = SysTick 冻结，台架实测过）。发送走寄存器级轮询，契约见 Send
 */
#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "main.h"

/* 语义编号，不是外设编号。编号 -> 外设的对应关系只在 bsp_uart.c 的 s_huart[] */
typedef enum {
    UART_ID_SCREEN = 0, /* USART6  串口屏 */
    UART_ID_K230,       /* USART2  视觉上位机（ROCK 5C），本侧只收不发 */
    UART_ID_SPARE,      /* UART4   预留   */
    UART_ID_MOTOR_A,    /* USART3  CoreXY A */
    UART_ID_MOTOR_B,    /* UART5   CoreXY B */
    UART_ID_TRAY,       /* USART1  转盘   */
    UART_ID_NUM
} UART_ID_e;

/* 依赖 CubeMX 已完成的 MX_USARTx_UART_Init()；只武装视觉线 RX */
void Bsp_Uart_Init(void);

/**
 * @brief  周期维护（安全守护任务 1ms 调）：视觉线噪声关断后到点自动重开；
 *         真机上顺带补视觉线的 UE/RE/TE 使能位
 */
void Bsp_Uart_Poll(void);

/**
 * @brief  阻塞发送（寄存器级：轮询 TXE 写 DR、等 TC，超时用纯计数不依赖 tick；
 *         同口并发调用经忙锁串行化，一条 Send = 一条完整帧）
 * @retval 1=整帧已发出（TC 已出）  0=硬件异常/参数非法。硬件异常时可能已发出部分
 *         字节（与旧 HAL 路径同口径），调用方必须让对应的运动记账作废并重回零
 */
uint8_t Bsp_Uart_Send(UART_ID_e id, const uint8_t *data, uint16_t len);

uint16_t Bsp_Uart_Available(UART_ID_e id);
uint8_t  Bsp_Uart_ReadByte(UART_ID_e id);
void     Bsp_Uart_Flush(UART_ID_e id);

/**
 * @brief  故障路径专用发送：直接轮询 TXE/TC 写 DR，不碰 HAL、不等 tick、不取忙锁
 * @retval 1=已发出  0=外设未使能或超时（计数上限）
 * @note   故障处理里 tick 可能已经停了（故障优先级高于 SysTick），HAL 的超时判断会挂死；
 *         正常流程请用 Bsp_Uart_Send
 */
uint8_t Bsp_Uart_SendDirect(UART_ID_e id, const uint8_t *data, uint16_t len);

#endif
