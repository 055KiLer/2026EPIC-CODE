/**
 * @file    bsp_uart.h
 * @brief   6 路串口：环形缓冲接收 + 阻塞发送（Bsp 层）
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

/* 依赖 CubeMX 已完成的 MX_USARTx_UART_Init() */
void Bsp_Uart_Init(void);

/**
 * @brief  阻塞发送（BUSY 时在同一超时预算内自动重试）
 * @retval 1=整帧已发出  0=超时/参数非法，一字节都没发出去
 * @note   返回 0 时调用方必须让对应的运动记账作废
 */
uint8_t Bsp_Uart_Send(UART_ID_e id, const uint8_t *data, uint16_t len);

uint16_t Bsp_Uart_Available(UART_ID_e id);
uint8_t  Bsp_Uart_ReadByte(UART_ID_e id);
void     Bsp_Uart_Flush(UART_ID_e id);

/**
 * @brief  读一段回包：等到收到至少 1 字节，然后收满或到超时为止
 * @param  buf/cap 目的缓冲与容量；timeout_ms 等待总时长
 * @retval 实际收到的字节数（0 = 指定时间内一个字节都没有）
 * @note   给"驱动器自检"用（Zdt_Probe）。除 tick 超时外还有一个纯计数上限：
 *         tick 万一被中断风暴冻住，这里也不会挂死（那类问题在台架上踩过）
 */
uint16_t Bsp_Uart_ReadReply(UART_ID_e id, uint8_t *buf, uint16_t cap, uint32_t timeout_ms);

/**
 * @brief  故障路径专用发送：直接轮询 TXE/TC 写 DR，不碰 HAL、不等 tick
 * @retval 1=已发出  0=外设未使能或超时（计数上限）
 * @note   故障处理里 tick 可能已经停了（故障优先级高于 SysTick），HAL 的超时判断会挂死；
 *         正常流程请用 Bsp_Uart_Send
 */
uint8_t Bsp_Uart_SendDirect(UART_ID_e id, const uint8_t *data, uint16_t len);

#endif