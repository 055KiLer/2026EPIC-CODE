/**
  ******************************************************************************
  * @file    usart.h
  * @brief   本工程六路串口：转盘线 + 电机 A 线 + 电机 B 线 + 控制台 + 屏线 + 视觉线
  *          （实例见 Config/dbg_config.h）
  * @note    2026-09-30 起接线与产品工程一致：A=USART3（M1）、B=UART5（M2）、
  *          转盘=USART1（M3）、控制台=UART4（CN1 预留口）。
  *          2026-10-01 接入产品另两条链路：屏=USART6（CN2）、视觉=USART2（CN3）
  ******************************************************************************
  */
#ifndef __USART_H
#define __USART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern UART_HandleTypeDef huart1; /* USART1 PB6/PB7  转盘 57 线（产品板 M3 座，广播 0x00） */
extern UART_HandleTypeDef huart3; /* USART3 PC10/PC11 电机 A 线（产品板 M1 座） */
extern UART_HandleTypeDef huart5; /* UART5  PC12/PD2  电机 B 线（产品板 M2 座） */
extern UART_HandleTypeDef huart4; /* UART4  PA0/PA1   控制台（产品板 CN1 预留口） */
extern UART_HandleTypeDef huart6; /* USART6 PC6/PC7   串口屏（产品板 CN2 座） */
extern UART_HandleTypeDef huart2; /* USART2 PD5/PD6   视觉 ROCK 5C（产品板 CN3 座，只收不发） */

void MX_USART1_UART_Init(void); /* 转盘 57 线 */
void MX_USART3_UART_Init(void); /* 电机 A 线 */
void MX_UART5_Init(void);       /* 电机 B 线 */
void MX_UART4_Init(void);       /* 控制台 */
void MX_USART6_UART_Init(void); /* 串口屏 */
void MX_USART2_UART_Init(void); /* 视觉 */

#ifdef __cplusplus
}
#endif

#endif
