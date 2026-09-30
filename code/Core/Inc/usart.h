/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for the usart.c file
  ******************************************************************************
  */
#ifndef __USART_H
#define __USART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 外设 -> 连接器 的对应关系以 Config/sort_config.h 顶部为准，这里只是句柄声明 */
extern UART_HandleTypeDef huart1;  /* USART1：ZDT 转盘电机 M3    */
extern UART_HandleTypeDef huart2;  /* USART2：视觉上位机 ROCK 5C  CN3 */
extern UART_HandleTypeDef huart3;  /* USART3：ZDT 电机 A   M1    */
extern UART_HandleTypeDef huart4;  /* UART4：预留          CN1   */
extern UART_HandleTypeDef huart5;  /* UART5：ZDT 电机 B    M2    */
extern UART_HandleTypeDef huart6;  /* USART6：TJC 串口屏   CN2   */

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_UART4_Init(void);
void MX_UART5_Init(void);
void MX_USART6_UART_Init(void);

#ifdef __cplusplus
}
#endif

#endif