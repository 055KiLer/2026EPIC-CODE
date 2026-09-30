/**
  ******************************************************************************
  * @file    stm32f4xx_it.h
  * @brief   Interrupt handlers 原型
  * @note    SVC/PendSV/SysTick 由 FreeRTOS 内核与 cmsis_os2.c 提供，此处不声明
  ******************************************************************************
  */
#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);

void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void USART3_IRQHandler(void);
void UART4_IRQHandler(void);
void UART5_IRQHandler(void);
void USART6_IRQHandler(void);

void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void EXTI9_5_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif
