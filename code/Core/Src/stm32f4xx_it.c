/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines
  * @attention
  * SVC_Handler / PendSV_Handler / SysTick_Handler 由 FreeRTOS 端口与 cmsis_os2.c 提供，
  * 本文件禁止再定义这三个函数；HAL 时基中断在 stm32f4xx_hal_timebase_tim.c
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
#include "usart.h"
#include "zdt_motor.h" /* 故障路径先停电机（A 组遗留 #3）*/

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* 故障里 tick 可能已停，所以用 Zdt_PanicStopAll（不依赖 HAL/tick）先停电机：
     转盘是速度模式持续转的，停不下来会一直转到断电（见《调试与推进计划》A 组 #3）*/
  Zdt_PanicStopAll();
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* 故障里 tick 可能已停，所以用 Zdt_PanicStopAll（不依赖 HAL/tick）先停电机：
     转盘是速度模式持续转的，停不下来会一直转到断电（见《调试与推进计划》A 组 #3）*/
  Zdt_PanicStopAll();
  while (1)
  {
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* 故障里 tick 可能已停，所以用 Zdt_PanicStopAll（不依赖 HAL/tick）先停电机：
     转盘是速度模式持续转的，停不下来会一直转到断电（见《调试与推进计划》A 组 #3）*/
  Zdt_PanicStopAll();
  while (1)
  {
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* 故障里 tick 可能已停，所以用 Zdt_PanicStopAll（不依赖 HAL/tick）先停电机：
     转盘是速度模式持续转的，停不下来会一直转到断电（见《调试与推进计划》A 组 #3）*/
  Zdt_PanicStopAll();
  while (1)
  {
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* 故障里 tick 可能已停，所以用 Zdt_PanicStopAll（不依赖 HAL/tick）先停电机：
     转盘是速度模式持续转的，停不下来会一直转到断电（见《调试与推进计划》A 组 #3）*/
  Zdt_PanicStopAll();
  while (1)
  {
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
}

/* 6 路串口：USART1 转盘 / USART2 视觉上位机(ROCK 5C) / USART3 电机A / UART4 预留 / UART5 电机B / USART6 屏 */
void USART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart1);
}

void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart2);
}

void USART3_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart3);
}

void UART4_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart4);
}

void UART5_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart5);
}

void USART6_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart6);
}

/* EXTI0 = 转盘原点，EXTI1 = X 限位，EXTI9_5 = Y 限位 */
void EXTI0_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(TRAY_ORIGIN_Pin);
}

void EXTI1_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(X_LIMIT_Pin);
}

void EXTI9_5_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(Y_LIMIT_Pin);
}