/**
  ******************************************************************************
  * @file    stm32f4xx_hal_msp.c
  * @brief   HAL MSP：四路 UART + TIM4 的时钟/引脚/NVIC
  * @note    引脚与产品工程逐脚一致（2026-09-30 PCB 修复后同步）：
  *          USART1=PB6/PB7(M3 转盘)、USART3=PC10/PC11(M1 电机A)、UART5=PC12/PD2(M2 电机B)、
  *          UART4=PA0/PA1(CN1 控制台)
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"

void HAL_MspInit(void)
{
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_RCC_SYSCFG_CLK_ENABLE();
}

/**
  * @brief UART MSP 初始化
  * @note  中断优先级抢占 5：控制台接收用中断，别的都轮询
  */
void HAL_UART_MspInit(UART_HandleTypeDef *uartHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  IRQn_Type irq;
  GPIO_TypeDef *gpio_tx = NULL;
  GPIO_TypeDef *gpio_rx = NULL;
  uint16_t pin_tx = 0;
  uint16_t pin_rx = 0;
  uint8_t alt = 0;

  if (uartHandle->Instance == USART1)
  {
    /* 转盘 57 线（产品板 M3 座） */
    irq = USART1_IRQn;
    gpio_tx = GPIOB; pin_tx = GPIO_PIN_6;
    gpio_rx = GPIOB; pin_rx = GPIO_PIN_7;
    alt = GPIO_AF7_USART1;
    __HAL_RCC_GPIOB_CLK_ENABLE();
  }
  else if (uartHandle->Instance == USART3)
  {
    /* 电机 A 线（产品板 M1 座） */
    irq = USART3_IRQn;
    gpio_tx = GPIOC; pin_tx = GPIO_PIN_10;
    gpio_rx = GPIOC; pin_rx = GPIO_PIN_11;
    alt = GPIO_AF7_USART3;
    __HAL_RCC_GPIOC_CLK_ENABLE();
  }
  else if (uartHandle->Instance == UART5)
  {
    /* 电机 B 线（产品板 M2 座） */
    irq = UART5_IRQn;
    gpio_tx = GPIOC; pin_tx = GPIO_PIN_12;
    gpio_rx = GPIOD; pin_rx = GPIO_PIN_2;
    alt = GPIO_AF8_UART5;
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
  }
  else if (uartHandle->Instance == UART4)
  {
    /* 控制台（产品板 CN1 预留口） */
    irq = UART4_IRQn;
    gpio_tx = GPIOA; pin_tx = GPIO_PIN_0;
    gpio_rx = GPIOA; pin_rx = GPIO_PIN_1;
    alt = GPIO_AF8_UART4;
    __HAL_RCC_GPIOA_CLK_ENABLE();
  }
  else
  {
    return;
  }

  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_USART3_CLK_ENABLE();
  __HAL_RCC_UART4_CLK_ENABLE();
  __HAL_RCC_UART5_CLK_ENABLE();

  GPIO_InitStruct.Pin = pin_tx;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = alt;
  HAL_GPIO_Init(gpio_tx, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = pin_rx;
  HAL_GPIO_Init(gpio_rx, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(irq, 5, 0);
  HAL_NVIC_EnableIRQ(irq);
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *tim_baseHandle)
{
  if (tim_baseHandle->Instance == TIM4)
  {
    __HAL_RCC_TIM4_CLK_ENABLE();
  }
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *tim_pwmHandle)
{
  if (tim_pwmHandle->Instance == TIM4)
  {
    HAL_TIM_MspPostInit(tim_pwmHandle);
  }
}