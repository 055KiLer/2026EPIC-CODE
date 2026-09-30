/**
  ******************************************************************************
  * @file    stm32f4xx_hal_msp.c
  * @brief   HAL MSP 初始化：外设时钟 + 引脚复用 + NVIC
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"

/**
  * @brief  Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_RCC_SYSCFG_CLK_ENABLE();
}

/**
  * @brief UART MSP 初始化（6 路）
  * @note  中断优先级抢占 5（低于限位 EXTI 抢占 0）。
  *        ⚠️ 这一组必须与 Config/sort_config.h 的 CFG_UART_* 注释逐字对应，
  *        接错了表现是"某个外设完全没反应"，而不是报错
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
    /* 转盘电机 M3 */
    irq = USART1_IRQn;
    gpio_tx = GPIOB; pin_tx = GPIO_PIN_6;
    gpio_rx = GPIOB; pin_rx = GPIO_PIN_7;
    alt = GPIO_AF7_USART1;
    __HAL_RCC_GPIOB_CLK_ENABLE();
  }
  else if (uartHandle->Instance == USART2)
  {
    /* 视觉上位机 ROCK 5C CN3（本侧只收不发，TX 仍按原理图接出） */
    irq = USART2_IRQn;
    gpio_tx = GPIOD; pin_tx = GPIO_PIN_5;
    gpio_rx = GPIOD; pin_rx = GPIO_PIN_6;
    alt = GPIO_AF7_USART2;
    __HAL_RCC_GPIOD_CLK_ENABLE();
  }
  else if (uartHandle->Instance == USART3)
  {
    /* CoreXY A 电机 M1 */
    irq = USART3_IRQn;
    gpio_tx = GPIOC; pin_tx = GPIO_PIN_10;
    gpio_rx = GPIOC; pin_rx = GPIO_PIN_11;
    alt = GPIO_AF7_USART3;
    __HAL_RCC_GPIOC_CLK_ENABLE();
  }
  else if (uartHandle->Instance == UART4)
  {
    /* 预留口 CN1，无设备 */
    irq = UART4_IRQn;
    gpio_tx = GPIOA; pin_tx = GPIO_PIN_0;
    gpio_rx = GPIOA; pin_rx = GPIO_PIN_1;
    alt = GPIO_AF8_UART4;
    __HAL_RCC_GPIOA_CLK_ENABLE();
  }
  else if (uartHandle->Instance == UART5)
  {
    /* CoreXY B 电机 M2 */
    irq = UART5_IRQn;
    gpio_tx = GPIOC; pin_tx = GPIO_PIN_12;
    gpio_rx = GPIOD; pin_rx = GPIO_PIN_2;
    alt = GPIO_AF8_UART5;
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
  }
  else if (uartHandle->Instance == USART6)
  {
    /* TJC 串口屏 CN2 */
    irq = USART6_IRQn;
    gpio_tx = GPIOC; pin_tx = GPIO_PIN_6;
    gpio_rx = GPIOC; pin_rx = GPIO_PIN_7;
    alt = GPIO_AF8_USART6;
    __HAL_RCC_GPIOC_CLK_ENABLE();
  }
  else
  {
    return;
  }

  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_USART2_CLK_ENABLE();
  __HAL_RCC_USART3_CLK_ENABLE();
  __HAL_RCC_UART4_CLK_ENABLE();
  __HAL_RCC_UART5_CLK_ENABLE();
  __HAL_RCC_USART6_CLK_ENABLE();

  /* 引脚复用 */
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

/**
  * @brief TIM_BASE MSP 初始化（TIM4 时钟，舵机 PWM）
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *tim_baseHandle)
{
  if (tim_baseHandle->Instance == TIM4)
  {
    __HAL_RCC_TIM4_CLK_ENABLE();
  }
}

/**
  * @brief TIM_PWM MSP 初始化（TIM4，含引脚复用）
  */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *tim_pwmHandle)
{
  if (tim_pwmHandle->Instance == TIM4)
  {
    __HAL_RCC_TIM4_CLK_ENABLE();
    HAL_TIM_MspPostInit(tim_pwmHandle);
  }
}