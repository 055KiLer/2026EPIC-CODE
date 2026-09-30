/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   GPIO 初始化（三个限位/原点开关 + EXTI）
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  初始化 GPIO：X_LIMIT=PB1(EXTI1)  Y_LIMIT=PC5(EXTI9_5)  TRAY_ORIGIN=PB0(EXTI0)
  * @note   开关常开 + 4.7k 上拉到 3V3，未触发=高、压下=低，所以触发沿是下降沿。
  *         改成上升沿会让回零永远等不到触发
  */
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };

  /* GPIO 时钟 */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* EXTI 中断优先级（抢占 0，高于串口） */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* PB0 TRAY_ORIGIN / PB1 X_LIMIT */
  GPIO_InitStruct.Pin = TRAY_ORIGIN_Pin | X_LIMIT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* PC5 Y_LIMIT */
  GPIO_InitStruct.Pin = Y_LIMIT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Y_LIMIT_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */