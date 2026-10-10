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
  /* ★ 投放点激光检测（2026-10-09）：上拉 + **下降沿 EXTI**（检测到物块 = 拉到低，用户实测）。
     ⚠️ 为什么必须走 EXTI（不是普通输入）：台架 `laser drop` 实测一次落料 ≈ **1 个采样点宽
     （µs 级）**，1ms 级的电平轮询会漏掉 99.9% ⇒ 产品用 PD14 = EXTI14 的下降沿计数
     （bsp_gpio.c 的 Bsp_Gpio_LaserEdges），App 在投放窗口里"快照比大小"取证据
     （判据见 sort_config.h 该节、sort_task.c 的 Sort_LaserEvidence）。
     ⚠️ 极性若改成高有效：这里要一起改成上升沿（GPIO_MODE_IT_RISING），两处同改。
     ⚠️ .ioc 里也要把该脚标成 GPIO_Input + 下降沿 EXTI，否则重生成后这里能跑、引脚表对不上。
     ★ 实测接线 = PD14；GPIOD 的时钟在本函数开头已开 */
  GPIO_InitStruct.Pin  = LASER_PART_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LASER_PART_GPIO_Port, &GPIO_InitStruct);

  /* PD14 = EXTI14 ⇒ EXTI15_10 那条中断线（处理函数在 stm32f4xx_it.c）。
     优先级与限位同一档（沿不能丢），高于串口 */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */