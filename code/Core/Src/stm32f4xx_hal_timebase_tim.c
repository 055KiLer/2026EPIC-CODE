/**
  ******************************************************************************
  * @file    stm32f4xx_hal_timebase_tim.c
  * @brief   HAL 时基改用 TIM6（1ms），把 SysTick 让给 FreeRTOS 调度器
  * @attention
  * APB1 = 42MHz，APB1 定时器时钟 = 2*42 = 84MHz：
  * 预分频 84-1 -> 1MHz，重装载 1000-1 -> 1ms 中断
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"

/* Private variables ---------------------------------------------------------*/
static TIM_HandleTypeDef s_htim6;

/**
  * @brief  HAL 时基初始化（HAL_Init / HAL_RCC_ClockConfig 内部自动调用）
  * @param  TickPriority 时基中断优先级
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  RCC_ClkInitTypeDef clkconfig = { 0 };
  uint32_t uwTimclock = 0u;
  uint32_t uwPrescalerValue = 0u;
  uint32_t pFLatency = 0u;

  /* HAL 时基走最低优先级，绝不抢占任务关键中断 */
  if (TickPriority >= (1UL << __NVIC_PRIO_BITS))
  {
    return HAL_ERROR;
  }

  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, TickPriority, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  uwTickPrio = TickPriority;

  __HAL_RCC_TIM6_CLK_ENABLE();

  /* 定时器时钟：APB1 分频非 1 时定时器时钟 = 2 * PCLK1 */
  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);
  if (clkconfig.APB1CLKDivider == RCC_HCLK_DIV1)
  {
    uwTimclock = HAL_RCC_GetPCLK1Freq();
  }
  else
  {
    uwTimclock = 2UL * HAL_RCC_GetPCLK1Freq();
  }

  /* 1MHz 计数 -> 1ms 中断 */
  uwPrescalerValue = (uint32_t)((uwTimclock / 1000000U) - 1U);

  s_htim6.Instance = TIM6;
  s_htim6.Init.Prescaler = uwPrescalerValue;
  s_htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  s_htim6.Init.Period = (1000000U / 1000U) - 1U;
  s_htim6.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  s_htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&s_htim6) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_TIM_Base_Start_IT(&s_htim6) != HAL_OK)
  {
    return HAL_ERROR;
  }
  return HAL_OK;
}

/**
  * @brief Suspend Tick increment.
  */
void HAL_SuspendTick(void)
{
  __HAL_TIM_DISABLE(&s_htim6);
}

/**
  * @brief Resume Tick increment.
  */
void HAL_ResumeTick(void)
{
  __HAL_TIM_ENABLE(&s_htim6);
}

/**
  * @brief  TIM6 周期中断 -> HAL 时基 +1
  */
void TIM6_DAC_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&s_htim6);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
}