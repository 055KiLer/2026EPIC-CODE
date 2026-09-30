/**
  ******************************************************************************
  * @file    tim.c
  * @brief   TIM4 双路舵机 PWM：50Hz、1MHz 计数（比较值 = 脉宽微秒数）
  * @note    通道与引脚（2026-09-30 起与产品板一致）：CH3 = PB8（产品板 H4，升降）、
  *          CH4 = PB9（产品板 H1，夹爪）—— 见 Config/dbg_config.h
  *          依据：产品 code/Core/Src/tim.c 与嘉立创原理图 V1.0（H1=PB9=夹爪、H4=PB8=升降）；
  *          旧「物创1」板的 PD12=CH1/PD15=CH4 接线随 PCB 修复作废
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "tim.h"

TIM_HandleTypeDef htim4;

void MX_TIM4_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
  TIM_MasterConfigTypeDef sMasterConfig = { 0 };
  TIM_OC_InitTypeDef sConfigOC = { 0 };

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;          /* APB1 定时器时钟 84MHz / 84 = 1MHz，1 tick = 1us */
  htim4.Init.Period = 19999;          /* 1MHz / 20000 = 50Hz 舵机标准周期 */
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;                /* 上电先不给脉冲，由 Servo_Out_Init 置安全位 */
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  /* ★ 必须显式调用（CubeMX 生成代码本有这一行，手搬时丢了）：
     上面先走的 HAL_TIM_Base_Init 已把 htim4.State 置成 READY，
     后面的 HAL_TIM_PWM_Init 因 State != RESET 跳过 HAL_TIM_PWM_MspInit
     → MspPostInit 永不执行 → PB8/PB9 停留浮空输入，舵机收不到任何脉冲。
     2026-09-25 台架实测：TIM4 正常计数、CCR 已写值，但 GPIO MODER=0 */
  HAL_TIM_MspPostInit(&htim4);
}

/**
  * @brief  PWM 引脚复用（★ 由 MX_TIM4_Init 末尾显式调用——HAL_TIM_PWM_MspInit 会因
  *         State 已被 Base_Init 置 READY 而被跳过，不能指望它走到这里）
  * @note   改通道必须同时改这里与 dbg_config.h：这里配错的表现是"舵机一点反应都没有"，
  *         而 PWM 定时器其实在正常计数，账面看不出来
  */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *timHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };

  if (timHandle->Instance == TIM4)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;   /* CH3 = PB8(H4 升降), CH4 = PB9(H1 夹爪) */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}