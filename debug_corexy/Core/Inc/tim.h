/**
  ******************************************************************************
  * @file    tim.h
  * @brief   TIM4 双路舵机 PWM（50Hz，1MHz 计数 → 比较值 = 脉宽 µs）
  ******************************************************************************
  */
#ifndef __TIM_H
#define __TIM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern TIM_HandleTypeDef htim4;

void MX_TIM4_Init(void);

/* PWM 引脚复用（定义在 tim.c，由 HAL_TIM_PWM_MspInit 调用） */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *timHandle);

#ifdef __cplusplus
}
#endif

#endif