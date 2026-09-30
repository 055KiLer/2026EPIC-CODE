/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for the tim.c file
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
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *timHandle);

#ifdef __cplusplus
}
#endif

#endif
