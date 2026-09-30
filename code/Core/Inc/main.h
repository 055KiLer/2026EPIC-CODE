/**
  ******************************************************************************
  * @file    main.h
  * @brief   Header for main.c file (CubeMX 兼容结构，可被 .ioc 重生成保留 USER CODE)
  ******************************************************************************
  */
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* 引脚标签（对应原理图 2026-09-14 版）。开关常开 + 4.7k 上拉，压下 = 低电平 */
#define TRAY_ORIGIN_Pin GPIO_PIN_0
#define TRAY_ORIGIN_GPIO_Port GPIOB
#define X_LIMIT_Pin GPIO_PIN_1
#define X_LIMIT_GPIO_Port GPIOB
#define Y_LIMIT_Pin GPIO_PIN_5
#define Y_LIMIT_GPIO_Port GPIOC

void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif