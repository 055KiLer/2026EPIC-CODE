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
/* 投放点激光检测（2026-10-09 新增硬件）：上拉输入、**不接 EXTI**——判据是"投放窗口内捕获
   落料脉冲"（电平连续 ~2ms，见 sort_config.h 该节），没有沿语义。
   ★ 用户实测接线：**PD14**，检测到物块 = 低电平 */
#define LASER_PART_Pin GPIO_PIN_14
#define LASER_PART_GPIO_Port GPIOD

void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif