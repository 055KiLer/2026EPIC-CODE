/**
  ******************************************************************************
  * @file    main.h
  * @brief   调试工程主头文件（产品板 / STM32F407VET6，2026-09-30 起接线与产品工程一致）
  ******************************************************************************
  */
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

void Error_Handler(void);

/* 下面这三个引脚标签本工程不用（这块板没接限位开关），
   保留只是为了 Config/sort_config.h（产品那份**原样**拷来验算法的副本）末尾的
   编译期核对能通过 —— 改它们没有意义 */
#define TRAY_ORIGIN_Pin GPIO_PIN_0
#define TRAY_ORIGIN_GPIO_Port GPIOB
#define X_LIMIT_Pin GPIO_PIN_1
#define X_LIMIT_GPIO_Port GPIOB
#define Y_LIMIT_Pin GPIO_PIN_5
#define Y_LIMIT_GPIO_Port GPIOC

#ifdef __cplusplus
}
#endif

#endif