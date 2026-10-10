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

/* 下面这些引脚标签本工程**不直接用**（接线走 dbg_config.h 的 DBG_CFG_* 表），
   保留只是为了 Config/sort_config.h（**薄壳直含产品那份**）末尾的编译期核对能通过。
   改接线改 dbg_config.h；改这里没有意义 */
#define TRAY_ORIGIN_Pin GPIO_PIN_0
#define TRAY_ORIGIN_GPIO_Port GPIOB
#define X_LIMIT_Pin GPIO_PIN_1
#define X_LIMIT_GPIO_Port GPIOB
#define Y_LIMIT_Pin GPIO_PIN_5
#define Y_LIMIT_GPIO_Port GPIOC
#define LASER_PART_Pin GPIO_PIN_14
#define LASER_PART_GPIO_Port GPIOD

#ifdef __cplusplus
}
#endif

#endif