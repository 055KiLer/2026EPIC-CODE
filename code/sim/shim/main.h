/* 仿真替身：替代 CubeMX 生成的 Core/Inc/main.h */
#ifndef __MAIN_H
#define __MAIN_H
#include "../sim_shim.h"

/* 引脚标签必须与产品 Core/Inc/main.h 逐字一致：Config/sort_config.h 末尾的编译期核对
   会拿它们和 CFG_* 比对。仿真替身少了这些宏就变成"只有真机才检查"，
   而仿真恰恰是最该先发现引脚写错的地方（原理图 2026-09-14：X=PB1 Y=PC5 原点=PB0） */
#define TRAY_ORIGIN_Pin      GPIO_PIN_0
#define TRAY_ORIGIN_GPIO_Port GPIOB
#define X_LIMIT_Pin          GPIO_PIN_1
#define X_LIMIT_GPIO_Port    GPIOB
#define Y_LIMIT_Pin          GPIO_PIN_5
#define Y_LIMIT_GPIO_Port    GPIOC
/* 投放点激光检测（2026-10-09）：与产品 Core/Inc/main.h 逐字一致（PD14，检测到 = 低） */
#define LASER_PART_Pin       GPIO_PIN_14
#define LASER_PART_GPIO_Port GPIOD

#endif
