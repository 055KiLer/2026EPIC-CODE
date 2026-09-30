/**
 * @file    dbg_gpio.h
 * @brief   限位/原点微动开关：电平 + 下降沿计数
 * @note    引脚在 dbg_config.h §3.5，与产品工程同脚（PB0/PB1/PC5，EXTI 下降沿 + 上拉）。
 *          极性：未触发=1、压下=0；沿计数只数"压下的下降沿"，与产品回零的判据同一条。
 */
#ifndef __DBG_GPIO_H
#define __DBG_GPIO_H

#include <stdint.h>

typedef enum {
    DBG_LIMIT_X = 0,
    DBG_LIMIT_Y,
    DBG_LIMIT_TRAY,
    DBG_LIMIT_COUNT
} Dbg_Limit_e;

void Dbg_Gpio_Init(void);
void Dbg_Gpio_Poll(void); /* 主循环每圈调用：维护"松开确认"，让沿计数=按压次数（只认压下） */

/* 裸下降沿（无消抖无闩锁，每个弹跳沿都记）：回零"触发即停"用。
   取走即清，返回沿时刻（HAL_GetTick 口径）；0 = 自上次取走后没有新沿 */
uint32_t Dbg_Gpio_TakeRawEdge(Dbg_Limit_e which);
void     Dbg_Gpio_ClearRawEdges(void);

const char *Dbg_Gpio_NameOf(Dbg_Limit_e which); /* 引脚名（"PB1" 等，来自 dbg_config.h） */
const char *Dbg_Gpio_RoleOf(Dbg_Limit_e which); /* 用途名（"X限位" 等） */

/* 电平：1=未触发（开） 0=压下 */
uint8_t Dbg_Gpio_Level(Dbg_Limit_e which);

/* 压下（下降沿）累计次数与最近一次沿的时刻（HAL_GetTick 口径，ms） */
uint32_t Dbg_Gpio_EdgeCount(Dbg_Limit_e which);
uint32_t Dbg_Gpio_LastEdgeTick(Dbg_Limit_e which);
void     Dbg_Gpio_ClearEdges(void);

#endif
