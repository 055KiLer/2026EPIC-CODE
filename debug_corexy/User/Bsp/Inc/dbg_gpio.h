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

/* ★ 投放点激光检测（2026-10-09；引脚/极性/启用开关见 dbg_config.h §3.6）。
   ⚠️ 台架实测：这套模块一次落料给的是**极窄事件**（≈1 个采样点 / µs 级）⇒ 台架侧要靠
   **µs 级密采样**才捕得到（`Dbg_App_LaserCapture` / `Dbg_App_LaserVerifyDrop`）；
   产品侧是 **EXTI 下降沿计数**（同样的判定语义：窗口内出现过一次检测）。
   本文件只提供两个原子读口：
   Dbg_Gpio_LaserRaw  = 原始电平（1=高 0=低）：极性核对时看它——把"线接反"与"宏写反"分开
   Dbg_Gpio_LaserPart = 判定（1="检测到"，按 DBG_CFG_LASER_ACTIVE_LEVEL）
   未启用（DBG_CFG_LASER_ENABLE=0）时两个都返回 0，且**一个脚都不碰** */
uint8_t Dbg_Gpio_LaserRaw(void);
uint8_t Dbg_Gpio_LaserPart(void);

#endif
