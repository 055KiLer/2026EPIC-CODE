/**
 * @file    bsp_gpio.h
 * @brief   限位/原点开关：电平查询 + EXTI 事件锁存（Bsp 层）
 * @note    ISR 只置位与计数（无发送、无延时、无 RTOS 调用），动作由守护任务取走，
 *          时延上界明确（1 个 tick）。本文件不感知任何电机/设备模块
 */
#ifndef __BSP_GPIO_H
#define __BSP_GPIO_H

#include "main.h"

typedef enum {
    BSP_GPIO_EVT_NONE       = 0,
    BSP_GPIO_EVT_XY_LIMIT   = (1u << 0), /* CoreXY 双轴须立即停止 */
    BSP_GPIO_EVT_TRAY_INDEX = (1u << 1)  /* 位置标定，不是急停 */
} Bsp_Gpio_Event_e;

/* 三个开关：1 = 触发（常开 + 上拉，压下读作 0） */
uint8_t Bsp_Gpio_LimitX(void);
uint8_t Bsp_Gpio_LimitY(void);
uint8_t Bsp_Gpio_TrayOrigin(void);

/* ★ 投放点激光检测（2026-10-09）：两条读口（判据与处置见 sort_config.h 该节）。
   Bsp_Gpio_LaserPart()  = 1 = 此刻读到"检测到"电平。模块输出若是**稳态电平**用它；
                           ⚠️ 台架实测这套模块给的是**极窄事件**（≈1µs，落了就回），
                           1ms 级的电平轮询抓不到它 —— 那种情形靠下面这条
   Bsp_Gpio_LaserEdges() = "检测到"的**下降沿**累计数（EXTI14 里 ++，只计数，ISR 不做别的）。
                           取证据 = "到点记下基准，窗口内看它涨没涨"（快照比大小，
                           不需要清零、也不怕窗口外的沿进来） */
uint8_t  Bsp_Gpio_LaserPart(void);
uint32_t Bsp_Gpio_LaserEdges(void);

/**
 * @brief  取走并清除累积的 EXTI 事件
 * @retval 位掩码；同种事件在读取前多次触发会合并（对急停语义无损）
 */
uint32_t Bsp_Gpio_TakeEvents(void);

/* 上电以来转盘原点沿总次数（供 App 与理论整圈数比对，检出丢步） */
uint32_t Bsp_Gpio_TrayIndexCount(void);

/**
 * @brief  X/Y 限位消抖状态的松开确认（安全守护任务每圈调用）。
 *         连续 CFG_LIMIT_DEBOUNCE_MS 松开才重新武装按压判定——挡掉 17~128 沿/按压
 *         的触点弹跳与松开沿（2026-09-26 台架实测），防误触发急停
 */
void Bsp_Gpio_Poll(void);

#endif