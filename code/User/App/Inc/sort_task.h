/**
 * @file    sort_task.h
 * @brief   分拣任务（App 层，电控文档 §3 流程的唯一实现处）
 */
#ifndef __SORT_TASK_H
#define __SORT_TASK_H

#include "main.h"

/* 硬件初始化已在 main() 中完成；本任务内完成回零等需要 RTOS 延时的动作 */
void SortTask(void *argument);

/* 判"没减一"的累计次数（2026-10-10 数量复核）：到账帧数量 ≠ 发车帧数量-1 的次数。
   屏上不体现（屏只有 6 行成功数），供调试链接口与仿真台核账用——
   产品自己的记账 vs 台架观测到的抓空次数，一比即可知复核逻辑有没有生效 */
uint16_t Sort_CountMissCount(void);

/* 到账帧等超时的累计次数（链路/上位机停发）：与"判没减一"分开计——
   前者是"没听到账"，后者是"听到了但账不对"，现场处置完全不同 */
uint16_t Sort_CountNoFrameCount(void);

/* 激光判"夹空"的累计次数（2026-10-09；现场已关闸，CFG_LASER_ENABLE=0 时恒 0） */
uint16_t Sort_LaserMissCount(void);

#endif