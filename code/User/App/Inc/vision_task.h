/**
 * @file    vision_task.h
 * @brief   视觉任务（App 层）：K230 帧解析 + 最新目标缓存 + 可信度判定
 * @note    K230 端负责目标挑选/去重/坐标标定；本任务只做三件事：
 *            1. 消费串口字节 -> 缓存最新一帧
 *            2. 打上"新鲜度"与"已消费"标记，保证一帧目标只被抓一次
 *            3. 把 链路活性 / 托盘已空 / 有可用目标 分别回答，绝不混为一谈
 *          必须分开的理由：K230 掉线时"无目标帧计数"会停住而不是继续增长，
 *          只看计数会永远等下去（现场表现 = 机器静止但没有任何指示）
 */
#ifndef __VISION_TASK_H
#define __VISION_TASK_H

#include "main.h"
#include "k230_uart.h"

void VisionTask(void *argument);

/**
 * @brief  领取一个可分拣的目标（取走即标记已消费）
 * @retval 1=取到"未消费且仍新鲜"的目标  0=没有可用目标
 * @note   物体被抓走后 K230 若仍在原坐标报目标，需它自己停发，否则靠 FRESH_MS 兜底
 */
uint8_t Vision_GetTarget(K230_Target_t *out);

/* 视觉链路是否活着（最近有校验通过的帧） */
uint8_t Vision_LinkAlive(void);

/**
 * @brief  ★ 最近一帧的**数量位**（2026-10-10 数量复核用；含状态 00 的无目标帧）
 * @param  out 数量 0~255
 * @retval 1=取到（自上次 Vision_Flush 之后已经来过校验通过的帧）  0=还没来帧/链路超时
 * @note   返回 0 = "还没来帧"，与"来了一帧、数量是 0"是两件事：
 *         清缓存后的等待循环靠这个区分，绝不要把"还没来帧"当"托盘空了"
 */
uint8_t Vision_GetCount(uint8_t *out);

/* 是否判定"托盘已空、本轮分拣完成"；链路死了绝不返回 1（那是故障不是完成） */
uint8_t Vision_TrayEmpty(void);

/* 清空串口缓冲与缓存，并以当前时刻重新起算链路活性（回零完成后调用） */
void Vision_Flush(void);

/* K230 托盘坐标 -> 机械坐标：两轴交换（视觉 x -> 机械 Y、视觉 y -> 机械 X）+ 视觉原点平移。
   四个标定常量（原点绝对坐标 + 两个符号）见 sort_config.h 的 CFG_VISION_* */
void Vision_ToMachine(const K230_Target_t *target, float *x_mm, float *y_mm);

#endif