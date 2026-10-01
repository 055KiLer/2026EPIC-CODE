/**
 * @file    dbg_vision.h
 * @brief   视觉链路调试模块：USART2 收字节 -> 产品解析器副本 -> 缓存/计数 + 坐标换算
 * @note    解析器 `k230_uart.c/h` 是**产品源码原样副本**（无 OS 依赖，直接拷贝）；
 *          本模块只做产品 vision_task.c 的"轮询版"缓存（裸机没有任务，主循环每圈 Poll）。
 *          ★ 这里验证的是**协议口径与坐标换算**（对端发什么、换算成机械系哪个点），
 *            不驱动任何运动——收到目标也不会去抓，控制台只打印
 */
#ifndef __DBG_VISION_H
#define __DBG_VISION_H

#include "main.h"
#include "k230_uart.h"

/* 主循环每圈调用：把 USART2 环形缓冲抽干喂进解析器并更新缓存 */
void Dbg_Vision_Poll(void);

/* 清缓存与计数器（`k230 clr`），并重新武装该线接收 */
void Dbg_Vision_Flush(void);

/* `k230`：打印缓存快照 + 换算出的机械坐标 + 收帧计数 + 换算配置（原点/符号） */
void Dbg_Vision_Print(void);

/**
 * @brief  `k230 live [秒]`：实时流。有目标帧逐条打印；无目标每秒汇总一行。
 *         途中控制台来任意字节即中止（字节被丢弃，不会排进命令队列）
 * @param  seconds 1~120，0 = 默认 10
 * @retval 1（正常结束或被中止）
 */
uint8_t Dbg_Vision_Live(uint32_t seconds);

/**
 * @brief  `k230 sim <类别1~6> <x_mm> <y_mm> [形状0~3]`：本地合成一帧 12 字节（校验和自动算）
 *         从解析器入口喂进去——不接线也能验"解析 -> 缓存 -> 换算"整条链
 * @param  shape 缺省 0（默认方案）；1=立方体 2=12面体 3=六棱柱；给 4+ 可验越界路径
 */
void Dbg_Vision_SimFeed(uint8_t category, uint8_t shape, float x_mm, float y_mm);

/* `stat` 的 [VISION] 行：链路活性 + 三类收帧计数 */
void Dbg_Vision_PrintStatusLine(void);

/* ---- 给 App 层"读帧位取目标"（Dbg_App_ReadTarget）用的查询接口 ---- */

/* 链路活性：CFG_K230_LINK_DEAD_MS 内有校验通过帧 = 1（口径 = 产品 Vision_LinkAlive） */
uint8_t Dbg_Vision_LinkAlive(void);

/* 取缓存里"有效且新鲜（≤CFG_K230_TARGET_FRESH_MS）"的目标；不消费——台架没有分拣任务，
   同一目标重复取会重复给出，拿走货物/上位机换目标后自然更新 */
uint8_t Dbg_Vision_GetTarget(K230_Target_t *out);

/* 无目标帧累计数（清缓存后重新计）——等待目标时的进度显示用 */
uint32_t Dbg_Vision_NoneCount(void);

/* 视觉系 -> 机械系换算（与产品 Vision_ToMachine 逐字同口径） */
void Dbg_Vision_ToMachine(const K230_Target_t *t, float *x_mm, float *y_mm);

#endif
