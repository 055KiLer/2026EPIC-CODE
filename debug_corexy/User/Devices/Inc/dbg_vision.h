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
 * @brief  ★`k230 raw [秒]`：线上真相工具——把字节按帧原样打出来（hex 12 字节 + 解码 + 校验判决）。
 *         旁路观察：不喂解析器、不动缓存与计数（`live`/快照的计数不受影响）。
 *         用途：`read` 报"没有可用的目标帧"时，用它区分线级三可能——
 *         ① 线上只有状态 0x00 的帧（上位机说"本帧无目标"）② 有目标帧但类别/形状越界
 *         （本机按设计丢弃，hex 里能看到那个数字）③ 乱字节/校验不过/没有字节（线/波特率/TX）。
 *         头 3 秒逐帧全打，之后只打目标帧/异常帧 + 每秒按状态分类的汇总。
 * @param  seconds 1~120，0 = 默认 10；任意键中止
 */
uint8_t Dbg_Vision_Raw(uint32_t seconds);

/**
 * @brief  `k230 sim <类别1~6> <x_mm> <y_mm> [形状0~3] [数量]`：本地合成一帧 12 字节
 *         （校验和自动算）从解析器入口喂进去——不接线也能验"解析 -> 缓存 -> 换算"整条链
 * @param  shape 缺省 0（默认方案）；1=立方体 2=12面体 3=六棱柱；给 4+ 可验越界路径
 * @param  count 数量位（下标 6，缺省 0）；给非 0 可一眼看到快照里的数量跟着变
 */
void Dbg_Vision_SimFeed(uint8_t category, uint8_t shape, float x_mm, float y_mm, uint8_t count);

/**
 * @brief  ★`k230 count [秒]`：**数量位观察**（2026-10-10）——只在数量/状态变化时打一行。
 *         拿一件货出去 / 放一件进来，数量位应当跟着变：这是"上位机填没填下标 6、填得对不对"
 *         的正面证据（产品侧的"两帧差 1"判据全靠它）。含**无目标帧**（托盘空的 0 也要看见）。
 * @param  seconds 1~120，0 = 默认 10；任意键中止
 */
uint8_t Dbg_Vision_CountWatch(uint32_t seconds);

/**
 * @brief  ★`k230 empty [秒]`：**空盘静默窗预演**（2026-10-10）——把产品那 5 秒的时间线
 *         放慢给你看：① 等你把最后一件拿走（数量首次读到 0 = 产品说的"读到零帧"，
 *         窗从这一刻起算）② 窗内 N 秒只观察不判定（数量变化照打、标「窗内忽略」）
 *         ③ 窗满后打印"产品从这里重新取帧"并给出窗后第一帧数量（= 下一件的发车帧）。
 *         用途：现场核"5 秒到底管不管用、从哪儿开始算"，也顺手证实上位机数量位是活的。
 * @param  seconds 1~120，0 = 缺省用产品的窗长（CFG_REFILL_HOLD_MS/1000）；任意键中止
 */
uint8_t Dbg_Vision_EmptyWindow(uint32_t seconds);

/* ★ 最近一帧的数量位（含无目标帧）：1 = 取到（清缓存以来已经有帧）  0 = 还没来帧。
   与"来了一帧、数量是 0"是两件事——产品 Vision_GetCount 同口径 */
uint8_t Dbg_Vision_GetCount(uint8_t *out);

/* 数量位那一帧的年龄（ms；还没来帧时无意义） */
uint32_t Dbg_Vision_CountAge(void);

/* 数量位那一帧的状态位：1=有目标 0=无目标 0xFF=还没来帧 */
uint8_t Dbg_Vision_CountState(void);

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

/* 目标帧**越界**累计数（类别不在 1~6、或形状不在 0~3）——"无新鲜帧"分诊用：
   越界 > 0 = 上位机其实在报目标，只是编码口径与产品不一致（不是"没识别到"） */
uint32_t Dbg_Vision_BadCount(void);

/* ★缓存快照（**不看新鲜度**）：缓存里记着目标就返回 1，并把目标与"多少 ms 前到的"写出来。
   与 Dbg_Vision_GetTarget 的区别 = 后者按 ≤CFG_K230_TARGET_FRESH_MS 过滤（= 产品取帧口径）；
   本接口只回答"清缓存前到底有没有目标、它多久前到的"——`read` 靠它把"流里刚有、现在断了"
   与"一直就没有可用目标帧"分开，再配 `k230`/`k230 raw` 定到具体哪一条 */
uint8_t Dbg_Vision_TargetSnapshot(K230_Target_t *out, uint32_t *age_ms);

/* 视觉系 -> 机械系换算（与产品 Vision_ToMachine 逐字同口径） */
void Dbg_Vision_ToMachine(const K230_Target_t *t, float *x_mm, float *y_mm);

#endif
