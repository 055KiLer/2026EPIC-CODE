/**
 * @file    dbg_console.h
 * @brief   串口命令控制台：收一行、解析、执行
 * @note    为什么做成交互式而不是"改代码-重烧-再看"：台上要试的组合很多
 *          （方向/地址/速度/脉冲/舵机行程），一次重烧一分钟；敲命令是秒级。
 */
#ifndef __DBG_CONSOLE_H
#define __DBG_CONSOLE_H

#include "main.h"

/* 处理已收到的字节（在超循环里反复调用即可，非阻塞） */
void Dbg_Console_Poll(void);

/* 打印命令表（上电也会自动打一次） */
void Dbg_Console_PrintHelp(void);

/* 上电以来有没有收到过任何字节：自动体检用它在"没人能敲命令"时决定要不要重跑 */
uint8_t Dbg_Console_HasEverReceived(void);

#endif