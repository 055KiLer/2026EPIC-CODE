/**
 * @file    dbg_print.h
 * @brief   调试打印：行缓冲 + 控制台输出（可选镜像到 ITM/SWO）
 * @note    为什么不用 printf：本工程未开 MicroLib，AC6 标准库重定向要动启动代码与
 *          __use_no_semihosting；自带格式器只认 %s %c %u %d %x %X %%（**不支持 %f**：
 *          要小数就按整数放大后打印，例如 mm 打 100 倍）。
 *          ⚠️ 不要在故障处理（HardFault 等）里打印：发送靠 HAL_GetTick 计超时，
 *          故障时 tick 已停 → 会卡在发送里。
 */
#ifndef __DBG_PRINT_H
#define __DBG_PRINT_H

#include "main.h"

#define DBG_PRINT_LINE_MAX 128u

void Dbg_Print_Init(void);

/* 攒满一行或遇到 '\n' 才真正发出（一次发一整行，省掉每字符一次 HAL 调用） */
void Dbg_Printf(const char *fmt, ...);

/* 强制把未成行的内容发出 */
void Dbg_Print_Flush(void);

#endif