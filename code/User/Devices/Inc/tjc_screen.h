/**
 * @file    tjc_screen.h
 * @brief   淘晶驰 TJC 串口屏指令封装（Device 层，指令尾 FF FF FF）
 * @note    屏端波特率需与 USART6 一致（建议双方 115200）。
 *          当前只用 Tjc_SetVal 写 6 行"成功数量"（屏上名称/图片是屏工程画死的）；
 *          SetText/SetPic 留着给屏工程改版时用，暂无调用方
 */
#ifndef __TJC_SCREEN_H
#define __TJC_SCREEN_H

#include "main.h"

void Tjc_Init(void);

/* 均返回 1=整条指令（含帧尾）已交给外设  0=任一环节发送失败 */
uint8_t Tjc_SetText(const char *var, const char *val);
uint8_t Tjc_SetVal(const char *var, int32_t num);
uint8_t Tjc_SetPic(const char *var, int32_t pic_id);

#endif