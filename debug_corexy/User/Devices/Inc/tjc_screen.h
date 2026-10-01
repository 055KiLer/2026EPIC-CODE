/**
 * @file    tjc_screen.h
 * @brief   淘晶驰 TJC 串口屏指令封装（Device 层，指令尾 FF FF FF）
 * @note    与产品 `code/User/Devices/tjc_screen.*` 同名同 API（联调时两边行为可对照），
 *          仅发送路由换成调试工程的 USART6（dbg_uart 屏线）。屏端波特率须同为 115200
 *          （TJC 出厂默认 9600，要在屏工程里改，否则一条指令都收不到）。
 *          产品只用 Tjc_SetVal 写 6 行"成功数量"（n1~n6，类别号=行号）；SetText/SetPic
 *          是屏工程改版时的备用指令，这里一并提供以便台架验证
 */
#ifndef __TJC_SCREEN_H
#define __TJC_SCREEN_H

#include "main.h"

/* 均返回 1=整条指令（含帧尾）已交给 USART6  0=任一环节发送失败 */
uint8_t Tjc_SetText(const char *var, const char *val);
uint8_t Tjc_SetVal(const char *var, int32_t num);
uint8_t Tjc_SetPic(const char *var, int32_t pic_id);

#endif
