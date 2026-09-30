/**
 * @file    k230_uart.h
 * @brief   视觉帧解析（Devices 层，纯字节流解析器）
 * @note    对端 = 视觉上位机 **ROCK 5C**（2026-09-22 由庐山派 K230 改型；`k230_*` 是历史命名，
 *          协议与上位机型号无关）。上位机侧的串口/系统要求见电控文档 §4.1"上位机侧要求"表
 *          帧：AA 55 | 状态 | 类别 形状 保留 保留 X_L X_H Y_L Y_H | 校验，共 12 字节
 *          状态 1=有目标 0=无目标；X/Y 小端 int16，单位 0.1mm
 *          ★ 2026-09-30 口径：**类别 ID 1~6** 放在原"颜色"字节里（上位机不再拆颜色×形状），
 *            原"形状"字节**固定发 0**；hasQR 保持解析但电控不使用（货物表面二维码属模拟污渍）
 *          本模块只做 字节流 -> 帧，串口取字节与目标缓存归 App/vision_task
 */
#ifndef __K230_UART_H
#define __K230_UART_H

#include "main.h"

typedef struct {
    uint8_t valid;    /* 1 = 当前帧有有效目标 */
    uint8_t category; /* 货物类别 ID 1~6（帧的原颜色字节） */
    uint8_t shape;    /* 保留字节（原形状字节）：上位机固定发 0，电控只用来判"口径是否一致" */
    uint8_t has_qr;   /* 电控忽略 */
    float   x_mm;
    float   y_mm;
} K230_Target_t;

typedef enum {
    K230_EV_NONE = 0, /* 字节已消费，未构成完整帧（或校验失败被丢弃） */
    K230_EV_TARGET,   /* 一帧有效目标（frame_out 已填充） */
    K230_EV_EMPTY     /* 一帧"无目标" */
} K230_Event_e;

/**
 * @brief  喂 1 字节给解析器
 * @param  frame_out EV_TARGET 时输出目标，可为 NULL
 * @retval 本字节触发的事件类型
 */
K230_Event_e K230_FeedByte(uint8_t byte, K230_Target_t *frame_out);

#endif