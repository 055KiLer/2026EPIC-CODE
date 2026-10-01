/**
 * @file    servo.h
 * @brief   升降/夹爪舵机（Device 层，TIM4 50Hz）
 * @note    通道由原理图固定：升降 = PB8 = TIM4_CH3，夹爪 = PB9 = TIM4_CH4
 */
#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"

#define SERVO_LIFT_CHANNEL TIM_CHANNEL_3 /* PB8 */
#define SERVO_CLAW_CHANNEL TIM_CHANNEL_4 /* PB9 */

typedef enum {
    SERVO_LIFT = 0,
    SERVO_CLAW
} Servo_ID_e;

typedef enum {
    SERVO_LIFT_UP = 0, /* 运输/投放 */
    SERVO_LIFT_DOWN    /* 抓取/轻放 */
} Servo_LiftPOS_e;

typedef enum {
    SERVO_CLAW_OPEN = 0, /* 投放释放 */
    SERVO_CLAW_CLOSE,    /* 夹紧（兜底值；抓取流程按形状走 CFG_GRIP_CLOSE_US） */
    SERVO_CLAW_HALF      /* 作废时就地轻放（兜底值；抓取流程按形状走 CFG_GRIP_HALF_US） */
} Servo_ClawPOS_e;

/* 启动两路 PWM 并进上电安全位（抬升 + 张开） */
void Servo_Init(void);

/* 切档；越界入参直接忽略（绝不把舵机开到未定义角度） */
void Servo_LiftSet(Servo_LiftPOS_e pos);
void Servo_ClawSet(Servo_ClawPOS_e pos);

/* 夹爪原始脉宽（µs）：抓取方案表按形状给值用；写入前按 CFG_SERVO_MIN/MAX_US 钳位 */
void Servo_ClawSetUs(uint16_t pulse_us);

#endif