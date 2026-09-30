/**
 * @file    servo_out.h
 * @brief   舵机输出（夹爪 / 升降），本调试工程专用
 * @note    TIM4 50Hz、1MHz 计数 → 比较值就是脉宽微秒数。
 *          通道与默认档位在 Config/dbg_config.h（产品板：H1 = TIM4_CH4/PB9 夹爪、H4 = TIM4_CH3/PB8 升降）
 */
#ifndef __SERVO_OUT_H
#define __SERVO_OUT_H

#include "main.h"

typedef enum {
    DBG_SERVO_CLAW = 0, /* 夹爪 */
    DBG_SERVO_LIFT,     /* 升降 */
    DBG_SERVO_COUNT
} Dbg_Servo_e;

/* 档位编号按各舵机的语义：夹爪 0=张开 1=闭合 2=半开；升降 0=抬升 1=下伸 */
#define SERVO_POS_0 0u
#define SERVO_POS_1 1u
#define SERVO_POS_2 2u

const char *Servo_Out_NameOf(Dbg_Servo_e servo);
const char *Servo_Out_PosNameOf(Dbg_Servo_e servo, uint8_t pos);
uint8_t     Servo_Out_PosCount(Dbg_Servo_e servo);
uint16_t    Servo_Out_PosUs(Dbg_Servo_e servo, uint8_t pos);
uint16_t    Servo_Out_CurrentUs(Dbg_Servo_e servo);

/* 起 PWM 并进安全位（升降抬升 + 夹爪张开） */
void Servo_Out_Init(void);

/**
 * @brief  切到某个档位
 * @retval 1=已下发  0=档位越界（不改动）
 */
uint8_t Servo_Out_SetPos(Dbg_Servo_e servo, uint8_t pos);

/**
 * @brief  直接给脉宽（找机械行程用）
 * @retval 实际下发的脉宽（已按 DBG_CFG_SERVO_MIN/MAX_US 钳位）
 */
uint16_t Servo_Out_SetUs(Dbg_Servo_e servo, uint16_t pulse_us);

/* 停 PWM（舵机失去信号，不会再顶） */
void Servo_Out_Stop(Dbg_Servo_e servo);

#endif