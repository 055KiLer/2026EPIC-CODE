/**
 * @file    servo.c
 * @brief   舵机 PWM：1MHz 计数（预分频 83），CCR 直接写微秒脉宽。档位在 sort_config.h 标定
 * @note    写成 CH1/CH2 会把 PWM 发到 PB6/PB7（那两脚是转盘电机的 USART1），
 *          表现为"两个舵机都不动、转盘串口收噪声"
 */
#include "servo.h"
#include "sort_config.h"
#include "tim.h"

static const uint16_t s_lift_us[2] = {CFG_LIFT_UP_US, CFG_LIFT_DOWN_US};
static const uint16_t s_claw_us[3] = {CFG_CLAW_OPEN_US, CFG_CLAW_CLOSE_US, CFG_CLAW_HALF_US};

/* 1 tick = 1us */
static void Servo_Write(Servo_ID_e id, uint16_t pulse_us) {
    uint32_t channel = (id == SERVO_LIFT) ? SERVO_LIFT_CHANNEL : SERVO_CLAW_CHANNEL;

    __HAL_TIM_SET_COMPARE(&CFG_SERVO_TIM, channel, pulse_us);
}

void Servo_Init(void) {
    HAL_TIM_PWM_Start(&CFG_SERVO_TIM, SERVO_LIFT_CHANNEL);
    HAL_TIM_PWM_Start(&CFG_SERVO_TIM, SERVO_CLAW_CHANNEL);
    Servo_LiftSet(SERVO_LIFT_UP);
    Servo_ClawSet(SERVO_CLAW_OPEN);
}

/* 两张表按枚举顺序排，越界读会取到相邻常量当脉宽，故一律先校验 */
void Servo_LiftSet(Servo_LiftPOS_e pos) {
    if ((unsigned)pos < (unsigned)(sizeof(s_lift_us) / sizeof(s_lift_us[0]))) {
        Servo_Write(SERVO_LIFT, s_lift_us[pos]);
    }
}

void Servo_ClawSet(Servo_ClawPOS_e pos) {
    if ((unsigned)pos < (unsigned)(sizeof(s_claw_us) / sizeof(s_claw_us[0]))) {
        Servo_Write(SERVO_CLAW, s_claw_us[pos]);
    }
}