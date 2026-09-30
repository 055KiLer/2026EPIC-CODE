/**
 * @file    servo_out.c
 * @brief   舵机档位与原始脉宽输出
 */
#include "servo_out.h"
#include "dbg_config.h"
#include "tim.h"

typedef struct {
    const char *name;
    uint32_t    channel;
    uint16_t    pos_us[3]; /* 档位脉宽；用不到的档位与上一个相同 */
    uint8_t     pos_count;
    const char *pos_name[3];
    uint16_t    current_us;
} Servo_Out_t;

/* 档位脉宽来自 dbg_config.h（2026-09-26 台架实测定稿）；抓取时序见该文件 §3 注释 */
static Servo_Out_t s_servo[DBG_SERVO_COUNT] = {
    {"CLAW",
     DBG_CFG_CLAW_TIM_CH,
     {DBG_CFG_CLAW_OPEN_US, DBG_CFG_CLAW_CLOSE_US, DBG_CFG_CLAW_HALF_US},
     3u,
     {"OPEN", "CLOSE", "PREGRIP"},
     0u},
    {"LIFT",
     DBG_CFG_LIFT_TIM_CH,
     {DBG_CFG_LIFT_UP_US, DBG_CFG_LIFT_DOWN_US, DBG_CFG_LIFT_DOWN_US},
     2u,
     {"UP", "DOWN", "-"},
     0u},
};

const char *Servo_Out_NameOf(Dbg_Servo_e servo) {
    return (servo < DBG_SERVO_COUNT) ? s_servo[servo].name : "?";
}

/* 该通道的 PWM 是否在跑（sstop 之后为 0） */
static uint8_t s_running[DBG_SERVO_COUNT];

uint8_t Servo_Out_PosCount(Dbg_Servo_e servo) {
    return (servo < DBG_SERVO_COUNT) ? s_servo[servo].pos_count : 0u;
}

const char *Servo_Out_PosNameOf(Dbg_Servo_e servo, uint8_t pos) {
    if (servo >= DBG_SERVO_COUNT || pos >= s_servo[servo].pos_count) {
        return "?";
    }
    return s_servo[servo].pos_name[pos];
}

uint16_t Servo_Out_PosUs(Dbg_Servo_e servo, uint8_t pos) {
    if (servo >= DBG_SERVO_COUNT || pos >= s_servo[servo].pos_count) {
        return 0u;
    }
    return s_servo[servo].pos_us[pos];
}

uint16_t Servo_Out_CurrentUs(Dbg_Servo_e servo) {
    return (servo < DBG_SERVO_COUNT) ? s_servo[servo].current_us : 0u;
}

static uint16_t Servo_Out_Clamp(uint16_t pulse_us) {
    if (pulse_us < DBG_CFG_SERVO_MIN_US) {
        return DBG_CFG_SERVO_MIN_US;
    }
    if (pulse_us > DBG_CFG_SERVO_MAX_US) {
        return DBG_CFG_SERVO_MAX_US;
    }
    return pulse_us;
}

uint16_t Servo_Out_SetUs(Dbg_Servo_e servo, uint16_t pulse_us) {
    uint16_t applied;

    if (servo >= DBG_SERVO_COUNT) {
        return 0u;
    }

    /* 被 sstop 停过 PWM 的通道，任何一次档位/脉宽命令都自动把它重新起起来，
       否则会出现"命令打出去了但舵机没反应"这种假故障 */
    if (s_running[servo] == 0u) {
        (void)HAL_TIM_PWM_Start(&htim4, s_servo[servo].channel);
        s_running[servo] = 1u;
    }

    applied = Servo_Out_Clamp(pulse_us);
    __HAL_TIM_SET_COMPARE(&htim4, s_servo[servo].channel, (uint32_t)applied);
    s_servo[servo].current_us = applied;

    return applied;
}

void Servo_Out_Init(void) {
    /* 上电安全位：夹爪 = 完全张开（DBG_CFG_CLAW_BOOT_US，巡航安全形——标定前临时用过
       1500 中位防堵转，2026-09-26 行程定稿后改为巡航位）；升降先抬到最高。
       （SetUs/SetPos 自己会把停过的 PWM 重新起起来，不用先手动 Start） */
    (void)Servo_Out_SetUs(DBG_SERVO_CLAW, DBG_CFG_CLAW_BOOT_US);
    (void)Servo_Out_SetPos(DBG_SERVO_LIFT, SERVO_POS_0);
}

uint8_t Servo_Out_SetPos(Dbg_Servo_e servo, uint8_t pos) {
    if (servo >= DBG_SERVO_COUNT || pos >= s_servo[servo].pos_count) {
        return 0u;
    }

    (void)Servo_Out_SetUs(servo, s_servo[servo].pos_us[pos]);
    return 1u;
}

void Servo_Out_Stop(Dbg_Servo_e servo) {
    if (servo < DBG_SERVO_COUNT) {
        (void)HAL_TIM_PWM_Stop(&htim4, s_servo[servo].channel);
        s_running[servo] = 0u;
    }
}