/**
 * @file    bsp_gpio.c
 * @brief   限位/原点开关：电平查询 + EXTI 事件锁存
 * @note    开关常开 + 上拉，压下 = 低电平，EXTI 配 FALLING（每圈唯一确定的边沿）
 */
#include "bsp_gpio.h"
#include "sort_config.h"

/* 一整圈最少数秒，20ms 内的重复上升沿只能是触点弹跳 */
#define TRAY_INDEX_DEBOUNCE_MS 20u

/* ★ X/Y 限位消抖 + 按压闩锁（2026-09-28 自台架 dbg_gpio 移植，实测依据）：
   单次按压产生 17~128 个沿、松开瞬间也能蹦 61 沿（弱上拉天线效应+触点弹跳），
   裸沿锁存会把"松开"判成"又撞了"→ 误触发急停 + 作废记账。
   两道门：①CFG_LIMIT_DEBOUNCE_MS 内只认一个沿 ②必须先由 Bsp_Gpio_Poll 确认
   "连续 40ms 松开"才武装下一次按压判定（转盘索引沿已有 20ms 消抖，不在此列） */
typedef struct {
    volatile uint8_t  armed;       /* 1 = 已确认松开，下一个被认可的下降沿才算撞限位 */
    volatile uint32_t last_accept; /* 上一次被认可的沿时刻 */
    volatile uint32_t idle_since;  /* 电平连续为"松开"的起始时刻（0 = 此刻压着） */
} Bsp_LimDbc_t;

static volatile uint32_t s_events;
static volatile uint32_t s_tray_index_cnt;
static volatile uint32_t s_tray_index_tick;
static Bsp_LimDbc_t s_lim_x;
static Bsp_LimDbc_t s_lim_y;

uint8_t Bsp_Gpio_LimitX(void) {
    uint8_t active = (HAL_GPIO_ReadPin(CFG_LIMIT_GPIO_X, CFG_LIMIT_PIN_X) == CFG_LIMIT_ACTIVE_LEVEL);

    return active ? 1u : 0u;
}

uint8_t Bsp_Gpio_LimitY(void) {
    uint8_t active = (HAL_GPIO_ReadPin(CFG_LIMIT_GPIO_Y, CFG_LIMIT_PIN_Y) == CFG_LIMIT_ACTIVE_LEVEL);

    return active ? 1u : 0u;
}

uint8_t Bsp_Gpio_TrayOrigin(void) {
    uint8_t active =
        (HAL_GPIO_ReadPin(CFG_ORIGIN_GPIO_TRAY, CFG_ORIGIN_PIN_TRAY) == CFG_LIMIT_ACTIVE_LEVEL);

    return active ? 1u : 0u;
}

uint32_t Bsp_Gpio_TakeEvents(void) {
    uint32_t events;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    events   = s_events;
    s_events = 0u;
    __set_PRIMASK(primask);

    return events;
}

uint32_t Bsp_Gpio_TrayIndexCount(void) {
    return s_tray_index_cnt;
}

/* 松开确认：电平连续 CFG_LIMIT_DEBOUNCE_MS 为"松开"才重新武装（安全守护任务每圈调用）。
   压着期间 armed 保持 0—— homing 的 WaitLevel 走电平查询不受影响，本闩锁只管"事件" */
void Bsp_Gpio_Poll(void) {
    uint32_t now = HAL_GetTick();

    for (uint8_t i = 0u; i < 2u; i++) {
        Bsp_LimDbc_t *st   = (i == 0u) ? &s_lim_x : &s_lim_y;
        uint8_t       active = (i == 0u) ? Bsp_Gpio_LimitX() : Bsp_Gpio_LimitY();

        if (active != 0u) {
            st->idle_since = 0u;
            st->armed      = 0u;
            continue;
        }
        if (st->idle_since == 0u) {
            st->idle_since = now;
            continue;
        }
        if ((now - st->idle_since) >= CFG_LIMIT_DEBOUNCE_MS) {
            st->armed = 1u;
        }
    }
}

/* X/Y 限位合成同一事件：CoreXY 两轴必须一起停才不会歪。
   未武装（没确认过松开/仍在压着/40ms 内的重复沿）一律不锁存——真按压一定是
   "武装状态下的一次下降沿"，弹跳和松开沿全被两道门拦下 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == CFG_LIMIT_PIN_X || GPIO_Pin == CFG_LIMIT_PIN_Y) {
        Bsp_LimDbc_t *st  = (GPIO_Pin == CFG_LIMIT_PIN_X) ? &s_lim_x : &s_lim_y;
        uint32_t      now = HAL_GetTick();

        if (st->armed == 0u) {
            return;
        }
        if ((now - st->last_accept) < CFG_LIMIT_DEBOUNCE_MS) {
            return;
        }
        st->last_accept = now;
        st->armed       = 0u;
        s_events |= BSP_GPIO_EVT_XY_LIMIT;
    } else if (GPIO_Pin == CFG_ORIGIN_PIN_TRAY) {
        /* 弹跳灌进第二次沿，App 的整圈核对就永远对不上，每次定位都会被误判成丢步 */
        uint32_t now = HAL_GetTick();

        if ((now - s_tray_index_tick) < TRAY_INDEX_DEBOUNCE_MS) {
            return;
        }
        s_tray_index_tick = now;
        s_tray_index_cnt++;
        s_events |= BSP_GPIO_EVT_TRAY_INDEX;
    }
}