/**
 * @file    bsp_gpio.c
 * @brief   限位/原点开关：电平查询 + EXTI 事件锁存
 * @note    开关常开 + 上拉，压下 = 低电平，EXTI 配 FALLING（每圈唯一确定的边沿）
 */
#include "bsp_gpio.h"
#include "sort_config.h"

/* ★ 消抖 + 按压闩锁（2026-09-28 自台架 dbg_gpio 移植，实测依据；2026-10-01 起转盘
   索引沿也用同款两道门，对齐台架验证过的实现）：
   单次按压产生 17~128 个沿、松开瞬间也能蹦 61 沿（弱上拉天线效应+触点弹跳），
   裸沿锁存会把"松开"判成"又撞了"→ 误触发急停 + 作废记账；转盘侧则是弹跳沿灌进
   整圈核对，把正常定位误判成丢步。
   两道门：①CFG_LIMIT_DEBOUNCE_MS 内只认一个沿 ②必须先由 Bsp_Gpio_Poll 确认
   "连续 40ms 松开"才武装下一次按压判定。真索引沿 ≥ 一圈(9.4s)一次，两道门不漏真沿 */
typedef struct {
    volatile uint8_t  armed;       /* 1 = 已确认松开，下一个被认可的下降沿才算数 */
    volatile uint32_t last_accept; /* 上一次被认可的沿时刻 */
    volatile uint32_t idle_since;  /* 电平连续为"松开"的起始时刻（0 = 此刻压着） */
} Bsp_LimDbc_t;

static volatile uint32_t s_events;
static volatile uint32_t s_tray_index_cnt;
/* ★ 转盘索引沿"待确认"时刻（≠0 = 收到一个沿、正在等电平确认压住才计数） */
static volatile uint32_t s_idx_pending_tick;
static Bsp_LimDbc_t s_lim_x;
static Bsp_LimDbc_t s_lim_y;
static Bsp_LimDbc_t s_tray_idx;

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

    for (uint8_t i = 0u; i < 3u; i++) {
        Bsp_LimDbc_t *st = (i == 0u) ? &s_lim_x : ((i == 1u) ? &s_lim_y : &s_tray_idx);
        uint8_t active =
            (i == 0u) ? Bsp_Gpio_LimitX() : ((i == 1u) ? Bsp_Gpio_LimitY() : Bsp_Gpio_TrayOrigin());

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

    /* ★ 转盘索引沿的电平确认（2026-10-08）：实测压住 ≥CFG_TRAY_INDEX_CONFIRM_MS 才计入
       按压计数；窗内电平始终松开 = 电气毛刺，丢弃（不然复核会误判成丢步、白跑一次回零） */
    if (s_idx_pending_tick != 0u) {
        if (Bsp_Gpio_TrayOrigin() != 0u) {
            if ((now - s_idx_pending_tick) >= CFG_TRAY_INDEX_CONFIRM_MS) {
                s_idx_pending_tick = 0u;
                s_tray_index_cnt++;
                s_events |= BSP_GPIO_EVT_TRAY_INDEX;
            }
        } else if ((now - s_idx_pending_tick) > CFG_TRAY_INDEX_CONFIRM_MS) {
            s_idx_pending_tick = 0u; /* 毛刺：电平已松开，不算一次按压 */
        }
    }
}

/* X/Y 限位合成同一事件：CoreXY 两轴必须一起停才不会歪。
   未武装（没确认过松开/仍在压着/40ms 内的重复沿）一律不锁存——真按压一定是
   "武装状态下的一次下降沿"，弹跳和松开沿全被两道门拦下。
   ★ 2026-10-08：撞限位的**判据已移到 App**（Motion_ServeEvents 改看"电平连续压住"，
   沿闩锁会把电流尖峰耦合出的假沿当成真撞，见 sort_config.h 该常量一节）；本事件位
   从此只承担"按压发生过"这条诊断语义，**不再是急停/作废记账的依据**。两道门与计数
   原样保留：它们定义的正是"按压"这个语义，且与台架 dbg_gpio（`lim` 命令）同构，
   台架与产品能逐条对表 */
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
        /* 弹跳灌进第二次沿，App 的整圈核对就永远对不上，每次定位都会被误判成丢步
           ——与 X/Y 同款两道门（台架 dbg_gpio 同构，见文件头实测依据）。
           ★ 2026-10-08 再加一道"电平确认"（计数交给 Bsp_Gpio_Poll）：一次电气毛刺同样能
           造出这里的一次下降沿，灌进整圈核对 ⇒ 复核不过 ⇒ 转盘白跑一次回零（现场现象）。
           真凸块压住开关几百 ms，毛刺 µs 级 ⇒ 只把"电平持续压住"的沿计入 s_tray_index_cnt */
        Bsp_LimDbc_t *st  = &s_tray_idx;
        uint32_t      now = HAL_GetTick();

        if (st->armed == 0u) {
            return;
        }
        if ((now - st->last_accept) < CFG_LIMIT_DEBOUNCE_MS) {
            return;
        }
        st->last_accept        = now;
        st->armed              = 0u;
        s_idx_pending_tick     = now; /* 待确认（确认在 Poll 里做，见 CFG_TRAY_INDEX_CONFIRM_MS） */
    }
}