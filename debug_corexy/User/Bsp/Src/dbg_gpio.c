/**
 * @file    dbg_gpio.c
 * @brief   限位/原点微动开关：初始化 + EXTI 下降沿计数
 * @note    台架版做的是"接线验证"那部分（上台架步骤.md §5 的判据）：
 *          电平读数证明"开关压下/松开看得见"，沿计数证明"EXTI 事件真的进来了"——
 *          产品回零靠的是沿不是电平，所以后者才是回零能不能跑起来的前提。
 */
#include "dbg_gpio.h"
#include "dbg_config.h"
#include "main.h"
#include "stm32f4xx_hal.h"

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} Dbg_Gpio_Pin_t;

/* 引脚表来自 dbg_config.h（★换线只改那里，这里是纯消费方） */
static const Dbg_Gpio_Pin_t s_lim[DBG_LIMIT_COUNT] = {
    {DBG_CFG_LIMIT_X_PORT, DBG_CFG_LIMIT_X_PIN},
    {DBG_CFG_LIMIT_Y_PORT, DBG_CFG_LIMIT_Y_PIN},
    {DBG_CFG_LIMIT_TRAY_PORT, DBG_CFG_LIMIT_TRAY_PIN},
};

static const char *const s_name[DBG_LIMIT_COUNT] = {
    DBG_CFG_LIMIT_X_NAME,
    DBG_CFG_LIMIT_Y_NAME,
    DBG_CFG_LIMIT_TRAY_NAME,
};

static const char *const s_role[DBG_LIMIT_COUNT] = {
    "X限位",
    "Y限位",
    "转盘原点",
};

/* EXTI 里写、命令里读：volatile。计数溢出无所谓（uint32 数不完） */
static volatile uint32_t s_edges[DBG_LIMIT_COUNT];
static volatile uint32_t s_last_tick[DBG_LIMIT_COUNT];

/* ★ 状态闩锁（2026-09-26 用户实测纠偏）：一次"按下-松开"循环里，机械弹跳会让下降沿
   出现在按压端**和松开端**（慢松手的弹跳能超过 40ms 门限），只按沿计数就成了
   "两倍次数"。规则改为：只有"确认松开"之后的下一个下降沿才计——一个按压循环恒 1 次，
   方向只认"压下" */
static volatile uint8_t s_confirmed[DBG_LIMIT_COUNT];  /* 1=已确认压下（等松开确认才解锁） */
static uint32_t         s_open_since[DBG_LIMIT_COUNT]; /* 电平连续为高的起点（主循环维护） */

/* ★ 裸下降沿（无消抖无闩锁）：回零"触发即停"专用。
   触发要快——40ms 消抖在 10mm/s 下就是 0.4mm 过冲；真伪由停完之后的电平复核把关 */
static volatile uint8_t  s_raw_flag[DBG_LIMIT_COUNT];
static volatile uint32_t s_raw_tick[DBG_LIMIT_COUNT];

void Dbg_Gpio_Init(void) {
    GPIO_InitTypeDef init = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* 与产品 gpio.c 同配置：EXTI 下降沿 + 内部上拉。
       改成上升沿会让"压下=触发"变"松开=触发"，回零一类的判据全部跟着错 */
    init.Mode  = GPIO_MODE_IT_FALLING;
    init.Pull  = GPIO_PULLUP;
    init.Speed = GPIO_SPEED_FREQ_LOW;

    init.Pin = DBG_CFG_LIMIT_TRAY_PIN | DBG_CFG_LIMIT_X_PIN; /* PB0 / PB1 同在 GPIOB */
    (void)HAL_GPIO_Init(GPIOB, &init);

    init.Pin = DBG_CFG_LIMIT_Y_PIN; /* PC5 在 GPIOC */
    (void)HAL_GPIO_Init(DBG_CFG_LIMIT_Y_PORT, &init);

    /* 抢占 0 高于串口（5）：沿不能丢，与产品工程同优先级。
       中断服务函数在 stm32f4xx_it.c，落到下面的 HAL_GPIO_EXTI_Callback */
    HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

const char *Dbg_Gpio_NameOf(Dbg_Limit_e which) {
    return (which < DBG_LIMIT_COUNT) ? s_name[which] : "?";
}

const char *Dbg_Gpio_RoleOf(Dbg_Limit_e which) {
    return (which < DBG_LIMIT_COUNT) ? s_role[which] : "?";
}

uint8_t Dbg_Gpio_Level(Dbg_Limit_e which) {
    if (which >= DBG_LIMIT_COUNT) {
        return 0u;
    }
    /* 未触发=1（上拉）、压下=0：正好是 GPIO 读数本身，不用再取反 */
    return (HAL_GPIO_ReadPin(s_lim[which].port, s_lim[which].pin) == GPIO_PIN_SET) ? 1u : 0u;
}

uint32_t Dbg_Gpio_EdgeCount(Dbg_Limit_e which) {
    return (which < DBG_LIMIT_COUNT) ? s_edges[which] : 0u;
}

uint32_t Dbg_Gpio_LastEdgeTick(Dbg_Limit_e which) {
    return (which < DBG_LIMIT_COUNT) ? s_last_tick[which] : 0u;
}

void Dbg_Gpio_ClearEdges(void) {
    for (uint8_t i = 0u; i < (uint8_t)DBG_LIMIT_COUNT; i++) {
        s_edges[i]     = 0u;
        s_last_tick[i] = 0u;
        s_confirmed[i] = 0u;
        s_open_since[i] = 0u;
    }
}

/* 所有 EXTI 线共用这一个回调，靠引脚号区分是哪一路。
   计数 = 按压次数（下降沿触发 + 消抖门限 + 状态闩锁三重过滤）：
   · 消抖：门限内的沿是同一次触发自己的弹跳
   · 闩锁：确认松开之前，松开端的弹跳/噪声一律不计（否则松开也 +1，变成两倍）
   · 电平读数不受影响（永远是最原始的引脚状态） */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    uint32_t now = HAL_GetTick();

    for (uint8_t i = 0u; i < (uint8_t)DBG_LIMIT_COUNT; i++) {
        if (s_lim[i].pin == GPIO_Pin) {
            s_raw_flag[i] = 1u; /* 裸沿最先记：回零触发即停等的就是它（含弹跳沿，真伪靠电平复核） */
            s_raw_tick[i] = now;

            if ((now - s_last_tick[i]) < (uint32_t)DBG_CFG_LIMIT_DEBOUNCE_MS) {
                return;
            }
            if (s_confirmed[i] != 0u) {
                return; /* 未确认松开：松开端的弹跳不计 */
            }
            s_edges[i]++;
            s_last_tick[i] = now;
            s_confirmed[i] = 1u;
            return;
        }
    }
}

uint32_t Dbg_Gpio_TakeRawEdge(Dbg_Limit_e which) {
    if (which >= DBG_LIMIT_COUNT) {
        return 0u;
    }
    if (s_raw_flag[which] != 0u) {
        s_raw_flag[which] = 0u;
        return s_raw_tick[which]; /* 沿的时刻（HAL_GetTick 口径）；0=没有新沿 */
    }
    return 0u;
}

void Dbg_Gpio_ClearRawEdges(void) {
    for (uint8_t i = 0u; i < (uint8_t)DBG_LIMIT_COUNT; i++) {
        s_raw_flag[i] = 0u;
    }
}

/* 主循环每圈调用（Dbg_App_Tick）：维护"松开确认"。电平连续为高超过消抖门限
   才解锁计数，期间松开端的弹跳沿全部被闩锁挡住 */
void Dbg_Gpio_Poll(void) {
    uint32_t now = HAL_GetTick();

    for (uint8_t i = 0u; i < (uint8_t)DBG_LIMIT_COUNT; i++) {
        if (Dbg_Gpio_Level((Dbg_Limit_e)i) != 0u) {
            if (s_confirmed[i] != 0u) {
                if (s_open_since[i] == 0u) {
                    s_open_since[i] = now;
                } else if ((now - s_open_since[i]) >= (uint32_t)DBG_CFG_LIMIT_DEBOUNCE_MS) {
                    s_confirmed[i]  = 0u;
                    s_open_since[i] = 0u;
                }
            }
        } else {
            s_open_since[i] = 0u;
        }
    }
}
