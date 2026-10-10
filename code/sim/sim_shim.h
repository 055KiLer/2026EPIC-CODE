/**
 ******************************************************************************
 * @file    sim_shim.h
 * @brief   主机侧仿真台的内核/硬件抽象替身（仅供 code/sim 使用，不参与上机编译）
 * @attention
 * 这里提供的是与 stm32f4xx_hal.h + FreeRTOS + CMSIS-RTOS2 完全同名的接口，
 * 目的是让 User/ 四层的产品源码 **一行不改** 就能在 PC 上跑起来。
 * 虚拟时钟只在 vTaskDelay / 阻塞发送 / 显式推进时前进，物理模型随时间步进。
 ******************************************************************************
 */
#ifndef __SIM_SHIM_H
#define __SIM_SHIM_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

/* ==================== 虚拟时钟 / 中断屏蔽 ==================== */
uint32_t sim_now_ms(void);
void     sim_advance_ms(uint32_t dt_ms);   /* 推进时钟并步进物理模型 */
int      sim_irq_enabled(void);

/* ==================== 故障注入 / 场景控制 ==================== */
typedef struct
{
    float item_x_mm;      /* 当前应抓目标的机械坐标（K230 已标定后的值） */
    float item_y_mm;
    uint8_t category;     /* 货物类别 ID 1~6（帧的原颜色字节）：决定进哪个盒 */
    uint8_t shape;        /* 货物形状 1~3（帧的原形状字节）：不参与分类，只选夹持方案 */
    uint8_t has_qr;       /* K230 帧 byte[5]：产品侧保持解析但不使用（模拟污渍） */
    float grab_radius_mm; /* 夹爪可夹住的半径 */
} sim_item_t;

#define SIM_MAX_ITEMS 16

typedef struct
{
    /* --- 执行器故障 --- */
    uint8_t motor_block[3];        /* 1 = 该电机轴堵转（指令在发，轴不动） */
    uint8_t tray_slip_pct;         /* 转盘输出轴打滑百分比（0=正常） */
    uint8_t serial_dead[6];        /* 1 = 该串口发送失败（对端断线） */
    uint8_t zdt_busy_rejects_fd;   /* 1 = 驱动器运动中来新位置帧不接管（语义未标定） */
    uint8_t tray_cam_count;        /* 转盘每转的原点凸块数（默认 1，>1 检验索引复核前提） */
    /* --- 传感器故障 --- */
    uint8_t limit_x_stuck;         /* 1 = X 限位常闭（永远读作触发） */
    uint8_t limit_x_dead;          /* 1 = X 限位断线（永远读作未触发，且不出 EXTI） */
    uint8_t limit_y_stuck;
    uint8_t limit_y_dead;
    uint8_t tray_sw_stuck;
    uint8_t tray_sw_dead;
    /* --- 投放点激光检测（2026-10-09 新硬件；2026-10-10 起**退出计数闸门**：爪挡光路） ---
       产品 CFG_LASER_ENABLE=0 时这两项不参与判定；台架 `laser drop`/`laser verify` 是它的
       验收工具（见 dbg_config.h §3.6），仿真台不再为它建场景 */
    uint8_t laser_dead;            /* 1 = 永远读不到"检测到"（断线/光路没对准/极性反） */
    uint8_t laser_stuck;           /* 1 = 永远读作"检测到"（卡住/短接）——夹空会被漏放 */
    /* --- 视觉 --- */
    uint8_t k230_link_down;        /* 1 = K230 不再发帧 */
    uint8_t k230_out_of_envelope;  /* 1 = K230 报告越界坐标 */
    uint8_t k230_count_zero;       /* ★ 1 = 数量位恒填 0（= 上位机还没实现下标 6）：
                                      "产品已上数量复核、K230 还没跟"的冻结症状，见 s25 */
    /* --- 几何 --- */
    float   limit_trigger_x_mm;    /* 限位实际触发时的 X（可正可负 = 安装偏差） */
    float   limit_trigger_y_mm;
    float   hard_stop_x_mm;        /* 机械硬限位（行程负向端） */
    float   hard_stop_y_mm;
    /* --- 行为开关 --- */
    uint8_t verbose;
} sim_cfg_t;

extern sim_cfg_t g_sim;

/* ==================== 运行结果观测 ==================== */
typedef struct
{
    /* 机械越界/碰撞 */
    int   hard_stop_hits;          /* 龙门顶到机械硬限位的次数 */
    float min_x_mm, min_y_mm;      /* 实测最小坐标 */
    float max_x_mm, max_y_mm;
    int   oob_command;             /* 被包络校验挡下的越界目标点数 */
    /* 运动记账 */
    int   move_rejected_untrusted; /* 因记账不可信被拒的绝对定位次数 */
    int   homing_failures;
    int   tray_rehomes;
    int   xy_untrust_events;       /* 运行中 XY 记账由可信变不可信的次数（白盒采样） */
    int   tray_untrust_events;     /* 运行中转盘记账被索引复核判废的次数 */
    /* 货物 */
    int   items_done;              /* 正确入盒件数 */
    int   items_empty_grab;        /* 抓空次数 */
    int   items_dropped_outside;   /* 掉在装置外/落点不合法 */
    int   bin_of[SIM_MAX_ITEMS];   /* 每件的落点盒号，-1 = 未入盒 */
    /* 显示 */
    int   screen_field_errors;
    int   screen_updates;
    uint8_t last_display_total;
    uint8_t last_display_seq;
    /* 时间 */
    uint32_t sim_ms;
    uint32_t cycle_ms[SIM_MAX_ITEMS];
    /* 稳定性 */
    int   watchdog_fired;          /* 仿真总时长超限 = 活锁/死锁 */
} sim_result_t;

extern sim_result_t g_res;

/* ==================== 场景 API ==================== */
void sim_reset(void);
void sim_load_items(const sim_item_t *items, int n);
int  sim_item_count(void);
int  sim_items_remaining(void);      /* 仍在托盘里的件数 */
void sim_run_ms(uint32_t ms);        /* 让全部任务与物理模型推进 ms */
uint8_t sim_gripper_holding(void);   /* 夹爪里是否压着货（物理判定） */
float sim_gantry_x(void);
float sim_gantry_y(void);
float sim_tray_angle(void);
int   sim_current_bin(void);         /* 物理上正对投放点的盒号 0~5 */
int   sim_motor_state(int idx, float *pos_mm); /* 0=A 1=B 2=tray */
void  sim_dump_screen(void);

/* ==================== HAL 替身 ==================== */
typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;

typedef struct { int dummy; } GPIO_TypeDef;
extern GPIO_TypeDef sim_GPIOA, sim_GPIOB, sim_GPIOC, sim_GPIOD;
#define GPIOA (&sim_GPIOA)
#define GPIOB (&sim_GPIOB)
#define GPIOC (&sim_GPIOC)
#define GPIOD (&sim_GPIOD)

/* 与 HAL 逐字一致（含 (uint16_t) 强制转换）：仿真要在"引脚号怎么比较"这一点上与真机
   同构。所以引脚核对必须写成编译期断言（负长度数组），**不要写 #if** —— 写 #if 在真机
   是硬错误、在这里则会因为"裸字面量"而静默通过，正是"仿真过、真机炸"的来路。
   见 Config/sort_config.h 末尾与 sim/sim_plant.c 顶部 */
#define GPIO_PIN_0  ((uint16_t)0x0001)
#define GPIO_PIN_1  ((uint16_t)0x0002)
#define GPIO_PIN_2  ((uint16_t)0x0004)
#define GPIO_PIN_3  ((uint16_t)0x0008)
#define GPIO_PIN_4  ((uint16_t)0x0010)
#define GPIO_PIN_5  ((uint16_t)0x0020)
#define GPIO_PIN_6  ((uint16_t)0x0040)
#define GPIO_PIN_7  ((uint16_t)0x0080)
#define GPIO_PIN_8  ((uint16_t)0x0100)
#define GPIO_PIN_9  ((uint16_t)0x0200)
#define GPIO_PIN_10 ((uint16_t)0x0400)
#define GPIO_PIN_11 ((uint16_t)0x0800)
#define GPIO_PIN_12 ((uint16_t)0x1000)
/* 13~15：2026-10-09 补（激光检测接的是 PD14）——值与 HAL 的 GPIO_PIN_n 定义逐字一致 */
#define GPIO_PIN_13 ((uint16_t)0x2000)
#define GPIO_PIN_14 ((uint16_t)0x4000)
#define GPIO_PIN_15 ((uint16_t)0x8000)

uint32_t HAL_GetTick(void);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);

/* 串口外设寄存器：故障路径（Bsp_Uart_SendDirect）会直接轮询 TXE/TC 写 DR ——
   那条路不能用 HAL（tick 在故障里可能已停），所以替身必须照真机的寄存器名提供字段。
   CMSIS 里 UART_TypeDef 与 USART_TypeDef 是两个名字、同一布局，这里照抄这个事实。
   SR 初值给成 TXE|TC，这样万一在仿真里被调到也是"立刻发完"，不会空转 */
typedef struct
{
    int instance;
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t CR1;
    volatile uint32_t CR3;
} UART_TypeDef;

typedef UART_TypeDef USART_TypeDef;

extern UART_TypeDef sim_USART1, sim_USART2, sim_USART3, sim_UART4, sim_UART5, sim_USART6;

#define USART_SR_TXE   ((uint32_t)0x0080)
#define USART_SR_TC    ((uint32_t)0x0040)
#define USART_CR1_UE   ((uint32_t)0x2000)
#define USART_CR1_RXNEIE ((uint32_t)0x0020)
#define USART_CR3_EIE  ((uint32_t)0x0001)
typedef struct
{
    UART_TypeDef *Instance;
    uint8_t *rx_buf;
    uint16_t rx_len;
    uint32_t baud;
} UART_HandleTypeDef;

#define USART1_INSTANCE 1
#define USART2_INSTANCE 2
#define USART3_INSTANCE 3
#define UART4_INSTANCE  4
#define UART5_INSTANCE  5
#define USART6_INSTANCE 6

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, uint8_t *data, uint16_t len, uint32_t timeout);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *h, uint8_t *buf, uint16_t len);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *h);

/* 由产品代码实现、仿真台反向调用 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *h);
void HAL_GPIO_EXTI_Callback(uint16_t pin);

typedef struct { int instance; } TIM_TypeDef;
extern TIM_TypeDef sim_TIM4;
typedef struct
{
    TIM_TypeDef *Instance;
    uint32_t ccr[4];
} TIM_HandleTypeDef;

#define TIM_CHANNEL_1 0u
#define TIM_CHANNEL_2 1u
#define TIM_CHANNEL_3 2u
#define TIM_CHANNEL_4 3u
#define __HAL_TIM_SET_COMPARE(h, ch, val)  ((h)->ccr[ch] = (uint32_t)(val))
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *h, uint32_t ch);

void Error_Handler(void);

/* 临界区 / 中断屏蔽 */
extern int sim_primask;
static inline uint32_t __get_PRIMASK(void) { return (uint32_t)sim_primask; }
static inline void __disable_irq(void) { sim_primask = 1; }
static inline void __enable_irq(void) { sim_primask = 0; }
static inline void __set_PRIMASK(uint32_t v) { sim_primask = (int)v; }

/* ==================== FreeRTOS / CMSIS-RTOS2 替身 ==================== */
typedef uint32_t TickType_t;
#define portMAX_DELAY ((TickType_t)0xFFFFFFFFu)
#define pdMS_TO_TICKS(x) ((TickType_t)(x))
#define pdTRUE  1
#define pdFALSE 0

void vTaskDelay(TickType_t ticks);
void sim_task_create(void (*fn)(void *), void *arg, const char *name, uint32_t stack_bytes, int prio);
void sim_start(void);

#define taskENTER_CRITICAL()  sim_enter_critical()
#define taskEXIT_CRITICAL()   sim_exit_critical()
void sim_enter_critical(void);
void sim_exit_critical(void);

typedef void *osMessageQueueId_t;
typedef enum {
    osOK = 0, osErrorTimeout = 2, osErrorResource = 4
} osStatus_t;
#define osWaitForever ((uint32_t)0xFFFFFFFFu)

osMessageQueueId_t osMessageQueueNew(uint32_t msg_count, uint32_t msg_size, const void *attr);
osStatus_t osMessageQueuePut(osMessageQueueId_t mq, const void *msg, uint32_t prio, uint32_t timeout);
osStatus_t osMessageQueueGet(osMessageQueueId_t mq, void *msg, void *msg_prio, uint32_t timeout);

#endif
