/**
 ******************************************************************************
 * @file    sim_kernel.c
 * @brief   协作式（按优先级选任务）虚拟内核 + HAL/FreeRTOS/CMSIS-RTOS2 替身
 * @attention
 * 与真机的已知差异（解读结论时须知）：
 *  - 任务只在 vTaskDelay / 队列阻塞处让出，无 tick 级抢占。产品代码每个循环体
 *    内都含 >=1ms 延时，故急停时延上界仍可用 1~2ms 界定。
 *  - 物理模型 100us 定步长；HAL_GetTick 保持 1ms 粒度（与真机一致）。
 *  - x86-64 栈帧尺寸与 Cortex-M4 不同，因此不据此评判任务栈余量。
 ******************************************************************************
 */
#include "sim_shim.h"
#include "sim_kernel.h"
#include "sim_plant.h"
#include <ucontext.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SIM_MAX_TASK 8
#define SIM_STACK    (256 * 1024)
#define STEP_US      100u

enum { T_FREE = 0, T_READY, T_BLOCKED, T_DEAD };

typedef struct
{
    ucontext_t ctx;
    char *stack;
    int state;
    int prio;
    uint64_t wake_us;
    void (*fn)(void *);
    void *arg;
    char name[16];
    uint32_t switches;
} sim_task_t;

static sim_task_t s_task[SIM_MAX_TASK];
static int s_running = -1;
static ucontext_t s_boot_ctx;
static int s_booting = 1;

static uint64_t s_now_us = 0;
uint32_t sim_now_ms(void) { return (uint32_t)(s_now_us / 1000u); }

/* ---- 半截帧回归注入（场景 s19）：屏端静默丢失一次发送调用的字节 ---- */
static uint32_t s_drop_armed_ms = 0u;   /* 到这个时刻后开始计数；0 = 未布防 */
static uint32_t s_drop_nth = 0u;        /* 丢第 nth 次调用（0 起） */
static uint32_t s_drop_seen = 0u;
static uint32_t s_drop_count = 0u;
static int      s_drop_line = -1;       /* 丢弃发生时屏指令日志已有条数 */

void sim_screen_arm_drop(uint32_t after_ms, uint32_t nth)
{
    s_drop_armed_ms = after_ms;
    s_drop_nth = nth;
    s_drop_seen = 0u;
}

uint32_t sim_screen_drops(void) { return s_drop_count; }
int sim_screen_drop_line(void) { return s_drop_line; }

int sim_primask = 0;
static int s_crit = 0;
int sim_irq_enabled(void) { return sim_primask ? 0 : 1; }

sim_cfg_t g_sim;
sim_result_t g_res;

#define SIM_MAXPEND 64
static uint16_t s_pend_exti[SIM_MAXPEND];
static int s_pend_exti_n;
static int s_pend_uart[6];

/* ==================== 中断投递 ==================== */
static void deliver_isrs(void)
{
    if (s_crit == 0 && !sim_primask) {
        for (int i = 0; i < 6; i++) {
            /* 每次只投 1 字节：保持"按波特率逐字节到达"的时间模型 */
            if (s_pend_uart[i] && sim_plant_uart_ready_rx(i)) {
                sim_plant_uart_deliver_rx(i);
                if (!sim_plant_uart_ready_rx(i)) {
                    s_pend_uart[i] = 0;
                }
            }
        }
    }
    if (!sim_primask) {
        int guard = 0;

        while (s_pend_exti_n > 0 && guard++ < SIM_MAXPEND) {
            uint16_t pin = s_pend_exti[0];

            for (int i = 0; i + 1 < s_pend_exti_n; i++) {
                s_pend_exti[i] = s_pend_exti[i + 1];
            }
            s_pend_exti_n--;
            HAL_GPIO_EXTI_Callback(pin);
        }
    }
}

void sim_raise_exti(uint16_t pin)
{
    if (s_pend_exti_n < SIM_MAXPEND) {
        s_pend_exti[s_pend_exti_n++] = pin;
    }
}

void sim_uart_byte_arrives(int uart_id)
{
    s_pend_uart[uart_id] = 1;
}

/* ==================== 时间推进 ==================== */
static void step_us(uint32_t dt)
{
    s_now_us += dt;
    sim_plant_step_us(dt);
    deliver_isrs();
}

void sim_advance_ms(uint32_t dt_ms)
{
    uint64_t target = s_now_us + (uint64_t)dt_ms * 1000u;

    while (s_now_us < target) {
        step_us(STEP_US);
    }
}

/* ==================== 让出与调度 ==================== */
static void yield_to_hub(void)
{
    int me = s_running;

    swapcontext(&s_task[me].ctx, &s_boot_ctx);
    s_running = me; /* 恢复现场时把"我是谁"写回来 */
}

static int pick_task(void)
{
    int best = -1;
    int bp = -1;

    for (int i = 0; i < SIM_MAX_TASK; i++) {
        if (s_task[i].state == T_READY && s_task[i].prio > bp) {
            bp = s_task[i].prio;
            best = i;
        }
    }
    return best;
}

static void mark_due(void)
{
    for (int i = 0; i < SIM_MAX_TASK; i++) {
        if (s_task[i].state == T_BLOCKED && s_now_us >= s_task[i].wake_us) {
            s_task[i].state = T_READY;
        }
    }
}

void vTaskDelay(TickType_t ticks)
{
    int me = s_running;

    s_task[me].state = T_BLOCKED;
    s_task[me].wake_us = s_now_us + (uint64_t)ticks * 1000u;
    yield_to_hub();
}

void sim_enter_critical(void) { s_crit++; }

void sim_exit_critical(void)
{
    if (s_crit > 0) {
        s_crit--;
    }
    deliver_isrs();
}

uint32_t HAL_GetTick(void) { return sim_now_ms(); }

static void task_trampoline(void)
{
    sim_task_t *t = &s_task[s_running];

    t->fn(t->arg);
    t->state = T_DEAD;
}

void sim_task_create(void (*fn)(void *), void *arg, const char *name, uint32_t stack_bytes, int prio)
{
    (void)stack_bytes;
    for (int i = 0; i < SIM_MAX_TASK; i++) {
        if (s_task[i].state == T_FREE) {
            sim_task_t *t = &s_task[i];

            t->stack = (char *)malloc(SIM_STACK);
            t->state = T_READY;
            t->prio = prio;
            t->fn = fn;
            t->arg = arg;
            t->wake_us = 0;
            t->switches = 0;
            snprintf(t->name, sizeof(t->name), "%s", name);
            getcontext(&t->ctx);
            t->ctx.uc_stack.ss_sp = t->stack;
            t->ctx.uc_stack.ss_size = SIM_STACK;
            t->ctx.uc_link = &s_boot_ctx;
            makecontext(&t->ctx, (void (*)(void))task_trampoline, 0);
            return;
        }
    }
    printf("sim: task table overflow\n");
    exit(2);
}

/* 主调度循环：把系统推进到 ms_budget 之后（或全部任务结束） */
void sim_run_ms(uint32_t ms_budget);
static int s_all_dead(void)
{
    for (int i = 0; i < SIM_MAX_TASK; i++) {
        if (s_task[i].state == T_READY || s_task[i].state == T_BLOCKED) {
            return 0;
        }
    }
    return 1;
}

void sim_run_until(uint64_t until_us)
{
    if (s_booting) {
        getcontext(&s_boot_ctx);
        s_booting = 0;
    }
    while (s_now_us < until_us) {
        int next;

        mark_due();
        next = pick_task();
        if (next >= 0) {
            s_running = next;
            s_task[next].switches++;
            swapcontext(&s_boot_ctx, &s_task[next].ctx);
            s_running = -1;
            if (s_all_dead()) {
                return;
            }
            continue;
        }
        /* 没有可运行任务：只推进时间（物理 + 中断照常） */
        step_us(STEP_US);
    }
    s_running = -1;
}

uint64_t sim_now_us_get(void) { return s_now_us; }
uint32_t sim_task_switches(int i) { return (i < SIM_MAX_TASK) ? s_task[i].switches : 0; }
const char *sim_task_name(int i) { return (i < SIM_MAX_TASK) ? s_task[i].name : "?"; }
int sim_task_state(int i) { return (i < SIM_MAX_TASK) ? s_task[i].state : -1; }

/* ==================== 消息队列 ==================== */
typedef struct
{
    uint8_t *mem;
    uint32_t size, count, head, tail, n;
} sim_mq_t;

osMessageQueueId_t osMessageQueueNew(uint32_t msg_count, uint32_t msg_size, const void *attr)
{
    sim_mq_t *q;

    (void)attr;
    q = (sim_mq_t *)calloc(1, sizeof(*q));
    q->n = msg_count;
    q->size = msg_size;
    q->mem = (uint8_t *)calloc(msg_count, msg_size);
    return q;
}

osStatus_t osMessageQueuePut(osMessageQueueId_t id, const void *msg, uint32_t prio, uint32_t timeout)
{
    sim_mq_t *q = (sim_mq_t *)id;

    (void)prio;
    (void)timeout;
    if (q->count == q->n) {
        return osErrorResource;
    }
    memcpy(q->mem + (size_t)q->tail * q->size, msg, q->size);
    q->tail = (q->tail + 1u) % q->n;
    q->count++;
    sim_mq_wake_all(); /* 唤醒因 osWaitForever 阻塞的接收任务 */
    return osOK;
}

osStatus_t osMessageQueueGet(osMessageQueueId_t id, void *msg, void *msg_prio, uint32_t timeout)
{
    sim_mq_t *q = (sim_mq_t *)id;

    (void)msg_prio;
    while (q->count == 0u) {
        if (timeout == 0u) {
            return osErrorTimeout;
        }
        s_task[s_running].state = T_BLOCKED;
        s_task[s_running].wake_us = (uint64_t)1 << 50; /* 等 Put 唤醒 */
        yield_to_hub();
    }
    memcpy(msg, q->mem + (size_t)q->head * q->size, q->size);
    q->head = (q->head + 1u) % q->n;
    q->count--;
    return osOK;
}

void sim_mq_wake_all(void)
{
    for (int i = 0; i < SIM_MAX_TASK; i++) {
        if (s_task[i].state == T_BLOCKED && s_task[i].wake_us > ((uint64_t)1 << 40)) {
            s_task[i].wake_us = s_now_us;
        }
    }
}

/* ==================== HAL 替身 ==================== */
GPIO_TypeDef sim_GPIOA, sim_GPIOB, sim_GPIOC, sim_GPIOD;
/* SR 初值给 TXE|TC、CR1 给 UE：故障路径（Bsp_Uart_SendDirect）直接轮询这两个位，
   给成"随时可发、已发完"，万一在仿真里被调到也不会空转 */
UART_TypeDef sim_USART1 = { 1, USART_SR_TXE | USART_SR_TC, 0, USART_CR1_UE, 0 };
UART_TypeDef sim_USART2 = { 2, USART_SR_TXE | USART_SR_TC, 0, USART_CR1_UE, 0 };
UART_TypeDef sim_USART3 = { 3, USART_SR_TXE | USART_SR_TC, 0, USART_CR1_UE, 0 };
UART_TypeDef sim_UART4  = { 4, USART_SR_TXE | USART_SR_TC, 0, USART_CR1_UE, 0 };
UART_TypeDef sim_UART5  = { 5, USART_SR_TXE | USART_SR_TC, 0, USART_CR1_UE, 0 };
UART_TypeDef sim_USART6 = { 6, USART_SR_TXE | USART_SR_TC, 0, USART_CR1_UE, 0 };
TIM_TypeDef sim_TIM4 = { 4 };
TIM_HandleTypeDef htim4 = { &sim_TIM4, { 0 } };
/* 波特率按真机 usart.c 逐路对应（UART4 是备用口，沿用 9600）；
   句柄 <-> 语义编号的表在下面 uart_index()，与产品 bsp_uart.c 的 s_huart[] 同序 */
UART_HandleTypeDef huart1 = { &sim_USART1, NULL, 0, 115200 };
UART_HandleTypeDef huart2 = { &sim_USART2, NULL, 0, 115200 };
UART_HandleTypeDef huart3 = { &sim_USART3, NULL, 0, 115200 };
UART_HandleTypeDef huart4 = { &sim_UART4, NULL, 0, 9600 };
UART_HandleTypeDef huart5 = { &sim_UART5, NULL, 0, 115200 };
UART_HandleTypeDef huart6 = { &sim_USART6, NULL, 0, 115200 };

/* 句柄 -> 语义编号（UART_ID_e）。必须与产品 bsp_uart.c 的 s_huart[] 一致，
   否则仿真会把指令投给错误的"对端设备"，测试等于白跑 */
static int uart_index(UART_HandleTypeDef *h)
{
    if (h == &huart6) { return 0; } /* SCREEN  USART6 */
    if (h == &huart2) { return 1; } /* K230    USART2 */
    if (h == &huart4) { return 2; } /* SPARE   UART4  */
    if (h == &huart3) { return 3; } /* MOTOR_A USART3 */
    if (h == &huart5) { return 4; } /* MOTOR_B UART5  */
    if (h == &huart1) { return 5; } /* TRAY    USART1 */
    return -1;
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    return sim_plant_pin_read(port, pin) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, uint8_t *data, uint16_t len, uint32_t timeout)
{
    int i = uart_index(h);
    uint32_t baud;
    uint32_t us;
    int silent_loss;

    (void)timeout;
    if (i < 0) {
        return HAL_ERROR;
    }
    /* 半截帧回归注入：让"本轮第 nth 次屏发送调用"的字节在屏端静默丢失
       （HAL 照常成功 —— 真机上"屏没收到"本来就无法从 TX 侧检出）。
       产品实现按整帧发送时，丢的是完整一条指令，屏解析不失步；
       若实现把一条指令拆多次发送，丢到的就是指令碎片，下一条指令会被拼进残帧 */
    silent_loss = 0;
    if ((i == 0) && (s_drop_armed_ms != 0u) && (sim_now_ms() >= s_drop_armed_ms)) {
        if (s_drop_seen == s_drop_nth) {
            s_drop_armed_ms = 0u;   /* 一次性 */
            s_drop_count++;
            s_drop_line = sim_tjc_line_count();
            silent_loss = 1;
            if (g_sim.verbose) {
                printf("  [%6.2fs][注入] 屏端静默丢失第%u次发送调用的字节\n",
                       (double)sim_now_ms() / 1000.0, s_drop_nth + 1u);
            }
        } else {
            s_drop_seen++;
        }
    }
    baud = h->baud;
    us = ((uint32_t)len * 10u * 1000000u + baud - 1u) / baud;
    if (!silent_loss && !g_sim.serial_dead[i]) {
        sim_plant_uart_txd(i, data, len);
    }
    /* 真机上 TX 移位寄存器照样跑：断线只是对端收不到，不改变本端耗时 */
    {
        uint64_t target = s_now_us + us;

        while (s_now_us < target) {
            step_us(STEP_US);
        }
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *h, uint8_t *buf, uint16_t len)
{
    if (uart_index(h) < 0 || len == 0u) {
        return HAL_ERROR;
    }
    h->rx_buf = buf;
    h->rx_len = len;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *h)
{
    h->rx_buf = NULL;
    h->rx_len = 0u;
    return HAL_OK;
}

int sim_uart_rx_attach(UART_HandleTypeDef *h, uint8_t byte)
{
    if (h->rx_buf == NULL) {
        return 0;
    }
    h->rx_buf[0] = byte;
    HAL_UART_RxCpltCallback(h);
    return 1;
}

UART_HandleTypeDef *sim_uart_handle(int id)
{
    /* 语义编号 -> 句柄，与 uart_index() / 产品 s_huart[] 同序 */
    switch (id) {
    case 0: return &huart6;
    case 1: return &huart2;
    case 2: return &huart4;
    case 3: return &huart3;
    case 4: return &huart5;
    case 5: return &huart1;
    default: return NULL;
    }
}

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *h, uint32_t ch)
{
    (void)h;
    (void)ch;
    return HAL_OK;
}

static jmp_buf s_err_jmp;
int sim_error_raised = 0;

void Error_Handler(void)
{
    printf("!! Error_Handler() 被调用（外设初始化失败）\n");
    sim_error_raised = 1;
    longjmp(s_err_jmp, 1);
}

int sim_boot(void) { return setjmp(s_err_jmp); }
