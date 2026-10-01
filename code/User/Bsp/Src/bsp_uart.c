/**
 * @file    bsp_uart.c
 * @brief   6 路串口：单字节中断接收 -> 环形缓冲；发送阻塞（寄存器级，不走 HAL）
 * @note    ★ 2026-10-01 对齐台架 dbg_uart 已实测根治的收发契约：
 *          1) 发送不走 HAL_UART_Transmit —— 它拿 gState 当门，不同 HAL 版本在超时后
 *             不一定复位，一旦卡 BUSY 此后每次调用一个字节都发不出去；且超时判定读
 *             tick，tick 被中断风暴冻住时永不满足。这里直接轮询 TXE/TC 写 DR、超时
 *             用纯计数，返回前等 TC = 最后一个字节真的移出线路（返回 = 已上线）。
 *             并发调用用逐路忙锁串行化（老 HAL 路径靠 gState 天然串行，这里显式补上）。
 *          2) RX 只武装真正消费接收的线路（视觉线）：屏只发不收、电机线广播不回包
 *             （电控文档 §4.3"主控只发不收"）、UART4 预留 —— 未用线路的 RX 开着 =
 *             悬空拾噪 → 错误中断风暴 → SysTick 冻结（台架实测过整机停摆）。
 *          3) 视觉线加噪声预算：500ms 窗口内错误超 8 次关 RX，2s 后自动重开
 *             （台架 DBG_CFG_DEV_LINE_REARM_MS 同款），线路恢复即自愈、无需复位。
 *          仿真分支例外：仿真台靠 HAL_UART_Transmit 注入"屏半截帧丢失/按波特率计时"
 *          （s19 场景依赖），所以 SIM 构建下发送走 HAL 路径，见 sim/Makefile 的 -DSIM
 */
#include "bsp_uart.h"
#include "usart.h"
#include "sort_config.h"

#define BUF_SIZE 256u
#define BUF_MASK (BUF_SIZE - 1u)

/* 视觉线噪声预算（台架 dbg_uart 同款数值） */
#define RX_ERR_WINDOW_MS 500u
#define RX_ERR_BUDGET    8u
#define RX_REARM_MS      2000u

typedef struct {
    volatile uint16_t head;
    volatile uint16_t tail;
    uint8_t           mem[BUF_SIZE];
} Uart_Ring_t;

static Uart_Ring_t s_rx[UART_ID_NUM];
static uint8_t     s_byte[UART_ID_NUM];

/* RX 武装状态：只有视觉线常开（文件头"接收纪律"）。ISR 与守护任务（Poll）共享 */
static volatile uint8_t s_rx_armed[UART_ID_NUM];
/* 视觉线噪声预算：错误窗口起点、窗口内错误数、关断后的重开时刻（0 = 未关断） */
static uint32_t          s_rx_err_win_tick;
static volatile uint32_t s_rx_err_cnt;
static volatile uint32_t s_rx_rearm_tick;
/* 逐路发送忙锁：同一口并发调用时串行化，保证"一条 Send = 一条完整帧不交错" */
static volatile uint8_t s_tx_busy[UART_ID_NUM];

/* 语义编号 -> 句柄，不是按外设编号直排。必须与 sort_config.h 顶部六行逐行同序：
   直排会让屏与转盘互换，引脚复用与波特率全都对得上，表现却是"屏不刷新 + 转盘不动" */
static UART_HandleTypeDef *const s_huart[UART_ID_NUM] = {
    &huart6, /* SCREEN  USART6 PC6/PC7   */
    &huart2, /* K230    USART2 PD5/PD6   */
    &huart4, /* SPARE   UART4  PA0/PA1   */
    &huart3, /* MOTOR_A USART3 PC10/PC11 */
    &huart5, /* MOTOR_B UART5  PC12/PD2  */
    &huart1  /* TRAY    USART1 PB6/PB7   */
};

void Bsp_Uart_Init(void) {
    for (int i = 0; i < UART_ID_NUM; i++) {
        s_rx[i].head = 0u;
        s_rx[i].tail = 0u;
        s_rx_armed[i] = 0u;
        s_tx_busy[i]  = 0u;
    }
    s_rx_err_win_tick = 0u;
    s_rx_err_cnt      = 0u;
    s_rx_rearm_tick   = 0u;

    /* 只武装视觉线（文件头"接收纪律"）；其余线路要收数据时按"用前武装"补 */
    s_rx_armed[UART_ID_K230] = 1u;
    HAL_UART_Receive_IT(s_huart[UART_ID_K230], &s_byte[UART_ID_K230], 1u);
}

/* 守护任务每圈调用（1ms）：视觉线被噪声预算关断后到点自动重开。
   真机上顺带补 UE/RE/TE 位（台架实测杂散 glitch 能把使能位打掉，不补 = 这条线静默聋掉）；
   仿真替身没有 RE/TE 宏，这段只属于真机 */
void Bsp_Uart_Poll(void) {
#if !defined(SIM)
    USART_TypeDef *u = s_huart[UART_ID_K230]->Instance;
    uint32_t want = (uint32_t)(USART_CR1_UE | USART_CR1_RE | USART_CR1_TE);

    if ((u->CR1 & want) != want) {
        u->CR1 |= want;
    }
#endif
    if ((s_rx_rearm_tick != 0u) && ((HAL_GetTick() - s_rx_rearm_tick) >= RX_REARM_MS)) {
        s_rx_rearm_tick          = 0u;
        s_rx_err_cnt             = 0u;
        s_rx_armed[UART_ID_K230] = 1u;
        HAL_UART_Receive_IT(s_huart[UART_ID_K230], &s_byte[UART_ID_K230], 1u);
    }
}

/* 短临界区忙锁（关中断只覆盖置位的一瞬，不覆盖整个发送）：拿不到就在预算内自旋。
   老实现靠 HAL gState 获得同样的串行化；寄存器路径必须显式补上 */
static uint8_t Tx_Lock(UART_ID_e id) {
    uint32_t guard = 4000000u;

    for (;;) {
        uint32_t primask = __get_PRIMASK();
        uint8_t  got     = 0u;

        __disable_irq();
        if (s_tx_busy[id] == 0u) {
            s_tx_busy[id] = 1u;
            got           = 1u;
        }
        __set_PRIMASK(primask);
        if (got != 0u) {
            return 1u;
        }
        if (guard-- == 0u) {
            return 0u; /* 持锁方卡死（硬件异常）：按发送失败处理，调用方作废记账 */
        }
    }
}

static void Tx_Unlock(UART_ID_e id) {
    s_tx_busy[id] = 0u;
}

uint8_t Bsp_Uart_Send(UART_ID_e id, const uint8_t *data, uint16_t len) {
    if (id >= UART_ID_NUM || data == NULL || len == 0u) {
        return 0u;
    }
    if (Tx_Lock(id) == 0u) {
        return 0u;
    }

#ifdef SIM
    /* 仿真：HAL_UART_Transmit 是仿真台的观察/注入点（屏半截帧丢失、按波特率计时），
       必须走 HAL 仿真台才看得见字节；HAL 在替身里总是立刻成功 */
    uint32_t t0 = HAL_GetTick();

    do {
        if (HAL_UART_Transmit(s_huart[id], (uint8_t *)data, len, 20u) == HAL_OK) {
            Tx_Unlock(id);
            return 1u;
        }
    } while ((HAL_GetTick() - t0) < CFG_UART_TX_TIMEOUT_MS);
    Tx_Unlock(id);
    return 0u;
#else
    /* ★ 不走 HAL（文件头"发送契约"）。超时用纯计数：115200 下一字节 ≈87µs，
       guard 只在硬件异常（TXE 永不置位）时兜底，正常路径跑不到 */
    USART_TypeDef *u = s_huart[id]->Instance;

    if ((u->CR1 & USART_CR1_UE) == 0u) {
        Tx_Unlock(id);
        return 0u; /* 外设没使能，别在这里等 */
    }
    for (uint16_t i = 0u; i < len; i++) {
        uint32_t guard = 200000u;

        while (((u->SR & USART_SR_TXE) == 0u) && (guard > 0u)) {
            guard--;
        }
        if (guard == 0u) {
            Tx_Unlock(id);
            return 0u;
        }
        u->DR = (uint32_t)data[i];
    }
    {
        uint32_t guard = 200000u;

        while (((u->SR & USART_SR_TC) == 0u) && (guard > 0u)) {
            guard--;
        }
    }
    Tx_Unlock(id);
    return 1u;
#endif
}

uint16_t Bsp_Uart_Available(UART_ID_e id) {
    if (id >= UART_ID_NUM) {
        return 0u;
    }
    return (uint16_t)((s_rx[id].head - s_rx[id].tail) & BUF_MASK);
}

uint8_t Bsp_Uart_ReadByte(UART_ID_e id) {
    if (id >= UART_ID_NUM) {
        return 0u; /* 调用前应确认 Available()>0，这里只保证不越界 */
    }

    Uart_Ring_t *ring = &s_rx[id];
    uint8_t      data = ring->mem[ring->tail];

    ring->tail = (uint16_t)((ring->tail + 1u) & BUF_MASK);
    return data;
}

void Bsp_Uart_Flush(UART_ID_e id) {
    if (id < UART_ID_NUM) {
        uint32_t primask = __get_PRIMASK();

        /* 与接收 ISR 共享 head/tail，必须原子取 head，否则可能写入过期值 */
        __disable_irq();
        s_rx[id].tail = s_rx[id].head;
        __set_PRIMASK(primask);
    }
}

static int Bsp_Uart_IndexOf(UART_HandleTypeDef *huart) {
    for (int i = 0; i < UART_ID_NUM; i++) {
        if (s_huart[i] == huart) {
            return i;
        }
    }
    return -1;
}

/* 故障路径专用发送：**不碰 HAL、不等 tick**。
   故障处理里 tick 可能已经停了（故障优先级高于 SysTick，它进不来），
   HAL_UART_Transmit 的超时判断要读 tick，会就此挂死 —— 而这条路的唯一任务
   就是"把停止帧送出去然后死"。直接轮询 TXE/TC 写 DR，超时用纯计数。
   正常流程请用 Bsp_Uart_Send（同款寄存器级实现 + 忙锁串行化） */
uint8_t Bsp_Uart_SendDirect(UART_ID_e id, const uint8_t *data, uint16_t len) {
    if (id >= UART_ID_NUM || data == NULL || len == 0u) {
        return 0u;
    }

    USART_TypeDef *u = s_huart[id]->Instance;

    if ((u->CR1 & USART_CR1_UE) == 0u) {
        return 0u; /* 外设没使能，别在这里等 */
    }

    for (uint16_t i = 0u; i < len; i++) {
        uint32_t guard = 200000u;

        while (((u->SR & USART_SR_TXE) == 0u) && (guard > 0u)) {
            guard--;
        }
        if (guard == 0u) {
            return 0u;
        }
        u->DR = (uint32_t)data[i];
    }

    uint32_t guard = 200000u;

    while (((u->SR & USART_SR_TC) == 0u) && (guard > 0u)) {
        guard--;
    }
    return 1u;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    int i = Bsp_Uart_IndexOf(huart);

    if (i >= 0) {
        Uart_Ring_t *ring = &s_rx[i];
        uint16_t     next = (uint16_t)((ring->head + 1u) & BUF_MASK);

        if (next != ring->tail) {
            ring->mem[ring->head] = s_byte[i];
            ring->head            = next;
        } else {
            /* 满：覆盖最旧（各数据流均有帧校验，可容忍） */
            ring->mem[ring->head] = s_byte[i];
            ring->tail            = (uint16_t)((ring->tail + 1u) & BUF_MASK);
            ring->head            = next;
        }
        if (s_rx_armed[i] != 0u) {
            HAL_UART_Receive_IT(huart, &s_byte[i], 1u);
        }
    }
}

/* 接收错误处置：只中止 RX（HAL_UART_Abort 是双向的，会把同时在飞的电机指令截断成
   半帧）。视觉线按噪声预算处置：窗口内零星错误立刻重挂；500ms 内超 8 次 = 线路异常
   （悬空/短路），关断静默、由 Bsp_Uart_Poll 到点重开试探 —— 台架实测错误风暴会打成
   中断风暴冻住 SysTick，"无脑立刻重挂"等于永动机 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    int i = Bsp_Uart_IndexOf(huart);

    if (i < 0) {
        return;
    }
    HAL_UART_AbortReceive(huart);
    s_rx_armed[i] = 0u;

    if (i == (int)UART_ID_K230) {
        uint32_t now = HAL_GetTick();

        if ((now - s_rx_err_win_tick) > RX_ERR_WINDOW_MS) {
            s_rx_err_win_tick = now;
            s_rx_err_cnt      = 0u;
        }
        s_rx_err_cnt++;
        if (s_rx_err_cnt >= RX_ERR_BUDGET) {
            s_rx_rearm_tick = now; /* 关断，Poll 在 RX_REARM_MS 后重开 */
        } else {
            s_rx_armed[UART_ID_K230] = 1u;
            HAL_UART_Receive_IT(huart, &s_byte[i], 1u);
        }
    }
    /* 其余线路本就不武装（文件头"接收纪律"）；万一武装了也保持关断，等显式重开 */
}
