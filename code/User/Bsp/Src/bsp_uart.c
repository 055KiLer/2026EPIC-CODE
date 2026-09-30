/**
 * @file    bsp_uart.c
 * @brief   6 路串口：单字节中断接收 -> 环形缓冲；发送阻塞
 */
#include "bsp_uart.h"
#include "usart.h"
#include "sort_config.h"

#define BUF_SIZE        256u
#define BUF_MASK        (BUF_SIZE - 1u)
#define SEND_TIMEOUT_MS CFG_UART_TX_TIMEOUT_MS

typedef struct {
    volatile uint16_t head;
    volatile uint16_t tail;
    uint8_t           mem[BUF_SIZE];
} Uart_Ring_t;

static Uart_Ring_t s_rx[UART_ID_NUM];
static uint8_t     s_byte[UART_ID_NUM];

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
        HAL_UART_Receive_IT(s_huart[i], &s_byte[i], 1u);
    }
}

uint8_t Bsp_Uart_Send(UART_ID_e id, const uint8_t *data, uint16_t len) {
    if (id >= UART_ID_NUM || data == NULL || len == 0u) {
        return 0u;
    }

    /* 同一句柄正在发送时 HAL 立即返回 BUSY 且不发任何字节，必须重试而不是忽略 */
    uint32_t t0 = HAL_GetTick();

    do {
        if (HAL_UART_Transmit(s_huart[id], (uint8_t *)data, len, 20u) == HAL_OK) {
            return 1u;
        }
    } while ((HAL_GetTick() - t0) < SEND_TIMEOUT_MS);

    return 0u;
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

uint16_t Bsp_Uart_ReadReply(UART_ID_e id, uint8_t *buf, uint16_t cap, uint32_t timeout_ms) {
    if (id >= UART_ID_NUM || buf == NULL || cap == 0u) {
        return 0u;
    }

    uint32_t t0    = HAL_GetTick();
    uint32_t guard = 2000000u; /* 纯计数上限：tick 被冻住时也不挂死（台架踩过这类问题） */
    uint16_t n     = 0u;

    while (n < cap) {
        if (Bsp_Uart_Available(id) > 0u) {
            buf[n] = Bsp_Uart_ReadByte(id);
            n++;
            t0 = HAL_GetTick(); /* 收到字节后重新给超时计时，等这一包收完 */
            continue;
        }
        if ((HAL_GetTick() - t0) >= timeout_ms) {
            break;
        }
        if (guard-- == 0u) {
            break;
        }
    }

    return n;
}

/* 故障路径专用发送：**不碰 HAL、不等 tick**。
   故障处理里 tick 可能已经停了（故障优先级高于 SysTick，它进不来），
   HAL_UART_Transmit 的超时判断要读 tick，会就此挂死 —— 而这条路的唯一任务
   就是"把停止帧送出去然后死"。所以直接轮询 TXE/TC 写 DR，超时用纯计数。
   正常流程请用 Bsp_Uart_Send（它有 BUSY 重试，语义不同） */
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
        HAL_UART_Receive_IT(huart, &s_byte[i], 1u);
    }
}

/* 溢出等错误后重新挂接收，防止接收链路死掉。
   只中止 RX：HAL_UART_Abort 是双向的，会把同时在飞的电机指令截断成半帧 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    int i = Bsp_Uart_IndexOf(huart);

    if (i >= 0) {
        HAL_UART_AbortReceive(huart);
        HAL_UART_Receive_IT(huart, &s_byte[i], 1u);
    }
}