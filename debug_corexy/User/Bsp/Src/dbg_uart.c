/**
 * @file    dbg_uart.c
 * @brief   控制台 + 电机 A/B/T 三条线 + 屏线 + 视觉线：单字节中断接收 -> 各自环形缓冲；发送阻塞
 * @note    2026-09-30 起与产品接线一致：电机 A=USART3、电机 B=UART5、转盘=USART1，一条线一台驱动器。
 *          2026-10-01 接入产品另两条链路：屏=USART6（CN2，只发）、视觉=USART2（CN3，只收）。
 *          六路共用同一套"自写 ISR + 环形缓冲"机制；设备线额外带噪声预算（见下）
 */
#include "dbg_uart.h"
#include "dbg_config.h"
#include "dbg_print.h" /* 线路体检要用 Dbg_Printf 打印结果 */
#include "usart.h"

#define BUF_SIZE DBG_UART_RX_BUF_SIZE
#define BUF_MASK (BUF_SIZE - 1u)

/* s_rx[] 下标：0=控制台 1=A 线 2=B 线 3=转盘线 4=屏线 5=视觉线 */
#define LINE_CONSOLE 0u
#define LINE_MOTOR_A 1u
#define LINE_MOTOR_B 2u
#define LINE_TRAY    3u
#define LINE_SCREEN  4u
#define LINE_VISION  5u
#define LINE_COUNT   6u

typedef struct {
    volatile uint16_t   head;
    volatile uint16_t   tail;
    uint8_t             mem[BUF_SIZE];
    uint8_t             byte;
    UART_HandleTypeDef *huart;
} Dbg_Rx_t;

static Dbg_Rx_t s_rx[LINE_COUNT];

/* ★ 电机线接收的"噪声预算"：电机线平时是**只发**的（驱动器回包才有意义，见 dbg_uart.h）。
   线上没接驱动器、或数据线没接好时，RX 引脚会悬空拾噪 → 帧错误中断不断重挂，
   形成中断风暴：主循环被彻底饿死，连 SysTick 都推进不了（HAL_GetTick 冻结），
   于是"打印停在半句、命令一条都不回、只能复位"——这类现象就是它造成的。
   对策：短时间内错够预算就**直接关掉该线接收**（连同错误中断），等下一次做应答探测
   （`ack`/`scan`/任何发帧前的 MotorFlush）再重新打开。电机线本来就以发送为主，代价极小 */
#define BUS_ERR_BUDGET    8u   /* 窗口内允许的"带错误标志字节"数（正常回包一个都不带） */
#define BUS_ERR_WINDOW_MS 500u

static uint8_t  s_err_count[LINE_COUNT];
static uint32_t s_err_tick[LINE_COUNT];
static uint8_t  s_rx_off[LINE_COUNT];    /* 1 = 该线接收已被主动关闭（等重新 arm）；控制台恒 0 */
static uint32_t s_off_tick[LINE_COUNT];  /* 被关的时刻：屏/视觉线到期自动重开（电机线按需再开） */

/* ★ UE 自愈计数（2026-09-26 台架实测新增）：两路 UART 的 CR1 曾被发现**同时**只剩
   RE|TE|RXNEIE、UE=0 —— 收发全停而主循环/SysTick 都活着，且固件内没有任何代码会清
   UE（grep 过全部写点）。是谁清的还没抓到（调试器？电气毛刺？），先让链路一丢就补
   并计数取证：stat 里能看到次数，再犯就有数据可查 */
static uint16_t s_ue_heal[LINE_COUNT]; /* [0]=控制台 [1]=A [2]=B [3]=转盘 [4]=屏 [5]=视觉 */

/* 电机槽位（Dbg_Motor_e：0=A 1=B 2=转盘）-> s_rx 下标（1/2/3） */
static uint8_t Dbg_Uart_LineOfMotor(uint8_t motor) {
    if (motor == 1u) {
        return LINE_MOTOR_B;
    }
    if (motor == 2u) {
        return LINE_TRAY;
    }
    return LINE_MOTOR_A; /* 0 与一切非法值都落到 A 线：发错线好过解引用野指针 */
}

static Dbg_Rx_t *Dbg_Uart_RxOf(UART_HandleTypeDef *huart) {
    for (uint8_t i = 0u; i < LINE_COUNT; i++) {
        if (s_rx[i].huart == huart) {
            return &s_rx[i];
        }
    }
    return NULL;
}

/* ★ 接收中断的开关一律**直接写寄存器位**，不用 __HAL_UART_ENABLE_IT：
   HAL 的 UART_IT_x 是"寄存器号<<28 | 位"的编码，把两个用 | 拼起来会破坏编码
   （UART_IT_RXNE|UART_IT_ERR = 0x30000021 → 被当成 CR3 的位），RXNEIE 根本置不上。
   实测代价：PA1 上明明量到 216 个边沿，UART 却一个字节都不进缓冲 —— 控制台"只出不进" */
static void Dbg_Uart_RxItEnable(UART_HandleTypeDef *h) {
    h->Instance->CR1 |= USART_CR1_RXNEIE;
    h->Instance->CR3 |= USART_CR3_EIE;
}

static void Dbg_Uart_RxItDisable(UART_HandleTypeDef *h) {
    h->Instance->CR1 &= ~USART_CR1_RXNEIE;
    h->Instance->CR3 &= ~USART_CR3_EIE;
}

static uint8_t Dbg_Uart_RxItIsEnabled(UART_HandleTypeDef *h) {
    return ((h->Instance->CR1 & USART_CR1_RXNEIE) != 0u) ? 1u : 0u;
}

/* 重新把某条线接收打开（含错误中断），并重置噪声预算。
   注意：**不用 HAL_UART_Receive_IT** —— 接收全靠自写的 USARTx_IRQHandler
   读 SR+DR 清标志（见 stm32f4xx_it.c），这条路不依赖 HAL 的 RxState，
   所以不会出现"状态不是 BUSY_RX 就既不清标志也不回调"的自锁 */
static void Dbg_Uart_LineArm(uint8_t line) {
    s_rx_off[line]    = 0u;
    s_err_count[line] = 0u;
    s_err_tick[line]  = HAL_GetTick();
    Dbg_Uart_RxItEnable(s_rx[line].huart);
}

static void Dbg_Uart_LineDisarm(uint8_t line) {
    s_rx_off[line]   = 1u;
    s_off_tick[line] = HAL_GetTick();
    Dbg_Uart_RxItDisable(s_rx[line].huart);
}

void Dbg_Uart_MotorArm(uint8_t motor) {
    Dbg_Uart_LineArm(Dbg_Uart_LineOfMotor(motor));
}

/* 关掉某条电机线接收：**这才是电机线的常态**。它是"只发"的，收只在做应答探测的
   那一小段时间里有意义；一直开着收，RX 悬空拾噪就会把中断打成风暴（实测过：
   横幅打一半停住、命令一条都不回、只能复位） */
void Dbg_Uart_MotorDisarm(uint8_t motor) {
    Dbg_Uart_LineDisarm(Dbg_Uart_LineOfMotor(motor));
}

/* 控制台"只出不进"时分清责任：PC 侧持续发字节，看 PA1 上有没有边沿。
   量不到边沿 = 字节根本没到引脚（线没接 / 模块 TX 没驱动），与固件无关；
   量得到边沿却收不到数据 = MCU 这一侧（引脚复用/中断）的问题 */
void Dbg_Uart_ProbeConsoleRx(void) {
    GPIO_InitTypeDef io = { 0 };
    uint8_t  first;
    uint8_t  prev;
    uint32_t edges = 0u;

    /* PA1 先从 UART 复用切成普通输入（上拉，与 UART 空闲电平一致） */
    io.Pin  = DBG_CFG_CONSOLE_RX_PIN;
    io.Mode = GPIO_MODE_INPUT;
    io.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DBG_CFG_CONSOLE_RX_GPIO, &io);

    first = ((DBG_CFG_CONSOLE_RX_GPIO->IDR & DBG_CFG_CONSOLE_RX_PIN) != 0u) ? 1u : 0u;
    prev  = first;

    for (uint32_t i = 0u; i < 3000000u; i++) { /* 密集采样，约 30~60ms */
        uint8_t v = ((DBG_CFG_CONSOLE_RX_GPIO->IDR & DBG_CFG_CONSOLE_RX_PIN) != 0u) ? 1u : 0u;

        if (v != prev) {
            edges++;
            prev = v;
        }
    }

    /* 还原成 UART 复用 */
    io.Mode      = GPIO_MODE_AF_PP;
    io.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    io.Alternate = DBG_CFG_CONSOLE_RX_AF;
    HAL_GPIO_Init(DBG_CFG_CONSOLE_RX_GPIO, &io);

    Dbg_Printf("[PROBE] PA1(控制台 RX): 采样起点=%s 边沿=%u 次\n", (first != 0u) ? "高" : "低",
               (unsigned)edges);
    Dbg_Printf("[PROBE]   PC 侧正在发字节时应看到很多边沿；边沿=0 就是字节没到引脚（线/模块）\n");
}

/* ---------- 中断入口（由 stm32f4xx_it.c 调用）---------- */

static void Dbg_RingPush(Dbg_Rx_t *rx, uint8_t byte) {
    uint16_t next = (uint16_t)((rx->head + 1u) & BUF_MASK);

    if (next != rx->tail) {
        rx->mem[rx->head] = byte;
        rx->head          = next;
    }
    /* 满：丢最新的（控制台丢一行重敲；线上的自检字节丢了再看一次） */
}

/* 控制台：永远只收，不做任何预算判断 —— 它是人敲命令的口，绝不能自己关掉 */
void Dbg_Uart_IsrPushConsole(uint8_t byte, uint32_t sr) {
    (void)sr;
    Dbg_RingPush(&s_rx[LINE_CONSOLE], byte);
}

/* 设备线（电机/屏/视觉）共用：噪声预算在这里执行。只有"带帧错误标志的字节"才计入噪声
   （正常回包/正常数据流不会带 ORE/FE/NE/PE），超预算就关掉该线接收，把 CPU 留给主循环。
   屏/视觉线被关掉后由 EnsureRxArmed 到期自动重开（电机线按需重开，见 Arm） */
static void Dbg_Uart_IsrPushLine(uint8_t line, uint8_t byte, uint32_t sr) {
    if ((sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) != 0u) {
        if ((HAL_GetTick() - s_err_tick[line]) > BUS_ERR_WINDOW_MS) {
            s_err_tick[line]  = HAL_GetTick();
            s_err_count[line] = 0u;
        }
        s_err_count[line]++;
        if (s_err_count[line] > BUS_ERR_BUDGET) {
            Dbg_Uart_LineDisarm(line);
            return;
        }
    }
    Dbg_RingPush(&s_rx[line], byte);
}

void Dbg_Uart_IsrPushMotor(uint8_t motor, uint8_t byte, uint32_t sr) {
    Dbg_Uart_IsrPushLine(Dbg_Uart_LineOfMotor(motor), byte, sr);
}

void Dbg_Uart_IsrPushScreen(uint8_t byte, uint32_t sr) {
    Dbg_Uart_IsrPushLine(LINE_SCREEN, byte, sr);
}

void Dbg_Uart_IsrPushVision(uint8_t byte, uint32_t sr) {
    Dbg_Uart_IsrPushLine(LINE_VISION, byte, sr);
}

void Dbg_Uart_Init(void) {
    s_rx[LINE_CONSOLE].huart = &DBG_CFG_CONSOLE_UART;
    s_rx[LINE_MOTOR_A].huart = &DBG_CFG_MOTOR_A_UART;
    s_rx[LINE_MOTOR_B].huart = &DBG_CFG_MOTOR_B_UART;
    s_rx[LINE_TRAY].huart    = &DBG_CFG_TRAY_UART;
    s_rx[LINE_SCREEN].huart  = &DBG_CFG_SCREEN_UART;
    s_rx[LINE_VISION].huart  = &DBG_CFG_VISION_UART;

    for (uint8_t i = 0u; i < LINE_COUNT; i++) {
        s_rx[i].head = 0u;
        s_rx[i].tail = 0u;
    }

    /* 控制台常开接收（人敲命令的口，必须一直在听）；接收全靠自写 ISR，不用 HAL 的接收状态机 */
    Dbg_Uart_RxItEnable(&DBG_CFG_CONSOLE_UART);

    /* ★ 电机线默认**不开接收**：要看回包的命令都会先 Dbg_Uart_MotorFlush()（它负责 arm），
       读完再 disarm。理由见本文件顶部的噪声预算说明 —— 一直开着会被脏线搞死 */
    Dbg_Uart_MotorDisarm(0u);
    Dbg_Uart_MotorDisarm(1u);
    Dbg_Uart_MotorDisarm(2u);

    /* ★ 屏/视觉线常开接收：视觉是持续数据流（10Hz 帧流），屏也可能发触控/返回数据；
       同样受噪声预算保护，被关掉后 EnsureRxArmed 每 DBG_CFG_DEV_LINE_REARM_MS 自动重开，
       所以"线没接"最多付每 2s 一窗的噪声预算，不会风暴 */
    Dbg_Uart_LineArm(LINE_SCREEN);
    Dbg_Uart_LineArm(LINE_VISION);
}

static uint16_t Dbg_Uart_AvailableOf(uint8_t idx) {
    return (uint16_t)((s_rx[idx].head - s_rx[idx].tail) & BUF_MASK);
}

static uint8_t Dbg_Uart_ReadOf(uint8_t idx) {
    uint8_t data = s_rx[idx].mem[s_rx[idx].tail];

    s_rx[idx].tail = (uint16_t)((s_rx[idx].tail + 1u) & BUF_MASK);
    return data;
}

uint16_t Dbg_Uart_Available(void) {
    return Dbg_Uart_AvailableOf(LINE_CONSOLE);
}

uint8_t Dbg_Uart_ReadByte(void) {
    return Dbg_Uart_ReadOf(LINE_CONSOLE);
}

uint16_t Dbg_Uart_MotorAvailable(uint8_t motor) {
    return Dbg_Uart_AvailableOf(Dbg_Uart_LineOfMotor(motor));
}

uint8_t Dbg_Uart_MotorReadByte(uint8_t motor) {
    return Dbg_Uart_ReadOf(Dbg_Uart_LineOfMotor(motor));
}

/* ---- 屏线 / 视觉线（Flush 语义同 MotorFlush：清积压 + 重新武装） ---- */

static void Dbg_Uart_DevFlush(uint8_t line) {
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_rx[line].tail = s_rx[line].head;
    __set_PRIMASK(primask);
    Dbg_Uart_LineArm(line);
}

uint16_t Dbg_Uart_ScreenAvailable(void) {
    return Dbg_Uart_AvailableOf(LINE_SCREEN);
}

uint8_t Dbg_Uart_ScreenReadByte(void) {
    return Dbg_Uart_ReadOf(LINE_SCREEN);
}

void Dbg_Uart_ScreenFlush(void) {
    Dbg_Uart_DevFlush(LINE_SCREEN);
}

uint16_t Dbg_Uart_VisionAvailable(void) {
    return Dbg_Uart_AvailableOf(LINE_VISION);
}

uint8_t Dbg_Uart_VisionReadByte(void) {
    return Dbg_Uart_ReadOf(LINE_VISION);
}

void Dbg_Uart_VisionFlush(void) {
    Dbg_Uart_DevFlush(LINE_VISION);
}

void Dbg_Uart_MotorFlush(uint8_t motor) {
    uint8_t line   = Dbg_Uart_LineOfMotor(motor);
    uint32_t primask = __get_PRIMASK();

    /* 与接收 ISR 共享 head/tail，必须原子取 head */
    __disable_irq();
    s_rx[line].tail = s_rx[line].head;
    __set_PRIMASK(primask);

    /* 发帧前一律把该线接收重新打开：它可能因为噪声超预算被关掉了，
       而所有"要看回包"的命令（ack/scan/loop/j…）都会先 Flush，于是自动恢复 ✓ */
    Dbg_Uart_MotorArm(motor);
}

/* ★ 发送也**不走 HAL**：直接轮询 TXE/TC 写 DR。
   为什么：HAL_UART_Transmit 用 gState 当门，一旦某次发送超时后 HAL 没把 gState 复位
   （不同 HAL 版本行为不同），此后每次调用都直接返回 BUSY、一个字节都发不出去 ——
   现象就是"板子明明在跑、输出却停住不动"。这里的超时用纯计数，不依赖 HAL_GetTick，
   连"tick 被中断风暴冻住"也一并免疫 */
static uint8_t Dbg_Uart_SendBlocking(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t len) {
    USART_TypeDef *u = huart->Instance;
    uint32_t       guard;

    if ((data == NULL) || (len == 0u)) {
        return 0u;
    }
    if ((u->CR1 & USART_CR1_UE) == 0u) { /* 外设没使能：别在这里死等 */
        return 0u;
    }

    for (uint16_t i = 0u; i < len; i++) {
        guard = 200000u;
        while (((u->SR & USART_SR_TXE) == 0u) && (guard > 0u)) {
            guard--;
        }
        if (guard == 0u) {
            return 0u;
        }
        u->DR = (uint32_t)data[i];
    }

    guard = 200000u; /* 等最后一个字节真的移出去（TC），保证返回=已上线 */
    while (((u->SR & USART_SR_TC) == 0u) && (guard > 0u)) {
        guard--;
    }

    return 1u;
}

uint8_t Dbg_Uart_SendMotor(uint8_t motor, const uint8_t *data, uint16_t len) {
    return Dbg_Uart_SendBlocking(s_rx[Dbg_Uart_LineOfMotor(motor)].huart, data, len);
}

uint8_t Dbg_Uart_SendConsole(const uint8_t *data, uint16_t len) {
    return Dbg_Uart_SendBlocking(&DBG_CFG_CONSOLE_UART, data, len);
}

uint8_t Dbg_Uart_SendScreen(const uint8_t *data, uint16_t len) {
    return Dbg_Uart_SendBlocking(s_rx[LINE_SCREEN].huart, data, len);
}

uint32_t Dbg_Uart_MotorBaud(uint8_t motor) {
    return s_rx[Dbg_Uart_LineOfMotor(motor)].huart->Init.BaudRate;
}

uint8_t Dbg_Uart_SetMotorBaud(uint8_t motor, uint32_t bps) {
    UART_HandleTypeDef *h = s_rx[Dbg_Uart_LineOfMotor(motor)].huart;

    /* ZDT 支持的那几档，别让它接受奇怪的值 */
    if ((bps != 9600u) && (bps != 19200u) && (bps != 38400u) && (bps != 57600u) && (bps != 115200u) &&
        (bps != 230400u)) {
        return 0u;
    }

    (void)HAL_UART_DeInit(h);
    h->Init.BaudRate = bps;
    if (HAL_UART_Init(h) != HAL_OK) {
        return 0u;
    }

    /* ★ DeInit/Init 会把接收中断一起清掉：这里必须重新打开，否则换过波特率之后
       所有回包都是 0 字节，看起来就像"驱动器不应答" */
    Dbg_Uart_MotorArm(motor);
    return 1u;
}

/* ★ 接收自愈：串口一旦因为噪声/溢出出错，HAL 可能停在非 BUSY_RX 状态，此后再也不收字节，
   而现象就是"板子还在打、命令却一个都不进"。主循环每圈查一次，发现没挂在收就重新挂上。
   （这也是"必须复位才能恢复"那类怪象的根治手段）
   ★★ UE/RE/TE 自愈（2026-09-26 新增，见 s_ue_heal 注释）：三路都查 UE——电机线的 UE 丢了
   连发帧都会静默失败，比 RX 丢失更隐蔽。缺什么位就补什么位，并计数留证 */
void Dbg_Uart_EnsureRxArmed(void) {
    const uint32_t essentials = USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;

    for (uint8_t i = 0u; i < LINE_COUNT; i++) {
        UART_HandleTypeDef *h = s_rx[i].huart;

        if (h == NULL) {
            continue;
        }
        if ((h->Instance->CR1 & essentials) != essentials) {
            h->Instance->CR1 |= essentials; /* 只补位，不整体重写：保留其余配置 */
            s_ue_heal[i]++;
        }
        if (s_rx_off[i] != 0u) {
            if ((i == LINE_SCREEN) || (i == LINE_VISION)) {
                /* ★ 屏/视觉线要**常收**（视觉是持续数据流）：被噪声预算关掉后到期自动重开，
                   "线没接"最多每 2s 付一窗噪声预算，不会风暴，也不需要人敲命令来恢复 */
                if ((HAL_GetTick() - s_off_tick[i]) > DBG_CFG_DEV_LINE_REARM_MS) {
                    Dbg_Uart_LineArm(i);
                }
            } else {
                /* 电机线被噪声预算主动关掉时不要偷偷重开，否则风暴又会回来（按需再开） */
                continue;
            }
        }
        if (Dbg_Uart_RxItIsEnabled(h) == 0u) {
            Dbg_Uart_RxItEnable(h);
        }
    }
}

uint16_t Dbg_Uart_UeHealCount(uint8_t which) {
    return (which < LINE_COUNT) ? s_ue_heal[which] : 0u;
}
