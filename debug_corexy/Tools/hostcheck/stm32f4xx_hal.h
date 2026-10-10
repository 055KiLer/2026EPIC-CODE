/**
 ******************************************************************************
 * @file    stm32f4xx_hal.h（★ 主机端编译校验用的**替身**，不参与真机构建）
 * @brief   给 debug_corexy 的 User 各层源码做 gcc -fsyntax-only 用的最小 HAL/CMSIS 面。
 *
 * 为什么有这个东西：本机没有 Keil（真机构建在用户机上），改台架代码时唯一的
 * 把关手段就是"在这里编一遍"。它只声明真机 HAL 里被 User 层用到的类型/函数，
 * **值域与位定义都不保证**（例：IRQn_Type 的数值随便给），因此它只能证明
 * "语法/类型/名字对得上"，证明不了"运行时行为对"。
 *
 * 用法（在 debug_corexy 目录下）：  bash Tools/hostcheck/check.sh
 *   ⇒ 它用本目录的替身头把 User 四层 Src 下的 .c 全过一遍，**0 错误 0 警告**才算过
 *   （Core/Src 不在校验范围：那里的时钟/外设初始化真机才验得了）
 *
 * ⚠️ 与 Tools/shim/ 的区别：shim 是**给已抽出的纯逻辑模块**（打印格式器/帧层/解析器）
 * 做单元测试时顶掉真配置用的；本目录是**给整个 User 树**做语法校验用的，两者互不替代。
 ******************************************************************************
 */
#ifndef __STM32F4XX_HAL_H
#define __STM32F4XX_HAL_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 基本类型 ==================== */
typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;

/* 引脚号（真机是 (uint16_t)0x0001 一类，值必须一致：sort_config.h 的编译期断言要比较） */
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
#define GPIO_PIN_13 ((uint16_t)0x2000)
#define GPIO_PIN_14 ((uint16_t)0x4000)
#define GPIO_PIN_15 ((uint16_t)0x8000)

/* ==================== GPIO ==================== */
typedef struct {
    volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2];
} GPIO_TypeDef;

typedef struct {
    uint32_t Pin, Mode, Pull, Speed, Alternate;
} GPIO_InitTypeDef;

#define GPIO_MODE_INPUT       0x00000000u
#define GPIO_MODE_OUTPUT_PP   0x00000001u
#define GPIO_MODE_AF_PP       0x00000002u
#define GPIO_MODE_IT_RISING   0x10110000u
#define GPIO_MODE_IT_FALLING  0x10210000u
#define GPIO_MODE_IT_RISING_FALLING 0x10310000u
#define GPIO_NOPULL           0x00000000u
#define GPIO_PULLUP           0x00000001u
#define GPIO_PULLDOWN         0x00000002u
#define GPIO_SPEED_FREQ_LOW        0x00000000u
#define GPIO_SPEED_FREQ_MEDIUM     0x00000001u
#define GPIO_SPEED_FREQ_HIGH       0x00000002u
#define GPIO_SPEED_FREQ_VERY_HIGH  0x00000003u

/* 复用功能号（真机是 GPIO_AF7_USART1 一类的宏；这里只做名字核对，值不保证） */
#define GPIO_AF7_USART1 7u
#define GPIO_AF7_USART2 7u
#define GPIO_AF7_USART3 7u
#define GPIO_AF8_UART4  8u
#define GPIO_AF8_UART5  8u
#define GPIO_AF8_USART6 8u
#define GPIO_AF2_TIM3   2u
#define GPIO_AF2_TIM4   2u

#define GPIOA ((GPIO_TypeDef *)0x40020000u)
#define GPIOB ((GPIO_TypeDef *)0x40020400u)
#define GPIOC ((GPIO_TypeDef *)0x40020800u)
#define GPIOD ((GPIO_TypeDef *)0x40020C00u)
#define GPIOE ((GPIO_TypeDef *)0x40021000u)

/* ==================== TIM ==================== */
typedef struct {
    volatile uint32_t CR1, CR2, SMCR, DIER, SR, EGR, CCMR1, CCMR2, CCER,
        CNT, PSC, ARR, RCR, CCR1, CCR2, CCR3, CCR4, BDTR, DCR, DMAR, OR;
} TIM_TypeDef;

typedef struct {
    uint32_t Prescaler, CounterMode, Period, ClockDivision, RepetitionCounter,
        AutoReloadPreload;
} TIM_Base_InitTypeDef;

typedef struct {
    uint32_t OCMode, Pulse, OCPolarity, OCNPolarity, OCFastMode, OCIdleState, OCNIdleState;
} TIM_OC_InitTypeDef;

typedef struct {
    TIM_TypeDef *Instance;
    TIM_Base_InitTypeDef Init;
    uint32_t Channel;
    void *hdma[7];
    void *Lock;
    uint32_t State;
} TIM_HandleTypeDef;

#define TIM_CHANNEL_1 0x00000000u
#define TIM_CHANNEL_2 0x00000004u
#define TIM_CHANNEL_3 0x00000008u
#define TIM_CHANNEL_4 0x0000000Cu
#define TIM_COUNTERMODE_UP 0x00000000u
#define TIM_OCMODE_PWM1 0x00000060u
#define TIM_OCPOLARITY_HIGH 0x00000000u
#define TIM_CLOCKDIVISION_DIV1 0x00000000u
#define TIM_AUTORELOAD_PRELOAD_DISABLE 0x00000000u

/* ==================== UART ==================== */
typedef struct {
    volatile uint32_t SR, DR, BRR, CR1, CR2, CR3, GTPR;
} USART_TypeDef;

/* 寄存器位（dbg_uart.c 的 UE 自愈/错误标志巡检用；值照真机 F4 手册） */
#define USART_SR_PE     (1u << 0)
#define USART_SR_FE     (1u << 1)
#define USART_SR_NE     (1u << 2)
#define USART_SR_ORE    (1u << 3)
#define USART_SR_RXNE   (1u << 5)
#define USART_SR_TC     (1u << 6)
#define USART_SR_TXE    (1u << 7)
#define USART_CR1_RE    (1u << 2)
#define USART_CR1_TE    (1u << 3)
#define USART_CR1_RXNEIE (1u << 5)
#define USART_CR1_UE    (1u << 13)
#define USART_CR3_EIE   (1u << 0)

typedef struct {
    uint32_t BaudRate, WordLength, StopBits, Parity, Mode, HwFlowCtl, OverSampling;
} UART_InitTypeDef;

typedef struct {
    USART_TypeDef *Instance;
    UART_InitTypeDef Init;
    uint8_t *pTxBuffPtr;
    uint16_t TxXferSize, TxXferCount;
    uint8_t *pRxBuffPtr;
    uint16_t RxXferSize, RxXferCount;
    void *hdmatx;
    void *hdmarx;
    void *Lock;
    uint32_t gState, RxState;
} UART_HandleTypeDef;

#define USART1 ((USART_TypeDef *)0x40011000u)
#define USART2 ((USART_TypeDef *)0x40004400u)
#define USART3 ((USART_TypeDef *)0x40004800u)
#define UART4  ((USART_TypeDef *)0x40004C00u)
#define UART5  ((USART_TypeDef *)0x40005000u)
#define USART6 ((USART_TypeDef *)0x40011400u)
#define TIM4   ((TIM_TypeDef *)0x40000800u)

#define UART_WORDLENGTH_8B 0x00000000u
#define UART_STOPBITS_1    0x00000000u
#define UART_PARITY_NONE   0x00000000u
#define UART_MODE_TX_RX    0x0000000Cu
#define UART_HWCONTROL_NONE 0x00000000u
#define UART_OVERSAMPLING_16 0x00000000u
#define UART_IT_RXNE 0x00000020u

/* ==================== 中断号 ==================== */
typedef enum {
    NonMaskableInt_IRQn = -14,
    EXTI0_IRQn = 6,
    EXTI1_IRQn = 7,
    EXTI9_5_IRQn = 23,
    USART1_IRQn = 37,
    USART2_IRQn = 38,
    USART3_IRQn = 39,
    UART4_IRQn = 52,
    UART5_IRQn = 53,
    USART6_IRQn = 71
} IRQn_Type;

/* ==================== CMSIS 内核替身 ==================== */
typedef struct { volatile uint32_t DHCSR, DCRSR, DCRDR, DEMCR; } CoreDebug_Type;
typedef struct { volatile uint32_t CTRL, CYCCNT, CPICNT, EXCCNT, SLEEPCNT, LSUCNT,
                                    FOLDCNT, PCSR; } DWT_Type;

#define CoreDebug ((CoreDebug_Type *)0xE000EDF0u)
#define DWT       ((DWT_Type *)0xE0001000u)
#define CoreDebug_DEMCR_TRCENA_Msk (1u << 24)
#define DWT_CTRL_CYCCNTENA_Msk     (1u << 0)

extern uint32_t SystemCoreClock;
void NVIC_SystemReset(void);

#define __get_PRIMASK()   (0u)
#define __set_PRIMASK(x)  do { (void)(x); } while (0)
#define __disable_irq()   do { } while (0)
#define __enable_irq()    do { } while (0)

/* ==================== HAL 函数（只声明被 User 层用到的那部分） ==================== */
HAL_StatusTypeDef HAL_GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_Init);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
void HAL_GPIO_EXTI_IRQHandler(uint16_t GPIO_Pin);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

uint32_t HAL_GetTick(void);
void HAL_IncTick(void);
void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority);
void HAL_NVIC_EnableIRQ(IRQn_Type IRQn);

HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_DeInit(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size,
                                    uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
void HAL_UART_IRQHandler(UART_HandleTypeDef *huart);
void HAL_UART_EnableIT(UART_HandleTypeDef *huart, uint32_t IT);
#define HAL_UART_ENABLE_IT(h, it) ((void)(h), (void)(it))

HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef *sConfig,
                                            uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

#define HAL_TIM_SET_COMPARE(h, ch, v) ((h)->Instance->CCR1 = (v))
#define __HAL_TIM_SET_COMPARE(h, ch, v) ((h)->Instance->CCR1 = (v))

/* Core/Src 里的初始化函数（User 层偶尔引用） */
void Error_Handler(void);
void MX_TIM4_Init(void);

/* RCC 时钟使能（User 层若用到，替身给空宏） */
#define __HAL_RCC_GPIOA_CLK_ENABLE() do { } while (0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() do { } while (0)
#define __HAL_RCC_GPIOC_CLK_ENABLE() do { } while (0)
#define __HAL_RCC_GPIOD_CLK_ENABLE() do { } while (0)
#define __HAL_RCC_SYSCFG_CLK_ENABLE() do { } while (0)

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4XX_HAL_H */