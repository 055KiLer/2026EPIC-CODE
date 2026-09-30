/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   中断与异常处理
  * @note    ⚠️ 故障处理里**先停电机再死循环**：产品工程现在是裸 while(1)
  *          （A 组遗留第 3 条），转盘以速度模式跑着时进 HardFault 会一直转到比赛结束。
  *          本文件是那条遗留的样板实现，验证后可直接移植回产品。
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
#include "dbg_config.h" /* 限位引脚宏 DBG_CFG_LIMIT_*（§3.5） */
#include "dbg_uart.h"
#include "usart.h"
#include "zdt_bus.h"

void NMI_Handler(void)
{
  Zdt_Bus_PanicStop(); /* 不确定是不是电机相关，一律先停 */
  while (1)
  {
  }
}

void HardFault_Handler(void)
{
  Zdt_Bus_PanicStop();
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  Zdt_Bus_PanicStop();
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  Zdt_Bus_PanicStop();
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  Zdt_Bus_PanicStop();
  while (1)
  {
  }
}

void DebugMon_Handler(void)
{
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

/* ★ 转盘线（USART1）+ 电机两条线（A=USART3 / B=UART5）+ 控制台（UART4）的接收中断都**自己把标志清干净**，
   不依赖 HAL 的状态机。
   血泪教训：这些函数如果留成**空函数**（早期总线只发不收），一旦打开 RXNEIE 就是灾难 ——
   中断里什么都不读，RXNE/ORE 永远置位 → 中断无限重进 → CPU 100% 卡在这里，
   现象是"上电横幅打一半就停住、之后命令一条都不回、只能复位"。
   现在的写法：无论什么状态都先读 SR + DR（这两下把 RXNE/ORE/FE/NE/PE 全清掉），
   所以绝不可能自锁；噪声由 dbg_uart.c 里的预算负责关掉 */
void USART1_IRQHandler(void)
{
  uint32_t sr = USART1->SR;
  uint8_t  byte = (uint8_t)USART1->DR;

  Dbg_Uart_IsrPushMotor(2u, byte, sr); /* 2 = 转盘 57 线（M3/USART1） */
}

void USART3_IRQHandler(void)
{
  uint32_t sr = USART3->SR;
  uint8_t  byte = (uint8_t)USART3->DR; /* 读 DR 才算清掉 RXNE/ORE */

  Dbg_Uart_IsrPushMotor(0u, byte, sr); /* 0 = 电机 A 线（M1/USART3） */
}

void UART5_IRQHandler(void)
{
  uint32_t sr = UART5->SR;
  uint8_t  byte = (uint8_t)UART5->DR;

  Dbg_Uart_IsrPushMotor(1u, byte, sr); /* 1 = 电机 B 线（M2/UART5） */
}

void UART4_IRQHandler(void)
{
  uint32_t sr = UART4->SR;
  uint8_t  byte = (uint8_t)UART4->DR;

  Dbg_Uart_IsrPushConsole(byte, sr);
}

/* 限位/原点开关的下降沿中断（dbg_config.h §3.5：PB0/PB1/PC5）。
   处理体在 dbg_gpio.c 的 HAL_GPIO_EXTI_Callback：只计数与记时刻，不做任何运动反应 ——
   台架只负责验证"沿进来了"，怎么响应沿是产品固件的事 */
void EXTI0_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(DBG_CFG_LIMIT_TRAY_PIN); /* PB0 转盘原点 */
}

void EXTI1_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(DBG_CFG_LIMIT_X_PIN); /* PB1 X 限位 */
}

void EXTI9_5_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(DBG_CFG_LIMIT_Y_PIN); /* PC5 Y 限位 */
}