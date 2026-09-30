/**
  ******************************************************************************
  * @file    usart.c
  * @brief   四路串口初始化：USART1 转盘线 + USART3 电机A线 + UART5 电机B线 + UART4 控制台
  * @note    2026-09-30 起接线与产品工程一致（PCB 已修复）：两台 CoreXY 驱动器
  *          各占一路 UART（M1/M2），不再共用一条总线
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "dbg_config.h"

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart4;

/* USART1：转盘 57 线，PB6(TX)/PB7(RX)，115200（产品板 M3 座，与产品 CFG_UART_TRAY 同口） */
void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = DBG_CFG_MOTOR_BAUD;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USART3：电机 A 线，PC10(TX)/PC11(RX)，115200（产品板 M1 座，与产品 CFG_UART_MOTOR_A 同口） */
void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = DBG_CFG_MOTOR_BAUD;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* UART5：电机 B 线，PC12(TX)/PD2(RX)，115200（产品板 M2 座，与产品 CFG_UART_MOTOR_B 同口） */
void MX_UART5_Init(void)
{
  huart5.Instance = UART5;
  huart5.Init.BaudRate = DBG_CFG_MOTOR_BAUD;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* UART4：控制台，PA0(TX)/PA1(RX)，115200（产品板 CN1 预留口；接调试探针 CDC 桥时 TX/RX 交叉） */
void MX_UART4_Init(void)
{
  huart4.Instance = UART4;
  huart4.Init.BaudRate = DBG_CFG_CONSOLE_BAUD;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
}
