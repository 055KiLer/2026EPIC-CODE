/**
  ******************************************************************************
  * @file    main.c
  * @brief   CoreXY + 夹爪 调试固件（产品板 / STM32F407VET6，接线与产品工程一致）
  * @note    与产品工程 `code/` 完全独立：裸机超循环、不启 RTOS。2026-09-30 PCB 修复后
  *          引脚同步为产品方案：转盘=USART1(M3)、电机 A=USART3(M1)、B=UART5(M2)，舵机=PB8/PB9，
  *          控制台=UART4(CN1)，限位=PB0/PB1/PC5。
  *          用途：先在这里验证完分拣流程，再换产品固件。
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"

/* USER CODE BEGIN Includes */
#include "dbg_config.h"
#include "dbg_app.h"
#include "dbg_console.h"
#include "dbg_gpio.h"
#include "dbg_print.h"
#include "dbg_uart.h"
#include "servo_out.h"
#include "zdt_bus.h"
/* USER CODE END Includes */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  /* 只开用得上的外设：两条电机线 + 控制台 + 舵机定时器；限位/原点开关照产品引脚
     PB0/PB1/PC5 接进来（dbg_config.h §3.5），用 `lim` 命令看电平与沿计数 */
  MX_USART1_UART_Init(); /* 转盘 57 线（产品板 M3，广播 0x00） */
  MX_USART3_UART_Init(); /* 电机 A 线（产品板 M1） */
  MX_UART5_Init();       /* 电机 B 线（产品板 M2） */
  MX_UART4_Init();       /* 控制台（产品板 CN1） */
  MX_TIM4_Init();        /* 舵机 PWM（PB8/PB9） */

  Dbg_Gpio_Init();   /* 限位/原点开关：EXTI 下降沿 + 上拉，与产品 gpio.c 同配置 */
  Dbg_Uart_Init();   /* 控制台接收中断 */
  Dbg_Print_Init();  /* 打印行缓冲 */
  Servo_Out_Init();  /* 舵机安全位：升降抬升 + 夹爪压 1500us 中位（角度未标定防堵转） */

  Dbg_App_Init();            /* 打配置 + 待核实项 + 上手提示 */
  Dbg_Console_PrintHelp();   /* 打命令表 */

  while (1)
  {
    /* 全部逻辑都在这里：收命令行 -> 执行。没有 RTOS，也就没有任务/栈/优先级问题 */
    Dbg_Console_Poll();
    Dbg_App_Tick();     /* 没收到过命令就定期重跑总线体检，方便随时打开串口看报告 */
  }
}

/**
  * @brief 时钟配置：HSI -> 168MHz
  * @note  刻意与产品工程一致（不用 HSE）：这样"速度/时间账"两边可直接对照。
  *        物创1 的板子有 8MHz 晶振，若要换成 HSE，改这里并把 PLLM 改成 8→4。
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
  RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;   /* APB1 = 42MHz（定时器 84MHz） */
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;   /* APB2 = 84MHz */

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  Zdt_Bus_PanicStop(); /* 初始化失败也要先让电机停，别带着未知状态跑 */
  __disable_irq();
  while (1)
  {
  }
}