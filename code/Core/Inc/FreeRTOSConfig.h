/* USER CODE BEGIN Header */
/*
 * FreeRTOS Kernel V10.3.1 配置（STM32F407 @168MHz）
 ******************************************************************************
 */
/* USER CODE END Header */
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* USER CODE BEGIN Includes */
/* Section where include file can be added */
/* USER CODE END Includes */

#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
#include <stdint.h>
extern uint32_t SystemCoreClock;
#endif
#ifndef CMSIS_device_header
#define CMSIS_device_header "stm32f4xx.h"
#endif /* CMSIS_device_header */

#define configENABLE_FPU                         1
#define configENABLE_MPU                         0

#define configUSE_PREEMPTION                     1
#define configSUPPORT_STATIC_ALLOCATION          1
#define configSUPPORT_DYNAMIC_ALLOCATION         1
#define configUSE_IDLE_HOOK                      0
#define configUSE_TICK_HOOK                      0
/* 运行时守卫：栈溢出与对象分配失败都会立刻急停三台电机（hook 在 freertos.c） */
#define configCHECK_FOR_STACK_OVERFLOW            2
#define configUSE_MALLOC_FAILED_HOOK              1
#define configCPU_CLOCK_HZ                       ( SystemCoreClock )       /* 168MHz */
#define configTICK_RATE_HZ                       ((TickType_t)1000)        /* 1ms 节拍 */
#define configMAX_PRIORITIES                     ( 56 )
#define configUSE_SB_COMPLETED_CALLBACK          ( 0 )
#define configUSE_MINI_LIST_ITEM                ( 1 )
#define configMINIMAL_STACK_SIZE                 ((uint16_t)128)
#define configTOTAL_HEAP_SIZE                    ((size_t)15360)           /* heap_4，4 任务+idle+定时器 */
#define configMAX_TASK_NAME_LEN                  ( 16 )
#define configHEAP_CLEAR_MEMORY_ON_FREE          0
#define configUSE_TRACE_FACILITY                 1
#define configUSE_16_BIT_TICKS                   0
#define configUSE_MUTEXES                        1
#define configQUEUE_REGISTRY_SIZE                8
#define configUSE_RECURSIVE_MUTEXES              1
#define configUSE_COUNTING_SEMAPHORES            1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  0
/* USER CODE BEGIN MESSAGE_BUFFER_LENGTH_TYPE */
#define configMESSAGE_BUFFER_LENGTH_TYPE         size_t
/* USER CODE END MESSAGE_BUFFER_LENGTH_TYPE */

/* 协程不使用 */
#define configUSE_CO_ROUTINES                    0
#define configMAX_CO_ROUTINE_PRIORITIES          ( 2 )

/* 软件定时器：cmsis_os2.c 封装依赖定时器服务任务，保持使能 */
#define configUSE_TIMERS                         1
#define configUSE_OS2_TIMER                      1
#define configTIMER_TASK_PRIORITY                ( 2 )
#define configTIMER_QUEUE_LENGTH                 ( 10 )
#define configTIMER_TASK_STACK_DEPTH             ( 256 )

/* CMSIS-RTOS V2 flags */
#define configUSE_OS2_THREAD_SUSPEND_RESUME  1
#define configUSE_OS2_THREAD_ENUMERATE       1
#define configUSE_OS2_EVENTFLAGS_FROM_ISR    1
#define configUSE_OS2_THREAD_FLAGS           1
#define configUSE_OS2_MUTEX                  1

#define INCLUDE_vTaskPrioritySet             1
#define INCLUDE_uxTaskPriorityGet            1
#define INCLUDE_vTaskDelete                  1
#define INCLUDE_vTaskCleanUpResources        0
#define INCLUDE_vTaskSuspend                 1
#define INCLUDE_xTaskDelayUntil              1
#define INCLUDE_vTaskDelay                   1
#define INCLUDE_xTaskGetSchedulerState       1
#define INCLUDE_xTimerPendFunctionCall       1
#define INCLUDE_xQueueGetMutexHolder         1
#define INCLUDE_uxTaskGetStackHighWaterMark  1
#define INCLUDE_xTaskGetCurrentTaskHandle    1
#define INCLUDE_eTaskGetState                1

/*
 * The CMSIS-RTOS V2 FreeRTOS wrapper is dependent on the heap implementation used
 * by the application thus the correct define need to be enabled below
 */
#define USE_FreeRTOS_HEAP_4

/* Cortex-M specific definitions. */
#ifdef __NVIC_PRIO_BITS
/* __BVIC_PRIO_BITS will be specified when CMSIS is being used. */
#define configPRIO_BITS         __NVIC_PRIO_BITS
#else
#define configPRIO_BITS         4
#endif

#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY   15

/* 高于该优先级(数值更小)的中断禁止调用 FreeRTOS FromISR API。
 * 本项目 ISR 均不调用 FromISR API（EXTI=0、UART=5、TIM6 由 HAL_InitTick 设置） */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

#define configKERNEL_INTERRUPT_PRIORITY   ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!! */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY  ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

/* Normal assert() semantics without relying on the provision of an assert.h header file. */
/* USER CODE BEGIN 1 */
#define configASSERT( x ) if ((x) == 0) {taskDISABLE_INTERRUPTS(); for( ;; );}
/* USER CODE END 1 */

/* 内核端口中断向量映射到 CMSIS 标准名（startup_stm32f407xx.s 中的弱符号被覆盖）。
 * 注意：stm32f4xx_it.c 中不得再定义 SVC_Handler / PendSV_Handler / SysTick_Handler */
#define vPortSVCHandler    SVC_Handler
#define xPortPendSVHandler PendSV_Handler
/* SysTick_Handler 由 CMSIS_RTOS_V2/cmsis_os2.c 提供 */
#define USE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION 0

/* USER CODE BEGIN Defines */
/* Section where parameter definitions can be added (for instance, to override default ones in FreeRTOS.h) */
/* USER CODE END Defines */

#endif /* FREERTOS_CONFIG_H */