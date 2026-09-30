/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : FreeRTOS 任务创建（CubeMX 结构）
 * @attention
 * 任务清单：safetyTask（1ms 守护，优先级最高）> visionTask > displayTask > sortTask
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"

#include "sort_task.h"
#include "vision_task.h"
#include "display_task.h"
#include "motion_app.h"
#include "zdt_motor.h"

/* main.c 中声明了 MX_FREERTOS_Init 原型 */

/* Private variables ---------------------------------------------------------*/
static osThreadId_t s_sort_task_handle;
static osThreadId_t s_vision_task_handle;
static osThreadId_t s_display_task_handle;
static osThreadId_t s_safety_task_handle;

/* Definitions for safetyTask */
static const osThreadAttr_t s_safety_task_attr = {
    .name = "safetyTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityHigh,  /* 必须高于全部业务任务，否则急停会被拖延 */
};
/* Definitions for sortTask */
static const osThreadAttr_t s_sort_task_attr = {
    .name = "sortTask",
    .stack_size = 512 * 4,                      /* 分拣状态机 + 浮点运动学 */
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for visionTask */
static const osThreadAttr_t s_vision_task_attr = {
    .name = "visionTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityNormal2, /* 视觉解析优先于分拣调度 */
};
/* Definitions for displayTask */
static const osThreadAttr_t s_display_task_attr = {
    .name = "displayTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityNormal1,
};

/**
 * @brief  FreeRTOS 对象与任务创建（main 中 osKernelInitialize 之后调用）
 */
void MX_FREERTOS_Init(void)
{
    DisplayMgr_CreateObjects();

    s_safety_task_handle = osThreadNew(Motion_SafetyTask, NULL, &s_safety_task_attr);
    s_vision_task_handle = osThreadNew(VisionTask, NULL, &s_vision_task_attr);
    s_display_task_handle = osThreadNew(DisplayTask, NULL, &s_display_task_attr);
    s_sort_task_handle = osThreadNew(SortTask, NULL, &s_sort_task_attr);

    (void)s_safety_task_handle;
    (void)s_vision_task_handle;
    (void)s_display_task_handle;
    (void)s_sort_task_handle;
}

/* ============================ 运行时守卫（fail-safe） ============================ */

/* 两处故障都意味着"软件状态已不可信"，唯一正确的物理动作是让三台电机立刻停下。
   ZDT 没有硬件使能线可拉，只能走正常发送通道下发急停帧 */
static void Freertos_EmergencyHalt(void)
{
    /* HAL 阻塞发送的超时判断依赖 HAL_GetTick，而 hook 可能在临界区内被调用
       （BASEPRI 仍屏蔽着节拍中断），tick 一停发送就会永远卡在超时循环里，
       "急停"反而变成"永不停"。故障现场不需要恢复上下文，先无条件开中断 */
    __set_BASEPRI(0U);
    __enable_irq();

    (void)Zdt_Stop(ZDT_ID_A);
    (void)Zdt_Stop(ZDT_ID_B);
    (void)Zdt_Stop(ZDT_ID_TRAY);

    __disable_irq();
    for (;;) {
        /* 停在现场：等人工复位，绝不带着未知状态继续动 */
    }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    Freertos_EmergencyHalt();
}

void vApplicationMallocFailedHook(void)
{
    /* 任务/队列创建失败 = 某个任务根本不存在（比如守护任务没起来），必须停 */
    Freertos_EmergencyHalt();
}