/**
 * @file    sort_task.h
 * @brief   分拣任务（App 层，电控文档 §3 流程的唯一实现处）
 */
#ifndef __SORT_TASK_H
#define __SORT_TASK_H

/* 硬件初始化已在 main() 中完成；本任务内完成回零等需要 RTOS 延时的动作 */
void SortTask(void *argument);

#endif