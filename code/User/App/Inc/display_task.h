/**
 * @file    display_task.h
 * @brief   显示任务（App 层）：队列驱动串口屏的 6 行"成功数量"逐行刷新
 * @note    题目红线：入盒 + 信息显示完成后才能分拣下一件——
 *          sort_task 流程为 Display_Show() -> Display_WaitDone(seq)，等待本轮这一条刷完。
 *          用序号而不是信号量：信号量只有 1 个格子，上一轮的迟到回执会让下一轮误以为已完成。
 *          序号还兼作 display 侧的**去重凭据**：同一序号 = 同一件的补投，只重发、不重复计数。
 *          "完成"的确切含义 = 本轮那 1 条 TJC 指令已交给 USART6（屏端渲染异步，主控不等 0x88）
 */
#ifndef __DISPLAY_TASK_H
#define __DISPLAY_TASK_H

#include "main.h"
#include "sort_config.h"

typedef struct {
    uint16_t seq;      /* 内部回执确认号（不上屏）：单调递增，兼作"这一件是否已计数"的凭据 */
    uint8_t  category; /* 货物类别 ID 1~6（已由 vision_task 校验）；**同时就是屏上的行号** */
} Display_Msg_t;

/* 创建 sort -> display 消息队列（MX_FREERTOS_Init 中、任务创建前调用） */
void DisplayMgr_CreateObjects(void);

void DisplayTask(void *argument);

/* 投递一条显示消息（非阻塞）；返回 0 = 队列满 */
uint8_t Display_Show(const Display_Msg_t *msg);

/* 等待指定序号那一轮刷屏完成；返回 0 = 超时（调用方应重投同一序号，不要跳过） */
uint8_t Display_WaitDone(uint16_t seq, uint32_t timeout_ms);

#endif