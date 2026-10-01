/**
 * @file    motion_app.h
 * @brief   运动执行器（App 层）：解算 + 下发 + 等待 + 记账可信度
 * @note    原点 = 双限位触发处，X 向右、Y 向下为正
 *          X/Y 限位是安全网不是定位手段：绝对位置靠回零标定一次 + 相对脉冲记账
 *          所有等待用 vTaskDelay；ISR 内不做任何发送
 */
#ifndef __MOTION_APP_H
#define __MOTION_APP_H

#include "main.h"

/* 清零记账坐标与盒号，并标记为不可信（回零前必须调用） */
void Motion_Init(void);

/* 守护任务入口：1ms 周期取走 EXTI 事件并急停（须在回零前创建） */
void Motion_SafetyTask(void *argument);

/**
 * @brief  绝对坐标运动（阻塞至估算时间结束）
 * @retval 1=按估算走完，记账已提交
 * @retval 0=出发前记账就不可信，或途中被急停（此时不会推进记账）
 */
uint8_t Motion_MoveXYBlocking(float x_mm, float y_mm, float speed_mm_s);

/**
 * @brief  XY 回零（先 X 后 Y；每轴：退出窗口→粗压→脱开→慢压取触发沿）
 * @retval 1=成功并已退到待机点  0=两次尝试均未拿到干净的触发沿
 * @note   回零期间守护任务让位（压限位是预期动作）；成功末尾自动退到 CFG_PARK_*
 */
uint8_t Motion_HomeXY(void);

/* 记账是否可信（被限位急停或发送失败后会变 0） */
uint8_t Motion_XyIsTrusted(void);

/* 转盘回零：速度模式单向压原点凸块，触发沿 = 索引零位 */
uint8_t Motion_TrayHome(void);

/* 转盘记账是否可信（丢步/索引沿对不上/发送失败后会变 0） */
uint8_t Motion_TrayIsTrusted(void);

/**
 * @brief  异步定位转盘到盒号：只发指令立即返回，与 XY 运动并行
 * @retval 估算运动时间 ms（0 = 已在位或无法下发）
 * @note   调用方必须等够时间后调用 Motion_TrayFinish 做索引复核并提交记账
 */
uint32_t Motion_TrayGotoBin(uint8_t bin);

/**
 * @brief  结束一笔转盘定位：补齐剩余等待 + 索引沿复核 + 提交盒号记账
 * @param  est_ms GotoBin 的返回值；内部按 HAL_GetTick 自动扣除已流逝时间
 * @retval 1=落点可信并已提交  0=不可信（下一件前应重新回零）
 */
uint8_t Motion_TrayFinish(uint32_t est_ms);

/* 当前记账盒号（仅供显示/调试） */
uint8_t Motion_TrayBin(void);

#endif