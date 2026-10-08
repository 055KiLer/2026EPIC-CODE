/**
 * @file    motion_kinematics.h
 * @brief   CoreXY 运动学解算（Algorithm 层，纯数学，无设备/RTOS 依赖）
 * @note    A = X+Y，B = X-Y（符号在 sort_config.h 标定）
 *          单位：位移 mm、速度 mm/s、时间 ms；电机转速为张大头指令单位 RPM
 */
#ifndef __MOTION_KINEMATICS_H
#define __MOTION_KINEMATICS_H

#include "main.h"

/**
 * @brief  XY 相对位移 -> 双电机脉冲/转速/时间（按脉冲比例配速，同时到位）
 * @param  pulse_a/b 输出各电机相对脉冲（含方向符号，正 = CW）
 * @param  time_ms   输出估算运动时间（含余量）
 */
void Kinematics_XYSolve(float delta_x, float delta_y, float speed_mm_s, int32_t *pulse_a,
                        int32_t *pulse_b, float *rpm_a, float *rpm_b, uint32_t *time_ms);

/* 单轴：脉冲数 + 转速 -> 运动时间估算 ms（梯形曲线用常数余量近似） */
uint32_t Kinematics_MoveTimeMs(int32_t pulse, float rpm);

/* 转盘：pulses 为电机轴细分脉冲（已含 1:20）；pulse 与 rpm 口径一致，不得再乘减速比 */
uint32_t Kinematics_TrayMoveTimeMs(uint32_t pulses, float rpm);

/* 两个脉冲记账点之间经过几个凸块（= 索引沿理论次数）；双向都成立（有符号脉冲） */
uint32_t Kinematics_TrayCrossingsBetween(int32_t from_pulses, int32_t to_pulses);

/* 累计脉冲（有符号）-> 当前对准投放点的盒号（就近取整，自动处理整圈回绕/负值） */
uint8_t Kinematics_TrayBinOf(int32_t pulses);

/**
 * @brief  绝对坐标是否落在工作包络内
 * @retval 1=合法  0=越界。软件原点 (0,0) 就是限位触发处，越界点一律不得下发
 */
uint8_t Kinematics_XyInEnvelope(float x_mm, float y_mm);

#endif