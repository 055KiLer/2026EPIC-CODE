/**
 ******************************************************************************
 * @file    sort_config.h (仿真替身)
 * @brief   先原样包含产品配置，再仅覆盖"文档 §6.2 中标注待机械回填"的占位坐标
 * @attention
 * 2026-10-01 起产品 sort_config.h 的几何/标定值已用台架实测值回填（第三象限口径），
 * 本文件的 SIM_* 与 sim_plant.c 的世界常量保持逐字一致作为"真实机器"真值；
 * 其余全部常量（时序、脉冲换算、包络、协议）一律来自产品文件，未做任何改动。
 * 构建时通过 -DSIM_FILL_GEOMETRY / -DSIM_TRAY_DIR=.. 选择场景配置。
 ******************************************************************************
 */
#ifndef __SIM_SORT_CONFIG_WRAP_H
#define __SIM_SORT_CONFIG_WRAP_H

#include "../../Config/sort_config.h"

/* "真实机器"的投放点坐标：必须与 sim_plant.c 里 g_plant 的默认值一致，
   sim_main 启动时会逐字比较并在不一致时直接终止仿真 */
#define SIM_DROP_X   (-70.0f)
#define SIM_DROP_Y   (-187.0f)

/* "真实机器"的视觉标定（视觉原点在机械坐标系里的位置 + 两轴符号）：
   同样必须与 sim_plant.c 的 g_plant 默认值一致，sim_main 启动时比较 */
#define SIM_VISION_ORIGIN_X (-80.0f)
#define SIM_VISION_ORIGIN_Y (-45.0f)
#define SIM_VISION_SIGN_VY_TO_X (+1.0f)
#define SIM_VISION_SIGN_VX_TO_Y (+1.0f)

#ifdef SIM_FILL_GEOMETRY
#undef CFG_DROP_X_MM
#undef CFG_DROP_Y_MM
#define CFG_DROP_X_MM   SIM_DROP_X
#define CFG_DROP_Y_MM   SIM_DROP_Y
/* 出厂配置（不定义 SIM_FILL_GEOMETRY）时产品侧视觉标定保持 0 = 未标定，
   此时视觉帧与机械坐标两轴交换后还差一个平移 —— S18 就测这个"未标定"表现 */
#undef CFG_VISION_ORIGIN_X_MM
#undef CFG_VISION_ORIGIN_Y_MM
#undef CFG_VISION_SIGN_VY_TO_X
#undef CFG_VISION_SIGN_VX_TO_Y
#define CFG_VISION_ORIGIN_X_MM   SIM_VISION_ORIGIN_X
#define CFG_VISION_ORIGIN_Y_MM   SIM_VISION_ORIGIN_Y
#define CFG_VISION_SIGN_VY_TO_X  SIM_VISION_SIGN_VY_TO_X
#define CFG_VISION_SIGN_VX_TO_Y  SIM_VISION_SIGN_VX_TO_Y
#endif

#ifdef SIM_TRAY_DIR
#undef CFG_TRAY_DIR_FORWARD
#define CFG_TRAY_DIR_FORWARD  SIM_TRAY_DIR
#endif

#ifdef SIM_AXIS_SIGN
#undef CFG_AXIS_A_SIGN
#undef CFG_AXIS_B_SIGN
#define CFG_AXIS_A_SIGN  SIM_AXIS_A
#define CFG_AXIS_B_SIGN  SIM_AXIS_B
#endif

#endif
