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

/* ★ 数量复核（2026-10-10）：仿真台按**现场配置**构建 —— 数量复核开、激光闸门关。
   模型的 K230 每帧都填**数量位的真值**（下标 6 = 托盘上还剩几件，见 sim_plant.c），
   所以"发车帧 vs 到账帧差 1"这条链在**全部场景**里都真在跑：只要有一件被误判/漏判，
   屏上计数就与真实入盒数不一致，那些场景当场变红（s24~s27 则专门验复核本身）。
   激光那一支（CFG_LASER_ENABLE=0）不在仿真里参与计数：现场就是因为"爪挡光路"把它关掉的
   （见产品 sort_config.h 该节），它的验收工具是台架 `laser drop` / `laser verify`
   （dbg_config.h §3.6）。这里两个宏都**显式**钉住，免得哪天产品缺省改了、仿真悄悄换了口径 */
#undef CFG_LASER_ENABLE
#define CFG_LASER_ENABLE 0
#undef CFG_COUNT_VERIFY_ENABLE
#define CFG_COUNT_VERIFY_ENABLE 1

/* ★ 反证用：#define SIM_NO_CLAMP_BAND 把 +侧极限位夹取带临时关成 0 —— s22 必须变红
   （"夹不到也去夹"没了 ⇒ 整件作废、一次都不伸爪）。用完删掉这一行 */
#ifdef SIM_NO_CLAMP_BAND
#undef CFG_XY_X_CLAMP_BAND_MM
#undef CFG_XY_Y_CLAMP_BAND_MM
#define CFG_XY_X_CLAMP_BAND_MM 0.0f
#define CFG_XY_Y_CLAMP_BAND_MM 0.0f
#endif

#endif
