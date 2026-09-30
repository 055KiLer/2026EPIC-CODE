/**
 ******************************************************************************
 * @file    sort_config.h (仿真替身)
 * @brief   先原样包含产品配置，再仅覆盖"文档 §6.2 中标注待机械回填"的占位坐标
 * @attention
 * 产品 sort_config.h 把 CFG_DROP 两个值故意留成 0/0（未回填前拒绝一切绝对定位）。
 * 仿真的"S1 正常路径"必须假设机械坐标已回填，因此这里按场景把它们补上；
 * 其余全部常量（时序、脉冲换算、包络、协议）一律来自产品文件，未做任何改动。
 * 构建时通过 -DSIM_FILL_GEOMETRY / -DSIM_TRAY_DIR=.. 选择场景配置。
 ******************************************************************************
 */
#ifndef __SIM_SORT_CONFIG_WRAP_H
#define __SIM_SORT_CONFIG_WRAP_H

#include "../../Config/sort_config.h"

/* "真实机器"的投放点坐标：必须与 sim_plant.c 里 g_plant 的默认值一致，
   sim_main 启动时会逐字比较并在不一致时直接终止仿真 */
#define SIM_DROP_X   200.0f
#define SIM_DROP_Y   340.0f

/* "真实机器"的视觉标定（视觉原点在机械坐标系里的位置 + 两轴符号）：
   同样必须与 sim_plant.c 的 g_plant 默认值一致，sim_main 启动时比较 */
#define SIM_VISION_ORIGIN_X 60.0f
#define SIM_VISION_ORIGIN_Y 45.0f
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

/* 上电驱动器自检在仿真里关掉：它要"等回包"，而仿真的时钟只在任务让出时推进，
   忙等会把自己挂住；仿真要覆盖的是运动学与记账，不是驱动器通信（那条靠主机侧
   帧测试 + 台架实测覆盖） */
#undef CFG_SELFTEST_ON_BOOT
#define CFG_SELFTEST_ON_BOOT 0

#endif
