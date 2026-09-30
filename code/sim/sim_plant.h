/**
 ******************************************************************************
 * @file    sim_plant.h
 * @brief   被控对象（机械 + 外设）模型接口
 ******************************************************************************
 */
#ifndef __SIM_PLANT_H
#define __SIM_PLANT_H
#include "sim_shim.h"

/* 每 100us 由内核调用一次：推进执行器、检测开关边沿、注入串口字节 */
void sim_plant_step_us(uint32_t dt_us);
/* 产品代码从某路串口发出一个字节时调用（电机指令解析 / 屏指令采集） */
void sim_plant_uart_txd(int uart_id, const uint8_t *data, uint16_t len);
/* 产品代码读开关引脚 */
int  sim_plant_pin_read(void *port, uint16_t pin);
/* 内核把字节交给产品代码的 RX 回调（受中断屏蔽控制） */
int  sim_plant_uart_ready_rx(int uart_id);
void sim_plant_uart_deliver_rx(int uart_id);

void sim_plant_reset(void);
void sim_plant_servo_poll(void);   /* 每个子步检查舵机档位变化 -> 抓/放事件 */

/* ★ 屏工程口径（"真实屏幕"的控件名）：6 行"成功数量" = n1~n6，行序 = 货物类型 ID 1~6。
   产品 sort_config.h 的 CFG_TJC_VAR_CNT_BASE / CFG_TJC_CNT_FIRST 必须与之一致
   （sim_main 启动时比对；数字部分另有编译期断言在 sim_plant.c） */
#define SIM_TJC_CNT_BASE  "n"
#define SIM_TJC_CNT_FIRST 1

/* ---- 场景可调的"真实机器"参数（故意与 sort_config.h 分开） ---- */
typedef struct
{
    double   cam_deg;            /* 凸块作用在微动开关上的角度宽度 */
    int      driver_ppr;         /* 驱动器实际细分数（电机轴脉冲/圈） */
    double   belt_mm;            /* 皮带轮每转直线行程 mm */
    double   gear;               /* 转盘总减速比（电机轴 -> 输出轴）= 皮带 30/16 × 减速箱 1:20 = 37.5 */
    int      phys_sign_a;        /* 真实 A 电机方向（-1 = 与主控假设相反） */
    int      phys_sign_b;
    int      tray_phys_sign;     /* 转盘"正转"的真实方向（-1 = CFG_TRAY_DIR_FORWARD 标定反了） */
    double   tray_start_deg;     /* 上电时转盘输出轴角度（决定回零是否要先退出凸块窗口） */
    double   home_x_mm, home_y_mm; /* 上电时机架所在位置（机器不自知） */
    float    drop_x, drop_y;     /* 投放点真实机械坐标 */
    double   vision_err_mm;      /* K230 坐标与实物的偏差 */
    /* ---- 视觉标定真值：视觉原点在机械坐标系里的位置 + 两轴符号（见电控文档 §4.1）---- */
    double   vision_origin_x_mm; /* 视觉原点 = 标定中心（托盘上距左边 87.5 / 距底部 82.5 mm） */
    double   vision_origin_y_mm;
    int      vision_sign_vy_to_x; /* 机械 X = 原点 + 符号 × 视觉 y */
    int      vision_sign_vx_to_y; /* 机械 Y = 原点 + 符号 × 视觉 x */
    uint8_t  verbose;
} plant_cfg_t;

extern plant_cfg_t g_plant;

/* ---- 观测量 ---- */
double tray_angle_deg(void);
int    sim_hard_stop_hits(void);
int    sim_held_item(void);
int    sim_item_state(int i);
int    sim_item_bin(int i);
int    sim_claw_us(void);
int    sim_lift_us(void);
int    sim_tjc_line_count(void);
const  char *sim_tjc_line(int i);
/* 屏上第 type 行（货物类型 ID 1~6）的"成功数量"文本："-" = 从未写过 */
const  char *sim_tjc_count(int type);
extern char g_tjc_t0[64], g_tjc_p0[16];
extern char g_tjc_cnt[6][16];
extern long g_zdt_frames[3], g_zdt_pos_frames, g_zdt_stop_frames, g_zdt_speed_frames, g_zdt_enable_frames;
extern long g_zdt_ign_busy;

#endif
