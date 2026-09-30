/**
 ******************************************************************************
 * @file    sim_plant.c
 * @brief   被控对象模型：CoreXY 龙门 / 1:20 转盘 / 微动开关 / 舵机 / 视觉上位机(ROCK 5C) / TJC
 * @attention
 * 所有物理常数放在 g_plant 里，**独立于** sort_config.h（代表"真实机器"）。
 * 这样主控标定值写错时，仿真是真的走歪、真的撞限位，而不是自我印证。
 * **对端设备的帧格式也不能照抄产品代码**：ZDT 位置帧长度按 Emm_V5.0 官方手册写（13 字节），
 * 曾因照抄实现写成 12 字节，导致产品代码的同类错误在仿真里"全绿"（见《仿真审计结论》§3.1）。
 * 原扫码（条码模块）支路已整条删除，见 User/App/Src/sort_task.c 顶部的历史说明。
 ******************************************************************************
 */
#include "sim_plant.h"
#include "sim_kernel.h"
#include "tim.h"          /* shim: extern TIM_HandleTypeDef htim4 */
#include "sort_config.h"  /* shim 包装：CFG_LIMIT_PIN_* / CFG_ORIGIN_PIN_TRAY / CFG_TJC_* */
#include <stdio.h>
#include <stdlib.h>       /* atoi：屏控件名里的行号 */
#include <string.h>
#include <math.h>

/* ==================== 真实机器默认参数 ==================== */
plant_cfg_t g_plant;

/* 开关引脚号直接取产品那份宏（Config/sort_config.h 末尾有同值断言）。引脚只有一份真源：
   仿真代表"真实机器"，但它验的是产品代码的引脚逻辑，两边各写一份并没有额外收益，
   反而给"模型和产品一起漂移"留了口子。这里再断言一次，让那种情况也在编译期暴露。
   端口归属在 sim_plant_pin_read 里按原理图固定（X=GPIOB Y=GPIOC 原点=GPIOB） */
#define LIM_X_PIN   CFG_LIMIT_PIN_X
#define LIM_Y_PIN   CFG_LIMIT_PIN_Y
#define TRAY_PIN    CFG_ORIGIN_PIN_TRAY

/* 与产品侧同理：引脚号比较**不能写成 #if** —— HAL 的 GPIO_PIN_n 是 ((uint16_t)0x....)，
   预处理阶段不是合法常量表达式。断言失败 = "array size is negative"（2026-09-15 踩过：
   产品侧那段 #if 让真机构建直接 52 个 error，仿真却因为 shim 用裸字面量安然通过） */
typedef char sim_plant_assert_lim_x[(LIM_X_PIN == GPIO_PIN_1) ? 1 : -1];
typedef char sim_plant_assert_lim_y[(LIM_Y_PIN == GPIO_PIN_5) ? 1 : -1];
typedef char sim_plant_assert_tray[(TRAY_PIN == GPIO_PIN_0) ? 1 : -1];

/* 舵机档位脉宽：★ 直接取自 Config/sort_config.h 的标定宏（经 shim/sort_config.h 包含），
   不再手抄字面量——2026-09-26 与 09-28 两次都是"标定改了、这里没跟"，
   表现 = 仿真里舵机事件全部失联、货物永不入盒、分拣空转刷总数。
   与配置绑死之后这类漂移在编译期就不可能再发生 */
#define US_LIFT_DOWN   ((uint32_t)CFG_LIFT_DOWN_US)
#define US_LIFT_UP     ((uint32_t)CFG_LIFT_UP_US)
#define US_CLAW_CLOSE  ((uint32_t)CFG_CLAW_CLOSE_US)
#define US_CLAW_HALF   ((uint32_t)CFG_CLAW_HALF_US)
#define US_CLAW_OPEN   ((uint32_t)CFG_CLAW_OPEN_US)

/* ==================== 电机 ==================== */
typedef struct
{
    double pos;      /* 电机轴绝对脉冲（带符号） */
    double target;
    double vel;      /* 脉冲/s，带符号 */
    int    vel_mode;
    int    moving;
} motor_t;

static motor_t s_mot[3]; /* 0=A 1=B 2=TRAY */
/* 转盘输出轴"真实"累计，单位仍是电机轴脉冲：打滑只吃增量，
   所以注入时刻不能把已经转过的角度追溯性地缩掉（那不是物理） */
static double s_tray_phys;
static unsigned s_tray_edges;   /* 索引上升沿计数（仅诊断用） */

/* ==================== 龙门 / 开关真值 ==================== */
static double s_x_mm, s_y_mm;
static int s_lim_x, s_lim_y, s_tray_sw;

/* ==================== 货物 ==================== */
enum { IT_TRAY = 0, IT_HELD, IT_BIN, IT_LOST };
typedef struct
{
    sim_item_t spec;
    double x, y;
    int state;
    int bin;
} item_t;

static item_t s_item[SIM_MAX_ITEMS];
static int s_item_n;
static int s_held = -1;
static uint32_t s_last_lift, s_last_claw;

/* ==================== 串口链路模型 ==================== */
typedef struct
{
    uint8_t  q[512];
    int      head, tail;
    uint64_t next_us;
    int      us_per_byte;
} rxlink_t;

static rxlink_t s_rx[6];
static uint64_t s_k230_next_us;

/* ==================== ZDT 指令还原 ==================== */
static uint8_t s_acc[3][16];
static int     s_acc_n[3];
long           g_zdt_ign_busy;   /* 驱动器运动中拒绝/排队掉的位置帧数 */
long g_zdt_frames[3];
long g_zdt_pos_frames, g_zdt_stop_frames, g_zdt_speed_frames, g_zdt_enable_frames;

/* ==================== TJC 采集 ==================== */
#define TJC_LOG 1024
static char s_tjc_log[TJC_LOG][136];
static int  s_tjc_n;
static uint8_t s_tjc_acc[128];
static int  s_tjc_acc_n;
static int  s_tjc_ff;
char g_tjc_t0[64], g_tjc_p0[16];
char g_tjc_cnt[6][16];   /* 屏上 6 行"成功数量"，下标 0 = 屏上第 1 行 */

/* ★ 屏工程口径（"真实屏幕"的控件名）：6 行成功数量 = n1~n6（行序 = 货物类型 ID 1~6）。
   定义在 sim_plant.h；产品配置必须与之一致 —— 名字对不上时真机表现是"那一列永远不刷新"，
   任何自检都看不出来，所以这里钉死数值部分、sim_main 启动时比对名字（字符串没法编译期比） */
typedef char sim_plant_assert_tjc_first[(SIM_TJC_CNT_FIRST == CFG_TJC_CNT_FIRST) ? 1 : -1];
typedef char sim_plant_assert_tjc_rows[(CFG_NUM_CATEGORIES == 6) ? 1 : -1];

/* ==================== 前置声明 ==================== */
static void tjc_parse(const char *s);
static void rx_push(int id, uint8_t b);
static void servo_poll(void);
static void try_grab(void);
static void release_item(uint32_t claw_us);

/* ==================== 复位 ==================== */
void sim_plant_reset(void)
{
    memset(s_mot, 0, sizeof(s_mot));
    s_tray_phys = 0.0;
    s_tray_edges = 0;
    memset(s_rx, 0, sizeof(s_rx));
    for (int i = 0; i < 6; i++) {
        s_rx[i].us_per_byte = 87;    /* 115200 */
    }
    s_rx[2].us_per_byte = 1042;      /* UART4 备用口 @9600（原扫码口，现无设备） */

    g_plant.cam_deg     = 2.0;    /* 凸块压在开关上的角度宽度（与 CFG_TRAY_BACKOFF_PULSES 直接对撞） */
    g_plant.driver_ppr  = 3200;
    g_plant.belt_mm     = 40.0;
    /* 转盘减速比 = 皮带(30/16) × 减速箱(1:20) = 37.5 ⇒ 转盘一圈 3200×37.5 = 120000 电机脉冲。
       ★ 2026-09-30 与机械实况对齐（产品 CFG_TRAY_* 同口径）。这里**故意**独立于 sort_config.h：
       产品标定写错时，仿真必须真的转到一半、真的过不了索引复核，而不是自我印证——09-30
       产品改成 37.5、模型仍留 20 时，s1/s2/s3 当场变红正是这条机制在起作用 */
    g_plant.gear        = 37.5;
    g_plant.phys_sign_a = 1;
    g_plant.phys_sign_b = 1;
    g_plant.tray_phys_sign = 1;
    g_plant.tray_start_deg = 120.0;
    g_plant.home_x_mm   = 300.0;
    g_plant.home_y_mm   = 200.0;
    g_plant.drop_x      = 200.0f;
    g_plant.drop_y      = 340.0f;
    g_plant.vision_err_mm = 0.0;
    /* 视觉标定真值：视觉原点（标定中心）在机械坐标系里 = (60,45)，两个方向符号都为正
       —— 与 shim/sort_config.h 的 SIM_VISION_* 必须逐字一致（sim_main 启动时比较） */
    g_plant.vision_origin_x_mm = 60.0;
    g_plant.vision_origin_y_mm = 45.0;
    g_plant.vision_sign_vy_to_x = 1;
    g_plant.vision_sign_vx_to_y = 1;
    g_plant.verbose     = 0;

    s_x_mm = g_plant.home_x_mm;
    s_y_mm = g_plant.home_y_mm;
    s_lim_x = s_lim_y = s_tray_sw = 0;
    s_item_n = 0;
    s_held = -1;
    s_last_lift = 0xFFFFFFFFu;
    s_last_claw = 0xFFFFFFFFu;
    s_k230_next_us = 0;
    memset(s_acc_n, 0, sizeof(s_acc_n));
    g_zdt_ign_busy = 0;
    memset(g_zdt_frames, 0, sizeof(g_zdt_frames));
    g_zdt_pos_frames = g_zdt_stop_frames = g_zdt_speed_frames = g_zdt_enable_frames = 0;
    s_tjc_n = 0;
    s_tjc_acc_n = 0;
    s_tjc_ff = 0;
    strcpy(g_tjc_t0, "-");
    strcpy(g_tjc_p0, "-");
    for (int i = 0; i < 6; i++) {
        strcpy(g_tjc_cnt[i], "-");
    }
    htim4.ccr[0] = 0;
    htim4.ccr[1] = 0;
    htim4.ccr[2] = 0;
    htim4.ccr[3] = 0;
}

void sim_load_items(const sim_item_t *items, int n)
{
    if (n > SIM_MAX_ITEMS) {
        n = SIM_MAX_ITEMS;
    }
    s_item_n = n;
    for (int i = 0; i < n; i++) {
        s_item[i].spec = items[i];
        s_item[i].x = items[i].item_x_mm;
        s_item[i].y = items[i].item_y_mm;
        s_item[i].state = IT_TRAY;
        s_item[i].bin = -1;
    }
    s_held = -1;
}

int sim_item_count(void) { return s_item_n; }

int sim_items_remaining(void)
{
    int c = 0;

    for (int i = 0; i < s_item_n; i++) {
        if (s_item[i].state == IT_TRAY) {
            c++;
        }
    }
    return c;
}

/* ==================== 几何 ==================== */
static void solve_gantry(void)
{
    double a = s_mot[0].pos / (double)g_plant.driver_ppr * g_plant.belt_mm * g_plant.phys_sign_a;
    double b = s_mot[1].pos / (double)g_plant.driver_ppr * g_plant.belt_mm * g_plant.phys_sign_b;

    s_x_mm = g_plant.home_x_mm + (a + b) / 2.0;
    s_y_mm = g_plant.home_y_mm + (a - b) / 2.0;
}

double tray_angle_deg(void)
{
    double out_rev = s_tray_phys / ((double)g_plant.driver_ppr * g_plant.gear);

    return g_plant.tray_start_deg
           + (double)g_plant.tray_phys_sign * out_rev * 360.0;
}

float sim_tray_angle(void) { return (float)tray_angle_deg(); }

int sim_current_bin(void)
{
    double m = fmod(tray_angle_deg(), 360.0);

    if (m < 0) {
        m += 360.0;
    }
    return (int)((m + 30.0) / 60.0) % 6;
}

float sim_gantry_x(void) { return (float)s_x_mm; }
float sim_gantry_y(void) { return (float)s_y_mm; }
uint8_t sim_gripper_holding(void) { return (s_held >= 0) ? 1u : 0u; }
int sim_held_item(void) { return s_held; }
int sim_item_state(int i) { return s_item[i].state; }
int sim_item_bin(int i) { return s_item[i].bin; }
int sim_hard_stop_hits(void) { return g_res.hard_stop_hits; }
int sim_claw_us(void) { return (int)s_last_claw; }
int sim_lift_us(void) { return (int)s_last_lift; }
int sim_tjc_line_count(void) { return s_tjc_n; }
const char *sim_tjc_line(int i) { return (i < s_tjc_n) ? s_tjc_log[i] : ""; }
const char *sim_tjc_count(int type)
{
    if (type < 1 || type > (int)(sizeof(g_tjc_cnt) / sizeof(g_tjc_cnt[0]))) {
        return "?";
    }
    return g_tjc_cnt[type - 1];
}

int sim_motor_state(int idx, float *pos_mm)
{
    if (idx < 0 || idx > 2) {
        return -1;
    }
    *pos_mm = (float)(s_mot[idx].pos / (double)g_plant.driver_ppr);
    return 0;
}

void sim_dump_screen(void)
{
    printf("  屏: 每类成功数 n1..n6 = %s %s %s %s %s %s（另 t0=%s p0=%s）\n",
           g_tjc_cnt[0], g_tjc_cnt[1], g_tjc_cnt[2], g_tjc_cnt[3], g_tjc_cnt[4], g_tjc_cnt[5],
           g_tjc_t0, g_tjc_p0);
}

/* ==================== 微动开关 ==================== */
static void update_switches(void)
{
    /* ★ 2026-09-28 与机械实况对齐：X/Y 限位开关装在**最大行程端**（不在原点端），
       越过触发点即压下（s_x_mm >= 触发值）；触发值 = 工作坐标系的
       (CFG_HOME_X_MAX_MM, CFG_HOME_Y_MAX_MM)，产品回零朝 +X/+Y 找开关 */
    int nx = (s_x_mm >= g_sim.limit_trigger_x_mm);
    int ny = (s_y_mm >= g_sim.limit_trigger_y_mm);
    double m = fmod(tray_angle_deg(), 360.0);
    int cams = (g_sim.tray_cam_count >= 1) ? g_sim.tray_cam_count : 1;
    double period = 360.0 / (double)cams;
    int nt;

    if (m < 0) {
        m += 360.0;
    }
    nt = (fmod(m, period) < g_plant.cam_deg);

    if (g_sim.limit_x_stuck) { nx = 1; }
    if (g_sim.limit_x_dead)  { nx = 0; }
    if (g_sim.limit_y_stuck) { ny = 1; }
    if (g_sim.limit_y_dead)  { ny = 0; }
    if (g_sim.tray_sw_stuck) { nt = 1; }
    if (g_sim.tray_sw_dead)  { nt = 0; }

    if (nx && !s_lim_x) {
        sim_raise_exti(LIM_X_PIN);
    }
    if (ny && !s_lim_y) {
        sim_raise_exti(LIM_Y_PIN);
    }
    if (nt && !s_tray_sw) {
        if (g_plant.verbose) {
            double phys_pulses = s_tray_phys;
            double cmd_pulses = s_mot[2].pos;

            printf("  [%6.2fs][盘沿] 第%u个索引上升沿 账面%ld脉冲 物理角%.1f度"
                   " | 实物%ld脉冲 指令%ld脉冲 差%ld\n",
                   (double)sim_now_us_get() / 1e6, ++s_tray_edges, (long)s_mot[2].pos,
                   tray_angle_deg(), (long)phys_pulses, (long)cmd_pulses,
                   (long)(cmd_pulses - phys_pulses));
        }
        sim_raise_exti(TRAY_PIN);
    }
    /* s_lim_x/s_lim_y/s_tray_sw 保存的是"开关是否压下"这个物理事实，不是引脚电平 ——
       引脚电平在 sim_plant_pin_read 里按常开+上拉（压下=低）换算 */
    s_lim_x = nx;
    s_lim_y = ny;
    s_tray_sw = nt;
}

int sim_plant_pin_read(void *port, uint16_t pin)
{
    /* 常开 + 上拉：未压下=高，压下=低。产品侧 CFG_LIMIT_ACTIVE_LEVEL=RESET 与
       EXTI 下降沿就是按这个接法写的；模型若在这里返回"压下=高"，
       接法一改就测不出极性错误，等于自我印证 */
    if (port == (void *)&sim_GPIOB && pin == LIM_X_PIN) {
        return s_lim_x ? 0 : 1;
    }
    if (port == (void *)&sim_GPIOC && pin == LIM_Y_PIN) {
        return s_lim_y ? 0 : 1;
    }
    if (port == (void *)&sim_GPIOB && pin == TRAY_PIN) {
        return s_tray_sw ? 0 : 1;
    }
    return 0;
}

/* ==================== ZDT 指令 ==================== */
static void zdt_apply(int mid, const uint8_t *f, int n)
{
    motor_t *m = &s_mot[mid];
    double sign, spd;
    uint32_t pulse;
    uint16_t rpm;

    g_zdt_frames[mid]++;
    if (g_plant.verbose) {
        printf("  [%7.2fs][ZDT%d] func=%02X dir=%02X rpm=%u pulse=%u -> pos %.0f\n",
               (double)sim_now_us_get() / 1e6, mid, f[1],
               (n >= 3) ? f[2] : 0u,
               (n >= 5) ? (unsigned)(((uint16_t)f[3] << 8) | f[4]) : 0u,
               (n >= 10) ? (unsigned)(((uint32_t)f[6] << 24) | ((uint32_t)f[7] << 16) |
                                      ((uint32_t)f[8] << 8) | f[9]) : 0u,
               m->pos);
    }
    switch (f[1]) {
    case 0xF3u:
        g_zdt_enable_frames++;
        break;
    case 0xFDu:
        if (m->moving && g_sim.zdt_busy_rejects_fd) {
            g_zdt_ign_busy++;
            return;              /* 语义未标定：这台驱动器可能把新目标当"排队"或直接丢 */
        }
        sign = (f[2] == 0x00u) ? 1.0 : -1.0;   /* 00=CW */
        rpm  = (uint16_t)(((uint16_t)f[3] << 8) | f[4]);
        pulse = ((uint32_t)f[6] << 24) | ((uint32_t)f[7] << 16) | ((uint32_t)f[8] << 8) | f[9];
        spd = (double)rpm / 60.0 * (double)g_plant.driver_ppr;
        if (spd <= 0.0) {
            printf("  [ZDT] 速度为 0（RPM=%u）电机不会动\n", rpm);
            break;
        }
        m->vel_mode = 0;
        m->target = m->pos + sign * (double)pulse;
        m->vel = (m->target >= m->pos) ? spd : -spd;
        m->moving = 1;
        g_zdt_pos_frames++;
        (void)n;
        break;
    case 0xF6u:
        sign = (f[2] == 0x00u) ? 1.0 : -1.0;
        rpm  = (uint16_t)(((uint16_t)f[3] << 8) | f[4]);
        m->vel_mode = 1;
        m->vel = sign * (double)rpm / 60.0 * (double)g_plant.driver_ppr;
        m->moving = 1;
        g_zdt_speed_frames++;
        (void)n;
        break;
    case 0xFEu:
        m->moving = 0;
        m->vel = 0.0;
        m->target = m->pos;
        g_zdt_stop_frames++;
        break;
    default:
        printf("  [ZDT] 未知功能码 %02X\n", f[1]);
        break;
    }
}

/* 定长协议的正确收帧方式：按功能码定长，0x6B 出现在脉冲区里也不会拆错 */
static int zdt_frame_len(uint8_t func)
{
    switch (func) {
    case 0xFDu: return 13; /* 脉冲后还有【运动模式】+【多机同步】两字节（Emm_V5.0 手册） */
    case 0xF6u: return 8;
    case 0xF3u: return 6;
    case 0xFEu: return 5;
    default:    return 0;
    }
}

static void zdt_drop1(int mid)
{
    for (int i = 0; i + 1 < s_acc_n[mid]; i++) {
        s_acc[mid][i] = s_acc[mid][i + 1];
    }
    s_acc_n[mid]--;
}

static void zdt_feed(int mid, uint8_t b)
{
    if (s_acc_n[mid] < 16) {
        s_acc[mid][s_acc_n[mid]++] = b;
    }
    while (s_acc_n[mid] > 0) {
        int len;

        if (s_acc[mid][0] != 0x00u) {
            zdt_drop1(mid);          /* 不在地址上：丢一字节重找帧头 */
            continue;
        }
        if (s_acc_n[mid] < 2) {
            return;
        }
        len = zdt_frame_len(s_acc[mid][1]);
        if (len == 0) {
            zdt_drop1(mid);
            continue;
        }
        if (s_acc_n[mid] < len) {
            return;
        }
        if (s_acc[mid][len - 1] != 0x6Bu) {
            zdt_drop1(mid);          /* 帧尾不对：当噪声处理 */
            continue;
        }
        zdt_apply(mid, s_acc[mid], len);
        for (int i = len; i < s_acc_n[mid]; i++) {
            s_acc[mid][i - len] = s_acc[mid][i];
        }
        s_acc_n[mid] -= len;
    }
}

void sim_plant_uart_txd(int uart_id, const uint8_t *data, uint16_t len)
{
    int mid = -1;

    if (uart_id == 3) {
        mid = 0;
    } else if (uart_id == 4) {
        mid = 1;
    } else if (uart_id == 5) {
        mid = 2;
    }

    if (uart_id == 0) {  /* 串口屏 */
        if (g_sim.verbose != 0u) {
            /* verbose 模式下把屏口原样打到标准输出，便于逐指令核对 */
            fwrite(data, 1u, len, stdout);
            fflush(stdout);
        }
        for (int i = 0; i < len; i++) {
            uint8_t b = data[i];

            if (b == 0xFFu) {
                s_tjc_ff++;
                if (s_tjc_ff >= 3) {
                    if (s_tjc_acc_n > 0) {
                        s_tjc_acc[s_tjc_acc_n] = '\0';
                        if (s_tjc_n < TJC_LOG) {
                            snprintf(s_tjc_log[s_tjc_n], sizeof(s_tjc_log[0]), "%s",
                                     (char *)s_tjc_acc);
                            s_tjc_n++;
                        }
                        tjc_parse((const char *)s_tjc_acc);
                        s_tjc_acc_n = 0;
                    }
                    s_tjc_ff = 0;
                }
                continue;
            }
            s_tjc_ff = 0;
            if (s_tjc_acc_n < 120) {
                s_tjc_acc[s_tjc_acc_n++] = b;
            }
        }
        return;
    }
    /* uart_id == 2（UART4 / SPARE）：原条码模块口，现无设备，发出的字节无人接收 */
    if (mid < 0) {
        return;
    }
    for (int i = 0; i < len; i++) {
        zdt_feed(mid, data[i]);
    }
}

/* TJC 文本指令的值带引号（t1.txt="1"）：取引号内的内容，
   否则上层拿 "-" 占位判空永远对不上 */
static const char *tjc_value(const char *eq)
{
    static char buf[64];
    size_t k = 0;

    if (*eq != '"') {
        return eq;
    }
    eq++;
    while (*eq != '\0' && *eq != '"' && k < sizeof(buf) - 1u) {
        buf[k++] = *eq++;
    }
    buf[k] = '\0';
    return buf;
}

static void tjc_parse(const char *s)
{
    const char *eq = strchr(s, '=');
    size_t      blen;

    if (eq == NULL) {
        return;
    }
    eq = tjc_value(eq + 1);
    if (strncmp(s, "t0.", 3) == 0) {
        snprintf(g_tjc_t0, sizeof(g_tjc_t0), "%s", eq);
    } else if (strncmp(s, "p0.", 3) == 0) {
        snprintf(g_tjc_p0, sizeof(g_tjc_p0), "%s", eq);
    } else if (strncmp(s, SIM_TJC_CNT_BASE, strlen(SIM_TJC_CNT_BASE)) == 0) {
        /* 6 行"成功数量"：按**屏工程**的名字（SIM_TJC_*）解析，不照抄产品实现的拼名逻辑——
           两边的名字若漂移，这里会解析不到，屏上计数保持 "-"，S1 的计数断言当场失败 */
        blen = strlen(SIM_TJC_CNT_BASE);
        if (s[blen] >= '0' && s[blen] <= '9') {
            int idx = atoi(s + blen) - SIM_TJC_CNT_FIRST;

            if (idx >= 0 && idx < (int)(sizeof(g_tjc_cnt) / sizeof(g_tjc_cnt[0]))) {
                snprintf(g_tjc_cnt[idx], sizeof(g_tjc_cnt[0]), "%s", eq);
                g_res.screen_updates++;
            }
        }
    }
}

/* ==================== 执行器步进 ==================== */
static void motor_step(motor_t *m, int mid, double dt_s)
{
    double st;

    if (!m->moving || m->vel == 0.0) {
        return;
    }
    if (g_sim.motor_block[mid]) {
        return;   /* 堵转：指令照收、轴不动 */
    }
    st = m->vel * dt_s;
    if (!m->vel_mode) {
        if ((m->vel > 0.0 && m->pos + st >= m->target) ||
            (m->vel < 0.0 && m->pos + st <= m->target)) {
            m->pos = m->target;
            m->moving = 0;
            return;
        }
    }
    m->pos += st;
}

/* ==================== 抓取 / 放件 ==================== */
static void try_grab(void)
{
    int best = -1;
    double bd = 1e18;

    if (s_held >= 0) {
        return;   /* 手上已有货：再闭一次不算第二次抓取 */
    }
    for (int i = 0; i < s_item_n; i++) {
        double d;

        if (s_item[i].state != IT_TRAY) {
            continue;
        }
        d = fabs(s_item[i].x - s_x_mm) + fabs(s_item[i].y - s_y_mm);
        if (d < bd) {
            bd = d;
            best = i;
        }
    }
    if (best >= 0 && bd <= s_item[best].spec.grab_radius_mm * 2.0) {
        s_item[best].state = IT_HELD;
        s_held = best;
        if (g_plant.verbose) {
            printf("  [%6.2fs][抓] #%d 成功 位置偏差 %.1fmm\n",
                   (double)sim_now_us_get() / 1e6, best, bd);
        }
    } else {
        g_res.items_empty_grab++;
        if (g_plant.verbose) {
            printf("  [%6.2fs][抓] 抓空！最近货物偏差 %.1fmm\n",
                   (double)sim_now_us_get() / 1e6, bd);
        }
    }
}

static void release_item(uint32_t claw_us)
{
    int i = s_held;

    s_held = -1;
    if (i < 0) {
        return;   /* 空爪张开 = 投放前的复位，不算落件 */
    }
    s_item[i].x = s_x_mm;
    s_item[i].y = s_y_mm;

    if (claw_us == US_CLAW_CLOSE) {
        return;   /* 不会发生：只处理离开闭合位的两档 */
    }
    {
        int to_drop = (claw_us == US_CLAW_OPEN) ? 1 : 0;   /* 全开=投放，半开=轻放 */
        double cx = g_plant.drop_x;
        double cy = g_plant.drop_y;
        double dx = s_x_mm - cx;
        double dy = s_y_mm - cy;
        int in_envelope = (s_x_mm >= 5.0 && s_x_mm <= 385.0 &&
                           s_y_mm >= 5.0 && s_y_mm <= 385.0);

        if (g_plant.verbose) {
            if (to_drop) {
                printf("  [%6.2fs][松爪] #%d 全开 物理(%.1f,%.1f) 投放点(%.0f,%.0f) 距离%.1f -> %s\n",
                       (double)sim_now_us_get() / 1e6, i, s_x_mm, s_y_mm, cx, cy,
                       sqrt(dx * dx + dy * dy),
                       (sqrt(dx * dx + dy * dy) <= 25.0) ? "入盒" : "掉在外面");
            } else {
                printf("  [%6.2fs][松爪] #%d 半开 物理(%.1f,%.1f) -> %s\n",
                       (double)sim_now_us_get() / 1e6, i, s_x_mm, s_y_mm,
                       in_envelope ? "就地轻放回料面" : "行程外掉落");
            }
        }

        if (to_drop) {
            if (sqrt(dx * dx + dy * dy) <= 25.0) {
                s_item[i].state = IT_BIN;
                s_item[i].bin = sim_current_bin();
                g_res.items_done++;
            } else {
                s_item[i].state = IT_LOST;
                s_item[i].bin = -1;
                g_res.items_dropped_outside++;
                printf("  [%6.2fs][落] #%d 松爪位置不对 (%.0f,%.0f) 期望投放点 (%.0f,%.0f)\n",
                       (double)sim_now_us_get() / 1e6, i, s_x_mm, s_y_mm, cx, cy);
            }
        } else if (in_envelope) {
            s_item[i].state = IT_TRAY;          /* 作废就地轻放：货回到料面，K230 会重新报 */
        } else {
            s_item[i].state = IT_LOST;
            s_item[i].bin = -1;
            g_res.items_dropped_outside++;
            printf("  [%6.2fs][落] #%d 轻放在了行程外 (%.0f,%.0f)\n",
                   (double)sim_now_us_get() / 1e6, i, s_x_mm, s_y_mm);
        }
    }
}

static void servo_poll(void)
{
    /* 硬件通道：升降=TIM4_CH3、夹爪=TIM4_CH4（原理图 PB8/PB9）。
       模型按 CCR 下标取值，与产品代码用哪个通道无关，所以这里用下标 2/3 */
    uint32_t lf = htim4.ccr[2];
    uint32_t cl = htim4.ccr[3];

    if (lf != s_last_lift) {
        s_last_lift = lf;
    }
    if (cl != s_last_claw) {
        uint32_t prev = s_last_claw;

        s_last_claw = cl;
        if (prev == 0xFFFFFFFFu) {
            return;               /* 上电安全位，不做判定 */
        }
        if (cl == US_CLAW_CLOSE && prev != US_CLAW_CLOSE) {
            if (s_last_lift == US_LIFT_DOWN) {
                try_grab();
            } else if (s_held >= 0) {
                g_res.items_empty_grab++;
                printf("  [%6.2fs][抓] 升降未下伸就闭合且手上有货 -> 货被夹飞风险\n",
                       (double)sim_now_us_get() / 1e6);
            }
            /* 抬升位闭合 = 投放后复位空爪，物理上夹不到东西，不作判定 */
        } else if (prev == US_CLAW_CLOSE && cl != US_CLAW_CLOSE) {
            release_item(cl);
        }
    }
}

void sim_plant_servo_poll(void) { servo_poll(); }

/* ==================== 每子步 ==================== */
void sim_plant_step_us(uint32_t dt_us)
{
    double dt_s = (double)dt_us / 1e6;
    double old_a = s_mot[0].pos;
    double old_b = s_mot[1].pos;
    double old_t = s_mot[2].pos;

    for (int i = 0; i < 3; i++) {
        motor_step(&s_mot[i], i, dt_s);
    }
    {
        double slip = 1.0 - (double)g_sim.tray_slip_pct / 100.0;

        s_tray_phys += (s_mot[2].pos - old_t) * slip;
    }
    solve_gantry();

    /* 机械硬限位：真值不能越过**正向端**（09-28 起开关在最大端），越过则本子步作废（顶死） */
    if (s_x_mm > g_sim.hard_stop_x_mm || s_y_mm > g_sim.hard_stop_y_mm) {
        static uint64_t last_report;

        s_mot[0].pos = old_a;
        s_mot[1].pos = old_b;
        solve_gantry();
        if (sim_now_us_get() - last_report > 500000u) {
            last_report = sim_now_us_get();
            g_res.hard_stop_hits++;
            printf("  [%6.2fs][撞] 龙门顶到机械硬限位 (%.1f,%.1f)\n",
                   (double)sim_now_us_get() / 1e6, s_x_mm, s_y_mm);
        }
    }
    if (s_x_mm < g_res.min_x_mm) { g_res.min_x_mm = (float)s_x_mm; }
    if (s_y_mm < g_res.min_y_mm) { g_res.min_y_mm = (float)s_y_mm; }
    if (s_x_mm > g_res.max_x_mm) { g_res.max_x_mm = (float)s_x_mm; }
    if (s_y_mm > g_res.max_y_mm) { g_res.max_y_mm = (float)s_y_mm; }

    update_switches();
    servo_poll();

    if (s_held >= 0) {
        s_item[s_held].x = s_x_mm;
        s_item[s_held].y = s_y_mm;
    }

    /* ---------- K230 每 100ms 一帧 ---------- */
    if (!g_sim.k230_link_down && sim_now_us_get() >= s_k230_next_us) {
        int t = -1;
        uint8_t f[12];
        uint8_t sum = 0;
        int16_t x10 = 0, y10 = 0;

        for (int i = 0; i < s_item_n; i++) {
            if (s_item[i].state == IT_TRAY) {
                t = i;
                break;
            }
        }
        f[0] = 0xAAu;
        f[1] = 0x55u;
        f[2] = (uint8_t)((t >= 0) ? 1 : 0);
        if (t >= 0) {
            /* 实物机械坐标（+ 视觉系统误差）-> 按视觉标定**反算**成视觉帧坐标。
               反算是故意的：产品侧 Vision_ToMachine 若写成纯平移、或符号/交换搞错，
               仿真是真的抓空、真的放错位，而不是自我印证 */
            double x_mm = s_item[t].x + g_plant.vision_err_mm;
            double y_mm = s_item[t].y + g_plant.vision_err_mm;
            double vx, vy;

            if (g_sim.k230_out_of_envelope) {
                x_mm = 9000.0;
                y_mm = 9000.0;
            }
            vy = (x_mm - g_plant.vision_origin_x_mm) / (double)g_plant.vision_sign_vy_to_x;
            vx = (y_mm - g_plant.vision_origin_y_mm) / (double)g_plant.vision_sign_vx_to_y;
            x10 = (int16_t)lround(vx * 10.0); /* 帧里的 X = 视觉 x（视觉系 x+ = 机械 Y+） */
            y10 = (int16_t)lround(vy * 10.0);
            f[3] = s_item[t].spec.category; /* 类别 ID 1~6（原"颜色"字节） */
            f[4] = 0x00u;                   /* 原"形状"字节：2026-09-30 起协议固定发 0 */
            f[5] = s_item[t].spec.has_qr;
            f[6] = 0x00u;
        } else {
            memset(&f[3], 0, 8);
        }
        f[7]  = (uint8_t)(x10 & 0xFF);
        f[8]  = (uint8_t)(((uint16_t)x10 >> 8) & 0xFF);
        f[9]  = (uint8_t)(y10 & 0xFF);
        f[10] = (uint8_t)(((uint16_t)y10 >> 8) & 0xFF);
        for (int i = 0; i < 11; i++) {
            sum = (uint8_t)(sum + f[i]);
        }
        f[11] = sum;
        for (int i = 0; i < 12; i++) {
            rx_push(1, f[i]);
        }
        s_k230_next_us = sim_now_us_get() + 100000u;
    }

    /* ---------- RX 逐字节节拍 ---------- */
    for (int i = 0; i < 6; i++) {
        rxlink_t *l = &s_rx[i];

        if (l->head == l->tail) {
            l->next_us = 0;
            continue;
        }
        if (l->next_us == 0) {
            l->next_us = sim_now_us_get() + (uint64_t)l->us_per_byte;
        }
        if (sim_now_us_get() >= l->next_us) {
            sim_uart_byte_arrives(i);
            l->next_us += (uint64_t)l->us_per_byte;
        }
    }
}

/* ==================== 产品代码 <-> 模型 ==================== */
static void rx_push(int id, uint8_t b)
{
    rxlink_t *l = &s_rx[id];
    int next = (l->tail + 1) % 512;

    if (next == l->head) {
        printf("  [模型] RX FIFO 溢出 id=%d（真机会 ORE）\n", id);
        return;
    }
    l->q[l->tail] = b;
    l->tail = next;
}

int sim_plant_uart_ready_rx(int uart_id)
{
    rxlink_t *l = &s_rx[uart_id];

    return (l->head != l->tail) ? 1 : 0;
}

void sim_plant_uart_deliver_rx(int uart_id)
{
    rxlink_t *l = &s_rx[uart_id];
    uint8_t b = l->q[l->head];
    UART_HandleTypeDef *h = sim_uart_handle(uart_id);

    l->head = (l->head + 1) % 512;
    if (h != NULL) {
        if (!sim_uart_rx_attach(h, b)) {
            HAL_UART_ErrorCallback(h);
        }
    }
}
