/**
 ******************************************************************************
 * @file    sim_main.c
 * @brief   主机侧仿真台入口：场景定义 + 上电时序复刻 + 结果判定
 * @attention
 * 上电时序与 Core/Src/main.c 严格一致：Bsp_Uart_Init -> Servo_Init -> Zdt_Init
 * -> 建队列 -> 建 4 个任务（优先级 守护>视觉>显示>分拣）。
 * 一个进程只跑一个场景：产品代码里的 static 状态（记账、可信标志、解析器）
 * 没有复位接口，换场景必须换进程，否则会拿到假结果。
 *
 * 用法： ./sim <scenario> [v]     v = 打印逐步动作轨迹
 ******************************************************************************
 */
#include "sim_shim.h"
#include "sim_plant.h"
#include "sim_kernel.h"

#include "main.h"
#include "sort_config.h"
#include "bsp_uart.h"
#include "servo.h"
#include "zdt_motor.h"
#include "motion_app.h"
#include "sort_task.h"
#include "vision_task.h"
#include "display_task.h"

#include <math.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern int sim_error_raised;
extern long g_zdt_ign_busy;

/* ==================== 场景数据 ==================== */
static sim_item_t s_items[SIM_MAX_ITEMS];
static int s_n_items;

#define MAX_EVT 16
typedef enum
{
    EV_K230_LINK,      /* val: 1=断链 0=恢复 */
    EV_TRAY_SLIP,      /* val: 打滑百分比 */
    EV_MOTOR_BLOCK,    /* val: 电机号 0/1/2，-1=解除 */
    EV_SERIAL_DEAD,    /* val: 串口号 0~5，-1=解除全部 */
    EV_LIMIT_X,        /* val: 0=正常 1=常闭 2=断线 */
    EV_LIMIT_GLITCH,   /* val: 0=X 1=Y —— 纯电气毛刺：只灌一次"下降沿"，开关电平一动不动 */
    EV_VISION_ERR,     /* val: 视觉系统误差 mm（0=恢复正常；> 抓取半径 12mm×2 时必抓空，见 s24） */
    EV_RELOAD,         /* val: 立即追加几件货（运行中补货：托盘没空就插——s27 的口径外代价） */
    EV_RELOAD_EMPTY,   /* val: 追加几件货，且**等托盘空了、机器自己判"空盘"之后 2s** 才放
                          （= 空盘静默窗内的补货，用户口径的做法，见 s26）；at_ms = 最早时刻 */
    EV_RELOAD_SPREAD,  /* val: 追加几件货，**逐件放**：空盘判据成立后 1.0s 放第 1 件，
                          之后每 1.0s 一件（全落在 5s 静默窗内）——操作员真实的"边放边补"，
                          见 s29；at_ms = 最早时刻 */
    EV_STOP            /* val: 无 —— 到这里就可以收工 */
} evt_what_e;

typedef struct
{
    uint32_t at_ms;
    evt_what_e what;
    int val;
} evt_t;

static evt_t s_evt[MAX_EVT];
static int s_n_evt;
static int s_evt_done;
static int s_stop_requested;

/* 白盒采样：运行中记账可信标志的下降沿 */
static uint8_t s_last_xy_trust, s_last_tray_trust;
static uint32_t s_last_beat_ms;

/* has_qr 保留：K230 帧的 byte[5] 照旧发出，只是产品侧不再使用它。
   类别 ID 1~6 = "颜色字节"；形状 1~3 = "形状字节"（2026-10-01 起启用：不参与分类，只选夹持方案） */
static void add_item(float x, float y, uint8_t category, uint8_t shape, uint8_t has_qr)
{
    if (s_n_items >= SIM_MAX_ITEMS) {
        printf("sim: 货物数量超上限\n");
        exit(2);
    }
    memset(&s_items[s_n_items], 0, sizeof(sim_item_t));
    s_items[s_n_items].item_x_mm = x;
    s_items[s_n_items].item_y_mm = y;
    s_items[s_n_items].category = category;
    s_items[s_n_items].shape = shape;
    s_items[s_n_items].has_qr = has_qr;
    s_items[s_n_items].grab_radius_mm = 12.0f;
    s_n_items++;
}

/* 料盘上的 5 列 x 若干行栅格，全部落在工作包络内且互相不重叠。
   货物用 6 类各若干件（类别 ID 1~6 = 屏上 6 行、盒号表的 6 个下标）+ 形状 0~3 轮着放
   （0=默认方案、1~3=特调立方体/12面体/六棱柱；形状只选夹持方案、不影响进盒——
   回归靠"方案-形状匹配"检查覆盖形状路由：传错形状 = 抓取滑脱 = FAIL） */
static void load_default_lot(int n)
{
    for (int i = 0; i < n; i++) {
        float x = -125.0f + 28.0f * (float)(i % 5);
        float y = -170.0f + 55.0f * (float)(i / 5);
        uint8_t cat = (uint8_t)(i % 6 + 1);              /* 1..6 轮着放 */
        uint8_t shape = (uint8_t)(i % 4);            /* 0~3 轮着放（0=默认方案 + 三种特调都覆盖到） */
        uint8_t use_qr = (uint8_t)((i % 2) == 0);        /* 帧里照发 hasQR 字节，产品不用 */

        add_item(x, y, cat, shape, use_qr);
    }
}

static void evt_add(uint32_t at_ms, evt_what_e what, int val)
{
    if (s_n_evt >= MAX_EVT) {
        printf("sim: 事件数超上限\n");
        exit(2);
    }
    s_evt[s_n_evt].at_ms = at_ms;
    s_evt[s_n_evt].what = what;
    s_evt[s_n_evt].val = val;
    s_n_evt++;
}

/* ==================== 复位 ==================== */
void sim_reset(void)
{
    memset(&g_sim, 0, sizeof(g_sim));
    memset(&g_res, 0, sizeof(g_res));
    memset(s_evt, 0, sizeof(s_evt));
    s_n_evt = 0;
    s_evt_done = 0;
    s_stop_requested = 0;
    s_n_items = 0;

    g_res.min_x_mm = 1e9f;
    g_res.min_y_mm = 1e9f;
    for (int i = 0; i < SIM_MAX_ITEMS; i++) {
        g_res.bin_of[i] = -1;
    }

    /* ★ 限位触发点 = 开关压下处，在**最大行程端**（09-28 与机械实况对齐；
       此前 0,0 是"开关在原点端"的旧模型，与 P0-6 修正前的产品同源同错）。
       触发值 = 工作坐标系的 (CFG_HOME_X_MAX_MM, CFG_HOME_Y_MAX_MM)；产品回零朝 + 找开关 */
    g_sim.limit_trigger_x_mm = CFG_HOME_X_MAX_MM;
    g_sim.limit_trigger_y_mm = CFG_HOME_Y_MAX_MM;
    g_sim.hard_stop_x_mm = CFG_HOME_X_MAX_MM + 25.0f;   /* 越过触发点后还能顶进去的机械余量 */
    g_sim.hard_stop_y_mm = CFG_HOME_Y_MAX_MM + 25.0f;
    g_sim.tray_cam_count = 1;

    sim_plant_reset();
}

/* ==================== 上电 + 调度 ==================== */
static void boot_product(void)
{
    Bsp_Uart_Init();
    Servo_Init();
    Zdt_Init();
    DisplayMgr_CreateObjects();

    sim_task_create(Motion_SafetyTask, NULL, "safety", 512u, 4);
    sim_task_create(VisionTask, NULL, "vision", 1024u, 3);
    sim_task_create(DisplayTask, NULL, "display", 1024u, 2);
    sim_task_create(SortTask, NULL, "sort", 2048u, 1);
}

/* ★ 人工再放货（2026-10-10 数量复核的补货场景）：位置取默认料盘**空着的**格子
   （第 3 行 y=-60 的后 3 列——12 件的默认批次只占到 (i=10,11) 两格），追加到 s_items 与
   物理模型两侧；类别/形状取 1~3（顺带覆盖特调夹持方案）。数量位由模型按"托盘还剩几件"
   自动跟着涨——这正是"运行中补货"会被数量复核看见的原因 */
/* 放一件"补货件"（第 idx 件）：位置取默认料盘**空着的**格子（第 3 行 y=-60 的后 3 列——
   12 件的默认批次只占到 (i=10,11) 两格），类别/形状 1~3（顺带覆盖特调夹持方案） */
static void reload_one(int idx)
{
    static const float xs[3] = {-69.0f, -41.0f, -13.0f};

    if (s_n_items >= SIM_MAX_ITEMS) {
        printf("sim: 补货超过货物上限\n");
        exit(2);
    }
    memset(&s_items[s_n_items], 0, sizeof(sim_item_t));
    s_items[s_n_items].item_x_mm = xs[idx % 3];
    s_items[s_n_items].item_y_mm = -60.0f;
    s_items[s_n_items].category = (uint8_t)(1 + (idx % 3));
    s_items[s_n_items].shape = (uint8_t)(1 + (idx % 3));
    s_items[s_n_items].has_qr = 0u;
    s_items[s_n_items].grab_radius_mm = 12.0f;
    s_n_items++;
    (void)sim_plant_add_items(&s_items[s_n_items - 1], 1);
}

static void reload_lot(int n)
{
    for (int i = 0; i < n; i++) {
        reload_one(i);
    }
}

static void apply_event(const evt_t *e)
{
    switch (e->what) {
    case EV_K230_LINK:
        g_sim.k230_link_down = (uint8_t)(e->val != 0);
        printf("\n>>> [注入] t=%ums K230 链路%s\n\n", e->at_ms,
               e->val ? "断开（不再发帧）" : "恢复");
        break;
    case EV_TRAY_SLIP:
        g_sim.tray_slip_pct = (uint8_t)e->val;
        printf("\n>>> [注入] t=%ums 转盘打滑 %d%%\n\n", e->at_ms, e->val);
        break;
    case EV_MOTOR_BLOCK:
        if (e->val < 0) {
            memset(g_sim.motor_block, 0, sizeof(g_sim.motor_block));
        } else {
            g_sim.motor_block[e->val] = 1u;
        }
        printf("\n>>> [注入] t=%ums 电机 %d 堵转\n\n", e->at_ms, e->val);
        break;
    case EV_SERIAL_DEAD:
        if (e->val < 0) {
            memset(g_sim.serial_dead, 0, sizeof(g_sim.serial_dead));
        } else {
            g_sim.serial_dead[e->val] = 1u;
        }
        printf("\n>>> [注入] t=%ums 串口 %d %s\n\n", e->at_ms, e->val,
               e->val >= 0 ? "发送失败" : "恢复");
        break;
    case EV_LIMIT_X:
        g_sim.limit_x_stuck = (uint8_t)(e->val == 1);
        g_sim.limit_x_dead = (uint8_t)(e->val == 2);
        printf("\n>>> [注入] t=%ums X 限位 -> %s\n\n", e->at_ms,
               e->val == 1 ? "常闭（永远触发）" : (e->val == 2 ? "断线（永远不触发）" : "正常"));
        break;
    case EV_LIMIT_GLITCH:
        /* 现场机制（2026-10-08 用户报"抓完之后有时自己回零校准"）：舵机/电机换向的电流
           尖峰耦合进限位线，EXTI 收到一次下降沿，但开关**机械上没被碰**（电平一动不动）。
           这里就是那个"只灌沿、不动电平"的注入——与真撞（update_switches 里电平跟着变）
           构成对照，见 s23。sim_raise_exti 走的就是真沿那条路（排队 → HAL_GPIO_EXTI_Callback） */
        sim_raise_exti((e->val == 0) ? CFG_LIMIT_PIN_X : CFG_LIMIT_PIN_Y);
        printf("\n>>> [注入] t=%ums %c 限位**电气毛刺**（只灌一次沿，开关电平不动）\n\n", e->at_ms,
               (e->val == 0) ? 'X' : 'Y');
        break;
    case EV_RELOAD:
        printf("\n>>> [注入] t=%ums 人工再放入 %d 件货（**运行中**：托盘还有货，口径之外）\n\n",
               e->at_ms, e->val);
        reload_lot(e->val);
        break;
    case EV_RELOAD_EMPTY:
        printf("\n>>> [注入] t=%ums 空盘静默窗内补货 %d 件（等托盘空 + 空盘判据成立 2s 才放）\n\n",
               e->at_ms, e->val);
        reload_lot(e->val);
        break;
    case EV_RELOAD_SPREAD:
        /* 逐件放由 run_budget 驱动（每片放一件）；这里只报"第一件已放" */
        printf("\n>>> [注入] t=%ums 空盘静默窗内**逐件**补货 %d 件（空盘判据成立后 1s 起，每 1s 一件）\n\n",
               e->at_ms, e->val);
        break;
    case EV_VISION_ERR:
        /* 视觉系统误差（标定没做好/坐标整体偏移）：**此后发出的每一帧**坐标整体偏 N mm，
           物块还在原来的地方 ⇒ 龙门按偏坐标去抓 = 真抓空（不去动 K230 的目标挑选逻辑） */
        g_plant.vision_err_mm = (double)e->val;
        printf("\n>>> [注入] t=%ums 视觉系统误差 = %dmm（0=恢复正常）\n\n", e->at_ms, e->val);
        break;
    case EV_STOP:
        s_stop_requested = 1;
        break;
    default:
        break;
    }
}

static void sample(void)
{
    uint8_t xt = Motion_XyIsTrusted();
    uint8_t tt = Motion_TrayIsTrusted();

    if (s_last_xy_trust && !xt) {
        g_res.xy_untrust_events++;
    }
    if (s_last_tray_trust && !tt) {
        g_res.tray_untrust_events++;
    }
    s_last_xy_trust = xt;
    s_last_tray_trust = tt;

    if (g_sim.verbose && (sim_now_ms() - s_last_beat_ms) >= 1000u) {
        s_last_beat_ms = sim_now_ms();
        printf("  [%6.2fs][心跳] xy_trust=%u tray_trust=%u link=%u empty=%u\n",
               (double)sim_now_ms() / 1000.0, xt, tt,
               Vision_LinkAlive(), Vision_TrayEmpty());
    }
}

static void run_budget(uint32_t budget_ms, uint32_t expect_done)
{
    const uint32_t slice = 50u;
    const int      base_items = s_n_items; /* 场景初始件数（补货前）——"最后一件落盒"的判据 */
    uint32_t last_drop_ms = 0u; /* 初始那批**全部落盒**（= items_done 到齐）的时刻 */
    int      spread_placed = 0; /* EV_RELOAD_SPREAD 已放几件 */

    for (uint32_t t = 0; t < budget_ms; t += slice) {
        if ((last_drop_ms == 0u) && (base_items > 0) && (g_res.items_done >= base_items)) {
            last_drop_ms = t;
            printf("  [仿真] t=%ums 初始 %d 件全部落盒 ⇒ 产品 +0.6s 读到零帧、随即开 %ums 静默窗\n"
                   "         （操作员可用时间 ≈ %ums：判零那 0.6s + 窗 %ums）\n",
                   (unsigned)t, base_items, (unsigned)CFG_REFILL_HOLD_MS,
                   (unsigned)(CFG_REFILL_HOLD_MS + 600u), (unsigned)CFG_REFILL_HOLD_MS);
        }
        while (s_evt_done < s_n_evt && s_evt[s_evt_done].at_ms <= t) {
            const evt_t *e = &s_evt[s_evt_done];

            /* ★ 空盘静默窗内补货（EV_RELOAD_EMPTY）：锚点 = **初始批全部落盒那一刻**
               （最后一件已投放、产品随即读到零帧并开 5s 静默窗）再等 2s 放货 ⇒ 稳稳落在窗内。
               ⚠️ 锚点**不能**用"托盘已空"：托盘是在**抓走**最后一件时就空了，而那一件还要走完
                 "运到投放点 → 松爪 → 静默 → 复核"（约 2s）——按"托盘空"起算会早放 ~2s，
                 那几件会出现在最后一件的**到账帧**里 ⇒ 把它判成"没减一"（s29 首版就是这么红的）。 */
            if (e->what == EV_RELOAD_EMPTY) {
                if ((last_drop_ms == 0u) || (t < (last_drop_ms + 2000u))) {
                    break; /* 还没到时机：留在队首，下一片再看（本场景里别在它后面再排事件） */
                }
                apply_event(e);
                s_evt_done++;
                continue;
            }
            /* ★ 窗内**逐件**补货（EV_RELOAD_SPREAD，s29）：最后一件落盒后 1.0/2.0/3.0s 各一件
               ⇒ 全落在 5s 窗内（窗 = [落盒+0.6s, 落盒+5.6s]）——"操作员一边放、机器一边等"。
               窗内机器**不取帧**（连看见都没看见）⇒ 窗后第一帧数量 = 3 ⇒ 件件计数正确。
               ⚠️ 这条就是"5s 窗到底有没有用"的判据：把 `CFG_REFILL_HOLD_MS` 改成 0 ⇒
                  机器在放第 2/3 件的途中就取了发车帧 ⇒ 到账数量对不上 ⇒ 少计 ⇒ 本场景变红 */
            if (e->what == EV_RELOAD_SPREAD) {
                if (last_drop_ms == 0u) {
                    break; /* 初始批还没走完：留在队首 */
                }
                if (spread_placed >= e->val) {
                    s_evt_done++;
                    continue;
                }
                if (t >= (last_drop_ms + 1000u + 1000u * (uint32_t)spread_placed)) {
                    reload_one(spread_placed);
                    spread_placed++;
                    printf("  [仿真] t=%ums 窗内放入第 %d/%d 件（最后一件落盒后 %ums）\n",
                           (unsigned)t, spread_placed, e->val,
                           (unsigned)(t - last_drop_ms));
                    if (spread_placed >= e->val) {
                        s_evt_done++;
                    }
                }
                break; /* 本片不再处理别的（逐件放要靠时间推进） */
            }
            apply_event(e);
            s_evt_done++;
        }
        /* 追加货物：必须在断链/待机之后立刻生效，才能测"非不可逆" */
        sim_run_until((uint64_t)(t + slice) * 1000ull);
        sample();
        if (s_stop_requested) {
            break;
        }
        if (expect_done > 0u && (uint32_t)g_res.items_done >= expect_done) {
            /* 最后一件的"投放 -> 复核 -> 刷屏"是异步的：立刻收尾会把第 N 次刷新截掉，
               屏上计数就永远少 1，看起来像显示丢帧。等它落屏再收 */
            sim_run_until((uint64_t)(t + 8000u) * 1000ull);
            sample();
            printf("  [仿真] 期望件数已达成，t=%us 提前结束\n",
                   (unsigned)(sim_now_ms() / 1000u));
            break;
        }
    }
    g_res.sim_ms = sim_now_ms();
}

/* ==================== 报告 ==================== */
static uint8_t expected_bin(uint8_t category)
{
    /* 与 App/sort_task.c 的 s_category_to_bin[] 一致（默认表 = 类别 1~6 依次进盒 0~5） */
    return (uint8_t)(category - 1u);
}

/* 屏上第 t 行（类别 ID t）应显示的成功数 = 该类别**真实入盒**的件数 */
static int expect_count_of_type(int t)
{
    int c = 0;

    for (int i = 0; i < s_n_items; i++) {
        if ((int)s_items[i].category == t && sim_item_bin(i) >= 0) {
            c++;
        }
    }
    return c;
}

static void print_screen_counts(void)
{
    printf("屏上每类成功数: ");
    for (int t = 1; t <= (int)CFG_NUM_CATEGORIES; t++) {
        printf("第%d行=%s ", t, sim_tjc_count(t));
    }
    printf("\n");
}

/* 屏与事实的交叉核对（题目红线：显示信息必须与实际一致）：逐行等于该类别的真实入盒数，
   且 6 行之和 = 实际入盒件数 */
static int check_screen_counts(void)
{
    int bad = 0;
    int sum = 0;

    for (int t = 1; t <= (int)CFG_NUM_CATEGORIES; t++) {
        const char *txt  = sim_tjc_count(t);
        int         got  = (txt[0] == '-') ? -1 : atoi(txt);
        int         want = expect_count_of_type(t);

        if (got != want) {
            printf("  [FAIL] 屏上第 %d 行(类别 %d) 成功数=%s，应为 %d\n", t, t, txt, want);
            bad++;
        }
        if (got > 0) {
            sum += got;
        }
    }
    if (sum != g_res.items_done) {
        printf("  [FAIL] 屏上 6 行之和 %d != 实际入盒 %d（显示与事实不一致 = 计数错误）\n",
               sum, g_res.items_done);
        bad++;
    }
    return bad;
}

/* 一行屏指令是否为"完整独立的 nX.val=<数字>"；碎片/两条黏在一行都过不了。
   S19 用它判"屏解析有没有失步" */
static int is_clean_count_line(const char *line)
{
    const char *p    = line;
    const char *base = CFG_TJC_VAR_CNT_BASE;

    while (*base != '\0') {
        if (*p++ != *base++) {
            return 0;
        }
    }
    if (*p < '0' || *p > '9') {
        return 0;
    }
    while (*p >= '0' && *p <= '9') {
        p++;
    }
    if (strncmp(p, ".val=", 5u) != 0) {
        return 0;
    }
    p += 5;
    if (*p < '0' || *p > '9') {
        return 0;
    }
    while (*p >= '0' && *p <= '9') {
        p++;
    }
    return (*p == '\0') ? 1 : 0;
}

static void dump_trace(void)
{
    printf("\n---- 串口屏收到的指令（共 %d 条）----\n", sim_tjc_line_count());
    for (int i = 0; i < sim_tjc_line_count(); i++) {
        printf("  %s\n", sim_tjc_line(i));
    }
}

static void report(uint32_t budget_ms)
{
    printf("\n================ 仿真结果 ================\n");
    printf("虚拟运行时长      : %.2f s / 预算 %.2f s\n",
           (double)g_res.sim_ms / 1000.0, (double)budget_ms / 1000.0);
    printf("入盒成功          : %d / 放货 %d\n", g_res.items_done, s_n_items);
    printf("抓空              : %d\n", g_res.items_empty_grab);
    printf("数量复核判没减一  : %u 次（到账帧数量 ≠ 发车帧数量-1）\n",
           (unsigned)Sort_CountMissCount());
    printf("数量复核等到账超时: %u 次（链路/上位机停发；与「没减一」分开计）\n",
           (unsigned)Sort_CountNoFrameCount());
    printf("激光判夹空        : %u 次（现场已关闸 CFG_LASER_ENABLE=0 ⇒ 恒 0）\n",
           (unsigned)Sort_LaserMissCount());
    printf("落在非法位置      : %d\n", g_res.items_dropped_outside);
    printf("XY 记账作废次数   : %d\n", g_res.xy_untrust_events);
    printf("转盘记账作废次数  : %d\n", g_res.tray_untrust_events);
    printf("顶硬限位(0.5s计)  : %d 次，实测最小坐标 (%.2f, %.2f) mm\n",
           g_res.hard_stop_hits, g_res.min_x_mm, g_res.min_y_mm);
    printf("实测最大坐标      : (%.2f, %.2f) mm\n", g_res.max_x_mm, g_res.max_y_mm);
    printf("ZDT 帧: A=%ld B=%ld 盘=%ld | 位置%ld 速度%ld 停止%ld 使能%ld 被拒%ld\n",
           g_zdt_frames[0], g_zdt_frames[1], g_zdt_frames[2],
           g_zdt_pos_frames, g_zdt_speed_frames, g_zdt_stop_frames,
           g_zdt_enable_frames, g_zdt_ign_busy);
    print_screen_counts();
    printf("每件事件: ");
    for (int i = 0; i < s_n_items; i++) {
        printf("#%d%s->%d ", i,
               (sim_item_state(i) == 2) ? "" : (sim_item_state(i) == 1 ? "(未入盒)" : "(异常)"),
               sim_item_bin(i));
    }
    printf("\n");
}

static int check_bins(void)
{
    int bad = 0;

    for (int i = 0; i < s_n_items; i++) {
        int want = expected_bin(s_items[i].category);
        int got = sim_item_bin(i);

        if (got < 0) {
            continue;   /* 没入盒的由 items_done 计数暴露 */
        }
        if (got != want) {
            printf("  [错盒] #%d 类别 %u 应进 %d 盒，实际 %d 盒\n",
                   i, s_items[i].category, want, got);
            bad++;
        }
    }
    return bad;
}

/* ==================== 场景 ==================== */
typedef struct
{
    const char *name;
    const char *desc;
    uint32_t budget_ms;
    uint32_t expect_done;
    void (*setup)(void);
    int (*verify)(void);      /* 返回 0 = 通过 */
} scenario_t;

static void s_default_items(void)
{
    load_default_lot(12);
}

static int v_all_in_right_bin(void)
{
    int bad = 0;

    if (g_res.items_done != 12) {
        printf("  [FAIL] 入盒 %d != 12\n", g_res.items_done);
        bad++;
    }
    if (g_res.items_empty_grab != 0) {
        printf("  [FAIL] 出现 %d 次抓空\n", g_res.items_empty_grab);
        bad++;
    }
    if (g_res.items_dropped_outside != 0) {
        printf("  [FAIL] 出现 %d 次非法落点\n", g_res.items_dropped_outside);
        bad++;
    }
    if (g_res.hard_stop_hits != 0) {
        printf("  [FAIL] 顶到机械硬限位 %d 次\n", g_res.hard_stop_hits);
        bad++;
    }
    if (g_res.tray_untrust_events != 0) {
        printf("  [FAIL] 转盘记账被作废 %d 次\n", g_res.tray_untrust_events);
        bad++;
    }
    if (g_res.xy_untrust_events != 0) {
        printf("  [FAIL] XY 记账被作废 %d 次\n", g_res.xy_untrust_events);
        bad++;
    }
    bad += check_bins();
    bad += check_screen_counts();
    return bad;
}

/* --- S1 正常路径 --- */
static void s1_setup(void) { s_default_items(); }

/* --- S2 清空后再放货（"完成"不可逆吗？） --- */
static void s2_setup(void)
{
    load_default_lot(3);
    /* 不设 EV_STOP：跑完 3 件后还要留约一倍时间空转，验证"托盘已空"不会误报也不会乱动 */
}

static int v_s2(void)
{
    int bad = 0;

    if (g_res.items_done != 3) {
        printf("  [FAIL] 第一轮应完成 3 件，实测 %d\n", g_res.items_done);
        bad++;
    }
    bad += check_bins();
    return bad;
}

/* --- S3 视觉链路中途断开 ---
 * 不断 EV_STOP：断链丢的只是"目标来源"，不是安全链，所以必须跑到第 12 件才算验完恢复能力。
 * 窗口刻意跨在"本件已投放、下一件还没领到目标"的边界上（每件约 21s，首件约 42s），
 * 否则整个断链会被一次运动的延时吸收掉，Sort_WaitVisionLink 这条分支根本不会被走到 */
static void s3_setup(void)
{
    load_default_lot(12);
    evt_add(100000u, EV_K230_LINK, 1);
    evt_add(130000u, EV_K230_LINK, 0);
}

static int v_s3(void)
{
    int bad = 0;

    if (g_res.items_dropped_outside != 0) {
        printf("  [FAIL] 断链期间发生了非法落点\n");
        bad++;
    }
    if (g_res.items_done != 12) {
        printf("  [FAIL] 断链只是等下一帧，恢复后应补齐 12 件，实测 %d\n", g_res.items_done);
        bad++;
    }
    if (g_res.xy_untrust_events != 0 || g_res.tray_untrust_events != 0) {
        printf("  [FAIL] 视觉断链不该动摇位置记账（作废 %d/%d 次）\n",
               g_res.xy_untrust_events, g_res.tray_untrust_events);
        bad++;
    }
    bad += check_bins();
    printf("  [观察] 断链 30s 后恢复，分拣任务仍在跑=%d\n", sim_task_state(3) != 3);
    print_screen_counts();
    return bad;
}

/* --- S6 转盘打滑 30% ---
 *
 * ⚠️ 本场景是【已知能力边界】，不是待修的代码缺陷 —— 结论有唯一性，写在这里免得后人重复劳动。
 *
 * 现象：30% 打滑注入后，转盘记账作废 2 次（索引复核确实检出了），但仍有 8 件放错盒。
 *
 * 原因是机械反馈的分辨率，不是软件取舍：
 *   - 转盘一圈只有【一个】索引沿，两次沿之间位置完全靠脉冲死推算；
 *   - 打滑量正比于走过的距离，所以"转过不足一圈"的打滑可以让"账面的整圈数"与
 *     "实物的整圈数"仍然相等 —— 索引复核完全正确，可实物已经偏了；
 *   - 偏差一旦超过半格（30 度）就会落到相邻盒，而这一路上没有任何传感器能看见它。
 * 也就是说：软件能保证的只有"检出上界 = 一段约一圈的行程"，这一段之内必然存在盲区。
 * 想把盲区消掉只有三条路，都属于硬件/标定范畴：
 *   1) 转盘加装更高分辨率的编码器或每格一个索引凸块；
 *   2) 缩短"重新回零"周期（工程折中：每 N 件强制重标一次，N 越小越准、越慢）；
 *   3) 从机械上消除打滑（张紧/防滑处理联轴器与减速箱）。
 * 本场景保留为 FAIL 是【故意】的：它如实反映"30% 打滑下装置会放错盒"这一事实，
 * 在 1)/2)/3) 任一落地之前，把它改成 PASS 只会掩盖风险。 */
static void s6_setup(void)
{
    load_default_lot(12);
    evt_add(60000u, EV_TRAY_SLIP, 30);
}

static int v_s6(void)
{
    int bad = 0;

    printf("  [观察] 打滑注入后：转盘记账作废 %d 次，重新回零后继续\n",
           g_res.tray_untrust_events);
    if (g_res.tray_untrust_events == 0) {
        printf("  [FAIL] 30%% 打滑（约 1/3 圈）未被索引复核检出 -> 会静默放错盒\n");
        bad++;
    }
    bad += check_bins();
    if (bad > 0) {
        printf("  [结论] 见本函数顶部注释：一圈只有一个索引沿，盲区是反馈分辨率上限，\n"
               "         属硬件/标定范畴（加索引 / 缩短重标周期 / 消除打滑），改软件消不掉\n");
    }
    return bad;
}

/* --- S7 X 限位常闭 --- */
static void s7_setup(void)
{
    load_default_lot(2);
    g_sim.limit_x_stuck = 1u;
}

static int v_s7(void)
{
    printf("  [观察] X 限位常闭：入盒 %d 件（应为 0，回零不可能成功），"
           "XY 作废 %d 次，顶硬限位 %d 次，最小 X=%.2f\n",
           g_res.items_done, g_res.xy_untrust_events, g_res.hard_stop_hits, g_res.min_x_mm);
    return (g_res.items_done == 0) ? 0 : 1;
}

/* --- S8 X 限位断线（安全网失效） --- */
static void s8_setup(void)
{
    load_default_lot(2);
    g_sim.limit_x_dead = 1u;
    evt_add(40000u, EV_LIMIT_X, 0);   /* 之后恢复正常，看会不会一路顶死 */
}

static int v_s8(void)
{
    printf("  [观察] X 限位断线期间：回零会怎么走 400mm 全行程，最小 X=%.2f mm，"
           "顶硬限位 %d 次（每次代表 0.5s 持续顶压）\n",
           g_res.min_x_mm, g_res.hard_stop_hits);
    return 0;
}

/* --- S9 驱动器运动中不接管新位置帧（语义未标定） --- */
static void s9_setup(void)
{
    load_default_lot(4);
    g_sim.zdt_busy_rejects_fd = 1u;
}

static int v_s9(void)
{
    printf("  [观察] 位置帧被拒 %ld 次，最小坐标 (%.2f,%.2f)，顶硬限位 %d 次，入盒 %d 件\n",
           g_zdt_ign_busy, g_res.min_x_mm, g_res.min_y_mm,
           g_res.hard_stop_hits, g_res.items_done);
    return 0;
}

/* --- S10 视觉报越界坐标 --- */
static void s10_setup(void)
{
    load_default_lot(2);
    g_sim.k230_out_of_envelope = 1u;
}

static int v_s10(void)
{
    printf("  [观察] 越界目标：入盒 %d（应 0），非法落点 %d（应 0），"
           "抓空 %d，机器仍在等待=%d\n",
           g_res.items_done, g_res.items_dropped_outside, g_res.items_empty_grab,
           sim_task_state(3) != 3);
    if (g_res.items_done != 0 || g_res.items_dropped_outside != 0) {
        return 1;
    }
    return 0;
}

/* --- S11 电机堵转（A 轴） --- */
static void s11_setup(void)
{
    load_default_lot(2);
    evt_add(24000u, EV_MOTOR_BLOCK, 0);
}

static int v_s11(void)
{
    printf("  [观察] A 轴堵转后：入盒 %d，抓空 %d，非法落点 %d，最小坐标 (%.2f,%.2f)\n",
           g_res.items_done, g_res.items_empty_grab, g_res.items_dropped_outside,
           g_res.min_x_mm, g_res.min_y_mm);
    return 0;
}

/* --- S12 串口屏断线（回执握手是否退化为死等） --- */
static void s12_setup(void)
{
    load_default_lot(2);
    evt_add(24000u, EV_SERIAL_DEAD, 0);
}

static int v_s12(void)
{
    printf("  [观察] USART6 发送失败后：入盒 %d，分拣任务状态=%d（3=已死）切换次数=%u\n",
           g_res.items_done, sim_task_state(3), sim_task_switches(3));
    print_screen_counts();
    printf("  [观察] 屏上第 1 行若停在旧值 = 显示链路断了而分拣还停在原地等它（红线守住）\n");
    return 0;
}

/* --- S13 视觉系统偏差 8mm（标定没做好） --- */
static void s13_setup(void)
{
    load_default_lot(4);
    g_plant.vision_err_mm = 8.0;
}

static int v_s13(void)
{
    printf("  [观察] 视觉偏 8mm：抓空 %d 次、入盒 %d 件（R5 抓空风险的量化）\n",
           g_res.items_empty_grab, g_res.items_done);
    return 0;
}

/* --- S14 凸块作用宽度 6 度（机械比预期宽） --- */
static void s14_setup(void)
{
    load_default_lot(2);
    g_plant.cam_deg = 6.0;
}

static int v_s14(void)
{
    printf("  [观察] 凸块 6° 时：CFG_TRAY_BACKOFF_PULSES=600 只有 %.3f°，"
           "转盘回零是否还能脱开？入盒 %d 件\n",
           600.0 / (double)CFG_TRAY_PULSES_PER_REV * 360.0, g_res.items_done);
    return (g_res.items_done == 2) ? 0 : 1;
}

/* --- S15 上电时凸块正压在开关上 --- */
static void s15_setup(void)
{
    load_default_lot(2);
    g_plant.tray_start_deg = 1.0;
}

static int v_s15(void)
{
    printf("  [观察] 上电即在凸块窗口内（回零步骤0 的退出动作）：入盒 %d 件\n",
           g_res.items_done);
    return (g_res.items_done == 2) ? 0 : 1;
}

/* --- S16 CFG_TRAY_DIR_FORWARD 标定反了 --- */
static void s16_setup(void)
{
    load_default_lot(12);
    g_plant.tray_phys_sign = -1;
}

static int v_s16(void)
{
    int bad = check_bins();

    printf("  [观察] 方向标定反了：入盒 %d 件，落点全部镜像；"
           "转盘记账作废 %d 次（0 = 自检完全没发现）\n",
           g_res.items_done, g_res.tray_untrust_events);
    if (g_res.tray_untrust_events == 0 && bad > 0) {
        printf("  [确认] 索引复核对方向错误【无检出能力】—— 静默放错盒，直接 0 分\n");
    }
    return 0;   /* 这是"如实描述"场景，不作 FAIL 判定 */
}

/* --- S17 每格都有凸块（托盘 6 个感应块） --- */
static void s17_setup(void)
{
    load_default_lot(4);
    g_sim.tray_cam_count = 6;
}

static int v_s17(void)
{
    printf("  [观察] 每圈 6 个索引沿：转盘记账作废 %d 次，入盒 %d 件\n",
           g_res.tray_untrust_events, g_res.items_done);
    return 0;
}

/* --- S18 按当前出厂配置（投放坐标未回填） --- */
/* s18（出厂配置坐标未回填）已于 2026-10-01 退役：产品 sort_config.h 的几何/标定值
   已用台架实测值回填，"未回填=拒绝一切绝对定位"的前提不复存在，该场景无法再构造。
   它验证过的保护（未标定→包络拒绝→不乱走）由代码评审与台架流程覆盖 */

/* --- S19 半截帧回归：屏端静默丢一次发送调用的字节 ---
 * 一轮刷新现在只有 1 条指令（那一行的成功数量）。产品写的是**绝对值**，所以丢一次之后
 * 下一件落屏会把该行补齐（若实现改成 val++，这里就永远少 1 = 计数错误）。
 * 布防时刻落在【上电 6 行清零之后、首件刷屏之前】：首件刷新约 21s，故取 10s。
 * 判据：丢弃后落屏的下一条必须是完整独立的 nX.val= 行（出现碎片/合并 = 屏解析已失步）；
 * 6 行计数仍与事实一致；分拣本身不受影响。 */
static void s19_setup(void)
{
    load_default_lot(2);
    /* 10s 后第一次屏发送 = 首件的成功数量（上电清零的 6 条已在 t≈0 发完） */
    sim_screen_arm_drop(10000u, 0u);
}

static int v_s19(void)
{
    int bad = 0;
    const char *line = sim_tjc_line(sim_screen_drop_line());

    if (sim_screen_drops() != 1u) {
        printf("  [FAIL] 注入应恰好命中一次，实测 %u 次\n", sim_screen_drops());
        bad++;
    }
    if (!is_clean_count_line(line)) {
        printf("  [FAIL] 丢弃后落屏的下一条指令是 '%s'，应为完整独立的 nX.val= 行"
               "（出现合并/碎片 = 屏解析已失步）\n",
               (line[0] == '\0') ? "<无>" : line);
        bad++;
    }
    bad += check_screen_counts();
    if (g_res.items_done != 2) {
        printf("  [FAIL] 丢一条屏指令不该影响分拣，应完成 2 件，实测 %d\n", g_res.items_done);
        bad++;
    }
    return bad;
}

/* --- S20 转盘双向走最近路径（2026-10-08"可以顺逆时针转"）：**强制反向路径** ---
 * 货物按 类别 1,6,1,6,1,6 交替布 ⇒ 盒号走 0,5,0,5,0,5：0→5 的最近路径是**反向 1 格**
 * （正转要 5 格）。判据 = s1 同款（每件都进对盒、不抓空、记账不作废）；
 * 反向那几笔顺带把"有符号脉冲记账 + 双向凸块计数（Kinematics_TrayCrossingsBetween）"跑一遍。
 * CFG_TRAY_ALLOW_REVERSE=0 时本场景也应全过（退化成单向正转，只是慢）。 */
static void s20_setup(void)
{
    for (int i = 0; i < 6; i++) {
        float   x     = -125.0f + 28.0f * (float)(i % 5);
        float   y     = -170.0f + 55.0f * (float)(i / 5);
        uint8_t cat   = (uint8_t)(((i % 2) == 0) ? 1 : 6); /* 盒号 0/5 交替 ⇒ 强制反向 */
        uint8_t shape = (uint8_t)(i % 4);

        add_item(x, y, cat, shape, (uint8_t)((i % 2) == 0));
    }
}

static int v_s20(void)
{
    int bad = 0;

    if (g_res.items_done != 6) {
        printf("  [FAIL] 入盒 %d != 6（反向路径没走通？）\n", g_res.items_done);
        bad++;
    }
    if (g_res.items_empty_grab != 0) {
        printf("  [FAIL] 出现 %d 次抓空\n", g_res.items_empty_grab);
        bad++;
    }
    if (g_res.tray_untrust_events != 0) {
        printf("  [FAIL] 转盘记账被作废 %d 次（反向路径的凸块/脉冲复核不过）\n",
               g_res.tray_untrust_events);
        bad++;
    }
    if (g_res.hard_stop_hits != 0) {
        printf("  [FAIL] 顶到机械硬限位 %d 次\n", g_res.hard_stop_hits);
        bad++;
    }
    bad += check_bins();
    bad += check_screen_counts();
    return bad;
}


/* --- S21 贴边物块（目标坐标落在限位开关那一侧）---
 * 布货（真值机械坐标）：物块落在 X ≈ -10 一带 ⇒ 经"视觉标定（模型真值 -80,-45 vs 产品 -70,-57.5）"
 * 换算到机器侧就是 X ≈ 0（含一个落到越界带、会被钳回 0 的）。
 * 旧口径（钳回 0 就近抓）= **命令龙门走到坐标 0**，而 0 就是回零开关触发线 ⇒ 龙门真压上开关
 * ⇒ 守护任务作废 XY 记账 ⇒ 下一圈重新回零（现场"抓完自己回零校准"）。
 * CFG_WORK_EDGE_MARGIN_MM 把目标回拉到 `上限 − 边距`（-3；2026-10-10 由 -5 缩到 -3）
 * ⇒ 同一件照样夹得住（夹爪 25mm 延伸带、仿真抓取容差 ~1 个物块直径），开关不再被压。
 * 判据：两件都进对盒、且 **XY 记账一次都没被作废**（作废 = 真压上开关）。 */
static void s21_setup(void)
{
    add_item(-10.0f, -100.0f, 1u, 0u, 0u); /* 机器侧 X ≈ 0（钳回带上） */
    add_item(-9.0f, -60.0f, 2u, 1u, 1u);   /* 机器侧 X ≈ +1（微越界钳位带上） */
    /* 回零完成之后注入 2mm 开环丢步：钳位到 0 的目标会让真实位置顶到 +2 ⇒ 压上开关 */
}

static int v_s21(void)
{
    int bad = 0;

    if (g_res.items_done != 2) {
        printf("  [FAIL] 入盒 %d != 2（贴边件没抓成？）\n", g_res.items_done);
        bad++;
    }
    if (g_res.xy_untrust_events != 0) {
        printf("  [FAIL] XY 记账被作废 %d 次 = 龙门真压上了回零开关（贴边件的目标坐标上了限位线）\n",
               g_res.xy_untrust_events);
        bad++;
    }
    if (g_res.tray_untrust_events != 0) {
        printf("  [FAIL] 转盘记账被作废 %d 次\n", g_res.tray_untrust_events);
        bad++;
    }
    if (g_res.hard_stop_hits != 0) {
        printf("  [FAIL] 顶到机械硬限位 %d 次\n", g_res.hard_stop_hits);
        bad++;
    }
    bad += check_bins();
    bad += check_screen_counts();
    return bad;
}

/* --- S22 极限位夹取带（+Y 侧）：夹不到也去夹 ---
 * 布货（真值机械坐标）：物块落在 Y ≈ +20 一带，经视觉标定换算到机器侧 Y ≈ +8
 * （超出旧的 3mm 微越界口径、在 25mm 带内）。
 * 旧口径 = 整件作废、机器不干等；新口径 = 钳到最极限可达位（上限 − 保护边距）**照样伸爪**
 * 去抓（用户口径："不管怎么样都直接夹"，宁可试一把）。抓没抓到取决于落点离真值多远：
 *   边距 5（旧）：落点 -5 离真值 +20.5 有 25.5mm > 抓取容差 24mm（= 2×12mm）⇒ 抓空；
 *   边距 3（2026-10-10）：落点 -3 ⇒ 23.5mm ≤ 24mm ⇒ **真抓中入盒**（更近的落点就是更准）。
 * 判据：**伸爪过至少一次**（入盒 + 抓空 ≥ 1；带值 = 0 时两个都是 0）+ 记账不被作废。 */
static void s22_setup(void)
{
    add_item(-60.0f, 20.5f, 1u, 0u, 0u); /* 机器侧 (X≈-50, Y≈+8)：带内、超出 3mm 口径 */
}

static int v_s22(void)
{
    int bad = 0;

    /* "伸爪过至少一次"把新旧口径分开：带 25mm ⇒ 去过（抓中或抓空）；带 0 ⇒ 整件作废、
       一次尝试都没有（两个计数都 0）⇒ 红。★ 判据看"有没有去抓"，不看"抓到没有" */
    if ((g_res.items_done + g_res.items_empty_grab) < 1) {
        printf("  [FAIL] 一次都没伸爪（入盒 %d + 抓空 %d = 0）——极限位夹取带没生效"
               "（旧口径会整件作废）\n",
               g_res.items_done, g_res.items_empty_grab);
        bad++;
    }
    if (g_res.xy_untrust_events != 0) {
        printf("  [FAIL] XY 记账被作废 %d 次（钳位目标仍顶到了限位开关？）\n", g_res.xy_untrust_events);
        bad++;
    }
    if (g_res.tray_untrust_events != 0) {
        printf("  [FAIL] 转盘记账被作废 %d 次\n", g_res.tray_untrust_events);
        bad++;
    }
    bad += check_bins();
    bad += check_screen_counts();
    return bad;
}

/* --- S23 限位电气毛刺（假"撞限位"）---
 * 现场现象（2026-10-08 用户）：①"抓完之后有时自己回零校准" ②"一上来连夹持点都到不了"。
 * 机制：EXTI 只认"武装状态下的一次下降沿"，舵机/电机换向的电流尖峰耦合进限位线就能造出
 * 假沿。旧口径（沿闩锁 ⇒ 急停 + 作废记账）每来一次毛刺就白跑一次回零；更糟的是若"停止帧
 * 立即发、记账晚一点才作废"那种拆分口径：毛刺被判成假之后记账被保留、运动却已被截在半路
 * ⇒ 记账说"已到位"、实物停在半路 ⇒ **坐标系静默平移**（越抓越歪、最后朝开关顶）。
 * 本场景：料盘 6 件照常运行，途中灌 4 次**纯电气沿**（开关电平一动不动，见 EV_LIMIT_GLITCH）。
 * 判据（把新旧口径分开的那一条）：**XY 记账一次都没被作废** + 6 件全部进对盒 + 不顶硬限位。
 * 反证（对照，实测过）：把 Motion_ServeEvents 退回"沿闩锁"口径 ⇒ 本场景立即变红
 * （xy_untrust ≥1、且白跑一次回零）。 */
static void s23_setup(void)
{
    load_default_lot(12); /* 跑满 12 件（≈90s）：4 次毛刺都落在"运动中/运动中段"，不是空转期 */
    evt_add(40000u, EV_LIMIT_GLITCH, 0);
    evt_add(55000u, EV_LIMIT_GLITCH, 1);
    evt_add(70000u, EV_LIMIT_GLITCH, 0);
    evt_add(85000u, EV_LIMIT_GLITCH, 1);
}

static int v_s23(void)
{
    int bad = 0;

    if (g_res.xy_untrust_events != 0) {
        printf("  [FAIL] XY 记账被作废 %d 次 = 毛刺被当成真撞（判据退回沿闩锁了？）"
               "——现场表现就是白跑一次回零\n", g_res.xy_untrust_events);
        bad++;
    }
    if (g_res.items_done != 12) {
        printf("  [FAIL] 入盒 %d != 12（毛刺不该让任何一件作废，也不该漂位）\n", g_res.items_done);
        bad++;
    }
    if (g_res.items_empty_grab != 0 || g_res.items_dropped_outside != 0) {
        printf("  [FAIL] 抓空 %d 次 / 非法落点 %d 次（坐标漂了？）\n",
               g_res.items_empty_grab, g_res.items_dropped_outside);
        bad++;
    }
    if (g_res.hard_stop_hits != 0) {
        printf("  [FAIL] 顶到机械硬限位 %d 次\n", g_res.hard_stop_hits);
        bad++;
    }
    if (g_res.tray_untrust_events != 0) {
        printf("  [FAIL] 转盘记账被作废 %d 次\n", g_res.tray_untrust_events);
        bad++;
    }
    bad += check_bins();
    bad += check_screen_counts();
    return bad;
}

/* --- S24 数量复核：夹空不计数（"真抓到才算一件"）---
 *
 * 现场缺陷的回归（用户口径："串口屏的显示是纯开环，会有夹持失败但是数量加一的问题"）：
 * 机械爪不装到位开关（R5），抓空时旧代码照样 s_total++ 再刷屏 ⇒ 屏上多一件、盒里没有
 * （显示与事实不一致 = 0 分）；上位机持续重报同一目标 ⇒ 还会再虚增一次。
 * 本场景注入"视觉系统误差 20mm"（> 抓取半径 12mm×2）：龙门照常走完全程、爪照常闭合，
 * 夹的却是空气 —— 托盘上什么都没少 ⇒ 到账帧数量 == 发车帧数量 ⇒ 判"没减一"。
 * 判据（把新旧口径分开的那一条）：**抓空期间屏上一动不动**；误差撤回后，被漏抓的那件
 * 重新报出、重新抓取、最终入盒 ⇒ 12 件全部入盒 + 屏上 6 行 = 真实入盒数（虚增一件就红）
 * + 产品自己的"数量复核判没减一"计数 = 模型观测到的抓空次数。
 * 反证：把复核关掉（产品 CFG_COUNT_VERIFY_ENABLE=0，等价于原来的"无条件计数"）⇒ 本场景
 * 立即变红：屏上计数比实际入盒多出抓空次数 —— 正是现场那条"夹持失败但数量加一"。 */
static void s24_setup(void)
{
    load_default_lot(12);
    evt_add(20000u, EV_VISION_ERR, 20); /* 整体偏 20mm：离最近物块也差 40mm > 24mm ⇒ 必抓空 */
    evt_add(50000u, EV_VISION_ERR, 0);  /* 撤回：被漏抓的那件重新报出，正常夹到 */
}

static int v_s24(void)
{
    int bad = 0;

    if (g_res.items_empty_grab < 1) {
        printf("  [FAIL] 一次抓空都没发生 —— 注入没生效，本场景什么都没验到\n");
        bad++;
    }
    if (g_res.items_done != 12) {
        printf("  [FAIL] 入盒 %d != 12（夹空只是白做一件：货还在托盘上，应能自愈重抓）\n",
               g_res.items_done);
        bad++;
    }
    if ((int)Sort_CountMissCount() != g_res.items_empty_grab) {
        printf("  [FAIL] 产品判没减一 %u 次 != 模型观测抓空 %d 次（复核口径不一致）\n",
               (unsigned)Sort_CountMissCount(), g_res.items_empty_grab);
        bad++;
    }
    bad += check_bins();
    bad += check_screen_counts(); /* 屏上 6 行 = 真实入盒数：虚增/少计都会在这里红 */
    return bad;
}

/* --- S25 数量位恒 0（上位机还没实现"下标 6"）：屏不涨、绝不虚增 ---
 *
 * 这是**上位机侧还没跟上协议时的症状冻结**（不是待修缺陷，所以判 PASS）：
 * 产品已按数量复核跑（K230 每帧还是老格式、下标 6 恒 0）⇒ 每件的到账数量都是 0，
 * 而发车数量也是 0 ⇒ 永远"没减一" ⇒ 分拣照常跑完（12 件照旧松爪、照旧各自落入自己的盒），
 * 但屏上停在 0。也就是说错只错在"少计"这一侧（肉眼可见），另一侧的"虚增"一条都不许有。
 * 现场判据：**盒里有货、屏上是 0 = 数量位没上线（或 CFG_COUNT_VERIFY_ENABLE 被改成 0），
 * 不是分拣坏了** —— 先看上位机的帧里下标 6 有没有数（台架 `k230 raw` 逐帧 hex 一眼可见）、
 * 再用台架 `k230 count` 拿一件、放一件核"数量跟着变"，别去动 sort_task 的流程。
 * ⚠️ 另一侧（数量位卡在一个固定值、且刚好等于发车帧数量）固件兜不住：那种虚增只能靠
 *    装机时核协议 + 边跑边人眼比对"屏上计数 vs 盒里实物"来发现。 */
static void s25_setup(void)
{
    load_default_lot(12);
    g_sim.k230_count_zero = 1u;
}

static int v_s25(void)
{
    int bad = 0;

    if (g_res.items_done != 12) {
        printf("  [FAIL] 入盒 %d != 12（数量位没跟上不该挡住分拣：照旧松爪、货落对盒）\n",
               g_res.items_done);
        bad++;
    }
    for (int t = 1; t <= (int)CFG_NUM_CATEGORIES; t++) {
        const char *txt = sim_tjc_count(t);
        int         got = (txt[0] == '-') ? -1 : atoi(txt);

        if (got != 0) {
            printf("  [FAIL] 屏上第 %d 行 = %s，应为 0（数量位读不出，一件都不许计数）\n", t, txt);
            bad++;
        }
    }
    if ((int)Sort_CountMissCount() != 12) {
        printf("  [FAIL] 产品判没减一 %u 次 != 12（每件都应判没减一）\n",
               (unsigned)Sort_CountMissCount());
        bad++;
    }
    bad += check_bins();
    printf("  [结论] 盒里 12 件、屏上全 0 = 数量位没上线的**可见**症状（少计，不虚增）：\n"
           "         先查上位机帧的下标 6（台架 k230 raw / k230 count），再查分拣流程\n");
    return bad;
}

/* --- S26 空盘静默窗内补货：窗后新一轮照常计数（用户口径的做法，必须好使）---
 *
 * 用户 2026-10-10 口径："当夹取完最后一个物块、投放结束之后已经读到那个为零的帧的时候，
 * 之后延时 5 秒钟什么都不读，等我们把物块重新放上去"——本场景就是这条操作流程：
 *   12 件跑空（末件复核用的是**无目标帧**的数量 0 ⇒ 末件照样 +1）
 *   → 产品判"托盘已空"⇒ 静默窗 5s（窗内模型补 3 件，位置取默认料盘空着的格子）
 *   → 窗后清缓存重取 ⇒ 下一件的发车帧是补货后的新账（3 件）⇒ 一一计数
 * 判据：15 件全部入盒、屏上 6 行 = 15、**判没减一 0 次**（补货没被当成"少减"）。
 * 为什么它必须好使：这是装置"一轮跑完接着跑下一轮"的唯一合法姿势——补货只看窗，
 * 不看人；窗内补，账就干净（见 sort_config.h 的 CFG_REFILL_HOLD_MS）。 */
static void s26_setup(void)
{
    load_default_lot(12);
    /* at_ms = 0：**不按绝对时刻**，一律等"产品自己判出空盘"（= 静默窗起点）再 +2s 放货
       （原先写 120000 时，实际判空在 ~78s ⇒ 补货落在窗**之后**，这个场景根本没在验窗内） */
    evt_add(0u, EV_RELOAD_EMPTY, 3);
}

static int v_s26(void)
{
    int bad = 0;

    if (g_res.items_done != 15) {
        printf("  [FAIL] 入盒 %d != 15（12 件 + 窗内补的 3 件）\n", g_res.items_done);
        bad++;
    }
    if (Sort_CountMissCount() != 0u) {
        printf("  [FAIL] 判没减一 %u 次 —— 静默窗内的补货被当成「少减」了："
               "窗内补货不该影响账（查 CFG_REFILL_HOLD_MS 与窗后的清缓存）\n",
               (unsigned)Sort_CountMissCount());
        bad++;
    }
    bad += check_bins();
    bad += check_screen_counts(); /* 屏上 6 行 = 15 = 真实入盒数（补货后的一轮也一件不丢） */
    return bad;
}

/* --- S27 运行中补货（托盘没空就插件）：只有可见少计，绝不虚增（口径外代价冻结）---
 *
 * 用户 2026-10-10 问的就是这一条："又会引出一个我们如果选择运行中投放物块，导致物块数量
 * 增多的问题"。答案分两半：
 *   ① 绝不虚增：数量变多只会让那一件判"没减一"⇒ **不计数**（屏上最多少一件，绝不多一件）；
 *   ② 代价可见：被"多出来的货"搅到的那一件（正在投放窗口里那一件）判不出来 ⇒ 屏上少 1，
 *      而它其实真的进盒了 ⇒ 屏 ≠ 盒 差 1（本场景把这条差额冻在这里，是**故意**的）。
 * 想避免就照 s26 的姿势做：**只在空盘静默窗内补货**（用户口径本来就是这么定的）。
 * 判据：15 件全入盒、屏上和 = 14 = 入盒数 - 1、判没减一 = 1、且**屏上任何一行都不许
 * 超过该类真实入盒数**（虚增 = 现场那条直接得 0 分的缺陷）。 */
static void s27_setup(void)
{
    load_default_lot(12);
    evt_add(60000u, EV_RELOAD, 3); /* 运行中补 3 件（此刻托盘上还有货） */
}

static int v_s27(void)
{
    int bad = 0;
    int sum = 0;

    if (g_res.items_done != 15) {
        printf("  [FAIL] 入盒 %d != 15（12 件 + 运行中补的 3 件；补货不该挡住分拣）\n",
               g_res.items_done);
        bad++;
    }
    if (Sort_CountMissCount() != 1u) {
        printf("  [FAIL] 判没减一 %u 次，应为 1（只有「被搅到的」那一件；"
               "多判 = 复核对不上账，少判 = 补货被当成真投放）\n",
               (unsigned)Sort_CountMissCount());
        bad++;
    }
    for (int t = 1; t <= (int)CFG_NUM_CATEGORIES; t++) {
        const char *txt  = sim_tjc_count(t);
        int         got  = (txt[0] == '-') ? -1 : atoi(txt);
        int         want = expect_count_of_type(t);

        if (got > want) {
            printf("  [FAIL] 屏上第 %d 行 = %s > 该类真实入盒 %d —— **虚增**（绝不允许）\n", t, txt,
                   want);
            bad++;
        }
        if (got > 0) {
            sum += got;
        }
    }
    if (sum != (g_res.items_done - 1)) {
        printf("  [FAIL] 屏上和 %d != 入盒数-1 = %d（口径外的代价应当**正好**一件，不是 %d 件）\n",
               sum, g_res.items_done - 1, g_res.items_done - sum);
        bad++;
    }
    bad += check_bins();
    printf("  [结论] 屏上比盒里少 1 件 = 运行中补货的**可见**代价（少计，绝不虚增）：\n"
           "         要避免就只在空盘静默窗里补货（s26 冻的就是那条正确姿势）\n");
    return bad;
}

/* --- S28 -X 夹爪延伸带（2026-10-01 台架口径的回归，2026-10-10 补）---
 * 布货（真值机械坐标）：物块落在 **X = -150**、Y = -170 —— 在 [-160, -135) 的延伸带内，
 * 超出软限位 -135。产品应把目标**钳到 -135 就近夹取**（夹爪构造够得着那 15mm），
 * 于是这一件照样抓到、入盒、计数（= 用户口径"贴边件不再整件作废、机器不干等"）。
 * 判据：入盒 1 + 屏上该类 = 1（真抓到才计数）+ 无抓空 + 无非法落点。
 * 反证（把延伸带收掉）：产品 `CFG_XY_X_CLAMP_MM` 从 -160 改回 -135（= 取消延伸带）
 * ⇒ 目标 -150 < -135 直接作废 ⇒ **入盒 0、机器干等** ⇒ 本场景立刻红。
 * （这条路径原先没有仿真覆盖；S21/S22 覆盖的是保护边距与 +Y 带，本场景补 -X 侧） */
static void s28_setup(void)
{
    add_item(-150.0f, -170.0f, 3u, 0u, 0u); /* 机器侧 (X=-150, Y=-170)：延伸带内 */
}

static int v_s28(void)
{
    int bad = 0;

    if (g_res.items_done != 1) {
        printf("  [FAIL] 入盒 %d != 1（延伸带内的件应钳到 -135 就近夹住；0 件 = 又整件作废了）\n",
               g_res.items_done);
        bad++;
    }
    if (g_res.items_empty_grab != 0) {
        printf("  [FAIL] 出现 %d 次抓空（钳到 -135 后 15mm 内的件应当夹得到）\n",
               g_res.items_empty_grab);
        bad++;
    }
    bad += check_bins();
    bad += check_screen_counts(); /* 屏上 = 盒里（数量复核口径下"真抓到"才有数） */
    return bad;
}

/* --- S29 空盘静默窗**真的有 5 秒吗**（窗内逐件补货 = 操作员的真实放法）---
 *
 * s26 验的是"窗内一次补完 ⇒ 账不被搅"；本场景进一步验**窗的时长本身**：
 * 12 件跑空 → 产品判出"托盘已空"（= 静默窗起点，日志会打印那一刻）→ 操作员**逐件**补
 * （判空后 1s / 2s / 3s 各放一件，全在 5s 窗内）→ 窗内机器一件都不该抓（它不取帧，
 * 连看见都没看见）→ 窗后第一帧 = 3 件 ⇒ 件件计数正确。
 * 判据：15 件全入盒、屏上 6 行 = 15、**判没减一 0 次**。
 * 反证（把窗时长改 0）：`CFG_REFILL_HOLD_MS` 0 ⇒ 机器在放第 2/3 件的过程中就取发车帧
 * （数量 1）⇒ 那一件的到账数量对不上 ⇒ 少计 ⇒ 屏上 ≠ 盒里 ⇒ 本场景立刻红
 * ——这正是"为什么必须有那个 5s"的直接证据。
 * ⚠️ 现场只能间接看：最后一件落下后约 6.5s（判空 1.6 + 窗 5.0）机器才开始抓第一件新货；
 * 台架 `k230 empty` 可以把这段时间线逐秒打出来（见 dbg_config.h §3.6b 附近说明）。 */
static void s29_setup(void)
{
    load_default_lot(12);
    evt_add(0u, EV_RELOAD_SPREAD, 3); /* at_ms = 0：等"空盘判据"（窗起点）起逐件放 */
}

static int v_s29(void)
{
    int bad = 0;

    if (g_res.items_done != 15) {
        printf("  [FAIL] 入盒 %d != 15（12 件 + 窗内逐件补的 3 件）\n", g_res.items_done);
        bad++;
    }
    if (Sort_CountMissCount() != 0u) {
        printf("  [FAIL] 判没减一 %u 次 —— 窗内补货被机器看见了：静默窗没起作用"
               "（`CFG_REFILL_HOLD_MS` 是不是被改小了？）\n",
               (unsigned)Sort_CountMissCount());
        bad++;
    }
    bad += check_bins();
    bad += check_screen_counts();
    return bad;
}

static scenario_t s_sc[] = {
    { "s1",  "正常路径 12 件",                 600000u, 12, s1_setup,  v_all_in_right_bin },
    { "s2",  "只放 3 件，跑完后等待",          150000u, 0,  s2_setup,  v_s2 },
    { "s3",  "视觉链路中途断开 30s",          600000u, 12, s3_setup,  v_s3 },
    { "s6",  "转盘打滑 30%",                  600000u, 0,  s6_setup,  v_s6 },
    { "s7",  "X 限位常闭",                    90000u,  0,  s7_setup,  v_s7 },
    { "s8",  "X 限位断线（安全网失效）",      120000u, 0,  s8_setup,  v_s8 },
    { "s9",  "驱动器运动中不接管新指令",       200000u, 0,  s9_setup,  v_s9 },
    { "s10", "视觉报越界坐标 9000mm",          90000u,  0,  s10_setup, v_s10 },
    { "s11", "A 电机堵转",                    120000u, 0,  s11_setup, v_s11 },
    { "s12", "串口屏断线",                    120000u, 0,  s12_setup, v_s12 },
    { "s13", "视觉系统偏差 8mm",              150000u, 0,  s13_setup, v_s13 },
    { "s14", "凸块作用宽度 6 度",             120000u, 2,  s14_setup, v_s14 },
    { "s15", "上电凸块正压开关",              120000u, 2,  s15_setup, v_s15 },
    { "s16", "转盘方向标定反了",              600000u, 12, s16_setup, v_s16 },
    { "s17", "每圈 6 个索引沿",               300000u, 0,  s17_setup, v_s17 },
    { "s19", "屏指令半截帧（静默丢一次发送）",   120000u, 2,  s19_setup, v_s19 },
    { "s20", "转盘双向最近路径（强制反向）",    300000u, 6,  s20_setup, v_s20 },
    { "s21", "贴边件不上限位线（保护边距）",    150000u, 2,  s21_setup, v_s21 },
    { "s22", "极限位夹取带（夹不到也去夹）",  120000u, 0,  s22_setup, v_s22 },
    { "s23", "限位电气毛刺不假急停（电平连续才认）", 300000u, 12, s23_setup, v_s23 },
    { "s24", "数量复核：夹空不计数（抓空期间屏上不动）", 300000u, 12, s24_setup, v_s24 },
    { "s25", "数量位恒 0（上位机未实现）：屏不涨、绝不虚增", 150000u, 0, s25_setup, v_s25 },
    { "s26", "空盘静默窗内补货：窗后新一轮照常计数",    600000u, 0,  s26_setup, v_s26 },
    { "s27", "运行中补货：只有可见少计、绝不虚增",      600000u, 0,  s27_setup, v_s27 },
    { "s28", "-X 夹爪延伸带（贴边件照样抓到）",        120000u, 1,  s28_setup, v_s28 },
    { "s29", "空盘静默窗 5s 真的存在（窗内逐件补货）",  600000u, 0,  s29_setup, v_s29 },
};

static void list_scenarios(void)
{
    for (size_t i = 0; i < sizeof(s_sc) / sizeof(s_sc[0]); i++) {
        printf("  %-5s %s\n", s_sc[i].name, s_sc[i].desc);
    }
}

int main(int argc, char **argv)
{
    scenario_t *sc = NULL;

    if (argc < 2) {
        printf("用法: %s <scenario> [v]\n", argv[0]);
        list_scenarios();
        return 2;
    }
    for (size_t i = 0; i < sizeof(s_sc) / sizeof(s_sc[0]); i++) {
        if (strcmp(argv[1], s_sc[i].name) == 0) {
            sc = &s_sc[i];
            break;
        }
    }
    if (sc == NULL) {
        printf("未知场景 %s\n", argv[1]);
        list_scenarios();
        return 2;
    }

    sim_reset();
    sc->setup();
    sim_load_items(s_items, s_n_items);

    if (CFG_DROP_X_MM != 0.0f) {
        if (fabsf(CFG_DROP_X_MM - g_plant.drop_x) > 0.01f ||
            fabsf(CFG_DROP_Y_MM - g_plant.drop_y) > 0.01f) {
            printf("致命：填充的几何常量与模型真值不一致，结论无意义\n");
            return 3;
        }
    }
    /* 全称"屏工程"的控件名（字符串没法做编译期比较，启动时比对）：
       对不上时真机表现是那一列永远不刷新，任何自检都看不出来 */
    if (strcmp(CFG_TJC_VAR_CNT_BASE, SIM_TJC_CNT_BASE) != 0 ||
        CFG_TJC_CNT_FIRST != SIM_TJC_CNT_FIRST) {
        printf("致命：屏控件名（产品 %s 起 %d / 屏工程 %s 起 %d）不一致，仿真结论无意义\n",
               CFG_TJC_VAR_CNT_BASE, (int)CFG_TJC_CNT_FIRST, SIM_TJC_CNT_BASE,
               (int)SIM_TJC_CNT_FIRST);
        return 3;
    }
    /* 视觉标定同理：产品侧一旦回填，就必须与模型真值逐字一致，否则"仿真过、真机偏" */
    if (CFG_VISION_ORIGIN_X_MM != 0.0f || CFG_VISION_ORIGIN_Y_MM != 0.0f) {
        if (fabsf(CFG_VISION_ORIGIN_X_MM - (float)g_plant.vision_origin_x_mm) > 0.01f ||
            fabsf(CFG_VISION_ORIGIN_Y_MM - (float)g_plant.vision_origin_y_mm) > 0.01f ||
            CFG_VISION_SIGN_VY_TO_X != (float)g_plant.vision_sign_vy_to_x ||
            CFG_VISION_SIGN_VX_TO_Y != (float)g_plant.vision_sign_vx_to_y) {
            printf("致命：视觉标定常量与模型真值不一致，结论无意义\n");
            return 3;
        }
    }
    if (argc > 2) {
        g_plant.verbose = 1;
        g_sim.verbose = 1;
    }

    printf("场景 %s：%s（货物 %d 件，几何 %s）\n", sc->name, sc->desc, s_n_items,
           (CFG_DROP_X_MM != 0.0f) ? "已回填" : "未回填(0/0)");
    printf("  参数: DROP=(%.0f,%.0f) PARK=(%.0f,%.0f) 包络 X[%.0f,%.0f] Y[%.0f,%.0f]\n",
           (double)CFG_DROP_X_MM, (double)CFG_DROP_Y_MM,
           (double)CFG_PARK_X_MM, (double)CFG_PARK_Y_MM,
           (double)CFG_WORK_X_MIN_MM, (double)CFG_WORK_X_MAX_MM,
           (double)CFG_WORK_Y_MIN_MM, (double)CFG_WORK_Y_MAX_MM);
    printf("  视觉: 模型原点=(%.0f,%.0f) 符号 vy->x=%+.0f vx->y=%+.0f | 产品标定 %s\n",
           g_plant.vision_origin_x_mm, g_plant.vision_origin_y_mm,
           (double)g_plant.vision_sign_vy_to_x, (double)g_plant.vision_sign_vx_to_y,
           (CFG_VISION_ORIGIN_X_MM != 0.0f || CFG_VISION_ORIGIN_Y_MM != 0.0f) ? "已回填"
                                                                            : "未回填(0,0)");

    if (sim_boot() != 0) {
        printf("!! 外设初始化进入 Error_Handler，仿真中止\n");
        return 4;
    }
    boot_product();
    run_budget(sc->budget_ms, sc->expect_done);
    report(sc->budget_ms);

    int bad = sc->verify();

    if (g_plant.verbose) {
        dump_trace();
    }
    printf("\n结论: %s\n", (bad == 0) ? "PASS" : "FAIL");
    return (bad == 0) ? 0 : 1;
}
