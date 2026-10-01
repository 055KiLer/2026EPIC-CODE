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
    EV_RELOAD,         /* val: 追加几件货 */
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
    case EV_RELOAD:
        printf("\n>>> [注入] t=%ums 人工再放入 %d 件货\n\n", e->at_ms, e->val);
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

    for (uint32_t t = 0; t < budget_ms; t += slice) {
        while (s_evt_done < s_n_evt && s_evt[s_evt_done].at_ms <= t) {
            apply_event(&s_evt[s_evt_done]);
            s_evt_done++;
        }
        /* 追加货物：必须在断链/待机之后立刻生效，才能测"非不可逆" */
        sim_run_until((uint64_t)(t + slice) * 1000ull);
        sample();
        if (s_stop_requested) {
            break;
        }
        if (expect_done > 0u && (uint32_t)g_res.items_done >= expect_done) {
            /* 最后一件的"投放 -> 刷屏"是异步的：立刻收尾会把第 N 次刷新截掉，
               屏上计数就永远少 1，看起来像显示丢帧。等它落屏再收 */
            sim_run_until((uint64_t)(t + 6000u) * 1000ull);
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
