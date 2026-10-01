/**
 * @file    dbg_vision.c
 * @brief   视觉链路调试模块（见 dbg_vision.h）
 * @note    缓存语义与产品 vision_task.c 对齐（联调时两边现象可直接对照）：
 *            链路活着 = 最近 CFG_K230_LINK_DEAD_MS 内有**校验通过**的帧
 *            可用目标 = 有目标帧 且 类别 ∈ 1~6 且 形状 ∈ 0~3（越界 = 不可用）
 *          区别只有一条：产品按 RTOS 任务轮询，这里由主循环每圈调 Dbg_Vision_Poll()
 *          换算 Vision_ToMachine 与产品逐字相同：两轴交换 + 平移（2026-09-30 视觉组口径）
 */
#include "dbg_vision.h"
#include "dbg_config.h"
#include "dbg_uart.h"
#include "dbg_print.h"
#include "sort_config.h"

static K230_Target_t s_target;        /* 最近一帧"可用"目标（usable 口径 = 产品） */
static uint32_t      s_target_tick;   /* 该帧到达时刻 */
static uint32_t      s_frame_tick;    /* 最近一次校验通过帧的时刻（链路活性） */
static uint32_t      s_cnt_target;    /* 可用目标帧数 */
static uint32_t      s_cnt_empty;     /* 无目标帧数（状态 0x00） */
static uint32_t      s_cnt_bad;       /* 有目标但类别/形状越界（= 上位机与主控口径不一致） */
static uint8_t       s_seen;          /* 上电以来有没有收过任何校验通过的帧 */

/* 浮点打印成 d.d（自写格式器没有 %f；同 dbg_app.c 的 PrintF1） */
static void Dbg_Vision_PrintF1(float v) {
    int32_t t = (int32_t)(v * 10.0f + ((v >= 0.0f) ? 0.5f : -0.5f));

    Dbg_Printf("%d.%d", (int)(t / 10), (int)((t < 0 ? -t : t) % 10));
}

/* ★ 与产品 vision_task.c 的 Vision_ToMachine 逐字相同（两轴交换 + 平移）。
   改产品的换算口径时，这里必须同步——两边不一致的话，`k230` 打的机械坐标就是假的 */
static void Dbg_Vision_ToMachineImpl(const K230_Target_t *t, float *x_mm, float *y_mm) {
    if (x_mm != NULL) {
        *x_mm = CFG_VISION_ORIGIN_X_MM + CFG_VISION_SIGN_VY_TO_X * t->y_mm;
    }
    if (y_mm != NULL) {
        *y_mm = CFG_VISION_ORIGIN_Y_MM + CFG_VISION_SIGN_VX_TO_Y * t->x_mm;
    }
}

uint8_t Dbg_Vision_LinkAlive(void) {
    return (s_seen != 0u) && ((HAL_GetTick() - s_frame_tick) <= CFG_K230_LINK_DEAD_MS) ? 1u : 0u;
}

/* 取"有效且新鲜"的目标（不消费；口径 = 产品 Vision_GetTarget 的前两个判据，
   没有"消费"概念——台架没有分拣任务，同一目标可重复取） */
uint8_t Dbg_Vision_GetTarget(K230_Target_t *out) {
    if (out == NULL) {
        return 0u;
    }
    if ((s_target.valid == 0u) ||
        ((HAL_GetTick() - s_target_tick) > CFG_K230_TARGET_FRESH_MS)) {
        return 0u;
    }
    *out = s_target;
    return 1u;
}

uint32_t Dbg_Vision_NoneCount(void) {
    return s_cnt_empty;
}

void Dbg_Vision_ToMachine(const K230_Target_t *t, float *x_mm, float *y_mm) {
    Dbg_Vision_ToMachineImpl(t, x_mm, y_mm);
}

/* 事件 -> 缓存/计数/链路（Poll 与 SimFeed 共用同一个入账口）：
   ★ SimFeed 走解析器正门后**必须**同样入账——此前它把事件丢掉、缓存只在 Poll 里更新，
   结果 `k230 sim` 注帧后快照照样显示"缓存目标：无"，照 上台架步骤.md 测会误判解析器坏了 */
static void Dbg_Vision_ApplyEvent(K230_Event_e ev, const K230_Target_t *frame, uint32_t now) {
    /* usable 判据 = 产品 vision_task.c 同款：类别 1~6 且形状 0~3（0=默认方案、1~3=特调），
       越界则宁可不抓，绝不用取模硬套错误类别 / 拿错夹持方案去夹 */
    uint8_t usable;

    if (ev == K230_EV_NONE) {
        return; /* 半帧或校验失败（解析器内部丢弃） */
    }
    s_frame_tick = now;
    s_seen       = 1u;

    usable = (ev == K230_EV_TARGET) && (frame->category >= 1u) &&
             (frame->category <= (uint8_t)CFG_NUM_CATEGORIES) &&
             (frame->shape <= (uint8_t)CFG_NUM_SHAPES);

    if (usable != 0u) {
        s_target      = *frame;
        s_target_tick = now;
        s_cnt_target++;
    } else {
        s_target.valid = 0u;
        if (ev == K230_EV_TARGET) {
            s_cnt_bad++;
        } else {
            s_cnt_empty++;
        }
    }
}

void Dbg_Vision_Poll(void) {
    while (Dbg_Uart_VisionAvailable() > 0u) {
        K230_Target_t frame;
        uint8_t       byte = Dbg_Uart_VisionReadByte();
        K230_Event_e  ev   = K230_FeedByte(byte, &frame);

        Dbg_Vision_ApplyEvent(ev, &frame, HAL_GetTick());
    }
}

void Dbg_Vision_Flush(void) {
    Dbg_Uart_VisionFlush(); /* 丢掉线上积压的半帧 + 重新武装接收 */

    s_target.valid = 0u;
    s_cnt_target   = 0u;
    s_cnt_empty    = 0u;
    s_cnt_bad      = 0u;
    s_seen         = 0u;
    s_frame_tick   = HAL_GetTick();
    s_target_tick  = HAL_GetTick();
}

void Dbg_Vision_Print(void) {
    uint32_t now = HAL_GetTick();

    Dbg_Uart_VisionFlush(); /* 重开接收 + 丢积压：打印的是"从现在起"的状态 */

    Dbg_Printf("[K230] 链路 %s（USART2/CN3 @%u bps；判据 = %ums 内有校验通过帧）\n",
               (Dbg_Vision_LinkAlive() != 0u) ? "活着" : (s_seen != 0u) ? "断了（曾有帧）" : "无流量",
               (unsigned)DBG_CFG_VISION_BAUD, (unsigned)CFG_K230_LINK_DEAD_MS);

    if ((s_target.valid != 0u) && ((now - s_target_tick) <= CFG_K230_TARGET_FRESH_MS)) {
        float mx = 0.0f;
        float my = 0.0f;

        Dbg_Vision_ToMachine(&s_target, &mx, &my);
        Dbg_Printf("[K230] 缓存目标：类别 %u 形状 %u（%ums 前），视觉系 (", (unsigned)s_target.category,
                   (unsigned)s_target.shape, (unsigned)(now - s_target_tick));
        Dbg_Vision_PrintF1(s_target.x_mm);
        Dbg_Printf(", ");
        Dbg_Vision_PrintF1(s_target.y_mm);
        Dbg_Printf(")mm -> 机械系 (");
        Dbg_Vision_PrintF1(mx);
        Dbg_Printf(", ");
        Dbg_Vision_PrintF1(my);
        Dbg_Printf(")mm\n");
    } else {
        Dbg_Printf("[K230] 缓存目标：无\n");
    }

    Dbg_Printf("[K230] 收帧计数：可用目标 %u / 无目标 %u / 越界 %u"
               "（越界>0 = 上位机与主控口径不一致：类别须 1~%u、形状须 0~%u）\n",
               (unsigned)s_cnt_target, (unsigned)s_cnt_empty, (unsigned)s_cnt_bad,
               (unsigned)CFG_NUM_CATEGORIES, (unsigned)CFG_NUM_SHAPES);

    Dbg_Printf("[K230] 换算配置：机械X=ORIGIN_X+(%d)×帧.y  机械Y=ORIGIN_Y+(%d)×帧.x  原点=(%d,%d)%s\n",
               (int)CFG_VISION_SIGN_VY_TO_X, (int)CFG_VISION_SIGN_VX_TO_Y,
               (int)CFG_VISION_ORIGIN_X_MM, (int)CFG_VISION_ORIGIN_Y_MM,
               ((CFG_VISION_ORIGIN_X_MM == 0.0f) && (CFG_VISION_ORIGIN_Y_MM == 0.0f))
                   ? "  ⚠️ 未标定：机械坐标不可用（产品包络也会拒绝）"
                   : "");
}

uint8_t Dbg_Vision_Live(uint32_t seconds) {
    uint32_t t0;
    uint32_t next_report;
    uint32_t empty0 = s_cnt_empty;
    uint32_t bad0   = s_cnt_bad;
    uint32_t tgt0   = s_cnt_target;

    if ((seconds == 0u) || (seconds > 120u)) {
        seconds = 10u;
    }

    Dbg_Uart_VisionFlush(); /* 从现在开始算，不吃历史积压 */
    Dbg_Printf("[K230] 实时 %us：有目标帧逐条打；无目标每秒汇总一行。任意键中止\n", (unsigned)seconds);

    t0          = HAL_GetTick();
    next_report = t0 + 1000u;

    while ((HAL_GetTick() - t0) < (seconds * 1000u)) {
        uint32_t before = s_cnt_target;

        /* 中止检查：控制台来任意字节就停（读掉丢弃，不排进命令队列——同回零软急停口径） */
        while (Dbg_Uart_Available() > 0u) {
            (void)Dbg_Uart_ReadByte();
            Dbg_Printf("\n[K230] 实时被中止（收到控制台字节）\n");
            return 1u;
        }

        Dbg_Vision_Poll();

        if (s_cnt_target != before) { /* 来了新目标帧：立刻打 */
            float mx = 0.0f;
            float my = 0.0f;

            Dbg_Vision_ToMachine(&s_target, &mx, &my);
            Dbg_Printf("[K230] 类别 %u 形状 %u  视觉(", (unsigned)s_target.category,
                       (unsigned)s_target.shape);
            Dbg_Vision_PrintF1(s_target.x_mm);
            Dbg_Printf(", ");
            Dbg_Vision_PrintF1(s_target.y_mm);
            Dbg_Printf(") -> 机械(");
            Dbg_Vision_PrintF1(mx);
            Dbg_Printf(", ");
            Dbg_Vision_PrintF1(my);
            Dbg_Printf(")\n");
        }

        if ((int32_t)(HAL_GetTick() - next_report) >= 0) {
            next_report += 1000u;
            Dbg_Printf("[K230] … 无目标+%u 越界+%u（累计 目标%u/空%u/坏%u）\n",
                       (unsigned)(s_cnt_empty - empty0), (unsigned)(s_cnt_bad - bad0),
                       (unsigned)s_cnt_target, (unsigned)s_cnt_empty, (unsigned)s_cnt_bad);
        }
    }

    Dbg_Printf("[K230] 实时 %us 结束：本段目标 %u 帧（此前 %u）\n", (unsigned)seconds,
               (unsigned)(s_cnt_target - tgt0), (unsigned)tgt0);
    return 1u;
}

void Dbg_Vision_SimFeed(uint8_t category, uint8_t shape, float x_mm, float y_mm) {
    uint8_t f[12];
    uint8_t sum = 0u;
    int16_t x10 = (int16_t)(x_mm * 10.0f + ((x_mm >= 0.0f) ? 0.5f : -0.5f));
    int16_t y10 = (int16_t)(y_mm * 10.0f + ((y_mm >= 0.0f) ? 0.5f : -0.5f));

    f[0] = 0xAAu;
    f[1] = 0x55u;
    f[2] = 0x01u; /* 有目标 */
    f[3] = category;
    f[4] = shape; /* 形状 0~3（2026-10-01：0=默认方案、1~3=特调；不参与分类） */
    f[5] = 0u; /* hasQR */
    f[6] = 0u; /* 保留 */
    f[7] = (uint8_t)(x10 & 0xFF); /* X int16 小端，0.1mm */
    f[8] = (uint8_t)((x10 >> 8) & 0xFF);
    f[9] = (uint8_t)(y10 & 0xFF); /* Y int16 小端，0.1mm */
    f[10] = (uint8_t)((y10 >> 8) & 0xFF);
    for (uint8_t i = 0u; i < 11u; i++) {
        sum = (uint8_t)(sum + f[i]);
    }
    f[11] = sum;

    for (uint8_t i = 0u; i < sizeof(f); i++) {
        K230_Target_t frame;
        K230_Event_e  ev = K230_FeedByte(f[i], &frame); /* 走产品解析器的正门，不走旁路 */

        Dbg_Vision_ApplyEvent(ev, &frame, HAL_GetTick()); /* 入账口与 Poll 共用：缓存/计数都更新 */
    }

    Dbg_Printf("[K230] 已本地注入一帧：类别 %u 形状 %u 视觉(", (unsigned)category,
               (unsigned)shape);
    Dbg_Vision_PrintF1(x_mm);
    Dbg_Printf(", ");
    Dbg_Vision_PrintF1(y_mm);
    Dbg_Printf(")mm —— `k230` 看缓存与换算结果\n");
}

void Dbg_Vision_PrintStatusLine(void) {
    Dbg_Printf("[STAT] 视觉 链路=%s 目标帧=%u 无目标=%u 越界=%u（`k230` 看快照与换算）\n",
               (Dbg_Vision_LinkAlive() != 0u) ? "活" : "无流量", (unsigned)s_cnt_target,
               (unsigned)s_cnt_empty, (unsigned)s_cnt_bad);
}
