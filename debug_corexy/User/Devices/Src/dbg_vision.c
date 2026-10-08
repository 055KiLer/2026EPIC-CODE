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

uint32_t Dbg_Vision_BadCount(void) {
    return s_cnt_bad;
}

/* ★ 不看新鲜度的缓存快照（见 dbg_vision.h）：read 的"清缓存前本来有没有目标"诊断入口 */
uint8_t Dbg_Vision_TargetSnapshot(K230_Target_t *out, uint32_t *age_ms) {
    if (s_target.valid == 0u) {
        return 0u;
    }
    if (out != NULL) {
        *out = s_target;
    }
    if (age_ms != NULL) {
        *age_ms = HAL_GetTick() - s_target_tick;
    }
    return 1u;
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

/* 逐帧原样打印（给 `k230 raw` 用）：hex 12 字节 + 解码 + 校验判决。
   @param force 1 = 无条件逐帧打（头 3 秒，先看清帧结构）；0 = 只打"有信息量"的帧
                （目标帧/异常帧）；状态 0x00 的帧交给每秒汇总，免得刷屏
   @retval 帧类别：1=目标帧（含越界）2=状态 00（本帧无目标）3=校验错 4=其它状态 */
static uint8_t Dbg_Vision_RawFrame(const uint8_t *f, uint8_t force, uint32_t *n_bad,
                                   uint32_t *n_none) {
    uint8_t sum = 0u;
    uint8_t kind;

    for (uint8_t i = 0u; i < 11u; i++) {
        sum = (uint8_t)(sum + f[i]);
    }

    /* 先分类计数（与"打不打"无关），再决定打印 */
    if (sum != f[11]) {
        (*n_bad)++;
        kind = 3u;
    } else if (f[2] == 0x00u) {
        (*n_none)++;
        kind = 2u;
    } else if (f[2] == 0x01u) {
        kind = 1u;
    } else {
        kind = 4u;
    }

    if ((force == 0u) && (kind == 2u)) {
        return kind;
    }

    Dbg_Printf("[RAW] ");
    for (uint8_t i = 0u; i < 12u; i++) {
        Dbg_Printf("%02X ", f[i]);
    }

    if (kind == 3u) {
        Dbg_Printf("| 校验错（前 11 字节累加 %02X ≠ 帧尾 %02X）——帧长/累加范围与 §4.1 不一致\n",
                   (unsigned)sum, (unsigned)f[11]);
        return kind;
    }
    if (kind == 2u) {
        Dbg_Printf("| 状态 00 = 本帧无目标（上位机在说：没有可抓的东西）\n");
        return kind;
    }
    if (kind == 4u) {
        Dbg_Printf("| 状态 %02X 既不是 00 也不是 01 —— 本机当无目标处理\n", (unsigned)f[2]);
        return kind;
    }

    {
        int16_t       x10 = (int16_t)((uint16_t)f[8] << 8 | f[7]);
        int16_t       y10 = (int16_t)((uint16_t)f[10] << 8 | f[9]);
        K230_Target_t t;
        float         mx = 0.0f;
        float         my = 0.0f;

        t.valid    = 1u;
        t.category = f[3];
        t.shape    = f[4];
        t.has_qr   = f[5];
        t.x_mm     = (float)x10 * 0.1f;
        t.y_mm     = (float)y10 * 0.1f;
        Dbg_Vision_ToMachineImpl(&t, &mx, &my);

        if ((t.category >= 1u) && (t.category <= (uint8_t)CFG_NUM_CATEGORIES) &&
            (t.shape <= (uint8_t)CFG_NUM_SHAPES)) {
            Dbg_Printf("| 目标帧 类别 %u 形状 %u [可用]  帧XY(", (unsigned)t.category,
                       (unsigned)t.shape);
        } else {
            Dbg_Printf("| 目标帧 类别 %u 形状 %u [★越界→本机按设计丢弃]  帧XY(", (unsigned)t.category,
                       (unsigned)t.shape);
        }
        Dbg_Vision_PrintF1(t.x_mm);
        Dbg_Printf(", ");
        Dbg_Vision_PrintF1(t.y_mm);
        Dbg_Printf(") -> 机械(");
        Dbg_Vision_PrintF1(mx);
        Dbg_Printf(", ");
        Dbg_Vision_PrintF1(my);
        Dbg_Printf(")\n");
    }
    return kind;
}

/* ★ `k230 raw [秒]`：把线上字节按帧原样打出来（hex + 解码 + 校验判决）——线上真相工具。
   ★ 旁路观察：**不喂解析器、不动缓存与计数**（`live`/快照的计数不受影响），也不消费控制台命令。
   用途：当 `read`/产品报"没有可用的目标帧"时，用它区分三种线级可能：
     ① 线上只有状态 0x00 的帧 ⇒ 上位机在说"本帧无目标"（识别/目标挑选）
     ② 线上有目标帧、但类别/形状越界 ⇒ 本机按设计丢弃（宁可不抓）；hex 里能直接看到那个数字
     ③ 线上是乱字节/校验不过/根本没有字节 ⇒ 线、波特率、或上位机 TX（再用并口 TTL 复核）
   头 3 秒逐帧全打（看清帧结构），之后只打目标帧/异常帧 + 每秒按状态分类的汇总。
   任意键中止（读掉的字节丢弃，不排进命令队列——同 live 口径） */
uint8_t Dbg_Vision_Raw(uint32_t seconds) {
    uint8_t  st  = 0u; /* 0=找 AA、1=等 55、2=收载荷 */
    uint8_t  idx = 0u;
    uint8_t  buf[12];
    uint32_t t0;
    uint32_t next_report;
    uint32_t n_frame = 0u, n_tgt = 0u, n_none = 0u, n_bad = 0u, n_other = 0u;
    uint32_t n_bytes = 0u, n_stray = 0u;
    uint32_t s_frame = 0u, s_tgt = 0u, s_none = 0u, s_bad = 0u, s_other = 0u, s_bytes = 0u;

    if ((seconds == 0u) || (seconds > 120u)) {
        seconds = 10u;
    }

    Dbg_Uart_VisionFlush(); /* 从现在开始算，不吃历史积压 */
    Dbg_Printf("[RAW] 视觉线原始帧 %us：逐帧 hex + 解码（旁路观察，不动缓存/计数）。任意键中止\n",
               (unsigned)seconds);

    t0          = HAL_GetTick();
    next_report = t0 + 1000u;

    for (;;) {
        if (Dbg_Uart_Available() > 0u) { /* 只排空不执行：中止观察不算软急停 */
            while (Dbg_Uart_Available() > 0u) {
                (void)Dbg_Uart_ReadByte();
            }
            Dbg_Printf("[RAW] 已中止\n");
            return 1u;
        }

        while (Dbg_Uart_VisionAvailable() > 0u) {
            uint8_t b = Dbg_Uart_VisionReadByte();

            n_bytes++;
            s_bytes++;

            if (st == 0u) {
                if (b == 0xAAu) {
                    buf[0] = b;
                    st     = 1u;
                } else {
                    n_stray++; /* 非帧头字节：只计数（帧错位/波特率不对时这里会涨） */
                }
            } else if (st == 1u) {
                if (b == 0x55u) {
                    buf[1] = b;
                    idx    = 2u;
                    st     = 2u;
                } else if (b == 0xAAu) {
                    /* 连续 AA：留在本状态重判（同解析器口径） */
                } else {
                    st = 0u;
                    n_stray++;
                }
            } else {
                buf[idx++] = b;
                if (idx >= 12u) {
                    uint8_t kind = Dbg_Vision_RawFrame(buf, ((HAL_GetTick() - t0) < 3000u) ? 1u : 0u,
                                                       &n_bad, &n_none);

                    st = 0u;
                    n_frame++;
                    s_frame++;
                    if (kind == 1u) {
                        n_tgt++;
                        s_tgt++;
                    } else if (kind == 4u) {
                        n_other++;
                        s_other++;
                    } else if (kind == 3u) {
                        s_bad++;
                    } else {
                        s_none++;
                    }
                }
            }
        }

        if ((int32_t)(HAL_GetTick() - next_report) >= 0) {
            next_report += 1000u;
            Dbg_Printf("[RAW] …本秒 帧 %u（目标 %u / 无目标 00 %u / 其它状态 %u / 校验错 %u）字节 %u\n",
                       (unsigned)s_frame, (unsigned)s_tgt, (unsigned)s_none, (unsigned)s_other,
                       (unsigned)s_bad, (unsigned)s_bytes);
            s_frame = 0u;
            s_tgt   = 0u;
            s_none  = 0u;
            s_other = 0u;
            s_bad   = 0u;
            s_bytes = 0u;
        }

        if ((HAL_GetTick() - t0) > (seconds * 1000u)) {
            Dbg_Printf("[RAW] %us 结束：帧 %u（目标 %u / 无目标 00 %u / 其它状态 %u / 校验错 %u）"
                       "字节 %u 散字节 %u\n",
                       (unsigned)seconds, (unsigned)n_frame, (unsigned)n_tgt, (unsigned)n_none,
                       (unsigned)n_other, (unsigned)n_bad, (unsigned)n_bytes, (unsigned)n_stray);
            if (n_frame == 0u) {
                Dbg_Printf("[RAW] ⚠️ 一帧都没组出来（散字节 %u）——先查线/波特率/上位机 TX\n",
                           (unsigned)n_stray);
            } else if (n_bad == n_frame) {
                Dbg_Printf("[RAW] ⚠️ 全是校验错——帧在线上，但帧长或累加范围与 §4.1 不一致\n");
            } else if (n_none == n_frame) {
                Dbg_Printf("[RAW] ⚠️ 全是本帧无目标（状态 00）——帧到得好好的，是上位机没把物块当目标\n");
            } else if (n_tgt > 0u) {
                Dbg_Printf("[RAW] 线上有目标帧（见上面 [RAW] 目标帧 行）——核它的类别/形状与坐标\n");
            }
            return 1u;
        }
    }
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
