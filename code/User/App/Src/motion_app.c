/**
 * @file    motion_app.c
 * @brief   运动执行器：解算 -> 下发 -> 等待 + 记账可信度
 * @note    1) 守护任务：唯一处置撞限位的地方（急停 + 作废记账；判据 = 电平**连续压住**，
 *             不是 EXTI 沿闩锁——沿会把电流尖峰耦合出的假"撞限位"当真，见 Motion_ServeEvents）
 *          2) XY：回零标定一次 + 相对脉冲记账；只在"没被急停"时提交记账；
 *             绝对定位先过 Kinematics_XyInEnvelope
 *          3) 转盘：单向趋近消背隙；原点开关回零时是停止条件、定位时是每圈一次的索引沿
 *          竞态：s_xy_trusted / s_stop_xy_pending 由守护任务写、分拣任务读，故 volatile
 */
#include "motion_app.h"
#include "sort_config.h"
#include "motion_kinematics.h"
#include "zdt_motor.h"
#include "bsp_gpio.h"
#include "bsp_uart.h"
#include <FreeRTOS.h>
#include <task.h>

static float            s_x          = 0.0f; /* 记账坐标（原点 = 双限位触发处） */
static float            s_y          = 0.0f;
static volatile uint8_t s_xy_trusted = 0u;

static uint8_t  s_bin              = 0u;
static uint8_t  s_tray_trusted     = 0u; /* 仅分拣任务读写，无需 volatile */
static uint8_t  s_tray_moving      = 0u;
static int32_t  s_tray_pulses      = 0;  /* 自回零起累计已发脉冲（**有符号**：双向走时可为负） */
static int32_t  s_tray_pulses_mark = 0;
static uint32_t s_tray_index_mark  = 0u;
static uint32_t s_tray_start_tick  = 0u;
static uint8_t  s_tray_pending_bin = 0u;
/* ★ 撞限位判定状态（2026-10-08 起判据 = 电平连续压住，见 Motion_ServeEvents）：
   s_lim_low_since = 电平"连续压住"的起点（0 = 此刻没压着）；s_lim_fired = 本次压住已处置过 */
static volatile uint32_t s_lim_low_since = 0u;
static volatile uint8_t  s_lim_fired     = 0u;

static volatile uint8_t s_stop_xy_pending = 0u;
static volatile uint8_t s_homing_xy       = 0u; /* 1 = 回零中：压限位是预期动作 */

/* 单台探应答的"上电自检"已整链删除（2026-10-01）：本批驱动器对**广播不回包**
   （台架 2026-09-30 定稿，电控文档 §4.3"主控只发不收"），发使能帧等回包只会
   永远误报失败。上电使能 = Zdt_Init 逐台发一帧使能即完事；驱动器是否真的在，
   由回零流程和第一件分拣的自然结果暴露 */

/* 两轴一起停（CoreXY 单停一轴会让龙门歪），两帧都发出才返回 1。
   两帧都是广播停（Zdt_Stop 固定广播地址）；帧间留间隔——台架实测背靠背帧会被驱动器丢 */
static uint8_t Motion_StopXY(void) {
    uint8_t ok_a = Zdt_Stop(ZDT_ID_A);
    uint8_t ok_b;

    vTaskDelay(pdMS_TO_TICKS(CFG_ZDT_STOP_GAP_MS));
    ok_b = Zdt_Stop(ZDT_ID_B);

    return (ok_a && ok_b) ? 1u : 0u;
}

/* travel_mm<0 即退出限位 */
static void Motion_AxisDelta(uint8_t axis, float travel_mm, float *dx, float *dy) {
    if (axis == 0u) {
        *dx = travel_mm * CFG_HOME_DIR_X;
        *dy = 0.0f;
    } else {
        *dx = 0.0f;
        *dy = travel_mm * CFG_HOME_DIR_Y;
    }
}

static uint8_t Motion_AxisSend(Zdt_ID_e id, int32_t pulse, float rpm) {
    if (pulse == 0) {
        return 1u;
    }

    Zdt_DIR_e dir       = (pulse > 0) ? ZDT_DIR_CW : ZDT_DIR_CCW;
    uint32_t  abs_pulse = (uint32_t)((pulse > 0) ? pulse : -pulse);

    return Zdt_MoveRelative(id, dir, (uint16_t)rpm, CFG_ZDT_ACCEL, abs_pulse);
}

/* 解算并下发一段 XY 合成运动；返回 0 表示指令未全部生效 */
static uint8_t Motion_SendXY(float dx, float dy, float speed_mm_s, uint32_t *time_ms) {
    int32_t  pulse_a = 0;
    int32_t  pulse_b = 0;
    float    rpm_a   = 0.0f;
    float    rpm_b   = 0.0f;
    uint32_t est_ms  = 0u;

    Kinematics_XYSolve(dx, dy, speed_mm_s, &pulse_a, &pulse_b, &rpm_a, &rpm_b, &est_ms);

    uint8_t ok_a = Motion_AxisSend(ZDT_ID_A, pulse_a, rpm_a);
    /* ★ 台架实测（09-25/28）：A/B 背靠背下发（≈1.1ms）驱动器丢帧——归零"跑斜线"的
       根源之一。台架口径 3ms 等应答+7ms 空闲；产品暂不读应答，取 10ms 纯间隔 */
    vTaskDelay(pdMS_TO_TICKS(CFG_ZDT_FRAME_GAP_MS));
    uint8_t ok_b = Motion_AxisSend(ZDT_ID_B, pulse_b, rpm_b);

    *time_ms = est_ms;
    if (!ok_a || !ok_b) {
        /* 只发出去一轴 = 那一轴已经动了，随后停止的位置没人知道（bsp_uart.h 的发送契约
           要求调用方作废记账）。不置这一位，接下来每一件都按错误基准做相对移动 */
        (void)Motion_StopXY(); /* 只有一轴动起来会走歪 */
        s_xy_trusted = 0u;
        return 0u;
    }
    return 1u;
}

static uint8_t Motion_WaitLevel(uint8_t axis, uint8_t want_active, uint32_t timeout_ms) {
    uint32_t t0 = HAL_GetTick();

    while ((HAL_GetTick() - t0) < timeout_ms) {
        uint8_t active = (axis == 0u) ? Bsp_Gpio_LimitX() : Bsp_Gpio_LimitY();

        if (active == want_active) {
            return 1u;
        }
        vTaskDelay(pdMS_TO_TICKS(2u));
    }
    return 0u;
}

/* 撞限位处置：真撞 -> 急停 + 作废记账（两者必须同一次做，见下）；转盘索引沿 -> 只计数
   ★ 2026-10-08 判据换成"电平连续压住"（第二次定案，前一版"沿闩锁 + 停后确认"有硬伤）：
   现场现象一"抓完之后有时自己回零校准"：EXTI 只要"武装状态下的一次下降沿"就锁存，
   舵机/电机换向的电流尖峰耦合进限位线同样能造出一次假"撞限位"；旧代码据此立即
   急停 + 作废记账 ⇒ 分拣任务下一圈重新回零（现场看到的就是那次白跑的回零校准）。
   现场现象二（前一版引入的硬伤，用户报"一上来连夹持点都到不了"）：把"停止帧照旧立即发、
   记账晚 40ms 再作废"分开做——毛刺判定为假之后记账被**保留**，可运动其实已被停帧截在半路
   ⇒ 记账声称"已到位"、实物停在半路 ⇒ 坐标系静默平移（每来一次毛刺漂一次，越抓越歪、
   最后朝开关顶），比"白跑一次回零"严重得多。
   ⇒ 判据改成物理事实：**真撞限位时开关一直被龙门压住（几百 ms 起），电气毛刺 µs~ms 级
      一晃就回**。于是：X 或 Y 电平**连续压住 ≥ CFG_LIMIT_CONFIRM_MS**（1ms 任务周期采样）
      才认真撞 ⇒ 急停 + 作废记账**一起**发生（停过的运动位置未知，只停不作废必然漂坐标）；
      窗内电平已松开 = 毛刺 ⇒ **不停、不作废**，本件照常走完（那串假回零就此消失）。
   代价只有过冲：真撞的停止帧比"边沿即停"晚这个窗发出（200mm/s 下 ≈2mm，见配置注释） */
static void Motion_ServeEvents(void) {
    uint32_t now = HAL_GetTick();

    /* 沿闩锁不再作判据（判据见上），清掉即可；保留 EXTI 闩锁本身只为"按压发生过"这条
       诊断语义（与台架 dbg 的 lim 计数同源） */
    (void)Bsp_Gpio_TakeEvents();

    uint8_t pressed = ((Bsp_Gpio_LimitX() != 0u) || (Bsp_Gpio_LimitY() != 0u)) ? 1u : 0u;

    /* 回零期间压限位是预期动作，收尾的退离相也还压着（见 Motion_HomeXY 的掩护段）：
       不判定、也不留状态，否则"刚归零完就被判真撞" */
    if (s_homing_xy != 0u) {
        s_lim_low_since = 0u;
        s_lim_fired     = 0u;
        return;
    }

    if (pressed == 0u) {
        s_lim_low_since = 0u; /* 松开：清连续计时，并允许下一次判定 */
        s_lim_fired     = 0u;
    } else if (s_lim_fired == 0u) {
        if (s_lim_low_since == 0u) {
            s_lim_low_since = now;
        } else if ((now - s_lim_low_since) >= CFG_LIMIT_CONFIRM_MS) {
            s_lim_fired       = 1u;
            s_xy_trusted      = 0u; /* 记账作废 */
            s_stop_xy_pending = 1u; /* 急停（与作废同一次处置，见函数头） */
        }
    }

    if (s_stop_xy_pending != 0u && Motion_StopXY()) {
        s_stop_xy_pending = 0u; /* 发不出去就 1ms 后重试 */
    }
}

void Motion_SafetyTask(void *argument) {
    (void)argument;

    for (;;) {
        Bsp_Gpio_Poll(); /* 限位消抖/闩锁的松开确认：连续 40ms 松开才武装下一次按压判定 */
        Bsp_Uart_Poll(); /* 视觉线噪声预算的重开试探 + UE/RE/TE 自愈（bsp_uart.c） */
        Motion_ServeEvents();
        vTaskDelay(pdMS_TO_TICKS(CFG_SAFETY_PERIOD_MS));
    }
}

void Motion_Init(void) {
    s_x               = 0.0f;
    s_y               = 0.0f;
    s_xy_trusted      = 0u;
    s_bin             = (uint8_t)CFG_TRAY_ZERO_BIN;
    s_tray_pulses     = 0;
    s_tray_trusted    = 0u;
    s_tray_moving     = 0u;
    s_stop_xy_pending  = 0u;
    s_homing_xy        = 0u;
    s_lim_low_since    = 0u;
    s_lim_fired        = 0u;
    (void)Bsp_Gpio_TakeEvents();
}

uint8_t Motion_XyIsTrusted(void) {
    return s_xy_trusted;
}

uint8_t Motion_MoveXYBlocking(float x_mm, float y_mm, float speed_mm_s) {
    if (!s_xy_trusted) {
        return 0u; /* 记账不可信时绝不能做绝对定位，交给调用方重新回零 */
    }
    /* 软件原点角在远离限位的一侧，触发点 = (CFG_HOME_X_MAX_MM, CFG_HOME_Y_MAX_MM)
       （09-28 台架实锤开关在最大行程端）；越界目标一律拒绝、一字节都不发，
       由调用方作废本件，绝不留下"半程撞限位"的状态 */
    if (!Kinematics_XyInEnvelope(x_mm, y_mm)) {
        return 0u;
    }

    uint32_t time_ms = 0u;

    if (!Motion_SendXY(x_mm - s_x, y_mm - s_y, speed_mm_s, &time_ms)) {
        return 0u;
    }
    if (time_ms > 0u) {
        vTaskDelay(pdMS_TO_TICKS(time_ms));
    }

    /* 只有"按估算走完且途中没撞限位"才允许提交记账 */
    if (!s_xy_trusted) {
        return 0u;
    }
    s_x = x_mm;
    s_y = y_mm;
    return 1u;
}

/* 单轴回零：退出窗口 -> 粗压 -> 脱开 -> 慢压取触发沿 */
static uint8_t Motion_HomeAxis(uint8_t axis) {
    float    dx      = 0.0f;
    float    dy      = 0.0f;
    uint32_t time_ms = 0u;

    for (uint8_t attempt = 0u; attempt < 2u; attempt++) {
        /* 0) 一上来就压在限位上时必须先退出，否则"已触发"会被当成回零成功 */
        if (!Motion_WaitLevel(axis, 0u, 200u)) {
            Motion_AxisDelta(axis, -(CFG_BACKOFF_MM * 4.0f + 10.0f), &dx, &dy);
            if (!Motion_SendXY(dx, dy, CFG_HOME_SLOW_MM_S, &time_ms)) {
                return 0u;
            }
            if (!Motion_WaitLevel(axis, 0u, time_ms + CFG_HOME_WAIT_EXTRA_MS)) {
                (void)Motion_StopXY();
                continue; /* 退不出窗口 = 开关卡住/接线故障 */
            }
            (void)Motion_StopXY();
        }

        /* 1) 粗压：估算行程给足余量，等触发 */
        Motion_AxisDelta(axis, CFG_HOME_TRAVEL_MM, &dx, &dy);
        if (!Motion_SendXY(dx, dy, CFG_HOME_SPEED_MM_S, &time_ms)) {
            return 0u;
        }
        if (!Motion_WaitLevel(axis, 1u, time_ms + CFG_HOME_WAIT_EXTRA_MS)) {
            (void)Motion_StopXY();
            continue;
        }
        (void)Motion_StopXY(); /* 触发即停，不留过压 */
        vTaskDelay(pdMS_TO_TICKS(50u));

        /* 2) 必须确认真的脱开才能去取第 3 步的沿：脱开量不够时第 3 步会"在仍压着时
           立刻触发"，原点被定义在开关窗口内任意一点，之后每次定位都带固定偏移——
           这比回零失败更危险，因为它看起来是成功的 */
        Motion_AxisDelta(axis, -CFG_BACKOFF_MM, &dx, &dy);
        if (!Motion_SendXY(dx, dy, CFG_HOME_SPEED_MM_S, &time_ms)) {
            return 0u;
        }
        if (!Motion_WaitLevel(axis, 0u, time_ms + CFG_HOME_WAIT_EXTRA_MS)) {
            (void)Motion_StopXY();
            continue;
        }
        (void)Motion_StopXY();

        /* 3) 慢速同方向再压：触发沿才是重复性最好的原点定义 */
        Motion_AxisDelta(axis, CFG_BACKOFF_MM * 2.0f + 5.0f, &dx, &dy);
        if (!Motion_SendXY(dx, dy, CFG_HOME_SLOW_MM_S, &time_ms)) {
            return 0u;
        }
        if (!Motion_WaitLevel(axis, 1u, time_ms + CFG_HOME_WAIT_EXTRA_MS)) {
            (void)Motion_StopXY();
            continue;
        }
        (void)Motion_StopXY();
        return 1u;
    }
    return 0u;
}

uint8_t Motion_HomeXY(void) {
    s_xy_trusted = 0u;
    s_homing_xy  = 1u; /* 独占两轴（含收尾退离相，见下） */

    uint8_t ok_x = Motion_HomeAxis(0u);
    uint8_t ok_y = Motion_HomeAxis(1u);

    /* ★ 触发点在工作坐标系里 = (CFG_HOME_X_MAX_MM, CFG_HOME_Y_MAX_MM)（开关装在最大
       行程端，09-28 台架实锤；此前误置 0,0 会把整个工作区映射到限位之外） */
    s_x          = CFG_HOME_X_MAX_MM;
    s_y          = CFG_HOME_Y_MAX_MM;
    s_xy_trusted = (ok_x && ok_y) ? 1u : 0u;

    /* ★ 退离动作必须在"回零掩护"里做完（s_homing_xy 仍 = 1）：慢压定零结束时两轴都还
       压在开关上，要等退离开关复位行程走完电平才回高——这段（几十 ms）里"电平压住"是
       正常状态，一旦让守护任务的撞限位判定跑起来就会判真撞 + 记账当场作废，回零永远接不上
       （判据换成"电平连续压住"之后这条尤其关键）。退离走完 = 两轴都在开关外（3mm 足够，
       归零自己就验证过）。顺序不可换：先提交坐标与可信标志 →（掩护中）退离 → 清沿 + 解除掩护 */
    uint8_t ok_park = 0u;

    if (s_xy_trusted) {
        ok_park = Motion_MoveXYBlocking(CFG_PARK_X_MM, CFG_PARK_Y_MM, CFG_HOME_SPEED_MM_S);
    }

    (void)Bsp_Gpio_TakeEvents(); /* 回零自己压出的沿（含退离时松开那一下）到此作废 */
    s_homing_xy = 0u;

    if ((!s_xy_trusted) || (ok_park == 0u)) {
        s_xy_trusted = 0u; /* 连退到待机点都没成 = 位置未知 */
        return 0u;
    }
    return 1u;
}

/* 转盘按电机轴口径相对走 pulses 个脉冲 */
static uint8_t Motion_TrayMovePulses(Zdt_DIR_e dir, uint32_t pulses, uint16_t rpm) {
    return Zdt_MoveRelative(ZDT_ID_TRAY, dir, rpm, CFG_ZDT_ACCEL, pulses);
}

#define TRAY_DIR_FWD  ((Zdt_DIR_e)(uint8_t)CFG_TRAY_DIR_FORWARD)
#define TRAY_DIR_BACK ((Zdt_DIR_e)(uint8_t)(1u - (uint8_t)CFG_TRAY_DIR_FORWARD))

static uint8_t Motion_WaitTrayLevel(uint8_t want_active, uint32_t timeout_ms) {
    uint32_t t0 = HAL_GetTick();

    while ((HAL_GetTick() - t0) < timeout_ms) {
        if (Bsp_Gpio_TrayOrigin() == want_active) {
            return 1u;
        }
        vTaskDelay(pdMS_TO_TICKS(2u));
    }
    return 0u;
}

uint8_t Motion_TrayHome(void) {
    s_tray_trusted = 0u;
    s_tray_moving  = 0u;

    /* 可能是在一笔定位中途来重新回零的：先停稳 */
    (void)Zdt_Stop(ZDT_ID_TRAY);
    vTaskDelay(pdMS_TO_TICKS(50u));

    for (uint8_t attempt = 0u; attempt < 2u; attempt++) {
        /* 0) 凸块可能正压在开关上：先前进退出窗口，才取得到干净的上升沿 */
        if (Bsp_Gpio_TrayOrigin()) {
            if (!Motion_TrayMovePulses(TRAY_DIR_FWD, CFG_TRAY_BACKOFF_PULSES * 2u,
                                       CFG_TRAY_APPROACH_RPM)) {
                return 0u;
            }
            if (!Motion_WaitTrayLevel(0u, CFG_TRAY_APPROACH_TIMEOUT_MS)) {
                (void)Zdt_Stop(ZDT_ID_TRAY);
                continue;
            }
            (void)Zdt_Stop(ZDT_ID_TRAY);
            /* ★ 帧间隙纪律（2026-10-01 台架实测背靠背 ≈1.1ms 丢帧，所有同线相邻帧距
               ≥10ms）：台架相位间靠 50ms 等待+控制台打印天然满足，产品无打印必须显式
               给——这里漏了速度帧会被丢，回零在相位1白等一整圈才超时重试 */
            vTaskDelay(pdMS_TO_TICKS(50u));
        }

        /* 1) 粗寻标：超时必须覆盖一整圈 */
        if (!Zdt_SpeedMode(ZDT_ID_TRAY, (Zdt_DIR_e)CFG_TRAY_DIR_FORWARD, CFG_TRAY_SEEK_RPM,
                           CFG_ZDT_ACCEL)) {
            return 0u;
        }
        if (!Motion_WaitTrayLevel(1u, CFG_TRAY_HOME_TIMEOUT_MS)) {
            (void)Zdt_Stop(ZDT_ID_TRAY);
            continue;
        }
        (void)Zdt_Stop(ZDT_ID_TRAY);
        vTaskDelay(pdMS_TO_TICKS(50u));

        /* 2) 必须反向脱开：旋转轴只有一条沿，继续正转会把它甩到开关身后，
           慢速再压就得再等一整圈（远超 APPROACH 超时）。
           ★ 台架实测（2026-10-01）凸块压住开关的弧宽 > 600 脉冲（1.8°）：退一份没脱开
           就**原地再加退一份**（最多 4 份 = 7.2°），而不是作废整次尝试、害下一轮
           "退出相+重新粗寻标"多转一整圈。
           ★ 过冲保护（2026-10-01 二次提速后实机现象）：480rpm 撞停的过冲可能把凸块
           直接送过开关作用弧——停车点已在开关身后、电平=松开，此时"反向退到松开"会
           立即**假成功**，慢压相就得从另一侧追将近一整圈（用户看到的"第二圈"）。
           处置：停车点已松开时**先反向慢转回找窗口**（上限 CFG_TRAY_REENTRY_PULSES，
           压上即停、再走常规退出）；走满没压上 = 触发是噪声假沿（停车点其实在窗口
           之前，回找的这一段只是把它再退远一点），直接进慢压相由正向慢压找真正的沿。
           两条路都不需要降速 */
        {
            uint8_t cleared      = 0u;
            uint8_t skip_release = 0u; /* 1 = 假沿/停在窗口前：无窗口可退出 */

            if (Bsp_Gpio_TrayOrigin() == 0u) { /* 停车就已经是松开 = 不在窗口内 */
                if (!Motion_TrayMovePulses(TRAY_DIR_BACK, CFG_TRAY_REENTRY_PULSES,
                                           CFG_TRAY_APPROACH_RPM)) {
                    return 0u;
                }
                if (Motion_WaitTrayLevel(1u, Kinematics_TrayMoveTimeMs(
                                                 CFG_TRAY_REENTRY_PULSES,
                                                 (float)CFG_TRAY_APPROACH_RPM)) != 0u) {
                    (void)Zdt_Stop(ZDT_ID_TRAY); /* 回进窗口：转下面的常规退出 */
                } else {
                    (void)Zdt_Stop(ZDT_ID_TRAY);
                    skip_release = 1u; /* 没找到窗口：停车点在窗口前，别在这里耗 */
                }
            }
            if (skip_release == 0u) {
                for (uint8_t k = 0u; k < 4u; k++) {
                    if (!Motion_TrayMovePulses(TRAY_DIR_BACK, CFG_TRAY_BACKOFF_PULSES,
                                               CFG_TRAY_APPROACH_RPM)) {
                        return 0u;
                    }
                    if (!Motion_WaitTrayLevel(0u, CFG_TRAY_APPROACH_TIMEOUT_MS)) {
                        (void)Zdt_Stop(ZDT_ID_TRAY);
                        continue; /* 这一份没退出来，再加退一份 */
                    }
                    (void)Zdt_Stop(ZDT_ID_TRAY);
                    cleared = 1u;
                    break;
                }
                if (cleared == 0u) {
                    continue; /* 4 份（7.2°）仍压着：开关粘死/凸块异常，换一次尝试 */
                }
            }
        }

        /* ★ 相位2→3 必须留帧间隙（台架 dbg_app.c:1339 同款 50ms，2026-10-01 补）：
           反向脱开的停止帧和慢压的速度模式帧背靠背（≈1.5ms）会被驱动器丢掉后者，
           表现 = "撞开关→弹回来（脱开成功）→托盘再也不动"，1.5s 超时作废重试，
           每次重试都死在同一处——用户看到的"转一圈→撞→弹回→愣住"循环即此 */
        vTaskDelay(pdMS_TO_TICKS(50u));

        /* 3) 慢速同方向再压：这条沿就是索引零位（与后续定位同方向，背隙已吃进） */
        if (!Zdt_SpeedMode(ZDT_ID_TRAY, (Zdt_DIR_e)CFG_TRAY_DIR_FORWARD, CFG_TRAY_APPROACH_RPM,
                           CFG_ZDT_ACCEL)) {
            return 0u;
        }
        if (!Motion_WaitTrayLevel(1u, CFG_TRAY_APPROACH_TIMEOUT_MS)) {
            (void)Zdt_Stop(ZDT_ID_TRAY);
            continue;
        }
        (void)Zdt_Stop(ZDT_ID_TRAY);
        vTaskDelay(pdMS_TO_TICKS(50u));

        /* 4) 对格偏移（2026-10-01 台架实测）：凸块索引零位与格中心差 CFG_TRAY_HOME_OFFSET_DEG，
           定零后**正向转过去**才是盒1 正对投放位（同 XY"归零后退 3mm 驻留"的思路）。
           ★ 脉冲记账的原点仍 = 索引沿：s_tray_pulses 从偏移量起算，索引沿复核
             （TrayIndexCrossings 按 PER_REV 分界）与盒号折算（TrayBinOf）都保持正确；
           ★ 附带：驻留位在开关外，不长期压着原点开关（无磨损、无沿计数噪声） */
        {
            uint32_t off_pulses =
                (uint32_t)CFG_TRAY_HOME_OFFSET_DEG * CFG_TRAY_PULSES_PER_REV / 360u;

            if (off_pulses != 0u) {
                if (!Motion_TrayMovePulses(TRAY_DIR_FWD, off_pulses, CFG_TRAY_APPROACH_RPM)) {
                    return 0u;
                }
                vTaskDelay(
                    pdMS_TO_TICKS(Kinematics_TrayMoveTimeMs(off_pulses, (float)CFG_TRAY_APPROACH_RPM)));
                s_tray_pulses = (int32_t)off_pulses; /* 记账原点 = 索引沿，驻留位 = 零位 + 偏移 */
            } else {
                s_tray_pulses = 0u;
            }
        }

        /* 5) 以此为累计脉冲与索引计数的共同基准 */
        s_bin          = (uint8_t)CFG_TRAY_ZERO_BIN;
        s_tray_trusted = 1u;
        return 1u;
    }

    s_tray_trusted = 0u;
    return 0u;
}

uint8_t Motion_TrayIsTrusted(void) {
    return s_tray_trusted;
}

uint8_t Motion_TrayBin(void) {
    return s_bin;
}

uint32_t Motion_TrayGotoBin(uint8_t bin) {
    if (!s_tray_trusted || bin >= CFG_TRAY_BINS) {
        return 0u;
    }

    /* 盒号差 -> 最短路径（2026-10-08 起默认允许反向，见 CFG_TRAY_ALLOW_REVERSE）：
       delta 先算"正转要几格"（0..5），超过半圈就折成负值（反向最近）——6 格时 |delta| ≤ 3，
       恰好 3 格（180°）的平局仍走正转，与老口径一致。
       前提标定不变：CFG_TRAY_DIR_FORWARD 必须使"正转 -> 盒号递增"，反了会整圈镜像放错 */
    int32_t delta = (int32_t)((bin + CFG_TRAY_BINS - s_bin) % CFG_TRAY_BINS);

    if (delta > (int32_t)(CFG_TRAY_BINS / 2)) {
        delta -= (int32_t)CFG_TRAY_BINS;
    }
#if CFG_TRAY_ALLOW_REVERSE == 0u
    if (delta < 0) {
        delta += (int32_t)CFG_TRAY_BINS; /* 老口径：单向正转消背隙（跨零位自然多转一圈） */
    }
#endif

    if (delta == 0) {
        s_tray_moving = 0u;
        return 0u; /* 已对位 */
    }

    uint32_t pulses = (uint32_t)((delta > 0) ? delta : -delta) * CFG_TRAY_PULSES_PER_BIN;
    Zdt_DIR_e dir   = (delta > 0) ? TRAY_DIR_FWD : TRAY_DIR_BACK;

    s_tray_pulses_mark = s_tray_pulses;
    s_tray_index_mark  = Bsp_Gpio_TrayIndexCount();
    s_tray_start_tick  = HAL_GetTick();

    if (!Motion_TrayMovePulses(dir, pulses, CFG_TRAY_SPEED_RPM)) {
        s_tray_trusted = 0u; /* 指令没发出去 = 转盘根本不会动 */
        return 0u;
    }

    s_tray_pulses += (delta > 0) ? (int32_t)pulses : -(int32_t)pulses;
    s_tray_pending_bin = bin;
    s_tray_moving      = 1u;

    return Kinematics_TrayMoveTimeMs(pulses, (float)CFG_TRAY_SPEED_RPM);
}

uint8_t Motion_TrayFinish(uint32_t est_ms) {
    if (!s_tray_moving) {
        return s_tray_trusted;
    }
    s_tray_moving = 0u;

    uint32_t elapsed_ms = HAL_GetTick() - s_tray_start_tick;

    if (est_ms > elapsed_ms) {
        vTaskDelay(pdMS_TO_TICKS(est_ms - elapsed_ms));
    }

    /* 索引复核：本笔理论"经过几个凸块"必须与实测按压数完全一致，对不上 = 减速箱丢步/堵转。
       双向都成立（反向经过的凸块同样压一次开关，见 Kinematics_TrayCrossingsBetween）。
       检出能力上界（仿真实测）：一圈只有一个沿，只有把累计脉冲推过/推离整圈边界的那一笔
       才看得见，故上界是一段约一圈的【行程】；漏检窗口内落点偏差理论上界为半圈 = 3 格。
       本模块是滑移【探测器】不是【补偿器】：检出的靠作废重回零，检不出的靠定期重新回零兜底 */
    uint32_t expect = Kinematics_TrayCrossingsBetween(s_tray_pulses_mark, s_tray_pulses);
    uint32_t actual = Bsp_Gpio_TrayIndexCount() - s_tray_index_mark;

    if (expect != actual) {
        s_tray_trusted = 0u;
        return 0u;
    }

    /* 第二条是记账之间的交叉验证：按符号累加的盒号 vs 按累计脉冲折算的盒号 */
    uint8_t pulse_bin =
        (uint8_t)((CFG_TRAY_ZERO_BIN + Kinematics_TrayBinOf(s_tray_pulses)) % CFG_TRAY_BINS);

    if (pulse_bin != s_tray_pending_bin) {
        s_tray_trusted = 0u;
        return 0u;
    }

    s_bin = s_tray_pending_bin;
    return 1u;
}