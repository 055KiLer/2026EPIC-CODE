/**
 * @file    sort_task.c
 * @brief   分拣任务（对应电控文档 §3.2 主分拣循环）
 * @note    循环骨架：标定可信? -> [到读帧位（投放点上方）＋静默期] -> **读事件（领目标 +
 *          给上一件的到账数量复核）** -> [转盘预定位(与 XY 并行) + XY 移动] -> 转盘索引复核
 *          -> 抓取 -> 到投放点 + 松爪 -> （就停在读帧位，等下一次读事件）下一件
 *
 *          四类"不干净"的处置：
 *            运动不可信（撞限位/丢步/发不出去）-> 本件作废，下轮开头自动重新回零
 *            视觉链路断 -> 停在待机点等恢复，这不等于"托盘已空"，绝不能报完成
 *            刷屏无回执 -> 停在原地按同一序号重投（题目红线：显示完成前不得分拣下一件）
 *            数量复核判"没减一"（R5）-> **不计数、不刷屏**，本件按作废走：货还在托盘上，
 *                              上位机会继续报它 ⇒ 下一轮自然重试（见 Sort_VerifyPending）
 *
 *          ★ 成功判据 = **两帧数量差 1**（2026-10-10 现场口径，替代激光闸门）：
 *            发车帧（领目标那一帧）数量 N、到账帧（下一轮清缓存后的第一帧）数量 M，
 *            M == N-1 才计数刷屏。两帧的取帧时机、空盘静默窗、口径外的代价
 *            都写在 sort_config.h 的"数量复核"一节（那是本文件的规格书）。
 *
 *          作废统一走 Sort_Settle：先就地轻放货物，再结清转盘那笔异步运动。
 *          带着货进下一轮会把货拖出装置 = 直接判本轮结束，比少分拣一件严重得多
 */
#include "sort_task.h"
#include "sort_config.h"
#include "vision_task.h"
#include "display_task.h"
#include "motion_app.h"
#include "bsp_gpio.h"
#include "servo.h"
#include <FreeRTOS.h>
#include <task.h>

/* 类别 -> 储物盒：下标 = 类别 ID - 1（类别 1~6，见 CFG_NUM_CATEGORIES）。
 * 现场抽签后把这 6 项改成 0~5 的一个排列（哪个类别进哪个盒由抽签决定；两类同盒违反归类规则）。
 * 下标无需再做范围判断：类别已被 vision_task 校验为 1~6，越界帧根本进不来 */
static const uint8_t s_category_to_bin[CFG_NUM_CATEGORIES] = {
    0u, 1u, 2u, 3u, 4u, 5u,
};

/* 形状 -> 夹持方案：**形状 0 = 默认方案**（不在三种特调之内，直接用兜底 CFG_CLAW_* 值）；
 * 形状 1~3 = 三种特调（1=立方体、2=12面体、3=六棱柱），下标 = 形状 - 1。
 * 形状**不参与分类**（进哪个盒只由类别定），只决定夹爪的初夹/夹紧角度。
 * 数值在 sort_config.h 的 CFG_GRIP_*（标定只改配置）；★2026-10-01 已按形状逐套实测定稿
 * （不再是三种形状的折中值）。
 * 下标无需再做范围判断：形状已被 vision_task 校验为 0~3，越界帧根本进不来 */
static const uint16_t s_grip_half_us[CFG_NUM_SHAPES]  = CFG_GRIP_HALF_US;
static const uint16_t s_grip_close_us[CFG_NUM_SHAPES] = CFG_GRIP_CLOSE_US;

/* 只有投放**复核通过**才 ++，所以屏上 6 行计数之和永远等于真实件数（丢一件会被判成计数错误）；
   同时它单调递增，既当 display 回执握手的序号，也当 display 侧"这一件是否已计数"的去重凭据 */
static uint16_t s_total = 0u;

/* 判"没减一"（夹空/滑脱/补货干扰）的累计次数（诊断，屏上不体现——屏只有 6 行成功数）。
   装机/调机时靠它区分"机器在空转"与"根本没读到数量位"（后者见 Sort_CountNoFrame）*/
static uint16_t s_count_miss = 0u;
/* 到账帧迟迟不来（超过 CFG_COUNT_WAIT_MS）的累计次数（诊断）：链路/上位机停发的可见凭据 */
static uint16_t s_count_noframe = 0u;

uint16_t Sort_CountMissCount(void) {
    return s_count_miss;
}

uint16_t Sort_CountNoFrameCount(void) {
    return s_count_noframe;
}

/* 激光判"夹空"的累计次数（诊断）：CFG_LASER_ENABLE=0（现场配置）时恒 0 —— 激光已退出闸门 */
static uint16_t s_laser_miss = 0u;

uint16_t Sort_LaserMissCount(void) {
    return s_laser_miss;
}

/* ★ 待复核件（2026-10-10 数量复核）：本件的"发车帧数量 + 类别"在这里等着下一次读事件
   （投放动作一走完就记下，真正的成功判定发生在下一轮清缓存后的第一帧，见 Sort_VerifyPending）。
   0 = 没有待复核件（作废在本件投放之前、或已经复核完了） */
static uint8_t s_pending = 0u;
static uint8_t s_pending_cat = 0u; /* 复核通过要刷的那一行 = 本件类别 */
static uint8_t s_pending_n0 = 0u;  /* 发车帧数量 N（本件还没被抓走时的托盘件数，≥1）*/

/* 1 = 龙门已在读帧位（投放点上方）且静默期已过，允许取帧；见 Sort_PrepareRead */
static uint8_t s_read_ready = 0u;

/* 1 = 本轮"空盘静默窗"已经给过（补货用）——同一段空盘只给一次，读到目标就清 */
static uint8_t s_refill_hold_done = 0u;

/* ★ 1 = "刚读到那个为零的帧"（到账数量 0）⇒ 主循环必须给一段静默窗（2026-10-10 二稿）。
   用户口径是**无条件**的："已经读到那个为零的帧的时候，之后延时 5 秒钟，什么都不读，
   等等我们把物块重新放上去"。第一版把它挂在"托盘已空（连读 10 帧无目标）"上 ⇒
   操作员在窗内**先放**一件的话，空盘判据当场被目标帧清掉、窗被跳过 ⇒ 机器立刻开工,
   与还在往里放的操作员抢时间 ⇒ 那一件的到账数量对不上（少计，仿真 s29 抓到）。
   现在改成"读到零帧就记一笔"，由主循环统一兑现（见 SortTask 的静默窗分支）。*/
static uint8_t s_refill_zero_seen = 0u;

static uint8_t Sort_Goto(float x_mm, float y_mm) {
    return Motion_MoveXYBlocking(x_mm, y_mm, CFG_XY_SPEED_MM_S);
}

/* 张爪必须是显式一步：进抓取序先把爪张到全开巡航形——直接下伸闭合等于对着货物合爪、
   必然抓空。入口不变量 = 爪已全开（投放/轻放收尾、上电位都是全开），这一步是兜底重申。
   初夹/夹紧的脉宽按**形状**选：0 = 默认方案（兜底值），1~3 = 特调方案查表
   （形状已被 vision_task 校验为 0~3） */
static void Sort_Grab(uint8_t shape) {
    uint16_t half_us  = (shape == 0u) ? (uint16_t)CFG_CLAW_HALF_US
                                      : s_grip_half_us[shape - 1u];
    uint16_t close_us = (shape == 0u) ? (uint16_t)CFG_CLAW_CLOSE_US
                                      : s_grip_close_us[shape - 1u];

    Servo_ClawSet(SERVO_CLAW_OPEN);
    vTaskDelay(pdMS_TO_TICKS(CFG_SERVO_MOVE_MS));
    /* ★ 到目标 XY、下放之前先初步夹住（台架 pick 定稿序，sort_config.h 抓取时序硬约束）：
       对着物块直接合到夹紧会把块顶歪/顶飞，先初夹轻接触再下放夹紧 */
    Servo_ClawSetUs(half_us);
    vTaskDelay(pdMS_TO_TICKS(CFG_SERVO_MOVE_MS));
    Servo_LiftSet(SERVO_LIFT_DOWN);
    /* 升降行程比换档慢，等待给双倍（台架定稿）：没放到底就夹紧会顶死 */
    vTaskDelay(pdMS_TO_TICKS(CFG_SERVO_MOVE_MS * 2u));
    Servo_ClawSetUs(close_us);
    vTaskDelay(pdMS_TO_TICKS(CFG_CLAW_CLOSE_MS));
    Servo_LiftSet(SERVO_LIFT_UP);
    vTaskDelay(pdMS_TO_TICKS(CFG_SERVO_MOVE_MS));
}

/* 轻放（下伸 -> 半开松爪 -> 抬升 -> 回全开）：只用于"作废时就地把货交还台面"。
   收尾必须回全开：之后龙门还要平移，半开的爪一样会扫到托盘边/旁边物块（巡航全开硬约束） */
static void Sort_PlaceSoft(void) {
    Servo_LiftSet(SERVO_LIFT_DOWN);
    vTaskDelay(pdMS_TO_TICKS(CFG_SERVO_MOVE_MS));
    Servo_ClawSet(SERVO_CLAW_HALF);
    vTaskDelay(pdMS_TO_TICKS(CFG_SERVO_MOVE_MS));
    Servo_LiftSet(SERVO_LIFT_UP);
    vTaskDelay(pdMS_TO_TICKS(CFG_SERVO_MOVE_MS));
    Servo_ClawSet(SERVO_CLAW_OPEN);
    vTaskDelay(pdMS_TO_TICKS(CFG_SERVO_MOVE_MS));
}

/* 判据只有一条：带着货的作废流程比不带着货危险得多（拖出装置 = 判本轮结束）。
   就地轻放最坏只是这一件白做，货物仍在台面上，K230 会重新报它为目标 */
static void Sort_ReleaseIfHolding(uint8_t *loaded) {
    if (*loaded == 0u) {
        return;
    }
    Sort_PlaceSoft();
    *loaded = 0u;
}

/* ★ 投放点激光证据（2026-10-09；完整口径见 sort_config.h 该节）：
   窗口内出现"**一次检测**"就算（用户口径："只要检测脉冲就可以了"）——两种形式都收：
     ① **下降沿计数比基准涨了**：台架实测一次落料 ≈ **1µs 宽的窄事件**，
        1ms 电平轮询会漏 ⇒ 只有 PD14/EXTI14 的沿抓得住（这是主路径）
     ② 此刻电平正是"检测到"：模块输出若是稳态电平（另一种模块/接法），用这条兜
   edges0 = 进窗口前记下的沿计数基准 ⇒ 窗口外的沿（路上、别的时段）不算数 */
#if CFG_LASER_ENABLE
static uint8_t Sort_LaserEvidence(uint32_t edges0) {
    if (Bsp_Gpio_LaserEdges() != edges0) {
        return 1u;
    }
    return (Bsp_Gpio_LaserPart() != 0u) ? 1u : 0u;
}
#endif

/* ★ 投放（松爪）：到投放点 → 静置（CFG_DROP_SETTLE_MS，等龙门停稳）→ 松爪 → 等舵机行程。
   ★ 激光复核（2026-10-09）挂在同一动作上，但**现场已关闸**（CFG_LASER_ENABLE=0，2026-10-10）：
   激光模块被夹爪遮住 ⇒ 分不清"货落下"与"爪在光路里" ⇒ 判据换成数量复核
   （见 sort_config.h 的"数量复核"一节、本文件的 Sort_VerifyPending）。
   激光那一支保留在下面（哪天模块挪开就是把 CFG_LASER_ENABLE 改回 1 的事）：
   窗口内出现"一次检测"就算——① 下降沿计数比基准涨了（台架实测一次落料 ≈1µs 窄事件，
   1ms 电平轮询会漏，只有 PD14/EXTI14 的沿抓得住）② 此刻电平正是"检测到"（稳态电平型模块）。
   @retval 1 = 松爪动作已走完（激光关闸，恒 1）或激光窗口内捕到检测
           0 = 激光开着但窗口内一次都没有（夹空：爪里本来就没货） */
static uint8_t Sort_DropOpen(void) {
#if CFG_LASER_ENABLE
    uint32_t t0;

    /* 窗口基准：到点这一刻记下沿计数（窗口外的沿——路上、别的时段——不算数） */
    uint32_t edges0 = Bsp_Gpio_LaserEdges();

    vTaskDelay(pdMS_TO_TICKS(CFG_DROP_SETTLE_MS)); /* 等龙门停稳再松爪、再等证据 */
    uint8_t  seen = Sort_LaserEvidence(edges0);    /* 松爪前先看一眼（稳态电平型模块也算证据） */

    Servo_ClawSet(SERVO_CLAW_OPEN);                /* 松爪：货落下 = 那一下窄事件 */
    t0 = HAL_GetTick();
    while (seen == 0u) {
        if ((HAL_GetTick() - t0) >= CFG_LASER_WAIT_MS) {
            break; /* 窗口内没证据 = 夹空（空爪松爪也是这个结果：本来就没东西落下来） */
        }
        vTaskDelay(pdMS_TO_TICKS(CFG_LASER_POLL_MS));
        seen = Sort_LaserEvidence(edges0);
    }

    /* 捕到会提前退出：把舵机行程时间补足（与旧 Sort_Drop 的 400ms 同理） */
    if ((HAL_GetTick() - t0) < (uint32_t)CFG_SERVO_MOVE_MS) {
        vTaskDelay(pdMS_TO_TICKS((uint32_t)CFG_SERVO_MOVE_MS - (uint32_t)(HAL_GetTick() - t0)));
    }
    return seen;
#else
    /* 激光关闸（现场配置）：松爪 + 等舵机行程，恒判 1 —— "到底抓没抓到"由下一轮
       读事件的数量差裁定（Sort_VerifyPending），不再在这里判 */
    vTaskDelay(pdMS_TO_TICKS(CFG_DROP_SETTLE_MS)); /* 等龙门停稳再松爪（与复核方式无关） */
    Servo_ClawSet(SERVO_CLAW_OPEN);                /* 松爪 + 保持全开（巡航安全形） */
    vTaskDelay(pdMS_TO_TICKS(CFG_SERVO_MOVE_MS));
    return 1u;
#endif
}

/* 标定兜底：XY 与转盘记账各自独立，哪条不可信就重标哪条。上电首次进主循环时两条都
   不可信，所以本函数同时承担"回零初始化"与"运行中掉信恢复"两个职责 */
static uint8_t Sort_EnsureCalibrated(void) {
    if (Motion_XyIsTrusted() && Motion_TrayIsTrusted()) {
        return 1u;
    }

    uint8_t ok = 1u;

    /* 回零会把龙门从读帧位带走（回零后停在退离位）⇒ 取帧资格当场作废。
       放在这里而不是主循环：守护任务中途作废记账也会走到本函数，那条路径同样要作废 */
    s_read_ready = 0u;

    if (!Motion_XyIsTrusted()) {
        ok = Motion_HomeXY();
    }
    if (ok && !Motion_TrayIsTrusted()) {
        ok = Motion_TrayHome();
    }

    if (!ok) {
        /* 回零失败 = 开关/机械/串口有问题。不停机：等一段再试，期间守护任务仍会急停，
           所以重试不会变成"反复撞限位"；人工排除故障后无需重启即可自动接上 */
        Vision_Flush();
        vTaskDelay(pdMS_TO_TICKS(2000u));
        return 0u;
    }

    Vision_Flush(); /* 丢掉旧帧与旧计数（文档 §3.1 步骤5） */
    return 1u;
}

static void Sort_WaitVisionLink(void) {
    while (!Vision_LinkAlive()) {
        vTaskDelay(pdMS_TO_TICKS(100u));
    }
    Vision_Flush();
}

/* 投递 + 只认本序号的回执。任一环节没成就补投再来一遍：同一序号内容完全相同，
   重复投递幂等（display 按序号去重，不会把同一件计两次），而显示任务的发送本身有界，
   所以迟早会公布这个序号，不会死等 */
static void Sort_UpdateDisplay(uint8_t category) {
    Display_Msg_t msg;

    msg.seq      = s_total;
    msg.category = category;

    for (;;) {
        (void)Display_Show(&msg);
        if (Display_WaitDone(msg.seq, CFG_DISPLAY_DONE_TIMEOUT_MS)) {
            return;
        }
    }
}

/* ★ 到账复核（2026-10-10；规格 = sort_config.h 的"数量复核"一节）：拿**到账帧的数量** M
   给上一件定案 ——
       M == N-1（N = 发车帧数量，本件还没被抓走时的托盘件数）⇒ 托盘确实少了一件
                ⇒ 本件真投放 ⇒ 计数 + 刷屏（屏上的数只有这一处会加）
       否则    ⇒ **不计数**：M == N = 夹空/滑脱（货还在托盘上，下一轮自然重抓）；
                其它 = 口径之外（运行中补货）⇒ 宁可少计，绝不虚增
   ★ 判决用的是"读懂的数量"而不是"动作走完了"：这是与旧口径（动作走完就计数）唯一的差别，
     也是现场"夹持失败但数量加一"那条缺陷的解药（反证见仿真 s24：关掉复核 ⇒ 屏上比盒里多出抓空次数）。
   @param count_now 到账帧的数量位（= 清缓存之后的第一帧，任何状态都算） */
static void Sort_VerifyPending(uint8_t count_now) {
#if CFG_COUNT_VERIFY_ENABLE
    /* ★ "读到那个为零的帧" = 到账帧数量 0（托盘刚被我方清空）⇒ 记一笔静默窗（无条件，
       与本次判定成不成无关：托盘空了就该给操作员补给的时间） */
    if (count_now == 0u) {
        s_refill_zero_seen = 1u;
    }
    if (s_pending == 0u) {
        return; /* 没有待复核件（本件作废在投放之前，或上一件已经复核过了） */
    }
    s_pending = 0u;
    if ((uint8_t)(count_now + 1u) == s_pending_n0) { /* M == N-1（写成加法：N=0 时不回绕） */
        s_total++;
        Sort_UpdateDisplay(s_pending_cat);
        return;
    }
    s_count_miss++; /* 数量没减一：夹空/滑脱（可见的少计）或运行中补货（口径外的代价） */
#else
    (void)count_now; /* 复核关掉（旧口径）：投放动作走完时就已经计数刷屏了，这里什么都不做 */
#endif
}

/* ★ 读事件（数量复核的"到账"时刻）：等**清缓存之后的第一帧**来给上一件定案。
   为什么是"清缓存后第一帧"：清缓存之前的帧属于"别的时候的托盘"（可能正是投放前拍的），
   拿它复核 = 拿旧账对新账 ⇒ 会把真投放判成没减一。
   为什么等的是**任何状态**的帧、而不只是目标帧：托盘空了就没有目标帧了——
   末件的 +1 与空盘静默窗全指望状态 00 的帧（它同样带数量位，见 k230_uart.h）。
   @retval 1 = 已复核（或本就没有待复核件）  0 = 等到超时（CFG_COUNT_WAIT_MS）：
             链路/上位机停发 —— 本件挂起、由调用方等链路恢复后重试，绝不带着旧账往下走 */
static uint8_t Sort_ReadEvent(void) {
#if CFG_COUNT_VERIFY_ENABLE
    uint32_t t0    = HAL_GetTick();
    uint8_t  count = 0u;

    while (Vision_GetCount(&count) == 0u) {
        if ((HAL_GetTick() - t0) >= CFG_COUNT_WAIT_MS) {
            return 0u;
        }
        vTaskDelay(pdMS_TO_TICKS(5u)); /* 帧周期 100ms：等一帧最多 20 步 */
    }
    Sort_VerifyPending(count);
    return 1u;
#else
    return 1u; /* 复核关掉：读事件退化成空操作（没有待复核件这回事） */
#endif
}

/* 作废出口的统一收尾：先放货（物理风险最高），再结清转盘那笔异步运动。
   Motion_TrayFinish 在上一笔已结清时是空操作，所以每个作废点都能无条件调；
   但绝不能不调：不结清就返回，下一轮会在上一笔仍在执行时再叠一笔相对运动 */
static void Sort_Settle(uint8_t *loaded, uint32_t tray_est_ms) {
    Sort_ReleaseIfHolding(loaded);
    (void)Motion_TrayFinish(tray_est_ms);
}

/* ★ 目标钳位（2026-10-10 从 Sort_DoCycle 里收出来：四条规则本来就是**一条链**，原先散在
   四段注释里，读起来像四个独立开关）。入参 = Vision_ToMachine 出的机械坐标，出参 = 钳位后的
   目标；顺序不能改（每条的输入是上一条的输出）：
     ① **-X 夹爪延伸带**（2026-10-01 台架口径）：物块可出现在 X ∈ [CFG_XY_X_CLAMP_MM, X_MIN)
        ——龙门停在 X_MIN 时夹爪构造仍够得着 ⇒ 钳到 X_MIN 就近夹；比 CLAMP 还远作废本件
     ② **+X/+Y 极限位夹取带**（2026-10-08 用户口径"到限位之后就按最极限的位置，不管怎么样都
        直接夹"）：落在 (上限, 上限+带] 的目标钳到上限照样伸爪；带值先与 CFG_WORK_OVERSHOOT_MM
        取大（带值 = 0 时退化成旧的"只允许 3mm 微越界"口径），带外作废本件
     ③ **限位开关保护边距**（2026-10-08；2026-10-10 边距 5 → 3mm）：最大端就是回零开关
        所在处（触发点 = 坐标 0 + 3mm），钳到上限等于每抓一次贴边件就朝开关顶满最后 3mm
        ——纯开环漂移吃掉那 3mm 就会真压开关 ⇒ 作废 XY 记账 ⇒ 下一圈白回零（"抓完自己
        回零校准"）。回拉到 `上限 − 边距`（3mm，见 sort_config.h 该宏注释）：夹爪有
        ~25mm 延伸带，少走这 3mm 照样夹得住，换来"永不朝开关方向顶满"
   在规则之外作废的件：**转盘还没动、爪未动，无在途动作需要 settle**，直接 return 即可。
   ⚠️ -Y 下限（= 投放点）不在本函数管：越界由 Motion_MoveXYBlocking 的工作包络拒绝
      ⇒ Sort_Goto 失败 ⇒ 走作废路径（与 Sort_DoCycle 里其余失败同口径）
   ⚠️ 上位机会持续重报同一目标 ⇒ 被作废的件会反复尝试：这是"看得见够不着"的可见故障，
      好过静默抓空计虚数（判据见台架 `read` 的越界判词）
   @retval 1 = 可用（*x_mm / *y_mm 已就地钳位）  0 = 整件作废 */
static uint8_t Sort_ClampTarget(float *x_mm, float *y_mm) {
    float x = *x_mm;
    float y = *y_mm;
    /* 带值取"带"与"微越界允许量"里的较大者：带值 = 0 时退化成旧口径 */
    float band_x = (CFG_XY_X_CLAMP_BAND_MM > CFG_WORK_OVERSHOOT_MM) ? CFG_XY_X_CLAMP_BAND_MM
                                                                    : CFG_WORK_OVERSHOOT_MM;
    float band_y = (CFG_XY_Y_CLAMP_BAND_MM > CFG_WORK_OVERSHOOT_MM) ? CFG_XY_Y_CLAMP_BAND_MM
                                                                    : CFG_WORK_OVERSHOOT_MM;

    if (x < CFG_WORK_X_MIN_MM) { /* ① */
        if (x < CFG_XY_X_CLAMP_MM) {
            return 0u;
        }
        x = CFG_WORK_X_MIN_MM;
    }
    if (x > CFG_WORK_X_MAX_MM) { /* ② */
        if (x > (CFG_WORK_X_MAX_MM + band_x)) {
            return 0u;
        }
        x = CFG_WORK_X_MAX_MM;
    }
    if (y > CFG_WORK_Y_MAX_MM) { /* ②（Y 只有上限侧：下限 = 投放点，越界交给运动层拒绝） */
        if (y > (CFG_WORK_Y_MAX_MM + band_y)) {
            return 0u;
        }
        y = CFG_WORK_Y_MAX_MM;
    }
    if (x > (CFG_WORK_X_MAX_MM - CFG_WORK_EDGE_MARGIN_MM)) { /* ③ */
        x = CFG_WORK_X_MAX_MM - CFG_WORK_EDGE_MARGIN_MM;
    }
    if (y > (CFG_WORK_Y_MAX_MM - CFG_WORK_EDGE_MARGIN_MM)) {
        y = CFG_WORK_Y_MAX_MM - CFG_WORK_EDGE_MARGIN_MM;
    }

    *x_mm = x;
    *y_mm = y;
    return 1u;
}

/* @retval 1=本件已投放（是否计数由下一轮读事件的数量复核裁定，见 Sort_VerifyPending）
           0=本件作废（不投放、不计数） */
static uint8_t Sort_DoCycle(const K230_Target_t *target) {
    float   target_x = 0.0f;
    float   target_y = 0.0f;
    uint8_t loaded   = 0u; /* 1 = 夹爪里正压着货，作废前必须先放下 */
    uint8_t bin      = s_category_to_bin[target->category - 1u];

    Vision_ToMachine(target, &target_x, &target_y);

    /* 目标钳位（延伸带 / 极限位夹取带 / 保护边距，见函数头）：带外 ⇒ 本件作废 */
    if (!Sort_ClampTarget(&target_x, &target_y)) {
        return 0u;
    }

    /* 步骤2+3：转盘预定位（异步）与 XY 移动并行，谁慢等谁 */
    uint32_t tray_est_ms = Motion_TrayGotoBin(bin);

    if (!Sort_Goto(target_x, target_y)) {
        Sort_Settle(&loaded, tray_est_ms);
        return 0u;
    }
    /* 转盘到位后必须过"索引沿 + 双路盒号交叉验证"才承认落点 */
    if (!Motion_TrayFinish(tray_est_ms)) {
        Sort_Settle(&loaded, tray_est_ms);
        return 0u;
    }

    /* 步骤4：抓取（按形状选夹持方案）。"到底抓没抓到"不再靠假设：由投放后那一帧的数量差裁定 */
    Sort_Grab(target->shape);
    loaded = 1u;

    /* 步骤6：投放（落差由机械侧转盘格兜底，R1）。松爪 = 投放动作，成功与否由读事件定案 */
    if (!Sort_Goto(CFG_DROP_X_MM, CFG_DROP_Y_MM)) {
        Sort_Settle(&loaded, tray_est_ms);
        return 0u;
    }
    uint8_t dropped = Sort_DropOpen(); /* 静置 + 松爪（+ 激光复核，若闸门开着） */
    loaded = 0u;                       /* 爪已全开（Sort_DropOpen 收尾保证），手里不再有东西 */

    if (dropped == 0u) {
        /* 激光闸门开着时才会走到这里（现场已关闸）：空爪松开在物理上无风险（转盘早已为这一类
           转好盒；万一"传感器看走了眼、货其实还在爪里"，它也是落进自己的盒），但**不计数** */
        s_laser_miss++;
        Sort_Settle(&loaded, tray_est_ms);
        return 0u;
    }

#if CFG_COUNT_VERIFY_ENABLE
    /* ★ 不在这里计数、也不在这里刷屏（2026-10-10 数量复核）：投放动作走完了 ≠ 抓到了货。
       把"发车帧数量 + 类别"记成待复核件，真正的定案交给**下一轮读事件**（Sort_ReadEvent →
       Sort_VerifyPending）——那时候已经清过缓存、拿到投放之后的托盘新账（数量差 1 才 +1）。
       屏上"显示完成前不得分拣下一件"的红线不受影响：定案与刷屏都发生在下一件动身之前 */
    s_pending     = 1u;
    s_pending_cat = target->category;
    s_pending_n0  = target->count;
#else
    /* 旧口径（复核关掉）：投放动作走完就认成功，立即计数 + 刷屏 */
    s_total++;
    Sort_UpdateDisplay(target->category);
#endif

    /* 步骤8：**不回待机点**——就停在投放点上方（= 读帧位，见 Sort_PrepareRead）。
       回待机点那趟路会让龙门重新扫过托盘、挡住相机，而且下一件还要再走回来。
       失败也不回退计数，下一轮开头会检测到掉信并重新标定 */
    return 1u;
}

/* 读帧位 = 投放点上方：归零/待机时龙门就在托盘上方，会把物块挡在相机视野外（2026-09-30
   现场口径），所以取帧必须先把龙门挪到不遮挡位。投放点本身就在托盘外、且每件都要去那里
   松爪 ⇒ "每件结束时停在投放点上方" = 零额外位移，读帧位白得。
   ★ 到位后必须【等静默期 + 清一次视觉缓存】：到位前的帧是在别处（可能正挡着相机）产生的，
   锁到它 = 按被遮挡/已取走的物块坐标去抓（抓空、计数虚增）。清缓存只在这里做，绝不能每圈
   都做——那会把 100ms 一帧的数据流整条丢掉，永远读不到目标 */
static uint8_t Sort_PrepareRead(void) {
    if (!Sort_Goto(CFG_DROP_X_MM, CFG_DROP_Y_MM)) {
        return 0u; /* 走不过去（撞限位/发不出去）：交回主循环，那里会作废记账并重新回零 */
    }
    vTaskDelay(pdMS_TO_TICKS(CFG_VISION_SETTLE_MS));
    Vision_Flush();
    return 1u;
}

void SortTask(void *argument) {
    (void)argument;

    /* 回零由主循环第一轮的 Sort_EnsureCalibrated 统一完成，失败会一直重试 */
    Motion_Init();

    for (;;) {
        K230_Target_t target;

        if (!Sort_EnsureCalibrated()) {
            continue;
        }
        if (!s_read_ready) {
            if (!Sort_PrepareRead()) {
                /* 走不到读帧位（投放点未标定/被包络拒绝/撞限位）：**必须让出 CPU 再重试**。
                   空转不打延时会把时间也一起停住——真机上 sort 任务独吃 CPU，仿真里整个场景
                   卡死不动（s18 出厂配置就是这条路）。不在这里作废记账：越界拒绝不改可信度，
                   下一轮开头会自己检测并重新回零 */
                vTaskDelay(pdMS_TO_TICKS(500u));
                continue;
            }
            s_read_ready = 1u;
        }
        /* ★ 读事件（2026-10-10）：先把**上一件**的到账复核落地，再领新目标。
           等不到帧（超时）= 链路问题 ⇒ 本件挂起、**绝不往下走**：带着没结的账去抓下一件，
           两件的数量差会搅在一起，那一件就永远判不出来了。等链路恢复后重试即可
           （期间机器没动过托盘，账还对得上） */
        if (s_pending != 0u) {
            if (!Sort_ReadEvent()) {
                s_count_noframe++;
                if (!Vision_LinkAlive()) {
                    Sort_WaitVisionLink(); /* 等恢复（内部清缓存）⇒ 回来重取第一帧 */
                }
                continue;
            }
        }
        /* 读 vision_task 缓存里"未消费且仍新鲜"的一帧（它自带发车帧数量 N） */
        if (Vision_GetTarget(&target)) {
            s_refill_hold_done = 0u; /* 又有货可抓：空盘静默窗重新计（见下面的空盘分支） */
            (void)Sort_DoCycle(&target);
            /* 本件结束：龙门已离开读帧位（成功时在投放点、作废时在轻放点），
               且物块刚被取走/放回，下一帧必须重新等视觉更新 */
            s_read_ready = 0u;
            continue;
        }
        if (!Vision_LinkAlive()) {
            Sort_WaitVisionLink();
            continue;
        }
        /* ★ 空盘静默窗（CFG_REFILL_HOLD_MS，用户口径 5s；见 sort_config.h"数量复核"一节）。
           **两个触发口，同一段空盘只给一次**（`s_refill_hold_done`，读到目标即重新计）：
             ① 零帧口（主用）：上一件的到账帧数量 = 0 —— "已经读到那个为零的帧" ⇒ 立刻给窗。
                必须**无条件**（不看此刻有没有新目标）：操作员可能在窗内先放一件再放其余的，
                若"见了目标就跳过窗"，机器会与还在放货的人抢时间 ⇒ 那一件的到账数量对不上
                （少计；仿真 s29 就是这条的回归）。窗内**什么都不读**（不取帧、不计判、不刷屏）
                ⇒ 操作员把货放完、退开，下一件的发车帧才是干净的新账
             ② 空盘口（兜底）：上电时托盘本来就是空的 —— 没有"上一件"，也就没有零帧，
                用"连续 M 帧无目标"判空后给同样的窗（同样只给一次）
           窗后一律：清缓存 + 重走"到读帧位 + 静默期 + 取帧"（`s_read_ready = 0`） */
#if CFG_COUNT_VERIFY_ENABLE
        if ((s_refill_zero_seen != 0u) ||
            ((Vision_TrayEmpty() != 0u) && (s_refill_hold_done == 0u))) {
            s_refill_zero_seen = 0u;
            s_refill_hold_done = 1u;
            vTaskDelay(pdMS_TO_TICKS(CFG_REFILL_HOLD_MS));
            Vision_Flush();
            s_read_ready = 0u;
            continue;
        }
#endif
        /* 托盘已空 -> 停在原点待机，屏上保留最终计数。这不是不可逆状态：
           再放货物、K230 重新报目标就自动继续分拣 */
        if (Vision_TrayEmpty()) {
            vTaskDelay(pdMS_TO_TICKS(100u));
        } else {
            vTaskDelay(pdMS_TO_TICKS(20u)); /* 等 K230 下一帧（周期约 100ms） */
        }
    }
}