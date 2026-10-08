/**
 * @file    sort_task.c
 * @brief   分拣任务（对应电控文档 §3.2 主分拣循环）
 * @note    循环骨架：标定可信? -> [到读帧位（投放点上方）＋静默期] -> 领目标
 *          -> [转盘预定位(与 XY 并行) + XY 移动] -> 转盘索引复核 -> 抓取 -> 投放
 *          -> 显示(等本轮回执) -> （就停在读帧位）下一件
 *
 *          三类"不干净"的处置：
 *            运动不可信（撞限位/丢步/发不出去）-> 本件作废，下轮开头自动重新回零
 *            视觉链路断 -> 停在待机点等恢复，这不等于"托盘已空"，绝不能报完成
 *            刷屏无回执 -> 停在原地按同一序号重投（题目红线：显示完成前不得分拣下一件）
 *
 *          作废统一走 Sort_Settle：先就地轻放货物，再结清转盘那笔异步运动。
 *          带着货进下一轮会把货拖出装置 = 直接判本轮结束，比少分拣一件严重得多
 */
#include "sort_task.h"
#include "sort_config.h"
#include "vision_task.h"
#include "display_task.h"
#include "motion_app.h"
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

/* 只有投放成功才 ++，所以屏上 6 行计数之和永远等于真实件数（丢一件会被判成计数错误）；
   同时它单调递增，既当 display 回执握手的序号，也当 display 侧"这一件是否已计数"的去重凭据 */
static uint16_t s_total = 0u;

/* 1 = 龙门已在读帧位（投放点上方）且静默期已过，允许取帧；见 Sort_PrepareRead */
static uint8_t s_read_ready = 0u;

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

/* 投放：抬升位直接张开松爪，松爪后**保持全开**（2026-10-01 用户口径：投放后回闭合，
   龙门平移去抓下一件时闭合的爪会撞托盘边/旁边物块——巡航全开是机构硬约束；
   下一件 Sort_Grab 第一步本来就要先张爪，这里不闭合正好衔接上）。
   不读码（货物顶面二维码属模拟污渍） */
static void Sort_Drop(void) {
    Servo_ClawSet(SERVO_CLAW_OPEN);
    vTaskDelay(pdMS_TO_TICKS(CFG_SERVO_MOVE_MS));
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

/* 作废出口的统一收尾：先放货（物理风险最高），再结清转盘那笔异步运动。
   Motion_TrayFinish 在上一笔已结清时是空操作，所以每个作废点都能无条件调；
   但绝不能不调：不结清就返回，下一轮会在上一笔仍在执行时再叠一笔相对运动 */
static void Sort_Settle(uint8_t *loaded, uint32_t tray_est_ms) {
    Sort_ReleaseIfHolding(loaded);
    (void)Motion_TrayFinish(tray_est_ms);
}

/* @retval 1=已投放、已计数、已刷屏  0=本件作废（不投放、不计数） */
static uint8_t Sort_DoCycle(const K230_Target_t *target) {
    float   target_x = 0.0f;
    float   target_y = 0.0f;
    uint8_t loaded   = 0u; /* 1 = 夹爪里正压着货，作废前必须先放下 */
    uint8_t bin      = s_category_to_bin[target->category - 1u];

    Vision_ToMachine(target, &target_x, &target_y);

    /* ★ 夹爪延伸夹取带（2026-10-01 台架口径）：物块可出现在 X ∈ [CLAMP, X_MIN)——龙门停在
       X_MIN 时夹爪构造仍够得着。目标 X 超出软限位就**钳位到 X_MIN 就近夹取**；比 CLAMP
       还远的作废本件（转盘还没动、爪未动，无在途动作需要 settle）。⚠️ 上位机会持续重报
       同一目标 → 会反复尝试同一件：这是"看得见够不着"的可见故障，好过静默抓空计虚数 */
    if (target_x < CFG_WORK_X_MIN_MM) {
        if (target_x < CFG_XY_X_CLAMP_MM) {
            return 0u;
        }
        target_x = CFG_WORK_X_MIN_MM;
    }
    /* ★ +X/+Y 微越界钳位（CFG_WORK_OVERSHOOT_MM，2026-10-02）：软限位 0 给回零开关留了
       3mm 内收，物块贴外沿映射到 (0,+3] 时机构仍可达，钳回 0 就近夹取；超过 = 标定/
       镜像问题，作废本件（上位机会持续重报同一目标 → 表现为"机器不动"——先核物块
       是不是贴边 + 坐标微微越界，别当成新鲜度问题排查）。同样无在途动作，无需 settle */
    /* ★ 极限位夹取带（2026-10-08 用户口径："y 轴过于偏夹不到——到限位之后就按最极限的位置，
       不管怎么样都直接夹"）：带内（上限 + 带值 之内）不再作废，钳到极限位照样伸爪
       （下面还有"保护边距"再让开 CFG_WORK_EDGE_MARGIN_MM）；带外才作废本件。
       带值 = 0 时退化成旧的"只允许 CFG_WORK_OVERSHOOT_MM 微越界"口径 */
    float band_x = (CFG_XY_X_CLAMP_BAND_MM > CFG_WORK_OVERSHOOT_MM) ? CFG_XY_X_CLAMP_BAND_MM
                                                                   : CFG_WORK_OVERSHOOT_MM;
    float band_y = (CFG_XY_Y_CLAMP_BAND_MM > CFG_WORK_OVERSHOOT_MM) ? CFG_XY_Y_CLAMP_BAND_MM
                                                                   : CFG_WORK_OVERSHOOT_MM;

    if (target_x > CFG_WORK_X_MAX_MM) {
        if (target_x > CFG_WORK_X_MAX_MM + band_x) {
            return 0u; /* 超出延伸带：夹爪也够不着 */
        }
        target_x = CFG_WORK_X_MAX_MM;
    }
    if (target_y > CFG_WORK_Y_MAX_MM) {
        if (target_y > CFG_WORK_Y_MAX_MM + band_y) {
            return 0u;
        }
        target_y = CFG_WORK_Y_MAX_MM;
    }

    /* ★ 限位开关保护边距（2026-10-08）：X/Y 最大端就是回零开关所在处（触发点 = 坐标 0 + 3mm），
       上面那条"微越界钳位"却会把龙门**指到 0**——等于每抓一次贴边件就朝开关顶满最后 3mm。
       纯开环的漂移/丢步（或跳齿）一旦吃掉这 3mm，开关真被压下 ⇒ 守护任务按"真撞限位"
       作废 XY 记账 ⇒ 下一圈重新回零（现场看到的"抓完自己回零校准"）。这里把落在保护边距内
       的目标回拉到 `上限 − 边距`——夹爪有 ~25mm 延伸带（CFG_XY_X_CLAMP_MM 一节），
       少走 5mm 照样夹得住，换来"永远不朝开关方向顶满"。只作用于最大端：-Y 下限 = 投放点
       必须走满、-X 走延伸带，都不受影响 */
    if (target_x > (CFG_WORK_X_MAX_MM - CFG_WORK_EDGE_MARGIN_MM)) {
        target_x = CFG_WORK_X_MAX_MM - CFG_WORK_EDGE_MARGIN_MM;
    }
    if (target_y > (CFG_WORK_Y_MAX_MM - CFG_WORK_EDGE_MARGIN_MM)) {
        target_y = CFG_WORK_Y_MAX_MM - CFG_WORK_EDGE_MARGIN_MM;
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

    /* 步骤4：抓取（按形状选夹持方案；不装到位开关，抓空风险 R5 已接受） */
    Sort_Grab(target->shape);
    loaded = 1u;

    /* 步骤6：投放（落差由机械侧转盘格兜底，R1）。
       从这里开始本件已经成功，后续任何一步失败都不回退计数 */
    if (!Sort_Goto(CFG_DROP_X_MM, CFG_DROP_Y_MM)) {
        Sort_Settle(&loaded, tray_est_ms);
        return 0u;
    }
    Sort_Drop();
    loaded = 0u;
    s_total++;

    /* 步骤7：显示更新（内部保证"本轮已刷完"才返回，红线在此守住） */
    Sort_UpdateDisplay(target->category);

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
        /* 读 vision_task 缓存里"未消费且仍新鲜"的一帧 */
        if (Vision_GetTarget(&target)) {
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
        /* 托盘已空 -> 停在原点待机，屏上保留最终计数。这不是不可逆状态：
           再放货物、K230 重新报目标就自动继续分拣 */
        if (Vision_TrayEmpty()) {
            vTaskDelay(pdMS_TO_TICKS(100u));
        } else {
            vTaskDelay(pdMS_TO_TICKS(20u)); /* 等 K230 下一帧（周期约 100ms） */
        }
    }
}