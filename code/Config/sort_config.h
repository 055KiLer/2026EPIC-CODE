/**
 * @file    sort_config.h
 * @brief   全部可调常量（标定只改这个文件）
 *          单位：时间 ms、位移 mm、速度 mm/s；电机协议层转速为 RPM
 */
#ifndef __SORT_CONFIG_H
#define __SORT_CONFIG_H

#include "main.h"

/* 外设 <-> 引脚 <-> 连接器：bsp_uart.c 的 s_huart[] 按此顺序索引，必须逐行同序 */
#define CFG_UART_SCREEN  0 /* USART6 PC6/PC7  -> CN2 串口屏 */
#define CFG_UART_K230    1 /* USART2 PD5/PD6  -> CN3 视觉 */
#define CFG_UART_SPARE   2 /* UART4  PA0/PA1  -> CN1 预留，无设备 */
#define CFG_UART_MOTOR_A 3 /* USART3 PC10/PC11 -> M1 */
#define CFG_UART_MOTOR_B 4 /* UART5  PC12/PD2  -> M2 */
#define CFG_UART_TRAY    5 /* USART1 PB6/PB7   -> M3 转盘 */

/* 限位/原点：常开接法 + 4.7k 上拉，压下 = 低电平 */
#define CFG_LIMIT_GPIO_X     GPIOB
#define CFG_LIMIT_PIN_X      GPIO_PIN_1 /* EXTI1   X 原点 */
#define CFG_LIMIT_GPIO_Y     GPIOC
#define CFG_LIMIT_PIN_Y      GPIO_PIN_5 /* EXTI9_5 Y 原点 */
#define CFG_ORIGIN_GPIO_TRAY GPIOB
#define CFG_ORIGIN_PIN_TRAY  GPIO_PIN_0     /* EXTI0   转盘原点 */
#define CFG_LIMIT_ACTIVE_LEVEL GPIO_PIN_RESET

/* 视觉上位机：ROCK 5C（2026-09-22 由庐山派 K230 改型；`k230_*` 全是历史命名，
   协议与型号无关，见电控文档 §4.1）。上位机侧的串口/系统要求见该节"上位机侧要求"表 */
#define CFG_K230_FRAME_PERIOD_MS  100
#define CFG_K230_NO_TARGET_FRAMES 10    /* 连续 M 帧无目标 -> 托盘已空 */
#define CFG_K230_LINK_DEAD_MS     1000u /* 超时无任何字节 -> 链路故障（≠ 完成） */
#define CFG_K230_TARGET_FRESH_MS  500u  /* 目标缓存有效期 */

/* ★ 视觉坐标 -> 机械坐标（2026-09-30 视觉组口径）：**两轴交换 + 平移**，不是纯平移。
   视觉标定中心 = 视觉原点，在托盘上"距**左边** 87.5mm、距**底部** 82.5mm"处；
   视觉系 x+ = 机械 Y+、视觉系 y+ = 机械 X+ ⇒ 机械 X 由帧里的 y 得来、机械 Y 由帧里的 x 得来。
   CFG_VISION_ORIGIN_X/Y_MM = 视觉原点在机械坐标系里的**绝对坐标**。
   ★ 2026-10-01 起产品坐标系 = 台架调试工程同款（第三象限，原点 = 归零驻留位），
     台架实测标定值**直接 1:1 填入**，不再有换算链：
     视觉原点 ≈ 夹爪走到 (-80, -45) 处（用户实测）；SIGN 符号 +1/+1 待四角核镜像定稿 */
/* ★ 2026-10-01 抓取偏移修正（实抓迭代定值，两轴独立标定，爪要往 + 走则原点 +）：
   Y：-45 → -60（首抓整体偏 +Y 10~20，回退中值 15）→ **-57.5**（再向 +Y 补 2.5）；
   X：-80 → -75（首抓偏 −X 约 5）→ **-70**（再向 +X 补 5）。
   微调口径：夹持点还需移动 Δmm，原点同向加 Δ（1:1）；误差若"随位置变化/镜像"
   才查 CFG_VISION_SIGN_*。改这里必须同步台架 sort_config.h */
#define CFG_VISION_ORIGIN_X_MM  (-70.0f)
#define CFG_VISION_ORIGIN_Y_MM  (-57.5f)
#define CFG_VISION_SIGN_VY_TO_X (+1.0f) /* 机械 X = ORIGIN_X + SIGN × 帧.y_mm */
#define CFG_VISION_SIGN_VX_TO_Y (+1.0f) /* 机械 Y = ORIGIN_Y + SIGN × 帧.x_mm */

/* ★ 读帧时机（2026-09-30 现场口径）：归零后龙门停在托盘上方会挡住相机，所以**读帧位 =
   投放点上方**（不遮挡位）——主循环先走到那里再取帧，见 sort_task.c 的读帧位说明。
   CFG_VISION_SETTLE_MS = 到读帧位后、取帧前的静默期：物块刚被夹走，上位机需要 ≤2s 才能把
   "当前应抓目标"换成下一个（§4.1 上位机职责）；"夹取→抬升→走到投放点→松爪→刷屏"本身已
   占掉大部分，这里只补最坏情形（下一件紧邻投放点、路上没花时间）的差额。
   症状判据：若出现"抓着刚取走物块的旧坐标"（抓空、计数虚增），把它调大 */
#define CFG_VISION_SETTLE_MS 500u

/* 串口屏 */
#define CFG_DISPLAY_DONE_TIMEOUT_MS 2000u

/* CoreXY */
#define CFG_PULSE_PER_REV_42 3200  /* 1.8° x 16 细分 */
#define CFG_XY_PULSE_PER_MM  80.0f /* GT2 20T：3200/40 */
#define CFG_AXIS_A_SIGN      (+1)
#define CFG_AXIS_B_SIGN      (+1)
/* ★ 回零方向：2026-09-28 台架实锤——两只限位开关装在 X/Y **最大行程端**（不在原点端）。
   回零必须朝 +X/+Y 找开关；原值 -1 会永远碰不到开关、顶着行程端推（P0-6，11s/轴/次） */
#define CFG_HOME_DIR_X       (+1.0f)
#define CFG_HOME_DIR_Y       (+1.0f)
/* 粗压行程预算：给足余量、由触发沿停住；真挂住时"顶着推"的距离就是它（宁小勿大）。
   台架实测最长行程 Y≈190 → 220（2026-10-01 随台架口径移植，旧值 400） */
#define CFG_HOME_TRAVEL_MM   220.0f
#define CFG_BACKOFF_MM       3.0f
/* ★ 触发点在工作坐标系里的位置 = (+3, +3)：2026-10-01 起产品坐标系与台架调试工程
   **完全同框**——原点 = 归零驻留位（触发点内退 CFG_BACKOFF_MM），工作区第三象限
   （X≤0 且 Y≤0，P0-6 二选一定稿）。台架标定值从此 1:1 搬运，零换算 */
#define CFG_HOME_X_MAX_MM    3.0f
#define CFG_HOME_Y_MAX_MM    3.0f

/* ★ A/B 两帧之间的最小间隔：2026-09-25/28 台架实测，背靠背下发（间隔≈1.1ms）时驱动器
   丢帧（第一台应答被打断、第二台静默）——归零"跑斜线"事件的根源之一。台架口径 =
   3ms 等应答 + 7ms 空闲共 10ms；产品暂不读应答，取 10ms 纯间隔 */
#define CFG_ZDT_FRAME_GAP_MS 10u
/* 停止帧之间的间隔：急停主项是第一帧（立即停一轴/广播停），第二帧 5ms 后到只是兜底 */
#define CFG_ZDT_STOP_GAP_MS 5u

/* 限位消抖 + 按压闩锁：2026-09-26 台架实测单次按压产生 17~128 个沿、松开瞬间也能蹦
   61 沿（弱上拉天线效应+触点弹跳）——裸沿锁存会把"松开"判成"又撞了"，误触发急停。
   两道门：40ms 内只认一个沿；必须先确认"连续 40ms 松开"才武装下一次按压 */
#define CFG_LIMIT_DEBOUNCE_MS 40u

#define CFG_XY_SPEED_MM_S   200.0f /* 2026-10-01 二次提速：100 → 200（= 300rpm 正好用满
                                     解算器上限 Kin_RpmForSpeed 的 300，不再有加档余量；
                                     丢步/撞限位频发就回 100） */
#define CFG_HOME_SPEED_MM_S 40.0f
#define CFG_HOME_SLOW_MM_S  10.0f

#define CFG_MOVE_EXTRA_MS      200   /* 加减速时间余量 */
#define CFG_HOME_WAIT_EXTRA_MS 1000u

/* ZDT 电机 */
/* 加速度字节：0xFF 是**同一批驱动器上实测可用**的取值（物创固件在该字段固化 0xFF）；
   10 是从没上过真机的保守值。实机首轮建议先用 0xFF（已验证 > 未验证），
   若大龙门丢步再往小调 —— 改这一处即可 */
#define CFG_ZDT_ACCEL          0xFF /* 台架全程验证值（同批驱动器实测可用；旧值 10 从未上过真机） */

/* ★ 帧内「速度字段」的单位系数：真实转速 × 该系数 = 帧内数值。
   产品按 RPM 算（`Kin_RpmForSpeed` 出的是 rpm，直接进帧）→ 系数 1；
   台架实测若发现"实际比估算慢约 10 倍"，说明驱动器按 0.1RPM 解释该字段，
   把这里改成 10 即可（只影响设备层写帧，算法层不动）。判定法见
   `台架调试结论与代码修改方案.md` §4：`j a cw 30 3200` 转一圈，秒表 ≈2s / ≈20s */
#define CFG_ZDT_SPEED_FIELD_SCALE 1u

/* 发送超时预算：仅仿真构建的重试路径还在用（真机发送已改寄存器级纯计数，不走 tick，
   见 bsp_uart.c）；保留宏让两侧构建共用同一份配置 */
#define CFG_UART_TX_TIMEOUT_MS 100

/* ★ 上电驱动器使能 = Zdt_Init 逐台发一帧使能即完事，**不读应答**：本批驱动器对广播
   不回包（台架 2026-09-30 定稿，电控文档 §4.3"主控只发不收"）。旧版"发使能帧等回包的
   上电自检"（Zdt_Probe）在这批硬件上永远误报失败，2026-10-01 整链删除；"探应答"类
   诊断只是台架调试工程的**单播**件（ack/scan），产品全广播寻址，没有等价物也不需要 */

/* 转盘：传动链 = 电机 16T 小齿轮 --同步带--> 30T 大齿轮 --减速箱 1:20--> 转盘
   ★ 总减速比 = (30/16) × 20 = **37.5**（2026-09-30 台架实测：电机整一圈 = 转盘 9.6°，
   手机指南针量"3200 脉冲"≈10° 与之一致）。旧值只算减速箱（1:20→64000/圈）**少了一级皮带**，
   导致每格只转 32°（应为 60°）、回零超时按 10s/圈 算而真实要 18.75s。 */
#define CFG_PULSE_PER_REV_57      3200
#define CFG_TRAY_BELT_TEETH_MOTOR 16u /* 电机侧小齿轮齿数 */
#define CFG_TRAY_BELT_TEETH_GEAR  30u /* 减速箱侧大齿轮齿数 */
#define CFG_TRAY_GEAR_RATIO       20u /* 减速箱减速比 */
#define CFG_TRAY_BINS             6
/* 转盘一圈的电机脉冲：3200 × 30 ÷ 16 × 20 = 120000 */
#define CFG_TRAY_PULSES_PER_REV                                                                    \
    ((uint32_t)CFG_PULSE_PER_REV_57 * CFG_TRAY_BELT_TEETH_GEAR * CFG_TRAY_GEAR_RATIO /             \
     CFG_TRAY_BELT_TEETH_MOTOR)
/* 每格 60°：120000/6 = 20000（正好整除；乘 2 再除的四舍五入写法保留，防以后改齿数除不尽） */
#define CFG_TRAY_PULSES_PER_BIN ((uint32_t)((CFG_TRAY_PULSES_PER_REV * 2u / CFG_TRAY_BINS + 1u) / 2u))
#define CFG_TRAY_ZERO_BIN       0
#define CFG_TRAY_SPEED_RPM                                                                     \
    240 /* 对格（2026-10-01 二次提速 = 台架 400% 口径回填）：约 6.4 输出 RPM，单格 1.6s；       \
           用户实机觉得 120 还慢 → 翻倍试；复核反复不过（丢步/堵转）就回 120 */
#define CFG_TRAY_SEEK_RPM            480   /* 粗寻标：一圈 4.7s（二次提速翻倍试） */
#define CFG_TRAY_APPROACH_RPM        120   /* 退出/脱开/慢压/对格偏移（二次提速翻倍；
                                              慢压精度略降 ≈0.04°→0.08°，格宽 60° 无感） */
#define CFG_TRAY_BACKOFF_PULSES      600u  /* = 转盘 1.8°；必须 > 凸块作用宽度，
                                              不够时固件会自动再退一份（2×）兜底 */
/* ★ 过冲回找窗口的搜索上限（2026-10-01 二次提速后实机现象）：480rpm 撞停的过冲可能
   把凸块直接送过开关作用弧——停车点落在开关**身后**、电平=松开，"反向脱开"会立即
   假成功，慢压相就得从另一侧追将近一整圈（表现为"白转一圈"）。处置：停车点已松开时
   反向慢转这么远**回找窗口**，压上即停、再走常规退出；走满没压上=触发是噪声假沿
   （停车点其实还在窗口之前），直接进慢压相。8 份 = 14.4°，覆盖过冲量级绰绰有余
   （14.4° 相当于 480rpm 下 187ms 的停车延迟，不可能有这么大） */
#define CFG_TRAY_REENTRY_PULSES (CFG_TRAY_BACKOFF_PULSES * 8u)
#define CFG_TRAY_APPROACH_TIMEOUT_MS 1500u /* 最坏前压行程 ≤4×600 脉冲 @120rpm = 375ms，裕量 4× */
#define CFG_TRAY_HOME_TIMEOUT_MS                                                 \
    25000u                        /* 必须 > 一圈：120000 脉冲 @480rpm = 4.7s（裕量 5×） */
#define CFG_TRAY_DIR_FORWARD 0x00 /* 标反 = 全部静默镜像放错盒 */

/* ★ 转盘回零后的对格偏移（2026-10-01 台架实测）：凸块索引零位与格中心差约 10°——
   定零后**继续正向转这么多度**才是盒1 正对投放位（同 XY"归零后退 3mm"的驻留位思路；
   首版 15° 实测偏多，改 10°）。附带：驻留位在开关外，不长期压着原点开关。
   脉冲记账原点仍 = 索引沿（实现见 Motion_TrayHome 相位4），重装凸块/托盘后重新实测 */
#define CFG_TRAY_HOME_OFFSET_DEG 10u

/* 安全守护 */
#define CFG_SAFETY_PERIOD_MS 1u

/* ★ 货物类别 ID（2026-09-30 视觉组口径）：上位机**不再拆颜色×形状**，直接给"类别"这一个
   数字 **1~6**，放在帧的原**颜色字节**里。类别号与
   **串口屏第 1~6 行**、**转盘盒号**一一对应：
     类别 -> 盒号   = sort_task.c 的 s_category_to_bin[6]（抽签后改成一个 0~5 的排列）
     类别 -> 屏上行 = 类别号本身（display_task.c 直接当行号用，不再有第二张表）
   ★ 形状字节（2026-10-01 视觉组口径）：0~3 有效，**不参与分类**，只用来选夹持方案——
   **0 = 默认方案**（不在三种特调之内：直接用兜底 CFG_CLAW_HALF_US / CFG_CLAW_CLOSE_US）；
   **1 = 立方体、2 = 12面体、3 = 六棱柱**（三种特调，参数见下方 CFG_GRIP_* 表）。
   有效帧判据 = 类别 ∈ 1~6 **且** 形状 ∈ 0~3；任一项越界一律当"本帧无目标"（口径不一
   时的表现 = "链路活着但一直无目标、机器不动"，宁可不抓，绝不静默放错盒/夹错方案） */
#define CFG_NUM_CATEGORIES 6
#define CFG_NUM_SHAPES     3 /* 特调形状个数（不含默认 0） */

/* 舵机：通道由原理图固定，改 CH1/CH2 会发到没有舵机的 PB6/PB7 上。
   档位值 = 2026-09-26 台架实测定稿 + 2026-10-01 二次标定 + **2026-10-01 三次标定
   （夹爪重装后全档重标，含三套形状特调回填）**（机构同一套，脉宽↔角度跟舵机+机构走，
   与板无关）。
   ★ 抓取时序（机构构造决定的硬约束）：巡航全程夹爪必须 OPEN（否则撞托盘边/旁边物块）
   → 到目标 XY、下放之前 OPEN→初夹 → 升降放到底 DOWN 之后才允许夹紧
   → 抬回 UP。sort_task 的投放/抓取流程照此序，别把"夹紧"提前到下放途中。
   ★ 初夹/夹紧按**形状 1~3** 各一套（见下方抓取方案表）：不同物块需要的夹持角度不同，
   2026-09-26 的单一定稿值曾是三种形状的折中值——**2026-10-01 已按形状逐套实测定稿** */
#define CFG_SERVO_TIM     htim4
#define CFG_LIFT_UP_US   1600  /* 完全升起（到顶）——夹爪重装后三次标定：1200 → 1600。
                                  ★升降只有两档、与形状无关 = 三种特调共用的值之一 */
#define CFG_LIFT_DOWN_US  1300 /* 完全放下（放到底）——夹爪重装后三次标定：900 → 1300；
                                  夹紧只能在放到底之后做 */
#define CFG_CLAW_OPEN_US  1250 /* 完全展开 = 巡航安全形 = 上电位 —— 夹爪重装后三次标定：1000 → 1250。
                                  ★三种特调共用的值之一（初夹/夹紧才按形状走 CFG_GRIP_* 表） */
#define CFG_CLAW_CLOSE_US 1950 /* 兜底夹紧（形状 0 = 默认方案；抓取流程走下方方案表） */
#define CFG_CLAW_HALF_US  1750 /* 兜底初夹（形状 0 = 默认方案，作废轻放同值） */
#define CFG_SERVO_MOVE_MS 400
#define CFG_CLAW_CLOSE_MS 600
/* 原始脉宽安全窗（clamp 在 servo.c 的写入处）：防止配置写坏值把舵机顶死 */
#define CFG_SERVO_MIN_US  400u
#define CFG_SERVO_MAX_US  2600u

/* ★ 形状 -> 夹持方案（2026-10-01 视觉组口径）：形状**不参与分类**（进哪个盒只由类别定），
   只用来选这套夹持参数——不同形状的物块需要的夹紧角度不同。
   **形状 0 = 默认方案**（不查此表，直接用 CFG_CLAW_HALF_US / CFG_CLAW_CLOSE_US 兜底值）；
   **1 = 立方体、2 = 12面体、3 = 六棱柱**；数组下标 = 形状 - 1。
   ★2026-10-01 用户实测回填（按形状逐套，不再是折中值）：1 立方体 1900/2000、
   2 十二面体 1900/2000、3 六棱柱 1800/1950（初夹/夹紧，µs）。
   ⚠️ 改这里必须同步台架 dbg_config.h 的 DBG_CFG_GRIP_*（同一套机构） */
#define CFG_GRIP_HALF_US  {1900u, 1900u, 1800u} /* 下放前初步夹住：立方体/12面体/六棱柱 */
#define CFG_GRIP_CLOSE_US {2000u, 2000u, 1950u} /* 放到底后夹紧：立方体/12面体/六棱柱 */

/* 投放区（2026-10-01 台架实测标定，台架同值）：X=-70、Y=-187（= Y 工作区下限）。
   投放点同时 = 读帧位（龙门停这里不挡相机，见 CFG_VISION_SETTLE_MS 说明） */
#define CFG_DROP_X_MM (-70.0f)
#define CFG_DROP_Y_MM (-187.0f)

/* 待机点 = 归零驻留位 = 坐标系原点（回零流程的最后一步就是退到这里，
   MoveXYBlocking 是零距离空走；正常分拣循环不再回待机点） */
#define CFG_PARK_X_MM 0.0f
#define CFG_PARK_Y_MM 0.0f

/* ★ 工作包络（第三象限，与台架调试工程同框，2026-10-01 台架实测标定）：
   X ∈ [-135, 0]（-135 = 龙门可达极限，用户实测；旧占位 8/360 作废）
   Y ∈ [-187, 0]（-187 = 投放点 = 下限，再向下不需要）
   越界目标一律拒绝、一字节都不发，由调用方作废本件 */
#define CFG_WORK_X_MIN_MM (-135.0f)
#define CFG_WORK_X_MAX_MM 0.0f
#define CFG_WORK_Y_MIN_MM (-187.0f)
#define CFG_WORK_Y_MAX_MM 0.0f

/* ★ 夹爪延伸夹取带（2026-10-01 台架口径）：物块可出现在 X ∈ [-160, -135)——龙门停在
   -135 时夹爪构造仍够得着并夹住。分拣目标 X 超出软限位时**钳位到 -135 就近夹取**
   （见 sort_task.c）；比 -160 还远的作废本件（夹爪延伸也够不着） */
#define CFG_XY_X_CLAMP_MM (-160.0f)

/* （待机点 CFG_PARK_* 定义已上移到投放区一节：= 坐标系原点 (0,0)） */

/* TJC 串口屏：屏工程是**静态 6 行统计表**（序号/图片/货物名称由屏工程画死，**行序 = 类别 ID
   1~6**），主控只写每行末尾的"成功数量"（每类单独计数，不写总数、不写名称/图片 ——
   2026-09-30 现场屏口径）。屏上行数 = CFG_NUM_CATEGORIES，行号 = 类别号，无需再对表。
   控件名 = CFG_TJC_VAR_CNT_BASE + 数字(CFG_TJC_CNT_FIRST + 行下标)，缺省 n1~n6
   （第 1 行 n1 …… 第 6 行 n6）；屏工程改了名字/起始号就改这两处，名字对不上 = 整列不刷新 */
#define CFG_TJC_VAR_CNT_BASE "n"
#define CFG_TJC_CNT_FIRST    1
#define CFG_TJC_NAME_MAX     12u

/* ★ 屏自检开关：1 = 上电时把 6 行**分别写成本行行号**（第 1 行写 1 …… 第 6 行写 6），
   而不是写 0 —— 一眼就能核"控件名对不对、行序对不对"，顺带把 USART6/CN2/波特率整条链验了
   （写 0 是看不出问题的：屏上本来就是 0）。
   判据：屏上"成功数量"列自上而下出现 1,2,3,4,5,6 = 全对；某行没动 = 那个控件名不存在。
   ⚠️ 正式运行必须回 0：它会让屏先显示这组假数，直到第一件投放后"6 行全量重写"才被纠正 */
#define CFG_TJC_SELFTEST_ON_BOOT 0

/* 硬件映射编译期核对：这组三元组散落在本文件、usart.c、hal_msp.c、gpio.c、.ioc、
   bsp_uart.c 六处，任何一处单独改错都不会报错，只表现为"某个外设完全没反应"，
   所以在这里钉死 */
#if CFG_UART_SCREEN != 0 || CFG_UART_K230 != 1 || CFG_UART_SPARE != 2 || \
    CFG_UART_MOTOR_A != 3 || CFG_UART_MOTOR_B != 4 || CFG_UART_TRAY != 5
#error "CFG_UART_* 枚举顺序变了：bsp_uart.c 的 s_huart[] 按此顺序索引，必须同步改"
#endif

/* 只能用编译期断言，不能写成 #if：HAL 的 GPIO_PIN_n 带强制转换，预处理阶段不是
   合法常量表达式（真机 52 个 error）；GPIO_PIN_RESET 是枚举量，#if 里两边都算 0 */
#define SORT_CFG_ASSERT(name, cond) typedef char sort_cfg_assert_##name[(cond) ? 1 : -1]

SORT_CFG_ASSERT(x_pin_main_h, CFG_LIMIT_PIN_X == X_LIMIT_Pin);
SORT_CFG_ASSERT(y_pin_main_h, CFG_LIMIT_PIN_Y == Y_LIMIT_Pin);
SORT_CFG_ASSERT(tray_pin_main_h, CFG_ORIGIN_PIN_TRAY == TRAY_ORIGIN_Pin);
SORT_CFG_ASSERT(x_pin_schematic, CFG_LIMIT_PIN_X == GPIO_PIN_1);
SORT_CFG_ASSERT(y_pin_schematic, CFG_LIMIT_PIN_Y == GPIO_PIN_5);
SORT_CFG_ASSERT(tray_pin_schematic, CFG_ORIGIN_PIN_TRAY == GPIO_PIN_0);
SORT_CFG_ASSERT(limit_active_level, CFG_LIMIT_ACTIVE_LEVEL == GPIO_PIN_RESET);

#undef SORT_CFG_ASSERT

#endif