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
   ★ 本工程坐标系 = 第三象限（原点 = 归零驻留位，X≤0/Y≤0）：
     2026-10-01 首次标定回填：视觉原点 ≈ 夹爪走到 (-80, -45) 处（用户实测，"大概"级精度，
     `k230` 换算从此为真值）；SIGN 符号仍默认 +1/+1，待托盘四角放货核镜像后定稿。
     精修法：四角各放一件看落点——恒定偏差→平移修 ORIGIN；镜像→翻对应 SIGN（±1）
   ★ 2026-10-01 抓取偏移修正（实抓迭代定值，与产品同步，爪要往 + 走则原点 +）：
     Y：-45 → -60 → **-57.5**（再向 +Y 补 2.5）；X：-80 → -75 → **-70**（再向 +X 补 5） */
#define CFG_VISION_ORIGIN_X_MM  (-70.0f)
#define CFG_VISION_ORIGIN_Y_MM  (-57.5f)
#define CFG_VISION_SIGN_VY_TO_X (+1.0f) /* 机械 X = ORIGIN_X + SIGN × 帧.y_mm */
#define CFG_VISION_SIGN_VX_TO_Y (+1.0f) /* 机械 Y = ORIGIN_Y + SIGN × 帧.x_mm */

/* ★ 读帧时机（2026-09-30 现场口径）：归零/待机时龙门正压托盘上方、挡相机，取的帧不可信
   ——读帧位 = 投放点上方，到位后先静默这段再清缓存取帧（台架对应 `read` 命令）。
   症状判据：出现"抓着刚取走物块的旧坐标"（抓空、计数虚增）就把它调大 */
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
#define CFG_HOME_TRAVEL_MM   400.0f
#define CFG_BACKOFF_MM       3.0f
/* 触发点在工作坐标系里的位置（回零成功后写入 s_x/s_y 的值）：机械上=开关压下点离
   工装原点角的距离，⚠️ 待卷尺标定回填；现值=行程-20 的占位（自带 20mm 防撞余量）。
   原点角在远离限位的一侧，工作区在 [CFG_WORK_MIN_MM, 触发点-20] 内，全为正坐标 */
#define CFG_HOME_X_MAX_MM (CFG_HOME_TRAVEL_MM - 20.0f)
#define CFG_HOME_Y_MAX_MM (CFG_HOME_TRAVEL_MM - 20.0f)

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

#define CFG_XY_SPEED_MM_S   100.0f
#define CFG_HOME_SPEED_MM_S 40.0f
#define CFG_HOME_SLOW_MM_S  10.0f

#define CFG_MOVE_EXTRA_MS      200   /* 加减速时间余量 */
#define CFG_HOME_WAIT_EXTRA_MS 1000u

/* ZDT 电机 */
/* 加速度字节：0xFF 是**同一批驱动器上实测可用**的取值（物创固件在该字段固化 0xFF）；
   10 是从没上过真机的保守值。实机首轮建议先用 0xFF（已验证 > 未验证），
   若大龙门丢步再往小调 —— 改这一处即可 */
#define CFG_ZDT_ACCEL          10

/* ★ 帧内「速度字段」的单位系数：真实转速 × 该系数 = 帧内数值。
   产品按 RPM 算（`Kin_RpmForSpeed` 出的是 rpm，直接进帧）→ 系数 1；
   台架实测若发现"实际比估算慢约 10 倍"，说明驱动器按 0.1RPM 解释该字段，
   把这里改成 10 即可（只影响设备层写帧，算法层不动）。判定法见
   `台架调试结论与代码修改方案.md` §4：`j a cw 30 3200` 转一圈，秒表 ≈2s / ≈20s */
#define CFG_ZDT_SPEED_FIELD_SCALE 1u

#define CFG_UART_TX_TIMEOUT_MS 100

/* ★ 上电驱动器自检：逐台发使能帧并读应答（`Zdt_Probe`）。
   为什么值得做：驱动器可能"完全不应答"（数据线/地址/波特率）或"只应答不执行运动"
   （控制模式被设成脉冲+方向）—— 这两种都只有自检能发现，而比赛时静默失效等于当场崩 */
#define CFG_SELFTEST_ON_BOOT   1
#define CFG_SELFTEST_TIMEOUT_MS 40u

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
    60                                     /* 总比 37.5 后约 1.6 输出 RPM，单格 6.25s；调速旋钮 \
                                              见调试工程 `tspd`，定稿值回填这里 */
#define CFG_TRAY_SEEK_RPM            120
#define CFG_TRAY_APPROACH_RPM        30
#define CFG_TRAY_BACKOFF_PULSES      600u  /* = 转盘 1.8°；必须 > 凸块作用宽度，
                                              不够时固件会自动再退一份（2×）兜底 */
#define CFG_TRAY_APPROACH_TIMEOUT_MS 4000u /* ★ 1500→4000（2026-10-01）：台架等待按
                                              tspd 百分比缩放（400% 时 4000→1500ms，@120rpm
                                              = 28.8° 行程预算 = 产品同口径）；相位3 在
                                              "过冲回找没找到窗口"后要多走 14.4° 才碰沿，
                                              旧 1500（缩放后 875ms=16.8°）不够 */
#define CFG_TRAY_HOME_TIMEOUT_MS                                                 \
    25000u                        /* 必须 > 一圈：120000 脉冲 @120rpm = 18.75s（总比 37.5） */
#define CFG_TRAY_DIR_FORWARD 0x00 /* 标反 = 全部静默镜像放错盒 */

/* 安全守护 */
#define CFG_SAFETY_PERIOD_MS 1u

/* 货物类别 ID（2026-09-30 视觉组口径，与产品同步）：上位机不拆颜色×形状，直接给类别 1~6
   （放帧的原"颜色"字节）。调试固件不跑视觉，这里只是与产品保持一份真源 */
#define CFG_NUM_CATEGORIES 6
/* ★ 特调形状个数（2026-10-01 视觉组口径，与产品同步）：形状 **0 = 默认方案**（不在三种
   特调之内），**1 = 立方体、2 = 12面体、3 = 六棱柱**。台架校验/命令的形状参上限取自这里
   （dbg_vision.c 收帧校验、dbg_app.c 的 pick、dbg_console.c）——缺它会直接编译不过 */
#define CFG_NUM_SHAPES 3

/* 舵机：通道由原理图固定，改 CH1/CH2 会发到没有舵机的 PB6/PB7 上。
   档位值 = 2026-09-26 台架实测定稿（机构同一套，脉宽↔角度跟舵机+机构走，与板无关）。
   ★ 抓取时序（机构构造决定的硬约束）：巡航全程夹爪必须 OPEN（否则撞托盘边/旁边物块）
   → 到目标 XY、下放之前 OPEN→HALF(初步夹住) → 升降放到底 DOWN 之后才允许 CLOSE(夹紧)
   → 抬回 UP。sort_task 的投放/抓取流程照此序，别把"夹紧"提前到下放途中 */
#define CFG_SERVO_TIM     htim4
/* ⚠️ 本文件是产品 code/Config/sort_config.h 的镜像（调试舵机档位真源在 dbg_config.h
   §3，这里只保证两份拷贝不打架）；改产品值时同位改这里 */
#define CFG_LIFT_UP_US   1650  /* 完全升起（到顶）——夹爪重装后三次标定：1150 → 1600 → 1650 */
#define CFG_LIFT_DOWN_US  1300 /* 完全放下（放到底）——夹爪重装后三次标定：900 → 1300 */
#define CFG_CLAW_OPEN_US  1250 /* 完全展开 = 巡航安全形 = 上电位 —— 夹爪重装后三次标定：1100 → 1250 */
#define CFG_CLAW_CLOSE_US 1950 /* 兜底夹紧（形状 0）——夹爪重装后三次标定：1750 → 1950；与产品同值 */
#define CFG_CLAW_HALF_US  1750 /* 兜底初夹（形状 0，作废轻放同值）——夹爪重装后三次标定：1600 → 1750 */
#define CFG_SERVO_MOVE_MS 400
#define CFG_CLAW_CLOSE_MS 600

/* 投放区：⚠️ 待机械组回填。工作坐标系原点角 = 远离限位的一侧（触发点在
   (CFG_HOME_X_MAX_MM, CFG_HOME_Y_MAX_MM)），回填前装置无法完成任何一件分拣（这是故意的） */
#define CFG_DROP_X_MM 0.0f
#define CFG_DROP_Y_MM 0.0f

#define CFG_WORK_MIN_MM   8.0f
#define CFG_WORK_MAX_X_MM (CFG_HOME_X_MAX_MM - 20.0f) /* 触发点内留 20mm，别顶着开关干活 */
#define CFG_WORK_MAX_Y_MM (CFG_HOME_Y_MAX_MM - 20.0f)

/* 待机点：2026-09-28 起与台架口径一致——归零后停在触发点内 20mm（再往里走是盲区，
   靠卷尺标定后再改）。不能贴着触发点停，否则一直保持限位触发，抖动全被判成撞限位 */
#define CFG_PARK_X_MM (CFG_HOME_X_MAX_MM - 20.0f)
#define CFG_PARK_Y_MM (CFG_HOME_Y_MAX_MM - 20.0f)

/* TJC 串口屏：屏工程是**静态 6 行统计表**（序号/图片/货物名称由屏工程画死，行序 = 货物类型
   ID 1~6，见产品 display_task.c 的类型表），主控只写每行末尾的"成功数量"（每类单独计数，
   不写总数、不写名称/图片 —— 2026-09-30 现场屏口径）。
   控件名 = CFG_TJC_VAR_CNT_BASE + 数字(CFG_TJC_CNT_FIRST + 行下标)，缺省 n1~n6
   （第 1 行 n1 …… 第 6 行 n6）；屏工程改了名字/起始号就改这两处，名字对不上 = 整列不刷新 */
#define CFG_TJC_VAR_CNT_BASE "n"
#define CFG_TJC_CNT_FIRST    1
#define CFG_TJC_NAME_MAX     12u

/* ★ 屏自检开关（与产品同步）：1 = 上电把 6 行分别写成行号 1..6（核控件名与行序），
   正式运行必须回 0。调试固件不接屏，这里只是保持一份真源 */
#define CFG_TJC_SELFTEST_ON_BOOT 0
/* 屏上行数 = 现场抽签的货物种类数（= 产品 display_task.c 类型表项数） */
#define CFG_DISPLAY_TYPE_COUNT 6u

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