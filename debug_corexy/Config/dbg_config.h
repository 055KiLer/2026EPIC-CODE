/**
 * @file    dbg_config.h
 * @brief   ★ 本调试工程的唯一接线与参数文件（换板/改线只改这里）
 * @note    ★ 2026-09-30 起：产品 PCB 已修复，调试固件改跑**产品板**，
 *          接线与产品工程（`code/`）**完全一致**（此前的「物创1」板临时接线作废）：
 *            1) CoreXY 两台驱动器**各占一路 UART**（M1=USART3=A，M2=UART5=B）——
 *               不再是一条总线挂多台；帧内地址字节仍是该台驱动器的 ID（一条线一台，可广播）
 *            2) 舵机 = TIM4 CH3/PB8(H4 升降) + CH4/PB9(H1 夹爪)，与产品原理图一致
 *            3) 控制台 = UART4 PA0/PA1（产品板 CN1 预留口）
 *            4) 限位/原点 = PB1(X)/PC5(Y)/PB0(转盘)，与产品 gpio.c 同脚同配置
 *          转盘 57 步进（M3=USART1 PB6/PB7）本工程暂未接入；接的时候开一路即可
 */
#ifndef __DBG_CONFIG_H
#define __DBG_CONFIG_H

#include "main.h"
#include "usart.h" /* 句柄 huart3 / huart5 / huart4 的声明，下面的宏要用 */

/* ================= 1. 步进电机（两台各占一路，产品接线） ================= */
/* A 台 = 产品板 M1 座 = USART3 PC10(TX)/PC11(RX)；B 台 = M2 座 = UART5 PC12(TX)/PD2(RX)
   波特率 115200（三来源核对过的步进总线口径）
   ★ 寻址定调（2026-09-30 用户确认）：一条线只有一台驱动器，**全部直接广播 0x00，
   不再搞单播/地址采纳**——与产品 zdt_motor.c 完全同口径（产品三台全用广播）。
   ★ 应答定调：本批驱动器**对广播不回包**（ID 不是 1 时广播帧无应答是实测现象），
   所以运动路径一律**只发不收**（产品同口径）；`ack`/`scan`/`send`/`j <数字地址>`
   保留为单播诊断工具（驱动器对自己的 ID 会回包），运动流程不依赖它们 */
#define DBG_CFG_MOTOR_A_UART      huart3
#define DBG_CFG_MOTOR_A_UART_INST USART3
#define DBG_CFG_MOTOR_B_UART      huart5
#define DBG_CFG_MOTOR_B_UART_INST UART5
#define DBG_CFG_MOTOR_BAUD        115200u

/* ---- 1b. 转盘 57 步进（M3 座 = USART1 PB6/PB7，2026-09-30 接入） ---- */
/* 转盘同样广播 0x00。减速比/每格脉冲/速度等全部直接用产品 sort_config.h 的 CFG_TRAY_* 常量 */
#define DBG_CFG_TRAY_UART      huart1
#define DBG_CFG_TRAY_UART_INST USART1

/* ★ 转盘速度字段系数（与 XY 的 spu **独立**）：1 = 字段即 RPM；10 = 字段是 0.1RPM。
   57 驱动器与 42 可能不是同一口径，所以单独一个旋钮（运行时 `tspu 10` 改，不用重烧）。
   判定法：`j t cw 30 3200 255`（电机一圈）= 秒表 ≈2s → 1；≈20s → 10。
   ★ 它同时缩放转盘的**所有等待超时**（相位1 覆盖一圈的超时、退开/脱开的等待）——
   这正是 2026-09-30 thome 两次误报失败的根因所在 */
#define DBG_CFG_TRAY_SPEED_SCALE_DEFAULT 1u

/* ★ 转盘回零后的对格偏移（2026-10-01 用户实测）：凸块索引零位与格中心差约 **10°**——
   回零定零后**继续正向（顺时针）转这么多度**才是盒1 正对投放位（同 XY"归零后退 3mm"的驻留位思路）。
   （首版 15° 用户实测偏多，改 10°；还差就小步改这个宏重烧，格宽 60°，1° 级精度足够。）
   附带好处：转盘驻留时**不压着原点开关**（无磨损、无沿计数噪声）。
   ⚠️ 只在整格内有意义（<60°）；重装凸块/托盘后重新实测；回零后 `lim` 原点电平应=1（开） */
#define DBG_CFG_TRAY_HOME_OFFSET_DEG 10u

/* ★ 过冲回找窗口的搜索上限（2026-10-01 二次提速后实机现象，与产品 CFG_TRAY_REENTRY_PULSES
   同口径 = 8×600）：480rpm 撞停的过冲可能把凸块直接送过开关作用弧——停车点落在开关
   身后、电平=松开，"反向脱开"会立即假成功，慢压相就得追将近一整圈（"白转一圈"）。
   处置：停车点已松开时反向慢转 4800 脉冲（14.4°）回找窗口，压上即停再走常规退出；
   走满没压上=假沿（停车点在窗口前），直接进慢压相。两条路都不用降速 */
#define DBG_CFG_TRAY_REENTRY_PULSES 4800u

/* ★ 转盘速度百分比（`tspd`，默认 400 = 用户 2026-10-01"还是慢，再翻倍试试"）：
   给"粗寻标 120 / 对格 60 / 慢压 30"统一乘的倍数 → 实际 480/240/120rpm：
   寻标一圈 4.7s、对格单格 1.6s（此前 200% = 240/120/60，再翻倍）。
   ★ 它只影响**声音/节拍**，定下来的值要写回产品 sort_config.h 的 CFG_TRAY_*_RPM；
   转盘的所有等待超时会按 100/pct 反向缩短（寻标超时 25000×100/400+500 = 6.75s > 一圈 4.7s ✓）。
   ★ 最没验证过的一档 = 寻标 480rpm：台架先 thome×3 + tb 对格，有丢步/堵转迹象就
   `tspd 200` 运行时回退（不用重烧） */
#define DBG_CFG_TRAY_SPEED_PCT_DEFAULT 400u

/* 广播地址：地址字节 0x00 = 该线上的驱动器都执行（不需要知道它的 ID）。
   ★ 全工程运动/停止/使能一律广播（见 §1 寻址定调）；`id a/b/t <地址>` 仅留作
   单播诊断实验用，`id x 0` 可随时回到广播 */
#define DBG_CFG_ADDR_BROADCAST 0x00u

/* 位置帧的加速度字节：
   ★ 取 0xFF —— 这是**同一块板、同一批驱动器上实测可用**的取值
     （物创 `drive/stepper_drive.c` 里该字段固化 0xFF，注释写"固定"）。
     产品 `code/` 用的是 10（CFG_ZDT_ACCEL），但那个值从没上过真机；
     调试时想对比就逐条覆盖：`j 0 cw 30 3200 10` */
#define DBG_CFG_ACCEL_DEFAULT 255u

/* V2 协议 16 字节位置帧的加速/减速度（两字段共用），单位 RPM/s。
   说明书示例用 511；45rpm 约 0.09s 加速完成，台架先用它，跑顺后再标定 */
#define DBG_CFG_ACC16_RPM_PS 511u
/* 位置帧的相对/绝对与多机同步字节（产品都是 0：相对、立即执行） */
#define DBG_CFG_POS_MODE_RELATIVE 0x00u

/* 等回包的时长（ms）：驱动器收到正确命令会立刻回 [地址][功能码][02][6B]
   （物创 `code/ZDT_drive.c` 里 `HAL_UART_Receive(..., 4 字节, 2ms)` 就是这么收的）
   → 这是整条链路唯一"有应答"的证据，比看电机转不转灵敏得多 */
#define DBG_CFG_ACK_WAIT_MS 40u

/* ★ 上电自动体检（6 档波特率 × 地址 1~8 找"有回包"的组合）：
   2026-09-30 定调全广播+不读应答后，ID 扫描对运动流程已无用——**默认关闭**。
   需要找驱动器 ID（如用官方上位机改配置前）手动跑 `scan`；要换波特率档 `baud` */
#define DBG_CFG_BOOT_AUTOSCAN 0
/* 体检重跑间隔（ms）：只在上电后【一个控制台字节都没收到】时才重跑，
   于是任意时刻打开串口都能看到一份完整报告；收到第一个字节就永久停 */
#define DBG_CFG_AUTOSCAN_REPEAT_MS 10000u

/* ★ 帧内「速度字段」的单位系数：真实转速 = 帧内数值 ÷ 这个系数。
   ⚠️ 产品与物创两条口径差 10 倍，且**两边都没上真机核过**：
        产品 code/  : 按 RPM 算（`Kin_RpmForSpeed` 出的就是 rpm，直接进帧）→ 系数 1
        物创 固件   : config_robot.h 该节标题直接写「运动速度 (0.1RPM)」，
                      且它的时间公式 z_time_s = 0.1mm数 × 30 ÷ 字段值
                      只有在"字段 = 0.1RPM"时才自洽                → 系数 10
   本工程默认 1（= 产品口径）：万一实际单位是 0.1RPM，也只是慢 10 倍；反过来的话
   会快 10 倍撞结构。上台架用秒表定：`j a cw 30 3200`（转一圈）≈2 s → 系数 1；
   ≈20 s → 系数 10（`spu 10`） */
#define DBG_CFG_SPEED_SCALE_DEFAULT 1u

/* ================= 2. 控制台串口（人敲命令的口） ================= */
/* ★ 产品板：UART4 = PA0/PA1 = **CN1 预留口**（与产品 sort_config.h 的 CFG_UART_SPARE
   同口同脚）——调试探针的 CDC 串口桥（COM 口）接这里：RX 接 PA0(TX)、TX 接 PA1(RX)、共地 */
#define DBG_CFG_CONSOLE_UART_INST UART4
#define DBG_CFG_CONSOLE_UART      huart4
#define DBG_CFG_CONSOLE_BAUD      115200u
/* 控制台引脚（PA1 = UART4_RX）：给"线路体检"用 —— 把 PA1 当输入采样，
   量 PC 侧发字节时有没有边沿，用来分清"线没接"还是"MCU 没收到" */
#define DBG_CFG_CONSOLE_RX_GPIO GPIOA
#define DBG_CFG_CONSOLE_RX_PIN  GPIO_PIN_1
#define DBG_CFG_CONSOLE_RX_AF   GPIO_AF8_UART4

/* 控制台行缓冲：一行最长 96 字符（超了丢弃整行），\r / \n / \r\n 都算行结束 */
#define DBG_CFG_CMD_MAX 96u

/* ================= 2b. 视觉线（USART2 = CN3）与屏线（USART6 = CN2） ================= */
/* ★ 与产品 sort_config.h 的 CFG_UART_K230 / CFG_UART_SCREEN 同口同脚（原理图 V1.0）：
      视觉 = USART2 PD5(TX)/PD6(RX)，115200 —— 产品口径**本侧只收不发**，CN3 只接 ROCK 5C
      屏   = USART6 PC6(TX)/PC7(RX)，115200 —— 只发不收（TJC 屏工程不改名就不回话）
   命令：`k230`（快照/live/sim/clr）、`scr`/`scrp`/`scrt`/`scrtest`；见 README §3 */
#define DBG_CFG_VISION_UART      huart2
#define DBG_CFG_VISION_UART_INST USART2
#define DBG_CFG_VISION_BAUD      115200u
#define DBG_CFG_SCREEN_UART      huart6
#define DBG_CFG_SCREEN_UART_INST USART6
#define DBG_CFG_SCREEN_BAUD      115200u

/* ★ 屏/视觉线被噪声预算关掉后的**自动重开间隔**：这两条线要常收（视觉是持续帧流），
   与电机线"发帧前按需再开"不同——线没接/悬空时最多每 2s 付一窗噪声预算，不会风暴 */
#define DBG_CFG_DEV_LINE_REARM_MS 2000u

/* 逐字符回显收到的字节 —— 排查"发命令没反应"最有用的一件工具：
     1 = 回显：可打印字符原样回显，**不可见字节打成 <0xNN>**
         （于是"行结束到底有没有到板上、是 \r 还是 \n"一眼可见）
     0 = 不回显（终端自带本地回显时输出更干净）
   链路确认没问题后就改 0 */
#define DBG_CFG_ECHO_RX 1

/* 打印是否同时镜像到 ITM(SWO/PB3)：Keil 调试会话里能同步看到一份 */
#define DBG_CFG_PRINT_MIRROR_ITM 1

/* ================= 3. 舵机（夹爪 / 升降） ================= */
/* ★ 产品板（2026-09-28 按原理图 V1.0 核对）：H1 = TIM4_CH4/PB9 = **夹爪**、
   H4 = TIM4_CH3/PB8 = **升降**，与产品 code/Core/Src/tim.c 完全一致。
   （旧「物创1」板的 PD12=CH1/PD15=CH4 接线随 PCB 修复一并作废——那块板
   PD12/PD14 是按键，与产品板引脚不同，不可混接）
   PWM 50Hz、1MHz 计数 → 比较值 = 脉宽微秒数 */
#define DBG_CFG_CLAW_TIM_CH TIM_CHANNEL_4 /* H1 / PB9 */
#define DBG_CFG_LIFT_TIM_CH TIM_CHANNEL_3 /* H4 / PB8 */

/* 档位脉宽（µs）：★2026-09-26 台架实测定稿 + 2026-10-01 二次标定（装好机构后 ±50µs
   扫行程，留了余量）+ **2026-10-01 三次标定（夹爪重装后全档重标：完全展开 1000→1250、
   完全升起 1200→1600、完全放下 900→1300、兜底初夹 1600→1750、兜底夹紧 1750→1950，
   并按形状回填三套特调）**。数值跟机构走，与早期占位值（500/2500 等）完全不同是正常的 */
#define DBG_CFG_CLAW_OPEN_US  1250u /* 完全张开（夹爪重装后三次标定：1000→1250）：巡航/越托盘
                                       必须用它，否则撞托盘边或旁边物块。★三种特调共用的值之一 */
#define DBG_CFG_CLAW_CLOSE_US 1950u /* 兜底夹紧（形状 0 = 默认方案；抓取走下方方案表） */
#define DBG_CFG_CLAW_HALF_US  1750u /* 兜底初夹（形状 0 = 默认方案，作废轻放同值） */
/* ★ 形状 -> 夹持方案（2026-10-01 视觉组口径）：形状**不参与分类**（进哪个盒只由类别定），
   只用来选这套夹持参数——不同形状的物块需要的夹紧角度不同。
   **形状 0 = 默认方案**（不查此表，直接用 DBG_CFG_CLAW_HALF_US / CLOSE_US 兜底值）；
   **1 = 立方体、2 = 12面体、3 = 六棱柱**；数组下标 = 形状 - 1。
   ★2026-10-01 用户实测回填（形状 1/2/3 各一套，不再是折中值）：
     1 立方体：初夹 1900 / 夹紧 2000   2 十二面体：初夹 1900 / 夹紧 2000
     3 六棱柱：初夹 1800 / 夹紧 1950（与兜底夹紧同值）
   ⚠️ 与产品 sort_config.h 的 CFG_GRIP_* 同步改 */
#define DBG_CFG_GRIP_HALF_US  {1900u, 1900u, 1800u} /* 立方体/12面体/六棱柱 */
#define DBG_CFG_GRIP_CLOSE_US {2000u, 2000u, 1950u} /* 立方体/12面体/六棱柱 */
#define DBG_CFG_LIFT_UP_US   1600u  /* 完全升起（到顶）——夹爪重装后三次标定：1200 → 1600。
                                       ★升降只有两档、与形状无关 = 三种特调共用的值之一 */

#define DBG_CFG_LIFT_DOWN_US  1300u /* 完全放下（放到底）——夹爪重装后三次标定：900 → 1300；
                                       夹紧只能在放到底之后做 */

/* ★ 抓取时序（机构构造决定的硬约束，2026-09-26 用户口述）：
   巡航全程夹爪 = OPEN → 到达目标 XY、下放之前 OPEN→初夹 → 升降放到底 DOWN
   → 才按形状方案夹紧 → 抬回 UP。
   ① 巡航不全开会撞托盘边界/旁边物块 ② 没放到底就夹紧会顶死。
   台架 `pick` 命令按这个时序复演（可选形状参 `pick X Y [mm/s [形状0~3]]（0=默认方案）`）；
   产品 sort_task 落地时照此序 */
/* ★ 上电安全位 = 完全展开（巡航安全形）。**直接引用 OPEN 档**而不是另抄一个数：
   重装标定时"上电位"与"全开档"必须同步，抄成两个字面量迟早漏改一处 */
#define DBG_CFG_CLAW_BOOT_US  DBG_CFG_CLAW_OPEN_US
/* 脉宽安全窗：命令行可达范围（防手误打到 0 或 5000 把舵机顶死） */
#define DBG_CFG_SERVO_MIN_US 400u
#define DBG_CFG_SERVO_MAX_US 2600u
/* 切档后等舵机走到位的默认时间 */
#define DBG_CFG_FW_STAMP "1001l" /* ★固件版本戳：横幅/stat 可见——"板上烧的哪个版本"一眼定，杜绝新旧混淆 */
#define DBG_CFG_SERVO_SETTLE_MS 400u

/* ================= 3.5 限位 / 原点微动开关 ================= */
/* ★ 与产品工程完全同脚（code/Core/Src/gpio.c：EXTI 下降沿 + 上拉，未触发=高、压下=低）：
      转盘原点 = PB0（EXTI0）  X 限位 = PB1（EXTI1）  Y 限位 = PC5（EXTI9_5）
   接法：微动开关 COM -> GND、NO -> 信号脚（NC 不接）。
   ★ 现在跑的就是产品板：PCB 上的 4.7k 外部上拉已在位（天线效应的硬件根治），
     内部上拉保持开启，二者并联无害。
   ⚠️ 引脚号（0/1/5）对应 stm32f4xx_it.c 里已开的三条 EXTI 中断线；
      换线只换端口宏与 NAME，引脚号别动（动了要同步改 NVIC 那三行） */
#define DBG_CFG_LIMIT_TRAY_PORT GPIOB
#define DBG_CFG_LIMIT_TRAY_PIN  GPIO_PIN_0
#define DBG_CFG_LIMIT_TRAY_NAME "PB0"
#define DBG_CFG_LIMIT_X_PORT    GPIOB
#define DBG_CFG_LIMIT_X_PIN     GPIO_PIN_1
#define DBG_CFG_LIMIT_X_NAME    "PB1"
#define DBG_CFG_LIMIT_Y_PORT    GPIOC
#define DBG_CFG_LIMIT_Y_PIN     GPIO_PIN_5
#define DBG_CFG_LIMIT_Y_NAME    "PC5"

/* 沿计数消抖门限（ms）：门限内的重复沿 = 同一次触发的弹跳，不计数。
   2026-09-26 台架实测依据：单次按压蹦 17~128 沿，且开关断开态下线路还会拾到
   自发 burst（直连 GND 后 60s 零事件 → 源头是弱上拉悬空段的天线效应+开关弹跳，
   硬件根治 = 产品设计的 4.7k 外部上拉，台架并上即同款）。
   ⚠️ 40ms 只用于台架"沿计数=按压次数"；产品回零的消抖/首沿闩锁是另一笔账
   （20ms@100mm/s≈2mm 过冲，见 调试开工手册 §8-1），PCB 修复后单独定 */
#define DBG_CFG_LIMIT_DEBOUNCE_MS 40u

/* ================= 4. 运动安全与速度 ================= */
/* 单段最大位移：命令行 typo（x 5000）时拒绝执行，别让龙门一路撞过去 */
#define DBG_CFG_MAX_MOVE_MM 200.0f

/* 速度上限（mm/s）：产品 2026-10-01 二次提速后 = 200（旧 100）；上限放到 220 放行
   与产品同档测试（此前 60 会把手动速度悄悄砍掉）。默认 100 = 产品旧档，稳妥 */
#define DBG_CFG_MAX_SPEED_MM_S 220.0f
#define DBG_CFG_DEFAULT_SPEED_MM_S 100.0f

/* 到位判定余量（ms）：与产品 CFG_MOVE_EXTRA_MS 同口径，由 Kinematics_* 内部用 */
#define DBG_CFG_MOVE_EXTRA_MS 200u

/* ================= 4b. 回零（homex/homey，X=PB1 Y=PC5） ================= */
/* ★ 2026-09-26 机械确认：两只开关装在**最大行程端**（X max / Y max），不在原点端！
   → 回零 = 向 +X/+Y 撞开关；触发点 = 最大行程端，原点从触发点反推。
   ★ 2026-09-30 用户定调：**简化成两相**（粗压撞开关即停 → 退一小段），
   删掉了"反向脱开确认 + 5mm/s 慢压定原点"两相——实现见 dbg_app.c 的 Dbg_App_HomeRun */
#define DBG_CFG_HOME_DIR_X        (+1)  /* +1 = 向 X+ 找开关（装在最大端）；若拆到原点端改 -1 */
#define DBG_CFG_HOME_DIR_Y        (+1)  /* +1 = 向 Y+ 找开关（装在最大端）；若拆到原点端改 -1 */
#define DBG_CFG_HOME_SPEED_DEFAULT 20u   /* 粗压 20mm/s（2026-10-01 用户定调"再快一倍"；触发沿→停
                                            实测 ~1.5ms → 过冲 ≈0.04mm，海绵余量 2.7cm 的零头。
                                            产品粗压设计值 = 40mm/s，台架仍保守一半。
                                            100mm/s 过冲也才 0.2mm——限制它的其实是撞击感，不是精度。
                                            不放心可单轴压回：`homex 10` */
#define DBG_CFG_HOME_SPEED_MAX     30u
#define DBG_CFG_HOME_SLOW_MM_S     5u    /* "起点已压着开关时先退开"这一下用的慢速 */

/* ================= 4c. 工作区软限位（`p`/`x`/`y`/`sq`/`pick` 等意图运动用） ================= */
/* ★ 坐标约定：**原点 = 归零后的驻留位**（触发点内退 DBG_CFG_HOME_BACKOFF_MM），
   工作区全在**第三象限**（X≤0 且 Y≤0），正方向是开关（禁止）；上界恒 0。
   ★ X 下限放宽（2026-10-01 用户实测：龙门可运动到 -135，旧值 -122 过严）：
   ★ Y 下限 = 投放点（再向下不需要）。
   ⚠️ 改 DBG_CFG_HOME_BACKOFF_MM 会让原点整体平移，下面两个数要**同位跟着改**！
      换机/重装机械后重新标定法：归零 → 手动走到投放点/机械端 → 读 `p` 的坐标写回这里 */
#define DBG_CFG_XY_MAX_MM   0.0f
#define DBG_CFG_XY_X_MIN_MM (-135.0f)
#define DBG_CFG_XY_Y_MIN_MM (-187.0f)

/* ★ 夹爪延伸夹取带（2026-10-01 用户提出）：物块可出现在 X ∈ [-160, -135)——龙门停在 -135
   时，夹爪构造仍够得着并夹住。`pick`（含 read 给出的坐标）的目标 X 超出软限位时
   **钳位到 -135 就近夹取**（打印提示，不拒绝）；比 -160 还远的照常拒绝（延伸也够不着）。
   手动 `p` 语义不变（X 下限同样是 -135）。产品移植时同款逻辑进 Sort_DoCycle */
#define DBG_CFG_XY_X_CLAMP_MM (-160.0f)

/* ★ 投放点坐标（2026-09-30 用户台架实测，新原点系）：X = -70、Y = -187（= Y 下限）。
   `drop [mm/s]` 一键转过去；抓取点/其余工位坐标待机械组回填后照此追加 */
#define DBG_CFG_DROP_X_MM (-70.0f)
#define DBG_CFG_DROP_Y_MM (-187.0f)

/* 找零行程上限：≥ 最长实际行程（Y 约 190）+ 余量。走满=没等到触发沿（开关没压到）。
   ★ 刻意取小：真挂住时"顶着推"的距离就是它（P0-6 教训——宁小勿大） */
#define DBG_CFG_HOME_TRAVEL_MM   220.0f

/* 触发后的退距（mm）：越小工作区越大，但必须保证开关可靠脱开。
   脱不开时固件会自动再退一份（共 2×）；仍不行就调大这个值 */
#define DBG_CFG_HOME_BACKOFF_MM   3.0f

/* ★ 上电自动归零（2026-10-01）：1 = 打完横幅/帮助后等 AUTO_HOME_SKIP_MS（期间控制台
   任意字节 = 本次跳过），自动跑 `home`（homex→homey→thome）——与产品"上电即启动、
   首圈 Sort_EnsureCalibrated 自动归零"同款。烧录/接线时不想让电机自己动：
   上电 3s 内发任意字节（一个回车即可），或把这里改 0 */
#define DBG_CFG_AUTO_HOME_ON_BOOT 1
#define DBG_CFG_AUTO_HOME_SKIP_MS 3000u

/* ================= 4d. 类别 -> 盒号（tb 用 1~6 直对类别 ID，2026-10-01） ================= */
/* ★ 盒号改用人类编号 **1~6**：**盒1 = thome 回零后对准投放位的那一格**，沿正转方向依次
   盒2~盒6（现场照此在转盘上贴标）。`tb <类别>` 一句到位：类别号经本表映射到物理盒。
   缺省恒等 = 类别 n 进盒 n；**抽签后只改两处且必须一致**：
     ① 这一行   ② 产品 code/User/App/Src/sort_task.c 的 s_category_to_bin[6]
      （那边是 0~5 下标数组、值 = 盒号-1，人机接口无感） */
#define DBG_CFG_CATEGORY_TO_BIN { 1u, 2u, 3u, 4u, 5u, 6u }

/* ================= 5. 待核实清单（产品板首次上台架逐条打勾） ================= */
/*  [ ] 1) A 台驱动器接 M1 座（USART3 PC10/PC11）、B 台接 M2 座（UART5 PC12/PD2），
          线序 TX↔RX 交叉、与主控共地；A/B 谁在左谁在右由装机决定，`x 30` 核方向
   [ ] 2) 每条线上的驱动器地址：上电自检自动采纳（看 [BOOT] 行），`scan` 可复核
   [ ] 3) 驱动器固件为 Emm V5.0（位置帧 13 字节的前提）
   [ ] 4) 控制台接 CN1（UART4 PA0/PA1）；线序 TX↔RX 交叉、共地
   [ ] 5) 舵机：夹爪插 H1(PB9/CH4)、升降插 H4(PB8/CH3)——产品板丝印为准
   [ ] 6) 实验电源：12V 给 42 电机（与产品一致）；先只接一台电机
   [ ] 7) 极性与急停：本工程没有守护任务，电源开关就是唯一急停
   [ ] 8) 驱动器是否应答（ack/上电自检的回包）：不应答也不代表链路不通，要靠手拧
   [ ] 9) 速度字段单位：RPM 还是 0.1RPM（秒表法，见 DBG_CFG_SPEED_SCALE_DEFAULT）
   [ ] 10) 驱动器控制模式：必须是"串口/通讯"模式，不能是脉冲+方向
   [ ] 11) 方向字节高低：ZDT 手册 0x00=CW/0x01=CCW（产品口径），物创固件是反的
   [ ] 12) 换板后首次 homex/homey 前先 `lim` 逐个按压核三路开关（X=PB1/Y=PC5/转盘=PB0） */

#endif