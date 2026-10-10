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
#include "usart.h"      /* 句柄 huart3 / huart5 / huart4 的声明，下面的宏要用 */
#include "sort_config.h" /* ★2026-10-10：产品标定（薄壳直含产品文件）——本文件的标定项
                            一律**引用**它（DBG_CFG_XXX CFG_XXX），不再抄数值，见文件中部注解 */

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
#define DBG_CFG_TRAY_HOME_OFFSET_DEG CFG_TRAY_HOME_OFFSET_DEG

/* ★ 2026-10-11 回零换代为"撞—退—再撞"（与产品同款）：旧"慢压渐压定零"间歇触发不了
   开关（实测回零失败主因），新流程 = 粗撞找凸块 → 反向退 CFG_TRAY_REHOME_BACKOFF_PULSES
   （产品宏，4.8°）走满 → 正向再撞，撞上沿 = 零位。旧过冲回找（REENTRY）整套删除——
   实测撞停点始终在凸块窗口内（电平=压下），回找从未触发 */
#define DBG_CFG_TRAY_REENTRY_PULSES CFG_TRAY_REENTRY_PULSES

/* ★ 转盘速度百分比（`tspd`，默认 100 = **产品档**：给"寻标 480 / 对格 240 / 慢压 120"
   乘 100% ⇒ 逐相与产品 CFG_TRAY_*_RPM 同速。2026-10-10 用户口径定案：
   诊断/校准（thome×3、tb 对格、过冲回找、丢步观察）**必须在 100% 下做**——别的百分比
   只是"试节拍/试声音"，用它校出来的结论不迁移到产品。
   ★ 历史坑：早年产品名义还是 120/60/30 时，台架默认 400%（= 现在的产品值）并回填进产品；
   后来台架调用点改成直接引用产品宏，百分比却没归位 100 ⇒ 台架一度比产品快 2~4 倍。
   本次修正：调用点引用产品宏 + 默认 100 = 两工程逐相同速。
   ★ 转盘等待超时会按 100/pct 反向缩短（寻标超时 25000×100/100+500 = 25.5s > 一圈 4.7s ✓）；
   丢步/堵转就 `tspd 200` 回退只当临时诊断手段（结论仍以 100% 为准） */
#define DBG_CFG_TRAY_SPEED_PCT_DEFAULT 100u

/* 广播地址：地址字节 0x00 = 该线上的驱动器都执行（不需要知道它的 ID）。
   ★ 全工程运动/停止/使能一律广播（见 §1 寻址定调）；`id a/b/t <地址>` 仅留作
   单播诊断实验用，`id x 0` 可随时回到广播 */
#define DBG_CFG_ADDR_BROADCAST 0x00u

/* 位置帧的加速度字节：**= 产品 CFG_ZDT_ACCEL**（0xFF；同一批驱动器实测可用值，
   2026-10-01 产品也已改成 0xFF）。想对比别的档就逐条覆盖：`j 0 cw 30 3200 10` */
#define DBG_CFG_ACCEL_DEFAULT CFG_ZDT_ACCEL

/* V2 协议 16 字节位置帧的加速/减速度（两字段共用），单位 RPM/s。
   说明书示例用 511；45rpm 约 0.09s 加速完成，台架先用它，跑顺后再标定 */
#define DBG_CFG_ACC16_RPM_PS 511u
/* 位置帧的相对/绝对与多机同步字节（产品都是 0：相对、立即执行） */
/* ★ 位置帧「运动模式」字节（**= 产品 CFG_ZDT_POS_MODE**，0x00 = 相对上一目标位置）：
   2026-10-08 台架**证伪** 0x02（切过去 x/y 出现侧向移动）= 这批固件的 0x02 不是"相对当前位置"，
   两处配置同时退回 0x00；现场"坐标系整体偏 +Y"的根因（中断运动留旧目标）仍未根治，
   现靠"少中断 + 中断后整轴回零"兜，彻底做法 = 读位置 0x36 + 绝对模式 */
#define DBG_CFG_POS_MODE_RELATIVE CFG_ZDT_POS_MODE

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

/* ★ 产品工作包络与"够得着就去"钳位（2026-10-08；**2026-10-10 起值直接取自产品文件**——
   Config/sort_config.h 已是"薄壳直含产品"的转发文件，台架不再有第二份数值）：
   `read` 用它**预演产品会怎么判**；`p`/`pick` 用**同一套钳位**去走——台架必须与产品同行为，
   否则现场"台架拒绝、产品会夹"对不上（2026-10-08 用户实测：`read` 能到投放位、
   `pick` 报目标越界 = 到不了夹持点；产品那边其实是钳位照夹的）。
   产品判据（= Sort_DoCycle 的决策树，逐条同序）：
     ① X ∈ [-160, -135)        ⇒ 钳到 -135 就近夹（夹爪延伸带 = DBG_CFG_PROD_X_CLAMP_MM）
     ② X < -160 / Y < -187     ⇒ 整件作废：机器一动不动（"宁可少抓，不放错盒"）
     ③ X/Y > 上限(0) 且 ≤ 上限+25mm ⇒ 钳到上限**照样伸爪**（极限位夹取带）
        超过 25mm               ⇒ 整件作废
     ④ 钳完再按保护边距回拉：> 上限-3mm 就压到 上限-3mm（永不朝回零开关顶满；
        2026-10-10 边距 5→3，见产品 `CFG_WORK_EDGE_MARGIN_MM` 注释）
   贴边件抓不到时：先查物块是不是真放在够不着的边上/托盘外，再核视觉→机械映射
   （原点 CFG_VISION_ORIGIN_*／符号 SIGN_*／视觉侧比例）——**不是新鲜度问题** */
/* ★ 2026-10-10 起**不再镜像数值**，直接引用产品宏（同一份真源，改产品台架自动跟）：
   这四个名字保留是因为 `read`/`p`/`pick` 的代码都在用它们——值就是产品的那四个 */
#define DBG_CFG_PROD_WORK_X_MIN  CFG_WORK_X_MIN_MM
#define DBG_CFG_PROD_WORK_X_MAX  CFG_WORK_X_MAX_MM
#define DBG_CFG_PROD_WORK_Y_MIN  CFG_WORK_Y_MIN_MM
#define DBG_CFG_PROD_WORK_Y_MAX  CFG_WORK_Y_MAX_MM
#define DBG_CFG_PROD_WORK_OVERSHOOT CFG_WORK_OVERSHOOT_MM /* 微越界允许量（已被 25mm 带覆盖） */
#define DBG_CFG_PROD_X_CLAMP_MM      CFG_XY_X_CLAMP_MM     /* -X 夹爪延伸带 */
#define DBG_CFG_PROD_CLAMP_BAND_MM   CFG_XY_X_CLAMP_BAND_MM /* +侧极限位夹取带值 */
#define DBG_CFG_PROD_WORK_EDGE_MARGIN CFG_WORK_EDGE_MARGIN_MM /* 保护边距 */

/* ★ 撞限位判据（2026-10-08，与产品 CFG_LIMIT_CONFIRM_MS **同值同口径**）：X/Y 电平
   **连续压住**这么久才算真撞（急停 + XY 记账作废）；窗内已松开 = 电气毛刺 ⇒ 不停、不作废。
   现场两个现象把口径逼到这一条：①"抓完之后有时自己回零校准"（旧"沿闩锁即作废"把舵机/
   电机电流尖峰耦合出的假沿当真，每来一次就白跑一次回零）②"一上来连夹持点都到不了"
   （拆分口径：停止帧立即发、记账晚 40ms 才作废 ⇒ 毛刺后记账被保留、可运动已被截在半路
   ⇒ 记账说"已到位"、实物停在半路 ⇒ 坐标静默平移）。实现见 dbg_app.c 的 Dbg_App_LimitWatch */
#define DBG_CFG_LIMIT_CONFIRM_MS CFG_LIMIT_CONFIRM_MS

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
   完全升起 1200→1600→1650、完全放下 900→1300、兜底初夹 1600→1750、兜底夹紧 1750→1950，
   并按形状回填三套特调；2026-10-02 完全升起 1650→1600——1650 顶死堵转，回退 50µs）**。
   数值跟机构走，与早期占位值（500/2500 等）完全不同是正常的 */
#define DBG_CFG_CLAW_OPEN_US  CFG_CLAW_OPEN_US /* 完全张开（夹爪重装后三次标定：1000→1250）：巡航/越托盘
                                       必须用它，否则撞托盘边或旁边物块。★三种特调共用的值之一 */
#define DBG_CFG_CLAW_CLOSE_US CFG_CLAW_CLOSE_US /* 兜底夹紧（形状 0 = 默认方案；抓取走下方方案表） */
#define DBG_CFG_CLAW_HALF_US  CFG_CLAW_HALF_US /* 兜底初夹（形状 0 = 默认方案，作废轻放同值） */
/* ★ 形状 -> 夹持方案（2026-10-01 视觉组口径）：形状**不参与分类**（进哪个盒只由类别定），
   只用来选这套夹持参数——不同形状的物块需要的夹紧角度不同。
   **形状 0 = 默认方案**（不查此表，直接用 DBG_CFG_CLAW_HALF_US / CLOSE_US 兜底值）；
   **1 = 立方体、2 = 12面体、3 = 六棱柱**；数组下标 = 形状 - 1。
   ★2026-10-01 用户实测回填（形状 1/2/3 各一套，不再是折中值）：
     1 立方体：初夹 1900 / 夹紧 2000   2 十二面体：初夹 1900 / 夹紧 **2050**（10-02 +50）
     3 六棱柱：初夹 1800 / 夹紧 **2000**（10-10 夹持力不够 +50：1950→2000）
   ★值就是产品 `CFG_GRIP_*`（下面两行直引，台架没有第二份——改产品即改台架） */
#define DBG_CFG_GRIP_HALF_US  CFG_GRIP_HALF_US /* 立方体/12面体/六棱柱 */
#define DBG_CFG_GRIP_CLOSE_US CFG_GRIP_CLOSE_US /* 立方体/12面体/六棱柱 */
#define DBG_CFG_LIFT_UP_US   CFG_LIFT_UP_US /* 完全升起（1650 顶死堵转，2026-10-02 回退 50µs）
                                       ★升降只有两档、与形状无关 = 三种特调共用的值之一 */

#define DBG_CFG_LIFT_DOWN_US  CFG_LIFT_DOWN_US /* 完全放下（放到底）；夹紧只能在放到底之后做 */

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
#define DBG_CFG_SERVO_MIN_US CFG_SERVO_MIN_US
#define DBG_CFG_SERVO_MAX_US CFG_SERVO_MAX_US
/* 切档后等舵机走到位的默认时间 */
#define DBG_CFG_FW_STAMP "1010a" /* ★固件版本戳：横幅/stat 可见——"板上烧的哪个版本"一眼定，杜绝新旧混淆
                                    （1008a/b = `read` 超时判词分诊 + `k230 raw` 线上真相；
                                     1008c = 撞限位判据换成"电平连续压住"（毛刺不停、不作废）
                                       + `p`/`pick` 与产品同口径钳位（延伸带/极限位带/保护边距）；
                                     1009a = 投放点激光检测首版（`laser` 即时读数/`verify`/`watch`）；
                                     1009b = 现场回填 PD14/检测=低 + `laser drop`（宽度量测）；
                                     1009c = 上机实测"一次落料 ≈1 个采样点宽（µs 级）、无毛刺"⇒
                                       判据定案"窗口内捕到一次检测即可"：去掉确认窗、`laser drop`
                                       宽度改**周期数+µs（自校准计时）**、打印去 %f（格式器不支持）、
                                       `laser verify` 改 µs 级密采样；
                                     1010a = 帧协议新增**数量位**（下标 6，2026-10-10 现场口径：
                                       激光被夹爪遮住 ⇒ 计数判据换成视觉"两帧数量差 1"）：
                                       快照/live/raw 全带数量、新增 `k230 count [秒]` 数量位观察、
                                       `k230 sim` 加可选数量参数） */
#define DBG_CFG_SERVO_SETTLE_MS CFG_SERVO_MOVE_MS

/* ================= 3.5 限位 / 原点微动开关 ================= */
/* ★ 与产品工程完全同脚（code/Core/Src/gpio.c：EXTI 下降沿 + 上拉，未触发=高、压下=低）：
      转盘原点 = PB0（EXTI0）  X 限位 = PB1（EXTI1）  Y 限位 = PC5（EXTI9_5）
   接法：微动开关 COM -> GND、NO -> 信号脚（NC 不接）。
   ★ 现在跑的就是产品板：PCB 上的 4.7k 外部上拉已在位（天线效应的硬件根治），
     内部上拉保持开启，二者并联无害。
   ⚠️ 引脚号（0/1/5）对应 stm32f4xx_it.c 里已开的三条 EXTI 中断线；
      换线只换端口宏与 NAME，引脚号别动（动了要同步改 NVIC 那三行） */
#define DBG_CFG_LIMIT_TRAY_PORT CFG_ORIGIN_GPIO_TRAY
#define DBG_CFG_LIMIT_TRAY_PIN  CFG_ORIGIN_PIN_TRAY
#define DBG_CFG_LIMIT_TRAY_NAME "PB0"
#define DBG_CFG_LIMIT_X_PORT    CFG_LIMIT_GPIO_X
#define DBG_CFG_LIMIT_X_PIN     CFG_LIMIT_PIN_X
#define DBG_CFG_LIMIT_X_NAME    "PB1"
#define DBG_CFG_LIMIT_Y_PORT    CFG_LIMIT_GPIO_Y
#define DBG_CFG_LIMIT_Y_PIN     CFG_LIMIT_PIN_Y
#define DBG_CFG_LIMIT_Y_NAME    "PC5"

/* 沿计数消抖门限（ms）：门限内的重复沿 = 同一次触发的弹跳，不计数。
   2026-09-26 台架实测依据：单次按压蹦 17~128 沿，且开关断开态下线路还会拾到
   自发 burst（直连 GND 后 60s 零事件 → 源头是弱上拉悬空段的天线效应+开关弹跳，
   硬件根治 = 产品设计的 4.7k 外部上拉，台架并上即同款）。
   ⚠️ 40ms 只用于台架"沿计数=按压次数"；产品回零的消抖/首沿闩锁是另一笔账
   （20ms@100mm/s≈2mm 过冲，见 `调试与推进计划.md` 附录 §8 第 1 条），PCB 修复后单独定 */
#define DBG_CFG_LIMIT_DEBOUNCE_MS 40u

/* ================= 3.6 投放点激光检测（2026-10-09 新增硬件） =================
   ★ 2026-10-10 起本模块**退出计数闸门**：现场实况 = **激光模块会被夹爪遮住**（爪下伸到投放点
     就把光路挡死）⇒ 分不清"货落下"与"爪在光路里"⇒ 识别失败。产品的计数判据因此换成
     **视觉帧数量位的两帧差 1**（帧下标 6，见 k230_uart.h 与产品 sort_config.h 的"数量复核"一节）。
     本节命令与硬件接线**全部保留**（模块没坏，只是判据换人）：`laser drop`/`laser verify`/
     `laser`/`laser watch` 仍是这个模块的验收工具——哪天把它挪到爪够不着的位置，先在这一侧
     验通，再把产品 CFG_LASER_ENABLE 改回 1。
     数量位的验收工具见下面的 3.6b（`k230 count`）。 */
/* ★ 现场口径（用户实测，本节的依据）：模块接 **PD14**，**检测到物块 = 低电平**；
   **物块落下的时间很短，且没有毛刺干扰** ⇒ 用户口径定案：**"只要检测脉冲就可以了"**。
   ★ 上机实测（1009b 的 `laser drop`，三次落料）：**一次落料 = 1 段事件、宽度 ≈0~1 µs**
   （等于一个采样点）⇒ **1ms 级的电平轮询按网格根本碰不到它**（会漏 99.9%）⇒
     产品侧取证据 = **EXTI 下降沿计数**（PD14 = EXTI14；gpio.c 配 IT_FALLING + EXTI15_10，
     bsp_gpio.c 只计数；App 在投放窗口里"快照比大小"）；
     台架侧 = **µs 级密采样**（`laser drop` / `laser verify`）——两侧判定语义一致：
     **窗口内出现过一次检测就算**，不问宽度、不设确认窗。
   产品侧的用途：松爪落料时在窗口内捕到这一次检测才允许计数——夹空（抓空/半途掉件）时
   屏上一件不加（见产品 Config/sort_config.h 该节、sort_task.c 的 Sort_VerifyDrop）。
   台架这一侧管两件事（命令见表）：
     ① **有效性验证**：`laser drop` 丢一块货过光路，看**段数**（一次落料应 = 1 段）
        与宽度（≈0µs 就印证"窄事件"结论、也印证产品必须走沿）
     ② **产品同口径预演**：`laser verify` 走完整序列（静置 → 开爪落料 → µs 密采样窗口 → 判定）
   ★ 接线：模块输出（DO / NPN 集电极开路）→ PD14，模块 GND → 板 GND，信号脚上拉 ⇒
     模块导通（检测到）= 读到低。模块自带指示灯就先看灯：灯亮而读数不动 = 输出类型/供电/
     共地问题，不是固件问题。
   ⚠️ 极性 DBG_CFG_LASER_ACTIVE_LEVEL = "检测到"时引脚读作的电平，**必须与产品
      CFG_LASER_ACTIVE_LEVEL 同值**（两处都是标定项，不是协议）。判定反了：先看 `laser`
      打出的**原始电平**（1=高 0=低）再决定翻哪个宏——"线接反"与"极性宏写反"分得清清楚楚。
      ⚠️ 产品的沿方向与极性绑定（检测=低 ⇒ 下降沿）；若哪天模块改高有效，两处要同改。
   ⚠️ 本工程原先把 ENABLE 缺省设成 0（= 一个脚都不碰，防占错脚毁控制台）；现在引脚已定死
      PD14（与控制台 CN1 的 PA0/PA1、与 USART2 的 PD5/PD6 都不冲突），所以**已置 1** */
#define DBG_CFG_LASER_ENABLE 1 /* 2026-10-09 用户实测接线后打开（引脚/极性已回填） */
#define DBG_CFG_LASER_NAME   "PD14" /* 引脚/极性**直引产品**（用户实测接线 PD14）—— */
#define DBG_CFG_LASER_PORT   CFG_LASER_GPIO          /* 换脚只改产品 CFG_LASER_*，两侧自动同值 */
#define DBG_CFG_LASER_PIN    CFG_LASER_PIN
#define DBG_CFG_LASER_ACTIVE_LEVEL CFG_LASER_ACTIVE_LEVEL /* 检测到物块 = 低电平 */
/* 产品同款判据参数（`laser verify` 用；改这里要与产品 sort_config.h 的 CFG_LASER_* 同步。
   ⚠️ 没有"确认窗"这个参数了：台架实测一次落料 ≈ **1 个采样点宽（µs 级）**，
   任何"连续 N ms 才算"的门都会把真事件判掉（用户口径："只要检测脉冲就可以了"）——
   产品侧取证据靠 **EXTI 下降沿计数**（PD14 = EXTI14，gpio.c 配 IT_FALLING/EXTI15_10），
   台架 `laser verify` 用 µs 级密采样，两者都只问"窗口内有没有出现过一次检测" */
#define DBG_CFG_LASER_SETTLE_MS  CFG_DROP_SETTLE_MS /* 到点静置（等龙门停稳再松爪）= 产品同值 */
#define DBG_CFG_LASER_POLL_MS    CFG_LASER_POLL_MS   /* 慢速采样周期（`laser watch` 用；判据不吃它） */
#define DBG_CFG_LASER_WAIT_MS    CFG_LASER_WAIT_MS   /* 松爪后等证据的窗口上限 */
/* `laser drop` 捕获窗口（秒）：µs 级快采样，把落料脉冲连个数带宽度量出来 */
#define DBG_CFG_LASER_CAP_S_DEFAULT 5u
#define DBG_CFG_LASER_CAP_S_MAX     60u

/* ================= 3.6b 数量位（2026-10-10 协议新增；产品的计数判据靠它） =================
   ★ 帧格式：`AA 55 | 状态 | 类别 形状 hasQR **数量** X_L X_H Y_L Y_H | 校验`（12 字节）。
     数量位 = **下标 6**（原"保留"字节，与 hasQR 那个保留字节并列的那个），范围 0~255，
     含义 = **本帧里托盘上物块的总数**（含当前目标那一个；只数托盘区内，不含投放区/料盒/爪里）。
     **每帧都要填；状态 00 的无目标帧也要照周期发、也要填**——托盘空了的那个 0 就是
     "末件 +1"与"空盘静默窗"的唯一凭据（产品 sort_task.c 的 Sort_ReadEvent 专等这种帧）。
     校验口径不变：前 11 字节累加和低 8 位 ⇒ 数量位**参与校验**（填错会让整帧被丢）。
   ★ 台架验收（本固件）：
       `k230`        快照里直接看"最近一帧数量位 = N（多少 ms 前，有目标/无目标）"
       `k230 count [秒]`  数量位观察：拿一件货出去 / 放一件进来，数量位应跟着变（**含无目标帧**）
       `k230 raw [秒]`    逐帧 hex + 解码（数量位就在解出来的那一行里）
       `k230 sim 类 x y [形状] [数量]`  本地注帧带数量，不接线也能验"解析->缓存->快照"
   ★ 现场判据：**盒里有货、产品屏上全 0** = 数量位没上线（上位机没填下标 6）——
     先在台架 `k230 count` 拿放一件核它变不变，别去查产品分拣流程（仿真 s25 冻的就是这条）。 */

/* ================= 4. 运动安全与速度 ================= */
/* 单段最大位移：命令行 typo（x 5000）时拒绝执行，别让龙门一路撞过去 */
#define DBG_CFG_MAX_MOVE_MM 200.0f

/* 速度上限（mm/s）：比产品档高一点（220 > 200），留手动超速做对比实验的余地 */
#define DBG_CFG_MAX_SPEED_MM_S 220.0f
/* 默认速度 = **产品档**（2026-10-10 对齐：台架默认就跑产品速度，免得"台架慢一半"
   让现象掩盖真问题；想慢下来手动给 `x 30 -50 80` 这类第三参） */
#define DBG_CFG_DEFAULT_SPEED_MM_S CFG_XY_SPEED_MM_S

/* 到位判定余量（ms）：与产品 CFG_MOVE_EXTRA_MS 同口径，由 Kinematics_* 内部用 */
#define DBG_CFG_MOVE_EXTRA_MS CFG_MOVE_EXTRA_MS

/* ================= 4b. 回零（homex/homey，X=PB1 Y=PC5） ================= */
/* ★ 2026-09-26 机械确认：两只开关装在**最大行程端**（X max / Y max），不在原点端！
   → 回零 = 向 +X/+Y 撞开关；触发点 = 最大行程端，原点从触发点反推。
   ★ 2026-09-30 用户定调：**简化成两相**（粗压撞开关即停 → 退一小段），
   删掉了"反向脱开确认 + 5mm/s 慢压定原点"两相——实现见 dbg_app.c 的 Dbg_App_HomeRun */
#define DBG_CFG_HOME_DIR_X        (+1)  /* +1 = 向 X+ 找开关（装在最大端）；若拆到原点端改 -1 */
#define DBG_CFG_HOME_DIR_Y        (+1)  /* +1 = 向 Y+ 找开关（装在最大端）；若拆到原点端改 -1 */
/* 粗压速度 = **产品档**（2026-10-10 对齐：产品 CFG_HOME_SPEED_MM_S = 40mm/s；
   触发沿→停实测 ~1.5ms ⇒ 过冲 ≈0.06mm，海绵余量 2.7cm 的零头，
   限制它的其实是撞击感不是精度）。想慢一点手动给：`homex 10` */
#define DBG_CFG_HOME_SPEED_DEFAULT 40u
#define DBG_CFG_HOME_SPEED_MAX     60u /* 手动给 homex/homey 的速度上限（默认 40 之下留余量） */
/* ⚠️ 这不是产品的"慢压定零"（台架两相制里没有那一相）——只用于**起点已压着开关时
   先慢慢退开**这一下（避免猛退把开关/机构拽一下）。产品是四相（含慢压定零），
   台架两相是 2026-09-30 用户定调的精简版，两者的"退距"是同一个值（下面 4c 节） */
#define DBG_CFG_HOME_SLOW_MM_S     5u

/* ================= 4c. 工作区软限位（`p`/`x`/`y`/`sq`/`pick` 等意图运动用） ================= */
/* ★ 坐标约定：**原点 = 归零后的驻留位**（触发点内退 DBG_CFG_HOME_BACKOFF_MM），
   工作区全在**第三象限**（X≤0 且 Y≤0），正方向是开关（禁止）；上界恒 0。
   ★ X 下限放宽（2026-10-01 用户实测：龙门可运动到 -135，旧值 -122 过严）：
   ★ Y 下限 = 投放点（再向下不需要）。
   ⚠️ 改 DBG_CFG_HOME_BACKOFF_MM 会让原点整体平移，下面两个数要**同位跟着改**！
      换机/重装机械后重新标定法：归零 → 手动走到投放点/机械端 → 读 `p` 的坐标写回这里 */
#define DBG_CFG_XY_MAX_MM   CFG_WORK_X_MAX_MM /* = 0（上界恒 0，正方向是开关） */
#define DBG_CFG_XY_X_MIN_MM CFG_WORK_X_MIN_MM
#define DBG_CFG_XY_Y_MIN_MM CFG_WORK_Y_MIN_MM

/* ★ 夹爪延伸夹取带（2026-10-01 用户提出）：物块可出现在 X ∈ [-160, -135)——龙门停在 -135
   时，夹爪构造仍够得着并夹住。`pick`（含 read 给出的坐标）的目标 X 超出软限位时
   **钳位到 -135 就近夹取**（打印提示，不拒绝）；比 -160 还远的照常拒绝（延伸也够不着）。
   手动 `p` 语义不变（X 下限同样是 -135）。产品移植时同款逻辑进 Sort_DoCycle */
#define DBG_CFG_XY_X_CLAMP_MM CFG_XY_X_CLAMP_MM

/* ★ 投放点坐标（2026-09-30 用户台架实测，新原点系）：X = -70、Y = -187（= Y 下限）。
   `drop [mm/s]` 一键转过去；抓取点/其余工位坐标待机械组回填后照此追加 */
#define DBG_CFG_DROP_X_MM CFG_DROP_X_MM /* 投放点 = 产品读帧位，同一份值 */
#define DBG_CFG_DROP_Y_MM CFG_DROP_Y_MM

/* 找零行程上限：≥ 最长实际行程（Y 约 190）+ 余量。走满=没等到触发沿（开关没压到）。
   ★ 刻意取小：真挂住时"顶着推"的距离就是它（P0-6 教训——宁小勿大） */
#define DBG_CFG_HOME_TRAVEL_MM   CFG_HOME_TRAVEL_MM /* = 产品同一值（220） */

/* 触发后的退距（mm）：越小工作区越大，但必须保证开关可靠脱开。
   脱不开时固件会自动再退一份（共 2×）；仍不行就调大这个值 */
#define DBG_CFG_HOME_BACKOFF_MM   CFG_BACKOFF_MM /* = 产品同一值（触发后回退 3mm，脱不开自动再退一份） */

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