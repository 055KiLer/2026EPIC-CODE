/**
 * @file    dbg_app.h
 * @brief   调试动作层：把"意图"变成两台电机的帧（X/Y 用产品的解算器）
 */
#ifndef __DBG_APP_H
#define __DBG_APP_H

#include "main.h"

/* 上电：打印配置与用法，两舵机进安全位（抬升 + 张开） */
void Dbg_App_Init(void);

/**
 * @brief  ★上电自动体检：6 档波特率 × 地址 1~8 各发一次使能帧，只打"有回包"的组合
 * @note   给"控制台收不到命令"那种情况兜底：板子自己把结论打在控制台上，
 *         只靠"板子→PC"这一个方向就能定位地址与波特率（由 DBG_CFG_BOOT_AUTOSCAN 开关）
 */
void Dbg_App_BusAutoScan(void);

/* 主循环每圈调用：没收到过控制台输入时定期重跑体检（见 dbg_app.c 的说明） */
void Dbg_App_Tick(void);

/* 打印全部关键配置（含待核实项），`cfg` 命令也调它 */
void Dbg_App_PrintConfig(void);

/* 状态汇总：记账、两位舵机当前脉宽、脉冲/mm、上一次运动的估算/实测 */
void Dbg_App_PrintStatus(void);

/* 使能/失能：
   Dbg_App_EnableBoth  = 按配置的 A/B 两个地址分别发（需要 ID 正确）
   Dbg_App_EnableAddr  = 只发给的这一个地址；**addr = 0 就是广播**（不需要知道 ID） */
uint8_t Dbg_App_EnableBoth(uint8_t enable);
uint8_t Dbg_App_EnableOne(uint8_t motor, uint8_t enable); /* motor：0=A 1=B，只发那一条线 */
uint8_t Dbg_App_EnableAddr(uint8_t addr, uint8_t enable);

/* 立即停止：广播停 + 按配置地址各停一次（不知道 ID 时也一定停得下来） */
uint8_t Dbg_App_StopBoth(void);
uint32_t Dbg_App_LastStopDoneTick(void); /* 最近一次 StopBoth 的广播停应答收完时刻（延迟测量） */
uint8_t Dbg_App_StopTest(void);      /* stoptest：运动中发停止三连并打印逐帧应答判决 */

/* 单台原始点动（脉冲为正 = 按产品口径的 CW）。
   motor：0=A 1=B 2=转盘（此时用该台登记地址）；-1 = 广播/数字地址，A/B 两线各发一遍 */
uint8_t Dbg_App_Jog(int8_t motor, uint8_t addr, uint8_t ccw, uint16_t rpm, uint8_t accel,
                    uint32_t pulse);

/**
 * @brief  广播走一段（不需要知道 ID）：两台同向同量 = 纯 X 平移
 * @param  mm  位移，正 = 按产品的 +X（右）；负 = 反向
 * @note   广播的固有限制：只能让两台做同样的动作，所以做不到纯 Y / 方块 / 耦合验证
 */
uint8_t Dbg_App_BroadcastMove(int32_t mm, uint16_t rpm);

/**
 * @brief  按 X/Y 意图走一段相对位移（走产品解算器 Kinematics_XYSolve）
 * @param  dx_mm/dy_mm 相对位移，正 = 右 / 下（原点在左上角）
 * @note   返回 0 表示被拒（超单段上限 / 发送失败）；此时已对双轴发停止
 */
uint8_t Dbg_App_MoveXY(int32_t dx_mm, int32_t dy_mm, uint16_t speed_mm_s);

/* 方块测试：X+ → Y+ → X- → Y- ，回到起点附近（验耦合与方向） */
uint8_t Dbg_App_Square(int32_t mm, uint16_t speed_mm_s);

/**
 * @brief  ★回零（homex/homey，**两相制**——2026-09-30 简化）：开关装在最大行程端
 *         （X max / Y max），向 +X/+Y 走去撞开关，对应限位（X=PB1 / Y=PC5）裸下降沿
 *         触发即停 → 退 DBG_CFG_HOME_BACKOFF_MM（默认 3mm）→ 电平复核必须回到"开"。
 *         假沿作废、脱不开会自动加倍退一次、两次尝试；途中敲任意键 = 软急停
 * @note   原点 = 触发点内退 BACKOFF 的驻留位；工作区在第三象限（见 dbg_config.h §4c）
 */
uint8_t Dbg_App_HomeX(uint16_t speed_mm_s);
uint8_t Dbg_App_HomeY(uint16_t speed_mm_s);

/* ★ 一键全回零（`home`）：homex → homey → thome 顺序串跑 = 产品上电 Sort_EnsureCalibrated
   自动做的那一步（产品 Motion_HomeXY 同样是 X 先 Y 后的串行）。任一步失败即停（后面不跑），
   每步都可用单轴命令单独重试 */
uint8_t Dbg_App_HomeAll(void);

/* ★ 上电自动归零（DBG_CFG_AUTO_HOME_ON_BOOT）：横幅/帮助打完后等
   DBG_CFG_AUTO_HOME_SKIP_MS，期间控制台任意字节 = 本次跳过（字节留在缓冲照常执行）；
   窗口内安静 = 自动跑 Dbg_App_HomeAll()。main.c 在打完帮助后调它 */
void Dbg_App_AutoHomeIfEnabled(void);

/* 打印当前记账位置（`p` 不带参数时） */
void Dbg_App_PrintPosition(void);

/**
 * @brief  ★绝对定位：走到工作区坐标 (x, y)——第三象限约定（X≤0、Y≤0，
 *         原点 = 归零驻留位），需先 homex + homey 且中途未停/未撞
 * @note   ★2026-10-08 起与产品 Sort_DoCycle **同口径钳位**（"够得着就去夹"，不再一律拒绝）：
 *         -X 延伸带 [-160,-135) 钳到 -135；+侧 (0, +25mm] 钳到上限；钳完按保护边距 5mm
 *         回拉（永朝回零开关顶满）。带外才拒绝（产品同样整件作废、机器一动不动）。
 *         位置为开环推算；真撞限位（电平连续压住 ≥DBG_CFG_LIMIT_CONFIRM_MS）或手动
 *         stop（半途停）都会作废记账——电气毛刺（压住不到该窗就松开）不停、不作废
 */
uint8_t Dbg_App_Goto(float x, float y, uint16_t speed_mm_s);

/**
 * @brief  标定用单轴位移：只用 X 方向、直接按给定 pulses/mm 算脉冲（绕过解算器）
 * @param  mm  要走的距离；ppm 每毫米脉冲数
 * @note   专用作 CFG_XY_PULSE_PER_MM 的卷尺标定：走 100mm，量实际值，按比例改 ppm
 */
uint8_t Dbg_App_CalMove(int32_t mm, uint16_t ppm);

/* 当前生效的 pulses/mm（取自 Config/sort_config.h 的 CFG_XY_PULSE_PER_MM） */
uint16_t Dbg_App_PulsePerMm(void);

/* 运行时改 A/B 地址：which 0=A 1=B；改完自动探测该地址（台架换电机不用重烧） */
uint8_t Dbg_App_SetMotorAddr(uint8_t which, uint8_t addr);

/* 逐 ID 试使能，用来核出**指定线**上驱动器的实际地址（motor：0=A 1=B 2=转盘） */
void Dbg_App_ScanId(uint8_t motor, uint8_t id_max);

/* 测一帧位置指令在某条线上的发送耗时（µs）（motor：0=A 1=B 2=转盘） */
void Dbg_App_MeasureFrame(uint8_t motor, uint8_t addr, uint16_t repeat);

/**
 * @brief  ★转盘回零演练（thome，四相，逐相对齐产品 Motion_TrayHome）：
 *         凸块退出 → 速度模式粗寻标@SEEK_RPM 等原点压下 → 反向脱开 BACKOFF_PULSES →
 *         慢压@APPROACH_RPM 定索引零位。全程走通 = 原点开关 PB0 校准正确。
 *         途中敲任意键 = 软急停；两次尝试（产品同款）
 * @param  seek_rpm 粗寻标速度（0 = 用产品 CFG_TRAY_SEEK_RPM）
 * @note   相位1 超时的三种可能：凸块没装 / 开关线不对（`lim` 核）/ 速度口径慢 10 倍（`spu`）
 */
uint8_t Dbg_App_TrayHome(uint16_t seek_rpm);

/**
 * @brief  ★转到指定盒号（tb，对齐产品 Motion_TrayGotoBin/TrayFinish）：
 *         单向正转趋近消背隙，steps × CFG_TRAY_PULSES_PER_BIN @ CFG_TRAY_SPEED_RPM；
 *         走完做产品同款双复核（索引沿计数 + 脉冲折算盒号），任一不过 = 记账作废
 * @param  bin 0~5（需先 thome）
 */
uint8_t Dbg_App_TrayGotoBin(uint8_t bin);

/* 打印转盘记账与标定常量（`tb` 不带参数时）；盒号显示 1~6（盒1=回零位那格） */
void Dbg_App_TrayPrint(void);

/**
 * @brief  ★tb <类别1~6> 一句到位：类别号查映射表（dbg_config.h §4d，缺省恒等）转到该
 *         类别的物理盒——与产品 Sort_DoCycle 的 s_category_to_bin 同一张表的台架副本
 * @param  category 视觉类别 ID 1~6（= `read` 打出的类别号）
 */
uint8_t Dbg_App_TrayGotoCategory(uint8_t category);

/**
 * @brief  ★盲测工具①：把"使能帧 + 位置帧"的**字节**打出来（不发）
 * @note   电机不动时的第一件事：对着驱动器手册核帧，或把这几行发给别人看
 */
void Dbg_App_DumpFrames(uint8_t addr, uint32_t pulse);

/**
 * @brief  ★盲测工具②：TX/RX 回环自检
 * @note   把板子的总线 TX 与 RX 用一根线短接（并把驱动器拔掉），
 *         这条命令发几帧后把**收到的东西**按十六进制打出来。
 *         收到自己发的字节 = MCU 的发送链路（时钟/引脚/波特率/MSP）全对，
 *         问题在驱动器那一侧的接线或配置；什么都收不到 = 反过来。
 */
void Dbg_App_Loopback(uint16_t repeat);

/**
 * @brief  ★盲测工具③：运行时改总线波特率（驱动器被别人改过波特率时用）
 * @retval 1=已切换
 */
uint8_t Dbg_App_SetBusBaud(uint32_t bps);

/**
 * @brief  ★盲测工具④：探测某个地址的驱动器有没有应答
 * @note   只发**使能帧**（不含任何运动指令，电机不会走），然后打回包并解读。
 *         驱动器正确接收会回 [地址][功能码][02][6B]（来源：物创 `code/ZDT_drive.c`
 *         里的 receive_* 应答表 + 2ms 的接收调用）。
 *         回包是"链路通"最硬的证据，比看电机转不转灵敏得多：
 *           回 02 → 帧已经到驱动器里了，问题不在 MCU 这一侧
 *           无回包 → 供电/接线/波特率/地址/控制模式，按打印出来的顺序查
 */
void Dbg_App_AskAck(uint8_t motor, uint8_t addr); /* motor：0=A 1=B 2=转盘 */

/**
 * @brief  ★盲测工具⑤：把任意一段字节直接发到总线（并打回包）
 * @note   用途：逐字节复现手册或物创固件里的帧，排除"我这一侧构帧"的所有变量
 */
uint8_t Dbg_App_SendRaw(uint8_t motor, const uint8_t *bytes, uint16_t len); /* 0=A 1=B 2=转盘 */

/**
 * @brief  速度字段单位系数：真实转速 ÷ 系数 = 帧内数值
 * @note   1 = 字段即 RPM（产品口径，本工程默认）；10 = 字段是 0.1RPM（物创口径）。
 *         只作用于"意图路径"（bx/x/y/xy/sq/cal）；`j` 永远发你给的原始数值
 */
void     Dbg_App_SetSpeedScale(uint16_t scale);
uint16_t Dbg_App_SpeedScale(void);

/* ★ 转盘速度系数（`tspu`）：与 XY 的 spu **独立**（57 与 42 可能不是同一口径）。
   1 = 字段即 RPM，10 = 0.1RPM；同时缩放转盘的所有等待超时 */
void     Dbg_App_SetTraySpeedScale(uint16_t scale);
uint16_t Dbg_App_TraySpeedScale(void);

/* ★ 转盘速度百分比（`tspd`）：给三种标定转速统一乘倍数（100 = 产品标定值）。
   只调节拍；等待超时会按 100/pct 反向缩放。定稿值要写回产品 CFG_TRAY_*_RPM */
void     Dbg_App_SetTraySpeedPct(uint16_t pct);
uint16_t Dbg_App_TraySpeedPct(void);

/* ============ ★ 示波器观测变量（Keil Watch / Logic Analyzer 里画曲线用） ============ */
/**
 * @note  一组**全局独立标量**（非 static + volatile → 必然在 ELF 符号表里）。
 *        ⚠️ 不打包成结构体：Watch/Logic Analyzer 只认"单个变量名"，结构体成员路径
 *        （g_scope.lim_y）在 LA 里直接 cannot evaluate（2026-09-26 实测）。
 *        主循环每圈在 Dbg_App_Tick 里刷新。用法：
 *          Watch：调试会话里输入 g_lim_y / g_lim_y_cnt / g_claw_us …
 *          Logic Analyzer：Setup → 新建信号 → 填同样的名字
 *          ⚠️ 程序跑着时必须勾 View → Periodic Window Update 窗口才刷新
 *          ⚠️ SEGGER J-Scope 只认 J-Link 硬件，本台架的 Horco 探针用不了
 *        各变量地址看 map 文件（debug_corexy.map，符号名逐一可查） */
extern volatile uint32_t g_scope_tick; /* 主循环圈数，一直爬 = 固件活着 */
extern volatile uint16_t g_claw_us;    /* 夹爪当前脉宽 */
extern volatile uint16_t g_lift_us;    /* 升降当前脉宽 */
extern volatile uint8_t  g_lim_x;      /* X 限位电平（原始，1=开 0=压下） */
extern volatile uint8_t  g_lim_y;      /* Y 限位电平 */
extern volatile uint8_t  g_lim_tray;   /* 转盘原点电平 */
extern volatile uint32_t g_lim_x_cnt;  /* 消抖后沿计数（=按压次数） */
extern volatile uint32_t g_lim_y_cnt;
extern volatile uint32_t g_lim_tray_cnt;
extern volatile uint32_t g_move_est_ms; /* 上次运动估算时长 */
extern volatile uint32_t g_move_act_ms; /* 上次运动实测时长 */

/**
 * @brief  ★抓取全流程（pick）：张开 → [可选先定位到 (x,y)] → 初步夹住 → 下放 → 夹紧 → 抬回
 * @note   时序由机构构造决定（dbg_config.h §3 注释）：巡航不全开会撞托盘边/物块，
 *         没放到底就夹紧会顶死。带坐标时先 `p` 同一套记账定位（需已归零），
 *         定位失败则爪时序不执行。
 *         shape 0~3（缺省 0）= 夹持方案：0 = 默认（兜底值）、1 = 立方体、2 = 12面体、
 *         3 = 六棱柱（取 DBG_CFG_GRIP_* 表对应值），与产品 Sort_Grab(shape) 同口径；
 *         越界传 0 兜底
 */
uint8_t Dbg_App_Pick(float x, float y, uint16_t speed_mm_s, uint8_t has_xy, uint8_t shape);

/* ★ 读帧位取目标（`read [等待s]`，台架版产品读帧纪律）：归零后走到投放点上方（不挡相机）
 *   → 静默 CFG_VISION_SETTLE_MS → 清缓存 → 等一帧新鲜目标，打印类别/视觉坐标/机械坐标
 *   和可直接照抄的 `pick X Y`。作为抓取依据的坐标必须出自它（归零位读帧=挡相机，不可信）
 *   ★ 2026-10-08：本命令的"先清缓存再等新帧"= 产品 Sort_PrepareRead 同款口径。现场口径 =
 *     上位机从上电起持续发流 ⇒ 机器报"没有可用的目标帧"只可能是三选一（不预设谁的问题）：
 *     ①流里是状态 0x00 的帧（上位机在说"本帧无目标"→识别/目标挑选）
 *     ②流里有目标帧但类别/形状越界（本机按设计丢弃；典型=把"认不出"发成 0/7）
 *     ③帧没到本机（校验/帧长/波特率/接线）
 *     超时判词按四种指纹给，并打印"清缓存前缓存里有没有目标、它多久前到的"（见 dbg_app.c 函数头）
 * @param wait_s 等待上限 1~120s，0 = 默认 10s；任意键中止
 */
uint8_t Dbg_App_ReadTarget(uint16_t wait_s);

#endif /* __DBG_APP_H__ */