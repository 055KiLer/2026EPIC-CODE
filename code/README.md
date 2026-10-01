# 智能分拣装置 · 电控代码

主控 STM32F407VET6 ｜ FreeRTOS + CMSIS-RTOS2 ｜ Keil MDK AC6 ｜ 风格遵循团队 `c-style`（`.clang-format` 在本目录）与 `platform-spec` 四层约定。
规格来源：`../智能分拣电控文档.md`（v1.0）；推进与验收：`../调试与推进计划.md`；仿真基线：`../仿真审计结论.md`。

## 0. 编译与烧录

- **Keil**：双击 `MDK-ARM/smart_sorting.uvprojx` → Build（首次提示器件包时装 Keil.STM32F4xx_DFP ≥3.x）→ DAPLink 下载 `smart_sorting/smart_sorting.hex`。
- **命令行**（团队工具，工程根目录的 `.emb.json` / `.vscode` 由 `emb.py init` 生成）：

```bat
python D:\toolchain\emb.py build "D:\CUBEMX DOCUMENT\工创\code"   :: 期望 0 Error / 0 Warning
python D:\toolchain\emb.py flash "D:\CUBEMX DOCUMENT\工创\code"   :: 烧录（调试器型号见 .vscode/settings.json）
python D:\toolchain\emb.py debug "D:\CUBEMX DOCUMENT\工创\code"   :: OpenOCD + gdb
```

- **格式化**：标准是仓库里的 `.clang-format`，提交前 `clang-format -i Config/*.h User/*/*/*.c User/*/*/*.h`；**`Core/` 不要格式化**（CubeMX 生成物）。
- **clangd 索引**：`compile_commands.json` 由 uvprojx 生成，改过文件清单或包含路径后重跑：
  `python Tools/keil_to_compile_commands.py --uvproj MDK-ARM/smart_sorting.uvprojx --cc "D:/Keil v5/ARM/ARMCLANG/bin/armclang.exe" --as "D:/Keil v5/ARM/ARMCLANG/bin/armasm.exe" --sysroot "D:/Keil v5/ARM/ARMCLANG"`
  （`--as` 必须一起给，否则启动文件那条记录指向不存在的汇编器。）

## 1. 架构

```
Core/                 CubeMX 兼容层 + freertos.c（建任务）+ TIM6 HAL 时基
Middlewares/          FreeRTOS 内核 + CMSIS-RTOS2 + heap_4
User/
├── Algorithm/        motion_kinematics —— 纯运动学（CoreXY 解算/时间估算），无设备、无 RTOS
├── App/              sort_task（主状态机）vision_task（视觉解析＋目标缓存）
│                     display_task（队列刷屏＋完成回执）motion_app（运动执行器＋回零＋记账）
├── Bsp/              bsp_uart（6 路环形缓冲）bsp_gpio（限位查询＋EXTI 事件锁存）
└── Devices/          zdt_motor servo k230_uart（视觉帧解析）tjc_screen
Config/sort_config.h  ★ 全部标定常量（改参数只改这个文件）
sim/                  主机侧仿真台：同一批 User/ 源码 + 假 HAL + 物理模型，17 个场景
Tools/                keil_to_compile_commands.py
```

依赖**单向** `App → Devices → Bsp`（Algorithm 独立）；**只有 App 层使用 RTOS API**；
跨层事件用**事件锁存**：EXTI ISR 只置位/计数，由 `safetyTask` 取走后执行急停。

| 任务 | 优先级 | 职责 |
|---|---|---|
| safetyTask | High | 1ms 守护：唯一处置 EXTI 事件处 → 急停双轴 + 作废记账可信度 |
| visionTask | Normal2 | 10ms 轮询视觉串口 → 解析帧 → 刷新最新目标/无目标计数 |
| displayTask | Normal1 | 阻塞等队列 → 按序号去重计数 → 把 6 行成功数量全量重写（绝对值）→ 公布已完成序号 |
| sortTask | Normal | 回零 → 到读帧位（投放点上方）等静默期 → 锁定目标 → 转盘并行＋抓取 → 投放 → 显示等回执 → **留在读帧位** |

显示红线（显示完成才分拣下一件）由"队列投递 + 序号回执"保证。

**上电时序**：`main()` 时钟与外设初始化 → `Bsp_Uart_Init / Servo_Init / Zdt_Init` → `osKernelStart()`；
`SortTask` 起来后 `Motion_Init` → XY 回零 → 退待机点 → 转盘回零 → 清视觉缓冲 → 主循环（第一轮先把自己挪到读帧位）。
（视觉上位机比 STM32 晚就绪属正常：链路没起来就停在读帧位等，不会误判成"托盘已空"。）

## 2. 硬件映射（真源＝原理图 2026-09-14 版）

| 功能 | 外设 | 引脚 | 连接器 |
|---|---|---|---|
| TJC 串口屏 | USART6 | PC6 TX / PC7 RX | CN2 |
| 视觉上位机（ROCK 5C） | USART2 | PD5 TX / PD6 RX | CN3（本侧只收不发） |
| 预留（原 GM75 扫码） | UART4 | PA0 TX / PA1 RX | CN1 |
| CoreXY A 电机 | USART3 | PC10 TX / PC11 RX | M1 |
| CoreXY B 电机 | UART5 | PC12 TX / PD2 RX | M2 |
| 转盘电机 | USART1 | PB6 TX / PB7 RX | M3 |
| 升降 / 夹爪舵机 | TIM4_CH3 / CH4 | PB8 / PB9 | **H4 / H1** |
| X 限位 / Y 限位 / 转盘原点 | EXTI1 / EXTI9_5 / EXTI0 | PB1 / PC5 / PB0 | H10 / H11 / H2 |

映射散落在 **6 处**（`sort_config.h`、`usart.c`、`hal_msp.c`、`gpio.c`、`.ioc`、`bsp_uart.c` 的 `s_huart[]`），
任何一处单独改错都不报错、只表现为"某个外设完全没反应"，因此 `sort_config.h` 末尾有编译期核对。
`s_huart[]` 是"**语义编号 → 句柄**"的表，不是按外设编号直排，必须与 `sort_config.h` 顶部六行逐行同序。
引脚/极性断言只能用编译期断言，不能写成 `#if`（HAL 的 `GPIO_PIN_n` 带强制转换、`GPIO_PIN_RESET` 是枚举量）。

**微动开关＝常开 + 4.7k 上拉到 3V3**：未触发＝高、压下＝低，故 `CFG_LIMIT_ACTIVE_LEVEL = GPIO_PIN_RESET`、EXTI 全配下降沿。
指示灯 PE10/PE12/PE14、按键 PD12/PD14 当前固件未使用。

## 3. 首次上电标定清单（全在 `Config/sort_config.h`，按序）

1. `CFG_AXIS_A/B_SIGN`、`CFG_HOME_DIR_X/Y`（电机方向）。**归零方向已由台架实锤固定为 `+1`**
   （两只限位开关装在 X/Y **最大行程端**，2026-09-28）；`CFG_HOME_X_MAX_MM/Y_MAX_MM`
   = 触发点在工作坐标系里的位置，⚠️ 待卷尺标定回填（现值 = 行程−20 的占位）
2. `CFG_XY_PULSE_PER_MM`（走 100mm 卷尺量）
3. `CFG_TRAY_DIR_FORWARD`、`CFG_TRAY_ZERO_BIN`（转盘方向与零位盒，需逐盒核对）
4. 舵机五档 PWM（`CFG_LIFT_*_US` / `CFG_CLAW_*_US`）——**夹爪重装后三次标定（10-01，含二提单兜底 + 特调回填）后的当前值**：
   **完全展开（全开）1250**（1000→1100→1250）、**兜底初夹 1750**（1600→1750）、**兜底夹紧 1950**（1750→1950）、
   **升降 完全升起 1600**（1200→1600）/ **完全放下 1300**（900→1300）（µs）。
   ★ 完全展开 1250 与完全升起 1600 是形状 1~3 三种特调（立方体/12面体/六棱柱）**共用**的值；
   上电安全位 = 完全展开（`CFG_CLAW_OPEN_US`）。
   ★ 形状特调表 `CFG_GRIP_HALF_US` / `CFG_GRIP_CLOSE_US` **已于 10-01 按形状逐套实测定稿回填**（不再是折中值）：
   1 立方体 **1900/2000**、2 十二面体 **1900/2000**、3 六棱柱 **1800/1950**（下标 = 形状−1，初夹/夹紧 µs）；
   **形状 0 = 默认方案 = 不查表**，直接用上面的兜底 1750/1950。
   ⚠️ 两处**判据盲区**（取值本身造成，非代码缺陷）：六棱柱夹紧 1950 与兜底夹紧同值 → 仿真查不出"形状 3 走了默认方案"；
   立方体与十二面体两套值完全相同 → 仿真查不出 1↔2 互换；这两条只能靠 `pick X Y 100 <形状>` 人眼验。
   （09-26 首次定稿 1000/1500/1700、升降 900/600 属历史记录；09-28 机构重装后曾标成 1100/1600/1700、升降 1150/900）
5. `CFG_DROP_X/Y_MM`、`CFG_VISION_ORIGIN_X/Y_MM` + `CFG_VISION_SIGN_*`（机械坐标与视觉标定，
   未回填时装置无法完成任何一件）。视觉系与机械系是**两轴交换**关系（视觉 x+ = 机械 Y+、
   视觉 y+ = 机械 X+），视觉原点 = 标定中心（托盘上距左边 87.5mm、距底部 82.5mm）——
   要回填的是"这个中心点在机械系里的绝对坐标"
6. `CFG_MOVE_EXTRA_MS`、`CFG_CLAW_CLOSE_MS`（到位余量 / 夹稳延时）
7. 速度类参数（先慢后快）
8. `sort_task.c` 的 `s_category_to_bin[6]`：抽签后把 6 个类别映射到盒号 0~5，
   六项必须是一个**排列**（双射）；**只需填这一张表**（类别号就是屏上行号）
9. 屏工程那 6 个 "成功数量" 控件名（缺省 `n1~n6`，行序 = 类别 ID 1~6，见 §4）

### 台架实测事实（2026-09-25~28，别再重复踩）

- **帧间隔必须留**：A/B 两帧背靠背（≈1.1ms）驱动器会丢帧 → `CFG_ZDT_FRAME_GAP_MS`；
  停止帧同理 `CFG_ZDT_STOP_GAP_MS`。**归零"跑斜线"的根源就是丢帧 + 撞停堵转保护**。
- **限位必须消抖 + 闩锁**：单次按压 17~128 个沿（弱上拉天线效应+触点弹跳）→
  `CFG_LIMIT_DEBOUNCE_MS`，否则"松开沿"会被判成"又撞了"，反复误急停。
- 驱动器**应答**是 4 字节 `[地址][功能码][02][6B]`；`E2` = 条件不满足（堵转保护中/未使能）
  ——停止帧"发出去没停"多半是它。台架结论全文见 `../台架调试结论与代码修改方案.md` §8。

## 4. 对外三方接口

- **视觉组（上位机 = 瑞莎 ROCK 5C）**：按文档 §4.1 发帧——`AA 55 + 状态 + 类别,形状(固定0),hasQR,0 + X,Y 小端 + 累加校验`，**12 字节定长**，≤100ms 周期，**持续重复发当前目标**；无目标也照周期发状态 0x00（停发会被判成链路故障）。**类别 = 1~6 放在原"颜色"字节里、形状字节固定发 0**（2026-09-30 起不再拆颜色×形状）；电控只认 1~6 且形状=0，越界/非 0 当"本帧无目标"（旧编码漏改时机器不动而不是悄悄放错盒）。目标挑选/去重/像素→**视觉坐标**标定在上位机端；**视觉系→机械系的交换与平移由 STM32 补**（帧里的 X/Y 是视觉系坐标，别按机械轴同向发）。
  ⚠️ **取帧时机**：STM32 只在**投放点上方（不遮挡相机处）**取帧（归零/待机时龙门挡着托盘），物块被夹走后 **<2s** 必须把"当前应抓目标"换成下一个，否则会锁到刚取走物块的旧坐标（抓空、计数虚增）。
  ⚠️ 上位机侧硬要求见文档 §4.1 表：串口用 **pin7/29 的 UART4-M2（`/dev/ttyS4`）**、termios raw、显式 115200、systemd 无显示器自启。
  注：模块名 `k230_uart` / `CFG_K230_*` 是历史命名，含义就是这台上位机。
- **机械组**：投放区坐标、每格容量 ≥4、投放落差兜底、升降行程。
- **屏怎么测**（联调第一件事 = 核控件名与行序，对不上只会"那一列永不刷新"）：PC 直连屏跑 `python Tools/screen_check.py <COM口>`（6 行分别写成行号，屏上出现 1~6 即全对）；或把 `CFG_TJC_SELFTEST_ON_BOOT` 置 1 烧产品固件，上电由主板写同样一组数（连 USART6/CN2/波特率一起验），**正式运行前改回 0**。
- **串口屏（TJC）**：屏是**静态 6 行统计表**（序号/图片/名称由屏工程按抽签画死），主控只写每行的"成功数量"：控件名 = `CFG_TJC_VAR_CNT_BASE` + 数字(`CFG_TJC_CNT_FIRST`+行)，**缺省 `n1~n6`**（行序 = 类别 ID 1~6）；上电清 6 行、**每件把 6 行全部重写**（写绝对值，屏侧静默丢一条也能在下一件补齐）；115200、禁触摸。

## 5. 设计要点

- 电机**只发不收**（广播 0x00、帧尾 0x6B），到位＝时间估算；**位置帧 13 字节**（脉冲后有【运动模式】+【多机同步】），少发一个字节会让驱动器此后一帧都不执行。
- 不读码：货物顶面的"二维码"按规则属模拟污渍干扰信息，分类只看视觉给出的类别号 1~6。
- **读帧位 = 投放点上方**：龙门停在托盘上方会挡住相机（2026-09-30 现场口径），所以每件先到投放点上方、等静默期 + 清视觉缓存，再取帧；每件结束就停在那里（不回待机点，省掉一个来回）。
- 抓空不检测（R5 已接受）；投放＝抬升位直接松爪，落差由机械侧兜底（R1）。
- 屏上只写 6 行的成功数量（名称/图片是屏工程画死的）；屏端字符集 GB2312，指令尾 `FF FF FF`，整帧一次发送（防半截帧让屏解析失步）。
- 中断都不调用 FreeRTOS FromISR API（EXTI 抢占 0 / UART 抢占 5 / TIM6 时基 15）。
- 转盘索引沿同时是绝对基准：结账时用实测沿数核对理论整圈数，对不上就作废本件并重新回零；**一圈只有一个沿，漏检窗口内检不出**（见 `../仿真审计结论.md`）。

## 6. 台架调试固件（做法，当前仓库里没有台架）

调试单块外设时，做一个"上电就跑、不启 RTOS、把结论打成表"的入口，但不为它在 `main()` 里加 `if`：
`User/Bench/` 用 ARM 启动钩子 `$Sub$$main` 在**链接期**替换上电入口，产品 `main()` 一字不改；开关关掉时台架整个编译成空文件。

1. `User/Bench/Inc/sort_bench_select.h`：唯一开关 `SORT_BENCH`（`NONE`＝产品固件，出场必用），指到某个台架即换固件；
2. `User/Bench/Src/xxx_bench_main.c`：`$Sub$$main` + 只开本步要用的外设，末尾调 `Xxx_Bench_Run()`，并冻结 TIM6（`DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM6_STOP`）否则 halt 时判据会变成假 SKIP/假 FAIL；
3. `User/Devices/Src/xxx_bench.c`：一个 `volatile` 结果表（每步一个判定枚举 + 证据值），逐条打印判据，每条判据对应产品代码里写死的一个假设；
4. 时间接缝抽成 `Xxx_Bench_WaitMs()` 这类函数，真机忙等 HAL tick、主机侧仿真用 hook 覆盖，同一份台架代码能在 PC 上跑假硬件；
5. 三条红线：FAIL 只允许表示"硬件或产品代码坏了"（人手没碰的场合判 SKIP 且要有证据）；不改产品代码迁就测试；输出只用 ASCII；
6. 新文件要加进 `smart_sorting.uvprojx` 并重跑 `keil_to_compile_commands.py`。