# 智能分拣装置 · 电控仓库导览

2027 中国大学生工程实践与创新能力大赛"智能+工程创新赛道 · 智能分拣"赛项的电控实现：
**STM32F407VET6 + FreeRTOS + CoreXY 龙门 + 6 格转盘 + 视觉上位机（ROCK 5C）**。
一处一件、抓到就投、投进哪一格看视觉报的**类别 1~6**，屏上按类别各自累计成功数。

---

## 1. 我要的东西在哪（文档地图）

| 我想…… | 看这份 | 它的权威范围 |
|---|---|---|
| 知道硬件怎么接、流程怎么走、协议字节是什么 | **`智能分拣电控文档.md`** | ★**规格真源**（引脚/协议/坐标/流程/异常） |
| 改一个常量、做一次标定 | **`code/Config/sort_config.h`** | ★**标定与算法常量的唯一真源**（每条常量旁边就写着它的取舍与现场判据） |
| 编译 / 烧录 / 看代码结构 / 三方接口 | `code/README.md` | 产品工程速查 |
| 第一次读这套代码 | `code/阅读路线.md` | 3 小时五站阅读路线 |
| 查"某条规则为什么长成现在这样" | **`台架调试记录与结论.md` 第一部分（主题结案）** | ★现场定案与取舍论证 |
| 查"哪一天做了什么、当时贴了什么日志" | `台架调试记录与结论.md` 第二部分（时间线日志） | 原始证据（版本戳/日志/实测数字） |
| 跑仿真、看回归基线 | **`仿真审计结论.md`** | ★**基线数字的唯一权威**（+ 仿真能证明/不能证明什么） |
| 知道下一步做什么、什么算过 | `调试与推进计划.md` | 阶段 0~4 验收；**附录 = 阶段 0/1 逐步操作**（原《调试开工手册》） |
| 上板前一步步怎么敲、不对先查哪里 | `debug_corexy/上台架步骤.md` | 台架操作单（按时间排，含期望输出与分诊） |
| 现场核 5 秒补货窗 / 数量位活不活 | `debug_corexy/README.md` 的 `k230 count` 与 `k230 empty` | 台架命令用法与判据 |
| 台架固件的命令全表 / 接线表 / 待核实清单 | `debug_corexy/README.md` | 台架固件参考手册 |
| 赛题原文 | `题目解析_提取文本.txt` | 官方条款（P26~P33 抽取） |

> 约定：**同一件事只在上面标 ★ 的那一份里写定稿**，其它文档只做指路或操作说明——
> 数字（基线件数/测试项数）一律回指 `仿真审计结论.md`，规则一律回指 `智能分拣电控文档.md` 与产品 `sort_config.h`。

## 2. 三个目录

```
code/           产品工程（Keil 工程 code/MDK-ARM/smart_sorting.uvprojx；比赛烧的就是它）
   Config/      ★ sort_config.h = 全部可调常量（标定只改这里）
   User/        四层：Algorithm / App / Bsp / Devices（依赖单向 App→Devices→Bsp）
   Core/        CubeMX 生成（引脚/时钟/中断；USER CODE 段外的改动会被重生成冲掉）
   sim/         主机侧仿真台（WSL + gcc；产品源码原样编译，替换 HAL/RTOS 与物理边界）
   Tools/       上位机侧小工具（如屏自检脚本）

debug_corexy/   台架调试工程（**按团队约定不入库**；唯一带控制台、能在板上逐条敲命令的那套）
   Config/      dbg_config.h = 台架接线与实验旋钮；sort_config.h = **薄壳，直含产品那份**
   User/        与产品同构的四层 + 命令台（60+ 命令）
   Tools/       hostcheck/ = 主机端编译校验；test_*.c = 四套主机侧单元测试

（根目录 = 文档 + 各一个 .gitignore；code/ 是独立 git 仓库）
```

## 3. 常用命令（本机没有 Keil：构建与回归都在 WSL 里跑）

```bash
# 仿真台：全场景回归（基线见 仿真审计结论.md；唯一 FAIL = 刻意保留的 s6）
wsl -e bash -lc 'cd "/mnt/d/CUBEMX DOCUMENT/工创/code/sim" && make run'

# 仿真台：单跑一个场景（A=v 打逐步轨迹）
wsl -e bash -lc 'cd "/mnt/d/CUBEMX DOCUMENT/工创/code/sim" && make s24 A=v'

# 台架工程：主机端编译校验（不用 Keil；0 错误 0 警告才算过）
wsl -e bash -lc 'cd "/mnt/d/CUBEMX DOCUMENT/工创/debug_corexy" && bash Tools/hostcheck/check.sh'

# 台架工程：四套主机侧单元测试（帧字节/视觉解析/打印格式器/格式标志）
wsl -e bash -lc 'cd "/mnt/d/CUBEMX DOCUMENT/工创/debug_corexy" && \
  gcc -std=gnu99 -Wall -Wextra -I Tools/shim -I Config -I User/Bsp/Inc -I User/Devices/Inc \
  Tools/test_zdt_frames.c User/Devices/Src/zdt_bus.c -o /tmp/tz && /tmp/tz'
# （其余三套同式：test_k230_frames.c + User/Devices/Src/k230_uart.c、
#   test_dbg_print.c / test_fmt_flags.c + User/Bsp/Src/dbg_print.c）

# 真机两条（需要 Keil + 板子）：
#   产品：code/MDK-ARM/smart_sorting.uvprojx（UV4 -r 全量重编译 → 0E/0W；-f 是烧录）
#   台架：debug_corexy/MDK-ARM/debug_corexy.uvprojx；上电横幅/stat 先核版本戳
```

## 4. 改代码前先知道的三条纪律

1. **常量只改一处**：标定/算法常量全在 `code/Config/sort_config.h`；台架工程的 `Config/sort_config.h`
   是**薄壳（直含产品文件）**，仿真台的世界常量也按它对齐（不一致时 sim 启动就终止）。
   要求"三处同值"的时代已经过去——现在是**一处真源 + 两处引用**。
2. **改完必过回归**：仿真 `make run` 回到基线（多一个 FAIL 就是回归）+ 台架 `Tools/hostcheck/check.sh`
   0E/0W + 四套主机测试全过。仿真台抓过真缺陷（活锁、计数不自愈、TrayCrossings 相位错、模型与标定脱钩等），
   它的判据被反向验证过（改坏=变红）。
3. **现场现象先分诊再改**：`台架调试记录与结论.md` 第一部分每条结案都带"现场判据"（看到 X 先查 Y），
   绝大多数"机器不动/计数不对"不需要动代码，先按判据定位到底层故障（贴边越界、数量位没上线、
   链路/波特率、限位毛刺、驱动器状态）。