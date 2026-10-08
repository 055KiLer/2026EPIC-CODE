/**
 * @file    test_zdt_frames.c
 * @brief   帧字节的主机侧测试（不用板子）
 * @note    为什么值得测：驱动器只看字节，多一个/少一个字节、字节序反了、
 *          校验字节位置错了，现象都是"电机一动不动"或"动得莫名其妙"，
 *          而在台架上根本看不出是哪一字节的问题。这里用**两份独立来源**交叉验证：
 *            ① 参考实现 = 物创 `Harvesting Robot/drive/stepper_drive.c` 的字段顺序
 *               （同一块板、同一批驱动器上跑过的代码）
 *            ② 字面量 = 物创 `code/ZDT_drive.c` 里那些 static 帧数组（手册示例抄下来的）
 *          跑法（在 debug_corexy 目录下）：
 *            wsl -e bash -lc 'cd /mnt/d/CUBEMX\ DOCUMENT/工创/debug_corexy && \
 *              gcc -std=gnu99 -Wall -Wextra -I Tools/shim -I User/Bsp/Inc -I User/Devices/Inc \
 *              -I User/Devices/Src \
 *              Tools/test_zdt_frames.c User/Devices/Src/zdt_bus.c -o /tmp/tf && /tmp/tf'
 */
#include <stdio.h>
#include <string.h>

#include "dbg_config.h"
#include "zdt_bus.h"

/* ---------- 替身：把发到各条电机线的字节抓进缓冲（带线路标记） ---------- */
#define CAP_MAX 64
static uint8_t  s_sent[CAP_MAX];
static uint8_t  s_sent_line[CAP_MAX]; /* 每个字节走的线：0=A 1=B */
static uint16_t s_sent_len;

static void Cap_Reset(void) {
    s_sent_len = 0u;
}

uint8_t Dbg_Uart_SendMotor(uint8_t motor, const uint8_t *data, uint16_t len) {
    for (uint16_t i = 0u; i < len; i++) {
        if (s_sent_len < CAP_MAX) {
            s_sent[s_sent_len]      = data[i];
            s_sent_line[s_sent_len] = motor;
            s_sent_len++;
        }
    }
    return 1u;
}

/* 打印工具（dbg_print.c 不参与本测试，这里只要能看到失败在哪一字节） */
uint8_t Dbg_Uart_SendConsole(const uint8_t *data, uint16_t len) {
    (void)data;
    (void)len;
    return 1u;
}

static int    g_fail;
static int    g_pass;

static void Check(const char *what, int cond) {
    if (cond) {
        g_pass++;
    } else {
        g_fail++;
        printf("  FAIL  %s\n", what);
    }
}

static void Dump(const char *tag, const uint8_t *b, uint16_t n) {
    printf("        %s (%u):", tag, (unsigned)n);
    for (uint16_t i = 0u; i < n; i++) {
        printf(" %02x", (unsigned)b[i]);
    }
    printf("\n");
}

static int BytesEq(const uint8_t *a, const uint8_t *b, uint16_t n) {
    return (memcmp(a, b, n) == 0) ? 1 : 0;
}

/* ============ ② 手册/参考固件里的字面量 ============ */
/* 物创 code/ZDT_drive.c：
     static uint8_t enable[]={0x01,0xF3,0xAB,0x01,0x00,0x6B};
     static uint8_t zero_set[]={0x01,0x93,0x88,0x01,0x6B};
     位置帧模板（相对）= {0x01,0xFD,dir,spH,spL,0xFF,p0,p1,p2,p3,md,sync,0x6B}
   注意：那份模板的方向字节是 0x01，代表它自己的极性约定；本工程按 ZDT 手册
   与产品 code/ 的口径（正=CW=0x00），两者**方向字节相反**，这是设计选择不是 bug，
   下面单独测这一条，免得被误判成"帧错了" */
static const uint8_t REF_ENABLE_1[]   = {0x01, 0xF3, 0xAB, 0x01, 0x00, 0x6B};
static const uint8_t REF_ZERO_1[]     = {0x01, 0x93, 0x88, 0x01, 0x6B};

/* ============ ① 参考实现：按物创 stepper_drive.c 的字段顺序搭帧 ============ */
/* 只用于"字段位置/字节序"的交叉验证，因此方向字节也用他们那份的极性 */
static uint16_t RefBuildPosition(const uint8_t *id_fd, int32_t pulse, uint16_t speed,
                                 uint8_t accel, uint8_t md, uint8_t *out) {
    uint32_t abs_p   = (pulse >= 0) ? (uint32_t)pulse : (uint32_t)(-pulse);
    uint8_t  dir     = (pulse > 0) ? 0x01u : 0x00u; /* 物创极性 */

    out[0]  = id_fd[0];
    out[1]  = 0xFDu;
    out[2]  = dir;
    out[3]  = (uint8_t)(speed >> 8);
    out[4]  = (uint8_t)speed;
    out[5]  = accel;
    out[6]  = (uint8_t)(abs_p >> 24);
    out[7]  = (uint8_t)(abs_p >> 16);
    out[8]  = (uint8_t)(abs_p >> 8);
    out[9]  = (uint8_t)abs_p;
    out[10] = md;
    out[11] = 0x00u;
    out[12] = 0x6Bu;
    return 13u;
}

/* ============ ③ 参考工程 `D:\CUBEMX DOCUMENT\motor test` 的帧 ============ */
/* 它的 Core/Src/stepper_motor.c 是**照抄式**实现：帧模板写死、把速度/脉冲填进去。
   这份工程的方向枚举是 STEPPER_DIR_CW = 0x00 / CCW = 0x01（与产品、与本工程同口径），
   全部帧都用广播地址 0x00（正好是本工程 `en 0` / `bx` / `stop` 那条路）。
   下面按它的代码原样拼一遍，再与本工程的构帧函数逐字节比。 */
static const uint8_t MT_BROADCAST = 0x00u;

static uint16_t MtBuildEnable(uint8_t enable, uint8_t *out) {
    const uint8_t t[6] = {MT_BROADCAST, 0xF3u, 0xABu, (uint8_t)(enable ? 1u : 0u), 0x00u, 0x6Bu};

    memcpy(out, t, 6u);
    return 6u;
}

static uint16_t MtBuildMoveRelative(uint8_t dir, uint16_t speed_rpm, uint8_t accel, uint32_t pulse,
                                    uint8_t *out) {
    uint8_t t[13] = {MT_BROADCAST, 0xFDu, dir, 0x00u, 0x00u, accel, 0x00u,
                     0x00u,       0x00u, 0x00u, 0x00u, 0x00u, 0x6Bu};

    t[3] = (uint8_t)(speed_rpm >> 8);
    t[4] = (uint8_t)speed_rpm;
    t[6] = (uint8_t)(pulse >> 24);
    t[7] = (uint8_t)(pulse >> 16);
    t[8] = (uint8_t)(pulse >> 8);
    t[9] = (uint8_t)pulse;
    memcpy(out, t, 13u);
    return 13u;
}

static uint16_t MtBuildStop(uint8_t *out) {
    const uint8_t t[5] = {MT_BROADCAST, 0xFEu, 0x98u, 0x00u, 0x6Bu};

    memcpy(out, t, 5u);
    return 5u;
}

int main(void) {
    uint8_t  f[16];
    uint16_t n;

    printf("== 1. 使能帧字面量（物创 ZDT_drive.c） ==\n");
    n = Zdt_Bus_BuildEnable(1u, 1u, f);
    Dump("mine ", f, n);
    Dump("ref  ", REF_ENABLE_1, sizeof(REF_ENABLE_1));
    Check("使能帧 6 字节", n == 6u);
    Check("使能帧字节一致", BytesEq(f, REF_ENABLE_1, 6u));

    n = Zdt_Bus_BuildEnable(1u, 0u, f);
    Check("失能帧第 4 字节 = 00", (n == 6u) && (f[3] == 0x00u) && (f[5] == 0x6Bu));

    printf("== 2. 位置帧：长度与字段位置 ==\n");
    n = Zdt_Bus_BuildPosition(1u, 3200, 45u, 255u, f);
    Dump("mine ", f, n);
    Check("位置帧 13 字节（少一字节驱动器此后一帧都不执行）", n == 13u);
    Check("功能码 = FD", f[1] == 0xFDu);
    Check("正脉冲 -> 方向 0x00（产品/手册口径正=CW）", f[2] == 0x00u);
    Check("速度大端 45 -> 00 2D", (f[3] == 0x00u) && (f[4] == 0x2Du));
    Check("加速度字节 = 255（同物创固件固化值）", f[5] == 0xFFu);
    Check("脉冲大端 3200 -> 00 00 0C 80", (f[6] == 0x00u) && (f[7] == 0x00u) &&
                                              (f[8] == 0x0Cu) && (f[9] == 0x80u));
    Check("运动模式 = 00（相对上一目标位置；0x02 已于 2026-10-08 台架证伪）", f[10] == 0x00u);
    Check("多机同步 = 00（立即执行）", f[11] == 0x00u);
    Check("校验字节 = 6B", f[12] == 0x6Bu);

    printf("== 3. 与参考实现的字段顺序交叉验证（除方向极性外必须逐字节一致） ==\n");
    {
        const struct {
            int32_t  pulse;
            uint16_t speed;
            uint8_t  accel;
        } cases[] = {
            {3200, 45u, 255u},   {1, 1u, 255u},      {1048576, 3000u, 10u},
            {-3200, 45u, 255u},  {-1, 1u, 0u},       {16777216, 5000u, 255u},
            {80, 256u, 128u},    {255, 65535u, 1u},
        };

        for (unsigned i = 0u; i < (sizeof(cases) / sizeof(cases[0])); i++) {
            uint8_t id_fd[2] = {1u, 0xFDu};
            uint8_t mine[13];
            uint8_t theirs[13];
            uint16_t mn = Zdt_Bus_BuildPosition(1u, cases[i].pulse, cases[i].speed,
                                                cases[i].accel, mine);
            uint16_t rn = RefBuildPosition(id_fd, cases[i].pulse, cases[i].speed,
                                           cases[i].accel, 0x00u, theirs);

            /* 方向字节极性不同：把它对齐后再比其余 12 字节 */
            theirs[2] = mine[2];
            if ((mn != 13u) || (rn != 13u) || !BytesEq(mine, theirs, 13u)) {
                printf("  case %u pulse=%d speed=%u accel=%u\n", i, (int)cases[i].pulse,
                       (unsigned)cases[i].speed, (unsigned)cases[i].accel);
                Dump("mine ", mine, mn);
                Dump("ref  ", theirs, rn);
            }
            Check("字段顺序/字节序一致", (mn == 13u) && (rn == 13u) && BytesEq(mine, theirs, 13u));
        }
    }

    printf("== 4. 方向极性：正脉冲两个实现应当相反（设计选择，台架定） ==\n");
    {
        uint8_t mine[13];
        uint8_t theirs[13];
        uint8_t id_fd[2] = {1u, 0xFDu};

        (void)Zdt_Bus_BuildPosition(1u, 100, 45u, 255u, mine);
        (void)RefBuildPosition(id_fd, 100, 45u, 255u, 0x00u, theirs);
        Check("正脉冲：mine=0x00 且 参考固件=0x01", (mine[2] == 0x00u) && (theirs[2] == 0x01u));
        (void)Zdt_Bus_BuildPosition(1u, -100, 45u, 255u, mine);
        (void)RefBuildPosition(id_fd, -100, 45u, 255u, 0x00u, theirs);
        Check("负脉冲：mine=0x01 且 参考固件=0x00", (mine[2] == 0x01u) && (theirs[2] == 0x00u));
    }

    printf("== 5. 发送路径：零脉冲不发、地址字节 = 目标地址 ==\n");
    Cap_Reset();
    Check("零脉冲视为成功且线上不发任何字节", (Zdt_Bus_MoveOn(0u, 3u, 0, 45u, 255u) == 1u) &&
                                                (s_sent_len == 0u));

    Cap_Reset();
    (void)Zdt_Bus_MoveOn(0u, 3u, 3200, 45u, 255u);
    Dump("sent ", s_sent, s_sent_len);
    Check("整帧 13 字节上线", s_sent_len == 13u);
    Check("第 1 字节 = 目标地址 3", (s_sent_len == 13u) && (s_sent[0] == 3u));
    Check("帧走的是 A 线（motor 0）", (s_sent_len == 13u) && (s_sent_line[0] == 0u));

    Cap_Reset();
    (void)Zdt_Bus_StopOn(1u, 7u);
    Dump("stop ", s_sent, s_sent_len);
    {
        const uint8_t ref_stop[5] = {0x07u, 0xFEu, 0x98u, 0x00u, 0x6Bu};
        Check("停止帧 = [地址,FE,98,00,6B]", BytesEq(s_sent, ref_stop, 5u));
        Check("帧走的是 B 线（motor 1）", (s_sent_len == 5u) && (s_sent_line[0] == 1u));
    }

    Cap_Reset();
    (void)Zdt_Bus_ZeroPositionOn(0u, 1u);
    Dump("zero ", s_sent, s_sent_len);
    Check("置零帧与物创字面量一致", BytesEq(s_sent, REF_ZERO_1, sizeof(REF_ZERO_1)));

    printf("== 6. StopBoth 路由：三条线各一帧广播停（全广播定调） ==\n");
    Cap_Reset();
    (void)Zdt_Bus_StopBoth();
    Dump("stopboth", s_sent, s_sent_len);
    Check("共 15 字节 = 3 线 × 5 字节", s_sent_len == 15u);
    {
        const uint8_t want[15] = {0x00u, 0xFEu, 0x98u, 0x00u, 0x6Bu,
                                  0x00u, 0xFEu, 0x98u, 0x00u, 0x6Bu,
                                  0x00u, 0xFEu, 0x98u, 0x00u, 0x6Bu};
        uint8_t       line_ok = 1u;

        for (uint16_t i = 0u; i < 5u; i++) {
            if ((s_sent_line[i] != 0u) || (s_sent_line[5u + i] != 1u) ||
                (s_sent_line[10u + i] != 2u)) {
                line_ok = 0u;
            }
        }
        Check("三帧都是广播停止帧 [00,FE,98,00,6B]", BytesEq(s_sent, want, 15u));
        Check("线路由正确（A 线→B 线→T 线）", line_ok != 0u);
    }

    printf("== 7. 故障路径 PanicStop：每条线一帧广播停 ==\n");
    Cap_Reset(); /* 故障路径走 Dbg_Uart_SendMotor（寄存器级、不读 tick），所以抓这里 */
    Zdt_Bus_PanicStop();
    Dump("panic", s_sent, s_sent_len);
    Check("共 15 字节 = 3 线 × 5 字节", s_sent_len == 15u);
    {
        const uint8_t want[15] = {0x00u, 0xFEu, 0x98u, 0x00u, 0x6Bu,  /* A 线广播停 */
                                  0x00u, 0xFEu, 0x98u, 0x00u, 0x6Bu,  /* B 线广播停 */
                                  0x00u, 0xFEu, 0x98u, 0x00u, 0x6Bu}; /* T 线广播停 */
        Check("三线各一帧广播停，帧体都合法", BytesEq(s_sent, want, 15u));
    }

    printf("== 8. 与参考工程 motor test 的帧逐字节比（广播地址那条路） ==\n");
    {
        uint8_t mine[13];
        uint8_t theirs[13];
        uint16_t mn;
        uint16_t rn;

        mn = Zdt_Bus_BuildEnable(MT_BROADCAST, 1u, mine);
        rn = MtBuildEnable(1u, theirs);
        Dump("mine(广播使能)", mine, mn);
        Dump("motor test  ", theirs, rn);
        Check("广播使能帧一致", (mn == rn) && BytesEq(mine, theirs, mn));

        mn = Zdt_Bus_BuildEnable(MT_BROADCAST, 0u, mine);
        rn = MtBuildEnable(0u, theirs);
        Check("广播失能帧一致", (mn == rn) && BytesEq(mine, theirs, mn));

        /* 位置帧：他们的方向枚举 CW=0x00/CCW=0x01，与本工程同口径，直接整帧比 */
        const struct {
            int32_t  pulse;
            uint16_t speed;
            uint8_t  accel;
            uint8_t  dir;
        } cases[] = {
            {3200, 45u, 255u, 0x00u},  {-3200, 45u, 255u, 0x01u}, {8000, 30u, 255u, 0x00u},
            {1, 1u, 0u, 0x00u},        {16777215u, 5000u, 255u, 0x00u},
        };

        for (unsigned i = 0u; i < (sizeof(cases) / sizeof(cases[0])); i++) {
            uint32_t abs_p = (uint32_t)((cases[i].pulse > 0) ? cases[i].pulse : -cases[i].pulse);

            mn = Zdt_Bus_BuildPosition(MT_BROADCAST, cases[i].pulse, cases[i].speed, cases[i].accel,
                                       mine);
            rn = MtBuildMoveRelative(cases[i].dir, cases[i].speed, cases[i].accel, abs_p, theirs);
            if ((mn != rn) || !BytesEq(mine, theirs, 13u)) {
                printf("  case %u pulse=%d speed=%u accel=%u\n", i, (int)cases[i].pulse,
                       (unsigned)cases[i].speed, (unsigned)cases[i].accel);
                Dump("mine ", mine, mn);
                Dump("motor test  ", theirs, rn);
            }
            Check("广播位置帧一致（含方向字节）", (mn == 13u) && (rn == 13u) &&
                                                      BytesEq(mine, theirs, 13u));
        }

        /* 方向枚举对得上：正脉冲 -> 0x00（他们叫 STEPPER_DIR_CW，本工程叫"正=CW"） */
        (void)Zdt_Bus_BuildPosition(MT_BROADCAST, 100, 45u, 255u, mine);
        Check("参考工程 CW = 0x00，与本工程正脉冲的一致性成立", (mine[2] == 0x00u));
    }

    printf("== 8. 广播停止帧与 motor test 一致 ==\n");
    {
        uint8_t  theirs[8];
        uint16_t rn = MtBuildStop(theirs);

        Cap_Reset();
        (void)Zdt_Bus_StopOn(0u, MT_BROADCAST); /* 走真实发送路径，抓它发出去的字节 */
        Dump("mine(广播停)", s_sent, s_sent_len);
        Dump("motor test  ", theirs, rn);
        Check("广播停止帧一致", (s_sent_len == rn) && BytesEq(s_sent, theirs, rn));
    }

    printf("\n== 结果：%d 项通过，%d 项失败 ==\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}