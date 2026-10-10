/**
 * @file    test_k230_frames.c
 * @brief   主机侧 k230 视觉帧解析器测试（不用板子）：解析器 = 产品源码原样副本
 * @note    帧格式见电控文档 §4.1（12 字节）：
 *            AA 55 | 状态 | 类别 形状 hasQR 数量 X_L X_H Y_L Y_H | 校验
 *          校验 = 前 11 字节累加和低 8 位；X/Y 小端 int16 单位 0.1mm（补码，可为负）
 *          ★ 2026-10-10：下标 6 = **数量位**（原"保留"字节）——本帧托盘上物块总数，
 *            与状态无关地填（无目标帧也要填，托盘空的 0 靠它）
 *          类别过滤（1~6 且形状=0）在缓存层（dbg_vision / 产品 vision_task），**不在解析器**——
 *          所以这里同时断言"解析器把越界类别原样上报"，防止两层职责被悄悄合并
 *          跑法（在 debug_corexy 目录下）：
 *            wsl -e bash -lc 'cd /mnt/d/CUBEMX\ DOCUMENT/工创/debug_corexy && \
 *              gcc -std=gnu99 -Wall -Wextra -I Tools/shim -I Config -I User/Bsp/Inc \
 *              -I User/Devices/Inc \
 *              Tools/test_k230_frames.c User/Devices/Src/k230_uart.c -o /tmp/tk && /tmp/tk'
 * @retval  0 = 全过；非 0 = 失败项数
 */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "k230_uart.h"

static int s_fail = 0;

#define CHECK(cond, name)                                                     \
    do {                                                                      \
        if (cond) {                                                           \
            printf("PASS %s\n", (name));                                      \
        } else {                                                              \
            printf("FAIL %s\n", (name));                                      \
            s_fail++;                                                         \
        }                                                                     \
    } while (0)

/* 组一帧：f[0..10] = 状态 类别 形状 hasQR 数量 X_L X_H Y_L Y_H，f[11] 由本函数填校验 */
static void BuildFrame(uint8_t f[12], uint8_t status, uint8_t category, uint8_t shape, int16_t x10,
                       int16_t y10) {
    uint8_t sum = 0u;

    f[0]  = 0xAAu;
    f[1]  = 0x55u;
    f[2]  = status;
    f[3]  = category;
    f[4]  = shape;
    f[5]  = 0u; /* hasQR */
    f[6]  = 0u; /* 数量位；要给非 0 用 BuildFrameC */
    f[7]  = (uint8_t)(x10 & 0xFF);
    f[8]  = (uint8_t)((x10 >> 8) & 0xFF);
    f[9]  = (uint8_t)(y10 & 0xFF);
    f[10] = (uint8_t)((y10 >> 8) & 0xFF);
    for (uint8_t i = 0u; i < 11u; i++) {
        sum = (uint8_t)(sum + f[i]);
    }
    f[11] = sum;
}

/* 带数量位的一帧（数量位参与校验——这正是"填错会整帧被丢"的来源） */
static void BuildFrameC(uint8_t f[12], uint8_t status, uint8_t category, uint8_t shape, int16_t x10,
                        int16_t y10, uint8_t count) {
    BuildFrame(f, status, category, shape, x10, y10);
    f[6] = count;
    {
        uint8_t sum = 0u;

        for (uint8_t i = 0u; i < 11u; i++) {
            sum = (uint8_t)(sum + f[i]);
        }
        f[11] = sum;
    }
}

/* 逐字节喂一整帧，返回最后一个非 NONE 事件 */
static K230_Event_e FeedBytes(const uint8_t *bytes, uint16_t len, K230_Target_t *out) {
    K230_Event_e ev = K230_EV_NONE;

    for (uint16_t i = 0u; i < len; i++) {
        K230_Event_e e = K230_FeedByte(bytes[i], out);

        if (e != K230_EV_NONE) {
            ev = e;
        }
    }
    return ev;
}

static K230_Event_e FeedTarget(uint8_t status, uint8_t category, uint8_t shape, int16_t x10,
                               int16_t y10, K230_Target_t *out) {
    uint8_t f[12];

    BuildFrame(f, status, category, shape, x10, y10);
    return FeedBytes(f, 12u, out);
}

int main(void) {
    K230_Target_t t;
    K230_Event_e  ev;

    /* 1) 有目标帧：类别 3、x=+60.0mm（x10=600）、y=+45.0mm（450） */
    memset(&t, 0, sizeof(t));
    ev = FeedTarget(0x01u, 3u, 0u, 600, 450, &t);
    CHECK(ev == K230_EV_TARGET, "有目标帧报 EV_TARGET");
    CHECK((t.valid == 1u) && (t.category == 3u) && (t.shape == 0u), "类别/形状字节取对");
    CHECK(fabsf(t.x_mm - 60.0f) < 0.05f, "x=+60.0mm");
    CHECK(fabsf(t.y_mm - 45.0f) < 0.05f, "y=+45.0mm");

    /* 2) 小端显式验证：把 X 两字节按大端摆（0x5825），读数必须变成 2256.5mm */
    memset(&t, 0, sizeof(t));
    {
        uint8_t f[12];
        uint8_t sum = 0u;

        BuildFrame(f, 0x01u, 1u, 0u, 0, 0);
        f[7] = 0x25u;
        f[8] = 0x58u; /* 故意大端 */
        for (uint8_t i = 0u; i < 11u; i++) { /* 字节改了，校验必须跟着重算 */
            sum = (uint8_t)(sum + f[i]);
        }
        f[11] = sum;
        ev    = FeedBytes(f, 12u, &t);
    }
    CHECK((ev == K230_EV_TARGET) && (fabsf(t.x_mm - 2256.5f) < 0.05f), "X 确为小端（反例=2256.5）");

    /* 3) 负坐标：-123.4mm → x10 = -1234（补码） */
    memset(&t, 0, sizeof(t));
    ev = FeedTarget(0x01u, 6u, 0u, -1234, -876, &t);
    CHECK((ev == K230_EV_TARGET) && (fabsf(t.x_mm - (-123.4f)) < 0.05f) &&
              (fabsf(t.y_mm - (-87.6f)) < 0.05f),
          "负坐标 int16 补码还原");

    /* 4) 无目标帧：状态 0x00 → EV_EMPTY 且 valid 清 0（防旧坐标残留） */
    memset(&t, 0, sizeof(t));
    t.valid = 1u; /* 故意先弄脏 */
    ev      = FeedTarget(0x00u, 0u, 0u, 0, 0, &t);
    CHECK((ev == K230_EV_EMPTY) && (t.valid == 0u), "无目标帧清 valid 位");

    /* 5) 校验错：改载荷不动校验 → 整帧丢弃，无事件 */
    {
        uint8_t f[12];

        BuildFrame(f, 0x01u, 2u, 0u, 100, 100);
        f[3] ^= 0xFFu;
        memset(&t, 0, sizeof(t));
        ev = FeedBytes(f, 12u, &t);
    }
    CHECK(ev == K230_EV_NONE, "校验错整帧丢弃");

    /* 6) 帧头前有垃圾 + 连续 0xAA 重同步（AA AA AA 55 …） */
    {
        /* 00 13 是垃圾；三个 AA 里前两个被重同步吸收；帧体 = 01 04 00 00 00 40 00 2C 01，
           x10=0x0040=64→6.4mm、y10=0x012C=300→30.0mm；校验 = AA+55+帧体 累加 = 0x71 */
        uint8_t noisy[] = {0x00, 0x13, 0xAA, 0xAA, 0xAA, 0x55, 0x01, 0x04, 0x00,
                           0x00, 0x00, 0x40, 0x00, 0x2C, 0x01, 0x71};
        memset(&t, 0, sizeof(t));
        ev = FeedBytes(noisy, (uint16_t)sizeof(noisy), &t);
    }
    CHECK((ev == K230_EV_TARGET) && (t.category == 4u) && (fabsf(t.x_mm - 6.4f) < 0.05f),
          "垃圾+连续AA后仍同步成功");

    /* 7) 半帧遇错字节：回帧头搜索后新帧完整解析 */
    memset(&t, 0, sizeof(t));
    (void)K230_FeedByte(0xAAu, &t); /* 只给半个帧头 */
    (void)K230_FeedByte(0x13u, &t); /* 第二字节错 → 回帧头搜索 */
    ev = FeedTarget(0x01u, 5u, 0u, 200, 300, &t);
    CHECK((ev == K230_EV_TARGET) && (t.category == 5u), "半帧作废后重新取帧");

    /* 8) 越界类别解析器**原样上报**（过滤职责在缓存层） */
    memset(&t, 0, sizeof(t));
    ev = FeedTarget(0x01u, 7u, 0u, 10, 10, &t);
    CHECK((ev == K230_EV_TARGET) && (t.category == 7u), "越界类别不在解析器过滤");

    /* 9) 载荷含 0xAA/0x55 不串帧：x10=0xAAAA=-21846→-2184.6、y10=0x5555=21845→2184.5 */
    memset(&t, 0, sizeof(t));
    ev = FeedTarget(0x01u, 2u, 0u, (int16_t)0xAAAA, (int16_t)0x5555, &t);
    CHECK((ev == K230_EV_TARGET) && (fabsf(t.x_mm - (-2184.6f)) < 0.05f) &&
              (fabsf(t.y_mm - 2184.5f) < 0.05f),
          "载荷含 AA/55 不串帧");

    /* 10) 两帧背靠背（中间零间隔）各自解析正确 */
    memset(&t, 0, sizeof(t));
    {
        uint8_t two[24];

        BuildFrame(&two[0], 0x01u, 1u, 0u, 100, 200);
        BuildFrame(&two[12], 0x01u, 2u, 0u, 300, 400);
        ev = FeedBytes(two, 24u, &t);
    }
    CHECK((ev == K230_EV_TARGET) && (t.category == 2u) && (fabsf(t.x_mm - 30.0f) < 0.05f),
          "两帧背靠背取到第二帧");

    /* 11) frame_out = NULL 不崩、事件照报（产品调用方"不要数据"的合法用法） */
    {
        uint8_t f[12];

        BuildFrame(f, 0x01u, 1u, 0u, 0, 0);
        ev = FeedBytes(f, 12u, NULL);
    }
    CHECK(ev == K230_EV_TARGET, "NULL 输出不吞事件");

    /* 12) ★数量位（下标 6，2026-10-10）：有目标帧照常解析出来 */
    memset(&t, 0, sizeof(t));
    {
        uint8_t f[12];

        BuildFrameC(f, 0x01u, 3u, 2u, 120, -80, 7u);
        ev = FeedBytes(f, 12u, &t);
    }
    CHECK((ev == K230_EV_TARGET) && (t.count == 7u), "有目标帧数量位 = 7");

    /* 13) ★无目标帧**也带数量位**：托盘空了要能读出来（状态 00 + 数量 0 = 空盘凭据） */
    memset(&t, 0, sizeof(t));
    {
        uint8_t f[12];

        BuildFrameC(f, 0x00u, 0u, 0u, 0, 0, 0u);
        ev = FeedBytes(f, 12u, &t);
    }
    CHECK((ev == K230_EV_EMPTY) && (t.count == 0u) && (t.valid == 0u),
          "无目标帧填数量位且清 valid");

    /* 14) 数量位边界：255（uint8 满量程）原样读出 */
    memset(&t, 0, sizeof(t));
    {
        uint8_t f[12];

        BuildFrameC(f, 0x01u, 1u, 0u, 0, 0, 255u);
        ev = FeedBytes(f, 12u, &t);
    }
    CHECK((ev == K230_EV_TARGET) && (t.count == 255u), "数量位 255 原样读出");

    /* 15) ★数量位进校验：改了数量位而不重算校验 ⇒ 整帧丢弃（防"数量被噪声改写却照用"） */
    {
        uint8_t f[12];

        BuildFrameC(f, 0x01u, 2u, 0u, 10, 10, 4u);
        f[6] = 9u; /* 只改数量位，校验不动 */
        memset(&t, 0, sizeof(t));
        ev = FeedBytes(f, 12u, &t);
    }
    CHECK(ev == K230_EV_NONE, "数量位改错而不重算校验 -> 整帧丢弃");

    /* 16) 数量位不干扰坐标/类别：同帧里三者各自独立取对 */
    memset(&t, 0, sizeof(t));
    {
        uint8_t f[12];

        BuildFrameC(f, 0x01u, 5u, 3u, -1234, 876, 12u);
        ev = FeedBytes(f, 12u, &t);
    }
    CHECK((ev == K230_EV_TARGET) && (t.category == 5u) && (t.shape == 3u) && (t.count == 12u) &&
              (fabsf(t.x_mm - (-123.4f)) < 0.05f) && (fabsf(t.y_mm - 87.6f) < 0.05f),
          "数量位与类别/形状/坐标互不干扰");

    printf(s_fail == 0 ? "\n全部通过\n" : "\n有 %d 项失败\n", s_fail);
    return s_fail;
}
