/**
 * @file    test_fmt_flags.c
 * @brief   格式器"标志位"专项测试（不占板子）
 * @note    为什么单独测：漏掉一个不认识的标志（比如 %+d 的 '+'）时，格式器会打一个 '?'
 *          却**不消费参数**，导致后面所有参数整体错位一位 —— 实测踩过：
 *          广播命令打出"各 20 脉冲 @1600rpm"，而真实值是"各 1600 脉冲 @30rpm"。
 *          这类错误在板子上看着像"数据不对"，很容易往硬件上怀疑，所以必须钉住。
 *          跑法：
 *            wsl -e bash -lc 'cd /mnt/d/CUBEMX\ DOCUMENT/工创/debug_corexy && \
 *              gcc -std=gnu99 -Wall -Wextra -I Tools/shim -I User/Bsp/Inc \
 *              Tools/test_fmt_flags.c User/Bsp/Src/dbg_print.c -o /tmp/tfl && /tmp/tfl'
 */
#include <stdio.h>
#include <string.h>

#include "dbg_print.h"

static char     s_cap[512];
static uint16_t s_len;

uint8_t Dbg_Uart_SendConsole(const uint8_t *data, uint16_t len) {
    for (uint16_t i = 0u; i < len; i++) {
        if (s_len < 511u) {
            s_cap[s_len] = (char)data[i];
            s_len++;
        }
    }
    s_cap[s_len] = '\0';
    return 1u;
}

static int s_fail;

static void Chk(const char *what, const char *want) {
    if (strcmp(s_cap, want) != 0) {
        printf("  FAIL  %s\n        得到 [%s]\n        期望 [%s]\n", what, s_cap, want);
        s_fail++;
    } else {
        printf("  PASS  %s\n", what);
    }
}

int main(void) {
    printf("格式器标志位测试\n");

    s_len = 0u;
    Dbg_Printf("%+d|%u|%u\n", -5, 7u, 9u);
    Chk("负值不打多余的 +，且参数不错位", "-5|7|9\n");

    s_len = 0u;
    Dbg_Printf("%+d|%u\n", 5, 7u);
    Chk("正值带 +，且参数不错位", "+5|7\n");

    s_len = 0u;
    Dbg_Printf("[INFO] 广播 X%+dmm（各 %u 脉冲 @%urpm）估算 %u ms\n", 20, 1600u, 30u, 1200u);
    Chk("广播那行的真实形态", "[INFO] 广播 X+20mm（各 1600 脉冲 @30rpm）估算 1200 ms\n");

    s_len = 0u;
    Dbg_Printf("%-6s|%02x|%d\n", "ab", 0x0fu, -12);
    Chk("原有左对齐/补零/负数未受影响", "ab    |0f|-12\n");

    s_len = 0u;
    Dbg_Printf("%d%%\n", 50);
    Chk("百分号转义仍是 %%", "50%\n");

    printf("------------------------------------------------\n");
    printf(s_fail ? "有 %d 项失败\n" : "全部通过\n", s_fail);
    return (s_fail == 0) ? 0 : 1;
}