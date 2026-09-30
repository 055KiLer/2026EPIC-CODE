/**
 * @file    test_dbg_print.c
 * @brief   打印格式器的主机侧测试（不用板子）
 * @note    为什么值得测：板上所有判据、命令回显、帮助表都经过这个格式器，
 *          它错了就是"输出乱码/参数错位"，而在板子上很难看出是格式器的锅。
 *          跑法（在 debug_corexy 目录下）：
 *            wsl -e bash -lc 'cd /mnt/d/CUBEMX\ DOCUMENT/工创/debug_corexy && \
 *              gcc -std=gnu99 -Wall -Wextra -I Tools/shim -I User/Bsp/Inc \
 *              Tools/test_dbg_print.c User/Bsp/Src/dbg_print.c -o /tmp/t && /tmp/t'
 *          （用 Tools/shim 里的 main.h / dbg_config.h 顶掉真机头文件）
 */
#include <stdio.h>
#include <string.h>

#include "dbg_print.h"

/* ---------- 替身：把"串口发出的字节"抓进缓冲 ---------- */
#define CAP_MAX 512
static char     s_cap[CAP_MAX];
static uint16_t s_cap_len;

uint8_t Dbg_Uart_SendConsole(const uint8_t *data, uint16_t len) {
    for (uint16_t i = 0u; i < len; i++) {
        if (s_cap_len < (CAP_MAX - 1u)) {
            s_cap[s_cap_len] = (char)data[i];
            s_cap_len++;
        }
    }
    s_cap[s_cap_len] = '\0';
    return 1u;
}

static int s_fail;

static void cap_reset(void) {
    s_cap_len = 0u;
    s_cap[0]  = '\0';
}

static void check(const char *name, const char *expected) {
    if (strcmp(s_cap, expected) == 0) {
        printf("  PASS  %s\n", name);
        return;
    }
    printf("  FAIL  %s\n        期望 [%s]\n        实际 [%s]\n", name, expected, s_cap);
    s_fail++;
}

int main(void) {
    printf("打印格式器主机侧测试\n");

    cap_reset();
    Dbg_Printf("u=%u d=%d x=%x X=%X c=%c s=%s p=%%\n", 42u, -7, 0xABu, 0xABu, 'Z', "ok");
    check("基本说明符", "u=42 d=-7 x=ab X=AB c=Z s=ok p=%\n");

    cap_reset();
    Dbg_Printf("%d %d %u\n", -1, -2147483647 - 1, 4294967295u);
    check("负数与极限值", "-1 -2147483648 4294967295\n");

    cap_reset();
    Dbg_Printf("[%5u|%05u|%2u|%02x|%08X]\n", 42u, 42u, 12345u, 0x0Au, 0xDEADu);
    check("位宽与补零", "[   42|00042|12345|0a|0000DEAD]\n");

    cap_reset();
    Dbg_Printf("[%-6s|%-4s|%-10s]\n", "en", "x", "abc");
    check("左对齐 %-Ns（位宽按字节数）", "[en    |x   |abc       ]\n");

    cap_reset();
    Dbg_Printf("[%8s|%s]\n", "hi", (const char *)NULL);
    check("右对齐与 NULL", "[      hi|(null)]\n");

    cap_reset();
    Dbg_Printf("float=%f\n", 1.5);
    check("不支持的 %f 打问号", "float=?\n");

    /* 超长行：内部行缓冲 128 字节，应切成多段发出且一个字符不丢。
       用程序构造，避免手数长度（上一次就是数错了） */
    {
        char big[193];

        for (size_t i = 0u; i < 192u; i++) {
            big[i] = 'a';
        }
        big[192] = '\0';

        cap_reset();
        Dbg_Printf("%s\n", big);

        size_t len = strlen(s_cap);
        int    ok  = (len == 193u);
        for (size_t i = 0u; ok && (i < 192u); i++) {
            ok = (s_cap[i] == 'a');
        }
        ok = ok && (s_cap[192] == '\n');
        printf("  %s  超长行分段（长度 %u，应 193，内容全 a）\n", ok ? "PASS" : "FAIL",
               (unsigned)len);
        if (!ok) {
            s_fail++;
        }
    }

    printf("------------------------------------------------\n");
    printf("%s\n", (s_fail == 0) ? "全部通过" : "有失败项");
    return (s_fail == 0) ? 0 : 1;
}