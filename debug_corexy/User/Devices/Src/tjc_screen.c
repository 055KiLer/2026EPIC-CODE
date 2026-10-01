/**
 * @file    tjc_screen.c
 * @brief   TJC 指令发送（阻塞）：每条指令连同帧尾 FF FF FF 拼进同一缓冲一次发出
 * @note    与产品 code/User/Devices/Src/tjc_screen.c 逐行同源，只有两处不同：
 *            1) 发送走 Dbg_Uart_SendScreen（USART6），产品走 Bsp_Uart_Send
 *            2) 删去 Tjc_Init（调试工程无需上电置主界面那一步，屏幕自带上电主界面）
 *          半途失败只会丢整条指令，不会把半截帧留在屏的接收缓冲里让后面所有指令错位
 */
#include "tjc_screen.h"
#include "dbg_uart.h"
#include "sort_config.h"
#include <string.h>

/* 控件名长度在编译期锁死：改长名字直接编译失败，而不是运行期栈溢出 */
typedef char tjc_name_len_check[(sizeof(CFG_TJC_VAR_CNT_BASE) - 1u + 4u <= CFG_TJC_NAME_MAX) ? 1 : -1];

#define TJC_TEXT_MAX 48u

#define TJC_BUF_VAL (CFG_TJC_NAME_MAX + 6u + 11u + 3u) /* ".pic=" 5 + NUL */
#define TJC_BUF_TEXT (CFG_TJC_NAME_MAX + 6u + 2u + TJC_TEXT_MAX + 3u)

/* 有界追加：返回新长度，绝不越过 cap */
static uint16_t Tjc_Put(char *dst, uint16_t cap, uint16_t len, const char *s) {
    while ((s != NULL) && (*s != '\0') && (len < cap)) {
        dst[len++] = *s++;
    }
    return len;
}

static int Tjc_Itoa(int32_t value, char *dst) {
    char     tmp[12];
    int      n = 0;
    int      i = 0;
    /* 先转无符号再取负：直接写 -value 在 value==INT32_MIN 时是有符号溢出（UB） */
    uint32_t u = (value < 0) ? (uint32_t)(-(uint32_t)value) : (uint32_t)value;

    do {
        tmp[n++] = (char)('0' + (u % 10u));
        u /= 10u;
    } while (u > 0u);

    if (value < 0) {
        dst[i++] = '-';
    }
    while (n > 0) {
        dst[i++] = tmp[--n];
    }
    dst[i] = '\0';
    return i;
}

static uint8_t Tjc_SendFrame(const char *buf, uint16_t len) {
    return Dbg_Uart_SendScreen((const uint8_t *)buf, len);
}

uint8_t Tjc_SetText(const char *var, const char *val) {
    char     buf[TJC_BUF_TEXT];
    uint16_t len = 0u;

    len        = Tjc_Put(buf, (uint16_t)(sizeof(buf) - 3u), len, var);
    len        = Tjc_Put(buf, (uint16_t)(sizeof(buf) - 3u), len, ".txt=\"");
    len        = Tjc_Put(buf, (uint16_t)(sizeof(buf) - 3u), len, val);
    len        = Tjc_Put(buf, (uint16_t)(sizeof(buf) - 3u), len, "\"");
    buf[len++] = (char)0xFFu;
    buf[len++] = (char)0xFFu;
    buf[len++] = (char)0xFFu;
    return Tjc_SendFrame(buf, len);
}

uint8_t Tjc_SetVal(const char *var, int32_t num) {
    char     buf[TJC_BUF_VAL];
    char     numstr[12];
    uint16_t len = 0u;

    (void)Tjc_Itoa(num, numstr);
    len        = Tjc_Put(buf, (uint16_t)(sizeof(buf) - 3u), len, var);
    len        = Tjc_Put(buf, (uint16_t)(sizeof(buf) - 3u), len, ".val=");
    len        = Tjc_Put(buf, (uint16_t)(sizeof(buf) - 3u), len, numstr);
    buf[len++] = (char)0xFFu;
    buf[len++] = (char)0xFFu;
    buf[len++] = (char)0xFFu;
    return Tjc_SendFrame(buf, len);
}

uint8_t Tjc_SetPic(const char *var, int32_t pic_id) {
    char     buf[TJC_BUF_VAL];
    char     numstr[12];
    uint16_t len = 0u;

    (void)Tjc_Itoa(pic_id, numstr);
    len        = Tjc_Put(buf, (uint16_t)(sizeof(buf) - 3u), len, var);
    len        = Tjc_Put(buf, (uint16_t)(sizeof(buf) - 3u), len, ".pic=");
    len        = Tjc_Put(buf, (uint16_t)(sizeof(buf) - 3u), len, numstr);
    buf[len++] = (char)0xFFu;
    buf[len++] = (char)0xFFu;
    buf[len++] = (char)0xFFu;
    return Tjc_SendFrame(buf, len);
}
