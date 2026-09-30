/**
 * @file    dbg_print.c
 * @brief   调试打印：行缓冲 + 控制台输出（ITM 镜像可选）
 */
#include "dbg_print.h"
#include "dbg_config.h"
#include "dbg_uart.h"
#include <stdarg.h>

static char     s_line[DBG_PRINT_LINE_MAX];
static uint16_t s_len;

#if ((DBG_CFG_PRINT_MIRROR_ITM) != 0) && defined(STM32F407xx)
#define PRINT_USE_ITM 1
#else
#define PRINT_USE_ITM 0
#endif

static void Dbg_Print_Emit(const char *buf, uint16_t len) {
    if (len == 0u) {
        return;
    }

    (void)Dbg_Uart_SendConsole((const uint8_t *)buf, len);

#if PRINT_USE_ITM
    /* 未接调试器 / 未开 trace 时 ITM_SendChar 自己会丢弃，不会阻塞 */
    for (uint16_t i = 0u; i < len; i++) {
        ITM_SendChar((uint32_t)(uint8_t)buf[i]);
    }
#endif
}

void Dbg_Print_Init(void) {
    s_len = 0u;
}

void Dbg_Print_Flush(void) {
    if (s_len != 0u) {
        Dbg_Print_Emit(s_line, s_len);
        s_len = 0u;
    }
}

static void Dbg_Print_PutChar(char c) {
    if (s_len >= (DBG_PRINT_LINE_MAX - 1u)) {
        Dbg_Print_Flush(); /* 满则先冲再写：宁可多切一行，也不丢字 */
    }

    s_line[s_len] = c;
    s_len++;

    if (c == '\n') {
        Dbg_Print_Flush();
    }
}

static void Dbg_Print_PutStr(const char *str) {
    if (str == NULL) {
        Dbg_Print_PutStr("(null)");
        return;
    }

    while (*str != '\0') {
        Dbg_Print_PutChar(*str);
        str++;
    }
}

static uint16_t Dbg_Print_StrLen(const char *str) {
    uint16_t len = 0u;

    if (str == NULL) {
        return 10u; /* "(null)" */
    }
    while (str[len] != '\0') {
        len++;
    }
    return len;
}

/* %s 的位宽：left=1 左对齐补右空格，否则右对齐补左空格 */
static void Dbg_Print_PutPadded(const char *str, uint8_t width, uint8_t left) {
    uint16_t len = Dbg_Print_StrLen(str);

    if (left != 0u) {
        Dbg_Print_PutStr(str);
        for (uint16_t i = len; i < width; i++) {
            Dbg_Print_PutChar(' ');
        }
        return;
    }

    for (uint16_t i = len; i < width; i++) {
        Dbg_Print_PutChar(' ');
    }
    Dbg_Print_PutStr(str);
}

/* width 为最小位宽，不足补 fill（'0' 就是 %02x 那种） */
static void Dbg_Print_PutU32(uint32_t value, uint32_t base, uint8_t upper, uint8_t width, char fill) {
    const char *digits = upper ? "0123456789ABCDEF" : "0123456789abcdef";
    char        tmp[11];
    uint8_t     n = 0u;

    do {
        tmp[n] = digits[value % base];
        n++;
        value /= base;
    } while (value != 0u);

    while (n < width) {
        tmp[n] = fill;
        n++;
    }

    while (n > 0u) {
        n--;
        Dbg_Print_PutChar(tmp[n]);
    }
}

static void Dbg_Print_PutI32(int32_t value, uint8_t width, char fill) {
    uint32_t magnitude = (uint32_t)value;

    if (value < 0) {
        Dbg_Print_PutChar('-');
        magnitude = 0u - magnitude; /* 二进制补码取负，避开 -INT32_MIN 的溢出 */
    }

    Dbg_Print_PutU32(magnitude, 10u, 0u, width, fill);
}

void Dbg_Printf(const char *fmt, ...) {
    va_list ap;

    if (fmt == NULL) {
        return;
    }

    va_start(ap, fmt);

    for (const char *p = fmt; *p != '\0'; p++) {
        char    fill  = ' ';
        uint8_t width = 0u;
        uint8_t left  = 0u;
        uint8_t plus  = 0u;

        if (*p != '%') {
            Dbg_Print_PutChar(*p);
            continue;
        }

        p++;
        if (*p == '%') {
            Dbg_Print_PutChar('%');
            continue;
        }

        /* 标志位（可叠加、顺序不限）。★ 必须每个都吃掉并置标志：
           漏掉一个不认识的标志时，它会被当成说明符、打一个 '?' 却**不消费参数**，
           于是后面所有参数整体错位一位（实测踩过：打印出"各 20 脉冲 @1600rpm"，
           而真实值是"各 1600 脉冲 @30rpm"） */
        for (;;) {
            if (*p == '-') { /* 左对齐（只对 %s 有意义；数字仍右对齐） */
                left = 1u;
                p++;
            } else if (*p == '+') { /* 正数也带符号 */
                plus = 1u;
                p++;
            } else if (*p == '0') {
                fill = '0';
                p++;
            } else {
                break;
            }
        }
        while (*p >= '0' && *p <= '9') {
            width = (uint8_t)((width * 10u) + (uint8_t)(*p - '0'));
            p++;
        }
        if (*p == 'l' || *p == 'L') {
            p++; /* 两个目标上 long 都是 32 位，直接吃掉长度修饰符 */
        }

        switch (*p) {
            case 'u':
                Dbg_Print_PutU32((uint32_t)va_arg(ap, unsigned int), 10u, 0u, width, fill);
                break;
            case 'd': {
                int32_t value = (int32_t)va_arg(ap, int);

                if ((plus != 0u) && (value >= 0)) {
                    Dbg_Print_PutChar('+');
                }
                Dbg_Print_PutI32(value, width, fill);
                break;
            }
            case 'x':
                Dbg_Print_PutU32((uint32_t)va_arg(ap, unsigned int), 16u, 0u, width, fill);
                break;
            case 'X':
                Dbg_Print_PutU32((uint32_t)va_arg(ap, unsigned int), 16u, 1u, width, fill);
                break;
            case 'c':
                Dbg_Print_PutChar((char)va_arg(ap, int));
                break;
            case 's':
                Dbg_Print_PutPadded(va_arg(ap, const char *), width, left);
                break;
            case '\0':
                p--; /* 末尾裸 '%'：退一格交给循环条件收尾，免得越界读 */
                break;
            default:
                /* 不支持的说明符（含 %f）打问号而非静默吃掉，免得对着输出猜 */
                Dbg_Print_PutChar('?');
                break;
        }
    }

    va_end(ap);
}