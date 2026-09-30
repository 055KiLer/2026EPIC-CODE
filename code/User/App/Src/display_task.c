/**
 * @file    display_task.c
 * @brief   显示任务：把"每种货物各成功抓取多少件"逐行写进屏上的静态统计表
 * @note    屏工程是 6 行固定表（序号/图片/货物名称三列由屏工程画死，**行序 = 类别 ID 1~6**），
 *          主控只写每行末尾的"成功数量"（控件名见 sort_config.h 的 CFG_TJC_VAR_CNT_BASE）。
 *          类别号就是行号，没有中间映射表——2026-09-30 起上位机只报一个类别数字 1~6，
 *          越界的帧在 vision_task 里已被当"无目标"丢掉，所以这里拿到的行号必定合法。
 *          ★ 屏上是**每类单独计数**，没有总数。每件都把 **6 行全部重写一遍**（写绝对值，
 *          不是 val++）：屏端没有回执，一条指令在屏侧静默丢失是主控检不出来的——只写"这一件
 *          那一行"的话，丢了就永远少一件（显示错误 = 0 分）；全量重写让下一次刷新把所有行
 *          自动补齐。代价 6 条 ≈7ms（115200），相对一件十几秒可忽略。
 *          计数放在本任务：屏上的数只有这一处真源，sort_task 只管发"第几类又成功一件"。
 */
#include "display_task.h"
#include "tjc_screen.h"
#include "cmsis_os2.h"
#include <FreeRTOS.h>
#include <task.h>

static uint16_t s_count[CFG_NUM_CATEGORIES]; /* 每类成功件数（屏上那一列的真值） */
static uint16_t s_last_seq = 0u;             /* 已计入的最大序号（去重凭据） */

static osMessageQueueId_t s_queue    = NULL;
static volatile uint16_t  s_done_seq = 0u; /* 已刷完的最大序号（display 写 / sort 读） */

/* 行号（= 类别 ID）-> 控件名："n" + 数字(CFG_TJC_CNT_FIRST + 行号 - 1)，如 n1~n6。
   长度上界由 tjc_screen.c 的编译期断言锁死（基名长度 + 3 位数字 + NUL ≤ CFG_TJC_NAME_MAX），
   这里仍按 cap 截断，防以后宏改乱了直接踩栈 */
static void Display_CountVarName(uint8_t row, char *dst, uint8_t cap) {
    const char *base = CFG_TJC_VAR_CNT_BASE;
    uint32_t    val  = (uint32_t)CFG_TJC_CNT_FIRST + (uint32_t)(row - 1u);
    char        num[4];
    uint8_t     len = 0u;
    uint8_t     n   = 0u;

    while ((base[len] != '\0') && ((uint8_t)(len + 1u) < cap)) {
        dst[len] = base[len];
        len++;
    }
    do { /* 十进制：先低位后高位，再倒着贴 */
        num[n++] = (char)('0' + (val % 10u));
        val /= 10u;
    } while ((val > 0u) && (n < (uint8_t)sizeof(num)));
    while ((n > 0u) && ((uint8_t)(len + 1u) < cap)) {
        dst[len++] = num[--n];
    }
    dst[len] = '\0';
}

/* 写一行的"成功数量"（只动数字列；该行的名称/图片是屏工程画死的，主控不碰）
   @retval 1=整条指令已交给 USART6  0=没发出去 */
static uint8_t Display_SendCount(uint8_t row, uint16_t count) {
    char var[CFG_TJC_NAME_MAX + 1u];

    Display_CountVarName(row, var, (uint8_t)sizeof(var));
    return Tjc_SetVal(var, (int32_t)count);
}

void DisplayMgr_CreateObjects(void) {
    s_queue = osMessageQueueNew(4u, sizeof(Display_Msg_t), NULL);
}

void DisplayTask(void *argument) {
    (void)argument;

    Tjc_Init();
    /* 上电把 6 行清零：屏可能没跟主控一起断电（屏上留着上一轮的数），
       不主动清就会出现"主控从 0 重新计数、屏上还是旧值"的错配。
       CFG_TJC_SELFTEST_ON_BOOT=1 时改成写行号（1..6）：清零看不出问题，写行号能一眼核
       "控件名 + 行序"（见 sort_config.h 的说明），第一件投放后的全量重写会覆盖掉 */
    for (uint8_t row = 1u; row <= (uint8_t)CFG_NUM_CATEGORIES; row++) {
#if CFG_TJC_SELFTEST_ON_BOOT
        (void)Display_SendCount(row, (uint16_t)row);
#else
        (void)Display_SendCount(row, 0u);
#endif
    }

    for (;;) {
        Display_Msg_t msg;

        if (osMessageQueueGet(s_queue, &msg, NULL, osWaitForever) == osOK) {
            uint8_t row = msg.category; /* 类别号就是行号，无中间映射 */
            uint8_t ok  = 1u;

            /* 序号只增不减：更大 = 新的一件（本类计数 +1）；相同 = 上一件的补投（只重发） */
            if ((int16_t)(msg.seq - s_last_seq) > 0) {
                s_last_seq = msg.seq;
                s_count[row - 1u]++;
            }

            /* 6 行全量重写（绝对值 & 不短路：哪一行没发出去都要知道，好让 sort 侧重投） */
            for (uint8_t r = 1u; r <= (uint8_t)CFG_NUM_CATEGORIES; r++) {
                ok &= Display_SendCount(r, s_count[r - 1u]);
            }
            if (ok) {
                s_done_seq = msg.seq;
            }
        }
    }
}

uint8_t Display_Show(const Display_Msg_t *msg) {
    if (msg == NULL || s_queue == NULL) {
        return 0u;
    }
    return (osMessageQueuePut(s_queue, msg, 0u, 0u) == osOK) ? 1u : 0u;
}

uint8_t Display_WaitDone(uint16_t seq, uint32_t timeout_ms) {
    uint32_t t0 = HAL_GetTick();

    /* 用有符号差判断，序号回绕仍然正确；已经冲过去的旧序号直接放行，不会永久卡死 */
    while ((int16_t)(s_done_seq - seq) < 0) {
        if ((HAL_GetTick() - t0) >= timeout_ms) {
            return 0u;
        }
        vTaskDelay(pdMS_TO_TICKS(2u));
    }
    return 1u;
}
