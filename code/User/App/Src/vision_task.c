/**
 * @file    vision_task.c
 * @brief   视觉任务：上位机串口（ROCK 5C）-> 纯解析器 -> 带可信度标记的最新目标缓存
 * @note    三个判据各自独立，互不代替：
 *            有可用目标 = 缓存有效 且 未消费 且 未过期
 *            链路活着   = 最近 CFG_K230_LINK_DEAD_MS 内有校验通过的帧（上位机停发 = 链路故障，
 *                         绝不等同于"托盘已空"，所以上位机无目标时也必须照周期发状态 0x00 帧）
 *            托盘已空   = 链路活着 且 连续 M 帧无目标（只影响轮询间隔，不是终止态）
 *          缓存整体在临界区内读写，分拣任务取到的必是"同一帧的数据 + 时间戳 + 标志"
 */
#include "vision_task.h"
#include "bsp_uart.h"
#include "sort_config.h"
#include <FreeRTOS.h>
#include <task.h>

#define VISION_POLL_PERIOD_MS 10u

typedef struct {
    K230_Target_t target;      /* 最近一帧有效目标 */
    uint32_t      target_tick; /* 到达时刻（新鲜度判定） */
    uint32_t      frame_tick;  /* 最近一次校验通过帧的时刻（链路活性） */
    uint32_t      none_cnt;    /* 连续"无目标"帧数 */
    uint8_t       consumed;    /* 已被分拣任务领走 */
} Vision_Cache_t;

static Vision_Cache_t s_cache;

static void Vision_UpdateCache(K230_Event_e event, const K230_Target_t *frame) {
    uint32_t now = HAL_GetTick();

    /* 类别 ID 越界、或形状保留字节非 0 = 视觉与主控的口径不一致（典型：旧"颜色+形状"编码
       没换），此时既没有正确的类别也没有正确的盒号，按"本帧无目标"处理：
       宁可少抓（表现为"链路活着但一直无目标"），绝不用取模硬套一个错误类别静默放错盒 */
    uint8_t usable = (event == K230_EV_TARGET) && (frame->category >= 1u) &&
                     (frame->category <= (uint8_t)CFG_NUM_CATEGORIES) && (frame->shape == 0u);

    taskENTER_CRITICAL();
    s_cache.frame_tick = now;

    if (usable) {
        s_cache.target      = *frame;
        s_cache.target_tick = now;
        s_cache.consumed    = 0u;
        s_cache.none_cnt    = 0u;
    } else {
        s_cache.target.valid = 0u;
        s_cache.none_cnt++;
    }
    taskEXIT_CRITICAL();
}

void VisionTask(void *argument) {
    (void)argument;

    for (;;) {
        while (Bsp_Uart_Available((UART_ID_e)CFG_UART_K230) > 0u) {
            uint8_t       byte = Bsp_Uart_ReadByte((UART_ID_e)CFG_UART_K230);
            K230_Target_t frame;
            K230_Event_e  event = K230_FeedByte(byte, &frame);

            if (event != K230_EV_NONE) {
                Vision_UpdateCache(event, &frame);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(VISION_POLL_PERIOD_MS));
    }
}

uint8_t Vision_GetTarget(K230_Target_t *out) {
    if (out == NULL) {
        return 0u;
    }

    taskENTER_CRITICAL();
    uint8_t fresh = (HAL_GetTick() - s_cache.target_tick) <= CFG_K230_TARGET_FRESH_MS;

    if (!s_cache.target.valid || s_cache.consumed || !fresh) {
        taskEXIT_CRITICAL();
        return 0u;
    }
    *out             = s_cache.target;
    s_cache.consumed = 1u; /* 领走即销账：一轮分拣只认一帧 */
    taskEXIT_CRITICAL();
    return 1u;
}

uint8_t Vision_LinkAlive(void) {
    taskENTER_CRITICAL();
    uint8_t alive = (HAL_GetTick() - s_cache.frame_tick) <= CFG_K230_LINK_DEAD_MS;
    taskEXIT_CRITICAL();
    return alive;
}

uint8_t Vision_TrayEmpty(void) {
    taskENTER_CRITICAL();
    uint8_t alive = (HAL_GetTick() - s_cache.frame_tick) <= CFG_K230_LINK_DEAD_MS;
    uint8_t empty = (s_cache.none_cnt >= (uint32_t)CFG_K230_NO_TARGET_FRAMES);
    taskEXIT_CRITICAL();
    return (alive && empty) ? 1u : 0u;
}

void Vision_Flush(void) {
    Bsp_Uart_Flush((UART_ID_e)CFG_UART_K230);

    taskENTER_CRITICAL();
    s_cache.target.valid = 0u; /* 旧帧不带入主循环 */
    s_cache.consumed     = 0u;
    s_cache.none_cnt     = 0u;
    s_cache.frame_tick   = HAL_GetTick(); /* 从这里才开始要求 K230 有流量 */
    s_cache.target_tick  = HAL_GetTick();
    taskEXIT_CRITICAL();
}

void Vision_ToMachine(const K230_Target_t *target, float *x_mm, float *y_mm) {
    /* 2026-09-30 视觉组口径：视觉系 x+ = 机械 Y+、视觉系 y+ = 机械 X+（两轴交换），
       视觉原点 = 标定中心（托盘上距左边 87.5mm、距底部 82.5mm）。四个常量见 sort_config.h */
    if (x_mm != NULL) {
        *x_mm = CFG_VISION_ORIGIN_X_MM + CFG_VISION_SIGN_VY_TO_X * target->y_mm;
    }
    if (y_mm != NULL) {
        *y_mm = CFG_VISION_ORIGIN_Y_MM + CFG_VISION_SIGN_VX_TO_Y * target->x_mm;
    }
}