/**
 * @file    k230_uart.c
 * @brief   视觉帧解析状态机（12 字节定长帧，见电控文档 §4.1）
 * @note    对端 = 视觉上位机 ROCK 5C（`k230_*` 是历史命名，协议与型号无关）
 *          坏帧直接丢弃并回到帧头搜索；连续 0xAA 可正确重同步
 */
#include "k230_uart.h"

#define K230_FRAME_LEN     12u
#define K230_ST_HAS_TARGET 0x01u
#define K230_ST_NO_TARGET  0x00u

typedef enum {
    K230_SYNC_AA = 0,
    K230_SYNC_55,
    K230_PAYLOAD
} K230_ParserState_e;

static K230_ParserState_e s_state = K230_SYNC_AA;
static uint8_t            s_buf[K230_FRAME_LEN];
static uint8_t            s_idx = 0u;

K230_Event_e K230_FeedByte(uint8_t byte, K230_Target_t *frame_out) {
    K230_Event_e event = K230_EV_NONE;

    switch (s_state) {
        case K230_SYNC_AA:
            if (byte == 0xAAu) {
                s_state = K230_SYNC_55;
            }
            break;

        case K230_SYNC_55:
            if (byte == 0x55u) {
                s_buf[0] = 0xAAu;
                s_buf[1] = 0x55u;
                s_idx    = 2u;
                s_state  = K230_PAYLOAD;
            } else if (byte == 0xAAu) {
                s_state = K230_SYNC_55; /* 连续 AA：留在本状态重判 */
            } else {
                s_state = K230_SYNC_AA;
            }
            break;

        case K230_PAYLOAD:
        default:
            s_buf[s_idx++] = byte;
            if (s_idx >= K230_FRAME_LEN) {
                uint8_t sum = 0u;

                for (uint8_t i = 0; i < (K230_FRAME_LEN - 1u); i++) {
                    sum += s_buf[i];
                }

                if (sum == s_buf[K230_FRAME_LEN - 1u]) {
                    if (s_buf[2] == K230_ST_HAS_TARGET) {
                        /* frame_out 为 NULL 只表示"不要数据"，绝不能改变事件本身：
                           折进 EMPTY 会让调用方把"有目标"读成"托盘已空"，提前收工 */
                        if (frame_out != NULL) {
                            int16_t x10 = (int16_t)((uint16_t)s_buf[8] << 8 | s_buf[7]);
                            int16_t y10 = (int16_t)((uint16_t)s_buf[10] << 8 | s_buf[9]);

                            frame_out->valid    = 1u;
                            frame_out->category = s_buf[3]; /* 类别 ID 1~6（原颜色字节） */
                            frame_out->shape    = s_buf[4]; /* 形状 0~3：0=默认方案、1~3=特调（不参与分类） */
                            frame_out->has_qr   = s_buf[5];
                            frame_out->x_mm     = (float)x10 * 0.1f;
                            frame_out->y_mm     = (float)y10 * 0.1f;
                        }
                        event = K230_EV_TARGET;
                    } else {
                        if (frame_out != NULL) {
                            frame_out->valid = 0u; /* 清有效位，否则复用结构体会读到旧坐标 */
                        }
                        event = K230_EV_EMPTY;
                    }
                }
                s_state = K230_SYNC_AA; /* 无论成败都重新找帧头 */
            }
            break;
    }
    return event;
}