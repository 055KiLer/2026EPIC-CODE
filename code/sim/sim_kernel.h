/* 内核对外接口（供 plant / scenarios 使用） */
#ifndef __SIM_KERNEL_H
#define __SIM_KERNEL_H
#include "sim_shim.h"

#include <stdint.h>
void sim_raise_exti(uint16_t pin);
void sim_uart_byte_arrives(int uart_id);
int  sim_uart_rx_attach(UART_HandleTypeDef *h, uint8_t byte);
UART_HandleTypeDef *sim_uart_handle(int id);
void sim_run_until(uint64_t until_us);
uint64_t sim_now_us_get(void);
void sim_mq_wake_all(void);
int  sim_boot(void);
extern int sim_error_raised;
const char *sim_task_name(int i);
int sim_task_state(int i);
uint32_t sim_task_switches(int i);

/* 半截帧回归注入（s19）：after_ms 之后屏的第 nth 次发送调用（0 起）字节静默丢失 */
void sim_screen_arm_drop(uint32_t after_ms, uint32_t nth);
uint32_t sim_screen_drops(void);
int sim_screen_drop_line(void);
#endif
