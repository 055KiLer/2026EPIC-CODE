/* 主机侧测试用的替身：顶掉真机的 Config/dbg_config.h（真机那份要 main.h/usart.h，
   host 上编译不了）。**只放主机测试用得上的宏**，值必须与真机一致 */
#ifndef __DBG_CONFIG_H
#define __DBG_CONFIG_H

#include <stdint.h>
#include <stddef.h>

/* dbg_print.c 用 */
#define DBG_CFG_PRINT_MIRROR_ITM 0

/* zdt_bus.c 用。2026-09-30 定调全广播：三台一律 0x00（一条线一台，无需 ID） */
#define DBG_CFG_ADDR_BROADCAST    0x00u
#define DBG_CFG_POS_MODE_RELATIVE 0x00u
#define DBG_CFG_ACC16_RPM_PS      511u

/* 2026-09-30 起调试工程与产品接线一致（每台一路 UART：A=USART3、B=UART5）；
   帧层只经 Dbg_Uart_SendMotor(motor, ...) 出帧，host 测试在测试文件里给替身抓帧 */

#endif
