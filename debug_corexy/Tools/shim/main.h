/* 主机侧测试用的替身：真机上是 CubeMX 的 Core/Inc/main.h */
#ifndef __MAIN_H
#define __MAIN_H
#include <stdint.h>
#include <stddef.h>

/* Config/sort_config.h（产品那份的原样副本）末尾有编译期引脚断言（SORT_CFG_ASSERT），
   主机侧编译也要过得去——这里的值照抄真机 main.h/CubeMX 口径，断言在主机上同样生效 */
#define GPIO_PIN_0        ((uint16_t)0x0001)
#define GPIO_PIN_1        ((uint16_t)0x0002)
#define GPIO_PIN_5        ((uint16_t)0x0020)
#define GPIO_PIN_RESET    ((uint8_t)0x00)

#define TRAY_ORIGIN_Pin   GPIO_PIN_0
#define X_LIMIT_Pin       GPIO_PIN_1
#define Y_LIMIT_Pin       GPIO_PIN_5

#endif
