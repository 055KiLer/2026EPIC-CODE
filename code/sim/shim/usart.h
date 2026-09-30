/* 仿真替身：替代 Core/Inc/usart.h（只提供 6 个句柄声明） */
#ifndef __USART_H
#define __USART_H
#include "../sim_shim.h"
extern UART_HandleTypeDef huart1, huart2, huart3, huart4, huart5, huart6;
#endif
