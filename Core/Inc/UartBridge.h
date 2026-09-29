#pragma once

#include "Measure.hpp"
#include "stm32l4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void Uart_Init(UART_HandleTypeDef* huart);
void Uart_SendMeasure(struct Measure* measure);

void Uart_StartRx();
void Uart_PollCommand();

#ifdef __cplusplus
}
#endif
