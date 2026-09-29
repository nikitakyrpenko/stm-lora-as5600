#pragma once

#include "Measure.hpp"
#include "stm32l4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void As5600_Init(I2C_HandleTypeDef* hi2c);
void As5600_Poll(struct Measure* measure);

#ifdef __cplusplus
}
#endif
