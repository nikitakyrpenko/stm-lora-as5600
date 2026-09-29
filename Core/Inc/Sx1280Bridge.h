#pragma once

#include "Measure.hpp"
#include "stm32l4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void Sx1280_Init(SPI_HandleTypeDef* hspi,
                 GPIO_TypeDef* nssPort,
                 GPIO_TypeDef* busy_port,
                 GPIO_TypeDef* nreset_port,
                 uint16_t nssPin,
                 uint16_t busy_pin,
                 uint16_t nreset_pin);

void Sx1280_Send(struct Measure* measure);
void Sx1280_OnDio1Irq();
void Sx1280_SetFrequency(uint32_t freq_hz);

#ifdef __cplusplus
}
#endif
