#include <stdint.h>

#include "Sx1280Bridge.h"
#include "Measure.hpp"
#include "Sx1280Device.hpp"
#include "main.h"
#include "stm32l4xx_nucleo.h"

namespace {
Sx1280Device* LORA_1280 = nullptr;

volatile bool Sx1280Device_initizalized_success = false;
volatile bool Sx1280Device_tx_done = true;

constexpr uint32_t TCXO_WARMUP_MS = 5;
}  // namespace

void Sx1280_Init(SPI_HandleTypeDef* hspi,
                 GPIO_TypeDef* nss_port,
                 GPIO_TypeDef* busy_port,
                 GPIO_TypeDef* nreset_port,
                 uint16_t nss_pin,
                 uint16_t busy_pin,
                 uint16_t nreset_pin)
{
  HAL_GPIO_WritePin(TCXOEN_GPIO_Port, TCXOEN_Pin, GPIO_PIN_SET);
  HAL_Delay(TCXO_WARMUP_MS);

  static Sx1280Device device(hspi, nss_port, busy_port, nreset_port, nss_pin, busy_pin, nreset_pin);
  LORA_1280 = &device;
  LORA_1280->reset();
  Sx1280Device_initizalized_success = LORA_1280->init();

  if (!Sx1280Device_initizalized_success) {
    return;
  }

  if (LORA_1280->get_status().is_ok()) {
    BSP_LED_On(LED_GREEN);
  }
}

void Sx1280_Send(Measure* measure)
{
  if (LORA_1280 == nullptr) {
    return;
  }
  if (!Sx1280Device_initizalized_success) {
    return;
  }
  if (!Sx1280Device_tx_done) {
    return;
  }

  uint8_t out[sizeof(Measure)]{};
  uint8_t written = measure->to_bytes(out, sizeof(Measure));

  if (LORA_1280->write_buffer(
        out, written, [](Sx1280Device::Status s) -> bool { return s.status == Sx1280Device::CommandStatus::RESERVED; })) {
    LORA_1280->clear_irq();
    if (LORA_1280->set_tx()) {
      Sx1280Device_tx_done = false;
    }
  }
}

void Sx1280_SetFrequency(uint32_t freq_hz)
{
  if (LORA_1280 == nullptr) {
    return;
  }
  if (!Sx1280Device_initizalized_success) {
    return;
  }

  LORA_1280->set_frequency(freq_hz);
}

void Sx1280_OnDio1Irq()
{
  if (LORA_1280 == nullptr) {
    return;
  }
  if (!Sx1280Device_initizalized_success) {
    return;
  }

  LORA_1280->clear_irq();
  Sx1280Device_tx_done = true;
}
