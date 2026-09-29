#pragma once

#include "Measure.hpp"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"

#include <cstdint>

class As5600Device {
  static constexpr uint32_t I2C_TIMEOUT_MS = 10;

public:
  struct Status {
    bool magnet_detected;    /*MD*/
    bool magnet_too_weak;    /*ML: AGC maximum gain overflow*/
    bool magnet_too_strong;  /*MH: AGC minimum gain overflow*/
    uint8_t raw;

    bool is_ok() const;
  };

  explicit As5600Device(I2C_HandleTypeDef* hi2c);

  bool get_status(Status* out);
  bool get_raw_angle(uint16_t* out);
  bool poll(Measure* out);

private:
  I2C_HandleTypeDef* hi2c_;

  bool read_register(uint8_t reg, uint8_t* out, uint16_t out_len);
};
