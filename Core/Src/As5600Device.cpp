#include "As5600Device.hpp"

#include "stm32l4xx_hal_def.h"

namespace {
constexpr uint8_t AS5600_I2C_ADDR_7BIT = 0x36;
constexpr uint16_t AS5600_I2C_ADDR = static_cast<uint16_t>(AS5600_I2C_ADDR_7BIT << 1);

constexpr uint8_t REG_STATUS = 0x0B;
constexpr uint8_t REG_RAW_ANGLE_HI = 0x0C;  // RAW ANGLE(11:8); 0x0D holds RAW ANGLE(7:0) right after

constexpr uint8_t STATUS_MD = 0x20;
constexpr uint8_t STATUS_ML = 0x10;
constexpr uint8_t STATUS_MH = 0x08;
}  // namespace

bool As5600Device::Status::is_ok() const
{
  return magnet_detected && !magnet_too_weak && !magnet_too_strong;
}

As5600Device::As5600Device(I2C_HandleTypeDef* hi2c) : hi2c_(hi2c) {}

bool As5600Device::read_register(uint8_t reg, uint8_t* out, uint16_t out_len)
{
  return HAL_I2C_Mem_Read(hi2c_, AS5600_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, out, out_len, I2C_TIMEOUT_MS) == HAL_OK;
}

bool As5600Device::get_status(Status* out)
{
  uint8_t raw = 0;
  if (!read_register(REG_STATUS, &raw, 1)) {
    return false;
  }

  out->raw = raw;
  out->magnet_detected = (raw & STATUS_MD) != 0;
  out->magnet_too_weak = (raw & STATUS_ML) != 0;
  out->magnet_too_strong = (raw & STATUS_MH) != 0;
  return true;
}

bool As5600Device::get_raw_angle(uint16_t* out)
{
  uint8_t buf[2] = {0};
  if (!read_register(REG_RAW_ANGLE_HI, buf, sizeof(buf))) {
    return false;
  }

  *out = (static_cast<uint16_t>(buf[0] & 0x0F) << 8) | buf[1];
  return true;
}

bool As5600Device::poll(Measure* out)
{
  Status status{};
  uint16_t angle_raw = 0;

  if (!get_status(&status) || !get_raw_angle(&angle_raw)) {
    return false;
  }

  out->angle_raw = angle_raw;
  out->status = status.raw;
  out->timestamp_ms = HAL_GetTick();
  return true;
}
