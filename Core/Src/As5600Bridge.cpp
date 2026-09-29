#include "As5600Bridge.h"

#include "As5600Device.hpp"

namespace {
As5600Device* AS5600 = nullptr;
}  // namespace

void As5600_Init(I2C_HandleTypeDef* hi2c)
{
  static As5600Device device(hi2c);
  AS5600 = &device;
}

void As5600_Poll(Measure* measure)
{
  if (AS5600 == nullptr) {
    return;
  }

  AS5600->poll(measure);
}
