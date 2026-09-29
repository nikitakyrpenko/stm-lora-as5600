#pragma once
#include <stdint.h>

#ifdef __cplusplus
#include <cstdio>
#endif

struct Measure {
  uint16_t angle_raw;
  uint16_t status;
  uint32_t timestamp_ms;

#ifdef __cplusplus
  uint8_t to_bytes(uint8_t* out, uint8_t len)
  {
    if (out == nullptr || len != sizeof(Measure)) {
      return 0;
    }
    out[0] = static_cast<uint8_t>(angle_raw);
    out[1] = static_cast<uint8_t>(angle_raw >> 8);
    out[2] = static_cast<uint8_t>(status);
    out[3] = static_cast<uint8_t>(status >> 8);
    out[4] = static_cast<uint8_t>(timestamp_ms);
    out[5] = static_cast<uint8_t>(timestamp_ms >> 8);
    out[6] = static_cast<uint8_t>(timestamp_ms >> 16);
    out[7] = static_cast<uint8_t>(timestamp_ms >> 24);

    return len;
  }

  uint8_t to_string(char* out, uint8_t len)
  {
    if (out == nullptr || len == 0) {
      return 0;
    }
    int written = std::snprintf(out, len, "angle=%u status=%u ts=%lu\r\n", angle_raw, status, static_cast<unsigned long>(timestamp_ms));

    if (written < 0) {
      return 0;
    }
    return static_cast<uint8_t>(written < len ? written : len - 1);
  }
#endif
};