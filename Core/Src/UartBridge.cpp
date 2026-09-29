#include "UartBridge.h"

#include <cstdio>
#include <cstring>

#include "Sx1280Bridge.h"

namespace {
UART_HandleTypeDef* HUART = nullptr;

constexpr uint32_t UART_TX_TIMEOUT_MS = 100;
constexpr uint16_t RX_BUF_SIZE = 32;

uint8_t rx_buf[RX_BUF_SIZE];
volatile uint16_t rx_len = 0;
volatile bool rx_ready = false;
}  // namespace

void Uart_Init(UART_HandleTypeDef* huart)
{
  HUART = huart;
}

void Uart_StartRx()
{
  if (HUART == nullptr) {
    return;
  }

  HAL_UARTEx_ReceiveToIdle_DMA(HUART, rx_buf, RX_BUF_SIZE);
}

void Uart_SendMeasure(Measure* measure)
{
  if (HUART == nullptr) {
    return;
  }

  char out[64];
  uint8_t written = measure->to_string(out, sizeof(out));

  HAL_UART_Transmit(HUART, reinterpret_cast<uint8_t*>(out), written, UART_TX_TIMEOUT_MS);
}

void Uart_PollCommand()
{
  if (!rx_ready) {
    return;
  }

  char cmd[RX_BUF_SIZE + 1];
  uint16_t len = rx_len < RX_BUF_SIZE ? rx_len : RX_BUF_SIZE;
  memcpy(cmd, rx_buf, len);
  cmd[len] = '\0';

  unsigned long freq_hz = 0;
  if (std::sscanf(cmd, "FREQ %lu", &freq_hz) == 1) {
    Sx1280_SetFrequency(static_cast<uint32_t>(freq_hz));

    char reply[32];
    int written = std::snprintf(reply, sizeof(reply), "freq=%lu\r\n", freq_hz);
    if (written > 0) {
      uint16_t reply_len = static_cast<uint16_t>(written < (int)sizeof(reply) ? written : sizeof(reply) - 1);
      HAL_UART_Transmit(HUART, reinterpret_cast<uint8_t*>(reply), reply_len, UART_TX_TIMEOUT_MS);
    }
  }

  rx_ready = false;
  Uart_StartRx();
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
  if (huart != HUART) {
    return;
  }

  rx_len = Size;
  rx_ready = true;
}
