#pragma once

#include <cstdint>
#include "Measure.hpp"
#include "stm32l476xx.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_gpio.h"
#include "stm32l4xx_hal_spi.h"

class Sx1280Device {
  static constexpr uint8_t BUSY_TIMEOUT_MS = 100;
  static constexpr uint8_t RESET_PULSE_MS = 5;

public:
  enum class CircutMode : uint8_t {
    RESERVED = 0,
    RESERVED_1 = 1,
    STDBY_RC = 2,   /*Idle, clocked by internal 13MHz RC oscillator*/
    STDBY_XOSC = 3, /*Idle, clocked by external crystal/TCXO*/
    FS = 4,         /*PLL locked to configured RF frequency, not yet Tx/Rx*/
    RX = 5,         /*Actively receiving*/
    TX = 6,         /*Actively transmitting*/
  };
  enum class CommandStatus : uint8_t {
    RESERVED,
    PROCCESSED /*The command has been terminated correctly*/,
    DATA_AVAILABLE,    /*A packet has been successfully received and data can be retrieved*/
    COMMAND_TIMEOUT,   /*A transaction from the host took too long to complete and triggered an internal watchdog*/
    COMMAND_ERROR,     /*The transceiver was unable to process command either because of an invalid opcode or because an incorrect number of
    parameters has been   provided*/
    FAILED_TO_EXECUTE, /*The command was successfully processed, however the transceiver could not execute the command;*/
    TX_DONE,           /*The transmission of the current packet has terminated */
  };
  struct Status {
    CircutMode mode;
    CommandStatus status;

    bool is_ok() const;
  };

  static constexpr uint8_t MAX_PAYLOAD_SIZE = 32;
  static constexpr uint8_t PAYLOAD_SIZE = sizeof(Measure);

  static_assert(PAYLOAD_SIZE <= MAX_PAYLOAD_SIZE, "Measure must fit inside MAX_PAYLOAD_SIZE");

  static constexpr uint16_t IRQ_TX_DONE = 0x0001;

  explicit Sx1280Device(SPI_HandleTypeDef* spi,
                        GPIO_TypeDef* NSS_chip,
                        GPIO_TypeDef* BUSY_chip,
                        GPIO_TypeDef* NRESET_chip,
                        uint16_t NSS_pin,
                        uint16_t BUSY_pin,
                        uint16_t NRESET_pin);

  Status get_status();
  bool init();
  void reset();
  bool set_frequency(uint32_t freq_hz);
  bool read_register(uint16_t address, uint8_t* out, uint16_t out_len);
  bool read_buffer(uint8_t offset, uint8_t* out, uint8_t out_len);
  bool write_buffer(const uint8_t* buf, uint8_t len, bool (*verify)(Status) = &check_processed);
  bool clear_irq(bool (*verify)(Status) = &check_processed);
  bool set_tx(bool (*verify)(Status) = &check_processed);
  bool get_irq_status(uint16_t* out_irq_status);

private:
  SPI_HandleTypeDef* hspi_;
  GPIO_TypeDef* NSS_chip_;
  GPIO_TypeDef* BUSY_chip_;
  GPIO_TypeDef* NRESET_chip_;
  uint16_t BUSY_pin_;
  uint16_t NSS_pin_;
  uint16_t NRESET_pin_;

  static bool check_processed(Status s);

  void SPI_NSS_begin();
  void SPI_NSS_end();
  bool BUSY_wait();
  bool send_command(uint8_t opcode, const uint8_t* params, uint16_t params_len, bool (*verify)(Status) = &check_processed);
};
