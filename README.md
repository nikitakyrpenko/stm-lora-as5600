# stm-lora-as5600

STM32 firmware for a wireless angle-sensor link: reads a magnetic angle sensor over I2C and transmits it over LoRa, with a UART command interface for runtime configuration.

## Hardware

- **MCU**: STM32 Nucleo-L476RG (STM32L476RGT3, bare-metal, CubeMX + HAL, C with a C++ driver layer)
- **Radio**: NiceRF LoRa1280-TCXO module (Semtech SX1280/SX1281, 2.4GHz), connected over `SPI2`
- **Angle sensor**: AS5600 magnetic rotary position sensor, connected over `I2C1`
- **Debug/command link**: USART2 over the Nucleo's ST-Link Virtual COM Port (VCP), 115200 8N1

## What it does

- Polls the AS5600 for the current angle and timestamps it into a `Measure` struct.
- Sends each `Measure` over the air via the SX1280 in LoRa mode (paired with a matching receiver, e.g. an ESP32-based raw-SPI RX).
- Prints each `Measure` out over UART for debugging (`screen /dev/ttyACM0 115200`).
- Accepts a `FREQ <hz>` text command over the same UART (DMA + IDLE-line RX) to change the SX1280's RF frequency at runtime, replying with `freq=<hz>` once applied — useful for testing different channels without reflashing. Both radios must be retuned to match, or the link breaks.

See `SX1280_DRIVER_PLAN.md` for the SX1280 driver's opcode/register reference.

## Interrupt-driven design

Three independent interrupt sources drive the firmware, each following the same pattern: the ISR only sets a flag (or stashes a length), and the actual work happens back in `main()`'s `while(1)` — keeping SPI/I2C transactions and command parsing out of interrupt context.

- **`DIO1` (EXTI1, `PA1`, rising-edge)** — the SX1280's `TxDone` interrupt, routed onto the physical `DIO1` pin via `SetDioIrqParams`. Fires once a LoRa transmission finishes; the handler just sets `DIO1_callback_triggered`, and the main loop then calls `Sx1280_OnDio1Irq()` to clear the chip's IRQ register and mark the radio free for the next send.
- **`TIM6` periodic interrupt** — free-running timebase (80MHz APB1 → /8000 prescaler → 10kHz tick, 5000-tick period = **500ms**), sets `TIM6_callback_triggered`. The main loop uses it to pace the whole read → transmit → debug-print cycle: poll the AS5600, send the `Measure` over LoRa, and print it over UART.
- **`USART2` RX via DMA (`DMA1_Channel6`, Normal mode, Byte-aligned) + IDLE-line interrupt** — `HAL_UARTEx_ReceiveToIdle_DMA` fills a 32-byte buffer autonomously; the UART's own IDLE interrupt (needs `USART2` global interrupt enabled in NVIC) fires `HAL_UARTEx_RxEventCallback` once the line goes quiet, even if the buffer isn't full — needed since commands like `FREQ <hz>` are shorter than the buffer and variable-length. The callback just records the byte count, and `Uart_PollCommand()` in the main loop parses and re-arms the DMA for the next command.

## Build and flash

Open the folder in the devcontainer (it passes through `/dev/ttyACM0` and the ST-Link USB bus). The CMake project and firmware binary are still named `homework_18`, so the output is `build/Debug/homework_18.elf`.

    cmake --preset Debug
    cmake --build --preset Debug

VS Code tasks: `STM32: build` (default), `STM32: flash` (objcopy to `.bin`, then `st-flash`), and a `Debug STM32L476 (st-util)` launch config over the ST-Link GDB server.
