# SX1280 LoRa Driver — Init, Periodic TX, TxDone Interrupt

## Context

This project is wired to a NiceRF LoRa1280-TCXO (SX1280-based) module over SPI2, and the wiring/power/reset-idle levels have already been verified working on real hardware: a manual `GetStatus` (opcode `0xC0`) probe in `main.c`'s `while(1)` loop currently blinks `LD2` whenever the chip returns a plausible status byte. That probe was just a wiring sanity check — the goal now is a real driver that (1) runs the SX1280's power-on/config sequence exactly once, (2) transmits an arbitrary payload over LoRa every ~1 second, and (3) detects "transmission done" via a hardware interrupt on the chip's `DIO1` pin (`TxDone`), not polling.

Decisions confirmed: **BLE first, LoRa as a follow-up** (revised from the original LoRa-only decision — switching between them later means re-running `SetPacketType` + `SetModulationParams` + `SetPacketParams` with their respective per-mode columns, not a lightweight tweak; see "Modulation/packet params depend on PacketType" note below), **C++** driver style (matching the `IDevice`-pattern used in the `homework_16` device-driver project). All 11 SPI command opcodes are now datasheet-confirmed (see command table below); still open: `SetModulationParams`/`SetPacketParams` per-field enum values (for BLE now, LoRa later), the IRQ bit-position table, and — specific to BLE — **whether the SX1280 constructs the BLE advertising PDU (access address, header byte, whitening) automatically, or whether a spec-compliant BLE packet must be hand-built before `WriteBuffer`**. This last point is the least-confident piece carried over from earlier discussion and needs explicit datasheet confirmation before `sendTestPacket()` can be written for BLE.
## Prerequisite — before writing driver code

**Get the real command-opcode table from the datasheet PDF** (opcode bytes for `SetStandby`, `SetPacketType`, `SetRfFrequency`, `SetModulationParams`, `SetPacketParams`, `SetBufferBaseAddress`, `WriteBuffer`, `SetTx`, `SetDioIrqParams`, `GetIrqStatus`, `ClearIrqStatus`), plus the `SetDioIrqParams`/`GetIrqStatus` IRQ bitmask table (bit position of `TxDone`), plus the `SetModulationParams`/`SetPacketParams` byte-field layouts for LoRa mode. Do not commit numeric opcodes to `Sx1280Device.cpp` until these are pasted and cross-checked — wrong values cause silent no-ops (chip ignores a malformed command), which is expensive to debug blind on hardware.

## Architecture note: `IDevice` cannot be reused as-is

`homework_16/include/IDevice.hpp` is built entirely around Linux userspace I2C (`int fd`, `ioctl` for device select, `read(2)`/`write(2)` syscalls). None of that exists on bare-metal STM32 HAL firmware. So this plan follows the *spirit* of that pattern (a clear class-based device abstraction) with a new, analogous interface shaped for HAL/SPI instead of Linux I2C — not a literal reuse of `IDevice`.

**C++/C bridge needed**: `main.c` is CubeMX-owned (regenerated as a `.c` file on every `.ioc` regen) and can't itself instantiate a C++ class. So `main.c` stays thin C, calling into a handful of `extern "C"` bridge functions whose implementation lives in the new `.cpp` file and wraps a single `Sx1280Device` instance internally.

## Files to create

- **`Core/Inc/Sx1280Device.hpp`** — class declaration:
  ```cpp
  class Sx1280Device {
  public:
    void init();                    // idempotent: internal bool guard, safe to call every boot
    bool isInitialized() const;
    void sendTestPacket();          // WriteBuffer + SetTx of an arbitrary payload
    void onDio1Irq();               // called from the EXTI callback; reads/clears chip IRQ status
    volatile bool txDoneFlag = false;

  private:
    void waitOnBusy();
    void hardwareReset();
    void writeCommand(uint8_t opcode, const uint8_t* params, uint16_t len);
    void readCommand(uint8_t opcode, uint8_t* out, uint16_t len);
    bool m_initialized = false;
  };

  extern "C" {
    void Sx1280_Init(void);
    void Sx1280_Loop(void);       // called every main-loop iteration; internally rate-limits to 1s
    void Sx1280_OnDio1Irq(void);  // called from HAL_GPIO_EXTI_Callback
  }
  ```
- **`Core/Src/Sx1280Device.cpp`** — implementation, including the bridge functions wrapping a single static `Sx1280Device` instance. Opcode constants (marked `// TODO verify` until the datasheet table is pasted) go at the top of this file or in a small `Sx1280Commands.hpp`.

Reuse from `main.c`: the existing `NSS_GPIO_Port`/`NSS_Pin`, `NReset_*`, `TCXOEN_*`, `Busy_*` macros (from `main.h`) and the global `hspi2` handle — `Sx1280Device.cpp` includes `main.h` for these (ST's HAL headers already wrap themselves in `extern "C"`, so calling HAL functions from `.cpp` is safe with no extra glue).

## `main.c` changes (all inside existing/new `USER CODE` markers — survives CubeMX regen)

1. **Bug fix, unrelated to LoRa but found during review**: `BSP_LED_Init(LED_GREEN)` and `BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI)` currently sit *outside* any `USER CODE` marker (between `USER CODE END 2` and the infinite-loop comment) — CubeMX silently deletes them on next regenerate. Move both inside `USER CODE BEGIN 2` / `END 2`.
2. `USER CODE BEGIN Includes`: add `#include "Sx1280Device.hpp"`.
3. `USER CODE BEGIN 2` (after the BSP init fix above): add `Sx1280_Init();`.
4. `USER CODE BEGIN WHILE`: replace the existing manual `GetStatus` probe with:
   ```c
   while (1)
   {
     Sx1280_Loop();   // internally sends a test packet once per second
   }
   ```
5. `USER CODE BEGIN 4` (new marker CubeMX creates after `main()` for callback overrides): add
   ```c
   void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
   {
     if (GPIO_Pin == DIO1_Pin)
     {
       Sx1280_OnDio1Irq();
     }
   }
   ```
   No dispatch-by-pin complexity needed beyond this one `if` — no other EXTI callback exists in the project yet (the Nucleo user button uses the BSP's own separate callback mechanism).

## `DIO1` / EXTI setup — via CubeMX GUI, same workflow as every other pin so far

1. In CubeMX, assign **`PA1`** as `GPIO_EXTI1` (avoids sharing the already-used `EXTI15_10_IRQHandler` that `PC13`/user-button occupies — a clean, separate handler is simpler). Label it `DIO1`.
2. Rising-edge trigger, pull-down. **Confirmed**: datasheet states the IRQ "will appear at the same time on DIO1" as a rising edge when the corresponding `irqMask`/`dio1Mask` bits are set — active-high confirmed. Drive type (push-pull vs open-drain) isn't explicitly stated; push-pull is a reasonable inferred default since no external pull-up is documented as required.
3. NVIC tab: enable `EXTI Line1 interrupt`.
4. Regenerate. This produces `DIO1_Pin`/`DIO1_GPIO_Port` in `main.h`, EXTI init lines in `MX_GPIO_Init()`, and a new `EXTI1_IRQHandler` in `stm32l4xx_it.c` — all generated, no hand-editing needed for this part.

## Build system changes

- **`CMakeLists.txt`**: add `CXX` to `enable_language(C CXX ASM)`, set `CMAKE_CXX_STANDARD 17` (embedded-friendly; the toolchain file already sets `-fno-rtti -fno-exceptions -fno-threadsafe-statics` and points `CMAKE_CXX_COMPILER` at `arm-none-eabi-g++`, so no toolchain-file changes needed). Add `Core/Src/Sx1280Device.cpp` to the existing empty `target_sources(...)` block.
- **`.devcontainer/apt-packages.in`**: add `libstdc++-arm-none-eabi-newlib` — same `Recommends`-not-`Depends` gotcha hit earlier with `libnewlib-arm-none-eabi`/`errno.h`; this is the C++-stdlib equivalent and wasn't needed until now since the project was pure C. Requires a devcontainer rebuild.

## Verification plan, staged (reuse already-proven techniques)

1. **Build sanity** — `STM32: build` VS Code task compiles clean with the new `.cpp` file and `CXX` enabled, before touching the `.ioc`.
2. **Reset/BUSY sequence** — flash a version with just TCXO-enable + reset + BUSY-wait; use `gdb-multiarch` + `st-util` to halt and read the `Busy` pin's GPIO IDR live, confirm it settles low after reset.
3. **Command sequence, oscilloscope** — probe `SCK`/`MOSI`/`NSS` to confirm each new command's byte count and framing looks right, and that `BUSY` pulses and clears after each one.
4. **Status verification** — interleave `GetStatus` reads (the one verified-working command) after each new config command during bring-up, to confirm the chip's mode bits actually changed as commanded.
5. **TX without IRQ first** — validate `WriteBuffer`+`SetTx` alone by temporarily polling `GetIrqStatus` in the loop (toggle LED on TxDone bit) *before* wiring up `DIO1`/EXTI — isolates "does TX happen" from "does the interrupt path work."
6. **DIO1/EXTI wiring** — after `.ioc` regen, confirm via GDB that `EXTI->PR1` bit 1 and NVIC enable state look right.
7. **Full pipeline acceptance** — flash final code, confirm `LD2` toggles once per second driven purely by the `HAL_GPIO_EXTI_Callback` → `Sx1280_OnDio1Irq()` → `txDoneFlag` path (not the software timer), and cross-check on the oscilloscope that `DIO1` pulses shortly after each `SetTx`, correlated 1:1 with the LED toggles.

## Full command list — opcodes to look up

| Command | Purpose | Status |
|---|---|---|
| **GetStatus** | Reads a status byte reporting chip mode/state. Used as the basic "is the chip alive and responding" probe. Single-byte full-duplex transfer (status returned in the same byte as the opcode, not a second byte). | **Confirmed** (`0xC0`, tested on hardware; response layout confirmed via Table 11-3/11-4) |
| **SetStandby** | Puts the chip into a configurable idle state. **Use `STDBY_XOSC` (`0x01`), not `STDBY_RC` (`0x00`)** — this hardware has a TCXO wired and sequenced (`TCXOEN`), and `STDBY_RC` would ignore it and clock off the imprecise internal 13MHz RC oscillator instead, defeating the point of the TCXO warm-up. Must run before changing most other settings. | **Confirmed** (`0x80`, param table 11-18) |
| **SetPacketType** | Selects which modem the chip operates as — GFSK/LoRa/Ranging/FLRC/BLE. Everything downstream (modulation params, packet params) is interpreted differently depending on this. | **Confirmed** (`0x8A`, param `LoRa = 0x01`) |
| **SetRfFrequency** | Sets the actual 2.4GHz carrier frequency, as a 3-byte big-endian register value: `freq_reg = RF_Hz / (52e6 / 2^18)`. Sets the **TX** frequency specifically — RX is offset down by the IF (default 1.3MHz), not relevant to current TX-only scope but worth knowing for later. | **Confirmed** (`0x86`) |
| **SetModulationParams** | For **BLE/GFSK**: `modParam1=BitrateBandwidth`, `modParam2=ModulationIndex`, `modParam3=ModulationShaping`/BT. **Recommended BLE values**: `(0x45, 0x01, 0x20)` = 1Mbps/1.2MHz, modulation index 0.5, BT=0.5 (Bluetooth Core Spec mandatory GFSK PHY). For **LoRa**: `modParam1=SF`, `modParam2=BW`, `modParam3=CR`, full value tables below. | **Fully confirmed**, both modes |
| **SetPacketParams** | For **BLE**: `Param1=ConnectionState=0x02(BLE_ADVERTISER)`, `Param2=CrcLength=0x10(BLE_CRC_3B)`, `Param3=BleTestPayload=0x00`(unconfirmed relevance outside Test modes), `Param4=Whitening=0x00(BLE_WHITENING_ENABLE)`, `Param5-7`=unused. For **LoRa**: `Param1=PreambleLength` (packed nibbles, mantissa×2^exponent — see below), `Param2=HeaderType`, `Param3=PayloadLength`(raw 1-255), `Param4=CRC`, `Param5=InvertIQ`, full value tables below. | **Fully confirmed**, both modes |
| **WriteRegister** | Separate command (not `WriteBuffer`) needed to set the BLE `SyncWord1` (32-bit access address, registers `0x09CF`-`0x09D2`) and optionally `CrcInit` (registers `0x9C7`-`0x9C9`) — **required before first BLE TX**, not automatic. Standard BLE advertising values: `SyncWord1 = 0x8E89BED6`, `CrcInit = 0x555555`. Also required for LoRa SF5/SF6/SF7/SF8 (see LoRa SF table note below). Layout `[address[15:8], address[7:0], data...]`. | **Confirmed** (`0x18`) |
| **SetBufferBaseAddress** | Tells the chip where in its internal FIFO memory the TX and RX regions start. `WriteBuffer`'s offset is relative to this. | **Confirmed** (`0x8F`, layout `[txBaseAddress, rxBaseAddress]`) |
| **WriteBuffer** | Copies the payload bytes into the chip's internal FIFO at the configured offset. Doesn't transmit anything yet — just stages the data. Status byte returned on every byte of the transaction (confirms the "status piggybacks on every command" behavior). | **Confirmed** (`0x1A`, layout `[offset, data...]`) |
| **SetTx** | Triggers actual transmission — the chip autonomously modulates and radiates whatever's currently staged in its FIFO. Only command where anything leaves the antenna. Layout `[periodBase, periodBaseCount[15:8], periodBaseCount[7:0]]`; `periodBaseCount=0x0000` = Single mode (no timeout, auto-returns to `STDBY_RC` on completion). **`ClearIrqStatus` must be called before `SetTx`**, not just after — datasheet requirement. | **Confirmed** (`0x83`) |
| **SetDioIrqParams** | Configures which internal IRQ events (TxDone, RxDone, etc.) get routed to which physical DIO pin (`DIO1`/`DIO2`/`DIO3`). `TxDone = bit 0` (full 16-bit IRQ table below). To route TxDone to DIO1: `dio1Mask = 0x0001`. Layout `[irqMask×2, dio1Mask×2, dio2Mask×2, dio3Mask×2]`. | **Fully confirmed** (`0x8D`) |
| **GetIrqStatus** | Reads which IRQ flags are currently set — used both in the interrupt handler (confirm it really was `TxDone`, bit 0) and for a polling-based fallback test before wiring up the interrupt. **Byte timing differs from the generic `readCommand()` sketch**: `[opcode, NOP, NOP, NOP]` sent, response is `[status, status, irqStatus[15:8], irqStatus[7:0]]` — 2 status bytes before real data starts, not 1. Needs a special-cased read or a configurable skip-count in `readCommand()`. | **Confirmed** (`0x15`) |
| **ClearIrqStatus** | Clears IRQ flags after they've been handled — without this, a flag stays latched and the interrupt won't fire cleanly again next time. Also required *before* `SetTx` (see that row). Layout `[irqMask[15:8], irqMask[7:0]]`. | **Confirmed** (`0x97`) |

**All 11 commands and every enum value needed for both BLE and LoRa are now fully confirmed** — the full LoRa value tables and IRQ bit table follow below. Nothing RX-related included since that's out of scope for now.

## LoRa `SetModulationParams` value tables

**SF** (`modParam1`):

| Symbol | Value | SF |
|---|---|---|
| `LORA_SF_5` | `0x50` | 5 |
| `LORA_SF_6` | `0x60` | 6 |
| `LORA_SF_7` | `0x70` | 7 |
| `LORA_SF_8` | `0x80` | 8 |
| `LORA_SF_9` | `0x90` | 9 |
| `LORA_SF_10` | `0xA0` | 10 |
| `LORA_SF_11` | `0xB0` | 11 |
| `LORA_SF_12` | `0xC0` | 12 |

**Important extra step, easy to miss**: depending on which SF is chosen, an additional `WriteRegister(0x925, ...)` call is *required*: SF5/SF6 → `WriteRegister(0x925, 0x1E)`; SF7/SF8 → `WriteRegister(0x925, 0x37)`; SF9-SF12 → `WriteRegister(0x925, 0x32)`. Not optional — datasheet states it directly.

**BW** (`modParam2`):

| Symbol | Value | Bandwidth (kHz) |
|---|---|---|
| `LORA_BW_1600` | `0x0A` | 1625.0 |
| `LORA_BW_800` | `0x18` | 812.5 |
| `LORA_BW_400` | `0x26` | 406.25 |
| `LORA_BW_200` | `0x34` | 203.125 |

**CR** (`modParam3`):

| Symbol | Value | Rate |
|---|---|---|
| `LORA_CR_4_5` | `0x01` | 4/5 |
| `LORA_CR_4_6` | `0x02` | 4/6 |
| `LORA_CR_4_7` | `0x03` | 4/7 |
| `LORA_CR_4_8` | `0x04` | 4/8 |
| `LORA_CR_LI_4_5` | `0x05` | 4/5, long interleaving |
| `LORA_CR_LI_4_6` | `0x06` | 4/6, long interleaving |
| `LORA_CR_LI_4_7` | `0x07` | 4/8, long interleaving |

## LoRa `SetPacketParams` value tables

**`Param1` PreambleLength** — packed as two 4-bit nibbles, not a raw value: `preamble length (symbols) = MANT * 2^EXP`, where `Param1[3:0] = MANT [1-15]`, `Param1[7:4] = EXP [1-15]`. Recommended preamble length is 12 symbols → e.g. `MANT=3, EXP=2` (`3*2^2=12`) → byte `= (EXP<<4)|MANT = 0x23`.

**`Param2` HeaderType**: `EXPLICIT_HEADER = 0x00` (variable-length packet), `IMPLICIT_HEADER = 0x80` (fixed-length, no header sent).

**`Param3` PayloadLength**: raw value, `1-255`. Note: max 253 bytes if using `LORA_CR_LI_4_7` with CRC enabled.

**`Param4` CRC**: `LORA_CRC_ENABLE = 0x20`, `LORA_CRC_DISABLE = 0x00`.

**`Param5` InvertIQ**: `LORA_IQ_STD = 0x40` (standard), `LORA_IQ_INVERTED = 0x00` (swapped) — note the counterintuitive encoding, "standard" is the nonzero value.

## Full IRQ bit table (Table 11-71)

| Bit | IRQ | Applies to |
|---|---|---|
| 0 | **TxDone** | All |
| 1 | RxDone | All |
| 2 | SyncWordValid | GFSK/BLE/FLRC |
| 3 | SyncWordError | FLRC |
| 4 | HeaderValid | LoRa/Ranging |
| 5 | HeaderError | LoRa/Ranging |
| 6 | CrcError | GFSK/BLE/FLRC/LoRa |
| 7 | RangingSlaveResponseDone | Ranging |
| 8 | RangingSlaveRequestDiscard | LoRa/Ranging |
| 9 | RangingMasterResultValid | Ranging |
| 10 | RangingMasterTimeout | Ranging |
| 11 | RangingMasterRequestValid | Ranging |
| 12 | CadDone | LoRa/Ranging |
| 13 | CadDetected | LoRa/Ranging |
| 14 | RxTxTimeout | All |
| 15 | PreambleDetected | All (if `SetLongPreamble` active) |

For our goal: `dio1Mask = 0x0001` (bit 0 only) routes just `TxDone` to `DIO1`.

## Status byte decode (`GetStatus` / Table 11-3)

The single status byte returned by `GetStatus` (and, per the datasheet, also piggybacked on every other command's response bytes) packs three fields:

```c
uint8_t status = status_rx[0];   // byte 0 -- same byte as the opcode, full-duplex

uint8_t circuit_mode    = (status >> 5) & 0x07;   // bits 7:5
uint8_t command_status  = (status >> 2) & 0x07;   // bits 4:2
uint8_t busy_bit        =  status       & 0x01;   // bit 0 (bit 1 reserved)
```

**Circuit mode** (bits 7:5): `0x2`=`STDBY_RC`, `0x3`=`STDBY_XOSC`, `0x4`=`FS`, `0x5`=`Rx`, `0x6`=`Tx`, `0x0`/`0x1`=Reserved.

**Command status** (bits 4:2): `0x1`=processed successfully, `0x2`=data available to host (RX), `0x3`=command timeout, `0x4`=command processing error (bad opcode/param count), `0x5`=failure to execute, `0x6`=**Command Tx done** (this is the value to check for confirming `SetTx` completed), `0x0`=Reserved.

Verified example: a live read of `0x41` decoded to `circuit_mode=STDBY_RC(0x2)`, `command_status=Reserved(0x0)`, `busy=1` — consistent with reading right after a debugger-driven reset, before any real command had been issued yet.

## BLE PDU structure — what actually goes into `WriteBuffer`

Confirmed directly from the datasheet: **the SX1280 does not build the BLE PDU for you** — "the headers are not generated by the transceiver and must be calculated externally and passed as part of the payload to the data buffer" (`sx1280.txt:2040-2041`). So the entire byte array handed to `WriteBuffer` has to already be a valid PDU; the chip only handles what's *around* it.

**Full over-the-air packet** (Figure 7-4, `sx1280.txt:1994-2009`) — only the middle piece is host-built, the rest is automatic:

| Field | Size | Who builds it |
|---|---|---|
| Preamble | 1 byte | Chip, automatic |
| Access Address | 4 bytes | Chip, automatic — from the `SyncWord1` register already written via `WriteRegister` in `init()` |
| **PDU** | **2–39 bytes** | **Host — this is the entire `WriteBuffer` payload** |
| CRC | 3 bytes | Chip, automatic — computed from the `CrcInit` register (address `0x09C7`-`0x09C9`, standard BLE value `0x555555` per `sx1280.txt:5080-5088` — **not yet wired into `init()`**, needed for a real scanner's CRC check to pass) |
| Whitening | — | Chip, automatic |

**PDU structure** (Figure 7-5, advertising-channel format, `sx1280.txt:2011-2041`):

```
PDU:
| Header  | AdvA    | AdvData      |
| 2 bytes | 6 bytes | 0–25 bytes   |   <- max PDU = 2 + 31 = 33 bytes total

Header byte 0: PDUtype(4b) | RFU(2b) | TxAdd(1b) | RxAdd(1b)
Header byte 1: Length(6b)  | RFU(2b)
```

`Length` = size of everything *after* the header, i.e. `6 (AdvA) + len(AdvData)` — not just the `AdvData` size alone.

**Constants we're using** (the `PDUtype` code, `TxAdd`/`RxAdd` meaning, address byte-order, and the "random static address" bit rule are standard **Bluetooth Core Spec** facts — the SX1280 datasheet only confirms the *field layout* above, not these specific values, same caveat as the earlier BLE-channel-frequency numbers):

- `PDUtype = ADV_NONCONN_IND = 0x02` — non-connectable, undirected advertising. Simplest valid PDU type; the device can never be connected to, pure one-way broadcast (no handshake — any listening scanner picks it up or doesn't, no ack/retry).
- `TxAdd = 1` (random address, not public — no IEEE registration needed for a test device).
- `RxAdd = 0` (unused/reserved for undirected advertising — only meaningful for directed types like `ADV_DIRECT_IND`).
- Header byte 0 = `PDUtype | (TxAdd<<6) | (RxAdd<<7) = 0x02 | 0x40 = 0x42`.
- Header byte 1 = `Length = 6 + len(AdvData)` (computed once `AdvData` size is finalized).
- `AdvA` placeholder — **BLE transmits multi-byte fields little-endian (LSB first) over the air**, so the type-bearing MSB goes *last* in the array, not first: `{0x01, 0x00, 0x00, 0x00, 0x00, 0xC0}`. The last byte's top 2 bits (`0xC0 = 0b11000000`) must be `11` for a valid *random static* address (Core Spec rule) — this placeholder satisfies that while being obviously fake.
- `AdvData` — fixed placeholder bytes for now (the part that becomes AS5600 angle data later). No `[Length][Type]` AD-structure framing needed for this pass — that's only required if a scanner app should render something recognizable (e.g. a device name); raw bytes transmit fine either way since the chip doesn't validate BLE-application-layer content.

## `WriteBuffer` — pseudocode and prerequisites

Before `WriteBuffer` is meaningful, these need to already be true, **in this order** — per datasheet §13.1.1 "Common Transceiver Settings" (`sx1280.txt:4357-4384`), which explicitly states "the order is important" for steps 1-4 below; the BLE-specific steps (5-7) layer on top, after the datasheet's generic sequence completes:

1. **Chip in Standby** — `SetStandby(STDBY_XOSC)` must have run (see confirmed opcode above), so the chip is in a configurable state clocked off the TCXO.
2. **PacketType = BLE** — `SetPacketType(PACKET_TYPE_BLE)` must have run, since buffer/packet interpretation depends on modem mode (BLE first per revised decision; LoRa follow-up will need this and downstream params switched back).
3. **RF frequency set** — `SetRfFrequency(rfFrequency)` must have run. **Chosen: BLE advertising channel 37 (2402 MHz)** — one of the three standard primary advertising channels (`37`=2402 MHz, `38`=2426 MHz, `39`=2480 MHz per Bluetooth Core Spec Vol 6, Part B, §1.4.1 — not SX1280-datasheet-sourced, the chip has no concept of BLE channel numbers, only raw frequency). Any one of the three works: BLE scanners hop across all three every scan cycle, so the phone doesn't need to match a specific channel. `freq_reg = RF_Hz / (52e6 / 2^18) = 2,402,000,000 / (52e6/262144) = 12,109,036 = 0xB8C4EC`.
4. **Buffer base address configured** — `SetBufferBaseAddress` must have run once; `WriteBuffer`'s offset parameter is relative to whatever TX base address this sets (commonly `0x00`). Datasheet §13.1.1 places this *before* `SetModulationParams`/`SetPacketParams`, not after — corrected from an earlier draft of this doc that had it last.
5. **Modulation params set** — `SetModulationParams` with BLE 1M PHY values (bitrate/bandwidth, modulation index, BT), confirmed above.
6. **Packet params set** — `SetPacketParams` with `BLE_ADVERTISER` connection state and the other confirmed BLE-column values, above.
7. **Access address written** — `WriteRegister` setting `SyncWord1` to the standard BLE advertising access address (`0x8E89BED6`), a separate register-level command from everything else here. BLE-specific, not part of the datasheet's generic §13.1.1 sequence, so it runs last, once, before the first `SetTx`.

Pseudocode, using the same SPI pattern already proven in the working `GetStatus` code (`NSS` low → transmit → `NSS` high, with a `BUSY` wait before each command since the chip won't accept a new command while still processing the last one):

```c
// --- One-time setup, before any WriteBuffer/SetTx ---

waitOnBusy();
NSS = LOW;
spiTransmit(0x80);                // SetStandby, confirmed opcode
spiTransmit(0x01);                // STDBY_XOSC -- confirmed correct choice for TCXO hardware
NSS = HIGH;

waitOnBusy();
NSS = LOW;
spiTransmit(0x8A);                // SetPacketType, confirmed opcode
spiTransmit(0x04);                // PACKET_TYPE_BLE, confirmed value (BLE first, per revised decision)
NSS = HIGH;

waitOnBusy();
NSS = LOW;
spiTransmit(0x86);                // SetRfFrequency, confirmed opcode
// TODO: rfFrequency channel not yet decided -- 2402/2426/2480 MHz are the
// standard BLE primary advertising channels (37/38/39). freq_reg = RF_Hz / (52e6/2^18).
spiTransmit((rfFrequency >> 16) & 0xFF);
spiTransmit((rfFrequency >> 8) & 0xFF);
spiTransmit(rfFrequency & 0xFF);
NSS = HIGH;

waitOnBusy();
NSS = LOW;
spiTransmit(0x8F);                // SetBufferBaseAddress, confirmed opcode
spiTransmit(txBaseAddress = 0x00);
spiTransmit(rxBaseAddress = 0x00);
NSS = HIGH;

waitOnBusy();
NSS = LOW;
spiTransmit(0x8B);                // SetModulationParams, confirmed opcode
spiTransmit(0x45);                // BitrateBandwidth: 1Mbps/1.2MHz (BLE 1M PHY)
spiTransmit(0x01);                // ModulationIndex: 0.5 (Bluetooth spec mandated)
spiTransmit(0x20);                // ModulationShaping/BT: 0.5 (Bluetooth spec mandated)
NSS = HIGH;

waitOnBusy();
NSS = LOW;
spiTransmit(0x8C);                // SetPacketParams, confirmed opcode
spiTransmit(0x02);                // ConnectionState = BLE_ADVERTISER
spiTransmit(0x10);                // CrcLength = BLE_CRC_3B
spiTransmit(0x00);                // BleTestPayload -- unconfirmed for Advertiser mode, safe filler
spiTransmit(0x00);                // Whitening = BLE_WHITENING_ENABLE
spiTransmit(0x00);                // Param5, unused
spiTransmit(0x00);                // Param6, unused
spiTransmit(0x00);                // Param7, unused
NSS = HIGH;

waitOnBusy();
NSS = LOW;
spiTransmit(0x18);                // WriteRegister, confirmed opcode
spiTransmit(0x09);  spiTransmit(0xCF);   // SyncWord1 register address, MSB first
spiTransmit(0x8E); spiTransmit(0x89); spiTransmit(0xBE); spiTransmit(0xD6); // standard BLE advertising access address
NSS = HIGH;
// One-time register write, not part of the datasheet's generic §13.1.1
// sequence -- BLE-specific, but must still happen before the first SetTx.

// --- Actual WriteBuffer call ---

waitOnBusy();
NSS = LOW;
spiTransmit(0x1A);                // WriteBuffer, confirmed opcode
spiTransmit(offset = 0x00);       // relative to txBaseAddress set above
for (byte : payload) {
    spiTransmit(byte);
}
NSS = HIGH;

// Payload now sits in the chip's internal FIFO. Nothing has left the
// antenna yet -- WriteBuffer only stages bytes; SetTx is what actually
// triggers RF transmission of whatever's currently in the buffer.

// --- ClearIrqStatus, REQUIRED before SetTx per datasheet ---

waitOnBusy();
NSS = LOW;
spiTransmit(0x97);                // ClearIrqStatus, confirmed opcode
spiTransmit(irqMask[15:8] = 0xFF);
spiTransmit(irqMask[7:0]  = 0xFF);
NSS = HIGH;

// --- SetTx: this is the only step where anything actually radiates ---

waitOnBusy();
NSS = LOW;
spiTransmit(0x83);                // SetTx, confirmed opcode
spiTransmit(periodBase = 0x00);
spiTransmit(periodBaseCount[15:8] = 0x00);
spiTransmit(periodBaseCount[7:0]  = 0x00);   // 0x0000 = Single mode, no timeout
NSS = HIGH;

// Chip now autonomously modulates and transmits the buffered payload,
// then auto-returns to STDBY_RC on completion (Single mode). TxDone
// fires on DIO1 once transmission finishes (once SetDioIrqParams has
// routed it there) -- see EXTI section above.
```

`BUSY` nuance: the chip pulls `BUSY` high on its own while processing each command (you don't control that, only observe it) — so `waitOnBusy()` before *every* command (not just the first) is what prevents sending a new command while the chip's still digesting the last one.

`SET_BUFFER_BASE_ADDRESS` and `WRITE_BUFFER` are still placeholder names — real opcode bytes pending datasheet lookup (see the command table above).

## Critical files

- `Core/Inc/Sx1280Device.hpp` (new)
- `Core/Src/Sx1280Device.cpp` (new)
- `Core/Src/main.c`
- `Core/Inc/main.h` (regenerated after `.ioc` change)
- `Core/Src/stm32l4xx_it.c` (regenerated after `.ioc` change)
- `CMakeLists.txt`
- `homework_18/homework_18.ioc`
- `.devcontainer/apt-packages.in`
