# Kuntur headstage firmware

STM32WB09KE firmware for the **Kuntur** headstage. Receives two channels of FPGA data at 30 000 SPS and streams them in real time over Bluetooth Low Energy to [Tunki](../../software/tunki/), the desktop app, via the NUCLEO-WB09KE [BLE–USB bridge](../ble-usb-bridge/).

---

## Hardware

- **MCU**: STM32WB09TEFX — Cortex-M0+ with an integrated BLE 5.4 radio
- **Board**: NUCLEO-WB09KE
- **Clock**: 32 MHz system (RC64MPLL ÷ 2), 32.768 kHz LSE for RTC timestamps
- **RAM**: 64 KB (shared with BLE stack)
- **Flash**: 512 KB

---

## Project Layout

```
firmware/headstage/
├── Core/
│   ├── Inc/            app_conf.h, main.h, utilities_conf.h …
│   └── Src/            main.c, app_entry.c, stm32wb0x_it.c …
├── STM32_BLE/
│   └── App/
│       ├── stream.h / stream.c        GATT service, StreamDataPacket_t
│       ├── stream_app.h / stream_app.c  Ring buffer, send task, simulation
│       ├── dis.h / dis.c              Device Information Service (GATT)
│       ├── dis_app.h / dis_app.c      DIS values (manufacturer, model, …)
│       └── app_ble.c                  BLE stack init, event dispatcher
├── Debug/              STM32CubeIDE build output (.elf, .hex, makefile)
└── STM32WB09TEFX_FLASH.ld  Linker script
```

---

## Building

The project is managed by **STM32CubeIDE** (Eclipse-based). The `Debug/` directory contains the auto-generated makefile.

```bash
# Requires the STM32CubeIDE ARM toolchain on PATH
# Typical location: /opt/st/stm32cubeide_<version>/plugins/
#   com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*/tools/bin

make -C firmware/headstage/Debug all     # from the repository root
```

Build outputs: `Debug/kuntur-mcu.elf`, `Debug/kuntur-mcu.hex`.

---

## Flashing

Use **STM32CubeProgrammer** via the NUCLEO on-board ST-LINK.

```bash
STM32_Programmer_CLI \
  -c port=SWD sn=<serial> \
  -w firmware/headstage/Debug/kuntur-mcu.elf \
  -v -rst
```

After reset the firmware waits **5 seconds** before starting the BLE stack, allowing a debugger to attach.

---

## Functions

### System initialisation
- Clock configuration: 32 MHz system clock, HSE + LSE enabled
- 5-second startup delay for debugger attach
- UART smoke-test banner (`"KUNTUR UART OK"`) sent at boot
- Peripheral init: GPIO, RNG, Radio, Radio timer, PKA, USART1, RTC

### BLE stack management
- Stack init: GATT, GAP, memory pool
- **Advertising**: fast on boot → low-power after 60 s timeout; device name `"Kuntur-Headstage"`
- **Connection management**: connect/disconnect events dispatched to all services
- **Link setup sequence** on connect: DLE first (251-byte PDU) → 2M PHY; sequenced to avoid simultaneous LL procedures
- **TX flow control**: stops sending on `0x88` (mblock pool full), resumes on `ACI_GATT_TX_POOL_AVAILABLE`
- NVM ticking for BLE stack state persistence
- Cryptographic hardware: RNG, PKA, AES (used internally for BLE security)

### GATT services

**STREAM service (0xFFF0)**
- `0xFFF1` — write-without-response, client → Kuntur (reserved for future commands)
- `0xFFF2` — notify, Kuntur → client; carries `StreamDataPacket_t` (244 bytes)

**Device Information Service (DIS)**
- Read-only characteristics: Manufacturer (`OpenIC`), Model (`Kuntur-0.1.0`), Serial (`1.0`), Hardware revision (`0.1.0`), Firmware revision (`0.1.0`)

### Data streaming pipeline
- **CCCD gating**: streaming only starts after the client enables notifications; `seq_num` resets to 0 on each new connection
- **Packet assembly**: 8-byte RTC header + 59 pairs × (`int16_t` ch0 + `int16_t` ch1) = 244 bytes
- **Ring buffer**: 4 096 pairs × 4 bytes = 16 KB, ISR-safe via atomic index increments (~137 ms at 30 kSPS)
- **Sequence number**: rolling 0–255, lets the receiver count dropped packets
- **Simulation fallback**: when ring buffer has < 59 pairs, sends a 10-bit sawtooth (CH0 = N mod 1024, CH1 = (N + 512) mod 1024) to verify the BLE pipe independently of FPGA hardware
- **Live data path**: `STREAM_APP_IngestSamples(ch0, ch1, numPairs)` — call from SPI DMA handler; pipeline switches automatically from simulation to live data when ≥ 59 pairs are available

### Timestamping
- RTC clocked by LSE at 32.768 kHz, `BYPSHAD` mode (direct register reads, non-blocking)
- Each packet header: seconds (H×3600 + M×60 + S) + sub-second ticks rescaled 0–31 999 (31.25 µs resolution)
- Decode: `base_µs = timestamp_s × 1 000 000 + timestamp_sub_s × 1000 / 32`

### UART debug output
- **USART1**, 115 200 baud, 8N1, TX on PA1 (ST-LINK VCP)
- `APP_DBG_MSG` / `printf` → `__io_putchar` → `HAL_UART_Transmit` (blocking, task context only)
- Logs: connection events, DLE/PHY updates, CCCD state, flow-off stats (every 500 resumes)

---

## Stream Mode: WB09KE_HF

| Parameter | Value |
|---|---|
| Constant | `STREAM_MODE_WB09KE_HF` |
| Device name | `Kuntur-Headstage` |
| Target sample rate | 30 000 SPS per channel |
| Channels | 2 (ch0, ch1 — signed `int16_t`) |
| PHY | 2M |
| Connection interval | 7.5 ms (set by WB09KE central) |
| ATT MTU | 247 bytes (244-byte payload) |
| DLE | 251-byte TX PDU |
| VTIMER period | 2 ms safety fallback |
| Send loop | Tight `BLE_STACK_Tick()` burst until pool exhausted (`0x88`) |
| CPUP | Disabled — WB09KE central controls CI directly |

### Packet format (`0xFFF2` notify, 244 bytes)

```
Offset  Size   Field
  0      4     timestamp_s     — RTC seconds (H×3600 + M×60 + S)
  4      2     timestamp_sub_s — sub-second ticks, range 0–31 999
  6      1     seq_num         — rolling 0–255, for drop detection
  7      1     num_pairs       — sample pairs in packet (normally 59)
  8    4×59    samples         — 59 × (int16_t ch0, int16_t ch1), interleaved
```

### Streaming pipeline

```
VTIMER callback (StreamSendTimerCb)  ← 2 ms safety fallback
    │
    └─► UTIL_SEQ task (StreamSendTask)
            │
            ├─ Ring buffer ≥ 59 pairs → send live FPGA data
            └─ Ring buffer empty      → send simulation sawtooth
                    │
                    └─► STREAM_NotifyData() → aci_gatt_srv_notify on 0xFFF2
                                │
                                └─► BLE_STACK_Tick()  ← tight loop until 0x88
```

TX flow control: `0x88` (ATT pool full) pauses sending; `ACI_GATT_TX_POOL_AVAILABLE_VSEVT_CODE` resumes it.

### Ingesting live FPGA data

Call `STREAM_APP_IngestSamples()` from the SPI DMA completion handler (ISR-safe):

```c
#include "stream_app.h"

/* From SPI DMA completion ISR or task: */
STREAM_APP_IngestSamples(ch0_buf, ch1_buf, num_pairs);
```

The pipeline switches from simulation to live data automatically once ≥ 59 pairs are in the buffer. On disconnect the ring buffer keeps filling; on reconnect the backlog drains at maximum BLE rate.

---

## RTC Notes

`AsynchPrediv = 0`, `SynchPrediv = 32767` → 1 Hz calendar, 32.768 kHz sub-second counter. `BYPSHAD` is set for non-blocking reads.

**CubeMX warning**: if CubeMX regenerates `Core/Src/main.c`, restore `AsynchPrediv` and `SynchPrediv` to `0` / `32767` manually — see the comment inside `MX_RTC_Init()`.

---

## License

Apache License 2.0 — see `LICENSE` at the root of this repository.
