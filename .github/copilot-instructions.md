# Copilot Instructions — CoMotion Tracker Firmware

## Project Overview

This is **CoMotion Tracker**, a Zephyr RTOS firmware for the **Seeed Studio XIAO nRF52840 BLE (Sense)** board. It is a wearable impact/motion tracker for combat sports (boxing, MMA, etc.) that measures real-time motion intensity, detects impacts, and streams data over BLE.

We are doing an **incremental bring-up** — starting from a bare LED blink test and adding one peripheral at a time, verifying each step on real hardware before moving on.

## Hardware

- **Board:** Seeed Studio XIAO BLE nRF52840 Sense (`xiao_ble/nrf52840/sense`)
- **MCU:** Nordic nRF52840 (ARM Cortex-M4F, 256KB RAM, 1MB Flash)
- **Onboard IMU:** LSM6DS3TR-C (I2C, power-gated via regulator on P1.08)
- **Onboard Microphone:** PDM mic (CLK = P1.00, DIN = P0.16, power-gated via regulator on P1.10)
- **LEDs:** 3 onboard LEDs (active LOW): Red (P0.26), Green (P0.30), Blue (P0.06)
- **External GPS:** ATGM332D on UART0 (TX=P1.11, RX=P1.12) at 9600 baud
- **External SD Card:** SPI2 (SCK=P1.13, MOSI=P1.15, MISO=P1.14), CS on D2 (P0.28)
- **Battery:** LiPo monitored via SAADC (direct register access, AIN7)
- **USB:** CDC ACM serial console auto-initialized by board defaults (new USB stack)

## SDK & Toolchain

- **nRF Connect SDK (NCS):** v3.2.3, installed at `C:\ncs\v3.2.3\`
- **Zephyr:** 4.2.99 (bundled with NCS)
- **Zephyr SDK:** 0.17.0 at `C:\ncs\toolchains\fd21892d0f\opt\zephyr-sdk`
- **Python:** 3.12.4 at `C:\ncs\toolchains\fd21892d0f\opt\bin\python.exe`
- **Build system:** CMake + Ninja via `west build`

## Build Command

```
$env:ZEPHYR_BASE = "C:\ncs\v3.2.3\zephyr"
$env:PATH = "C:\ncs\toolchains\fd21892d0f\opt\bin;C:\ncs\toolchains\fd21892d0f\opt\bin\cmake\bin;C:\ncs\toolchains\fd21892d0f\opt\bin\ninja;C:\ncs\toolchains\fd21892d0f\opt\zephyr-sdk\arm-zephyr-eabi\bin;$env:PATH"
& "C:\ncs\toolchains\fd21892d0f\opt\bin\python.exe" -m west build --build-dir d:/repos/comotion-firmware-zephyr/build d:/repos/comotion-firmware-zephyr --board xiao_ble/nrf52840/sense --pristine
```

## Critical Rules

### 1. Always Reference Official Documentation

All code, Kconfig options, device tree bindings, and API usage **must** be based on official documentation:

- **Zephyr API:** https://docs.zephyrproject.org/latest/
- **nRF Connect SDK:** https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/index.html
- **nrfx drivers:** https://docs.nordicsemi.com/bundle/nrfx-latest/page/index.html
- **Device Tree bindings:** https://docs.zephyrproject.org/latest/build/dts/index.html
- **Kconfig reference:** https://docs.zephyrproject.org/latest/kconfig.html

Do **not** guess API signatures, Kconfig symbol names, or device tree properties. When unsure, look up the actual header files in the Zephyr/NCS source tree at `C:\ncs\v3.2.3\`.

### 2. Board-Specific Awareness

This board has important hardware details that must be respected:

- **USB console** is handled entirely by board defaults (new USB stack, `CONFIG_USB_DEVICE_STACK_NEXT=y`, `CONFIG_CDC_ACM_SERIAL_INITIALIZE_AT_BOOT=y`). Do NOT manually call `usb_enable()` or force the legacy USB stack.
- **IMU and mic have power-gate regulators** controlled by GPIO. `CONFIG_REGULATOR=y` is needed for them to power on.
- **PDM mic pins** are CLK=P1.00, DIN=P0.16 (NOT P0.01/P0.00). The mic also needs P1.10 driven HIGH for power.
- **UART0** is on P1.11/P1.12 (XIAO D6/D7) — used for GPS. Do NOT create a separate uart1 on the same pins.
- **LEDs are active LOW** — the `gpio_pin_set_dt()` API handles this automatically via DTS flags.
- **Board DTS files** are at `C:\ncs\v3.2.3\zephyr\boards\seeed\xiao_ble\`

### 3. Incremental Development

We are building this firmware one peripheral at a time:

1. ✅ LED blink + USB serial console
2. ✅ IMU (LSM6DS3TR-C via Zephyr sensor API — ±16g, 1000dps, 104Hz)
3. ✅ BLE (advertising as "CoMotion" + NUS for IMU streaming)
4. ✅ GPS (ATGM332D on UART0 via Zephyr GNSS framework)
5. ✅ PDM microphone (Zephyr DMIC API, burst-mode 16kHz mono)
6. ✅ SD card (SPI FAT FS — zephyr,sdhc-spi-slot + sdmmc-disk, CSV logging)
7. ⬜ Battery ADC (direct SAADC)
8. ⬜ Full integration

Each step must compile, flash, and run correctly on real hardware before proceeding to the next.

### 4. Code Style

- Use tabs for indentation (Zephyr project convention)
- Use `printk()` for debug output (goes to USB CDC ACM console)
- Use Zephyr logging (`LOG_INF`, `LOG_ERR`, etc.) for module-level logging
- Prefer Zephyr APIs over direct register access where available
- Keep `prj.conf` minimal — let board defconfig defaults handle what they can
- Overlay (`boards/xiao_ble_nrf52840_sense.overlay`) should only add what the board DTS doesn't already provide

### 5. Common Pitfalls (Learned the Hard Way)

- **Do NOT use `CONFIG_USB_DEVICE_STACK=y`** — the board uses the new stack (`CONFIG_USB_DEVICE_STACK_NEXT=y`) by default
- **Do NOT add `&power` or `&temp` disabled nodes** to the overlay — they don't exist in this board's DTS
- **SAADC busy-wait loops need timeouts** — bare `while(!event);` will hang if the peripheral is in an unexpected state
- **`sensor_sample_fetch()` blocks on I2C** — if the IMU is not powered, this hangs forever
- **PowerShell `Set-Content` adds a UTF-8 BOM** — use `[System.IO.File]::WriteAllText()` with `UTF8Encoding($false)` instead
- **nrfx PDM API is instance-based in v3.x** — use `nrfx_pdm_init(&instance, ...)` not `nrfx_pdm_init(...)`

### 6. File Structure

```
CMakeLists.txt          — Build config (list source files here)
prj.conf                — Kconfig (keep minimal, board defaults handle USB/serial)
boards/
  xiao_ble_nrf52840_sense.overlay  — Device tree overlay (only add what board lacks)
src/
  main.c                — Entry point and main loop
docs/
  ARCHITECTURE.md        — System architecture documentation
  BUILD.md              — Build instructions
  DEVICE_TREE.md        — Device tree documentation
```
