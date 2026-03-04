# CoMotion Tracker — Zephyr Firmware

Zephyr RTOS firmware for the **XIAO nRF52840 BLE (Sense)** — a wearable impact/motion tracker for combat sports.

> **Migration:** This replaces the [PlatformIO version](https://github.com/louisza/comotion-firmware) with Zephyr OS / Nordic nRF Connect SDK for better power management, Coded PHY support (4× range), and professional toolchain.

## Features

- **BLE Coded PHY** — Extended range (~4× vs 1M PHY) with adaptive reconnection
- **9-axis IMU** — LSM6DS3 accelerometer + gyroscope for motion tracking
- **Impact Detection** — Real-time force/intensity classification
- **GPS Integration** — Optional UART GPS module for location logging
- **SD Card Logging** — FAT filesystem data logging (SPI)
- **Low Power** — Zephyr LPM with battery voltage monitoring (ADC)
- **USB Console** — CDC ACM for debug output

## Hardware

| Component | Details |
|-----------|---------|
| MCU | Seeed XIAO nRF52840 BLE (Sense) |
| IMU | LSM6DS3 (onboard) |
| GPS | UART GPS module (optional) |
| Storage | MicroSD via SPI |
| Battery | LiPo with ADC monitoring |

## Quick Start

### Prerequisites

- [nRF Connect SDK](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/installation.html) (v2.5+ recommended)
- `west` meta-tool
- CMake 3.20+

### Build & Flash

```bash
# Set up Zephyr workspace (if not done)
west init -m https://github.com/nrfconnect/sdk-nrf --mr v2.5.0 ncs
cd ncs && west update

# Clone this repo
git clone https://github.com/louisza/comotion-firmware-zephyr.git
cd comotion-firmware-zephyr

# Build
west build -b xiao_ble/nrf52840/sense

# Flash (via UF2 or J-Link)
west flash
```

### Monitor Output

```bash
# USB serial console
screen /dev/ttyACM0 115200
```

## Project Structure

```
├── CMakeLists.txt          # Zephyr build configuration
├── prj.conf                # Kconfig options (BLE, sensors, SD, GPS, etc.)
├── boards/
│   └── xiao_ble_nrf52840_sense.overlay  # Device tree overlay
└── src/
    ├── main.c              # Entry point, thread orchestration
    ├── ble.c/h             # BLE Coded PHY, NUS, reconnection
    ├── sensors.c/h         # IMU data acquisition
    ├── intensity.c/h       # Impact detection & classification
    ├── gps.c/h             # GPS UART parsing
    ├── sdcard.c/h          # SD card FAT filesystem logging
    └── common.h            # Shared definitions & config
```

## Documentation

- [Build Guide](docs/BUILD.md) — Detailed build & flash instructions
- [Architecture](docs/ARCHITECTURE.md) — Code structure & data flow
- [Device Tree](docs/DEVICE_TREE.md) — Board overlay customization

## License

MIT — see [LICENSE](LICENSE)
