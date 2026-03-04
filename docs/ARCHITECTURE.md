# Architecture

## Overview

The firmware runs on Zephyr RTOS with cooperative threading. Each subsystem (BLE, sensors, GPS, SD card) operates independently and communicates via shared state and Zephyr message queues.

## Modules

### `main.c` — Entry Point
- Initializes all subsystems
- Starts worker threads
- Manages system state and power modes

### `ble.c/h` — Bluetooth LE
- Coded PHY (S=8) for 4× range over 1M PHY
- Extended advertising with two ad sets
- Nordic UART Service (NUS) for data streaming
- Adaptive reconnection with exponential backoff
- Connection parameter negotiation

### `sensors.c/h` — IMU
- LSM6DS3 via I2C (accelerometer + gyroscope)
- Configurable ODR and full-scale range
- Trigger-based data ready interrupts

### `intensity.c/h` — Impact Detection
- Real-time acceleration magnitude calculation
- Threshold-based impact classification
- Configurable sensitivity levels

### `gps.c/h` — Location
- UART GPS module (NMEA parsing)
- Interrupt-driven RX
- Optional — disabled if no GPS hardware present

### `sdcard.c/h` — Data Logging
- SPI SD card with FAT filesystem
- Timestamped CSV/binary logging
- Auto-creates session files

### `common.h` — Shared Definitions
- Configuration constants
- Shared data structures
- Feature flags

## Data Flow

```
IMU (I2C) → sensors.c → intensity.c → classification
                ↓              ↓
            sdcard.c ←──── ble.c → BLE NUS → App
                ↑
            gps.c (UART)
```

## Threading Model

Zephyr preemptive scheduler with dedicated threads per subsystem. Main thread handles initialization and power management.
