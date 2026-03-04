# Build Guide

## Prerequisites

1. **nRF Connect SDK (NCS)** v2.5.0+
   ```bash
   # Install west
   pip3 install west

   # Initialize NCS workspace
   west init -m https://github.com/nrfconnect/sdk-nrf --mr v2.5.0 ncs
   cd ncs && west update
   ```

2. **Zephyr SDK toolchain** — [Download](https://github.com/zephyrproject-rtos/sdk-ng/releases)

3. **CMake 3.20+**, **Python 3.8+**, **ninja**

## Building

```bash
cd comotion-firmware-zephyr

# Standard build
west build -b xiao_ble/nrf52840/sense

# Clean rebuild
west build -b xiao_ble/nrf52840/sense --pristine

# With extra config
west build -b xiao_ble/nrf52840/sense -- -DOVERLAY_CONFIG=debug.conf
```

## Flashing

### UF2 (No debugger needed)
1. Double-tap reset on XIAO to enter UF2 bootloader
2. Copy `build/zephyr/zephyr.uf2` to the mounted drive

### J-Link
```bash
west flash
```

## Debug Output

USB CDC ACM console at 115200 baud:
```bash
screen /dev/ttyACM0 115200
# or
minicom -D /dev/ttyACM0 -b 115200
```

## Common Issues

| Issue | Solution |
|-------|----------|
| `CONFIG_BT_NUS` not found | Ensure NCS (not vanilla Zephyr) is used |
| No USB serial | Check `CONFIG_USB_DEVICE_STACK=y` in prj.conf |
| IMU not detected | Verify I2C address in device tree overlay |
| SD card mount fails | Check SPI pin assignments in overlay |
