# Device Tree Overlay

## `boards/xiao_ble_nrf52840_sense.overlay`

The overlay customizes the XIAO BLE board's device tree for CoMotion hardware:

### What It Configures

- **I2C0** — LSM6DS3 IMU (onboard, address 0x6A)
- **SPI** — SD card (external, custom pin assignments)
- **UART1** — GPS module (TX/RX pins)
- **ADC** — Battery voltage divider channel
- **GPIO** — LED indicators

### Customizing Pin Assignments

Edit the overlay to change pin mappings for your hardware revision:

```dts
&spi1 {
    cs-gpios = <&gpio0 YOUR_CS_PIN GPIO_ACTIVE_LOW>;
    // Update for your SD card CS pin
};

&uart1 {
    tx-pin = <YOUR_TX_PIN>;
    rx-pin = <YOUR_RX_PIN>;
    // Update for your GPS module wiring
};
```

### Adding New Peripherals

1. Add node to the overlay
2. Enable corresponding Kconfig in `prj.conf`
3. Add driver code in `src/`

See [Zephyr Device Tree docs](https://docs.zephyrproject.org/latest/build/dts/index.html) for reference.
