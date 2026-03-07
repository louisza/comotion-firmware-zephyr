/*
 * CoMotion Tracker — Battery Voltage Monitor
 *
 * Hardware: LiPo battery through voltage divider to AIN7 (P0.31).
 *   R_upper = 1 MΩ (VBATT to AIN7)
 *   R_lower = 510 KΩ (AIN7 to P0.14)
 *   P0.14 driven LOW enables the divider.
 *
 * Uses Zephyr voltage-divider sensor driver which handles:
 *   - ADC channel setup (gain 1/6, internal ref 0.6V, 12-bit)
 *   - Raw-to-millivolt conversion
 *   - Divider ratio scaling (full_ohms / output_ohms)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "battery.h"

LOG_MODULE_REGISTER(battery, CONFIG_LOG_DEFAULT_LEVEL);

/* Voltage divider sensor from DTS */
static const struct device *vbatt_dev = DEVICE_DT_GET(DT_NODELABEL(vbatt));

/* P0.14 enables the voltage divider (drive LOW to enable) */
static const struct device *gpio0_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
#define VBATT_EN_PIN 14

/* Last sampled value */
static int last_mv;

int battery_init(void)
{
	int ret;

	/* Enable voltage divider: P0.14 output LOW */
	if (!device_is_ready(gpio0_dev)) {
		LOG_ERR("GPIO0 device not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure(gpio0_dev, VBATT_EN_PIN, GPIO_OUTPUT_LOW);
	if (ret) {
		LOG_ERR("Failed to configure VBATT enable pin: %d", ret);
		return ret;
	}

	/* Small delay for divider to settle */
	k_msleep(1);

	/* Verify voltage-divider sensor is ready */
	if (!device_is_ready(vbatt_dev)) {
		LOG_ERR("Battery sensor not ready");
		return -ENODEV;
	}

	/* Take an initial sample */
	ret = battery_sample();
	if (ret) {
		LOG_WRN("Initial battery sample failed: %d", ret);
	} else {
		LOG_INF("Battery: %d mV (%d%%)", last_mv, battery_level_pct());
	}

	return 0;
}

int battery_sample(void)
{
	struct sensor_value val;
	int ret;

	ret = sensor_sample_fetch(vbatt_dev);
	if (ret) {
		LOG_ERR("Battery fetch failed: %d", ret);
		return ret;
	}

	ret = sensor_channel_get(vbatt_dev, SENSOR_CHAN_VOLTAGE, &val);
	if (ret) {
		LOG_ERR("Battery channel get failed: %d", ret);
		return ret;
	}

	/* sensor_value: val1 = integer volts, val2 = fractional microvolts */
	last_mv = val.val1 * 1000 + val.val2 / 1000;

	return 0;
}

int battery_millivolts(void)
{
	return last_mv;
}

int battery_level_pct(void)
{
	/* Linear mapping for LiPo (spec: 3.3V=0%, 4.2V=100%):
	 *   4200 mV = 100%
	 *   3300 mV = 0%
	 */
	int pct = (last_mv - 3300) * 100 / (4200 - 3300);

	if (pct < 0) {
		pct = 0;
	}
	if (pct > 100) {
		pct = 100;
	}

	return pct;
}
