/*
 * CoMotion Tracker — Battery Voltage Monitor
 *
 * Reads LiPo battery voltage via voltage divider on AIN7 (P0.31).
 * P0.14 enables the divider (drive LOW).
 * Uses Zephyr voltage-divider sensor driver.
 */

#ifndef BATTERY_H
#define BATTERY_H

/**
 * Initialize battery monitor: enable voltage divider, verify sensor.
 *
 * @return 0 on success, negative errno on failure
 */
int battery_init(void);

/**
 * Take a battery voltage sample (blocks briefly for ADC).
 *
 * @return 0 on success, negative errno on failure
 */
int battery_sample(void);

/**
 * Get the last sampled battery voltage in millivolts.
 * Call battery_sample() first.
 */
int battery_millivolts(void);

/**
 * Get estimated battery level as 0–100%.
 * Simple linear mapping: 3000 mV = 0%, 4200 mV = 100%.
 * Call battery_sample() first.
 */
int battery_level_pct(void);

#endif /* BATTERY_H */
