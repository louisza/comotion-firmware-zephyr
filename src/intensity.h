/*
 * CoMotion Tracker — Intensity & Movement Module
 *
 * Processes accelerometer + gyroscope data to compute:
 *   1. Real-time intensity (1-second EMA)
 *   2. Medium-term intensity (1-minute EMA)
 *   3. Long-term intensity (10-minute rolling average)
 *   4. Movement counter (rising-edge detector)
 *
 * All intensity values are dimensionless floats (0 = still, ~5 = very active).
 */

#ifndef INTENSITY_H
#define INTENSITY_H

#include <stdint.h>

/**
 * Initialize intensity tracking. Call once at boot.
 */
void intensity_init(void);

/**
 * Reset all session statistics. Call on logging start.
 */
void intensity_reset(void);

/**
 * Feed one IMU sample. Call at 104 Hz.
 *
 * @param ax_g    X acceleration in g
 * @param ay_g    Y acceleration in g
 * @param az_g    Z acceleration in g
 * @param gx_dps  X gyro in degrees/s
 * @param gy_dps  Y gyro in degrees/s
 * @param gz_dps  Z gyro in degrees/s
 */
void intensity_feed(float ax_g, float ay_g, float az_g,
		    float gx_dps, float gy_dps, float gz_dps);

/**
 * Sample current intensity into the 10-minute rolling history.
 * Call at exactly 2 Hz.
 */
void intensity_sample_history(void);

/** Get 1-second EMA of intensity (fast response). */
float intensity_get_current(void);

/** Get 1-minute EMA of intensity (medium term). */
float intensity_get_1min(void);

/** Get 10-minute rolling average of intensity. */
float intensity_get_10min_avg(void);

/** Get rising-edge movement count (session total). */
uint16_t intensity_get_move_count(void);

#endif /* INTENSITY_H */
