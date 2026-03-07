/*
 * CoMotion Tracker — Dual-Trigger Impact Detection
 *
 * Requires BOTH an IMU high-g spike AND an audio spike within 200ms
 * of each other for a confirmed impact. This eliminates false positives
 * from shouts (audio only) or hard steps (IMU only).
 *
 * Debounce: 3 seconds minimum between confirmed impacts.
 * BLE flag: auto-clears 2 seconds after the last confirmed impact.
 */

#ifndef IMPACT_H
#define IMPACT_H

#include <stdint.h>
#include <stdbool.h>

/**
 * Initialize impact detection. Call once at boot.
 */
void impact_init(void);

/**
 * Reset session impact statistics. Call on logging start.
 */
void impact_reset(void);

/**
 * Feed an IMU sample for impact threshold detection.
 * Call at 104 Hz from the IMU loop.
 *
 * @param accel_mag_g  Acceleration magnitude in g (sqrt of sum of squares)
 */
void impact_feed_imu(float accel_mag_g);

/**
 * Feed audio data and run confirmation logic.
 * Call at 10 Hz from the audio thread.
 *
 * @param peak      Raw audio peak amplitude (0–32767)
 * @param baseline  Current audio baseline (slowly-adapted RMS)
 */
void impact_feed_audio(uint16_t peak, float baseline);

/** Get total confirmed impact count for this session. */
uint8_t impact_get_count(void);

/** True if a confirmed impact occurred within the last 2 seconds. */
bool impact_is_recent(void);

/** Get timestamp of last confirmed impact (0 = none). */
int64_t impact_get_last_time(void);

#endif /* IMPACT_H */
