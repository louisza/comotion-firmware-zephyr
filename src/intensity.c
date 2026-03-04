/*
 * ═══════════════════════════════════════════════════════════════
 * CoMotion Tracker — Intensity Calculation
 *
 * Ported verbatim from Arduino firmware.
 * Pure C math — no library dependencies.
 * ═══════════════════════════════════════════════════════════════
 */

#include <math.h>
#include "common.h"
#include "intensity.h"

/* ─── State ─── */
static float intensity_raw;
static float intensity_current;
static float intensity_1min;
static float intensity_history[INTENSITY_HISTORY_SIZE];
static uint16_t history_index;
static uint16_t samples_in_history;
static float    intensity_10min_sum;

/* Movement detection */
static bool was_above_threshold;

/* ═══════════════════════════════════════════════════════════════
 * Called at 104 Hz from IMU timer
 * ═══════════════════════════════════════════════════════════════ */

void intensity_update(void)
{
	float ax = g_imu.ax, ay = g_imu.ay, az = g_imu.az;
	float gx = g_imu.gx, gy = g_imu.gy, gz = g_imu.gz;

	float accel_mag = sqrtf(ax*ax + ay*ay + az*az);
	float gyro_mag  = sqrtf(gx*gx + gy*gy + gz*gz);

	float accel_activity = (accel_mag > 1.0f) ? (accel_mag - 1.0f) : 0.0f;
	float gyro_activity  = gyro_mag / INTENSITY_GYRO_SCALE;

	intensity_raw = INTENSITY_ACCEL_WEIGHT * accel_activity
		      + INTENSITY_GYRO_WEIGHT  * gyro_activity;

	/* Exponential moving averages */
	intensity_current = intensity_current * (1.0f - INTENSITY_EMA_FAST)
			  + intensity_raw * INTENSITY_EMA_FAST;
	intensity_1min    = intensity_1min * (1.0f - INTENSITY_EMA_SLOW)
			  + intensity_raw * INTENSITY_EMA_SLOW;

	/* Update max GPS speed */
	if (g_gps.valid && g_gps.speed > g_max_speed_session) {
		g_max_speed_session = g_gps.speed;
	}

	/* Movement counter: rising edge above threshold */
	bool is_above = intensity_current > 0.3f;
	if (is_above && !was_above_threshold) {
		g_move_count++;
	}
	was_above_threshold = is_above;
}

/* ═══════════════════════════════════════════════════════════════
 * Called at 2 Hz — rolling 10-minute history
 * ═══════════════════════════════════════════════════════════════ */

void intensity_sample_history(void)
{
	intensity_10min_sum -= intensity_history[history_index];
	intensity_history[history_index] = intensity_current;
	intensity_10min_sum += intensity_current;

	if (samples_in_history < INTENSITY_HISTORY_SIZE) {
		samples_in_history++;
	}

	history_index = (history_index + 1) % INTENSITY_HISTORY_SIZE;
}

/* ─── Getters ─── */

float intensity_get_current(void)
{
	return intensity_current;
}

float intensity_get_1min(void)
{
	return intensity_1min;
}

float intensity_get_10min_avg(void)
{
	uint16_t divisor = (samples_in_history > 0) ? samples_in_history : 1;
	return intensity_10min_sum / divisor;
}

void intensity_reset(void)
{
	g_max_speed_session = 0;
	g_impact_count = 0;
	g_move_count = 0;
	intensity_current = 0;
	intensity_1min = 0;
	intensity_10min_sum = 0;
	history_index = 0;
	samples_in_history = 0;

	for (int i = 0; i < INTENSITY_HISTORY_SIZE; i++) {
		intensity_history[i] = 0;
	}
}
