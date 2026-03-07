/*
 * CoMotion Tracker — Intensity & Movement Module
 *
 * Algorithm (spec section 2):
 *
 *   accelMag = sqrt(ax² + ay² + az²)
 *   gyroMag  = sqrt(gx² + gy² + gz²)
 *   accelActivity = max(0, accelMag - 1.0)   // subtract 1g gravity
 *   gyroActivity  = gyroMag / 100.0           // normalize 0-500 dps → 0-5
 *   intensityRaw  = 0.7 × accelActivity + 0.3 × gyroActivity
 *
 * EMAs updated at 104 Hz:
 *   1-second:  alpha = 0.019    = 2 / (1×104 + 1)
 *   1-minute:  alpha = 0.00032  = 2 / (60×104 + 1)
 *
 * 10-minute rolling average: circular buffer of 1200 floats at 2 Hz.
 *
 * Movement counter: rising edge of intensityCurrent crossing 0.3.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <math.h>

#include "intensity.h"

LOG_MODULE_REGISTER(intensity, CONFIG_LOG_DEFAULT_LEVEL);

/* ─── Constants (from spec section 13) ─── */
#define ACCEL_WEIGHT        0.7f
#define GYRO_WEIGHT         0.3f
#define GYRO_SCALE          100.0f

#define EMA_FAST_ALPHA      0.019f      /* 1-second at 104 Hz */
#define EMA_SLOW_ALPHA      0.00032f    /* 1-minute at 104 Hz */

#define HISTORY_SIZE        1200        /* 10 min at 2 Hz */
#define MOVEMENT_THRESHOLD  0.3f

/* ─── State ─── */
static float intensity_current;                /* 1-second EMA */
static float intensity_1min;                   /* 1-minute EMA */

static float intensity_history[HISTORY_SIZE];  /* 10-min circular buffer */
static float intensity_10min_sum;
static uint16_t history_index;
static uint16_t history_count;                 /* actual samples (ramps to 1200) */

static uint16_t move_count;
static bool was_above;                         /* for rising-edge detector */

/* ─── Public API ─── */

void intensity_init(void)
{
	intensity_reset();
	LOG_INF("Intensity tracker ready (accel %.0f%% + gyro %.0f%%)",
		(double)(ACCEL_WEIGHT * 100.0f),
		(double)(GYRO_WEIGHT * 100.0f));
}

void intensity_reset(void)
{
	intensity_current = 0.0f;
	intensity_1min = 0.0f;
	intensity_10min_sum = 0.0f;
	history_index = 0;
	history_count = 0;
	move_count = 0;
	was_above = false;
	memset(intensity_history, 0, sizeof(intensity_history));
	LOG_INF("Intensity session reset");
}

void intensity_feed(float ax_g, float ay_g, float az_g,
		    float gx_dps, float gy_dps, float gz_dps)
{
	/* Acceleration magnitude (in g) */
	float accel_mag = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);

	/* Gyroscope magnitude (in dps) */
	float gyro_mag = sqrtf(gx_dps * gx_dps + gy_dps * gy_dps + gz_dps * gz_dps);

	/* Activity: subtract 1g from accel, normalize gyro */
	float accel_activity = (accel_mag > 1.0f) ? (accel_mag - 1.0f) : 0.0f;
	float gyro_activity = gyro_mag / GYRO_SCALE;

	/* Weighted raw intensity */
	float raw = ACCEL_WEIGHT * accel_activity + GYRO_WEIGHT * gyro_activity;

	/* 1-second EMA */
	intensity_current = intensity_current * (1.0f - EMA_FAST_ALPHA)
			  + raw * EMA_FAST_ALPHA;

	/* 1-minute EMA */
	intensity_1min = intensity_1min * (1.0f - EMA_SLOW_ALPHA)
		       + raw * EMA_SLOW_ALPHA;

	/* Movement counter: rising edge above threshold */
	bool is_above = (intensity_current > MOVEMENT_THRESHOLD);

	if (is_above && !was_above) {
		move_count++;
	}
	was_above = is_above;
}

void intensity_sample_history(void)
{
	/* Subtract old value, add new */
	intensity_10min_sum -= intensity_history[history_index];
	intensity_history[history_index] = intensity_current;
	intensity_10min_sum += intensity_current;

	if (history_count < HISTORY_SIZE) {
		history_count++;
	}

	history_index = (history_index + 1) % HISTORY_SIZE;
}

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
	if (history_count == 0) {
		return 0.0f;
	}
	return intensity_10min_sum / (float)history_count;
}

uint16_t intensity_get_move_count(void)
{
	return move_count;
}
