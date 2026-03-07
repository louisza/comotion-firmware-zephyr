/*
 * CoMotion Tracker — Dual-Trigger Impact Detection
 *
 * Algorithm (spec section 3):
 *
 * IMU trigger (104 Hz):
 *   if accelMag > 3.0g → lastImuSpike = now
 *
 * Audio trigger (10 Hz):
 *   if peak > baseline × 8.0 AND peak > 3000 → lastAudioSpike = now
 *
 * Confirmation (10 Hz, in audio path):
 *   if (now - lastImuSpike < 200ms) AND (now - lastAudioSpike < 200ms)
 *   AND (now - lastImpact > 3000ms):
 *     → CONFIRMED IMPACT
 *
 * BLE flag stays set for 2 seconds after last confirmed impact.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "impact.h"

LOG_MODULE_REGISTER(impact, CONFIG_LOG_DEFAULT_LEVEL);

/* ─── Constants (spec section 13) ─── */
#define ACCEL_THRESHOLD_G       3.0f     /* g for IMU trigger */
#define AUDIO_THRESHOLD_RATIO   8.0f     /* peak / baseline ratio */
#define AUDIO_MIN_PEAK          3000     /* absolute minimum peak */
#define CONFIRM_WINDOW_MS       200      /* both triggers within this */
#define DEBOUNCE_MS             3000     /* minimum between impacts */
#define FLAG_DURATION_MS        2000     /* BLE flag auto-clear time */

/* ─── State ─── */
static int64_t last_imu_spike;       /* timestamp of last IMU threshold crossing */
static int64_t last_audio_spike;     /* timestamp of last audio trigger */
static int64_t last_impact;          /* timestamp of last confirmed impact */
static uint8_t impact_count;

void impact_init(void)
{
	impact_reset();
	LOG_INF("Impact detector ready (IMU>%.1fg + audio>%.0f× baseline, "
		"window=%dms, debounce=%dms)",
		(double)ACCEL_THRESHOLD_G, (double)AUDIO_THRESHOLD_RATIO,
		CONFIRM_WINDOW_MS, DEBOUNCE_MS);
}

void impact_reset(void)
{
	last_imu_spike = 0;
	last_audio_spike = 0;
	last_impact = 0;
	impact_count = 0;
	LOG_INF("Impact stats reset");
}

void impact_feed_imu(float accel_mag_g)
{
	if (accel_mag_g > ACCEL_THRESHOLD_G) {
		last_imu_spike = k_uptime_get();
	}
}

void impact_feed_audio(uint16_t peak, float baseline)
{
	int64_t now = k_uptime_get();

	/* Audio trigger: relative AND absolute threshold */
	bool above_relative = ((float)peak > baseline * AUDIO_THRESHOLD_RATIO);
	bool above_absolute = (peak > AUDIO_MIN_PEAK);

	if (above_relative && above_absolute) {
		last_audio_spike = now;
	}

	/* Dual-trigger confirmation */
	bool imu_confirmed = (last_imu_spike > 0) &&
			     ((now - last_imu_spike) < CONFIRM_WINDOW_MS);
	bool audio_confirmed = (last_audio_spike > 0) &&
			       ((now - last_audio_spike) < CONFIRM_WINDOW_MS);

	if (imu_confirmed && audio_confirmed &&
	    (now - last_impact) > DEBOUNCE_MS) {
		/* CONFIRMED IMPACT */
		impact_count++;
		last_impact = now;
		last_imu_spike = 0;     /* prevent double-fire from same spike */
		last_audio_spike = 0;
		LOG_INF("IMPACT #%u confirmed (dual-trigger)", impact_count);
	}
}

uint8_t impact_get_count(void)
{
	return impact_count;
}

bool impact_is_recent(void)
{
	if (last_impact == 0) {
		return false;
	}
	return (k_uptime_get() - last_impact) < FLAG_DURATION_MS;
}

int64_t impact_get_last_time(void)
{
	return last_impact;
}
