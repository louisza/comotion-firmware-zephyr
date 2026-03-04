/*
 * ═══════════════════════════════════════════════════════════════
 * CoMotion Tracker — Sensors (IMU, PDM Microphone, Battery ADC)
 * ═══════════════════════════════════════════════════════════════
 */

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <nrfx_pdm.h>
#include <math.h>

#include "common.h"
#include "sensors.h"
#include "ble.h"

LOG_MODULE_REGISTER(sensors, LOG_LEVEL_INF);

/* ─── IMU ─── */
static const struct device *imu_dev;
static float gyro_offset_x, gyro_offset_y, gyro_offset_z;
static bool  gyro_calibrated;

/* ─── PDM ─── */
#define PDM_BUFFER_SIZE 512
static int16_t pdm_buf_a[PDM_BUFFER_SIZE];
static int16_t pdm_buf_b[PDM_BUFFER_SIZE];
static volatile int16_t *pdm_ready_buf;
static volatile int      pdm_ready_len;
static volatile bool     pdm_data_ready;

/* Impact detection state */
static volatile int64_t last_imu_spike;
static volatile int64_t last_audio_spike;
static volatile int64_t last_audio_impact;

/* ─── Battery ADC ─── */
/* Using direct NRF_SAADC registers (same as Arduino firmware) */

/* ═══════════════════════════════════════════════════════════════
 * PDM Callback
 * ═══════════════════════════════════════════════════════════════ */

static void pdm_handler(nrfx_pdm_evt_t const *evt)
{
	if (evt->buffer_released) {
		pdm_ready_buf = evt->buffer_released;
		pdm_ready_len = PDM_BUFFER_SIZE;
		pdm_data_ready = true;
	}

	/* Queue next buffer */
	if (evt->buffer_requested) {
		int16_t *next = (evt->buffer_released == pdm_buf_a)
				? pdm_buf_b : pdm_buf_a;
		nrfx_pdm_buffer_set(next);
	}
}

/* ═══════════════════════════════════════════════════════════════
 * Init
 * ═══════════════════════════════════════════════════════════════ */

int sensors_init(void)
{
	int err = 0;

	/* ─── IMU ─── */
	imu_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);
	if (!device_is_ready(imu_dev)) {
		LOG_ERR("IMU device not ready");
		return -ENODEV;
	}

	/* Configure ODR to 104 Hz (closest match) */
	struct sensor_value odr = { .val1 = 104, .val2 = 0 };
	sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ,
			SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
	sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ,
			SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);

	/* Set accel range to ±4g */
	struct sensor_value range = { .val1 = 4, .val2 = 0 };
	sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ,
			SENSOR_ATTR_FULL_SCALE, &range);

	LOG_INF("[IMU] LSM6DS3 ready (104 Hz, ±4g)");

	/* ─── PDM Microphone ─── */
	/* Enable mic power (P1.10 HIGH) — board DTS has regulator,
	 * but ensure it's on */
	nrfx_pdm_config_t pdm_cfg = NRFX_PDM_DEFAULT_CONFIG(
		/* CLK */ NRF_PDM_PSEL_DISCONNECTED,
		/* DIN */ NRF_PDM_PSEL_DISCONNECTED);

	/* Use the actual XIAO Sense PDM pins from the board DTS:
	 * CLK = P0.01, DIN = P0.00 (check board schematic) */
	pdm_cfg.pinout.clk = NRF_GPIO_PIN_MAP(0, 1);
	pdm_cfg.pinout.din = NRF_GPIO_PIN_MAP(0, 0);
	pdm_cfg.clock_freq = NRF_PDM_FREQ_1032K;
	pdm_cfg.ratio = NRF_PDM_RATIO_64X;
	pdm_cfg.gain_l = NRF_PDM_GAIN_DEFAULT;
	pdm_cfg.gain_r = NRF_PDM_GAIN_DEFAULT;
	pdm_cfg.mode = NRF_PDM_MODE_MONO;
	pdm_cfg.edge = NRF_PDM_EDGE_LEFTFALLING;

	err = nrfx_pdm_init(&pdm_cfg, pdm_handler);
	if (err != NRFX_SUCCESS) {
		LOG_ERR("[PDM] Init failed: %d", err);
		return -EIO;
	}

	err = nrfx_pdm_start();
	if (err != NRFX_SUCCESS) {
		LOG_ERR("[PDM] Start failed: %d", err);
		return -EIO;
	}
	nrfx_pdm_buffer_set(pdm_buf_a);

	LOG_INF("[PDM] Microphone ready (16 kHz mono)");

	/* ─── Battery ADC (direct register, same as Arduino) ─── */
	/* Enable VBAT pin P0.14 as output LOW */
	NRF_P0->DIRSET = (1 << 14);
	NRF_P0->OUTCLR = (1 << 14);

	NRF_SAADC->ENABLE = 0;
	NRF_SAADC->CH[0].PSELP = 8; /* AIN7 = P0.31 */
	NRF_SAADC->CH[0].PSELN = 0;
	NRF_SAADC->CH[0].CONFIG =
		(SAADC_CH_CONFIG_RESP_Bypass    << SAADC_CH_CONFIG_RESP_Pos)   |
		(SAADC_CH_CONFIG_RESN_Bypass    << SAADC_CH_CONFIG_RESN_Pos)   |
		(SAADC_CH_CONFIG_GAIN_Gain1_6   << SAADC_CH_CONFIG_GAIN_Pos)   |
		(SAADC_CH_CONFIG_REFSEL_Internal << SAADC_CH_CONFIG_REFSEL_Pos) |
		(SAADC_CH_CONFIG_TACQ_10us      << SAADC_CH_CONFIG_TACQ_Pos)   |
		(SAADC_CH_CONFIG_MODE_SE        << SAADC_CH_CONFIG_MODE_Pos);
	NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_12bit;
	NRF_SAADC->OVERSAMPLE = SAADC_OVERSAMPLE_OVERSAMPLE_Bypass;
	NRF_SAADC->ENABLE = 1;

	LOG_INF("[BAT] ADC ready");

	return 0;
}

/* ═══════════════════════════════════════════════════════════════
 * IMU Reading (called from 104 Hz timer)
 * ═══════════════════════════════════════════════════════════════ */

void sensors_read_imu(void)
{
	struct sensor_value accel[3], gyro[3];

	if (sensor_sample_fetch(imu_dev) < 0) {
		return;
	}

	sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
	sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_XYZ, gyro);

	/* Convert from m/s² to g (Zephyr reports m/s²) */
	g_imu.ax = sensor_value_to_float(&accel[0]) / 9.80665f;
	g_imu.ay = sensor_value_to_float(&accel[1]) / 9.80665f;
	g_imu.az = sensor_value_to_float(&accel[2]) / 9.80665f;

	/* Convert from rad/s to dps (Zephyr reports rad/s) */
	g_imu.gx = sensor_value_to_float(&gyro[0]) * 57.2957795f - gyro_offset_x;
	g_imu.gy = sensor_value_to_float(&gyro[1]) * 57.2957795f - gyro_offset_y;
	g_imu.gz = sensor_value_to_float(&gyro[2]) * 57.2957795f - gyro_offset_z;

	/* Track high-g events for dual-trigger impact detection */
	float accel_mag = sqrtf(g_imu.ax * g_imu.ax + g_imu.ay * g_imu.ay +
				g_imu.az * g_imu.az);
	if (accel_mag > IMPACT_ACCEL_THRESHOLD) {
		last_imu_spike = k_uptime_get();
	}
}

/* ═══════════════════════════════════════════════════════════════
 * Gyro Calibration
 * ═══════════════════════════════════════════════════════════════ */

#define GYRO_CAL_TIMEOUT_MS    10000
#define GYRO_CAL_STILL_TIME_MS 1000
#define GYRO_CAL_SAMPLES       100
#define GYRO_CAL_THRESHOLD_DPS 5.0f

bool sensors_calibrate_gyro(void)
{
	int64_t start = k_uptime_get();
	int64_t still_start = 0;
	bool was_still = false;

	/* Phase 1: Wait for stillness */
	while (k_uptime_get() - start < GYRO_CAL_TIMEOUT_MS) {
		struct sensor_value gyro[3];
		sensor_sample_fetch(imu_dev);
		sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_XYZ, gyro);

		float gx = sensor_value_to_float(&gyro[0]) * 57.2957795f;
		float gy = sensor_value_to_float(&gyro[1]) * 57.2957795f;
		float gz = sensor_value_to_float(&gyro[2]) * 57.2957795f;
		float mag = sqrtf(gx * gx + gy * gy + gz * gz);

		if (mag < GYRO_CAL_THRESHOLD_DPS) {
			if (!was_still) {
				still_start = k_uptime_get();
				was_still = true;
			} else if (k_uptime_get() - still_start >= GYRO_CAL_STILL_TIME_MS) {
				break;
			}
		} else {
			was_still = false;
		}
		k_msleep(10);
	}

	if (!was_still || (k_uptime_get() - still_start < GYRO_CAL_STILL_TIME_MS)) {
		return false;
	}

	/* Phase 2: Collect and average */
	float sum_x = 0, sum_y = 0, sum_z = 0;
	for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
		struct sensor_value gyro[3];
		sensor_sample_fetch(imu_dev);
		sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_XYZ, gyro);
		sum_x += sensor_value_to_float(&gyro[0]) * 57.2957795f;
		sum_y += sensor_value_to_float(&gyro[1]) * 57.2957795f;
		sum_z += sensor_value_to_float(&gyro[2]) * 57.2957795f;
		k_msleep(5);
	}

	gyro_offset_x = sum_x / GYRO_CAL_SAMPLES;
	gyro_offset_y = sum_y / GYRO_CAL_SAMPLES;
	gyro_offset_z = sum_z / GYRO_CAL_SAMPLES;
	gyro_calibrated = true;

	LOG_INF("[CAL] Offsets: %.2f, %.2f, %.2f dps",
		(double)gyro_offset_x, (double)gyro_offset_y, (double)gyro_offset_z);
	return true;
}

/* ═══════════════════════════════════════════════════════════════
 * Audio Processing (called from 10 Hz timer)
 * ═══════════════════════════════════════════════════════════════ */

void sensors_process_audio(void)
{
	if (!pdm_data_ready || pdm_ready_len == 0) return;
	pdm_data_ready = false;

	const int16_t *buf = (const int16_t *)pdm_ready_buf;
	int count = pdm_ready_len;

	long sum_squares = 0;
	int peak = 0;
	int zero_crossings = 0;
	int prev_sample = 0;

	for (int i = 0; i < count; i++) {
		int sample = buf[i];
		sum_squares += (long)sample * sample;
		int abs_sample = (sample < 0) ? -sample : sample;
		if (abs_sample > peak) peak = abs_sample;

		if (i > 0) {
			if ((prev_sample >= 0 && sample < 0) ||
			    (prev_sample < 0 && sample >= 0)) {
				zero_crossings++;
			}
		}
		prev_sample = sample;
	}

	g_audio_rms = sqrtf((float)sum_squares / count);
	g_audio_peak = peak;
	g_audio_zcr = zero_crossings;

	bool above_relative = (peak > g_audio_baseline * AUDIO_IMPACT_THRESHOLD);
	bool above_absolute = (peak > AUDIO_IMPACT_MIN_PEAK);
	bool audio_impact = above_relative && above_absolute;

	if (!audio_impact) {
		g_audio_baseline = AUDIO_BASELINE_ALPHA * g_audio_rms +
				   (1 - AUDIO_BASELINE_ALPHA) * g_audio_baseline;
	}

	int64_t now = k_uptime_get();

	if (audio_impact) {
		last_audio_spike = now;
	}

	/* Dual-trigger: audio + IMU within window */
	bool imu_confirmed = (last_imu_spike > 0) &&
			     (now - last_imu_spike < IMPACT_CONFIRM_WINDOW_MS);
	bool audio_confirmed = (last_audio_spike > 0) &&
			       (now - last_audio_spike < IMPACT_CONFIRM_WINDOW_MS);

	if (imu_confirmed && audio_confirmed &&
	    (now - last_audio_impact > AUDIO_IMPACT_DEBOUNCE_MS)) {
		last_audio_impact = now;
		last_imu_spike = 0;
		last_audio_spike = 0;
		ble_set_impact_flag();
		if (g_is_logging) {
			extern void sdcard_mark_event(const char *name);
			sdcard_mark_event("IMPACT");
		}
		LOG_INF("[IMPACT] Confirmed! Peak=%d Baseline=%d",
			peak, (int)g_audio_baseline);
	}
}

/* ═══════════════════════════════════════════════════════════════
 * Battery (direct SAADC — identical to Arduino firmware)
 * ═══════════════════════════════════════════════════════════════ */

float sensors_read_battery(void)
{
	static volatile int16_t result;

	NRF_SAADC->RESULT.PTR = (uint32_t)&result;
	NRF_SAADC->RESULT.MAXCNT = 1;

	NRF_SAADC->TASKS_START = 1;
	while (!NRF_SAADC->EVENTS_STARTED);
	NRF_SAADC->EVENTS_STARTED = 0;

	NRF_SAADC->TASKS_SAMPLE = 1;
	while (!NRF_SAADC->EVENTS_END);
	NRF_SAADC->EVENTS_END = 0;

	NRF_SAADC->TASKS_STOP = 1;
	while (!NRF_SAADC->EVENTS_STOPPED);
	NRF_SAADC->EVENTS_STOPPED = 0;

	int adc_val = (result < 0) ? 0 : (int)result;
	return (float)adc_val / 4095.0f * 3.6f * 3.0f;
}
