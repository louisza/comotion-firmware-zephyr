/*
 * CoMotion Tracker — Full Integrated Firmware
 *
 * XIAO BLE Sense (nRF52840) wearable impact/motion tracker.
 *
 * Architecture:
 *   104 Hz timer:  IMU read → intensity → impact(IMU) → SD log (if logging)
 *   10 Hz thread:  Audio PDM burst → RMS/peak/ZCR → baseline → impact(audio)
 *   2 Hz (every 52 cycles): BLE advertising update, LED, intensity history
 *   30s:           Battery ADC sample
 *
 * BLE:
 *   Broadcast: 20-byte manufacturer data (no connection needed for monitoring)
 *   NUS:       Text commands for start/stop logging, status, etc.
 *
 * LED:
 *   Booting       → Blue solid
 *   Idle          → Blue slow blink (500ms)
 *   Logging+GPS   → Green solid
 *   Logging noGPS → Green fast blink (250ms)
 *   Low battery   → Red solid (overrides)
 *   Event marked  → Blue 50ms flash
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <limits.h>

#include "intensity.h"
#include "impact.h"
#include "audio.h"
#include "sdcard.h"
#include "ble_adv.h"
#include "gps.h"
#include "battery.h"
#include "filter.h"
#include "device_id.h"

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

#define FIRMWARE_VERSION "1.0.0"

/* ─── Timing constants (spec section 12–13) ─── */
#define IMU_PERIOD_US       9615    /* 1/104 Hz ≈ 9.615 ms */
#define FAST_CYCLES         21      /* 104/5 ≈ 21 → ~5 Hz (BLE updates) */
#define SLOW_CYCLES         52      /* 104/2 = 52 → 2 Hz */
#define LOG_CYCLES          10      /* 104/10 ≈ 10 → ~10 Hz SD card logging */
#define BATT_CYCLES         3120    /* 104 × 30 = 3120 → every 30s */
#define GPS_STALE_MS        2000    /* GPS fix older than this → stale (generous for jitter) */
#define GYRO_CAL_TIMEOUT_MS 10000
#define GYRO_CAL_STILL_MS   1000
#define GYRO_CAL_SAMPLES    100
#define GYRO_CAL_THRESH_DPS 5.0f
#define EVENT_FLASH_MS      50      /* Blue LED flash for event */

/* ─── LEDs (active LOW, handled by DTS flags) ─── */
static const struct gpio_dt_spec led_red   = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec led_blue  = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);

/* ─── IMU ─── */
static const struct device *imu;
static bool imu_ok;

/* Gyro calibration offsets (spec section 9) */
static float gyro_off_x, gyro_off_y, gyro_off_z;

/* ─── 104 Hz timer + semaphore ─── */
K_SEM_DEFINE(imu_tick, 0, 1);

static void imu_timer_handler(struct k_timer *t)
{
	ARG_UNUSED(t);
	k_sem_give(&imu_tick);
}

K_TIMER_DEFINE(imu_timer, imu_timer_handler, NULL);

/* ─── Session state ─── */
static int64_t log_start_time;
static float max_speed_kmh;

/* ─── LED state ─── */
static int64_t event_flash_off;

/* ─── IMU low-pass filters (per-axis EMA, α=0.3 → ~5 Hz cutoff at 104 Hz) ─── */
static struct filter_ema filt_ax = FILTER_EMA_INIT(0.3f);
static struct filter_ema filt_ay = FILTER_EMA_INIT(0.3f);
static struct filter_ema filt_az = FILTER_EMA_INIT(0.3f);
static struct filter_ema filt_gx = FILTER_EMA_INIT(0.3f);
static struct filter_ema filt_gy = FILTER_EMA_INIT(0.3f);
static struct filter_ema filt_gz = FILTER_EMA_INIT(0.3f);

/* ─── Deferred NUS command processing ─── */
static char pending_cmd[64];
static K_SEM_DEFINE(cmd_sem, 0, 1);

/* ─── Sensor value → float converters ─── */

/**
 * Zephyr accel sensor_value (m/s²) → g.
 * 1g = 9.80665 m/s².
 */
static inline float sv_to_g(const struct sensor_value *sv)
{
	float ms2 = (float)sv->val1 + (float)sv->val2 / 1000000.0f;
	return ms2 / 9.80665f;
}

/**
 * Zephyr gyro sensor_value (rad/s) → degrees/s.
 * 1 rad/s = 57.2958 deg/s.
 */
static inline float sv_to_dps(const struct sensor_value *sv)
{
	float rads = (float)sv->val1 + (float)sv->val2 / 1000000.0f;
	return rads * 57.2958f;
}

/* ─── IMU Init ─── */

static int init_imu(void)
{
	imu = DEVICE_DT_GET_ONE(st_lsm6dsl);
	if (!device_is_ready(imu)) {
		LOG_ERR("IMU: device not ready");
		return -ENODEV;
	}

	struct sensor_value val;

	val.val1 = 16; val.val2 = 0;
	sensor_attr_set(imu, SENSOR_CHAN_ACCEL_XYZ,
			SENSOR_ATTR_FULL_SCALE, &val);

	val.val1 = 1000; val.val2 = 0;
	sensor_attr_set(imu, SENSOR_CHAN_GYRO_XYZ,
			SENSOR_ATTR_FULL_SCALE, &val);

	val.val1 = 104; val.val2 = 0;
	sensor_attr_set(imu, SENSOR_CHAN_ACCEL_XYZ,
			SENSOR_ATTR_SAMPLING_FREQUENCY, &val);
	sensor_attr_set(imu, SENSOR_CHAN_GYRO_XYZ,
			SENSOR_ATTR_SAMPLING_FREQUENCY, &val);

	LOG_INF("IMU: LSM6DS3TR-C ready (16g, 1000dps, 104Hz)");
	return 0;
}

/* ─── Gyro Calibration (spec section 9) ─── */

static void gyro_calibrate(void)
{
	struct sensor_value gyro[3];
	int64_t start = k_uptime_get();
	int64_t still_start = 0;
	bool was_still = false;

	LOG_INF("Gyro cal: waiting for stillness...");

	/* Phase 1: Wait for stillness */
	while ((k_uptime_get() - start) < GYRO_CAL_TIMEOUT_MS) {
		if (sensor_sample_fetch(imu) != 0) {
			k_msleep(10);
			continue;
		}
		sensor_channel_get(imu, SENSOR_CHAN_GYRO_X, &gyro[0]);
		sensor_channel_get(imu, SENSOR_CHAN_GYRO_Y, &gyro[1]);
		sensor_channel_get(imu, SENSOR_CHAN_GYRO_Z, &gyro[2]);

		float gx = sv_to_dps(&gyro[0]);
		float gy = sv_to_dps(&gyro[1]);
		float gz = sv_to_dps(&gyro[2]);
		float mag = sqrtf(gx * gx + gy * gy + gz * gz);

		if (mag < GYRO_CAL_THRESH_DPS) {
			if (!was_still) {
				still_start = k_uptime_get();
				was_still = true;
			} else if ((k_uptime_get() - still_start) >=
				   GYRO_CAL_STILL_MS) {
				goto phase2;
			}
		} else {
			was_still = false;
		}
		k_msleep(10);
	}

	LOG_WRN("Gyro cal: timeout — using zero offsets");
	return;

phase2:
	LOG_INF("Gyro cal: collecting %d samples...", GYRO_CAL_SAMPLES);

	float sum_x = 0, sum_y = 0, sum_z = 0;
	int count = 0;

	for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
		if (sensor_sample_fetch(imu) == 0) {
			sensor_channel_get(imu, SENSOR_CHAN_GYRO_X, &gyro[0]);
			sensor_channel_get(imu, SENSOR_CHAN_GYRO_Y, &gyro[1]);
			sensor_channel_get(imu, SENSOR_CHAN_GYRO_Z, &gyro[2]);
			sum_x += sv_to_dps(&gyro[0]);
			sum_y += sv_to_dps(&gyro[1]);
			sum_z += sv_to_dps(&gyro[2]);
			count++;
		}
		k_msleep(5);
	}

	if (count > 0) {
		gyro_off_x = sum_x / (float)count;
		gyro_off_y = sum_y / (float)count;
		gyro_off_z = sum_z / (float)count;
		LOG_INF("Gyro cal: offset (%.2f, %.2f, %.2f) dps from %d samples",
			(double)gyro_off_x, (double)gyro_off_y,
			(double)gyro_off_z, count);
	}
}

/* ─── NUS Receive Callback (runs on BT thread — must be lightweight!) ─── */

static void nus_rx_handler(struct bt_conn *conn, const uint8_t *data,
			   uint16_t len)
{
	ARG_UNUSED(conn);

	/* Just stash the command and signal the main loop to process it.
	 * The BT RX thread has a tiny stack — SD card, snprintf, etc.
	 * would overflow it instantly.
	 */
	uint16_t copy = MIN(len, sizeof(pending_cmd) - 1);

	memcpy(pending_cmd, data, copy);
	pending_cmd[copy] = '\0';

	/* Strip trailing whitespace/newline */
	while (copy > 0 && (pending_cmd[copy - 1] == '\n' ||
			    pending_cmd[copy - 1] == '\r' ||
			    pending_cmd[copy - 1] == ' ')) {
		pending_cmd[--copy] = '\0';
	}

	k_sem_give(&cmd_sem);
}

/* ─── Command Processor (runs on main thread with 8KB stack) ─── */

static void process_command(const char *cmd)
{
	LOG_INF("NUS CMD: \"%s\"", cmd);

	if (strcmp(cmd, "ping") == 0) {
		ble_adv_nus_send("pong\n");
	} else if (strcmp(cmd, "info") == 0) {
		ble_adv_nus_send("CoMotion v2.0.0\n");
	} else if (strcmp(cmd, "bat") == 0) {
		char buf[32];

		snprintf(buf, sizeof(buf), "BAT:%d.%02dV\n",
			 battery_millivolts() / 1000,
			 (battery_millivolts() % 1000) / 10);
		ble_adv_nus_send(buf);
	} else if (strcmp(cmd, "start") == 0) {
		if (sdcard_is_logging()) {
			ble_adv_nus_send("ERR:Already logging\n");
			return;
		}
		/* Reset session stats */
		intensity_reset();
		impact_reset();
		max_speed_kmh = 0;
		log_start_time = k_uptime_get();

		int ret = sdcard_start_logging();

		if (ret == 0) {
			ble_adv_nus_send("OK:Logging started\n");
		} else {
			ble_adv_nus_send("ERR:Failed to start logging\n");
		}
	} else if (strcmp(cmd, "stop") == 0) {
		sdcard_stop_logging();
		ble_adv_nus_send("OK:Logging stopped\n");
	} else if (strcmp(cmd, "status") == 0) {
		char buf[200];
		int pos = 0;

		pos += snprintf(buf + pos, sizeof(buf) - pos, "STATUS:%s\n",
				sdcard_is_logging() ? "LOGGING" : "IDLE");

		struct gps_data sg = {0};

		gps_get_data(&sg);
		pos += snprintf(buf + pos, sizeof(buf) - pos,
				"GPS:%s SATS:%u\n",
				(sg.has_fix && sg.satellites >= 4)
					? "OK" : "NO_FIX",
				sg.satellites);

		pos += snprintf(buf + pos, sizeof(buf) - pos,
				"BAT:%d.%02dV\n",
				battery_millivolts() / 1000,
				(battery_millivolts() % 1000) / 10);

		if (sdcard_is_logging()) {
			int64_t elapsed = (k_uptime_get() - log_start_time) / 1000;

			pos += snprintf(buf + pos, sizeof(buf) - pos,
					"FILE:%s\n",
					sdcard_get_filename());
			pos += snprintf(buf + pos, sizeof(buf) - pos,
					"SAMPLES:%u\n",
					sdcard_get_sample_count());
			pos += snprintf(buf + pos, sizeof(buf) - pos,
					"ELAPSED:%lld\n", elapsed);
		}
		ble_adv_nus_send(buf);
	} else if (strncmp(cmd, "event:", 6) == 0) {
		sdcard_mark_event(cmd + 6);
		ble_adv_nus_send("OK:Event marked\n");
		/* Blue LED flash */
		gpio_pin_set_dt(&led_blue, 1);
		event_flash_off = k_uptime_get() + EVENT_FLASH_MS;
	} else if (strcmp(cmd, "bat") == 0) {
		char msg[32];
		snprintf(msg, sizeof(msg), "BAT:%d.%02dV\n",
			 battery_millivolts() / 1000,
			 (battery_millivolts() % 1000) / 10);
		ble_adv_nus_send(msg);
	} else if (strcmp(cmd, "info") == 0) {
		char msg[64];
		snprintf(msg, sizeof(msg), "CoMotion v%s [%s] (Zephyr)\n",
			 FIRMWARE_VERSION, device_id_full());
		ble_adv_nus_send(msg);
	} else if (strcmp(cmd, "ping") == 0) {
		ble_adv_nus_send("pong\n");
	} else if (strcmp(cmd, "focus") == 0) {
		ble_adv_set_focus(true);
		ble_adv_nus_send("OK:Focus mode ON (60s)\n");
	} else if (strcmp(cmd, "normal") == 0) {
		ble_adv_set_focus(false);
		ble_adv_nus_send("OK:Normal mode\n");
	} else if (strcmp(cmd, "gps") == 0) {
		struct gps_data g = {0};

		gps_get_data(&g);
		char buf[256];
		int pos = 0;

		pos += gps_format(&g, buf + pos, sizeof(buf) - pos);
		pos += snprintf(buf + pos, sizeof(buf) - pos, "\n");
		pos += snprintf(buf + pos, sizeof(buf) - pos,
				"HDOP:%u.%u UPD:%u\n",
				g.hdop / 1000, (g.hdop % 1000) / 100,
				g.update_count);
		ble_adv_nus_send(buf);
	} else if (strcmp(cmd, "LIST") == 0) {
		sdcard_handle_list();
	} else if (strncmp(cmd, "DUMP:", 5) == 0) {
		sdcard_handle_dump(cmd + 5);
	} else if (strcmp(cmd, "DUMP_LATEST") == 0) {
		sdcard_handle_dump_latest();
	} else if (strncmp(cmd, "DELETE:", 7) == 0) {
		sdcard_handle_delete(cmd + 7);
	} else if (strcmp(cmd, "STATUS") == 0) {
		sdcard_handle_status_cmd();
	} else if (strcmp(cmd, "ABORT") == 0) {
		sdcard_handle_abort();
	} else if (strncmp(cmd, "ACK:", 4) == 0) {
		/* ACK from app during transfer — pacing handled in dump */
	} else {
		ble_adv_nus_send("ERR:Unknown command\n");
	}
}

/* ─── LED Update (spec section 11, called at 2 Hz) ─── */

static void update_leds(uint32_t cycle)
{
	int64_t now = k_uptime_get();

	/* Event flash auto-off */
	if (event_flash_off && now >= event_flash_off) {
		gpio_pin_set_dt(&led_blue, 0);
		event_flash_off = 0;
	}

	/* Low battery overrides everything */
	if (battery_millivolts() > 0 && battery_millivolts() < 3300) {
		gpio_pin_set_dt(&led_red, 1);
		gpio_pin_set_dt(&led_green, 0);
		gpio_pin_set_dt(&led_blue, 0);
		return;
	}
	gpio_pin_set_dt(&led_red, 0);

	if (sdcard_is_logging()) {
		gpio_pin_set_dt(&led_blue, 0);

		/* Logging: green behavior depends on GPS */
		struct gps_data lg = {0};
		bool gps_fix = (gps_get_data(&lg) == 0
				&& lg.has_fix && lg.satellites >= 4
				&& lg.last_fix_ms > 0
				&& (now - lg.last_fix_ms) < GPS_STALE_MS);

		if (gps_fix) {
			/* Green solid */
			gpio_pin_set_dt(&led_green, 1);
		} else {
			/* Green fast blink (250ms at 2Hz → toggle every call) */
			gpio_pin_set_dt(&led_green, (cycle / SLOW_CYCLES) & 1);
		}
	} else {
		gpio_pin_set_dt(&led_green, 0);

		/* Idle: blue slow blink (500ms at 2Hz → toggle every call) */
		if (!event_flash_off) {
			gpio_pin_set_dt(&led_blue, (cycle / SLOW_CYCLES) & 1);
		}
	}
}

/* ─── Build BLE Advertising Packet (spec section 1) ─── */

static void build_ble_packet(struct ble_packet *pkt)
{
	memset(pkt, 0, sizeof(*pkt));

	/*
	 * Single atomic GPS snapshot — all fields + timing are consistent.
	 * Pre-zero so fields are safe even if gps_get_data() returns -ENODATA.
	 */
	struct gps_data gps = {0};
	bool gps_ok = (gps_get_data(&gps) == 0);
	int64_t now = k_uptime_get();

	/* Fix age based on last VALID fix, not last callback */
	int64_t fix_age_ms = (gps_ok && gps.last_fix_ms > 0)
			   ? (now - gps.last_fix_ms) : INT64_MAX;
	bool have_gps = gps_ok && gps.has_fix
		     && fix_age_ms < GPS_STALE_MS
		     && gps.satellites >= 4;

	/* Status flags */
	uint8_t flags = 0;

	if (sdcard_is_logging()) {
		flags |= STATUS_LOGGING;
	}

	if (have_gps) {
		flags |= STATUS_GPS_FIX;
	}

	if (battery_millivolts() > 0 && battery_millivolts() < 3300) {
		flags |= STATUS_BATT_LOW;
	}

	if (impact_is_recent()) {
		flags |= STATUS_IMPACT;
	}

	if (ble_adv_is_connected()) {
		flags |= STATUS_CONNECTED;
	}

	if (ble_adv_is_focus()) {
		flags |= STATUS_FOCUS;
	}

	pkt->status_flags = flags;

	/* Battery */
	pkt->battery_pct = (uint8_t)battery_level_pct();

	/* Intensity: encode per spec */
	float ic = intensity_get_current();
	float i1 = intensity_get_1min();
	float i10 = intensity_get_10min_avg();

	pkt->intensity_1s = (uint8_t)MIN(ic * 100.0f, 255.0f);
	pkt->intensity_1min = (uint8_t)MIN(i1 * 100.0f, 255.0f);

	uint32_t i10_enc = (uint32_t)MIN(i10 * 1000.0f, 65535.0f);

	pkt->intensity_10min = (uint16_t)i10_enc;

	/*
	 * GPS speed — uses Kalman-filtered speed for cleaner BLE output.
	 * Capped at 200 km/h to prevent garbage from corrupting max.
	 * Additional guards: require fix_quality > 0 (valid fix) and HDOP < 5000
	 * (< 5.0) to reject wild speed values from early converging fixes.
	 */
	if (have_gps && gps.fix_quality > 0 && gps.hdop > 0 && gps.hdop < 5000) {
		uint32_t speed_capped = MIN(gps.speed_filt_mmps, 55556u);
		float speed_kmh = (float)speed_capped * 0.0036f;

		pkt->speed_now = (uint8_t)MIN(speed_kmh * 2.0f, 255.0f);

		if (speed_kmh > max_speed_kmh) {
			max_speed_kmh = speed_kmh;
		}
	} else if (have_gps) {
		/* Have fix but low quality — show speed_now but don't update max */
		uint32_t speed_capped = MIN(gps.speed_filt_mmps, 55556u);
		float speed_kmh = (float)speed_capped * 0.0036f;

		pkt->speed_now = (uint8_t)MIN(speed_kmh * 2.0f, 255.0f);
	}
	pkt->speed_max = (uint8_t)MIN(max_speed_kmh * 2.0f, 255.0f);

	/* Impact count */
	pkt->impact_count = impact_get_count();

	/* GPS status: (fix_age_seconds << 4) | sat_count
	 * Uses fix age (not update age) so the app sees how old the POSITION is.
	 * If we've never had a fix, age = 15 (max).
	 */
	if (gps_ok) {
		uint8_t age_s;

		if (gps.last_fix_ms > 0 && fix_age_ms < INT64_MAX) {
			age_s = (uint8_t)MIN(fix_age_ms / 1000, 15);
		} else {
			age_s = 15; /* never had a fix */
		}

		uint8_t sats = (uint8_t)MIN(gps.satellites, 15);

		pkt->gps_status = (age_s << 4) | sats;
	}

	/* Move count */
	pkt->move_count = intensity_get_move_count();

	/* Session seconds: always report time since boot so the app
	 * knows the tracker is alive even when not logging to SD.
	 * When logging, this also serves as the logging duration
	 * (log_start_time is set at boot-level init).
	 */
	{
		int64_t elapsed = now / 1000;

		pkt->session_seconds = (uint16_t)MIN(elapsed, 65535);
	}

	/* Audio peak */
	struct audio_data aud;

	audio_get_data(&aud);
	pkt->audio_peak = (uint8_t)MIN(aud.peak / 128, 255);

	/* GPS position — absolute int32 in deci-microdegrees (×10⁷).
	 * Uses EMA-filtered position for cleaner BLE output.
	 * App decodes: degrees = latitude / 10000000.0
	 * Range: ±214.7° (full globe), precision: ~1.1 cm.
	 * Simple integer divide from nanodegrees: ndeg / 100.
	 */
	if (have_gps) {
		pkt->latitude  = (int32_t)(gps.latitude_filt_ndeg / 100LL);
		pkt->longitude = (int32_t)(gps.longitude_filt_ndeg / 100LL);
	} else {
		pkt->latitude  = GPS_NO_FIX;
		pkt->longitude = GPS_NO_FIX;
	}

	/* Bearing: filtered millidegrees → ×10 (0–3600 for 0.0–360.0°) */
	if (have_gps) {
		pkt->bearing = (uint16_t)MIN(gps.bearing_filt_mdeg / 100u, 3600u);
	}

	/* HDOP: milli-DOP → ×10 (e.g. 1200 → 12 = 1.2 HDOP)
	 * Only meaningful when we have a current, valid fix.
	 */
	if (have_gps) {
		pkt->hdop = (uint8_t)MIN(gps.hdop / 100u, 255u);
	}

	/* Fix quality: always report so app can see 0=no-fix vs 1=SPS etc. */
	if (gps_ok) {
		pkt->fix_quality = gps.fix_quality;
	}
}

/* ─── CSV Line Formatter (spec section 8, called at 104 Hz if logging) ─── */

static void format_and_log_csv(float ax, float ay, float az,
			       float gx, float gy, float gz)
{
	static double last_lat, last_lng, last_lat_filt, last_lng_filt;
	static float last_speed, last_course;
	static uint32_t last_sats;
	static bool have_last_gps;

	char line[256];
	int pos = 0;

	/* timestamp (ms since boot) */
	pos += snprintf(line + pos, sizeof(line) - pos, "%u,",
			(uint32_t)k_uptime_get());

	/* IMU: accel (g, 4dp), gyro (dps, 2dp) */
	pos += snprintf(line + pos, sizeof(line) - pos,
			"%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,",
			(double)ax, (double)ay, (double)az,
			(double)gx, (double)gy, (double)gz);

	/* GPS: lat,lng,speed,course,sats — single atomic read */
	struct gps_data csv_gps = {0};
	bool csv_gps_ok = (gps_get_data(&csv_gps) == 0);
	int64_t csv_fix_age = (csv_gps_ok && csv_gps.last_fix_ms > 0)
			   ? (k_uptime_get() - csv_gps.last_fix_ms)
			   : INT64_MAX;
	bool csv_have_gps = csv_gps_ok && csv_gps.has_fix
			 && csv_fix_age < GPS_STALE_MS
			 && csv_gps.satellites >= 4;

	if (csv_have_gps) {
		/* Fresh fix — update cache and log with stale=0 */
		last_lat = (double)csv_gps.latitude_ndeg / 1e9;
		last_lng = (double)csv_gps.longitude_ndeg / 1e9;
		last_lat_filt = (double)csv_gps.latitude_filt_ndeg / 1e9;
		last_lng_filt = (double)csv_gps.longitude_filt_ndeg / 1e9;
		last_speed = (float)csv_gps.speed_mmps * 0.0036f;
		last_course = (float)csv_gps.bearing_mdeg / 1000.0f;
		last_sats = csv_gps.satellites;
		have_last_gps = true;

		pos += snprintf(line + pos, sizeof(line) - pos,
				"%.6f,%.6f,%.6f,%.6f,%.1f,%.1f,%u,0,",
				last_lat, last_lng, last_lat_filt, last_lng_filt,
				(double)last_speed, (double)last_course, last_sats);
	} else if (have_last_gps) {
		/* Stale — forward-fill last known position, stale=1 */
		pos += snprintf(line + pos, sizeof(line) - pos,
				"%.6f,%.6f,%.6f,%.6f,0.0,%.1f,%u,1,",
				last_lat, last_lng, last_lat_filt, last_lng_filt,
				(double)last_course, last_sats);
	} else {
		/* No GPS ever — empty fields, stale=1 */
		pos += snprintf(line + pos, sizeof(line) - pos, ",,,,,,,1,");
	}

	/* Audio: rms, peak, zcr */
	struct audio_data aud;

	audio_get_data(&aud);
	pos += snprintf(line + pos, sizeof(line) - pos, "%.0f,%u,%u,",
			(double)aud.rms, aud.peak, aud.zcr);

	/* Event tag (consumed on first use) */
	char evt[32];

	if (sdcard_consume_event(evt, sizeof(evt))) {
		/* Quote if it contains a comma */
		if (strchr(evt, ',')) {
			pos += snprintf(line + pos, sizeof(line) - pos,
					"\"%s\"", evt);
		} else {
			pos += snprintf(line + pos, sizeof(line) - pos,
					"%s", evt);
		}
	}

	/* Newline */
	if (pos < (int)sizeof(line) - 1) {
		line[pos++] = '\n';
	}

	sdcard_write(line, pos);
}

/* ═══════════════════════════════════════════════════════════════
 * Main
 * ═══════════════════════════════════════════════════════════════ */
int main(void)
{
	int ret;
	uint32_t cycle = 0;

	/* Blue LED solid during boot */
	gpio_pin_configure_dt(&led_red, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led_green, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led_blue, GPIO_OUTPUT_ACTIVE);

	/* Give USB 2 seconds to enumerate */
	k_msleep(2000);

	printk("\n\n");
	printk("================================================\n");
	printk("  CoMotion Tracker v2.0.0\n");
	printk("  XIAO BLE Sense (nRF52840)\n");
	printk("================================================\n\n");

	/* --- IMU --- */
	ret = init_imu();
	if (ret) {
		printk("[!!] IMU init failed\n");
	} else {
		imu_ok = true;
		printk("[OK] IMU (LSM6DS3TR-C)\n");

		/* Gyro calibration */
		gyro_calibrate();
	}

	/* --- Intensity + Impact --- */
	intensity_init();
	impact_init();
	printk("[OK] Intensity + impact detection\n");

	/* --- GPS --- */
	{
		const struct device *gnss = DEVICE_DT_GET(DT_NODELABEL(gnss));

		if (device_is_ready(gnss)) {
			printk("[OK] GPS on UART0\n");
			ret = gps_init();
			if (ret) {
				printk("[!!] GPS config failed (%d)\n", ret);
			} else {
				printk("[OK] GPS configured\n");
			}
		} else {
			printk("[!!] GPS not ready\n");
		}
	}

	/* --- Audio (starts its own thread) --- */
	ret = audio_init();
	if (ret) {
		printk("[!!] Audio init failed (%d)\n", ret);
	} else {
		printk("[OK] Audio (PDM 16kHz, 10Hz processing)\n");
	}

	/* --- Device Identity (must be before BLE and SD card) --- */
	device_id_init();
	printk("[OK] Device ID: %s (short: %s)\n", device_id_full(), device_id_short());

	/* --- SD Card --- */
	ret = sdcard_init();
	if (ret) {
		printk("[!!] SD init failed (%d)\n", ret);
	} else {
		printk("[OK] SD card\n");
	}

	/* --- Battery --- */
	ret = battery_init();
	if (ret) {
		printk("[!!] Battery init failed (%d)\n", ret);
	} else {
		printk("[OK] Battery: %d mV (%d%%)\n",
		       battery_millivolts(), battery_level_pct());
	}

	/* --- BLE (advertising + NUS) --- */
	ret = ble_adv_init(nus_rx_handler);
	if (ret) {
		printk("[!!] BLE init failed (%d)\n", ret);
	} else {
		printk("[OK] BLE advertising as \"%s\"\n", CONFIG_BT_DEVICE_NAME);
	}

	/* Boot complete — turn off blue LED */
	gpio_pin_set_dt(&led_blue, 0);

	printk("\n--- Running at 104 Hz ---\n\n");

	/* ═══ Start 104 Hz timer ═══ */
	k_timer_start(&imu_timer, K_USEC(IMU_PERIOD_US), K_USEC(IMU_PERIOD_US));

	/* ═══ Main Loop (104 Hz) ═══ */
	while (1) {
		k_sem_take(&imu_tick, K_FOREVER);
		cycle++;

		/* ─── Check for pending NUS commands ─── */
		if (k_sem_take(&cmd_sem, K_NO_WAIT) == 0) {
			process_command(pending_cmd);
		}

		/* ─── IMU Read + Process ─── */
		float ax_g = 0, ay_g = 0, az_g = 0;
		float gx_dps = 0, gy_dps = 0, gz_dps = 0;

		if (imu_ok) {
			struct sensor_value accel[3], gyro[3];

			if (sensor_sample_fetch(imu) == 0) {
				sensor_channel_get(imu, SENSOR_CHAN_ACCEL_X,
						   &accel[0]);
				sensor_channel_get(imu, SENSOR_CHAN_ACCEL_Y,
						   &accel[1]);
				sensor_channel_get(imu, SENSOR_CHAN_ACCEL_Z,
						   &accel[2]);
				sensor_channel_get(imu, SENSOR_CHAN_GYRO_X,
						   &gyro[0]);
				sensor_channel_get(imu, SENSOR_CHAN_GYRO_Y,
						   &gyro[1]);
				sensor_channel_get(imu, SENSOR_CHAN_GYRO_Z,
						   &gyro[2]);

				ax_g = sv_to_g(&accel[0]);
				ay_g = sv_to_g(&accel[1]);
				az_g = sv_to_g(&accel[2]);

				/* Apply gyro calibration offsets */
				gx_dps = sv_to_dps(&gyro[0]) - gyro_off_x;
				gy_dps = sv_to_dps(&gyro[1]) - gyro_off_y;
				gz_dps = sv_to_dps(&gyro[2]) - gyro_off_z;

				/*
				 * Low-pass filter: per-axis EMA (α=0.3,
				 * ~5 Hz cutoff at 104 Hz sample rate).
				 * Removes sensor noise and vibration
				 * while preserving motion dynamics.
				 *
				 * Raw values (ax_g etc.) are preserved
				 * for SD card CSV logging.
				 */
				float fax = filter_ema_update(&filt_ax, ax_g);
				float fay = filter_ema_update(&filt_ay, ay_g);
				float faz = filter_ema_update(&filt_az, az_g);
				float fgx = filter_ema_update(&filt_gx, gx_dps);
				float fgy = filter_ema_update(&filt_gy, gy_dps);
				float fgz = filter_ema_update(&filt_gz, gz_dps);

				/* Feed FILTERED values to intensity tracker */
				intensity_feed(fax, fay, faz,
					       fgx, fgy, fgz);

				/* Feed FILTERED magnitude to impact detector */
				float accel_mag = sqrtf(fax * fax +
							fay * fay +
							faz * faz);
				impact_feed_imu(accel_mag);
			}
		}

		/* ─── SD Logging at ~10 Hz (every 10 cycles) ─── */
		if (sdcard_is_logging() && (cycle % LOG_CYCLES) == 0) {
			format_and_log_csv(ax_g, ay_g, az_g,
					   gx_dps, gy_dps, gz_dps);
		}

		/* ─── 5 Hz BLE update (every 21 cycles ≈ 200ms) ─── */
		if ((cycle % FAST_CYCLES) == 0) {
			struct ble_packet pkt;

			build_ble_packet(&pkt);
			ble_adv_update(&pkt);
		}

		/* ─── 2 Hz tasks (every 52 cycles) ─── */
		if ((cycle % SLOW_CYCLES) == 0) {
			/* Intensity history sample */
			intensity_sample_history();

			/* LED update */
			update_leds(cycle);

			/* Console status (every ~4s = every 2nd slow cycle) */
			if ((cycle % (SLOW_CYCLES * 2)) == 0) {
				printk("[%u] INT:%.2f IMP:%u BAT:%dmV(%d%%)\n",
				       cycle,
				       (double)intensity_get_current(),
				       impact_get_count(),
				       battery_millivolts(),
				       battery_level_pct());
			}
		}

		/* ─── Battery read every 30s ─── */
		if ((cycle % BATT_CYCLES) == 0) {
			battery_sample();
		}
	}

	return 0;
}
