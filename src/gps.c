/*
 * CoMotion Tracker - GPS Module (ATGM332D via Zephyr GNSS framework)
 *
 * Uses the built-in gnss-nmea-generic driver which handles:
 *   - UART interrupt-driven receive via modem_chat
 *   - NMEA sentence parsing (RMC + GGA, matched by UTC timestamp)
 *   - Structured data output via GNSS_DATA_CALLBACK_DEFINE
 *
 * We register a callback and store the latest fix.  Position fields are
 * only updated on valid fixes — during no-fix periods the last known
 * good position is preserved (the consumer checks last_fix_ms for
 * staleness).
 *
 * The ATGM332D is configured at init via PCAS commands sent directly
 * over UART (the gnss-nmea-generic driver has no configuration API).
 */

#include "gps.h"
#include "filter.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gnss.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <string.h>

LOG_MODULE_REGISTER(gps, CONFIG_LOG_DEFAULT_LEVEL);

/* ─── PCAS Configuration for ATGM332D ─────────────────────────
 *
 * The ATGM332D uses the CASIC PCAS protocol (same as Luatos Air530Z).
 * The gnss-nmea-generic Zephyr driver has an empty gnss_api — all
 * gnss_set_*() calls return -ENOSYS.  We configure the module by
 * sending raw PCAS commands via uart_poll_out() on the same UART.
 *
 * Commands sent at init (after a 500ms settle delay):
 *   PCAS03 — Reduce NMEA output to GGA+RMC only (bandwidth critical)
 *   PCAS02 — Set 5Hz fix rate (200ms interval)
 *   PCAS04 — Enable GPS + BeiDou constellations
 *   PCAS11 — Set normal (portable) navigation mode
 */

/* Compute NMEA XOR checksum over the payload (between $ and *) */
static uint8_t nmea_checksum(const char *payload)
{
	uint8_t cksum = 0;

	for (const char *p = payload; *p; p++) {
		cksum ^= (uint8_t)*p;
	}
	return cksum;
}

/* Send a complete NMEA sentence: $<payload>*<checksum>\r\n */
static void gps_send_pcas(const struct device *uart, const char *payload)
{
	char buf[80];
	uint8_t ck = nmea_checksum(payload);
	int len = snprintf(buf, sizeof(buf), "$%s*%02X\r\n", payload, ck);

	for (int i = 0; i < len; i++) {
		uart_poll_out(uart, buf[i]);
	}

	/* Module needs time to process each command — 100ms is safe */
	k_msleep(100);
}

/* Store UART device for deferred retry */
static const struct device *gps_uart;

static void gps_send_config(const struct device *uart)
{
	/*
	 * PCAS03: NMEA sentence output filter — MUST be first.
	 * Fields: GGA,GLL,GSA,GSV,RMC,VTG,ZDA,ANT,DHV,LPS,UTC,GST,TIM
	 * Enable GGA (field 1) and RMC (field 5) only.
	 * At 5Hz / 9600 baud: GGA+RMC ≈ 120 B/fix × 5 = 600 B/s
	 * (within the 960 B/s UART capacity).
	 *
	 * *** CRITICAL: filter BEFORE rate increase ***
	 * If PCAS02 lands but PCAS03 doesn't, we'd have 5Hz × all
	 * sentences = ~2500 B/s on a 960 B/s UART → total data loss.
	 */
	gps_send_pcas(uart, "PCAS03,1,0,0,0,1,0,0,0,0,0,0,0,0");

	/*
	 * PCAS02: Fix update rate in milliseconds.
	 * 200ms = 5Hz (maximum practical rate at 9600 baud with GGA+RMC).
	 */
	gps_send_pcas(uart, "PCAS02,200");

	/*
	 * PCAS04: Constellation selection (bitmask).
	 * 5 = GPS + BeiDou (0b101) — more satellites = better accuracy.
	 * NOTE: this can trigger a warm restart of the receiver.
	 */
	gps_send_pcas(uart, "PCAS04,5");

	/*
	 * PCAS11: Navigation mode.
	 * 0 = normal (portable) — raw fixes, no motion-model smoothing.
	 */
	gps_send_pcas(uart, "PCAS11,0");
}

int gps_init(void)
{
	const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));

	if (!device_is_ready(uart)) {
		LOG_ERR("GPS UART not ready");
		return -ENODEV;
	}

	gps_uart = uart;

	/*
	 * Wait for the module to finish its boot NMEA burst.
	 * The ATGM332D outputs a flurry of default sentences at power-on
	 * that can last 1–2 seconds.  Commands sent during the burst are
	 * silently dropped.  2s is conservative but safe.
	 */
	k_msleep(2000);

	LOG_INF("Configuring ATGM332D via PCAS commands...");
	gps_send_config(uart);

	LOG_INF("GPS configured: 5Hz, normal, GPS+BDS, GGA+RMC");
	printk("     5Hz | Normal | GPS+BDS | GGA+RMC\n");

	return 0;
}

/* ─── Spinlock-protected latest fix ─── */

static struct k_spinlock gps_lock;
static struct gps_data   latest;
static bool              data_valid;   /* true after first callback */

/* ─── GPS signal filters (state lives outside lock, updated only in cb) ─── */

/*
 * Speed Kalman: process noise Q = (5 m/s² × 0.2s)² = 1.0 (m/s)²
 * allows human sprinting acceleration.  Measurement noise R = 4.0
 * (m/s)² = typical GPS speed noise (~2 m/s std dev).
 */
static struct filter_kalman1d gps_speed_kalman =
	FILTER_KALMAN1D_INIT(1.0f, 4.0f);

/*
 * Position EMA: α = 0.5 at 5Hz → ~400ms time constant.
 * Light smoothing — removes single-fix jitter without adding
 * noticeable latency at walking/running speeds.
 *
 * NOTE: float has ~7 significant digits, nanodegrees can be up to
 * ~1.8e11.  Casting to float loses ~4 decimal digits of precision
 * (~1m error at most).  Acceptable for BLE telemetry.
 * Raw nanodegree values are preserved in latitude_ndeg/longitude_ndeg.
 */
static struct filter_ema gps_lat_ema = FILTER_EMA_INIT(0.5f);
static struct filter_ema gps_lon_ema = FILTER_EMA_INIT(0.5f);

/*
 * Bearing circular EMA: α = 0.4 at 5Hz.
 * Handles 0°/360° wrap correctly via sin/cos decomposition.
 * Only updated when speed > 2 km/h (bearing is noise at standstill).
 */
static struct filter_angle_ema gps_brg_ema = FILTER_ANGLE_EMA_INIT(0.4f);

/* ─── Fix rate verification ─── */
#define RATE_CHECK_SAMPLES  10  /* measure over first N callbacks */
static int64_t rate_first_ts;       /* uptime of callback #1 */
static uint32_t rate_cb_count;      /* callbacks since boot (outside lock) */
static bool rate_reported;          /* true once we've logged the result */
static bool config_retried;         /* true once we've done a one-shot retry */

/* ─── Deferred config retry work ─── */
static void config_retry_handler(struct k_work *work);
static K_WORK_DEFINE(config_retry_work, config_retry_handler);

static void config_retry_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!gps_uart) {
		return;
	}

	LOG_WRN("GPS: boot commands likely dropped — resending config");
	printk("GPS: retrying PCAS config...\n");
	gps_send_config(gps_uart);

	/* Reset rate measurement to verify the retry worked */
	rate_cb_count = 0;
	rate_reported = false;
}

/*
 * GNSS data callback — fires once per fix cycle (after both GGA and RMC
 * with matching UTC timestamps are received by the gnss-nmea-generic
 * driver).  This fires even when there is no fix.
 */
static void gnss_data_cb(const struct device *dev, const struct gnss_data *data)
{
	k_spinlock_key_t key = k_spin_lock(&gps_lock);
	int64_t now = k_uptime_get();

	bool fix = (data->info.fix_status != GNSS_FIX_STATUS_NO_FIX);

	/*
	 * Fix-rate verification: measure the average interval over the
	 * first RATE_CHECK_SAMPLES callbacks and log the result once.
	 * This confirms whether PCAS02 actually took effect (5Hz → ~200ms
	 * between callbacks, vs 1Hz default → ~1000ms).
	 *
	 * If rate is ~1Hz (>300ms avg), the boot commands were likely
	 * dropped during the module's startup burst.  We resend ONCE
	 * via a work item (can't sleep in a spinlock/callback context).
	 */
	rate_cb_count++;
	if (rate_cb_count == 1) {
		rate_first_ts = now;
	} else if (rate_cb_count == (RATE_CHECK_SAMPLES + 1) && !rate_reported) {
		int64_t elapsed = now - rate_first_ts;
		uint32_t avg_ms = (uint32_t)(elapsed / RATE_CHECK_SAMPLES);
		uint32_t rate_hz_x10 = (avg_ms > 0) ? (10000 / avg_ms) : 0;

		LOG_INF("GPS fix rate: %u ms avg (%u.%u Hz) over %d samples",
			avg_ms, rate_hz_x10 / 10, rate_hz_x10 % 10,
			RATE_CHECK_SAMPLES);
		printk("GPS: measured %u ms/fix = %u.%u Hz\n",
		       avg_ms, rate_hz_x10 / 10, rate_hz_x10 % 10);

		if (avg_ms > 300 && !config_retried) {
			LOG_WRN("GPS rate ~%u.%u Hz — retrying config",
				rate_hz_x10 / 10, rate_hz_x10 % 10);
			config_retried = true;
			k_work_submit(&config_retry_work);
		}
		rate_reported = true;
	}

	/* Always update: fix flag, quality, time, counters */
	latest.has_fix      = fix;
	latest.fix_quality  = (uint8_t)data->info.fix_quality;
	latest.hour         = data->utc.hour;
	latest.minute       = data->utc.minute;
	latest.second       = (uint8_t)(data->utc.millisecond / 1000);
	latest.update_count++;
	latest.last_update_ms = now;

	/*
	 * Satellites and HDOP: only update when the driver has a fix.
	 *
	 * The gnss-nmea-generic GGA parser returns early on quality=0
	 * (no fix) WITHOUT updating satellites_cnt or hdop — those
	 * fields retain stale values from the last valid fix.  We
	 * zero them here to prevent misleading the BLE packet/app.
	 */
	if (fix) {
		latest.satellites = data->info.satellites_cnt;
		latest.hdop       = data->info.hdop;
	} else {
		latest.satellites = 0;
		latest.hdop       = 0;
	}

	/*
	 * Position and dynamics: only update on a valid fix.
	 *
	 * When the module loses fix, the gnss-nmea-generic driver does NOT
	 * update nav_data (RMC parser returns early on status 'V').
	 * We freeze the last known good position so the BLE packet doesn't
	 * flap between valid coordinates and the no-fix sentinel.
	 * The consumer uses last_fix_ms to decide whether the position
	 * is still usable.
	 *
	 * Filtering pipeline (on valid fix only):
	 *   1. Outlier rejection — reject impossible jumps
	 *   2. Store raw values (always available for CSV logging)
	 *   3. Feed filters and store filtered values
	 */
	if (fix) {
		int64_t raw_lat  = data->nav_data.latitude;
		int64_t raw_lon  = data->nav_data.longitude;
		uint32_t raw_spd = data->nav_data.speed;
		uint32_t raw_brg = data->nav_data.bearing;

		bool accepted = true;

		/*
		 * Outlier rejection: skip this fix if it implies
		 * impossible motion since the last accepted fix.
		 */
		if (latest.last_fix_ms > 0) {
			/* Position jump check:
			 * At the equator, 1° ≈ 111 km, so 100m ≈ 900000 ndeg.
			 * At 5Hz (200ms), 100m/0.2s = 500 m/s = 1800 km/h.
			 * Anything beyond that is a GPS glitch.
			 */
			int64_t dlat = raw_lat - latest.latitude_ndeg;
			int64_t dlon = raw_lon - latest.longitude_ndeg;

			if (dlat < 0) dlat = -dlat;
			if (dlon < 0) dlon = -dlon;

			/* 900000 ndeg ≈ 100m at equator */
			if (dlat > 900000LL || dlon > 900000LL) {
				accepted = false;
				LOG_WRN("GPS: position jump rejected "
					"(dlat=%lld dlon=%lld ndeg)",
					dlat, dlon);
			}

			/* Speed jump check: >50 km/h change between fixes
			 * 50 km/h = 13889 mm/s
			 */
			int32_t dspd = (int32_t)raw_spd - (int32_t)latest.speed_mmps;
			if (dspd < 0) dspd = -dspd;

			if (dspd > 13889) {
				accepted = false;
				LOG_WRN("GPS: speed jump rejected "
					"(%u -> %u mm/s)",
					latest.speed_mmps, raw_spd);
			}
		}

		if (accepted) {
			/* Store raw values */
			latest.latitude_ndeg  = raw_lat;
			latest.longitude_ndeg = raw_lon;
			latest.altitude_mm    = data->nav_data.altitude;
			latest.speed_mmps     = raw_spd;
			latest.bearing_mdeg   = raw_brg;
			latest.last_fix_ms    = now;

			/* --- Filtered speed (1D Kalman) --- */
			float spd_ms = (float)raw_spd / 1000.0f;
			float filt_ms = filter_kalman1d_update(
				&gps_speed_kalman, spd_ms);
			latest.speed_filt_mmps =
				(uint32_t)(filt_ms * 1000.0f);

			/* --- Filtered position (EMA) --- */
			latest.latitude_filt_ndeg = (int64_t)
				filter_ema_update(&gps_lat_ema,
					(float)raw_lat);
			latest.longitude_filt_ndeg = (int64_t)
				filter_ema_update(&gps_lon_ema,
					(float)raw_lon);

			/* --- Filtered bearing (circular EMA) ---
			 * Only update when moving >2 km/h (556 mm/s);
			 * bearing is meaningless when stationary.
			 */
			if (raw_spd > 556) {
				latest.bearing_filt_mdeg =
					filter_angle_ema_update(
						&gps_brg_ema, raw_brg);
			}
		}
	}

	data_valid = true;
	k_spin_unlock(&gps_lock, key);
}

GNSS_DATA_CALLBACK_DEFINE(DEVICE_DT_GET(DT_NODELABEL(gnss)), gnss_data_cb);

int gps_get_data(struct gps_data *out)
{
	k_spinlock_key_t key = k_spin_lock(&gps_lock);

	if (!data_valid) {
		k_spin_unlock(&gps_lock, key);
		return -ENODATA;
	}

	*out = latest;

	k_spin_unlock(&gps_lock, key);
	return 0;
}

int gps_format(const struct gps_data *data, char *buf, int buf_size)
{
	if (!data->has_fix) {
		return snprintf(buf, buf_size,
			"GPS: no fix (%u sats, %u updates)",
			data->satellites, data->update_count);
	}

	/*
	 * Convert nanodegrees to degrees with 6 decimal places.
	 * We use absolute value + separate sign string to correctly
	 * handle coordinates between -1° and 0° (e.g. -0.5° near
	 * the equator or prime meridian).
	 */
	const char *lat_sign = (data->latitude_ndeg < 0) ? "-" : "";
	int64_t lat_abs = (data->latitude_ndeg < 0)
			? -data->latitude_ndeg : data->latitude_ndeg;
	int lat_deg  = (int)(lat_abs / 1000000000LL);
	int lat_frac = (int)((lat_abs % 1000000000LL) / 1000);

	const char *lon_sign = (data->longitude_ndeg < 0) ? "-" : "";
	int64_t lon_abs = (data->longitude_ndeg < 0)
			? -data->longitude_ndeg : data->longitude_ndeg;
	int lon_deg  = (int)(lon_abs / 1000000000LL);
	int lon_frac = (int)((lon_abs % 1000000000LL) / 1000);

	int64_t age_ms = (data->last_fix_ms > 0)
		? (k_uptime_get() - data->last_fix_ms) : -1LL;

	return snprintf(buf, buf_size,
		"GPS: %s%d.%06d,%s%d.%06d alt=%dm spd=%u.%01um/s "
		"hdop=%u.%u %usat %02u:%02u:%02u age=%lldms",
		lat_sign, lat_deg, lat_frac,
		lon_sign, lon_deg, lon_frac,
		data->altitude_mm / 1000,
		data->speed_mmps / 1000, (data->speed_mmps % 1000) / 100,
		data->hdop / 1000, (data->hdop % 1000) / 100,
		data->satellites,
		data->hour, data->minute, data->second,
		age_ms);
}
