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

int gps_init(void)
{
	const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));

	if (!device_is_ready(uart)) {
		LOG_ERR("GPS UART not ready");
		return -ENODEV;
	}

	/*
	 * Wait for the module to finish its boot NMEA burst.
	 * The ATGM332D outputs a flurry of default sentences at power-on;
	 * commands sent during that burst are likely to be ignored.
	 */
	k_msleep(500);

	LOG_INF("Configuring ATGM332D via PCAS commands...");

	/*
	 * PCAS03: NMEA sentence output filter
	 * Fields: GGA,GLL,GSA,GSV,RMC,VTG,ZDA,ANT,DHV,LPS,UTC,GST,TIM
	 * Enable GGA (field 1) and RMC (field 5) only.
	 * At 5Hz / 9600 baud: GGA+RMC ≈ 120 B/fix × 5 = 600 B/s
	 * (within the 960 B/s UART capacity).
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
	 */
	gps_send_pcas(uart, "PCAS04,5");

	/*
	 * PCAS11: Navigation mode.
	 * 0 = normal (portable) — raw fixes, no motion-model smoothing.
	 */
	gps_send_pcas(uart, "PCAS11,0");

	LOG_INF("GPS configured: 5Hz, normal, GPS+BDS, GGA+RMC");
	printk("     5Hz | Normal | GPS+BDS | GGA+RMC\n");

	return 0;
}

/* ─── Spinlock-protected latest fix ─── */

static struct k_spinlock gps_lock;
static struct gps_data   latest;
static bool              data_valid;   /* true after first callback */

/* ─── Fix rate verification ─── */
#define RATE_CHECK_SAMPLES  10  /* measure over first N callbacks */
static int64_t rate_first_ts;       /* uptime of callback #1 */
static uint32_t rate_cb_count;      /* callbacks since boot (outside lock) */
static bool rate_reported;          /* true once we've logged the result */

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

		if (avg_ms > 300) {
			LOG_WRN("GPS rate slower than expected — "
				"PCAS02 may not have taken effect (default 1Hz)");
		}
		rate_reported = true;
	}

	/* Always update: fix flag, satellite info, HDOP, quality, time, counters */
	latest.has_fix      = fix;
	latest.satellites   = data->info.satellites_cnt;
	latest.hdop         = data->info.hdop;
	latest.fix_quality  = (uint8_t)data->info.fix_quality;
	latest.hour         = data->utc.hour;
	latest.minute       = data->utc.minute;
	latest.second       = (uint8_t)(data->utc.millisecond / 1000);
	latest.update_count++;
	latest.last_update_ms = now;

	/*
	 * Position and dynamics: only update on a valid fix.
	 *
	 * When the module loses fix, the gnss-nmea-generic driver does NOT
	 * update nav_data (RMC parser returns early on status 'V').
	 * We freeze the last known good position so the BLE packet doesn't
	 * flap between valid coordinates and the no-fix sentinel.
	 * The consumer uses last_fix_ms to decide whether the position
	 * is still usable.
	 */
	if (fix) {
		latest.latitude_ndeg  = data->nav_data.latitude;
		latest.longitude_ndeg = data->nav_data.longitude;
		latest.altitude_mm    = data->nav_data.altitude;
		latest.speed_mmps     = data->nav_data.speed;
		latest.bearing_mdeg   = data->nav_data.bearing;
		latest.last_fix_ms    = now;
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
