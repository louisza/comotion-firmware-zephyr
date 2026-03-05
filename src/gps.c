/*
 * CoMotion Tracker - GPS Module (ATGM332D via Zephyr GNSS framework)
 *
 * Uses the built-in gnss-nmea-generic driver which handles:
 *   - UART interrupt-driven receive
 *   - NMEA sentence parsing (RMC, GGA, GSV)
 *   - Structured data output via callback
 *
 * We just register a callback and store the latest fix.
 */

#include "gps.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gnss.h>
#include <stdio.h>

/* Latest fix — protected by a spinlock since callback runs from modem thread */
static struct k_spinlock gps_lock;
static struct gps_data latest;
static bool data_valid;
static uint32_t update_count;

/* Called by the GNSS framework whenever a new NMEA fix/update is parsed */
static void gnss_data_cb(const struct device *dev, const struct gnss_data *data)
{
	k_spinlock_key_t key = k_spin_lock(&gps_lock);

	latest.has_fix = (data->info.fix_status != GNSS_FIX_STATUS_NO_FIX);
	latest.latitude_ndeg = data->nav_data.latitude;
	latest.longitude_ndeg = data->nav_data.longitude;
	latest.altitude_mm = data->nav_data.altitude;
	latest.speed_mmps = data->nav_data.speed;
	latest.satellites = data->info.satellites_cnt;
	latest.hour = data->utc.hour;
	latest.minute = data->utc.minute;
	latest.second = (uint8_t)(data->utc.millisecond / 1000);
	data_valid = true;
	update_count++;

	k_spin_unlock(&gps_lock, key);
}

/* Pass the DT_GET directly — the macro needs a compile-time constant */
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

uint32_t gps_get_update_count(void)
{
	k_spinlock_key_t key = k_spin_lock(&gps_lock);
	uint32_t count = update_count;
	k_spin_unlock(&gps_lock, key);
	return count;
}

int gps_format(const struct gps_data *data, char *buf, int buf_size)
{
	if (!data->has_fix) {
		return snprintf(buf, buf_size, "GPS: no fix (%u sats)",
			data->satellites);
	}

	/*
	 * Convert nanodegrees to degrees with 6 decimal places.
	 * We split into integer + fractional parts to avoid floating point.
	 * Fractional part: (abs(nanodeg) % 1000000000) / 1000 → microdegrees
	 */
	int lat_deg = (int)(data->latitude_ndeg / 1000000000LL);
	int lat_frac = (int)((data->latitude_ndeg < 0
		? -data->latitude_ndeg : data->latitude_ndeg) % 1000000000LL / 1000);
	int lon_deg = (int)(data->longitude_ndeg / 1000000000LL);
	int lon_frac = (int)((data->longitude_ndeg < 0
		? -data->longitude_ndeg : data->longitude_ndeg) % 1000000000LL / 1000);

	return snprintf(buf, buf_size,
		"GPS: %d.%06d,%d.%06d alt=%dm spd=%u.%01um/s %usat %02u:%02u:%02u",
		lat_deg, lat_frac,
		lon_deg, lon_frac,
		data->altitude_mm / 1000,
		data->speed_mmps / 1000, (data->speed_mmps % 1000) / 100,
		data->satellites,
		data->hour, data->minute, data->second);
}
