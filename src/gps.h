/*
 * CoMotion Tracker - GPS Module (ATGM332D via Zephyr GNSS framework)
 */

#ifndef GPS_H
#define GPS_H

#include <stdbool.h>
#include <stdint.h>

/* Latest GPS fix data — updated asynchronously by GNSS callback */
struct gps_data {
	bool has_fix;
	int64_t latitude_ndeg;   /* nanodegrees (divide by 1e9 for degrees) */
	int64_t longitude_ndeg;  /* nanodegrees */
	int32_t altitude_mm;     /* millimeters above MSL */
	uint32_t speed_mmps;     /* millimeters per second */
	uint16_t satellites;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
};

/*
 * Get a snapshot of the latest GPS data.
 * Returns 0 if data is available (even without fix), -ENODATA if no NMEA yet.
 */
int gps_get_data(struct gps_data *out);

/*
 * Get the number of NMEA updates received from the GPS module.
 * If this is 0, the UART link may not be working.
 * If this is >0 but has_fix is false, the module is alive but searching.
 */
uint32_t gps_get_update_count(void);

/*
 * Format GPS data into a human-readable string.
 * Returns number of characters written.
 */
int gps_format(const struct gps_data *data, char *buf, int buf_size);

#endif /* GPS_H */
