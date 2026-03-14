/*
 * CoMotion Tracker - GPS Module (ATGM332D via Zephyr GNSS framework)
 *
 * All GPS state is captured atomically via gps_get_data() so consumers
 * see a consistent snapshot of position + timing + satellite info.
 */

#ifndef GPS_H
#define GPS_H

#include <stdbool.h>
#include <stdint.h>

/*
 * GPS fix snapshot — updated asynchronously by the GNSS callback.
 *
 * Position and dynamics fields are ONLY updated when the module reports
 * a valid fix.  During no-fix periods the last known good position is
 * preserved — the consumer checks last_fix_ms to decide staleness.
 *
 * All fields (position + timing + counters) are captured under a single
 * spinlock by gps_get_data(), so races between "is it fresh?" and
 * "what are the coordinates?" are impossible.
 */
struct gps_data {
	/* Fix state (updated every callback) */
	bool     has_fix;          /* true if most recent callback had a fix  */
	uint16_t satellites;       /* satellites tracked (always current)     */
	uint32_t hdop;             /* HDOP × 1000 (e.g. 1500 = 1.5)         */
	uint8_t  fix_quality;      /* GNSS_FIX_QUALITY_* (0=invalid, 1=SPS…) */

	/* Position — frozen at last valid fix, never cleared (RAW) */
	int64_t  latitude_ndeg;    /* nanodegrees (÷ 1e9 → degrees)          */
	int64_t  longitude_ndeg;   /* nanodegrees                            */
	int32_t  altitude_mm;      /* millimeters above MSL                  */

	/* Position — filtered (EMA-smoothed, outlier-rejected) */
	int64_t  latitude_filt_ndeg;
	int64_t  longitude_filt_ndeg;

	/* Dynamics — frozen at last valid fix (RAW) */
	uint32_t speed_mmps;       /* millimeters per second                 */
	uint32_t bearing_mdeg;     /* millidegrees (0–360000)                */

	/* Dynamics — filtered */
	uint32_t speed_filt_mmps;  /* Kalman-filtered speed (mm/s)           */
	uint32_t bearing_filt_mdeg;/* circular-EMA-filtered bearing (mdeg)   */

	/* UTC date+time (from latest NMEA sentence, valid even without fix) */
	uint16_t year;     /* e.g. 2026 */
	uint8_t  month;    /* 1-12 */
	uint8_t  day;      /* 1-31 */
	uint8_t  hour;
	uint8_t  minute;
	uint8_t  second;
	bool     time_valid; /* true once we've received at least one UTC time */

	/* Timing — kernel uptime in ms, 0 means "never" */
	int64_t  last_fix_ms;      /* when last VALID fix was received        */
	int64_t  last_update_ms;   /* when last GNSS callback fired (any)    */
	uint32_t update_count;     /* total GNSS callbacks received           */
};

/*
 * Initialize GPS module: send PCAS configuration commands to ATGM332D
 * for 5Hz update rate, automotive nav mode, GPS+BeiDou, GGA+RMC only.
 *
 * Must be called after the GNSS driver has started (POST_KERNEL).
 * Returns 0 on success, negative errno on failure.
 */
int gps_init(void);

/*
 * Get an atomic snapshot of the latest GPS data.
 *
 * All fields (position, timing, counts) are captured under a single
 * spinlock so the caller sees a consistent view.
 *
 * Returns 0 if at least one GNSS callback has fired, -ENODATA if the
 * module has never produced any output.
 */
int gps_get_data(struct gps_data *out);

/*
 * Format GPS data into a human-readable string.
 * Handles negative coordinates correctly (including -0.x° near equator).
 * Returns number of characters written.
 */
int gps_format(const struct gps_data *data, char *buf, int buf_size);

#endif /* GPS_H */
