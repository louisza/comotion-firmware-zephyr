/*
 * ═══════════════════════════════════════════════════════════════
 * CoMotion Tracker — Common Definitions
 * ═══════════════════════════════════════════════════════════════
 */
#ifndef COMOTION_COMMON_H
#define COMOTION_COMMON_H

#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>

#define FIRMWARE_VERSION "3.0.0"
#define DEVICE_NAME      "CoMotion"

/* ─── Field center (GPS reference for BLE position encoding) ─── */
#define FIELD_CENTER_LAT  (-25.7479f)
#define FIELD_CENTER_LNG  ( 28.2293f)

/* ─── Sampling rates ─── */
#define IMU_SAMPLE_RATE_HZ      104
#define AUDIO_RMS_RATE_HZ       10
#define GPS_TIMEOUT_MS          2000
#define BLE_ADV_UPDATE_MS       500
#define INTENSITY_SAMPLE_HZ     2
#define BATTERY_READ_INTERVAL_S 30

/* ─── Audio impact detection (dual-trigger) ─── */
#define AUDIO_IMPACT_THRESHOLD   8.0f
#define AUDIO_IMPACT_MIN_PEAK    3000
#define AUDIO_BASELINE_ALPHA     0.005f
#define AUDIO_IMPACT_DEBOUNCE_MS 3000
#define IMPACT_ACCEL_THRESHOLD   3.0f
#define IMPACT_CONFIRM_WINDOW_MS 200

/* ─── Intensity calculation ─── */
#define INTENSITY_ACCEL_WEIGHT  0.7f
#define INTENSITY_GYRO_WEIGHT   0.3f
#define INTENSITY_GYRO_SCALE    100.0f
#define INTENSITY_EMA_FAST      0.019f
#define INTENSITY_EMA_SLOW      0.00032f
#define INTENSITY_HISTORY_SEC   600
#define INTENSITY_HISTORY_SIZE  (INTENSITY_HISTORY_SEC * INTENSITY_SAMPLE_HZ)

/* ─── BLE ─── */
#define BLE_MANUFACTURER_ID     0xFFFF
#define BLE_ADV_INTERVAL_NORMAL 500
#define BLE_ADV_INTERVAL_FOCUS  100
#define IMPACT_FLAG_DURATION_MS 2000
#define ADV_DATA_LEN            20

/* ─── Status flags (byte 0 of advertising data) ─── */
#define STATUS_FLAG_LOGGING     0x01
#define STATUS_FLAG_GPS_FIX     0x02
#define STATUS_FLAG_LOW_BATTERY 0x04
#define STATUS_FLAG_IMPACT      0x08
#define STATUS_FLAG_BLE_CONN    0x10
#define STATUS_FLAG_FOCUS_MODE  0x20

/* ─── SD Card ─── */
#define SD_BUFFER_SIZE          4096
#define SD_FLUSH_INTERVAL_MS    5000

/* ─── Shared IMU reading ─── */
struct imu_reading {
	float ax, ay, az;   /* Accelerometer (g) */
	float gx, gy, gz;   /* Gyroscope (dps, calibration applied) */
};

/* ─── GPS data ─── */
struct gps_data {
	float    latitude;
	float    longitude;
	float    speed;       /* km/h */
	float    course;      /* degrees */
	int      satellites;
	int64_t  last_update; /* k_uptime_get() ms */
	bool     valid;
};

/* ─── Global state ─── */
extern struct imu_reading g_imu;
extern struct gps_data    g_gps;
extern volatile bool      g_is_logging;
extern float              g_battery_voltage;
extern volatile float     g_audio_rms;
extern volatile int       g_audio_peak;
extern volatile int       g_audio_zcr;
extern volatile float     g_audio_baseline;

/* ─── Session stats ─── */
extern float    g_max_speed_session;
extern uint8_t  g_impact_count;
extern uint16_t g_move_count;
extern int64_t  g_log_start_time;
extern uint32_t g_sample_count;

#endif /* COMOTION_COMMON_H */
