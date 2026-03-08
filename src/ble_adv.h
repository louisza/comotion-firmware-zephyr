/*
 * CoMotion Tracker — BLE Advertising Module
 *
 * Builds and broadcasts a 27-byte manufacturer-specific advertising packet.
 * The companion app reads this via passive BLE scanning (no connection needed).
 *
 * GPS coordinates are absolute int32 values in deci-microdegrees (×10⁷),
 * so the app just divides by 10000000.0 — no field-center needed.
 *
 * Also manages advertising parameters (normal vs focus mode) and
 * NUS (Nordic UART Service) for commands.
 */

#ifndef BLE_ADV_H
#define BLE_ADV_H

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/bluetooth/conn.h>

/*
 * 27-byte BLE broadcast packet
 *
 * Transmitted as manufacturer-specific data in a Coded PHY extended
 * advertising set.  The app decodes GPS by dividing int32 by 10⁷.
 */
struct ble_packet {
	uint8_t  status_flags;    /* Byte 0:  bitfield */
	uint8_t  battery_pct;     /* Byte 1:  0–100 */
	uint8_t  intensity_1s;    /* Byte 2:  min(current×100, 255) */
	uint8_t  intensity_1min;  /* Byte 3:  min(1min×100, 255) */
	uint16_t intensity_10min; /* Byte 4-5: LE, min(avg×1000, 65535) */
	uint8_t  speed_now;       /* Byte 6:  min(km/h×2, 255) */
	uint8_t  speed_max;       /* Byte 7:  min(maxKmh×2, 255) */
	uint8_t  impact_count;    /* Byte 8:  session total */
	uint8_t  gps_status;      /* Byte 9:  (age<<4) | sats */
	uint16_t move_count;      /* Byte 10-11: LE */
	uint16_t session_seconds; /* Byte 12-13: LE, 0 if not logging */
	uint8_t  audio_peak;      /* Byte 14: min(rawPeak/128, 255) */
	int32_t  latitude;        /* Byte 15-18: LE, deci-microdeg (÷10⁷→°) */
	int32_t  longitude;       /* Byte 19-22: LE, deci-microdeg (÷10⁷→°) */
	uint16_t bearing;         /* Byte 23-24: LE, ×10 (0–3600 = 0.0–360.0°) */
	uint8_t  hdop;            /* Byte 25: ×10 (0–255 = 0.0–25.5 HDOP) */
	uint8_t  fix_quality;     /* Byte 26: 0=none,1=SPS,2=DGNSS,3=PPS,4=RTK */
} __packed;

/* Status flag bits */
#define STATUS_LOGGING    0x01
#define STATUS_GPS_FIX    0x02
#define STATUS_BATT_LOW   0x04
#define STATUS_IMPACT     0x08
#define STATUS_CONNECTED  0x10
#define STATUS_FOCUS      0x20

/* GPS no-fix sentinel — app checks: if (lat == GPS_NO_FIX) → no position */
#define GPS_NO_FIX        0x7FFFFFFF

/**
 * Initialize BLE: enable Bluetooth, configure NUS, start advertising.
 * NUS received callback will be set to the provided handler.
 *
 * @param rx_cb  Function called when data received over NUS
 * @return 0 on success, negative errno on failure
 */
int ble_adv_init(void (*rx_cb)(struct bt_conn *, const uint8_t *, uint16_t));

/**
 * Update the advertising packet with current sensor data.
 * Call at 5 Hz (normal) or 10 Hz (focus mode).
 *
 * @param pkt  Pointer to fully-populated 27-byte packet struct
 */
void ble_adv_update(const struct ble_packet *pkt);

/**
 * Enable focus mode: 100ms advertising interval, auto-expires in 60s.
 */
void ble_adv_set_focus(bool enable);

/** Check if focus mode is currently active. */
bool ble_adv_is_focus(void);

/** Check if a BLE central is connected. */
bool ble_adv_is_connected(void);

/** Get the current connection (NULL if none). */
struct bt_conn *ble_adv_get_conn(void);

/**
 * Send data over NUS (chunked ≤20 bytes, 5ms inter-chunk delay).
 *
 * @param msg  Null-terminated string to send
 */
void ble_adv_nus_send(const char *msg);

/**
 * Send raw binary data over NUS (for file transfer).
 * Sends in one call; caller must ensure len ≤ NUS MTU (≤240 bytes).
 *
 * @param data  Pointer to raw bytes
 * @param len   Number of bytes to send
 */
void ble_adv_nus_send_raw(const uint8_t *data, uint16_t len);

#endif /* BLE_ADV_H */
