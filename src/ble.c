/*
 * ═══════════════════════════════════════════════════════════════
 * CoMotion Tracker — BLE (Dual Advertising + NUS)
 *
 * Two simultaneous advertising sets:
 *   1. Legacy 1M PHY   — connectable, for NUS commands
 *   2. Coded PHY (S8)  — non-connectable, long range data
 *
 * Both carry the same 20-byte manufacturer data payload.
 * TX power: +8 dBm (via CONFIG_BT_CTLR_TX_PWR_PLUS_8)
 * ═══════════════════════════════════════════════════════════════
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <bluetooth/services/nus.h>
#include <zephyr/logging/log.h>
#include <math.h>

#include "common.h"
#include "ble.h"
#include "intensity.h"

LOG_MODULE_REGISTER(ble, LOG_LEVEL_INF);

/* Forward declaration */
extern void process_command(const char *cmd);

/* ─── State ─── */
static struct bt_le_ext_adv *adv_legacy;
static struct bt_le_ext_adv *adv_coded;
static struct bt_conn *current_conn;
static bool ble_connected;
static int64_t impact_flag_time;

/* ─── Manufacturer data: 2 bytes company ID + 20 bytes payload ─── */
static uint8_t mfg_data[22];

/* ─── Advertising data arrays ─── */
static struct bt_data ad_legacy[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, sizeof(mfg_data)),
};

static struct bt_data ad_coded[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, sizeof(mfg_data)),
};

static struct bt_data sd_data[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, sizeof(DEVICE_NAME) - 1),
};

/* ═══════════════════════════════════════════════════════════════
 * Connection Callbacks
 * ═══════════════════════════════════════════════════════════════ */

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_ERR("[BLE] Connection failed: 0x%02x", err);
		return;
	}
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("[BLE] Connected: %s", addr);
	current_conn = bt_conn_ref(conn);
	ble_connected = true;
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("[BLE] Disconnected: 0x%02x", reason);
	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
	ble_connected = false;

	/* Restart legacy advertising after disconnect */
	int err = bt_le_ext_adv_start(adv_legacy, BT_LE_EXT_ADV_START_DEFAULT);
	if (err) {
		LOG_ERR("[BLE] Failed to restart legacy adv: %d", err);
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected_cb,
	.disconnected = disconnected_cb,
};

/* ═══════════════════════════════════════════════════════════════
 * NUS Callbacks
 * ═══════════════════════════════════════════════════════════════ */

static void nus_received_cb(struct bt_conn *conn, const uint8_t *data,
			     uint16_t len)
{
	char cmd[128];
	int copy_len = MIN(len, sizeof(cmd) - 1);
	memcpy(cmd, data, copy_len);
	cmd[copy_len] = '\0';

	/* Trim trailing whitespace */
	while (copy_len > 0 &&
	       (cmd[copy_len - 1] == '\n' || cmd[copy_len - 1] == '\r' ||
		cmd[copy_len - 1] == ' ')) {
		cmd[--copy_len] = '\0';
	}

	process_command(cmd);
}

static struct bt_nus_cb nus_cb = {
	.received = nus_received_cb,
};

/* ═══════════════════════════════════════════════════════════════
 * Advertising Setup
 * ═══════════════════════════════════════════════════════════════ */

static int create_legacy_adv(void)
{
	int err;
	struct bt_le_adv_param param = BT_LE_ADV_PARAM_INIT(
		BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_EXT_ADV,
		800, 800, /* 500ms interval (800 × 0.625ms) */
		NULL);

	err = bt_le_ext_adv_create(&param, NULL, &adv_legacy);
	if (err) {
		LOG_ERR("Failed to create legacy adv set: %d", err);
		return err;
	}

	err = bt_le_ext_adv_set_data(adv_legacy, ad_legacy, ARRAY_SIZE(ad_legacy),
				     sd_data, ARRAY_SIZE(sd_data));
	if (err) {
		LOG_ERR("Failed to set legacy adv data: %d", err);
		return err;
	}

	return 0;
}

static int create_coded_adv(void)
{
	int err;
	struct bt_le_adv_param param = BT_LE_ADV_PARAM_INIT(
		BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_CODED,
		800, 800, /* 500ms interval */
		NULL);

	err = bt_le_ext_adv_create(&param, NULL, &adv_coded);
	if (err) {
		LOG_ERR("Failed to create coded adv set: %d", err);
		return err;
	}

	err = bt_le_ext_adv_set_data(adv_coded, ad_coded, ARRAY_SIZE(ad_coded),
				     NULL, 0);
	if (err) {
		LOG_ERR("Failed to set coded adv data: %d", err);
		return err;
	}

	return 0;
}

/* ═══════════════════════════════════════════════════════════════
 * Public API
 * ═══════════════════════════════════════════════════════════════ */

int ble_init(void)
{
	int err;

	/* Initialize manufacturer data company ID (0xFFFF LE) */
	mfg_data[0] = BLE_MANUFACTURER_ID & 0xFF;
	mfg_data[1] = (BLE_MANUFACTURER_ID >> 8) & 0xFF;

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("bt_enable failed: %d", err);
		return err;
	}

	/* NUS service */
	err = bt_nus_init(&nus_cb);
	if (err) {
		LOG_ERR("bt_nus_init failed: %d", err);
		return err;
	}

	/* Create both advertising sets */
	err = create_legacy_adv();
	if (err) return err;

	err = create_coded_adv();
	if (err) return err;

	/* Start both */
	err = bt_le_ext_adv_start(adv_legacy, BT_LE_EXT_ADV_START_DEFAULT);
	if (err) {
		LOG_ERR("Failed to start legacy adv: %d", err);
		return err;
	}
	LOG_INF("[BLE] Legacy 1M advertising started");

	err = bt_le_ext_adv_start(adv_coded, BT_LE_EXT_ADV_START_DEFAULT);
	if (err) {
		LOG_ERR("Failed to start coded adv: %d", err);
		return err;
	}
	LOG_INF("[BLE] Coded PHY (S8) advertising started");

	return 0;
}

bool ble_is_connected(void)
{
	return ble_connected;
}

void ble_send(const char *msg)
{
	if (!current_conn) return;

	int len = strlen(msg);
	int offset = 0;

	while (offset < len) {
		int chunk = MIN(20, len - offset);
		bt_nus_send(current_conn, (const uint8_t *)(msg + offset), chunk);
		offset += chunk;
		k_msleep(5);
	}
	bt_nus_send(current_conn, (const uint8_t *)"\n", 1);
}

void ble_set_impact_flag(void)
{
	impact_flag_time = k_uptime_get();
	if (g_impact_count < 255) {
		g_impact_count++;
	}
}

void ble_update_advertising(void)
{
	int64_t now = k_uptime_get();
	uint8_t *d = &mfg_data[2]; /* Skip 2-byte company ID */

	/* Impact flag auto-clear */
	bool impact_active = (impact_flag_time > 0) &&
			     (now - impact_flag_time < IMPACT_FLAG_DURATION_MS);

	/* Status flags */
	uint8_t flags = 0;
	if (g_is_logging)                                 flags |= STATUS_FLAG_LOGGING;
	if (g_gps.valid && g_gps.satellites >= 4)         flags |= STATUS_FLAG_GPS_FIX;
	if (g_battery_voltage > 0 && g_battery_voltage < 3.3f)
		flags |= STATUS_FLAG_LOW_BATTERY;
	if (impact_active)                                flags |= STATUS_FLAG_IMPACT;
	if (ble_connected)                                flags |= STATUS_FLAG_BLE_CONN;

	float batt_pct = CLAMP((g_battery_voltage - 3.3f) / 0.9f * 100.0f, 0, 100);
	uint8_t intensity_now = (uint8_t)CLAMP((int)(intensity_get_current() * 100), 0, 255);
	uint8_t intensity_1m  = (uint8_t)CLAMP((int)(intensity_get_1min() * 100), 0, 255);
	uint16_t intensity_10m = (uint16_t)CLAMP(
		(int)(intensity_get_10min_avg() * 1000), 0, 65535);

	/* Speed at 0.5 km/h resolution: byte = speed × 2 */
	uint8_t speed_now = (uint8_t)CLAMP((int)(g_gps.speed * 2.0f), 0, 255);
	uint8_t speed_max = (uint8_t)CLAMP((int)(g_max_speed_session * 2.0f), 0, 255);

	uint8_t gps_age = (uint8_t)CLAMP(
		(int)((now - g_gps.last_update) / 1000), 0, 15);
	uint8_t gps_sats = (uint8_t)CLAMP(g_gps.satellites, 0, 15);
	uint8_t gps_status = (gps_age << 4) | gps_sats;

	uint16_t session_sec = 0;
	if (g_is_logging) {
		session_sec = (uint16_t)CLAMP(
			(int)((now - g_log_start_time) / 1000), 0, 65535);
	}

	uint8_t audio_peak_scaled = (uint8_t)CLAMP(g_audio_peak / 128, 0, 255);

	/* Pack 20-byte payload (same format as Arduino firmware) */
	d[0]  = flags;
	d[1]  = (uint8_t)batt_pct;
	d[2]  = intensity_now;
	d[3]  = intensity_1m;
	d[4]  = intensity_10m & 0xFF;
	d[5]  = (intensity_10m >> 8) & 0xFF;
	d[6]  = speed_now;
	d[7]  = speed_max;
	d[8]  = g_impact_count;
	d[9]  = gps_status;
	d[10] = g_move_count & 0xFF;
	d[11] = (g_move_count >> 8) & 0xFF;
	d[12] = session_sec & 0xFF;
	d[13] = (session_sec >> 8) & 0xFF;
	d[14] = audio_peak_scaled;

	/* GPS position: int16 offsets from field center */
	if (g_gps.valid) {
		int16_t lat_off = (int16_t)CLAMP(
			(g_gps.latitude - FIELD_CENTER_LAT) * 100000.0, -32767, 32767);
		int16_t lng_off = (int16_t)CLAMP(
			(g_gps.longitude - FIELD_CENTER_LNG) * 100000.0, -32767, 32767);
		d[15] = lat_off & 0xFF;
		d[16] = (lat_off >> 8) & 0xFF;
		d[17] = lng_off & 0xFF;
		d[18] = (lng_off >> 8) & 0xFF;
	} else {
		/* Sentinel: 0x7FFF = no position */
		d[15] = 0xFF; d[16] = 0x7F;
		d[17] = 0xFF; d[18] = 0x7F;
	}
	d[19] = 0;

	/* Update both advertising sets — no stop/start needed! */
	bt_le_ext_adv_set_data(adv_legacy, ad_legacy, ARRAY_SIZE(ad_legacy),
			       sd_data, ARRAY_SIZE(sd_data));
	bt_le_ext_adv_set_data(adv_coded, ad_coded, ARRAY_SIZE(ad_coded),
			       NULL, 0);
}
