/*
 * CoMotion Tracker — BLE Advertising Module (Coded PHY)
 *
 * Dual-advertising architecture for maximum range on a hockey field:
 *
 *   Set 1 — BROADCAST (Coded PHY, non-connectable)
 *     • 27-byte manufacturer data with live telemetry
 *     • Always running, even during a connection
 *     • Coded PHY S=8 for ~4× range over 1M PHY
 *     • Updated at 5 Hz (normal) or ~20 Hz (focus)
 *
 *   Set 2 — CONNECTABLE (1M PHY)
 *     • Advertises NUS UUID so phones can connect for commands
 *     • Stops when a connection is active, restarts on disconnect
 *     • 1M PHY so any phone can discover and connect
 *
 * The broadcast set never stops — the sideline phone always sees
 * live data even at 40m+, regardless of connection state.
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/conn.h>
#include <bluetooth/services/nus.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdio.h>

#include "ble_adv.h"
#include "device_id.h"

LOG_MODULE_REGISTER(ble_adv, CONFIG_LOG_DEFAULT_LEVEL);

/* ─── Constants ─── */
#define MFG_ID_LOW              0xFF
#define MFG_ID_HIGH             0xFF
#define ADV_INTERVAL_NORMAL     320     /* 200ms in 0.625ms units */
#define ADV_INTERVAL_FOCUS      80      /* 50ms in 0.625ms units */
#define CONN_ADV_INTERVAL       800     /* 500ms for connectable set */
#define FOCUS_TIMEOUT_MS        60000

/* ─── Manufacturer data: 2-byte company ID + 27-byte payload ─── */
static uint8_t mfg_data[2 + sizeof(struct ble_packet)];

/* ─── Dynamic BLE name: "CoMotion-XXXX" (max 14 chars + NUL) ─── */
static char ble_name[16] = "CoMotion";
static uint8_t ble_name_len = 8;

/* ─── Extended advertising sets ─── */
static struct bt_le_ext_adv *bcast_adv;   /* Coded PHY broadcast */
static struct bt_le_ext_adv *conn_adv;    /* 1M connectable for NUS */
static bool bcast_running;                /* true when broadcast set is active */

/* ─── Broadcast advertising data (Coded PHY, non-connectable) ─── */
/* No Flags — Flags SHALL NOT be present in non-connectable extended
 * advertising (BT Core Spec Vol 3, Part C, §11.1.3).
 * Device name IS included so the app can identify CoMotion broadcasts.
 * Name data/len patched at init time with device ID suffix.
 */
static struct bt_data bcast_ad[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, ble_name, 8),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, sizeof(mfg_data)),
};

/* ─── Connectable advertising data (1M PHY) ─── */
static struct bt_data conn_ad_data[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS,
		      (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, ble_name, 8),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

/* ─── Connection state ─── */
static struct bt_conn *current_conn;
static bool connected;

/* ─── Focus mode ─── */
static bool focus_active;
static int64_t focus_expire_time;

/* ─── Track connectable advertising state ─── */
static bool conn_adv_started;

/* ─── Deferred connectable advertising restart ─── */
static void conn_adv_restart_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(conn_adv_restart_work, conn_adv_restart_handler);

static void conn_adv_restart_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (connected) {
		return;
	}

	int ret = bt_le_ext_adv_start(conn_adv, BT_LE_EXT_ADV_START_DEFAULT);

	if (ret == 0 || ret == -EALREADY) {
		conn_adv_started = true;
		LOG_INF("BLE: connectable adv restarted");
	} else {
		LOG_WRN("BLE: connectable adv restart failed: %d, "
			"retrying in 1s", ret);
		k_work_schedule(&conn_adv_restart_work, K_SECONDS(1));
	}
}

/* ─── BLE Connection Callbacks ─── */

static void on_connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_ERR("BLE: connection failed (err %u)", err);
		return;
	}

	current_conn = bt_conn_ref(conn);
	connected = true;
	conn_adv_started = false;
	LOG_INF("BLE: connected");

	/* Request optimized connection parameters:
	 * 30-50ms interval, 0 latency, 5s supervision timeout
	 */
	static const struct bt_le_conn_param conn_param =
		BT_LE_CONN_PARAM_INIT(24, 40, 0, 500);
	int ret = bt_conn_le_param_update(conn, &conn_param);

	if (ret && ret != -EALREADY) {
		LOG_WRN("BLE: conn param update failed: %d", ret);
	}

	/* Request Coded PHY for the connection too */
	static const struct bt_conn_le_phy_param phy_param = {
		.options = BT_CONN_LE_PHY_OPT_CODED_S8,
		.pref_tx_phy = BT_GAP_LE_PHY_CODED,
		.pref_rx_phy = BT_GAP_LE_PHY_CODED,
	};

	ret = bt_conn_le_phy_update(conn, &phy_param);
	if (ret && ret != -EALREADY) {
		LOG_WRN("BLE: PHY update failed: %d", ret);
	}

	/* Note: broadcast adv set keeps running — no interruption */
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("BLE: disconnected (reason %u)", reason);
	connected = false;

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}

	/* Restart connectable advertising after a short delay */
	k_work_schedule(&conn_adv_restart_work, K_MSEC(100));
}

static void on_phy_updated(struct bt_conn *conn,
			   struct bt_conn_le_phy_info *param)
{
	const char *tx_name, *rx_name;

	switch (param->tx_phy) {
	case BT_GAP_LE_PHY_1M:    tx_name = "1M"; break;
	case BT_GAP_LE_PHY_2M:    tx_name = "2M"; break;
	case BT_GAP_LE_PHY_CODED: tx_name = "Coded"; break;
	default:                   tx_name = "?"; break;
	}

	switch (param->rx_phy) {
	case BT_GAP_LE_PHY_1M:    rx_name = "1M"; break;
	case BT_GAP_LE_PHY_2M:    rx_name = "2M"; break;
	case BT_GAP_LE_PHY_CODED: rx_name = "Coded"; break;
	default:                   rx_name = "?"; break;
	}

	LOG_INF("BLE: PHY updated -> TX:%s RX:%s", tx_name, rx_name);
}

BT_CONN_CB_DEFINE(conn_cbs) = {
	.connected        = on_connected,
	.disconnected     = on_disconnected,
	.le_phy_updated   = on_phy_updated,
};

/* ─── Broadcast interval update (for focus mode) ─── */

static int update_broadcast_interval(uint16_t interval)
{
	int ret;

	/* Must stop → reconfigure → restart to change interval */
	bt_le_ext_adv_stop(bcast_adv);
	bcast_running = false;

	struct bt_le_adv_param param = BT_LE_ADV_PARAM_INIT(
		BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_CODED | BT_LE_ADV_OPT_USE_IDENTITY,
		interval, interval, NULL);

	ret = bt_le_ext_adv_update_param(bcast_adv, &param);
	if (ret) {
		LOG_ERR("BLE: broadcast param update failed: %d", ret);
		/* Fall through — try to restart with old params rather
		 * than leaving the broadcast permanently stopped.
		 */
	}

	/* Retry start up to 3 times — radio may be busy with other set */
	for (int attempt = 0; attempt < 3; attempt++) {
		ret = bt_le_ext_adv_start(bcast_adv,
					  BT_LE_EXT_ADV_START_DEFAULT);
		if (ret == 0 || ret == -EALREADY) {
			bcast_running = true;
			return 0;
		}
		LOG_WRN("BLE: broadcast restart attempt %d/3 failed: %d",
			attempt + 1, ret);
		k_msleep(10);
	}

	LOG_ERR("BLE: broadcast restart FAILED — will recover on next update");
	return ret;
}

/* ─── Public API ─── */

int ble_adv_init(void (*rx_cb)(struct bt_conn *, const uint8_t *, uint16_t))
{
	int ret;

	/* Initialize manufacturer data with company ID */
	mfg_data[0] = MFG_ID_LOW;
	mfg_data[1] = MFG_ID_HIGH;
	memset(&mfg_data[2], 0, sizeof(struct ble_packet));

	/* Build dynamic BLE name: "CoMotion-XXXX" using device ID */
	snprintf(ble_name, sizeof(ble_name), "CoMotion-%s", device_id_short());
	ble_name_len = strlen(ble_name);

	/* Patch advertising data structs with actual name length */
	bcast_ad[0].data_len = ble_name_len;
	conn_ad_data[1].data_len = ble_name_len;

	/* Also set the GAP device name so connected clients see it */
	bt_set_name(ble_name);

	LOG_INF("BLE name: %s", ble_name);

	/* Enable Bluetooth */
	ret = bt_enable(NULL);
	if (ret) {
		LOG_ERR("BLE: bt_enable failed: %d", ret);
		return ret;
	}

	/* Initialize NUS */
	static struct bt_nus_cb nus_cbs;

	nus_cbs.received = rx_cb;
	nus_cbs.send_enabled = NULL;

	ret = bt_nus_init(&nus_cbs);
	if (ret) {
		LOG_ERR("BLE: NUS init failed: %d", ret);
		return ret;
	}

	/* ─── Set 1: Coded PHY broadcast (non-connectable, always on) ─── */
	struct bt_le_adv_param bcast_param = BT_LE_ADV_PARAM_INIT(
		BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_CODED | BT_LE_ADV_OPT_USE_IDENTITY,
		ADV_INTERVAL_NORMAL, ADV_INTERVAL_NORMAL, NULL);

	ret = bt_le_ext_adv_create(&bcast_param, NULL, &bcast_adv);
	if (ret) {
		LOG_ERR("BLE: broadcast adv create failed: %d", ret);
		return ret;
	}

	ret = bt_le_ext_adv_set_data(bcast_adv,
				     bcast_ad, ARRAY_SIZE(bcast_ad),
				     NULL, 0);
	if (ret) {
		LOG_ERR("BLE: broadcast set data failed: %d", ret);
		return ret;
	}

	ret = bt_le_ext_adv_start(bcast_adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (ret) {
		LOG_ERR("BLE: broadcast start failed: %d", ret);
		return ret;
	}
	bcast_running = true;

	LOG_INF("BLE: broadcast started (Coded PHY, 200ms)");

	/* ─── Set 2: 1M connectable (for NUS commands) ─── */
	struct bt_le_adv_param conn_param = BT_LE_ADV_PARAM_INIT(
		BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_USE_IDENTITY,
		CONN_ADV_INTERVAL, CONN_ADV_INTERVAL, NULL);

	ret = bt_le_ext_adv_create(&conn_param, NULL, &conn_adv);
	if (ret) {
		LOG_ERR("BLE: connectable adv create failed: %d", ret);
		return ret;
	}

	ret = bt_le_ext_adv_set_data(conn_adv,
				     conn_ad_data, ARRAY_SIZE(conn_ad_data),
				     NULL, 0);
	if (ret) {
		LOG_ERR("BLE: connectable set data failed: %d", ret);
		return ret;
	}

	ret = bt_le_ext_adv_start(conn_adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (ret) {
		LOG_ERR("BLE: connectable start failed: %d", ret);
		return ret;
	}
	conn_adv_started = true;

	LOG_INF("BLE: connectable started (1M PHY, NUS)");
	LOG_INF("BLE: advertising as \"%s\"", CONFIG_BT_DEVICE_NAME);

	return 0;
}

void ble_adv_update(const struct ble_packet *pkt)
{
	/* Check focus mode expiry */
	if (focus_active && k_uptime_get() >= focus_expire_time) {
		LOG_INF("BLE: focus mode expired -> normal");
		focus_active = false;
		update_broadcast_interval(ADV_INTERVAL_NORMAL);
	}

	/* If broadcast died (e.g. failed restart), try to recover */
	if (!bcast_running) {
		int rc = bt_le_ext_adv_start(bcast_adv,
					     BT_LE_EXT_ADV_START_DEFAULT);
		if (rc == 0 || rc == -EALREADY) {
			bcast_running = true;
			LOG_INF("BLE: broadcast recovered");
		}
	}

	/* Copy packet into manufacturer data (after the 2-byte company ID) */
	memcpy(&mfg_data[2], pkt, sizeof(struct ble_packet));

	/* Push updated data to the controller */
	int ret = bt_le_ext_adv_set_data(bcast_adv,
					 bcast_ad, ARRAY_SIZE(bcast_ad),
					 NULL, 0);
	if (ret == -EAGAIN) {
		/* Controller mid-TX — data is in mfg_data and will be
		 * picked up on the next call (~200ms later at worst).
		 */
	} else if (ret) {
		LOG_WRN("BLE: set_data failed: %d — marking for recovery", ret);
		bcast_running = false;
	}
}

void ble_adv_set_focus(bool enable)
{
	if (enable) {
		focus_active = true;
		focus_expire_time = k_uptime_get() + FOCUS_TIMEOUT_MS;
		LOG_INF("BLE: focus mode ON (60s)");
		update_broadcast_interval(ADV_INTERVAL_FOCUS);
	} else {
		focus_active = false;
		LOG_INF("BLE: normal mode");
		update_broadcast_interval(ADV_INTERVAL_NORMAL);
	}
}

bool ble_adv_is_focus(void)
{
	return focus_active;
}

bool ble_adv_is_connected(void)
{
	return connected;
}

struct bt_conn *ble_adv_get_conn(void)
{
	return current_conn;
}

void ble_adv_nus_send(const char *msg)
{
	if (!connected || !current_conn) {
		return;
	}

	/* Send message + newline terminator so app can detect line boundaries */
	char buf[258];
	int len = snprintf(buf, sizeof(buf), "%s\n", msg);
	if (len <= 0 || len >= (int)sizeof(buf)) {
		return;
	}
	int off = 0;

	while (off < len) {
		int chunk = MIN(240, len - off);
		int ret = -ENOMEM;

		/* Retry up to 3 times on TX buffer full (-ENOMEM) */
		for (int attempt = 0; attempt < 3; attempt++) {
			ret = bt_nus_send(current_conn,
					  (const uint8_t *)buf + off, chunk);
			if (ret != -ENOMEM) {
				break;
			}
			k_msleep(10);
		}

		if (ret) {
			LOG_WRN("NUS send failed: %d (dropped %d/%d bytes)",
				ret, len - off, len);
			return;
		}
		off += chunk;
		if (off < len) {
			k_msleep(5);
		}
	}
}

void ble_adv_nus_send_raw(const uint8_t *data, uint16_t len)
{
	if (!connected || !current_conn || !data || len == 0) {
		return;
	}

	int ret = -ENOMEM;

	/* Retry up to 3 times on TX buffer full (-ENOMEM) */
	for (int attempt = 0; attempt < 3; attempt++) {
		ret = bt_nus_send(current_conn, data, len);
		if (ret != -ENOMEM) {
			break;
		}
		k_msleep(10);
	}

	if (ret) {
		LOG_WRN("NUS raw send failed: %d (dropped %u bytes)", ret, len);
	}
}
