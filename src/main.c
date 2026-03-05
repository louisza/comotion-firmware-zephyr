/*
 * CoMotion Tracker - Step 7: LED + IMU + BLE + GPS + PDM Mic + SD Card + Battery
 *
 * XIAO BLE Sense hardware:
 *   LEDs: Red (P0.26), Green (P0.30), Blue (P0.06) — active LOW
 *   IMU:  LSM6DS3TR-C on I2C0 @ 0x6A
 *   BLE:  Advertising as "CoMotion" with NUS
 *   GPS:  ATGM332D on UART0 (P1.11/P1.12) @ 9600 baud
 *   Mic:  MSM261D3526HICPM-C PDM (CLK=P1.00, DIN=P0.16)
 *   SD:   External SD card on SPI2, CS=P0.28 (XIAO D2)
 *   Batt: LiPo via voltage divider on AIN7 (P0.31), enable P0.14
 *
 * Blue LED  = BLE connected
 * Green LED = IMU read in progress
 * Red LED   = error
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/conn.h>
#include <bluetooth/services/nus.h>
#include <stdio.h>

#include "gps.h"
#include "pdm_mic.h"
#include "sdcard.h"
#include "battery.h"

/* ─── LEDs ─── */
static const struct gpio_dt_spec led_red   = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec led_blue  = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);

/* ─── IMU ─── */
static const struct device *imu;

/* ─── BLE state ─── */
static struct bt_conn *current_conn;
static bool nus_notifications_enabled;

/* ─── BLE Advertising Data ─── */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

/* ─── BLE Connection Callbacks ─── */
static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("BLE: connection failed (err %u)\n", err);
		return;
	}
	current_conn = bt_conn_ref(conn);
	gpio_pin_set_dt(&led_blue, 1);
	printk("BLE: connected\n");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("BLE: disconnected (reason %u)\n", reason);
	gpio_pin_set_dt(&led_blue, 0);
	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
	nus_notifications_enabled = false;

	/* Restart advertising */
	bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
}

BT_CONN_CB_DEFINE(conn_cbs) = {
	.connected    = connected,
	.disconnected = disconnected,
};

/* ─── NUS Callbacks ─── */
static void nus_received(struct bt_conn *conn, const uint8_t *const data, uint16_t len)
{
	char buf[64];
	uint16_t copy_len = (len < sizeof(buf) - 1) ? len : sizeof(buf) - 1;

	memcpy(buf, data, copy_len);
	buf[copy_len] = '\0';

	printk("BLE RX: %s\n", buf);

	/* Echo back for testing */
	char reply[80];
	int n = snprintf(reply, sizeof(reply), "echo: %s\n", buf);
	bt_nus_send(conn, (uint8_t *)reply, n);
}

static void nus_send_enabled(enum bt_nus_send_status status)
{
	nus_notifications_enabled = (status == BT_NUS_SEND_STATUS_ENABLED);
	printk("BLE: NUS notifications %s\n",
		nus_notifications_enabled ? "enabled" : "disabled");
}

static struct bt_nus_cb nus_cbs = {
	.received     = nus_received,
	.send_enabled = nus_send_enabled,
};

/* ─── IMU ─── */
static int init_imu(void)
{
	imu = DEVICE_DT_GET_ONE(st_lsm6dsl);
	if (!device_is_ready(imu)) {
		printk("IMU: device not ready!\n");
		return -ENODEV;
	}

	struct sensor_value val;

	val.val1 = 16;
	val.val2 = 0;
	sensor_attr_set(imu, SENSOR_CHAN_ACCEL_XYZ,
		SENSOR_ATTR_FULL_SCALE, &val);

	val.val1 = 1000;
	val.val2 = 0;
	sensor_attr_set(imu, SENSOR_CHAN_GYRO_XYZ,
		SENSOR_ATTR_FULL_SCALE, &val);

	val.val1 = 104;
	val.val2 = 0;
	sensor_attr_set(imu, SENSOR_CHAN_ACCEL_XYZ,
		SENSOR_ATTR_SAMPLING_FREQUENCY, &val);
	sensor_attr_set(imu, SENSOR_CHAN_GYRO_XYZ,
		SENSOR_ATTR_SAMPLING_FREQUENCY, &val);

	printk("IMU: LSM6DS3TR-C ready (16g, 1000dps, 104Hz)\n");
	return 0;
}

static void read_and_send_imu(int cycle)
{
	struct sensor_value accel[3], gyro[3];

	if (sensor_sample_fetch(imu)) {
		printk("[%d] IMU: fetch failed\n", cycle);
		return;
	}

	sensor_channel_get(imu, SENSOR_CHAN_ACCEL_X, &accel[0]);
	sensor_channel_get(imu, SENSOR_CHAN_ACCEL_Y, &accel[1]);
	sensor_channel_get(imu, SENSOR_CHAN_ACCEL_Z, &accel[2]);
	sensor_channel_get(imu, SENSOR_CHAN_GYRO_X, &gyro[0]);
	sensor_channel_get(imu, SENSOR_CHAN_GYRO_Y, &gyro[1]);
	sensor_channel_get(imu, SENSOR_CHAN_GYRO_Z, &gyro[2]);

	/* Print to USB serial */
	printk("[%d] A: %d.%02d %d.%02d %d.%02d  G: %d.%02d %d.%02d %d.%02d\n",
		cycle,
		accel[0].val1, abs(accel[0].val2) / 10000,
		accel[1].val1, abs(accel[1].val2) / 10000,
		accel[2].val1, abs(accel[2].val2) / 10000,
		gyro[0].val1, abs(gyro[0].val2) / 10000,
		gyro[1].val1, abs(gyro[1].val2) / 10000,
		gyro[2].val1, abs(gyro[2].val2) / 10000);

	/* Also send over BLE if connected + notifications on */
	if (nus_notifications_enabled) {
		char msg[120];
		int n = snprintf(msg, sizeof(msg),
			"A:%d.%02d,%d.%02d,%d.%02d G:%d.%02d,%d.%02d,%d.%02d\n",
			accel[0].val1, abs(accel[0].val2) / 10000,
			accel[1].val1, abs(accel[1].val2) / 10000,
			accel[2].val1, abs(accel[2].val2) / 10000,
			gyro[0].val1, abs(gyro[0].val2) / 10000,
			gyro[1].val1, abs(gyro[1].val2) / 10000,
			gyro[2].val1, abs(gyro[2].val2) / 10000);
		bt_nus_send(NULL, (uint8_t *)msg, n);
	}
}

/* ─── Main ─── */
int main(void)
{
	int err;
	int cycle = 0;

	/* Give USB 2 seconds to enumerate */
	k_msleep(2000);

	printk("\n\n=== CoMotion Step 7: LED + IMU + BLE + GPS + Mic + SD + Batt ===\n");

	/* --- LEDs --- */
	err  = gpio_pin_configure_dt(&led_red,   GPIO_OUTPUT_INACTIVE);
	err |= gpio_pin_configure_dt(&led_green, GPIO_OUTPUT_INACTIVE);
	err |= gpio_pin_configure_dt(&led_blue,  GPIO_OUTPUT_INACTIVE);
	if (err) {
		printk("LED config failed: %d\n", err);
		return err;
	}
	printk("LEDs: OK\n");

	/* --- IMU --- */
	err = init_imu();
	if (err) {
		printk("IMU init failed — continuing without IMU\n");
	}

	/* --- GPS --- */
	/* GNSS framework auto-initializes from device tree.
	 * Data arrives via GNSS_DATA_CALLBACK_DEFINE in gps.c */
	{
		const struct device *gnss = DEVICE_DT_GET(DT_NODELABEL(gnss));
		if (device_is_ready(gnss)) {
			printk("GPS: GNSS device ready on UART0 (9600 baud)\n");
		} else {
			printk("GPS: ERROR - GNSS device NOT ready!\n");
			gpio_pin_set_dt(&led_red, 1);
		}
	}

	/* --- BLE --- */
	err = bt_enable(NULL);
	if (err) {
		printk("BLE: bt_enable failed (err %d)\n", err);
		gpio_pin_set_dt(&led_red, 1);
		return err;
	}
	printk("BLE: stack enabled\n");

	err = bt_nus_init(&nus_cbs);
	if (err) {
		printk("BLE: NUS init failed (err %d)\n", err);
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		printk("BLE: advertising failed (err %d)\n", err);
	} else {
		printk("BLE: advertising as \"%s\"\n", CONFIG_BT_DEVICE_NAME);
	}

	/* --- PDM Microphone --- */
	err = pdm_mic_init();
	if (err) {
		printk("MIC: init failed (err %d) — continuing without mic\n", err);
	} else {
		err = pdm_mic_start();
		if (err) {
			printk("MIC: start failed (err %d)\n", err);
		}
	}

	/* --- SD Card --- */
	err = sdcard_init();
	if (err) {
		printk("SD: init failed (err %d) — continuing without SD\n", err);
	}

	/* --- Battery --- */
	err = battery_init();
	if (err) {
		printk("BATT: init failed (err %d) — continuing without battery\n", err);
	}

	printk("\nReady! Connect via nRF Connect app.\n");
	printk("IMU streams every 500ms. GPS/Mic/SD/Batt every 2s.\n\n");

	/* --- Main loop --- */
	while (1) {
		cycle++;

		gpio_pin_set_dt(&led_green, 1);

		/* IMU */
		if (imu && device_is_ready(imu)) {
			read_and_send_imu(cycle);
		}

		/* GPS — print every 2 seconds (every 4th cycle) */
		if ((cycle % 4) == 0) {
			struct gps_data gps;
			char gps_str[128];

			if (gps_get_data(&gps) == 0) {
				gps_format(&gps, gps_str, sizeof(gps_str));
				printk("[%d] %s\n", cycle, gps_str);

				/* Send GPS over BLE (fix or no-fix status) */
				if (nus_notifications_enabled) {
					int len = strlen(gps_str);
					gps_str[len] = '\n';
					bt_nus_send(NULL, (uint8_t *)gps_str, len + 1);
				}
			} else {
				uint32_t nmea_count = gps_get_update_count();
				if (nmea_count > 0) {
					snprintf(gps_str, sizeof(gps_str),
						"GPS: UART alive (%u NMEA msgs), parsing...\n",
						nmea_count);
				} else {
					snprintf(gps_str, sizeof(gps_str),
						"GPS: no NMEA data yet (check wiring)\n");
				}
				printk("[%d] %s", cycle, gps_str);

				if (nus_notifications_enabled) {
					bt_nus_send(NULL, (uint8_t *)gps_str,
						strlen(gps_str));
				}
			}

			/* Mic — read one block of audio and report level */
			struct mic_data mic;
			bool mic_ok = (pdm_mic_read(&mic, 200) == 0);
			if (mic_ok) {
				char mic_str[64];
				int n = snprintf(mic_str, sizeof(mic_str),
					"MIC: rms=%u peak=%d lvl=%u%%\n",
					mic.rms, mic.peak, mic.level_pct);
				printk("[%d] %s", cycle, mic_str);

				if (nus_notifications_enabled) {
					bt_nus_send(NULL, (uint8_t *)mic_str, n);
				}
			}

			/* Battery — sample and report */
			int batt_mv = 0, batt_pct = 0;
			if (battery_sample() == 0) {
				batt_mv = battery_millivolts();
				batt_pct = battery_level_pct();
				char batt_str[48];
				int n = snprintf(batt_str, sizeof(batt_str),
					"BATT: %d mV (%d%%)\n",
					batt_mv, batt_pct);
				printk("[%d] %s", cycle, batt_str);

				if (nus_notifications_enabled) {
					bt_nus_send(NULL, (uint8_t *)batt_str, n);
				}
			}

			/* SD Card — log a summary line */
			if (sdcard_is_mounted()) {
				char sd_line[200];
				struct gps_data gps_snap;
				bool have_gps = (gps_get_data(&gps_snap) == 0);
				int pos = 0;

				pos += snprintf(sd_line + pos,
					sizeof(sd_line) - pos,
					"%lld,", k_uptime_get());
				if (have_gps) {
					pos += snprintf(sd_line + pos,
						sizeof(sd_line) - pos,
						"%.6f,%.6f,%d,%u,",
						gps_snap.latitude_ndeg / 1e9,
						gps_snap.longitude_ndeg / 1e9,
						gps_snap.altitude_mm / 1000,
						gps_snap.satellites);
				} else {
					pos += snprintf(sd_line + pos,
						sizeof(sd_line) - pos,
						",,,,");
				}
				snprintf(sd_line + pos,
					sizeof(sd_line) - pos,
					"%u,%d,%u,%d,%d",
					mic_ok ? mic.rms : 0,
					mic_ok ? mic.peak : 0,
					mic_ok ? mic.level_pct : 0,
					batt_mv, batt_pct);

				sdcard_write_line(sd_line);

				/* Flush every 10 seconds (5th write) */
				if ((cycle % 20) == 0) {
					sdcard_flush();
				}
			}
		}

		gpio_pin_set_dt(&led_green, 0);

		k_msleep(500);
	}

	return 0;
}
