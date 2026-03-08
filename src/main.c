/*
 * ═══════════════════════════════════════════════════════════════
 * CoMotion Tracker — Main Entry Point
 * Board: XIAO nRF52840 Sense
 * RTOS: Zephyr / nRF Connect SDK
 * ═══════════════════════════════════════════════════════════════
 *
 * Threads:
 *   - IMU timer     : 104 Hz sensor read + intensity update
 *   - Audio timer   : 10 Hz PDM processing
 *   - Main thread   : BLE adv update, SD flush, LED, battery, commands
 *   - GPS           : UART IRQ-driven (no thread needed)
 * ═══════════════════════════════════════════════════════════════
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/usb/usb_device.h>

#include "common.h"
#include "ble.h"
#include "sensors.h"
#include "gps.h"
#include "sdcard.h"
#include "intensity.h"

LOG_MODULE_REGISTER(comotion, LOG_LEVEL_INF);

/* ═══════════════════════════════════════════════════════════════
 * Global State
 * ═══════════════════════════════════════════════════════════════ */

struct imu_reading g_imu = {0};
struct gps_data    g_gps = {0};
volatile bool      g_is_logging = false;
float              g_battery_voltage = 0;
volatile float     g_audio_rms = 0;
volatile int       g_audio_peak = 0;
volatile int       g_audio_zcr = 0;
volatile float     g_audio_baseline = 500.0f;

float    g_max_speed_session = 0;
uint8_t  g_impact_count = 0;
uint16_t g_move_count = 0;
int64_t  g_log_start_time = 0;
uint32_t g_sample_count = 0;

/* ─── LEDs ─── */
static const struct gpio_dt_spec led_red   = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec led_blue  = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);

/* ─── IMU Timer (104 Hz) ─── */
static void imu_timer_handler(struct k_timer *timer);
K_TIMER_DEFINE(imu_timer, imu_timer_handler, NULL);

static void imu_timer_handler(struct k_timer *timer)
{
	sensors_read_imu();
	intensity_update();

	if (g_is_logging) {
		sdcard_log_sample(k_uptime_get());
	}
}

/* ─── Audio Timer (10 Hz) ─── */
static void audio_timer_handler(struct k_timer *timer);
K_TIMER_DEFINE(audio_timer, audio_timer_handler, NULL);

static void audio_timer_handler(struct k_timer *timer)
{
	sensors_process_audio();
}

/* ─── LED control ─── */
static void leds_init(void)
{
	gpio_pin_configure_dt(&led_red,   GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led_green, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led_blue,  GPIO_OUTPUT_INACTIVE);
}

static void leds_all_off(void)
{
	gpio_pin_set_dt(&led_red,   0);
	gpio_pin_set_dt(&led_green, 0);
	gpio_pin_set_dt(&led_blue,  0);
}

static void led_blink(const struct gpio_dt_spec *led, int count, int delay_ms)
{
	for (int i = 0; i < count; i++) {
		gpio_pin_set_dt(led, 1);
		k_msleep(delay_ms);
		gpio_pin_set_dt(led, 0);
		k_msleep(delay_ms);
	}
}

static void update_led_status(void)
{
	static bool toggle = false;
	toggle = !toggle;

	leds_all_off();

	if (g_is_logging) {
		if (g_gps.valid && g_gps.satellites >= 4) {
			gpio_pin_set_dt(&led_green, 1); /* Solid green */
		} else {
			gpio_pin_set_dt(&led_green, toggle ? 1 : 0); /* Blink */
		}
	} else {
		gpio_pin_set_dt(&led_blue, toggle ? 1 : 0); /* Slow blink idle */
	}

	/* Red override for low battery */
	if (g_battery_voltage > 0 && g_battery_voltage < 3.3f) {
		gpio_pin_set_dt(&led_red, 1);
	}
}

/* ═══════════════════════════════════════════════════════════════
 * Command Processing (shared by NUS + serial)
 * ═══════════════════════════════════════════════════════════════ */

void process_command(const char *cmd)
{
	LOG_INF("[CMD] %s", cmd);

	if (strcmp(cmd, "start") == 0) {
		if (sdcard_start_logging()) {
			ble_send("OK:Logging started");
		} else {
			ble_send("ERR:Failed to start logging");
		}
	} else if (strcmp(cmd, "stop") == 0) {
		sdcard_stop_logging();
		ble_send("OK:Logging stopped");
	} else if (strcmp(cmd, "status") == 0) {
		char msg[128];
		snprintf(msg, sizeof(msg), "STATUS:%s",
			 g_is_logging ? "LOGGING" : "IDLE");
		ble_send(msg);
		snprintf(msg, sizeof(msg), "GPS:%s SATS:%d",
			 g_gps.valid ? "OK" : "NO_FIX", g_gps.satellites);
		ble_send(msg);
		snprintf(msg, sizeof(msg), "BAT:%.2fV", (double)g_battery_voltage);
		ble_send(msg);
	} else if (strncmp(cmd, "event:", 6) == 0) {
		sdcard_mark_event(cmd + 6);
		ble_send("OK:Event marked");
	} else if (strcmp(cmd, "bat") == 0) {
		char msg[32];
		snprintf(msg, sizeof(msg), "BAT:%.2fV", (double)g_battery_voltage);
		ble_send(msg);
	} else if (strcmp(cmd, "info") == 0) {
		char msg[64];
		snprintf(msg, sizeof(msg), "CoMotion v%s (Zephyr)", FIRMWARE_VERSION);
		ble_send(msg);
	} else if (strcmp(cmd, "ping") == 0) {
		ble_send("pong");
	} else if (strcmp(cmd, "LIST") == 0) {
		sdcard_handle_list();
	} else if (strncmp(cmd, "DUMP:", 5) == 0) {
		sdcard_handle_dump(cmd + 5);
	} else if (strcmp(cmd, "DUMP_LATEST") == 0) {
		sdcard_handle_dump_latest();
	} else if (strncmp(cmd, "DELETE:", 7) == 0) {
		sdcard_handle_delete(cmd + 7);
	} else if (strcmp(cmd, "STATUS") == 0) {
		sdcard_handle_status_cmd();
	} else if (strcmp(cmd, "ABORT") == 0) {
		sdcard_handle_abort();
	} else if (strncmp(cmd, "ACK:", 4) == 0) {
		/* ACK from app during transfer — just ignore (pacing handled in dump) */
	} else {
		ble_send("ERR:Unknown command");
	}
}

/* ═══════════════════════════════════════════════════════════════
 * Main
 * ═══════════════════════════════════════════════════════════════ */

int main(void)
{
	int err;

	/* USB console */
	err = usb_enable(NULL);
	if (err) {
		LOG_WRN("USB init failed: %d", err);
	}
	/* Give USB time to enumerate */
	k_msleep(1000);

	LOG_INF("═══════════════════════════════════════════════");
	LOG_INF("  CoMotion Tracker v%s (Zephyr/NCS)", FIRMWARE_VERSION);
	LOG_INF("═══════════════════════════════════════════════");

	/* LEDs */
	leds_init();
	gpio_pin_set_dt(&led_blue, 1); /* Blue = booting */

	bool all_good = true;

	/* GPS */
	LOG_INF("[GPS] Initializing...");
	if (gps_init() == 0) {
		LOG_INF("[GPS] OK");
	} else {
		LOG_ERR("[GPS] FAILED");
		all_good = false;
	}

	/* Sensors (IMU + PDM + Battery) */
	LOG_INF("[SENSORS] Initializing...");
	if (sensors_init() == 0) {
		LOG_INF("[SENSORS] OK");
		LOG_INF("[CAL] Gyro calibration — keep device still...");
		if (sensors_calibrate_gyro()) {
			LOG_INF("[CAL] Success!");
		} else {
			LOG_WRN("[CAL] Skipped — device not still");
		}
	} else {
		LOG_ERR("[SENSORS] FAILED");
		all_good = false;
	}

	/* SD Card */
	LOG_INF("[SD] Initializing...");
	if (sdcard_init() == 0) {
		LOG_INF("[SD] OK");
	} else {
		LOG_ERR("[SD] FAILED");
		all_good = false;
	}

	/* Battery */
	g_battery_voltage = sensors_read_battery();
	LOG_INF("[BAT] %.2fV", (double)g_battery_voltage);

	/* BLE */
	LOG_INF("[BLE] Initializing...");
	if (ble_init() == 0) {
		LOG_INF("[BLE] OK — Dual advertising (1M + Coded PHY)");
	} else {
		LOG_ERR("[BLE] FAILED");
		all_good = false;
	}

	leds_all_off();

	if (all_good) {
		LOG_INF("✓ All systems ready!");
		LOG_INF("  Send 'start' via BLE to begin logging");
		led_blink(&led_green, 3, 100);
	} else {
		LOG_ERR("✗ Some components failed");
		led_blink(&led_red, 5, 100);
	}

	/* Start periodic timers */
	k_timer_start(&imu_timer, K_USEC(1000000 / IMU_SAMPLE_RATE_HZ),
		       K_USEC(1000000 / IMU_SAMPLE_RATE_HZ));
	k_timer_start(&audio_timer, K_MSEC(1000 / AUDIO_RMS_RATE_HZ),
		       K_MSEC(1000 / AUDIO_RMS_RATE_HZ));

	/* ─── Main loop ─── */
	int64_t last_adv_update = 0;
	int64_t last_led_update = 0;
	int64_t last_bat_read = 0;
	int64_t last_intensity_sample = 0;
	int64_t last_sd_flush = 0;

	while (1) {
		int64_t now = k_uptime_get();

		/* BLE advertising data update (500ms) */
		if (now - last_adv_update >= BLE_ADV_UPDATE_MS) {
			last_adv_update = now;
			ble_update_advertising();
		}

		/* SD buffer flush */
		if (g_is_logging && (now - last_sd_flush >= SD_FLUSH_INTERVAL_MS)) {
			last_sd_flush = now;
			sdcard_flush();
		}

		/* LED status (500ms) */
		if (now - last_led_update >= 500) {
			last_led_update = now;
			update_led_status();
		}

		/* Battery (30s) */
		if (now - last_bat_read >= BATTERY_READ_INTERVAL_S * 1000) {
			last_bat_read = now;
			g_battery_voltage = sensors_read_battery();
		}

		/* Intensity history (2 Hz) */
		if (now - last_intensity_sample >= (1000 / INTENSITY_SAMPLE_HZ)) {
			last_intensity_sample = now;
			intensity_sample_history();
		}

		k_msleep(10); /* ~100 Hz main loop */
	}

	return 0;
}
