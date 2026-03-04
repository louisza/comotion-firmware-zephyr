/*
 * ═══════════════════════════════════════════════════════════════
 * CoMotion Tracker — GPS (UART + NMEA Parser)
 *
 * ATGM332D on UART1, 9600 baud
 * NMEA parser copied verbatim from Arduino firmware.
 * ═══════════════════════════════════════════════════════════════
 */

#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "common.h"
#include "gps.h"

LOG_MODULE_REGISTER(gps, LOG_LEVEL_INF);

static const struct device *gps_uart;
static char nmea_buf[100];
static int  nmea_idx;

/* ─── Forward declarations ─── */
static void parse_nmea_char(char c);
static void parse_nmea_sentence(void);
static bool validate_checksum(void);
static void parse_rmc(void);
static void parse_gga(void);

/* ═══════════════════════════════════════════════════════════════
 * UART IRQ Callback
 * ═══════════════════════════════════════════════════════════════ */

static void gps_uart_cb(const struct device *dev, void *user_data)
{
	uint8_t c;

	if (!uart_irq_update(dev)) return;

	while (uart_irq_rx_ready(dev)) {
		if (uart_fifo_read(dev, &c, 1) == 1) {
			parse_nmea_char((char)c);
		}
	}
}

/* ═══════════════════════════════════════════════════════════════
 * Init
 * ═══════════════════════════════════════════════════════════════ */

int gps_init(void)
{
	gps_uart = DEVICE_DT_GET(DT_NODELABEL(uart1));
	if (!device_is_ready(gps_uart)) {
		LOG_ERR("UART1 not ready");
		return -ENODEV;
	}

	uart_irq_callback_user_data_set(gps_uart, gps_uart_cb, NULL);
	uart_irq_rx_enable(gps_uart);

	LOG_INF("[GPS] UART1 ready (9600 baud)");
	return 0;
}

void gps_poll(void)
{
	/* GPS is IRQ-driven; nothing to poll */
}

/* ═══════════════════════════════════════════════════════════════
 * NMEA Parser (verbatim from Arduino firmware)
 * ═══════════════════════════════════════════════════════════════ */

static void parse_nmea_char(char c)
{
	if (c == '$') {
		nmea_idx = 0;
	}
	if (nmea_idx < 99) {
		nmea_buf[nmea_idx++] = c;
		nmea_buf[nmea_idx] = '\0';
	}
	if (c == '\n') {
		parse_nmea_sentence();
		nmea_idx = 0;
	}
}

static bool validate_checksum(void)
{
	char *star = strchr(nmea_buf, '*');
	if (!star || strlen(star) < 3) return false;

	uint8_t expected = (uint8_t)strtol(star + 1, NULL, 16);
	uint8_t calculated = 0;
	for (char *p = nmea_buf + 1; p < star; p++) {
		calculated ^= (uint8_t)*p;
	}
	return (calculated == expected);
}

static void parse_nmea_sentence(void)
{
	if (!validate_checksum()) return;

	if (strncmp(nmea_buf, "$GPRMC", 6) == 0 ||
	    strncmp(nmea_buf, "$GNRMC", 6) == 0) {
		parse_rmc();
	} else if (strncmp(nmea_buf, "$GPGGA", 6) == 0 ||
		   strncmp(nmea_buf, "$GNGGA", 6) == 0) {
		parse_gga();
	}
}

static void parse_rmc(void)
{
	char *p = nmea_buf;
	int field = 0;
	char *tokens[15];

	tokens[0] = p;
	while (*p && field < 14) {
		if (*p == ',') {
			*p = '\0';
			tokens[++field] = p + 1;
		}
		p++;
	}
	if (field < 8) return;

	if (tokens[2][0] != 'A') {
		g_gps.valid = false;
		return;
	}

	/* Latitude (DDMM.MMMM) */
	if (strlen(tokens[3]) > 0) {
		float raw = atof(tokens[3]);
		int deg = (int)(raw / 100);
		float min = raw - deg * 100;
		g_gps.latitude = deg + min / 60.0f;
		if (tokens[4][0] == 'S') g_gps.latitude = -g_gps.latitude;
	}

	/* Longitude (DDDMM.MMMM) */
	if (strlen(tokens[5]) > 0) {
		float raw = atof(tokens[5]);
		int deg = (int)(raw / 100);
		float min = raw - deg * 100;
		g_gps.longitude = deg + min / 60.0f;
		if (tokens[6][0] == 'W') g_gps.longitude = -g_gps.longitude;
	}

	/* Speed (knots → km/h) */
	if (strlen(tokens[7]) > 0) {
		g_gps.speed = atof(tokens[7]) * 1.852f;
	}

	/* Course */
	if (strlen(tokens[8]) > 0) {
		g_gps.course = atof(tokens[8]);
	}

	g_gps.valid = true;
	g_gps.last_update = k_uptime_get();
}

static void parse_gga(void)
{
	char *p = nmea_buf;
	int field = 0;
	char *tokens[15];

	tokens[0] = p;
	while (*p && field < 14) {
		if (*p == ',') {
			*p = '\0';
			tokens[++field] = p + 1;
		}
		p++;
	}
	if (field >= 7) {
		g_gps.satellites = atoi(tokens[7]);
	}
}
