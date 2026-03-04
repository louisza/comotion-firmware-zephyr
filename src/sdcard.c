/*
 * ═══════════════════════════════════════════════════════════════
 * CoMotion Tracker — SD Card Logging (FAT FS + Ring Buffer)
 * ═══════════════════════════════════════════════════════════════
 */

#include <zephyr/fs/fs.h>
#include <zephyr/fs/fs_sys.h>
#include <ff.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <string.h>

#include "common.h"
#include "sdcard.h"

LOG_MODULE_REGISTER(sdcard, LOG_LEVEL_INF);

/* ─── FAT FS Mount ─── */
static FATFS fat_fs;
static struct fs_mount_t mp = {
	.type = FS_FATFS,
	.fs_data = &fat_fs,
	.mnt_point = "/SD:",
};
static bool sd_mounted;

/* ─── Log File ─── */
static struct fs_file_t log_file;
static bool file_open;
static char log_filename[32];

/* ─── Ring Buffer ─── */
static char sd_buffer[SD_BUFFER_SIZE];
static volatile uint16_t buf_head;
static volatile uint16_t buf_tail;
static volatile uint16_t buf_count;
static char write_buffer[SD_BUFFER_SIZE];

/* ─── Event ─── */
static char current_event[32];

/* ═══════════════════════════════════════════════════════════════
 * Ring Buffer (verbatim from Arduino firmware)
 * ═══════════════════════════════════════════════════════════════ */

static void add_to_buffer(const char *data, int len)
{
	for (int i = 0; i < len && buf_count < SD_BUFFER_SIZE; i++) {
		sd_buffer[buf_head] = data[i];
		buf_head = (buf_head + 1) % SD_BUFFER_SIZE;
		buf_count++;
	}
}

/* ═══════════════════════════════════════════════════════════════
 * Init
 * ═══════════════════════════════════════════════════════════════ */

int sdcard_init(void)
{
	fs_file_t_init(&log_file);

	int err = fs_mount(&mp);
	if (err) {
		LOG_ERR("[SD] Mount failed: %d", err);
		return err;
	}

	sd_mounted = true;
	LOG_INF("[SD] Mounted");
	return 0;
}

/* ═══════════════════════════════════════════════════════════════
 * Logging Control
 * ═══════════════════════════════════════════════════════════════ */

bool sdcard_start_logging(void)
{
	if (g_is_logging) return true;
	if (!sd_mounted) return false;

	/* Find next available filename */
	int num = 1;
	while (num < 1000) {
		snprintf(log_filename, sizeof(log_filename),
			 "/SD:/LOG%03d.CSV", num);
		struct fs_dirent entry;
		if (fs_stat(log_filename, &entry) != 0) break;
		num++;
	}
	if (num >= 1000) {
		LOG_ERR("[SD] No available filename");
		return false;
	}

	int err = fs_open(&log_file, log_filename,
			  FS_O_CREATE | FS_O_WRITE | FS_O_APPEND);
	if (err) {
		LOG_ERR("[SD] Failed to create file: %d", err);
		return false;
	}
	file_open = true;

	/* Write CSV header */
	const char *header = "timestamp,ax,ay,az,gx,gy,gz,"
			     "lat,lng,speed,course,sats,"
			     "audio_rms,audio_peak,audio_zcr,event\n";
	fs_write(&log_file, header, strlen(header));
	fs_sync(&log_file);

	buf_head = 0;
	buf_tail = 0;
	buf_count = 0;
	g_sample_count = 0;
	g_log_start_time = k_uptime_get();
	current_event[0] = '\0';

	/* Reset session stats */
	g_max_speed_session = 0;
	g_impact_count = 0;
	g_move_count = 0;

	g_is_logging = true;
	LOG_INF("[LOG] Started: %s", log_filename);
	return true;
}

void sdcard_stop_logging(void)
{
	if (!g_is_logging) return;

	sdcard_flush();

	if (file_open) {
		fs_close(&log_file);
		file_open = false;
	}

	int64_t duration = (k_uptime_get() - g_log_start_time) / 1000;
	float rate = (duration > 0) ? (float)g_sample_count / duration : 0;

	LOG_INF("[LOG] Stopped — %u samples, %llds, %.1f Hz",
		g_sample_count, duration, (double)rate);
	g_is_logging = false;
}

/* ═══════════════════════════════════════════════════════════════
 * Log a Sample (called from 104 Hz timer)
 * ═══════════════════════════════════════════════════════════════ */

void sdcard_log_sample(int64_t timestamp)
{
	char escaped_event[36];
	if (strchr(current_event, ',') != NULL) {
		snprintf(escaped_event, sizeof(escaped_event),
			 "\"%s\"", current_event);
	} else {
		strncpy(escaped_event, current_event, sizeof(escaped_event) - 1);
		escaped_event[sizeof(escaped_event) - 1] = '\0';
	}

	char line[220];
	int len;
	int64_t now = k_uptime_get();
	bool gps_valid = g_gps.valid &&
			 (now - g_gps.last_update < GPS_TIMEOUT_MS);

	if (gps_valid) {
		len = snprintf(line, sizeof(line),
			"%lld,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,"
			"%.6f,%.6f,%.1f,%.1f,%d,"
			"%.0f,%d,%d,%s\n",
			timestamp,
			(double)g_imu.ax, (double)g_imu.ay, (double)g_imu.az,
			(double)g_imu.gx, (double)g_imu.gy, (double)g_imu.gz,
			(double)g_gps.latitude, (double)g_gps.longitude,
			(double)g_gps.speed, (double)g_gps.course,
			g_gps.satellites,
			(double)g_audio_rms, g_audio_peak, g_audio_zcr,
			escaped_event);
	} else {
		len = snprintf(line, sizeof(line),
			"%lld,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,"
			",,,,,"
			"%.0f,%d,%d,%s\n",
			timestamp,
			(double)g_imu.ax, (double)g_imu.ay, (double)g_imu.az,
			(double)g_imu.gx, (double)g_imu.gy, (double)g_imu.gz,
			(double)g_audio_rms, g_audio_peak, g_audio_zcr,
			escaped_event);
	}

	add_to_buffer(line, len);
	g_sample_count++;

	if (current_event[0] != '\0') {
		current_event[0] = '\0';
	}
}

/* ═══════════════════════════════════════════════════════════════
 * Flush Buffer to SD Card
 * ═══════════════════════════════════════════════════════════════ */

void sdcard_flush(void)
{
	if (buf_count == 0 || !file_open) return;

	int write_len = 0;
	unsigned int key = irq_lock();
	while (buf_count > 0 && write_len < SD_BUFFER_SIZE - 1) {
		write_buffer[write_len++] = sd_buffer[buf_tail];
		buf_tail = (buf_tail + 1) % SD_BUFFER_SIZE;
		buf_count--;
	}
	irq_unlock(key);

	if (write_len > 0) {
		fs_write(&log_file, write_buffer, write_len);
		fs_sync(&log_file);
	}
}

void sdcard_mark_event(const char *name)
{
	strncpy(current_event, name, sizeof(current_event) - 1);
	current_event[sizeof(current_event) - 1] = '\0';
	LOG_INF("[EVENT] %s", current_event);
}
