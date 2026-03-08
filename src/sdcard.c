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

/* ─── Transfer state ─── */
static volatile bool transfer_abort;
static char last_log_filename[32];

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
	strncpy(last_log_filename, log_filename, sizeof(last_log_filename));
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

/* ═══════════════════════════════════════════════════════════════
 * Log Transfer — NUS commands for BLE download pipeline
 * ═══════════════════════════════════════════════════════════════ */

/* Forward: send string over NUS */
extern void ble_send(const char *msg);
/* Forward: send raw binary over NUS */
extern void ble_send_raw(const uint8_t *data, uint16_t len);

/**
 * Scan first/last bytes of a CSV file to find GPS timestamps.
 * Returns first valid timestamp in *start, last in *end (Unix epoch).
 * Returns 0 if not found.
 *
 * Looks for the "timestamp" column (column 0) — a numeric Unix epoch ms value.
 * Converts ms → seconds for the epoch.
 */
static void scan_file_timestamps(const char *path, int64_t *start, int64_t *end)
{
	struct fs_file_t f;
	fs_file_t_init(&f);
	*start = 0;
	*end = 0;

	if (fs_open(&f, path, FS_O_READ) != 0) return;

	struct fs_dirent entry;
	if (fs_stat(path, &entry) != 0) {
		fs_close(&f);
		return;
	}

	/* Read first 512 bytes to find first timestamp */
	char buf[512];
	int n = fs_read(&f, buf, sizeof(buf));
	if (n > 0) {
		/* Skip header line */
		char *p = memchr(buf, '\n', n);
		if (p && p < buf + n - 1) {
			p++; /* Start of second line */
			/* Parse first field (timestamp) */
			char *comma = memchr(p, ',', buf + n - p);
			if (comma) {
				char tmp[20];
				int len = MIN(comma - p, (int)sizeof(tmp) - 1);
				memcpy(tmp, p, len);
				tmp[len] = '\0';
				int64_t ts = strtoll(tmp, NULL, 10);
				if (ts > 1000000000000LL) ts /= 1000; /* ms → s */
				if (ts > 1600000000) *start = ts;
			}
		}
	}

	/* Read last 1024 bytes to find last timestamp */
	off_t fsize = entry.size;
	off_t seek_pos = (fsize > 1024) ? fsize - 1024 : 0;
	fs_seek(&f, seek_pos, FS_SEEK_SET);

	char buf2[1024];
	n = fs_read(&f, buf2, sizeof(buf2));
	if (n > 0) {
		/* Find last complete line */
		char *last_nl = NULL;
		char *prev_nl = NULL;
		for (int i = n - 2; i >= 0; i--) {
			if (buf2[i] == '\n') {
				if (!last_nl) {
					last_nl = &buf2[i];
				} else {
					prev_nl = &buf2[i];
					break;
				}
			}
		}
		char *line_start = prev_nl ? prev_nl + 1 : buf2;
		if (last_nl && line_start < last_nl) {
			char *comma = memchr(line_start, ',', last_nl - line_start);
			if (comma) {
				char tmp[20];
				int len = MIN(comma - line_start, (int)sizeof(tmp) - 1);
				memcpy(tmp, line_start, len);
				tmp[len] = '\0';
				int64_t ts = strtoll(tmp, NULL, 10);
				if (ts > 1000000000000LL) ts /= 1000;
				if (ts > 1600000000) *end = ts;
			}
		}
	}

	fs_close(&f);
}

void sdcard_handle_list(void)
{
	if (!sd_mounted) {
		ble_send("ERR:SD not mounted");
		return;
	}

	struct fs_dir_t dir;
	fs_dir_t_init(&dir);

	if (fs_opendir(&dir, "/SD:") != 0) {
		ble_send("ERR:Cannot open SD dir");
		return;
	}

	struct fs_dirent entry;
	while (fs_readdir(&dir, &entry) == 0 && entry.name[0] != '\0') {
		/* Only list .CSV files */
		int nlen = strlen(entry.name);
		if (nlen < 5) continue;
		if (strcasecmp(&entry.name[nlen - 4], ".CSV") != 0) continue;

		/* Scan for timestamps */
		char fullpath[48];
		snprintf(fullpath, sizeof(fullpath), "/SD:/%s", entry.name);

		int64_t ts_start = 0, ts_end = 0;
		scan_file_timestamps(fullpath, &ts_start, &ts_end);

		char msg[128];
		snprintf(msg, sizeof(msg), "FILE:%s,%u,%lld,%lld",
			 entry.name, (unsigned)entry.size, ts_start, ts_end);
		ble_send(msg);
	}

	fs_closedir(&dir);
	ble_send("END_LIST");
}

void sdcard_handle_dump(const char *filename)
{
	if (!sd_mounted) {
		ble_send("ERR:SD not mounted");
		return;
	}

	char fullpath[48];
	snprintf(fullpath, sizeof(fullpath), "/SD:/%s", filename);

	struct fs_dirent entry;
	if (fs_stat(fullpath, &entry) != 0) {
		ble_send("ERR:File not found");
		return;
	}

	struct fs_file_t f;
	fs_file_t_init(&f);
	if (fs_open(&f, fullpath, FS_O_READ) != 0) {
		ble_send("ERR:Cannot open file");
		return;
	}

	/* Send transfer header */
	char header[64];
	snprintf(header, sizeof(header), "XFER:%s,%u,0", filename, (unsigned)entry.size);
	ble_send(header);
	k_msleep(50); /* Let app process the header */

	/* Send binary chunks: [seq_u16_le][payload_240] */
	transfer_abort = false;
	uint16_t seq = 0;
	uint8_t chunk[242]; /* 2 byte seq + 240 byte payload */
	uint32_t total_sent = 0;

	while (!transfer_abort) {
		int n = fs_read(&f, &chunk[2], 240);
		if (n <= 0) break;

		chunk[0] = seq & 0xFF;
		chunk[1] = (seq >> 8) & 0xFF;

		ble_send_raw(chunk, 2 + n);
		total_sent += n;
		seq++;

		/* Pace: ~5 KB/s, yield between chunks */
		k_msleep(2);
	}

	fs_close(&f);

	if (transfer_abort) {
		ble_send("ERR:Transfer aborted");
	} else {
		char footer[64];
		snprintf(footer, sizeof(footer), "END_DUMP:%u,0", total_sent);
		ble_send(footer);
	}
}

void sdcard_handle_dump_latest(void)
{
	if (last_log_filename[0] != '\0') {
		/* Strip /SD:/ prefix for the dump handler */
		const char *name = last_log_filename;
		if (strncmp(name, "/SD:/", 5) == 0) name += 5;
		sdcard_handle_dump(name);
	} else {
		ble_send("ERR:No logs recorded yet");
	}
}

void sdcard_handle_delete(const char *filename)
{
	if (!sd_mounted) {
		ble_send("ERR:SD not mounted");
		return;
	}

	/* Don't delete the file currently being written */
	char fullpath[48];
	snprintf(fullpath, sizeof(fullpath), "/SD:/%s", filename);
	if (g_is_logging && strcmp(fullpath, log_filename) == 0) {
		ble_send("ERR:Cannot delete active log");
		return;
	}

	if (fs_unlink(fullpath) == 0) {
		ble_send("OK");
	} else {
		ble_send("ERR:Delete failed");
	}
}

void sdcard_handle_status_cmd(void)
{
	if (!sd_mounted) {
		ble_send("ERR:SD not mounted");
		return;
	}

	struct fs_statvfs stat;
	if (fs_statvfs("/SD:", &stat) != 0) {
		ble_send("ERR:Cannot read SD stats");
		return;
	}

	uint32_t free_kb = (stat.f_bfree * stat.f_frsize) / 1024;
	uint32_t total_kb = (stat.f_blocks * stat.f_frsize) / 1024;

	/* Count CSV files */
	struct fs_dir_t dir;
	fs_dir_t_init(&dir);
	int file_count = 0;
	if (fs_opendir(&dir, "/SD:") == 0) {
		struct fs_dirent entry;
		while (fs_readdir(&dir, &entry) == 0 && entry.name[0] != '\0') {
			int nlen = strlen(entry.name);
			if (nlen >= 5 && strcasecmp(&entry.name[nlen - 4], ".CSV") == 0) {
				file_count++;
			}
		}
		fs_closedir(&dir);
	}

	char msg[64];
	snprintf(msg, sizeof(msg), "SD:%u,%u,%d", free_kb, total_kb, file_count);
	ble_send(msg);
}

void sdcard_handle_abort(void)
{
	transfer_abort = true;
}
