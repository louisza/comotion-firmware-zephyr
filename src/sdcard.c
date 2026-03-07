/*
 * CoMotion Tracker — SD Card Logging Module
 *
 * Architecture:
 *   - 4096-byte ring buffer (main loop writes CSV lines at 104 Hz)
 *   - Dedicated writer thread flushes to SD when ≥50% full or every 5s
 *   - Spinlock protects ring buffer for SPSC thread safety
 *   - Session files: LOG001.CSV → LOG999.CSV (auto-increment)
 *
 * CSV format (spec section 8):
 *   timestamp,ax,ay,az,gx,gy,gz,lat,lng,speed,course,sats,
 *   audio_rms,audio_peak,audio_zcr,event
 */

#include <zephyr/kernel.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/fs/fs.h>
#include <zephyr/sys/ring_buffer.h>
#include <ff.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <string.h>

#include "sdcard.h"

LOG_MODULE_REGISTER(sdcard, CONFIG_LOG_DEFAULT_LEVEL);

/* ─── FAT FS Mount ─── */
#define DISK_NAME    "SD"
#define MOUNT_POINT  "/" DISK_NAME ":"

static FATFS fat_fs;
static struct fs_mount_t mount_pt = {
	.type      = FS_FATFS,
	.fs_data   = &fat_fs,
	.mnt_point = MOUNT_POINT,
};

static bool mounted;

/* ─── Ring Buffer (spec: 4096 bytes) ─── */
#define SD_BUFFER_SIZE 4096

RING_BUF_DECLARE(sd_ring, SD_BUFFER_SIZE);
static struct k_spinlock ring_lock;

/* ─── Writer Thread ─── */
#define SD_WRITER_STACK  4096
#define SD_WRITER_PRIO   10         /* lower priority than main + audio */
#define SD_FLUSH_INTERVAL_MS 5000

K_THREAD_STACK_DEFINE(sd_writer_stack, SD_WRITER_STACK);
static struct k_thread sd_writer_thread;
static K_SEM_DEFINE(sd_flush_sem, 0, 1);
static bool writer_running;

/* ─── Log File ─── */
#define CSV_HEADER "timestamp,ax,ay,az,gx,gy,gz,lat,lng,speed,course," \
	"sats,audio_rms,audio_peak,audio_zcr,event\n"

static struct fs_file_t log_file;
static bool file_open;
static bool logging;
static char log_path[32];
static char log_filename[16];   /* e.g., "LOG042.CSV" */
static uint32_t sample_count;

/* ─── Event Tagging ─── */
static char event_tag[32];
static bool event_pending;

/* ─── Flush buffer (outside ring for SD write) ─── */
static uint8_t flush_buf[SD_BUFFER_SIZE];

/* ─── Helpers ─── */

static int find_next_session(void)
{
	struct fs_dir_t dir;
	struct fs_dirent entry;
	int max_num = 0;

	fs_dir_t_init(&dir);
	if (fs_opendir(&dir, MOUNT_POINT) != 0) {
		return 1;
	}

	while (fs_readdir(&dir, &entry) == 0 && entry.name[0] != '\0') {
		int num;

		if (sscanf(entry.name, "LOG%d.CSV", &num) == 1 ||
		    sscanf(entry.name, "log%d.csv", &num) == 1) {
			if (num > max_num) {
				max_num = num;
			}
		}
	}

	fs_closedir(&dir);
	return max_num + 1;
}

static void do_flush(void)
{
	if (!file_open) {
		return;
	}

	uint32_t total = 0;

	/* Copy data out of ring buffer under spinlock */
	k_spinlock_key_t key = k_spin_lock(&ring_lock);
	uint32_t len = ring_buf_get(&sd_ring, flush_buf, sizeof(flush_buf));

	k_spin_unlock(&ring_lock, key);

	if (len == 0) {
		return;
	}

	/* Write to SD (outside lock — this is the slow part) */
	int ret = fs_write(&log_file, flush_buf, len);

	if (ret < 0) {
		LOG_ERR("SD write failed: %d", ret);
	} else {
		total += len;
	}

	fs_sync(&log_file);

	if (total > 0) {
		LOG_DBG("SD flush: %u bytes", total);
	}
}

/* ─── Writer Thread ─── */

static void sd_writer_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	LOG_INF("SD writer thread started");

	while (writer_running) {
		/* Wait for signal or periodic timeout */
		k_sem_take(&sd_flush_sem, K_MSEC(SD_FLUSH_INTERVAL_MS));

		if (logging && file_open) {
			do_flush();
		}
	}

	/* Final flush on shutdown */
	if (file_open) {
		do_flush();
	}
}

/* ─── Public API ─── */

int sdcard_init(void)
{
	uint32_t block_count, block_size;
	uint64_t size_mb;
	int ret;

	fs_file_t_init(&log_file);
	ring_buf_reset(&sd_ring);

	/* Initialize disk */
	ret = disk_access_init(DISK_NAME);
	if (ret) {
		LOG_WRN("SD: disk_access_init failed: %d (no card?)", ret);
		return ret;
	}

	ret = disk_access_ioctl(DISK_NAME, DISK_IOCTL_GET_SECTOR_COUNT,
				&block_count);
	if (ret) {
		LOG_ERR("SD: cannot get sector count: %d", ret);
		return ret;
	}

	ret = disk_access_ioctl(DISK_NAME, DISK_IOCTL_GET_SECTOR_SIZE,
				&block_size);
	if (ret) {
		LOG_ERR("SD: cannot get sector size: %d", ret);
		return ret;
	}

	size_mb = (uint64_t)block_count * block_size / (1024 * 1024);
	LOG_INF("SD: card detected — %u MB", (uint32_t)size_mb);

	ret = fs_mount(&mount_pt);
	if (ret) {
		LOG_ERR("SD: mount failed: %d", ret);
		return ret;
	}

	mounted = true;
	LOG_INF("SD: FAT filesystem mounted at %s", MOUNT_POINT);

	/* Start writer thread */
	writer_running = true;
	k_thread_create(&sd_writer_thread, sd_writer_stack,
			K_THREAD_STACK_SIZEOF(sd_writer_stack),
			sd_writer_fn, NULL, NULL, NULL,
			SD_WRITER_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(&sd_writer_thread, "sd_writer");

	return 0;
}

bool sdcard_is_mounted(void)
{
	return mounted;
}

int sdcard_start_logging(void)
{
	int ret;

	if (!mounted) {
		return -ENODEV;
	}

	if (logging) {
		LOG_WRN("SD: already logging");
		return -EALREADY;
	}

	/* Find next session file */
	int session = find_next_session();

	if (session > 999) {
		LOG_ERR("SD: max session count reached");
		return -ENOSPC;
	}

	snprintf(log_filename, sizeof(log_filename), "LOG%03d.CSV", session);
	snprintf(log_path, sizeof(log_path), MOUNT_POINT "/%s", log_filename);

	/* Open file */
	ret = fs_open(&log_file, log_path, FS_O_CREATE | FS_O_WRITE);
	if (ret) {
		LOG_ERR("SD: cannot open %s: %d", log_path, ret);
		return ret;
	}

	file_open = true;

	/* Write CSV header */
	ret = fs_write(&log_file, CSV_HEADER, strlen(CSV_HEADER));
	if (ret < 0) {
		LOG_ERR("SD: header write failed: %d", ret);
		fs_close(&log_file);
		file_open = false;
		return ret;
	}

	fs_sync(&log_file);

	/* Reset ring buffer and counters */
	k_spinlock_key_t key = k_spin_lock(&ring_lock);

	ring_buf_reset(&sd_ring);
	k_spin_unlock(&ring_lock, key);

	sample_count = 0;
	event_pending = false;
	event_tag[0] = '\0';
	logging = true;

	LOG_INF("SD: logging started → %s", log_path);
	return 0;
}

int sdcard_stop_logging(void)
{
	if (!logging) {
		return 0;
	}

	logging = false;

	/* Flush remaining data */
	do_flush();

	if (file_open) {
		fs_close(&log_file);
		file_open = false;
	}

	LOG_INF("SD: logging stopped (%s, %u samples)", log_filename,
		sample_count);
	return 0;
}

bool sdcard_is_logging(void)
{
	return logging;
}

uint32_t sdcard_write(const char *data, uint32_t len)
{
	if (!logging) {
		return 0;
	}

	k_spinlock_key_t key = k_spin_lock(&ring_lock);
	uint32_t written = ring_buf_put(&sd_ring, (const uint8_t *)data, len);
	uint32_t used = ring_buf_size_get(&sd_ring);

	k_spin_unlock(&ring_lock, key);

	if (written < len) {
		LOG_WRN("SD ring overflow: dropped %u bytes", len - written);
	} else {
		sample_count++;
	}

	/* Signal writer thread if ≥50% full */
	if (used >= SD_BUFFER_SIZE / 2) {
		k_sem_give(&sd_flush_sem);
	}

	return written;
}

void sdcard_check_flush(void)
{
	/* The writer thread handles periodic flushing via semaphore timeout.
	 * This function can be called to force a check. */
	if (logging) {
		k_sem_give(&sd_flush_sem);
	}
}

void sdcard_flush(void)
{
	k_sem_give(&sd_flush_sem);
	/* Give writer thread time to process */
	k_msleep(10);
}

void sdcard_mark_event(const char *tag)
{
	strncpy(event_tag, tag, sizeof(event_tag) - 1);
	event_tag[sizeof(event_tag) - 1] = '\0';
	event_pending = true;
}

bool sdcard_consume_event(char *buf, uint32_t buf_size)
{
	if (!event_pending) {
		buf[0] = '\0';
		return false;
	}

	strncpy(buf, event_tag, buf_size - 1);
	buf[buf_size - 1] = '\0';
	event_pending = false;
	event_tag[0] = '\0';
	return true;
}

const char *sdcard_get_filename(void)
{
	return logging ? log_filename : "";
}

uint32_t sdcard_get_sample_count(void)
{
	return sample_count;
}
