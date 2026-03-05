/*
 * CoMotion Tracker — SD Card Module
 *
 * External SD card on SPI2, CS = P0.28 (XIAO D2).
 * Mounts FAT filesystem and writes CSV log files.
 *
 * Uses Zephyr disk_access + FAT FS APIs.
 * DTS provides: sdhc-spi-slot under &spi2 with sdmmc-disk child.
 */

#include <zephyr/kernel.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/fs/fs.h>
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
	.type     = FS_FATFS,
	.fs_data  = &fat_fs,
	.mnt_point = MOUNT_POINT,
};

static bool mounted;

/* ─── Log File ─── */
#define LOG_FILENAME MOUNT_POINT "/LOG.CSV"

static struct fs_file_t log_file;
static bool file_open;

/* ─── Public API ─── */

int sdcard_init(void)
{
	uint32_t block_count, block_size;
	uint64_t size_mb;
	int ret;

	fs_file_t_init(&log_file);

	/* Initialize disk */
	ret = disk_access_init(DISK_NAME);
	if (ret) {
		LOG_WRN("SD: disk_access_init failed: %d (no card?)", ret);
		return ret;
	}

	/* Get card info */
	ret = disk_access_ioctl(DISK_NAME, DISK_IOCTL_GET_SECTOR_COUNT, &block_count);
	if (ret) {
		LOG_ERR("SD: cannot get sector count: %d", ret);
		return ret;
	}

	ret = disk_access_ioctl(DISK_NAME, DISK_IOCTL_GET_SECTOR_SIZE, &block_size);
	if (ret) {
		LOG_ERR("SD: cannot get sector size: %d", ret);
		return ret;
	}

	size_mb = (uint64_t)block_count * block_size / (1024 * 1024);
	LOG_INF("SD: card detected — %u MB", (uint32_t)size_mb);

	/* Mount FAT filesystem */
	ret = fs_mount(&mount_pt);
	if (ret) {
		LOG_ERR("SD: mount failed: %d", ret);
		return ret;
	}

	mounted = true;
	LOG_INF("SD: FAT filesystem mounted at %s", MOUNT_POINT);
	return 0;
}

bool sdcard_is_mounted(void)
{
	return mounted;
}

int sdcard_write_line(const char *line)
{
	int ret;

	if (!mounted) {
		return -ENODEV;
	}

	/* Open file on first write */
	if (!file_open) {
		ret = fs_open(&log_file, LOG_FILENAME,
			      FS_O_CREATE | FS_O_WRITE | FS_O_APPEND);
		if (ret) {
			LOG_ERR("SD: cannot open %s: %d", LOG_FILENAME, ret);
			return ret;
		}
		file_open = true;
		LOG_INF("SD: logging to %s", LOG_FILENAME);
	}

	/* Write the line + newline */
	ret = fs_write(&log_file, line, strlen(line));
	if (ret < 0) {
		LOG_ERR("SD: write failed: %d", ret);
		return ret;
	}

	ret = fs_write(&log_file, "\n", 1);
	if (ret < 0) {
		LOG_ERR("SD: write newline failed: %d", ret);
		return ret;
	}

	return 0;
}

int sdcard_flush(void)
{
	if (!file_open) {
		return 0;
	}
	return fs_sync(&log_file);
}

int sdcard_close(void)
{
	int ret = 0;

	if (file_open) {
		ret = fs_close(&log_file);
		file_open = false;
		if (ret) {
			LOG_ERR("SD: close file failed: %d", ret);
		}
	}

	if (mounted) {
		ret = fs_unmount(&mount_pt);
		mounted = false;
		if (ret) {
			LOG_ERR("SD: unmount failed: %d", ret);
		} else {
			LOG_INF("SD: unmounted");
		}
	}

	return ret;
}
