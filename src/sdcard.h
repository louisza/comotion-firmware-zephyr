/*
 * CoMotion Tracker — SD Card Module
 *
 * Mount/unmount FAT filesystem on external SD card (SPI).
 * Write sensor data as CSV log files.
 */

#ifndef SDCARD_H
#define SDCARD_H

#include <stdint.h>
#include <stdbool.h>

/**
 * Initialize the SD card subsystem: check disk, mount FAT FS.
 *
 * @return 0 on success, negative errno on failure
 */
int sdcard_init(void);

/**
 * Check if SD card is mounted and ready.
 */
bool sdcard_is_mounted(void);

/**
 * Write a line of text to the current log file.
 * Creates the file on first call.
 *
 * @param line  Null-terminated string to write (newline appended automatically)
 * @return 0 on success, negative errno on failure
 */
int sdcard_write_line(const char *line);

/**
 * Flush buffered data to the SD card.
 * Call periodically to avoid data loss.
 *
 * @return 0 on success, negative errno on failure
 */
int sdcard_flush(void);

/**
 * Close the log file and unmount the filesystem.
 *
 * @return 0 on success, negative errno on failure
 */
int sdcard_close(void);

#endif /* SDCARD_H */
