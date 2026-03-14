/*
 * CoMotion Tracker — SD Card Logging Module
 *
 * Ring-buffer based SD card logger for 104 Hz CSV data.
 * Explicit start/stop logging via BLE commands.
 * Session files: LOG001.CSV through LOG999.CSV.
 */

#ifndef SDCARD_H
#define SDCARD_H

#include <stdint.h>
#include <stdbool.h>

/**
 * Initialize SD card: check disk, mount FAT filesystem.
 *
 * @return 0 on success, negative errno on failure
 */
int sdcard_init(void);

/**
 * Check if SD card is mounted and ready.
 */
bool sdcard_is_mounted(void);

/**
 * Start a new logging session.
 * Opens next LOGnnn.CSV file, writes CSV header, begins accepting data.
 *
 * @return 0 on success, negative errno on failure
 */
int sdcard_start_logging(void);

/**
 * Stop the current logging session.
 * Flushes ring buffer, closes file.
 *
 * @return 0 on success, negative errno on failure
 */
int sdcard_stop_logging(void);

/**
 * Check if logging is currently active.
 */
bool sdcard_is_logging(void);

/**
 * Write a line to the ring buffer for later flush to SD.
 * Non-blocking; drops data if ring buffer is full.
 *
 * @param data   Pointer to data bytes
 * @param len    Number of bytes to write
 * @return number of bytes written, or 0 if buffer full
 */
uint32_t sdcard_write(const char *data, uint32_t len);

/**
 * Check if ring buffer needs flushing (≥50% full or 5s elapsed).
 * If so, flushes to SD card. Call from main loop or timer.
 */
void sdcard_check_flush(void);

/**
 * Force an immediate flush of the ring buffer to SD.
 */
void sdcard_flush(void);

/**
 * Mark an event tag to be included in the next logged CSV row.
 * The tag is cleared after it appears in one row.
 *
 * @param tag  Event string (will be truncated to 31 chars)
 */
void sdcard_mark_event(const char *tag);

/**
 * Get the current event tag (empty string if none pending).
 * Calling this consumes the event (returns it once).
 *
 * @param buf      Destination buffer
 * @param buf_size Size of destination buffer
 * @return true if an event was pending
 */
bool sdcard_consume_event(char *buf, uint32_t buf_size);

/**
 * Get current session filename (e.g., "LOG042.CSV").
 * Returns empty string if not logging.
 */
const char *sdcard_get_filename(void);

/**
 * Get number of CSV data rows written in current session.
 */
uint32_t sdcard_get_sample_count(void);

/* ─── Log Transfer (NUS commands) ─── */
void sdcard_handle_list(void);
void sdcard_handle_dump(const char *filename);
void sdcard_handle_dump_latest(void);
void sdcard_handle_delete(const char *filename);
void sdcard_handle_status_cmd(void);
void sdcard_handle_abort(void);

/* ─── Player Name (persisted to SD card) ─── */

/**
 * Set the player name (from NUS NAME: command).
 * Persists to /SD:/player.txt for survival across resets.
 */
void sdcard_set_player_name(const char *name);

/**
 * Get the current player name. Returns "" if unset.
 */
const char *sdcard_get_player_name(void);

/**
 * Check if auto-start flag is set (device should log on boot).
 * Flag is stored in /SD:/autostart.txt.
 */
bool sdcard_get_autostart(void);

/**
 * Set or clear the auto-start flag.
 */
void sdcard_set_autostart(bool enabled);

#endif /* SDCARD_H */
