/*
 * CoMotion Tracker — Device Identity Module
 *
 * Reads the nRF52840's factory-programmed FICR.DEVICEID registers
 * to produce a stable, unique 8-char hex ID per physical chip.
 *
 * Used for: BLE device name suffix, CSV header metadata,
 * BLE packet identification, and player assignment in the app.
 */

#ifndef DEVICE_ID_H
#define DEVICE_ID_H

#include <stdint.h>

/**
 * Read FICR and compute device identity.
 * Must be called once at boot, before BLE or SD card init.
 */
void device_id_init(void);

/**
 * Get the full 8-char hex device ID (e.g. "A3F7B2C1").
 * Null-terminated, always uppercase.
 */
const char *device_id_full(void);

/**
 * Get the short 4-char device ID suffix (e.g. "A3F7").
 * Used for BLE name: "CoMotion-A3F7"
 */
const char *device_id_short(void);

/**
 * Get the raw 4-byte device ID value.
 */
uint32_t device_id_raw(void);

#endif /* DEVICE_ID_H */
