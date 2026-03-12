/*
 * CoMotion Tracker — Device Identity Module
 *
 * Uses nRF52840 FICR (Factory Information Configuration Registers)
 * which are programmed at the factory and unique per chip.
 *
 * FICR.DEVICEID[0] and FICR.DEVICEID[1] are 32-bit values.
 * We XOR them to get a single 32-bit hash, then format as hex.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <soc.h>
#include <stdio.h>
#include <string.h>

#include "device_id.h"

LOG_MODULE_REGISTER(device_id, CONFIG_LOG_DEFAULT_LEVEL);

static uint32_t dev_id_raw;
static char dev_id_full_str[9];   /* "A3F7B2C1\0" */
static char dev_id_short_str[5];  /* "A3F7\0" */
static bool initialized;

void device_id_init(void)
{
	/* Read factory-programmed device ID registers */
	uint32_t id0 = NRF_FICR->DEVICEID[0];
	uint32_t id1 = NRF_FICR->DEVICEID[1];

	/* XOR to mix both words into one 32-bit value */
	dev_id_raw = id0 ^ id1;

	/* Format as uppercase hex */
	snprintf(dev_id_full_str, sizeof(dev_id_full_str),
		 "%08X", dev_id_raw);

	/* Short suffix = first 4 chars (most significant bytes) */
	memcpy(dev_id_short_str, dev_id_full_str, 4);
	dev_id_short_str[4] = '\0';

	initialized = true;

	LOG_INF("Device ID: %s (FICR: %08x ^ %08x)", dev_id_full_str, id0, id1);
}

const char *device_id_full(void)
{
	if (!initialized) {
		return "00000000";
	}
	return dev_id_full_str;
}

const char *device_id_short(void)
{
	if (!initialized) {
		return "0000";
	}
	return dev_id_short_str;
}

uint32_t device_id_raw(void)
{
	return dev_id_raw;
}
