/*
 * ═══════════════════════════════════════════════════════════════
 * CoMotion Tracker — SD Card Logging
 * ═══════════════════════════════════════════════════════════════
 */
#ifndef COMOTION_SDCARD_H
#define COMOTION_SDCARD_H

#include "common.h"

int  sdcard_init(void);
bool sdcard_start_logging(void);
void sdcard_stop_logging(void);
void sdcard_log_sample(int64_t timestamp);
void sdcard_flush(void);
void sdcard_mark_event(const char *name);

#endif
