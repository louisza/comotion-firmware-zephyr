/*
 * ═══════════════════════════════════════════════════════════════
 * CoMotion Tracker — GPS (UART + NMEA Parser)
 * ═══════════════════════════════════════════════════════════════
 */
#ifndef COMOTION_GPS_H
#define COMOTION_GPS_H

#include "common.h"

int  gps_init(void);
void gps_poll(void);  /* Called from main loop or thread */

#endif
