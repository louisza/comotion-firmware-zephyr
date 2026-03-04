/*
 * ═══════════════════════════════════════════════════════════════
 * CoMotion Tracker — Sensors (IMU, PDM, Battery)
 * ═══════════════════════════════════════════════════════════════
 */
#ifndef COMOTION_SENSORS_H
#define COMOTION_SENSORS_H

#include "common.h"

int  sensors_init(void);
void sensors_read_imu(void);
void sensors_process_audio(void);
float sensors_read_battery(void);

/* Gyro calibration */
bool sensors_calibrate_gyro(void);

#endif
