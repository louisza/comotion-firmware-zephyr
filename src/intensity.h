/*
 * ═══════════════════════════════════════════════════════════════
 * CoMotion Tracker — Intensity Calculation
 * ═══════════════════════════════════════════════════════════════
 */
#ifndef COMOTION_INTENSITY_H
#define COMOTION_INTENSITY_H

#include "common.h"

void  intensity_update(void);
void  intensity_sample_history(void);
float intensity_get_current(void);
float intensity_get_1min(void);
float intensity_get_10min_avg(void);
void  intensity_reset(void);

#endif
