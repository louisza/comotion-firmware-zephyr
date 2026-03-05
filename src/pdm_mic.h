/*
 * CoMotion Tracker — PDM Microphone Module
 *
 * Uses the onboard MSM261D3526HICPM-C PDM microphone via Zephyr DMIC API.
 * Captures 16-bit mono PCM at 16 kHz and computes RMS sound level.
 */

#ifndef PDM_MIC_H
#define PDM_MIC_H

#include <stdint.h>

/** Sound level data from one capture block */
struct mic_data {
	uint16_t rms;		/* RMS amplitude (0–32767) */
	int16_t  peak;		/* Peak absolute sample value */
	uint8_t  level_pct;	/* Rough loudness 0–100% */
};

/**
 * Initialize and configure the PDM microphone.
 * Call once at boot after regulators are up.
 *
 * @return 0 on success, negative errno on failure
 */
int pdm_mic_init(void);

/**
 * Start continuous PDM capture.
 * After this, pdm_mic_read() will return audio blocks.
 *
 * @return 0 on success, negative errno on failure
 */
int pdm_mic_start(void);

/**
 * Stop PDM capture.
 *
 * @return 0 on success, negative errno on failure
 */
int pdm_mic_stop(void);

/**
 * Read one block of PCM data and compute sound level.
 * Blocks for up to @p timeout_ms waiting for data.
 *
 * @param[out] data  Sound level results
 * @param[in]  timeout_ms  Max wait time in milliseconds
 * @return 0 on success, -EAGAIN on timeout, negative errno on error
 */
int pdm_mic_read(struct mic_data *data, int32_t timeout_ms);

#endif /* PDM_MIC_H */
