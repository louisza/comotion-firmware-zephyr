/*
 * CoMotion Tracker — Audio Processing Module
 *
 * Replaces pdm_mic module. Adds a dedicated processing thread at 10 Hz,
 * baseline adaptation for impact detection, and zero-crossing rate.
 *
 * Uses burst-mode PDM capture (proven reliable on this hardware).
 */

#ifndef AUDIO_H
#define AUDIO_H

#include <stdint.h>

/** Audio metrics from one 100ms capture window */
struct audio_data {
	float    rms;        /* RMS amplitude (float, from int16 samples) */
	uint16_t peak;       /* Peak absolute sample value (0–32767) */
	uint16_t zcr;        /* Zero crossing count in the window */
};

/**
 * Initialize PDM hardware and start the audio processing thread.
 * The thread runs at ~10 Hz using burst-mode PDM capture.
 *
 * @return 0 on success, negative errno on failure
 */
int audio_init(void);

/**
 * Get a snapshot of the latest audio data.
 * Thread-safe (atomic copy).
 *
 * @param[out] out  Destination for audio metrics
 */
void audio_get_data(struct audio_data *out);

/**
 * Get the current audio baseline (slowly-adapted RMS).
 * Used for impact detection threshold calculation.
 */
float audio_get_baseline(void);

#endif /* AUDIO_H */
