/*
 * CoMotion Tracker — PDM Microphone Module (burst-mode)
 *
 * Uses the Zephyr DMIC API (backed by nrfx PDM driver) to capture
 * 16-bit mono PCM at 16 kHz from the onboard PDM microphone.
 *
 * Operates in burst mode: each pdm_mic_read() starts the PDM, captures
 * one block of audio (100 ms), computes the sound level, then stops.
 * This avoids buffer exhaustion from continuous capture and saves power.
 *
 * The mic power regulator (P1.10) is handled automatically by the
 * regulator framework — the board DTS defines the regulator-fixed
 * node with regulator-boot-on (added in our overlay).
 *
 * Hardware: MSM261D3526HICPM-C
 *   CLK = P1.00  (pinctrl in board DTS)
 *   DIN = P0.16  (pinctrl in board DTS)
 *   PWR = P1.10  (regulator-fixed in board DTS)
 */

#include <zephyr/kernel.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>

#include "pdm_mic.h"

LOG_MODULE_REGISTER(pdm_mic, CONFIG_LOG_DEFAULT_LEVEL);

/* ─── Configuration ─── */
#define SAMPLE_RATE     16000		/* 16 kHz */
#define SAMPLE_BITS     16
#define BYTES_PER_SAMPLE sizeof(int16_t)
#define NUM_CHANNELS    1		/* Mono */

/* Block size for 100 ms of audio */
#define BLOCK_SIZE      (BYTES_PER_SAMPLE * (SAMPLE_RATE / 10) * NUM_CHANNELS)

/* Number of blocks the driver can queue */
#define BLOCK_COUNT     4

/* Memory slab for the driver to allocate audio buffers */
K_MEM_SLAB_DEFINE_STATIC(pdm_mem_slab, BLOCK_SIZE, BLOCK_COUNT, 4);

/* ─── Device ─── */
static const struct device *dmic_dev;
static bool mic_ready;

/**
 * Configure the DMIC hardware. Can be called multiple times
 * (the nrfx driver uninits/reinits internally).
 */
static int pdm_mic_configure(void)
{
	struct pcm_stream_cfg stream = {
		.pcm_rate   = SAMPLE_RATE,
		.pcm_width  = SAMPLE_BITS,
		.block_size = BLOCK_SIZE,
		.mem_slab   = &pdm_mem_slab,
	};

	struct dmic_cfg cfg = {
		.io = {
			.min_pdm_clk_freq = 1000000,
			.max_pdm_clk_freq = 3500000,
			.min_pdm_clk_dc   = 40,
			.max_pdm_clk_dc   = 60,
		},
		.streams = &stream,
		.channel = {
			.req_num_streams = 1,
			.req_num_chan    = 1,
			.req_chan_map_lo = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT),
			.req_chan_map_hi = 0,
		},
	};

	return dmic_configure(dmic_dev, &cfg);
}

int pdm_mic_init(void)
{
	int ret;

	dmic_dev = DEVICE_DT_GET(DT_NODELABEL(dmic_dev));
	if (!device_is_ready(dmic_dev)) {
		LOG_ERR("DMIC device not ready");
		return -ENODEV;
	}

	ret = pdm_mic_configure();
	if (ret < 0) {
		LOG_ERR("DMIC configure failed: %d", ret);
		return ret;
	}

	mic_ready = true;
	LOG_INF("PDM mic ready: %u Hz, %u-bit mono (burst mode)", SAMPLE_RATE, SAMPLE_BITS);
	return 0;
}

int pdm_mic_start(void)
{
	/* No-op in burst mode — start/stop happens inside pdm_mic_read() */
	return 0;
}

int pdm_mic_stop(void)
{
	/* No-op in burst mode */
	return 0;
}

int pdm_mic_read(struct mic_data *data, int32_t timeout_ms)
{
	void *buffer;
	uint32_t size;
	int ret;

	if (!mic_ready) {
		return -ENODEV;
	}

	/* Re-configure before each burst (resets driver state cleanly) */
	ret = pdm_mic_configure();
	if (ret < 0) {
		LOG_ERR("DMIC reconfigure failed: %d", ret);
		return ret;
	}

	/* Start capture */
	ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
	if (ret < 0) {
		LOG_ERR("DMIC start failed: %d", ret);
		return ret;
	}

	/* Read one block (100 ms of audio) */
	ret = dmic_read(dmic_dev, 0, &buffer, &size, timeout_ms);
	if (ret < 0) {
		LOG_ERR("DMIC read failed: %d", ret);
		dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);
		return ret;
	}

	/* Stop capture immediately */
	dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);

	/* Drain any remaining queued buffers */
	void *extra;
	uint32_t extra_size;
	while (dmic_read(dmic_dev, 0, &extra, &extra_size, 0) == 0) {
		k_mem_slab_free(&pdm_mem_slab, extra);
	}

	/* Process PCM samples: compute RMS and peak */
	int16_t *samples = (int16_t *)buffer;
	uint32_t num_samples = size / BYTES_PER_SAMPLE;
	int64_t sum_sq = 0;
	int16_t peak = 0;

	for (uint32_t i = 0; i < num_samples; i++) {
		int16_t s = samples[i];
		int16_t abs_s = (s < 0) ? -s : s;

		sum_sq += (int32_t)s * (int32_t)s;

		if (abs_s > peak) {
			peak = abs_s;
		}
	}

	/* Free the buffer back to the slab */
	k_mem_slab_free(&pdm_mem_slab, buffer);

	/* Compute RMS: sqrt(sum_sq / num_samples) */
	uint32_t mean_sq = (uint32_t)(sum_sq / num_samples);
	uint32_t rms = 0;

	/* Integer square root (Newton's method) */
	if (mean_sq > 0) {
		uint32_t x = mean_sq;
		uint32_t y = (x + 1) / 2;
		while (y < x) {
			x = y;
			y = (x + mean_sq / x) / 2;
		}
		rms = x;
	}

	data->rms = (uint16_t)rms;
	data->peak = peak;

	/* Scale to 0–100%: full-scale int16 is 32767 */
	uint32_t pct = ((uint32_t)rms * 100) / 32767;
	data->level_pct = (pct > 100) ? 100 : (uint8_t)pct;

	return 0;
}
