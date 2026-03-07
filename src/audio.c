/*
 * CoMotion Tracker — Audio Processing Module
 *
 * Dedicated thread captures PDM audio in burst mode at ~10 Hz:
 *   1. Start PDM → read one 100ms block → stop PDM
 *   2. Compute RMS, peak, zero-crossing rate
 *   3. Adapt audio baseline (slow EMA of RMS, paused during impact)
 *   4. Feed impact detector (dual-trigger confirmation)
 *
 * Hardware: MSM261D3526HICPM-C on XIAO BLE Sense
 *   CLK = P1.00, DIN = P0.16, PWR = P1.10 (regulator-boot-on)
 *
 * Uses Zephyr DMIC API backed by nrfx PDM driver.
 */

#include <zephyr/kernel.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/logging/log.h>
#include <math.h>

#include "audio.h"
#include "impact.h"

LOG_MODULE_REGISTER(audio, CONFIG_LOG_DEFAULT_LEVEL);

/* ─── PDM Configuration ─── */
#define SAMPLE_RATE      16000
#define SAMPLE_BITS      16
#define BYTES_PER_SAMPLE sizeof(int16_t)
#define NUM_CHANNELS     1

/* 100ms block = 1600 samples */
#define BLOCK_SIZE       (BYTES_PER_SAMPLE * (SAMPLE_RATE / 10) * NUM_CHANNELS)
#define BLOCK_COUNT      4

K_MEM_SLAB_DEFINE_STATIC(audio_mem_slab, BLOCK_SIZE, BLOCK_COUNT, 4);

/* ─── Baseline adaptation (spec section 3.3) ─── */
#define BASELINE_ALPHA   0.005f   /* slow adaptation rate */
#define BASELINE_INIT    500.0f   /* initial baseline RMS */

/* ─── Thread ─── */
#define AUDIO_STACK_SIZE 2048
#define AUDIO_PRIORITY   7        /* below main (which should be higher prio) */

K_THREAD_STACK_DEFINE(audio_stack, AUDIO_STACK_SIZE);
static struct k_thread audio_thread_data;

/* ─── State ─── */
static const struct device *dmic_dev;
static bool dmic_ready;

/* Latest audio data — written by audio thread, read by main thread */
static volatile struct audio_data latest_data;
static volatile float audio_baseline = BASELINE_INIT;

/* ─── PDM burst-mode helpers ─── */

static int pdm_configure(void)
{
	struct pcm_stream_cfg stream = {
		.pcm_rate   = SAMPLE_RATE,
		.pcm_width  = SAMPLE_BITS,
		.block_size = BLOCK_SIZE,
		.mem_slab   = &audio_mem_slab,
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
			.req_chan_map_lo =
				dmic_build_channel_map(0, 0, PDM_CHAN_LEFT),
			.req_chan_map_hi = 0,
		},
	};

	return dmic_configure(dmic_dev, &cfg);
}

/**
 * Burst-read one 100ms PDM block.
 * Returns a pointer to the sample buffer (caller must free to slab).
 */
static int pdm_burst_read(void **buf_out, uint32_t *size_out)
{
	int ret;

	/* Reconfigure before each burst (resets driver state cleanly) */
	ret = pdm_configure();
	if (ret < 0) {
		return ret;
	}

	/* Start capture */
	ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
	if (ret < 0) {
		return ret;
	}

	/* Read one block (~100ms) */
	ret = dmic_read(dmic_dev, 0, buf_out, size_out, 200);
	if (ret < 0) {
		dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);
		return ret;
	}

	/* Stop capture */
	dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);

	/* Drain any remaining queued buffers */
	void *extra;
	uint32_t extra_size;

	while (dmic_read(dmic_dev, 0, &extra, &extra_size, 0) == 0) {
		k_mem_slab_free(&audio_mem_slab, extra);
	}

	return 0;
}

/* ─── Audio processing ─── */

static void process_audio_block(int16_t *samples, uint32_t num_samples)
{
	int64_t sum_sq = 0;
	int16_t peak = 0;
	uint16_t zcr = 0;
	bool prev_positive = (samples[0] >= 0);

	for (uint32_t i = 0; i < num_samples; i++) {
		int16_t s = samples[i];
		int16_t abs_s = (s < 0) ? -s : s;

		sum_sq += (int32_t)s * (int32_t)s;

		if (abs_s > peak) {
			peak = abs_s;
		}

		/* Zero-crossing detection */
		bool positive = (s >= 0);

		if (i > 0 && positive != prev_positive) {
			zcr++;
		}
		prev_positive = positive;
	}

	/* RMS */
	float rms = sqrtf((float)sum_sq / (float)num_samples);

	/* Update latest data (atomic-ish — single struct write) */
	latest_data.rms = rms;
	latest_data.peak = (uint16_t)peak;
	latest_data.zcr = zcr;

	/* Audio impact check and baseline adaptation (spec section 3.2-3.3) */
	float baseline = audio_baseline;
	bool above_relative = ((float)peak > baseline * 8.0f);
	bool above_absolute = (peak > 3000);
	bool audio_impact = above_relative && above_absolute;

	/* Baseline adapts only when NOT in an impact event */
	if (!audio_impact) {
		audio_baseline = BASELINE_ALPHA * rms
			       + (1.0f - BASELINE_ALPHA) * baseline;
	}

	/* Feed the dual-trigger impact detector */
	impact_feed_audio((uint16_t)peak, baseline);
}

/* ─── Audio thread ─── */

static void audio_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	LOG_INF("Audio thread started (10 Hz burst-mode)");

	while (1) {
		if (!dmic_ready) {
			k_msleep(1000);
			continue;
		}

		void *buffer;
		uint32_t size;
		int ret = pdm_burst_read(&buffer, &size);

		if (ret == 0) {
			int16_t *samples = (int16_t *)buffer;
			uint32_t num_samples = size / BYTES_PER_SAMPLE;

			process_audio_block(samples, num_samples);
			k_mem_slab_free(&audio_mem_slab, buffer);
		} else {
			LOG_WRN("PDM burst read failed: %d", ret);
			k_msleep(100); /* back off on error */
		}

		/*
		 * No explicit sleep needed — the PDM burst read blocks
		 * for ~100ms, naturally pacing the thread at ~10 Hz.
		 */
	}
}

/* ─── Public API ─── */

int audio_init(void)
{
	dmic_dev = DEVICE_DT_GET(DT_NODELABEL(dmic_dev));
	if (!device_is_ready(dmic_dev)) {
		LOG_ERR("DMIC device not ready");
		return -ENODEV;
	}

	int ret = pdm_configure();

	if (ret < 0) {
		LOG_ERR("DMIC configure failed: %d", ret);
		return ret;
	}

	dmic_ready = true;
	LOG_INF("PDM mic ready: %u Hz %u-bit mono (burst mode)", SAMPLE_RATE,
		SAMPLE_BITS);

	/* Start the audio processing thread */
	k_thread_create(&audio_thread_data, audio_stack,
			K_THREAD_STACK_SIZEOF(audio_stack),
			audio_thread_fn, NULL, NULL, NULL,
			AUDIO_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&audio_thread_data, "audio");

	return 0;
}

void audio_get_data(struct audio_data *out)
{
	/* Simple copy — struct is small, reads are effectively atomic */
	out->rms = latest_data.rms;
	out->peak = latest_data.peak;
	out->zcr = latest_data.zcr;
}

float audio_get_baseline(void)
{
	return audio_baseline;
}
