/*
 * CoMotion Tracker — Lightweight DSP Filter Primitives
 *
 * Reusable, struct-based filters for real-time signal processing on
 * the nRF52840 (Cortex-M4F with hardware single-precision FPU).
 *
 * All filters are stateless functions operating on caller-owned state
 * structs.  This makes them safe to use from any context (ISR, thread,
 * callback) as long as the state struct isn't shared without locking.
 *
 * Filters provided:
 *   - EMA (exponential moving average) — single-pole IIR low-pass
 *   - Median-of-3 — nonlinear spike rejection
 *   - Circular-angle EMA — for bearing/heading (handles 0°/360° wrap)
 *   - 1D Kalman filter — for speed/position with process + measurement noise
 */

#ifndef FILTER_H
#define FILTER_H

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

/* ═══════════════════════════════════════════════════════════════
 *  EMA — Exponential Moving Average (single-pole IIR low-pass)
 *
 *  y[n] = α × x[n] + (1 − α) × y[n−1]
 *
 *  α = 1.0 → no filtering (pass-through)
 *  α → 0   → very heavy smoothing
 *
 *  Time constant τ ≈ dt / α  where dt is the sample period.
 *  For 104 Hz with α = 0.3: τ ≈ 9.6ms / 0.3 ≈ 32ms (~5 Hz cutoff).
 * ═══════════════════════════════════════════════════════════════ */

struct filter_ema {
	float value;        /* current filtered output */
	float alpha;        /* smoothing factor (0, 1] */
	bool  primed;       /* false until first sample seeds the filter */
};

#define FILTER_EMA_INIT(a) { .value = 0.0f, .alpha = (a), .primed = false }

static inline float filter_ema_update(struct filter_ema *f, float sample)
{
	if (!f->primed) {
		f->value = sample;
		f->primed = true;
	} else {
		f->value = f->alpha * sample + (1.0f - f->alpha) * f->value;
	}
	return f->value;
}

static inline void filter_ema_reset(struct filter_ema *f)
{
	f->value = 0.0f;
	f->primed = false;
}

/* ═══════════════════════════════════════════════════════════════
 *  Median-of-3 — Nonlinear spike rejection
 *
 *  Returns the median of the 3 most recent samples.
 *  Excellent for removing single-sample outliers (e.g. GPS glitches)
 *  while preserving signal edges better than a moving average.
 *
 *  Latency: 1 sample.  Needs 2 samples before output is meaningful
 *  (first 2 outputs pass through).
 * ═══════════════════════════════════════════════════════════════ */

struct filter_median3 {
	float buf[3];       /* ring of last 3 samples */
	uint8_t idx;        /* next write position (0–2) */
	uint8_t count;      /* samples received so far (0–3, saturates) */
};

#define FILTER_MEDIAN3_INIT { .buf = {0}, .idx = 0, .count = 0 }

static inline float filter_median3_update(struct filter_median3 *f, float sample)
{
	f->buf[f->idx] = sample;
	f->idx = (f->idx + 1) % 3;
	if (f->count < 3) {
		f->count++;
	}

	if (f->count < 3) {
		return sample;  /* not enough history yet */
	}

	/* Sort 3 values — branchless-friendly on Cortex-M4 */
	float a = f->buf[0], b = f->buf[1], c = f->buf[2];

	if (a > b) { float t = a; a = b; b = t; }
	if (b > c) { float t = b; b = c; c = t; }
	if (a > b) { float t = a; a = b; b = t; }

	return b;  /* median */
}

/* ═══════════════════════════════════════════════════════════════
 *  Circular-angle EMA — for bearings / headings (0–360°)
 *
 *  Handles the 0°/360° discontinuity correctly by filtering in
 *  sin/cos space, then converting back to angle.
 *
 *  Input/output: millidegrees (0–360000) for direct GPS compatibility.
 * ═══════════════════════════════════════════════════════════════ */

struct filter_angle_ema {
	float sin_avg;      /* filtered sin component */
	float cos_avg;       /* filtered cos component */
	float alpha;
	bool  primed;
};

#define FILTER_ANGLE_EMA_INIT(a) { \
	.sin_avg = 0.0f, .cos_avg = 1.0f, .alpha = (a), .primed = false \
}

/**
 * Update the circular EMA with a new angle in millidegrees.
 * Returns the filtered angle in millidegrees (0–360000).
 */
static inline uint32_t filter_angle_ema_update(struct filter_angle_ema *f,
					       uint32_t angle_mdeg)
{
	/* Convert millidegrees → radians */
	float rad = (float)angle_mdeg * (3.14159265f / 180000.0f);
	float s = sinf(rad);
	float c = cosf(rad);

	if (!f->primed) {
		f->sin_avg = s;
		f->cos_avg = c;
		f->primed = true;
	} else {
		f->sin_avg = f->alpha * s + (1.0f - f->alpha) * f->sin_avg;
		f->cos_avg = f->alpha * c + (1.0f - f->alpha) * f->cos_avg;
	}

	/* Convert back to millidegrees (0–360000) */
	float out_rad = atan2f(f->sin_avg, f->cos_avg);

	if (out_rad < 0.0f) {
		out_rad += 2.0f * 3.14159265f;
	}

	return (uint32_t)(out_rad * (180000.0f / 3.14159265f));
}

/* ═══════════════════════════════════════════════════════════════
 *  1D Kalman Filter — for speed or scalar position
 *
 *  State: single scalar (e.g. speed in mm/s)
 *  Model: constant-velocity with process noise Q (acceleration²)
 *         and measurement noise R (sensor variance).
 *
 *  Q controls responsiveness — larger = trusts measurements more
 *  R controls smoothness   — larger = trusts prediction more
 *
 *  For GPS speed at 5Hz:
 *    Q ≈ (5 m/s² × 0.2s)² = 1.0 (m/s)² → allows human sprinting
 *    R ≈ (2 m/s)² = 4.0 (m/s)² → typical GPS speed noise
 * ═══════════════════════════════════════════════════════════════ */

struct filter_kalman1d {
	float x;            /* state estimate */
	float p;            /* estimate covariance */
	float q;            /* process noise variance */
	float r;            /* measurement noise variance */
	bool  primed;
};

#define FILTER_KALMAN1D_INIT(process_noise, meas_noise) { \
	.x = 0.0f, .p = 1.0f, \
	.q = (process_noise), .r = (meas_noise), \
	.primed = false \
}

/**
 * Update the 1D Kalman filter with a new measurement.
 * Returns the filtered estimate.
 */
static inline float filter_kalman1d_update(struct filter_kalman1d *f,
					   float measurement)
{
	if (!f->primed) {
		f->x = measurement;
		f->p = f->r;   /* initial uncertainty = measurement noise */
		f->primed = true;
		return f->x;
	}

	/* Predict (constant-velocity model: x_pred = x, p_pred = p + Q) */
	float p_pred = f->p + f->q;

	/* Update */
	float k = p_pred / (p_pred + f->r);  /* Kalman gain */

	f->x = f->x + k * (measurement - f->x);
	f->p = (1.0f - k) * p_pred;

	return f->x;
}

static inline void filter_kalman1d_reset(struct filter_kalman1d *f)
{
	f->x = 0.0f;
	f->p = 1.0f;
	f->primed = false;
}

#endif /* FILTER_H */
