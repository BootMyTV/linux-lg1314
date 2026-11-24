/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Floating point helpers for the files built with the FPU enabled
 * (lg1314_dovi_dm.c, lg1314_hdr_tf.c): no libm in the kernel, so the few
 * transcendentals needed are here. Include only from those files; callers
 * bracket their use with kernel_fpu_begin() / kernel_fpu_end().
 */
#ifndef _LG1314_FPMATH_H_
#define _LG1314_FPMATH_H_

#include <linux/types.h>

static inline double dm_min(double a, double b) { return a < b ? a : b; }
static inline double dm_max(double a, double b) { return a > b ? a : b; }
static inline double dm_clamp(double v, double lo, double hi) { return dm_min(dm_max(v, lo), hi); }

union dbits {
	double	d;
	u64	u;
};

/* log2(x), x > 0 */
static inline double dm_log2(double x)
{
	union dbits b = { .d = x };
	int e = (int)((b.u >> 52) & 0x7ff) - 1023;
	double m, s, s2, t;

	b.u = (b.u & ~(0x7ffULL << 52)) | (1023ULL << 52);	/* m in [1, 2) */
	m = b.d;
	if (m > 1.4142135623730951) {
		m *= 0.5;
		e++;
	}
	/* ln(m) = 2 atanh(s), s = (m - 1) / (m + 1), |s| <= 0.1716: to s^15
	 * (the next term is below 6e-15) */
	s = (m - 1) / (m + 1);
	s2 = s * s;
	t = s2 * (1.0 / 15);
	t = s2 * (1.0 / 13 + t);
	t = s2 * (1.0 / 11 + t);
	t = s2 * (1.0 / 9 + t);
	t = s2 * (1.0 / 7 + t);
	t = s2 * (1.0 / 5 + t);
	t = s2 * (1.0 / 3 + t);
	return e + 2 * s * (1 + t) * 1.4426950408889634;
}

/* 2^x */
static inline double dm_exp2(double x)
{
	/* 1 / i */
	static const double inv[12] = {
		0, 1.0 / 1, 1.0 / 2, 1.0 / 3, 1.0 / 4, 1.0 / 5, 1.0 / 6, 1.0 / 7,
		1.0 / 8, 1.0 / 9, 1.0 / 10, 1.0 / 11,
	};
	union dbits b;
	double n, f, r;
	int i;

	if (x < -1020)
		return 0;
	if (x > 1020)
		x = 1020;
	n = (double)(long)(x + (x >= 0 ? 0.5 : -0.5));
	f = (x - n) * 0.6931471805599453;	/* |f| <= ln2 / 2 */
	/* e^f, Taylor to degree 11 (the next term is below 7e-15) */
	r = 1;
	for (i = 11; i > 0; i--)
		r = 1 + r * f * inv[i];
	b.u = (u64)((long)n + 1023) << 52;
	return r * b.d;
}

static inline double dm_pow(double x, double y)
{
	if (x <= 0)
		return 0;
	return dm_exp2(y * dm_log2(x));
}

/* SMPTE ST 2084 */
#define PQ_M1	(2610.0 / 16384)
#define PQ_M2	(2523.0 / 4096 * 128)
#define PQ_C1	(3424.0 / 4096)
#define PQ_C2	(2413.0 / 4096 * 32)
#define PQ_C3	(2392.0 / 4096 * 32)

/* A PQ signal (0..1) in cd/m² */
static inline double pq_to_nits(double v)
{
	double p;

	if (v <= 0)
		return 0;
	p = dm_pow(v, 1 / PQ_M2);
	return 10000 * dm_pow(dm_max(p - PQ_C1, 0) / (PQ_C2 - PQ_C3 * p), 1 / PQ_M1);
}

#endif
