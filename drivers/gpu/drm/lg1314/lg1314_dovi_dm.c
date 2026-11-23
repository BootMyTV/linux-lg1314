// SPDX-License-Identifier: GPL-2.0
/*
 * LG1314 Dolby Vision display management, DM4 model, in floating point.
 *
 * PQ signals are normalized to 0..1 (12-bit codes / 4096, as the hardware's
 * tables use them), luminance is in cd/m².
 *
 * Built with the FPU enabled; no libm in the kernel, so the few
 * transcendentals needed are here.
 */
#include <linux/bits.h>

#include "lg1314_dovi_dm.h"

#define E12	(1.0 / 4096)	/* one 12-bit code */

/* ---- math -------------------------------------------------------------- */

static double dm_min(double a, double b) { return a < b ? a : b; }
static double dm_max(double a, double b) { return a > b ? a : b; }
static double dm_clamp(double v, double lo, double hi) { return dm_min(dm_max(v, lo), hi); }

union dbits {
	double	d;
	u64	u;
};

/* log2(x), x > 0 */
static double dm_log2(double x)
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

/* 1 / i */
static const double dm_inv[12] = {
	0, 1.0 / 1, 1.0 / 2, 1.0 / 3, 1.0 / 4, 1.0 / 5, 1.0 / 6, 1.0 / 7,
	1.0 / 8, 1.0 / 9, 1.0 / 10, 1.0 / 11,
};

/* 2^x */
static double dm_exp2(double x)
{
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
		r = 1 + r * f * dm_inv[i];
	b.u = (u64)((long)n + 1023) << 52;
	return r * b.d;
}

static double dm_pow(double x, double y)
{
	if (x <= 0)
		return 0;
	return dm_exp2(y * dm_log2(x));
}

static double dm_sqrt(double x)
{
	return x > 0 ? __builtin_sqrt(x) : 0;
}

/* SMPTE ST 2084 */
#define PQ_M1	(2610.0 / 16384)
#define PQ_M2	(2523.0 / 4096 * 128)
#define PQ_C1	(3424.0 / 4096)
#define PQ_C2	(2413.0 / 4096 * 32)
#define PQ_C3	(2392.0 / 4096 * 32)

static double pq_to_nits(double v)
{
	double p;

	if (v <= 0)
		return 0;
	p = dm_pow(v, 1 / PQ_M2);
	return 10000 * dm_pow(dm_max(p - PQ_C1, 0) / (PQ_C2 - PQ_C3 * p), 1 / PQ_M1);
}

static double nits_to_pq(double l)
{
	double y;

	if (l <= 0)
		return 0;
	y = dm_pow(l / 10000, PQ_M1);
	return dm_pow((PQ_C1 + PQ_C2 * y) / (1 + PQ_C3 * y), PQ_M2);
}

static void m33(const s16 *m, unsigned int frac, double out[9])
{
	double s = 1.0 / (1 << frac);
	int i;

	for (i = 0; i < 9; i++)
		out[i] = m[i] * s;
}

/* ---- tone curve ---------------------------------------------------------- */

struct tc {
	double	x[3], y[3], m[3];
	double	tmin, tmax;
	double	slope, offset, power;
};

/* Cubic Hermite between (xa, ya, ma) and (xb, yb, mb). */
static double hermite(double x, double xa, double ya, double ma,
		      double xb, double yb, double mb)
{
	double h = xb - xa, t = (x - xa) / h, t2 = t * t, t3 = t2 * t;

	return (2 * t3 - 3 * t2 + 1) * ya + (t3 - 2 * t2 + t) * h * ma +
	       (3 * t2 - 2 * t3) * yb + (t3 - t2) * h * mb;
}

/* The curve at x (PQ): the spline, then the slope / offset / power trim. */
static double tc_eval(const struct tc *t, double x)
{
	double y, span = t->tmax - t->tmin, u, r;

	x = dm_clamp(x, 0, 1);
	if (x <= t->x[0])
		y = t->y[0] + (x - t->x[0]) * t->m[0];
	else if (x <= t->x[1])
		y = hermite(x, t->x[0], t->y[0], t->m[0], t->x[1], t->y[1], t->m[1]);
	else if (x < t->x[2])
		y = hermite(x, t->x[1], t->y[1], t->m[1], t->x[2], t->y[2], t->m[2]);
	else
		y = t->y[2] + (x - t->x[2]) * t->m[2];

	u = t->offset + t->slope * (y - t->tmin) / span;
	r = dm_pow(u < 0 ? -u : u, dm_min(t->power, 2.0));
	y = (u < 0 ? -r : r) * span + t->tmin;
	return dm_clamp(y, 0, 1);
}

/*
 * Where the source mid-tone lands: a helper spline through (lower end ->
 * just above target black), (target black + 0.1 -> itself), (upper end ->
 * 0.1 below target peak).
 */
static double mid_map(double smin, double smid, double smax, double tmin, double tmax)
{
	double c = tmin + 0.1, ylo = tmin + 0.02;
	double x0 = dm_min(smin, tmin), xe = dm_max(tmax, smax);
	double ye = dm_max(tmax - 0.1, c);
	double dx0 = c - x0, dx1 = xe - c;
	double s0 = dm_max(3 * (c - ylo) / dx0, 0), s1 = dm_max(3 * (ye - c) / dx1, 0);
	double ma, mc, me;

	mc = dm_min(dm_min(dm_min(2 * (tmax - tmin), 1), s0), s1);
	ma = dm_min(dm_min(s0, 1), (c - ylo) * (c - ylo) / (dx0 * dx0));
	me = dm_min(dm_min(s1, 1), (ye - c) * (ye - c) / (dx1 * dx1));
	if (smid <= c)
		return hermite(smid, x0, ylo, ma, c, c, mc);
	return hermite(smid, c, c, mc, xe, ye, me);
}

/* ICtCp of the red primary at luminance @l through @rgb2lms. */
static void red_ictcp(double l, const double *rgb2lms, const double *lms2ictcp,
		      double out[3])
{
	double lms[3];
	int i;

	for (i = 0; i < 3; i++)
		lms[i] = nits_to_pq(dm_max(l * rgb2lms[i * 3], 0));
	for (i = 0; i < 3; i++)
		out[i] = lms2ictcp[i * 3] * lms[0] + lms2ictcp[i * 3 + 1] * lms[1] +
			 lms2ictcp[i * 3 + 2] * lms[2];
}

/* Saturation scale for a source intensity @xi the curve maps to @yi. */
static double sat_scale(const struct tc *t, double smin, double yi, double xi,
			double gain)
{
	double r = dm_min(dm_max(yi - t->tmin, E12) / dm_max(t->tmin - smin, E12), 1);
	double k = -(r - 1) * (r - 1), w, a, b, ratio, inv, g;

	if (xi <= 0x64000 / 67108864.0) {
		w = 0;
	} else if (xi < 0xe10000 / 67108864.0) {
		double v = (xi - 0x64000 / 67108864.0) * 4096 / 875;

		w = 1 - (1 - v) * (1 - v);
	} else {
		w = 1;
	}
	a = dm_max(yi, E12) - 1;
	b = dm_max(xi, E12) - 1;
	ratio = (1 + a * a * a) / (1 + b * b * b);
	inv = 1 / ratio;
	g = inv >= w ? dm_min(inv, 1) : w;
	return dm_clamp(ratio * g * (1 + k) * gain, 0, 2);
}

static s32 q26(double v)
{
	return (s32)(v * 67108864 + (v < 0 ? -0.5 : 0.5));
}

/* The curve and the saturation terms derived with it. */
struct tc_sat {
	double	smin;			/* the scene's black, as the curve uses it */
	double	gain;			/* saturation gain trim */
	double	chroma_src;		/* the source red primary's chroma */
	double	sat_i, sat_c;		/* intensity / chroma reduction at it */
	double	cw;			/* chroma weight trim */
	double	src_red[3], tgt_red[3];
};

static void tc_setup(const struct dm_tc_in *in, struct tc *t, struct tc_sat *sc)
{
	double smin = in->src_min * E12, smid = in->src_mid * E12, smax = in->src_max * E12;
	double m1[9], m2[9], m3[9];
	double ymid, a, b, c, y0, y1, y2, s_lo, s_hi, r, mid;
	double chroma_tgt, ti, sat;

	t->tmin = in->tgt_min * E12;
	t->tmax = in->tgt_max * E12;
	t->slope = in->slope / 4096.0;
	t->offset = in->offset / 4096.0;
	t->power = in->power / 4096.0;

	/* the scene's range, at least a code each side of the mid */
	smid = dm_clamp(smid, E12, 1 - E12);
	smin = smin < 0 ? 0 : dm_min(smin, smid - E12);
	smax = smid + E12 <= smax ? dm_min(smax, 1) : smid + E12;

	/* anchors: the mid where mid_map puts it, the ends at the source's
	 * distance from it, all within the target */
	ymid = mid_map(smin, smid, smax, t->tmin, t->tmax);
	a = dm_min(ymid - t->tmin, smid - smin);
	a = t->tmin + a < smid ? 0 : t->tmin + a - smid;
	b = dm_min(t->tmax - ymid, smax - smid);
	c = dm_min(t->tmax - b, smid);
	y1 = dm_min(dm_max(a + c, t->tmin + E12), t->tmax - E12);
	y0 = dm_max(y1 - smid + smin, t->tmin);
	y2 = dm_min(smax + y1 - smid, t->tmax);

	/* slopes: monotone (at most 3x the secant), the ends at most the
	 * secant squared, the mid's bounded by the mid contrast */
	s_lo = 3 * (y1 - y0) / (smid - smin);
	s_hi = 3 * (y2 - y1) / (smax - smid);
	r = (y1 - y0) / (smid - smin);
	t->m[0] = dm_min(s_lo, r * r);
	mid = dm_max(in->mid_contrast / 4096.0 + 1 - smid + y1, 0);
	t->m[1] = dm_min(dm_min(s_lo, s_hi), mid);
	r = (y2 - y1) / (smax - smid);
	t->m[2] = dm_min(dm_min(s_hi, 1), r * r);
	t->x[0] = smin;
	t->x[1] = smid;
	t->x[2] = smax;
	t->y[0] = y0;
	t->y[1] = y1;
	t->y[2] = y2;

	/* saturation: the red primary at the source's and the target's peak */
	m33(in->src_rgb2lms, in->src_rgb2lms_frac, m1);
	m33(in->tgt_rgb2lms, in->tgt_rgb2lms_frac, m2);
	m33(in->lms2ictcp, in->lms2ictcp_frac, m3);
	red_ictcp(dm_max(pq_to_nits(smax), 5), m1, m3, sc->src_red);
	red_ictcp(pq_to_nits(t->tmax), m2, m3, sc->tgt_red);
	sc->smin = smin;
	sc->gain = in->saturation_gain / 4096.0;
	sc->cw = in->chroma_weight / 4096.0;
	sc->chroma_src = dm_sqrt(sc->src_red[1] * sc->src_red[1] +
				 sc->src_red[2] * sc->src_red[2]);
	ti = dm_max(tc_eval(t, sc->src_red[0]), E12);
	sat = sat_scale(t, smin, ti, sc->src_red[0], sc->gain);
	chroma_tgt = dm_sqrt(sc->tgt_red[1] * sc->tgt_red[1] + sc->tgt_red[2] * sc->tgt_red[2]);
	sc->sat_i = dm_min(sc->tgt_red[0] / ti, 1) - 1;
	sc->sat_c = dm_min(chroma_tgt / dm_max(sat * sc->chroma_src, E12), 1) - 1;
	sc->chroma_src = dm_max(sc->chroma_src, E12);
}

void dm_tone_curve(const struct dm_tc_in *in, u16 curve[DM_TC_SAMPLES],
		   struct dm_tc_params *params)
{
	struct tc_sat sc;
	struct tc t;
	int i;

	tc_setup(in, &t, &sc);
	for (i = 0; i < DM_TC_SAMPLES; i++) {
		double y = tc_eval(&t, i * (0x20100 / 67108864.0));
		int v = (int)(y * 4096 + 0.125);

		curve[i] = v > 4095 ? 4095 : v;
	}
	if (params) {
		for (i = 0; i < 3; i++) {
			params->x[i] = q26(t.x[i]);
			params->y[i] = q26(t.y[i]);
			params->m[i] = q26(t.m[i]);
			params->src_red[i] = q26(sc.src_red[i]);
			params->tgt_red[i] = q26(sc.tgt_red[i]);
		}
		params->chroma_src = q26(sc.chroma_src);
		params->sat_i = q26(sc.sat_i);
		params->sat_c = q26(sc.sat_c);
	}
}

/* ---- colour volumes ------------------------------------------------------ */

/*
 * Primaries, xy: R, G, B, white; by the index Dolby's metadata (L9, L10) and
 * target display table use (0 P3-D65, 1 BT.709, 2 BT.2020, ...).
 */
static const double dm_prims[][8] = {
	/*  Rx        Ry        Gx        Gy        Bx        By        Wx        Wy */
	{  0.68000,  0.32000,  0.26500,  0.69000,  0.15000,  0.06000,  0.31270,  0.32900 },	/*  0 P3-D65 */
	{  0.64000,  0.33000,  0.30000,  0.60000,  0.15000,  0.06000,  0.31270,  0.32900 },	/*  1 BT.709 */
	{  0.70800,  0.29200,  0.17000,  0.79700,  0.13100,  0.04600,  0.31270,  0.32900 },	/*  2 BT.2020 */
	{  0.63000,  0.34000,  0.31000,  0.59500,  0.15500,  0.07000,  0.31270,  0.32900 },	/*  3 BT.601 525 (SMPTE C) */
	{  0.64000,  0.33000,  0.29000,  0.60000,  0.15000,  0.06000,  0.31270,  0.32900 },	/*  4 BT.601 625 (EBU) */
	{  0.68000,  0.32000,  0.26500,  0.69000,  0.15000,  0.06000,  0.31400,  0.35100 },	/*  5 DCI-P3 (DCI white) */
	{  0.73470,  0.26530,  0.00000,  1.00000,  0.00010, -0.07700,  0.32168,  0.33767 },	/*  6 ACES AP0 */
	{  0.73000,  0.28000,  0.14000,  0.85500,  0.10000, -0.05000,  0.31270,  0.32900 },	/*  7 S-Gamut */
	{  0.76600,  0.27500,  0.22500,  0.80000,  0.08900, -0.08700,  0.31270,  0.32900 },	/*  8 S-Gamut3.Cine */
	{  0.69300,  0.30400,  0.20800,  0.76100,  0.14670,  0.05270,  0.31270,  0.32900 },	/*  9 */
	{  0.68670,  0.30850,  0.23100,  0.69000,  0.14890,  0.06380,  0.31270,  0.32900 },	/* 10 */
	{  0.67810,  0.31890,  0.23650,  0.70480,  0.14100,  0.04890,  0.31270,  0.32900 },	/* 11 */
	{  0.68000,  0.32000,  0.26500,  0.69000,  0.15000,  0.06000,  0.31270,  0.32900 },	/* 12 P3-D65 (again) */
	{  0.70420,  0.29400,  0.22710,  0.72500,  0.14160,  0.05160,  0.31270,  0.32900 },	/* 13 */
	{  0.67450,  0.31020,  0.22120,  0.71090,  0.15200,  0.06190,  0.31270,  0.32900 },	/* 14 */
	{  0.68050,  0.31910,  0.25220,  0.67020,  0.13970,  0.05540,  0.31270,  0.32900 },	/* 15 */
	{  0.68380,  0.30850,  0.27090,  0.63780,  0.14780,  0.05890,  0.31270,  0.32900 },	/* 16 */
	{  0.67530,  0.31930,  0.26360,  0.68350,  0.15210,  0.06270,  0.31270,  0.32900 },	/* 17 */
	{  0.69810,  0.28980,  0.18140,  0.71890,  0.15170,  0.05670,  0.31270,  0.32900 },	/* 18 */
};

#define DM_NPRIMS	(sizeof(dm_prims) / sizeof(dm_prims[0]))

/* Dolby's predefined L8 target displays: index, peak, primaries. */
static const struct {
	u8	index;
	u16	nits;
	u8	prim;
} dm_displays[] = {
	{ 1, 100, 1 }, { 27, 600, 0 }, { 28, 600, 2 }, { 37, 2000, 0 }, { 38, 2000, 2 },
	{ 48, 1000, 0 }, { 49, 1000, 2 },
};

/* XYZ -> LMS (Hunt-Pointer-Estevez), the DM's LMS crosstalk, LMS' -> ICtCp. */
static const double dm_xyz2lms[9] = {
	0.40023822, 0.70759315, -0.08080558,
	-0.22629810, 1.16531559, 0.04570077,
	0, 0, 0.91822495,
};
#define DM_CROSSTALK	0.04		/* DM4's LMS (ICtCp's) */
#define DM3_CROSSTALK	(654 / 32768.0)	/* DM3's (IPTPQc2's 2 %, as stock rounds it) */
static const double dm_lms2ictcp[9] = {
	0.5, 0.5, 0,
	3305 / 4096.0, -6806 / 4096.0, 3501 / 4096.0,
	17933 / 4096.0, -17390 / 4096.0, -543 / 4096.0,
};

static void mul33(const double *a, const double *b, double *out)
{
	double r[9];
	int i, j;

	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
			r[i * 3 + j] = a[i * 3] * b[j] + a[i * 3 + 1] * b[3 + j] +
				       a[i * 3 + 2] * b[6 + j];
	for (i = 0; i < 9; i++)
		out[i] = r[i];
}

static int inv33(const double *m, double *out)
{
	double d = m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) +
		   m[2] * (m[3] * m[7] - m[4] * m[6]);

	if (d == 0)
		return -1;
	out[0] = (m[4] * m[8] - m[5] * m[7]) / d;
	out[1] = (m[2] * m[7] - m[1] * m[8]) / d;
	out[2] = (m[1] * m[5] - m[2] * m[4]) / d;
	out[3] = (m[5] * m[6] - m[3] * m[8]) / d;
	out[4] = (m[0] * m[8] - m[2] * m[6]) / d;
	out[5] = (m[2] * m[3] - m[0] * m[5]) / d;
	out[6] = (m[3] * m[7] - m[4] * m[6]) / d;
	out[7] = (m[1] * m[6] - m[0] * m[7]) / d;
	out[8] = (m[0] * m[4] - m[1] * m[3]) / d;
	return 0;
}

/* RGB -> XYZ for primaries @p (white Y = 1). */
static void rgb2xyz(const double *p, double *m)
{
	double prim[9], inv[9], w[3], s[3];
	int i;

	for (i = 0; i < 3; i++) {
		prim[i] = p[2 * i] / p[2 * i + 1];
		prim[3 + i] = 1;
		prim[6 + i] = (1 - p[2 * i] - p[2 * i + 1]) / p[2 * i + 1];
	}
	w[0] = p[6] / p[7];
	w[1] = 1;
	w[2] = (1 - p[6] - p[7]) / p[7];
	inv33(prim, inv);
	for (i = 0; i < 3; i++)
		s[i] = inv[i * 3] * w[0] + inv[i * 3 + 1] * w[1] + inv[i * 3 + 2] * w[2];
	for (i = 0; i < 9; i++)
		m[i] = prim[i] * s[i % 3];
}

/* RGB -> LMS with crosstalk @c (X(c) = (1 - 3c) I + c J) for primaries @p. */
static void rgb2lms_ct(const double *p, double c, double *m)
{
	double x[9], ct[9];
	int i;

	for (i = 0; i < 9; i++)
		ct[i] = i % 4 ? c : 1 - 2 * c;
	rgb2xyz(p, x);
	mul33(dm_xyz2lms, x, m);
	mul33(ct, m, m);
}

/* RGB -> DM4's LMS for primaries @p. */
static void rgb2lms(const double *p, double *m)
{
	rgb2lms_ct(p, DM_CROSSTALK, m);
}

/*
 * How far a display's colour volume is from the reference: its red primary
 * at peak in ICtCp, |peak PQ - I| + |Ct| + |Cp|.
 */
static double volume_key(const double *p, double nits, double pq)
{
	double x[9], lms[3], ictcp[3], m[9];
	int i;

	rgb2lms(p, m);
	for (i = 0; i < 3; i++)
		lms[i] = nits_to_pq(dm_max(m[i * 3] * nits, 0));
	for (i = 0; i < 3; i++)
		ictcp[i] = dm_lms2ictcp[i * 3] * lms[0] + dm_lms2ictcp[i * 3 + 1] * lms[1] +
			   dm_lms2ictcp[i * 3 + 2] * lms[2];
	(void)x;
	return (pq > ictcp[0] ? pq - ictcp[0] : ictcp[0] - pq) +
	       (ictcp[1] < 0 ? -ictcp[1] : ictcp[1]) + (ictcp[2] < 0 ? -ictcp[2] : ictcp[2]);
}

/* ---- trims --------------------------------------------------------------- */

/* L8 fields, raw (12-bit, 2048 neutral; vectors 8-bit, 128 neutral). */
enum { T_SLOPE, T_OFFSET, T_POWER, T_CHROMA, T_SAT, T_MS, T_MC, T_CLIP,
       T_SATV, T_HUEV = T_SATV + 6, T_N = T_HUEV + 6 };

struct trim_set {
	s16	v[T_N];		/* raw: 12-bit, the vectors 8-bit */
	double	key;		/* the pass's sort key */
	bool	identity;	/* the mastering display's: no trims */
};

static void trim_neutral(struct trim_set *s)
{
	int i;

	for (i = 0; i < T_N; i++)
		s->v[i] = i < T_SATV ? 2048 : 128;
	s->identity = false;
}

/* Pass A interpolates by peak, pass B (colour) by colour volume. */
static const u8 dm_pass_a[] = { T_SLOPE, T_OFFSET, T_POWER, T_MS, T_MC, T_CLIP };

static bool in_pass_a(int f)
{
	unsigned int i;

	for (i = 0; i < sizeof(dm_pass_a); i++)
		if (dm_pass_a[i] == f)
			return true;
	return false;
}

/* A target display of the stream: by L8 index, Dolby's table or L10. */
static bool l8_display(const struct lg1314_dovi_ext *x, u8 index, double *pq,
		       double *nits, double prim[8])
{
	unsigned int i, k;

	for (i = 0; i < sizeof(dm_displays) / sizeof(dm_displays[0]); i++) {
		if (dm_displays[i].index != index)
			continue;
		*nits = dm_displays[i].nits;
		*pq = nits_to_pq(*nits);
		for (k = 0; k < 8; k++)
			prim[k] = dm_prims[dm_displays[i].prim][k];
		return true;
	}
	for (i = 0; i < x->num_l10; i++) {
		if (x->l10[i].target_display_index != index)
			continue;
		*pq = x->l10[i].target_max_pq / 4096.0;
		*nits = pq_to_nits(*pq);
		if (x->l10[i].target_primary_index < DM_NPRIMS) {
			for (k = 0; k < 8; k++)
				prim[k] = dm_prims[x->l10[i].target_primary_index][k];
		} else {
			for (k = 0; k < 8; k++)
				prim[k] = (s16)x->l10[i].primaries[k] / 32768.0;
		}
		return true;
	}
	return false;
}

/* Keys a code apart (in Q18) are the same display: keep the first. */
static bool key_dup(const struct trim_set *s, int n, double key)
{
	long k = (long)(key * 262144);
	long a = (k + 0x100) & ~0xffL, b = (k + 0x80) & ~0xffL;
	int i;

	for (i = 0; i < n; i++) {
		long q = (long)(s[i].key * 262144);

		if (((q + 0x100) & ~0xffL) == a || ((q + 0x80) & ~0xffL) == b)
			return true;
	}
	return false;
}

/*
 * The sets bracketing @target (sorted by key; one may be the mastering
 * display's identity): interpolate the fields of the pass into @out.
 * Returns false when the only set to use is the identity.
 */
static bool trims_interp(struct trim_set *s, int n, double target, bool pass_a,
			 double *out)
{
	int i, j, lo, hi;
	double w;

	for (i = 1; i < n; i++) {		/* insertion sort, n is small */
		struct trim_set t = s[i];

		for (j = i; j > 0 && s[j - 1].key > t.key; j--)
			s[j] = s[j - 1];
		s[j] = t;
	}
	if (target < s[0].key) {
		lo = hi = 0;
	} else {
		for (lo = 0; lo + 1 < n && s[lo + 1].key <= target; lo++)
			;
		hi = lo + 1 < n ? lo + 1 : lo;
	}
	if (lo == hi && s[lo].identity)
		return false;
	w = lo == hi ? 0 : (target - s[lo].key) / (s[hi].key - s[lo].key);
	for (i = 0; i < T_N; i++)
		if (in_pass_a(i) == pass_a)
			out[i] = s[lo].v[i] + w * (s[hi].v[i] - s[lo].v[i]);
	return true;
}

static void l8_trims(const struct lg1314_dovi_ext *x, int k, struct trim_set *s)
{
	const typeof(x->l8[0]) *t = &x->l8[k];
	int i;

	trim_neutral(s);
	s->v[T_SLOPE] = t->slope;
	s->v[T_OFFSET] = t->offset;
	s->v[T_POWER] = t->power;
	s->v[T_CHROMA] = t->chroma_weight;
	s->v[T_SAT] = t->saturation_gain;
	s->v[T_MS] = t->ms_weight;
	if (t->len > 10)
		s->v[T_MC] = t->target_mid_contrast;
	if (t->len > 12)
		s->v[T_CLIP] = t->clip_trim;
	if (t->len > 13)
		for (i = 0; i < 6; i++)
			s->v[T_SATV + i] = t->saturation_vector[i];
	if (t->len > 19)
		for (i = 0; i < 6; i++)
			s->v[T_HUEV + i] = t->hue_vector[i];
}

static double clampd(double v, double lo, double hi)
{
	return dm_clamp(v, lo, hi);
}

/*
 * The CM v4.0 trims for the target: the stream's L8 sets (and the mastering
 * display, untrimmed) ordered by peak for slope / offset / power / mid
 * contrast / clip, by colour volume for chroma and saturation, the two
 * bracketing the target interpolated.
 */
static void dm_l8_select(const struct lg1314_dovi_dm *m, const struct dm_target *tg,
			 const double *src_prim, double src_nits, struct dm_trims *out)
{
	const struct lg1314_dovi_ext *x = &m->ext;
	struct trim_set sa[LG1314_DOVI_MAX_L8 + 1], sb[LG1314_DOVI_MAX_L8 + 1];
	double raw[T_N], tprim[8], pq, nits, prim[8];
	int na = 0, nb = 0, k, i;
	bool a, b;

	for (i = 0; i < 8; i++)
		tprim[i] = tg->primaries[i] / 50000.0;
	for (k = 0; k < x->num_l8; k++) {
		if (!l8_display(x, x->l8[k].target_display_index, &pq, &nits, prim))
			continue;
		if (pq * 262144 > 0xfdbf && !key_dup(sa, na, pq)) {
			l8_trims(x, k, &sa[na]);
			sa[na++].key = pq;
		}
		if (pq * 262144 > 0xfdbf) {
			double kb = volume_key(prim, nits, pq);

			if (!key_dup(sb, nb, kb)) {
				l8_trims(x, k, &sb[nb]);
				sb[nb++].key = kb;
			}
		}
	}
	/* the mastering display: no trims */
	pq = m->source_max_pq / 4096.0;
	trim_neutral(&sa[na]);
	sa[na].identity = true;
	sa[na].key = pq;
	if (!key_dup(sa, na, pq))
		na++;
	trim_neutral(&sb[nb]);
	sb[nb].identity = true;
	sb[nb].key = volume_key(src_prim, dm_min(src_nits, 10000), pq);
	if (!key_dup(sb, nb, sb[nb].key))
		nb++;

	for (i = 0; i < T_N; i++)
		raw[i] = i < T_SATV ? 2048 : 128;
	a = trims_interp(sa, na, tg->max_pq / 4096.0, true, raw);
	b = trims_interp(sb, nb,
			 volume_key(tprim, dm_min(pq_to_nits(tg->max_pq / 4096.0), 10000),
				    tg->max_pq / 4096.0), false, raw);
	if (!a)
		for (i = 0; i < T_N; i++)
			if (in_pass_a(i))
				raw[i] = i < T_SATV ? 2048 : 128;
	if (!b)
		for (i = 0; i < T_N; i++)
			if (!in_pass_a(i))
				raw[i] = i < T_SATV ? 2048 : 128;

	out->slope = (s32)clampd(2 * raw[T_SLOPE], 0, 0x2000);
	out->offset = (s32)clampd(2 * (raw[T_OFFSET] - 2048), -0x1000, 0x1000);
	/* stock only lifts black (a positive offset) for streams with L5 */
	if (!(x->levels & BIT(5)) && out->offset > 0)
		out->offset = 0;
	out->power = (s32)clampd(2 * raw[T_POWER], 0, 0x2000);
	out->chroma_weight = (s32)clampd(raw[T_CHROMA] - 2048, -0x1000, 0x1000);
	out->saturation_gain = (s32)clampd(raw[T_SAT] + 2048, 0x800, 0x1800);
	out->ms_weight = (s32)(2 * raw[T_MS]);
	out->mid_contrast = (s32)clampd(raw[T_MC] - 2048, -0x800, 0x800);
	out->clip = (s32)(raw[T_CLIP] - 2048);
	for (i = 0; i < 6; i++) {
		out->saturation_vector[i] = (s32)(raw[T_SATV + i] - 128);
		out->hue_vector[i] = (s32)(raw[T_HUEV + i] - 128);
	}
}

static void m33_s16(const double *m, s16 *out, u8 *frac)
{
	double max = 0;
	int f, i;

	for (i = 0; i < 9; i++)
		max = dm_max(max, m[i] < 0 ? -m[i] : m[i]);
	for (f = 15; f > 0 && max * (1 << f) + 0.5 > 32767; f--)
		;
	for (i = 0; i < 9; i++)
		out[i] = (s16)(m[i] * (1 << f) + (m[i] < 0 ? -0.5 : 0.5));
	*frac = f;
}

void dm_tone_curve_in(const struct lg1314_dovi_dm *m, const struct dm_target *t,
		      struct dm_tc_in *in, struct dm_trims *trims)
{
	const struct lg1314_dovi_ext *x = &m->ext;
	double src_prim[8], tprim[8], mm[9];
	int smin, smid, smax, i;

	/* the source: L9's primaries (BT.2020 without), the mastering peak */
	for (i = 0; i < 8; i++)
		src_prim[i] = dm_prims[2][i];
	if ((x->levels & BIT(9)) && x->l9.source_primary_index < DM_NPRIMS)
		for (i = 0; i < 8; i++)
			src_prim[i] = dm_prims[x->l9.source_primary_index][i];
	else if ((x->levels & BIT(9)) && x->l9.len > 1)
		for (i = 0; i < 8; i++)
			src_prim[i] = (s16)x->l9.primaries[i] / 32768.0;
	dm_l8_select(m, t, src_prim, pq_to_nits(m->source_max_pq / 4096.0), trims);

	/* the scene: L1, shifted by L3 */
	smin = x->l1.min_pq;
	smax = x->l1.max_pq;
	smid = x->l1.avg_pq;
	if (x->levels & BIT(3)) {
		smin = smin + 2 * (x->l3.min_pq_offset - 2048);
		smax = smax + 2 * (x->l3.max_pq_offset - 2048);
		smid = smid + 2 * (x->l3.avg_pq_offset - 2048);
		smin = smin < 0 ? 0 : smin;
		smax = smax < 0 ? 0 : smax;
		smid = smid < 0 ? 0 : smid;
	}
	smid = smid < 1 ? 1 : smid > 0xfff ? 0xfff : smid;
	if (smid <= smin)
		smin = smid - 1;
	smax += trims->clip;
	if (smid < smax)
		smax = smax > 0x1000 ? 0x1000 : smax;
	else
		smax = smid + 1;

	in->src_min = smin;
	in->src_mid = smid;
	in->src_max = smax;
	in->tgt_min = t->min_pq;
	in->tgt_max = t->max_pq;
	in->slope = trims->slope;
	in->offset = trims->offset;
	in->power = trims->power;
	in->mid_contrast = trims->mid_contrast;
	in->saturation_gain = trims->saturation_gain;
	in->chroma_weight = trims->chroma_weight;
	rgb2lms(src_prim, mm);
	m33_s16(mm, in->src_rgb2lms, &in->src_rgb2lms_frac);
	for (i = 0; i < 8; i++)
		tprim[i] = t->primaries[i] / 50000.0;
	rgb2lms(tprim, mm);
	m33_s16(mm, in->tgt_rgb2lms, &in->tgt_rgb2lms_frac);
	for (i = 0; i < 9; i++)
		in->lms2ictcp[i] = (s16)(dm_lms2ictcp[i] * 4096);
	in->lms2ictcp_frac = 12;
}

/* ---- 3D LUT -------------------------------------------------------------- */

/* The curve's input for output @y: the lowest x it maps to y or above. */
static double tc_inverse(const struct tc *t, double y)
{
	double lo = 0, hi = 1;
	int i;

	if (tc_eval(t, 1) <= y)
		return 1;
	for (i = 0; i < 40; i++) {
		double m = (lo + hi) / 2;

		if (tc_eval(t, m) < y)
			lo = m;
		else
			hi = m;
	}
	return hi;
}

/*
 * The saturation tracking intensity and chroma: a colour of chroma @c moves
 * by 1 + s (c / the source red primary's chroma)^3 (s < 0, at most down to
 * 1 + s), and by the chroma weight trim.
 */
static double chroma_term(const struct tc_sat *sc, double s, double c, double k)
{
	double r = c / sc->chroma_src;

	return dm_clamp(1 + s * r * r * r, 1 + s, 1) * (1 + k * sc->cw * c);
}

static int inv_m33(const s16 *m, unsigned int frac, double out[9])
{
	double d[9];

	m33(m, frac, d);
	return inv33(d, out);
}

void dm_lut_slice(const struct dm_tc_in *in, const struct dm_output *out,
		  unsigned int k, u16 lut[DM_LUT_DIM][DM_LUT_DIM][3])
{
	double ict2lms[9], lms2rgb[9], ycc[9], off[3];
	double I, s, lmin, lmax, gamma_inv, span;
	struct tc_sat sc;
	struct tc t;
	int a, b, i;

	tc_setup(in, &t, &sc);
	inv_m33(in->lms2ictcp, in->lms2ictcp_frac, ict2lms);
	inv_m33(in->tgt_rgb2lms, in->tgt_rgb2lms_frac, lms2rgb);
	for (i = 0; i < 9; i++)
		ycc[i] = out->rgb2ycc[i] / 65536.0;
	for (i = 0; i < 3; i++)
		off[i] = out->ycc_offset[i] / 65536.0;
	gamma_inv = 4096.0 / out->gamma;
	lmin = pq_to_nits(t.tmin);
	lmax = pq_to_nits(t.tmax);
	span = lmax - lmin;

	/* the node's intensity (the tone curve's output), its saturation
	 * scale from where the curve took it */
	I = t.tmin + k * (t.tmax - t.tmin) / (DM_LUT_DIM - 1);
	s = sat_scale(&t, sc.smin, I, tc_inverse(&t, I), sc.gain);

	for (a = 0; a < DM_LUT_DIM; a++) {		/* Cp */
		for (b = 0; b < DM_LUT_DIM; b++) {	/* Ct */
			double ct = -0.5 + (double)b / (DM_LUT_DIM - 1);
			double cp = -0.5 + (double)a / (DM_LUT_DIM - 1);
			double c = dm_sqrt(ct * ct + cp * cp);
			double i2 = I * chroma_term(&sc, sc.sat_i, c, 4);
			double f = s * chroma_term(&sc, sc.sat_c, c, 3);
			double ictcp[3] = { i2, ct * f, cp * f }, lms[3], v[3];

			for (i = 0; i < 3; i++) {
				double x = ict2lms[i * 3] * ictcp[0] + ict2lms[i * 3 + 1] * ictcp[1] +
					   ict2lms[i * 3 + 2] * ictcp[2];

				lms[i] = pq_to_nits(dm_clamp(x, 0, 1));
			}
			/* the panel's RGB, clipped to its range, gamma encoded */
			for (i = 0; i < 3; i++) {
				double l = lms2rgb[i * 3] * lms[0] + lms2rgb[i * 3 + 1] * lms[1] +
					   lms2rgb[i * 3 + 2] * lms[2];

				v[i] = dm_pow(dm_clamp((l - lmin) / span, 0, 1), gamma_inv);
			}
			for (i = 0; i < 3; i++) {
				double y = off[i] + ycc[i * 3] * v[0] + ycc[i * 3 + 1] * v[1] +
					   ycc[i * 3 + 2] * v[2];
				long q = (long)(y * 65536 + 0.5);

				lut[a][b][i] = q < 0 ? 0 : q > 65535 ? 65535 : q;
			}
		}
	}
}

/* ---- the record ---------------------------------------------------------- */

/* 16-bit to the record's 12, rounded. */
static u32 rec12(u32 v)
{
	v = (v + 8) >> 4;
	return v > 4095 ? 4095 : v;
}

void dm_rec_regs(const struct dm_target *t, bool dm3, s32 ms_weight,
		 const struct lg1314_dovi_ext *x, u32 regs[DM_REC_REGS_WORDS])
{
	u32 lo = dm3 ? t->dm3_lut_min : t->min_pq << 3;	/* Q15 */
	u32 hi = dm3 ? t->dm3_lut_max : t->max_pq << 3;
	u32 ms = ms_weight < 0 ? 0 : ms_weight > 0xffff ? 0xffff : ms_weight;
	int i;

	for (i = 0; i < DM_REC_REGS_WORDS; i++)
		regs[i] = 0;
	regs[2] = 0xc;
	regs[3] = ms << 16 | 0x3ffc;		/* the multi-scale weight trim */
	regs[4] = 0x1fff;
	regs[5] = 0xfff;
	/* the LUT's I range, and Ct / Cp: -0.5..0.5 */
	regs[6] = 0xc000u << 16 | lo;
	regs[7] = 0xffffc000;
	regs[8] = 0x4000u << 16 | hi;
	regs[9] = hi > lo ? BIT(30) / (hi - lo) : 0;
	regs[10] = 0x8000;
	regs[11] = 0x8000;
	/* the active area (L5): top << 12 | left, bottom << 12 | right */
	if (x->levels & BIT(5)) {
		regs[13] = (x->l5.top & 0xfff) << 12 | (x->l5.left & 0xfff);
		regs[14] = (x->l5.bottom & 0xfff) << 12 | (x->l5.right & 0xfff);
	}
}

void dm_rec_curve(const struct dm_tc_in *in, u32 words[DM_REC_CURVE_WORDS])
{
	u16 c[DM_TC_SAMPLES];
	int i;

	dm_tone_curve(in, c, NULL);
	for (i = 0; i < DM_REC_CURVE_WORDS; i++)
		words[i] = c[2 * i] | (u32)c[2 * i + 1] << 12;
}

static void rec_lut_pack(unsigned int k, u16 scratch[DM_LUT_DIM][DM_LUT_DIM][3],
			 u32 lut[DM_REC_LUT_WORDS]);

void dm_rec_lut_slice(const struct dm_tc_in *in, const struct dm_output *out,
		      unsigned int k, u16 scratch[DM_LUT_DIM][DM_LUT_DIM][3],
		      u32 lut[DM_REC_LUT_WORDS])
{
	dm_lut_slice(in, out, k, scratch);
	rec_lut_pack(k, scratch, lut);
}

/* Layer @k of a LUT (lut[a][b][Y, Cb, Cr]) into the record's form. */
static void rec_lut_pack(unsigned int k, u16 scratch[DM_LUT_DIM][DM_LUT_DIM][3],
			 u32 lut[DM_REC_LUT_WORDS])
{
	unsigned int a, b, i;

	for (a = 0; a < DM_LUT_DIM; a++) {
		for (b = 0; b < DM_LUT_DIM; b++) {
			unsigned int n = (a * DM_LUT_DIM + b) * DM_LUT_DIM + k;
			unsigned int bit = n % 3 * 36;
			u32 *g = &lut[n / 3 * 4];

			for (i = 0; i < 3; i++, bit += 12) {
				u32 v = rec12(scratch[a][b][i]);

				g[bit / 32] |= v << (bit % 32);
				if (bit % 32 > 20)
					g[bit / 32 + 1] |= v >> (32 - bit % 32);
			}
		}
	}
}

/* ---- DM3 (CM v2.9) --------------------------------------------------------- */

/*
 * DM3 maps in IPT-PQ: a rational tone curve through three anchors, L2's
 * trims, and a 3D LUT scaling intensity and chroma with the curve.
 */

static const double dm_lms2ipt[9] = {
	0.4,     0.4,     0.2,
	4.455,  -4.851,   0.396,
	0.8056,  0.3572, -1.1628,
};

/* The curve's limits on the scene (PQ, Q31 in stock) */
#define DM3_SMAX_MIN	(0x4108b6b6 / 2147483648.0)	/* 100 cd/m² */
#define DM3_SMIN_MAX	(0x13316bf8 / 2147483648.0)
#define DM3_MID_GAP	(0x15e00000 / 2147483648.0)	/* mid above min */
#define DM3_TOP_GAP	(0x80000 / 2147483648.0)	/* mid below max */
#define DM3_Q15_MAX	(0x7fff / 32768.0)		/* stock's LUT maths: Q15 */

/* L2's trims for the target: raw - 2048, ms weight raw (2048 neutral). */
enum { T3_SLOPE, T3_OFFSET, T3_POWER, T3_CHROMA, T3_SAT, T3_MS, T3_N };

/*
 * The L2 trims for target peak @pq: interpolated in PQ between the L2 sets
 * bracketing it, among those for displays up to the source's peak; the
 * source's peak counts as untrimmed unless an L2 set is for it. Floored.
 */
static void dm3_trims(const struct lg1314_dovi_ext *x, u16 src_max, u16 pq, s32 out[T3_N])
{
	struct { u16 key; s32 v[T3_N]; } p[LG1314_DOVI_MAX_L2 + 1], tmp;
	bool at_src = false;
	int n = 0, i, j, lo;
	double w;

	for (i = 0; i < x->num_l2; i++) {
		const typeof(x->l2[0]) *t = &x->l2[i];

		if (t->target_max_pq > src_max)
			continue;
		at_src |= t->target_max_pq == src_max;
		p[n].key = t->target_max_pq;
		p[n].v[T3_SLOPE] = t->slope - 2048;
		p[n].v[T3_OFFSET] = t->offset - 2048;
		p[n].v[T3_POWER] = t->power - 2048;
		p[n].v[T3_CHROMA] = t->chroma_weight - 2048;
		p[n].v[T3_SAT] = t->saturation_gain - 2048;
		p[n].v[T3_MS] = t->ms_weight;
		n++;
	}
	if (!at_src) {
		p[n].key = src_max;
		for (j = 0; j < T3_N; j++)
			p[n].v[j] = j == T3_MS ? 2048 : 0;
		n++;
	}
	for (i = 1; i < n; i++)
		for (j = i; j > 0 && p[j - 1].key > p[j].key; j--) {
			tmp = p[j];
			p[j] = p[j - 1];
			p[j - 1] = tmp;
		}
	if (pq <= p[0].key || n == 1) {
		for (j = 0; j < T3_N; j++)
			out[j] = p[0].v[j];
		return;
	}
	if (pq >= p[n - 1].key) {
		for (j = 0; j < T3_N; j++)
			out[j] = p[n - 1].v[j];
		return;
	}
	for (lo = 0; p[lo + 1].key < pq; lo++)
		;
	w = (double)(pq - p[lo].key) / (p[lo + 1].key - p[lo].key);
	for (j = 0; j < T3_N; j++) {
		double v = p[lo].v[j] + w * (p[lo + 1].v[j] - p[lo].v[j]);
		s32 f = (s32)v;

		out[j] = f > v ? f - 1 : f;		/* floor */
	}
}

void dm3_scene(const struct lg1314_dovi_dm *m, const struct dm_target *t,
	       struct dm3_in *in, s32 *ms_weight)
{
	const struct lg1314_dovi_ext *x = &m->ext;
	s32 tr[T3_N];
	int i;

	in->tmin = t->dm3_min_pq;
	in->tmax = t->dm3_max_pq;
	in->lut_min = t->dm3_lut_min;
	in->lut_max = t->dm3_lut_max;
	for (i = 0; i < 8; i++)
		in->primaries[i] = t->primaries[i];
	in->l1_min = x->l1.min_pq;
	in->l1_max = x->l1.max_pq;
	in->l1_avg = x->l1.avg_pq;
	in->src_min = m->source_min_pq;
	in->src_max = m->source_max_pq;
	dm3_trims(x, m->source_max_pq, t->dm3_max_pq, tr);
	in->slope = tr[T3_SLOPE];
	/* stock only lifts black (a positive offset) for streams with L5 */
	in->offset = !(x->levels & BIT(5)) && tr[T3_OFFSET] > 0 ? 0 : tr[T3_OFFSET];
	in->power = tr[T3_POWER];
	in->chroma_weight = tr[T3_CHROMA];
	in->saturation_gain = tr[T3_SAT];
	*ms_weight = 2 * tr[T3_MS];
}

/* The curve and what the LUT needs of it. */
struct tc3 {
	double	tmin, tmax;
	bool	expand;		/* the source fits in the target: inverse curve */
	double	d;		/* mid-range difference (source - target, or the
				 * reverse when expanding) */
	double	n, c[3];	/* y = ((c0 + c1 x^n) / (1 + c2 x^n))^(1/3) */
	double	slope, offset, power;
};

static void tc3_setup(const struct dm3_in *in, struct tc3 *t)
{
	double lmin = in->l1_min * E12, lmax = in->l1_max * E12, lavg = in->l1_avg * E12;
	double srcmin = in->src_min * E12, srcmax = in->src_max * E12;
	double lo, hi, ref_lo, ref_hi, smin, smid, smax, refmid, shift;
	double x[3], y[3], a[9], ia[9], b[3];
	int i;

	t->tmin = in->tmin * E12;
	t->tmax = in->tmax * E12;
	t->expand = in->src_max < in->tmax;
	if (t->expand) {
		/* map the target's range onto the source's, and invert */
		lmin = t->tmin;
		lavg = (t->tmin + t->tmax) / 2;
		lmax = t->tmax;
		lo = srcmin;
		hi = srcmax;
		ref_lo = t->tmin;
		ref_hi = t->tmax;
	} else {
		lo = t->tmin;
		hi = t->tmax;
		ref_lo = srcmin;
		ref_hi = srcmax;
	}
	smax = dm_max(lmax, DM3_SMAX_MIN);
	smin = dm_min(lmin, DM3_SMIN_MAX);
	smid = dm_max(lavg, smin + DM3_MID_GAP);
	if (smid > smax - DM3_TOP_GAP)
		smid = smax - DM3_TOP_GAP;
	refmid = (ref_lo + ref_hi) / 2;
	t->d = refmid - (lo + hi) / 2;
	shift = t->d * (0.5 + 1.5 * (smid - refmid));

	x[0] = smin;
	x[2] = smax;
	x[1] = dm_clamp(smid, smin + 0.15 * (smax - smin), smin + 0.85 * (smax - smin));
	y[0] = dm_max(lo, smin - shift);
	y[2] = dm_min(hi, smax - shift);
	y[1] = dm_clamp(x[1] - shift, y[0] + 0.15 * (y[2] - y[0]), y[0] + 0.85 * (y[2] - y[0]));

	/* through the anchors: c0 + c1 X - c2 X Y = Y, X = x^n, Y = y^3 */
	t->n = 3 * (1 + t->d);
	for (i = 0; i < 3; i++) {
		double xn = dm_pow(x[i], t->n), y3 = y[i] * y[i] * y[i];

		a[i * 3] = 1;
		a[i * 3 + 1] = xn;
		a[i * 3 + 2] = -xn * y3;
		b[i] = y3;
	}
	inv33(a, ia);
	for (i = 0; i < 3; i++)
		t->c[i] = ia[i * 3] * b[0] + ia[i * 3 + 1] * b[1] + ia[i * 3 + 2] * b[2];
	t->slope = in->slope / 4096.0;
	t->offset = in->offset / 4096.0;
	t->power = in->power / 4096.0;
}

static double tc3_eval(const struct tc3 *t, double x)
{
	double y;

	if (!t->expand) {
		double xn = dm_pow(x, t->n), r = (t->c[0] + t->c[1] * xn) / (1 + t->c[2] * xn);

		y = r > 0 ? dm_pow(r, 1 / 3.0) : 0;
		y = y * (1 + t->slope) + t->offset;
		y = y > 0 ? dm_pow(y, 1 + t->power) : 0;
	} else {
		double yy = (dm_pow(x, 1 / (1 + t->power)) - t->offset) / (1 + t->slope);
		double y3 = yy > 0 ? yy * yy * yy : 0;
		double r = (y3 - t->c[0]) / (t->c[1] - t->c[2] * y3);

		y = r > 0 ? dm_pow(r, 1 / t->n) : 0;
	}
	return y;
}

/* The curve's 512 samples (12-bit): rounded, within the target, rising. */
static void tc3_samples(const struct tc3 *t, const struct dm3_in *in, u16 c[DM_TC_SAMPLES])
{
	int i, v, prev = 0;

	for (i = 0; i < DM_TC_SAMPLES; i++) {
		v = (int)(tc3_eval(t, i * 8 * E12) * 4096 + 0.5);
		v = v < in->tmin ? in->tmin : v > in->tmax ? in->tmax : v;
		prev = v > prev ? v : prev;
		c[i] = prev;
	}
}

void dm3_tone_curve(const struct dm3_in *in, u16 curve[DM_TC_SAMPLES])
{
	struct tc3 t;

	tc3_setup(in, &t);
	tc3_samples(&t, in, curve);
}

/*
 * The source intensity the curve maps to @v (PQ): within its rising part;
 * above its top, where it first reaches it; below its bottom, where it last
 * leaves it.
 */
static double tc3_inverse(const u16 c[DM_TC_SAMPLES], double v)
{
	int lo = 0, hi = DM_TC_SAMPLES - 1, j;

	v *= 4096;
	while (lo + 1 < DM_TC_SAMPLES && c[lo + 1] == c[0])
		lo++;
	while (hi > 0 && c[hi - 1] == c[DM_TC_SAMPLES - 1])
		hi--;
	if (v <= c[lo])
		return lo * 8 * E12;
	if (v >= c[hi])
		return hi * 8 * E12;
	for (j = lo; j + 1 < hi && c[j + 1] <= v; j++)
		;
	return (j + (c[j + 1] > c[j] ? (v - c[j]) / (c[j + 1] - c[j]) : 0)) * 8 * E12;
}

void dm3_lut_slice(const struct dm3_in *in, const struct dm_output *out,
		   unsigned int k, u16 lut[DM_LUT_DIM][DM_LUT_DIM][3])
{
	double ipt2lms[9], lms2rgb[9], tprim[8], m[9], ycc[9], off[3];
	double I, s, g, ai, ac, sgn, gain, lmin, lmax, span, gamma_inv, xsrc;
	u16 curve[DM_TC_SAMPLES];
	struct tc3 t;
	int a, b, i;

	tc3_setup(in, &t);
	tc3_samples(&t, in, curve);
	inv33(dm_lms2ipt, ipt2lms);
	for (i = 0; i < 8; i++)
		tprim[i] = in->primaries[i] / 50000.0;
	rgb2lms_ct(tprim, DM3_CROSSTALK, m);
	inv33(m, lms2rgb);
	for (i = 0; i < 9; i++)
		ycc[i] = out->rgb2ycc[i] / 65536.0;
	for (i = 0; i < 3; i++)
		off[i] = out->ycc_offset[i] / 65536.0;
	gamma_inv = 4096.0 / out->gamma;
	lmin = pq_to_nits(t.tmin);
	lmax = pq_to_nits(t.tmax);
	span = lmax - lmin;

	/* the layer's intensity, its saturation from where the curve took it */
	I = (in->lut_min + k * (double)(in->lut_max - in->lut_min) / (DM_LUT_DIM - 1)) / 32768;
	gain = 1 + in->saturation_gain / 4096.0;
	xsrc = tc3_inverse(curve, I);
	s = t.expand ? gain / (1 + xsrc - I) : gain * (1 + I - xsrc);
	s = dm_clamp(s, 0, 2);
	/* intensity and chroma lose (gain, expanding) with chroma squared */
	g = t.d - 5.0 / 3 * in->chroma_weight / 4096;
	sgn = t.expand ? 8 : -8;
	ai = sgn * (t.expand ? 2.5 : 0.75) * g;
	ac = sgn * 0.75 * g;

	for (a = 0; a < DM_LUT_DIM; a++) {		/* T */
		for (b = 0; b < DM_LUT_DIM; b++) {	/* P */
			double p = -0.5 + (double)b / (DM_LUT_DIM - 1);
			double tt = -0.5 + (double)a / (DM_LUT_DIM - 1);
			double c2 = dm_min(p * p + tt * tt, 1);
			double fi = dm_clamp(1 + ai * c2, 0, DM3_Q15_MAX * 2);
			double fc = s * dm_clamp(1 + ac * c2, 0, DM3_Q15_MAX * 2);
			/* Q15 in stock: I below 1, P and T within +-1 */
			double ipt[3] = { dm_min(I * fi, DM3_Q15_MAX),
					  dm_clamp(p * fc, -1, DM3_Q15_MAX),
					  dm_clamp(tt * fc, -1, DM3_Q15_MAX) }, lms[3], v[3];

			for (i = 0; i < 3; i++)
				lms[i] = pq_to_nits(dm_clamp(ipt2lms[i * 3] * ipt[0] +
							     ipt2lms[i * 3 + 1] * ipt[1] +
							     ipt2lms[i * 3 + 2] * ipt[2], 0, 1));
			for (i = 0; i < 3; i++) {
				double l = lms2rgb[i * 3] * lms[0] + lms2rgb[i * 3 + 1] * lms[1] +
					   lms2rgb[i * 3 + 2] * lms[2];

				v[i] = dm_pow(dm_clamp((l - lmin) / span, 0, 1), gamma_inv);
			}
			for (i = 0; i < 3; i++) {
				double y = off[i] + ycc[i * 3] * v[0] + ycc[i * 3 + 1] * v[1] +
					   ycc[i * 3 + 2] * v[2];
				long q = (long)(y * 65536 + 0.5);

				lut[a][b][i] = q < 0 ? 0 : q > 65535 ? 65535 : q;
			}
		}
	}
}

void dm3_rec_curve(const struct dm3_in *in, u32 words[DM_REC_CURVE_WORDS])
{
	u16 c[DM_TC_SAMPLES];
	int i;

	dm3_tone_curve(in, c);
	for (i = 0; i < DM_REC_CURVE_WORDS; i++)
		words[i] = c[2 * i] | (u32)c[2 * i + 1] << 12;
}

void dm3_rec_lut_slice(const struct dm3_in *in, const struct dm_output *out,
		       unsigned int k, u16 scratch[DM_LUT_DIM][DM_LUT_DIM][3],
		       u32 lut[DM_REC_LUT_WORDS])
{
	dm3_lut_slice(in, out, k, scratch);
	rec_lut_pack(k, scratch, lut);
}

u32 dm_pq_lum(u16 pq)
{
	return (u32)(pq_to_nits(pq * E12) * 10000 + 0.5);
}
