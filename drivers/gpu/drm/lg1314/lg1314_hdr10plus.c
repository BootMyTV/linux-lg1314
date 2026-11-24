// SPDX-License-Identifier: GPL-2.0
/*
 * HDR10+ tone mapping: a scene's SMPTE ST 2094-40 metadata to the HDR block's
 * tone curve, in floating point (built with the FPU enabled).
 *
 * The reference receiver method of ATSC A/341 (amendment for ST 2094-40,
 * A.4.3): the metadata's curve (the "basis OOTF", a knee point and Bezier
 * anchors, ST 2094-40 8.7) is made for a target display of T nits; for the
 * panel's D nits a "guided" curve is mixed from it:
 *   - input: maxRGB / Norm, clipped to 1, with Norm = max(D, HM), HM the
 *     scene's 99th maxRGB percentile (distribution_values[8]); output times D
 *     (a scene within the panel's range is shown as it is);
 *   - D >= T: knee and Bezier move towards the identity line, linearly from
 *     the basis curve at D = T to identity at D = Norm;
 *   - D < T: towards a clip, linearly from the basis curve at D = T (from 0,
 *     the document's D_L);
 *   - then the first anchor is set for a continuous slope at the knee.
 * No black point compensation (neither document has it). Metadata without a
 * curve gets a knee picked from the scene's average and a matching Bezier,
 * as libplacebo does (pl_tone_map_st2094_40, st2094_pick_knee), for D.
 *
 * The tone curve works in gamma 2.2: x = (input / 10000 cd/m²)^(1/2.2), the
 * output (16 bits) = (output / panel peak)^(1/2.2). It is applied to R, G and
 * B alike.
 */
#include "lg1314_fpmath.h"
#include "lg1314_hdr10plus.h"

/* libplacebo's defaults (PL_TONE_MAP_CONSTANTS) for the knee without a curve */
#define KNEE_ADAPTATION	0.3
#define KNEE_MINIMUM	0.1
#define KNEE_MAXIMUM	0.8
#define KNEE_DEFAULT	0.4

static double nits_to_pq(double nits)
{
	double y = dm_pow(dm_max(nits, 0) / 10000, PQ_M1);

	return dm_pow((PQ_C1 + PQ_C2 * y) / (1 + PQ_C3 * y), PQ_M2);
}

static double mix(double a, double b, double t)
{
	return a + (b - a) * t;
}

static double smoothstep(double e0, double e1, double x)
{
	double t = dm_clamp((x - e0) / (e1 - e0), 0, 1);

	return t * t * (3 - 2 * t);
}

/* libplacebo's st2094_pick_knee (no black level): in nits, mixed in PQ */
static void pick_knee(double in_max, double in_avg, double out_max,
		      double *src_knee_out, double *dst_knee_out)
{
	double src_min = nits_to_pq(0), src_max = nits_to_pq(in_max);
	double src_avg = in_avg ? nits_to_pq(in_avg) : 0;
	double dst_min = nits_to_pq(0), dst_max = nits_to_pq(out_max);
	double src_knee_min = mix(src_min, src_max, KNEE_MINIMUM);
	double src_knee_max = mix(src_min, src_max, KNEE_MAXIMUM);
	double dst_knee_min = mix(dst_min, dst_max, KNEE_MINIMUM);
	double dst_knee_max = mix(dst_min, dst_max, KNEE_MAXIMUM);
	double src_knee, target, adapted, adaptation, dst_knee;

	src_knee = src_avg ? src_avg : mix(src_min, src_max, KNEE_DEFAULT);
	src_knee = dm_clamp(src_knee, src_knee_min, src_knee_max);
	target = (src_knee - src_min) / (src_max - src_min);
	adapted = mix(dst_min, dst_max, target);
	adaptation = mix(1.0, KNEE_ADAPTATION, smoothstep(dst_min, dst_knee_min, src_knee));
	dst_knee = mix(dm_min(src_knee, dst_knee_max), adapted, adaptation);
	dst_knee = dm_max(dst_knee, dst_knee_min);
	*src_knee_out = pq_to_nits(src_knee);
	*dst_knee_out = pq_to_nits(dst_knee);
}

/* x^k, with 0^0 = 1 */
static double ipow(double x, int k)
{
	double r = 1;

	while (k-- > 0)
		r *= x;
	return r;
}

static double binom(int n, int k)
{
	double r = 1;
	int i;

	for (i = 1; i <= k; i++)
		r = r * (n - k + i) / i;
	return r;
}

/* ST 2094-40 equation (1) */
static double bezier(const double *P, int n, double t)
{
	double b = 0;
	int p;

	for (p = 0; p <= n; p++)
		b += binom(n, p) * ipow(t, p) * ipow(1 - t, n - p) * P[p];
	return b;
}

/* ST 2094-40 equations (3) to (5) */
static double ootf(double s, double kx, double ky, const double *P, int n)
{
	if (kx <= 0)
		return ky + (1 - ky) * bezier(P, n, s);
	if (kx >= 1)
		return ky * s;
	if (s < kx)
		return ky / kx * s;
	return ky + (1 - ky) * bezier(P, n, (s - kx) / (1 - kx));
}

void lg1314_hdr10plus_curve(const struct lg1314_hdr10plus_in *in, u32 *y)
{
	const struct lg1314_hdr10plus *m = &in->md;
	double D = in->panel_nits, T, norm, maxscl, kx, ky, P[17];
	int n, p, i;

	maxscl = dm_max(dm_max(m->maxscl[0], m->maxscl[1]), m->maxscl[2]) / 10.0;
	norm = m->hm ? m->hm / 10.0 : maxscl ? maxscl : in->static_nits;
	norm = dm_max(D, dm_min(norm, 10000));

	if (m->num_anchors) {
		double w;

		kx = m->knee_x / 4095.0;
		ky = m->knee_y / 4095.0;
		n = m->num_anchors + 1;
		P[0] = 0;
		for (p = 1; p < n; p++)
			P[p] = m->anchors[p - 1] / 1023.0;
		P[n] = 1;
		T = dm_max(m->target_nits, 1);

		if (D >= T) {
			/* towards identity: knee (0.5, 0.5), anchors p / N */
			w = norm > T ? dm_clamp((norm - D) / (norm - T), 0, 1) : 0;
			kx = mix(0.5, kx, w);
			ky = mix(0.5, ky, w);
			for (p = 1; p < n; p++)
				P[p] = mix((double)p / n, P[p], w);
		} else {
			/* towards a clip: knee (0, 0), anchors 1 */
			w = dm_clamp(D / T, 0, 1);
			kx *= w;
			ky *= w;
			for (p = 1; p < n; p++)
				P[p] = mix(1, P[p], w);
		}
		/* slope continuity at the knee (A.4.3.2.4) */
		if (kx > 0 && kx < 1 && ky < 1)
			P[1] = dm_clamp(ky / kx * (1 - kx) / (1 - ky) / n, 0, 1);
	} else {
		/* no curve: a knee from the scene's average, for D directly */
		double in_avg = maxscl ? m->avg_maxrgb / 10.0 : 0;
		double src_knee, dst_knee, slope;

		pick_knee(norm, in_avg, D, &src_knee, &dst_knee);
		kx = src_knee / norm;
		ky = dst_knee / D;
		slope = ky / kx * (1 - kx) / (1 - ky);
		n = (int)slope;
		n = (int)dm_clamp(n + (slope > n), 2, 16);
		P[0] = 0;
		P[1] = dm_min(slope / n, 1.0);
		for (p = 2; p <= n; p++)
			P[p] = 1;
	}
	kx = dm_clamp(kx, 0, 1);
	ky = dm_clamp(ky, 0, 1);

	for (i = 0; i < in->pts; i++) {
		/* the curve's input: nits from the gamma-2.2 x position */
		double nits = dm_pow(in->x[i] / 4294967295.0, 2.2) * 10000;
		double s = dm_min(nits / norm, 1), v;

		v = dm_clamp(ootf(s, kx, ky, P, n), 0, 1) * D;
		v = dm_pow(v / in->panel_nits, 1 / 2.2) * 65535 + 0.5;
		y[i] = (u32)dm_clamp(v, 0, 65535);
	}
}
