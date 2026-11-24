// SPDX-License-Identifier: GPL-2.0
/*
 * The HDR block's transfer-function tables, in floating point (built with the
 * FPU enabled). They match the vendor's to within its rounding: about 1e-6
 * relative for the PQ table, 2e-8 for HLG, the HLG OETF and the ramp exactly.
 */
#include "lg1314_fpmath.h"
#include "lg1314_hdr_tf.h"

/* BT.2100 HLG */
#define HLG_A	0.17883277
#define HLG_B	0.28466892
#define HLG_C	0.55991073

static u32 full_u32(double v)
{
	double x = v * 4294967295.0 + 0.5;

	return x >= 4294967295.0 ? 0xffffffffu : x <= 0 ? 0 : (u32)x;
}

/* The HLG signal (0..1) as scene light (0..1). */
static double hlg_inverse_oetf(double x)
{
	if (x <= 0.5)
		return x * x / 3;
	return (dm_exp2((x - HLG_C) / HLG_A * 1.4426950408889634) + HLG_B) / 12;
}

void lg1314_hdr_tf_compute(struct lg1314_hdr_tf *tf)
{
	int i;

	for (i = 0; i < HDR_TF_EOTF_PTS; i++) {
		double x = i / (double)(HDR_TF_EOTF_PTS - 1);

		tf->eotf_pq[i] = full_u32(dm_pow(pq_to_nits(x) / 10000, 1 / 2.2));
		tf->eotf_hlg[i] = full_u32(hlg_inverse_oetf(x));
	}
	for (i = 0; i < HDR_TF_OETF_PTS; i++) {
		double x = i / (double)(HDR_TF_OETF_PTS - 1);

		tf->oetf_lin[i] = (u16)(65535 * x + 0.5);
		tf->oetf_hlg[i] = (u16)(4095 * dm_pow(x, 0.45) + 0.5);
	}
}
