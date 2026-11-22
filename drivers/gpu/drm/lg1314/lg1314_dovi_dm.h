/* SPDX-License-Identifier: GPL-2.0 */
/*
 * LG1314 Dolby Vision display management (the DM4 model): the tone curve, the
 * 3D LUT and the per-scene DM registers the display engine's Dolby block takes.
 *
 * Computed in floating point (lg1314_dovi_dm.c is built with the FPU enabled);
 * nothing in this interface is floating point. Callers bracket calls with
 * kernel_fpu_begin() / kernel_fpu_end().
 */
#ifndef LG1314_DOVI_DM_H
#define LG1314_DOVI_DM_H

#include <linux/types.h>

#define DM_TC_SAMPLES	512

/* Tone curve inputs: PQ values are 12-bit codes, gains Q12. */
struct dm_tc_in {
	u16	src_min, src_mid, src_max;	/* the scene (L1, with L3) */
	u16	tgt_min, tgt_max;		/* the target display */
	s32	slope, offset, power;		/* trims (L2 / L8) */
	s32	mid_contrast;			/* added to the mid slope bound */
	s32	saturation_gain;
	/* RGB to LMS of the source and of the target, LMS' to ICtCp:
	 * s16 with @frac fraction bits */
	s16	src_rgb2lms[9], tgt_rgb2lms[9], lms2ictcp[9];
	u8	src_rgb2lms_frac, tgt_rgb2lms_frac, lms2ictcp_frac;
};

/* The curve's parameters, for comparison (Q26: 1.0 = 1 << 26). */
struct dm_tc_params {
	s32	x[3], y[3], m[3];		/* spline anchors and slopes */
	s32	src_red[3], tgt_red[3];		/* ICtCp of the red primaries at peak */
	s32	chroma_src, sat_i, sat_c;	/* saturation compensation */
};

/* The curve: 512 samples of PQ (12-bit) over the input range. */
void dm_tone_curve(const struct dm_tc_in *in, u16 curve[DM_TC_SAMPLES],
		   struct dm_tc_params *params);

#endif
