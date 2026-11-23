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
#include <linux/soc/lg/lg1314-dovi-meta.h>

#define DM_TC_SAMPLES	512
#define DM_LUT_DIM	17

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
	s32	chroma_weight;			/* 0 neutral */
};

/* The curve's parameters, for comparison (Q26: 1.0 = 1 << 26). */
struct dm_tc_params {
	s32	x[3], y[3], m[3];		/* spline anchors and slopes */
	s32	src_red[3], tgt_red[3];		/* ICtCp of the red primaries at peak */
	s32	chroma_src, sat_i, sat_c;	/* saturation compensation */
};

/* The display the DM maps to. */
struct dm_target {
	u16	min_pq, max_pq;			/* 12-bit */
	u16	primaries[8];			/* R, G, B, W xy, 1/50000 */
	/* DM3 rounds the target its own way: its range (12-bit) and its
	 * 3D LUT's I range (Q15) */
	u16	dm3_min_pq, dm3_max_pq;
	u16	dm3_lut_min, dm3_lut_max;
};

/* The trims for a target, in the DM's units (Q12 unless noted). */
struct dm_trims {
	s32	slope, offset, power;		/* slope, power 4096 neutral */
	s32	chroma_weight, saturation_gain;	/* gain 4096 neutral */
	s32	ms_weight, mid_contrast, clip;
	s32	saturation_vector[6], hue_vector[6];	/* 8-bit, signed */
};

/*
 * The tone curve's inputs for a picture: the scene (L1 / L3), the trims for
 * @t (L8 / L10 interpolated, CM v4.0), the source and target colour volumes.
 */
void dm_tone_curve_in(const struct lg1314_dovi_dm *m, const struct dm_target *t,
		      struct dm_tc_in *in, struct dm_trims *trims);

/* The curve: 512 samples of PQ (12-bit) over the input range. */
void dm_tone_curve(const struct dm_tc_in *in, u16 curve[DM_TC_SAMPLES],
		   struct dm_tc_params *params);

/* The panel's signal: its EOTF exponent, RGB -> Y'CbCr. */
struct dm_output {
	u16	gamma;				/* Q12 (2.2: 9011) */
	s32	rgb2ycc[9];			/* Q16 */
	s32	ycc_offset[3];			/* Q16 */
};

/*
 * One intensity layer @k of the 3D LUT: ICtCp (I from the tone curve's
 * output range, Ct and Cp -0.5..0.5) to the panel's Y'CbCr, 16 bits;
 * lut[Cp][Ct][Y, Cb, Cr].
 */
void dm_lut_slice(const struct dm_tc_in *in, const struct dm_output *out,
		  unsigned int k, u16 lut[DM_LUT_DIM][DM_LUT_DIM][3]);

/*
 * The record's DM part (offsets from the record's start):
 *   +0x74..+0xb0	registers: the multi-scale weight trim, the 3D LUT's input
 *			ranges (the target's)
 *   +0xb0		3D LUT: nodes I fastest, then Ct, then Cp; Y, Cb, Cr 12 bits
 *			each, three nodes to four words (low bits first)
 *   +0x6710		tone curve: 512 x 12 bits, two to a word
 *   +0x6b10		degamma (non-PQ signals only)
 */
#define DM_REC_REGS		0x74
#define DM_REC_REGS_WORDS	15
#define DM_REC_LUT		0xb0
#define DM_REC_LUT_WORDS	((DM_LUT_DIM * DM_LUT_DIM * DM_LUT_DIM + 2) / 3 * 4)
#define DM_REC_CURVE		0x6710
#define DM_REC_CURVE_WORDS	(DM_TC_SAMPLES / 2)

/* The registers: for DM3 or DM4, the multi-scale weight trim, the stream's
 * active area (L5). */
void dm_rec_regs(const struct dm_target *t, bool dm3, s32 ms_weight,
		 const struct lg1314_dovi_ext *x, u32 regs[DM_REC_REGS_WORDS]);

/* The curve for @in, packed. */
void dm_rec_curve(const struct dm_tc_in *in, u32 words[DM_REC_CURVE_WORDS]);

/*
 * Layer @k of the 3D LUT, packed into @lut (cleared before the first layer);
 * @scratch: room for the layer's nodes.
 */
void dm_rec_lut_slice(const struct dm_tc_in *in, const struct dm_output *out,
		      unsigned int k, u16 scratch[DM_LUT_DIM][DM_LUT_DIM][3],
		      u32 lut[DM_REC_LUT_WORDS]);

/* DM3 (CM v2.9 metadata): a scene's inputs. */
struct dm3_in {
	u16	tmin, tmax;			/* the target, 12-bit */
	u16	lut_min, lut_max;		/* the LUT's I range, Q15 */
	u16	primaries[8];			/* the target's, 1/50000 */
	u16	l1_min, l1_max, l1_avg;		/* the scene */
	u16	src_min, src_max;		/* the mastering display */
	s16	slope, offset, power;		/* L2 for the target, raw - 2048 */
	s16	chroma_weight, saturation_gain;
};

/* A picture's DM3 inputs for @t, and L2's multi-scale weight (register form). */
void dm3_scene(const struct lg1314_dovi_dm *m, const struct dm_target *t,
	       struct dm3_in *in, s32 *ms_weight);
void dm3_tone_curve(const struct dm3_in *in, u16 curve[DM_TC_SAMPLES]);
/* Layer @k of DM3's 3D LUT: IPT (I over the LUT's range, P and T -0.5..0.5)
 * to the panel's Y'CbCr, 16 bits; lut[T][P][Y, Cb, Cr]. */
void dm3_lut_slice(const struct dm3_in *in, const struct dm_output *out,
		   unsigned int k, u16 lut[DM_LUT_DIM][DM_LUT_DIM][3]);
void dm3_rec_curve(const struct dm3_in *in, u32 words[DM_REC_CURVE_WORDS]);
void dm3_rec_lut_slice(const struct dm3_in *in, const struct dm_output *out,
		       unsigned int k, u16 scratch[DM_LUT_DIM][DM_LUT_DIM][3],
		       u32 lut[DM_REC_LUT_WORDS]);

/* A 12-bit PQ code as luminance, 1/10000 cd/m². */
u32 dm_pq_lum(u16 pq);

#endif
