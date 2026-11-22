/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Dolby Vision RPU (reference processing unit) parsing and the display
 * engine's composer register image built from it.
 */
#ifndef LG1314_DOVI_H
#define LG1314_DOVI_H

#include <linux/types.h>
#include <linux/soc/lg/lg1314-dovi-meta.h>

#define DOVI_COMP_WORDS		LG1314_DOVI_COMP_WORDS
#define DOVI_MAX_PIVOTS		9	/* luma; chroma uses at most 5 */
#define DOVI_MMR_COEFS		22	/* constant + 3 orders of 7 */
#define DOVI_NUM_RPU_IDS	16

/* One picture's composer (reshaping) parameters, fixed point. */
struct dovi_mapping {
	u8	num_pivots[3];			/* pieces + 1 */
	u16	pivot[3][DOVI_MAX_PIVOTS];	/* absolute, BL code values */
	u8	mapping_idc[3];			/* 0 polynomial, 1 MMR */
	u8	poly_order[3][DOVI_MAX_PIVOTS - 1];	/* 1 linear, 2 quadratic */
	s32	poly_int[3][DOVI_MAX_PIVOTS - 1][3];
	u32	poly_frac[3][DOVI_MAX_PIVOTS - 1][3];
	u8	mmr_order[2];			/* chroma: 1..3 */
	s32	mmr_int[2][DOVI_MMR_COEFS];
	u32	mmr_frac[2][DOVI_MMR_COEFS];
};

struct dovi_rpu {
	u8	profile;			/* 5, 8 (single layer) */
	u8	vdr_bit_depth, bl_bit_depth, el_bit_depth;
	u8	coef_log2_denom;
	bool	disable_residual;
	bool	dm_present;
	u8	dm_levels[2];			/* extension block counts: CM v2.9, v4.0 */
	const struct dovi_mapping *map;	/* in the parser state */
	const struct lg1314_dovi_dm *dm;	/* in the parser state; NULL: none yet */
};

/* Parser state carried across pictures (sequence info, mappings by id). */
struct dovi_state {
	bool	seq_valid;
	u8	vdr_bit_depth, bl_bit_depth, el_bit_depth;
	u8	coef_data_type, coef_log2_denom;
	bool	bl_full_range;
	bool	disable_residual;
	u16	rpu_format;
	u8	vdr_rpu_profile;
	bool	el_spatial_resampling;
	bool	dm_compressed;
	bool	dm_valid;
	struct lg1314_dovi_dm dm;		/* the last DM header */
	u16	valid_ids;			/* bit per vdr_rpu_id */
	struct dovi_mapping map[DOVI_NUM_RPU_IDS];
};

/*
 * Parse one RPU: an HEVC NAL unit of type 62 (with or without start code and
 * NAL header) still carrying emulation prevention bytes. @scratch must hold
 * @len bytes.
 */
int dovi_rpu_parse(struct dovi_state *st, const u8 *nal, size_t len,
		   u8 *scratch, struct dovi_rpu *rpu);

/* The composer register image for a picture of @width x @height. */
void dovi_composer_regs(const struct dovi_rpu *rpu, u32 width, u32 height,
			u32 regs[DOVI_COMP_WORDS]);

#endif
