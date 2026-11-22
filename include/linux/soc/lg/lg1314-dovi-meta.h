/* SPDX-License-Identifier: GPL-2.0 */
/*
 * LG1314 Dolby Vision: what the video decoder hands the display for each
 * picture of a Dolby Vision stream.
 */
#ifndef _LINUX_SOC_LG_LG1314_DOVI_META_H
#define _LINUX_SOC_LG_LG1314_DOVI_META_H

#include <linux/types.h>

#define LG1314_DOVI_COMP_WORDS	173	/* composer register image */

#define LG1314_DOVI_EOTF_PQ	0xffff	/* signal_eotf: SMPTE ST 2084 */

/* signal_color_space */
#define LG1314_DOVI_CS_YCC	0
#define LG1314_DOVI_CS_RGB	1
#define LG1314_DOVI_CS_IPT	2

#define LG1314_DOVI_MAX_L2	8
#define LG1314_DOVI_MAX_L8	8
#define LG1314_DOVI_MAX_L10	4

/* Display management extension blocks (PQ values 12-bit unless noted). */
struct lg1314_dovi_ext {
	u32	levels;			/* BIT(level) per level present, 0..31 */
	bool	has_l254, has_l255;
	struct {
		u16	min_pq, max_pq, avg_pq;
	} l1;				/* the scene's luminance */
	u8	num_l2;
	struct {
		u16	target_max_pq;
		u16	slope, offset, power, chroma_weight, saturation_gain;
		s16	ms_weight;	/* 13-bit signed */
	} l2[LG1314_DOVI_MAX_L2];	/* trims per target display */
	struct {
		u16	min_pq_offset, max_pq_offset, avg_pq_offset;
	} l3;
	struct {
		u16	anchor_pq, anchor_power;
	} l4;
	struct {
		u16	left, right, top, bottom;	/* 13-bit */
	} l5;				/* active area */
	struct {
		u16	max_display_mastering_luminance;
		u16	min_display_mastering_luminance;
		u16	max_cll, max_fall;
	} l6;
	u8	num_l8;
	struct {
		u8	len;		/* payload bytes: 10, 12, 13, 19, 25 */
		u8	target_display_index;
		u16	slope, offset, power, chroma_weight, saturation_gain;
		u16	ms_weight;
		u16	target_mid_contrast, clip_trim;
		u8	saturation_vector[6], hue_vector[6];
	} l8[LG1314_DOVI_MAX_L8];	/* CM v4.0 trims */
	struct {
		u8	len;		/* 1 or 17 */
		u8	source_primary_index;
		u16	primaries[8];	/* R xy, G xy, B xy, W xy */
	} l9;
	u8	num_l10;
	struct {
		u8	len;		/* 5 or 21 */
		u8	target_display_index;
		u16	target_max_pq, target_min_pq;
		u8	target_primary_index;
		u16	primaries[8];
	} l10[LG1314_DOVI_MAX_L10];
	u8	l11[4];			/* content type, white point, ... */
	u8	l254[2];		/* dm mode, dm version index */
	u8	l255[6];
};

/* The RPU's display management header: the signal and its colour space. */
struct lg1314_dovi_dm {
	u8	affected_dm_metadata_id, current_dm_metadata_id;
	u8	scene_refresh_flag;
	s16	ycc_to_rgb[9];		/* Q13, row major */
	u32	ycc_to_rgb_offset[3];	/* Q28, of the signal's full scale */
	s16	rgb_to_lms[9];		/* Q14, row major */
	u16	signal_eotf;
	u16	eotf_param0, eotf_param1;
	u32	eotf_param2;
	u8	bit_depth;
	u8	color_space;
	u8	chroma_format;
	u8	full_range;		/* 0 narrow, 1 full, 2 SDI */
	u16	source_min_pq, source_max_pq;	/* 12-bit PQ */
	u16	source_diagonal;	/* mastering display, inches */
	struct lg1314_dovi_ext ext;
};

struct lg1314_dovi_pic {
	u32			comp[LG1314_DOVI_COMP_WORDS];
	bool			dm_valid;	/* a DM header was seen */
	struct lg1314_dovi_dm	dm;
};

#endif
