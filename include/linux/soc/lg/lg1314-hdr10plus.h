/* SPDX-License-Identifier: GPL-2.0 */
/*
 * LG1314 HDR10+: a picture's dynamic metadata (SMPTE ST 2094-40, application
 * 4, first processing window), as the video decoder hands it to the display.
 */
#ifndef _LINUX_SOC_LG_LG1314_HDR10PLUS_H
#define _LINUX_SOC_LG_LG1314_HDR10PLUS_H

#include <linux/types.h>

#define LG1314_HDR10PLUS_ANCHORS	15

struct lg1314_hdr10plus {
	u32	target_nits;	/* targeted_system_display_maximum_luminance */
	u32	maxscl[3];	/* the scene's max R, G, B: 0.1 cd/m² */
	u32	avg_maxrgb;	/* 0.1 cd/m² */
	u32	hm;		/* distribution_values[8], the maxRGB percentile
				 * the reference method normalizes by (0.1 cd/m²);
				 * 0: fewer than 9 percentiles */
	u16	knee_x, knee_y;	/* / 4095 */
	u8	num_anchors;	/* 0: no tone curve (tone_mapping_flag 0) */
	u16	anchors[LG1314_HDR10PLUS_ANCHORS];	/* / 1023 */
};

#endif
