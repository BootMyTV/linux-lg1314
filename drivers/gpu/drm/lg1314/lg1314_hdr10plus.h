/* SPDX-License-Identifier: GPL-2.0 */
/*
 * HDR10+ tone mapping: a scene's dynamic metadata to the HDR block's tone
 * curve (lg1314_hdr10plus.c, built with the FPU).
 */
#ifndef _LG1314_HDR10PLUS_H_
#define _LG1314_HDR10PLUS_H_

#include <linux/types.h>
#include <linux/soc/lg/lg1314-hdr10plus.h>

struct lg1314_hdr10plus_in {
	struct lg1314_hdr10plus	md;
	u32	panel_nits;	/* the output's full scale */
	u32	static_nits;	/* the stream's peak, for metadata without MaxSCL */
	const u32 *x;		/* the curve's x positions (1.0 = 2^32 - 1) */
	int	pts;
};

/* @y: @in->pts outputs (16 bits); uses the FPU (kernel_fpu_begin / end). */
void lg1314_hdr10plus_curve(const struct lg1314_hdr10plus_in *in, u32 *y);

#endif
