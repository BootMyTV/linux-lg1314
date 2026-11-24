/* SPDX-License-Identifier: GPL-2.0 */
/*
 * The HDR block's transfer-function tables, computed once at init: they
 * follow from the standards, unlike the tone curves (hdr.bin).
 */
#ifndef _LG1314_HDR_TF_H_
#define _LG1314_HDR_TF_H_

#include <linux/types.h>

#define HDR_TF_EOTF_PTS		1024	/* 10-bit signal in, u32 out (1.0 = 2^32 - 1) */
#define HDR_TF_OETF_PTS		2048	/* u16 out */

struct lg1314_hdr_tf {
	/* HDR10: PQ to luminance (/ 10000 cd/m²), gamma 2.2 encoded */
	u32 eotf_pq[HDR_TF_EOTF_PTS];
	/* HDR10's OETF slot (not enabled for HDR10): a linear ramp */
	u16 oetf_lin[HDR_TF_OETF_PTS];
	/* HLG: the BT.2100 inverse OETF, linear scene light */
	u32 eotf_hlg[HDR_TF_EOTF_PTS];
	/* HLG: back to the panel's gamma, x^0.45, 12-bit */
	u16 oetf_hlg[HDR_TF_OETF_PTS];
};

/* Fills the tables; uses the FPU (call between kernel_fpu_begin / end). */
void lg1314_hdr_tf_compute(struct lg1314_hdr_tf *tf);

#endif
