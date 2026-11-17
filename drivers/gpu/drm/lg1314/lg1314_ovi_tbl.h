/* SPDX-License-Identifier: GPL-2.0 */
/*
 * OVI bring-up table for VX1 / 3840x2160 / 120 Hz / RGBW (OLED55B9PLA link).
 *
 * entry = { addr, val }; opcode = addr & 0x3:
 *   0 WRITE : writel(val, addr)   1 WAIT : udelay(val)
 *   2 CHECK : val 1=READ_START 0=CHECK_END (bracketed regs are preserved),
 *             val>=0xA0 selects a rev-specific set
 * Register addresses are 4-byte aligned so always decode as WRITE.
 */

#ifndef _LG1314_OVI_TBL_H_
#define _LG1314_OVI_TBL_H_

#include <linux/types.h>

struct lg1314_ovi_reg {
	u32 addr;
	u32 val;
};

#define LG1314_OVI_OP_MASK	0x3
#define LG1314_OVI_OP_WRITE	0x0
#define LG1314_OVI_OP_WAIT	0x1
#define LG1314_OVI_OP_CHECK	0x2
#define LG1314_OVI_CHECK_END	0x0
#define LG1314_OVI_READ_START	0x1
#define LG1314_OVI_SET_A0_START	0xa0
#define LG1314_OVI_SET_B0_START	0xb0

static const struct lg1314_ovi_reg lg1314_ovi_4k120_vx1_16lane[] = {
	/* TCON, WPR, SIW, FCIC, Pix, Pix2 reset (released again in channel-on) */
	{ 0xc930a808, 0x001ff850 },

	/* OIF / TCON timing */
	{ 0xc9070004, 0x00000020 }, /* tcon_control */
	{ 0xc9070008, 0x00000080 }, /* tcon_test_ctl */
	{ 0xc907000c, 0x00000000 }, /* tcon_sync_ctl */
	{ 0xc907001c, 0x00000010 }, /* tcon_odc_opt2 */
	{ 0xc9070020, 0x00000109 }, /* tcon_btc_config */
	{ 0xc9070090, 0x00000094 }, /* tcon_sgen_hbp */
	{ 0xc9070094, 0x0000002c }, /* tcon_sgen_hs */
	{ 0xc9070098, 0x00000058 }, /* tcon_sgen_hfp */
	{ 0xc907009c, 0x00000024 }, /* tcon_sgen_vbp */
	{ 0xc90700a0, 0x0000000a }, /* tcon_sgen_vs */
	{ 0xc90700a4, 0x0000002c }, /* tcon_sgen_vfp */
	{ 0xc90700a8, 0x00000780 }, /* tcon_sgen_dewidth = 1920 */
	{ 0xc90700b0, 0x00000870 }, /* tcon_sgen_denum  = 2160 */
	{ 0xc90700b8, 0x00000060 }, /* tcon_vblank_width */
	{ 0xc90700bc, 0x00000004 }, /* tcon_vsync_width */
	{ 0xc90700d4, 0x00000000 }, /* tcon_pgen_ctrl */
	{ 0xc9070154, 0x00000006 }, /* tcon_dither_carry */
	{ 0xc9070228, 0x00000001 }, /* front dummy */
	{ 0xc907022c, 0x0000000c }, /* back dummy */
	{ 0xc9070230, 0x00000870 }, /* vertical line = 2160 */
	{ 0xc9070234, 0x00000000 }, /* ltd mode */
	{ 0xc9070238, 0x00000000 }, /* tcon_rgb_sel0 */
	{ 0xc9070250, 0x00000780 }, /* horizontal mode = 1920 */

	{ 0x00000002, 0x00000001 }, /* READ_START */
	{ 0xc9070300, 0x00000000 }, /* tcon_ddiv_mode (preserved) */
	{ 0x00000002, 0x00000000 }, /* CHECK_END */

	{ 0xc9070304, 0x00020043 }, /* tcon_ddiv_hblank */
	{ 0xc9070308, 0x00000f00 }, /* tcon_ddiv_hwidth = 3840 */
	{ 0xc9079008, 0x00000002 }, /* vx1_config0 */
	{ 0xc907900c, 0x00000000 }, /* vx1_config1 */
	{ 0xc9079010, 0x00000000 }, /* vx1_config2 */
	{ 0xc9079014, 0x00000000 }, /* vx1_config3 */
	{ 0xc9079018, 0x00000000 }, /* vx1_config4 */
	{ 0xc907901c, 0x00000000 }, /* lvds_config0 */
	{ 0xc9079020, 0x00000000 }, /* lvds_config1 */
	{ 0xc9079024, 0x00000000 }, /* lvds_config2 */
	{ 0xc9079028, 0x00000000 }, /* lvds_config3 */

	{ 0x00000002, 0x00000001 }, /* READ_START */
	{ 0xc9079094, 0x00000000 }, /* vx1_lane_sel0 (preserved) */
	{ 0xc9079098, 0x00000000 }, /* vx1_lane_sel1 (preserved) */
	{ 0xc907909c, 0x00000000 }, /* vx1_lane_sel2 (preserved) */
	{ 0xc90790a0, 0x00000000 }, /* vx1_lane_sel3 (preserved) */
	{ 0xc90790a4, 0x00000000 }, /* vx1_lane_sel4 (preserved) */
	{ 0xc90790a8, 0x00000000 }, /* vx1_lane_sel5 (preserved) */
	{ 0x00000002, 0x00000000 }, /* CHECK_END */

	{ 0xc90701dc, 0x00000003 }, /* init test */

	/* CTOP : CLK */
	{ 0xc930a804, 0x00500020 }, /* tcon_clock */
	{ 0xc9307004, 0x00000000 }, /* 594 MHz */
	{ 0xc9308004, 0x00000000 }, /* disp clocks (594) */

	/* CTOP : C4TX */
	{ 0xc930a008, 0x00000016 }, /* sel_fin=1 */

	{ 0x00000002, 0x00000001 }, /* READ_START */
	{ 0xc930a00c, 0x00000000 }, /* itune (preserved) */
	{ 0xc930a010, 0x00000000 }, /* itune (preserved) */
	{ 0xc930a014, 0x00000000 }, /* itune (preserved) */
	{ 0xc930a018, 0x00000000 }, /* flip_en (preserved) */
	{ 0xc930a01c, 0x00000000 }, /* pemp (preserved) */
	{ 0xc930a020, 0x00000000 }, /* pemp (preserved) */
	{ 0xc930a024, 0x00000000 }, /* pemp (preserved) */
	{ 0x00000002, 0x00000000 }, /* CHECK_END */

	{ 0xc930a02c, 0x000ffff0 }, /* link_en=1 (4-19) */
	{ 0xc930a030, 0x0000088a }, /* lp_en=0 */
	{ 0xc930a034, 0x006db6de }, /* link_rstn_en=1 (2-5) */
	{ 0xc930a04c, 0x1557f000 },
	{ 0xc930a0a0, 0x00003808 }, /* cih=1, sel_mode=3 */
	{ 0xc930a0a4, 0x00000000 }, /* enc_rst/off=0 */
	{ 0xc930a0b0, 0x00000000 }, /* scr_off=0 */
	{ 0xc930a0b4, 0x00000000 }, /* scr_rst=0 */
	{ 0xc930a0a0, 0x00003809 }, /* pdb=1 */

	{ 0x00000001, 0x000003e8 }, /* WAIT 1000 us */

	{ 0xc930a0a0, 0x0000380d }, /* link_rstn=1 */
};

#endif /* _LG1314_OVI_TBL_H_ */
