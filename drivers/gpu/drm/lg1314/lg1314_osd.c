// SPDX-License-Identifier: GPL-2.0
/* LG1314 OSD0 graphics plane: HW init, scanout header, enable. */

#include "lg1314_drm.h"

#define OSD0_REG(field)		(OSD_BANK_PHYS + OSD0_CTRL_BASE + (field))

static void osd_rmw(struct lg1314_drm *lg, u32 phys, u32 mask, u32 val)
{
	lg1314_wr(lg, phys, (lg1314_rd(lg, phys) & ~mask) | (val & mask));
}

/*
 * Port of FBDEV_M19_CMN_InitHW + SetupSRScaler: program the OSD display
 * geometry (3840x2160 output) and enable the SR 2x scaler (1920 -> 3840).
 */
void lg1314_osd_init(struct lg1314_drm *lg)
{
	int i;

	lg1314_wr(lg, OSD_TOP_INIT_A_PHYS, OSD_TOP_INIT_A_VAL);
	lg1314_wr(lg, OSD_TOP_INIT_B_PHYS, OSD_TOP_INIT_B_VAL);

	/* Per-OSD-block display size = panel output (3840x2160). */
	for (i = 0; i < 8; i++)
		osd_rmw(lg, OSD_BANK_PHYS + OSD_COMMON_DISP_SIZE0 + i * 4,
			OSD_DISP_SIZE_MASK, OSD_DISP_SIZE_VAL);

	/* Mixer display sizes (separate / blend0 / blend1). */
	osd_rmw(lg, OSD_BANK_PHYS + OSD_SOSD_DISP_SIZE,  OSD_DISP_SIZE_MASK, OSD_DISP_SIZE_VAL);
	osd_rmw(lg, OSD_BANK_PHYS + OSD_BOSD0_DISP_SIZE, OSD_DISP_SIZE_MASK, OSD_DISP_SIZE_VAL);
	osd_rmw(lg, OSD_BANK_PHYS + OSD_BOSD1_DISP_SIZE, OSD_DISP_SIZE_MASK, OSD_DISP_SIZE_VAL);

	/* Fixed mixer mux order (M19). */
	lg1314_wr(lg, OSD_BANK_PHYS + OSD_SOSD_MUX1,  OSD_MIXER_MUX_ORDER);
	lg1314_wr(lg, OSD_BANK_PHYS + OSD_BOSD0_MUX1, OSD_MIXER_MUX_ORDER);
	lg1314_wr(lg, OSD_BANK_PHYS + OSD_BOSD1_MUX1, OSD_MIXER_MUX_ORDER);

	/* OSD sync pulse delay. */
	osd_rmw(lg, OSD_TOP_PHYS + OSD_TOP_SOSD_PULSE, OSD_PULSE_MASK, OSD_PULSE_TIMER);
	osd_rmw(lg, OSD_TOP_PHYS + OSD_TOP_BOSD_PULSE, OSD_PULSE_MASK, OSD_PULSE_TIMER);

	/* SR scaler: SHP output size = panel, top_bypass off, OSD detour off, CSC on. */
	osd_rmw(lg, OSD_SHP_PHYS + SHP_CORE_CTRL_01, 0x0fff0fff, SHP_SIZE_VAL);
	osd_rmw(lg, OSD_SHP_PHYS + SHP_CORE_CTRL_00, SHP_TOP_BYPASS, 0);
	osd_rmw(lg, OSD_BANK_PHYS + OSD_BITMAP_CTRL0,
		OSD_SHP_DETOUR_EN | OSD_SR_CSC_EN, OSD_SR_CSC_EN);
}

void lg1314_osd_enable(struct lg1314_drm *lg, bool on)
{
	u32 main = lg1314_rd(lg, OSD0_REG(OSD_CTRL_MAIN));
	u32 sync = lg1314_rd(lg, OSD0_REG(OSD_CTRL_SYNC0));

	if (on) {
		main |= OSD_EN;
		sync |= OSD_SYNC_EN;
	} else {
		main &= ~OSD_EN;
		sync &= ~OSD_SYNC_EN;
	}
	lg1314_wr(lg, OSD0_REG(OSD_CTRL_MAIN), main);
	lg1314_wr(lg, OSD0_REG(OSD_CTRL_SYNC0), sync);
}

/*
 * Build the 7-word OSD header (FBDEV_M19_OSD_HDR_T) and latch it. The plane
 * input is the framebuffer (w x h); the output is the full panel, so the SR
 * block scales 1920x1080 -> 3840x2160. wpl is the stride in 8-byte OSD words.
 */
void lg1314_osd_flip(struct lg1314_drm *lg, u32 fb_phys, u32 pitch,
		     u32 w, u32 h, bool opaque)
{
	u32 hdr[7];
	u32 wpl = pitch >> 3;
	int i;

	hdr[0] = 0;						/* ypos | xpos<<16 | ckey_en */
	hdr[1] = (h & 0xfff) | ((w & 0xfff) << 16);		/* h_mem | w_mem (input) */
	hdr[2] = (LG1314_PANEL_H & 0xfff) |			/* h_out | w_out (output) */
		 ((LG1314_PANEL_W & 0xfff) << 16);
	hdr[3] = (wpl & 0xffff) | (0xff << 16) |		/* wpl | galpha */
		 (OSD_PXLFMT_ARGB8888 << 24) |
		 (OSD_PXLDEPTH_32BPP << 28) |
		 (opaque ? OSD_GLOBAL_ALPHA_EN : 0);		/* XRGB: ignore per-pixel alpha */
	hdr[4] = 0;						/* color_key */
	hdr[5] = 0;						/* palette ptr */
	hdr[6] = fb_phys;					/* bitmap ptr */

	for (i = 0; i < 7; i++)
		lg1314_wr(lg, OSD0_REG(OSD_CTRL_HDR0 + i * 4), hdr[i]);

	lg1314_wr(lg, OSD0_REG(OSD_CTRL_MAIN),
		  lg1314_rd(lg, OSD0_REG(OSD_CTRL_MAIN)) | OSD_HDR_READY);
}
