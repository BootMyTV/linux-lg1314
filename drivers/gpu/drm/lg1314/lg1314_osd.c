// SPDX-License-Identifier: GPL-2.0
/* LG1314 OSD0 graphics plane: HW init, scanout header, enable. */

#include <linux/sizes.h>

#include "lg1314_drm.h"

#define OSD0_REG(field)		(OSD_BANK_PHYS + OSD0_CTRL_BASE + (field))
#define AFBC_REG(field)		(OSD_AFBC_PHYS + (field))

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

/* Show or hide the OSD layer, leaving its sync (and the vsync interrupt the
 * display's vblank comes from) running. */
void lg1314_osd_layer(struct lg1314_drm *lg, bool on)
{
	osd_rmw(lg, OSD0_REG(OSD_CTRL_MAIN), OSD_EN, on ? OSD_EN : 0);
}

/*
 * Switch OSD0 between linear and AFBC scanout (FBDEV_M19_CMN_SetupFBC): the
 * AFBC decoder path, its auto init, bitmap-only mode, and the byte/channel
 * order the decoder output needs. Only on change.
 */
static void osd_set_afbc(struct lg1314_drm *lg, bool afbc)
{
	u32 dpath = lg1314_rd(lg, OSD_TOP_PHYS + OSD_TOP_CTRL_DPATH);

	if (!!(dpath & OSD_DPATH_AFBC_EN) == afbc)
		return;

	lg1314_wr(lg, OSD_TOP_PHYS + OSD_TOP_CTRL_DPATH,
		  afbc ? dpath | OSD_DPATH_AFBC_EN : dpath & ~OSD_DPATH_AFBC_EN);
	lg1314_wr(lg, OSD_TOP_PHYS + OSD_TOP_AUTO_INIT_AFBC,
		  afbc ? OSD_AUTO_INIT_AFBC_ON : 0);
	lg1314_wr(lg, OSD0_REG(OSD_CTRL_SWAP),
		  afbc ? OSD_SWAP_AFBC : OSD_SWAP_LINEAR);
	osd_rmw(lg, OSD0_REG(OSD_CTRL_PDEC), OSD_PDEC_CH_MASK,
		afbc ? OSD_PDEC_CH_AFBC : OSD_PDEC_CH_LINEAR);
	osd_rmw(lg, OSD0_REG(OSD_CTRL_MAIN), OSD_BITMAP_ONLY,
		afbc ? OSD_BITMAP_ONLY : 0);
}

/*
 * Point the AFBC decoder at a 16x16-superblock buffer (header at @fb_phys,
 * body offsets relative to it) and request the update
 * (FBDEV_M19_CMN_WriteFBCHdrRegs + FlushFBCHdrRegs).
 */
static void osd_afbc_flip(struct lg1314_drm *lg, u32 fb_phys, u32 pitch,
			  u32 w, u32 h)
{
	u32 hdr_size = ALIGN(DIV_ROUND_UP(w, 16) * DIV_ROUND_UP(h, 16) * 16, SZ_1K);

	/* Stock sets this to the body; the decoder doesn't use it. */
	lg1314_wr(lg, OSD0_REG(OSD_BASE_ADDR), fb_phys + hdr_size);

	lg1314_wr(lg, AFBC_REG(AFBC_HD_BASE), fb_phys);
	lg1314_wr(lg, AFBC_REG(AFBC_PL_BASE), fb_phys);	/* same as header */
	lg1314_wr(lg, AFBC_REG(AFBC_PL_END), 0xffffffff);
	lg1314_wr(lg, AFBC_REG(AFBC_START), 0);
	lg1314_wr(lg, AFBC_REG(AFBC_END),
		  (ALIGN(w - 2, 2) & 0xfff) | (((h - 1) & 0xfff) << 16));
	lg1314_wr(lg, AFBC_REG(AFBC_IMG_WIDTH), (pitch / 4) & 0xfff);
	lg1314_wr(lg, AFBC_REG(AFBC_BUF_DEPTH), AFBC_BUF_DEPTH_VAL);
	lg1314_wr(lg, AFBC_REG(AFBC_CTRL_C), AFBC_CTRL_C_VAL);
	lg1314_wr(lg, AFBC_REG(AFBC_DEFAULT), AFBC_DEFAULT_VAL);
	lg1314_wr(lg, AFBC_REG(AFBC_CTRL0), AFBC_CTRL0_CFG);
}

/*
 * Build the 7-word OSD header (FBDEV_M19_OSD_HDR_T) and latch it. The plane
 * input is the framebuffer (w x h); the output is the full panel, so the SR
 * block scales 1920x1080 -> 3840x2160. wpl is the stride in 8-byte OSD words.
 * For AFBC (up to 1920 wide, see lg1314_primary_atomic_check), @fb_phys
 * is the header and @pitch the pixel-row pitch of the superblock-aligned
 * width.
 */
void lg1314_osd_flip(struct lg1314_drm *lg, u32 fb_phys, u32 pitch,
		     u32 w, u32 h, bool opaque, bool afbc)
{
	u32 hdr[7];
	u32 wpl = pitch >> 3;
	u32 scaler, want;
	int i;

	osd_set_afbc(lg, afbc);

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

	/*
	 * Buffer mode lets the OSD use scaler memory as extra line memory for
	 * an unscaled (1:1) plane; without it a 3840-wide plane shows the left
	 * part, a gap, then a duplicate. The bit is active-low: clear it for a
	 * 1:1 plane, leave it set when scaling. Only written on change.
	 */
	scaler = lg1314_rd(lg, OSD0_REG(OSD_CTRL_SCALER));
	want = w == LG1314_PANEL_W ? scaler & ~OSD_SCALER_BUF_MODE_EN_N
				   : scaler | OSD_SCALER_BUF_MODE_EN_N;
	if (want != scaler)
		lg1314_wr(lg, OSD0_REG(OSD_CTRL_SCALER), want);

	if (afbc)
		osd_afbc_flip(lg, fb_phys, pitch, w, h);

	lg1314_wr(lg, OSD0_REG(OSD_CTRL_MAIN),
		  lg1314_rd(lg, OSD0_REG(OSD_CTRL_MAIN)) | OSD_HDR_READY);

	if (afbc)
		lg1314_wr(lg, AFBC_REG(AFBC_CTRL0),
			  AFBC_CTRL0_CFG | AFBC_CTRL0_UPDATE);
}

/* Enable or disable the mixer vsync interrupt, dropping a stale one. */
void lg1314_osd_vsync_irq(struct lg1314_drm *lg, bool on)
{
	u32 v = lg1314_rd(lg, OSD_IRQ_PHYS) & ~OSD_IRQ_PENDING_MASK;

	v = on ? v | OSD_IRQ_BOSD_EN : v & ~OSD_IRQ_BOSD_EN;
	lg1314_wr(lg, OSD_IRQ_PHYS, v | OSD_IRQ_BOSD_PENDING);
}

/* Clear a pending mixer vsync; false if there was none. */
bool lg1314_osd_vsync_ack(struct lg1314_drm *lg)
{
	u32 v = lg1314_rd(lg, OSD_IRQ_PHYS);

	if (!(v & OSD_IRQ_BOSD_PENDING))
		return false;
	lg1314_wr(lg, OSD_IRQ_PHYS,
		  (v & ~OSD_IRQ_PENDING_MASK) | OSD_IRQ_BOSD_PENDING);
	return true;
}
