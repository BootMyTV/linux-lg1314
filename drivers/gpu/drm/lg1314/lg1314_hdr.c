// SPDX-License-Identifier: GPL-2.0
/*
 * HDR10 on the video plane.
 *
 * The picture engine's HDR block (window 0) is run by the DE firmware: it
 * loads the curves from a table area in memory the host sets up (EOTF, OETF
 * and tone map slots, each with a 4-bit update counter in the area's header)
 * and switches the block to HDR10 itself when the decoder's per-picture
 * metadata says PQ. The host supplies the curves, tells the firmware the
 * stream's HDR mode (DE command 0x61) and writes the few HDR block registers
 * the firmware leaves alone.
 *
 * The curves are the vendor's static ones (lg1314/hdr.bin): the tone map is
 * interpolated at the stream's peak luminance from 12 per-luminance curves,
 * as the vendor picture-quality software does when "dynamic tone mapping"
 * is off. With it on (hdr_dtm), a pass per video tick reads the block's
 * histogram, finds the scene's peak and interpolates one of the vendor's
 * dynamic tables at it instead, gliding from curve to curve.
 */

#include <linux/firmware.h>
#include <linux/fpu.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/hdmi.h>
#include <linux/delay.h>
#include <linux/moduleparam.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>
#include <drm/drm_atomic.h>
#include <drm/drm_connector.h>
#include <drm/drm_edid.h>
#include <drm/drm_managed.h>
#include <drm/drm_print.h>

#include "lg1314_drm.h"
#include "lg1314_hdr_tf.h"
#include "lg1314_hdr10plus.h"

/*
 * The vendor's tuned tables (tone curves, colour correction, gamut 3D LUTs),
 * as taken from the stock software. TODO: extract them from the stock
 * firmware at install time (blob extractor) instead of shipping hdr.bin.
 * The transfer functions are computed (lg1314_hdr_tf.c).
 */
#define HDR_FW_NAME		"lg1314/hdr.bin"
#define HDR_FW_VERSION		5

/* Table area: placed where the vendor driver puts it, in the back-end region. */
#define HDR_AREA_OFF		0x240000	/* be_db (+0x40000) + 0x200000 */
#define HDR_AREA_VERSION	6
#define HDR_AREA_SIZE		0x60000
#define HDR_EOTF_A		0x4cca0		/* HLG */
#define HDR_EOTF_B		0x4dca0		/* HDR10 */
#define HDR_EOTF_C		0x4eca0		/* HDR10, 10000 nit */
#define HDR_OETF_A		0x50ca0		/* HLG */
#define HDR_OETF_C		0x52ca0		/* HDR10 */
#define HDR_TONE_A		0x54ca0		/* HLG */
#define HDR_TONE_B		0x54fb8		/* HDR10 */
#define HDR_TONE_E		0x55900		/* SDR */
#define HDR_YLUT_A		0x55f30		/* HLG: luminance gain */
/* header word, counter shift */
#define HDR_CNT_EOTF_A		1, 0
#define HDR_CNT_EOTF_B		1, 4
#define HDR_CNT_EOTF_C		1, 8
#define HDR_CNT_OETF_A		1, 16
#define HDR_CNT_OETF_C		1, 24
#define HDR_CNT_TONE_A		2, 0
#define HDR_CNT_TONE_B		2, 4
#define HDR_CNT_TONE_E		2, 16
#define HDR_CNT_YLUT_A		2, 24

/* DE firmware commands */
#define DE_CMD_SET_HDR_MODE	0x61		/* {win, LX_HDR_MODE_T} */
#define PQL_LOAD_HDR_INFO	0x5000001b	/* table area address */
#define PQL_SET_HDR_INFO	0x5000001c	/* 0: normal */
#define HDR_MODE_OFF		0
#define HDR_MODE_HDR10		2
#define HDR_MODE_HLG		5

/* HDR block registers, window 0 */
#define PE_HDR_PHYS		0xc9005800
#define HDR_TOP_CTRL_05		0x014		/* reg_hist_src [23:20] */
#define HDR_MODE_MASK		GENMASK(31, 30)	/* reg_hdr_mode */
#define HDR_MODE_LUT		2		/* through the block (0: bypass) */
#define HDR_HUE_SAT_CTRL	0x0d0
#define HDR_HUE_SAT_ALPHA	0x0d4
#define HDR_DITHER_CTRL_00	0x2e8
#define HDR_PCC_CTRL_01		0x0a4		/* 5 words: 9 coefficients, 2 per word */
#define HDR_LGAIN_LUT		0x360		/* 8 words */
#define HDR_HIST_IA_CTRL	0x244
#define HDR_HIST_IA_DATA1	0x248		/* bin counts [22:0] */
#define HIST_IA_READ		0x00011000	/* read mode, auto-increment */
#define HIST_IA_NORMAL		0x00018000
#define HDR_SGAIN_LUT		0x380		/* 8 words */
#define PRSV_SRC_VENDOR		2		/* reg_hue_sat_prsv_src [17:16], as at vendor init */
#define HIST_SRC_Y		0
#define HIST_SRC_MAX_RGB	4

/*
 * The source histograms in the noise reduction block (TPD): three, the
 * firmware routes Y, hue and saturation (HSV) to them. Read by the host as
 * the vendor's driver does: request (rd_en), read a frame or two later,
 * done.
 */
#define TPD_CTRL_02		0xc9003348	/* [30] HSV, [31] scaled HSV */
#define TPD_HIST_AI_CTRL	0xc9003390
#define TPD_HIST_AI_DATA	0xc9003394
#define TPD_HIST_NORMAL		BIT(15)
#define TPD_HIST_AI		BIT(12)
#define TPD_HIST_SEL_S		(2 << 13)
#define TPD_HIST_AVAILABLE	BIT(28)
#define TPD_HIST_RD_EN		BIT(30)
#define TPD_HIST_RD_DONE	BIT(31)
/*
 * Dither must stay enabled: "dither off" makes full-scale pixels overflow
 * (black with green fringes). The vendor driver picks the bit mode instead:
 * bypass for SDR, 10 bit for HDR10.
 */
#define DITHER_SDR		0x31
#define DITHER_HDR10		0x01
/* gain selects: out_sgain [12], out_lgain [8], inp_s [4], inp_l [0] */
#define ALPHA_SDR		0x1010
#define ALPHA_HDR10		0x0110

/*
 * The block's primaries conversion (PCC), in its G, B, R order, s2.12:
 * identity, as the vendor's (its calibration matrix, identity on this panel).
 * The gamut goes to the panel's primaries later, in the back end's 3D LUT.
 * (Something before us leaves a BT.2020 to BT.709 default here with one
 * coefficient wrong: a pink cast whenever video goes through the block.)
 */
static const u16 hdr_pcc_identity[9] = {
	0x1000, 0, 0,
	0, 0x1000, 0,
	0, 0, 0x1000,
};

#define TONE_PTS		33
#define TONE_LUMS		12
#define HLG_LUMS		9
#define HIST_BINS		32

/*
 * The vendor's HDR picture modes (hdr_dtm_mode): their dynamic tone map
 * tables, and for HLG their static curve too.
 */
enum { DTM_CINEMA, DTM_CINEMA_HOME, DTM_STANDARD, DTM_VIVID, DTM_MODES };
static const char *const dtm_names[DTM_MODES] = {
	"cinema", "cinema_home", "standard", "vivid",
};

static bool hdr_dtm;
module_param(hdr_dtm, bool, 0644);
MODULE_PARM_DESC(hdr_dtm, "HDR10 and HLG dynamic tone mapping (off: the static curve; HDR10's is PQ-accurate to 400 nit)");
static int hdr_dtm_mode = DTM_CINEMA;
module_param(hdr_dtm_mode, int, 0644);
MODULE_PARM_DESC(hdr_dtm_mode, "The vendor picture mode for tone mapping: 0 cinema, 1 cinema home, 2 standard, 3 vivid (HLG static: cinema modes as BT.2100, the others brighter)");
static bool hdr_dtm_cut;
module_param(hdr_dtm_cut, bool, 0644);
MODULE_PARM_DESC(hdr_dtm_cut, "Change curves at once on a scene change (vendor: glide)");
/*
 * The panel's own boost for highlights on a dark picture (vendor: "Peak
 * Brightness"): the timing controller's module HDR mode with a table of
 * extra drive against the picture's average light level (39 steps of about
 * 0.8 % of full-screen white; none above about 30 %). Vendor levels: off /
 * low / medium / high (its HDR10 Cinema and Dolby Vision default), and the
 * panel's own table (its Vivid).
 */
enum { BOOST_OFF, BOOST_LOW, BOOST_MEDIUM, BOOST_HIGH, BOOST_PANEL, BOOST_LEVELS };
static int hdr_peak = BOOST_HIGH;
module_param(hdr_peak, int, 0644);
MODULE_PARM_DESC(hdr_peak, "Panel highlight boost for HDR10, HLG and Dolby Vision: 0 off, 1 low, 2 medium, 3 high, 4 the panel's table");
/*
 * The back end's 3D LUT (L3D, vendor "WCG"), between the picture engine and the
 * panel, video only: the vendor's tables for this panel (55" OLED), loaded by
 * the DE firmware from a packed image (BE_WCG_LOAD_LUT) and switched by
 * section (BE_WCG_SET_MODE). SDR gets BT.709 to the panel's primaries, as the
 * vendor's Cinema mode (colour gamut Auto); HDR10 and HLG the vendor's
 * BT.2020 to panel table (the vendor converts HDR10 with a matrix in another
 * block, its own table here gives the same colours); Dolby Vision none (its
 * display management targets the panel already).
 */
enum { WCG_IDENTITY, WCG_BT709, WCG_BT2020 };
static bool video_gamut = true;
module_param(video_gamut, bool, 0644);
MODULE_PARM_DESC(video_gamut, "Map video to the panel's gamut as the vendor's Cinema mode (off: the panel's native gamut)");
static bool hdr_dtm_oled = true;
module_param(hdr_dtm_oled, bool, 0644);
MODULE_PARM_DESC(hdr_dtm_oled, "With dynamic tone mapping: dim highlights in bright, low-saturation scenes, as the vendor does for OLED");
static bool hdr_dtm_debug;
module_param(hdr_dtm_debug, bool, 0644);
MODULE_PARM_DESC(hdr_dtm_debug, "Log scene changes");
/*
 * HDR10+ (SMPTE ST 2094-40): the scene's metadata makes the HDR10 curve
 * (lg1314_hdr10plus.c), ahead of dynamic tone mapping. Its output's full
 * scale is the panel's peak as the static curves have it.
 */
static bool hdr_hdr10plus = true;
module_param(hdr_hdr10plus, bool, 0644);
MODULE_PARM_DESC(hdr_hdr10plus, "HDR10+: tone map each scene by its metadata (off: as HDR10)");
static unsigned int hdr10plus_panel = 700;
module_param(hdr10plus_panel, uint, 0644);
MODULE_PARM_DESC(hdr10plus_panel, "HDR10+: the panel peak the curve maps to, cd/m2");
/* pictures shown without metadata before a stream counts as plain HDR10 */
#define H10P_MISSING_MAX	12

struct lg1314_hdr_cc {
	__le32 lgain_x[8], lgain_y[8], sgain_x[8], sgain_y[8];
	__le32 hue_blend, sat_blend;
};

struct lg1314_hdr_dyn {
	__le32 lum[TONE_LUMS];		/* breakpoints: peak as a PQ code */
	__le32 y[TONE_PTS][TONE_LUMS];
};

/*
 * HLG: the system gamma as a gain on luminance (x << 16 | gain, 4096 = 1;
 * the vendor's OLED table is the BT.2100 OOTF for about 650 nit), applied
 * after the inverse OETF; then the tone curve on linear light, then the OETF
 * (x^0.45) back to the panel's gamma. The firmware turns the OETF on for HLG
 * only with a table in its slot.
 */
struct lg1314_hdr_hlg {
	__le32 tone_y[TONE_PTS];	/* static, standard / vivid (cinema: identity) */
	__le32 ylut[TONE_PTS];
	struct lg1314_hdr_cc cc;
	struct lg1314_hdr_hlg_dyn {
		__le32 lum[HLG_LUMS];	/* breakpoints: scene peak as an HLG code */
		__le32 y[TONE_PTS][HLG_LUMS];
	} dyn[3];			/* cinema, standard, vivid */
} __packed;

/*
 * The vendor's OLED highlight adjustment of the dynamic curve
 * (ToneMapAdjustForOLED): HDR10 cinema, cinema home, standard, vivid; HLG
 * cinema (both), standard, vivid.
 */
struct lg1314_hdr_adj {
	u8 bin_thr;		/* highlights: bins above the level - this */
	u8 hl_thr;		/* their share (%) at least this, below 26 */
	__le16 apl_x[11], apl_gain[11];	/* gain (/255) against APL */
	__le16 lvl_x[12];	/* the scene peak (signal code) */
	__le16 d[TONE_PTS][12];	/* what each point loses at gain 255 */
} __packed;

/* hdr.bin: this, then u32 wcg_size and the WCG image. */
struct lg1314_hdr_fw {
	char magic[4];
	__le32 version;
	__le32 tone_x[TONE_PTS];
	__le32 tone_lum[TONE_LUMS];
	__le32 tone_y[TONE_PTS][TONE_LUMS];
	struct lg1314_hdr_cc cc_sdr, cc_hdr10;
	struct lg1314_hdr_dyn dyn[DTM_MODES];
	struct lg1314_hdr_hlg hlg;
	struct lg1314_hdr_adj adj[7];
} __packed;

/* Vendor: PQL_ENG_HDR_setToneMapDbForHDR, by picture mode. */
static const u8 hlg_dyn_of[DTM_MODES] = { 0, 0, 1, 2 };
/* Vendor: ToneMapAdjustForOLED, by picture mode. */
static const u8 adj_hdr10_of[DTM_MODES] = { 0, 1, 2, 3 };
static const u8 adj_hlg_of[DTM_MODES] = { 4, 4, 5, 6 };

static bool hlg_static2(int mode)
{
	return mode == DTM_CINEMA || mode == DTM_CINEMA_HOME;
}

/* DE firmware commands for the back end (destination DPPB, MCU 1) */
#define BE_WCG_LOAD_LUT		0xb0010002	/* {load, image address, -1, -1} */
#define BE_WCG_SET_MODE		0xb0010003	/* {enable, section, -1, -1} */
#define L3D_CTRL_PHYS		0xc9010f00
#define L3D_EN			BIT(0)
#define L3D_WR_SEL		BIT(8)		/* flips at each buffer swap */
#define WCG_STAGE_MAX		0x40000		/* kdrv-be up to be_db */

static void hdr_bump(struct lg1314_drm *lg, unsigned int word, unsigned int shift)
{
	void __iomem *p = lg->hdr.area + word * 4;
	u32 v = readl(p), c = (v >> shift) & 0xf;

	c = c >= 0xf ? 1 : c + 1;
	writel((v & ~(0xfu << shift)) | c << shift, p);
}

static void hdr_put(struct lg1314_drm *lg, u32 off, const __le32 *src, size_t n)
{
	size_t i;

	for (i = 0; i < n; i++)
		writel(le32_to_cpu(src[i]), lg->hdr.area + off + i * 4);
}

static void hdr_put_u32(struct lg1314_drm *lg, u32 off, const u32 *src, size_t n)
{
	size_t i;

	for (i = 0; i < n; i++)
		writel(src[i], lg->hdr.area + off + i * 4);
}

/* The HLG tone x axis: 33 even steps (the 16-bit position in both halves). */
static u32 hlg_tone_x(int i)
{
	return i < TONE_PTS - 1 ? i * 0x08000800u : 0xffffffffu;
}

/* VPQ_ENG_UTIL_InterpolTblU32ToU32: linear, clamped at the ends. */
static u32 hdr_interp(u32 v, const __le32 *in, const __le32 *out, int n)
{
	int i;

	if (v <= le32_to_cpu(in[0]))
		return le32_to_cpu(out[0]);
	for (i = 0; i < n - 1; i++) {
		u32 a = le32_to_cpu(in[i]), b = le32_to_cpu(in[i + 1]);
		u32 ya = le32_to_cpu(out[i]), yb = le32_to_cpu(out[i + 1]);

		if (v > b)
			continue;
		if (b == a)
			return ya;
		return ya + (s64)((s32)(yb - ya)) * (v - a) / (b - a);
	}
	return le32_to_cpu(out[n - 1]);
}

/* VPQ_ENG_UTIL_InterpolTblU16ToU16: the same, on 16-bit tables. */
static u32 hdr_interp16(u32 v, const __le16 *in, const __le16 *out, int n)
{
	int i;

	if (v <= le16_to_cpu(in[0]))
		return le16_to_cpu(out[0]);
	for (i = 0; i < n - 1; i++) {
		u32 a = le16_to_cpu(in[i]), b = le16_to_cpu(in[i + 1]);
		s32 ya = le16_to_cpu(out[i]), yb = le16_to_cpu(out[i + 1]);

		if (v > b)
			continue;
		if (b == a)
			return ya;
		return ya + (yb - ya) * (s32)(v - a) / (s32)(b - a);
	}
	return le16_to_cpu(out[n - 1]);
}

/*
 * SDR goes through the block too (mode 2) with the SDR tone slot: identity,
 * else it gets whatever curve the block last loaded (the HDR10 one after an
 * HDR10 stream: SDR blown out).
 */
static void hdr_put_tone_identity(struct lg1314_drm *lg)
{
	const struct lg1314_hdr_fw *fw = lg->hdr.fw;
	int c, i;

	for (c = 0; c < 3; c++) {
		u32 off = HDR_TONE_E + c * 2 * TONE_PTS * 4;

		hdr_put(lg, off, fw->tone_x, TONE_PTS);
		for (i = 0; i < TONE_PTS; i++)
			writel(le32_to_cpu(fw->tone_x[i]) >> 16,
			       lg->hdr.area + off + (TONE_PTS + i) * 4);
	}
	hdr_bump(lg, HDR_CNT_TONE_E);
}

/* A tone curve into slot B: the same for R, G and B. */
static void hdr_put_tone_y(struct lg1314_drm *lg, const u32 *y)
{
	const struct lg1314_hdr_fw *fw = lg->hdr.fw;
	int c, i;

	for (c = 0; c < 3; c++) {
		u32 off = HDR_TONE_B + c * 2 * TONE_PTS * 4;

		hdr_put(lg, off, fw->tone_x, TONE_PTS);
		for (i = 0; i < TONE_PTS; i++)
			writel(y[i], lg->hdr.area + off + (TONE_PTS + i) * 4);
	}
	hdr_bump(lg, HDR_CNT_TONE_B);
}

/* The OETF table: u16 pairs, halves swapped. */
static void hdr_put_oetf(struct lg1314_drm *lg, u32 off, const u16 *v)
{
	int i;

	for (i = 0; i < HDR_TF_OETF_PTS / 2; i++)
		writel(v[2 * i] << 16 | v[2 * i + 1], lg->hdr.area + off + i * 4);
}

/* An HLG tone curve into slot A: the same for R, G and B. */
static void hdr_put_tone_a(struct lg1314_drm *lg, const u32 *y)
{
	int c, i;

	for (c = 0; c < 3; c++) {
		u32 off = HDR_TONE_A + c * 2 * TONE_PTS * 4;

		for (i = 0; i < TONE_PTS; i++) {
			writel(hlg_tone_x(i), lg->hdr.area + off + i * 4);
			writel(y[i], lg->hdr.area + off + (TONE_PTS + i) * 4);
		}
	}
	hdr_bump(lg, HDR_CNT_TONE_A);
}

/* The static HLG curve for a picture mode. */
static void hlg_tone_static(struct lg1314_drm *lg, int mode, u32 *y)
{
	const struct lg1314_hdr_hlg *h = lg->hdr.hlg_fw;
	int i;

	/* cinema modes: identity, as BT.2100 has it */
	for (i = 0; i < TONE_PTS; i++)
		y[i] = hlg_static2(mode) ? (65535 * i + TONE_PTS / 2) / (TONE_PTS - 1) :
					   le32_to_cpu(h->tone_y[i]);
}

static void hdr_put_hlg_static(struct lg1314_drm *lg, int mode)
{
	u32 y[TONE_PTS];

	hlg_tone_static(lg, mode, y);
	hdr_put_tone_a(lg, y);
	lg->hdr.hlg_static = hlg_static2(mode);
}

/* The HLG slots: EOTF, OETF, the static tone curve and luminance gain. */
static void hdr_put_hlg(struct lg1314_drm *lg)
{
	const struct lg1314_hdr_hlg *h = lg->hdr.hlg_fw;

	hdr_put_u32(lg, HDR_EOTF_A, lg->hdr.tf->eotf_hlg, HDR_TF_EOTF_PTS);
	hdr_bump(lg, HDR_CNT_EOTF_A);
	hdr_put_oetf(lg, HDR_OETF_A, lg->hdr.tf->oetf_hlg);
	hdr_bump(lg, HDR_CNT_OETF_A);
	hdr_put_hlg_static(lg, clamp(READ_ONCE(hdr_dtm_mode), 0, DTM_MODES - 1));
	hdr_put(lg, HDR_YLUT_A, h->ylut, TONE_PTS);
	hdr_bump(lg, HDR_CNT_YLUT_A);
}

/* The static curve for a peak luminance. */
static void hdr_tone_static(struct lg1314_drm *lg, u32 nits, u32 *y)
{
	const struct lg1314_hdr_fw *fw = lg->hdr.fw;
	int i;

	for (i = 0; i < TONE_PTS; i++)
		y[i] = hdr_interp(nits, fw->tone_lum, fw->tone_y[i], TONE_LUMS);
}

static void hdr_put_tone(struct lg1314_drm *lg, u32 nits)
{
	u32 y[TONE_PTS];

	hdr_tone_static(lg, nits, y);
	hdr_put_tone_y(lg, y);
}


static void hdr_rmw(struct lg1314_drm *lg, u32 off, u32 mask, u32 val)
{
	u32 v = lg1314_rd(lg, PE_HDR_PHYS + off);

	lg1314_wr(lg, PE_HDR_PHYS + off, (v & ~mask) | (val & mask));
}

/* The registers the vendor driver writes for SDR, HDR10 or HLG (SetModeInfo,
 * SetColorCorrect). */
static void hdr_regs(struct lg1314_drm *lg, u32 mode)
{
	bool hdr = mode != HDR_MODE_OFF;
	const struct lg1314_hdr_cc *cc =
		mode == HDR_MODE_HLG ? &lg->hdr.hlg_fw->cc :
		hdr ? &lg->hdr.fw->cc_hdr10 : &lg->hdr.fw->cc_sdr;
	const u16 *pcc;
	int k;

	/*
	 * Everything goes through the block (the firmware sets that itself
	 * once it has the table area, at the first video session; SDR too).
	 */
	hdr_rmw(lg, HDR_TOP_CTRL_05,
		HDR_MODE_MASK | GENMASK(23, 20) | GENMASK(17, 16),
		HDR_MODE_LUT << 30 | PRSV_SRC_VENDOR << 16 |
		(mode == HDR_MODE_HLG ? HIST_SRC_Y : HIST_SRC_MAX_RGB) << 20);
	hdr_rmw(lg, HDR_HUE_SAT_ALPHA, 0x1111, hdr ? ALPHA_HDR10 : ALPHA_SDR);
	lg1314_wr(lg, PE_HDR_PHYS + HDR_DITHER_CTRL_00,
		  hdr ? DITHER_HDR10 : DITHER_SDR);
	hdr_rmw(lg, HDR_HUE_SAT_CTRL, GENMASK(13, 4) | GENMASK(25, 16),
		(le32_to_cpu(cc->hue_blend) & 0x3ff) << 4 |
		(le32_to_cpu(cc->sat_blend) & 0x3ff) << 16);
	pcc = hdr_pcc_identity;
	for (k = 0; k < 5; k++)
		lg1314_wr(lg, PE_HDR_PHYS + HDR_PCC_CTRL_01 + 4 * k,
			  pcc[2 * k] << 16 | (k < 4 ? pcc[2 * k + 1] : 0));
	for (k = 0; k < 8; k++) {
		lg1314_wr(lg, PE_HDR_PHYS + HDR_LGAIN_LUT + 4 * k,
			  le32_to_cpu(cc->lgain_x[k]) << 16 |
			  (le32_to_cpu(cc->lgain_y[k]) & 0xffff));
		lg1314_wr(lg, PE_HDR_PHYS + HDR_SGAIN_LUT + 4 * k,
			  le32_to_cpu(cc->sgain_x[k]) << 16 |
			  (le32_to_cpu(cc->sgain_y[k]) & 0xffff));
	}
}

/*
 * The peak luminance the tone map is chosen for, as the vendor software
 * derives it from the mastering display's peak and MaxCLL (in nits): the
 * mastering peak, or MaxCLL when that is lower (unless the mastering peak is
 * above 4000 nits); 4000 with neither; at least 450.
 */
static u32 hdr_peak_nits(u32 mdl, u32 cll)
{
	u32 v;

	if (!mdl)
		v = cll ? cll : 4000;
	else if (mdl > 4000 || !cll || mdl < cll)
		v = mdl;
	else
		v = cll;
	return max(v, 450u);
}

/* The DE's HDR mode for the stream (LX_HDR_MODE_T), Dolby Vision aside. */
u32 lg1314_hdr_mode(struct lg1314_drm *lg)
{
	return lg->hdr.hdr10 ? HDR_MODE_HDR10 :
	       lg->hdr.hlg ? HDR_MODE_HLG : HDR_MODE_OFF;
}

static int hdr_set_mode(struct lg1314_drm *lg, u32 mode)
{
	u32 m[2] = { 0, mode };

	return lg1314_de_cmd(lg, DE_CMD_SET_HDR_MODE, m, sizeof(m));
}

/*
 * Apply the stream's HDR metadata (NULL or neither PQ nor HLG: SDR). Called
 * from the commit, only when the metadata changed.
 */
static void lg1314_hdr_apply(struct lg1314_drm *lg,
			     const struct hdr_metadata_infoframe *md)
{
	bool hdr10 = md && md->eotf == HDMI_EOTF_SMPTE_ST2084;
	bool hlg = md && md->eotf == HDMI_EOTF_BT_2100_HLG && lg->hdr.hlg_fw;
	u32 mode = hdr10 ? HDR_MODE_HDR10 : hlg ? HDR_MODE_HLG : HDR_MODE_OFF;
	u32 nits = 0;

	if (!lg->hdr.fw)
		return;
	mutex_lock(&lg->hdr.lock);
	lg->hdr.dtm_reset = true;
	lg->hdr.boost_level = -1;	/* a panel power cycle resets the TCON */

	if (hdr10) {
		nits = hdr_peak_nits(md->max_display_mastering_luminance,
				     md->max_cll);
		if (nits != lg->hdr.nits || lg->hdr.dtm_on) {
			hdr_put_tone(lg, nits);
			lg->hdr.nits = nits;
			lg->hdr.dtm_on = false;
		}
	}
	if (hlg)
		hdr_put_hlg(lg);
	if (mode != lg1314_hdr_mode(lg)) {
		if (!READ_ONCE(lg->dovi.active))	/* else Dolby Vision's mode */
			hdr_set_mode(lg, mode);
		lg->hdr.hdr10 = hdr10;
		lg->hdr.hlg = hlg;
	}
	hdr_regs(lg, mode);
	if (hdr10) {
		lg->hdr.mdl_min = md->min_display_mastering_luminance;
		lg->hdr.mdl_max = md->max_display_mastering_luminance;
		lg->hdr.max_cll = md->max_cll;
		lg->hdr.max_fall = md->max_fall;
		dev_info(lg->dev, "HDR10: mastering %u nit, MaxCLL %u: tone map for %u nit\n",
			 md->max_display_mastering_luminance, md->max_cll, nits);
	} else if (hlg) {
		dev_info(lg->dev, "HLG\n");
	} else if (md && md->eotf) {
		dev_info(lg->dev, "EOTF %u not supported: shown as SDR\n", md->eotf);
	} else {
		dev_info(lg->dev, "SDR\n");
	}
	mutex_unlock(&lg->hdr.lock);
	lg1314_hdr_kick(lg);
	lg1314_status_update(lg);
}

/*
 * Dynamic tone mapping, after the vendor's (PQL_ENG_HDR_DynamicToneMapTask):
 * the block's histogram (32 bins of the full-range 10-bit signal: max(R, G, B)
 * of PQ for HDR10, Y of HLG) gives the scene's peak as a signal code; the
 * dynamic table is interpolated at the mean of the last three peaks, and the
 * curve in the slot moves toward the result a little per tick.
 */

/* The level below which a fraction (Q15) of the pixels lie. */
static u32 dtm_find_peak(const u32 *h, u64 total, u32 q15)
{
	u64 limit = (total * q15) >> 15;
	int i;

	for (i = 0; i < HIST_BINS; i++) {
		if (limit < h[i])
			return i * 32 + div64_u64(limit * 32, h[i]);
		limit -= h[i];
	}
	return 1024;
}

/* The mastering display's peak as a PQ code (vendor: GetMaxPeakLevel). */
static u32 dtm_max_level(u32 mdl)
{
	static const struct { u16 nits, code; } t[] = {
		{ 4001, 0x3ff }, { 3000, 0x37c }, { 2500, 0x367 }, { 2000, 0x34f },
		{ 1500, 0x32e }, { 1200, 0x315 }, { 1000, 0x303 }, { 800, 0x2e8 },
		{ 500, 0x2b5 },
	};
	int i;

	if (!mdl || mdl == 4000)
		return 0x39b;
	for (i = 0; i < ARRAY_SIZE(t); i++)
		if (mdl >= t[i].nits)
			return t[i].code;
	return 0x2a9;
}

/*
 * The scene's peak (vendor: SetAverageMaxLevel), before clamping: settle is
 * how close the stepped percentiles must come (HDR10 10, HLG 23), q15 the
 * percentile that caps the result.
 */
static u32 dtm_detect(const u32 *h, u64 total, u32 apl, s32 settle, u32 q15)
{
	u64 lim_pair = div_u64(total, 500), lim_bin = div_u64(total, 400);
	u64 lim_low = div_u64(total, 2500);
	u32 d[4], diff[3] = { 0 }, perc = 0, bin = 0, pair = 64, three = 0, p, k;
	int i;

	/* a percentile, stepped down until three successive steps settle */
	d[0] = dtm_find_peak(h, total, 0x7fef);
	for (k = 1; k < 20; k++) {
		d[k & 3] = dtm_find_peak(h, total, 0x7fef - 32 * k);
		diff[(k - 1) % 3] = d[(k - 1) & 3] - d[k & 3];
		if (k >= 3 && (s32)diff[0] <= settle && (s32)diff[1] <= settle &&
		    (s32)diff[2] <= settle) {
			perc = d[(k - 3) & 3];
			break;
		}
	}
	/* the highest bin with 0.25 % of the pixels */
	for (i = HIST_BINS - 1; i >= 0; i--)
		if (h[i] > lim_bin) {
			bin = (i + 1) * 32;
			break;
		}
	/* the highest pair of bins with 0.2 % and within 20 % (sigma / mean) */
	for (i = HIST_BINS - 1; i >= 2; i--) {
		u64 a = h[i], b = h[i - 1], s = a + b;

		if (s >= lim_pair && 50 * (a > b ? a - b : b - a) * (a > b ? a - b : b - a) <= s * s) {
			pair = (i + 1) * 32;
			break;
		}
	}
	/* the highest three bins each with 0.04 to 0.2 %, 0.2 % together */
	for (i = HIST_BINS - 1; i >= 2; i--) {
		u64 a = h[i], b = h[i - 1], cc = h[i - 2];

		if (a + b + cc >= lim_pair && a >= lim_low && b >= lim_low &&
		    cc >= lim_low && a <= lim_pair && b <= lim_pair && cc <= lim_pair) {
			three = (i + 2) * 32;
			break;
		}
	}
	p = max3(perc, bin, pair);
	p = max(p, three);
	p = min(p, dtm_find_peak(h, total, q15) + 14);
	p = max(p, apl);
	return min(p, 1023u);
}

/*
 * The saturation histogram, every other call: a request, then the read.
 * True when lg->hdr.sat holds a fresh one.
 */
static bool dtm_read_sat(struct lg1314_drm *lg)
{
	int i;

	if (!lg->hdr.sat_req) {
		lg1314_wr(lg, TPD_HIST_AI_CTRL, TPD_HIST_RD_EN | TPD_HIST_NORMAL);
		lg->hdr.sat_req = true;
		return false;
	}
	lg->hdr.sat_req = false;
	lg1314_wr(lg, TPD_HIST_AI_CTRL, TPD_HIST_RD_EN | TPD_HIST_SEL_S | TPD_HIST_AI);
	if (!(lg1314_rd(lg, TPD_HIST_AI_CTRL) & TPD_HIST_AVAILABLE)) {
		lg1314_wr(lg, TPD_HIST_AI_CTRL, TPD_HIST_RD_EN | TPD_HIST_NORMAL);
		return false;
	}
	for (i = 0; i < HIST_BINS; i++)
		lg->hdr.sat[i] = lg1314_rd(lg, TPD_HIST_AI_DATA) & GENMASK(21, 0);
	lg1314_wr(lg, TPD_HIST_AI_CTRL,
		  TPD_HIST_RD_DONE | TPD_HIST_RD_EN | TPD_HIST_NORMAL);
	return true;
}

/*
 * Vendor: ToneMapAdjustForOLED. With few strongly coloured pixels (bins
 * 8-13 of saturation: at most 30 %, HLG 40 %) and a moderate share of
 * highlights (pixels above the scene peak less bin_thr bins: hl_thr to 25 %),
 * pull the curve down by a table amount for the peak, scaled by APL.
 */
static bool dtm_oled_adjust(struct lg1314_drm *lg, const struct lg1314_hdr_adj *a,
			    const u32 *h, u64 total, u32 apl, u32 level, bool hlg,
			    u32 *y)
{
	u64 sat = 0, mid = 0, hl = 0;
	u32 gain, hl_pct, sat_pct, from = level - 32 * a->bin_thr;
	int i;

	for (i = 0; i < HIST_BINS; i++) {
		sat += lg->hdr.sat[i];
		if (i >= 8 && i <= 13)
			mid += lg->hdr.sat[i];
		if (from < i * 32 + 31)	/* unsigned, as the vendor's */
			hl += h[i];
	}
	if (!sat)
		return false;
	sat_pct = div64_u64(100 * mid, sat);
	hl_pct = div64_u64(100 * hl, total);
	if (READ_ONCE(hdr_dtm_debug) && time_after(jiffies, lg->hdr.adj_logged + 2 * HZ)) {
		lg->hdr.adj_logged = jiffies;
		dev_info(lg->dev, "DTM OLED adjust: mid saturation %u %%, highlights %u %%, APL %u, peak %u\n",
			 sat_pct, hl_pct, apl, level);
	}
	if (hl_pct < a->hl_thr || hl_pct >= 26 || sat_pct > (hlg ? 40 : 30))
		return false;
	gain = hdr_interp16(apl, a->apl_x, a->apl_gain, 11);
	for (i = 0; i < TONE_PTS; i++) {
		u32 d = hdr_interp16(level, a->lvl_x, a->d[i], 12) * gain / 255;

		y[i] = y[i] > d ? min(y[i] - d, 0xffffu) : 0;
	}
	return gain != 0;
}

/* The HLG cap percentile by picture mode (vendor: SetAverageMaxLevel). */
static const u16 hlg_cap_q15[DTM_MODES] = { 0x7fde, 0x7f5b, 0x7ed8, 0x7ed8 };

/* Back to the static curve, if a dynamic one (or another mode's) is in. */
static void dtm_static(struct lg1314_drm *lg, int mode)
{
	if (lg->hdr.dtm_on && !lg->hdr.hlg) {
		hdr_put_tone(lg, lg->hdr.nits);
		lg->hdr.dtm_on = false;
	}
	if (lg->hdr.hlg && lg->hdr.hlg_static != hlg_static2(mode))
		hdr_put_hlg_static(lg, mode);
}

bool lg1314_hdr_hdr10plus_on(struct lg1314_drm *lg)
{
	return READ_ONCE(lg->hdr.hdr10) && READ_ONCE(lg->hdr.h10p_active) &&
	       READ_ONCE(hdr_hdr10plus) && !READ_ONCE(lg->dovi.active);
}

void lg1314_hdr_hdr10plus_shown(struct lg1314_drm *lg,
				const struct lg1314_hdr10plus *md)
{
	bool kick = false, status = false;

	mutex_lock(&lg->hdr.lock);
	if (md) {
		lg->hdr.h10p_missing = 0;
		if (!lg->hdr.h10p_active || memcmp(md, &lg->hdr.h10p, sizeof(*md))) {
			status = !lg->hdr.h10p_active;
			lg->hdr.h10p = *md;
			lg->hdr.h10p_active = true;
			lg->hdr.h10p_seq++;
			kick = true;
		}
	} else if (lg->hdr.h10p_active &&
		   ++lg->hdr.h10p_missing > H10P_MISSING_MAX) {
		lg->hdr.h10p_active = false;
		kick = status = true;
	}
	mutex_unlock(&lg->hdr.lock);
	if (kick)
		lg1314_hdr_kick(lg);
	if (status)
		lg1314_status_update(lg);
}

void lg1314_hdr_hdr10plus_reset(struct lg1314_drm *lg)
{
	bool was;

	mutex_lock(&lg->hdr.lock);
	was = lg->hdr.h10p_active;
	lg->hdr.h10p_active = false;
	lg->hdr.h10p_missing = 0;
	mutex_unlock(&lg->hdr.lock);
	if (was) {
		lg1314_hdr_kick(lg);
		lg1314_status_update(lg);
	}
}

/* The HDR10+ curve for the metadata on screen into slot B, if not in yet. */
static void hdr10plus_put(struct lg1314_drm *lg)
{
	const struct lg1314_hdr_fw *fw = lg->hdr.fw;
	struct lg1314_hdr10plus_in in = {
		.md = lg->hdr.h10p,
		.panel_nits = clamp(READ_ONCE(hdr10plus_panel), 100u, 10000u),
		.static_nits = lg->hdr.nits,
		.pts = TONE_PTS,
	};
	u32 x[TONE_PTS], y[TONE_PTS];
	int i;

	if (lg->hdr.dtm_on && lg->hdr.h10p_done == lg->hdr.h10p_seq)
		return;
	for (i = 0; i < TONE_PTS; i++)
		x[i] = le32_to_cpu(fw->tone_x[i]);
	in.x = x;
	kernel_fpu_begin();
	lg1314_hdr10plus_curve(&in, y);
	kernel_fpu_end();
	if (!lg->hdr.dtm_on || READ_ONCE(hdr_dtm_debug))
		dev_info(lg->dev, "HDR10+: MaxSCL %u/%u/%u, avg %u (0.1 nit), target %u nit, %s%u anchors -> %u..%u\n",
			 in.md.maxscl[0], in.md.maxscl[1], in.md.maxscl[2],
			 in.md.avg_maxrgb, in.md.target_nits,
			 in.md.num_anchors ? "" : "no curve, ", in.md.num_anchors,
			 y[0], y[TONE_PTS - 1]);
	hdr_put_tone_y(lg, y);
	memcpy(lg->hdr.dtm_prev, y, sizeof(y));
	lg->hdr.dtm_on = true;
	lg->hdr.h10p_done = lg->hdr.h10p_seq;
}

static void lg1314_hdr_dtm_work(struct lg1314_drm *lg)
{
	const struct lg1314_hdr_fw *fw = lg->hdr.fw;
	const struct lg1314_hdr_hlg *hf = lg->hdr.hlg_fw;
	u32 h[HIST_BINS], y[TONE_PTS], level, lo, hi, apl, peakdiff, m1 = 0, m2 = 0;
	u64 total = 0, weighted = 0, hdiff = 0;
	bool fresh, cut = false, pattern, changed = false, hlg;
	int mode = clamp(READ_ONCE(hdr_dtm_mode), 0, DTM_MODES - 1);
	int i;

	mutex_lock(&lg->hdr.lock);
	hlg = lg->hdr.hlg;
	if (!fw || READ_ONCE(lg->dovi.active) || (!lg->hdr.hdr10 && !hlg))
		goto out;
	if (lg->hdr.hdr10 && lg->hdr.h10p_active && READ_ONCE(hdr_hdr10plus)) {
		hdr10plus_put(lg);
		lg->hdr.dtm_reset = true;	/* DTM starts afresh after */
		goto out;
	}
	if (!READ_ONCE(hdr_dtm)) {
		dtm_static(lg, mode);
		goto out;
	}

	lg1314_wr(lg, PE_HDR_PHYS + HDR_HIST_IA_CTRL, HIST_IA_READ);
	for (i = 0; i < HIST_BINS; i++)
		h[i] = lg1314_rd(lg, PE_HDR_PHYS + HDR_HIST_IA_DATA1) & GENMASK(22, 0);
	lg1314_wr(lg, PE_HDR_PHYS + HDR_HIST_IA_CTRL, HIST_IA_NORMAL);
	for (i = 0; i < HIST_BINS; i++) {
		total += h[i];
		weighted += (u64)h[i] * (i * 32 + 16);
		hdiff += abs((s32)(h[i] - lg->hdr.dtm_hist[i]));
		if (h[i] > m1) {
			m2 = m1;
			m1 = h[i];
		} else if (h[i] > m2) {
			m2 = h[i];
		}
	}
	memcpy(lg->hdr.dtm_hist, h, sizeof(h));
	if (!total)
		goto out;
	apl = div64_u64(weighted, total);
	/* a test pattern: the static curve, as the vendor does */
	pattern = 100 * ((u64)m1 + m2) >= 99 * total;

	fresh = (hlg ? lg->hdr.hlg_static >= 0 : !lg->hdr.dtm_on) ||
		lg->hdr.dtm_reset || mode != lg->hdr.dtm_mode;
	lg->hdr.dtm_reset = false;
	lg->hdr.dtm_mode = mode;

	if (hlg) {
		const struct lg1314_hdr_hlg_dyn *dyn = &hf->dyn[hlg_dyn_of[mode]];

		lo = le32_to_cpu(dyn->lum[0]);
		hi = le32_to_cpu(dyn->lum[HLG_LUMS - 1]);
		level = dtm_detect(h, total, apl, 23, hlg_cap_q15[mode]);
	} else {
		lo = 650;
		hi = dtm_max_level(lg->hdr.mdl_max);
		level = dtm_detect(h, total, apl, 10, 0x7faf);
	}
	level = level > lo ? min(level, hi) : lo;
	lg->hdr.dtm_peak[2] = lg->hdr.dtm_peak[1];
	lg->hdr.dtm_peak[1] = lg->hdr.dtm_peak[0];
	lg->hdr.dtm_peak[0] = level;
	peakdiff = abs((s32)(level - lg->hdr.dtm_peak[2]));
	/* scene change: the peak moved and most of the histogram did */
	if (fresh || (peakdiff >= 50 && 4 * hdiff > 3 * total)) {
		cut = !fresh;
		lg->hdr.dtm_peak[1] = lg->hdr.dtm_peak[2] = level;
	}
	level = DIV_ROUND_CLOSEST(lg->hdr.dtm_peak[0] + lg->hdr.dtm_peak[1] +
				  lg->hdr.dtm_peak[2], 3);
	if ((cut || fresh) && READ_ONCE(hdr_dtm_debug))
		dev_info(lg->dev, "DTM %s%s: %s, peak %u (APL %u)\n", hlg ? "HLG " : "",
			 dtm_names[mode], fresh ? "start" : "scene change", level, apl);
	lg->hdr.dtm_level = level;

	if (hlg && pattern) {
		hlg_tone_static(lg, DTM_CINEMA, y);	/* vendor: the HLG2 table */
	} else if (hlg) {
		const struct lg1314_hdr_hlg_dyn *dyn = &hf->dyn[hlg_dyn_of[mode]];

		for (i = 0; i < TONE_PTS; i++)
			y[i] = hdr_interp(level, dyn->lum, dyn->y[i], HLG_LUMS);
	} else if (pattern) {
		hdr_tone_static(lg, lg->hdr.nits, y);
	} else {
		const struct lg1314_hdr_dyn *dyn = &fw->dyn[mode];

		for (i = 0; i < TONE_PTS; i++)
			y[i] = hdr_interp(level, dyn->lum, dyn->y[i], TONE_LUMS);
	}

	if (fresh) {
		lg->hdr.dtm_adj = false;
		lg->hdr.sat_ok = false;
		lg->hdr.sat_req = false;
		/* the vendor's histogram set-up: scaled HSV */
		lg1314_wr(lg, TPD_CTRL_02, lg1314_rd(lg, TPD_CTRL_02) | GENMASK(31, 30));
	}
	if (READ_ONCE(hdr_dtm_oled)) {
		bool adj;

		lg->hdr.sat_ok |= dtm_read_sat(lg);
		adj = lg->hdr.sat_ok &&
		      dtm_oled_adjust(lg, &fw->adj[hlg ? adj_hlg_of[mode] : adj_hdr10_of[mode]],
				      h, total, apl, level, hlg, y);
		if (adj != lg->hdr.dtm_adj && READ_ONCE(hdr_dtm_debug))
			dev_info(lg->dev, "DTM OLED highlight adjust %s (peak %u, APL %u)\n",
				 adj ? "on" : "off", level, apl);
		lg->hdr.dtm_adj = adj;
	}

	/* vendor: PQL_HDR_SlowChangeLUT */
	if (fresh || (cut && READ_ONCE(hdr_dtm_cut))) {
		lg->hdr.dtm_keep = 0;
	} else if (cut || lg->hdr.dtm_keep) {
		/* after a scene change: 1/50 of the way per tick, 29 ticks */
		lg->hdr.dtm_keep = cut ? 29 : lg->hdr.dtm_keep - 1;
		for (i = 0; i < TONE_PTS; i++) {
			s32 v = lg->hdr.dtm_prev[i] +
				((s32)(y[i] - lg->hdr.dtm_prev[i])) / 50;

			y[i] = min_t(s32, v, 0xffff);
		}
	} else {
		/* otherwise in steps: the largest change / 70 (HLG 100), + 1 */
		u32 big = 0, steps;

		for (i = 0; i < TONE_PTS; i++)
			big = max_t(u32, big, abs((s32)(y[i] - lg->hdr.dtm_prev[i])));
		steps = big / (hlg ? 100 : 70) + 1;
		if (steps > 1)
			for (i = 0; i < TONE_PTS; i++)
				y[i] = min_t(s32, lg->hdr.dtm_prev[i] +
					     ((s32)(y[i] - lg->hdr.dtm_prev[i])) / (s32)steps,
					     0xffff);
	}

	for (i = 0; i < TONE_PTS; i++)
		changed |= y[i] != lg->hdr.dtm_prev[i];
	if (changed || fresh) {
		if (hlg)
			hdr_put_tone_a(lg, y);
		else
			hdr_put_tone_y(lg, y);
		memcpy(lg->hdr.dtm_prev, y, sizeof(y));
	}
	if (hlg)
		lg->hdr.hlg_static = -1;
	else
		lg->hdr.dtm_on = true;
out:
	mutex_unlock(&lg->hdr.lock);
}

/* Timing controller registers: 16-bit addresses, big endian. */
#define TCON_MODULE		0x000b
#define TCON_MODULE_HDR		BIT(3)
#define TCON_BOOST_TBL		0x092e
#define BOOST_ENTRIES		39

/*
 * The panel's own table as this panel (55" B9) has it at power-on, for when
 * module HDR is found on (left from an earlier run, the table then being
 * ours): the timing controller keeps its state across a reboot.
 */
static const u8 boost_dflt_b9[BOOST_ENTRIES] = {
	0x7d, 0x7d, 0x7d, 0x7d, 0x7d, 0x6f, 0x65, 0x5f, 0x5a, 0x56, 0x53, 0x50, 0x4e,
	0x4c, 0x4b, 0x42, 0x3a, 0x33, 0x2d, 0x27, 0x23, 0x1e, 0x1a, 0x16, 0x13, 0x10,
	0x0d, 0x0a, 0x08, 0x06, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static int tcon_read(struct lg1314_drm *lg, u16 reg, u8 *val)
{
	u8 a[2] = { reg >> 8, reg & 0xff };
	struct i2c_msg msg[2] = {
		{ .addr = lg->hdr.tcon->addr, .len = 2, .buf = a },
		{ .addr = lg->hdr.tcon->addr, .flags = I2C_M_RD, .len = 1, .buf = val },
	};
	int ret = i2c_transfer(lg->hdr.tcon->adapter, msg, 2);

	return ret == 2 ? 0 : ret < 0 ? ret : -EIO;
}

static int tcon_write(struct lg1314_drm *lg, u16 reg, u8 val)
{
	u8 b[3] = { reg >> 8, reg & 0xff, val };
	int ret = i2c_master_send(lg->hdr.tcon, b, 3);

	return ret == 3 ? 0 : ret < 0 ? ret : -EIO;
}

/* The vendor's table for a level (DIL_VPQ_ENGINE_OLED_SetHdrBoost). */
static void boost_table(const u8 *dflt, int level, u8 *t)
{
	/* entries up to this one take its value; Cinema / Standard gains */
	static const u8 flat[] = { [BOOST_LOW] = 26, [BOOST_MEDIUM] = 20, [BOOST_HIGH] = 14 };
	int i, k;

	if (level == BOOST_OFF) {
		memset(t, 0, BOOST_ENTRIES);
		return;
	}
	if (level == BOOST_PANEL) {
		memcpy(t, dflt, BOOST_ENTRIES);
		return;
	}
	k = flat[level];
	for (i = 0; i < BOOST_ENTRIES; i++) {
		int j = max(i, k);

		t[i] = min(dflt[j] * (j < 15 ? 105 : 100) / 100, 255);
	}
}

/*
 * The boost for what is playing: on for HDR10, HLG and Dolby Vision (the
 * vendor's picks it by picture mode, the same for both); otherwise
 * module HDR off with the panel's own table back (as at power-on).
 */
static void hdr_boost_update(struct lg1314_drm *lg)
{
	int level = READ_ONCE(hdr_peak), i, ret;
	u8 t[BOOST_ENTRIES], m;

	if (!lg->hdr.tcon)
		return;
	if (!lg->hdr.hdr10 && !lg->hdr.hlg && !READ_ONCE(lg->dovi.active))
		level = BOOST_OFF;
	level = clamp(level, 0, BOOST_LEVELS - 1);
	if (level == lg->hdr.boost_level || time_before(jiffies, lg->hdr.boost_retry))
		return;

	ret = tcon_read(lg, TCON_MODULE, &m);
	if (!ret && !lg->hdr.boost_dflt_ok) {
		/* the panel's table, untouched while module HDR is off */
		for (i = 0; i < BOOST_ENTRIES && !ret; i++)
			ret = tcon_read(lg, TCON_BOOST_TBL + i, &lg->hdr.boost_dflt[i]);
		if (!ret && (m & TCON_MODULE_HDR)) {
			dev_warn(lg->dev, "panel boost: module HDR left on, using the B9's own table\n");
			memcpy(lg->hdr.boost_dflt, boost_dflt_b9, BOOST_ENTRIES);
		}
		lg->hdr.boost_dflt_ok = !ret;
	}
	if (ret)
		goto fail;

	if (level == BOOST_OFF && (m & TCON_MODULE_HDR))
		ret = tcon_write(lg, TCON_MODULE, m & ~TCON_MODULE_HDR);
	boost_table(lg->hdr.boost_dflt, level == BOOST_OFF ? BOOST_PANEL : level, t);
	for (i = 0; i < BOOST_ENTRIES && !ret; i++)
		ret = tcon_write(lg, TCON_BOOST_TBL + i, t[i]);
	if (!ret && level != BOOST_OFF && !(m & TCON_MODULE_HDR))
		ret = tcon_write(lg, TCON_MODULE, m | TCON_MODULE_HDR);
	if (ret)
		goto fail;
	if (level != lg->hdr.boost_shown)
		dev_info(lg->dev, "panel boost: %s\n", (const char *[]){
			 "off", "low", "medium", "high", "panel" }[level]);
	lg->hdr.boost_level = lg->hdr.boost_shown = level;
	return;
fail:
	lg->hdr.boost_retry = jiffies + HZ;
	dev_warn_ratelimited(lg->dev, "panel boost: timing controller: %d\n", ret);
}

/* Wait for the firmware's L3D buffer swap (the section's last write). */
static bool wcg_wait_swap(struct lg1314_drm *lg, u32 before)
{
	int i;

	for (i = 0; i < 50; i++) {
		msleep(10);
		if ((lg1314_rd(lg, L3D_CTRL_PHYS) ^ before) & L3D_WR_SEL)
			return true;
	}
	return false;
}

/* The section for the stream: -1 for none. */
static int wcg_want(struct lg1314_drm *lg)
{
	if (!READ_ONCE(video_gamut))
		return -1;
	if (READ_ONCE(lg->dovi.active))
		return WCG_IDENTITY;
	return READ_ONCE(lg->hdr.hdr10) || READ_ONCE(lg->hdr.hlg) ?
	       WCG_BT2020 : WCG_BT709;
}

static void hdr_gamut_update(struct lg1314_drm *lg)
{
	int want;
	u32 p[4] = { 1, 0, ~0u, ~0u }, before;
	void __iomem *stage;

	if (!lg->hdr.wcg)
		return;
	want = wcg_want(lg);
	if (want == lg->hdr.wcg_section || time_before(jiffies, lg->hdr.wcg_retry))
		return;

	if (want >= 0 && !lg->hdr.wcg_loaded) {
		stage = ioremap_wc(lg->de_mem.be.start, lg->hdr.wcg_size);
		if (!stage)
			return;
		memcpy_toio(stage, lg->hdr.wcg, lg->hdr.wcg_size);
		wmb();
		iounmap(stage);
		p[1] = lg->de_mem.be.start;
		if (lg1314_de_cmd(lg, BE_WCG_LOAD_LUT, p, sizeof(p)))
			goto fail;
		lg->hdr.wcg_loaded = true;
	}

	before = lg1314_rd(lg, L3D_CTRL_PHYS);
	p[0] = want >= 0;
	p[1] = want >= 0 ? want : ~0u;
	if (lg1314_de_cmd(lg, BE_WCG_SET_MODE, p, sizeof(p)))
		goto fail;
	if (want >= 0 && !wcg_wait_swap(lg, before)) {
		dev_warn(lg->dev, "gamut LUT: no buffer swap\n");
		goto fail;
	}
	lg->hdr.wcg_section = want;
	dev_info(lg->dev, "gamut LUT: %s\n", (const char *[]){ "off", "none",
		 "BT.709 to panel", "BT.2020 to panel" }[want + 1]);
	return;
fail:
	lg->hdr.wcg_section = -2;
	lg->hdr.wcg_retry = jiffies + HZ;
}

static void lg1314_hdr_work(struct work_struct *work)
{
	struct lg1314_drm *lg = container_of(work, struct lg1314_drm, hdr.dtm_work);

	mutex_lock(&lg->hdr.lock);
	hdr_boost_update(lg);
	hdr_gamut_update(lg);
	mutex_unlock(&lg->hdr.lock);
	lg1314_hdr_dtm_work(lg);
}

/* From the video tick IRQ: a pass per tick while HDR plays. */
void lg1314_hdr_tick(struct lg1314_drm *lg)
{
	bool hlg = READ_ONCE(lg->hdr.hlg) && !READ_ONCE(lg->dovi.active);
	bool hdr = READ_ONCE(lg->hdr.hdr10) || hlg || READ_ONCE(lg->dovi.active);

	if ((READ_ONCE(lg->hdr.hdr10) && (READ_ONCE(hdr_dtm) || lg->hdr.dtm_on ||
					   READ_ONCE(lg->hdr.h10p_active))) ||
	    (hlg && (READ_ONCE(hdr_dtm) || READ_ONCE(lg->hdr.hlg_static) !=
		     hlg_static2(clamp(READ_ONCE(hdr_dtm_mode), 0, DTM_MODES - 1)))) ||
	    (lg->hdr.wcg && READ_ONCE(lg->hdr.wcg_section) !=
	     wcg_want(lg)) ||
	    (lg->hdr.tcon && READ_ONCE(lg->hdr.boost_level) !=
	     (hdr ?
	      clamp(READ_ONCE(hdr_peak), 0, BOOST_LEVELS - 1) : BOOST_OFF)))
		queue_work(system_highpri_wq, &lg->hdr.dtm_work);
}

/* Something changed (stream, Dolby Vision): look again. */
void lg1314_hdr_kick(struct lg1314_drm *lg)
{
	queue_work(system_highpri_wq, &lg->hdr.dtm_work);
}

int lg1314_hdr_setup(struct lg1314_drm *lg)
{
	INIT_WORK(&lg->hdr.dtm_work, lg1314_hdr_work);
	lg->hdr.boost_level = -1;
	lg->hdr.boost_shown = BOOST_OFF;
	lg->hdr.wcg_section = -2;
	return drmm_mutex_init(&lg->drm, &lg->hdr.lock);
}

/* Leave the timing controller as at power-on: it keeps its state over a reboot. */
void lg1314_hdr_fini(struct lg1314_drm *lg)
{
	cancel_work_sync(&lg->hdr.dtm_work);
	mutex_lock(&lg->hdr.lock);
	lg->hdr.hdr10 = false;
	lg->hdr.hlg = false;
	WRITE_ONCE(lg->dovi.active, false);
	lg->hdr.boost_retry = 0;
	hdr_boost_update(lg);
	mutex_unlock(&lg->hdr.lock);
}

/*
 * Temporal noise reduction: the firmware switches it on with each video
 * session, untuned (no motion or scene-change handling): moving objects
 * trail and cuts fade over several frames. The vendor's PE task rewrites the
 * enable from the user setting all the time; we keep it off (it's for noisy
 * sources anyway).
 */
#define PE_ND0_TNR_CTRL_00	0xc9003140	/* reg_tnr_en [0] */
#define PE_ND1_TNR_CTRL_00	0xc9004140

static void lg1314_pe_commit(struct lg1314_drm *lg)
{
	static const u32 tnr[] = { PE_ND0_TNR_CTRL_00, PE_ND1_TNR_CTRL_00 };
	int i;

	for (i = 0; i < ARRAY_SIZE(tnr); i++) {
		u32 v = lg1314_rd(lg, tnr[i]);

		if (v & BIT(0))
			lg1314_wr(lg, tnr[i], v & ~BIT(0));
	}
}

void lg1314_hdr_commit(struct lg1314_drm *lg, struct drm_atomic_state *state)
{
	struct drm_connector_state *cs =
		drm_atomic_get_new_connector_state(state, &lg->connector);
	struct drm_property_blob *blob;
	const struct hdr_output_metadata *md = NULL;

	lg1314_pe_commit(lg);
	if (!cs || !lg->hdr.fw)
		return;
	blob = cs->hdr_output_metadata;
	if (blob && blob->length >= sizeof(*md))
		md = blob->data;

	if (lg->hdr.applied &&
	    (blob ? blob->base.id : 0) == lg->hdr.blob_id)
		return;
	lg->hdr.blob_id = blob ? blob->base.id : 0;
	lg->hdr.applied = true;
	lg1314_hdr_apply(lg, md && md->metadata_type == HDMI_STATIC_METADATA_TYPE1 ?
			 &md->hdmi_metadata_type1 : NULL);
}

/*
 * Once the DE firmware is up: the table area with the HDR10 curves (tone map
 * for the vendor's default 4000 nit, used until a stream's metadata says
 * otherwise), handed to the firmware, and the SDR register settings.
 */
void lg1314_hdr_init(struct lg1314_drm *lg)
{
	const struct firmware *fwf;
	struct lg1314_hdr_fw *fw;
	phys_addr_t area = lg->de_mem.be.start + HDR_AREA_OFF;
	u32 a = area, info = 0, wcg_size = 0;
	struct lg1314_hdr_tf *tf;
	int i, ret;

	if (resource_size(&lg->de_mem.be) < HDR_AREA_OFF + HDR_AREA_SIZE)
		return;
	ret = request_firmware(&fwf, HDR_FW_NAME, lg->dev);
	if (ret) {
		dev_info(lg->dev, "%s not found: no HDR\n", HDR_FW_NAME);
		return;
	}
	if (fwf->size > sizeof(*fw) + 4)
		wcg_size = get_unaligned_le32(fwf->data + sizeof(*fw));
	if (fwf->size < sizeof(*fw) || memcmp(fwf->data, "LGHD", 4) ||
	    le32_to_cpu(((struct lg1314_hdr_fw *)fwf->data)->version) != HDR_FW_VERSION ||
	    !wcg_size || wcg_size > WCG_STAGE_MAX ||
	    fwf->size != sizeof(*fw) + 4 + wcg_size) {
		dev_err(lg->dev, "%s: not version %d\n", HDR_FW_NAME, HDR_FW_VERSION);
		release_firmware(fwf);
		return;
	}
	fw = devm_kmemdup(lg->dev, fwf->data, sizeof(*fw), GFP_KERNEL);
	lg->hdr.wcg = devm_kmemdup(lg->dev, fwf->data + sizeof(*fw) + 4, wcg_size,
				   GFP_KERNEL);
	lg->hdr.wcg_size = lg->hdr.wcg ? wcg_size : 0;
	release_firmware(fwf);
	tf = devm_kzalloc(lg->dev, sizeof(*tf), GFP_KERNEL);
	if (!fw || !tf)
		return;
	kernel_fpu_begin();
	lg1314_hdr_tf_compute(tf);
	kernel_fpu_end();
	lg->hdr.tf = tf;
	lg->hdr.hlg_fw = &fw->hlg;

	lg->hdr.area = devm_ioremap_wc(lg->dev, area, HDR_AREA_SIZE);
	if (!lg->hdr.area)
		return;
	lg->hdr.fw = fw;

	writel(HDR_AREA_VERSION, lg->hdr.area);
	for (i = 1; i < 8; i++)
		writel(0, lg->hdr.area + i * 4);
	/* B and C: the PQ EOTF (the vendor's "10000 nit" table is the same) */
	hdr_put_u32(lg, HDR_EOTF_B, tf->eotf_pq, HDR_TF_EOTF_PTS);
	hdr_bump(lg, HDR_CNT_EOTF_B);
	hdr_put_u32(lg, HDR_EOTF_C, tf->eotf_pq, HDR_TF_EOTF_PTS);
	hdr_bump(lg, HDR_CNT_EOTF_C);
	hdr_put_oetf(lg, HDR_OETF_C, tf->oetf_lin);
	hdr_bump(lg, HDR_CNT_OETF_C);
	hdr_put_tone(lg, 4000);
	lg->hdr.nits = 4000;
	hdr_put_tone_identity(lg);
	wmb();

	lg1314_de_cmd(lg, PQL_LOAD_HDR_INFO, &a, sizeof(a));
	lg1314_de_cmd(lg, PQL_SET_HDR_INFO, &info, sizeof(info));
	hdr_regs(lg, HDR_MODE_OFF);
	lg->hdr.applied = true;
	dev_info(lg->dev, "HDR tables at %pa\n", &area);
}

/*
 * The panel's EDID, made up: an internal panel has none, but players decide
 * from it whether the display takes HDR (Kodi: an HDR static metadata block
 * with PQ). Base block: 3840x2160 60 Hz, BT.709 primaries. CTA-861
 * extension: HDR static metadata (SDR, HDR, PQ, HLG; desired peak 700 nit,
 * frame average 200), colorimetry BT.2020.
 */
static const u8 lg1314_edid[] = {
	0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x1e, 0x6d, 0x14, 0x13,
	0x00, 0x00, 0x00, 0x00, 0x01, 0x1d, 0x01, 0x04, 0xb0, 0x79, 0x44, 0x78,
	0x0a, 0xee, 0x91, 0xa3, 0x54, 0x4c, 0x99, 0x26, 0x0f, 0x50, 0x54, 0x00,
	0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x08, 0xe8, 0x00, 0x30, 0xf2, 0x70,
	0x5a, 0x80, 0xb0, 0x58, 0x8a, 0x00, 0xba, 0xa8, 0x42, 0x00, 0x00, 0x1e,
	0x00, 0x00, 0x00, 0xfc, 0x00, 0x4c, 0x47, 0x31, 0x33, 0x31, 0x34, 0x20,
	0x4f, 0x4c, 0x45, 0x44, 0x0a, 0x20, 0x00, 0x00, 0x00, 0xfd, 0x00, 0x17,
	0x3d, 0x1e, 0xa0, 0x3c, 0x00, 0x0a, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
	0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x62, 0x02, 0x03, 0x0f, 0x00,
	0xe6, 0x06, 0x0f, 0x01, 0x7a, 0x40, 0x00, 0xe3, 0x05, 0xc0, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x8e,
};

void lg1314_hdr_edid(struct drm_connector *connector)
{
	const struct drm_edid *edid = drm_edid_alloc(lg1314_edid, sizeof(lg1314_edid));

	drm_edid_connector_update(connector, edid);
	drm_edid_free(edid);
}

int lg1314_hdr_init_props(struct lg1314_drm *lg)
{
	return drm_connector_attach_hdr_output_metadata_property(&lg->connector);
}
