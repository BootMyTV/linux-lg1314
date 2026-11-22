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
 * is off.
 */

#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/hdmi.h>
#include <drm/drm_atomic.h>
#include <drm/drm_connector.h>
#include <drm/drm_edid.h>
#include <drm/drm_print.h>

#include "lg1314_drm.h"

#define HDR_FW_NAME		"lg1314/hdr.bin"

/* Table area: placed where the vendor driver puts it, in the back-end region. */
#define HDR_AREA_OFF		0x240000	/* be_db (+0x40000) + 0x200000 */
#define HDR_AREA_VERSION	6
#define HDR_AREA_SIZE		0x60000
#define HDR_EOTF_B		0x4dca0		/* HDR10 */
#define HDR_EOTF_C		0x4eca0		/* HDR10, 10000 nit */
#define HDR_OETF_C		0x52ca0		/* HDR10 */
#define HDR_TONE_B		0x54fb8		/* HDR10 */
#define HDR_TONE_E		0x55900		/* SDR */
/* header word, counter shift */
#define HDR_CNT_EOTF_B		1, 4
#define HDR_CNT_EOTF_C		1, 8
#define HDR_CNT_OETF_C		1, 24
#define HDR_CNT_TONE_B		2, 4
#define HDR_CNT_TONE_E		2, 16

/* DE firmware commands */
#define DE_CMD_SET_HDR_MODE	0x61		/* {win, LX_HDR_MODE_T} */
#define PQL_LOAD_HDR_INFO	0x5000001b	/* table area address */
#define PQL_SET_HDR_INFO	0x5000001c	/* 0: normal */
#define HDR_MODE_OFF		0
#define HDR_MODE_HDR10		2

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
#define HDR_SGAIN_LUT		0x380		/* 8 words */
#define HIST_SRC_MAX_RGB	4
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
 * The block's primaries conversion (PCC), in its G, B, R order, s2.12. The
 * vendor writes identity (its calibration matrix, identity on this panel) and
 * maps BT.2020 to the panel later, in the back end's gamut LUT, which we
 * don't run. HDR10 gets the vendor kernel's own default instead, BT.2020 to
 * BT.709, which matches what the panel gets for SDR video; SDR identity.
 * (Something before us leaves the default with one coefficient wrong: a
 * pink cast whenever video goes through the block.)
 */
static const u16 hdr_pcc_2020_709[9] = {
	0x121f, 0x7fde, 0x7e02,		/*  1.1326, -0.0083, -0.1245 */
	0x7e65, 0x11e5, 0x7fb6,		/* -0.1003,  1.1184, -0.0181 */
	0x769a, 0x7ed6, 0x1a8f,		/* -0.5874, -0.0728,  1.6599 */
};

static const u16 hdr_pcc_identity[9] = {
	0x1000, 0, 0,
	0, 0x1000, 0,
	0, 0, 0x1000,
};

#define TONE_PTS		33
#define TONE_LUMS		12

struct lg1314_hdr_cc {
	__le32 lgain_x[8], lgain_y[8], sgain_x[8], sgain_y[8];
	__le32 hue_blend, sat_blend;
};

struct lg1314_hdr_fw {
	char magic[4];
	__le32 version;
	__le32 eotf[1024];
	__le32 eotf_10k[1024];
	__le16 oetf[2048];
	__le32 tone_x[TONE_PTS];
	__le32 tone_lum[TONE_LUMS];
	__le32 tone_y[TONE_PTS][TONE_LUMS];
	struct lg1314_hdr_cc cc_sdr, cc_hdr10;
} __packed;

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

/* The tone map for a peak luminance: the same curve for R, G and B. */
static void hdr_put_tone(struct lg1314_drm *lg, u32 nits)
{
	const struct lg1314_hdr_fw *fw = lg->hdr.fw;
	u32 y[TONE_PTS];
	int c, i;

	for (i = 0; i < TONE_PTS; i++)
		y[i] = hdr_interp(nits, fw->tone_lum, fw->tone_y[i], TONE_LUMS);
	for (c = 0; c < 3; c++) {
		u32 off = HDR_TONE_B + c * 2 * TONE_PTS * 4;

		hdr_put(lg, off, fw->tone_x, TONE_PTS);
		for (i = 0; i < TONE_PTS; i++)
			writel(y[i], lg->hdr.area + off + (TONE_PTS + i) * 4);
	}
	hdr_bump(lg, HDR_CNT_TONE_B);
}

static void hdr_put_eotf(struct lg1314_drm *lg, bool over_4000)
{
	const struct lg1314_hdr_fw *fw = lg->hdr.fw;

	hdr_put(lg, HDR_EOTF_B, over_4000 ? fw->eotf_10k : fw->eotf, 1024);
	hdr_bump(lg, HDR_CNT_EOTF_B);
}

static void hdr_rmw(struct lg1314_drm *lg, u32 off, u32 mask, u32 val)
{
	u32 v = lg1314_rd(lg, PE_HDR_PHYS + off);

	lg1314_wr(lg, PE_HDR_PHYS + off, (v & ~mask) | (val & mask));
}

/* The registers the vendor driver writes for SDR or HDR10 (SetModeInfo,
 * SetColorCorrect). */
static void hdr_regs(struct lg1314_drm *lg, bool hdr10)
{
	const struct lg1314_hdr_cc *cc = hdr10 ? &lg->hdr.fw->cc_hdr10 :
						 &lg->hdr.fw->cc_sdr;
	const u16 *pcc;
	int k;

	/*
	 * Everything goes through the block (the firmware sets that itself
	 * once it has the table area, at the first video session; SDR too).
	 */
	hdr_rmw(lg, HDR_TOP_CTRL_05, HDR_MODE_MASK | GENMASK(23, 20),
		HDR_MODE_LUT << 30 | HIST_SRC_MAX_RGB << 20);
	hdr_rmw(lg, HDR_HUE_SAT_ALPHA, 0x1111, hdr10 ? ALPHA_HDR10 : ALPHA_SDR);
	lg1314_wr(lg, PE_HDR_PHYS + HDR_DITHER_CTRL_00,
		  hdr10 ? DITHER_HDR10 : DITHER_SDR);
	hdr_rmw(lg, HDR_HUE_SAT_CTRL, GENMASK(13, 4) | GENMASK(25, 16),
		(le32_to_cpu(cc->hue_blend) & 0x3ff) << 4 |
		(le32_to_cpu(cc->sat_blend) & 0x3ff) << 16);
	pcc = hdr10 ? hdr_pcc_2020_709 : hdr_pcc_identity;
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

static int hdr_set_mode(struct lg1314_drm *lg, u32 mode)
{
	u32 m[2] = { 0, mode };

	return lg1314_de_cmd(lg, DE_CMD_SET_HDR_MODE, m, sizeof(m));
}

/*
 * Apply the stream's HDR metadata (NULL or not PQ: SDR). Called from the
 * commit, only when the metadata changed.
 */
static void lg1314_hdr_apply(struct lg1314_drm *lg,
			     const struct hdr_metadata_infoframe *md)
{
	bool hdr10 = md && md->eotf == HDMI_EOTF_SMPTE_ST2084;
	u32 nits = 0;

	if (!lg->hdr.fw)
		return;

	if (hdr10) {
		nits = hdr_peak_nits(md->max_display_mastering_luminance,
				     md->max_cll);
		if (nits != lg->hdr.nits) {
			hdr_put_eotf(lg, nits > 4000);
			hdr_put_tone(lg, nits);
			lg->hdr.nits = nits;
		}
	}
	if (hdr10 != lg->hdr.hdr10) {
		if (!READ_ONCE(lg->dovi.active))	/* else Dolby Vision's mode */
			hdr_set_mode(lg, hdr10 ? HDR_MODE_HDR10 : HDR_MODE_OFF);
		lg->hdr.hdr10 = hdr10;
	}
	hdr_regs(lg, hdr10);
	if (hdr10)
		dev_info(lg->dev, "HDR10: mastering %u nit, MaxCLL %u: tone map for %u nit\n",
			 md->max_display_mastering_luminance, md->max_cll, nits);
	else
		dev_info(lg->dev, "SDR\n");
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
	u32 a = area, info = 0;
	int i, ret;

	if (resource_size(&lg->de_mem.be) < HDR_AREA_OFF + HDR_AREA_SIZE)
		return;
	ret = request_firmware(&fwf, HDR_FW_NAME, lg->dev);
	if (ret) {
		dev_info(lg->dev, "%s not found: no HDR10\n", HDR_FW_NAME);
		return;
	}
	if (fwf->size != sizeof(*fw) || memcmp(fwf->data, "LGHD", 4) ||
	    le32_to_cpu(((struct lg1314_hdr_fw *)fwf->data)->version) != 1) {
		dev_err(lg->dev, "%s: bad file\n", HDR_FW_NAME);
		release_firmware(fwf);
		return;
	}
	fw = devm_kmemdup(lg->dev, fwf->data, sizeof(*fw), GFP_KERNEL);
	release_firmware(fwf);
	if (!fw)
		return;

	lg->hdr.area = devm_ioremap_wc(lg->dev, area, HDR_AREA_SIZE);
	if (!lg->hdr.area)
		return;
	lg->hdr.fw = fw;

	writel(HDR_AREA_VERSION, lg->hdr.area);
	for (i = 1; i < 8; i++)
		writel(0, lg->hdr.area + i * 4);
	hdr_put(lg, HDR_EOTF_C, fw->eotf_10k, 1024);
	hdr_bump(lg, HDR_CNT_EOTF_C);
	for (i = 0; i < 1024; i++)	/* u16 pairs, halves swapped */
		writel(le16_to_cpu(fw->oetf[2 * i]) << 16 |
		       le16_to_cpu(fw->oetf[2 * i + 1]),
		       lg->hdr.area + HDR_OETF_C + i * 4);
	hdr_bump(lg, HDR_CNT_OETF_C);
	hdr_put_eotf(lg, false);
	hdr_put_tone(lg, 4000);
	lg->hdr.nits = 4000;
	hdr_put_tone_identity(lg);
	wmb();

	lg1314_de_cmd(lg, PQL_LOAD_HDR_INFO, &a, sizeof(a));
	lg1314_de_cmd(lg, PQL_SET_HDR_INFO, &info, sizeof(info));
	hdr_regs(lg, false);
	lg->hdr.applied = true;
	dev_info(lg->dev, "HDR10 tables at %pa\n", &area);
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
