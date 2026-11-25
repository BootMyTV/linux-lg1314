// SPDX-License-Identifier: GPL-2.0
/*
 * An HDMI input in the video window (display-engine window 0), selected with the
 * CRTC property VIDEO_INPUT (off, HDMI 1..4). The frontend draws over it on the
 * OSD (primary plane), as over video. Window 0 is the video plane's otherwise: a
 * commit can't have both.
 *
 * The receiver (lg1314-hdmirx) reports what each input carries; the window is
 * set up from that and again whenever it changes, and blanked without a signal.
 * The capture side (CVI) is run by the display-engine firmware: the host sends
 * the source description and picks the CVI clock in CTOP. Sequence from the GPL
 * kdriver (DE_REG_M19_CviMap, DE_REG_M19_GetFIR) and stock's
 * KADP_DE_SetCviSrcType (HDMI branch); see cfw/hdmi_rx.md.
 */

#include <drm/drm_atomic.h>
#include <drm/drm_print.h>
#include <linux/bitfield.h>
#include <linux/of.h>

#include <media/lg1314-hdmirx.h>

#include "lg1314_drm.h"

/* display-engine firmware commands */
#define DE_CMD_CVI_SRC_TYPE	0x0d	/* LX_DE_CVI_SRC_TYPE_T */
#define DE_CMD_CVI_CSC		0x0e	/* LX_PE_CSC_INPUT_T */
#define DE_CMD_CVI_FIR		0x35	/* LX_DE_CVI_FIR_T */
#define DE_CMD_CVI_CSAMPLE	0x37	/* LX_DE_CSAMPLE_MODE_T */
#define DE_SRC_HDMI		6	/* LX_DE_IN_SRC_HDMI */

/* CTOP CVI_SYN crg_cvi01: CVI0/1 clock = HDMI link A..D (window 0: CVI0 + CVI1) */
#define CVI_SYN_CRG_CVI01	0xc9302004
#define CVI0_CLK_SEL		GENMASK(2, 0)
#define CVI1_CLK_SEL		GENMASK(6, 4)

/* LX_DE_COLOR_STD_T */
enum { DE_COLOR_601, DE_COLOR_709, DE_COLOR_RGB, DE_COLOR_RGBEX, DE_COLOR_XVYCC601,
       DE_COLOR_XVYCC709, DE_COLOR_601_CVD, DE_COLOR_2020_CL, DE_COLOR_2020_NCL };

/* LX_DE_CVI_SRC_TYPE_T, 0x5c bytes (the same in the 05.50 kernel) */
struct de_cvi_src_type {
	u32 channel;
	u16 hsize, vsize, hoffset, voffset;
	u32 color;		/* LX_DE_COLOR_STD_T */
	u32 aspect;		/* 0 4:3, 1 16:9 */
	u32 scan;		/* 0 interlaced, 1 progressive */
	u32 bit_res;		/* 0 10 bit */
	u32 c_sample;		/* 1 4:4:4 */
	u32 de_mode;		/* 1 HDMI (CEA) */
	u32 sampling;		/* 0 normal */
	u32 sc_c_sample;	/* scaler: 0 4:2:0, 1 4:2:2, 2 4:4:4 */
	u32 stability, field_polarity, yc_delay, cvi_port, trid_format;
	u32 input_src;		/* LX_DE_CVI_SRC_HDMI = 6 */
	u32 cvd_color;
	u8 game[4];
	u32 split_bpp, split_mode;
	u16 split_overlap, split_w, split_h, split_rate;
} __packed;
static_assert(sizeof(struct de_cvi_src_type) == 0x5c);

/* LX_DE_CVI_FIR_T */
struct de_cvi_fir {
	u32 channel, enable;
	s16 y[8], c[6];
} __packed;
static_assert(sizeof(struct de_cvi_fir) == 0x24);

/* LX_PE_CSC_INPUT_T: the CVI's input colour conversion */
struct de_cvi_csc {
	u32 win, enable;
	u16 matrix[9];		/* Q12; rows Y, Cb, Cr; columns G, B, R */
	u16 offset[6];		/* 10-bit codes: in G, B, R; out Y, Cb, Cr */
	u16 pad;
} __packed;
static_assert(sizeof(struct de_cvi_csc) == 40);

/*
 * GBR -> YCbCr (limited) matrices from LG's GPL kdriver (pe_hdr_hw_m19.c): "linea"
 * for limited-range RGB, "limit" for full range (scaled by 219/255).
 */
static const u16 csc_bypass[9] = { 0x1000, 0, 0, 0, 0x1000, 0, 0, 0, 0x1000 };
static const u16 csc_601_lim[9] = { 0x0964, 0x01d3, 0x04c9, 0xfa93, 0x082d, 0xfd3f,
				    0xf927, 0xfeac, 0x082d };
static const u16 csc_601_full[9] = { 0x0810, 0x0191, 0x041d, 0xfb58, 0x0706, 0xfda2,
				     0xfa1d, 0xfedd, 0x0706 };
static const u16 csc_709_lim[9] = { 0x0b70, 0x0126, 0x0368, 0xf9b3, 0x082d, 0xfe22,
				    0xf894, 0xff40, 0x082d };
static const u16 csc_709_full[9] = { 0x09d2, 0x00fd, 0x02ed, 0xfa98, 0x0706, 0xfe62,
				     0xf99e, 0xff5d, 0x0706 };

static bool input_is_rgb(const struct lg1314_hdmirx_signal *sig)
{
	return sig->dvi || sig->format == LG1314_HDMIRX_RGB;
}

/* RGB quantisation: default is limited for CE formats (any VIC but 640x480), else full */
static bool input_rgb_limited(const struct lg1314_hdmirx_signal *sig)
{
	return !sig->dvi && (sig->rgb_quant == 1 || (sig->rgb_quant == 0 && sig->vic > 1));
}

/*
 * The CVI's colour conversion: RGB sources to YCbCr (BT.709, BT.601 for SD),
 * which is what the display engine is then told; YCbCr passes unchanged.
 */
static void input_csc(const struct lg1314_hdmirx_signal *sig, struct de_cvi_csc *csc)
{
	bool sd = sig->timings.bt.height < 720;

	memset(csc, 0, sizeof(*csc));
	csc->enable = 1;
	if (!input_is_rgb(sig)) {
		memcpy(csc->matrix, csc_bypass, sizeof(csc->matrix));
		return;
	}
	if (input_rgb_limited(sig)) {
		memcpy(csc->matrix, sd ? csc_601_lim : csc_709_lim, sizeof(csc->matrix));
	} else {
		memcpy(csc->matrix, sd ? csc_601_full : csc_709_full, sizeof(csc->matrix));
		csc->offset[3] = 64;
	}
	csc->offset[4] = 512;
	csc->offset[5] = 512;
}

/* The colour standard the CVI is told, from the AVI infoframe. */
static u32 input_de_color(const struct lg1314_hdmirx_signal *sig)
{
	const struct v4l2_bt_timings *bt = &sig->timings.bt;

	/* converted to YCbCr by the CVI (input_csc()) */
	if (input_is_rgb(sig))
		return bt->height < 720 ? DE_COLOR_601 : DE_COLOR_709;
	switch (sig->colorimetry) {
	case 1:
		return DE_COLOR_601;
	case 2:
		return DE_COLOR_709;
	case 3:
		switch (sig->ext_colorimetry) {
		case 0:
			return DE_COLOR_XVYCC601;
		case 1:
			return DE_COLOR_XVYCC709;
		case 2:			/* sYCC601 */
		case 3:			/* opYCC601 */
			return DE_COLOR_601;
		case 5:
			return DE_COLOR_2020_CL;
		case 6:
			return DE_COLOR_2020_NCL;
		}
		return DE_COLOR_709;
	}
	/* no colorimetry: SD 601, else 709 */
	return bt->height < 720 ? DE_COLOR_601 : DE_COLOR_709;
}

/*
 * The panel rate locked to an input of @mhz: the one of 120, 100, 96 Hz that is
 * a whole multiple of it (within 1 %; 59.94 -> 120 too), else 120.
 */
static unsigned int input_panel_hz(u64 mhz)
{
	static const unsigned int panel[] = { 120, 100, 96 };
	unsigned int i;
	u64 n;

	for (i = 0; mhz && i < ARRAY_SIZE(panel); i++) {
		n = DIV_ROUND_CLOSEST_ULL((u64)panel[i] * 1000, mhz);
		if (n && abs((s64)(n * mhz) - panel[i] * 1000) * 100 <= panel[i] * 1000)
			return panel[i];
	}
	return 120;
}

static int input_program(struct lg1314_drm *lg, unsigned int connector,
			 const struct lg1314_hdmirx_signal *sig)
{
	const struct v4l2_bt_timings *bt = &sig->timings.bt;
	struct lg1314_de_rect in = { bt->width, bt->height, 0, 0 };
	struct lg1314_de_rect out = { LG1314_PANEL_W, LG1314_PANEL_H, 0, 0 };
	struct de_cvi_fir fir = {
		.channel = 0, .enable = 1,
		.y = { 256 }, .c = { 128, 74, 0, -11, 0, 1 },	/* normal sampling */
	};
	struct de_cvi_src_type src = {
		.channel = 0,
		.hsize = bt->width, .vsize = bt->height,
		.color = input_de_color(sig),
		.aspect = 1,
		.scan = !bt->interlaced,
		.bit_res = 0,
		.c_sample = 1,
		.de_mode = 1,
		.sampling = 0,
		.sc_c_sample = 1,
		.input_src = 6,
	};
	struct { u32 channel, three_tap; } csample = { 0, 1 };
	int port = lg1314_hdmirx_connector_port(lg->input.rx, connector);
	u64 tot = (u64)V4L2_DV_BT_FRAME_WIDTH(bt) * V4L2_DV_BT_FRAME_HEIGHT(bt);
	u64 mhz = tot ? div64_u64(bt->pixelclock * 1000, tot) : 0;
	u32 v;
	int ret;

	if (port < 0)
		return port;

	/*
	 * A change while showing: no frames from between the two set-ups. Possibly
	 * unnecessary: every change seen so far dropped the link, and the receiver
	 * mutes it while it re-trains; kept for changes that don't (colorimetry or
	 * range only).
	 */
	if (lg->input.shown)
		lg1314_video_de_blank(lg, true);

	/* CVI clocks from the HDMI link (the firmware selects the data) */
	v = lg1314_rd(lg, CVI_SYN_CRG_CVI01);
	v &= ~(CVI0_CLK_SEL | CVI1_CLK_SEL);
	v |= FIELD_PREP(CVI0_CLK_SEL, port) | FIELD_PREP(CVI1_CLK_SEL, port);
	lg1314_wr(lg, CVI_SYN_CRG_CVI01, v);

	ret = lg1314_video_de_source(lg, DE_SRC_HDMI, port, NULL, NULL);
	if (!ret)
		ret = lg1314_de_cmd(lg, DE_CMD_CVI_CSAMPLE, &csample, sizeof(csample));
	if (!ret)
		ret = lg1314_de_cmd(lg, DE_CMD_CVI_FIR, &fir, sizeof(fir));
	if (!ret)
		ret = lg1314_de_cmd(lg, DE_CMD_CVI_SRC_TYPE, &src, sizeof(src));
	if (!ret) {
		struct de_cvi_csc csc;

		input_csc(sig, &csc);
		ret = lg1314_de_cmd(lg, DE_CMD_CVI_CSC, &csc, sizeof(csc));
	}
	if (!ret)
		ret = lg1314_video_de_source(lg, -1, 0, &in, &out);
	if (ret)
		return ret;

	lg->input.panel_hz = input_panel_hz(mhz);
	lg1314_de_frame_lock(lg, lg->input.panel_hz);
	drm_info(&lg->drm, "input HDMI %u: %ux%u%c %llu.%03llu Hz, colour %u, %u bit\n",
		 connector, bt->width, bt->height, bt->interlaced ? 'i' : 'p',
		 mhz / 1000, mhz % 1000, src.color, sig->bpc);
	return lg1314_video_de_blank(lg, false);
}

/* Show the selected input as it is now. Called with input.lock held. */
static void input_apply(struct lg1314_drm *lg)
{
	struct lg1314_hdmirx_signal sig;
	unsigned int k = lg->input.active;

	if (!k)
		return;
	if (lg1314_hdmirx_get_signal(lg->input.rx, k, &sig) || !sig.locked) {
		if (lg->input.shown) {
			drm_info(&lg->drm, "input HDMI %u: no signal\n", k);
			lg1314_video_de_blank(lg, true);
		}
		lg->input.shown = false;
		return;
	}
	if (lg->input.shown && !memcmp(&sig, &lg->input.sig, sizeof(sig)))
		return;
	lg->input.sig = sig;
	lg->input.shown = !input_program(lg, k, &sig);
	if (!lg->input.shown)
		drm_err(&lg->drm, "input HDMI %u: window setup failed\n", k);
}

static void input_work(struct work_struct *work)
{
	struct lg1314_drm *lg = container_of(work, struct lg1314_drm, input.work);

	mutex_lock(&lg->input.lock);
	input_apply(lg);
	mutex_unlock(&lg->input.lock);
}

static int input_notify(struct notifier_block *nb, unsigned long connector, void *data)
{
	struct lg1314_drm *lg = container_of(nb, struct lg1314_drm, input.nb);

	if (connector == READ_ONCE(lg->input.active))
		schedule_work(&lg->input.work);
	return NOTIFY_OK;
}

/* Select connector @k (0: none) for window 0; from the commit. */
void lg1314_input_set(struct lg1314_drm *lg, unsigned int k)
{
	if (!lg->input.rx)
		return;
	mutex_lock(&lg->input.lock);
	if (k == lg->input.active)
		goto out;
	if (lg->input.active) {
		lg1314_video_de_blank(lg, true);
		lg1314_video_de_release(lg);
		if (lg->input.panel_hz)
			lg1314_de_frame_lock(lg, lg1314_panel_hz(lg->refresh_mhz));
		lg->input.panel_hz = 0;
		lg->input.shown = false;
		drm_info(&lg->drm, "input off\n");
	}
	WRITE_ONCE(lg->input.active, k);
	lg1314_hdmirx_select(lg->input.rx, k);
	if (k) {
		/* black until the input shows */
		lg1314_video_de_blank(lg, true);
		input_apply(lg);
	}
out:
	mutex_unlock(&lg->input.lock);
}

/* Whether the panel's rate follows an input now (not the mode). */
bool lg1314_input_owns_rate(struct lg1314_drm *lg)
{
	return READ_ONCE(lg->input.panel_hz) != 0;
}

/* The receiver, for a commit selecting an input; NULL: none yet. */
struct lg1314_hdmirx *lg1314_input_rx(struct lg1314_drm *lg)
{
	struct lg1314_hdmirx *rx;

	if (lg->input.rx || !lg->input.np)
		return lg->input.rx;
	rx = lg1314_hdmirx_get(lg->input.np);
	if (IS_ERR(rx))
		return NULL;
	if (cmpxchg(&lg->input.rx, NULL, rx)) {
		lg1314_hdmirx_put(rx);
		return lg->input.rx;
	}
	lg1314_hdmirx_register_notifier(rx, &lg->input.nb);
	return rx;
}

static const struct drm_prop_enum_list input_names[] = {
	{ 0, "off" }, { 1, "HDMI 1" }, { 2, "HDMI 2" }, { 3, "HDMI 3" }, { 4, "HDMI 4" },
};

int lg1314_input_init(struct lg1314_drm *lg)
{
	mutex_init(&lg->input.lock);
	INIT_WORK(&lg->input.work, input_work);
	lg->input.nb.notifier_call = input_notify;
	lg->input.np = of_parse_phandle(lg->dev->of_node, "lge,hdmirx", 0);
	if (!lg->input.np)
		return 0;
	lg->input.prop = drm_property_create_enum(&lg->drm, 0, "VIDEO_INPUT", input_names,
						  ARRAY_SIZE(input_names));
	if (!lg->input.prop)
		return -ENOMEM;
	drm_object_attach_property(&lg->crtc.base, lg->input.prop, 0);
	return 0;
}

void lg1314_input_fini(struct lg1314_drm *lg)
{
	if (lg->input.rx) {
		lg1314_hdmirx_unregister_notifier(lg->input.rx, &lg->input.nb);
		cancel_work_sync(&lg->input.work);
		lg1314_hdmirx_select(lg->input.rx, 0);
		lg1314_hdmirx_put(lg->input.rx);
		lg->input.rx = NULL;
	}
	of_node_put(lg->input.np);
	lg->input.np = NULL;
}
