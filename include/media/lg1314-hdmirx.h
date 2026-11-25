/* SPDX-License-Identifier: GPL-2.0 */
/*
 * LG1314 HDMI receiver: interface for the display driver, which shows an input
 * in its video window.
 */
#ifndef _MEDIA_LG1314_HDMIRX_H
#define _MEDIA_LG1314_HDMIRX_H

#include <linux/types.h>
#include <linux/videodev2.h>
#include <drm/drm_mode.h>

struct device_node;
struct notifier_block;
struct lg1314_hdmirx;

#define LG1314_HDMIRX_CONNECTORS	4

/* AVI Y: the pixel encoding on the link */
enum lg1314_hdmirx_format {
	LG1314_HDMIRX_RGB,
	LG1314_HDMIRX_YCBCR422,
	LG1314_HDMIRX_YCBCR444,
	LG1314_HDMIRX_YCBCR420,
};

/* What a port receives. Fields past @locked are valid only when it is set. */
struct lg1314_hdmirx_signal {
	bool locked;			/* link locked and the timing stable */
	bool dvi;			/* no infoframes: RGB, full range */
	struct v4l2_dv_timings timings;
	enum lg1314_hdmirx_format format;
	u8 colorimetry;			/* AVI C: 0 none, 1 BT.601, 2 BT.709, 3 extended */
	u8 ext_colorimetry;		/* AVI EC, when C = 3 */
	u8 rgb_quant;			/* AVI Q: 0 default, 1 limited, 2 full */
	u8 vic;
	u8 bpc;				/* 8, 10, 12, 16 */
	/*
	 * >1: the HDMI switch spreads the picture over this many receiver ports,
	 * left to right (lg1314_hdmirx_split_port()); the timings are the whole
	 * picture's.
	 */
	u8 split;
	/* the source's HDR infoframe (CTA-861 DRM); eotf 0 (and all 0): none */
	struct hdr_metadata_infoframe hdr;
};

enum lg1314_hdmirx_audio_type {
	LG1314_HDMIRX_AUDIO_NONE,
	LG1314_HDMIRX_AUDIO_PCM,
	LG1314_HDMIRX_AUDIO_COMPRESSED,	/* IEC 61937 in 2-channel samples */
	LG1314_HDMIRX_AUDIO_HBR,		/* high bit rate (TrueHD, DTS-HD MA) */
};

/* The audio a port receives (the receiver's I2S outputs carry it once unmuted). */
struct lg1314_hdmirx_audio {
	enum lg1314_hdmirx_audio_type type;
	unsigned int rate;		/* Hz, from ACR N/CTS and the TMDS clock */
	unsigned int channels;		/* PCM: from the audio infoframe */
};

/* The receiver behind @np; ERR_PTR(-EPROBE_DEFER) until it has probed. */
struct lg1314_hdmirx *lg1314_hdmirx_get(struct device_node *np);
void lg1314_hdmirx_put(struct lg1314_hdmirx *rx);

/*
 * Connectors are numbered as on the TV (HDMI 1..4). @connector 0: none; the
 * receiver routes the selected connector through the HDMI switch, if any.
 */
void lg1314_hdmirx_select(struct lg1314_hdmirx *rx, unsigned int connector);
/* SoC receiver port of @connector, or -EINVAL */
int lg1314_hdmirx_connector_port(struct lg1314_hdmirx *rx, unsigned int connector);
int lg1314_hdmirx_get_signal(struct lg1314_hdmirx *rx, unsigned int connector,
			     struct lg1314_hdmirx_signal *sig);
/* The receiver port carrying part @i (0: leftmost) of a split picture. */
int lg1314_hdmirx_split_port(struct lg1314_hdmirx *rx, unsigned int i);
int lg1314_hdmirx_selected_audio(struct lg1314_hdmirx *rx, struct lg1314_hdmirx_audio *aud);

/*
 * Called (process context) when a connector's signal or audio changes, and when
 * the selection changes; the action is the connector number.
 */
int lg1314_hdmirx_register_notifier(struct lg1314_hdmirx *rx, struct notifier_block *nb);
void lg1314_hdmirx_unregister_notifier(struct lg1314_hdmirx *rx, struct notifier_block *nb);

#endif
