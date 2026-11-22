/* SPDX-License-Identifier: GPL-2.0 */
/*
 * LG1314 video-decode coprocessor: video output. The coprocessor firmware
 * paces pictures onto a display-engine video window; the display driver
 * hands it frames through this interface.
 */
#ifndef _LINUX_SOC_LG_LG1314_VDEC_VO_H
#define _LINUX_SOC_LG_LG1314_VDEC_VO_H

#include <linux/types.h>
#include <linux/videodev2.h>
#include <linux/soc/lg/lg1314-dovi-meta.h>

/*
 * The decoder's tiled picture layout (8x4 tiles, NV12 sizes and offsets): a
 * V4L2 CAPTURE format of the decoder, a framebuffer modifier of the video
 * plane. Only the display engine reads it.
 */
#define LG1314_V4L2_PIX_FMT_NV12_8X4	v4l2_fourcc('L', 'G', 'T', '8')
#define LG1314_FORMAT_MOD_NV12_8X4	((__u64)0x4c << 56 | 1)

struct device_node;
struct lg1314_vdec;
struct lg1314_vdec_vo;

struct lg1314_vdec_vo_ops {
	/* Frame @id is on screen (latched at a vsync). Process context. */
	void (*shown)(void *ctx, u32 id);
	/* Frame @id is off screen or was dropped: its memory is free again. */
	void (*released)(void *ctx, u32 id);
};

/* An NV12 picture in physical memory. */
struct lg1314_vdec_vo_frame {
	u32	id;			/* reported back through the ops */
	u32	addr_y, addr_c;
	u32	width, height;		/* the buffer's size */
	u32	stride;			/* both planes */
	u32	crop_left, crop_right, crop_top, crop_bottom;
	bool	tiled;			/* LG1314_FORMAT_MOD_NV12_8X4, a decoder
					 * picture still held by userspace */
	s64	pts_ns;			/* presentation time, -1: none */
};

/* The decoder device behind @np, once its firmware is running. */
struct lg1314_vdec *lg1314_vdec_get(struct device_node *np);
void lg1314_vdec_put(struct lg1314_vdec *v);

/*
 * Open an output feeding display-engine channel @de_id, ticking at
 * @rate_num/@rate_den Hz (each a 16-bit value). @dovi_profile non-zero: the
 * pictures are a Dolby Vision stream of that (single-layer) profile.
 */
struct lg1314_vdec_vo *lg1314_vdec_vo_open(struct lg1314_vdec *v,
					   unsigned int de_id,
					   unsigned int rate_num,
					   unsigned int rate_den,
					   unsigned int dovi_profile,
					   const struct lg1314_vdec_vo_ops *ops,
					   void *ctx);
/* Change the tick rate of a running output. */
int lg1314_vdec_vo_set_rate(struct lg1314_vdec_vo *vo, unsigned int rate_num,
			    unsigned int rate_den);
/* Queue a frame; it goes on screen at the output's next tick. -ENOENT: a
 * decoder picture (tiled) that is gone and must not be shown. */
int lg1314_vdec_vo_write(struct lg1314_vdec_vo *vo,
			 const struct lg1314_vdec_vo_frame *f);
/* No callbacks run once this returns; frames not yet released never are. */
void lg1314_vdec_vo_close(struct lg1314_vdec_vo *vo);

/*
 * A decoder picture (by luma address): its timestamp (ns) and, for a Dolby
 * Vision stream, its metadata (@pic) and, as the return value, the stream's
 * profile; 0: not a Dolby Vision stream. -ENOENT: not a current picture;
 * -ENODATA: a Dolby Vision picture without metadata. The timestamp and
 * @session (the decoder session the picture is from) are set unless -ENOENT.
 */
int lg1314_vdec_picture_dovi(struct lg1314_vdec *v, u32 addr_y, u64 *timestamp,
			     u32 *session, struct lg1314_dovi_pic *pic);

#endif
