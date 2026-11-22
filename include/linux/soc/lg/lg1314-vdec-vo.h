/* SPDX-License-Identifier: GPL-2.0 */
/*
 * LG1314 video-decode coprocessor: video output. The coprocessor firmware
 * paces pictures onto a display-engine video window; the display driver
 * hands it frames through this interface.
 */
#ifndef _LINUX_SOC_LG_LG1314_VDEC_VO_H
#define _LINUX_SOC_LG_LG1314_VDEC_VO_H

#include <linux/types.h>

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
};

/* The decoder device behind @np, once its firmware is running. */
struct lg1314_vdec *lg1314_vdec_get(struct device_node *np);
void lg1314_vdec_put(struct lg1314_vdec *v);

/*
 * Open an output feeding display-engine channel @de_id, ticking at
 * @rate_num/@rate_den Hz (each a 16-bit value).
 */
struct lg1314_vdec_vo *lg1314_vdec_vo_open(struct lg1314_vdec *v,
					   unsigned int de_id,
					   unsigned int rate_num,
					   unsigned int rate_den,
					   const struct lg1314_vdec_vo_ops *ops,
					   void *ctx);
/* Change the tick rate of a running output. */
int lg1314_vdec_vo_set_rate(struct lg1314_vdec_vo *vo, unsigned int rate_num,
			    unsigned int rate_den);
/* Queue a frame; it goes on screen at the output's next tick. */
int lg1314_vdec_vo_write(struct lg1314_vdec_vo *vo,
			 const struct lg1314_vdec_vo_frame *f);
/* No callbacks run once this returns; frames not yet released never are. */
void lg1314_vdec_vo_close(struct lg1314_vdec_vo *vo);

#endif
