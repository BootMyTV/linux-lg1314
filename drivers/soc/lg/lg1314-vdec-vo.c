// SPDX-License-Identifier: GPL-2.0
/*
 * LG1314 video-decode coprocessor: video output.
 *
 * The firmware's output ("vo") takes pictures by physical address, shows
 * each one on a display-engine video window at its next tick, and reports
 * when a picture went on screen and when it came off again. Readiness
 * changes arrive as a message; the events themselves are read with a call.
 */

#include <linux/ioctl.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/soc/lg/lg1314-vdec-vo.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#include "lg1314-vdec.h"

/* The firmware's output commands (type 'v'), 32-bit layouts. */
struct fw_vo_write {
	u64	priv;
	s64	pts;			/* -1: none */
	s32	droppable;
	s32	crop_left, crop_right, crop_top, crop_bottom;
	s32	interlace;		/* 0: progressive */
	s32	display_period;		/* 1: progressive frame */
	s32	multi_picture_index;	/* -1: 2D */
	u32	addr_y, addr_cb, addr_cr;
	u32	flags;
	u32	extra;
	u32	decoder_priv;
} __aligned(8);
static_assert(sizeof(struct fw_vo_write) == 72);

struct fw_vo_fb_info {
	s32	width, height, stride;
};

struct fw_vo_port {
	u32	wid, connect;
};

struct fw_vo_info {
	u32	id;			/* fourcc */
	u32	priv;
};

#define VO_SET_FRAMEBUFFER_INFO		_IOW('v', 3, struct fw_vo_fb_info)
#define VO_SET_FRAMERATE_ES		_IO('v', 5)
#define VO_SET_COLOR_SUBSAMPLE		_IO('v', 11)
#define VO_SET_FRAMERATE_CONTAINER	_IO('v', 15)
#define VO_SET_DE_ID			_IO('v', 16)
#define VO_SET_FREERUN			_IO('v', 27)
#define VO_SET_OUTPUT_PORT		_IOW('v', 26, struct fw_vo_port)
#define VO_WRITE_PICTURE		_IOW('v', 28, struct fw_vo_write)
#define VO_GET_INFO			_IOR('v', 29, struct fw_vo_info)

#define VO_SUBSAMPLE_420		0x00020002
#define VO_RATE(num, den)		((num) << 16 | (den))

#define VO_FCC(a, b, c, d)		((a) << 24 | (b) << 16 | (c) << 8 | (d))
#define VO_EV_SHOWN			VO_FCC('G', 'D', 'V', 'D')
#define VO_EV_OFF			VO_FCC('D', 'I', 'S', 'P')
#define VO_EV_DROP			VO_FCC('D', 'R', 'O', 'P')

#define FW_POLLIN			0x1

struct lg1314_vdec_vo {
	struct lg1314_vdec		*v;
	u32				handle;
	unsigned int			de_id;
	const struct lg1314_vdec_vo_ops	*ops;
	void				*ctx;

	struct list_head		list;	/* in vo_list */
	u32				poll;	/* readiness last reported */
	struct work_struct		work;	/* reads events */

	struct mutex			lock;	/* serialises calls */
	struct fw_vo_fb_info		fb;	/* geometry last set */
};

static LIST_HEAD(vo_list);
static DEFINE_SPINLOCK(vo_list_lock);

static int vo_ioctl(struct lg1314_vdec_vo *vo, unsigned int cmd, u32 arg,
		    void *data)
{
	u32 size = _IOC_SIZE(cmd);
	unsigned int dir = VDEC_CALL_WR;
	struct {
		u32 cmd, arg;
		u8 data[sizeof(struct fw_vo_write)];
	} __aligned(8) msg = { .cmd = cmd, .arg = arg };
	int ret, result;

	if (WARN_ON(size > sizeof(msg.data)))
		return -EINVAL;
	if (size && (_IOC_DIR(cmd) & _IOC_WRITE))
		memcpy(msg.data, data, size);
	if (size && (_IOC_DIR(cmd) & _IOC_READ))
		dir |= VDEC_CALL_RD;

	ret = lg1314_vdec_call(vo->v, VDEC_CALL_VO_IOCTL, vo->handle, &msg,
			       2 * sizeof(u32) + size, dir, &result);
	if (ret)
		return ret;
	if (result < 0)
		return result;
	if (size && (_IOC_DIR(cmd) & _IOC_READ))
		memcpy(data, msg.data, size);
	return 0;
}

static void vo_event_work(struct work_struct *work)
{
	struct lg1314_vdec_vo *vo = container_of(work, struct lg1314_vdec_vo, work);
	struct fw_vo_info info;

	/* Reading an event updates the readiness before the call returns, so
	 * this never asks with nothing pending (which would block). */
	while (READ_ONCE(vo->poll) & FW_POLLIN) {
		mutex_lock(&vo->lock);
		if (vo_ioctl(vo, VO_GET_INFO, 0, &info)) {
			mutex_unlock(&vo->lock);
			break;
		}
		mutex_unlock(&vo->lock);

		switch (info.id) {
		case VO_EV_SHOWN:
			vo->ops->shown(vo->ctx, info.priv);
			break;
		case VO_EV_OFF:
		case VO_EV_DROP:
			vo->ops->released(vo->ctx, info.priv);
			break;
		}
	}
}

/* An output's readiness changed. IRQ context. */
void lg1314_vdec_vo_poll(u32 handle, u32 mask)
{
	struct lg1314_vdec_vo *vo;

	spin_lock(&vo_list_lock);
	list_for_each_entry(vo, &vo_list, list) {
		if (vo->handle == handle) {
			WRITE_ONCE(vo->poll, mask);
			if (mask & FW_POLLIN)
				queue_work(system_highpri_wq, &vo->work);
		}
	}
	spin_unlock(&vo_list_lock);
}

struct lg1314_vdec_vo *lg1314_vdec_vo_open(struct lg1314_vdec *v,
					   unsigned int de_id,
					   unsigned int rate_num,
					   unsigned int rate_den,
					   const struct lg1314_vdec_vo_ops *ops,
					   void *ctx)
{
	struct fw_vo_port port = { .wid = 0, .connect = 1 };
	struct lg1314_vdec_vo *vo;
	int ret, handle;

	vo = kzalloc(sizeof(*vo), GFP_KERNEL);
	if (!vo)
		return ERR_PTR(-ENOMEM);
	vo->v = v;
	vo->de_id = de_id;
	vo->ops = ops;
	vo->ctx = ctx;
	mutex_init(&vo->lock);
	INIT_WORK(&vo->work, vo_event_work);

	ret = lg1314_vdec_call(v, VDEC_CALL_VO_OPEN, 0, NULL, 0, 0, &handle);
	if (!ret && !handle)
		ret = -EIO;
	if (ret)
		goto err_free;
	vo->handle = handle;

	spin_lock_irq(&vo_list_lock);
	list_add(&vo->list, &vo_list);
	spin_unlock_irq(&vo_list_lock);

	/*
	 * Show each picture at the next tick, ignoring timestamps (free run),
	 * ticking at rate_num/rate_den Hz.
	 */
	ret = vo_ioctl(vo, VO_SET_DE_ID, de_id, NULL);
	if (!ret)
		ret = vo_ioctl(vo, VO_SET_COLOR_SUBSAMPLE, VO_SUBSAMPLE_420, NULL);
	if (!ret)
		ret = vo_ioctl(vo, VO_SET_FRAMERATE_ES,
			       VO_RATE(rate_num, rate_den), NULL);
	if (!ret)
		ret = vo_ioctl(vo, VO_SET_FRAMERATE_CONTAINER,
			       VO_RATE(rate_num, rate_den), NULL);
	if (!ret)
		ret = vo_ioctl(vo, VO_SET_FREERUN, 1, NULL);
	if (!ret)
		ret = vo_ioctl(vo, VO_SET_OUTPUT_PORT, 0, &port);
	if (ret) {
		lg1314_vdec_vo_close(vo);
		return ERR_PTR(ret);
	}
	return vo;

err_free:
	kfree(vo);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(lg1314_vdec_vo_open);

int lg1314_vdec_vo_write(struct lg1314_vdec_vo *vo,
			 const struct lg1314_vdec_vo_frame *f)
{
	struct fw_vo_fb_info fb = {
		.width = f->width, .height = f->height, .stride = f->stride,
	};
	struct fw_vo_write w = {
		.priv = f->id,
		.pts = -1,
		.crop_left = f->crop_left, .crop_right = f->crop_right,
		.crop_top = f->crop_top, .crop_bottom = f->crop_bottom,
		.display_period = 1,
		.multi_picture_index = -1,
		.addr_y = f->addr_y,
		.addr_cb = f->addr_c,
	};
	u32 c;
	int ret = 0;

	/* The firmware finds the picture's layout (map type) from its decoded
	 * picture; without it, it shows the memory as raster. The second
	 * part's address is the decoder's too (not NV12's for HEVC). */
	if (f->tiled) {
		/* the picture can't go away until the firmware has the frame */
		lg1314_vdec_pictures_lock(vo->v, false);
		w.decoder_priv = lg1314_vdec_picture_priv(vo->v, f->addr_y, &c);
		/* gone (its decoder session ended, e.g. a seek): it would
		 * show as raw memory */
		if (!w.decoder_priv) {
			lg1314_vdec_pictures_unlock(vo->v, false);
			return -ENOENT;
		}
		w.addr_cb = c;
	}

	mutex_lock(&vo->lock);
	if (memcmp(&fb, &vo->fb, sizeof(fb))) {
		ret = vo_ioctl(vo, VO_SET_FRAMEBUFFER_INFO, 0, &fb);
		if (!ret)
			vo->fb = fb;
	}
	if (!ret)
		ret = vo_ioctl(vo, VO_WRITE_PICTURE, 0, &w);
	mutex_unlock(&vo->lock);
	if (f->tiled)
		lg1314_vdec_pictures_unlock(vo->v, false);
	return ret;
}
EXPORT_SYMBOL_GPL(lg1314_vdec_vo_write);

/* Change the tick rate of a running output. */
int lg1314_vdec_vo_set_rate(struct lg1314_vdec_vo *vo, unsigned int rate_num,
			    unsigned int rate_den)
{
	int ret;

	if (rate_num > U16_MAX || rate_den > U16_MAX)
		return -EINVAL;
	mutex_lock(&vo->lock);
	ret = vo_ioctl(vo, VO_SET_FRAMERATE_CONTAINER, VO_RATE(rate_num, rate_den),
		       NULL);
	mutex_unlock(&vo->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(lg1314_vdec_vo_set_rate);

void lg1314_vdec_vo_close(struct lg1314_vdec_vo *vo)
{
	struct fw_vo_port port = { .wid = 0, .connect = 0 };
	int result;

	spin_lock_irq(&vo_list_lock);
	list_del(&vo->list);
	spin_unlock_irq(&vo_list_lock);
	cancel_work_sync(&vo->work);

	vo_ioctl(vo, VO_SET_OUTPUT_PORT, 0, &port);
	lg1314_vdec_call(vo->v, VDEC_CALL_VO_CLOSE, vo->handle, NULL, 0, 0,
			 &result);
	kfree(vo);
}
EXPORT_SYMBOL_GPL(lg1314_vdec_vo_close);
