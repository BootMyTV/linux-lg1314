// SPDX-License-Identifier: GPL-2.0
/*
 * LG1314 video-decode coprocessor: V4L2 stateful decoder.
 *
 * Each open file is one decoder instance in the firmware. Compressed access
 * units queued on OUTPUT are copied into the instance's stream buffer (CPB);
 * the firmware decodes into its own picture buffers (from the dpb pool) and
 * reports each picture in a notification ring. Decoded pictures are copied
 * into CAPTURE buffers (NV12) and released back to the firmware.
 *
 * The stream geometry is first known when the first picture is decoded: that
 * picture is held, V4L2_EVENT_SOURCE_CHANGE is signalled, and pictures are
 * returned once CAPTURE has been set up for it.
 */

#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-vmalloc.h>

#include "lg1314-vdec.h"

#define DEC_NAME		"lg1314-vdec"

#define DEC_MIN_W		64
#define DEC_MIN_H		64
#define DEC_MAX_W		4096
#define DEC_MAX_H		2304
#define DEC_DEF_W		1920
#define DEC_DEF_H		1080
#define DEC_ES_MIN_SIZE		SZ_4M
#define DEC_CAP_MIN_BUFS	4
#define DEC_MAX_IN_FLIGHT	256	/* access units queued in the firmware;
					 * the CPB's space is the real limit */
#define DEC_CPB_MARGIN		SZ_64K

/* ---- firmware decoder interface ---- */

#define FW_IOC_TYPE		'v'
#define FW_LEN(x)		(sizeof(x))

struct fw_buffer_info {
	u32 addr;
	s32 size;
	s32 read_offset;
	s32 write_offset;
};

struct fw_es_data {
	u64 timestamp;
	u64 uid;
	u32 size;
	u32 buf1_offset;
	u32 buf1_size;
	u32 buf2_offset;
	u32 buf2_size;
	s32 au_type;
	s32 frame_count;
	u32 pad;
};
static_assert(sizeof(struct fw_es_data) == 48);

/* A decoded picture as reported in the notification ring. */
struct fw_decoded {
	s64 timestamp;
	u64 uid;
	u64 user_dpb_priv;
	u32 phys_y, phys_cb, phys_cr;
	s32 buffer_index;
	s32 error_blocks;
	u32 report;
	s32 framerate_num, framerate_den;
	s32 crop_left, crop_right, crop_top, crop_bottom;
	s32 stride, width, height;
	s32 display_width, display_height;
	s32 interlace;
	s32 picture_type;
	s32 display_period;
	u8 rest0[188 - 104];
	u32 vdec_priv;
	u32 addr_y, addr_y_bot;
	u32 addr_cb, addr_cb_bot, addr_cr;
	u32 addr_extra;
	u8 picd_info[32];
};
static_assert(sizeof(struct fw_decoded) == 248);
static_assert(offsetof(struct fw_decoded, report) == 44);
static_assert(offsetof(struct fw_decoded, stride) == 72);
static_assert(offsetof(struct fw_decoded, picture_type) == 96);
static_assert(offsetof(struct fw_decoded, addr_y) == 192);
static_assert(offsetof(struct fw_decoded, addr_cb) == 200);

#define FW_REPORT_EOS		BIT(2)
#define FW_REPORT_ERROR_MASK	0xff00

enum { FW_PIC_I, FW_PIC_P, FW_PIC_B };

#define FW_FOURCC(a, b, c, d)	((a) | ((b) << 8) | ((c) << 16) | ((d) << 24))
#define FW_NOTI_DECO		FW_FOURCC('D', 'E', 'C', 'O')
#define FW_NOTI_ENDD		FW_FOURCC('E', 'N', 'D', 'D')

#define FW_GET_NOTI_INFO	_IOR(FW_IOC_TYPE, 11, struct fw_buffer_info)
#define FW_SET_NOTI_TAIL	_IO(FW_IOC_TYPE, 13)
#define FW_GET_ES_BUFFER	_IOWR(FW_IOC_TYPE, 9, struct fw_es_data)
#define FW_PUT_ES_BUFFER	_IOW(FW_IOC_TYPE, 10, struct fw_es_data)
#define FW_UNREF_DECODEBUFFER	_IOW(FW_IOC_TYPE, 17, u32)
#define FW_SET_OUTPUT_MEMFMT	_IOW(FW_IOC_TYPE, 20, s32)
#define FW_SET_CODEC		_IO(FW_IOC_TYPE, 23)
#define FW_SET_INPUT_DEVICE	_IO(FW_IOC_TYPE, 25)
#define FW_SET_OUTPUT_DEVICE	_IO(FW_IOC_TYPE, 26)
#define FW_GET_CPB_INFO		_IOR(FW_IOC_TYPE, 33, struct fw_buffer_info)
#define FW_INIT			_IO(FW_IOC_TYPE, 45)
#define FW_EOS			_IO(FW_IOC_TYPE, 47)

#define FW_CODEC_H264		0
#define FW_SRC_BUFF		3
#define FW_DST_BUFF		8
#define FW_MEMFMT_RASTER	0
#define FW_AU_DATA		3

/* Readiness bits the firmware reports. */
#define FW_POLLIN		0x1

/* ---- driver state ---- */

struct lg1314_vdec_dec {
	struct lg1314_vdec	*v;
	struct device		*dev;
	struct v4l2_device	v4l2_dev;
	struct video_device	vfd;
	struct v4l2_m2m_dev	*m2m_dev;
	struct mutex		lock;		/* ioctls and vb2 queues */
	struct workqueue_struct	*wq;
	spinlock_t		ctxs_lock;
	struct list_head	ctxs;
};

struct dec_frame {
	struct list_head	list;
	struct fw_decoded	d;
};

struct dec_ctx {
	struct v4l2_fh		fh;
	struct lg1314_vdec_dec	*dec;
	struct list_head	node;		/* in dec->ctxs */
	struct v4l2_ctrl_handler ctrls;

	struct v4l2_pix_format_mplane out_fmt;
	struct v4l2_pix_format_mplane cap_fmt;
	struct v4l2_rect	visible;
	bool			have_geometry;	/* cap_fmt is the stream's */
	bool			src_change;	/* waiting for CAPTURE setup */

	/* Firmware session; fw_lock protects it and everything below. */
	struct mutex		fw_lock;
	u32			handle;		/* 0: none */
	void			*noti;
	u32			noti_size;
	void			*cpb;
	u32			cpb_size;
	struct list_head	pending;	/* decoded, not yet returned */
	unsigned int		in_flight;
	bool			cpb_full;
	bool			draining;	/* STOP received */
	bool			eos_sent;
	bool			eos_reported;	/* last picture reported */
	u32			cap_seq;
	u32			out_seq;
	u64			uid;

	struct work_struct	noti_work;
};

static inline struct dec_ctx *file_to_ctx(struct file *file)
{
	return container_of(file->private_data, struct dec_ctx, fh);
}

/* ---- H.264 sequence parameter set (stream geometry) ---- */

struct h264_bits {
	const u8	*buf;
	size_t		len;		/* bytes, emulation prevention removed */
	size_t		pos;		/* bit position */
};

static int h264_bit(struct h264_bits *b)
{
	int v;

	if (b->pos >= b->len * 8)
		return -1;
	v = (b->buf[b->pos / 8] >> (7 - b->pos % 8)) & 1;
	b->pos++;
	return v;
}

static int h264_bits(struct h264_bits *b, unsigned int n, u32 *out)
{
	u32 v = 0;
	int bit;

	while (n--) {
		bit = h264_bit(b);
		if (bit < 0)
			return -EINVAL;
		v = (v << 1) | bit;
	}
	*out = v;
	return 0;
}

/* Exp-Golomb ue(v). */
static int h264_ue(struct h264_bits *b, u32 *out)
{
	unsigned int zeros = 0;
	u32 rest;
	int bit;

	while ((bit = h264_bit(b)) == 0)
		if (++zeros > 31)
			return -EINVAL;
	if (bit < 0 || h264_bits(b, zeros, &rest))
		return -EINVAL;
	*out = (1u << zeros) - 1 + rest;
	return 0;
}

static int h264_se(struct h264_bits *b, s32 *out)
{
	u32 v;

	if (h264_ue(b, &v))
		return -EINVAL;
	*out = (v & 1) ? (s32)((v + 1) / 2) : -(s32)(v / 2);
	return 0;
}

static int h264_skip_scaling_list(struct h264_bits *b, unsigned int size)
{
	s32 last = 8, next = 8, delta;
	unsigned int i;

	for (i = 0; i < size; i++) {
		if (next) {
			if (h264_se(b, &delta))
				return -EINVAL;
			next = (last + delta + 256) % 256;
		}
		last = next ?: last;
	}
	return 0;
}

struct h264_geometry {
	u32 width, height;		/* coded */
	struct v4l2_rect visible;
};

/* Parse an SPS NAL payload (after the NAL header byte). */
static int h264_parse_sps(const u8 *rbsp, size_t len, struct h264_geometry *g)
{
	struct h264_bits b = { .buf = rbsp, .len = len };
	u32 profile, v, chroma = 1, poc_type, w_mbs, h_units, frame_mbs_only;
	u32 crop[4] = { 0 }, cx, cy;
	unsigned int i, n;
	s32 sv;

	if (h264_bits(&b, 8, &profile) || h264_bits(&b, 16, &v) || h264_ue(&b, &v))
		return -EINVAL;
	switch (profile) {
	case 100: case 110: case 122: case 244: case 44: case 83: case 86:
	case 118: case 128: case 138: case 139: case 134: case 135:
		if (h264_ue(&b, &chroma))
			return -EINVAL;
		if (chroma == 3 && h264_bits(&b, 1, &v))
			return -EINVAL;
		if (h264_ue(&b, &v) || h264_ue(&b, &v) || h264_bits(&b, 1, &v) ||
		    h264_bits(&b, 1, &v))
			return -EINVAL;
		if (v) {	/* seq_scaling_matrix_present_flag */
			n = chroma == 3 ? 12 : 8;
			for (i = 0; i < n; i++) {
				if (h264_bits(&b, 1, &v))
					return -EINVAL;
				if (v && h264_skip_scaling_list(&b, i < 6 ? 16 : 64))
					return -EINVAL;
			}
		}
		break;
	}
	if (h264_ue(&b, &v) || h264_ue(&b, &poc_type))	/* log2_max_frame_num */
		return -EINVAL;
	if (poc_type == 0) {
		if (h264_ue(&b, &v))
			return -EINVAL;
	} else if (poc_type == 1) {
		if (h264_bits(&b, 1, &v) || h264_se(&b, &sv) || h264_se(&b, &sv) ||
		    h264_ue(&b, &n))
			return -EINVAL;
		for (i = 0; i < n; i++)
			if (h264_se(&b, &sv))
				return -EINVAL;
	}
	if (h264_ue(&b, &v) || h264_bits(&b, 1, &v) ||	/* max_num_ref_frames, gaps */
	    h264_ue(&b, &w_mbs) || h264_ue(&b, &h_units) ||
	    h264_bits(&b, 1, &frame_mbs_only))
		return -EINVAL;
	if (!frame_mbs_only && h264_bits(&b, 1, &v))
		return -EINVAL;
	if (h264_bits(&b, 1, &v) || h264_bits(&b, 1, &v))	/* direct_8x8, cropping */
		return -EINVAL;
	if (v)
		for (i = 0; i < 4; i++)
			if (h264_ue(&b, &crop[i]))
				return -EINVAL;

	g->width = (w_mbs + 1) * 16;
	g->height = (h_units + 1) * 16 * (2 - frame_mbs_only);
	cx = chroma == 0 || chroma == 3 ? 1 : 2;
	cy = (chroma == 1 ? 2 : 1) * (2 - frame_mbs_only);
	g->visible.left = crop[0] * cx;
	g->visible.top = crop[2] * cy;
	if ((crop[0] + crop[1]) * cx >= g->width ||
	    (crop[2] + crop[3]) * cy >= g->height)
		return -EINVAL;
	g->visible.width = g->width - (crop[0] + crop[1]) * cx;
	g->visible.height = g->height - (crop[2] + crop[3]) * cy;
	return 0;
}

/* Find the first SPS in an Annex-B access unit and parse it. */
static int h264_find_sps(const u8 *data, size_t size, struct h264_geometry *g)
{
	size_t i, start, end, n;
	u8 *rbsp;
	int ret;

	for (i = 0; i + 3 < size; i++) {
		if (data[i] || data[i + 1] || data[i + 2] != 1 ||
		    (data[i + 3] & 0x1f) != 7)
			continue;
		start = i + 4;
		for (end = start; end + 2 < size; end++)
			if (!data[end] && !data[end + 1] && data[end + 2] <= 1)
				break;
		if (end + 2 >= size)
			end = size;
		/* Strip emulation prevention bytes (00 00 03). */
		rbsp = kmalloc(end - start, GFP_KERNEL);
		if (!rbsp)
			return -ENOMEM;
		for (n = 0, i = start; i < end; i++) {
			if (i >= start + 2 && data[i] == 3 && !data[i - 1] && !data[i - 2])
				continue;
			rbsp[n++] = data[i];
		}
		ret = h264_parse_sps(rbsp, n, g);
		kfree(rbsp);
		return ret;
	}
	return -ENOENT;
}

/* ---- firmware calls ---- */

/* An ioctl on the instance's firmware decoder; returns the firmware's result. */
static int dec_fw_ioctl(struct dec_ctx *ctx, u32 cmd, u32 arg, void *data)
{
	u32 size = _IOC_SIZE(cmd);
	unsigned int dir = VDEC_CALL_WR;
	int ret, result;
	u32 *buf;

	buf = kzalloc(2 * sizeof(u32) + size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	buf[0] = cmd;
	buf[1] = arg;
	if (size && (_IOC_DIR(cmd) & _IOC_WRITE))
		memcpy(buf + 2, data, size);
	if (size && (_IOC_DIR(cmd) & _IOC_READ))
		dir |= VDEC_CALL_RD;

	ret = lg1314_vdec_call(ctx->dec->v, VDEC_CALL_VDEC_IOCTL, ctx->handle,
			       buf, 2 * sizeof(u32) + size, dir, &result);
	if (!ret && size && (_IOC_DIR(cmd) & _IOC_READ))
		memcpy(data, buf + 2, size);
	kfree(buf);
	return ret ?: result;
}

static void dec_session_close(struct dec_ctx *ctx)
{
	struct dec_frame *f, *tmp;
	int result;

	if (!ctx->handle)
		return;
	list_for_each_entry_safe(f, tmp, &ctx->pending, list) {
		list_del(&f->list);
		kfree(f);
	}
	if (ctx->noti)
		memunmap(ctx->noti);
	if (ctx->cpb)
		memunmap(ctx->cpb);
	ctx->noti = ctx->cpb = NULL;
	lg1314_vdec_call(ctx->dec->v, VDEC_CALL_VDEC_CLOSE, ctx->handle, NULL, 0,
			 0, &result);
	dev_dbg(ctx->dec->dev, "ctx %p: session %#x closed\n", ctx, ctx->handle);
	ctx->handle = 0;
}

/* Map a firmware ring (physical address) for the CPU. */
static void *dec_map_ring(struct dec_ctx *ctx, phys_addr_t phys, u32 size)
{
	if (!phys || !size || !lg1314_vdec_in_carveout(ctx->dec->v, phys, size))
		return NULL;
	return memremap(phys, size, MEMREMAP_WC);
}

static int dec_session_open(struct dec_ctx *ctx)
{
	struct lg1314_vdec *v = ctx->dec->v;
	struct fw_buffer_info info;
	s32 memfmt = FW_MEMFMT_RASTER;
	int ret, handle;

	ret = lg1314_vdec_call(v, VDEC_CALL_VDEC_OPEN, 0, NULL, 0, 0, &handle);
	if (ret)
		return ret;
	if (!handle)
		return -EBUSY;
	ctx->handle = handle;

	ret = dec_fw_ioctl(ctx, FW_SET_CODEC, FW_CODEC_H264, NULL);
	if (!ret)
		ret = dec_fw_ioctl(ctx, FW_SET_INPUT_DEVICE, FW_SRC_BUFF, NULL);
	if (!ret)
		ret = dec_fw_ioctl(ctx, FW_SET_OUTPUT_DEVICE, FW_DST_BUFF, NULL);
	if (!ret)
		ret = dec_fw_ioctl(ctx, FW_SET_OUTPUT_MEMFMT, 0, &memfmt);
	if (!ret)
		ret = dec_fw_ioctl(ctx, FW_INIT, 0, NULL);
	if (ret)
		goto fail;

	ret = dec_fw_ioctl(ctx, FW_GET_NOTI_INFO, 0, &info);
	if (ret)
		goto fail;
	ctx->noti_size = info.size;
	ctx->noti = dec_map_ring(ctx, lg1314_vdec_copro_to_phys(v, info.addr),
				 info.size);
	ret = dec_fw_ioctl(ctx, FW_GET_CPB_INFO, 0, &info);
	if (ret)
		goto fail;
	ctx->cpb_size = info.size;
	ctx->cpb = dec_map_ring(ctx, info.addr, info.size);
	if (!ctx->noti || !ctx->cpb) {
		ret = -ENOMEM;
		goto fail;
	}

	dev_dbg(ctx->dec->dev, "ctx %p: session %#x open\n", ctx, ctx->handle);
	ctx->in_flight = 0;
	ctx->cpb_full = false;
	ctx->draining = ctx->eos_sent = ctx->eos_reported = false;
	return 0;

fail:
	dev_err(ctx->dec->dev, "decoder setup failed: %d\n", ret);
	dec_session_close(ctx);
	/* The firmware fails setup when it is out of decoder memory (stream
	 * buffers for two decoders at a time). */
	return ret == -ENOMEM || ret == -ETIMEDOUT ? ret : -EBUSY;
}

/* ---- pictures out ---- */

static void dec_copy_plane(void *dst, u32 dst_stride, const void *src,
			   u32 src_stride, u32 width, u32 rows)
{
	if (dst_stride == src_stride) {
		memcpy(dst, src, (size_t)src_stride * rows);
		return;
	}
	while (rows--) {
		memcpy(dst, src, width);
		dst += dst_stride;
		src += src_stride;
	}
}

/* Visible size of a decoded picture. */
static u32 dec_vis_w(const struct fw_decoded *d)
{
	return d->width - d->crop_left - d->crop_right;
}

static u32 dec_vis_h(const struct fw_decoded *d)
{
	return d->height - d->crop_top - d->crop_bottom;
}

/*
 * Copy the visible part of a decoded picture into a CAPTURE buffer (NV12):
 * luma at addr_y and interleaved chroma at addr_cb, both in the coded layout,
 * offset by the crop.
 */
static int dec_copy_frame(struct dec_ctx *ctx, const struct fw_decoded *d,
			  struct vb2_v4l2_buffer *dst)
{
	struct lg1314_vdec *v = ctx->dec->v;
	u32 bpl = ctx->cap_fmt.plane_fmt[0].bytesperline;
	u32 h = ctx->cap_fmt.height;
	u32 w = min_t(u32, dec_vis_w(d), ctx->cap_fmt.width);
	u32 left = d->crop_left & ~1u, top = d->crop_top & ~1u;
	const void *y, *c;
	void *out;

	y = lg1314_vdec_frame_map(v, d->addr_y + (phys_addr_t)d->stride * top + left,
				  (size_t)d->stride * h);
	c = lg1314_vdec_frame_map(v, d->addr_cb + (phys_addr_t)d->stride * (top / 2) +
				  left, (size_t)d->stride * h / 2);
	out = vb2_plane_vaddr(&dst->vb2_buf, 0);
	if (!y || !c || !out)
		return -EFAULT;

	dec_copy_plane(out, bpl, y, d->stride, w, h);
	dec_copy_plane(out + bpl * h, bpl, c, d->stride, w, h / 2);
	vb2_set_plane_payload(&dst->vb2_buf, 0, ctx->cap_fmt.plane_fmt[0].sizeimage);
	return 0;
}

static void dec_set_cap_fmt(struct dec_ctx *ctx, u32 width, u32 height)
{
	struct v4l2_pix_format_mplane *f = &ctx->cap_fmt;
	u32 bpl = ALIGN(width, 64);

	f->pixelformat = V4L2_PIX_FMT_NV12;
	f->width = width;
	f->height = ALIGN(height, 2);
	f->field = V4L2_FIELD_NONE;
	f->num_planes = 1;
	f->plane_fmt[0].bytesperline = bpl;
	f->plane_fmt[0].sizeimage = bpl * f->height * 3 / 2;
	f->colorspace = ctx->out_fmt.colorspace;
	f->xfer_func = ctx->out_fmt.xfer_func;
	f->ycbcr_enc = ctx->out_fmt.ycbcr_enc;
	f->quantization = ctx->out_fmt.quantization;
}

static bool dec_geometry_matches(struct dec_ctx *ctx, const struct fw_decoded *d)
{
	return ctx->have_geometry && dec_vis_w(d) == ctx->cap_fmt.width &&
	       ALIGN(dec_vis_h(d), 2) == ctx->cap_fmt.height;
}

/* A new stream geometry (the visible picture size): adopt it and tell
 * userspace. Pictures come out cropped, so CAPTURE is exactly that size. */
static void dec_signal_geometry(struct dec_ctx *ctx, u32 width, u32 height)
{
	static const struct v4l2_event ev = {
		.type = V4L2_EVENT_SOURCE_CHANGE,
		.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
	};

	dec_set_cap_fmt(ctx, width, height);
	ctx->visible = (struct v4l2_rect){ 0, 0, width, ALIGN(height, 2) };
	ctx->have_geometry = true;
	ctx->src_change = true;
	v4l2_event_queue_fh(&ctx->fh, &ev);
}

/* A picture whose geometry the headers didn't announce: adopt it. */
static void dec_source_change(struct dec_ctx *ctx, const struct fw_decoded *d)
{
	dec_signal_geometry(ctx, dec_vis_w(d), dec_vis_h(d));
}

static void dec_signal_eos(struct dec_ctx *ctx)
{
	static const struct v4l2_event ev = { .type = V4L2_EVENT_EOS };

	v4l2_event_queue_fh(&ctx->fh, &ev);
}

/* Return pending pictures (and the end of stream) to CAPTURE. fw_lock held. */
static void dec_deliver(struct dec_ctx *ctx)
{
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	struct vb2_v4l2_buffer *dst;
	struct dec_frame *f;

	while ((f = list_first_entry_or_null(&ctx->pending, struct dec_frame, list))) {
		if (!dec_geometry_matches(ctx, &f->d)) {
			if (!ctx->src_change)
				dec_source_change(ctx, &f->d);
			return;
		}
		if (ctx->src_change || !vb2_is_streaming(v4l2_m2m_get_dst_vq(m2m_ctx)))
			return;
		dst = v4l2_m2m_dst_buf_remove(m2m_ctx);
		if (!dst)
			return;

		if (dec_copy_frame(ctx, &f->d, dst)) {
			vb2_set_plane_payload(&dst->vb2_buf, 0, 0);
			v4l2_m2m_buf_done(dst, VB2_BUF_STATE_ERROR);
		} else {
			dst->vb2_buf.timestamp = f->d.timestamp;
			dst->field = V4L2_FIELD_NONE;
			dst->sequence = ctx->cap_seq++;
			dst->flags &= ~(V4L2_BUF_FLAG_KEYFRAME | V4L2_BUF_FLAG_PFRAME |
					V4L2_BUF_FLAG_BFRAME);
			dst->flags |= f->d.picture_type == FW_PIC_I ? V4L2_BUF_FLAG_KEYFRAME :
				      f->d.picture_type == FW_PIC_P ? V4L2_BUF_FLAG_PFRAME :
				      V4L2_BUF_FLAG_BFRAME;
			v4l2_m2m_buf_done(dst, f->d.report & FW_REPORT_ERROR_MASK ?
					  VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);
		}
		dec_fw_ioctl(ctx, FW_UNREF_DECODEBUFFER, 0, &f->d.addr_y);
		list_del(&f->list);
		kfree(f);
	}

	if (ctx->eos_reported && !ctx->src_change &&
	    vb2_is_streaming(v4l2_m2m_get_dst_vq(m2m_ctx))) {
		dst = v4l2_m2m_dst_buf_remove(m2m_ctx);
		if (dst) {
			ctx->eos_reported = false;
			ctx->draining = false;
			v4l2_m2m_last_buffer_done(m2m_ctx, dst);
			dec_signal_eos(ctx);
		}
	}
}

/* Read len bytes of the notification ring at off (wrapping). */
static void dec_noti_read(struct dec_ctx *ctx, u32 off, void *out, u32 len)
{
	u32 first;

	off %= ctx->noti_size;
	first = min(len, ctx->noti_size - off);
	memcpy(out, ctx->noti + off, first);
	if (len > first)
		memcpy(out + first, ctx->noti, len - first);
}

/* Consume the notification ring. fw_lock held. Returns true if the firmware
 * may have room for more input. */
static bool dec_drain_noti(struct dec_ctx *ctx)
{
	struct fw_buffer_info info;
	bool progressed = false;
	u32 tail, head, hdr[2];

	for (;;) {
		if (dec_fw_ioctl(ctx, FW_GET_NOTI_INFO, 0, &info))
			break;
		tail = info.read_offset;
		head = info.write_offset;
		if (tail == head || info.size != ctx->noti_size)
			break;

		while (tail != head) {
			u32 len;

			dec_noti_read(ctx, tail, hdr, sizeof(hdr));
			len = hdr[0] - 4;	/* payload */
			if (hdr[1] == FW_NOTI_ENDD) {
				/* one access unit consumed */
				if (ctx->in_flight)
					ctx->in_flight--;
				progressed = true;
			} else if (hdr[1] == FW_NOTI_DECO &&
				   len == sizeof(struct fw_decoded)) {
				struct dec_frame *f = kzalloc(sizeof(*f), GFP_KERNEL);

				if (f) {
					dec_noti_read(ctx, tail + 8, &f->d, len);
					if (f->d.addr_y) {
						list_add_tail(&f->list, &ctx->pending);
					} else {
						if (f->d.report & FW_REPORT_EOS)
							ctx->eos_reported = true;
						kfree(f);
					}
				}
				progressed = true;
			}
			tail = (tail + 8 + ALIGN(len, 4)) % ctx->noti_size;
		}
		dec_fw_ioctl(ctx, FW_SET_NOTI_TAIL, tail, NULL);
	}
	return progressed;
}

static void dec_noti_work(struct work_struct *work)
{
	struct dec_ctx *ctx = container_of(work, struct dec_ctx, noti_work);
	bool room = false;

	mutex_lock(&ctx->fw_lock);
	if (ctx->handle) {
		room = dec_drain_noti(ctx);
		dec_deliver(ctx);
		if (room)
			ctx->cpb_full = false;
	}
	mutex_unlock(&ctx->fw_lock);
	if (room)
		v4l2_m2m_try_schedule(ctx->fh.m2m_ctx);
}

void lg1314_vdec_dec_poll(struct lg1314_vdec_dec *dec, u32 handle, u32 mask)
{
	struct dec_ctx *ctx;
	unsigned long flags;

	if (!dec || !(mask & FW_POLLIN))
		return;
	spin_lock_irqsave(&dec->ctxs_lock, flags);
	list_for_each_entry(ctx, &dec->ctxs, node)
		if (ctx->handle == handle)
			queue_work(dec->wq, &ctx->noti_work);
	spin_unlock_irqrestore(&dec->ctxs_lock, flags);
}

/* ---- access units in ---- */

/* Free space in the CPB, or 0 if it can't be determined. */
static u32 dec_cpb_free(struct dec_ctx *ctx)
{
	struct fw_buffer_info info;
	u32 used;

	if (dec_fw_ioctl(ctx, FW_GET_CPB_INFO, 0, &info) || !info.size)
		return 0;
	used = (info.write_offset - info.read_offset + info.size) % info.size;
	return info.size - used;
}

/* Copy one access unit into the CPB and hand it to the firmware. fw_lock held. */
static int dec_feed(struct dec_ctx *ctx, struct vb2_v4l2_buffer *src)
{
	const void *data = vb2_plane_vaddr(&src->vb2_buf, 0);
	u32 size = vb2_get_plane_payload(&src->vb2_buf, 0);
	struct fw_es_data es = { .size = size };
	struct h264_geometry g;
	int ret;

	dev_dbg(ctx->dec->dev, "ctx %p: feed %u bytes, starts %*ph\n",
		ctx, size, 5, data);

	/* Announce the geometry as soon as the headers give it. */
	if (!h264_find_sps(data, size, &g) &&
	    (!ctx->have_geometry || g.visible.width != ctx->cap_fmt.width ||
	     ALIGN(g.visible.height, 2) != ctx->cap_fmt.height))
		dec_signal_geometry(ctx, g.visible.width, g.visible.height);

	ret = dec_fw_ioctl(ctx, FW_GET_ES_BUFFER, 0, &es);
	if (ret)
		return ret;
	if (es.buf1_offset + es.buf1_size > ctx->cpb_size ||
	    es.buf2_offset + es.buf2_size > ctx->cpb_size ||
	    es.buf1_size + es.buf2_size < size)
		return -EIO;
	memcpy(ctx->cpb + es.buf1_offset, data, min(size, es.buf1_size));
	if (size > es.buf1_size)
		memcpy(ctx->cpb + es.buf2_offset, data + es.buf1_size,
		       size - es.buf1_size);
	wmb();

	es.timestamp = src->vb2_buf.timestamp;
	es.uid = ctx->uid++;
	es.size = size;
	es.au_type = FW_AU_DATA;
	es.frame_count = 1;
	return dec_fw_ioctl(ctx, FW_PUT_ES_BUFFER, 0, &es);
}

static void dec_device_run(void *priv)
{
	struct dec_ctx *ctx = priv;
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	struct vb2_v4l2_buffer *src;
	u32 size;
	int ret;

	mutex_lock(&ctx->fw_lock);
	src = v4l2_m2m_next_src_buf(m2m_ctx);
	if (!ctx->handle || !src)
		goto out;

	size = vb2_get_plane_payload(&src->vb2_buf, 0);
	if (size) {
		if (ctx->in_flight >= DEC_MAX_IN_FLIGHT ||
		    dec_cpb_free(ctx) < size + DEC_CPB_MARGIN) {
			ctx->cpb_full = true;
			goto out;
		}
		ret = dec_feed(ctx, src);
	} else {
		ret = 0;
	}

	/* The data is in the CPB now: the buffer is consumed. */
	src = v4l2_m2m_src_buf_remove(m2m_ctx);
	src->sequence = ctx->out_seq++;
	v4l2_m2m_buf_done(src, ret ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);
	if (!ret && size)
		ctx->in_flight++;

	/* Draining and that was the last input: end the stream. */
	if (ctx->draining && !ctx->eos_sent && !v4l2_m2m_num_src_bufs_ready(m2m_ctx)) {
		dec_fw_ioctl(ctx, FW_EOS, 0, NULL);
		ctx->eos_sent = true;
	}
out:
	mutex_unlock(&ctx->fw_lock);
	v4l2_m2m_job_finish(ctx->dec->m2m_dev, m2m_ctx);
}

static int dec_job_ready(void *priv)
{
	struct dec_ctx *ctx = priv;

	return READ_ONCE(ctx->handle) && !READ_ONCE(ctx->cpb_full);
}

static const struct v4l2_m2m_ops dec_m2m_ops = {
	.device_run	= dec_device_run,
	.job_ready	= dec_job_ready,
};

/* ---- vb2 ---- */

static int dec_queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
			   unsigned int *nplanes, unsigned int sizes[],
			   struct device *alloc_devs[])
{
	struct dec_ctx *ctx = vb2_get_drv_priv(vq);
	struct v4l2_pix_format_mplane *f = V4L2_TYPE_IS_OUTPUT(vq->type) ?
					   &ctx->out_fmt : &ctx->cap_fmt;

	if (*nplanes)
		return *nplanes != 1 || sizes[0] < f->plane_fmt[0].sizeimage ?
		       -EINVAL : 0;
	*nplanes = 1;
	sizes[0] = f->plane_fmt[0].sizeimage;
	if (V4L2_TYPE_IS_CAPTURE(vq->type))
		*nbuffers = max_t(unsigned int, *nbuffers, DEC_CAP_MIN_BUFS);
	return 0;
}

static int dec_buf_out_validate(struct vb2_buffer *vb)
{
	to_vb2_v4l2_buffer(vb)->field = V4L2_FIELD_NONE;
	return 0;
}

static int dec_buf_prepare(struct vb2_buffer *vb)
{
	struct dec_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	if (V4L2_TYPE_IS_CAPTURE(vb->type)) {
		if (vb2_plane_size(vb, 0) < ctx->cap_fmt.plane_fmt[0].sizeimage)
			return -EINVAL;
	} else if (vb2_get_plane_payload(vb, 0) > vb2_plane_size(vb, 0)) {
		return -EINVAL;
	}
	return 0;
}

static void dec_buf_queue(struct vb2_buffer *vb)
{
	struct dec_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, to_vb2_v4l2_buffer(vb));
	if (V4L2_TYPE_IS_CAPTURE(vb->type))
		queue_work(ctx->dec->wq, &ctx->noti_work);	/* deliver */
}

static void dec_return_bufs(struct dec_ctx *ctx, bool output,
			    enum vb2_buffer_state state)
{
	struct vb2_v4l2_buffer *vb;

	while ((vb = output ? v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx) :
			     v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx)))
		v4l2_m2m_buf_done(vb, state);
}

static int dec_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct dec_ctx *ctx = vb2_get_drv_priv(vq);
	int ret = 0;

	mutex_lock(&ctx->fw_lock);
	if (V4L2_TYPE_IS_OUTPUT(vq->type)) {
		ctx->uid = 0;
		ctx->out_seq = 0;
		ret = dec_session_open(ctx);
	} else {
		/* CAPTURE (re)configured for the current geometry. */
		ctx->cap_seq = 0;
		ctx->src_change = false;
	}
	mutex_unlock(&ctx->fw_lock);

	if (ret)	/* vb2 wants the buffers back as queued */
		dec_return_bufs(ctx, V4L2_TYPE_IS_OUTPUT(vq->type),
				VB2_BUF_STATE_QUEUED);
	else
		queue_work(ctx->dec->wq, &ctx->noti_work);
	return ret;
}

static void dec_stop_streaming(struct vb2_queue *vq)
{
	struct dec_ctx *ctx = vb2_get_drv_priv(vq);

	if (V4L2_TYPE_IS_OUTPUT(vq->type)) {
		cancel_work_sync(&ctx->noti_work);
		mutex_lock(&ctx->fw_lock);
		dec_session_close(ctx);
		mutex_unlock(&ctx->fw_lock);
		v4l2_m2m_clear_state(ctx->fh.m2m_ctx);
	}
	dec_return_bufs(ctx, V4L2_TYPE_IS_OUTPUT(vq->type), VB2_BUF_STATE_ERROR);
}

static const struct vb2_ops dec_vb2_ops = {
	.queue_setup		= dec_queue_setup,
	.buf_out_validate	= dec_buf_out_validate,
	.buf_prepare		= dec_buf_prepare,
	.buf_queue		= dec_buf_queue,
	.start_streaming	= dec_start_streaming,
	.stop_streaming		= dec_stop_streaming,
};

static int dec_queue_init(void *priv, struct vb2_queue *src_vq,
			  struct vb2_queue *dst_vq)
{
	struct dec_ctx *ctx = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->ops = &dec_vb2_ops;
	src_vq->mem_ops = &vb2_vmalloc_memops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &ctx->dec->lock;
	src_vq->dev = ctx->dec->dev;
	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->ops = &dec_vb2_ops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &ctx->dec->lock;
	dst_vq->dev = ctx->dec->dev;
	return vb2_queue_init(dst_vq);
}

/* ---- ioctls ---- */

static int dec_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	strscpy(cap->driver, DEC_NAME, sizeof(cap->driver));
	strscpy(cap->card, "LG1314 video decoder", sizeof(cap->card));
	return 0;
}

static int dec_enum_fmt(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;
	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		f->pixelformat = V4L2_PIX_FMT_H264;
		f->flags = V4L2_FMT_FLAG_COMPRESSED | V4L2_FMT_FLAG_DYN_RESOLUTION;
	} else {
		f->pixelformat = V4L2_PIX_FMT_NV12;
	}
	return 0;
}

static int dec_enum_framesizes(struct file *file, void *priv,
			       struct v4l2_frmsizeenum *fs)
{
	if (fs->index || fs->pixel_format != V4L2_PIX_FMT_H264)
		return -EINVAL;
	fs->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fs->stepwise = (struct v4l2_frmsize_stepwise){
		DEC_MIN_W, DEC_MAX_W, 2, DEC_MIN_H, DEC_MAX_H, 2,
	};
	return 0;
}

static int dec_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct dec_ctx *ctx = file_to_ctx(file);

	f->fmt.pix_mp = V4L2_TYPE_IS_OUTPUT(f->type) ? ctx->out_fmt : ctx->cap_fmt;
	return 0;
}

static void dec_adjust_out_fmt(struct v4l2_pix_format_mplane *f)
{
	f->pixelformat = V4L2_PIX_FMT_H264;
	f->width = clamp(f->width, (u32)DEC_MIN_W, (u32)DEC_MAX_W);
	f->height = clamp(f->height, (u32)DEC_MIN_H, (u32)DEC_MAX_H);
	f->field = V4L2_FIELD_NONE;
	f->num_planes = 1;
	f->plane_fmt[0].bytesperline = 0;
	f->plane_fmt[0].sizeimage = max_t(u32, f->plane_fmt[0].sizeimage,
					  max_t(u32, DEC_ES_MIN_SIZE,
						f->width * f->height * 3 / 4));
}

static int dec_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct dec_ctx *ctx = file_to_ctx(file);

	if (V4L2_TYPE_IS_OUTPUT(f->type))
		dec_adjust_out_fmt(&f->fmt.pix_mp);
	else
		f->fmt.pix_mp = ctx->cap_fmt;	/* fixed by the stream */
	return 0;
}

static int dec_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct dec_ctx *ctx = file_to_ctx(file);
	struct vb2_queue *vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);

	if (vb2_is_busy(vq))
		return -EBUSY;
	dec_try_fmt(file, priv, f);
	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		ctx->out_fmt = f->fmt.pix_mp;
		if (!ctx->have_geometry) {
			dec_set_cap_fmt(ctx, ctx->out_fmt.width, ctx->out_fmt.height);
			ctx->visible = (struct v4l2_rect){ 0, 0, ctx->out_fmt.width,
							   ctx->out_fmt.height };
		}
	}
	return 0;
}

static int dec_g_selection(struct file *file, void *priv,
			   struct v4l2_selection *s)
{
	struct dec_ctx *ctx = file_to_ctx(file);

	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	switch (s->target) {
	case V4L2_SEL_TGT_COMPOSE:
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		s->r = ctx->visible;
		return 0;
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		s->r = (struct v4l2_rect){ 0, 0, ctx->cap_fmt.width, ctx->cap_fmt.height };
		return 0;
	}
	return -EINVAL;
}

static int dec_decoder_cmd(struct file *file, void *priv,
			   struct v4l2_decoder_cmd *dc)
{
	struct dec_ctx *ctx = file_to_ctx(file);
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	int ret;

	ret = v4l2_m2m_ioctl_try_decoder_cmd(file, priv, dc);
	if (ret)
		return ret;

	if (dc->cmd == V4L2_DEC_CMD_START) {
		v4l2_m2m_clear_state(m2m_ctx);
		vb2_clear_last_buffer_dequeued(v4l2_m2m_get_dst_vq(m2m_ctx));
		mutex_lock(&ctx->fw_lock);
		ctx->draining = ctx->eos_sent = false;
		mutex_unlock(&ctx->fw_lock);
		v4l2_m2m_try_schedule(m2m_ctx);
		return 0;
	}

	/* STOP: end of stream after the queued input is decoded. */
	if (!vb2_is_streaming(v4l2_m2m_get_src_vq(m2m_ctx)))
		return 0;
	mutex_lock(&ctx->fw_lock);
	if (!ctx->draining && ctx->handle) {
		ctx->draining = true;
		if (!v4l2_m2m_num_src_bufs_ready(m2m_ctx)) {
			dec_fw_ioctl(ctx, FW_EOS, 0, NULL);
			ctx->eos_sent = true;
		}
	}
	mutex_unlock(&ctx->fw_lock);
	return 0;
}

static int dec_subscribe_event(struct v4l2_fh *fh,
			       const struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_EOS:
		return v4l2_event_subscribe(fh, sub, 0, NULL);
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subscribe(fh, sub);
	}
	return v4l2_ctrl_subscribe_event(fh, sub);
}

static const struct v4l2_ioctl_ops dec_ioctl_ops = {
	.vidioc_querycap		= dec_querycap,
	.vidioc_enum_fmt_vid_cap	= dec_enum_fmt,
	.vidioc_enum_fmt_vid_out	= dec_enum_fmt,
	.vidioc_enum_framesizes		= dec_enum_framesizes,
	.vidioc_g_fmt_vid_cap_mplane	= dec_g_fmt,
	.vidioc_g_fmt_vid_out_mplane	= dec_g_fmt,
	.vidioc_try_fmt_vid_cap_mplane	= dec_try_fmt,
	.vidioc_try_fmt_vid_out_mplane	= dec_try_fmt,
	.vidioc_s_fmt_vid_cap_mplane	= dec_s_fmt,
	.vidioc_s_fmt_vid_out_mplane	= dec_s_fmt,
	.vidioc_g_selection		= dec_g_selection,

	.vidioc_reqbufs			= v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf		= v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf			= v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf			= v4l2_m2m_ioctl_dqbuf,
	.vidioc_prepare_buf		= v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs		= v4l2_m2m_ioctl_create_bufs,
	.vidioc_expbuf			= v4l2_m2m_ioctl_expbuf,
	.vidioc_streamon		= v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff		= v4l2_m2m_ioctl_streamoff,

	.vidioc_try_decoder_cmd		= v4l2_m2m_ioctl_try_decoder_cmd,
	.vidioc_decoder_cmd		= dec_decoder_cmd,

	.vidioc_subscribe_event		= dec_subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,
};

/* ---- file ops ---- */

static int dec_open(struct file *file)
{
	struct lg1314_vdec_dec *dec = video_drvdata(file);
	struct dec_ctx *ctx;
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ctx->dec = dec;
	mutex_init(&ctx->fw_lock);
	INIT_LIST_HEAD(&ctx->pending);
	INIT_WORK(&ctx->noti_work, dec_noti_work);
	v4l2_fh_init(&ctx->fh, video_devdata(file));
	file->private_data = &ctx->fh;

	ctx->out_fmt.width = DEC_DEF_W;
	ctx->out_fmt.height = DEC_DEF_H;
	ctx->out_fmt.colorspace = V4L2_COLORSPACE_REC709;
	dec_adjust_out_fmt(&ctx->out_fmt);
	dec_set_cap_fmt(ctx, DEC_DEF_W, DEC_DEF_H);
	ctx->visible = (struct v4l2_rect){ 0, 0, DEC_DEF_W, DEC_DEF_H };

	v4l2_ctrl_handler_init(&ctx->ctrls, 1);
	v4l2_ctrl_new_std(&ctx->ctrls, NULL, V4L2_CID_MIN_BUFFERS_FOR_CAPTURE,
			  1, 32, 1, DEC_CAP_MIN_BUFS);
	ret = ctx->ctrls.error;
	if (ret)
		goto err_ctrls;
	ctx->fh.ctrl_handler = &ctx->ctrls;

	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(dec->m2m_dev, ctx, dec_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto err_ctrls;
	}
	/* Input is consumed (headers parsed) before CAPTURE is set up. */
	ctx->fh.m2m_ctx->ignore_cap_streaming = true;
	v4l2_m2m_set_dst_buffered(ctx->fh.m2m_ctx, true);

	spin_lock_irq(&dec->ctxs_lock);
	list_add(&ctx->node, &dec->ctxs);
	spin_unlock_irq(&dec->ctxs_lock);
	v4l2_fh_add(&ctx->fh, file);
	return 0;

err_ctrls:
	v4l2_ctrl_handler_free(&ctx->ctrls);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return ret;
}

static int dec_release(struct file *file)
{
	struct dec_ctx *ctx = file_to_ctx(file);
	struct lg1314_vdec_dec *dec = ctx->dec;

	mutex_lock(&dec->lock);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	mutex_unlock(&dec->lock);
	cancel_work_sync(&ctx->noti_work);
	mutex_lock(&ctx->fw_lock);
	dec_session_close(ctx);
	mutex_unlock(&ctx->fw_lock);

	spin_lock_irq(&dec->ctxs_lock);
	list_del(&ctx->node);
	spin_unlock_irq(&dec->ctxs_lock);
	flush_workqueue(dec->wq);	/* a poll may have queued us meanwhile */

	v4l2_fh_del(&ctx->fh, file);
	v4l2_fh_exit(&ctx->fh);
	v4l2_ctrl_handler_free(&ctx->ctrls);
	kfree(ctx);
	return 0;
}

/*
 * Input is consumed as soon as it is in the CPB, so while the first pictures
 * are being decoded (before CAPTURE is set up) neither queue may hold a
 * buffer. The m2m helper reports that as an error; here it just means
 * "waiting" (for V4L2_EVENT_SOURCE_CHANGE).
 */
static __poll_t dec_poll(struct file *file, struct poll_table_struct *wait)
{
	struct dec_ctx *ctx = file_to_ctx(file);
	__poll_t rc = v4l2_m2m_fop_poll(file, wait);

	if (!vb2_is_streaming(v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx)))
		rc &= ~EPOLLERR;
	return rc;
}

static const struct v4l2_file_operations dec_fops = {
	.owner		= THIS_MODULE,
	.open		= dec_open,
	.release	= dec_release,
	.poll		= dec_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= v4l2_m2m_fop_mmap,
};

/* ---- setup ---- */

struct lg1314_vdec_dec *lg1314_vdec_dec_init(struct lg1314_vdec *v)
{
	struct lg1314_vdec_dec *dec;
	int ret;

	dec = kzalloc(sizeof(*dec), GFP_KERNEL);
	if (!dec)
		return ERR_PTR(-ENOMEM);
	dec->v = v;
	dec->dev = lg1314_vdec_device(v);
	mutex_init(&dec->lock);
	spin_lock_init(&dec->ctxs_lock);
	INIT_LIST_HEAD(&dec->ctxs);

	dec->wq = alloc_ordered_workqueue(DEC_NAME "-dec", WQ_HIGHPRI);
	if (!dec->wq) {
		ret = -ENOMEM;
		goto err_free;
	}

	ret = v4l2_device_register(dec->dev, &dec->v4l2_dev);
	if (ret)
		goto err_wq;

	dec->m2m_dev = v4l2_m2m_init(&dec_m2m_ops);
	if (IS_ERR(dec->m2m_dev)) {
		ret = PTR_ERR(dec->m2m_dev);
		goto err_v4l2;
	}

	dec->vfd = (struct video_device){
		.fops		= &dec_fops,
		.ioctl_ops	= &dec_ioctl_ops,
		.minor		= -1,
		.release	= video_device_release_empty,
		.lock		= &dec->lock,
		.v4l2_dev	= &dec->v4l2_dev,
		.vfl_dir	= VFL_DIR_M2M,
		.device_caps	= V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING,
	};
	strscpy(dec->vfd.name, DEC_NAME, sizeof(dec->vfd.name));
	video_set_drvdata(&dec->vfd, dec);

	ret = video_register_device(&dec->vfd, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_m2m;
	dev_info(dec->dev, "V4L2 decoder registered as /dev/video%d\n",
		 dec->vfd.num);
	return dec;

err_m2m:
	v4l2_m2m_release(dec->m2m_dev);
err_v4l2:
	v4l2_device_unregister(&dec->v4l2_dev);
err_wq:
	destroy_workqueue(dec->wq);
err_free:
	kfree(dec);
	return ERR_PTR(ret);
}

void lg1314_vdec_dec_exit(struct lg1314_vdec_dec *dec)
{
	if (!dec)
		return;
	video_unregister_device(&dec->vfd);
	v4l2_m2m_release(dec->m2m_dev);
	v4l2_device_unregister(&dec->v4l2_dev);
	destroy_workqueue(dec->wq);
	kfree(dec);
}
