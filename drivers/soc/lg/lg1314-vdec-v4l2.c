// SPDX-License-Identifier: GPL-2.0
/*
 * LG1314 video-decode coprocessor: V4L2 stateful decoder.
 *
 * Each open file is one decoder instance in the firmware. Compressed access
 * units queued on OUTPUT are copied into the instance's stream buffer (CPB);
 * the firmware decodes into picture buffers it asks the host for, and reports
 * each picture in a notification ring.
 *
 * The firmware's pictures are NV12 in the coded size, followed by its own
 * motion data, which is what CAPTURE buffers hold: when the firmware asks for
 * a picture buffer, it is lent a CAPTURE buffer, so pictures are decoded
 * straight into the buffers userspace gets. A picture goes back to the
 * firmware when userspace requeues its buffer. A picture buffer that no
 * CAPTURE buffer could be lent for comes from the dpb pool instead, and its
 * pictures are copied into a CAPTURE buffer the firmware doesn't hold.
 *
 * Input carrying a new stream geometry waits until CAPTURE has been set up
 * for it (V4L2_EVENT_SOURCE_CHANGE), so that the firmware's picture buffers
 * can be lent. A picture of a geometry the headers didn't announce is held
 * the same way.
 */

#include <linux/debugfs.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/soc/lg/lg1314-vdec-vo.h>
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
/* Picture buffers the firmware allocates: what the stream keeps plus six,
 * and at least 10 (13 above 1080p). */
#define DEC_FW_PICS_EXTRA	6
#define DEC_FW_PICS_MIN		10
#define DEC_FW_PICS_MIN_UHD	13
#define DEC_CPB_MARGIN		SZ_64K
/* Frames queued in the firmware at most (stock keeps 5), and how often to
 * look again while it is full. */
#define DEC_FW_QUEUE		8
#define DEC_FW_QUEUE_POLL_MS	2
/*
 * The firmware tags each picture with its access unit's timestamp only once
 * it has seen input timestamps go backwards (presentation order); before
 * that it hands them out in output order. So it is given access unit
 * numbers instead, the first one out of order, and the driver keeps the
 * real timestamps.
 */
#define DEC_TS_RING		4096	/* access units in flight, at most */
#define DEC_TS_FIRST		(1LL << 62)	/* tag of the first access unit */
#define DEC_CAP_SPARE		3	/* CAPTURE buffers kept from the firmware
					 * for copied pictures */

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

/* An access unit as a list of NAL units (each from its start code). */
#define FW_MAX_NALS		64
struct fw_es_nal_list {
	u64 timestamp;
	u64 uid;
	u32 size;
	u32 buf1_offset;
	u32 buf1_size;
	u32 buf2_offset;
	u32 buf2_size;
	s32 au_type;
	s32 frame_count;
	u32 nal_count;
	u32 nal_size[FW_MAX_NALS];
};
static_assert(sizeof(struct fw_es_nal_list) == 304);

/* What the firmware holds of the input: CPB bytes, access units queued. */
struct fw_buffer_status {
	u32 cpb_depth;
	u32 cpb_size;
	u32 auib_depth;
	u32 auib_size;
	u32 frame_count;
};

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

#define FW_GET_BUFFER_STATUS	_IOR(FW_IOC_TYPE, 7, struct fw_buffer_status)
#define FW_GET_NOTI_INFO	_IOR(FW_IOC_TYPE, 11, struct fw_buffer_info)
#define FW_SET_NOTI_TAIL	_IO(FW_IOC_TYPE, 13)
#define FW_GET_ES_BUFFER	_IOWR(FW_IOC_TYPE, 9, struct fw_es_data)
#define FW_PUT_ES_BUFFER	_IOW(FW_IOC_TYPE, 10, struct fw_es_data)
#define FW_PUT_ES_BUFFER_NAL_LIST _IOW(FW_IOC_TYPE, 68, struct fw_es_nal_list)
#define FW_UNREF_DECODEBUFFER	_IOW(FW_IOC_TYPE, 17, u32)
#define FW_SET_OUTPUT_MEMFMT	_IOW(FW_IOC_TYPE, 20, s32)
#define FW_SET_CODEC		_IO(FW_IOC_TYPE, 23)
#define FW_SET_INPUT_DEVICE	_IO(FW_IOC_TYPE, 25)
#define FW_SET_OUTPUT_DEVICE	_IO(FW_IOC_TYPE, 26)
#define FW_GET_CPB_INFO		_IOR(FW_IOC_TYPE, 33, struct fw_buffer_info)
#define FW_INIT			_IO(FW_IOC_TYPE, 45)
#define FW_EOS			_IO(FW_IOC_TYPE, 47)

#define FW_CODEC_H264		0
#define FW_CODEC_HEVC		16
#define FW_SRC_BUFF		3
#define FW_DST_BUFF		8
#define FW_MEMFMT_RASTER	0
#define FW_MEMFMT_TILED		1
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

/*
 * A CAPTURE buffer as firmware memory. It lives while either the vb2 buffer
 * or the firmware has it (dec_dpb_lock); the memory itself is held through a
 * dma-buf for as long.
 */
struct dec_slot {
	struct dma_buf		*pin;
	u32			phys, size;
	bool			vb;		/* the vb2 buffer exists */
	bool			fw;		/* lent to the firmware */
	bool			shown;		/* holds a picture userspace has */
	bool			back;		/* ... and userspace returned it */
	bool			user;		/* handed to userspace, not yet queued
						 * again (a picture, an error or the
						 * end-of-stream marker) */
	u32			shown_addr;	/* that picture, for the firmware */
	u32			shown_gen;	/* in that session */
	u32			shown_priv;	/* its firmware key (vdec_priv) */
	u32			shown_c;	/* its chroma (or second part) */
	struct dec_ctx		*ctx;		/* while vb */
};

static DEFINE_SPINLOCK(dec_dpb_lock);

static bool dec_zerocopy = true;
module_param_named(zerocopy, dec_zerocopy, bool, 0644);
MODULE_PARM_DESC(zerocopy, "decode into CAPTURE buffers (else copy pictures into them)");

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
	unsigned int		npending;
	/* What delivery last waited for (debugfs): a CAPTURE buffer index, its
	 * slot state, and the session of the picture shown in it. */
	int			wait_index;
	u32			wait_state;
	bool			cpb_full;
	bool			draining;	/* STOP received */
	bool			eos_sent;
	bool			eos_reported;	/* last picture reported */
	u32			cap_seq;
	u32			out_seq;
	u64			uid;
	u32			gen;		/* firmware session count */
	u64			ts[DEC_TS_RING];	/* timestamps by uid */
	u32			cap_min;	/* CAPTURE buffers wanted */
	struct v4l2_ctrl	*cap_min_ctrl;
	/* The firmware's input queue, fetched on a write to the debugfs
	 * status file. */
	bool			want_bufstat;
	int			bufstat_ret;
	struct fw_buffer_status	bufstat;

	/* CAPTURE buffers by index (dec_dpb_lock). */
	struct dec_slot		*slots[VB2_MAX_FRAME];

	struct work_struct	noti_work;
	struct delayed_work	retry_work;	/* input queue full */
	bool			got_picture;	/* in this session */
};

static bool dec_is_hevc(const struct dec_ctx *ctx)
{
	return ctx->out_fmt.pixelformat == V4L2_PIX_FMT_HEVC;
}

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
	u32 dpb_frames;			/* pictures the decoder has to keep */
	u32 bit_depth;			/* HEVC: luma */
};

static int h264_skip_hrd(struct h264_bits *b)
{
	u32 n, v;
	unsigned int i;

	if (h264_ue(b, &n) || n > 31 || h264_bits(b, 8, &v))
		return -EINVAL;
	for (i = 0; i <= n; i++)
		if (h264_ue(b, &v) || h264_ue(b, &v) || h264_bits(b, 1, &v))
			return -EINVAL;
	return h264_bits(b, 20, &v);
}

/* max_dec_frame_buffering from the VUI, if it has one. */
static int h264_vui_dpb_frames(struct h264_bits *b, u32 *frames)
{
	u32 v, nal, vcl;

	if (h264_bits(b, 1, &v))			/* aspect_ratio_info */
		return -EINVAL;
	if (v && (h264_bits(b, 8, &v) || (v == 255 && h264_bits(b, 32, &v))))
		return -EINVAL;
	if (h264_bits(b, 1, &v) || (v && h264_bits(b, 1, &v)))	/* overscan */
		return -EINVAL;
	if (h264_bits(b, 1, &v))			/* video_signal_type */
		return -EINVAL;
	if (v && (h264_bits(b, 4, &v) || (v & 1 && h264_bits(b, 24, &v))))
		return -EINVAL;
	if (h264_bits(b, 1, &v) || (v && (h264_ue(b, &v) || h264_ue(b, &v))))
		return -EINVAL;				/* chroma_loc_info */
	if (h264_bits(b, 1, &v))			/* timing_info */
		return -EINVAL;
	if (v && (h264_bits(b, 32, &v) || h264_bits(b, 32, &v) || h264_bits(b, 1, &v)))
		return -EINVAL;
	if (h264_bits(b, 1, &nal) || (nal && h264_skip_hrd(b)) ||
	    h264_bits(b, 1, &vcl) || (vcl && h264_skip_hrd(b)) ||
	    ((nal || vcl) && h264_bits(b, 1, &v)) ||	/* low_delay_hrd */
	    h264_bits(b, 1, &v) ||			/* pic_struct_present */
	    h264_bits(b, 1, &v))			/* bitstream_restriction */
		return -EINVAL;
	if (!v)
		return -ENOENT;
	if (h264_bits(b, 1, &v) || h264_ue(b, &v) || h264_ue(b, &v) ||
	    h264_ue(b, &v) || h264_ue(b, &v) || h264_ue(b, &v) ||
	    h264_ue(b, frames))
		return -EINVAL;
	return 0;
}

/* Parse an SPS NAL payload (after the NAL header byte). */
static int h264_parse_sps(const u8 *rbsp, size_t len, struct h264_geometry *g)
{
	struct h264_bits b = { .buf = rbsp, .len = len };
	u32 profile, v, chroma = 1, poc_type, w_mbs, h_units, frame_mbs_only;
	u32 crop[4] = { 0 }, cx, cy, refs, frames;
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
	if (h264_ue(&b, &refs) || h264_bits(&b, 1, &v) ||	/* max_num_ref_frames, gaps */
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
	g->dpb_frames = min(refs, 16u);
	if (!h264_bits(&b, 1, &v) && v && !h264_vui_dpb_frames(&b, &frames))
		g->dpb_frames = min(max(frames, refs), 16u);

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

/* ---- HEVC sequence parameter set ---- */

/* profile_tier_level() with profilePresentFlag 1 */
static int hevc_skip_ptl(struct h264_bits *b, u32 max_sub_layers_minus1)
{
	u32 v, profile = 0, level = 0;
	unsigned int i;

	/* general: profile space/tier/idc (8), compatibility flags (32), 4 flags,
	 * 43 bits of constraints, 1 bit, level (8): 96 bits */
	if (h264_bits(b, 32, &v) || h264_bits(b, 32, &v) || h264_bits(b, 32, &v))
		return -EINVAL;
	for (i = 0; i < max_sub_layers_minus1; i++) {
		if (h264_bits(b, 2, &v))
			return -EINVAL;
		profile |= (v >> 1) << i;
		level |= (v & 1) << i;
	}
	if (max_sub_layers_minus1)
		for (i = max_sub_layers_minus1; i < 8; i++)
			if (h264_bits(b, 2, &v))
				return -EINVAL;
	for (i = 0; i < max_sub_layers_minus1; i++) {
		if (profile & BIT(i) && (h264_bits(b, 32, &v) ||
					 h264_bits(b, 32, &v) || h264_bits(b, 24, &v)))
			return -EINVAL;
		if (level & BIT(i) && h264_bits(b, 8, &v))
			return -EINVAL;
	}
	return 0;
}

static int hevc_parse_sps(const u8 *rbsp, size_t len, struct h264_geometry *g)
{
	struct h264_bits b = { .buf = rbsp, .len = len };
	u32 v, subs, chroma, w, h, crop[4] = { 0 }, dpb = 1, cx, cy;
	unsigned int i;

	if (h264_bits(&b, 4, &v) || h264_bits(&b, 3, &subs) || h264_bits(&b, 1, &v) ||
	    hevc_skip_ptl(&b, subs) || h264_ue(&b, &v) || h264_ue(&b, &chroma))
		return -EINVAL;
	if (chroma == 3 && h264_bits(&b, 1, &v))
		return -EINVAL;
	if (h264_ue(&b, &w) || h264_ue(&b, &h) || h264_bits(&b, 1, &v))
		return -EINVAL;
	if (v)		/* conformance window */
		for (i = 0; i < 4; i++)
			if (h264_ue(&b, &crop[i]))
				return -EINVAL;
	/* bit depths, log2_max_pic_order_cnt_lsb, sub_layer_ordering_info */
	if (h264_ue(&b, &g->bit_depth) || h264_ue(&b, &v) || h264_ue(&b, &v) ||
	    h264_bits(&b, 1, &v))
		return -EINVAL;
	for (i = v ? 0 : subs; i <= subs; i++) {
		if (h264_ue(&b, &dpb) || h264_ue(&b, &v) || h264_ue(&b, &v))
			return -EINVAL;
		dpb++;
	}
	if (!w || !h || w > DEC_MAX_W || h > DEC_MAX_H)
		return -EINVAL;
	g->bit_depth += 8;
	g->dpb_frames = min(dpb, 16u);
	g->width = w;
	g->height = h;
	cx = chroma == 1 || chroma == 2 ? 2 : 1;
	cy = chroma == 1 ? 2 : 1;
	if ((crop[0] + crop[1]) * cx >= w || (crop[2] + crop[3]) * cy >= h)
		return -EINVAL;
	g->visible.left = crop[0] * cx;
	g->visible.top = crop[2] * cy;
	g->visible.width = w - (crop[0] + crop[1]) * cx;
	g->visible.height = h - (crop[2] + crop[3]) * cy;
	return 0;
}

/* Find the first SPS in an Annex-B access unit (H.264, or HEVC) and parse
 * it. */
static int dec_find_sps(const u8 *data, size_t size, bool hevc,
			struct h264_geometry *g)
{
	size_t i, start, end, n;
	u8 *rbsp;
	int ret;

	for (i = 0; i + 4 < size; i++) {
		if (data[i] || data[i + 1] || data[i + 2] != 1)
			continue;
		if (hevc ? (data[i + 3] >> 1 & 0x3f) != 33 : (data[i + 3] & 0x1f) != 7)
			continue;
		start = i + (hevc ? 5 : 4);
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
		ret = hevc ? hevc_parse_sps(rbsp, n, g) : h264_parse_sps(rbsp, n, g);
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
	ctx->npending = 0;
	if (ctx->noti)
		memunmap(ctx->noti);
	if (ctx->cpb)
		memunmap(ctx->cpb);
	ctx->noti = ctx->cpb = NULL;
	lg1314_vdec_pictures_lock(ctx->dec->v, true);
	lg1314_vdec_call(ctx->dec->v, VDEC_CALL_VDEC_CLOSE, ctx->handle, NULL, 0,
			 0, &result);
	dev_dbg(ctx->dec->dev, "ctx %p: session %#x closed\n", ctx, ctx->handle);
	WRITE_ONCE(ctx->handle, 0);
	lg1314_vdec_pictures_unlock(ctx->dec->v, true);
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
	s32 memfmt = ctx->cap_fmt.pixelformat == LG1314_V4L2_PIX_FMT_NV12_8X4 ?
		     FW_MEMFMT_TILED : FW_MEMFMT_RASTER;
	int ret, handle;

	/* A new session before it has a handle: pictures of the last one
	 * are no longer current from here on (the handle may be the same). */
	ctx->gen++;
	ret = lg1314_vdec_call(v, VDEC_CALL_VDEC_OPEN, 0, NULL, 0, 0, &handle);
	if (ret)
		return ret;
	if (!handle)
		return -EBUSY;
	ctx->handle = handle;

	ret = dec_fw_ioctl(ctx, FW_SET_CODEC, dec_is_hevc(ctx) ? FW_CODEC_HEVC :
							     FW_CODEC_H264, NULL);
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
	ctx->cpb_full = false;
	ctx->draining = ctx->eos_sent = ctx->eos_reported = false;
	ctx->got_picture = false;
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
 * Copy a decoded picture held in the dpb pool into a CAPTURE buffer (NV12,
 * the coded size): luma at addr_y and interleaved chroma at addr_cb.
 */
static int dec_copy_frame(struct dec_ctx *ctx, const struct fw_decoded *d,
			  struct vb2_v4l2_buffer *dst)
{
	struct lg1314_vdec *v = ctx->dec->v;
	u32 bpl = ctx->cap_fmt.plane_fmt[0].bytesperline;
	u32 h = ctx->cap_fmt.height;
	u32 w = min_t(u32, d->width, ctx->cap_fmt.width);
	const void *y, *c;
	void *out;

	y = lg1314_vdec_frame_map(v, d->addr_y, (size_t)d->stride * h);
	c = lg1314_vdec_frame_map(v, d->addr_cb, (size_t)d->stride * h / 2);
	out = vb2_plane_vaddr(&dst->vb2_buf, 0);
	if (!y || !c || !out)
		return -EFAULT;

	dec_copy_plane(out, bpl, y, d->stride, w, h);
	dec_copy_plane(out + bpl * h, bpl, c, d->stride, w, h / 2);
	vb2_set_plane_payload(&dst->vb2_buf, 0, ctx->cap_fmt.plane_fmt[0].sizeimage);
	return 0;
}

/*
 * CAPTURE is the firmware's picture layout for a coded size (whole
 * macroblocks): NV12 with the chroma right after the luma, then 64 bytes of
 * motion data per macroblock, in whole pages. Raster, or the decoder's own
 * 8x4 tiles (same sizes; only the display engine reads those), as chosen
 * before decoding starts.
 */
static void dec_set_cap_fmt(struct dec_ctx *ctx, u32 width, u32 height)
{
	struct v4l2_pix_format_mplane *f = &ctx->cap_fmt;
	u32 extra;

	if (dec_is_hevc(ctx)) {
		/*
		 * HEVC: only the decoder's own (compressed) layout, which the
		 * display engine reads; P and B pictures don't decode into
		 * raster. Measured: a table of 64 KiB per 2048x512, then 16 KiB
		 * per 64x128 (8 and 10 bits alike), then 256 bytes per 64x64.
		 */
		u32 w64 = DIV_ROUND_UP(width, 64);

		width = ALIGN(width, 8);
		height = ALIGN(height, 8);
		f->pixelformat = LG1314_V4L2_PIX_FMT_NV12_8X4;
		f->plane_fmt[0].bytesperline = w64 * 64;
		f->plane_fmt[0].sizeimage = ALIGN(
			DIV_ROUND_UP(width, 2048) * DIV_ROUND_UP(height, 512) * SZ_64K +
			w64 * DIV_ROUND_UP(height, 128) * SZ_16K +
			w64 * DIV_ROUND_UP(height, 64) * 256, SZ_4K);
	} else {
		width = ALIGN(width, 16);
		height = ALIGN(height, 16);
		extra = (width / 16) * (height / 16) * 64;
		if (f->pixelformat != LG1314_V4L2_PIX_FMT_NV12_8X4)
			f->pixelformat = V4L2_PIX_FMT_NV12;
		f->plane_fmt[0].bytesperline = width;
		f->plane_fmt[0].sizeimage = ALIGN(width * height * 3 / 2 + extra,
						  SZ_4K);
	}
	f->width = width;
	f->height = height;
	f->field = V4L2_FIELD_NONE;
	f->num_planes = 1;
	f->colorspace = ctx->out_fmt.colorspace;
	f->xfer_func = ctx->out_fmt.xfer_func;
	f->ycbcr_enc = ctx->out_fmt.ycbcr_enc;
	f->quantization = ctx->out_fmt.quantization;
}

static bool dec_geometry_matches(struct dec_ctx *ctx, const struct fw_decoded *d)
{
	return ctx->have_geometry && d->width == ctx->cap_fmt.width &&
	       d->height == ctx->cap_fmt.height;
}

static bool dec_geometry_is(struct dec_ctx *ctx, u32 width, u32 height,
			    const struct v4l2_rect *visible)
{
	return ctx->have_geometry && width == ctx->cap_fmt.width &&
	       height == ctx->cap_fmt.height &&
	       !memcmp(visible, &ctx->visible, sizeof(*visible));
}

/* A new stream geometry (coded size and visible part): adopt it and tell
 * userspace. */
static void dec_signal_geometry(struct dec_ctx *ctx, u32 width, u32 height,
				const struct v4l2_rect *visible)
{
	static const struct v4l2_event ev = {
		.type = V4L2_EVENT_SOURCE_CHANGE,
		.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
	};

	dec_set_cap_fmt(ctx, width, height);
	ctx->visible = *visible;
	ctx->have_geometry = true;
	ctx->src_change = true;
	v4l2_event_queue_fh(&ctx->fh, &ev);
}

/* A picture whose geometry the headers didn't announce: adopt it. */
static void dec_source_change(struct dec_ctx *ctx, const struct fw_decoded *d)
{
	struct v4l2_rect visible = {
		d->crop_left, d->crop_top, dec_vis_w(d), dec_vis_h(d),
	};

	dec_signal_geometry(ctx, d->width, d->height, &visible);
}

static void dec_signal_eos(struct dec_ctx *ctx)
{
	static const struct v4l2_event ev = { .type = V4L2_EVENT_EOS };

	v4l2_event_queue_fh(&ctx->fh, &ev);
}

/* The access unit a picture was decoded from, or -1 if it doesn't say. */
static s64 dec_picture_uid(struct dec_ctx *ctx, const struct fw_decoded *d)
{
	u64 uid = d->timestamp == DEC_TS_FIRST ? 0 : d->timestamp;

	return d->timestamp < 0 || uid >= ctx->uid ? -1 : uid;
}

static u64 dec_picture_timestamp(struct dec_ctx *ctx, const struct fw_decoded *d)
{
	s64 uid = dec_picture_uid(ctx, d);

	return uid < 0 ? 0 : ctx->ts[uid % DEC_TS_RING];
}

/*
 * The stream can't go on as set up: fail the CAPTURE queue (poll reports an
 * error, DQBUF fails) rather than leave userspace waiting for a picture that
 * won't come. CAPTURE STREAMOFF clears it.
 */
static void dec_fail(struct dec_ctx *ctx, const char *why)
{
	dev_warn_ratelimited(ctx->dec->dev, "decoder stopped: %s\n", why);
	vb2_queue_error(v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx));
}

/* ---- CAPTURE buffers lent to the firmware ---- */

static void dec_slot_free(struct dec_slot *sl)
{
	dma_buf_put(sl->pin);
	kfree(sl);
}

static struct dec_slot *dec_slot_get(struct dec_ctx *ctx, unsigned int index)
{
	return index < ARRAY_SIZE(ctx->slots) ? ctx->slots[index] : NULL;
}

/* The CAPTURE buffer the firmware's picture at addr is in, or -1. */
static int dec_slot_find(struct dec_ctx *ctx, u32 addr)
{
	int i, index = -1;

	spin_lock(&dec_dpb_lock);
	for (i = 0; i < ARRAY_SIZE(ctx->slots); i++) {
		struct dec_slot *sl = ctx->slots[i];

		if (sl && sl->fw && addr >= sl->phys && addr - sl->phys < sl->size) {
			index = i;
			break;
		}
	}
	spin_unlock(&dec_dpb_lock);
	return index;
}

/* Take a queued CAPTURE buffer the firmware doesn't hold, or with any_buf,
 * any queued one. */
static struct vb2_v4l2_buffer *dec_take_dst(struct dec_ctx *ctx, bool any_buf)
{
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	struct v4l2_m2m_buffer *b;
	struct vb2_v4l2_buffer *dst = NULL, *first = NULL;
	unsigned long flags;

	spin_lock_irqsave(&m2m_ctx->cap_q_ctx.rdy_spinlock, flags);
	v4l2_m2m_for_each_dst_buf(m2m_ctx, b) {
		struct dec_slot *sl;
		bool lent;

		spin_lock(&dec_dpb_lock);
		sl = dec_slot_get(ctx, b->vb.vb2_buf.index);
		lent = sl && sl->fw;
		spin_unlock(&dec_dpb_lock);
		if (!first)
			first = &b->vb;
		if (!lent) {
			dst = &b->vb;
			break;
		}
	}
	spin_unlock_irqrestore(&m2m_ctx->cap_q_ctx.rdy_spinlock, flags);
	if (!dst && any_buf)
		dst = first;
	if (dst)
		v4l2_m2m_dst_buf_remove_by_buf(m2m_ctx, dst);
	return dst;
}

/* Give the firmware back the pictures whose buffers userspace returned.
 * fw_lock held. */
static void dec_release_returned(struct dec_ctx *ctx)
{
	u32 addr[VB2_MAX_FRAME];
	unsigned int i, n = 0;

	spin_lock(&dec_dpb_lock);
	for (i = 0; i < ARRAY_SIZE(ctx->slots); i++) {
		struct dec_slot *sl = ctx->slots[i];

		if (!sl || !sl->shown || !sl->back)
			continue;
		if (sl->fw && sl->shown_gen == ctx->gen)
			addr[n++] = sl->shown_addr;
		sl->shown = sl->back = false;
	}
	spin_unlock(&dec_dpb_lock);

	if (!n)
		return;
	/* not while a frame with one of them goes to the video output */
	lg1314_vdec_pictures_lock(ctx->dec->v, true);
	for (i = 0; i < n && ctx->handle; i++)
		dec_fw_ioctl(ctx, FW_UNREF_DECODEBUFFER, 0, &addr[i]);
	lg1314_vdec_pictures_unlock(ctx->dec->v, true);
}

u32 lg1314_vdec_dec_dpb_alloc(struct lg1314_vdec_dec *dec, u32 size, u32 align,
			      void **cookie, bool *wait)
{
	bool held_only = false;
	struct dec_slot *pick = NULL;
	struct dec_ctx *ctx;
	unsigned long flags;
	bool several = false;
	unsigned int i;

	if (!dec || !READ_ONCE(dec_zerocopy))
		return 0;
	spin_lock_irqsave(&dec->ctxs_lock, flags);
	spin_lock(&dec_dpb_lock);
	/*
	 * The firmware doesn't say which decoder it allocates for: lend only
	 * when exactly one has a buffer that fits, keeping a few back for
	 * pictures that have to be copied. Prefer a buffer userspace doesn't
	 * hold: one it still shows from before a restart of the session would
	 * only come back once newer pictures arrive, and those might be the
	 * ones decoded into it.
	 */
	list_for_each_entry(ctx, &dec->ctxs, node) {
		struct dec_slot *fit = NULL, *fit_held = NULL;
		unsigned int unlent = 0;

		if (!ctx->handle)
			continue;
		for (i = 0; i < ARRAY_SIZE(ctx->slots); i++) {
			struct dec_slot *sl = ctx->slots[i];

			if (!sl || sl->fw)
				continue;
			unlent++;
			/* (near) exactly the picture size: HEVC's working
			 * buffers come from the same requests */
			if (sl->size < size || sl->size - size >= SZ_64K ||
			    (sl->phys & (max(align, 1u) - 1)))
				continue;
			if (sl->user) {
				if (!fit_held)
					fit_held = sl;
			} else if (!fit) {
				fit = sl;
			}
		}
		/* HEVC has no spare buffers: one userspace holds (likely on
		 * screen, from before a seek) would be decoded into while
		 * shown, and waited for. Rather wait for one to come back. */
		if (!fit && fit_held && dec_is_hevc(ctx)) {
			held_only = true;
			continue;
		}
		if (!fit)
			fit = fit_held;
		if (!fit || unlent <= (dec_is_hevc(ctx) ? 0 : DEC_CAP_SPARE))
			continue;
		if (pick)
			several = true;
		pick = fit;
	}
	if (pick && !several) {
		pick->fw = true;
		*cookie = pick;
	} else {
		pick = NULL;
		*wait = held_only;
	}
	spin_unlock(&dec_dpb_lock);
	spin_unlock_irqrestore(&dec->ctxs_lock, flags);
	return pick ? pick->phys : 0;
}

u32 lg1314_vdec_dec_picture_priv(struct lg1314_vdec_dec *dec, u32 addr,
				 u32 *addr_c)
{
	struct dec_ctx *ctx;
	unsigned long flags;
	unsigned int i;
	u32 priv = 0;

	if (!dec)
		return 0;
	spin_lock_irqsave(&dec->ctxs_lock, flags);
	spin_lock(&dec_dpb_lock);
	list_for_each_entry(ctx, &dec->ctxs, node) {
		for (i = 0; i < ARRAY_SIZE(ctx->slots) && !priv; i++) {
			struct dec_slot *sl = ctx->slots[i];

			/* only while the firmware still has the picture: its
			 * session is the open one */
			if (sl && sl->shown && !sl->back && sl->shown_addr == addr &&
			    sl->shown_gen == ctx->gen && READ_ONCE(ctx->handle)) {
				priv = sl->shown_priv;
				*addr_c = sl->shown_c;
			}
		}
		if (priv)
			break;
	}
	spin_unlock(&dec_dpb_lock);
	spin_unlock_irqrestore(&dec->ctxs_lock, flags);
	return priv;
}

void lg1314_vdec_dec_dpb_failed(struct lg1314_vdec_dec *dec)
{
	struct dec_ctx *ctx;
	unsigned long flags;

	if (!dec)
		return;
	/* Whose request it was isn't known: every decoder with a session. */
	spin_lock_irqsave(&dec->ctxs_lock, flags);
	list_for_each_entry(ctx, &dec->ctxs, node)
		if (ctx->handle)
			dec_fail(ctx, "out of memory for a picture buffer");
	spin_unlock_irqrestore(&dec->ctxs_lock, flags);
}

void lg1314_vdec_dec_dpb_free(void *cookie)
{
	struct dec_slot *sl = cookie;
	bool gone;

	spin_lock(&dec_dpb_lock);
	sl->fw = false;
	gone = !sl->vb;
	spin_unlock(&dec_dpb_lock);
	if (gone)
		dec_slot_free(sl);
}

/* A CAPTURE buffer goes to userspace. */
static void dec_slot_handed_out(struct dec_ctx *ctx, struct vb2_v4l2_buffer *dst)
{
	struct dec_slot *sl;

	spin_lock(&dec_dpb_lock);
	sl = dec_slot_get(ctx, dst->vb2_buf.index);
	if (sl)
		sl->user = true;
	spin_unlock(&dec_dpb_lock);
}

/* Return pending pictures (and the end of stream) to CAPTURE. fw_lock held. */
static void dec_deliver(struct dec_ctx *ctx)
{
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	struct vb2_v4l2_buffer *dst;
	struct dec_frame *f;
	bool ok;
	int index;

	dec_release_returned(ctx);

	while ((f = list_first_entry_or_null(&ctx->pending, struct dec_frame, list))) {
		if (!dec_geometry_matches(ctx, &f->d)) {
			if (!ctx->src_change)
				dec_source_change(ctx, &f->d);
			return;
		}
		if (ctx->src_change || !vb2_is_streaming(v4l2_m2m_get_dst_vq(m2m_ctx)))
			return;

		index = dec_slot_find(ctx, f->d.addr_y);
		if (index >= 0) {
			/* Decoded in place: wait for userspace to queue it. */
			dst = v4l2_m2m_dst_buf_remove_by_idx(m2m_ctx, index);
			if (!dst) {
				struct dec_slot *sl;

				spin_lock(&dec_dpb_lock);
				sl = ctx->slots[index];
				ctx->wait_index = index;
				ctx->wait_state = sl ? sl->fw | sl->shown << 1 |
						       sl->back << 2 | sl->shown_gen << 8 : 0;
				spin_unlock(&dec_dpb_lock);
				return;
			}
			ctx->wait_index = -1;
			spin_lock(&dec_dpb_lock);
			if (ctx->slots[index]) {
				ctx->slots[index]->shown = true;
				ctx->slots[index]->back = false;
				ctx->slots[index]->shown_addr = f->d.addr_y;
				ctx->slots[index]->shown_gen = ctx->gen;
				ctx->slots[index]->shown_priv = f->d.vdec_priv;
				ctx->slots[index]->shown_c = f->d.addr_cb;
			}
			spin_unlock(&dec_dpb_lock);
			vb2_set_plane_payload(&dst->vb2_buf, 0,
					      ctx->cap_fmt.plane_fmt[0].sizeimage);
			ok = true;
		} else {
			dst = dec_take_dst(ctx, false);
			if (!dst)
				return;
			ok = !dec_copy_frame(ctx, &f->d, dst);
			dec_fw_ioctl(ctx, FW_UNREF_DECODEBUFFER, 0, &f->d.addr_y);
		}

		if (!ok) {
			vb2_set_plane_payload(&dst->vb2_buf, 0, 0);
			dec_slot_handed_out(ctx, dst);
			v4l2_m2m_buf_done(dst, VB2_BUF_STATE_ERROR);
		} else {
			dst->vb2_buf.timestamp = dec_picture_timestamp(ctx, &f->d);
			dst->field = V4L2_FIELD_NONE;
			dst->sequence = ctx->cap_seq++;
			dst->flags &= ~(V4L2_BUF_FLAG_KEYFRAME | V4L2_BUF_FLAG_PFRAME |
					V4L2_BUF_FLAG_BFRAME);
			dst->flags |= f->d.picture_type == FW_PIC_I ? V4L2_BUF_FLAG_KEYFRAME :
				      f->d.picture_type == FW_PIC_P ? V4L2_BUF_FLAG_PFRAME :
				      V4L2_BUF_FLAG_BFRAME;
			dec_slot_handed_out(ctx, dst);
			v4l2_m2m_buf_done(dst, f->d.report & FW_REPORT_ERROR_MASK ?
					  VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);
		}
		list_del(&f->list);
		ctx->npending--;
		kfree(f);
	}

	if (ctx->eos_reported && !ctx->src_change &&
	    vb2_is_streaming(v4l2_m2m_get_dst_vq(m2m_ctx))) {
		dst = dec_take_dst(ctx, true);
		if (dst) {
			ctx->eos_reported = false;
			ctx->draining = false;
			vb2_set_plane_payload(&dst->vb2_buf, 0, 0);
			dec_slot_handed_out(ctx, dst);
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
				progressed = true;
			} else if (hdr[1] == FW_NOTI_DECO &&
				   len == sizeof(struct fw_decoded)) {
				struct dec_frame *f = kzalloc(sizeof(*f), GFP_KERNEL);

				if (f) {
					dec_noti_read(ctx, tail + 8, &f->d, len);
					if (f->d.addr_y) {
						ctx->got_picture = true;
						list_add_tail(&f->list, &ctx->pending);
						ctx->npending++;
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
	if (ctx->want_bufstat) {
		ctx->want_bufstat = false;
		ctx->bufstat_ret = ctx->handle ?
			dec_fw_ioctl(ctx, FW_GET_BUFFER_STATUS, 0, &ctx->bufstat) :
			-ENODEV;
	}
	mutex_unlock(&ctx->fw_lock);
	if (room)
		v4l2_m2m_try_schedule(ctx->fh.m2m_ctx);
}

/* Look at the firmware's input queue again. */
static void dec_retry_work(struct work_struct *work)
{
	struct dec_ctx *ctx = container_of(to_delayed_work(work), struct dec_ctx,
					   retry_work);

	mutex_lock(&ctx->fw_lock);
	ctx->cpb_full = false;
	mutex_unlock(&ctx->fw_lock);
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

/* Frames the firmware holds that it hasn't decoded (or skipped) yet; 0 if
 * it can't tell. fw_lock held. */
static u32 dec_fw_queued(struct dec_ctx *ctx)
{
	struct fw_buffer_status st;

	return dec_fw_ioctl(ctx, FW_GET_BUFFER_STATUS, 0, &st) ? 0 : st.frame_count;
}

/* Copy one access unit into the CPB and hand it to the firmware. fw_lock held. */
/*
 * HEVC: whether an access unit has NAL units that aren't the base layer's
 * (Dolby Vision's RPU and enhancement layer: types 62 and 63, or a layer id
 * above 0); if so, a copy without them in *@out (*@out_size bytes).
 */
static int dec_hevc_base_layer(const u8 *data, u32 size, u8 **out, u32 *out_size)
{
	u32 i, start, end, n = 0;
	bool other = false;
	u8 *buf = NULL;

	for (int pass = 0; pass < 2; pass++) {
		for (i = 0; i + 4 < size; ) {
			bool skip;

			if (data[i] || data[i + 1] || data[i + 2] != 1) {
				i++;
				continue;
			}
			/* a start code, its leading zero byte if 4 bytes long */
			start = i && !data[i - 1] ? i - 1 : i;
			for (end = i + 3; end + 2 < size; end++)
				if (!data[end] && !data[end + 1] && data[end + 2] <= 1)
					break;
			if (end + 2 >= size)
				end = size;
			skip = (data[i + 3] >> 1 & 0x3f) >= 62 ||
			       ((data[i + 3] & 1) << 5 | data[i + 4] >> 3);
			if (!pass) {
				other |= skip;
			} else if (!skip) {
				memcpy(buf + n, data + start, end - start);
				n += end - start;
			}
			i = end;
		}
		if (!other)
			return 0;
		if (!pass) {
			buf = kvmalloc(size, GFP_KERNEL);
			if (!buf)
				return -ENOMEM;
		}
	}
	*out = buf;
	*out_size = n;
	return 1;
}

/*
 * The sizes of an Annex-B access unit's NAL units, each from its start code
 * to the next one (the first from the start); past FW_MAX_NALS the rest goes
 * with the last. Returns how many.
 */
static u32 dec_nal_sizes(const u8 *data, u32 size, u32 *sizes)
{
	u32 i, n = 0, start = 0;

	for (i = 3; i + 3 < size; i++) {
		if (data[i] || data[i + 1] || data[i + 2] != 1)
			continue;
		/* a new NAL unit at i (or i - 1 for a 4-byte start code) */
		if (i - (data[i - 1] ? 0 : 1) > start && n < FW_MAX_NALS - 1) {
			u32 at = i - (data[i - 1] ? 0 : 1);

			sizes[n++] = at - start;
			start = at;
		}
		i += 2;
	}
	sizes[n++] = size - start;
	return n;
}

static int dec_feed(struct dec_ctx *ctx, struct vb2_v4l2_buffer *src)
{
	const void *data = vb2_plane_vaddr(&src->vb2_buf, 0);
	u32 size = vb2_get_plane_payload(&src->vb2_buf, 0);
	struct fw_es_data es;
	u8 *copy = NULL;
	int ret;

	/* the decoder decodes the base layer, and garbles it if handed more */
	if (dec_is_hevc(ctx)) {
		ret = dec_hevc_base_layer(data, size, &copy, &size);
		if (ret < 0)
			return ret;
		if (copy)
			data = copy;
	}
	es = (struct fw_es_data){ .size = size };

	dev_dbg(ctx->dec->dev, "ctx %p: feed %u bytes, starts %*ph\n",
		ctx, size, 5, data);

	ret = dec_fw_ioctl(ctx, FW_GET_ES_BUFFER, 0, &es);
	if (!ret && (es.buf1_offset + es.buf1_size > ctx->cpb_size ||
		     es.buf2_offset + es.buf2_size > ctx->cpb_size ||
		     es.buf1_size + es.buf2_size < size))
		ret = -EIO;
	if (ret) {
		kvfree(copy);
		return ret;
	}
	memcpy(ctx->cpb + es.buf1_offset, data, min(size, es.buf1_size));
	if (size > es.buf1_size)
		memcpy(ctx->cpb + es.buf2_offset, data + es.buf1_size,
		       size - es.buf1_size);
	wmb();

	es.uid = ctx->uid++;
	es.timestamp = es.uid ? es.uid : DEC_TS_FIRST;
	ctx->ts[es.uid % DEC_TS_RING] = src->vb2_buf.timestamp;
	es.size = size;
	es.au_type = FW_AU_DATA;
	/* a running frame number: the firmware counts what it holds with it */
	es.frame_count = (es.uid + 1) & S32_MAX;
	if (dec_is_hevc(ctx)) {
		/* HEVC's decoder finds slices only as separate NAL units: with
		 * a picture's slices in one piece, it decodes the first */
		struct fw_es_nal_list *nl = kzalloc(sizeof(*nl), GFP_KERNEL);

		if (!nl) {
			kvfree(copy);
			return -ENOMEM;
		}
		nl->timestamp = es.timestamp;
		nl->uid = es.uid;
		nl->size = es.size;
		nl->buf1_offset = es.buf1_offset;
		nl->buf1_size = es.buf1_size;
		nl->buf2_offset = es.buf2_offset;
		nl->buf2_size = es.buf2_size;
		nl->au_type = es.au_type;
		nl->frame_count = es.frame_count;
		nl->nal_count = dec_nal_sizes(data, size, nl->nal_size);
		ret = dec_fw_ioctl(ctx, FW_PUT_ES_BUFFER_NAL_LIST, 0, nl);
		kfree(nl);
		kvfree(copy);
		return ret;
	}
	kvfree(copy);
	return dec_fw_ioctl(ctx, FW_PUT_ES_BUFFER, 0, &es);
}

/*
 * Whether an access unit has to wait for CAPTURE: it announces a geometry
 * CAPTURE isn't set up for (signalled here), or one is still being set up.
 * Then the firmware's picture buffers can be CAPTURE buffers. fw_lock held.
 */
static bool dec_wait_geometry(struct dec_ctx *ctx, struct vb2_v4l2_buffer *src)
{
	struct h264_geometry g;

	if (ctx->src_change)
		return true;
	if (dec_find_sps(vb2_plane_vaddr(&src->vb2_buf, 0),
			 vb2_get_plane_payload(&src->vb2_buf, 0), dec_is_hevc(ctx), &g) ||
	    dec_geometry_is(ctx, g.width, g.height, &g.visible))
		return false;
	/* Enough CAPTURE buffers for all of the firmware's pictures, when
	 * they are to be lent. */
	ctx->cap_min = DEC_CAP_MIN_BUFS;
	if (dec_is_hevc(ctx))
		/* allocated as the decoder needs them, up to this; no spares:
		 * a copied picture of the decoder's layout can't be shown */
		ctx->cap_min = g.dpb_frames + DEC_FW_PICS_EXTRA + 1;
	else if (READ_ONCE(dec_zerocopy))
		ctx->cap_min = max(g.dpb_frames + DEC_FW_PICS_EXTRA,
				   g.width * g.height > 1920 * 1088 ?
				   DEC_FW_PICS_MIN_UHD : DEC_FW_PICS_MIN) +
			       DEC_CAP_SPARE;
	v4l2_ctrl_s_ctrl(ctx->cap_min_ctrl, ctx->cap_min);
	dec_signal_geometry(ctx, g.width, g.height, &g.visible);
	return true;
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
		if (dec_wait_geometry(ctx, src))
			goto out;
		/*
		 * A short input queue in the firmware, as stock keeps. A new
		 * session decodes only I pictures until one is back, and its
		 * feeder skips a deep queue of other pictures before that: one
		 * access unit at a time until the first picture. The firmware
		 * doesn't report pictures it skips, so its own count is asked.
		 */
		if (dec_fw_queued(ctx) >= (ctx->got_picture ? DEC_FW_QUEUE : 1)) {
			ctx->cpb_full = true;
			schedule_delayed_work(&ctx->retry_work,
					      msecs_to_jiffies(DEC_FW_QUEUE_POLL_MS));
			goto out;
		}
		if (dec_cpb_free(ctx) < size + DEC_CPB_MARGIN) {
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

	return READ_ONCE(ctx->handle) && !READ_ONCE(ctx->cpb_full) &&
	       !READ_ONCE(ctx->src_change);
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
	/* HEVC's pictures at 4K leave room for no more than it uses */
	if (V4L2_TYPE_IS_CAPTURE(vq->type))
		*nbuffers = dec_is_hevc(ctx) ? ctx->cap_min :
			    max(*nbuffers, ctx->cap_min);
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

/* A CAPTURE buffer the firmware can reach becomes lendable. */
static int dec_buf_init(struct vb2_buffer *vb)
{
	struct dec_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	dma_addr_t phys = vb2_dma_contig_plane_dma_addr(vb, 0);
	size_t size = vb2_plane_size(vb, 0);
	struct dec_slot *sl;

	if (V4L2_TYPE_IS_OUTPUT(vb->type) || vb->index >= ARRAY_SIZE(ctx->slots) ||
	    phys + size > U32_MAX ||
	    !lg1314_vdec_in_carveout(ctx->dec->v, phys, size))
		return 0;	/* only ever copied into */

	sl = kzalloc(sizeof(*sl), GFP_KERNEL);
	if (!sl)
		return -ENOMEM;
	if (vb->memory == VB2_MEMORY_DMABUF) {
		sl->pin = vb->planes[0].dbuf;
		get_dma_buf(sl->pin);
	} else {
		sl->pin = vb->vb2_queue->mem_ops->get_dmabuf(vb, vb->planes[0].mem_priv,
							     O_RDWR);
		if (!sl->pin) {
			kfree(sl);
			return 0;
		}
	}
	sl->phys = phys;
	sl->size = size;
	sl->vb = true;
	sl->user = true;	/* until it is queued */

	spin_lock(&dec_dpb_lock);
	ctx->slots[vb->index] = sl;
	spin_unlock(&dec_dpb_lock);
	return 0;
}

/* The vb2 buffer goes away; the firmware may keep the memory a while. */
static void dec_buf_cleanup(struct vb2_buffer *vb)
{
	struct dec_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct dec_slot *sl;
	bool release = false, gone = false;
	u32 addr = 0;

	if (V4L2_TYPE_IS_OUTPUT(vb->type) || vb->index >= ARRAY_SIZE(ctx->slots))
		return;

	spin_lock(&dec_dpb_lock);
	sl = ctx->slots[vb->index];
	ctx->slots[vb->index] = NULL;
	if (sl) {
		if (sl->shown && sl->fw && sl->shown_gen == ctx->gen) {
			release = true;
			addr = sl->shown_addr;
		}
		sl->shown = sl->back = false;
		sl->vb = false;
		gone = !sl->fw;
	}
	spin_unlock(&dec_dpb_lock);
	if (!sl)
		return;

	if (release) {
		mutex_lock(&ctx->fw_lock);
		if (ctx->handle)
			dec_fw_ioctl(ctx, FW_UNREF_DECODEBUFFER, 0, &addr);
		mutex_unlock(&ctx->fw_lock);
	}
	if (gone)
		dec_slot_free(sl);
}

static void dec_buf_queue(struct vb2_buffer *vb)
{
	struct dec_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	if (V4L2_TYPE_IS_CAPTURE(vb->type)) {
		struct dec_slot *sl;

		/* Its picture can go back to the firmware (dec_deliver). */
		spin_lock(&dec_dpb_lock);
		sl = dec_slot_get(ctx, vb->index);
		if (sl && sl->shown)
			sl->back = true;
		if (sl)
			sl->user = false;
		spin_unlock(&dec_dpb_lock);
	}

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, to_vb2_v4l2_buffer(vb));
	if (V4L2_TYPE_IS_CAPTURE(vb->type))
		queue_work(ctx->dec->wq, &ctx->noti_work);	/* deliver */
}

static void dec_return_bufs(struct dec_ctx *ctx, bool output,
			    enum vb2_buffer_state state)
{
	struct vb2_v4l2_buffer *vb;

	while ((vb = output ? v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx) :
			     v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx))) {
		if (!output)
			dec_slot_handed_out(ctx, vb);
		v4l2_m2m_buf_done(vb, state);
	}
}

/* Whether the CAPTURE buffers are all big enough for the current format. */
static bool dec_cap_fits(struct dec_ctx *ctx)
{
	struct vb2_queue *q = v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx);
	unsigned int i, n = 0;

	for (i = 0; i < q->max_num_buffers; i++) {
		struct vb2_buffer *vb = vb2_get_buffer(q, i);

		if (!vb || !vb2_plane_size(vb, 0))	/* dmabuf not attached yet */
			continue;
		if (vb2_plane_size(vb, 0) < ctx->cap_fmt.plane_fmt[0].sizeimage)
			return false;
		n++;
	}
	return n && n >= ctx->cap_min;
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
		/* CAPTURE (re)configured: for the current geometry? */
		ctx->cap_seq = 0;
		if (dec_cap_fits(ctx))
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
		cancel_delayed_work_sync(&ctx->retry_work);
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
	.buf_init		= dec_buf_init,
	.buf_cleanup		= dec_buf_cleanup,
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
	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		if (f->index > 1)
			return -EINVAL;
		f->pixelformat = f->index ? V4L2_PIX_FMT_HEVC : V4L2_PIX_FMT_H264;
		f->flags = V4L2_FMT_FLAG_COMPRESSED | V4L2_FMT_FLAG_DYN_RESOLUTION;
		return 0;
	}
	switch (f->index + dec_is_hevc(file_to_ctx(file))) {
	case 0:
		f->pixelformat = V4L2_PIX_FMT_NV12;
		return 0;
	case 1:
		f->pixelformat = LG1314_V4L2_PIX_FMT_NV12_8X4;
		strscpy(f->description, "LG1314 decoder layout",
			sizeof(f->description));
		return 0;
	}
	return -EINVAL;
}

static int dec_enum_framesizes(struct file *file, void *priv,
			       struct v4l2_frmsizeenum *fs)
{
	if (fs->index || (fs->pixel_format != V4L2_PIX_FMT_H264 &&
			  fs->pixel_format != V4L2_PIX_FMT_HEVC))
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
	if (f->pixelformat != V4L2_PIX_FMT_HEVC)
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

	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		dec_adjust_out_fmt(&f->fmt.pix_mp);
	} else {
		bool tiled = f->fmt.pix_mp.pixelformat == LG1314_V4L2_PIX_FMT_NV12_8X4;

		/* The sizes are fixed by the stream, the layout can be chosen. */
		f->fmt.pix_mp = ctx->cap_fmt;
		if (!dec_is_hevc(ctx))
			f->fmt.pix_mp.pixelformat = tiled ? LG1314_V4L2_PIX_FMT_NV12_8X4 :
							    V4L2_PIX_FMT_NV12;
	}
	return 0;
}

static int dec_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct dec_ctx *ctx = file_to_ctx(file);
	struct vb2_queue *vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);

	if (vb2_is_busy(vq))
		return -EBUSY;
	dec_try_fmt(file, priv, f);
	if (!V4L2_TYPE_IS_OUTPUT(f->type) &&
	    f->fmt.pix_mp.pixelformat != ctx->cap_fmt.pixelformat) {
		/* The firmware's output layout is set when its session opens. */
		mutex_lock(&ctx->fw_lock);
		if (ctx->handle) {
			mutex_unlock(&ctx->fw_lock);
			return -EBUSY;
		}
		ctx->cap_fmt.pixelformat = f->fmt.pix_mp.pixelformat;
		mutex_unlock(&ctx->fw_lock);
	}
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
		/* After a source change: CAPTURE as it is fits the stream, or
		 * the stream can't be decoded into it. */
		if (ctx->src_change &&
		    vb2_is_streaming(v4l2_m2m_get_dst_vq(m2m_ctx))) {
			if (dec_cap_fits(ctx))
				ctx->src_change = false;
			else
				dec_fail(ctx, "CAPTURE buffers too few or too small for the stream");
		}
		mutex_unlock(&ctx->fw_lock);
		queue_work(ctx->dec->wq, &ctx->noti_work);
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

	ctx = kvzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ctx->dec = dec;
	mutex_init(&ctx->fw_lock);
	INIT_LIST_HEAD(&ctx->pending);
	ctx->wait_index = -1;
	ctx->bufstat_ret = 1;	/* not fetched */
	INIT_WORK(&ctx->noti_work, dec_noti_work);
	INIT_DELAYED_WORK(&ctx->retry_work, dec_retry_work);
	v4l2_fh_init(&ctx->fh, video_devdata(file));
	file->private_data = &ctx->fh;

	ctx->out_fmt.width = DEC_DEF_W;
	ctx->out_fmt.height = DEC_DEF_H;
	ctx->out_fmt.colorspace = V4L2_COLORSPACE_REC709;
	dec_adjust_out_fmt(&ctx->out_fmt);
	dec_set_cap_fmt(ctx, DEC_DEF_W, DEC_DEF_H);
	ctx->visible = (struct v4l2_rect){ 0, 0, DEC_DEF_W, DEC_DEF_H };

	v4l2_ctrl_handler_init(&ctx->ctrls, 1);
	ctx->cap_min = DEC_CAP_MIN_BUFS;
	ctx->cap_min_ctrl = v4l2_ctrl_new_std(&ctx->ctrls, NULL,
					      V4L2_CID_MIN_BUFFERS_FOR_CAPTURE,
					      1, VB2_MAX_FRAME, 1, DEC_CAP_MIN_BUFS);
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
	kvfree(ctx);
	return ret;
}

static int dec_release(struct file *file)
{
	struct dec_ctx *ctx = file_to_ctx(file);
	struct lg1314_vdec_dec *dec = ctx->dec;

	mutex_lock(&dec->lock);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	mutex_unlock(&dec->lock);
	cancel_delayed_work_sync(&ctx->retry_work);
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
	kvfree(ctx);
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

/* debugfs lg1314-vdec-status: each open decoder's state, for debugging stalls. */
static int dec_status_show(struct seq_file *m, void *unused)
{
	struct lg1314_vdec_dec *dec = m->private;
	struct dec_ctx *ctx;
	unsigned long flags;
	unsigned int i, lent, held;

	spin_lock_irqsave(&dec->ctxs_lock, flags);
	list_for_each_entry(ctx, &dec->ctxs, node) {
		struct v4l2_m2m_ctx *m2m = ctx->fh.m2m_ctx;

		lent = held = 0;
		spin_lock(&dec_dpb_lock);
		for (i = 0; i < ARRAY_SIZE(ctx->slots); i++) {
			struct dec_slot *sl = ctx->slots[i];

			if (sl && sl->fw)
				lent++;
			if (sl && sl->user)
				held++;
		}
		spin_unlock(&dec_dpb_lock);
		seq_printf(m, "ctx %p session %#x gen %u: fed %llu delivered %u pending %u\n",
			   ctx, ctx->handle, ctx->gen, ctx->uid, ctx->cap_seq,
			   ctx->npending);
		seq_printf(m, "  OUTPUT %s ready %u, CAPTURE %s ready %u, %ux%u\n",
			   vb2_is_streaming(v4l2_m2m_get_src_vq(m2m)) ? "on" : "off",
			   v4l2_m2m_num_src_bufs_ready(m2m),
			   vb2_is_streaming(v4l2_m2m_get_dst_vq(m2m)) ? "on" : "off",
			   v4l2_m2m_num_dst_bufs_ready(m2m),
			   ctx->cap_fmt.width, ctx->cap_fmt.height);
		seq_printf(m, "  src_change %d cpb_full %d draining %d eos_sent %d eos_reported %d\n",
			   ctx->src_change, ctx->cpb_full, ctx->draining,
			   ctx->eos_sent, ctx->eos_reported);
		seq_printf(m, "  picture buffers lent %u, held by userspace %u, CAPTURE wanted %u\n",
			   lent, held, ctx->cap_min);
		if (!ctx->bufstat_ret)
			seq_printf(m, "  firmware input (at last status write): %u access units, %u frames, CPB %u of %u bytes\n",
				   ctx->bufstat.auib_depth, ctx->bufstat.frame_count,
				   ctx->bufstat.cpb_depth, ctx->bufstat.cpb_size);
		else if (ctx->bufstat_ret != 1)
			seq_printf(m, "  firmware input: error %d\n", ctx->bufstat_ret);
		if (ctx->wait_index >= 0)
			seq_printf(m, "  delivery waits for CAPTURE %d: lent %u shown %u returned %u (session %u)\n",
				   ctx->wait_index, ctx->wait_state & 1,
				   ctx->wait_state >> 1 & 1, ctx->wait_state >> 2 & 1,
				   ctx->wait_state >> 8);
	}
	spin_unlock_irqrestore(&dec->ctxs_lock, flags);
	return 0;
}
static int dec_status_open(struct inode *inode, struct file *file)
{
	return single_open(file, dec_status_show, inode->i_private);
}

/* Writing anything re-reads every decoder's notifications (a lost doorbell
 * would leave pictures unread). */
static ssize_t dec_status_write(struct file *file, const char __user *buf,
				size_t len, loff_t *ppos)
{
	struct lg1314_vdec_dec *dec = file_inode(file)->i_private;
	struct dec_ctx *ctx;
	unsigned long flags;

	spin_lock_irqsave(&dec->ctxs_lock, flags);
	list_for_each_entry(ctx, &dec->ctxs, node) {
		WRITE_ONCE(ctx->want_bufstat, true);
		queue_work(dec->wq, &ctx->noti_work);
	}
	spin_unlock_irqrestore(&dec->ctxs_lock, flags);
	return len;
}

static const struct file_operations dec_status_fops = {
	.owner		= THIS_MODULE,
	.open		= dec_status_open,
	.read		= seq_read,
	.write		= dec_status_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

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
	debugfs_create_file("lg1314-vdec-status", 0600, NULL, dec, &dec_status_fops);
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
	debugfs_lookup_and_remove("lg1314-vdec-status", NULL);
	video_unregister_device(&dec->vfd);
	v4l2_m2m_release(dec->m2m_dev);
	v4l2_device_unregister(&dec->v4l2_dev);
	destroy_workqueue(dec->wq);
	kfree(dec);
}
