// SPDX-License-Identifier: GPL-2.0
/*
 * LG1314 video plane: the display engine's main video window, below the OSD.
 *
 * Pictures reach the window through the video-decode coprocessor's output,
 * which latches a picture into the display engine at its next tick and
 * reports when it is on screen and when it is off again. A commit's flip
 * completes when its picture is on screen; a framebuffer stays referenced
 * until it is off screen.
 */

#include <linux/debugfs.h>
#include <linux/of.h>
#include <linux/seq_file.h>
#include <linux/soc/lg/lg1314-vdec-vo.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_blend.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_print.h>
#include <drm/drm_vblank.h>

#include "lg1314_drm.h"

#define to_video(p)		container_of(p, struct lg1314_video, plane)
#define VIDEO_DE_CH		0	/* display-engine window / channel */

/*
 * The output ticks at the mode's refresh rate (lg1314_video_set_tick). Its
 * clock may differ from the system's: once an output runs, the tick is
 * measured against the system clock and the rate asked for corrected (in
 * ppm, kept for later rates and outputs); refined a few times, then kept.
 * Measured so far: no error, in free-run mode.
 */
#define VIDEO_CALIB_DELAY	msecs_to_jiffies(2000)
#define VIDEO_CALIB_PASSES	3
#define VIDEO_CALIB_MIN_TICKS	60

/* Display-engine firmware commands for the window. */
#define DE_CMD_INPUT_WIN	0x05
#define DE_CMD_OUT_WIN		0x06
#define DE_CMD_WIN_BLANK	0x1f
#define DE_CMD_WIN_PORT_SRC	0x52
#define DE_CMD_SOURCE_SIZE	0x5f
#define DE_CMD_WIN_INFO		0x68
#define DE_SRC_MVI		7	/* decoded video */

struct de_win_dim {
	u32 win;
	struct lg1314_de_rect rect;
};

struct de_source_size {
	u32 win;
	struct lg1314_de_rect size, win_rect;
};

struct de_win_info {
	u32 win;
	u32 rotate;
	struct lg1314_de_rect size, in, out;
};

struct de_port_src {
	u32 win, type, port, attr;
};

struct de_win_blank {
	u32 win;
	u32 enable;
	u8 r, g, b, pad;
	u32 is_rgb;
};

static const u32 video_formats[] = { DRM_FORMAT_NV12 };
static const u64 video_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	LG1314_FORMAT_MOD_NV12_8X4,	/* decoder pictures only */
	DRM_FORMAT_MOD_INVALID,
};

/* Blank (mute) the window to black, or show the pictures again. */
static int video_de_blank(struct lg1314_drm *lg, bool blank)
{
	struct de_win_blank b = { .win = VIDEO_DE_CH, .enable = blank, .is_rgb = 1 };

	return lg1314_de_cmd(lg, DE_CMD_WIN_BLANK, &b, sizeof(b));
}

/* Point the window at the decoder output and size it: @in is the visible
 * picture, @out its place on the panel. */
static int video_de_window(struct lg1314_drm *lg, struct lg1314_video *v,
			   const struct lg1314_de_rect *in, const struct lg1314_de_rect *out)
{
	struct de_source_size ss = { VIDEO_DE_CH, *in, *in };
	struct de_win_dim iw = { VIDEO_DE_CH, *in };
	struct de_win_dim ow = { VIDEO_DE_CH, *out };
	struct de_win_info wi = { VIDEO_DE_CH, 0, *in, *in, *out };
	int ret;

	if (!v->src_set) {
		struct de_port_src src = { VIDEO_DE_CH, DE_SRC_MVI, 0, 0 };

		/* Sent twice, as the vendor driver does. */
		ret = lg1314_de_cmd(lg, DE_CMD_WIN_PORT_SRC, &src, sizeof(src));
		if (!ret)
			ret = lg1314_de_cmd(lg, DE_CMD_WIN_PORT_SRC, &src, sizeof(src));
		if (ret)
			return ret;
		v->src_set = true;
	}
	ret = lg1314_de_cmd(lg, DE_CMD_SOURCE_SIZE, &ss, sizeof(ss));
	if (!ret)
		ret = lg1314_de_cmd(lg, DE_CMD_INPUT_WIN, &iw, sizeof(iw));
	if (!ret)
		ret = lg1314_de_cmd(lg, DE_CMD_OUT_WIN, &ow, sizeof(ow));
	if (!ret)
		ret = lg1314_de_cmd(lg, DE_CMD_WIN_INFO, &wi, sizeof(wi));
	return ret;
}

/* ---- frame timing trace ---- */

static void video_trace(struct lg1314_video *v, char ev, u32 id, u32 addr)
{
	struct lg1314_drm *lg = to_lg1314(v->plane.dev);
	struct lg1314_video_trace *t;
	unsigned long flags;

	spin_lock_irqsave(&v->trace_lock, flags);
	t = &v->trace[v->trace_head++ % ARRAY_SIZE(v->trace)];
	t->ns = ktime_get_ns();
	t->tick = READ_ONCE(lg->tick_seq);
	t->id = id;
	t->addr = addr;
	t->ev = ev;
	spin_unlock_irqrestore(&v->trace_lock, flags);
}

#ifdef CONFIG_DEBUG_FS
/*
 * One line per event, oldest first: time (ns), video tick count, event (w:
 * written to the output, s: on screen, r: off screen), frame id, and for a
 * write the picture's luma address (a commit can repeat a picture). Writing
 * anything clears it.
 */
static int video_trace_show(struct seq_file *m, void *unused)
{
	struct lg1314_video *v = m->private;
	struct lg1314_video_trace *buf;
	unsigned int head, n, i;
	unsigned long flags;

	buf = kvmalloc(sizeof(v->trace), GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	spin_lock_irqsave(&v->trace_lock, flags);
	memcpy(buf, v->trace, sizeof(v->trace));
	head = v->trace_head;
	spin_unlock_irqrestore(&v->trace_lock, flags);
	n = min_t(unsigned int, head, ARRAY_SIZE(v->trace));
	for (i = head - n; i != head; i++) {
		struct lg1314_video_trace *t = &buf[i % ARRAY_SIZE(v->trace)];

		seq_printf(m, "%llu %u %c %u %x\n", t->ns, t->tick, t->ev, t->id,
			   t->addr);
	}
	kvfree(buf);
	return 0;
}

static int video_trace_open(struct inode *inode, struct file *file)
{
	return single_open(file, video_trace_show, inode->i_private);
}

static ssize_t video_trace_write(struct file *file, const char __user *ubuf,
				 size_t len, loff_t *ppos)
{
	struct lg1314_video *v = ((struct seq_file *)file->private_data)->private;
	unsigned long flags;

	spin_lock_irqsave(&v->trace_lock, flags);
	v->trace_head = 0;
	spin_unlock_irqrestore(&v->trace_lock, flags);
	return len;
}

static const struct file_operations video_trace_fops = {
	.owner = THIS_MODULE,
	.open = video_trace_open,
	.read = seq_read,
	.write = video_trace_write,
	.llseek = seq_lseek,
	.release = single_release,
};

void lg1314_video_debugfs_init(struct lg1314_drm *lg)
{
	if (lg->video.vdec_np)
		debugfs_create_file("video_trace", 0600, lg->drm.debugfs_root,
				    &lg->video, &video_trace_fops);
}
#else
void lg1314_video_debugfs_init(struct lg1314_drm *lg) { }
#endif

/* ---- frame bookkeeping (callbacks run in process context) ---- */

static void video_shown(void *ctx, u32 id)
{
	struct lg1314_video *v = ctx;
	struct lg1314_hdr10plus md;
	unsigned long flags;
	int i, found = -1;

	video_trace(v, 's', id, 0);
	/* HDR10+: the metadata of the picture now on screen */
	spin_lock_irqsave(&v->h10p_lock, flags);
	for (i = 0; i < ARRAY_SIZE(v->h10p_q); i++) {
		if (v->h10p_q[i].id == id && id) {
			found = v->h10p_q[i].has;
			if (found)
				md = v->h10p_q[i].md;
			v->h10p_q[i].id = 0;
			break;
		}
	}
	spin_unlock_irqrestore(&v->h10p_lock, flags);
	if (found >= 0)
		lg1314_hdr_hdr10plus_shown(to_lg1314(v->plane.dev),
					   found ? &md : NULL);
	/* The window stays blanked until it has a picture to show. */
	if (READ_ONCE(v->blanked)) {
		video_de_blank(to_lg1314(v->plane.dev), false);
		WRITE_ONCE(v->blanked, false);
	}
}

static void video_released(void *ctx, u32 id)
{
	struct lg1314_video *v = ctx;
	struct drm_framebuffer *fb = NULL;
	int i;

	video_trace(v, 'r', id, 0);
	mutex_lock(&v->held_lock);
	for (i = 0; i < ARRAY_SIZE(v->held); i++) {
		if (v->held[i].fb && v->held[i].id == id) {
			fb = v->held[i].fb;
			v->held[i].fb = NULL;
			break;
		}
	}
	mutex_unlock(&v->held_lock);
	if (fb)
		drm_framebuffer_put(fb);
}

static const struct lg1314_vdec_vo_ops video_vo_ops = {
	.shown = video_shown,
	.released = video_released,
};

static void video_calibrate(struct work_struct *work)
{
	struct lg1314_video *v = container_of(to_delayed_work(work),
					      struct lg1314_video, calib_work);
	struct lg1314_drm *lg = to_lg1314(v->plane.dev);
	unsigned int n = READ_ONCE(lg->tick_n), rate;
	u64 period;

	if (!v->vo || n < VIDEO_CALIB_MIN_TICKS)
		return;
	period = div_u64(READ_ONCE(lg->tick_last_ns) - READ_ONCE(lg->tick_first_ns),
			 n - 1);
	/* rate for the tick = current rate * measured period / wanted period */
	rate = div64_u64((u64)v->rate_mhz * period * v->tick_mhz +
			 NSEC_PER_SEC * 1000ULL / 2, NSEC_PER_SEC * 1000ULL);
	drm_dbg_kms(v->plane.dev, "video tick %llu ns at %u mHz -> %u mHz\n",
		    period, v->rate_mhz, rate);
	if (rate != v->rate_mhz && rate <= U16_MAX &&
	    !lg1314_vdec_vo_set_rate(v->vo, rate, 1000)) {
		v->rate_mhz = rate;
		v->corr_ppm = (int)div_u64((u64)rate * 1000000 + v->tick_mhz / 2,
					   v->tick_mhz) - 1000000;
		lg->tick_n = 0;
	}
	if (++v->calib_passes < VIDEO_CALIB_PASSES)
		schedule_delayed_work(&v->calib_work, VIDEO_CALIB_DELAY);
	else
		drm_info(v->plane.dev, "video output rate %u.%03u Hz for a %u.%03u Hz tick (%d ppm)\n",
			 v->rate_mhz / 1000, v->rate_mhz % 1000,
			 v->tick_mhz / 1000, v->tick_mhz % 1000, v->corr_ppm);
}

/* The rate to ask the output for, for the wanted tick. */
static unsigned int video_rate(const struct lg1314_video *v)
{
	return div_u64((u64)v->tick_mhz * (1000000 + v->corr_ppm) + 500000, 1000000);
}

/*
 * The mode's refresh changed (or is first set): the output ticks at it from
 * now on, an open one at once.
 */
void lg1314_video_set_tick(struct lg1314_drm *lg, unsigned int tick_mhz)
{
	struct lg1314_video *v = &lg->video;
	unsigned int rate;

	if (tick_mhz == v->tick_mhz)
		return;
	v->tick_mhz = tick_mhz;
	rate = video_rate(v);
	if (v->vo && rate != v->rate_mhz &&
	    lg1314_vdec_vo_set_rate(v->vo, rate, 1000))
		drm_err(v->plane.dev, "video output rate %u mHz not set\n", rate);
	v->rate_mhz = rate;
	/* the calibration measures the new tick from here */
	lg->tick_n = 0;
}

static int video_hold(struct lg1314_video *v, struct drm_framebuffer *fb,
		      u32 id)
{
	int i;

	mutex_lock(&v->held_lock);
	for (i = 0; i < ARRAY_SIZE(v->held); i++) {
		if (!v->held[i].fb) {
			drm_framebuffer_get(fb);
			v->held[i].fb = fb;
			v->held[i].id = id;
			break;
		}
	}
	mutex_unlock(&v->held_lock);
	return i < ARRAY_SIZE(v->held) ? 0 : -EBUSY;
}

static void video_release_all(struct lg1314_video *v)
{
	int i;

	mutex_lock(&v->held_lock);
	for (i = 0; i < ARRAY_SIZE(v->held); i++) {
		if (v->held[i].fb) {
			drm_framebuffer_put(v->held[i].fb);
			v->held[i].fb = NULL;
		}
	}
	mutex_unlock(&v->held_lock);
}

/* ---- plane ---- */

static u32 video_plane_addr(struct drm_framebuffer *fb, int plane)
{
	return drm_fb_dma_get_gem_obj(fb, plane)->dma_addr + fb->offsets[plane];
}

static int video_atomic_check(struct drm_plane *plane,
			      struct drm_atomic_state *state)
{
	struct drm_plane_state *ps = drm_atomic_get_new_plane_state(state, plane);
	struct lg1314_drm *lg = to_lg1314(plane->dev);
	struct lg1314_video *v = to_video(plane);
	struct drm_framebuffer *fb = ps->fb;
	struct drm_crtc_state *cs;
	u32 y, c, end_y, end_c;
	int ret;

	if (!fb || !ps->crtc)
		return 0;

	cs = drm_atomic_get_new_crtc_state(state, ps->crtc);
	ret = drm_atomic_helper_check_plane_state(ps, cs, 1, INT_MAX, true, true);
	if (ret || !ps->visible)
		return ret;

	if (!v->vdec) {
		struct lg1314_vdec *vdec = lg1314_vdec_get(v->vdec_np);

		if (IS_ERR(vdec)) {
			drm_dbg_kms(plane->dev, "video output not available\n");
			return -EINVAL;
		}
		v->vdec = vdec;
		/* its Dolby Vision metadata, to prepare scenes ahead */
		lg1314_dovi_attach(lg, vdec);
	}

	/* One stride for both planes, both inside the memory the display
	 * engine reads pictures from. */
	if (fb->pitches[0] != fb->pitches[1] || fb->pitches[0] > U16_MAX)
		return -EINVAL;
	y = video_plane_addr(fb, 0);
	c = video_plane_addr(fb, 1);
	end_y = y + fb->pitches[0] * fb->height;
	end_c = c + fb->pitches[1] * (fb->height / 2);
	if (y < lg->video_mem_start || c < lg->video_mem_start ||
	    end_y > lg->video_mem_end || end_c > lg->video_mem_end) {
		drm_dbg_kms(plane->dev, "video buffer %#x/%#x outside %#x-%#x\n",
			    y, c, lg->video_mem_start, lg->video_mem_end);
		return -EINVAL;
	}
	return 0;
}

/* Close the output (and its Dolby Vision mode). */
static void video_vo_close(struct lg1314_drm *lg, struct lg1314_video *v)
{
	/* Output closed first: no callback can unblank the window after this. */
	cancel_delayed_work_sync(&v->calib_work);
	lg1314_vdec_vo_close(v->vo);
	v->vo = NULL;
	if (v->dovi_profile)
		lg1314_dovi_stop(lg);
	v->dovi_profile = 0;
	lg1314_hdr_hdr10plus_reset(lg);
}

static void video_atomic_update(struct drm_plane *plane,
				struct drm_atomic_state *state)
{
	struct drm_plane_state *ps = drm_atomic_get_new_plane_state(state, plane);
	struct lg1314_drm *lg = to_lg1314(plane->dev);
	struct lg1314_video *v = to_video(plane);
	struct drm_framebuffer *fb = ps->fb;
	const struct drm_display_mode *mode;
	struct lg1314_vdec_vo_frame f;
	struct drm_crtc_state *cs;
	struct lg1314_de_rect in, out;
	u64 ts = 0;
	u32 session = 0;
	int ret, dovi = -ENOENT;

	if (!fb || !ps->crtc || !ps->visible || !v->vdec)
		return;
	cs = drm_atomic_get_new_crtc_state(state, ps->crtc);
	mode = &cs->mode;

	/* A Dolby Vision stream's picture: its timestamp and composer image. */
	if (fb->modifier == LG1314_FORMAT_MOD_NV12_8X4)
		dovi = lg1314_vdec_picture_dovi(v->vdec, video_plane_addr(fb, 0), &ts,
						&session, &v->dovi_pic);

	/* A stream of the other kind on the same plane (a new title without the
	 * plane going off; a picture that isn't the decoder's), or a new Dolby
	 * Vision stream: the output's mode is set at open, and its pictures go
	 * with it. */
	if (v->vo && (!!v->dovi_profile != (dovi > 0 || dovi == -ENODATA) ||
		      (v->dovi_profile && session != v->dovi_session))) {
		video_vo_close(lg, v);
		/* the closed output reports no more releases */
		video_release_all(v);
	}

	/* A Dolby Vision stream starts with a picture that has a record (one
	 * after a seek may refer to metadata from before it): until then
	 * nothing is shown. */
	if (!v->vo && dovi == -ENODATA)
		return;

	if (!v->vo) {
		/* the output's mode is set before its first picture */
		v->dovi_profile = dovi > 0 && !lg1314_dovi_start(lg, dovi) ? dovi : 0;
		v->dovi_last_addr = 0;
		v->dovi_pic_valid = false;
		v->dovi_session = session;
		v->vo = lg1314_vdec_vo_open(v->vdec, VIDEO_DE_CH, v->rate_mhz, 1000,
					    v->dovi_profile, &video_vo_ops, v);
		if (IS_ERR(v->vo)) {
			drm_err(plane->dev, "video output open failed: %ld\n",
				PTR_ERR(v->vo));
			v->vo = NULL;
			if (v->dovi_profile)
				lg1314_dovi_stop(lg);
			v->dovi_profile = 0;
			return;
		}
		video_de_blank(lg, true);
		WRITE_ONCE(v->blanked, true);
		lg1314_vsync_video_phase(lg, true);
		if (v->calib_passes < VIDEO_CALIB_PASSES)
			schedule_delayed_work(&v->calib_work, VIDEO_CALIB_DELAY);
	}

	/* The visible part of the picture, and where it goes on the panel. */
	in = (struct lg1314_de_rect){
		.w = drm_rect_width(&ps->src) >> 16,
		.h = drm_rect_height(&ps->src) >> 16,
	};
	out = (struct lg1314_de_rect){
		.x = ps->dst.x1 * LG1314_PANEL_W / mode->hdisplay,
		.y = ps->dst.y1 * LG1314_PANEL_H / mode->vdisplay,
		.w = drm_rect_width(&ps->dst) * LG1314_PANEL_W / mode->hdisplay,
		.h = drm_rect_height(&ps->dst) * LG1314_PANEL_H / mode->vdisplay,
	};
	if (memcmp(&in, &v->in, sizeof(in)) || memcmp(&out, &v->out, sizeof(out))) {
		ret = video_de_window(lg, v, &in, &out);
		if (ret)
			drm_err(plane->dev, "video window setup failed: %d\n", ret);
		v->in = in;
		v->out = out;
	}

	f = (struct lg1314_vdec_vo_frame){
		.id = ++v->next_id,
		.addr_y = video_plane_addr(fb, 0),
		.addr_c = video_plane_addr(fb, 1),
		.width = fb->width,
		.height = fb->height,
		.stride = fb->pitches[0],
		.crop_left = ps->src.x1 >> 16,
		.crop_top = ps->src.y1 >> 16,
		.crop_right = fb->width - (ps->src.x2 >> 16),
		.crop_bottom = fb->height - (ps->src.y2 >> 16),
		.tiled = fb->modifier == LG1314_FORMAT_MOD_NV12_8X4,
		.pts_ns = -1,
	};
	/* Dolby Vision: the picture carries its PTS, its record is ready
	 * before the display engine asks for it. A picture is written once:
	 * the output shows it until the next one (a repeat would repeat its
	 * PTS). */
	if (v->dovi_profile && dovi != -ENOENT) {
		if (f.addr_y == v->dovi_last_addr && ts == v->dovi_last_ts)
			return;
		v->dovi_last_addr = f.addr_y;
		v->dovi_last_ts = ts;
		f.pts_ns = ts;
		/* every picture written has a record: one without its own
		 * (its RPU unusable) gets the previous picture's metadata */
		if (dovi > 0)
			v->dovi_pic_valid = true;
		/* profile 7: its enhancement layer picture, if the decoder has
		 * it (only for its own record: the residual is the picture's) */
		if (dovi > 0 && v->dovi_pic.el_y) {
			f.el_y = v->dovi_pic.el_y;
			f.el_c = v->dovi_pic.el_c;
			f.el_stride = v->dovi_pic.el_stride;
			f.el_size = v->dovi_pic.el_size;
		} else if (dovi <= 0) {
			v->dovi_pic.comp[0] = 1;	/* no residual */
		}
		if (v->dovi_pic_valid)
			lg1314_dovi_queue(lg, lg1314_dovi_pts90(ts), &v->dovi_pic);
	}
	/* HDR10+: the picture's metadata, applied once it is on screen */
	if (!v->dovi_profile) {
		unsigned long flags;
		unsigned int k;

		spin_lock_irqsave(&v->h10p_lock, flags);
		k = v->h10p_head++ % ARRAY_SIZE(v->h10p_q);
		v->h10p_q[k].id = f.id;
		v->h10p_q[k].has = dovi == 0 &&
			!lg1314_vdec_picture_hdr10plus(v->vdec, f.addr_y,
							&v->h10p_q[k].md);
		spin_unlock_irqrestore(&v->h10p_lock, flags);
	}
	if (video_hold(v, fb, f.id)) {
		drm_err(plane->dev, "video: too many frames in flight\n");
		return;
	}
	ret = lg1314_vdec_vo_write(v->vo, &f);
	if (ret == -ENOENT) {
		/* a decoder picture that's no longer there: keep the last one */
		drm_dbg_kms(plane->dev, "video frame %u gone, dropped\n", f.id);
		video_released(v, f.id);
		return;
	}
	if (ret) {
		drm_err(plane->dev, "video frame write failed: %d\n", ret);
		video_released(v, f.id);
		return;
	}	video_trace(v, 'w', f.id, f.addr_y);
}

static void video_atomic_disable(struct drm_plane *plane,
				 struct drm_atomic_state *state)
{
	struct lg1314_drm *lg = to_lg1314(plane->dev);
	struct lg1314_video *v = to_video(plane);

	if (!v->vo)
		return;
	video_vo_close(lg, v);
	lg1314_vsync_video_phase(lg, false);
	video_de_blank(lg, true);
	WRITE_ONCE(v->blanked, true);
	memset(&v->in, 0, sizeof(v->in));
	memset(&v->out, 0, sizeof(v->out));
	video_release_all(v);
}

static const struct drm_plane_helper_funcs video_helper_funcs = {
	.atomic_check = video_atomic_check,
	.atomic_update = video_atomic_update,
	.atomic_disable = video_atomic_disable,
};

static bool video_format_mod_supported(struct drm_plane *plane, u32 format,
				       u64 modifier)
{
	return format == DRM_FORMAT_NV12 &&
	       (modifier == DRM_FORMAT_MOD_LINEAR ||
		modifier == LG1314_FORMAT_MOD_NV12_8X4);
}

static const struct drm_plane_funcs video_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
	.reset = drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
	.format_mod_supported = video_format_mod_supported,
};

int lg1314_video_init(struct lg1314_drm *lg)
{
	struct lg1314_video *v = &lg->video;
	int ret;

	mutex_init(&v->held_lock);
	spin_lock_init(&v->trace_lock);
	spin_lock_init(&v->h10p_lock);
	INIT_DELAYED_WORK(&v->calib_work, video_calibrate);
	v->tick_mhz = 60000;
	v->rate_mhz = 60000;
	v->vdec_np = of_parse_phandle(lg->dev->of_node, "lge,vdec", 0);
	if (!v->vdec_np) {
		dev_info(lg->dev, "no lge,vdec: no video plane\n");
		return 0;
	}

	ret = drm_universal_plane_init(&lg->drm, &v->plane,
				       drm_crtc_mask(&lg->crtc),
				       &video_plane_funcs, video_formats,
				       ARRAY_SIZE(video_formats), video_modifiers,
				       DRM_PLANE_TYPE_OVERLAY, "video");
	if (ret)
		return ret;
	drm_plane_helper_add(&v->plane, &video_helper_funcs);
	/* Below the OSD (the primary plane, zpos 1). */
	return drm_plane_create_zpos_immutable_property(&v->plane, 0);
}

void lg1314_video_fini(struct lg1314_drm *lg)
{
	struct lg1314_video *v = &lg->video;

	if (!v->vdec_np)
		return;
	cancel_delayed_work_sync(&v->calib_work);
	lg1314_dovi_detach(lg);
	if (v->vdec)
		lg1314_vdec_put(v->vdec);
	of_node_put(v->vdec_np);
}
