// SPDX-License-Identifier: GPL-2.0
/* DRM/KMS driver for the LG1314 display pipeline (LG OLED55B9PLA). */

#include <linux/aperture.h>
#include <linux/dma-mapping.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <drm/clients/drm_client_setup.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_blend.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_vblank.h>
#include <linux/bitfield.h>
#include <linux/interrupt.h>

#include "lg1314_drm.h"

static const struct {
	u32 phys;
	u32 size;
} lg1314_regions[LG1314_REG_COUNT] = {
	[LG1314_REG_OSD]    = { 0xc9010000, 0x2000 },
	[LG1314_REG_OVI]    = { 0xc9070000, 0x10000 },
	[LG1314_REG_CTOP]   = { 0xc9300000, 0x10000 },
	[LG1314_REG_DE_IPC] = { 0xc9026000, 0x1000 },
	[LG1314_REG_DPE]    = { 0xc9020000, 0x1000 },
	[LG1314_REG_PLL]    = { 0xc9c10000, 0x1000 },
	[LG1314_REG_IMX]    = { IMX_PHYS, 0x100 },
	[LG1314_REG_PE_HDR] = { 0xc9005800, 0x400 },
	[LG1314_REG_PE_ND]  = { 0xc9003000, 0x2000 },
};

void __iomem *lg1314_map(struct lg1314_drm *lg, u32 phys)
{
	int i;

	for (i = 0; i < LG1314_REG_COUNT; i++) {
		struct lg1314_mmio *m = &lg->mmio[i];

		if (phys >= m->phys && phys < m->phys + m->size)
			return m->virt + (phys - m->phys);
	}
	WARN_ONCE(1, "lg1314: unmapped reg %#x\n", phys);
	return NULL;
}

void lg1314_wr(struct lg1314_drm *lg, u32 phys, u32 val)
{
	void __iomem *p = lg1314_map(lg, phys);

	if (p)
		writel(val, p);
}

u32 lg1314_rd(struct lg1314_drm *lg, u32 phys)
{
	void __iomem *p = lg1314_map(lg, phys);

	return p ? readl(p) : 0;
}

/* -------------------------------------------------------------- KMS core -- */

/*
 * One CRTC: the OSD (primary plane) and the video plane, mixed by the display
 * engine. Vblank is every other OSD mixer vsync: the panel refreshes at twice
 * the mode's rate, locked to the video output's tick (lg1314_de_frame_lock).
 */

static const u32 lg1314_formats[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
};

/*
 * AFBC scanout: Arm 16x16 superblocks, sparse body. That is what stock webOS
 * scans and what Panfrost renders on Midgard. With stock's decoder settings
 * a YTR buffer shows garbage (no YTR enable is known), so only the plain
 * variant is offered, and only for ARGB8888: Panfrost's XRGB8888 AFBC comes
 * out with blue lost whatever the OSD's channel order (cfw/4k-osd.md).
 */
#define LG1314_AFBC_MOD	DRM_FORMAT_MOD_ARM_AFBC(AFBC_FORMAT_MOD_BLOCK_SIZE_16x16 | \
						AFBC_FORMAT_MOD_SPARSE)
/*
 * Off by default: Kodi renders one ARGB AFBC surface and registers it as
 * XRGB8888 while no video plays, and AFBC mode ignores hdr3's global alpha,
 * so its GUI comes out tinted (cfw/4k-osd.md, pass 6).
 */
static bool afbc;
module_param(afbc, bool, 0444);
MODULE_PARM_DESC(afbc, "Offer AFBC scanout (ARGB8888) on the OSD plane (default off)");

static const u64 lg1314_primary_modifiers[] = {
	LG1314_AFBC_MOD,
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID,
};

static bool lg1314_primary_format_mod_supported(struct drm_plane *plane,
						u32 format, u64 modifier)
{
	return modifier == DRM_FORMAT_MOD_LINEAR ||
	       (afbc && modifier == LG1314_AFBC_MOD &&
		format == DRM_FORMAT_ARGB8888);
}

static void lg1314_osd_show(struct lg1314_drm *lg, struct drm_plane_state *ps)
{
	struct drm_framebuffer *fb = ps->fb;

	lg1314_osd_flip(lg, drm_fb_dma_get_gem_addr(fb, ps, 0), fb->pitches[0],
			fb->width, fb->height, !fb->format->has_alpha,
			drm_is_afbc(fb->modifier));
}

/* ---- primary plane (OSD) ---- */

/* The OSD takes 1280x720 to 3840x2160, full screen; the device minimum is
 * lower, for video frames on the video plane. */
static int lg1314_primary_atomic_check(struct drm_plane *plane,
				       struct drm_atomic_state *state)
{
	struct drm_plane_state *ps = drm_atomic_get_new_plane_state(state, plane);
	struct drm_framebuffer *fb = ps->fb;
	struct drm_crtc_state *cs;

	if (!fb || !ps->crtc)
		return 0;
	if (fb->width < LG1314_FB_MIN_W || fb->height < LG1314_FB_MIN_H)
		return -EINVAL;
	/*
	 * A 3840-wide AFBC plane hangs the decoder partway through the frame
	 * (cfw/4k-osd.md, pass 6). Unsolved, so AFBC is limited to the
	 * SR-scaled sizes.
	 */
	if (drm_is_afbc(fb->modifier) && fb->width > LG1314_FB_W) {
		drm_dbg_kms(plane->dev, "AFBC wider than %u not supported\n",
			    LG1314_FB_W);
		return -EINVAL;
	}

	cs = drm_atomic_get_new_crtc_state(state, ps->crtc);
	return drm_atomic_helper_check_plane_state(ps, cs, DRM_PLANE_NO_SCALING,
						   DRM_PLANE_NO_SCALING,
						   false, false);
}

static void lg1314_primary_atomic_update(struct drm_plane *plane,
					 struct drm_atomic_state *state)
{
	struct drm_plane_state *ps = drm_atomic_get_new_plane_state(state, plane);
	struct lg1314_drm *lg = to_lg1314(plane->dev);

	if (!lg->enabled)
		return;
	/* No framebuffer: the display stays on, showing only the video plane. */
	if (ps->fb)
		lg1314_osd_show(lg, ps);
	lg1314_osd_layer(lg, ps->fb);
}

static const struct drm_plane_helper_funcs lg1314_primary_helper_funcs = {
	.atomic_check = lg1314_primary_atomic_check,
	.atomic_update = lg1314_primary_atomic_update,
};

static const struct drm_plane_funcs lg1314_primary_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
	.reset = drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
	.format_mod_supported = lg1314_primary_format_mod_supported,
};

/* ---- CRTC ---- */

/*
 * Vblank: every other panel refresh (OSD mixer vsync); while video is shown,
 * the video output's tick instead (DE sync pulse i5 following the video
 * source), which is when a new video frame is latched. The panel runs close
 * to, but not phase-locked with, twice that tick.
 */
static irqreturn_t lg1314_vsync_irq(int irq, void *arg)
{
	struct lg1314_drm *lg = arg;
	bool vblank;

	if (!lg1314_osd_vsync_ack(lg))
		return IRQ_NONE;

	vblank = !READ_ONCE(lg->video_phase) && (++lg->vsync_count & 1);
	if (vblank)
		drm_crtc_handle_vblank(&lg->crtc);
	return IRQ_HANDLED;
}

static irqreturn_t lg1314_video_tick_irq(int irq, void *arg)
{
	struct lg1314_drm *lg = arg;

	if (!(lg1314_rd(lg, DE_IPC_PHYS + DE_IPC_CPU_FLAG) & DE_IPC_CPU_FLAG_IMX) ||
	    !(lg1314_rd(lg, IMX_PHYS + IMX_INTR_REG) & IMX_PULSE_I5))
		return IRQ_NONE;
	lg1314_wr(lg, IMX_PHYS + IMX_INTR_REG, IMX_PULSE_I5);

	if (READ_ONCE(lg->video_phase)) {
		u64 now = ktime_get_ns();

		if (!lg->tick_n++)
			lg->tick_first_ns = now;
		lg->tick_last_ns = now;
		drm_crtc_handle_vblank(&lg->crtc);
	}
	return IRQ_HANDLED;
}

void lg1314_vsync_video_phase(struct lg1314_drm *lg, bool on)
{
	u32 pulse = lg1314_rd(lg, IMX_PHYS + IMX_CTRL_INTR_PULSE);
	u32 mask = lg1314_rd(lg, IMX_PHYS + IMX_INTR_MASK);
	u32 mux = lg1314_rd(lg, IMX_PHYS + IMX_INTR_MUX);

	u32 en = lg1314_rd(lg, DE_IPC_PHYS + DE_IPC_CPU_FLAG_EN);

	if (on) {
		pulse = (pulse & ~IMX_PULSE_I5_CFG) |
			FIELD_PREP(IMX_PULSE_I5_CFG, IMX_PULSE_SRC0);
		lg1314_wr(lg, IMX_PHYS + IMX_CTRL_INTR_PULSE, pulse);
		lg1314_wr(lg, IMX_PHYS + IMX_INTR_REG, IMX_PULSE_I5);
		lg1314_wr(lg, IMX_PHYS + IMX_INTR_MUX, mux | IMX_PULSE_I5);
		lg1314_wr(lg, IMX_PHYS + IMX_INTR_MASK, mask & ~IMX_PULSE_I5);
		lg->tick_n = 0;
		WRITE_ONCE(lg->video_phase, true);
		lg1314_wr(lg, DE_IPC_PHYS + DE_IPC_CPU_FLAG_EN, en | DE_IPC_CPU_FLAG_IMX);
	} else {
		lg1314_wr(lg, DE_IPC_PHYS + DE_IPC_CPU_FLAG_EN, en & ~DE_IPC_CPU_FLAG_IMX);
		lg1314_wr(lg, IMX_PHYS + IMX_INTR_MASK, mask | IMX_PULSE_I5);
		lg1314_wr(lg, IMX_PHYS + IMX_INTR_MUX, mux & ~IMX_PULSE_I5);
		lg1314_wr(lg, IMX_PHYS + IMX_INTR_REG, IMX_PULSE_I5);
		WRITE_ONCE(lg->video_phase, false);
	}
}

static int lg1314_enable_vblank(struct drm_crtc *crtc)
{
	lg1314_osd_vsync_irq(to_lg1314(crtc->dev), true);
	return 0;
}

static void lg1314_disable_vblank(struct drm_crtc *crtc)
{
	lg1314_osd_vsync_irq(to_lg1314(crtc->dev), false);
}

static void lg1314_crtc_atomic_enable(struct drm_crtc *crtc,
				      struct drm_atomic_state *state)
{
	struct lg1314_drm *lg = to_lg1314(crtc->dev);
	struct drm_plane_state *ps = drm_atomic_get_new_plane_state(state, &lg->primary);
	int ret;

	ret = lg1314_panel_vcc(lg, true);
	if (ret)
		return;

	lg1314_osd_init(lg);

	ret = lg1314_ovi_enable(lg);
	if (ret)
		goto err_vcc;

	/*
	 * The DE firmware is downloaded once and keeps running across display
	 * off/on: downloading it again onto the running core leaves it unable
	 * to take commands.
	 */
	if (!lg->de_fw_loaded) {
		ret = lg1314_de_load_fw(lg);
		if (ret)
			goto err_ovi;
		lg->de_fw_loaded = true;
		if (!lg1314_de_wait_ready(lg)) {
			lg1314_de_frame_lock(lg);
			lg1314_hdr_init(lg);
		}
	}

	/* First frame before the panel lights up. */
	lg1314_osd_enable(lg, true);
	if (ps && ps->fb)
		lg1314_osd_show(lg, ps);
	lg1314_osd_layer(lg, ps && ps->fb);

	ret = lg1314_panel_evdd(lg, true);
	if (ret)
		goto err_ovi;

	lg->enabled = true;
	drm_crtc_vblank_on(crtc);
	return;

err_ovi:
	lg1314_ovi_disable(lg);
err_vcc:
	lg1314_panel_vcc(lg, false);
}

static void lg1314_crtc_atomic_disable(struct drm_crtc *crtc,
				       struct drm_atomic_state *state)
{
	struct lg1314_drm *lg = to_lg1314(crtc->dev);
	struct drm_crtc_state *cs = drm_atomic_get_new_crtc_state(state, crtc);

	drm_crtc_vblank_off(crtc);

	if (lg->enabled) {
		lg->enabled = false;
		lg1314_osd_enable(lg, false);
		lg1314_ovi_disable(lg);
		lg1314_panel_evdd(lg, false);
		lg1314_panel_vcc(lg, false);
	}

	spin_lock_irq(&crtc->dev->event_lock);
	if (cs->event) {
		drm_crtc_send_vblank_event(crtc, cs->event);
		cs->event = NULL;
	}
	spin_unlock_irq(&crtc->dev->event_lock);
}

/*
 * Complete the flip at the next vblank, unless the video plane took the
 * event: then it completes when the video frame is on screen.
 */
static void lg1314_crtc_atomic_flush(struct drm_crtc *crtc,
				     struct drm_atomic_state *state)
{
	struct drm_crtc_state *cs = drm_atomic_get_new_crtc_state(state, crtc);
	struct drm_pending_vblank_event *event;

	lg1314_hdr_commit(to_lg1314(crtc->dev), state);

	spin_lock_irq(&crtc->dev->event_lock);
	event = cs->event;
	cs->event = NULL;
	if (event) {
		if (cs->active && drm_crtc_vblank_get(crtc) == 0)
			drm_crtc_arm_vblank_event(crtc, event);
		else
			drm_crtc_send_vblank_event(crtc, event);
	}
	spin_unlock_irq(&crtc->dev->event_lock);
}

static const struct drm_crtc_helper_funcs lg1314_crtc_helper_funcs = {
	.atomic_enable = lg1314_crtc_atomic_enable,
	.atomic_disable = lg1314_crtc_atomic_disable,
	.atomic_flush = lg1314_crtc_atomic_flush,
};

static const struct drm_crtc_funcs lg1314_crtc_funcs = {
	.reset = drm_atomic_helper_crtc_reset,
	.destroy = drm_crtc_cleanup,
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
	.enable_vblank = lg1314_enable_vblank,
	.disable_vblank = lg1314_disable_vblank,
};

/* --------------------------------------------------------------- connector -- */

/*
 * The OSD graphics plane is SR-scaled to the 3840x2160 panel. The OSD header
 * input size is set from the framebuffer per flip; the SR output is fixed to
 * the panel. Timings are nominal; the real panel timing is fixed by the OVI
 * VX1 table.
 */
static const struct drm_display_mode lg1314_modes[] = {
	/*
	 * Native 4K plane (1:1, OSD buffer mode). NB: slips lines under heavy
	 * memory traffic (OSD fetch starved on LBUS); see cfw/4k-osd.md.
	 */
	{ DRM_MODE("3840x2160", DRM_MODE_TYPE_DRIVER,
		 594000, 3840, 4016, 4104, 4400, 0,
		 2160, 2168, 2178, 2250, 0,
		 DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) },
	{ DRM_MODE("1920x1080", DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
		 148500, 1920, 2008, 2052, 2200, 0,
		 1080, 1084, 1089, 1125, 0,
		 DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) },
	{ DRM_MODE("1280x720", DRM_MODE_TYPE_DRIVER,
		 74250, 1280, 1390, 1430, 1650, 0,
		 720, 725, 730, 750, 0,
		 DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) },
};

static int lg1314_conn_get_modes(struct drm_connector *connector)
{
	int i, count = 0;

	lg1314_hdr_edid(connector);

	for (i = 0; i < ARRAY_SIZE(lg1314_modes); i++) {
		struct drm_display_mode *mode;

		mode = drm_mode_duplicate(connector->dev, &lg1314_modes[i]);
		if (!mode)
			continue;
		drm_mode_probed_add(connector, mode);
		count++;
	}
	return count;
}

static const struct drm_connector_helper_funcs lg1314_conn_helper_funcs = {
	.get_modes = lg1314_conn_get_modes,
};

static const struct drm_connector_funcs lg1314_conn_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

/* ------------------------------------------------------------------- driver -- */

static const struct drm_framebuffer_funcs lg1314_fb_funcs = {
	.destroy = drm_gem_fb_destroy,
	.create_handle = drm_gem_fb_create_handle,
};

/* drm_gem_fb_create, plus the AFBC size checks for AFBC framebuffers. */
static struct drm_framebuffer *
lg1314_fb_create(struct drm_device *dev, struct drm_file *file,
		 const struct drm_format_info *info,
		 const struct drm_mode_fb_cmd2 *mode_cmd)
{
	struct drm_afbc_framebuffer *afbc_fb;
	int ret;

	afbc_fb = kzalloc(sizeof(*afbc_fb), GFP_KERNEL);
	if (!afbc_fb)
		return ERR_PTR(-ENOMEM);

	ret = drm_gem_fb_init_with_funcs(dev, &afbc_fb->base, file, info,
					 mode_cmd, &lg1314_fb_funcs);
	if (ret) {
		kfree(afbc_fb);
		return ERR_PTR(ret);
	}

	if (drm_is_afbc(mode_cmd->modifier[0])) {
		ret = drm_gem_fb_afbc_init(dev, info, mode_cmd, afbc_fb);
		if (ret) {
			drm_framebuffer_put(&afbc_fb->base);
			return ERR_PTR(ret);
		}
	}
	return &afbc_fb->base;
}

static const struct drm_mode_config_funcs lg1314_mode_config_funcs = {
	.fb_create = lg1314_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

DEFINE_DRM_GEM_DMA_FOPS(lg1314_fops);

static struct drm_driver lg1314_drm_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops = &lg1314_fops,
	.name = "lg1314",
	.desc = "LG1314 OLED display",
	.major = 1,
	.minor = 0,
	DRM_GEM_DMA_DRIVER_OPS,
	DRM_FBDEV_DMA_DRIVER_OPS,
};

static int lg1314_init_mode_config(struct lg1314_drm *lg)
{
	struct drm_device *drm = &lg->drm;
	int ret;

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;

	drm->mode_config.min_width = LG1314_VIDEO_MIN_W;
	drm->mode_config.max_width = LG1314_PANEL_W;
	drm->mode_config.min_height = LG1314_VIDEO_MIN_H;
	drm->mode_config.max_height = LG1314_PANEL_H;
	drm->mode_config.funcs = &lg1314_mode_config_funcs;

	drm_connector_helper_add(&lg->connector, &lg1314_conn_helper_funcs);
	ret = drm_connector_init(drm, &lg->connector, &lg1314_conn_funcs,
				 DRM_MODE_CONNECTOR_DPI);
	if (ret)
		return ret;

	ret = drm_universal_plane_init(drm, &lg->primary, 0, &lg1314_primary_funcs,
				       lg1314_formats, ARRAY_SIZE(lg1314_formats),
				       afbc ? lg1314_primary_modifiers :
					      lg1314_primary_modifiers + 1,
				       DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret)
		return ret;
	drm_plane_helper_add(&lg->primary, &lg1314_primary_helper_funcs);
	/* The OSD sits above the video plane. */
	ret = drm_plane_create_zpos_immutable_property(&lg->primary, 1);
	if (ret)
		return ret;

	ret = drm_crtc_init_with_planes(drm, &lg->crtc, &lg->primary, NULL,
					&lg1314_crtc_funcs, NULL);
	if (ret)
		return ret;
	drm_crtc_helper_add(&lg->crtc, &lg1314_crtc_helper_funcs);

	ret = drm_simple_encoder_init(drm, &lg->encoder, DRM_MODE_ENCODER_NONE);
	if (ret)
		return ret;
	lg->encoder.possible_crtcs = drm_crtc_mask(&lg->crtc);
	ret = drm_connector_attach_encoder(&lg->connector, &lg->encoder);
	if (ret)
		return ret;
	ret = lg1314_hdr_init_props(lg);
	if (ret)
		return ret;

	ret = lg1314_video_init(lg);
	if (ret)
		return ret;

	ret = drm_vblank_init(drm, 1);
	if (ret)
		return ret;

	drm_mode_config_reset(drm);
	return 0;
}

/* A reserved-memory region of ours, by its memory-region-names entry. */
static int lg1314_get_region(struct lg1314_drm *lg, const char *name,
			     struct resource *res)
{
	struct device_node *np;
	int idx, ret;

	idx = of_property_match_string(lg->dev->of_node, "memory-region-names", name);
	if (idx < 0)
		return dev_err_probe(lg->dev, idx, "missing %s region\n", name);
	np = of_parse_phandle(lg->dev->of_node, "memory-region", idx);
	if (!np)
		return dev_err_probe(lg->dev, -EINVAL, "bad %s region\n", name);
	ret = of_address_to_resource(np, 0, res);
	of_node_put(np);
	return ret;
}

/* The DE firmware image region and the memory layout handed to the firmware. */
static int lg1314_get_de_regions(struct lg1314_drm *lg)
{
	struct lg1314_de_mem *m = &lg->de_mem;
	struct resource fw;
	int ret;

	ret = lg1314_get_region(lg, "de-fw", &fw);
	if (!ret)
		ret = lg1314_get_region(lg, "de-fw-m0", &m->fw_m0);
	if (!ret)
		ret = lg1314_get_region(lg, "de-prew", &m->prew);
	if (!ret)
		ret = lg1314_get_region(lg, "de-hdr", &m->hdr);
	if (!ret)
		ret = lg1314_get_region(lg, "de-frm", &m->frm);
	if (!ret)
		ret = lg1314_get_region(lg, "de-vt", &m->vt);
	if (!ret)
		ret = lg1314_get_region(lg, "be", &m->be);
	if (!ret)
		ret = lg1314_get_region(lg, "vdec-dpb", &m->vdec_dpb);
	if (!ret)
		ret = lg1314_get_region(lg, "scanout", &m->scanout);
	if (ret)
		return ret;
	if (of_property_read_u32(lg->dev->of_node, "lge,de-model", &m->model))
		return dev_err_probe(lg->dev, -EINVAL, "missing lge,de-model\n");

	lg->de_fw_phys = fw.start;
	lg->de_fw_size = resource_size(&fw);

	/* The DE reads pictures from the decoded-picture and surface pools. */
	lg->video_mem_start = min(m->vdec_dpb.start, m->scanout.start);
	lg->video_mem_end = max(m->vdec_dpb.end, m->scanout.end) + 1;
	return 0;
}

static int lg1314_get_micom(struct lg1314_drm *lg)
{
	struct device_node *np;

	np = of_parse_phandle(lg->dev->of_node, "lge,micom", 0);
	if (!np)
		return dev_err_probe(lg->dev, -EINVAL, "missing lge,micom phandle\n");

	lg->micom = of_find_i2c_device_by_node(np);
	of_node_put(np);
	if (!lg->micom)
		return -EPROBE_DEFER;
	return 0;
}

static void lg1314_release_rmem(void *dev)
{
	of_reserved_mem_device_release(dev);
}

static int lg1314_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct lg1314_drm *lg;
	int ret, i;

	/*
	 * Scanout buffers come from the "scanout" pool (idx 0). -EBUSY: the
	 * pool is still attached from an earlier binding that didn't release
	 * it (older builds of this driver); it's the same pool, so carry on.
	 * Attached before the DRM device, so it is released after it.
	 */
	ret = of_reserved_mem_device_init_by_idx(dev, dev->of_node, 0);
	if (ret && ret != -EBUSY)
		return dev_err_probe(dev, ret, "no scanout memory region\n");
	ret = devm_add_action_or_reset(dev, lg1314_release_rmem, dev);
	if (ret)
		return ret;

	lg = devm_drm_dev_alloc(dev, &lg1314_drm_driver, struct lg1314_drm, drm);
	if (IS_ERR(lg))
		return PTR_ERR(lg);
	lg->dev = dev;
	platform_set_drvdata(pdev, lg);

	ret = drmm_mutex_init(&lg->drm, &lg->de_lock);
	if (ret)
		return ret;

	for (i = 0; i < LG1314_REG_COUNT; i++) {
		lg->mmio[i].phys = lg1314_regions[i].phys;
		lg->mmio[i].size = lg1314_regions[i].size;
		lg->mmio[i].virt = devm_ioremap(dev, lg1314_regions[i].phys,
						lg1314_regions[i].size);
		if (!lg->mmio[i].virt)
			return -ENOMEM;
	}

	ret = lg1314_get_micom(lg);
	if (ret)
		return ret;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	/* The DE firmware image and working-memory regions. */
	ret = lg1314_get_de_regions(lg);
	if (ret)
		goto err_micom;

	/* Preserve lxboot's channel-power register value. */
	lg->chpower_save = lg1314_rd(lg, LG1314_OVI_CHPOWER_REG);

	ret = lg1314_init_mode_config(lg);
	if (ret)
		goto err_video;

	lg->irq = platform_get_irq(pdev, 0);
	if (lg->irq < 0) {
		ret = lg->irq;
		goto err_video;
	}
	lg1314_osd_vsync_irq(lg, false);
	ret = devm_request_irq(dev, lg->irq, lg1314_vsync_irq, IRQF_SHARED,
			       "lg1314-vsync", lg);
	if (ret)
		goto err_video;

	ret = platform_get_irq(pdev, 1);
	if (ret < 0)
		goto err_video;
	ret = devm_request_irq(dev, ret, lg1314_video_tick_irq, IRQF_SHARED,
			       "lg1314-video-tick", lg);
	if (ret)
		goto err_video;

	/* DE IPC (firmware -> host): Dolby Vision record requests */
	ret = platform_get_irq_optional(pdev, 2);
	if (ret > 0)
		ret = lg1314_dovi_init(lg, ret);
	else if (ret == -ENXIO)
		ret = 0;
	if (ret)
		goto err_video;

	/*
	 * Evict any firmware framebuffer before we take over the display.
	 * lxboot leaves the OSD scanning out a linear buffer that the boot
	 * DT can expose as a simple-framebuffer (driven by simpledrm/fbcon);
	 * that buffer lives apart from our scanout pool, so remove all
	 * conflicting framebuffers by name rather than by our own aperture.
	 */
	ret = aperture_remove_all_conflicting_devices(lg1314_drm_driver.name);
	if (ret)
		goto err_video;

	ret = drm_dev_register(&lg->drm, 0);
	if (ret)
		goto err_video;
	lg1314_de_debugfs_init(lg);

	drm_client_setup(&lg->drm, NULL);
	return 0;

err_video:
	lg1314_dovi_fini(lg);
	lg1314_video_fini(lg);
err_micom:
	put_device(&lg->micom->dev);
	return ret;
}

static void lg1314_remove(struct platform_device *pdev)
{
	struct lg1314_drm *lg = platform_get_drvdata(pdev);

	drm_dev_unplug(&lg->drm);
	drm_atomic_helper_shutdown(&lg->drm);
	lg1314_dovi_fini(lg);
	lg1314_video_fini(lg);
	put_device(&lg->micom->dev);
}

static void lg1314_shutdown(struct platform_device *pdev)
{
	struct lg1314_drm *lg = platform_get_drvdata(pdev);

	drm_atomic_helper_shutdown(&lg->drm);
}

static const struct of_device_id lg1314_of_match[] = {
	{ .compatible = "lge,lg1314-drm" },
	{ }
};
MODULE_DEVICE_TABLE(of, lg1314_of_match);

static struct platform_driver lg1314_platform_driver = {
	.probe = lg1314_probe,
	.remove = lg1314_remove,
	.shutdown = lg1314_shutdown,
	.driver = {
		.name = "lg1314-drm",
		.of_match_table = lg1314_of_match,
	},
};
module_platform_driver(lg1314_platform_driver);

MODULE_DESCRIPTION("LG1314 OLED DRM driver");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE(DE_FW_NAME);
