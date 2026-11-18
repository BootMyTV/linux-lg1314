// SPDX-License-Identifier: GPL-2.0
/* DRM/KMS driver for the LG1314 display pipeline (LG OLED55B9PLA). */

#include <linux/dma-mapping.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>

#include <drm/clients/drm_client_setup.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>

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

/* ---------------------------------------------------------------- KMS pipe -- */

static const u32 lg1314_formats[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
};

static void lg1314_pipe_enable(struct drm_simple_display_pipe *pipe,
			       struct drm_crtc_state *crtc_state,
			       struct drm_plane_state *plane_state)
{
	struct lg1314_drm *lg = to_lg1314(pipe->crtc.dev);
	struct drm_framebuffer *fb = plane_state->fb;
	int ret;

	ret = lg1314_panel_vcc(lg, true);
	if (ret)
		return;

	lg1314_osd_init(lg);

	ret = lg1314_ovi_enable(lg);
	if (ret)
		goto err_vcc;

	ret = lg1314_de_load_fw(lg);
	if (ret)
		goto err_ovi;

	lg1314_osd_enable(lg, true);
	if (fb)
		lg1314_osd_flip(lg, drm_fb_dma_get_gem_addr(fb, plane_state, 0),
				fb->pitches[0], fb->width, fb->height,
				!fb->format->has_alpha);

	ret = lg1314_panel_evdd(lg, true);
	if (ret)
		goto err_ovi;

	lg->enabled = true;
	return;

err_ovi:
	lg1314_ovi_disable(lg);
err_vcc:
	lg1314_panel_vcc(lg, false);
}

static void lg1314_pipe_disable(struct drm_simple_display_pipe *pipe)
{
	struct lg1314_drm *lg = to_lg1314(pipe->crtc.dev);

	if (!lg->enabled)
		return;
	lg->enabled = false;

	lg1314_osd_enable(lg, false);
	lg1314_ovi_disable(lg);
	lg1314_panel_evdd(lg, false);
	lg1314_panel_vcc(lg, false);
}

static void lg1314_pipe_update(struct drm_simple_display_pipe *pipe,
			       struct drm_plane_state *old)
{
	struct lg1314_drm *lg = to_lg1314(pipe->crtc.dev);
	struct drm_plane_state *state = pipe->plane.state;
	struct drm_framebuffer *fb = state->fb;

	if (!lg->enabled || !fb)
		return;

	lg1314_osd_flip(lg, drm_fb_dma_get_gem_addr(fb, state, 0),
			fb->pitches[0], fb->width, fb->height,
			!fb->format->has_alpha);
}

static const struct drm_simple_display_pipe_funcs lg1314_pipe_funcs = {
	.enable = lg1314_pipe_enable,
	.disable = lg1314_pipe_disable,
	.update = lg1314_pipe_update,
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

static const struct drm_mode_config_funcs lg1314_mode_config_funcs = {
	.fb_create = drm_gem_fb_create,
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

	drm->mode_config.min_width = LG1314_FB_MIN_W;
	drm->mode_config.max_width = LG1314_PANEL_W;
	drm->mode_config.min_height = LG1314_FB_MIN_H;
	drm->mode_config.max_height = LG1314_PANEL_H;
	drm->mode_config.funcs = &lg1314_mode_config_funcs;

	drm_connector_helper_add(&lg->connector, &lg1314_conn_helper_funcs);
	ret = drm_connector_init(drm, &lg->connector, &lg1314_conn_funcs,
				 DRM_MODE_CONNECTOR_DPI);
	if (ret)
		return ret;

	ret = drm_simple_display_pipe_init(drm, &lg->pipe, &lg1314_pipe_funcs,
					   lg1314_formats,
					   ARRAY_SIZE(lg1314_formats), NULL,
					   &lg->connector);
	if (ret)
		return ret;

	drm_mode_config_reset(drm);
	return 0;
}

static int lg1314_get_de_fw_region(struct lg1314_drm *lg)
{
	struct device_node *np;
	struct resource res;
	int ret;

	np = of_parse_phandle(lg->dev->of_node, "memory-region", 1);
	if (!np)
		return dev_err_probe(lg->dev, -EINVAL, "missing de-fw region\n");

	ret = of_address_to_resource(np, 0, &res);
	of_node_put(np);
	if (ret)
		return ret;

	lg->de_fw_phys = res.start;
	lg->de_fw_size = resource_size(&res);
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

static int lg1314_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct lg1314_drm *lg;
	int ret, i;

	lg = devm_drm_dev_alloc(dev, &lg1314_drm_driver, struct lg1314_drm, drm);
	if (IS_ERR(lg))
		return PTR_ERR(lg);
	lg->dev = dev;
	platform_set_drvdata(pdev, lg);

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

	/* Scanout buffers come from the "scanout" pool (idx 0). */
	ret = of_reserved_mem_device_init_by_idx(dev, dev->of_node, 0);
	if (ret)
		return dev_err_probe(dev, ret, "no scanout memory region\n");

	/* The DE firmware region (idx 1) is fixed at its embedded base. */
	ret = lg1314_get_de_fw_region(lg);
	if (ret)
		goto err_micom;

	/* Preserve lxboot's channel-power register value. */
	lg->chpower_save = lg1314_rd(lg, LG1314_OVI_CHPOWER_REG);

	ret = lg1314_init_mode_config(lg);
	if (ret)
		goto err_micom;

	ret = drm_dev_register(&lg->drm, 0);
	if (ret)
		goto err_micom;

	drm_client_setup(&lg->drm, NULL);
	return 0;

err_micom:
	put_device(&lg->micom->dev);
	return ret;
}

static void lg1314_remove(struct platform_device *pdev)
{
	struct lg1314_drm *lg = platform_get_drvdata(pdev);

	drm_dev_unplug(&lg->drm);
	drm_atomic_helper_shutdown(&lg->drm);
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
