// SPDX-License-Identifier: GPL-2.0
/*
 * LG1314 (lg1k / M19) SoC temperature sensor.
 *
 * A single on-die TSADC in the CPU power block. Enable it, then read a raw
 * count and convert with a linear fit. The stock vendor PM code (pm_m19_avs)
 * picks the fit coefficients from a per-die silicon bin (fast/nominal/slow,
 * selected by measured leakage); this driver uses the nominal bin, which is
 * accurate enough to drive thermal throttling:
 *
 *	temp_C = aparam * raw / 10000 - bparam	(nominal: aparam=157, bparam=352)
 *
 * The per-unit bin could be recovered later (the stock cmdline carries it in
 * kdrv_lg1314.m19_avsinfo) to tighten the reading.
 */
#include <linux/bits.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>

#define TS_CTRL		0x00
#define  TS_EN		BIT(0)
#define TS_RAW		0x14

/* Nominal-bin linear fit; see above. */
#define TS_APARAM	157
#define TS_BPARAM	352

struct lg1314_tsensor {
	void __iomem	*base;
};

static int lg1314_ts_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct lg1314_tsensor *ts = thermal_zone_device_priv(tz);
	u32 raw = readl(ts->base + TS_RAW);

	/* The TSADC needs a moment after enable before it produces a count. */
	if (!raw)
		return -EAGAIN;

	/* temp_C = APARAM*raw/10000 - BPARAM, returned in millidegrees. */
	*temp = (int)(((s64)TS_APARAM * raw) / 10) - TS_BPARAM * 1000;
	return 0;
}

static const struct thermal_zone_device_ops lg1314_ts_ops = {
	.get_temp = lg1314_ts_get_temp,
};

static int lg1314_tsensor_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct thermal_zone_device *tz;
	struct lg1314_tsensor *ts;

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ts->base))
		return PTR_ERR(ts->base);

	writel(TS_EN, ts->base + TS_CTRL);

	tz = devm_thermal_of_zone_register(dev, 0, ts, &lg1314_ts_ops);
	if (IS_ERR(tz))
		return dev_err_probe(dev, PTR_ERR(tz),
				     "failed to register thermal zone\n");

	return 0;
}

static const struct of_device_id lg1314_tsensor_of_match[] = {
	{ .compatible = "lge,lg1314-tsensor" },
	{ }
};
MODULE_DEVICE_TABLE(of, lg1314_tsensor_of_match);

static struct platform_driver lg1314_tsensor_driver = {
	.probe = lg1314_tsensor_probe,
	.driver = {
		.name = "lg1314-tsensor",
		.of_match_table = lg1314_tsensor_of_match,
	},
};
module_platform_driver(lg1314_tsensor_driver);

MODULE_DESCRIPTION("LG1314 (M19) SoC temperature sensor");
MODULE_LICENSE("GPL");
