// SPDX-License-Identifier: GPL-2.0
/*
 * USB 2.0 host PHY driver for the LG LG1314 (M19A1) DTV SoC.
 *
 * Per external host port this manages VBUS (the "ctrl" GPIO), PHY reset
 * deassert (the "phyrst" register region + "phy-rst-msk") and, where present,
 * the GPIO pinmux (the "pinmux" region + "gpio-pinmux"). The remaining
 * PHY/host parameter programming the vendor driver performs is done by the boot
 * firmware; this covers what the generic PHY framework needs to power a port.
 */
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>

struct lg1314_usb_phy {
	struct gpio_desc *vbus;
	void __iomem *phyrst;
	void __iomem *pinmux;
	u32 phyrst_msk[2];	/* <mask, invert> */
	u32 pinmux_msk[2];	/* <mask, invert> */
};

/* enable=1 asserts the field (or clears it, when the "invert" cell is set) */
static void lg1314_field_set(void __iomem *addr, u32 mask, u32 inv, bool enable)
{
	u32 v = readl(addr);

	if (inv)
		v = enable ? (v & ~mask) : (v | mask);
	else
		v = enable ? (v | mask) : (v & ~mask);

	writel(v, addr);
}

static int lg1314_usb_phy_power_on(struct phy *phy)
{
	struct lg1314_usb_phy *p = phy_get_drvdata(phy);

	if (p->pinmux)
		lg1314_field_set(p->pinmux, p->pinmux_msk[0], p->pinmux_msk[1],
				 true);
	if (p->phyrst)
		lg1314_field_set(p->phyrst, p->phyrst_msk[0], p->phyrst_msk[1],
				 true);

	gpiod_set_value_cansleep(p->vbus, 1);

	return 0;
}

static int lg1314_usb_phy_power_off(struct phy *phy)
{
	struct lg1314_usb_phy *p = phy_get_drvdata(phy);

	gpiod_set_value_cansleep(p->vbus, 0);

	return 0;
}

static const struct phy_ops lg1314_usb_phy_ops = {
	.power_on = lg1314_usb_phy_power_on,
	.power_off = lg1314_usb_phy_power_off,
	.owner = THIS_MODULE,
};

/*
 * Map a named reg region. Uses devm_ioremap() rather than an exclusive
 * request, because the "phyrst" region is shared between the host ports.
 */
static void __iomem *lg1314_usb_phy_iomap(struct device *dev,
					  struct device_node *np, int idx)
{
	struct resource res;

	if (of_address_to_resource(np, idx, &res))
		return NULL;

	return devm_ioremap(dev, res.start, resource_size(&res));
}

static int lg1314_usb_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct lg1314_usb_phy *p;
	struct phy_provider *provider;
	struct phy *phy;
	int idx;

	p = devm_kzalloc(dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	p->vbus = devm_gpiod_get(dev, "ctrl", GPIOD_OUT_LOW);
	if (IS_ERR(p->vbus))
		return dev_err_probe(dev, PTR_ERR(p->vbus),
				     "missing ctrl (vbus) gpio\n");

	idx = of_property_match_string(np, "reg-names", "phyrst");
	if (idx >= 0 &&
	    !of_property_read_u32_array(np, "phy-rst-msk", p->phyrst_msk, 2)) {
		p->phyrst = lg1314_usb_phy_iomap(dev, np, idx);
		if (!p->phyrst)
			return -ENOMEM;
	}

	idx = of_property_match_string(np, "reg-names", "pinmux");
	if (idx >= 0 &&
	    !of_property_read_u32_array(np, "gpio-pinmux", p->pinmux_msk, 2)) {
		p->pinmux = lg1314_usb_phy_iomap(dev, np, idx);
		if (!p->pinmux)
			return -ENOMEM;
	}

	phy = devm_phy_create(dev, np, &lg1314_usb_phy_ops);
	if (IS_ERR(phy))
		return dev_err_probe(dev, PTR_ERR(phy), "failed to create phy\n");

	phy_set_drvdata(phy, p);

	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR(provider))
		return PTR_ERR(provider);

	dev_info(dev, "registered USB host PHY\n");

	return 0;
}

static const struct of_device_id lg1314_usb_phy_of_match[] = {
	{ .compatible = "lge,lg1k-usb2phy" },
	{ .compatible = "lge,lg1k-usb1phy" },
	{ }
};
MODULE_DEVICE_TABLE(of, lg1314_usb_phy_of_match);

static struct platform_driver lg1314_usb_phy_driver = {
	.probe = lg1314_usb_phy_probe,
	.driver = {
		.name = "lg1314-usb-phy",
		.of_match_table = lg1314_usb_phy_of_match,
	},
};
module_platform_driver(lg1314_usb_phy_driver);

MODULE_DESCRIPTION("LG LG1314 USB host PHY driver");
MODULE_LICENSE("GPL");
