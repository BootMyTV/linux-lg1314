// SPDX-License-Identifier: GPL-2.0
/*
 * Output ports of the LG1314 standby MICOM, as GPIO lines.
 *
 * The MICOM drives a few board signals for the SoC. Each port is set with its
 * own two-byte command {opcode, level}; there is no read-back, so the last
 * level set is reported. The lines are registered on the MICOM's DT node.
 *
 *   line 0  headphone mute (vendor port 0x2104, opcode 0x38; 1 = muted)
 */

#include <linux/gpio/driver.h>
#include <linux/mfd/lg1314-micom.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>

static const u8 micom_gpio_opcodes[] = {
	0x38,	/* headphone mute */
};

#define MICOM_GPIO_NUM	ARRAY_SIZE(micom_gpio_opcodes)

struct micom_gpio {
	struct gpio_chip gc;
	struct lg1314_micom *micom;
	struct mutex lock;
	unsigned long state;
};

static int micom_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	return GPIO_LINE_DIRECTION_OUT;
}

static int micom_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct micom_gpio *mg = gpiochip_get_data(gc);

	return test_bit(offset, &mg->state);
}

static int micom_gpio_set(struct gpio_chip *gc, unsigned int offset, int value)
{
	struct micom_gpio *mg = gpiochip_get_data(gc);
	const u8 cmd[] = { micom_gpio_opcodes[offset], !!value };
	int ret;

	mutex_lock(&mg->lock);
	ret = lg1314_micom_cmd(mg->micom, cmd, sizeof(cmd));
	if (!ret)
		assign_bit(offset, &mg->state, value);
	mutex_unlock(&mg->lock);
	return ret;
}

static int micom_gpio_direction_output(struct gpio_chip *gc, unsigned int offset,
				       int value)
{
	return micom_gpio_set(gc, offset, value);
}

static int micom_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct micom_gpio *mg;

	mg = devm_kzalloc(dev, sizeof(*mg), GFP_KERNEL);
	if (!mg)
		return -ENOMEM;
	mg->micom = dev_get_drvdata(dev->parent);
	mutex_init(&mg->lock);
	/* The vendor keeps the headphone output muted unless it is in use. */
	mg->state = BIT(0);

	mg->gc.label = "lg1314-micom";
	mg->gc.parent = dev;
	mg->gc.fwnode = dev_fwnode(dev->parent);
	mg->gc.owner = THIS_MODULE;
	mg->gc.base = -1;
	mg->gc.ngpio = MICOM_GPIO_NUM;
	mg->gc.can_sleep = true;
	mg->gc.get_direction = micom_gpio_get_direction;
	mg->gc.direction_output = micom_gpio_direction_output;
	mg->gc.get = micom_gpio_get;
	mg->gc.set = micom_gpio_set;

	return devm_gpiochip_add_data(dev, &mg->gc, mg);
}

static struct platform_driver micom_gpio_driver = {
	.probe = micom_gpio_probe,
	.driver = {
		.name = "lg1314-micom-gpio",
	},
};
module_platform_driver(micom_gpio_driver);

MODULE_DESCRIPTION("LG1314 standby MICOM output ports");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:lg1314-micom-gpio");
