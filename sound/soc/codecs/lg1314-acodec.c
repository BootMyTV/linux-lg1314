// SPDX-License-Identifier: GPL-2.0
/*
 * LG1314 internal audio codec: the headphone / line-out DAC.
 *
 * The audio DSPs' "sub" I2S output (PCMSUB) feeds an LG1152-style DAC whose
 * digital half sits in the MIP_DIG block (0xc910c300..) and analogue half in
 * CTOP_BMC (0xc930e470..). Its reset is bit 2 of the audio reset register in
 * BMC_SYN (0xc930e004), shared with the DSP clock setup. The setup sequences
 * are the vendor's (M19A0 mixed-IP InitCodecRegisters / StartCodec /
 * SetDACOutMute, 05.50 kernel). After the DAC, the headphone output has a
 * mute switch of its own, driven by the board (on the LJ9 by the standby
 * MICOM): mute-gpios.
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>

#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/soc.h>

/* MIP_DIG, relative to 0xc910c300 (LG1152_AUDIO_03..) */
#define DAC_DEM			0x08	/* dem0, mutec */
#define DAC_DIF			0x0c	/* test0, dif1, dif0 */
#define DAC_OUT_SEL		0x10
#define DAC_VOL			0x14	/* dac0_vol_con: attenuation, 0 = full */
#define  DAC_VOL_MASK		GENMASK(13, 0)
#define DAC_RATE		0x18
#define DAC_RATE_MUTE		0x1c
#define DAC_POP			0x20
#define  DAC_POP_MUTE_ENA	BIT(2)
#define DAC_POP_RATE		0x24
#define DAC_INVERT		0x3c
#define DAC_REG_40		0x40
#define DAC_REG_44		0x44
#define DAC_REG_48		0x48
#define DAC_REG_4C		0x4c
#define DAC_REG_50		0x50
#define DAC_REG_54		0x54
#define DAC_REG_58		0x58
#define DAC_REG_5C		0x5c
#define DAC_REG_60		0x60

/* CTOP_BMC analogue codec, relative to 0xc930e470 (acodec_00..05) */
#define ATOP(n)			(4 * (n))

/* BMC_SYN audio reset register: bit 2 holds the DAC in reset */
#define RST_DAC			BIT(2)

#define ACODEC_RAMP_US		200

struct lg1314_acodec {
	struct device *dev;
	void __iomem *dac;
	void __iomem *atop;
	void __iomem *rst;
	struct gpio_desc *mute_gpio;
	struct gpio_desc *hp_det_gpio;
	struct mutex lock;
	bool muted;

	struct snd_soc_jack jack;
	struct snd_soc_jack_pin jack_pin;
	struct snd_soc_jack_gpio jack_gpio;
};

static void rmw(void __iomem *reg, u32 clear, u32 set)
{
	writel((readl(reg) & ~clear) | set, reg);
}

/* The reset register is shared with the clock setup: whole-word RMW. */
static void dac_reset(struct lg1314_acodec *ac, bool assert)
{
	rmw(ac->rst, RST_DAC, assert ? RST_DAC : 0);
	udelay(ACODEC_RAMP_US);
}

static void dac_vol(struct lg1314_acodec *ac, u32 att)
{
	rmw(ac->dac + DAC_VOL, DAC_VOL_MASK, att);
	udelay(ACODEC_RAMP_US);
}

/* Vendor de-pop: release the reset at full attenuation, then ramp up. */
static void dac_unmute(struct lg1314_acodec *ac)
{
	int i;

	dac_reset(ac, true);
	dac_vol(ac, 0x3ff);
	dac_reset(ac, false);
	for (i = 4; i >= 0; i--)
		dac_vol(ac, i);
}

static void dac_mute(struct lg1314_acodec *ac)
{
	rmw(ac->dac + DAC_POP, 0, DAC_POP_MUTE_ENA);
	udelay(ACODEC_RAMP_US);
	dac_reset(ac, true);
	rmw(ac->dac + DAC_POP, DAC_POP_MUTE_ENA, 0);
}

static void acodec_init(struct lg1314_acodec *ac)
{
	void __iomem *d = ac->dac, *a = ac->atop;
	u32 v;

	/* InitCodecRegisters */
	rmw(a + ATOP(0), 0xff, 0xda);
	rmw(a + ATOP(1), 0x3ff, 0x244);
	rmw(a + ATOP(2), 0x0f, 0x10);
	rmw(a + ATOP(5), BIT(9), 0);
	rmw(d + DAC_REG_40, 0x0f, 0x02);
	rmw(d + DAC_REG_44, 0x3f, 0x40);
	rmw(d + DAC_REG_48, 0x3fff, 0x0001);
	rmw(d + DAC_REG_4C, 0x1ff, 0x008);
	rmw(d + DAC_REG_50, 0x1ff, 0x008);
	rmw(d + DAC_REG_54, 0, BIT(0));
	rmw(d + DAC_REG_58, BIT(0), 0);
	rmw(d + DAC_REG_5C, BIT(0), 0);
	rmw(a + ATOP(3), 0xf03e, 0xa004);
	v = readl(a + ATOP(4));
	v = (v & 0xccec8000) | 0x22224aa3;	/* the bits the vendor's RMW keeps */
	writel(v, a + ATOP(4));
	rmw(a + ATOP(5), 0x37, 0x24);
	rmw(d + DAC_DEM, 0x03, 0x0c);
	rmw(d + DAC_DIF, 0x3f, 0x20);
	rmw(d + DAC_OUT_SEL, 0x3f, 0);
	rmw(d + DAC_VOL, DAC_VOL_MASK, 0);
	rmw(d + DAC_RATE, 0x1ff, 0x008);
	rmw(d + DAC_RATE_MUTE, 0x1ff, 0x001);
	rmw(d + DAC_POP, BIT(0), 0x06);
	rmw(d + DAC_POP_RATE, 0x3fff, 0x0008);
	rmw(d + DAC_INVERT, 0x03, 0x04);
	rmw(d + DAC_REG_60, 0, BIT(0));
	rmw(d + DAC_POP, DAC_POP_MUTE_ENA, 0);

	/* StartCodec */
	rmw(a + ATOP(2), 0, 0x0f);
	rmw(a + ATOP(5), 0, BIT(9));
	rmw(a + ATOP(3), 0, 0x32);
	dac_unmute(ac);
}

/*
 * The DAC runs from probe on: idle, the DSP feeds it silence, and resetting
 * it at each stream start (the vendor's mute) glitches the start of the
 * audio. Streams only switch the mute after it.
 */
static void acodec_set_mute(struct lg1314_acodec *ac, bool mute)
{
	mutex_lock(&ac->lock);
	if (mute != ac->muted) {
		gpiod_set_value_cansleep(ac->mute_gpio, mute);
		ac->muted = mute;
	}
	mutex_unlock(&ac->lock);
}

static int acodec_mute_stream(struct snd_soc_dai *dai, int mute, int stream)
{
	acodec_set_mute(snd_soc_component_get_drvdata(dai->component), mute);
	return 0;
}

static int acodec_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_I2S)
		return -EINVAL;
	return 0;
}

static const struct snd_soc_dai_ops acodec_dai_ops = {
	.set_fmt = acodec_set_fmt,
	.mute_stream = acodec_mute_stream,
	.no_capture_mute = 1,
};

static struct snd_soc_dai_driver acodec_dai = {
	.name = "lg1314-acodec",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |
			   SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops = &acodec_dai_ops,
};

static const struct snd_soc_dapm_widget acodec_widgets[] = {
	SND_SOC_DAPM_DAC("DAC", "Playback", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUTPUT("HPOUT"),
};

static const struct snd_soc_dapm_route acodec_routes[] = {
	{ "HPOUT", NULL, "DAC" },
};

/* Plug detection, reported on the card as "Headphone Jack". */
static int acodec_probe(struct snd_soc_component *c)
{
	struct lg1314_acodec *ac = snd_soc_component_get_drvdata(c);
	int ret;

	if (!ac->hp_det_gpio)
		return 0;
	ac->jack_pin.pin = "HPOUT";	/* the DAPM output above */
	ac->jack_pin.mask = SND_JACK_HEADPHONE;
	ret = snd_soc_card_jack_new_pins(c->card, "Headphone Jack", SND_JACK_HEADPHONE,
					 &ac->jack, &ac->jack_pin, 1);
	if (ret)
		return ret;
	ac->jack_gpio.name = "Headphone detection";
	ac->jack_gpio.report = SND_JACK_HEADPHONE;
	ac->jack_gpio.desc = ac->hp_det_gpio;
	ac->jack_gpio.debounce_time = 150;
	return snd_soc_jack_add_gpios(&ac->jack, 1, &ac->jack_gpio);
}

static void acodec_remove(struct snd_soc_component *c)
{
	struct lg1314_acodec *ac = snd_soc_component_get_drvdata(c);

	if (ac->hp_det_gpio)
		snd_soc_jack_free_gpios(&ac->jack, 1, &ac->jack_gpio);
}

static const struct snd_soc_component_driver acodec_component = {
	.probe = acodec_probe,
	.remove = acodec_remove,
	.dapm_widgets = acodec_widgets,
	.num_dapm_widgets = ARRAY_SIZE(acodec_widgets),
	.dapm_routes = acodec_routes,
	.num_dapm_routes = ARRAY_SIZE(acodec_routes),
	.endianness = 1,
};

/* The pages are shared (video ADC, pin control, DSP clocks): map, don't claim. */
static void __iomem *acodec_map(struct platform_device *pdev, const char *name)
{
	struct resource *res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);

	if (!res)
		return IOMEM_ERR_PTR(-ENODEV);
	return devm_ioremap(&pdev->dev, res->start, resource_size(res)) ?:
		IOMEM_ERR_PTR(-ENOMEM);
}

static int lg1314_acodec_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct lg1314_acodec *ac;

	ac = devm_kzalloc(dev, sizeof(*ac), GFP_KERNEL);
	if (!ac)
		return -ENOMEM;
	ac->dev = dev;
	mutex_init(&ac->lock);
	platform_set_drvdata(pdev, ac);

	ac->dac = acodec_map(pdev, "dac");
	ac->atop = acodec_map(pdev, "atop");
	ac->rst = acodec_map(pdev, "reset");
	if (IS_ERR(ac->dac) || IS_ERR(ac->atop) || IS_ERR(ac->rst))
		return dev_err_probe(dev, -ENODEV, "missing dac/atop/reset registers\n");

	ac->mute_gpio = devm_gpiod_get_optional(dev, "mute", GPIOD_OUT_HIGH);
	if (IS_ERR(ac->mute_gpio))
		return dev_err_probe(dev, PTR_ERR(ac->mute_gpio), "mute gpio\n");
	ac->hp_det_gpio = devm_gpiod_get_optional(dev, "hp-det", GPIOD_IN);
	if (IS_ERR(ac->hp_det_gpio))
		return dev_err_probe(dev, PTR_ERR(ac->hp_det_gpio), "hp-det gpio\n");

	acodec_init(ac);
	ac->muted = true;

	return devm_snd_soc_register_component(dev, &acodec_component, &acodec_dai, 1);
}

static void lg1314_acodec_remove(struct platform_device *pdev)
{
	struct lg1314_acodec *ac = platform_get_drvdata(pdev);

	acodec_set_mute(ac, true);
	dac_mute(ac);
}

static const struct of_device_id lg1314_acodec_of_match[] = {
	{ .compatible = "lge,lg1314-acodec" },
	{ }
};
MODULE_DEVICE_TABLE(of, lg1314_acodec_of_match);

static struct platform_driver lg1314_acodec_driver = {
	.probe = lg1314_acodec_probe,
	.remove = lg1314_acodec_remove,
	.driver = {
		.name = "lg1314-acodec",
		.of_match_table = lg1314_acodec_of_match,
	},
};
module_platform_driver(lg1314_acodec_driver);

MODULE_DESCRIPTION("LG1314 internal audio codec (headphone DAC)");
MODULE_LICENSE("GPL");
