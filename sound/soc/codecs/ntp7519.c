// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for the NeoFidelity NTP7519 audio amplifier
 *
 * The NTP7519 is a stereo I2S class-D amplifier with its own DSP (parametric
 * and all-pass EQ, DRC). Its tuning is per speaker system, so the whole setup
 * comes from a firmware file (ntpfw format) named by the "firmware-name"
 * property. The register layout otherwise matches the NTP8918.
 *
 * Several amplifiers may share one reset line (e.g. a TV's main and woofer
 * amplifiers); each loads its firmware once the line is released.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#include "ntpfw.h"

#define NTP7519_MASTER_VOL		0x0c
#define  NTP7519_MASTER_VOL_MIN		0x04	/* -125.5 dB, 0.5 dB steps */
#define  NTP7519_MASTER_VOL_DEFAULT	0xff	/* 0 dB: the vendor runs the amps flat, volume in the DSP */
#define NTP7519_EQ_CTRL			0x0e	/* 0x0e-0x15: PEQ / AP-EQ enables */
#define  NTP7519_EQ_CTRL_NUM		8
#define NTP7519_SOFT_MUTE		0x33
#define  NTP7519_SOFT_MUTE_ALL		(BIT(0) | BIT(1))
#define NTP7519_PWM_SWITCH		0x34
#define NTP7519_PWM_MASK_CTRL0		0x35
#define NTP7519_BANK			0x7e
#define NTP7519_REG_MAX			0x7f

#define NTP7519_FW_MAGIC		0x37353139	/* "7519" */

struct ntp7519_priv {
	struct i2c_client *i2c;
	struct regmap *regmap;
	struct reset_control *reset;

	struct mutex lock;	/* volume and mute */
	unsigned int vol;	/* master volume register value */
	bool muted;
	bool eq;
	u8 eq_ctrl[NTP7519_EQ_CTRL_NUM];	/* as the firmware set them */
};

/*
 * Writing the master volume (like switching the PWM outputs on) clears the
 * soft mute. So while muted the volume is only remembered, and unmuting writes
 * it before releasing the mute.
 */
static int ntp7519_apply(struct ntp7519_priv *ntp)
{
	int ret;

	if (ntp->muted)
		return regmap_write(ntp->regmap, NTP7519_SOFT_MUTE, NTP7519_SOFT_MUTE_ALL);
	ret = regmap_write(ntp->regmap, NTP7519_MASTER_VOL, ntp->vol);
	if (ret)
		return ret;
	return regmap_write(ntp->regmap, NTP7519_SOFT_MUTE, 0);
}

static int ntp7519_vol_get(struct snd_kcontrol *kc, struct snd_ctl_elem_value *uc)
{
	struct ntp7519_priv *ntp = snd_soc_component_get_drvdata(snd_kcontrol_chip(kc));

	mutex_lock(&ntp->lock);
	uc->value.integer.value[0] = ntp->vol - NTP7519_MASTER_VOL_MIN;
	mutex_unlock(&ntp->lock);
	return 0;
}

static int ntp7519_vol_put(struct snd_kcontrol *kc, struct snd_ctl_elem_value *uc)
{
	struct ntp7519_priv *ntp = snd_soc_component_get_drvdata(snd_kcontrol_chip(kc));
	long val = uc->value.integer.value[0];
	int ret = 0;

	if (val < 0 || val > 0xff - NTP7519_MASTER_VOL_MIN)
		return -EINVAL;
	mutex_lock(&ntp->lock);
	if (ntp->vol != val + NTP7519_MASTER_VOL_MIN) {
		ntp->vol = val + NTP7519_MASTER_VOL_MIN;
		ret = ntp7519_apply(ntp);
		if (!ret)
			ret = 1;
	}
	mutex_unlock(&ntp->lock);
	return ret;
}

/* EQ bypass: the vendor's "EQ off" clears the enables, "on" restores them. */
static int ntp7519_eq_get(struct snd_kcontrol *kc, struct snd_ctl_elem_value *uc)
{
	struct ntp7519_priv *ntp = snd_soc_component_get_drvdata(snd_kcontrol_chip(kc));

	mutex_lock(&ntp->lock);
	uc->value.integer.value[0] = ntp->eq;
	mutex_unlock(&ntp->lock);
	return 0;
}

static int ntp7519_eq_put(struct snd_kcontrol *kc, struct snd_ctl_elem_value *uc)
{
	struct ntp7519_priv *ntp = snd_soc_component_get_drvdata(snd_kcontrol_chip(kc));
	bool on = uc->value.integer.value[0];
	int i, ret = 0;

	mutex_lock(&ntp->lock);
	if (ntp->eq == on)
		goto out;
	for (i = 0; i < NTP7519_EQ_CTRL_NUM && !ret; i++)
		ret = regmap_write(ntp->regmap, NTP7519_EQ_CTRL + i,
				   on ? ntp->eq_ctrl[i] : 0);
	if (!ret && ntp->muted)
		ret = ntp7519_apply(ntp);
	if (!ret) {
		ntp->eq = on;
		ret = 1;
	}
out:
	mutex_unlock(&ntp->lock);
	return ret;
}

static const DECLARE_TLV_DB_SCALE(ntp7519_vol_tlv, -12550, 50, 0);

static const struct snd_kcontrol_new ntp7519_controls[] = {
	SOC_SINGLE_EXT_TLV("Playback Volume", SND_SOC_NOPM, 0,
			   0xff - NTP7519_MASTER_VOL_MIN, 0,
			   ntp7519_vol_get, ntp7519_vol_put, ntp7519_vol_tlv),
	SOC_SINGLE_BOOL_EXT("EQ Switch", 0, ntp7519_eq_get, ntp7519_eq_put),
};

static const struct snd_soc_dapm_widget ntp7519_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("AIFIN", "Playback", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUTPUT("OUT1"),
	SND_SOC_DAPM_OUTPUT("OUT2"),
};

static const struct snd_soc_dapm_route ntp7519_dapm_routes[] = {
	{ "OUT1", NULL, "AIFIN" },
	{ "OUT2", NULL, "AIFIN" },
};

static const struct snd_soc_component_driver ntp7519_component = {
	.controls = ntp7519_controls,
	.num_controls = ARRAY_SIZE(ntp7519_controls),
	.dapm_widgets = ntp7519_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(ntp7519_dapm_widgets),
	.dapm_routes = ntp7519_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(ntp7519_dapm_routes),
	.endianness = 1,
};

static int ntp7519_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	/* The firmware leaves the input in I2S slave mode. */
	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_I2S)
		return -EINVAL;
	return 0;
}

static int ntp7519_mute_stream(struct snd_soc_dai *dai, int mute, int stream)
{
	struct ntp7519_priv *ntp = snd_soc_component_get_drvdata(dai->component);
	int ret;

	mutex_lock(&ntp->lock);
	ntp->muted = mute;
	ret = ntp7519_apply(ntp);
	mutex_unlock(&ntp->lock);
	return ret;
}

static const struct snd_soc_dai_ops ntp7519_dai_ops = {
	.set_fmt = ntp7519_set_fmt,
	.mute_stream = ntp7519_mute_stream,
	.no_capture_mute = 1,
};

static struct snd_soc_dai_driver ntp7519_dai = {
	.name = "ntp7519-amplifier",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |
			   SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops = &ntp7519_dai_ops,
};

static const struct regmap_config ntp7519_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = NTP7519_REG_MAX,
	.cache_type = REGCACHE_NONE,
};

/*
 * After the firmware (the vendor's setup sequence, which ends at 0 dB): a safe
 * volume, outputs switched on, then muted (the first two clear the mute).
 */
static const struct reg_sequence ntp7519_post_fw[] = {
	{ NTP7519_BANK, 0 },
	{ NTP7519_MASTER_VOL, NTP7519_MASTER_VOL_DEFAULT },
	{ NTP7519_PWM_MASK_CTRL0, 0x04 },
	{ NTP7519_PWM_SWITCH, 0x00 },
	{ NTP7519_SOFT_MUTE, NTP7519_SOFT_MUTE_ALL },
};

static int ntp7519_i2c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct ntp7519_priv *ntp;
	unsigned int val;
	const char *fw;
	int i, ret;

	ntp = devm_kzalloc(dev, sizeof(*ntp), GFP_KERNEL);
	if (!ntp)
		return -ENOMEM;
	ntp->i2c = i2c;
	mutex_init(&ntp->lock);
	ntp->vol = NTP7519_MASTER_VOL_DEFAULT;
	ntp->muted = true;
	i2c_set_clientdata(i2c, ntp);

	ret = device_property_read_string(dev, "firmware-name", &fw);
	if (ret)
		return dev_err_probe(dev, ret, "no firmware-name\n");

	ntp->regmap = devm_regmap_init_i2c(i2c, &ntp7519_regmap);
	if (IS_ERR(ntp->regmap))
		return dev_err_probe(dev, PTR_ERR(ntp->regmap), "regmap init failed\n");

	ntp->reset = devm_reset_control_get_optional_shared(dev, NULL);
	if (IS_ERR(ntp->reset))
		return dev_err_probe(dev, PTR_ERR(ntp->reset), "failed to get reset\n");

	/*
	 * Pulse the reset (low >= 20 ms, as the vendor does). With a shared
	 * line only the first amplifier's pulse reaches the pin: the line stays
	 * released while another user has it deasserted.
	 */
	ret = reset_control_deassert(ntp->reset);
	if (ret)
		return ret;
	reset_control_assert(ntp->reset);
	fsleep(20000);
	ret = reset_control_deassert(ntp->reset);
	if (ret)
		return ret;
	fsleep(20000);

	ret = ntpfw_load(i2c, fw, NTP7519_FW_MAGIC);
	if (ret)
		goto err_reset;
	ret = regmap_multi_reg_write(ntp->regmap, ntp7519_post_fw,
				     ARRAY_SIZE(ntp7519_post_fw));
	/* one at a time: reads don't auto-increment */
	for (i = 0; i < NTP7519_EQ_CTRL_NUM && !ret; i++) {
		ret = regmap_read(ntp->regmap, NTP7519_EQ_CTRL + i, &val);
		ntp->eq_ctrl[i] = val;
	}
	if (ret)
		goto err_reset;
	ntp->eq = true;

	ret = devm_snd_soc_register_component(dev, &ntp7519_component, &ntp7519_dai, 1);
	if (ret) {
		dev_err_probe(dev, ret, "failed to register component\n");
		goto err_reset;
	}
	dev_info(dev, "loaded %s\n", fw);
	return 0;

err_reset:
	reset_control_assert(ntp->reset);
	return ret;
}

static void ntp7519_i2c_remove(struct i2c_client *i2c)
{
	struct ntp7519_priv *ntp = i2c_get_clientdata(i2c);

	regmap_write(ntp->regmap, NTP7519_SOFT_MUTE, NTP7519_SOFT_MUTE_ALL);
	reset_control_assert(ntp->reset);
}

static const struct i2c_device_id ntp7519_i2c_id[] = {
	{ "ntp7519" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ntp7519_i2c_id);

static const struct of_device_id ntp7519_of_match[] = {
	{ .compatible = "neofidelity,ntp7519" },
	{ }
};
MODULE_DEVICE_TABLE(of, ntp7519_of_match);

static struct i2c_driver ntp7519_i2c_driver = {
	.probe = ntp7519_i2c_probe,
	.remove = ntp7519_i2c_remove,
	.id_table = ntp7519_i2c_id,
	.driver = {
		.name = "ntp7519",
		.of_match_table = ntp7519_of_match,
	},
};
module_i2c_driver(ntp7519_i2c_driver);

MODULE_DESCRIPTION("NeoFidelity NTP7519 audio amplifier driver");
MODULE_LICENSE("GPL");
