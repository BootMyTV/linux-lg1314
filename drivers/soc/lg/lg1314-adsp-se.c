// SPDX-License-Identifier: GPL-2.0
/*
 * LG1314 audio DSPs: the speaker chain through LG's sound engine (SE), as stock
 * builds it on DSP1 with the "legacy" modules (by fixed module id, through the
 * DSP's connection manager):
 *
 *   shared buffer -> PCMCVT_P13 -> SE -> OUT_SPK -> PCM (I2S to the amplifiers)
 *
 * (Stock reads the system mix through PCMREN_P0 / DAP before PCMCVT_P13; with
 * Dolby processing off the SE path is mixer -> SE.) PCMCVT_P13 takes the stream
 * format by command, so the buffer carries plain 2 x s32 frames.
 *
 * The SE's tuning lives in memory: one block per function in the SE region,
 * announced with SE_CMD_SET_PARAM {fn, init/var}, plus a mode word
 * (SE_CMD_SET_MODE) that switches functions on. The blocks are what tvservice
 * writes for this model, prepared offline from the firmware
 * ("lg1314/lgse.bin", see cfw/lgse_blob.py): a base set (initialisation, volume
 * at 0 dB: the volume is the player's) and one set per sound mode.
 */

#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#include <sound/control.h>
#include <sound/soc.h>

#include "lg1314-adsp.h"

#define SE_DSP			1
#define SE_FW			"lg1314/lgse.bin"
#define SE_DEFAULT_MODE		"Standard"

/* legacy module ids (05.50 kernel's adec_module_def) and the uids stock gives them */
#define MOD_PCMCVT_P13		0x41
#define MOD_SE			0x61
#define MOD_OUT_SPK		0x64
#define MOD_PCM			0x78
#define UID_PCMCVT_P13		0x03260001
#define UID_SE			0x03370001
#define UID_OUT_SPK		0x033a0001
#define UID_PCM			0x034a0001

/* connection manager (module_cmd_connection.h), addressed to the DSP's manager */
#define CM_CMD_CONNECT		ADSP_CMD(0xf000, 16)	/* {in mod, in port, out mod, out port} */
#define CM_CMD_ALLOC_BUF	ADSP_CMD(0xf010, 20)	/* {mod, port, offset, size, cowork} */
#define CM_CMD_CREATE_MODULE	ADSP_CMD(0xf020, 8)	/* {id, uid} */

#define PCMCVT_CMD_SET_ALLFMT	ADSP_CMD(0x5002, 32)
#define SE_CMD_SET_MODE		ADSP_CMD(0x5600, 12)	/* {mode, deq, peq} */
#define SE_CMD_SET_PARAM	ADSP_CMD(0x5601, 8)	/* {fn, 0 init / 1 var} */
#define SE_CMD_SET_BYPASS	ADSP_CMD(0x5602, 4)
#define SE_CMD_SET_OUTCH	ADSP_CMD(0x5603, 4)
#define SE_CMD_SET_INOUTFS	ADSP_CMD(0x5604, 8)
#define SE_CMD_SET_INP_CH_CONFIG ADSP_CMD(0x5607, 8)	/* {channels, acmod} */
#define OUTCTRL_CMD_SET_ALL	ADSP_CMD(0x5700, 28)
#define OUTCTRL_CMD_SET_CHANNEL	ADSP_CMD(0x5706, 4)
#define PCMOUT_CMD_SET_SPKCH	ADSP_CMD(0x5900, 4)
#define PCMOUT_CMD_SET_SPKFS	ADSP_CMD(0x5903, 4)

#define SE_ACMOD_STEREO		2
#define OUTCTRL_GAIN_0DB	0x800000

/* lgse.bin */
struct se_fw_set {
	char name[16];
	__le32 mode[3];
	__le32 records;
} __packed;

struct se_fw_rec {
	__le16 fn;
	__le16 type;
	__le16 offset;		/* u32 index in the SE region */
	__le16 words;
	__le32 data[];
} __packed;

struct adsp_se_set {
	const char *name;
	u32 mode[3];
	u32 records;
	const struct se_fw_rec *rec;
};

struct adsp_se {
	struct lg1314_adsp *adsp;
	const struct firmware *fw;
	struct adsp_se_set *sets;	/* [0] base, then the sound modes */
	unsigned int nsets;
	const char **names;		/* sound mode names, for the control */

	struct mutex lock;		/* the settings below */
	unsigned int mode;		/* index into sets[1..] */
	bool enabled;
};

static int se_cmd(struct adsp_se *se, u32 target, u32 cmd, const void *param, u32 len)
{
	int ret = adsp_imc_command(se->adsp, SE_DSP, target, cmd, param, len, NULL, 0, 1000);

	if (ret)
		dev_err(se->adsp->dev, "SE chain: %02x cmd %08x: %d\n", target, cmd, ret);
	return ret;
}

static int se_parse(struct adsp_se *se)
{
	const u8 *p = se->fw->data, *end = p + se->fw->size;
	unsigned int i, r;
	u32 n;

	if (se->fw->size < 12 || memcmp(p, "LGSE", 4) || get_unaligned_le32(p + 4) != 1)
		return -EINVAL;
	n = get_unaligned_le32(p + 8);
	if (n < 2 || n > 32)
		return -EINVAL;
	p += 12;
	se->sets = devm_kcalloc(se->adsp->dev, n, sizeof(*se->sets), GFP_KERNEL);
	se->names = devm_kcalloc(se->adsp->dev, n - 1, sizeof(*se->names), GFP_KERNEL);
	if (!se->sets || !se->names)
		return -ENOMEM;
	for (i = 0; i < n; i++) {
		const struct se_fw_set *s = (const void *)p;
		struct adsp_se_set *set = &se->sets[i];
		char *name;

		if (end - p < sizeof(*s))
			return -EINVAL;
		name = devm_kzalloc(se->adsp->dev, sizeof(s->name) + 1, GFP_KERNEL);
		if (!name)
			return -ENOMEM;
		memcpy(name, s->name, sizeof(s->name));
		set->name = name;
		for (r = 0; r < 3; r++)
			set->mode[r] = le32_to_cpu(s->mode[r]);
		set->records = le32_to_cpu(s->records);
		p += sizeof(*s);
		set->rec = (const void *)p;
		for (r = 0; r < set->records; r++) {
			const struct se_fw_rec *rec = (const void *)p;
			u32 words;

			if (end - p < sizeof(*rec))
				return -EINVAL;
			words = le16_to_cpu(rec->words);
			if (end - p < sizeof(*rec) + 4 * words ||
			    4 * (le16_to_cpu(rec->offset) + words) > MEM_SE_SIZE)
				return -EINVAL;
			p += sizeof(*rec) + 4 * words;
		}
		if (i)
			se->names[i - 1] = set->name;
	}
	se->nsets = n;
	return 0;
}

/* Write a set's blocks, announce each, then its mode word (unless bypassed). */
static int se_apply(struct adsp_se *se, const struct adsp_se_set *set)
{
	const u8 *p = (const u8 *)set->rec;
	u32 *mem = se->adsp->mem + MEM_SE;
	unsigned int r;
	int ret;

	for (r = 0; r < set->records; r++) {
		const struct se_fw_rec *rec = (const void *)p;
		u32 words = le16_to_cpu(rec->words), i;
		u32 param[2] = { le16_to_cpu(rec->fn), le16_to_cpu(rec->type) };

		for (i = 0; i < words; i++)
			mem[le16_to_cpu(rec->offset) + i] = le32_to_cpu(rec->data[i]);
		wmb();
		ret = se_cmd(se, MOD_SE, SE_CMD_SET_PARAM, param, sizeof(param));
		if (ret)
			return ret;
		p += sizeof(*rec) + 4 * words;
	}
	return se_cmd(se, MOD_SE, SE_CMD_SET_MODE, set->mode, sizeof(set->mode));
}

static int se_create(struct adsp_se *se, u32 id, u32 uid)
{
	u32 param[2] = { id, uid };

	return se_cmd(se, ADSP_MOD_MAN_DSP(SE_DSP), CM_CMD_CREATE_MODULE, param, sizeof(param));
}

static int se_connect(struct adsp_se *se, u32 out, u32 in)
{
	u32 param[4] = { in, 0, out, 0 };

	return se_cmd(se, ADSP_MOD_MAN_DSP(SE_DSP), CM_CMD_CONNECT, param, sizeof(param));
}

/*
 * Load the tuning and build the chain (not started). Returns the module that
 * reads the shared buffer. -ENOENT: no tuning file, use the plain chain.
 */
int adsp_se_build(struct lg1314_adsp *adsp, struct adsp_se **sep, u32 *reader)
{
	static const u32 allfmt[8] = { 48000, 2, 32, 0, 0, 48000, 2, 32 };
	static const u32 outctrl[7] = { 0, 48000, OUTCTRL_GAIN_0DB, 1, 0, 0, 0 };
	u32 inoutfs[2] = { 48000, 48000 }, inp[2] = { 2, SE_ACMOD_STEREO }, ch = 2, fs = 48000;
	struct adsp_se *se;
	unsigned int i;
	int ret;

	se = devm_kzalloc(adsp->dev, sizeof(*se), GFP_KERNEL);
	if (!se)
		return -ENOMEM;
	se->adsp = adsp;
	mutex_init(&se->lock);
	se->enabled = true;
	ret = firmware_request_nowarn(&se->fw, SE_FW, adsp->dev);
	if (ret)
		return -ENOENT;
	ret = se_parse(se);
	if (ret) {
		dev_err(adsp->dev, "%s: bad file\n", SE_FW);
		goto err;
	}
	for (i = 1; i < se->nsets; i++)
		if (!strcmp(se->sets[i].name, SE_DEFAULT_MODE))
			se->mode = i - 1;

	ret = se_create(se, MOD_PCMCVT_P13, UID_PCMCVT_P13);
	if (!ret)
		ret = se_create(se, MOD_SE, UID_SE);
	if (!ret)
		ret = se_create(se, MOD_OUT_SPK, UID_OUT_SPK);
	if (!ret)
		ret = se_create(se, MOD_PCM, UID_PCM);
	if (!ret)
		ret = se_connect(se, MOD_PCMCVT_P13, MOD_SE);
	if (!ret)
		ret = se_connect(se, MOD_SE, MOD_OUT_SPK);
	if (!ret)
		ret = se_connect(se, MOD_OUT_SPK, MOD_PCM);
	/* formats as stock: 48 kHz stereo s32 throughout, one I2S line (2 channels) */
	if (!ret)
		ret = se_cmd(se, MOD_PCMCVT_P13, PCMCVT_CMD_SET_ALLFMT, allfmt, sizeof(allfmt));
	if (!ret)
		ret = se_cmd(se, MOD_SE, SE_CMD_SET_INOUTFS, inoutfs, sizeof(inoutfs));
	if (!ret)
		ret = se_cmd(se, MOD_SE, SE_CMD_SET_OUTCH, &ch, sizeof(ch));
	if (!ret)
		ret = se_cmd(se, MOD_SE, SE_CMD_SET_INP_CH_CONFIG, inp, sizeof(inp));
	if (!ret)
		ret = se_cmd(se, MOD_OUT_SPK, OUTCTRL_CMD_SET_CHANNEL, &ch, sizeof(ch));
	if (!ret)
		ret = se_cmd(se, MOD_OUT_SPK, OUTCTRL_CMD_SET_ALL, outctrl, sizeof(outctrl));
	if (!ret)
		ret = se_cmd(se, MOD_PCM, PCMOUT_CMD_SET_SPKCH, &ch, sizeof(ch));
	if (!ret)
		ret = se_cmd(se, MOD_PCM, PCMOUT_CMD_SET_SPKFS, &fs, sizeof(fs));
	/* tuning: initialisation, then the sound mode */
	if (!ret)
		ret = se_apply(se, &se->sets[0]);
	if (!ret)
		ret = se_apply(se, &se->sets[1 + se->mode]);
	if (ret)
		goto err;

	*sep = se;
	*reader = MOD_PCMCVT_P13;
	dev_info(adsp->dev, "speaker chain through the sound engine, mode %s\n",
		 se->sets[1 + se->mode].name);
	return 0;

err:
	release_firmware(se->fw);
	se->fw = NULL;
	return ret;
}

/* Hand the shared buffer to the chain's reader. */
int adsp_se_attach(struct adsp_se *se, struct adsp_shbuf *sb)
{
	u32 alloc[5] = { MOD_PCMCVT_P13, 0, sb->offset, sb->size, sb->uid };

	return se_cmd(se, ADSP_MOD_MAN_DSP(SE_DSP), CM_CMD_ALLOC_BUF, alloc, sizeof(alloc));
}

/* Start the chain, sink first. */
int adsp_se_start(struct adsp_se *se)
{
	static const u32 order[] = { MOD_PCM, MOD_OUT_SPK, MOD_SE, MOD_PCMCVT_P13 };
	unsigned int i;
	int ret;

	ret = se_cmd(se, MOD_PCMCVT_P13, ADSP_MOD_INIT, NULL, 0);
	for (i = 0; !ret && i < ARRAY_SIZE(order); i++)
		ret = se_cmd(se, order[i], ADSP_MOD_START, NULL, 0);
	return ret;
}

void adsp_se_free(struct adsp_se *se)
{
	if (se)
		release_firmware(se->fw);
}

/* ALSA controls on the speaker card: "Sound Mode", "Sound Engine Switch". */
static int se_mode_info(struct snd_kcontrol *kc, struct snd_ctl_elem_info *ui)
{
	struct adsp_se *se = snd_kcontrol_chip(kc);

	return snd_ctl_enum_info(ui, 1, se->nsets - 1, se->names);
}

static int se_mode_get(struct snd_kcontrol *kc, struct snd_ctl_elem_value *uv)
{
	struct adsp_se *se = snd_kcontrol_chip(kc);

	uv->value.enumerated.item[0] = READ_ONCE(se->mode);
	return 0;
}

static int se_mode_put(struct snd_kcontrol *kc, struct snd_ctl_elem_value *uv)
{
	struct adsp_se *se = snd_kcontrol_chip(kc);
	unsigned int mode = uv->value.enumerated.item[0];
	int ret;

	if (mode >= se->nsets - 1)
		return -EINVAL;
	mutex_lock(&se->lock);
	if (mode == se->mode) {
		mutex_unlock(&se->lock);
		return 0;
	}
	ret = se_apply(se, &se->sets[1 + mode]);
	if (!ret)
		se->mode = mode;
	mutex_unlock(&se->lock);
	return ret ? ret : 1;
}

static int se_switch_get(struct snd_kcontrol *kc, struct snd_ctl_elem_value *uv)
{
	struct adsp_se *se = snd_kcontrol_chip(kc);

	uv->value.integer.value[0] = READ_ONCE(se->enabled);
	return 0;
}

static int se_switch_put(struct snd_kcontrol *kc, struct snd_ctl_elem_value *uv)
{
	struct adsp_se *se = snd_kcontrol_chip(kc);
	bool on = uv->value.integer.value[0];
	u32 bypass = !on;
	int ret;

	mutex_lock(&se->lock);
	if (on == se->enabled) {
		mutex_unlock(&se->lock);
		return 0;
	}
	ret = se_cmd(se, MOD_SE, SE_CMD_SET_BYPASS, &bypass, sizeof(bypass));
	if (!ret)
		se->enabled = on;
	mutex_unlock(&se->lock);
	return ret ? ret : 1;
}

static const struct snd_kcontrol_new se_controls[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Sound Mode",
		.info = se_mode_info,
		.get = se_mode_get,
		.put = se_mode_put,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Sound Engine Switch",
		.info = snd_ctl_boolean_mono_info,
		.get = se_switch_get,
		.put = se_switch_put,
	},
};

int adsp_se_add_controls(struct snd_soc_component *c, struct adsp_se *se)
{
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(se_controls); i++) {
		ret = snd_ctl_add(c->card->snd_card, snd_ctl_new1(&se_controls[i], se));
		if (ret)
			return ret;
	}
	return 0;
}
