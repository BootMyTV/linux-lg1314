// SPDX-License-Identifier: GPL-2.0
/*
 * LG1314 audio DSPs: the speaker chain through LG's sound engine (SE), as stock
 * builds it on DSP1 with the "legacy" modules (by fixed module id, through the
 * DSP's connection manager):
 *
 *   shared buffer -> PCMCVT_P13 -> MIX_0 in 0 -> SE -> OUT_SPK -> PCM (I2S to the amps)
 *   HDMI_IN_0 (DSP0) -> buffer -> PCMCVT_P12 -> MIX_0 in 1
 *   HDMI_IN_1 (DSP0) -> buffer -> PCMCVT_P11 -> buffer -> glue -> gain (new-style,
 *                                  DSP1) -> the mixer of the output playing it
 *
 * Both capture the same receiver port when started (SAI0 / SAI1); one runs at a time,
 * the first for the speakers, the second for optical / headphones.
 *
 * The source's audio clock (recovered by the receiver) and ours differ slightly
 * (e.g. 13 ppm). A rate loop (se_hdmi_rate_work) measures the source against the
 * 90 kHz system clock, from the HDMI input's interrupt counter, and sets the branch
 * converter's input rate (it resamples to 48 kHz) so that what it produces matches
 * what the output consumes; the buffers in between keep their level. (Stock's
 * CTRLSRC module, set up as stock does, has no audible effect on our converters.)
 *
 * The second mixer input is an HDMI receiver port: the receiver's I2S reaches the
 * audio block, DSP0's HDMI_IN_0 captures the port it is told (HAL_SetHdmiInputPort
 * at its start) and hands 2 x s32 frames to DSP1: 24-bit samples in bits 31:8,
 * full-scale s32 (a 0 dBFS sine peaks at 0x7fffff00), so the converters' gain is 0 dB.
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
#include <linux/workqueue.h>

#include <sound/control.h>
#include <sound/soc.h>
#include <sound/tlv.h>

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
#define MOD_HDMI_IN_0		0x10	/* on DSP0 */
#define MOD_HDMI_IN_1		0x11
#define MOD_PCMCVT_P11		0x3f
#define MOD_PCMCVT_P12		0x40
#define MOD_MIX_0		0x5f
#define UID_HDMI_IN_0		0x02100001
#define UID_HDMI_IN_1		0x02110001
#define UID_PCMCVT_P11		0x033f0001
#define UID_PCMCVT_P12		0x03400001
/*
 * The firmware's per-interrupt bookkeeping in the shared area: SAI input interrupt
 * count and the 90 kHz system time (GSTC) of the last one, for HDMI_IN_0 / _1.
 * One interrupt per SAI buffer unit (BC_SIZE_HDMI, 1 KiB = 32 frames of 8 x s32).
 */
#define AUD_HDMIIN_CNT(b)	((b) ? 0x129c : 0x1294)
#define AUD_HDMIIN_GSTC(b)	(AUD_HDMIIN_CNT(b) + 4)
#define HDMIIN_FRAMES_PER_INT	32
#define GSTC_HZ			90000
#define HDMI_RATE		48000
#define HDMI_RATE_PERIOD_MS	1000
#define HDMI_RATE_MAX_PPM	1000	/* sane source clocks are within ~100 ppm */
#define HDMI_RATE_LOG_S		600
#define UID_MIX_0		0x035f0001
#define HDMI_DSP		0
#define MIX_PORT_HOST		0
#define MIX_PORT_HDMI		1
#define MIX_GAIN_0DB		1024
#define HDMI_BUF_SIZE		0x40000
#define HDMI_BUF_AU		0x1000
#define HDMI_BUF_MAX_AU		64

/* connection manager (module_cmd_connection.h), addressed to the DSP's manager */
#define CM_CMD_CONNECT		ADSP_CMD(0xf000, 16)	/* {in mod, in port, out mod, out port} */
#define CM_CMD_ALLOC_BUF	ADSP_CMD(0xf010, 20)	/* {mod, port, offset, size, cowork} */
#define CM_CMD_CREATE_MODULE	ADSP_CMD(0xf020, 8)	/* {id, uid} */
#define CM_PORT_OUT(n)		(0x100 | ((n) << 3))

#define HDMI_CMD_SET_FMT	ADSP_CMD(0x4300, 16)	/* {I2S/LJ/RJ, resolution, sck, lrck pol} */
#define HDMI_CMD_SET_MODE	ADSP_CMD(0x4301, 12)	/* {0 PCM / codec, port, 0 I2S} */
#define PCMCVT_CMD_SET_GAIN	ADSP_CMD(0x5004, 8)	/* {Q23 gain, enable} */
#define PCMCVT_GAIN_HDMI	0x00800000		/* 0 dB: the input is full-scale s32 */
#define GAIN_CMD_SET_GAIN	ADSP_CMD(0x6600, 4)	/* new-style gain: Q23 */
#define GAIN_CMD_SET_CHANNEL	ADSP_CMD(0x6601, 4)
#define PCMCVT_CMD_CHANGE_FS	ADSP_CMD(0x5003, 8)	/* {in Hz, out Hz}: resamples */
#define MIX_CMD_ENABLE_PORT	ADSP_CMD(0x5400, 8)	/* {port, enable} */
#define MIX_CMD_SET_EXT		ADSP_CMD(0x5402, 28)
#define MIX_CMD_SET_FORMAT	ADSP_CMD(0x5403, 12)	/* {port, channels, bits} */
#define MIX_CMD_SET_GAIN	ADSP_CMD(0x5404, 8)	/* {port, gain (1024 = 0 dB)} */

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
	bool hdmi;			/* HDMI branch built */
	int hdmi_port;			/* receiver port playing, -1: none */
	u32 hdmi_gain;			/* HDMI input gain, 1024 = 0 dB */
	int hdmi_branch;		/* 0: speakers, 1: the glue */
	struct delayed_work rate_work;	/* the rate loop, while a port plays */
	bool rate_based;		/* rate_cnt/rate_gstc are from this run */
	u32 rate_cnt, rate_gstc;	/* last readings */
	u64 rate_frames, rate_ticks;	/* input frames, GSTC ticks since the start */
	s64 rate_err;			/* frames produced - consumed, Q16 */
	u32 rate_fs;			/* converter input rate now */
	u64 rate_log_ticks;		/* when to log the loop's state next */
	u32 hdmi_glue, hdmi_gain_uid;	/* branch 1's end: glue -> gain (DSP1) */
	bool hdmi_glue_used;		/* an output takes branch 1 */
};

static int se_cmd_on(struct adsp_se *se, int dsp, u32 target, u32 cmd, const void *param,
		     u32 len)
{
	int ret = adsp_imc_command(se->adsp, dsp, target, cmd, param, len, NULL, 0, 1000);

	if (ret)
		dev_err(se->adsp->dev, "SE chain: dsp%d %02x cmd %08x: %d\n", dsp, target, cmd,
			ret);
	return ret;
}

static int se_cmd(struct adsp_se *se, u32 target, u32 cmd, const void *param, u32 len)
{
	return se_cmd_on(se, SE_DSP, target, cmd, param, len);
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

/* output ports go as the connection manager encodes them (stock: 0x100 | n << 3) */
static int se_connect_port(struct adsp_se *se, u32 out, u32 in, u32 in_port)
{
	u32 param[4] = { in, in_port, out, CM_PORT_OUT(0) };

	return se_cmd(se, ADSP_MOD_MAN_DSP(SE_DSP), CM_CMD_CONNECT, param, sizeof(param));
}

static int se_connect(struct adsp_se *se, u32 out, u32 in)
{
	return se_connect_port(se, out, in, 0);
}

/* A mixer input: stereo s32, as stock's _AUDIO_SetMixerConfig (no fades). */
static int se_mix_port(struct adsp_se *se, u32 port, u32 gain, bool enable)
{
	u32 fmt[3] = { port, 2, 32 }, gain_p[2] = { port, gain }, en[2] = { port, enable };
	/* {port, wait 768, initial mute 0, no auto fade in/out, fade 256, no delay} */
	u32 ext[7] = { port, 256 * 3, 0, 0, 0, 8, 0 };
	int ret;

	ret = se_cmd(se, MOD_MIX_0, MIX_CMD_SET_FORMAT, fmt, sizeof(fmt));
	if (!ret)
		ret = se_cmd(se, MOD_MIX_0, MIX_CMD_SET_EXT, ext, sizeof(ext));
	if (!ret)
		ret = se_cmd(se, MOD_MIX_0, MIX_CMD_SET_GAIN, gain_p, sizeof(gain_p));
	if (!ret)
		ret = se_cmd(se, MOD_MIX_0, MIX_CMD_ENABLE_PORT, en, sizeof(en));
	return ret;
}

/* The two HDMI capture branches: input module (DSP0), converter (DSP1). */
static const struct {
	u32 in, in_uid, cvt, cvt_uid;
} se_hdmi_branch[2] = {
	{ MOD_HDMI_IN_0, UID_HDMI_IN_0, MOD_PCMCVT_P12, UID_PCMCVT_P12 },
	{ MOD_HDMI_IN_1, UID_HDMI_IN_1, MOD_PCMCVT_P11, UID_PCMCVT_P11 },
};

/* A legacy shared buffer: reader @rd on @rd_dsp, writer @wr (output port 0) on @wr_dsp. */
static int se_legacy_buffer(struct adsp_se *se, int rd_dsp, u32 rd, int wr_dsp, u32 wr)
{
	u32 alloc[5];
	struct adsp_shbuf *sb;
	int ret;

	sb = adsp_shbuf_alloc(se->adsp, rd_dsp, rd, HDMI_BUF_SIZE, HDMI_BUF_AU, HDMI_BUF_MAX_AU);
	if (IS_ERR(sb))
		return PTR_ERR(sb);
	alloc[0] = rd;
	alloc[1] = 0;
	alloc[2] = sb->offset;
	alloc[3] = sb->size;
	alloc[4] = wr;
	ret = se_cmd_on(se, rd_dsp, ADSP_MOD_MAN_DSP(rd_dsp), CM_CMD_ALLOC_BUF, alloc,
			sizeof(alloc));
	if (ret)
		return ret;
	alloc[0] = wr;
	alloc[1] = CM_PORT_OUT(0);
	alloc[4] = rd;
	return se_cmd_on(se, wr_dsp, ADSP_MOD_MAN_DSP(wr_dsp), CM_CMD_ALLOC_BUF, alloc,
			 sizeof(alloc));
}

/*
 * The HDMI branches, built once (the inputs stay stopped until a port plays):
 * DSP0's HDMI inputs, a shared buffer each to DSP1 (both ends attached by their
 * managers, as stock's connection manager does between legacy modules), the
 * converters (0 dB), and branch 1's glue. Outputs are attached later.
 */
static int se_build_hdmi(struct adsp_se *se)
{
	static const u32 allfmt[8] = { 48000, 2, 32, 0, 0, 48000, 2, 32 };
	static const u32 gain[2] = { PCMCVT_GAIN_HDMI, 1 };
	int i, ret = 0;

	for (i = 0; !ret && i < ARRAY_SIZE(se_hdmi_branch); i++) {
		u32 create[2] = { se_hdmi_branch[i].in, se_hdmi_branch[i].in_uid };
		u32 cvt = se_hdmi_branch[i].cvt;

		ret = se_cmd_on(se, HDMI_DSP, ADSP_MOD_MAN_DSP(HDMI_DSP), CM_CMD_CREATE_MODULE,
				create, sizeof(create));
		if (!ret)
			ret = se_create(se, cvt, se_hdmi_branch[i].cvt_uid);
		if (!ret)
			ret = se_cmd(se, cvt, PCMCVT_CMD_SET_ALLFMT, allfmt, sizeof(allfmt));
		if (!ret)
			ret = se_cmd(se, cvt, PCMCVT_CMD_SET_GAIN, gain, sizeof(gain));
		if (!ret)
			ret = se_legacy_buffer(se, SE_DSP, cvt, HDMI_DSP, se_hdmi_branch[i].in);
	}
	if (!ret)
		ret = se_mix_port(se, MIX_PORT_HDMI, se->hdmi_gain, false);
	if (!ret)
		ret = adsp_module_create(se->adsp, SE_DSP, "glue", &se->hdmi_glue);
	if (!ret)
		ret = adsp_module_create(se->adsp, SE_DSP, "gain", &se->hdmi_gain_uid);
	/* the glue's output is connected before it gets its buffer: then it reads it */
	if (!ret)
		ret = adsp_module_connect(se->adsp, se->hdmi_glue, 0, se->hdmi_gain_uid, 0);
	return ret;
}

/*
 * Branch 1's end (the gain): the outputs that can play it connect its output 0 to
 * their mixer while they do (see lg1314-adsp-pcm.c).
 */
u32 adsp_se_hdmi_out(struct adsp_se *se)
{
	if (!se || !se->hdmi)
		return 0;
	se->hdmi_glue_used = true;
	return se->hdmi_gain_uid;
}

static int se_hdmi_set_gain(struct adsp_se *se, u32 gain)
{
	u32 mix[2] = { MIX_PORT_HDMI, gain }, q23 = gain << 13;

	if (se->hdmi_branch == 0)
		return se_cmd(se, MOD_MIX_0, MIX_CMD_SET_GAIN, mix, sizeof(mix));
	return adsp_module_cmd(se->adsp, se->hdmi_gain_uid, GAIN_CMD_SET_GAIN, &q23,
			       sizeof(q23), NULL, 0);
}

/*
 * Connect the converters' outputs, once the outputs are built (and have connected
 * branch 1's glue): PCMCVT_P12 -> MIX_0 in 1; PCMCVT_P11 writes a shared buffer
 * that the glue reads. Each end gets that buffer from its own side
 * (CM_CMD_ALLOC_BUF / GLUE_CMD_SET_SHARED_BUFFER) with the other as cowork, the
 * buffer protocol of legacy modules across DSPs and of host -> glue. (LG's own
 * legacy -> new bridge, GLUE_Px + GLUE_CMD_SET_ENABLE, faults the 05.50 firmware.)
 */
int adsp_se_hdmi_attach(struct adsp_se *se)
{
	u32 shared[3], alloc[5], ch = 2, zero = 0;
	struct adsp_shbuf *sb;
	int i, ret;

	if (!se || !se->hdmi)
		return -ENODEV;
	mutex_lock(&se->lock);
	ret = se_connect_port(se, MOD_PCMCVT_P12, MOD_MIX_0, MIX_PORT_HDMI);
	if (!ret && se->hdmi_glue_used) {
		sb = adsp_shbuf_alloc(se->adsp, SE_DSP, se->hdmi_glue, HDMI_BUF_SIZE,
				      HDMI_BUF_AU, HDMI_BUF_MAX_AU);
		ret = PTR_ERR_OR_ZERO(sb);
		if (!ret) {
			shared[0] = sb->offset;
			shared[1] = sb->size;
			shared[2] = MOD_PCMCVT_P11;
			ret = adsp_imc_command(se->adsp, SE_DSP, se->hdmi_glue,
					       ADSP_GLUE_SET_SHARED_BUFFER, shared, sizeof(shared),
					       NULL, 0, 1000);
		}
		if (!ret) {
			alloc[0] = MOD_PCMCVT_P11;
			alloc[1] = CM_PORT_OUT(0);
			alloc[2] = sb->offset;
			alloc[3] = sb->size;
			alloc[4] = se->hdmi_glue;
			ret = se_cmd(se, ADSP_MOD_MAN_DSP(SE_DSP), CM_CMD_ALLOC_BUF, alloc,
				     sizeof(alloc));
		}
		if (!ret)
			ret = adsp_module_cmd(se->adsp, se->hdmi_glue, ADSP_MOD_START, NULL, 0,
					      NULL, 0);
		if (!ret)
			ret = adsp_module_cmd(se->adsp, se->hdmi_gain_uid, GAIN_CMD_SET_CHANNEL,
					      &ch, sizeof(ch), NULL, 0);
		if (!ret)
			ret = adsp_module_cmd(se->adsp, se->hdmi_gain_uid, GAIN_CMD_SET_GAIN,
					      &zero, sizeof(zero), NULL, 0);
		if (!ret)
			ret = adsp_module_cmd(se->adsp, se->hdmi_gain_uid, ADSP_MOD_START, NULL,
					      0, NULL, 0);
	}
	/* legacy modules start once their output is connected */
	for (i = 0; !ret && i < ARRAY_SIZE(se_hdmi_branch); i++) {
		u32 cvt = se_hdmi_branch[i].cvt;

		if (i == 1 && !se->hdmi_glue_used)
			break;
		ret = se_cmd(se, cvt, ADSP_MOD_INIT, NULL, 0);
		if (!ret)
			ret = se_cmd(se, cvt, ADSP_MOD_START, NULL, 0);
	}
	if (ret)
		se->hdmi = false;
	mutex_unlock(&se->lock);
	return ret;
}

static int se_hdmi_set_fs(struct adsp_se *se, u32 fs)
{
	u32 p[2] = { fs, HDMI_RATE };
	int ret = 0;

	if (fs != se->rate_fs) {
		ret = se_cmd(se, se_hdmi_branch[se->hdmi_branch].cvt, PCMCVT_CMD_CHANGE_FS, p,
			     sizeof(p));
		if (!ret)
			se->rate_fs = fs;
	}
	return ret;
}

/*
 * The rate loop. The input delivers HDMIIN_FRAMES_PER_INT frames per interrupt at
 * the source's rate; the outputs consume HDMI_RATE on the system clock (GSTC; the
 * speaker I2S counts exactly 3000 interrupts per 90000 ticks). The converter turns
 * in frames into in x HDMI_RATE / rate_fs out frames. rate_err tracks produced -
 * consumed since the start; the input rate is set to the source's measured rate
 * (cumulative, so it converges), rounded up while a backlog builds and down while
 * it drains: the level holds within about a frame. Steps of 1 Hz (21 ppm) are
 * inaudible, and changing the rate doesn't click.
 */
static void se_hdmi_rate_work(struct work_struct *work)
{
	struct adsp_se *se = container_of(to_delayed_work(work), struct adsp_se, rate_work);
	struct lg1314_adsp *adsp = se->adsp;
	u32 cnt, gstc, dcnt, dticks, fs;
	u64 rate_q16, in;

	mutex_lock(&se->lock);
	if (se->hdmi_port < 0)
		goto out;
	cnt = adsp_rd(adsp, AUD_HDMIIN_CNT(se->hdmi_branch));
	gstc = adsp_rd(adsp, AUD_HDMIIN_GSTC(se->hdmi_branch));
	dcnt = cnt - se->rate_cnt;
	dticks = gstc - se->rate_gstc;
	if (!dcnt || !dticks)
		goto again;			/* no input */
	se->rate_cnt = cnt;
	se->rate_gstc = gstc;
	in = (u64)dcnt * HDMIIN_FRAMES_PER_INT;
	/*
	 * The first interval starts at whatever the counters held before this run (the
	 * time of an interrupt from an earlier one): only a baseline. Intervals far
	 * off the nominal rate (a stall, a source change) re-baseline as well.
	 */
	if (!se->rate_based ||
	    abs((s64)in * GSTC_HZ - (s64)dticks * HDMI_RATE) >
	    (s64)dticks * HDMI_RATE / 1000000 * HDMI_RATE_MAX_PPM) {
		se->rate_based = true;
		goto again;
	}
	se->rate_frames += in;
	se->rate_ticks += dticks;
	se->rate_err += div_u64(in * HDMI_RATE << 16, se->rate_fs);
	se->rate_err -= div_u64((u64)dticks * HDMI_RATE << 16, GSTC_HZ);

	rate_q16 = div64_u64(se->rate_frames * GSTC_HZ << 16, se->rate_ticks);
	fs = rate_q16 >> 16;
	if (se->rate_err > 0)
		fs++;				/* backlog: take more input per output */
	fs = clamp_t(u32, fs, HDMI_RATE - HDMI_RATE / 1000, HDMI_RATE + HDMI_RATE / 1000);
	se_hdmi_set_fs(se, fs);
	if (se->rate_ticks >= se->rate_log_ticks) {
		s64 ppm = div64_s64(((s64)rate_q16 - ((s64)HDMI_RATE << 16)) * 1000000,
				    (s64)HDMI_RATE << 16);

		dev_info(adsp->dev, "HDMI audio: source clock %+lld ppm (converter %u Hz, level %+lld frames)\n",
			 ppm, se->rate_fs, se->rate_err >> 16);
		se->rate_log_ticks += HDMI_RATE_LOG_S * GSTC_HZ;
	}
again:
	schedule_delayed_work(&se->rate_work, msecs_to_jiffies(HDMI_RATE_PERIOD_MS));
out:
	mutex_unlock(&se->lock);
}

/* Start the loop for the branch just started. se->lock held. */
static void se_hdmi_rate_start(struct adsp_se *se)
{
	se->rate_cnt = adsp_rd(se->adsp, AUD_HDMIIN_CNT(se->hdmi_branch));
	se->rate_gstc = adsp_rd(se->adsp, AUD_HDMIIN_GSTC(se->hdmi_branch));
	se->rate_frames = 0;
	se->rate_ticks = 0;
	se->rate_err = 0;
	se->rate_log_ticks = 30 * GSTC_HZ;	/* then every HDMI_RATE_LOG_S */
	se->rate_based = false;
	se->rate_fs = HDMI_RATE;
	schedule_delayed_work(&se->rate_work, msecs_to_jiffies(HDMI_RATE_PERIOD_MS));
}

static void se_hdmi_stop(struct adsp_se *se)
{
	u32 en[2] = { MIX_PORT_HDMI, 0 };

	if (se->hdmi_port < 0)
		return;
	if (se->hdmi_branch == 0)
		se_cmd(se, MOD_MIX_0, MIX_CMD_ENABLE_PORT, en, sizeof(en));
	else
		se_hdmi_set_gain(se, 0);
	se_cmd_on(se, HDMI_DSP, se_hdmi_branch[se->hdmi_branch].in, ADSP_MOD_STOP, NULL, 0);
	se_hdmi_set_fs(se, HDMI_RATE);
	se->hdmi_port = -1;
}

/*
 * Play receiver port @port (0..3), or stop (-1): through branch 0 into the
 * speakers' mixer, or through branch 1 (@branch 1) to whichever output has
 * connected its end. The receiver must have its audio output unmuted for the port.
 */
int adsp_se_hdmi(struct adsp_se *se, int port, int branch)
{
	static const u32 fmt[4] = { 0, 3, 0, 0 };	/* I2S, 24 bit */
	u32 mode[3] = { 0, port, 0 }, en[2] = { MIX_PORT_HDMI, 1 };
	int ret = 0;
	u32 in;

	if (!se || !se->hdmi || (branch && !se->hdmi_glue_used))
		return -ENODEV;
	mutex_lock(&se->lock);
	if (port == se->hdmi_port && branch == se->hdmi_branch)
		goto out;
	se_hdmi_stop(se);
	se->hdmi_branch = branch;
	se->rate_fs = HDMI_RATE;
	if (port < 0)
		goto out;

	in = se_hdmi_branch[branch].in;
	ret = se_hdmi_set_gain(se, se->hdmi_gain);
	if (!ret)
		ret = se_cmd_on(se, HDMI_DSP, in, HDMI_CMD_SET_FMT, fmt, sizeof(fmt));
	if (!ret)
		ret = se_cmd_on(se, HDMI_DSP, in, HDMI_CMD_SET_MODE, mode, sizeof(mode));
	if (!ret)
		ret = se_cmd_on(se, HDMI_DSP, in, ADSP_MOD_START, NULL, 0);
	if (!ret && !branch)
		ret = se_cmd(se, MOD_MIX_0, MIX_CMD_ENABLE_PORT, en, sizeof(en));
	if (!ret)
		se_hdmi_rate_start(se);
	if (!ret) {
		se->hdmi_port = port;
		dev_info(se->adsp->dev, "HDMI audio: port %d (branch %d)\n", port, branch);
	}
out:
	if (port < 0)
		dev_info(se->adsp->dev, "HDMI audio: off\n");
	mutex_unlock(&se->lock);
	return ret;
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
	se->hdmi_port = -1;
	se->hdmi_gain = MIX_GAIN_0DB;
	INIT_DELAYED_WORK(&se->rate_work, se_hdmi_rate_work);
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
		ret = se_create(se, MOD_MIX_0, UID_MIX_0);
	if (!ret)
		ret = se_connect_port(se, MOD_PCMCVT_P13, MOD_MIX_0, MIX_PORT_HOST);
	if (!ret)
		ret = se_connect(se, MOD_MIX_0, MOD_SE);
	if (!ret)
		ret = se_mix_port(se, MIX_PORT_HOST, MIX_GAIN_0DB, true);
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

	if (se_build_hdmi(se))
		dev_warn(adsp->dev, "no HDMI input on the speaker chain\n");
	else
		se->hdmi = true;

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
	static const u32 order[] = { MOD_PCM, MOD_OUT_SPK, MOD_SE, MOD_MIX_0, MOD_PCMCVT_P13 };
	unsigned int i;
	int ret;

	ret = se_cmd(se, MOD_PCMCVT_P13, ADSP_MOD_INIT, NULL, 0);
	for (i = 0; !ret && i < ARRAY_SIZE(order); i++)
		ret = se_cmd(se, order[i], ADSP_MOD_START, NULL, 0);
	return ret;
}

void adsp_se_free(struct adsp_se *se)
{
	if (!se)
		return;
	cancel_delayed_work_sync(&se->rate_work);
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

/* "HDMI Input Playback Volume": the mixer gain of the HDMI input, linear (1024 = 0 dB) */
static int se_hdmi_vol_info(struct snd_kcontrol *kc, struct snd_ctl_elem_info *ui)
{
	ui->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	ui->count = 1;
	ui->value.integer.min = 0;
	ui->value.integer.max = MIX_GAIN_0DB;
	return 0;
}

static int se_hdmi_vol_get(struct snd_kcontrol *kc, struct snd_ctl_elem_value *uv)
{
	struct adsp_se *se = snd_kcontrol_chip(kc);

	uv->value.integer.value[0] = READ_ONCE(se->hdmi_gain);
	return 0;
}

static int se_hdmi_vol_put(struct snd_kcontrol *kc, struct snd_ctl_elem_value *uv)
{
	struct adsp_se *se = snd_kcontrol_chip(kc);
	long v = uv->value.integer.value[0];
	int ret = 0, changed = 0;

	if (v < 0 || v > MIX_GAIN_0DB)
		return -EINVAL;
	mutex_lock(&se->lock);
	if (v != se->hdmi_gain) {
		u32 old = se->hdmi_gain;

		se->hdmi_gain = v;
		ret = se->hdmi && se->hdmi_port >= 0 ? se_hdmi_set_gain(se, v) : 0;
		if (ret)
			se->hdmi_gain = old;
		else
			changed = 1;
	}
	mutex_unlock(&se->lock);
	return ret ? ret : changed;
}

static const DECLARE_TLV_DB_LINEAR(se_hdmi_vol_tlv, TLV_DB_GAIN_MUTE, 0);

static const struct snd_kcontrol_new se_controls[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "HDMI Input Playback Volume",
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE | SNDRV_CTL_ELEM_ACCESS_TLV_READ,
		.info = se_hdmi_vol_info,
		.get = se_hdmi_vol_get,
		.put = se_hdmi_vol_put,
		.tlv.p = se_hdmi_vol_tlv,
	},
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
