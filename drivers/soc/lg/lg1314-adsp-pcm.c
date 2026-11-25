// SPDX-License-Identifier: GPL-2.0
/*
 * LG1314 audio DSPs: PCM outputs (ASoC CPU DAIs + platform).
 *
 * One fixed chain per output on DSP1: glue -> output module -> hardware. The
 * glue reads a shared buffer that we keep full: from the ALSA buffer while a
 * stream runs, with silence otherwise. The outputs never stop on their own:
 * starved, they replay their last few milliseconds, so a buffer must never run
 * dry.
 *
 * Without a format event the output modules take s32 frames: output_speaker 4
 * slots (0/1 reach the I2S line to the amplifiers), output_spdif and output_hp
 * (the internal DAC's "sub" I2S) 2. Stereo is
 * expanded to that here.
 *
 * The speakers go through LG's sound engine instead when its tuning file is
 * there (lg1314-adsp-se.c: stock's legacy chain, 2 x s32 frames); the plain
 * output_speaker chain is the fallback (and the choice with speaker_se=0).
 *
 * Each output is a separate card, so a separate ASoC component (a component
 * belongs to one card, and cards find their PCM component by DT node): a child
 * device per output, on the adsp node's child of the output's name.
 */

#include <linux/bits.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/workqueue.h>

#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "lg1314-adsp.h"

#define OUT_DSP			1
#define OUT_AU_FRAMES		512			/* 10.7 ms per AU */
#define OUT_MAX_AU		8
/* buffer size in AUs: after the header and an AU of slack, 4 AUs (43 ms) queue */
#define OUT_SHBUF_AUS		6

/* output_spdif commands (module_cmd_spdifout.h) */
#define SPDIF_CMD_SET_OUTFORMAT	ADSP_CMD(0x5a00, 8)	/* {0 PCM / 1 ES, fs (ES)} */
#define SPDIF_CMD_SET_LIGHT	ADSP_CMD(0x5a04, 4)	/* {on} */

/* audio block: S/PDIF output enable ("light") */
#define AUD_SPDIFOUT_SEL	0x610
#define  AUD_SPDIFOUT_ENABLE	BIT(5)

static bool speaker_se = true;
module_param(speaker_se, bool, 0444);
MODULE_PARM_DESC(speaker_se, "Speakers through the sound engine (if lg1314/lgse.bin is there)");

#define OUT_FORMATS		(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | \
				 SNDRV_PCM_FMTBIT_S32_LE)

enum { OUT_SPEAKER, OUT_SPDIF, OUT_HP, OUT_NUM };

struct adsp_out;

struct adsp_out_desc {
	const char *node;		/* DT child of the adsp node: the DAI */
	const char *pcm_name;		/* what players show next to the card */
	const char *module;
	unsigned int slots;		/* s32 per frame */
	bool hdmi;			/* can take the HDMI input */
	int (*setup)(struct adsp_out *o);
	void (*teardown)(struct adsp_out *o);
};

struct adsp_out {
	struct lg1314_adsp *adsp;
	const struct adsp_out_desc *desc;
	int index;
	struct platform_device *pdev;	/* DAI + PCM component */
	u32 glue, mod;
	u32 mixer;			/* host glue (in 0), HDMI (in 1) -> output */
	struct adsp_se *se;		/* speakers through the sound engine */
	struct snd_soc_component *component;
	bool hold;			/* codecs unmuted without a stream (HDMI audio) */
	struct work_struct hold_work;
	unsigned int slots;		/* s32 per frame in the shared buffer */
	struct adsp_shbuf *sb;
	struct work_struct fill_work;
	unsigned int au_bytes;

	spinlock_t lock;			/* stream state below */
	struct snd_pcm_substream *substream;	/* running */
	snd_pcm_uframes_t pos;			/* next frame to copy */
	snd_pcm_uframes_t period_pos;

	s32 *au;
};

struct adsp_pcm {
	struct workqueue_struct *wq;
	struct adsp_out out[OUT_NUM];
	struct mutex hdmi_lock;		/* the HDMI routing below */
	int hdmi_target;		/* output for the HDMI input (control) */
	int hdmi_port;			/* receiver port playing, -1: none */
	int hdmi_on;			/* output playing it, -1: none */
	u32 hdmi_out;			/* the HDMI branch's end (a gain) */
};

static int adsp_spdif_setup(struct adsp_out *o)
{
	u32 fmt[2] = { 0, 48000 }, on = 1;
	int ret;

	ret = adsp_module_cmd(o->adsp, o->mod, SPDIF_CMD_SET_OUTFORMAT, fmt, sizeof(fmt),
			      NULL, 0);
	if (!ret)
		ret = adsp_module_cmd(o->adsp, o->mod, SPDIF_CMD_SET_LIGHT, &on, sizeof(on),
				      NULL, 0);
	if (!ret)
		adsp_wr(o->adsp, AUD_SPDIFOUT_SEL,
			adsp_rd(o->adsp, AUD_SPDIFOUT_SEL) | AUD_SPDIFOUT_ENABLE);
	return ret;
}

static void adsp_spdif_teardown(struct adsp_out *o)
{
	adsp_wr(o->adsp, AUD_SPDIFOUT_SEL,
		adsp_rd(o->adsp, AUD_SPDIFOUT_SEL) & ~AUD_SPDIFOUT_ENABLE);
}

static const struct adsp_out_desc adsp_outs[OUT_NUM] = {
	[OUT_SPEAKER] = {
		.node = "speaker", .pcm_name = "Built-in",
		.module = "output_speaker", .slots = 4,
	},
	[OUT_SPDIF] = {
		.node = "spdif", .pcm_name = "S/PDIF",
		.module = "output_spdif", .slots = 2,
		.setup = adsp_spdif_setup, .teardown = adsp_spdif_teardown,
		.hdmi = true,
	},
	[OUT_HP] = {
		.node = "headphones", .pcm_name = "Analog",
		.module = "output_hp", .slots = 2,
		.hdmi = true,
	},
};

static const struct snd_pcm_hardware adsp_out_hw = {
	.info = SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID | SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats = OUT_FORMATS,
	.rates = SNDRV_PCM_RATE_48000,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = 256 * 1024,
	.period_bytes_min = 1024,
	.period_bytes_max = 64 * 1024,
	.periods_min = 2,
	.periods_max = 64,
};

/*
 * One AU from the running stream (or silence). Returns the substream if a
 * period elapsed.
 */
static struct snd_pcm_substream *adsp_out_take(struct adsp_out *o)
{
	unsigned int slots = o->slots;
	struct snd_pcm_substream *elapsed = NULL;
	struct snd_pcm_runtime *rt;
	unsigned long flags;
	int i, c;

	memset(o->au, 0, o->au_bytes);
	spin_lock_irqsave(&o->lock, flags);
	if (!o->substream)
		goto out;
	rt = o->substream->runtime;
	for (i = 0; i < OUT_AU_FRAMES; i++) {
		s32 *dst = &o->au[i * slots];

		for (c = 0; c < 2; c++) {
			switch (rt->format) {
			case SNDRV_PCM_FORMAT_S16_LE:
				dst[c] = ((s16 *)rt->dma_area)[o->pos * 2 + c] << 16;
				break;
			case SNDRV_PCM_FORMAT_S24_LE:
				dst[c] = ((s32 *)rt->dma_area)[o->pos * 2 + c] << 8;
				break;
			default:
				dst[c] = ((s32 *)rt->dma_area)[o->pos * 2 + c];
				break;
			}
		}
		if (++o->pos == rt->buffer_size)
			o->pos = 0;
		if (++o->period_pos == rt->period_size) {
			o->period_pos = 0;
			elapsed = o->substream;
		}
	}
out:
	spin_unlock_irqrestore(&o->lock, flags);
	return elapsed;
}

static void adsp_out_fill(struct work_struct *work)
{
	struct adsp_out *o = container_of(work, struct adsp_out, fill_work);
	struct snd_pcm_substream *elapsed;

	while (adsp_shbuf_free_space(o->sb) >= o->au_bytes) {
		elapsed = adsp_out_take(o);
		if (adsp_shbuf_write_au(o->sb, o->au, o->au_bytes))
			break;
		if (elapsed)
			snd_pcm_period_elapsed(elapsed);
	}
}

/* BM_EVT_READ: the glue consumed data (called from the IPC rx work). */
static void adsp_out_on_read(void *priv)
{
	struct adsp_out *o = priv;

	queue_work(o->adsp->pcm->wq, &o->fill_work);
}

static struct adsp_out *to_out(struct snd_soc_component *c)
{
	return snd_soc_component_get_drvdata(c);
}

static int adsp_out_open(struct snd_soc_component *c, struct snd_pcm_substream *ss)
{
	snd_soc_set_runtime_hwparams(ss, &adsp_out_hw);
	return snd_pcm_hw_constraint_integer(ss->runtime, SNDRV_PCM_HW_PARAM_PERIODS);
}

static int adsp_out_prepare(struct snd_soc_component *c, struct snd_pcm_substream *ss)
{
	struct adsp_out *o = to_out(c);
	unsigned long flags;

	spin_lock_irqsave(&o->lock, flags);
	o->pos = 0;
	o->period_pos = 0;
	spin_unlock_irqrestore(&o->lock, flags);
	return 0;
}

static int adsp_out_trigger(struct snd_soc_component *c, struct snd_pcm_substream *ss,
			    int cmd)
{
	struct adsp_out *o = to_out(c);
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&o->lock, flags);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		o->substream = ss;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		o->substream = NULL;
		break;
	default:
		ret = -EINVAL;
	}
	spin_unlock_irqrestore(&o->lock, flags);
	return ret;
}

static snd_pcm_uframes_t adsp_out_pointer(struct snd_soc_component *c,
					  struct snd_pcm_substream *ss)
{
	return READ_ONCE(to_out(c)->pos);
}

/* After a stop: the fill work no longer references the stream. */
static int adsp_out_sync_stop(struct snd_soc_component *c, struct snd_pcm_substream *ss)
{
	flush_work(&to_out(c)->fill_work);
	return 0;
}

static int adsp_out_construct(struct snd_soc_component *c, struct snd_soc_pcm_runtime *rtd)
{
	/* ASoC leaves the name empty; players list the device by it */
	strscpy(rtd->pcm->name, to_out(c)->desc->pcm_name);
	snd_pcm_set_managed_buffer_all(rtd->pcm, SNDRV_DMA_TYPE_VMALLOC, NULL,
				       0, adsp_out_hw.buffer_bytes_max);
	return 0;
}

/*
 * Keep the output's codecs (the speaker amps) unmuted while something plays
 * through the DSP without an ALSA stream (HDMI audio), and muted again after,
 * unless a stream is running then. ASoC mutes them when a stream ends, before
 * the component's hw_free, which queues this again.
 */
static void adsp_out_hold_work(struct work_struct *work)
{
	struct adsp_out *o = container_of(work, struct adsp_out, hold_work);
	struct snd_soc_component *c = o->component;
	struct snd_soc_pcm_runtime *rtd;
	struct snd_soc_dai *dai;
	bool hold = READ_ONCE(o->hold);
	int i;

	if (!c || !c->card)
		return;
	for_each_card_rtds(c->card, rtd) {
		if (snd_soc_rtd_to_cpu(rtd, 0)->component != c)
			continue;
		snd_soc_dpcm_mutex_lock(rtd);
		for_each_rtd_codec_dais(rtd, i, dai)
			if (!snd_soc_dai_stream_active(dai, SNDRV_PCM_STREAM_PLAYBACK))
				snd_soc_dai_digital_mute(dai, !hold, SNDRV_PCM_STREAM_PLAYBACK);
		snd_soc_dpcm_mutex_unlock(rtd);
	}
}

static int adsp_out_hw_free(struct snd_soc_component *c, struct snd_pcm_substream *ss)
{
	struct adsp_out *o = to_out(c);

	if (READ_ONCE(o->hold))
		schedule_work(&o->hold_work);
	return 0;
}

static const struct snd_kcontrol_new adsp_hdmi_target_control;

static int adsp_out_component_probe(struct snd_soc_component *c)
{
	struct adsp_out *o = to_out(c);

	o->component = c;
	int ret = o->se ? adsp_se_add_controls(c, o->se) : 0;

	if (!ret && o->se)
		ret = snd_ctl_add(c->card->snd_card,
				  snd_ctl_new1(&adsp_hdmi_target_control, o->adsp));
	return ret;
}

static const struct snd_soc_component_driver adsp_out_component = {
	.name = "lg1314-adsp-pcm",
	.probe = adsp_out_component_probe,
	.open = adsp_out_open,
	.prepare = adsp_out_prepare,
	.trigger = adsp_out_trigger,
	.pointer = adsp_out_pointer,
	.sync_stop = adsp_out_sync_stop,
	.hw_free = adsp_out_hw_free,
	.pcm_construct = adsp_out_construct,
};

static int adsp_i2s_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	/* The DSP's I2S outputs: I2S, clocks from the SoC. */
	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_I2S)
		return -EINVAL;
	return 0;
}

static const struct snd_soc_dai_ops adsp_i2s_dai_ops = {
	.set_fmt = adsp_i2s_set_fmt,
};

static struct snd_soc_dai_driver adsp_out_dais[OUT_NUM] = {
	[OUT_SPEAKER] = {
		.name = "lg1314-adsp-speaker",
		.playback = {
			.stream_name = "Speaker Playback",
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_48000,
			.formats = OUT_FORMATS,
		},
		.ops = &adsp_i2s_dai_ops,
	},
	[OUT_SPDIF] = {
		.name = "lg1314-adsp-spdif",
		.playback = {
			.stream_name = "S/PDIF Playback",
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_48000,
			.formats = OUT_FORMATS,
		},
	},
	[OUT_HP] = {
		.name = "lg1314-adsp-headphones",
		.playback = {
			.stream_name = "Headphone Playback",
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_48000,
			.formats = OUT_FORMATS,
		},
		.ops = &adsp_i2s_dai_ops,
	},
};

/* The per-output child devices ("lg1314-adsp-out"): register the component. */
static int adsp_out_probe(struct platform_device *pdev)
{
	struct adsp_out *o = *(struct adsp_out **)dev_get_platdata(&pdev->dev);

	platform_set_drvdata(pdev, o);
	return devm_snd_soc_register_component(&pdev->dev, &adsp_out_component,
					       &adsp_out_dais[o->index], 1);
}

struct platform_driver adsp_out_driver = {
	.probe = adsp_out_probe,
	.driver = {
		.name = "lg1314-adsp-out",
	},
};

/* The speakers through the sound engine: stereo s32 into its legacy chain. */
static int adsp_out_init_se(struct adsp_out *o, u32 reader)
{
	struct lg1314_adsp *adsp = o->adsp;
	int ret;

	o->slots = 2;
	o->au_bytes = OUT_AU_FRAMES * o->slots * sizeof(s32);
	o->au = devm_kzalloc(adsp->dev, o->au_bytes, GFP_KERNEL);
	if (!o->au)
		return -ENOMEM;
	o->sb = adsp_shbuf_alloc(adsp, OUT_DSP, reader, OUT_SHBUF_AUS * o->au_bytes,
				 o->au_bytes, OUT_MAX_AU);
	if (IS_ERR(o->sb))
		return PTR_ERR(o->sb);
	o->sb->priv = o;
	ret = adsp_se_attach(o->se, o->sb);
	if (ret)
		return ret;
	adsp_out_fill(&o->fill_work);
	o->sb->on_read = adsp_out_on_read;
	return adsp_se_start(o->se);
}

/*
 * An output that can take the HDMI input: its host glue into a new-style mixer,
 * whose input 1 takes the HDMI branch's end while the input plays on this output.
 *   glue (host) --------------------> mixer in 0 -> output
 *   (HDMI glue -> gain) -- when playing --> mixer in 1
 * The mixer waits for every input that has delivered data: once the HDMI branch
 * stops, input 1 must be disconnected and the mixer restarted, or the output
 * underruns (and buzzes). Connected before the host glue gets its buffer: a glue
 * is the reader only if its output is connected by then.
 */
static int adsp_out_build_hdmi(struct adsp_out *o)
{
	struct lg1314_adsp *adsp = o->adsp;
	int ret;

	ret = adsp_module_create(adsp, OUT_DSP, "mixer", &o->mixer);
	if (!ret)
		ret = adsp_module_connect(adsp, o->glue, 0, o->mixer, 0);
	if (!ret)
		ret = adsp_module_connect(adsp, o->mixer, 0, o->mod, 0);
	return ret;
}

/* The HDMI branch's end into / out of the output's mixer. */
static int adsp_out_hdmi_attach(struct adsp_out *o, u32 hdmi_out)
{
	return adsp_module_connect(o->adsp, hdmi_out, 0, o->mixer, 1);
}

static void adsp_out_hdmi_detach(struct adsp_out *o, u32 hdmi_out)
{
	adsp_module_disconnect(o->adsp, hdmi_out, 0, o->mixer, 1);
	adsp_module_cmd(o->adsp, o->mixer, ADSP_MOD_STOP, NULL, 0, NULL, 0);
	adsp_module_cmd(o->adsp, o->mixer, ADSP_MOD_START, NULL, 0, NULL, 0);
}

/* Build an output's chain, prime it with silence and start it. */
static int adsp_out_init(struct lg1314_adsp *adsp, struct adsp_out *o, int index)
{
	const struct adsp_out_desc *desc = &adsp_outs[index];
	struct device *dev = adsp->dev;
	u32 reader, hdmi_out;
	int ret;

	o->adsp = adsp;
	o->desc = desc;
	o->index = index;
	o->slots = desc->slots;
	spin_lock_init(&o->lock);
	INIT_WORK(&o->fill_work, adsp_out_fill);
	INIT_WORK(&o->hold_work, adsp_out_hold_work);

	if (index == OUT_SPEAKER && speaker_se) {
		ret = adsp_se_build(adsp, &o->se, &reader);
		if (!ret)
			return adsp_out_init_se(o, reader);
		if (ret != -ENOENT)
			dev_warn(dev, "sound engine chain: %d, using the plain one\n", ret);
		o->se = NULL;
	}

	o->au_bytes = OUT_AU_FRAMES * o->slots * sizeof(s32);
	o->au = devm_kzalloc(dev, o->au_bytes, GFP_KERNEL);
	if (!o->au)
		return -ENOMEM;
	ret = adsp_module_create(adsp, OUT_DSP, "glue", &o->glue);
	if (!ret)
		ret = adsp_module_create(adsp, OUT_DSP, desc->module, &o->mod);
	hdmi_out = desc->hdmi ? adsp_se_hdmi_out(adsp->pcm->out[OUT_SPEAKER].se) : 0;
	if (!ret && hdmi_out) {
		adsp->pcm->hdmi_out = hdmi_out;
		ret = adsp_out_build_hdmi(o);
	} else if (!ret)
		ret = adsp_module_connect(adsp, o->glue, 0, o->mod, 0);
	if (ret)
		return ret;
	o->sb = adsp_shbuf_create(adsp, o->glue, OUT_SHBUF_AUS * o->au_bytes, o->au_bytes,
				  OUT_MAX_AU);
	if (IS_ERR(o->sb))
		return PTR_ERR(o->sb);
	o->sb->priv = o;
	adsp_out_fill(&o->fill_work);
	o->sb->on_read = adsp_out_on_read;
	if (desc->setup) {
		ret = desc->setup(o);
		if (ret)
			return ret;
	}
	if (o->mixer) {
		ret = adsp_module_cmd(adsp, o->mixer, ADSP_MOD_START, NULL, 0, NULL, 0);
		if (ret)
			return ret;
	}
	return adsp_module_cmd(adsp, o->mod, ADSP_MOD_START, NULL, 0, NULL, 0);
}

/* The output's DAI device, on its DT node (none: the output has no DAI). */
static int adsp_out_add_dai(struct adsp_out *o)
{
	struct device_node *np;
	struct platform_device *pdev;
	int ret;

	np = of_get_child_by_name(o->adsp->dev->of_node, o->desc->node);
	if (!np)
		return 0;
	pdev = platform_device_alloc("lg1314-adsp-out", o->index);
	if (!pdev) {
		of_node_put(np);
		return -ENOMEM;
	}
	pdev->dev.parent = o->adsp->dev;
	device_set_node(&pdev->dev, of_fwnode_handle(np));	/* put on release */
	ret = platform_device_add_data(pdev, &o, sizeof(o));
	if (!ret)
		ret = platform_device_add(pdev);
	if (ret) {
		platform_device_put(pdev);
		return ret;
	}
	o->pdev = pdev;
	return 0;
}

static void adsp_out_exit(struct adsp_out *o)
{
	struct lg1314_adsp *adsp = o->adsp;

	if (o->pdev) {
		platform_device_unregister(o->pdev);
		o->pdev = NULL;
	}
	if (!o->sb || IS_ERR(o->sb))
		return;
	mutex_lock(&adsp->shbuf_lock);
	o->sb->on_read = NULL;
	mutex_unlock(&adsp->shbuf_lock);
	cancel_work_sync(&o->fill_work);
	cancel_work_sync(&o->hold_work);
	o->component = NULL;
	if (o->desc->teardown)
		o->desc->teardown(o);
	adsp_se_free(o->se);
	o->se = NULL;
}

/* Play the HDMI input (port, -1: none) on the selected output. hdmi_lock held. */
static int adsp_pcm_hdmi_route(struct lg1314_adsp *adsp)
{
	struct adsp_pcm *pcm = adsp->pcm;
	int i, port = pcm->hdmi_port, on = port >= 0 ? pcm->hdmi_target : -1, ret;
	struct adsp_out *t = on >= 0 ? &pcm->out[on] : NULL;

	/* the output it leaves: its mixer must not wait for the branch's end */
	if (pcm->hdmi_on > OUT_SPEAKER && pcm->hdmi_on != on)
		adsp_out_hdmi_detach(&pcm->out[pcm->hdmi_on], pcm->hdmi_out);
	ret = on > OUT_SPEAKER && pcm->hdmi_on != on ?
	      adsp_out_hdmi_attach(t, pcm->hdmi_out) : 0;
	if (!ret)
		ret = adsp_se_hdmi(pcm->out[OUT_SPEAKER].se, port, on > OUT_SPEAKER);
	if (ret && on > OUT_SPEAKER)
		adsp_out_hdmi_detach(t, pcm->hdmi_out);
	if (ret) {
		adsp_se_hdmi(pcm->out[OUT_SPEAKER].se, -1, 0);
		port = -1;
		on = -1;
		t = NULL;
	}
	pcm->hdmi_on = on;
	/* the playing output's codecs stay unmuted without a stream */
	for (i = 0; i < OUT_NUM; i++) {
		struct adsp_out *o = &pcm->out[i];
		bool hold = o == t;

		if (READ_ONCE(o->hold) != hold) {
			WRITE_ONCE(o->hold, hold);
			schedule_work(&o->hold_work);
		}
	}
	return ret;
}

/* HDMI receiver port @port (-1: none) plays, on the output the control selects. */
int adsp_pcm_hdmi(struct lg1314_adsp *adsp, int port)
{
	struct adsp_pcm *pcm = adsp->pcm;
	int ret;

	if (!pcm)
		return -ENODEV;
	mutex_lock(&pcm->hdmi_lock);
	pcm->hdmi_port = port;
	ret = adsp_pcm_hdmi_route(adsp);
	mutex_unlock(&pcm->hdmi_lock);
	return ret;
}

/* "HDMI Input Output": where the HDMI input's audio plays (players set it). */
static const char * const adsp_hdmi_targets[OUT_NUM] = {
	[OUT_SPEAKER] = "Speakers", [OUT_SPDIF] = "Optical", [OUT_HP] = "Headphones",
};

static int adsp_hdmi_target_info(struct snd_kcontrol *kc, struct snd_ctl_elem_info *ui)
{
	return snd_ctl_enum_info(ui, 1, OUT_NUM, adsp_hdmi_targets);
}

static int adsp_hdmi_target_get(struct snd_kcontrol *kc, struct snd_ctl_elem_value *uv)
{
	struct lg1314_adsp *adsp = snd_kcontrol_chip(kc);

	uv->value.enumerated.item[0] = READ_ONCE(adsp->pcm->hdmi_target);
	return 0;
}

static int adsp_hdmi_target_put(struct snd_kcontrol *kc, struct snd_ctl_elem_value *uv)
{
	struct lg1314_adsp *adsp = snd_kcontrol_chip(kc);
	struct adsp_pcm *pcm = adsp->pcm;
	unsigned int t = uv->value.enumerated.item[0];
	int ret = 0;

	if (t >= OUT_NUM)
		return -EINVAL;
	if (t != OUT_SPEAKER && !pcm->out[t].mixer)
		return -ENODEV;
	mutex_lock(&pcm->hdmi_lock);
	if (t != pcm->hdmi_target) {
		pcm->hdmi_target = t;
		ret = adsp_pcm_hdmi_route(adsp);
		ret = ret ? ret : 1;
	}
	mutex_unlock(&pcm->hdmi_lock);
	return ret;
}

static const struct snd_kcontrol_new adsp_hdmi_target_control = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "HDMI Input Output",
	.info = adsp_hdmi_target_info,
	.get = adsp_hdmi_target_get,
	.put = adsp_hdmi_target_put,
};

/* Connect the HDMI branches (built with the speaker chain) once the outputs are. */
static void adsp_pcm_attach_hdmi(struct lg1314_adsp *adsp)
{
	int ret = adsp_se_hdmi_attach(adsp->pcm->out[OUT_SPEAKER].se);

	if (ret)
		dev_warn(adsp->dev, "no HDMI input audio: %d\n", ret);
}

/*
 * Build the output chains and add their DAIs. The DSP modules stay when this
 * fails or the driver goes: they cannot be torn down reliably yet.
 */
int adsp_pcm_init(struct lg1314_adsp *adsp)
{
	struct device *dev = adsp->dev;
	struct adsp_pcm *pcm;
	int i, ret;

	pcm = devm_kzalloc(dev, sizeof(*pcm), GFP_KERNEL);
	if (!pcm)
		return -ENOMEM;
	mutex_init(&pcm->hdmi_lock);
	pcm->hdmi_port = -1;
	pcm->hdmi_on = -1;
	pcm->hdmi_target = OUT_SPEAKER;
	pcm->wq = alloc_ordered_workqueue("adsp-pcm", WQ_HIGHPRI);
	if (!pcm->wq)
		return -ENOMEM;
	adsp->pcm = pcm;

	for (i = 0; i < OUT_NUM; i++) {
		ret = adsp_out_init(adsp, &pcm->out[i], i);
		if (!ret)
			ret = adsp_out_add_dai(&pcm->out[i]);
		if (ret) {
			dev_err(dev, "%s: %d\n", adsp_outs[i].module, ret);
			goto err;
		}
	}
	adsp_pcm_attach_hdmi(adsp);
	return 0;

err:
	for (i = 0; i < OUT_NUM; i++)
		adsp_out_exit(&pcm->out[i]);
	destroy_workqueue(pcm->wq);
	adsp->pcm = NULL;
	return ret;
}

void adsp_pcm_exit(struct lg1314_adsp *adsp)
{
	struct adsp_pcm *pcm = adsp->pcm;
	int i;

	if (!pcm)
		return;
	for (i = 0; i < OUT_NUM; i++)
		adsp_out_exit(&pcm->out[i]);
	destroy_workqueue(pcm->wq);
	adsp->pcm = NULL;
}
