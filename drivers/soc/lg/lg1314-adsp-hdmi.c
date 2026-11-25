// SPDX-License-Identifier: GPL-2.0
/*
 * LG1314 audio DSPs: HDMI input audio follows the picture. The HDMI receiver
 * (lge,hdmirx) knows the connector being shown (the display selects it) and
 * whether its audio is live; while it is, its port plays through the speakers'
 * mixer (lg1314-adsp-se.c).
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/workqueue.h>

#include <media/lg1314-hdmirx.h>

#include "lg1314-adsp.h"

#define HDMI_LOOKUP_MS		1000
#define HDMI_LOOKUP_TRIES	120

struct adsp_hdmi {
	struct lg1314_adsp *adsp;
	struct device_node *np;
	struct lg1314_hdmirx *rx;
	struct notifier_block nb;
	struct delayed_work lookup;
	struct work_struct update;
	unsigned int tries;
	int port;			/* playing, -1: none */
	u32 rate;			/* its sample rate */
};

static void adsp_hdmi_update(struct work_struct *work)
{
	struct adsp_hdmi *h = container_of(work, struct adsp_hdmi, update);
	struct lg1314_hdmirx_audio aud;
	int port = lg1314_hdmirx_selected_audio(h->rx, &aud);
	u32 rate;

	/* PCM for now; compressed / HBR streams need DSP0's decoders */
	if (port >= 0 && aud.type != LG1314_HDMIRX_AUDIO_PCM)
		port = -1;
	if (port < 0)
		port = -1;
	rate = port >= 0 && aud.rate ? aud.rate : 48000;
	if (port == h->port && (port < 0 || rate == h->rate))
		return;
	if (adsp_pcm_hdmi(h->adsp, port, rate) && port >= 0)
		port = -1;
	h->port = port;
	h->rate = rate;
}

static int adsp_hdmi_notify(struct notifier_block *nb, unsigned long connector, void *data)
{
	struct adsp_hdmi *h = container_of(nb, struct adsp_hdmi, nb);

	schedule_work(&h->update);
	return NOTIFY_OK;
}

static void adsp_hdmi_lookup(struct work_struct *work)
{
	struct adsp_hdmi *h = container_of(work, struct adsp_hdmi, lookup.work);
	struct lg1314_hdmirx *rx = lg1314_hdmirx_get(h->np);

	if (rx == ERR_PTR(-EPROBE_DEFER) && ++h->tries < HDMI_LOOKUP_TRIES) {
		schedule_delayed_work(&h->lookup, msecs_to_jiffies(HDMI_LOOKUP_MS));
		return;
	}
	if (IS_ERR(rx)) {
		dev_warn(h->adsp->dev, "no HDMI receiver: %ld\n", PTR_ERR(rx));
		return;
	}
	h->rx = rx;
	h->nb.notifier_call = adsp_hdmi_notify;
	lg1314_hdmirx_register_notifier(rx, &h->nb);
	schedule_work(&h->update);
}

void adsp_hdmi_init(struct lg1314_adsp *adsp)
{
	struct device_node *np = of_parse_phandle(adsp->dev->of_node, "lge,hdmirx", 0);
	struct adsp_hdmi *h;

	if (!np)
		return;
	h = devm_kzalloc(adsp->dev, sizeof(*h), GFP_KERNEL);
	if (!h) {
		of_node_put(np);
		return;
	}
	h->adsp = adsp;
	h->np = np;
	h->port = -1;
	INIT_DELAYED_WORK(&h->lookup, adsp_hdmi_lookup);
	INIT_WORK(&h->update, adsp_hdmi_update);
	adsp->hdmi = h;
	schedule_delayed_work(&h->lookup, 0);
}

void adsp_hdmi_exit(struct lg1314_adsp *adsp)
{
	struct adsp_hdmi *h = adsp->hdmi;

	if (!h)
		return;
	cancel_delayed_work_sync(&h->lookup);
	if (h->rx) {
		lg1314_hdmirx_unregister_notifier(h->rx, &h->nb);
		cancel_work_sync(&h->update);
		if (h->port >= 0)
			adsp_pcm_hdmi(adsp, -1, 0);
		lg1314_hdmirx_put(h->rx);
	}
	of_node_put(h->np);
	adsp->hdmi = NULL;
}
