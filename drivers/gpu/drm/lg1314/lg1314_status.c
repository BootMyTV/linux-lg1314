// SPDX-License-Identifier: GPL-2.0
/*
 * What the panel is showing, for userspace: the device's video_mode
 * attribute, "key=value" lines. mode= (sdr, hdr10, dolby) comes first and
 * seq= (counts changes) last; userspace polls for POLLPRI, the attribute is
 * notified whenever the text changes.
 *
 *   mode=dolby			mode=hdr10
 *   profile=8			mastering=0.0050-1000 cd/m2
 *   dm=DM3			maxcll=1000 cd/m2
 *   metadata=CM v2.9, 1 L2 trim	maxfall=400 cd/m2
 *   mastering=0.0050-4000 cd/m2	tonemap=1000 cd/m2
 *   target=0.0010-600 cd/m2	refresh=47.952 Hz
 *   refresh=47.952 Hz		seq=3
 *   seq=7
 *
 * refresh= is the mode's refresh rate, for every mode.
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>

#include "lg1314_drm.h"

/* A luminance in 1/10000 cd/m²: four decimals below 10 cd/m², whole above. */
static int status_lum(char *buf, size_t n, u32 l)
{
	if (l >= 100000)
		return scnprintf(buf, n, "%u", (l + 5000) / 10000);
	return scnprintf(buf, n, "%u.%04u", l / 10000, l % 10000);
}

static int status_range(char *buf, size_t n, const char *key, u32 lo, u32 hi)
{
	int len = scnprintf(buf, n, "%s=", key);

	len += status_lum(buf + len, n - len, lo);
	len += scnprintf(buf + len, n - len, "-");
	len += status_lum(buf + len, n - len, hi);
	return len + scnprintf(buf + len, n - len, " cd/m2\n");
}

/* The text for the current state (without seq=). Under status.lock. */
static int status_text(struct lg1314_drm *lg, char *buf, size_t n)
{
	const struct lg1314_dovi_status *dv = &lg->dovi.status;
	int len = 0;

	if (READ_ONCE(lg->dovi.active)) {
		len += scnprintf(buf + len, n - len, "mode=dolby\nprofile=%u\n", dv->profile);
		if (dv->dm) {
			len += scnprintf(buf + len, n - len, "dm=DM%u\n", dv->dm);
			if (dv->dm == 3)
				len += scnprintf(buf + len, n - len,
						 "metadata=CM v2.9, %u L2 trim%s\n",
						 dv->trims, dv->trims == 1 ? "" : "s");
			else
				len += scnprintf(buf + len, n - len,
						 "metadata=CM v4.0 (version %u), %u L8 trim%s\n",
						 dv->cm_version, dv->trims,
						 dv->trims == 1 ? "" : "s");
			len += status_range(buf + len, n - len, "mastering",
					    dv->src_min, dv->src_max);
			len += status_range(buf + len, n - len, "target",
					    dv->tgt_min, dv->tgt_max);
		}
	} else if (lg->hdr.hdr10) {
		len += scnprintf(buf + len, n - len, "mode=hdr10\n");
		if (lg->hdr.mdl_max)
			len += status_range(buf + len, n - len, "mastering",
					    lg->hdr.mdl_min, lg->hdr.mdl_max * 10000);
		if (lg->hdr.max_cll)
			len += scnprintf(buf + len, n - len, "maxcll=%u cd/m2\n",
					 lg->hdr.max_cll);
		if (lg->hdr.max_fall)
			len += scnprintf(buf + len, n - len, "maxfall=%u cd/m2\n",
					 lg->hdr.max_fall);
		len += scnprintf(buf + len, n - len, "tonemap=%u cd/m2\n", lg->hdr.nits);
	} else {
		len += scnprintf(buf + len, n - len, "mode=sdr\n");
	}
	if (lg->refresh_mhz)
		len += scnprintf(buf + len, n - len, "refresh=%u.%03u Hz\n",
				 lg->refresh_mhz / 1000, lg->refresh_mhz % 1000);
	return len;
}

/* The state changed (maybe): new text, and a notification if it differs. */
void lg1314_status_update(struct lg1314_drm *lg)
{
	char buf[LG1314_STATUS_LEN];
	bool changed;
	int len;

	mutex_lock(&lg->status.lock);
	len = status_text(lg, buf, sizeof(buf));
	changed = len != lg->status.len || memcmp(buf, lg->status.text, len);
	if (changed) {
		memcpy(lg->status.text, buf, len);
		lg->status.len = len;
		lg->status.seq++;
	}
	mutex_unlock(&lg->status.lock);
	if (changed)
		sysfs_notify(&lg->dev->kobj, NULL, "video_mode");
}

static ssize_t video_mode_show(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
	struct lg1314_drm *lg = dev_get_drvdata(dev);
	ssize_t len;

	mutex_lock(&lg->status.lock);
	len = sysfs_emit(buf, "%.*sseq=%u\n", lg->status.len, lg->status.text,
			 lg->status.seq);
	mutex_unlock(&lg->status.lock);
	return len;
}
static DEVICE_ATTR_RO(video_mode);

static struct attribute *lg1314_status_attrs[] = {
	&dev_attr_video_mode.attr,
	NULL,
};

static const struct attribute_group lg1314_status_group = {
	.attrs = lg1314_status_attrs,
};

int lg1314_status_init(struct lg1314_drm *lg)
{
	mutex_init(&lg->status.lock);
	lg1314_status_update(lg);
	return devm_device_add_group(lg->dev, &lg1314_status_group);
}
