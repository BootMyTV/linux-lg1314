// SPDX-License-Identifier: GPL-2.0
/* LG1314 display-engine PQE firmware download. */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/sizes.h>
#include <linux/uaccess.h>
#include "lg1314_drm.h"

/*
 * Memory layout table the DE firmware reads at start-up (in the DE IPC block):
 * a magic/model header followed by {base, size} pairs. Region sizes that are
 * not whole reserved regions are the stock platform's.
 */
#define DE_TAG_MAGIC		0xdecafe01	/* magic << 8 | 1 (ASIC) */
#define DE_PREW_SIZE		(70 * SZ_1M)	/* pre-write, then FRC0 */
#define DE_FRC0_SIZE		(68 * SZ_1M)
#define DE_BE_FPP_SIZE		(19 * SZ_1M)
#define DE_BE_FRC1_SIZE		(9 * SZ_1M)
#define DE_BE_FRC2_SIZE		(48 * SZ_1M)
#define DE_BE_DB_OFF		0x40000		/* into the back-end region */
#define DE_FW_M0_SLOT		SZ_32K		/* shared-memory slot size */

struct de_tag_mem {
	u32 base;
	u32 size;
};

struct de_tag {
	u32 system_info;
	u32 model;
	struct de_tag_mem firmware, prew, frm_m1, frm_m2, bt_m1, bt_m2,
			  vt_m1, vt_m2, hdr, be_fpp, be_frc0, be_frc1, be_frc2,
			  be_db, fpp_info, vdec_pool_0, vdec_pool_1,
			  sys_time, sys_time_diff, video_delay;
};
static_assert(sizeof(struct de_tag) == 42 * sizeof(u32));

static struct de_tag_mem de_mem(const struct resource *r)
{
	return (struct de_tag_mem){ (u32)r->start, (u32)resource_size(r) };
}

static void lg1314_de_write_tag(struct lg1314_drm *lg)
{
	const struct lg1314_de_mem *m = &lg->de_mem;
	u32 m0 = (u32)m->fw_m0.start;
	u32 dpb = (u32)m->vdec_dpb.start, dpb_size = resource_size(&m->vdec_dpb);
	u32 surf = (u32)m->scanout.start, surf_size = resource_size(&m->scanout);
	struct de_tag t = {
		.system_info	= DE_TAG_MAGIC,
		.model		= m->model,
		.firmware	= { (u32)lg->de_fw_phys, (u32)lg->de_fw_size },
		.prew		= { (u32)m->prew.start, DE_PREW_SIZE },
		.frm_m2		= de_mem(&m->frm),
		.bt_m2		= { (u32)m->vt.start, 0 },
		.vt_m2		= de_mem(&m->vt),
		.hdr		= de_mem(&m->hdr),
		.be_fpp		= { 0, DE_BE_FPP_SIZE },
		.be_frc0	= { (u32)m->prew.start + DE_PREW_SIZE, DE_FRC0_SIZE },
		.be_frc1	= { 0, DE_BE_FRC1_SIZE },
		.be_frc2	= { 0, DE_BE_FRC2_SIZE },
		.be_db		= { (u32)m->be.start + DE_BE_DB_OFF, 0 },
		.fpp_info	= { m0 + 2 * DE_FW_M0_SLOT, SZ_32K },
		.sys_time	= { m0 + 3 * DE_FW_M0_SLOT, 0 },
		.video_delay	= { m0 + 4 * DE_FW_M0_SLOT, 16 },
	};

	/* Video pool: the decoded-picture pool together with the graphics
	 * surfaces, as one span from whichever comes first. */
	if (dpb < surf)
		t.vdec_pool_0 = (struct de_tag_mem){ dpb, dpb_size + surf_size };
	else
		t.vdec_pool_0 = (struct de_tag_mem){ surf, dpb - surf + dpb_size };
	/* The firmware takes video (and Dolby Vision enhancement layer)
	 * pictures only from these spans: the second is the decoder's
	 * overflow pool, as stock's for its high-frame-rate pool. */
	if (m->vdec_shared.end)
		t.vdec_pool_1 = de_mem(&m->vdec_shared);

	memcpy_toio(lg->mmio[LG1314_REG_DE_IPC].virt + DE_IPC_TAG, &t, sizeof(t));
	wmb();
}

int lg1314_de_load_fw(struct lg1314_drm *lg)
{
	const struct firmware *fw;
	void __iomem *ipc, *fwmem;
	u32 phys, ctrl;
	int ret;

	if (lg->de_fw_phys < DE_FW_DDR_MIN || !lg->de_fw_size)
		return -EINVAL;

	/*
	 * A firmware already running (this driver loaded before, in this
	 * boot) is kept: the MCU has no reset we can use (the vendor's is
	 * compiled out; the CTOP candidates hang the bus), and holding it,
	 * copying and releasing it lets the old code run on over the new
	 * image, never taking commands. The vendor's test for a running
	 * firmware (DE_REG_M19_IPCisAlive): it has enabled its IPC interrupt.
	 */
	if (readl(lg->mmio[LG1314_REG_DE_IPC].virt + DE_IPC_INT_ENABLE) &
	    DE_IPC_MBOX_IRQ) {
		dev_info(lg->dev, "DE firmware already running: kept\n");
		return 0;
	}

	ret = request_firmware(&fw, DE_FW_NAME, lg->dev);
	if (ret) {
		dev_err(lg->dev, "DE firmware %s not found: %d\n", DE_FW_NAME, ret);
		return ret;
	}
	if (fw->size > lg->de_fw_size) {
		ret = -ENOSPC;
		goto out;
	}

	ipc = lg->mmio[LG1314_REG_DE_IPC].virt;
	phys = (u32)lg->de_fw_phys;

	fwmem = ioremap_wc(lg->de_fw_phys, fw->size);
	if (!fwmem) {
		ret = -ENOMEM;
		goto out;
	}

	/* Hold the MCU, publish the memory layout, copy the image in. */
	ctrl = readl(ipc + DE_IPC_CTRL);
	writel(ctrl | DE_IPC_CTRL_HOLD, ipc + DE_IPC_CTRL);

	lg1314_de_write_tag(lg);

	memcpy_toio(fwmem, fw->data, fw->size);
	wmb();
	iounmap(fwmem);

	writel(phys, ipc + DE_IPC_ADR5_LO);
	writel(phys, ipc + DE_IPC_ADR5_HI);

	ctrl = readl(ipc + DE_IPC_CTRL) & ~DE_IPC_CTRL_B1;
	writel(ctrl, ipc + DE_IPC_CTRL);
	usleep_range(1000, 2000);
	ctrl = readl(ipc + DE_IPC_CTRL) & ~DE_IPC_CTRL_HOLD;
	writel(ctrl, ipc + DE_IPC_CTRL);

	dev_info(lg->dev, "DE firmware downloaded (%zu KiB @ %#x)\n",
		 fw->size >> 10, phys);
	ret = 0;
out:
	release_firmware(fw);
	return ret;
}

/*
 * Firmware command mailbox. A message is a u32 command followed by its
 * parameter struct, sent in chunks of up to 124 bytes. Each chunk has a
 * header word {size, chunks remaining, from host, value}; the firmware
 * clears the size byte once it has taken the chunk. Replies arrive the same
 * way in the receive mailbox, which the host frees by clearing its size byte.
 */
#define DE_MBOX_FROM_HOST	2
#define DE_MBOX_TYPE_VAL	2
#define DE_MBOX_HDR(len, cont)	((len) | (cont) << 8 | DE_MBOX_FROM_HOST << 16 | \
				 DE_MBOX_TYPE_VAL << 24)
#define DE_MBOX_PAYLOAD		(DE_IPC_MBOX_SIZE - 4)
#define DE_MBOX_MSG_MAX		256
#define DE_MBOX_TIMEOUT_US	200000

#define DE_MBOX_KICK_US		20000

static int de_mbox_wait(struct lg1314_drm *lg, u32 off, bool full)
{
	void __iomem *mbox = lg->mmio[LG1314_REG_DE_IPC].virt + off;
	u32 v;

	return readl_poll_timeout(mbox, v, !!(v & 0xff) == full, 100,
				  DE_MBOX_TIMEOUT_US);
}

/*
 * Wait for the firmware to take the chunk in the transmit mailbox. A
 * doorbell rung before the firmware listens is lost, so ring it again while
 * waiting.
 */
static int de_mbox_wait_taken(struct lg1314_drm *lg)
{
	void __iomem *ipc = lg->mmio[LG1314_REG_DE_IPC].virt;
	int waited;
	u32 v;

	for (waited = 0; waited < DE_MBOX_TIMEOUT_US; waited += DE_MBOX_KICK_US) {
		if (!readl_poll_timeout(ipc + DE_IPC_MBOX_TX, v, !(v & 0xff), 100,
					DE_MBOX_KICK_US))
			return 0;
		writel(DE_IPC_MBOX_IRQ, ipc + DE_IPC_INT_EVENT);
	}
	return -ETIMEDOUT;
}

static void de_ack_irqs(struct lg1314_drm *lg)
{
	void __iomem *ipc = lg->mmio[LG1314_REG_DE_IPC].virt;
	u32 st = readl(ipc + DE_IPC_EXT_STATUS);

	if (st)
		writel(st, ipc + DE_IPC_EXT_CLEAR);
}

static int de_send(struct lg1314_drm *lg, u32 cmd, const void *data, size_t len)
{
	void __iomem *tx = lg->mmio[LG1314_REG_DE_IPC].virt + DE_IPC_MBOX_TX;
	u8 msg[DE_MBOX_MSG_MAX] __aligned(4);
	size_t total = sizeof(cmd) + len, off, n, i;
	unsigned int cont;
	int ret = 0;

	lockdep_assert_held(&lg->de_lock);

	if (total > sizeof(msg))
		return -EINVAL;
	memcpy(msg, &cmd, sizeof(cmd));
	if (len)
		memcpy(msg + sizeof(cmd), data, len);

	cont = total / DE_MBOX_PAYLOAD;
	for (off = 0; off < total; off += n, cont--) {
		n = min_t(size_t, total - off, DE_MBOX_PAYLOAD);

		ret = de_mbox_wait_taken(lg);
		if (ret)
			break;
		for (i = 0; i < n; i += 4) {
			u32 w = 0;

			memcpy(&w, msg + off + i, min_t(size_t, 4, n - i));
			writel(w, tx + 4 + i);
		}
		writel(DE_MBOX_HDR(n, cont), tx);
		writel(DE_IPC_MBOX_IRQ,
		       lg->mmio[LG1314_REG_DE_IPC].virt + DE_IPC_INT_EVENT);
	}
	if (!ret)
		ret = de_mbox_wait_taken(lg);
	de_ack_irqs(lg);

	if (ret)
		dev_err(lg->dev, "DE command %#x not taken by firmware\n", cmd);
	return ret;
}

int lg1314_de_cmd(struct lg1314_drm *lg, u32 cmd, const void *data, size_t len)
{
	int ret;

	mutex_lock(&lg->de_lock);
	ret = de_send(lg, cmd, data, len);
	mutex_unlock(&lg->de_lock);
	return ret;
}

/* Send a parameterless request and read back its reply (single chunk). */
int lg1314_de_query(struct lg1314_drm *lg, u32 cmd, void *out, size_t len)
{
	void __iomem *rx = lg->mmio[LG1314_REG_DE_IPC].virt + DE_IPC_MBOX_RX;
	size_t got;
	int ret;

	mutex_lock(&lg->de_lock);
	writel(0, rx);
	ret = de_send(lg, cmd, NULL, 0);
	if (!ret)
		ret = de_mbox_wait(lg, DE_IPC_MBOX_RX, true);
	if (!ret) {
		size_t i;

		/* The mailbox only takes 32-bit accesses. */
		got = min_t(size_t, readl(rx) & 0xff, len);
		for (i = 0; i < got; i += 4) {
			u32 w = readl(rx + 4 + i);

			memcpy(out + i, &w, min_t(size_t, 4, got - i));
		}
		ret = got;
	} else {
		dev_err(lg->dev, "DE request %#x: no reply\n", cmd);
	}
	writel(0, rx);
	de_ack_irqs(lg);
	mutex_unlock(&lg->de_lock);
	return ret;
}

/*
 * A freshly started firmware takes the memory layout table out of the
 * transmit mailbox once it is up; wait for that before the first command.
 */
#define DE_FW_READY_TIMEOUT_US	2000000
#define DE_CMD_GET_FW_INFO	0x02

int lg1314_de_wait_ready(struct lg1314_drm *lg)
{
	void __iomem *tx = lg->mmio[LG1314_REG_DE_IPC].virt + DE_IPC_MBOX_TX;
	u32 v;
	int ret;

	struct {			/* LX_DE_FIRMWARE_INFO_T */
		u32 version;
		u16 year, month, day;
	} __packed info;
	int tries;

	ret = readl_poll_timeout(tx, v, !(v & 0xff), 1000, DE_FW_READY_TIMEOUT_US);
	/* Then until it answers a request. */
	for (tries = 0; !ret && tries < 10; tries++) {
		ret = lg1314_de_query(lg, DE_CMD_GET_FW_INFO, &info, sizeof(info));
		if (ret == sizeof(info)) {
			dev_info(lg->dev, "DE firmware %u (%04u-%02u-%02u) running\n",
				 info.version, info.year, info.month, info.day);
			return 0;
		}
		ret = 0;
	}
	dev_err(lg->dev, "DE firmware did not come up\n");
	return -ETIMEDOUT;
}

#define DE_CMD_SET_FR_RATE	0x1c

/*
 * Lock the panel timing to the video source (the decoder output's tick, at
 * half the panel rate), as the vendor driver does at playback. The panel
 * runs at 96, 100 or 120 Hz (the rates stock's frame lock uses); locked, it
 * follows the source within that class (95.904 Hz for a 47.952 Hz tick).
 */
int lg1314_de_frame_lock(struct lg1314_drm *lg, unsigned int panel_hz)
{
	struct {
		u32 force_free_run;
		u16 rate;
		u16 pad;
	} fr = { .force_free_run = 0, .rate = panel_hz };

	return lg1314_de_cmd(lg, DE_CMD_SET_FR_RATE, &fr, sizeof(fr));
}

#ifdef CONFIG_DEBUG_FS
/*
 * de_cmd: write "<cmd> [u32 ...]" (hex) to send a command with those
 * parameter words; "? <cmd>" sends a request and keeps the reply, which a
 * read returns as hex words.
 */
static u32 de_dbg_reply[DE_MBOX_PAYLOAD / 4];
static int de_dbg_reply_len;

static ssize_t de_cmd_write(struct file *file, const char __user *ubuf,
			    size_t count, loff_t *ppos)
{
	struct lg1314_drm *lg = file->private_data;
	u32 words[DE_MBOX_MSG_MAX / 4];
	char buf[512], *p = buf, *tok;
	bool query = false;
	int n = 0, ret;

	if (count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';

	while ((tok = strsep(&p, " \t\n"))) {
		if (!*tok)
			continue;
		if (!n && !query && !strcmp(tok, "?")) {
			query = true;
			continue;
		}
		if (n == ARRAY_SIZE(words))
			return -E2BIG;
		ret = kstrtou32(tok, 16, &words[n++]);
		if (ret)
			return ret;
	}
	if (!n)
		return -EINVAL;

	if (query) {
		ret = lg1314_de_query(lg, words[0], de_dbg_reply,
				      sizeof(de_dbg_reply));
		de_dbg_reply_len = ret < 0 ? 0 : ret;
	} else {
		ret = lg1314_de_cmd(lg, words[0], &words[1], (n - 1) * 4);
	}
	return ret < 0 ? ret : count;
}

static ssize_t de_cmd_read(struct file *file, char __user *ubuf,
			   size_t count, loff_t *ppos)
{
	char buf[DE_MBOX_PAYLOAD / 4 * 9 + 2];
	int i, len = 0;

	for (i = 0; i < DIV_ROUND_UP(de_dbg_reply_len, 4); i++)
		len += scnprintf(buf + len, sizeof(buf) - len, "%08x ",
				 de_dbg_reply[i]);
	len += scnprintf(buf + len, sizeof(buf) - len, "\n");
	return simple_read_from_buffer(ubuf, count, ppos, buf, len);
}

static const struct file_operations de_cmd_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = de_cmd_write,
	.read = de_cmd_read,
	.llseek = default_llseek,
};

void lg1314_de_debugfs_init(struct lg1314_drm *lg)
{
	debugfs_create_file("de_cmd", 0600, lg->drm.debugfs_root, lg,
			    &de_cmd_fops);
}
#else
void lg1314_de_debugfs_init(struct lg1314_drm *lg) { }
#endif
