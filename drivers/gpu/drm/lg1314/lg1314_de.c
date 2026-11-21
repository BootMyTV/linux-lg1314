// SPDX-License-Identifier: GPL-2.0
/* LG1314 display-engine PQE firmware download. */

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/sizes.h>
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
