// SPDX-License-Identifier: GPL-2.0
/* LG1314 display-engine PQE firmware download. */

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include "lg1314_drm.h"

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
	memcpy_toio(fwmem, fw->data, fw->size);
	wmb();
	iounmap(fwmem);

	ctrl = readl(ipc + DE_IPC_CTRL);
	writel(ctrl | DE_IPC_CTRL_HOLD, ipc + DE_IPC_CTRL);

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
