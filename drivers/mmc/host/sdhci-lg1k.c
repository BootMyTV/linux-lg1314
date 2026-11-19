// SPDX-License-Identifier: GPL-2.0-only
/*
 * SDHCI driver for the eMMC 5.0 host on LG lg1k-family DTV SoCs (LG1314).
 *
 * The host is an Arasan-derived SDHCI 3.0 core behind an LG "top" wrapper
 * that holds the I/O delay taps, a DLL, a strobe delay and the pad drive
 * strength. The input sampling tap depends on the card, so it is found by
 * CMD21 tuning in HS200 and reused, plus an optional offset, for HS400.
 * Output taps, strobe delay and pad drive strength compensate for SoC/board
 * skew and loading, so they come from the device tree.
 *
 * Based on the LG vendor driver (drivers/mmc/host/sdhci-lg1k.c, 4.4 tree),
 * Copyright (C) 2013 LG Electronics, Chanho Min, Hankyung Yu.
 */

#include <linux/bitfield.h>
#include <linux/bitmap.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "sdhci-pltfm.h"

/* Vendor registers in the SDHCI block */
#define LG1K_CLOCK_GATING	0x78

/* Top wrapper registers (offsets for the 5.0-v3 host) */
#define LG1K_TOP_TAB		0x10
#define LG1K_TOP_DLL_RDY	0x4c
#define LG1K_TOP_DLL		0x60
#define LG1K_TOP_DELAY		0x6c
#define LG1K_TOP_DLL_SELECT	0x7c

#define LG1K_TAB_IN_MASK	GENMASK(7, 0)
#define LG1K_TAB_IN_EN		BIT(8)
#define LG1K_TAB_IN_OFF		GENMASK(8, 0)
#define LG1K_TAB_IN_WINDOW	BIT(9)
#define LG1K_TAB_OUT_MASK	GENMASK(19, 16)
#define LG1K_TAB_OUT_EN		BIT(20)

#define LG1K_DLL_TRM_ICP	GENMASK(3, 0)
#define LG1K_DLL_EN		BIT(7)
#define LG1K_DLL_DS		GENMASK(22, 20)

#define LG1K_DELAY_STRB		GENMASK(3, 0)
#define LG1K_DELAY_CELL		GENMASK(5, 4)

#define LG1K_NUM_IN_TAPS	32

/* DLL charge pump setting; the same on every lg1k board */
#define LG1K_TRM_ICP		4

#define LG1K_MIN_CLOCK		398000

struct sdhci_lg1k {
	void __iomem *top;

	/* Input taps: HS200 found by tuning, HS400 derived from it */
	u32 hs200_in;
	u32 hs400_in;
	s32 hs400_itap_offset;

	/* From the device tree */
	u32 hs_out, hs_in;
	u32 hs200_out;
	u32 hs400_out;
	u32 strb;
	u32 host_ds;
};

static struct sdhci_lg1k *to_lg1k(struct sdhci_host *host)
{
	return sdhci_pltfm_priv(sdhci_priv(host));
}

/*
 * The core only tolerates 32-bit accesses. Sub-word writes are done as
 * read-modify-write, and the transfer mode register is shadowed so that it
 * reaches the hardware in the same write as the command.
 */
static u16 lg1k_readw(struct sdhci_host *host, int reg)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);

	if (reg == SDHCI_TRANSFER_MODE)
		return pltfm_host->xfer_mode_shadow;

	return readl(host->ioaddr + (reg & ~3)) >> ((reg & 3) * 8);
}

static u8 lg1k_readb(struct sdhci_host *host, int reg)
{
	return readl(host->ioaddr + (reg & ~3)) >> ((reg & 3) * 8);
}

static void lg1k_writew(struct sdhci_host *host, u16 val, int reg)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	void __iomem *addr = host->ioaddr + (reg & ~3);
	int shift = (reg & 3) * 8;

	switch (reg) {
	case SDHCI_TRANSFER_MODE:
		pltfm_host->xfer_mode_shadow = val;
		return;
	case SDHCI_COMMAND:
		writel(val << 16 | pltfm_host->xfer_mode_shadow, addr);
		return;
	}

	writel((readl(addr) & ~(0xffff << shift)) | (val << shift), addr);
}

static void lg1k_writeb(struct sdhci_host *host, u8 val, int reg)
{
	void __iomem *addr = host->ioaddr + (reg & ~3);
	int shift = (reg & 3) * 8;

	writel((readl(addr) & ~(0xff << shift)) | (val << shift), addr);
}

static void lg1k_top_rmw(struct sdhci_lg1k *lg, u32 reg, u32 mask, u32 val)
{
	u32 v = readl(lg->top + reg);

	writel((v & ~mask) | val, lg->top + reg);
}

static void lg1k_set_intab(struct sdhci_lg1k *lg, u32 in, bool enable)
{
	u32 tab = readl(lg->top + LG1K_TOP_TAB);

	tab &= ~LG1K_TAB_IN_OFF;
	writel(tab, lg->top + LG1K_TOP_TAB);
	if (!enable)
		return;

	tab |= LG1K_TAB_IN_WINDOW;
	writel(tab, lg->top + LG1K_TOP_TAB);
	tab = (tab & ~LG1K_TAB_IN_MASK) | (in & LG1K_TAB_IN_MASK);
	writel(tab, lg->top + LG1K_TOP_TAB);
	tab &= ~LG1K_TAB_IN_WINDOW;
	writel(tab, lg->top + LG1K_TOP_TAB);
	tab |= LG1K_TAB_IN_EN;
	writel(tab, lg->top + LG1K_TOP_TAB);
}

static void lg1k_set_outtab(struct sdhci_lg1k *lg, u32 out, bool enable)
{
	if (enable)
		lg1k_top_rmw(lg, LG1K_TOP_TAB, LG1K_TAB_OUT_MASK | LG1K_TAB_OUT_EN,
			     LG1K_TAB_OUT_EN | FIELD_PREP(LG1K_TAB_OUT_MASK, out));
	else
		lg1k_top_rmw(lg, LG1K_TOP_TAB, LG1K_TAB_OUT_EN, 0);
}

static void lg1k_set_taps(struct sdhci_lg1k *lg, u32 out, u32 in)
{
	lg1k_set_outtab(lg, out, true);
	lg1k_set_intab(lg, in, true);
}

static void lg1k_disable_dll(struct sdhci_lg1k *lg)
{
	lg1k_top_rmw(lg, LG1K_TOP_DLL, LG1K_DLL_EN, 0);
	mdelay(1);
}

static void lg1k_enable_dll(struct sdhci_host *host)
{
	struct sdhci_lg1k *lg = to_lg1k(host);
	u32 rdy;

	lg1k_disable_dll(lg);
	lg1k_top_rmw(lg, LG1K_TOP_DLL, 0, LG1K_DLL_EN);

	if (readl_poll_timeout_atomic(lg->top + LG1K_TOP_DLL_RDY, rdy,
				      rdy & 1, 100, 10000))
		dev_warn(mmc_dev(host->mmc), "DLL did not lock\n");
}

static void lg1k_set_strobe(struct sdhci_lg1k *lg)
{
	u32 v = readl(lg->top + LG1K_TOP_DELAY);

	if (FIELD_GET(LG1K_DELAY_STRB, v) == lg->strb)
		return;

	writel((v & ~LG1K_DELAY_STRB) | lg->strb, lg->top + LG1K_TOP_DELAY);
	mdelay(1);
}

static void lg1k_set_trm_icp(struct sdhci_lg1k *lg)
{
	u32 v = readl(lg->top + LG1K_TOP_DLL);

	if (FIELD_GET(LG1K_DLL_TRM_ICP, v) == LG1K_TRM_ICP)
		return;

	writel((v & ~LG1K_DLL_TRM_ICP) | LG1K_TRM_ICP, lg->top + LG1K_TOP_DLL);
	mdelay(1);
}

/* Program DLL and taps for the bus mode currently selected in HOST_CONTROL2 */
static void lg1k_apply_timing(struct sdhci_host *host)
{
	struct sdhci_lg1k *lg = to_lg1k(host);
	u16 mode = sdhci_readw(host, SDHCI_HOST_CONTROL2) & SDHCI_CTRL_UHS_MASK;

	switch (mode) {
	case SDHCI_CTRL_HS400:
		lg1k_set_strobe(lg);
		lg1k_enable_dll(host);
		lg1k_set_taps(lg, lg->hs400_out, lg->hs400_in);
		break;
	case SDHCI_CTRL_UHS_SDR104:
		lg1k_enable_dll(host);
		/*
		 * The HS200 -> HS400 switch drops to 52 MHz while SDR104 is
		 * still selected; use the 50 MHz taps for that step.
		 */
		if (host->clock == 52000000)
			lg1k_set_taps(lg, lg->hs_out, lg->hs_in);
		else
			lg1k_set_taps(lg, lg->hs200_out, lg->hs200_in);
		break;
	case SDHCI_CTRL_UHS_DDR50:
	case SDHCI_CTRL_UHS_SDR25:
		lg1k_enable_dll(host);
		lg1k_set_taps(lg, lg->hs_out, lg->hs_in);
		break;
	default:
		lg1k_disable_dll(lg);
		lg1k_set_outtab(lg, 0, false);
		lg1k_set_intab(lg, 0, false);
		break;
	}
}

static void lg1k_set_clock(struct sdhci_host *host, unsigned int clock)
{
	struct sdhci_lg1k *lg = to_lg1k(host);

	lg1k_set_trm_icp(lg);

	sdhci_set_clock(host, clock);

	lg1k_apply_timing(host);

	/* Standard delay cell up to 52 MHz, DLL above */
	lg1k_top_rmw(lg, LG1K_TOP_DELAY, LG1K_DELAY_CELL, 0);
	writel(clock <= 52000000, lg->top + LG1K_TOP_DLL_SELECT);

	lg1k_top_rmw(lg, LG1K_TOP_DLL, LG1K_DLL_DS,
		     FIELD_PREP(LG1K_DLL_DS,
				clock >= 52000000 ? lg->host_ds : 0));
}

/*
 * Legacy high-speed eMMC runs in the SDR25 slot so that it gets the DLL and
 * the 50 MHz taps. The 1.8V signalling bit tracks the mode, since the pads
 * are fixed at 1.8V and the voltage switch itself is a no-op.
 */
static void lg1k_set_uhs_signaling(struct sdhci_host *host, unsigned int timing)
{
	u16 ctrl2 = sdhci_readw(host, SDHCI_HOST_CONTROL2);

	ctrl2 &= ~(SDHCI_CTRL_UHS_MASK | SDHCI_CTRL_VDD_180);

	switch (timing) {
	case MMC_TIMING_MMC_HS400:
		ctrl2 |= SDHCI_CTRL_HS400 | SDHCI_CTRL_VDD_180;
		break;
	case MMC_TIMING_MMC_HS200:
	case MMC_TIMING_UHS_SDR104:
		ctrl2 |= SDHCI_CTRL_UHS_SDR104 | SDHCI_CTRL_VDD_180;
		break;
	case MMC_TIMING_UHS_SDR50:
		ctrl2 |= SDHCI_CTRL_UHS_SDR50 | SDHCI_CTRL_VDD_180;
		break;
	case MMC_TIMING_MMC_DDR52:
	case MMC_TIMING_UHS_DDR50:
		ctrl2 |= SDHCI_CTRL_UHS_DDR50 | SDHCI_CTRL_VDD_180;
		break;
	case MMC_TIMING_MMC_HS:
	case MMC_TIMING_UHS_SDR25:
		ctrl2 |= SDHCI_CTRL_UHS_SDR25;
		break;
	case MMC_TIMING_UHS_SDR12:
		ctrl2 |= SDHCI_CTRL_UHS_SDR12;
		break;
	}

	sdhci_writew(host, ctrl2, SDHCI_HOST_CONTROL2);
}

/*
 * Pick the middle of the largest run of passing taps. The tap range is
 * circular, so a run may wrap from the last tap back to the first.
 * Returns -1 if no tap passed.
 */
static int lg1k_best_tap(const unsigned long *pass, int *start, int *len)
{
	int s, l;

	*start = 0;
	*len = 0;

	if (bitmap_full(pass, LG1K_NUM_IN_TAPS)) {
		*len = LG1K_NUM_IN_TAPS;
		return LG1K_NUM_IN_TAPS / 2;
	}

	for (s = 0; s < LG1K_NUM_IN_TAPS; s++) {
		/* only consider the first tap of each run */
		if (!test_bit(s, pass) ||
		    test_bit((s + LG1K_NUM_IN_TAPS - 1) % LG1K_NUM_IN_TAPS, pass))
			continue;
		for (l = 0; l < LG1K_NUM_IN_TAPS &&
		     test_bit((s + l) % LG1K_NUM_IN_TAPS, pass); l++)
			;
		if (l > *len) {
			*start = s;
			*len = l;
		}
	}

	if (!*len)
		return -1;

	return (*start + *len / 2) % LG1K_NUM_IN_TAPS;
}

/*
 * In HS400 read data is captured against the card's data strobe, so the
 * best input tap can differ from the HS200 one by a part-specific amount.
 * EXT_CSD reads in HS400 don't exercise the bus hard enough to find that
 * window's edges, so it isn't tuned; boards whose eMMC needs a shift
 * describe it with "lge,hs400-itap-offset".
 */
static u32 lg1k_hs400_tap(struct sdhci_lg1k *lg, u32 hs200_tap)
{
	return (hs200_tap + lg->hs400_itap_offset) & (LG1K_NUM_IN_TAPS - 1);
}

/* HS200: sweep the input tap with CMD21 */
static int lg1k_execute_tuning(struct sdhci_host *host, u32 opcode)
{
	struct sdhci_lg1k *lg = to_lg1k(host);
	struct mmc_host *mmc = host->mmc;
	DECLARE_BITMAP(pass, LG1K_NUM_IN_TAPS) = { };
	int tap, start, len;

	sdhci_writel(host, 1, LG1K_CLOCK_GATING);

	for (tap = 0; tap < LG1K_NUM_IN_TAPS; tap++) {
		lg1k_set_taps(lg, lg->hs200_out, tap);
		if (!mmc_send_tuning(mmc, opcode, NULL))
			__set_bit(tap, pass);
		else
			mmc_send_abort_tuning(mmc, opcode);
	}

	tap = lg1k_best_tap(pass, &start, &len);
	if (tap < 0) {
		dev_err(mmc_dev(mmc), "HS200 tuning failed: no passing input tap\n");
		lg1k_set_taps(lg, lg->hs200_out, lg->hs200_in);
		return -EIO;
	}

	lg->hs200_in = tap;
	lg->hs400_in = lg1k_hs400_tap(lg, tap);
	lg1k_set_taps(lg, lg->hs200_out, lg->hs200_in);

	dev_info(mmc_dev(mmc), "HS200 input tap %d (window %d+%d, map %*pb)\n",
		tap, start, len, LG1K_NUM_IN_TAPS, pass);

	return 0;
}

static unsigned int lg1k_get_max_clock(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);

	return pltfm_host->clock;
}

static unsigned int lg1k_get_min_clock(struct sdhci_host *host)
{
	return LG1K_MIN_CLOCK;
}

static int lg1k_start_signal_voltage_switch(struct mmc_host *mmc,
					    struct mmc_ios *ios)
{
	/*
	 * I/O is hard-wired to 1.8V. Setting the 1.8V enable bit below
	 * ~26 MHz upsets the Arasan core, so leave it to set_uhs_signaling.
	 */
	if (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_180)
		return 0;

	return sdhci_start_signal_voltage_switch(mmc, ios);
}

static const struct sdhci_ops lg1k_sdhci_ops = {
	.read_w			= lg1k_readw,
	.read_b			= lg1k_readb,
	.write_w		= lg1k_writew,
	.write_b		= lg1k_writeb,
	.set_clock		= lg1k_set_clock,
	.get_max_clock		= lg1k_get_max_clock,
	.get_min_clock		= lg1k_get_min_clock,
	.set_bus_width		= sdhci_set_bus_width,
	.reset			= sdhci_reset,
	.set_uhs_signaling	= lg1k_set_uhs_signaling,
	.platform_execute_tuning = lg1k_execute_tuning,
};

static const struct sdhci_pltfm_data lg1k_sdhci_pdata = {
	.ops	= &lg1k_sdhci_ops,
	.quirks	= SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |
		  SDHCI_QUIRK_FORCE_BLK_SZ_2048 |
		  SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN |
		  SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK,
	.quirks2 = SDHCI_QUIRK2_PRESET_VALUE_BROKEN,
};

static int sdhci_lg1k_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_host *host;
	struct sdhci_lg1k *lg;
	int ret;

	host = sdhci_pltfm_init(pdev, &lg1k_sdhci_pdata, sizeof(*lg));
	if (IS_ERR(host))
		return PTR_ERR(host);

	pltfm_host = sdhci_priv(host);
	lg = sdhci_pltfm_priv(pltfm_host);

	lg->top = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(lg->top))
		return PTR_ERR(lg->top);

	sdhci_get_property(pdev);
	if (!pltfm_host->clock)
		return dev_err_probe(dev, -EINVAL, "missing clock-frequency\n");

	ret = mmc_of_parse(host->mmc);
	if (ret)
		return ret;

	lg->hs200_in = LG1K_NUM_IN_TAPS / 2;
	lg->hs400_in = lg->hs200_in;
	lg->hs400_itap_offset = 0;

	/*
	 * A mode can only be used if its taps are described. Missing ones
	 * step the host down, as far as legacy timing (delay line unused).
	 * The capability registers are latched here so the core can't
	 * re-advertise what was masked.
	 */
	sdhci_read_caps(host);

	if (of_property_read_u32(np, "lge,otap-del-sel-mmc-hs", &lg->hs_out) ||
	    of_property_read_u32(np, "lge,itap-del-sel-mmc-hs", &lg->hs_in)) {
		dev_warn(dev, "no mmc-hs taps, limiting to legacy timing\n");
		host->caps &= ~SDHCI_CAN_DO_HISPD;
		host->mmc->caps &= ~(MMC_CAP_MMC_HIGHSPEED | MMC_CAP_1_8V_DDR);
		lg->hs200_out = U32_MAX;
	} else if (of_property_read_u32(np, "lge,otap-del-sel-hs200",
					&lg->hs200_out)) {
		lg->hs200_out = U32_MAX;
	}

	if (lg->hs200_out == U32_MAX) {
		host->quirks2 |= SDHCI_QUIRK2_BROKEN_HS200;
		host->mmc->caps2 &= ~(MMC_CAP2_HSX00_1_8V | MMC_CAP2_HS400_ES);
	} else if (of_property_read_u32(np, "lge,otap-del-sel-hs400",
					&lg->hs400_out)) {
		host->mmc->caps2 &= ~(MMC_CAP2_HS400_1_8V | MMC_CAP2_HS400_ES);
	}

	of_property_read_s32(np, "lge,hs400-itap-offset", &lg->hs400_itap_offset);

	of_property_read_u32(np, "lge,strobe-dly-sel-hs400", &lg->strb);
	of_property_read_u32(np, "lge,drive-strength-sel", &lg->host_ds);

	host->mmc_host_ops.start_signal_voltage_switch =
		lg1k_start_signal_voltage_switch;
	/*
	 * The DAT0 level in PRESENT_STATE stays "busy" once the bus is in
	 * HS400, so poll busy with CMD13 instead (as the vendor kernel did).
	 */
	host->mmc_host_ops.card_busy = NULL;

	return sdhci_add_host(host);
}

static const struct of_device_id sdhci_lg1k_of_match[] = {
	{ .compatible = "lge,lg1k-sdhci-5.0-v3" },
	{ }
};
MODULE_DEVICE_TABLE(of, sdhci_lg1k_of_match);

static struct platform_driver sdhci_lg1k_driver = {
	.driver = {
		.name = "sdhci-lg1k",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = sdhci_lg1k_of_match,
		.pm = &sdhci_pltfm_pmops,
	},
	.probe = sdhci_lg1k_probe,
	.remove = sdhci_pltfm_remove,
};
module_platform_driver(sdhci_lg1k_driver);

MODULE_DESCRIPTION("SDHCI driver for LG lg1k eMMC 5.0 host");
MODULE_LICENSE("GPL");
