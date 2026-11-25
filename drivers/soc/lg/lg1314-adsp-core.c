// SPDX-License-Identifier: GPL-2.0
/*
 * LG1314 audio DSPs (2 x Xtensa): bring-up.
 *
 * DSP0 decodes and mixes, DSP1 post-processes and drives the outputs. Both
 * run LG's audio firmware (05.50 images) out of a reserved DRAM area: each
 * DSP sees its image through window 0x50000000 ("offset5"), its base memory
 * through 0x60000000 ("offset6") and the sound-engine parameters through the
 * "offset61" window. The host fills in the memory map shared with the
 * firmware (audio block 0x1000-0x13ff), releases the cores one at a time and
 * waits for each to report "download done" over IMC.
 *
 * Every register access to the audio block must be a full 32-bit access:
 * SWRESET in particular takes a byte write as a full-word write with the other
 * bytes zero, which scrambles the DSP resets.
 */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "lg1314-adsp.h"

/* Audio block registers */
#define AUD_BC_IEC_STOP		0x064
#define AUD_IEC_CTRL		0x120
#define AUD_HDMI_ARC_CTRL	0x614
#define AUD_INT_CLR(n)		(0x704 + 0xc * (n))
#define AUD_INT_EN(n)		(0x708 + 0xc * (n))
#define AUD_RUNSTALL(n)		(0x718 + 4 * (n))
#define AUD_PRID(n)		(0x730 + 4 * (n))
#define AUD_SWRESET		0x738
#define AUD_DEBUGEN(n)		(0x748 + 4 * (n))
#define AUD_DEBUGPC(n)		(0x768 + 4 * (n))
#define AUD_OFFSET5(n)		((n) ? 0x7b4 : 0x794)
#define AUD_OFFSET6(n)		((n) ? 0x7b8 : 0x798)
#define AUD_OFFSET61(n)		(0x7d8 + 4 * (n))
#define AUD_SCT0BYPASS		0x7ec
#define AUD_BC_RESET		0x904
#define AUD_AOC_CTRL		0xe00
#define AUD_SHARED		0x1000		/* 0x1000-0x13ff: host/firmware map */
#define AUD_SHARED_SIZE		0x400
#define AUD_RING_BASE		0x100c		/* 8 x {base, size}: a2p p2a a2d d2a d2p p2d dbgp dbgd */
#define AUD_SE_BASE		0x104c
#define AUD_SE_SIZE		0x1050
#define AUD_BUF_REVERSE		0x1130		/* 4 words, "checkbit" init */
#define AUD_OFF_SCHEDULER	0x1258
#define AUD_HB(n)		(0x12ac + 4 * (n))	/* heartbeat counters */
#define AUD_VER_CHIP		0x12e8		/* SoC revision, lx_chip_rev() */
#define AUD_PTS_LIP0		0x1274
#define AUD_PTS_LIP1		0x1278
#define AUD_TPM_BUFSIZE		0x127c
#define AUD_TPA_BUFSIZE		0x1280
#define AUD_DEC_BASETIME	0x12c0		/* dec0 cbt/sbt, dec1 cbt/sbt, basepts, basestc */
#define AUD_VER1(n)		(0x12f0 + 4 * (n))
#define AUD_VER0(n)		(0x12f8 + 4 * (n))
#define AUD_BC_TABLE		0x1300		/* 16 x {base, size, amfl, ul} */

/*
 * The firmware picks chip-specific paths from aud_ver_chip (e.g. the HDMI input
 * select in HAL_SetHdmiInputPort is skipped below M19). Stock writes
 * lx_chip_rev(): chip << 8 | revision, M19 = 0x23; the B9 is M19 A1.
 */
#define LG1314_CHIP_REV		0x23a1

#define SWRESET_VAL		0x019fc13f	/* AXI, APB, AVER, FS, AAD, HMR, I2S */
#define SWRESET_AXI		BIT(0)
#define SWRESET_DSP(n)		((n) ? (BIT(13) | BIT(22)) : (BIT(12) | BIT(21)))
#define SWRESET_OCD(n)		BIT(10 + (n))

/* AUD_SYN (0xc9c60c00): memory flush handshake */
#define SYN_FLUSH_EN		0x138
#define SYN_FLUSH_DONE		0x13c

/* Audio bus port protector (LBUS MPRO) */
#define MPRO_ISR_CLEAR		0x0c
#define MPRO_SLOT_EN		0x24
#define MPRO_RANGE(n)		(0x28 + 8 * (n))

/*
 * BMC_SYN (0xc930e000): audio clock muxes, DTOs and clock-domain resets.
 * Stock programs these in MIXED_IP_ADEC_InitClockRegisters + SetClockSource
 * (48 kHz); the speaker/S/PDIF/ARC outputs and the DSPs' sample-rate
 * interrupts run from them. The APLL feeding them (CTOP_BMC r10/r11) is
 * already up from the boot loader.
 */
#define BMC_SWRST		0x04
#define BMC_ACLK_MUX(n)		(0x08 + 4 * (n))	/* 4 regs, mux1..13 {src, div} */
#define BMC_ADTO_ADD(n)		(0x18 + 8 * (n))	/* DTO 0: S/PDIF, 1: ARC, 2 */
#define BMC_ADTO_ERR(n)		(0x1c + 8 * (n))
#define BMC_ADTO_ERR_MASK	GENMASK(11, 0)
#define BMC_ADTO_SYNC		BIT(16)
/* audio bits of BMC_SWRST: f256fs .. fs00 (0-18), auad (23) */
#define BMC_SWRST_AUDIO		(GENMASK(18, 0) | BIT(23))
#define BMC_SWRST_AUDIO_VAL	(BIT(6) | BIT(7) | BIT(17))	/* fs24, fs23, adto4 held */
#define CTOP_APLL_R11		0x42c			/* in the BMC page */
#define CTOP_APLL_UP		(BIT(24) | BIT(25) | BIT(26))	/* pdb, rstb, dco_rstb */


/* Buffer controllers: base (in the area), size, upper limit. */
static const struct {
	u32 off, size, ul;
} adsp_bcs[16] = {
	{ 0x2c00000, 0x00200, 0x100 },	/* TP main */
	{ 0x2c08000, 0x00200, 0x100 },	/* TP sub */
	{ 0x2c10000, 0x00800, 0x400 },	/* SAI0 (HDMI) */
	{ 0x2c18000, 0x00800, 0x400 },	/* SAI1 (HDMI) */
	{ 0x2c20000, 0x00200, 0x100 },	/* SAI2 (SIF) */
	{ 0x2c28000, 0x00200, 0x100 },	/* SAI3 (ADC) */
	{ 0x2c30000, 0x00200, 0x100 },	/* SAI4 (ADC) */
	{ 0x2c38000, 0x00400, 0x200 },	/* AOC main (speaker) */
	{ 0x2c40000, 0x00200, 0x100 },	/* AOC sub (headphone) */
	{ 0x2c48000, 0x00200, 0x100 },	/* AOC S/PDIF */
	{ 0x2c50000, 0x00800, 0x400 },	/* AOC ARC */
	{ 0x2c58000, 0x00800, 0x400 },	/* AOC eARC */
	{ 0x2c60000, 0x1f800, 0x900 },	/* AAD input 0 */
	{ 0x2ca0000, 0x1f800, 0x900 },	/* AAD input 1 */
	{ 0x2ce0000, 0x09f40, 0x2d8 },	/* AAD input 1 (NICAM) */
	{ 0x2d20000, 0x1f800, 0x900 },	/* AAD input 2 */
};

static void adsp_init_shared(struct lg1314_adsp *adsp)
{
	static const u8 ring_order[8] = {
		ADSP_RING_A2P, ADSP_RING_P2A, ADSP_RING_A2D, ADSP_RING_D2A,
		ADSP_RING_D2P, ADSP_RING_P2D, ADSP_RING_DBGP, ADSP_RING_DBGD,
	};
	phys_addr_t base = adsp->mem_phys;
	int i;

	for (i = 0; i < AUD_SHARED_SIZE; i += 4)
		adsp_wr(adsp, AUD_SHARED + i, 0);

	adsp_wr(adsp, AUD_HDMI_ARC_CTRL, adsp_rd(adsp, AUD_HDMI_ARC_CTRL) | 1);
	adsp_wr(adsp, AUD_SCT0BYPASS, adsp_rd(adsp, AUD_SCT0BYPASS) & ~0x1f);

	adsp_wr(adsp, AUD_SHARED + 0x0, base + MEM_SE);		/* shared (SE) */
	adsp_wr(adsp, AUD_SHARED + 0x4, base + MEM_DSP_BASE(0));
	adsp_wr(adsp, AUD_SHARED + 0x8, base + MEM_DSP_BASE(1));
	for (i = 0; i < 8; i++) {
		struct adsp_ring *r = &adsp->ring[ring_order[i]];

		adsp_wr(adsp, AUD_RING_BASE + 8 * i,
			base + MEM_IPC + (r->mem - (adsp->mem + MEM_IPC)));
		adsp_wr(adsp, AUD_RING_BASE + 8 * i + 4, r->size);
	}
	adsp_wr(adsp, AUD_SE_BASE, base + MEM_SE);
	adsp_wr(adsp, AUD_SE_SIZE, MEM_SE_SIZE);

	for (i = 0; i < 6; i++)
		adsp_wr(adsp, AUD_DEC_BASETIME + 4 * i, 0xffffffff);
	adsp_wr(adsp, AUD_PTS_LIP0, 0xffffffff);
	adsp_wr(adsp, AUD_PTS_LIP1, 0xffffffff);
	adsp_wr(adsp, AUD_TPM_BUFSIZE, 0x03a00380);
	adsp_wr(adsp, AUD_TPA_BUFSIZE, 0x03a00380);
	adsp_wr(adsp, AUD_OFF_SCHEDULER, 0x00010001);
	adsp_wr(adsp, AUD_VER_CHIP, LG1314_CHIP_REV);

	for (i = 0; i < ARRAY_SIZE(adsp_bcs); i++) {
		adsp_wr(adsp, AUD_BC_TABLE + 16 * i + 0, base + adsp_bcs[i].off);
		adsp_wr(adsp, AUD_BC_TABLE + 16 * i + 4, adsp_bcs[i].size);
		adsp_wr(adsp, AUD_BC_TABLE + 16 * i + 12, adsp_bcs[i].ul);
	}
	for (i = 0; i < 4; i++)
		adsp_wr(adsp, AUD_BUF_REVERSE + 4 * i, 0xa0000000);
}

/* Deny the audio bus master everything outside its area. */
static void adsp_init_mpro(struct lg1314_adsp *adsp)
{
	writel(0, adsp->mpro + MPRO_RANGE(0));
	writel((adsp->mem_phys - 1) & 0xfffff000, adsp->mpro + MPRO_RANGE(0) + 4);
	writel(adsp->mem_phys + adsp->mem_size, adsp->mpro + MPRO_RANGE(1));
	writel(0xfffff000, adsp->mpro + MPRO_RANGE(1) + 4);
	writel(0x111, adsp->mpro + MPRO_SLOT_EN);
	writel(0xc000, adsp->mpro + MPRO_ISR_CLEAR);
}

/* Stall both cores, stop the outputs and buffer controllers, flush memory. */
static int adsp_reset_bc(struct lg1314_adsp *adsp)
{
	u32 done;
	int n, ret;

	for (n = 0; n < ADSP_NUM_DSP; n++) {
		adsp_wr(adsp, AUD_RUNSTALL(n), 1);
		adsp_wr(adsp, AUD_INT_EN(n), 0);
	}
	adsp_wr(adsp, AUD_IEC_CTRL, 0);
	adsp_wr(adsp, AUD_AOC_CTRL, 0);
	adsp_wr(adsp, AUD_BC_IEC_STOP, 0x11);
	adsp_wr(adsp, AUD_BC_IEC_STOP, 0);
	adsp_wr(adsp, AUD_BC_RESET, 0x11111);
	adsp_wr(adsp, AUD_BC_RESET, 0);

	writel(1, adsp->syn + SYN_FLUSH_EN);
	ret = readl_poll_timeout(adsp->syn + SYN_FLUSH_DONE, done, done, 100, 20000);
	writel(0, adsp->syn + SYN_FLUSH_EN);
	if (ret)
		dev_warn(adsp->dev, "memory flush timed out\n");
	return 0;
}

static int adsp_boot_dsp(struct lg1314_adsp *adsp, int n, const struct firmware *fw)
{
	u32 dsp_base = adsp->mem_phys + MEM_DSP_BASE(n);
	u32 img_off = MEM_DSP_BASE(n) + MEM_DSP_SIZE(n) - MEM_DSP_IMAGE(n);
	struct adsp_waiter *w;
	u32 v;
	int ret;

	if (fw->size > MEM_DSP_IMAGE(n)) {
		dev_err(adsp->dev, "dsp%d: image too big (%zu)\n", n, fw->size);
		return -EINVAL;
	}

	adsp_wr(adsp, AUD_RUNSTALL(n), 1);
	adsp_wr(adsp, AUD_INT_EN(n), 0);
	v = adsp_rd(adsp, AUD_SWRESET) & ~(SWRESET_DSP(n) | SWRESET_OCD(n));
	adsp_wr(adsp, AUD_SWRESET, v);
	adsp_wr(adsp, AUD_DEBUGEN(n), 1);
	adsp_wr(adsp, AUD_PRID(n), n);
	adsp_wr(adsp, AUD_VER0(n), 0);
	adsp_wr(adsp, AUD_VER1(n), 0);
	adsp_wr(adsp, AUD_OFFSET5(n), adsp->mem_phys + img_off);
	adsp_wr(adsp, AUD_OFFSET6(n), dsp_base);
	adsp_wr(adsp, AUD_OFFSET61(n), adsp->mem_phys + MEM_SE);

	memcpy(adsp->mem + img_off, fw->data, fw->size);
	wmb();

	/* the notification arrives once the firmware has initialised */
	w = adsp_imc_register_event(adsp, n, ADSP_MOD_MAN_DSP(n), ADSP_EVT_DSP_DN_DONE, 1);
	if (IS_ERR(w))
		return PTR_ERR(w);

	adsp_wr(adsp, AUD_SWRESET, adsp_rd(adsp, AUD_SWRESET) | SWRESET_DSP(n));
	adsp_wr(adsp, AUD_INT_EN(n), 1);
	adsp_wr(adsp, AUD_RUNSTALL(n), 0);

	ret = adsp_wait(adsp, w, 3000);
	adsp_waiter_put(adsp, w);
	if (ret) {
		dev_err(adsp->dev, "dsp%d: no download-done (%d), pc %08x\n", n, ret,
			adsp_rd(adsp, AUD_DEBUGPC(n)));
		return ret ? ret : -EIO;
	}
	dev_info(adsp->dev, "dsp%d: running (\"%s\")\n", n, adsp->proc_name[n]);
	return 0;
}

static void adsp_init_clocks(struct lg1314_adsp *adsp)
{
	static const u32 mux[4] = { 0x64141004, 0x14140101, 0x04040110, 0x00000074 };
	static const u32 dto[3] = { 0x1021c6b8, 0x10219700, 0x1021c6b8 };	/* 48 kHz */
	void __iomem *bmc = adsp->bmc;
	u32 v;
	int i;

	v = readl(bmc + CTOP_APLL_R11);
	if ((v & CTOP_APLL_UP) != CTOP_APLL_UP)
		dev_warn(adsp->dev, "audio PLL is down (%08x)\n", v);

	for (i = 0; i < ARRAY_SIZE(mux); i++)
		writel(mux[i], bmc + BMC_ACLK_MUX(i));
	for (i = 0; i < ARRAY_SIZE(dto); i++) {
		writel(dto[i], bmc + BMC_ADTO_ADD(i));
		v = readl(bmc + BMC_ADTO_ERR(i)) & ~(BMC_ADTO_ERR_MASK | BMC_ADTO_SYNC);
		writel(v, bmc + BMC_ADTO_ERR(i));
	}
	for (i = 0; i < ARRAY_SIZE(dto); i++)
		writel(readl(bmc + BMC_ADTO_ERR(i)) | BMC_ADTO_SYNC, bmc + BMC_ADTO_ERR(i));
	udelay(100);

	v = readl(bmc + BMC_SWRST);
	writel((v & ~BMC_SWRST_AUDIO) | BMC_SWRST_AUDIO_VAL, bmc + BMC_SWRST);
}

static int adsp_boot(struct lg1314_adsp *adsp)
{
	const struct firmware *fw[ADSP_NUM_DSP] = {};
	u32 sw = adsp_rd(adsp, AUD_SWRESET);
	int n, ret;

	/*
	 * Resetting the audio AXI interface after the DSPs have run leaves them
	 * unable to translate their addresses until the next reboot.
	 */
	if (sw & (SWRESET_DSP(0) | SWRESET_DSP(1))) {
		dev_err(adsp->dev, "DSPs already started this boot (SWRESET %08x); reboot to restart them\n",
			sw);
		return -EBUSY;
	}

	for (n = 0; n < ADSP_NUM_DSP; n++) {
		char name[32];

		snprintf(name, sizeof(name), "lg1314/audio_dsp%d.bin", n);
		ret = request_firmware(&fw[n], name, adsp->dev);
		if (ret) {
			dev_err(adsp->dev, "%s: %d\n", name, ret);
			goto out;
		}
	}

	memset(adsp->mem + MEM_SE, 0, MEM_SE_SIZE);
	memset(adsp->mem + MEM_IPC, 0, MEM_IPC_SIZE);
	adsp_init_mpro(adsp);
	adsp_wr(adsp, AUD_SWRESET, sw | SWRESET_VAL);
	adsp_init_clocks(adsp);
	adsp_init_shared(adsp);
	adsp_reset_bc(adsp);
	adsp->running = true;
	schedule_delayed_work(&adsp->poll_work, msecs_to_jiffies(20));

	ret = adsp_boot_dsp(adsp, 1, fw[1]);
	if (!ret)
		ret = adsp_boot_dsp(adsp, 0, fw[0]);
out:
	for (n = 0; n < ADSP_NUM_DSP; n++)
		release_firmware(fw[n]);
	return ret;
}

static irqreturn_t adsp_irq(int irq, void *data)
{
	struct lg1314_adsp *adsp = data;
	int n = irq == adsp->irq[1];

	atomic_inc(&adsp->irq_count[n]);
	adsp_wr(adsp, AUD_INT_CLR(n), 1);
	adsp_wr(adsp, AUD_INT_CLR(n), 0);
	queue_work(system_highpri_wq, &adsp->rx_work);
	return IRQ_HANDLED;
}

/* Backstop for missed interrupts (and the only receive path without them). */
static void adsp_poll_work(struct work_struct *work)
{
	struct lg1314_adsp *adsp = container_of(to_delayed_work(work),
						struct lg1314_adsp, poll_work);

	queue_work(system_highpri_wq, &adsp->rx_work);
	if (adsp->running)
		schedule_delayed_work(&adsp->poll_work, msecs_to_jiffies(100));
}


/* ---- module control (LG framework registry / connector) ---- */

static int uid_dsp(u32 uid)
{
	u32 proc = ADSP_UID_PROC(uid);

	if (proc == ADSP_CORE_DSP(0) || proc == ADSP_CORE_DSP(1))
		return proc - ADSP_CORE_DSP(0);
	if (uid == ADSP_MOD_MAN_DSP(0) || uid == ADSP_MOD_MAN_DSP(1))
		return uid - ADSP_MOD_MAN_DSP(0);
	if (uid == ADSP_MOD_DBG_DSP(0) || uid == ADSP_MOD_DBG_DSP(1))
		return uid - ADSP_MOD_DBG_DSP(0);
	return -EINVAL;
}

int adsp_module_cmd(struct lg1314_adsp *adsp, u32 uid, u32 cmd, const void *param,
		    u32 len, void *rsp, u32 rsp_len)
{
	int dsp = uid_dsp(uid);

	if (dsp < 0)
		return dsp;
	return adsp_imc_command(adsp, dsp, uid, cmd, param, len, rsp, rsp_len, 1000);
}
EXPORT_SYMBOL_GPL(adsp_module_cmd);

int adsp_module_create(struct lg1314_adsp *adsp, int dsp, const char *name, u32 *uid)
{
	struct {
		char name[32];
		u32 uid;
	} param = {};
	struct adsp_impl *impl, *found = NULL;
	int ret;

	mutex_lock(&adsp->impl_lock);
	list_for_each_entry(impl, &adsp->impls[dsp], list)
		if (!strcmp(impl->name, name))
			found = impl;
	if (found)
		param.uid = ADSP_UID(ADSP_CORE_DSP(dsp), found->num, found->next_index++);
	mutex_unlock(&adsp->impl_lock);
	if (!found)
		return -ENOENT;

	strscpy(param.name, name, sizeof(param.name));
	ret = adsp_imc_command(adsp, dsp, ADSP_UID_REGISTRY(ADSP_CORE_DSP(dsp)),
			       ADSP_REGISTRY_CREATE, &param, sizeof(param), NULL, 0, 1000);
	if (ret)
		return ret;
	*uid = param.uid;
	return 0;
}
EXPORT_SYMBOL_GPL(adsp_module_create);

int adsp_module_destroy(struct lg1314_adsp *adsp, u32 uid)
{
	int dsp = uid_dsp(uid);

	if (dsp < 0)
		return dsp;
	return adsp_imc_command(adsp, dsp, ADSP_UID_REGISTRY(ADSP_UID_PROC(uid)),
				ADSP_REGISTRY_DESTROY, &uid, sizeof(uid), NULL, 0, 1000);
}
EXPORT_SYMBOL_GPL(adsp_module_destroy);

/* Both modules on the same DSP. */
int adsp_module_connect(struct lg1314_adsp *adsp, u32 out_uid, u32 out_port,
			u32 in_uid, u32 in_port)
{
	u32 param[4] = { out_uid, out_port, in_uid, in_port };
	int dsp = uid_dsp(out_uid);

	if (dsp < 0 || uid_dsp(in_uid) != dsp)
		return -EINVAL;
	return adsp_imc_command(adsp, dsp, ADSP_UID_CONNECTOR(ADSP_UID_PROC(out_uid)),
				ADSP_CONNECTOR_CONNECT, param, sizeof(param), NULL, 0, 1000);
}
EXPORT_SYMBOL_GPL(adsp_module_connect);

int adsp_module_disconnect(struct lg1314_adsp *adsp, u32 out_uid, u32 out_port,
			   u32 in_uid, u32 in_port)
{
	u32 param[4] = { out_uid, out_port, in_uid, in_port };
	int dsp = uid_dsp(out_uid);

	if (dsp < 0 || uid_dsp(in_uid) != dsp)
		return -EINVAL;
	return adsp_imc_command(adsp, dsp, ADSP_UID_CONNECTOR(ADSP_UID_PROC(out_uid)),
				ADSP_CONNECTOR_DISCONNECT, param, sizeof(param), NULL, 0, 1000);
}
EXPORT_SYMBOL_GPL(adsp_module_disconnect);

/* New-style modules: "name=value" string properties. */
int adsp_module_set_prop(struct lg1314_adsp *adsp, u32 uid, const char *prop)
{
	u32 len = strlen(prop) + 1;
	u8 param[ADSP_IPC_MAX_PARAM];

	if (4 + ALIGN(len, 4) > sizeof(param))
		return -EINVAL;
	memset(param, 0, sizeof(param));
	memcpy(param, &len, 4);
	memcpy(param + 4, prop, len);
	return adsp_module_cmd(adsp, uid, ADSP_MOD_SET_PROP, param, 4 + ALIGN(len, 4),
			       NULL, 0);
}
EXPORT_SYMBOL_GPL(adsp_module_set_prop);

/*
 * debugfs "ctl": drive the framework by hand while the graph is worked out.
 *   create <dsp> <impl> | destroy <uid> | connect <uid> <port> <uid> <port>
 *   cmd <uid> <cmd> [u32 ...]           (all numbers hex)
 *   prop <uid> <name=value>             string property
 *   loglevel <dsp> <log module> <level> firmware log level (0 none .. 6 trace)
 *   shbuf <glue uid> <size> <au len> <au count> [chunk]
 *                                       buffer into a reader glue; "pcm" feeds it
 *   shbuf_destroy
 * Reading returns the last result.
 */
static ssize_t adsp_ctl_write(struct file *f, const char __user *ubuf, size_t len, loff_t *off)
{
	struct lg1314_adsp *adsp = file_inode(f)->i_private;
	char buf[256], *p = buf, *tok, *argv[40];
	int argc = 0, ret = -EINVAL;
	u32 v[40], rsp[32] = {};

	if (len >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	buf[len] = 0;
	while ((tok = strsep(&p, " \t\n")) && argc < ARRAY_SIZE(argv))
		if (*tok)
			argv[argc++] = tok;
	if (!argc)
		return len;
	for (int i = 1; i < argc; i++)
		if (kstrtou32(argv[i], 16, &v[i]))
			v[i] = 0;

	if (!strcmp(argv[0], "create") && argc == 3) {
		u32 uid = 0;

		ret = adsp_module_create(adsp, v[1], argv[2], &uid);
		snprintf(adsp->ctl_result, sizeof(adsp->ctl_result), "%d %08x\n", ret, uid);
	} else if (!strcmp(argv[0], "destroy") && argc == 2) {
		ret = adsp_module_destroy(adsp, v[1]);
		snprintf(adsp->ctl_result, sizeof(adsp->ctl_result), "%d\n", ret);
	} else if (!strcmp(argv[0], "connect") && argc == 5) {
		ret = adsp_module_connect(adsp, v[1], v[2], v[3], v[4]);
		snprintf(adsp->ctl_result, sizeof(adsp->ctl_result), "%d\n", ret);
	} else if ((!strcmp(argv[0], "cmd") && argc >= 3) ||
		   (!strcmp(argv[0], "dcmd") && argc >= 4)) {
		/* dcmd: DSP given explicitly, for targets the uid doesn't place (legacy ids) */
		int first = argv[0][0] == 'd' ? 2 : 1;
		u32 words[32];
		int n = argc - first - 2, o, i;

		for (i = 0; i < n && i < ARRAY_SIZE(words); i++)
			words[i] = v[first + 2 + i];
		if (first == 2)
			ret = adsp_imc_command(adsp, v[1] & 1, v[2], v[3], words, 4 * n, rsp,
					       sizeof(rsp), 1000);
		else
			ret = adsp_module_cmd(adsp, v[1], v[2], words, 4 * n, rsp, sizeof(rsp));
		o = snprintf(adsp->ctl_result, sizeof(adsp->ctl_result), "%d", ret);
		for (i = 0; i < 8 && o < sizeof(adsp->ctl_result) - 10; i++)
			o += snprintf(adsp->ctl_result + o, sizeof(adsp->ctl_result) - o, " %08x", rsp[i]);
		snprintf(adsp->ctl_result + o, sizeof(adsp->ctl_result) - o, "\n");
	} else if (!strcmp(argv[0], "loglevel") && argc == 4) {
		struct {
			s32 level;
			char name[32];
		} lp = { .level = v[3] };

		strscpy(lp.name, argv[2], sizeof(lp.name));
		ret = adsp_module_cmd(adsp, ADSP_MOD_DBG_DSP(v[1] & 1), ADSP_DBG_MESSAGE_LEVEL,
				      &lp, sizeof(lp), NULL, 0);
		snprintf(adsp->ctl_result, sizeof(adsp->ctl_result), "%d\n", ret);
	} else if (!strcmp(argv[0], "prop") && argc == 3) {
		ret = adsp_module_set_prop(adsp, v[1], argv[2]);
		snprintf(adsp->ctl_result, sizeof(adsp->ctl_result), "%d\n", ret);
	} else if ((!strcmp(argv[0], "shbuf") && argc >= 5 && !adsp->dbg_sb) ||
		   ((!strcmp(argv[0], "dshbuf") || !strcmp(argv[0], "xshbuf")) && argc >= 6 &&
		    !adsp->dbg_sb)) {
		/*
		 * dshbuf <dsp> <reader> ...: reader given as a legacy module id;
		 * xshbuf: the same, allocated only (attach it with a dcmd)
		 */
		int f = argv[0][0] != 's';
		struct adsp_shbuf *sb = argv[0][0] == 'x' ?
			adsp_shbuf_alloc(adsp, v[1] & 1, v[2], v[3], v[4], v[5]) : f ?
			adsp_shbuf_create_on(adsp, v[1] & 1, v[2], v[3], v[4], v[5]) :
			adsp_shbuf_create(adsp, v[1], v[2], v[3], v[4]);

		ret = PTR_ERR_OR_ZERO(sb);
		if (!ret) {
			adsp->dbg_sb = sb;
			adsp->dbg_chunk = argc > 5 + f ? v[5 + f] : sb->au_max_len;
		}
		snprintf(adsp->ctl_result, sizeof(adsp->ctl_result), "%d %08x %08x\n", ret,
			 ret ? 0 : sb->uid, ret ? 0 : sb->offset);
	} else if (!strcmp(argv[0], "shbuf_destroy") && adsp->dbg_sb) {
		adsp_shbuf_destroy(adsp->dbg_sb);
		adsp->dbg_sb = NULL;
		snprintf(adsp->ctl_result, sizeof(adsp->ctl_result), "0\n");
	} else {
		snprintf(adsp->ctl_result, sizeof(adsp->ctl_result), "bad command\n");
	}
	return len;
}

static ssize_t adsp_ctl_read(struct file *f, char __user *ubuf, size_t len, loff_t *off)
{
	struct lg1314_adsp *adsp = file_inode(f)->i_private;

	return simple_read_from_buffer(ubuf, len, off, adsp->ctl_result,
				       strlen(adsp->ctl_result));
}

static const struct file_operations adsp_ctl_fops = {
	.write = adsp_ctl_write,
	.read = adsp_ctl_read,
};

/* debugfs "pcm": raw data into the debug shared buffer, one AU per chunk. */
static ssize_t adsp_pcm_write(struct file *f, const char __user *ubuf, size_t len, loff_t *off)
{
	struct lg1314_adsp *adsp = file_inode(f)->i_private;
	struct adsp_shbuf *sb = adsp->dbg_sb;
	size_t done = 0;
	void *chunk;
	int ret = 0;

	if (!sb || !adsp->dbg_chunk || adsp->dbg_chunk > sb->au_max_len)
		return -ENODEV;
	chunk = kmalloc(adsp->dbg_chunk, GFP_KERNEL);
	if (!chunk)
		return -ENOMEM;
	while (done < len) {
		u32 n = min_t(size_t, len - done, adsp->dbg_chunk);

		if (copy_from_user(chunk, ubuf + done, n)) {
			ret = -EFAULT;
			break;
		}
		ret = wait_event_interruptible_timeout(sb->wq,
				adsp_shbuf_free_space(sb) >= n, msecs_to_jiffies(2000));
		if (ret <= 0) {
			ret = ret ? ret : -ETIMEDOUT;
			break;
		}
		ret = adsp_shbuf_write_au(sb, chunk, n);
		if (ret)
			break;
		done += n;
	}
	kfree(chunk);
	return done ? done : ret;
}

static const struct file_operations adsp_pcm_fops = {
	.write = adsp_pcm_write,
};

static int adsp_status_show(struct seq_file *s, void *unused)
{
	struct lg1314_adsp *adsp = s->private;
	struct adsp_impl *impl;
	int n, i;

	seq_printf(s, "swreset %08x\n", adsp_rd(adsp, AUD_SWRESET));
	for (n = 0; n < ADSP_NUM_DSP; n++)
		seq_printf(s, "dsp%d: \"%s\" pc %08x stall %u heartbeat %08x irqs %u\n", n,
			   adsp->proc_name[n], adsp_rd(adsp, AUD_DEBUGPC(n)),
			   adsp_rd(adsp, AUD_RUNSTALL(n)), adsp_rd(adsp, AUD_HB(n)),
			   atomic_read(&adsp->irq_count[n]));
	for (i = 0; i < ADSP_RING_NUM; i++)
		seq_printf(s, "ring %-4s r %05x w %05x\n", adsp->ring[i].name,
			   adsp_rd(adsp, adsp->ring[i].rptr), adsp_rd(adsp, adsp->ring[i].wptr));
	mutex_lock(&adsp->impl_lock);
	for (n = 0; n < ADSP_NUM_DSP; n++) {
		seq_printf(s, "dsp%d impls:", n);
		list_for_each_entry(impl, &adsp->impls[n], list)
			seq_printf(s, " %s(%02x)", impl->name, impl->num);
		seq_puts(s, "\n");
	}
	mutex_unlock(&adsp->impl_lock);
	if (adsp->dbg_sb) {
		struct adsp_shbuf *sb = adsp->dbg_sb;

		seq_printf(s, "shbuf %08x -> %08x at %08x: data %#x, %u AUs; "
			   "read %d/%d write %d/%d\n", sb->uid, sb->reader, sb->offset,
			   sb->data_size, sb->max_au, sb->st.read_idx, sb->st.read_off,
			   sb->st.write_idx, sb->st.write_off);
	}
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(adsp_status);

static int lg1314_adsp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct lg1314_adsp *adsp;
	struct reserved_mem *rmem;
	struct device_node *np;
	struct resource *res;
	int n, ret;

	adsp = devm_kzalloc(dev, sizeof(*adsp), GFP_KERNEL);
	if (!adsp)
		return -ENOMEM;
	adsp->dev = dev;
	mutex_init(&adsp->tx_lock);
	mutex_init(&adsp->impl_lock);
	mutex_init(&adsp->shbuf_lock);
	INIT_LIST_HEAD(&adsp->shbufs);
	spin_lock_init(&adsp->wait_lock);
	INIT_LIST_HEAD(&adsp->waiters);
	for (n = 0; n < ADSP_NUM_DSP; n++)
		INIT_LIST_HEAD(&adsp->impls[n]);
	INIT_WORK(&adsp->rx_work, adsp_rx_work);
	INIT_DELAYED_WORK(&adsp->poll_work, adsp_poll_work);
	platform_set_drvdata(pdev, adsp);

	adsp->aud = devm_platform_ioremap_resource_byname(pdev, "aud");
	if (IS_ERR(adsp->aud))
		return PTR_ERR(adsp->aud);
	/* shared pages (pinctrl owns AUD_SYN's and BMC's, the LBUS block the port's) */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "syn");
	adsp->syn = res ? devm_ioremap(dev, res->start, resource_size(res)) : NULL;
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "mpro");
	adsp->mpro = res ? devm_ioremap(dev, res->start, resource_size(res)) : NULL;
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "bmc");
	adsp->bmc = res ? devm_ioremap(dev, res->start, resource_size(res)) : NULL;
	if (!adsp->syn || !adsp->mpro || !adsp->bmc)
		return dev_err_probe(dev, -ENODEV, "missing syn/mpro/bmc registers\n");

	np = of_parse_phandle(dev->of_node, "memory-region", 0);
	rmem = np ? of_reserved_mem_lookup(np) : NULL;
	of_node_put(np);
	if (!rmem || rmem->size < MEM_END)
		return dev_err_probe(dev, -EINVAL, "need a %#x-byte memory-region\n", MEM_END);
	adsp->mem_phys = rmem->base;
	adsp->mem_size = rmem->size;
	adsp->mem = devm_memremap(dev, rmem->base, rmem->size, MEMREMAP_WC);
	if (IS_ERR(adsp->mem))
		return PTR_ERR(adsp->mem);
	adsp_ipc_init(adsp);
	ret = adsp_pool_init(adsp, MEM_POOL, MEM_END - MEM_POOL);
	if (ret)
		return ret;

	for (n = 0; n < ADSP_NUM_DSP; n++) {
		adsp->irq[n] = platform_get_irq(pdev, n);
		if (adsp->irq[n] < 0)
			return adsp->irq[n];
		ret = devm_request_irq(dev, adsp->irq[n], adsp_irq, 0, dev_name(dev), adsp);
		if (ret)
			return ret;
	}

	adsp->debugfs = debugfs_create_dir("lg1314-adsp", NULL);
	debugfs_create_file("status", 0444, adsp->debugfs, adsp, &adsp_status_fops);
	debugfs_create_file("ctl", 0600, adsp->debugfs, adsp, &adsp_ctl_fops);
	debugfs_create_file("pcm", 0200, adsp->debugfs, adsp, &adsp_pcm_fops);

	ret = adsp_boot(adsp);
	if (!ret)
		ret = adsp_pcm_init(adsp);
	if (ret)
		dev_err(dev, "DSP boot or speaker setup failed: %d\n", ret);
	else
		adsp_hdmi_init(adsp);
	/* stay bound either way: status and logs remain available */
	return 0;
}

static void lg1314_adsp_remove(struct platform_device *pdev)
{
	struct lg1314_adsp *adsp = platform_get_drvdata(pdev);
	struct adsp_impl *impl, *tmp;
	int n;

	adsp_hdmi_exit(adsp);
	adsp_pcm_exit(adsp);
	adsp->running = false;
	cancel_delayed_work_sync(&adsp->poll_work);
	for (n = 0; n < ADSP_NUM_DSP; n++)
		disable_irq(adsp->irq[n]);
	cancel_work_sync(&adsp->rx_work);
	debugfs_remove_recursive(adsp->debugfs);
	while (!list_empty(&adsp->shbufs))
		adsp_shbuf_destroy(list_first_entry(&adsp->shbufs, struct adsp_shbuf, list));
	for (n = 0; n < ADSP_NUM_DSP; n++)
		list_for_each_entry_safe(impl, tmp, &adsp->impls[n], list)
			kfree(impl);
	/* the DSPs keep running: they cannot be restarted until reboot */
}

static const struct of_device_id lg1314_adsp_of_match[] = {
	{ .compatible = "lge,lg1314-adsp" },
	{ }
};
MODULE_DEVICE_TABLE(of, lg1314_adsp_of_match);

static struct platform_driver lg1314_adsp_driver = {
	.probe = lg1314_adsp_probe,
	.remove = lg1314_adsp_remove,
	.driver = {
		.name = "lg1314-adsp",
		.of_match_table = lg1314_adsp_of_match,
	},
};
static struct platform_driver * const lg1314_adsp_drivers[] = {
	&adsp_out_driver,
	&lg1314_adsp_driver,
};

static int __init lg1314_adsp_init(void)
{
	return platform_register_drivers(lg1314_adsp_drivers, ARRAY_SIZE(lg1314_adsp_drivers));
}
module_init(lg1314_adsp_init);

static void __exit lg1314_adsp_exit(void)
{
	platform_unregister_drivers(lg1314_adsp_drivers, ARRAY_SIZE(lg1314_adsp_drivers));
}
module_exit(lg1314_adsp_exit);

MODULE_DESCRIPTION("LG1314 audio DSP driver");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE("lg1314/audio_dsp0.bin");
MODULE_FIRMWARE("lg1314/audio_dsp1.bin");
