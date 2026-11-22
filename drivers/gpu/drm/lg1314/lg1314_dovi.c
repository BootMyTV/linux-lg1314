// SPDX-License-Identifier: GPL-2.0
/*
 * LG1314 Dolby Vision display: the display engine's Dolby block composes and
 * maps each picture from a record the host provides.
 *
 * Records live in a ring in the "de-hdr" memory the DE firmware is given:
 * a header page, then DOVI_SLOTS slots. For each picture shown, the firmware
 * raises a DE IPC interrupt with the picture's PTS (90 kHz) in the header and
 * the host answers with the slot holding that picture's record.
 */

#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/soc/lg/lg1314-vdec-vo.h>

#include <drm/drm_print.h>

#include "lg1314_drm.h"

#define DOVI_FW_NAME		"lg1314/dovi_deleteme.bin"
#define DOVI_FW_MAGIC		0x5644474c	/* "LGDV" */

#define DOVI_HDR_SIZE		0x1000
#define DOVI_SLOT_SIZE		0x8000
#define DOVI_SLOTS		95
#define DOVI_HDR_PTS		0x0
#define DOVI_HDR_STATE		0x8
#define DOVI_HDR_SLOT		0xc
#define DOVI_STATE_REQUEST	2	/* the firmware wants a record */
#define DOVI_STATE_TAKEN	1
#define DOVI_STATE_READY	3

/* A record: PTS, DM registers and LUTs, composer registers, active area. */
#define DOVI_REC_PTS		0x0
#define DOVI_REC_DM		0x8
#define DOVI_REC_DM_END		0xb0
#define DOVI_REC_COMP		0x6f10
#define DOVI_REC_TAIL		0x71c4
#define DOVI_REC_SIZE		0x71e0

/*
 * The DM's input stage, from the RPU's DM header: the signal's range, YCC to
 * RGB (or L'M'S') matrix and offsets, whether it is PQ, and RGB to the DM's
 * LMS. Matrices are 9 x s16 row major, two to a word, the last word holding
 * the ninth coefficient and the fraction bits in its top half.
 */
#define DM_RANGE		0x08	/* min << 16 | span */
#define DM_RANGE_INV		0x0c	/* 2^32 / (span << (16 - bit depth)) */
#define DM_YCC_TO_RGB		0x1c
#define DM_YCC_TO_RGB_OFF	0x30	/* 3 x s32: matrix x offsets, in codes */
#define DM_SIGNAL_PQ		0x3c
#define DM_RGB_TO_LMS		0x4c

#define DM_YCC_TO_RGB_FRAC	13
#define DM_RGB_TO_LMS_FRAC	14	/* the RPU's */
/* The DM's LMS: BT.2100 ICtCp's, with 4 % crosstalk (Q15). */
#define DM_CROSSTALK		1311

#define DE_CMD_SET_HDR_MODE	0x61
#define DE_HDR_MODE_OFF		0
#define DE_HDR_MODE_DOLBY	1
#define DE_HDR_MODE_HDR10	2

#define DE_IPC_EXT_STATUS	0x10
#define DE_IPC_EXT_CLEAR	0x14
#define DE_IPC_EXT_DOVI		BIT(22)

struct dovi_fw {
	__le32	magic;
	__le32	version;
	u8	record[DOVI_REC_SIZE];	/* template: DM registers and LUTs */
};

static u8 __iomem *dovi_slot(struct lg1314_drm *lg, unsigned int i)
{
	return lg->dovi.ring + DOVI_HDR_SIZE + i * DOVI_SLOT_SIZE;
}

static int dovi_set_mode(struct lg1314_drm *lg, u32 mode)
{
	u32 m[2] = { 0, mode };

	return lg1314_de_cmd(lg, DE_CMD_SET_HDR_MODE, m, sizeof(m));
}

/* The DE IPC interrupt: acknowledge everything, answer record requests. */
static irqreturn_t lg1314_dovi_irq(int irq, void *arg)
{
	struct lg1314_drm *lg = arg;
	u32 st = lg1314_rd(lg, DE_IPC_PHYS + DE_IPC_EXT_STATUS);
	unsigned int i;
	u32 pts;

	if (!st)
		return IRQ_NONE;
	lg1314_wr(lg, DE_IPC_PHYS + DE_IPC_EXT_CLEAR, st);
	if (!(st & DE_IPC_EXT_DOVI) || !READ_ONCE(lg->dovi.active) ||
	    readl(lg->dovi.ring + DOVI_HDR_STATE) != DOVI_STATE_REQUEST)
		return IRQ_HANDLED;

	pts = readl(lg->dovi.ring + DOVI_HDR_PTS);
	writel(DOVI_STATE_TAKEN, lg->dovi.ring + DOVI_HDR_STATE);
	for (i = 0; i < DOVI_SLOTS; i++) {
		if (readl(dovi_slot(lg, i) + DOVI_REC_PTS) == pts &&
		    !readl(dovi_slot(lg, i) + DOVI_REC_PTS + 4)) {
			writel(i, lg->dovi.ring + DOVI_HDR_SLOT);
			writel(DOVI_STATE_READY, lg->dovi.ring + DOVI_HDR_STATE);
			return IRQ_HANDLED;
		}
	}
	lg->dovi.misses++;
	return IRQ_HANDLED;
}

/* A Dolby Vision stream starts: records ready, the DE in its Dolby mode. */
int lg1314_dovi_start(struct lg1314_drm *lg)
{
	const struct firmware *fw = lg->dovi.fw;
	unsigned int i;
	int ret;

	if (!lg->dovi.ring)
		return -ENODEV;
	if (!fw) {
		ret = request_firmware(&fw, DOVI_FW_NAME, lg->dev);
		if (ret)
			return ret;
		if (fw->size != sizeof(struct dovi_fw) ||
		    le32_to_cpu(((const struct dovi_fw *)fw->data)->magic) != DOVI_FW_MAGIC) {
			dev_err(lg->dev, "%s: bad image\n", DOVI_FW_NAME);
			release_firmware(fw);
			return -EINVAL;
		}
		lg->dovi.fw = fw;
	}

	memset_io(lg->dovi.ring, 0, DOVI_HDR_SIZE);
	for (i = 0; i < DOVI_SLOTS; i++) {
		writel(0xff, dovi_slot(lg, i) + DOVI_REC_PTS);
		writel(0, dovi_slot(lg, i) + DOVI_REC_PTS + 4);
	}
	lg->dovi.next = 0;
	lg->dovi.misses = 0;
	wmb();
	WRITE_ONCE(lg->dovi.active, true);
	ret = dovi_set_mode(lg, DE_HDR_MODE_DOLBY);
	if (ret)
		WRITE_ONCE(lg->dovi.active, false);
	else
		dev_info(lg->dev, "Dolby Vision: on\n");
	return ret;
}

/* The stream ends: the DE back to the HDR mode the HDR10 side has set. */
void lg1314_dovi_stop(struct lg1314_drm *lg)
{
	if (!READ_ONCE(lg->dovi.active))
		return;
	WRITE_ONCE(lg->dovi.active, false);
	dovi_set_mode(lg, lg->hdr.hdr10 ? DE_HDR_MODE_HDR10 : DE_HDR_MODE_OFF);
	dev_info(lg->dev, "Dolby Vision: off (%u requests without a record)\n",
		 lg->dovi.misses);
}

/* PTS in the DE's units (90 kHz), as the video output converts it. */
u32 lg1314_dovi_pts90(u64 ns)
{
	return div_u64(div_u64(ns, 10) * 9, 10000);
}

static void dm_put_m33(u32 *dm, unsigned int off, const s16 *m, unsigned int frac)
{
	u32 *w = &dm[(off - DOVI_REC_DM) / 4];
	int i;

	for (i = 0; i < 4; i++)
		w[i] = (u16)m[2 * i] | (u32)(u16)m[2 * i + 1] << 16;
	w[4] = (u16)m[8] | frac << 16;
}

/* The signal's code range: GetRange's modes (narrow, full, SDI). */
static void dm_range(const struct lg1314_dovi_dm *m, u32 *min, u32 *span)
{
	unsigned int bd = m->bit_depth;

	switch (m->full_range) {
	case 0:
		*min = 16 << (bd - 8);
		*span = (235 << (bd - 8)) - *min;
		break;
	case 1:
		*min = 0;
		*span = BIT(bd) - 1;
		break;
	default:
		*min = 1 << (bd - 8);
		*span = (1019 << (bd - 10)) - *min;
		break;
	}
}

/*
 * The DM's RGB to LMS: the crosstalk matrix X(c) = (1 - 3c) I + c J times
 * the stream's RGB to LMS. (Profile 5's signal is L'M'S' already, and its
 * RGB to LMS undoes the 2 % crosstalk it was made with.) Fraction bits as
 * many as an s16 takes, at most 15.
 */
static unsigned int dm_rgb_to_lms(const s16 *r, s16 *out)
{
	const s64 c = DM_CROSSTALK, d = BIT(15) - 3 * DM_CROSSTALK;
	unsigned int frac, sh, i, j;
	s64 m[9], max = 0;

	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++) {
			m[i * 3 + j] = d * r[i * 3 + j] +
				       c * (r[j] + r[3 + j] + r[6 + j]);
			max = max(max, abs(m[i * 3 + j]));
		}
	/* m: Q(15 + DM_RGB_TO_LMS_FRAC) */
	for (frac = 15; frac > 0; frac--) {
		sh = 15 + DM_RGB_TO_LMS_FRAC - frac;
		if ((max + BIT(sh - 1)) >> sh <= S16_MAX)
			break;
	}
	sh = 15 + DM_RGB_TO_LMS_FRAC - frac;
	for (i = 0; i < 9; i++)
		out[i] = (m[i] + BIT(sh - 1)) >> sh;
	return frac;
}

/* The DM's input stage from the RPU's DM header (dm: the record's DM words). */
static void dovi_dm_input(const struct lg1314_dovi_dm *m, u32 *dm)
{
	const s16 *y = m->ycc_to_rgb;
	u32 min, span;
	s16 lms[9];
	int i;

	if (m->bit_depth < 10 || m->bit_depth > 16)
		return;
	dm_range(m, &min, &span);
	dm[(DM_RANGE - DOVI_REC_DM) / 4] = min << 16 | span;
	dm[(DM_RANGE_INV - DOVI_REC_DM) / 4] =
		div_u64(U32_MAX, span << (16 - m->bit_depth));
	dm_put_m33(dm, DM_YCC_TO_RGB, y, DM_YCC_TO_RGB_FRAC);
	/* offsets: Q28 x Q13 >> 16, codes of a 12-bit signal in Q13 */
	for (i = 0; i < 3; i++)
		dm[(DM_YCC_TO_RGB_OFF - DOVI_REC_DM) / 4 + i] =
			(u32)(((s64)y[3 * i] * m->ycc_to_rgb_offset[0] +
			       (s64)y[3 * i + 1] * m->ycc_to_rgb_offset[1] +
			       (s64)y[3 * i + 2] * m->ycc_to_rgb_offset[2]) >> 16);
	dm[(DM_SIGNAL_PQ - DOVI_REC_DM) / 4] = m->signal_eotf == LG1314_DOVI_EOTF_PQ;
	dm_put_m33(dm, DM_RGB_TO_LMS, lms, dm_rgb_to_lms(m->rgb_to_lms, lms));
}

/* A picture about to be queued: its record into the next slot. */
void lg1314_dovi_queue(struct lg1314_drm *lg, u32 pts90,
		       const struct lg1314_dovi_pic *pic)
{
	const struct dovi_fw *fw = (const struct dovi_fw *)lg->dovi.fw->data;
	u8 __iomem *rec = dovi_slot(lg, lg->dovi.next);
	u32 dm[(DOVI_REC_DM_END - DOVI_REC_DM) / 4];

	memcpy(dm, fw->record + DOVI_REC_DM, sizeof(dm));
	if (pic->dm_valid)
		dovi_dm_input(&pic->dm, dm);

	lg->dovi.next = (lg->dovi.next + 1) % DOVI_SLOTS;
	writel(0xff, rec + DOVI_REC_PTS);
	wmb();
	memcpy_toio(rec + DOVI_REC_DM, dm, sizeof(dm));
	memcpy_toio(rec + DOVI_REC_DM_END, fw->record + DOVI_REC_DM_END,
		    DOVI_REC_COMP - DOVI_REC_DM_END);
	memcpy_toio(rec + DOVI_REC_COMP, pic->comp, sizeof(pic->comp));
	memset_io(rec + DOVI_REC_TAIL, 0, DOVI_REC_SIZE - DOVI_REC_TAIL);
	wmb();
	writel(0, rec + DOVI_REC_PTS + 4);
	writel(pts90, rec + DOVI_REC_PTS);
}

int lg1314_dovi_init(struct lg1314_drm *lg, int irq)
{
	struct resource *r = &lg->de_mem.hdr;

	if (resource_size(r) < DOVI_HDR_SIZE + DOVI_SLOTS * DOVI_SLOT_SIZE) {
		dev_info(lg->dev, "de-hdr too small: no Dolby Vision\n");
		return 0;
	}
	lg->dovi.ring = devm_ioremap_wc(lg->dev, r->start, resource_size(r));
	if (!lg->dovi.ring)
		return -ENOMEM;
	return devm_request_irq(lg->dev, irq, lg1314_dovi_irq, 0,
				"lg1314-de-ipc", lg);
}

void lg1314_dovi_fini(struct lg1314_drm *lg)
{
	release_firmware(lg->dovi.fw);
	lg->dovi.fw = NULL;
}
