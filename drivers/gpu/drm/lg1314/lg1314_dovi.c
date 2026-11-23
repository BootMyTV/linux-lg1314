// SPDX-License-Identifier: GPL-2.0
/*
 * LG1314 Dolby Vision display: the display engine's Dolby block composes and
 * maps each picture from a record the host provides.
 *
 * Records live in a ring in the "de-hdr" memory the DE firmware is given:
 * a header page, then DOVI_SLOTS slots. For each picture shown, the firmware
 * raises a DE IPC interrupt with the picture's PTS (90 kHz) in the header and
 * the host answers with the slot holding that picture's record.
 *
 * The record's display management part (lg1314_dovi_dm.c) maps the stream to
 * the panel in its Cinema picture mode, in floating point. Its tables (tone
 * curve, 3D LUT) change with the scene: computed ahead from the metadata the
 * decoder parses as it is fed, by a worker, and kept for the last few scenes;
 * a picture whose scene isn't ready has them computed as it is queued.
 */

#include <linux/fpu.h>
#include <linux/io.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/soc/lg/lg1314-vdec-vo.h>

#include <drm/drm_print.h>

#include "lg1314_drm.h"
#include "lg1314_dovi_dm.h"

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
#define DOVI_REC_COMP		0x6f10
#define DOVI_REC_TAIL		0x71c4
#define DOVI_REC_SIZE		0x71e0

/*
 * The DM's input stage, from the RPU's DM header: the signal's range, YCC to
 * RGB (or L'M'S') matrix and offsets, whether it is PQ, RGB to the DM's LMS
 * and (static) L'M'S' to ICtCp. Matrices are 9 x s16 row major, two to a word, the last word holding
 * the ninth coefficient and the fraction bits in its top half.
 */
#define DM_RANGE		0x08	/* min << 16 | span */
#define DM_RANGE_INV		0x0c	/* 2^32 / (span << (16 - bit depth)) */
#define DM_YCC_TO_RGB		0x1c
#define DM_YCC_TO_RGB_OFF	0x30	/* 3 x s32: matrix x offsets, in codes */
#define DM_SIGNAL_PQ		0x3c
#define DM_RGB_TO_LMS		0x4c
#define DM_LMS_TO_ICTCP		0x60

#define DM_YCC_TO_RGB_FRAC	13
#define DM_RGB_TO_LMS_FRAC	14	/* the RPU's */

/* The DM's LMS, crosstalk in Q15: DM4's BT.2100 ICtCp 4 %, DM3's IPTPQc2 2 %
 * (as stock rounds it). */
#define DM4_CROSSTALK		1311
#define DM3_CROSSTALK		654

/* L'M'S' to ICtCp (BT.2100, DM4) and to IPT (DM3), Q12. */
static const s16 dm_lms_to_ictcp[9] = {
	 2048,   2048,     0,
	 3305,  -6806,  3501,
	17933, -17390,  -543,
};

static const s16 dm_lms_to_ipt[9] = {
	 1638,   1638,   819,
	18248, -19870,  1622,
	 3300,   1463, -4763,
};

/*
 * The target: the panel in its Cinema picture mode (19Y_C2_XXB9's), 0.001 to
 * 600 cd/m², its primaries; its signal: gamma 2.2, RGB to Y'CbCr (Q16).
 */
static const struct dm_target dovi_target = {
	.min_pq		= 26,
	.max_pq		= 2852,
	/*		    Rx     Ry     Gx     Gy     Bx     By     Wx     Wy */
	.primaries	= { 33945, 16055, 12600, 34235,  7165,  2385, 15635, 16450 },
	.dm3_min_pq	= 26,
	.dm3_max_pq	= 2851,
	.dm3_lut_min	= 207,
	.dm3_lut_max	= 22815,
};

static const struct dm_output dovi_output = {
	.gamma		= 9011,
	.rgb2ycc	= {  11922,  40107,  4049,
			    -6573, -22106, 28679,
			    28679, -26052, -2627 },
	.ycc_offset	= { 4096, 32768, 32768 },
};

#define DOVI_SCENES		4	/* scenes' tables kept */
#define DOVI_PENDING		8	/* parsed metadata the worker hasn't seen */

/* A scene: the DM that maps it and its inputs (what the tables depend on). */
struct dovi_scene {
	bool			dm3;
	struct dm_tc_in		dm4;
	struct dm3_in		dm3in;
};

/* A scene's tables, in the record's form. */
struct dovi_tables {
	struct dovi_scene	in;		/* the scene */
	u64			used;		/* LRU stamp, 0: empty */
	u32			curve[DM_REC_CURVE_WORDS];
	u32			lut[DM_REC_LUT_WORDS];
};

struct lg1314_dovi_dm_state {
	/* The record's DM part as last queued, and the scene it is for. */
	u32			rec[(DOVI_REC_COMP - DOVI_REC_DM) / 4];
	struct dovi_scene	in;
	bool			valid;
	u16			scratch[DM_LUT_DIM][DM_LUT_DIM][3];

	/* Computed ahead: scenes' tables (lock), the metadata to look at
	 * (pending_lock), the worker's own copy and tables. */
	struct mutex		lock;
	struct dovi_tables	scenes[DOVI_SCENES];
	u64			stamp;
	spinlock_t		pending_lock;
	struct lg1314_dovi_dm	pending[DOVI_PENDING];
	unsigned int		pending_first, pending_n;
	struct work_struct	work;
	struct lg1314_dovi_dm	work_dm;
	struct dovi_tables	work_tables;
	u16			work_scratch[DM_LUT_DIM][DM_LUT_DIM][3];
	struct lg1314_vdec	*vdec;		/* the decoder telling us */

	/* per stream: scenes ready when queued / computed then; the slowest;
	 * the DM last used (-1: none yet), pictures mapped by each */
	unsigned int		ready, late;
	u64			max_ns;
	int			dm;
	unsigned int		pics[2];	/* DM4, DM3 */
};

#define DM_WORD(off)		(((off) - DOVI_REC_DM) / 4)

#define DE_CMD_SET_HDR_MODE	0x61
#define DE_HDR_MODE_OFF		0
#define DE_HDR_MODE_DOLBY	1
#define DE_HDR_MODE_HDR10	2

#define DE_IPC_EXT_STATUS	0x10
#define DE_IPC_EXT_CLEAR	0x14
#define DE_IPC_EXT_DOVI		BIT(22)

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
int lg1314_dovi_start(struct lg1314_drm *lg, unsigned int profile)
{
	unsigned int i;
	int ret;

	if (!lg->dovi.ring)
		return -ENODEV;
	lg->dovi.dm->valid = false;
	lg->dovi.dm->ready = 0;
	lg->dovi.dm->late = 0;
	lg->dovi.dm->max_ns = 0;
	lg->dovi.dm->dm = -1;
	lg->dovi.dm->pics[0] = 0;
	lg->dovi.dm->pics[1] = 0;

	memset_io(lg->dovi.ring, 0, DOVI_HDR_SIZE);
	for (i = 0; i < DOVI_SLOTS; i++) {
		writel(0xff, dovi_slot(lg, i) + DOVI_REC_PTS);
		writel(0, dovi_slot(lg, i) + DOVI_REC_PTS + 4);
	}
	lg->dovi.next = 0;
	lg->dovi.misses = 0;
	wmb();
	WRITE_ONCE(lg->dovi.active, true);
	memset(&lg->dovi.status, 0, sizeof(lg->dovi.status));
	lg->dovi.status.profile = profile;
	ret = dovi_set_mode(lg, DE_HDR_MODE_DOLBY);
	if (ret)
		WRITE_ONCE(lg->dovi.active, false);
	else
		dev_info(lg->dev, "Dolby Vision: on\n");
	lg1314_status_update(lg);
	return ret;
}

/* The stream ends: the DE back to the HDR mode the HDR10 side has set. */
void lg1314_dovi_stop(struct lg1314_drm *lg)
{
	if (!READ_ONCE(lg->dovi.active))
		return;
	WRITE_ONCE(lg->dovi.active, false);
	dovi_set_mode(lg, lg->hdr.hdr10 ? DE_HDR_MODE_HDR10 : DE_HDR_MODE_OFF);
	dev_info(lg->dev, "Dolby Vision: off (%u requests without a record; pictures: %u DM4, %u DM3; scenes: %u ready, %u computed when queued, slowest %llu us)\n",
		 lg->dovi.misses, lg->dovi.dm->pics[0], lg->dovi.dm->pics[1],
		 lg->dovi.dm->ready, lg->dovi.dm->late,
		 div_u64(lg->dovi.dm->max_ns, 1000));
	lg1314_status_update(lg);
}

/* PTS in the DE's units (90 kHz), as the video output converts it. */
u32 lg1314_dovi_pts90(u64 ns)
{
	return div_u64(div_u64(ns, 10) * 9, 10000);
}

static void dm_put_m33(u32 *dm, unsigned int off, const s16 *m, unsigned int frac)
{
	u32 *w = &dm[DM_WORD(off)];
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
static unsigned int dm_rgb_to_lms(const s16 *r, s64 c, s16 *out)
{
	const s64 d = BIT(15) - 3 * c;
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

/* The DM's input stage from the RPU's DM header (dm: the record's DM words),
 * for DM3 (IPT) or DM4 (ICtCp). */
static void dovi_dm_input(const struct lg1314_dovi_dm *m, bool dm3, u32 *dm)
{
	const s16 *y = m->ycc_to_rgb;
	u32 min, span;
	s16 lms[9];
	int i;

	if (m->bit_depth < 10 || m->bit_depth > 16)
		return;
	dm_range(m, &min, &span);
	dm[DM_WORD(DM_RANGE)] = min << 16 | span;
	dm[DM_WORD(DM_RANGE_INV)] =
		div_u64(U32_MAX, span << (16 - m->bit_depth));
	dm_put_m33(dm, DM_YCC_TO_RGB, y, DM_YCC_TO_RGB_FRAC);
	/* offsets: Q28 x Q13 >> 16, codes of a 12-bit signal in Q13 */
	for (i = 0; i < 3; i++)
		dm[DM_WORD(DM_YCC_TO_RGB_OFF) + i] =
			(u32)(((s64)y[3 * i] * m->ycc_to_rgb_offset[0] +
			       (s64)y[3 * i + 1] * m->ycc_to_rgb_offset[1] +
			       (s64)y[3 * i + 2] * m->ycc_to_rgb_offset[2]) >> 16);
	dm[DM_WORD(DM_SIGNAL_PQ)] = m->signal_eotf == LG1314_DOVI_EOTF_PQ;
	dm_put_m33(dm, DM_RGB_TO_LMS, lms,
		   dm_rgb_to_lms(m->rgb_to_lms, dm3 ? DM3_CROSSTALK : DM4_CROSSTALK, lms));
	dm_put_m33(dm, DM_LMS_TO_ICTCP, dm3 ? dm_lms_to_ipt : dm_lms_to_ictcp, 12);
}

/*
 * Which DM maps a picture, as stock chooses in the Cinema mode: DM4 for CM
 * v4.0 metadata (level 254, version above 1), DM3 for CM v2.9.
 */
static bool dovi_dm3(const struct lg1314_dovi_dm *m)
{
	return !(m->ext.has_l254 && m->ext.l254[1] > 1);
}

/* The scene's inputs and the registers for DM metadata @m. FPU section. */
static void dovi_dm_scene(const struct lg1314_dovi_dm *m, struct dovi_scene *in,
			  u32 *regs)
{
	struct dm_trims trims;
	s32 ms;

	memset(in, 0, sizeof(*in));
	in->dm3 = dovi_dm3(m);
	if (in->dm3) {
		dm3_scene(m, &dovi_target, &in->dm3in, &ms);
	} else {
		dm_tone_curve_in(m, &dovi_target, &in->dm4, &trims);
		ms = trims.ms_weight;
	}
	if (regs)
		dm_rec_regs(&dovi_target, in->dm3, ms, &m->ext, regs);
}

/* A scene's tables, one LUT layer per FPU section to keep them short. */
static void dovi_dm_tables(const struct dovi_scene *in, u32 *curve, u32 *lut,
			   u16 scratch[DM_LUT_DIM][DM_LUT_DIM][3])
{
	unsigned int k;

	kernel_fpu_begin();
	if (in->dm3)
		dm3_rec_curve(&in->dm3in, curve);
	else
		dm_rec_curve(&in->dm4, curve);
	kernel_fpu_end();
	memset(lut, 0, DM_REC_LUT_WORDS * 4);
	for (k = 0; k < DM_LUT_DIM; k++) {
		kernel_fpu_begin();
		if (in->dm3)
			dm3_rec_lut_slice(&in->dm3in, &dovi_output, k, scratch, lut);
		else
			dm_rec_lut_slice(&in->dm4, &dovi_output, k, scratch, lut);
		kernel_fpu_end();
	}
}

/* A kept scene's tables, marked used; under d->lock. */
static struct dovi_tables *dovi_dm_find(struct lg1314_dovi_dm_state *d,
					const struct dovi_scene *in)
{
	unsigned int i;

	for (i = 0; i < DOVI_SCENES; i++) {
		struct dovi_tables *t = &d->scenes[i];

		if (t->used && !memcmp(&t->in, in, sizeof(*in))) {
			t->used = ++d->stamp;
			return t;
		}
	}
	return NULL;
}

/* The worker: tables for the scenes of parsed metadata, if not kept yet. */
static void dovi_dm_work(struct work_struct *work)
{
	struct lg1314_dovi_dm_state *d =
		container_of(work, struct lg1314_dovi_dm_state, work);
	struct dovi_tables *t = &d->work_tables, *lru;
	unsigned int i;

	for (;;) {
		spin_lock(&d->pending_lock);
		if (!d->pending_n) {
			spin_unlock(&d->pending_lock);
			return;
		}
		d->work_dm = d->pending[d->pending_first];
		d->pending_first = (d->pending_first + 1) % DOVI_PENDING;
		d->pending_n--;
		spin_unlock(&d->pending_lock);

		kernel_fpu_begin();
		dovi_dm_scene(&d->work_dm, &t->in, NULL);
		kernel_fpu_end();
		mutex_lock(&d->lock);
		lru = dovi_dm_find(d, &t->in);
		mutex_unlock(&d->lock);
		if (lru)
			continue;

		dovi_dm_tables(&t->in, t->curve, t->lut, d->work_scratch);

		mutex_lock(&d->lock);
		lru = &d->scenes[0];
		for (i = 1; i < DOVI_SCENES; i++)
			if (d->scenes[i].used < lru->used)
				lru = &d->scenes[i];
		memcpy(lru, t, sizeof(*t));
		lru->used = ++d->stamp;
		mutex_unlock(&d->lock);
	}
}

/* The decoder parsed an access unit's DM metadata (spinlock held). */
static void dovi_dm_parsed(void *data, const struct lg1314_dovi_dm *m)
{
	struct lg1314_dovi_dm_state *d = data;
	unsigned int i;

	spin_lock(&d->pending_lock);
	if (d->pending_n == DOVI_PENDING) {		/* behind: drop the oldest */
		d->pending_first = (d->pending_first + 1) % DOVI_PENDING;
		d->pending_n--;
	}
	i = (d->pending_first + d->pending_n++) % DOVI_PENDING;
	d->pending[i] = *m;
	spin_unlock(&d->pending_lock);
	queue_work(system_unbound_wq, &d->work);
}

/* The decoder the video plane uses: its metadata, from now on. */
void lg1314_dovi_attach(struct lg1314_drm *lg, struct lg1314_vdec *vdec)
{
	struct lg1314_dovi_dm_state *d = lg->dovi.dm;

	if (!d || d->vdec)
		return;
	d->vdec = vdec;
	lg1314_vdec_set_dovi_listener(vdec, dovi_dm_parsed, d);
}

void lg1314_dovi_detach(struct lg1314_drm *lg)
{
	struct lg1314_dovi_dm_state *d = lg->dovi.dm;

	if (!d || !d->vdec)
		return;
	lg1314_vdec_set_dovi_listener(d->vdec, NULL, NULL);
	cancel_work_sync(&d->work);
	d->vdec = NULL;
}

/*
 * A picture's DM metadata, as it is queued: the input stage and registers
 * each time; the tables when the scene changes, kept ones if ready.
 */
static void dovi_dm_update(struct lg1314_drm *lg, const struct lg1314_dovi_dm *m)
{
	struct lg1314_dovi_dm_state *d = lg->dovi.dm;
	struct lg1314_dovi_status st;
	struct dovi_tables *t;
	struct dovi_scene in;
	u64 t0;

	dovi_dm_input(m, dovi_dm3(m), d->rec);
	st = lg->dovi.status;
	kernel_fpu_begin();
	dovi_dm_scene(m, &in, &d->rec[DM_WORD(DM_REC_REGS)]);
	st.src_min = dm_pq_lum(m->source_min_pq);
	st.src_max = dm_pq_lum(m->source_max_pq);
	/* the panel (DM3 rounds its range a code lower: same display) */
	st.tgt_min = dm_pq_lum(dovi_target.min_pq);
	st.tgt_max = dm_pq_lum(dovi_target.max_pq);
	kernel_fpu_end();
	/* the mastering display as L6 has it (exact), else the DM header's */
	if ((m->ext.levels & BIT(6)) && m->ext.l6.max_display_mastering_luminance) {
		st.src_min = m->ext.l6.min_display_mastering_luminance;
		st.src_max = m->ext.l6.max_display_mastering_luminance * 10000;
	}
	st.dm = in.dm3 ? 3 : 4;
	st.trims = in.dm3 ? m->ext.num_l2 : m->ext.num_l8;
	st.cm_version = in.dm3 ? 0 : m->ext.l254[1];
	if (memcmp(&st, &lg->dovi.status, sizeof(st))) {
		lg->dovi.status = st;
		lg1314_status_update(lg);
	}
	d->pics[in.dm3]++;
	if (d->dm != in.dm3) {
		if (in.dm3)
			dev_info_ratelimited(lg->dev,
					     "Dolby Vision: DM3 (CM v2.9 metadata, %u L2 trims, mastering PQ %u..%u)\n",
					     m->ext.num_l2, m->source_min_pq, m->source_max_pq);
		else
			dev_info_ratelimited(lg->dev,
					     "Dolby Vision: DM4 (CM v4.0 metadata, version %u, %u L8 trims, mastering PQ %u..%u)\n",
					     m->ext.l254[1], m->ext.num_l8, m->source_min_pq,
					     m->source_max_pq);
		d->dm = in.dm3;
	}
	if (d->valid && !memcmp(&in, &d->in, sizeof(in)))
		return;

	mutex_lock(&d->lock);
	t = dovi_dm_find(d, &in);
	if (t) {
		memcpy(&d->rec[DM_WORD(DM_REC_CURVE)], t->curve, sizeof(t->curve));
		memcpy(&d->rec[DM_WORD(DM_REC_LUT)], t->lut, sizeof(t->lut));
		d->ready++;
	}
	mutex_unlock(&d->lock);
	if (!t) {
		t0 = ktime_get_ns();
		dovi_dm_tables(&in, &d->rec[DM_WORD(DM_REC_CURVE)],
			       &d->rec[DM_WORD(DM_REC_LUT)], d->scratch);
		d->late++;
		d->max_ns = max(d->max_ns, ktime_get_ns() - t0);
	}
	memcpy(&d->in, &in, sizeof(in));
	d->valid = true;
}

/* A picture about to be queued: its record into the next slot. */
void lg1314_dovi_queue(struct lg1314_drm *lg, u32 pts90,
		       const struct lg1314_dovi_pic *pic)
{
	struct lg1314_dovi_dm_state *d = lg->dovi.dm;
	u8 __iomem *rec = dovi_slot(lg, lg->dovi.next);

	/* a picture without DM metadata keeps the last; none yet: no record */
	if (pic->dm_valid)
		dovi_dm_update(lg, &pic->dm);
	if (!d->valid)
		return;

	lg->dovi.next = (lg->dovi.next + 1) % DOVI_SLOTS;
	writel(0xff, rec + DOVI_REC_PTS);
	wmb();
	memcpy_toio(rec + DOVI_REC_DM, d->rec, sizeof(d->rec));
	memcpy_toio(rec + DOVI_REC_COMP, pic->comp, sizeof(pic->comp));
	memset_io(rec + DOVI_REC_TAIL, 0, DOVI_REC_SIZE - DOVI_REC_TAIL);
	wmb();
	writel(0, rec + DOVI_REC_PTS + 4);
	writel(pts90, rec + DOVI_REC_PTS);
}

static void dovi_dm_free(void *d)
{
	kvfree(d);
}

int lg1314_dovi_init(struct lg1314_drm *lg, int irq)
{
	struct resource *r = &lg->de_mem.hdr;
	struct lg1314_dovi_dm_state *d;
	int ret;

	if (resource_size(r) < DOVI_HDR_SIZE + DOVI_SLOTS * DOVI_SLOT_SIZE) {
		dev_info(lg->dev, "de-hdr too small: no Dolby Vision\n");
		return 0;
	}
	if (!kernel_fpu_available()) {
		dev_info(lg->dev, "no FPU: no Dolby Vision\n");
		return 0;
	}
	d = kvzalloc(sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;
	ret = devm_add_action_or_reset(lg->dev, dovi_dm_free, d);
	if (ret)
		return ret;
	mutex_init(&d->lock);
	spin_lock_init(&d->pending_lock);
	INIT_WORK(&d->work, dovi_dm_work);
	lg->dovi.dm = d;
	lg->dovi.ring = devm_ioremap_wc(lg->dev, r->start, resource_size(r));
	if (!lg->dovi.ring)
		return -ENOMEM;
	return devm_request_irq(lg->dev, irq, lg1314_dovi_irq, 0,
				"lg1314-de-ipc", lg);
}
