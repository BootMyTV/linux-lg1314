// SPDX-License-Identifier: GPL-2.0-only
/*
 * Dolby Vision RPU parsing (header and reshaping mapping) and the display
 * engine's composer register image.
 *
 * The RPU syntax follows dovi_tool's parser (https://github.com/quietvoid/
 * dovi_tool, MIT licence, Copyright (c) quietvoid).
 */
#include <linux/bitops.h>
#include <linux/bits.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>

#include "lg1314-dovi.h"

#define RPU_PREFIX		0x19
#define RPU_FINAL_BYTE		0x80
#define RPU_NAL_HDR0		0x7c	/* nal_unit_type 62, layer 0 */
#define RPU_NAL_HDR1		0x01

struct bitreader {
	const u8	*p;
	size_t		bits, pos;
	bool		err;
};

static u32 br_bits(struct bitreader *b, unsigned int n)
{
	u64 v = 0;

	if (n > 32 || b->pos + n > b->bits) {
		b->err = true;
		return 0;
	}
	while (n--) {
		v = v << 1 | ((b->p[b->pos >> 3] >> (7 - (b->pos & 7))) & 1);
		b->pos++;
	}
	return v;
}

static u32 br_ue(struct bitreader *b)
{
	unsigned int zeros = 0;

	while (!b->err && !br_bits(b, 1))
		if (++zeros > 31)
			b->err = true;
	if (b->err)
		return 0;
	return (u32)((1ULL << zeros) - 1 + br_bits(b, zeros));
}

static s32 br_se(struct bitreader *b)
{
	u32 v = br_ue(b);

	return v & 1 ? (s32)((v + 1) >> 1) : -(s32)(v >> 1);
}

/* CRC-32/MPEG-2: polynomial 0x04c11db7, MSB first, no final xor. */
static u32 rpu_crc32(const u8 *p, size_t len)
{
	u32 crc = 0xffffffff;
	int i;

	while (len--) {
		crc ^= (u32)*p++ << 24;
		for (i = 0; i < 8; i++)
			crc = crc & BIT(31) ? crc << 1 ^ 0x04c11db7 : crc << 1;
	}
	return crc;
}

/* Strip start code / NAL header, remove emulation prevention bytes. */
static size_t rpu_unescape(const u8 *nal, size_t len, u8 *out)
{
	size_t i = 0, n = 0, zeros = 0;

	if (len >= 4 && !nal[0] && !nal[1] && !nal[2] && nal[3] == 1)
		i = 4;
	else if (len >= 3 && !nal[0] && !nal[1] && nal[2] == 1)
		i = 3;
	if (len >= i + 2 && nal[i] == RPU_NAL_HDR0 && nal[i + 1] == RPU_NAL_HDR1)
		i += 2;
	for (; i < len; i++) {
		if (zeros >= 2 && nal[i] == 3) {
			zeros = 0;
			continue;
		}
		zeros = nal[i] ? 0 : zeros + 1;
		out[n++] = nal[i];
	}
	return n;
}

static int rpu_parse_seq(struct dovi_state *st, struct bitreader *b, u16 format)
{
	br_bits(b, 1);				/* chroma_resampling_explicit_filter_flag */
	st->coef_data_type = br_bits(b, 2);
	if (st->coef_data_type != 0)
		return -EOPNOTSUPP;		/* floating point coefficients */
	st->coef_log2_denom = br_ue(b);
	if (st->coef_log2_denom > 32)
		return -EINVAL;
	br_bits(b, 2);				/* vdr_rpu_normalized_idc */
	st->bl_full_range = br_bits(b, 1);
	if (!(format & 0x700)) {
		st->bl_bit_depth = br_ue(b) + 8;
		st->el_bit_depth = (br_ue(b) & 0xff) + 8;	/* upper bits: ext_mapping_idc */
		st->vdr_bit_depth = br_ue(b) + 8;
		br_bits(b, 1);			/* spatial_resampling_filter_flag */
		st->dm_compressed = br_bits(b, 3) == 1;	/* reserved_zero_3bits */
		st->el_spatial_resampling = br_bits(b, 1);
		st->disable_residual = br_bits(b, 1);
	}
	if (st->bl_bit_depth > 16 || st->el_bit_depth > 16 || st->vdr_bit_depth > 16)
		return -EINVAL;
	st->seq_valid = !b->err;
	return b->err ? -EINVAL : 0;
}

static int rpu_parse_poly(struct dovi_state *st, struct bitreader *b,
			  struct dovi_mapping *m, int c, int piece)
{
	u32 order_m1 = br_ue(b);
	int k;

	if (order_m1 > 1)
		return -EINVAL;
	if (!order_m1 && br_bits(b, 1))
		return -EOPNOTSUPP;		/* linear interpolation */
	m->poly_order[c][piece] = order_m1 + 1;
	for (k = 0; k < (int)order_m1 + 2; k++) {
		m->poly_int[c][piece][k] = br_se(b);
		m->poly_frac[c][piece][k] = br_bits(b, st->coef_log2_denom);
	}
	return 0;
}

static int rpu_parse_mmr(struct dovi_state *st, struct bitreader *b,
			 struct dovi_mapping *m, int c, int piece)
{
	u32 order_m1;
	int j, k;

	/* the register image holds one MMR set per chroma component */
	if (!c || piece)
		return -EOPNOTSUPP;
	order_m1 = br_bits(b, 2);
	if (order_m1 > 2)
		return -EINVAL;
	m->mmr_order[c - 1] = order_m1 + 1;
	m->mmr_int[c - 1][0] = br_se(b);
	m->mmr_frac[c - 1][0] = br_bits(b, st->coef_log2_denom);
	for (j = 0; j <= (int)order_m1; j++) {
		for (k = 0; k < 7; k++) {
			m->mmr_int[c - 1][1 + j * 7 + k] = br_se(b);
			m->mmr_frac[c - 1][1 + j * 7 + k] =
				br_bits(b, st->coef_log2_denom);
		}
	}
	return 0;
}

static int rpu_parse_mapping(struct dovi_state *st, struct bitreader *b,
			     struct dovi_mapping *m)
{
	int c, i, ret;

	memset(m, 0, sizeof(*m));
	br_ue(b);				/* mapping_color_space */
	br_ue(b);				/* mapping_chroma_format_idc */
	for (c = 0; c < 3; c++) {
		u32 np = br_ue(b) + 2;
		u32 acc = 0;

		/* luma up to 8 pieces, chroma up to 4 (register image) */
		if (np > (c ? 5u : (u32)DOVI_MAX_PIVOTS))
			return -EOPNOTSUPP;
		m->num_pivots[c] = np;
		for (i = 0; i < (int)np; i++) {
			acc += br_bits(b, st->bl_bit_depth);
			if (acc >= BIT(st->bl_bit_depth))
				return -EINVAL;
			m->pivot[c][i] = acc;
		}
	}
	/* profile 7: the residual's dequantisation, linear dead zone only */
	m->nlq = !(st->rpu_format & 0x700) && !st->disable_residual;
	if (m->nlq) {
		if (br_bits(b, 3) != 0)		/* nlq_method_idc */
			return -EOPNOTSUPP;
		for (i = 0; i < 2; i++)
			m->nlq_pivot[i] = br_bits(b, st->bl_bit_depth);
	}
	br_ue(b);				/* num_x_partitions_minus1 */
	br_ue(b);				/* num_y_partitions_minus1 */
	for (c = 0; c < 3; c++) {
		for (i = 0; i < m->num_pivots[c] - 1; i++) {
			u32 idc = br_ue(b);

			if (b->err)
				return -EINVAL;
			if (i && idc != m->mapping_idc[c])
				return -EOPNOTSUPP;
			m->mapping_idc[c] = idc;
			if (idc == 0)
				ret = rpu_parse_poly(st, b, m, c, i);
			else if (idc == 1)
				ret = rpu_parse_mmr(st, b, m, c, i);
			else
				ret = -EINVAL;
			if (ret)
				return ret;
		}
	}
	if (m->nlq) {
		for (c = 0; c < 3; c++) {
			m->nlq_offset[c] = br_bits(b, st->el_bit_depth);
			m->nlq_in_max_int[c] = br_ue(b);
			m->nlq_in_max[c] = br_bits(b, st->coef_log2_denom);
			m->nlq_slope_int[c] = br_ue(b);
			m->nlq_slope[c] = br_bits(b, st->coef_log2_denom);
			m->nlq_threshold_int[c] = br_ue(b);
			m->nlq_threshold[c] = br_bits(b, st->coef_log2_denom);
		}
	}
	return b->err ? -EINVAL : 0;
}

/* A minimal enhancement layer: its residual dequantises to zero. */
static bool rpu_nlq_is_mel(const struct dovi_mapping *m)
{
	int c;

	for (c = 0; c < 3; c++)
		if (m->nlq_offset[c] || m->nlq_in_max_int[c] != 1 || m->nlq_in_max[c] ||
		    m->nlq_slope_int[c] || m->nlq_slope[c] ||
		    m->nlq_threshold_int[c] || m->nlq_threshold[c])
			return false;
	return true;
}

static void br_align(struct bitreader *b)
{
	if (b->pos & 7)
		br_bits(b, 8 - (b->pos & 7));
}

static void rpu_l8_l10_primaries(struct bitreader *b, u16 *p)
{
	int i;

	for (i = 0; i < 8; i++)
		p[i] = br_bits(b, 16);
}

/* One extension block's payload (@len bytes) into @x. */
static void rpu_ext_block(struct bitreader *b, u8 level, u32 len,
			  struct lg1314_dovi_ext *x)
{
	int i;

	switch (level) {
	case 1:
		x->l1.min_pq = br_bits(b, 12);
		x->l1.max_pq = br_bits(b, 12);
		x->l1.avg_pq = br_bits(b, 12);
		break;
	case 2:
		if (x->num_l2 < LG1314_DOVI_MAX_L2) {
			typeof(x->l2[0]) *t = &x->l2[x->num_l2++];

			t->target_max_pq = br_bits(b, 12);
			t->slope = br_bits(b, 12);
			t->offset = br_bits(b, 12);
			t->power = br_bits(b, 12);
			t->chroma_weight = br_bits(b, 12);
			t->saturation_gain = br_bits(b, 12);
			t->ms_weight = sign_extend32(br_bits(b, 13), 12);
		}
		break;
	case 3:
		x->l3.min_pq_offset = br_bits(b, 12);
		x->l3.max_pq_offset = br_bits(b, 12);
		x->l3.avg_pq_offset = br_bits(b, 12);
		break;
	case 4:
		x->l4.anchor_pq = br_bits(b, 12);
		x->l4.anchor_power = br_bits(b, 12);
		break;
	case 5:
		x->l5.left = br_bits(b, 13);
		x->l5.right = br_bits(b, 13);
		x->l5.top = br_bits(b, 13);
		x->l5.bottom = br_bits(b, 13);
		break;
	case 6:
		x->l6.max_display_mastering_luminance = br_bits(b, 16);
		x->l6.min_display_mastering_luminance = br_bits(b, 16);
		x->l6.max_cll = br_bits(b, 16);
		x->l6.max_fall = br_bits(b, 16);
		break;
	case 8:
		if (x->num_l8 < LG1314_DOVI_MAX_L8) {
			typeof(x->l8[0]) *t = &x->l8[x->num_l8++];

			t->len = len;
			t->target_display_index = br_bits(b, 8);
			t->slope = br_bits(b, 12);
			t->offset = br_bits(b, 12);
			t->power = br_bits(b, 12);
			t->chroma_weight = br_bits(b, 12);
			t->saturation_gain = br_bits(b, 12);
			t->ms_weight = br_bits(b, 12);
			if (len > 10)
				t->target_mid_contrast = br_bits(b, 12);
			if (len > 12)
				t->clip_trim = br_bits(b, 12);
			if (len > 13)
				for (i = 0; i < 6; i++)
					t->saturation_vector[i] = br_bits(b, 8);
			if (len > 19)
				for (i = 0; i < 6; i++)
					t->hue_vector[i] = br_bits(b, 8);
		}
		break;
	case 9:
		x->l9.len = len;
		x->l9.source_primary_index = br_bits(b, 8);
		if (len > 1)
			rpu_l8_l10_primaries(b, x->l9.primaries);
		break;
	case 10:
		if (x->num_l10 < LG1314_DOVI_MAX_L10) {
			typeof(x->l10[0]) *t = &x->l10[x->num_l10++];

			t->len = len;
			t->target_display_index = br_bits(b, 8);
			t->target_max_pq = br_bits(b, 12);
			t->target_min_pq = br_bits(b, 12);
			t->target_primary_index = br_bits(b, 8);
			if (len > 5)
				rpu_l8_l10_primaries(b, t->primaries);
		}
		break;
	case 11:
		for (i = 0; i < 4; i++)
			x->l11[i] = br_bits(b, 8);
		break;
	case 254:
		x->has_l254 = true;
		for (i = 0; i < 2; i++)
			x->l254[i] = br_bits(b, 8);
		break;
	case 255:
		x->has_l255 = true;
		for (i = 0; i < 6; i++)
			x->l255[i] = br_bits(b, 8);
		break;
	}
	if (level < 32)
		x->levels |= BIT(level);
}

/* One set of display-management extension blocks; returns the block count. */
static int rpu_ext_blocks(struct bitreader *b, struct lg1314_dovi_ext *x)
{
	u32 n = br_ue(b), i;

	if (n > 255)
		return -EINVAL;
	br_align(b);
	for (i = 0; i < n && !b->err; i++) {
		u32 len = br_ue(b);
		u8 level = br_bits(b, 8);
		size_t end;

		if (len > (b->bits - b->pos) / 8) {
			b->err = true;
			break;
		}
		end = b->pos + len * 8;
		rpu_ext_block(b, level, len, x);
		if (b->pos > end) {		/* fields beyond the block */
			b->err = true;
			break;
		}
		b->pos = end;
	}
	return b->err ? -EINVAL : n;
}

/* Display-management metadata: walked to its end (the CRC must follow). */
static int rpu_parse_dm(struct dovi_state *st, struct bitreader *b,
			struct dovi_rpu *rpu)
{
	int n;

	struct lg1314_dovi_ext *x = &st->dm.ext;

	st->dm.affected_dm_metadata_id = br_ue(b);
	st->dm.current_dm_metadata_id = br_ue(b);
	st->dm.scene_refresh_flag = br_ue(b);
	/* a compressed DM leaves the header out: the last one holds */
	if (!st->dm_compressed) {
		struct lg1314_dovi_dm *dm = &st->dm;
		int i;

		for (i = 0; i < 9; i++)
			dm->ycc_to_rgb[i] = (s16)br_bits(b, 16);
		for (i = 0; i < 3; i++)
			dm->ycc_to_rgb_offset[i] = br_bits(b, 32);
		for (i = 0; i < 9; i++)
			dm->rgb_to_lms[i] = (s16)br_bits(b, 16);
		dm->signal_eotf = br_bits(b, 16);
		dm->eotf_param0 = br_bits(b, 16);
		dm->eotf_param1 = br_bits(b, 16);
		dm->eotf_param2 = br_bits(b, 32);
		dm->bit_depth = br_bits(b, 5);
		dm->color_space = br_bits(b, 2);
		dm->chroma_format = br_bits(b, 2);
		dm->full_range = br_bits(b, 2);
		dm->source_min_pq = br_bits(b, 12);
		dm->source_max_pq = br_bits(b, 12);
		dm->source_diagonal = br_bits(b, 10);
		st->dm_valid = !b->err;
	}
	memset(x, 0, sizeof(*x));
	n = rpu_ext_blocks(b, x);
	if (n < 0)
		return n;
	rpu->dm_levels[0] = n;
	if (b->bits - b->pos >= 56) {
		n = rpu_ext_blocks(b, x);
		if (n < 0)
			return n;
		rpu->dm_levels[1] = n;
	}
	br_align(b);
	return b->err ? -EINVAL : 0;
}

int dovi_rpu_parse(struct dovi_state *st, const u8 *nal, size_t len,
		   u8 *scratch, struct dovi_rpu *rpu)
{
	struct bitreader b;
	size_t n = rpu_unescape(nal, len, scratch);
	u32 crc, id;
	u16 format;
	bool seq, use_prev;
	int ret;

	while (n && !scratch[n - 1])
		n--;
	if (n < 7 || scratch[0] != RPU_PREFIX || scratch[n - 1] != RPU_FINAL_BYTE)
		return -EINVAL;
	crc = (u32)scratch[n - 5] << 24 | scratch[n - 4] << 16 |
	      scratch[n - 3] << 8 | scratch[n - 2];
	if (rpu_crc32(scratch + 1, n - 6) != crc)
		return -EBADMSG;

	b = (struct bitreader){ .p = scratch, .bits = (n - 5) * 8, .pos = 8 };
	if (br_bits(&b, 6) != 2)		/* rpu_type */
		return -EINVAL;
	format = br_bits(&b, 11);
	st->vdr_rpu_profile = br_bits(&b, 4);
	br_bits(&b, 4);				/* vdr_rpu_level */
	seq = br_bits(&b, 1);
	if (seq) {
		ret = rpu_parse_seq(st, &b, format);
		if (ret)
			return ret;
		st->rpu_format = format;
	}
	if (!st->seq_valid)
		return -ENODATA;

	memset(rpu, 0, sizeof(*rpu));
	rpu->dm_present = br_bits(&b, 1);
	use_prev = br_bits(&b, 1);
	if (!st->vdr_rpu_profile && st->bl_full_range)
		rpu->profile = 5;
	else if (st->vdr_rpu_profile == 1)
		rpu->profile = st->el_spatial_resampling && !st->disable_residual ?
			       (st->vdr_bit_depth == 12 ? 7 : 4) : 8;
	/* with an enhancement layer: profile 7 only */
	if (!rpu->profile || (!st->disable_residual && rpu->profile != 7))
		return -EOPNOTSUPP;

	if (use_prev) {
		id = br_ue(&b);
		if (id >= DOVI_NUM_RPU_IDS || !(st->valid_ids & BIT(id)))
			return -ENODATA;
	} else {
		id = br_ue(&b);
		if (id >= DOVI_NUM_RPU_IDS)
			return -EINVAL;
		ret = rpu_parse_mapping(st, &b, &st->map[id]);
		if (ret) {
			st->valid_ids &= ~BIT(id);
			return ret;
		}
		st->valid_ids |= BIT(id);
	}
	if (rpu->dm_present) {
		ret = rpu_parse_dm(st, &b, rpu);
		if (ret)
			return ret;
	}
	br_align(&b);
	/* the payload must end where the CRC starts */
	if (b.err || b.pos != b.bits)
		return -EPROTO;

	rpu->vdr_bit_depth = st->vdr_bit_depth;
	rpu->bl_bit_depth = st->bl_bit_depth;
	rpu->el_bit_depth = st->el_bit_depth;
	rpu->coef_log2_denom = st->coef_log2_denom;
	rpu->disable_residual = st->disable_residual;
	rpu->map = &st->map[id];
	rpu->el_mel = rpu->map->nlq && rpu_nlq_is_mel(rpu->map);
	rpu->dm = st->dm_valid ? &st->dm : NULL;
	return 0;
}

static u32 coef30(s32 i, u32 f, u8 denom)
{
	return ((u64)(s64)i << denom | f) & 0x3fffffff;
}

/*
 * Register image: per component the pivots (10 bits, two per word), the
 * pieces' polynomial orders (2 bits each), the polynomial coefficients
 * (int << denom | frac, 30 bits), for chroma the MMR coefficients (40 bits as
 * a low word and a high byte) and order, and residual (NLQ) parameters, which
 * a single-layer stream leaves zero.
 */
void dovi_composer_regs(const struct dovi_rpu *rpu, u32 width, u32 height,
			u32 regs[DOVI_COMP_WORDS])
{
	static const struct {
		u8 idc, npiv, piv, ord, poly, pieces, mmr, mmr_ord, nlq;
	} lay[3] = {
		{ 0,   4,   5,   10,  11,  8, 0,   0,   35  },	/* Y */
		{ 39,  40,  41,  44,  45,  4, 57,  101, 102 },	/* U */
		{ 106, 107, 108, 111, 112, 4, 124, 168, 169 },	/* V */
	};
	const struct dovi_mapping *m = rpu->map;
	u8 d = rpu->coef_log2_denom;
	int c, i, k;

	memset(regs, 0, DOVI_COMP_WORDS * sizeof(u32));
	/*
	 * No residual unless the picture comes with its enhancement layer's
	 * (the decoder clears it then): profile 7 otherwise composes its base
	 * layer alone (exact for a minimal EL).
	 */
	regs[0] = 1;
	regs[1] = width | height << 13;
	regs[2] = rpu->bl_bit_depth | rpu->el_bit_depth << 4 |
		  rpu->vdr_bit_depth << 8;
	regs[3] = d;

	for (c = 0; c < 3; c++) {
		/*
		 * The residual's dequantisation: offset, then (int << denom |
		 * frac, 24 bits) the dead zone's slope, vdr_in_max, the dead
		 * zone's threshold. Used once word 0 says so (an EL picture).
		 */
		if (m->nlq) {
			regs[lay[c].nlq] = m->nlq_offset[c];
			regs[lay[c].nlq + 1] = ((u64)m->nlq_slope_int[c] << d |
						m->nlq_slope[c]) & 0xffffff;
			regs[lay[c].nlq + 2] = ((u64)m->nlq_in_max_int[c] << d |
						m->nlq_in_max[c]) & 0xffffff;
			regs[lay[c].nlq + 3] = ((u64)m->nlq_threshold_int[c] << d |
						m->nlq_threshold[c]) & 0xffffff;
		}
		if (c)
			regs[lay[c].idc] = m->mapping_idc[c];
		regs[lay[c].npiv] = m->num_pivots[c];
		for (i = 0; i < m->num_pivots[c]; i++)
			regs[lay[c].piv + i / 2] |= (u32)m->pivot[c][i] << (10 * (i & 1));
		for (i = 0; i < lay[c].pieces; i++) {
			regs[lay[c].ord] |= (u32)m->poly_order[c][i] << (2 * i);
			for (k = 0; k < 3; k++)
				regs[lay[c].poly + 3 * i + k] =
					coef30(m->poly_int[c][i][k],
					       m->poly_frac[c][i][k], d);
		}
		if (!c)
			continue;
		for (k = 0; k < DOVI_MMR_COEFS; k++) {
			u64 v = (u64)(s64)m->mmr_int[c - 1][k] << d |
				m->mmr_frac[c - 1][k];

			regs[lay[c].mmr + 2 * k] = (u32)v;
			regs[lay[c].mmr + 2 * k + 1] = (v >> 32) & 0xff;
		}
		regs[lay[c].mmr_ord] = m->mmr_order[c - 1];
	}
}

/*
 * HDR10+ (SMPTE ST 2094-40) in an HEVC prefix SEI NAL unit (from its start
 * code): registered user data, country 0xB5, provider 0x003C, provider code
 * 0x0001, application 4. The first window's parameters into @md; 0, or
 * -ENOENT if the NAL unit carries none. @scratch holds @len bytes.
 */
static int hdr10plus_t35(const u8 *p, size_t n, struct lg1314_hdr10plus *md)
{
	struct bitreader b;
	u32 nwin, w, i, k, v;

	if (n < 7 || p[0] != 0xb5 || p[1] || p[2] != 0x3c || p[3] || p[4] != 1 ||
	    p[5] != 4)
		return -ENOENT;
	b = (struct bitreader){ .p = p, .bits = n * 8, .pos = 7 * 8 };
	memset(md, 0, sizeof(*md));
	nwin = br_bits(&b, 2);
	if (!nwin)
		return -EINVAL;
	for (w = 1; w < nwin; w++) {
		for (i = 0; i < 6; i++)
			br_bits(&b, 16);	/* window corners, ellipse centre */
		br_bits(&b, 8);			/* rotation_angle */
		for (i = 0; i < 3; i++)
			br_bits(&b, 16);	/* ellipse axes */
		br_bits(&b, 1);			/* overlap_process_option */
	}
	md->target_nits = br_bits(&b, 27);
	if (br_bits(&b, 1)) {			/* targeted display's actual peaks */
		v = br_bits(&b, 5) * br_bits(&b, 5);
		for (i = 0; i < v; i++)
			br_bits(&b, 4);
	}
	for (w = 0; w < nwin; w++) {
		for (i = 0; i < 3; i++) {
			v = br_bits(&b, 17);
			if (!w)
				md->maxscl[i] = v;
		}
		v = br_bits(&b, 17);
		if (!w)
			md->avg_maxrgb = v;
		k = br_bits(&b, 4);		/* distribution percentiles */
		for (i = 0; i < k; i++) {
			br_bits(&b, 7);
			v = br_bits(&b, 17);
			if (!w && i == 8)
				md->hm = v;
		}
		br_bits(&b, 10);		/* fraction_bright_pixels */
	}
	if (br_bits(&b, 1)) {			/* mastering display's actual peaks */
		v = br_bits(&b, 5) * br_bits(&b, 5);
		for (i = 0; i < v; i++)
			br_bits(&b, 4);
	}
	for (w = 0; w < nwin; w++) {
		if (br_bits(&b, 1)) {		/* tone_mapping_flag */
			u16 kx = br_bits(&b, 12), ky = br_bits(&b, 12);

			k = br_bits(&b, 4);
			if (!w) {
				md->knee_x = kx;
				md->knee_y = ky;
				md->num_anchors = k;
			}
			for (i = 0; i < k; i++) {
				v = br_bits(&b, 10);
				if (!w)
					md->anchors[i] = v;
			}
		}
		if (br_bits(&b, 1))		/* color_saturation_mapping_flag */
			br_bits(&b, 6);
	}
	return b.err ? -EINVAL : 0;
}

int hdr10plus_parse_sei(const u8 *nal, size_t len, u8 *scratch,
			struct lg1314_hdr10plus *md)
{
	size_t i = 0, n = 0, zeros = 0, size;
	u32 type;

	if (len >= 4 && !nal[0] && !nal[1] && !nal[2] && nal[3] == 1)
		i = 4;
	else if (len >= 3 && !nal[0] && !nal[1] && nal[2] == 1)
		i = 3;
	if (len < i + 2 || (nal[i] >> 1 & 0x3f) != 39)
		return -ENOENT;
	for (i += 2; i < len; i++) {
		if (zeros >= 2 && nal[i] == 3) {
			zeros = 0;
			continue;
		}
		zeros = nal[i] ? 0 : zeros + 1;
		scratch[n++] = nal[i];
	}
	/* sei_message()s up to the trailing bits */
	for (i = 0; i + 1 < n;) {
		type = size = 0;
		while (i < n && scratch[i] == 0xff)
			type += scratch[i++];
		if (i >= n)
			break;
		type += scratch[i++];
		while (i < n && scratch[i] == 0xff)
			size += scratch[i++];
		if (i >= n)
			break;
		size += scratch[i++];
		if (size > n - i)
			break;
		if (type == 4 && !hdr10plus_t35(scratch + i, size, md))
			return 0;
		i += size;
	}
	return -ENOENT;
}
