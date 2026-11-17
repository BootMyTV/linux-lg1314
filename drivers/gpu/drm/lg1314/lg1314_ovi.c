// SPDX-License-Identifier: GPL-2.0
/* LG1314 OVI bring-up: drive the VX1 link to the OLED panel. */

#include <linux/delay.h>
#include "lg1314_drm.h"
#include "lg1314_ovi_tbl.h"

/*
 * lxboot already tuned the display PLL/PHY. Entries bracketed by
 * READ_START/CHECK_END are read back from live hardware here and re-applied
 * verbatim in ovi_apply(), so that tuning is preserved.
 */
static void ovi_read_preserve(struct lg1314_drm *lg, struct lg1314_ovi_reg *tbl,
			      size_t n)
{
	bool reading = false;
	size_t i;

	for (i = 0; i < n; i++) {
		u32 op = tbl[i].addr & LG1314_OVI_OP_MASK;

		if (op == LG1314_OVI_OP_CHECK) {
			if (tbl[i].val == LG1314_OVI_READ_START)
				reading = true;
			else if (tbl[i].val == LG1314_OVI_CHECK_END)
				reading = false;
			continue;
		}
		if (op == LG1314_OVI_OP_WRITE && reading)
			tbl[i].val = lg1314_rd(lg, tbl[i].addr);
	}
}

/* Apply the (now preserved) table. CHECK val >= SET_A0_START gates a
 * rev-specific register set; unused by this table but handled for completeness.
 */
static void ovi_apply(struct lg1314_drm *lg, const struct lg1314_ovi_reg *tbl,
		      size_t n)
{
	bool ignore = false;
	size_t i;

	for (i = 0; i < n; i++) {
		u32 op = tbl[i].addr & LG1314_OVI_OP_MASK;

		switch (op) {
		case LG1314_OVI_OP_WAIT:
			if (!ignore)
				udelay(tbl[i].val);
			break;
		case LG1314_OVI_OP_CHECK:
			if (tbl[i].val >= LG1314_OVI_SET_A0_START)
				ignore = (tbl[i].val != LG1314_OVI_SET_A0_START);
			else if (tbl[i].val == LG1314_OVI_CHECK_END)
				ignore = false;
			break;
		default: /* WRITE */
			if (!ignore)
				lg1314_wr(lg, tbl[i].addr, tbl[i].val);
			break;
		}
	}
}

/* Spread-spectrum + DPE-path glue around the main table (preserved SS values). */
static void ovi_disp_init_extras(struct lg1314_drm *lg)
{
	u32 ss1 = lg1314_rd(lg, LG1314_OVI_SS_REG1);
	u32 ss2 = lg1314_rd(lg, LG1314_OVI_SS_REG2);
	u32 clkgate = lg1314_rd(lg, LG1314_OVI_CLKGATE_REG);

	lg1314_wr(lg, LG1314_OVI_DISP_PLL_REG, 0xfb800000);	/* PLL power-down */
	lg1314_wr(lg, LG1314_OVI_SS_REG1, ss1);
	lg1314_wr(lg, LG1314_OVI_SS_REG2, ss2);
	lg1314_wr(lg, LG1314_OVI_DISP_PLL_REG, 0xff800000);	/* PLL power-up */

	lg1314_wr(lg, LG1314_OVI_DPE_PATH_REG, 0x1);
	lg1314_wr(lg, LG1314_OVI_CLKGATE_REG, clkgate);
}

int lg1314_ovi_enable(struct lg1314_drm *lg)
{
	struct lg1314_ovi_reg *work;
	size_t n = ARRAY_SIZE(lg1314_ovi_4k120_vx1_16lane);

	work = kmemdup(lg1314_ovi_4k120_vx1_16lane,
		       sizeof(lg1314_ovi_4k120_vx1_16lane), GFP_KERNEL);
	if (!work)
		return -ENOMEM;

	ovi_disp_init_extras(lg);
	ovi_read_preserve(lg, work, n);
	ovi_apply(lg, work, n);
	kfree(work);

	/* Channel power on, then release the TCON/WPR/... SW reset. */
	lg1314_wr(lg, LG1314_OVI_CHPOWER_REG, lg->chpower_save);
	lg1314_wr(lg, LG1314_OVI_RESET_REL_REG, 0x0);

	return 0;
}

void lg1314_ovi_disable(struct lg1314_drm *lg)
{
	lg1314_wr(lg, LG1314_OVI_CHPOWER_REG,
		  lg->chpower_save & LG1314_OVI_CHPOWER_OFF_MASK);
}
