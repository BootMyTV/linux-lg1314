// SPDX-License-Identifier: GPL-2.0
/*
 * LG1314 (lg1k family) CPU and GPU clock controller.
 *
 * Models two clock domains:
 *
 *	xtal ──▶ cpupll (PLL) ──▶ cpuclk (÷2) ──▶ CPU
 *	xtal ──▶ gpupll (PLL) ──▶ [gate] ──────▶ GPU (Mali-T820)
 *
 * cpupll is a programmable PLL; its three control words live at reg[0] and are
 * XOR-scrambled with the per-SoC "fix" mask from the DT. reg[1] is a one-bit
 * switch that parks the CPU on a stable source while the PLL relocks. cpuclk is
 * a fixed ÷2 and carries CLK_SET_RATE_PARENT so a cpufreq rate request on it is
 * forwarded up to the PLL (the generic fixed-factor binding cannot express that
 * flag, which is why the divider is registered here).
 *
 * gpupll (reg[2]) feeds the Mali GPU. Unlike the CPU PLL it is read through an
 * unscrambled register window. It is modelled read-only and reports the rate
 * the boot loader left the PLL at. The GPU arrives already clocked and out of
 * reset, and poking the gate (gpu_clk_gate_en, 0xC9205000 bit 12) from here was
 * found to stall the GPU mid soft-reset -- that bit gates the clock *off* -- so
 * gating, reset and rate programming are deferred to a dedicated GPU power
 * sequence rather than hidden behind clk enable/set_rate.
 *
 * The remaining SoC clocks (xtal, bus clock) are plain fixed / fixed-factor
 * clocks and stay as independent DT nodes.
 */

#include <linux/bitfield.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>

#include <dt-bindings/clock/lge,lg1314-clk.h>

/* PLL control words (reg bank 0), after XOR with fix[]. */
#define PLL_CTRL0		0x00
#define PLL_CTRL1		0x04
#define PLL_CTRL2		0x08

#define PLL_M			GENMASK(26, 21)	/* in CTRL0: feedback pre-divider */
#define PLL_CIH			GENMASK(31, 28)	/* in CTRL0: VCO band		 */
#define PLL_FOUT_OD		GENMASK(12, 11)	/* in CTRL1: output divider (2^od)*/
#define PLL_NSC			GENMASK(22, 19)	/* in CTRL1: feedback, sub-count	 */
#define PLL_NPC			GENMASK(28, 23)	/* in CTRL1: feedback, main count */

/* Switch register (reg bank 1): park CPU on the stable clock while relocking. */
#define PLL_SWITCH		BIT(12)

/* VCO operating range, MHz. */
#define PLL_VCO_MIN		1200
#define PLL_VCO_MAX		2700

struct lg1k_pll {
	struct clk_hw		hw;
	void __iomem		*ctrl;	/* reg bank 0: control words	*/
	void __iomem		*sw;	/* reg bank 1: relock switch	*/
	u32			fix[3];
};

#define to_lg1k_pll(_hw)	container_of(_hw, struct lg1k_pll, hw)

static void lg1k_pll_read(const struct lg1k_pll *pll, u32 r[3])
{
	r[0] = pll->fix[0] ^ readl_relaxed(pll->ctrl + PLL_CTRL0);
	r[1] = pll->fix[1] ^ readl_relaxed(pll->ctrl + PLL_CTRL1);
	r[2] = pll->fix[2] ^ readl_relaxed(pll->ctrl + PLL_CTRL2);
}

static unsigned long lg1k_pll_recalc_rate(struct clk_hw *hw,
					  unsigned long parent_rate)
{
	const struct lg1k_pll *pll = to_lg1k_pll(hw);
	unsigned long in_mhz = parent_rate / 1000000;
	u32 r[3], npc, nsc, m, od;

	lg1k_pll_read(pll, r);

	npc = FIELD_GET(PLL_NPC, r[1]);
	nsc = FIELD_GET(PLL_NSC, r[1]);
	m   = FIELD_GET(PLL_M, r[0]);
	od  = FIELD_GET(PLL_FOUT_OD, r[1]);

	return (((npc << 2) + nsc) * in_mhz / ((m + 1) << od)) * 1000000;
}

static long lg1k_pll_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *parent_rate)
{
	unsigned long in_mhz = *parent_rate / 1000000;
	unsigned long out_mhz = rate / 1000000;
	unsigned int feedback;
	int div;

	for (div = 3; div >= 0; div--) {
		unsigned long vco = out_mhz << div;

		if (vco < PLL_VCO_MIN || vco > PLL_VCO_MAX)
			continue;

		feedback = (vco + in_mhz / 2) / in_mhz;
		return (long)(((feedback * in_mhz) >> div) * 1000000);
	}

	/* Not roundable; leave the rate unchanged (set_rate will reject it). */
	return rate;
}

/* VCO band thresholds (MHz) -> CIH code, highest first. */
static const struct {
	unsigned int freq;
	u32 cih;
} lg1k_pll_cih[] = {
	{ 2650, 0xc }, { 2480, 0xa }, { 2300, 0x9 }, { 2080, 0x7 },
	{ 1870, 0x6 }, { 1600, 0x4 }, { 1200, 0x3 },
};

/* Compute the CTRL0/CTRL1 fields for freq_out; preserves the other bits. */
static int lg1k_pll_find_ctrl(u32 r[3], unsigned long rate,
			      unsigned long parent_rate)
{
	unsigned long in_mhz = parent_rate / 1000000;
	unsigned long out_mhz = rate / 1000000;
	unsigned int feedback, npc, nsc, i;
	int div;

	for (div = 3; div >= 0; div--) {
		unsigned long vco = out_mhz << div;

		if (vco % in_mhz)
			continue;
		if (vco < PLL_VCO_MIN || vco > PLL_VCO_MAX)
			continue;

		feedback = vco / in_mhz;
		npc = (feedback - 4) / 4;
		nsc = feedback - npc * 4;

		for (i = 0; i < ARRAY_SIZE(lg1k_pll_cih); i++) {
			if (vco <= lg1k_pll_cih[i].freq)
				continue;

			r[0] &= ~PLL_CIH;
			r[0] |= FIELD_PREP(PLL_CIH, lg1k_pll_cih[i].cih);
			r[1] &= ~(PLL_FOUT_OD | PLL_NSC | PLL_NPC);
			r[1] |= FIELD_PREP(PLL_FOUT_OD, div);
			r[1] |= FIELD_PREP(PLL_NSC, nsc);
			r[1] |= FIELD_PREP(PLL_NPC, npc);
			return 0;
		}
	}

	return -EINVAL;
}

static int lg1k_pll_set_rate(struct clk_hw *hw, unsigned long rate,
			     unsigned long parent_rate)
{
	struct lg1k_pll *pll = to_lg1k_pll(hw);
	u32 r[3], v;
	int ret;

	lg1k_pll_read(pll, r);

	ret = lg1k_pll_find_ctrl(r, rate, parent_rate);
	if (ret)
		return ret;

	/* Park the CPU on the stable clock, relock the PLL, switch back. */
	v = readl_relaxed(pll->sw) | PLL_SWITCH;
	writel_relaxed(v, pll->sw);

	writel_relaxed(r[0] ^ pll->fix[0], pll->ctrl + PLL_CTRL0);
	writel_relaxed(r[1] ^ pll->fix[1], pll->ctrl + PLL_CTRL1);
	writel_relaxed(r[2] ^ pll->fix[2], pll->ctrl + PLL_CTRL2);

	udelay(50);	/* PLL relock */

	v = readl_relaxed(pll->sw) ^ PLL_SWITCH;
	writel_relaxed(v, pll->sw);

	return 0;
}

static const struct clk_ops lg1k_pll_ops = {
	.recalc_rate	= lg1k_pll_recalc_rate,
	.round_rate	= lg1k_pll_round_rate,
	.set_rate	= lg1k_pll_set_rate,
};

/*
 * GPU PLL (CTOP_DPE gpupll). Unscrambled window (no 'fix' mask); same field
 * layout as the CPU PLL but the feedback divider divides by M directly (not
 * M+1). set_rate rewrites ctrl0/ctrl1 bracketed by a park on the gpu_clk_sel
 * mux so the live GPU does not glitch while the PLL relocks (mirrors the CPU
 * PLL's switch-bit park).
 *
 * ctrl0 bit 2 disables spread spectrum. set_rate always sets it: the spread
 * parameters are not configured, and enabling spread spectrum on this PLL
 * makes the GPU unstable.
 */
#define GPU_PLL_CTRL0		0x00	/* CTOP_DPE gpupll word 0 (0xc930a0d8) */
#define GPU_PLL_CTRL1		0x04	/* CTOP_DPE gpupll word 1 (0xc930a0dc) */
#define GPU_PLL_DSS		BIT(2)	/* ctrl0: spread spectrum disable */
#define GPU_SEL			BIT(28)	/* gpu_clk_sel: park GPU off gpupll (0xc9c10464) */

struct lg1k_gpu {
	struct clk_hw		hw;
	void __iomem		*ctrl;	/* gpupll control words	*/
	void __iomem		*sel;	/* gpu_clk_sel mux	*/
};

#define to_lg1k_gpu(_hw)	container_of(_hw, struct lg1k_gpu, hw)

static unsigned long lg1k_gpu_recalc_rate(struct clk_hw *hw,
					  unsigned long parent_rate)
{
	const struct lg1k_gpu *gpu = to_lg1k_gpu(hw);
	unsigned long in_mhz = parent_rate / 1000000;
	u32 r0, r1, npc, nsc, m, od;

	r0 = readl_relaxed(gpu->ctrl + GPU_PLL_CTRL0);
	r1 = readl_relaxed(gpu->ctrl + GPU_PLL_CTRL1);

	npc = FIELD_GET(PLL_NPC, r1);
	nsc = FIELD_GET(PLL_NSC, r1);
	m   = FIELD_GET(PLL_M, r0);
	od  = FIELD_GET(PLL_FOUT_OD, r1);

	if (!m)
		m = 1;

	return ((((npc << 2) + nsc) * in_mhz / m) >> od) * 1000000;
}

static int lg1k_gpu_set_rate(struct clk_hw *hw, unsigned long rate,
			     unsigned long parent_rate)
{
	struct lg1k_gpu *gpu = to_lg1k_gpu(hw);
	u32 r[3], sel;
	int ret;

	r[0] = readl_relaxed(gpu->ctrl + GPU_PLL_CTRL0);
	r[1] = readl_relaxed(gpu->ctrl + GPU_PLL_CTRL1);
	r[2] = 0;

	ret = lg1k_pll_find_ctrl(r, rate, parent_rate);
	if (ret)
		return ret;

	/* recalc floors M to 1 and find_ctrl solves for /1; pin M to match. */
	r[0] &= ~PLL_M;

	/* Keep spread spectrum off. */
	r[0] |= GPU_PLL_DSS;

	/* Park the GPU on the alternate source while the PLL relocks. */
	sel = readl_relaxed(gpu->sel);
	writel_relaxed(sel | GPU_SEL, gpu->sel);

	/* Reconfigure. */
	writel_relaxed(r[0], gpu->ctrl + GPU_PLL_CTRL0);
	writel_relaxed(r[1], gpu->ctrl + GPU_PLL_CTRL1);
	udelay(50);				/* PLL relock */

	/* Unpark. */
	writel_relaxed(sel & ~GPU_SEL, gpu->sel);

	return 0;
}

static const struct clk_ops lg1k_gpu_ops = {
	.recalc_rate	= lg1k_gpu_recalc_rate,
	.round_rate	= lg1k_pll_round_rate,
	.set_rate	= lg1k_gpu_set_rate,
};

static struct clk_hw *lg1k_gpu_register(struct device_node *np,
					const char *parent)
{
	struct clk_init_data init = {
		.name		= "gpupll",
		.ops		= &lg1k_gpu_ops,
		.parent_names	= &parent,
		.num_parents	= 1,
		.flags		= CLK_GET_RATE_NOCACHE,
	};
	struct lg1k_gpu *gpu;
	int ret;

	gpu = kzalloc(sizeof(*gpu), GFP_KERNEL);
	if (!gpu)
		return ERR_PTR(-ENOMEM);

	gpu->ctrl = of_iomap(np, 2);
	gpu->sel = of_iomap(np, 3);
	if (!gpu->ctrl || !gpu->sel) {
		ret = -ENOMEM;
		goto err;
	}

	gpu->hw.init = &init;
	ret = clk_hw_register(NULL, &gpu->hw);
	if (ret)
		goto err;

	return &gpu->hw;

err:
	if (gpu->ctrl)
		iounmap(gpu->ctrl);
	if (gpu->sel)
		iounmap(gpu->sel);
	kfree(gpu);
	return ERR_PTR(ret);
}

static void __init lg1k_clk_setup(struct device_node *np)
{
	const char *parent = of_clk_get_parent_name(np, 0);
	struct clk_init_data init = {
		.name		= "cpupll",
		.ops		= &lg1k_pll_ops,
		.parent_names	= &parent,
		.num_parents	= 1,
		.flags		= CLK_GET_RATE_NOCACHE,
	};
	struct clk_hw_onecell_data *data;
	struct lg1k_pll *pll;
	struct clk_hw *cpu, *gpu;
	int ret;

	if (!parent) {
		pr_err("%pOF: no parent clock\n", np);
		return;
	}

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	data = kzalloc(struct_size(data, hws, LG1314_CLK_NR), GFP_KERNEL);
	if (!pll || !data)
		goto err;

	pll->ctrl = of_iomap(np, 0);
	pll->sw = of_iomap(np, 1);
	if (!pll->ctrl || !pll->sw) {
		pr_err("%pOF: cannot map registers\n", np);
		goto err;
	}

	if (of_property_read_u32_array(np, "fix", pll->fix, 3)) {
		pr_err("%pOF: missing 'fix' property\n", np);
		goto err;
	}

	pll->hw.init = &init;
	ret = clk_hw_register(NULL, &pll->hw);
	if (ret) {
		pr_err("%pOF: cannot register cpupll: %d\n", np, ret);
		goto err;
	}

	cpu = clk_hw_register_fixed_factor_parent_hw(NULL, "cpuclk", &pll->hw,
						     CLK_SET_RATE_PARENT, 1, 2);
	if (IS_ERR(cpu)) {
		pr_err("%pOF: cannot register cpuclk: %ld\n", np, PTR_ERR(cpu));
		goto err_unreg;
	}

	gpu = lg1k_gpu_register(np, parent);
	if (IS_ERR(gpu)) {
		pr_err("%pOF: cannot register gpupll: %ld\n", np, PTR_ERR(gpu));
		goto err_unreg_cpu;
	}

	data->num = LG1314_CLK_NR;
	data->hws[LG1314_CLK_CPUPLL] = &pll->hw;
	data->hws[LG1314_CLK_CPU] = cpu;
	data->hws[LG1314_CLK_GPU] = gpu;

	ret = of_clk_add_hw_provider(np, of_clk_hw_onecell_get, data);
	if (ret) {
		pr_err("%pOF: cannot add clock provider: %d\n", np, ret);
		goto err_unreg_gpu;
	}
	return;

err_unreg_gpu:
	clk_hw_unregister(gpu);
err_unreg_cpu:
	clk_hw_unregister_fixed_factor(cpu);
err_unreg:
	clk_hw_unregister(&pll->hw);
err:
	if (pll && pll->ctrl)
		iounmap(pll->ctrl);
	if (pll && pll->sw)
		iounmap(pll->sw);
	kfree(data);
	kfree(pll);
}
CLK_OF_DECLARE(lg1314_clk, "lge,lg1314-clk", lg1k_clk_setup);
