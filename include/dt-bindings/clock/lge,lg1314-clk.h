/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Clock IDs for the LG1314 (lg1k family) CPU clock controller.
 */
#ifndef _DT_BINDINGS_CLOCK_LGE_LG1314_CLK_H
#define _DT_BINDINGS_CLOCK_LGE_LG1314_CLK_H

#define LG1314_CLK_CPUPLL	0	/* CPU PLL (xtal-fed)			*/
#define LG1314_CLK_CPU		1	/* CPU clock = CPU PLL / 2 (cpufreq)	*/
#define LG1314_CLK_GPU		2	/* GPU PLL + gate (Mali-T820 "core")	*/

#define LG1314_CLK_NR		3

#endif /* _DT_BINDINGS_CLOCK_LGE_LG1314_CLK_H */
