// SPDX-License-Identifier: GPL-2.0
/*
 * LG1314 (M19) HDMI receiver: four Synopsys DesignWare HDMI RX 2.0 controllers
 * (one per connector), each with an LG "top" block (HPD, EDID SRAM, 5V detect,
 * PHY power) and an LG PHY on the SoC's I2C bus 9.
 *
 * Power-up, controller defaults, 5V detection, EDID and HPD, PHY init and link lock
 * (polled, as stock's main thread), and one V4L2 subdev per port (EDID, input status,
 * DV timings), audio clock recovery and the I2S outputs to the audio block.
 *
 * On LG's 2019 OLEDs (B9 etc.) the connectors are behind an HDMI 2.1 switch
 * ("HAWK2", lg-hawk2.c), which serves EDID/HPD/DDC to the sources; the SoC ports see
 * its TX side. The switch only drives a port once the PHY terminates it.
 *
 * Sequences follow LG's GPL kdriver (core/hdmi20/hdmi20_hal_driver.c,
 * synopsys_api/hdmi_rx_ctrl.c, chip/m19/hdmi20/hdmi20_rx_phy_m19a0.c).
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/math64.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#include <linux/v4l2-dv-timings.h>
#include <media/lg-hawk2.h>
#include <media/lg1314-hdmirx.h>
#include <media/v4l2-dv-timings.h>

#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#define HDMIRX_PORTS		4
#define HDMIRX_EDID_FW		"lg1314/hdmirx-edid.bin"
#define HDMIRX_POLL_MS		100
#define HDMIRX_HPD_LOW_MS	600	/* stock: HPD low at least this long at start */

/* top block (per port, LG) */
#define TOP_ARC			0x00c
#define  TOP_ARC_MODE		BIT(0)
#define TOP_HPD_DDC_SET		0x008
#define  TOP_HPD		BIT(4)
#define  TOP_HPD_SEL		BIT(2)
#define TOP_5V			0x010
#define TOP_OUT_FORMAT		0x014	/* link -> CVI pixel format (LG) */
#define TOP_OUT_422		0x018
#define  TOP_5V_DET		BIT(20)
#define TOP_MAIN_I2C_DELAY	0x024
#define TOP_ACR_PLL_MODE	0x02c
#define TOP_ACR_N		0x030	/* audio clock regeneration, from the packets */
#define TOP_ACR_CTS		0x034
#define TOP_ACR_N_FS		0x038	/* 0: 128 fs, 1: 256 fs (UD) */
#define TOP_ACR_DIV		0x03c
#define TOP_ACR_PLL_V		0x040	/* TMDS clock range */
#define TOP_AUD_INFO_CLR_SEL	0x06c
#define TOP_AUD_CHST0		0x070	/* IEC 60958 channel status byte 0 */
#define  CHST0_NON_PCM		BIT(1)
#define TOP_INTR_CLR_AUDIO	0x118
#define TOP_INFO_CLR_AUDIO	0x11c
#define TOP_MAIN_PHY_CTRL	0x090
#define  TOP_PHY_PDB		BIT(0)
#define TOP_PHY_I2C		0x094
#define  TOP_PHY_I2C_RSTN	BIT(0)
#define  TOP_PHY_I2C_PORT	GENMASK(9, 8)
#define TOP_MAIN_PVO_CTRL	0x0c0
#define TOP_CLK_MEAS_REF	0x958
#define TOP_EDID		0x1000
#define TOP_EDID_SIZE		512

/* ALED top (shared) */
#define ALED_I2C_CLK_SEL	0x050
/* per-port ALED blocks (aled-ports + 0x400 * port) */
#define ALED_PORT_SIZE		0x400
#define ALED_AD_HPD		0x254	/* HDMI20_RX_ALED_AD_HPD */
#define  ALED_AD_HPD_SET_HPD	BIT(0)

/* controller (Synopsys, RA_*) */
#define RA_HDMI_SETUP_CTRL	0x000
#define  HOT_PLUG_DETECT	BIT(0)
#define RA_HDMI_OVR_CTRL	0x004
#define RA_LINK_PHY_LOCK	0x030	/* LG: link side PHY lock */
#define  LINK_PHY_LOCKED	BIT(0)
#define RA_HDMI_PCB_CTRL	0x038
#define  INPUT_SELECT		BIT(16)
#define RA_HDMI_MODE_RECOVER	0x080	/* LG name: "HDMI mode recover" */
#define RA_GCP_CTRL		0x0a8
#define RA_LOCAL_MUTE0		0x0b0
#define RA_LOCAL_MUTE1		0x0b4
#define  LOCAL_MUTE		BIT(16)	/* link outputs the mute colour */
#define RA_HDMI_SYNC_CTRL	0x090
#define  VS_POL_ADJ_MODE	GENMASK(4, 3)
#define  HS_POL_ADJ_MODE	GENMASK(2, 1)
#define  POL_ADJ_AUTO		2
#define RA_HDMI_CKM_EVLTM	0x094
#define  EVAL_TIME		GENMASK(15, 4)
#define RA_HDMI_CKM_F		0x098
#define  CKM_MINFREQ		GENMASK(15, 0)
#define  CKM_MAXFREQ		GENMASK(31, 16)
#define RA_HDMI_CKM_RESULT	0x09c
#define  CKM_CLKRATE		GENMASK(15, 0)	/* TMDS clock measured; 0: none */
#define RA_HDMI_STS		0x0bc
#define RA_HDCP_CTRL		0x0c0
#define RA_HDCP_BCAPS		0x0c4
#define  DCM_CURRENT_MODE	GENMASK(31, 28)
#define RA_HA_THRESHOLD		0x140	/* LG name */
#define RA_MD_HT0		0x148
#define  HS_CLK			GENMASK(15, 0)	/* hsync, md_clk cycles */
#define RA_MD_HT1		0x14c
#define  HTOT_PIX		GENMASK(31, 16)
#define  HOFS_PIX		GENMASK(15, 0)
#define RA_MD_HACT_PX		0x150
#define RA_MD_VCTRL		0x158
#define  V_OFFS_LIN_MODE	BIT(4)	/* 1: MD_VOL = front porch */
#define RA_MD_VSC		0x15c	/* vsync duration, md_clk cycles */
#define RA_MD_VTC		0x160	/* frame duration, md_clk cycles */
#define RA_MD_VOL		0x164
#define RA_MD_VAL		0x168
#define RA_MD_VTL		0x170
#define RA_MD_STS		0x180
#define  ILACE			BIT(3)
#define RA_VERTICAL_TIMING	0x16c	/* LG name */
#define RA_AUD_CTRL		0x200
#define RA_AUD_PLL_CTRL		0x208
#define RA_AUD_CLK_CTRL		0x214
#define RA_AUD_FIFO_CTRL	0x240
#define  AFIF_INIT		BIT(0)
#define RA_AUD_FIFO_TH		0x244
#define RA_AUD_CHEXTR_CTRL	0x254
#define RA_AUD_MUTE_CTRL	0x258
#define  AUD_MUTE_SEL		GENMASK(6, 5)
#define RA_AUD_FIFO_FILLSTS1	0x25c
#define RA_AUD_SAO_CTRL		0x260	/* serial (I2S) outputs */
#define  SAO_ON_32BIT		0x001
#define  SAO_MUTED		0x1e0
#define RA_AUD_PAO_CTRL		0x264
#define RA_AUD_FIFO_STS		0x27c
#define  AUD_FIFO_PASS		BIT(2)
#define RA_HDMI_RESMPL_CTRL	0x0a4
#define RA_SNPS_PHYG3_CTRL	0x2c0
#define  PHYG3_PORTSEL		GENMASK(3, 2)
#define RA_PDEC_CTRL		0x300
#define  PFIFO_STORE_FILTER_EN	BIT(31)
#define  PFIFO_STORE_HDR	BIT(29)	/* LG's hdmi_rx_ctrl_packet_HDR (13) */
#define  PD_FIFO_FILL_INFO_CLR	BIT(8)
#define  PD_FIFO_CLR		BIT(5)
#define  PD_FIFO_WE		BIT(4)
#define  PDEC_BCH_EN		BIT(0)
#define RA_PDEC_ERR_FILTER	0x33c
#define RA_PDEC_ASP_CTRL	0x340
#define RA_PDEC_FIFO_STS	0x308
#define  PD_FIFO_FILL_MAX	GENMASK(25, 16)	/* words, since the last fill-info clear */
#define RA_PDEC_FIFO_DATA	0x30c	/* 8 reads per packet: HB0..2, 0, PB0..27 */
#define RA_PDEC_STS		0x360
#define RA_PDEC_AUD_STS		0x364
#define  AUD_STS_SAMPLES	BIT(0)	/* audio sample packets */
#define  AUD_STS_HBR		BIT(3)
#define RA_PDEC_ACR_CTS		0x390
#define RA_PDEC_ACR_N		0x394
#define  ACR_VAL		GENMASK(19, 0)
#define RA_PDEC_AIF_CTRL	0x3c0
#define RA_PDEC_AIF_PB0		0x3c8
#define  AIF_CC			GENMASK(2, 0)	/* channel count - 1, 0: from the stream */
#define RA_PDEC_RSVD_HDR	0x900
#define  DVIDET			BIT(28)
#define RA_PDEC_AVI_PB		0x3a4
#define  VID_IDENT_CODE		GENMASK(30, 24)
#define  AVI_EXT_COLORIMETRY	GENMASK(22, 20)
#define  AVI_RGB_QUANT		GENMASK(19, 18)
#define  AVI_COLORIMETRY	GENMASK(15, 14)
#define  VIDEO_FORMAT		GENMASK(6, 5)	/* 3: YCbCr 4:2:0 */
#define RA_SCDC_CONFIG		0x800
#define RA_SCDC_808		0x808	/* stock writes 1 (meaning not documented) */
#define RA_SCDC_80C		0x80c
#define RA_HDCP_OVERRIDE	0x81c
#define RA_SCDC_MANUF		0x860
#define RA_SCDC_REGS0		0x820
#define  SCDC_TMDS_RATIO_40	BIT(17)	/* stock HDMI20_HAL_IsSCDCvalueUD */
#define  AUTO_VMUTE		BIT(6)
#define  AUTO_SPFLAT_MUTE	GENMASK(5, 2)
#define RA_HDCP22_IEN_CLR	0xf60
#define RA_HDCP22_ICLR		0xf70
#define RA_PDEC_IEN_CLR		0xf78
#define RA_PDEC_ICLR		0xf88
#define RA_AUD_CLK_IEN_CLR	0xf90
#define RA_AUD_CLK_ICLR		0xfa0
#define RA_AUD_FIFO_IEN_CLR	0xfa8
#define RA_AUD_FIFO_ISTS	0xfb0
#define RA_AUD_FIFO_ICLR	0xfb8
#define RA_MD_IEN_CLR		0xfc0
#define RA_MD_ICLR		0xfd0
#define RA_HDMI_IEN_CLR		0xfd8
#define RA_HDMI_IEN_SET		0xfdc
#define  CLK_CHANGE		GENMASK(6, 5)
#define RA_HDMI_ISTS		0xfe0
#define  VS_POL_ADJ_ISTS	BIT(9)	/* sync polarity adjusted: source sends positive */
#define  HS_POL_ADJ_ISTS	BIT(8)
#define RA_HDMI_ICLR		0xfe8
#define RA_DMI_SW_RST		0xff0
#define  DMI_SW_RST_AUDIO	BIT(4)

/* stock values (gHDMI20RxDevControler defaults, M19) */
#define LG_MODE_RECOVER		0x30010000
#define LG_VERTICAL_TIMING	0x00000852
#define LG_HA_THRESHOLD		0x00000110
#define LG_CLK_MEAS_REF		0x960
#define LG_PVO_MODE		0x15
#define LG_ACR_PLL_MODE		0x10001
#define LG_FF0_RESET_0		0xfefe	/* LINK_REG_0XFF0_RESET_VALUE_0 */
#define LG_HDCP_BCAPS		0x12174
#define LG_GCP_CTRL		0x40414
#define LG_SCDC_CONFIG		0x1f11
#define LG_SCDC_MANUF		0xabcdef01
#define LG_SCDC_80C		0x3ff0c15c

/* PHY (LG, I2C): bank 0 at 0x28 + port, bank 1 ("B1") at 0x38 + port */
#define PHY_ADDR(bank, port)	((bank) ? 0x38 + (port) : 0x28 + (port))
#define PHY_TMDS_FREQ_MSB	0x31	/* bank 0: TMDS clock in 10 kHz units */
#define PHY_TMDS_FREQ_LSB	0x32
#define PHY_CR_DONE		0x34	/* bank 0, bit 0 */
#define PHY_EQ_CAL		0x39	/* bank 0, bit 0 eq_cal_mode_val */
#define PHYB1_CS_MAN_CH0	0x5e	/* bits 4:0, ch1/ch2 follow */
#define PHYB1_TCS_DONE		0x64	/* bit 4 */
#define PHYB1_SCDT		0x7b	/* bits 3..1: scdt_ch0..2 */
#define PHYB1_TCS_EN		0x83	/* bit 0 */
#define PHYB1_DR_N2		0xa2	/* bits 6:4 */
#define PHYB1_CLK_INV		0xab	/* bit 5 muxed_clk_inv (ports 1/2) */
#define PHYB1_DFE		0xb0	/* bit 5 dfe_dig_on */
#define PHY_LOCK_RETRY_POLLS	30	/* ours: re-init the PHY after 3 s without lock */
#define LOCAL_MUTE_HOLD_MS	500	/* stock LOCAL_MUTE_HOLD_ORIGIN_TIME */

/*
 * Audio, per poll (stock HDMI20_HAL_Rx_Get_AudioInfo counts 20 ms polls: stable
 * after 0x14, audio reset at 0x13; ours 100 ms). The receiver's own FIFO should
 * sit between these fill levels; stock nudges CTS by 0x32 to keep it there.
 */
#define AUD_STABLE_RESET	3	/* polls without error: clear and reset the audio path */
#define AUD_STABLE		5	/* then unmute */
#define AUD_ACR_ERR_MAX		10	/* polls with an empty FIFO: restart the ACR PLL */
#define AUD_FIFO_FILL_MIN	0x100
#define AUD_FIFO_FILL_MAX	0x180
#define AUD_CTS_ADJUST		0x32

struct hdmirx_phy_op {
	u8 bank, reg, mask, val_3g, val_6g;
};

#include "lg1314-hdmirx-phy.h"

/* clocks (kHz): mode detection, TMDS range */
#define MD_CLK_KHZ		49500
#define TMDS_CLK_MIN_KHZ	25000
#define TMDS_CLK_MAX_KHZ	600000

/* glue registers outside the receiver (single words in shared pages) */
#define DDC_GPIO_OFF		0x90	/* 0xc9302018: DDC pins out of GPIO mode */
#define HDCP22_GATE_OFF		0x3fc	/* 0xc930209c: HDCP 2.2 engine bus gating off */

struct lg1314_hdmirx;

struct hdmirx_port {
	struct lg1314_hdmirx *rx;
	unsigned int id;
	void __iomem *top;
	void __iomem *ctrl;
	struct v4l2_subdev sd;

	struct mutex lock;		/* the state below */
	u8 edid[TOP_EDID_SIZE];
	unsigned int edid_blocks;	/* 0: no EDID, HPD stays low */
	bool cable;			/* source 5V seen */
	bool hpd;
	bool phy_up;			/* Rx_Initiate_Phy done for this connection */
	bool ud;			/* PHY in 6G mode (TMDS bit clock ratio 1/40, scrambled) */
	bool locked;
	unsigned int nolock;		/* polls with TMDS but no lock */
	int out_format;			/* TOP_OUT_* set for; -1: not yet */
	bool muted;			/* link outputs the mute colour */
	unsigned long mute_since;	/* jiffies */
	/* the signal as reported (lg1314_hdmirx_get_signal): taken once the
	 * same measurement was seen on two polls in a row */
	u32 sig_key[8];
	bool sig_settled;
	struct lg1314_hdmirx_signal sig;
	bool sig_changed;		/* to report after this poll */
	/* audio (independent of sig: no notifier, no effect on the picture) */
	struct lg1314_hdmirx_audio aud;
	bool aud_changed;
	bool aud_muted;
	bool acr_clear;			/* first ACR update after a reset: zeros, as stock */
	unsigned int aud_stable;	/* polls without FIFO error */
	/* HDR infoframe (Dynamic Range and Mastering), from the packet FIFO */
	struct hdr_metadata_infoframe hdr;	/* eotf 0 without one */
	unsigned long hdr_seen;		/* jiffies of the last one */
	unsigned int acr_err;		/* polls with an empty FIFO */
};

struct lg1314_hdmirx {
	struct device *dev;
	void __iomem *aled;
	void __iomem *aled_port;	/* per-port ALED blocks */
	void __iomem *ddc;
	void __iomem *gate;
	struct gpio_desc *isol;
	struct i2c_adapter *phy_i2c;
	struct v4l2_device v4l2;
	struct hdmirx_port port[HDMIRX_PORTS];
	struct delayed_work poll;
	bool started;			/* HPD-low period over, isolation released */
	struct lg_hawk2 *sw;		/* HDMI switch in front, if any */
	int connector_port[LG1314_HDMIRX_CONNECTORS];	/* HDMI k -> port */
	unsigned int selected;		/* connector shown (lg1314_hdmirx_select) */
	/*
	 * Receiver ports the switch spreads each connector's picture over (1: none),
	 * as of the last poll; the picture is then on the ports of HDMI 1..n, left to
	 * right (stock's _VFE_HDMI_CheckVideoSplitTimingValid), whatever the
	 * connector. split_conn: the connector split now (0: none).
	 */
	unsigned int split[LG1314_HDMIRX_CONNECTORS];
	unsigned int split_conn;
	struct blocking_notifier_head notifier;
};

static u32 top_rd(struct hdmirx_port *p, u32 off)
{
	return readl(p->top + off);
}

static void top_wr(struct hdmirx_port *p, u32 off, u32 val)
{
	writel(val, p->top + off);
}

static void top_rmw(struct hdmirx_port *p, u32 off, u32 clr, u32 set)
{
	top_wr(p, off, (top_rd(p, off) & ~clr) | set);
}

static u32 ctrl_rd(struct hdmirx_port *p, u32 off)
{
	return readl(p->ctrl + off);
}

static void ctrl_wr(struct hdmirx_port *p, u32 off, u32 val)
{
	writel(val, p->ctrl + off);
}

static void ctrl_field(struct hdmirx_port *p, u32 off, u32 mask, u32 val)
{
	ctrl_wr(p, off, (ctrl_rd(p, off) & ~mask) | ((val << __ffs(mask)) & mask));
}

/* PHY power: PDB, then (1 ms later) its I2C out of reset. */
static void hdmirx_phy_power(struct hdmirx_port *p, bool on)
{
	if (on) {
		top_rmw(p, TOP_MAIN_PHY_CTRL, 0, TOP_PHY_PDB);
		usleep_range(1000, 2000);
		top_rmw(p, TOP_PHY_I2C, 0, TOP_PHY_I2C_RSTN);
	} else {
		top_rmw(p, TOP_MAIN_PHY_CTRL, TOP_PHY_PDB, 0);
		top_rmw(p, TOP_PHY_I2C, TOP_PHY_I2C_RSTN, 0);
	}
}

static void hdmirx_link_reset(struct hdmirx_port *p, bool main);

/*
 * HDMI20_HAL_Rx_SetLocalVideoMute: while the link (re)trains, it outputs the mute
 * colour (black) instead of whatever it decodes; released once the signal is
 * stable (hdmirx_poll_port()).
 */
static void hdmirx_mute(struct hdmirx_port *p, bool on)
{
	u32 v = readl(p->ctrl + RA_LOCAL_MUTE1);

	writel(on ? v | LOCAL_MUTE : v & ~LOCAL_MUTE, p->ctrl + RA_LOCAL_MUTE1);
	if (on && !p->muted)
		p->mute_since = jiffies;
	p->muted = on;
}

static int phy_rd(struct hdmirx_port *p, int bank, u8 reg)
{
	union i2c_smbus_data d;
	int ret;

	ret = i2c_smbus_xfer(p->rx->phy_i2c, PHY_ADDR(bank, p->id), 0, I2C_SMBUS_READ, reg,
			     I2C_SMBUS_BYTE_DATA, &d);
	return ret < 0 ? ret : d.byte;
}

static int phy_rmw(struct hdmirx_port *p, int bank, u8 reg, u8 mask, u8 val)
{
	union i2c_smbus_data d;
	int old = phy_rd(p, bank, reg);

	if (old < 0)
		return old;
	d.byte = (old & ~mask) | (val & mask);
	return i2c_smbus_xfer(p->rx->phy_i2c, PHY_ADDR(bank, p->id), 0, I2C_SMBUS_WRITE, reg,
			      I2C_SMBUS_BYTE_DATA, &d);
}

static int hdmirx_phy_ops(struct hdmirx_port *p, unsigned int first, unsigned int end, bool ud)
{
	unsigned int i;
	int ret;

	for (i = first; i < end; i++) {
		const struct hdmirx_phy_op *op = &hdmirx_phy_seq[i];

		ret = phy_rmw(p, op->bank, op->reg, op->mask, ud ? op->val_6g : op->val_3g);
		if (!ret && op->bank && op->reg == PHYB1_CLK_INV && (p->id == 1 || p->id == 2))
			ret = phy_rmw(p, 1, PHYB1_CLK_INV, BIT(5), BIT(5));
		if (ret)
			return ret;
	}
	return 0;
}

/* SCDC TMDS_Config as written by the source (here: HAWK2's TX) */
static bool hdmirx_scdc_ud(struct hdmirx_port *p)
{
	return ctrl_rd(p, RA_SCDC_REGS0) & SCDC_TMDS_RATIO_40;
}

/* HDMI20_PHY_M19A0_Rx_Initiate_Phy; the link main reset (LinkVideoReset(1)) before it */
static int hdmirx_phy_init(struct hdmirx_port *p)
{
	int ret;

	hdmirx_link_reset(p, true);
	hdmirx_mute(p, true);
	p->ud = hdmirx_scdc_ud(p);
	p->locked = false;
	p->nolock = 0;
	ret = hdmirx_phy_ops(p, 0, ARRAY_SIZE(hdmirx_phy_seq), p->ud);
	top_rmw(p, TOP_ARC, TOP_ARC_MODE, 0);
	if (ret)
		dev_err(p->rx->dev, "port %u: PHY init failed: %d\n", p->id, ret);
	else
		dev_info(p->rx->dev, "port %u: PHY init (%s)\n", p->id, p->ud ? "6G" : "3G");
	return ret;
}

/* __HDMI20_HAL_Rx_CheckSCDC: SCDC clock ratio changed -> HDMI20_PHY_M19A0_Set_3G6G_Mode */
static void hdmirx_phy_set_mode(struct hdmirx_port *p, bool ud)
{
	dev_info(p->rx->dev, "port %u: SCDC -> %s\n", p->id, ud ? "6G" : "3G");
	if (ud)
		ctrl_wr(p, RA_DMI_SW_RST, LG_FF0_RESET_0);
	else
		hdmirx_link_reset(p, false);
	hdmirx_mute(p, true);
	p->ud = ud;
	p->locked = false;
	p->nolock = 0;
	hdmirx_phy_ops(p, PHY_MODE_FIRST, PHY_MODE_END, ud);
	/* RS gain changed: redo the EQ offset calibration */
	phy_rmw(p, 0, PHY_EQ_CAL, BIT(0), 0);
	phy_rmw(p, 0, PHY_EQ_CAL, BIT(0), BIT(0));
}

/* __HDMI20_PHY_M19A0_SetPhaseSW: TCS off until CR done + SCDT again */
static void hdmirx_phy_phase_sw(struct hdmirx_port *p)
{
	int i;

	phy_rmw(p, 1, PHYB1_TCS_EN, BIT(0), 0);
	for (i = 0; i < 3; i++)
		phy_rmw(p, 1, PHYB1_CS_MAN_CH0 + i, 0x1f, p->ud ? 0x8 : 0xa);
	phy_rmw(p, 1, PHYB1_DFE, BIT(5), BIT(5));
	phy_rmw(p, 1, PHYB1_DR_N2, 0x70, p->ud << 4);
}

/*
 * HDMI20_PHY_M19A0_UpdatePhyLockFlag, cut down: with the phase SW workaround on
 * (stock default), start TCS once clock recovery is done and all three channels
 * have symbol sync; locked = TCS done and the link side reports PHY lock. Stock also
 * checks TCS min/max history and DFE B1 levels (all 0 by default) - not done here.
 */
static void hdmirx_phy_update(struct hdmirx_port *p)
{
	bool locked;
	int tcs, cr, scdt;

	if (hdmirx_scdc_ud(p) != p->ud)
		hdmirx_phy_set_mode(p, !p->ud);

	tcs = phy_rd(p, 1, PHYB1_TCS_DONE);
	if (tcs < 0)
		return;
	if (!(tcs & BIT(4))) {
		cr = phy_rd(p, 0, PHY_CR_DONE);
		scdt = phy_rd(p, 1, PHYB1_SCDT);
		if (cr >= 0 && scdt >= 0 && (cr & BIT(0)) && (scdt & 0x0e) == 0x0e) {
			phy_rmw(p, 1, PHYB1_DR_N2, 0x70, 0x70);
			phy_rmw(p, 1, PHYB1_TCS_EN, BIT(0), BIT(0));
			phy_rmw(p, 1, PHYB1_DFE, BIT(5), BIT(5));
		}
	}

	/*
	 * TCS done and the link-PHY lock can stay set after the clock goes (seen on
	 * a port the HDMI switch stopped feeding when a split ended): also require
	 * the controller to measure a clock.
	 */
	locked = (tcs & BIT(4)) && (ctrl_rd(p, RA_LINK_PHY_LOCK) & LINK_PHY_LOCKED) &&
		 (ctrl_rd(p, RA_HDMI_CKM_RESULT) & CKM_CLKRATE);
	if (locked && !p->locked) {
		ctrl_wr(p, RA_DMI_SW_RST, LG_FF0_RESET_0);
		dev_info(p->rx->dev, "port %u: link locked (%s)\n", p->id, p->ud ? "6G" : "3G");
	} else if (!locked && p->locked) {
		dev_info(p->rx->dev, "port %u: link lost\n", p->id);
		hdmirx_mute(p, true);
		hdmirx_phy_phase_sw(p);
	}
	p->locked = locked;

	/* stock just waits while there is no TMDS clock */
	if (locked || phy_rd(p, 0, PHY_TMDS_FREQ_MSB) <= 0)
		p->nolock = 0;
	else if (++p->nolock >= PHY_LOCK_RETRY_POLLS)
		hdmirx_phy_init(p);
}

/* __HDMI20_HAL_Rx_Set_EnableHPD (not inverted) */
static void hdmirx_set_hpd(struct hdmirx_port *p, bool on)
{
	ctrl_field(p, RA_HDMI_SETUP_CTRL, HOT_PLUG_DETECT, on);
	top_rmw(p, TOP_HPD_DDC_SET, TOP_HPD, (on ? TOP_HPD : 0) | TOP_HPD_SEL);
	if (on != p->hpd)
		dev_info(p->rx->dev, "port %u: HPD %s\n", p->id, on ? "up" : "down");
	p->hpd = on;
}

static void hdmirx_write_edid(struct hdmirx_port *p)
{
	u32 w;
	int i;

	for (i = 0; i < TOP_EDID_SIZE; i += 4) {
		memcpy(&w, &p->edid[i], 4);
		top_wr(p, TOP_EDID + i, le32_to_cpu((__force __le32)w));
	}
}

static void hdmirx_audio_fifo_reset(struct hdmirx_port *p)
{
	ctrl_wr(p, RA_AUD_FIFO_ICLR, 0x1f);
	ctrl_field(p, RA_AUD_FIFO_CTRL, AFIF_INIT, 1);
	ctrl_field(p, RA_AUD_FIFO_CTRL, AFIF_INIT, 0);
}

/*
 * The receiver's I2S outputs feed the audio block ("hmr" inputs, selected and
 * captured by the audio DSP). Muted: outputs off, as stock's
 * HDMI20_HAL_Rx_Set_AudioMuteControl.
 */
static void hdmirx_audio_mute(struct hdmirx_port *p, bool on)
{
	if (on != p->aud_muted)
		p->aud_changed = true;	/* live or not: the audio DSP follows it */
	if (on) {
		ctrl_wr(p, RA_AUD_SAO_CTRL, SAO_MUTED);
	} else {
		ctrl_field(p, RA_AUD_MUTE_CTRL, AUD_MUTE_SEL, 0);	/* force unmute */
		ctrl_wr(p, RA_AUD_SAO_CTRL, SAO_ON_32BIT);
	}
	p->aud_muted = on;
}

/* HDMI20_HAL_Rx_Set_Audio_Configure (non-HBR); after every link reset */
static void hdmirx_audio_config(struct hdmirx_port *p)
{
	top_wr(p, TOP_ACR_DIV, 0);
	top_wr(p, TOP_ACR_N_FS, p->ud);
	ctrl_wr(p, RA_AUD_CTRL, 0x60);
	ctrl_wr(p, RA_AUD_CLK_CTRL, 3);
	ctrl_wr(p, RA_AUD_PLL_CTRL, 0);
	ctrl_wr(p, RA_AUD_FIFO_TH, 0x01401008);
	ctrl_wr(p, RA_AUD_FIFO_CTRL, 0x10000);
	ctrl_wr(p, RA_HDMI_RESMPL_CTRL, 1);
	ctrl_wr(p, RA_AUD_MUTE_CTRL, 0x007004a8);
	ctrl_wr(p, RA_AUD_PAO_CTRL, 0x6000);
	ctrl_wr(p, RA_PDEC_AIF_CTRL, 0x100);
	ctrl_wr(p, RA_AUD_CHEXTR_CTRL, 0xfc);		/* 8 channels */
	hdmirx_audio_mute(p, true);
	hdmirx_audio_fifo_reset(p);
	p->acr_clear = true;
	p->aud_stable = 0;
	p->acr_err = 0;
}

/* TMDS character clock, kHz (the PHY measures the clock lane: 1/4 of it when UD) */
static unsigned int hdmirx_tmds_khz(struct hdmirx_port *p)
{
	int msb = phy_rd(p, 0, PHY_TMDS_FREQ_MSB), lsb = phy_rd(p, 0, PHY_TMDS_FREQ_LSB);

	if (msb < 0 || lsb < 0)
		return 0;
	return ((msb << 8) | lsb) * 10 * (p->ud ? 4 : 1);
}

/* HDMI20_HAL_Rx_Get_PLLFreqRange */
static u32 hdmirx_acr_range(unsigned int khz)
{
	return khz <= 40000 ? 0 : khz <= 80000 ? 1 : khz <= 160000 ? 2 : 3;
}

/* HDMI20_HAL_Rx_Get_AudioFreqFromTMDSClock: fs = N * f_TMDS / (128 * CTS) */
static unsigned int hdmirx_audio_rate(u32 n, u32 cts, unsigned int khz)
{
	static const struct { unsigned int below, rate; } map[] = {
		{ 22983, 22050 }, { 30000, 24000 }, { 33800, 32000 }, { 45965, 44100 },
		{ 67000, 48000 }, { 91935, 88200 }, { 135000, 96000 }, { 183870, 176400 },
		{ 210000, 192000 },
	};
	u64 fs;
	int i;

	if (!n || !cts || !khz)
		return 0;
	fs = div64_u64((u64)n * khz * 1000, (u64)cts * 128);
	for (i = 0; i < ARRAY_SIZE(map); i++)
		if (fs < map[i].below)
			return map[i].rate;
	return 768000;
}

/*
 * Stock's audio thread work, cut down: ACR values from the packets into the audio
 * PLL, FIFO watch (reset on errors, PLL restart when it stays empty, CTS nudged to
 * keep the fill centred), and the output muted until the stream has been clean
 * for AUD_STABLE polls.
 */
static void hdmirx_audio_poll(struct hdmirx_port *p)
{
	struct lg1314_hdmirx_audio aud = { };
	u32 n = 0, cts = 0, sts, ists, fill, st;
	unsigned int khz = 0;

	if (p->locked) {
		n = FIELD_GET(ACR_VAL, ctrl_rd(p, RA_PDEC_ACR_N));
		cts = FIELD_GET(ACR_VAL, ctrl_rd(p, RA_PDEC_ACR_CTS));
		khz = hdmirx_tmds_khz(p);
	}
	aud.rate = hdmirx_audio_rate(n, cts, khz);
	st = p->locked ? ctrl_rd(p, RA_PDEC_AUD_STS) : 0;
	if (st & AUD_STS_HBR)
		aud.type = LG1314_HDMIRX_AUDIO_HBR;
	else if (st & AUD_STS_SAMPLES)
		aud.type = top_rd(p, TOP_AUD_CHST0) & CHST0_NON_PCM ?
			   LG1314_HDMIRX_AUDIO_COMPRESSED : LG1314_HDMIRX_AUDIO_PCM;
	if (aud.type == LG1314_HDMIRX_AUDIO_PCM) {
		aud.channels = FIELD_GET(AIF_CC, ctrl_rd(p, RA_PDEC_AIF_PB0)) + 1;
		if (aud.channels == 1)
			aud.channels = 2;	/* "refer to stream header" */
	}
	if (!aud.type || !aud.rate)
		aud = (struct lg1314_hdmirx_audio){ };

	if (memcmp(&aud, &p->aud, sizeof(aud))) {
		if (aud.type || p->aud.type)
			dev_info(p->rx->dev, "port %u: audio %u, %u Hz, %u ch\n", p->id,
				 aud.type, aud.rate, aud.channels);
		p->aud = aud;
		p->aud_changed = true;
		p->aud_stable = 0;
		if (!p->aud_muted)
			hdmirx_audio_mute(p, true);
	}

	if (p->acr_clear || !aud.type) {
		n = 0;
		cts = 0;
		p->acr_clear = false;
	} else if (p->aud_stable >= AUD_STABLE) {
		fill = ctrl_rd(p, RA_AUD_FIFO_FILLSTS1);
		if (fill > AUD_FIFO_FILL_MAX)
			cts -= AUD_CTS_ADJUST;
		else if (fill < AUD_FIFO_FILL_MIN)
			cts += AUD_CTS_ADJUST;
	}
	top_wr(p, TOP_ACR_N_FS, p->ud);
	top_wr(p, TOP_ACR_CTS, cts);
	top_wr(p, TOP_ACR_N, n);
	top_wr(p, TOP_ACR_PLL_V, hdmirx_acr_range(khz));
	if (!aud.type)
		return;

	sts = ctrl_rd(p, RA_AUD_FIFO_STS);
	ists = ctrl_rd(p, RA_AUD_FIFO_ISTS);
	if (!sts || !ists) {
		if (++p->acr_err >= AUD_ACR_ERR_MAX) {
			top_wr(p, TOP_ACR_PLL_MODE, 0x110001);
			top_wr(p, TOP_ACR_PLL_MODE, 0x110101);
			top_wr(p, TOP_ACR_PLL_MODE, 0x110001);
			top_wr(p, TOP_ACR_PLL_MODE, LG_ACR_PLL_MODE);
			p->acr_err = 0;
			dev_info(p->rx->dev, "port %u: audio clock restarted\n", p->id);
		}
	} else {
		p->acr_err = 0;
	}

	if (sts != AUD_FIFO_PASS || ists != AUD_FIFO_PASS) {
		if (!p->aud_muted)
			hdmirx_audio_mute(p, true);
		if (p->aud_stable >= AUD_STABLE)
			dev_info(p->rx->dev, "port %u: audio FIFO error %x/%x\n", p->id, sts, ists);
		p->aud_stable = 0;
		ctrl_wr(p, RA_DMI_SW_RST, DMI_SW_RST_AUDIO);
		hdmirx_audio_fifo_reset(p);
		return;
	}
	if (p->aud_stable < AUD_STABLE)
		p->aud_stable++;
	if (p->aud_stable == AUD_STABLE_RESET) {
		/* clear the latched audio info and start the path clean */
		top_wr(p, TOP_INFO_CLR_AUDIO, BIT(3));
		top_wr(p, TOP_AUD_INFO_CLR_SEL, 1);
		top_wr(p, TOP_INFO_CLR_AUDIO, 0);
		top_wr(p, TOP_AUD_INFO_CLR_SEL, 0);
		usleep_range(4000, 5000);
		top_wr(p, TOP_INTR_CLR_AUDIO, 0x1800);
		ctrl_wr(p, RA_DMI_SW_RST, DMI_SW_RST_AUDIO);
		hdmirx_audio_fifo_reset(p);
	} else if (p->aud_stable == AUD_STABLE && p->aud_muted) {
		hdmirx_audio_mute(p, false);
		dev_info(p->rx->dev, "port %u: audio unmuted\n", p->id);
	}
}

/* hdmi_rx_ctrl_config */
/*
 * The source's HDR infoframe (CTA-861 Dynamic Range and Mastering, type 0x87). The
 * controller stores one in the packet FIFO each time it arrives (every frame). The
 * FIFO's status bits are no help here (the "new entry" one is a latched interrupt
 * status, cleared with the audio ones; reading past the end doesn't underflow, it
 * returns stale words), so: after a clear, read the first packet if one arrived
 * (the fill high-water mark says so), check it, clear again. Dropped when none
 * came for a while.
 */
#define HDR_IF_TYPE		0x87
#define HDR_IF_TIMEOUT_MS	500

static bool hdmirx_hdr_parse(const u8 *b, struct hdr_metadata_infoframe *md)
{
	unsigned int len = b[2], i;
	const u8 *pb = &b[4];
	u8 sum = b[0] + b[1] + b[2];

	if (b[0] != HDR_IF_TYPE || b[1] != 1 || len < 26 || len > 27)
		return false;
	for (i = 0; i <= len; i++)
		sum += pb[i];
	if (sum)
		return false;
	memset(md, 0, sizeof(*md));
	md->eotf = pb[1] & 7;
	md->metadata_type = pb[2] & 7;
	for (i = 0; i < 3; i++) {
		md->display_primaries[i].x = get_unaligned_le16(&pb[3 + 4 * i]);
		md->display_primaries[i].y = get_unaligned_le16(&pb[5 + 4 * i]);
	}
	md->white_point.x = get_unaligned_le16(&pb[15]);
	md->white_point.y = get_unaligned_le16(&pb[17]);
	md->max_display_mastering_luminance = get_unaligned_le16(&pb[19]);
	md->min_display_mastering_luminance = get_unaligned_le16(&pb[21]);
	md->max_cll = get_unaligned_le16(&pb[23]);
	md->max_fall = get_unaligned_le16(&pb[25]);
	return true;
}

/*
 * The DE firmware learns an input's HDR state from this LG register (it reads it
 * through the receiver's second window at 0xC9040000 + 0x8000 * port + 0x7f40) and
 * only then treats the window as HDR (mode, EOTF, tone curves). Stock
 * HDMI20_HAL_SetCscHdrInfoToVSC: EOTF [3:0], max mastering luminance in thousands
 * of nits, rounded up, 1..15, [7:4].
 */
#define RA_LG_HDR_INFO		0x1f40

static void hdmirx_hdr_info(struct hdmirx_port *p)
{
	unsigned int lum = p->hdr.max_display_mastering_luminance;
	u32 v = (lum ? min((lum - 1) / 1000 + 1, 15u) : 1) << 4 | (p->hdr.eotf & 0xf);

	if (ctrl_rd(p, RA_LG_HDR_INFO) != v)
		ctrl_wr(p, RA_LG_HDR_INFO, v);
}

static void hdmirx_pdec_fifo_clear(struct hdmirx_port *p)
{
	u32 v = ctrl_rd(p, RA_PDEC_CTRL);

	ctrl_wr(p, RA_PDEC_CTRL, v | PD_FIFO_CLR | PD_FIFO_FILL_INFO_CLR);
	ctrl_wr(p, RA_PDEC_CTRL, v & ~(PD_FIFO_CLR | PD_FIFO_FILL_INFO_CLR));
}

static void hdmirx_hdr_poll(struct hdmirx_port *p)
{
	struct hdr_metadata_infoframe md;
	unsigned int i;
	u8 b[32];

	if (p->locked && FIELD_GET(PD_FIFO_FILL_MAX, ctrl_rd(p, RA_PDEC_FIFO_STS)) >= 8) {
		for (i = 0; i < sizeof(b); i += 4)
			put_unaligned_le32(ctrl_rd(p, RA_PDEC_FIFO_DATA), &b[i]);
		if (hdmirx_hdr_parse(b, &md)) {
			p->hdr = md;
			p->hdr_seen = jiffies;
		}
	}
	hdmirx_pdec_fifo_clear(p);
	if (!p->locked || (p->hdr.eotf &&
	    time_after(jiffies, p->hdr_seen + msecs_to_jiffies(HDR_IF_TIMEOUT_MS))))
		memset(&p->hdr, 0, sizeof(p->hdr));
	hdmirx_hdr_info(p);
}

static void hdmirx_ctrl_config(struct hdmirx_port *p)
{
	/* controller 1.30a: the evaluation period counts md_clk cycles */
	u32 evaltime = 4095, min, max;

	/* control_cfg; each controller has a single PHY: input port 0 */
	ctrl_wr(p, RA_HDMI_OVR_CTRL, ~0);
	ctrl_field(p, RA_HDMI_SYNC_CTRL, VS_POL_ADJ_MODE, POL_ADJ_AUTO);
	ctrl_field(p, RA_HDMI_SYNC_CTRL, HS_POL_ADJ_MODE, POL_ADJ_AUTO);
	ctrl_field(p, RA_HDMI_CKM_EVLTM, EVAL_TIME, evaltime);
	min = TMDS_CLK_MIN_KHZ * evaltime / MD_CLK_KHZ;
	max = TMDS_CLK_MAX_KHZ * evaltime / MD_CLK_KHZ;
	ctrl_field(p, RA_HDMI_CKM_F, CKM_MINFREQ, min);
	ctrl_field(p, RA_HDMI_CKM_F, CKM_MAXFREQ, max);
	ctrl_field(p, RA_HDMI_PCB_CTRL, INPUT_SELECT, 0);
	ctrl_field(p, RA_SNPS_PHYG3_CTRL, PHYG3_PORTSEL, 0);
	/* ours (not stock): MD_VOL reports the vertical front porch, see hdmirx_measure() */
	ctrl_field(p, RA_MD_VCTRL, V_OFFS_LIN_MODE, 1);

	/*
	 * packets: HDR infoframes into the FIFO (hdmirx_hdr_poll()); audio (muted
	 * until it is stable, hdmirx_audio_poll())
	 */
	ctrl_wr(p, RA_PDEC_CTRL, PFIFO_STORE_FILTER_EN | PFIFO_STORE_HDR | PD_FIFO_WE |
		PDEC_BCH_EN);
	ctrl_wr(p, RA_PDEC_ASP_CTRL, AUTO_VMUTE | AUTO_SPFLAT_MUTE);
	hdmirx_audio_config(p);
	ctrl_wr(p, RA_PDEC_CTRL, ctrl_rd(p, RA_PDEC_CTRL) | PD_FIFO_FILL_INFO_CLR | PD_FIFO_CLR);
	ctrl_wr(p, RA_PDEC_CTRL, ctrl_rd(p, RA_PDEC_CTRL) & ~(PD_FIFO_FILL_INFO_CLR | PD_FIFO_CLR));
	ctrl_wr(p, RA_HDMI_IEN_SET, CLK_CHANGE);	/* Synopsys' "HPD workaround" */
	ctrl_wr(p, RA_PDEC_ERR_FILTER, 0x1ff);
}

/*
 * HDMI20_HAL_Rx_LinkVideoReset: main (0xffff) or soft reset of the link, then the
 * configuration it wipes, HPD restored. HDCP and audio setup are not done yet.
 */
static void hdmirx_link_reset(struct hdmirx_port *p, bool main)
{
	u32 hpd = ctrl_rd(p, RA_HDMI_SETUP_CTRL);
	u32 mute_b0 = ctrl_rd(p, RA_LOCAL_MUTE0), mute_b4 = ctrl_rd(p, RA_LOCAL_MUTE1);

	ctrl_wr(p, RA_DMI_SW_RST, main ? 0xffff : LG_FF0_RESET_0);
	ctrl_wr(p, RA_LOCAL_MUTE0, mute_b0);
	ctrl_wr(p, RA_LOCAL_MUTE1, mute_b4);
	ctrl_wr(p, RA_HDCP_BCAPS, LG_HDCP_BCAPS);	/* HDCP 1.4 DDC at 100 kHz */
	hdmirx_ctrl_config(p);
	ctrl_wr(p, RA_HDMI_MODE_RECOVER, LG_MODE_RECOVER);
	ctrl_wr(p, RA_VERTICAL_TIMING, LG_VERTICAL_TIMING);
	ctrl_wr(p, RA_HA_THRESHOLD, LG_HA_THRESHOLD);
	ctrl_wr(p, RA_HDMI_SYNC_CTRL, 0x14);		/* VS/HS polarity auto */
	/* no HDCP keys written: stock's "test mode" values */
	ctrl_wr(p, RA_HDCP_CTRL, 0x01000000);
	ctrl_wr(p, RA_HDCP_OVERRIDE, 0x2);
	ctrl_wr(p, RA_GCP_CTRL, LG_GCP_CTRL);		/* deep colour auto detect */
	ctrl_wr(p, RA_HDMI_SETUP_CTRL, hpd);
	ctrl_wr(p, RA_HDMI_ICLR, VS_POL_ADJ_ISTS | HS_POL_ADJ_ISTS);
	/* AV mute on, T7 HDR fix, reserved packet parsing (header 0x7f) */
	ctrl_wr(p, RA_PDEC_CTRL, (ctrl_rd(p, RA_PDEC_CTRL) | BIT(15) | BIT(30)) & ~BIT(5));
	ctrl_wr(p, RA_PDEC_RSVD_HDR, 0x7f);
	/* SCDC */
	ctrl_wr(p, RA_SCDC_CONFIG, LG_SCDC_CONFIG);
	ctrl_wr(p, RA_SCDC_MANUF, LG_SCDC_MANUF);
	ctrl_wr(p, RA_SCDC_808, 0x1);
	ctrl_wr(p, RA_SCDC_80C, LG_SCDC_80C);
}

/* hdmi_rx_ctrl_open (interrupts off) + HDMI20_HAL_Rx_Initialize */
static void hdmirx_ctrl_init(struct hdmirx_port *p)
{
	static const u32 ien_clr[] = { RA_PDEC_IEN_CLR, RA_AUD_CLK_IEN_CLR, RA_AUD_FIFO_IEN_CLR,
				       RA_MD_IEN_CLR, RA_HDMI_IEN_CLR, RA_HDCP22_IEN_CLR };
	static const u32 iclr[] = { RA_PDEC_ICLR, RA_AUD_CLK_ICLR, RA_AUD_FIFO_ICLR,
				    RA_MD_ICLR, RA_HDMI_ICLR, RA_HDCP22_ICLR };
	int i;

	for (i = 0; i < ARRAY_SIZE(ien_clr); i++)
		ctrl_wr(p, ien_clr[i], ~0);
	for (i = 0; i < ARRAY_SIZE(iclr); i++)
		ctrl_wr(p, iclr[i], ~0);
	hdmirx_link_reset(p, true);
	top_wr(p, TOP_ACR_PLL_MODE, LG_ACR_PLL_MODE);
}

/* HDMI20_Open's per-port part */
static void hdmirx_top_init(struct hdmirx_port *p)
{
	top_wr(p, TOP_MAIN_I2C_DELAY, 1);
	top_wr(p, TOP_MAIN_PVO_CTRL, LG_PVO_MODE);
	top_wr(p, TOP_CLK_MEAS_REF, LG_CLK_MEAS_REF);
	top_rmw(p, TOP_PHY_I2C, TOP_PHY_I2C_PORT, FIELD_PREP(TOP_PHY_I2C_PORT, p->id));
}

/*
 * __HDMI20_HAL_SetVideoColorDomain: how the link hands pixels to the CVI, from
 * the AVI pixel encoding (DVI: RGB).
 */
static void hdmirx_out_format(struct hdmirx_port *p, enum lg1314_hdmirx_format f)
{
	static const u32 fmt[] = { 0x10006, 0x21, 0x21, 0x06 };	/* RGB, 422, 444, 420 */

	u32 mute;

	if (p->out_format == f)
		return;
	top_wr(p, TOP_OUT_422, f == LG1314_HDMIRX_YCBCR422);
	top_wr(p, TOP_OUT_FORMAT, fmt[f]);
	p->out_format = f;
	/* HDMI20_HAL_Rx_SetLocalVideoMuteColor: black in that format */
	mute = ctrl_rd(p, RA_LOCAL_MUTE1) & LOCAL_MUTE;
	if (f == LG1314_HDMIRX_YCBCR422 || f == LG1314_HDMIRX_YCBCR444)
		mute |= 0x8000;
	ctrl_wr(p, RA_LOCAL_MUTE0, f == LG1314_HDMIRX_RGB ? 0 : 0x8000);
	ctrl_wr(p, RA_LOCAL_MUTE1, mute);
}

static int hdmirx_measure(struct hdmirx_port *p, struct v4l2_dv_timings *t);

/*
 * Follow what the port receives. The measurement must repeat on two polls
 * before it is reported, so a source changing mode doesn't show its
 * intermediate states.
 */
static void hdmirx_track_signal(struct hdmirx_port *p)
{
	struct lg1314_hdmirx_signal sig = { };
	u32 key[ARRAY_SIZE(p->sig_key)] = { };
	u32 avi, dcm;

	if (p->locked) {
		avi = ctrl_rd(p, RA_PDEC_AVI_PB);
		sig.dvi = ctrl_rd(p, RA_PDEC_STS) & DVIDET;
		sig.format = sig.dvi ? LG1314_HDMIRX_RGB : FIELD_GET(VIDEO_FORMAT, avi);
		hdmirx_out_format(p, sig.format);

		key[0] = ctrl_rd(p, RA_MD_HT1);
		key[1] = ctrl_rd(p, RA_MD_HACT_PX);
		key[2] = ctrl_rd(p, RA_MD_VTL);
		key[3] = ctrl_rd(p, RA_MD_VAL);
		key[4] = ctrl_rd(p, RA_MD_VTC) >> 10;	/* frame period, coarse */
		key[5] = sig.dvi ? 0 : avi;
		key[6] = ctrl_rd(p, RA_HDMI_STS) & DCM_CURRENT_MODE;
		key[7] = sig.dvi;
	}
	if (memcmp(key, p->sig_key, sizeof(key))) {
		memcpy(p->sig_key, key, sizeof(key));
		p->sig_settled = false;
		return;
	}
	if (p->sig_settled) {
		/* the HDR metadata can change on its own */
		if (p->sig.locked && memcmp(&p->hdr, &p->sig.hdr, sizeof(p->hdr))) {
			p->sig.hdr = p->hdr;
			p->sig_changed = true;
		}
		return;
	}
	p->sig_settled = true;

	if (p->locked && !hdmirx_measure(p, &sig.timings)) {
		sig.locked = true;
		if (!sig.dvi) {
			sig.colorimetry = FIELD_GET(AVI_COLORIMETRY, avi);
			sig.ext_colorimetry = FIELD_GET(AVI_EXT_COLORIMETRY, avi);
			sig.rgb_quant = FIELD_GET(AVI_RGB_QUANT, avi);
			sig.vic = FIELD_GET(VID_IDENT_CODE, avi);
		}
		dcm = FIELD_GET(DCM_CURRENT_MODE, ctrl_rd(p, RA_HDMI_STS));
		sig.bpc = dcm == 5 ? 10 : dcm == 6 ? 12 : dcm == 7 ? 16 : 8;
		sig.hdr = p->hdr;
	}
	if (memcmp(&sig, &p->sig, sizeof(sig))) {
		p->sig = sig;
		p->sig_changed = true;
	}
}

static int hdmirx_port_connector(struct lg1314_hdmirx *rx, unsigned int port)
{
	int k;

	for (k = 0; k < LG1314_HDMIRX_CONNECTORS; k++)
		if (rx->connector_port[k] == port)
			return k + 1;
	return 0;
}

static void hdmirx_report_connector(struct lg1314_hdmirx *rx, int k)
{
	char env[16], attr[8], *envp[] = { env, NULL };

	snprintf(env, sizeof(env), "HDMI=%d", k);
	blocking_notifier_call_chain(&rx->notifier, k, NULL);
	kobject_uevent_env(&rx->dev->kobj, KOBJ_CHANGE, envp);
	/* for poll(POLLPRI) on the status file */
	snprintf(attr, sizeof(attr), "hdmi%d", k);
	sysfs_notify(&rx->dev->kobj, NULL, attr);
}

/* Part i of a split picture: the port of connector HDMI i + 1. */
static bool hdmirx_split_part(struct lg1314_hdmirx *rx, unsigned int port)
{
	unsigned int k = rx->split_conn, i;

	for (i = 0; k && i < rx->split[k - 1]; i++)
		if (rx->connector_port[i] == port)
			return true;
	return false;
}

static void hdmirx_report(struct hdmirx_port *p)
{
	struct lg1314_hdmirx *rx = p->rx;
	int k = hdmirx_port_connector(rx, p->id);

	if (k)
		hdmirx_report_connector(rx, k);
	if (rx->split_conn && rx->split_conn != k && hdmirx_split_part(rx, p->id))
		hdmirx_report_connector(rx, rx->split_conn);
}

/*
 * Over how many receiver ports the switch spreads connector @k's picture: the
 * switch's full width over the width the first part's port measures, when that
 * is 2 or 4 (the switch's own split report misses some modes); else its report.
 */
static unsigned int hdmirx_connector_split(struct lg1314_hdmirx *rx, unsigned int k)
{
	int port = rx->connector_port[k - 1], first = rx->connector_port[0];
	unsigned int full, part = 0, n;
	struct hdmirx_port *p;

	if (!rx->sw || port < 0)
		return 1;
	full = lg_hawk2_width(rx->sw, port);
	if (first >= 0) {
		p = &rx->port[first];
		mutex_lock(&p->lock);
		if (p->sig.locked)
			part = p->sig.timings.bt.width;
		mutex_unlock(&p->lock);
	}
	if (full && part && (full == 2 * part || full == 4 * part))
		return full / part;
	if (full && part && full == part)
		return 1;
	n = lg_hawk2_split(rx->sw, port);
	return n <= LG1314_HDMIRX_CONNECTORS ? n : 1;
}

/*
 * Follow the switch's split of each connector's picture. A change is reported for
 * the connector and for those whose ports the split borrows (their status).
 */
static void hdmirx_track_split(struct lg1314_hdmirx *rx)
{
	unsigned int k, i, n, owner = 0, old_owner = rx->split_conn;
	unsigned int old_n = old_owner ? rx->split[old_owner - 1] : 1;
	bool changed = false;

	for (k = 1; k <= LG1314_HDMIRX_CONNECTORS; k++) {
		n = hdmirx_connector_split(rx, k);
		if (n > 1 && !owner)
			owner = k;
		if (n != rx->split[k - 1]) {
			dev_info(rx->dev, "HDMI %u: picture over %u receiver port%s\n", k, n,
				 n > 1 ? "s" : "");
			WRITE_ONCE(rx->split[k - 1], n);
			changed = true;
		}
	}
	WRITE_ONCE(rx->split_conn, owner);
	if (!changed && owner == old_owner)
		return;
	n = max(old_n, owner ? rx->split[owner - 1] : 1);
	for (i = 0; i < n; i++)
		hdmirx_report_connector(rx, i + 1);
	if (old_owner > n)
		hdmirx_report_connector(rx, old_owner);
	if (owner > n && owner != old_owner)
		hdmirx_report_connector(rx, owner);
}

static void hdmirx_report_audio(struct hdmirx_port *p)
{
	int k = hdmirx_port_connector(p->rx, p->id);
	char attr[8];

	if (!k)
		return;
	blocking_notifier_call_chain(&p->rx->notifier, k, NULL);
	snprintf(attr, sizeof(attr), "audio%d", k);
	sysfs_notify(&p->rx->dev->kobj, NULL, attr);
}

static void hdmirx_poll_port(struct hdmirx_port *p)
{
	bool changed, aud_changed;

	bool cable = top_rd(p, TOP_5V) & TOP_5V_DET;

	mutex_lock(&p->lock);
	if (cable != p->cable) {
		dev_info(p->rx->dev, "port %u: source 5V %s\n", p->id, cable ? "on" : "off");
		p->cable = cable;
		p->sig_changed = true;	/* the status file says "disconnected" or not */
		if (cable) {
			hdmirx_phy_power(p, true);
			p->phy_up = !hdmirx_phy_init(p);
		} else {
			hdmirx_set_hpd(p, false);
			ctrl_wr(p, RA_DMI_SW_RST, 0xffff);
			hdmirx_phy_power(p, false);
			p->phy_up = false;
			p->locked = false;
		}
	}
	if (p->cable && !p->hpd && p->edid_blocks)
		hdmirx_set_hpd(p, true);
	if (p->cable && !p->phy_up)
		p->phy_up = !hdmirx_phy_init(p);
	else if (p->cable)
		hdmirx_phy_update(p);
	if (!p->locked)
		p->out_format = -1;
	hdmirx_hdr_poll(p);
	hdmirx_track_signal(p);
	hdmirx_audio_poll(p);
	if (p->muted && p->sig.locked &&
	    time_after_eq(jiffies, p->mute_since + msecs_to_jiffies(LOCAL_MUTE_HOLD_MS)))
		hdmirx_mute(p, false);
	changed = p->sig_changed;
	p->sig_changed = false;
	aud_changed = p->aud_changed;
	p->aud_changed = false;
	mutex_unlock(&p->lock);

	if (changed)
		hdmirx_report(p);
	if (aud_changed)
		hdmirx_report_audio(p);
}

static void hdmirx_poll(struct work_struct *work)
{
	struct lg1314_hdmirx *rx = container_of(work, struct lg1314_hdmirx, poll.work);
	int i;

	if (!rx->started) {
		/* end of the HPD-low period: isolation off (stock does it again here) */
		gpiod_set_value_cansleep(rx->isol, 1);
		/*
		 * Stock's ENABLE_OPERATION, after the isolation release: "ALED HPD set
		 * up" on every port. Without it the HDMI switch sees no HPD from the
		 * receivers and sends them nothing. The bit survives a watchdog reset,
		 * not a power cycle.
		 */
		for (i = 0; i < HDMIRX_PORTS; i++) {
			void __iomem *r = rx->aled_port + ALED_PORT_SIZE * i + ALED_AD_HPD;

			writel(readl(r) | ALED_AD_HPD_SET_HPD, r);
		}
		rx->started = true;
	}
	/* stock's watchdog on the ALED I2C clock select */
	if (readl(rx->aled + ALED_I2C_CLK_SEL) != 6)
		writel(6, rx->aled + ALED_I2C_CLK_SEL);
	for (i = 0; i < HDMIRX_PORTS; i++)
		hdmirx_poll_port(&rx->port[i]);
	hdmirx_track_split(rx);
	schedule_delayed_work(&rx->poll, msecs_to_jiffies(HDMIRX_POLL_MS));
}

/* EDID: up to 4 blocks in the top block's SRAM; HPD low while it changes. */
static int hdmirx_set_edid_locked(struct hdmirx_port *p, const u8 *edid, unsigned int blocks)
{
	if (blocks > TOP_EDID_SIZE / 128)
		return -E2BIG;
	if (p->hpd) {
		hdmirx_set_hpd(p, false);
		msleep(100);
	}
	memset(p->edid, 0, sizeof(p->edid));
	memcpy(p->edid, edid, blocks * 128);
	p->edid_blocks = blocks;
	hdmirx_write_edid(p);
	return 0;
}

static struct hdmirx_port *to_port(struct v4l2_subdev *sd)
{
	return container_of(sd, struct hdmirx_port, sd);
}

static int hdmirx_get_edid(struct v4l2_subdev *sd, struct v4l2_edid *edid)
{
	struct hdmirx_port *p = to_port(sd);
	int ret = 0;

	memset(edid->reserved, 0, sizeof(edid->reserved));
	if (edid->pad)
		return -EINVAL;
	mutex_lock(&p->lock);
	if (edid->start_block == 0 && edid->blocks == 0) {
		edid->blocks = p->edid_blocks;
		goto out;
	}
	if (!p->edid_blocks) {
		ret = -ENODATA;
		goto out;
	}
	if (edid->start_block >= p->edid_blocks) {
		ret = -EINVAL;
		goto out;
	}
	edid->blocks = min(edid->blocks, p->edid_blocks - edid->start_block);
	memcpy(edid->edid, p->edid + edid->start_block * 128, edid->blocks * 128);
out:
	mutex_unlock(&p->lock);
	return ret;
}

static int hdmirx_set_edid(struct v4l2_subdev *sd, struct v4l2_edid *edid)
{
	struct hdmirx_port *p = to_port(sd);
	int ret;

	memset(edid->reserved, 0, sizeof(edid->reserved));
	if (edid->pad || edid->start_block)
		return -EINVAL;
	if (edid->blocks > TOP_EDID_SIZE / 128) {
		edid->blocks = TOP_EDID_SIZE / 128;
		return -E2BIG;
	}
	mutex_lock(&p->lock);
	ret = hdmirx_set_edid_locked(p, edid->edid, edid->blocks);
	mutex_unlock(&p->lock);
	return ret;
}

static int hdmirx_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct hdmirx_port *p = to_port(sd);

	mutex_lock(&p->lock);
	if (!p->cable)
		*status = V4L2_IN_ST_NO_POWER;
	else if (!p->locked)
		*status = V4L2_IN_ST_NO_SIGNAL;
	else
		*status = 0;
	mutex_unlock(&p->lock);
	return 0;
}

static const struct v4l2_dv_timings_cap hdmirx_timings_cap = {
	.type = V4L2_DV_BT_656_1120,
	/* HDMI 2.0 receiver: up to 4K60 (594 MHz) */
	V4L2_INIT_BT_TIMINGS(640, 4096, 480, 2160, 25000000, 600000000,
			     V4L2_DV_BT_STD_CEA861 | V4L2_DV_BT_STD_DMT |
			     V4L2_DV_BT_STD_CVT | V4L2_DV_BT_STD_GTF,
			     V4L2_DV_BT_CAP_PROGRESSIVE | V4L2_DV_BT_CAP_INTERLACED |
			     V4L2_DV_BT_CAP_REDUCED_BLANKING | V4L2_DV_BT_CAP_CUSTOM)
};

/*
 * hdmi_rx_ctrl_get_video: the mode detector counts TMDS characters, so horizontal
 * values are scaled by the deep colour factor (and doubled for 4:2:0). Prefer the
 * AVI VIC's CEA-861 timings when they match the measurement.
 */
static int hdmirx_measure(struct hdmirx_port *p, struct v4l2_dv_timings *t)
{
	struct v4l2_bt_timings *bt = &t->bt;
	u32 ht1 = ctrl_rd(p, RA_MD_HT1), vtc = ctrl_rd(p, RA_MD_VTC);
	u32 avi = ctrl_rd(p, RA_PDEC_AVI_PB);
	bool dvi = ctrl_rd(p, RA_PDEC_STS) & DVIDET;
	bool il = ctrl_rd(p, RA_MD_STS) & ILACE;
	u32 htot, hofs, hact, hsync, vtot, vfp, vact, vsync, ists, num = 100, den = 100;
	struct v4l2_dv_timings cea;
	u8 vic = dvi ? 0 : FIELD_GET(VID_IDENT_CODE, avi);

	switch (FIELD_GET(DCM_CURRENT_MODE, ctrl_rd(p, RA_HDMI_STS))) {
	case 5:
		den = 125;
		break;
	case 6:
		den = 150;
		break;
	case 7:
		den = 200;
		break;
	}
	if (!dvi && FIELD_GET(VIDEO_FORMAT, avi) == 3)
		num *= 2;
	htot = FIELD_GET(HTOT_PIX, ht1) * num / den;
	hofs = FIELD_GET(HOFS_PIX, ht1) * num / den;
	hact = (ctrl_rd(p, RA_MD_HACT_PX) & 0xffff) * num / den;
	vtot = ctrl_rd(p, RA_MD_VTL) & 0xffff;
	vfp = ctrl_rd(p, RA_MD_VOL) & 0xffff;
	vact = ctrl_rd(p, RA_MD_VAL) & 0xffff;
	if (il) {
		vtot <<= 1;
		vfp <<= 1;
		vact <<= 1;
		vtc <<= 1;
	}
	if (!htot || !vtot || !vtc || !hact || !vact || hact + hofs > htot || vact + vfp > vtot)
		return -ENOLCK;

	if (vic && v4l2_find_dv_timings_cea861_vic(&cea, vic) &&
	    cea.bt.width == hact && cea.bt.height == vact &&
	    !!cea.bt.interlaced == il) {
		*t = cea;
		return 0;
	}

	memset(t, 0, sizeof(*t));
	t->type = V4L2_DV_BT_656_1120;
	bt->width = hact;
	bt->height = vact;
	bt->interlaced = il ? V4L2_DV_INTERLACED : V4L2_DV_PROGRESSIVE;
	bt->pixelclock = div_u64((u64)htot * vtot * MD_CLK_KHZ * 1000, vtc);
	ists = ctrl_rd(p, RA_HDMI_ISTS);
	if (ists & VS_POL_ADJ_ISTS)
		bt->polarities |= V4L2_DV_VSYNC_POS_POL;
	if (ists & HS_POL_ADJ_ISTS)
		bt->polarities |= V4L2_DV_HSYNC_POS_POL;
	/* HOFS = hsync + back porch; the sync widths are measured in md_clk cycles */
	hsync = DIV_ROUND_CLOSEST_ULL((u64)FIELD_GET(HS_CLK, ctrl_rd(p, RA_MD_HT0)) *
				      bt->pixelclock, MD_CLK_KHZ * 1000);
	hsync = min(hsync, hofs);
	bt->hsync = hsync;
	bt->hbackporch = hofs - hsync;
	bt->hfrontporch = htot - hact - hofs;
	vsync = DIV_ROUND_CLOSEST_ULL((u64)ctrl_rd(p, RA_MD_VSC) * vtot, vtc);
	vsync = min(vsync, vtot - vact - vfp);
	bt->vsync = vsync;
	bt->vfrontporch = vfp;
	bt->vbackporch = vtot - vact - vfp - vsync;
	return 0;
}

static int hdmirx_query_dv_timings(struct v4l2_subdev *sd, unsigned int pad,
				   struct v4l2_dv_timings *t)
{
	struct hdmirx_port *p = to_port(sd);
	int ret;

	if (pad)
		return -EINVAL;
	mutex_lock(&p->lock);
	ret = p->locked ? hdmirx_measure(p, t) : -ENOLINK;
	mutex_unlock(&p->lock);
	return ret;
}

static int hdmirx_dv_timings_cap(struct v4l2_subdev *sd, struct v4l2_dv_timings_cap *cap)
{
	if (cap->pad)
		return -EINVAL;
	*cap = hdmirx_timings_cap;
	return 0;
}

static const struct v4l2_subdev_video_ops hdmirx_video_ops = {
	.g_input_status = hdmirx_g_input_status,
};

static const struct v4l2_subdev_pad_ops hdmirx_pad_ops = {
	.get_edid = hdmirx_get_edid,
	.set_edid = hdmirx_set_edid,
	.query_dv_timings = hdmirx_query_dv_timings,
	.dv_timings_cap = hdmirx_dv_timings_cap,
};

static const struct v4l2_subdev_ops hdmirx_sd_ops = {
	.video = &hdmirx_video_ops,
	.pad = &hdmirx_pad_ops,
};

static void hdmirx_load_default_edid(struct lg1314_hdmirx *rx)
{
	const struct firmware *fw;
	unsigned int i;

	if (firmware_request_nowarn(&fw, HDMIRX_EDID_FW, rx->dev))
		return;
	if (!fw->size || fw->size % 128 || fw->size > TOP_EDID_SIZE) {
		dev_warn(rx->dev, "%s: bad size %zu\n", HDMIRX_EDID_FW, fw->size);
	} else {
		for (i = 0; i < HDMIRX_PORTS; i++) {
			mutex_lock(&rx->port[i].lock);
			hdmirx_set_edid_locked(&rx->port[i], fw->data, fw->size / 128);
			mutex_unlock(&rx->port[i].lock);
		}
		dev_info(rx->dev, "default EDID: %zu blocks\n", fw->size / 128);
	}
	release_firmware(fw);
}

static void __iomem *hdmirx_map_word(struct platform_device *pdev, const char *name)
{
	struct resource *res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);

	/* single registers in pages other drivers own: map, don't request */
	if (!res)
		return IOMEM_ERR_PTR(-ENODEV);
	return devm_ioremap(&pdev->dev, res->start, resource_size(res)) ?:
	       IOMEM_ERR_PTR(-ENOMEM);
}

/* ---- interface for the display driver ---- */

struct lg1314_hdmirx *lg1314_hdmirx_get(struct device_node *np)
{
	struct platform_device *pdev = of_find_device_by_node(np);
	struct lg1314_hdmirx *rx;

	if (!pdev)
		return ERR_PTR(-EPROBE_DEFER);
	rx = platform_get_drvdata(pdev);
	if (!rx || !try_module_get(THIS_MODULE)) {
		put_device(&pdev->dev);
		return ERR_PTR(-EPROBE_DEFER);
	}
	return rx;
}
EXPORT_SYMBOL_GPL(lg1314_hdmirx_get);

void lg1314_hdmirx_put(struct lg1314_hdmirx *rx)
{
	put_device(rx->dev);
	module_put(THIS_MODULE);
}
EXPORT_SYMBOL_GPL(lg1314_hdmirx_put);

int lg1314_hdmirx_connector_port(struct lg1314_hdmirx *rx, unsigned int connector)
{
	if (!connector || connector > LG1314_HDMIRX_CONNECTORS ||
	    rx->connector_port[connector - 1] < 0)
		return -EINVAL;
	return rx->connector_port[connector - 1];
}
EXPORT_SYMBOL_GPL(lg1314_hdmirx_connector_port);

void lg1314_hdmirx_select(struct lg1314_hdmirx *rx, unsigned int connector)
{
	int port = lg1314_hdmirx_connector_port(rx, connector);

	if (rx->sw)
		lg_hawk2_select(rx->sw, port < 0 ? -1 : port);
	if (connector != READ_ONCE(rx->selected)) {
		WRITE_ONCE(rx->selected, port < 0 ? 0 : connector);
		blocking_notifier_call_chain(&rx->notifier, connector, NULL);
	}
}
EXPORT_SYMBOL_GPL(lg1314_hdmirx_select);

/*
 * The audio of the selected connector: its receiver port when that has audio and
 * outputs it (unmuted), else -ENODATA.
 */
int lg1314_hdmirx_selected_audio(struct lg1314_hdmirx *rx, struct lg1314_hdmirx_audio *aud)
{
	unsigned int k = READ_ONCE(rx->selected);
	int port = lg1314_hdmirx_connector_port(rx, k);
	struct hdmirx_port *p;
	bool live;

	if (port < 0)
		return -ENODATA;
	/* split: the audio comes with the first part (API_VIDEO_HDMI_GetAudioPortInfo) */
	if (READ_ONCE(rx->split[k - 1]) > 1)
		port = rx->connector_port[0];
	p = &rx->port[port];
	mutex_lock(&p->lock);
	*aud = p->aud;
	live = p->aud.type && !p->aud_muted;
	mutex_unlock(&p->lock);
	return live ? port : -ENODATA;
}
EXPORT_SYMBOL_GPL(lg1314_hdmirx_selected_audio);

static void hdmirx_port_signal(struct hdmirx_port *p, struct lg1314_hdmirx_signal *sig,
			       bool *cable)
{
	mutex_lock(&p->lock);
	*sig = p->sig;
	*cable = p->cable;
	mutex_unlock(&p->lock);
}

/*
 * What connector @k shows: its port's signal, or, while the switch splits its
 * picture over n ports, the picture put together from them (each part must be
 * locked with the same size and format; the rest is the first part's). A port borrowed by another
 * connector's split reports no cable.
 */
static int hdmirx_connector_signal(struct lg1314_hdmirx *rx, unsigned int k,
				   struct lg1314_hdmirx_signal *sig, bool *cable)
{
	int port = lg1314_hdmirx_connector_port(rx, k);
	unsigned int owner = READ_ONCE(rx->split_conn), n, i;
	struct v4l2_bt_timings *bt = &sig->timings.bt;
	struct lg1314_hdmirx_signal part;
	bool part_cable;

	if (port < 0)
		return port;
	n = READ_ONCE(rx->split[k - 1]);
	if (n <= 1) {
		if (owner && owner != k && k <= READ_ONCE(rx->split[owner - 1])) {
			memset(sig, 0, sizeof(*sig));
			*cable = false;
			return 0;
		}
		hdmirx_port_signal(&rx->port[port], sig, cable);
		return 0;
	}

	hdmirx_port_signal(&rx->port[rx->connector_port[0]], sig, cable);
	for (i = 1; sig->locked && i < n; i++) {
		hdmirx_port_signal(&rx->port[rx->connector_port[i]], &part, &part_cable);
		/* blanking is measured per port (+-2 px): compare the picture only */
		if (!part.locked || part.format != sig->format || part.bpc != sig->bpc ||
		    part.timings.bt.width != bt->width ||
		    part.timings.bt.height != bt->height ||
		    part.timings.bt.interlaced != bt->interlaced)
			sig->locked = false;
	}
	if (!sig->locked) {
		memset(sig, 0, sizeof(*sig));
		return 0;
	}
	bt->width *= n;
	bt->hfrontporch *= n;
	bt->hsync *= n;
	bt->hbackporch *= n;
	bt->pixelclock *= n;
	sig->split = n;
	return 0;
}

int lg1314_hdmirx_get_signal(struct lg1314_hdmirx *rx, unsigned int connector,
			     struct lg1314_hdmirx_signal *sig)
{
	bool cable;

	return hdmirx_connector_signal(rx, connector, sig, &cable);
}
EXPORT_SYMBOL_GPL(lg1314_hdmirx_get_signal);

/* The receiver port carrying part @i (from the left) of a split picture. */
int lg1314_hdmirx_split_port(struct lg1314_hdmirx *rx, unsigned int i)
{
	if (i >= LG1314_HDMIRX_CONNECTORS)
		return -EINVAL;
	return rx->connector_port[i];
}
EXPORT_SYMBOL_GPL(lg1314_hdmirx_split_port);

int lg1314_hdmirx_register_notifier(struct lg1314_hdmirx *rx, struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&rx->notifier, nb);
}
EXPORT_SYMBOL_GPL(lg1314_hdmirx_register_notifier);

void lg1314_hdmirx_unregister_notifier(struct lg1314_hdmirx *rx, struct notifier_block *nb)
{
	blocking_notifier_chain_unregister(&rx->notifier, nb);
}
EXPORT_SYMBOL_GPL(lg1314_hdmirx_unregister_notifier);

/*
 * sysfs hdmi1..hdmi4: "disconnected", "no signal", or e.g.
 * "1920x1080p60.00 ycbcr444 bt709 rgb-default 8bit" (a uevent HDMI=<k> follows changes)
 */
static ssize_t hdmirx_status_show(struct device *dev, struct device_attribute *attr,
				  char *buf)
{
	static const char * const fmt[] = { "rgb", "ycbcr422", "ycbcr444", "ycbcr420" };
	static const char * const col[] = { "none", "bt601", "bt709", "ext" };
	static const char * const quant[] = { "default", "limited", "full", "reserved" };
	struct lg1314_hdmirx *rx = dev_get_drvdata(dev);
	unsigned int k = attr->attr.name[4] - '0';
	struct lg1314_hdmirx_signal sig;
	const struct v4l2_bt_timings *bt = &sig.timings.bt;
	u64 tot, mhz;
	bool cable;

	if (hdmirx_connector_signal(rx, k, &sig, &cable))
		return -ENODEV;
	if (!cable)
		return sysfs_emit(buf, "disconnected\n");
	if (!sig.locked)
		return sysfs_emit(buf, "no signal\n");
	tot = (u64)V4L2_DV_BT_FRAME_WIDTH(bt) * V4L2_DV_BT_FRAME_HEIGHT(bt);
	mhz = tot ? div64_u64(bt->pixelclock * 1000, tot) : 0;
	if (sig.dvi)
		return sysfs_emit(buf, "%ux%u%c%llu.%02llu dvi\n", bt->width, bt->height,
				  bt->interlaced ? 'i' : 'p', mhz / 1000, mhz % 1000 / 10);
	if (sig.colorimetry == 3)
		return sysfs_emit(buf, "%ux%u%c%llu.%02llu %s ext%u %s %ubit\n", bt->width,
				  bt->height, bt->interlaced ? 'i' : 'p', mhz / 1000,
				  mhz % 1000 / 10, fmt[sig.format], sig.ext_colorimetry,
				  quant[sig.rgb_quant], sig.bpc);
	return sysfs_emit(buf, "%ux%u%c%llu.%02llu %s %s %s %ubit\n", bt->width, bt->height,
			  bt->interlaced ? 'i' : 'p', mhz / 1000, mhz % 1000 / 10,
			  fmt[sig.format], col[sig.colorimetry], quant[sig.rgb_quant], sig.bpc);
}

/* sysfs audio1..audio4: "none", or e.g. "pcm 48000 2ch", "compressed 48000", "hbr 192000" */
static ssize_t hdmirx_audio_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	static const char * const type[] = { "none", "pcm", "compressed", "hbr" };
	struct lg1314_hdmirx *rx = dev_get_drvdata(dev);
	struct lg1314_hdmirx_audio aud;
	unsigned int k = attr->attr.name[5] - '0';
	int port = lg1314_hdmirx_connector_port(rx, k);

	if (port < 0)
		return -ENODEV;
	mutex_lock(&rx->port[port].lock);
	aud = rx->port[port].aud;
	mutex_unlock(&rx->port[port].lock);
	if (aud.type == LG1314_HDMIRX_AUDIO_PCM)
		return sysfs_emit(buf, "pcm %u %uch\n", aud.rate, aud.channels);
	if (aud.type)
		return sysfs_emit(buf, "%s %u\n", type[aud.type], aud.rate);
	return sysfs_emit(buf, "none\n");
}

static DEVICE_ATTR(hdmi1, 0444, hdmirx_status_show, NULL);
static DEVICE_ATTR(hdmi2, 0444, hdmirx_status_show, NULL);
static DEVICE_ATTR(hdmi3, 0444, hdmirx_status_show, NULL);
static DEVICE_ATTR(hdmi4, 0444, hdmirx_status_show, NULL);
static DEVICE_ATTR(audio1, 0444, hdmirx_audio_show, NULL);
static DEVICE_ATTR(audio2, 0444, hdmirx_audio_show, NULL);
static DEVICE_ATTR(audio3, 0444, hdmirx_audio_show, NULL);
static DEVICE_ATTR(audio4, 0444, hdmirx_audio_show, NULL);

static struct attribute *hdmirx_attrs[] = {
	&dev_attr_hdmi1.attr, &dev_attr_hdmi2.attr,
	&dev_attr_hdmi3.attr, &dev_attr_hdmi4.attr,
	&dev_attr_audio1.attr, &dev_attr_audio2.attr,
	&dev_attr_audio3.attr, &dev_attr_audio4.attr, NULL
};

static const struct attribute_group hdmirx_attr_group = {
	.attrs = hdmirx_attrs,
};

static int hdmirx_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np;
	struct lg1314_hdmirx *rx;
	char name[8];
	int i, ret;

	rx = devm_kzalloc(dev, sizeof(*rx), GFP_KERNEL);
	if (!rx)
		return -ENOMEM;
	rx->dev = dev;
	BLOCKING_INIT_NOTIFIER_HEAD(&rx->notifier);

	rx->aled_port = devm_platform_ioremap_resource_byname(pdev, "aled-ports");
	if (IS_ERR(rx->aled_port))
		return PTR_ERR(rx->aled_port);
	rx->aled = devm_platform_ioremap_resource_byname(pdev, "aled");
	if (IS_ERR(rx->aled))
		return PTR_ERR(rx->aled);
	rx->ddc = hdmirx_map_word(pdev, "ddc");
	if (IS_ERR(rx->ddc))
		return PTR_ERR(rx->ddc);
	rx->gate = hdmirx_map_word(pdev, "gate");
	if (IS_ERR(rx->gate))
		return PTR_ERR(rx->gate);
	/* isolation held until the HPD-low period is over */
	rx->isol = devm_gpiod_get(dev, "isolation", GPIOD_OUT_LOW);
	if (IS_ERR(rx->isol))
		return dev_err_probe(dev, PTR_ERR(rx->isol), "isolation gpio\n");
	np = of_parse_phandle(dev->of_node, "lge,phy-i2c-bus", 0);
	if (!np)
		return dev_err_probe(dev, -EINVAL, "no lge,phy-i2c-bus\n");
	rx->phy_i2c = of_get_i2c_adapter_by_node(np);
	of_node_put(np);
	if (!rx->phy_i2c)
		return -EPROBE_DEFER;

	/* connectors: through the HDMI switch, else HDMI k = port k - 1 */
	np = of_parse_phandle(dev->of_node, "lge,hdmi-switch", 0);
	if (np) {
		rx->sw = lg_hawk2_get(np);
		of_node_put(np);
		if (IS_ERR(rx->sw)) {
			ret = PTR_ERR(rx->sw);
			rx->sw = NULL;
			goto err_i2c;
		}
	}
	for (i = 0; i < LG1314_HDMIRX_CONNECTORS; i++) {
		rx->connector_port[i] = rx->sw ? lg_hawk2_connector_port(rx->sw, i + 1) : i;
		rx->split[i] = 1;
	}

	for (i = 0; i < HDMIRX_PORTS; i++) {
		struct hdmirx_port *p = &rx->port[i];

		p->rx = rx;
		p->id = i;
		p->out_format = -1;
		mutex_init(&p->lock);
		snprintf(name, sizeof(name), "top%d", i);
		p->top = devm_platform_ioremap_resource_byname(pdev, name);
		if (IS_ERR(p->top)) {
			ret = PTR_ERR(p->top);
			goto err_i2c;
		}
		snprintf(name, sizeof(name), "ctrl%d", i);
		p->ctrl = devm_platform_ioremap_resource_byname(pdev, name);
		if (IS_ERR(p->ctrl)) {
			ret = PTR_ERR(p->ctrl);
			goto err_i2c;
		}
	}

	/* HDMI20_Open: DDC pins, HDCP 2.2 gating, ALED I2C clock, per-port top */
	writel(readl(rx->ddc) | DDC_GPIO_OFF, rx->ddc);
	writel(readl(rx->gate) | HDCP22_GATE_OFF, rx->gate);
	writel(6, rx->aled + ALED_I2C_CLK_SEL);
	for (i = 0; i < HDMIRX_PORTS; i++) {
		hdmirx_top_init(&rx->port[i]);
		hdmirx_ctrl_init(&rx->port[i]);
		hdmirx_set_hpd(&rx->port[i], false);
		memset_io(rx->port[i].top + TOP_EDID, 0, TOP_EDID_SIZE);
	}

	ret = v4l2_device_register(dev, &rx->v4l2);
	if (ret)
		goto err_i2c;
	for (i = 0; i < HDMIRX_PORTS; i++) {
		struct v4l2_subdev *sd = &rx->port[i].sd;

		v4l2_subdev_init(sd, &hdmirx_sd_ops);
		sd->owner = THIS_MODULE;
		sd->dev = dev;
		sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
		snprintf(sd->name, sizeof(sd->name), "lg1314-hdmirx %d", i);
		ret = v4l2_device_register_subdev(&rx->v4l2, sd);
		if (ret)
			goto err_v4l2;
	}
	ret = v4l2_device_register_subdev_nodes(&rx->v4l2);
	if (ret)
		goto err_v4l2;

	hdmirx_load_default_edid(rx);
	platform_set_drvdata(pdev, rx);
	ret = devm_device_add_group(dev, &hdmirx_attr_group);
	if (ret)
		goto err_v4l2;
	INIT_DELAYED_WORK(&rx->poll, hdmirx_poll);
	schedule_delayed_work(&rx->poll, msecs_to_jiffies(HDMIRX_HPD_LOW_MS));
	return 0;

err_v4l2:
	v4l2_device_unregister(&rx->v4l2);
err_i2c:
	if (rx->sw)
		lg_hawk2_put(rx->sw);
	i2c_put_adapter(rx->phy_i2c);
	return ret;
}

static void hdmirx_remove(struct platform_device *pdev)
{
	struct lg1314_hdmirx *rx = platform_get_drvdata(pdev);
	int i;

	cancel_delayed_work_sync(&rx->poll);
	for (i = 0; i < HDMIRX_PORTS; i++) {
		hdmirx_set_hpd(&rx->port[i], false);
		hdmirx_phy_power(&rx->port[i], false);
	}
	v4l2_device_unregister(&rx->v4l2);
	if (rx->sw)
		lg_hawk2_put(rx->sw);
	i2c_put_adapter(rx->phy_i2c);
}

static const struct of_device_id hdmirx_of_match[] = {
	{ .compatible = "lge,lg1314-hdmirx" },
	{ }
};
MODULE_DEVICE_TABLE(of, hdmirx_of_match);

static struct platform_driver hdmirx_driver = {
	.probe = hdmirx_probe,
	.remove = hdmirx_remove,
	.driver = {
		.name = "lg1314-hdmirx",
		.of_match_table = hdmirx_of_match,
	},
};
module_platform_driver(hdmirx_driver);

MODULE_DESCRIPTION("LG1314 HDMI receiver");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE(HDMIRX_EDID_FW);
