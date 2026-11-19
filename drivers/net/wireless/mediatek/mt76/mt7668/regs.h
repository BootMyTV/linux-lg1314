/* SPDX-License-Identifier: ISC */

#ifndef __MT7668_REGS_H
#define __MT7668_REGS_H

#define MT_HW_REV			0x80000000
#define MT_HW_CHIPID			0x80000008

/* firmware/driver handshake */
#define MT_SW_SYNC0			0x81021250
#define MT_SW_SYNC0_PWR_ON		BIT(0)
#define MT_SW_SYNC0_READY		GENMASK(2, 0)

/* USB DMA */
#define MT_UDMA_TX_QSEL			0x50029008
#define MT_FW_DL_EN			BIT(3)

#define MT_UDMA_WLCFG_1			0x5002900c
#define MT_WL_RX_AGG_PKT_LMT		GENMASK(7, 0)

#define MT_UDMA_WLCFG_0			0x50029018
#define MT_WL_TX_EN			BIT(23)
#define MT_WL_RX_EN			BIT(22)
#define MT_WL_RX_AGG_EN			BIT(21)
#define MT_WL_RX_MPSZ_PAD0		BIT(18)
#define MT_WL_RX_AGG_LMT		GENMASK(15, 8)
#define MT_WL_RX_AGG_TO			GENMASK(7, 0)

/* RX descriptor (same layout as MT7615) */
#define MT_RXD0_LENGTH			GENMASK(15, 0)
#define MT_RXD0_NORMAL_GROUP_1		BIT(25)
#define MT_RXD0_NORMAL_GROUP_2		BIT(26)
#define MT_RXD0_NORMAL_GROUP_3		BIT(27)
#define MT_RXD0_NORMAL_GROUP_4		BIT(28)
#define MT_RXD0_PKT_TYPE		GENMASK(31, 29)

/* type 7 is firmware-defined; the subtype tells events from frames */
#define MT_RXD0_SW_PKT_TYPE_MASK	(GENMASK(31, 29) | GENMASK(19, 16))
#define MT_RXD0_SW_PKT_TYPE_EVENT	0xe0000000
#define MT_RXD0_SW_PKT_TYPE_FRAME	0xe0010000

#define MT_RXD1_NORMAL_ADDR_TYPE	GENMASK(2, 1)
#define MT_RXD1_NORMAL_U2M		BIT(1)
#define MT_RXD1_NORMAL_CH_FREQ		GENMASK(15, 8)
#define MT_RXD1_NORMAL_HDR_LEN		GENMASK(21, 16)
#define MT_RXD1_NORMAL_HDR_OFFSET	BIT(22)
#define MT_RXD1_NORMAL_HDR_TRANS	BIT(23)

#define MT_RXD2_NORMAL_WLAN_IDX		GENMASK(7, 0)
#define MT_RXD2_NORMAL_CLM		BIT(19)
#define MT_RXD2_NORMAL_SEC_MODE		GENMASK(15, 12)
#define MT_RXD2_NORMAL_FCS_ERR		BIT(17)
#define MT_RXD2_NORMAL_CM		BIT(18)
#define MT_RXD2_NORMAL_ICV_ERR		BIT(20)
#define MT_RXD2_NORMAL_TKIP_MIC_ERR	BIT(21)
#define MT_RXD2_NORMAL_AMSDU_ERR	BIT(23)
#define MT_RXD2_NORMAL_MAX_LEN_ERROR	BIT(24)
#define MT_RXD2_NORMAL_HDR_TRANS_ERROR	BIT(25)
#define MT_RXD2_NORMAL_FRAG		BIT(27)

enum {
	MT_PHY_BW_20,
	MT_PHY_BW_40,
	MT_PHY_BW_80,
	MT_PHY_BW_160,
};

/* TX descriptor (same layout as MT7615) */
#define MT_TXD0_P_IDX			BIT(31)
#define MT_TXD0_Q_IDX			GENMASK(30, 26)
#define MT_TXD0_ETH_TYPE_OFFSET		GENMASK(22, 16)
#define MT_TXD0_TX_BYTES		GENMASK(15, 0)

#define MT_TXD1_OWN_MAC			GENMASK(31, 26)
#define MT_TXD1_PKT_FMT			GENMASK(25, 24)
#define MT_TXD1_TID			GENMASK(23, 21)
#define MT_TXD1_LONG_FORMAT		BIT(15)
#define MT_TXD1_HDR_FORMAT		GENMASK(14, 13)
#define MT_TXD1_HDR_INFO		GENMASK(12, 8)
#define MT_TXD1_ETH_802_3		BIT(12)
#define MT_TXD1_WLAN_IDX		GENMASK(7, 0)

#define MT_TXD2_FIX_RATE		BIT(31)
#define MT_TXD2_MAX_TX_TIME		GENMASK(23, 16)
#define MT_TXD2_MULTICAST		BIT(10)
#define MT_TXD2_FRAME_TYPE		GENMASK(5, 4)
#define MT_TXD2_SUB_TYPE		GENMASK(3, 0)

#define MT_TXD3_SN_VALID		BIT(31)
#define MT_TXD3_SEQ			GENMASK(27, 16)
#define MT_TXD3_REM_TX_COUNT		GENMASK(15, 11)
/* first bit of a field the firmware reads: send even if the peer is held */
#define MT_TXD3_FORCE_TX		BIT(4)
#define MT_TXD3_PROTECT_FRAME		BIT(1)
#define MT_TXD3_NO_ACK			BIT(0)

#define MT_TXD5_TX_STATUS_MCU		BIT(9)
#define MT_TXD5_PID			GENMASK(7, 0)

#define MT_TXD6_TX_RATE			GENMASK(29, 16)

#define MT_TXD7_SPE_IDX			GENMASK(15, 11)
#define MT_TXD7_SPE_IDX_ALL		0x18	/* both antenna paths */

#define MT_HDR_FORMAT_802_3		0
#define MT_HDR_FORMAT_802_11		2

/* data frames carry a 44-byte header for the CR4 after the TXD */
#define MT_TXP_SIZE			44
#define MT_TXD0_ETH_TYPE_OFFSET_802_3	8

/* LMAC queues for TC0-3 (BK, BE, VI, VO); management goes to the MCU */
#define MT_TX_MCU_PORT_RX_Q1		1
/*
 * Over USB, MCU-port queues 0-3 are sent as 28-31 ("H2C DMA" mapping); the
 * CR4 firmware strips this again.
 */
#define MT_TX_MCU_Q_IDX_H2CDMA		0x1c
#define MT_TX_RETRY_LIMIT		30
#define MT_TX_MGMT_LIFETIME		61	/* ~2 s, in 32.768 ms units */

#define MT_TX_TYPE_CMD			2
#define MT_TX_TYPE_FW			3
#define MT_HDR_FORMAT_CMD		1

#define MT_TX_MCU_PORT_RX_Q0		0
#define MT_TX_MCU_PORT_RX_FWDL		0x1e

#endif
