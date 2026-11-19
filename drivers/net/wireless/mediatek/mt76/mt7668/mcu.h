/* SPDX-License-Identifier: ISC */

#ifndef __MT7668_MCU_H
#define __MT7668_MCU_H

#include "../mt76_connac_mcu.h"

/*
 * The firmware speaks the older (pre-"unified") MediaTek command set. Most
 * IDs match the "CE" commands mt76 uses for connac offload firmware.
 */
enum {
	MT7668_CMD_DUMMY = 0x00,
	MT7668_CMD_BASIC_CONFIG = 0x02,
	MT7668_CMD_SCAN_REQ = 0x03,
	MT7668_CMD_ADD_REMOVE_KEY = 0x07,
	MT7668_CMD_SET_RX_FILTER = 0x0a,
	MT7668_CMD_BSS_ACTIVATE_CTRL = 0x11,
	MT7668_CMD_SET_BSS_INFO = 0x12,
	MT7668_CMD_UPDATE_STA_RECORD = 0x13,
	MT7668_CMD_REMOVE_STA_RECORD = 0x14,
	MT7668_CMD_PM_BSS_CONNECTED = 0x16,
	MT7668_CMD_PM_BSS_ABORT = 0x17,
	MT7668_CMD_SCAN_CANCEL = 0x1b,
	MT7668_CMD_CH_PRIVILEGE = 0x1c,
	MT7668_CMD_GET_NIC_CAPAB = 0x80,
	MT7668_CMD_GET_NIC_CAPAB_V2 = 0x8a,
	MT7668_CMD_FW_LOG_2_HOST = 0xc5,
};

/* extended commands (CID 0xed) */
enum {
	MT7668_EXT_CMD_EFUSE_BUFFER_MODE = 0x21,
};

#define MT7668_CMD(_t)		FIELD_PREP(__MCU_CMD_FIELD_ID, MT7668_CMD_##_t)
#define MT7668_EXT_QUERY(_t)	(FIELD_PREP(__MCU_CMD_FIELD_ID, MCU_CMD_EXT_CID) | \
				 FIELD_PREP(__MCU_CMD_FIELD_EXT_ID,		\
					    MT7668_EXT_CMD_##_t) |		\
				 __MCU_CMD_FIELD_QUERY)
#define MT7668_CE_CMD(_t)	(__MCU_CMD_FIELD_CE | MT7668_CMD(_t))
#define MT7668_CE_QUERY(_t)	(MT7668_CE_CMD(_t) | __MCU_CMD_FIELD_QUERY)

enum {
	MT7668_EVENT_SCAN_DONE = 0x0d,
	MT7668_EVENT_TX_DONE = 0x0f,
	MT7668_EVENT_CH_PRIVILEGE = 0x10,
	MT7668_EVENT_BEACON_TIMEOUT = 0x13,
	MT7668_EVENT_DEBUG_MSG = 0x27,
	MT7668_EVENT_EXT = 0xed,
	MT7668_EVENT_ASSERT_DUMP = 0xf0,
};

enum {
	MT7668_NETWORK_TYPE_AIS,
	MT7668_NETWORK_TYPE_P2P,
};

/* values used by the firmware's BSS/STA records */
#define MT7668_PHY_TYPE_HR_DSSS		BIT(0)
#define MT7668_PHY_TYPE_ERP		BIT(1)
#define MT7668_PHY_TYPE_OFDM		BIT(3)
#define MT7668_PHY_TYPE_HT		BIT(4)
#define MT7668_PHY_TYPE_VHT		BIT(5)

#define MT7668_PHY_TYPE_ERP_INDEX	1
#define MT7668_PHY_TYPE_OFDM_INDEX	3

#define MT7668_STA_TYPE_LEGACY_AP	(BIT(0) | BIT(6))

enum {
	MT7668_STA_STATE_1,
	MT7668_STA_STATE_2,
	MT7668_STA_STATE_3,
};

enum {
	MT7668_MEDIA_STATE_CONNECTED,
	MT7668_MEDIA_STATE_DISCONNECTED,
};

#define MT7668_OP_MODE_INFRASTRUCTURE	0
#define MT7668_AUTH_MODE_OPEN		0
#define MT7668_AUTH_MODE_WPA2_PSK	7
#define MT7668_ENC_STATUS_DISABLED	1
#define MT7668_ENC_STATUS_CCMP		6

enum {
	MT7668_BAND_NULL,
	MT7668_BAND_2G4,
	MT7668_BAND_5G,
};

enum {
	MT7668_SCO_SCN = 0,
	MT7668_SCO_SCA = 1,
	MT7668_SCO_SCB = 3,
};

enum {
	MT7668_CW_20_40MHZ,
	MT7668_CW_80MHZ,
	MT7668_CW_160MHZ,
	MT7668_CW_80P80MHZ,
};

#define MT7668_CH_REQ_TYPE_JOIN		0
#define MT7668_DBDC_BAND_AUTO		4
#define MT7668_CH_JOIN_INTERVAL		4000	/* ms */

/* rate codes: TX mode << 6 | rate index */
#define MT7668_RATE_CCK_1M		0x0000
#define MT7668_RATE_OFDM_6M		0x004b

#define MT7668_RX_FILTER_DIRECTED	BIT(0)
#define MT7668_RX_FILTER_MULTICAST	BIT(1)
#define MT7668_RX_FILTER_ALL_MULTICAST	BIT(2)
#define MT7668_RX_FILTER_BROADCAST	BIT(3)
#define MT7668_RX_FILTER_PROMISCUOUS	BIT(5)

struct mt7668_mcu_rxd {
	__le32 rxd[4];

	__le16 len;
	__le16 pkt_type_id;

	u8 eid;
	u8 seq;
	__le16 __rsv;

	u8 ext_eid;
	u8 __rsv1[2];
	u8 s2d_index;
} __packed;

struct mt7668_mcu_debug_msg {
	__le16 id;
	u8 type;
	u8 flags;
	__le32 value;
	__le16 msg_size;
	u8 rsv[2];
	char msg[];
} __packed;

#define MT7668_NIC_CAP_LEN	0x74

struct mt7668_nic_capability {
	__le16 product_id;
	__le16 fw_version;
	__le16 driver_version;
	u8 hw_5g_disabled;
	u8 eeprom_used;
	u8 mac_addr[ETH_ALEN];
	u8 mac_endian;
	u8 hw_not_support_ac;
	u8 rf_version;
	u8 phy_version;
	u8 rf_cal_fail;
	u8 bb_cal_fail;
	char date_code[16];
	__le32 feature_flag[2];
	__le32 compile_flag[2];
	u8 branch_info[4];
	u8 fw_build_number;
	u8 hw_nss_1x1;
	u8 hw_no_dbdc;
	u8 hw_wf0_only;
} __packed;

/* capability v2: TLVs */
enum {
	MT7668_NIC_CAP_TX_RESOURCE,
	MT7668_NIC_CAP_EFUSE_ADDRESS,
};

struct mt7668_nic_cap_v2 {
	__le16 n_elements;
	u8 rsv[2];
	u8 data[];
} __packed;

struct mt7668_nic_cap_tlv {
	__le32 tag;
	__le32 len;
	u8 data[];
} __packed;

/* "use the efuse" (no EEPROM image): the firmware applies its calibration */
#define MT7668_EFUSE_MODE_EFUSE		0
#define MT7668_EFUSE_COUNT		0xff
#define MT7668_EFUSE_CMD_TYPE		0x11

struct mt7668_efuse_buffer_mode {
	u8 source;
	u8 count;
	u8 cmd_type;
	u8 rsv;
	u8 content[];
} __packed;

struct mt7668_basic_config {
	u8 native_80211;
	u8 assert_path;
	u8 debug_level;
	u8 rsv;
	__le16 tx_csum;
	__le16 rx_csum;
	u8 rsv1[4];
} __packed;

struct mt7668_bss_activate {
	u8 bss_idx;
	u8 active;
	u8 network_type;
	u8 omac_idx;
	u8 bss_addr[ETH_ALEN];
	u8 bmc_wlan_idx;
	u8 rsv;
} __packed;

struct mt7668_ch_privilege {
	u8 bss_idx;
	u8 token;
	u8 action;		/* 0: request, 1: abort */
	u8 primary_channel;
	u8 sco;
	u8 band;
	u8 channel_width;
	u8 center_seg1;
	u8 center_seg2;
	u8 req_type;
	u8 dbdc_band;
	u8 rsv;
	__le32 max_interval;	/* ms */
	u8 rsv2[8];
} __packed;

/* same layout for the grant event */
struct mt7668_ch_privilege_event {
	u8 bss_idx;
	u8 token;
	u8 status;
	u8 primary_channel;
	u8 sco;
	u8 band;
	u8 channel_width;
	u8 center_seg1;
	u8 center_seg2;
	u8 req_type;
	u8 dbdc_band;
	u8 rsv;
	__le32 grant_interval;
	u8 rsv2[8];
} __packed;

struct mt7668_bss_rlm {
	u8 bss_idx;
	u8 band;
	u8 primary_channel;
	u8 sco;
	u8 erp_protect;
	u8 ht_protect;
	u8 gf_operation;
	u8 tx_rifs;
	__le16 ht_op_info3;
	__le16 ht_op_info2;
	u8 ht_op_info1;
	u8 short_preamble;
	u8 short_slot;
	u8 vht_channel_width;
	u8 vht_center_seg1;
	u8 vht_center_seg2;
	__le16 vht_basic_mcs;
	u8 nss;
	u8 rsv;
} __packed;

struct mt7668_bss_info {
	u8 bss_idx;
	u8 connection_state;
	u8 op_mode;
	u8 ssid_len;
	u8 ssid[IEEE80211_MAX_SSID_LEN];
	u8 bssid[ETH_ALEN];
	u8 is_qbss;
	u8 rsv;
	__le16 operational_rates;
	__le16 basic_rates;
	u8 sta_rec_idx_ap;
	u8 rsv1;
	__le16 default_fixed_rate;
	u8 non_ht_basic_phy_type;
	u8 auth_mode;
	u8 enc_status;
	u8 phy_type_set;
	u8 wapi_mode;
	u8 is_ap_mode;
	u8 bmc_wlan_idx;
	u8 hidden_ssid_mode;
	u8 disconnect_detect_th;
	u8 rsv2[3];
	__le32 private_data;
	struct mt7668_bss_rlm rlm;
	u8 dbdc_band;
	u8 wmm_set;
	u8 dbdc_action;
	u8 nss;
	u8 rsv3[22];
} __packed;

struct mt7668_sta_rec {
	u8 sta_idx;
	u8 sta_type;
	u8 mac_addr[ETH_ALEN];
	__le16 aid;
	__le16 listen_interval;
	u8 bss_idx;
	u8 phy_type_set;
	__le16 rates;
	__le16 basic_rates;
	u8 is_qos;
	u8 is_uapsd;
	u8 sta_state;
	u8 mcs_set;
	u8 sup_mcs32;
	u8 rsv1;
	u8 rx_mcs_bitmask[10];
	__le16 rx_highest_rate;
	__le32 tx_rate_info;
	__le16 ht_cap_info;
	__le16 ht_ext_cap;
	__le32 tx_bf_cap;
	u8 ampdu_param;
	u8 asel_cap;
	u8 rcpi;
	u8 need_resp;
	u8 uapsd_ac;
	u8 uapsd_sp;
	u8 wlan_idx;
	u8 bmc_wlan_idx;
	__le32 vht_cap_info;
	__le16 vht_rx_mcs_map;
	__le16 vht_rx_highest_rate;
	__le16 vht_tx_mcs_map;
	__le16 vht_tx_highest_rate;
	u8 rts_policy;
	u8 vht_op_mode;
	u8 traffic_data_type;
	u8 tx_gf_mode;
	u8 tx_sgi_mode;
	u8 tx_stbc_mode;
	__le16 default_fixed_rate;
	u8 tx_ampdu;
	u8 rx_ampdu;
	u8 rsv2[2];
	__le32 fixed_phy_rate;
	__le16 max_link_speed;
	__le16 min_link_speed;
	__le32 flags;
	u8 tx_ba_size;
	u8 rx_ba_size;
	u8 rsv3[2];
	u8 txbf_pfmu_info[32];
	u8 tx_amsdu_in_ampdu;
	u8 rx_amsdu_in_ampdu;
	u8 rsv4[2];
	__le32 tx_max_amsdu_in_ampdu_len;
} __packed;

struct mt7668_sta_remove {
	u8 action;		/* 0: one STA */
	u8 sta_idx;
	u8 bss_idx;
	u8 rsv;
} __packed;

struct mt7668_key {
	u8 add;
	u8 tx_key;
	u8 key_type;		/* 1: pairwise */
	u8 is_authenticator;
	u8 peer_addr[ETH_ALEN];
	u8 bss_idx;
	u8 cipher;
	u8 key_id;
	u8 key_len;
	u8 wlan_idx;
	u8 mgmt_protection;
	u8 key[32];
	u8 key_rsc[16];
} __packed;

struct mt7668_tx_done_event {
	u8 pid;
	u8 status;
	__le16 seq;
	u8 wlan_idx;
	u8 tx_count;
	__le16 tx_rate;
} __packed;

#endif
