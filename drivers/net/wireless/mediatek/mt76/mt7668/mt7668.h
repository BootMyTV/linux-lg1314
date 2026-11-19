/* SPDX-License-Identifier: ISC */

#ifndef __MT7668_H
#define __MT7668_H

#include <linux/kernel.h>
#include <linux/usb.h>

#include "../mt76.h"
#include "../mt76_connac_mcu.h"
#include "regs.h"

#define MT7668_ROM_PATCH		"mediatek/mt7668_patch_e2_hdr.bin"
#define MT7668_FIRMWARE_N9		"mediatek/WIFI_RAM_CODE_MT7668.bin"
#define MT7668_FIRMWARE_CR4		"mediatek/WIFI_RAM_CODE2_USB_MT7668.bin"

#define MT7668_PATCH_ADDRESS		0xc8000

/* The host owns the WTBL; the firmware keeps entries 0 and 1 */
#define MT7668_WTBL_SIZE		32
#define MT7668_WTBL_RESERVED		2

/* own MAC address slot used for the station interface */
#define MT7668_OMAC_STA			1

#define MT7668_MAX_INTERFACES		1

struct mt7668_vif {
	struct mt76_vif_link mt76; /* must be first */
	struct mt76_wcid bmc_wcid;
	struct ieee80211_sta *ap_sta;

	u8 ch_token;
	bool ch_requested;
};

struct mt7668_sta {
	struct mt76_wcid wcid; /* must be first */
};

struct mt7668_dev {
	union { /* must be first */
		struct mt76_dev mt76;
		struct mt76_phy mphy;
	};

	u32 vif_mask;
	bool fw_debug;

	struct completion ch_grant;

	struct {
		u16 product_id;
		u16 fw_version;
		bool no_5ghz;
		bool no_vht;
		bool nss_1x1;
		bool wf0_only;
		u32 efuse_start;
		u32 efuse_end;
		char date[17];
	} cap;
};

static inline struct mt7668_dev *mt7668_hw_dev(struct ieee80211_hw *hw)
{
	struct mt76_phy *phy = hw->priv;

	return container_of(phy->dev, struct mt7668_dev, mt76);
}

/* mcu.c */
int mt7668_mcu_init(struct mt7668_dev *dev);
void mt7668_mcu_rx_event(struct mt7668_dev *dev, struct sk_buff *skb);
int mt7668_mcu_bss_activate(struct mt7668_dev *dev, struct ieee80211_vif *vif,
			    bool enable);
int mt7668_mcu_hw_scan(struct mt7668_dev *dev, struct ieee80211_vif *vif,
		       struct ieee80211_scan_request *scan_req);
int mt7668_mcu_set_rx_filter(struct mt7668_dev *dev, u32 filter);
int mt7668_mcu_fw_log_2_host(struct mt7668_dev *dev, bool enable);
int mt7668_mcu_set_key(struct mt7668_dev *dev, struct ieee80211_vif *vif,
		       struct ieee80211_sta *sta, struct ieee80211_key_conf *key,
		       enum set_key_cmd cmd);
int mt7668_mcu_cancel_hw_scan(struct mt7668_dev *dev,
			      struct ieee80211_vif *vif);
int mt7668_mcu_ch_privilege(struct mt7668_dev *dev, struct ieee80211_vif *vif,
			    bool request);
int mt7668_mcu_sta_update(struct mt7668_dev *dev, struct ieee80211_vif *vif,
			  struct ieee80211_sta *sta, u8 state);
int mt7668_mcu_sta_remove(struct mt7668_dev *dev, struct ieee80211_vif *vif,
			  struct ieee80211_sta *sta);
int mt7668_mcu_set_bss_info(struct mt7668_dev *dev, struct ieee80211_vif *vif,
			    bool connected);
int mt7668_mcu_set_bss_pm(struct mt7668_dev *dev, struct ieee80211_vif *vif,
			  bool connected);
u16 mt7668_rate_set(struct ieee80211_supported_band *sband, u32 rates);
u8 mt7668_band(enum nl80211_band band);

/* mac.c */
void mt7668_queue_rx_skb(struct mt76_dev *mdev, enum mt76_rxq_id q,
			 struct sk_buff *skb, u32 *info);
int mt7668_tx_prepare_skb(struct mt76_dev *mdev, void *txwi_ptr,
			  enum mt76_txq_id qid, struct mt76_wcid *wcid,
			  struct ieee80211_sta *sta,
			  struct mt76_tx_info *tx_info);
void mt7668_tx_complete_skb(struct mt76_dev *mdev, struct mt76_queue_entry *e);
bool mt7668_tx_status_data(struct mt76_dev *mdev, u8 *update);
void mt7668_mac_tx_done(struct mt7668_dev *dev, struct sk_buff *skb);

/* main.c / init.c */
extern const struct ieee80211_ops mt7668_ops;
extern const struct mt76_driver_ops mt7668_drv_ops;
int mt7668_register_device(struct mt7668_dev *dev);
void mt7668_unregister_device(struct mt7668_dev *dev);

#endif
