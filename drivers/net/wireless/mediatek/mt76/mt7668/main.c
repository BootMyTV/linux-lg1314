// SPDX-License-Identifier: ISC

#include <linux/debugfs.h>
#include <linux/etherdevice.h>

#include "mt7668.h"
#include "mcu.h"

static int mt7668_start(struct ieee80211_hw *hw)
{
	struct mt7668_dev *dev = mt7668_hw_dev(hw);

	set_bit(MT76_STATE_RUNNING, &dev->mphy.state);

	return 0;
}

static void mt7668_stop(struct ieee80211_hw *hw, bool suspend)
{
	struct mt7668_dev *dev = mt7668_hw_dev(hw);

	clear_bit(MT76_STATE_RUNNING, &dev->mphy.state);
	mt76u_stop_tx(&dev->mt76);
}

static int mt7668_add_interface(struct ieee80211_hw *hw,
				struct ieee80211_vif *vif)
{
	struct mt7668_vif *mvif = (struct mt7668_vif *)vif->drv_priv;
	struct mt7668_dev *dev = mt7668_hw_dev(hw);
	int idx, ret = 0;

	if (vif->type != NL80211_IFTYPE_STATION)
		return -EOPNOTSUPP;

	mutex_lock(&dev->mt76.mutex);

	idx = ffs(~dev->vif_mask) - 1;
	if (idx >= MT7668_MAX_INTERFACES) {
		ret = -ENOSPC;
		goto out;
	}

	mvif->mt76.idx = idx;
	mvif->mt76.omac_idx = MT7668_OMAC_STA + idx;
	mvif->mt76.band_idx = 0;
	mvif->mt76.wmm_idx = 0;

	/* broadcast/multicast WTBL entry for this BSS */
	ret = mt76_wcid_alloc(dev->mt76.wcid_mask, MT7668_WTBL_SIZE);
	if (ret < 0)
		goto out;

	mt76_wcid_init(&mvif->bmc_wcid, 0);
	mvif->bmc_wcid.idx = ret;
	mvif->bmc_wcid.hw_key_idx = -1;
	mvif->mt76.wcid = &mvif->bmc_wcid;

	ret = mt7668_mcu_bss_activate(dev, vif, true);
	if (ret) {
		mt76_wcid_mask_clear(dev->mt76.wcid_mask, mvif->bmc_wcid.idx);
		goto out;
	}

	rcu_assign_pointer(dev->mt76.wcid[mvif->bmc_wcid.idx],
			   &mvif->bmc_wcid);
	if (vif->txq) {
		struct mt76_txq *mtxq = (struct mt76_txq *)vif->txq->drv_priv;

		mtxq->wcid = mvif->bmc_wcid.idx;
	}

	dev->vif_mask |= BIT(idx);

	/* the firmware watches beacons and reports their loss */
	vif->driver_flags |= IEEE80211_VIF_BEACON_FILTER;
out:
	mutex_unlock(&dev->mt76.mutex);

	return ret;
}

static void mt7668_remove_interface(struct ieee80211_hw *hw,
				    struct ieee80211_vif *vif)
{
	struct mt7668_vif *mvif = (struct mt7668_vif *)vif->drv_priv;
	struct mt7668_dev *dev = mt7668_hw_dev(hw);
	int idx = mvif->bmc_wcid.idx;

	mutex_lock(&dev->mt76.mutex);

	mt7668_mcu_bss_activate(dev, vif, false);

	rcu_assign_pointer(dev->mt76.wcid[idx], NULL);
	mt76_wcid_mask_clear(dev->mt76.wcid_mask, idx);
	dev->vif_mask &= ~BIT(mvif->mt76.idx);

	mutex_unlock(&dev->mt76.mutex);

	mt76_wcid_cleanup(&dev->mt76, &mvif->bmc_wcid);
}

static int mt7668_config(struct ieee80211_hw *hw, int radio_idx, u32 changed)
{
	return 0;
}

static void mt7668_configure_filter(struct ieee80211_hw *hw,
				    unsigned int changed_flags,
				    unsigned int *total_flags, u64 multicast)
{
	struct mt7668_dev *dev = mt7668_hw_dev(hw);
	u32 filter = MT7668_RX_FILTER_DIRECTED | MT7668_RX_FILTER_BROADCAST |
		     MT7668_RX_FILTER_MULTICAST |
		     MT7668_RX_FILTER_ALL_MULTICAST;

	*total_flags &= FIF_ALLMULTI;

	mutex_lock(&dev->mt76.mutex);
	mt7668_mcu_set_rx_filter(dev, filter);
	mutex_unlock(&dev->mt76.mutex);
}

static int mt7668_hw_scan(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
			  struct ieee80211_scan_request *req)
{
	struct mt7668_dev *dev = mt7668_hw_dev(hw);
	int ret;

	mutex_lock(&dev->mt76.mutex);
	ret = mt7668_mcu_hw_scan(dev, vif, req);
	mutex_unlock(&dev->mt76.mutex);

	return ret;
}

static void mt7668_cancel_hw_scan(struct ieee80211_hw *hw,
				  struct ieee80211_vif *vif)
{
	struct mt7668_dev *dev = mt7668_hw_dev(hw);

	mutex_lock(&dev->mt76.mutex);
	mt7668_mcu_cancel_hw_scan(dev, vif);
	mutex_unlock(&dev->mt76.mutex);
}

static int mt7668_set_key(struct ieee80211_hw *hw, enum set_key_cmd cmd,
			  struct ieee80211_vif *vif, struct ieee80211_sta *sta,
			  struct ieee80211_key_conf *key)
{
	struct mt7668_dev *dev = mt7668_hw_dev(hw);
	int ret;

	/* management frame protection stays in software */
	if (key->cipher == WLAN_CIPHER_SUITE_AES_CMAC ||
	    key->cipher == WLAN_CIPHER_SUITE_BIP_CMAC_256)
		return -EOPNOTSUPP;

	mutex_lock(&dev->mt76.mutex);
	ret = mt7668_mcu_set_key(dev, vif, sta, key, cmd);
	mutex_unlock(&dev->mt76.mutex);

	if (!ret)
		key->hw_key_idx = key->keyidx;

	return ret;
}

static int mt7668_set_rts_threshold(struct ieee80211_hw *hw, int radio_idx,
				    u32 val)
{
	/* not supported yet: the firmware keeps its default */
	return 0;
}

static void mt7668_tx(struct ieee80211_hw *hw,
		      struct ieee80211_tx_control *control,
		      struct sk_buff *skb)
{
	struct mt7668_dev *dev = mt7668_hw_dev(hw);
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
	struct ieee80211_vif *vif = info->control.vif;
	struct mt76_wcid *wcid = &dev->mt76.global_wcid;

	if (control->sta) {
		struct mt7668_sta *msta;

		msta = (struct mt7668_sta *)control->sta->drv_priv;
		wcid = &msta->wcid;
	} else if (vif) {
		struct mt7668_vif *mvif = (struct mt7668_vif *)vif->drv_priv;

		wcid = &mvif->bmc_wcid;
	}

	mt76_tx(&dev->mphy, control->sta, wcid, skb);
}

static void mt7668_mgd_prepare_tx(struct ieee80211_hw *hw,
				  struct ieee80211_vif *vif,
				  struct ieee80211_prep_tx_info *info)
{
	struct mt7668_dev *dev = mt7668_hw_dev(hw);

	/* the firmware only goes to the BSS channel when asked to */
	mutex_lock(&dev->mt76.mutex);
	mt7668_mcu_ch_privilege(dev, vif, true);
	mutex_unlock(&dev->mt76.mutex);
}

static void mt7668_mgd_complete_tx(struct ieee80211_hw *hw,
				   struct ieee80211_vif *vif,
				   struct ieee80211_prep_tx_info *info)
{
	struct mt7668_dev *dev = mt7668_hw_dev(hw);

	if (info->success && info->subtype != IEEE80211_STYPE_DEAUTH &&
	    info->subtype != IEEE80211_STYPE_DISASSOC)
		return;

	mutex_lock(&dev->mt76.mutex);
	if (!vif->cfg.assoc)
		mt7668_mcu_ch_privilege(dev, vif, false);
	mutex_unlock(&dev->mt76.mutex);
}

static void mt7668_bss_info_changed(struct ieee80211_hw *hw,
				    struct ieee80211_vif *vif,
				    struct ieee80211_bss_conf *info,
				    u64 changed)
{
	struct mt7668_dev *dev = mt7668_hw_dev(hw);
	struct ieee80211_sta *sta;

	/*
	 * The receive filter drops frames from other BSSIDs, so the firmware
	 * needs the BSSID before authentication.
	 */
	if ((changed & BSS_CHANGED_BSSID) && !vif->cfg.assoc &&
	    !is_zero_ether_addr(info->bssid)) {
		mutex_lock(&dev->mt76.mutex);
		mt7668_mcu_set_bss_info(dev, vif, false);
		mutex_unlock(&dev->mt76.mutex);
	}

	if (!(changed & BSS_CHANGED_ASSOC))
		return;

	mutex_lock(&dev->mt76.mutex);

	if (vif->cfg.assoc) {
		/* BSS first, then the station's final record */
		mt7668_mcu_set_bss_info(dev, vif, true);

		sta = ((struct mt7668_vif *)vif->drv_priv)->ap_sta;
		if (sta)
			mt7668_mcu_sta_update(dev, vif, sta,
					      MT7668_STA_STATE_3);

		mt7668_mcu_set_bss_pm(dev, vif, true);
		mt7668_mcu_ch_privilege(dev, vif, false);
	} else {
		mt7668_mcu_set_bss_pm(dev, vif, false);
		mt7668_mcu_set_bss_info(dev, vif, false);
		mt7668_mcu_ch_privilege(dev, vif, false);
	}

	mutex_unlock(&dev->mt76.mutex);
}

const struct ieee80211_ops mt7668_ops = {
	.add_chanctx = ieee80211_emulate_add_chanctx,
	.remove_chanctx = ieee80211_emulate_remove_chanctx,
	.change_chanctx = ieee80211_emulate_change_chanctx,
	.switch_vif_chanctx = ieee80211_emulate_switch_vif_chanctx,
	.tx = mt7668_tx,
	.start = mt7668_start,
	.stop = mt7668_stop,
	.add_interface = mt7668_add_interface,
	.remove_interface = mt7668_remove_interface,
	.config = mt7668_config,
	.configure_filter = mt7668_configure_filter,
	.hw_scan = mt7668_hw_scan,
	.cancel_hw_scan = mt7668_cancel_hw_scan,
	.sta_state = mt76_sta_state,
	.wake_tx_queue = mt76_wake_tx_queue,
	.get_txpower = mt76_get_txpower,
	.set_rts_threshold = mt7668_set_rts_threshold,
	.mgd_prepare_tx = mt7668_mgd_prepare_tx,
	.mgd_complete_tx = mt7668_mgd_complete_tx,
	.bss_info_changed = mt7668_bss_info_changed,
	.set_key = mt7668_set_key,
};

static int mt7668_sta_add(struct mt76_dev *mdev, struct ieee80211_vif *vif,
			  struct ieee80211_sta *sta)
{
	struct mt7668_dev *dev = container_of(mdev, struct mt7668_dev, mt76);
	struct mt7668_sta *msta = (struct mt7668_sta *)sta->drv_priv;
	int idx;

	idx = mt76_wcid_alloc(mdev->wcid_mask, MT7668_WTBL_SIZE);
	if (idx < 0)
		return -ENOSPC;

	msta->wcid.idx = idx;
	msta->wcid.hw_key_idx = -1;
	msta->wcid.sta = 1;

	if (vif->type == NL80211_IFTYPE_STATION)
		((struct mt7668_vif *)vif->drv_priv)->ap_sta = sta;

	return mt7668_mcu_sta_update(dev, vif, sta, MT7668_STA_STATE_1);
}

static void mt7668_sta_remove(struct mt76_dev *mdev, struct ieee80211_vif *vif,
			      struct ieee80211_sta *sta)
{
	struct mt7668_dev *dev = container_of(mdev, struct mt7668_dev, mt76);
	struct mt76_wcid *wcid = (struct mt76_wcid *)sta->drv_priv;

	mt7668_mcu_sta_remove(dev, vif, sta);
	rcu_assign_pointer(mdev->wcid[wcid->idx], NULL);

	if (((struct mt7668_vif *)vif->drv_priv)->ap_sta == sta)
		((struct mt7668_vif *)vif->drv_priv)->ap_sta = NULL;
}

const struct mt76_driver_ops mt7668_drv_ops = {
	.drv_flags = MT_DRV_RX_DMA_HDR | MT_DRV_HW_MGMT_TXQ,
	.tx_prepare_skb = mt7668_tx_prepare_skb,
	.tx_complete_skb = mt7668_tx_complete_skb,
	.tx_status_data = mt7668_tx_status_data,
	.rx_skb = mt7668_queue_rx_skb,
	.sta_add = mt7668_sta_add,
	.sta_remove = mt7668_sta_remove,
};

static int mt7668_fw_debug_set(void *data, u64 val)
{
	struct mt7668_dev *dev = data;
	int ret;

	mutex_lock(&dev->mt76.mutex);
	ret = mt7668_mcu_fw_log_2_host(dev, !!val);
	mutex_unlock(&dev->mt76.mutex);

	return ret;
}

static int mt7668_fw_debug_get(void *data, u64 *val)
{
	struct mt7668_dev *dev = data;

	*val = dev->fw_debug;

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_fw_debug, mt7668_fw_debug_get,
			 mt7668_fw_debug_set, "%lld\n");

int mt7668_register_device(struct mt7668_dev *dev)
{
	struct ieee80211_hw *hw = mt76_hw(dev);
	struct wiphy *wiphy = hw->wiphy;
	struct dentry *dir;
	int ret;

	hw->queues = 4;
	hw->max_rates = 3;
	hw->max_report_rates = 7;
	hw->max_rate_tries = 11;
	hw->max_tx_fragments = 1;
	hw->extra_tx_headroom = MT_TXD_SIZE + MT_TXP_SIZE;
	hw->sta_data_size = sizeof(struct mt7668_sta);
	hw->vif_data_size = sizeof(struct mt7668_vif);

	wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION);
	wiphy->max_scan_ie_len = MT76_CONNAC_SCAN_IE_LEN;
	wiphy->max_scan_ssids = 4;

	ieee80211_hw_set(hw, HAS_RATE_CONTROL);
	ieee80211_hw_set(hw, CONNECTION_MONITOR);

	dev->mphy.cap.has_2ghz = true;
	dev->mphy.cap.has_5ghz = !dev->cap.no_5ghz;
	dev->mphy.antenna_mask = 0x3;
	dev->mphy.chainmask = 0x3;

	ret = mt76_register_device(&dev->mt76, !dev->cap.no_vht, mt76_rates,
				   ARRAY_SIZE(mt76_rates));
	if (ret)
		return ret;

	dir = mt76_register_debugfs(&dev->mt76);
	if (dir)
		debugfs_create_file("fw_debug", 0600, dir, dev, &fops_fw_debug);

	return 0;
}
