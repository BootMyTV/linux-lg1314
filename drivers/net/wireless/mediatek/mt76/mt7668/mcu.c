// SPDX-License-Identifier: ISC

#include <linux/firmware.h>
#include <linux/module.h>

#include "mt7668.h"
#include "mcu.h"
#include "../mt76_connac_mcu.h"

#define MT7668_PATCH_SEM_RETRY		51
#define MT7668_FW_CHUNK			2048
#define MT7668_FW_N_REGIONS		2
#define MT7668_FW_CRC_LEN		4
#define MT7668_SCAN_CHANNEL_TIME	60

/* commands to the running firmware go to both N9 and CR4 */
#define MT7668_S2D_H2N_H2C		3

struct mt7668_patch_hdr {
	char build_date[16];
	char platform[4];
	__be32 hw_sw_ver;
	__be32 patch_ver;
	__be16 checksum;
} __packed;

struct mt7668_fw_trailer {
	__le32 addr;
	u8 chip_id;
	u8 feature_set;
	u8 eco_code;
	char fw_ver[10];
	char build_date[15];
	__le32 len;
} __packed;

static bool mt7668_mcu_running(struct mt7668_dev *dev)
{
	return test_bit(MT76_STATE_MCU_RUNNING, &dev->mphy.state);
}

static void
mt7668_mcu_fill_msg(struct mt7668_dev *dev, struct sk_buff *skb, int cmd,
		    int *wait_seq)
{
	bool fwdl = cmd == MCU_CMD(FW_SCATTER);
	struct mt76_connac2_mcu_txd *txd;
	u8 seq, q_idx, ext_cid;
	u32 val;

	seq = ++dev->mt76.mcu.msg_seq & 0xf;
	if (!seq)
		seq = ++dev->mt76.mcu.msg_seq & 0xf;
	if (wait_seq)
		*wait_seq = seq;

	txd = (struct mt76_connac2_mcu_txd *)skb_push(skb, sizeof(*txd));
	memset(txd, 0, sizeof(*txd));

	q_idx = fwdl ? MT_TX_MCU_PORT_RX_FWDL : MT_TX_MCU_PORT_RX_Q0;

	val = FIELD_PREP(MT_TXD0_TX_BYTES, skb->len) | MT_TXD0_P_IDX;
	if (mt7668_mcu_running(dev))
		val |= FIELD_PREP(MT_TXD0_Q_IDX, q_idx | MT_TX_MCU_Q_IDX_H2CDMA);
	else
		val |= FIELD_PREP(MT_TXD0_Q_IDX, q_idx);
	txd->txd[0] = cpu_to_le32(val);

	val = MT_TXD1_LONG_FORMAT |
	      FIELD_PREP(MT_TXD1_HDR_FORMAT, MT_HDR_FORMAT_CMD) |
	      FIELD_PREP(MT_TXD1_PKT_FMT, fwdl ? MT_TX_TYPE_FW : MT_TX_TYPE_CMD);
	txd->txd[1] = cpu_to_le32(val);

	ext_cid = FIELD_GET(__MCU_CMD_FIELD_EXT_ID, cmd);

	txd->len = cpu_to_le16(skb->len - sizeof(txd->txd));
	txd->pq_id = cpu_to_le16(MCU_PQ_ID(1, q_idx));
	/* firmware chunks carry CID 0 */
	txd->cid = fwdl ? 0 : FIELD_GET(__MCU_CMD_FIELD_ID, cmd);
	txd->pkt_type = MCU_PKT_ID;
	txd->seq = seq;
	txd->ext_cid = ext_cid;
	txd->s2d_index = mt7668_mcu_running(dev) ? MT7668_S2D_H2N_H2C :
						   MCU_S2D_H2N;

	if (ext_cid || (cmd & __MCU_CMD_FIELD_CE)) {
		txd->set_query = cmd & __MCU_CMD_FIELD_QUERY ? MCU_Q_QUERY :
							      MCU_Q_SET;
		txd->ext_cid_ack = 0;
	} else {
		txd->set_query = MCU_Q_NA;
	}
}

static int
mt7668_mcu_send_message(struct mt76_dev *mdev, struct sk_buff *skb,
			int cmd, int *seq)
{
	struct mt7668_dev *dev = container_of(mdev, struct mt7668_dev, mt76);
	int ret, pad;

	mt7668_mcu_fill_msg(dev, skb, cmd, seq);

	/* pad to 4 bytes, then a zero dword ends the transfer */
	pad = round_up(skb->len, 4) + 4 - skb->len;
	ret = mt76_skb_adjust_pad(skb, pad);
	if (ret < 0)
		goto out;

	ret = mt76u_bulk_msg(mdev, skb->data, skb->len, NULL, 1000,
			     MT_EP_OUT_INBAND_CMD);
out:
	dev_kfree_skb(skb);

	return ret;
}

static int
mt7668_mcu_parse_response(struct mt76_dev *mdev, int cmd,
			  struct sk_buff *skb, int seq)
{
	struct mt7668_mcu_rxd *rxd;

	if (!skb) {
		dev_err(mdev->dev, "Message %08x (seq %d) timeout\n", cmd, seq);
		return -ETIMEDOUT;
	}

	rxd = (struct mt7668_mcu_rxd *)skb->data;
	if (seq != rxd->seq)
		return -EAGAIN;

	/* ROM events have a shorter header: status is at 24 */
	if (cmd == MCU_CMD(PATCH_SEM_CONTROL)) {
		skb_pull(skb, offsetof(struct mt7668_mcu_rxd, ext_eid));
		return *skb->data;
	}

	skb_pull(skb, sizeof(*rxd));

	return 0;
}

static void
mt7668_mcu_debug_msg_event(struct mt7668_dev *dev, struct sk_buff *skb)
{
	struct mt7668_mcu_debug_msg *msg;
	int len;

	if (!dev->fw_debug)
		return;

	skb_pull(skb, sizeof(struct mt7668_mcu_rxd));
	if (skb->len < sizeof(*msg))
		return;

	msg = (struct mt7668_mcu_debug_msg *)skb->data;
	len = min_t(int, le16_to_cpu(msg->msg_size), skb->len - sizeof(*msg));
	len = strnlen(msg->msg, len);
	while (len && (msg->msg[len - 1] == '\n' || msg->msg[len - 1] == '\r'))
		len--;

	dev_info(dev->mt76.dev, "fw: %.*s\n", len, msg->msg);
}

static void
mt7668_mcu_scan_done_event(struct mt7668_dev *dev, struct sk_buff *skb)
{
	struct cfg80211_scan_info info = {
		.aborted = false,
	};

	if (!test_and_clear_bit(MT76_HW_SCANNING, &dev->mphy.state))
		return;

	ieee80211_scan_completed(dev->mphy.hw, &info);
}

static void
mt7668_mcu_ch_privilege_event(struct mt7668_dev *dev, struct sk_buff *skb)
{
	struct mt7668_ch_privilege_event *ev;

	skb_pull(skb, sizeof(struct mt7668_mcu_rxd));
	if (skb->len < sizeof(*ev))
		return;

	ev = (struct mt7668_ch_privilege_event *)skb->data;
	dev_dbg(dev->mt76.dev, "channel %d granted for %d ms (status %d)\n",
		ev->primary_channel, le32_to_cpu(ev->grant_interval),
		ev->status);
	if (!ev->status)
		complete(&dev->ch_grant);
}

static void
mt7668_mcu_beacon_timeout_event(struct mt7668_dev *dev, struct sk_buff *skb)
{
	skb_pull(skb, sizeof(struct mt7668_mcu_rxd));
	if (skb->len < sizeof(struct mt76_connac_beacon_loss_event))
		return;

	ieee80211_iterate_active_interfaces_atomic(dev->mphy.hw,
					IEEE80211_IFACE_ITER_RESUME_ALL,
					mt76_connac_mcu_beacon_loss_iter,
					skb->data);
}

static void
mt7668_mcu_rx_unsolicited_event(struct mt7668_dev *dev, struct sk_buff *skb)
{
	struct mt7668_mcu_rxd *rxd = (struct mt7668_mcu_rxd *)skb->data;

	switch (rxd->eid) {
	case MT7668_EVENT_DEBUG_MSG:
		mt7668_mcu_debug_msg_event(dev, skb);
		break;
	case MT7668_EVENT_SCAN_DONE:
		mt7668_mcu_scan_done_event(dev, skb);
		break;
	case MT7668_EVENT_CH_PRIVILEGE:
		mt7668_mcu_ch_privilege_event(dev, skb);
		break;
	case MT7668_EVENT_BEACON_TIMEOUT:
		mt7668_mcu_beacon_timeout_event(dev, skb);
		break;
	case MT7668_EVENT_TX_DONE:
		skb_pull(skb, sizeof(*rxd));
		mt7668_mac_tx_done(dev, skb);
		break;
	case MT7668_EVENT_ASSERT_DUMP:
		skb_pull(skb, sizeof(*rxd));
		dev_err(dev->mt76.dev, "firmware assert: %.*s\n",
			(int)strnlen(skb->data, skb->len), skb->data);
		break;
	default:
		dev_dbg(dev->mt76.dev, "unhandled event %02x seq %d\n",
			rxd->eid, rxd->seq);
		break;
	}

	dev_kfree_skb(skb);
}

void mt7668_mcu_rx_event(struct mt7668_dev *dev, struct sk_buff *skb)
{
	struct mt7668_mcu_rxd *rxd = (struct mt7668_mcu_rxd *)skb->data;

	if (skb->len < sizeof(*rxd)) {
		dev_kfree_skb(skb);
		return;
	}

	switch (rxd->eid) {
	case MT7668_EVENT_DEBUG_MSG:
	case MT7668_EVENT_SCAN_DONE:
	case MT7668_EVENT_CH_PRIVILEGE:
	case MT7668_EVENT_BEACON_TIMEOUT:
	case MT7668_EVENT_TX_DONE:
	case MT7668_EVENT_ASSERT_DUMP:
		mt7668_mcu_rx_unsolicited_event(dev, skb);
		return;
	}

	if (!rxd->seq)
		mt7668_mcu_rx_unsolicited_event(dev, skb);
	else
		mt76_mcu_rx_event(&dev->mt76, skb);
}

static int mt7668_load_patch(struct mt7668_dev *dev)
{
	const struct mt7668_patch_hdr *hdr;
	const struct firmware *fw;
	struct {
		__le32 addr;
		__le32 len;
		__le32 mode;
	} req;
	int i, sem, ret;
	u32 len;

	/*
	 * The semaphore answers "not downloaded, no semaphore" while the
	 * chip is still settling; retry for a while, then download anyway.
	 */
	for (i = 0; i < MT7668_PATCH_SEM_RETRY; i++) {
		sem = mt76_connac_mcu_patch_sem_ctrl(&dev->mt76, true);
		if (sem != PATCH_NOT_DL_SEM_FAIL)
			break;
		msleep(100);
	}

	switch (sem) {
	case PATCH_IS_DL:
		return 0;
	case PATCH_NOT_DL_SEM_SUCCESS:
	case PATCH_NOT_DL_SEM_FAIL:
		break;
	default:
		dev_err(dev->mt76.dev, "Failed to get patch semaphore: %d\n",
			sem);
		return -EAGAIN;
	}

	ret = request_firmware(&fw, MT7668_ROM_PATCH, dev->mt76.dev);
	if (ret)
		goto out_sem;

	if (!fw || !fw->data || fw->size < sizeof(*hdr)) {
		dev_err(dev->mt76.dev, "Invalid firmware\n");
		ret = -EINVAL;
		goto out;
	}

	hdr = (const struct mt7668_patch_hdr *)fw->data;
	dev_info(dev->mt76.dev, "HW/SW Version: 0x%x, Build Time: %.16s\n",
		 be32_to_cpu(hdr->hw_sw_ver), hdr->build_date);

	len = fw->size - sizeof(*hdr);
	req.addr = cpu_to_le32(MT7668_PATCH_ADDRESS);
	req.len = cpu_to_le32(len);
	req.mode = cpu_to_le32(DL_MODE_NEED_RSP);

	ret = mt76_mcu_send_msg(&dev->mt76, MCU_CMD(PATCH_START_REQ),
				&req, sizeof(req), true);
	if (ret) {
		dev_err(dev->mt76.dev, "Download request failed\n");
		goto out;
	}

	ret = __mt76_mcu_send_firmware(&dev->mt76, MCU_CMD(FW_SCATTER),
				       fw->data + sizeof(*hdr), len,
				       MT7668_FW_CHUNK);
	if (ret) {
		dev_err(dev->mt76.dev, "Failed to send firmware to device\n");
		goto out;
	}

	ret = mt76_connac_mcu_start_patch(&dev->mt76);
	if (ret)
		dev_err(dev->mt76.dev, "Failed to start patch\n");

out:
	release_firmware(fw);
out_sem:
	if (sem == PATCH_NOT_DL_SEM_SUCCESS &&
	    mt76_connac_mcu_patch_sem_ctrl(&dev->mt76, false) !=
	    PATCH_REL_SEM_SUCCESS) {
		dev_err(dev->mt76.dev, "Failed to release patch semaphore\n");
		ret = -EAGAIN;
	}

	return ret;
}

static int
mt7668_load_ram(struct mt7668_dev *dev, const char *name, bool cr4)
{
	const struct mt7668_fw_trailer *hdr;
	const struct firmware *fw;
	int i, ret, offset = 0;

	ret = request_firmware(&fw, name, dev->mt76.dev);
	if (ret)
		return ret;

	if (!fw || !fw->data ||
	    fw->size < MT7668_FW_N_REGIONS * sizeof(*hdr)) {
		dev_err(dev->mt76.dev, "Invalid firmware\n");
		ret = -EINVAL;
		goto out;
	}

	hdr = (const struct mt7668_fw_trailer *)
	      (fw->data + fw->size - MT7668_FW_N_REGIONS * sizeof(*hdr));
	dev_info(dev->mt76.dev, "%s Firmware Version: %.10s, Build Time: %.15s\n",
		 cr4 ? "CR4" : "N9", hdr->fw_ver, hdr->build_date);

	for (i = 0; i < MT7668_FW_N_REGIONS; i++) {
		u32 len = le32_to_cpu(hdr[i].len) + MT7668_FW_CRC_LEN;
		u32 addr = le32_to_cpu(hdr[i].addr);
		u32 mode;

		mode = mt76_connac_mcu_gen_dl_mode(&dev->mt76,
						   hdr[i].feature_set, cr4);

		if (offset + len > fw->size) {
			ret = -EINVAL;
			goto out;
		}

		ret = mt76_connac_mcu_init_download(&dev->mt76, addr, len,
						    mode);
		if (ret) {
			dev_err(dev->mt76.dev, "Download request failed\n");
			goto out;
		}

		ret = __mt76_mcu_send_firmware(&dev->mt76, MCU_CMD(FW_SCATTER),
					       fw->data + offset, len,
					       MT7668_FW_CHUNK);
		if (ret) {
			dev_err(dev->mt76.dev,
				"Failed to send firmware to device\n");
			goto out;
		}

		offset += len;
	}

	ret = mt76_connac_mcu_start_firmware(&dev->mt76, 0,
					     cr4 ? FW_START_WORKING_PDA_CR4 : 0);
	if (ret)
		dev_err(dev->mt76.dev, "Failed to start %s firmware\n",
			cr4 ? "CR4" : "N9");
out:
	release_firmware(fw);

	return ret;
}

static int mt7668_load_firmware(struct mt7668_dev *dev)
{
	int ret;

	mt76_set(dev, MT_UDMA_TX_QSEL, MT_FW_DL_EN);

	ret = mt7668_load_patch(dev);
	if (ret)
		goto out;

	ret = mt7668_load_ram(dev, MT7668_FIRMWARE_N9, false);
	if (ret)
		goto out;

	ret = mt7668_load_ram(dev, MT7668_FIRMWARE_CR4, true);
	if (ret)
		goto out;

	if (!mt76_poll_msec(dev, MT_SW_SYNC0, MT_SW_SYNC0_READY,
			    MT_SW_SYNC0_READY, 1500)) {
		dev_err(dev->mt76.dev, "Firmware not ready (sync0 %08x)\n",
			mt76_rr(dev, MT_SW_SYNC0));
		ret = -EIO;
	}
out:
	mt76_clear(dev, MT_UDMA_TX_QSEL, MT_FW_DL_EN);

	return ret;
}

static int mt7668_mcu_get_nic_capability(struct mt7668_dev *dev)
{
	struct mt7668_nic_capability *cap;
	u8 req[MT7668_NIC_CAP_LEN] = {};
	struct sk_buff *skb;
	int ret;

	ret = mt76_mcu_send_and_get_msg(&dev->mt76,
					MT7668_CE_QUERY(GET_NIC_CAPAB),
					req, sizeof(req), true, &skb);
	if (ret)
		return ret;

	if (skb->len < sizeof(*cap)) {
		ret = -EINVAL;
		goto out;
	}

	cap = (struct mt7668_nic_capability *)skb->data;
	dev->cap.product_id = le16_to_cpu(cap->product_id);
	dev->cap.fw_version = le16_to_cpu(cap->fw_version);
	dev->cap.no_5ghz = cap->hw_5g_disabled;
	dev->cap.no_vht = cap->hw_not_support_ac;
	dev->cap.nss_1x1 = cap->hw_nss_1x1;
	dev->cap.wf0_only = cap->hw_wf0_only;
	memcpy(dev->cap.date, cap->date_code, sizeof(cap->date_code));
	ether_addr_copy(dev->mphy.macaddr, cap->mac_addr);

	dev_info(dev->mt76.dev,
		 "chip %04x fw %04x built %s, eeprom %d, 5GHz %s, VHT %s\n",
		 dev->cap.product_id, dev->cap.fw_version, dev->cap.date,
		 cap->eeprom_used, dev->cap.no_5ghz ? "off" : "on",
		 dev->cap.no_vht ? "off" : "on");
out:
	dev_kfree_skb(skb);

	return ret;
}

static int mt7668_mcu_get_nic_capability_v2(struct mt7668_dev *dev)
{
	struct mt7668_nic_cap_v2 *cap;
	struct sk_buff *skb;
	int i, ret, n;
	u32 off;

	ret = mt76_mcu_send_and_get_msg(&dev->mt76,
					MT7668_CE_QUERY(GET_NIC_CAPAB_V2),
					NULL, 0, true, &skb);
	if (ret)
		return ret;

	if (skb->len < sizeof(*cap)) {
		ret = -EINVAL;
		goto out;
	}

	cap = (struct mt7668_nic_cap_v2 *)skb->data;
	n = le16_to_cpu(cap->n_elements);
	off = sizeof(*cap);
	for (i = 0; i < n; i++) {
		struct mt7668_nic_cap_tlv *tlv;
		u32 len;

		if (off + sizeof(*tlv) > skb->len)
			break;

		tlv = (struct mt7668_nic_cap_tlv *)(skb->data + off);
		len = le32_to_cpu(tlv->len);
		if (off + sizeof(*tlv) + len > skb->len)
			break;

		switch (le32_to_cpu(tlv->tag)) {
		case MT7668_NIC_CAP_EFUSE_ADDRESS:
			if (len < 8)
				break;
			dev->cap.efuse_start = get_unaligned_le32(tlv->data);
			dev->cap.efuse_end = get_unaligned_le32(tlv->data + 4);
			break;
		default:
			break;
		}

		off += sizeof(*tlv) + len;
	}
out:
	dev_kfree_skb(skb);

	return ret;
}

/*
 * Tell the firmware to take its RF calibration from the efuse. Without it
 * the radio transmits but never sees an ACK.
 */
static int mt7668_mcu_set_efuse_mode(struct mt7668_dev *dev)
{
	struct mt7668_efuse_buffer_mode *req;
	u32 len = 0;
	int ret;

	if (dev->cap.efuse_end >= dev->cap.efuse_start &&
	    dev->cap.efuse_end - dev->cap.efuse_start < 2048)
		len = dev->cap.efuse_end - dev->cap.efuse_start + 1;

	req = kzalloc(sizeof(*req) + len, GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	req->source = MT7668_EFUSE_MODE_EFUSE;
	req->count = MT7668_EFUSE_COUNT;
	req->cmd_type = MT7668_EFUSE_CMD_TYPE;

	ret = mt76_mcu_send_msg(&dev->mt76, MT7668_EXT_QUERY(EFUSE_BUFFER_MODE),
				req, sizeof(*req) + len, true);
	kfree(req);

	return ret;
}

static int mt7668_mcu_basic_config(struct mt7668_dev *dev)
{
	struct mt7668_basic_config req = {};

	return mt76_mcu_send_msg(&dev->mt76, MT7668_CE_CMD(BASIC_CONFIG),
				 &req, sizeof(req), false);
}

int mt7668_mcu_bss_activate(struct mt7668_dev *dev, struct ieee80211_vif *vif,
			    bool enable)
{
	struct mt7668_vif *mvif = (struct mt7668_vif *)vif->drv_priv;
	struct mt7668_bss_activate req = {
		.bss_idx = mvif->mt76.idx,
		.active = enable,
		.network_type = MT7668_NETWORK_TYPE_AIS,
		.omac_idx = mvif->mt76.omac_idx,
		.bmc_wlan_idx = mvif->bmc_wcid.idx,
	};

	ether_addr_copy(req.bss_addr, vif->addr);

	return mt76_mcu_send_msg(&dev->mt76,
				 MT7668_CE_CMD(BSS_ACTIVATE_CTRL),
				 &req, sizeof(req), false);
}

int mt7668_mcu_hw_scan(struct mt7668_dev *dev, struct ieee80211_vif *vif,
		       struct ieee80211_scan_request *scan_req)
{
	struct mt7668_vif *mvif = (struct mt7668_vif *)vif->drv_priv;
	struct cfg80211_scan_request *sreq = &scan_req->req;
	struct mt76_connac_hw_scan_req *req;
	int i, n_ssids = 0, len, err, duration;
	struct sk_buff *skb;

	if (test_bit(MT76_HW_SCANNING, &dev->mphy.state))
		return -EBUSY;

	/* the firmware only knows the fields up to the probe request IEs */
	len = offsetof(struct mt76_connac_hw_scan_req, ies) +
	      min_t(int, sreq->ie_len, MT76_CONNAC_SCAN_IE_LEN);
	skb = mt76_mcu_msg_alloc(&dev->mt76, NULL, len);
	if (!skb)
		return -ENOMEM;

	req = (struct mt76_connac_hw_scan_req *)skb_put_zero(skb, len);

	mvif->mt76.scan_seq_num = (mvif->mt76.scan_seq_num + 1) & 0x7f;
	req->seq_num = mvif->mt76.scan_seq_num;
	req->bss_idx = mvif->mt76.idx;
	req->scan_type = sreq->n_ssids ? 1 : 0;

	for (i = 0; i < sreq->n_ssids && n_ssids < ARRAY_SIZE(req->ssids); i++) {
		if (!sreq->ssids[i].ssid_len)
			continue;

		req->ssids[n_ssids].ssid_len =
			cpu_to_le32(sreq->ssids[i].ssid_len);
		memcpy(req->ssids[n_ssids].ssid, sreq->ssids[i].ssid,
		       sreq->ssids[i].ssid_len);
		n_ssids++;
	}
	req->ssid_type = n_ssids ? BIT(2) : BIT(0);
	req->ssids_num = n_ssids;

	duration = MT7668_SCAN_CHANNEL_TIME;
	if (!sreq->n_ssids)
		duration *= 2;
	req->channel_dwell_time = cpu_to_le16(duration);
	req->timeout_value = cpu_to_le16(sreq->n_channels * duration);

	if (sreq->n_channels > ARRAY_SIZE(req->channels)) {
		req->channel_type = 0;
	} else {
		req->channel_type = 4;
		req->channels_num = sreq->n_channels;
		for (i = 0; i < sreq->n_channels; i++) {
			struct ieee80211_channel *chan = sreq->channels[i];

			req->channels[i].band =
				chan->band == NL80211_BAND_2GHZ ? 1 : 2;
			req->channels[i].channel_num = chan->hw_value;
		}
	}

	len -= offsetof(struct mt76_connac_hw_scan_req, ies);
	if (len) {
		memcpy(req->ies, sreq->ie, len);
		req->ies_len = cpu_to_le16(len);
	}

	set_bit(MT76_HW_SCANNING, &dev->mphy.state);
	err = mt76_mcu_skb_send_msg(&dev->mt76, skb,
				    MT7668_CE_CMD(SCAN_REQ), false);
	if (err < 0)
		clear_bit(MT76_HW_SCANNING, &dev->mphy.state);

	return err;
}

static_assert(sizeof(struct mt7668_ch_privilege) == 0x18);
static_assert(sizeof(struct mt7668_bss_info) == 0x74);
static_assert(sizeof(struct mt7668_sta_rec) == 0x88);

u8 mt7668_band(enum nl80211_band band)
{
	return band == NL80211_BAND_2GHZ ? MT7668_BAND_2G4 : MT7668_BAND_5G;
}

/* mac80211 rate bitmap (mt76_rates order) to the firmware's rate set */
u16 mt7668_rate_set(struct ieee80211_supported_band *sband, u32 rates)
{
	/* firmware bits: 1, 2, 5.5, 11, 22, 33, 6, 9, ..., 54 Mbps */
	static const u16 bitrate_to_bit[][2] = {
		{ 10, 0 }, { 20, 1 }, { 55, 2 }, { 110, 3 },
		{ 60, 6 }, { 90, 7 }, { 120, 8 }, { 180, 9 },
		{ 240, 10 }, { 360, 11 }, { 480, 12 }, { 540, 13 },
	};
	u16 set = 0;
	int i, j;

	for (i = 0; i < sband->n_bitrates; i++) {
		if (!(rates & BIT(i)))
			continue;

		for (j = 0; j < ARRAY_SIZE(bitrate_to_bit); j++)
			if (sband->bitrates[i].bitrate == bitrate_to_bit[j][0])
				set |= BIT(bitrate_to_bit[j][1]);
	}

	return set;
}

static u8 mt7668_sco(struct cfg80211_chan_def *chandef)
{
	struct ieee80211_channel *chan = chandef->chan;

	switch (chandef->width) {
	case NL80211_CHAN_WIDTH_40:
		return chandef->center_freq1 > chan->center_freq ?
		       MT7668_SCO_SCA : MT7668_SCO_SCB;
	case NL80211_CHAN_WIDTH_80:
	case NL80211_CHAN_WIDTH_80P80:
	case NL80211_CHAN_WIDTH_160:
		/* 5 GHz 40 MHz pairs are 36+40, 44+48, ... */
		return (chan->hw_value / 4) % 2 ? MT7668_SCO_SCA :
						  MT7668_SCO_SCB;
	default:
		return MT7668_SCO_SCN;
	}
}

static void
mt7668_chandef_params(struct cfg80211_chan_def *chandef, u8 *sco, u8 *width,
		      u8 *seg1, u8 *seg2)
{
	*sco = mt7668_sco(chandef);
	*width = MT7668_CW_20_40MHZ;
	*seg1 = 0;
	*seg2 = 0;

	switch (chandef->width) {
	case NL80211_CHAN_WIDTH_80:
		*width = MT7668_CW_80MHZ;
		*seg1 = ieee80211_frequency_to_channel(chandef->center_freq1);
		break;
	case NL80211_CHAN_WIDTH_160:
		*width = MT7668_CW_160MHZ;
		*seg1 = ieee80211_frequency_to_channel(chandef->center_freq1);
		break;
	default:
		break;
	}
}

int mt7668_mcu_set_key(struct mt7668_dev *dev, struct ieee80211_vif *vif,
		       struct ieee80211_sta *sta, struct ieee80211_key_conf *key,
		       enum set_key_cmd cmd)
{
	struct mt7668_vif *mvif = (struct mt7668_vif *)vif->drv_priv;
	bool pairwise = key->flags & IEEE80211_KEY_FLAG_PAIRWISE;
	struct mt7668_key req = {
		.add = cmd == SET_KEY,
		.tx_key = pairwise,
		.key_type = pairwise,
		.bss_idx = mvif->mt76.idx,
		.key_id = key->keyidx,
		.key_len = key->keylen,
	};

	switch (key->cipher) {
	case WLAN_CIPHER_SUITE_WEP40:
		req.cipher = MT_CIPHER_WEP40;
		break;
	case WLAN_CIPHER_SUITE_WEP104:
		req.cipher = MT_CIPHER_WEP104;
		break;
	case WLAN_CIPHER_SUITE_TKIP:
		req.cipher = MT_CIPHER_TKIP;
		break;
	case WLAN_CIPHER_SUITE_CCMP:
		req.cipher = MT_CIPHER_AES_CCMP;
		break;
	default:
		return -EOPNOTSUPP;
	}

	if (key->keylen > sizeof(req.key))
		return -EINVAL;

	if (sta) {
		ether_addr_copy(req.peer_addr, sta->addr);
		req.wlan_idx = ((struct mt76_wcid *)sta->drv_priv)->idx;
	} else {
		ether_addr_copy(req.peer_addr, vif->bss_conf.bssid);
		req.wlan_idx = mvif->bmc_wcid.idx;
	}
	memcpy(req.key, key->key, key->keylen);

	return mt76_mcu_send_msg(&dev->mt76, MT7668_CE_CMD(ADD_REMOVE_KEY),
				 &req, sizeof(req), false);
}

int mt7668_mcu_fw_log_2_host(struct mt7668_dev *dev, bool enable)
{
	struct {
		u8 ctrl;
		u8 mcu_dest;	/* 0: N9 */
		u8 level;
		u8 rsv;
		__le32 host_sec;
		__le32 host_msec;
	} req = {
		.ctrl = enable,
	};

	dev->fw_debug = enable;

	return mt76_mcu_send_msg(&dev->mt76, MT7668_CE_CMD(FW_LOG_2_HOST),
				 &req, sizeof(req), false);
}

int mt7668_mcu_set_rx_filter(struct mt7668_dev *dev, u32 filter)
{
	struct {
		__le32 filter;
		u8 rsv[64];
	} req = {
		.filter = cpu_to_le32(filter),
	};

	return mt76_mcu_send_msg(&dev->mt76, MT7668_CE_CMD(SET_RX_FILTER),
				 &req, sizeof(req), false);
}

int mt7668_mcu_ch_privilege(struct mt7668_dev *dev, struct ieee80211_vif *vif,
			    bool request)
{
	struct mt7668_vif *mvif = (struct mt7668_vif *)vif->drv_priv;
	struct cfg80211_chan_def *chandef = &vif->bss_conf.chanreq.oper;
	struct mt7668_ch_privilege req = {
		.bss_idx = mvif->mt76.idx,
		.action = !request,
		.req_type = MT7668_CH_REQ_TYPE_JOIN,
		.dbdc_band = MT7668_DBDC_BAND_AUTO,
		.max_interval = cpu_to_le32(MT7668_CH_JOIN_INTERVAL),
	};
	int ret;

	if (!chandef->chan)
		return -EINVAL;

	if (!request && !mvif->ch_requested)
		return 0;

	if (request)
		mvif->ch_token++;
	req.token = mvif->ch_token;

	req.primary_channel = chandef->chan->hw_value;
	req.band = mt7668_band(chandef->chan->band);
	mt7668_chandef_params(chandef, &req.sco, &req.channel_width,
			      &req.center_seg1, &req.center_seg2);

	reinit_completion(&dev->ch_grant);
	ret = mt76_mcu_send_msg(&dev->mt76, MT7668_CE_CMD(CH_PRIVILEGE),
				&req, sizeof(req), false);
	if (ret || !request) {
		mvif->ch_requested = false;
		return ret;
	}

	mvif->ch_requested = true;
	if (!wait_for_completion_timeout(&dev->ch_grant, HZ)) {
		dev_err(dev->mt76.dev, "channel %d not granted\n",
			req.primary_channel);
		return -ETIMEDOUT;
	}

	return 0;
}

int mt7668_mcu_sta_update(struct mt7668_dev *dev, struct ieee80211_vif *vif,
			  struct ieee80211_sta *sta, u8 state)
{
	struct mt7668_vif *mvif = (struct mt7668_vif *)vif->drv_priv;
	struct mt76_wcid *wcid = (struct mt76_wcid *)sta->drv_priv;
	struct ieee80211_link_sta *link = &sta->deflink;
	struct ieee80211_channel *chan = vif->bss_conf.chanreq.oper.chan;
	enum nl80211_band band = chan ? chan->band : NL80211_BAND_2GHZ;
	struct ieee80211_supported_band *sband;
	struct mt7668_sta_rec req = {
		.sta_idx = wcid->idx,
		.sta_type = MT7668_STA_TYPE_LEGACY_AP,
		.aid = cpu_to_le16(vif->cfg.aid),
		.listen_interval =
			cpu_to_le16(dev->mphy.hw->conf.listen_interval),
		.bss_idx = mvif->mt76.idx,
		.is_qos = sta->wme,
		.is_uapsd = sta->wme && sta->uapsd_queues,
		.need_resp = state == MT7668_STA_STATE_3,
		.sta_state = state,
		.wlan_idx = wcid->idx,
		.bmc_wlan_idx = 0xff,
		.rts_policy = 3,
		.tx_ampdu = 1,
		.rx_ampdu = 1,
		.tx_ba_size = 64,
		.rx_ba_size = 32,
	};

	sband = band == NL80211_BAND_2GHZ ? &dev->mphy.sband_2g.sband :
					    &dev->mphy.sband_5g.sband;
	ether_addr_copy(req.mac_addr, sta->addr);

	req.rates = cpu_to_le16(mt7668_rate_set(sband,
						link->supp_rates[band]));
	req.basic_rates = cpu_to_le16(mt7668_rate_set(sband,
					vif->bss_conf.basic_rates));
	req.default_fixed_rate = cpu_to_le16(band == NL80211_BAND_2GHZ ?
					     MT7668_RATE_CCK_1M :
					     MT7668_RATE_OFDM_6M);
	req.tx_amsdu_in_ampdu = 1;
	req.rx_amsdu_in_ampdu = 1;
	req.tx_max_amsdu_in_ampdu_len = cpu_to_le32(4096);

	if (band == NL80211_BAND_2GHZ) {
		req.phy_type_set = MT7668_PHY_TYPE_HR_DSSS;
		if (link->supp_rates[band] & GENMASK(11, 4))
			req.phy_type_set |= MT7668_PHY_TYPE_ERP;
	} else {
		req.phy_type_set = MT7668_PHY_TYPE_OFDM;
	}

	if (link->ht_cap.ht_supported) {
		struct ieee80211_sta_ht_cap *ht = &link->ht_cap;

		req.phy_type_set |= MT7668_PHY_TYPE_HT;
		req.ht_cap_info = cpu_to_le16(ht->cap);
		req.ampdu_param = ht->ampdu_factor |
				  (ht->ampdu_density << 2);
		memcpy(req.rx_mcs_bitmask, ht->mcs.rx_mask,
		       sizeof(req.rx_mcs_bitmask));
		req.rx_highest_rate = ht->mcs.rx_highest;
		req.mcs_set = ht->mcs.rx_mask[0];
		req.sup_mcs32 = !!(ht->mcs.rx_mask[4] & BIT(0));
	}

	if (link->vht_cap.vht_supported) {
		struct ieee80211_sta_vht_cap *vht = &link->vht_cap;

		req.phy_type_set |= MT7668_PHY_TYPE_VHT;
		req.vht_cap_info = cpu_to_le32(vht->cap);
		req.vht_rx_mcs_map = vht->vht_mcs.rx_mcs_map;
		req.vht_rx_highest_rate = vht->vht_mcs.rx_highest;
		req.vht_tx_mcs_map = vht->vht_mcs.tx_mcs_map;
		req.vht_tx_highest_rate = vht->vht_mcs.tx_highest;
	}

	/* the final update is acknowledged (event 0x0c) */
	return mt76_mcu_send_msg(&dev->mt76, MT7668_CE_CMD(UPDATE_STA_RECORD),
				 &req, sizeof(req), req.need_resp);
}

int mt7668_mcu_sta_remove(struct mt7668_dev *dev, struct ieee80211_vif *vif,
			  struct ieee80211_sta *sta)
{
	struct mt7668_vif *mvif = (struct mt7668_vif *)vif->drv_priv;
	struct mt76_wcid *wcid = (struct mt76_wcid *)sta->drv_priv;
	struct mt7668_sta_remove req = {
		.sta_idx = wcid->idx,
		.bss_idx = mvif->mt76.idx,
	};

	return mt76_mcu_send_msg(&dev->mt76, MT7668_CE_CMD(REMOVE_STA_RECORD),
				 &req, sizeof(req), false);
}

int mt7668_mcu_set_bss_info(struct mt7668_dev *dev, struct ieee80211_vif *vif,
			    bool connected)
{
	struct mt7668_vif *mvif = (struct mt7668_vif *)vif->drv_priv;
	struct ieee80211_bss_conf *conf = &vif->bss_conf;
	struct cfg80211_chan_def *chandef = &conf->chanreq.oper;
	struct mt7668_bss_info req = {
		.bss_idx = mvif->mt76.idx,
		.connection_state = connected ? MT7668_MEDIA_STATE_CONNECTED :
						MT7668_MEDIA_STATE_DISCONNECTED,
		.op_mode = MT7668_OP_MODE_INFRASTRUCTURE,
		.is_qbss = conf->qos || (mvif->ap_sta && mvif->ap_sta->wme),
		.sta_rec_idx_ap = 0xfe,
		.auth_mode = MT7668_AUTH_MODE_OPEN,
		.enc_status = MT7668_ENC_STATUS_DISABLED,
		.bmc_wlan_idx = mvif->bmc_wcid.idx,
		.nss = 2,
	};
	struct ieee80211_supported_band *sband;
	struct ieee80211_sta *sta;
	enum nl80211_band band;
	u8 sco, width, seg1, seg2;

	if (!chandef->chan)
		return -EINVAL;

	band = chandef->chan->band;
	sband = band == NL80211_BAND_2GHZ ? &dev->mphy.sband_2g.sband :
					    &dev->mphy.sband_5g.sband;

	req.ssid_len = min_t(u8, vif->cfg.ssid_len, IEEE80211_MAX_SSID_LEN);
	memcpy(req.ssid, vif->cfg.ssid, req.ssid_len);
	ether_addr_copy(req.bssid, conf->bssid);
	req.basic_rates = cpu_to_le16(mt7668_rate_set(sband,
						      conf->basic_rates));

	if (band == NL80211_BAND_2GHZ) {
		req.default_fixed_rate = cpu_to_le16(MT7668_RATE_CCK_1M);
		req.non_ht_basic_phy_type = MT7668_PHY_TYPE_ERP_INDEX;
		req.phy_type_set = MT7668_PHY_TYPE_HR_DSSS |
				   MT7668_PHY_TYPE_ERP;
	} else {
		req.default_fixed_rate = cpu_to_le16(MT7668_RATE_OFDM_6M);
		req.non_ht_basic_phy_type = MT7668_PHY_TYPE_OFDM_INDEX;
		req.phy_type_set = MT7668_PHY_TYPE_OFDM;
	}

	sta = mvif->ap_sta;
	if (sta) {
		struct mt76_wcid *wcid = (struct mt76_wcid *)sta->drv_priv;

		req.sta_rec_idx_ap = wcid->idx;
		req.operational_rates =
			cpu_to_le16(mt7668_rate_set(sband,
					sta->deflink.supp_rates[band]));
		if (sta->deflink.ht_cap.ht_supported)
			req.phy_type_set |= MT7668_PHY_TYPE_HT;
		if (sta->deflink.vht_cap.vht_supported)
			req.phy_type_set |= MT7668_PHY_TYPE_VHT;
	}

	rcu_read_lock();
	if (conf->bss) {
		const struct element *ht = NULL;
		const struct cfg80211_bss_ies *ies;

		ies = rcu_dereference(conf->bss->ies);
		if (ies) {
			if (cfg80211_find_elem(WLAN_EID_RSN, ies->data, ies->len)) {
				/* WPA2-PSK with CCMP as far as the firmware cares */
				req.auth_mode = MT7668_AUTH_MODE_WPA2_PSK;
				req.enc_status = MT7668_ENC_STATUS_CCMP;
			}
			ht = cfg80211_find_elem(WLAN_EID_HT_OPERATION, ies->data,
						ies->len);
		}
		if (ht && ht->datalen >= sizeof(struct ieee80211_ht_operation)) {
			const struct ieee80211_ht_operation *op =
				(const void *)ht->data;

			req.rlm.ht_op_info1 = op->ht_param;
			req.rlm.ht_op_info3 = op->stbc_param;
			req.rlm.gf_operation = 1;
			req.rlm.tx_rifs = 1;
		}
	}
	rcu_read_unlock();

	mt7668_chandef_params(chandef, &sco, &width, &seg1, &seg2);
	req.rlm.bss_idx = mvif->mt76.idx;
	req.rlm.band = mt7668_band(band);
	req.rlm.primary_channel = chandef->chan->hw_value;
	req.rlm.sco = sco;
	req.rlm.ht_op_info2 = cpu_to_le16(conf->ht_operation_mode);
	req.rlm.short_preamble = conf->use_short_preamble;
	req.rlm.short_slot = conf->use_short_slot;
	req.rlm.vht_channel_width = width;
	req.rlm.vht_center_seg1 = seg1;
	req.rlm.vht_center_seg2 = seg2;
	req.rlm.vht_basic_mcs = req.basic_rates;
	req.rlm.nss = 2;

	return mt76_mcu_send_msg(&dev->mt76, MT7668_CE_CMD(SET_BSS_INFO),
				 &req, sizeof(req), false);
}

int mt7668_mcu_set_bss_pm(struct mt7668_dev *dev, struct ieee80211_vif *vif,
			  bool connected)
{
	struct mt7668_vif *mvif = (struct mt7668_vif *)vif->drv_priv;
	struct {
		u8 bss_idx;
		u8 dtim_period;
		__le16 aid;
		__le16 bcn_interval;
		__le16 atim_window;
		u8 uapsd;
		u8 bmc_delivered_ac;
		u8 bmc_triggered_ac;
		u8 pad;
	} req = {
		.bss_idx = mvif->mt76.idx,
		.dtim_period = vif->bss_conf.dtim_period,
		.aid = cpu_to_le16(vif->cfg.aid),
		.bcn_interval = cpu_to_le16(vif->bss_conf.beacon_int),
	};
	struct {
		u8 bss_idx;
		u8 pad[3];
	} abort = {
		.bss_idx = mvif->mt76.idx,
	};

	if (!connected)
		return mt76_mcu_send_msg(&dev->mt76,
					 MT7668_CE_CMD(PM_BSS_ABORT),
					 &abort, sizeof(abort), false);

	return mt76_mcu_send_msg(&dev->mt76, MT7668_CE_CMD(PM_BSS_CONNECTED),
				 &req, sizeof(req), false);
}

int mt7668_mcu_cancel_hw_scan(struct mt7668_dev *dev,
			      struct ieee80211_vif *vif)
{
	struct mt7668_vif *mvif = (struct mt7668_vif *)vif->drv_priv;
	struct {
		u8 seq_num;
		u8 is_ext_channel;
		u8 rsv[2];
	} req = {
		.seq_num = mvif->mt76.scan_seq_num,
	};
	struct cfg80211_scan_info info = {
		.aborted = true,
	};
	int ret;

	ret = mt76_mcu_send_msg(&dev->mt76, MT7668_CE_CMD(SCAN_CANCEL),
				&req, sizeof(req), false);

	/* a scan-done event arriving later finds the scan already completed */
	if (test_and_clear_bit(MT76_HW_SCANNING, &dev->mphy.state))
		ieee80211_scan_completed(dev->mphy.hw, &info);

	return ret;
}

int mt7668_mcu_init(struct mt7668_dev *dev)
{
	static const struct mt76_mcu_ops mt7668_mcu_ops = {
		.headroom = sizeof(struct mt76_connac2_mcu_txd),
		.tailroom = 8,
		.mcu_skb_send_msg = mt7668_mcu_send_message,
		.mcu_parse_response = mt7668_mcu_parse_response,
	};
	int ret;

	dev->mt76.mcu_ops = &mt7668_mcu_ops;
	dev->mt76.mcu.timeout = 3 * HZ;

	ret = mt7668_load_firmware(dev);
	if (ret)
		return ret;

	set_bit(MT76_STATE_MCU_RUNNING, &dev->mphy.state);

	/* the firmware drops the first command after start; send a dummy */
	ret = mt76_mcu_send_msg(&dev->mt76, MT7668_CMD(DUMMY),
				NULL, 0, false);
	if (ret)
		goto err;

	ret = mt7668_mcu_get_nic_capability(dev);
	if (ret)
		goto err;

	ret = mt7668_mcu_get_nic_capability_v2(dev);
	if (ret)
		goto err;

	ret = mt7668_mcu_basic_config(dev);
	if (ret)
		goto err;

	ret = mt7668_mcu_set_efuse_mode(dev);
	if (ret)
		goto err;

	return 0;

err:
	clear_bit(MT76_STATE_MCU_RUNNING, &dev->mphy.state);
	return ret;
}
