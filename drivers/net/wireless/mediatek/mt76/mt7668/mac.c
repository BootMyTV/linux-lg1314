// SPDX-License-Identifier: ISC

#include <linux/etherdevice.h>

#include "mt7668.h"
#include "mcu.h"

#define MT_RXV1_TX_RATE			GENMASK(6, 0)
#define MT_RXV1_HT_STBC			GENMASK(8, 7)
#define MT_RXV1_HT_AD_CODE		BIT(9)
#define MT_RXV1_TX_MODE			GENMASK(14, 12)
#define MT_RXV1_FRAME_MODE		GENMASK(16, 15)
#define MT_RXV1_HT_SHORT_GI		BIT(19)
#define MT_RXV2_NSTS			GENMASK(29, 27)
#define MT_RXV4_RCPI0			GENMASK(7, 0)
#define MT_RXV4_RCPI1			GENMASK(15, 8)

#define to_rssi(field, rxv)		((FIELD_GET(field, rxv) - 220) / 2)

/*
 * The firmware converts received data frames to 802.3, EAPOL included.
 * mac80211 only takes 802.3 frames on its fast path, which needs an
 * authorized station, so rebuild the 802.11 header.
 */
static int
mt7668_mac_rx_untranslate(struct sk_buff *skb, struct mt76_wcid *wcid,
			  bool has_g4, u16 fc, u16 seq, u16 qos)
{
	struct ieee80211_sta *sta;
	struct ieee80211_hdr hdr = {};
	struct ethhdr eth;
	int need;

	if (skb->len < ETH_HLEN)
		return -EINVAL;

	sta = container_of((void *)wcid, struct ieee80211_sta, drv_priv);
	memcpy(&eth, skb->data, ETH_HLEN);

	if (!has_g4)
		fc = IEEE80211_FTYPE_DATA | IEEE80211_STYPE_DATA |
		     IEEE80211_FCTL_FROMDS;

	/* station mode: frames come from the AP */
	if ((fc & (IEEE80211_FCTL_TODS | IEEE80211_FCTL_FROMDS)) !=
	    IEEE80211_FCTL_FROMDS)
		return -EINVAL;

	fc &= ~(IEEE80211_FCTL_ORDER | IEEE80211_FCTL_PROTECTED);
	hdr.frame_control = cpu_to_le16(fc);
	hdr.seq_ctrl = cpu_to_le16(seq);
	ether_addr_copy(hdr.addr1, eth.h_dest);
	ether_addr_copy(hdr.addr2, sta->addr);
	ether_addr_copy(hdr.addr3, eth.h_source);

	need = 24 + 6 + (ieee80211_is_data_qos(hdr.frame_control) ? 2 : 0);
	if (skb_cow_head(skb, need))
		return -ENOMEM;

	/* keep the ethertype, put LLC/SNAP in front of it */
	skb_pull(skb, ETH_HLEN - 2);
	if (eth.h_proto == cpu_to_be16(ETH_P_AARP) ||
	    eth.h_proto == cpu_to_be16(ETH_P_IPX))
		memcpy(skb_push(skb, 6), bridge_tunnel_header, 6);
	else if (be16_to_cpu(eth.h_proto) >= ETH_P_802_3_MIN)
		memcpy(skb_push(skb, 6), rfc1042_header, 6);
	else
		skb_pull(skb, 2);

	if (ieee80211_is_data_qos(hdr.frame_control)) {
		__le16 qos_ctl = cpu_to_le16(qos);

		memcpy(skb_push(skb, IEEE80211_QOS_CTL_LEN), &qos_ctl,
		       IEEE80211_QOS_CTL_LEN);
	}
	memcpy(skb_push(skb, 24), &hdr, 24);

	return 0;
}

static int mt7668_mac_fill_rx(struct mt7668_dev *dev, struct sk_buff *skb)
{
	struct mt76_rx_status *status = (struct mt76_rx_status *)skb->cb;
	struct mt76_phy *mphy = &dev->mphy;
	struct ieee80211_supported_band *sband;
	__le32 *rxd = (__le32 *)skb->data;
	u32 rxd0 = le32_to_cpu(rxd[0]);
	u32 rxd1 = le32_to_cpu(rxd[1]);
	u32 rxd2 = le32_to_cpu(rxd[2]);
	bool hdr_trans = rxd1 & MT_RXD1_NORMAL_HDR_TRANS;
	u16 g4_fc = 0, g4_seq = 0, g4_qos = 0;
	bool has_g4 = false;
	struct mt76_wcid *wcid;
	u8 chfreq, idx;
	int i;

	memset(status, 0, sizeof(*status));

	if (!test_bit(MT76_STATE_RUNNING, &mphy->state))
		return -EINVAL;

	if (rxd2 & (MT_RXD2_NORMAL_MAX_LEN_ERROR | MT_RXD2_NORMAL_AMSDU_ERR))
		return -EINVAL;

	if (hdr_trans && (rxd2 & (MT_RXD2_NORMAL_CM | MT_RXD2_NORMAL_FRAG)))
		return -EINVAL;

	idx = FIELD_GET(MT_RXD2_NORMAL_WLAN_IDX, rxd2);
	wcid = idx < ARRAY_SIZE(dev->mt76.wcid) ?
	       rcu_dereference(dev->mt76.wcid[idx]) : NULL;
	if (wcid && wcid->sta) {
		status->wcid = wcid;
	} else if (wcid) {
		/* group frames decrypted on the BSS entry come from the AP */
		struct mt7668_vif *mvif;

		mvif = container_of(wcid, struct mt7668_vif, bmc_wcid);
		if (mvif->ap_sta)
			status->wcid = (struct mt76_wcid *)mvif->ap_sta->drv_priv;
	}

	/* translated frames need the station they came from */
	if (hdr_trans && !status->wcid)
		return -EINVAL;

	if (rxd2 & MT_RXD2_NORMAL_FCS_ERR)
		status->flag |= RX_FLAG_FAILED_FCS_CRC;

	if (rxd2 & (MT_RXD2_NORMAL_ICV_ERR | MT_RXD2_NORMAL_TKIP_MIC_ERR))
		status->flag |= RX_FLAG_ONLY_MONITOR;

	/* decrypted by the hardware, IV and MIC removed */
	if (FIELD_GET(MT_RXD2_NORMAL_SEC_MODE, rxd2) &&
	    !(rxd2 & (MT_RXD2_NORMAL_CLM | MT_RXD2_NORMAL_CM)))
		status->flag |= RX_FLAG_DECRYPTED | RX_FLAG_IV_STRIPPED |
				RX_FLAG_MMIC_STRIPPED | RX_FLAG_MIC_STRIPPED;

	chfreq = FIELD_GET(MT_RXD1_NORMAL_CH_FREQ, rxd1);
	status->band = chfreq > 14 ? NL80211_BAND_5GHZ : NL80211_BAND_2GHZ;
	status->freq = ieee80211_channel_to_frequency(chfreq, status->band);
	sband = status->band == NL80211_BAND_5GHZ ? &mphy->sband_5g.sband :
						    &mphy->sband_2g.sband;
	if (!sband->channels)
		return -EINVAL;

	rxd += 4;
	if (rxd0 & MT_RXD0_NORMAL_GROUP_4) {
		/* the original frame control, sequence and QoS control */
		g4_fc = le32_to_cpu(rxd[0]) & 0xffff;
		g4_seq = le32_to_cpu(rxd[2]) & 0xffff;
		g4_qos = le32_to_cpu(rxd[2]) >> 16;
		has_g4 = true;
		rxd += 4;
	}
	if (rxd0 & MT_RXD0_NORMAL_GROUP_1) {
		u8 *data = (u8 *)rxd;

		/* packet number, for mac80211's replay check */
		if (status->flag & RX_FLAG_DECRYPTED) {
			switch (FIELD_GET(MT_RXD2_NORMAL_SEC_MODE, rxd2)) {
			case MT_CIPHER_AES_CCMP:
			case MT_CIPHER_TKIP:
				for (i = 0; i < 6; i++)
					status->iv[i] = data[5 - i];
				break;
			default:
				break;
			}
		}
		rxd += 4;
	}
	if (rxd0 & MT_RXD0_NORMAL_GROUP_2)
		rxd += 2;

	if (rxd0 & MT_RXD0_NORMAL_GROUP_3) {
		u32 rxdg0 = le32_to_cpu(rxd[0]);
		u32 rxdg1 = le32_to_cpu(rxd[1]);
		u32 rxdg3 = le32_to_cpu(rxd[3]);
		u8 stbc = FIELD_GET(MT_RXV1_HT_STBC, rxdg0);
		bool cck = false;

		i = FIELD_GET(MT_RXV1_TX_RATE, rxdg0);
		switch (FIELD_GET(MT_RXV1_TX_MODE, rxdg0)) {
		case MT_PHY_TYPE_CCK:
			cck = true;
			fallthrough;
		case MT_PHY_TYPE_OFDM:
			i = mt76_get_rate(&dev->mt76, sband, i, cck);
			break;
		case MT_PHY_TYPE_HT_GF:
		case MT_PHY_TYPE_HT:
			status->encoding = RX_ENC_HT;
			if (i > 31)
				return -EINVAL;
			break;
		case MT_PHY_TYPE_VHT:
			status->nss = FIELD_GET(MT_RXV2_NSTS, rxdg1) + 1;
			status->encoding = RX_ENC_VHT;
			break;
		default:
			return -EINVAL;
		}
		status->rate_idx = i;

		switch (FIELD_GET(MT_RXV1_FRAME_MODE, rxdg0)) {
		case MT_PHY_BW_20:
			break;
		case MT_PHY_BW_40:
			status->bw = RATE_INFO_BW_40;
			break;
		case MT_PHY_BW_80:
			status->bw = RATE_INFO_BW_80;
			break;
		default:
			return -EINVAL;
		}

		if (rxdg0 & MT_RXV1_HT_SHORT_GI)
			status->enc_flags |= RX_ENC_FLAG_SHORT_GI;
		if (rxdg0 & MT_RXV1_HT_AD_CODE)
			status->enc_flags |= RX_ENC_FLAG_LDPC;

		status->enc_flags |= RX_ENC_FLAG_STBC_MASK * stbc;

		status->chains = mphy->antenna_mask;
		status->chain_signal[0] = to_rssi(MT_RXV4_RCPI0, rxdg3);
		status->chain_signal[1] = to_rssi(MT_RXV4_RCPI1, rxdg3);

		rxd += 6;
	}

	if ((u8 *)rxd - skb->data >= skb->len)
		return -EINVAL;

	skb_pull(skb, (u8 *)rxd - skb->data);
	if (rxd1 & MT_RXD1_NORMAL_HDR_OFFSET)
		skb_pull(skb, 2);

	if (hdr_trans) {
		/*
		 * A failed translation leaves a 2-byte length after the
		 * ethertype (no LLC-SNAP match, or a VLAN tag).
		 */
		if (rxd2 & MT_RXD2_NORMAL_HDR_TRANS_ERROR) {
			int pad_start = 12;

			if (skb->len < pad_start + 6)
				return -EINVAL;
			if (get_unaligned_be16(skb->data + pad_start) ==
			    ETH_P_8021Q)
				pad_start += 4;
			else
				pad_start = 0;
			if (pad_start) {
				memmove(skb->data + 2, skb->data, pad_start);
				skb_pull(skb, 2);
			}
		}
		return mt7668_mac_rx_untranslate(skb, status->wcid, has_g4,
						 g4_fc, g4_seq, g4_qos);
	}

	if (skb->len < sizeof(struct ieee80211_hdr_3addr) - ETH_ALEN)
		return -EINVAL;


	return 0;
}

void mt7668_queue_rx_skb(struct mt76_dev *mdev, enum mt76_rxq_id q,
			 struct sk_buff *skb, u32 *info)
{
	struct mt7668_dev *dev = container_of(mdev, struct mt7668_dev, mt76);
	__le32 *rxd = (__le32 *)skb->data;
	u32 rxd0, type;

	if (skb->len < 4 * sizeof(*rxd))
		goto free;

	rxd0 = le32_to_cpu(rxd[0]);
	type = FIELD_GET(MT_RXD0_PKT_TYPE, rxd0);
	if (type == PKT_TYPE_RX_EVENT) {
		switch (rxd0 & MT_RXD0_SW_PKT_TYPE_MASK) {
		case MT_RXD0_SW_PKT_TYPE_EVENT:
			mt7668_mcu_rx_event(dev, skb);
			return;
		case MT_RXD0_SW_PKT_TYPE_FRAME:
			type = PKT_TYPE_NORMAL;
			break;
		default:
			goto free;
		}
	}

	switch (type) {
	case PKT_TYPE_NORMAL: {
		int err;

		rcu_read_lock();
		err = mt7668_mac_fill_rx(dev, skb);
		rcu_read_unlock();
		if (!err) {
			mt76_rx(&dev->mt76, q, skb);
			return;
		}
		break;
	}
	default:
		break;
	}

free:
	dev_kfree_skb(skb);
}

static const u8 mt7668_lmac_queue[] = {
	[IEEE80211_AC_BK] = 0,
	[IEEE80211_AC_BE] = 1,
	[IEEE80211_AC_VI] = 2,
	[IEEE80211_AC_VO] = 3,
};

/* 802.11 data to 802.3, as the firmware wants data frames */
static int mt7668_tx_to_8023(struct sk_buff *skb, u8 *tid)
{
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *)skb->data;
	int hdrlen = ieee80211_hdrlen(hdr->frame_control);
	u8 addr[2 * ETH_ALEN];
	u8 *payload;

	/*
	 * The hardware encrypts: with a hardware key mac80211 only marks the
	 * frame protected. Anything it encrypted itself can't be converted.
	 */
	if ((ieee80211_has_protected(hdr->frame_control) &&
	     !IEEE80211_SKB_CB(skb)->control.hw_key) || skb->len < hdrlen)
		return -EOPNOTSUPP;

	if (ieee80211_is_data_qos(hdr->frame_control))
		*tid = *ieee80211_get_qos_ctl(hdr) & IEEE80211_QOS_CTL_TID_MASK;

	ether_addr_copy(addr, ieee80211_get_DA(hdr));
	ether_addr_copy(addr + ETH_ALEN, ieee80211_get_SA(hdr));

	payload = skb->data + hdrlen;
	if (skb->len >= hdrlen + 8 &&
	    (ether_addr_equal(payload, rfc1042_header) ||
	     ether_addr_equal(payload, bridge_tunnel_header))) {
		/* keep the ethertype */
		skb_pull(skb, hdrlen + 6);
	} else {
		__be16 len = cpu_to_be16(skb->len - hdrlen);

		skb_pull(skb, hdrlen);
		memcpy(skb_push(skb, sizeof(len)), &len, sizeof(len));
	}
	memcpy(skb_push(skb, sizeof(addr)), addr, sizeof(addr));

	return 0;
}

/* data frame: TXD, the CR4 header, then the 802.3 frame */
struct mt7668_tx_ctl {
	struct mt76_vif_link *mvif;
	bool hw_key;
	bool fixed_rate;
};

/*
 * mt76's TX status bookkeeping shares space with info->control, so read
 * what we need from it first.
 */
static void mt7668_tx_ctl(struct sk_buff *skb, struct mt7668_tx_ctl *ctl)
{
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
	struct ieee80211_vif *vif = info->control.vif;

	ctl->mvif = vif ? (struct mt76_vif_link *)vif->drv_priv : NULL;
	ctl->hw_key = !!info->control.hw_key;
	ctl->fixed_rate = info->control.flags & IEEE80211_TX_CTRL_PORT_CTRL_PROTO ||
			  info->flags & IEEE80211_TX_CTL_USE_MINRATE;
}

static void
mt7668_mac_write_txwi_8023(struct mt7668_dev *dev, struct sk_buff *skb,
			   struct mt76_wcid *wcid, int pid, u8 tid,
			   struct mt7668_tx_ctl *ctl)
{
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
	struct mt76_vif_link *mvif = ctl->mvif;
	__le32 *txwi = (__le32 *)skb->data;
	u8 *txp = skb->data + MT_TXD_SIZE;
	/* station mode sends everything to the AP; only the BSS entry is group */
	bool multicast = !wcid->sta;
	bool fixed_rate = ctl->fixed_rate;
	u32 val;

	val = FIELD_PREP(MT_TXD0_TX_BYTES, skb->len) |
	      FIELD_PREP(MT_TXD0_ETH_TYPE_OFFSET, MT_TXD0_ETH_TYPE_OFFSET_802_3) |
	      FIELD_PREP(MT_TXD0_Q_IDX,
			 mt7668_lmac_queue[skb_get_queue_mapping(skb)]);
	txwi[0] = cpu_to_le32(val);

	val = FIELD_PREP(MT_TXD1_WLAN_IDX, wcid->idx) |
	      MT_TXD1_LONG_FORMAT | MT_TXD1_ETH_802_3 |
	      FIELD_PREP(MT_TXD1_HDR_FORMAT, MT_HDR_FORMAT_802_3) |
	      FIELD_PREP(MT_TXD1_TID, tid) |
	      FIELD_PREP(MT_TXD1_OWN_MAC, mvif ? mvif->omac_idx : 0);
	txwi[1] = cpu_to_le32(val);

	val = 0;
	if (multicast)
		val |= MT_TXD2_MULTICAST;
	if (fixed_rate)
		val |= MT_TXD2_FIX_RATE;
	txwi[2] = cpu_to_le32(val);

	val = FIELD_PREP(MT_TXD3_REM_TX_COUNT, MT_TX_RETRY_LIMIT);
	if (multicast || (info->flags & IEEE80211_TX_CTL_NO_ACK))
		val |= MT_TXD3_NO_ACK;
	/* encrypt with the key in the station's WTBL entry */
	if (ctl->hw_key)
		val |= MT_TXD3_PROTECT_FRAME;
	txwi[3] = cpu_to_le32(val);

	txwi[4] = 0;

	val = FIELD_PREP(MT_TXD5_PID, pid);
	if (pid >= MT_PACKET_ID_FIRST)
		val |= MT_TXD5_TX_STATUS_MCU;
	txwi[5] = cpu_to_le32(val);

	val = 0;
	if (fixed_rate)
		val = FIELD_PREP(MT_TXD6_TX_RATE,
				 info->band == NL80211_BAND_2GHZ ?
				 MT7668_RATE_CCK_1M : MT7668_RATE_OFDM_6M);
	txwi[6] = cpu_to_le32(val);
	txwi[7] = 0;

	memset(txp, 0, MT_TXP_SIZE);
	txp[0] = 1;
	txp[4] = mvif ? mvif->idx : 0;
}

/* the TXD has been pushed in front of the 802.11 frame */
static void
mt7668_mac_write_txwi(struct mt7668_dev *dev, struct sk_buff *skb,
		      struct mt76_wcid *wcid, enum mt76_txq_id qid, int pid,
		      struct mt7668_tx_ctl *ctl)
{
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
	struct ieee80211_hdr *hdr;
	__le32 *txwi = (__le32 *)skb->data;
	bool mcu = qid == MT_TXQ_PSD;
	bool multicast;
	__le16 fc;
	int hdr_len;
	u8 omac_idx = 0, tid = 0;
	u32 val;

	hdr = (struct ieee80211_hdr *)(skb->data + MT_TXD_SIZE);
	fc = hdr->frame_control;
	hdr_len = ieee80211_hdrlen(fc);
	multicast = is_multicast_ether_addr(hdr->addr1);

	if (ctl->mvif)
		omac_idx = ctl->mvif->omac_idx;

	if (ieee80211_is_data_qos(fc))
		tid = *ieee80211_get_qos_ctl(hdr) & IEEE80211_QOS_CTL_TID_MASK;

	/*
	 * Frames outside the data queues go to the MCU as "command" packets,
	 * at the lowest rate and with a lifetime.
	 */
	val = FIELD_PREP(MT_TXD0_TX_BYTES, skb->len);
	if (mcu)
		val |= MT_TXD0_P_IDX |
		       FIELD_PREP(MT_TXD0_Q_IDX, MT_TX_MCU_PORT_RX_Q1 |
						 MT_TX_MCU_Q_IDX_H2CDMA) |
		       FIELD_PREP(MT_TXD0_ETH_TYPE_OFFSET, (hdr_len + 4) / 2);
	else
		val |= FIELD_PREP(MT_TXD0_Q_IDX,
				  mt7668_lmac_queue[skb_get_queue_mapping(skb)]);
	txwi[0] = cpu_to_le32(val);

	val = FIELD_PREP(MT_TXD1_WLAN_IDX, wcid->idx) |
	      MT_TXD1_LONG_FORMAT |
	      FIELD_PREP(MT_TXD1_HDR_FORMAT, MT_HDR_FORMAT_802_11) |
	      FIELD_PREP(MT_TXD1_HDR_INFO, hdr_len / 2) |
	      FIELD_PREP(MT_TXD1_TID, tid) |
	      FIELD_PREP(MT_TXD1_OWN_MAC, omac_idx);
	if (mcu)
		val |= FIELD_PREP(MT_TXD1_PKT_FMT, MT_TX_TYPE_CMD);
	txwi[1] = cpu_to_le32(val);

	val = FIELD_PREP(MT_TXD2_FRAME_TYPE, (le16_to_cpu(fc) >> 2) & 0x3) |
	      FIELD_PREP(MT_TXD2_SUB_TYPE, (le16_to_cpu(fc) >> 4) & 0xf);
	if (multicast)
		val |= MT_TXD2_MULTICAST;
	if (mcu)
		val |= MT_TXD2_FIX_RATE |
		       FIELD_PREP(MT_TXD2_MAX_TX_TIME, MT_TX_MGMT_LIFETIME);
	txwi[2] = cpu_to_le32(val);

	val = FIELD_PREP(MT_TXD3_REM_TX_COUNT, MT_TX_RETRY_LIMIT);
	if (multicast || (info->flags & IEEE80211_TX_CTL_NO_ACK))
		val |= MT_TXD3_NO_ACK;
	if (mcu)
		val |= MT_TXD3_FORCE_TX;
	/* management through the MCU gets its sequence number from the MAC */
	if (!mcu && !ieee80211_is_data_qos(fc)) {
		u16 seq = le16_to_cpu(hdr->seq_ctrl);

		val |= MT_TXD3_SN_VALID |
		       FIELD_PREP(MT_TXD3_SEQ, IEEE80211_SEQ_TO_SN(seq));
	}
	txwi[3] = cpu_to_le32(val);

	txwi[4] = 0;

	val = FIELD_PREP(MT_TXD5_PID, pid);
	if (pid >= MT_PACKET_ID_FIRST)
		val |= MT_TXD5_TX_STATUS_MCU;
	txwi[5] = cpu_to_le32(val);

	val = 0;
	if (mcu)
		val = FIELD_PREP(MT_TXD6_TX_RATE,
				 info->band == NL80211_BAND_2GHZ ?
				 MT7668_RATE_CCK_1M : MT7668_RATE_OFDM_6M);
	txwi[6] = cpu_to_le32(val);
	txwi[7] = cpu_to_le32(FIELD_PREP(MT_TXD7_SPE_IDX, MT_TXD7_SPE_IDX_ALL));
}

int mt7668_tx_prepare_skb(struct mt76_dev *mdev, void *txwi_ptr,
			  enum mt76_txq_id qid, struct mt76_wcid *wcid,
			  struct ieee80211_sta *sta,
			  struct mt76_tx_info *tx_info)
{
	struct mt7668_dev *dev = container_of(mdev, struct mt7668_dev, mt76);
	struct sk_buff *skb = tx_info->skb;
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *)skb->data;
	int pid, pad, err;
	u8 tid = skb->priority & IEEE80211_QOS_CTL_TID_MASK;
	struct mt7668_tx_ctl ctl;

	if (!wcid)
		wcid = &mdev->global_wcid;

	if (qid != MT_TXQ_PSD && ieee80211_is_data_present(hdr->frame_control)) {
		err = mt7668_tx_to_8023(skb, &tid);
		if (err)
			return err;

		mt7668_tx_ctl(skb, &ctl);
		pid = mt76_tx_status_skb_add(mdev, wcid, skb);
		skb_push(skb, MT_TXD_SIZE + MT_TXP_SIZE);
		mt7668_mac_write_txwi_8023(dev, skb, wcid, pid, tid, &ctl);
	} else {
		mt7668_tx_ctl(skb, &ctl);
		pid = mt76_tx_status_skb_add(mdev, wcid, skb);
		skb_push(skb, MT_TXD_SIZE);
		mt7668_mac_write_txwi(dev, skb, wcid, qid, pid, &ctl);
	}

	/* pad to 4 bytes, then a zero dword ends the transfer */
	pad = round_up(skb->len, 4) + 4 - skb->len;
	err = mt76_skb_adjust_pad(skb, pad);

	if (err)
		idr_remove(&wcid->pktid, pid);

	return err;
}

void mt7668_tx_complete_skb(struct mt76_dev *mdev, struct mt76_queue_entry *e)
{
	u32 txd1;

	if (!e->skb)
		return;

	txd1 = get_unaligned_le32(e->skb->data + 4);
	if (FIELD_GET(MT_TXD1_HDR_FORMAT, txd1) == MT_HDR_FORMAT_802_3)
		skb_pull(e->skb, MT_TXD_SIZE + MT_TXP_SIZE);
	else
		skb_pull(e->skb, MT_TXD_SIZE);

	mt76_tx_complete_skb(mdev, e->wcid, e->skb);
}

void mt7668_mac_tx_done(struct mt7668_dev *dev, struct sk_buff *skb)
{
	struct mt7668_tx_done_event *ev;
	struct mt76_dev *mdev = &dev->mt76;
	struct mt76_wcid *wcid;
	struct sk_buff_head list;
	struct sk_buff *status_skb;

	if (skb->len < sizeof(*ev))
		return;

	ev = (struct mt7668_tx_done_event *)skb->data;
	if (ev->wlan_idx >= ARRAY_SIZE(mdev->wcid))
		return;

	rcu_read_lock();
	wcid = rcu_dereference(mdev->wcid[ev->wlan_idx]);
	if (!wcid)
		goto out;

	mt76_tx_status_lock(mdev, &list);
	status_skb = mt76_tx_status_skb_get(mdev, wcid, ev->pid, &list);
	if (status_skb) {
		struct ieee80211_tx_info *info = IEEE80211_SKB_CB(status_skb);

		ieee80211_tx_info_clear_status(info);
		if (!ev->status)
			info->flags |= IEEE80211_TX_STAT_ACK;
		info->status.rates[0].idx = -1;
		mt76_tx_status_skb_done(mdev, status_skb, &list);
	}
	mt76_tx_status_unlock(mdev, &list);
out:
	rcu_read_unlock();
}

bool mt7668_tx_status_data(struct mt76_dev *mdev, u8 *update)
{
	return false;
}
