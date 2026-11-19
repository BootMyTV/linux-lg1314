// SPDX-License-Identifier: ISC

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/usb.h>

#include "mt7668.h"
#include "mcu.h"

static const struct usb_device_id mt7668u_device_table[] = {
	{ USB_DEVICE_AND_INTERFACE_INFO(0x043e, 0x3109, 0xff, 0xff, 0xff) },
	{ },
};

static u32 mt7668u_rr(struct mt76_dev *dev, u32 addr)
{
	u32 ret;

	mutex_lock(&dev->usb.usb_ctrl_mtx);
	ret = ___mt76u_rr(dev, MT_VEND_READ_EXT,
			  USB_DIR_IN | USB_TYPE_VENDOR, addr);
	mutex_unlock(&dev->usb.usb_ctrl_mtx);

	return ret;
}

static void mt7668u_wr(struct mt76_dev *dev, u32 addr, u32 val)
{
	mutex_lock(&dev->usb.usb_ctrl_mtx);
	___mt76u_wr(dev, MT_VEND_WRITE_EXT,
		    USB_DIR_OUT | USB_TYPE_VENDOR, addr, val);
	mutex_unlock(&dev->usb.usb_ctrl_mtx);
}

static u32 mt7668u_rmw(struct mt76_dev *dev, u32 addr, u32 mask, u32 val)
{
	mutex_lock(&dev->usb.usb_ctrl_mtx);
	val |= ___mt76u_rr(dev, MT_VEND_READ_EXT,
			   USB_DIR_IN | USB_TYPE_VENDOR, addr) & ~mask;
	___mt76u_wr(dev, MT_VEND_WRITE_EXT,
		    USB_DIR_OUT | USB_TYPE_VENDOR, addr, val);
	mutex_unlock(&dev->usb.usb_ctrl_mtx);

	return val;
}

static void mt7668u_copy(struct mt76_dev *dev, u32 offset,
			 const void *data, int len)
{
	struct mt76_usb *usb = &dev->usb;
	int ret, i = 0, batch_len;
	const u8 *val = data;

	len = round_up(len, 4);

	mutex_lock(&usb->usb_ctrl_mtx);
	while (i < len) {
		batch_len = min_t(int, usb->data_len, len - i);
		memcpy(usb->data, val + i, batch_len);
		ret = __mt76u_vendor_request(dev, MT_VEND_WRITE_EXT,
					     USB_DIR_OUT | USB_TYPE_VENDOR,
					     (offset + i) >> 16, offset + i,
					     usb->data, batch_len);
		if (ret < 0)
			break;

		i += batch_len;
	}
	mutex_unlock(&usb->usb_ctrl_mtx);
}

static int mt7668u_power_on(struct mt7668_dev *dev)
{
	int ret;

	if (mt76_rr(dev, MT_SW_SYNC0) & MT_SW_SYNC0_PWR_ON)
		return 0;

	ret = mt76u_vendor_request(&dev->mt76, MT_VEND_POWER_ON,
				   USB_DIR_OUT | USB_TYPE_VENDOR,
				   0x0, 0x1, NULL, 0);
	if (ret)
		return ret;

	if (!mt76_poll_msec(dev, MT_SW_SYNC0, MT_SW_SYNC0_PWR_ON,
			    MT_SW_SYNC0_PWR_ON, 500)) {
		dev_err(dev->mt76.dev, "Timeout for power on\n");
		return -EIO;
	}

	return 0;
}

static void mt7668u_dma_init(struct mt7668_dev *dev)
{
	/* one packet per RX transfer: mt76's USB RX path expects that */
	mt76_clear(dev, MT_UDMA_WLCFG_0, MT_WL_RX_AGG_EN);
	mt76_set(dev, MT_UDMA_WLCFG_0,
		 MT_WL_TX_EN | MT_WL_RX_EN | MT_WL_RX_MPSZ_PAD0);
}

static int mt7668u_probe(struct usb_interface *usb_intf,
			 const struct usb_device_id *id)
{
	static struct mt76_bus_ops bus_ops = {
		.rr = mt7668u_rr,
		.wr = mt7668u_wr,
		.rmw = mt7668u_rmw,
		.read_copy = mt76u_read_copy,
		.write_copy = mt7668u_copy,
		.type = MT76_BUS_USB,
	};
	struct usb_device *udev = interface_to_usbdev(usb_intf);
	struct mt7668_dev *dev;
	struct mt76_dev *mdev;
	int ret;

	mdev = mt76_alloc_device(&usb_intf->dev, sizeof(*dev), &mt7668_ops,
				 &mt7668_drv_ops);
	if (!mdev)
		return -ENOMEM;

	dev = container_of(mdev, struct mt7668_dev, mt76);
	init_completion(&dev->ch_grant);
	udev = usb_get_dev(udev);
	usb_set_intfdata(usb_intf, dev);

	ret = __mt76u_init(mdev, usb_intf, &bus_ops);
	if (ret < 0)
		goto error;

	mdev->rev = (mt76_rr(dev, MT_HW_CHIPID) << 16) |
		    (mt76_rr(dev, MT_HW_REV) & 0xff);
	dev_dbg(mdev->dev, "ASIC revision: %04x\n", mdev->rev);
	if (mt76_chip(mdev) != 0x7668) {
		ret = -ENODEV;
		goto error;
	}

	ret = mt7668u_power_on(dev);
	if (ret)
		goto error;

	ret = mt76u_alloc_mcu_queue(mdev);
	if (ret)
		goto error;

	ret = mt76u_alloc_queues(mdev);
	if (ret)
		goto error;

	mt7668u_dma_init(dev);
	set_bit(MT76_STATE_INITIALIZED, &dev->mphy.state);

	/* WTBL entries the firmware keeps for itself */
	for (ret = 0; ret < MT7668_WTBL_RESERVED; ret++)
		mt76_wcid_mask_set(mdev->wcid_mask, ret);

	ret = mt7668_mcu_init(dev);
	if (ret)
		goto error;

	ret = mt7668_register_device(dev);
	if (ret)
		goto error;

	return 0;

error:
	mt76u_queues_deinit(mdev);
	usb_set_intfdata(usb_intf, NULL);
	usb_put_dev(udev);
	mt76_free_device(mdev);

	return ret;
}

static void mt7668u_disconnect(struct usb_interface *usb_intf)
{
	struct mt7668_dev *dev = usb_get_intfdata(usb_intf);

	if (!dev)
		return;

	mt76_unregister_device(&dev->mt76);
	clear_bit(MT76_STATE_INITIALIZED, &dev->mphy.state);
	mt76u_queues_deinit(&dev->mt76);

	usb_set_intfdata(usb_intf, NULL);
	usb_put_dev(interface_to_usbdev(usb_intf));

	mt76_free_device(&dev->mt76);
}

MODULE_DEVICE_TABLE(usb, mt7668u_device_table);
MODULE_FIRMWARE(MT7668_ROM_PATCH);
MODULE_FIRMWARE(MT7668_FIRMWARE_N9);
MODULE_FIRMWARE(MT7668_FIRMWARE_CR4);

static struct usb_driver mt7668u_driver = {
	.name		= KBUILD_MODNAME,
	.id_table	= mt7668u_device_table,
	.probe		= mt7668u_probe,
	.disconnect	= mt7668u_disconnect,
	.soft_unbind	= 1,
	.disable_hub_initiated_lpm = 1,
};
module_usb_driver(mt7668u_driver);

MODULE_DESCRIPTION("MediaTek MT7668U (USB) wireless driver");
MODULE_LICENSE("Dual BSD/GPL");
