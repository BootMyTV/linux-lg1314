// SPDX-License-Identifier: GPL-2.0
/*
 * Standby MICOM of LG1314 (LJ9 / OLED55B9PLA) TVs, on I2C at 0x29.
 *
 * The MICOM stays powered while the TV is in standby. Besides hosting the
 * child functions (RTC, IR power key), it switches power and reset of the on-board MT7668
 * Wi-Fi/Bluetooth combo, which sits on the SoC's internal USB port:
 *
 *   {0xd2, on}   combo power
 *   {0x94, on}   combo reset (0 = asserted)
 *
 * An SoC-only reset leaves the combo powered and running whatever firmware the
 * previous kernel loaded, after which it no longer enumerates. So the combo is
 * powered down on shutdown/reboot and power-cycled at probe (the latter also
 * covering resets that skip shutdown, e.g. a watchdog after a hang).
 *
 * Commands are plain I2C writes (no register address); queries write an opcode
 * and read the reply. Every transfer is retried, as the vendor stack does.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/mfd/core.h>
#include <linux/mfd/lg1314-micom.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/workqueue.h>

#define MICOM_RETRIES		3

#define MICOM_CMD_COMBO_POWER	0xd2
#define MICOM_CMD_COMBO_RESET	0x94

/* Combo off time for the boot-time power cycle, and power-to-reset delay. */
#define COMBO_OFF_MS		500
#define COMBO_POWER_TO_RESET_MS	10

struct lg1314_micom_priv {
	struct lg1314_micom micom;	/* first: it is the i2c clientdata */
	struct work_struct combo_work;
};

int lg1314_micom_cmd(struct lg1314_micom *micom, const u8 *buf, size_t len)
{
	int ret = -EIO, i;

	for (i = 0; i < MICOM_RETRIES; i++) {
		ret = i2c_master_send(micom->client, buf, len);
		if (ret == len)
			return 0;
	}
	return ret < 0 ? ret : -EIO;
}
EXPORT_SYMBOL_GPL(lg1314_micom_cmd);

int lg1314_micom_query(struct lg1314_micom *micom, u8 opcode, u8 *buf,
		       size_t len)
{
	struct i2c_msg msgs[] = {
		{ .addr = micom->client->addr, .len = 1, .buf = &opcode },
		{ .addr = micom->client->addr, .flags = I2C_M_RD,
		  .len = len, .buf = buf },
	};
	int ret = -EIO, i;

	for (i = 0; i < MICOM_RETRIES; i++) {
		ret = i2c_transfer(micom->client->adapter, msgs,
				   ARRAY_SIZE(msgs));
		if (ret == ARRAY_SIZE(msgs))
			return 0;
	}
	return ret < 0 ? ret : -EIO;
}
EXPORT_SYMBOL_GPL(lg1314_micom_query);

static int micom_combo_set(struct lg1314_micom *micom, bool on)
{
	const u8 power[] = { MICOM_CMD_COMBO_POWER, on };
	const u8 reset[] = { MICOM_CMD_COMBO_RESET, on };
	int ret;

	ret = lg1314_micom_cmd(micom, power, sizeof(power));
	if (ret)
		return ret;
	if (on)
		msleep(COMBO_POWER_TO_RESET_MS);
	return lg1314_micom_cmd(micom, reset, sizeof(reset));
}

/* Boot-time combo power cycle, off the probe path (it sleeps ~0.5 s). */
static void micom_combo_cycle_work(struct work_struct *work)
{
	struct lg1314_micom_priv *priv =
		container_of(work, struct lg1314_micom_priv, combo_work);
	struct lg1314_micom *micom = &priv->micom;
	int ret;

	/* Cold-start the combo whatever state the last session left it in. */
	ret = micom_combo_set(micom, false);
	if (!ret) {
		msleep(COMBO_OFF_MS);
		ret = micom_combo_set(micom, true);
	}
	if (ret)
		dev_warn(micom->dev, "Wi-Fi/BT combo power cycle failed: %d\n",
			 ret);
}

static const struct mfd_cell lg1314_micom_cells[] = {
	{ .name = "lg1314-micom-rtc" },
	{ .name = "lg1314-micom-pwrkey" },
	{ .name = "lg1314-micom-gpio" },
};

static int lg1314_micom_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct lg1314_micom_priv *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->micom.dev = dev;
	priv->micom.client = client;
	INIT_WORK(&priv->combo_work, micom_combo_cycle_work);
	i2c_set_clientdata(client, &priv->micom);

	/*
	 * Children (the RTC) first and synchronously, so the RTC exists by the
	 * time the kernel sets the system clock from it at late_initcall.
	 */
	ret = devm_mfd_add_devices(dev, PLATFORM_DEVID_AUTO,
				   lg1314_micom_cells,
				   ARRAY_SIZE(lg1314_micom_cells),
				   NULL, 0, NULL);
	if (ret)
		return ret;

	schedule_work(&priv->combo_work);
	return 0;
}

static void lg1314_micom_remove(struct i2c_client *client)
{
	struct lg1314_micom_priv *priv = i2c_get_clientdata(client);

	cancel_work_sync(&priv->combo_work);
}

static void lg1314_micom_shutdown(struct i2c_client *client)
{
	struct lg1314_micom_priv *priv = i2c_get_clientdata(client);
	int ret;

	cancel_work_sync(&priv->combo_work);
	ret = micom_combo_set(&priv->micom, false);
	if (ret)
		dev_warn(&client->dev, "Wi-Fi/BT combo power-off failed: %d\n",
			 ret);
}

static const struct of_device_id lg1314_micom_of_match[] = {
	{ .compatible = "lge,lg1314-micom" },
	{ }
};
MODULE_DEVICE_TABLE(of, lg1314_micom_of_match);

static struct i2c_driver lg1314_micom_driver = {
	.driver = {
		.name		= "lg1314-micom",
		.of_match_table	= lg1314_micom_of_match,
	},
	.probe		= lg1314_micom_probe,
	.remove		= lg1314_micom_remove,
	.shutdown	= lg1314_micom_shutdown,
};
module_i2c_driver(lg1314_micom_driver);

MODULE_DESCRIPTION("LG1314 standby MICOM");
MODULE_LICENSE("GPL");
