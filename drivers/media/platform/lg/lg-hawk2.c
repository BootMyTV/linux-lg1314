// SPDX-License-Identifier: GPL-2.0
/*
 * LG "HAWK2" HDMI 2.1 switch, between the HDMI connectors and the SoC's HDMI 2.0
 * receivers on LG's 2019 OLED TVs (B9 etc.). It owns the connectors' DDC, HPD and
 * EDID (EDIDs live in its flash and are picked by type/index) and forwards the
 * selected source to the SoC port with the same number.
 *
 * Host protocol (from tvservice's hdmi21sw_drv_* / hdmi21sw_ctl_*): 7-byte commands
 * {group | checksum << 4, opcode, p0..p4}, the checksum being the low nibble of the
 * sum of bytes 1..6; group 0xD = set, 0xE = get (a plain read of the answer follows).
 * The switch raises CMD_READY when it can take a command and again when the answer
 * is ready.
 *
 * Stock flow (hdmi21sw_ctl_polling_thread): wait for the boot magic, select the
 * EDID of every port, then poll the change flags. The switch drives HPD itself
 * after an EDID select. The receiver driver picks the source (lg_hawk2_select());
 * until it does, the source follows the port where 5V appeared last.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#include <media/lg-hawk2.h>

#define HAWK2_PORTS		4
#define HAWK2_SET		0xd
#define HAWK2_GET		0xe

/* gets */
#define HAWK2_GET_FW_VER	0x00	/* 4 ASCII characters */
#define HAWK2_GET_SW_STATUS	0x01	/* 4 bytes, HAWK2_BOOT_MAGIC once booted */
#define HAWK2_GET_CHANGED	0x07	/* 4 bytes, clear on read */
#define HAWK2_GET_5V		0x06	/* port; 1 byte */
/* sets */
#define HAWK2_SET_SOURCE	0x05	/* previous port, new port */
#define HAWK2_SET_EDID		0x12	/* port, type, index, user port */

#define HAWK2_BOOT_MAGIC	0x80336647
#define HAWK2_EDID_UHD		1	/* tvservice HDMI21SW_EDID_TYPE: 1 UHD, 2 FHD, 3 8K */
#define HAWK2_EDID_INDEX	0	/* UHD list entry 0: video 2, audio 0, ALLM off */

/* HAWK2_GET_CHANGED (hdmi21sw_drv_get_status_changed) */
#define CHG_BOOT_DONE		BIT(2)
#define CHG_CHECKSUM_ERR	BIT(3)
#define CHG_5V(p)		BIT(7 + 5 * (p))

#define HAWK2_POLL_MS		200
#define HAWK2_BOOT_TRIES	100	/* stock: 100 polls, then reboot the switch */
#define HAWK2_CMD_READY_US	1000000

struct lg_hawk2 {
	struct i2c_client *client;
	struct gpio_desc *reset;
	struct gpio_desc *cmd_ready;
	struct mutex lock;		/* command sequence */
	struct delayed_work work;
	u32 user_port[HAWK2_PORTS];	/* connector number (HDMI n) of each port */
	bool booted;
	unsigned int boot_polls;
	bool pwr5v[HAWK2_PORTS];
	int source;			/* -1: none selected yet */
	int prev_source;		/* stock sends the previous port along */
	int wanted;			/* lg_hawk2_select(); -1: follow 5V */
};

static int hawk2_wait_ready(struct lg_hawk2 *hw)
{
	int val;

	return read_poll_timeout(gpiod_get_value_cansleep, val, val > 0, 1000,
				 HAWK2_CMD_READY_US, false, hw->cmd_ready);
}

static int hawk2_cmd(struct lg_hawk2 *hw, u8 grp, u8 op, const u8 *par, unsigned int npar,
		     u8 *ans, unsigned int nans)
{
	u8 buf[7] = { 0, op };
	unsigned int i, sum = 0;
	int ret;

	memcpy(&buf[2], par, min(npar, 5u));
	for (i = 1; i < 7; i++)
		sum += buf[i];
	buf[0] = (grp & 0xf) | (sum & 0xf) << 4;

	mutex_lock(&hw->lock);
	ret = hawk2_wait_ready(hw);
	if (!ret)
		ret = i2c_master_send(hw->client, buf, sizeof(buf));
	if (ret >= 0 && nans) {
		ret = hawk2_wait_ready(hw);
		if (!ret)
			ret = i2c_master_recv(hw->client, ans, nans);
	}
	mutex_unlock(&hw->lock);
	return ret < 0 ? ret : 0;
}

static int hawk2_get(struct lg_hawk2 *hw, u8 op, int port, u8 *ans, unsigned int nans)
{
	u8 p = port;

	return hawk2_cmd(hw, HAWK2_GET, op, &p, port >= 0, ans, nans);
}

static int hawk2_set(struct lg_hawk2 *hw, u8 op, const u8 *par, unsigned int npar)
{
	return hawk2_cmd(hw, HAWK2_SET, op, par, npar, NULL, 0);
}

static int hawk2_get_u32(struct lg_hawk2 *hw, u8 op, u32 *val)
{
	u8 b[4];
	int ret = hawk2_get(hw, op, -1, b, sizeof(b));

	if (!ret)
		*val = get_unaligned_le32(b);
	return ret;
}

/* hdmi21sw_drv_gpio_reboot */
static void hawk2_reboot(struct lg_hawk2 *hw)
{
	dev_warn(&hw->client->dev, "rebooting the switch\n");
	gpiod_set_value_cansleep(hw->reset, 1);
	msleep(10);
	gpiod_set_value_cansleep(hw->reset, 0);
	hw->booted = false;
	hw->boot_polls = 0;
}

static void hawk2_select_source(struct lg_hawk2 *hw, int port)
{
	u8 par[2] = { hw->prev_source, port };
	int ret = hawk2_set(hw, HAWK2_SET_SOURCE, par, sizeof(par));

	if (ret) {
		dev_err(&hw->client->dev, "source %d: %d\n", port, ret);
		return;
	}
	dev_info(&hw->client->dev, "source: port %d (HDMI %u)\n", port, hw->user_port[port]);
	hw->prev_source = port;
	hw->source = port;
}

static void hawk2_update_5v(struct lg_hawk2 *hw, int port)
{
	u8 v;
	bool on;

	if (hawk2_get(hw, HAWK2_GET_5V, port, &v, 1))
		return;
	on = v & 1;
	if (on == hw->pwr5v[port])
		return;
	hw->pwr5v[port] = on;
	dev_info(&hw->client->dev, "port %d (HDMI %u): 5V %s\n", port, hw->user_port[port],
		 on ? "on" : "off");
	if (on && port != hw->source && hw->wanted < 0)
		hawk2_select_source(hw, port);
}

/* hdmi21sw_ctl_polling_thread, BOOT_READY: EDID per port, then the active source */
static void hawk2_setup(struct lg_hawk2 *hw)
{
	struct device *dev = &hw->client->dev;
	u8 fw[4] = { '?', '?', '?', '?' };
	int p, ret;

	hawk2_get(hw, HAWK2_GET_FW_VER, -1, fw, sizeof(fw));
	dev_info(dev, "switch ready, firmware %.4s\n", fw);
	for (p = 0; p < HAWK2_PORTS; p++) {
		u8 par[4] = { p, HAWK2_EDID_UHD, HAWK2_EDID_INDEX, hw->user_port[p] };

		ret = hawk2_set(hw, HAWK2_SET_EDID, par, sizeof(par));
		if (ret)
			dev_err(dev, "port %d: EDID select: %d\n", p, ret);
	}
	if (hw->wanted >= 0)
		hw->source = hw->wanted;
	if (hw->source >= 0)
		hawk2_select_source(hw, hw->source);
	for (p = 0; p < HAWK2_PORTS; p++) {
		hw->pwr5v[p] = false;
		hawk2_update_5v(hw, p);
	}
}

static void hawk2_apply_wanted(struct lg_hawk2 *hw);

static void hawk2_work(struct work_struct *work)
{
	struct lg_hawk2 *hw = container_of(work, struct lg_hawk2, work.work);
	u32 val;
	int p;

	if (!hw->booted) {
		if (!hawk2_get_u32(hw, HAWK2_GET_SW_STATUS, &val) && val == HAWK2_BOOT_MAGIC) {
			hw->booted = true;
			hawk2_get_u32(hw, HAWK2_GET_CHANGED, &val);	/* clear */
			hawk2_setup(hw);
		} else if (++hw->boot_polls >= HAWK2_BOOT_TRIES) {
			hawk2_reboot(hw);
		}
		goto out;
	}

	hawk2_apply_wanted(hw);
	if (hawk2_get_u32(hw, HAWK2_GET_CHANGED, &val))
		goto out;
	if (val & CHG_CHECKSUM_ERR) {
		hawk2_reboot(hw);
		goto out;
	}
	if (val & CHG_BOOT_DONE) {
		/* the switch restarted on its own: its EDID selection is gone */
		hw->booted = false;
		hw->boot_polls = 0;
		goto out;
	}
	for (p = 0; p < HAWK2_PORTS; p++)
		if (val & CHG_5V(p))
			hawk2_update_5v(hw, p);
out:
	schedule_delayed_work(&hw->work, msecs_to_jiffies(HAWK2_POLL_MS));
}

struct lg_hawk2 *lg_hawk2_get(struct device_node *np)
{
	struct i2c_client *client = of_find_i2c_device_by_node(np);
	struct lg_hawk2 *hw;

	if (!client)
		return ERR_PTR(-EPROBE_DEFER);
	hw = i2c_get_clientdata(client);
	if (!hw || !try_module_get(client->dev.driver->owner)) {
		put_device(&client->dev);
		return ERR_PTR(-EPROBE_DEFER);
	}
	return hw;
}
EXPORT_SYMBOL_GPL(lg_hawk2_get);

void lg_hawk2_put(struct lg_hawk2 *hw)
{
	module_put(hw->client->dev.driver->owner);
	put_device(&hw->client->dev);
}
EXPORT_SYMBOL_GPL(lg_hawk2_put);

int lg_hawk2_connector_port(struct lg_hawk2 *hw, unsigned int connector)
{
	int p;

	for (p = 0; p < HAWK2_PORTS; p++)
		if (hw->user_port[p] == connector)
			return p;
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(lg_hawk2_connector_port);

/* Runs from the poll work, which owns the switch state. */
static void hawk2_apply_wanted(struct lg_hawk2 *hw)
{
	int want = READ_ONCE(hw->wanted);

	if (want >= 0 && hw->booted && want != hw->source)
		hawk2_select_source(hw, want);
}

void lg_hawk2_select(struct lg_hawk2 *hw, int port)
{
	if (port >= HAWK2_PORTS)
		port = -1;
	WRITE_ONCE(hw->wanted, port);
	mod_delayed_work(system_wq, &hw->work, 0);
}
EXPORT_SYMBOL_GPL(lg_hawk2_select);

static int hawk2_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct gpio_desc *pm;
	struct lg_hawk2 *hw;
	int ret;

	hw = devm_kzalloc(dev, sizeof(*hw), GFP_KERNEL);
	if (!hw)
		return -ENOMEM;
	hw->client = client;
	hw->source = -1;
	hw->wanted = -1;
	mutex_init(&hw->lock);
	i2c_set_clientdata(client, hw);

	ret = of_property_read_u32_array(dev->of_node, "lge,user-ports", hw->user_port,
					 HAWK2_PORTS);
	if (ret)
		return dev_err_probe(dev, ret, "lge,user-ports\n");

	/* stock hdmi21sw_drv_gpio_init: reset released, power-management (standby) off */
	hw->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(hw->reset))
		return dev_err_probe(dev, PTR_ERR(hw->reset), "reset gpio\n");
	pm = devm_gpiod_get_optional(dev, "standby", GPIOD_OUT_LOW);
	if (IS_ERR(pm))
		return dev_err_probe(dev, PTR_ERR(pm), "standby gpio\n");
	hw->cmd_ready = devm_gpiod_get(dev, "cmd-ready", GPIOD_IN);
	if (IS_ERR(hw->cmd_ready))
		return dev_err_probe(dev, PTR_ERR(hw->cmd_ready), "cmd-ready gpio\n");

	INIT_DELAYED_WORK(&hw->work, hawk2_work);
	schedule_delayed_work(&hw->work, 0);
	return 0;
}

static void hawk2_remove(struct i2c_client *client)
{
	struct lg_hawk2 *hw = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&hw->work);
}

static const struct of_device_id hawk2_of_match[] = {
	{ .compatible = "lge,hawk2" },
	{ }
};
MODULE_DEVICE_TABLE(of, hawk2_of_match);

static struct i2c_driver hawk2_driver = {
	.probe = hawk2_probe,
	.remove = hawk2_remove,
	.driver = {
		.name = "lg-hawk2",
		.of_match_table = hawk2_of_match,
	},
};
module_i2c_driver(hawk2_driver);

MODULE_DESCRIPTION("LG HAWK2 HDMI 2.1 switch");
MODULE_LICENSE("GPL");
