// SPDX-License-Identifier: GPL-2.0
/*
 * RTC in the LG1314 (LJ9 / OLED55B9PLA) standby MICOM, on I2C at 0x29.
 *
 * The MICOM stays powered while the TV is in standby and keeps time across
 * SoC power-off. All fields are plain binary:
 *
 *   read:  write {0x80}, read 7 bytes {century, year, month, mday, hour, min, sec}
 *   set:   write {0x24, century, year, month, mday, hour, min, sec, wday}
 *          (wday 0 = Sunday)
 *
 * After the MICOM loses power the clock restarts from an all-zero date;
 * month/mday 0 means "never set".
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/rtc.h>

#define MICOM_CMD_RTC_READ	0x80
#define MICOM_CMD_RTC_SET	0x24
#define MICOM_RTC_READ_LEN	7
#define MICOM_RETRIES		3

struct lg1314_micom_rtc {
	struct i2c_client *client;
	struct rtc_device *rtc;
};

static int micom_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct lg1314_micom_rtc *mr = dev_get_drvdata(dev);
	u8 b[MICOM_RTC_READ_LEN];
	int ret, i;

	for (i = 0; i < MICOM_RETRIES; i++) {
		ret = i2c_smbus_read_i2c_block_data(mr->client, MICOM_CMD_RTC_READ,
						    sizeof(b), b);
		if (ret == sizeof(b))
			break;
	}
	if (ret < 0)
		return ret;
	if (ret != sizeof(b))
		return -EIO;

	/* Cleared after MICOM power loss: not set. */
	if (!b[2] || !b[3])
		return -EINVAL;

	tm->tm_year = b[0] * 100 + b[1] - 1900;
	tm->tm_mon = b[2] - 1;
	tm->tm_mday = b[3];
	tm->tm_hour = b[4];
	tm->tm_min = b[5];
	tm->tm_sec = b[6];
	tm->tm_wday = -1;	/* not reported; rtc_valid_tm() doesn't need it */

	return rtc_valid_tm(tm);
}

static int micom_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct lg1314_micom_rtc *mr = dev_get_drvdata(dev);
	int year = tm->tm_year + 1900;
	u8 b[8] = {
		year / 100, year % 100, tm->tm_mon + 1, tm->tm_mday,
		tm->tm_hour, tm->tm_min, tm->tm_sec, tm->tm_wday,
	};
	int ret, i;

	for (i = 0; i < MICOM_RETRIES; i++) {
		ret = i2c_smbus_write_i2c_block_data(mr->client, MICOM_CMD_RTC_SET,
						     sizeof(b), b);
		if (!ret)
			break;
	}
	return ret;
}

static const struct rtc_class_ops micom_rtc_ops = {
	.read_time	= micom_rtc_read_time,
	.set_time	= micom_rtc_set_time,
};

static int lg1314_micom_rtc_probe(struct i2c_client *client)
{
	struct lg1314_micom_rtc *mr;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_I2C_BLOCK))
		return -ENODEV;

	mr = devm_kzalloc(&client->dev, sizeof(*mr), GFP_KERNEL);
	if (!mr)
		return -ENOMEM;
	mr->client = client;
	i2c_set_clientdata(client, mr);

	mr->rtc = devm_rtc_allocate_device(&client->dev);
	if (IS_ERR(mr->rtc))
		return PTR_ERR(mr->rtc);

	mr->rtc->ops = &micom_rtc_ops;
	/*
	 * The year is a century byte plus a two-digit year; keep to the range
	 * the MICOM firmware is known to handle.
	 */
	mr->rtc->range_min = RTC_TIMESTAMP_BEGIN_2000;
	mr->rtc->range_max = RTC_TIMESTAMP_END_2099;

	return devm_rtc_register_device(mr->rtc);
}

static const struct of_device_id lg1314_micom_rtc_of_match[] = {
	{ .compatible = "lge,lg1314-micom" },
	{ }
};
MODULE_DEVICE_TABLE(of, lg1314_micom_rtc_of_match);

static const struct i2c_device_id lg1314_micom_rtc_id[] = {
	{ "lg1314-micom" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lg1314_micom_rtc_id);

static struct i2c_driver lg1314_micom_rtc_driver = {
	.driver = {
		.name		= "rtc-lg1314-micom",
		.of_match_table	= lg1314_micom_rtc_of_match,
	},
	.probe		= lg1314_micom_rtc_probe,
	.id_table	= lg1314_micom_rtc_id,
};
module_i2c_driver(lg1314_micom_rtc_driver);

MODULE_DESCRIPTION("LG1314 standby MICOM RTC");
MODULE_LICENSE("GPL");
