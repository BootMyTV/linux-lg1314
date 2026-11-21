// SPDX-License-Identifier: GPL-2.0
/*
 * RTC kept by the LG1314 standby MICOM (MFD child of lg1314-micom).
 *
 * The MICOM keeps time across SoC resets and standby. All fields are plain
 * binary:
 *
 *   read:  query 0x80, 7 bytes {century, year, month, mday, hour, min, sec}
 *   set:   command {0x24, century, year, month, mday, hour, min, sec, wday}
 *          (wday 0 = Sunday)
 *
 * After the MICOM loses power the clock restarts from an all-zero date;
 * month/mday 0 means "never set".
 */

#include <linux/mfd/lg1314-micom.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>

#define MICOM_CMD_RTC_READ	0x80
#define MICOM_CMD_RTC_SET	0x24
#define MICOM_RTC_READ_LEN	7

static int micom_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct lg1314_micom *micom = dev_get_drvdata(dev->parent);
	u8 b[MICOM_RTC_READ_LEN];
	int ret;

	ret = lg1314_micom_query(micom, MICOM_CMD_RTC_READ, b, sizeof(b));
	if (ret)
		return ret;

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
	struct lg1314_micom *micom = dev_get_drvdata(dev->parent);
	int year = tm->tm_year + 1900;
	const u8 b[] = {
		MICOM_CMD_RTC_SET,
		year / 100, year % 100, tm->tm_mon + 1, tm->tm_mday,
		tm->tm_hour, tm->tm_min, tm->tm_sec, tm->tm_wday,
	};

	return lg1314_micom_cmd(micom, b, sizeof(b));
}

static const struct rtc_class_ops micom_rtc_ops = {
	.read_time	= micom_rtc_read_time,
	.set_time	= micom_rtc_set_time,
};

static int lg1314_micom_rtc_probe(struct platform_device *pdev)
{
	struct rtc_device *rtc;

	rtc = devm_rtc_allocate_device(&pdev->dev);
	if (IS_ERR(rtc))
		return PTR_ERR(rtc);

	rtc->ops = &micom_rtc_ops;
	/*
	 * The year is a century byte plus a two-digit year; keep to the range
	 * the MICOM firmware is known to handle.
	 */
	rtc->range_min = RTC_TIMESTAMP_BEGIN_2000;
	rtc->range_max = RTC_TIMESTAMP_END_2099;

	return devm_rtc_register_device(rtc);
}

static struct platform_driver lg1314_micom_rtc_driver = {
	.driver = {
		.name	= "lg1314-micom-rtc",
	},
	.probe	= lg1314_micom_rtc_probe,
};
module_platform_driver(lg1314_micom_rtc_driver);

MODULE_DESCRIPTION("LG1314 standby MICOM RTC");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:lg1314-micom-rtc");
