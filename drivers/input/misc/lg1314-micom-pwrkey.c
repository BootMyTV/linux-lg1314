// SPDX-License-Identifier: GPL-2.0
/*
 * Power key of the IR remote, as seen by the LG1314 standby MICOM (MFD child
 * of lg1314-micom).
 *
 * The MICOM owns the IR receiver and queues decoded key events; the host polls
 * them with query 0xc0, which returns {type, code, ~code} and dequeues it
 * ({0xff, 0xff, 0xff} = nothing pending). Types: 0x02 remote press, 0x03
 * remote repeat, 0x0a remote release; codes are the command byte of LG's NEC
 * IR frames.
 *
 * Only the power key (code 0x08) is reported, as KEY_POWER: the remote also
 * sends keys over Bluetooth LE when paired, which reach the system through
 * their own driver. Every other event is drained and dropped. The power key
 * has been seen arriving as a lone release event, so either type counts, and
 * a press followed by its release is reported once.
 */

#include <linux/input.h>
#include <linux/mfd/lg1314-micom.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#define MICOM_QUERY_KEY		0xc0
#define MICOM_KEY_EVENT_LEN	3

#define MICOM_KEY_REMOTE_PRESS	0x02
#define MICOM_KEY_REMOTE_REPEAT	0x03
#define MICOM_KEY_REMOTE_RELEASE 0x0a
#define MICOM_KEY_NONE		0xff

#define LG_IR_CODE_POWER	0x08

#define POLL_INTERVAL_MS	40	/* as the vendor stack polls */
#define MAX_EVENTS_PER_POLL	8	/* drain bursts without hogging the bus */

struct micom_pwrkey {
	struct lg1314_micom *micom;
	bool pressed;	/* KEY_POWER reported for a press, awaiting its release */
};

static void micom_pwrkey_report(struct input_dev *input)
{
	input_report_key(input, KEY_POWER, 1);
	input_sync(input);
	input_report_key(input, KEY_POWER, 0);
	input_sync(input);
}

static void micom_pwrkey_poll(struct input_dev *input)
{
	struct micom_pwrkey *pk = input_get_drvdata(input);
	u8 ev[MICOM_KEY_EVENT_LEN];
	int i;

	for (i = 0; i < MAX_EVENTS_PER_POLL; i++) {
		if (lg1314_micom_query(pk->micom, MICOM_QUERY_KEY, ev, sizeof(ev)))
			return;
		if (ev[0] == MICOM_KEY_NONE)
			return;
		if ((ev[1] ^ ev[2]) != 0xff || ev[1] != LG_IR_CODE_POWER)
			continue;

		switch (ev[0]) {
		case MICOM_KEY_REMOTE_PRESS:
			if (!pk->pressed) {
				micom_pwrkey_report(input);
				pk->pressed = true;
			}
			break;
		case MICOM_KEY_REMOTE_RELEASE:
			if (!pk->pressed)
				micom_pwrkey_report(input);
			pk->pressed = false;
			break;
		default:	/* repeats while held: already reported */
			break;
		}
	}
}

static int lg1314_micom_pwrkey_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct micom_pwrkey *pk;
	struct input_dev *input;
	int ret;

	pk = devm_kzalloc(dev, sizeof(*pk), GFP_KERNEL);
	if (!pk)
		return -ENOMEM;
	pk->micom = dev_get_drvdata(dev->parent);

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	input->name = "LG1314 MICOM IR power key";
	input->phys = "lg1314-micom/input0";
	input->id.bustype = BUS_I2C;
	input_set_capability(input, EV_KEY, KEY_POWER);
	input_set_drvdata(input, pk);

	ret = input_setup_polling(input, micom_pwrkey_poll);
	if (ret)
		return ret;
	input_set_poll_interval(input, POLL_INTERVAL_MS);

	return input_register_device(input);
}

static struct platform_driver lg1314_micom_pwrkey_driver = {
	.driver = {
		.name	= "lg1314-micom-pwrkey",
	},
	.probe	= lg1314_micom_pwrkey_probe,
};
module_platform_driver(lg1314_micom_pwrkey_driver);

MODULE_DESCRIPTION("LG1314 standby MICOM IR power key");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:lg1314-micom-pwrkey");
