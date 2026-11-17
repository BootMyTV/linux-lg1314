// SPDX-License-Identifier: GPL-2.0
/*
 * B9 OLED panel-rail power sequencing over the standby MICOM (I2C 0x29):
 *   PanelVcc/12V  on {0x3a,0x01}/off {0x3a,0x00}, status reg 0xd1
 *   EVDD/24V      on {0x36}/off {0x37},           status reg 0xd0
 * On: Vcc -> t2 -> (OVI channel power) -> t3 -> EVDD. Off mirrors with t5/t6
 * (OLED discharge guards).
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include "lg1314_drm.h"

/* B9 tool-type warm-up delays (_gOledWarmDelay), milliseconds. */
#define LG1314_T2_VDD_TO_DATA		65
#define LG1314_T3_DATA_TO_EVDD		650
#define LG1314_T5_EVDD_TO_DATA		50
#define LG1314_T6_VDD_TO_VDD		1800

#define MICOM_PANEL_VCC_REG		0x3a
#define MICOM_PANEL_VCC_STATUS		0xd1
#define MICOM_EVDD_ON			0x36
#define MICOM_EVDD_OFF			0x37
#define MICOM_EVDD_STATUS		0xd0

static int micom_verify(struct lg1314_drm *lg, u8 reg, u8 want)
{
	int i, v;

	for (i = 0; i < 20; i++) {
		v = i2c_smbus_read_byte_data(lg->micom, reg);
		if (v >= 0 && (v & 0x1) == want)
			return 0;
		usleep_range(10000, 11000);
	}
	dev_err(lg->dev, "MICOM reg %#x != %u (last %d)\n", reg, want, v);
	return -EIO;
}

int lg1314_panel_vcc(struct lg1314_drm *lg, bool on)
{
	int ret;

	ret = i2c_smbus_write_byte_data(lg->micom, MICOM_PANEL_VCC_REG,
					on ? 0x01 : 0x00);
	if (ret)
		return ret;
	ret = micom_verify(lg, MICOM_PANEL_VCC_STATUS, on ? 1 : 0);
	if (ret)
		return ret;

	if (on)
		msleep(LG1314_T2_VDD_TO_DATA);
	else
		msleep(LG1314_T6_VDD_TO_VDD);
	return 0;
}

int lg1314_panel_evdd(struct lg1314_drm *lg, bool on)
{
	int ret;

	if (on)
		msleep(LG1314_T3_DATA_TO_EVDD);

	ret = i2c_smbus_write_byte(lg->micom, on ? MICOM_EVDD_ON : MICOM_EVDD_OFF);
	if (ret)
		return ret;
	ret = micom_verify(lg, MICOM_EVDD_STATUS, on ? 1 : 0);
	if (ret)
		return ret;

	if (!on)
		msleep(LG1314_T5_EVDD_TO_DATA);
	return 0;
}
