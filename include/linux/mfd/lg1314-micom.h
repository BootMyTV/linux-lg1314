/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Standby MICOM of LG1314-based TVs: shared access for the MFD child drivers.
 */
#ifndef __LINUX_MFD_LG1314_MICOM_H
#define __LINUX_MFD_LG1314_MICOM_H

#include <linux/types.h>

struct device;
struct i2c_client;

struct lg1314_micom {
	struct device *dev;
	struct i2c_client *client;
};

/* Send a command: buf[0] is the opcode, the rest its arguments. */
int lg1314_micom_cmd(struct lg1314_micom *micom, const u8 *buf, size_t len);

/* Send a one-byte query opcode, then read len bytes of reply. */
int lg1314_micom_query(struct lg1314_micom *micom, u8 opcode, u8 *buf,
		       size_t len);

#endif /* __LINUX_MFD_LG1314_MICOM_H */
