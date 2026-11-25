/* SPDX-License-Identifier: GPL-2.0 */
/*
 * LG "HAWK2" HDMI 2.1 switch: interface for the SoC's HDMI receiver driver.
 */
#ifndef _MEDIA_LG_HAWK2_H
#define _MEDIA_LG_HAWK2_H

struct device_node;
struct lg_hawk2;

/* The switch behind @np; ERR_PTR(-EPROBE_DEFER) until it has probed. */
struct lg_hawk2 *lg_hawk2_get(struct device_node *np);
void lg_hawk2_put(struct lg_hawk2 *hw);

/* The switch port (= SoC receiver port) of connector HDMI @connector (1-based). */
int lg_hawk2_connector_port(struct lg_hawk2 *hw, unsigned int connector);

/* Route @port to the SoC; -1: follow the port where 5V appeared last. */
void lg_hawk2_select(struct lg_hawk2 *hw, int port);

#endif
