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

/*
 * The number of SoC receiver ports the switch spreads the picture it receives on
 * @port over (1, 2 or 4: left to right over the ports of connectors HDMI 1, 2, ...),
 * for signals beyond one HDMI 2.0 receiver (4K100/120 over FRL).
 */
int lg_hawk2_source(struct lg_hawk2 *hw);
unsigned int lg_hawk2_split(struct lg_hawk2 *hw, int port);
/*
 * The active width of the picture the switch receives on @port (0: none). Its split
 * report reads 0 for some FRL modes that are split all the same (2560x1440@120 over
 * four ports), so the receiver driver compares this with what its ports measure.
 */
unsigned int lg_hawk2_width(struct lg_hawk2 *hw, int port);

#endif
