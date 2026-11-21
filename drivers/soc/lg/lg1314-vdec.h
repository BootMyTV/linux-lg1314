/* SPDX-License-Identifier: GPL-2.0 */
/*
 * LG1314 video-decode coprocessor: interface between the core driver
 * (firmware boot, IPC) and the parts layered on it.
 */
#ifndef LG1314_VDEC_H
#define LG1314_VDEC_H

#include <linux/bits.h>
#include <linux/err.h>
#include <linux/types.h>

struct device;
struct lg1314_vdec;
struct lg1314_vdec_pt;

/* Host->firmware calls (lg1314_vdec_call). */
enum {
	VDEC_CALL_VDEC_OPEN,	/* -> firmware decoder handle, 0 on failure */
	VDEC_CALL_VDEC_CLOSE,	/* priv = handle */
	VDEC_CALL_VDEC_IOCTL,	/* priv = handle, data = {cmd, arg} + ioctl struct */
	VDEC_CALL_VO_OPEN,
	VDEC_CALL_VO_CLOSE,
	VDEC_CALL_VO_IOCTL,
};
#define VDEC_CALL_WR		BIT(0)	/* copy data to the callee */
#define VDEC_CALL_RD		BIT(1)	/* copy data back from the callee */
#define VDEC_CALL_MAX_DATA	0x8000	/* largest call argument */

int lg1314_vdec_call(struct lg1314_vdec *v, u32 call, u32 priv,
		     void *data, u32 size, unsigned int dir, int *result);
struct device *lg1314_vdec_device(struct lg1314_vdec *v);
/* Physical address of a coprocessor address, 0 if it isn't mapped. */
phys_addr_t lg1314_vdec_copro_to_phys(struct lg1314_vdec *v, u32 addr);
/* Whether [phys, phys + size) lies in the carveout the firmware can reach. */
bool lg1314_vdec_in_carveout(struct lg1314_vdec *v, phys_addr_t phys, size_t size);
/* CPU-readable mapping of part of a decoded frame (dpb pool), or NULL. */
const void *lg1314_vdec_frame_map(struct lg1314_vdec *v, phys_addr_t phys,
				  size_t len);

/* V4L2 stateful decoder (lg1314-vdec-v4l2.c). */
struct lg1314_vdec_dec;
struct lg1314_vdec_dec *lg1314_vdec_dec_init(struct lg1314_vdec *v);
void lg1314_vdec_dec_exit(struct lg1314_vdec_dec *dec);
/* A decoder's readiness changed (POLLIN etc.). IRQ context. */
void lg1314_vdec_dec_poll(struct lg1314_vdec_dec *dec, u32 handle, u32 mask);

#if IS_ENABLED(CONFIG_LG1314_VDEC_PASSTHROUGH)
struct lg1314_vdec_pt *lg1314_vdec_pt_init(struct lg1314_vdec *v);
void lg1314_vdec_pt_exit(struct lg1314_vdec_pt *pt);
/* A decoder's readiness changed: mask is POLLIN/POLLOUT/POLLERR. IRQ context. */
void lg1314_vdec_pt_poll(struct lg1314_vdec_pt *pt, u32 handle, u32 mask);
#else
static inline struct lg1314_vdec_pt *lg1314_vdec_pt_init(struct lg1314_vdec *v)
{
	return NULL;
}
static inline void lg1314_vdec_pt_exit(struct lg1314_vdec_pt *pt) { }
static inline void lg1314_vdec_pt_poll(struct lg1314_vdec_pt *pt, u32 handle,
				       u32 mask) { }
#endif

#endif
