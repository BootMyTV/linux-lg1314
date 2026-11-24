// SPDX-License-Identifier: GPL-2.0
/*
 * LG1314 video-decode coprocessor (Xtensa MCU) bring-up.
 *
 * The LG1314 performs video decode on an on-chip Xtensa coprocessor that runs a
 * firmware image loaded at runtime from a reserved DRAM carveout. The host
 * programs the MCU control block to point the processor at the firmware and
 * releases it from reset, then communicates over a shared-memory mailbox with a
 * doorbell interrupt.
 *
 * This driver loads the firmware, boots the coprocessor, and exposes the
 * firmware's debug console as a virtual serial port (ttyLXIPC0): bytes are
 * carried through ring buffers in the shared carveout, with head/tail indices
 * in the mailbox and the doorbell signalling data in either direction.
 *
 * On the same mailbox sits a message layer (rings of small fixed-size
 * messages) and, on top of it, a call layer: either side can invoke a numbered
 * function on the other, passing a parameter block in the shared argument
 * area, and gets an integer result back. The decode interfaces are calls.
 */

#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/genalloc.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/serial_core.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/tty_flip.h>
#include <linux/workqueue.h>

#include <linux/soc/lg/lg1314-vdec-vo.h>

#include "lg1314-vdec.h"

#define VDEC_FW_NAME		"lg1314/vdec.bin"

static int vdec_debug;
module_param_named(debug, vdec_debug, int, 0644);
MODULE_PARM_DESC(debug, "dump the IPC mailbox on probe, log firmware buffer allocations");

/* Single coprocessor instance, for the sysfs test triggers (see below). */
struct lg1314_vdec;
static struct lg1314_vdec *vdec_instance;

/* MCU control block ("RC"): processor control, address windows, params, doorbell. */
#define VDEC_MCU_RC_PHYS	0xc9b00700
#define VDEC_MCU_RC_SIZE	0x200	/* param regs live at +0x100.. */
#define RC_PROC_CTRL		0x00	/* bit0 halt, bit2 debug-en, bit28 reset */
#define RC_SROM_OFFSET		0x10	/* firmware (code) window base, phys */
#define RC_SRAM_OFFSET_0	0x14	/* data window bases, phys */
#define RC_SRAM_OFFSET_1	0x18
#define RC_SRAM_OFFSET_2	0x1c
/* Host-side doorbell block (the firmware/Xtensa side uses 0x40 instead). */
#define RC_INTR_EN		0x30
#define RC_INTR_STATUS		0x34
#define RC_INTR_CLEAR		0x38
#define RC_TO_INTERNAL		0x3c	/* write to signal the coprocessor */
#define RC_PARAM_100		0x100	/* cleared */
#define RC_BOOTPARAM_PTR	0x104	/* copro address of the Xtensa bootparams */
#define RC_PARAM_108		0x108	/* param_base in copro RAM (0x6000_0000) space */
#define RC_PARAM_180		0x180	/* cleared */
#define PROC_CTRL_HALT		BIT(0)
#define PROC_CTRL_DEBUG		BIT(2)
#define PROC_CTRL_RESET		BIT(28)
#define INTR_BIT		0x400

/*
 * Coprocessor address map (fixed in the firmware; the host picks the physical
 * targets via the window registers): code at copro 0x5000_0000 maps through
 * SROM_OFFSET (256 MiB); data at copro 0x6000_0000, 0x8000_0000 and
 * 0xa000_0000 (512 MiB each) through SRAM_OFFSET_0/1/2. The data windows are
 * set as on the stock platform, copro = phys + 0x2000_0000 for phys
 * 0x4000_0000..0x7fff_ffff (and the third window identity-mapped); the
 * firmware relies on those offsets, so its memory must lie in that range.
 */
#define VDEC_COPRO_ROM		0x50000000
#define VDEC_DATA0_PHYS		0x40000000UL
#define VDEC_DATA1_PHYS		0x60000000UL
#define VDEC_DATA2_PHYS		0xa0000000UL
#define VDEC_DATA0_COPRO	0x60000000
#define VDEC_DATA_END		0x80000000UL	/* end of data windows 0+1 */
#define VDEC_WINDOW_SIZE	0x20000000
/* Reserved ROM region size (image + bss + embedded rootfs); M19 = 18 MiB + 512 KiB. */
#define VDEC_ROM_SIZE		(18 * 1024 * 1024 + 512 * 1024)
/*
 * Memory (stock layout). The "mcu" region holds the stream-buffer pool
 * ("cpb"), then the coprocessor's own pool ("vdec"): the IPC area (queue +
 * argument area), the firmware image (ROM region, the firmware runs inside
 * it), then general allocations. The "dpb" region is the decoded-picture
 * pool. The firmware asks us for buffers from these pools by name.
 */
#define VDEC_CPB_POOL_SIZE	0x01800000
#define VDEC_HMA_ORDER		12	/* allocation granule: 4 KiB */
/* "shared" regions: no-map memory the dpb pool overflows into */
#define VDEC_SHARED_MAX		4
#define VDEC_BOOTPARAM_MAX	512
#define VDEC_CMDLINE \
	"console=ttyAMA0,115200n81 console=ttyIPC root=mtd1 quiet " \
	"vdec.pool_size=0 vdec.dpb_pool_size=0 vdec.dpb_base=0x%llx " \
	"vdec.vdec_run_in_lite_model=0 vdec.vo_run_in_lite_model=0"
/* Xtensa boot-parameter tags. */
#define BP_TAG_FIRST		0x7b0b
#define BP_TAG_COMMAND_LINE	0x1001
#define BP_TAG_LAST		0x7e0b

/*
 * MCU mailbox ("IPC"): word 0 is the firmware-ready flag; the rest holds the
 * {head,tail} descriptors for the shared-memory rings. The host owns the "send"
 * side of each pair; the coprocessor owns the matching "receive" side.
 */
#define VDEC_MCU_IPC_PHYS	0xc9b00800
#define VDEC_MCU_IPC_SIZE	0x100
#define IPC_READY_FLAG		0x00	/* build magic, written once the firmware's init is done */
#define IPC_LOG_SEND_ADDR	0x60	/* firmware writes this during IPC init */
/* The firmware's log ring (descriptor at +0x60: addr, size, head, tail, in
 * the argument area). The firmware stalls when it is full, so the host keeps
 * it drained; the latest part is kept for debugfs. */
#define IPC_FW_LOG		0x60
#define VDEC_FW_LOG_KEEP	SZ_64K
/* Console ring descriptors (host view): tx owned by us, rx owned by firmware. */
#define IPC_UART_TX_HEAD	0x70
#define IPC_UART_TX_TAIL	0x74
#define IPC_UART_RX_HEAD	0x78
#define IPC_UART_RX_TAIL	0x7c

/* IPC area layout: a small queue area, then a 1 MiB argument area. */
#define VDEC_QUEUE_OFF		0x0
#define VDEC_QUEUE_SIZE		0x1000
#define VDEC_ARG_OFF		0x1000
#define VDEC_ARG_SIZE		0x100000
#define VDEC_IPC_AREA_SIZE	(VDEC_ARG_OFF + VDEC_ARG_SIZE)
/* Console byte rings within the argument area (host view). */
#define VDEC_UART_TX_OFF	0x40000
#define VDEC_UART_RX_OFF	0x60000
#define VDEC_UART_RING_SIZE	0x20000

/*
 * Message layer. The mailbox holds four-word queue descriptors
 * {addr, size, head, tail}; addr is a byte offset into the queue area (the
 * carveout base) to a ring of fixed-size q_arguments. Host owns the send
 * queue's head and the receive queue's tail; the coprocessor owns the others.
 * The host must publish its send descriptor's addr/size: the firmware reads it
 * as its own receive queue.
 */
#define IPC_SEND_QUEUE		0x10	/* host -> coprocessor */
#define IPC_RECV_QUEUE		0x20	/* coprocessor -> host */
#define IPC_PIPE_SEND		0x30	/* host -> coprocessor byte pipe */
#define IPC_LOG_SEND		0x50	/* host -> coprocessor log */
/* Host-owned byte rings in the argument area (the coprocessor's own pipe and
 * log rings sit at 0xa0000 and 0xe0000). */
#define VDEC_PIPE_SEND_OFF	0x80000
#define VDEC_LOG_SEND_OFF	0xc0000
#define VDEC_BYTE_RING_SIZE	0x20000
#define Q_ADDR			0x0
#define Q_SIZE			0x4
#define Q_HEAD			0x8
#define Q_TAIL			0xc
#define VDEC_SEND_RING_OFF	0x0	/* host send ring, into the queue area */
#define VDEC_QARG_SIZE		20	/* sizeof(struct vdec_qarg) */

struct vdec_qarg {
	u32 target_number;
	u32 type;
	u32 size;
	u32 offs;
	u32 ipc_number;
};

/*
 * Message types. 0..5 are the transport's own (new/kill/return call machinery,
 * pipe, log, ioctl); the 0x1000_xxxx values are the vdec async notifications.
 * Ping/pong is the simplest round-trip: the firmware answers a ping with a pong.
 */
enum {
	VDEC_MSG_NEW,
	VDEC_MSG_KILL,
	VDEC_MSG_RETURN,
	VDEC_MSG_PIPE,
	VDEC_MSG_LOG,
	VDEC_MSG_IOCTL,
};
#define IPC_VDEC_TYPE_POLL_VDEC	0x10000000	/* copro->host: input consumed */
#define IPC_VDEC_TYPE_POLL_VO	0x10000001	/* copro->host: output readiness */
#define IPC_VDEC_TYPE_POLL_VO	0x10000001	/* copro->host: frame ready */
#define IPC_VDEC_TYPE_PONG	0x10000002	/* copro->host: ping reply */
#define IPC_VDEC_TYPE_SUSPEND	0x10001000	/* host->copro */
#define IPC_VDEC_TYPE_PING	0x10001001	/* host->copro */

/*
 * Call layer. A call is a VDEC_MSG_NEW message whose offs/size locate a
 * parameter block in the argument area: a vdec_call_hdr followed by the data.
 * The callee answers with VDEC_MSG_RETURN, target_number = the call's
 * ipc_number, offs = the (signed) result. VDEC_MSG_KILL asks the callee to
 * abandon a call; it still returns. Each side allocates its parameter blocks
 * from its own part of the argument area.
 */
struct vdec_call_hdr {
	u32 call;
	u32 priv;
	u32 data_size;
};

#define VDEC_PARAM_POOL_OFF	0x0	/* host parameter blocks, in the arg area */
#define VDEC_PARAM_POOL_SIZE	0x20000	/* (the firmware's follow, at 0x20000) */
#define VDEC_CALL_TIMEOUT_MS	2000
#define VDEC_RUNNING_TIMEOUT_MS	5000

/* Host->firmware calls are numbered (lg1314-vdec.h); a few extra are FourCCs. */
#define VDEC_FOURCC(a, b, c, d)	((a) | ((b) << 8) | ((c) << 16) | ((d) << 24))
#define VDEC_CALL_UCOM		VDEC_FOURCC('u', 'c', 'o', 'm')	/* firmware init command */

/* Firmware->host calls, by number. */
static const char * const vdec_remote_call_names[] = {
	"hma_alloc_vdec",
	"hma_alloc_dpb",
	"hma_alloc_pool",
	"hma_free_vdec",
	"hma_free_dpb",
	"hma_free_pool",
	"audioinf_get_basetime",
	"clockgate",
	"bus_port_set",
	"kick_te",
};

#define VDEC_READY_TIMEOUT_MS	1500

struct lg1314_vdec {
	struct device		*dev;
	void __iomem		*rc;		/* MCU control block */
	void __iomem		*ipc;		/* MCU mailbox */
	void			*queue;		/* message ring area (carveout base, WC) */
	void			*arg;		/* argument area (carveout, WC) */
	u32			ipc_count;	/* our outgoing ipc_number sequence */
	spinlock_t		send_lock;	/* serialises the send ring */
	int			irq;

	struct gen_pool		*param_pool;	/* host call parameter blocks */
	spinlock_t		wait_lock;	/* protects waiters */
	struct list_head	waiters;	/* our calls awaiting a return */
	struct workqueue_struct	*wq;		/* runs firmware->host calls */

	struct gen_pool		*hma_vdec;	/* firmware buffer pools (phys) */
	struct gen_pool		*hma_cpb;
	struct gen_pool		*hma_shared;	/* dpb overflow (no-map), or NULL */
	struct list_head	hma_allocs;	/* live allocations, for frees */
	struct mutex		hma_lock;
	/* A decoded picture's key goes to the video output only while its
	 * decoder session is open: writers of such frames read, closing a
	 * session writes. */
	struct rw_semaphore	pictures_lock;

	spinlock_t		dovi_lock;	/* the Dolby Vision listener */
	lg1314_vdec_dovi_fn	dovi_fn;
	void			*dovi_data;

	struct lg1314_vdec_pt	*pt;		/* raw decoder interface, if built */
	struct lg1314_vdec_dec	*dec;		/* V4L2 decoder */

	struct resource		mcu_mem;	/* cpb pool, IPC area, ROM, vdec pool */
	struct resource		dpb_mem;	/* decoded pictures */
	struct resource		shared_mem[VDEC_SHARED_MAX];	/* dpb overflow */
	unsigned int		n_shared;
	phys_addr_t		ipc_phys;	/* IPC area ("param_base") */
	phys_addr_t		rom_phys;	/* firmware image */
	void __iomem		*rom;		/* ROM region (image + bootparam) */

	struct uart_driver	uart_driver;
	struct uart_port	uart_port;

	spinlock_t		fwlog_lock;	/* the firmware log ring's tail */
	char			*fwlog;		/* its latest VDEC_FW_LOG_KEEP bytes */
	size_t			fwlog_len;	/* bytes ever kept */
	struct delayed_work	fwlog_work;	/* drains it when idle */
	struct dentry		*fwlog_file;
	bool			rx_running;

	bool			ready;		/* firmware up, interfaces registered */
};

/* ---- doorbell ---- */

static void vdec_ring_doorbell(struct lg1314_vdec *v)
{
	writel(1, v->rc + RC_TO_INTERNAL);
}

/* ---- message layer ---- */

static u32 vdec_qfield(struct lg1314_vdec *v, u32 queue, u32 field)
{
	return readl(v->ipc + queue + field);
}

/* Publish an empty host-owned byte ring (offset into the argument area). */
static void vdec_publish_ring(struct lg1314_vdec *v, u32 queue, u32 off)
{
	writel(off, v->ipc + queue + Q_ADDR);
	writel(VDEC_BYTE_RING_SIZE, v->ipc + queue + Q_SIZE);
	writel(0, v->ipc + queue + Q_HEAD);
	writel(0, v->ipc + queue + Q_TAIL);
}

/*
 * Post one q_argument to the host->coprocessor ring and ring the doorbell. The
 * message's sequence number is returned through ipc_number (if non-NULL).
 */
static int vdec_msg_send(struct lg1314_vdec *v, u32 type, u32 target,
			 u32 offs, u32 size, u32 *ipc_number)
{
	struct vdec_qarg arg = {
		.target_number = target,
		.type = type,
		.size = size,
		.offs = offs,
	};
	u32 ring = vdec_qfield(v, IPC_SEND_QUEUE, Q_ADDR);
	u32 qsize = vdec_qfield(v, IPC_SEND_QUEUE, Q_SIZE);
	u32 head, tail, next;
	unsigned long flags;

	if (!qsize)
		return -ENODEV;		/* send descriptor not published yet */

	spin_lock_irqsave(&v->send_lock, flags);
	head = vdec_qfield(v, IPC_SEND_QUEUE, Q_HEAD);
	tail = vdec_qfield(v, IPC_SEND_QUEUE, Q_TAIL);
	next = (head + 1) % qsize;
	if (next == tail) {
		spin_unlock_irqrestore(&v->send_lock, flags);
		dev_warn(v->dev, "send queue full (head %u tail %u)\n", head, tail);
		return -EBUSY;
	}

	arg.ipc_number = v->ipc_count++;
	memcpy(v->queue + ring + head * VDEC_QARG_SIZE, &arg, sizeof(arg));
	wmb();				/* publish the entry before the index */
	writel(next, v->ipc + IPC_SEND_QUEUE + Q_HEAD);
	spin_unlock_irqrestore(&v->send_lock, flags);

	if (ipc_number)
		*ipc_number = arg.ipc_number;
	vdec_ring_doorbell(v);
	return 0;
}

/* ---- call layer ---- */

struct vdec_waiter {
	struct list_head	list;
	struct completion	done;
	u32			ipc_number;
	int			ret;
};

struct vdec_remote_call {
	struct work_struct	work;
	struct lg1314_vdec	*v;
	struct vdec_qarg	arg;
};

/* Calls need the firmware's init to have finished (its handlers are modules). */
static int vdec_wait_running(struct lg1314_vdec *v)
{
	unsigned long deadline = jiffies + msecs_to_jiffies(VDEC_RUNNING_TIMEOUT_MS);

	while (!readl(v->ipc + IPC_READY_FLAG)) {
		if (time_after(jiffies, deadline))
			return -ETIMEDOUT;
		msleep(10);
	}
	return 0;
}

/*
 * Call a firmware function and wait for its result. data/size is the call's
 * argument, copied into a parameter block before the call (VDEC_CALL_WR) and
 * back out after it (VDEC_CALL_RD). Returns 0 and the callee's result in
 * *result, or a negative errno if the call could not be made.
 */
int lg1314_vdec_call(struct lg1314_vdec *v, u32 call, u32 priv,
		     void *data, u32 size, unsigned int dir, int *result)
{
	u32 len = sizeof(struct vdec_call_hdr) + size;
	struct vdec_call_hdr *hdr;
	struct vdec_waiter w;
	unsigned long flags, va;
	bool returned;
	int ret;

	might_sleep();
	if (size > VDEC_CALL_MAX_DATA)
		return -EINVAL;
	ret = vdec_wait_running(v);
	if (ret)
		return ret;

	va = gen_pool_alloc(v->param_pool, len);
	if (!va)
		return -ENOMEM;
	hdr = (struct vdec_call_hdr *)va;
	hdr->call = call;
	hdr->priv = priv;
	hdr->data_size = size;
	if (size) {
		if (dir & VDEC_CALL_WR)
			memcpy(hdr + 1, data, size);
		else
			memset(hdr + 1, 0, size);
	}

	/* Register the waiter before the doorbell: replies can be very quick. */
	init_completion(&w.done);
	spin_lock_irqsave(&v->wait_lock, flags);
	ret = vdec_msg_send(v, VDEC_MSG_NEW, 0, va - (unsigned long)v->arg, len,
			    &w.ipc_number);
	if (!ret)
		list_add_tail(&w.list, &v->waiters);
	spin_unlock_irqrestore(&v->wait_lock, flags);
	if (ret)
		goto out_free;

	if (!wait_for_completion_timeout(&w.done,
					 msecs_to_jiffies(VDEC_CALL_TIMEOUT_MS))) {
		/* Ask the firmware to abandon it; a killed call still returns. */
		dev_warn(v->dev, "call %#x (num %u) timed out, killing\n",
			 call, w.ipc_number);
		vdec_msg_send(v, VDEC_MSG_KILL, w.ipc_number, 0, 0, NULL);
		wait_for_completion_timeout(&w.done,
					    msecs_to_jiffies(VDEC_CALL_TIMEOUT_MS));
	}

	spin_lock_irqsave(&v->wait_lock, flags);
	returned = list_empty(&w.list);	/* the return path unlinks us */
	if (!returned)
		list_del(&w.list);
	spin_unlock_irqrestore(&v->wait_lock, flags);
	if (!returned) {
		/* The firmware may still write the block: leak it. */
		dev_err(v->dev, "call %#x (num %u) never returned\n",
			call, w.ipc_number);
		return -ETIMEDOUT;
	}

	if (size && (dir & VDEC_CALL_RD))
		memcpy(data, hdr + 1, size);
	*result = w.ret;
	ret = 0;
out_free:
	gen_pool_free(v->param_pool, va, len);
	return ret;
}

/* A VDEC_MSG_RETURN for one of our calls. IRQ context. */
static void vdec_call_returned(struct lg1314_vdec *v, u32 ipc_number, int ret)
{
	struct vdec_waiter *w;

	spin_lock(&v->wait_lock);
	list_for_each_entry(w, &v->waiters, list) {
		if (w->ipc_number == ipc_number) {
			list_del_init(&w->list);
			w->ret = ret;
			complete(&w->done);
			spin_unlock(&v->wait_lock);
			return;
		}
	}
	spin_unlock(&v->wait_lock);
	dev_warn(v->dev, "return for unknown call %u\n", ipc_number);
}

/* ---- for the layers on top ---- */

/* A buffer handed to the firmware (hma call). */
struct vdec_hma_alloc {
	struct list_head	list;
	struct gen_pool		*pool;	/* NULL: from the dpb CMA area (DMA API) */
	void			*vaddr;	/* dpb: kernel mapping (overflow pool: WC) */
	void			*lent;	/* dpb: a decoder's output buffer instead */
	u32			phys;
	u32			size;
};


/*
 * For the display driver: the decoder behind @np, holding a reference to its
 * device. Unbinding the decoder while the display uses it is not supported.
 */
struct lg1314_vdec *lg1314_vdec_get(struct device_node *np)
{
	struct platform_device *pdev = of_find_device_by_node(np);
	struct lg1314_vdec *v;

	if (!pdev)
		return ERR_PTR(-EPROBE_DEFER);
	device_lock(&pdev->dev);
	v = platform_get_drvdata(pdev);
	if (!v || !READ_ONCE(v->ready))
		v = ERR_PTR(-EPROBE_DEFER);
	device_unlock(&pdev->dev);
	if (IS_ERR(v))
		put_device(&pdev->dev);
	return v;
}
EXPORT_SYMBOL_GPL(lg1314_vdec_get);

void lg1314_vdec_put(struct lg1314_vdec *v)
{
	put_device(v->dev);
}
EXPORT_SYMBOL_GPL(lg1314_vdec_put);

struct device *lg1314_vdec_device(struct lg1314_vdec *v)
{
	return v->dev;
}

/*
 * Coprocessor address -> physical, through the windows as programmed (code
 * window at 0x5000_0000, then 512 MiB data windows at 0x6/0x8/0xa000_0000).
 */
phys_addr_t lg1314_vdec_copro_to_phys(struct lg1314_vdec *v, u32 addr)
{
	static const struct { u32 base, reg; } win[] = {
		{ 0x50000000, RC_SROM_OFFSET },
		{ 0x60000000, RC_SRAM_OFFSET_0 },
		{ 0x80000000, RC_SRAM_OFFSET_1 },
		{ 0xa0000000, RC_SRAM_OFFSET_2 },
	};
	int i;

	for (i = ARRAY_SIZE(win) - 1; i >= 0; i--) {
		if (addr >= win[i].base) {
			if (addr - win[i].base >= (i ? VDEC_WINDOW_SIZE : 0x10000000))
				return 0;
			return readl(v->rc + win[i].reg) + (addr - win[i].base);
		}
	}
	return 0;
}

static bool vdec_in(const struct resource *r, phys_addr_t phys, size_t size)
{
	return phys >= r->start && phys <= r->end && size <= r->end - phys + 1;
}

/* The firmware's memory: its own regions and the buffers it is given. */
bool lg1314_vdec_in_carveout(struct lg1314_vdec *v, phys_addr_t phys, size_t size)
{
	unsigned int i;

	if (vdec_in(&v->mcu_mem, phys, size) || vdec_in(&v->dpb_mem, phys, size))
		return true;
	for (i = 0; i < v->n_shared; i++)
		if (vdec_in(&v->shared_mem[i], phys, size))
			return true;
	return false;
}

/*
 * Kernel mapping of [phys, phys + len) inside a dpb-pool buffer (a decoded
 * frame), made coherent for CPU reads: cached from the CMA area, uncached
 * from the overflow pool. NULL if it isn't one.
 */
const void *lg1314_vdec_frame_map(struct lg1314_vdec *v, phys_addr_t phys,
				  size_t len)
{
	struct vdec_hma_alloc *a;
	const void *p = NULL;

	mutex_lock(&v->hma_lock);
	list_for_each_entry(a, &v->hma_allocs, list) {
		if ((a->pool && a->pool != v->hma_shared) || a->lent ||
		    phys < a->phys || phys - a->phys >= a->size ||
		    len > a->size - (phys - a->phys))
			continue;
		if (!a->pool)
			dma_sync_single_for_cpu(v->dev, phys, len, DMA_FROM_DEVICE);
		p = a->vaddr + (phys - a->phys);
		break;
	}
	mutex_unlock(&v->hma_lock);
	return p;
}

/*
 * A small host-filled buffer the firmware reads (from the vdec pool), with a
 * write-combined kernel mapping. Returns NULL on failure.
 */
void *lg1314_vdec_mem_alloc(struct lg1314_vdec *v, size_t size, u32 *phys)
{
	unsigned long p = gen_pool_alloc(v->hma_vdec, size);
	void *va;

	if (!p)
		return NULL;
	va = memremap(p, size, MEMREMAP_WC);
	if (!va) {
		gen_pool_free(v->hma_vdec, p, size);
		return NULL;
	}
	*phys = p;
	return va;
}

void lg1314_vdec_mem_free(struct lg1314_vdec *v, void *va, u32 phys, size_t size)
{
	memunmap(va);
	gen_pool_free(v->hma_vdec, phys, size);
}

/* ---- firmware buffer allocator (firmware->host hma calls) ---- */

/* Pool by the firmware's name for it. */
static struct gen_pool *vdec_hma_pool(struct lg1314_vdec *v, const char *name)
{
	if (!strcmp(name, "dpb"))
		return NULL;		/* the dpb CMA area */
	if (!strcmp(name, "cpb"))
		return v->hma_cpb;
	if (strcmp(name, "vdec"))
		dev_info(v->dev, "unknown pool \"%s\", using vdec\n", name);
	return v->hma_vdec;
}

/* Returns the buffer's physical address, or 0 (the firmware's failure value). */
/*
 * pool NULL = the dpb pool: a CMA area, allocated through the DMA API so the
 * memory has a (cached) kernel mapping for reading decoded frames. DMA
 * allocations are aligned to their (power-of-two rounded) size, so asking for
 * at least the alignment gets it.
 */
static unsigned long vdec_hma_alloc_dpb(struct lg1314_vdec *v,
					struct vdec_hma_alloc *a, u32 size,
					u32 align)
{
	dma_addr_t dma;

	/* The firmware retries a failed request at once and for ever: no
	 * allocator report each time (it floods the console). */
	a->vaddr = dma_alloc_noncoherent(v->dev, max(size, align), &dma,
					 DMA_FROM_DEVICE, GFP_KERNEL | __GFP_NOWARN);
	if (!a->vaddr)
		return 0;
	if (align && (dma & (align - 1))) {
		dma_free_noncoherent(v->dev, max(size, align), a->vaddr, dma,
				     DMA_FROM_DEVICE);
		return 0;
	}
	/* The allocator cleared it through the cache: write that back now, or
	 * evicted lines land on top of what the firmware writes later. */
	dma_sync_single_for_device(v->dev, dma, max(size, align), DMA_FROM_DEVICE);
	return dma;
}

/*
 * The dpb pool's overflow: no-map memory, so mapped write-combined, for the
 * rare picture the CPU copies out. Sets a->pool. The display firmware must be
 * told about it too (its second video pool), or it won't show pictures from
 * it.
 */
static unsigned long vdec_hma_alloc_shared(struct lg1314_vdec *v,
					   struct vdec_hma_alloc *a, u32 size,
					   u32 align, const char *name)
{
	struct genpool_data_align data = { .align = max(align, 1u) };
	unsigned long phys;

	phys = gen_pool_alloc_algo(v->hma_shared, size, gen_pool_first_fit_align, &data);
	if (!phys)
		return 0;
	a->vaddr = memremap(phys, size, MEMREMAP_WC);
	if (!a->vaddr) {
		gen_pool_free(v->hma_shared, phys, size);
		return 0;
	}
	/* Zeroed, as from the CMA area: the firmware doesn't write every part
	 * of a picture buffer. */
	memset(a->vaddr, 0, size);
	a->pool = v->hma_shared;
	dev_dbg(v->dev, "hma: %u bytes for \"%s\" from the overflow pool at %#lx\n",
		size, name, phys);
	return phys;
}

static void vdec_hma_free_one(struct lg1314_vdec *v, struct vdec_hma_alloc *a)
{
	if (a->vaddr && a->pool)
		memunmap(a->vaddr);
	if (a->pool)
		gen_pool_free(a->pool, a->phys, a->size);
	else if (a->lent)
		lg1314_vdec_dec_dpb_free(a->lent);
	else
		dma_free_noncoherent(v->dev, a->size, a->vaddr, a->phys,
				     DMA_FROM_DEVICE);
	kfree(a);
}

void lg1314_vdec_pictures_lock(struct lg1314_vdec *v, bool closing)
{
	if (closing)
		down_write(&v->pictures_lock);
	else
		down_read(&v->pictures_lock);
}

void lg1314_vdec_pictures_unlock(struct lg1314_vdec *v, bool closing)
{
	if (closing)
		up_write(&v->pictures_lock);
	else
		up_read(&v->pictures_lock);
}

void lg1314_vdec_vo_hold(struct lg1314_vdec *v, u32 addr, bool hold)
{
	lg1314_vdec_dec_vo_hold(v->dec, addr, hold);
}

u32 lg1314_vdec_picture_priv(struct lg1314_vdec *v, u32 addr, u32 *addr_c)
{
	return lg1314_vdec_dec_picture_priv(v->dec, addr, addr_c);
}

int lg1314_vdec_picture_dovi(struct lg1314_vdec *v, u32 addr, u64 *timestamp,
			     u32 *session, struct lg1314_dovi_pic *pic)
{
	return lg1314_vdec_dec_picture_dovi(v->dec, addr, timestamp, session, pic);
}
EXPORT_SYMBOL_GPL(lg1314_vdec_picture_dovi);

void lg1314_vdec_set_dovi_listener(struct lg1314_vdec *v, lg1314_vdec_dovi_fn fn,
				   void *data)
{
	spin_lock(&v->dovi_lock);
	v->dovi_fn = fn;
	v->dovi_data = data;
	spin_unlock(&v->dovi_lock);
}
EXPORT_SYMBOL_GPL(lg1314_vdec_set_dovi_listener);

void lg1314_vdec_dovi_parsed(struct lg1314_vdec *v, const struct lg1314_dovi_dm *dm)
{
	spin_lock(&v->dovi_lock);
	if (v->dovi_fn)
		v->dovi_fn(v->dovi_data, dm);
	spin_unlock(&v->dovi_lock);
}

static u32 vdec_hma_alloc(struct lg1314_vdec *v, struct gen_pool *pool,
			  u32 size, u32 align, const char *name)
{
	struct genpool_data_align data = { .align = align };
	struct vdec_hma_alloc *a;
	unsigned long phys;

	if (!size)
		return 0;
	a = kzalloc(sizeof(*a), GFP_KERNEL);
	if (!a)
		return 0;
	if (pool) {
		phys = gen_pool_alloc_algo(pool, size, gen_pool_first_fit_align, &data);
	} else {
		/* A decoded-picture buffer goes straight to userspace when a
		 * decoder has an output buffer to lend for it (H.264: gx_dpb;
		 * HEVC: ipt17, along with its working buffers, which don't fit
		 * one). */
		bool wait = false;

		phys = 0;
		if (!strcmp(name, "gx_dpb") || !strcmp(name, "ipt17"))
			phys = lg1314_vdec_dec_dpb_alloc(v->dec, size, align,
							 &a->lent, &wait);
		if (wait) {
			/* The firmware asks again at once: not too often. */
			kfree(a);
			msleep(5);
			return 0;
		}
		if (!phys) {
			phys = vdec_hma_alloc_dpb(v, a, size, align);
			size = max(size, align);
		}
		if (!phys && v->hma_shared)
			phys = vdec_hma_alloc_shared(v, a, size, align, name);
	}
	if (!phys) {
		dev_warn_ratelimited(v->dev, "hma: out of memory for %u bytes (align %#x) in %s for \"%s\"\n",
				     size, align, pool ? (pool == v->hma_cpb ? "cpb" : "vdec") : "dpb",
				     name);
		kfree(a);
		/* The firmware gives up on the stream (a working buffer: the
		 * codec never starts, decoding times out for ever): fail it,
		 * so the player sees an error rather than a frozen picture. */
		if (!pool)
			lg1314_vdec_dec_dpb_failed(v->dec);
		return 0;
	}
	if (pool)
		a->pool = pool;	/* else NULL, or the overflow pool */
	a->phys = phys;
	a->size = size;
	mutex_lock(&v->hma_lock);
	list_add(&a->list, &v->hma_allocs);
	mutex_unlock(&v->hma_lock);
	return phys;
}

static void vdec_hma_free(struct lg1314_vdec *v, u32 phys)
{
	struct vdec_hma_alloc *a;

	mutex_lock(&v->hma_lock);
	list_for_each_entry(a, &v->hma_allocs, list) {
		if (a->phys == phys) {
			list_del(&a->list);
			mutex_unlock(&v->hma_lock);
			vdec_hma_free_one(v, a);
			return;
		}
	}
	mutex_unlock(&v->hma_lock);
	dev_warn(v->dev, "hma: free of unknown buffer %#x\n", phys);
}

/* Firmware->host call numbers (index into vdec_remote_call_names). */
enum {
	VDEC_RCALL_HMA_ALLOC_VDEC,
	VDEC_RCALL_HMA_ALLOC_DPB,
	VDEC_RCALL_HMA_ALLOC_POOL,
	VDEC_RCALL_HMA_FREE_VDEC,
	VDEC_RCALL_HMA_FREE_DPB,
	VDEC_RCALL_HMA_FREE_POOL,
};

#define VDEC_RCALL_MAX_DATA	256

/*
 * Run one firmware->host call. data/size is the call's argument (a copy).
 * hma_alloc_pool's argument is {size, align, "pool\0name\0"}; hma_free_pool's
 * is {addr, "pool\0"} (the address alone identifies the buffer).
 */
static int vdec_remote_call_run(struct lg1314_vdec *v, u32 call, u32 priv,
				u8 *data, u32 size)
{
	u32 *w = (u32 *)data;
	u32 phys;

	switch (call) {
	case VDEC_RCALL_HMA_ALLOC_VDEC:
	case VDEC_RCALL_HMA_ALLOC_DPB:
	case VDEC_RCALL_HMA_ALLOC_POOL: {
		struct gen_pool *pool;
		const char *pool_name = "vdec", *name = "";

		if (size < 8)
			return 0;
		if (call == VDEC_RCALL_HMA_ALLOC_DPB) {
			pool_name = "dpb";
		} else if (call == VDEC_RCALL_HMA_ALLOC_POOL) {
			data[size - 1] = '\0';
			pool_name = (char *)data + 8;
			if (pool_name + strlen(pool_name) + 1 < (char *)data + size)
				name = pool_name + strlen(pool_name) + 1;
		}
		pool = vdec_hma_pool(v, pool_name);
		phys = vdec_hma_alloc(v, pool, w[0], w[1], name);
		if (vdec_debug)
			dev_info(v->dev, "hma: alloc %s %u (align %#x) %s = %#x\n",
				 pool_name, w[0], w[1], name, phys);
		return phys;
	}
	case VDEC_RCALL_HMA_FREE_VDEC:
	case VDEC_RCALL_HMA_FREE_DPB:
	case VDEC_RCALL_HMA_FREE_POOL:
		if (size < 4)
			return -EINVAL;
		if (vdec_debug)
			dev_info(v->dev, "hma: free %#x\n", w[0]);
		vdec_hma_free(v, w[0]);
		return 0;
	default:
		/* Not implemented yet: log what the firmware asks for. */
		dev_info(v->dev, "firmware call %s (%u) priv %#x, args %*ph: ignored\n",
			 call < ARRAY_SIZE(vdec_remote_call_names) ?
				vdec_remote_call_names[call] : "?",
			 call, priv, min_t(u32, size, 16), data);
		return 0;
	}
}

/* Run a firmware->host call and send back its result. */
static void vdec_remote_call_work(struct work_struct *work)
{
	struct vdec_remote_call *rc = container_of(work, struct vdec_remote_call, work);
	struct lg1314_vdec *v = rc->v;
	u8 data[VDEC_RCALL_MAX_DATA + 1];
	struct vdec_call_hdr hdr;
	u32 dsize;
	int ret;
	int tries;

	if (rc->arg.size < sizeof(hdr) || rc->arg.offs > VDEC_ARG_SIZE ||
	    rc->arg.size > VDEC_ARG_SIZE - rc->arg.offs) {
		dev_err(v->dev, "bad call block %#x/%u\n", rc->arg.offs, rc->arg.size);
		ret = -EINVAL;
		goto reply;
	}
	memcpy(&hdr, v->arg + rc->arg.offs, sizeof(hdr));
	dsize = min3(hdr.data_size, rc->arg.size - (u32)sizeof(hdr),
		     (u32)VDEC_RCALL_MAX_DATA);
	memcpy(data, v->arg + rc->arg.offs + sizeof(hdr), dsize);
	data[dsize] = '\0';

	ret = vdec_remote_call_run(v, hdr.call, hdr.priv, data, dsize);

reply:
	/* The caller blocks until we answer, so retry while the ring is full. */
	for (tries = 0; tries < 100; tries++) {
		if (vdec_msg_send(v, VDEC_MSG_RETURN, rc->arg.ipc_number, ret, 0,
				  NULL) != -EBUSY)
			break;
		msleep(1);
	}
	kfree(rc);
}

/* A VDEC_MSG_NEW from the firmware. IRQ context: defer to the workqueue. */
static void vdec_remote_call(struct lg1314_vdec *v, const struct vdec_qarg *arg)
{
	struct vdec_remote_call *rc = kzalloc(sizeof(*rc), GFP_ATOMIC);

	if (!rc) {
		vdec_msg_send(v, VDEC_MSG_RETURN, arg->ipc_number, -ENOMEM, 0, NULL);
		return;
	}
	INIT_WORK(&rc->work, vdec_remote_call_work);
	rc->v = v;
	rc->arg = *arg;
	queue_work(v->wq, &rc->work);
}

static void vdec_msg_dispatch(struct lg1314_vdec *v, const struct vdec_qarg *arg)
{
	switch (arg->type) {
	case VDEC_MSG_NEW:
		vdec_remote_call(v, arg);
		break;
	case VDEC_MSG_RETURN:
		vdec_call_returned(v, arg->target_number, (int)arg->offs);
		break;
	case IPC_VDEC_TYPE_POLL_VDEC:
		/* target_number = decoder handle, offs = poll mask */
		lg1314_vdec_pt_poll(v->pt, arg->target_number, arg->offs);
		lg1314_vdec_dec_poll(v->dec, arg->target_number, arg->offs);
		break;
	case IPC_VDEC_TYPE_POLL_VO:
		/* target_number = output handle, offs = poll mask */
		lg1314_vdec_pt_poll(v->pt, arg->target_number, arg->offs);
		lg1314_vdec_vo_poll(arg->target_number, arg->offs);
		break;
	default:
		dev_info(v->dev,
			 "msg: type %#010x target %u offs %#x size %u num %u\n",
			 arg->type, arg->target_number, arg->offs, arg->size,
			 arg->ipc_number);
		break;
	}
}

/* Drain the coprocessor->host message ring. Called from the doorbell IRQ. */
static void vdec_msg_rx(struct lg1314_vdec *v)
{
	u32 ring = vdec_qfield(v, IPC_RECV_QUEUE, Q_ADDR);
	u32 size = vdec_qfield(v, IPC_RECV_QUEUE, Q_SIZE);
	u32 head = vdec_qfield(v, IPC_RECV_QUEUE, Q_HEAD);
	u32 tail = vdec_qfield(v, IPC_RECV_QUEUE, Q_TAIL);

	if (!size || head == tail)
		return;

	while (tail != head) {
		struct vdec_qarg arg;

		memcpy(&arg, v->queue + ring + tail * VDEC_QARG_SIZE, sizeof(arg));
		vdec_msg_dispatch(v, &arg);
		tail = (tail + 1) % size;
	}
	writel(tail, v->ipc + IPC_RECV_QUEUE + Q_TAIL);
}

/* ---- sysfs test triggers ---- */

/*
 * `echo 1 > /sys/module/lg1314_vdec/parameters/ping` sends a ping carrying a
 * sequence number in offs; the firmware echoes the latest one back in a pong's
 * offs (back-to-back pings coalesce into one pong).
 */
static u32 vdec_ping_seq;

static int vdec_ping_set(const char *val, const struct kernel_param *kp)
{
	struct lg1314_vdec *v = vdec_instance;
	int ret;

	if (!v)
		return -ENODEV;
	ret = vdec_msg_send(v, IPC_VDEC_TYPE_PING, 0, ++vdec_ping_seq, 0, NULL);
	if (ret)
		return ret;
	dev_info(v->dev, "sent ping %u\n", vdec_ping_seq);
	return 0;
}

static const struct kernel_param_ops vdec_ping_ops = {
	.set = vdec_ping_set,
};
module_param_cb(ping, &vdec_ping_ops, NULL, 0200);
MODULE_PARM_DESC(ping, "send a ping message to the coprocessor");

/*
 * `echo 'a:b' > /sys/module/lg1314_vdec/parameters/ucom` calls the firmware's
 * user-command entry: its init splits the string on ':' and logs each part
 * (the firmware log is at arg + 0xe0000), so it exercises a full call.
 */
static int vdec_ucom_set(const char *val, const struct kernel_param *kp)
{
	struct lg1314_vdec *v = vdec_instance;
	char cmd[128];
	int ret, result;

	if (!v)
		return -ENODEV;
	strscpy(cmd, val, sizeof(cmd));
	cmd[strcspn(cmd, "\n")] = '\0';
	ret = lg1314_vdec_call(v, VDEC_CALL_UCOM, 0, cmd, strlen(cmd) + 1,
			VDEC_CALL_WR, &result);
	if (ret)
		return ret;
	dev_info(v->dev, "ucom \"%s\" = %d\n", cmd, result);
	return 0;
}

static const struct kernel_param_ops vdec_ucom_ops = {
	.set = vdec_ucom_set,
};
module_param_cb(ucom, &vdec_ucom_ops, NULL, 0200);
MODULE_PARM_DESC(ucom, "send a user command to the coprocessor's init (call test)");

/*
 * `echo 1 > /sys/module/lg1314_vdec/parameters/open` opens a decoder instance
 * and closes it again, logging the handle (the firmware's own pointer) and any
 * buffers the firmware requests meanwhile.
 */
static int vdec_open_set(const char *val, const struct kernel_param *kp)
{
	struct lg1314_vdec *v = vdec_instance;
	int ret, handle, result;

	if (!v)
		return -ENODEV;
	ret = lg1314_vdec_call(v, VDEC_CALL_VDEC_OPEN, 0, NULL, 0, 0, &handle);
	if (ret)
		return ret;
	dev_info(v->dev, "vdec open = %#x\n", handle);
	if (!handle)
		return -EIO;
	ret = lg1314_vdec_call(v, VDEC_CALL_VDEC_CLOSE, handle, NULL, 0, 0, &result);
	if (ret)
		return ret;
	dev_info(v->dev, "vdec close = %d\n", result);
	return 0;
}

static const struct kernel_param_ops vdec_open_ops = {
	.set = vdec_open_set,
};
module_param_cb(open, &vdec_open_ops, NULL, 0200);
MODULE_PARM_DESC(open, "open and close a decoder instance (call test)");

/* ---- reset / boot ---- */

static int vdec_load_firmware(struct lg1314_vdec *v)
{
	const struct firmware *fw;
	int ret;

	ret = request_firmware(&fw, VDEC_FW_NAME, v->dev);
	if (ret) {
		dev_err(v->dev, "cannot load %s: %d\n", VDEC_FW_NAME, ret);
		return ret;
	}
	if (fw->size > VDEC_ROM_SIZE - VDEC_BOOTPARAM_MAX) {
		dev_err(v->dev, "firmware (%zu) too large for ROM region\n", fw->size);
		ret = -EFBIG;
		goto out;
	}
	memcpy_toio(v->rom, fw->data, fw->size);
	dev_info(v->dev, "loaded %s (%zu bytes) at %pa\n",
		 VDEC_FW_NAME, fw->size, &v->rom_phys);
out:
	release_firmware(fw);
	return ret;
}

/*
 * Build the Xtensa boot-parameter tag list (just a command line) at the end of
 * the ROM region, and point the MCU param register at it.
 */
static void vdec_setup_bootparams(struct lg1314_vdec *v)
{
	u8 bp[VDEC_BOOTPARAM_MAX];
	char cmdline[160];
	unsigned int clen, a = 0;

	/* The firmware's own DPB pool is disabled (dpb_pool_size=0, it asks us
	 * instead); point dpb_base at our dpb pool anyway. */
	snprintf(cmdline, sizeof(cmdline), VDEC_CMDLINE, (u64)v->dpb_mem.start);
	clen = (strlen(cmdline) + 1 + 7) & ~7u;

	memset(bp, 0, sizeof(bp));
	/* BP_TAG_FIRST, size 0 */
	bp[a++] = BP_TAG_FIRST & 0xff; bp[a++] = BP_TAG_FIRST >> 8;
	bp[a++] = 0; bp[a++] = 0;
	/* BP_TAG_COMMAND_LINE, size clen, then the string */
	bp[a++] = BP_TAG_COMMAND_LINE & 0xff; bp[a++] = BP_TAG_COMMAND_LINE >> 8;
	bp[a++] = clen & 0xff; bp[a++] = clen >> 8;
	memcpy(bp + a, cmdline, strlen(cmdline) + 1);
	a += clen;
	/* BP_TAG_LAST, size 0 */
	bp[a++] = BP_TAG_LAST & 0xff; bp[a++] = BP_TAG_LAST >> 8;
	bp[a++] = 0; bp[a++] = 0;

	memcpy_toio(v->rom + (VDEC_ROM_SIZE - VDEC_BOOTPARAM_MAX), bp, sizeof(bp));
	writel(VDEC_COPRO_ROM + VDEC_ROM_SIZE - VDEC_BOOTPARAM_MAX,
	       v->rc + RC_BOOTPARAM_PTR);
}

/* Dump the mailbox so a failed boot shows whether the Xtensa executed at all. */
static void vdec_dump_mailbox(struct lg1314_vdec *v)
{
	int i;

	if (!vdec_debug)
		return;
	for (i = 0; i < 0x80; i += 0x10)
		dev_info(v->dev, "mailbox +%02x: %08x %08x %08x %08x\n", i,
			 readl(v->ipc + i), readl(v->ipc + i + 4),
			 readl(v->ipc + i + 8), readl(v->ipc + i + 12));
}

static int vdec_boot(struct lg1314_vdec *v)
{
	unsigned long deadline;
	u32 ready;

	/* Hold the core in reset + halt while we program it. */
	writel(PROC_CTRL_RESET | PROC_CTRL_HALT, v->rc + RC_PROC_CTRL);

	/* Clear the mailbox (RC+0x100.. == IPC) so the firmware starts from a known
	 * state and its writes are distinguishable from stale values. */
	memset_io(v->ipc, 0, VDEC_MCU_IPC_SIZE);

	/* Address windows: code at the firmware image, data as on stock. */
	writel(lower_32_bits(v->rom_phys), v->rc + RC_SROM_OFFSET);
	writel(VDEC_DATA0_PHYS, v->rc + RC_SRAM_OFFSET_0);
	writel(VDEC_DATA1_PHYS, v->rc + RC_SRAM_OFFSET_1);
	writel(VDEC_DATA2_PHYS, v->rc + RC_SRAM_OFFSET_2);

	/* Param registers (RC+0x100..): param_base (the IPC area) as a copro
	 * data address. */
	writel(0, v->rc + RC_PARAM_100);
	writel(lower_32_bits(v->ipc_phys - VDEC_DATA0_PHYS) + VDEC_DATA0_COPRO,
	       v->rc + RC_PARAM_108);
	writel(0, v->rc + RC_PARAM_180);
	vdec_setup_bootparams(v);

	/* Release reset, enable debug, release halt -> the core runs. */
	writel(PROC_CTRL_HALT, v->rc + RC_PROC_CTRL);
	writel(PROC_CTRL_HALT | PROC_CTRL_DEBUG, v->rc + RC_PROC_CTRL);
	writel(PROC_CTRL_DEBUG, v->rc + RC_PROC_CTRL);

	/* Wait for the firmware to bring up its IPC: it writes the log-queue
	 * descriptor (addr at mailbox +0x60) during early init. The ready flag at
	 * +0x00 is not a reliable signal (the core keeps running across host
	 * reboots, so it can hold a stale value). */
	deadline = jiffies + msecs_to_jiffies(VDEC_READY_TIMEOUT_MS);
	do {
		ready = readl(v->ipc + IPC_LOG_SEND_ADDR);
		if (ready)
			break;
		usleep_range(1000, 2000);
	} while (time_before(jiffies, deadline));

	vdec_dump_mailbox(v);
	if (!ready) {
		dev_err(v->dev, "coprocessor IPC did not come up\n");
		return -ETIMEDOUT;
	}

	/*
	 * Publish the host send-queue descriptor: ring at the start of the
	 * queue area, half the area (the other half is the coprocessor's send
	 * ring at +0x800). The firmware reads this as its own receive queue.
	 */
	writel(VDEC_SEND_RING_OFF, v->ipc + IPC_SEND_QUEUE + Q_ADDR);
	writel(VDEC_QUEUE_SIZE / 2 / VDEC_QARG_SIZE, v->ipc + IPC_SEND_QUEUE + Q_SIZE);
	writel(0, v->ipc + IPC_SEND_QUEUE + Q_HEAD);
	writel(0, v->ipc + IPC_SEND_QUEUE + Q_TAIL);
	/* Likewise the host pipe and log rings; the firmware checks their sizes
	 * on every doorbell. */
	vdec_publish_ring(v, IPC_PIPE_SEND, VDEC_PIPE_SEND_OFF);
	vdec_publish_ring(v, IPC_LOG_SEND, VDEC_LOG_SEND_OFF);

	/* Enable the Xtensa->ARM doorbell only now that its IPC is up. */
	writel(INTR_BIT, v->rc + RC_INTR_EN);
	dev_info(v->dev, "coprocessor up (log queue @ %#x, recv ring @ %#x/%u)\n",
		 ready, vdec_qfield(v, IPC_RECV_QUEUE, Q_ADDR),
		 vdec_qfield(v, IPC_RECV_QUEUE, Q_SIZE));
	return 0;
}

/* ---- console: shared-memory byte rings ---- */

/* Drain the firmware->host ring into the tty. Called from the doorbell IRQ. */
static void vdec_uart_rx(struct lg1314_vdec *v)
{
	struct tty_port *tport = &v->uart_port.state->port;
	u32 head = readl(v->ipc + IPC_UART_RX_HEAD);
	u32 tail = readl(v->ipc + IPC_UART_RX_TAIL);
	bool pushed = false;

	if (head == tail)
		return;		/* nothing to drain; don't ack (avoids a storm) */

	while (head != tail) {
		u32 chunk = (tail <= head) ? head - tail
					   : VDEC_UART_RING_SIZE - tail;
		u8 buf[256];
		int moved;

		if (chunk > sizeof(buf))
			chunk = sizeof(buf);
		memcpy(buf, v->arg + VDEC_UART_RX_OFF + tail, chunk);

		if (v->rx_running) {
			moved = tty_insert_flip_string(tport, buf, chunk);
			pushed |= moved > 0;
		} else {
			moved = chunk;	/* discard if the port isn't open */
		}
		if (moved <= 0)
			break;

		tail = (tail + moved) % VDEC_UART_RING_SIZE;
	}

	writel(tail, v->ipc + IPC_UART_RX_TAIL);
	vdec_ring_doorbell(v);
	if (pushed)
		tty_flip_buffer_push(tport);
}

/* Ring is a power of two, so masking is the modulo. */
#define VDEC_UART_RING_MASK	(VDEC_UART_RING_SIZE - 1)

/* Push pending tty data into the host->firmware ring, one byte at a time. */
static void vdec_uart_tx(struct lg1314_vdec *v)
{
	struct uart_port *port = &v->uart_port;
	u32 head0 = readl(v->ipc + IPC_UART_TX_HEAD);
	u32 head = head0;
	u8 ch;

	uart_port_tx(port, ch,
		/* tx_ready: the firmware has consumed enough to leave room */
		((readl(v->ipc + IPC_UART_TX_TAIL) - head - 1)
			& VDEC_UART_RING_MASK) != 0,
		/* put the byte into the ring and advance our head */
		({
			memcpy(v->arg + VDEC_UART_TX_OFF + head, &ch, 1);
			head = (head + 1) & VDEC_UART_RING_MASK;
		}));

	if (head != head0) {
		writel(head, v->ipc + IPC_UART_TX_HEAD);
		vdec_ring_doorbell(v);
	}
}

/* ---- firmware log ---- */

/* Consume the firmware's log ring, keeping the latest part. Any context. */
static void vdec_fwlog_drain(struct lg1314_vdec *v)
{
	u32 off, size, head, tail, n, i;
	unsigned long flags;

	spin_lock_irqsave(&v->fwlog_lock, flags);
	off = vdec_qfield(v, IPC_FW_LOG, Q_ADDR);
	size = vdec_qfield(v, IPC_FW_LOG, Q_SIZE);
	head = vdec_qfield(v, IPC_FW_LOG, Q_HEAD);
	tail = vdec_qfield(v, IPC_FW_LOG, Q_TAIL);
	if (!size || off > VDEC_ARG_SIZE || size > VDEC_ARG_SIZE - off ||
	    head >= size || tail >= size)
		goto out;
	while (tail != head) {
		n = head > tail ? head - tail : size - tail;
		if (v->fwlog)
			for (i = 0; i < n; i++)
				v->fwlog[v->fwlog_len++ % VDEC_FW_LOG_KEEP] =
					((const char *)v->arg)[off + tail + i];
		tail = (tail + n) % size;
	}
	writel(tail, v->ipc + IPC_FW_LOG + Q_TAIL);
out:
	spin_unlock_irqrestore(&v->fwlog_lock, flags);
}

static void vdec_fwlog_work(struct work_struct *work)
{
	struct lg1314_vdec *v = container_of(to_delayed_work(work),
					     struct lg1314_vdec, fwlog_work);

	vdec_fwlog_drain(v);
	schedule_delayed_work(&v->fwlog_work, msecs_to_jiffies(500));
}

static int vdec_fwlog_show(struct seq_file *m, void *unused)
{
	struct lg1314_vdec *v = m->private;
	size_t len, start, i;
	unsigned long flags;
	char *copy;

	copy = kvmalloc(VDEC_FW_LOG_KEEP, GFP_KERNEL);
	if (!copy)
		return -ENOMEM;
	spin_lock_irqsave(&v->fwlog_lock, flags);
	len = min_t(size_t, v->fwlog_len, VDEC_FW_LOG_KEEP);
	start = v->fwlog_len - len;
	for (i = 0; i < len; i++)
		copy[i] = v->fwlog[(start + i) % VDEC_FW_LOG_KEEP];
	spin_unlock_irqrestore(&v->fwlog_lock, flags);
	seq_write(m, copy, len);
	kvfree(copy);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(vdec_fwlog);

static irqreturn_t vdec_irq(int irq, void *data)
{
	struct lg1314_vdec *v = data;

	writel(INTR_BIT, v->rc + RC_INTR_CLEAR);
	vdec_msg_rx(v);
	vdec_uart_rx(v);
	vdec_fwlog_drain(v);
	return IRQ_HANDLED;
}

/* ---- uart_ops ---- */

static unsigned int vdec_tx_empty(struct uart_port *port)
{
	struct lg1314_vdec *v = container_of(port, struct lg1314_vdec, uart_port);

	return readl(v->ipc + IPC_UART_TX_HEAD) == readl(v->ipc + IPC_UART_TX_TAIL)
		? TIOCSER_TEMT : 0;
}

static void vdec_start_tx(struct uart_port *port)
{
	vdec_uart_tx(container_of(port, struct lg1314_vdec, uart_port));
}

static int vdec_startup(struct uart_port *port)
{
	container_of(port, struct lg1314_vdec, uart_port)->rx_running = true;
	return 0;
}

static void vdec_shutdown(struct uart_port *port)
{
	container_of(port, struct lg1314_vdec, uart_port)->rx_running = false;
}

static void vdec_stop_rx(struct uart_port *port)
{
	container_of(port, struct lg1314_vdec, uart_port)->rx_running = false;
}

static unsigned int vdec_get_mctrl(struct uart_port *port)
{
	return TIOCM_CAR | TIOCM_CTS | TIOCM_DSR;
}

static void vdec_set_mctrl(struct uart_port *port, unsigned int mctrl) { }
static void vdec_stop_tx(struct uart_port *port) { }
static void vdec_break_ctl(struct uart_port *port, int ctl) { }
static void vdec_set_termios(struct uart_port *port, struct ktermios *new,
			     const struct ktermios *old) { }
static const char *vdec_type(struct uart_port *port) { return "LXIPC"; }
static void vdec_release_port(struct uart_port *port) { }
static int vdec_request_port(struct uart_port *port) { return 0; }
static void vdec_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE)
		port->type = PORT_GENERIC;
}

static const struct uart_ops vdec_uart_ops = {
	.tx_empty	= vdec_tx_empty,
	.set_mctrl	= vdec_set_mctrl,
	.get_mctrl	= vdec_get_mctrl,
	.stop_tx	= vdec_stop_tx,
	.start_tx	= vdec_start_tx,
	.stop_rx	= vdec_stop_rx,
	.break_ctl	= vdec_break_ctl,
	.startup	= vdec_startup,
	.shutdown	= vdec_shutdown,
	.set_termios	= vdec_set_termios,
	.type		= vdec_type,
	.release_port	= vdec_release_port,
	.request_port	= vdec_request_port,
	.config_port	= vdec_config_port,
};

static int vdec_console_register(struct lg1314_vdec *v)
{
	int ret;

	v->uart_driver.owner = THIS_MODULE;
	v->uart_driver.driver_name = "lxipc";
	v->uart_driver.dev_name = "ttyLXIPC";
	v->uart_driver.nr = 1;

	ret = uart_register_driver(&v->uart_driver);
	if (ret)
		return ret;

	v->uart_port.dev = v->dev;
	v->uart_port.ops = &vdec_uart_ops;
	v->uart_port.type = PORT_GENERIC;	/* virtual; name comes from ->type() */
	v->uart_port.iotype = UPIO_MEM;
	v->uart_port.fifosize = 128;
	v->uart_port.line = 0;

	ret = uart_add_one_port(&v->uart_driver, &v->uart_port);
	if (ret)
		uart_unregister_driver(&v->uart_driver);
	return ret;
}

/* ---- probe / remove ---- */

/* A reserved-memory region of ours, by its memory-region-names entry. */
static int vdec_get_region(struct device *dev, const char *name,
			   struct resource *res)
{
	struct device_node *np;
	int idx, ret;

	idx = of_property_match_string(dev->of_node, "memory-region-names", name);
	if (idx < 0)
		return dev_err_probe(dev, idx, "missing %s memory region\n", name);
	np = of_parse_phandle(dev->of_node, "memory-region", idx);
	if (!np)
		return dev_err_probe(dev, -EINVAL, "bad %s memory region\n", name);
	ret = of_address_to_resource(np, 0, res);
	of_node_put(np);
	return ret;
}

static void vdec_release_rmem(void *dev)
{
	of_reserved_mem_device_release(dev);
}

static void vdec_destroy_wq(void *wq)
{
	destroy_workqueue(wq);
}

/*
 * gen_pool_destroy() BUGs on outstanding allocations, and the firmware may
 * still use (or a timed-out call may still own) a block: leak such a pool.
 */
static void vdec_pool_release(void *data)
{
	struct gen_pool *pool = data;

	if (gen_pool_avail(pool) != gen_pool_size(pool)) {
		pr_warn("lg1314-vdec: pool still in use, leaking it\n");
		return;
	}
	gen_pool_destroy(pool);
}

/* A pool over [virt, virt + size), which is phys if the pool hands out
 * physical addresses (virt == phys). */
static struct gen_pool *vdec_pool_create(struct device *dev, int order,
					 unsigned long virt, phys_addr_t phys,
					 size_t size)
{
	struct gen_pool *pool = gen_pool_create(order, NUMA_NO_NODE);
	int ret;

	if (!pool)
		return ERR_PTR(-ENOMEM);
	ret = gen_pool_add_virt(pool, virt, phys, size, NUMA_NO_NODE);
	if (ret) {
		gen_pool_destroy(pool);
		return ERR_PTR(ret);
	}
	ret = devm_add_action_or_reset(dev, vdec_pool_release, pool);
	return ret ? ERR_PTR(ret) : pool;
}

/*
 * The dpb pool's overflow: every memory-region named "shared" (no-map
 * memory, possibly in pieces), one allocator. Optional.
 */
static int vdec_shared_init(struct lg1314_vdec *v)
{
	struct device *dev = v->dev;
	struct device_node *np;
	struct resource *r;
	int n, i, ret;

	n = of_property_count_strings(dev->of_node, "memory-region-names");
	for (i = 0; i < n; i++) {
		const char *name;

		if (of_property_read_string_index(dev->of_node, "memory-region-names",
						  i, &name) || strcmp(name, "shared"))
			continue;
		if (v->n_shared == VDEC_SHARED_MAX)
			return dev_err_probe(dev, -EINVAL, "too many shared regions\n");
		r = &v->shared_mem[v->n_shared];
		np = of_parse_phandle(dev->of_node, "memory-region", i);
		if (!np)
			return dev_err_probe(dev, -EINVAL, "bad shared memory region\n");
		ret = of_address_to_resource(np, 0, r);
		of_node_put(np);
		if (ret)
			return ret;
		if (r->start < VDEC_DATA0_PHYS || r->end >= VDEC_DATA_END)
			return dev_err_probe(dev, -EINVAL, "shared region %pR outside the coprocessor's data windows\n", r);
		if (!v->hma_shared) {
			v->hma_shared = vdec_pool_create(dev, VDEC_HMA_ORDER, r->start,
							 r->start, resource_size(r));
			if (IS_ERR(v->hma_shared))
				return PTR_ERR(v->hma_shared);
		} else {
			ret = gen_pool_add_virt(v->hma_shared, r->start, r->start,
						resource_size(r), NUMA_NO_NODE);
			if (ret)
				return ret;
		}
		v->n_shared++;
	}
	if (v->hma_shared)
		dev_info(dev, "dpb overflow: %zu MiB in %u regions\n",
			 gen_pool_size(v->hma_shared) >> 20, v->n_shared);
	return 0;
}

/* Give back every buffer the firmware still holds (module unload). */
static void vdec_hma_release_all(struct lg1314_vdec *v)
{
	struct vdec_hma_alloc *a, *tmp;

	mutex_lock(&v->hma_lock);
	list_for_each_entry_safe(a, tmp, &v->hma_allocs, list) {
		list_del(&a->list);
		vdec_hma_free_one(v, a);
	}
	mutex_unlock(&v->hma_lock);
}

static int lg1314_vdec_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct lg1314_vdec *v;
	int ret, idx;

	v = devm_kzalloc(dev, sizeof(*v), GFP_KERNEL);
	if (!v)
		return -ENOMEM;
	v->dev = dev;
	spin_lock_init(&v->send_lock);
	spin_lock_init(&v->dovi_lock);
	spin_lock_init(&v->wait_lock);
	INIT_LIST_HEAD(&v->waiters);
	INIT_LIST_HEAD(&v->hma_allocs);
	mutex_init(&v->hma_lock);
	init_rwsem(&v->pictures_lock);

	v->rc = devm_ioremap(dev, VDEC_MCU_RC_PHYS, VDEC_MCU_RC_SIZE);
	v->ipc = devm_ioremap(dev, VDEC_MCU_IPC_PHYS, VDEC_MCU_IPC_SIZE);
	if (!v->rc || !v->ipc)
		return -ENOMEM;

	ret = vdec_get_region(dev, "mcu", &v->mcu_mem);
	if (!ret)
		ret = vdec_get_region(dev, "dpb", &v->dpb_mem);
	if (ret)
		return ret;
	if (resource_size(&v->mcu_mem) < VDEC_CPB_POOL_SIZE + VDEC_IPC_AREA_SIZE +
					 VDEC_ROM_SIZE + SZ_64K)
		return dev_err_probe(dev, -ENOMEM, "mcu region too small\n");
	if (v->mcu_mem.start < VDEC_DATA0_PHYS || v->mcu_mem.end >= VDEC_DATA_END ||
	    v->dpb_mem.start < VDEC_DATA0_PHYS || v->dpb_mem.end >= VDEC_DATA_END)
		return dev_err_probe(dev, -EINVAL,
				     "memory outside the coprocessor's data windows\n");
	v->ipc_phys = v->mcu_mem.start + VDEC_CPB_POOL_SIZE;
	v->rom_phys = v->ipc_phys + VDEC_IPC_AREA_SIZE;

	v->rom = devm_ioremap_wc(dev, v->rom_phys, VDEC_ROM_SIZE);
	v->queue = devm_memremap(dev, v->ipc_phys + VDEC_QUEUE_OFF, VDEC_QUEUE_SIZE, MEMREMAP_WC);
	v->arg = devm_memremap(dev, v->ipc_phys + VDEC_ARG_OFF, VDEC_ARG_SIZE, MEMREMAP_WC);
	if (!v->rom || IS_ERR_OR_NULL(v->queue) || IS_ERR_OR_NULL(v->arg))
		return -ENOMEM;

	v->param_pool = vdec_pool_create(dev, 6,
					 (unsigned long)v->arg + VDEC_PARAM_POOL_OFF,
					 v->ipc_phys + VDEC_ARG_OFF + VDEC_PARAM_POOL_OFF,
					 VDEC_PARAM_POOL_SIZE);
	if (IS_ERR(v->param_pool))
		return PTR_ERR(v->param_pool);

	/* Firmware buffer pools (physical addresses). */
	v->hma_cpb = vdec_pool_create(dev, VDEC_HMA_ORDER, v->mcu_mem.start,
				      v->mcu_mem.start, VDEC_CPB_POOL_SIZE);
	if (IS_ERR(v->hma_cpb))
		return PTR_ERR(v->hma_cpb);
	v->hma_vdec = vdec_pool_create(dev, VDEC_HMA_ORDER,
				       v->rom_phys + VDEC_ROM_SIZE,
				       v->rom_phys + VDEC_ROM_SIZE,
				       v->mcu_mem.end + 1 - (v->rom_phys + VDEC_ROM_SIZE));
	if (IS_ERR(v->hma_vdec))
		return PTR_ERR(v->hma_vdec);
	/* The dpb region is a CMA area: firmware frame buffers come from it
	 * through the DMA API (and later the V4L2 buffers). */
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	idx = of_property_match_string(dev->of_node, "memory-region-names", "dpb");
	ret = of_reserved_mem_device_init_by_idx(dev, dev->of_node, idx);
	if (ret)
		return dev_err_probe(dev, ret, "cannot use the dpb region\n");
	ret = devm_add_action_or_reset(dev, vdec_release_rmem, dev);
	if (ret)
		return ret;
	ret = vdec_shared_init(v);
	if (ret)
		return ret;

	/* Before the IRQ: devres then frees the IRQ first, and destroying the
	 * workqueue drains any firmware call still running. */
	v->wq = alloc_workqueue("lg1314-vdec", WQ_HIGHPRI, 0);
	if (!v->wq)
		return -ENOMEM;
	ret = devm_add_action_or_reset(dev, vdec_destroy_wq, v->wq);
	if (ret)
		return ret;

	spin_lock_init(&v->fwlog_lock);
	v->fwlog = devm_kzalloc(dev, VDEC_FW_LOG_KEEP, GFP_KERNEL);
	INIT_DELAYED_WORK(&v->fwlog_work, vdec_fwlog_work);

	v->irq = platform_get_irq(pdev, 0);
	if (v->irq < 0)
		return v->irq;
	ret = devm_request_irq(dev, v->irq, vdec_irq, 0, "lg1314-vdec", v);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, v);

	ret = vdec_load_firmware(v);
	if (ret)
		return ret;
	ret = vdec_boot(v);
	if (ret)
		return ret;
	schedule_delayed_work(&v->fwlog_work, 0);
	v->fwlog_file = debugfs_create_file("lg1314-vdec-fwlog", 0400, NULL, v,
					    &vdec_fwlog_fops);

	ret = vdec_console_register(v);
	if (ret)
		goto err_fwlog;

	v->pt = lg1314_vdec_pt_init(v);
	if (IS_ERR(v->pt)) {
		ret = PTR_ERR(v->pt);
		goto err_console;
	}
	v->dec = lg1314_vdec_dec_init(v);
	if (IS_ERR(v->dec)) {
		ret = PTR_ERR(v->dec);
		v->dec = NULL;
		lg1314_vdec_pt_exit(v->pt);
		goto err_console;
	}

	vdec_instance = v;	/* enable the sysfs test triggers */
	WRITE_ONCE(v->ready, true);
	return 0;

err_console:
	uart_remove_one_port(&v->uart_driver, &v->uart_port);
	uart_unregister_driver(&v->uart_driver);
err_fwlog:
	debugfs_remove(v->fwlog_file);
	cancel_delayed_work_sync(&v->fwlog_work);
	return ret;
}

static void lg1314_vdec_remove(struct platform_device *pdev)
{
	struct lg1314_vdec *v = platform_get_drvdata(pdev);

	vdec_instance = NULL;
	WRITE_ONCE(v->ready, false);
	lg1314_vdec_dec_exit(v->dec);
	lg1314_vdec_pt_exit(v->pt);
	/* Stop taking messages and finish firmware calls in flight, then return
	 * the firmware's buffers so the pools can go. */
	writel(0, v->rc + RC_INTR_EN);
	disable_irq(v->irq);
	flush_workqueue(v->wq);
	vdec_hma_release_all(v);

	uart_remove_one_port(&v->uart_driver, &v->uart_port);
	uart_unregister_driver(&v->uart_driver);
	debugfs_remove(v->fwlog_file);
	cancel_delayed_work_sync(&v->fwlog_work);
}

static const struct of_device_id lg1314_vdec_of_match[] = {
	{ .compatible = "lge,lg1314-vdec" },
	{ }
};
MODULE_DEVICE_TABLE(of, lg1314_vdec_of_match);

static struct platform_driver lg1314_vdec_driver = {
	.probe = lg1314_vdec_probe,
	.remove = lg1314_vdec_remove,
	.driver = {
		.name = "lg1314-vdec",
		.of_match_table = lg1314_vdec_of_match,
	},
};
module_platform_driver(lg1314_vdec_driver);

MODULE_DESCRIPTION("LG1314 video-decode coprocessor");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("DMA_BUF");
