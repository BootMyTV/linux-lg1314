/* SPDX-License-Identifier: GPL-2.0 */
/*
 * LG1314 audio DSPs: shared definitions.
 *
 * Wire formats (IPC packets, IMC actions, command encodings, module ids) follow
 * LG's audio framework (GPL kdriver core/audio/common), which runs on both DSPs
 * and, in stock webOS, as the "ARM" processor inside the kernel.
 */
#ifndef __LG1314_ADSP_H__
#define __LG1314_ADSP_H__

#include <linux/atomic.h>
#include <linux/completion.h>
#include <linux/genalloc.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#define ADSP_NUM_DSP		2

/*
 * Memory map within the reserved area (stock's "MS12 2.0" layout). Each DSP's
 * image goes in the top of its region.
 */
#define MEM_DSP_BASE(n)		((n) ? 0x01a00000 : 0x00000000)
#define MEM_DSP_SIZE(n)		((n) ? 0x00f00000 : 0x01a00000)
#define MEM_DSP_IMAGE(n)	((n) ? 0x00400000 : 0x00700000)
#define MEM_SE			0x02900000
#define MEM_SE_SIZE		0x00200000
#define MEM_IPC			0x02b00000
#define MEM_IPC_SIZE		0x00100000
#define MEM_POOL		0x02d60000	/* after the BCs: shared buffers */
#define MEM_END			0x03460000

/* IPC packet header (follows a 32-bit sync word in the ring). */
struct adsp_ipc_hdr {
	u32 cmd;
	u32 sender_id;		/* message id / action id / timestamp */
	u32 sender;		/* sending module */
	u32 action;		/* enum adsp_imc_action */
	u32 option;		/* COMMAND: bit0 response wanted; RESPONSE: result */
	u32 target;		/* target module (REGISTER: notification level) */
	u32 ext;		/* REGISTER: event id */
	u32 param_len;
} __packed;

#define ADSP_IPC_SYNC		0x01444542
#define ADSP_IPC_DBG_SYNC	0x19c0db90
#define ADSP_IPC_END		0x00444e45
#define ADSP_IPC_MAX_PKT	0x200
#define ADSP_IPC_MAX_PARAM	(ADSP_IPC_MAX_PKT - sizeof(struct adsp_ipc_hdr) - 4)

enum adsp_imc_action {
	ADSP_IMC_DEBUG,
	ADSP_IMC_COMMAND,
	ADSP_IMC_RESPONSE,
	ADSP_IMC_REGISTER,
	ADSP_IMC_CANCEL,
	ADSP_IMC_NOTIFICATION,
};

/* IMC option flags for REGISTER */
#define ADSP_IMC_OPT_RSP	BIT(0)
#define ADSP_IMC_OPT_CALLBACK	BIT(1)	/* notify (vs. execute a command) */
#define ADSP_IMC_OPT_REPEAT	BIT(2)

/* Command encoding: id in the top half, parameter size in the bottom. */
#define ADSP_CMD(id, size)	((((u32)(id) & 0xffff) << 16) | ((size) & 0xffff))
#define ADSP_CMD_MOD(mod, n)	(((mod) << 8) | (n))
#define ADSP_CMD_MANAGER	0x80
#define ADSP_MGR_ADD_PROCESSOR	ADSP_CMD(ADSP_CMD_MOD(ADSP_CMD_MANAGER, 0x01), 32)
#define ADSP_MGR_ADD_MODULE_IMPL ADSP_CMD(ADSP_CMD_MOD(ADSP_CMD_MANAGER, 0x02), 36)

#define ADSP_REGISTRY_CREATE	ADSP_CMD(ADSP_CMD_MOD(ADSP_CMD_MANAGER, 0xf0), 36)
#define ADSP_REGISTRY_DESTROY	ADSP_CMD(ADSP_CMD_MOD(ADSP_CMD_MANAGER, 0xf1), 4)
#define ADSP_CONNECTOR_CONNECT	ADSP_CMD(0xf100, 16)
#define ADSP_CONNECTOR_DISCONNECT ADSP_CMD(0xf101, 16)	/* {out uid, port, in uid, port} */
#define ADSP_CONNECTOR_DISCONNECT ADSP_CMD(0xf101, 16)

/* Generic module commands */
#define ADSP_MOD_INIT		ADSP_CMD(0x1000, 0)
#define ADSP_MOD_START		ADSP_CMD(0x1001, 0)
#define ADSP_MOD_STOP		ADSP_CMD(0x1002, 0)
#define ADSP_MOD_SET_PROP	ADSP_CMD(0x1018, 4)	/* {u32 len; char str[]} */
#define ADSP_MOD_ASYNC_EVENT	ADSP_CMD(0x1019, 0)	/* serialized event, any size */

#define ADSP_DBG_MESSAGE_LEVEL	ADSP_CMD(0x5fc6, 36)	/* {s32 level; char name[32]} */

/* Glue ("jumper" end) and buffer manager */
#define ADSP_GLUE_SET_SHARED_BUFFER ADSP_CMD(0xf100, 12)	/* {offset, size, cowork uid} */
#define ADSP_BM_PROCESS_EVT_REMOTE ADSP_CMD(0xf101, sizeof(struct adsp_bm_evt))
#define ADSP_BM_EVT_READ	2
#define ADSP_BM_EVT_WRITE	3

/* A shared buffer's read/write state, exchanged in BM events. */
struct adsp_buf_status {
	s32 index_allocated;	/* an AU is being written */
	s32 read_off;
	s32 write_off;
	s16 read_idx;
	s16 write_idx;
	s32 written;		/* bytes in the AU being written */
	s32 passed;
	s32 repeat_count;
} __packed;

struct adsp_bm_evt {
	u32 evt_type;		/* ADSP_BM_EVT_* */
	u32 buf_type;
	u32 base_addr;		/* buffer offset (DSP view) */
	struct adsp_buf_status status;
	u32 update_mask;
} __packed;

/* Events */
#define ADSP_EVT_DSP_DN_DONE	0x0200	/* DSP firmware download done */

/* Static module ids, and the processor-relative uid form. */
#define ADSP_MOD_MAN_ARM	1
#define ADSP_MOD_MAN_DSP(n)	(2 + (n))
#define ADSP_MOD_DBG_DSP(n)	(4 + (n))
#define ADSP_CORE_ARM		1
#define ADSP_CORE_DSP(n)	(2 + (n))
#define ADSP_UID_PROC(uid)	(((uid) >> 24) & 0xff)
#define ADSP_UID(proc, impl, idx) (((proc) << 24) | ((impl) << 16) | (idx))
#define ADSP_UID_REGISTRY(proc)	ADSP_UID(proc, 0, 1)
#define ADSP_UID_CONNECTOR(proc) ADSP_UID(proc, 0, 2)
#define ADSP_IMPL_FIRST		0x10	/* below: reserved (registry, connector, ...) */
#define ADSP_IMPL_ARM_SHBUF	0x10	/* our ARM-side buffer endpoints */

#define ADSP_ERR_NOT_IMPLEMENTED 17	/* framework ADEC_ERR_NOT_IMPLEMENTED */

/* One direction of an IPC channel. */
struct adsp_ring {
	const char *name;
	void *mem;		/* ring memory (write-combined) */
	u32 size;
	u32 sync;
	u32 rptr;		/* register offsets (audio block) */
	u32 wptr;
};

enum adsp_ring_id {
	ADSP_RING_A2D, ADSP_RING_D2A,	/* host <-> DSP0 */
	ADSP_RING_A2P, ADSP_RING_P2A,	/* host <-> DSP1 */
	ADSP_RING_D2P, ADSP_RING_P2D,	/* DSP0 <-> DSP1 (not ours) */
	ADSP_RING_DBGD, ADSP_RING_DBGP,	/* DSP0 / DSP1 debug output */
	ADSP_RING_NUM,
};

/* Something waiting on a response or a notification. */
struct adsp_waiter {
	struct list_head list;
	u32 action;		/* ADSP_IMC_RESPONSE or ADSP_IMC_NOTIFICATION */
	u32 id;
	struct completion done;
	s32 result;
	void *buf;
	u32 buf_len;
	u32 len;
};

/* A module implementation a DSP announced (ADD_MODULE_IMPL). */
struct adsp_impl {
	struct list_head list;
	char name[32];
	u32 rank;
	u32 num;		/* uid implementation byte */
	u32 next_index;
};

struct lg1314_adsp {
	struct device *dev;
	void __iomem *aud;	/* 0xc9a00000 audio block */
	void __iomem *syn;	/* 0xc9c60c00 AUD_SYN (memory flush) */
	void __iomem *mpro;	/* 0xc9822600 audio bus port protector */
	void __iomem *bmc;	/* 0xc930e000 BMC_SYN audio clocks (+ CTOP_BMC) */
	void *mem;
	phys_addr_t mem_phys;
	size_t mem_size;
	int irq[ADSP_NUM_DSP];
	atomic_t irq_count[ADSP_NUM_DSP];

	struct adsp_ring ring[ADSP_RING_NUM];
	struct mutex tx_lock;		/* host -> DSP rings */
	spinlock_t wait_lock;
	struct list_head waiters;
	atomic_t msg_id;
	atomic_t action_idx;

	struct work_struct rx_work;
	struct delayed_work poll_work;	/* backstop for missed interrupts */
	bool running;

	struct mutex impl_lock;
	char proc_name[ADSP_NUM_DSP][32];
	struct list_head impls[ADSP_NUM_DSP];

	struct gen_pool *pool;		/* shared buffers (DSP-visible) */
	struct mutex shbuf_lock;
	struct list_head shbufs;
	u32 shbuf_next;

	struct adsp_pcm *pcm;		/* PCM outputs (lg1314-adsp-pcm.c) */
	struct adsp_hdmi *hdmi;		/* HDMI input audio (lg1314-adsp-hdmi.c) */

	struct dentry *debugfs;
	struct adsp_shbuf *dbg_sb;
	u32 dbg_chunk;
	char ctl_result[128];
};

/*
 * ARM end of a "jumper": a CommonBuffer in shared memory that we write and a
 * DSP glue module reads. The read status comes back in BM_EVT_READ messages.
 */
struct adsp_shbuf {
	struct list_head list;
	struct lg1314_adsp *adsp;
	int dsp;
	u32 uid;		/* ours: the reader's cowork module */
	u32 reader;		/* reader glue uid */
	void *mem;		/* header, AU table, data */
	u32 size;
	phys_addr_t phys;
	u32 offset;		/* DSP-visible address */
	void *data;
	u32 data_size;
	u32 max_au;
	u32 au_max_len;
	spinlock_t lock;
	struct adsp_buf_status st;
	wait_queue_head_t wq;
	void (*on_read)(void *priv);	/* space freed (may sleep: workqueue) */
	void *priv;
};

/* lg1314-adsp-core.c */
int adsp_module_create(struct lg1314_adsp *adsp, int dsp, const char *impl, u32 *uid);
int adsp_module_destroy(struct lg1314_adsp *adsp, u32 uid);
int adsp_module_connect(struct lg1314_adsp *adsp, u32 out_uid, u32 out_port,
			u32 in_uid, u32 in_port);
int adsp_module_disconnect(struct lg1314_adsp *adsp, u32 out_uid, u32 out_port,
			   u32 in_uid, u32 in_port);
int adsp_module_cmd(struct lg1314_adsp *adsp, u32 uid, u32 cmd, const void *param,
		    u32 len, void *rsp, u32 rsp_len);
int adsp_module_set_prop(struct lg1314_adsp *adsp, u32 uid, const char *prop);

/* lg1314-adsp-buf.c */
int adsp_pool_init(struct lg1314_adsp *adsp, u32 off, u32 size);
struct adsp_shbuf *adsp_shbuf_create(struct lg1314_adsp *adsp, u32 reader, u32 size,
				     u32 au_max_len, u32 max_au);
struct adsp_shbuf *adsp_shbuf_alloc(struct lg1314_adsp *adsp, int dsp, u32 reader,
				    u32 size, u32 au_max_len, u32 max_au);
struct adsp_shbuf *adsp_shbuf_create_on(struct lg1314_adsp *adsp, int dsp, u32 reader,
					u32 size, u32 au_max_len, u32 max_au);
void adsp_shbuf_destroy(struct adsp_shbuf *sb);
u32 adsp_shbuf_free_space(struct adsp_shbuf *sb);
int adsp_shbuf_write_au(struct adsp_shbuf *sb, const void *data, u32 len);
void adsp_shbuf_event(struct lg1314_adsp *adsp, u32 uid, const void *param, u32 len);

/* lg1314-adsp-se.c */
struct adsp_se;
struct snd_soc_component;
int adsp_se_build(struct lg1314_adsp *adsp, struct adsp_se **se, u32 *reader);
int adsp_se_attach(struct adsp_se *se, struct adsp_shbuf *sb);
int adsp_se_start(struct adsp_se *se);
void adsp_se_free(struct adsp_se *se);
int adsp_se_add_controls(struct snd_soc_component *c, struct adsp_se *se);
int adsp_se_hdmi(struct adsp_se *se, int port, int branch, u32 rate);
u32 adsp_se_hdmi_out(struct adsp_se *se);
int adsp_se_hdmi_attach(struct adsp_se *se);

/* lg1314-adsp-pcm.c */
int adsp_pcm_init(struct lg1314_adsp *adsp);
void adsp_pcm_exit(struct lg1314_adsp *adsp);
int adsp_pcm_hdmi(struct lg1314_adsp *adsp, int port, u32 rate);

/* lg1314-adsp-hdmi.c */
void adsp_hdmi_init(struct lg1314_adsp *adsp);
void adsp_hdmi_exit(struct lg1314_adsp *adsp);
struct platform_driver;
extern struct platform_driver adsp_out_driver;

/* lg1314-adsp-ipc.c */
void adsp_ipc_init(struct lg1314_adsp *adsp);
void adsp_rx_work(struct work_struct *work);
int adsp_imc_command(struct lg1314_adsp *adsp, int dsp, u32 target, u32 cmd,
		     const void *param, u32 len, void *rsp, u32 rsp_len,
		     unsigned int timeout_ms);
struct adsp_waiter *adsp_imc_register_event(struct lg1314_adsp *adsp, int dsp,
					    u32 owner, u32 event, u32 level);
int adsp_wait(struct lg1314_adsp *adsp, struct adsp_waiter *w,
	      unsigned int timeout_ms);
void adsp_waiter_put(struct lg1314_adsp *adsp, struct adsp_waiter *w);

static inline u32 adsp_rd(struct lg1314_adsp *adsp, u32 off)
{
	return readl(adsp->aud + off);
}

/* The audio block wants full 32-bit writes; byte writes corrupt e.g. SWRESET. */
static inline void adsp_wr(struct lg1314_adsp *adsp, u32 off, u32 val)
{
	writel(val, adsp->aud + off);
}

#endif
