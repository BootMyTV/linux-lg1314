// SPDX-License-Identifier: GPL-2.0
/*
 * LG1314 audio DSPs: IPC rings and the IMC message layer (host side).
 *
 * Each direction between the host and a DSP is a ring in the shared IPC
 * memory, with read/write offsets in the audio block. A packet is a sync
 * word, an adsp_ipc_hdr and the parameters (padded to 4 bytes); a writer that
 * would run past the end leaves ADSP_IPC_END and continues at offset 0. The
 * debug rings carry the DSPs' log lines in the same packet format.
 *
 * On top, the IMC layer (LG's framework) addresses modules: commands (with an
 * optional response), event registrations and the notifications they produce.
 */

#include <linux/device.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#include "lg1314-adsp.h"

#define AUD_DIRQ(n)		(0x740 + 4 * (n))	/* doorbell to DSP n */

static const struct {
	const char *name;
	u32 off;		/* within the IPC area */
	u32 size;
	u32 sync;
	u32 ptr;		/* rptr register; wptr follows */
} adsp_rings[ADSP_RING_NUM] = {
	[ADSP_RING_A2D]  = { "a2d",  0x00000, 0x18400, ADSP_IPC_SYNC,     0x1080 },
	[ADSP_RING_D2A]  = { "d2a",  0x18400, 0x18400, ADSP_IPC_SYNC,     0x1088 },
	[ADSP_RING_A2P]  = { "a2p",  0x30800, 0x18400, ADSP_IPC_SYNC,     0x1070 },
	[ADSP_RING_P2A]  = { "p2a",  0x48c00, 0x18400, ADSP_IPC_SYNC,     0x1078 },
	[ADSP_RING_D2P]  = { "d2p",  0x61000, 0x18400, ADSP_IPC_SYNC,     0x1090 },
	[ADSP_RING_P2D]  = { "p2d",  0x79400, 0x18400, ADSP_IPC_SYNC,     0x1098 },
	[ADSP_RING_DBGD] = { "dbgd", 0x91800, 0x30800, ADSP_IPC_DBG_SYNC, 0x10a8 },
	[ADSP_RING_DBGP] = { "dbgp", 0xc2000, 0x30800, ADSP_IPC_DBG_SYNC, 0x10a0 },
};

/* Set up the ring descriptors; "ipc" points at the IPC area in adsp->mem. */
void adsp_ipc_init(struct lg1314_adsp *adsp)
{
	void *ipc = adsp->mem + 0x2b00000;
	int i;

	for (i = 0; i < ADSP_RING_NUM; i++) {
		struct adsp_ring *r = &adsp->ring[i];

		r->name = adsp_rings[i].name;
		r->mem = ipc + adsp_rings[i].off;
		r->size = adsp_rings[i].size;
		r->sync = adsp_rings[i].sync;
		r->rptr = adsp_rings[i].ptr;
		r->wptr = adsp_rings[i].ptr + 4;
	}
}

static u32 ring_word(struct adsp_ring *r, u32 off)
{
	return READ_ONCE(*(u32 *)(r->mem + off));
}

static void ring_put(struct adsp_ring *r, u32 off, u32 val)
{
	WRITE_ONCE(*(u32 *)(r->mem + off), val);
}

static int ring_write(struct lg1314_adsp *adsp, struct adsp_ring *r,
		      const struct adsp_ipc_hdr *hdr, const void *param)
{
	u32 len = 4 + sizeof(*hdr) + ALIGN(hdr->param_len, 4);
	u32 wp = adsp_rd(adsp, r->wptr), rp = adsp_rd(adsp, r->rptr);
	u32 free = wp < rp ? rp - wp : r->size - wp + rp;
	const u32 *src;
	u32 i;

	if (len > ADSP_IPC_MAX_PKT || wp >= r->size || (wp & 3))
		return -EINVAL;
	if (len >= free)
		return -ENOSPC;
	if (wp + len >= r->size) {
		if (rp <= len)
			return -ENOSPC;
		ring_put(r, wp, ADSP_IPC_END);
		wp = 0;
	}

	ring_put(r, wp, r->sync);
	src = (const u32 *)hdr;
	for (i = 0; i < sizeof(*hdr) / 4; i++)
		ring_put(r, wp + 4 + 4 * i, src[i]);
	if (hdr->param_len) {
		u8 tmp[ADSP_IPC_MAX_PARAM + 4] __aligned(4) = {};

		memcpy(tmp, param, hdr->param_len);
		src = (const u32 *)tmp;
		for (i = 0; i < ALIGN(hdr->param_len, 4) / 4; i++)
			ring_put(r, wp + 4 + sizeof(*hdr) + 4 * i, src[i]);
	}
	wmb();		/* packet before the write offset */
	adsp_wr(adsp, r->wptr, wp + len);
	return 0;
}

static void doorbell(struct lg1314_adsp *adsp, int dsp)
{
	adsp_wr(adsp, AUD_DIRQ(dsp), 1);
	adsp_wr(adsp, AUD_DIRQ(dsp), 0);
}

static int adsp_send(struct lg1314_adsp *adsp, int dsp,
		     const struct adsp_ipc_hdr *hdr, const void *param)
{
	struct adsp_ring *r = &adsp->ring[dsp ? ADSP_RING_A2P : ADSP_RING_A2D];
	int ret;

	mutex_lock(&adsp->tx_lock);
	ret = ring_write(adsp, r, hdr, param);
	if (!ret)
		doorbell(adsp, dsp);
	mutex_unlock(&adsp->tx_lock);
	if (ret)
		dev_err(adsp->dev, "%s: send cmd %08x failed: %d\n", r->name, hdr->cmd, ret);
	return ret;
}

/*
 * Read one packet. Returns its length (header + parameters), 0 if the ring is
 * empty, or a negative error. *hdr and *param point into "buf".
 */
static int ring_read(struct lg1314_adsp *adsp, struct adsp_ring *r, u8 *buf,
		     u32 buf_len)
{
	u32 wp = adsp_rd(adsp, r->wptr), rp = adsp_rd(adsp, r->rptr);
	struct adsp_ipc_hdr *hdr = (void *)buf;
	u32 i, len;

	if (wp >= r->size || rp >= r->size || (wp | rp) & 3) {
		dev_err(adsp->dev, "%s: bad offsets r %x w %x\n", r->name, rp, wp);
		adsp_wr(adsp, r->rptr, wp);
		return -EIO;
	}
	if (rp == wp)
		return 0;

	if (ring_word(r, rp) == ADSP_IPC_END)
		rp = 0;
	while (rp != wp && ring_word(r, rp) != r->sync) {
		rp += 4;
		if (rp >= r->size)
			rp = 0;
	}
	if (rp == wp) {
		dev_warn(adsp->dev, "%s: lost sync\n", r->name);
		adsp_wr(adsp, r->rptr, wp);
		return -EIO;
	}
	rp += 4;
	rmb();
	for (i = 0; i < sizeof(*hdr) / 4; i++)
		((u32 *)buf)[i] = ring_word(r, rp + 4 * i);
	len = sizeof(*hdr) + ALIGN(hdr->param_len, 4);
	if (hdr->param_len > ADSP_IPC_MAX_PKT || len > buf_len ||
	    rp + len > r->size) {
		dev_err(adsp->dev, "%s: bad packet (cmd %08x len %u)\n", r->name,
			hdr->cmd, hdr->param_len);
		adsp_wr(adsp, r->rptr, wp);
		return -EIO;
	}
	for (; i < len / 4; i++)
		((u32 *)buf)[i] = ring_word(r, rp + 4 * i);
	adsp_wr(adsp, r->rptr, rp + len);
	return len;
}

static struct adsp_waiter *find_waiter(struct lg1314_adsp *adsp, u32 action, u32 id)
{
	struct adsp_waiter *w;

	list_for_each_entry(w, &adsp->waiters, list)
		if (w->action == action && w->id == id)
			return w;
	return NULL;
}

static void complete_waiter(struct lg1314_adsp *adsp, u32 action, u32 id,
			    s32 result, const void *param, u32 len)
{
	struct adsp_waiter *w;
	unsigned long flags;

	spin_lock_irqsave(&adsp->wait_lock, flags);
	w = find_waiter(adsp, action, id);
	if (w) {
		w->result = result;
		w->len = min(len, w->buf_len);
		if (w->buf && w->len)
			memcpy(w->buf, param, w->len);
		list_del_init(&w->list);
		complete(&w->done);
	}
	spin_unlock_irqrestore(&adsp->wait_lock, flags);
	if (!w)
		dev_dbg(adsp->dev, "unexpected %s id %x\n",
			action == ADSP_IMC_RESPONSE ? "response" : "notification", id);
}

static void add_impl(struct lg1314_adsp *adsp, int dsp, const char *name, u32 rank)
{
	struct adsp_impl *impl = kzalloc(sizeof(*impl), GFP_KERNEL);

	if (!impl)
		return;
	strscpy(impl->name, name, sizeof(impl->name));
	impl->rank = rank;
	impl->next_index = 1;
	mutex_lock(&adsp->impl_lock);
	impl->num = ADSP_IMPL_FIRST + list_count_nodes(&adsp->impls[dsp]);
	list_add_tail(&impl->list, &adsp->impls[dsp]);
	mutex_unlock(&adsp->impl_lock);
}

/* Commands addressed to the host's manager (MAN_ARM). */
static void handle_command(struct lg1314_adsp *adsp, int dsp,
			   const struct adsp_ipc_hdr *hdr, const u8 *param)
{
	s32 result = ADSP_ERR_NOT_IMPLEMENTED;

	if (hdr->target == ADSP_MOD_MAN_ARM && hdr->cmd == ADSP_MGR_ADD_PROCESSOR) {
		struct adsp_impl *impl, *tmp;

		mutex_lock(&adsp->impl_lock);
		list_for_each_entry_safe(impl, tmp, &adsp->impls[dsp], list) {
			list_del(&impl->list);
			kfree(impl);
		}
		strscpy(adsp->proc_name[dsp], param, sizeof(adsp->proc_name[dsp]));
		mutex_unlock(&adsp->impl_lock);
		dev_dbg(adsp->dev, "dsp%d: processor \"%s\"\n", dsp, adsp->proc_name[dsp]);
	} else if (hdr->target == ADSP_MOD_MAN_ARM && hdr->cmd == ADSP_MGR_ADD_MODULE_IMPL) {
		char name[32];

		strscpy(name, param, sizeof(name));
		add_impl(adsp, dsp, name, get_unaligned_le32(param + 32));
	} else if (ADSP_UID_PROC(hdr->target) == ADSP_CORE_ARM &&
		   hdr->cmd == ADSP_BM_PROCESS_EVT_REMOTE) {
		adsp_shbuf_event(adsp, hdr->target, param, hdr->param_len);
		result = 0;
	} else if (ADSP_UID_PROC(hdr->target) == ADSP_CORE_ARM &&
		   (hdr->cmd & 0xffff0000) == ADSP_MOD_ASYNC_EVENT) {
		dev_dbg(adsp->dev, "dsp%d: event for %08x from %08x\n", dsp, hdr->target,
			hdr->sender);
		result = 0;
	} else {
		dev_info(adsp->dev, "dsp%d: command %08x for module %x from %x (%u bytes)\n",
			 dsp, hdr->cmd, hdr->target, hdr->sender, hdr->param_len);
	}

	if (hdr->option & ADSP_IMC_OPT_RSP) {
		struct adsp_ipc_hdr rsp = {
			.cmd = hdr->cmd,
			.sender_id = hdr->sender_id,
			.sender = ADSP_MOD_MAN_ARM,
			.action = ADSP_IMC_RESPONSE,
			.option = result,
			.target = hdr->sender,
		};

		adsp_send(adsp, dsp, &rsp, NULL);
	}
}

static void drain_imc(struct lg1314_adsp *adsp, int dsp)
{
	struct adsp_ring *r = &adsp->ring[dsp ? ADSP_RING_P2A : ADSP_RING_D2A];
	u8 buf[ADSP_IPC_MAX_PKT] __aligned(4);
	int len;

	while ((len = ring_read(adsp, r, buf, sizeof(buf))) > 0) {
		struct adsp_ipc_hdr *hdr = (void *)buf;
		u8 *param = buf + sizeof(*hdr);

		switch (hdr->action) {
		case ADSP_IMC_COMMAND:
			handle_command(adsp, dsp, hdr, param);
			break;
		case ADSP_IMC_RESPONSE:
			complete_waiter(adsp, ADSP_IMC_RESPONSE, hdr->sender_id,
					(s32)hdr->option, param, hdr->param_len);
			break;
		case ADSP_IMC_NOTIFICATION:
			complete_waiter(adsp, ADSP_IMC_NOTIFICATION, hdr->sender_id, 0,
					param, hdr->param_len);
			break;
		default:
			dev_info(adsp->dev, "dsp%d: IMC action %u cmd %08x (ignored)\n",
				 dsp, hdr->action, hdr->cmd);
			break;
		}
	}
}

static void drain_debug(struct lg1314_adsp *adsp, int dsp)
{
	struct adsp_ring *r = &adsp->ring[dsp ? ADSP_RING_DBGP : ADSP_RING_DBGD];
	u8 buf[ADSP_IPC_MAX_PKT] __aligned(4);
	int len;

	while ((len = ring_read(adsp, r, buf, sizeof(buf))) > 0) {
		struct adsp_ipc_hdr *hdr = (void *)buf;
		char *s = buf + sizeof(*hdr);

		s[min_t(u32, hdr->param_len, len - sizeof(*hdr) - 1)] = 0;
		strim(s);
		if (*s)
			dev_info(adsp->dev, "dsp%d: %s\n", dsp, s);
	}
}

void adsp_rx_work(struct work_struct *work)
{
	struct lg1314_adsp *adsp = container_of(work, struct lg1314_adsp, rx_work);
	int dsp;

	for (dsp = 0; dsp < ADSP_NUM_DSP; dsp++) {
		drain_debug(adsp, dsp);
		drain_imc(adsp, dsp);
	}
}

static struct adsp_waiter *waiter_new(struct lg1314_adsp *adsp, u32 action, u32 id,
				      void *buf, u32 buf_len)
{
	struct adsp_waiter *w = kzalloc(sizeof(*w), GFP_KERNEL);
	unsigned long flags;

	if (!w)
		return NULL;
	w->action = action;
	w->id = id;
	w->buf = buf;
	w->buf_len = buf_len;
	init_completion(&w->done);
	spin_lock_irqsave(&adsp->wait_lock, flags);
	list_add_tail(&w->list, &adsp->waiters);
	spin_unlock_irqrestore(&adsp->wait_lock, flags);
	return w;
}

void adsp_waiter_put(struct lg1314_adsp *adsp, struct adsp_waiter *w)
{
	unsigned long flags;

	if (!w)
		return;
	spin_lock_irqsave(&adsp->wait_lock, flags);
	list_del(&w->list);
	spin_unlock_irqrestore(&adsp->wait_lock, flags);
	kfree(w);
}

int adsp_wait(struct lg1314_adsp *adsp, struct adsp_waiter *w, unsigned int timeout_ms)
{
	if (!wait_for_completion_timeout(&w->done, msecs_to_jiffies(timeout_ms)))
		return -ETIMEDOUT;
	return w->result;
}

/*
 * Send a command to a module on DSP "dsp". With "rsp" (or rsp_len 0 but a
 * timeout) the DSP is asked to respond; the framework result (0 = ok) is
 * returned, the response parameters land in rsp.
 */
int adsp_imc_command(struct lg1314_adsp *adsp, int dsp, u32 target, u32 cmd,
		     const void *param, u32 len, void *rsp, u32 rsp_len,
		     unsigned int timeout_ms)
{
	struct adsp_ipc_hdr hdr = {
		.cmd = cmd,
		.sender = ADSP_MOD_MAN_ARM,
		.action = ADSP_IMC_COMMAND,
		.target = target,
		.param_len = len,
	};
	struct adsp_waiter *w = NULL;
	int ret;

	if (len > ADSP_IPC_MAX_PARAM)
		return -EINVAL;
	hdr.sender_id = atomic_inc_return(&adsp->msg_id) & 0xffffff;
	if (timeout_ms) {
		hdr.option = ADSP_IMC_OPT_RSP;
		w = waiter_new(adsp, ADSP_IMC_RESPONSE, hdr.sender_id, rsp, rsp_len);
		if (!w)
			return -ENOMEM;
	}
	ret = adsp_send(adsp, dsp, &hdr, param);
	if (!ret && w)
		ret = adsp_wait(adsp, w, timeout_ms);
	adsp_waiter_put(adsp, w);
	return ret;
}

/*
 * Ask DSP "dsp" to notify the host (once) when module "owner" raises "event".
 * Returns a waiter to adsp_wait() on, then adsp_waiter_put().
 */
struct adsp_waiter *adsp_imc_register_event(struct lg1314_adsp *adsp, int dsp,
					    u32 owner, u32 event, u32 level)
{
	u32 action_id = (ADSP_CORE_ARM << 20) |
			(atomic_inc_return(&adsp->action_idx) & 0xfffff);
	struct adsp_ipc_hdr hdr = {
		.sender_id = action_id,
		.sender = ADSP_MOD_MAN_ARM,
		.action = ADSP_IMC_REGISTER,
		.option = ADSP_IMC_OPT_CALLBACK,
		.target = level,
		.ext = ((event & 0xffffff) << 8) | (owner & 0xff),
	};
	struct adsp_waiter *w;

	w = waiter_new(adsp, ADSP_IMC_NOTIFICATION, action_id, NULL, 0);
	if (!w)
		return ERR_PTR(-ENOMEM);
	if (adsp_send(adsp, dsp, &hdr, NULL)) {
		adsp_waiter_put(adsp, w);
		return ERR_PTR(-EIO);
	}
	return w;
}
