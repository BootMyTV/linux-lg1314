// SPDX-License-Identifier: GPL-2.0
/*
 * LG1314 audio DSPs: buffers shared with the DSPs.
 *
 * A stream crosses into a DSP through a "jumper": a CommonBuffer in shared
 * memory, written here and read by a glue module on the DSP (which pushes
 * each access unit (AU) into the module connected to its output). Both sides
 * keep their own copy of the read/write status and exchange it in
 * BM_PROCESS_EVT_REMOTE messages: BM_EVT_WRITE from us after each AU, and
 * BM_EVT_READ from the glue (to our "cowork" uid) as it consumes them.
 *
 * Layout (05.50 firmware):
 *   header {max_size, au_max_len, max_index, repeat state[7]}
 *   AU table at 0x28, 0x30 bytes per AU (one spare)
 *   au_max_len bytes of slack (the reader linearizes a wrapped AU there)
 *   max_size bytes of ring data
 */

#include <linux/device.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/string.h>

#include "lg1314-adsp.h"

#define CBUF_AU_TABLE		0x28
#define CBUF_AU_SIZE		0x30
#define CBUF_STRUCT_SIZE(n)	(CBUF_AU_TABLE + CBUF_AU_SIZE * ((n) + 1))
#define CBUF_GUARD		4	/* the writer never fills the ring completely */

struct cbuf_hdr {
	u32 max_size;
	u32 au_max_len;
	u32 max_index;
	u32 repeat[7];
} __packed;

/* ADEC_AU_INFO_T (40 bytes in 05.50) + data offset */
struct cbuf_au {
	u64 size;
	u32 timestamp;
	u32 gstc;
	s16 index;
	u8 flags;
	s8 discontinuity;
	u32 last_au;
	u32 first_module_gstc;
	u32 data_type;		/* 0: PCM */
	u32 rsvd[2];		/* added after the 2019 source; zero */
	u32 buf_offset;
	u32 pad;
} __packed;

static_assert(sizeof(struct cbuf_hdr) == CBUF_AU_TABLE);
static_assert(sizeof(struct cbuf_au) == CBUF_AU_SIZE);

/* DSP view of shared memory: 0x10000000 + offset from the SE area. */
#define DSP_SHMEM_BASE		0x10000000

int adsp_pool_init(struct lg1314_adsp *adsp, u32 off, u32 size)
{
	adsp->pool = devm_gen_pool_create(adsp->dev, 12, -1, "adsp-shbuf");
	if (IS_ERR(adsp->pool))
		return PTR_ERR(adsp->pool);
	return gen_pool_add_virt(adsp->pool, (unsigned long)adsp->mem + off,
				 adsp->mem_phys + off, size, -1);
}

static struct cbuf_au *shbuf_au(struct adsp_shbuf *sb, int idx)
{
	return sb->mem + CBUF_AU_TABLE + idx * CBUF_AU_SIZE;
}

static u32 shbuf_free_locked(struct adsp_shbuf *sb)
{
	const struct adsp_buf_status *st = &sb->st;

	if ((st->write_idx + 1) % sb->max_au == st->read_idx)
		return 0;
	if (st->write_off < st->read_off)
		return st->read_off - st->write_off - CBUF_GUARD;
	return sb->data_size - st->write_off + st->read_off - CBUF_GUARD;
}

/* Bytes that fit in one more AU right now (0: wait for a BM_EVT_READ). */
u32 adsp_shbuf_free_space(struct adsp_shbuf *sb)
{
	unsigned long flags;
	u32 n;

	spin_lock_irqsave(&sb->lock, flags);
	n = min(shbuf_free_locked(sb), sb->au_max_len);
	spin_unlock_irqrestore(&sb->lock, flags);
	return n;
}
EXPORT_SYMBOL_GPL(adsp_shbuf_free_space);

static void shbuf_notify(struct adsp_shbuf *sb, u32 evt)
{
	struct adsp_bm_evt e = {
		.evt_type = evt,
		.base_addr = sb->offset,
	};
	unsigned long flags;

	spin_lock_irqsave(&sb->lock, flags);
	e.status = sb->st;
	spin_unlock_irqrestore(&sb->lock, flags);
	adsp_imc_command(sb->adsp, sb->dsp, sb->reader, ADSP_BM_PROCESS_EVT_REMOTE,
			 &e, sizeof(e), NULL, 0, 0);
}

/*
 * Append one AU and tell the reader. Only one writer per buffer. Returns
 * -EAGAIN if it doesn't fit yet.
 */
int adsp_shbuf_write_au(struct adsp_shbuf *sb, const void *data, u32 len)
{
	struct adsp_buf_status *st = &sb->st;
	struct cbuf_au *au;
	unsigned long flags;
	u32 off, first;

	if (!len || len > sb->au_max_len)
		return -EINVAL;
	spin_lock_irqsave(&sb->lock, flags);
	if (shbuf_free_locked(sb) < len) {
		spin_unlock_irqrestore(&sb->lock, flags);
		return -EAGAIN;
	}
	off = st->write_off;
	au = shbuf_au(sb, st->write_idx);
	spin_unlock_irqrestore(&sb->lock, flags);

	/* Only we move the write side, and the reader leaves this AU alone. */
	first = min(len, sb->data_size - off);
	memcpy(sb->data + off, data, first);
	memcpy(sb->data, data + first, len - first);
	memset(au, 0, sizeof(*au));
	au->buf_offset = off;
	au->size = len;

	spin_lock_irqsave(&sb->lock, flags);
	st->write_off = (off + len) % sb->data_size;
	st->write_idx = (st->write_idx + 1) % sb->max_au;
	st->index_allocated = 0;
	st->written = 0;
	spin_unlock_irqrestore(&sb->lock, flags);

	/* Data and AU entry reach memory before the ring write (and its barrier). */
	wmb();
	shbuf_notify(sb, ADSP_BM_EVT_WRITE);
	return 0;
}
EXPORT_SYMBOL_GPL(adsp_shbuf_write_au);

/* A BM command for one of our uids (from the IPC rx work). */
void adsp_shbuf_event(struct lg1314_adsp *adsp, u32 uid, const void *param, u32 len)
{
	const struct adsp_bm_evt *e = param;
	struct adsp_shbuf *sb, *found = NULL;
	unsigned long flags;

	if (len < sizeof(*e))
		return;
	mutex_lock(&adsp->shbuf_lock);
	list_for_each_entry(sb, &adsp->shbufs, list)
		if (sb->uid == uid)
			found = sb;
	if (!found || e->evt_type != ADSP_BM_EVT_READ) {
		mutex_unlock(&adsp->shbuf_lock);
		dev_dbg(adsp->dev, "BM event %u for %08x ignored\n", e->evt_type, uid);
		return;
	}
	sb = found;
	spin_lock_irqsave(&sb->lock, flags);
	sb->st.read_idx = e->status.read_idx;
	sb->st.read_off = e->status.read_off;
	spin_unlock_irqrestore(&sb->lock, flags);
	wake_up(&sb->wq);
	if (sb->on_read)
		sb->on_read(sb->priv);
	mutex_unlock(&adsp->shbuf_lock);
}

/*
 * Allocate a buffer and hand it to @reader, a glue on a DSP whose output is
 * already connected (that makes it the reading end).
 */
struct adsp_shbuf *adsp_shbuf_create(struct lg1314_adsp *adsp, u32 reader, u32 size,
				     u32 au_max_len, u32 max_au)
{
	return adsp_shbuf_create_on(adsp, ADSP_UID_PROC(reader) - ADSP_CORE_DSP(0), reader,
				    size, au_max_len, max_au);
}
EXPORT_SYMBOL_GPL(adsp_shbuf_create);

/*
 * Allocate and initialise a buffer read by @reader on @dsp (legacy module ids
 * don't carry the DSP), without handing it over: the caller attaches it.
 */
struct adsp_shbuf *adsp_shbuf_alloc(struct lg1314_adsp *adsp, int dsp, u32 reader,
				    u32 size, u32 au_max_len, u32 max_au)
{
	struct adsp_shbuf *sb;
	struct cbuf_hdr *hdr;
	phys_addr_t phys;

	if (dsp < 0 || dsp >= ADSP_NUM_DSP || max_au < 2 ||
	    size < CBUF_STRUCT_SIZE(max_au) + 2 * au_max_len)
		return ERR_PTR(-EINVAL);

	sb = kzalloc(sizeof(*sb), GFP_KERNEL);
	if (!sb)
		return ERR_PTR(-ENOMEM);
	sb->mem = gen_pool_dma_alloc(adsp->pool, size, &phys);
	if (!sb->mem) {
		kfree(sb);
		return ERR_PTR(-ENOMEM);
	}
	sb->adsp = adsp;
	sb->dsp = dsp;
	sb->reader = reader;
	sb->size = size;
	sb->offset = DSP_SHMEM_BASE + (phys - (adsp->mem_phys + MEM_SE));
	sb->max_au = max_au;
	sb->au_max_len = au_max_len;
	sb->data = sb->mem + CBUF_STRUCT_SIZE(max_au) + au_max_len;
	sb->data_size = size - CBUF_STRUCT_SIZE(max_au) - au_max_len;
	spin_lock_init(&sb->lock);
	init_waitqueue_head(&sb->wq);

	memset(sb->mem, 0, CBUF_STRUCT_SIZE(max_au));
	hdr = sb->mem;
	hdr->max_size = sb->data_size;
	hdr->au_max_len = au_max_len;
	hdr->max_index = max_au;
	wmb();

	mutex_lock(&adsp->shbuf_lock);
	sb->uid = ADSP_UID(ADSP_CORE_ARM, ADSP_IMPL_ARM_SHBUF, ++adsp->shbuf_next);
	list_add_tail(&sb->list, &adsp->shbufs);
	mutex_unlock(&adsp->shbuf_lock);
	return sb;
}
EXPORT_SYMBOL_GPL(adsp_shbuf_alloc);

/* Allocate a buffer and hand it to @reader, a glue on @dsp. */
struct adsp_shbuf *adsp_shbuf_create_on(struct lg1314_adsp *adsp, int dsp, u32 reader,
					u32 size, u32 au_max_len, u32 max_au)
{
	struct adsp_shbuf *sb;
	u32 param[3];
	int ret;

	sb = adsp_shbuf_alloc(adsp, dsp, reader, size, au_max_len, max_au);
	if (IS_ERR(sb))
		return sb;
	param[0] = sb->offset;
	param[1] = size;
	param[2] = sb->uid;
	ret = adsp_imc_command(adsp, dsp, reader, ADSP_GLUE_SET_SHARED_BUFFER, param,
			       sizeof(param), NULL, 0, 1000);
	if (ret) {
		dev_err(adsp->dev, "glue %08x: set shared buffer: %d\n", reader, ret);
		adsp_shbuf_destroy(sb);
		return ERR_PTR(ret < 0 ? ret : -EIO);
	}
	return sb;
}
EXPORT_SYMBOL_GPL(adsp_shbuf_create_on);

/* The reader glue must be destroyed first (it keeps using the memory). */
void adsp_shbuf_destroy(struct adsp_shbuf *sb)
{
	struct lg1314_adsp *adsp = sb->adsp;

	mutex_lock(&adsp->shbuf_lock);
	list_del(&sb->list);
	mutex_unlock(&adsp->shbuf_lock);
	gen_pool_free(adsp->pool, (unsigned long)sb->mem, sb->size);
	kfree(sb);
}
EXPORT_SYMBOL_GPL(adsp_shbuf_destroy);
