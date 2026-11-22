// SPDX-License-Identifier: GPL-2.0
/*
 * LG1314 video-decode coprocessor: raw decoder interface (development only).
 *
 * /dev/lxvdec forwards the firmware's own decoder commands unchanged so the
 * decode protocol can be worked out from userspace. Each open file is one
 * decoder instance in the firmware.
 *
 *  - ioctl(cmd, arg) goes to the firmware as is: cmd uses the firmware's
 *    numbering (type 'v'); if cmd has an argument struct, it is copied in
 *    and/or out according to cmd's direction bits. Structs use the firmware's
 *    32-bit layout, and cmd's size bits must match it. The return value is
 *    the firmware's.
 *  - mmap offset 0 maps the notification ring, offset <ring size> the stream
 *    buffer (CPB), once GET_NOTI_INFO / GET_CPB_INFO have reported them.
 *  - poll() reports the readiness the firmware last announced.
 *
 * /dev/lxvo does the same for the firmware's video output (picture display):
 * each open file is one output instance, ioctls are the firmware's VO
 * commands, and poll() follows the output's readiness. It has no mmap.
 *
 * Unbinding the device while the file is open is not supported.
 */

#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "lg1314-vdec.h"

#define PT_IOC_TYPE		'v'

/* The firmware's buffer description (LX_VDEC_BUFFER_INFO_T). */
struct pt_buffer_info {
	u32 addr;
	s32 size;
	s32 read_offset;
	s32 write_offset;
};

#define PT_GET_NOTI_INFO	_IOR(PT_IOC_TYPE, 11, struct pt_buffer_info)
#define PT_GET_NOTI_INFO_WAIT	_IOR(PT_IOC_TYPE, 12, struct pt_buffer_info)
#define PT_GET_CPB_INFO		_IOR(PT_IOC_TYPE, 33, struct pt_buffer_info)

/* Readiness bits as the firmware reports them. */
#define FW_POLLIN		0x1
#define FW_POLLOUT		0x4
#define FW_POLLERR		0x8

struct lg1314_vdec_pt {
	struct lg1314_vdec	*v;
	struct miscdevice	misc;
	struct miscdevice	misc_vo;
	spinlock_t		lock;		/* protects files */
	struct list_head	files;
};

struct pt_file {
	struct list_head	list;
	struct lg1314_vdec_pt	*pt;
	u32			handle;		/* the firmware's decoder or output */
	bool			vo;		/* an output (/dev/lxvo) */
	u32			poll;		/* FW_POLL* */
	wait_queue_head_t	wait;

	struct mutex		lock;		/* protects the buffers below */
	phys_addr_t		noti_phys;
	u32			noti_size;
	phys_addr_t		cpb_phys;
	u32			cpb_size;
};

static const struct file_operations pt_vo_fops;

static int pt_open(struct inode *inode, struct file *f)
{
	struct miscdevice *misc = f->private_data;
	bool vo = misc->fops == &pt_vo_fops;
	struct lg1314_vdec_pt *pt = vo ?
		container_of(misc, struct lg1314_vdec_pt, misc_vo) :
		container_of(misc, struct lg1314_vdec_pt, misc);
	struct pt_file *pf;
	int ret, handle;

	pf = kzalloc(sizeof(*pf), GFP_KERNEL);
	if (!pf)
		return -ENOMEM;
	pf->pt = pt;
	pf->vo = vo;
	pf->poll = FW_POLLOUT;
	init_waitqueue_head(&pf->wait);
	mutex_init(&pf->lock);

	ret = lg1314_vdec_call(pt->v, vo ? VDEC_CALL_VO_OPEN : VDEC_CALL_VDEC_OPEN,
			       0, NULL, 0, 0, &handle);
	if (!ret && !handle)
		ret = -EIO;
	if (ret) {
		kfree(pf);
		return ret;
	}
	pf->handle = handle;

	spin_lock_irq(&pt->lock);
	list_add(&pf->list, &pt->files);
	spin_unlock_irq(&pt->lock);
	f->private_data = pf;
	return 0;
}

static int pt_release(struct inode *inode, struct file *f)
{
	struct pt_file *pf = f->private_data;
	struct lg1314_vdec_pt *pt = pf->pt;
	int result;

	spin_lock_irq(&pt->lock);
	list_del(&pf->list);
	spin_unlock_irq(&pt->lock);

	lg1314_vdec_call(pt->v, pf->vo ? VDEC_CALL_VO_CLOSE : VDEC_CALL_VDEC_CLOSE,
			 pf->handle, NULL, 0, 0, &result);
	kfree(pf);
	return 0;
}

/* Remember where the firmware's rings are, for mmap. */
static void pt_note_buffers(struct pt_file *pf, unsigned int cmd,
			    const struct pt_buffer_info *bi)
{
	struct lg1314_vdec *v = pf->pt->v;

	mutex_lock(&pf->lock);
	switch (cmd) {
	case PT_GET_NOTI_INFO:
	case PT_GET_NOTI_INFO_WAIT:
		/* A coprocessor address. */
		pf->noti_phys = lg1314_vdec_copro_to_phys(v, bi->addr);
		pf->noti_size = bi->size;
		break;
	case PT_GET_CPB_INFO:
		/* A physical address (it came from our allocator). */
		pf->cpb_phys = bi->addr;
		pf->cpb_size = bi->size;
		break;
	}
	mutex_unlock(&pf->lock);
}

static long pt_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	struct pt_file *pf = f->private_data;
	void __user *uarg = (void __user *)arg;
	u32 size = _IOC_SIZE(cmd);
	unsigned int dir = VDEC_CALL_WR;
	int ret, result;
	u32 *data;

	if (_IOC_TYPE(cmd) != PT_IOC_TYPE)
		return -ENOTTY;
	if (size > VDEC_CALL_MAX_DATA - 2 * sizeof(u32))
		return -EINVAL;

	/* {cmd, arg} then the argument struct, as the firmware expects. */
	data = kzalloc(2 * sizeof(u32) + size, GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	data[0] = cmd;
	data[1] = (u32)arg;
	if (size) {
		if ((_IOC_DIR(cmd) & _IOC_WRITE) && copy_from_user(data + 2, uarg, size)) {
			ret = -EFAULT;
			goto out;
		}
		if (_IOC_DIR(cmd) & _IOC_READ)
			dir |= VDEC_CALL_RD;
	}

	ret = lg1314_vdec_call(pf->pt->v,
			       pf->vo ? VDEC_CALL_VO_IOCTL : VDEC_CALL_VDEC_IOCTL,
			       pf->handle,
			       data, 2 * sizeof(u32) + size, dir, &result);
	if (ret)
		goto out;
	if (size && (_IOC_DIR(cmd) & _IOC_READ) && copy_to_user(uarg, data + 2, size)) {
		ret = -EFAULT;
		goto out;
	}
	if (!pf->vo && result >= 0 && size == sizeof(struct pt_buffer_info))
		pt_note_buffers(pf, cmd, (struct pt_buffer_info *)(data + 2));
	ret = result;
out:
	kfree(data);
	return ret;
}

static int pt_mmap(struct file *f, struct vm_area_struct *vma)
{
	struct pt_file *pf = f->private_data;
	struct lg1314_vdec *v = pf->pt->v;
	size_t len = vma->vm_end - vma->vm_start;
	phys_addr_t phys = 0;
	u32 size = 0;

	if (pf->vo)
		return -ENODEV;

	mutex_lock(&pf->lock);
	if (vma->vm_pgoff == 0) {
		phys = pf->noti_phys;
		size = pf->noti_size;
	} else if (pf->noti_size && vma->vm_pgoff == pf->noti_size >> PAGE_SHIFT) {
		phys = pf->cpb_phys;
		size = pf->cpb_size;
	}
	mutex_unlock(&pf->lock);

	if (!phys || !PAGE_ALIGNED(phys) || len != PAGE_ALIGN(size) ||
	    !lg1314_vdec_in_carveout(v, phys, len)) {
		dev_warn(lg1314_vdec_device(v),
			 "lxvdec: refusing mmap at page %lu (%zu bytes): buffer %pa/%u\n",
			 vma->vm_pgoff, len, &phys, size);
		return -EINVAL;
	}

	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	vm_flags_set(vma, VM_IO | VM_DONTEXPAND | VM_DONTDUMP);
	return remap_pfn_range(vma, vma->vm_start, PHYS_PFN(phys), len,
			       vma->vm_page_prot);
}

static __poll_t pt_poll_wait(struct file *f, struct poll_table_struct *wait)
{
	struct pt_file *pf = f->private_data;
	u32 mask;

	poll_wait(f, &pf->wait, wait);
	mask = READ_ONCE(pf->poll);
	return (mask & FW_POLLIN ? EPOLLIN | EPOLLRDNORM : 0) |
	       (mask & FW_POLLOUT ? EPOLLOUT | EPOLLWRNORM : 0) |
	       (mask & FW_POLLERR ? EPOLLERR : 0);
}

static const struct file_operations pt_fops = {
	.owner		= THIS_MODULE,
	.open		= pt_open,
	.release	= pt_release,
	.unlocked_ioctl	= pt_ioctl,
	.mmap		= pt_mmap,
	.poll		= pt_poll_wait,
};

static const struct file_operations pt_vo_fops = {
	.owner		= THIS_MODULE,
	.open		= pt_open,
	.release	= pt_release,
	.unlocked_ioctl	= pt_ioctl,
	.poll		= pt_poll_wait,
};

void lg1314_vdec_pt_poll(struct lg1314_vdec_pt *pt, u32 handle, u32 mask)
{
	struct pt_file *pf;

	if (!pt)
		return;
	spin_lock(&pt->lock);
	list_for_each_entry(pf, &pt->files, list) {
		if (pf->handle == handle) {
			WRITE_ONCE(pf->poll, mask);
			wake_up_interruptible(&pf->wait);
		}
	}
	spin_unlock(&pt->lock);
}

struct lg1314_vdec_pt *lg1314_vdec_pt_init(struct lg1314_vdec *v)
{
	struct lg1314_vdec_pt *pt;
	int ret;

	pt = kzalloc(sizeof(*pt), GFP_KERNEL);
	if (!pt)
		return ERR_PTR(-ENOMEM);
	pt->v = v;
	spin_lock_init(&pt->lock);
	INIT_LIST_HEAD(&pt->files);
	pt->misc.minor = MISC_DYNAMIC_MINOR;
	pt->misc.name = "lxvdec";
	pt->misc.fops = &pt_fops;
	pt->misc.mode = 0600;
	pt->misc.parent = lg1314_vdec_device(v);

	pt->misc_vo = pt->misc;
	pt->misc_vo.name = "lxvo";
	pt->misc_vo.fops = &pt_vo_fops;

	ret = misc_register(&pt->misc);
	if (ret) {
		kfree(pt);
		return ERR_PTR(ret);
	}
	ret = misc_register(&pt->misc_vo);
	if (ret) {
		misc_deregister(&pt->misc);
		kfree(pt);
		return ERR_PTR(ret);
	}
	dev_warn(lg1314_vdec_device(v),
		 "raw decoder interfaces /dev/lxvdec, /dev/lxvo enabled (development only)\n");
	return pt;
}

void lg1314_vdec_pt_exit(struct lg1314_vdec_pt *pt)
{
	if (!pt)
		return;
	misc_deregister(&pt->misc_vo);
	misc_deregister(&pt->misc);
	kfree(pt);
}
