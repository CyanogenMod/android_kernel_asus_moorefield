/*
 * Copyright (c) 2013--2014 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/css2600-psys.h>
#include <linux/device.h>
#include <linux/dma-attrs.h>
#include <linux/dma-buf.h>
#include <linux/fs.h>
#include <linux/highmem.h>
#include <linux/iommu.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/sizes.h>
#include <linux/uaccess.h>

#include "css2600.h"
#include "css2600-bus.h"
#include "css2600-psys-lib.h"
#include "css2600-psys.h"
#include "css2600-wrapper-2401.h"
#include "ia_css_psys_process_group.h"

#define CSS2600_PSYS_NUM_DEVICES	4

static dev_t css2600_psys_dev_t;
static DECLARE_BITMAP(css2600_psys_devices, CSS2600_PSYS_NUM_DEVICES);
static DEFINE_MUTEX(css2600_psys_mutex);

static struct bus_type css2600_psys_bus = {
	.name = CSS2600_PSYS_NAME,
};

static const struct css2600_psys_capability caps = {
	.version = 1,
	.driver = "css2600-psys",
};

static struct css2600_psys_kbuffer *css2600_psys_lookup_kbuffer(
			struct css2600_psys_fh *fh, int fd)
{
	struct css2600_psys_kbuffer *kbuffer;

	list_for_each_entry(kbuffer, &fh->bufmap, list) {
		if (kbuffer->fd == fd)
			return kbuffer;
	}

	return NULL;
}

static int css2600_psys_get_userpages(struct css2600_psys_kbuffer *kbuf)
{
	struct vm_area_struct *vma;
	unsigned long start, end;
	int npages, array_size;
	struct page **pages;
	struct sg_table *sgt;
	int nr = 0;
	int ret = -ENOMEM;

	start = (unsigned long)kbuf->userptr;
	end = PAGE_ALIGN(start + kbuf->len);
	npages = (end - (start & PAGE_MASK)) >> PAGE_SHIFT;
	array_size = npages * sizeof(struct page *);

	sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt)
		return -ENOMEM;

	if (array_size <= PAGE_SIZE)
		pages = kzalloc(array_size, GFP_KERNEL);
	else
		pages = vzalloc(array_size);
	if (!pages)
		goto error;

	down_read(&current->mm->mmap_sem);
	vma = find_vma(current->mm, start);
	if (!vma) {
		ret = -EFAULT;
		goto error_up_read;
	}
	/*
	 * For buffers from Gralloc, VM_PFNMAP is expected,
	 * but VM_IO is set. Possibly bug in Gralloc.
	 */
	kbuf->vma_is_io = vma->vm_flags & (VM_IO | VM_PFNMAP);

	if (kbuf->vma_is_io) {
		unsigned long io_start = start;

		for (nr = 0; nr < npages; nr++, io_start += PAGE_SIZE) {
			unsigned long pfn;

			ret = follow_pfn(vma, io_start, &pfn);
			if (ret)
				goto error_up_read;
			pages[nr] = pfn_to_page(pfn);
		}
	} else {
		nr = get_user_pages(current, current->mm, start & PAGE_MASK,
				    npages, 1, 0, pages, NULL);
		if (nr < npages)
			goto error_up_read;
	}
	up_read(&current->mm->mmap_sem);

	ret = sg_alloc_table_from_pages(sgt, pages, npages,
					start & ~PAGE_MASK, kbuf->len,
					GFP_KERNEL);
	if (ret < 0)
		goto error;

	kbuf->sgt = sgt;
	kbuf->pages = pages;
	kbuf->npages = npages;

	return 0;

error_up_read:
	up_read(&current->mm->mmap_sem);
error:
	if (!kbuf->vma_is_io)
		while (nr > 0)
			put_page(pages[--nr]);

	if (array_size <= PAGE_SIZE)
		kfree(pages);
	else
		vfree(pages);

	kfree(sgt);

	return ret;
}

static void css2600_psys_put_userpages(struct css2600_psys_kbuffer *kbuf)
{
	if (!kbuf->userptr || !kbuf->sgt)
		return;

	if (!kbuf->vma_is_io)
		while (kbuf->npages)
			put_page(kbuf->pages[--kbuf->npages]);

	if (is_vmalloc_addr(kbuf->pages))
		vfree(kbuf->pages);
	else
		kfree(kbuf->pages);

	sg_free_table(kbuf->sgt);
	kfree(kbuf->sgt);
	kbuf->sgt = NULL;
}

static int css2600_dma_buf_attach(struct dma_buf *dbuf, struct device *dev,
				  struct dma_buf_attachment *attach)
{
	attach->priv = dbuf->priv;
	return 0;
}

static void css2600_dma_buf_detach(struct dma_buf *dbuf,
				   struct dma_buf_attachment *attach)
{
}

static struct sg_table *css2600_dma_buf_map(struct dma_buf_attachment *attach,
					    enum dma_data_direction dir)
{
	struct css2600_psys_kbuffer *kbuf = attach->priv;
	int ret;

	ret = css2600_psys_get_userpages(kbuf);
	if (ret)
		return NULL;

	ret = dma_map_sg(attach->dev, kbuf->sgt->sgl, kbuf->sgt->orig_nents,
			 dir);
	if (ret < kbuf->sgt->orig_nents) {
		css2600_psys_put_userpages(kbuf);
		return ERR_PTR(-EIO);
	}

	return kbuf->sgt;
}

static void css2600_dma_buf_unmap(struct dma_buf_attachment *attach,
				  struct sg_table *sg,
				  enum dma_data_direction dir)
{
	struct css2600_psys_kbuffer *kbuf = attach->priv;

	dma_unmap_sg(attach->dev, sg->sgl, sg->orig_nents, dir);
	css2600_psys_put_userpages(kbuf);
}

static int css2600_dma_buf_mmap(struct dma_buf *dbuf,
				struct vm_area_struct *vma)
{
	return -ENOTTY;
}

static void *css2600_dma_buf_kmap(struct dma_buf *dbuf, unsigned long pgnum)
{
	return NULL;
}

static void *css2600_dma_buf_kmap_atomic(struct dma_buf *dbuf, unsigned long pgnum)
{
	return NULL;
}

static void css2600_dma_buf_release(struct dma_buf *buf)
{
}

int css2600_dma_buf_begin_cpu_access(struct dma_buf *dma_buf, size_t start,
				     size_t len, enum dma_data_direction dir)
{
	return -ENOTTY;
}

static void *css2600_dma_buf_vmap(struct dma_buf *dmabuf)
{
	struct css2600_psys_kbuffer *kbuf = dmabuf->priv;

	return vm_map_ram(kbuf->pages, kbuf->npages, 0, PAGE_KERNEL);
}

static void css2600_dma_buf_vunmap(struct dma_buf *dmabuf, void *vaddr)
{
	struct css2600_psys_kbuffer *kbuf = dmabuf->priv;

	vm_unmap_ram(vaddr, kbuf->npages);
}

static struct dma_buf_ops css2600_dma_buf_ops = {
	.attach = css2600_dma_buf_attach,
	.detach = css2600_dma_buf_detach,
	.map_dma_buf = css2600_dma_buf_map,
	.unmap_dma_buf = css2600_dma_buf_unmap,
	.release = css2600_dma_buf_release,
	.begin_cpu_access = css2600_dma_buf_begin_cpu_access,
	.kmap = css2600_dma_buf_kmap,
	.kmap_atomic = css2600_dma_buf_kmap_atomic,
	.mmap = css2600_dma_buf_mmap,
	.vmap = css2600_dma_buf_vmap,
	.vunmap = css2600_dma_buf_vunmap,
};

static int css2600_psys_open(struct inode *inode, struct file *file)
{
	struct css2600_psys *psys = inode_to_css2600_psys(inode);
	struct css2600_psys_fh *fh;
	int i;

	fh = kzalloc(sizeof(*fh), GFP_KERNEL);
	if (!fh)
		return -ENOMEM;

	INIT_LIST_HEAD(&fh->bufmap);
	INIT_LIST_HEAD(&fh->eventq);
	for (i = 0; i < CSS2600_PSYS_CMD_PRIORITY_NUM; i++)
		INIT_LIST_HEAD(&fh->kcmds[i]);

	init_waitqueue_head(&fh->wait);

	fh->psys = psys;
	file->private_data = fh;

	mutex_lock(&psys->mutex);
	list_add(&fh->list, &psys->fhs);
	mutex_unlock(&psys->mutex);

	return 0;
}

static void css2600_psys_clean_kcmd(struct css2600_psys_kcmd *kcmd)
{
	if (!kcmd)
		return;

	if (kcmd->pg) {
		ia_css_process_group_destroy(kcmd->pg);
		kcmd->pg = NULL;
	}

	kfree(kcmd->pg_manifest);
	kfree(kcmd->pg_params);
	kfree(kcmd->kbufs);
	kfree(kcmd);
}

static int css2600_psys_queue_event(struct css2600_psys_fh *fh,
				    struct css2600_psys_event *e)
{
	struct css2600_psys *psys = fh->psys;
	struct css2600_psys_eventq *eventq;

	/* FIXME: shouldn't allocate memory here */
	eventq = kzalloc(sizeof(*eventq), GFP_KERNEL);
	if (!eventq) {
		dev_err(&psys->adev->dev, "no memory for event\n");
		return -ENOMEM;
	}

	eventq->ev = *e;

	dev_dbg(&psys->adev->dev, "queue event %u\n", eventq->ev.type);

	mutex_lock(&psys->mutex);
	list_add_tail(&eventq->list, &fh->eventq);
	mutex_unlock(&psys->mutex);

	wake_up_interruptible(&fh->wait);

	return 0;
}

static int css2600_psys_release(struct inode *inode, struct file *file)
{
	struct css2600_psys *psys = inode_to_css2600_psys(inode);
	struct css2600_psys_fh *fh = file->private_data;
	struct css2600_psys_kbuffer *kbuf, *kbuf0;
	struct css2600_psys_kcmd *kcmd, *kcmd0;
	int i;

	mutex_lock(&psys->mutex);
	for (i = 0; i < CSS2600_PSYS_CMD_PRIORITY_NUM; i++)
		list_for_each_entry_safe(kcmd, kcmd0, &fh->kcmds[i], list) {
			list_del(&kcmd->list);
			css2600_psys_clean_kcmd(kcmd);
		}

	list_for_each_entry_safe(kbuf, kbuf0, &fh->bufmap, list) {
		list_del(&kbuf->list);
		css2600_psys_put_userpages(kbuf);
		kfree(kbuf);
	}
	list_del(&fh->list);
	mutex_unlock(&psys->mutex);
	kfree(fh);

	return 0;
}

static int css2600_psys_getbuf(struct css2600_psys_buffer *buf,
			       struct css2600_psys_fh *fh)
{
	struct css2600_psys_kbuffer *kbuf;
	struct css2600_psys *psys = fh->psys;
	struct dma_buf *dbuf;
	int ret;

	if (!buf->userptr) {
		dev_err(&psys->adev->dev, "Buffer allocation not supported\n");
		return -EINVAL;
	}

	kbuf = kzalloc(sizeof(*kbuf), GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;

	kbuf->len = buf->len;
	kbuf->userptr = buf->userptr;

	dbuf = dma_buf_export(kbuf, &css2600_dma_buf_ops, kbuf->len, 0);
	if (IS_ERR(dbuf)) {
		kfree(kbuf);
		return PTR_ERR(dbuf);
	}

	ret = dma_buf_fd(dbuf, 0);
	if (ret < 0) {
		kfree(kbuf);
		return ret;
	}
	kbuf->fd = buf->fd = ret;

	mutex_lock(&psys->mutex);
	list_add_tail(&kbuf->list, &fh->bufmap);
	mutex_unlock(&psys->mutex);

	dev_dbg(&psys->adev->dev, "IOC_GETBUF: userptr %p to %d\n",
		buf->userptr, buf->fd);

	return 0;
}

static int css2600_psys_putbuf(struct css2600_psys_buffer *buf,
			       struct css2600_psys_fh *fh)
{
	struct css2600_psys *psys = fh->psys;
	struct css2600_psys_kbuffer *kbuf;
	struct dma_buf *dbuf;

	mutex_lock(&psys->mutex);
	dbuf = dma_buf_get(buf->fd);
	if (IS_ERR(dbuf)) {
		mutex_unlock(&psys->mutex);
		return PTR_ERR(dbuf);
	}
	kbuf = dbuf->priv;
	list_del(&kbuf->list);
	mutex_unlock(&psys->mutex);

	dma_buf_put(dbuf);
	css2600_psys_put_userpages(kbuf);
	kfree(kbuf);

	dev_dbg(&psys->adev->dev, "IOC_PUTBUF: buffer %d\n", buf->fd);

	return 0;
}

struct css2600_psys_kcmd *
css2600_psys_copy_cmd(struct css2600_psys_command *cmd,
		      struct css2600_psys_fh *fh)
{
	struct css2600_psys *psys = fh->psys;
	struct css2600_psys_kcmd *kcmd;
	unsigned int i;
	int *buffers;
	int ret;

	kcmd = kzalloc(sizeof(*kcmd), GFP_KERNEL);
	if (!kcmd)
		return NULL;

	buffers = kzalloc(cmd->bufcount * sizeof(buffers[0]), GFP_KERNEL);
	if (!buffers)
		goto error;

	kcmd->pg_manifest = kzalloc(cmd->pg_manifest_size, GFP_KERNEL);
	if (!kcmd->pg_manifest)
		goto error;

	kcmd->pg_params = kzalloc(cmd->pg_params_size, GFP_KERNEL);
	if (!kcmd->pg_params)
		goto error;

	kcmd->kbufs = kzalloc(cmd->bufcount * sizeof(kcmd->kbufs[0]),
			      GFP_KERNEL);
	if (!kcmd->kbufs)
		goto error;

	ret = copy_from_user(kcmd->pg_manifest, cmd->pg_manifest,
			     cmd->pg_manifest_size);
	if (ret)
		goto error;

	kcmd->pg_manifest_size = cmd->pg_manifest_size;

	ret = copy_from_user(kcmd->pg_params, cmd->pg_params,
			     cmd->pg_params_size);
	if (ret)
		goto error;

	kcmd->pg_params_size = cmd->pg_params_size;

	ret = copy_from_user(buffers, cmd->buffers,
			     cmd->bufcount * sizeof(buffers[0]));
	if (ret)
		goto error;

	kcmd->nbuffers = cmd->bufcount;
	kcmd->id = cmd->id;
	kcmd->issue_id = cmd->issue_id;
	kcmd->priority = cmd->priority;
	if (kcmd->priority >= CSS2600_PSYS_CMD_PRIORITY_NUM)
		goto error;

	for (i = 0; i < kcmd->nbuffers; i++) {
		mutex_lock(&psys->mutex);
		kcmd->kbufs[i] = css2600_psys_lookup_kbuffer(fh, buffers[i]);
		mutex_unlock(&psys->mutex);
		if (!kcmd->kbufs[i] || !kcmd->kbufs[i]->sgt)
			goto error;
		/* FIXME: flush only when needed */
		dma_sync_sg_for_device(&psys->adev->dev,
				       kcmd->kbufs[i]->sgt->sgl,
				       kcmd->kbufs[i]->sgt->orig_nents,
				       DMA_BIDIRECTIONAL);
	}

	kfree(buffers);

	return kcmd;
error:
	css2600_psys_clean_kcmd(kcmd);

	if (buffers)
		kfree(buffers);

	return NULL;
}

static struct css2600_psys_kcmd *__peek_next_cmd(struct css2600_psys *psys)
{
	struct css2600_psys_fh *fh;
	int i;

	for (i = 0; i < CSS2600_PSYS_CMD_PRIORITY_NUM; i++) {
		list_for_each_entry(fh, &psys->fhs, list) {
			if (list_empty(&fh->kcmds[i]))
				continue;
			return list_first_entry(&fh->kcmds[i],
						struct css2600_psys_kcmd,
						list);
		}
	}

	return NULL;
}

static int __can_cmd_run(struct css2600_psys *psys,
			 struct css2600_psys_kcmd *kcmd)
{
	return kcmd && !(psys->resource_bitmap & ia_css_process_group_get_resource_bitmap(kcmd->pg));
}

static void __set_cmd_running(struct css2600_psys *psys,
			     struct css2600_psys_kcmd *kcmd)
{
	psys->resource_bitmap |= ia_css_process_group_get_resource_bitmap(kcmd->pg);
	list_move_tail(&kcmd->fh->list, &psys->fhs);
	list_del(&kcmd->list);
}

static struct css2600_psys_kcmd *__get_next_cmd(struct css2600_psys *psys)
{
	struct css2600_psys_kcmd *kcmd;

	kcmd = __peek_next_cmd(psys);
	if (!__can_cmd_run(psys, kcmd))
		return NULL;

	__set_cmd_running(psys, kcmd);

	return kcmd;
}

static int css2600_psys_run_next(struct css2600_psys *psys)
{
	struct css2600_psys_kcmd *kcmd;
	int ret;

	mutex_lock(&psys->mutex);
	kcmd = __get_next_cmd(psys);

	if (!kcmd) {
		mutex_unlock(&psys->mutex);
		return 0;
	}

	ret = -ia_css_process_group_start(kcmd->pg);
	mutex_unlock(&psys->mutex);
	if (ret) {
		struct css2600_psys_event ev = {
			.type = CSS2600_PSYS_EVENT_TYPE_CMD_COMPLETE,
			.error = -EIO,
			.buffer_idx = 0,
			.id = kcmd->id,
			.issue_id = kcmd->issue_id
		};

		css2600_psys_queue_event(kcmd->fh, &ev);
		mutex_lock(&psys->mutex);
		css2600_psys_clean_kcmd(kcmd);
		mutex_unlock(&psys->mutex);
	}

	return ret;
}

static int css2600_psys_qcmd(struct css2600_psys_command *cmd,
			     struct css2600_psys_fh *fh)
{
	struct css2600_psys *psys = fh->psys;
	struct css2600_psys_kcmd *kcmd;
	unsigned int terminal_count;
	unsigned int i;
	int ret = -ENOMEM;

	kcmd = css2600_psys_copy_cmd(cmd, fh);
	if (!kcmd)
		return -EINVAL;

	kcmd->fh = fh;

	mutex_lock(&psys->mutex);
	kcmd->pg = ia_css_process_group_create(kcmd->pg_manifest,
					       kcmd->pg_params);
	if (!kcmd->pg) {
		dev_err(&psys->adev->dev, "failed to create process group\n");
		ret = -EIO;
		goto error;
	}

	terminal_count = ia_css_process_group_get_terminal_count(kcmd->pg);
	if (terminal_count > kcmd->nbuffers) {
		ret = -EINVAL;
		goto error;
	}

	for (i = 0; i < terminal_count; i++) {
		ia_css_buffer_state_t buffer_state;
		ia_css_terminal_t *terminal;
		ia_css_terminal_type_t type;
		vied_vaddress_t buffer;

		terminal = ia_css_process_group_get_terminal(kcmd->pg, i);
		if (!terminal)
			continue;

		type = ia_css_terminal_get_type(terminal);

		switch (type) {
		case IA_CSS_TERMINAL_TYPE_PARAM_CACHED:
		case IA_CSS_TERMINAL_TYPE_PARAM_STREAM:
		case IA_CSS_TERMINAL_TYPE_DATA_IN:
		case IA_CSS_TERMINAL_TYPE_STATE_IN:
			buffer_state = IA_CSS_BUFFER_FULL;
			break;
		case IA_CSS_TERMINAL_TYPE_DATA_OUT:
		case IA_CSS_TERMINAL_TYPE_STATE_OUT:
			buffer_state = IA_CSS_BUFFER_EMPTY;
			break;
		default:
			dev_err(&psys->adev->dev,
				"unknown terminal type: 0x%x\n", type);
			continue;
		}

		buffer = (vied_vaddress_t)kcmd->kbufs[i]->dma_addr;

		ret = -ia_css_process_group_attach_buffer(kcmd->pg, buffer,
							  buffer_state, i);
		if (ret) {
			ret = -EIO;
			goto error;
		}
	}

	ia_css_process_group_set_token(kcmd->pg, kcmd);
	/* FIXME: hardcoded to 1 until real value available */
	ia_css_process_group_set_resource_bitmap(kcmd->pg, 1);

	ret = -ia_css_process_group_submit(kcmd->pg);
	if (ret) {
		ret = -EIO;
		goto error;
	}

	list_add_tail(&kcmd->list, &fh->kcmds[cmd->priority]);
	kcmd = __get_next_cmd(psys);

	if (kcmd && ia_css_process_group_start(kcmd->pg)) {
		ret = -EIO;
		goto error;
	}
	mutex_unlock(&psys->mutex);

	dev_dbg(&psys->adev->dev, "IOC_QCMD: id:%d issue_id:0x%llx pri:%d\n",
		cmd->id, cmd->issue_id, cmd->priority);

	return 0;

error:
	css2600_psys_clean_kcmd(kcmd);
	mutex_unlock(&psys->mutex);

	return ret;
}

static long css2600_ioctl_dqevent(struct css2600_psys_event *event,
				  struct css2600_psys_fh *fh,
				  unsigned int f_flags)
{
	struct css2600_psys *psys = fh->psys;
	struct css2600_psys_eventq *evq;
	int rval;

	dev_dbg(&psys->adev->dev, "IOC_DQEVENT\n");

	if (!(f_flags & O_NONBLOCK)) {
		rval = wait_event_interruptible(fh->wait,
						!list_empty(&fh->eventq));
		if (rval == -ERESTARTSYS)
			return rval;
	}

	mutex_lock(&psys->mutex);
	if (list_empty(&fh->eventq)) {
		mutex_unlock(&psys->mutex);
		return -ENODATA;
	}

	evq = list_first_entry(&fh->eventq, struct css2600_psys_eventq, list);
	*event = evq->ev;

	list_del(&evq->list);
	mutex_unlock(&psys->mutex);

	kfree(evq);

	return 0;
}


static long css2600_psys_mapbuf(int fd, struct css2600_psys_fh *fh)
{
	struct css2600_psys *psys = fh->psys;
	struct css2600_psys_kbuffer *kbuf;
	int ret;

	mutex_lock(&psys->mutex);
	kbuf = css2600_psys_lookup_kbuffer(fh, fd);
	mutex_unlock(&psys->mutex);

	if (!kbuf) {
		kbuf = kzalloc(sizeof(*kbuf), GFP_KERNEL);
		if (!kbuf)
			return -ENOMEM;
	}

	kbuf->dbuf = dma_buf_get(fd);
	if (IS_ERR(kbuf->dbuf)) {
		ret = -EINVAL;
		goto error;
	}
	kbuf->fd = fd;

	kbuf->db_attach = dma_buf_attach(kbuf->dbuf, &psys->adev->dev);
	if (IS_ERR(kbuf->db_attach)) {
		ret = PTR_ERR(kbuf->db_attach);
		goto error_put;
	}

	kbuf->sgt = dma_buf_map_attachment(kbuf->db_attach, DMA_BIDIRECTIONAL);
	if (IS_ERR_OR_NULL(kbuf->sgt)) {
		ret = -EINVAL;
		goto error_detach;
	}

	kbuf->dma_addr = sg_dma_address(kbuf->sgt->sgl);

	kbuf->kaddr = dma_buf_vmap(kbuf->dbuf);
	if (!kbuf->kaddr) {
		ret = -EINVAL;
		goto error_detach;
	}

#if IS_ENABLED(CONFIG_VIDEO_CSS2600_2401)
	ret = css2600_wrapper_register_buffer(kbuf->dma_addr, kbuf->kaddr,
					      kbuf->len);
	if (ret)
		goto error_vunmap;
#endif

	dev_dbg(&psys->adev->dev, "IOC_MAPBUF: mapped fd %d\n", fd);

	return 0;

error_vunmap:
	dma_buf_vunmap(kbuf->dbuf, kbuf->kaddr);
error_detach:
	dma_buf_detach(kbuf->dbuf, kbuf->db_attach);
error_put:
	dma_buf_put(kbuf->dbuf);
error:
	if (!kbuf->userptr)
		kfree(kbuf);
	return ret;
}

static long css2600_psys_unmapbuf(int fd, struct css2600_psys_fh *fh)
{
	struct css2600_psys_kbuffer *kbuf;
	struct css2600_psys *psys = fh->psys;

	mutex_lock(&psys->mutex);
	kbuf = css2600_psys_lookup_kbuffer(fh, fd);
	if (!kbuf) {
		mutex_unlock(&psys->mutex);
		return -EINVAL;
	}
	mutex_unlock(&psys->mutex);

	dma_buf_vunmap(kbuf->dbuf, kbuf->kaddr);
#if IS_ENABLED(CONFIG_VIDEO_CSS2600_2401)
	css2600_wrapper_unregister_buffer(kbuf->dma_addr);
#endif
	dma_buf_unmap_attachment(kbuf->db_attach, kbuf->sgt, DMA_BIDIRECTIONAL);

	dma_buf_detach(kbuf->dbuf, kbuf->db_attach);

	dma_buf_put(kbuf->dbuf);

	if (!kbuf->userptr)
		kfree(kbuf);

	dev_dbg(&psys->adev->dev, "IOC_UNMAPBUF: fd %d\n", fd);

	return 0;
}

static unsigned int css2600_psys_poll(struct file *file,
				      struct poll_table_struct *wait)
{
	struct css2600_psys_fh *fh = file->private_data;
	struct css2600_psys *psys = fh->psys;
	unsigned int res = 0;

	dev_dbg(&psys->adev->dev, "css2600 poll\n");

	poll_wait(file, &fh->wait, wait);

	mutex_lock(&psys->mutex);
	if (!list_empty(&fh->eventq))
		res = POLLIN;
	mutex_unlock(&psys->mutex);

	dev_dbg(&psys->adev->dev, "css2600 poll res %u\n", res);

	return res;
}

static long css2600_psys_ioctl(struct file *file, unsigned int cmd,
			      unsigned long arg)
{
	union {
		struct css2600_psys_buffer buf;
		struct css2600_psys_command cmd;
		struct css2600_psys_event ev;
		struct css2600_psys_capability caps;
	} karg;
	int err = 0;
	void __user *up = (void __user *)arg;

	switch (cmd) {
	case CSS2600_IOC_MAPBUF:
		return css2600_psys_mapbuf(arg, file->private_data);
	case CSS2600_IOC_UNMAPBUF:
		return css2600_psys_unmapbuf(arg, file->private_data);
	}

	if (_IOC_SIZE(cmd) > sizeof(karg))
		return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		err = copy_from_user(&karg, up, _IOC_SIZE(cmd));
		if (err)
			return -EFAULT;
	}

	switch (cmd) {
	case CSS2600_IOC_QUERYCAP:
		karg.caps = caps;
		break;
	case CSS2600_IOC_GETBUF:
		err = css2600_psys_getbuf(&karg.buf, file->private_data);
		break;
	case CSS2600_IOC_PUTBUF:
		err = css2600_psys_putbuf(&karg.buf, file->private_data);
		break;
	case CSS2600_IOC_QCMD:
		err = css2600_psys_qcmd(&karg.cmd, file->private_data);
		break;
	case CSS2600_IOC_DQEVENT:
		err = css2600_ioctl_dqevent(&karg.ev, file->private_data,
					    file->f_flags);
		break;
	default:
		err = -ENOTTY;
		break;
	}

	if (err)
		return err;

	if (_IOC_DIR(cmd) & _IOC_READ)
		if (copy_to_user(up, &karg, _IOC_SIZE(cmd)))
			return -EFAULT;

	return err;
}

static const struct file_operations css2600_psys_fops = {
	.open = css2600_psys_open,
	.release = css2600_psys_release,
	.unlocked_ioctl = css2600_psys_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = css2600_psys_compat_ioctl32,
#endif
	.poll = css2600_psys_poll,
	.owner = THIS_MODULE,
};

static void css2600_psys_dev_release(struct device *dev)
{
}

static int css2600_psys_probe(struct css2600_bus_device *adev)
{
	struct css2600_psys *psys;
	struct ia_css_syscom_config *syscom_config;
	void *syscom_buffer;
	unsigned int minor;
	int rval = -E2BIG;

	if (!css2600_wrapper_init_done())
		return -EPROBE_DEFER;

	mutex_lock(&css2600_psys_mutex);

	minor = find_next_zero_bit(css2600_psys_devices,
				   CSS2600_PSYS_NUM_DEVICES, 0);
	if (minor == CSS2600_PSYS_NUM_DEVICES) {
		dev_err(&adev->dev, "too many devices\n");
		goto out_unlock;
	}

	psys = devm_kzalloc(&adev->dev, sizeof(*psys), GFP_KERNEL);
	if (!psys) {
		rval = -ENOMEM;
		goto out_unlock;
	}

	psys->adev = adev;

	cdev_init(&psys->cdev, &css2600_psys_fops);
	psys->cdev.owner = css2600_psys_fops.owner;

	rval = cdev_add(&psys->cdev,
			MKDEV(MAJOR(css2600_psys_dev_t), minor), 1);
	if (rval) {
		dev_err(&adev->dev, "cdev_add failed (%d)\n", rval);
		goto out_unlock;
	}

	mutex_init(&psys->mutex);
	INIT_LIST_HEAD(&psys->fhs);

	css2600_bus_set_drvdata(adev, psys);

	psys->dev.parent = &adev->dev;
	psys->dev.bus = &css2600_psys_bus;
	psys->dev.devt = MKDEV(MAJOR(css2600_psys_dev_t), minor);
	psys->dev.release = css2600_psys_dev_release;
	dev_set_name(&psys->dev, "ipu-psys" "%d", minor);
	rval = device_register(&psys->dev);
	if (rval < 0) {
		dev_err(&psys->dev, "psys device_register failed\n");
		goto out_mutex_destroy;
	}

	set_bit(minor, css2600_psys_devices);

	dev_info(&adev->dev, "psys probe minor: %d\n", minor);
#if IS_ENABLED(CONFIG_VIDEO_CSS2600_2401)
	syscom_config = ia_css_psys_specify();
	syscom_config->specific_addr = libcss2401_get_sp_fw();
	syscom_buffer = NULL;
#else
	syscom_config = NULL;
	syscom_buffer = kzalloc(ia_css_sizeof_psys(syscom_config), GFP_KERNEL);
	if (!syscom_buffer) {
		dev_err(&psys->dev, "psys unable to alloc syscom config (%u)\n",
			ia_css_sizeof_psys(syscom_config));
		goto out_mutex_destroy;
	}
#endif
	psys->dev_ctx = ia_css_psys_open(syscom_buffer, syscom_config);

	mutex_unlock(&css2600_psys_mutex);

	return 0;

out_mutex_destroy:
	mutex_destroy(&psys->mutex);
	cdev_del(&psys->cdev);
out_unlock:
	mutex_unlock(&css2600_psys_mutex);

	return rval;
}

static void css2600_psys_remove(struct css2600_bus_device *adev)
{
	struct css2600_psys *psys = css2600_bus_get_drvdata(adev);

	mutex_lock(&css2600_psys_mutex);
	ia_css_psys_close(psys->dev_ctx);

	device_unregister(&psys->dev);

	clear_bit(MINOR(psys->cdev.dev), css2600_psys_devices);
	cdev_del(&psys->cdev);

	mutex_unlock(&css2600_psys_mutex);

	mutex_destroy(&psys->mutex);

	dev_info(&adev->dev, "removed\n");

}

static void psys_isr(struct css2600_bus_device *adev)
{
	struct css2600_psys *psys = css2600_bus_get_drvdata(adev);
	struct css2600_psys_event ev = { 0 };
	struct css2600_psys_kcmd *kcmd;
	struct css2600_psys_fh *fh;
	struct ia_css_psys_event_s event;
	int ret;

	mutex_lock(&psys->mutex);
	ret = ia_css_psys_event_queue_receive(psys->dev_ctx,
			IA_CSS_PSYS_EVENT_QUEUE_MAIN_ID,
			&event);
	if (!ret) {
		mutex_unlock(&psys->mutex);
		dev_err(&adev->dev, "interrupt but no events received\n");
		return;
	}

/* TODO: Events not yet defined in 2600 library */
#if IS_ENABLED(CONFIG_VIDEO_CSS2600_2401)
	if (event.status != IA_CSS_PSYS_EVENT_TYPE_CMD_COMPLETE)
		ev.error = -EIO;
#endif

	dev_info(&adev->dev, "psys received event status:%d\n", event.status);

	kcmd = (struct css2600_psys_kcmd *)event.token;
	psys->resource_bitmap &= ~ia_css_process_group_get_resource_bitmap(kcmd->pg);

/* TODO: Events not yet defined in 2600 library */
#if IS_ENABLED(CONFIG_VIDEO_CSS2600_2401)
	ev.type = CSS2600_PSYS_EVENT_TYPE_CMD_COMPLETE;
#endif
	ev.id = kcmd->id;
	ev.issue_id = kcmd->issue_id;

	fh = kcmd->fh;
	css2600_psys_clean_kcmd(kcmd);
	mutex_unlock(&psys->mutex);

	css2600_psys_queue_event(fh, &ev);
	css2600_psys_run_next(psys);
}

static struct css2600_bus_driver css2600_psys_driver = {
	.probe = css2600_psys_probe,
	.remove = css2600_psys_remove,
	.isr = psys_isr,
	.wanted = CSS2600_PSYS_NAME,
	.drv = {
		.name = CSS2600_PSYS_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init css2600_psys_init(void)
{
	int rval = alloc_chrdev_region(&css2600_psys_dev_t, 0,
				   CSS2600_PSYS_NUM_DEVICES, CSS2600_PSYS_NAME);
	if (rval) {
		pr_err("can't alloc css2600 psys chrdev region (%d)\n", rval);
		return rval;
	}

	rval = bus_register(&css2600_psys_bus);
	if (rval) {
		pr_warn("can't register css2600 psys bus (%d)\n", rval);
		goto out_bus_register;
	}

	css2600_bus_register_driver(&css2600_psys_driver);

	return rval;

out_bus_register:
	unregister_chrdev_region(css2600_psys_dev_t, CSS2600_PSYS_NUM_DEVICES);

	return rval;
}

static void __exit css2600_psys_exit(void)
{
	css2600_bus_unregister_driver(&css2600_psys_driver);
	bus_unregister(&css2600_psys_bus);
	unregister_chrdev_region(css2600_psys_dev_t, CSS2600_PSYS_NUM_DEVICES);
}

module_init(css2600_psys_init);
module_exit(css2600_psys_exit);

MODULE_AUTHOR("Sakari Ailus <sakari.ailus@linux.intel.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Intel css2600 processing system driver");
