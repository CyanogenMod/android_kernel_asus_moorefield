/*
 * Copyright (c) 2013 Intel Corporation. All Rights Reserved.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/wait.h>
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#include "css2600-psys-stub.h"
#include "css2600-psys.h"
#include "psysstub.h"

#define CSS2600_NUM_DEVICES	4

static dev_t css2600_dev_t;
static DECLARE_BITMAP(css2600_devices, CSS2600_NUM_DEVICES);
static DEFINE_MUTEX(css2600_devices_mutex);
static struct platform_device *css2600_platform_device;

static struct bus_type css2600_psys_bus = {
	.name = CSS2600_PSYS_STUB_NAME,
};

static const struct css2600_capability caps = {
	.version = 1,
	.driver = "css2600-psys-stub",
};

static inline bool is_priority_valid(uint32_t priority)
{
	return priority < CSS2600_CMD_PRIORITY_NUM;
}

static inline bool is_id_valid(int32_t id)
{
	return id == CSS2600_PSYS_STUB_PREVIEW ||
	       id == CSS2600_PSYS_STUB_VIDEO ||
	       id == CSS2600_PSYS_STUB_CAPTURE;
}

static int css2600_queue_event(struct css2600_fh *fh, struct css2600_event *e)
{
	struct css2600_device *isp = device_to_css2600_device(fh->dev);
	struct css2600_eventq *eventq;
	struct css2600_event *ev;

	eventq = kzalloc(sizeof(*eventq), GFP_KERNEL);
	if (!eventq)
		return -ENOMEM;
	ev = kzalloc(sizeof(*ev), GFP_KERNEL);
	if (!ev)
		goto out;
	*ev = *e;
	eventq->ev = ev;
	dev_dbg(fh->dev, "queue event %u (%p)\n", ev->type, ev);

	mutex_lock(&isp->mutex);
	list_add_tail(&eventq->list, &fh->eventq);
	mutex_unlock(&isp->mutex);

	wake_up_interruptible(&fh->wait);
	return 0;
out:
	kfree(eventq);
	return -ENOMEM;
}

/**
 * This should be moved to a separate file as this part will be
 * replaced by the actual PSYS hardware API.
 */
static int psysstub_validate_buffers(struct css2600_buffer *buffer,
				     int bufcount)
{
	struct psysparam *params;
	int err = 0;

	if (bufcount < CSS2600_PSYS_STUB_BUFS_MIN)
		return -EINVAL;

	/* get params */
	params = kzalloc(sizeof(*params), GFP_KERNEL);
	if (!params) {
		kfree(buffer);
		return -ENOMEM;
	}

	if (copy_from_user(params, buffer[CSS2600_PSYS_STUB_PARAMS_IDX].userptr,
				sizeof(*params))) {
		err = -EFAULT;
		goto out;
	}

	if (buffer[CSS2600_PSYS_STUB_VIDEO_INPUT_IDX].len <
	    params->input.bytesperline * params->input.height) {
		err = -EINVAL;
		goto out;
	}

	if (buffer[CSS2600_PSYS_STUB_VIDEO_OUTPUT_IDX].len <
	    params->output.bytesperline * params->output.height) {
		err = -EINVAL;
		goto out;
	}

out:
	kfree(params);
	return err;
}

static int psysstub_runisp(struct css2600_run_cmd *cmd,
			   struct css2600_device *isp)
{
	struct css2600_event ev;

	if (!cmd)
		return -EINVAL;

	switch(cmd->command.id) {
	case CSS2600_PSYS_STUB_PREVIEW:
		msleep(30);
		break;
	case CSS2600_PSYS_STUB_VIDEO:
		usleep_range(14800, 15000);
		break;
	case CSS2600_PSYS_STUB_CAPTURE:
		msleep(500);
		break;
	default:
		return 0;
	}

	mutex_lock(&isp->mutex);
	isp->cur_cmd = NULL;
	if (cmd->flags & CSS2600_STUB_CMD_CANCEL) {
		cmd->flags = 0;
		mutex_unlock(&isp->mutex);
		return 0;
	}
	if (cmd->flags & CSS2600_STUB_CMD_SUSPEND) {
		cmd->flags = 0;
		mutex_unlock(&isp->mutex);
		return -EINTR;
	}
	mutex_unlock(&isp->mutex);

	ev.type = CSS2600_EVENT_TYPE_CMD_COMPLETE;
	ev.ev.cmd_done.id = cmd->command.id;
	ev.ev.cmd_done.issue_id = cmd->command.issue_id;
	css2600_queue_event(cmd->fh, &ev);

	return 0;
}

static struct css2600_run_cmd *__psysstub_lookup_cmd(
		struct css2600_device *isp, struct css2600_command *command)
{
	struct css2600_run_cmd *cmd;
	int i;

	for(i = 0; i < CSS2600_CMD_PRIORITY_NUM; i++)
		list_for_each_entry(cmd, &isp->commands[i], list)
			if (cmd->command.issue_id == command->issue_id)
				return cmd;
	return 0;
}

static struct css2600_run_cmd *__psysstub_next_cmd(struct css2600_device *isp)
{
	struct css2600_run_cmd *cmd = NULL;
	int i;

	/* start from highest piority */
	for(i = 0; i < CSS2600_CMD_PRIORITY_NUM; i++) {
		if (list_empty(&isp->commands[i]))
			continue;
		cmd = list_first_entry(&isp->commands[i],
				struct css2600_run_cmd, list);
		list_del(&cmd->list);
		break;
	}
	if (!cmd)
		isp->queue_empty = true;
	return cmd;
}

static struct css2600_run_cmd *psysstub_next_cmd(struct css2600_device *isp)
{

	struct css2600_run_cmd *cmd;

	mutex_lock(&isp->mutex);
	cmd = __psysstub_next_cmd(isp);
	isp->cur_cmd = cmd;
	mutex_unlock(&isp->mutex);

	return cmd;
}

static void psysstub_run_cmd(struct work_struct *work)
{
	struct css2600_device *isp = container_of(work, struct css2600_device, run_cmd);
	struct css2600_run_cmd *cmd;

	while ((cmd = psysstub_next_cmd(isp)) != NULL) {
		if (psysstub_runisp(cmd, isp) == -EINTR)
			continue;
		kfree(cmd);
	}
}

static int css2600_open(struct inode *inode, struct file *file)
{
	struct css2600_device *isp = inode_to_css2600_device(inode);
	struct css2600_fh *fh;

	fh = kzalloc(sizeof(*fh), GFP_KERNEL);
	if (!fh)
		return -ENOMEM;

	init_waitqueue_head(&fh->wait);
	INIT_LIST_HEAD(&fh->bufmap);
	INIT_LIST_HEAD(&fh->eventq);

	fh->dev = &isp->dev;
	file->private_data = fh;

	mutex_lock(&isp->mutex);

	list_add(&fh->list, &isp->fhs);

	mutex_unlock(&isp->mutex);

	return 0;
}

static struct css2600_kbuffer *css2600_lookup_kbuffer(struct css2600_fh *fh, int fd)
{
	struct css2600_kbuffer *kbuffer = NULL;

	list_for_each_entry(kbuffer, &fh->bufmap, list) {
		if (kbuffer->fd == fd)
			return kbuffer;
	}

	return 0;
}

static long css2600_buffer_from_user(struct css2600_buffer *buf,
				     void __user *arg, int compat)
{
#ifdef CONFIG_COMPAT
	struct css2600_buffer32 __user *user_buffer = arg;
	compat_uptr_t ptr;

	if (compat) {
		if (!access_ok(VERIFY_READ, user_buffer,
			       sizeof(struct css2600_buffer32)) ||
		    get_user(buf->len, &user_buffer->len) ||
		    get_user(buf->fd, &user_buffer->fd) ||
		    get_user(buf->flags, &user_buffer->flags) ||
		    get_user(ptr, &user_buffer->userptr))
			return -EFAULT;

		buf->userptr = compat_ptr(ptr);

		return 0;
	}
#endif
	return copy_from_user(buf, arg, sizeof(*buf));
}

static long css2600_buffer_to_user(struct css2600_buffer *buf,
				   void __user *arg, int compat)
{
#ifdef CONFIG_COMPAT
	struct css2600_buffer32 __user *user_buffer = arg;
	compat_uptr_t ptr = ptr_to_compat(buf->userptr);

	if (compat) {
		if (!access_ok(VERIFY_WRITE, user_buffer,
			       sizeof(struct css2600_buffer32)) ||
		    put_user(buf->len, &user_buffer->len) ||
		    put_user(buf->fd, &user_buffer->fd) ||
		    put_user(buf->flags, &user_buffer->flags) ||
		    put_user(ptr, &user_buffer->userptr))
			return -EFAULT;

		return 0;
	}
#endif
	return copy_to_user(arg, buf, sizeof(*buf));
}

static long css2600_command_from_user(struct css2600_command *cmd,
					void __user *arg, int compat)
{
#ifdef CONFIG_COMPAT
	struct css2600_command32 __user *user_cmd = arg;
	compat_uptr_t ptr;

	if (compat) {
		if (!access_ok(VERIFY_READ, user_cmd,
				sizeof(struct css2600_command32)) ||
		    get_user(cmd->id, &user_cmd->id) ||
		    get_user(cmd->priority, &user_cmd->priority) ||
		    get_user(cmd->bufcount, &user_cmd->bufcount) ||
		    get_user(ptr, &user_cmd->buffers) ||
		    get_user(cmd->issue_id, &user_cmd->issue_id))
			return -EFAULT;

		/* Note: cmd->buffers remains as array of compat buffers */
		cmd->buffers = compat_ptr(ptr);

		return 0;
	}
#endif
	return copy_from_user(cmd, arg, sizeof(*cmd));
}

static struct css2600_buffer *css2600_command_buffers_from_user(
		struct css2600_command *cmd, int compat)
{
	struct css2600_buffer *buffers;
#ifdef CONFIG_COMPAT
	struct css2600_buffer32 *compat_buffers;
	int i;
#endif

	if (cmd->bufcount < CSS2600_PSYS_STUB_BUFS_MIN)
		return NULL;

	buffers = kzalloc(sizeof(*buffers) * cmd->bufcount, GFP_KERNEL);
	if (!buffers)
		return NULL;

#ifdef CONFIG_COMPAT
	if (compat) {
		compat_buffers =
			(struct css2600_buffer32 __user *) cmd->buffers;
		for (i = 0; i < cmd->bufcount; i++) {
			if (!access_ok(VERIFY_READ, &compat_buffers[i],
					sizeof(struct css2600_buffer32))) {
				kfree(buffers);
				return NULL;
			}
			if (css2600_buffer_from_user(&buffers[i],
						     &compat_buffers[i],
						     compat)) {
				kfree(buffers);
				return NULL;
			}
		}
		return buffers;
	}
#endif

	if (copy_from_user(buffers, cmd->buffers,
			   sizeof(*buffers) * cmd->bufcount)) {
		kfree(buffers);
		return NULL;
	}

	return buffers;
}

static long css2600_ioctl_mapbuf(struct file *file,
				 void __user *arg, int compat)
{
	struct css2600_fh *fh = file->private_data;
	struct css2600_kbuffer *kbuffer;
	struct css2600_buffer buf;

	if (css2600_buffer_from_user(&buf, arg, compat))
		return -EFAULT;

	kbuffer = css2600_lookup_kbuffer(fh, buf.fd);
	if (!kbuffer)
		return -EINVAL;
	if (kbuffer->mapped)
		return -EINVAL;
	buf.flags |= CSS2600_BUFFER_FLAG_MAPPED;
	if (css2600_buffer_to_user(&buf, arg, compat)) {
		kfree(kbuffer);
		return -EFAULT;
	}
	kbuffer->mapped = 1;
	dev_dbg(fh->dev, "IOC_MAPBUF: mapped fd %d\n", kbuffer->fd);

	return 0;
}

static long css2600_ioctl_unmapbuf(struct file *file,
				   void __user *arg, int compat)
{
	struct css2600_fh *fh = file->private_data;
	struct css2600_kbuffer *kbuffer;
	struct css2600_buffer buf;

	if (css2600_buffer_from_user(&buf, arg, compat))
		return -EFAULT;
	kbuffer = css2600_lookup_kbuffer(fh, buf.fd);
	if (!kbuffer)
		return -EINVAL;
	if (!kbuffer->mapped)
		return -EINVAL;
	buf.flags &= ~CSS2600_BUFFER_FLAG_MAPPED;
	if (css2600_buffer_to_user(&buf, arg, compat)) {
		kfree(kbuffer);
		return -EFAULT;
	}
	kbuffer->mapped = 0;
	dev_dbg(fh->dev, "IOC_UNMAPBUF: fd %d\n", kbuffer->fd);

	return 0;
}

static long css2600_ioctl_getbuf(struct file *file,
				 void __user *arg, int compat)
{
	struct css2600_fh *fh = file->private_data;
	struct css2600_kbuffer *kbuffer;
	struct css2600_buffer buffer;

	if (css2600_buffer_from_user(&buffer, arg, compat))
		return -EFAULT;
	kbuffer = kzalloc(sizeof(*kbuffer), GFP_KERNEL);
	if (!kbuffer)
		return -ENOMEM;
	/* TODO: allocate with DMABUF */
	kbuffer->fd = (int)kbuffer;
	kbuffer->userptr = buffer.userptr;
	buffer.fd = kbuffer->fd;
	if (css2600_buffer_to_user(&buffer, arg, compat)) {
		kfree(kbuffer);
		return -EFAULT;
	}
	list_add_tail(&kbuffer->list, &fh->bufmap);
	dev_dbg(fh->dev, "IOC_GETBUF: userptr %p to %d\n",
		buffer.userptr, buffer.fd);
	return 0;
}

static long css2600_ioctl_putbuf(struct file *file,
				 void __user *arg, int compat)
{
	struct css2600_fh *fh = file->private_data;
	struct css2600_kbuffer *kbuffer = NULL;
	struct css2600_buffer buffer;

	if (css2600_buffer_from_user(&buffer, arg, compat))
		return -EFAULT;
	kbuffer = css2600_lookup_kbuffer(fh, buffer.fd);
	if (!kbuffer)
		return -EINVAL;
	list_del(&kbuffer->list);
	kfree(kbuffer);
	dev_dbg(fh->dev, "IOC_PUTBUF: buffer %d\n", buffer.fd);

	return 0;
}

static long css2600_ioctl_qcmd(struct file *file,
			       struct css2600_command __user *arg,
			       int compat)
{
	struct css2600_fh *fh = file->private_data;
	struct css2600_device *isp = device_to_css2600_device(fh->dev);
	struct css2600_run_cmd *cmd;
	struct css2600_run_cmd *cur_cmd = isp->cur_cmd;
	struct css2600_buffer *buffers = NULL;
	int err;

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;

	if (css2600_command_from_user(&cmd->command, arg, compat)) {
		err = -EFAULT;
		goto error;
	}

	dev_dbg(fh->dev, "IOC_QCMD: length %u\n", cmd->command.bufcount);

	buffers = css2600_command_buffers_from_user(&cmd->command, compat);
	if (!buffers) {
		err = -EFAULT;
		goto error;
	}

	if (!is_priority_valid(cmd->command.priority) ||
	    !is_id_valid(cmd->command.id) ||
	    psysstub_validate_buffers(buffers, cmd->command.bufcount)
	    ) {
		err = -EINVAL;
		goto error;
	}
	cmd->fh = fh;

	if (cur_cmd && cur_cmd->command.priority > cmd->command.priority) {
		cur_cmd->flags |= CSS2600_STUB_CMD_SUSPEND;
		list_add(&cur_cmd->list, &isp->commands[cur_cmd->command.priority]);
		isp->cur_cmd = NULL;
		list_add(&cmd->list, &isp->commands[cmd->command.priority]);
	} else {
		list_add_tail(&cmd->list,
			      &isp->commands[cmd->command.priority]);
	}

	if (isp->queue_empty) {
		queue_work(isp->run_cmd_queue, &isp->run_cmd);
		isp->queue_empty = false;
	}

	kfree(buffers);
	return 0;
error:
	kfree(buffers);
	kfree(cmd);
	return err;
}

static long css2600_ioctl_dqevent(struct file *file, struct css2600_event __user *arg)
{
	struct css2600_fh *fh = file->private_data;
	struct css2600_eventq *evq;
	struct css2600_event *ev;

	dev_dbg(fh->dev, "IOC_DQEVENT\n");

	/* TODO: eventq accesses must be serialized */
	/* TODO: should be able to block on this */

	if (list_empty(&fh->eventq))
		return -EINVAL;

	evq = list_first_entry(&fh->eventq, struct css2600_eventq, list);
	ev = evq->ev;

	if (copy_to_user(arg, ev, sizeof *ev))
		return -EFAULT;

	list_del(&evq->list);
	kfree(evq);
	kfree(ev);

	return 0;
}

static long css2600_ioctl_cmd_cancel(struct file *file,
				     void __user *arg, int compat)
{
	struct css2600_fh *fh = file->private_data;
	struct css2600_device *isp = device_to_css2600_device(fh->dev);
	struct css2600_run_cmd *cmd;
	struct css2600_command command;

	if (css2600_command_from_user(&command, arg, compat))
		return -EFAULT;

	if (isp->cur_cmd &&
	    command.issue_id == isp->cur_cmd->command.issue_id) {
		isp->cur_cmd->flags |= CSS2600_STUB_CMD_CANCEL;
		isp->cur_cmd = NULL;
	} else {
		cmd = __psysstub_lookup_cmd(isp, &command);
		if (!cmd)
			return -EINVAL;

		list_del(&cmd->list);
		kfree(cmd);
	}

	return 0;
}

#ifdef CONFIG_COMPAT
static long css2600_compat_ioctl32(struct file *file,
				   unsigned int cmd,
				   unsigned long arg)
{
	int err = 0;
	struct css2600_fh *fh = file->private_data;
	void __user *argp = compat_ptr(arg);

	dev_dbg(fh->dev, "compat ioctl nr %d\n", _IOC_NR(cmd));

	switch (cmd) {
	case CSS2600_IOC_QUERYCAP:
		err = copy_to_user(argp, &caps, sizeof(caps));
		break;
	case CSS2600_IOC_MAPBUF32:
		err = css2600_ioctl_mapbuf(file, argp, 1);
		break;
	case CSS2600_IOC_UNMAPBUF32:
		err = css2600_ioctl_unmapbuf(file, argp, 1);
		break;
	case CSS2600_IOC_GETBUF32:
		err = css2600_ioctl_getbuf(file, argp, 1);
		break;
	case CSS2600_IOC_PUTBUF32:
		err = css2600_ioctl_putbuf(file, argp, 1);
		break;
	case CSS2600_IOC_QCMD32:
		err = css2600_ioctl_qcmd(file, argp, 1);
		break;
	case CSS2600_IOC_DQEVENT:
		err = css2600_ioctl_dqevent(file, argp);
		break;
	case CSS2600_IOC_CMD_CANCEL32:
		err = css2600_ioctl_cmd_cancel(file, argp, 1);
		break;
	default:
		err = -ENOTTY;
	}

	return err;
}
#endif

static long css2600_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	struct css2600_fh *fh = file->private_data;
	struct css2600_device *isp = device_to_css2600_device(fh->dev);
	void __user *argp = (void __user*)arg;

	mutex_lock(&isp->mutex);
	switch (cmd) {
	case CSS2600_IOC_QUERYCAP:
		err = copy_to_user(argp, &caps, sizeof caps);
		break;
	case CSS2600_IOC_MAPBUF:
		err = css2600_ioctl_mapbuf(file, argp, 0);
		break;
	case CSS2600_IOC_UNMAPBUF:
		err = css2600_ioctl_unmapbuf(file, argp, 0);
		break;
	case CSS2600_IOC_GETBUF:
		err = css2600_ioctl_getbuf(file, argp, 0);
		break;
	case CSS2600_IOC_PUTBUF:
		err = css2600_ioctl_putbuf(file, argp, 0);
		break;
	case CSS2600_IOC_QCMD:
		err = css2600_ioctl_qcmd(file, argp, 0);
		break;
	case CSS2600_IOC_DQEVENT:
		err = css2600_ioctl_dqevent(file, argp);
		break;
	case CSS2600_IOC_CMD_CANCEL:
		err = css2600_ioctl_cmd_cancel(file, argp, 0);
		break;
	default:
		err = -ENOTTY;
	}
	mutex_unlock(&isp->mutex);
	return err;
}

static unsigned int css2600_poll(struct file *file, struct poll_table_struct *wait)
{
	struct css2600_fh *fh = file->private_data;
	struct css2600_device *isp = device_to_css2600_device(fh->dev);
	unsigned int res = 0;

	dev_dbg(fh->dev, "css2600 poll\n");

	poll_wait(file, &fh->wait, wait);

	mutex_lock(&isp->mutex);
	if (!list_empty(&fh->eventq))
		res = POLLIN;
	mutex_unlock(&isp->mutex);

	dev_dbg(fh->dev, "css2600 poll res %u\n", res);

	return res;
}


static int css2600_release(struct inode *inode, struct file *file)
{
	struct css2600_device *isp = inode_to_css2600_device(inode);
	struct css2600_fh *fh = file->private_data;
	struct css2600_kbuffer *bm, *bm0;
	struct css2600_run_cmd *cmd, *cmd0;
	int i;

	cancel_work_sync(&isp->run_cmd);

	mutex_lock(&isp->mutex);
	list_del(&fh->list);

	for (i = 0; i < CSS2600_CMD_PRIORITY_NUM; i++)
		list_for_each_entry_safe(cmd, cmd0, &isp->commands[i], list) {
			list_del(&cmd->list);
			kfree(cmd);
		}

	list_for_each_entry_safe(bm, bm0, &fh->bufmap, list) {
		list_del(&bm->list);
		kfree(bm);
	}
	mutex_unlock(&isp->mutex);
	kfree(fh);

	return 0;
}

static const struct file_operations css2600_fops = {
	.open = css2600_open,
	.release = css2600_release,
	.unlocked_ioctl = css2600_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = css2600_compat_ioctl32,
#endif
	.poll = css2600_poll,
	.owner = THIS_MODULE,
};

void css2600_dev_release(struct device *dev)
{
}

static int css2600_platform_probe(struct platform_device *pdev)
{
	struct css2600_device *isp;
	unsigned int minor;
	int i;
	int rval = -E2BIG;

	mutex_lock(&css2600_devices_mutex);

	minor = find_next_zero_bit(css2600_devices, CSS2600_NUM_DEVICES, 0);
	if (minor == CSS2600_NUM_DEVICES) {
		dev_err(&pdev->dev, "too many devices\n");
		goto out_unlock;
	}

	isp = devm_kzalloc(&pdev->dev, sizeof(*isp), GFP_KERNEL);
	if (!isp) {
		dev_err(&pdev->dev, "Failed to alloc CI ISP structure\n");
		rval = -ENOMEM;
		goto out_unlock;
	}

	cdev_init(&isp->cdev, &css2600_fops);
	isp->cdev.owner = css2600_fops.owner;

	rval = cdev_add(&isp->cdev, MKDEV(MAJOR(css2600_dev_t), minor), 1);
	if (rval) {
		dev_err(&pdev->dev, "cdev_add failed (%d)\n", rval);
		goto out_unlock;
	}

	set_bit(minor, css2600_devices);

	isp->dev.bus = &css2600_psys_bus;
	isp->dev.devt = MKDEV(MAJOR(css2600_dev_t), minor);
	isp->dev.release = css2600_dev_release;
	dev_set_name(&isp->dev, CSS2600_PSYS_STUB_NAME "%d", minor);
	rval = device_register(&isp->dev);
	if (rval < 0) {
		dev_err(&isp->dev, "device_register failed\n");
		goto out_unlock;
	}

	platform_set_drvdata(pdev, isp);

	isp->run_cmd_queue = alloc_workqueue(caps.driver, WQ_UNBOUND, 1);
	if (isp->run_cmd_queue == NULL) {
		dev_err(&isp->dev, "Failed to initialize workq\n");
		rval = -ENOMEM;
		goto out_unlock;
	}
	INIT_WORK(&isp->run_cmd, psysstub_run_cmd);
	isp->queue_empty = true;

	for (i = 0; i < CSS2600_CMD_PRIORITY_NUM; i++)
		INIT_LIST_HEAD(&isp->commands[i]);

	mutex_unlock(&css2600_devices_mutex);
	mutex_init(&isp->mutex);
	INIT_LIST_HEAD(&isp->fhs);

	return 0;

out_unlock:
	mutex_unlock(&css2600_devices_mutex);

	return rval;
}

static int css2600_platform_remove(struct platform_device *pdev)
{
	struct css2600_device *isp = platform_get_drvdata(pdev);

	mutex_lock(&css2600_devices_mutex);

	if (isp->run_cmd_queue) {
		destroy_workqueue(isp->run_cmd_queue);
		isp->run_cmd_queue = NULL;
	}

	device_unregister(&isp->dev);

	clear_bit(MINOR(isp->cdev.dev), css2600_devices);
	cdev_del(&isp->cdev);

	mutex_unlock(&css2600_devices_mutex);

	mutex_destroy(&isp->mutex);

	return 0;
}

static struct platform_device_id css2600_psys_id_table[] = {
        { CSS2600_PSYS_STUB_NAME, 0 },
        { },
};

static struct platform_driver css2600_platform_driver = {
	.driver = {
#ifdef CONFIG_PM
		.pm = &(const struct dev_pm_ops){},
#endif /* CONFIG_PM */
		.name = CSS2600_PSYS_STUB_NAME,
		.owner = THIS_MODULE,
	},
	.probe = css2600_platform_probe,
	.remove = css2600_platform_remove,
	.id_table = css2600_psys_id_table,
};

static int __init css2600_init(void)
{
	int rval = alloc_chrdev_region(&css2600_dev_t, 0, CSS2600_NUM_DEVICES,
				       CSS2600_PSYS_STUB_NAME);

	if (rval) {
		pr_warn("can't allocate css2600 chrdev region (%d)\n", rval);
		return rval;
	}

	rval = bus_register(&css2600_psys_bus);
	if (rval) {
		pr_warn("can't register css2600 bus (%d)\n", rval);
		goto out_bus_register;
	}

	css2600_platform_device = platform_device_alloc(CSS2600_PSYS_STUB_NAME, -1);
	if (!css2600_platform_device) {
		pr_warn("can't allocate platform device\n");
		rval = -ENOMEM;
		goto out_bus_register;
	}

	rval = platform_device_add(css2600_platform_device);
	if (rval) {
		pr_warn("can't add pci driver (%d)\n", rval);
		goto out_platform_device_register;
	}

	rval = platform_driver_register(&css2600_platform_driver);
	if (rval) {
		pr_warn("can't register pci driver (%d)\n", rval);
		goto out_platform_driver_register;
	}

	return 0;

out_platform_driver_register:
	platform_device_unregister(css2600_platform_device);

out_platform_device_register:
	bus_unregister(&css2600_psys_bus);

out_bus_register:
	unregister_chrdev_region(css2600_dev_t, CSS2600_NUM_DEVICES);

	return rval;
}

static void __exit css2600_exit(void)
{
	platform_driver_unregister(&css2600_platform_driver);
	platform_device_unregister(css2600_platform_device);
	css2600_platform_device = NULL;
	bus_unregister(&css2600_psys_bus);
	unregister_chrdev_region(css2600_dev_t, CSS2600_NUM_DEVICES);
}

module_init(css2600_init);
module_exit(css2600_exit);

MODULE_AUTHOR("Sakari Ailus <sakari.ailus@intel.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Intel Atom ISP driver");
