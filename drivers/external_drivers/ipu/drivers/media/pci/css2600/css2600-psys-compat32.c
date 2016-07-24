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

#include <linux/compat.h>
#include <linux/css2600-psys.h>
#include <linux/errno.h>
#include <linux/uaccess.h>

static long native_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = -ENOTTY;

	if (file->f_op->unlocked_ioctl)
		ret = file->f_op->unlocked_ioctl(file, cmd, arg);

	return ret;
}

struct css2600_psys_buffer32 {
	uint64_t len;
	compat_uptr_t userptr;
	int fd;
	uint32_t flags;
	uint32_t reserved[2];
} __packed;

struct css2600_psys_command32 {
	uint64_t issue_id;
	uint32_t id;
	uint32_t priority;
	compat_uptr_t pg_manifest;
	compat_uptr_t pg_params;
	compat_uptr_t buffers;
	uint32_t pg_manifest_size;
	uint32_t pg_params_size;
	uint32_t bufcount;
	uint32_t reserved[3];
} __packed;

static int get_css2600_psys_command32(struct css2600_psys_command *kp,
				      struct css2600_psys_command32 __user *up)
{
	compat_uptr_t pgm, pgp, bufs;

	if (!access_ok(VERIFY_READ, up, sizeof(struct css2600_psys_buffer32)) ||
	    get_user(kp->issue_id, &up->issue_id) ||
	    get_user(kp->id, &up->id) ||
	    get_user(kp->priority, &up->priority) ||
	    get_user(pgm, &up->pg_manifest) ||
	    get_user(pgp, &up->pg_params) ||
	    get_user(bufs, &up->buffers) ||
	    get_user(kp->pg_manifest_size, &up->pg_manifest_size) ||
	    get_user(kp->pg_params_size, &up->pg_params_size) ||
	    get_user(kp->bufcount, &up->bufcount))
		return -EFAULT;

	kp->pg_manifest = compat_ptr(pgm);
	kp->pg_params = compat_ptr(pgp);
	kp->buffers = compat_ptr(bufs);

	return 0;
}

static int get_css2600_psys_buffer32(struct css2600_psys_buffer *kp,
				     struct css2600_psys_buffer32 __user *up)
{
	compat_uptr_t ptr;

	if (!access_ok(VERIFY_READ, up, sizeof(struct css2600_psys_buffer32)) ||
	    get_user(kp->len, &up->len) ||
	    get_user(ptr, &up->userptr) ||
	    get_user(kp->fd, &up->fd) ||
	    get_user(kp->flags, &up->flags))
		return -EFAULT;

	kp->userptr = compat_ptr(ptr);

	return 0;
}

static int put_css2600_psys_buffer32(struct css2600_psys_buffer *kp,
				     struct css2600_psys_buffer32 __user *up)
{
	compat_uptr_t ptr = (u32)((unsigned long)kp->userptr);

	if (!access_ok(VERIFY_WRITE, up,
		       sizeof(struct css2600_psys_buffer32)) ||
	    put_user(kp->len, &up->len) ||
	    put_user(ptr, &up->userptr) ||
	    put_user(kp->fd, &up->fd) ||
	    put_user(kp->flags, &up->flags))
		return -EFAULT;

	return 0;
}

#define CSS2600_IOC_GETBUF32 _IOWR('A', 4, struct css2600_psys_buffer32)
#define CSS2600_IOC_PUTBUF32 _IOWR('A', 5, struct css2600_psys_buffer32)
#define CSS2600_IOC_QCMD32 _IOWR('A', 6, struct css2600_psys_command32)
#define CSS2600_IOC_CMD_CANCEL32 _IOWR('A', 8, struct css2600_psys_command32)

long css2600_psys_compat_ioctl32(struct file *file, unsigned int cmd,
					unsigned long arg)
{
	union {
		struct css2600_psys_buffer buf;
		struct css2600_psys_command cmd;
		struct css2600_psys_event ev;
	} karg;
	int compatible_arg = 1;
	int err = 0;
	void __user *up = compat_ptr(arg);

	switch (cmd) {
	case CSS2600_IOC_GETBUF32:
		cmd = CSS2600_IOC_GETBUF;
		break;
	case CSS2600_IOC_PUTBUF32:
		cmd = CSS2600_IOC_PUTBUF;
		break;
	case CSS2600_IOC_QCMD32:
		cmd = CSS2600_IOC_QCMD;
		break;
	}

	switch (cmd) {
	case CSS2600_IOC_GETBUF:
	case CSS2600_IOC_PUTBUF:
		err = get_css2600_psys_buffer32(&karg.buf, up);
		compatible_arg = 0;
		break;
	case CSS2600_IOC_QCMD:
		err = get_css2600_psys_command32(&karg.cmd, up);
		compatible_arg = 0;
		break;
	}
	if (err)
		return err;

	if (compatible_arg) {
		err = native_ioctl(file, cmd, (unsigned long)up);
	} else {
		mm_segment_t old_fs = get_fs();

		set_fs(KERNEL_DS);
		err = native_ioctl(file, cmd, (unsigned long)&karg);
		set_fs(old_fs);
	}

	if (err)
		return err;

	switch (cmd) {
	case CSS2600_IOC_GETBUF:
		err = put_css2600_psys_buffer32(&karg.buf, up);
		break;
	}
	return err;
}
EXPORT_SYMBOL_GPL(css2600_psys_compat_ioctl32);
