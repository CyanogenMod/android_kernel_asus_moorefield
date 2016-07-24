/*
 * intel_fg_helper.c : Intel fuel gauging config helper
 *
 * (C) Copyright 2014 Intel Corporation
 * Author: Gopal, Saranya <saranya.gopal@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/atomic.h>

#define MAX_DATA_SIZE	256

static atomic_t fopen_count;
static bool is_fg_data_avail;
static void *fg_data;
static int num_bytes;

int (*intel_restore_data)(void *data, int len);
int (*intel_store_data)(void *data, int len);

/**
 * intel_fg_get_data - get FG config data
 * @data - config data output pointer
 * @len - length of config data
 */
int intel_fg_get_data(void *data, int len)
{
	if (!is_fg_data_avail)
		return -ENODATA;
	if (num_bytes < len)
		return -EINVAL;
	memcpy(data, fg_data, len);
	return 0;
}
EXPORT_SYMBOL(intel_fg_get_data);

/**
 * intel_fg_set_data - set FG config data
 * This function will set the config data to be stored
 * when the device file content is saved.
 * @data - config data input pointer
 * @len - length of the config data
 */
void intel_fg_set_data(void *data, int len)
{
	memcpy(fg_data, data, len);
}
EXPORT_SYMBOL(intel_fg_set_data);

/**
 * intel_fg_set_restore_fn - set the pointer to restore function.
 * The restore function set by this function will be called once FG
 * config data is written to the device file.
 * @restore_fn - pointer to restore function.
 */
void intel_fg_set_restore_fn(int (*restore_fn)(void *data, int len))
{
	intel_restore_data = restore_fn;
}
EXPORT_SYMBOL(intel_fg_set_restore_fn);

static int intel_fg_dev_file_open(struct inode *inode, struct file *file)
{
	if (atomic_read(&fopen_count))
		return -EBUSY;
	atomic_inc(&fopen_count);
	return 0;
}

/**
 * intel_fg_set_store_fn - set the pointer to store function
 * The store function set by this function will be called when
 * FG config data is read from the device file.
 * @store_fn - pointer to store function.
 */
void intel_fg_set_store_fn(int (*store_fn)(void *data, int len))
{
	intel_store_data = store_fn;
}
EXPORT_SYMBOL(intel_fg_set_store_fn);

static int intel_fg_dev_file_close(struct inode *inode, struct file *file)
{
	atomic_dec(&fopen_count);
	return 0;
}

static ssize_t intel_fg_dev_file_read(struct file *f, char __user *buf,
					size_t len, loff_t *off)
{
	if (!is_fg_data_avail)
		return -ENODATA;
	if (intel_store_data && !intel_store_data(fg_data, num_bytes))
		pr_info("FG data is updated");
	if (!copy_to_user(buf, fg_data, len))
		return len;
	return -EINVAL;
}

static ssize_t intel_fg_dev_file_write(struct file *f, const char __user *buf,
					size_t len, loff_t *off)
{
	if (copy_from_user(fg_data, buf, len))
		return -EINVAL;
	if (intel_restore_data && !intel_restore_data(fg_data, len))
		pr_info("FG data restored successfully");
	is_fg_data_avail = true;
	num_bytes = len;
	return len;
}

static const struct file_operations intel_fg_helper_fops = {
	.owner = THIS_MODULE,
	.open = &intel_fg_dev_file_open,
	.release = &intel_fg_dev_file_close,
	.read = &intel_fg_dev_file_read,
	.write = &intel_fg_dev_file_write,
};

static struct miscdevice fg_helper = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "intel_fg",
	.fops = &intel_fg_helper_fops,
};

static int __init fg_helper_init(void)
{
	fg_data = kzalloc(MAX_DATA_SIZE, GFP_KERNEL);
	intel_store_data = NULL;
	intel_restore_data = NULL;
	return misc_register(&fg_helper);
}
fs_initcall(fg_helper_init);

static void __exit fg_helper_exit(void)
{
	misc_deregister(&fg_helper);
}
module_exit(fg_helper_exit);

MODULE_AUTHOR("Saranya Gopal <saranya.gopal@intel.com>");
MODULE_DESCRIPTION("Fuel gauge config helper");
MODULE_LICENSE("GPL");
