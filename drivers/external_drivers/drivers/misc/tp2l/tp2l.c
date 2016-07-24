/*
 *  tp2l.c - TP2L module to export tracepoint event data to PTI
 *
 *  Copyright (C) Intel 2013
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * TP2L stands for Tracepoint To Log. This module is responsible for doing
 * a binding between the tracepoint events (as defined in
 * Documentation/trace/tracepoints.txt) and Intel's proprietary trace
 * infrastrcuture.
 *
 * For each tracepoint event that is supported by TP2L
 * (see tp2l_includes.h), a new probe is created to export the event data
 * (ascii string defined by the event's TP_printk macro) to the STM block
 * for an output either on the PTI port or the USB.DvC.Trace interface.
 *
 * A debugfs is created (/sys/kernel/debug/tp2l) to handle the enabling of
 * the TP2L probe for each tracepoint event.
 *
 * Note that in the future this module could be improved so that:
 *  - it can support different outputs (not only STM PTI/USB.DvC.Trace)
 *  - it can support an output in binary format
 */


#include <linux/module.h>
#include <linux/slab.h>
#include <linux/pti.h>
#include <linux/debugfs.h>


/* Uncomment to compile in test mode */
#define TP2L_TEST

#ifdef TP2L_TEST
#include "tp2l_test.c"
#endif


static struct pti_masterchannel *mc;

/* List of tp2l_element objects (they are created in tp2l.h file) */
static struct list_head tp2l_list;

/* A tp2l_system_dir element is associated to each system directory
 * in the tp2l debugfs, e.g. /sys/kernel/debug/tp2l/events/my_system/
 * Each system includes one or several tracepoint events (list of
 * tp2l_event_file elements).
 */
struct tp2l_system_dir {
	struct list_head list;
	const char *name;
	struct dentry *dir;
	struct list_head event_files;
};

/* A tp2l_event_file element is associated to each tp2l element
 * (i.e. tracepoint event supported by tp2l) in the tp2l debugfs,
 * e.g. /sys/kernel/debug/tp2l/events/my_system/my_event/
 */
struct tp2l_event_file {
	struct list_head list;
	struct tp2l_element *elt;
	bool enabled;
};

/* This is the root dir in the tp2l debugfs, i.e. /sys/kernel/debug/tp2l */
static struct dentry *d_tp2l;

/* List of tp2l_system_dir elements (lists all the systems supported by tp2l) */
static struct list_head tp2l_systems_list;

/* Including tp2l_includes.h file with DEFINE_TP2L_PROBES being defined
 * results in creating all the tp2l elements and their associated probes
 */
#define DEFINE_TP2L_PROBES
#include "tp2l_includes.h"
#undef DEFINE_TP2L_PROBES

/* This method is used to enable/disable the tp2l probe for a given
 * tracepoint event.
 */
static int tp2l_event_enable_disable(struct tp2l_event_file *file,
				     int enable)
{
	if (!file->elt)
		return -EINVAL;

	if (enable && !file->enabled) {
		file->elt->reg_fn(file->elt->probe_fn, file->elt);
		file->enabled = true;
	} else if (!enable && file->enabled) {
		file->elt->unreg_fn(file->elt->probe_fn, file->elt);
		file->enabled = false;
	}

	return 0;
}

static int tp2l_generic_open(struct inode *inode, struct file *filp)
{
	filp->private_data = inode->i_private;
	return 0;
}

/* Write method that is associated to the 'enable' debugfs file
 * associated to each tp2l element in the tp2l debugfs, e.g.
 * /sys/kernel/debug/tp2l/events/my_system/my_event/enable
 *
 * Writing '0' (resp. '1') in this file disables (resp. enables) the
 * tp2l probe for the tracepoint event.
 */
static ssize_t event_enable_write(struct file *filp, const char __user *ubuf,
				  size_t cnt, loff_t *ppos)
{
	struct tp2l_event_file *event_file = filp->private_data;
	unsigned long val;
	int ret;

	if (!event_file)
		return -EINVAL;

	ret = kstrtoul_from_user(ubuf, cnt, 10, &val);
	if (ret)
		return ret;

	switch (val) {
	case 0:
	case 1:
		ret = tp2l_event_enable_disable(event_file, val);
		break;
	default:
		return -EINVAL;
	}

	*ppos += cnt;

	return ret ? ret : cnt;
}

/* read method that is associated to the 'enable' file associated
 * to each tp2l element in the tp2l debugfs, e.g.
 * /sys/kernel/debug/tp2l/events/my_system/my_event/enable
 *
 * Reading '0' (resp. '1') means that the tp2l probe for the tracepoint
 * event is disabled (resp. enabled).
 */
static ssize_t event_enable_read(struct file *filp, char __user *ubuf,
				  size_t cnt, loff_t *ppos)
{
	struct tp2l_event_file *event_file = filp->private_data;
	char *buf;

	if (!event_file)
		return -EINVAL;

	if (event_file->enabled)
		buf = "1\n";
	else
		buf = "0\n";

	return simple_read_from_buffer(ubuf, cnt, ppos, buf, strlen(buf));
}

/* Write method that is associated to the 'enable' debugfs file
 * located in each system directory in the tp2l debugfs, e.g.
 * /sys/kernel/debug/tp2l/events/my_system/enable
 *
 * Writing '0' (resp. '1') in this file disables (resp. enables) the
 * tp2l probe for the all the tracepoint events of the system.
 */
static ssize_t system_enable_write(struct file *filp, const char __user *ubuf,
				  size_t cnt, loff_t *ppos)
{
	struct tp2l_system_dir *system_dir = filp->private_data;
	struct tp2l_event_file *event_file;
	unsigned long val;
	int ret;

	if (!system_dir)
		return -EINVAL;

	ret = kstrtoul_from_user(ubuf, cnt, 10, &val);
	if (ret)
		return ret;

	if ((val != 0) && (val != 1))
		return -EINVAL;

	list_for_each_entry(event_file, &system_dir->event_files, list) {
		ret = tp2l_event_enable_disable(event_file, val);
		if (ret)
			return ret;
	}

	*ppos += cnt;

	return cnt;
}

/* read method that is associated to the 'enable' debugfs file
 * located in each system directory in the tp2l debugfs, e.g.
 * /sys/kernel/debug/tp2l/events/my_system/enable
 *
 * Reading '0' (resp. '1') means that the tp2l probe of all the
 * tracepoint events of the system are disabled (resp. enabled).
 */
static ssize_t system_enable_read(struct file *filp, char __user *ubuf,
				  size_t cnt, loff_t *ppos)
{
	const char set_to_char[4] = { '?', '0', '1', 'X' };
	struct tp2l_system_dir *system_dir = filp->private_data;
	struct tp2l_event_file *event_file;
	char buf[2];
	int set = 0;

	if (!system_dir)
		return -EINVAL;

	list_for_each_entry(event_file, &system_dir->event_files, list) {
		set |= (1 << !!(event_file->enabled));
		if (set == 3)
			break;
	}

	buf[0] = set_to_char[set];
	buf[1] = '\n';

	return simple_read_from_buffer(ubuf, cnt, ppos, buf, 2);
}

/* Write method that is associated to the 'enable' debugfs file
 * located in the 'events'directory, i.e.
 * /sys/kernel/debug/tp2l/events/enable
 *
 * Writing '0' (resp. '1') in this file disables (resp. enables) the
 * tp2l probe of all the tracepoint events that are supported by tp2l
 */
static ssize_t tp2l_enable_write(struct file *filp, const char __user *ubuf,
				 size_t cnt, loff_t *ppos)
{
	struct tp2l_system_dir *system_dir;
	struct tp2l_event_file *event_file;
	unsigned long val;
	int ret;

	ret = kstrtoul_from_user(ubuf, cnt, 10, &val);
	if (ret)
		return ret;

	if ((val != 0) && (val != 1))
		return -EINVAL;

	list_for_each_entry(system_dir, &tp2l_systems_list, list) {
		list_for_each_entry(event_file, &system_dir->event_files,
				    list) {
			ret = tp2l_event_enable_disable(event_file, val);
			if (ret)
				return ret;
		}
	}

	*ppos += cnt;

	return cnt;
}

/* read method that is associated to the 'enable' debugfs file
 * located in the 'events'directory, i.e.
 * /sys/kernel/debug/tp2l/events/enable
 *
 * Reading '0' (resp. '1') means that the tp2l probe of all the
 * tracepoint events supported by tp2l are disabled (resp. enabled).
 */
static ssize_t tp2l_enable_read(struct file *filp, char __user *ubuf,
				size_t cnt, loff_t *ppos)
{
	const char set_to_char[4] = { '?', '0', '1', 'X' };
	struct tp2l_system_dir *system_dir;
	struct tp2l_event_file *event_file;
	char buf[2];
	int set = 0;

	list_for_each_entry(system_dir, &tp2l_systems_list, list) {
		list_for_each_entry(event_file, &system_dir->event_files,
				    list) {
			set |= (1 << !!(event_file->enabled));
			if (set == 3)
				break;
		}
	}

	buf[0] = set_to_char[set];
	buf[1] = '\n';

	return simple_read_from_buffer(ubuf, cnt, ppos, buf, 2);
}

static const struct file_operations tp2l_enable_fops = {
	.open = tp2l_generic_open,
	.read = tp2l_enable_read,
	.write = tp2l_enable_write,
};

static const struct file_operations system_enable_fops = {
	.open = tp2l_generic_open,
	.read = system_enable_read,
	.write = system_enable_write,
};

static const struct file_operations event_enable_fops = {
	.open = tp2l_generic_open,
	.read = event_enable_read,
	.write = event_enable_write,
};

/* Get the system directory in debugfs for a given tp2l element,
 * or create a system directory if not already existing.
 */
static struct tp2l_system_dir *tp2l_system_dir(struct dentry *parent,
					       struct tp2l_element *elt)
{
	struct dentry *dir;
	struct tp2l_system_dir *system;

	/* Check if this dir has already been created */
	list_for_each_entry(system, &tp2l_systems_list, list)
		if (strcmp(system->name, elt->system) == 0)
			return system;

	/* If not, create a new system dir */
	system = kmalloc(sizeof(*system), GFP_KERNEL);
	if (!system)
		goto out_fail;

	dir = debugfs_create_dir(elt->system, parent);
	if (!dir)
		goto out_free;

	system->name = elt->system;
	system->dir = dir;
	INIT_LIST_HEAD(&system->event_files);
	list_add_tail(&system->list, &tp2l_systems_list);

	if (!debugfs_create_file("enable", 0644, dir, system,
				 &system_enable_fops))
		pr_warn("Could not create debugfs '%s/enable' entry\n",
			   elt->system);

	return system;

out_free:
	kfree(dir);
out_fail:
	pr_warn("No memory to create event system '%s'",
		   elt->system);
	return NULL;
}

/* Create the tp2l debugfs */
static int tp2l_create_dirs(void)
{
	struct dentry *d_events, *d_system, *d_event;
	struct tp2l_system_dir *system_dir;
	struct tp2l_event_file *event_file;
	struct tp2l_element *elt;

	d_tp2l = debugfs_create_dir("tp2l", NULL);

	d_events = debugfs_create_dir("events", d_tp2l);

	/* Create the debugfs directory(ies) required for
	 * each tp2l element.
	 */
	list_for_each_entry(elt, &tp2l_list, list) {
		system_dir = tp2l_system_dir(d_events, elt);
		if ((!system_dir) || (!system_dir->dir))
			continue;

		d_system = system_dir->dir;
		d_event = debugfs_create_dir(elt->name, d_system);
		if (!d_event) {
			pr_warn("Could not create debugfs '%s' directory\n",
				   elt->name);
			continue;
		}

		event_file = kmalloc(sizeof(*event_file), GFP_KERNEL);
		if (!event_file) {
			pr_warn("No memory to create event '%s'",
				   elt->name);
			return -ENOMEM;
		}
		event_file->enabled = false;
		event_file->elt = elt;

		if (!debugfs_create_file("enable", 0644, d_event, event_file,
					 &event_enable_fops)) {
			pr_warn("Could not create debugfs '%s/enable' entry\n",
				elt->system);
			kfree(event_file);
			continue;
		}

		list_add_tail(&event_file->list, &system_dir->event_files);
	}

	if (!debugfs_create_file("enable", 0644, d_events, NULL,
				 &tp2l_enable_fops))
		pr_warn("Could not create debugfs '%s/enable' entry\n",
			   elt->system);

	return 0;
}

/* Clean-up the tp2l debugfs */
static void tp2l_remove_dirs(void)
{
	struct tp2l_system_dir *current_system, *next_system;
	struct tp2l_event_file *current_event, *next_event;

	list_for_each_entry_safe(current_system, next_system,
				 &tp2l_systems_list, list) {
		list_for_each_entry_safe(current_event, next_event,
					 &current_system->event_files, list) {
			list_del(&current_event->list);
			kfree(current_event);
		}
		list_del(&current_system->list);
		kfree(current_system);
	}

	debugfs_remove_recursive(d_tp2l);
}





static int __init tp2l_init(void)
{
	mc = pti_request_masterchannel(1, "tp2l");
	INIT_LIST_HEAD(&tp2l_list);
	INIT_LIST_HEAD(&tp2l_systems_list);

/* Including tp2l_includes.h file with ADD_TP2L_ELEMENT being defined
 * results in adding all the tp2l elements to the tp2l_list
 */
#define ADD_TP2L_ELEMENT
#include "tp2l_includes.h"
#undef ADD_TP2L_ELEMENT

	tp2l_create_dirs();

	return 0;
}

static void __exit tp2l_exit(void)
{
	pti_release_masterchannel(mc);

	tp2l_remove_dirs();
}

module_init(tp2l_init)
module_exit(tp2l_exit)


MODULE_LICENSE("GPL");
