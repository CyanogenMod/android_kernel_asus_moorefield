#ifndef __LINUX_SHOW_SENSOR_PROCFS_H
#define __LINUX_SHOW_SENSOR_PROCFS_H

#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/types.h>

/* proc error status */
#define PROC_ERR 1 /* proc_create failed */

struct proc_dir_entry;

/* Create proc file */
int create_proc_file(const char *file_name, umode_t mode, struct proc_dir_entry *parent, const struct file_operations *proc_fops);

#endif
