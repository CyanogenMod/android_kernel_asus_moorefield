/*
 * logger_sventx.c - logger messages redirection to SVENTX
 *
 *  Copyright (C) Intel 2014
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
 * To enable redirection of logger messages to SVENTX, configure
 * 'out' parameter of 'logger_sventx' module in sysfs with one or
 * more values among (main, system, radio, kernel), e.g:
 *
 *  # echo "main,system" > /sys/module/logger_sventx/parameters/out
 *
 * Possible log buffers are : main, system, radio, kernel
 */

#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/sventx/sventx.h>
#include "logger.h"
#include "logger_kernel.h"

static psven_handle_t svenHandle;

static const sven_guid_t guid = {
	0x7953a961, 0xeb7a, 0x4d76, { 0x82, 0xd7, 0xe3, 0x1c, 0x40, 0xf1, 0x27, 0xb3 } };

#define TMP_BUF_SIZE 256

struct sventx_plugin {
	char *log_name;
	bool enabled;
	struct logger_plugin *plugin;
	u8 *tmp_buf;          /* tmp buffer to concat message segments */
	unsigned int tmp_len; /* accumulated length of segments in tmp buffer */
	struct list_head list;
};

static LIST_HEAD(plugin_list);

/**
 * @logger_sventx_init() - this callback function is called by logger.c
 * when a plug-in is added (via a call to logger_add_plugin)
 *
 * @cb_data: callback data for the plug-in (in our case it is a pointer
 *           to the sventx_plugin structure
 */
static void logger_sventx_init(void *cb_data)
{
	struct sventx_plugin *sventx_plugin = (struct sventx_plugin *)cb_data;

	if (unlikely(sventx_plugin == NULL))
		return;

	sventx_plugin->tmp_buf = kmalloc(TMP_BUF_SIZE, GFP_KERNEL);
	sventx_plugin->tmp_len = 0;
}

/**
 * @logger_sventx_exit() - this callback function is called by logger.c
 * when a plug-in is removed (via a call to logger_remove_plugin)
 *
 * @cb_data: callback data for the plug-in (in our case it is a pointer
 *           to the sventx_plugin structure
 */
static void logger_sventx_exit(void *cb_data)
{
	struct sventx_plugin *sventx_plugin = (struct sventx_plugin *)cb_data;

	if (unlikely(sventx_plugin == NULL))
		return;

	kfree(sventx_plugin->tmp_buf);
	sventx_plugin->tmp_buf = NULL;
	sventx_plugin->tmp_len = 0;
}

/**
 * @logger_sventx_write_seg() - this callback function is called by logger.c
 * when writing a segment of message (logger_aio_write)
 *
 * @buf:      data to be written (message segment)
 * @len:      length of the data to be written
 * @from_usr: true if data is from user-space
 * @som:      Start Of Message indication
 * @eom:      End Of Message indication
 * @cb_data:  callback data for the plug-in (in our case it is a pointer
 *            to the sventx_plugin structure
 */
static void logger_sventx_write_seg(void *buf, unsigned int len, bool from_usr,
				    bool som, bool eom, void *cb_data)
{
	struct sventx_plugin *sventx_plugin = (struct sventx_plugin *)cb_data;

	if (unlikely((sventx_plugin == NULL) ||
		     (sventx_plugin->tmp_buf == NULL)))
		return;

	/* We concatenate all the message segments in the tmp buffer
	 * until we get the End Of Message (EOM) indication.
	 * If the message length is longer than TMP_BUFFER_SIZE, then
	 * it is truncated.
	 */

	len = min(len, TMP_BUF_SIZE - 1 - sventx_plugin->tmp_len);
	if (len == 0)
		goto check_eom;

	if (from_usr) {
		if (copy_from_user(
			    &sventx_plugin->tmp_buf[sventx_plugin->tmp_len],
			    buf, len)) {
			len = 0;
			goto check_eom;
		}
	} else
		memcpy(&sventx_plugin->tmp_buf[sventx_plugin->tmp_len],
		       buf, len);

	sventx_plugin->tmp_len += len;

check_eom:
	if (eom) {
		SVEN_WRITE(svenHandle,
			   SVEN_SEVERITY_NORMAL,
			   0,
			   sventx_plugin->tmp_buf,
			   sventx_plugin->tmp_len);
		sventx_plugin->tmp_len = 0;
	}
}

/**
 * @logger_sventx_write_seg_recover() - this callback function is called
 * by logger.c when an issue is encountered while writing a segmented
 * message (logger_aio_write)
 *
 * @cb_data: callback data for the plug-in (in our case it is a pointer
 *           to the sventx_plugin structure
 */
static void logger_sventx_write_seg_recover(void *cb_data)
{
	struct sventx_plugin *sventx_plugin = (struct sventx_plugin *)cb_data;

	if (unlikely(sventx_plugin == NULL))
		return;

	/* Simply re-initialize the tmp buffer */
	sventx_plugin->tmp_len = 0;
}

/**
 * @create_sventx_plugin() - creates a @sventx_plugin for a given logger
 *
 * @name: logger's name
 */
static int create_sventx_plugin(const char *name)
{
	int ret = 0;
	struct logger_plugin *log_plugin;
	struct sventx_plugin *sventx_plugin;

	log_plugin = kzalloc(sizeof(struct logger_plugin), GFP_KERNEL);
	if (log_plugin == NULL)
		return -ENOMEM;

	sventx_plugin = kzalloc(sizeof(struct sventx_plugin), GFP_KERNEL);
	if (sventx_plugin == NULL) {
		ret = -ENOMEM;
		goto out_free_log_plugin;
	}

	sventx_plugin->log_name = kstrdup(name, GFP_KERNEL);
	sventx_plugin->enabled = false;
	sventx_plugin->plugin = log_plugin;

	log_plugin->init = logger_sventx_init;
	log_plugin->exit = logger_sventx_exit;
	log_plugin->write_seg = logger_sventx_write_seg;
	log_plugin->write_seg_recover = logger_sventx_write_seg_recover;
	log_plugin->data = (void *)sventx_plugin;

	list_add_tail(&sventx_plugin->list, &plugin_list);

	return 0;

out_free_log_plugin:
	kfree(log_plugin);
	return ret;
}

static int __init init_logger_sventx(void)
{
	int ret;

	/* SVEN handle initialisation */
	svenHandle = SVEN_ALLOC_HANDLE(NULL);

	if (svenHandle == NULL)
		return -EFAULT;

	SVEN_SET_HANDLE_GUID_UNIT(svenHandle, guid, 0);

	ret = create_sventx_plugin(LOGGER_LOG_RADIO);
	if (unlikely(ret))
		goto out;

	ret = create_sventx_plugin(LOGGER_LOG_SYSTEM);
	if (unlikely(ret))
		goto out;

	ret = create_sventx_plugin(LOGGER_LOG_MAIN);
	if (unlikely(ret))
		goto out;

	ret = create_sventx_plugin(LOGGER_LOG_KERNEL_BOT);
	if (unlikely(ret))
		goto out;

	return 0;

out:
	return ret;
}

static void __exit exit_logger_sventx(void)
{
	struct sventx_plugin *current_plugin, *next_plugin;

	list_for_each_entry_safe(current_plugin, next_plugin,
				 &plugin_list, list) {
		kfree(current_plugin->log_name);
		kfree(current_plugin->plugin);
		kfree(current_plugin->tmp_buf);
		list_del(&current_plugin->list);
		kfree(current_plugin);
	}

	/* clean up SVEN handle */
	if (svenHandle != 0)
		SVEN_DELETE_HANDLE(svenHandle);
}

module_init(init_logger_sventx)
module_exit(exit_logger_sventx)

/*
 * set_out - 'out' parameter set function from 'logger_sventx' module
 *
 * called when writing to 'out' parameter from 'logger_sventx' module in sysfs
 */
static int set_out(const char *val, struct kernel_param *kp)
{
	const char *name;
	struct sventx_plugin *plugin;

	list_for_each_entry(plugin, &plugin_list, list) {
		name = plugin->log_name;

		/* remove "log_" in the log_name string */
		name += 4;

		/* hack: user asks for "kernel", but the
		 * plugin is actually associated to "kern_bot" logger
		 */
		if (!strcmp(name, "kern_bot"))
			name = "kernel";

		if (strstr(val, name)) {
			if (plugin->enabled == false) {
				logger_add_plugin(plugin->plugin,
						  plugin->log_name);
				plugin->enabled = true;
			}
		} else if (plugin->enabled == true) {
			logger_remove_plugin(plugin->plugin, plugin->log_name);
			plugin->enabled = false;
		}
	}

	return 0;
}

/*
 * get_out - 'out' parameter get function from 'logger_sventx' module
 *
 * called when reading 'out' parameter from 'logger_sventx' module in sysfs
 */
static int get_out(char *buffer, struct kernel_param *kp)
{
	const char *name;
	const char *k = ",";
	struct sventx_plugin *plugin;

	list_for_each_entry(plugin, &plugin_list, list) {
		if (plugin->enabled == true) {
			name = plugin->log_name;

			/* remove "log_" in the log_name string */
			name += 4;

			/* hack: if plugin is associated to "kern_bot" logger,
			 * user actually wants to see "kernel"
			 */
			if (!strcmp(name, "kern_bot"))
				name = "kernel";

			strcat(buffer, name);
			strcat(buffer, k);
		}
	}
	buffer[strlen(buffer)-1] = '\0';

	return strlen(buffer);
}

module_param_call(out, set_out, get_out, NULL, 0644);
MODULE_PARM_DESC(out, "configure logger to sventx [main|events|radio|system|kernel]");

