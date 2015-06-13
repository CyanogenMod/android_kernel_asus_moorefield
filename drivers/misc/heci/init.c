/*
 * Initialization protocol for HECI driver
 *
 * Copyright (c) 2003-2014, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/export.h>
#include <linux/pci.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include "heci.h"
#include "heci_dev.h"
#include "hbm.h"
#include "client.h"
#include "utils.h"

#ifdef dev_dbg
#undef dev_dbg
#endif
#define dev_dbg dev_err

const char *heci_dev_state_str(int state)
{
#define HECI_DEV_STATE(state) case HECI_DEV_##state: return #state
	switch (state) {
	HECI_DEV_STATE(INITIALIZING);
	HECI_DEV_STATE(INIT_CLIENTS);
	HECI_DEV_STATE(ENABLED);
	HECI_DEV_STATE(RESETTING);
	HECI_DEV_STATE(DISABLED);
	HECI_DEV_STATE(POWER_DOWN);
	HECI_DEV_STATE(POWER_UP);
	default:
		return "unkown";
	}
#undef HECI_DEV_STATE
}
EXPORT_SYMBOL(heci_dev_state_str);

void heci_device_init(struct heci_device *dev)
{
	/* setup our list array */
	INIT_LIST_HEAD(&dev->file_list);
	INIT_LIST_HEAD(&dev->device_list);
	mutex_init(&dev->device_lock);
	init_waitqueue_head(&dev->wait_hw_ready);
	init_waitqueue_head(&dev->wait_recvd_msg);
	init_waitqueue_head(&dev->wait_dma_ready);
	dev->dev_state = HECI_DEV_INITIALIZING;

	/* We need to reserve something, because client #0
	 * is reserved for HECI bus messages
	 */
	bitmap_zero(dev->host_clients_map, HECI_CLIENTS_MAX);
	dev->open_handle_count = 0;

	/*
	 * Reserving the first three client IDs
	 * 0: Reserved for HECI Bus Message communications
	 * 1: Reserved for Watchdog
	 * 2: Reserved for AMTHI
	 */
	bitmap_set(dev->host_clients_map, 0, 3);

	heci_io_list_init(&dev->read_list);
	heci_io_list_init(&dev->write_list);
	heci_io_list_init(&dev->write_waiting_list);
	heci_io_list_init(&dev->ctrl_wr_list);
	heci_io_list_init(&dev->ctrl_rd_list);

	INIT_DELAYED_WORK(&dev->timer_work, heci_timer);
	INIT_WORK(&dev->init_work, heci_host_client_init);
}
EXPORT_SYMBOL_GPL(heci_device_init);

/**
 * heci_start - initializes host and fw to start work.
 *
 * @dev: the device structure
 *
 * returns 0 on success, <0 on failure.
 */
int heci_start(struct heci_device *dev)
{
	heci_hw_config(dev);

#ifdef FORCE_FW_INIT_RESET
	/* wait for FW-initiated reset flow, indefinitely */
	heci_hw_start(dev);
	heci_enable_interrupts(dev);
	timed_wait_for_timeout(WAIT_FOR_CONNECT_SLICE, dev->recvd_hw_ready, (2*HZ));
	/* Lock only after FW-reset flow worked or failed.
	 * Otherwise interrupts BH will be locked
	 */
	mutex_lock(&dev->device_lock);
	if (dev->recvd_hw_ready)
		goto reset_done;
#else
	mutex_lock(&dev->device_lock);
#endif

	/* acknowledge interrupt and stop interupts */
	heci_clear_interrupts(dev);
	dev_dbg(&dev->pdev->dev, "reset in start the heci device.\n");
	heci_reset(dev, 1);

reset_done:
	if (heci_hbm_start_wait(dev)) {
		dev_err(&dev->pdev->dev, "HBM haven't started");
		goto err;
	}

	if (!heci_host_is_ready(dev)) {
		dev_err(&dev->pdev->dev, "host is not ready.\n");
		goto err;
	}

	if (!heci_hw_is_ready(dev)) {
		dev_err(&dev->pdev->dev, "ME is not ready.\n");
		goto err;
	}

	/*if (dev->version.major_version != HBM_MAJOR_VERSION ||
	    dev->version.minor_version != HBM_MINOR_VERSION) {
		dev_dbg(&dev->pdev->dev, "HECI start failed.\n");
		goto err;
	}*/

	dev_dbg(&dev->pdev->dev, "link layer has been established.\n");

	mutex_unlock(&dev->device_lock);
	return 0;
err:
	dev_err(&dev->pdev->dev, "link layer initialization failed.\n");
	dev->dev_state = HECI_DEV_DISABLED;
	mutex_unlock(&dev->device_lock);
	return -ENODEV;
}
EXPORT_SYMBOL_GPL(heci_start);

/**
 * heci_reset - resets host and fw.
 *
 * @dev: the device structure
 * @interrupts_enabled: if interrupt should be enabled after reset.
 */
void heci_reset(struct heci_device *dev, int interrupts_enabled)
{
	bool unexpected;
	int ret;

	unexpected = (dev->dev_state != HECI_DEV_INITIALIZING &&
			dev->dev_state != HECI_DEV_DISABLED &&
			dev->dev_state != HECI_DEV_POWER_DOWN &&
			dev->dev_state != HECI_DEV_POWER_UP);

	ret = heci_hw_reset(dev, interrupts_enabled);
	if (ret) {
		dev_err(&dev->pdev->dev, "hw reset failed disabling the device\n");
		interrupts_enabled = false;
		dev->dev_state = HECI_DEV_DISABLED;
	}

	dev->hbm_state = HECI_HBM_IDLE;

	if (dev->dev_state != HECI_DEV_INITIALIZING) {
		if (dev->dev_state != HECI_DEV_DISABLED &&
		    dev->dev_state != HECI_DEV_POWER_DOWN)
			dev->dev_state = HECI_DEV_RESETTING;

		heci_cl_all_disconnect(dev);
		memset(&dev->wr_ext_msg, 0, sizeof(dev->wr_ext_msg));
	}

	dev->me_clients_num = 0;
	dev->rd_msg_hdr = 0;

	if (unexpected)
		dev_warn(&dev->pdev->dev, "unexpected reset: dev_state = %s\n",
			 heci_dev_state_str(dev->dev_state));

	if (!interrupts_enabled) {
		dev_dbg(&dev->pdev->dev, "intr not enabled end of reset\n");
		return;
	}
	dev_dbg(&dev->pdev->dev, "before sending HOST start\n");
	ret = heci_hw_start(dev);
	if (ret) {
		dev_err(&dev->pdev->dev, "hw_start failed disabling the device\n");
		dev->dev_state = HECI_DEV_DISABLED;
		return;
	}

	dev_dbg(&dev->pdev->dev, "link is established start sending messages.\n");
	/* link is established * start sending messages.  */

	dev->dev_state = HECI_DEV_INIT_CLIENTS;
	dev->hbm_state = HECI_HBM_START;
	heci_hbm_start_req(dev);
	/* wake up all readings so they can be interrupted */
	heci_cl_all_read_wakeup(dev);

	/* remove all waiting requests */
	heci_cl_all_write_clear(dev);
}
EXPORT_SYMBOL_GPL(heci_reset);

void heci_stop(struct heci_device *dev)
{
	dev_dbg(&dev->pdev->dev, "stopping the device.\n");
	mutex_lock(&dev->device_lock);
	cancel_delayed_work(&dev->timer_work);
	dev->dev_state = HECI_DEV_POWER_DOWN;
	heci_reset(dev, 0);
	mutex_unlock(&dev->device_lock);
	flush_scheduled_work();
}
EXPORT_SYMBOL_GPL(heci_stop);

/**
 * heci_write_is_idle - check if there is pending write transaction
 *
 * @dev: the device structure
 * returns true if the writ queues are empty
 */
bool heci_write_is_idle(struct heci_device *dev)
{
	bool idle = (dev->wr_ext_msg.hdr.length == 0  &&
		list_empty(&dev->ctrl_wr_list.list) &&
		list_empty(&dev->write_list.list));

	dev_dbg(&dev->pdev->dev, "pm: is idle[%d] extra=%d, ctrl=%d write=%d\n",
		idle,
		dev->wr_ext_msg.hdr.length == 0,
		list_empty(&dev->ctrl_wr_list.list),
		list_empty(&dev->write_list.list));

	return idle;
}
EXPORT_SYMBOL_GPL(heci_write_is_idle);
