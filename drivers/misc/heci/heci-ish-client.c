/*
 * HECI client drive for HID (ISH)
 *
 * Copyright (c) 2014, Intel Corporation.
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

#include <linux/device.h>
#include <linux/hid.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/kthread.h>
#include <linux/uuid.h>
#include "heci_dev.h"
#include "client.h"
#include "heci-hid.h"

/* Declaration will go to header file, probably*/
void set_ish_special_client(struct heci_cl *);
void unset_ish_special_client(void);
int ish_heci_cl_send(struct heci_cl *cl, u8 *buf, size_t length, bool blocking);
int ish_heci_cl_recv(struct heci_cl *cl, u8 *buf, size_t length);
int ish_heci_connect(void);
int heci_hid_probe(unsigned cur_hid_dev);
void heci_hid_remove(void);

/* Global vars, may eventually end up in a structure */
struct heci_cl	*ish_heci_cl;	/* ISH HECI client */

int ish_heci_client_found;	/* Set when ISH HECI client is successfully probed */
int may_send;			/* Global flag that determines if sender thread can safely send something or it should wait more */
int enum_devices_done;		/* Enum devices response complete flag */
int hid_descr_done;		/* Get HID descriptor complete flag */
int report_descr_done;		/* Get report descriptor complete flag */
int get_report_done;		/* Get Feature/Input report complete flag */

struct device_info *hid_devices;
unsigned cur_hid_dev;
unsigned hid_dev_count;
unsigned max_hid_devices = /*1*/ MAX_HID_DEVICES;
unsigned num_hid_devices;
unsigned char *hid_descr[MAX_HID_DEVICES];
int hid_descr_size[MAX_HID_DEVICES];
unsigned char *report_descr[MAX_HID_DEVICES];
int report_descr_size[MAX_HID_DEVICES];
struct hid_device *hid_sensor_hubs[MAX_HID_DEVICES];

/* HECI client driver structures and API for bus interface */
void process_recv(void *recv_buf, size_t len)
{
	struct hostif_msg *recv_msg = (struct hostif_msg *)recv_buf;
	unsigned char *payload;
	int rv;
	size_t	size;
	struct device_info *dev_info;
	int i;
	int data_len;
	int payload_len;
	int report_type;

	data_len = len;

	may_send = 0;
	do {
		if (data_len < sizeof(struct hostif_msg_hdr)) {
			pr_err("[hid-ish]: %s(): error, received %d less than data header %d\n",
				__func__, data_len, sizeof(struct hostif_msg_hdr));
			break;
		}

		payload = recv_buf + sizeof(struct hostif_msg_hdr);
		payload_len = data_len - sizeof(struct hostif_msg_hdr);

		switch (recv_msg->hdr.command & CMD_MASK) {
		default:
			break;

		case HOSTIF_DM_ENUM_DEVICES:
			hid_dev_count = (unsigned)*payload;
			hid_devices = kmalloc(hid_dev_count * sizeof(struct device_info), GFP_KERNEL);
			if (hid_devices)
				memset(hid_devices, 0, hid_dev_count * sizeof(struct device_info));

			for (i = 0; i < hid_dev_count; ++i) {
				if (1 + sizeof(struct device_info) * i >= data_len)
					break;

				dev_info = (struct device_info *)(payload + 1 + sizeof(struct device_info) * i);
				if (hid_devices)
					memcpy(hid_devices + i, dev_info, sizeof(struct device_info));
			}

			enum_devices_done = 1;
			break;

		case HOSTIF_GET_HID_DESCRIPTOR:
			hid_descr[cur_hid_dev] = kmalloc(payload_len, GFP_KERNEL);
			if (hid_descr[cur_hid_dev])
				memcpy(hid_descr[cur_hid_dev], payload, payload_len);
			hid_descr_size[cur_hid_dev] = payload_len;
			hid_descr_done = 1;
			break;

		case HOSTIF_GET_REPORT_DESCRIPTOR:
			report_descr[cur_hid_dev] = kmalloc(payload_len, GFP_KERNEL);
			if (report_descr[cur_hid_dev])
				memcpy(report_descr[cur_hid_dev], payload, payload_len);
			report_descr_size[cur_hid_dev] = payload_len;
			report_descr_done = 1;
			break;

		case HOSTIF_GET_FEATURE_REPORT:
			report_type = HID_FEATURE_REPORT;
			goto do_get_report;

		case HOSTIF_GET_INPUT_REPORT:
			report_type = HID_INPUT_REPORT;
do_get_report:
			/* Get index of device that matches this id */
			for (i = 0; i < num_hid_devices; ++i)
				if (recv_msg->hdr.device_id == hid_devices[i].dev_id)
					if (hid_sensor_hubs[i] != NULL) {
						hid_input_report(hid_sensor_hubs[i], report_type, payload, payload_len, 0);
						break;
					}
			get_report_done = 1;
			break;

		case HOSTIF_PUBLISH_INPUT_REPORT:
			report_type = HID_INPUT_REPORT;
			for (i = 0; i < num_hid_devices; ++i)
				if (recv_msg->hdr.device_id == hid_devices[i].dev_id)
					if (hid_sensor_hubs[i] != NULL)
						hid_input_report(hid_sensor_hubs[i], report_type, payload, payload_len, 0);
			break;
		}
	} while (0);
	may_send = 1;
}


void ish_cl_event_cb(struct heci_cl_device *device, u32 events, void *context)
{
	size_t r_length;

	if (!(device->events & 1<<HECI_CL_EVENT_RX))
		return;

	if (!device->cl)
		return;

	if (!device->cl->read_cb)
		return;

	if (!device->cl->read_cb->response_buffer.data)
		return;

	r_length = device->cl->read_cb->buf_idx;

	/* decide what to do with received data */
	process_recv(device->cl->read_cb->response_buffer.data, r_length);
	heci_io_cb_free(device->cl->read_cb);
	device->cl->reading_state = HECI_IDLE;
	device->cl->read_cb = NULL;
}


int ish_heci_cl_probe(struct heci_cl_device *dev, const struct heci_cl_device_id *id)
{
	int rv;

	if (!dev || !dev->cl)
		return	-ENODEV;

	if (uuid_le_cmp(ish_heci_guid, dev->cl->device_uuid) != 0)
		return	-ENODEV;

	ish_heci_cl = dev->cl;

	set_ish_special_client(ish_heci_cl);

	ish_heci_client_found = 1;

	return	0;
}


int ish_heci_cl_remove(struct heci_cl_device *dev)
{
	heci_hid_remove();
	unset_ish_special_client();
	ish_heci_client_found = 0;
	ish_heci_cl = NULL;
	return	0;
}


struct heci_cl_driver ish_heci_cl_driver = {
	.name = "ish",
	.probe = ish_heci_cl_probe,
	.remove = ish_heci_cl_remove,
};


int ish_heci_connect(void)
{
	struct heci_cl *cl = ish_heci_cl;
	struct heci_device *dev;
	long timeout = HECI_CL_CONNECT_TIMEOUT;
	int rv;
	int i;

	if (WARN_ON(!cl || !cl->dev))
		return -ENODEV;

	dev = cl->dev;

	/* Get HECI client ID by GUID */
	if (dev->dev_state != HECI_DEV_ENABLED)
		return	-ENODEV;

	if (cl->state != HECI_FILE_INITIALIZING && cl->state != HECI_FILE_DISCONNECTED)
		return	-EBUSY;

	/* find HECI cclient we're trying to connect to */
	i = heci_me_cl_by_uuid(dev, &ish_heci_guid);
	if (i < 0 || dev->me_clients[i].props.fixed_address)
		return	-ENODEV;

	cl->me_client_id = dev->me_clients[i].client_id;
	cl->state = HECI_FILE_CONNECTING;

	/* Do connect */
	mutex_lock(&dev->device_lock);
	if (heci_hbm_cl_connect_req(dev, cl)) {
		mutex_unlock(&dev->device_lock);
		return	-ENODEV;
	}

	cl->timer_count = HECI_CONNECT_TIMEOUT;
	mutex_unlock(&dev->device_lock);
	rv = wait_event_timeout(dev->wait_recvd_msg,
		(dev->dev_state == HECI_DEV_ENABLED &&
		(cl->state == HECI_FILE_CONNECTED ||
		cl->state == HECI_FILE_DISCONNECTED)), 5 * HZ);

	/* If FW reset arrived, this will happen. Don't check cl->, as 'cl' may be freed already */
	if (dev->dev_state != HECI_DEV_ENABLED) {
		rv = -ENODEV;
		goto	quit;
	}

	if (cl->state != HECI_FILE_CONNECTED)
		rv = -EFAULT;
	else
		rv = cl->status;
quit:
	return	rv;
}

static int ish_init(void)
{
	int rv;
	static unsigned char buf[4096];
	unsigned len;
	struct hostif_msg *msg = (struct hostif_msg *)buf;
	int i;

	/* Register HECI client device driver - ISH */
	rv = heci_cl_driver_register(&ish_heci_cl_driver);

	timed_wait_for_timeout(WAIT_FOR_SEND_SLICE, ish_heci_client_found, (5 * HZ));
	if (!ish_heci_client_found) {
		rv = -ENODEV;
		goto	ret;
	}

	rv = ish_heci_connect();
	if (rv)
		goto ret;

	/* Register read callback */
	heci_cl_register_event_cb(ish_heci_cl->device, ish_cl_event_cb, NULL);

	/*
	 * Wait until we can send without risking flow-control break scenario (sending OUR FC ahead of message, so that FW will respond)
	 * We probably need here only a small delay in order to let our FC to be sent over to FW
	 */
	timed_wait_for_timeout(WAIT_FOR_SEND_SLICE, 1 /*may_send*/,  (5 * HZ));

	/* Send HOSTIF_DM_ENUM_DEVICES and receive response in kthread_read */
	memset(msg, 0, sizeof(struct hostif_msg));
	msg->hdr.command = HOSTIF_DM_ENUM_DEVICES;
	len = sizeof(struct hostif_msg);
	rv = ish_heci_cl_send(ish_heci_cl, buf, len, 0);       /* Non-Blocking */
	rv = 0;

	timed_wait_for_timeout(WAIT_FOR_SEND_SLICE, enum_devices_done, (5 * HZ));

	if (!enum_devices_done) {
		rv = -ETIMEDOUT;
		goto	err_ret;
	}

	if (!hid_devices) {
		rv = -ENOMEM;
err_ret:
		if (waitqueue_active(&ish_heci_cl->rx_wait))
			wake_up_interruptible(&ish_heci_cl->rx_wait);

		goto	ret;
	}

	/* Send GET_HID_DESCRIPTOR for each device */
	/* Temporary work-around for multi-descriptor traffic: read only the first one */
	/* Will be removed when multi-TLC are supported */

	num_hid_devices = hid_dev_count;
/*
	if (num_hid_devices > 1)
		num_hid_devices = 1;
*/
	for (i = 0; i < num_hid_devices /*hid_dev_count*/; ++i) {
		cur_hid_dev = i;
		/* Get HID descriptor */
		hid_descr_done = 0;
		memset(msg, 0, sizeof(struct hostif_msg));
		msg->hdr.command = HOSTIF_GET_HID_DESCRIPTOR;
		msg->hdr.device_id = hid_devices[i].dev_id;
		len = sizeof(struct hostif_msg);
		rv = ish_heci_cl_send(ish_heci_cl, buf, len, 0);       /* Non-Blocking */
		rv = 0;
#ifdef HOST_VIRTUALBOX
		timed_wait_for(WAIT_FOR_SEND_SLICE, hid_descr_done);
#else
		timed_wait_for_timeout(WAIT_FOR_SEND_SLICE, hid_descr_done, (5 * HZ));
#endif
		if (!hid_descr_done) {
			continue;
/*
			rv = -ETIMEDOUT;
err_ret2:
			kfree(hid_devices);
			goto	err_ret;
*/
		}
		if (!hid_descr[i]) {
			continue;
/*
			rv = -ENOMEM;
			goto	err_ret2;
*/
		}

		/* Get report descriptor */
		report_descr_done = 0;
		memset(msg, 0, sizeof(struct hostif_msg));
		msg->hdr.command = HOSTIF_GET_REPORT_DESCRIPTOR;
		msg->hdr.device_id = hid_devices[i].dev_id;
		len = sizeof(struct hostif_msg);
		rv = ish_heci_cl_send(ish_heci_cl, buf, len, 0);       /* Non-Blocking */
		rv = 0;

#ifdef HOST_VIRTUALBOX
		timed_wait_for(WAIT_FOR_SEND_SLICE, report_descr_done);
#else
		timed_wait_for_timeout(WAIT_FOR_SEND_SLICE, report_descr_done, (5 * HZ));
#endif
		if (!report_descr_done) {
			continue;
/*
			rv = -ETIMEDOUT;
			goto	err_ret3;
*/
		}
		if (!report_descr[i]) {
			continue;
/*
			rv = -ENOMEM;
err_ret3:
			kfree(hid_descr);
			goto	err_ret2;
*/
		}

		rv = heci_hid_probe(i);
		if (rv) {
			continue;
/*
			kfree(report_descr[i]);
			goto	err_ret3;
*/
		}
	} /* for() */


ret:
	if (rv)
		heci_cl_driver_unregister(&ish_heci_cl_driver);

	return	rv;
}


static void __exit ish_exit(void)
{
	int rv;

	unset_ish_special_client();
	heci_cl_driver_unregister(&ish_heci_cl_driver);
}

module_init(ish_init);
module_exit(ish_exit);

MODULE_DESCRIPTION("ISH HECI client driver");
MODULE_AUTHOR("Intel Corporation");
MODULE_LICENSE("GPL");
