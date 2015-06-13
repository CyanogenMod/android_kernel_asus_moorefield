/*
 * HECI-HID glue driver.
 *
 * Copyright (c) 2012-2014, Intel Corporation.
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
#include <linux/timer.h>
#include <linux/kthread.h>
#include "heci-hid.h"
#include "platform-config.h"

/* TODO - figure out if this number is used for anything but assignment. BUS_I2C is not */
#define	BUS_HECI	0x44
/* TODO: just to bootstrap, numbers will probably change */
#define	ISH_HID_VENDOR	0x8086
#define	ISH_HID_PRODUCT	0x22D8
#define	ISH_HID_VERSION	0x0200

extern unsigned char	*report_descr[MAX_HID_DEVICES];
extern int	report_descr_size[MAX_HID_DEVICES];
extern struct device_info	*hid_devices;
extern int	may_send;
extern int	get_report_done;			/* Get Feature/Input report complete flag */
extern unsigned	cur_hid_dev;
extern struct hid_device	*hid_sensor_hubs[MAX_HID_DEVICES];
extern unsigned	num_hid_devices;
extern struct heci_cl  *ish_heci_cl;			/* HECI client */

int ish_heci_cl_send(struct heci_cl *cl, u8 *buf, size_t length, bool blocking);

static int heci_hid_parse(struct hid_device *hid)
{
	int	rv;

	rv = hid_parse_report(hid, report_descr[cur_hid_dev], report_descr_size[cur_hid_dev]);
	if (rv)
		return	rv;

	return 0;
}

static int heci_hid_start(struct hid_device *hid)
{
	return 0;
}

/* should we free smth? */
static void heci_hid_stop(struct hid_device *hid)
{
	return;
}

/* probably connect might be here (move from probe) */
static int heci_hid_open(struct hid_device *hid)
{
	return 0;
}


/* naturally if connect in open, disconnect here */
static void heci_hid_close(struct hid_device *hid)
{
	return;
}

static int heci_hid_power(struct hid_device *hid, int lvl)
{
	return 0;
}

static void heci_set_feature(struct hid_device *hid, char *buf,
				unsigned len, int report_id)
{
	int	rv;
	struct hostif_msg *msg = (struct hostif_msg *)buf;
	int	i;

	memset(msg, 0, sizeof(struct hostif_msg));
	msg->hdr.command = HOSTIF_SET_FEATURE_REPORT;
	for (i = 0; i < num_hid_devices; ++i)
		if (hid == hid_sensor_hubs[i]) {
			/* FIXME- temporary when single collection exists,
			 * then has to be part of hid_device custom fields
			 */
			msg->hdr.device_id = hid_devices[i].dev_id;
			break;
		}
	if (i == num_hid_devices)
		return;

#ifdef HOST_VIRTUALBOX
	timed_wait_for(WAIT_FOR_SEND_SLICE, may_send);
#else
	timed_wait_for_timeout(WAIT_FOR_SEND_SLICE, may_send, (5*HZ));
#endif
	rv = ish_heci_cl_send(ish_heci_cl, buf, len, 0); /* Non-Blocking */
}

static void heci_get_report(struct hid_device *hid,
				int report_id, int report_type)
{
	int	rv;
	static unsigned char	buf[5];
	unsigned	len;
	struct hostif_msg_to_sensor *msg =
			(struct hostif_msg_to_sensor *)buf;
	int	i;

	len = sizeof(struct hostif_msg_to_sensor);

	/* Send HOSTIF_DM_ENUM_DEVICES and receive response in kthread_read */
	memset(msg, 0, sizeof(struct hostif_msg_to_sensor));
	msg->hdr.command = (report_type == HID_FEATURE_REPORT) ? HOSTIF_GET_FEATURE_REPORT : HOSTIF_GET_INPUT_REPORT;
	for (i = 0; i < num_hid_devices; ++i)
		if (hid == hid_sensor_hubs[i]) {
			/* FIXME - temporary when single collection exists,
			 * then has to be part of hid_device custom fields
			 */
			msg->hdr.device_id = hid_devices[i].dev_id;
			break;
		}
	if (i == num_hid_devices)
		return;

	msg->report_id = report_id;

#ifdef HOST_VIRTUALBOX
	timed_wait_for(WAIT_FOR_SEND_SLICE, may_send);
#else
	timed_wait_for_timeout(WAIT_FOR_SEND_SLICE, may_send, (5*HZ));
#endif
	rv = ish_heci_cl_send(ish_heci_cl, buf, len, 0);       /* Non-Blocking */
}


static void heci_hid_request(struct hid_device *hid,
			struct hid_report *rep, int reqtype)
{
	/* this is specific report length, just HID part of it */
	unsigned len = ((rep->size - 1) >> 3) + 1 + (rep->id > 0);
	char *buf;
	/* s32 checkValue = 0; */
	/* int i = 0; */
	unsigned header_size =  sizeof(struct hostif_msg);

	len += header_size;

	switch (reqtype) {
	case HID_REQ_GET_REPORT:
		heci_get_report(hid, rep->id, rep->type);
		break;
	case HID_REQ_SET_REPORT:
		buf = kzalloc(len, GFP_KERNEL);
		hid_output_report(rep, buf + header_size);
	/* checkValue = rep->field[3]->value[0]; */
	/* for(;i < len; i++) */
		heci_set_feature(hid, buf, len, rep->id);
		break;
	}

	return;
}


static int heci_hid_hidinput_input_event(struct input_dev *dev,
		unsigned int type, unsigned int code, int value)
{
	return 0;
}


static int heci_wait_for_response(struct hid_device *hid)
{
	timed_wait_for_timeout(WAIT_FOR_SEND_SLICE, get_report_done, (10 * HZ));

	if (!get_report_done) {
		dbg_hid("timeout waiting for heci device\n");
		return -1;
	}

	return 0;
}


static struct hid_ll_driver heci_hid_ll_driver = {
	.parse = heci_hid_parse,
	.start = heci_hid_start,
	.stop = heci_hid_stop,
	.open = heci_hid_open,
	.close = heci_hid_close,
	.power = heci_hid_power,
	.request = heci_hid_request,
	.hidinput_input_event = heci_hid_hidinput_input_event,
	.wait = heci_wait_for_response
};


struct tmp_heci_data {
	int hdesc_length;
	struct task_struct	*read_task;
};

static struct tmp_heci_data thd;


static int heci_hid_get_raw_report(struct hid_device *hid,
		unsigned char report_number, __u8 *buf,
		size_t count, unsigned char report_type)
{
	return	0;
}

static int heci_hid_output_raw_report(struct hid_device *hid,
		__u8 *buf, size_t count, unsigned char report_type)
{
	return	0;
}

static void i2c_hid_request(struct hid_device *hid,
			struct hid_report *rep, int reqtype)
{
}

/* probably the best way make it driver probe so it will create
 * device with itself as ll_driver, as usb and i2c do
 */
int heci_hid_probe(unsigned cur_hid_dev)
{
	int rv;
	struct hid_device *hid;

	hid = hid_allocate_device();
	if (IS_ERR(hid)) {
		rv = PTR_ERR(hid);
		return	-ENOMEM;
	}

	hid->ll_driver = &heci_hid_ll_driver;
	hid->hid_get_raw_report = heci_hid_get_raw_report;
	hid->hid_output_raw_report = heci_hid_output_raw_report;
	hid->bus = BUS_HECI;
	hid->version = le16_to_cpu(ISH_HID_VERSION);
	hid->vendor = le16_to_cpu(ISH_HID_VENDOR);
	hid->product = le16_to_cpu(ISH_HID_PRODUCT);

	snprintf(hid->name, sizeof(hid->name),
		"%s %04hX:%04hX", "hid-heci", hid->vendor, hid->product);

	rv = hid_add_device(hid);
	if (rv) {
		if (rv != -ENODEV)
			dev_err(&hid->dev, "can't add hid device: %d\n", rv);
		kfree(hid);
		return	rv;
	}

	hid_sensor_hubs[cur_hid_dev] = hid;
	return 0;
}

void heci_hid_remove(void)
{
	int rv;
	int i;

	for (i = 0; i < num_hid_devices; ++i)
		if (hid_sensor_hubs[i]) {
			hid_destroy_device(hid_sensor_hubs[i]);
			hid_sensor_hubs[i] = NULL;
		}
}
