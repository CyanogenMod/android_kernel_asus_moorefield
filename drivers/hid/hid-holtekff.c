/*
 *  Force feedback support for Holtek On Line Grip based gamepads
 *
 *  These include at least a Brazilian "Clone Joypad Super Power Fire"
 *  which uses vendor ID 0x1241 and identifies as "HOLTEK On Line Grip".
 *
 *  Copyright (c) 2011 Anssi Hannula <anssi.hannula@iki.fi>
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/hid.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include "hid-ids.h"
#include <linux/hid-holtekff.h>
#if 1

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Anssi Hannula <anssi.hannula@iki.fi>");
MODULE_DESCRIPTION("Force feedback support for Holtek On Line Grip based devices");

/*
 * These commands and parameters are currently known:
 *
 * byte 0: command id:
 * 	01  set effect parameters
 * 	02  play specified effect
 * 	03  stop specified effect
 * 	04  stop all effects
 * 	06  stop all effects
 * 	(the difference between 04 and 06 isn't known; win driver
 * 	 sends 06,04 on application init, and 06 otherwise)
 * 
 * Commands 01 and 02 need to be sent as pairs, i.e. you need to send 01
 * before each 02.
 *
 * The rest of the bytes are parameters. Command 01 takes all of them, and
 * commands 02,03 take only the effect id.
 *
 * byte 1:
 *	bits 0-3: effect id:
 * 		1: very strong rumble
 * 		2: periodic rumble, short intervals
 * 		3: very strong rumble
 * 		4: periodic rumble, long intervals
 * 		5: weak periodic rumble, long intervals
 * 		6: weak periodic rumble, short intervals
 * 		7: periodic rumble, short intervals
 * 		8: strong periodic rumble, short intervals
 * 		9: very strong rumble
 * 		a: causes an error
 * 		b: very strong periodic rumble, very short intervals
 * 		c-f: nothing
 *	bit 6: right (weak) motor enabled
 *	bit 7: left (strong) motor enabled
 *
 * bytes 2-3:  time in milliseconds, big-endian
 * bytes 5-6:  unknown (win driver seems to use at least 10e0 with effect 1
 * 		       and 0014 with effect 6)
 * byte 7:
 *	bits 0-3: effect magnitude
 */

#define CHRGING_TIME_20S      (20)
#define HOLTEKFF_MSG_LENGTH     8
static u8 read_test_buf[] =           { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static u8 mread_buf[] =           { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static u8 show_buf[] =           { 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static u8 camera_apk_alive;
struct hid_device *mhid;
struct holtekff_device *mholtekff;
/////
static struct holtekff_device *holtekff;
static struct hid_report *report;
static struct hid_input *hidinput;
static struct list_head *report_list;
static struct input_dev *dev;
////
struct holtekff_device {
	struct hid_field *field;
};
///////////////////
u32 static charging_time = CHRGING_TIME_20S;
u8 static file_node_flag;
static struct class* holtek_charging_userCtrl_class;
static struct device* holtek_charging_register_ctrl_dev;
static ssize_t holtek_charging_send_cmd_show(struct device *dev,
        struct device_attribute *attr, char *buf){

    int ret;
    ret = sprintf(buf, "ASUSBSP --- charging_time is %d \n", charging_time);

    return ret;
}


static ssize_t holtek_charging_send_cmd_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count){


    charging_time = -1;
    sscanf(buf, "%d", &charging_time);


    printk(KERN_INFO "ASUSBSP --- charging_time is %d \n", charging_time);
    return count;
}


DEVICE_ATTR(holtek_charging_send_cmd, 0660, holtek_charging_send_cmd_show, holtek_charging_send_cmd_store);
static void holtekff_send(struct holtekff_device *holtekff,
			  struct hid_device *hid,
			  const u8 data[HOLTEKFF_MSG_LENGTH]);
static struct workqueue_struct *brook_workqueue;
static void brook_3_routine(struct work_struct *);
static DECLARE_DELAYED_WORK(brook_3_work, brook_3_routine);
static void brook_3_routine(struct work_struct *ws)
{
    u8 charging_cmd[8] = {0x1,0,0,0,0,0,0,0};
    printk("ASUSBSP --- \n");
   if(camera_apk_alive &&!!mholtekff){
       queue_delayed_work(brook_workqueue, &brook_3_work, charging_time*HZ);
       holtekff_send(mholtekff, mhid, charging_cmd);
   }
}
//////////////////
static void holtekff_send(struct holtekff_device *holtekff,
			  struct hid_device *hid,
			  const u8 data[HOLTEKFF_MSG_LENGTH])
{
	int i;
    if(!holtekff)
          return;

	for (i = 0; i < HOLTEKFF_MSG_LENGTH; i++) {
		holtekff->field->value[i] = data[i];
	}

	hid_hw_request(hid, holtekff->field->report, HID_REQ_SET_REPORT);
    for (i = 0; i < HOLTEKFF_MSG_LENGTH; i++) {
	   printk(KERN_INFO	"ASUS, @holtekff_send: 0x%x \n", holtekff->field->value[i]);
	}
}

static void holtekff_get(struct holtekff_device *holtekff,
			  struct hid_device *hid,
			  u8** read_buf)
{
    int i;
    if(!holtekff)
        return;

    if (NULL != hid->hid_output_raw_report){
        hid->hid_get_raw_report(hid,0, mread_buf, 9, HID_FEATURE_REPORT);
    }

    hid_hw_wait(hid);
    for (i = 0; i < 9; i++) {
       printk(KERN_INFO "ASUS, AFTER GET: %x \n", mread_buf[i]);
	}
    *read_buf = mread_buf + 1;
}

static int holtekff_init(struct hid_device *hid)
{
	hidinput = list_entry(hid->inputs.next,
						struct hid_input, list);
	report_list =
			&hid->report_enum[HID_FEATURE_REPORT].report_list;
	dev = hidinput->input;
	if (list_empty(report_list)) {
		hid_err(hid, "no output report found\n");
		return -ENODEV;
	}
	report = list_entry(report_list->next, struct hid_report, list);
	holtekff = kzalloc(sizeof(*holtekff), GFP_KERNEL);
	if (!holtekff)
		return -ENOMEM;

	if(!mholtekff){
        mholtekff = holtekff;
    }

    if(!mhid){
        mhid = hid;
    }
    //set_bit(FF_RUMBLE, dev->ffbit);
	holtekff->field = report->field[0];
	report_list =
	&hid->report_enum[HID_FEATURE_REPORT].report_list;

	/* initialize the same way as win driver does */
    holtekff_send(holtekff, hid, show_buf);
    mdelay(1000);
    holtekff_get(holtekff, hid, read_test_buf);
	brook_workqueue = create_workqueue("brook_wq");
    if(camera_apk_alive){
        queue_delayed_work(brook_workqueue, &brook_3_work,0);
    }
    hid_info(hid, "Force feedback for Holtek On Line Grip based devices by Anssi Hannula <anssi.hannula@iki.fi>\n");
	return 0;
}
#else
static inline int holtekff_init(struct hid_device *hid)
{
	return 0;
}
#endif

static int holtek_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	int ret;
	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "parse failed\n");
		goto err;
	}
	ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT & ~HID_CONNECT_FF);
	if (ret) {
		hid_err(hdev, "hw start failed\n");
		goto err;
	}
	holtekff_init(hdev);
	printk(KERN_INFO "ASUSBSP --- 005\n");
    if(!file_node_flag){
        holtek_charging_userCtrl_class = class_create(THIS_MODULE, "holtek_charging_dev");
        holtek_charging_register_ctrl_dev = device_create(holtek_charging_userCtrl_class, NULL, 0, "%s", "send_command");
        device_create_file(holtek_charging_register_ctrl_dev, &dev_attr_holtek_charging_send_cmd);
        file_node_flag =1;
    }
    return 0;
err:
	return ret;
}

static void holtek_remove(struct hid_device *hid){
    printk(KERN_INFO "ASUS --- @holtek_remove \n");
    cancel_delayed_work(&brook_3_work);
    kfree(mholtekff);
    mholtekff = NULL;
    mhid = NULL;
}

void Xe_flash_send_cmd(u8 cmd[8]){
    printk(KERN_INFO "ASUS --- @Xe_flash_send_cmd \n");
    holtekff_send(mholtekff, mhid, cmd);
}

void Xe_flash_camera_set_alive(u8 alive_flag){

    camera_apk_alive = alive_flag;
    printk(KERN_INFO "ASUSBSP --- camera_apk_alive is %d \n", camera_apk_alive);

    if(Xe_flash_inserted()){
        if(camera_apk_alive){
            queue_delayed_work(brook_workqueue, &brook_3_work, 0);
        }else{
            cancel_delayed_work(&brook_3_work);
        }
    }
}

void Xe_flash_rcv_cmd(u8 read_cmd[8], u8** read_back_buff){
    printk(KERN_INFO "ASUS --- @Xe_flash_rcv_cmd \n");
    holtekff_send(mholtekff, mhid, read_cmd);
    mdelay(10);
    holtekff_get(mholtekff, mhid, read_back_buff);
}

int Xe_flash_inserted(void){
    return (!!mholtekff);
}
static const struct hid_device_id holtek_devices[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_ASUSTEK, 0x7799) },
	{ }
};
MODULE_DEVICE_TABLE(hid, holtek_devices);

static struct hid_driver holtek_driver = {
	.name = "holtek",
	.id_table = holtek_devices,
	.probe = holtek_probe,
    .remove = holtek_remove,
};
module_hid_driver(holtek_driver);
EXPORT_SYMBOL_GPL(Xe_flash_send_cmd);
EXPORT_SYMBOL_GPL(Xe_flash_rcv_cmd);
EXPORT_SYMBOL_GPL(Xe_flash_inserted);
EXPORT_SYMBOL_GPL(Xe_flash_camera_set_alive);
MODULE_LICENSE("GPL");
