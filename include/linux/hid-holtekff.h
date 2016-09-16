#ifndef __HID_XE_FLASH_H
#define __HID_XE_FLASH_H
#include <media/v4l2-subdev.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/mod_devicetable.h> /* hid_device_id */
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/semaphore.h>
#include <linux/power_supply.h>
#include <uapi/linux/hid.h>

int Xe_flash_inserted(void);
int Xe_flash_on(struct v4l2_subdev *sd, u16 delay, u16 pulse);
void Xe_flash_send_cmd(u8 cmd[8]);
void Xe_flash_rcv_cmd(u8 read_cmd[8], u8** read_back_buff);
void Xe_flash_camera_set_alive(u8 alive_flag);
#endif //__HID_XE_FLASH_H

