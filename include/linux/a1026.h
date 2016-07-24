/* include/linux/a1026.h - a1026 voice processor driver
 *
 * Copyright (C) 2009 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __LINUX_A1026_H
#define __LINUX_A1026_H

#include <linux/ioctl.h>
#include <linux/i2c.h>

#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#define A1026_MAX_FW_SIZE	(256*1024)
struct a1026img {
	unsigned char *buf;
	unsigned img_size;
};


/* indicates if a1026_set_config() performs a full configuration or only
 * a voice processing algorithm configuration */
/* IOCTLs for Audience A1026 */
#define A1026_IOCTL_MAGIC 'u'

#define A1026_BOOTUP_INIT  _IO(A1026_IOCTL_MAGIC, 0x01)
#define A1026_SUSPEND   _IO(A1026_IOCTL_MAGIC, 0x02)
#define A1026_ENABLE_CLOCK   _IO(A1026_IOCTL_MAGIC, 0x03)

#ifdef __KERNEL__

/* A1026 Command codes */
#define A100_msg_Sync		0x80000000
#define A100_msg_Sync_Ack	0x80000000

#define A100_msg_Reset		0x8002
#define RESET_IMMEDIATE		0x0000
#define RESET_DELAYED		0x0001

#define A100_msg_BootloadInitiate	0x8003

/* Set Power State */
#define A100_msg_Sleep		0x80100001
/* Audio Path Commands */

/* Bypass */
#define A100_msg_Bypass		0x801C /* 0ff = 0x0000; on = 0x0001 (Default) */

#define A1026_msg_BOOT		0x0001
#define A1026_msg_BOOT_ACK	0x01

/* general definitions */
#define TIMEOUT			20 /* ms */
#define RETRY_CNT		5
#define POLLING_RETRY_CNT	3
#define A1026_ERROR_CODE	0xffff
#define A1026_SLEEP		0
#define A1026_ACTIVE		1
#define A1026_CMD_FIFO_DEPTH	64
#define ERROR			0xffffffff

enum A1026_config_mode {
	A1026_CONFIG_FULL,
	A1026_CONFIG_VP
};

struct a1026_platform_data {
	uint32_t gpio_a1026_wakeup;
	uint32_t gpio_a1026_reset;
	int (*request_resources) (struct i2c_client *client);
	void (*free_resources) (struct i2c_client *client);
	void (*reset) (bool state);
	void (*wakeup) (bool state);
};


#endif /* __KERNEL__ */
#endif /* __LINUX_A1026_H */
