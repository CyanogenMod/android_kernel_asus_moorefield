/*
 * Copyright (C) 2012 Broadcom Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/poll.h>
#include <linux/version.h>
#include <linux/proc_fs.h>
#include <linux/debugfs.h>
#include <linux/delay.h>

#include <linux/nfc/bcm2079x.h>

#include "bcm2079x-i2c.h"

static u32 nfc_slave_addr=0x76;


static int asus_nfc_write(struct file *filp, const char __user *buffer, size_t count, loff_t *ppos)
{
	struct bcm2079x_dev *bcm2079x_dev = PDE_DATA(filp->f_inode);
        int ret=0;
        u8  tmp[4];
        int len;
        char buf[256];

        tmp[0] = 0x10;
        tmp[1] = 0x2f;
        tmp[2] = 0x04;
        tmp[3] = 0x00;
        len = sprintf(buf, "addr: 0x%02X, ret=%d, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n", 
                        nfc_slave_addr, ret, tmp[0], tmp[1], tmp[2], tmp[3]);

        dev_info(&bcm2079x_dev->client->dev, "%s", buf);

        nfc_set_client_addr(bcm2079x_dev, nfc_slave_addr);
	ret = i2c_master_send(bcm2079x_dev->client, tmp, sizeof(tmp));
	if (ret != sizeof(tmp)) {
		dev_err(&bcm2079x_dev->client->dev,
			"failed to write %d\n", ret);
		ret = -EIO;
	}
        return sizeof(tmp);
}

static int asus_nfc_read(struct file *filp, char __user *buffer, size_t count, loff_t *ppos)
{
	struct bcm2079x_dev *bcm2079x_dev = PDE_DATA(filp->f_inode);
        int ret=0;
        u8  tmp[4];
        int len;
        char buf[256];

        if (!bcm2079x_dev) {
                pr_err("get dev fail");
                return 0;
        }

        buf[0] = 0;
        nfc_set_client_addr(bcm2079x_dev, nfc_slave_addr);

	ret = i2c_master_recv(bcm2079x_dev->client, tmp, 4);
        len = sprintf(buf, "addr: 0x%02X, ret=%d, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n", 
                        nfc_slave_addr, ret, tmp[0], tmp[1], tmp[2], tmp[3]);
	len = simple_read_from_buffer(buffer, count, ppos, buf, len);
        dev_info(&bcm2079x_dev->client->dev, "%s", buf);

        return len;
}

static int test_probe(struct i2c_client *client) //basic test
{
	struct bcm2079x_dev *bcm2079x_dev = NULL;
        int ret=0;
        u8  tmp[4];
        u8 buf[256];
        int len;

	bcm2079x_dev = i2c_get_clientdata(client);
        gpio_set_value(bcm2079x_dev->en_gpio, 1);
        msleep(100);

        tmp[0] = 0x10;
        tmp[1] = 0x2f;
        tmp[2] = 0x04;
        tmp[3] = 0x00;
        nfc_set_client_addr(bcm2079x_dev, nfc_slave_addr);
	ret = i2c_master_send(client, tmp, sizeof(tmp));
	if (ret != sizeof(tmp)) {
		dev_err(&client->dev,
			"failed to write %d\n", ret);
		ret = -EIO;
                goto err;
	}

        msleep(10);
	ret = i2c_master_recv(client, tmp, 4);
        len = sprintf(buf, "addr: 0x%02X, ret=%d, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n", 
                        nfc_slave_addr, ret, tmp[0], tmp[1], tmp[2], tmp[3]);
        if (ret < 0) {
                dev_info(&client->dev, "%s", buf);
                ret = -EIO;
                goto err;
        }
err:
        gpio_set_value(bcm2079x_dev->en_gpio, 0);
        return ret;
}


int bcm2079x_probe_test(struct i2c_client *client)
{
        struct proc_dir_entry *pde=NULL;
        u32 ret=0;
        u8  tmp[4];
	struct bcm2079x_dev *bcm2079x_dev = NULL;
        static struct file_operations asus_nfc_fop = {
                .read = asus_nfc_read,
                .write = asus_nfc_write,
        };

	bcm2079x_dev = i2c_get_clientdata(client);

        pr_info("enter %s\n", __func__);

        pde = proc_create_data("asus_nfc_test", 0666, NULL, &asus_nfc_fop, bcm2079x_dev); 
        if (!pde) {
                pr_err("create /proc/asus_nfc_test\n");
        }

        if (!debugfs_create_x32("nfc_slave_addr", S_IRUGO | S_IWUSR, NULL,&nfc_slave_addr))
                pr_err("create nfc_slave_addr node FAIL\n");

        return test_probe(client);
}

