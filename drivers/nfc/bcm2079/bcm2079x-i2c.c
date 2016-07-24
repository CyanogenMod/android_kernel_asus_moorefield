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

#include <linux/HWVersion.h>
#include <linux/nfc/bcm2079x.h>

#include "bcm2079x-i2c.h"

/* do not change below */
#define MAX_BUFFER_SIZE		780

/* Read data */
#define PACKET_HEADER_SIZE_NCI	(4)
#define PACKET_HEADER_SIZE_HCI	(3)
#define PACKET_TYPE_NCI		(16)
#define PACKET_TYPE_HCIEV	(4)
#define MAX_PACKET_SIZE		(PACKET_HEADER_SIZE_NCI + 255)

#if defined(CONFIG_ASUS_FACTORY_MODE) && CONFIG_ASUS_FACTORY_MODE
static int asus_nfc_test = 1;
#else
static int asus_nfc_test = 0;
#endif
static int asus_nfc_charger_mode= 0;

extern int Read_PCB_ID(void);
extern int Read_PROJ_ID(void);

static void bcm2079x_init_stat(struct bcm2079x_dev *bcm2079x_dev)
{
	bcm2079x_dev->count_irq = 0;
}

void bcm2079x_disable_irq(struct bcm2079x_dev *bcm2079x_dev)
{
	unsigned long flags;

        pr_info("enter %s\n", __func__);

	spin_lock_irqsave(&bcm2079x_dev->irq_enabled_lock, flags);
	if (bcm2079x_dev->irq_enabled) {
		disable_irq_nosync(bcm2079x_dev->client->irq);
		bcm2079x_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&bcm2079x_dev->irq_enabled_lock, flags);
}

void bcm2079x_enable_irq(struct bcm2079x_dev *bcm2079x_dev)
{
	unsigned long flags;

        pr_info("enter %s\n", __func__);

	spin_lock_irqsave(&bcm2079x_dev->irq_enabled_lock, flags);
	if (!bcm2079x_dev->irq_enabled) {
		bcm2079x_dev->irq_enabled = true;
		enable_irq(bcm2079x_dev->client->irq);
	}
	spin_unlock_irqrestore(&bcm2079x_dev->irq_enabled_lock, flags);
}

void nfc_set_client_addr(struct bcm2079x_dev *bcm2079x_dev, int addr)
{
	struct i2c_client *client = bcm2079x_dev->client;

        pr_info("enter %s\n", __func__);

	dev_info(&client->dev,
		"Set client device address from 0x%04X flag = "\
		"%02x, to  0x%04X\n",
	client->addr, client->flags, addr);
	client->addr = addr;
	if (addr < 0x80)
		client->flags &= ~I2C_CLIENT_TEN;
	else
		client->flags |= I2C_CLIENT_TEN;
}

static irqreturn_t bcm2079x_dev_irq_handler(int irq, void *dev_id)
{
	struct bcm2079x_dev *bcm2079x_dev = dev_id;
	unsigned long flags;

        pr_info("enter %s\n", __func__);

	spin_lock_irqsave(&bcm2079x_dev->irq_enabled_lock, flags);
	bcm2079x_dev->count_irq++;
	spin_unlock_irqrestore(&bcm2079x_dev->irq_enabled_lock, flags);
	wake_up(&bcm2079x_dev->read_wq);

	return IRQ_HANDLED;
}

static unsigned int bcm2079x_dev_poll(struct file *filp, poll_table *wait)
{
	struct bcm2079x_dev *bcm2079x_dev = filp->private_data;
	unsigned int mask = 0;
	unsigned long flags;

        pr_info("enter %s\n", __func__);

	poll_wait(filp, &bcm2079x_dev->read_wq, wait);

	spin_lock_irqsave(&bcm2079x_dev->irq_enabled_lock, flags);
	if (bcm2079x_dev->count_irq > 0) {
		bcm2079x_dev->count_irq--;
		mask |= POLLIN | POLLRDNORM;
	}
	spin_unlock_irqrestore(&bcm2079x_dev->irq_enabled_lock, flags);

	return mask;
}

static ssize_t bcm2079x_dev_read(struct file *filp, char __user *buf,
				  size_t count, loff_t *offset)
{
	struct bcm2079x_dev *bcm2079x_dev = filp->private_data;
	unsigned char tmp[MAX_BUFFER_SIZE];
	int total, len, ret;

        pr_info("enter %s\n", __func__);

	total = 0;
	len = 0;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	mutex_lock(&bcm2079x_dev->read_mutex);

	/** Read the first 4 bytes to include the length of the NCI or HCI packet.
	**/
	ret = i2c_master_recv(bcm2079x_dev->client, tmp, 4);
	if (ret == 4) {
		total = ret;
		/** First byte is the packet type
		**/
		switch (tmp[0]) {
		case PACKET_TYPE_NCI:
			len = tmp[PACKET_HEADER_SIZE_NCI-1];
			break;

		case PACKET_TYPE_HCIEV:
			len = tmp[PACKET_HEADER_SIZE_HCI-1];
			if (len == 0)
				total--;/*Since payload is 0, decrement total size (from 4 to 3) */
			else
				len--;/*First byte of payload is in tmp[3] already */
			break;

		default:
			len = 0;/*Unknown packet byte */
			break;
		} /* switch*/

		/** make sure full packet fits in the buffer
		**/
		if (len > 0 && (len + total) <= count) {
			/** read the remainder of the packet.
			**/
			ret = i2c_master_recv(bcm2079x_dev->client, tmp+total, len);
			if (ret == len)
				total += len;
		} /* if */
	} /* if */

	mutex_unlock(&bcm2079x_dev->read_mutex);

	if (total > count || copy_to_user(buf, tmp, total)) {
		dev_err(&bcm2079x_dev->client->dev,
			"failed to copy to user space, total = %d\n", total);
		total = -EFAULT;
	}
        //int i;
        //for (i=0; i<total; i+=8) {
        //        pr_info("buf[%02X]: %02X %02X %02X %02X %02X %02X %02X %02X\n", i,
        //                        tmp[i], tmp[i+1], tmp[i+2], tmp[i+3],
        //                        tmp[i+4], tmp[i+5], tmp[i+6], tmp[i+7]
        //               );
        //}

	return total;
}

static ssize_t bcm2079x_dev_write(struct file *filp, const char __user *buf,
				   size_t count, loff_t *offset)
{
	struct bcm2079x_dev *bcm2079x_dev = filp->private_data;
	char tmp[MAX_BUFFER_SIZE];
	int ret;
        int retry=3;

        pr_info("enter %s\n", __func__);

	if (count > MAX_BUFFER_SIZE) {
		dev_err(&bcm2079x_dev->client->dev, "out of memory\n");
		return -ENOMEM;
	}

	if (copy_from_user(tmp, buf, count)) {
		dev_err(&bcm2079x_dev->client->dev,
			"failed to copy from user space\n");
		return -EFAULT;
	}

        //int i,j;
        //for (i=0; i<count; i+=8) {
        //        pr_info("buf[%02X]: %02X %02X %02X %02X %02X %02X %02X %02X\n", i,
        //                        tmp[i], tmp[i+1], tmp[i+2], tmp[i+3],
        //                        tmp[i+4], tmp[i+5], tmp[i+6], tmp[i+7]
        //        );
        //}

	mutex_lock(&bcm2079x_dev->read_mutex);
	/* Write data */

        ret = i2c_master_send(bcm2079x_dev->client, tmp, count);
	if (ret != count) {
		dev_err(&bcm2079x_dev->client->dev,
			"failed to write %d != %d\n", ret, count);
		ret = -EIO;
	}
	mutex_unlock(&bcm2079x_dev->read_mutex);

	return ret;
}

static int bcm2079x_dev_open(struct inode *inode, struct file *filp)
{
	int ret = 0;

	struct bcm2079x_dev *bcm2079x_dev = container_of(filp->private_data,
							   struct bcm2079x_dev,
							   bcm2079x_device);

        pr_info("enter %s\n", __func__);
	filp->private_data = bcm2079x_dev;
	bcm2079x_init_stat(bcm2079x_dev);
	bcm2079x_enable_irq(bcm2079x_dev);
	dev_info(&bcm2079x_dev->client->dev,
		 "bcm2079x_dev_open: %d,%d\n", imajor(inode), iminor(inode));

	return ret;
}

static long bcm2079x_dev_unlocked_ioctl(struct file *filp,
					 unsigned int cmd, unsigned long arg)
{
	struct bcm2079x_dev *bcm2079x_dev = filp->private_data;

	dev_info(&bcm2079x_dev->client->dev,
			"%s, Rev CMD&Arg (0x%x, 0x%lx)\n", __func__, cmd, arg);

	switch (cmd) {
	case BCMNFC_READ_FULL_PACKET:
		break;
	case BCMNFC_READ_MULTI_PACKETS:
		break;
	/* Remove a workaround since BCM20793 chip default is 7 bits address */
	case BCMNFC_CHANGE_ADDR:
		break;
	case BCMNFC_POWER_CTL:
		gpio_set_value(bcm2079x_dev->en_gpio, arg);
		break;
	case BCMNFC_WAKE_CTL:
		gpio_set_value(bcm2079x_dev->wake_gpio, arg);
		break;
	case BCMNFC_SET_ADDR:
		nfc_set_client_addr(bcm2079x_dev, arg);
		break;
	default:
		dev_err(&bcm2079x_dev->client->dev,
			"%s, unknown cmd (%x, %lx)\n", __func__, cmd, arg);
		return -ENOSYS;
	}

	return 0;
}

static const struct file_operations bcm2079x_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.poll = bcm2079x_dev_poll,
	.read = bcm2079x_dev_read,
	.write = bcm2079x_dev_write,
	.open = bcm2079x_dev_open,
        .unlocked_ioctl = bcm2079x_dev_unlocked_ioctl,
	.compat_ioctl = bcm2079x_dev_unlocked_ioctl
};

static int bcm2079x_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	int ret=0;
	struct bcm2079x_pdata *platform_data;
	struct bcm2079x_dev *bcm2079x_dev;

        pr_info("enter %s\n", __func__);

	platform_data = client->dev.platform_data;

        dev_info(&client->dev, "%s, probing bcm2079x driver flags = %x\n", __func__, client->flags);
	if (platform_data == NULL) {
		dev_err(&client->dev, "nfc probe fail\n");
		return -ENODEV;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "need I2C_FUNC_I2C\n");
		return -ENODEV;
	}

	ret = platform_data->request_resources(client);
	if (ret)
		return -ENODEV;

	gpio_set_value(platform_data->en_gpio, 0);
	gpio_set_value(platform_data->wake_gpio, 0);

	bcm2079x_dev = kzalloc(sizeof(*bcm2079x_dev), GFP_KERNEL);
	if (bcm2079x_dev == NULL) {
		dev_err(&client->dev, "failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_exit;
	}

	bcm2079x_dev->wake_gpio = platform_data->wake_gpio;
	bcm2079x_dev->irq_gpio = platform_data->irq_gpio;
	bcm2079x_dev->en_gpio = platform_data->en_gpio;
	bcm2079x_dev->client = client;

	/* init mutex and queues */
	init_waitqueue_head(&bcm2079x_dev->read_wq);
	mutex_init(&bcm2079x_dev->read_mutex);
	spin_lock_init(&bcm2079x_dev->irq_enabled_lock);

	bcm2079x_dev->bcm2079x_device.minor = MISC_DYNAMIC_MINOR;
	bcm2079x_dev->bcm2079x_device.name = "bcm2079x-i2c";
	bcm2079x_dev->bcm2079x_device.fops = &bcm2079x_dev_fops;

	ret = misc_register(&bcm2079x_dev->bcm2079x_device);
	if (ret) {
		dev_err(&client->dev, "misc_register failed\n");
		goto err_misc_register;
	}

	/* request irq.	 the irq is set whenever the chip has data available
	 * for reading.	 it is cleared when all data has been read.
	 */
	dev_info(&client->dev, "requesting IRQ %d\n", client->irq);
	bcm2079x_dev->irq_enabled = true;
	ret = request_irq(client->irq, bcm2079x_dev_irq_handler,
			  IRQF_TRIGGER_RISING, client->name, bcm2079x_dev);
	if (ret) {
		dev_err(&client->dev, "request_irq failed\n");
		goto err_request_irq_failed;
	}
	bcm2079x_disable_irq(bcm2079x_dev);
	i2c_set_clientdata(client, bcm2079x_dev);
	dev_info(&client->dev, "%s, probing bcm2079x driver exited successfully\n", __func__);

        if (asus_nfc_test == 1) {
                ret = bcm2079x_probe_test(client);
                pr_info("##################################################\n");
                if (ret<0) {
                        pr_info("             NFC TEST FAIL              \n");
                }else {
                        pr_info("             NFC TEST OK              \n");
                }
                pr_info("##################################################\n");
        }

	return 0;
//
err_request_irq_failed:
	misc_deregister(&bcm2079x_dev->bcm2079x_device);
err_misc_register:
	mutex_destroy(&bcm2079x_dev->read_mutex);
	kfree(bcm2079x_dev);
err_exit:
	gpio_free(platform_data->wake_gpio);
	gpio_free(platform_data->en_gpio);
	gpio_free(platform_data->irq_gpio);
	return ret;
}

static int bcm2079x_remove(struct i2c_client *client)
{
	struct bcm2079x_dev *bcm2079x_dev;

        pr_info("enter %s\n", __func__);

	bcm2079x_dev = i2c_get_clientdata(client);
	free_irq(client->irq, bcm2079x_dev);
	misc_deregister(&bcm2079x_dev->bcm2079x_device);
	mutex_destroy(&bcm2079x_dev->read_mutex);
	gpio_free(bcm2079x_dev->irq_gpio);
	gpio_free(bcm2079x_dev->en_gpio);
	gpio_free(bcm2079x_dev->wake_gpio);
	kfree(bcm2079x_dev);

	return 0;
}

static const struct i2c_device_id bcm2079x_id[] = {
	{"bcm2079x-i2c", 0},
	{"bcm20795", 0},
	{}
};

static struct i2c_driver bcm2079x_driver = {
	.id_table = bcm2079x_id,
	.probe = bcm2079x_probe,
	.remove = bcm2079x_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "bcm2079x-i2c",
	},
};

/*
 * module load/unload record keeping
 */

int bcm2079x_info_read(struct file *filp, char __user *buffer, size_t count, loff_t *ppos)
{
	int len=0;
	ssize_t ret = 0;
	char buff[128];
        u32 ese_support=0;
        int real_proj_id=0;
        int proj_id=0;
        int pcb_id=0;

        pcb_id = Read_PCB_ID();
        real_proj_id = (pcb_id & PROJ_ID_MASK) >> PROJ_ID_SHIFT;
        proj_id = Read_PROJ_ID();
        ese_support = real_proj_id == PROJ_ID_ZE551ML_ESE ? 1 : 0;

        //len += sprintf(buff, "pcb_id: 0x%X, proj_id: 0x%X, real_proj_id: 0x%X, ese: %d\n", 
        //                pcb_id, 
        //                proj_id,
        //                real_proj_id,
        //                ese_support
        //        );
        len += sprintf(buff, "%d\n", ese_support);

	ret = simple_read_from_buffer(buffer,count,ppos,buff,len);

  	return ret;

}

static int __init bcm2079x_mode_setup(char *str)
{
        if (strncmp(str, "charger", 7) == 0) {
                asus_nfc_charger_mode = 1;
        }
	return 1;
}
__setup("androidboot.mode=", bcm2079x_mode_setup);

static int __init bcm2079x_test_setup(char *str)
{
        if (strncmp(str, "on", 2) == 0) {
                asus_nfc_test = 1;
        }
	return 1;
}
__setup("asus.dbg.nfc_test=", bcm2079x_test_setup);

static int __init bcm2079x_dev_init(void)
{
	int ret=0;

        pr_info("enter %s\n", __func__);

        /* no eSE sku, comment it... 
        static struct file_operations bcm2079x_info_fop = {
                .read = bcm2079x_info_read,
                //.write = asus_battery_info_proc_write,
        };
        ret = proc_create("nfc_ese_support", 0666,NULL, &bcm2079x_info_fop); 
        if(!ret) {
                pr_err("create /proc/nfc_ese_support fail\n");
        }
        */

        if (asus_nfc_charger_mode) {
                pr_info("Charger mode deteted.... skip initiallize");
                return 0;
        }

	return i2c_add_driver(&bcm2079x_driver);
}
module_init(bcm2079x_dev_init);

static void __exit bcm2079x_dev_exit(void)
{
        pr_info("enter %s\n", __func__);
	//project_id project;
	//project = asustek_get_project_id();

	//if (project == 0)
	//	i2c_del_driver(&bcm2079x_driver);
}
module_exit(bcm2079x_dev_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("NFC bcm2079x driver");
MODULE_LICENSE("GPL");
