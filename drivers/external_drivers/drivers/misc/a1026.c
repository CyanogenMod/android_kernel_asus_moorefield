/* drivers/i2c/chips/a1026.c - a1026 voice processor driver
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

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <asm/intel_scu_ipcutil.h>
#include <linux/a1026.h>
#include <linux/i2c.h>

#define FIRMWARE_NAME_MAX_LENGTH	64
#define DEBUG				0
#define ENABLE_DIAG_IOCTLS		0
/* Max cmd length on es305 side is 252 */
#define ES305_I2C_CMD_FIFO_SIZE		128
/* delay (in us) in order to wait for stable power supplies &
 * for stable system clock. Audience recommends at least 1ms. */
#define ES305_HARD_RESET_PERIOD		1100
/*
 * This driver is based on the eS305-UG-APIGINTEL-V0 2.pdf spec
 * for the eS305 Voice Processor
 */

struct vp_ctxt {
	struct i2c_client *i2c_dev;
	struct a1026_platform_data *pdata;
	unsigned long open;
	int suspended;
	struct mutex mutex;
} *es305;

static int execute_cmdmsg(unsigned int msg, struct vp_ctxt *vp);
static int suspend(struct vp_ctxt *vp);

enum bool {gpio_l, gpio_h};

static int es305_i2c_read(u8 *rxData, int length, struct vp_ctxt *the_vp)
{
	int rc;
	struct i2c_client *client = the_vp->i2c_dev;

	rc = i2c_master_recv(client, rxData, length);
	if (rc < 0) {
		pr_debug("%s: transfer error %d\n", __func__, rc);
		return rc;
	}

#if DEBUG
	{
		int i;
		for (i = 0; i < length; i++)
			pr_debug("%s: rx[%d] = %2x\n", __func__, i, rxData[i]);
	}
#endif

	return 0;
}

static int es305_i2c_write(const u8 *txData, int length, struct vp_ctxt *the_vp)
{
	int rc;
	struct i2c_client *client = the_vp->i2c_dev;

	rc = i2c_master_send(client, txData, length);
	if (rc < 0) {
		pr_debug("%s: transfer error %d\n", __func__, rc);
		return rc;
	}

#if DEBUG
	{
		int i;
		for (i = 0; i < length; i++)
			pr_debug("%s: tx[%d] = %2x\n", __func__, i, txData[i]);
	}
#endif

	return 0;
}

static void es305_i2c_sw_reset(unsigned int reset_cmd, struct vp_ctxt *vp)
{
	int rc;
	u8 msgbuf[4];

	msgbuf[0] = (reset_cmd >> 24) & 0xFF;
	msgbuf[1] = (reset_cmd >> 16) & 0xFF;
	msgbuf[2] = (reset_cmd >> 8) & 0xFF;
	msgbuf[3] = reset_cmd & 0xFF;

	pr_debug("%s: %08x\n", __func__, reset_cmd);

	rc = es305_i2c_write(msgbuf, 4, vp);
	if (!rc)
		msleep(20);
		/* 20ms is recommended polling period -- p8 spec*/
}

static int es305_open(struct inode *inode, struct file *file)
{
	/* Check if device is already open */
	if (test_and_set_bit(0, &es305->open))
		return -EBUSY;

	file->private_data = es305;
	return 0;
}

static int es305_release(struct inode *inode, struct file *file)
{
	clear_bit(0, &es305->open);
	return 0;
}

static int suspend(struct vp_ctxt *vp)
{
	int rc;

	/* Put es305 into sleep mode */
	rc = execute_cmdmsg(A100_msg_Sleep, vp);
	if (rc < 0) {
		pr_debug("%s: suspend error\n", __func__);
		goto set_suspend_err;
	}

	vp->suspended = 1;
	msleep(120); /* 120 defined by fig 2 of eS305 as the time to wait
			before clock gating */
	pr_debug("A1026 suspend\n");
	rc = intel_scu_ipc_set_osc_clk0(false, CLK0_AUDIENCE);
	if (rc)
		pr_err("ipc clk disable command failed: %d\n", rc);

set_suspend_err:
	return rc;
}

static ssize_t es305_bootup_init(struct vp_ctxt *vp, const char *firmware_name)
{
	int rc, pass = 0;
	int remaining;
	int retry = RETRY_CNT;
	int i;
	const u8 *index;
	u8 buf[2];
	const struct firmware *fw_entry;

	if (firmware_name == NULL || firmware_name[0] == '\0')
		firmware_name = "vpimg.bin";
	dev_dbg(&vp->i2c_dev->dev, "firmware file: %s\n", firmware_name);

	if (request_firmware(&fw_entry, firmware_name, &vp->i2c_dev->dev)) {
		dev_err(&vp->i2c_dev->dev, "Firmware not available\n");
		return -EFAULT;
	}

	if (fw_entry->size > A1026_MAX_FW_SIZE) {
		pr_err("%s: invalid es305 image size %d\n", __func__,
				fw_entry->size);
		return -EINVAL;
	}

	while (retry--) {
		/* Reset es305 chip */
		vp->pdata->reset(gpio_l);
		/* delay in order to wait for stable power supplies & for
		 * stable system clock */
		usleep_range(ES305_HARD_RESET_PERIOD, ES305_HARD_RESET_PERIOD);
		/* Take out of reset */
		vp->pdata->reset(gpio_h);
		msleep(50); /* Delay defined in Figure 1 of eS305 spec */


		/* Boot Cmd to es305 */
		buf[0] = A1026_msg_BOOT >> 8;
		buf[1] = A1026_msg_BOOT & 0xff;

		rc = es305_i2c_write(buf, 2, vp);
		if (rc < 0) {
			pr_err("%s: set boot mode error (%d retries left)\n",
					__func__, retry);
			continue;
		}

		mdelay(1); /* eS305 internal delay */
		rc = es305_i2c_read(buf, 1, vp);
		if (rc < 0) {
			pr_err("%s: boot mode ack error (%d retries left)\n",
					__func__, retry);
			continue;
		}

		if (buf[0] != A1026_msg_BOOT_ACK) {
			pr_err("%s: not a boot-mode ack (%d retries left)\n",
					__func__, retry);
			continue;
		}
		pr_debug("%s:ACK =  %d\n",
				__func__, buf[0]);
		remaining = fw_entry->size / I2C_SMBUS_BLOCK_MAX;
		index = fw_entry->data;

		pr_debug("%s: starting to load image (%d passes)...\n",
				__func__,
				remaining);

		for (i = 0; i < remaining; i++) {
			rc = es305_i2c_write(index, I2C_SMBUS_BLOCK_MAX, vp);
			index += I2C_SMBUS_BLOCK_MAX;
			if (rc < 0)
				break;
		}
		if (rc >= 0 && fw_entry->size % I2C_SMBUS_BLOCK_MAX)
			rc = es305_i2c_write(index, fw_entry->size %
				I2C_SMBUS_BLOCK_MAX, vp);

		if (rc < 0) {
			pr_err("%s: fw load error %d (%d retries left)\n",
					__func__, rc, retry);
			continue;
		}

		msleep(100); /* Delay time before issue a Sync Cmd
				BUGBUG should be 10*/

		pr_debug("%s: firmware loaded successfully\n", __func__);

		rc = execute_cmdmsg(A100_msg_Sync, vp);
		if (rc < 0) {
			pr_err("%s: sync command error %d (%d retries left)\n",
					__func__, rc, retry);
			continue;
		}

		pass = 1;
		break;
	}
	if (pass)
		pr_debug("%s: initialized!\n", __func__);
	else
		pr_err("%s: initialization failed\n", __func__);

	release_firmware(fw_entry);
	return rc;
}

static ssize_t chk_wakeup_es305(struct vp_ctxt *the_vp)
{
	int rc = 0, retry = 3;

	if (the_vp->suspended == 1) {
		the_vp->pdata->wakeup(gpio_l);
		msleep(30); /* es305b spec: need to wait 30ms after wake-up */

		do {
			rc = execute_cmdmsg(A100_msg_Sync, the_vp);
		} while ((rc < 0) && --retry);

		the_vp->pdata->wakeup(gpio_h);
		if (rc < 0) {
			pr_err("%s: failed (%d)\n", __func__, rc);
			goto wakeup_sync_err;
		}

		the_vp->suspended = 0;
	}

wakeup_sync_err:
	return rc;
}

int execute_cmdmsg(unsigned int msg, struct vp_ctxt *vp)
{
	int rc;
	int retries, pass = 0;
	u8 msgbuf[4];
	u8 chkbuf[4];
	unsigned int sw_reset;

	sw_reset = ((A100_msg_BootloadInitiate << 16) | RESET_IMMEDIATE);

	msgbuf[0] = (msg >> 24) & 0xFF;
	msgbuf[1] = (msg >> 16) & 0xFF;
	msgbuf[2] = (msg >> 8) & 0xFF;
	msgbuf[3] = msg & 0xFF;

	memcpy(chkbuf, msgbuf, 4);

	rc = es305_i2c_write(msgbuf, 4, vp);
	if (rc < 0) {
		pr_debug("%s: error %d\n", __func__, rc);
		es305_i2c_sw_reset(sw_reset, vp);
		return rc;
	}

	/* We don't need to get Ack after sending out a suspend command */
	if (msg == A100_msg_Sleep)
		return rc;

	retries = POLLING_RETRY_CNT;
	msleep(1); /* BUGBUG should be 20 p8 spec */
	while (retries--) {
		rc = 0;

		memset(msgbuf, 0, sizeof(msgbuf));
		rc = es305_i2c_read(msgbuf, 4, vp);
		if (rc < 0) {
			pr_debug("%s: ack-read error %d (%d retries)\n"
			, __func__, rc, retries);
			continue;
		}

		if (msgbuf[0] == 0x80  && msgbuf[1] == chkbuf[1]) {
			pass = 1;
			pr_debug("%s: ACK OF SYNC CMD\n", __func__);
			break;
		} else if (msgbuf[0] == 0xff && msgbuf[1] == 0xff) {
			pr_debug("%s: illegal cmd %08x\n", __func__, msg);
			rc = -EINVAL;
			break;
		} else if (msgbuf[0] == 0x00 && msgbuf[1] == 0x00) {
			pr_debug("%s: not ready (%d retries)\n", __func__,
					retries);
			rc = -EBUSY;
		} else {
			pr_debug("%s: cmd/ack mismatch: (%d retries left)\n",
					__func__,
					retries);
#if DEBUG
			pr_debug("%s: msgbuf[0] = %x\n", __func__, msgbuf[0]);
			pr_debug("%s: msgbuf[1] = %x\n", __func__, msgbuf[1]);
			pr_debug("%s: msgbuf[2] = %x\n", __func__, msgbuf[2]);
			pr_debug("%s: msgbuf[3] = %x\n", __func__, msgbuf[3]);
#endif
			rc = -EBUSY;
		}
		msleep(20); /* eS305 spec p. 8 : use polling */
	}

	if (!pass) {
		pr_err("%s: failed execute cmd %08x (%d)\n", __func__,
				msg, rc);
		es305_i2c_sw_reset(sw_reset, vp);
	}
	return rc;
}

static ssize_t es305_write(struct file *file, const char __user *buff,
	size_t count, loff_t *offp)
{
	int rc;
	unsigned int i, j, sw_reset;
	unsigned int nb_block, nb_sub_block;
	unsigned int msg_buf_count, size_cmd_snd, remaining;
	unsigned char *kbuf;
	struct vp_ctxt *the_vp;
#if DEBUG
	unsigned char msgbuf[4];
#endif

	sw_reset = ((A100_msg_BootloadInitiate << 16) | RESET_IMMEDIATE);

	mutex_lock(&es305->mutex);

	the_vp = file->private_data;
	if (!the_vp) {
		rc = -EINVAL;
		goto out_unlock;
	}

	rc = chk_wakeup_es305(the_vp);
	if (rc < 0)
		goto out_unlock;

	kbuf = kmalloc(count, GFP_KERNEL);
	if (!kbuf) {
		rc = -ENOMEM;
		goto out_unlock;
	}

	if (copy_from_user(kbuf, buff, count)) {
		rc = -EFAULT;
		goto out_kfree;
	}
#if DEBUG
		for (i = 0; i < count; i++)
			pr_debug("%s: kbuf %x\n", __func__, kbuf[i]);
#endif
	size_cmd_snd = ES305_I2C_CMD_FIFO_SIZE;
	msg_buf_count = 0;
	nb_block = count / size_cmd_snd;
	nb_sub_block = size_cmd_snd / I2C_SMBUS_BLOCK_MAX;
	for (i = 0; i < nb_block; i++) {
		for (j = 0; j < nb_sub_block; j++) {
			rc = es305_i2c_write(&kbuf[msg_buf_count],
				I2C_SMBUS_BLOCK_MAX, the_vp);
			if (rc < 0) {
				pr_err("ES305 CMD block write error!\n");
				es305_i2c_sw_reset(sw_reset, the_vp);
				goto out_kfree;
			}
			msg_buf_count += I2C_SMBUS_BLOCK_MAX;
			pr_debug("write OK block %d sub-block %d\n", i, j);
		}
		usleep_range(1*USEC_PER_MSEC, 2*USEC_PER_MSEC);
	}

	remaining = count - msg_buf_count;
	if (rc >= 0 && remaining) {
		nb_sub_block = remaining / I2C_SMBUS_BLOCK_MAX;
		for (i = 0; i < nb_sub_block; i++) {
			rc = es305_i2c_write(&kbuf[msg_buf_count],
				I2C_SMBUS_BLOCK_MAX, the_vp);
			if (rc < 0) {
				pr_err("ES305 CMD block write error!\n");
				es305_i2c_sw_reset(sw_reset, the_vp);
				goto out_kfree;
			}
			msg_buf_count += I2C_SMBUS_BLOCK_MAX;
			remaining -= I2C_SMBUS_BLOCK_MAX;
			pr_debug("write OK last block sub-block %d\n", i);
		}
	}

	if (rc >= 0 && remaining) {
		rc = es305_i2c_write(&kbuf[msg_buf_count], remaining, the_vp);
		if (rc < 0) {
			pr_err("ES305 CMD block write error!\n");
			es305_i2c_sw_reset(sw_reset, the_vp);
			goto out_kfree;
		}
		pr_debug("write OK remaining %d\n", remaining);
	}

	rc = count;
out_kfree:
	kfree(kbuf);
out_unlock:
	mutex_unlock(&es305->mutex);
	return rc;
}

static ssize_t es305_read(struct file *file, char __user *buff, size_t count,
		loff_t *offp)
{
	u8 *kbuf;
	struct vp_ctxt *the_vp;
	int rc;

	kbuf = kmalloc(count, GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;

	mutex_lock(&es305->mutex);

	the_vp = file->private_data;
	if (!the_vp) {
		mutex_unlock(&es305->mutex);
		rc = -EINVAL;
		goto out_kfree;
	}

	es305_i2c_read(kbuf, count, the_vp);

	mutex_unlock(&es305->mutex);
#if DEBUG
	{
		int i;
		for (i = 0; i < count; i++)
			pr_debug("%s: kbuf %x\n", __func__, kbuf[i]);
	}
#endif
	if (copy_to_user(buff, kbuf, count)) {
		rc = -EFAULT;
		goto out_kfree;
	}
	rc = count;

out_kfree:
	kfree(kbuf);
	return rc;
}

static long es305_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct vp_ctxt *the_vp;
	/* rc to u32 for 32bit->64bit safety as return value */
	u32 rc;
	char firmware_name[FIRMWARE_NAME_MAX_LENGTH];

	pr_debug("-> %s\n", __func__);


	if (file && file->private_data)
		the_vp = file->private_data;
	else
		return -EINVAL;
	pr_debug("%s: ioctl vp Ok\n", __func__);

	switch (cmd) {
	case A1026_BOOTUP_INIT:
		mutex_lock(&the_vp->mutex);
		rc = strncpy_from_user(firmware_name,
			(const char * __user) arg,
			FIRMWARE_NAME_MAX_LENGTH);
		if (rc == FIRMWARE_NAME_MAX_LENGTH)
			rc = -ERANGE;
		if (rc >= 0)
			rc = es305_bootup_init(the_vp, firmware_name);
		mutex_unlock(&the_vp->mutex);
		break;
	case A1026_SUSPEND:
		mutex_lock(&the_vp->mutex);
		rc = suspend(the_vp);
		mutex_unlock(&the_vp->mutex);
		if (rc < 0)
			pr_err("suspend error\n");
		break;
	case A1026_ENABLE_CLOCK:
		pr_debug("%s:ipc clk enable command\n", __func__);
		mutex_lock(&the_vp->mutex);
		rc = intel_scu_ipc_set_osc_clk0(true, CLK0_AUDIENCE);
		mutex_unlock(&the_vp->mutex);
		if (rc) {
			pr_err("ipc clk enable command failed: %d\n", rc);
			return rc;
		}
		break;
	default:
		pr_debug("%s: invalid command %d\n", __func__, _IOC_NR(cmd));
		rc = -EINVAL;
		break;
	}

	return rc;
}

static const struct file_operations a1026_fops = {
	.owner = THIS_MODULE,
	.open = es305_open,
	.release = es305_release,
	.write = es305_write,
	.read = es305_read,
	.unlocked_ioctl = es305_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = es305_ioctl,
#endif
	.llseek = no_llseek,
};

static struct miscdevice a1026_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "audience_es305",
	.fops = &a1026_fops,
};

static int a1026_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int rc;
	struct vp_ctxt *the_vp;
	struct a1026_platform_data *pdata;

	dev_dbg(&client->dev, "probe\n");

	the_vp = kzalloc(sizeof(struct vp_ctxt), GFP_KERNEL);
	if (!the_vp) {
		rc = -ENOMEM;
		dev_err(&client->dev, "platform data is out of memory\n");
		goto err_exit;
	}

	the_vp->i2c_dev = client;
	i2c_set_clientdata(client, the_vp);

	pdata = client->dev.platform_data;
	if (!pdata) {
		rc = -EINVAL;
		dev_err(&client->dev, "platform data is invalid\n");
		goto err_kfree;
	}

	rc = pdata->request_resources(client);
	if (rc) {
		dev_err(&client->dev, "Cannot get ressources\n");
		goto err_kfree;
	}

	mutex_init(&the_vp->mutex);
	rc = misc_register(&a1026_device);
	if (rc) {
		dev_err(&client->dev, "es305_device register failed\n");
		goto err_misc_register;
	}

	es305 = the_vp;
	es305->pdata = pdata;
	pdata->wakeup(gpio_h);
	pdata->reset(gpio_h);

	return 0;

err_misc_register:
	mutex_destroy(&the_vp->mutex);
err_kfree:
	kfree(the_vp);
	i2c_set_clientdata(client, NULL);
err_exit:
	return rc;
}

static int a1026_remove(struct i2c_client *client)
{
	struct a1026_platform_data *pdata;

	pdata = client->dev.platform_data;
	misc_deregister(&a1026_device);
	mutex_destroy(&es305->mutex);
	pdata->free_resources(client);
	kfree(i2c_get_clientdata(client));
	i2c_set_clientdata(client, NULL);
	return 0;
}

static int a1026_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}

static int a1026_resume(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id a1026_id[] = {
	{ "audience_es305", 0 },
	{ }
};

static struct i2c_driver a1026_driver = {
	.probe = a1026_probe,
	.remove = a1026_remove,
	.suspend = a1026_suspend,
	.resume	= a1026_resume,
	.id_table = a1026_id,
	.driver = {
		.name = "audience_es305",
	},
};

static int __init a1026_init(void)
{
	pr_debug("AUDIENCE%s\n", __func__);

	return i2c_add_driver(&a1026_driver);
}

static void __exit a1026_exit(void)
{
	i2c_del_driver(&a1026_driver);
}

module_init(a1026_init);
module_exit(a1026_exit);

MODULE_DESCRIPTION("A1026 voice processor driver");
MODULE_LICENSE("GPL");
