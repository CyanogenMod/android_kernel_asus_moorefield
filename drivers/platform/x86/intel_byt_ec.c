/*
 * intel_byt_e.c - Baytrail EC interface driver
 *
 * Copyright (C) 2013 Intel Corporation
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author: Ramakrishna Pallala <ramakrishna.pallala@intel.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/notifier.h>
#include <linux/sched.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/input.h>
#include <linux/acpi.h>
#include <linux/acpi_gpio.h>
#include <asm/intel_byt_ec.h>
#include <asm/intel_byt_buttons.h>

#define EC_SPACE_SIZE 256

/* EC status register */
#define BYT_EC_FLAG_OBF		0x01	/* Output buffer full */
#define BYT_EC_FLAG_IBF		0x02	/* Input buffer full */
#define BYT_EC_FLAG_IGN		0x04	/* Ignore the status */
#define BYT_EC_FLAG_CMD		0x08	/* Byte in data buffer is CMD */
#define BYT_EC_FLAG_BURST	0x10	/* burst mode */
#define BYT_EC_FLAG_SCI		0x20	/* EC-SCI occurred */
#define BYT_EC_FLAG_SMI		0x40	/* EC-SMI occurred */
#define BYT_EC_FLAG_IGN1	0x80	/* Ignore the status */

#define BYT_EC_DELAY		500	/* Wait 500ms max. during EC ops */
#define BYT_EC_UDELAY_GLK	1000	/* Wait 1ms max. to get global lock */
#define BYT_EC_MSI_UDELAY	550	/* Wait 550us for MSI EC */

#define BYT_EC_MAX_SCI_QUEUE		32

static void byt_ec_add_devices(void);
static BLOCKING_NOTIFIER_HEAD(byt_ec_evt_notifier_list);

struct ec_chip_info {
	struct platform_device	*pdev;
	struct mutex		io_lock;
	unsigned long		gpio;
	unsigned long		cmd_addr;
	unsigned long		data_addr;
	int			irq;
	wait_queue_head_t	wait;
};

static struct ec_chip_info *chip_ptr;

static inline u8 ec_read_status(struct ec_chip_info *chip)
{
	u8 x = inb(chip->cmd_addr);
	return x;
}

static inline u8 ec_read_data(struct ec_chip_info *chip)
{
	u8 x = inb(chip->data_addr);
	return x;
}

static inline void ec_write_cmd(struct ec_chip_info *chip, u8 command)
{
	outb(command, chip->cmd_addr);
}

static inline void ec_write_data(struct ec_chip_info *chip, u8 data)
{
	outb(data, chip->data_addr);
}

static int ec_check_ibf0(struct ec_chip_info *chip)
{
	u8 status = ec_read_status(chip);
	return (status & BYT_EC_FLAG_IBF) == 0;
}

static int ec_wait_ibf0(struct ec_chip_info *chip)
{
	unsigned long delay = jiffies + msecs_to_jiffies(BYT_EC_DELAY);
	unsigned long min_delay = msecs_to_jiffies(10);

	/* interrupt wait manually if GPE mode is not active */
	while (time_before(jiffies, delay))
		if (wait_event_timeout(chip->wait, ec_check_ibf0(chip),
					min_delay))
			return 0;

	return -ETIMEDOUT;
}

static int ec_check_obf1(struct ec_chip_info *chip)
{
	u8 status = ec_read_status(chip);
	return (status & BYT_EC_FLAG_OBF) == BYT_EC_FLAG_OBF;
}

static int ec_wait_obf1(struct ec_chip_info *chip)
{
	unsigned long delay = jiffies + msecs_to_jiffies(BYT_EC_DELAY);
	unsigned long min_delay = msecs_to_jiffies(10);

	/* interrupt wait manually if GPE mode is not active */
	while (time_before(jiffies, delay))
		if (wait_event_timeout(chip->wait, ec_check_obf1(chip),
					min_delay))
			return 0;

	return -ETIMEDOUT;
}

static int byt_ec_query(struct ec_chip_info *chip, u8 *val)
{
	int ret;

	ec_write_cmd(chip, BYT_EC_COMMAND_QUERY);
	ret = ec_wait_obf1(chip);
	if (ret < 0) {
		dev_err(&chip->pdev->dev, "EC tx/rx error%d\n", ret);
		return ret;
	}
	*val = ec_read_data(chip);

	return 0;
}

static int byt_ec_read8(struct ec_chip_info *chip, u8 addr, u8 *val)
{
	int ret;

	ec_write_cmd(chip, BYT_EC_COMMAND_READ);
	ret = ec_wait_ibf0(chip);
	if (ret < 0) {
		dev_err(&chip->pdev->dev, "EC tx/rx error%d\n", ret);
		return ret;
	}
	ec_write_data(chip, addr);
	ret = ec_wait_obf1(chip);
	if (ret < 0) {
		dev_err(&chip->pdev->dev, "EC tx/rx error%d\n", ret);
		return ret;
	}
	*val = ec_read_data(chip);

	return 0;
}

static int byt_ec_write8(struct ec_chip_info *chip, u8 addr, u8 val)
{
	int ret;

	ec_write_cmd(chip, BYT_EC_COMMAND_WRITE);
	ret = ec_wait_ibf0(chip);
	if (ret < 0) {
		dev_err(&chip->pdev->dev, "EC tx/rx error%d\n", ret);
		return ret;
	}

	ec_write_data(chip, addr);
	ret = ec_wait_ibf0(chip);
	if (ret < 0) {
		dev_err(&chip->pdev->dev, "EC tx/rx error%d\n", ret);
		return ret;
	}

	ec_write_data(chip, val);
	ret = ec_wait_ibf0(chip);
	if (ret < 0) {
		dev_err(&chip->pdev->dev, "EC tx/rx error%d\n", ret);
		return ret;
	}

	return 0;
}

int byt_ec_send_cmd(u8 command)
{
	int ret;

	if (!chip_ptr)
		return -EINVAL;

	mutex_lock(&chip_ptr->io_lock);

	ec_write_cmd(chip_ptr, command);
	ret = ec_wait_ibf0(chip_ptr);
	if (ret < 0)
		dev_err(&chip_ptr->pdev->dev, "EC tx/rx error%d\n", ret);

	mutex_unlock(&chip_ptr->io_lock);
	return ret;
}
EXPORT_SYMBOL(byt_ec_send_cmd);

int byt_ec_read_byte(u8 addr, u8 *val)
{
	int err;
	u8 temp_data;

	if (!chip_ptr)
		return -ENODEV;

	mutex_lock(&chip_ptr->io_lock);
	err = byt_ec_read8(chip_ptr, addr, &temp_data);
	mutex_unlock(&chip_ptr->io_lock);

	if (!err) {
		*val = temp_data;
		return 0;
	} else
		return err;
}
EXPORT_SYMBOL(byt_ec_read_byte);

int byt_ec_write_byte(u8 addr, u8 val)
{
	int err;

	if (!chip_ptr)
		return -ENODEV;

	mutex_lock(&chip_ptr->io_lock);
	err = byt_ec_write8(chip_ptr, addr, val);
	mutex_unlock(&chip_ptr->io_lock);

	return err;
}
EXPORT_SYMBOL(byt_ec_write_byte);

int byt_ec_read_word(u8 addr, u16 *val)
{
	int err;
	u8 temp_data;

	if (!chip_ptr)
		return -ENODEV;

	mutex_lock(&chip_ptr->io_lock);
	/* read byte 0 */
	err = byt_ec_read8(chip_ptr, addr, &temp_data);
	if (err)
		goto rd_word_err;
	*val = temp_data;
	/* read byte 1 */
	err = byt_ec_read8(chip_ptr, addr + 1, &temp_data);
	if (err)
		goto rd_word_err;
	*val |= ((u16)temp_data << 8);
	mutex_unlock(&chip_ptr->io_lock);

	return 0;

rd_word_err:
	mutex_unlock(&chip_ptr->io_lock);
	return err;
}
EXPORT_SYMBOL(byt_ec_read_word);

int byt_ec_write_word(u8 addr, u16 val)
{
	int err;

	if (!chip_ptr)
		return -ENODEV;

	mutex_lock(&chip_ptr->io_lock);
	/* write byte 0 */
	err = byt_ec_write8(chip_ptr, addr, (val & 0xff));
	if (err)
		goto wr_word_err;
	/* write byte 1 */
	err = byt_ec_write8(chip_ptr, addr, ((val >> 8) & 0xff));
	if (err)
		goto wr_word_err;
	mutex_unlock(&chip_ptr->io_lock);

	return 0;

wr_word_err:
	mutex_unlock(&chip_ptr->io_lock);
	return err;
}
EXPORT_SYMBOL(byt_ec_write_word);

static irqreturn_t ec_intr_thread_handler(int id, void *dev)
{
	struct ec_chip_info *chip = dev;
	int ret, i;
	u8 val = 0, stat;

	stat = ec_read_status(chip);

	if (stat & BYT_EC_FLAG_OBF) {
		dev_dbg(&chip->pdev->dev, "OBF event\n");
		wake_up(&chip->wait);
	}
	if (stat & BYT_EC_FLAG_IBF) {
		dev_dbg(&chip->pdev->dev, "IBF event\n");
		wake_up(&chip->wait);
	}
	if (stat & BYT_EC_FLAG_CMD)
		dev_dbg(&chip->pdev->dev, "CMD event\n");
	if (stat & BYT_EC_FLAG_BURST)
		dev_dbg(&chip->pdev->dev, "Burst event\n");
	if (stat & BYT_EC_FLAG_SMI)
		dev_dbg(&chip->pdev->dev, "SMI event\n");

	/* query event */
	for (i = 0; i < BYT_EC_MAX_SCI_QUEUE; i++) {
		if ((ec_read_status(chip) & BYT_EC_FLAG_SCI)) {
			mutex_lock(&chip->io_lock);
			ret = byt_ec_query(chip, &val);
			mutex_unlock(&chip->io_lock);
			/* Handle SCI query code */
			if (!ret)
				blocking_notifier_call_chain(
				&byt_ec_evt_notifier_list, val, chip);
			dev_info(&chip->pdev->dev, "query stat:%x\n", val);
			continue;
		}
		break;
	}

	return IRQ_HANDLED;
}

static void byt_enable_acpi_mode(struct ec_chip_info *chip)
{

	ec_write_cmd(chip, BYT_EC_ACPI_ENABLE);
	/* add 100mSec delay */
	mdelay(100);

	return;
}

static void byt_disable_acpi_mode(struct ec_chip_info *chip)
{

	ec_write_cmd(chip, BYT_EC_ACPI_DISABLE);
	/* add 100mSec delay */
	mdelay(100);

	return;
}

/* EC buttons platform data */
static struct byt_keys_button byt_m_nrpt_buttons[] = {
	{ KEY_POWER,		EV_KEY,	"Power_btn", 1 },
	{ },
};

static struct byt_keys_button byt_m_rpt_buttons[] = {
	{ KEY_VOLUMEUP,		EV_KEY,	"Volume_up", 1 },
	{ KEY_VOLUMEDOWN,	EV_KEY,	"Volume_down", 1 },
	{ KEY_HOME,		EV_KEY,	"Home_btn", 1 },
};

static struct byt_keys_platform_data byt_key_pdata[2] = {
	{
		.buttons = byt_m_nrpt_buttons,
		.nbuttons = 1,
		.rep = 0,
	}, {
		.buttons = byt_m_rpt_buttons,
		.nbuttons = 3,
		.rep = 1,
	},
};

static int byt_ec_create_device(const char *name, void *pdata)
{
	int ret = 0;
	struct platform_device *pdev = NULL;

	pdev = platform_device_alloc(name, -1);
	if (!pdev) {
		pr_err("out of memory for platform dev %s\n", name);
		goto dev_add_error;
	}

	pdev->dev.platform_data = pdata;
	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("failed to add %s platform device\n", name);
		platform_device_put(pdev);
	}

dev_add_error:

	return ret;
}

static void byt_ec_add_devices()
{
	/* Add battery device */
	byt_ec_create_device("ec_battery", NULL);

	/* Add button devices */
	byt_ec_create_device("byt_m_nrpt_btns", &byt_key_pdata[0]);
	byt_ec_create_device("byt_m_rpt_btns", &byt_key_pdata[1]);

	/* Add thermal device bellow */

	return;
}


void byt_ec_evt_register_notify(struct notifier_block *nb)
{
	blocking_notifier_chain_register(&byt_ec_evt_notifier_list, nb);
}
EXPORT_SYMBOL(byt_ec_evt_register_notify);

void byt_ec_evt_unregister_notify(struct notifier_block *nb)
{
	blocking_notifier_chain_unregister(&byt_ec_evt_notifier_list, nb);
}
EXPORT_SYMBOL(byt_ec_evt_unregister_notify);

static int byt_ec_probe(struct platform_device *pdev)
{
	struct ec_chip_info *chip;
	int ret;
	u8 val, stat;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&pdev->dev, "mem alloc failed\n");
		return -ENOMEM;
	}

	chip->pdev = pdev;
	platform_set_drvdata(pdev, chip);
	mutex_init(&chip->io_lock);
	init_waitqueue_head(&chip->wait);

	chip->gpio = acpi_get_gpio_by_index(&pdev->dev, 0, NULL);
	chip->data_addr = 0x62;
	chip->cmd_addr = 0x66;

	if (!request_region(chip->data_addr, 1, "ec-data")) {
		dev_err(&pdev->dev, "Could not request io port 0x%lx\n",
							chip->data_addr);
		kfree(chip);
		return -EBUSY;
	}
	if (!request_region(chip->cmd_addr, 1, "ec-cmd")) {
		dev_err(&pdev->dev, "Could not request io port 0x%lx\n",
							chip->cmd_addr);
		release_region(chip->data_addr, 1);
		kfree(chip);
		return -EBUSY;
	}

	/* enable EC acpi mode */
	byt_enable_acpi_mode(chip);

	/* read status register */
	stat = ec_read_status(chip);

	/* clear pending data */
	if (stat & BYT_EC_FLAG_OBF)
		ret = ec_read_data(chip);

	/* clear pending event */
	if (stat & BYT_EC_FLAG_SCI) {
		ret = byt_ec_query(chip, &val);
		if (!ret)
			dev_info(&chip->pdev->dev, "query stat:%x\n", val);
		else
			dev_info(&chip->pdev->dev, "query err:%d\n", ret);
	}

	chip->irq = gpio_to_irq(chip->gpio);
	ret = request_threaded_irq(chip->irq, NULL,
					ec_intr_thread_handler,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					"byt-ec", chip);
	if (ret) {
		dev_warn(&chip->pdev->dev,
			"cannot get IRQ:%d\n", chip->irq);
		chip->irq = -1;
	} else {
		dev_info(&chip->pdev->dev, "IRQ No:%d\n", chip->irq);
	}

	chip_ptr = chip;
	byt_ec_add_devices();

	return 0;
}

static int byt_ec_remove(struct platform_device *pdev)
{
	struct ec_chip_info *chip = platform_get_drvdata(pdev);

	/* disable EC acpi mode */
	byt_disable_acpi_mode(chip);
	free_irq(chip->irq, chip);
	release_region(chip->cmd_addr, 1);
	release_region(chip->data_addr, 1);
	kfree(chip);
	return 0;
}

static struct acpi_device_id byt_ec_acpi_match[] = {
	{ "BYTEC001", 0 },
	{ },
};
MODULE_DEVICE_TABLE(acpi, byt_ec_acpi_match);


static struct platform_driver byt_ec_driver = {
	.probe = byt_ec_probe,
	.remove = byt_ec_remove,
	.driver = {
		.name = "byt-ec",
		.acpi_match_table = ACPI_PTR(byt_ec_acpi_match),
	},
};

static int __init byt_ec_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&byt_ec_driver);
	if (ret < 0) {
		pr_err("platform driver reg failed %s\n",
					"byt-ec");
		return ret;
	}
	return 0;
}
fs_initcall(byt_ec_init);

static void __exit byt_ec_exit(void)
{
	platform_driver_unregister(&byt_ec_driver);
}
module_exit(byt_ec_exit);

MODULE_AUTHOR("Ramakrishna Pallala <ramakrishna.pallala@intel.com>");
MODULE_DESCRIPTION("Baytrail EC Battery Driver");
MODULE_LICENSE("GPL");
