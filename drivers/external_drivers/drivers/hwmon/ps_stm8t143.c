/*
 * STM8T143 proximity sensor driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/sysfs.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/acpi.h>
#include <linux/acpi_gpio.h>

/*
* gpio pin number of output data
* ctrl input pin is always active after power on
*
* Byt-cr RVP board,94 for Baylake board
* Get from ACPI if BIOS provided
* #define	PS_STM8T143_DATA_GPIO 	149
* #define	PS_STM8T143_LPM_GPIO 	122
*/

#define PS_STM8T143_DRIVER_NAME "stm8t143"
#define PS_STM8T143_INPUT_NAME 	"stm8t143"

/*Enable when ctrl gpio is ready*/
static bool ps_lpm_enable = 0;

#ifdef CONFIG_PS_STM8T143_DEBUG
static unsigned int debug_level = 0;
#define DBG_LEVEL1		1
#define DBG_LEVEL2		2
#define DBG_LEVEL3		3
#define DBG_LEVEL4		4
#define SENSOR_DBG(level, ...)				\
do {							\
	if (level <= debug_level)			\
		printk(KERN_DEBUG "<stm8t143>[%d]%s"	\
			 "\n",				\
			__LINE__, __func__,		\
			##__VA_ARGS__);			\
} while (0)

#else
#define SENSOR_DBG(level, ...)
#endif

struct stm8t143_data {
	struct platform_device *pdev;
	struct input_dev *input_dev;
	struct mutex lock;
	int enabled;
	int enabled_suspend; //state before suspend
	int gpio_data;
	int gpio_irq;
	int gpio_ctrl;
};

static int stm8t143_init(struct stm8t143_data *stm8t143)
{
	int ret = 0;
	SENSOR_DBG(DBG_LEVEL3);

	if (ps_lpm_enable) {
		/* put PS sensor into low power mode if not used */
		ret = gpio_direction_output(stm8t143->gpio_ctrl, 1);
		if (ret < 0) {
			dev_err(&stm8t143->pdev->dev,
			"stm8t143 gpio direction output 0 failed with %d\n", ret);
		}
	}
	return ret;
}

static void stm8t143_get_data(struct stm8t143_data *stm8t143, s32 *data)
{
	*data = gpio_get_value(stm8t143->gpio_data);
	SENSOR_DBG(DBG_LEVEL2, "data=%d\n", *data);
}

/*since hw is always in work, so enable&disable in sw itself*/
static void stm8t143_enable(struct stm8t143_data *stm8t143)
{
	int data;
	int ret;

	SENSOR_DBG(DBG_LEVEL3);

	mutex_lock(&stm8t143->lock);
	//gpio_set_value(stm8t143->gpio_data, 0);
	if (!stm8t143->enabled) {
		int irq = gpio_to_irq(stm8t143->gpio_irq);
		enable_irq(irq);
		if (ps_lpm_enable) {
			ret = gpio_direction_output(stm8t143->gpio_ctrl, 0);
			if (ret < 0) {
				dev_err(&stm8t143->pdev->dev,
				"stm8t143 gpio direction output 0 failed with %d\n", ret);
			}
		}
		stm8t143->enabled = 1;
	}
	mutex_unlock(&stm8t143->lock);

	/*input first data*/
	stm8t143_get_data(stm8t143, &data);
	input_report_abs(stm8t143->input_dev, ABS_X, data);
	input_sync(stm8t143->input_dev);
}

static void stm8t143_disable(struct stm8t143_data *stm8t143)
{
	int ret;
	SENSOR_DBG(DBG_LEVEL3);

	mutex_lock(&stm8t143->lock);
	//gpio_set_value(stm8t143->gpio_data, 1);
	if (stm8t143->enabled) {
		int irq = gpio_to_irq(stm8t143->gpio_irq);
		if (ps_lpm_enable) {
			/* the CTRL pin is tied high for Halt conversion mode */
			ret = gpio_direction_output(stm8t143->gpio_ctrl, 1);
			if (ret < 0) {
				dev_err(&stm8t143->pdev->dev,
				"stm8t143 gpio direction output 0 failed with %d\n", ret);
			}
		}
		disable_irq(irq);
		stm8t143->enabled = 0;
	}

	mutex_unlock(&stm8t143->lock);
}

static irqreturn_t stm8t143_irq(int irq, void *dev_id)
{
	struct stm8t143_data *stm8t143 = (struct stm8t143_data*)dev_id;
	int data;

	stm8t143_get_data(stm8t143, &data);
	input_report_abs(stm8t143->input_dev, ABS_X, data);
	input_sync(stm8t143->input_dev);

	return IRQ_HANDLED;
}

static int stm8t143_input_init(struct stm8t143_data *stm8t143)
{
	int ret;
	struct input_dev *input;

	SENSOR_DBG(DBG_LEVEL3);

	input = input_allocate_device();
	if (!input) {
		dev_err(&stm8t143->pdev->dev, "input device allocate failed\n");
		return -ENOMEM;
	}
	input->name = PS_STM8T143_INPUT_NAME;
	input->dev.parent = &stm8t143->pdev->dev;
	set_bit(EV_ABS, input->evbit);
	set_bit(ABS_X, input->absbit);

	ret = input_register_device(input);
	if (ret) {
		dev_err(&stm8t143->pdev->dev,
			"unable to register input device %s:%d\n",
			input->name, ret);
		goto err;
	}

	stm8t143->input_dev = input;
	return 0;
err:
	input_free_device(input);
	return ret;
}

static int stm8t143_get_data_init(struct stm8t143_data *stm8t143)
{
	int ret;
	int irq;

	gpio_direction_input(stm8t143->gpio_irq);
	irq = gpio_to_irq(stm8t143->gpio_irq);

	irq_set_status_flags(irq, IRQ_NOAUTOEN);
	ret = request_threaded_irq(irq, NULL, stm8t143_irq,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			PS_STM8T143_DRIVER_NAME, stm8t143);
	if (ret < 0) {
		gpio_free(stm8t143->gpio_irq);
		dev_err(&stm8t143->pdev->dev,
			"Fail to request irq:%d ret=%d\n", irq, ret);
	}
	return ret;
}

static ssize_t stm8t143_lpm_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ps_lpm_enable);
}

static ssize_t stm8t143_lpm_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	struct stm8t143_data *stm8t143 = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	if (val) {
		if (stm8t143->gpio_ctrl >= 0)
			ps_lpm_enable = 1;
	} else
		ps_lpm_enable = 0;

	return count;
}

static ssize_t stm8t143_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct stm8t143_data *stm8t143 = dev_get_drvdata(dev);
	int enabled;

	enabled = stm8t143->enabled;
	return sprintf(buf, "%d\n", stm8t143->enabled);
}

static ssize_t stm8t143_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct stm8t143_data *stm8t143 = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	if (val)
		stm8t143_enable(stm8t143);
	else
		stm8t143_disable(stm8t143);

	return count;
}

/*Avoid file operation error in HAL layer*/
static ssize_t stm8t143_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int delay = -1;
	return sprintf(buf, "%d\n", delay);
}

static ssize_t stm8t143_delay_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static DEVICE_ATTR(poll, S_IRUGO|S_IWUSR, stm8t143_delay_show,
		stm8t143_delay_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR, stm8t143_enable_show,
		stm8t143_enable_store);
static DEVICE_ATTR(lpm_enable, S_IRUGO|S_IWUSR, stm8t143_lpm_enable_show,
		stm8t143_lpm_enable_store);

static struct attribute *stm8t143_attributes[] = {
	&dev_attr_poll.attr,
	&dev_attr_enable.attr,
	&dev_attr_lpm_enable.attr,
	NULL
};

static struct attribute_group stm8t143_attribute_group = {
	.attrs = stm8t143_attributes
};

static int stm8t143_probe(struct platform_device *pdev)
{
	int err;
	struct stm8t143_data *stm8t143;

	SENSOR_DBG(DBG_LEVEL3);

	stm8t143 = kzalloc(sizeof(struct stm8t143_data), GFP_KERNEL);
	if (!stm8t143) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, stm8t143);

	stm8t143->pdev = pdev;
	mutex_init(&stm8t143->lock);
	stm8t143->gpio_irq = acpi_get_gpio_by_index(&pdev->dev, 0, NULL);
	if (stm8t143->gpio_irq < 0) {
		dev_err(&pdev->dev, "Fail to get irq gpio pin by ACPI\n");
		err = -EINVAL;
		goto err_gpio;
	}
	err = gpio_request(stm8t143->gpio_irq, PS_STM8T143_DRIVER_NAME);
	if (err < 0) {
		dev_err(&pdev->dev, "stm8t143 gpio request failed with %d\n", err);
		goto err_gpio;
	}
	stm8t143->gpio_data = stm8t143->gpio_irq;

	stm8t143->gpio_ctrl = acpi_get_gpio_by_index(&pdev->dev, 1, NULL);
	if (stm8t143->gpio_ctrl < 0) {
		dev_warn(&pdev->dev, "Fail to get ctrl gpio pin by ACPI\n");
	} else {
		err = gpio_request(stm8t143->gpio_ctrl, PS_STM8T143_DRIVER_NAME);
		if (err < 0) {
			dev_err(&pdev->dev, "stm8t143 gpio request failed with %d\n", err);
			stm8t143->gpio_ctrl = -1;
		}
	}
	if (stm8t143->gpio_ctrl >= 0)
		ps_lpm_enable = 1;

	SENSOR_DBG(DBG_LEVEL3, "data gpio:%d\n", stm8t143->gpio_data);
	SENSOR_DBG(DBG_LEVEL3, "ctrl gpio:%d\n", stm8t143->gpio_ctrl);

	err = stm8t143_init(stm8t143);
	if (err < 0) {
		dev_err(&pdev->dev, "stm8t143_initchip failed with %d\n", err);
		goto err_init;
	}

	err = stm8t143_input_init(stm8t143);
	if (err < 0) {
		dev_err(&pdev->dev, "input init error\n");
		goto err_init;
	}

	err = stm8t143_get_data_init(stm8t143);
	if (err < 0) {
		dev_err(&pdev->dev, "input init error\n");
		goto err_data_init;
	}

	err = sysfs_create_group(&pdev->dev.kobj, &stm8t143_attribute_group);
	if (err) {
		dev_err(&pdev->dev, "sysfs can not create group\n");
		goto err_sys_init;
	}

	return 0;

err_sys_init:
	free_irq(stm8t143->gpio_irq, stm8t143);
err_data_init:
	input_unregister_device(stm8t143->input_dev);
err_init:
	gpio_free(stm8t143->gpio_irq);
	if (stm8t143->gpio_ctrl >= 0)
		gpio_free(stm8t143->gpio_ctrl);
err_gpio:
	kfree(stm8t143);
	return err;
}

static int stm8t143_remove(struct platform_device *pdev)
{
	struct stm8t143_data *stm8t143 = platform_get_drvdata(pdev);
	int irq = gpio_to_irq(stm8t143->gpio_irq);

	stm8t143_disable(stm8t143);
	sysfs_remove_group(&pdev->dev.kobj, &stm8t143_attribute_group);
	input_unregister_device(stm8t143->input_dev);
	free_irq(irq, stm8t143);
	gpio_free(stm8t143->gpio_irq);
	if (stm8t143->gpio_ctrl >= 0)
		gpio_free(stm8t143->gpio_ctrl);
	kfree(stm8t143);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int stm8t143_suspend(struct device *dev)
{
	struct stm8t143_data *stm8t143 = dev_get_drvdata(dev);

	stm8t143->enabled_suspend = stm8t143->enabled;
	stm8t143_disable(stm8t143);
	return 0;
}

static int stm8t143_resume(struct device *dev)
{
	struct stm8t143_data *stm8t143 = dev_get_drvdata(dev);

	if (stm8t143->enabled_suspend)
		stm8t143_enable(stm8t143);
	return 0;
}
static SIMPLE_DEV_PM_OPS(stm8t143_pm, stm8t143_suspend, stm8t143_resume);
#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_ACPI
static const struct acpi_device_id ps_stm8t143_acpi_ids[] = {
	{ "SRCL0001", 0 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, ps_stm8t143_acpi_ids);
#endif

static struct platform_driver ps_stm8t143_driver = {
	.probe		= stm8t143_probe,
	.remove		= stm8t143_remove,
	.driver	= {
		.name	= PS_STM8T143_DRIVER_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM_SLEEP
		.pm	= &stm8t143_pm,
#endif
#ifdef CONFIG_ACPI
		.acpi_match_table = ACPI_PTR(ps_stm8t143_acpi_ids),
#endif
	},
};

static int __init ps_stm8t143_init(void)
{
	int ret;

	ret = platform_driver_register(&ps_stm8t143_driver);
	if (ret < 0)
		printk(KERN_ERR "Fail to register ps stm8t143 driver\n");

	return ret;
}

static void __exit ps_stm8t143_exit(void)
{
	platform_driver_unregister(&ps_stm8t143_driver);
}

module_init(ps_stm8t143_init);
module_exit(ps_stm8t143_exit);

MODULE_AUTHOR("IO&Sensor");
MODULE_DESCRIPTION("Proximity Sensor STM8T143");
MODULE_LICENSE("GPL V2");
