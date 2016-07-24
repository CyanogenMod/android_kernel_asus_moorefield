/*
 *  pww_world_led.c - PWM driver for world LED
 *
 *  Copyright (C) 2011-13 Intel Corp
 *  Author: Yun Wei <yun.wei@intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/acpi.h>
#include <linux/pm_runtime.h>
#include <linux/lnw_gpio.h>
#include <linux/intel_pwm_world_led.h>
#include <asm/intel-mid.h>
#include <linux/leds.h>
#include "pwm_world_led.h"

union sst_pwmctrl_reg {
	struct {
		u32 pwmtd:8;
		u32 pwmbu:22;
		u32 pwmswupdate:1;
		u32 pwmenable:1;
	} part;
	u32 full;
};

#define DEFAULT_BASE_UNIT 0x80
#define DEFAULT_DUTY_CYCLE 0x40
#define MAX_BRIGHTNESS 15

static u32 brightness_tbl[] = {
	255, 237, 219, 201, 183,
	165, 147, 129, 111, 93,
	75, 57, 39, 21, 0
};

unsigned long pwm_base_unit = DEFAULT_BASE_UNIT;
unsigned long pwm_duty_cycle = DEFAULT_DUTY_CYCLE;
enum led_brightness current_brightness;

static struct led_classdev cdev_world_led;

struct world_led_info *g_pwm_info;

static int world_led_configure(struct world_led_info *info, bool enable)
{
	union sst_pwmctrl_reg pwmctrl;

	if (enable) {
		lnw_gpio_set_alt(info->gpio_pwm, info->alt_fn);

		/*1. Enable the PWM by setting PWM enable bit to 1 */
		pwmctrl.full = readl(info->shim);
		pr_debug("WorldLed:Read pwmctrl %x\n", readl(info->shim));
		pwmctrl.part.pwmenable = 1;
		writel(pwmctrl.full, info->shim);

		/*2. Read the PWM register to make sure there is no pending
		*update.
		*/
		pwmctrl.full = readl(info->shim);
		pr_debug("Read pwmctrl %x\n", pwmctrl.full);

		/*check pwnswupdate bit */
		if (pwmctrl.part.pwmswupdate)
			return -EBUSY;
		/*Base unit == 1*/
		pwmctrl.part.pwmswupdate = 0x1;

		/* validate values input */
		*info->duty_cycle = current_brightness;
		if (*info->base_unit > info->max_base_unit)
			*info->base_unit = info->max_base_unit;
		if (*info->duty_cycle > info->max_duty_cycle)
			*info->duty_cycle = info->max_duty_cycle;
		pwmctrl.part.pwmbu = *info->base_unit;
		pwmctrl.part.pwmtd = *info->duty_cycle;
		writel(pwmctrl.full,  info->shim);
		pr_debug("Read pwmctrl %x\n", pwmctrl.full);
	} else { /*disable PWM block */
		lnw_gpio_set_alt(info->gpio_pwm, 0);

		/*1. setting PWM enable bit to 0 */
		pwmctrl.full = readl(info->shim);
		pwmctrl.part.pwmenable = 0;
		writel(pwmctrl.full,  info->shim);
	}
	return 0;
}

static void world_led_enable(struct world_led_info *info)
{
	pr_debug("%s: Enable\n", __func__);
	mutex_lock(&info->lock);
	pm_runtime_get_sync(info->dev);

	/* Wait for 850us per spec, give 100us buffer */
	usleep_range(950, 1000);

	/* Enable the Trigger line */
	info->pwm_configure(info, true);

	info->enabled = true;
	mutex_unlock(&info->lock);
}

static void world_led_disable(struct world_led_info *info)
{
	pr_debug("%s: Disable\n", __func__);
	mutex_lock(&info->lock);
	info->enabled = false;
	info->pwm_configure(info, false);
	pm_runtime_put(info->dev);
	mutex_unlock(&info->lock);
}

static void world_led_update(struct world_led_info *info)
{
	union sst_pwmctrl_reg pwmctrl;

	pr_debug("%s: Update\n", __func__);

	mutex_lock(&info->lock);

	*info->duty_cycle = current_brightness;

	/* Program the Base Unit and On Time Divisor values */
	pwmctrl.full = readl(info->shim);
	pr_debug("Read pwmctrl %x\n", readl(info->shim));
	pwmctrl.part.pwmbu = *info->base_unit;
	pwmctrl.part.pwmtd = *info->duty_cycle;
	writel(pwmctrl.full,  info->shim);
	pr_debug("Updated pwmctrl: %x\n", pwmctrl.full);

	/* Set the Software update Bit */
	pwmctrl.part.pwmswupdate = 0x1;
	writel(pwmctrl.full,  info->shim);

	mutex_unlock(&info->lock);
}

/*******************************************************************************
 * SYSFS                                                                       *
 ******************************************************************************/
static void world_set_brightness(struct led_classdev *led,
				enum led_brightness value)
{
	struct world_led_info *info = g_pwm_info;
	pr_debug("%s: Set world led brightness to %d", __func__, value);

	if (value == 0) {
		world_led_disable(info);
	} else {
		current_brightness = brightness_tbl[value];

		/*
		 * If we're previously disabled then do enable,
		 * otherwise just update
		 */
		if (info->enabled == false)
			world_led_enable(info);
		else
			world_led_update(info);
	}
}

static DEVICE_ULONG_ATTR(pwm_baseunit, S_IRUGO | S_IWUSR, pwm_base_unit);
static DEVICE_ULONG_ATTR(pwm_ontime_div, S_IRUGO | S_IWUSR, pwm_duty_cycle);

static struct attribute *world_led_attrs[] = {
	&dev_attr_pwm_baseunit.attr.attr,
	&dev_attr_pwm_ontime_div.attr.attr,
	0,
};

static const struct attribute_group world_led_attr_group = {
	.attrs = world_led_attrs,
};

#if CONFIG_PM
static int world_led_runtime_suspend(struct device *dev)
{
	struct world_led_info *info = dev_get_drvdata(dev);

	pr_debug("In %s\n", __func__);

	info->pwm_configure(info, false);
	return 0;
}

static int world_led_runtime_resume(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	return 0;
}

static void world_led_complete(struct device *dev)
{
	pr_debug("In %s\n", __func__);
}

static const struct dev_pm_ops world_led_pm_ops = {
	.prepare = world_led_runtime_suspend,
	.complete = world_led_complete,
	.runtime_suspend = world_led_runtime_suspend,
	.runtime_resume = world_led_runtime_resume,
};
#endif

struct world_led_info *world_led_setup(struct device *dev, struct world_led_pdata *data)
{
	struct world_led_info *info;

	info =  devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info) {
		pr_err("%s: no memory for driver context", __func__);
		return NULL;
	}

	info->alt_fn = data->alt_fn;
	info->ext_drv = data->ext_drv;
	info->gpio_pwm = data->gpio_pwm;
	info->name = data->name;

	info->dev = dev;
	mutex_init(&info->lock);
	info->world_led_attr_group = &world_led_attr_group;

	pwm_base_unit = data->base_unit;
	pwm_duty_cycle = data->time_divisor;
	info->base_unit = &pwm_base_unit;
	info->duty_cycle = &pwm_duty_cycle;

	info->enable = world_led_enable;
	info->disable = world_led_disable;

	info->pwm_configure = world_led_configure;

	info->max_base_unit = INTEL_WORLD_LED_MAX_BASEUNIT;
	info->max_duty_cycle = INTEL_WORLD_LED_MAX_TIMEDIVISOR;

	return info;
}


static int world_led_probe(struct pci_dev *pci,
			const struct pci_device_id *pci_id)
{
	struct world_led_info *info;
	struct device *dev = &pci->dev;
	struct world_led_pdata *data;
	struct led_classdev *led;
	int ret;

	data = pci->dev.platform_data;
	if (!data) {
		dev_err(&pci->dev, "Failed to get world_led platform data\n");
		return -ENODEV;
	}

	info = world_led_setup(dev, data);
	if (!info)
		return -ENODEV;

	/* Init the device */
	ret = pci_enable_device(pci);
	if (ret) {
		pr_err("PWM device can't be enabled\n");
		goto do_freegpio_world_led_info;
	}

	ret = pci_request_regions(pci, INTEL_WORLD_LED_DRV_NAME);
	if (ret)
		goto do_disable_device;
	pci_dev_get(pci);

	/* world led Shim */
	info->shim =  pci_ioremap_bar(pci, 0);
	if (!info->shim) {
		pr_err("ioremap failed for world_led driver\n");
		goto do_release_regions;
	}

	ret = sysfs_create_group(&dev->kobj, info->world_led_attr_group);
	if (ret) {
		pr_err("PWM could not register sysfs files\n");
		goto do_unmap_shim;
	}

	pci_set_drvdata(pci, info);
	pm_runtime_allow(&pci->dev);
	pm_runtime_put_noidle(&pci->dev);

	g_pwm_info = info;

	/* Create LED device */
	led = &cdev_world_led;
	led->name = "world";
	led->brightness = 0;
	led->max_brightness = MAX_BRIGHTNESS;
	led->brightness_set = world_set_brightness;
	if (led_classdev_register(dev, led) < 0)
		dev_err(dev, "error registering LED %s\n", led->name);

	return ret;

do_unmap_shim:
	iounmap(info->shim);
do_release_regions:
	pci_release_regions(pci);
do_disable_device:
	pci_disable_device(pci);
do_freegpio_world_led_info:
	devm_kfree(dev, info);

	return ret;
}

static void world_led_remove(struct pci_dev *pci)
{
	struct world_led_info *info = pci_get_drvdata(pci);
	sysfs_remove_group(&info->dev->kobj, info->world_led_attr_group);
	iounmap(info->shim);
	devm_kfree(&pci->dev, info);
	pci_release_regions(pci);
	pci_disable_device(pci);
	pci_set_drvdata(pci, NULL);
}

/* PCI Routines */
static DEFINE_PCI_DEVICE_TABLE(intel_world_led_ids) = {
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_WORLD_LED), 0},
	{ 0, }
};

MODULE_DEVICE_TABLE(pci, intel_world_led_ids);

static struct pci_driver world_led_driver = {
	.name = INTEL_WORLD_LED_DRV_NAME,
	.id_table = intel_world_led_ids,
	.probe = world_led_probe,
	.remove = world_led_remove,
#ifdef CONFIG_PM
	.driver = {
		.pm = &world_led_pm_ops,
	},
#endif
};



/**
* world_led_init - Module init function
*
* Registers with PCI
* Registers platform
* Init all data strutures
*/
static int __init world_led_init(void)
{
	int ret = 0;

	/* Register with PCI */
	ret = pci_register_driver(&world_led_driver);
	if (ret)
		pr_err("PCI register failed\n");

	return ret;
}

/**
* world_led_exit - Module exit function
*
* Unregisters with PCI
* Unregisters platform
* Frees all data strutures
*/
static void __exit world_led_exit(void)
{
	pci_unregister_driver(&world_led_driver);
	pr_debug("world_led driver exited\n");
	return;
}

late_initcall(world_led_init);
module_exit(world_led_exit);

MODULE_ALIAS("pci:world_led");
MODULE_DESCRIPTION("Intel(R) world led driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Yun Wei <yun.wei@intel.com>");
