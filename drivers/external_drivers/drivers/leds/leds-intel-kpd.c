/*
 * leds-intel-kpd.c - Intel Keypad LED driver
 *
 * Copyright (C) 2011 Intel Corporation
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/leds.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/rpmsg.h>
#include <linux/early_suspend_sysfs.h>

#include <asm/intel_scu_pmic.h>
#include <asm/intel_mid_pwm.h>
#include <asm/intel_mid_rpmsg.h>
#include <asm/intel_mid_remoteproc.h>

static void intel_keypad_led_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	intel_mid_pwm(PWM_LED, value);
}

static struct led_classdev intel_kpd_led = {
	.name			= "intel_keypad_led",
	.brightness_set		= intel_keypad_led_set,
	.brightness		= LED_OFF,
	.max_brightness		= 15,
};

static void intel_kpd_led_early_suspend(struct early_suspend *h)
{
	led_classdev_suspend(&intel_kpd_led);
}

static void intel_kpd_led_late_resume(struct early_suspend *h)
{
	led_classdev_resume(&intel_kpd_led);
}

static ssize_t early_suspend_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	if (!strncmp(buf, EARLY_SUSPEND_ON, EARLY_SUSPEND_STATUS_LEN))
		led_classdev_suspend(&intel_kpd_led);
	else if (!strncmp(buf, EARLY_SUSPEND_OFF, EARLY_SUSPEND_STATUS_LEN))
		led_classdev_resume(&intel_kpd_led);

	return count;
}

static DEVICE_EARLY_SUSPEND_ATTR(early_suspend_store);

static struct early_suspend intel_kpd_led_suspend_desc = {
	.level   = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
	.suspend = intel_kpd_led_early_suspend,
	.resume  = intel_kpd_led_late_resume,
};

static int intel_kpd_led_rpmsg_probe(struct rpmsg_channel *rpdev)
{
	int ret;

	dev_info(&rpdev->dev, "Probed kpd_led rpmsg device\n");

	ret = led_classdev_register(&rpdev->dev, &intel_kpd_led);
	if (ret) {
		dev_err(&rpdev->dev, "register kpd_led failed");
		return ret;
	}

	device_create_file(&rpdev->dev, &dev_attr_early_suspend);
	intel_keypad_led_set(&intel_kpd_led, intel_kpd_led.brightness);
	register_early_suspend(&intel_kpd_led_suspend_desc);
	register_early_suspend_device(&rpdev->dev);

	return 0;
}

static void intel_kpd_led_rpmsg_remove(struct rpmsg_channel *rpdev)
{
	intel_keypad_led_set(&intel_kpd_led, LED_OFF);
	device_remove_file(&rpdev->dev, &dev_attr_early_suspend);
	unregister_early_suspend(&intel_kpd_led_suspend_desc);
	unregister_early_suspend_device(&rpdev->dev);
	led_classdev_unregister(&intel_kpd_led);
}

static void intel_kpd_led_rpmsg_cb(struct rpmsg_channel *rpdev, void *data,
					int len, void *priv, u32 src)
{
	dev_warn(&rpdev->dev, "unexpected, message\n");

	print_hex_dump(KERN_DEBUG, __func__, DUMP_PREFIX_NONE, 16, 1,
		       data, len,  true);
}
static struct rpmsg_device_id intel_kpd_led_id_table[] = {
	{ .name	= "rpmsg_kpd_led" },
	{ },
};

static struct rpmsg_driver intel_kpd_led_rpmsg = {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= intel_kpd_led_id_table,
	.probe		= intel_kpd_led_rpmsg_probe,
	.callback	= intel_kpd_led_rpmsg_cb,
	.remove		= intel_kpd_led_rpmsg_remove,
};

static int __init intel_kpd_led_rpmsg_init(void)
{
	return register_rpmsg_driver(&intel_kpd_led_rpmsg);
}

module_init(intel_kpd_led_rpmsg_init);

static void __exit intel_kpd_led_rpmsg_exit(void)
{
	return unregister_rpmsg_driver(&intel_kpd_led_rpmsg);
}
module_exit(intel_kpd_led_rpmsg_exit);

MODULE_DESCRIPTION("Intel Keypad LED Driver");
MODULE_LICENSE("GPL v2");
