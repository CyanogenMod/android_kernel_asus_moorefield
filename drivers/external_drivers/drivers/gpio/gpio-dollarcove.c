/*
 *
 * Copyright (C) 2014 Intel Corporation
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
 * Author: Bin Yang <bin.yang@intel.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/sched.h>
#include <linux/mfd/intel_mid_pmic.h>
#include <linux/gpio.h>
#include <asm/intel_vlv2.h>

#define NUM_GPIO		2
#define GPIO0_CTL		0x90
#define GPIO0_LDO		0x91
#define GPIO_STATUS		0x94

static int dollarcove_gpio_direction_input(struct gpio_chip *chip,
		unsigned gpio)
{
	int ctl = GPIO0_CTL + 2 * gpio;
	int ldo = GPIO0_LDO + 2 * gpio;

	intel_mid_pmic_writeb(ldo, 0xb);
	intel_mid_pmic_writeb(ctl, 0x2);
	return 0;
}

static int dollarcove_gpio_direction_output(struct gpio_chip *chip,
		unsigned gpio, int value)
{
	int ctl = GPIO0_CTL + 2 * gpio;
	int ldo = GPIO0_LDO + 2 * gpio;

	intel_mid_pmic_writeb(ldo, 0xb);
	intel_mid_pmic_writeb(ctl, value ? 1 : 0);
	return 0;
}

static int dollarcove_gpio_get(struct gpio_chip *chip, unsigned gpio)
{
	return intel_mid_pmic_readb(GPIO_STATUS) & (1 << gpio);
}

static void dollarcove_gpio_set(struct gpio_chip *chip,
		unsigned gpio, int value)
{
	int ctl = GPIO0_CTL + 2 * gpio;

	intel_mid_pmic_writeb(ctl, value ? 1 : 0);
}

static void dollarcove_gpio_dbg_show(struct seq_file *s,
				struct gpio_chip *chip)
{
	int i;
	u8 ctl;

	for (i = 0; i < 2; i++) {
		ctl = intel_mid_pmic_readb(GPIO0_CTL + 2 * i);
		seq_printf(s,
			" gpio-%-2d %s\n",
			i,
			(ctl & 0x7) == 0x2 ? "in" :
				(ctl & 0x7) == 0x0 ? "out-lo " :
				(ctl & 0x7) == 0x1 ? "out-hi " : "unknown");
	}
}

static int dollarcove_gpio_probe(struct platform_device *pdev)
{
	int retval;
	struct device *dev = intel_mid_pmic_dev();
	static struct gpio_chip chip;

	chip.label = "intel_dollarcove";
	chip.direction_input = dollarcove_gpio_direction_input;
	chip.direction_output = dollarcove_gpio_direction_output;
	chip.get = dollarcove_gpio_get;
	chip.set = dollarcove_gpio_set;
	chip.base = -1;
	chip.ngpio = NUM_GPIO;
	chip.can_sleep = 1;
	chip.dev = dev;
	chip.dbg_show = dollarcove_gpio_dbg_show;
	retval = gpiochip_add(&chip);
	if (retval) {
		pr_warn("dollarcove: add gpio chip error: %d\n", retval);
		return retval;
	}

	return 0;
}

static struct platform_driver dollarcove_gpio_driver = {
	.probe = dollarcove_gpio_probe,
	.driver = {
		.name = "dollar_cove_gpio",
	},
};

module_platform_driver(dollarcove_gpio_driver);

MODULE_AUTHOR("Yang Bin<bin.yang@intel.com>");
MODULE_DESCRIPTION("Intel Dollar Cove GPIO Driver");
MODULE_LICENSE("GPL");
