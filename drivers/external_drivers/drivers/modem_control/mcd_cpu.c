/*
 * linux/drivers/modem_control/mcd_cpu.c
 *
 * Version 1.0
 * This code permits to access the cpu specifics
 * of each supported platform.
 * among other things, it permits to configure and access gpios
 *
 * Copyright (C) 2013 Intel Corporation. All rights reserved.
 *
 * Contact: Ranquet Guillaume <guillaumex.ranquet@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/mdm_ctrl_board.h>

#include "mdm_util.h"

/**
 * mdm_ctrl_configure_gpio - Configure GPIOs
 * @gpio: GPIO to configure
 * @direction: GPIO direction - 0: IN | 1: OUT
 *
 */
static inline int mdm_ctrl_configure_gpio(int gpio,
					  int direction,
					  int value, const char *desc)
{
	int ret;

	ret = gpio_request(gpio, desc);

	if (direction)
		ret += gpio_direction_output(gpio, value);
	else
		ret += gpio_direction_input(gpio);

	if (ret) {
		pr_err(DRVNAME ": Unable to configure GPIO%d (%s)", gpio, desc);
		ret = -ENODEV;
	}

	return ret;
}

int cpu_init_gpio(void *data)
{
	struct mdm_ctrl_cpu_data *cpu_data = data;
	int ret;

	pr_debug("cpu_init");

	/* Configure the RESET_BB gpio */
	ret = mdm_ctrl_configure_gpio(cpu_data->gpio_rst_bbn, 1, 0, "ModemControl_RST_BB");
	if (ret)
		goto out;

	/* Configure the ON gpio */
	ret = mdm_ctrl_configure_gpio(cpu_data->gpio_pwr_on, 1, 0, "ModemControl_ON");
	if (ret)
		goto free_ctx5;

	/* Configure the RESET_OUT gpio & irq */
	ret = mdm_ctrl_configure_gpio(cpu_data->gpio_rst_out, 0, 0, "ModemControl_RST_OUT");
	if (ret)
		goto free_ctx4;

	cpu_data->irq_reset = gpio_to_irq(cpu_data->gpio_rst_out);
	if (cpu_data->irq_reset < 0) {
		goto free_ctx3;
	}

	/* Configure the CORE_DUMP gpio & irq */
	ret = mdm_ctrl_configure_gpio(cpu_data->gpio_cdump, 0, 0, "ModemControl_CORE_DUMP");
	if (ret)
		goto free_ctx2;

	cpu_data->irq_cdump = gpio_to_irq(cpu_data->gpio_cdump);
	if (cpu_data->irq_cdump < 0) {
		goto free_ctx1;
	}

	pr_info(DRVNAME
		": GPIO (rst_bbn: %d, pwr_on: %d, rst_out: %d, fcdp_rb: %d)\n",
		cpu_data->gpio_rst_bbn, cpu_data->gpio_pwr_on,
		cpu_data->gpio_rst_out, cpu_data->gpio_cdump);

	return 0;

 free_ctx1:
	gpio_free(cpu_data->gpio_cdump);
 free_ctx2:
	if (cpu_data->irq_reset > 0)
		free_irq(cpu_data->irq_reset, cpu_data);
	cpu_data->irq_reset = 0;
 free_ctx3:
	gpio_free(cpu_data->gpio_rst_out);
 free_ctx4:
	gpio_free(cpu_data->gpio_pwr_on);
 free_ctx5:
	gpio_free(cpu_data->gpio_rst_bbn);
 out:
	return -ENODEV;
}

int cpu_cleanup_gpio(void *data)
{
	struct mdm_ctrl_cpu_data *cpu_data = data;

	gpio_free(cpu_data->gpio_cdump);
	gpio_free(cpu_data->gpio_rst_out);
	gpio_free(cpu_data->gpio_pwr_on);
	gpio_free(cpu_data->gpio_rst_bbn);

	cpu_data->irq_cdump = 0;
	cpu_data->irq_reset = 0;

	return 0;
}

int get_gpio_irq_cdump(void *data)
{
	struct mdm_ctrl_cpu_data *cpu_data = data;
	return cpu_data->irq_cdump;
}

int get_gpio_irq_rst(void *data)
{
	struct mdm_ctrl_cpu_data *cpu_data = data;
	return cpu_data->irq_reset;
}

int get_gpio_mdm_state(void *data)
{
	struct mdm_ctrl_cpu_data *cpu_data = data;
	return gpio_get_value(cpu_data->gpio_rst_out);
}

int get_gpio_rst(void *data)
{
	struct mdm_ctrl_cpu_data *cpu_data = data;
	return cpu_data->gpio_rst_bbn;
}

int get_gpio_pwr(void *data)
{
	struct mdm_ctrl_cpu_data *cpu_data = data;
	return cpu_data->gpio_pwr_on;
}

int get_gpio_on(void *data)
{
	struct mdm_ctrl_cpu_data *cpu_data = data;
	return cpu_data->gpio_on_key;
}

int cpu_init_gpio_ngff(void *data)
{
	struct mdm_ctrl_cpu_data *cpu_data = data;
	int ret;

	pr_debug("cpu_init");

	/* Configure the RESET_BB gpio */
	ret = mdm_ctrl_configure_gpio(cpu_data->gpio_rst_bbn, 1, 0, "RST_BB");
	if (ret)
		goto out;

	ret = mdm_ctrl_configure_gpio(GPIO_RST_USBHUB, 1, 1, "USB_HUB_reset");

	pr_info(DRVNAME ": GPIO (rst_bbn: %d, rst_usb_hub: %d)\n",
		cpu_data->gpio_rst_bbn,GPIO_RST_USBHUB);

	if (ret)
		goto free_ctx1;

	return 0;
 free_ctx1:
	gpio_free(cpu_data->gpio_rst_bbn);
 out:
	return -ENODEV;
}

int cpu_cleanup_gpio_ngff(void *data)
{
	struct mdm_ctrl_cpu_data *cpu_data = data;

	gpio_free(cpu_data->gpio_rst_bbn);
	gpio_free(GPIO_RST_USBHUB);

	return 0;
}

int get_gpio_mdm_state_ngff(void *data)
{
	return 0;
}

int get_gpio_irq_cdump_ngff(void *data)
{
	return 0;
}

int get_gpio_irq_rst_ngff(void *data)
{
	return 0;
}

