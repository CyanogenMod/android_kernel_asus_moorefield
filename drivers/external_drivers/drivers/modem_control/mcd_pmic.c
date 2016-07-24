/*
 * linux/drivers/modem_control/mcd_pmic.c
 *
 * Version 1.0
 * This code allows to access the pmic specifics
 * of each supported platforms
 * among other things, it permits to communicate with the SCU/PMIC
 * to cold boot/reset the modem
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
#include <asm/intel_scu_pmic.h>
#include <linux/mdm_ctrl_board.h>

int pmic_io_init(void *data)
{
	return 0;
}

int pmic_io_power_on_ctp_mdm(void *data)
{
	/* on ctp, there's no power on, only a reset operation exist */
	/* just do nothing! */
	return 0;
}

int pmic_io_power_on_mdm(void *data)
{
	struct mdm_ctrl_pmic_data *pmic_data = data;
	int ret = 0;
	u16 addr = pmic_data->chipctrl;
	u8 def_value = 0x00;
	u8 iodata;

	if (pmic_data->chipctrl_mask) {
		/* Get the current register value in order to not
		 * override it
		 */
		ret = intel_scu_ipc_readv(&addr, &def_value, 1);
		if (ret) {
			pr_err(DRVNAME ": ipc_readv() failed %d", ret);
			return -1;
		}
	}
	/* Write the new register value (CHIPCNTRL_ON) */
	iodata = (def_value & pmic_data->chipctrl_mask) | pmic_data->chipctrlon;
	ret = intel_scu_ipc_writev(&addr, &iodata, 1);
	if (ret) {
		pr_err(DRVNAME ": scu_ipc_writev(ON) failed (ret: %d)", ret);
		return -1;
	}
	/* Wait before RESET_PWRDN_N to be 1 */
	usleep_range(pmic_data->pwr_down_duration,
		     pmic_data->pwr_down_duration);

	return 0;
}

int pmic_io_power_off_mdm(void *data)
{
	struct mdm_ctrl_pmic_data *pmic_data = data;
	int ret = 0;
	u16 addr = pmic_data->chipctrl;
	u8 iodata;
	u8 def_value = 0x00;

	if (pmic_data->chipctrl_mask) {
		/* Get the current register value in order to not override it */
		ret = intel_scu_ipc_readv(&addr, &def_value, 1);
		if (ret) {
			pr_err(DRVNAME ": ipc_readv() failed (ret: %d)", ret);
			return -1;
		}
	}

	/* Write the new register value (CHIPCNTRL_OFF) */
	iodata =
	    (def_value & pmic_data->chipctrl_mask) | pmic_data->chipctrloff;
	ret = intel_scu_ipc_writev(&addr, &iodata, 1);
	if (ret) {
		pr_err(DRVNAME ": ipc_writev(OFF)  failed (ret: %d)", ret);
		return -1;
	}
	/* Safety sleep. Avoid to directly call power on. */
	usleep_range(pmic_data->pwr_down_duration,
		     pmic_data->pwr_down_duration);

	return 0;
}

int pmic_io_cleanup(void *data)
{
	return 0;
}

int pmic_io_get_early_pwr_on(void *data)
{
	struct mdm_ctrl_pmic_data *pmic_data = data;
	return pmic_data->early_pwr_on;
}

int pmic_io_get_early_pwr_off(void *data)
{
	struct mdm_ctrl_pmic_data *pmic_data = data;
	return pmic_data->early_pwr_off;
}
