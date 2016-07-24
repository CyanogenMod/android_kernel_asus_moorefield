/*
 * platform_smb1357.c: smb1357 platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <linux/power/smb1357-charger.h>
#include <asm/intel-mid.h>
#include "platform_smb1357.h"

static struct smb1357_charger_platform_data smb1357_pdata = {
	.use_mains		= true,
	.use_usb			= true,
	.inok_gpio			= -1,
};

void *smb1357_platform_data(void *info)
{
	smb1357_pdata.inok_gpio = get_gpio_by_name(SMB1357_INOK_GPIO);
	return &smb1357_pdata;
}
