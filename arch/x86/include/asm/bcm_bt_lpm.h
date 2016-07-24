/*
 * Copyright (C) 2009 Google, Inc.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/

#ifndef BCM_BT_LMP_H
#define BCM_BT_LMP_H

struct bcm_bt_lpm_platform_data {
	int gpio_wake;		/* CPU -> BCM wakeup gpio */
	int gpio_host_wake;	/* BCM -> CPU wakeup gpio */
	int int_host_wake;	/* BCM -> CPU wakeup irq */
#ifdef CONFIG_PF450CL
	int gpio_reset;         /* GPIO reset pin of BT/FM */
	int gpio_reg_on;	/* GPIO regular on pin of BT/FM */
#else
	int gpio_enable;	/* GPIO enable/disable BT/FM */
#endif
	int port;		/* UART port to use with BT/FM */
};

#endif
