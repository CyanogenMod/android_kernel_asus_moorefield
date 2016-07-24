/*
 * This driver provides interface for buggy IPs
 * Copyright (c) 2014, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#include <linux/init.h>
#include <linux/pci.h>
#include <linux/pm_qos.h>
#include <linux/intel_mid_pm.h>

#define PCI_DEVICE_ID_IUNIT_CHT		0x22b8
#define PCI_DEVICE_ID_IUNIT_BYT		0x0f38
#define ISP_PWR_ISLAND			0x1

#ifdef CONFIG_INTEL_SOC_PMC
extern int pmc_nc_set_power_state(int islands, int state_type, int reg);
#endif
