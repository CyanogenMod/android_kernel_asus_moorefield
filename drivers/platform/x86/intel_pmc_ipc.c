/*
 * intel_pmc_ipc.c: Driver for the Intel PMC IPC mechanism
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

#include <linux/bug.h>
#include <asm/intel_scu_pmic.h>

int intel_scu_ipc_ioread8(u16 addr, u8 *data)
{
	WARN(1, "Invalid, please avoid runtime %s call on BXT.\n", __func__);

	return 0;
}
EXPORT_SYMBOL(intel_scu_ipc_ioread8);

int intel_scu_ipc_ioread32(u16 addr, u32 *data)
{
	WARN(1, "Invalid, please avoid runtime %s call on BXT.\n", __func__);

	return 0;
}
EXPORT_SYMBOL(intel_scu_ipc_ioread32);

int intel_scu_ipc_readv(u16 *addr, u8 *data, int len)
{
	WARN(1, "Invalid, please avoid runtime %s call on BXT.\n", __func__);

	return 0;
}
EXPORT_SYMBOL(intel_scu_ipc_readv);

int intel_scu_ipc_iowrite8(u16 addr, u8 data)
{
	WARN(1, "Invalid, please avoid runtime %s call on BXT.\n", __func__);

	return 0;
}
EXPORT_SYMBOL(intel_scu_ipc_iowrite8);

int intel_scu_ipc_writev(u16 *addr, u8 *data, int len)
{
	WARN(1, "Invalid, please avoid runtime %s call on BXT.\n", __func__);

	return 0;
}
EXPORT_SYMBOL(intel_scu_ipc_writev);

int intel_scu_ipc_update_register(u16 addr, u8 data, u8 mask)
{
	WARN(1, "Invalid, please avoid runtime %s call on BXT.\n", __func__);

	return 0;
}
EXPORT_SYMBOL(intel_scu_ipc_update_register);

