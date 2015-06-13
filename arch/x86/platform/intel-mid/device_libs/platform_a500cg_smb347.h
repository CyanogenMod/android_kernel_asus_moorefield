/*
 * platform_smb347.h: smb347 platform data header file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _PLATFORM_SMB347_H_
#define _PLATFORM_SMB347_H_

#if 0
#define SMB347_IRQ_GPIO		52
#endif
#define SMB347_STAT_GPIO "chg_stat"
#define SMB347_INOK_GPIO	"CHG_INOK#"
extern void *smb347_platform_data(void *info) __attribute__((weak));
#endif
