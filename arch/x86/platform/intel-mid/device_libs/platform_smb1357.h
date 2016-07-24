/*
 * platform_smb1357.h: smb1357 platform data header file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _PLATFORM_SMB1357_H_
#define _PLATFORM_SMB1357_H_

#define SMB1357_STAT_GPIO	 "chg_stat"
#define SMB1357_INOK_GPIO	"CHG_INOK#"
extern void *smb1357_platform_data(void *info) __attribute__((weak));
#endif
