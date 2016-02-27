/*
 * mdm_apcdmp.h
 *
 * Intel Mobile driver for modem debugging.
 *
 * Copyright (C) 2015 ASUS Corporation. All rights reserved.
 *
 * Contact: Yuehtsang Li <Yuehtsang_Li@asus.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 */

#ifndef _MDM_APCDMP_H
#define _MDM_APCDMP_H

#include <linux/ioctl.h>


#define MDM_APCDMP_MAGIC 0x87
#define MDM_APCDMP_IMC726X _IO(MDM_APCDMP_MAGIC, 0)
#define MDM_APCDMP_IMC2230 _IO(MDM_APCDMP_MAGIC, 1)
#define MDM_APCDMP_DEBUG_IMC726X _IO(MDM_APCDMP_MAGIC, 2)
#define MDM_APCDMP_DEBUG_IMC2230 _IO(MDM_APCDMP_MAGIC, 3)

#endif /* _MDM_APCDMP_H */

