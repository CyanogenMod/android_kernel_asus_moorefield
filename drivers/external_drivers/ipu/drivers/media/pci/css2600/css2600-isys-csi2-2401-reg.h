/*
 * Copyright (c) 2013--2014 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef CSS2600_ISYS_CSI2_2401_REG_H
#define CSS2600_ISYS_CSI2_2401_REG_H

#define CSI2_REG_2401_CSI_RX_DLY_CNT_TERMEN_CLANE	0x18
#define CSI2_REG_2401_CSI_RX_DLY_CNT_SETTLE_CLANE	0x1c
/* 0..3 */
#define CSI2_REG_2401_CSI_RX_DLY_CNT_TERMEN_DLANE(n)	(0x20 + (n) * 8)
#define CSI2_REG_2401_CSI_RX_DLY_CNT_SETTLE_DLANE(n)	(0x24 + (n) * 8)

#endif /* CSS2600_ISYS_CSI2_2401_REG_H */
