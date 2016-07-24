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

#ifndef CSS2600_ISYS_CSI2_REG_H
#define CSS2600_ISYS_CSI2_REG_H

/* CSI-2 register ranges */
/* CSI-2 port registers 0..3 (A..D) */
#define CSI2_REG_CSI2_PORT_OFFSET(n)			(0x0400 * (n))

/* CSI-2 port specific sub-ranges */
#define CSI2_REG_CSI2_PORT_CSI_RX_OFFSET		0x0000
#define CSI2_REG_CSI2_PORT_MIPI_BE_OFFSET		0x0100
#define CSI2_REG_CSI2_PORT_PG_OFFSET			0x0200
#define CSI2_REG_CSI2_PORT_IRQ_CONTROLLER_OFFSET	0x0300

#define __CSI2_PORT_REG(port, subrange, reg)		\
	(CSI2_REG_CSI2_PORT_OFFSET(port)		\
	 + CSI2_REG_CSI2_PORT_##subrange##_OFFSET	\
	 + CSI2_REG_##subrange##_##reg)

#define CSI2_REG_CSI_RX_ENABLE				0x00
#define CSI2_CSI_RX_ENABLE_ENABLE			0x01
/* Enabled lanes - 1 */
#define CSI2_REG_CSI_RX_NOF_ENABLED_LANES		0x04
#define CSI2_REG_CSI_RX_HBP_TESTMODE_ENABLE		0x08
#define CSI2_REG_CSI_RX_ERROR_HANDLING			0x0c
#define CSI2_REG_CSI_RX_SP_IF_CONFIG			0x10
#define CSI2_REG_CSI_RX_LP_IF_CONFIG			0x14
#define CSI2_REG_CSI_RX_STATUS				0x18
#define CSI2_CSI_RX_STATUS_BUSY				0x01
#define CSI2_REG_CSI_RX_STATUS_DLANE_HS			0x1c
#define CSI2_REG_CSI_RX_STATUS_DLANE_LP			0x20
#define CSI2_REG_CSI_RX_DLY_CNT_TERMEN_CLANE		0x24
#define CSI2_REG_CSI_RX_DLY_CNT_SETTLE_CLANE		0x28
/* 0..3 */
#define CSI2_REG_CSI_RX_DLY_CNT_TERMEN_DLANE(n)		(0x2c + (n) * 8)
#define CSI2_REG_CSI_RX_DLY_CNT_SETTLE_DLANE(n)		(0x30 + (n) * 8)

#define CSI2_CSI_RX_DLY_CNT_TERMEN_CLANE_A		0
#define CSI2_CSI_RX_DLY_CNT_TERMEN_CLANE_B		0
#define CSI2_CSI_RX_DLY_CNT_SETTLE_CLANE_A		95
#define CSI2_CSI_RX_DLY_CNT_SETTLE_CLANE_B		-8

#define CSI2_CSI_RX_DLY_CNT_TERMEN_DLANE_A		0
#define CSI2_CSI_RX_DLY_CNT_TERMEN_DLANE_B		0
#define CSI2_CSI_RX_DLY_CNT_SETTLE_DLANE_A		85
#define CSI2_CSI_RX_DLY_CNT_SETTLE_DLANE_B		-2

#endif /* CSS2600_ISYS_CSI2_REG_H */
