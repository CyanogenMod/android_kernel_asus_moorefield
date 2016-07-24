/*
 * intel_dcovex_regulator.h - Support for dollar cove XB pmic
 * Copyright (c) 2015, Intel Corporation.
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __INTEL_DCOVEX_REGULATOR_H_
#define __INTEL_DCOVEX_REGULATOR_H_

#include <linux/regulator/driver.h>

/* LDO and BUCK voltage control registers */

/* DLDOs */
#define DCOVEX_LDO1_VOL_CTRL		0x15
#define DCOVEX_LDO2_VOL_CTRL		0x16
#define DCOVEX_LDO3_VOL_CTRL		0x17
#define DCOVEX_LDO4_VOL_CTRL		0x18
/* ELDOs */
#define DCOVEX_LDO5_VOL_CTRL		0x19
#define DCOVEX_LDO6_VOL_CTRL		0x1a
#define DCOVEX_LDO7_VOL_CTRL		0x1b
/* FLDOs */
#define DCOVEX_LDO8_VOL_CTRL		0x1c
#define DCOVEX_LDO9_VOL_CTRL		0x1d
#define DCOVEX_LDO10_VOL_CTRL		0x1d

#define DCOVEX_BUCK6_VOL_CTRL		0x20
#define DCOVEX_BUCK5_VOL_CTRL		0x21
					/* 0x22 is RESERVED */
#define DCOVEX_BUCK1_VOL_CTRL		0x23
#define DCOVEX_BUCK4_VOL_CTRL		0x24
#define DCOVEX_BUCK3_VOL_CTRL		0x25
#define DCOVEX_BUCK2_VOL_CTRL		0x26
					/* 0x27 BUCK1/2/3/4/5 DVM */
/* ADOs */
#define DCOVEX_LDO11_VOL_CTRL		0x28
#define DCOVEX_LDO12_VOL_CTRL		0x29
#define DCOVEX_LDO13_VOL_CTRL		0x2a

/* GPIOs */
#define DCOVEX_GPIO1_VOL_CTRL		0x93

#define DCOVEX_GPIO0_EN_REG		0x90
#define DCOVEX_GPIO1_EN_REG		0x92

enum {
	DCOVEX_ID_BUCK1 = 1,
	DCOVEX_ID_BUCK2,
	DCOVEX_ID_BUCK3,
	DCOVEX_ID_BUCK4,
	DCOVEX_ID_BUCK5,
	DCOVEX_ID_BUCK6,

	DCOVEX_ID_LDO1,
	DCOVEX_ID_LDO2,
	DCOVEX_ID_LDO3,
	DCOVEX_ID_LDO4,
	DCOVEX_ID_LDO5,	/* ELDO1 */
	DCOVEX_ID_LDO6,
	DCOVEX_ID_LDO7,
	DCOVEX_ID_LDO8,	/* FLDO1 */
	DCOVEX_ID_LDO9,
	DCOVEX_ID_LDO10,
	DCOVEX_ID_LDO11,/* ALDO1 */
	DCOVEX_ID_LDO12,
	DCOVEX_ID_LDO13,

	DCOVEX_ID_GPIO1,

	DCOVEX_ID_MAX,
};

struct dcovex_regulator_info {
	struct regulator_desc	desc;
	struct regulator_dev	*regulator;
	struct regulator_init_data *init_data;
	int vol_reg;
	int vol_nbits;
	int vol_shift;
	int enable_reg;		/* enable register base  */
	int enable_bit;
};

#endif
