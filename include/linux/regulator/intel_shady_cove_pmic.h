/*
 * intel_shady_cove_pmic.h - Support for Shady Cove pmic VR
 * Copyright (c) 2014, Intel Corporation.
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *
 */
#ifndef __INTEL_SHADY_COVE_PMIC_H_
#define __INTEL_SHADY_COVE_PMIC_H_

struct regulator_init_data;

enum intel_regulator_id {
	VPROG1,
	VPROG2,
	VPROG3,
	VFLEX,
};

/* Voltage tables for Regulators */
static const unsigned int VPROG1_VSEL_table[] = {
	1500000, 1800000, 2500000, 2800000,
};

static const unsigned int VPROG2_VSEL_table[] = {
	1500000, 1800000, 2500000, 2850000,
};

static const unsigned int VPROG3_VSEL_table[] = {
	1050000, 1830000, 1850000, 2900000,
};
static const unsigned int VFLEX_VSEL_table[] = {
	4500000, 4750000, 5000000,
};

/* Slave Address for all regulators */
#define VPROG1CNT_ADDR	0x0ac
#define VPROG2CNT_ADDR	0x0ad
#define VPROG3CNT_ADDR	0x0ae
#define VFLEXCNT_ADDR	0x0ab

/* Slave Address for B0 PMIC regulators */
#define VPROG1CNT_B0_PMIC_ADDR 0x140
#define VPROG2CNT_B0_PMIC_ADDR 0x141
#define VPROG3CNT_B0_PMIC_ADDR 0x142

/**
 * intel_pmic_info - platform data for intel pmic
 * @pmic_reg: pmic register that is to be used for this VR
 */
struct intel_pmic_info {
	struct regulator_init_data *init_data;
	struct regulator_dev *intel_pmic_rdev;
	const u16 *table;
	u16 pmic_reg;
	u8 table_len;
};

#endif /* __INTEL_SHADY_COVE_PMIC_H_ */
