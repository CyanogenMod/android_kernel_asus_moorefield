/*
 * intel_crystal_cove_plus_pmic.h - Support for CrystalCove plus pmic VR
 * Copyright (c) 2014, Intel Corporation.
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#ifndef __INTEL_CRYSTAL_COVE_PLUS_PMIC_H_
#define __INTEL_CRYSTAL_COVE_PLUS_PMIC_H_

#include <linux/regulator/driver.h>

enum CCOVEP_REGULATOR_ID {
	CCOVEP_ID_V3P3A = 1,
	CCOVEP_ID_V3P3SX,
	CCOVEP_ID_V1P05A,
	CCOVEP_ID_V1P15,
	CCOVEP_ID_V1P8A,
	CCOVEP_ID_V5P0A,
	CCOVEP_ID_V2P8SX,
	CCOVEP_ID_VDDQ,
};

struct ccovep_regulator_info {
	struct regulator_desc	desc;
	struct regulator_dev	*regulator;
	struct regulator_init_data *init_data;
	int vol_reg;
};

#endif /* __INTEL_CRYSTAL_COVE_PLUS_PMIC_H_ */
