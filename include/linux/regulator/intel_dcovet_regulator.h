/*
 * intel_dcovet_regulator.h - Support for dollar cove TB pmic
 * Copyright (c) 2013, Intel Corporation.
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
#ifndef __INTEL_DCOVET_REGULATOR_H_
#define __INTEL_DCOVET_REGULATOR_H_

#include <linux/regulator/driver.h>

/* Buck 1-6 and LDO 1-5 are dedicated LDOs for Platform &
*  should not be played around with by  device Drivers    */

enum {
	DCOVET_ID_LDO6 = 1,
	DCOVET_ID_LDO7,
	DCOVET_ID_LDO8,
	DCOVET_ID_LDO9,
	DCOVET_ID_LDO10,
	DCOVET_ID_LDO11,
	DCOVET_ID_LDO12,
	DCOVET_ID_LDO13,
	DCOVET_ID_LDO14,

	DCOVET_ID_MAX,
};

struct dcovet_regulator_info {
	struct regulator_desc	desc;
	struct regulator_dev	*regulator;
	struct regulator_init_data *init_data;
	int ctrl_reg;
	int volt_nbits;
	int volt_shift;
};

#endif
