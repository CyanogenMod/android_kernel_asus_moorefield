/*
 * platform_apds990x.c: apds990x platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <linux/i2c/apds990x.h>
#include <asm/intel-mid.h>
#include "platform_apds990x.h"

void *apds990x_platform_data(void *info)
{
	static struct apds990x_platform_data platform_data = {
		.cf = {
			.cf1    = 37404,
			.irf1   = 82558,
			.cf2    = 19756,
			.irf2   = 35753,
			.df     = 52,
			.ga     = 5348,
			.incan  = 1545,
			.min_ir = 0,
		},
		.pdrive         = 0,
		.ppcount        = 1,
	};

	if (INTEL_MID_BOARD(2, PHONE, MFLD, LEX, ENG)) {
		platform_data.cf.cf1    = 8602;
		platform_data.cf.irf1   = 6552;
		platform_data.cf.cf2    = 1064;
		platform_data.cf.irf2   = 860;

		if (SPID_HARDWARE_ID(MFLD, PHONE, LEX, PR21) ||
		    SPID_HARDWARE_ID(MFLD, PHONE, LEX, PR1M) ||
		    SPID_HARDWARE_ID(MFLD, PHONE, LEX, PR11))
			platform_data.cf.ga = 1474;
		else
			platform_data.cf.ga = 11796;
	} else if ((INTEL_MID_BOARD(2, PHONE, CLVTP, VB, PRO)) ||
		   (INTEL_MID_BOARD(2, PHONE, CLVTP, VB, ENG))) {
		platform_data.cf.cf1    = 6539;
		platform_data.cf.irf1   = 25202;
		platform_data.cf.cf2    = 3390;
		platform_data.cf.irf2   = 6112;
		platform_data.cf.ga     = 11239;
		platform_data.cf.incan  = 676;
		platform_data.cf.min_ir = 4;
		if (SPID_HARDWARE_ID(CLVTP, PHONE, VB, PR20)) {
			platform_data.cf.cf1    = 42573;
			platform_data.cf.irf1   = 111851;
			platform_data.cf.cf2    = 9393;
			platform_data.cf.irf2   = 16956;
			platform_data.cf.ga     = 6204;
			platform_data.cf.incan  = 1432;
			platform_data.cf.min_ir = 1;
		}
	}

	platform_data.gpio_number = get_gpio_by_name("AL-intr");

	return &platform_data;
}
