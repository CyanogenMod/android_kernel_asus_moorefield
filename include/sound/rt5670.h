/*
 * linux/sound/rt5670.h -- Platform data for RT5670
 *
 * Copyright 2011 Realtek Microelectronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_SND_RT5670_H
#define __LINUX_SND_RT5670_H

struct rt5670_platform_data {
	bool asrc_en;
	int jd_mode;
	/*
	0: disable,
	1: 3.3v, 2 port,
	2: 3.3v, 1 port,
	3: 1.8v, 1 port,
	*/
	bool in2_diff;
	bool in3_diff;
	bool in4_diff;
	bool bclk_32fs[4];
};

#endif
