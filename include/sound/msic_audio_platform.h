/*
 * sound/msic_audio_platform.h -- Platform data for MSIC_AUDIO
 *
 * (C) Copyright 2012 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#ifndef __SND_MSIC_AUDIO_PLATFORM_H
#define __SND_MSIC_AUDIO_PLATFORM_H


#include <linux/sfi.h>
struct msic_audio_platform_data {
	/* Intel software platform id*/
	const struct soft_platform_id *spid;
	int jack_gpio;
};

#endif
