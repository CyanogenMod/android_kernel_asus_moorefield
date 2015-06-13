/*
 * platform_cht_audio.h: Cherrytrail audio platform data header file
 *
 * (C) Copyright 2013 Intel Corporation
 * Author: Mythri P K<mythri.p.k@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _PLATFORM_CHT_AUDIO_H_
#define _PLATFORM_CHT_AUDIO_H_

enum {
	CHT_AUD_AIF1 = 0,
	CHT_AUD_POWER,
	CHT_AUD_PROBE_DEV,
};

enum {
	CHT_DPCM_AUD_AIF1 = 0,
	CHT_DPCM_DB,
	CHT_DPCM_LL,
	CHT_DPCM_COMPR,
	CHT_DPCM_VOIP,
	CHT_DPCM_PROBE,
};

#endif
