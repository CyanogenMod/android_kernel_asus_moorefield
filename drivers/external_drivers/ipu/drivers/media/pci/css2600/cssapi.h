/*
 * Copyright (c) 2014 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef CSSAPI_H
#define CSSAPI_H

#if IS_ENABLED(CONFIG_VIDEO_CSS2600_2401)
#include <libcss2401.h>
#include "lib2401/ia_css_env.h"
#else
/* Hacks to cover lack of multibinary support in CSSAPI */
#include <lib2401/css_2401_system/pixelgen_global.h>
static inline int ia_css_isysapi_rx_set_csi_port_cfg(
	void *context, int dt, /* from this we can get format type*/
	int channel_id,	int src, int num_lanes) { return 0; }
static inline int ia_css_isysapi_rx_set_tpg_cfg(
	void *context, int src, struct pixelgen_tpg_cfg_s *cfg)
{ return 0; }
#endif /* IS_ENABLED(CONFIG_VIDEO_CSS2600_2401) */

#if IS_ENABLED(CONFIG_VIDEO_CSS2600_2600)
#include "libcss2600.h"
#endif

#endif
