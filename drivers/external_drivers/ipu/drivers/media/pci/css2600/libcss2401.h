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

#ifndef LIBCSS2401_H
#define LIBCSS2401_H

#include <ia_css.h>
#include <ia_css_buffer_pool.h>
#include <ia_css_env.h>
#include <ia_css_fwctrl_public.h>
#include <ia_css_isys_ext_public.h>
#include <ia_css_isysapi.h>
#include <ia_css_pg_param_internal.h>
#include <ia_css_psys_frameadapter.h>
#include <ia_css_psys_event.h>
#include <ia_css_psys_process_group.h>
#include <ia_css_psys_terminal.h>
#include <ia_css_psys_device.h>

#include <sh_css_firmware.h>

struct ia_css_fw_info *libcss2401_get_sp_fw(void);

#endif
