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
 *
 */

#ifndef CSS2600_PSYS_LIB_H
#define CSS2600_PSYS_LIB_H

#if IS_ENABLED(CONFIG_VIDEO_CSS2600_2401)
#include "lib2401/bxt_sandbox/psyspoc/interface/ia_css_psys_event.h"
#include "lib2401/bxt_sandbox/ia_camera/shared/program_group/interface/ia_css_pg_param_internal.h"
#include "lib2401/bxt_sandbox/ia_camera/pgvfpp/interface/ia_css_pgvfpp.h"
#else
#include <ia_css_psys_process_group.h>
#include <ia_css_psys_terminal.h>
#include <ia_css_psys_device.h>
#endif

#endif /* CSS2600_PSYS_LIB_H */
