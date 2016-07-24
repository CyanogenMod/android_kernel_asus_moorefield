/**************************************************************************
 * Copyright (c) 2012, Intel Corporation.
 * All Rights Reserved.

 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Authors:
 *    Dale B. Stimson <dale.b.stimson@intel.com>
 *
 */

#if !defined GBURST_HW_IF_H
#define GBURST_HW_IF_H

#if (defined CONFIG_GPU_BURST) || (defined CONFIG_GPU_BURST_MODULE)

#include "sgx_mkif_km.h"
#include "device.h"

struct gburst_hw_if_info_s {
	PVRSRV_DEVICE_NODE *gsh_gburst_psDeviceNode;
	SGXMKIF_HWPERF_CB  *gsh_gburst_psHWPerfCB;
	int                 gsh_initialized;
};

int gburst_hw_if_get_info(struct gburst_hw_if_info_s *psh);

#if (defined(GBURST_HW_PVRSCOPESERVICE_SUPPORT))
void gburst_hw_reconfigure_groups(void);
#endif

#endif /* if (defined CONFIG_GPU_BURST) || (defined CONFIG_GPU_BURST_MODULE) */

#endif /* if !defined GBURST_HW_IF_H */
