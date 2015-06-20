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
 */

#if !defined GBURST_INTERFACE_H
#define GBURST_INTERFACE_H

#if (defined CONFIG_GPU_BURST) || (defined CONFIG_GPU_BURST_MODULE)

struct gburst_pvt_s;

typedef void (*gburst_power_state_set_f)(struct gburst_pvt_s *gbprv, int st_on);

/**
 * struct gburst_interface_s -
 * @gbs_priv: Private data handle, opaque to the other driver.
 * @gbs_power_state_set: Function to callback when dev power changes.
 */
struct gburst_interface_s {
	struct gburst_pvt_s *gbs_priv;
	gburst_power_state_set_f gbs_power_state_set;
};


/**
 * gburst_interface_set_data() - Provide some gburst data for hooks
 * inside the graphics driver.
 * Also, the symbol dependency will establish a load order dependency for
 * the case where both the graphics driver and the gburst driver are modules,
 * ensuring that the graphics driver is loaded and initialized before gburst.
 */
void gburst_interface_set_data(struct gburst_interface_s *gb_interface);


/**
 * gburst_interface_power_state_set() - Indicate that power is off (0) or on (1).
 * This is a hook called from the low-level device driver.
 */
void gburst_interface_power_state_set(int st_on);

#endif /* if (defined CONFIG_GPU_BURST) || (defined CONFIG_GPU_BURST_MODULE) */

#endif /* if !defined GBURST_INTERFACE_H */
