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
 *    Jari Luoma-aho  <jari.luoma-aho@intel.com>
 *    Jari Nippula    <jari.nippula@intel.com>
 *
 */

#if !defined GBURST_HW_H
#define GBURST_HW_H

#if (defined CONFIG_GPU_BURST) || (defined CONFIG_GPU_BURST_MODULE)

int gburst_hw_init(void);

int gburst_hw_gpu_freq_mhz_info(int freq_MHz);

int gburst_hw_set_perf_status_periodic(int on_or_off);

int gburst_hw_inq_num_counters(int *ctr_first, int *ctr_last);

int gburst_hw_inq_num_cores(void);

int gburst_hw_set_counter_id(unsigned int ctr_ix, int ctr_grp,
                        	int ctr_bit, int cntrbits, int summux);

int gburst_hw_inq_counter_id(unsigned int ctr_ix, int *ctr_grp,
	                        int *ctr_bit, int *cntrbits, int *summux);

int gburst_hw_inq_counter_coeff(unsigned int ctr_ix);

int gburst_hw_set_counter_coeff(unsigned int ctr_ix, int coeff);

int gburst_hw_perf_data_get_indices(int *ix_roff, int *ix_woff);

int gburst_hw_perf_data_get_data(uint32_t *time_stamp, int *counters_storable,
	uint32_t **p_pdat);

int gburst_hw_perf_data_read_index_incr(uint32_t *ix_roff);

int gburst_hw_mutex_lock(void);

int gburst_hw_mutex_unlock(void);

int gburst_hw_is_access_denied(void);

void gburst_hw_flush_buffer(void);

#endif /* if (defined CONFIG_GPU_BURST) || (defined CONFIG_GPU_BURST_MODULE) */

#endif /* if !defined GBURST_HW_H */
