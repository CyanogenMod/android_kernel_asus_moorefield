/*
 * intel_soc_pmc.h
 *
 * Copyright (C) 2014 Intel Corp
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 */

#ifndef __INTEL_SOC_PMC_H
#define __INTEL_SOC_PMC_H

int pmc_pc_set_freq(int clk_num, int freq_type);
int pmc_pc_configure(int clk_num, u32 conf);

#endif /* __INTEL_SOC_PMC_H */
