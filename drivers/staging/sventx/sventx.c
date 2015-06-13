/*
  This file is part of the SVENTX trace library.
  Copyright(c) 2013-2014 Intel Corporation. All rights reserved.

  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
  The full GNU General Public License is included in this distribution
  in the file called license.txt.

  Contact Information:
    Intel Corporation
    2200 Mission College Blvd.
    Santa Clara, CA  97052
*/

/* SVEN TRACE instrumentation API for kernel code */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/cpu.h>

#include "sventx.h"

/*
 * Module attributes
 */
MODULE_DESCRIPTION("SVENTX - Software Trace API for North Peak");
MODULE_SUPPORTED_DEVICE("Intel Processors");
MODULE_AUTHOR("Norbert Schulz <norbert.schulz@intel.com>");
MODULE_LICENSE("Dual BSD/GPL");

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 8, 0)
static bool trace = true;
#else
static int trace = 1;
#endif
module_param(trace, bool, S_IRUGO);
MODULE_PARM_DESC(trace,
		 "     Send SVEN trace events for driver actions, default=1(on)");

static int debug;
module_param(debug, int, S_IRUGO);
MODULE_PARM_DESC(debug,
		 "     Set diagnostic messages print level, default=0(off)");

/* unique module id of this module */
static sven_guid_t sventx_guid = {
	0x494E5443, 0xD423, 0x49CF,
	{0x91, 0x97, 0x47, 0x24, 0xec, 0xf7, 0xe5, 0x5f}
};

static psven_handle_t svh_driver;

DEFINE_PER_CPU(volatile struct sven_sth_channel *, sventx_sth_mmio) = 0;


#if defined(FAKE_NPK)
static void *npk_alloc_sth_sven_ptr(int cpu)
{
	static struct sven_sth_channel dummy[NR_CPUS];

	return &dummy[cpu];
}

static void npk_free_sth_sven_ptr(int cpu, void __iomem *channel)
{
}
#else
extern void *npk_alloc_sth_sven_ptr(int cpu);
extern void npk_free_sth_sven_ptr(int cpu, void __iomem *channel);
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 25)
#define get_online_cpus lock_cpu_hotplug
#define put_online_cpus unlock_cpu_hotplug
#endif

static struct sven_sth_channel dummy_sth_mem;

static int __init sventx_init_module(void)
{
	int i;

	/* Allocate STH output channels for the kernel from the NPK driver
	 * Each cpu uses a dedicated IO channel using the PER_CPU concept.
	 * This enables lock-free IO to the STH.
	 */
	if (debug)
		pr_info("SVENTX: Requesting per CPU STH pointers ...\n");

	get_online_cpus();
	for_each_online_cpu(i) {
		volatile struct sven_sth_channel **p;
		p = &per_cpu(sventx_sth_mmio, i);
		*p = (struct sven_sth_channel *) npk_alloc_sth_sven_ptr(i);

		if (*p == NULL) {
			/* Failed to allocate STH channel. This means that all
			 * trace output from this CPU will get lost. We
			 * redirect the STH pointer into local memory to give
			 * it a valid address. This saves the sventx library
			 * from testing the pointer, but no trace will be
			 *  produced.
			 */
			*p = &dummy_sth_mem;
			pr_info("SVENTX: no STH pointer for CPU %2d\n. Skipping trace generation.",
			       i);
		} else if (debug) {
			pr_info(
			       "SVENTX: STH pointer for CPU %2d: %p\n", i,
			       per_cpu(sventx_sth_mmio, i));
		}
	}
	put_online_cpus();

	/* Bring the SVEN environment up...
	 */
	SVEN_INIT(NULL, NULL);

	if (trace) {

		/* Allocate a SVEN handle to issue events from this driver.
		 */
		svh_driver = SVEN_ALLOC_HANDLE(0);
		SVEN_SET_HANDLE_GUID_UNIT(svh_driver, sventx_guid, 0);

		if (debug)
			pr_info("SVENTX: sending driver load event...\n");

		SVEN_DEBUG(svh_driver, SVEN_SEVERITY_NORMAL,
			   "sventx loaded", sizeof("sventx loaded"));
	}

	return 0;
}

static void __exit sventx_cleanup_module(void)
{
	int i;


	if (trace) {
		if (debug)
			pr_info("SVENTX: sending driver exit SVEN event...\n");

		SVEN_DEBUG(svh_driver, SVEN_SEVERITY_NORMAL,
			   "sventx unloaded", sizeof("sventx unloaded"));
		SVEN_DELETE_HANDLE(svh_driver);
		SVEN_SHUTDOWN(NULL);
	}

	/* release STH channels
	 */
	get_online_cpus();
	for_each_online_cpu(i) {
		if (per_cpu(sventx_sth_mmio, i) != &dummy_sth_mem) {
			npk_free_sth_sven_ptr(
				i,
				(void *)per_cpu(sventx_sth_mmio, i));

			if (debug) {
				pr_info(
				       "SVENTX: free STH pointer for CPU %2d\n",
				       i);
			}
		}
	}
	put_online_cpus();
}

module_init(sventx_init_module);
module_exit(sventx_cleanup_module);

/*
 * Pull in the entire SVENTX library code to be build as a single
 * kernel module object.
 */
#include "sventx_init.c"
#include "sventx_compiler.c"
#include "sventx_inline.c"
#include "sventx_write.c"
#include "sventx_crc32.c"
#include "sventx_api.c"

EXPORT_SYMBOL(sventx_init_handle);
EXPORT_SYMBOL(sventx_delete_handle);
EXPORT_PER_CPU_SYMBOL(sventx_sth_mmio);

/*
 * configuration dependent module kernel exports
 */

#if defined(SVEN_PCFG_ENABLE_LOCATION_ADDRESS)
EXPORT_SYMBOL(sventx_return_addr);
#endif

#if defined(SVEN_PCFG_ENABLE_STRING_API)
EXPORT_SYMBOL(sventx_write_debug_string);
#endif

#if defined(SVEN_PCFG_ENABLE_CATID32_API)
EXPORT_SYMBOL(sventx_write_catalog32_message);
#endif
#if defined(SVEN_PCFG_ENABLE_CATID64_API)
EXPORT_SYMBOL(sventx_write_catalog64_message);
#endif

#if defined(SVEN_PCFG_ENABLE_WRITE_API)
EXPORT_SYMBOL(sventx_write_raw_message);
#endif

#if defined(SVEN_PCFG_ENABLE_API_API)
EXPORT_SYMBOL(sventx_write_api_call_message);
EXPORT_SYMBOL(sventx_write_api_return_message);
#endif

#if defined(SVEN_PCFG_ENABLE_REGISTER_API)
EXPORT_SYMBOL(sventx_write_32bit_regio_message);
EXPORT_SYMBOL(sventx_write_64bit_regio_message);
#endif
