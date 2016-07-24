/*

  This file is provided under a dual BSD/GPLv2 license.  When using or
  redistributing this file, you may do so under either license.

  GPL LICENSE SUMMARY

  Copyright(c) 2012-2014 Intel Corporation. All rights reserved.

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

  BSD LICENSE

  Copyright(c) 2012-2014 Intel Corporation. All rights reserved.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Intel Corporation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* Simple "hello world style kernel module showing SVEN instrumentation
 */
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/uaccess.h>

MODULE_LICENSE("GPL");

#include "sventx.h"

static psven_handle_t svenHandle;
static struct timer_list tml;


static const sven_guid_t guid = {
	0x494E5443, 0x0D6C, 0x471A,
	{0x95, 0x9f, 0x9d, 0xfa, 0xe9, 0x4f, 0x22, 0xb3}
};

/*
 * Slow timer tick callback
 *
 * re-arm itself to run every second
 */
static void sventest_timer_tick(unsigned long ptr)
{
	static unsigned count;
	struct timer_list *p = (struct timer_list *) ptr;

	pr_info("SVENTX: sventest_timer_tick:%d\n", count);
	SVEN_SHORT32(svenHandle, count);


	if (count % 2) {
		SVEN_DEBUG(svenHandle, SVEN_SEVERITY_NORMAL, "tick",
			   /*length */ 5);
	} else {
		SVEN_DEBUG(svenHandle, SVEN_SEVERITY_NORMAL, "tack",
			   /*length */ 5);
	}

	++count;

	/* arm timer again */
	p->expires = jiffies + HZ;	/* one second */
	add_timer(p);
}

void test_sven_api(void)
{
	SVEN_DEBUG(svenHandle, SVEN_SEVERITY_NORMAL, "Hello world!",
		   /*length */ 12);
	SVEN_DEBUG_LOC16(svenHandle, SVEN_SEVERITY_NORMAL, 0x1604,
			 "Hello world!", /*length */ 12);
	SVEN_DEBUG_LOC32(svenHandle, SVEN_SEVERITY_NORMAL, 0x1604,
			 "Hello world!", /*length */ 12);
	SVEN_DEBUG_LOCADDR(svenHandle, SVEN_SEVERITY_NORMAL,
			   "Hello world!", /*length */ 12);

	SVEN_FUNC_ENTER(svenHandle, SVEN_SEVERITY_NORMAL);
	SVEN_FUNC_ENTER_LOC16(svenHandle, SVEN_SEVERITY_NORMAL, 100);

	SVEN_CATALOG64_0(svenHandle, SVEN_SEVERITY_WARNING,
			 /*ID*/ 0xDECAFF00CAFEBABEull);
	SVEN_CATALOG64_1(svenHandle, SVEN_SEVERITY_WARNING,
			 /*ID*/ 0xDECAFF00CAFEBABEull, 1);
	SVEN_CATALOG64_2(svenHandle, SVEN_SEVERITY_WARNING,
			 /*ID*/ 0xDECAFF00CAFEBABEull, 1, 2);
	SVEN_CATALOG64_3(svenHandle, SVEN_SEVERITY_WARNING,
			 /*ID*/ 0xDECAFF00CAFEBABEull, 1, 2, 3);
	SVEN_CATALOG64_4(svenHandle, SVEN_SEVERITY_WARNING,
			 /*ID*/ 0xDECAFF00CAFEBABEull, 1, 2, 3, 4);

	SVEN_CATALOG32_0(svenHandle, SVEN_SEVERITY_WARNING, 0xCAFEBABEull);

	SVEN_CATALOG64_6_LOCADDR(svenHandle, SVEN_SEVERITY_WARNING,
				 /*I D */ 0xDECAFF00CAFEBABEull, 1, 2, 3,
				 4, 5, 6);
	SVEN_CATALOG64_6_LOC16(svenHandle, SVEN_SEVERITY_WARNING, 0x123,
			       /*ID*/ 0xDECAFF00CAFEBABEull, 1, 2, 3, 4, 5,
			       6);
	SVEN_CATALOG64_6_LOC32(svenHandle, SVEN_SEVERITY_WARNING, 0x123,
			       /*ID*/ 0xDECAFF00CAFEBABEull, 1, 2, 3, 4, 5,
			       6);

	SVEN_DEBUG_ASSERT(svenHandle, SVEN_SEVERITY_ERROR, 0);


	SVEN_API_CALL_6(svenHandle, SVEN_SEVERITY_NORMAL,	/*APISET */
			0x100, /*FUNCTION*/ 0xaa55, 1, 2, 3, 4, 5, 6);


	SVEN_API_RETURN(svenHandle, SVEN_SEVERITY_NORMAL,	/*APISET */
			0x100, /*FUNCTION*/ 0xaa55, 0x1122334455667788ull);

	SVEN_SHORT32(svenHandle, 0x1234);

}

/* older kernels don't have pr_alert */
#ifndef pr_alert
#define pr_alert pr_info
#endif

static int __init sventest_init(void)
{
	/* SVEN handle initialisation */
	svenHandle = SVEN_ALLOC_HANDLE(NULL);

	if (svenHandle == NULL) {
		pr_alert("Initializing SVEN handle failed\n");
		return -EFAULT;
	}
	SVEN_SET_HANDLE_GUID_UNIT(svenHandle, guid, /*unit */ 1);


	init_timer(&tml);
	tml.data = (unsigned long) &tml;
	tml.function = sventest_timer_tick;
	tml.expires = jiffies + HZ;	/* one second */

	/* launch slow timer */

	add_timer(&tml);


	/* simple debug print using SVEN */
	SVEN_DEBUG(svenHandle, SVEN_SEVERITY_NORMAL,
		   "SVENTEST initialized", sizeof("SVENTEST initialized"));

	test_sven_api();

	return 0;
}

static void __exit sventest_exit(void)
{

	del_timer_sync(&tml);

	SVEN_DEBUG(svenHandle, SVEN_SEVERITY_NORMAL, "SVENTEST: bye bye",
		   sizeof("SVENTEST: bye bye"));

	/* clean up SVEN handle */
	if (svenHandle != 0)
		SVEN_DELETE_HANDLE(svenHandle);
}


module_init(sventest_init);
module_exit(sventest_exit);
