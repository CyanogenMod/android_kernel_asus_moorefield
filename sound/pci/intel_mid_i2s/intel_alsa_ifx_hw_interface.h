/*
 *	intel_alsa_ssp_hw_interface.h
 *
 *  Copyright (C) 2010 Intel Corp
 *  Authors:	Selma Bensaid <selma.bensaid@intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#ifndef INTEL_ALSA_SSP_HW_INTERFACE_H_
#define INTEL_ALSA_SSP_HW_INTERFACE_H_

#include <linux/cdev.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/fcntl.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/workqueue.h>
#include <linux/firmware.h>
#include <linux/ioctl.h>
#include <asm/div64.h>

#include <sound/core.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/info.h>

#include <sound/intel_alsa_ssp_common.h>

int intel_alsa_ifx_control(int command,
	struct intel_alsa_ssp_stream_info *pl_str_info);
void intel_alsa_ifx_transfer_data(struct work_struct *work);
int intel_alsa_ifx_dma_capture_complete(void *param);
int intel_alsa_ifx_dma_playback_complete(void *param);
void intel_alsa_reset_ifx_status(void);

struct intel_alsa_stream_status {
	void *ssp_handle;
	spinlock_t lock;
	unsigned long stream_info;

};
#endif /* INTEL_ALSA_SSP_HW_INTERFACE_H_ */
