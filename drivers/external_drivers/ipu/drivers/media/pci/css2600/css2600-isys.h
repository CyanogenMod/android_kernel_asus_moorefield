/*
 * Copyright (c) 2013--2014 Intel Corporation. All Rights Reserved.
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

#ifndef CSS2600_ISYS_H
#define CSS2600_ISYS_H

#include <linux/spinlock.h>

#include <media/v4l2-device.h>
#include <media/media-device.h>

#include "css2600.h"
#include "css2600-isys-csi2.h"
#include "css2600-isys-isa.h"
#include "css2600-isys-lib.h"
#include "css2600-isys-tpg.h"
#include "css2600-isys-video.h"
#include "css2600-pdata.h"

/*
 * This might be really 64 bytes. This is a hack: psys binaries just
 * expect 256 bytes, but the CSS library is missing this
 * information.
 */
#define CSS2600_ISYS_2401_MEM_LINE_ALIGN	256
#define CSS2600_ISYS_2600_MEM_LINE_ALIGN	64

#define CSS2600_ISYS_FREQ_BXT_FPGA		25000000UL
#define CSS2600_ISYS_FREQ_BXT_A0		533000000UL

struct task_struct;

/*
 * struct css2600_isys
 *
 * @lock: serialise access to pipes
 * @pipes: pipelines per stream ID
 * @ssi: ssi library private pointer
 * @line_align: line alignment in memory
 */
struct css2600_isys {
	struct media_device media_dev;
	struct v4l2_device v4l2_dev;
	struct css2600_bus_device *adev;
	spinlock_t lock;
	struct css2600_isys_pipeline *pipes[N_IA_CSS_ISYS_STREAM_SRC];
	spinlock_t lib_lock;
	void *ssi;
	unsigned int line_align;
	unsigned int video_opened;
	struct task_struct *isr_thread;

	struct css2600_isys_pdata *pdata;

	struct css2600_isys_csi2 csi2[CSS2600_ISYS_MAX_CSI2_PORTS];
	struct css2600_isys_tpg tpg[CSS2600_ISYS_MAX_TPGS];
	struct css2600_isys_isa isa;
};

extern const struct v4l2_ioctl_ops css2600_isys_ioctl_ops;

int css2600_pipeline_pm_use(struct media_entity *entity, int use);
int css2600_isys_isr_run(void *ptr);

#define css2600_lib_call(func, isys, ...)				\
	({								\
		 int rval;						\
									\
		 rval = -ia_css_isys_##func((isys)->ssi, __VA_ARGS__);	\
									\
		 rval;							\
	})

#endif /* CSS2600_ISYS_H */
