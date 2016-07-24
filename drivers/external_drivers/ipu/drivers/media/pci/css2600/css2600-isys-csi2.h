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

#ifndef CSS2600_ISYS_CSI2_H
#define CSS2600_ISYS_CSI2_H

#include <media/media-entity.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-core.h>

#include "css2600-isys-queue.h"
#include "css2600-isys-subdev.h"
#include "css2600-isys-video.h"

struct css2600_isys_csi2_pdata;
struct css2600_isys;

#define CSI2_PAD_SINK			0
#define CSI2_PAD_SOURCE			1
#define NR_OF_CSI2_PADS			2

#define CSS2600_ISYS_CSI2_SENSOR_CFG_LANE_CLOCK	0
#define CSS2600_ISYS_CSI2_SENSOR_CFG_LANE_DATA(n)	((n) + 1)

/*
 * struct css2600_isys_csi2
 *
 * @nlanes: number of lanes in the receiver
 */
struct css2600_isys_csi2 {
	struct css2600_isys_csi2_pdata *pdata;
	struct css2600_isys *isys;
	struct css2600_isys_subdev asd;
	struct css2600_isys_video av;

	void __iomem *base;
	unsigned int nlanes;
	unsigned int index;
};

struct css2600_isys_csi2_timing {
	uint32_t ctermen;
	uint32_t csettle;
	uint32_t dtermen;
	uint32_t dsettle;
};

#define to_css2600_isys_csi2(sd)					\
	container_of(to_css2600_isys_subdev(sd), struct css2600_isys_csi2, asd)

int css2600_isys_csi2_calc_timing(struct css2600_isys_csi2 *csi2,
				  struct css2600_isys_csi2_timing *timing,
				  uint32_t accinv);
int css2600_isys_csi2_init(struct css2600_isys_csi2 *csi2, struct css2600_isys *isys,
		      void __iomem *base, unsigned int lanes,
		      unsigned int index);
void css2600_isys_csi2_cleanup(struct css2600_isys_csi2 *csi2);
void css2600_isys_csi2_isr(struct css2600_isys_csi2 *csi2);

#endif /* CSS2600_ISYS_CSI2_H */
