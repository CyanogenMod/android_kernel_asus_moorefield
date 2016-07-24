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

#ifndef CSS2600_ISYS_TPG_H
#define CSS2600_ISYS_TPG_H

#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-core.h>

#include "css2600-isys-subdev.h"
#include "css2600-isys-video.h"
#include "css2600-isys-queue.h"

struct css2600_isys_tpg_pdata;
struct css2600_isys;

#define TPG_PAD_SOURCE			0
#define NR_OF_TPG_PADS			1

/*
 * struct css2600_isys_tpg
 *
 * @nlanes: number of lanes in the receiver
 */
struct css2600_isys_tpg {
	struct css2600_isys_tpg_pdata *pdata;
	struct css2600_isys *isys;
	struct css2600_isys_subdev asd;
	struct css2600_isys_video av;

	void __iomem *base;
	unsigned int index;
	int streaming;

	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *pixel_rate;
};

#define to_css2600_isys_tpg(sd)					\
	container_of(to_css2600_isys_subdev(sd), struct css2600_isys_tpg, asd)

int css2600_isys_tpg_init(struct css2600_isys_tpg *tpg, struct css2600_isys *isys,
		      void __iomem *base, unsigned int index);
void css2600_isys_tpg_cleanup(struct css2600_isys_tpg *tpg);
void css2600_isys_tpg_isr(struct css2600_isys_tpg *tpg);

#endif /* CSS2600_ISYS_TPG_H */
