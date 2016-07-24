/*
 * Copyright (c) 2014 Intel Corporation. All Rights Reserved.
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

#ifndef CSS2600_ISYS_ISA_H
#define CSS2600_ISYS_ISA_H

#include <media/media-entity.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-core.h>

#include "css2600-isys-queue.h"
#include "css2600-isys-subdev.h"
#include "css2600-isys-video.h"

struct css2600_isys_isa_pdata;
struct css2600_isys;

#define ISA_PAD_SINK			0
#define ISA_PAD_SOURCE			1
#define NR_OF_ISA_PADS			2

/*
 * struct css2600_isys_isa
 */
struct css2600_isys_isa {
	struct css2600_isys_isa_pdata *pdata;
	struct css2600_isys_subdev asd;
	struct css2600_isys_video av;

	void __iomem *base;
};

#define to_css2600_isys_isa(sd)					\
	container_of(to_css2600_isys_subdev(sd), struct css2600_isys_isa, asd)

int css2600_isys_isa_init(struct css2600_isys_isa *isa,
			  struct css2600_isys *isys, void __iomem *base);
void css2600_isys_isa_cleanup(struct css2600_isys_isa *isa);
void css2600_isys_isa_isr(struct css2600_isys_isa *isa);

#endif /* CSS2600_ISYS_ISA_H */
