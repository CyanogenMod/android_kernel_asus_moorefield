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

#include <linux/device.h>
#include <linux/dma-attrs.h>
#include <linux/iommu.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/sizes.h>

#include <media/css2600-isys.h>
#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-core.h>

#include "css2600.h"
#include "css2600-bus.h"
#include "css2600-isys.h"
#include "css2600-isys-isa.h"
#include "css2600-isys-lib.h"
#include "css2600-isys-subdev.h"
#include "css2600-isys-video.h"

static const uint32_t isa_supported_codes_pad[] = {
	V4L2_MBUS_FMT_SBGGR12_1X12,
	V4L2_MBUS_FMT_SGBRG12_1X12,
	V4L2_MBUS_FMT_SGRBG12_1X12,
	V4L2_MBUS_FMT_SRGGB12_1X12,
	V4L2_MBUS_FMT_SBGGR10_1X10,
	V4L2_MBUS_FMT_SGBRG10_1X10,
	V4L2_MBUS_FMT_SGRBG10_1X10,
	V4L2_MBUS_FMT_SRGGB10_1X10,
	V4L2_MBUS_FMT_SBGGR8_1X8,
	V4L2_MBUS_FMT_SGBRG8_1X8,
	V4L2_MBUS_FMT_SGRBG8_1X8,
	V4L2_MBUS_FMT_SRGGB8_1X8,
	0,
};

static const uint32_t *isa_supported_codes[] = {
	isa_supported_codes_pad,
	isa_supported_codes_pad,
};

static struct v4l2_subdev_internal_ops isa_sd_internal_ops = {
	.open = css2600_isys_subdev_open,
	.close = css2600_isys_subdev_close,
};

static const struct v4l2_subdev_core_ops isa_sd_core_ops = {
};

static int set_stream(struct v4l2_subdev *sd, int enable)
{
	return 0;
}

static const struct v4l2_subdev_video_ops isa_sd_video_ops = {
	.s_stream = set_stream,
};

static const struct v4l2_subdev_pad_ops isa_sd_pad_ops = {
	.link_validate = css2600_isys_subdev_link_validate,
	.get_fmt = css2600_isys_subdev_get_ffmt,
	.set_fmt = css2600_isys_subdev_set_ffmt,
};

static struct v4l2_subdev_ops isa_sd_ops = {
	.core = &isa_sd_core_ops,
	.video = &isa_sd_video_ops,
	.pad = &isa_sd_pad_ops,
};

static struct media_entity_operations isa_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

void css2600_isys_isa_cleanup(struct css2600_isys_isa *isa)
{
	v4l2_device_unregister_subdev(&isa->asd.sd);
	css2600_isys_subdev_cleanup(&isa->asd);
	css2600_isys_video_cleanup(&isa->av);
}

int css2600_isys_isa_init(struct css2600_isys_isa *isa,
			  struct css2600_isys *isys, void __iomem *base)
{
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.pad = ISA_PAD_SINK,
		.format = {
			.width = 4096,
			.height = 3072,
		},
	};
	int rval;

	isa->base = base;

	BUG_ON(CSS2600_ISYS_MAX_PAD < NR_OF_ISA_PADS);
	isa->asd.pad[ISA_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	isa->asd.pad[ISA_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	isa->asd.sd.entity.ops = &isa_entity_ops;

	rval = css2600_isys_subdev_init(&isa->asd, &isa_sd_ops, 0,
					NR_OF_ISA_PADS);
	if (rval)
		goto fail;

	isa->asd.isys = isys;
	isa->asd.part_of_isl = true;
	isa->asd.supported_codes = isa_supported_codes;
	css2600_isys_subdev_set_ffmt(&isa->asd.sd, NULL, &fmt);

	isa->asd.sd.internal_ops = &isa_sd_internal_ops;
	snprintf(isa->asd.sd.name, sizeof(isa->asd.sd.name),
		 CSS2600_NAME " ISA");
	v4l2_set_subdevdata(&isa->asd.sd, &isa->asd);
	rval = v4l2_device_register_subdev(&isys->v4l2_dev, &isa->asd.sd);
	if (rval) {
		dev_info(&isys->adev->dev, "can't register v4l2 subdev\n");
		goto fail;
	}

	snprintf(isa->av.vdev.name, sizeof(isa->av.vdev.name),
		 CSS2600_NAME " ISA capture");
	isa->av.isys = isys;
	isa->av.aq.css_pin_type = IA_CSS_ISYS_PIN_TYPE_RAW_NS;
	rval = css2600_isys_video_init(
		&isa->av, &isa->asd.sd.entity, ISA_PAD_SOURCE, 0);
	if (rval) {
		dev_info(&isys->adev->dev, "can't init video node\n");
		goto fail;
	}

	return 0;

fail:
	css2600_isys_isa_cleanup(isa);

	return rval;
}

void css2600_isys_isa_isr(struct css2600_isys_isa *isa)
{
}
