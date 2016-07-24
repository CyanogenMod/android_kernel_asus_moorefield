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

#include <linux/device.h>
#include <linux/dma-attrs.h>
#include <linux/iommu.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/sizes.h>

#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-core.h>

#include "css2600.h"
#include "css2600-bus.h"
#include "css2600-isys.h"
#include "css2600-isys-lib.h"
#include "css2600-isys-subdev.h"
#include "css2600-isys-tpg-2401.h"
#include "css2600-isys-video.h"

static const uint32_t tpg_supported_codes_pad[] = {
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

static const uint32_t *tpg_supported_codes[] = {
	tpg_supported_codes_pad,
};

static struct v4l2_subdev_internal_ops tpg_sd_internal_ops = {
	.open = css2600_isys_subdev_open,
	.close = css2600_isys_subdev_close,
};

static const struct v4l2_subdev_core_ops tpg_sd_core_ops = {
};

static int set_stream_one(struct css2600_isys_tpg *tpg)
{
	struct css2600_isys_pipeline *ip = tpg->isys->pipes[tpg->asd.source];
	int rval;

	dev_dbg(&tpg->isys->adev->dev,
		"set_stream_one: stopping stream again\n");

	reinit_completion(&ip->stream_stop_completion);
	rval = -ia_css_isys_stream_stop(tpg->isys->ssi, ip->source);
	if (rval < 0) {
		dev_dbg(&tpg->isys->adev->dev, "can't stop stream (%d)\n",
			rval);
	} else {
		wait_for_completion(&ip->stream_stop_completion);
		dev_dbg(&tpg->isys->adev->dev, "stream stop complete\n");
	}

	dev_dbg(&tpg->isys->adev->dev,
		"set_stream_one: starting stream again\n");

	reinit_completion(&ip->stream_start_completion);
	rval = -ia_css_isys_stream_start(tpg->isys->ssi, ip->source,
					 NULL);
	if (rval < 0) {
		dev_dbg(&tpg->isys->adev->dev, "can't start streaning (%d)\n",
			rval);
		return rval;
	}

	wait_for_completion(&ip->stream_start_completion);
	dev_dbg(&tpg->isys->adev->dev, "stream start complete\n");

	return 0;
}

static int set_stream(struct v4l2_subdev *sd, int enable)
{
	struct css2600_isys_tpg *tpg = to_css2600_isys_tpg(sd);
	struct v4l2_mbus_framefmt *ffmt = __css2600_isys_get_ffmt(
		sd, NULL, TPG_PAD_SOURCE, V4L2_SUBDEV_FORMAT_ACTIVE);
	struct pixelgen_tpg_cfg_s cfg = {
		.color_cfg = { 51, 102, 255, 0, 100, 160 },
		.sync_gen_cfg = {
			 .hblank_cycles = 100,
			 .vblank_cycles = 100,
			 .nr_of_frames = 1,
			 .pixels_per_line = ffmt->width,
			 .lines_per_frame = ffmt->height,
			 .pixels_per_clock = 1,
		 },
		.mode = PIXELGEN_TPG_MODE_CHBO,
		.mask_cfg = {
			.h_mask = (1<<4) - 1,
			.v_mask = (1<<4) - 1,
			.hv_mask = (1<<8) - 1,
		},
		.delta_cfg = {
			.h_delta = -2,
			.v_delta = 3,
		},
	};
	int rval;

	dev_dbg(&tpg->isys->adev->dev, "tpg s_stream %d\n", enable);

	if (!enable) {
		tpg->streaming = 0;
		return 0;
	}

	if (tpg->streaming)
		return set_stream_one(tpg);

	rval = ia_css_isysapi_rx_set_tpg_cfg(tpg->isys->ssi, tpg->asd.source,
					     &cfg);

	if (rval) {
		dev_info(&tpg->isys->adev->dev,
			 "tpg configuration failed (%d)\n", rval);
		return -EINVAL;
	}

	tpg->streaming = 1;

	return 0;
}

static const struct v4l2_subdev_video_ops tpg_sd_video_ops = {
	.s_stream = set_stream,
};

static const struct v4l2_subdev_pad_ops tpg_sd_pad_ops = {
	.get_fmt = css2600_isys_subdev_get_ffmt,
	.set_fmt = css2600_isys_subdev_set_ffmt,
};

static struct v4l2_subdev_ops tpg_sd_ops = {
	.core = &tpg_sd_core_ops,
	.video = &tpg_sd_video_ops,
	.pad = &tpg_sd_pad_ops,
};

static struct media_entity_operations tpg_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

void css2600_isys_tpg_2401_cleanup(struct css2600_isys_tpg *tpg)
{
	v4l2_device_unregister_subdev(&tpg->asd.sd);
	css2600_isys_subdev_cleanup(&tpg->asd);

	if (tpg->isys->pdata->type == CSS2600_ISYS_TYPE_CSS2401)
		return;

	css2600_isys_video_cleanup(&tpg->av);
}

int css2600_isys_tpg_2401_init(struct css2600_isys_tpg *tpg,
			       struct css2600_isys *isys,
			       void __iomem *base, unsigned int index)
{
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.pad = TPG_PAD_SOURCE,
		.format = {
			.width = 4096,
			.height = 3072,
			.code = V4L2_MBUS_FMT_SGRBG8_1X8,
		},
	};
	int rval;

	tpg->isys = isys;
	tpg->base = base;
	tpg->index = index;

	BUG_ON(CSS2600_ISYS_MAX_PAD < NR_OF_TPG_PADS);
	tpg->asd.pad[TPG_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	tpg->asd.sd.entity.ops = &tpg_entity_ops;

	rval = css2600_isys_subdev_init(&tpg->asd, &tpg_sd_ops, 0,
					NR_OF_TPG_PADS);
	if (rval)
		goto fail;

	tpg->asd.isys = isys;
	tpg->asd.source = IA_CSS_ISYS_STREAM_SRC_MIPIGEN_PORT0 + index;
	tpg->asd.supported_codes = tpg_supported_codes;
	css2600_isys_subdev_set_ffmt(&tpg->asd.sd, NULL, &fmt);

	tpg->asd.sd.internal_ops = &tpg_sd_internal_ops;
	snprintf(tpg->asd.sd.name, sizeof(tpg->asd.sd.name),
		 CSS2600_NAME " TPG %u", index);
	v4l2_set_subdevdata(&tpg->asd.sd, &tpg->asd);
	rval = v4l2_device_register_subdev(&isys->v4l2_dev, &tpg->asd.sd);
	if (rval) {
		dev_info(&isys->adev->dev, "can't register v4l2 subdev\n");
		goto fail;
	}

	if (isys->pdata->type == CSS2600_ISYS_TYPE_CSS2401)
		return 0;

	snprintf(tpg->av.vdev.name, sizeof(tpg->av.vdev.name),
		 CSS2600_NAME " TPG %u capture", index);
	tpg->av.isys = isys;
	tpg->av.aq.css_pin_type = IA_CSS_ISYS_PIN_TYPE_RAW_NS;
	rval = css2600_isys_video_init(
		&tpg->av, &tpg->asd.sd.entity, TPG_PAD_SOURCE,
		MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE);
	if (rval) {
		dev_info(&isys->adev->dev, "can't init video node\n");
		goto fail;
	}

	return 0;

fail:
	css2600_isys_tpg_2401_cleanup(tpg);

	return rval;
}

void css2600_isys_tpg_2401_isr(struct css2600_isys_tpg *tpg)
{
}
