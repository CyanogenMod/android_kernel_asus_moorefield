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
#include "css2600-isys-tpg.h"
#include "css2600-isys-tpg-reg.h"
#include "css2600-isys-video.h"

static const uint32_t tpg_supported_codes_pad[] = {
	V4L2_MBUS_FMT_SBGGR8_1X8,
	0,
};

static const uint32_t *tpg_supported_codes[] = {
	tpg_supported_codes_pad,
};

static int tpg_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
			      struct v4l2_mbus_frame_desc *desc)
{
	struct css2600_isys_tpg *tpg = to_css2600_isys_tpg(sd);
	struct v4l2_mbus_frame_desc_entry *entry = desc->entry;

	desc->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
	entry->bpp = tpg->asd.ffmt_entry->compressed;
	entry->pixelcode = tpg->asd.ffmt_entry->code;
	entry->size.two_dim.width = tpg->asd.ffmt[pad].width;
	entry->size.two_dim.height = tpg->asd.ffmt[pad].height;
	entry->bus.csi2.data_type = tpg->asd.ffmt_entry->mipi_data_type;
	desc->num_entries = 1;

	return 0;
}

static struct v4l2_subdev_internal_ops tpg_sd_internal_ops = {
	.open = css2600_isys_subdev_open,
	.close = css2600_isys_subdev_close,
};

static const struct v4l2_subdev_core_ops tpg_sd_core_ops = {
};

static int set_stream(struct v4l2_subdev *sd, int enable)
{
	struct css2600_isys_tpg *tpg = to_css2600_isys_tpg(sd);

	if (!enable) {
		writel(0, tpg->base + MIPI_GEN_REG_COM_ENABLE);
		return 0;
	}

	writel(0, tpg->base + MIPI_GEN_REG_COM_DTYPE);
	writel(tpg->asd.ffmt_entry->mipi_data_type,
	       tpg->base + MIPI_GEN_REG_COM_VTYPE);
	writel(0, tpg->base + MIPI_GEN_REG_COM_VCHAN);
	writel(tpg->asd.ffmt[0].width,
	       tpg->base + MIPI_GEN_REG_COM_WCOUNT);

	writel(0, tpg->base + MIPI_GEN_REG_SYNG_NOF_FRAMES);
	writel(tpg->asd.ffmt[0].width /
	       css2600_isys_bpp_to_ppc(tpg->asd.ffmt_entry->compressed),
	       tpg->base + MIPI_GEN_REG_SYNG_NOF_PIXELS);
	writel(tpg->asd.ffmt[0].height,
	       tpg->base + MIPI_GEN_REG_SYNG_NOF_LINES);

	writel(0, tpg->base + MIPI_GEN_REG_TPG_MODE);
	writel(-1, tpg->base + MIPI_GEN_REG_TPG_HCNT_MASK);
	writel(-1, tpg->base + MIPI_GEN_REG_TPG_VCNT_MASK);
	writel(-1, tpg->base + MIPI_GEN_REG_TPG_XYCNT_MASK);
	writel(0, tpg->base + MIPI_GEN_REG_TPG_HCNT_DELTA);
	writel(0, tpg->base + MIPI_GEN_REG_TPG_VCNT_DELTA);

	v4l2_ctrl_handler_setup(&tpg->asd.ctrl_handler);

	writel(2, tpg->base + MIPI_GEN_REG_COM_ENABLE);
	return 0;
}

static const struct v4l2_subdev_video_ops tpg_sd_video_ops = {
	.s_stream = set_stream,
};

static int css2600_isys_tpg_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct css2600_isys_tpg *tpg =
		container_of(container_of(ctrl->handler,
					  struct css2600_isys_subdev,
					  ctrl_handler),
			     struct css2600_isys_tpg, asd);

	switch (ctrl->id) {
	case V4L2_CID_HBLANK:
		writel(tpg->asd.ffmt[0].width + ctrl->val,
		       tpg->base + MIPI_GEN_REG_SYNG_HBLANK_CYC);
		break;
	case V4L2_CID_VBLANK:
		writel(tpg->asd.ffmt[0].height + ctrl->val,
		       tpg->base + MIPI_GEN_REG_SYNG_VBLANK_CYC);
		break;
	}

	return 0;
}

static const struct v4l2_ctrl_ops css2600_isys_tpg_ctrl_ops = {
	.s_ctrl = css2600_isys_tpg_s_ctrl,
};

static int64_t css2600_isys_tpg_rate(struct css2600_isys_tpg *tpg,
				     unsigned int bpp)
{
	int64_t rate = css2600_isys_bpp_to_ppc(bpp);

	switch (tpg->isys->pdata->type) {
	case CSS2600_ISYS_TYPE_CSS2600_FPGA:
		return rate * CSS2600_ISYS_FREQ_BXT_FPGA;
		break;
	case CSS2600_ISYS_TYPE_CSS2600:
		return rate * CSS2600_ISYS_FREQ_BXT_A0;
		break;
	default:
		return rate; /* SLE, right? :-) */
	}
}

static void css2600_isys_tpg_init_controls(struct v4l2_subdev *sd)
{
	struct css2600_isys_tpg *tpg = to_css2600_isys_tpg(sd);

	tpg->hblank = v4l2_ctrl_new_std(
		&tpg->asd.ctrl_handler, &css2600_isys_tpg_ctrl_ops,
		V4L2_CID_HBLANK, 8, 65535, 1, 1024);

	tpg->vblank = v4l2_ctrl_new_std(
		&tpg->asd.ctrl_handler, &css2600_isys_tpg_ctrl_ops,
		V4L2_CID_VBLANK, 8, 65535, 1, 1024);

	tpg->pixel_rate = v4l2_ctrl_new_std(
		&tpg->asd.ctrl_handler, &css2600_isys_tpg_ctrl_ops,
		V4L2_CID_PIXEL_RATE, 0, 0, 1, 0);

	tpg->pixel_rate->cur.val = css2600_isys_tpg_rate(tpg, 8);

	if (tpg->pixel_rate)
		tpg->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;
}

static int css2600_isys_tpg_set_ffmt(struct v4l2_subdev *sd,
				     struct v4l2_subdev_fh *fh,
				     struct v4l2_subdev_format *fmt)
{
	struct css2600_isys_tpg *tpg = to_css2600_isys_tpg(sd);
	struct css2600_isys_ffmt_entry *entry;
	int rval;

	mutex_lock(&tpg->asd.mutex);
	rval = __css2600_isys_subdev_set_ffmt(sd, fh, fmt);
	entry = tpg->asd.ffmt_entry;
	mutex_unlock(&tpg->asd.mutex);

	if (rval || fmt->which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return rval;

	v4l2_ctrl_s_ctrl_int64(tpg->pixel_rate,
			       css2600_isys_tpg_rate(tpg, entry->compressed));

	return 0;
}

static const struct v4l2_subdev_pad_ops tpg_sd_pad_ops = {
	.get_fmt = css2600_isys_subdev_get_ffmt,
	.set_fmt = css2600_isys_tpg_set_ffmt,
	.get_frame_desc = tpg_get_frame_desc,
};

static struct v4l2_subdev_ops tpg_sd_ops = {
	.core = &tpg_sd_core_ops,
	.video = &tpg_sd_video_ops,
	.pad = &tpg_sd_pad_ops,
};

static struct media_entity_operations tpg_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

void css2600_isys_tpg_cleanup(struct css2600_isys_tpg *tpg)
{
	v4l2_device_unregister_subdev(&tpg->asd.sd);
	css2600_isys_subdev_cleanup(&tpg->asd);
	css2600_isys_video_cleanup(&tpg->av);
}

int css2600_isys_tpg_init(struct css2600_isys_tpg *tpg, struct css2600_isys *isys,
		      void __iomem *base, unsigned int index)
{
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.pad = TPG_PAD_SOURCE,
		.format = {
			.width = 4096,
			.height = 3072,
		},
	};
	int rval;

	tpg->isys = isys;
	tpg->base = base;
	tpg->index = index;

	BUG_ON(CSS2600_ISYS_MAX_PAD < NR_OF_TPG_PADS);
	tpg->asd.pad[TPG_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	tpg->asd.sd.entity.ops = &tpg_entity_ops;
	tpg->asd.ctrl_init = css2600_isys_tpg_init_controls;

	rval = css2600_isys_subdev_init(&tpg->asd, &tpg_sd_ops, 3,
					NR_OF_TPG_PADS);
	if (rval)
		return rval;

	tpg->asd.isys = isys;
	tpg->asd.source = IA_CSS_ISYS_STREAM_SRC_CSI2_PORT0 + index;
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
	tpg->av.aq.css_pin_type = IA_CSS_ISYS_PIN_TYPE_MIPI;
	tpg->av.mipi_csi2_fmt = true;
	rval = css2600_isys_video_init(
		&tpg->av, &tpg->asd.sd.entity, TPG_PAD_SOURCE, 0);
	if (rval) {
		dev_info(&isys->adev->dev, "can't init video node\n");
		goto fail;
	}

	return 0;

fail:
	css2600_isys_tpg_cleanup(tpg);

	return rval;
}

void css2600_isys_tpg_isr(struct css2600_isys_tpg *tpg)
{
}
