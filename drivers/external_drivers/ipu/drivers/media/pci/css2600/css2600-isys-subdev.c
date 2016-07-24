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

#include <linux/types.h>
#include <linux/v4l2-mediabus.h>
#include <linux/videodev2.h>

#include <media/media-entity.h>

#include "css2600-isys.h"
#include "css2600-isys-video.h"
#include "css2600-isys-subdev.h"
#include <cssapi.h>

static struct css2600_isys_ffmt_entry ffmts[] = {
	{ V4L2_MBUS_FMT_SBGGR12_1X12, CSS2600_ISYS_MIPI_CSI2_TYPE_RAW12, 12, },
	{ V4L2_MBUS_FMT_SGBRG12_1X12, CSS2600_ISYS_MIPI_CSI2_TYPE_RAW12, 12, },
	{ V4L2_MBUS_FMT_SGRBG12_1X12, CSS2600_ISYS_MIPI_CSI2_TYPE_RAW12, 12, },
	{ V4L2_MBUS_FMT_SRGGB12_1X12, CSS2600_ISYS_MIPI_CSI2_TYPE_RAW12, 12, },
	{ V4L2_MBUS_FMT_SBGGR10_1X10, CSS2600_ISYS_MIPI_CSI2_TYPE_RAW10, 10, },
	{ V4L2_MBUS_FMT_SGBRG10_1X10, CSS2600_ISYS_MIPI_CSI2_TYPE_RAW10, 10, },
	{ V4L2_MBUS_FMT_SGRBG10_1X10, CSS2600_ISYS_MIPI_CSI2_TYPE_RAW10, 10, },
	{ V4L2_MBUS_FMT_SRGGB10_1X10, CSS2600_ISYS_MIPI_CSI2_TYPE_RAW10, 10, },
	{ V4L2_MBUS_FMT_SBGGR8_1X8, CSS2600_ISYS_MIPI_CSI2_TYPE_RAW8, 8, },
	{ V4L2_MBUS_FMT_SGBRG8_1X8, CSS2600_ISYS_MIPI_CSI2_TYPE_RAW8, 8, },
	{ V4L2_MBUS_FMT_SGRBG8_1X8, CSS2600_ISYS_MIPI_CSI2_TYPE_RAW8, 8, },
	{ V4L2_MBUS_FMT_SRGGB8_1X8, CSS2600_ISYS_MIPI_CSI2_TYPE_RAW8, 8, },
};

struct v4l2_mbus_framefmt *__css2600_isys_get_ffmt(
	struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
	unsigned int pad, unsigned int which)
{
	struct css2600_isys_subdev *asd = to_css2600_isys_subdev(sd);

	if (which == V4L2_SUBDEV_FORMAT_ACTIVE)
		return &asd->ffmt[pad];
	else
		return v4l2_subdev_get_try_format(fh, pad);
}

struct v4l2_rect *__css2600_isys_get_selection(
	struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh, unsigned int target,
	unsigned int pad, unsigned int which)
{
	struct css2600_isys_subdev *asd = to_css2600_isys_subdev(sd);

	if (which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		switch (target) {
		case V4L2_SEL_TGT_CROP:
			return &asd->crop[pad];
		case V4L2_SEL_TGT_COMPOSE:
			/*
			 * Compose is valid for sink pads only, and
			 * for css2600 sink pads are always zero.
			 */
			BUG_ON(pad);
			return &asd->compose;
		}
	} else {
		switch (target) {
		case V4L2_SEL_TGT_CROP:
			return v4l2_subdev_get_try_crop(fh, pad);
		case V4L2_SEL_TGT_COMPOSE:
			BUG_ON(pad);
			return v4l2_subdev_get_try_compose(fh, pad);
		}
	}
	BUG();
}

static int target_valid(struct v4l2_subdev *sd, unsigned int target,
			unsigned int pad)
{
	struct css2600_isys_subdev *asd = to_css2600_isys_subdev(sd);

	switch (target) {
	case V4L2_SEL_TGT_CROP:
		return asd->valid_tgts[pad].crop;
	case V4L2_SEL_TGT_COMPOSE:
		return asd->valid_tgts[pad].compose;
	default:
		return 0;
	}
}

static void fmt_propagate(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			  struct v4l2_mbus_framefmt *ffmt, struct v4l2_rect *r,
			  enum isys_subdev_prop_tgt tgt, unsigned int pad,
			  unsigned int which)
{
	struct v4l2_mbus_framefmt *ffmts[CSS2600_ISYS_MAX_PAD];
	struct v4l2_rect *crops[CSS2600_ISYS_MAX_PAD];
	struct v4l2_rect *compose;
	unsigned int i;

	if (tgt == CSS2600_ISYS_SUBDEV_PROP_TGT_NR_OF)
		return;

	for (i = 0; i < sd->entity.num_pads; i++) {
		ffmts[i] = __css2600_isys_get_ffmt(sd, fh, i, which);
		crops[i] = __css2600_isys_get_selection(
			sd, fh, V4L2_SEL_TGT_CROP, i, which);
	}
	compose = __css2600_isys_get_selection(sd, fh, V4L2_SEL_TGT_COMPOSE,
					       0, which);

	switch (tgt) {
	case CSS2600_ISYS_SUBDEV_PROP_TGT_SINK_FMT:
		crops[pad]->left = crops[pad]->top = 0;
		crops[pad]->width = ffmt->width;
		crops[pad]->height = ffmt->height;
		for (i = 1; i < sd->entity.num_pads; i++)
			ffmts[i]->code = ffmt->code;
		fmt_propagate(sd, fh, NULL, crops[pad], tgt + 1, pad, which);
		return;
	case CSS2600_ISYS_SUBDEV_PROP_TGT_SINK_CROP:
		compose->left = compose->top = 0;
		compose->width = r->width;
		compose->height = r->height;
		fmt_propagate(sd, fh, NULL, compose, tgt + 1, pad, which);
		return;
	case CSS2600_ISYS_SUBDEV_PROP_TGT_SINK_COMPOSE:
		for (i = 1; i < sd->entity.num_pads; i++) {
			crops[pad]->left = crops[pad]->top = 0;
			crops[pad]->width = r->width;
			crops[pad]->height = r->height;
			fmt_propagate(sd, fh, NULL, crops[pad], tgt + 1, i,
				      which);
		}
		return;
	case CSS2600_ISYS_SUBDEV_PROP_TGT_SOURCE_CROP:
		ffmts[pad]->width = r->width;
		ffmts[pad]->height = r->height;
		return;
	}
}

int __css2600_isys_subdev_set_ffmt(struct v4l2_subdev *sd,
				 struct v4l2_subdev_fh *fh,
				 struct v4l2_subdev_format *fmt)
{
	struct css2600_isys_subdev *asd = to_css2600_isys_subdev(sd);
	struct v4l2_mbus_framefmt *ffmt =
		__css2600_isys_get_ffmt(sd, fh, fmt->pad, fmt->which);
	uint32_t code = asd->supported_codes[fmt->pad][0];
	unsigned int i;

	BUG_ON(!mutex_is_locked(&asd->mutex));

	for (i = 0; asd->supported_codes[fmt->pad][i]; i++) {
		if (asd->supported_codes[fmt->pad][i] == fmt->format.code) {
			code = asd->supported_codes[fmt->pad][i];
			break;
		}
	}

	for (i = 0; i < ARRAY_SIZE(ffmts); i++) {
		if (ffmts[i].code == code) {
			asd->ffmt_entry = ffmts + i;
			break;
		}
	}

	BUG_ON(!asd->ffmt_entry);

	fmt->format.code = code;

	if (!fmt->pad) {
		/* This is either a sink pad or an entity has no sink pads. */
		*ffmt = fmt->format;

		fmt_propagate(sd, fh, ffmt, NULL,
			      CSS2600_ISYS_SUBDEV_PROP_TGT_SINK_FMT, fmt->pad,
			      fmt->which);
	} else {
		/* Source pad */
		if (asd->allow_source_fmt_change)
			ffmt->code = fmt->format.code;
	}

	fmt->format = *ffmt;

	return 0;
}

int css2600_isys_subdev_set_ffmt(struct v4l2_subdev *sd,
				 struct v4l2_subdev_fh *fh,
				 struct v4l2_subdev_format *fmt)
{
	struct css2600_isys_subdev *asd = to_css2600_isys_subdev(sd);
	int rval;

	mutex_lock(&asd->mutex);
	rval = __css2600_isys_subdev_set_ffmt(sd, fh, fmt);
	mutex_unlock(&asd->mutex);

	return rval;
}

int css2600_isys_subdev_get_ffmt(struct v4l2_subdev *sd,
				 struct v4l2_subdev_fh *fh,
				 struct v4l2_subdev_format *fmt)
{
	struct css2600_isys_subdev *asd = to_css2600_isys_subdev(sd);

	mutex_lock(&asd->mutex);
	fmt->format = *__css2600_isys_get_ffmt(sd, fh, fmt->pad, fmt->which);
	mutex_unlock(&asd->mutex);

	return 0;
}

int css2600_isys_subdev_set_sel(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh,
				struct v4l2_subdev_selection *fmt)
{
	struct css2600_isys_subdev *asd = to_css2600_isys_subdev(sd);
	struct media_pad *pad = &asd->sd.entity.pads[fmt->pad];
	unsigned int tgt;

	if (!target_valid(sd, fmt->target, fmt->pad))
		return -EINVAL;

	switch (fmt->target) {
	case V4L2_SEL_TGT_CROP:
		tgt = pad->flags & MEDIA_PAD_FL_SINK
			? CSS2600_ISYS_SUBDEV_PROP_TGT_SINK_CROP
			: CSS2600_ISYS_SUBDEV_PROP_TGT_SOURCE_CROP;
		break;
	case V4L2_SEL_TGT_COMPOSE:
		tgt = CSS2600_ISYS_SUBDEV_PROP_TGT_SINK_COMPOSE;
		break;
	default:
		BUG();
	}

	fmt_propagate(sd, fh, NULL, &fmt->r, tgt, fmt->pad, fmt->which);

	return 0;
}

int css2600_isys_subdev_get_sel(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh,
				struct v4l2_subdev_selection *fmt)
{
	if (!target_valid(sd, fmt->target, fmt->pad))
		return -EINVAL;

	fmt->r = *__css2600_isys_get_selection(sd, fh, fmt->target, fmt->pad,
					       fmt->which);

	return 0;
}

/*
 * Besides validating the link, figure out the external pad and the
 * ISYS library source.
 */
int css2600_isys_subdev_link_validate(
	struct v4l2_subdev *sd, struct media_link *link,
	struct v4l2_subdev_format *source_fmt,
	struct v4l2_subdev_format *sink_fmt)
{
	struct v4l2_subdev *source_sd =
		media_entity_to_v4l2_subdev(link->source->entity);
	struct css2600_isys_pipeline *ip =
		container_of(sd->entity.pipe,
			     struct css2600_isys_pipeline, pipe);
	struct css2600_isys_subdev *asd = to_css2600_isys_subdev(sd);

	if (source_sd->owner != THIS_MODULE) {
		/*
		 * source_sd isn't ours --- sd must be the external
		 * sub-device.
		 */
		ip->external = link->source;
		ip->source = to_css2600_isys_subdev(sd)->source;
		dev_dbg(&asd->isys->adev->dev, "%s: using source %d\n",
			sd->entity.name, ip->source);
	} else if (source_sd->entity.num_pads == 1) {
		/* All internal sources have a single pad. */
		ip->external = link->source;
		ip->source = to_css2600_isys_subdev(source_sd)->source;
		if (to_css2600_isys_subdev(source_sd)->part_of_isl)
			ip->isl_enabled = true;

		dev_dbg(&asd->isys->adev->dev, "%s: using source %d\n",
			sd->entity.name, ip->source);
	}

	if (to_css2600_isys_subdev(sd)->part_of_isl)
		ip->isl_enabled = true;

	return v4l2_subdev_link_validate_default(sd, link, source_fmt,
						 sink_fmt);
}

int css2600_isys_subdev_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct css2600_isys_subdev *asd = to_css2600_isys_subdev(sd);
	struct v4l2_rect *try_compose = v4l2_subdev_get_try_crop(fh, 0);
	unsigned int i;

	mutex_lock(&asd->mutex);

	*try_compose = asd->compose;

	for (i = 0; i < asd->sd.entity.num_pads; i++) {
		struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_get_try_format(fh, i);
		struct v4l2_rect *try_crop =
			v4l2_subdev_get_try_crop(fh, i);

		*try_fmt = asd->ffmt[i];
		*try_crop = asd->crop[i];
	}

	mutex_unlock(&asd->mutex);

	return 0;
}

int css2600_isys_subdev_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return 0;
}

int css2600_isys_subdev_init(struct css2600_isys_subdev *asd,
			     struct v4l2_subdev_ops *ops, unsigned int nr_ctrls,
			     unsigned int num_pads)
{
	int rval;

	mutex_init(&asd->mutex);

	v4l2_subdev_init(&asd->sd, ops);

	asd->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	asd->sd.owner = THIS_MODULE;

	rval = media_entity_init(&asd->sd.entity, num_pads, asd->pad, 0);
	if (rval)
		goto out_mutex_destroy;

	if (asd->ctrl_init) {
		rval = v4l2_ctrl_handler_init(&asd->ctrl_handler, nr_ctrls);
		if (rval)
			goto out_media_entity_cleanup;

		asd->ctrl_init(&asd->sd);
		if (asd->ctrl_handler.error) {
			rval = asd->ctrl_handler.error;
			goto out_v4l2_ctrl_handler_free;
		}

		asd->sd.ctrl_handler = &asd->ctrl_handler;
	}

	asd->source = -1;

	return 0;

out_v4l2_ctrl_handler_free:
	v4l2_ctrl_handler_free(&asd->ctrl_handler);

out_media_entity_cleanup:
	media_entity_cleanup(&asd->sd.entity);

out_mutex_destroy:
	mutex_destroy(&asd->mutex);

	return rval;
}

void css2600_isys_subdev_cleanup(struct css2600_isys_subdev *asd)
{
	media_entity_cleanup(&asd->sd.entity);
	v4l2_ctrl_handler_free(&asd->ctrl_handler);
	mutex_destroy(&asd->mutex);
}
