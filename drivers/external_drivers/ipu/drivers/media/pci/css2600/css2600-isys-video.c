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

#include <linux/init_task.h>
#include <linux/kthread.h>

#include <media/media-entity.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>

#include "css2600-isys.h"
#include "css2600-isys-lib.h"
#include "css2600-isys-video.h"

static const struct css2600_isys_pixelformat isys_pfmts[] = {
	{ V4L2_PIX_FMT_SBGGR12, 16, 12, V4L2_MBUS_FMT_SBGGR12_1X12, IA_CSS_ISYS_FRAME_FORMAT_RAW, CSS2600_ISYS_MIPI_CSI2_TYPE_RAW12 },
	{ V4L2_PIX_FMT_SGBRG12, 16, 12, V4L2_MBUS_FMT_SGBRG12_1X12, IA_CSS_ISYS_FRAME_FORMAT_RAW, CSS2600_ISYS_MIPI_CSI2_TYPE_RAW12 },
	{ V4L2_PIX_FMT_SGRBG12, 16, 12, V4L2_MBUS_FMT_SGRBG12_1X12, IA_CSS_ISYS_FRAME_FORMAT_RAW, CSS2600_ISYS_MIPI_CSI2_TYPE_RAW12 },
	{ V4L2_PIX_FMT_SRGGB12, 16, 12, V4L2_MBUS_FMT_SRGGB12_1X12, IA_CSS_ISYS_FRAME_FORMAT_RAW, CSS2600_ISYS_MIPI_CSI2_TYPE_RAW12 },
	{ V4L2_PIX_FMT_SBGGR10, 16, 10, V4L2_MBUS_FMT_SBGGR10_1X10, IA_CSS_ISYS_FRAME_FORMAT_RAW, CSS2600_ISYS_MIPI_CSI2_TYPE_RAW10 },
	{ V4L2_PIX_FMT_SGBRG10, 16, 10, V4L2_MBUS_FMT_SGBRG10_1X10, IA_CSS_ISYS_FRAME_FORMAT_RAW, CSS2600_ISYS_MIPI_CSI2_TYPE_RAW10 },
	{ V4L2_PIX_FMT_SGRBG10, 16, 10, V4L2_MBUS_FMT_SGRBG10_1X10, IA_CSS_ISYS_FRAME_FORMAT_RAW, CSS2600_ISYS_MIPI_CSI2_TYPE_RAW10 },
	{ V4L2_PIX_FMT_SRGGB10, 16, 10, V4L2_MBUS_FMT_SRGGB10_1X10, IA_CSS_ISYS_FRAME_FORMAT_RAW, CSS2600_ISYS_MIPI_CSI2_TYPE_RAW10 },
	{ V4L2_PIX_FMT_SBGGR8, 8, 8, V4L2_MBUS_FMT_SBGGR8_1X8, IA_CSS_ISYS_FRAME_FORMAT_RAW, CSS2600_ISYS_MIPI_CSI2_TYPE_RAW8 },
	{ V4L2_PIX_FMT_SGBRG8, 8, 8, V4L2_MBUS_FMT_SGBRG8_1X8, IA_CSS_ISYS_FRAME_FORMAT_RAW, CSS2600_ISYS_MIPI_CSI2_TYPE_RAW8 },
	{ V4L2_PIX_FMT_SGRBG8, 8, 8, V4L2_MBUS_FMT_SGRBG8_1X8, IA_CSS_ISYS_FRAME_FORMAT_RAW, CSS2600_ISYS_MIPI_CSI2_TYPE_RAW8 },
	{ V4L2_PIX_FMT_SRGGB8, 8, 8, V4L2_MBUS_FMT_SRGGB8_1X8, IA_CSS_ISYS_FRAME_FORMAT_RAW, CSS2600_ISYS_MIPI_CSI2_TYPE_RAW8 },
	/* MIPI CSI-2 MUST BE LAST */
	{ V4L2_PIX_FMT_MIPI_CSI2, -1, -1, -1, IA_CSS_ISYS_FRAME_FORMAT_RAW, CSS2600_ISYS_MIPI_CSI2_TYPE_RAW8 },
	{ }
};

#define ISYS_PFMT_MIPI_CSI2	(ARRAY_SIZE(isys_pfmts) - 2)

/**
 * Returns true if device does not support real interrupts and
 * polling must be used.
 */
static int css2600_poll_for_events(struct css2600_isys_video *av)
{
	return av->isys->pdata->type == CSS2600_ISYS_TYPE_CSS2600_FPGA ||
		av->isys->pdata->type == CSS2600_ISYS_TYPE_CSS2600;
}

static int video_open(struct file *file)
{
	struct css2600_isys_video *av = video_drvdata(file);
	int rval;

	rval = v4l2_fh_open(file);
	if (rval)
		return rval;

	rval = css2600_pipeline_pm_use(&av->vdev.entity, 1);
	if (rval)
		goto out_v4l2_fh_release;

	if (!av->isys->video_opened++ && css2600_poll_for_events(av)) {
		static const struct sched_param param = {
			.sched_priority = MAX_USER_RT_PRIO/2,
		};

		av->isys->isr_thread = kthread_run(css2600_isys_isr_run,
						   av->isys, CSS2600_NAME);

		if (IS_ERR(av->isys->isr_thread)) {
			rval = PTR_ERR(av->isys->isr_thread);
			goto out_css2600_pipeline_pm_use;
		}

		sched_setscheduler(av->isys->isr_thread, SCHED_FIFO, &param);
	}

	return 0;

out_css2600_pipeline_pm_use:
	css2600_pipeline_pm_use(&av->vdev.entity, 0);

out_v4l2_fh_release:
	v4l2_fh_release(file);

	return rval;
}

static int video_release(struct file *file)
{
	struct css2600_isys_video *av = video_drvdata(file);

	vb2_fop_release(file);

	if (!--av->isys->video_opened && css2600_poll_for_events(av))
		kthread_stop(av->isys->isr_thread);

	css2600_pipeline_pm_use(&av->vdev.entity, 0);

	return 0;
}

const struct css2600_isys_pixelformat *css2600_isys_get_pixelformat(
	struct css2600_isys_video *av, uint32_t pixelformat)
{
	struct media_pad *pad = av->vdev.entity.links[0].source;
	const uint32_t *supported_codes =
		to_css2600_isys_subdev(
			media_entity_to_v4l2_subdev(pad->entity))
		->supported_codes[pad->index];
	const struct css2600_isys_pixelformat *pfmt;

	if (av->mipi_csi2_fmt)
		return &isys_pfmts[ISYS_PFMT_MIPI_CSI2];

	for (pfmt = isys_pfmts; pfmt->bpp; pfmt++) {
		unsigned int i;

		if (pfmt->pixelformat != pixelformat)
			continue;

		for (i = 0; supported_codes[i]; i++) {
			if (pfmt->code == supported_codes[i])
				return pfmt;
		}
	}

	/* Not found. Get the default, i.e. the first defined one. */
	for (pfmt = isys_pfmts; pfmt->bpp; pfmt++) {
		if (pfmt->code == *supported_codes)
			return pfmt;
	}

	BUG();
}

static int vidioc_querycap(struct file *file, void *fh,
			   struct v4l2_capability *cap)
{
	struct css2600_isys_video *av = video_drvdata(file);

	strlcpy(cap->driver, CSS2600_ISYS_NAME, sizeof(cap->driver));
	strlcpy(cap->card, av->isys->media_dev.model, sizeof(cap->card));
	strlcpy(cap->bus_info, av->isys->media_dev.bus_info,
		sizeof(cap->bus_info));
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	cap->device_caps = cap->capabilities;

	return 0;
}

static int vidioc_g_fmt_vid_cap(struct file *file, void *fh,
				struct v4l2_format *fmt)
{
	struct css2600_isys_video *av = video_drvdata(file);

	mutex_lock(&av->mutex);
	fmt->fmt.pix = av->pix;
	mutex_unlock(&av->mutex);

	return 0;
}

const struct css2600_isys_pixelformat *css2600_isys_video_try_fmt_vid_cap(
	struct css2600_isys_video *av, struct v4l2_pix_format *pix)
{
	const struct css2600_isys_pixelformat *pfmt =
		css2600_isys_get_pixelformat(av, pix->pixelformat);
	struct media_entity *entity;
	struct media_pad *pad;
	struct v4l2_subdev *sd;
	struct v4l2_mbus_frame_desc desc = { 0 };
	unsigned int i;
	int rval;

	pix->pixelformat = pfmt->pixelformat;

	if (pfmt->pixelformat != V4L2_PIX_FMT_MIPI_CSI2) {
		pix->bytesperline = ALIGN(
			pix->width * DIV_ROUND_UP(pfmt->bpp, 8),
			av->isys->line_align);
		pix->sizeimage = max(pix->sizeimage,
				     pix->bytesperline * pix->height);
		return pfmt;
	}

	/* MIPI CSI-2 has no meaningful bytesperline. */
	pix->bytesperline = 0;
	pix->sizeimage = 0;

	entity = css2600_isys_video_find_facing_external(av);
	if (!entity) {
		dev_dbg(&av->isys->adev->dev,
			"can't find entity facing external\n");
		goto out_zero;
	}

	pad = media_entity_remote_pad(entity->pads);
	if (!pad) {
		dev_dbg(&av->isys->adev->dev, "can't find external pad\n");
		goto out_zero;
	}

	sd = media_entity_to_v4l2_subdev(pad->entity);
	rval = v4l2_subdev_call(sd, pad, get_frame_desc, pad->index, &desc);
	if (rval) {
		dev_dbg(&av->isys->adev->dev,
			"can't get framedescriptor (%d)\n", rval);
		goto out_zero;
	}

	for (i = 0; i < desc.num_entries; i++) {
		struct v4l2_mbus_frame_desc_entry *entry = &desc.entry[i];
		unsigned int line_length;

		if (entry->flags & V4L2_MBUS_FRAME_DESC_FL_BLOB) {
			dev_dbg(&av->isys->adev->dev,
				"1-dimension DMA not supported for raw csi-2\n");
			goto out_zero;
		}

		/* line start / end codes */
		line_length = DIV_ROUND_UP(
			2 * 32
			+ (unsigned int)entry->size.two_dim.width * entry->bpp,
			8);

		pix->sizeimage += roundup(line_length, av->isys->line_align)
			* entry->size.two_dim.height;
	}

out_zero:
	return pfmt;
}

static int vidioc_s_fmt_vid_cap(struct file *file, void *fh,
				struct v4l2_format *fmt)
{
	struct css2600_isys_video *av = video_drvdata(file);
	int rval = 0;

	mutex_lock(&av->mutex);

	if (av->streaming) {
		rval = -EBUSY;
		goto out;
	}

	av->pfmt = css2600_isys_video_try_fmt_vid_cap(av, &fmt->fmt.pix);
	av->pix = fmt->fmt.pix;

out:
	mutex_unlock(&av->mutex);

	return rval;
}

static int vidioc_try_fmt_vid_cap(struct file *file, void *fh,
				  struct v4l2_format *fmt)
{
	struct css2600_isys_video *av = video_drvdata(file);

	css2600_isys_video_try_fmt_vid_cap(av, &fmt->fmt.pix);

	return 0;
}

/*
 * Return true if an entity directly connected to an Iunit entity is
 * an image source for the ISP. This can be any external directly
 * connected entity or any of the test pattern generators in the
 * Iunit.
 */
static bool is_external(struct css2600_isys_video *av,
			struct media_entity *entity)
{
	struct v4l2_subdev *sd;
	unsigned int i;

	/* All video nodes are ours. */
	if (media_entity_type(entity) != MEDIA_ENT_T_V4L2_SUBDEV)
		return false;

	sd = media_entity_to_v4l2_subdev(entity);

	if (sd->owner != THIS_MODULE)
		return true;

	for (i = 0; i < CSS2600_ISYS_MAX_TPGS && av->isys->tpg[i].isys; i++)
		if (entity == &av->isys->tpg[i].asd.sd.entity)
			return true;

	return false;
}

static int link_validate(struct media_link *link)
{
	struct v4l2_subdev_format fmt = { 0 };
	struct v4l2_subdev *sd =
		media_entity_to_v4l2_subdev(link->source->entity);
	struct css2600_isys_video *av =
		container_of(link->sink, struct css2600_isys_video, pad);
	struct css2600_isys_pipeline *ip =
		to_css2600_isys_pipeline(av->vdev.entity.pipe);
	int rval;

	fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	fmt.pad = link->source->index;
	rval = v4l2_subdev_call(sd, pad, get_fmt, NULL, &fmt);
	if (rval)
		return rval;

	if (fmt.format.width != av->pix.width
	    || fmt.format.height != av->pix.height)
		return -EINVAL;

	if (av->pfmt->pixelformat != V4L2_PIX_FMT_MIPI_CSI2
	    && fmt.format.code != av->pfmt->code)
		return -EINVAL;

	if (is_external(av, link->source->entity)) {
		ip->external = media_entity_remote_pad(av->vdev.entity.pads);
		ip->source = to_css2600_isys_subdev(
			media_entity_to_v4l2_subdev(
				link->source->entity))->source;
	}

	return 0;
}

/* Create stream and start it using the CSS library API. */
static int start_stream_firmware(struct css2600_isys_video *av,
				 struct css2600_isys_buffer *ib)
{
	struct css2600_isys_pipeline *ip =
		to_css2600_isys_pipeline(av->vdev.entity.pipe);
	struct ia_css_isys_stream_cfg_data stream_cfg = {
		.isl_use = ip->isl_enabled,
		.vc = 0,
		.nof_input_pins = 1,
		.input_pins = {
			{
				.dt = av->pfmt->mipi_data_type,
				.input_res = {
					.width = av->pix.width,
					.height = av->pix.height,
				},
				.crop = {
					 .top_offset = 0,
					 .left_offset = 0,
					 .right_offset = av->pix.width,
					 .bottom_offset = av->pix.height,
				},
			},
		},
		.nof_output_pins = 1,
		.output_pins = {
			{
				.pt = av->aq.css_pin_type,
				.type_specifics.ft =
				css2600_isys_video_type_specifics(
					av->aq.css_pin_type, av->pfmt),
				.output_res = {
					.width = av->pix.width,
					.height = av->pix.height,
				},
				.send_irq = 1,
			},
		},
		.src = 0,
	};
	struct ia_css_isys_frame_buff_set buf = {
		.output_pins = {
			{
				.info = {
					.type_specifics.ft =
					css2600_isys_video_type_specifics(
						av->aq.css_pin_type, av->pfmt),
					.pt = av->aq.css_pin_type,
					.output_res = {
						.width = av->pix.width,
						.height = av->pix.height,
					},
				},
			},
		},
		.input_pin_set = {
			.blc_param.addr = 1,
			.lsc_param.addr = 1,
			.dpc_param.addr = 1,
		},
		.send_irq_sof = 1,
		.send_irq_eof = 1,
	};
	struct ia_css_isys_frame_buff_set *__buf = NULL;
	int rval;

	if (ib) {
		struct vb2_buffer *vb = css2600_isys_buffer_to_vb2_buffer(ib);

		buf.output_pins[0].payload.addr =
			*(dma_addr_t *)vb2_plane_cookie(vb, 0);
		buf.output_pins[0].payload.out_buf_id = vb->v4l2_buf.index + 1;
		__buf = &buf;
	}

	reinit_completion(&ip->stream_open_completion);
	rval = css2600_lib_call(stream_open, av->isys, ip->source, &stream_cfg);
	if (rval < 0) {
		dev_dbg(&av->isys->adev->dev, "can't open stream (%d)\n",
			rval);
		return rval;;
	}

	wait_for_completion(&ip->stream_open_completion);
	dev_dbg(&av->isys->adev->dev, "stream open complete\n");

	reinit_completion(&ip->stream_start_completion);
	rval = css2600_lib_call(stream_start, av->isys, ip->source, NULL);
	if (rval < 0) {
		dev_dbg(&av->isys->adev->dev, "can't start streaning (%d)\n",
			rval);
		goto out_stream_close;
	}

	wait_for_completion(&ip->stream_start_completion);
	dev_dbg(&av->isys->adev->dev, "stream start complete\n");

	reinit_completion(&ip->capture_ack_completion);
	rval = css2600_lib_call(stream_capture_indication, av->isys,
				ip->source, &buf);
	if (rval < 0) {
		dev_dbg(&av->isys->adev->dev,
			"capture indication failed (%d)\n", rval);
		goto out_stream_stop;
	}

	/* There are no capture acks on 2401 */
	if (av->isys->pdata->type == CSS2600_ISYS_TYPE_CSS2600_FPGA ||
	    av->isys->pdata->type == CSS2600_ISYS_TYPE_CSS2600) {
		wait_for_completion(&ip->capture_ack_completion);
		dev_dbg(&av->isys->adev->dev, "capture ack complete\n");
	}

	return 0;

out_stream_stop:
	reinit_completion(&ip->stream_stop_completion);
	rval = css2600_lib_call(stream_stop, av->isys, ip->source);
	if (rval < 0) {
		dev_dbg(&av->isys->adev->dev, "can't stop stream (%d)\n",
			rval);
	} else {
		wait_for_completion(&ip->stream_stop_completion);
		dev_dbg(&av->isys->adev->dev, "stream stop complete\n");
	}

out_stream_close:
	reinit_completion(&ip->stream_close_completion);
	rval = css2600_lib_call(stream_close, av->isys, ip->source);
	if (rval < 0) {
		dev_dbg(&av->isys->adev->dev, "can't close stream (%d)\n",
			rval);
	} else {
		wait_for_completion(&ip->stream_close_completion);
		dev_dbg(&av->isys->adev->dev, "stream close complete\n");
	}
	return rval;
}

static void stop_streaming_firmware(struct css2600_isys_video *av)
{
	struct css2600_isys_pipeline *ip =
		to_css2600_isys_pipeline(av->vdev.entity.pipe);
	int rval;

	reinit_completion(&ip->stream_stop_completion);
	rval = css2600_lib_call(stream_stop, av->isys, ip->source);
	if (rval < 0) {
		dev_dbg(&av->isys->adev->dev, "can't stop stream (%d)\n",
			rval);
	} else {
		wait_for_completion(&ip->stream_stop_completion);
		dev_dbg(&av->isys->adev->dev, "stream stop complete\n");
	}

	reinit_completion(&ip->stream_close_completion);
	rval = css2600_lib_call(stream_close, av->isys, ip->source);
	if (rval < 0) {
		dev_dbg(&av->isys->adev->dev, "can't close stream (%d)\n",
			rval);
	} else {
		wait_for_completion(&ip->stream_close_completion);
		dev_dbg(&av->isys->adev->dev, "stream close complete\n");
	}
}

struct media_entity *
css2600_isys_video_find_facing_external(struct css2600_isys_video *av)
{
	struct media_entity_graph graph;
	struct media_entity *entity = &av->vdev.entity, *prev_entity;
	struct media_device *mdev = entity->parent;

	mutex_lock(&mdev->graph_mutex);

	media_entity_graph_walk_start(&graph, &av->vdev.entity);

	while ((prev_entity = entity) &&
	       (entity = media_entity_graph_walk_next(&graph))) {
		dev_dbg(&av->isys->adev->dev, "entity %s %d, prev %s %d\n",
			entity->name, is_external(av, entity),
			prev_entity->name, is_external(av, prev_entity));

		if (!is_external(av, prev_entity) || is_external(av, entity))
			continue;

		mutex_unlock(&mdev->graph_mutex);

		return entity;
	}

	mutex_unlock(&mdev->graph_mutex);

	return NULL;
}

int css2600_isys_video_set_streaming(struct css2600_isys_video *av,
				     unsigned int state,
				     struct css2600_isys_buffer *ib)
{
	struct media_device *mdev = av->vdev.entity.parent;
	struct media_entity_graph graph;
	struct media_entity *entity, *entity2;
	struct css2600_isys_pipeline *ip;
	unsigned long flags;
	unsigned int entities = 0;
	unsigned int i;
	int rval = 0;

	mutex_lock(&av->mutex);

	if (state == av->streaming)
		goto out_unlock;

	if (state) {
		av->ip.external = NULL;
		atomic_set(&av->ip.sequence, 0);
		av->ip.isl_enabled = 0;

		rval = media_entity_pipeline_start(&av->vdev.entity,
						   &av->ip.pipe);
		if (rval < 0)
			goto out_unlock;

		ip = to_css2600_isys_pipeline(av->vdev.entity.pipe);
		if (!ip->external) {
			dev_err(&av->isys->adev->dev,
				"no external entity set! Driver bug?\n");
			rval = -EINVAL;
			goto out_media_entity_pipeline_stop;
		}
		dev_dbg(&av->isys->adev->dev, "external entity %s\n",
			ip->external->entity->name);

		ip->continuous = true;

		/*
		 * Any stream from the 2401 test pattern generators
		 * requires kicking them again to receive the next
		 * frame. Store the information whether this is
		 * necessary to the pipeline.
		 */
		for (i = 0; i < CSS2600_ISYS_MAX_TPGS &&
			     av->isys->tpg[i].isys &&
			     av->isys->pdata->type == CSS2600_ISYS_TYPE_CSS2401;
		     i++) {
			if (av->isys->tpg[i].asd.sd.entity.pipe != &ip->pipe)
				continue;

			ip->continuous = false;
			break;
		}

		dev_dbg(&av->isys->adev->dev, "continuous %d\n",
			ip->continuous);
	} else {
		ip = to_css2600_isys_pipeline(av->vdev.entity.pipe);
	}

	mutex_lock(&mdev->graph_mutex);

	media_entity_graph_walk_start(&graph, &av->vdev.entity);

	while ((entity = media_entity_graph_walk_next(&graph))) {
		struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);

		dev_dbg(&av->isys->adev->dev, "entity %s\n", entity->name);

		/* We don't support non-linear pipelines yet. */
		if (media_entity_type(entity) != MEDIA_ENT_T_V4L2_SUBDEV)
			continue;

		/*
		 * Don't start truly external devices quite yet. Test
		 * pattern generators can be started on 2401 ONLY.
		 */
		if (media_entity_to_v4l2_subdev(entity)->owner != THIS_MODULE
		    || (av->isys->pdata->type != CSS2600_ISYS_TYPE_CSS2401 &&
			ip->external->entity == entity))
			continue;

		dev_dbg(&av->isys->adev->dev, "s_stream %s\n",
			entity->name);
		rval = v4l2_subdev_call(sd, video, s_stream, state);
		if (!state)
			continue;
		if (rval && rval != -ENOIOCTLCMD) {
			mutex_unlock(&mdev->graph_mutex);
			goto out_media_entity_stop_streaming;
		}

		if (entity->id >= sizeof(entities) << 3) {
			mutex_unlock(&mdev->graph_mutex);
			WARN_ON(1);
			goto out_media_entity_stop_streaming;
		}

		entities |= 1 << entity->id;
	}

	mutex_unlock(&mdev->graph_mutex);

	spin_lock_irqsave(&av->isys->lock, flags);
	av->isys->pipes[ip->source] = ip;
	spin_unlock_irqrestore(&av->isys->lock, flags);

	/* Oh crap */
	if (state) {
		dev_dbg(&av->isys->adev->dev, "source %d\n", ip->source);
		rval = start_stream_firmware(av, ib);
		if (rval)
			goto out_media_entity_stop_streaming;
	} else {
		stop_streaming_firmware(av);
	}

	/*
	 * If the "external" entity was not a test pattern generator,
	 * start it now.
	 */
	if (media_entity_to_v4l2_subdev(ip->external->entity)->owner
	    != THIS_MODULE ||
	    av->isys->pdata->type != CSS2600_ISYS_TYPE_CSS2401) {
		dev_dbg(&av->isys->adev->dev, "s_stream %s (ext)\n",
			ip->external->entity->name);
		rval = v4l2_subdev_call(
			media_entity_to_v4l2_subdev(ip->external->entity),
			video, s_stream, state);
		if (rval && state)
			goto out_media_entity_stop_streaming_firmware;
	}

	if (!state)
		media_entity_pipeline_stop(&av->vdev.entity);

	av->streaming = state;

	mutex_unlock(&av->mutex);
	return 0;

out_media_entity_stop_streaming_firmware:
	stop_streaming_firmware(av);

out_media_entity_stop_streaming:
	mutex_lock(&mdev->graph_mutex);

	media_entity_graph_walk_start(&graph, &av->vdev.entity);

	while (state && (entity2 = media_entity_graph_walk_next(&graph))
		&& entity2 != entity) {
		struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity2);

		if (!(entity2->id << 1 & entities))
			continue;

		v4l2_subdev_call(sd, video, s_stream, 0);
	}

	mutex_unlock(&mdev->graph_mutex);

out_media_entity_pipeline_stop:
	if (state)
		media_entity_pipeline_stop(&av->vdev.entity);

out_unlock:
	mutex_unlock(&av->mutex);

	return rval;
}

static const struct v4l2_ioctl_ops ioctl_ops = {
	.vidioc_querycap = vidioc_querycap,
	.vidioc_g_fmt_vid_cap = vidioc_g_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap = vidioc_s_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap = vidioc_try_fmt_vid_cap,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
};

static const struct media_entity_operations entity_ops = {
	.link_validate = link_validate,
};

static const struct v4l2_file_operations isys_fops = {
	.owner = THIS_MODULE,
	.poll = vb2_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = vb2_fop_mmap,
	.open = video_open,
	.release = video_release,
};

/*
 * Do everything that's needed to initialise things related to video
 * buffer queue, video node, and the related media entity. The caller
 * is expected to assign isys field and set the name of the video
 * device.
 */
int css2600_isys_video_init(struct css2600_isys_video *av,
			    struct media_entity *source,
			    unsigned int source_pad,
			    unsigned int flags)
{
	int rval;

	mutex_init(&av->mutex);
	init_completion(&av->ip.stream_open_completion);
	init_completion(&av->ip.stream_close_completion);
	init_completion(&av->ip.stream_start_completion);
	init_completion(&av->ip.stream_stop_completion);
	init_completion(&av->ip.capture_ack_completion);

	rval = css2600_isys_queue_init(&av->aq);
	if (rval)
		goto out_mutex_destroy;

	av->pad.flags = MEDIA_PAD_FL_SINK;
	rval = media_entity_init(&av->vdev.entity, 1, &av->pad, 0);
	if (rval)
		goto out_css2600_isys_queue_cleanup;

	av->vdev.entity.ops = &entity_ops;
	av->vdev.release = video_device_release_empty;
	av->vdev.fops = &isys_fops;
	av->vdev.v4l2_dev = &av->isys->v4l2_dev;
	av->vdev.ioctl_ops = &ioctl_ops;
	av->vdev.queue = &av->aq.vbq;
	set_bit(V4L2_FL_USES_V4L2_FH, &av->vdev.flags);
	video_set_drvdata(&av->vdev, av);

	rval = media_entity_create_link(
		source, source_pad, &av->vdev.entity, 0, flags);
	if (rval) {
		dev_info(&av->isys->adev->dev, "can't create link\n");
		goto out_media_entity_cleanup;
	}

	if (av->mipi_csi2_fmt)
		av->pfmt = &isys_pfmts[ISYS_PFMT_MIPI_CSI2];
	else
		av->pfmt = css2600_isys_video_try_fmt_vid_cap(av, &av->pix);

	rval = video_register_device(&av->vdev, VFL_TYPE_GRABBER, -1);
	if (rval)
		goto out_media_entity_cleanup;

	return rval;

out_media_entity_cleanup:
	media_entity_cleanup(&av->vdev.entity);

out_css2600_isys_queue_cleanup:
	css2600_isys_queue_cleanup(&av->aq);

out_mutex_destroy:
	mutex_destroy(&av->mutex);

	return rval;
}

void css2600_isys_video_cleanup(struct css2600_isys_video *av)
{
	video_unregister_device(&av->vdev);
	media_entity_cleanup(&av->vdev.entity);
	mutex_destroy(&av->mutex);
}
