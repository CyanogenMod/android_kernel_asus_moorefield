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

#include <linux/completion.h>
#include <linux/device.h>
#include <linux/dma-attrs.h>
#include <linux/iommu.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/sizes.h>
#include <linux/string.h>

#include <media/media-entity.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>

#include "css2600.h"
#include "css2600-bus.h"
#include "css2600-isys.h"
#include "css2600-isys-csi2.h"
#include "css2600-isys-video.h"

static int queue_setup(struct vb2_queue *q, const struct v4l2_format *fmt,
		       unsigned int *num_buffers, unsigned int *num_planes,
		       unsigned int sizes[], void *alloc_ctxs[])
{
	struct css2600_isys_queue *aq = vb2_queue_to_css2600_isys_queue(q);
	struct css2600_isys_video *av = css2600_isys_queue_to_video(aq);
	const struct css2600_isys_pixelformat *pfmt;
	struct v4l2_pix_format pix;

	if (fmt)
		pix = fmt->fmt.pix;
	else
		pix = av->pix;

	pfmt = css2600_isys_video_try_fmt_vid_cap(av, &pix);

	*num_planes = 1;

	sizes[0] = pix.sizeimage;
	alloc_ctxs[0] = aq->ctx;

	dev_dbg(&av->isys->adev->dev, "queue setup: buffer size %d\n",
		sizes[0]);

	return 0;
}

void css2600_isys_queue_lock(struct vb2_queue *q)
{
	struct css2600_isys_queue *aq = vb2_queue_to_css2600_isys_queue(q);
	struct css2600_isys_video *av = css2600_isys_queue_to_video(aq);

	dev_dbg(&av->isys->adev->dev, "queue lock\n");
	mutex_lock(&aq->mutex);
}

void css2600_isys_queue_unlock(struct vb2_queue *q)
{
	struct css2600_isys_queue *aq = vb2_queue_to_css2600_isys_queue(q);
	struct css2600_isys_video *av = css2600_isys_queue_to_video(aq);

	dev_dbg(&av->isys->adev->dev, "queue unlock\n");
	mutex_unlock(&aq->mutex);
}

static int buf_init(struct vb2_buffer *vb)
{
	struct css2600_isys_queue *aq =
		vb2_queue_to_css2600_isys_queue(vb->vb2_queue);
	struct css2600_isys_video *av = css2600_isys_queue_to_video(aq);

	dev_dbg(&av->isys->adev->dev, "buf_init\n");
	return 0;
}

static int buf_prepare(struct vb2_buffer *vb)
{
	struct css2600_isys_queue *aq =
		vb2_queue_to_css2600_isys_queue(vb->vb2_queue);
	struct css2600_isys_video *av = css2600_isys_queue_to_video(aq);

	dev_dbg(&av->isys->adev->dev, "configured size %u, buffer size %lu\n",
		av->pix.sizeimage, vb2_plane_size(vb, 0));

	if (av->pix.sizeimage > vb2_plane_size(vb, 0))
		return -EINVAL;

	vb2_set_plane_payload(vb, 0, av->pix.sizeimage);

	return 0;
}

static int buf_finish(struct vb2_buffer *vb)
{
	struct css2600_isys_queue *aq =
		vb2_queue_to_css2600_isys_queue(vb->vb2_queue);
	struct css2600_isys_video *av = css2600_isys_queue_to_video(aq);

	dev_dbg(&av->isys->adev->dev, "buf_finish %u\n", vb->v4l2_buf.index);
	return 0;
}

static void buf_cleanup(struct vb2_buffer *vb)
{
	struct css2600_isys_queue *aq =
		vb2_queue_to_css2600_isys_queue(vb->vb2_queue);
	struct css2600_isys_video *av = css2600_isys_queue_to_video(aq);

	dev_dbg(&av->isys->adev->dev, "buf_cleanup\n");
}

static void __buf_queue(struct vb2_buffer *vb, bool force)
{
	struct css2600_isys_queue *aq =
		vb2_queue_to_css2600_isys_queue(vb->vb2_queue);
	struct css2600_isys_video *av = css2600_isys_queue_to_video(aq);
	struct css2600_isys_buffer *ib = to_css2600_isys_buffer(vb);
	struct ia_css_isys_frame_buff_set buf = {
		.output_pins = {
			{
				.payload = {
					.addr =
					*(dma_addr_t *)vb2_plane_cookie(vb, 0),
					/* + 1 due to library issues */
					.out_buf_id = vb->v4l2_buf.index + 1,
				},
				.info = {
					.type_specifics.ft =
					css2600_isys_video_type_specifics(
						av->aq.css_pin_type, av->pfmt),
					.pt = aq->css_pin_type,
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
	unsigned long flags;
	int rval;

	dev_dbg(&av->isys->adev->dev, "buf_queue %d/%p\n", vb->v4l2_buf.index,
		ib);
	dev_dbg(&av->isys->adev->dev, "iova %lu\n",
		(unsigned long)*(dma_addr_t *)vb2_plane_cookie(vb, 0));

	if (!vb->vb2_queue->streaming && !force) {
		dev_dbg(&av->isys->adev->dev,
			"not streaming yet, adding to incoming\n");
		spin_lock_irqsave(&aq->lock, flags);
		list_add(&ib->head, &aq->incoming);
		spin_unlock_irqrestore(&aq->lock, flags);
		return;
	}

	spin_lock_irqsave(&aq->lock, flags);
	list_add(&ib->head, &aq->active);
	spin_unlock_irqrestore(&aq->lock, flags);

	rval = css2600_lib_call(
		stream_capture_indication, av->isys,
		to_css2600_isys_pipeline(av->vdev.entity.pipe)->source, &buf);
	if (rval < 0) {
		dev_dbg(&av->isys->adev->dev,
			"capture indication failed (%d)\n", rval);
		spin_lock_irqsave(&aq->lock, flags);
		list_del(&ib->head);
		spin_unlock_irqrestore(&aq->lock, flags);
		return;
	}
	dev_dbg(&av->isys->adev->dev, "queued buffer\n");
}

static void queue_stream_one(struct css2600_isys_queue *aq)
{
	struct css2600_isys_video *av = css2600_isys_queue_to_video(aq);
	struct v4l2_subdev *sd =
		media_entity_to_v4l2_subdev(
			to_css2600_isys_pipeline(av->vdev.entity.pipe)->
			external->entity);
	struct css2600_isys_buffer *ib = NULL;
	unsigned long flags;
	int rval;

	spin_lock_irqsave(&aq->lock, flags);
	dev_dbg(&av->isys->adev->dev, "active empty %d, incoming empty %d\n",
		list_empty(&aq->active), list_empty(&aq->incoming));
	if (list_empty(&aq->active) && !list_empty(&aq->incoming)) {
		ib = list_last_entry(&aq->incoming,
				     struct css2600_isys_buffer, head);
		list_del(&ib->head);
	}
	spin_unlock_irqrestore(&aq->lock, flags);
	dev_dbg(&av->isys->adev->dev, "queue_stream_one buffer %p\n", ib);

	if (!ib)
		return;

	rval = v4l2_subdev_call(sd, video, s_stream, 1);
	if (rval) {
		dev_err(&av->isys->adev->dev, "s_stream failed!\n");
		goto fail_requeue;
	}

	__buf_queue(css2600_isys_buffer_to_vb2_buffer(ib), false);

	return;

fail_requeue:
	spin_lock_irqsave(&aq->lock, flags);
	list_add_tail(&ib->head, &aq->incoming);
	spin_unlock_irqrestore(&aq->lock, flags);
}

static void queue_stream_one_work(struct work_struct *work)
{
	struct css2600_isys_queue *aq =
		container_of(work, struct css2600_isys_queue, work);

	queue_stream_one(aq);
}

static void buf_queue(struct vb2_buffer *vb)
{
	struct css2600_isys_queue *aq =
		vb2_queue_to_css2600_isys_queue(vb->vb2_queue);
	struct css2600_isys_video *av = css2600_isys_queue_to_video(aq);
	struct css2600_isys_buffer *ib = to_css2600_isys_buffer(vb);
	unsigned long flags;

	dev_dbg(&av->isys->adev->dev, "buf_queue %p\n", ib);
	if (!av->streaming ||
	    to_css2600_isys_pipeline(av->vdev.entity.pipe)->continuous) {
		__buf_queue(vb, false);
	} else {
		/* Put to the incoming queue and try to stream. */
		spin_lock_irqsave(&aq->lock, flags);
		list_add(&ib->head, &aq->incoming);
		spin_unlock_irqrestore(&aq->lock, flags);

		if (vb->vb2_queue->streaming)
			queue_stream_one(aq);
	}
}

static int start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct css2600_isys_queue *aq = vb2_queue_to_css2600_isys_queue(q);
	struct css2600_isys_video *av = css2600_isys_queue_to_video(aq);
	struct css2600_isys_pipeline *ip;
	struct css2600_isys_buffer *ib = NULL;
	struct vb2_buffer *vb;
	unsigned long flags;
	int rval;

	dev_dbg(&av->isys->adev->dev, "input mipi data type %u\n",
		av->pfmt->mipi_data_type);
	dev_dbg(&av->isys->adev->dev, "width %u, height %u\n",
		av->pix.width, av->pix.height);
	dev_dbg(&av->isys->adev->dev, "css pixelformat %u\n",
		av->pfmt->css_pixelformat);

	spin_lock_irqsave(&aq->lock, flags);
	if (!list_empty(&aq->incoming)) {
		ib = list_last_entry(&aq->incoming,
				     struct css2600_isys_buffer, head);
		vb = css2600_isys_buffer_to_vb2_buffer(ib);

		list_del(&ib->head);
		list_add(&ib->head, &aq->active);
		dev_dbg(&av->isys->adev->dev,
			"pre-queueing buffer %u/%p from incoming\n",
			vb->v4l2_buf.index, ib);
	}
	spin_unlock_irqrestore(&aq->lock, flags);

	rval = css2600_isys_video_set_streaming(av, 1, ib);
	if (rval)
		goto out_pipe_null;

	ip = to_css2600_isys_pipeline(av->vdev.entity.pipe);

	spin_lock_irqsave(&aq->lock, flags);
	while (!list_empty(&aq->incoming)) {
		ib = list_last_entry(&aq->incoming,
				     struct css2600_isys_buffer, head);
		vb = css2600_isys_buffer_to_vb2_buffer(ib);

		list_del(&ib->head);

		spin_unlock_irqrestore(&aq->lock, flags);

		dev_dbg(&av->isys->adev->dev,
			"queueing buffer %u/%p from active\n",
			vb->v4l2_buf.index, ib);
		__buf_queue(vb, true);

		spin_lock_irqsave(&aq->lock, flags);
		if (!ip->continuous)
			break;
	}
	spin_unlock_irqrestore(&aq->lock, flags);

	return 0;

out_pipe_null:
	spin_lock_irqsave(&aq->lock, flags);
	if (ib) {
		list_del(&ib->head);
		list_add(&ib->head, &aq->incoming);
	}
	spin_unlock_irqrestore(&aq->lock, flags);

	css2600_isys_video_set_streaming(av, 0, NULL);
	spin_lock_irqsave(&av->isys->lock, flags);
	av->isys->pipes[to_css2600_isys_pipeline(
				av->vdev.entity.pipe)->source] = NULL;
	spin_unlock_irqrestore(&av->isys->lock, flags);

	spin_lock_irqsave(&aq->lock, flags);
	while (!list_empty(&aq->incoming)) {
		struct css2600_isys_buffer *ib =
			list_last_entry(&aq->incoming,
					struct css2600_isys_buffer, head);
		struct vb2_buffer *vb = css2600_isys_buffer_to_vb2_buffer(ib);

		dev_dbg(&av->isys->adev->dev,
			"removing buffer %u/%p from incoming\n",
			vb->v4l2_buf.index, ib);
		list_del(&ib->head);
		vb2_buffer_done(vb, VB2_BUF_STATE_QUEUED);
	}
	spin_unlock_irqrestore(&aq->lock, flags);

	return rval;
}

static int stop_streaming(struct vb2_queue *q)
{
	struct css2600_isys_queue *aq = vb2_queue_to_css2600_isys_queue(q);
	struct css2600_isys_video *av = css2600_isys_queue_to_video(aq);
	int source = to_css2600_isys_pipeline(av->vdev.entity.pipe)->source;
	unsigned long flags;

	flush_workqueue(aq->wq);

	css2600_isys_video_set_streaming(av, 0, NULL);

	spin_lock_irqsave(&aq->lock, flags);
	while (!list_empty(&aq->incoming)) {
		struct css2600_isys_buffer *ib =
			list_first_entry(&aq->incoming,
					 struct css2600_isys_buffer, head);
		struct vb2_buffer *vb = css2600_isys_buffer_to_vb2_buffer(ib);

		list_del(&ib->head);
		spin_unlock_irqrestore(&aq->lock, flags);

		vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);

		dev_dbg(&av->isys->adev->dev, "stop_streaming incoming %u/%p\n",
			vb->v4l2_buf.index, ib);

		spin_lock_irqsave(&aq->lock, flags);
	}
	while (!list_empty(&aq->active)) {
		struct css2600_isys_buffer *ib =
			list_first_entry(&aq->active,
					 struct css2600_isys_buffer, head);
		struct vb2_buffer *vb = css2600_isys_buffer_to_vb2_buffer(ib);

		list_del(&ib->head);
		spin_unlock_irqrestore(&aq->lock, flags);

		vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);

		dev_dbg(&av->isys->adev->dev, "stop_streaming active %u/%p\n",
			vb->v4l2_buf.index, ib);

		spin_lock_irqsave(&aq->lock, flags);
	}
	spin_unlock_irqrestore(&aq->lock, flags);

	spin_lock_irqsave(&av->isys->lock, flags);
	av->isys->pipes[source] = NULL;
	spin_unlock_irqrestore(&av->isys->lock, flags);

	return 0;
}

void css2600_isys_queue_buf_done(struct css2600_isys_pipeline *ip,
				 struct ia_css_isys_resp_info *info)
{
	struct css2600_isys_video *av =
		container_of(ip, struct css2600_isys_video, ip);
	struct css2600_isys_queue *aq = &av->aq;
	unsigned int sequence = atomic_read(&ip->sequence);
	struct css2600_isys_buffer *ib = NULL;
	struct vb2_buffer *vb;
	unsigned long flags;

	spin_lock_irqsave(&aq->lock, flags);
	if (!list_empty(&aq->active)) {
		ib = list_last_entry(&aq->active, struct css2600_isys_buffer, head);
		list_del(&ib->head);
	}
	dev_dbg(&av->isys->adev->dev, "dequeued buffer %p\n", ib);
	spin_unlock_irqrestore(&aq->lock, flags);

	if (WARN_ON_ONCE(!ib))
		return;

	/*
	 * We deliver fake frame end events on 2401. No events are
	 * delivered on frames that are not captured.
	 */
	if (av->isys->pdata->type == CSS2600_ISYS_TYPE_CSS2401) {
		struct media_pad *csi2_pad =
			media_entity_remote_pad(av->vdev.entity.pads);
		struct v4l2_event ev = { .type = V4L2_EVENT_FRAME_END,
					 .u.frame_sync.frame_sequence =
					 atomic_inc_return(&ip->sequence) };

		BUG_ON(!csi2_pad);

		v4l2_event_queue(media_entity_to_v4l2_subdev(
					 csi2_pad->entity)->devnode, &ev);
	}

	vb = css2600_isys_buffer_to_vb2_buffer(ib);

	v4l2_get_timestamp(&vb->v4l2_buf.timestamp);
	vb->v4l2_buf.sequence = sequence;

	vb2_buffer_done(vb, VB2_BUF_STATE_DONE);

	if (!ip->continuous)
		queue_work(aq->wq, &aq->work);
}

struct vb2_ops css2600_isys_queue_ops = {
	.queue_setup = queue_setup,
	.wait_prepare = css2600_isys_queue_unlock,
	.wait_finish = css2600_isys_queue_lock,
	.buf_init = buf_init,
	.buf_prepare = buf_prepare,
	.buf_finish = buf_finish,
	.buf_cleanup = buf_cleanup,
	.start_streaming = start_streaming,
	.stop_streaming = stop_streaming,
	.buf_queue = buf_queue,
};

int css2600_isys_queue_init(struct css2600_isys_queue *aq)
{
	struct css2600_isys *isys = css2600_isys_queue_to_video(aq)->isys;
	int rval;

	aq->vbq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	aq->vbq.lock = &aq->mutex;
	aq->vbq.io_modes = VB2_USERPTR | VB2_MMAP;
	aq->vbq.drv_priv = aq;
	aq->vbq.buf_struct_size = sizeof(struct css2600_isys_buffer)
		+ sizeof(struct vb2_buffer);
	aq->vbq.ops = &css2600_isys_queue_ops;
	aq->vbq.mem_ops = &vb2_dma_contig_memops;
#if LINUX_VERSION_CODE > KERNEL_VERSION(3,14,0)
	aq->vbq.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
#else
	aq->vbq.timestamp_type = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
#endif

	aq->wq = alloc_ordered_workqueue(CSS2600_NAME, 0);
	if (!aq->wq)
		return -ENOMEM;

	INIT_WORK(&aq->work, queue_stream_one_work);

	rval = vb2_queue_init(&aq->vbq);
	if (rval)
		goto fail;

	aq->ctx = vb2_dma_contig_init_ctx(&isys->adev->dev);
	if (IS_ERR(aq->ctx)) {
		vb2_queue_release(&aq->vbq);
		rval = PTR_ERR(aq->ctx);
		goto fail;
	}

	mutex_init(&aq->mutex);
	spin_lock_init(&aq->lock);
	INIT_LIST_HEAD(&aq->active);
	INIT_LIST_HEAD(&aq->incoming);

	return 0;

fail:
	destroy_workqueue(aq->wq);

	return rval;
}

void css2600_isys_queue_cleanup(struct css2600_isys_queue *aq)
{
	if (IS_ERR_OR_NULL(aq->ctx))
		return;

	vb2_dma_contig_cleanup_ctx(aq->ctx);
	vb2_queue_release(&aq->vbq);
	mutex_destroy(&aq->mutex);
	destroy_workqueue(aq->wq);

	aq->ctx = NULL;
}
