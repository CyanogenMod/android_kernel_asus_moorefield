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

#ifndef CSS2600_ISYS_VIDEO_H
#define CSS2600_ISYS_VIDEO_H

#include <linux/mutex.h>
#include <linux/videodev2.h>
#include <media/media-entity.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "css2600-isys-queue.h"

struct css2600_isys;

struct css2600_isys_pixelformat {
	uint32_t pixelformat;
	uint32_t bpp;
	uint32_t bpp_packed;
	uint32_t code;
	uint32_t css_pixelformat;
	uint32_t mipi_data_type;
};

struct css2600_isys_pipeline {
	struct media_pipeline pipe;
	struct media_pad *external;
	bool continuous;
	atomic_t sequence;
	int source; /* SSI stream source */
	bool isl_enabled;
	struct completion stream_open_completion;
	struct completion stream_close_completion;
	struct completion stream_start_completion;
	struct completion stream_stop_completion;
	struct completion capture_ack_completion;
};

#define to_css2600_isys_pipeline(__pipe)				\
	container_of((__pipe), struct css2600_isys_pipeline, pipe)

struct css2600_isys_video {
	/* Serialise access to other fields in the struct. */
	struct mutex mutex;
	struct media_pad pad;
	struct video_device vdev;
	struct v4l2_pix_format pix;
	const struct css2600_isys_pixelformat *pfmt;
	struct css2600_isys_queue aq;
	struct css2600_isys *isys;
	struct css2600_isys_pipeline ip;
	unsigned int streaming;
	bool mipi_csi2_fmt;
};

#define css2600_isys_queue_to_video(__aq) \
	container_of(__aq, struct css2600_isys_video, aq)

const struct css2600_isys_pixelformat *css2600_isys_get_pixelformat(
	struct css2600_isys_video *av, uint32_t pixelformat);

static inline unsigned int css2600_isys_video_type_specifics(
	unsigned int pin_type, const struct css2600_isys_pixelformat *pfmt)
{
	if (pin_type == IA_CSS_ISYS_PIN_TYPE_MIPI)
		return pfmt->mipi_data_type;
	else
		return pfmt->css_pixelformat;
}

struct media_entity *
css2600_isys_video_find_facing_external(struct css2600_isys_video *av);
const struct css2600_isys_pixelformat *css2600_isys_video_try_fmt_vid_cap(
	struct css2600_isys_video *av, struct v4l2_pix_format *pix);
int css2600_isys_video_set_streaming(struct css2600_isys_video *av,
				     unsigned int state,
				     struct css2600_isys_buffer *ib);
int css2600_isys_video_init(struct css2600_isys_video *av,
			    struct media_entity *source,
			    unsigned int source_pad,
			    unsigned int flags);
void css2600_isys_video_cleanup(struct css2600_isys_video *av);

#endif /* CSS2600_ISYS_VIDEO_H */
