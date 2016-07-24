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

#include <media/css2600-isys.h>
#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-core.h>

#include "css2600.h"
#include "css2600-bus.h"
#include "css2600-isys.h"
#include "css2600-isys-csi2.h"
#include "css2600-isys-csi2-reg.h"
#include "css2600-isys-lib.h"
#include "css2600-isys-subdev.h"
#include "css2600-isys-video.h"

static const uint32_t csi2_supported_codes_pad[] = {
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

static const uint32_t *csi2_supported_codes[] = {
	csi2_supported_codes_pad,
	csi2_supported_codes_pad,
};

static struct v4l2_subdev_internal_ops csi2_sd_internal_ops = {
	.open = css2600_isys_subdev_open,
	.close = css2600_isys_subdev_close,
};

static const struct v4l2_subdev_core_ops csi2_sd_core_ops = {
};

/*
 * The ISP2401 new input system CSI2+ receiver has several
 * parameters affecting the receiver timings. These depend
 * on the MIPI bus frequency F in Hz (sensor transmitter rate)
 * as follows:
 *	register value = (A/1e9 + B * UI) / COUNT_ACC
 * where
 *	UI = 1 / (2 * F) in seconds
 *	COUNT_ACC = counter accuracy in seconds
 *	For ANN and CHV, COUNT_ACC = 0.0625 ns
 *	For BXT,  COUNT_ACC = 0.125 ns
 *
 * A and B are coefficients from the table below,
 * depending whether the register minimum or maximum value is
 * calculated.
 *				       Minimum     Maximum
 * Clock lane			       A     B     A     B
 * reg_rx_csi_dly_cnt_termen_clane     0     0    38     0
 * reg_rx_csi_dly_cnt_settle_clane    95    -8   300   -16
 * Data lanes
 * reg_rx_csi_dly_cnt_termen_dlane0    0     0    35     4
 * reg_rx_csi_dly_cnt_settle_dlane0   85    -2   145    -6
 * reg_rx_csi_dly_cnt_termen_dlane1    0     0    35     4
 * reg_rx_csi_dly_cnt_settle_dlane1   85    -2   145    -6
 * reg_rx_csi_dly_cnt_termen_dlane2    0     0    35     4
 * reg_rx_csi_dly_cnt_settle_dlane2   85    -2   145    -6
 * reg_rx_csi_dly_cnt_termen_dlane3    0     0    35     4
 * reg_rx_csi_dly_cnt_settle_dlane3   85    -2   145    -6
 *
 * We use the minimum values of both A and B.
 */

#define DIV_SHIFT	8

static uint32_t calc_timing(int32_t a, int32_t b, int64_t link_freq,
			    int32_t accinv)
{
	return accinv * a + (accinv * b * (500000000 >> DIV_SHIFT)
			     / (int32_t)(link_freq >> DIV_SHIFT));
}

int css2600_isys_csi2_calc_timing(struct css2600_isys_csi2 *csi2,
				  struct css2600_isys_csi2_timing *timing,
				  uint32_t accinv)
{
	struct css2600_isys_pipeline *pipe =
		container_of(csi2->asd.sd.entity.pipe,
			     struct css2600_isys_pipeline, pipe);
	struct v4l2_subdev *ext_sd =
		media_entity_to_v4l2_subdev(pipe->external->entity);
	struct v4l2_ext_control c = { .id = V4L2_CID_LINK_FREQ, };
	struct v4l2_ext_controls cs = { .count = 1,
					.controls = &c, };
	struct v4l2_querymenu qm = { .id = c.id, };
	int rval;

	rval = v4l2_g_ext_ctrls(ext_sd->ctrl_handler, &cs);
	if (rval) {
		dev_info(&csi2->isys->adev->dev, "can't get link frequency\n");
		return rval;
	}

	qm.index = c.value;

	rval = v4l2_querymenu(ext_sd->ctrl_handler, &qm);
	if (rval) {
		dev_info(&csi2->isys->adev->dev, "can't get menu item\n");
		return rval;
	}

	dev_dbg(&csi2->isys->adev->dev, "%s: link frequency %lld\n", __func__,
		qm.value);

	if (!qm.value)
		return -EINVAL;

	timing->ctermen = calc_timing(
		CSI2_CSI_RX_DLY_CNT_TERMEN_CLANE_A,
		CSI2_CSI_RX_DLY_CNT_TERMEN_CLANE_B, qm.value, accinv);
	timing->csettle = calc_timing(
		CSI2_CSI_RX_DLY_CNT_SETTLE_CLANE_A,
		CSI2_CSI_RX_DLY_CNT_SETTLE_CLANE_B, qm.value, accinv);
	dev_dbg(&csi2->isys->adev->dev, "ctermen %u\n", timing->ctermen);
	dev_dbg(&csi2->isys->adev->dev, "csettle %u\n", timing->csettle);

	timing->dtermen = calc_timing(
		CSI2_CSI_RX_DLY_CNT_TERMEN_DLANE_A,
		CSI2_CSI_RX_DLY_CNT_TERMEN_DLANE_B, qm.value, accinv);
	timing->dsettle = calc_timing(
		CSI2_CSI_RX_DLY_CNT_SETTLE_DLANE_A,
		CSI2_CSI_RX_DLY_CNT_SETTLE_DLANE_B, qm.value, accinv);
	dev_dbg(&csi2->isys->adev->dev, "dtermen %u\n", timing->dtermen);
	dev_dbg(&csi2->isys->adev->dev, "dsettle %u\n", timing->dsettle);

	return 0;
}

#define CSI2_ACCINV	8

static int set_stream(struct v4l2_subdev *sd, int enable)
{
	struct css2600_isys_csi2 *csi2 = to_css2600_isys_csi2(sd);
	struct css2600_isys_pipeline *ip =
		container_of(sd->entity.pipe,
			     struct css2600_isys_pipeline, pipe);
	struct css2600_isys_csi2_config *cfg =
		v4l2_get_subdev_hostdata(
			media_entity_to_v4l2_subdev(ip->external->entity));
	struct css2600_isys_csi2_timing timing;
	unsigned int i;
	int rval;

	dev_dbg(&csi2->isys->adev->dev, "csi2 s_stream %d\n", enable);

	if (!enable) {
		writel(0, csi2->base + CSI2_REG_CSI_RX_ENABLE);
		return 0;
	}

	rval = css2600_isys_csi2_calc_timing(csi2, &timing, CSI2_ACCINV);
	if (rval)
		return rval;

	writel(timing.ctermen,
	       csi2->base + CSI2_REG_CSI_RX_DLY_CNT_TERMEN_CLANE);
	writel(timing.csettle,
	       csi2->base + CSI2_REG_CSI_RX_DLY_CNT_SETTLE_CLANE);

	for (i = 0; i < cfg->nlanes; i++) {
		writel(timing.dtermen,
		       csi2->base + CSI2_REG_CSI_RX_DLY_CNT_TERMEN_DLANE(i));
		writel(timing.dsettle,
		       csi2->base + CSI2_REG_CSI_RX_DLY_CNT_SETTLE_DLANE(i));
	}

	writel(CSI2_CSI_RX_ENABLE_ENABLE, csi2->base + CSI2_REG_CSI_RX_ENABLE);

	return 0;
}

static const struct v4l2_subdev_video_ops csi2_sd_video_ops = {
	.s_stream = set_stream,
};

static const struct v4l2_subdev_pad_ops csi2_sd_pad_ops = {
	.link_validate = css2600_isys_subdev_link_validate,
	.get_fmt = css2600_isys_subdev_get_ffmt,
	.set_fmt = css2600_isys_subdev_set_ffmt,
};

static struct v4l2_subdev_ops csi2_sd_ops = {
	.core = &csi2_sd_core_ops,
	.video = &csi2_sd_video_ops,
	.pad = &csi2_sd_pad_ops,
};

static struct media_entity_operations csi2_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

void css2600_isys_csi2_cleanup(struct css2600_isys_csi2 *csi2)
{
	v4l2_device_unregister_subdev(&csi2->asd.sd);
	css2600_isys_subdev_cleanup(&csi2->asd);
	css2600_isys_video_cleanup(&csi2->av);
}

int css2600_isys_csi2_init(struct css2600_isys_csi2 *csi2, struct css2600_isys *isys,
		      void __iomem *base, unsigned int nlanes,
		      unsigned int index)
{
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.pad = CSI2_PAD_SINK,
		.format = {
			.width = 4096,
			.height = 3072,
		},
	};
	int rval;

	csi2->isys = isys;
	csi2->base = base;
	csi2->nlanes = nlanes;
	csi2->index = index;

	BUG_ON(CSS2600_ISYS_MAX_PAD < NR_OF_CSI2_PADS);
	csi2->asd.pad[CSI2_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	csi2->asd.pad[CSI2_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	csi2->asd.sd.entity.ops = &csi2_entity_ops;

	rval = css2600_isys_subdev_init(&csi2->asd, &csi2_sd_ops, 0,
					NR_OF_CSI2_PADS);
	if (rval)
		goto fail;

	csi2->asd.isys = isys;
	csi2->asd.source = IA_CSS_ISYS_STREAM_SRC_CSI2_PORT0 + index;
	csi2->asd.supported_codes = csi2_supported_codes;
	css2600_isys_subdev_set_ffmt(&csi2->asd.sd, NULL, &fmt);

	csi2->asd.sd.internal_ops = &csi2_sd_internal_ops;
	snprintf(csi2->asd.sd.name, sizeof(csi2->asd.sd.name),
		 CSS2600_NAME " CSI-2 %u", index);
	v4l2_set_subdevdata(&csi2->asd.sd, &csi2->asd);
	rval = v4l2_device_register_subdev(&isys->v4l2_dev, &csi2->asd.sd);
	if (rval) {
		dev_info(&isys->adev->dev, "can't register v4l2 subdev\n");
		goto fail;
	}

	snprintf(csi2->av.vdev.name, sizeof(csi2->av.vdev.name),
		 CSS2600_NAME " CSI-2 %u capture", index);
	csi2->av.isys = isys;
	csi2->av.aq.css_pin_type = IA_CSS_ISYS_PIN_TYPE_MIPI;
	csi2->av.mipi_csi2_fmt = true;
	rval = css2600_isys_video_init(
		&csi2->av, &csi2->asd.sd.entity, CSI2_PAD_SOURCE, 0);
	if (rval) {
		dev_info(&isys->adev->dev, "can't init video node\n");
		goto fail;
	}

	return 0;

fail:
	css2600_isys_csi2_cleanup(csi2);

	return rval;
}

void css2600_isys_csi2_isr(struct css2600_isys_csi2 *csi2)
{
}

static bool css2600_isys_csi2_is_idle(struct css2600_isys_csi2 *csi2)
{
	return readl(csi2->base + CSI2_REG_CSI_RX_STATUS)
		!= CSI2_CSI_RX_STATUS_BUSY;
}
