/*
 * platform_xactor.c: mipi CSI xactor platform library file
 *
 * (C) Copyright 2014 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#include "platform_camera.h"

static int csi_xactor_a_csi_configure(struct v4l2_subdev *sd, int flag)
{
	static const int LANES = 4;

	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY, LANES,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_rggb, flag);
}

static int csi_xactor_b_csi_configure(struct v4l2_subdev *sd, int flag)
{
	static const int LANES = 1;

	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, LANES,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_rggb, flag);
}

static int csi_xactor_c_csi_configure(struct v4l2_subdev *sd, int flag)
{
	static const int LANES = 2;

	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_TERTIARY, LANES,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_rggb, flag);
}

static struct atomisp_camera_caps csi_xactor_camera_caps;

static struct atomisp_camera_caps *csi_xactor_get_camera_caps(void)
{
	csi_xactor_camera_caps.sensor_num = 1;
	csi_xactor_camera_caps.sensor[0].stream_num = 2;
	return &csi_xactor_camera_caps;
}

static struct camera_sensor_platform_data csi_xactor_a_platform_data_ops = {
	.csi_cfg        = csi_xactor_a_csi_configure,
	.get_camera_caps = csi_xactor_get_camera_caps,
};

static struct camera_sensor_platform_data csi_xactor_b_platform_data_ops = {
	.csi_cfg        = csi_xactor_b_csi_configure,
};

static struct camera_sensor_platform_data csi_xactor_c_platform_data_ops = {
	.csi_cfg        = csi_xactor_c_csi_configure,
};

void *csi_xactor_a_platform_data(void *info)
{
	return &csi_xactor_a_platform_data_ops;
}

void *csi_xactor_b_platform_data(void *info)
{
	return &csi_xactor_b_platform_data_ops;
}

void *csi_xactor_c_platform_data(void *info)
{
	return &csi_xactor_c_platform_data_ops;
}

struct sfi_device_table_entry a = {
	.type = SFI_DEV_TYPE_I2C,
	.host_num = 1,
	.addr = 0x6c,
	.irq = 0,
	.name = "xactor_a"
};

struct sfi_device_table_entry b = {
	.type = SFI_DEV_TYPE_I2C,
	.host_num = 1,
	.addr = 0x6d,
	.irq = 0,
	.name = "xactor_b"
};

struct sfi_device_table_entry c = {
	.type = SFI_DEV_TYPE_I2C,
	.host_num = 1,
	.addr = 0x6e,
	.irq = 0,
	.name = "xactor_c"
};

static int __init platform_csi_xactor_module_init(void)
{
	struct devs_id *dev;
	pr_info("Adding CSI xactor device\n");
	dev = get_device_id(a.type, a.name);
	if (dev)
		dev->device_handler(&a, dev);

	dev = get_device_id(b.type, b.name);
	if (dev)
		dev->device_handler(&b, dev);

#if 0
	dev = get_device_id(c.type, c.name);
	if (dev)
		dev->device_handler(&c, dev);
#endif
	return 0;
}

module_init(platform_csi_xactor_module_init);
