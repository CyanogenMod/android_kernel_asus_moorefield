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

#ifndef MEDIA_CSS2600_H
#define MEDIA_CSS2600_H

#include <linux/i2c.h>

#define CSS2600_ISYS_MAX_CSI2_LANES		4

struct css2600_isys_csi2_config {
	unsigned int nlanes;
	unsigned int port;
};

struct css2600_isys_subdev_i2c_info {
	struct i2c_board_info board_info;
	int i2c_adapter_id;
};

struct css2600_isys_subdev_info {
	struct css2600_isys_csi2_config *csi2;
	struct css2600_isys_subdev_i2c_info i2c;
};

struct css2600_isys_subdev_pdata {
	struct css2600_isys_subdev_info **subdevs;
};

#endif /* MEDIA_CSS2600_H */
