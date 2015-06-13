/*
 * include/linux/extcon/extcon-fsa9285.h
 *
 * Copyright (C) 2013 Intel Corporation
 * Ramakrishna Pallala <ramakrishna.pallala@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef _EXTCON_FSA9285_H_
#define _EXTCON_FSA9285_H_

#include <linux/module.h>
#include <linux/extcon.h>

/**
 * struct fsa9285_pdata - platform data for FSA 9285 device.
 * @enable_vbus  - call back to enable VBUS
 * @disable_vbus  - call back to disable VBUS
 * @sdp_setup  - call back to setup SDP connection
 */
struct fsa9285_pdata {
	int mux_gpio;
	int xsd_gpio;
	int (*enable_vbus)(void);
	int (*disable_vbus)(void);
	int (*sdp_pre_setup)(void);
	int (*sdp_post_setup)(void);
};

#endif /* _EXTCON_FSA9285_H */
