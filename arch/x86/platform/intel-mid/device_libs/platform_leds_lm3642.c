/*
 * platform_leds_lm3559.c: lm3559 platform data initilization file
 *
 * (C) Copyright 2012 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/platform_data/leds-lm3642.h>

static struct lm3642_platform_data platform_data;
void *lm3642_platform_data(void *info)
{

	platform_data.torch_pin  = LM3642_TORCH_PIN_DISABLE;
	platform_data.strobe_pin = LM3642_STROBE_PIN_DISABLE;
	platform_data.tx_pin  = LM3642_TX_PIN_DISABLE;
	platform_data.uvlo  = LM3642_UVLO_DISABLE;

	return &platform_data;
}
