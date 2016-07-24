/*
 * platform_scale_adc.h: Head File for GPADC Scaling driver
 *
 * (C) Copyright 2014 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _PLATFORM_SCALE_ADC_H_
#define _PLATFORM_SCALE_ADC_H_

#define SCALE_ADC_DEV_NAME	"scale_adc"

extern void __init *scale_adc_platform_data(void *info) __attribute__((weak));
#endif
