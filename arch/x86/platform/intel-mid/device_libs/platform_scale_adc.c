/*
 * platform_scale_adc.c: Platform data for GPADC Scaling driver
 *
 * (C) Copyright 2014 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sfi.h>
#include <linux/iio/iio.h>
#include <linux/iio/machine.h>
#include <linux/iio/types.h>
#include <asm/intel-mid.h>
#include <asm/intel_basincove_gpadc.h>
#include <asm/intel_scale_gpadc.h>
#include <asm/intel_mid_remoteproc.h>

#include "platform_scale_adc.h"

#define SCALE_ADC_MAP(_adc_channel_label,			\
		     _consumer_dev_name,                        \
		     _consumer_channel)                         \
	{                                                       \
		.adc_channel_label = _adc_channel_label,        \
		.consumer_dev_name = _consumer_dev_name,        \
		.consumer_channel = _consumer_channel,          \
	}

struct iio_map scale_iio_maps[] = {
	SCALE_ADC_MAP("CH0", "PMICTEMP", "SPMICTEMP"),
	SCALE_ADC_MAP("CH1", "BATTEMP", "SBATTEMP0"),
	SCALE_ADC_MAP("CH2", "BATTEMP", "SBATTEMP1"),
	SCALE_ADC_MAP("CH3", "SYSTEMP", "SSYSTEMP0"),
	SCALE_ADC_MAP("CH4", "SYSTEMP", "SSYSTEMP1"),
	SCALE_ADC_MAP("CH5", "SYSTEMP", "SSYSTEMP2"),
	{ },
};

#define SCALE_ADC_CHANNEL(_type, _channel, _datasheet_name) \
	{                               \
		.indexed = 1,           \
		.type = _type,          \
		.channel = _channel,    \
		.datasheet_name = _datasheet_name,      \
	}
static const struct iio_chan_spec const scale_adc_channels[] = {
	SCALE_ADC_CHANNEL(IIO_TEMP, 0, "CH0"),
	SCALE_ADC_CHANNEL(IIO_TEMP, 1, "CH1"),
	SCALE_ADC_CHANNEL(IIO_TEMP, 2, "CH2"),
	SCALE_ADC_CHANNEL(IIO_TEMP, 3, "CH3"),
	SCALE_ADC_CHANNEL(IIO_TEMP, 4, "CH4"),
	SCALE_ADC_CHANNEL(IIO_TEMP, 5, "CH5"),
};

static struct channel_thrms_map shadycove_thrms_map[] = {
	{0, "PMICTEMP", NTC_10K},
	{1, "BATTEMP0", NTC_10K},
	{2, "BATTEMP1", NTC_10K},
	{3, "SYSTEMP0", NTC_10K},
	{4, "SYSTEMP1", NTC_10K},
	{5, "SYSTEMP2", NTC_10K},
};

static struct intel_scale_gpadc_platform_data scale_adc_pdata;

void __init *scale_adc_platform_data(void *info)
{
	struct platform_device *pdev = NULL;
	int ret;

	pdev = platform_device_alloc(SCALE_ADC_DEV_NAME, -1);

	if (!pdev) {
		pr_err("out of memory for SFI platform dev %s\n",
					SCALE_ADC_DEV_NAME);
		goto out;
	}

	scale_adc_pdata.channel_num = SCALE_CH_NUM;
	scale_adc_pdata.gpadc_iio_maps = scale_iio_maps;
	scale_adc_pdata.gpadc_channels = scale_adc_channels;

	if (INTEL_MID_BOARD(1, PHONE, MOFD) ||
			INTEL_MID_BOARD(1, TABLET, MOFD)) {
		scale_adc_pdata.pmic_adc_temp_conv = shadycove_pmic_adc_temp_conv;
		scale_adc_pdata.scale_chan_map = shadycove_thrms_map;
	}

	pdev->dev.platform_data = &scale_adc_pdata;

	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("failed to add scale_adc platform device\n");
		platform_device_put(pdev);
		goto out;
	}

out:
	return &scale_adc_pdata;
}
