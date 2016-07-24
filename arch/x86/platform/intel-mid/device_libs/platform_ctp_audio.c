/*
 * platform_ctp_audio.c: CLVS audio platform data initilization file
 *
 * (C) Copyright 2008-2013 Intel Corporation
 * Author: KP Jeeja<jeeja.kp@intel.com>
 * Author: Dharageswari.R<dharageswari.r@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/scatterlist.h>
#include <linux/init.h>
#include <linux/sfi.h>
#include <linux/platform_device.h>
#include <asm/platform_sst_audio.h>
#include <asm/intel-mid.h>
#include <linux/platform_data/intel_mid_remoteproc.h>
#include <asm/platform_ctp_audio.h>
#include "platform_msic.h"
/*FIX ME:change it to get_gpio_by_name once the name is added in IFWI*/
#define GPIO_DMIC_1_EN 88

static struct ctp_audio_platform_data ctp_audio_pdata = {
	.spid = &spid,
};

void *ctp_audio_platform_data(void *info)
{
	struct platform_device *pdev;
	int ret;
	struct sfi_device_table_entry *pentry = info;
	char name[SFI_NAME_LEN+1];

	ctp_audio_pdata.codec_gpio_hsdet = get_gpio_by_name("gpio_plugdet");
#if defined(CONFIG_ME372CL) || defined(CONFIG_PF450CL)
	ctp_audio_pdata.codec_gpio_button = get_gpio_by_name("HOOK_DET");
#else
	ctp_audio_pdata.codec_gpio_button = get_gpio_by_name("gpio_codec_int");
#endif /* CONFIG_ME372CL */
	ctp_audio_pdata.codec_gpio_dmic = GPIO_DMIC_1_EN;
	ret = add_sst_platform_device();
	if (ret < 0)
		return NULL;

	pdev = platform_device_alloc("hdmi-audio", -1);
	if (!pdev) {
		pr_err("failed to allocate hdmi-audio platform device\n");
		return NULL;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("failed to add hdmi-audio platform device\n");
		platform_device_put(pdev);
		return NULL;
	}

	/* DEV names from IFWI is not properly terminated.
	This causes id comparision to fail.
	Make a copy of name and make sure null terminated */
	memset(name, 0, sizeof(name));
	strncpy(name, pentry->name, SFI_NAME_LEN);
	pdev = platform_device_alloc(name, -1);
	if (!pdev) {
		pr_err("failed to allocate clvcs_audio platform device\n");
		return NULL;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("failed to add clvcs_audio platform device\n");
		platform_device_put(pdev);
		return NULL;
	}
	if (platform_device_add_data(pdev, &ctp_audio_pdata,
			sizeof(struct ctp_audio_platform_data))) {
		pr_err("failed to add ctp_audio platform data\n");
		platform_device_put(pdev);
		return NULL;
	}

	register_rpmsg_service("rpmsg_msic_clv_audio", RPROC_SCU,
				RP_MSIC_CLV_AUDIO);
	return NULL;
}
