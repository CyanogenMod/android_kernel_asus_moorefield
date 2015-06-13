/*
 * platform_florida.c: Florida codec platform data initilization file
 *
 * (C) Copyright 2014 Intel Corporation
 * Author: Praveen Diwakar <praveen.diwakar@intel.com>
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
#include <asm/intel-mid.h>
#include <linux/platform_data/intel_mid_remoteproc.h>
#include <asm/platform_sst_audio.h>
#include <asm/platform_mrfld_audio.h>
#include "platform_msic.h"
#include <linux/i2c.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/arizona/pdata.h>
#include <linux/mfd/arizona/registers.h>
#include <asm/intel-mid.h>
#include <linux/input.h>

/***********WM8280 1.8V REGUATOR*************/
static struct regulator_consumer_supply vflorida1_consumer[] = {
	REGULATOR_SUPPLY("AVDD",     "1-001a"),
	REGULATOR_SUPPLY("DBVDD1", "1-001a"),
	REGULATOR_SUPPLY("LDOVDD", "1-001a"),
	REGULATOR_SUPPLY("CPVDD",   "1-001a"),
	REGULATOR_SUPPLY("DBVDD2", "florida-codec"),
	REGULATOR_SUPPLY("DBVDD3", "florida-codec"),
	REGULATOR_SUPPLY("CPVDD",   "florida-codec"),
};

static struct regulator_init_data vflorida1_data = {
		.constraints = {
			.always_on = 1,
		},
		.num_consumer_supplies	=	ARRAY_SIZE(vflorida1_consumer),
		.consumer_supplies	=	vflorida1_consumer,
};

static struct fixed_voltage_config vflorida1_config = {
	.supply_name	= "DC_1V8",
	.microvolts	= 1800000,
	.gpio		= -EINVAL,
	.init_data	= &vflorida1_data,
};

static struct platform_device vflorida1_device = {
	.name = "reg-fixed-voltage",
	.id = PLATFORM_DEVID_AUTO,
	.dev = {
		.platform_data = &vflorida1_config,
	},
};

/***********WM8280 5V REGUATOR*************/
static struct regulator_consumer_supply vflorida2_consumer[] = {
	REGULATOR_SUPPLY("SPKVDDL", "florida-codec"),
	REGULATOR_SUPPLY("SPKVDDR", "florida-codec"),
};

static struct regulator_init_data vflorida2_data = {
		.constraints = {
			.always_on = 1,
		},
		.num_consumer_supplies	=	ARRAY_SIZE(vflorida2_consumer),
		.consumer_supplies	=	vflorida2_consumer,
};

static struct fixed_voltage_config vflorida2_config = {
	.supply_name	= "DC_5V",
	.microvolts	= 3700000,
	.gpio		= -EINVAL,
	.init_data  = &vflorida2_data,
};

static struct platform_device vflorida2_device = {
	.name = "reg-fixed-voltage",
	.id = PLATFORM_DEVID_AUTO,
	.dev = {
		.platform_data = &vflorida2_config,
	},
};

/***********WM8280 MC Driver platform data*************/
static struct mrfld_audio_platform_data moor_audio_pdata = {
	.spid = &spid,
};

/***********WM8280 Codec Driver platform data*************/
static const struct arizona_micd_range micd_ctp_ranges[] = {
	{ .max =  11, .key = BTN_0 },
	{ .max =  28, .key = BTN_1 },
	{ .max =  54, .key = BTN_2 },
	{ .max = 100, .key = BTN_3 },
	{ .max = 186, .key = BTN_4 },
	{ .max = 430, .key = BTN_5 },
};

static const struct arizona_micd_config micd_modes[] = {
	/*{Acc Det on Micdet2, Use Micbias1 for detection,
	 * Set GPIO to 0 to selecte this polarity}*/
	{ ARIZONA_ACCDET_SRC, 1, 0 },
	/*{Acc Det on Micdet1, Use Micbias2 for detection,
	 * Set GPIO to 1 to selecte this polarity}*/
	{ 0, 2, 1 },
};

static struct arizona_pdata florida_pdata  = {
	.reset = 0, /*No Reset GPIO from AP, use SW reset*/
	.ldoena = 0, /*TODO: Add actual GPIO for LDOEN, use SW Control for now*/
	.irq_flags = IRQF_TRIGGER_LOW | IRQF_ONESHOT,
	.clk32k_src = ARIZONA_32KZ_MCLK2, /*Onboard OSC provides 32K on MCLK2*/
	/*IN1 uses both MICBIAS1 and MICBIAS2 based on jack polarity,
	the below values in dmic_ref only has meaning for DMIC's and not AMIC's*/
	.dmic_ref = {ARIZONA_DMIC_MICBIAS1, ARIZONA_DMIC_MICBIAS3, 0, 0},
	.inmode = {ARIZONA_INMODE_SE, ARIZONA_INMODE_DMIC, 0, 0},
	.gpio_base = 0, /* Base allocated by gpio core*/
	.micd_pol_gpio = 2, /* GPIO3 (offset 2 from gpio_base) of the codec*/
	.micd_configs = micd_modes,
	.num_micd_configs = ARRAY_SIZE(micd_modes),
	.jd_gpio5 = 0, /*gpio5 is not used for jack detection, instead JACKDET pin is used*/
	.micd_ranges = micd_ctp_ranges, /*Impedance Range for HS Buttons*/
	.num_micd_ranges = ARRAY_SIZE(micd_ctp_ranges),
	.gpsw = 0x3, /*Use the Micbias Clamping Switch to prevent pops*/
	.micd_force_micbias = true,
	.dynamic_gpio = true,
};

/***********WM8280 Codec Device*************/
static struct i2c_board_info __initdata florida_board_info = {
	I2C_BOARD_INFO("wm8280", 0x1a),
	.platform_data = &florida_pdata,
};

void *moor_wm8280_audio_platform_data(void *info)
{
	struct platform_device *pdev;
	int ret, codec_gpio;

	ret = add_sst_platform_device();
	if (ret < 0) {
		pr_err("%s failed to sst_platform device\n", __func__);
		return NULL;
	}

	/***********WM8280 Get Codec IRQ*************/
	codec_gpio = get_gpio_by_name("audiocodec_int");
	if (codec_gpio < 0) {
		pr_err("%s failed for : %d\n", __func__, codec_gpio);
		return -EINVAL;
	}
	florida_board_info.irq = codec_gpio + INTEL_MID_IRQ_OFFSET;

	/***********WM8280 Register Codec Device*************/
	i2c_register_board_info(1, &florida_board_info, 1);

	/***********WM8280 Register Regulator*************/
	platform_device_register(&vflorida1_device);
	platform_device_register(&vflorida2_device);


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

	/***********WM8280 Register Machine Device*************/
	pdev = platform_device_alloc("moor_florida", -1);
	if (!pdev) {
		pr_err("failed to allocate moor_florida platform device\n");
		return NULL;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("failed to add moor_florida platform device\n");
		platform_device_put(pdev);
		return NULL;
	}
	if (platform_device_add_data(pdev, &moor_audio_pdata,
				     sizeof(moor_audio_pdata))) {
		pr_err("failed to add moor_florida platform data\n");
		platform_device_put(pdev);
		return NULL;
	}

	register_rpmsg_service("rpmsg_moor_florida_audio", RPROC_SCU,
				RP_MSIC_MRFLD_AUDIO);

	return NULL;
}
