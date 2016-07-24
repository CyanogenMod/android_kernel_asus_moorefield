/*
 * platform_arizona.c: arizona platform data initilization file
 *
 * (C) Copyright 2013 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/lnw_gpio.h>
#include <linux/sfi.h>
#include <asm/intel-mid.h>
#include <linux/platform_data/intel_mid_remoteproc.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/spi/intel_mid_ssp_spi.h>
#include <linux/spi/pxa2xx_spi.h>
#include <linux/mfd/arizona/pdata-largo.h>
#include <asm/platform_sst_audio.h>
#include <asm/platform_mrfld_audio.h>
#include "platform_arizona.h"
#include <sound/wm_adsp-largo.h>

/***********CS47L241 REGUATOR platform data*************/
static struct regulator_consumer_supply vcs47l241_consumer[] = {
	REGULATOR_SUPPLY("CPVDD1", "spi3.0"),
	REGULATOR_SUPPLY("CPVDD2", "spi3.0"),
	REGULATOR_SUPPLY("AVDD", "spi3.0"),
	REGULATOR_SUPPLY("MICVDD", "spi3.0"),
	REGULATOR_SUPPLY("DBVDD", "spi3.0"),
};

static struct regulator_init_data vcs47l241_data = {
		.constraints = {
			.always_on = 1,
		},
		.num_consumer_supplies	=	ARRAY_SIZE(vcs47l241_consumer),
		.consumer_supplies	=	vcs47l241_consumer,
};

static struct fixed_voltage_config vcs47l241_config = {
	.supply_name	= "V_1P80_VDD1",
	.microvolts	= 1800000,
	.gpio		= -EINVAL,
	.init_data	= &vcs47l241_data,
};

static struct platform_device vcs47l241_device = {
	.name = "reg-fixed-voltage",
	.id = PLATFORM_DEVID_AUTO,
	.dev = {
		.platform_data = &vcs47l241_config,
	},
};

/***********CS47L242 REGUATOR platform data*************/
static struct regulator_consumer_supply vcs47l242_consumer[] = {
	REGULATOR_SUPPLY("SPKVDD1", "spi3.0"),
	REGULATOR_SUPPLY("SPKVDD2", "spi3.0"),
};

static struct regulator_init_data vcs47l242_data = {
		.constraints = {
			.always_on = 1,
		},
		.num_consumer_supplies	=	ARRAY_SIZE(vcs47l242_consumer),
		.consumer_supplies	=	vcs47l242_consumer,
};

static struct fixed_voltage_config vcs47l242_config = {
	.supply_name	= "V_VSYS_AUDIO",
	.microvolts	= 1800000,
	.gpio		= -EINVAL,
	.init_data  = &vcs47l242_data,
};

static struct platform_device vcs47l242_device = {
	.name = "reg-fixed-voltage",
	.id = PLATFORM_DEVID_AUTO,
	.dev = {
		.platform_data = &vcs47l242_config,
	},
};


/***********CS47L243 REGUATOR platform data*************/
static struct regulator_consumer_supply vcs47l243_consumer[] = {
	REGULATOR_SUPPLY("DCVDD", "spi3.0"),
};

static struct regulator_init_data vcs47l243_data = {
		.constraints = {
			.always_on = 1,
		},
		.num_consumer_supplies	=	ARRAY_SIZE(vcs47l243_consumer),
		.consumer_supplies	=	vcs47l243_consumer,
};

static struct fixed_voltage_config vcs47l243_config = {
	.supply_name	= "V_1P24_AUDIO",
	.microvolts	= 1240000,
	.gpio		= -EINVAL,
	.init_data  = &vcs47l243_data,
};

static struct platform_device vcs47l243_device = {
	.name = "reg-fixed-voltage",
	.id = PLATFORM_DEVID_AUTO,
	.dev = {
		.platform_data = &vcs47l243_config,
	},
};


static struct platform_device *cs47l24_reg_devices[] __initdata = {
	&vcs47l241_device,
	&vcs47l242_device,
	&vcs47l243_device
};

static struct pxa2xx_spi_chip chip;

struct wm_adsp_fw_defs dsp3_fw = {
	.name = "SoundClear",
	.file = "soundclear-control",
	.binfile = "None",
};

static struct arizona_pdata arizona_pdata = {
	/* 1 use reset pin  0 use soft reset*/
	.reset = 0,
	/* the clk32k src is external mclk2 */
	.clk32k_src = ARIZONA_32KZ_MCLK2,
	/* micbias1&2 is set to 1.8v, enable ext_cap and bypass */
	.micbias[1] = {1800, 1, 0, 0, 1},
	.micbias[2] = {1800, 1, 0, 0, 1},
	/* audiocodec_int_n is active-low*/
	.irq_flags = IRQF_TRIGGER_FALLING,

	/* DMIC entry */
	.dmic_ref = {ARIZONA_DMIC_MICBIAS1, 0, 0, 0},
	.inmode = {ARIZONA_INMODE_DMIC, 0, 0, 0},

	/*DSP3 fw*/
	.fw_defs[2] = &dsp3_fw,
	.num_fw_defs[2] = 1,
};


static void cs_control(u32 command)
{
	udelay(2);
	if (gpio_is_valid(arizona_pdata.cs_gpio))
		gpio_set_value(arizona_pdata.cs_gpio, (command == 0x2) ? 1 : 0);

	udelay(2);
}


static int gpio_init(void)
{
	arizona_pdata.cs_gpio = get_gpio_by_name("audiocodec_cs");

	if (arizona_pdata.cs_gpio < 0)
		return -1;

	int err = gpio_request(arizona_pdata.cs_gpio, "audiocodec_cs");

	err = gpio_direction_output(arizona_pdata.cs_gpio, 1);

	return 0;
}


static int arizona_get_irq_data(struct arizona_pdata *pdata,
			struct spi_board_info *spi_info, char *name)
{
	int codec_gpio;

	codec_gpio = get_gpio_by_name(name);
	if (codec_gpio < 0) {
		pr_err("%s failed for : %d\n", __func__, codec_gpio);
		return -EINVAL;
	}
	pdata->irq_gpio = codec_gpio;
	spi_info->irq = codec_gpio + INTEL_MID_IRQ_OFFSET;
	return codec_gpio;
}


/***********CS4724 MC Driver platform data*************/
static struct mrfld_audio_platform_data phoenix_audio_pdata = {
	.spid = &spid,
};

void __init *arizona_platform_data(void *info)
{
	struct spi_board_info *spi_info = (struct spi_board_info *)info;
	struct platform_device *pdev;
	int ret;
	int irq = 0;

	if (gpio_init() != 0)
		return NULL;

	spi_info->mode = SPI_MODE_0;
	chip.cs_control = cs_control;
	spi_info->controller_data = &chip;

	if ((INTEL_MID_BOARD(1, PHONE, MRFL)) ||
		   (INTEL_MID_BOARD(1, TABLET, MRFL)) ||
		   (INTEL_MID_BOARD(1, PHONE, MOFD)) ||
		   (INTEL_MID_BOARD(1, TABLET, MOFD))) {

		platform_add_devices(cs47l24_reg_devices,
			ARRAY_SIZE(cs47l24_reg_devices));
		irq = arizona_get_irq_data(&arizona_pdata, spi_info,
						"audiocodec_int");

		if (irq < 0)
			return NULL;
	} else {
		pr_err("Not supported....\n");
		return NULL;
	}

	ret = add_sst_platform_device();
	if (ret < 0) {
		pr_err("%s failed to sst_platform device\n", __func__);
		return NULL;
	}

	/***********CS4724 Register Machine Device*************/
	pdev = platform_device_alloc("phoenix_largo", -1);
	if (!pdev) {
		pr_err("failed to allocate phoenix_largo platform device\n");
		return NULL;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("failed to add phoenix_largo platform device\n");
		platform_device_put(pdev);
		return NULL;
	}
	if (platform_device_add_data(pdev, &phoenix_audio_pdata,
				     sizeof(phoenix_audio_pdata))) {
		pr_err("failed to add phoenix_largo platform data\n");
		platform_device_put(pdev);
		return NULL;
	}

	register_rpmsg_service("rpmsg_phoenix_largo_audio", RPROC_SCU,
				RP_MSIC_MRFLD_AUDIO);

	return &arizona_pdata;
}
