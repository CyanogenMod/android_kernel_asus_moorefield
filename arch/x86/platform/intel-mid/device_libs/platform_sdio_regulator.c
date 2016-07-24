/*
 * platform_sdio_regulator.c: sdio regulator platform device initilization file
 *
 * (C) Copyright 2011 Intel Corporation
 * Author: chuanxiao.dong@intel.com, feiyix.ning@intel.com
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <asm/intel-mid.h>
#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <linux/delay.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/gpio-regulator.h>
#include <linux/acpi.h>
#include <linux/acpi_gpio.h>
#include <linux/mfd/intel_mid_pmic.h>
#include <asm/cpu_device_id.h>
#include <linux/regulator/intel_whiskey_cove_pmic.h>
#include <linux/regulator/intel_dcovex_regulator.h>

#define DELAY_ONOFF 250

struct acpi_ids { char *hid; char *uid; };

#define INTEL_CHV_CPU	0x4c
#define INTEL_VLV_CPU	0x37
#define CHT_GPIO_SE_BASE	0xfed98000
#define CHT_GPIO_SE_LEN		0x7fff

#define CHT_VSDCARD_PWR_GPIO	276
#define CHT_VSDCARD_PWR_ADR	0x5818
#define CHT_VSDCARD_PWR_CONF0	0x8102

#define CHT_VSDIO_1P8_GPIO	283
#define CHT_VSDIO_1P8_ADR	0x5850
#define CHT_VSDIO_1P8_CONF0	0x8100

static const struct x86_cpu_id intel_cpus[] = {
	{ X86_VENDOR_INTEL, 6, INTEL_VLV_CPU, X86_FEATURE_ANY, 0 },
	{ X86_VENDOR_INTEL, 6, INTEL_CHV_CPU, X86_FEATURE_ANY, 0 },
	{}
};

static struct acpi_ids intel_brc_ids[] = {
	{"BCM4321", NULL}, /* BYT SDIO */
	{"RTL8723" , NULL}, /* BCR SDIO */
	{"BCM43241", NULL}, /* CHT RVP */
	{ },
};

static struct regulator_consumer_supply wlan_vmmc_supply = {
	.supply = "vmmc",
};

static struct regulator_init_data wlan_vmmc_data = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies = &wlan_vmmc_supply,
};

static struct fixed_voltage_config vwlan = {
	.supply_name            = "wlan_en_acpi",
	.microvolts             = 1800000,
	.gpio                   = -EINVAL,
	.startup_delay          = 1000 * DELAY_ONOFF,
	.enable_high            = 1,
	.enabled_at_boot        = 0,
	.init_data              = &wlan_vmmc_data,
};

static void vwlan_device_release(struct device *dev) {}

static struct platform_device vwlan_device = {
	.name   = "reg-fixed-voltage",
	.id             = PLATFORM_DEVID_AUTO,
	.dev = {
		.platform_data  = &vwlan,
		.release = vwlan_device_release,
	},
};

static struct acpi_device *acpi_bus_get_parent(acpi_handle handle)
{
	struct acpi_device *device = NULL;
	acpi_status status;
	int result;
	struct acpi_device *acpi_root;

	result = acpi_bus_get_device(ACPI_ROOT_OBJECT, &acpi_root);
	if (result)
		return NULL;

	/*
	 * Fixed hardware devices do not appear in the namespace and do not
	 * have handles, but we fabricate acpi_devices for them, so we have
	 * to deal with them specially.
	 */
	if (!handle)
		return acpi_root;

	do {
		status = acpi_get_parent(handle, &handle);
		if (ACPI_FAILURE(status))
			return status == AE_NULL_ENTRY ? NULL : acpi_root;
	} while (acpi_bus_get_device(handle, &device));

	return device;
}

static int sdhc_acpi_match(struct device *dev, void *data)
{
	struct acpi_ids *ids = data;
	struct acpi_handle *handle = ACPI_HANDLE(dev);
	struct acpi_device_info *info;
	acpi_status status;
	int match = 0;

	status = acpi_get_object_info(handle, &info);
	if (ACPI_FAILURE(status))
		return 0;

	if (!(info->valid & ACPI_VALID_UID) ||
		!(info->valid & ACPI_VALID_HID))
		goto out;

	if (!strncmp(ids->hid, info->hardware_id.string, strlen(ids->hid)))
		if (!strncmp(ids->uid, info->unique_id.string,
					strlen(ids->uid)))
			match = 1;
out:
	kfree(info);
	return match;
}

static int brc_acpi_match(struct device *dev, void *data)
{
	struct acpi_ids *ids = data;

	if (!strncmp(ids->hid, dev_name(dev), strlen(ids->hid)))
			return true;

	return false;
}


static int sdio_fixed_regulator_register_by_acpi(struct platform_device *pdev)
{
	struct device *dev;
	struct acpi_ids *brc_ids;
	struct fixed_voltage_config *fixedcfg = NULL;
	struct regulator_init_data *data = NULL;
	struct acpi_handle *handle;
	struct acpi_device *parent;

	if (!pdev)
		return -ENODEV;
	fixedcfg = pdev->dev.platform_data;
	if (!fixedcfg)
		return -ENODEV;
	data = fixedcfg->init_data;
	if (!data || !data->consumer_supplies)
		return -ENODEV;

	/* get the GPIO pin from ACPI device first */
	for (brc_ids = intel_brc_ids; brc_ids->hid; brc_ids++) {
		dev = bus_find_device(&platform_bus_type, NULL,
				brc_ids, brc_acpi_match);
		if (dev) {
			handle = ACPI_HANDLE(dev);
			if (!ACPI_HANDLE(dev))
				continue;
			parent = acpi_bus_get_parent(handle);
			if (!parent)
				continue;

			data->consumer_supplies->dev_name =
						dev_name(&parent->dev);
			fixedcfg->gpio = acpi_get_gpio_by_index(dev, 1, NULL);
			if (fixedcfg->gpio < 0) {
				dev_info(dev, "No wlan-enable GPIO\n");
				continue;
			}
			dev_info(dev, "wlan-enable GPIO %d found\n",
					fixedcfg->gpio);
			break;
		}
	}

	if (brc_ids->hid) {
		/* add a regulator to control wlan enable gpio */
		return platform_device_register(&vwlan_device);
	}

	return -ENODEV;
}

/* fixed regulator */
static struct regulator_consumer_supply ccove_vsdcard_consumer =
				REGULATOR_SUPPLY("vmmc", "");

static struct regulator_init_data ccove_vsdcard_data = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &ccove_vsdcard_consumer,
};

static struct fixed_voltage_config ccove_vsdcard = {
	.supply_name	= "gpio_vsdcard",
	.microvolts	= 3300000,
	.init_data	= &ccove_vsdcard_data,
};

/* GPIO regulator */
static struct regulator_consumer_supply ccove_vsdio_consumer =
				REGULATOR_SUPPLY("vqmmc", "");

static struct regulator_init_data ccove_vsdio_data = {
	.constraints = {
		.min_uV			= 1700000,
		.max_uV			= 3300000,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE |
					REGULATOR_CHANGE_STATUS,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &ccove_vsdio_consumer,
};

static struct gpio_regulator_state ccove_vsdio_states[] = {
	{
		.value = 3300000,
		.gpios = 0,
	},
	{
		.value = 1800000,
		.gpios = 1,
	},
};

static struct gpio ccove_vsdio_gpios = {
	.flags = GPIOF_OUT_INIT_LOW,
	.label = "byt_vsdio",
};

static struct gpio_regulator_config ccove_vsdio = {
	.supply_name	= "gpio_vsdio",
	.enable_gpio	= -ENODEV,
	.gpios		= &ccove_vsdio_gpios,
	.nr_gpios	= 1,
	.states		= ccove_vsdio_states,
	.nr_states	= ARRAY_SIZE(ccove_vsdio_states),
	.type		= REGULATOR_VOLTAGE,
	.init_data	= &ccove_vsdio_data,
};

static struct acpi_ids intel_sdhc_ids[] = {
	{"80860F14", "3"}, /* BYT SD */
	{"INT33BB", "3"},
	{ },
};

static bool intel_cpu_module(unsigned int *cpu)
{
	const struct x86_cpu_id *id;
	if (!cpu)
		return false;
	id = x86_match_cpu(intel_cpus);
	if (!id)
		return false;
	*cpu = id->model;
	return true;
}

static void intel_setup_ccove_sd_regulators(void)
{
	struct device *dev = NULL;
	struct acpi_ids *sdhc_ids;
	u32 cpu;

	for (sdhc_ids = intel_sdhc_ids; sdhc_ids->hid; sdhc_ids++) {
		dev = bus_find_device(&platform_bus_type, NULL,
				sdhc_ids, sdhc_acpi_match);
		if (dev)
			break;
	}

	if (!dev)
		return;

	/* configure vsdio */
	ccove_vsdio_consumer.dev_name = dev_name(dev);
	ccove_vsdio_gpios.gpio =
		acpi_get_gpio_by_index(dev, 2, NULL);
	/* configure vsdcard */
	ccove_vsdcard_consumer.dev_name = dev_name(dev);
	ccove_vsdcard.gpio =
		acpi_get_gpio_by_index(dev, 3, NULL);

	if (intel_cpu_module(&cpu) && (cpu == INTEL_CHV_CPU)) {
		void __iomem *ioaddr = ioremap_nocache(CHT_GPIO_SE_BASE,
				CHT_GPIO_SE_LEN);
		/* set pin to GPIO mode */
		writel(CHT_VSDCARD_PWR_CONF0, ioaddr + CHT_VSDCARD_PWR_ADR);
		ccove_vsdcard.gpio = CHT_VSDCARD_PWR_GPIO;

		writel(CHT_VSDIO_1P8_CONF0, ioaddr + CHT_VSDIO_1P8_ADR);
		ccove_vsdio_gpios.gpio = CHT_VSDIO_1P8_GPIO;
	}

	if (intel_cpu_module(&cpu) && (cpu == INTEL_VLV_CPU)) {
		/* change GPIO pins to be output GPIO */
		if (ccove_vsdio_gpios.gpio >= 0) {
			lnw_gpio_set_alt(ccove_vsdio_gpios.gpio, 0);
			gpio_request(ccove_vsdio_gpios.gpio, "sd_vqmmc");
			gpio_direction_output(ccove_vsdio_gpios.gpio, 0);
			gpio_free(ccove_vsdio_gpios.gpio);
		}
		if (ccove_vsdcard.gpio > 0) {
			lnw_gpio_set_alt(ccove_vsdcard.gpio, 0);
			gpio_request(ccove_vsdcard.gpio, "sd_vmmc");
			gpio_direction_output(ccove_vsdcard.gpio, 0);
			gpio_free(ccove_vsdcard.gpio);
		}
	}

	if (ccove_vsdio_gpios.gpio < 0) {
		pr_warn("%s: No VSDIO GPIO found\n", __func__);
		ccove_vsdio.supply_name = NULL;
	}
	intel_mid_pmic_set_pdata("gpio-regulator", &ccove_vsdio,
			sizeof(struct gpio_regulator_config), 0);

	if (ccove_vsdcard.gpio < 0) {
		pr_warn("%s: No VSDCARD GPIO found\n", __func__);
		ccove_vsdcard.supply_name = NULL;
	}
	intel_mid_pmic_set_pdata("reg-fixed-voltage", &ccove_vsdcard,
			sizeof(struct fixed_voltage_config), 0);
}

/*************************************************************
*
* WCOVE SD card related regulator
*
*************************************************************/

static struct regulator_consumer_supply vmmc_consumer[] = {
	REGULATOR_SUPPLY("vmmc", "INT33BB:01"),
};

static struct regulator_init_data vmmc_data = {
	.constraints = {
		.min_uV = 3000000,
		.max_uV = 3400000,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
	},
	.num_consumer_supplies = ARRAY_SIZE(vmmc_consumer),
	.consumer_supplies	= vmmc_consumer,
};

static struct wcove_regulator_info wcove_vmmc_info = {
	.init_data = &vmmc_data,
};

static struct regulator_consumer_supply vqmmc_consumer[] = {
	REGULATOR_SUPPLY("vqmmc", "INT33BB:01"),
};

static struct regulator_init_data vqmmc_data = {
	.constraints = {
		.min_uV = 1700000,
		.max_uV = 3400000,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
	},
	.num_consumer_supplies = ARRAY_SIZE(vqmmc_consumer),
	.consumer_supplies	= vqmmc_consumer,
};

static struct wcove_regulator_info wcove_vqmmc_info = {
	.init_data = &vqmmc_data,
};

/****************DCOVEX LDO2 RAIL Platform Data********************/
static struct regulator_consumer_supply dcovex_ldo2_consumer[] = {
	REGULATOR_SUPPLY("vmmc", "80860F14:01"),
	REGULATOR_SUPPLY("vmmc", "INT33BB:01"),
};

static struct regulator_init_data dcovex_ldo2_data = {
	.constraints = {
		.name			= "LDO_2",
		.min_uV			= 1800000,
		.max_uV			= 3300000,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE |
						REGULATOR_CHANGE_STATUS,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
	},
	.num_consumer_supplies	= ARRAY_SIZE(dcovex_ldo2_consumer),
	.consumer_supplies	= dcovex_ldo2_consumer,
};

static struct dcovex_regulator_info dcovex_ldo2 = {
	.init_data = &dcovex_ldo2_data,
};

/****************DCOVEX GPIO_1 RAIL Platform Data ********************/
static struct regulator_consumer_supply dcovex_gpio1_consumer[] = {
	REGULATOR_SUPPLY("vqmmc", "80860F14:01"),
	REGULATOR_SUPPLY("vqmmc", "INT33BB:01"),
};

static struct regulator_init_data dcovex_gpio1_data = {
	.constraints = {
		.name			= "GPIO_1",
		.min_uV			= 1800000,
		.max_uV			= 3300000,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE |
						REGULATOR_CHANGE_STATUS,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
	},
	.num_consumer_supplies	= ARRAY_SIZE(dcovex_gpio1_consumer),
	.consumer_supplies	= dcovex_gpio1_consumer,
};

static struct dcovex_regulator_info dcovex_gpio1 = {
	.init_data = &dcovex_gpio1_data,
};

static int __init sdio_regulator_init(void)
{
	int ret;
	/* register fixed regulator through ACPI device */
	ret = sdio_fixed_regulator_register_by_acpi(&vwlan_device);
	if (ret)
		pr_err("%s: No SDIO device regulator registered\n", __func__);

	intel_setup_ccove_sd_regulators();

	/* register SD card regulator for whiskey cove PMIC */
	intel_mid_pmic_set_pdata("wcove_regulator", &wcove_vmmc_info,
			sizeof(struct regulator_init_data), WCOVE_ID_V3P3SD + 1);
	intel_mid_pmic_set_pdata("wcove_regulator", &wcove_vqmmc_info,
			sizeof(struct regulator_init_data), WCOVE_ID_VSDIO + 1);

	/* register SD card regulator for XPOWER PMIC*/
	intel_mid_pmic_set_pdata("dcovex_regulator", &dcovex_ldo2,
			sizeof(struct dcovex_regulator_info),
			DCOVEX_ID_LDO2 + 1);
	intel_mid_pmic_set_pdata("dcovex_regulator", &dcovex_gpio1,
			sizeof(struct dcovex_regulator_info),
			DCOVEX_ID_GPIO1 + 1);

	return 0;
}
rootfs_initcall(sdio_regulator_init);
