/*
 * cyttsp5_platform.c
 * Cypress TrueTouch(TM) Standard Product V5 Platform Module.
 * For use with Cypress Txx5xx parts.
 * Supported parts include:
 * TMA5XX
 *
 * Copyright (C) 2013 Cypress Semiconductor
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Contact Cypress Semiconductor at www.cypress.com <ttdrivers@cypress.com>
 *
 */

#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>

/* cyttsp */
#include <linux/cyttsp5_bus.h>
#include <linux/cyttsp5_core.h>
#include <linux/cyttsp5_platform.h>

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_FW_UPGRADE
#include "cyttsp5_fw.h"
static struct cyttsp5_touch_firmware cyttsp5_firmware = {
	.img = cyttsp4_img,
	.size = ARRAY_SIZE(cyttsp4_img),
	.ver = cyttsp4_ver,
	.vsize = ARRAY_SIZE(cyttsp4_ver),
};
#else
static struct cyttsp5_touch_firmware cyttsp5_firmware = {
	.img = NULL,
	.size = 0,
	.ver = NULL,
	.vsize = 0,
};
#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_TTCONFIG_UPGRADE
#include "cyttsp5_params.h"
static struct touch_settings cyttsp5_sett_param_regs = {
	.data = (uint8_t *)&cyttsp4_param_regs[0],
	.size = ARRAY_SIZE(cyttsp4_param_regs),
	.tag = 0,
};

static struct touch_settings cyttsp5_sett_param_size = {
	.data = (uint8_t *)&cyttsp4_param_size[0],
	.size = ARRAY_SIZE(cyttsp4_param_size),
	.tag = 0,
};

static struct cyttsp5_touch_config cyttsp5_ttconfig = {
	.param_regs = &cyttsp5_sett_param_regs,
	.param_size = &cyttsp5_sett_param_size,
	.fw_ver = ttconfig_fw_ver,
	.fw_vsize = ARRAY_SIZE(ttconfig_fw_ver),
};
#else
static struct cyttsp5_touch_config cyttsp5_ttconfig = {
	.param_regs = NULL,
	.param_size = NULL,
	.fw_ver = NULL,
	.fw_vsize = 0,
};
#endif

struct cyttsp5_loader_platform_data _cyttsp5_loader_platform_data = {
	.fw = &cyttsp5_firmware,
	.ttconfig = &cyttsp5_ttconfig,
	.flags = CY_LOADER_FLAG_NONE,
};
static struct regulator *ldo3_reg;

int cyttsp5_xres(struct cyttsp5_core_platform_data *pdata,
		struct device *dev)
{
	int rst_gpio = pdata->rst_gpio;
	int rc = 0;
	/* RESET regulator first if needed */
	if (ldo3_reg == NULL) {
		dev_info(dev, "getting regulator in xres \n");
		ldo3_reg = regulator_get(dev, "LDO3");
		if (IS_ERR(ldo3_reg)) {
			dev_err(dev, "Failed to get regulator LDO3");
			rc = PTR_ERR(ldo3_reg);
			ldo3_reg = NULL;
		} else {
			msleep(200);
			dev_info(dev, "enabling regulator in xres \n");
			rc = regulator_enable(ldo3_reg);
		}
		if (rc) {
			dev_err(dev,
				"%s: Failed to enable LDO3 in xres\n", __func__);
		}
	}

	/* RESET GPIO */
	gpio_set_value(rst_gpio, 1);
	msleep(20);
	gpio_set_value(rst_gpio, 0);
	msleep(40);
	gpio_set_value(rst_gpio, 1);
	msleep(20);
	dev_info(dev,
		"%s: RESET CYTTSP gpio=%d r=%d\n", __func__,
		pdata->rst_gpio, rc);
	return rc;
}

int cyttsp5_init(struct cyttsp5_core_platform_data *pdata,
		int on, struct device *dev)
{
	int rst_gpio = pdata->rst_gpio;
	int irq_gpio = pdata->irq_gpio;
	int rc = 0;

	if (ldo3_reg == NULL) {
		ldo3_reg = regulator_get(dev, "LDO3");
		if (IS_ERR(ldo3_reg)) {
			dev_err(dev, "Failed to get regulator LDO3");
			rc = PTR_ERR(ldo3_reg);
			ldo3_reg = NULL;
			goto out;
		}
	}

	if (on) {
		rc = regulator_enable(ldo3_reg);
		if (rc) {
			dev_err(dev,
				"%s: Failed to enable LDO3\n", __func__);
			goto out;
		}
		rc = gpio_request(rst_gpio, NULL);
		if (rc < 0) {
			gpio_free(rst_gpio);
			rc = gpio_request(rst_gpio, NULL);
		}
		if (rc < 0) {
			dev_err(dev,
				"%s: Fail request gpio=%d\n", __func__,
				rst_gpio);
		} else {
			rc = gpio_direction_output(rst_gpio, 1);
			if (rc < 0) {
				pr_err("%s: Fail set output gpio=%d\n",
					__func__, rst_gpio);
				gpio_free(rst_gpio);
			} else {
				rc = gpio_request(irq_gpio, NULL);
				if (rc < 0) {
					gpio_free(irq_gpio);
					rc = gpio_request(irq_gpio,
						NULL);
				}
				if (rc < 0) {
					dev_err(dev,
						"%s: Fail request gpio=%d\n",
						__func__, irq_gpio);
					gpio_free(rst_gpio);
				} else {
					gpio_direction_input(irq_gpio);
				}
			}
		}
	} else {
		gpio_free(rst_gpio);
		gpio_free(irq_gpio);
		/* the following should turn off 2.85V iff we are only user */
		rc = regulator_disable(ldo3_reg);
		if (rc) {
			dev_err(dev, "%s: Failed to disable LDO3\n",
				__func__);
		}
	}

out:
	dev_info(dev,
		"%s: INIT CYTTSP RST gpio=%d and IRQ gpio=%d r=%d\n",
		__func__, rst_gpio, irq_gpio, rc);
	return rc;
}

static int cyttsp5_wakeup(struct cyttsp5_core_platform_data *pdata,
		struct device *dev, atomic_t *ignore_irq)
{
	int irq_gpio = pdata->irq_gpio;
	int rc = 0;

	if (ignore_irq)
		atomic_set(ignore_irq, 1);
	rc = gpio_direction_output(irq_gpio, 0);
	if (rc < 0) {
		if (ignore_irq)
			atomic_set(ignore_irq, 0);
		dev_err(dev,
			"%s: Fail set output gpio=%d\n",
			__func__, irq_gpio);
	} else {
		udelay(2000);
		rc = gpio_direction_input(irq_gpio);
		if (ignore_irq)
			atomic_set(ignore_irq, 0);
		if (rc < 0) {
			dev_err(dev,
				"%s: Fail set input gpio=%d\n",
				__func__, irq_gpio);
		}
	}

	dev_info(dev,
		"%s: WAKEUP CYTTSP gpio=%d r=%d\n", __func__,
		irq_gpio, rc);
	return rc;
}

static int cyttsp5_sleep(struct cyttsp5_core_platform_data *pdata,
		struct device *dev, atomic_t *ignore_irq)
{
	return 0;
}

int cyttsp5_power(struct cyttsp5_core_platform_data *pdata,
		int on, struct device *dev, atomic_t *ignore_irq)
{
	if (on)
		return cyttsp5_wakeup(pdata, dev, ignore_irq);

	return cyttsp5_sleep(pdata, dev, ignore_irq);
}

int cyttsp5_irq_stat(struct cyttsp5_core_platform_data *pdata,
		struct device *dev)
{
	return gpio_get_value(pdata->irq_gpio);
}
