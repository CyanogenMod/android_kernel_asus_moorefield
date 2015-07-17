/*
 *  intel_mid_vibra.c - Intel vibrator driver for Clovertrail and mrfld phone
 *
 *  Copyright (C) 2011-13 Intel Corp
 *  Author: KP, Jeeja <jeeja.kp@intel.com>
 *  Author: Vinod Koul <vinod.koul@intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */


#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/acpi.h>
#include <linux/pm_runtime.h>
#include <linux/lnw_gpio.h>
#include <linux/input/intel_mid_vibra.h>
#include <trace/events/power.h>
#include <asm/intel_scu_pmic.h>
#include <linux/hrtimer.h>
#include <linux/wakelock.h>
#include <../../../../../drivers/staging/android/timed_output.h>
#include "mid_vibra.h"

extern bool is_incall;

static struct {
	struct mutex lock;
	struct work_struct work;
	struct hrtimer timer;
	struct wake_lock wklock;
	struct vibra_info *info;
} vibdata;

union sst_pwmctrl_reg {
	struct {
		u32 pwmtd:8;
		u32 pwmbu:22;
		u32 pwmswupdate:1;
		u32 pwmenable:1;
	} part;
	u32 full;
};

static int vibra_driver_write(struct i2c_adapter *adap, u8 i2c_addr,
				u8 reg, u8 value)
{
	struct i2c_msg msg;
	u8 buffer[2];
	int ret = 0;

	buffer[0] = reg;
	buffer[1] = value;
	pr_debug("write for %x, value %x", buffer[0], buffer[1]);

	msg.addr = i2c_addr;
	msg.len = 2;
	msg.buf = (u8 *)&buffer;
	msg.flags = 0;
	ret = i2c_transfer(adap, &msg, 1);
	if (ret != 1)
		pr_err("i2c write error: %d for reg %x", ret, reg);
	else
		ret = 0;
	return ret;
}

static int vibra_driver_read(struct i2c_adapter *adap, u8 i2c_addr,
				u8 reg, u8 *value)
{
	struct i2c_msg xfer[2];
	int ret = 0;

	xfer[0].addr = i2c_addr;
	xfer[0].flags = 0;
	xfer[0].len = 1;
	xfer[0].buf = (u8 *)&reg; /* write address */

	xfer[1].addr = i2c_addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = 1;
	xfer[1].buf = (u8 *)value; /*Read value */

	ret = i2c_transfer(adap, xfer, 2);
	if (ret != 2) {
		pr_err("%s:i2c transfer err:%d for reg %x", __func__, ret, reg);
	} else {
		pr_debug("Read from 0x%x, the val 0x%x\n", reg, *value);
		return 0;
	}

	return ret;
}

static int vibra_soc_pwm_configure(struct vibra_info *info, bool enable)
{
	union sst_pwmctrl_reg pwmctrl;

	if (enable) {
		lnw_gpio_set_alt(info->gpio_pwm, info->alt_fn);

		/*1. Enable the PWM by setting PWM enable bit to 1 */
		pwmctrl.full = readl(info->shim);
		pr_debug("Vibra:Read pwmctrl %x\n", readl(info->shim));
		pwmctrl.part.pwmenable = 1;
		writel(pwmctrl.full, info->shim);

		/*2. Read the PWM register to make sure there is no pending
		*update.
		*/
		pwmctrl.full = readl(info->shim);
		pr_debug("Read pwmctrl %x\n", pwmctrl.full);

		/*check pwnswupdate bit */
		if (pwmctrl.part.pwmswupdate)
			return -EBUSY;
		/*Base unit == 1*/
		pwmctrl.part.pwmswupdate = 0x1;

		/* validate values input */
		if (*info->base_unit > info->max_base_unit)
			*info->base_unit = info->max_base_unit;
		if (*info->duty_cycle > info->max_duty_cycle)
			*info->duty_cycle = info->max_duty_cycle;
		pwmctrl.part.pwmbu = *info->base_unit;
		pwmctrl.part.pwmtd = *info->duty_cycle;
		writel(pwmctrl.full,  info->shim);
		pr_debug("Read pwmctrl %x\n", pwmctrl.full);
	} else { /*disable PWM block */
		lnw_gpio_set_alt(info->gpio_pwm, 0);

		/*1. setting PWM enable bit to 0 */
		pwmctrl.full = readl(info->shim);
		pwmctrl.part.pwmenable = 0;
		writel(pwmctrl.full,  info->shim);
	}
	return 0;
}

#define VIBRAGPO_REG 0x33
#define VIBRAPWM_REG 0x32

static int vibra_pmic_pwm_configure(struct vibra_info *info, bool enable)
{
	int ret = 0;
	pr_debug("%s: %s\n", __func__, enable ? "on" : "off");
	pr_debug("%s: is_incall is %s\n", __func__, is_incall? "true" : "false"); /* do not enable vibrator when audio android mode is incall(2) */
	if (enable) {	/*enable PWM block */
#ifdef CONFIG_ZS550ML
		lnw_gpio_set_alt(info->gpio_pwm, info->alt_fn);
		ret = gpio_direction_output(info->gpio_en, 1);
		if (ret)
			printk("[VIB] write vibra_en on value failed, ret = %d\n", ret);
		ret = gpio_direction_output(info->gpio_pwm, 1);
		if (ret)
			printk("[VIB] write vibra_pwm on value failed, ret = %d\n", ret);
#else
		if(is_incall) {
			ret = intel_scu_ipc_iowrite8(VIBRAPWM_REG, 0x41);
			pr_debug("%s: enable PWM block when incall\n", __func__);
			if (ret)
				printk("[VIB] write vibra_drv3102_enable PWM duty value faild, ret = %d\n", ret);
		} else {
			ret = intel_scu_ipc_iowrite8(VIBRAGPO_REG, 0x16);
			pr_debug("%s: enable GPO block\n", __func__);
			if (ret)
				printk("[VIB] write vibra_drv3102_enable GPO value failed, ret = %d\n", ret);
		}
#endif
	} else {	/*disable PWM block */
#ifdef CONFIG_ZS550ML
		lnw_gpio_set_alt(info->gpio_pwm, 0);
		ret = gpio_direction_output(info->gpio_pwm, 0);
		if (ret)
			printk("[VIB] write vibra_pwm off value failed, ret = %d\n", ret);
		ret = gpio_direction_output(info->gpio_en, 0);
		if (ret)
			printk("[VIB] write vibra_en off value failed, ret = %d\n", ret);
#else
		pr_debug("%s: disable GPO/PWM block\n", __func__);
		ret = intel_scu_ipc_iowrite8(VIBRAGPO_REG,0x04);
		if (ret)
			printk("[VIB] write vibra_disable GPO value faild, ret = %d\n", ret);
		ret = intel_scu_ipc_iowrite8(VIBRAPWM_REG,0x40);
		if (ret)
			printk("[VIB] write vibra_disable PWM duty value faild, ret = %d\n", ret);
#endif
	}
	return 0;
}


/* Enable vibra */
static void vibra_drv2605_enable(struct vibra_info *info)
{
	pr_debug("%s: Enable", __func__);
	trace_vibrator(1);
	mutex_lock(&info->lock);
	pm_runtime_get_sync(info->dev);

	/* Enable the EN line */
	vibra_gpio_set_value(info, 1);

	/* Wait for 850us per spec, give 100us buffer */
	usleep_range(950, 1000);

	/* Enable the Trigger line */
	info->pwm_configure(info, true);

	info->enabled = true;
	mutex_unlock(&info->lock);
}

static void vibra_drv3102_enable(struct vibra_info *info)
{
	pr_debug("%s: Enable\n", __func__);
	trace_vibrator(1);
	mutex_lock(&info->lock);
//	pm_runtime_get_sync(info->dev);
	info->pwm_configure(info, true);
	info->enabled = true;
	mutex_unlock(&info->lock);
}

static void vibra_drv3102_disable(struct vibra_info *info)
{
	pr_debug("%s: Disable\n", __func__);
	trace_vibrator(0);
	mutex_lock(&info->lock);
	info->enabled = false;
	info->pwm_configure(info, false);
//	pm_runtime_put(info->dev);
	mutex_unlock(&info->lock);
}


static void vibra_disable(struct vibra_info *info)
{
	pr_debug("%s: Disable", __func__);
	trace_vibrator(0);
	mutex_lock(&info->lock);
	vibra_gpio_set_value_cansleep(info, 0);
	info->enabled = false;
	info->pwm_configure(info, false);
	pm_runtime_put(info->dev);
	mutex_unlock(&info->lock);
}

static void vibra_drv8601_enable(struct vibra_info *info)
{
	pr_debug("%s: Enable", __func__);
	trace_vibrator(1);
	mutex_lock(&info->lock);
	pm_runtime_get_sync(info->dev);
	info->pwm_configure(info, true);
	vibra_gpio_set_value_cansleep(info, 1);
	info->enabled = true;
	mutex_unlock(&info->lock);
}

/*******************************************************************************
 * SYSFS                                                                       *
 ******************************************************************************/

static ssize_t vibra_show_vibrator(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct vibra_info *info = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", info->enabled);

}

static ssize_t vibra_set_vibrator(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	long vibrator_enable;
	struct vibra_info *info = dev_get_drvdata(dev);

	if (kstrtol(buf, 0, &vibrator_enable))
		return -EINVAL;
	if (vibrator_enable == info->enabled)
		return len;
	else if (vibrator_enable == 0)
		info->disable(info);
	else if (vibrator_enable == 1)
		info->enable(info);
	else
		return -EINVAL;
	return len;
}

unsigned long mid_vibra_base_unit;
unsigned long mid_vibra_duty_cycle;

static DEVICE_ATTR(vibrator, S_IRUGO | S_IWUSR,
		   vibra_show_vibrator, vibra_set_vibrator);
static DEVICE_ULONG_ATTR(pwm_baseunit, S_IRUGO | S_IWUSR, mid_vibra_base_unit);
static DEVICE_ULONG_ATTR(pwm_ontime_div, S_IRUGO | S_IWUSR, mid_vibra_duty_cycle);

static struct attribute *vibra_attrs[] = {
	&dev_attr_vibrator.attr,
	&dev_attr_pwm_baseunit.attr.attr,
	&dev_attr_pwm_ontime_div.attr.attr,
	0,
};

static const struct attribute_group vibra_attr_group = {
	.attrs = vibra_attrs,
};

/*** Module ***/
#if CONFIG_PM
static int intel_vibra_runtime_suspend(struct device *dev)
{
	struct vibra_info *info = dev_get_drvdata(dev);

	pr_debug("In %s\n", __func__);
	mutex_lock(&info->lock);
	info->pwm_configure(info, false);
	mutex_unlock(&info->lock);
	return 0;
}

static int intel_vibra_runtime_resume(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	return 0;
}

static void intel_vibra_complete(struct device *dev) {
	pr_debug("In %s\n", __func__);
	intel_vibra_runtime_resume(dev);
}

static const struct dev_pm_ops intel_mid_vibra_pm_ops = {
	.prepare = intel_vibra_runtime_suspend,
	.complete = intel_vibra_complete,
	.runtime_suspend = intel_vibra_runtime_suspend,
	.runtime_resume = intel_vibra_runtime_resume,
};
#endif

#define MRFLD_VIBRA_BUS		0x2

/* vibra_drv2604_calibrate: initializes the ext drv and auto calibrates it one time
 *
 * @info: vibra driver context
 */
static int vibra_drv2605_calibrate(struct vibra_info *info)
{
#define DRV2605_I2C_ADDR	0x5a

#define DRV2605_STATUS		0x00
#define DRV2605_MODE		0x01
#define DRV2605_GO		0x0c
#define DRV2605_VOLTAGE		0x16
#define DRV2605_CLAMP		0x17
#define DRV2605_AUTO_CAL_COMP	0x18
#define DRV2605_FB_CONTROL	0x1a

#define DRV2605_AUTO_CALIB	0x07
#define DRV2605_2_0V		0x5b
#define DRV2605_LRA		0xa4
#define DRV2605_PWM		0x03
#define DRV2605_GO_BIT		0x01

#define DRV2605_STANDBY_BIT         6
#define DRV2605_DIAG_RESULT_BIT     3

/* default value of the register 0x18 */
#define DRV2605_AUTO_CAL_COMP_VALUE	0x0D

	struct i2c_adapter *adap;
	u8 status = 0, mode = 0;
	u8 reg_val = DRV2605_AUTO_CAL_COMP_VALUE;

	adap = i2c_get_adapter(MRFLD_VIBRA_BUS);
	if (!adap) {
		pr_err("can't find bus adapter");
		return -EIO;
	}

	/* enable gpio first */
	vibra_gpio_set_value(info, 1);
	/* wait for gpio to settle and drv to accept i2c */
	usleep_range(1000, 1100);

	/* get the device out of standby */
	vibra_driver_write(adap, DRV2605_I2C_ADDR, DRV2605_MODE, DRV2605_PWM);

	vibra_driver_read(adap, DRV2605_I2C_ADDR, DRV2605_MODE, &mode);
	/* Is Device Ready?? */
	if (!((mode >> DRV2605_STANDBY_BIT) & 0x1)) {

		vibra_driver_read(adap, DRV2605_I2C_ADDR, DRV2605_STATUS, &status);
		vibra_driver_read(adap, DRV2605_I2C_ADDR, DRV2605_AUTO_CAL_COMP, &reg_val);

		/* Is it Auto Calibrated?? */
		/* Check the diag result bit & default value of auto calib comp register */
		if (!((status >> DRV2605_DIAG_RESULT_BIT) & 0x1) &&
			(reg_val != DRV2605_AUTO_CAL_COMP_VALUE)) {

			vibra_gpio_set_value(info, 0);
			pr_debug("Do Nothing -  Device Calibrated\n");
			return 0;
		}
	}

	pr_warn("Calibrating the vibra..\n");
	/*put device in auto calibrate mode*/
	vibra_driver_write(adap, DRV2605_I2C_ADDR, DRV2605_MODE, DRV2605_AUTO_CALIB);
	vibra_driver_write(adap, DRV2605_I2C_ADDR, DRV2605_FB_CONTROL, DRV2605_LRA);
	vibra_driver_write(adap, DRV2605_I2C_ADDR, DRV2605_VOLTAGE, DRV2605_2_0V);
	vibra_driver_write(adap, DRV2605_I2C_ADDR, DRV2605_CLAMP, DRV2605_2_0V);
	vibra_driver_write(adap, DRV2605_I2C_ADDR, DRV2605_GO, DRV2605_GO_BIT);

	/* wait for auto calibration to complete
	 * polling of driver does not work
	 */
	msleep(1000);
	/* set the driver in pwm mode */
	vibra_driver_write(adap, DRV2605_I2C_ADDR, DRV2605_MODE, DRV2605_PWM);
	vibra_gpio_set_value(info, 0);

	return 0;
}

struct vibra_info *mid_vibra_setup(struct device *dev, struct mid_vibra_pdata *data)
{
	struct vibra_info *info;

	pr_debug("probe data divisor %x, base %x, alt_fn %d ext_drv %d, name: %s",
		data->time_divisor, data->base_unit, data->alt_fn, data->ext_drv, data->name);

	info =  devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info) {
		pr_err("%s: no memory for driver context", __func__);
		return NULL;
	}

	info->alt_fn = data->alt_fn;
	info->ext_drv = data->ext_drv;
	info->gpio_en = data->gpio_en;
	info->gpio_pwm = data->gpio_pwm;
	info->name = data->name;
	info->use_gpio_en = data->use_gpio_en;

	info->dev = dev;
	mutex_init(&info->lock);
	info->vibra_attr_group = &vibra_attr_group;

	mid_vibra_base_unit = data->base_unit;
	mid_vibra_duty_cycle = data->time_divisor;
	info->base_unit = &mid_vibra_base_unit;
	info->duty_cycle = &mid_vibra_duty_cycle;

	/* ops */
	if (!strncmp(info->name, "drv8601", 8)) {
		info->enable = vibra_drv8601_enable;
	} else if (!strncmp(info->name, "drv2605", 8)) {
		info->enable = vibra_drv2605_enable;
	} else if (!strncmp(info->name, "drv2603", 8)) {
		info->enable = vibra_drv2605_enable;
	} else if (!strncmp(info->name, "VIBR22A8", 8)) {
	/* The CHT vibra does not use any driver chip. It does not use "enable"
	   GPIO either. However, the existing vibra_drv8601_enable() and
	   vibra_disable() functions  can be reused on CHT since gpio apis
	   are not called when enable gpio control is not used */
		info->enable = vibra_drv8601_enable;
	} else if (!strncmp(info->name, "drv3102", 8)) {
		info->enable = vibra_drv3102_enable;
	} else {
		pr_err("%s: unsupported vibrator device", __func__);
		return NULL;
	}

	if (!strncmp(info->name, "drv3102", 8))
		info->disable = vibra_drv3102_disable;
	else
		info->disable = vibra_disable;

	vibdata.info = info;

	return info;
}

#define VIBRAPWM_CTRL 0x36
static void vibrator_on(void)
{
	wake_lock(&vibdata.wklock);
	vibdata.info->enable(vibdata.info);
}

static void vibrator_off(void)
{
	wake_unlock(&vibdata.wklock);
	vibdata.info->disable(vibdata.info);
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
	if (hrtimer_active(&vibdata.timer)) {
		ktime_t r = hrtimer_get_remaining(&vibdata.timer);
		return ktime_to_ms(r);
	}

	return 0;
}

static void vibrator_enable(struct timed_output_dev *dev, int value)
{
	mutex_lock(&vibdata.lock);

	hrtimer_cancel(&vibdata.timer);
	cancel_work_sync(&vibdata.work);

	if (value) {
		vibrator_on();
		hrtimer_start(&vibdata.timer, ns_to_ktime((u64) value * NSEC_PER_MSEC), HRTIMER_MODE_REL);
	} else
		vibrator_off();

	mutex_unlock(&vibdata.lock);
}

static struct timed_output_dev to_dev = {
	.name = "vibrator",
	.get_time = vibrator_get_time,
	.enable = vibrator_enable,
};

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	schedule_work(&vibdata.work);
	return HRTIMER_NORESTART;
}

static void vibrator_work(struct work_struct *work)
{
	vibrator_off();
}

static void timed_output_subclass_init(void)
{
	hrtimer_init(&vibdata.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vibdata.timer.function = vibrator_timer_func;
	INIT_WORK(&vibdata.work, vibrator_work);
	wake_lock_init(&vibdata.wklock, WAKE_LOCK_SUSPEND, "vibrator");
	mutex_init(&vibdata.lock);

	if (timed_output_dev_register(&to_dev) < 0) {
		pr_err("%s: fail to create timed output dev\n", __func__);
	}
}

static int intel_mid_vibra_probe(struct pci_dev *pci,
			const struct pci_device_id *pci_id)
{
	struct vibra_info *info;
	struct device *dev = &pci->dev;
	struct mid_vibra_pdata *data;
	int ret;

	pr_debug("Probe for DID %x\n", pci->device);

	data = pci->dev.platform_data;
	if (!data) {
		dev_err(&pci->dev, "Failed to get vibrator platform data\n");
		return -ENODEV;
	}

	info = mid_vibra_setup(dev, data);
	if (!info)
		return -ENODEV;

/* disable SOC control
	info->pwm_configure = vibra_soc_pwm_configure;

	info->max_base_unit = INTEL_VIBRA_MAX_BASEUNIT;
	info->max_duty_cycle = INTEL_VIBRA_MAX_TIMEDIVISOR;
*/
	info->pwm_configure = vibra_pmic_pwm_configure;

	if (info->use_gpio_en) {
		pr_debug("using gpios en: %d, pwm %d",
				info->gpio_en, info->gpio_pwm);
		ret = gpio_request_one(info->gpio_en, GPIOF_DIR_OUT,
				"VIBRA ENABLE");
		if (ret != 0) {
			pr_err("gpio_request(%d) fails:%d\n",
					info->gpio_en, ret);
			goto out;
		}

		ret = gpio_request_one(info->gpio_pwm, GPIOF_DIR_OUT,
				"VIBRA PWM");
		if (ret != 0) {
			pr_err("gpio_request(%d) fails:%d\n",
					info->gpio_pwm, ret);
			goto out;
		}
	}

	/* Init the device */
	ret = pci_enable_device(pci);
	if (ret) {
		pr_err("device can't be enabled\n");
		goto do_freegpio_vibra_enable;
	}
	ret = pci_request_regions(pci, INTEL_VIBRA_DRV_NAME);
	if (ret)
		goto do_disable_device;
	pci_dev_get(pci);

	/* vibra Shim */
	info->shim =  pci_ioremap_bar(pci, 0);
	if (!info->shim) {
		pr_err("ioremap failed for vibra driver\n");
		goto do_release_regions;
	}
	pr_debug("Base reg: %#x", (unsigned int) pci_resource_start(pci, 0));

	ret = sysfs_create_group(&dev->kobj, info->vibra_attr_group);
	if (ret) {
		pr_err("could not register sysfs files\n");
		goto do_unmap_shim;
	}

	if (info->ext_drv)
		vibra_drv2605_calibrate(info);

	ret = intel_scu_ipc_iowrite8(VIBRAPWM_CTRL, 0x78);
	if (ret)
		pr_err("write vibra_drv3102_enable duty value faild\n");

	pci_set_drvdata(pci, info);
	pm_runtime_allow(&pci->dev);
	pm_runtime_put_noidle(&pci->dev);

	timed_output_subclass_init();

	return ret;

do_unmap_shim:
	iounmap(info->shim);
do_release_regions:
	pci_release_regions(pci);
do_disable_device:
	pci_disable_device(pci);
do_freegpio_vibra_enable:
	vibra_gpio_free(info);
out:
	return ret;
}

static void intel_mid_vibra_remove(struct pci_dev *pci)
{
	struct vibra_info *info = pci_get_drvdata(pci);
	vibra_gpio_free(info);
	sysfs_remove_group(&info->dev->kobj, info->vibra_attr_group);
	iounmap(info->shim);
	pci_release_regions(pci);
	pci_disable_device(pci);
	pci_set_drvdata(pci, NULL);
}

/* PCI Routines */
static DEFINE_PCI_DEVICE_TABLE(intel_vibra_ids) = {
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_VIBRA_CLV), 0},
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_VIBRA_MRFLD), 0},
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_VIBRA_MOOR), 0},
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, intel_vibra_ids);

static struct pci_driver vibra_driver = {
	.name = INTEL_VIBRA_DRV_NAME,
	.id_table = intel_vibra_ids,
	.probe = intel_mid_vibra_probe,
	.remove = intel_mid_vibra_remove,
#ifdef CONFIG_PM
	.driver = {
		.pm = &intel_mid_vibra_pm_ops,
	},
#endif
};

static const struct acpi_device_id vibra_acpi_ids[];

void *mid_vibra_acpi_get_drvdata(const char *hid)
{
	const struct acpi_device_id *id;

	for (id = vibra_acpi_ids; id->id[0]; id++)
		if (!strncmp(id->id, hid, 16))
			return (void *)id->driver_data;
	return 0;
}

static const struct acpi_device_id vibra_acpi_ids[] = {
	{ "VIB8601", (kernel_ulong_t) &pmic_vibra_data_byt_ffrd8 },
	{ "VIBR22A8", (kernel_ulong_t) &pmic_vibra_data_cht },
	{},
};
MODULE_DEVICE_TABLE(acpi, vibra_acpi_ids);

static struct platform_driver plat_vibra_driver = {
	.driver = {
		.name = "intel_mid_pmic_vibra",
		.owner = THIS_MODULE,
		.acpi_match_table = ACPI_PTR(vibra_acpi_ids),
#ifdef CONFIG_PM
		.pm = &intel_mid_vibra_pm_ops,
#endif
	},
	.probe = intel_mid_plat_vibra_probe,
	.remove = intel_mid_plat_vibra_remove,
};

/**
* intel_mid_vibra_init - Module init function
*
* Registers with PCI
* Registers platform
* Init all data strutures
*/
static int __init intel_mid_vibra_init(void)
{
	int ret = 0;

	/* Register with PCI */
	ret = pci_register_driver(&vibra_driver);
	if (ret)
		pr_err("PCI register failed\n");

	ret = platform_driver_register(&plat_vibra_driver);
	if (ret)
		pr_err("Platform register failed\n");

	return ret;
}

/**
* intel_mid_vibra_exit - Module exit function
*
* Unregisters with PCI
* Unregisters platform
* Frees all data strutures
*/
static void __exit intel_mid_vibra_exit(void)
{
	pci_unregister_driver(&vibra_driver);
	platform_driver_unregister(&plat_vibra_driver);
	pr_debug("intel_mid_vibra driver exited\n");
	return;
}

late_initcall(intel_mid_vibra_init);
module_exit(intel_mid_vibra_exit);

MODULE_ALIAS("pci:intel_mid_vibra");
MODULE_DESCRIPTION("Intel(R) MID Vibra driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("KP Jeeja <jeeja.kp@intel.com>");
