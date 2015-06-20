/*
 * intel_mid_vdd.c - Intel Clovertrail Platform Voltage Drop Detection Driver
 *
 *
 * Copyright (C) 2012 Intel Corporation
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author: Durgadoss R <durgadoss.r@intel.com>
 *         Chaurasia, Avinash K <avinash.k.chaurasia@intel.com>
 *
 * This driver monitors the voltage level of the system. When the system
 * voltage cross the programmed threshold, it notifies the CPU of the
 * change. Also, the driver configures the HW to take some actions to prevent
 * system crash due to sudden drop in voltage. The HW unit that does all
 * these, is named as Burst Control Unit(BCU).
 */


#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/hwmon-sysfs.h>
#include <linux/hwmon.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/timer.h>
#include <linux/power_supply.h>
#include <asm/intel_scu_ipc.h>
#include <asm/intel_scu_pmic.h>
#include <linux/spinlock.h>
#include <linux/ratelimit.h>
#include <linux/mfd/intel_mid_pmic.h>
#include <asm/intel-mid.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/mfd/intel_msic.h>

#ifndef CONFIG_ACPI

 #define DRIVER_NAME	"msic_vdd"
 #define DEVICE_NAME	"msic_vdd"

 #define OFFSET		0x109
 #define BCUIRQ		0x00A
 #define MBCUIRQ	0x019

 #define VWARNA_THRES		0x02	/* 3.1V */
 #define VWARNB_THRES		0x03	/* 3.0V */
 #define VWARNCRIT_THRES	0x04	/* 2.9V */

#else

 #define DRIVER_NAME	"crystal_cove_bcu"
 #define DEVICE_NAME	"crystal_cove_bcu"

 #define OFFSET		0x0B4
 #define BCUIRQ		0x007
 #define MBCUIRQ	0x014

 #define VWARNA_THRES		0x06 /*3.0V*/
 #define VWARNB_THRES		0x04 /*2.9V*/
 #define VWARNCRIT_THRES	0x06 /*2.7V*/

#endif

#define VWARNA_CFG	(OFFSET + 0)
#define VWARNB_CFG	(OFFSET + 1)
#define VCRIT_CFG	(OFFSET + 2)
#define BCUDISA_BEH	(OFFSET + 3)
#define BCUDISB_BEH	(OFFSET + 4)
#define BCUDISCRIT_BEH	(OFFSET + 5)
#define BCUPROCHOT_BEH	(OFFSET + 6)

#define SBCUIRQ		(OFFSET + 7)
#define SBCUCTRL	(OFFSET + 8)

/* Level 1 PMIC IRQ register */
#define MIRQLVL1	0x0E

#define VWARNA_VOLT_THRES	VWARNA_THRES
#define VWARNB_VOLT_THRES	VWARNB_THRES
#define VCRIT_VOLT_THRES	VWARNCRIT_THRES

#define SBCUDISCRIT_MASK	0x02
#define SBCUDISB_MASK		0x04
#define SBCUDISA_MASK		0x08
#define SPROCHOT_B_MASK		0x01

/* bcu control pin status register set */
#define SBCUCTRL_SET		0x0F

/* BCUIRQ register settings */
#define VCRIT_IRQ		(1 << 2)
#define VWARNA_IRQ		(1 << 1)
#define VWARNB_IRQ		(1 << 0)

/* Enable BCU voltage comparator */
#define VCOMP_ENABLE		(1 << 3)

/* BCU real time status flags for corresponding signals */
#define SVWARNB			(1<<0)
#define SVWARNA			(1<<1)
#define SVCRIT			(1<<2)

/* curently 20 fast clock is set */
#define DEBOUNCE		(0x70)

/*
 * this flag has been used in lot of places to update the proper bit
 * in registers
 */
#define FLAGS_THRES_REGS	0xF7

/* unmask the 2nd level interrupts as by default they are masked */
#define UNMASK_MBCUIRQ		0x00

/* check whether bit is sticky or not by checking 3rd bit */
#define IS_STICKY(data)		(data & 0x04)

#define SET_ACTION_MASK(data, value, bit) (data | (value << bit))

/* This macro is used in hardcoding of gpio */
#define MSIC_VDD		0x24

/* defines reading BCU registers from SRAM */
#define MSIC_BCU_STAT		0xFFFFEFC8
#define MSIC_BCU_LEN		1

#define BCU_SMIP_OFFSET		0x5A5

/* voltage threshold limits in CTP*/
#define MAX_VOLTAGE             3300
#define MIN_VOLTAGE             2600

/* mask level 2 interrupt register set */
#define MBCUIRQ_SET		0x07

/* Level 1 bcu irq mask */
#define MIRQLVL1_BCU_MASK	(1<<2)

/* interrupt bit masks for MBCUIRQ reg*/
#define VCRIT_MASK		0x04
#define VWARNA_MASK		0x02
#define VWARNB_MASK		0x01

/* Comparator enable-disable  bit mask */
#define VWARN_CFG_MASK		0x08
#define VCRITSDWNEN_MASK	0x10

/* O/P signal enable-disable mask */
#define BCUDIS_MASK		0x01
#define PROCHOT_B_MASK		0x03

/* Vsys status bit mask for SBCUIRQ register */
#define SBCUIRQ_MASK		(VCRIT_MASK | VWARNA_MASK | VWARNB_MASK)

/* default values for register configuration */
#define INIT_VWARNA		(VWARNA_VOLT_THRES | DEBOUNCE | VCOMP_ENABLE)
#define INIT_VWARNB		(VWARNB_VOLT_THRES | DEBOUNCE | VCOMP_ENABLE)
#define INIT_VCRIT		(VCRIT_VOLT_THRES | DEBOUNCE | VCOMP_ENABLE)
#define INIT_BCUDISA		0x05
#define INIT_BCUDISB		0x05
#define INIT_BCUISCRIT		0x00
#define INIT_BCUPROCHOT		0x06
#define	INIT_MBCU		0x00

#define BCU_QUEUE		"bcu-queue"
#define BASE_TIME		30
#define STEP_TIME		15
#define VWARNA_INTR_EN_DELAY	(30 * HZ)

/* Generic macro and string to send the
 * uevent along with env to userspace
 */
#define EVT_STR			"BCUEVT="
#define GET_ENVP(EVT)		(EVT_STR#EVT)

#define CAMFLASH_STATE_NORMAL       0
#define CAMFLASH_STATE_CRITICAL      3

/* Defined to match the corresponding bit positions of the interrupt */
enum { VWARNB_EVENT = 1, VWARNA_EVENT = 2, VCRIT_EVENT = 4};

static DEFINE_MUTEX(vdd_update_lock);
static DEFINE_SPINLOCK(vdd_interrupt_lock);

static struct workqueue_struct *bcu_workqueue;
static void unmask_theburst(struct work_struct *work);
static struct intel_msic_vdd_pdata *pdata;

struct vdd_smip_data {
	u8 vwarna_cfg;
	u8 vwarnb_cfg;
	u8 vcrit_cfg;
	u8 bcudisa_beh;
	u8 bcudisb_beh;
	u8 bcudiscrit_beh;
	u8 bcuprochot_beh;
	u8 mbcu_irq;
	/* 12 bits are reserved for bcu*/
};

struct vdd_info {
	unsigned int irq;
	uint32_t intr_count_lvl1;
	uint32_t intr_count_lvl2;
	uint32_t intr_count_lvl3;
	struct device *dev;
	struct platform_device *pdev;
	/* mapping SRAM address for BCU interrupts */
	void __iomem *bcu_intr_addr;
	unsigned int delay;
	u64 seed_time;
	struct delayed_work vdd_intr_dwork;
	struct vdd_smip_data init_reg_data;
};

static uint8_t cam_flash_state;

static uint8_t global_irq_data;

/**
  * vdd_set_bits - set bit in register corresponding to mask value
  * @addr - address of register where changes are to be done
  * @mask - mask value according to which value will be set
  */
static inline int vdd_set_bits(u16 addr, u8 mask)
{
	return intel_scu_ipc_update_register(addr, 0xff, mask);
}

/**
  * vdd_set_register - initialize register with value
  * @addr - address of register
  * @value - value to be intialized with
  */
static inline int vdd_set_register(u16 addr, u8 value)
{
	return intel_scu_ipc_update_register(addr, value, 0xff);
}

/* --- Sysfs Implementation --- */
static ssize_t store_action_mask(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	long assert_pin;
	if (kstrtol(buf, 10, &assert_pin))
		return -EINVAL;

	/*
	 * 0x1F represents five output mask bit for 4 different registers
	 * when value is 0x1F then all the output pins will be masked and
	 * and when its 0x00 then it means all the output pins are unmasked
	 * and if interrupt occurs then all the output pins will be trigerred
	 * according to settings
	 */
	if (assert_pin < 0x00 || assert_pin > 0x1F)
		return -EINVAL;

	mutex_lock(&vdd_update_lock);
	ret = intel_scu_ipc_update_register(BCUDISA_BEH,
		((assert_pin >> 0) & 1), 0x01);
	if (ret)
		goto mask_ipc_fail;
	ret = intel_scu_ipc_update_register(BCUDISB_BEH,
		((assert_pin >> 1) & 1), 0x01);
	if (ret)
		goto mask_ipc_fail;
	ret = intel_scu_ipc_update_register(BCUDISCRIT_BEH,
		((assert_pin >> 2) & 1), 0x01);
	if (ret)
		goto mask_ipc_fail;
	ret = intel_scu_ipc_update_register(BCUPROCHOT_BEH,
		((assert_pin >> 3) & 1)<<1, 0x02);
	if (ret)
		goto mask_ipc_fail;
	ret = intel_scu_ipc_update_register(BCUPROCHOT_BEH,
		((assert_pin >> 4) & 1), 0x01);
	if (ret)
		goto mask_ipc_fail;
	ret = count;
mask_ipc_fail:
	mutex_unlock(&vdd_update_lock);
	return ret;

}

static ssize_t show_action_mask(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	uint8_t data;
	long assert_pin = 0;

	mutex_lock(&vdd_update_lock);
	ret = intel_scu_ipc_ioread8(BCUDISA_BEH, &data);
	if (ret)
		goto mask_ipc_fail;
	assert_pin = SET_ACTION_MASK(assert_pin, (data & 1), 0);

	ret = intel_scu_ipc_ioread8(BCUDISB_BEH, &data);
	if (ret)
		goto mask_ipc_fail;
	assert_pin = SET_ACTION_MASK(assert_pin, (data & 1), 1);

	ret = intel_scu_ipc_ioread8(BCUDISCRIT_BEH, &data);
	if (ret)
		goto mask_ipc_fail;
	assert_pin = SET_ACTION_MASK(assert_pin, (data & 1), 2);

	ret = intel_scu_ipc_ioread8(BCUPROCHOT_BEH, &data);
	if (ret)
		goto mask_ipc_fail;
	assert_pin = SET_ACTION_MASK(assert_pin, ((data>>1) & 1), 3);

	ret = intel_scu_ipc_ioread8(BCUPROCHOT_BEH, &data);
	if (ret)
		goto mask_ipc_fail;
	assert_pin = SET_ACTION_MASK(assert_pin, (data & 1), 4);
mask_ipc_fail:
	mutex_unlock(&vdd_update_lock);
	return sprintf(buf, "%ld\n", assert_pin);
}

static ssize_t store_bcu_status(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	long bcu_enable;

	if (kstrtol(buf, 10, &bcu_enable))
		return -EINVAL;
	if (bcu_enable != 0 &&  bcu_enable != 1)
		return -EINVAL;

	mutex_lock(&vdd_update_lock);

	/*
	 * bcu_enable = 1 means enable bcu, to do this it need to set 3rd
	 * bit from right side so mask will be 0x08
	 */
	/*
	 * pdata must be checked as pdata came as null in some platform
	 */
	if ((!pdata) || !(pdata->disable_unused_comparator & DISABLE_VWARNA)) {
		ret = intel_scu_ipc_update_register(VWARNA_CFG,
			(bcu_enable << 3), 0x08);
		if (ret)
			goto bcu_ipc_fail;
	}
	if ((!pdata) || !(pdata->disable_unused_comparator & DISABLE_VWARNB)) {
		ret = intel_scu_ipc_update_register(VWARNB_CFG,
			(bcu_enable << 3), 0x08);
		if (ret)
			goto bcu_ipc_fail;
	}
	if ((!pdata) || !(pdata->disable_unused_comparator & DISABLE_VCRIT)) {
		ret = intel_scu_ipc_update_register(VCRIT_CFG,
			(bcu_enable << 3), 0x08);
		if (ret)
			goto bcu_ipc_fail;
	}
	ret = count;
bcu_ipc_fail:
	mutex_unlock(&vdd_update_lock);
	return ret;
}

static ssize_t show_bcu_status(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret, bcu_enable;
	uint8_t data;

	ret = intel_scu_ipc_ioread8(VWARNA_CFG, &data);
	if (ret)
		return ret;
	/* if BCU is enabled then return value will be 1 else 0 */
	bcu_enable = ((data >> 3) & 0x01);
	return sprintf(buf, "%d\n", bcu_enable);
}

static ssize_t store_volt_thres(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	uint8_t data;
	long volt;
	long volt_limit_offset = 0;
	struct sensor_device_attribute_2 *s_attr =
					to_sensor_dev_attr_2(attr);

	if (kstrtol(buf, 10, &volt))
		return -EINVAL;
	/*
	 * In case of baytrail, VWARNA's range is from
	 * 3.6 volt to 2.9 volt.For this we set
	 * volt_limit_offset to 300mV.
	 * It remains zero for all other cases. i.e. Cases
	 * were the range is from 3.3 volt to 2.6 volt.
	 */
	if ((!pdata->is_clvp) && (!s_attr->nr))
		volt_limit_offset = 300;

	if (volt > (MAX_VOLTAGE + volt_limit_offset) ||
			volt < (MIN_VOLTAGE + volt_limit_offset))
		return -EINVAL;

	mutex_lock(&vdd_update_lock);

	ret = intel_scu_ipc_ioread8(VWARNA_CFG + s_attr->nr, &data);
	if (ret)
		goto ipc_fail;

	/*
	 * The VWARN*_CFG registers hold 7 voltage values, in [0-2]
	 * bits. 3.3v corresponds to 0 and 2.6v corresponds to 7. So,
	 * find the difference(diff) from MAX_VOLTAGE and divide it by
	 * 100(since the values are entered as mV). Then, set bits
	 * [0-2] to 'diff'
	 */
	data = (data & 0xF8) | (((MAX_VOLTAGE + volt_limit_offset)
				- volt)/100);

	ret = intel_scu_ipc_iowrite8(VWARNA_CFG + s_attr->nr, data);
	if (ret)
		goto ipc_fail;

	ret = count;

ipc_fail:
	mutex_unlock(&vdd_update_lock);
	return ret;
}

static ssize_t show_volt_thres(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret, volt;
	uint8_t data;
	long volt_limit_offset = 0;
	struct sensor_device_attribute_2 *s_attr =
					to_sensor_dev_attr_2(attr);

	ret = intel_scu_ipc_ioread8(VWARNA_CFG + s_attr->nr, &data);
	if (ret)
		return ret;

	if ((!pdata->is_clvp) && (!s_attr->nr))
		volt_limit_offset = 300;

	/* Read bits [0-2] of data and multiply by 100(for mV) */
	volt = (data & 0x07) * 100;

	return sprintf(buf, "%d\n", (MAX_VOLTAGE + volt_limit_offset) - volt);
}

static ssize_t show_irq_status(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	uint8_t irq_status;

	ret = intel_scu_ipc_ioread8(SBCUIRQ, &irq_status);
	if (ret)
		return ret;

	return sprintf(buf, "%x\n", irq_status);
}

static ssize_t show_action_status(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	uint8_t action_status;
	int ret;

	ret = intel_scu_ipc_ioread8(SBCUCTRL, &action_status);
	if (ret)
		return ret;

	return sprintf(buf, "%x\n", action_status);
}

static ssize_t show_intr_count(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	uint32_t value;
	int level = to_sensor_dev_attr(attr)->index;
	struct vdd_info *vinfo = dev_get_drvdata(dev);

	if (vinfo == NULL) {
		dev_err(dev, "Unable to get driver private data.\n");
		return -EIO;
	}

	switch (level) {
	case VWARNA_EVENT:
		value = vinfo->intr_count_lvl1;
		break;
	case VWARNB_EVENT:
		value = vinfo->intr_count_lvl2;
		break;
	case VCRIT_EVENT:
		value = vinfo->intr_count_lvl3;
		break;
	default:
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", value);
}

static ssize_t store_camflash_ctrl(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	uint8_t value;
	if (kstrtou8(buf, 10, &value))
		return -EINVAL;

	if ((value < CAMFLASH_STATE_NORMAL) ||
		(value > CAMFLASH_STATE_CRITICAL))
		return -EINVAL;

	cam_flash_state = value;
	return count;
}

static ssize_t show_camflash_ctrl(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", cam_flash_state);
}

static void unmask_theburst(struct work_struct *work)
{
	int ret;
	uint8_t irq_data;
	struct delayed_work *dwork = to_delayed_work(work);
	struct vdd_info *vinfo = container_of(dwork, struct vdd_info,
				 vdd_intr_dwork);
	if (pdata->is_clvp) {
		intel_scu_ipc_update_register(MBCUIRQ,
				~MBCUIRQ_SET, VCRIT_MASK);
		vinfo->seed_time = jiffies_64;
	} else {
		intel_scu_ipc_update_register(MBCUIRQ,
				~MBCUIRQ_SET, VWARNA_MASK);
		ret = intel_scu_ipc_ioread8(SBCUIRQ, &irq_data);
		if (ret) {
			dev_warn(&vinfo->pdev->dev,
				"Read VSYS flag failed\n");
		} else {
			/* Clear the BCUDISA and PROCHOT_B signal
			 * if Vsys is above VWARNA threshold.
			 */
			if (!(irq_data & SVWARNA))
				intel_scu_ipc_update_register(SBCUCTRL,
					SBCUCTRL_SET, SBCUDISA_MASK);
				intel_scu_ipc_update_register(SBCUCTRL,
					SBCUCTRL_SET, SPROCHOT_B_MASK);
		}
	}
}

static inline struct power_supply *get_psy_battery(void)
{
	struct class_dev_iter iter;
	struct device *dev;
	static struct power_supply *pst;

	class_dev_iter_init(&iter, power_supply_class, NULL, NULL);
	while ((dev = class_dev_iter_next(&iter))) {
		pst = (struct power_supply *)dev_get_drvdata(dev);
		if (pst->type == POWER_SUPPLY_TYPE_BATTERY) {
			class_dev_iter_exit(&iter);
			return pst;
		}
	}
	class_dev_iter_exit(&iter);

	return NULL;
}

/**
 * Initiate Graceful Shutdown by setting the SOC to 0% via battery driver and
 * post the power supply changed event to indicate the change in battery level.
 */
static inline int bcu_action_voltage_drop(void)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int ret;

	psy = get_psy_battery();
	if (!psy)
		return -EINVAL;

	/* setting battery capacity to 0 */
	val.intval = 0;
	ret = psy->set_property(psy, POWER_SUPPLY_PROP_CAPACITY, &val);
	if (ret < 0)
		return ret;

	power_supply_changed(psy);
	return 0;
}

/**
  * handle_events - handle different type of interrupts related to BCU
  * @flag - is the enumeration value for VWARNA, VWARNB or
  * a VCRIT interrupt.
  * @dev_data - device information
  */
static void handle_events(int flag, void *dev_data)
{
	uint8_t irq_data, sticky_data;
	struct vdd_info *vinfo = (struct vdd_info *)dev_data;
	int ret;

	ret = intel_scu_ipc_ioread8(SBCUIRQ, &irq_data);
	if (ret)
		goto handle_ipc_fail;

	switch (flag) {
	case VCRIT_EVENT:
		pr_info_ratelimited("%s: VCRIT_EVENT occurred\n",
					DRIVER_NAME);
		if (!pdata->is_clvp) {
			/* Masking VCRIT after one event occurs */
			ret = intel_scu_ipc_update_register(MBCUIRQ,
					MBCUIRQ_SET, VCRIT_MASK);
			if (ret)
				dev_err(&vinfo->pdev->dev,
					"VCRIT mask failed\n");
		}
		if (vinfo->seed_time && time_before((unsigned long)(jiffies_64 -
			vinfo->seed_time), (unsigned long)
			msecs_to_jiffies(STEP_TIME))) {
			vinfo->delay += STEP_TIME;
		} else {
			vinfo->delay = BASE_TIME;
		}
		if (bcu_workqueue) {
			ret = intel_scu_ipc_update_register(MBCUIRQ,
				MBCUIRQ_SET, VCRIT_MASK);
			if (ret) {
				dev_err(&vinfo->pdev->dev,
				"VCRIT mask failed\n");
			} else {
				if (pdata->is_clvp) {
					queue_delayed_work(bcu_workqueue,
					&(vinfo->vdd_intr_dwork),
					msecs_to_jiffies(vinfo->delay));
				}
			}
		} else {
			dev_warn(&vinfo->pdev->dev,
				"Workqueue is not present\n");
		}
		if (!(irq_data & SVCRIT)) {
			ret = intel_scu_ipc_ioread8(BCUDISCRIT_BEH,
				&sticky_data);
			if (ret)
				goto handle_ipc_fail;
			if (IS_STICKY(sticky_data)) {
				/*
				 * Clear the BCUDISCRIT bit in SBCUCTRL
				 * register i.e. 1st bit. We have to clear this
				 * bit manually as it will not go low
				 * automatically in case sticky bit is set.
				 */
				ret = intel_scu_ipc_update_register(SBCUCTRL,
						SBCUCTRL_SET, SBCUDISCRIT_MASK);
				if (ret)
					goto handle_ipc_fail;
			}
		}
		break;
	case VWARNB_EVENT:
		pr_info_ratelimited("%s: VWARNB_EVENT occurred\n",
					DRIVER_NAME);
		/**
		 * Trigger graceful shutdown via battery driver by setting SOC
		 * to 0%
		 */
		dev_info(&vinfo->pdev->dev, "EM_BCU: Trigger Graceful Shutdown\n");
		ret = bcu_action_voltage_drop();
		if (ret)
			dev_err(&vinfo->pdev->dev,
				"EM_BCU: Triggering Graceful Shutdown Failed.");

		if (!pdata->is_clvp) {

			ret = intel_scu_ipc_update_register(MBCUIRQ,
					MBCUIRQ_SET, VWARNB_MASK);
			if (ret)
				dev_err(&vinfo->pdev->dev,
					"VWARNB mask failed\n");

			/* No need to process further for byt, as user-space
			 * initiates a graceful shutdown on VWARNB
			 */
			break;
		}
		if (!(irq_data & SVWARNB)) {
			/* Vsys is above WARNB level */
			intel_scu_ipc_ioread8(BCUDISB_BEH, &sticky_data);

			if (IS_STICKY(sticky_data)) {
				/*
				 * Clear the BCUDISB bit in SBCUCTRL register
				 * i.e. 2nd bit. We have to clear this bit
				 * manually as it will not go low automatically
				 * in case sticky bit is set.
				 */
				ret = intel_scu_ipc_update_register(SBCUCTRL,
						SBCUCTRL_SET, SBCUDISB_MASK);
				if (ret)
					goto handle_ipc_fail;
			}
		}
		break;
	case VWARNA_EVENT:
		pr_info_ratelimited("%s: VWARNA_EVENT occurred\n",
					DRIVER_NAME);
		if (!pdata->is_clvp && bcu_workqueue) {
			ret = intel_scu_ipc_update_register(MBCUIRQ,
					MBCUIRQ_SET, VWARNA_MASK);
			if (ret) {
				dev_err(&vinfo->pdev->dev,
					"VWARNA mask failed\n");
			} else {
				/* Schedule to unmask after
				 * 30 seconds
				 */
				queue_delayed_work(bcu_workqueue,
					&vinfo->vdd_intr_dwork,
					VWARNA_INTR_EN_DELAY);
			}
		}
		if (!(irq_data & SVWARNA)) {
			/* Vsys is above WARNA level */
			intel_scu_ipc_ioread8(BCUDISA_BEH, &sticky_data);

			if (IS_STICKY(sticky_data)) {
				/*
				 * Clear the BCUDISA bit in SBCUCTRL register
				 * i.e. 3rd bit. We have to clear this bit
				 * manually as it will not go low automatically
				 * in case sticky bit is set.
				 */
				ret = intel_scu_ipc_update_register(SBCUCTRL,
						SBCUCTRL_SET, SBCUDISA_MASK);
				if (ret)
					goto handle_ipc_fail;
			}
		}
		break;
	default:
		dev_warn(&vinfo->pdev->dev, "Unresolved interrupt occurred\n");
	}

	return;
handle_ipc_fail:
	if (flag & VCRIT_EVENT)
		kobject_uevent(&vinfo->pdev->dev.kobj, KOBJ_CHANGE);
	dev_warn(&vinfo->pdev->dev, "ipc read/write failed\n");
}

/**
  * vdd_intrpt_handler - interrupt handler for BCU, runs in kernel context
  * @id - irq value
  * @dev - device information
  */
static irqreturn_t vdd_intrpt_handler(int id, void *dev)
{
	struct vdd_info *vinfo = (struct vdd_info *)dev;
	uint8_t irq_data;
	unsigned long __flags;

#ifndef CONFIG_ACPI
	irq_data = readb(vinfo->bcu_intr_addr);
	spin_lock_irqsave(&vdd_interrupt_lock, __flags);
	global_irq_data |= irq_data;
	spin_unlock_irqrestore(&vdd_interrupt_lock, __flags);
#endif
	return IRQ_WAKE_THREAD;
};

/**
  * vdd_interrupt_thread_handler - threaded interrupt handler for BCU
  * @irq - irq
  * @dev_data - device information
  */
static irqreturn_t vdd_interrupt_thread_handler(int irq, void *dev_data)
{
	uint8_t irq_data, event = 0, clear_irq, ret;
	struct vdd_info *vinfo = (struct vdd_info *)dev_data;

#ifdef CONFIG_ACPI
	ret = intel_scu_ipc_ioread8(BCUIRQ, &global_irq_data);
	if (ret)
		dev_warn(&vinfo->pdev->dev, "ipc read/write failed\n");
#endif
	spin_lock_irq(&vdd_interrupt_lock);
	clear_irq = global_irq_data;
	irq_data = global_irq_data;
	global_irq_data = 0;
	spin_unlock_irq(&vdd_interrupt_lock);
	if (irq_data == 0) {
		dev_err(&vinfo->pdev->dev, "no more interrupt to process\n");
		return IRQ_NONE;
	}

	mutex_lock(&vdd_update_lock);
	if (irq_data & VCRIT_IRQ) {
		/* BCU VCRIT Interrupt */
		event = VCRIT_EVENT;
		vinfo->intr_count_lvl3 += 1;

		handle_events(event, dev_data);
	}
	if (irq_data & VWARNA_IRQ) {
		/* BCU WARNA Interrupt */
		event = VWARNA_EVENT;
		vinfo->intr_count_lvl1 += 1;

		handle_events(event, dev_data);
	}
	if (irq_data & VWARNB_IRQ) {
		/* BCU WARNB Interrupt */
		event = VWARNB_EVENT;
		vinfo->intr_count_lvl2 += 1;

		handle_events(event, dev_data);
	}

#ifdef CONFIG_ACPI
	ret = intel_scu_ipc_iowrite8(BCUIRQ, clear_irq);
	clear_irq = 0;
	if (ret)
		dev_warn(&vinfo->pdev->dev, "ipc read/write failed\n");
#endif
	mutex_unlock(&vdd_update_lock);
	return IRQ_HANDLED;
}

static SENSOR_DEVICE_ATTR_2(voltage_warnA, S_IRUGO | S_IWUSR,
				show_volt_thres, store_volt_thres, 0, 0);
static SENSOR_DEVICE_ATTR_2(voltage_warnB, S_IRUGO | S_IWUSR,
				show_volt_thres, store_volt_thres, 1, 0);
static SENSOR_DEVICE_ATTR_2(voltage_warn_crit, S_IRUGO | S_IWUSR,
				show_volt_thres, store_volt_thres, 2, 0);
static SENSOR_DEVICE_ATTR_2(action_mask, S_IRUGO | S_IWUSR, show_action_mask,
				store_action_mask, 0, 0);
static SENSOR_DEVICE_ATTR_2(bcu_status, S_IRUGO | S_IWUSR, show_bcu_status,
				store_bcu_status, 0, 0);
static SENSOR_DEVICE_ATTR_2(irq_status, S_IRUGO, show_irq_status,
				NULL, 0, 0);
static SENSOR_DEVICE_ATTR_2(action_status, S_IRUGO, show_action_status,
				NULL, 0, 0);
static SENSOR_DEVICE_ATTR(intr_count_level1, S_IRUGO,
				show_intr_count, NULL, 2);
static SENSOR_DEVICE_ATTR(intr_count_level2, S_IRUGO,
				show_intr_count, NULL, 1);
static SENSOR_DEVICE_ATTR(intr_count_level3, S_IRUGO,
				show_intr_count, NULL, 4);
static SENSOR_DEVICE_ATTR(camflash_ctrl, S_IRUGO | S_IWUSR,
				show_camflash_ctrl, store_camflash_ctrl, 0);

static struct attribute *mid_vdd_attrs[] = {
	&sensor_dev_attr_voltage_warnA.dev_attr.attr,
	&sensor_dev_attr_voltage_warnB.dev_attr.attr,
	&sensor_dev_attr_voltage_warn_crit.dev_attr.attr,
	&sensor_dev_attr_action_mask.dev_attr.attr,
	&sensor_dev_attr_bcu_status.dev_attr.attr,
	&sensor_dev_attr_irq_status.dev_attr.attr,
	&sensor_dev_attr_action_status.dev_attr.attr,
	&sensor_dev_attr_intr_count_level1.dev_attr.attr,
	&sensor_dev_attr_intr_count_level2.dev_attr.attr,
	&sensor_dev_attr_intr_count_level3.dev_attr.attr,
	&sensor_dev_attr_camflash_ctrl.dev_attr.attr,
	NULL
};

static struct attribute_group mid_vdd_gr = {
	.name = "msic_voltage",
	.attrs = mid_vdd_attrs
};

/**
  * restore_default_value - assigns data structure with default values
  * @vdata - structure to be assigned
  */
static void restore_default_value(struct vdd_smip_data *vdata)
{
	vdata->vwarna_cfg = INIT_VWARNA;
	vdata->vwarnb_cfg = INIT_VWARNB;
	vdata->vcrit_cfg = INIT_VCRIT;
	vdata->bcudisa_beh = INIT_BCUDISA;
	vdata->bcudisb_beh = INIT_BCUDISB;
	vdata->bcudiscrit_beh = INIT_BCUISCRIT;
	vdata->bcuprochot_beh = INIT_BCUPROCHOT;
	vdata->mbcu_irq = INIT_MBCU;
}

/**
  * program_bcu - configures bcu related registers
  * @pdev - device pointer
  * @vinfo - device information
  */
static int program_bcu(struct platform_device *pdev, struct vdd_info *vinfo)
{
	int ret;
	uint8_t irq_data;

	/* earlier this structure was used locally only.
	 * now it needs to be accessed by other functions
	 * also as part of device info
	 */
	struct vdd_smip_data *vdata = &vinfo->init_reg_data;

	/* will be useful in case of clvp only
	 * in baytrail we are calling a dummy function
	 */
	ret = intel_scu_ipc_read_mip((u8 *)vdata, sizeof(struct
		vdd_smip_data), BCU_SMIP_OFFSET, 1);
	if (ret) {
		dev_err(&pdev->dev, "scu ipc read failed\n");
		restore_default_value(vdata);
		goto configure_register;
	}

	/* clear all bcu actions that are already set if
	 * system voltage is above thresholds
	 */
	ret = intel_scu_ipc_ioread8(SBCUIRQ, &irq_data);
	if (ret) {
		dev_warn(&pdev->dev, "Read VSYS flag failed\n");
		goto vdd_init_error;
	} else if (!(irq_data & SBCUIRQ_MASK)) {
		ret = intel_scu_ipc_iowrite8(SBCUCTRL, SBCUCTRL_SET);
		if (ret) {
			dev_warn(&pdev->dev, "scu ipc write failed\n");
			goto vdd_init_error;
		}
	}

	/*
	 * valid voltage will be in range 2.6-3.3V so 0 means values aren't
	 * defined in SMIP
	 */
	if (vdata->vwarna_cfg == 0)
		restore_default_value(vdata);

configure_register:
	/* configure all related register */
	/* CTP and VB aren't using WARNB so disabling it and CMS is not in
	 * place, so disabling CRIT also. Once CMS comes please remove CRIT
	 * from this part.
	 */
	/*
	 * pdata must be checked as pdata came as null in some platform
	 */
	if (pdata && (pdata->disable_unused_comparator & DISABLE_VWARNA))
		vdata->vwarna_cfg = 0;
	if (pdata && (pdata->disable_unused_comparator & DISABLE_VWARNB))
		vdata->vwarnb_cfg = 0;
	if (pdata && (pdata->disable_unused_comparator & DISABLE_VCRIT))
		vdata->vcrit_cfg = 0;


	ret = vdd_set_register(VWARNA_CFG, vdata->vwarna_cfg);
	ret |= vdd_set_register(VWARNB_CFG, vdata->vwarnb_cfg);
	ret |= vdd_set_register(VCRIT_CFG, vdata->vcrit_cfg);
	ret |= vdd_set_register(BCUDISA_BEH, vdata->bcudisa_beh);
	ret |= vdd_set_register(BCUDISB_BEH, vdata->bcudisb_beh);
	ret |= vdd_set_register(BCUDISCRIT_BEH, vdata->bcudiscrit_beh);
	ret |= vdd_set_register(BCUPROCHOT_BEH, vdata->bcuprochot_beh);
	ret |= vdd_set_register(MBCUIRQ, vdata->mbcu_irq);
	if (ret)
		goto vdd_init_error;

	return 0;
vdd_init_error:
	free_irq(vinfo->irq, vinfo);
	return ret;
}

#ifdef CONFIG_ACPI
extern void *msic_vdd_platform_data(void *);
#endif
static int mid_vdd_probe(struct platform_device *pdev)
{
	int ret;
	struct vdd_info *vinfo = devm_kzalloc(&pdev->dev,
				sizeof(struct vdd_info), GFP_KERNEL);
	if (!vinfo) {
		dev_err(&pdev->dev, "kzalloc failed\n");
		return -ENOMEM;
	}

	vinfo->pdev = pdev;
	vinfo->irq = platform_get_irq(pdev, 0);
	platform_set_drvdata(pdev, vinfo);
	pdata = pdev->dev.platform_data;
#ifndef CONFIG_ACPI
	vinfo->bcu_intr_addr = ioremap(MSIC_BCU_STAT, MSIC_BCU_LEN);
#else
	pdata = msic_vdd_platform_data(NULL);
	vinfo->bcu_intr_addr = BCUIRQ;
#endif
	if (!vinfo->bcu_intr_addr) {
		dev_err(&pdev->dev, "ioremap failed\n");
		return -ENOMEM;
	}

	/* Creating a sysfs group with mid_vdd_gr attributes */
	ret = sysfs_create_group(&pdev->dev.kobj, &mid_vdd_gr);
	if (ret) {
		dev_err(&pdev->dev, "sysfs_create_group failed\n");
		goto vdd_error1;
	}

	/* registering with hwmon class */
	vinfo->dev = hwmon_device_register(&pdev->dev);
	if (IS_ERR(vinfo->dev)) {
		ret = PTR_ERR(vinfo->dev);
		vinfo->dev = NULL;
		dev_err(&pdev->dev, "hwmon device registration failed\n");
		goto vdd_error2;
	}
	ret = request_threaded_irq(vinfo->irq, vdd_intrpt_handler,
					vdd_interrupt_thread_handler,
					IRQF_TRIGGER_FALLING|IRQF_ONESHOT,
					DRIVER_NAME, vinfo);
	if (ret) {
		dev_err(&pdev->dev, "request_threaded_irq failed:%d\n",
			ret);
		goto vdd_error3;
	}

	bcu_workqueue = create_singlethread_workqueue(BCU_QUEUE);
	if (!bcu_workqueue)
		dev_err(&pdev->dev, "workqueue creation failed\n");

	INIT_DELAYED_WORK(&(vinfo->vdd_intr_dwork), unmask_theburst);

	cam_flash_state = CAMFLASH_STATE_NORMAL;

	ret = program_bcu(pdev, vinfo);
	if (!ret)
		return ret;
vdd_error3:
	hwmon_device_unregister(vinfo->dev);
vdd_error2:
	sysfs_remove_group(&pdev->dev.kobj, &mid_vdd_gr);
vdd_error1:
	iounmap(vinfo->bcu_intr_addr);
	return ret;
}

static int mid_vdd_resume(struct device *dev)
{
	int ret;
	long bcu_enable = 1;
	struct vdd_info *vinfo = dev_get_drvdata(dev);

	mutex_lock(&vdd_update_lock);
	/* enabling comparators only if they
	 * are not disabled during initialization
	 * pdata must be checked as pdata came as null in some platform
	 */
	if ((!pdata) || !(pdata->disable_unused_comparator & DISABLE_VWARNA)) {
		ret = intel_scu_ipc_update_register(VWARNA_CFG,
				(bcu_enable << 3), VWARN_CFG_MASK);
		if (ret)
			goto bcu_ipc_fail;
	}
	if ((!pdata) || !(pdata->disable_unused_comparator & DISABLE_VWARNB)) {
		ret = intel_scu_ipc_update_register(VWARNB_CFG,
			(bcu_enable << 3), VWARN_CFG_MASK);
		if (ret)
			goto bcu_ipc_fail;
	}
	if ((!pdata) || !(pdata->disable_unused_comparator & DISABLE_VCRIT)) {
		ret = intel_scu_ipc_update_register(VCRIT_CFG,
			(bcu_enable << 4), VCRITSDWNEN_MASK);
		ret |= intel_scu_ipc_update_register(VCRIT_CFG,
			(bcu_enable << 3), VWARN_CFG_MASK);
		if (ret)
			goto bcu_ipc_fail;
	}

	/* Enabling o/p lines */
	ret = intel_scu_ipc_update_register(BCUDISA_BEH,
				vinfo->init_reg_data.bcudisa_beh, BCUDIS_MASK);
	ret |= intel_scu_ipc_update_register(BCUDISB_BEH,
				vinfo->init_reg_data.bcudisb_beh, BCUDIS_MASK);
	ret |= intel_scu_ipc_update_register(BCUDISCRIT_BEH,
			vinfo->init_reg_data.bcudiscrit_beh, BCUDIS_MASK);
	ret |= intel_scu_ipc_update_register(BCUPROCHOT_BEH,
			vinfo->init_reg_data.bcuprochot_beh, PROCHOT_B_MASK);
	if (ret)
		goto bcu_ipc_fail;

	/* Unmasking interrupt */
	ret = vdd_set_register(MBCUIRQ, vinfo->init_reg_data.mbcu_irq);
	ret = intel_scu_ipc_update_register(MIRQLVL1,
					~MIRQLVL1_BCU_MASK, MIRQLVL1_BCU_MASK);

bcu_ipc_fail:
	mutex_unlock(&vdd_update_lock);
	return ret;
}

static int mid_vdd_suspend(struct device *dev)
{
	int ret;
	long bcu_disable = 0;

	mutex_lock(&vdd_update_lock);
	/*
	 * To properly disable BCU
	 * Interrupts should be disabled
	 * after this o/p signals should be disabled
	 * finally the comparators should be disabled
	 */
	/* Masking interrupts */
	ret = intel_scu_ipc_update_register(MIRQLVL1,
					MIRQLVL1_BCU_MASK, MIRQLVL1_BCU_MASK);
	ret |= vdd_set_register(MBCUIRQ, MBCUIRQ_SET);
	/* Disabling o/p lines*/
	ret |= intel_scu_ipc_update_register(BCUPROCHOT_BEH,
							0x0, PROCHOT_B_MASK);
	ret |= intel_scu_ipc_update_register(BCUDISA_BEH, 0x0, BCUDIS_MASK);
	ret |= intel_scu_ipc_update_register(BCUDISB_BEH, 0x0, BCUDIS_MASK);
	ret |= intel_scu_ipc_update_register(BCUDISCRIT_BEH, 0x0, BCUDIS_MASK);
	if (ret)
		goto bcu_ipc_fail;

	/* Disabling Comparators only if they are already enabled
	 * during initialization
	 * pdata must be checked as pdata came as null in some platform
	 */
	if ((!pdata) || !(pdata->disable_unused_comparator & DISABLE_VWARNA)) {
		ret = intel_scu_ipc_update_register(VWARNA_CFG,
				(bcu_disable << 3), VWARN_CFG_MASK);
		if (ret)
			goto bcu_ipc_fail;
	}
	if ((!pdata) || !(pdata->disable_unused_comparator & DISABLE_VWARNB)) {
		ret = intel_scu_ipc_update_register(VWARNB_CFG,
				(bcu_disable << 3), VWARN_CFG_MASK);
		if (ret)
			goto bcu_ipc_fail;
	}
	if ((!pdata) || !(pdata->disable_unused_comparator & DISABLE_VCRIT)) {
		ret = intel_scu_ipc_update_register(VCRIT_CFG,
				(bcu_disable << 4), VCRITSDWNEN_MASK);
		ret |= intel_scu_ipc_update_register(VCRIT_CFG,
				(bcu_disable << 3), VWARN_CFG_MASK);
	}
bcu_ipc_fail:
	mutex_unlock(&vdd_update_lock);
	return ret;
}

static int mid_vdd_remove(struct platform_device *pdev)
{
	struct vdd_info *vinfo = platform_get_drvdata(pdev);

	if (vinfo) {
		if (bcu_workqueue)
			destroy_workqueue(bcu_workqueue);
		free_irq(vinfo->irq, vinfo);
		hwmon_device_unregister(vinfo->dev);
		sysfs_remove_group(&pdev->dev.kobj, &mid_vdd_gr);
		iounmap(vinfo->bcu_intr_addr);
	}
	return 0;
}

/*********************************************************************
 *		Driver initialisation and finalization
 *********************************************************************/
static const struct platform_device_id vdd_id_table[] = {
	{ DEVICE_NAME, 1 },
	{ },
};

static const struct dev_pm_ops msic_vdd_pm_ops = {
	.suspend = mid_vdd_suspend,
	.resume = mid_vdd_resume,
};

static struct platform_driver mid_volt_drop_detect_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.pm = &msic_vdd_pm_ops,
		},
	.probe = mid_vdd_probe,
	.remove = mid_vdd_remove,
	.id_table = vdd_id_table,
};

static int __init mid_vdd_module_init(void)
{
	return platform_driver_register(&mid_volt_drop_detect_driver);
}

static void __exit mid_vdd_module_exit(void)
{
	platform_driver_unregister(&mid_volt_drop_detect_driver);
}

module_init(mid_vdd_module_init);
module_exit(mid_vdd_module_exit);

MODULE_AUTHOR("Durgadoss R <durgadoss.r@intel.com>");
MODULE_AUTHOR("Chaurasia, Avinash K <avinash.k.chaurasia@intel.com>");
MODULE_DESCRIPTION("Intel CloverView Voltage Drop Detection Driver");
MODULE_LICENSE("GPL");
