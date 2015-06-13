/*
 * extcon-fsa9285.c - FSA9285 extcon driver
 *
 * Copyright (C) 2013 Intel Corporation
 * Ramakrishna Pallala <ramakrishna.pallala@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/usb/phy.h>
#include <linux/notifier.h>
#include <linux/extcon.h>
#include <linux/pm_runtime.h>
#include <linux/acpi.h>
#include <linux/acpi_gpio.h>
#include <linux/power_supply.h>
#include <linux/extcon/extcon-fsa9285.h>
#include <linux/wakelock.h>

/* FSA9285 I2C registers */
#define FSA9285_REG_DEVID		0x01
#define FSA9285_REG_CTRL		0x02
#define FSA9285_REG_INTR		0x03
#define FSA9285_REG_INTR_MASK		0x05
#define FSA9285_REG_OHM_CODE		0x07
#define FSA9285_REG_TIMING		0x08
#define FSA9285_REG_STATUS		0x09
#define FSA9285_REG_DEVTYPE		0x0A
#define FSA9285_REG_DACSAR		0x0B
#define FSA9285_REG_MAN_SW		0x13
#define FSA9285_REG_MAN_CHGCTRL		0x14

/* device ID value */
#define DEVID_VALUE		0x10

/* Control */
#define CTRL_EN_DCD_TOUT	(1 << 7)
#define CTRL_EM_RESETB		(1 << 6)
#define CTRL_EM_ID_DIS		(1 << 5)
#define CTRL_EM_MAN_SW		(1 << 4)
#define CTRL_INT_MASK		(1 << 0)

/* Interrupts */
#define INTR_OCP_CHANGE		(1 << 6)
#define INTR_OVP_CHANGE		(1 << 5)
#define INTR_MIC_OVP		(1 << 4)
#define INTR_OHM_CHANGE		(1 << 3)
#define INTR_VBUS_CHANGE	(1 << 2)
#define INTR_BC12_DONE		(1 << 1)

/* resistance codes */
#define OHM_CODE_USB_SLAVE	0x16
#define OHM_CODE_UART		0x0C
#define OHM_CODE_USB_ACA	0x0A

/* Timing */
#define TIMING_500MS		0x6

/* Status */
#define STATUS_ID_SHORT		(1 << 7)
#define STATUS_OCP		(1 << 6)
#define STATUS_OVP		(1 << 5)
#define STATUS_MIC_OVP		(1 << 4)
#define STATUS_ID_NO_FLOAT	(1 << 3)
#define STATUS_VBUS_VALID	(1 << 2)
#define STATUS_DCD		(1 << 0)

/* Device Type */
#define DEVTYPE_DOCK		(1 << 5)
#define DEVTYPE_DCP		(1 << 2)
#define DEVTYPE_CDP		(1 << 1)
#define DEVTYPE_SDP		(1 << 0)

/*
 * Manual Switch
 * D- [7:5] / D+ [4:2]
 * 000: Open all / 001: HOST USB1 / 010: AUDIO / 011: HOST USB2 / 100: MIC
 * VBUS_IN SW[1:0]
 * 00: Open all/ 01: N/A / 10: VBUS_IN to MIC / 11: VBUS_IN to VBUS_OUT
 */
#define MAN_SW_DPDM_MIC			((4 << 5) | (4 << 2))
#define MAN_SW_DPDM_HOST2		((3 << 5) | (3 << 2))
/* same code can be used for AUDIO/UART */
#define MAN_SW_DPDM_UART		((2 << 5) | (2 << 2))
#define MAN_SW_DPDM_HOST1		((1 << 5) | (1 << 2))
#define MAN_SW_DPDP_AUTO		((0 << 5) | (0 << 2))
#define MAN_SW_VBUSIN_MASK		(3 << 0)
#define MAN_SW_VBUSIN_VOUT		(3 << 0)
#define MAN_SW_VBUSIN_MIC		(2 << 0)
#define MAN_SW_VBUSIN_AUTO		(0 << 0)

/* Manual Charge Control */
#define CHGCTRL_ASSERT_CHG_DETB		(1 << 4)
#define CHGCTRL_MIC_OVP_EN		(1 << 3)
#define CHGCTRL_ASSERT_DP		(1 << 2)

#define FSA_CHARGE_CUR_DCP		2000
#define FSA_CHARGE_CUR_ACA		2000
#define FSA_CHARGE_CUR_CDP		1500
#define FSA_CHARGE_CUR_SDP		500

#define FSA9285_EXTCON_SDP		"CHARGER_USB_SDP"
#define FSA9285_EXTCON_DCP		"CHARGER_USB_DCP"
#define FSA9285_EXTCON_CDP		"CHARGER_USB_CDP"
#define FSA9285_EXTCON_ACA		"CHARGER_USB_ACA"
#define FSA9285_EXTCON_DOCK		"Dock"
#define FSA9285_EXTCON_USB_HOST		"USB-Host"

#define MAX_RETRY			3

static const char *fsa9285_extcon_cable[] = {
	FSA9285_EXTCON_SDP,
	FSA9285_EXTCON_DCP,
	FSA9285_EXTCON_CDP,
	FSA9285_EXTCON_ACA,
	FSA9285_EXTCON_DOCK,
	FSA9285_EXTCON_USB_HOST,
	NULL,
};

struct fsa9285_chip {
	struct i2c_client	*client;
	struct fsa9285_pdata	*pdata;
	struct usb_phy		*otg;
	struct extcon_dev	*edev;

	/* reg data */
	u8	cntl;
	u8	man_sw;
	u8	man_chg_cntl;

	bool	vbus_drive;
	bool	a_bus_drop;

	struct wake_lock wakelock;
};

static struct fsa9285_chip *chip_ptr;
extern void *fsa9285_platform_data(void);

static int fsa9285_write_reg(struct i2c_client *client,
		int reg, int value)
{
	int ret;
	int retry;

	for (retry = 0; retry < MAX_RETRY; retry++) {
		ret = i2c_smbus_write_byte_data(client, reg, value);
		if (ret < 0)
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		else
			break;
	}

	return ret;
}

static int fsa9285_read_reg(struct i2c_client *client, int reg)
{
	int ret;
	int retry;

	for (retry = 0; retry < MAX_RETRY; retry++) {
		ret = i2c_smbus_read_byte_data(client, reg);
		if (ret < 0)
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		else
			break;
	}

	return ret;
}

static void fsa9285_vbus_cntl_state(struct usb_phy *phy)
{
	struct fsa9285_chip *chip = chip_ptr;
	int ret = 0;

	if (!chip)
		return;

	if (phy->vbus_state == VBUS_DISABLED) {
		dev_info(&chip->client->dev,
			"a_bus_drop event(true)\n");
		chip->a_bus_drop = true;
		if (chip->vbus_drive)
			ret = chip->pdata->disable_vbus();
	} else {
		dev_info(&chip->client->dev,
			"a_bus_drop event(false)\n");
		chip->a_bus_drop = false;
		if (chip->vbus_drive)
			ret = chip->pdata->enable_vbus();
	}

	if (ret < 0)
		dev_warn(&chip->client->dev,
			"pmic vbus control failed\n");
}

static int fsa9285_detect_dev(struct fsa9285_chip *chip)
{
	struct i2c_client *client = chip->client;
	static bool notify_otg, notify_charger;
	static char *cable;
	static struct power_supply_cable_props cable_props;
	int stat, devtype, ohm_code, cntl, intmask, ret;
	u8 w_man_sw, w_man_chg_cntl;
	bool discon_evt = false, drive_vbus = false;
	int vbus_mask = 0;
	int usb_switch = 1;

	/* read status registers */
	ret = fsa9285_read_reg(client, FSA9285_REG_CTRL);
	if (ret < 0)
		goto dev_det_i2c_failed;
	else
		cntl = ret;

	ret = fsa9285_read_reg(client, FSA9285_REG_DEVTYPE);
	if (ret < 0)
		goto dev_det_i2c_failed;
	else
		devtype = ret;

	ret = fsa9285_read_reg(client, FSA9285_REG_STATUS);
	if (ret < 0)
		goto dev_det_i2c_failed;
	else
		stat = ret;

	ret = fsa9285_read_reg(client, FSA9285_REG_OHM_CODE);
	if (ret < 0)
		goto dev_det_i2c_failed;
	else
		ohm_code = ret;

	dev_info(&client->dev, "devtype:%x, Stat:%x, ohm:%x cntl:%x\n",
				devtype, stat, ohm_code, cntl);

	/* set default register setting */
	w_man_sw = (chip->man_sw & 0x3) | MAN_SW_DPDM_HOST1;
	w_man_chg_cntl = chip->man_chg_cntl & ~CHGCTRL_ASSERT_CHG_DETB;

	if (stat & STATUS_ID_SHORT) {
		if (ohm_code == OHM_CODE_USB_SLAVE) {
			dev_info(&chip->client->dev,
				"USB slave device connecetd\n");
			drive_vbus = true;
		}
	} else if ((stat & STATUS_ID_NO_FLOAT) && (stat & STATUS_VBUS_VALID)) {
		if (ohm_code == OHM_CODE_UART) {
			dev_info(&chip->client->dev,
				"UART device connecetd\n");
			/* select UART */
			w_man_sw = (chip->man_sw & 0x3) | MAN_SW_DPDM_UART;
		} else if (ohm_code == OHM_CODE_USB_ACA) {
			dev_info(&chip->client->dev,
				"ACA device connecetd\n");
			notify_charger = true;
			cable = FSA9285_EXTCON_ACA;
			cable_props.chrg_evt =
					POWER_SUPPLY_CHARGER_EVENT_CONNECT;
			cable_props.chrg_type =
					POWER_SUPPLY_CHARGER_TYPE_USB_ACA;
			cable_props.ma = FSA_CHARGE_CUR_ACA;
			if (!wake_lock_active(&chip->wakelock))
				wake_lock(&chip->wakelock);
		} else {
			/* unknown device */
			dev_warn(&chip->client->dev, "unknown ID detceted\n");
		}
	} else if (devtype & DEVTYPE_SDP) {
		dev_info(&chip->client->dev,
				"SDP cable connecetd\n");
		/* select Host2 */
		w_man_sw = (chip->man_sw & 0x3) | MAN_SW_DPDM_HOST2;
		w_man_chg_cntl = chip->man_chg_cntl | CHGCTRL_ASSERT_CHG_DETB;
		notify_otg = true;
		vbus_mask = 1;
	} else if (devtype & DEVTYPE_CDP) {
		dev_info(&chip->client->dev,
				"CDP cable connecetd\n");
		/* select Host2 */
		w_man_sw = (chip->man_sw & 0x3) | MAN_SW_DPDM_HOST2;
		notify_otg = true;
		vbus_mask = 1;
		notify_charger = true;
		cable = FSA9285_EXTCON_CDP;
		cable_props.chrg_evt = POWER_SUPPLY_CHARGER_EVENT_CONNECT;
		cable_props.chrg_type = POWER_SUPPLY_CHARGER_TYPE_USB_CDP;
		cable_props.ma = FSA_CHARGE_CUR_CDP;
	} else if (devtype & DEVTYPE_DCP) {
		dev_info(&chip->client->dev,
				"DCP cable connecetd\n");
		notify_charger = true;
		cable = FSA9285_EXTCON_DCP;
		cable_props.chrg_evt = POWER_SUPPLY_CHARGER_EVENT_CONNECT;
		cable_props.chrg_type = POWER_SUPPLY_CHARGER_TYPE_USB_DCP;
		cable_props.ma = FSA_CHARGE_CUR_DCP;
		if (!wake_lock_active(&chip->wakelock))
			wake_lock(&chip->wakelock);
	} else if (devtype & DEVTYPE_DOCK) {
		dev_info(&chip->client->dev,
				"Dock connecetd\n");
		notify_charger = true;
		cable = FSA9285_EXTCON_DOCK;
		cable_props.chrg_evt = POWER_SUPPLY_CHARGER_EVENT_CONNECT;
		cable_props.chrg_type = POWER_SUPPLY_CHARGER_TYPE_ACA_DOCK;
		cable_props.ma = FSA_CHARGE_CUR_ACA;
		if (!wake_lock_active(&chip->wakelock))
			wake_lock(&chip->wakelock);
	} else {
		dev_warn(&chip->client->dev,
			"ID or VBUS change event\n");
		if (stat & STATUS_VBUS_VALID)
			chip->cntl = cntl | CTRL_EN_DCD_TOUT;
		else
			chip->cntl = cntl & ~CTRL_EN_DCD_TOUT;

		ret = fsa9285_write_reg(client, FSA9285_REG_CTRL, chip->cntl);
		if (ret < 0)
			dev_warn(&chip->client->dev, "i2c write failed\n");
		/* disconnect event */
		discon_evt = true;
		/* usb switch off per nothing attached */
		usb_switch = 0;
		cable_props.ma = 0;
		cable_props.chrg_evt = POWER_SUPPLY_CHARGER_EVENT_DISCONNECT;
	}

	/* VBUS control */
	if (drive_vbus && !chip->a_bus_drop) {
		intmask = fsa9285_read_reg(client, FSA9285_REG_INTR_MASK);
		if (intmask < 0)
			goto dev_det_i2c_failed;

		/* disable VBUS interrupt */
		ret = fsa9285_write_reg(client, FSA9285_REG_INTR_MASK,
						intmask | INTR_VBUS_CHANGE);
		if (ret < 0)
			goto dev_det_i2c_failed;

		ret = chip->pdata->enable_vbus();

		/* clear interrupt */
		ret = fsa9285_read_reg(client, FSA9285_REG_INTR);
		if (ret < 0)
			dev_err(&chip->client->dev,
				"i2c read failed:%d\n", ret);

		/* enable VBUS interrupt */
		ret = fsa9285_write_reg(client, FSA9285_REG_INTR_MASK,
						intmask);
		if (ret < 0)
			goto dev_det_i2c_failed;

	} else
		ret = chip->pdata->disable_vbus();
	if (ret < 0)
		dev_warn(&chip->client->dev,
			"pmic vbus control failed\n");
	chip->vbus_drive = drive_vbus;

	/* handle SDP case before enabling CHG_DETB */
	if (w_man_chg_cntl & CHGCTRL_ASSERT_CHG_DETB)
		ret = chip->pdata->sdp_pre_setup();
	else if (!discon_evt)
		ret = chip->pdata->sdp_post_setup();

	if (ret < 0)
		dev_warn(&chip->client->dev,
			"sdp cable control failed\n");

	if (chip->pdata->xsd_gpio != -1) {
		if (usb_switch)
			gpio_direction_output(chip->pdata->xsd_gpio, 0);
		else
			gpio_direction_output(chip->pdata->xsd_gpio, 1);
	}

	if (chip->pdata->mux_gpio != -1) {
		if (vbus_mask)
			gpio_direction_output(chip->pdata->mux_gpio, 1);
		else
			gpio_direction_output(chip->pdata->mux_gpio, 0);
	}

	/* enable manual charge detection */
	ret = fsa9285_write_reg(client,
			FSA9285_REG_MAN_CHGCTRL, w_man_chg_cntl);
	if (ret < 0)
		goto dev_det_i2c_failed;

	/* select the controller */
	ret = fsa9285_write_reg(client, FSA9285_REG_MAN_SW, w_man_sw);
	if (ret < 0)
		goto dev_det_i2c_failed;

	if (discon_evt) {
		if (notify_otg) {
			atomic_notifier_call_chain(&chip->otg->notifier,
						USB_EVENT_VBUS, &vbus_mask);
			notify_otg = false;
		}
		if (notify_charger) {
			atomic_notifier_call_chain(&power_supply_notifier,
					POWER_SUPPLY_CABLE_EVENT, &cable_props);
			notify_charger = false;
			cable = NULL;
		}
		if (wake_lock_active(&chip->wakelock))
			wake_unlock(&chip->wakelock);
	} else {
		if (notify_otg)
			atomic_notifier_call_chain(&chip->otg->notifier,
						USB_EVENT_VBUS, &vbus_mask);
		if (notify_charger) {
			atomic_notifier_call_chain(&power_supply_notifier,
					POWER_SUPPLY_CABLE_EVENT, &cable_props);
		}
	}

	return 0;

dev_det_i2c_failed:
	dev_err(&chip->client->dev, "i2c read failed:%d\n", ret);
	return ret;
}

static irqreturn_t fsa9285_irq_handler(int irq, void *data)
{
	struct fsa9285_chip *chip = data;
	struct i2c_client *client = chip->client;
	int ret;

	pm_runtime_get_sync(&chip->client->dev);

	/* clear interrupt */
	ret = fsa9285_read_reg(client, FSA9285_REG_INTR);
	if (ret <= 0) {
		dev_info(&client->dev, "invalid intr:%x\n", ret);
		goto isr_ret;
	} else
		dev_info(&client->dev, "clear intr:%x\n", ret);

	mdelay(10);
	/* device detection */
	ret = fsa9285_detect_dev(chip);
	if (ret < 0)
		dev_err(&chip->client->dev,
				"fsa9285 detecting devices failed:%d\n", ret);
isr_ret:
	pm_runtime_put_sync(&chip->client->dev);
	return IRQ_HANDLED;
}

static int fsa9285_irq_init(struct fsa9285_chip *chip)
{
	struct i2c_client *client = chip->client;
	int ret, gpio_num, cntl, man_sw, man_chg_cntl;
	struct acpi_gpio_info gpio_info;

	/* clear interrupt */
	ret = fsa9285_read_reg(client, FSA9285_REG_INTR);
	if (ret < 0)
		goto irq_i2c_failed;

	/* Mask unused interrupts */
	ret = fsa9285_write_reg(client, FSA9285_REG_INTR_MASK, 0xFF &
		~(INTR_OHM_CHANGE | INTR_BC12_DONE | INTR_VBUS_CHANGE));
	if (ret < 0)
		goto irq_i2c_failed;

	/*
	 * unmask INT line to SOC and
	 * also enable manual switching
	 */
	ret = fsa9285_read_reg(client, FSA9285_REG_CTRL);
	if (ret < 0)
		goto irq_i2c_failed;
	else
		cntl = ret;
	cntl = (cntl | CTRL_EM_MAN_SW) & ~(CTRL_INT_MASK | CTRL_EN_DCD_TOUT);
	ret = fsa9285_write_reg(client, FSA9285_REG_CTRL, cntl);
	if (ret < 0)
		goto irq_i2c_failed;
	chip->cntl = cntl;

	/* select the switch */
	ret = fsa9285_read_reg(client, FSA9285_REG_MAN_SW);
	if (ret < 0)
		goto irq_i2c_failed;
	else
		man_sw = ret;
	/* select HOST1 by default */
	ret = fsa9285_write_reg(client, FSA9285_REG_MAN_SW,
				(man_sw & 0x3) | MAN_SW_DPDM_HOST1);
	if (ret < 0)
		goto irq_i2c_failed;
	chip->man_sw = man_sw;

	/* get charge control setting */
	ret = fsa9285_read_reg(client, FSA9285_REG_MAN_CHGCTRL);
	if (ret < 0)
		goto irq_i2c_failed;
	else
		man_chg_cntl = ret;
	chip->man_chg_cntl = man_chg_cntl;

	/* get kernel GPIO number */
	gpio_num = acpi_get_gpio_by_index(&client->dev, 0, &gpio_info);
	/* get irq number */
	chip->client->irq = gpio_to_irq(gpio_num);
	if (client->irq) {
		ret = request_threaded_irq(client->irq, NULL,
				fsa9285_irq_handler,
				IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				"fsa9285", chip);
		if (ret) {
			dev_err(&client->dev, "failed to reqeust IRQ\n");
			return ret;
		}
		enable_irq_wake(client->irq);
	} else {
		dev_err(&client->dev, "IRQ not set\n");
		return -EINVAL;
	}

	return 0;

irq_i2c_failed:
	dev_err(&chip->client->dev, "i2c read failed:%d\n", ret);
	return ret;
}

static int fsa9285_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct device *dev = &client->dev;
	struct fsa9285_chip *chip;
	int ret = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	ret = fsa9285_read_reg(client, FSA9285_REG_DEVID);
	if (ret < 0 || ret != DEVID_VALUE) {
		dev_err(&client->dev,
			"fsa chip ID check failed:%d\n", ret);
		return -ENODEV;
	}


	chip = kzalloc(sizeof(struct fsa9285_chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "failed to allocate driver data\n");
		return -ENOMEM;
	}

	chip->client = client;
	chip->pdata = dev->platform_data;
	chip->pdata =   fsa9285_platform_data();
	i2c_set_clientdata(client, chip);
	chip_ptr = chip;

	/* register with extcon */
	chip->edev = kzalloc(sizeof(struct extcon_dev), GFP_KERNEL);
	if (!chip->edev) {
		dev_err(&client->dev, "mem alloc failed\n");
		ret = -ENOMEM;
		goto extcon_mem_failed;
	}
	chip->edev->name = "fsa9285";
	chip->edev->supported_cable = fsa9285_extcon_cable;
	ret = extcon_dev_register(chip->edev, &client->dev);
	if (ret) {
		dev_err(&client->dev, "extcon registration failed!!\n");
		goto extcon_reg_failed;
	}

	/* OTG notification */
	chip->otg = usb_get_phy(USB_PHY_TYPE_USB2);
	if (!chip->otg) {
		dev_warn(&client->dev, "Failed to get otg transceiver!!\n");
		goto otg_reg_failed;
	}
	chip->otg->a_bus_drop = fsa9285_vbus_cntl_state;

	ret = fsa9285_irq_init(chip);
	if (ret)
		goto intr_reg_failed;

	wake_lock_init(&chip->wakelock, WAKE_LOCK_SUSPEND,
						"fsa_charger_wakelock");
	/* device detection */
	ret = fsa9285_detect_dev(chip);
	if (ret < 0)
		dev_warn(&client->dev, "probe: detection failed\n");

	/* Init Runtime PM State */
	pm_runtime_put_noidle(&chip->client->dev);
	pm_schedule_suspend(&chip->client->dev, MSEC_PER_SEC);

	return 0;

intr_reg_failed:
	if (client->irq)
		free_irq(client->irq, chip);
/* WA for FFRD8 */
	if (chip->pdata->mux_gpio != -1)
		gpio_free(chip->pdata->mux_gpio);
/* gpio_req_failed: */
	usb_put_phy(chip->otg);
otg_reg_failed:
	extcon_dev_unregister(chip->edev);
extcon_reg_failed:
	kfree(chip->edev);
extcon_mem_failed:
	kfree(chip);
	return ret;
}

static int fsa9285_remove(struct i2c_client *client)
{
	struct fsa9285_chip *chip = i2c_get_clientdata(client);

	if (chip->pdata->mux_gpio != -1)
		gpio_free(chip->pdata->mux_gpio);
	free_irq(client->irq, chip);
	usb_put_phy(chip->otg);
	extcon_dev_unregister(chip->edev);
	kfree(chip->edev);
	pm_runtime_get_noresume(&chip->client->dev);
	kfree(chip);
	return 0;
}

static void fsa9285_shutdown(struct i2c_client *client)
{
	dev_dbg(&client->dev, "fsa9285 shutdown\n");

	if (client->irq > 0)
		disable_irq(client->irq);
	return;
}

static int fsa9285_suspend(struct device *dev)
{
	struct fsa9285_chip *chip = dev_get_drvdata(dev);

	if (chip->client->irq > 0) {
		disable_irq(chip->client->irq);
	}

	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int fsa9285_resume(struct device *dev)
{
	struct fsa9285_chip *chip = dev_get_drvdata(dev);

	if (chip->client->irq > 0) {
		enable_irq(chip->client->irq);
	}

	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int fsa9285_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int fsa9285_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int fsa9285_runtime_idle(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static const struct dev_pm_ops fsa9285_pm_ops = {
		SET_SYSTEM_SLEEP_PM_OPS(fsa9285_suspend,
				fsa9285_resume)
		SET_RUNTIME_PM_OPS(fsa9285_runtime_suspend,
				fsa9285_runtime_resume,
				fsa9285_runtime_idle)
};

static const struct i2c_device_id fsa9285_id[] = {
	{"fsa9285", 0},
	{"SFSA9285", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, fsa9285_id);

static const struct acpi_device_id acpi_fsa9285_id[] = {
	{"SFSA9285", 0},
	{}
};
MODULE_DEVICE_TABLE(acpi, acpi_fsa9285_id);

static struct i2c_driver fsa9285_i2c_driver = {
	.driver = {
		.name = "fsa9285",
		.owner	= THIS_MODULE,
		.pm	= &fsa9285_pm_ops,
		.acpi_match_table = ACPI_PTR(acpi_fsa9285_id),
	},
	.probe = fsa9285_probe,
	.remove = fsa9285_remove,
	.id_table = fsa9285_id,
	.shutdown = fsa9285_shutdown,
};

/*
 * Module stuff
 */

static int __init fsa9285_extcon_init(void)
{
	int ret = i2c_add_driver(&fsa9285_i2c_driver);
	if (ret)
		printk(KERN_ERR "Unable to register ULPMC i2c driver\n");

	return ret;
}
late_initcall(fsa9285_extcon_init);

static void __exit fsa9285_extcon_exit(void)
{
	i2c_del_driver(&fsa9285_i2c_driver);
}
module_exit(fsa9285_extcon_exit);

MODULE_AUTHOR("Ramakrishna Pallala <ramakrishna.pallala@intel.com>");
MODULE_DESCRIPTION("FSA9285 extcon driver");
MODULE_LICENSE("GPL");
