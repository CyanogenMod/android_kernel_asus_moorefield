/*
 * bq27742x charger driver
 *
 * Copyright (c) 2015, ASUSTek, Inc. All Rights Reserved.
 * Written bychih-hsuan_chang <chih-hsuan_chang@asus.com>
 *
 */

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/HWVersion.h>
#include <linux/earlysuspend.h>
#include <asm/unaligned.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#define GAUGE_ERR(...)		printk("[bq27742_err] " __VA_ARGS__)
#define GAUGE_INFO(...)		printk("[bq27742] " __VA_ARGS__)

/* bq27742 standard commands */
#define BQ27742_REG_CONTROL	0x00
#define BQ27742_REG_TEMP		0x06
#define BQ27742_REG_VOLT		0x08
#define BQ27742_REG_RM		0x10
#define BQ27742_REG_FCC		0x12
#define BQ27742_REG_CURR		0x14
#define BQ27742_REG_SOC		0x2C
#define BQ27742_REG_CC		0x2A
#define BQ27742_REG_SOH		0x2E
#define BQ27742_REG_DC		0x3C
/* bq27742 control() commands */
#define BQ27742_FW_VERSION	0x0002
#define BQ27742_HW_VERSION	0x0003
/* bq27742 setting defines */
#define BQ27742_INIT_TIME		1*HZ
#define BQ27742_UPDATE_TIME	0*HZ
#define BQ27742_POLLING_TIME	60*HZ

/* global variables */
static int charging_status=0;
static s32 fw_ver, hw_ver;
static struct bq27742_chip *g_bq27742_chip;

/*extern symbols*/
extern int Read_HW_ID(void);
extern int Read_PROJ_ID(void);

struct bq27742_chip {
	struct power_supply psy_battery;
	struct i2c_client *client;
	struct switch_dev batt_sdev;
	struct workqueue_struct *bq27742_delay_wq;
	struct delayed_work bq27742_status_work;
};

static int bq27742_write_reg(struct i2c_client *client, u8 reg, u16 value)
{
	int ret = i2c_smbus_write_word_data(client, reg, value);

	if (ret < 0)
		GAUGE_ERR("%s: err and ret = %d\n", __func__, ret);

	return ret;
}

static int bq27742_read_reg(struct i2c_client *client, u8 reg)
{
	int ret = i2c_smbus_read_word_data(client, reg);

	if (ret < 0)
		GAUGE_ERR("%s: err and ret =%d\n", __func__, ret);

	return ret;
}

static int bq27742_status(struct i2c_client *client)
{
	return charging_status;
}

static int bq27742_soc(struct i2c_client *client)
{
	s32 soc=0;

	soc = bq27742_read_reg(client, BQ27742_REG_SOC);
	if (soc < 0) {
		GAUGE_ERR("%s: err and ret =%d\n", __func__, soc);
		return 50;
	} else {
		return soc;
	}
}

static int bq27742_voltage(struct i2c_client *client)
{
	s32 volt=0;

	volt = bq27742_read_reg(client, BQ27742_REG_VOLT);
	if (volt < 0) {
		GAUGE_ERR("%s: err and ret =%d\n", __func__, volt);
		return 3500;
	} else {
		return volt;
	}
}

static int bq27742_current(struct i2c_client *client)
{
	s32 curr=0;
	s32 curr_final=0;

	curr = bq27742_read_reg(client, BQ27742_REG_CURR);
	if (curr < 0) {
		GAUGE_ERR("%s: err and ret =%d\n", __func__, curr);
		return 0;
	} else {
		if (curr > 32767) {
			curr_final = (65536 - curr)*(-1);
		} else {
			curr_final = curr;
		}
		return curr_final;
	}
}

static int bq27742_temperature(struct i2c_client *client)
{
	s32 temp=0;

	temp = bq27742_read_reg(client, BQ27742_REG_TEMP);
	if (temp < 0) {
		GAUGE_ERR("%s: err and ret =%d\n", __func__, temp);
		return 250;
	} else {
		/* 0.1 oK -> 0.1 oC*/
		temp = temp - 2730;
		return temp;
	}
}

static int bq27742_fcc(struct i2c_client *client)
{
	s32 fcc=0;

	fcc = bq27742_read_reg(client, BQ27742_REG_FCC);
	if (fcc < 0) {
		GAUGE_ERR("%s: err and ret =%d\n", __func__, fcc);
		return 3000;
	} else {
		return fcc;
	}
}

static int bq27742_dc(struct i2c_client *client)
{
	s32 dc=0;

	dc = bq27742_read_reg(client, BQ27742_REG_DC);
	if (dc < 0) {
		GAUGE_ERR("%s: err and ret =%d\n", __func__, dc);
		return 3000;
	} else {
		return dc;
	}
}

static int bq27742_rm(struct i2c_client *client)
{
	s32 rm=0;

	rm = bq27742_read_reg(client, BQ27742_REG_RM);
	if (rm < 0) {
		GAUGE_ERR("%s: err and ret =%d\n", __func__, rm);
		return 1500;
	} else {
		return rm;
	}
}

static int bq27742_soh(struct i2c_client *client)
{
	s32 soh=0;

	soh = bq27742_read_reg(client, BQ27742_REG_SOH);
	if (soh < 0) {
		GAUGE_ERR("%s: err and ret =%d\n", __func__, soh);
		return 50;
	} else {
		return soh;
	}
}

static int bq27742_cc(struct i2c_client *client)
{
	s32 cc=0;

	cc = bq27742_read_reg(client, BQ27742_REG_CC);
	if (cc < 0) {
		GAUGE_ERR("%s: err and ret =%d\n", __func__, cc);
		return -1;
	} else {
		return cc;
	}
}
/* add power_supply attr:battery_soh +++*/
static ssize_t bq27742_sysfs_show_battery_soh(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bq27742_chip *chip = container_of(psy, struct bq27742_chip, psy_battery);
	return sprintf(buf, "%d\n", bq27742_soh(chip->client));
}

static DEVICE_ATTR(battery_soh, S_IRUGO,
		bq27742_sysfs_show_battery_soh, NULL);

static struct attribute *bq27742_sysfs_attributes[] = {
	&dev_attr_battery_soh.attr,
	NULL,
};

static const struct attribute_group bq27742_sysfs_attr_group = {
	.attrs = bq27742_sysfs_attributes,
};
/* add power_supply attr:battery_soh ---*/

static enum power_supply_property bq27742_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
};

static int bq27742_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	int ret = 0;
	struct bq27742_chip *chip = container_of(psy, struct bq27742_chip, psy_battery);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bq27742_status(chip->client);
                break;
	case POWER_SUPPLY_PROP_CAPACITY:
		 if (bq27742_status(chip->client) == POWER_SUPPLY_STATUS_FULL) {
			val->intval = 100;
		} else {
			val->intval = bq27742_soc(chip->client);
		}
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		/* change the voltage unit from Milli Voltage (mV) to Micro Voltage (uV) */
		val->intval = bq27742_voltage(chip->client) * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		/* change the current unit from Milli Ampere (mA) to Micro Ampere (uA) */
		val->intval = bq27742_current(chip->client) * 1000;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = bq27742_temperature(chip->client);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = bq27742_fcc(chip->client);
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = bq27742_rm(chip->client);
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

static int bq27742_set_property(struct power_supply *psy,
	enum power_supply_property psp,
	const union power_supply_propval *val)
{
	struct bq27742_chip *chip = dev_get_drvdata(psy->dev->parent);

	charging_status = psp;
	cancel_delayed_work(&chip->bq27742_status_work);
	queue_delayed_work(chip->bq27742_delay_wq, &chip->bq27742_status_work,
					BQ27742_UPDATE_TIME);
	return 0;
}

static void bq27742_status_update_work(struct work_struct *work)
{
	struct bq27742_chip *chip = container_of(work,
			struct bq27742_chip,
			bq27742_status_work.work);

	GAUGE_INFO("%s: show battery information\n", __func__);
	switch (bq27742_status(chip->client)) {
		case POWER_SUPPLY_STATUS_CHARGING:
			GAUGE_INFO("status: CHARGING\n");
		break;
		case POWER_SUPPLY_STATUS_DISCHARGING:
			GAUGE_INFO("status: DISCHARGING\n");
		break;
		case POWER_SUPPLY_STATUS_NOT_CHARGING:
			GAUGE_INFO("status: NOT_CHARGING\n");
		break;
		case POWER_SUPPLY_STATUS_FULL:
			GAUGE_INFO("status: FULL\n");
		break;
		case POWER_SUPPLY_STATUS_QUICK_CHARGING:
			GAUGE_INFO("status: QUICK_CHARGING\n");
		break;
		case POWER_SUPPLY_STATUS_NOT_QUICK_CHARGING:
			GAUGE_INFO("status: NOT_QUICK_CHARGING\n");
		break;
		default:
			GAUGE_INFO("status: UNKNOWN\n");
		break;
	}
	GAUGE_INFO("SOC=%d(%%), FCC=%d(mAh), DC=%d(mAh), RM=%d(mAh), TEMP=%d(C), VOLT=%d(mV), CUR=%d(mA), CC=%d, SOH=%d(%%)\n",
		bq27742_soc(chip->client),
		bq27742_fcc(chip->client),
		bq27742_dc(chip->client),
		bq27742_rm(chip->client),
		bq27742_temperature(chip->client)/10,
		bq27742_voltage(chip->client),
		bq27742_current(chip->client),
		bq27742_cc(chip->client),
		bq27742_soh(chip->client)
	);
	power_supply_changed(&chip->psy_battery);
	queue_delayed_work(chip->bq27742_delay_wq, &chip->bq27742_status_work,
					BQ27742_POLLING_TIME);
}

static ssize_t batt_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%04X %04X\n", fw_ver, hw_ver);
}

static int battery_status_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "FCC=%d(mAh),DC=%d(mAh),RM=%d(mAh),TEMP=%d(C),VOLT=%d(mV),CUR=%d(mA),CC=%d,SOH=%d(%%)\n",
		bq27742_fcc(g_bq27742_chip->client),
		bq27742_dc(g_bq27742_chip->client),
		bq27742_rm(g_bq27742_chip->client),
		bq27742_temperature(g_bq27742_chip->client)/10,
		bq27742_voltage(g_bq27742_chip->client),
		bq27742_current(g_bq27742_chip->client),
		bq27742_cc(g_bq27742_chip->client),
		bq27742_soh(g_bq27742_chip->client)
	);
	return 0;
}

static int battery_status_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, battery_status_proc_read, NULL);
}

static struct file_operations battery_status_proc_ops = {
	.open = battery_status_proc_open,
	.read = seq_read,
	.release = single_release,
};

static void create_battery_status_proc_file(void)
{
	static struct proc_dir_entry *battery_status_proc_file;

	battery_status_proc_file = proc_create("battery_soh", 0444, NULL, &battery_status_proc_ops);
	if (battery_status_proc_file) {
		GAUGE_INFO("create battery_status_proc_file sucessed!\n");
	} else {
		GAUGE_INFO("create battery_status_proc_file failed!\n");
	}
}

static int bq27742_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct bq27742_chip *chip;
	int ret;

	GAUGE_INFO("%s +++\n", __func__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA))
		return -EIO;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	chip->client = client;
	i2c_set_clientdata(client, chip);
	g_bq27742_chip = chip;

	chip->psy_battery.name = "battery";
	chip->psy_battery.type = POWER_SUPPLY_TYPE_BATTERY;
	chip->psy_battery.get_property = bq27742_get_property;
	chip->psy_battery.set_property = bq27742_set_property;
	chip->psy_battery.properties = bq27742_battery_props;
	chip->psy_battery.num_properties = ARRAY_SIZE(bq27742_battery_props);

	ret = power_supply_register(&client->dev, &chip->psy_battery);
	if (ret) {
		dev_err(&client->dev, "failed: power supply register\n");
		i2c_set_clientdata(client, NULL);
		kfree(chip);
		return ret;
	}

	sysfs_create_group(&chip->psy_battery.dev->kobj,
			&bq27742_sysfs_attr_group);

	/*initial work queue and works*/
	chip->bq27742_delay_wq = create_workqueue("BQ27742_WORK_QUEUE");
	if (unlikely(!chip->bq27742_delay_wq)) {
		GAUGE_ERR("unable to create bq27742 gauge workqueue\n");
	} else {
		INIT_DELAYED_WORK(&chip->bq27742_status_work, bq27742_status_update_work);
		queue_delayed_work(chip->bq27742_delay_wq, &chip->bq27742_status_work,
					BQ27742_INIT_TIME);
	}

	/* register switch device for battery version info */
	chip->batt_sdev.name = "battery";
	chip->batt_sdev.print_name = batt_switch_name;
	if (switch_dev_register(&chip->batt_sdev) < 0)
		GAUGE_ERR("fail to register battery switch\n");

	/* get version */
	ret = bq27742_write_reg(client, BQ27742_REG_CONTROL, BQ27742_FW_VERSION);
	if (ret < 0) {
		GAUGE_ERR("write to control() failed\n");
		fw_ver = 0xffff;
	} else {
		fw_ver = bq27742_read_reg(client, BQ27742_REG_CONTROL);
		if (fw_ver < 0) {
			GAUGE_ERR("get fw version failed\n");
			fw_ver = 0xffff;
		}
	}
	ret = bq27742_write_reg(client, BQ27742_REG_CONTROL, BQ27742_HW_VERSION);
	if (ret < 0) {
		GAUGE_ERR("write to control() failed\n");
		hw_ver = 0xffff;
	} else {
		hw_ver = bq27742_read_reg(client, BQ27742_REG_CONTROL);
		if (hw_ver < 0) {
			GAUGE_ERR("get hw version failed\n");
			hw_ver = 0xffff;
		}
	}
	GAUGE_INFO("FW version=0x%04X, HW version=0x%04X\n", fw_ver, hw_ver);
	/* Add battery status proc file */
	create_battery_status_proc_file();

	GAUGE_INFO("%s ---\n", __func__);
	return 0;
}

static void bq27742_shutdown(struct i2c_client *client)
{
	struct bq27742_chip *chip = i2c_get_clientdata(client);

	GAUGE_INFO("%s\n", __func__);
	power_supply_unregister(&chip->psy_battery);
	i2c_set_clientdata(client, NULL);
	kfree(chip);
}

#ifdef CONFIG_PM
static int bq27742_suspend(struct device *dev)
{
	struct bq27742_chip *chip = dev_get_drvdata(dev);

	GAUGE_INFO("%s\n", __func__);
	cancel_delayed_work(&chip->bq27742_status_work);
	return 0;
}

static int bq27742_resume(struct device *dev)
{
	struct bq27742_chip *chip = dev_get_drvdata(dev);

	GAUGE_INFO("%s\n", __func__);
	queue_delayed_work(chip->bq27742_delay_wq, &chip->bq27742_status_work,
					BQ27742_UPDATE_TIME);
	return 0;
}

#else
#define bq27742_suspend	NULL
#define bq27742_resume	NULL
#endif

#ifdef CONFIG_PM_RUNTIME
static int bq27742_runtime_suspend(struct device *dev)
{
	GAUGE_INFO("%s\n", __func__);
	return 0;
}

static int bq27742_runtime_resume(struct device *dev)
{
	GAUGE_INFO("%s\n", __func__);
	return 0;
}

static int bq27742_runtime_idle(struct device *dev)
{
	GAUGE_INFO("%s\n", __func__);
	return 0;
}

#else
#define bq27742_runtime_suspend		NULL
#define bq27742_runtime_resume		NULL
#define bbq27742_runtime_idle 		NULL
#endif

static const struct dev_pm_ops bq27742_pm_ops = {
	.suspend = bq27742_suspend,
	.resume = bq27742_resume,
	.runtime_suspend = bq27742_runtime_suspend,
	.runtime_resume = bq27742_runtime_resume,
	.runtime_idle = bq27742_runtime_idle,
};

static const struct i2c_device_id bq27742_id[] = {
	{ "bq27742", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, bq27742_id);

static struct i2c_driver bq27742_i2c_driver = {
	.driver	= {
		.name	= "bq27742",
		.owner	= THIS_MODULE,
		.pm		= &bq27742_pm_ops,
	},
	.id_table		= bq27742_id,
	.probe		= bq27742_probe,
	.shutdown	= bq27742_shutdown,
};

static int __init bq27742_init(void)
{
	int ret;

	GAUGE_INFO("%s\n", __func__);

	ret = i2c_add_driver(&bq27742_i2c_driver);
	if (ret)
		GAUGE_ERR("%s: i2c_add_driver failed\n", __func__);

	return ret;
}
module_init(bq27742_init);

static void __exit bq27742_exit(void)
{
	GAUGE_INFO("%s\n", __func__);
	i2c_del_driver(&bq27742_i2c_driver);
}
module_exit(bq27742_exit);

MODULE_AUTHOR("Chih-Hsuan Chang <Chih-Hsuan_Chang@asus.com>");
MODULE_DESCRIPTION("bq27742 battery monitor driver");
MODULE_LICENSE("GPL");
