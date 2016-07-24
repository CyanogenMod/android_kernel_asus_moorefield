/*
 * intel_byt_buttons.c - Baytrail-M key buttons driver
 *
 * Copyright (C) 2013 Intel Corporation
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
 * Author: Ning Li <ning.li@intel.com>
 */


#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/notifier.h>
#include <linux/platform_device.h>
#include <asm/intel_byt_ec.h>
#include <asm/intel_byt_buttons.h>
#include <acpi/acpi_bus.h>
#include <acpi/acpi_drivers.h>

#define DRIVER_NAME "byt_buttons"

#define POWER_BTN_STAT_MASK	(1 << 0)
#define VOLUP_BTN_STAT_MASK	(1 << 1)
#define VOLDOWN_BTN_STAT_MASK	(1 << 2)
#define HOME_BTN_STAT_MASK	(1 << 3)

struct byt_buttons_priv {
	struct input_dev *input;
	struct notifier_block nb;
	struct byt_keys_button *buttons;
	u8 last_stat;
	struct mutex btn_mutex;
};

static int byt_ec_evt_btn_callback(struct notifier_block *nb,
					unsigned long event, void *data)
{
	u8 stat;
	u8 valid;
	int ret = NOTIFY_OK;
	struct byt_buttons_priv *priv;

	priv = container_of(nb, struct byt_buttons_priv, nb);

	switch (event) {
	case BYT_EC_SCI_VOLUMEUP_BTN:
	case BYT_EC_SCI_VOLUMEDOWN_BTN:
	case BYT_EC_SCI_HOME_BTN:
	case BYT_EC_SCI_POWER_BTN:
		dev_dbg(&priv->input->dev, "ec button event\n");
		mutex_lock(&priv->btn_mutex);
		ret = byt_ec_read_byte(BYT_EC_BUTTON_STATUS, &stat);
		if (ret) {
			dev_err(&priv->input->dev,
				"Query button status failed\n");
			mutex_unlock(&priv->btn_mutex);
			return NOTIFY_DONE;
		}

		valid = stat ^ priv->last_stat;

		if (valid & VOLUP_BTN_STAT_MASK) {
			input_event(priv->input, EV_KEY, KEY_VOLUMEUP,
				!(stat & VOLUP_BTN_STAT_MASK));
		}
		if (valid & VOLDOWN_BTN_STAT_MASK) {
			input_event(priv->input, EV_KEY, KEY_VOLUMEDOWN,
				!(stat & VOLDOWN_BTN_STAT_MASK));
		}
		if (valid & POWER_BTN_STAT_MASK) {
			input_event(priv->input, EV_KEY, KEY_POWER,
				!(stat & POWER_BTN_STAT_MASK));
			acpi_clear_event(ACPI_EVENT_POWER_BUTTON);
		}
		if (valid & HOME_BTN_STAT_MASK) {
			input_event(priv->input, EV_KEY, KEY_HOME,
				!(stat & HOME_BTN_STAT_MASK));
		}

		input_sync(priv->input);
		priv->last_stat = stat;

		mutex_unlock(&priv->btn_mutex);
		break;
	default:
		dev_err(&priv->input->dev, "Invalid event\n");
		ret = NOTIFY_DONE;
	}

	return ret;
}

static int byt_buttons_probe(struct platform_device *pdev)
{
	struct input_dev *input;
	struct byt_buttons_priv *priv;
	int i;
	int ret;
	struct byt_keys_platform_data *pdata = pdev->dev.platform_data;
	struct byt_keys_button *button;

	if (pdata == NULL) {
		dev_err(&pdev->dev, "No button platform data\n");
		return -EINVAL;
	}

	priv = kzalloc(sizeof(struct byt_buttons_priv), GFP_KERNEL);
	input = input_allocate_device();
	if (!priv || !input)
		return -ENOMEM;

	priv->input = input;
	platform_set_drvdata(pdev, priv);

	input->name = pdev->name;
	input->phys = "buttons/input0";
	input->id.bustype = BUS_HOST;
	input->dev.parent = &pdev->dev;

	if (pdata->rep)
		__set_bit(EV_REP, input->evbit);

	for (i = 0; i < pdata->nbuttons; i++) {
		button = &pdata->buttons[i];
		input_set_capability(input, button->type ?: EV_KEY,
					button->code);
	}

	ret = input_register_device(input);
	if (ret) {
		dev_err(&pdev->dev,
			"unable to register input dev, error %d\n", ret);
		goto err;
	}

	priv->buttons = pdata->buttons;
	priv->last_stat = POWER_BTN_STAT_MASK |
			  VOLUP_BTN_STAT_MASK |
			  VOLDOWN_BTN_STAT_MASK |
			  HOME_BTN_STAT_MASK;

	mutex_init(&priv->btn_mutex);

	/* Register notifier call as EC SCI events */
	priv->nb.notifier_call = &byt_ec_evt_btn_callback;
	byt_ec_evt_register_notify(&priv->nb);

	dev_info(&pdev->dev, "Probed %s devivce\n", pdev->name);
	return 0;

err:
	platform_set_drvdata(pdev, NULL);
	input_free_device(input);
	kfree(priv);
	return ret;
}

static int byt_buttons_remove(struct platform_device *pdev)
{
	struct byt_buttons_priv *priv = platform_get_drvdata(pdev);

	byt_ec_evt_unregister_notify(&priv->nb);
	platform_set_drvdata(pdev, NULL);
	input_unregister_device(priv->input);
	kfree(priv);

	return 0;
}

static const struct platform_device_id byt_buttons_table[] = {
	{"byt_m_rpt_btns", 1},
	{"byt_m_nrpt_btns", 1},
};

static struct platform_driver byt_buttons_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.probe	= byt_buttons_probe,
	.remove	= byt_buttons_remove,
	.id_table = byt_buttons_table,
};

static int __init byt_buttons_module_init(void)
{
	return platform_driver_register(&byt_buttons_driver);
}

static void __exit byt_buttons_module_exit(void)
{
	platform_driver_unregister(&byt_buttons_driver);
}

device_initcall(byt_buttons_module_init);
module_exit(byt_buttons_module_exit);

MODULE_AUTHOR("Ning Li <ning.li@intel.com>");
MODULE_DESCRIPTION("Intel Baytrail-M Buttons Driver");
MODULE_LICENSE("GPL v2");

