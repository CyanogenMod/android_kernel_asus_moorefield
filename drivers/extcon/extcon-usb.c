/*
 * extcon-usb - extcon driver for usb accessory detection
 *		compatible with switch-mid
 *
 * Copyright (C) 2013 Intel Corp.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/extcon.h>
#include <linux/extcon-usb.h>

struct usb_extcon_data {
	struct extcon_dev edev;
};

static const char *name_usb_headsets_anlg = "usb_headsets_anlg_insert";
static const char *name_usb_headsets_dgtl = "usb_headsets_dgtl_insert";
static const char *name_usb_headsets_pull_out = "usb_headsets_pull_out";
static const char *state_usb_hs_dgtl = "2";
static const char *state_usb_hs_anlg = "1";
static const char *state_usb_hs_pull_out = "0";

static struct usb_extcon_data *usb_detect_extcon_data;

void usb_extcon_headset_report(u32 state)
{
	if (usb_detect_extcon_data)
		extcon_set_state(&usb_detect_extcon_data->edev, state);
}
EXPORT_SYMBOL_GPL(usb_extcon_headset_report);

static ssize_t usb_headset_print_name(struct extcon_dev *edev, char *buf)
{
	const char *name;

	if (!buf)
		return -EINVAL;

	switch (extcon_get_state(edev)) {
	case USB_HEADSET_PULL_OUT:
		name = name_usb_headsets_pull_out;
		break;
	case USB_HEADSET_ANLG:
		name = name_usb_headsets_anlg;
		break;
	case USB_HEADSET_DGTL:
		name = name_usb_headsets_dgtl;
		break;
	default:
		name = NULL;
		break;
	}

	if (name)
		return sprintf(buf, "%s\n", name);
	else
		return -EINVAL;
}

static ssize_t usb_headset_print_state(struct extcon_dev *edev, char *buf)
{
	const char *state;

	if (!buf)
		return -EINVAL;

	switch (extcon_get_state(edev)) {
	case USB_HEADSET_PULL_OUT:
		state = state_usb_hs_pull_out;
		break;
	case USB_HEADSET_ANLG:
		state = state_usb_hs_anlg;
		break;
	case USB_HEADSET_DGTL:
		state = state_usb_hs_dgtl;
		break;
	default:
		state = NULL;
		break;
	}

	if (state)
		return sprintf(buf, "%s\n", state);
	else
		return -EINVAL;
}

static int usb_extcon_probe(struct platform_device *pdev)
{
	struct usb_extcon_data *extcon_data;
	int ret = 0;

	extcon_data = devm_kzalloc(&pdev->dev, sizeof(struct usb_extcon_data),
				GFP_KERNEL);
	if (!extcon_data)
		return -ENOMEM;
	usb_detect_extcon_data = extcon_data;

	extcon_data->edev.name = "usb_audio_ext";
	extcon_data->edev.print_name = usb_headset_print_name;
	extcon_data->edev.print_state = usb_headset_print_state;

	ret = extcon_dev_register(&extcon_data->edev, &pdev->dev);
	if (ret < 0)
		goto err_extcon_dev_register;

	platform_set_drvdata(pdev, extcon_data);
	return 0;

err_extcon_dev_register:
	return ret;
}

static int usb_extcon_remove(struct platform_device *pdev)
{
	struct usb_extcon_data *extcon_data = platform_get_drvdata(pdev);

	extcon_dev_unregister(&extcon_data->edev);
	usb_detect_extcon_data = NULL;

	return 0;
}

static struct platform_driver usb_extcon_driver = {
	.probe          = usb_extcon_probe,
	.remove         = usb_extcon_remove,
	.driver         = {
		.name   = "extcon-usb",
		.owner  = THIS_MODULE,
	},
};

module_platform_driver(usb_extcon_driver);

MODULE_AUTHOR("Vamsi krishna Kalidindi (vamsi.krishnax.kalidindi@intel.com)");
MODULE_DESCRIPTION("USB Extcon driver");
MODULE_LICENSE("GPL v2");
