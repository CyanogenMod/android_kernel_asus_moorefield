/*
 * Intel MID Platform Langwell/Penwell OTG EHCI Controller PCI Bus Glue.
 *
 * Copyright (c) 2008 - 2013, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License 2 as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/usb/otg.h>
#include <linux/usb/intel_mid_otg.h>
#include <linux/wakelock.h>
#include <linux/usb/penwell_otg.h>

static int usb_otg_suspend(struct usb_hcd *hcd)
{
	struct usb_phy			*otg;
	struct intel_mid_otg_xceiv	*iotg;

	otg = usb_get_phy(USB_PHY_TYPE_USB2);
	if (otg == NULL) {
		pr_err("%s: failed to get otg transceiver\n", __func__);
		return -EINVAL;
	}
	iotg = otg_to_mid_xceiv(otg);
	pr_info("%s: OTG HNP update suspend\n", __func__);

	atomic_notifier_call_chain(&iotg->iotg_notifier,
				MID_OTG_NOTIFY_HSUSPEND, iotg);
	usb_put_phy(otg);
	return 0;
}

static int usb_otg_resume(struct usb_hcd *hcd)
{
	struct usb_phy			*otg;
	struct intel_mid_otg_xceiv	*iotg;

	otg = usb_get_phy(USB_PHY_TYPE_USB2);
	if (otg == NULL) {
		pr_err("%s: failed to get otg transceiver\n", __func__);
		return -EINVAL;
	}
	iotg = otg_to_mid_xceiv(otg);
	dev_dbg(otg->dev, "%s OTG HNP update resume\n", __func__);

	atomic_notifier_call_chain(&iotg->iotg_notifier,
				MID_OTG_NOTIFY_HRESUME, iotg);
	usb_put_phy(otg);
	return 0;
}

static int ehci_mid_probe(struct pci_dev *pdev,
				const struct pci_device_id *id)
{
	struct hc_driver		*driver;
	struct usb_phy			*otg;
	struct intel_mid_otg_xceiv	*iotg;
	struct intel_mid_otg_pdata	*otg_pdata;
	struct usb_hcd			*hcd;
	struct ehci_hcd			*ehci;
	int				irq;
	int				retval;

	pr_debug("initializing Intel MID USB OTG Host Controller\n");

	/* we need not call pci_enable_dev since otg transceiver already take
	 * the control of this device and this probe actaully gets called by
	 * otg transceiver driver with HNP protocol.
	 */
	irq = pdev->irq;

	if (!id)
		return -EINVAL;
	driver = (struct hc_driver *)id->driver_data;
	if (!driver)
		return -EINVAL;

	hcd = usb_create_hcd(driver, &pdev->dev, dev_name(&pdev->dev));
	if (!hcd) {
		retval = -ENOMEM;
		goto err1;
	}

	ehci = hcd_to_ehci(hcd);
	/* this will be called in ehci_bus_suspend and ehci_bus_resume */
	ehci->otg_suspend = usb_otg_suspend;
	ehci->otg_resume = usb_otg_resume;
	otg = usb_get_phy(USB_PHY_TYPE_USB2);
	if (otg == NULL) {
		pr_err("%s:  failed to get otg transceiver\n", __func__);
		retval = -EINVAL;
		goto err1;
	}

	iotg = otg_to_mid_xceiv(otg);
	hcd->regs = iotg->base;

	hcd->rsrc_start = pci_resource_start(pdev, 0);
	hcd->rsrc_len = pci_resource_len(pdev, 0);

	if (hcd->regs == NULL) {
		dev_dbg(&pdev->dev, "error mapping memory\n");
		retval = -EFAULT;
		goto err2;
	}

	otg_pdata = pdev->dev.platform_data;
	if (otg_pdata == NULL) {
		dev_err(&pdev->dev, "Failed to get OTG platform data.\n");
		retval = -ENODEV;
		goto err2;
	}
	hcd->power_budget = otg_pdata->power_budget;

	/* Mandatorily set the controller as remote-wakeup enabled */
	device_set_wakeup_enable(&pdev->dev, true);

	retval = usb_add_hcd(hcd, irq, IRQF_DISABLED | IRQF_SHARED);
	if (retval != 0)
		goto err2;
	retval = otg_set_host(otg->otg, &hcd->self);
	if (!otg->otg->default_a)
		hcd->self.is_b_host = 1;
	usb_put_phy(otg);
	return retval;

err2:
	usb_put_hcd(hcd);
err1:
	dev_err(&pdev->dev, "init %s fail, %d\n", dev_name(&pdev->dev), retval);
	return retval;
}

void ehci_mid_remove(struct pci_dev *dev)
{
	struct usb_hcd *hcd = pci_get_drvdata(dev);

	if (!hcd)
		return;
	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);
}

/* Intel MID OTG EHCI driver */
static struct pci_driver ehci_otg_driver = {
	.name =	"ehci-intel-mid",
	.id_table =	pci_ids,

	.probe =	ehci_mid_probe,
	.remove =	ehci_mid_remove,

#ifdef CONFIG_PM_SLEEP
	.driver =	{
		.pm =	&usb_hcd_pci_pm_ops
	},
#endif
	.shutdown =	usb_hcd_pci_shutdown,
};

static int ehci_mid_start_host(struct intel_mid_otg_xceiv *iotg)
{
	struct pci_dev	*pdev;
	int		retval;

	if (iotg == NULL)
		return -EINVAL;

	pdev = to_pci_dev(iotg->otg.dev);

	retval = ehci_mid_probe(pdev, ehci_otg_driver.id_table);
	if (retval)
		dev_dbg(iotg->otg.dev, "Failed to start host\n");

	return retval;
}

static int ehci_mid_stop_host(struct intel_mid_otg_xceiv *iotg)
{
	struct pci_dev	*pdev;

	if (iotg == NULL)
		return -EINVAL;

	pdev = to_pci_dev(iotg->otg.dev);

	ehci_mid_remove(pdev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int ehci_mid_suspend_host(struct intel_mid_otg_xceiv *iotg)
{
	int		retval;

	if (iotg == NULL)
		return -EINVAL;

	if (ehci_otg_driver.driver.pm == NULL)
		return -EINVAL;

	retval = ehci_otg_driver.driver.pm->suspend(iotg->otg.dev);
	if (retval)
		dev_warn(iotg->otg.dev, "suspend failed, return %d\n", retval);

	return retval;

}

static int ehci_mid_suspend_noirq_host(struct intel_mid_otg_xceiv *iotg)
{
	int		retval;

	if (iotg == NULL)
		return -EINVAL;

	if (ehci_otg_driver.driver.pm == NULL)
		return -EINVAL;

	retval = ehci_otg_driver.driver.pm->suspend_noirq(iotg->otg.dev);
	if (retval)
		dev_warn(iotg->otg.dev, "suspend_noirq failed, return %d\n",
				retval);

	return retval;

}

static int ehci_mid_resume_host(struct intel_mid_otg_xceiv *iotg)
{
	int		retval;

	if (iotg == NULL)
		return -EINVAL;

	if (ehci_otg_driver.driver.pm == NULL)
		return -EINVAL;

	retval = ehci_otg_driver.driver.pm->resume(iotg->otg.dev);
	if (retval)
		dev_warn(iotg->otg.dev, "resume failed, return %d\n", retval);

	return retval;
}

static int ehci_mid_resume_noirq_host(struct intel_mid_otg_xceiv *iotg)
{
	int		retval;

	if (iotg == NULL)
		return -EINVAL;

	if (ehci_otg_driver.driver.pm == NULL)
		return -EINVAL;

	retval = ehci_otg_driver.driver.pm->resume_noirq(iotg->otg.dev);
	if (retval)
		dev_warn(iotg->otg.dev, "resume_noirq failed, return %d\n",
				retval);

	return retval;
}
#else

#define ehci_mid_suspend_host NULL
#define ehci_mid_suspend_noirq_host NULL
#define ehci_mid_resume_host NULL
#define ehci_mid_resume_noirq_host NULL

#endif

#ifdef CONFIG_PM_RUNTIME
static int ehci_mid_runtime_suspend_host(struct intel_mid_otg_xceiv *iotg)
{
	int		retval;

	if (iotg == NULL)
		return -EINVAL;

	if (ehci_otg_driver.driver.pm == NULL)
		return -EINVAL;

	retval = ehci_otg_driver.driver.pm->runtime_suspend(iotg->otg.dev);
	if (retval)
		dev_warn(iotg->otg.dev, "runtime suspend failed\n");

	return retval;
}

static int ehci_mid_runtime_resume_host(struct intel_mid_otg_xceiv *iotg)
{
	int		retval;
	struct device	*dev;
	struct pci_dev	*pci_dev;
	struct usb_hcd	*hcd;


	if (iotg == NULL)
		return -EINVAL;

	if (ehci_otg_driver.driver.pm == NULL)
		return -EINVAL;

	dev = iotg->otg.dev;
	pci_dev = to_pci_dev(dev);
	hcd = pci_get_drvdata(pci_dev);

	retval = ehci_otg_driver.driver.pm->runtime_resume(dev);
	if (retval)
		dev_warn(dev, "runtime suspend failed\n");

	/* Workaround: EHCI currently unable to deal with interrupt
	*  when remote wakeup happens,
	*  so a manuallly root-hub resuming is performed here.
	*/
	if (!retval) {
		dev_dbg(dev, "resume root hub\n");
		usb_hcd_resume_root_hub(hcd);
	}
	return retval;
}
#else

#define ehci_mid_runtime_suspend_host NULL
#define ehci_mid_runtime_resume_host NULL

#endif

static int intel_mid_ehci_driver_register(struct pci_driver *host_driver)
{
	struct usb_phy			*otg;
	struct intel_mid_otg_xceiv	*iotg;

	otg = usb_get_phy(USB_PHY_TYPE_USB2);
	if (otg == NULL || host_driver == NULL)
		return -EINVAL;

	iotg = otg_to_mid_xceiv(otg);
	iotg->start_host = ehci_mid_start_host;
	iotg->stop_host = ehci_mid_stop_host;
	iotg->runtime_suspend_host = ehci_mid_runtime_suspend_host;
	iotg->runtime_resume_host = ehci_mid_runtime_resume_host;

	iotg->suspend_host = ehci_mid_suspend_host;
	iotg->suspend_noirq_host = ehci_mid_suspend_noirq_host;
	iotg->resume_host = ehci_mid_resume_host;
	iotg->resume_noirq_host = ehci_mid_resume_noirq_host;

	/* notify host driver is registered */
	atomic_notifier_call_chain(&iotg->iotg_notifier,
				MID_OTG_NOTIFY_HOSTADD, iotg);

	usb_put_phy(otg);

	return 0;
}

static void intel_mid_ehci_driver_unregister(struct pci_driver *host_driver)
{
	struct usb_phy			*otg;
	struct intel_mid_otg_xceiv	*iotg;

	otg = usb_get_phy(USB_PHY_TYPE_USB2);
	if (otg == NULL)
		return;

	iotg = otg_to_mid_xceiv(otg);
	iotg->start_host = NULL;
	iotg->stop_host = NULL;
	iotg->runtime_suspend_host = NULL;
	iotg->runtime_resume_host = NULL;

	/* notify host driver is unregistered */
	atomic_notifier_call_chain(&iotg->iotg_notifier,
				MID_OTG_NOTIFY_HOSTREMOVE, iotg);

	usb_put_phy(otg);
}

