/*
 * Intel MID Platform EHCI SPH Controller PCI Bus Glue.
 *
 * Copyright (c) 2013, Intel Corporation.
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

#include <linux/pci.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/usb/ehci_sph_pci.h>
#include <linux/gpio.h>

static void sph_phy_enable(struct pci_dev *pdev)
{
	struct ehci_sph_pdata	*sph_pdata;

	sph_pdata = pdev->dev.platform_data;

	if (!sph_pdata)
		return;
	if (!sph_pdata->has_gpio)
		return;
	gpio_direction_output(sph_pdata->gpio_cs_n, 0);

	gpio_direction_output(sph_pdata->gpio_reset_n, 0);
	usleep_range(200, 500);
	gpio_set_value(sph_pdata->gpio_reset_n, 1);
}

static void sph_phy_disable(struct pci_dev *pdev)
{
	struct ehci_sph_pdata	*sph_pdata;

	sph_pdata = pdev->dev.platform_data;

	if (!sph_pdata)
		return;
	if (!sph_pdata->has_gpio)
		return;

	gpio_direction_output(sph_pdata->gpio_cs_n, 1);
}

static int sph_gpio_init(struct pci_dev *pdev)
{
	struct ehci_sph_pdata	*sph_pdata;
	int			retval = 0;

	sph_pdata = pdev->dev.platform_data;

	if (!sph_pdata) {
		retval = -ENODEV;
		goto ret;
	}

	if (!sph_pdata->has_gpio)
		goto ret;

	if (gpio_is_valid(sph_pdata->gpio_cs_n)) {
		retval = gpio_request(sph_pdata->gpio_cs_n, "sph_phy_cs_n");
		if (retval < 0) {
			dev_err(&pdev->dev, "Request GPIO %d with error %d\n",
			sph_pdata->gpio_cs_n, retval);
			retval = -ENODEV;
			goto ret;
		}
	} else {
		retval = -ENODEV;
		goto ret;
	}

	if (gpio_is_valid(sph_pdata->gpio_reset_n)) {
		retval = gpio_request(sph_pdata->gpio_reset_n,
				"sph_phy_reset_n");
		if (retval < 0) {
			dev_err(&pdev->dev, "Request GPIO %d with error %d\n",
			sph_pdata->gpio_reset_n, retval);
			retval = -ENODEV;
			goto err;
		}
	} else {
		retval = -ENODEV;
		goto err;
	}

	sph_phy_enable(pdev);

	return retval;
err:
	gpio_free(sph_pdata->gpio_cs_n);

ret:
	return retval;
}

static void sph_gpio_cleanup(struct pci_dev *pdev)
{
	struct ehci_sph_pdata	*sph_pdata;

	sph_pdata = pdev->dev.platform_data;

	if (!sph_pdata)
		return;
	if (!sph_pdata->has_gpio)
		return;
	if (gpio_is_valid(sph_pdata->gpio_cs_n))
		gpio_free(sph_pdata->gpio_cs_n);
	if (gpio_is_valid(sph_pdata->gpio_reset_n))
		gpio_free(sph_pdata->gpio_reset_n);
}

static int sph_set_enabled(struct pci_dev *pdev)
{
	struct ehci_sph_pdata   *sph_pdata;
	int			retval = 0;

	sph_pdata = pdev->dev.platform_data;

	if (!sph_pdata) {
		retval = -ENODEV;
		return retval;
	}

	sph_pdata->enabled = sph_enabled();
	return retval;
}
static int ehci_sph_probe(struct pci_dev *pdev,
				const struct pci_device_id *id)
{
	struct hc_driver	*driver;
	struct usb_hcd		*hcd;
	int			retval;

	pr_debug("Initializing Intel MID USB SPH Host Controller\n");

	if (!id)
		return -EINVAL;
	driver = (struct hc_driver *)id->driver_data;
	if (!driver)
		return -EINVAL;

	retval = sph_gpio_init(pdev);
	if (retval < 0)
		return retval;

	/* set the sph_pdata->enabled firstly */
	retval = sph_set_enabled(pdev);
	if (retval < 0)
		return retval;

	if (pci_enable_device(pdev) < 0)
		return -ENODEV;
	pdev->current_state = PCI_D0;

	if (!pdev->irq) {
		dev_dbg(&pdev->dev, "No IRQ.\n");
		retval = -ENODEV;
		goto disable_pci;
	}

	hcd = usb_create_hcd(driver, &pdev->dev, dev_name(&pdev->dev));
	if (!hcd) {
		retval = -ENOMEM;
		goto disable_pci;
	}

	hcd->rsrc_start = pci_resource_start(pdev, 0);
	hcd->rsrc_len = pci_resource_len(pdev, 0);
	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len,
			driver->description)) {
		dev_dbg(&pdev->dev, "controller already in use\n");
		retval = -EBUSY;
		goto put_hcd;
	}

	hcd->regs = ioremap_nocache(hcd->rsrc_start, hcd->rsrc_len);
	if (hcd->regs == NULL) {
		dev_dbg(&pdev->dev, "error mapping memory\n");
		retval = -EFAULT;
		goto release_mem_region;
	}

	pci_set_master(pdev);

	retval = usb_add_hcd(hcd, pdev->irq, IRQF_DISABLED | IRQF_SHARED);
	if (retval != 0)
		goto unmap_registers;

	pm_runtime_put_noidle(&pdev->dev);
	pm_runtime_allow(&pdev->dev);

	return retval;

unmap_registers:
		iounmap(hcd->regs);
release_mem_region:
		release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
put_hcd:
	dev_set_drvdata(&pdev->dev, NULL);
	usb_put_hcd(hcd);
disable_pci:
	sph_gpio_cleanup(pdev);
	pci_disable_device(pdev);
	dev_err(&pdev->dev, "init %s fail, %d\n", dev_name(&pdev->dev), retval);
	return retval;
}

static void ehci_sph_remove(struct pci_dev *pdev)
{
	struct usb_hcd *hcd = pci_get_drvdata(pdev);

	if (!hcd)
		return;

	pm_runtime_get_noresume(&pdev->dev);
	pm_runtime_forbid(&pdev->dev);

	/* Fake an interrupt request in order to give the driver a chance
	 * to test whether the controller hardware has been removed (e.g.,
	 * cardbus physical eject).
	 */
	local_irq_disable();
	usb_hcd_irq(0, hcd);
	local_irq_enable();

	usb_remove_hcd(hcd);
	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);

	dev_set_drvdata(&pdev->dev, NULL);
	usb_put_hcd(hcd);
	sph_gpio_cleanup(pdev);
	pci_disable_device(pdev);
}

#ifdef CONFIG_PM_SLEEP
static int sph_pci_suspend_noirq(struct device *dev)
{
	struct pci_dev		*pci_dev = to_pci_dev(dev);
	struct usb_hcd		*hcd = pci_get_drvdata(pci_dev);
	struct ehci_hcd		*ehci = hcd_to_ehci(hcd);
	u32			temp;
	int			retval;

	dev_dbg(dev, "%s --->\n", __func__);

	temp = ehci_readl(ehci, hcd->regs + CLV_SPHCFG);
	/* ULPI 1 ref-clock switch off for power saving */
	temp |= CLV_SPHCFG_REFCKDIS;
	ehci_writel(ehci, temp, hcd->regs + CLV_SPHCFG);
	temp = ehci_readl(ehci, hcd->regs + CLV_SPHCFG);
	dev_dbg(dev, "%s ULPI1 ref-clock switch off, SPHCFG = 0x%08X\n",
			__func__, temp);

	retval = usb_hcd_pci_pm_ops.suspend_noirq(dev);
	if (!retval) {
		sph_phy_disable(pci_dev);
		dev_dbg(dev, "%s Disable SPH PHY\n", __func__);
	} else {
		temp = ehci_readl(ehci, hcd->regs + CLV_SPHCFG);
		/* ULPI 1 ref-clock switch on if failed */
		temp &= ~CLV_SPHCFG_REFCKDIS;
		ehci_writel(ehci, temp, hcd->regs + CLV_SPHCFG);
	}
	dev_dbg(dev, "%s <--- retval = %d\n", __func__, retval);
	return retval;
}

static int sph_pci_suspend(struct device *dev)
{
	int	retval;

	dev_dbg(dev, "%s --->\n", __func__);
	retval = usb_hcd_pci_pm_ops.suspend(dev);
	dev_dbg(dev, "%s <--- retval = %d\n", __func__, retval);
	return retval;
}

static int sph_pci_resume_noirq(struct device *dev)
{
	struct pci_dev		*pci_dev = to_pci_dev(dev);
	struct usb_hcd		*hcd = pci_get_drvdata(pci_dev);
	struct ehci_hcd		*ehci = hcd_to_ehci(hcd);
	int			retval;
	u32			temp;

	dev_dbg(dev, "%s --->\n", __func__);

	dev_dbg(dev, "%s Enable SPH PHY\n", __func__);
	sph_phy_enable(pci_dev);

	retval = usb_hcd_pci_pm_ops.resume_noirq(dev);

	temp = ehci_readl(ehci, hcd->regs + CLV_SPHCFG);

	/* ULPI 1 ref-clock switch on after S3 */
	temp &= ~CLV_SPHCFG_REFCKDIS;
	ehci_writel(ehci, temp, hcd->regs + CLV_SPHCFG);

	temp = ehci_readl(ehci, hcd->regs + CLV_SPHCFG);
	dev_dbg(dev, "%s ULPI1 ref-clock switch on, SPHCFG = 0x%08X\n",
			__func__, temp);

	dev_dbg(dev, "%s <--- retval = %d\n", __func__, retval);
	return retval;
}

static int sph_pci_resume(struct device *dev)
{
	int	retval;

	dev_dbg(dev, "%s --->\n", __func__);
	retval = usb_hcd_pci_pm_ops.resume(dev);
	dev_dbg(dev, "%s <--- retval = %d\n", __func__, retval);
	return retval;
}

#else
#define sph_pci_suspend_noirq	NULL
#define sph_pci_suspend		NULL
#define sph_pci_resume_noirq	NULL
#define sph_pci_resume		NULL
#endif

#ifdef CONFIG_PM_RUNTIME
static int sph_pci_runtime_suspend(struct device *dev)
{
	int	retval;

	dev_dbg(dev, "%s --->\n", __func__);
	retval = usb_hcd_pci_pm_ops.runtime_suspend(dev);
	dev_dbg(dev, "%s <--- retval = %d\n", __func__, retval);
	return retval;
}

static int sph_pci_runtime_resume(struct device *dev)
{
	int			retval;

	dev_dbg(dev, "%s --->\n", __func__);
	retval = usb_hcd_pci_pm_ops.runtime_resume(dev);

	dev_dbg(dev, "%s <--- retval = %d\n", __func__, retval);
	return retval;
}
#else
#define sph_pci_runtime_suspend	NULL
#define sph_pci_runtime_resume	NULL
#endif

static DEFINE_PCI_DEVICE_TABLE(sph_pci_ids) = {
	{
		.vendor =	PCI_VENDOR_ID_INTEL,
		.device =	PCI_DEVICE_ID_INTEL_CLV_SPH,
		.subvendor =	PCI_ANY_ID,
		.subdevice =	PCI_ANY_ID,
		.driver_data =  (unsigned long) &ehci_pci_hc_driver,
	},
	{ /* end: all zeroes */ }
};

static const struct dev_pm_ops sph_pm_ops = {
	.suspend	= sph_pci_suspend,
	.suspend_noirq	= sph_pci_suspend_noirq,
	.resume		= sph_pci_resume,
	.resume_noirq	= sph_pci_resume_noirq,
	.runtime_suspend = sph_pci_runtime_suspend,
	.runtime_resume	= sph_pci_runtime_resume,
};

static struct pci_driver ehci_sph_driver = {
	.name =	"ehci_sph",
	.id_table =	sph_pci_ids,

	.probe =	ehci_sph_probe,
	.remove =	ehci_sph_remove,

#ifdef CONFIG_PM_SLEEP
	.driver =	{
		.pm = &sph_pm_ops
	},
#endif
	.shutdown =	usb_hcd_pci_shutdown,
};
