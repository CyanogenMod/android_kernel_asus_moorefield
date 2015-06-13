/*
 * CE4100's SPI device is more or less the same one as found on PXA
 *
 */
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/spi/pxa2xx_spi.h>
#include <linux/pm_runtime.h>

static int ce4100_pci_suspend(struct device *dev)
{
	dev_dbg(dev, "suspend called\n");

	return 0;
}

static int ce4100_pci_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "runtime suspend called\n");

	return 0;
}

static int ce4100_pci_resume(struct device *dev)
{
	dev_dbg(dev, "resume called\n");

	return 0;
}

static int ce4100_pci_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "runtime resume called\n");

	return 0;
}

static const struct dev_pm_ops ce4100_pm_ops = {
	.suspend_late = ce4100_pci_suspend,
	.resume_early = ce4100_pci_resume,
	SET_RUNTIME_PM_OPS(ce4100_pci_runtime_suspend,
			   ce4100_pci_runtime_resume,
			   NULL)
};

#define VNDR_CAPABILITY_ADID_OFFSET	6
static inline u8 ssp_cfg_get_spi_bus_nb(u8 ssp_cfg)
{
	return ((ssp_cfg) >> 2) & 0x07;
}

static int ce4100_spi_probe(struct pci_dev *dev,
		const struct pci_device_id *ent)
{
	struct platform_device_info pi;
	int ret;
	struct platform_device *pdev;
	struct pxa2xx_spi_master spi_pdata;
	struct ssp_device *ssp;
	void __iomem **tbl;
	u8 ssp_cfg;
	int pos;

	pos = pci_find_capability(dev, PCI_CAP_ID_VNDR);
	if (pos > 0) {
		pci_read_config_byte(dev,
			pos + VNDR_CAPABILITY_ADID_OFFSET,
			&ssp_cfg);
	} else {
		dev_info(&dev->dev, "No Vendor Specific PCI capability\n");
		return -ENODEV;
	}

	ret = pcim_enable_device(dev);
	if (ret)
		return ret;

	ret = pcim_iomap_regions(dev, 1 << 0, "PXA2xx SPI");
	if (ret)
		return ret;

	memset(&spi_pdata, 0, sizeof(spi_pdata));
	spi_pdata.num_chipselect = 4;

	ssp = &spi_pdata.ssp;
	ssp->phys_base = pci_resource_start(dev, 0);
	tbl = (void __iomem **) pcim_iomap_table(dev);
	if (!tbl) {
		dev_err(&dev->dev, "failed to ioremap() registers\n");
		return -EIO;
	}
	ssp->mmio_base = tbl[0];
	ssp->irq = dev->irq;
	ssp->type = ent->driver_data;
	ssp->port_id = ssp_cfg_get_spi_bus_nb(ssp_cfg);

	memset(&pi, 0, sizeof(pi));
	pi.parent = &dev->dev;
	pi.name = "pxa2xx-spi";
	pi.id = ssp->port_id;
	pi.data = &spi_pdata;
	pi.size_data = sizeof(spi_pdata);

	pdev = platform_device_register_full(&pi);
	if (IS_ERR(pdev))
		return PTR_ERR(pdev);

	pci_set_drvdata(dev, pdev);

	pm_runtime_set_autosuspend_delay(&dev->dev, 50);
	pm_runtime_use_autosuspend(&dev->dev);
	pm_runtime_put_noidle(&dev->dev);
	pm_runtime_allow(&dev->dev);

	return 0;
}

static void ce4100_spi_remove(struct pci_dev *dev)
{
	struct platform_device *pdev = pci_get_drvdata(dev);

	pm_runtime_forbid(&dev->dev);
	pm_runtime_get_noresume(&dev->dev);

	platform_device_unregister(pdev);
}

static DEFINE_PCI_DEVICE_TABLE(ce4100_spi_devices) = {
	{ PCI_VDEVICE(INTEL, 0x2e6a), PXA25x_SSP },
	{ PCI_VDEVICE(INTEL, 0x1194), INTEL_SSP },
};
MODULE_DEVICE_TABLE(pci, ce4100_spi_devices);

static struct pci_driver ce4100_spi_driver = {
	.name           = "ce4100_spi",
	.id_table       = ce4100_spi_devices,
	.probe          = ce4100_spi_probe,
	.remove         = ce4100_spi_remove,
	.driver         = {
		.pm     = &ce4100_pm_ops,
	},
};

module_pci_driver(ce4100_spi_driver);

MODULE_DESCRIPTION("CE4100 PCI-SPI glue code for PXA's driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Sebastian Andrzej Siewior <bigeasy@linutronix.de>");
