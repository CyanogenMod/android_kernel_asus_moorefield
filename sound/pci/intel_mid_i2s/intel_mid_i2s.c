/*
  * <Driver for I2S protocol on SSP (Moorestown and Medfield hardware)>
  * Copyright (c) 2010, Intel Corporation.
  * Louis LE GALL <louis.le.gall intel.com>
  *
  * This program is free software; you can redistribute it and/or modify it
  * under the terms and conditions of the GNU General Public License,
  * version 2, as published by the Free Software Foundation.
  *
  * This program is distributed in the hope it will be useful, but WITHOUT
  * ANY WARRANTY; without evenp the implied warranty of MERCHANTABILITY or
  * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  * more details.
  *
  * You should have received a copy of the GNU General Public License along with
  * this program; if not, write to the Free Software Foundation, Inc.,
  * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
  */
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/pm_runtime.h>
#include <linux/pci_regs.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/acpi.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/gpio.h>
#include <linux/scatterlist.h>
#include <linux/list.h>
#include <linux/async.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/lnw_gpio.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/intel_mid_i2s_common.h>
#include <linux/intel_mid_i2s_if.h>
#include <asm/intel-mid.h>

#include "intel_mid_i2s.h"

#define VERSION_SSP "1.3.0"

MODULE_AUTHOR("Louis LE GALL <louis.le.gall intel.com>");
MODULE_DESCRIPTION("Intel MID I2S/PCM SSP Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(VERSION_SSP);

#define SSPCODEC "SSPC0000"
#define SSPMODEM "SSPM0000"
#define SSPBLUET "SSPB0000"
#define SSPACPI5 "SSPX0000"

/* ACPI arguments */
#define AARG_NA 	0x00
#define AARG_FREQ8	0x08
#define AARG_FREQ16	0x10
#define AARG_FREQ48	0x30

#define ACPI_UUID_NBPARAMS	4

#define CLOCK_19200_KHZ		19200000

#define UNASSIGNED_GPIO_MAPPING 0xffff

/* FIXME: use of lpeshim_base_address should be
 * avoided and replaced by a call to SST driver that will
 * take care to access LPE Shim registers */

/* LPE Shim registers base address (needed for HW IRQ acknowledge) */
static void __iomem *lpeshim_base_address;

/*
 * Currently this limit to ONE modem on platform
 */
static unsigned long modem_found_and_i2s_setup_ok;
static unsigned long ssps_found;

static unsigned long ssp_ip_features;

static void(*intel_mid_i2s_modem_probe_cb)(void);
static void(*intel_mid_i2s_modem_remove_cb)(void);

/*
 * structures for pci probing
 */
#ifdef CONFIG_PM
static const struct dev_pm_ops intel_mid_i2s_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(intel_mid_i2s_suspend,
				intel_mid_i2s_resume)
	.runtime_suspend = intel_mid_i2s_runtime_suspend,
	.runtime_resume = intel_mid_i2s_runtime_resume,
};
#endif

/************** PCI **************/

static DEFINE_PCI_DEVICE_TABLE(pci_ids) = {
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, MFLD_SSP1_DEVICE_ID) },
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, MFLD_SSP0_DEVICE_ID) },
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, CLV_SSP1_DEVICE_ID) },
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, CLV_SSP0_DEVICE_ID) },
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, MRFL_SSP_DEVICE_ID) },
	{ 0, }, /* terminate list */
};

static struct pci_driver intel_mid_i2s_driver = {
#ifdef CONFIG_PM
	.driver = {
		.pm = &intel_mid_i2s_pm_ops,
	},
#endif
	.name = DRIVER_NAME,
	.id_table = pci_ids,
	.probe = intel_mid_i2s_probe,
	.remove = intel_mid_i2s_remove,
};

/************* ACPI **************/
#ifdef CONFIG_ACPI
static const struct acpi_device_id i2s_acpi_ids[] = {
/*	{"XXXNAMEXXX", (kernel_ulong_t) &intel_byt_info },*/
	{SSPCODEC, 0 },
	{SSPMODEM, 0 },
	{SSPBLUET, 0 },
	{SSPACPI5, 0 },
	{"", 0 },
};
MODULE_DEVICE_TABLE(acpi, i2s_acpi_ids);

static struct platform_driver i2s_acpi_driver = {
	.driver = {
		.name			= "intel_i2s_acpi",
		.owner			= THIS_MODULE,
		.acpi_match_table	= ACPI_PTR(i2s_acpi_ids),
		.pm			= &intel_mid_i2s_pm_ops,
	},
	.probe = i2s_acpi_probe,
	.remove = i2s_acpi_remove,
};

/* Configuration of timing in DSDT ACPI tables */

static struct name_to_config_type config_assoc_table[] = {
	{ "PCM0", SSPCONF_PCM0},
	{ "PCM1", SSPCONF_PCM1},
	{ "I2S0", SSPCONF_I2S0},
	{ "", SSPCONF_NONE},
};

static struct intel_mid_i2s_ssp_config ssp_config[] = {
	[SSPCONF_PCM0] = {
		.ssp_psp_T1 = 0,
		.ssp_psp_T2 = 0,
		.ssp_psp_T4 = 0,
		.ssp_psp_T5 = 0,
		.ssp_psp_T6 = 1,
	},
	[SSPCONF_PCM1] = {
		.ssp_psp_T1 = 0,
		.ssp_psp_T2 = 1,
		.ssp_psp_T4 = 0,
		.ssp_psp_T5 = 0,
		.ssp_psp_T6 = 1,
	},
	[SSPCONF_I2S0] = {
		.ssp_psp_T1 = 6,
		.ssp_psp_T2 = 2,
		.ssp_psp_T4 = 0,
		.ssp_psp_T5 = 14,
		.ssp_psp_T6 = 16,
	},
};

#endif

/*
 * Local functions declaration
 */
#ifdef _LLI_ENABLED_
static inline void i2s_enable(struct intel_mid_i2s_hdl *drv_data, bool enable);
static inline void i2s_enable_dma_rx_intr(struct intel_mid_i2s_hdl *drv_data,
					  bool enable);
static inline void i2s_enable_dma_tx_intr(struct intel_mid_i2s_hdl *drv_data,
					  bool enable);
#else
static inline void i2s_enable(struct intel_mid_i2s_hdl *drv_data);
static inline void i2s_disable(struct intel_mid_i2s_hdl *drv_data);
#endif /* _LLI_ENABLED_ */

static void i2s_finalize_read(struct intel_mid_i2s_hdl *drv_data);
static void i2s_finalize_write(struct intel_mid_i2s_hdl *drv_data);

/*
 * POWER MANAGEMENT FUNCTIONS
 */

#ifdef CONFIG_PM
/**
 * intel_mid_i2s_driver_suspend - driver power management suspend activity
 * @dev : pointer of the device to suspend
 *
 * Output parameters
 *      error : 0 means no error
 */
static int intel_mid_i2s_suspend(struct device *dev)
{
	struct intel_mid_i2s_hdl *drv_data = dev_get_drvdata(dev);
	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return 0;
	dev_dbg(drv_data->ssp_dev, "SUSPEND SSP\n");

	return 0;
}

/**
 * intel_mid_i2s_driver_resume - driver power management suspend activity
 * @device_ptr : pointer of the device to resume
 *
 * Output parameters
 *      error : 0 means no error
 */
static int intel_mid_i2s_resume(struct device *dev)
{
	struct intel_mid_i2s_hdl *drv_data = dev_get_drvdata(dev);

	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return -EFAULT;
	dev_dbg(drv_data->ssp_dev, "Trying to *resume SSP device\n");

	dev_dbg(drv_data->ssp_dev, "resumed\n");
	return 0;
}

/**
 * intel_mid_i2s_runtime_suspend - runtime power management suspend activity
 * @device_ptr : pointer of the device to resume
 *
 * Output parameters
 *      error : 0 means no error
 */
static int intel_mid_i2s_runtime_suspend(struct device *device_ptr)
{
	struct pci_dev *pdev;
	struct intel_mid_i2s_hdl *drv_data;
	void __iomem *reg;

	pdev = to_pci_dev(device_ptr);
	WARN(!pdev, "Pci dev=NULL\n");
	if (!pdev)
		return -EFAULT;
	drv_data = (struct intel_mid_i2s_hdl *) pci_get_drvdata(pdev);
	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return -EFAULT;
	if (test_bit(I2S_PORT_OPENED, &drv_data->flags)) {
		dev_err(device_ptr,
			"Trying to suspend a device that is opened\n");
		return -ENODEV;
	}
	reg = drv_data->ioaddr;
	dev_dbg(drv_data->ssp_dev, "Suspend of SSP requested !!\n");
	return intel_mid_i2s_suspend(device_ptr);
}

/**
 * intel_mid_i2s_runtime_resume - runtime power management resume activity
 * @device_ptr : pointer of the device to resume
 *
 * Output parameters
 *      error : 0 means no error
 */
static int intel_mid_i2s_runtime_resume(struct device *device_ptr)
{
	struct pci_dev *pdev;
	struct intel_mid_i2s_hdl *drv_data;
	pdev = to_pci_dev(device_ptr);
	WARN(!pdev, "Pci dev=NULL\n");
	if (!pdev)
		return -EFAULT;
	drv_data = (struct intel_mid_i2s_hdl *) pci_get_drvdata(pdev);
	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return -EFAULT;
	dev_dbg(drv_data->ssp_dev, "RT RESUME SSP ID\n");
	return intel_mid_i2s_resume(device_ptr);
}

#endif
/*
 * INTERFACE FUNCTIONS
 */

/**
 * intel_mid_i2s_set_modem_probe_cb - Setup modem probe callback
 *
 * Setup the callback in case of modem I2S detection, use for several
 * CMT speech on a platform without collision
 *
 * Output parameters
 *      none
 */
void intel_mid_i2s_set_modem_probe_cb(void(*probe_cb)(void))
{
	intel_mid_i2s_modem_probe_cb = probe_cb;
	/* no lock/mutex as limited to one modem on platform */
	if (test_bit(MODEM_FND, &modem_found_and_i2s_setup_ok))
		(*intel_mid_i2s_modem_probe_cb)();
}
EXPORT_SYMBOL_GPL(intel_mid_i2s_set_modem_probe_cb);

/**
 * intel_mid_i2s_set_modem_remove_cb - Remove modem probe callback
 *
 * Remove the callback in case of modem I2S detection, use for several
 * CMT speech on a platform without collision
 *
 * Output parameters
 *      none
 */
void intel_mid_i2s_set_modem_remove_cb(void(*remove_cb)(void))
{
	intel_mid_i2s_modem_remove_cb = remove_cb;
}
EXPORT_SYMBOL_GPL(intel_mid_i2s_set_modem_remove_cb);

/**
 * intel_mid_i2s_flush - This is the I2S flush request
 * @drv_data : pointer on private i2s driver data (by i2s_open function)
 *
 * It will flush the TX FIFO
 * WARNING: this function is used in a Burst Mode context where it is called
 * between Bursts i.e. when there is no FMSYNC, no transfer ongoing at
 * that time
 * If you need a flush while SSP configured in I2S is BUSY and FMSYNC are
 * generated, you have to write another function
 * (loop on BUSY bit and do not limit the flush to at most 16 samples)
 *
 * Output parameters
 *      int : number of samples flushed
 */
int intel_mid_i2s_flush(struct intel_mid_i2s_hdl *drv_data)
{
	u32 sssr, data;
	u32 num = 0;
	u8  rsre;
	void __iomem *reg;

	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return 0;
	reg = drv_data->ioaddr;
	sssr = read_SSSR(reg);
	dev_dbg(drv_data->ssp_dev, "in flush sssr=0x%08X\n", sssr);

	rsre = read_SSCR1(reg) & (SSCR1_RSRE_MASK << SSCR1_RSRE_SHIFT);
	if (rsre) {
		/*
		 * Flush "by hand" was generating spurious DMA SERV REQUEST
		 * from SSP to DMA => then buggy retrieval
		 * of data for next dma_req
		 * Disable: RX Service Request from RX fifo to DMA
		 * as we will flush by hand
		 */
		clear_SSCR1_reg(reg, RSRE);
	}

	/* i2s_flush is called in between 2 bursts
	 * => no FMSYNC at that time (i.e. SSP not busy)
	 * => at most 16 samples in the FIFO */
	while ((read_SSSR(reg) & (SSSR_RNE_MASK<<SSSR_RNE_SHIFT))
			&& (num < FIFO_SIZE)) {
		data = read_SSDR(reg);
		num++;
	}

	if (rsre) {
		/* Enable: RX Service Request from RX fifo to DMA
		 * as flush by hand is done
		 */
		set_SSCR1_reg(reg, RSRE);
	}

	sssr = read_SSSR(reg);
	dev_dbg(drv_data->ssp_dev, "out flush sssr=0x%08X\n", sssr);
	return num;
}
EXPORT_SYMBOL_GPL(intel_mid_i2s_flush);

/**
 * intel_mid_i2s_get_rx_fifo_level - returns I2S rx fifo level
 * @drv_data : pointer on private i2s driver data (by i2s_open function)
 *
 * Output parameters
 *      int : Number of samples currently in the RX FIFO (negative = error)
 */
int intel_mid_i2s_get_rx_fifo_level(struct intel_mid_i2s_hdl *drv_data)
{
	u32 sssr;
	u32 rne, rfl;
	void __iomem *reg;

	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return -EFAULT;
	reg = drv_data->ioaddr;
	sssr = read_SSSR(reg);
	rfl = GET_SSSR_val(sssr, RFL);
	rne = GET_SSSR_val(sssr, RNE);
	if (!rne)
		return 0;
	else
		return rfl+1;
}
EXPORT_SYMBOL_GPL(intel_mid_i2s_get_rx_fifo_level);

/**
 * intel_mid_i2s_get_tx_fifo_level - returns I2S tx fifo level
 * @drv_data : pointer on private i2s driver data (by i2s_open function)
 *
 * Output parameters
 *      int : number of samples currently in the TX FIFO (negative = error)
 */
int intel_mid_i2s_get_tx_fifo_level(struct intel_mid_i2s_hdl *drv_data)
{
	u32 sssr;
	u32 tnf, tfl;
	void __iomem *reg;
	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return -EFAULT;
	reg = drv_data->ioaddr;
	sssr = read_SSSR(reg);
	tfl = GET_SSSR_val(sssr, TFL);
	tnf = GET_SSSR_val(sssr, TNF);
	if (!tnf)
		return 16;
	else
		return tfl;
}
EXPORT_SYMBOL_GPL(intel_mid_i2s_get_tx_fifo_level);

/**
 * intel_mid_i2s_set_rd_cb - set the callback function after read is done
 * @drv_data : handle of corresponding ssp i2s (given by i2s_open function)
 * @read_callback : pointer of callback function
 *
 * Output parameters
 *      error : 0 means no error
 */
int intel_mid_i2s_set_rd_cb(struct intel_mid_i2s_hdl *drv_data,
				int (*read_callback)(void *param))
{
	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return -EFAULT;
	mutex_lock(&drv_data->mutex);
	if (!test_bit(I2S_PORT_OPENED, &drv_data->flags)) {
		dev_warn(drv_data->ssp_dev, "set WR CB I2S_PORT NOT_OPENED");
		mutex_unlock(&drv_data->mutex);
		return -EPERM;
	}
	/* Do not change read parameters in the middle of a READ request */
	if (test_bit(I2S_PORT_READ_BUSY, &drv_data->flags)) {
		dev_warn(drv_data->ssp_dev, "CB reject I2S_PORT_READ_BUSY");
		mutex_unlock(&drv_data->mutex);
		return -EBUSY;
	}
	drv_data->read_callback = read_callback;
	drv_data->read_len = 0;
	mutex_unlock(&drv_data->mutex);
	return 0;

}
EXPORT_SYMBOL_GPL(intel_mid_i2s_set_rd_cb);

/**
 * intel_mid_i2s_set_wr_cb - set the callback function after write is done
 * @drv_data : handle of corresponding ssp i2s (given by i2s_open  function)
 * @write_callback : pointer of callback function
 *
 * Output parameters
 *      error : 0 means no error
 */
int intel_mid_i2s_set_wr_cb(struct intel_mid_i2s_hdl *drv_data,
				int (*write_callback)(void *param))
{
	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return -EFAULT;
	mutex_lock(&drv_data->mutex);
	if (!test_bit(I2S_PORT_OPENED, &drv_data->flags)) {
		dev_warn(drv_data->ssp_dev, "set WR CB I2S_PORT NOT_OPENED");
		mutex_unlock(&drv_data->mutex);
		return -EPERM;
	}
	/* Do not change write parameters in the middle of a WRITE request */
	if (test_bit(I2S_PORT_WRITE_BUSY, &drv_data->flags)) {
		dev_warn(drv_data->ssp_dev, "CB reject I2S_PORT_WRITE_BUSY");
		mutex_unlock(&drv_data->mutex);
		return -EBUSY;
	}
	drv_data->write_callback = write_callback;
	drv_data->write_len = 0;
	mutex_unlock(&drv_data->mutex);
	return 0;
}
EXPORT_SYMBOL_GPL(intel_mid_i2s_set_wr_cb);

/**
 * intel_mid_i2s_lli_rd_req - request a LLI read from i2s peripheral
 * @drv_data : handle of corresponding ssp i2s (given by i2s_open function)
 * @lli_array:
 * @lli_length:
 * @lli_mode:
 * @param:
 */
int intel_mid_i2s_lli_rd_req(struct intel_mid_i2s_hdl *drv_data,
			struct intel_mid_i2s_lli *lli_array, int lli_length,
			enum i2s_lli_mode lli_mode, void *param)
{
	struct dma_async_tx_descriptor *rxdesc = NULL;
	struct scatterlist *temp_sg = NULL;
	struct dma_chan *rxchan = NULL;
	enum dma_ctrl_flags flag;
	int i;

	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return -EFAULT;

	rxchan = drv_data->rxchan;

	if (!rxchan) {
		dev_warn(drv_data->ssp_dev, "rd_req FAILED no rxchan\n");
		return -EINVAL;
	}
	if (lli_length <= 0) {
		dev_warn(drv_data->ssp_dev, "wr_req length less than 1\n");
		return -EINVAL;
	}
	dev_dbg(drv_data->ssp_dev, "FCT Enter = %s\n", __func__);

	/*
	 * Prepare the scatterlist struct array
	 */
	dev_dbg(drv_data->ssp_dev, "kzalloc of scatterlist rd\n");
	temp_sg = kzalloc(sizeof(struct scatterlist)*lli_length, GFP_KERNEL);
	if (!temp_sg) {
		dev_warn(drv_data->ssp_dev,
				 "temp_sg alloc of size %d bytes failed",
				 sizeof(struct scatterlist)*lli_length);
		return -ENOMEM;
	}
	dev_dbg(drv_data->ssp_dev, "kzalloc rd done\n");

	/* initialise scatterlist array */
	sg_init_table(temp_sg, lli_length);

	/*
	 * Fill the Link list with Destination addresses
	 * mutex_lock(&drv_data->mutex); between sg_set_buf not required ?
	 */
	drv_data->rxsgl = temp_sg;
	for (i = 0; i < lli_length; i++)
		sg_set_buf(temp_sg++, lli_array[i].addr, lli_array[i].leng);

	flag = DMA_PREP_INTERRUPT | DMA_CTRL_ACK;
	if (lli_mode == I2S_CIRCULAR_MODE)
		flag |= DMA_PREP_CIRCULAR_LIST;

	dev_dbg(drv_data->ssp_dev, "Calling prep_slave_sg\n");

	/* Enable the SSP Read Request */
	set_SSCR1_reg((drv_data->ioaddr), RSRE);
	change_SSCR0_reg((drv_data->ioaddr), RIM,
			 ((drv_data->current_settings).rx_fifo_interrupt));

	rxdesc = rxchan->device->device_prep_slave_sg(rxchan, drv_data->rxsgl,
			lli_length, DMA_DEV_TO_MEM, flag, NULL);

	if (!rxdesc) {
		dev_warn(drv_data->ssp_dev,
			"rd_req device_prep_slave_sg FAILED\n");
		return -EFAULT;
	}

	/*
	 * Only 1 LLI READ is allowed
	 */
	if (test_and_set_bit(I2S_PORT_READ_BUSY, &drv_data->flags)) {
		dev_warn(drv_data->ssp_dev, "RD reject I2S_PORT READ_BUSY");
		return -EBUSY;
	}
	dev_dbg(drv_data->ssp_dev, "RD dma tx submit\n");
	rxdesc->callback = i2s_lli_read_done;
	drv_data->read_param = param;
	rxdesc->callback_param = drv_data;
	rxdesc->tx_submit(rxdesc);
	return 0;

}
EXPORT_SYMBOL_GPL(intel_mid_i2s_lli_rd_req);

/**
 * intel_mid_i2s_lli_wr_req - request a LLI wrie from i2s peripheral
 * @drv_data : handle of corresponding ssp i2s (given by i2s_open function)
 * @lli_array:
 * @lli_length:
 * @lli_mode:
 * @param:
 */
int intel_mid_i2s_lli_wr_req(struct intel_mid_i2s_hdl *drv_data,
		struct intel_mid_i2s_lli *lli_array, int lli_length,
		enum i2s_lli_mode lli_mode, void *param)
{
	struct dma_async_tx_descriptor *txdesc = NULL;
	struct scatterlist *temp_sg = NULL;
	struct dma_chan *txchan = NULL;
	enum dma_ctrl_flags flag;
	int i;

	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return -EFAULT;

	txchan = drv_data->txchan;

	if (!txchan) {
		dev_warn(drv_data->ssp_dev, "wr_req but no txchan\n");
		return -EINVAL;
	}
	if (lli_length <= 0) {
		dev_warn(drv_data->ssp_dev, "wr_req length less than 1\n");
		return -EINVAL;
	}

	/*
	 * Determine the list length
	 */
	dev_dbg(drv_data->ssp_dev, "kzalloc of scatterlist wr\n");
	temp_sg = kzalloc(sizeof(struct scatterlist)*lli_length, GFP_KERNEL);
	if (!temp_sg) {
		dev_warn(drv_data->ssp_dev,
				 "temp_sg alloc of size %d bytes failed",
				 sizeof(struct scatterlist)*lli_length);
		return -ENOMEM;
	}
	dev_dbg(drv_data->ssp_dev, "kzalloc wr done\n");

	/* initialise scatterlist array */
	sg_init_table(temp_sg, lli_length);

	/*
	 * Fill the Link list with Source addresses
	 * mutex_lock(&drv_data->mutex); not required for sg_set_buf?
	 */
	drv_data->txsgl = temp_sg;
	for (i = 0; i < lli_length; i++)
		sg_set_buf(temp_sg++, lli_array[i].addr, lli_array[i].leng);

	flag = DMA_PREP_INTERRUPT | DMA_CTRL_ACK;
	if (lli_mode == I2S_CIRCULAR_MODE)
		flag |= DMA_PREP_CIRCULAR_LIST;
	/* Enable the SSP Write Request */
	set_SSCR1_reg((drv_data->ioaddr), TSRE);
	change_SSCR0_reg((drv_data->ioaddr), TIM,
			 ((drv_data->current_settings).tx_fifo_interrupt));

	dev_dbg(drv_data->ssp_dev, "prep slave sg wr\n");
	txdesc =  txchan->device->device_prep_slave_sg(txchan, drv_data->txsgl,
				lli_length, DMA_MEM_TO_DEV, flag, NULL);
	dev_dbg(drv_data->ssp_dev, "prep slave sg wr done\n");
	if (!txdesc) {
		dev_warn(drv_data->ssp_dev,
			"wr_req device_prep_slave_sg FAILED\n");
		return -1;
	}

	/*
	 * Only 1 LLI WRITE is allowed
	 */
	if (test_and_set_bit(I2S_PORT_WRITE_BUSY, &drv_data->flags)) {
		dev_warn(drv_data->ssp_dev, "WR reject I2S_PORT WRITE_BUSY");
		return -EBUSY;
	}
	dev_dbg(drv_data->ssp_dev, "WR dma tx summit\n");
	txdesc->callback = i2s_lli_write_done;
	drv_data->write_param = param;
	txdesc->callback_param = drv_data;
	txdesc->tx_submit(txdesc);
	dev_dbg(drv_data->ssp_dev, "wr dma req programmed\n");
	return 0;
}
EXPORT_SYMBOL_GPL(intel_mid_i2s_lli_wr_req);

/**
 * intel_mid_i2s_rd_req - request a read from i2s peripheral
 * @drv_data : handle of corresponding ssp i2s (given by i2s_open function)
 * @dst : destination buffer where the read sample should be put
 * @len : number of sample to be read (160 samples only right now)
 * @param : private context parameter to give back to read callback
 *
 * Output parameters
 *      error : 0 means no error
 */
int intel_mid_i2s_rd_req(struct intel_mid_i2s_hdl	*drv_data,
			 u32				*destination,
			 size_t				len,
			 void				*param)
{
	int result;

	/* Checks */
	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return -EFAULT;
	if (!len) {
		dev_warn(drv_data->ssp_dev, "rd req invalid len=0");
		return -EINVAL;
	}

	/* Allow only 1 READ at a time. */
	if (test_and_set_bit(I2S_PORT_READ_BUSY, &drv_data->flags)) {
		dev_warn(drv_data->ssp_dev, "RD reject I2S_PORT READ_BUSY");
		return -EBUSY;
	}

	dev_dbg(drv_data->ssp_dev, "%s() dst=%p, len=%d, drv_data=%p",
		__func__, destination, len, drv_data);

	if (drv_data->current_settings.ssp_rx_dma == SSP_RX_DMA_ENABLE)
		result = rd_req_dma(drv_data, destination, len, param);
	else
		result = rd_req_cpu(drv_data, destination, len, param);

	/* Error management */
	if (result)
		goto return_clr_busy;

	return result;

	/* Error management: free resources */
return_clr_busy:
	clear_bit(I2S_PORT_READ_BUSY, &drv_data->flags);
	return result;
}
EXPORT_SYMBOL_GPL(intel_mid_i2s_rd_req);

static int rd_req_cpu(struct intel_mid_i2s_hdl *drv_data,
		      u32			*destination,
		      size_t			len,
		      void			*param)
{
	dev_dbg(drv_data->ssp_dev, "%s() - ENTER", __func__);

	/* Initiate read */
	drv_data->mask_sr |= ((SSSR_RFS_MASK << SSSR_RFS_SHIFT) |
			      (SSSR_ROR_MASK << SSSR_ROR_SHIFT));

	clear_bit(I2S_PORT_COMPLETE_READ, &drv_data->flags);
	set_SSCR1_reg((drv_data->ioaddr), RIE);
	change_SSCR0_reg((drv_data->ioaddr), RIM,
			 ((drv_data->current_settings).rx_fifo_interrupt));

	/* Save param */
	drv_data->read_ptr.cpu	= (u16 *)destination;
	drv_data->read_len	= len;
	drv_data->read_param	= param;

	return 0;
}

static int rd_req_dma(struct intel_mid_i2s_hdl *drv_data,
		      u32			*destination,
		      size_t			len,
		      void			*param)
{
	/* Locals declaration */
	int				result = 0;
	struct dma_async_tx_descriptor	*rxdesc = NULL;
	struct dma_chan			*rxchan = drv_data->rxchan;
	enum dma_ctrl_flags		flag;
	dma_addr_t			ssdr_addr;
	dma_addr_t			dst;

	dev_dbg(drv_data->ssp_dev, "%s() - ENTER", __func__);

	/* Checks */
	WARN(!drv_data->dmacdev, "DMA device=NULL\n");
	if (!drv_data->dmacdev)
		return -EFAULT;

	if (!rxchan) {
		dev_warn(drv_data->ssp_dev, "rd_req FAILED no rxchan\n");
		return -EINVAL;
	}

	/* Map dma address */
	dst = dma_map_single(NULL, destination, len, DMA_FROM_DEVICE);
	if (!dst) {
		dev_warn(drv_data->ssp_dev, "can't map DMA address %p",
			 destination);
		return -ENOMEM;
	}

	/* Prepare RX dma transfer */
	ssdr_addr = (drv_data->paddr + OFFSET_SSDR);
	flag = DMA_PREP_INTERRUPT | DMA_CTRL_ACK;

	rxdesc = rxchan->device->device_prep_dma_memcpy(
					rxchan,		/* DMA Channel */
					dst,		/* DAR */
					ssdr_addr,	/* SAR */
					len,		/* Data Length */
					flag);		/* Flag */
	if (!rxdesc) {
		dev_warn(drv_data->ssp_dev, "can not prep dma memcpy");
		result = -EFAULT;
		goto return_unmap;
	}

	set_SSCR1_reg((drv_data->ioaddr), RSRE);
	change_SSCR0_reg((drv_data->ioaddr), RIM,
			 ((drv_data->current_settings).rx_fifo_interrupt));

	/* Save param */
	drv_data->read_ptr.dma	= dst;
	drv_data->read_len	= len;
	drv_data->read_param	= param;

	rxdesc->callback	= i2s_read_done;
	rxdesc->callback_param	= drv_data;

	/* Submit DMA transfer */
	rxdesc->tx_submit(rxdesc);
	return result;

	/* Error management: free resources */
return_unmap:
	dma_unmap_single(NULL, dst, len, DMA_FROM_DEVICE);
	return result;
}

/**
 * intel_mid_i2s_wr_req - request a write to i2s peripheral
 * @drv_data : handle of corresponding ssp i2s (given by i2s_open function)
 * @src : source buffer where the samples to wrote should be get
 * @len : number of sample to be read (160 samples only right now)
 * @param : private context parameter to give back to write callback
 *
 * Output parameters
 *      error : 0 means no error
 */
int intel_mid_i2s_wr_req(struct intel_mid_i2s_hdl	*drv_data,
			 u32				*source,
			 size_t				len,
			 void				*param)
{
	int result;

	/* Checks */
	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return -EFAULT;
	if (!len) {
		dev_warn(drv_data->ssp_dev, "invalid wr len 0");
		return -EINVAL;
	}

	/* Allow only 1 WRITE at a time */
	if (test_and_set_bit(I2S_PORT_WRITE_BUSY, &drv_data->flags)) {
		dev_warn(drv_data->ssp_dev, "WR reject I2S_PORT WRITE_BUSY");
		return -EBUSY;
	}

	dev_dbg(drv_data->ssp_dev, "%s() src=%p, len=%d, drv_data=%p",
		__func__, source, len, drv_data);

	if (drv_data->current_settings.ssp_tx_dma == SSP_TX_DMA_ENABLE)
		result = wr_req_dma(drv_data, source, len, param);
	else
		result = wr_req_cpu(drv_data, source, len, param);

	/* Error management */
	if (result)
		goto return_clr_busy;

	return result;

	/* Error management: free resources */
return_clr_busy:
	clear_bit(I2S_PORT_WRITE_BUSY, &drv_data->flags);
	return result;
}
EXPORT_SYMBOL_GPL(intel_mid_i2s_wr_req);

static int wr_req_cpu(struct intel_mid_i2s_hdl *drv_data,
		      u32			*source,
		      size_t			len,
		      void			*param)
{
	dev_dbg(drv_data->ssp_dev, "%s() - ENTER (src=0x%08x, len=%d"
		, __func__, (u32)source, len);

	/* Initiate write */
	drv_data->mask_sr |= ((SSSR_TFS_MASK << SSSR_TFS_SHIFT) |
			      (SSSR_TUR_MASK << SSSR_TUR_SHIFT));

	clear_bit(I2S_PORT_COMPLETE_WRITE, &drv_data->flags);

	set_SSCR1_reg((drv_data->ioaddr), TIE);
	change_SSCR0_reg((drv_data->ioaddr), TIM,
			 ((drv_data->current_settings).tx_fifo_interrupt));

	/* Save param */
	drv_data->write_ptr.cpu	= (u16 *)source;
	drv_data->write_len	= len;
	drv_data->write_param	= param;

	return 0;
}

static int wr_req_dma(struct intel_mid_i2s_hdl *drv_data,
		      u32			*source,
		      size_t			len,
		      void			*param)
{
	int				result = 0;
	struct dma_async_tx_descriptor	*txdesc = NULL;
	struct dma_chan			*txchan = drv_data->txchan;
	enum dma_ctrl_flags		flag;
	dma_addr_t			ssdr_addr;
	dma_addr_t			src;

	dev_dbg(drv_data->ssp_dev, "%s() - ENTER", __func__);

	WARN(!drv_data->dmacdev, "DMA device=NULL\n");
	if (!drv_data->dmacdev)
		return -EFAULT;

	if (!txchan) {
		dev_warn(drv_data->ssp_dev, "wr_req but no txchan\n");
		return -EINVAL;
	}

	/* Map DMA address */
	src = dma_map_single(NULL, source, len, DMA_TO_DEVICE);
	if (!src) {
		dev_warn(drv_data->ssp_dev, "can't map DMA address %p",
						source);
		return -ENOMEM;
	}

	/* Prepare TX dma transfer */
	ssdr_addr = (dma_addr_t)(u32)(drv_data->paddr + OFFSET_SSDR);
	flag = DMA_PREP_INTERRUPT | DMA_CTRL_ACK;

	txdesc = txchan->device->device_prep_dma_memcpy(
					txchan,		/* DMA Channel */
					ssdr_addr,	/* DAR */
					src,		/* SAR */
					len,		/* Data Length */
					flag);		/* Flag */
	if (!txdesc) {
		dev_warn(drv_data->ssp_dev,
			"wr_req dma memcpy FAILED(src=%08x,len=%d,txchan=%p)\n",
			(unsigned int)src, len, txchan);
		result = -EFAULT;
		goto return_unmap;
	}

	set_SSCR1_reg((drv_data->ioaddr), TSRE);
	change_SSCR0_reg((drv_data->ioaddr), TIM,
			 ((drv_data->current_settings).tx_fifo_interrupt));

	/* Save params */
	drv_data->write_ptr.dma	= src;
	drv_data->write_len	= len;
	drv_data->write_param	= param;

	txdesc->callback	= i2s_write_done;
	txdesc->callback_param	= drv_data;

	/* Submit DMA transfer */
	txdesc->tx_submit(txdesc);
	return result;

	/* Error management: free resources */
return_unmap:
	dma_unmap_single(NULL, src, len, DMA_TO_DEVICE);
	return result;
}

/**
 * intel_mid_i2s_open_pci - reserve and start a SSP depending of it's usage
 * @usage : select which ssp i2s you need by giving usage (BT,MODEM...)
 * @ps_settings : hardware settings to configure the SSP module
 *
 * May sleep (driver_find_device) : no lock permitted when called.
 *
 * Output parameters
 *      handle : handle of the selected SSP, or NULL if not found
 */
struct intel_mid_i2s_hdl *intel_mid_i2s_open_pci(
					enum intel_mid_i2s_ssp_usage usage)
{
	struct pci_dev *pdev;
	struct intel_mid_i2s_hdl *drv_data = NULL;
	struct device *found_device = NULL;
	pr_debug("%s : open called,searching for device with usage=%x !\n",
			DRIVER_NAME, usage);
	found_device = driver_find_device(&(intel_mid_i2s_driver.driver), NULL,
						&usage, check_device_pci);
	if (!found_device) {
		pr_debug("%s : open can not found with usage=0x%02X\n",
				DRIVER_NAME, (int)usage);
		return NULL;
	}
	pdev = to_pci_dev(found_device);
	drv_data = pci_get_drvdata(pdev);

	if (!drv_data) {
		dev_err(found_device, "no drv_data in open pdev=%p\n", pdev);
		put_device(found_device);
		return NULL;
	}
	mutex_lock(&drv_data->mutex);

	/* pm_runtime */
	pm_runtime_get_sync(drv_data->ssp_dev);

	if (test_bit(I2S_PORT_CLOSING, &drv_data->flags)) {
		dev_err(drv_data->ssp_dev, "Opening a closing I2S!");
		goto open_error;
	}
	/* Indicate that the HW param config is not set yet */
	drv_data->current_settings.mode = SSP_INVALID_MODE;

	/* there is no need to "wake up" as we can not close an opening i2s */
	clear_bit(I2S_PORT_WRITE_BUSY, &drv_data->flags);
	clear_bit(I2S_PORT_READ_BUSY, &drv_data->flags);
	mutex_unlock(&drv_data->mutex);
	return drv_data;

open_error:
	put_device(found_device);
	pm_runtime_put_sync(drv_data->ssp_dev);
	mutex_unlock(&drv_data->mutex);
	return NULL;
}

/**
 * intel_mid_i2s_open_acpi - reserve and start a SSP depending of it's usage
 * @usage : select which ssp i2s you need by giving usage (BT,MODEM...)
 * @ps_settings : hardware settings to configure the SSP module
 *
 * May sleep (driver_find_device) : no lock permitted when called.
 *
 * Output parameters
 *      handle : handle of the selected SSP, or NULL if not found
 */
struct intel_mid_i2s_hdl *intel_mid_i2s_open_acpi(
				enum intel_mid_i2s_ssp_usage usage)
{
	struct intel_mid_i2s_hdl *drv_data = NULL;
#ifndef CONFIG_ACPI
	WARN(true, "== opening ACPI without ACPI in kernel ! ==");
#else
	struct device *found_device = NULL;

	pr_info("%s : I2S open called,searching for device with usage=%x !\n",
			DRIVER_NAME, usage);

	found_device = driver_find_device(&(i2s_acpi_driver.driver), NULL,
						&usage, check_device_acpi);
	if (!found_device) {
		pr_info("%s : i2s_open can not found with usage=0x%02X\n",
				DRIVER_NAME, (int)usage);
		return NULL;
	}

	drv_data = (struct intel_mid_i2s_hdl *) dev_get_drvdata(found_device);

	pm_runtime_get_sync(drv_data->ssp_dev);

#endif /* CONFIG_ACPI */
	return drv_data;

}

/**
 * intel_mid_i2s_open - reserve and start a SSP depending of it's usage
 * @usage : select which ssp i2s you need by giving usage (BT,MODEM...)
 * @ps_settings : hardware settings to configure the SSP module
 *
 * May sleep (driver_find_device) : no lock permitted when called.
 *
 * Output parameters
 *      handle : handle of the selected SSP, or NULL if not found
 */
struct intel_mid_i2s_hdl *intel_mid_i2s_open(enum intel_mid_i2s_ssp_usage usage)
{
	pr_info("INFO: I2S DRIVER opening usage=%d FEAT_ACPI=%d FEAT_PCI=%d\n",
		usage,
		test_bit(FEAT_ACPI, &ssp_ip_features),
		test_bit(FEAT_PCI, &ssp_ip_features));

	if (test_bit(FEAT_ACPI, &ssp_ip_features)) {
		return intel_mid_i2s_open_acpi(usage);
	} else if (test_bit(FEAT_PCI, &ssp_ip_features)) {
		return intel_mid_i2s_open_pci(usage);
	} else {
		WARN(true, "== I2S open without probe! ==");
		return 0;
	}

}
EXPORT_SYMBOL_GPL(intel_mid_i2s_open);

/**
 * intel_mid_i2s_close - release and stop the SSP
 * @drv_data : handle of corresponding ssp i2s (given by i2s_open function)
 *
 * WARNING: This is not -yet- allowed to call close from a read/write callback !
 *
 * Output parameters
 *      none
 */
void intel_mid_i2s_close(struct intel_mid_i2s_hdl *drv_data)
{
	void __iomem *reg;

	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return;
	mutex_lock(&drv_data->mutex);
	if (!test_bit(I2S_PORT_OPENED, &drv_data->flags)) {
		dev_err(drv_data->ssp_dev, "not opened but closing?");
		mutex_unlock(&drv_data->mutex);
		return;
	}

	set_bit(I2S_PORT_CLOSING, &drv_data->flags);
	dev_dbg(drv_data->ssp_dev, "Status bit pending write=%d read=%d\n",
			test_bit(I2S_PORT_WRITE_BUSY, &drv_data->flags),
			test_bit(I2S_PORT_READ_BUSY, &drv_data->flags));
	if (test_bit(I2S_PORT_WRITE_BUSY, &drv_data->flags) ||
	     test_bit(I2S_PORT_READ_BUSY, &drv_data->flags)) {
		dev_dbg(drv_data->ssp_dev,
				"Pending callback in close...\n");
	}

	reg = drv_data->ioaddr;
	dev_dbg(drv_data->ssp_dev, "Stopping the SSP\n");

#ifdef _LLI_ENABLED_
	i2s_enable(drv_data, false);
#else
	i2s_disable(drv_data);
#endif /* _LLI_ENABLED_ */

	put_device(drv_data->ssp_dev);
	write_SSCR0(0, reg);
	/*
	 * Set the SSP in SLAVE Mode and Enable TX tristate
	 * to ensure that when leaving D0i3 the SSP does
	 * not drive the FS and TX pins.
	 *
	 */
	write_SSCR1((SSCR1_SFRMDIR_MASK << SSCR1_SFRMDIR_SHIFT)
			| (SSCR1_SCLKDIR_MASK << SSCR1_SCLKDIR_SHIFT)
			| (SSCR1_TTE_MASK << SSCR1_TTE_SHIFT),
			reg);

	dev_dbg(drv_data->ssp_dev, "SSP Stopped.\n");
	clear_bit(I2S_PORT_CLOSING, &drv_data->flags);
	clear_bit(I2S_PORT_OPENED, &drv_data->flags);

	/* pm runtime */
	pm_runtime_put_sync(drv_data->ssp_dev);

	mutex_unlock(&drv_data->mutex);
}
EXPORT_SYMBOL_GPL(intel_mid_i2s_close);

/*
 * INTERNAL FUNCTIONS
 */

#ifdef _LLI_ENABLED_
/**
 * i2s_enable -  Enable/disable SSP device
 * @drv_data : pointer to driver data
 * @enable : boolean to either enable or disable device
 * Writes SSP register to enable the device
 */
static inline void i2s_enable(struct intel_mid_i2s_hdl *drv_data, bool enable)
{
	if (enable)
		set_SSCR0_reg(drv_data->ioaddr, SSE)
	else
		clear_SSCR0_reg(drv_data->ioaddr, SSE)
}

/**
 * i2s_enable_dma_rx_intr -  Enables/disables the Receive FIFO DMA Service Request
 * @drv_data : pointer to driver data
 * @enable : boolean to either enable or disable service request
 * Writes SSP register to enable Service Request
 */
static inline void i2s_enable_dma_rx_intr(struct intel_mid_i2s_hdl *drv_data,
							bool enable)
{
	if (enable) {
		set_SSCR1_reg(drv_data->ioaddr, RSRE)
		change_SSCR0_reg((drv_data->ioaddr), RIM,
			((drv_data->current_settings).rx_fifo_interrupt));
	} else {
		change_SSCR0_reg(drv_data->ioaddr, RIM,
						 SSP_RX_FIFO_OVER_INT_DISABLE);
		clear_SSCR1_reg(drv_data->ioaddr, RSRE)
	}
}

/**
 * i2s_enable_dma_tx_intr -  Enables/disables the Transmit FIFO DMA Service Request
 * @drv_data : pointer to driver data
 * @enable : boolean to either enable or disable service request
 * Writes SSP register to enable Service Request
 */
static inline void i2s_enable_dma_tx_intr(struct intel_mid_i2s_hdl *drv_data,
							bool enable)
{
	if (enable) {
		set_SSCR1_reg(drv_data->ioaddr, TSRE)
		change_SSCR0_reg((drv_data->ioaddr), TIM,
			((drv_data->current_settings).tx_fifo_interrupt));
	} else {
		change_SSCR0_reg(drv_data->ioaddr, TIM,
						 SSP_TX_FIFO_UNDER_INT_DISABLE);
		clear_SSCR1_reg(drv_data->ioaddr, TSRE)
	}
}
#else
/**
 * i2s_enable - enable SSP device
 * @drv_data : pointer to driver data
 *
 * Writes SSP register to enable the device
 */
static inline void i2s_enable(struct intel_mid_i2s_hdl *drv_data)
{
	set_SSCR0_reg(drv_data->ioaddr, SSE);
}

/**
 * i2s_disable - disable SSP device
 * @drv_data : pointer to driver data
 *
 * Writes SSP register to disable the device
 */
static inline void i2s_disable(struct intel_mid_i2s_hdl *drv_data)
{
	clear_SSCR0_reg(drv_data->ioaddr, SSE);
}
#endif /* _LLI_ENABLED_ */

/**
 * check_device_pci -  return if the device is the usage we want (usage =*data)
 * @device_ptr : pointer on device struct
 * @data : pointer pointer on usage we are looking for
 *
 * this is called for each device by find_device() from intel_mid_i2s_open()
 * Info : when found, the flag of driver is set to I2S_PORT_OPENED
 *
 * Output parameters
 *      integer : return 0 means not the device or already started. go next
 *		  return != 0 means stop the search and return this device
 */
static int
check_device_pci(struct device *device_ptr, void *data)
{
	struct pci_dev *pdev;
	struct intel_mid_i2s_hdl *drv_data;
	enum intel_mid_i2s_ssp_usage usage;
	enum intel_mid_i2s_ssp_usage usage_to_find;

	pdev = to_pci_dev(device_ptr);
	WARN(!pdev, "Pci device=NULL\n");
	if (!pdev)
		return 0;
	drv_data = (struct intel_mid_i2s_hdl *) pci_get_drvdata(pdev);
	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return 0;
	dev_dbg(&(pdev->dev), "Check device pci_dev ptr = 0X%p\n", pdev);
	usage_to_find = *((enum intel_mid_i2s_ssp_usage *) data);
	usage = drv_data->usage;

	/* Can be done in one "if" but separated in purpose : take care of
	 * test_and_set_bit that need to be done AFTER the check on usage.
	 */
	if (usage == usage_to_find) {
		if (!test_and_set_bit(I2S_PORT_OPENED, &drv_data->flags))
			return 1;  /* Already opened, do not use this result */
	};
	return 0; /* not usage we look for, or already opened */
}

#ifdef CONFIG_ACPI
/**
 * check_device_acpi -  return if the device is the usage we want (usage =*data)
 * @device_ptr : pointer on device struct
 * @data : pointer pointer on usage we are looking for
 *
 * this is called for each device by find_device() from intel_mid_i2s_open()
 * Info : when found, the flag of driver is set to I2S_PORT_OPENED
 *
 * Output parameters
 *      integer : return 0 means not the device or already started. go next
 *		  return != 0 means stop the search and return this device
 */
static int
check_device_acpi(struct device *device_ptr, void *data)
{
	struct intel_mid_i2s_hdl *drv_data;
	enum intel_mid_i2s_ssp_usage usage;
	enum intel_mid_i2s_ssp_usage usage_to_find;

	drv_data = (struct intel_mid_i2s_hdl *) dev_get_drvdata(device_ptr);
	usage = drv_data->usage;
	usage_to_find = *((enum intel_mid_i2s_ssp_usage *) data);
	pr_debug("i2sChecking the device device_ptr=%p drv_data=%p (used=%d) w/ usage = %d searching %d\n",
	 device_ptr, drv_data,
	 test_bit(I2S_PORT_OPENED, &drv_data->flags), usage, usage_to_find);

	if (usage == usage_to_find) {
		if (!test_and_set_bit(I2S_PORT_OPENED, &drv_data->flags))
			return 1;  /* Already opened, do not use this result */
	}

	return 0;
}
#endif

/**
 * i2s_reset_command_done - reset driver state if dma callback is not excuted before stream stop
 * @cmd : command to be executed
 * @arg : void pointer to that should be driver data (context)
 *
 * Output parameters
 *      none
 */
static void i2s_reset_command_done(struct intel_mid_i2s_hdl *drv_data,
				enum intel_mid_i2s_ssp_cmd cmd)
{
	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return;

	switch (cmd) {
	case SSP_CMD_FREE_TX:
		drv_data->write_len = 0;
		change_SSCR0_reg(drv_data->ioaddr, TIM,
				SSP_TX_FIFO_UNDER_INT_DISABLE);
		clear_bit(I2S_PORT_WRITE_BUSY, &drv_data->flags);
		break;

	case SSP_CMD_FREE_RX:
		drv_data->read_len = 0;
		change_SSCR0_reg(drv_data->ioaddr, RIM,
				SSP_RX_FIFO_OVER_INT_DISABLE);
		clear_bit(I2S_PORT_READ_BUSY, &drv_data->flags);
		break;

	default:
		dev_warn(drv_data->ssp_dev, "Unknown reset cmd received!");
		break;
	}
	return;
}

/**
 * i2s_read_done - callback from the _dma tasklet_ after read
 * @arg : void pointer to that should be driver data (context)
 *
 * Output parameters
 *      none
 */
static void i2s_read_done(void *arg)
{
	int status = 0;

	struct intel_mid_i2s_hdl *drv_data = arg;
	void *param_complete;
	void __iomem *reg ;

	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return;
	if (!test_bit(I2S_PORT_READ_BUSY, &drv_data->flags))
		dev_dbg(drv_data->ssp_dev, "spurious read dma complete");

	dma_unmap_single(NULL, drv_data->read_ptr.dma,
			 drv_data->read_len, DMA_FROM_DEVICE);
	drv_data->read_len = 0;
	reg = drv_data->ioaddr;
	/* Rx fifo overrun Interrupt */
	change_SSCR0_reg(reg, RIM, SSP_RX_FIFO_OVER_INT_DISABLE);
	param_complete = drv_data->read_param;
	/* Do not change order sequence:
	 * READ_BUSY clear, then test PORT_CLOSING
	 * wakeup for close() function
	 */
	clear_bit(I2S_PORT_READ_BUSY, &drv_data->flags);
	if (test_bit(I2S_PORT_CLOSING, &drv_data->flags))
		return;
	if (drv_data->read_callback != NULL)
		status = drv_data->read_callback(param_complete);
	else
		dev_warn(drv_data->ssp_dev, "RD done but not callback set");

}

/**
 * i2s_lli_read_done - callback from the _dma tasklet_ after read
 * @arg : void pointer to that should be driver data (context)
 *
 * Output parameters
 *      none
 */
static void i2s_lli_read_done(void *arg)
{
	int status = 0;

	struct intel_mid_i2s_hdl *drv_data = arg;
	void *param_complete;
	void __iomem *reg ;

	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return;
	if (!test_bit(I2S_PORT_READ_BUSY, &drv_data->flags))
		dev_dbg(drv_data->ssp_dev, "spurious read dma complete");

	reg = drv_data->ioaddr;

#ifndef _LLI_ENABLED_
	/* Rx fifo overrun Interrupt */
	change_SSCR0_reg(reg, RIM, SSP_RX_FIFO_OVER_INT_DISABLE);
#endif /* _LLI_ENABLED_ */

	param_complete = drv_data->read_param;

#ifndef _LLI_ENABLED_
	/* Do not change order sequence:
	 * READ_BUSY clear, then test PORT_CLOSING
	 * wakeup for close() function
	 */
	clear_bit(I2S_PORT_READ_BUSY, &drv_data->flags);
#endif /* _LLI_ENABLED_ */

	if (test_bit(I2S_PORT_CLOSING, &drv_data->flags))
		return;
	if (drv_data->read_callback != NULL)
		status = drv_data->read_callback(param_complete);
	else
		dev_warn(drv_data->ssp_dev, "RD done but not callback set");
}

/**
 * i2s_write_done() : callback from the _dma tasklet_ after write
 * @arg : void pointer to that should be driver data (context)
 *
 * Output parameters
 *      none
 */
static void i2s_write_done(void *arg)
{
	struct intel_mid_i2s_hdl	*drv_data	= arg;
	int				status		= 0;
	void				*param_complete;
	void __iomem			*reg ;

	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return;
	if (!test_bit(I2S_PORT_WRITE_BUSY, &drv_data->flags))
		dev_dbg(drv_data->ssp_dev, "spurious write dma complete");

	dma_unmap_single(NULL, drv_data->write_ptr.dma,
			 drv_data->write_len, DMA_TO_DEVICE);
	drv_data->write_len	= 0;
	drv_data->write_ptr.cpu	= NULL;

	reg = drv_data->ioaddr;
	change_SSCR0_reg(reg, TIM, SSP_TX_FIFO_UNDER_INT_DISABLE);
	dev_dbg(drv_data->ssp_dev, "DMA channel disable..\n");
	param_complete = drv_data->write_param;

	/* Do not change order sequence:
	 * WRITE_BUSY clear, then test PORT_CLOSING
	 * wakeup for close() function
	 */
	clear_bit(I2S_PORT_WRITE_BUSY, &drv_data->flags);
	if (test_bit(I2S_PORT_CLOSING, &drv_data->flags))
		return;
	if (drv_data->write_callback != NULL)
		status = drv_data->write_callback(param_complete);
	else
		dev_warn(drv_data->ssp_dev, "WR done but no callback set");
}

/**
 * i2s_lli_write_done - callback from the _dma tasklet_ after write
 * @arg : void pointer to that should be driver data (context)
 *
 * Output parameters
 *      none
 */
static void i2s_lli_write_done(void *arg)
{
	int status = 0;
	void *param_complete;
	struct intel_mid_i2s_hdl *drv_data = arg;
	void __iomem *reg ;

	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return;
	if (!test_bit(I2S_PORT_WRITE_BUSY, &drv_data->flags))
		dev_dbg(drv_data->ssp_dev, "spurious write dma complete");

	dev_dbg(drv_data->ssp_dev, "lli wr Done!\n");

	reg = drv_data->ioaddr;

#ifndef _LLI_ENABLED_
	change_SSCR0_reg(reg, TIM, SSP_TX_FIFO_UNDER_INT_DISABLE);
	dev_dbg(drv_data->ssp_dev, "DMA channel disable..\n");
#endif /* _LLI_ENABLED_ */

	param_complete = drv_data->write_param;

#ifndef _LLI_ENABLED_
	/* Do not change order sequence:
	 * WRITE_BUSY clear, then test PORT_CLOSING
	 * wakeup for close() function
	 */
	clear_bit(I2S_PORT_WRITE_BUSY, &drv_data->flags);
#endif /* _LLI_ENABLED_ */

	if (test_bit(I2S_PORT_CLOSING, &drv_data->flags))
		return;
	if (drv_data->write_callback != NULL)
		status = drv_data->write_callback(param_complete);
	else
		dev_warn(drv_data->ssp_dev, "WR done but no callback set");
}

static bool chan_filter(struct dma_chan *chan, void *param)
{
	struct intel_mid_i2s_hdl *drv_data = (struct intel_mid_i2s_hdl *)param;
	return (chan->device->dev == drv_data->dmacdev);
}
static int i2s_compute_dma_width(u16 ssp_data_size,
				 enum dma_slave_buswidth *dma_width)
{
	if (ssp_data_size <= 8)
		*dma_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	else if (ssp_data_size <= 16)
		*dma_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	else if (ssp_data_size <= 32)
		*dma_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	else
		return -EINVAL;

	return 0;
}
static int i2s_compute_dma_msize(u8 ssp_threshold,
				 enum intel_mid_dma_msize *dma_msize)
{

	switch (ssp_threshold) {
	case 1:
		*dma_msize = LNW_DMA_MSIZE_1;
		break;
	case 4:
		*dma_msize = LNW_DMA_MSIZE_4;
		break;
	case 8:
		*dma_msize = LNW_DMA_MSIZE_8;
		break;
	case 16:
		*dma_msize = LNW_DMA_MSIZE_16;
		break;
	case 32:
		*dma_msize = LNW_DMA_MSIZE_32;
		break;
	case 64:
		*dma_msize = LNW_DMA_MSIZE_64;
		break;
	default:
		return -EINVAL;
		break;
	}
	return 0;
}

/**
 * intel_mid_i2s_command() : execute simple commands on ssp driver
 * @drv_data : handle of corresponding ssp i2s (given by i2s_open function)
 * @cmd : command to be executed
 *
 * Execute simple commands on ssp driver
 *
 * Output parameters
 *      error : 0 means no error
 */

int intel_mid_i2s_command(struct intel_mid_i2s_hdl *drv_data,
			enum intel_mid_i2s_ssp_cmd cmd,
			const struct intel_mid_i2s_settings *hw_ssp_settings)
{
	void __iomem *reg;

	struct intel_mid_dma_slave *rxs, *txs;
	struct intel_mid_i2s_settings *ssp_settings = NULL;
	dma_cap_mask_t mask;
	int retval = 0;
	int temp = 0;

	int s;
	struct dma_chan *channel;

	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return -EFAULT;
	reg = drv_data->ioaddr;

	pr_debug("FCT %s CMD = %d\n", __func__, cmd);

	/* actions */
	switch (cmd) {

	case SSP_CMD_ABORT:
		/* Abort write */
		if (test_and_clear_bit(I2S_PORT_WRITE_BUSY, &drv_data->flags)) {
			dev_dbg(drv_data->ssp_dev,
					"%s : abort write", __func__);

			clear_bit(I2S_PORT_COMPLETE_WRITE, &drv_data->flags);
			i2s_finalize_write(drv_data);
		}

		/* Abort read */
		if (test_and_clear_bit(I2S_PORT_READ_BUSY, &drv_data->flags)) {
			dev_dbg(drv_data->ssp_dev,
					"%s: abort read", __func__);

			clear_bit(I2S_PORT_COMPLETE_READ, &drv_data->flags);
			i2s_finalize_read(drv_data);
		}

#ifdef _LLI_ENABLED_
		i2s_enable(drv_data, false);
#else
		i2s_disable(drv_data);
#endif /* _LLI_ENABLED_ */
		break;

	case SSP_CMD_SET_HW_CONFIG:
		set_ssp_i2s_hw(drv_data, hw_ssp_settings);
		break;

	case SSP_CMD_ENABLE_SSP:
		if (test_bit(FEAT_DIV_CTRL, &ssp_ip_features)) {

			if (drv_data->device_instance == SSP1_INSTANCE) {
				/* Set internal frequency to
				 * 19.2Mhz vs default 25Mhz */
				write_LPE_SSP1_DIV_CTRL_L(
				SSP19200MHZ_DIV_CTRL_L, lpeshim_base_address);
				write_LPE_SSP1_DIV_CTRL_H(
				SSP19200MHZ_DIV_CTRL_H, lpeshim_base_address);
			} else if (drv_data->device_instance == SSP2_INSTANCE) {
				/* Set internal frequency to
				 * 19.2Mhz vs default 25Mhz */
				write_LPE_SSP2_DIV_CTRL_L(
				SSP19200MHZ_DIV_CTRL_L, lpeshim_base_address);
				write_LPE_SSP2_DIV_CTRL_H(
				SSP19200MHZ_DIV_CTRL_H, lpeshim_base_address);
			} else {
				dev_info(drv_data->ssp_dev, "NO DIV_CTRL FOR SSP0!\n");
				WARN(1, "Trying to use SSP0 with ACPI SSP driver\n");
			}
		}
#ifdef _LLI_ENABLED_
		i2s_enable(drv_data, true);
#else
		i2s_enable(drv_data);
#endif /* _LLI_ENABLED_ */
		break;

	case SSP_CMD_DISABLE_SSP:
#ifdef _LLI_ENABLED_
		i2s_enable(drv_data, false);
#else
		i2s_disable(drv_data);
#endif /* _LLI_ENABLED_ */
		break;

#ifdef _LLI_ENABLED_
	case SSP_CMD_ENABLE_DMA_RX_INTR:
		i2s_enable_dma_rx_intr(drv_data, true);
		break;

	case SSP_CMD_ENABLE_DMA_TX_INTR:
		i2s_enable_dma_tx_intr(drv_data, true);
		break;

	case SSP_CMD_DISABLE_DMA_RX_INTR:
		i2s_enable_dma_rx_intr(drv_data, false);
		break;

	case SSP_CMD_DISABLE_DMA_TX_INTR:
		i2s_enable_dma_tx_intr(drv_data, false);
		break;
#endif /* _LLI_ENABLED_ */

	case SSP_CMD_ALLOC_TX:
		ssp_settings = &(drv_data->current_settings);

		/* Check if DMA channel allocation needed */
		if (ssp_settings->ssp_tx_dma != SSP_TX_DMA_ENABLE)
			break;

		WARN(!drv_data->dmacdev, "DMA device=NULL\n");
		if (!drv_data->dmacdev)
			return -EFAULT;

		if (ssp_settings->mode == SSP_INVALID_MODE) {
			WARN(1, "Trying to alloc TX channel however"
				"the HW Config is NOT set\n");
			retval = -1;
			goto err_exit;
		}

		WARN(drv_data->txchan, "Trying to realloc TX channel\n");
		if (drv_data->txchan != NULL)
			return -EFAULT;

		WARN((!(ssp_settings->ssp_active_tx_slots_map)),
				"Trying to alloc non active tx channel\n");

		/* 2. init tx channel */
		txs = &drv_data->dmas_tx;
		txs->dma_slave.direction = DMA_TO_DEVICE;
		txs->hs_mode = LNW_DMA_HW_HS;
		txs->cfg_mode = LNW_DMA_MEM_TO_PER;
		txs->dma_slave.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
#ifdef _LLI_ENABLED_
		txs->dma_slave.dst_addr = (drv_data->paddr + OFFSET_SSDR);
#endif /* _LLI_ENABLED_ */

		temp = i2s_compute_dma_width(ssp_settings->data_size,
						&txs->dma_slave.dst_addr_width);
		if (temp != 0) {
			dev_err(drv_data->ssp_dev,
				"TX DMA Channel Bad data_size = %d\n",
				ssp_settings->data_size);
			retval = -2;
			goto err_exit;

		}

		temp = i2s_compute_dma_msize(
			ssp_settings->ssp_tx_fifo_threshold+1,
			&txs->dma_slave.src_maxburst);

		if (temp != 0) {
			dev_err(drv_data->ssp_dev,
				"TX DMA Channel Bad TX FIFO Threshold = %d\n",
				ssp_settings->ssp_tx_fifo_threshold);
			retval = -3;
			goto err_exit;

		}

		temp = i2s_compute_dma_msize(
			ssp_settings->ssp_tx_fifo_threshold+1,
			&txs->dma_slave.dst_maxburst);

		if (temp != 0) {
			dev_err(drv_data->ssp_dev,
				"TX DMA Channel Bad TX FIFO Threshold = %d\n",
				ssp_settings->ssp_tx_fifo_threshold);
			retval = -4;
			goto err_exit;
		}

		txs->device_instance = drv_data->device_instance;
		dma_cap_zero(mask);
		dma_cap_set(DMA_SLAVE, mask);
		dma_cap_set(DMA_MEMCPY, mask);

		drv_data->txchan = dma_request_channel(mask, chan_filter,
							drv_data);
		if (!drv_data->txchan) {
			dev_err(drv_data->ssp_dev,
				"Could not get Tx channel\n");
			retval = -5;
			goto err_exit;
		}

		temp = drv_data->txchan->device->device_control(
				drv_data->txchan, DMA_SLAVE_CONFIG,
				(unsigned long) &txs->dma_slave);
		if (temp) {
			dev_err(drv_data->ssp_dev,
					"Tx slave control failed\n");
			retval = -6;
			goto err_exit;
		}
		break;

	case SSP_CMD_FREE_TX:
		ssp_settings = &(drv_data->current_settings);

		/* Check if DMA channel allocation needed */
		if (ssp_settings->ssp_tx_dma != SSP_TX_DMA_ENABLE) {
			i2s_reset_command_done(drv_data, SSP_CMD_FREE_TX);
			break;
		}

		WARN(!drv_data->dmacdev, "DMA device=NULL\n");
		if (!drv_data->dmacdev)
			return -EFAULT;

		channel = drv_data->txchan;

		WARN(!channel, "Trying to free non existant TX channel\n");
		if (channel == NULL)
			return -EFAULT;

		WARN((!(ssp_settings->ssp_active_tx_slots_map)),
				"Trying to free non active tx channel\n");

		s = channel->device->device_control(channel,
							DMA_TERMINATE_ALL, 0);
		WARN(s, "DMA TERMINATE of TX returns error\n");

		dma_release_channel(channel);
		dma_unmap_single(NULL, drv_data->write_ptr.dma,
				 drv_data->write_len, DMA_TO_DEVICE);
		i2s_reset_command_done(drv_data, SSP_CMD_FREE_TX);
		drv_data->txchan = NULL;
		break;

	case SSP_CMD_ALLOC_RX:
		ssp_settings = &(drv_data->current_settings);

		/* Check if DMA channel allocation needed */
		if (ssp_settings->ssp_rx_dma != SSP_RX_DMA_ENABLE)
			break;

		WARN(!drv_data->dmacdev, "DMA device=NULL\n");
		if (!drv_data->dmacdev)
			return -EFAULT;

		if (ssp_settings->mode == SSP_INVALID_MODE) {
			WARN(1, "Trying to alloc RX channel however"
				"the HW Config is NOT set\n");
			retval = -7;
			goto err_exit;
		}

		WARN(drv_data->rxchan, "Trying to realloc RX channel\n");
		if (drv_data->rxchan != NULL)
			return -EFAULT;

		WARN((!(ssp_settings->ssp_active_rx_slots_map)),
				"Trying to alloc non active rx channel\n");

		/* 1. init rx channel */
		rxs = &drv_data->dmas_rx;
		rxs->dma_slave.direction = DMA_FROM_DEVICE;
		rxs->hs_mode = LNW_DMA_HW_HS;
		rxs->cfg_mode = LNW_DMA_PER_TO_MEM;
		temp = i2s_compute_dma_width(ssp_settings->data_size,
						&rxs->dma_slave.src_addr_width);

		if (temp != 0) {
			dev_err(drv_data->ssp_dev,
				"RX DMA Channel Bad data_size = %d\n",
				ssp_settings->data_size);
			retval = -8;
			goto err_exit;

		}
		rxs->dma_slave.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
#ifdef _LLI_ENABLED_
		rxs->dma_slave.src_addr = (drv_data->paddr + OFFSET_SSDR);
#endif /* _LLI_ENABLED_ */

		temp = i2s_compute_dma_msize(
			ssp_settings->ssp_rx_fifo_threshold
			, &rxs->dma_slave.src_maxburst);

		if (temp != 0) {
			dev_err(drv_data->ssp_dev,
				"RX DMA Channel Bad RX FIFO Threshold\n");
			retval = -9;
			goto err_exit;

		}

		temp = i2s_compute_dma_msize(
			ssp_settings->ssp_rx_fifo_threshold
			, &rxs->dma_slave.dst_maxburst);

		if (temp != 0) {
			dev_err(drv_data->ssp_dev,
				"RX DMA Channel Bad RX FIFO Threshold\n");
			retval = -10;
			goto err_exit;

		}

		rxs->device_instance = drv_data->device_instance;
		dma_cap_zero(mask);
		dma_cap_set(DMA_MEMCPY, mask);
		dma_cap_set(DMA_SLAVE, mask);
		drv_data->rxchan = dma_request_channel(mask, chan_filter,
							drv_data);
		if (!drv_data->rxchan) {
			dev_err(drv_data->ssp_dev,
				"Could not get Rx channel\n");
			retval = -11;
			goto err_exit;
		}

		temp = drv_data->rxchan->device->device_control(
				drv_data->rxchan, DMA_SLAVE_CONFIG,
				(unsigned long) &rxs->dma_slave);

		if (temp) {
			dev_err(drv_data->ssp_dev,
				"Rx slave control failed\n");
			retval = -12;
			goto err_exit;
		}
		break;

	case SSP_CMD_FREE_RX:
		ssp_settings = &(drv_data->current_settings);

		/* Check if DMA channel allocation needed */
		if (ssp_settings->ssp_rx_dma != SSP_RX_DMA_ENABLE) {
			i2s_reset_command_done(drv_data, SSP_CMD_FREE_RX);
			break;
		}

		WARN(!drv_data->dmacdev, "DMA device=NULL\n");
		if (!drv_data->dmacdev)
			return -EFAULT;

		channel = drv_data->rxchan;

		WARN(!channel, "Trying to free non existant RX channel\n");
		if (channel == NULL)
			return -EFAULT;

		WARN((!(ssp_settings->ssp_active_rx_slots_map)),
				"Trying to free non active rx channel\n");

		s = channel->device->device_control(channel,
							DMA_TERMINATE_ALL, 0);

		WARN(s, "DMA TERMINATE of RX returns error\n");

		dma_release_channel(channel);
		dma_unmap_single(NULL, drv_data->read_ptr.dma,
				 drv_data->read_len, DMA_FROM_DEVICE);
		i2s_reset_command_done(drv_data, SSP_CMD_FREE_RX);
		drv_data->rxchan = NULL;
		break;

	default:
		dev_warn(drv_data->ssp_dev, "Incorrect command received !");
		return -EFAULT;
		break;
	}
	return 0;

err_exit:
	if (drv_data->txchan)
		dma_release_channel(drv_data->txchan);
	if (drv_data->rxchan)
		dma_release_channel(drv_data->rxchan);
	drv_data->rxchan = NULL;
	drv_data->txchan = NULL;
	return retval;

}
EXPORT_SYMBOL_GPL(intel_mid_i2s_command);

static void ssp1_dump_registers(struct intel_mid_i2s_hdl *drv_data)
{
	u32 irq_status;
	u32 status;

	void __iomem *reg = drv_data->ioaddr;
	struct device *ddbg = drv_data->ssp_dev;
	dev_dbg(ddbg, "Dump - Base Address = 0x%08X\n", (u32)reg);

	irq_status = read_SSSR(reg);
	dev_dbg(ddbg, "dump SSSR=0x%08X\n", irq_status);
	status = read_SSCR0(reg);
	dev_dbg(ddbg, "dump SSCR0=0x%08X\n", status);
	status = read_SSCR1(reg);
	dev_dbg(ddbg, "dump SSCR1=0x%08X\n", status);
	status = read_SSPSP(reg);
	dev_dbg(ddbg, "dump SSPSP=0x%08X\n", status);
	status = read_SSTSA(reg);
	dev_dbg(ddbg, "dump SSTSA=0x%08X\n", status);
	status = read_SSRSA(reg);
	dev_dbg(ddbg, "dump SSRSA=0x%08X\n", status);
	status = read_SSTO(reg);
	dev_dbg(ddbg, "dump SSTO=0x%08X\n", status);
	status = read_SSITR(reg);
	dev_dbg(ddbg, "dump SSITR=0x%08X\n", status);
	status = read_SSTSS(reg);
	dev_dbg(ddbg, "dump SSTSS=0x%08X\n", status);
	status = read_SSACD(reg);
	dev_dbg(ddbg, "dump SSACD=0x%08X\n", status);

#ifdef __MRFL_SPECIFIC__
	status = read_SSCR2(reg);
	dev_dbg(ddbg, "dump SSCR2=0x%08X\n", status);
	status = read_SSFS(reg);
	dev_dbg(ddbg, "dump SSFS=0x%08X\n", status);
	status = read_SFIFOL(reg);
	dev_dbg(ddbg, "dump SFIFOL=0x%08X\n", status);
	status = read_SFIFOTT(reg);
	dev_dbg(ddbg, "dump SFIFOTT=0x%08X\n", status);
	status = read_SSCR3(reg);
	dev_dbg(ddbg, "dump SSCR3=0x%08X\n", status);
	status = read_SSCR4(reg);
	dev_dbg(ddbg, "dump SSCR4=0x%08X\n", status);
	status = read_SSCR5(reg);
	dev_dbg(ddbg, "dump SSCR5=0x%08X\n", status);
#endif /* __MRFL_SPECIFIC__ */
}

/**
 * i2s_irq(): function that handles the SSP Interrupts (errors)
 * @irq : IRQ Number
 * @dev_id : structure that contains driver information
 *
 * This interrupts do nothing but warnings in case there is some problems
 * in I2S connection (underruns, overruns...). This may be reported by adding a
 * new interface to the driver, but not yet requested by "users" of this driver
 *
 * Output parameters
 *      NA
 */
static irqreturn_t i2s_irq(int irq, void *dev_id)
{
	irqreturn_t		 irq_status	= IRQ_NONE;
	struct intel_mid_i2s_hdl *drv_data	= dev_id;
	void __iomem		 *reg		= drv_data->ioaddr;
	struct device		 *ddbg		= drv_data->ssp_dev;
	/* SSSR register content */
	u32			 sssr		= 0;
	/* Monitored SSSR bit mask */
	u32			 sssr_masked	= 0;
	/* SSSR bits that need to be cleared after event handling */
	u32			 sssr_clr_mask	= 0;
	/* SSSR register content */
	u32			 isrx		= 0;

#ifdef CONFIG_PM_SLEEP
	if (ddbg->power.is_prepared)
		goto i2s_irq_return;
#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_PM_RUNTIME
	if (ddbg->power.runtime_status != RPM_ACTIVE)
		goto i2s_irq_return;
#endif /* CONFIG_PM_RUNTIME */

	sssr = read_SSSR(reg);
	sssr_masked = sssr & (drv_data->mask_sr);

	/* Any processing needed ? */
	if (!sssr_masked)
		goto i2s_irq_return;

	/*
	 * Some processing needs to be done
	 */
	irq_status = IRQ_HANDLED;

	/* may be improved by using a tasklet to send the error
	 * (underrun,...) to client by using callback
	 */
	/* Handle Receive Over Run event */
	if (sssr_masked & (SSSR_ROR_MASK << SSSR_ROR_SHIFT)) {
		dev_warn(ddbg, "%s RX FIFO OVER RUN SSSR=0x%08X\n",
			__func__, sssr);
		sssr_clr_mask |= (SSSR_ROR_MASK << SSSR_ROR_SHIFT);
	}
	/* Handle Transmit Under Run event */
	if (sssr_masked & (SSSR_TUR_MASK << SSSR_TUR_SHIFT)) {
		dev_warn(ddbg, "%s TX FIFO UNDER RUN SSSR=0x%08X\n",
			__func__, sssr);
		sssr_clr_mask |= (SSSR_TUR_MASK << SSSR_TUR_SHIFT);
	}
	/* Handle Receiver time-out INTerrupt event */
	if (sssr_masked & (SSSR_TINT_MASK << SSSR_TINT_SHIFT)) {
		dev_dbg(ddbg, "%s RX TIME OUT SSSR=0x%08X\n",
			__func__, sssr);
		sssr_clr_mask |= (SSSR_TINT_MASK << SSSR_TINT_SHIFT);
	}
	/* Handle Peripheral trailing byte INTerrupt event */
	if (sssr_masked & (SSSR_PINT_MASK << SSSR_PINT_SHIFT)) {
		dev_dbg(ddbg, "%s TRAILING BYTE SSSR=0x%08X\n",
			__func__, sssr);
		sssr_clr_mask |= (SSSR_PINT_MASK << SSSR_PINT_SHIFT);
	}
	/* Handle End Of Chain event */
	if (sssr_masked & (SSSR_EOC_MASK << SSSR_EOC_SHIFT)) {
		dev_dbg(ddbg, "%s END OF CHAIN SSSR=0x%08X\n",
			__func__, sssr);
		sssr_clr_mask |= (SSSR_EOC_MASK << SSSR_EOC_SHIFT);
	}

	/* Handle Receive Fifo Service request event */
	if (sssr_masked & (SSSR_RFS_MASK << SSSR_RFS_SHIFT))
		irq_status = i2s_irq_handle_RFS(drv_data, sssr);

	/* Handle Transmit Fifo Service request event */
	if (sssr_masked & (SSSR_TFS_MASK << SSSR_TFS_SHIFT))
		irq_status = i2s_irq_handle_TFS(drv_data, sssr);

	/* Clear sticky bits */
	write_SSSR((sssr & sssr_clr_mask), reg);

	/* Clear LPE sticky bits depending on SSP instance */

	if (test_bit(FEAT_ISRX_IMRX, &ssp_ip_features)) {
		switch (drv_data->device_instance) {
		case SSP0_INSTANCE:
			isrx = LPE_ISRX_IAPIS_SSP0_MASK <<
						LPE_ISRX_IAPIS_SSP0_SHIFT;
			write_LPE_ISRX(isrx, lpeshim_base_address);
			break;
		case SSP1_INSTANCE:
			isrx = LPE_ISRX_IAPIS_SSP1_MASK <<
						LPE_ISRX_IAPIS_SSP1_SHIFT;
			write_LPE_ISRX(isrx, lpeshim_base_address);
			break;
		case SSP2_INSTANCE:
			isrx = LPE_ISRX_IAPIS_SSP2_MASK <<
						LPE_ISRX_IAPIS_SSP2_SHIFT;
			write_LPE_ISRX(isrx, lpeshim_base_address);
			break;
		default:
			dev_dbg(ddbg, "no instance info to clear ISRX\n");
			break;
		}
	}

i2s_irq_return:
	return irq_status;
}

static inline
irqreturn_t i2s_irq_handle_RFS(struct intel_mid_i2s_hdl *drv_data, u32 sssr)
{
	irqreturn_t	irq_status	= IRQ_HANDLED;
	u16		data_cnt	= 0;

	if (drv_data->current_settings.ssp_rx_dma == SSP_RX_DMA_ENABLE) {
		dev_warn(drv_data->ssp_dev,
			 "%s: DMA enabled, this function shouldn't be called",
			 __func__);
		goto i2s_irq_handle_RFS_return;
	}

	if ((drv_data->read_ptr.cpu == NULL) || (drv_data->read_len <= 0)) {
		dev_warn(drv_data->ssp_dev,
			 "%s: Invalid read param: addr=0x%08X, len=%i",
			 __func__, (int)drv_data->read_ptr.cpu,
			 drv_data->read_len);
		goto i2s_irq_handle_RFS_return;
	}

	/* Get available data count in receive FIFO */
	if (GET_SSSR_val(sssr, RNE))
		data_cnt = GET_SSSR_val(sssr, RFL) + 1;

	/* Read data from receive FIFO */
	while ((data_cnt > 0) && (drv_data->read_len > 0)) {
		*(drv_data->read_ptr.cpu) = (u16)read_SSDR(drv_data->ioaddr);
		drv_data->read_ptr.cpu++;
		drv_data->read_len--;
		data_cnt--;
	}

	/* Check for last data */
	if (drv_data->read_len <= 0) {
		/* Stop interruption */
		drv_data->mask_sr &= ~((SSSR_RFS_MASK << SSSR_RFS_SHIFT) |
				       (SSSR_ROR_MASK << SSSR_ROR_SHIFT));
		clear_SSCR1_reg((drv_data->ioaddr), RIE);
		change_SSCR0_reg(drv_data->ioaddr, RIM,
				 SSP_RX_FIFO_OVER_INT_DISABLE);

		/* Schedule irq thread for final treatment
		 * in i2s_irq_deferred */
		set_bit(I2S_PORT_COMPLETE_READ, &drv_data->flags);
		irq_status = IRQ_WAKE_THREAD;
	}

i2s_irq_handle_RFS_return:
	return irq_status;
}

static inline
irqreturn_t i2s_irq_handle_TFS(struct intel_mid_i2s_hdl *drv_data, u32 sssr)
{
	irqreturn_t	irq_status	= IRQ_HANDLED;
	u16		data_cnt	= FIFO_SIZE;

	if (drv_data->current_settings.ssp_tx_dma == SSP_TX_DMA_ENABLE) {
		dev_warn(drv_data->ssp_dev,
			 "%s: DMA enabled, this function shouldn't be called",
			 __func__);
		goto i2s_irq_handle_TFS_return;
	}

	if ((drv_data->write_ptr.cpu == NULL) || (drv_data->write_len <= 0)) {
		dev_warn(drv_data->ssp_dev,
			 "%s: Invalid write param: addr=0x%08X, len=%i",
			 __func__, (int)drv_data->write_ptr.cpu,
			 drv_data->write_len);
		goto i2s_irq_handle_TFS_return;
	}

	/* Get available space in transmit FIFO */
	if (GET_SSSR_val(sssr, TNF))
		data_cnt -= GET_SSSR_val(sssr, TFL);

	/* Write data to transmit FIFO */
	while ((data_cnt > 0) && (drv_data->write_len > 0)) {
		write_SSDR((u32)(*(drv_data->write_ptr.cpu)),
			   drv_data->ioaddr);
		drv_data->write_ptr.cpu++;
		drv_data->write_len--;
		data_cnt--;
	}

	/* Check for last data */
	if (drv_data->write_len <= 0) {
		/* Stop interruption */
		drv_data->mask_sr &= ~((SSSR_TFS_MASK << SSSR_TFS_SHIFT) |
				       (SSSR_TUR_MASK << SSSR_TUR_SHIFT));
		clear_SSCR1_reg((drv_data->ioaddr), TIE);
		change_SSCR0_reg(drv_data->ioaddr, TIM,
				 SSP_TX_FIFO_UNDER_INT_DISABLE);

		/* Schedule irq thread for final treatment
		 * in i2s_irq_deferred */
		set_bit(I2S_PORT_COMPLETE_WRITE, &drv_data->flags);
		irq_status = IRQ_WAKE_THREAD;
	}

i2s_irq_handle_TFS_return:
	return irq_status;
}

static irqreturn_t i2s_irq_deferred(int irq, void *dev_id)
{
	/* Locals */
	struct intel_mid_i2s_hdl *drv_data = dev_id;

	dev_dbg(drv_data->ssp_dev, "%s", __func__);

	/* Finalize reading without DMA */
	if ((drv_data->current_settings.ssp_rx_dma != SSP_RX_DMA_ENABLE) &&
	    test_and_clear_bit(I2S_PORT_COMPLETE_READ, &drv_data->flags)) {
		if (!test_bit(I2S_PORT_READ_BUSY, &drv_data->flags)) {
			dev_dbg(drv_data->ssp_dev,
				"%s: spurious read complete",
				__func__);
		}

		dev_dbg(drv_data->ssp_dev,
			"%s: read complete",
			__func__);

		i2s_finalize_read(drv_data);

	/* Finalize writing without DMA */
	} else
	  if ((drv_data->current_settings.ssp_tx_dma != SSP_TX_DMA_ENABLE) &&
	      test_and_clear_bit(I2S_PORT_COMPLETE_WRITE, &drv_data->flags)) {

		if (!test_bit(I2S_PORT_WRITE_BUSY, &drv_data->flags)) {
			dev_dbg(drv_data->ssp_dev,
				"%s : spurious write complete",
				__func__);
		}

		dev_dbg(drv_data->ssp_dev,
			"%s : write complete",
			__func__);

		i2s_finalize_write(drv_data);

	/* Error */
	} else {
		dev_warn(drv_data->ssp_dev,
			 "%s: Unexpected function call"
			 "- flags=0x%08lx, rx_dma=%d, tx_dma=%d",
			 __func__,
			 drv_data->flags,
			 drv_data->current_settings.ssp_rx_dma,
			 drv_data->current_settings.ssp_tx_dma);
	}

	return IRQ_HANDLED;
}

static void i2s_finalize_read(struct intel_mid_i2s_hdl *drv_data)
{
	/* Mask Rx fifo overrun irq */
	change_SSCR0_reg(drv_data->ioaddr, RIM, SSP_RX_FIFO_OVER_INT_DISABLE);

	/* Reset read param:
	 *   must be done before call to the callback
	 *   where a read can be rearmed */
	drv_data->read_len	= 0;
	drv_data->read_ptr.cpu	= NULL;

	/* Do not change order sequence:
	 * READ_BUSY clear, then test PORT_CLOSING
	 * wakeup for close() function
	 */
	clear_bit(I2S_PORT_READ_BUSY, &drv_data->flags);
	if (!test_bit(I2S_PORT_CLOSING, &drv_data->flags)) {
		if (drv_data->read_callback != NULL)
			(void)drv_data->read_callback
					(drv_data->read_param);
		else
			dev_warn(drv_data->ssp_dev,
				 "%s: no callback set",
				 __func__);
	}
}

static void i2s_finalize_write(struct intel_mid_i2s_hdl *drv_data)
{
	/* Mask Tx fifo underrun irq */
	change_SSCR0_reg(drv_data->ioaddr, TIM, SSP_TX_FIFO_UNDER_INT_DISABLE);

	/* Reset write param:
	 *   must be done before call to the callback
	 *   where a write can be rearmed */
	drv_data->write_len	= 0;
	drv_data->write_ptr.cpu	= NULL;

	/* Do not change order sequence:
	 * WRITE_BUSY clear, then test PORT_CLOSING
	 * wakeup for close() function
	 */
	clear_bit(I2S_PORT_WRITE_BUSY, &drv_data->flags);
	if (!test_bit(I2S_PORT_CLOSING, &drv_data->flags)) {
		if (drv_data->write_callback != NULL)
			(void)drv_data->write_callback
					(drv_data->write_param);
		else
			dev_warn(drv_data->ssp_dev,
				 "%s: no callback set",
				 __func__);
	}
}

/**
 * calculate_sspsp_psp - separate function that calculate sspsp register
 * @ps_settings : pointer of the settings struct
 *
 * this function is to simplify/clarify set_ssp_i2s_hw function
 *
 *
 * Output parameters
 *      u32 : calculated SSPSP register
 */
static u32 calculate_sspsp_psp(const struct intel_mid_i2s_settings *ps_settings)
{
	u32 sspsp;
	sspsp = SSPSP_reg(FSRT,	ps_settings->ssp_frmsync_timing_bit)
		|SSPSP_reg(ETDS,	ps_settings->ssp_end_transfer_state)
		|SSPSP_reg(SCMODE,	ps_settings->ssp_serial_clk_mode)
		|SSPSP_reg(DMYSTOP,	ps_settings->ssp_psp_T4)
		|SSPSP_reg(SFRMDLY,	ps_settings->ssp_psp_T5)
		|SSPSP_reg(SFRMWDTH,	ps_settings->ssp_psp_T6)
		|SSPSP_reg(SFRMP,	ps_settings->ssp_frmsync_pol_bit);
	return sspsp;
}

/**
 * calculate_sscr0_scr - separate function that calculate scr register (master)
 * @ps_settings : pointer of the settings struct
 *
 * this function is to simplify/clarify set_ssp_i2s_hw function
 *
 * Output parameters
 *      u32 : calculated SCR register, or 0xFFFF if error
 */
static u32 calculate_sscr0_scr(struct intel_mid_i2s_hdl *drv_data,
		    const struct intel_mid_i2s_settings *ps_settings,
		    u8 *frame_rate_divider_new)
{

	long calculated_scr, remainder;
	u16 l_ssp_data_size = ps_settings->data_size;
	enum mrst_ssp_frm_freq freq = ps_settings->master_mode_standard_freq;
	struct device *ddbg = drv_data->ssp_dev;
	u16 delay = ps_settings->ssp_psp_T2 +
		    ps_settings->ssp_psp_T4 +
		    ps_settings->ssp_psp_T1;
	int frame_sync_length, hw_freq, requested_freq;

	dev_dbg(ddbg, "delay=%d\n", delay);

	/*
	 * A delay will be taken into account only if next frame sync is
	 * asserted at the end of T4 timing
	 */
	if ((ps_settings->ssp_frmsync_timing_bit !=
			NEXT_FRMS_ASS_AFTER_END_OF_T4) && (delay != 0)) {
		dev_warn(ddbg, "Review SSP config, clock not accurate\n");
	}

	dev_dbg(ddbg, "T1=%d, T2=%d, T4=%d, T5=%d, T6=%d\n",
		ps_settings->ssp_psp_T1, ps_settings->ssp_psp_T2,
		ps_settings->ssp_psp_T4, ps_settings->ssp_psp_T5,
		ps_settings->ssp_psp_T6);

	/* frequency */
	switch (freq) {
	case SSP_FRM_FREQ_8_000:
		requested_freq = 8000;
		break;
	case SSP_FRM_FREQ_11_025:
		requested_freq = 11025;
		break;
	case SSP_FRM_FREQ_16_000:
		requested_freq = 16000;
		break;
	case SSP_FRM_FREQ_22_050:
		requested_freq = 22050;
		break;
	case SSP_FRM_FREQ_44_100:
		requested_freq = 44100;
		break;
	case SSP_FRM_FREQ_48_000:
		requested_freq = 48000;
		break;
	default:
		dev_warn(ddbg, "frequency not unsupported\n");
		return 0xFFFF;
		break;
	};

	/* bits per sample */
	if ((l_ssp_data_size != 8) && (l_ssp_data_size != 16) &&
						(l_ssp_data_size != 32)) {
		dev_warn(ddbg, "Master mode bit per sample=%d unsupported\n",
							l_ssp_data_size);
		return 0xFFFF;
	}
	/*
	 * We consider that LPE selected 19.2Mhz CLK
	 */
	frame_sync_length = delay +
		ps_settings->frame_rate_divider_control * l_ssp_data_size;

	/*
	 * Calculate the divider, and the remainder of the division
	 */
	calculated_scr = CLOCK_19200_KHZ / (requested_freq * frame_sync_length);
	remainder = CLOCK_19200_KHZ % (requested_freq * frame_sync_length);

	dev_dbg(ddbg, "calculated_scr=%ld remainder=%ld\n",
						calculated_scr, remainder);

	/*
	 * If the rest is half the divider, increment the scr
	 */
	if (remainder > (frame_sync_length * requested_freq) >> 1) {
		calculated_scr += 1;
		dev_dbg(ddbg, "calculated_scr incremented=%ld\n",
						calculated_scr);
	}

	/*
	 * Calculate the real frequency that will be set up on HW
	 */
	hw_freq = CLOCK_19200_KHZ / (calculated_scr * frame_sync_length);

	dev_dbg(ddbg, "hw_freq=%dHz High limit=%dHz Low limit=%dHz\n",
			hw_freq, (1004 * requested_freq / 1000),
			(996 * requested_freq / 1000));

	/*
	 * Allow the frequency to be requested frequency +/- 0,4%
	 */
	WARN(((hw_freq < (996 * requested_freq / 1000)) ||
				(hw_freq > (1004 * requested_freq / 1000))),
				"Master could not generate proper frequency");

	/*
	 * this do not change the frame_rate_divider, but may be optimized later
	 */
	*frame_rate_divider_new = ps_settings->frame_rate_divider_control;

	return (u32)((calculated_scr-1)<<SSCR0_SCR_SHIFT);
}

/**
 * calculate_ssacd - separate function that calculate ssacd register (master)
 * @ps_settings : pointer of the settings struct
 *
 * this function is to simplify/clarify set_ssp_i2s_hw function
 *
 * Output parameters
 *      u32 : calculated SSACD register, or 0xFFFF if error
 */
static u32 calculate_ssacd(struct intel_mid_i2s_hdl *drv_data,
		    const struct intel_mid_i2s_settings *ps_settings,
		    u8 *frame_rate_divider_new)
{
	u8 calculated_ssacd;
	u16 l_ssp_data_size = ps_settings->data_size;
	enum mrst_ssp_timeslot num_timeslot;
	enum mrst_ssp_bit_per_sample bit_per_sample;
	enum mrst_ssp_frm_freq freq = ps_settings->master_mode_standard_freq;
	struct device *ddbg = drv_data->ssp_dev;
	static u8 frame_rate_divider[SSP_TIMESLOT_SIZE] = {
		[SSP_TIMESLOT_1] = 1,
		[SSP_TIMESLOT_2] = 2,
		[SSP_TIMESLOT_4] = 4,
		[SSP_TIMESLOT_8] = 8 };

	/* timeslot */
	switch (ps_settings->frame_rate_divider_control) {
	case 1:
		num_timeslot = SSP_TIMESLOT_1;
		break;
	case 2:
		num_timeslot = SSP_TIMESLOT_2;
		break;
	case 4:
		num_timeslot = SSP_TIMESLOT_4;
		break;
	case 8:
		num_timeslot = SSP_TIMESLOT_8;
		break;
	default:
		dev_warn(ddbg, "Master mode timeslot=%d unsupported\n",
			ps_settings->frame_rate_divider_control);
		return 0xFFFF;
		break;
	};

	/* bits per sample */
	if (l_ssp_data_size == 8)
		bit_per_sample = SSP_BIT_PER_SAMPLE_8;
	else if (l_ssp_data_size == 16)
		bit_per_sample = SSP_BIT_PER_SAMPLE_16;
	else if (l_ssp_data_size == 32)
		bit_per_sample = SSP_BIT_PER_SAMPLE_32;
	else {
		dev_warn(ddbg, "Master mode bit per sample=%d unsupported\n",
				l_ssp_data_size);
		return 0xFFFF;
	}

/* 3 special cases when SSP clk force more than really used timeslots
 * to keep the wanted framesync freq. "Active" timeslots number do not change.
 * a) freq = 16Khz, active timeslot=1, bit per sample=8 => real timeslot=2
 * b) freq = 8Khz, active timeslot=1, bit per sample=8  => real timeslot=2
 * b) freq = 8Khz, active timeslot=1, bit per sample=16 => real timeslot=2
 * c) freq = 8Khz, active timeslot=2, bit per sample=8  => real timeslot=4
 * loop below will avoid these cases, and try to increment real timeslot to find
 * a working case.
 */

	while ((num_timeslot < SSP_TIMESLOT_8)
	 && (ssp_ssacd[freq][bit_per_sample][num_timeslot]
				== SSP_SSACD_NOT_AVAILABLE)) {
		num_timeslot++;
	}

/* Check of any unexpected error such as undefined freq with master mode */
	calculated_ssacd = ssp_ssacd[freq][bit_per_sample][num_timeslot];
	if (calculated_ssacd == SSP_SSACD_NOT_AVAILABLE) {
		dev_warn(ddbg, "Master mode unexpected error freq=%d,"
		"bitPerSamp=%d,calculated timeslot=%d, original timeslot=%d\n",
		freq, bit_per_sample, num_timeslot,
		ps_settings->frame_rate_divider_control);
		return 0xFFFF;
	}

	*frame_rate_divider_new = frame_rate_divider[num_timeslot];

	return (u32)(calculated_ssacd);
}

/**
 * calculate_sscr0_psp: separate function that calculate sscr0 register
 * @ps_settings : pointer of the settings struct
 *
 * this function is to simplify/clarify set_ssp_i2s_hw function
 *
 * Output parameters
 *      u32 : calculated SSCR0 register
 */
u32 calculate_sscr0_psp(const struct intel_mid_i2s_settings *ps_settings)
{
	u16 l_ssp_data_size = ps_settings->data_size;
	u32 sscr0;
	if (l_ssp_data_size > 16) {
		sscr0 =   SSCR0_reg(DSS, SSCR0_DataSize(l_ssp_data_size - 16))
			| SSCR0_reg(EDSS, 1);
	} else {
		sscr0 =   SSCR0_reg(DSS, SSCR0_DataSize(l_ssp_data_size))
			| SSCR0_reg(EDSS, 0);
	}
/*
Can be replaced by code below :
sscr0 = SSCR0_reg(DSS, (l_ssp_data_size - 1) & 0x0F)
| SSCR0_reg(EDSS, ((l_ssp_data_size - 1) & 0x10) >> 8);
*/
	sscr0 |= SSCR0_reg(MOD,	ps_settings->mode)
		|SSCR0_reg(FRF,	ps_settings->frame_format)
		|SSCR0_reg(RIM,	SSP_RX_FIFO_OVER_INT_DISABLE)
		|SSCR0_reg(TIM,	SSP_TX_FIFO_UNDER_INT_DISABLE);
	return sscr0;
}

/**
 * calculate_sscr1_psp - separate function that calculate sscr1 register
 * @ps_settings : pointer of the settings struct
 *
 * this function is to simplify/clarify set_ssp_i2s_hw function
 *
 * Output parameters
 *      u32 : calculated SSCR1 register
 */
u32 calculate_sscr1_psp(const struct intel_mid_i2s_settings *ps_settings)
{
	u32 sscr1;
	sscr1 = SSCR1_reg(SFRMDIR, ps_settings->sspsfrm_direction)
	|SSCR1_reg(SCLKDIR, ps_settings->sspslclk_direction)
	|SSCR1_reg(TTELP, ps_settings->tx_tristate_phase)
	|SSCR1_reg(TTE, ps_settings->tx_tristate_enable)
	|SSCR1_reg(TRAIL, ps_settings->ssp_trailing_byte_mode)
	|SSCR1_reg(TINTE, ps_settings->ssp_rx_timeout_interrupt_status)
	|SSCR1_reg(PINTE, ps_settings->ssp_trailing_byte_interrupt_status)
	|SSCR1_reg(LBM, ps_settings->ssp_loopback_mode_status)
	|SSCR1_reg(RWOT, ps_settings->ssp_duplex_mode)
	|SSCR1_reg(RFT, SSCR1_RxTresh(ps_settings->ssp_rx_fifo_threshold))
	|SSCR1_reg(TFT, SSCR1_TxTresh(ps_settings->ssp_tx_fifo_threshold));
	return sscr1;
}

void override_timing(struct intel_mid_i2s_hdl *drv_data)
{
#ifdef CONFIG_ACPI
	struct intel_mid_i2s_ssp_config *conf;

	switch (drv_data->current_settings.master_mode_standard_freq) {
	case SSP_FRM_FREQ_8_000:
		conf = &(drv_data->config_c08k);
		break;
	case SSP_FRM_FREQ_16_000:
		conf = &(drv_data->config_c16k);
		break;
	case SSP_FRM_FREQ_48_000:
		conf = &(drv_data->config_c48k);
		break;
	default:
		pr_info("SSP I2S driver:no override settings for this freq\n");
		return;
	}

	if (conf->valid) {
		pr_info("SSP I2S driver: OVERRIDE settings\n");
		pr_info("SSP I2S driver: settings:\n%d\n%d\n%d\n%d\n%d\n\n",
			conf->ssp_psp_T1, conf->ssp_psp_T2,
			conf->ssp_psp_T4, conf->ssp_psp_T5, conf->ssp_psp_T6);
		drv_data->current_settings.ssp_psp_T1 = conf->ssp_psp_T1;
		drv_data->current_settings.ssp_psp_T2 = conf->ssp_psp_T2;
		drv_data->current_settings.ssp_psp_T4 = conf->ssp_psp_T4;
		drv_data->current_settings.ssp_psp_T5 = conf->ssp_psp_T5;
		drv_data->current_settings.ssp_psp_T6 = conf->ssp_psp_T6;
	} else {
		pr_info("SSP I2S driver: OVERRIDE settings not valid\n");
	}
#endif
}

/**
 * set_ssp_i2s_hw - configure the SSP driver according to the ps_settings
 * @drv_data : structure that contains all details about the SSP Driver
 * @ps_settings : structure that contains SSP Hardware settings
 *
 * it also store ps_settings the drv_data
 *
 * Output parameters
 *      NA
 */
static void set_ssp_i2s_hw(struct intel_mid_i2s_hdl *drv_data,
			const struct intel_mid_i2s_settings *ps_settings_param)
{
	u32 sscr0 = 0;
	u32 sscr1 = 0;
	u32 sstsa = 0;
	u32 ssrsa = 0;
	u32 sspsp = 0;
	u32 sssr = 0;
	u32 ssacd = 0;
	u32 sscr0_scr;
	struct intel_mid_i2s_settings *ps_settings;
#ifdef __MRFL_SPECIFIC__
	u32 sfifott = 0;
#endif /* __MRFL_SPECIFIC__ */
	u8 frame_rate_divider;

	/* Get the SSP Settings */
	u16 l_ssp_clk_frm_mode = 0xFF;
	void __iomem *reg = drv_data->ioaddr;
	struct device *ddbg = drv_data->ssp_dev;
	dev_dbg(ddbg, "setup SSP I2S PCM1 configuration\n");

	/*
	 * Save the current I2S Configuration
	 */
	drv_data->current_settings = *ps_settings_param;
	ps_settings = &(drv_data->current_settings);
	/*
	 * override the timing settings if given by acpi
	 */
	override_timing(drv_data);

	if ((ps_settings->sspsfrm_direction == SSPSFRM_MASTER_MODE)
	   && (ps_settings->sspslclk_direction == SSPSCLK_MASTER_MODE)) {
		l_ssp_clk_frm_mode = SSP_IN_MASTER_MODE;
	} else if ((ps_settings->sspsfrm_direction == SSPSFRM_SLAVE_MODE)
		  && (ps_settings->sspslclk_direction == SSPSCLK_SLAVE_MODE)) {
		l_ssp_clk_frm_mode = SSP_IN_SLAVE_MODE;
	} else {
		dev_err(ddbg, "Unsupported I2S PCM1 configuration\n");
		goto leave;
	}
	dev_dbg(ddbg, "SSPSFRM_DIRECTION:%d:\n",
		ps_settings->sspsfrm_direction);
	dev_dbg(ddbg, "SSPSCLK_DIRECTION:%d:\n",
		ps_settings->sspslclk_direction);
	if (ps_settings->frame_format != PSP_FORMAT) {
		dev_err(ddbg, "UNSUPPORTED FRAME FORMAT:%d:\n",
			ps_settings->frame_format);
		goto leave;
	}

	/*********** DMA Transfer Mode ***********/
	dev_dbg(ddbg, "FORMAT :%d:\n", ps_settings->frame_format);
	sscr0 = calculate_sscr0_psp(ps_settings);
	dev_dbg(ddbg, " sscr0 :0x%08X\n", sscr0);
	sscr1 = calculate_sscr1_psp(ps_settings);
	dev_dbg(ddbg, " sscr1 :0x%08X\n", sscr1);
	if (ps_settings->mode == SSP_IN_NETWORK_MODE) {
		dev_dbg(ddbg, "MODE :%d:\n", ps_settings->mode);
		sspsp = calculate_sspsp_psp(ps_settings);
		dev_dbg(ddbg, "sspsp :0x%08X\n", sspsp);
		/* set the active TX time slot (bitmap) */
		sstsa = SSTSA_reg(TTSA, ps_settings->ssp_active_tx_slots_map);
		/* set the active RX time slot (bitmap) */
		ssrsa = SSRSA_reg(RTSA, ps_settings->ssp_active_rx_slots_map);
		frame_rate_divider = ps_settings->frame_rate_divider_control;
		/*
		 *  Master mode requires clk_output_enable in LPE_SHIM_CLKCTL
		 */
		if (l_ssp_clk_frm_mode == SSP_IN_MASTER_MODE) {
			switch (ps_settings->master_mode_clk_selection) {
			case SSP_ONCHIP_CLOCK:
				sscr0_scr = calculate_sscr0_scr(drv_data,
					     ps_settings, &frame_rate_divider);
				if (ssacd == 0xFFFF) {
					dev_err(ddbg,
						"Error during SCR calculation");
					goto leave;
				}
				sscr0 |= sscr0_scr;
				break;
			case SSP_NETWORK_CLOCK:
				sscr0 |= SSCR0_reg(NCS, 1);
				break;
			case SSP_EXTERNAL_CLOCK:
				sscr0 |= SSCR0_reg(ECS, 1);
				break;
			case SSP_ONCHIP_AUDIO_CLOCK:
				/*
				 * this mode may not be fonctionnal
				 * on all chipsets.
				 */
				ssacd = calculate_ssacd(drv_data,
					 ps_settings, &frame_rate_divider);
				if (ssacd == 0xFFFF) {
					dev_err(ddbg,
					 "Error during SSACD calculation");
					goto leave;
				}
				sscr0 |= SSCR0_reg(ACS, 1);
				break;
			default:
				dev_err(ddbg, "Master Mode clk select UNKNOWN");
				break;
			}
			sspsp |= SSPSP_reg(STRTDLY, ps_settings->ssp_psp_T1)
				|SSPSP_reg(DMYSTRT, ps_settings->ssp_psp_T2);
		} else {	/* Set the Slave Clock Free Running Status */
			sscr1 |= SSCR1_reg(SCFR,
				    ps_settings->slave_clk_free_running_status);
		}
		sscr0 |= SSCR0_reg(FRDC, SSCR0_SlotsPerFrm(frame_rate_divider));
		dev_dbg(ddbg, "sscr0 :0x%08X\n", sscr0);

	} else {  /* SSP_IN_NORMAL_MODE */
		dev_err(ddbg, "UNSUPPORTED MODE");
		goto leave;
	}

	/* Set SSP status mask */
	drv_data->mask_sr = ((SSSR_BCE_MASK << SSSR_BCE_SHIFT) |
			     (SSSR_EOC_MASK << SSSR_EOC_SHIFT) |
			     (SSSR_TINT_MASK << SSSR_TINT_SHIFT) |
			     (SSSR_PINT_MASK << SSSR_PINT_SHIFT));

	if (ps_settings->ssp_rx_dma == SSP_RX_DMA_ENABLE)
		drv_data->mask_sr |= (SSSR_ROR_MASK << SSSR_ROR_SHIFT);
	if (ps_settings->ssp_tx_dma == SSP_TX_DMA_ENABLE)
		drv_data->mask_sr |= (SSSR_TUR_MASK << SSSR_TUR_SHIFT);

	/* Clear status */
	sssr = (SSSR_BCE_MASK << SSSR_BCE_SHIFT)
	     | (SSSR_TUR_MASK << SSSR_TUR_SHIFT)
	     | (SSSR_TINT_MASK << SSSR_TINT_SHIFT)
	     | (SSSR_PINT_MASK << SSSR_PINT_SHIFT)
	     | (SSSR_ROR_MASK << SSSR_ROR_SHIFT);

#ifdef __MRFL_SPECIFIC__
	sfifott = replace_SFIFOTT_RFT(sfifott,
			SSCR1_RxTresh(ps_settings->ssp_rx_fifo_threshold));
	sfifott = replace_SFIFOTT_TFT(sfifott,
			SSCR1_TxTresh(ps_settings->ssp_tx_fifo_threshold));
	dev_dbg(ddbg, "WRITE SFIFOTT: 0x%08X\n", sfifott);
#endif /* __MRFL_SPECIFIC__ */

	/* disable SSP */
#ifdef _LLI_ENABLED_
	i2s_enable(drv_data, false);
#else
	i2s_disable(drv_data);
#endif /* _LLI_ENABLED_ */

	dev_dbg(ddbg, "WRITE SSCR0 DISABLE\n");

	/* Clear status */
	write_SSSR(sssr, reg);
	dev_dbg(ddbg, "WRITE SSSR: 0x%08X\n", sssr);
	write_SSCR0(sscr0, reg);
	dev_dbg(ddbg, "WRITE SSCR0\n");

	/* first set CR1 without interrupt and service enables */
	write_SSCR1(sscr1, reg);
	write_SSPSP(sspsp, reg);
	write_SSTSA(sstsa, reg);
	write_SSRSA(ssrsa, reg);
	write_SSACD(ssacd, reg);
#ifdef __MRFL_SPECIFIC__
	write_SFIFOTT(sfifott, reg);
#endif /* __MRFL_SPECIFIC__ */

	/* set the time out for the reception */
	write_SSTO(0, reg);
	ssp1_dump_registers(drv_data);

leave:
	return;
}

static int
intel_mid_i2s_find_usage(struct pci_dev *pdev,
			 struct intel_mid_i2s_hdl *drv_data,
			 enum intel_mid_i2s_ssp_usage *usage,
			 struct intel_mid_ssp_gpio *ssp_gpio)
{
	int pos;
	u16  adid = 0;
	int status = 0;

	*usage = SSP_USAGE_UNASSIGNED;
	pos = pci_find_capability(pdev, PCI_CAP_ID_VNDR);
	dev_info((&pdev->dev),
		"Probe/find capability (VNDR %d pos=0x%x)\n",
		PCI_CAP_ID_VNDR, pos);
	if (pos > 0) {
		pos += PCI_CAP_OFFSET_ADID;
		pci_read_config_word(pdev, pos, &adid);

		dev_info(&(pdev->dev), "Vendor capability adid = 0x%x\n", adid);
		if ((adid & 0x1F) == PCI_CAP_ADID_I2S_BT_FM)
			*usage	= SSP_USAGE_BLUETOOTH_FM;
		else if ((adid & 0x1F) == PCI_CAP_ADID_I2S_MODEM)
			*usage	= SSP_USAGE_MODEM;
		else
			*usage	= SSP_USAGE_UNASSIGNED;

		ssp_gpio->ssp_fs_gpio_mapping =
			(adid >> PCI_ADID_SSP_FS_GPIO_MAPPING_SHIFT) &
			 PCI_ADID_SSP_FS_GPIO_MAPPING_MASK;
		ssp_gpio->ssp_fs_mode =
			(adid >> PCI_ADID_SSP_FS_GPIO_MODE_SHIFT) &
			PCI_ADID_SSP_FS_GPIO_MODE_MASK;

		dev_info(&(pdev->dev),
			 "Detected PCI SSP (ID: %04x:%04x) "
			 "ssp_fs_gpio_mapping =%x "
			 "ssp_fs_mode =%x\n",
			 pdev->vendor, pdev->device,
			 ssp_gpio->ssp_fs_gpio_mapping,
			 ssp_gpio->ssp_fs_mode);
	}

	/* If there is no capability, check with old PCI_ID */
#ifdef BYPASS_ADID
	if (*usage == SSP_USAGE_UNASSIGNED) {
		dev_warn(&(pdev->dev),
			"Vendor capability not present/invalid\n");
		switch (pdev->device) {
		case MFLD_SSP1_DEVICE_ID:
		case CLV_SSP1_DEVICE_ID:
			*usage	= SSP_USAGE_BLUETOOTH_FM;
			break;
		case MFLD_SSP0_DEVICE_ID:
		case CLV_SSP0_DEVICE_ID:
			*usage	= SSP_USAGE_MODEM;
			break;
		}
	}
#endif
	/* FIXME: will be remove when correct ADID generated in PCI table */
	if (*usage == SSP_USAGE_UNASSIGNED) {
		dev_warn(&(pdev->dev), "Incorrect ADID: MRFL WA => assign 1\n");

		switch (drv_data->paddr) {
		case MRFL_SSP0_REG_BASE_ADDRESS:
			*usage	= SSP_USAGE_MODEM;
			break;
		case MRFL_SSP1_REG_BASE_ADDRESS:
			*usage	= SSP_USAGE_BLUETOOTH_FM;
			break;
		case MRFL_SSP2_REG_BASE_ADDRESS:
			/* Won't probe : no usage */
			break;
		}
	}

	if (*usage == SSP_USAGE_UNASSIGNED) {
		dev_info((&pdev->dev),
			"No probe for I2S PCI-ID: %04x:%04x, ADID(0x%x)=0x%x\n",
			pdev->vendor, pdev->device, pos, adid);
		status = -ENODEV;
		goto err_find_usage;
	}
	dev_info(&(pdev->dev),
		"Detected PCI SSP (ID: %04x:%04x) usage =%x\n",
		pdev->vendor, pdev->device, *usage);
	dev_dbg(&(pdev->dev),
		" found PCI SSP controller(ID: %04x:%04x)\n",
		pdev->vendor, pdev->device);

	/* Init the driver data structure fields*/
	switch (pdev->device) {
	case MFLD_SSP0_DEVICE_ID:
	case CLV_SSP0_DEVICE_ID:
		drv_data->device_instance = SSP0_INSTANCE;
		break;

	case MFLD_SSP1_DEVICE_ID:
	case CLV_SSP1_DEVICE_ID:
		drv_data->device_instance = SSP1_INSTANCE;
		break;

	case MRFL_SSP_DEVICE_ID:
		/* FIXME: use of MRFL_SSPx_REG_BASE_ADDRESS should be
		 * avoided by PCI table reorganization which allow to
		 * determine SSP # from PCI info */

		/* Get device instance using register base address */
		switch (drv_data->paddr) {
		case MRFL_SSP0_REG_BASE_ADDRESS:
			drv_data->device_instance = SSP0_INSTANCE;
			break;
		case MRFL_SSP1_REG_BASE_ADDRESS:
			drv_data->device_instance = SSP1_INSTANCE;
			break;
		case MRFL_SSP2_REG_BASE_ADDRESS:
			drv_data->device_instance = SSP2_INSTANCE;
			break;
		}
		break;

	default:
		dev_err(&(pdev->dev),
			"Can not determine device instance (PCI ID:%04x)\n",
			pdev->device);
		status = -ENODEV;
		goto err_find_usage;
	}

err_find_usage:
	return status;
}

/**
 * intel_mid_i2s_probe - probing function for the pci selected
 * @pdev : pci_dev pointer that is probed
 * @ent : pci_device_id
 *
 * Output parameters
 *      NA
 */
static int intel_mid_i2s_probe(struct pci_dev *pdev,
				const struct pci_device_id *ent)
{
	struct intel_mid_i2s_hdl *drv_data;
	struct intel_mid_ssp_gpio ssp_fs_pin;
	struct platform_device *asoc_pdev;
	enum intel_mid_i2s_ssp_usage usage;
	struct pci_dev *dmac1;
	resource_size_t lpeshim_base_address_p;
	resource_size_t base_address_p;
	int status = 0;
	int ret = 0;

	drv_data = kzalloc(sizeof(struct intel_mid_i2s_hdl), GFP_KERNEL);
	dev_dbg(&(pdev->dev), "%s Probe,drv_data =%p\n", DRIVER_NAME, drv_data);
	if (!drv_data) {
		dev_err((&pdev->dev), "Can't alloc driver data in probe\n");
		status = -ENOMEM;
		goto leave;
	}
	dev_info((&pdev->dev), "Detected PCI SSP (ID: %04x:%04x)\n",
				pdev->vendor, pdev->device);

	/* Enable device */
	status = pci_enable_device(pdev);
	if (status) {
		dev_err((&pdev->dev), "Can not enable device.Err=%d\n", status);
		goto err_i2s_probe0;
	}

	mutex_init(&drv_data->mutex);

	drv_data->ssp_dev = &(pdev->dev);

#ifdef CONFIG_ACPI
	/* no override of timing for PCI, only from ACPI */
	drv_data->config_c08k.valid = false;
	drv_data->config_c16k.valid = false;
	drv_data->config_c48k.valid = false;
#endif

	/*
	 * Get basic io resource and map it for SSP1 [BAR=0]
	 */
	if ((pdev->device == MFLD_SSP0_DEVICE_ID) ||
	    (pdev->device == MFLD_SSP1_DEVICE_ID) ||
	    (pdev->device == CLV_SSP0_DEVICE_ID) ||
	    (pdev->device == CLV_SSP1_DEVICE_ID) ||
	    (pdev->device == MRFL_SSP_DEVICE_ID)) {
		drv_data->paddr = pci_resource_start(pdev, MRST_SSP_BAR);
		drv_data->iolen = pci_resource_len(pdev, MRST_SSP_BAR);
		status = pci_request_region(pdev, MRST_SSP_BAR,
						dev_name(&pdev->dev));
	} else {
		dev_err(&pdev->dev,
			"Don't know which BAR to use for this SSP PCDID=%x\n",
			pdev->device);
		status = -ENODEV;
		goto err_i2s_probe1;
	}

	dev_dbg(&(pdev->dev), "paddr = : %x\n", (unsigned int) drv_data->paddr);
	dev_dbg(&(pdev->dev), "iolen = : %d\n", drv_data->iolen);

	if (status) {
		dev_err((&pdev->dev), "Can't request region. err=%d\n", status);
		goto err_i2s_probe1;
	}

	/* map bus memory into CPU space */
	drv_data->ioaddr = pci_ioremap_bar(pdev, MRST_SSP_BAR);
	if (!drv_data->ioaddr) {
		dev_err((&pdev->dev), "ioremap_nocache error\n");
		status = -ENOMEM;
		goto err_i2s_probe2;
	}

	dev_dbg(&(pdev->dev), "ioaddr = : %p\n", drv_data->ioaddr);

	/* Find SSP usage */
	ssp_fs_pin.ssp_fs_gpio_mapping = UNASSIGNED_GPIO_MAPPING;
	status = intel_mid_i2s_find_usage(pdev, drv_data, &usage, &ssp_fs_pin);
	if (status)
		goto err_i2s_probe3;

	drv_data->usage = usage;

	/* calculation of LPESHIM Base address */
	/* offset address ssp : (0xA0000+(drv_data->device_instance)*0x1000) */
	/* offset LPE Shim . 0x14000 */
	if (drv_data->device_instance == 0)
		base_address_p = drv_data->paddr - SSP0_OFFSET;
	else if (drv_data->device_instance == 1)
		base_address_p = drv_data->paddr - SSP1_OFFSET;
	else if (drv_data->device_instance == 2)
		base_address_p = drv_data->paddr - SSP2_OFFSET;
	else {
		dev_err(&pdev->dev,
			"Unknown instance %d for SSP\n",
			drv_data->device_instance);
		status = -ENODEV;
		goto err_i2s_probe3;
	}

	lpeshim_base_address_p = base_address_p + LPESHIM_OFFSET;

	pr_info("SSP I2S driver : lpeshim=%lx (MRFL=0xff340000) SSP%d\n",
	  (unsigned long int)lpeshim_base_address_p, drv_data->device_instance);
	/* remapping addresses */
	lpeshim_base_address = devm_ioremap(
				&(pdev->dev),
				lpeshim_base_address_p,
				MRFL_LPE_SHIM_REG_SIZE);

	/* prepare for DMA channel allocation */
	/* get the pci_dev structure pointer */
	switch (pdev->device) {
	case MFLD_SSP0_DEVICE_ID:
	case MFLD_SSP1_DEVICE_ID:
		dmac1 = pci_get_device(PCI_VENDOR_ID_INTEL,
						 MFLD_LPE_DMA_DEVICE_ID,
						 NULL);
		WARN(!dmac1, "DMA MFLD device=NULL\n");
		drv_data->dmacdev = &dmac1->dev;
	break;

	case CLV_SSP0_DEVICE_ID:
	case CLV_SSP1_DEVICE_ID:
		dmac1 = pci_get_device(PCI_VENDOR_ID_INTEL,
						 CLV_LPE_DMA_DEVICE_ID,
						 NULL);
		WARN(!dmac1, "DMA CLV device=NULL\n");
		drv_data->dmacdev = &dmac1->dev;
	break;

	case MRFL_SSP_DEVICE_ID:
		dmac1 = pci_get_device(PCI_VENDOR_ID_INTEL,
						 MRFL_LPE_DMA_DEVICE_ID,
						 NULL);
		WARN(!dmac1, "DMA MRFL device=NULL\n");
		drv_data->dmacdev = &dmac1->dev;
	break;

	default:
		dev_err(&pdev->dev,
			"Don't know dma device ID for this SSP PCDID=%x\n",
			pdev->device);
		goto err_i2s_probe3;
	}

	/* in case the stop dma have to wait for end of callbacks   */
	/* This will be removed when TERMINATE_ALL available in DMA */
	if (!drv_data->dmacdev) {
		/* CPU data transfer allowed if no DMA available */
		dev_warn(drv_data->ssp_dev,
			"DMACDEV not found, only CPU data transfer allowed\n");
	}

	/* increment ref count of pci device structure already done by */
	/* pci_get_device. will do a pci_dev_put when exiting the module */
	pci_set_drvdata(pdev, drv_data);

	/* set SSP FrameSync and CLK direction in INPUT mode in order
	 * to avoid disturbing peripherals
	 */
	write_SSCR1((SSCR1_SFRMDIR_MASK << SSCR1_SFRMDIR_SHIFT)
		  | (SSCR1_SCLKDIR_MASK << SSCR1_SCLKDIR_SHIFT)
		  | (SSCR1_TTE_MASK << SSCR1_TTE_SHIFT),
		  drv_data->ioaddr);

	if ((intel_mid_identify_cpu() == INTEL_MID_CPU_CHIP_PENWELL) ||
	    (intel_mid_identify_cpu() == INTEL_MID_CPU_CHIP_CLOVERVIEW)) {
		/*
		 * Switch the SSP_FS pin from GPIO Input Mode
		 * to functional Mode
		 */
		if (ssp_fs_pin.ssp_fs_gpio_mapping == UNASSIGNED_GPIO_MAPPING)
			dev_err(drv_data->ssp_dev, "Bad GPIO initialisation\n");
		else if (usage == SSP_USAGE_MODEM)
			gpio_request(ssp_fs_pin.ssp_fs_gpio_mapping,
								"ssp_modem");
		else if (usage == SSP_USAGE_BLUETOOTH_FM)
			gpio_request(ssp_fs_pin.ssp_fs_gpio_mapping,
								"ssp_bt");
		else {
			dev_err(drv_data->ssp_dev,
				"Bad SSP Usage\n");
			status = -EINVAL;
			goto err_i2s_probe3;
		}

		if (ssp_fs_pin.ssp_fs_gpio_mapping != UNASSIGNED_GPIO_MAPPING)
			lnw_gpio_set_alt(ssp_fs_pin.ssp_fs_gpio_mapping,
				ssp_fs_pin.ssp_fs_mode);

		dev_dbg(&pdev->dev, "SET GPIO_A0N %d to %d Mode\n",
			ssp_fs_pin.ssp_fs_gpio_mapping,
			ssp_fs_pin.ssp_fs_mode);
	}

	/* Attach to IRQ */
	drv_data->irq = pdev->irq;
	dev_dbg(&(pdev->dev), "attaching to IRQ: %04x\n", pdev->irq);

	status = request_threaded_irq(drv_data->irq,
					  i2s_irq,
					  i2s_irq_deferred,
					  IRQF_SHARED,
					  "i2s ssp",
					  drv_data);

	if (status < 0)	{
		dev_err(&pdev->dev, "can not get IRQ. status err=%d\n", status);
		goto err_i2s_probe3;
	}

	pm_runtime_put_noidle(drv_data->ssp_dev);
	pm_runtime_allow(drv_data->ssp_dev);

	if (usage == SSP_USAGE_MODEM) {
		WARN(test_and_set_bit(MODEM_FND, &modem_found_and_i2s_setup_ok),
			"Only one modem supported in platform\n");
		if (intel_mid_i2s_modem_probe_cb)
			(*intel_mid_i2s_modem_probe_cb)();
	}

	set_bit(FEAT_PCI, &ssp_ip_features);
	WARN(test_bit(FEAT_ACPI, &ssp_ip_features),
			"PCI and ACPI for SSP at the same time");

	/*
	 * Create the WL1273 BT/FM ASoC platform devices
	 */
	pr_info("ALLOCATE FOR ASOC\n");

	if (usage == SSP_USAGE_MODEM)
		WARN(test_and_set_bit(MODEM_USAGE_FND, &ssps_found), "SSP for modem usage already probed");

	if (usage == SSP_USAGE_BLUETOOTH_FM)
		WARN(test_and_set_bit(BT_USAGE_FND, &ssps_found), "SSP for BT/FM usage already probed");

	if (test_bit(MODEM_USAGE_FND, &ssps_found) &
		test_bit(BT_USAGE_FND, &ssps_found)) {
		/*
		 * MID SSP CPU DAI
		 */
		asoc_pdev = platform_device_alloc("mid-ssp-dai", -1);
		if (!asoc_pdev) {
			dev_err(&pdev->dev,
					"platform mid-ssp-dai allocation failed\n");
			status = -ENODEV;
			goto leave;
		}
		ret = platform_device_add(asoc_pdev);
		if (ret) {
			dev_err(&pdev->dev,
					"platform mid-ssp-dai add failed\n");
			platform_device_put(asoc_pdev);
		}
		pr_info("I2S: platform mid-ssp-dai allocated\n");
	}

	goto leave;
err_i2s_probe3:
	iounmap(drv_data->ioaddr);
err_i2s_probe2:
	pci_release_region(pdev, MRST_SSP_BAR);
err_i2s_probe1:
	pci_disable_device(pdev);
err_i2s_probe0:
	kfree(drv_data);
leave:
	return status;
}

static void intel_mid_i2s_remove(struct pci_dev *pdev)
{
	struct intel_mid_i2s_hdl *drv_data;
	enum intel_mid_i2s_ssp_usage usage;
	u32 device = pdev->device;

	drv_data = pci_get_drvdata(pdev);
	if (!drv_data) {
		dev_err(&pdev->dev, "no drv_data in pci device to remove!\n");
		goto leave;
	}
	usage = drv_data->usage;
	if (test_bit(I2S_PORT_OPENED, &drv_data->flags)) {
		dev_warn(&pdev->dev, "Not closed before removing pci_dev!\n");
		intel_mid_i2s_close(drv_data);
	}
	pci_set_drvdata(pdev, NULL);
	/* Stop DMA is already done during close()  */
	/* Disable the SSP at the peripheral and SOC level */
	write_SSCR0(0, drv_data->ioaddr);
	free_irq(drv_data->irq, drv_data);
	iounmap(drv_data->ioaddr);
	pci_release_region(pdev, MRST_SSP_BAR);
	pci_release_region(pdev, MRST_LPE_BAR);
	pci_disable_device(pdev);
	kfree(drv_data);
	if (usage == SSP_USAGE_MODEM) {
		clear_bit(MODEM_FND, &modem_found_and_i2s_setup_ok);
		if (intel_mid_i2s_modem_remove_cb)
			(*intel_mid_i2s_modem_remove_cb)();
	}
	if (device == CLV_SSP0_DEVICE_ID)
		clear_bit(MODEM_USAGE_FND, &ssps_found);
	if (device == CLV_SSP1_DEVICE_ID)
		clear_bit(BT_USAGE_FND, &ssps_found);

leave:
	return;
}

	/*
	 * ACPI driver
	 */

int i2s_acpi_remove(struct platform_device *platdev)
{
	struct intel_mid_i2s_hdl *drv_data;

	drv_data = (struct intel_mid_i2s_hdl *) dev_get_drvdata(&platdev->dev);
	pr_info(" I2S : remove acpi device, free drv_data %p\n", drv_data);

	if (!drv_data) {
		dev_err(&platdev->dev, "no drv_data in pci device to remove!\n");
		return 0;
	}

	if (test_bit(I2S_PORT_OPENED, &drv_data->flags)) {
		dev_warn(drv_data->ssp_dev, "Not closed before removing pci_dev!\n");
		intel_mid_i2s_close(drv_data);
	}
	dev_set_drvdata(&platdev->dev, NULL);
	/* Stop DMA is already done during close()  */
	/* pci_dev_put(drv_data->dmac1);  */
	write_SSCR0(0, drv_data->ioaddr);
	free_irq(drv_data->irq, drv_data);

	kfree(drv_data);

	return 0;
}

#ifdef CONFIG_ACPI
void
retrieve_ssp_config(acpi_handle handle, struct intel_mid_i2s_ssp_config *config,
			char *id)
{
	acpi_status status = AE_OK;
	struct acpi_buffer obj_buffer = { ACPI_ALLOCATE_BUFFER, NULL };
	union acpi_object *out_obj;
	int i = 0;

	status = acpi_evaluate_object(handle, id, NULL, &obj_buffer);
	pr_err("%s:acpi_evaluate_object %s, status:%d\n", __func__, id, status);
	if (ACPI_FAILURE(status)) {
		pr_err("%s: ERROR %d evaluating ID:%s\n", __func__, status, id);
		return;
	}

	out_obj = obj_buffer.pointer;
	if (!out_obj || out_obj->type != ACPI_TYPE_STRING) {
		pr_err("%s: Invalid type: ACPI_TYPE_STRING for Id:%s\n", __func__, id);
		status = AE_BAD_PARAMETER;
		return;
	}

	pr_info("Config %s = %s\n", id, out_obj->string.pointer);

	while (config_assoc_table[i].config_name[0]) {
		if (strstr(out_obj->string.pointer,
		    config_assoc_table[i].config_name)) {
			enum ssp_config_type type = config_assoc_table[i].type;
			config->ssp_psp_T1 = ssp_config[type].ssp_psp_T1;
			config->ssp_psp_T2 = ssp_config[type].ssp_psp_T2;
			config->ssp_psp_T4 = ssp_config[type].ssp_psp_T4;
			config->ssp_psp_T5 = ssp_config[type].ssp_psp_T5;
			config->ssp_psp_T6 = ssp_config[type].ssp_psp_T6;
			config->valid = true;
			pr_info("Config found :\nT1=%d\nT2=%d\nT4=%d\nT5=%d\nT6=%d\n",
				config->ssp_psp_T1,
				config->ssp_psp_T2,
				config->ssp_psp_T4,
				config->ssp_psp_T5,
				config->ssp_psp_T6);
			return;
		}
		i++;
	}
	status = AE_BAD_PARAMETER;
	pr_info("Config not found, will not override %s settings\n", id);
	return;

}
#endif

static const u8 intel_ssp_uuid_usage[] = {
	/* 886a3f26-600c-4401-b7b1-01e9c2e7e77e */
	0x26, 0x3f, 0x6a, 0x88,			/* inverted */
	0x0c, 0x60,				/* inverted */
	0x01, 0x44,				/* inverted */
	0xb7, 0xb1,				/* in order */
	0x01, 0xe9, 0xc2, 0xe7, 0xe7, 0x7e	/* in order */
};

static const u8 intel_ssp_uuid_instance[] = {
	/* 30d3f83e-2ee1-4bf0-86e9-f69ded2887ee */
	0x3e, 0xf8, 0xd3, 0x30,
	0xe1, 0x2e,
	0xf0, 0x4b,
	0x86, 0xe9,
	0xf6, 0x9d, 0xed, 0x28, 0x87, 0xee
};

static const u8 intel_ssp_uuid_lpebase[] = {
	/* 208b1400-f7c8-4325-ab32-53cd79b7d0a6 */
	0x00, 0x14, 0x8b, 0x20,
	0xc8, 0xf7,
	0x25, 0x43,
	0xab, 0x32,
	0x53, 0xcd, 0x79, 0xb7, 0xd0, 0xa6
};

static const u8 intel_ssp_uuid_timings[] = {
	/* e6e37c60-e78b-4fbd-bd26-5bd3667a6c9a */
	0x60, 0x7c, 0xe3, 0xe6,
	0x8b, 0xe7,
	0xbd, 0x4f,
	0xbd, 0x26,
	0x5b, 0xd3, 0x66, 0x7a, 0x6c, 0x9a
};

#ifdef CONFIG_ACPI
int
i2s_get_ssp_param(acpi_handle handle, const u8 *intel_ssp_uuid,
			acpi_object_type acpi_type,
			int arg1, int arg2, int arg3,
			struct acpi_buffer *output)
{
	struct acpi_object_list input;
	union acpi_object params[4];
	union acpi_object *obj;
	int ret;

	output->length = ACPI_ALLOCATE_BUFFER;
	output->pointer = NULL;

	pr_debug("********* I2S driver ACPI5 entries *******:\n");
	input.count = ACPI_UUID_NBPARAMS;
	input.pointer = params;
	params[0].type = ACPI_TYPE_BUFFER;
	params[0].buffer.length = sizeof(intel_ssp_uuid_usage); /* size of uuid */
	params[0].buffer.pointer = (char *)intel_ssp_uuid;
	params[1].type = ACPI_TYPE_INTEGER;
	params[1].integer.value = arg1;
	params[2].type = ACPI_TYPE_INTEGER;
	params[2].integer.value = arg2;
	params[3].type = ACPI_TYPE_INTEGER;
	params[3].integer.value = arg3;

	ret = acpi_evaluate_object(handle, "_DSM", &input, output);
	if (ret) {
		pr_info("I2S driver failed to evaluate _DSM: %d\n", ret);
		return -ENODEV;
	} else
		pr_debug("I2S driver OK to evaluate _DSM: %d\n", ret);

	obj = (union acpi_object *)output->pointer;
	pr_debug("I2S obj type count=%d et add = %p ou %p\n", output->length, output->pointer, obj);
	if (!obj) {
		pr_info("I2S driver failed to get obj output!!!\n");
		return -ENODEV;
	}

	if (obj->type == ACPI_TYPE_STRING)
		pr_debug("I2S obj is >%s<\n", obj->string.pointer);
	else if (obj->type == ACPI_TYPE_INTEGER)
		pr_debug("I2S obj is =%d\n", (int)obj->integer.value);
	else if (obj->type == ACPI_TYPE_BUFFER)
		pr_debug("I2S obj is [%d]=%02x,%02x,%02x,%02x\n", obj->buffer.length, obj->buffer.pointer[0], obj->buffer.pointer[1], obj->buffer.pointer[2], obj->buffer.pointer[3]);
	else
		pr_debug("I2S obj is UNKNOWN TYPE %d\n", obj->type);

	if (obj->type != acpi_type)
		return -ENODEV;
	return 0;
}
#endif

int i2s_acpi_probe(struct platform_device *platdev)
{

#ifndef CONFIG_ACPI
	WARN(true, "== Probing ACPI without ACPI in kernel ! ==");
	return -ENODEV;
#else
	const char *hid;
	struct resource *rsrc;
	acpi_handle handle = ACPI_HANDLE(&platdev->dev);
	struct acpi_device *device;
	resource_size_t lpeshim_base_address_p;
	resource_size_t base_address_p;
	int status = 0;
	int ret;
	struct intel_mid_i2s_hdl *drv_data;
	struct platform_device *asoc_pdev;

	union acpi_object arg;
	struct acpi_object_list arg_list;
	unsigned long long value;
	acpi_status a_status = AE_OK;

	/* not MID CPU and ACPI currently means BYT */
	if (intel_mid_identify_cpu() == INTEL_CPU_CHIP_NOTMID)  {
		set_bit(FEAT_DIV_CTRL, &ssp_ip_features);
		set_bit(FEAT_ISRX_IMRX, &ssp_ip_features);
		pr_info("INFO: I2S DRIVER DIV CTRL/ISRX active - CPU BYT.\n");
	}

	drv_data = devm_kzalloc(&platdev->dev,
				sizeof(struct intel_mid_i2s_hdl), GFP_KERNEL);
	pr_info("SSP I2S ACPI driver : allocated drv_data=%p\n", drv_data);
	if (!drv_data) {
		pr_info("Can't alloc driver data in probe\n");
		status = -ENOMEM;
		goto acpi_probe_err1;
	}

	ret = acpi_bus_get_device(handle, &device);
	if (ret) {
		pr_err("%s: could not get acpi device - %d\n", __func__, ret);
		status = -ENODEV;
		goto acpi_probe_err1;
	}
	if (acpi_bus_get_status(device) || !device->status.present) {
		pr_err("%s: device has invalid status", __func__);
		status = -ENODEV;
		goto acpi_probe_err1;
	}

	hid = acpi_device_hid(device);
	pr_info("SSP I2S driver : ACPI probe for **HID=%s** , drv_data=%p\n",
			hid, drv_data);
	rsrc = platform_get_resource(platdev, IORESOURCE_MEM, 0);
	if (!rsrc) {
		pr_err("Invalid base from IFWI");
		return -EIO;
	}
	pr_info("SSP I2S driver : IFWI start IA =0x%x end=0x%x\n",
			(unsigned int)rsrc->start, (unsigned int)rsrc->end);

	mutex_init(&drv_data->mutex);
	drv_data->ssp_dev = &(platdev->dev);
	/* ioaddr is from the IA point of view */
	drv_data->ioaddr = devm_ioremap_nocache(drv_data->ssp_dev, rsrc->start,
						resource_size(rsrc));
	drv_data->iolen = resource_size(rsrc);

	if (strcmp(hid, SSPACPI5) == 0) {	/* ACPI EDK2  */
		struct acpi_buffer output = { ACPI_ALLOCATE_BUFFER, NULL };
		union acpi_object *obj;

		/* Get USAGE from ACPI table */
		status = i2s_get_ssp_param(handle, intel_ssp_uuid_usage,
						ACPI_TYPE_STRING,
						AARG_NA, AARG_NA, AARG_NA,
						&output);
		if (status)
			goto acpi_probe_err1;
		obj = (union acpi_object *)output.pointer;
		pr_debug("I2S USAGE IS %s\n", obj->string.pointer);

		if (!strcmp(ACPI_MODEM_USAGE, obj->string.pointer)) {
			pr_debug("SSP I2S driver : MODEM\n");
			drv_data->usage = SSP_USAGE_MODEM;
		} else if (!strcmp(ACPI_BLUET_USAGE, obj->string.pointer)) {
			pr_debug("SSP I2S driver : BLUETOOTH\n");
			drv_data->usage = SSP_USAGE_BLUETOOTH_FM;
		} else if (!strcmp(ACPI_CODEC_USAGE, obj->string.pointer)) {
			pr_debug("SSP I2S driver : CODEC\n");
			drv_data->usage = SSP_USAGE_UNASSIGNED;
		}
		kfree(obj);

		/* Get LPEBASE from ACPI table */
		status = i2s_get_ssp_param(handle, intel_ssp_uuid_lpebase,
						ACPI_TYPE_INTEGER,
						AARG_NA, AARG_NA, AARG_NA,
						&output);
		if (status)
			goto acpi_probe_err1;
		obj = (union acpi_object *)output.pointer;

		drv_data->paddr = (dma_addr_t) obj->integer.value;
		pr_debug("SSP I2S driver : LPE BASE %x (expecting 0xFF2AX000)\n",
			 (unsigned int)drv_data->paddr);

		kfree(obj);

		/* Get INSTANCE from ACPI table */
		status = i2s_get_ssp_param(handle, intel_ssp_uuid_instance,
						ACPI_TYPE_INTEGER,
						AARG_NA, AARG_NA, AARG_NA,
						&output);
		if (status)
			goto acpi_probe_err1;
		obj = (union acpi_object *)output.pointer;
		switch (obj->integer.value) {
		case ACPI_SSP0_INSTANCE:
			drv_data->device_instance = SSP0_INSTANCE;
			break;
		case ACPI_SSP1_INSTANCE:
			drv_data->device_instance = SSP1_INSTANCE;
			break;
		case ACPI_SSP2_INSTANCE:
			drv_data->device_instance = SSP2_INSTANCE;
			break;
		default:
			drv_data->device_instance = NONE_INSTANCE;
		}

		pr_debug("I2S INSTANCE is =%d\n", drv_data->device_instance);
		kfree(obj);

		/* Get TIMING for 8Khz from ACPI table */
		status = i2s_get_ssp_param(handle, intel_ssp_uuid_timings,
						ACPI_TYPE_BUFFER,
						AARG_FREQ8, AARG_NA, AARG_NA,
						&output);
		if (status)
			goto acpi_probe_err1;
		obj = (union acpi_object *)output.pointer;
		if (obj->buffer.length != 5) {
			pr_info("I2S ERROR Bad acpi 8khz buffer length\n");
			status = -EINVAL;
			goto acpi_probe_err1;
		}
		drv_data->config_c08k.valid = true;
		drv_data->config_c08k.ssp_psp_T1 = obj->buffer.pointer[0];
		drv_data->config_c08k.ssp_psp_T2 = obj->buffer.pointer[1];
		drv_data->config_c08k.ssp_psp_T4 = obj->buffer.pointer[2];
		drv_data->config_c08k.ssp_psp_T5 = obj->buffer.pointer[3];
		drv_data->config_c08k.ssp_psp_T6 = obj->buffer.pointer[4];
		kfree(obj);

		/* Get TIMING for 16Khz from ACPI table */
		status = i2s_get_ssp_param(handle, intel_ssp_uuid_timings,
						ACPI_TYPE_BUFFER,
						AARG_FREQ16, AARG_NA, AARG_NA,
						&output);
		if (status)
			goto acpi_probe_err1;
		obj = (union acpi_object *)output.pointer;
		if (obj->buffer.length != 5) {
			pr_info("I2S ERROR Bad acpi 16khz buffer length\n");
			status = -EINVAL;
			goto acpi_probe_err1;
		}
		drv_data->config_c16k.valid = true;
		drv_data->config_c16k.ssp_psp_T1 = obj->buffer.pointer[0];
		drv_data->config_c16k.ssp_psp_T2 = obj->buffer.pointer[1];
		drv_data->config_c16k.ssp_psp_T4 = obj->buffer.pointer[2];
		drv_data->config_c16k.ssp_psp_T5 = obj->buffer.pointer[3];
		drv_data->config_c16k.ssp_psp_T6 = obj->buffer.pointer[4];
		kfree(obj);

		/* Get TIMING for 48Khz from ACPI table */
		status = i2s_get_ssp_param(handle, intel_ssp_uuid_timings,
						ACPI_TYPE_BUFFER,
						AARG_FREQ48, AARG_NA, AARG_NA,
						&output);
		if (status)
			goto acpi_probe_err1;
		obj = (union acpi_object *)output.pointer;
		if (obj->buffer.length != 5) {
			pr_info("I2S ERROR Bad acpi 48khz buffer length\n");
			status = -EINVAL;
			goto acpi_probe_err1;
		}
		drv_data->config_c48k.valid = true;
		drv_data->config_c48k.ssp_psp_T1 = obj->buffer.pointer[0];
		drv_data->config_c48k.ssp_psp_T2 = obj->buffer.pointer[1];
		drv_data->config_c48k.ssp_psp_T4 = obj->buffer.pointer[2];
		drv_data->config_c48k.ssp_psp_T5 = obj->buffer.pointer[3];
		drv_data->config_c48k.ssp_psp_T6 = obj->buffer.pointer[4];
		kfree(obj);

		/* calculation of LPESHIM Base address */
		if (drv_data->device_instance == SSP0_INSTANCE)
			base_address_p = rsrc->start - SSP0_OFFSET;
		else if (drv_data->device_instance == SSP1_INSTANCE)
			base_address_p = rsrc->start - SSP1_OFFSET;
		else if (drv_data->device_instance == SSP2_INSTANCE)
			base_address_p = rsrc->start - SSP2_OFFSET;
		else {
			dev_err(drv_data->ssp_dev, "Unknown instance %d for SSP\n",
				drv_data->device_instance);
			status = -ENODEV;
			goto acpi_probe_err1;
		}

		lpeshim_base_address_p = base_address_p + LPESHIM_OFFSET;

		pr_debug("SSP I2S driver:lpeshim=%08x (VLV2=0xdf540000) SSP%d\n",
		(unsigned int)lpeshim_base_address_p, drv_data->device_instance);
		/* remapping addresses */
		lpeshim_base_address = devm_ioremap(drv_data->ssp_dev,
						lpeshim_base_address_p,
						VLV2_LPE_SHIM_REG_SIZE);

	} else { /* keep FDK compatibility */
		/* get the instance of ssp device */
		drv_data->device_instance = NONE_INSTANCE; /*invalid value by default*/
		value = ACPI_NONE_INSTANCE;
		arg_list.count = 1;
		arg_list.pointer = &arg;
		arg.type = ACPI_TYPE_INTEGER;
		arg.integer.value = 0;
		a_status = acpi_evaluate_integer(handle, "INST", &arg_list, &value);
		if (ACPI_FAILURE(a_status))
			pr_info("SSP I2S driver : ACPI ERROR %d no instance\n",
				(int) a_status);

		pr_info("SSP I2S driver : ACPI Instance %lld\n", value);
		if (value == ACPI_SSP0_INSTANCE)
			drv_data->device_instance = SSP0_INSTANCE;
		else if (value == ACPI_SSP1_INSTANCE)
			drv_data->device_instance = SSP1_INSTANCE;
		else if (value == ACPI_SSP2_INSTANCE)
			drv_data->device_instance = SSP2_INSTANCE;

		pr_debug("SSP I2S driver : HID=%s instance=%d\n", hid,
			drv_data->device_instance);
		/* code above will need to be fixed and data in IFWI */

		/* calculation of LPESHIM Base address */
		if (drv_data->device_instance == SSP0_INSTANCE)
			base_address_p = rsrc->start - SSP0_OFFSET;
		else if (drv_data->device_instance == SSP1_INSTANCE)
			base_address_p = rsrc->start - SSP1_OFFSET;
		else if (drv_data->device_instance == SSP2_INSTANCE)
			base_address_p = rsrc->start - SSP2_OFFSET;
		else {
			dev_err(drv_data->ssp_dev, "Unknown instance %d for SSP\n",
				drv_data->device_instance);
			status = -ENODEV;
			goto acpi_probe_err1;
		}

		lpeshim_base_address_p = base_address_p + LPESHIM_OFFSET;

		pr_debug("SSP I2S driver:lpeshim=%08x (VLV2=0xdf540000) SSP%d\n",
		(unsigned int)lpeshim_base_address_p, drv_data->device_instance);
		/* remapping addresses */
		lpeshim_base_address = devm_ioremap(drv_data->ssp_dev,
						lpeshim_base_address_p,
						VLV2_LPE_SHIM_REG_SIZE);

		/* get the SSP IP base address from LPE point of view */
		arg_list.count = 1;
		arg_list.pointer = &arg;
		arg.type = ACPI_TYPE_INTEGER;
		arg.integer.value = 0;
		a_status = acpi_evaluate_integer(handle, "LBAS", &arg_list, &value);
		if (ACPI_FAILURE(a_status)) {
			pr_info("SSP I2S driver : ACPI ERROR %d value=%llx\n",
				 (int) a_status, value);
			status = -ENODEV;
			goto acpi_probe_err1;
		}

		drv_data->paddr = (dma_addr_t) value;
		pr_debug("SSP I2S driver : LPE BASE %x (expecting 0xFF2AX000)\n",
			 (unsigned int)drv_data->paddr);

		/* Get the configuration of each frequency */
		/* 8Khz, 16Khz and 48Khz */
		drv_data->config_c08k.valid = false;
		drv_data->config_c16k.valid = false;
		drv_data->config_c48k.valid = false;
		retrieve_ssp_config(handle, &(drv_data->config_c08k), CONFIG_C08K);
		retrieve_ssp_config(handle, &(drv_data->config_c16k), CONFIG_C16K);
		retrieve_ssp_config(handle, &(drv_data->config_c48k), CONFIG_C48K);

		/* unmasking the SSPx IRQ to IA */
		if (drv_data->device_instance == 0)
			clear_LPE_IMRX_reg(lpeshim_base_address, IAPIS_SSP0)
		else if (drv_data->device_instance == 1)
			clear_LPE_IMRX_reg(lpeshim_base_address, IAPIS_SSP1)
		else if (drv_data->device_instance == 2)
			clear_LPE_IMRX_reg(lpeshim_base_address, IAPIS_SSP2)

		/* TODO BZ154289:currently will get info from dsdt acpi table */
		if (strcmp(hid, SSPMODEM) == 0)
			drv_data->usage = SSP_USAGE_MODEM;
		else if (strcmp(hid, SSPBLUET) == 0)
			drv_data->usage = SSP_USAGE_BLUETOOTH_FM;
		else
			drv_data->usage = SSP_USAGE_UNASSIGNED;

		pr_debug("SSP I2S driver : Probe for HID=%s usage is %d\n",
				hid, drv_data->usage);
	}  /* endif EDK / FDK */

	/* Get DMA enumeration UEFI */
	drv_data->dmacdev = intel_mid_get_acpi_dma(ACPI_DMA_EDK);
	/* if no DMA of UEFI, use FDK name */
	if (drv_data->dmacdev == NULL)
		drv_data->dmacdev = intel_mid_get_acpi_dma(ACPI_DMA_FDK);
	if (drv_data->dmacdev == NULL) {
		WARN(drv_data->dmacdev == NULL,
		"SSP I2S Driver : unable to get DMA of SSP Device\n");
		status = -ENODEV;
		goto acpi_probe_err1;
	}

	pr_debug("SSP I2S driver : dmac1 dmadev found = %p\n",
			drv_data->dmacdev);

	drv_data->irq = platform_get_irq(platdev, 0);
	pr_debug("I2S irq from platdev is:%d\n", drv_data->irq);
	ret = devm_request_threaded_irq(drv_data->ssp_dev, drv_data->irq,
					i2s_irq, i2s_irq_deferred, IRQF_SHARED,
					"i2s ssp", drv_data);
	if (ret)
		return ret;
	pr_debug("I2S Registered IRQ %#x\n", drv_data->irq);

	set_bit(FEAT_ACPI, &ssp_ip_features);
	WARN(test_bit(FEAT_PCI, &ssp_ip_features),
			"ACPI and PCI for SSP at the same time");

	/*
	 * Create the WL1273 BT/FM ASoC platform devices
	 */
	pr_debug("I2S ALLOCATE FOR ASOC ACPI VERSION\n");

	if (drv_data->usage == SSP_USAGE_MODEM)
		WARN(test_and_set_bit(MODEM_USAGE_FND, &ssps_found),
					"SSP for modem usage already probed");

	if (drv_data->usage == SSP_USAGE_BLUETOOTH_FM)
		WARN(test_and_set_bit(BT_USAGE_FND, &ssps_found),
					"SSP for BT/FM usage already probed");

	/* mount the asoc card only on BT I2S as Modem is optional */
	if ((test_bit(BT_USAGE_FND, &ssps_found)) &&
		(drv_data->usage == SSP_USAGE_BLUETOOTH_FM)) {
		/*
		 * MID SSP CPU DAI
		 */
		asoc_pdev = platform_device_alloc("mid-ssp-dai", -1);
		if (!asoc_pdev) {
			dev_err(drv_data->ssp_dev,
				"platform mid-ssp-dai allocation failed\n");
			status = -ENODEV;
			goto acpi_probe_err1;
		}
		ret = platform_device_add(asoc_pdev);
		if (ret) {
			dev_err(drv_data->ssp_dev,
					"platform mid-ssp-dai add failed\n");
			platform_device_put(asoc_pdev);
		}
		pr_debug("I2S: platform mid-ssp-dai allocated\n");
	}

	/* runtime */

	pm_runtime_enable(drv_data->ssp_dev);

	dev_set_drvdata(drv_data->ssp_dev, drv_data);

	return status;

acpi_probe_err1:
	pr_info("SSP I2S driver : ACPI probe END\n");
	return status;

#endif

}

/**
 * intel_mid_i2s_init - register pci driver
 *
 */
static int __init intel_mid_i2s_init(void)
{
	int ret = 0;
	clear_bit(MODEM_FND, &modem_found_and_i2s_setup_ok);
	clear_bit(MODEM_USAGE_FND, &ssps_found);

	clear_bit(BT_USAGE_FND, &ssps_found);

	clear_bit(FEAT_ACPI, &ssp_ip_features);
	clear_bit(FEAT_PCI, &ssp_ip_features);
	clear_bit(FEAT_DIV_CTRL, &ssp_ip_features);
	clear_bit(FEAT_ISRX_IMRX, &ssp_ip_features);

	pr_info("INFO: I2S DRIVER loading... ver: %s cpu=%d\n", VERSION_SSP,
						intel_mid_identify_cpu());

	ret = pci_register_driver(&intel_mid_i2s_driver);
	if (ret)
		pr_err("I2S driver PCI register failed\n");
#ifdef CONFIG_ACPI
	else {
		ret = platform_driver_register(&i2s_acpi_driver);
		if (ret)
			pr_err("I2S driver ACPI registerfailed, PCI only\n");
		else
			pr_info("INFO: I2S DRIVER init OK\n");
	}
#endif /* CONFIG_ACPI */

	if ((intel_mid_identify_cpu() == INTEL_MID_CPU_CHIP_VALLEYVIEW2) ||
	    (intel_mid_identify_cpu() == INTEL_MID_CPU_CHIP_ANNIEDALE)) {
		set_bit(FEAT_DIV_CTRL, &ssp_ip_features);
		set_bit(FEAT_ISRX_IMRX, &ssp_ip_features);
		pr_info("INFO: I2S DRIVER DIV CTRL and ISRX active\n");
	}

	return ret;
}

static void __exit intel_mid_i2s_exit(void)
{
	pci_unregister_driver(&intel_mid_i2s_driver);
#ifdef CONFIG_ACPI
	platform_driver_unregister(&i2s_acpi_driver);
#endif /* CONFIG_ACPI */
	pr_debug("I2S driver unloaded\n");
}

module_init(intel_mid_i2s_init);
module_exit(intel_mid_i2s_exit);
