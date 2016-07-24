/*
 *  psh.c - Merrifield PSH IA side driver
 *
 *  (C) Copyright 2012 Intel Corporation
 *  Author: Alek Du <alek.du@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License v2 as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA	02110-1301, USA
 */

/*
 * PSH IA side driver for Merrifield Platform
 */

//#define VPROG2_SENSOR

#include <linux/device.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/pci.h>
#include <linux/circ_buf.h>
#include <linux/completion.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <asm/intel_psh_ipc.h>
#include <linux/string.h>
#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/pm_runtime.h>
#include <linux/intel_mid_pm.h>
#include "psh_ia_common.h"
#include <asm/intel_mid_rpmsg.h>
#include <asm/intel-mid.h>
#include <linux/kct.h>

#include <linux/fs.h> 
#include <asm/segment.h>
#include <asm/uaccess.h>
#include <linux/buffer_head.h>
#ifdef VPROG2_SENSOR
#include <asm/intel_scu_ipcutil.h>
#endif

#define APP_IMR_SIZE (1024 * 256)

// Add for reading project ID
#define PROJECT_NODE_PATH "/sys/module/intel_mid_sfi/parameters/project_id"

static bool disable_psh_recovery = false;
static bool force_psh_recovery = false;

module_param(disable_psh_recovery, bool, S_IRUGO);
module_param(force_psh_recovery, bool, S_IRUGO | S_IWUSR);

enum {
	imr_allocate = 0,
	imr_pci_shim = 1,
};

struct psh_plt_priv {
	int imr_src;
	struct device *hwmon_dev;
	struct device *dev;
	void *imr2;		/* IMR2 */
	void *ddr;		/* IMR3 */
	uintptr_t imr2_phy;
	uintptr_t ddr_phy;
	struct loop_buffer lbuf;
};



static int psh_recovery = 0;

// Add for reading project ID
// ZE550ML : 0x17
// ZE551ML : 0x1F
// ZR550ML : 0x1C
// ZX550ML : 0x1B
long getProjectId(void)
{
	struct file *fp = NULL;
	char buffer[256] = {0};
	mm_segment_t oldfs;
	long project_id;
	int ret;

	oldfs = get_fs();
	set_fs(get_ds());
	
    fp = filp_open(PROJECT_NODE_PATH, O_RDONLY, 0);
    if (IS_ERR(fp)) {
        psh_err("Open project node fail!!!!\n");
        return -1;
    }
	
    ret = vfs_read(fp, buffer, sizeof(buffer)-1, &(fp->f_pos));
	set_fs(oldfs);	
    filp_close(fp,NULL);
    if(ret < 0) {
        psh_err("fail to read PROJECT_NODE_PATH, ret=%d !\n", ret);
        return -1;
    }

    ret = strict_strtol(buffer,10,&project_id);
    psh_err("The project id is = 0x%x\n", project_id);
	
    return project_id;

}

int do_psh_recovery(struct psh_ia_priv *psh_ia_data)
{
	struct psh_plt_priv *plt_priv =
			(struct psh_plt_priv *)psh_ia_data->platform_priv;
	int ret = 0;

	psh_err("PSH Recovery started, please wait ... \n");

	psh_recovery = 1;
	/* set to D0 state */
	pm_runtime_get_sync(plt_priv->dev);

	/* set recovery IPC cmd to SCU */
	ret = rpmsg_send_generic_simple_command(0xA2, 0);
	if (ret) {
		psh_err("failed to send recovery cmd to SCU, ret=%d\n", ret);
		goto f_out;
	}
	/* maybe may reduce the wait time */
	msleep(2000);

	ret = do_setup_ddr(plt_priv->dev);
	if (ret) {
		psh_err("failed to setup ddr during psh recovery\n");
		goto f_out;
	}
	else {
		psh_err("PSH recovery done \n");
	}


f_out:
	/* Report recovery event in crashtool*/
	/* Replace CT_ADDITIONAL_APLOG with
	CT_ADDITIONAL_FWMSG|CT_ADDITIONAL_APLOG once online log is working */

	kct_log(CT_EV_CRASH, "PSH", "RECOVERY", 0, "", "", "", "", "", "",
		"", CT_ADDITIONAL_FWMSG | CT_ADDITIONAL_APLOG);

	pm_runtime_put(plt_priv->dev);
	psh_recovery = 0;

	return ret;
}


int recovery_send_cmd(struct psh_ia_priv *psh_ia_data,
                        struct ia_cmd *cmd, int len)
{
	int ret = 0;
	static struct resp_cmd_ack cmd_ack;

	cmd_ack.cmd_id = cmd->cmd_id;
	psh_ia_data->cmd_ack = &cmd_ack;

	psh_ia_data->cmd_in_progress = cmd->cmd_id;
	ret = process_send_cmd(psh_ia_data, PSH2IA_CHANNEL0,
			cmd, len);
	psh_ia_data->cmd_in_progress = CMD_INVALID;
	if (ret) {
		psh_err("send cmd (id = %d) failed, ret=%d\n",
			cmd->cmd_id, ret);
		goto f_out;
	}
	if (cmd->cmd_id == CMD_FW_UPDATE)
		goto f_out;

ack_wait:
	if (!wait_for_completion_timeout(&psh_ia_data->cmd_comp, 5 * HZ)) {
		psh_err("no CMD_ACK for %d back, timeout!\n",
			cmd_ack.cmd_id);
		ret = -ETIMEDOUT;
	} else if (cmd_ack.ret) {
		if (cmd_ack.ret == E_CMD_ASYNC)
			goto ack_wait;
		psh_err("CMD %d return error %d!\n", cmd_ack.cmd_id,
				cmd_ack.ret);
		ret = -EREMOTEIO;
	}

f_out:
	psh_ia_data->cmd_ack = NULL;
	return ret;
}

int process_send_cmd(struct psh_ia_priv *psh_ia_data,
			int ch, struct ia_cmd *cmd, int len)
{
	int i, j;
	int ret = 0;
	u8 *pcmd = (u8 *)cmd;
	struct psh_msg in;

	if (ch == PSH2IA_CHANNEL0 && cmd->cmd_id == CMD_RESET) {
		intel_psh_ipc_disable_irq();
		ia_lbuf_read_reset(psh_ia_data->lbuf);
	}

	/* map from virtual channel to real channel */
	ch = ch - PSH2IA_CHANNEL0 + PSH_SEND_CH0;

	for (i = 0; i < len; i += 7) {
		u8 left = len - i;
		u8 *ptr = (u8 *)&in;

		memset(&in, 0, sizeof(in));

		if (left > 7) {
			left = 7;
			in.msg |= PSH_IPC_CONTINUE;
		}

		for (j = 0; j < left; j++) {
			if (j == 3)
				ptr++;
			*ptr = *pcmd;
			ptr++;
			pcmd++;
		}

		ret = intel_ia2psh_command(&in, NULL, ch, 1000000);
		if (ret) {
			psh_err("sendcmd %d by IPC %d failed!, ret=%d\n",
					cmd->cmd_id, ch, ret);
			ret = -EIO;
			goto f_out;
		}
	}

f_out:
	if (ch == PSH2IA_CHANNEL0 && cmd->cmd_id == CMD_RESET)
		intel_psh_ipc_enable_irq();

	
	if (!disable_psh_recovery) {
		if ((ret && (psh_recovery == 0)) || (force_psh_recovery))
		{
			if (force_psh_recovery) {
				psh_err("forced PSH recovery\n");
				force_psh_recovery = 0;
			}
			if (do_psh_recovery(psh_ia_data))
				psh_err("PSH recovery failed\n");
		}
	}

	return ret;
}

extern struct soft_platform_id spid;
int do_setup_ddr(struct device *dev)
{
	struct psh_ia_priv *ia_data =
			(struct psh_ia_priv *)dev_get_drvdata(dev);
	struct psh_plt_priv *plt_priv =
			(struct psh_plt_priv *)ia_data->platform_priv;
	uintptr_t ddr_phy = plt_priv->ddr_phy;
	uintptr_t imr2_phy = plt_priv->imr2_phy;
	const struct firmware *fw_entry;
	struct ia_cmd cmd_user = {
		.cmd_id = CMD_SETUP_DDR,
		.sensor_id = 0,
		};
	static int fw_load_done;
	int load_default = 0;
	char fname[40];

	if ((fw_load_done)&&(psh_recovery == 0))
		return 0;

#ifdef VPROG2_SENSOR
	if (getProjectId() != 0x1c && getProjectId() != 0x1d)
	{
		intel_scu_ipc_msic_vprog2(1);
		msleep(500);
	}
#endif

	if (getProjectId() == 0x17)
		snprintf(fname, 40, "psh.bin.ze550ml");
	else if (getProjectId() == 0x1F)
		snprintf(fname, 40, "psh.bin.ze551ml");
	else if (getProjectId() == 0x1B)
		snprintf(fname, 40, "psh.bin.zx550ml");
	else
		snprintf(fname, 40, "psh.bin");
/*
	snprintf(fname, 40, "psh.bin.%04x.%04x.%04x.%04x.%04x.%04x",
				(int)spid.customer_id,
				(int)spid.vendor_id,
				(int)spid.manufacturer_id,
				(int)spid.platform_family_id,
				(int)spid.product_line_id,
				(int)spid.hardware_id);
*/
again:
	if (!request_firmware(&fw_entry, fname, dev)) {
		if (!fw_entry)
			return -ENOMEM;

		psh_debug("psh fw size %d virt:0x%p\n",
				(int)fw_entry->size, fw_entry->data);
		if (fw_entry->size > APP_IMR_SIZE) {
			psh_err("psh fw size too big\n");
		} else {
			struct ia_cmd cmd = {
				.cmd_id = CMD_RESET,
				.sensor_id = 0,
				};

			memcpy(plt_priv->imr2, fw_entry->data,
				fw_entry->size);
			*(uintptr_t *)(&cmd.param) = imr2_phy;
			cmd.tran_id = 0x1;
			if (process_send_cmd(ia_data, PSH2IA_CHANNEL3, &cmd, 7)) {
				release_firmware(fw_entry);
				return -1;
			}
			ia_data->load_in_progress = 1;
			wait_for_completion_timeout(&ia_data->cmd_load_comp,
					3 * HZ);
			fw_load_done = 1;
		}
		release_firmware(fw_entry);
	} else {
		psh_err("cannot find psh firmware(%s)\n", fname);
		if (!load_default) {
			psh_err("try to load default psh.bin\n");
			snprintf(fname, 20, "psh.bin");
			load_default = 1;
			goto again;
		}
	}
	ia_lbuf_read_reset(ia_data->lbuf);
	*(unsigned long *)(&cmd_user.param) = ddr_phy;
	return psh_recovery?recovery_send_cmd(ia_data, &cmd_user, 7):ia_send_cmd(ia_data, &cmd_user, 7);
}

static void psh2ia_channel_handle(u32 msg, u32 param, void *data)
{
	struct pci_dev *pdev = (struct pci_dev *)data;
	struct psh_ia_priv *ia_data =
			(struct psh_ia_priv *)dev_get_drvdata(&pdev->dev);
	struct psh_plt_priv *plt_priv =
			(struct psh_plt_priv *)ia_data->platform_priv;
	u8 *dbuf = NULL;
	u16 size = 0;

	if (unlikely(ia_data->load_in_progress)) {
		ia_data->load_in_progress = 0;
		complete(&ia_data->cmd_load_comp);
		return;
	}

	while (!ia_lbuf_read_next(ia_data,
			&plt_priv->lbuf, &dbuf, &size)) {
		ia_handle_frame(ia_data, dbuf, size);
	}
	sysfs_notify(&pdev->dev.kobj, NULL, "data_size");
}

static int psh_imr_init(struct pci_dev *pdev,
			int imr_src, uintptr_t *phy_addr, void **virt_addr,
			unsigned size, int bar)
{
	struct page *pg;
	void __iomem *mem;
	int ret = 0;
	unsigned long start = 0, len;

	if (imr_src == imr_allocate) {
		/* dynamic alloct memory region */
		pg = alloc_pages(GFP_KERNEL | GFP_DMA32 | __GFP_ZERO,
						get_order(size));
		if (!pg) {
			dev_err(&pdev->dev, "can not allocate app page imr buffer\n");
			ret = -ENOMEM;
			goto err;
		}
		*phy_addr = page_to_phys(pg);
		*virt_addr = page_address(pg);
	} else if (imr_src == imr_pci_shim) {
		/* dedicate isolated memory region */
		start = pci_resource_start(pdev, bar);
		len = pci_resource_len(pdev, bar);
		if (!start || !len) {
			dev_err(&pdev->dev, "bar %d address not set\n", bar);
			ret = -EINVAL;
			goto err;
		}

		ret = pci_request_region(pdev, bar, "psh");
		if (ret) {
			dev_err(&pdev->dev, "failed to request psh region "
				"0x%lx-0x%lx\n", start,
				(unsigned long)pci_resource_end(pdev, bar));
			goto err;
		}

		mem = ioremap_nocache(start, len);
		if (!mem) {
			dev_err(&pdev->dev, "can not ioremap app imr address\n");
			ret = -EINVAL;
			goto err_ioremap;
		}

		*phy_addr = start;
		*virt_addr = (void *)mem;
	} else {
		dev_err(&pdev->dev, "Invalid chip imr source\n");
		ret = -EINVAL;
		goto err;
	}

	return 0;

err_ioremap:
	pci_release_region(pdev, bar);
err:
	return ret;
}

static void psh_imr_free(int imr_src, void *virt_addr, unsigned size)
{
	if (imr_src == imr_allocate)
		__free_pages(virt_to_page(virt_addr), get_order(size));
	else if (imr_src == imr_pci_shim)
		iounmap((void __iomem *)virt_addr);
}

static int psh_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	int ret = -1;
	struct psh_ia_priv *ia_data;
	struct psh_plt_priv *plt_priv;

	ret = pci_enable_device(pdev);
	if (ret) {
		dev_err(&pdev->dev, "fail to enable psh pci device\n");
		goto pci_err;
	}

	plt_priv = kzalloc(sizeof(*plt_priv), GFP_KERNEL);
	if (!plt_priv) {
		dev_err(&pdev->dev, "can not allocate plt_priv\n");
		goto plt_err;
	}

	switch (intel_mid_identify_cpu()) {
	case INTEL_MID_CPU_CHIP_TANGIER:
		if (intel_mid_soc_stepping() == 0)
			plt_priv->imr_src = imr_allocate;
		else
			plt_priv->imr_src = imr_pci_shim;
		break;
	case INTEL_MID_CPU_CHIP_ANNIEDALE:
		plt_priv->imr_src = imr_pci_shim;
		break;
	default:
		dev_err(&pdev->dev, "error memory region\n");
		goto psh_imr2_err;
		break;
	}

	/* init IMR2 */
	ret = psh_imr_init(pdev, plt_priv->imr_src,
				&plt_priv->imr2_phy, &plt_priv->imr2,
				APP_IMR_SIZE, 0);
	if (ret)
		goto psh_imr2_err;


	/* init IMR3 */
	ret = psh_imr_init(pdev, plt_priv->imr_src,
				&plt_priv->ddr_phy, &plt_priv->ddr,
				BUF_IA_DDR_SIZE, 1);
	if (ret)
		goto psh_ddr_err;

	ret = psh_ia_common_init(&pdev->dev, &ia_data);
	if (ret) {
		dev_err(&pdev->dev, "fail to init psh_ia_common\n");
		goto psh_ia_err;
	}

	ia_lbuf_read_init(&plt_priv->lbuf,
				plt_priv->ddr,
				BUF_IA_DDR_SIZE, NULL);
	ia_data->lbuf = &plt_priv->lbuf;

	plt_priv->hwmon_dev = hwmon_device_register(&pdev->dev);
	if (!plt_priv->hwmon_dev) {
		dev_err(&pdev->dev, "fail to register hwmon device\n");
		goto hwmon_err;
	}
	plt_priv->dev = &pdev->dev;
	ia_data->platform_priv = plt_priv;

	ret = intel_psh_ipc_bind(PSH_RECV_CH0, psh2ia_channel_handle, pdev);
	if (ret) {
		dev_err(&pdev->dev, "fail to bind channel\n");
		goto irq_err;
	}

	/* just put this dev into suspend status always, since this is fake */
	pm_runtime_put_noidle(&pdev->dev);
	pm_runtime_allow(&pdev->dev);
	return 0;

irq_err:
	hwmon_device_unregister(plt_priv->hwmon_dev);
hwmon_err:
	psh_ia_common_deinit(&pdev->dev);
psh_ia_err:
	psh_imr_free(plt_priv->imr_src, plt_priv->ddr, BUF_IA_DDR_SIZE);
psh_ddr_err:
	psh_imr_free(plt_priv->imr_src, plt_priv->imr2, APP_IMR_SIZE);
psh_imr2_err:
	kfree(plt_priv);
plt_err:
	pci_dev_put(pdev);
pci_err:
	return ret;
}

static void psh_remove(struct pci_dev *pdev)
{
	struct psh_ia_priv *ia_data =
			(struct psh_ia_priv *)dev_get_drvdata(&pdev->dev);
	struct psh_plt_priv *plt_priv =
			(struct psh_plt_priv *)ia_data->platform_priv;

	psh_imr_free(plt_priv->imr_src, plt_priv->ddr, BUF_IA_DDR_SIZE);
	psh_imr_free(plt_priv->imr_src, plt_priv->imr2, APP_IMR_SIZE);

	intel_psh_ipc_unbind(PSH_RECV_CH0);

	hwmon_device_unregister(plt_priv->hwmon_dev);

	kfree(plt_priv);

	psh_ia_common_deinit(&pdev->dev);
}

static int psh_suspend(struct device *dev)
{
	return psh_ia_comm_suspend(dev);
}

static int psh_resume(struct device *dev)
{
	return psh_ia_comm_resume(dev);
}

static int psh_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "runtime suspend called\n");
	return 0;
}

static int psh_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "runtime resume called\n");
	return 0;
}

static const struct dev_pm_ops psh_drv_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(psh_suspend,
			psh_resume)
	SET_RUNTIME_PM_OPS(psh_runtime_suspend,
			psh_runtime_resume, NULL)
};

static DEFINE_PCI_DEVICE_TABLE(pci_ids) = {
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x11a4)},
	{ 0,}
};

MODULE_DEVICE_TABLE(pci, pci_ids);

static struct pci_driver psh_driver = {
	.name = "psh",
	.driver = {
		.pm = &psh_drv_pm_ops,
	},
	.id_table = pci_ids,
	.probe	= psh_probe,
	.remove	= psh_remove,
};

static int __init psh_init(void)
{
	return pci_register_driver(&psh_driver);
}

static void __exit psh_exit(void)
{
	pci_unregister_driver(&psh_driver);
}

module_init(psh_init);
module_exit(psh_exit);
MODULE_LICENSE("GPL v2");
