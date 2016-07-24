/*
 *  sst_pvt.c - Intel SST Driver for audio engine
 *
 *  Copyright (C) 2008-10	Intel Corp
 *  Authors:	Vinod Koul <vinod.koul@intel.com>
 *		Harsha Priya <priya.harsha@intel.com>
 *		Dharageswari R <dharageswari.r@intel.com>
 *		KP Jeeja <jeeja.kp@intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This file contains all private functions
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kobject.h>
#include <linux/pci.h>
#include <linux/fs.h>
#include <linux/firmware.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <sound/asound.h>
#include <sound/pcm.h>
#include <sound/compress_offload.h>
#include "../sst_platform.h"
#include "../platform_ipc_v2.h"
#include "sst.h"

#define SST_EXCE_DUMP_BASE	0xFFFF2c00
#define SST_EXCE_DUMP_WORD	4
#define SST_EXCE_DUMP_LEN	32
#define SST_EXCE_DUMP_SIZE	((SST_EXCE_DUMP_LEN)*(SST_EXCE_DUMP_WORD))
#define SST_EXCE_DUMP_OFFSET	0xA00

#define SHIM_DUMP_BUF_SIZE	1024

/* Table for shim register offset and name */
static struct sst_shim_reg_table shim_reg_table[] = {
	{.offset = SST_CSR,		.name = "CSR"},
	{.offset = SST_PISR,		.name = "PISR"},
	{.offset = SST_PIMR,		.name = "PIMR"},
	{.offset = SST_ISRX,		.name = "ISRX"},
	{.offset = SST_ISRD,		.name = "ISRD"},
	{.offset = SST_IMRX,		.name = "IMRX"},
	{.offset = SST_IMRD,		.name = "IMRD"},
	{.offset = SST_IPCX,		.name = "IPCX"},
	{.offset = SST_IPCD,		.name = "IPCD"},
	{.offset = SST_ISRSC,		.name = "ISRSC"},
	{.offset = SST_ISRLPESC,	.name = "ISRLPESC"},
	{.offset = SST_IMRSC,		.name = "IMRSC"},
	{.offset = SST_IMRLPESC,	.name = "IMRLPESC"},
	{.offset = SST_IPCSC,		.name = "IPCSC"},
	{.offset = SST_IPCLPESC,	.name = "IPCLPESC"},
	{.offset = SST_CLKCTL,		.name = "CLKCTL"},
	{.offset = SST_CSR2,		.name = "CSR2"},
	{.offset = SST_TMRSTAT,		.name = "SST_TMRSTAT"},
};

char *sst_get_shim_buf(struct intel_sst_drv *sst_drv_ctx)
{
	char *buf;
	int i;
	int pos = 0;
	unsigned long long val = 0;

	/* This is to hold shim register values, offset and name */
	buf = kzalloc(SHIM_DUMP_BUF_SIZE, GFP_ATOMIC);
	if (!buf)
		return buf;
	for (i = 0; i < ARRAY_SIZE(shim_reg_table); i++) {
		val = sst_shim_read64(sst_drv_ctx->shim, shim_reg_table[i].offset);
		pos += sprintf(buf + pos, "0x%.2x: %.8llx\t\t\t  %s\n",
			 shim_reg_table[i].offset, val, shim_reg_table[i].name);
	}
	return buf;
}

/*
 * sst_wait_interruptible - wait on event
 *
 * @sst_drv_ctx: Driver context
 * @block: Driver block to wait on
 *
 * This function waits without a timeout (and is interruptable) for a
 * given block event
 */
int sst_wait_interruptible(struct intel_sst_drv *sst_drv_ctx,
				struct sst_block *block)
{
	int retval = 0;

	if (!wait_event_interruptible(sst_drv_ctx->wait_queue,
				block->condition)) {
		/* event wake */
		if (block->ret_code < 0) {
			pr_err("stream failed %d\n", block->ret_code);
			retval = -EBUSY;
		} else {
			pr_debug("event up\n");
			retval = 0;
		}
	} else {
		pr_err("signal interrupted\n");
		retval = -EINTR;
	}
	return retval;

}

unsigned long long read_shim_data(struct intel_sst_drv *sst, int addr)
{
	unsigned long long val = 0;

	switch (sst->pci_id) {
	case SST_CLV_PCI_ID:
		val = sst_shim_read(sst->shim, addr);
		break;
	case SST_MRFLD_PCI_ID:
	case PCI_DEVICE_ID_INTEL_SST_MOOR:
	case SST_BYT_PCI_ID:
		val = sst_shim_read64(sst->shim, addr);
		break;
	}
	return val;
}

void write_shim_data(struct intel_sst_drv *sst, int addr,
				unsigned long long data)
{
	switch (sst->pci_id) {
	case SST_CLV_PCI_ID:
		sst_shim_write(sst->shim, addr, (u32) data);
		break;
	case SST_MRFLD_PCI_ID:
	case PCI_DEVICE_ID_INTEL_SST_MOOR:
	case SST_BYT_PCI_ID:
		sst_shim_write64(sst->shim, addr, (u64) data);
		break;
	}
}


void dump_sst_shim(struct intel_sst_drv *sst)
{
	unsigned long irq_flags;
	char *buf;

	spin_lock_irqsave(&sst->ipc_spin_lock, irq_flags);
	buf = sst_get_shim_buf(sst);
	if (!buf) {
		pr_err("Unable to allocate shim buffer\n");
		return;
	}
	pr_err("%s\n", buf);
	kfree(buf);
	spin_unlock_irqrestore(&sst->ipc_spin_lock, irq_flags);
}

void reset_sst_shim(struct intel_sst_drv *sst)
{
	union config_status_reg_mrfld csr;

	pr_err("Resetting few Shim registers\n");
	write_shim_data(sst, sst->ipc_reg.ipcx, 0x0);
	write_shim_data(sst, sst->ipc_reg.ipcd, 0x0);
	write_shim_data(sst, SST_ISRX, 0x0);
	write_shim_data(sst, SST_ISRD, 0x0);
	write_shim_data(sst, SST_IPCSC, 0x0);
	write_shim_data(sst, SST_IPCLPESC, 0x0);
	write_shim_data(sst, SST_ISRSC, 0x0);
	write_shim_data(sst, SST_ISRLPESC, 0x0);
	write_shim_data(sst, SST_PISR, 0x0);

	/* Reset the CSR value to the default value. i.e 0x1e40001*/
	csr.full = sst_shim_read64(sst_drv_ctx->shim, SST_CSR);
	csr.part.xt_snoop = 0;
	csr.full &= ~(0xf);
	csr.full |= 0x01;
	sst_shim_write64(sst_drv_ctx->shim, SST_CSR, csr.full);
}

#define SST_DUMP_LPE_STACK_SIZE 600
static void dump_lpe_stack(struct intel_sst_drv *sst)
{
	u32 dump_word;
	u32 i;
	void __iomem *addr;

	addr = sst->dram + SST_LPE_STACK_OFFSET;
	pr_err("Dump LPE stack area begin@ %p\n:", (u32 *)addr);
	for (i = 0; i < (SST_DUMP_LPE_STACK_SIZE/(sizeof(u32))); i++) {
		dump_word = readl(addr + i * (sizeof(u32)));
		pr_err("Data[%d]=%#x\n", i, dump_word);
	}
	pr_err("Firmware exception dump ends\n");
}

static void dump_sst_crash_area(void)
{
	void __iomem *fw_dump_area;
	u32 dump_word;
	u8 i;

	/* dump the firmware SRAM where the exception details are stored */
	fw_dump_area = ioremap_nocache(SST_EXCE_DUMP_BASE, SST_EXCE_DUMP_SIZE);

	pr_err("Firmware exception dump begins:\n");
	pr_err("Exception start signature:%#x\n", readl(fw_dump_area + SST_EXCE_DUMP_WORD));
	pr_err("EXCCAUSE:\t\t\t%#x\n", readl(fw_dump_area + SST_EXCE_DUMP_WORD*2));
	pr_err("EXCVADDR:\t\t\t%#x\n", readl(fw_dump_area + (SST_EXCE_DUMP_WORD*3)));
	pr_err("Firmware additional data:\n");

	/* dump remaining FW debug data */
	for (i = 1; i < (SST_EXCE_DUMP_LEN-4+1); i++) {
		dump_word = readl(fw_dump_area + (SST_EXCE_DUMP_WORD*3)
						+ (i*SST_EXCE_DUMP_WORD));
		pr_err("Data[%d]=%#x\n", i, dump_word);
	}
	iounmap(fw_dump_area);
	pr_err("Firmware exception dump ends\n");
}

/**
 * dump_ram_area - dumps the iram/dram into a local buff
 *
 * @sst			: pointer to driver context
 * @recovery		: pointer to the struct containing buffers
 * @iram		: true if iram dump else false
 * This function dumps the iram dram data into the respective buffers
 */
static void dump_ram_area(struct intel_sst_drv *sst,
			struct sst_dump_buf *dump_buf, enum sst_ram_type type)
{
	if (type == SST_IRAM) {
		pr_err("Iram dumped in buffer\n");
		memcpy_fromio(dump_buf->iram_buf.buf, sst->iram,
				dump_buf->iram_buf.size);
	} else {
		pr_err("Dram dumped in buffer\n");
		memcpy_fromio(dump_buf->dram_buf.buf, sst->dram,
				dump_buf->dram_buf.size);
	}
}

/*FIXME Disabling IRAM/DRAM dump for timeout issues */
static void sst_stream_recovery(struct intel_sst_drv *sst)
{
	struct stream_info *str_info;
	u8 i;
	for (i = 1; i <= sst->info.max_streams; i++) {
		pr_err("Audio: Stream %d, state %d\n", i, sst->streams[i].status);
		if (sst->streams[i].status != STREAM_UN_INIT) {
			str_info = &sst_drv_ctx->streams[i];
			if (str_info->pcm_substream)
				snd_pcm_stop(str_info->pcm_substream, SNDRV_PCM_STATE_SETUP);
			else if (str_info->compr_cb_param)
				snd_compr_stop(str_info->compr_cb_param);
			sst->streams[i].status = STREAM_RESET;
		}
	}
}

static void sst_dump_ipc_dispatch_lists(struct intel_sst_drv *sst)
{
	struct ipc_post *m, *_m;
	unsigned long irq_flags;

	spin_lock_irqsave(&sst->ipc_spin_lock, irq_flags);
	if (list_empty(&sst->ipc_dispatch_list))
		pr_err("ipc dispatch list is Empty\n");

	list_for_each_entry_safe(m, _m, &sst->ipc_dispatch_list, node) {
		pr_err("ipc-dispatch:pending msg header %#x\n", m->header.full);
		list_del(&m->node);
		kfree(m->mailbox_data);
		kfree(m);
	}
	spin_unlock_irqrestore(&sst->ipc_spin_lock, irq_flags);
}

static void sst_dump_rx_lists(struct intel_sst_drv *sst)
{
	struct ipc_post *m, *_m;
	unsigned long irq_flags;

	spin_lock_irqsave(&sst->rx_msg_lock, irq_flags);
	if (list_empty(&sst->rx_list))
		pr_err("rx msg list is empty\n");

	list_for_each_entry_safe(m, _m, &sst->rx_list, node) {
		pr_err("rx: pending msg header %#x\n", m->header.full);
		list_del(&m->node);
		kfree(m->mailbox_data);
		kfree(m);
	}
	spin_unlock_irqrestore(&sst->rx_msg_lock, irq_flags);
}

/* num_dwords: should be multiple of 4 */
static void dump_buffer_fromio(void __iomem *from,
				     unsigned int num_dwords)
{
	int i;
	u32 val[4];

	if (num_dwords % 4) {
		pr_err("%s: num_dwords %d not multiple of 4\n",
				__func__, num_dwords);
		return;
	}

	pr_err("****** Start *******\n");
	pr_err("Dump %d dwords, from location %p\n", num_dwords, from);

	for (i = 0; i < num_dwords; ) {
		val[0] = ioread32(from + (i++ * 4));
		val[1] = ioread32(from + (i++ * 4));
		val[2] = ioread32(from + (i++ * 4));
		val[3] = ioread32(from + (i++ * 4));
		pr_err("%.8x %.8x %.8x %.8x\n", val[0], val[1], val[2], val[3]);
	}
	pr_err("****** End *********\n\n\n");
}

static void sst_stall_lpe_n_wait(struct intel_sst_drv *sst)
{
	union config_status_reg_mrfld csr;
	void __iomem *dma_reg0 = sst->debugfs.dma_reg[0];
	void __iomem *dma_reg1 = sst->debugfs.dma_reg[1];
	int offset = 0x3A0; /* ChEnReg of DMA */


	pr_err("Before stall: DMA_0 Ch_EN %#llx DMA_1 Ch_EN %#llx\n",
				sst_reg_read64(dma_reg0, offset),
				sst_reg_read64(dma_reg1, offset));

	/* CSR operations should be in context of sst_lock */
	/* Stall LPE */
	csr.full = sst_shim_read64(sst->shim, SST_CSR);
	csr.part.runstall  = 1;
	sst_shim_write64(sst->shim, SST_CSR, csr.full);

	/* A 5ms delay, before resetting the LPE */
	usleep_range(5000, 5100);

	pr_err("After stall: DMA_0 Ch_EN %#llx DMA_1 Ch_EN %#llx\n",
				sst_reg_read64(dma_reg0, offset),
				sst_reg_read64(dma_reg1, offset));
}

#if IS_ENABLED(CONFIG_INTEL_SCU_IPC)
static void sst_send_scu_reset_ipc(struct intel_sst_drv *sst)
{
	int ret = 0;

	/* Reset and power gate the LPE */
	ret = intel_scu_ipc_simple_command(IPC_SCU_LPE_RESET, 0);
	if (ret) {
		pr_err("Power gating LPE failed %d\n", ret);
		reset_sst_shim(sst);
	} else {
		pr_err("LPE reset via SCU is success!!\n");
		pr_err("dump after LPE power cycle\n");
		dump_sst_shim(sst);

		/* Mask the DMA & SSP interrupts */
		sst_shim_write64(sst->shim, SST_IMRX, 0xFFFF0038);
	}
}
#else
static void sst_send_scu_reset_ipc(struct intel_sst_drv *sst)
{
	pr_debug("%s: do nothing, just return\n", __func__);
}
#endif

#define NUM_DWORDS		256
#define DUMP_SRAM_CHECKPOINT_DWORDS		640
void sst_do_recovery_mrfld(struct intel_sst_drv *sst)
{
	char iram_event[30], dram_event[30], ddr_imr_event[65], event_type[30];
	char *envp[5];
	int env_offset = 0;
	bool reset_dapm;
	struct sst_platform_cb_params cb_params;

	/*
	 * setting firmware state as RESET so that the firmware will get
	 * redownloaded on next request.This is because firmare not responding
	 * for 1 sec is equalant to some unrecoverable error of FW.
	 */
	pr_err("Audio: Intel SST engine encountered an unrecoverable error\n");
	pr_err("Audio: trying to reset the dsp now\n");

	mutex_lock(&sst->sst_lock);
	sst->sst_state = SST_RECOVERY;
	mutex_unlock(&sst->sst_lock);

	dump_stack();
	dump_sst_shim(sst);
	dump_lpe_stack(sst);
	cb_params.params = &reset_dapm;
	cb_params.event = SST_PLATFORM_TRIGGER_RECOVERY;
	reset_dapm = true;
	sst_platform_cb(&cb_params);

	mutex_lock(&sst->sst_lock);
	sst_stall_lpe_n_wait(sst);
	mutex_unlock(&sst->sst_lock);

	/* dump mailbox and sram */
	pr_err("Dumping Mailbox IA to LPE...\n");
	dump_buffer_fromio(sst->ipc_mailbox, NUM_DWORDS);
	pr_err("Dumping Mailbox LPE to IA...Lw\n");
	dump_buffer_fromio(sst->ipc_mailbox + sst->mailbox_recv_offset, NUM_DWORDS);
	pr_err("Dumping SRAM CHECKPOINT...\n");
	dump_buffer_fromio(sst->mailbox +  sst->pdata->debugfs_data->checkpoint_offset, DUMP_SRAM_CHECKPOINT_DWORDS);

	if (sst_drv_ctx->ops->set_bypass) {

		mutex_lock(&sst->sst_lock);
		sst_drv_ctx->ops->set_bypass(true);
		dump_ram_area(sst, &(sst->dump_buf), SST_IRAM);
		dump_ram_area(sst, &(sst->dump_buf), SST_DRAM);
		sst_drv_ctx->ops->set_bypass(false);
		mutex_unlock(&sst->sst_lock);

	}

	snprintf(event_type, sizeof(event_type), "EVENT_TYPE=SST_RECOVERY");
	envp[env_offset++] = event_type;
	snprintf(iram_event, sizeof(iram_event), "IRAM_DUMP_SIZE=%d",
					sst->dump_buf.iram_buf.size);
	envp[env_offset++] = iram_event;
	snprintf(dram_event, sizeof(dram_event), "DRAM_DUMP_SIZE=%d",
					sst->dump_buf.dram_buf.size);
	envp[env_offset++] = dram_event;

	if (sst->ddr != NULL) {
		snprintf(ddr_imr_event, sizeof(ddr_imr_event),
		"DDR_IMR_DUMP_SIZE=%d DDR_IMR_ADDRESS=%p", (sst->ddr_end - sst->ddr_base), sst->ddr);
		envp[env_offset++] = ddr_imr_event;
	}
	envp[env_offset] = NULL;
	kobject_uevent_env(&sst->dev->kobj, KOBJ_CHANGE, envp);
	pr_err("Recovery Uevent Sent!!\n");

	/* Send IPC to SCU to power gate and reset the LPE */
	sst_send_scu_reset_ipc(sst);

	pr_err("reset the pvt id from val %d\n", sst_drv_ctx->pvt_id);
	spin_lock(&sst_drv_ctx->block_lock);
	sst_drv_ctx->pvt_id = 0;

	spin_unlock(&sst_drv_ctx->block_lock);
	sst_dump_ipc_dispatch_lists(sst_drv_ctx);
	sst_dump_rx_lists(sst_drv_ctx);

	if (sst_drv_ctx->fw_in_mem) {
		pr_err("Clearing the cached FW copy...\n");
		kfree(sst_drv_ctx->fw_in_mem);
		sst_drv_ctx->fw_in_mem = NULL;
	}

	mutex_lock(&sst->sst_lock);
	sst->sst_state = SST_RESET;
	sst_stream_recovery(sst);
	mutex_unlock(&sst->sst_lock);

	/* Delay is to ensure that the stream is closed before
	 * powering on DAPM widget
	 */
	usleep_range(10000, 12000);
	reset_dapm = false;
	sst_platform_cb(&cb_params);
}

void sst_debug_dump(struct intel_sst_drv *sst)
{
	pr_err("Audio: Intel SST engine encountered an unrecoverable error\n");

	dump_stack();
	dump_sst_shim(sst);

	/* dump mailbox and sram */
	pr_err("Dumping Mailbox IA to LPE...\n");
	dump_buffer_fromio(sst->ipc_mailbox, NUM_DWORDS);
	pr_err("Dumping Mailbox LPE to IA...Lw\n");
	dump_buffer_fromio(sst->ipc_mailbox + sst->mailbox_recv_offset, NUM_DWORDS);
	if (sst->pdata->debugfs_data != NULL) {
		pr_err("Dumping SRAM CHECKPOINT...\n");
		dump_buffer_fromio(sst->mailbox + sst->pdata->debugfs_data->checkpoint_offset, sst->pdata->debugfs_data->checkpoint_size);
	}

	if (sst->sst_state == SST_FW_RUNNING &&
		sst_drv_ctx->pci_id == SST_CLV_PCI_ID)
		dump_sst_crash_area();

	sst_dump_ipc_dispatch_lists(sst_drv_ctx);

	/* Do stream recovery for CTP and BYT in case of sst timeout. For other platforms trigger
	   ipanic in case of sst timeout*/
	if ((sst_drv_ctx->pci_id == SST_CLV_PCI_ID) || (sst_drv_ctx->pci_id == SST_BYT_PCI_ID)) {
		mutex_lock(&sst->sst_lock);
		sst->sst_state = SST_RESET;
		sst_stream_recovery(sst);
		mutex_unlock(&sst->sst_lock);
	} else {
		pr_err("Triggering IPANIC due to SST wait timeout\n");
		dump_lpe_stack(sst);
		BUG();
	}

}

/*
 * sst_wait_timeout - wait on event for timeout
 *
 * @sst_drv_ctx: Driver context
 * @block: Driver block to wait on
 *
 * This function waits with a timeout value (and is not interruptible) on a
 * given block event
 */
int sst_wait_timeout(struct intel_sst_drv *sst_drv_ctx, struct sst_block *block)
{
	int retval = 0;
	unsigned int block_timeout;
	if (sst_drv_ctx->pci_id == SST_CLV_PCI_ID)
		block_timeout = SST_BLOCK_TIMEOUT_2SEC;
	else
		block_timeout = SST_BLOCK_TIMEOUT;

	/* NOTE:
	Observed that FW processes the alloc msg and replies even
	before the alloc thread has finished execution */
	pr_debug("sst: waiting for condition %x ipc %d drv_id %d\n",
		       block->condition, block->msg_id, block->drv_id);
	if (wait_event_timeout(sst_drv_ctx->wait_queue,
				block->condition,
				msecs_to_jiffies(block_timeout))) {
		/* event wake */
		pr_debug("sst: Event wake %x\n", block->condition);
		pr_debug("sst: message ret: %d\n", block->ret_code);
		retval = -block->ret_code;
	} else {
		block->on = false;
		pr_err("sst: Wait timed-out condition:%#x, msg_id:%#x fw_state %#x\n",
				block->condition, block->msg_id, sst_drv_ctx->sst_state);

		if (sst_drv_ctx->sst_state == SST_FW_LOADING) {
			pr_err("Can't recover as timedout while downloading the FW\n");
			pr_err("reseting fw state to RESET from %d ...\n", sst_drv_ctx->sst_state);
			sst_drv_ctx->sst_state = SST_RESET;

			dump_sst_shim(sst_drv_ctx);

			/* Reset & Power Off the LPE only for MRFLD */
			if (sst_drv_ctx->pci_id == SST_MRFLD_PCI_ID) {
				mutex_lock(&sst_drv_ctx->sst_lock);
				sst_stall_lpe_n_wait(sst_drv_ctx);
				mutex_lock(&sst_drv_ctx->sst_lock);

				/* Send IPC to SCU to power gate and reset the LPE */
				sst_send_scu_reset_ipc(sst_drv_ctx);
			}
		} else if (sst_drv_ctx->sst_state == SST_SHUTDOWN) {
			pr_err("sst: intel sst is already shutdown\n");
		} else {
			if (sst_drv_ctx->ops->do_recovery)
				sst_drv_ctx->ops->do_recovery(sst_drv_ctx);
		}

		retval = -EBUSY;
	}
	return retval;
}

/*
 * sst_create_ipc_msg - create a IPC message
 *
 * @arg: ipc message
 * @large: large or short message
 *
 * this function allocates structures to send a large or short
 * message to the firmware
 */
int sst_create_ipc_msg(struct ipc_post **arg, bool large)
{
	struct ipc_post *msg;

	msg = kzalloc(sizeof(struct ipc_post), GFP_ATOMIC);
	if (!msg) {
		pr_err("kzalloc ipc msg failed\n");
		return -ENOMEM;
	}
	if (large) {
		msg->mailbox_data = kzalloc(sst_drv_ctx->mailbox_size, GFP_ATOMIC);
		if (!msg->mailbox_data) {
			kfree(msg);
			pr_err("kzalloc mailbox_data failed");
			return -ENOMEM;
		}
	} else {
		msg->mailbox_data = NULL;
	}
	msg->is_large = large;
	*arg = msg;
	return 0;
}

/*
 * sst_create_block_and_ipc_msg - Creates IPC message and sst block
 * @arg: passed to sst_create_ipc_message API
 * @large: large or short message
 * @sst_drv_ctx: sst driver context
 * @block: return block allocated
 * @msg_id: IPC
 * @drv_id: stream id or private id
 */
int sst_create_block_and_ipc_msg(struct ipc_post **arg, bool large,
		struct intel_sst_drv *sst_drv_ctx, struct sst_block **block,
		u32 msg_id, u32 drv_id)
{
	int retval = 0;
	retval = sst_create_ipc_msg(arg, large);
	if (retval)
		return retval;
	*block = sst_create_block(sst_drv_ctx, msg_id, drv_id);
	if (*block == NULL) {
		kfree(*arg);
		return -ENOMEM;
	}
	return retval;
}

/*
 * sst_clean_stream - clean the stream context
 *
 * @stream: stream structure
 *
 * this function resets the stream contexts
 * should be called in free
 */
void sst_clean_stream(struct stream_info *stream)
{
	stream->status = STREAM_UN_INIT;
	stream->prev = STREAM_UN_INIT;
	mutex_lock(&stream->lock);
	stream->cumm_bytes = 0;
	mutex_unlock(&stream->lock);
}

void sst_update_timer(struct intel_sst_drv *sst_drv_ctx)
{
	struct intel_sst_drv *sst = sst_drv_ctx;

	if (sst_drv_ctx->pdata->start_recovery_timer) {
		if (&sst->monitor_lpe.sst_timer != NULL) {
			mod_timer(&sst->monitor_lpe.sst_timer, jiffies +
				msecs_to_jiffies(sst->monitor_lpe.interval));
			sst->monitor_lpe.prev_match_val = read_shim_data(sst, SST_TMRCTL);
		}
	}
}

void sst_trigger_recovery(struct work_struct *work)
{
	struct sst_monitor_lpe *monitor_lpe = container_of(work,
							struct sst_monitor_lpe, mwork);
	struct intel_sst_drv *sst  = container_of(monitor_lpe,
						struct intel_sst_drv, monitor_lpe);
	if (sst->ops->do_recovery)
		sst->ops->do_recovery(sst);
	return;
}

void sst_timer_cb(unsigned long data)
{
	struct intel_sst_drv *sst = (struct intel_sst_drv *)data;
	u64 curr_match_val = read_shim_data(sst, SST_TMRCTL);

	if (curr_match_val != sst->monitor_lpe.prev_match_val) {

		mod_timer(&sst->monitor_lpe.sst_timer, jiffies +
					msecs_to_jiffies(sst->monitor_lpe.interval));
		sst->monitor_lpe.prev_match_val = curr_match_val;

	} else {
		pr_err(" triggering recovery !!!\n");
		queue_work(sst->recovery_wq, &sst->monitor_lpe.mwork);
		del_timer(&sst->monitor_lpe.sst_timer);
	}

	return;

}

int sst_set_timer(struct sst_monitor_lpe *monitor_lpe, bool enable)
{
	int ret = 0;
	if (enable) {
		ret = mod_timer(&monitor_lpe->sst_timer, jiffies +
					msecs_to_jiffies(monitor_lpe->interval));
		pr_debug("sst: recovery timer started, timer interval=%d sec\n",
								monitor_lpe->interval/1000);
	} else  {

		if (&monitor_lpe->sst_timer != NULL)
			ret = del_timer_sync(&monitor_lpe->sst_timer);
		monitor_lpe->prev_match_val = 0;
		pr_debug("sst: recovery timer stopped\n");

	}

	return ret;
}
