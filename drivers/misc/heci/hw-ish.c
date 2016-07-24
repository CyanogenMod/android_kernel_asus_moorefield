/*
 * H/W layer of HECI provider device (ISH)
 *
 * Copyright (c) 2014, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/pci.h>
#include <linux/sched.h>
#include "client.h"
#include "hw-ish.h"
#include "utils.h"
#include "heci.h"
#include "heci_dev.h"
#include "hbm.h"

#ifdef dev_dbg
#undef dev_dbg
#endif
static void no_dev_dbg(void *v, char *s, ...)
{
}
/*#define dev_dbg dev_err */
#define dev_dbg no_dev_dbg

#include <linux/delay.h>


/**
 * heci_reg_read - Reads 32bit data from the heci device
 *
 * @base_addr: the base address of PCI memory BAR that contains the register to be read
 * @offset: offset from which to read the data
 *
 * returns the byte read.
 */
static inline u32 heci_reg_read(void __iomem *base_addr, unsigned long offset)
{
	u32 result;

	result = readl(base_addr + offset);
	return result;
}

/**
 * heci_reg_write - Writes 32bit data to the heci device
 *
 * @base_addr: the base address of PCI memory BAR that contains the register to be written
 * @offset: offset from which to write the data
 * @value: the byte to write
 */
static inline void heci_reg_write(void __iomem *base_addr,
				unsigned long offset, u32 value)
{

	writel(value, base_addr + offset);
}

/**
 * ish_reg_read - reads 32bit data from the ISH BAR
 *
 * @dev: the device structure
 * @offset: offset from which to read the data
 */
static inline u32 ish_reg_read(struct heci_device *dev, unsigned long offset)
{
	struct heci_ish_hw *hw = to_ish_hw(dev);
	return heci_reg_read(hw->mem_addr, offset);
}

/**
 * ish_reg_write - Writes 32bit data to the ISH BAR
 *
 * @dev: the device structure
 * @offset: offset from which to write the data
 * @value: the byte to write
 */
static inline void ish_reg_write(struct heci_device *dev,
				unsigned long offset, u32 value)
{
	struct heci_ish_hw *hw = to_ish_hw(dev);
	heci_reg_write(hw->mem_addr, offset, value);
}

static inline u32 ish_read_fw_sts_reg(struct heci_device *dev)
{
	return ish_reg_read(dev, IPC_REG_ISH_HOST_FWSTS);
}

bool check_generated_interrupt(struct heci_device *dev)
{
	bool interrupt_generated = true;
	u32 pisr_val = 0;

	pisr_val = ish_reg_read(dev, IPC_REG_PISR);
	interrupt_generated = IPC_INT_FROM_ISH_TO_HOST(pisr_val);

	return interrupt_generated;
}


u32 ipc_output_payload_read(struct heci_device *dev, unsigned long index)
{
	return ish_reg_read(dev, IPC_REG_ISH2HOST_MSG +	(index * sizeof(u32)));
}

/**
 * heci_read_slots - reads a message from heci device.
 *
 * @dev: the device structure
 * @buffer: message buffer will be written
 * @buffer_length: message size will be read
 */
static u32 heci_ish_read(struct heci_device *dev, unsigned char *buffer,
				unsigned long buffer_length)
{
	u32	i;
	u32	*reg_buf = (u32 *)buffer;
	u32	msg_offs;

	dev_dbg(&dev->pdev->dev, "buffer-length = %lu buf[0]0x%08X\n",
		buffer_length, ipc_output_payload_read(dev, 0));

	msg_offs = IPC_REG_ISH2HOST_MSG + sizeof(struct heci_msg_hdr);
	for (i = 0; i < buffer_length; i += sizeof(u32))
		*reg_buf++ = ish_reg_read(dev, msg_offs + i);

	/* Clear the doorbell */
	ish_reg_write(dev, IPC_REG_ISH2HOST_DRBL, 0);

	return 0;
}

/**
 * heci_ish_is_input_ready - check if ISH is ready for receiving data
 *
 * @dev: the device structure
 */
static bool heci_ish_is_input_ready(struct heci_device *dev)
{
	u32 doorbell_val;

	doorbell_val = ish_reg_read(dev, IPC_REG_HOST2ISH_DRBL);
	return !IPC_IS_BUSY(doorbell_val);
}

/**
 * heci_clear_interrupts - clear and stop interrupts
 *
 * @dev: the device structure
 */
static void heci_ish_intr_clear(struct heci_device *dev)
{
	dev_dbg(&dev->pdev->dev, "heci_ish_intr_clear - doing nothing\n");

}

/**
 * heci_ish_intr_enable - enables heci device interrupts
 *
 * @dev: the device structure
 */
static void heci_ish_intr_enable(struct heci_device *dev)
{
	/* u32 host_status = 0; */

	dev_dbg(&dev->pdev->dev, "heci_ish_intr_enable\n");
	/* temporary: disable FW from sending more data
	 * host_status = ish_reg_read(dev, IPC_REG_HOST_COMM);
	 * IPC_CLEAR_HOST_BUSY_READING(host_status);
	*/
	if (dev->pdev->revision == REVISION_ID_CHT_A0 ||
		(dev->pdev->revision & REVISION_ID_SI_MASK) == REVISION_ID_CHT_A0_SI)
		ish_reg_write(dev, IPC_REG_HOST_COMM, 0x81);
	else if (dev->pdev->revision == REVISION_ID_CHT_B0) {
		uint32_t host_comm_val;

		host_comm_val = ish_reg_read(dev, IPC_REG_HOST_COMM);
		host_comm_val |= IPC_HOSTCOMM_INT_EN_BIT | 0x81;
		ish_reg_write(dev, IPC_REG_HOST_COMM, host_comm_val);
	}
}

/**
 * heci_disable_interrupts - disables heci device interrupts
 *
 * @dev: the device structure
 */
static void heci_ish_intr_disable(struct heci_device *dev)
{
	u32 host_status = 0;

	dev_dbg(&dev->pdev->dev, "heci_ish_intr_disable\n");
	/* temporary: disable FW from sending more data
	 * host_status = ish_reg_read(dev, IPC_REG_HOST_COMM);
	 * IPC_SET_HOST_BUSY_READING(host_status);
	 */
	if (dev->pdev->revision == REVISION_ID_CHT_A0 ||
		(dev->pdev->revision & REVISION_ID_SI_MASK) == REVISION_ID_CHT_A0_SI)
		/*ish_reg_write(dev, IPC_REG_HOST_COMM, 0xC1)*/;
	else if (dev->pdev->revision == REVISION_ID_CHT_B0) {
		uint32_t host_comm_val;

		host_comm_val = ish_reg_read(dev, IPC_REG_HOST_COMM);
		host_comm_val &= ~IPC_HOSTCOMM_INT_EN_BIT;
		host_comm_val |= 0xC1;
		ish_reg_write(dev, IPC_REG_HOST_COMM, host_comm_val);
	}
}

/**
 * heci_ish_irq_quick_handler - The ISR of the HECI device
 *
 * @irq: The irq number
 * @dev_id: pointer to the device structure
 *
 * returns irqreturn_t
 */
irqreturn_t heci_ish_irq_quick_handler(int irq, void *dev_id)
{
	struct heci_device *dev = dev_id;
	struct heci_ish_hw *hw = to_ish_hw(dev);
	uint32_t doorbell_val;
	uint32_t mng_cmd;
	struct heci_msg_hdr *heci_hdr;
	struct heci_cl *cl_pos = NULL;
	struct heci_cl *cl_next = NULL;
	int cl_msg_ok = 0;
	bool interrupt_generated = true;
	u32 pisr_val = 0;

	/* For debug purposes, check and dump doorbell before PISR */
	doorbell_val = ish_reg_read(dev, IPC_REG_ISH2HOST_DRBL);

	/* Check that it's interrupt from ISH (may be shared) */
	pisr_val = ish_reg_read(dev, IPC_REG_PISR);
	interrupt_generated = IPC_INT_FROM_ISH_TO_HOST(pisr_val);

	if (!interrupt_generated)
		return IRQ_NONE;

	dev_dbg(&dev->pdev->dev, "before doorbell\n");
	hw->doorbell_val = ish_reg_read(dev, IPC_REG_ISH2HOST_DRBL);

	if (!IPC_IS_BUSY(hw->doorbell_val))
		return IRQ_NONE;
	dev_dbg(&dev->pdev->dev, "doorbell is busy - YES\n");

	heci_ish_intr_disable(dev);

	/* Clear doorbell bit (31) to stop level interrupt generation
	 * and set busy reading bit (30) to not release ownership on
	 * mailslot regs  (A0 HW BUG workaround) */
	/* doorbell_val = ish_reg_read(dev, IPC_REG_ISH2HOST_DRBL);
	doorbell_val &= ~0x80000000;
	doorbell_val |= 0x40000000;
	ish_reg_write(dev, IPC_REG_ISH2HOST_DRBL, doorbell_val);*/

	if (dev->pdev->revision == REVISION_ID_CHT_A0 ||
		(dev->pdev->revision & REVISION_ID_SI_MASK) == REVISION_ID_CHT_A0_SI) {
		ish_reg_write(dev, IPC_REG_ISH2HOST_DRBL, IPC_HOST_OWNS_MSG_BIT);
	}

	/* Any other mng command than reset_notify
	 * and reset_notify_ack won't wake BH handler
	 */
	if (IPC_HEADER_GET_PROTOCOL(hw->doorbell_val) == IPC_PROTOCOL_MNG) {
		mng_cmd = IPC_HEADER_GET_MNG_CMD(hw->doorbell_val);
		if (mng_cmd != MNG_RESET_NOTIFY && mng_cmd != MNG_RESET_NOTIFY_ACK) {
			ish_reg_write(dev, IPC_REG_ISH2HOST_DRBL, 0);
			/* Here and below: we need to actually read this register
			 * in order to unblock further interrupts on CHT A0
			 */
			dev->hbuf_is_ready = heci_ish_is_input_ready(dev);
			heci_enable_interrupts(dev);
			return	IRQ_HANDLED;
		}
	}

	if (IPC_HEADER_GET_PROTOCOL(hw->doorbell_val) != IPC_PROTOCOL_HECI)
		return	IRQ_WAKE_THREAD;

	/* Read HECI header dword. In future, read message here too */
	dev->rd_msg_hdr = heci_read_hdr(dev);
	heci_hdr = (struct heci_msg_hdr *)&dev->rd_msg_hdr;


	/* If received client message AND it is flow control, handle it here */
	if (!heci_hdr->host_addr) {
		/* FIXME: we don't use general read because that clears doorbell,
		 * and we don't want to do it here for A0 (yet).
		 * Later, either remove clearing doorbell from .read(),
		 * or keep all the reading only here */

		/*heci_read_slots(dev, dev->rd_msg_buf, heci_hdr->length);*/
		int i;
		uint32_t msg_offs;
		uint32_t *reg_buf = (uint32_t *)dev->rd_msg_buf;
		unsigned buf_len = heci_hdr->length;
		struct heci_bus_message	*heci_msg;

		msg_offs = IPC_REG_ISH2HOST_MSG + sizeof(struct heci_msg_hdr);
		for (i = 0; i < buf_len; i += sizeof(uint32_t))
			*reg_buf++ = ish_reg_read(dev, msg_offs + i);

		/*heci_hbm_dispatch(dev, heci_hdr);*/
		heci_msg = (struct heci_bus_message *)dev->rd_msg_buf;

		if (heci_msg->hbm_cmd == HECI_FLOW_CONTROL_CMD) {
			struct hbm_flow_control *flow_control =
					(struct hbm_flow_control *)heci_msg;
			struct heci_cl *cl = NULL;
			struct heci_cl *next = NULL;

			list_for_each_entry_safe(cl, next, &dev->file_list, link) {
				if (cl->host_client_id == flow_control->host_addr &&
					cl->me_client_id == flow_control->me_addr) {
					cl->heci_flow_ctrl_creds++;
					break;
				}
			}

			ish_reg_write(dev, IPC_REG_ISH2HOST_DRBL, 0);
			/* Here and above: we need to actually read this register
			 * in order to unblock further interrupts on CHT A0 */
			dev->hbuf_is_ready = heci_ish_is_input_ready(dev);
			heci_enable_interrupts(dev);
			return	IRQ_HANDLED;
		}
	}

	return IRQ_WAKE_THREAD;
}


static int heci_ish_fw_reset_handler(struct heci_device *dev)
{
	uint32_t reset_id;
	uint32_t reset_notify_ack =
		IPC_BUILD_MNG_MSG(MNG_RESET_NOTIFY_ACK, sizeof(uint32_t));

	/* Read reset ID */
	reset_id = ish_reg_read(dev, IPC_REG_ISH2HOST_MSG) & 0xFFFF;

	/* Handle FW-initiated reset */
	dev->dev_state = HECI_DEV_RESETTING;

	/* Clear HOST2ISH.ILUP (what's it?) */
	heci_ish_clr_host_rdy(dev);

	/* Handle ISH reset against upper layers */
	/* Remove all client devices */
	heci_bus_remove_all_clients(dev);

	/* Send reset_ack(reset_id) */
	ish_reg_write(dev, IPC_REG_HOST2ISH_MSG, reset_id);
	timed_wait_for_timeout(WAIT_FOR_SEND_SLICE, heci_ish_is_input_ready(dev), (2 * HZ));
	if (!heci_ish_is_input_ready(dev)) {
		/* ISH is dead */
		return	-EPIPE;
	} else {
		ish_reg_write(dev, IPC_REG_HOST2ISH_DRBL, reset_notify_ack);
	}

	/* Set HOST2ISH.ILUP */
	heci_ish_set_host_rdy(dev);

	/* Wait for ISH'es ILUP and HECI_READY */
	timed_wait_for_timeout(WAIT_FOR_SEND_SLICE, heci_ish_hw_is_ready(dev), (2 * HZ));
	if (!heci_ish_hw_is_ready(dev)) {
		/* ISH is dead */
		return -ENODEV;
	}

	return	0;
}


/**
 * heci_interrupt_thread_handler - function called after ISR to handle the interrupt
 * processing.
 *
 * @irq: The irq number
 * @dev_id: pointer to the device structure
 *
 * returns irqreturn_t
 *
 */
irqreturn_t heci_ish_irq_thread_handler(int irq, void *dev_id)
{

	struct heci_device *dev = (struct heci_device *) dev_id;
	struct heci_ish_hw *hw = to_ish_hw(dev);
	struct heci_cl_cb complete_list;
	u32 mng_cmd = 0;
	s32 slots;
	int rets;

again:
	dev_dbg(&dev->pdev->dev, "function called after ISR to handle the interrupt processing.\n");

	/* initialize our complete list */
	mutex_lock(&dev->device_lock);
	heci_io_list_init(&complete_list);

	/* Management protocol (not HECI) */
	if (IPC_HEADER_GET_PROTOCOL(hw->doorbell_val) == IPC_PROTOCOL_MNG) {
		mng_cmd = IPC_HEADER_GET_MNG_CMD(hw->doorbell_val);
		dev_dbg(&dev->pdev->dev, "got mng command: %d\n", mng_cmd);

		switch (mng_cmd) {
		default:
			break;

		case MNG_RESET_NOTIFY:
			if (heci_ish_fw_reset_handler(dev) != 0)
				break;

			/* ISH is ILUP & HECI-ready. Restart HECI */
			dev->recvd_hw_ready = 1;
			if (waitqueue_active(&dev->wait_hw_ready))
				wake_up_interruptible(&dev->wait_hw_ready);

			dev->dev_state = HECI_DEV_INIT_CLIENTS;
			dev->hbm_state = HECI_HBM_START;
			heci_hbm_start_req(dev);
			break;

		case MNG_RESET_NOTIFY_ACK:
			dev->recvd_hw_ready = 1;
			if (waitqueue_active(&dev->wait_hw_ready))
				wake_up_interruptible(&dev->wait_hw_ready);
			break;
		}

		/* DD -- in case of "MNG_" ALSO clear doorbell and clear
		 * busy-reading bit. This is correct here also for both
		 * A0 and B0
		 */
		ish_reg_write(dev, IPC_REG_ISH2HOST_DRBL, 0);

		goto	end;
	}

	/* Output Doorbell:
	 * Detection of SeC having sent output to host
	 */
	slots = heci_count_full_read_slots(dev);

	/* Read from ish */
	rets = heci_irq_read_handler(dev, &complete_list, &slots);
	dev_dbg(&dev->pdev->dev,
		"from heci_irq_read_handler ret=%d.\n", rets);
	if (rets)
		goto end;

	rets = heci_irq_write_handler(dev, &complete_list);
	dev_dbg(&dev->pdev->dev, "heci_irq_write_handler ret=%d.\n", rets);

end:
	dev_dbg(&dev->pdev->dev, "end of bottom half function.\n");
	/* FIXME: Check if this is correct !!! */
	dev->hbuf_is_ready = heci_ish_is_input_ready(dev);

	mutex_unlock(&dev->device_lock);
	heci_irq_compl_handler(dev, &complete_list);

out:
	/*if (heci_ish_pending_interrupts(dev))
		goto again;*/

	heci_enable_interrupts(dev);

	return IRQ_HANDLED;
}

/**
 * heci_ish_hw_is_ready - check if the hw is ready
 *
 * @dev: the device structure
 */
bool heci_ish_hw_is_ready(struct heci_device *dev)
{
	u32 ish_status =  ish_reg_read(dev, IPC_REG_ISH_HOST_FWSTS);
	return IPC_IS_ISH_ILUP(ish_status) && IPC_IS_ISH_HECI_READY(ish_status);
}

/**
 * heci_ish_host_is_ready - check if the host is ready
 *
 * @dev: the device structure
 */
bool heci_ish_host_is_ready(struct heci_device *dev)
{
	return true;
}

void heci_ish_set_host_rdy(struct heci_device *dev)
{
	u32  host_status = ish_reg_read(dev, IPC_REG_HOST_COMM);
	dev_dbg(&dev->pdev->dev, "before HOST start host_status=%08X\n", host_status);
	IPC_SET_HOST_READY(host_status);
	ish_reg_write(dev, IPC_REG_HOST_COMM, host_status);
	host_status = ish_reg_read(dev, IPC_REG_HOST_COMM);
	dev_dbg(&dev->pdev->dev, "actually sent HOST start host_status=%08X\n", host_status);
}

void heci_ish_clr_host_rdy(struct heci_device *dev)
{
	u32  host_status = ish_reg_read(dev, IPC_REG_HOST_COMM);
	dev_dbg(&dev->pdev->dev, "before HOST start host_status=%08X\n", host_status);
	IPC_CLEAR_HOST_READY(host_status);
	ish_reg_write(dev, IPC_REG_HOST_COMM, host_status);
	host_status = ish_reg_read(dev, IPC_REG_HOST_COMM);
	dev_dbg(&dev->pdev->dev, "actually sent HOST start host_status=%08X\n", host_status);
}

/**
 * heci_ish_hw_reset - resets host and fw.
 *
 * @dev: the device structure
 * @intr_enable: if interrupt should be enabled after reset.
 */
static int heci_ish_hw_reset(struct heci_device *dev, bool intr_enable)
{
	struct heci_ish_hw *hw = to_ish_hw(dev);
	struct IPC_MNG_PAYLOAD_TYPE ipc_mng_msg;
	u32 outmsg = 0;
	u32 doorbell_val = 0;

	dev_dbg(&dev->pdev->dev, "heci_ish_hw_reset\n");
	/*temporary we'll send reset*/

	ipc_mng_msg.reset_id   = 1;
	ipc_mng_msg.reserved   = 0;

	heci_ish_intr_enable(dev);

	/* DEBUG: send self-interrupt and wait 100 (ms) for it to appear in klog */
	/* ish_reg_write(dev, IPC_REG_ISH2HOST_DRBL, 0x80000000);
	 * mdelay(100);
	 */

	/*send message */
	memcpy(&outmsg, &ipc_mng_msg, sizeof(ipc_mng_msg));
	ish_reg_write(dev, IPC_REG_HOST2ISH_MSG, outmsg);

	/* Clear the incoming doorbell */
	ish_reg_write(dev, IPC_REG_ISH2HOST_DRBL, 0);

	/* Fixed: this should be set BEFORE writing RESET_NOTIFY,
	 * lest response will be received BEFORE this clearing... */
	dev->recvd_hw_ready = 0;

	doorbell_val =  IPC_BUILD_MNG_MSG(MNG_RESET_NOTIFY, sizeof(ipc_mng_msg));
	ish_reg_write(dev,   IPC_REG_HOST2ISH_DRBL, doorbell_val);

	mutex_unlock(&dev->device_lock);
	wait_event_interruptible(dev->wait_hw_ready, dev->recvd_hw_ready);
	mutex_lock(&dev->device_lock);

	dev_dbg(&dev->pdev->dev, "exit initial link wait\n");

	return 0;
}

static void heci_ish_hw_config(struct heci_device *dev)
{

	struct heci_ish_hw *hw = to_ish_hw(dev);
	/* Doesn't change in runtime */
	dev->hbuf_depth = PAYLOAD_SIZE / 4;
	hw->doorbell_val = 0;
	dev_dbg(&dev->pdev->dev, "heci_ish_hw_config\n");
}

static int heci_ish_hw_start(struct heci_device *dev)
{
	dev_dbg(&dev->pdev->dev, "heci_ish_hw_start\n");
	heci_ish_set_host_rdy(dev);
	return 0;
}


static int heci_ish_hbuf_empty_slots(struct heci_device *dev)
{
	dev_dbg(&dev->pdev->dev, "heci_ish_hbuf_empty_slots\n");
	return dev->hbuf_depth;
}

static size_t heci_ish_hbuf_max_len(const struct heci_device *dev)
{
	return PAYLOAD_SIZE - sizeof(struct heci_msg_hdr);
}

static int heci_ish_count_full_read_slots(struct heci_device *dev)
{
	/* read buffers has static size */
	return  PAYLOAD_SIZE / 4;
}

static u32 heci_ish_read_hdr(const struct heci_device *dev)
{
	return ish_reg_read(dev, IPC_REG_ISH2HOST_MSG);
}


/**
 * heci_ish_write - writes a message to heci device.
 *
 * @dev: the device structure
 * @header: header of message
 * @buf: message buffer will be written
 * returns 1 if success, 0 - otherwise.
 */

static int heci_ish_write(struct heci_device *dev,
			struct heci_msg_hdr *header, unsigned char *buf)
{
	struct heci_ish_hw *hw = to_ish_hw(dev);
	u32 doorbell_val;
	unsigned long length = header->length;
	unsigned long rem;
	u32 *reg_buf = (u32 *)buf;
	int i;

	dev_dbg(&dev->pdev->dev, "heci_ish_write %ld\n bytes", length);

	/* If  doorbell is busy, return -EBUSY */
	if (!dev->ops->hbuf_is_ready(dev))
		return	-EBUSY;

	if ((length + sizeof(struct heci_msg_hdr)) > PAYLOAD_SIZE) {
		dev_dbg(&dev->pdev->dev, "write lenght exceeded = %ld\n", length);
		return 0;
	}

	/* Write header */
	ish_reg_write(dev, IPC_REG_HOST2ISH_MSG, *((u32 *)header));

	/* Write the message */
	for (i = 0; i < length / 4; i++)
		ish_reg_write(dev, IPC_REG_HOST2ISH_MSG + ((i + 1) * sizeof(u32)), reg_buf[i]);

	rem = length & 0x3;
	if (rem > 0) {
		u32 reg = 0;
		memcpy(&reg, &buf[length - rem], rem);
		ish_reg_write(dev, IPC_REG_HOST2ISH_MSG + ((i + 1) * sizeof(u32)), reg);
	}

	dev->hbuf_is_ready = false;

	/* Set the door bell values and busy bit; only when we write it to
	 * the register, ISH will read the data
	 */
	doorbell_val = IPC_BUILD_HEADER(header->length + sizeof(struct heci_msg_hdr), IPC_PROTOCOL_HECI, 1);
	ish_reg_write(dev, IPC_REG_HOST2ISH_DRBL, doorbell_val);

	return 0;
}

static const struct heci_hw_ops heci_ish_hw_ops = {
	.host_is_ready = heci_ish_host_is_ready,
	.hw_is_ready = heci_ish_hw_is_ready,
	.hw_reset = heci_ish_hw_reset,
	.hw_config = heci_ish_hw_config,
	.hw_start = heci_ish_hw_start,
	.intr_clear = heci_ish_intr_clear,
	.intr_enable = heci_ish_intr_enable,
	.intr_disable = heci_ish_intr_disable,
	.hbuf_free_slots = heci_ish_hbuf_empty_slots,
	.hbuf_is_ready = heci_ish_is_input_ready,
	.hbuf_max_len = heci_ish_hbuf_max_len,
	.write = heci_ish_write,
	.rdbuf_full_slots = heci_ish_count_full_read_slots,
	.read_hdr = heci_ish_read_hdr,
	.read = heci_ish_read
};

struct heci_device *heci_ish_dev_init(struct pci_dev *pdev)
{
	struct heci_device *dev;
	struct heci_ish_hw *hw;

	dev = kzalloc(sizeof(struct heci_device) +
			 sizeof(struct heci_ish_hw), GFP_KERNEL);
	if (!dev)
		return NULL;

	heci_device_init(dev);
	hw = to_ish_hw(dev);
	dev->ops = &heci_ish_hw_ops;
	dev->pdev = pdev;
	return dev;
}
