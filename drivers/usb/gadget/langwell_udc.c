/*
 * Intel Langwell/Penwell USB Device Controller driver
 * Copyright (C) 2008-2010, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */


/* #undef	DEBUG */
/* #undef	VERBOSE_DEBUG */

#if defined(CONFIG_USB_LANGWELL_OTG) || defined(CONFIG_USB_PENWELL_OTG)
#define	OTG_TRANSCEIVER
#endif


#include <linux/module.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/otg.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/wakelock.h>
#include <asm/unaligned.h>

#include "langwell_udc.h"

#ifdef	OTG_TRANSCEIVER
#include <linux/usb/penwell_otg.h>
#endif

#define	DRIVER_DESC	"Intel USB2.0 Device Controller driver"
#define	DRIVER_VERSION	"June 3, 2010"

static const char driver_name[] = "langwell_udc";
static const char driver_desc[] = DRIVER_DESC;

/* controller device global variable */
static struct langwell_udc	*the_controller;

/* for endpoint 0 operations */
static const struct usb_endpoint_descriptor
langwell_ep0_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	0,
	.bmAttributes =		USB_ENDPOINT_XFER_CONTROL,
	.wMaxPacketSize =	EP0_MAX_PKT_SIZE,
};

static void ep_set_halt(struct langwell_ep *ep, int value);


#ifdef CONFIG_DEBUG_FS
#ifdef readl
#undef readl
#endif
#ifdef writel
#undef writel
#endif
#define readl(addr) ({ if (check_pm_otg()) { \
	panic("usb otg, read reg:%p, pm_sss0_base:0x%x",  \
	addr, *(pm_sss0_base)); }; __le32_to_cpu(__raw_readl(addr)); })
#define writel(b, addr) ({ if (check_pm_otg()) { \
	panic("usb otg, write reg:%p, pm_sss0_base:0x%x",  \
	addr, *(pm_sss0_base)); }; __raw_writel(__cpu_to_le32(b), addr); })
#endif


#ifdef	VERBOSE_DEBUG
/*-------------------------------------------------------------------------*/
/* debugging */

/* dump_ep dumps all information a specific endpoint
*  @ep_index: the endpoint index
*  Caller should lock dev->lock first
*/
static void dump_ep(struct langwell_udc *dev, int ep_index)
{
	struct langwell_ep *ep;
	struct langwell_dqh *dqh = &dev->ep_dqh[ep_index];
	struct langwell_request *req;
	struct langwell_dtd     *dtd;
	u32	value, i;
	u32 in = ep_index % 2;
	u32 num = ep_index / 2;

	printk(KERN_ERR"Showing ep %d (ep%d%s) ...\n\n",
			ep_index, num, in ? "IN" : "OUT");

	printk(KERN_ERR"[ep REG]\n");
	printk(KERN_ERR"\tendptPRIME Bit: %u, endptSTATUS Bit: %u\n",
		readl(&dev->op_regs->endptprime) & (1 << (num + in ? 16 : 0)),
		readl(&dev->op_regs->endptstat) & (1 << (num + in ? 16 : 0))
		);
	value = readl(&dev->op_regs->endptctrl[num]);
	value >>= in ? 16 : 0;
	printk(KERN_ERR"\tendptCTRL = 0x%08x\n"
		"\t\tEnabled? %s, Stalled? %s\n",
		readl(&dev->op_regs->endptctrl[num]),
		value & BIT(7) ? "Yes" : "NO",
		value & BIT(0) ? "Yes" : "NO"
		);
	value = readl(&dev->op_regs->endptsetupstat);
	printk(KERN_ERR
		"\tendptSETUPSTAT = 0x%04x\n\n",
		value & SETUPSTAT_MASK
		);

	printk(KERN_ERR"[dQH]\n");
	printk(KERN_ERR
		"\tcurrent = 0x%08x\n"
		"\tdtd_next = 0x%08x\n"
		"\tdtd_status = 0x%08x\n"
		"\tdtd_ioc = 0x%08x\n"
		"\tdtd_total = 0x%08x\n\n",
		dqh->dqh_current,
		dqh->dtd_next,
		dqh->dtd_status,
		dqh->dtd_ioc,
		dqh->dtd_total
		);

	if (ep_index == 0)
		return;

	printk(KERN_ERR"[Request & dTD]\n");
	ep = &dev->ep[ep_index == 1 ? 0 : ep_index];
	list_for_each_entry(req, &ep->queue, queue) {
		printk(KERN_ERR
			"req %p actual 0x%x length "
			"0x%x  buf %p\n",
			&req->req, req->req.actual,
			req->req.length, req->req.buf
			);

		dtd = req->head;
		for (i = 0; i < req->dtd_count; i++) {
			printk(KERN_ERR
				"\tdTD %d (virt addr 0x%p)\n"
				"\t\tdtd_next = 0x%08x\n"
				"\t\tdtd_status = 0x%08x\n"
				"\t\tdtd_ioc = 0x%08x\n"
				"\t\tdtd_total = 0x%08x\n"
				"\t\tdtd_dma = 0x%08llx\n",
				i, dtd,
				dtd->dtd_next,
				dtd->dtd_status,
				dtd->dtd_ioc,
				dtd->dtd_total,
				dtd->dtd_dma
				);

			if (i != req->dtd_count - 1)
				dtd = (struct langwell_dtd *)dtd->next_dtd_virt;
		}
	}

	return;
}


static inline void print_all_registers(struct langwell_udc *dev)
{
	int	i;

	dev_dbg(&dev->pdev->dev, STRING_LNW_REGS(dev));

	for (i = 0; i < dev->ep_max / 2; i++) {
		dev_dbg(&dev->pdev->dev, "endptctrl[%d]=0x%08x\n",
				i, readl(&dev->op_regs->endptctrl[i]));
	}
}
#else

#define	print_all_registers(dev)	do { } while (0)

#endif /* VERBOSE_DEBUG */


/*-------------------------------------------------------------------------*/

#define	is_in(ep)	(((ep)->ep_num == 0) ? ((ep)->dev->ep0_dir ==	\
			USB_DIR_IN) : (usb_endpoint_dir_in((ep)->desc)))

#define	DIR_STRING(ep)	(is_in(ep) ? "in" : "out")


static char *type_string(const struct usb_endpoint_descriptor *desc)
{
	switch (usb_endpoint_type(desc)) {
	case USB_ENDPOINT_XFER_BULK:
		return "bulk";
	case USB_ENDPOINT_XFER_ISOC:
		return "iso";
	case USB_ENDPOINT_XFER_INT:
		return "int";
	};

	return "control";
}

#ifdef	OTG_TRANSCEIVER
static void langwell_udc_notify_otg(unsigned long event)
{
	struct langwell_udc	*dev = the_controller;

	if (dev && dev->iotg && event)
		atomic_notifier_call_chain(&dev->iotg->iotg_notifier,
				event, dev->iotg);
}
#else
static inline void langwell_udc_notify_otg(unsigned long event)
{
	return;
}
#endif

/* configure endpoint control registers */
static void ep_reset(struct langwell_ep *ep, unsigned char ep_num,
		unsigned char is_in, unsigned char ep_type)
{
	struct langwell_udc	*dev;
	u32			endptctrl;

	dev = ep->dev;
	dev_vdbg(&dev->pdev->dev, "---> %s()\n", __func__);

	endptctrl = readl(&dev->op_regs->endptctrl[ep_num]);
	if (is_in) {	/* TX */
		if (ep_num)
			endptctrl |= EPCTRL_TXR;
		endptctrl |= EPCTRL_TXE;
		endptctrl |= ep_type << EPCTRL_TXT_SHIFT;
	} else {	/* RX */
		if (ep_num)
			endptctrl |= EPCTRL_RXR;
		endptctrl |= EPCTRL_RXE;
		endptctrl |= ep_type << EPCTRL_RXT_SHIFT;
	}

	writel(endptctrl, &dev->op_regs->endptctrl[ep_num]);

	/* has to manually clear halt for non-control endpoints */
	ep_set_halt(ep, 0);

	dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
}


/* reset ep0 dQH and endptctrl */
static void ep0_reset(struct langwell_udc *dev)
{
	struct langwell_ep	*ep;
	int			i;

	dev_vdbg(&dev->pdev->dev, "---> %s()\n", __func__);

	/* ep0 in and out */
	for (i = 0; i < 2; i++) {
		ep = &dev->ep[i];
		ep->dev = dev;

		/* ep0 dQH */
		ep->dqh = &dev->ep_dqh[i];

		/* configure ep0 endpoint capabilities in dQH */
		ep->dqh->dqh_ios = 1;
		ep->dqh->dqh_mpl = EP0_MAX_PKT_SIZE;

		/*
		 * Disable HW zero length termination
		 * select for control endpoints.
		 */
		ep->dqh->dqh_zlt = 1;

		ep->dqh->dqh_mult = 0;

		ep->dqh->dtd_next = DTD_TERM;

		/* configure ep0 control registers */
		ep_reset(&dev->ep[0], 0, i, USB_ENDPOINT_XFER_CONTROL);
	}

	dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
}


/*-------------------------------------------------------------------------*/

/* endpoints operations */

/* configure endpoint, making it usable */
static int langwell_ep_enable(struct usb_ep *_ep,
		const struct usb_endpoint_descriptor *desc)
{
	struct langwell_udc	*dev;
	struct langwell_ep	*ep;
	u16			max = 0;
	unsigned long		flags;
	int			i, retval = 0;
	unsigned char		zlt, ios = 0, mult = 0;

	if (!_ep || !desc)
		return -EINVAL;

	ep = container_of(_ep, struct langwell_ep, ep);
	dev = ep->dev;
	dev_vdbg(&dev->pdev->dev, "---> %s()\n", __func__);

	if (ep->desc || desc->bDescriptorType != USB_DT_ENDPOINT)
		return -EINVAL;

	if (!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN)
		return -ESHUTDOWN;

	max = usb_endpoint_maxp(desc);

	/*
	 * disable HW zero length termination select
	 * driver handles zero length packet through req->req.zero
	 */
	zlt = 1;

	/*
	 * sanity check type, direction, address, and then
	 * initialize the endpoint capabilities fields in dQH
	 */
	switch (usb_endpoint_type(desc)) {
	case USB_ENDPOINT_XFER_CONTROL:
		ios = 1;
		break;
	case USB_ENDPOINT_XFER_BULK:
		if ((dev->gadget.speed == USB_SPEED_HIGH
					&& max != 512)
				|| (dev->gadget.speed == USB_SPEED_FULL
					&& max > 64)) {
			goto done;
		}
		break;
	case USB_ENDPOINT_XFER_INT:
		if (strstr(ep->ep.name, "-iso")) /* bulk is ok */
			goto done;

		switch (dev->gadget.speed) {
		case USB_SPEED_HIGH:
			if (max <= 1024)
				break;
		case USB_SPEED_FULL:
			if (max <= 64)
				break;
		default:
			if (max <= 8)
				break;
			goto done;
		}
		break;
	case USB_ENDPOINT_XFER_ISOC:
		if (strstr(ep->ep.name, "-bulk")
				|| strstr(ep->ep.name, "-int"))
			goto done;

		switch (dev->gadget.speed) {
		case USB_SPEED_HIGH:
			if (max <= 1024)
				break;
		case USB_SPEED_FULL:
			if (max <= 1023)
				break;
		default:
			goto done;
		}
		/*
		 * FIXME:
		 * calculate transactions needed for high bandwidth iso
		 */
		mult = (unsigned char)(1 + ((max >> 11) & 0x03));
		max = max & 0x7ff;	/* bit 0~10 */
		/* 3 transactions at most */
		if (mult > 3)
			goto done;
		break;
	default:
		goto done;
	}

	spin_lock_irqsave(&dev->lock, flags);

	ep->ep.maxpacket = max;
	ep->desc = desc;
	ep->stopped = 0;
	ep->ep_num = usb_endpoint_num(desc);

	/* ep_type */
	ep->ep_type = usb_endpoint_type(desc);

	/* configure endpoint control registers */
	ep_reset(ep, ep->ep_num, is_in(ep), ep->ep_type);

	/* configure endpoint capabilities in dQH */
	i = ep->ep_num * 2 + is_in(ep);
	ep->dqh = &dev->ep_dqh[i];
	ep->dqh->dqh_ios = ios;
	ep->dqh->dqh_mpl = cpu_to_le16(max);
	ep->dqh->dqh_zlt = zlt;
	ep->dqh->dqh_mult = mult;
	ep->dqh->dtd_next = DTD_TERM;

	dev_dbg(&dev->pdev->dev, "enabled %s (ep%d%s-%s), max %04x\n",
			_ep->name,
			ep->ep_num,
			DIR_STRING(ep),
			type_string(desc),
			max);

	spin_unlock_irqrestore(&dev->lock, flags);
done:
	dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
	return retval;
}


/*-------------------------------------------------------------------------*/

/* retire a request */
static void done(struct langwell_ep *ep, struct langwell_request *req,
		int status)
{
	struct langwell_udc	*dev = ep->dev;
	unsigned		stopped = ep->stopped;
	struct langwell_dtd	*curr_dtd, *next_dtd;
	int			i;

	dev_vdbg(&dev->pdev->dev, "---> %s()\n", __func__);

	/* remove the req from ep->queue */
	list_del_init(&req->queue);

	if (req->req.status == -EINPROGRESS)
		req->req.status = status;
	else
		status = req->req.status;

	/* WORKAROUND: This is for a HW IP issue found
	* Controller reads "Next Link Pointer" from dTD even after dTD
	* is retired. Thus, software can't free the dTD too early.
	* Currently add 500 nanoseconds delay before freeing the dTD.
	* The exact delay may change if HW team finds out the more
	* appropriate/safe number.
	*/
	ndelay(500);

	/* free dTD for the request */
	if (dev->dtd_pool) {
		next_dtd = req->head;
		for (i = 0; i < req->dtd_count; i++) {
			curr_dtd = next_dtd;
			if (i != req->dtd_count - 1)
				next_dtd = curr_dtd->next_dtd_virt;
			dma_pool_free(dev->dtd_pool, curr_dtd,
					curr_dtd->dtd_dma);
		}
	}

	if (req->mapped) {
		dma_unmap_single(&dev->pdev->dev,
			req->req.dma, req->req.length,
			is_in(ep) ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
		req->req.dma = DMA_ADDR_INVALID;
		req->mapped = 0;
	} else if (req->req.dma != DMA_ADDR_INVALID)
		dma_sync_single_for_cpu(&dev->pdev->dev, req->req.dma,
				req->req.length,
				is_in(ep) ? DMA_TO_DEVICE : DMA_FROM_DEVICE);

	if (status != -ESHUTDOWN)
		dev_dbg(&dev->pdev->dev,
				"complete %s, req %p, stat %d, len %u/%u\n",
				ep->ep.name, &req->req, status,
				req->req.actual, req->req.length);

	/* don't modify queue heads during completion callback */
	ep->stopped = 1;

	spin_unlock(&dev->lock);
	/* complete routine from gadget driver */
	if (req->req.complete)
		req->req.complete(&ep->ep, &req->req);

	spin_lock(&dev->lock);
	ep->stopped = stopped;

	dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
}


static void langwell_ep_fifo_flush(struct usb_ep *_ep);

/* delete all endpoint requests, called with spinlock held */
static void nuke(struct langwell_ep *ep, int status)
{
	/* called with spinlock held */
	ep->stopped = 1;

	/* endpoint fifo flush */
	if (&ep->ep && ep->desc)
		langwell_ep_fifo_flush(&ep->ep);

	while (!list_empty(&ep->queue)) {
		struct langwell_request	*req = NULL;
		req = list_entry(ep->queue.next, struct langwell_request,
				queue);
		done(ep, req, status);
	}
}


/*-------------------------------------------------------------------------*/

/* endpoint is no longer usable */
static int langwell_ep_disable(struct usb_ep *_ep)
{
	struct langwell_ep	*ep;
	unsigned long		flags;
	struct langwell_udc	*dev;
	int			ep_num;
	u32			endptctrl;

	if (!_ep)
		return -EINVAL;

	ep = container_of(_ep, struct langwell_ep, ep);
	dev = ep->dev;
	dev_vdbg(&dev->pdev->dev, "---> %s()\n", __func__);

	if (!ep->desc)
		return -EINVAL;

	spin_lock_irqsave(&dev->lock, flags);

	if (!ep->desc) {
		spin_unlock_irqrestore(&dev->lock, flags);
		dev_err(&dev->pdev->dev, "ep has already disabled\n");
		return -EINVAL;
	}

	/* disable endpoint control register */
	ep_num = ep->ep_num;
	endptctrl = readl(&dev->op_regs->endptctrl[ep_num]);
	if (is_in(ep))
		endptctrl &= ~EPCTRL_TXE;
	else
		endptctrl &= ~EPCTRL_RXE;
	/* clear TXT bits, or this TXT bits value remains.
	 * It affects afterward endpoint configuration
	 */
	endptctrl &= ~(USB_ENDPOINT_XFERTYPE_MASK << EPCTRL_TXT_SHIFT);

	writel(endptctrl, &dev->op_regs->endptctrl[ep_num]);

	/* nuke all pending requests (does flush) */
	nuke(ep, -ESHUTDOWN);

	ep->desc = NULL;
	ep->ep.desc = NULL;
	ep->stopped = 1;
	ep->ep.maxpacket = (unsigned short) ~0;

	spin_unlock_irqrestore(&dev->lock, flags);

	dev_dbg(&dev->pdev->dev, "disabled %s\n", _ep->name);
	dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);

	return 0;
}


/* allocate a request object to use with this endpoint */
static struct usb_request *langwell_alloc_request(struct usb_ep *_ep,
		gfp_t gfp_flags)
{
	struct langwell_ep	*ep;
	struct langwell_udc	*dev;
	struct langwell_request	*req = NULL;

	if (!_ep)
		return NULL;

	ep = container_of(_ep, struct langwell_ep, ep);
	dev = ep->dev;
	dev_vdbg(&dev->pdev->dev, "---> %s()\n", __func__);

	req = kzalloc(sizeof(*req), gfp_flags);
	if (!req)
		return NULL;

	req->req.dma = DMA_ADDR_INVALID;
	INIT_LIST_HEAD(&req->queue);

	dev_vdbg(&dev->pdev->dev, "alloc request for %s\n", _ep->name);
	dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
	return &req->req;
}


/* free a request object */
static void langwell_free_request(struct usb_ep *_ep,
		struct usb_request *_req)
{
	struct langwell_ep	*ep;
	struct langwell_udc	*dev;
	struct langwell_request	*req = NULL;

	if (!_ep || !_req)
		return;

	ep = container_of(_ep, struct langwell_ep, ep);
	dev = ep->dev;
	dev_vdbg(&dev->pdev->dev, "---> %s()\n", __func__);

	req = container_of(_req, struct langwell_request, req);
	WARN_ON(!list_empty(&req->queue));

	if (_req)
		kfree(req);

	dev_vdbg(&dev->pdev->dev, "free request for %s\n", _ep->name);
	dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
}


/*-------------------------------------------------------------------------*/

/* queue dTD and PRIME endpoint */
static int queue_dtd(struct langwell_ep *ep, struct langwell_request *req)
{
	u32			bit_mask, usbcmd, endptstat, dtd_dma;
	u8			dtd_status;
	int			i;
	struct langwell_dqh	*dqh;
	struct langwell_udc	*dev;
	bool			saw_zero;
	u32			timeout;

	dev = ep->dev;
	dev_vdbg(&dev->pdev->dev, "---> %s()\n", __func__);

	i = ep->ep_num * 2 + is_in(ep);
	dqh = &dev->ep_dqh[i];

	if (ep->ep_num)
		dev_vdbg(&dev->pdev->dev, "%s\n", ep->name);
	else
		/* ep0 */
		dev_vdbg(&dev->pdev->dev, "%s-%s\n", ep->name, DIR_STRING(ep));

	dev_vdbg(&dev->pdev->dev, "ep_dqh[%d] addr: 0x%p\n",
			i, &(dev->ep_dqh[i]));

	bit_mask = is_in(ep) ?
		(1 << (ep->ep_num + 16)) : (1 << (ep->ep_num));

	dev_vdbg(&dev->pdev->dev, "bit_mask = 0x%08x\n", bit_mask);

	/* check if the pipe is empty */
	if (!(list_empty(&ep->queue))) {
		/* add dTD to the end of linked list */
		struct langwell_request	*lastreq;
		lastreq = list_entry(ep->queue.prev,
				struct langwell_request, queue);

		lastreq->tail->dtd_next =
			cpu_to_le32(req->head->dtd_dma & DTD_NEXT_MASK);

		/* read prime bit, if 1 goto out */
		if (readl(&dev->op_regs->endptprime) & bit_mask)
			goto out;

		/* CLVP A0 */
		if (dev->pdev->device == 0xE006 && dev->pdev->revision < 0xC) {
			saw_zero = false;
step_3:
			/* set ATDTW bit in USBCMD */
			usbcmd = readl(&dev->op_regs->usbcmd);
			writel(usbcmd | CMD_ATDTW, &dev->op_regs->usbcmd);

			/* read correct status bit */
			endptstat = readl(&dev->op_regs->endptstat) & bit_mask;

			if (!(readl(&dev->op_regs->usbcmd) & CMD_ATDTW)) {
				saw_zero = true;
				goto step_3;
			}

			if (!saw_zero) {
				timeout = 20;
				while (timeout &&
				readl(&dev->op_regs->usbcmd) & CMD_ATDTW)
					timeout--;

				if (timeout > 0) {
					saw_zero = true;
					goto step_3;
				}
			}
		/* All other silicons except CLVP A0 */
		} else {
			do {
				/* set ATDTW bit in USBCMD */
				usbcmd = readl(&dev->op_regs->usbcmd);
				writel(usbcmd | CMD_ATDTW,
					&dev->op_regs->usbcmd);

				/* read correct status bit */
				endptstat =
				readl(&dev->op_regs->endptstat) & bit_mask;

			} while (!(readl(&dev->op_regs->usbcmd) & CMD_ATDTW));
		}
		/* write ATDTW bit to 0 */
		usbcmd = readl(&dev->op_regs->usbcmd);
		writel(usbcmd & ~CMD_ATDTW, &dev->op_regs->usbcmd);

		if (endptstat)
			goto out;
	}

	/* write dQH next pointer and terminate bit to 0 */
	dtd_dma = req->head->dtd_dma & DTD_NEXT_MASK;
	dqh->dtd_next = cpu_to_le32(dtd_dma);

	/* clear active and halt bit */
	dtd_status = (u8) ~(DTD_STS_ACTIVE | DTD_STS_HALTED);
	dqh->dtd_status &= dtd_status;
	dev_vdbg(&dev->pdev->dev, "dqh->dtd_status = 0x%x\n", dqh->dtd_status);

	/* ensure that updates to the dQH will occur before priming */
	wmb();

	/* write 1 to endptprime register to PRIME endpoint */
	bit_mask = is_in(ep) ? (1 << (ep->ep_num + 16)) : (1 << ep->ep_num);
	dev_vdbg(&dev->pdev->dev, "endprime bit_mask = 0x%08x\n", bit_mask);
	writel(bit_mask, &dev->op_regs->endptprime);
out:
	dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
	return 0;
}


/* fill in the dTD structure to build a transfer descriptor */
static struct langwell_dtd *build_dtd(struct langwell_request *req,
		unsigned *length, dma_addr_t *dma, int *is_last)
{
	u32			 buf_ptr;
	struct langwell_dtd	*dtd = NULL;
	struct langwell_udc	*dev;
	int			i;

	dev = req->ep->dev;
	dev_vdbg(&dev->pdev->dev, "---> %s()\n", __func__);

	/* the maximum transfer length, up to 16k bytes */
	*length = min(req->req.length - req->req.actual,
			(unsigned)DTD_MAX_TRANSFER_LENGTH);

	/* create dTD dma_pool resource */
	if (dev->dtd_pool)
		dtd = dma_pool_alloc(dev->dtd_pool, GFP_ATOMIC, dma);

	if (dtd == NULL)
		return dtd;
	dtd->dtd_dma = *dma;

	/* initialize buffer page pointers */
	buf_ptr = (u32)(req->req.dma + req->req.actual);
	for (i = 0; i < 5; i++)
		dtd->dtd_buf[i] = cpu_to_le32(buf_ptr + i * PAGE_SIZE);

	req->req.actual += *length;

	/* fill in total bytes with transfer size */
	dtd->dtd_total = cpu_to_le16(*length);
	dev_vdbg(&dev->pdev->dev, "dtd->dtd_total = %d\n", dtd->dtd_total);

	/* set is_last flag if req->req.zero is set or not */
	if (req->req.zero) {
		if (*length == 0 || (*length % req->ep->ep.maxpacket) != 0)
			*is_last = 1;
		else
			*is_last = 0;
	} else if (req->req.length == req->req.actual) {
		*is_last = 1;
	} else
		*is_last = 0;

	if (*is_last == 0)
		dev_vdbg(&dev->pdev->dev, "multi-dtd request!\n");

	/* set interrupt on complete bit for the last dTD */
	if (*is_last && !req->req.no_interrupt)
		dtd->dtd_ioc = 1;

	/* set multiplier override 0 for non-ISO and non-TX endpoint */
	dtd->dtd_multo = 0;

	/* set the active bit of status field to 1 */
	dtd->dtd_status = DTD_STS_ACTIVE;
	dev_vdbg(&dev->pdev->dev, "dtd->dtd_status = 0x%02x\n",
			dtd->dtd_status);

	dev_vdbg(&dev->pdev->dev, "length = %d, dma addr= 0x%08x\n",
			*length, (int)*dma);
	dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
	return dtd;
}


/* generate dTD linked list for a request */
static int req_to_dtd(struct langwell_request *req)
{
	unsigned		count;
	int			is_last, is_first = 1;
	struct langwell_dtd	*dtd, *last_dtd = NULL;
	struct langwell_udc	*dev;
	dma_addr_t		dma;

	dev = req->ep->dev;
	dev_vdbg(&dev->pdev->dev, "---> %s()\n", __func__);
	do {
		dtd = build_dtd(req, &count, &dma, &is_last);
		if (dtd == NULL)
			return -ENOMEM;

		if (is_first) {
			is_first = 0;
			req->head = dtd;
		} else {
			last_dtd->dtd_next = cpu_to_le32(dma);
			last_dtd->next_dtd_virt = dtd;
		}
		last_dtd = dtd;
		req->dtd_count++;
	} while (!is_last);

	/* set terminate bit to 1 for the last dTD */
	dtd->dtd_next = DTD_TERM;

	req->tail = dtd;

	dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
	return 0;
}

/*-------------------------------------------------------------------------*/

/* queue (submits) an I/O requests to an endpoint */
static int langwell_ep_queue(struct usb_ep *_ep, struct usb_request *_req,
		gfp_t gfp_flags)
{
	struct langwell_request	*req;
	struct langwell_ep	*ep;
	struct langwell_udc	*dev;
	unsigned long		flags;
	int			is_iso = 0, in = 0;

	if (unlikely(!_ep || !_req))
		return -EINVAL;
	/* always require a cpu-view buffer */
	req = container_of(_req, struct langwell_request, req);
	ep = container_of(_ep, struct langwell_ep, ep);

	if (!_req->complete || !_req->buf
			|| !list_empty(&req->queue)) {
		return -EINVAL;
	}

	dev = ep->dev;
	req->ep = ep;
	dev_vdbg(&dev->pdev->dev, "---> %s()\n", __func__);

	/* Put all accesses to ep->desc here.
	*
	* ep->desc can be disabled by interrupt handler
	* ( i.e. ep->desc == NULL ) ,
	* but it's safe to access ep->desc if we lock it first.
	*/
	spin_lock_irqsave(&dev->lock, flags);
	if (!ep->desc) {
		spin_unlock_irqrestore(&dev->lock, flags);
		dev_err(&dev->pdev->dev, "ep disabled while enqueuing a request\n");
		return -EINVAL;
	}

	if (usb_endpoint_xfer_isoc(ep->desc)) {
		if (req->req.length > ep->ep.maxpacket) {
			spin_unlock_irqrestore(&dev->lock, flags);
			return -EMSGSIZE;
		}
		is_iso = 1;
	}
	in = is_in(ep);

	spin_unlock_irqrestore(&dev->lock, flags);

	if (unlikely(!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN))
		return -ESHUTDOWN;

	/* set up dma mapping in case the caller didn't */
	if (_req->dma == DMA_ADDR_INVALID) {
		if (_req->length) {
			_req->dma = dma_map_single(&dev->pdev->dev,
				_req->buf, _req->length,
				in ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
			req->mapped = 1;
			dev_vdbg(&dev->pdev->dev, "req->mapped = 1\n");
		} else
			req->mapped = 0;
	} else {
		dma_sync_single_for_device(&dev->pdev->dev,
				_req->dma, _req->length,
				in ?  DMA_TO_DEVICE : DMA_FROM_DEVICE);
		req->mapped = 0;
		dev_vdbg(&dev->pdev->dev, "req->mapped = 0\n");
	}

	dev_dbg(&dev->pdev->dev,
			"%s queue req %p, len %u, buf %p, dma 0x%08x\n",
			_ep->name,
			_req, _req->length, _req->buf, (int)_req->dma);

	_req->status = -EINPROGRESS;
	_req->actual = 0;
	req->dtd_count = 0;

	spin_lock_irqsave(&dev->lock, flags);

	/* Needs to check ep->desc again, ep might be disabled
	* during lock is not held
	*/
	if (!ep->desc) {
		spin_unlock_irqrestore(&dev->lock, flags);
		/* TODO:unmap req buffer if it was mapped */
		dev_err(&dev->pdev->dev, "ep disabled while enqueuing a request\n");
		return -EINVAL;
	}

	/* build and put dTDs to endpoint queue */
	if (!req_to_dtd(req)) {
		queue_dtd(ep, req);
	} else {
		spin_unlock_irqrestore(&dev->lock, flags);
		return -ENOMEM;
	}

	/* update ep0 state */
	if (ep->ep_num == 0)
		dev->ep0_state = DATA_STATE_XMIT;

	if (likely(req != NULL)) {
		list_add_tail(&req->queue, &ep->queue);
		dev_vdbg(&dev->pdev->dev, "list_add_tail()\n");
	}

	spin_unlock_irqrestore(&dev->lock, flags);

	dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
	return 0;
}


/* dequeue (cancels, unlinks) an I/O request from an endpoint */
static int langwell_ep_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct langwell_ep	*ep;
	struct langwell_udc	*dev;
	struct langwell_request	*req;
	unsigned long		flags;
	int			stopped, ep_num, retval = 0;
	u32			endptctrl;

	if (!_ep || !_req)
		return -EINVAL;

	ep = container_of(_ep, struct langwell_ep, ep);
	dev = ep->dev;
	dev_vdbg(&dev->pdev->dev, "---> %s()\n", __func__);

	if (!dev->got_irq)
		return -ENODEV;

	if (!ep->desc)
		return -EINVAL;

	if (!dev->driver)
		return -ESHUTDOWN;

	spin_lock_irqsave(&dev->lock, flags);
	stopped = ep->stopped;

	/* quiesce dma while we patch the queue */
	ep->stopped = 1;
	ep_num = ep->ep_num;

	/* disable endpoint control register */
	if (ep->desc) {
		endptctrl = readl(&dev->op_regs->endptctrl[ep_num]);
		if (is_in(ep))
			endptctrl &= ~EPCTRL_TXE;
		else
			endptctrl &= ~EPCTRL_RXE;
		writel(endptctrl, &dev->op_regs->endptctrl[ep_num]);
	}

	/* make sure it's still queued on this endpoint */
	list_for_each_entry(req, &ep->queue, queue) {
		if (&req->req == _req)
			break;
	}

	if (&req->req != _req) {
		retval = -EINVAL;
		goto done;
	}

	/* queue head may be partially complete. */
	if (ep->queue.next == &req->queue) {
		dev_dbg(&dev->pdev->dev, "unlink (%s) dma\n", _ep->name);
		_req->status = -ECONNRESET;
		langwell_ep_fifo_flush(&ep->ep);

		/* not the last request in endpoint queue */
		if (ep->queue.prev != &req->queue) {
			struct langwell_dqh	*dqh;
			struct langwell_request	*next_req;

			dqh = ep->dqh;
			next_req = list_entry(req->queue.next,
					struct langwell_request, queue);

			/* point the dQH to the first dTD of next request */
		}
	} else {
		struct langwell_request	*prev_req;

		prev_req = list_entry(req->queue.prev,
				struct langwell_request, queue);
		writel(readl(&req->tail->dtd_next),
				&prev_req->tail->dtd_next);
	}

	done(ep, req, -ECONNRESET);

done:
	/* enable endpoint again */
	if (ep->desc) {
		endptctrl = readl(&dev->op_regs->endptctrl[ep_num]);
		if (is_in(ep))
			endptctrl |= EPCTRL_TXE;
		else
			endptctrl |= EPCTRL_RXE;
		writel(endptctrl, &dev->op_regs->endptctrl[ep_num]);
	}

	ep->stopped = stopped;
	spin_unlock_irqrestore(&dev->lock, flags);

	dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
	return retval;
}


/*-------------------------------------------------------------------------*/

/* endpoint set/clear halt */
static void ep_set_halt(struct langwell_ep *ep, int value)
{
	u32			endptctrl = 0;
	int			ep_num;
	struct langwell_udc	*dev = ep->dev;
	dev_vdbg(&dev->pdev->dev, "---> %s()\n", __func__);

	ep_num = ep->ep_num;
	endptctrl = readl(&dev->op_regs->endptctrl[ep_num]);

	/* value: 1 - set halt, 0 - clear halt */
	if (value) {
		/* set the stall bit */
		if (is_in(ep))
			endptctrl |= EPCTRL_TXS;
		else
			endptctrl |= EPCTRL_RXS;
	} else {
		/* clear the stall bit and reset data toggle */
		if (is_in(ep)) {
			endptctrl &= ~EPCTRL_TXS;
			endptctrl |= EPCTRL_TXR;
		} else {
			endptctrl &= ~EPCTRL_RXS;
			endptctrl |= EPCTRL_RXR;
		}
	}

	writel(endptctrl, &dev->op_regs->endptctrl[ep_num]);

	dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
}


/* set the endpoint halt feature */
static int _langwell_ep_set_halt(struct usb_ep *_ep, int value, int chq)
{
	struct langwell_ep	*ep;
	struct langwell_udc	*dev;
	unsigned long		flags;
	int			retval = 0;

	if (!_ep)
		return -EINVAL;

	ep = container_of(_ep, struct langwell_ep, ep);
	dev = ep->dev;
	dev_vdbg(&dev->pdev->dev, "---> %s()\n", __func__);

	if (!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN)
		return -ESHUTDOWN;

	spin_lock_irqsave(&dev->lock, flags);
	if (!ep->desc) {
		retval = -EINVAL;
		goto done;
	}
	if (usb_endpoint_xfer_isoc(ep->desc)) {
		retval = -EOPNOTSUPP;
		goto done;
	}

	/*
	 * attempt to halt IN ep will fail if any transfer requests
	 * are still queue
	 */
	if (chq && !list_empty(&ep->queue) && is_in(ep) && value) {
		/* IN endpoint FIFO holds bytes */
		dev_dbg(&dev->pdev->dev, "%s FIFO holds bytes\n", _ep->name);
		retval = -EAGAIN;
		goto done;
	}

	/* endpoint set/clear halt */
	if (ep->ep_num) {
		ep_set_halt(ep, value);
	} else { /* endpoint 0 */
		dev->ep0_state = WAIT_FOR_SETUP;
		dev->ep0_dir = USB_DIR_OUT;
	}
done:
	spin_unlock_irqrestore(&dev->lock, flags);

	dev_dbg(&dev->pdev->dev, "%s %s halt\n",
			_ep->name, value ? "set" : "clear");
	dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
	return retval;
}

static int langwell_ep_set_halt(struct usb_ep *_ep, int value)
{
	return _langwell_ep_set_halt(_ep, value, 1);
}

/* set the halt feature and ignores clear requests */
static int langwell_ep_set_wedge(struct usb_ep *_ep)
{
	struct langwell_ep	*ep;
	struct langwell_udc	*dev;

	if (!_ep)
		return -EINVAL;

	ep = container_of(_ep, struct langwell_ep, ep);
	dev = ep->dev;
	dev_vdbg(&dev->pdev->dev, "---> %s()\n", __func__);

	if (!ep->desc)
		return -EINVAL;

	dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
	return usb_ep_set_halt(_ep);
}

/*
 * Enter or exit PHY low power state. If it changed
 * PHY status, return 1 else return 0.
 */
int langwell_phy_low_power(struct langwell_udc *dev, bool flag)
{
	u32		devlc;
	u8		devlc_byte2;
	dev_dbg(&dev->pdev->dev, "---> %s()\n", __func__);

	/*FIXME: get back to d0 first? */

	devlc = readl(&dev->op_regs->devlc);

	/*
	 * If PHY already in or out low power state,
	 * exit func and return 0
	 */
	if ((devlc & LPM_PHCD) == (flag ? LPM_PHCD : 0)) {
		dev_dbg(&dev->pdev->dev, "<--- %s()\n", __func__);
		return 0;
	}
	dev_vdbg(&dev->pdev->dev, "devlc = 0x%08x\n", devlc);

	if (flag)
		devlc |= LPM_PHCD;
	else
		devlc &= ~LPM_PHCD;

	/* FIXME: workaround for Langwell A1/A2/A3 sighting */
	devlc_byte2 = (devlc >> 16) & 0xff;
	writeb(devlc_byte2, (u8 *)&dev->op_regs->devlc + 2);

	devlc = readl(&dev->op_regs->devlc);
	dev_vdbg(&dev->pdev->dev,
		 "%s PHY low power suspend, devlc = 0x%08x\n",
		 flag ? "enter" : "exit", devlc);
	dev_dbg(&dev->pdev->dev, "<--- %s()\n", __func__);
	return 1;
}
EXPORT_SYMBOL(langwell_phy_low_power);

/*Switch PHY power mode. If it is changed, delay some millisecond*/
static inline void langwell_phy_low_power_delay(struct langwell_udc *dev,
						bool flag, unsigned long late)
{
	if (langwell_phy_low_power(dev, flag))
			mdelay(late);
}


/* flush contents of a fifo */
static void langwell_ep_fifo_flush(struct usb_ep *_ep)
{
	struct langwell_ep	*ep;
	struct langwell_udc	*dev;
	u32			flush_bit;
	unsigned long		timeout;

	if (!_ep)
		return;

	ep = container_of(_ep, struct langwell_ep, ep);
	dev = ep->dev;

	dev_vdbg(&dev->pdev->dev, "---> %s()\n", __func__);

	if (!ep->desc) {
		dev_vdbg(&dev->pdev->dev, "ep->desc is NULL\n");
		dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
		return;
	}

	dev_vdbg(&dev->pdev->dev, "%s-%s fifo flush\n",
			_ep->name, DIR_STRING(ep));

	/*FIXME: get back to d0 first? */

	/* delay 3 millisecond to wait for phy exiting low power mode,
	   otherwise it easy to cause fabric error. the function can't use
	   msleep in here, because composite device will call this function with
	   holding spin lock. although it call pm_runtime_get_sync before which
	   would cause schedule too, but composite device would call
	   pm_runtime_get_sync prior to this function, so hereby it would
	   not cause schedule, just count plug one.
	*/
	langwell_phy_low_power_delay(dev, 0, 3);

	/* flush endpoint buffer */
	if (ep->ep_num == 0)
		flush_bit = (1 << 16) | 1;
	else if (is_in(ep))
		flush_bit = 1 << (ep->ep_num + 16);	/* TX */
	else
		flush_bit = 1 << ep->ep_num;		/* RX */

	/* wait until flush complete */
	timeout = 100 * FLUSH_TIMEOUT;
	do {
		writel(flush_bit, &dev->op_regs->endptflush);
		while (readl(&dev->op_regs->endptflush)) {
			if (--timeout == 0) {
				dev_err(&dev->pdev->dev, "ep flush timeout\n");
				goto done;
			}
			udelay(10);
		}
	} while (readl(&dev->op_regs->endptstat) & flush_bit);
done:

	dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
}


/* endpoints operations structure */
static const struct usb_ep_ops langwell_ep_ops = {

	/* configure endpoint, making it usable */
	.enable		= langwell_ep_enable,

	/* endpoint is no longer usable */
	.disable	= langwell_ep_disable,

	/* allocate a request object to use with this endpoint */
	.alloc_request	= langwell_alloc_request,

	/* free a request object */
	.free_request	= langwell_free_request,

	/* queue (submits) an I/O requests to an endpoint */
	.queue		= langwell_ep_queue,

	/* dequeue (cancels, unlinks) an I/O request from an endpoint */
	.dequeue	= langwell_ep_dequeue,

	/* set the endpoint halt feature */
	.set_halt	= langwell_ep_set_halt,

	/* set the halt feature and ignores clear requests */
	.set_wedge	= langwell_ep_set_wedge,

	/* flush contents of a fifo */
	.fifo_flush	= langwell_ep_fifo_flush,
};


/*-------------------------------------------------------------------------*/

/* device controller usb_gadget_ops structure */

static inline int can_pullup(struct langwell_udc *dev)
{
	return dev->driver && dev->softconnected &&
		 dev->vbus_active && !dev->stopped;
}

static void langwell_udc_pullup(struct langwell_udc *dev, u32 on)
{
	u32 usbcmd;
	u32 usbmode;

	if (dev->stopped)
		return;

	usbmode = readl(&dev->op_regs->usbmode);
	usbcmd = readl(&dev->op_regs->usbcmd);
	if (!(usbcmd & CMD_RUNSTOP) && on) {
		if (can_pullup(dev)) {
			if (dev->sdis) {
				usbmode |= MODE_SDIS;
				dev_info(&dev->pdev->dev, "disable streaming mode\n");
			} else {
				usbmode &= ~MODE_SDIS;
				dev_info(&dev->pdev->dev, "enable streaming mode\n");
			}
			writel(usbmode, &dev->op_regs->usbmode);
			usbcmd |= CMD_RUNSTOP;
			writel(usbcmd, &dev->op_regs->usbcmd);
		}
	} else if ((usbcmd & CMD_RUNSTOP) && !on) {

#ifdef	OTG_TRANSCEIVER
		/*BZ28614 WA:
		* Tell PHY to transit to FS before clearing RS bit,
		* this avoid PHY getting hang during the disconnect.
		*/
		if (is_clovertrail(dev->pdev)) {
			if (pnw_otg_ulpi_write(ULPI_FUNCTRL, 0x65))
				dev_err(&dev->pdev->dev,
				"w/a 28614 ulpi write fail\n");
		}
#endif
		usbcmd &= ~CMD_RUNSTOP;
		writel(usbcmd, &dev->op_regs->usbcmd);
	}
}

/* returns the current frame number */
static int langwell_get_frame(struct usb_gadget *_gadget)
{
	struct langwell_udc	*dev;
	u16			retval;

	if (!_gadget)
		return -ENODEV;

	dev = container_of(_gadget, struct langwell_udc, gadget);
	dev_vdbg(&dev->pdev->dev, "---> %s()\n", __func__);

	retval = readl(&dev->op_regs->frindex) & FRINDEX_MASK;

	dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
	return retval;
}


/* tries to wake up the host connected to this gadget */
static int langwell_wakeup(struct usb_gadget *_gadget)
{
	struct langwell_udc	*dev;
	u32			portsc1;
	unsigned long		flags;

	if (!_gadget)
		return 0;

	dev = container_of(_gadget, struct langwell_udc, gadget);
	dev_vdbg(&dev->pdev->dev, "---> %s()\n", __func__);

	/* remote wakeup feature not enabled by host */
	if (!dev->remote_wakeup) {
		dev_info(&dev->pdev->dev, "remote wakeup is disabled\n");
		return -ENOTSUPP;
	}

	spin_lock_irqsave(&dev->lock, flags);

	portsc1 = readl(&dev->op_regs->portsc1);
	if (!(portsc1 & PORTS_SUSP)) {
		spin_unlock_irqrestore(&dev->lock, flags);
		return 0;
	}

	/* LPM L1 to L0 or legacy remote wakeup */
	if (dev->lpm && dev->lpm_state == LPM_L1)
		dev_info(&dev->pdev->dev, "LPM L1 to L0 remote wakeup\n");
	else
		dev_info(&dev->pdev->dev, "device remote wakeup\n");

	/* force port resume */
	portsc1 |= PORTS_FPR;
	writel(portsc1, &dev->op_regs->portsc1);

	spin_unlock_irqrestore(&dev->lock, flags);

	dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
	return 0;
}


/* notify controller that VBUS is powered or not */
static int langwell_vbus_session(struct usb_gadget *_gadget, int is_active)
{
	struct langwell_udc	*dev;
	unsigned long		flags;

	if (!_gadget)
		return -ENODEV;

	dev = container_of(_gadget, struct langwell_udc, gadget);
	dev_vdbg(&dev->pdev->dev, "---> %s()\n", __func__);

	spin_lock_irqsave(&dev->lock, flags);
	dev_vdbg(&dev->pdev->dev, "VBUS status: %s\n",
			is_active ? "on" : "off");

	if (dev->vbus_active == !!is_active)
		goto done;

	dev->vbus_active = !!is_active;

	if (is_active)
		langwell_udc_pullup(dev, 1);
	else
		langwell_udc_pullup(dev, 0);

done:
	spin_unlock_irqrestore(&dev->lock, flags);

	dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
	return 0;
}


/* constrain controller's VBUS power usage */
static int langwell_vbus_draw(struct usb_gadget *_gadget, unsigned mA)
{
	struct langwell_udc	*dev;

	if (!_gadget)
		return -ENODEV;

	dev = container_of(_gadget, struct langwell_udc, gadget);
	dev_vdbg(&dev->pdev->dev, "---> %s()\n", __func__);

	if (dev->transceiver) {
		dev_vdbg(&dev->pdev->dev, "usb_phy_set_power\n");
		dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
		return usb_phy_set_power(dev->transceiver, mA);
	}

	dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
	return -ENOTSUPP;
}


/* D+ pullup, software-controlled connect/disconnect to USB host */
static int langwell_pullup(struct usb_gadget *_gadget, int is_on)
{
	struct langwell_udc	*dev;
	unsigned long		flags;

	if (!_gadget)
		return -ENODEV;

	dev = container_of(_gadget, struct langwell_udc, gadget);

	dev_vdbg(&dev->pdev->dev, "---> %s()\n", __func__);

	spin_lock_irqsave(&dev->lock, flags);
	if (dev->softconnected == !!is_on)
		goto done;
	dev->softconnected = !!is_on;

	if (is_on)
		langwell_udc_pullup(dev, 1);
	else
		langwell_udc_pullup(dev, 0);

done:
	spin_unlock_irqrestore(&dev->lock, flags);

	dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
	return 0;
}

static int langwell_start(struct usb_gadget *g,
		struct usb_gadget_driver *driver);

static int langwell_stop(struct usb_gadget *g,
		struct usb_gadget_driver *driver);

/* device controller usb_gadget_ops structure */
static const struct usb_gadget_ops langwell_ops = {

	/* returns the current frame number */
	.get_frame	= langwell_get_frame,

	/* tries to wake up the host connected to this gadget */
	.wakeup		= langwell_wakeup,

	/* set the device selfpowered feature, always selfpowered */
	/* .set_selfpowered = langwell_set_selfpowered, */

	/* notify controller that VBUS is powered or not */
	.vbus_session	= langwell_vbus_session,

	/* constrain controller's VBUS power usage */
	.vbus_draw	= langwell_vbus_draw,

	/* D+ pullup, software-controlled connect/disconnect to USB host */
	.pullup		= langwell_pullup,

	.udc_start	= langwell_start,
	.udc_stop	= langwell_stop,
};


/*-------------------------------------------------------------------------*/

/* device controller operations */

/* reset device controller */
static int langwell_udc_reset(struct langwell_udc *dev)
{
	u32		usbcmd, usbmode, devlc, endpointlistaddr;
	u8		devlc_byte0, devlc_byte2;
	unsigned long	timeout;
	unsigned long	sbuscfg_addr;

	if (!dev)
		return -EINVAL;

	dev_dbg(&dev->pdev->dev, "---> %s()\n", __func__);

	/* clear devlc's auto low power bit */
	devlc = readl(&dev->op_regs->devlc);
	devlc &= ~LPM_ASUS;
	writel(devlc, &dev->op_regs->devlc);

	/* disconnect pullup */
	langwell_udc_pullup(dev, 0);

	/* reset device controller */
	usbcmd = readl(&dev->op_regs->usbcmd);
	usbcmd |= CMD_RST;
	writel(usbcmd, &dev->op_regs->usbcmd);

	/* wait for reset to complete */
	timeout = 100 * RESET_TIMEOUT;
	while (readl(&dev->op_regs->usbcmd) & CMD_RST) {
		if (--timeout == 0) {
			dev_err(&dev->pdev->dev, "device reset timeout\n");
			return -ETIMEDOUT;
		}
		udelay(10);
	}

	/* change ITC value. ITC field is used to set the maximum rate at which
	   the device controller will issue interrupts. ITC contain the maximum
	   interrupt interval measured in micro-frames. ITC=2 is the smallest
	   available value. The USB network gadget become instability if ITC is
	   set to 1 or 0.
	*/
	usbcmd = readl(&dev->op_regs->usbcmd);
	usbcmd &= ~CMD_SET_ITC(0xff);	/* clear ITC field */
	usbcmd |= CMD_SET_ITC(0x02);	/* set ITC field */
	writel(usbcmd, &dev->op_regs->usbcmd);

	/* set controller to device mode */
	usbmode = readl(&dev->op_regs->usbmode);
	usbmode |= MODE_DEVICE;

	/* turn setup lockout off, require setup tripwire in usbcmd */
	usbmode |= MODE_SLOM;

	writel(usbmode, &dev->op_regs->usbmode);
	usbmode = readl(&dev->op_regs->usbmode);
	dev_vdbg(&dev->pdev->dev, "usbmode=0x%08x\n", usbmode);

	/* Write-Clear setup status */
	writel(0, &dev->op_regs->usbsts);

	/* FIXME: workaround for bug9000364367 */
	sbuscfg_addr = (unsigned long)dev->cap_regs + SBUSCFG_REG_OFFSET;
	writel(0x7, (void *)sbuscfg_addr);

	/* if support USB LPM, ACK all LPM token */
	if (dev->lpm) {
		devlc = readl(&dev->op_regs->devlc);
		dev_vdbg(&dev->pdev->dev, "devlc = 0x%08x\n", devlc);
		/* FIXME: workaround for Langwell A1/A2/A3 sighting */
		devlc &= ~LPM_STL;	/* don't STALL LPM token */
		devlc &= ~LPM_NYT_ACK;	/* ACK LPM token */
		devlc_byte0 = devlc & 0xff;
		devlc_byte2 = (devlc >> 16) & 0xff;
		writeb(devlc_byte0, (u8 *)&dev->op_regs->devlc);
		writeb(devlc_byte2, (u8 *)&dev->op_regs->devlc + 2);
		devlc = readl(&dev->op_regs->devlc);
		dev_vdbg(&dev->pdev->dev,
				"ACK LPM token, devlc = 0x%08x\n", devlc);
	}

	/* fill endpointlistaddr register */
	endpointlistaddr = dev->ep_dqh_dma;
	endpointlistaddr &= ENDPOINTLISTADDR_MASK;
	writel(endpointlistaddr, &dev->op_regs->endpointlistaddr);

	dev_vdbg(&dev->pdev->dev,
		"dQH base (vir: %p, phy: 0x%08x), endpointlistaddr=0x%08x\n",
		dev->ep_dqh, endpointlistaddr,
		readl(&dev->op_regs->endpointlistaddr));
	dev_dbg(&dev->pdev->dev, "<--- %s()\n", __func__);
	return 0;
}


/* reinitialize device controller endpoints */
static int eps_reinit(struct langwell_udc *dev)
{
	struct langwell_ep	*ep;
	char			name[14];
	int			i;

	dev_vdbg(&dev->pdev->dev, "---> %s()\n", __func__);

	/* initialize ep0 */
	ep = &dev->ep[0];
	ep->dev = dev;
	strncpy(ep->name, "ep0", sizeof(ep->name));
	ep->ep.name = ep->name;
	ep->ep.ops = &langwell_ep_ops;
	ep->stopped = 0;
	ep->ep.maxpacket = EP0_MAX_PKT_SIZE;
	ep->ep_num = 0;
	ep->desc = &langwell_ep0_desc;
	INIT_LIST_HEAD(&ep->queue);

	ep->ep_type = USB_ENDPOINT_XFER_CONTROL;

	/* initialize other endpoints */
	for (i = 2; i < dev->ep_max; i++) {
		ep = &dev->ep[i];
		if (i % 2)
			snprintf(name, sizeof(name), "ep%din", i / 2);
		else
			snprintf(name, sizeof(name), "ep%dout", i / 2);
		ep->dev = dev;
		strncpy(ep->name, name, sizeof(ep->name));
		ep->ep.name = ep->name;

		ep->ep.ops = &langwell_ep_ops;
		ep->stopped = 0;
		ep->ep.maxpacket = (unsigned short) ~0;
		ep->ep_num = i / 2;

		INIT_LIST_HEAD(&ep->queue);
		list_add_tail(&ep->ep.ep_list, &dev->gadget.ep_list);
	}

	dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
	return 0;
}


/* enable interrupt and set controller to run state */
static void langwell_udc_start(struct langwell_udc *dev)
{
	u32	usbintr;
	dev_dbg(&dev->pdev->dev, "---> %s()\n", __func__);

	/* enable interrupts */
	usbintr = INTR_SLE	/* suspend */
		/* | INTR_ULPIE	 ULPI */
		/* | INTR_SRE	SOF received */
		| INTR_URE	/* USB reset */
		| INTR_SEE	/* system error */
		| INTR_PCE	/* port change detect */
		| INTR_UEE	/* USB error interrupt */
		| INTR_UE;	/* USB interrupt */
	writel(usbintr, &dev->op_regs->usbintr);

	/* clear stopped bit */
	dev->stopped = 0;

	/* Connect pullup if we can */
	langwell_udc_pullup(dev, 1);

	dev_dbg(&dev->pdev->dev, "<--- %s()\n", __func__);
}

/* disable test mode */
static void langwell_udc_stop_testmode(struct langwell_udc *dev)
{
	u32	portsc1;

	dev_dbg(&dev->pdev->dev, "---> %s()\n", __func__);

	portsc1 = readl(&dev->op_regs->portsc1);

	if (PORTS_PTC(portsc1))	{
		dev_info(&dev->pdev->dev, "Exit USB Test mode\n");
		portsc1 &= ~PORTS_PTC_MASK;
		writel(portsc1, &dev->op_regs->portsc1);
	}

	langwell_udc_notify_otg(
			MID_OTG_NOTIFY_TEST_MODE_STOP);

	dev_dbg(&dev->pdev->dev, "<--- %s()\n", __func__);
	return;
}

/* disable interrupt and set controller to stop state */
static void langwell_udc_stop(struct langwell_udc *dev)
{
	dev_dbg(&dev->pdev->dev, "---> %s()\n", __func__);

	/* disconnect pullup */
	langwell_udc_pullup(dev, 0);

	/* disable all interrupts */
	writel(0, &dev->op_regs->usbintr);

	/* set stopped bit */
	dev->stopped = 1;

	dev_dbg(&dev->pdev->dev, "<--- %s()\n", __func__);
}


/* stop all USB activities */
static void stop_activity(struct langwell_udc *dev,
			  struct usb_gadget_driver *driver)
{
	struct langwell_ep	*ep;
	dev_dbg(&dev->pdev->dev, "---> %s()\n", __func__);

	/* report disconnect; the driver is already quiesced */
	if (dev->driver) {
		spin_unlock(&dev->lock);
		dev->driver->disconnect(&dev->gadget);
		spin_lock(&dev->lock);
	}

	nuke(&dev->ep[0], -ESHUTDOWN);

	list_for_each_entry(ep, &dev->gadget.ep_list, ep.ep_list) {
		nuke(ep, -ESHUTDOWN);
	}

	dev_dbg(&dev->pdev->dev, "<--- %s()\n", __func__);
}


/*-------------------------------------------------------------------------*/

/* device "function" sysfs attribute file */
static ssize_t show_function(struct device *_dev,
		struct device_attribute *attr, char *buf)
{
	struct langwell_udc	*dev = dev_get_drvdata(_dev);

	if (!dev->driver || !dev->driver->function
			|| strlen(dev->driver->function) > PAGE_SIZE)
		return 0;

	return scnprintf(buf, PAGE_SIZE, "%s\n", dev->driver->function);
}
static DEVICE_ATTR(function, S_IRUGO, show_function, NULL);


static inline enum usb_device_speed lpm_device_speed(u32 reg)
{
	switch (LPM_PSPD(reg)) {
	case LPM_SPEED_HIGH:
		return USB_SPEED_HIGH;
	case LPM_SPEED_FULL:
		return USB_SPEED_FULL;
	case LPM_SPEED_LOW:
		return USB_SPEED_LOW;
	default:
		return USB_SPEED_UNKNOWN;
	}
}


#define lnw_scnprintf(next, size, fmt, args...) do {\
	unsigned t;\
	t = scnprintf((next), (size), fmt, ## args);\
	(size) -= t;\
	(next) += t;\
	} while (0)

static void langwell_dump_registers(struct langwell_udc *dev,
				char **next, unsigned *size)
{
	int i;

	lnw_scnprintf(*next, *size, STRING_LNW_REGS(dev));

	for (i = 0; i < dev->ep_max / 2; i++) {
		lnw_scnprintf(*next, *size, "endptctrl[%d]=0x%08x\n",
				i, readl(&dev->op_regs->endptctrl[i]));
	}

}

static void langwell_dump_ep(struct langwell_udc *dev, int ep_index,
				char **next, unsigned *size)
{
	struct langwell_ep *ep = &dev->ep[ep_index];
	struct langwell_dqh *dqh = &dev->ep_dqh[ep_index];
	struct langwell_request *req;
	struct langwell_dtd *dtd;
	int         i, j;

	lnw_scnprintf(*next, *size,
		"<EP %d-%s>\n", ep->ep_num, ep_index % 2 ? "in" : "out");
	lnw_scnprintf(*next, *size, "[dQH]\n");

	for (i = 0; i < DQH_DW_SIZE; i++)
		lnw_scnprintf(*next, *size,
			"\tDW%02d: 0x%08x\n", i, *((u32 *)dqh + i));

	lnw_scnprintf(*next, *size, "[Request & dTD]\n");
	/* ep1's requests are managed in ep0 too */
	if (ep_index == 1)
		ep = &dev->ep[0];

	list_for_each_entry(req, &ep->queue, queue) {

		lnw_scnprintf(*next, *size,
			"req %p actual 0x%x length 0x%x "
			"buf 0x%p (dma addr 0x%llx)\n",
			&req->req, req->req.actual,
			req->req.length, req->req.buf, req->req.dma);

		dtd = req->head;
		for (i = 0; i < req->dtd_count; i++) {
			lnw_scnprintf(*next, *size,
				"\tdTD %d virt_addr 0x%p, dma addr 0x%08llx\n",
				i, dtd, dtd->dtd_dma);

			for (j = 0; j < DTD_DW_SIZE; j++)
				lnw_scnprintf(*next, *size, "\tDW%d: 0x%08x\n",
					j, *((u32 *)dtd + j));

			if (i != req->dtd_count - 1)
				dtd = dtd->next_dtd_virt;
		}
	}
}

/** show_langwell_snapt - shows a snapshot of HW & SW.
 *
 * 1. Dump of all registers
 * 2. For each enabled endpoint, dump the following info:
 *  a). all requests queued: request addr, request buffer addr,
 *      request buffer len.
 *  b). dQH and all dTDs linked.
 */
static ssize_t show_langwell_snapshot(struct device *_dev,
			struct device_attribute *attr, char *buf)
{
	struct langwell_udc *dev = the_controller;
	struct langwell_ep  *ep;
	char            *next;
	unsigned        size;
	unsigned long       flags;
	int i;

	next = buf;
	size = PAGE_SIZE;

	if (!dev->got_irq) {
		lnw_scnprintf(next, size,
			"langwell snapshot not available. Not connected?\n");
		goto out;
	}

	pm_runtime_get_sync(&dev->pdev->dev);
	spin_lock_irqsave(&dev->lock, flags);
	langwell_dump_registers(dev, &next, &size);
	for (i = 0; i < dev->ep_max; i++) {
		ep = &dev->ep[i];
		if (ep->desc || i == 1)
			langwell_dump_ep(dev, i, &next, &size);
	}
	spin_unlock_irqrestore(&dev->lock, flags);
	pm_runtime_put(&dev->pdev->dev);

out:
	return PAGE_SIZE - size;
}
static DEVICE_ATTR(langwell_snapshot, S_IRUGO, show_langwell_snapshot, NULL);

/* device "langwell_udc" sysfs attribute file */
static ssize_t show_langwell_udc(struct device *_dev,
		struct device_attribute *attr, char *buf)
{
	struct langwell_udc	*dev = dev_get_drvdata(_dev);
	struct langwell_request *req;
	struct langwell_ep	*ep = NULL;
	char			*next;
	unsigned		size;
	unsigned		t;
	unsigned		i;
	unsigned long		flags;
	u32			tmp_reg;

	next = buf;
	size = PAGE_SIZE;
	spin_lock_irqsave(&dev->lock, flags);

	/* driver basic information */
	t = scnprintf(next, size,
			DRIVER_DESC "\n"
			"%s version: %s\n"
			"Gadget driver: %s\n\n",
			driver_name, DRIVER_VERSION,
			dev->driver ? dev->driver->driver.name : "(none)");
	size -= t;
	next += t;

	/* device registers */
	tmp_reg = readl(&dev->op_regs->usbcmd);
	t = scnprintf(next, size,
			"USBCMD reg:\n"
			"SetupTW: %d\n"
			"Run/Stop: %s\n\n",
			(tmp_reg & CMD_SUTW) ? 1 : 0,
			(tmp_reg & CMD_RUNSTOP) ? "Run" : "Stop");
	size -= t;
	next += t;

	tmp_reg = readl(&dev->op_regs->usbsts);
	t = scnprintf(next, size,
			"USB Status Reg:\n"
			"Device Suspend: %d\n"
			"Reset Received: %d\n"
			"System Error: %s\n"
			"USB Error Interrupt: %s\n\n",
			(tmp_reg & STS_SLI) ? 1 : 0,
			(tmp_reg & STS_URI) ? 1 : 0,
			(tmp_reg & STS_SEI) ? "Error" : "No error",
			(tmp_reg & STS_UEI) ? "Error detected" : "No error");
	size -= t;
	next += t;

	tmp_reg = readl(&dev->op_regs->usbintr);
	t = scnprintf(next, size,
			"USB Intrrupt Enable Reg:\n"
			"Sleep Enable: %d\n"
			"SOF Received Enable: %d\n"
			"Reset Enable: %d\n"
			"System Error Enable: %d\n"
			"Port Change Dectected Enable: %d\n"
			"USB Error Intr Enable: %d\n"
			"USB Intr Enable: %d\n\n",
			(tmp_reg & INTR_SLE) ? 1 : 0,
			(tmp_reg & INTR_SRE) ? 1 : 0,
			(tmp_reg & INTR_URE) ? 1 : 0,
			(tmp_reg & INTR_SEE) ? 1 : 0,
			(tmp_reg & INTR_PCE) ? 1 : 0,
			(tmp_reg & INTR_UEE) ? 1 : 0,
			(tmp_reg & INTR_UE) ? 1 : 0);
	size -= t;
	next += t;

	tmp_reg = readl(&dev->op_regs->frindex);
	t = scnprintf(next, size,
			"USB Frame Index Reg:\n"
			"Frame Number is 0x%08x\n\n",
			(tmp_reg & FRINDEX_MASK));
	size -= t;
	next += t;

	tmp_reg = readl(&dev->op_regs->deviceaddr);
	t = scnprintf(next, size,
			"USB Device Address Reg:\n"
			"Device Addr is 0x%x\n\n",
			USBADR(tmp_reg));
	size -= t;
	next += t;

	tmp_reg = readl(&dev->op_regs->endpointlistaddr);
	t = scnprintf(next, size,
			"USB Endpoint List Address Reg:\n"
			"Endpoint List Pointer is 0x%x\n\n",
			EPBASE(tmp_reg));
	size -= t;
	next += t;

	tmp_reg = readl(&dev->op_regs->portsc1);
	t = scnprintf(next, size,
		"USB Port Status & Control Reg:\n"
		"Port Reset: %s\n"
		"Port Suspend Mode: %s\n"
		"Over-current Change: %s\n"
		"Port Enable/Disable Change: %s\n"
		"Port Enabled/Disabled: %s\n"
		"Current Connect Status: %s\n"
		"LPM Suspend Status: %s\n\n",
		(tmp_reg & PORTS_PR) ? "Reset" : "Not Reset",
		(tmp_reg & PORTS_SUSP) ? "Suspend " : "Not Suspend",
		(tmp_reg & PORTS_OCC) ? "Detected" : "No",
		(tmp_reg & PORTS_PEC) ? "Changed" : "Not Changed",
		(tmp_reg & PORTS_PE) ? "Enable" : "Not Correct",
		(tmp_reg & PORTS_CCS) ?  "Attached" : "Not Attached",
		(tmp_reg & PORTS_SLP) ? "LPM L1" : "LPM L0");
	size -= t;
	next += t;

	tmp_reg = readl(&dev->op_regs->devlc);
	t = scnprintf(next, size,
		"Device LPM Control Reg:\n"
		"Parallel Transceiver : %d\n"
		"Serial Transceiver : %d\n"
		"Port Speed: %s\n"
		"Port Force Full Speed Connenct: %s\n"
		"PHY Low Power Suspend Clock: %s\n"
		"BmAttributes: %d\n\n",
		LPM_PTS(tmp_reg),
		(tmp_reg & LPM_STS) ? 1 : 0,
		usb_speed_string(lpm_device_speed(tmp_reg)),
		(tmp_reg & LPM_PFSC) ? "Force Full Speed" : "Not Force",
		(tmp_reg & LPM_PHCD) ? "Disabled" : "Enabled",
		LPM_BA(tmp_reg));
	size -= t;
	next += t;

	tmp_reg = readl(&dev->op_regs->usbmode);
	t = scnprintf(next, size,
			"USB Mode Reg:\n"
			"Controller Mode is : %s\n\n", ({
				char *s;
				switch (MODE_CM(tmp_reg)) {
				case MODE_IDLE:
					s = "Idle"; break;
				case MODE_DEVICE:
					s = "Device Controller"; break;
				case MODE_HOST:
					s = "Host Controller"; break;
				default:
					s = "None"; break;
				}
				s;
			}));
	size -= t;
	next += t;

	tmp_reg = readl(&dev->op_regs->endptsetupstat);
	t = scnprintf(next, size,
			"Endpoint Setup Status Reg:\n"
			"SETUP on ep 0x%04x\n\n",
			tmp_reg & SETUPSTAT_MASK);
	size -= t;
	next += t;

	for (i = 0; i < dev->ep_max / 2; i++) {
		tmp_reg = readl(&dev->op_regs->endptctrl[i]);
		t = scnprintf(next, size, "EP Ctrl Reg [%d]: 0x%08x\n",
				i, tmp_reg);
		size -= t;
		next += t;
	}
	tmp_reg = readl(&dev->op_regs->endptprime);
	t = scnprintf(next, size, "EP Prime Reg: 0x%08x\n\n", tmp_reg);
	size -= t;
	next += t;

	/* langwell_udc, langwell_ep, langwell_request structure information */
	ep = &dev->ep[0];
	t = scnprintf(next, size, "%s MaxPacketSize: 0x%x, ep_num: %d\n",
			ep->ep.name, ep->ep.maxpacket, ep->ep_num);
	size -= t;
	next += t;

	if (list_empty(&ep->queue)) {
		t = scnprintf(next, size, "its req queue is empty\n\n");
		size -= t;
		next += t;
	} else {
		list_for_each_entry(req, &ep->queue, queue) {
			t = scnprintf(next, size,
				"req %p actual 0x%x length 0x%x  buf %p\n",
				&req->req, req->req.actual,
				req->req.length, req->req.buf);
			size -= t;
			next += t;
		}
	}
	/* other gadget->eplist ep */
	list_for_each_entry(ep, &dev->gadget.ep_list, ep.ep_list) {
		if (ep->desc) {
			t = scnprintf(next, size,
					"\n%s MaxPacketSize: 0x%x, "
					"ep_num: %d\n",
					ep->ep.name, ep->ep.maxpacket,
					ep->ep_num);
			size -= t;
			next += t;

			if (list_empty(&ep->queue)) {
				t = scnprintf(next, size,
						"its req queue is empty\n\n");
				size -= t;
				next += t;
			} else {
				list_for_each_entry(req, &ep->queue, queue) {
					t = scnprintf(next, size,
						"req %p actual 0x%x length "
						"0x%x  buf %p\n",
						&req->req, req->req.actual,
						req->req.length, req->req.buf);
					size -= t;
					next += t;
				}
			}
		}
	}

	spin_unlock_irqrestore(&dev->lock, flags);
	return PAGE_SIZE - size;
}
static DEVICE_ATTR(langwell_udc, S_IRUGO, show_langwell_udc, NULL);

static ssize_t
show_sdis(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct langwell_udc	*udc = the_controller;
	char			*next;
	unsigned		size;
	unsigned		t;

	next = buf;
	size = PAGE_SIZE;

	t = scnprintf(next, size, "Stream mode %s",
			(udc->sdis) ? "off" : "on");

	size -= t;
	next += t;

	return PAGE_SIZE - size;
}

static ssize_t
store_sdis(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct langwell_udc	*udc = the_controller;

	if (count > 2)
		return -1;

	if (buf[0] == '0') {
		udc->sdis = 0;
		dev_dbg(&udc->pdev->dev, "enable Stream mode\n");
	} else if (buf[0] == '1') {
		udc->sdis = 1;
		dev_dbg(&udc->pdev->dev, "disable Stream mode\n");
	}

	return count;
}
static DEVICE_ATTR(sdis, S_IRUGO | S_IWUSR | S_IWGRP, show_sdis, store_sdis);

/* device "remote_wakeup" sysfs attribute file */
static ssize_t store_remote_wakeup(struct device *_dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct langwell_udc	*dev = dev_get_drvdata(_dev);
	unsigned long		flags;
	ssize_t			rc = count;

	if (count > 2)
		return -EINVAL;

	if (count > 0 && buf[count-1] == '\n')
		((char *) buf)[count-1] = 0;

	if (buf[0] != '1')
		return -EINVAL;

	/* force remote wakeup enabled in case gadget driver doesn't support */
	spin_lock_irqsave(&dev->lock, flags);
	dev->remote_wakeup = 1;
	dev->dev_status |= (1 << USB_DEVICE_REMOTE_WAKEUP);
	spin_unlock_irqrestore(&dev->lock, flags);

	langwell_wakeup(&dev->gadget);

	return rc;
}
static DEVICE_ATTR(remote_wakeup, S_IWUSR, NULL, store_remote_wakeup);


/*-------------------------------------------------------------------------*/

/*
 * when a driver is successfully registered, it will receive
 * control requests including set_configuration(), which enables
 * non-control requests.  then usb traffic follows until a
 * disconnect is reported.  then a host may connect again, or
 * the driver might get unbound.
 */

static int langwell_start(struct usb_gadget *g,
		struct usb_gadget_driver *driver)
{
	struct langwell_udc	*dev = gadget_to_langwell(g);
	unsigned long		flags;
	int			retval;

	dev_dbg(&dev->pdev->dev, "---> %s()\n", __func__);

	spin_lock_irqsave(&dev->lock, flags);

	/* hook up the driver ... */
	driver->driver.bus = NULL;
	dev->driver = driver;
	dev->gadget.dev.driver = &driver->driver;

	spin_unlock_irqrestore(&dev->lock, flags);

	retval = device_create_file(&dev->pdev->dev, &dev_attr_function);
	if (retval)
		goto err;

	dev->usb_state = USB_STATE_ATTACHED;
	dev->ep0_state = WAIT_FOR_SETUP;
	dev->ep0_dir = USB_DIR_OUT;

	/* bind OTG transceiver */
	if (dev->transceiver)
		(void)otg_set_peripheral(dev->transceiver->otg, &dev->gadget);

	/* enable interrupt and set controller to run state */
	if (dev->got_irq)
		langwell_udc_start(dev);

	dev_vdbg(&dev->pdev->dev,
			"After langwell_udc_start(), print all registers:\n");
	print_all_registers(dev);

	dev_info(&dev->pdev->dev, "register driver: %s\n",
			driver->driver.name);
	dev_dbg(&dev->pdev->dev, "<--- %s()\n", __func__);

	return 0;

err:
	dev->gadget.dev.driver = NULL;
	dev->driver = NULL;

	dev_dbg(&dev->pdev->dev, "<--- %s()\n", __func__);

	return retval;
}

/* unregister gadget driver */
static int langwell_stop(struct usb_gadget *g,
		struct usb_gadget_driver *driver)
{
	struct langwell_udc	*dev = gadget_to_langwell(g);
	unsigned long		flags;

	dev_dbg(&dev->pdev->dev, "---> %s()\n", __func__);

#ifdef	OTG_TRANSCEIVER
	pm_runtime_get_sync(&dev->pdev->dev);
#endif
	/* exit PHY low power suspend */
	if (dev->pdev->device != 0x0829)
		langwell_phy_low_power(dev, 0);

	/* unbind OTG transceiver */
	if (dev->transceiver)
		(void)otg_set_peripheral(dev->transceiver->otg, 0);

	/* disable interrupt and set controller to stop state */
	langwell_udc_stop(dev);

	dev->usb_state = USB_STATE_ATTACHED;
	dev->ep0_state = WAIT_FOR_SETUP;
	dev->ep0_dir = USB_DIR_OUT;

	spin_lock_irqsave(&dev->lock, flags);

	/* stop all usb activities */
	dev->gadget.speed = USB_SPEED_UNKNOWN;
	dev->gadget.dev.driver = NULL;
	dev->driver = NULL;
	stop_activity(dev, driver);
	spin_unlock_irqrestore(&dev->lock, flags);

#ifdef	OTG_TRANSCEIVER
	pm_runtime_put_sync(&dev->pdev->dev);
#endif

	dev_info(&dev->pdev->dev, "unregistered driver '%s'\n",
			driver->driver.name);
	dev_dbg(&dev->pdev->dev, "<--- %s()\n", __func__);

	return 0;
}

/*-------------------------------------------------------------------------*/

/*
 * setup tripwire is used as a semaphore to ensure that the setup data
 * payload is extracted from a dQH without being corrupted
 */
static void setup_tripwire(struct langwell_udc *dev)
{
	u32			usbcmd,
				endptsetupstat;
	unsigned long		timeout;
	struct langwell_dqh	*dqh;

	dev_vdbg(&dev->pdev->dev, "---> %s()\n", __func__);

	/* ep0 OUT dQH */
	dqh = &dev->ep_dqh[EP_DIR_OUT];

	/* Write-Clear endptsetupstat */
	endptsetupstat = readl(&dev->op_regs->endptsetupstat);
	writel(endptsetupstat, &dev->op_regs->endptsetupstat);

	/* wait until endptsetupstat is cleared */
	timeout = jiffies + SETUPSTAT_TIMEOUT;
	while (readl(&dev->op_regs->endptsetupstat)) {
		if (time_after(jiffies, timeout)) {
			dev_err(&dev->pdev->dev, "setup_tripwire timeout\n");
			break;
		}
		cpu_relax();
	}

	/* while a hazard exists when setup packet arrives */
	do {
		/* set setup tripwire bit */
		usbcmd = readl(&dev->op_regs->usbcmd);
		writel(usbcmd | CMD_SUTW, &dev->op_regs->usbcmd);

		/* copy the setup packet to local buffer */
		memcpy(&dev->local_setup_buff, &dqh->dqh_setup, 8);
	} while (!(readl(&dev->op_regs->usbcmd) & CMD_SUTW));

	/* Write-Clear setup tripwire bit */
	usbcmd = readl(&dev->op_regs->usbcmd);
	writel(usbcmd & ~CMD_SUTW, &dev->op_regs->usbcmd);

	dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
}


/* protocol ep0 stall, will automatically be cleared on new transaction */
static void ep0_stall(struct langwell_udc *dev)
{
	u32	endptctrl;

	dev_vdbg(&dev->pdev->dev, "---> %s()\n", __func__);

	/* set TX and RX to stall */
	endptctrl = readl(&dev->op_regs->endptctrl[0]);
	endptctrl |= EPCTRL_TXS | EPCTRL_RXS;
	writel(endptctrl, &dev->op_regs->endptctrl[0]);

	/* update ep0 state */
	dev->ep0_state = WAIT_FOR_SETUP;
	dev->ep0_dir = USB_DIR_OUT;

	dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
}


/* PRIME a status phase for ep0 */
static int prime_status_phase(struct langwell_udc *dev, int dir)
{
	struct langwell_request	*req;
	struct langwell_ep	*ep;
	int			status = 0;

	dev_vdbg(&dev->pdev->dev, "---> %s()\n", __func__);

	if (dir == EP_DIR_IN)
		dev->ep0_dir = USB_DIR_IN;
	else
		dev->ep0_dir = USB_DIR_OUT;

	ep = &dev->ep[0];
	dev->ep0_state = WAIT_FOR_OUT_STATUS;

	req = dev->status_req;

	req->ep = ep;
	req->req.length = 0;
	req->req.dma = DMA_ADDR_INVALID;
	req->mapped = 0;
	req->req.status = -EINPROGRESS;
	req->req.actual = 0;
	req->req.complete = NULL;
	req->dtd_count = 0;

	if (!req_to_dtd(req))
		status = queue_dtd(ep, req);
	else
		return -ENOMEM;

	if (status)
		dev_err(&dev->pdev->dev, "can't queue ep0 status request\n");

	list_add_tail(&req->queue, &ep->queue);

	dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
	return status;
}

static void test_mode_complete(struct usb_ep *ep, struct usb_request *_req)
{
	struct langwell_udc	*dev = the_controller;
	struct langwell_request	*req;
	u32			portsc1;

	req = container_of(_req, struct langwell_request, req);

	switch (req->test_mode) {
	case TEST_J:
	case TEST_K:
	case TEST_SE0_NAK:
		langwell_vbus_draw(&dev->gadget, 0);
	case TEST_PACKET:
	case TEST_FORCE_EN:
		langwell_udc_notify_otg(
			MID_OTG_NOTIFY_TEST_MODE_START);
		portsc1 = readl(&dev->op_regs->portsc1);
		portsc1 |= req->test_mode << 16;
		writel(portsc1, &dev->op_regs->portsc1);
		dev_info(&dev->pdev->dev,
			"Enter USB Test Mode 0x%x\n", req->test_mode);
		break;
	default:
		dev_warn(&dev->pdev->dev, "unknown test mode\n");
		break;
	}
}

/* prime_status_phase_test_mode - PRIME a status phase for test mode request
 */
static int prime_status_phase_test_mode(struct langwell_udc *dev,
						int dir, unsigned test_mode)
{
	struct langwell_request	*req;
	struct langwell_ep	*ep;
	int			status = 0;

	dev_vdbg(&dev->pdev->dev, "---> %s()\n", __func__);

	if (dir == EP_DIR_IN)
		dev->ep0_dir = USB_DIR_IN;
	else
		dev->ep0_dir = USB_DIR_OUT;

	ep = &dev->ep[0];
	dev->ep0_state = WAIT_FOR_OUT_STATUS;

	req = dev->status_req;

	req->ep = ep;
	req->test_mode = test_mode;
	req->req.length = 0;
	req->req.dma = DMA_ADDR_INVALID;
	req->mapped = 0;
	req->req.status = -EINPROGRESS;
	req->req.actual = 0;
	req->req.complete = test_mode_complete;
	req->dtd_count = 0;

	if (!req_to_dtd(req))
		status = queue_dtd(ep, req);
	else
		return -ENOMEM;

	if (status)
		dev_err(&dev->pdev->dev, "can't queue ep0 status request\n");

	list_add_tail(&req->queue, &ep->queue);

	dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
	return status;
}

/* SET_ADDRESS request routine */
static void set_address(struct langwell_udc *dev, u16 value,
		u16 index, u16 length)
{
	dev_vdbg(&dev->pdev->dev, "---> %s()\n", __func__);

	/* save the new address to device struct */
	dev->dev_addr = (u8) value;
	dev_vdbg(&dev->pdev->dev, "dev->dev_addr = %d\n", dev->dev_addr);

	/* update usb state */
	dev->usb_state = USB_STATE_ADDRESS;

	/* STATUS phase */
	if (prime_status_phase(dev, EP_DIR_IN))
		ep0_stall(dev);

	dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
}


/* return endpoint by windex */
static struct langwell_ep *get_ep_by_windex(struct langwell_udc *dev,
		u16 wIndex)
{
	struct langwell_ep		*ep;
	dev_vdbg(&dev->pdev->dev, "---> %s()\n", __func__);

	if ((wIndex & USB_ENDPOINT_NUMBER_MASK) == 0)
		return &dev->ep[0];

	list_for_each_entry(ep, &dev->gadget.ep_list, ep.ep_list) {
		u8	bEndpointAddress;
		if (!ep->desc)
			continue;

		bEndpointAddress = ep->desc->bEndpointAddress;
		if ((wIndex ^ bEndpointAddress) & USB_DIR_IN)
			continue;

		if ((wIndex & USB_ENDPOINT_NUMBER_MASK)
			== (bEndpointAddress & USB_ENDPOINT_NUMBER_MASK))
			return ep;
	}

	dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
	return NULL;
}


/* return whether endpoint is stalled, 0: not stalled; 1: stalled */
static int ep_is_stall(struct langwell_ep *ep)
{
	struct langwell_udc	*dev = ep->dev;
	u32			endptctrl;
	int			retval;

	dev_vdbg(&dev->pdev->dev, "---> %s()\n", __func__);
	if (!ep->desc)
		return 0;

	endptctrl = readl(&dev->op_regs->endptctrl[ep->ep_num]);
	if (is_in(ep))
		retval = endptctrl & EPCTRL_TXS ? 1 : 0;
	else
		retval = endptctrl & EPCTRL_RXS ? 1 : 0;

	dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
	return retval;
}


/* GET_STATUS request routine */
static void get_status(struct langwell_udc *dev, u8 request_type, u16 value,
		u16 index, u16 length)
{
	struct langwell_request	*req;
	struct langwell_ep	*ep;
	u16	status_data = 0;	/* 16 bits cpu view status data */
	int	status = 0;
	int	flag = 0;

	dev_vdbg(&dev->pdev->dev, "---> %s()\n", __func__);

	ep = &dev->ep[0];

	if ((request_type & USB_RECIP_MASK) == USB_RECIP_DEVICE) {
		/* HNP polling for host_request_flag */
		if (index == OTG_STATUS_SELECTOR) {
			status_data = dev->gadget.host_request_flag;
			flag = 1;

			dev_vdbg(&dev->pdev->dev,
				"request_flag 0x%x\n", status_data);

		} else
			status_data = dev->dev_status;
	} else if ((request_type & USB_RECIP_MASK) == USB_RECIP_INTERFACE) {
		/* get interface status */
		status_data = 0;
	} else if ((request_type & USB_RECIP_MASK) == USB_RECIP_ENDPOINT) {
		/* get endpoint status */
		struct langwell_ep	*epn;
		epn = get_ep_by_windex(dev, index);
		/* stall if endpoint doesn't exist */
		if (!epn)
			goto stall;

		status_data = ep_is_stall(epn) << USB_ENDPOINT_HALT;
	}

	dev_dbg(&dev->pdev->dev, "get status data: 0x%04x\n", status_data);

	dev->ep0_dir = USB_DIR_IN;

	/* borrow the per device status_req */
	req = dev->status_req;

	/* fill in the reqest structure */
	*((u16 *) req->req.buf) = cpu_to_le16(status_data);
	req->ep = ep;
	req->req.length = flag ? 1 : 2;
	req->req.dma = dma_map_single(&dev->pdev->dev,
		req->req.buf, req->req.length, DMA_TO_DEVICE);
	req->mapped = 1;
	req->req.status = -EINPROGRESS;
	req->req.actual = 0;
	req->req.complete = NULL;
	req->dtd_count = 0;

	/* prime the data phase */
	if (!req_to_dtd(req))
		status = queue_dtd(ep, req);
	else			/* no mem */
		goto stall;

	if (status) {
		dev_err(&dev->pdev->dev,
				"response error on GET_STATUS request\n");
		goto stall;
	}

	list_add_tail(&req->queue, &ep->queue);
	dev->ep0_state = DATA_STATE_XMIT;

	dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
	return;
stall:
	ep0_stall(dev);
	dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
}


/* setup packet interrupt handler */
static void handle_setup_packet(struct langwell_udc *dev,
		struct usb_ctrlrequest *setup)
{
	u16	wValue = le16_to_cpu(setup->wValue);
	u16	wIndex = le16_to_cpu(setup->wIndex);
	u16	wLength = le16_to_cpu(setup->wLength);

	dev_vdbg(&dev->pdev->dev, "---> %s()\n", __func__);

	/* ep0 fifo flush */
	nuke(&dev->ep[0], -ESHUTDOWN);

	dev_dbg(&dev->pdev->dev, "SETUP %02x.%02x v%04x i%04x l%04x\n",
			setup->bRequestType, setup->bRequest,
			wValue, wIndex, wLength);

	/* Delegate non-standard requests to composite driver */
	if ((setup->bRequestType & USB_TYPE_MASK) != USB_TYPE_STANDARD)
		goto delegate;

	/* We process some stardard setup requests here */
	switch (setup->bRequest) {
	case USB_REQ_GET_STATUS:
		dev_dbg(&dev->pdev->dev, "SETUP: USB_REQ_GET_STATUS\n");
		/* get status, DATA and STATUS phase */
		if ((setup->bRequestType & (USB_DIR_IN | USB_TYPE_MASK))
					!= (USB_DIR_IN | USB_TYPE_STANDARD))
			break;
		get_status(dev, setup->bRequestType, wValue, wIndex, wLength);
		goto end;

	case USB_REQ_SET_ADDRESS:
		dev_dbg(&dev->pdev->dev, "SETUP: USB_REQ_SET_ADDRESS\n");
		/* STATUS phase */
		if (setup->bRequestType != (USB_DIR_OUT | USB_TYPE_STANDARD
						| USB_RECIP_DEVICE))
			break;
		set_address(dev, wValue, wIndex, wLength);
		goto end;

	case USB_REQ_CLEAR_FEATURE:
	case USB_REQ_SET_FEATURE:
		/* STATUS phase */
	{
		int rc = -EOPNOTSUPP;
		if (setup->bRequest == USB_REQ_SET_FEATURE)
			dev_dbg(&dev->pdev->dev,
					"SETUP: USB_REQ_SET_FEATURE\n");
		else if (setup->bRequest == USB_REQ_CLEAR_FEATURE)
			dev_dbg(&dev->pdev->dev,
					"SETUP: USB_REQ_CLEAR_FEATURE\n");

		if ((setup->bRequestType & (USB_RECIP_MASK | USB_TYPE_MASK))
				== (USB_RECIP_ENDPOINT | USB_TYPE_STANDARD)) {
			struct langwell_ep	*epn;
			epn = get_ep_by_windex(dev, wIndex);
			/* stall if endpoint doesn't exist */
			if (!epn) {
				ep0_stall(dev);
				goto end;
			}

			if (wValue != 0 || wLength != 0
					|| epn->ep_num > dev->ep_max) {
				ep0_stall(dev);
				break;
			}

			spin_unlock(&dev->lock);
			rc = _langwell_ep_set_halt(&epn->ep,
				(setup->bRequest == USB_REQ_SET_FEATURE)
				? 1 : 0, 0);
			spin_lock(&dev->lock);

		} else if ((setup->bRequestType & (USB_RECIP_MASK
				| USB_TYPE_MASK)) == (USB_RECIP_DEVICE
				| USB_TYPE_STANDARD)) {
			rc = 0;
			switch (wValue) {
			case USB_DEVICE_REMOTE_WAKEUP:
				if (setup->bRequest == USB_REQ_SET_FEATURE) {
					dev->remote_wakeup = 1;
					dev->dev_status |= (1 << wValue);
				} else {
					dev->remote_wakeup = 0;
					dev->dev_status &= ~(1 << wValue);
				}
				break;
			case USB_DEVICE_TEST_MODE:
				dev_dbg(&dev->pdev->dev, "SETUP: TEST MODE\n");
				if ((wIndex & 0xff) ||
					(dev->gadget.speed != USB_SPEED_HIGH))
					ep0_stall(dev);

				switch (wIndex >> 8) {
				case TEST_J:
				case TEST_K:
				case TEST_SE0_NAK:
				case TEST_PACKET:
				case TEST_FORCE_EN:
					if (prime_status_phase_test_mode(dev,
						EP_DIR_IN, wIndex >> 8))
						ep0_stall(dev);
					goto end;
				case TEST_SRP_REQD:
					langwell_udc_notify_otg(
						MID_OTG_NOTIFY_TEST_SRP_REQD);
					break;
				default:
					rc = -EOPNOTSUPP;
				}
				break;
			case USB_DEVICE_B_HNP_ENABLE:
				dev->gadget.b_hnp_enable = 1;
				dev->dev_status |= (1 << wValue);
				break;
			case USB_DEVICE_A_HNP_SUPPORT:
				dev->gadget.a_hnp_support = 1;
				dev->dev_status |= (1 << wValue);
				break;
			case USB_DEVICE_A_ALT_HNP_SUPPORT:
				dev->gadget.a_alt_hnp_support = 1;
				dev->dev_status |= (1 << wValue);
				break;
			default:
				rc = -EOPNOTSUPP;
				break;
			}
		} else
			break;

		if (rc == 0) {
			if (prime_status_phase(dev, EP_DIR_IN))
				ep0_stall(dev);
		} else if (rc == -EOPNOTSUPP)
			ep0_stall(dev);
		goto end;
	}

	case USB_REQ_GET_DESCRIPTOR:
		dev_dbg(&dev->pdev->dev,
				"SETUP: USB_REQ_GET_DESCRIPTOR\n");
		goto delegate;

	case USB_REQ_SET_DESCRIPTOR:
		dev_dbg(&dev->pdev->dev,
				"SETUP: USB_REQ_SET_DESCRIPTOR unsupported\n");
		goto delegate;

	case USB_REQ_GET_CONFIGURATION:
		dev_dbg(&dev->pdev->dev,
				"SETUP: USB_REQ_GET_CONFIGURATION\n");
		goto delegate;

	case USB_REQ_SET_CONFIGURATION:
		dev_dbg(&dev->pdev->dev,
				"SETUP: USB_REQ_SET_CONFIGURATION\n");
		goto delegate;

	case USB_REQ_GET_INTERFACE:
		dev_dbg(&dev->pdev->dev,
				"SETUP: USB_REQ_GET_INTERFACE\n");
		goto delegate;

	case USB_REQ_SET_INTERFACE:
		dev_dbg(&dev->pdev->dev,
				"SETUP: USB_REQ_SET_INTERFACE\n");
		goto delegate;

	case USB_REQ_SYNCH_FRAME:
		dev_dbg(&dev->pdev->dev,
				"SETUP: USB_REQ_SYNCH_FRAME unsupported\n");
		goto delegate;

	default:
		/* delegate USB standard requests to the gadget driver */
		goto delegate;
delegate:
		/* USB requests handled by gadget */
		if (wLength) {
			/* DATA phase from gadget, STATUS phase from udc */
			dev->ep0_dir = (setup->bRequestType & USB_DIR_IN)
					?  USB_DIR_IN : USB_DIR_OUT;
			dev_vdbg(&dev->pdev->dev,
					"dev->ep0_dir = 0x%x, wLength = %d\n",
					dev->ep0_dir, wLength);
			spin_unlock(&dev->lock);
			if (dev->driver->setup(&dev->gadget,
					&dev->local_setup_buff) < 0)
				ep0_stall(dev);
			spin_lock(&dev->lock);
			dev->ep0_state = (setup->bRequestType & USB_DIR_IN)
					?  DATA_STATE_XMIT : DATA_STATE_RECV;
		} else {
			/* no DATA phase, IN STATUS phase from gadget */
			dev->ep0_dir = USB_DIR_IN;
			dev_vdbg(&dev->pdev->dev,
					"dev->ep0_dir = 0x%x, wLength = %d\n",
					dev->ep0_dir, wLength);
			spin_unlock(&dev->lock);
			if (dev->driver->setup(&dev->gadget,
					&dev->local_setup_buff) < 0)
				ep0_stall(dev);
			spin_lock(&dev->lock);
			dev->ep0_state = WAIT_FOR_OUT_STATUS;
		}
		break;
	}
end:
	dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
}


/* transfer completion, process endpoint request and free the completed dTDs
 * for this request
 */
static int process_ep_req(struct langwell_udc *dev, int index,
		struct langwell_request *curr_req)
{
	struct langwell_dtd	*curr_dtd;
	struct langwell_dqh	*curr_dqh;
	int			td_complete, actual, remaining_length;
	int			i, dir;
	u8			dtd_status = 0;
	int			retval = 0;

	curr_dqh = &dev->ep_dqh[index];
	dir = index % 2;

	curr_dtd = curr_req->head;
	td_complete = 0;
	actual = curr_req->req.length;

	dev_vdbg(&dev->pdev->dev, "---> %s()\n", __func__);

	for (i = 0; i < curr_req->dtd_count; i++) {

		/* command execution states by dTD */
		dtd_status = curr_dtd->dtd_status;

		barrier();
		remaining_length = le16_to_cpu(curr_dtd->dtd_total);
		actual -= remaining_length;

		if (!dtd_status) {
			/* transfers completed successfully */
			if (!remaining_length) {
				td_complete++;
				dev_vdbg(&dev->pdev->dev,
					"dTD transmitted successfully\n");
			} else {
				if (dir) {
					dev_vdbg(&dev->pdev->dev,
						"TX dTD remains data\n");
					retval = -EPROTO;
					break;

				} else {
					td_complete++;
					if (i < curr_req->dtd_count - 1) {
						WARN(1, "Short packet received on ep%d-IN,\n"
							"but dTD isn't the last one.\n",
							index / 2);
						retval = -EREMOTEIO;
					}
					break;
				}
			}
		} else {
			/* transfers completed with errors */
			if (dtd_status & DTD_STS_ACTIVE) {
				dev_dbg(&dev->pdev->dev,
					"dTD status ACTIVE dQH[%d]\n", index);
				retval = 1;
				return retval;
			} else if (dtd_status & DTD_STS_HALTED) {
				dev_err(&dev->pdev->dev,
					"dTD error %08x dQH[%d]\n",
					dtd_status, index);
				/* clear the errors and halt condition */
				curr_dqh->dtd_status = 0;
				retval = -EPIPE;
				break;
			} else if (dtd_status & DTD_STS_DBE) {
				dev_dbg(&dev->pdev->dev,
					"data buffer (overflow) error\n");
				retval = -EPROTO;
				break;
			} else if (dtd_status & DTD_STS_TRE) {
				dev_dbg(&dev->pdev->dev,
					"transaction(ISO) error\n");
				retval = -EILSEQ;
				break;
			} else
				dev_err(&dev->pdev->dev,
					"unknown error (0x%x)!\n",
					dtd_status);
		}

		if (i != curr_req->dtd_count - 1)
			curr_dtd = (struct langwell_dtd *)
				curr_dtd->next_dtd_virt;
	}

	if (retval)
		return retval;

	curr_req->req.actual = actual;

	dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
	return 0;
}


/* complete DATA or STATUS phase of ep0 prime status phase if needed */
static void ep0_req_complete(struct langwell_udc *dev,
		struct langwell_ep *ep0, struct langwell_request *req)
{
	u32	new_addr;
	dev_vdbg(&dev->pdev->dev, "---> %s()\n", __func__);

	if (dev->usb_state == USB_STATE_ADDRESS) {
		/* set the new address */
		new_addr = (u32)dev->dev_addr;
		writel(new_addr << USBADR_SHIFT, &dev->op_regs->deviceaddr);

		new_addr = USBADR(readl(&dev->op_regs->deviceaddr));
		dev_vdbg(&dev->pdev->dev, "new_addr = %d\n", new_addr);
	}

	done(ep0, req, 0);

	switch (dev->ep0_state) {
	case DATA_STATE_XMIT:
		/* receive status phase */
		if (prime_status_phase(dev, EP_DIR_OUT))
			ep0_stall(dev);
		break;
	case DATA_STATE_RECV:
		/* send status phase */
		if (prime_status_phase(dev, EP_DIR_IN))
			ep0_stall(dev);
		break;
	case WAIT_FOR_OUT_STATUS:
		dev->ep0_state = WAIT_FOR_SETUP;
		break;
	case WAIT_FOR_SETUP:
		dev_err(&dev->pdev->dev, "unexpect ep0 packets\n");
		break;
	default:
		ep0_stall(dev);
		break;
	}

	dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
}


/* USB transfer completion interrupt */
static void handle_trans_complete(struct langwell_udc *dev)
{
	u32			complete_bits;
	int			i, ep_num, dir, bit_mask, status;
	struct langwell_ep	*epn;
	struct langwell_request	*curr_req, *temp_req;

	dev_vdbg(&dev->pdev->dev, "---> %s()\n", __func__);

	complete_bits = readl(&dev->op_regs->endptcomplete);
	dev_vdbg(&dev->pdev->dev, "endptcomplete register: 0x%08x\n",
			complete_bits);

	/* Write-Clear the bits in endptcomplete register */
	writel(complete_bits, &dev->op_regs->endptcomplete);

	if (!complete_bits) {
		dev_dbg(&dev->pdev->dev, "complete_bits = 0\n");
		goto done;
	}

	for (i = 0; i < dev->ep_max; i++) {
		ep_num = i / 2;
		dir = i % 2;

		bit_mask = 1 << (ep_num + 16 * dir);

		if (!(complete_bits & bit_mask))
			continue;

		/* ep0 */
		if (i == 1)
			epn = &dev->ep[0];
		else
			epn = &dev->ep[i];

		if (epn->name == NULL) {
			dev_warn(&dev->pdev->dev, "invalid endpoint\n");
			continue;
		}

		if (i < 2)
			/* ep0 in and out */
			dev_dbg(&dev->pdev->dev, "%s-%s transfer completed\n",
					epn->name,
					is_in(epn) ? "in" : "out");
		else
			dev_dbg(&dev->pdev->dev, "%s transfer completed\n",
					epn->name);

		/* process the req queue until an uncomplete request */
		list_for_each_entry_safe(curr_req, temp_req,
				&epn->queue, queue) {
			status = process_ep_req(dev, i, curr_req);
			dev_vdbg(&dev->pdev->dev, "%s req status: %d\n",
					epn->name, status);

			/* Short Read on non-last dTD is not recoverable due to
			 * HW limitation. The best we can do is to recycle dTDs
			 * and notify the caller that we screw up... :(
			 */
			if (unlikely(status == -EREMOTEIO)) {

				u32 value;

				value = readl(&dev->op_regs->endptctrl[ep_num]);
				writel(value & ~EPCTRL_RXE,
					&dev->op_regs->endptctrl[ep_num]);
				nuke(epn, status);
				value = readl(&dev->op_regs->endptctrl[ep_num]);
				writel(value | EPCTRL_RXE,
					&dev->op_regs->endptctrl[ep_num]);
				break;
			}

			if (status == 1)
				break;

			/* write back status to req */
			curr_req->req.status = status;

			/* ep0 request completion */
			if (ep_num == 0) {
				ep0_req_complete(dev, epn, curr_req);
				break;
			} else {
				/* Check to guarantee ep is enabled */
				if (!epn->desc) {
					dev_err(&dev->pdev->dev,
					"epn is disabled, break in handle trans complete\n");
					break;
				}
				done(epn, curr_req, status);
			}
		}
	}
done:
	dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
}

/* port change detect interrupt handler */
static void handle_port_change(struct langwell_udc *dev)
{
	u32		portsc1, devlc;
	u32		speed;

	dev_vdbg(&dev->pdev->dev, "---> %s()\n", __func__);

	if (dev->bus_reset)
		dev->bus_reset = 0;

	portsc1 = readl(&dev->op_regs->portsc1);
	devlc = readl(&dev->op_regs->devlc);
	dev_vdbg(&dev->pdev->dev, "portsc1 = 0x%08x, devlc = 0x%08x\n",
			portsc1, devlc);

	/* bus reset is finished */
	if (!(portsc1 & PORTS_PR)) {
		/* get the speed */
		dev->gadget.speed = lpm_device_speed(devlc);
		dev_vdbg(&dev->pdev->dev, "dev->gadget.speed = %d\n",
			dev->gadget.speed);

		speed = LPM_PSPD(devlc);
		switch (speed) {
		case LPM_SPEED_HIGH:
			dev->gadget.speed = USB_SPEED_HIGH;
			break;
		case LPM_SPEED_FULL:
			dev->gadget.speed = USB_SPEED_FULL;
			break;
		case LPM_SPEED_LOW:
			dev->gadget.speed = USB_SPEED_LOW;
			break;
		default:
			dev->gadget.speed = USB_SPEED_UNKNOWN;
			break;
		}

	}

	/* LPM L0 to L1 */
	if (dev->lpm && dev->lpm_state == LPM_L0)
		if (portsc1 & PORTS_SUSP && portsc1 & PORTS_SLP) {
			dev_info(&dev->pdev->dev, "LPM L0 to L1\n");
			dev->lpm_state = LPM_L1;
		}

	/* LPM L1 to L0, force resume or remote wakeup finished */
	if (dev->lpm && dev->lpm_state == LPM_L1)
		if (!(portsc1 & PORTS_SUSP)) {
			dev_info(&dev->pdev->dev, "LPM L1 to L0\n");
			dev->lpm_state = LPM_L0;
		}

	/* update USB state */
	if (!dev->resume_state)
		dev->usb_state = USB_STATE_DEFAULT;

	dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
}


/* USB reset interrupt handler */
static void handle_usb_reset(struct langwell_udc *dev)
{
	u32		deviceaddr,
			endptsetupstat,
			endptcomplete;
	unsigned long	timeout;

	dev_vdbg(&dev->pdev->dev, "---> %s()\n", __func__);

	/* notify reset event to OTG */
	langwell_udc_notify_otg(MID_OTG_NOTIFY_CRESET);

	/* Write-Clear the device address */
	deviceaddr = readl(&dev->op_regs->deviceaddr);
	writel(deviceaddr & ~USBADR_MASK, &dev->op_regs->deviceaddr);

	dev->dev_addr = 0;

	/* clear usb state */
	dev->resume_state = 0;

	/* LPM L1 to L0, reset */
	if (dev->lpm)
		dev->lpm_state = LPM_L0;

	dev->ep0_dir = USB_DIR_OUT;
	dev->ep0_state = WAIT_FOR_SETUP;

	/* remote wakeup reset to 0 when the device is reset */
	dev->remote_wakeup = 0;
	dev->gadget.b_hnp_enable = 0;
	dev->gadget.a_hnp_support = 0;
	dev->gadget.a_alt_hnp_support = 0;

	/* Write-Clear all the setup token semaphores */
	endptsetupstat = readl(&dev->op_regs->endptsetupstat);
	writel(endptsetupstat, &dev->op_regs->endptsetupstat);

	/* Write-Clear all the endpoint complete status bits */
	endptcomplete = readl(&dev->op_regs->endptcomplete);
	writel(endptcomplete, &dev->op_regs->endptcomplete);

	/* wait until all endptprime bits cleared */
	timeout = 100 * PRIME_TIMEOUT;
	while (readl(&dev->op_regs->endptprime)) {
		if (--timeout == 0) {
			dev_err(&dev->pdev->dev, "USB reset timeout\n");
			break;
		}
		udelay(10);
	}

	/* write 1s to endptflush register to clear any primed buffers */
	writel((u32) ~0, &dev->op_regs->endptflush);

	if (!(readl(&dev->op_regs->portsc1) & PORTS_PR))
		dev_dbg(&dev->pdev->dev, "Intented to reset device controller?\n");

	/* bus is reseting */
	dev->bus_reset = 1;

	/* reset all the queues, stop all USB activities */
	stop_activity(dev, dev->driver);
	dev->usb_state = USB_STATE_DEFAULT;

	dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
}


/* USB bus suspend/resume interrupt */
static void handle_bus_suspend(struct langwell_udc *dev)
{
	dev_dbg(&dev->pdev->dev, "---> %s()\n", __func__);

	if (dev->usb_state == USB_STATE_SUSPENDED)
		return;

	dev->resume_state = dev->usb_state;
	dev->usb_state = USB_STATE_SUSPENDED;

	langwell_udc_notify_otg(MID_OTG_NOTIFY_CSUSPEND);

	/* report suspend to the driver */
	if (dev->driver) {
		if (dev->driver->suspend) {
			spin_unlock(&dev->lock);
			dev->driver->suspend(&dev->gadget);
			spin_lock(&dev->lock);
			dev_dbg(&dev->pdev->dev, "suspend %s\n",
					dev->driver->driver.name);
		}
	}

	dev_dbg(&dev->pdev->dev, "<--- %s()\n", __func__);
}


static void handle_bus_resume(struct langwell_udc *dev)
{
	dev_dbg(&dev->pdev->dev, "---> %s()\n", __func__);

	dev->usb_state = dev->resume_state;
	dev->resume_state = 0;

	langwell_udc_notify_otg(MID_OTG_NOTIFY_CRESUME);

	/* report resume to the driver */
	if (dev->driver) {
		if (dev->driver->resume) {
			spin_unlock(&dev->lock);
			dev->driver->resume(&dev->gadget);
			spin_lock(&dev->lock);
			dev_dbg(&dev->pdev->dev, "resume %s\n",
					dev->driver->driver.name);
		}
	}

	dev_dbg(&dev->pdev->dev, "<--- %s()\n", __func__);
}


/* USB device controller interrupt handler */
static irqreturn_t langwell_irq(int irq, void *_dev)
{
	struct langwell_udc	*dev = _dev;
	u32			usbsts,
				usbintr,
				irq_sts,
				portsc1;

	dev_vdbg(&dev->pdev->dev, "---> %s()\n", __func__);

	if (dev->stopped) {
		dev_vdbg(&dev->pdev->dev, "handle IRQ_NONE\n");
		dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
		return IRQ_NONE;
	}

	spin_lock(&dev->lock);

	/* USB status */
	usbsts = readl(&dev->op_regs->usbsts);

	/* USB interrupt enable */
	usbintr = readl(&dev->op_regs->usbintr);

	irq_sts = usbsts & usbintr;
	dev_vdbg(&dev->pdev->dev,
			"usbsts = 0x%08x, usbintr = 0x%08x, irq_sts = 0x%08x\n",
			usbsts, usbintr, irq_sts);

	if (!irq_sts) {
		dev_vdbg(&dev->pdev->dev, "handle IRQ_NONE\n");
		dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
		spin_unlock(&dev->lock);
		return IRQ_NONE;
	}

	/* Write-Clear interrupt status bits */
	writel(irq_sts, &dev->op_regs->usbsts);

	/* resume from suspend */
	portsc1 = readl(&dev->op_regs->portsc1);
	if (dev->usb_state == USB_STATE_SUSPENDED)
		if (!(portsc1 & PORTS_SUSP))
			handle_bus_resume(dev);

	/* USB interrupt */
	if (irq_sts & STS_UI) {
		dev_vdbg(&dev->pdev->dev, "USB interrupt\n");

		/* setup packet received from ep0 */
		if (readl(&dev->op_regs->endptsetupstat)
				& EP0SETUPSTAT_MASK) {
			dev_vdbg(&dev->pdev->dev,
				"USB SETUP packet received interrupt\n");
			/* setup tripwire semaphone */
			setup_tripwire(dev);
			handle_setup_packet(dev, &dev->local_setup_buff);
		}

		/* USB transfer completion */
		if (readl(&dev->op_regs->endptcomplete)) {
			dev_vdbg(&dev->pdev->dev,
				"USB transfer completion interrupt\n");
			handle_trans_complete(dev);
		}
	}

	/* SOF received interrupt (for ISO transfer) */
	if (irq_sts & STS_SRI) {
		/* FIXME */
		/* dev_vdbg(&dev->pdev->dev, "SOF received interrupt\n"); */
	}

	/* port change detect interrupt */
	if (irq_sts & STS_PCI) {
		dev_vdbg(&dev->pdev->dev, "port change detect interrupt\n");
		handle_port_change(dev);
	}

	/* suspend interrupt */
	if (irq_sts & STS_SLI) {
		dev_vdbg(&dev->pdev->dev, "suspend interrupt\n");
		handle_bus_suspend(dev);
	}

	/* USB reset interrupt */
	if (irq_sts & STS_URI) {
		dev_vdbg(&dev->pdev->dev, "USB reset interrupt\n");
		handle_usb_reset(dev);
	}

	/* USB error or system error interrupt */
	if (irq_sts & (STS_UEI | STS_SEI)) {
		/* FIXME */
		dev_warn(&dev->pdev->dev, "error IRQ, irq_sts: %x\n", irq_sts);
	}

	spin_unlock(&dev->lock);

	dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
	return IRQ_HANDLED;
}


/*-------------------------------------------------------------------------*/

/* release device structure */
static void gadget_release(struct device *_dev)
{
	struct langwell_udc	*dev = dev_get_drvdata(_dev);

	dev_dbg(&dev->pdev->dev, "---> %s()\n", __func__);

	complete(dev->done);

	wake_lock_destroy(&dev->wake_lock);

	dev_dbg(&dev->pdev->dev, "<--- %s()\n", __func__);
	kfree(dev);
}


/* enable SRAM caching if SRAM detected */
static void sram_init(struct langwell_udc *dev)
{
	struct pci_dev		*pdev = dev->pdev;
	void __iomem		*base = NULL;
	void __iomem		*addr = NULL;

	dev_dbg(&dev->pdev->dev, "---> %s()\n", __func__);

	dev->sram_addr = pci_resource_start(pdev, 1);
	dev->sram_size = pci_resource_len(pdev, 1);
	dev_info(&dev->pdev->dev, "Found private SRAM at %x size:%x\n",
			dev->sram_addr, dev->sram_size);

	/* initialize SRAM to 0 to avoid ECC errors */
	base = ioremap_nocache(dev->sram_addr, dev->sram_size);
	if (base == NULL) {
		dev_err(&dev->pdev->dev, "SRAM init: ioremap failed\n");
		return;
	}

	addr = base;

	while (addr < base + dev->sram_size) {
		writel(0x0, addr);
		addr = addr + 4;
	}

	iounmap(base);

	dev->got_sram = 1;

	if (pci_request_region(pdev, 1, kobject_name(&pdev->dev.kobj))) {
		dev_warn(&dev->pdev->dev, "SRAM request failed\n");
		dev->got_sram = 0;
	} else if (!dma_declare_coherent_memory(&pdev->dev, dev->sram_addr,
			dev->sram_addr, dev->sram_size, DMA_MEMORY_MAP)) {
		dev_warn(&dev->pdev->dev, "SRAM DMA declare failed\n");
		pci_release_region(pdev, 1);
		dev->got_sram = 0;
	}

	dev_dbg(&dev->pdev->dev, "<--- %s()\n", __func__);
}


/* release SRAM caching */
static void sram_deinit(struct langwell_udc *dev)
{
	struct pci_dev *pdev = dev->pdev;

	dev_dbg(&dev->pdev->dev, "---> %s()\n", __func__);

	dma_release_declared_memory(&pdev->dev);
	pci_release_region(pdev, 1);

	dev->got_sram = 0;

	dev_info(&dev->pdev->dev, "release SRAM caching\n");
	dev_dbg(&dev->pdev->dev, "<--- %s()\n", __func__);
}


/* tear down the binding between this driver and the pci device */
static void langwell_udc_remove(struct pci_dev *pdev)
{
	struct langwell_udc	*dev = pci_get_drvdata(pdev);

	DECLARE_COMPLETION(done);

	BUG_ON(dev->driver);
	dev_dbg(&dev->pdev->dev, "---> %s()\n", __func__);

	dev->done = &done;

#ifndef	OTG_TRANSCEIVER
	pm_runtime_get_noresume(&pdev->dev);

	/* free dTD dma_pool and dQH */
	if (dev->dtd_pool) {
		dma_pool_destroy(dev->dtd_pool);
		dev->dtd_pool = NULL;
	}

	if (dev->ep_dqh)
		dma_free_coherent(&pdev->dev, dev->ep_dqh_size,
			dev->ep_dqh, dev->ep_dqh_dma);

	/* release SRAM caching */
	if (dev->has_sram && dev->got_sram)
		sram_deinit(dev);
#endif

	if (dev->status_req) {
		kfree(dev->status_req->req.buf);
		kfree(dev->status_req);
	}

	kfree(dev->ep);

	/* disable IRQ handler */
	if (dev->got_irq)
		free_irq(pdev->irq, dev);

#ifndef	OTG_TRANSCEIVER
	if (dev->cap_regs)
		iounmap(dev->cap_regs);

	if (dev->region)
		release_mem_region(pci_resource_start(pdev, 0),
				pci_resource_len(pdev, 0));

	if (dev->enabled)
		pci_disable_device(pdev);
#else
	if (dev->transceiver) {
		usb_put_phy(dev->transceiver);
		dev->transceiver = NULL;
		dev->iotg = NULL;
	}
#endif

	dev->cap_regs = NULL;

	dev_info(&dev->pdev->dev, "unbind\n");
	dev_dbg(&dev->pdev->dev, "<--- %s()\n", __func__);

	device_unregister(&dev->gadget.dev);
	device_remove_file(&pdev->dev, &dev_attr_langwell_udc);
	device_remove_file(&pdev->dev, &dev_attr_remote_wakeup);
	device_remove_file(&pdev->dev, &dev_attr_sdis);
	device_remove_file(&pdev->dev, &dev_attr_langwell_snapshot);

#ifndef	OTG_TRANSCEIVER
	pci_set_drvdata(pdev, NULL);
#endif

	/* free dev, wait for the release() finished */
	wait_for_completion(&done);
}


/*
 * wrap this driver around the specified device, but
 * don't respond over USB until a gadget driver binds to us.
 */
static int langwell_udc_probe(struct pci_dev *pdev,
		const struct pci_device_id *id)
{
	struct langwell_udc	*dev;
#ifndef	OTG_TRANSCEIVER
	unsigned long		resource, len;
	int			size;
#endif
	void			__iomem *base = NULL;
	int			retval;

	/* alloc, and start init */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (dev == NULL) {
		retval = -ENOMEM;
		goto error;
	}
	/* FIXME: should totally remove this global*/
	the_controller = dev;

	/* initialize device spinlock */
	spin_lock_init(&dev->lock);

	dev->pdev = pdev;
	dev_dbg(&dev->pdev->dev, "---> %s()\n", __func__);

	pci_set_drvdata(pdev, dev);

#ifdef	OTG_TRANSCEIVER
	/* PCI device is already enabled by usb_phy driver */
	dev->enabled = 1;

	/* mem region and register base */
	dev->region = 1;
	dev->transceiver = usb_get_phy(USB_PHY_TYPE_USB2);

	dev->iotg = otg_to_mid_xceiv(dev->transceiver);

	base = dev->iotg->base;
	dev->is_peripheral_start = 0;

	/*
	 * In OTG case, OTG Transceiver driver initialize itself early
	 * so if udc want to access hardware, just get_sync it back.
	 */
	pm_runtime_get_sync(&dev->pdev->dev);
#else
	/* now all the pci goodies ... */
	if (pci_enable_device(pdev) < 0) {
		retval = -ENODEV;
		goto error;
	}
	dev->enabled = 1;

	/* control register: BAR 0 */
	resource = pci_resource_start(pdev, 0);
	len = pci_resource_len(pdev, 0);
	if (!request_mem_region(resource, len, driver_name)) {
		dev_err(&dev->pdev->dev, "controller already in use\n");
		retval = -EBUSY;
		goto error;
	}
	dev->region = 1;

	base = ioremap_nocache(resource, len);
#endif
	if (base == NULL) {
		dev_err(&dev->pdev->dev, "can't map memory\n");
		retval = -EFAULT;
		goto error;
	}

	dev->cap_regs = (struct langwell_cap_regs __iomem *) base;
	dev_vdbg(&dev->pdev->dev, "dev->cap_regs: %p\n", dev->cap_regs);
	dev->op_regs = (struct langwell_op_regs __iomem *)
		(base + OP_REG_OFFSET);
	dev_vdbg(&dev->pdev->dev, "dev->op_regs: %p\n", dev->op_regs);

	/* irq setup after old hardware is cleaned up */
	if (!pdev->irq) {
		dev_err(&dev->pdev->dev, "No IRQ. Check PCI setup!\n");
		retval = -ENODEV;
		goto error;
	}

	dev->has_sram = 1;

	dev->got_sram = 0;
	dev_vdbg(&dev->pdev->dev, "dev->has_sram: %d\n", dev->has_sram);

#ifndef	OTG_TRANSCEIVER
	/* enable SRAM caching if detected */
	if (dev->has_sram && !dev->got_sram)
		sram_init(dev);

	dev_info(&dev->pdev->dev,
			"irq %d, io mem: 0x%08lx, len: 0x%08lx, pci mem 0x%p\n",
			pdev->irq, resource, len, base);
	/* enables bus-mastering for device dev */
	pci_set_master(pdev);

	if (request_irq(pdev->irq, langwell_irq, IRQF_SHARED,
				driver_name, dev) != 0) {
		dev_err(&dev->pdev->dev,
				"request interrupt %d failed\n", pdev->irq);
		retval = -EBUSY;
		goto error;
	}
	dev->got_irq = 1;
#endif
	/* default in streaming mode */
	dev->sdis = 0;

	/* set stopped bit */
	dev->stopped = 1;

	/* capabilities and endpoint number */
	dev->lpm = (readl(&dev->cap_regs->hccparams) & HCC_LEN) ? 1 : 0;
	dev->dciversion = readw(&dev->cap_regs->dciversion);
	dev->devcap = (readl(&dev->cap_regs->dccparams) & DEVCAP) ? 1 : 0;
	dev_vdbg(&dev->pdev->dev, "dev->lpm: %d\n", dev->lpm);
	dev_vdbg(&dev->pdev->dev, "dev->dciversion: 0x%04x\n",
			dev->dciversion);
	dev_vdbg(&dev->pdev->dev, "dccparams: 0x%08x\n",
			readl(&dev->cap_regs->dccparams));
	dev_vdbg(&dev->pdev->dev, "dev->devcap: %d\n", dev->devcap);
	if (!dev->devcap) {
		dev_err(&dev->pdev->dev, "can't support device mode\n");
		retval = -ENODEV;
		goto error;
	}

	/* a pair of endpoints (out/in) for each address */
	dev->ep_max = DEN(readl(&dev->cap_regs->dccparams)) * 2;
	dev_vdbg(&dev->pdev->dev, "dev->ep_max: %d\n", dev->ep_max);

	/* allocate endpoints memory */
	dev->ep = kzalloc(sizeof(struct langwell_ep) * dev->ep_max,
			GFP_KERNEL);
	if (!dev->ep) {
		dev_err(&dev->pdev->dev, "allocate endpoints memory failed\n");
		retval = -ENOMEM;
		goto error;
	}

#ifndef	OTG_TRANSCEIVER
	/* allocate device dQH memory */
	size = dev->ep_max * sizeof(struct langwell_dqh);
	dev_vdbg(&dev->pdev->dev, "orig size = %zd\n", size);
	size = roundup(size, DQH_ALIGNMENT);
	dev->ep_dqh = dma_alloc_coherent(&pdev->dev, size,
					&dev->ep_dqh_dma, GFP_KERNEL);
	if (!dev->ep_dqh) {
		dev_err(&dev->pdev->dev, "allocate dQH memory failed\n");
		retval = -ENOMEM;
		goto error;
	}
	dev->ep_dqh_size = size;
	dev_vdbg(&dev->pdev->dev, "ep_dqh_size = %zd\n", dev->ep_dqh_size);
#endif

	/* initialize ep0 status request structure */
	dev->status_req = kzalloc(sizeof(struct langwell_request), GFP_KERNEL);
	if (!dev->status_req) {
		dev_err(&dev->pdev->dev,
				"allocate status_req memory failed\n");
		retval = -ENOMEM;
		goto error;
	}
	INIT_LIST_HEAD(&dev->status_req->queue);

	/* allocate a small amount of memory to get valid address */
	dev->status_req->req.buf = kmalloc(8, GFP_KERNEL);
	dev->status_req->req.dma = DMA_ADDR_INVALID;

	dev->resume_state = USB_STATE_NOTATTACHED;
	dev->usb_state = USB_STATE_POWERED;
	dev->ep0_dir = USB_DIR_OUT;

	/* remote wakeup reset to 0 when the device is reset */
	dev->remote_wakeup = 0;

#ifndef	OTG_TRANSCEIVER
	/* reset device controller */
	langwell_udc_reset(dev);
#endif

	/* initialize gadget structure */
	dev->gadget.ops = &langwell_ops;	/* usb_gadget_ops */
	dev->gadget.ep0 = &dev->ep[0].ep;	/* gadget ep0 */
	INIT_LIST_HEAD(&dev->gadget.ep_list);	/* ep_list */
	dev->gadget.speed = USB_SPEED_UNKNOWN;	/* speed */
	dev->gadget.max_speed = USB_SPEED_HIGH;	/* support dual speed */
#if defined(OTG_TRANSCEIVER) && defined(CONFIG_USB_LANGWELLUDC_OTG)
	dev->gadget.is_otg = 1;			/* support otg mode */
#endif

	/* the "gadget" abstracts/virtualizes the controller */
	dev_set_name(&dev->gadget.dev, "gadget");
	dev->gadget.dev.parent = &pdev->dev;
	dev->gadget.dev.dma_mask = pdev->dev.dma_mask;
	dev->gadget.dev.release = gadget_release;
	dev->gadget.name = driver_name;		/* gadget name */

	/* controller endpoints reinit */
	eps_reinit(dev);

#ifndef	OTG_TRANSCEIVER
	/* reset ep0 dQH and endptctrl */
	ep0_reset(dev);

	/* create dTD dma_pool resource */
	dev->dtd_pool = dma_pool_create("langwell_dtd",
			&dev->pdev->dev,
			sizeof(struct langwell_dtd),
			DTD_ALIGNMENT,
			DMA_BOUNDARY);

	if (!dev->dtd_pool) {
		retval = -ENOMEM;
		goto error;
	}
#endif

	wake_lock_init(&dev->wake_lock, WAKE_LOCK_SUSPEND,
			pci_name(dev->pdev));

	/* done */
	dev_info(&dev->pdev->dev, "%s\n", driver_desc);
	dev_info(&dev->pdev->dev, "irq %d, pci mem %p\n", pdev->irq, base);
	dev_info(&dev->pdev->dev, "Driver version: " DRIVER_VERSION "\n");
	dev_info(&dev->pdev->dev, "Support (max) %d endpoints\n", dev->ep_max);
	dev_info(&dev->pdev->dev, "Device interface version: 0x%04x\n",
			dev->dciversion);
	dev_info(&dev->pdev->dev, "Controller mode: %s\n",
			dev->devcap ? "Device" : "Host");
	dev_info(&dev->pdev->dev, "Support USB LPM: %s\n",
			dev->lpm ? "Yes" : "No");

	dev_vdbg(&dev->pdev->dev,
			"After langwell_udc_probe(), print all registers:\n");
	print_all_registers(dev);

	retval = usb_add_gadget_udc(&pdev->dev, &dev->gadget);
	if (retval)
		goto error;

	retval = device_create_file(&pdev->dev, &dev_attr_langwell_udc);
	if (retval)
		goto error;

	retval = device_create_file(&pdev->dev, &dev_attr_remote_wakeup);
	if (retval)
		goto error_attr1;

	retval = device_create_file(&pdev->dev, &dev_attr_sdis);
	if (retval)
		goto error_attr2;

	retval = device_create_file(&pdev->dev, &dev_attr_langwell_snapshot);
	if (retval)
		goto error_attr3;

#ifdef OTG_TRANSCEIVER
	pm_runtime_put_sync(&pdev->dev);
#else
	pm_runtime_put_noidle(&pdev->dev);
#endif
	dev_vdbg(&dev->pdev->dev, "<--- %s()\n", __func__);
	return 0;

error_attr3:
	device_remove_file(&pdev->dev, &dev_attr_sdis);
error_attr2:
	device_remove_file(&pdev->dev, &dev_attr_remote_wakeup);
error_attr1:
	device_remove_file(&pdev->dev, &dev_attr_langwell_udc);
error:
	if (dev) {
		dev_dbg(&dev->pdev->dev, "<--- %s()\n", __func__);
		langwell_udc_remove(pdev);
	}

	return retval;
}


/* device controller suspend */
static int langwell_udc_suspend(struct pci_dev *pdev, pm_message_t state)
{
	struct langwell_udc	*dev = pci_get_drvdata(pdev);
	unsigned long		flags;

	dev_dbg(&dev->pdev->dev, "---> %s()\n", __func__);

	usb_del_gadget_udc(&dev->gadget);
	/* disable interrupt and set controller to stop state */
	langwell_udc_stop(dev);

	/* disable IRQ handler */
	if (dev->got_irq)
		free_irq(pdev->irq, dev);
	dev->got_irq = 0;

	spin_lock_irqsave(&dev->lock, flags);
	/* stop all usb activities */
	stop_activity(dev, dev->driver);
	spin_unlock_irqrestore(&dev->lock, flags);

	/* save PCI state */
	pci_save_state(pdev);

	/* free dTD dma_pool and dQH */
	if (dev->dtd_pool) {
		dma_pool_destroy(dev->dtd_pool);
		dev->dtd_pool = NULL;
	}

	if (dev->ep_dqh)
		dma_free_coherent(&pdev->dev, dev->ep_dqh_size,
			dev->ep_dqh, dev->ep_dqh_dma);

	/* release SRAM caching */
	if (dev->has_sram && dev->got_sram)
		sram_deinit(dev);

	/* set device power state */
	pci_set_power_state(pdev, PCI_D3hot);

	/* enter PHY low power suspend */
	langwell_phy_low_power(dev, 1);

	dev_dbg(&dev->pdev->dev, "<--- %s()\n", __func__);
	return 0;
}


/* device controller resume */
static int langwell_udc_resume(struct pci_dev *pdev)
{
	struct langwell_udc	*dev = pci_get_drvdata(pdev);
	size_t			size;

	dev_dbg(&dev->pdev->dev, "---> %s()\n", __func__);

	/* exit PHY low power suspend */
	langwell_phy_low_power(dev, 0);

	/* set device D0 power state */
	pci_set_power_state(pdev, PCI_D0);

	/* enable SRAM caching if detected */
	if (dev->has_sram && !dev->got_sram)
		sram_init(dev);

	/* allocate device dQH memory */
	size = dev->ep_max * sizeof(struct langwell_dqh);
	dev_vdbg(&dev->pdev->dev, "orig size = %zd\n", size);
	size = roundup(size, DQH_ALIGNMENT);
	dev->ep_dqh = dma_alloc_coherent(&pdev->dev, size,
					&dev->ep_dqh_dma, GFP_KERNEL);
	if (!dev->ep_dqh) {
		dev_err(&dev->pdev->dev, "allocate dQH memory failed\n");
		return -ENOMEM;
	}
	dev->ep_dqh_size = size;
	dev_vdbg(&dev->pdev->dev, "ep_dqh_size = %zd\n", dev->ep_dqh_size);

	/* create dTD dma_pool resource */
	dev->dtd_pool = dma_pool_create("langwell_dtd",
			&dev->pdev->dev,
			sizeof(struct langwell_dtd),
			DTD_ALIGNMENT,
			DMA_BOUNDARY);

	if (!dev->dtd_pool)
		return -ENOMEM;

	/* restore PCI state */
	pci_restore_state(pdev);

	/* enable IRQ handler */
	if (request_irq(pdev->irq, langwell_irq, IRQF_SHARED,
				driver_name, dev) != 0) {
		dev_err(&dev->pdev->dev, "request interrupt %d failed\n",
				pdev->irq);
		return -EBUSY;
	}
	dev->got_irq = 1;

	/* reset and start controller to run state */
	if (dev->stopped) {
		/* reset device controller */
		langwell_udc_reset(dev);

		/* reset ep0 dQH and endptctrl */
		ep0_reset(dev);

		/* start device if gadget is loaded */
		if (dev->driver)
			langwell_udc_start(dev);
	}

	/* reset USB status */
	dev->usb_state = USB_STATE_ATTACHED;
	dev->ep0_state = WAIT_FOR_SETUP;
	dev->ep0_dir = USB_DIR_OUT;

	dev_dbg(&dev->pdev->dev, "<--- %s()\n", __func__);
	return 0;
}



/* pci driver shutdown */
static void langwell_udc_shutdown(struct pci_dev *pdev)
{
	struct langwell_udc	*dev = pci_get_drvdata(pdev);
	u32			usbmode;

	dev_dbg(&dev->pdev->dev, "---> %s()\n", __func__);

	/* reset controller mode to IDLE */
	usbmode = readl(&dev->op_regs->usbmode);
	dev_dbg(&dev->pdev->dev, "usbmode = 0x%08x\n", usbmode);
	usbmode &= (~3 | MODE_IDLE);
	writel(usbmode, &dev->op_regs->usbmode);

	dev_dbg(&dev->pdev->dev, "<--- %s()\n", __func__);
}

/*-------------------------------------------------------------------------*/

static const struct pci_device_id pci_ids[] = { {
	.class =	((PCI_CLASS_SERIAL_USB << 8) | 0xfe),
	.class_mask =	~0,
	.vendor =	0x8086,
	.device =	0x0811,
	.subvendor =	PCI_ANY_ID,
	.subdevice =	PCI_ANY_ID,
}, {
	.class =	((PCI_CLASS_SERIAL_USB << 8) | 0xfe),
	.class_mask =	~0,
	.vendor =	0x8086,
	.device =	0x0829,
	.subvendor =	PCI_ANY_ID,
	.subdevice =	PCI_ANY_ID,
}, {
	.class =	((PCI_CLASS_SERIAL_USB << 8) | 0xfe),
	.class_mask =	~0,
	.vendor =	0x8086,
	.device =	0xE006,
	.subvendor =	PCI_ANY_ID,
	.subdevice =	PCI_ANY_ID,
}, { /* end: all zeroes */ }
};

MODULE_DEVICE_TABLE(pci, pci_ids);


static struct pci_driver langwell_pci_driver = {
	.name =		(char *) driver_name,
	.id_table =	pci_ids,

	.probe =	langwell_udc_probe,
	.remove =	langwell_udc_remove,

	/* device controller suspend/resume */
	.suspend =	langwell_udc_suspend,
	.resume =	langwell_udc_resume,

	.shutdown =	langwell_udc_shutdown,
};


#ifdef	OTG_TRANSCEIVER
static int intel_mid_start_peripheral(struct intel_mid_otg_xceiv *iotg)
{
	struct langwell_udc	*dev = the_controller;
	size_t			size;
	unsigned long		flags;

	dev_dbg(&dev->pdev->dev, "---> %s()\n", __func__);

	wake_lock(&dev->wake_lock);
	pm_runtime_get_sync(&dev->pdev->dev);

	/* exit PHY low power suspend */
	langwell_phy_low_power(dev, 0);

	/* enable SRAM caching if detected */
	if (dev->has_sram && !dev->got_sram)
		sram_init(dev);

	/* allocate device dQH memory */
	size = dev->ep_max * sizeof(struct langwell_dqh);
	dev_vdbg(&dev->pdev->dev, "orig size = %d\n", size);
	size = roundup(size, DQH_ALIGNMENT);
	dev->ep_dqh = dma_alloc_coherent(&dev->pdev->dev, size,
					&dev->ep_dqh_dma, GFP_KERNEL);
	if (!dev->ep_dqh) {
		dev_err(&dev->pdev->dev, "allocate dQH memory failed\n");
		return -ENOMEM;
	}
	dev->ep_dqh_size = size;
	dev_vdbg(&dev->pdev->dev, "ep_dqh_size = %d\n", dev->ep_dqh_size);

	/* create dTD dma_pool resource */
	dev->dtd_pool = dma_pool_create("langwell_dtd",
			&dev->pdev->dev,
			sizeof(struct langwell_dtd),
			DTD_ALIGNMENT,
			DMA_BOUNDARY);

	if (!dev->dtd_pool)
		return -ENOMEM;

	/* enable IRQ handler */
	if (request_irq(dev->pdev->irq, langwell_irq, IRQF_SHARED,
				driver_name, dev) != 0) {
		dev_err(&dev->pdev->dev, "request interrupt %d failed\n",
				dev->pdev->irq);
		return -EBUSY;
	}
	dev->got_irq = 1;

	spin_lock_irqsave(&dev->lock, flags);
	dev->vbus_active = 1;

	/* reset and start controller to run state */
	if (dev->stopped) {
		/* reset device controller */
		langwell_udc_reset(dev);

		/* reset ep0 dQH and endptctrl */
		ep0_reset(dev);

		/* reset USB status */
		dev->usb_state = USB_STATE_ATTACHED;
		dev->ep0_state = WAIT_FOR_SETUP;
		dev->ep0_dir = USB_DIR_OUT;

		/* Enable interrupts */
		langwell_udc_start(dev);

	}
	spin_unlock_irqrestore(&dev->lock, flags);
	dev->is_peripheral_start = 1;

	dev_dbg(&dev->pdev->dev, "<--- %s()\n", __func__);
	return 0;
}

static int intel_mid_stop_peripheral(struct intel_mid_otg_xceiv *iotg)
{
	struct langwell_udc	*dev = the_controller;
	unsigned long		flags;

	if (!dev->is_peripheral_start)
		return 1;

	dev_dbg(&dev->pdev->dev, "---> %s()\n", __func__);

	/* disable test mode */
	langwell_udc_stop_testmode(dev);

	/* disable interrupt and set controller to stop state */
	langwell_udc_stop(dev);

	/* diable IRQ handler */
	if (dev->got_irq)
		free_irq(dev->pdev->irq, dev);
	dev->got_irq = 0;

	spin_lock_irqsave(&dev->lock, flags);
	/* stop all usb activities */
	stop_activity(dev, dev->driver);
	spin_unlock_irqrestore(&dev->lock, flags);

	/* free dTD dma_pool and dQH */
	if (dev->dtd_pool) {
		dma_pool_destroy(dev->dtd_pool);
		dev->dtd_pool = NULL;
	}

	if (dev->ep_dqh) {
		dma_free_coherent(&dev->pdev->dev, dev->ep_dqh_size,
			dev->ep_dqh, dev->ep_dqh_dma);
		dev->ep_dqh = NULL;
	}

	/* release SRAM caching */
	if (dev->has_sram && dev->got_sram)
		sram_deinit(dev);

	spin_lock_irqsave(&dev->lock, flags);
	dev->vbus_active = 0;
	spin_unlock_irqrestore(&dev->lock, flags);

	pm_runtime_put(&dev->pdev->dev);
	wake_unlock(&dev->wake_lock);

	dev->is_peripheral_start = 0;

	dev_dbg(&dev->pdev->dev, "<--- %s()\n", __func__);
	return 0;
}

static int intel_mid_register_peripheral(struct pci_driver *peripheral_driver)
{
	struct usb_phy			*otg;
	struct intel_mid_otg_xceiv	*iotg;
	struct pci_dev			*pdev;
	int				retval;

	otg = usb_get_phy(USB_PHY_TYPE_USB2);
	if (otg == NULL)
		return -ENODEV;

	if (peripheral_driver == NULL || peripheral_driver->probe == NULL)
		return -EINVAL;

	pdev = to_pci_dev(otg->dev);
	retval = peripheral_driver->probe(pdev, peripheral_driver->id_table);
	if (retval) {
		dev_dbg(&pdev->dev, "client probe function failed\n");
		return retval;
	}

	iotg = otg_to_mid_xceiv(otg);

	iotg->start_peripheral = intel_mid_start_peripheral;
	iotg->stop_peripheral = intel_mid_stop_peripheral;

	langwell_udc_notify_otg(MID_OTG_NOTIFY_CLIENTADD);

	usb_put_phy(otg);

	return 0;
}


static void intel_mid_unregister_peripheral(struct pci_driver
		*peripheral_driver)
{
	struct usb_phy			*otg;
	struct intel_mid_otg_xceiv	*iotg;
	struct pci_dev			*pdev;

	otg = usb_get_phy(USB_PHY_TYPE_USB2);
	if (otg == NULL)
		return;

	if (peripheral_driver ==  NULL || peripheral_driver->remove == NULL)
		return;

	pdev = to_pci_dev(otg->dev);
	peripheral_driver->remove(pdev);

	iotg = otg_to_mid_xceiv(otg);

	iotg->start_peripheral = NULL;
	iotg->stop_peripheral = NULL;

	atomic_notifier_call_chain(&iotg->iotg_notifier,
				MID_OTG_NOTIFY_CLIENTREMOVE, iotg);

	usb_put_phy(otg);
}
#endif


static int __init init(void)
{
#ifdef	OTG_TRANSCEIVER
	return intel_mid_register_peripheral(&langwell_pci_driver);
#else
	return pci_register_driver(&langwell_pci_driver);
#endif
}
module_init(init);


static void __exit cleanup(void)
{
#ifdef	OTG_TRANSCEIVER
	intel_mid_unregister_peripheral(&langwell_pci_driver);
#else
	pci_unregister_driver(&langwell_pci_driver);
#endif
}
module_exit(cleanup);


MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("Xiaochen Shen <xiaochen.shen@intel.com>");
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");

