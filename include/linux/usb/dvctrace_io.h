/*
 * Gadget Driver for Android DvC.Trace Debug Capability
 *
 * Copyright (C) 2008-2010, Intel Corporation.
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

#ifndef _F_DVC_TRACE_H_
#define _F_DVC_TRACE_H_

#include <linux/usb/gadget.h>	/* needed for struct usb_ep */

#define DVC_IO_MASK_ONLINE   (1<<0)
#define DVC_IO_MASK_TRANSFER (1<<1)
#define DVC_IO_MASK_ERROR    (1<<2)

#define DVC_IO_MASK_ONLINE_T (DVC_IO_MASK_ONLINE|DVC_IO_MASK_TRANSFER)

#define DVC_IO_MASK_ALL (DVC_IO_MASK_ONLINE|DVC_IO_MASK_TRANSFER|DVC_IO_MASK_ERROR)

#define DVC_IO_STATUS_SET(s, m) atomic_set(s, atomic_read(s)|(m))
#define DVC_IO_STATUS_CLEAR(s, m) atomic_set(s, atomic_read(s) & (~(m)))
#define DVC_IO_STATUS_GET(s, m) (atomic_read(s) & (m))


struct dvc_io {
	int (*on_dvc_setup)(atomic_t *dvc_status);
	int (*on_endpoint_creation)(struct usb_ep *, struct usb_function *);

	int (*on_start_transfer)(int);
	int (*on_disable_transfer)(void);

	void (*on_dvc_unbind)(void);
	void (*on_dvc_cleanup)(void);

	int (*is_io_enabled)(void);
};

int dvc_io_register(struct dvc_io *);
int dvc_io_unregister(struct dvc_io *);

#endif /* _F_DVC_TRACE_H_ */
