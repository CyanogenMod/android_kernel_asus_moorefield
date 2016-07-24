/*
 *  Extcon USB
 *
 * Copyright (C) 2013 Intel Corp.
 * Author: Vamsi krishna Kalidindi <vamsi.krishnax.kalidindi@intel.com>
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

#ifndef __EXTCON_USB_H__
#define __EXTCON_USB_H__

#if IS_ENABLED(CONFIG_EXTCON_USB)
void usb_extcon_headset_report(u32 state);
#else
void usb_extcon_headset_report(u32 state)
{
}
#endif

enum {
	USB_HEADSET_PULL_OUT = 0,
	USB_HEADSET_ANLG = 1,
	USB_HEADSET_DGTL = 2,
};

#endif
