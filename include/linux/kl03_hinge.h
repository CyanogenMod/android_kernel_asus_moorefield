/*
 * Copyright (c) 2014 Google, Inc.
 * Corey Tabaka <eieio@google.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef _LINUX_KL03_HINGE_H
#define _LINUX_KL03_HINGE_H

#define KL03_HINGE_HINGE_GPIO "hinge_closed"
#define KL03_HINGE_PHOTO_GPIO "cam_but_fw_rec"
#define KL03_HINGE_PREVIEW_GPIO "preview_trigd"
#define KL03_HINGE_RESET_GPIO "mcu_reset"
#define KL03_HINGE_BOOTCFG_GPIO "mcu_bootccfg"

struct hinge_platform_data {
	int irq_hinge_gpio;
	int irq_photo_gpio;
	int irq_preview_gpio;

	int bootcfg_gpio;
	int reset_gpio;
	/* active level (high/low) of bootcfg and reset gpios */
	int active_polarity;
};

#endif /* _LINUX_KL03_HINGE_H */


