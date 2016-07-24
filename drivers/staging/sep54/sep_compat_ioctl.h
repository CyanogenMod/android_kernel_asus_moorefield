/*
 * Copyright (C) 2013  Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the
 * Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */


#ifndef __SEP_COMPAT_IOCTL_H__
#define __SEP_COMPAT_IOCTL_H__

#include <linux/fs.h>

/**
 * \brief drm_compat_ioctl
 *
 * \param filp
 * \param cmd
 * \param arg
 * \return 0 on success or a negtive number on failure
 */
long sep_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

#endif
