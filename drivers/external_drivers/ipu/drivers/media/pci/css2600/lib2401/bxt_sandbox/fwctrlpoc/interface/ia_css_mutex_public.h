/*
 * Support for Intel Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2010 - 2014 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#ifndef __IA_CSS_MUTEX_PUBLIC_H__
#define __IA_CSS_MUTEX_PUBLIC_H__

struct ia_css_mutex;

 /**
 * @brief ia_css_create_mutex() - Create and initialize a mutex
 * @param mutex_ref: reference to a mutex pointer
 * @return  error code (errno.h)
 */
 int ia_css_create_mutex(struct ia_css_mutex **mutex_ref);

 /**
 * @brief ia_css_delete_mutex() - Delete a mutex
 * @param mutex: reference to mutex
 * @return  error code (errno.h)
 */
 int ia_css_delete_mutex(struct ia_css_mutex *mutex);

 /**
 * @brief ia_css_acquire_mutex() - Acquire a mutex
 * @param mutex: reference to mutex
 * @return  error code (errno.h)
 */
 int ia_css_acquire_mutex(struct ia_css_mutex *mutex);

 /**
 * @brief ia_css_release_mutex() - Release a mutex
 * @param mutex: reference to mutex
 * @return  error code (errno.h)
 */
 int ia_css_release_mutex(struct ia_css_mutex *mutex);

#endif /* __IA_CSS_MUTEX_PUBLIC_H__ */
