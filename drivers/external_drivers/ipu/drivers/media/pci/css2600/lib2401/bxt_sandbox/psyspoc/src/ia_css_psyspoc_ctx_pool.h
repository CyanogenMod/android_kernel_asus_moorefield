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

#ifndef __IA_CSS_PSYSPOC_CTX_POOL__
#define __IA_CSS_PSYSPOC_CTX_POOL__

#include "type_support.h"

 /**
 * @brief Init psyspoc specific contexts
 * @param none
 * @return non-zero on error
 */
int32_t ia_css_psyspoc_ctx_pool_init(void);

 /**
 * @brief Free psyspoc specific contexts
 * @param none
 * @return none
 */
void ia_css_psyspoc_ctx_pool_uninit(void);

 /**
 * @brief Acquired psyspoc specific host context
 * @param none
 * @return Acquired poc context buffer (host only)
 */
void *ia_css_psyspoc_ctx_pool_acquire_buffer(void);

 /**
 * @brief Free psyspoc specific host context
 * @param poc context buffer to be freed.
 * @return none
 */
void ia_css_psyspoc_ctx_pool_release_token(uint64_t token);

 /**
 * @brief Get host address from token.
 * @param Token to be mapped
 * @return host address corresponding to this token
 */
void *ia_css_psyspoc_ctx_pool_buffer_map(uint64_t token);

 /**
 * @brief Get token from host address.
 * @param host address to be mapped
 * @return token corresponding to the host address
 */
uint64_t ia_css_psyspoc_ctx_pool_token_map(void *host_buf);

#endif /* __IA_CSS_PSYSPOC_CTX_POOL__ */
