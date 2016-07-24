/*
 * Copyright (c) 2013 Intel Corporation. All Rights Reserved.
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

#ifndef CSS2600_WRAPPER_2401_H
#define CSS2600_WRAPPER_2401_H

struct device;
struct ia_css_env;
struct css2600_isys_iomem_filter;
struct firmware;

int css2600_isys_iomem_filter_add(struct device *dev, void __iomem *addr,
				  size_t size);
int css2600_isys_iomem_filters_add(struct device *dev, void __iomem **addr,
				   unsigned int naddr, size_t size);
void css2600_isys_iomem_filter_remove(struct device *dev);
int css2600_wrapper_register_buffer(dma_addr_t iova, void *addr, size_t bytes);
void css2600_wrapper_unregister_buffer(dma_addr_t iova);

void css2600_wrapper_init(void __iomem *basepsys, void __iomem *baseisys,
			  const struct firmware *fw);
int css2600_wrapper_set_domain(struct iommu_domain *domain);
int css2600_wrapper_set_device(struct device *dev);
int css2600_wrapper_iommu_add(void);
int css2600_wrapper_init_done(void);
int css2600_wrapper_set_iommus(unsigned int iommus);

#endif /* CSS2600_WRAPPER_2401_H */
