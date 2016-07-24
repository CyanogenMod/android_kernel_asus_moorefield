#ifndef _IA_CSS_TPROXY_CMEM_H_
#define _IA_CSS_TPROXY_CMEM_H_

#include <error_support.h>
#include <vied_bit.h>
#include "ia_css_tproxy_local.h"

static uint8_t cache_memory[IA_CSS_TPROXY_CACHE_MEMORY_SIZE];

static uint32_t next_available_memory = 0;

struct ia_css_tproxy_cmem_group {
	uint32_t addr;
	uint8_t size;
	uint8_t is_used;
};

static struct ia_css_tproxy_cmem_group cmem_group_list[IA_CSS_N_TPROXY_CMEM_GROUPS];

STORAGE_CLASS_INLINE uint32_t ia_css_tproxy_cmem_alloc(
	const uint32_t size)
{
	uint32_t i;
	IA_CSS_TPROXY_DEBUG_INFO("cmem_alloc() enter\n");
	IA_CSS_TPROXY_DEBUG_INFO("memory size: ");
	IA_CSS_TPROXY_DEBUG_DUMP2(next_available_memory, size);

	/* Search a freed channel group which has been allocated
	 * previously for the same size  so it can be recycled.
	 */
	for (i = 0; i< IA_CSS_N_TPROXY_CMEM_GROUPS; i++) {
		if (cmem_group_list[i].size  == size &&
			cmem_group_list[i].is_used == false) {
			cmem_group_list[i].is_used == true;
			return cmem_group_list[i].addr;
		}
	}

	/* Find a new group if no group can be recycled */
	for (i = 0; i < IA_CSS_N_TPROXY_CMEM_GROUPS; i++) {
		if (cmem_group_list[i].size == 0) {
			break;
		}
	}

	IA_CSS_TPROXY_DEBUG_ASSERT((next_available_memory + size) <=
		IA_CSS_TPROXY_CACHE_MEMORY_SIZE);
	if ((next_available_memory + size > IA_CSS_TPROXY_CACHE_MEMORY_SIZE) ||
		(i == IA_CSS_N_TPROXY_CMEM_GROUPS))
	{
		errno = ENOMEM;
		IA_CSS_TPROXY_DEBUG_ERROR("cmem_alloc() failed\n");
		return 0;
	}

	/* Create a new group */
	cmem_group_list[i].addr = next_available_memory +
		(uint32_t)cache_memory + IA_CSS_TPROXY_SERVER_DMEM_BASE;
	cmem_group_list[i].size = size;
	cmem_group_list[i].is_used = true;

	next_available_memory += size;
	IA_CSS_TPROXY_DEBUG_INFO("cmem_alloc() exit\n");

	return cmem_group_list[i].addr;
}

STORAGE_CLASS_INLINE int ia_css_tproxy_cmem_free(
	const uint32_t addr)
{
	uint32_t i = 0;

	IA_CSS_TPROXY_DEBUG_ASSERT(addr);
	/* leave the size so the group can be recycled
	 * */
	for (i = 0; i < IA_CSS_N_TPROXY_CMEM_GROUPS; i++)
	{
		if (cmem_group_list[i].addr == addr) {
			cmem_group_list[i].is_used = false;
			return 0;
		}
	}

	return EINVAL;
}

#endif /*_IA_CSS_TPROXY_CMEM_H_*/
