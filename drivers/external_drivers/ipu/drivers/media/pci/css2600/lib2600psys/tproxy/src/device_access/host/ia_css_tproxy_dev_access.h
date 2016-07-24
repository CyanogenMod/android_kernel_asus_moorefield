#ifndef _IA_CSS_TPROXY_DEV_ACCESS_H_
#define _IA_CSS_TPROXY_DEV_ACCESS_H_
#include <type_support.h>
#include <storage_class.h>
#include "ia_css_tproxy_debug.h"
#include <vied/vied_subsystem_access.h>
#define IA_CSS_TPROXY_SUBSYSTEM_PSYS0 0

STORAGE_CLASS_INLINE void ia_css_tproxy_mem_store_32(
	const uint32_t addr,
	const uint32_t val)
{
	IA_CSS_TPROXY_DEBUG_ASSERT(addr);
	vied_subsystem_store_32(IA_CSS_TPROXY_SUBSYSTEM_PSYS0, addr, val);
}

STORAGE_CLASS_INLINE uint32_t ia_css_tproxy_mem_load_32(
	const uint32_t addr)
{
	IA_CSS_TPROXY_DEBUG_ASSERT(addr);
	return vied_subsystem_load_32(IA_CSS_TPROXY_SUBSYSTEM_PSYS0, addr);
}

#endif /*_IA_CSS_TPROXY_DEV_ACCESS_H_*/
