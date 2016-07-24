#ifndef __CPU_MEM_SUPPORT_H_INCLUDED__
#define __CPU_MEM_SUPPORT_H_INCLUDED__

#if defined (__KERNEL__)

#include <linux/slab.h>

static inline void*
ia_css_cpu_mem_alloc(unsigned int size)
{
	return kmalloc(size, GFP_KERNEL);
}

static inline void*
ia_css_cpu_mem_alloc_page_aligned(unsigned int size)
{
	return ia_css_cpu_mem_alloc(size); // todo: align to page size
}

static inline int
ia_css_cpu_mem_protect(void* ptr, unsigned int size, int prot)
{
	// nothing here yet
}

static inline void*
ia_css_cpu_mem_copy(void* dst, const void* src, unsigned int size)
{
	return memcpy(dst, src, size); // available in kernel?
}

static inline void
ia_css_cpu_mem_free(void* ptr)
{
	kfree(ptr);
}
#else

#include <stdlib.h>
#include <string.h>
/* Needed for the MPROTECT */
#include <unistd.h>
#include <malloc.h>
#include <sys/types.h>
#include <sys/mman.h>


static inline void*
ia_css_cpu_mem_alloc(unsigned int size)
{
	return malloc(size);
}

static inline void*
ia_css_cpu_mem_alloc_page_aligned(unsigned int size)
{
	int pagesize; 
	pagesize = sysconf(_SC_PAGE_SIZE);
	return memalign(pagesize, size);
}

static inline void*
ia_css_cpu_mem_copy(void* dst, const void* src, unsigned int size)
{
	return memcpy(dst, src, size);
}

static inline void
ia_css_cpu_mem_free(void* ptr)
{
	free(ptr);
}

#endif

#endif /* __CPU_MEM_SUPPORT_H_INCLUDED__ */
