#ifndef __BUFFER_H__
#define __BUFFER_H__

#include "type_support.h" // workaround: needed because <vied/shared_memory_map.h> uses size_t
#include "vied/shared_memory_map.h"

static const int sid = 0;
static const int mid = 0;

typedef enum {
	buffer_unmapped,	// buffer is not accessible by cpu, nor css
	buffer_write,		// output buffer: css has write access
				// input  buffer: cpu has write access
	buffer_read, 		// input  buffer: css has read access
				// output buffer: cpu has read access
	buffer_cpu, 		// shared buffer: cpu has read and write access
	buffer_css		// shared buffer: css has read and write access
} buffer_state;

struct ia_css_buffer_s
{
	unsigned int		size;		// number of bytes bytes allocated
	host_virtual_address_t	mem;		// allocated virtual memory object
	vied_virtual_address_t	css_address;	// virtual address to be used on css/firmware
	void*			cpu_address;	// virtual address to be used on cpu/host
	buffer_state		state;
};

typedef struct ia_css_buffer_s* ia_css_buffer_t;

ia_css_buffer_t	ia_css_buffer_alloc(unsigned int size);
void		ia_css_buffer_free(ia_css_buffer_t b);

#endif /*__BUFFER_H__*/
