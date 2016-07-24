#ifndef __SYS_QUEUE_STRUCT_H__
#define __SYS_QUEUE_STRUCT_H__

// queue description, shared between sender and receiver

#include "type_support.h"

#ifdef __VIED_CELL
typedef struct {uint32_t v[2];} host_buffer_address_t;
#else
typedef uint64_t		host_buffer_address_t;
#endif

typedef uint32_t		vied_buffer_address_t;


struct sys_queue
{
	host_buffer_address_t host_address;
	vied_buffer_address_t vied_address;
	unsigned int size;
	unsigned int token_size;
	unsigned int wr_reg; // reg no in subsystem's regmem
	unsigned int rd_reg;
	unsigned int _align;
};

#endif /*__QUEUE_STRUCT_H__*/
