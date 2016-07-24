#ifndef __RECV_PORT_STRUCT_H__
#define __RECV_PORT_STRUCT_H__

#include "buffer_type.h"

struct recv_port
{
	buffer_address buffer;
	unsigned int size;
	unsigned int token_size;
	unsigned int wr_reg; // reg no in subsystem's regmem
	unsigned int rd_reg;
};

#endif /*__RECV_PORT_STRUCT_H__*/
