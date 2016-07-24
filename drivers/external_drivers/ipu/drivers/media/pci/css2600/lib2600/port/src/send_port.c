// implemented:
#include "send_port.h"

// used:
#include "queue.h"
#include "queue_struct.h"
#include "send_port_struct.h"
#include "regmem_access.h"
#include "buffer_access.h"
#include "modadd.h"

void
send_port_open(struct send_port* p, struct sys_queue* q)
{
	p->size         = q->size;
	p->token_size   = q->token_size;
	p->wr_reg	= q->wr_reg;
	p->rd_reg	= q->rd_reg;
#ifdef __VIED_CELL
	p->buffer       = q->vied_address;
#else
	p->buffer       = q->host_address;
#endif
}

static inline unsigned int
send_port_index(struct send_port* p, unsigned int i)
{
	unsigned int wr = regmem_load_32(p->wr_reg);
	return OP_std_modadd(wr, i, p->size);
}

////

unsigned int
send_port_available(struct send_port* p)
{
	unsigned int rd   = regmem_load_32(p->rd_reg);
	unsigned int wr   = regmem_load_32(p->wr_reg);
	return OP_std_modadd(rd, -(wr+1), p->size);
}

static inline void
send_port_copy(struct send_port* p, unsigned int i, const void* data)
{
	unsigned int wr   = send_port_index(p, i);
	unsigned int token_size = p->token_size;
	buffer_address addr = p->buffer + (wr * token_size);
	buffer_store(addr, data, token_size);
}

static inline void
send_port_release(struct send_port* p, unsigned int i)
{
	unsigned int wr = send_port_index(p, i);
	regmem_store_32(p->wr_reg, wr);
}

////

unsigned int
send_port_transfer(struct send_port* p, const void* data)
{
	if (!send_port_available(p)) return 0;
	send_port_copy(p, 0, data);
	send_port_release(p, 1);
	return 1;
}
