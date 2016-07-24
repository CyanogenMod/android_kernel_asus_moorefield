// implemented:
#include "recv_port.h"

// used:
#include "queue.h"
#include "queue_struct.h"
#include "recv_port_struct.h"
#include "regmem_access.h"
#include "buffer_access.h"
#include "modadd.h"

void
recv_port_open(struct recv_port* p, struct sys_queue* q)
{
  p->size   = q->size;
  p->token_size = q->token_size;
  p->wr_reg = q->wr_reg;
  p->rd_reg = q->rd_reg;

#ifdef __VIED_CELL
  p->buffer = q->vied_address;
#else
  p->buffer = q->host_address;
#endif
}

static inline unsigned int
recv_port_index(struct recv_port* p, unsigned int i)
{
  unsigned int rd = regmem_load_32(p->rd_reg);
  return OP_std_modadd(rd, i, p->size);
}

////

unsigned int
recv_port_available(struct recv_port* p)
{
  unsigned int wr = regmem_load_32(p->wr_reg);
  unsigned int rd = regmem_load_32(p->rd_reg);
  return OP_std_modadd(wr, -rd, p->size);
}

static inline void
recv_port_copy(struct recv_port* p, unsigned int i, void* data)
{
  unsigned int rd   = recv_port_index(p, i);
  unsigned int token_size = p->token_size;
  buffer_address addr = p->buffer + (rd * token_size);
  buffer_load(addr, data, token_size);
}

static inline void
recv_port_release(struct recv_port* p, unsigned int i)
{
  unsigned int rd = recv_port_index(p, i);
  regmem_store_32(p->rd_reg, rd);
}

unsigned int
recv_port_transfer(struct recv_port* p, void* data)
{
	if (!recv_port_available(p)) return 0;
	recv_port_copy(p, 0, data);
	recv_port_release(p, 1);
	return 1;
}
