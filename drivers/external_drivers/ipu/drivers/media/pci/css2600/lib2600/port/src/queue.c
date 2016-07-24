#include "queue.h"
#include "type_support.h" // workaround because shared_memory_access.h does uses size_t
#include "vied/shared_memory_access.h"
#include "vied/shared_memory_map.h"
#include "regmem_alloc.h"
#include "regmem_access.h"

static const int mid = 0;
static const int sid = 0;

void
sys_queue_init(struct sys_queue* q, unsigned int size, unsigned int token_size)
{
	q->size         = size + 1;
	q->token_size   = token_size;
	// allocate the shared buffer (and map to CPU)
	q->host_address  = shared_memory_alloc(mid, q->size * q->token_size);
	// map the buffer to CSS
	q->vied_address	= shared_memory_map(sid, mid, q->host_address);

	// allocate the shared read and writer pointers
	q->wr_reg	= regmem_alloc();
	q->rd_reg	= regmem_alloc(); // return register number / address
	// initialize the shared read and write pointers
	regmem_store_32(q->wr_reg, 0);
	regmem_store_32(q->rd_reg, 0);
}

void
sys_queue_uninit(struct sys_queue* q)
{
	shared_memory_unmap(sid, mid, q->vied_address);
	shared_memory_free(mid, q->host_address);

	regmem_free(); // return register number / address
	regmem_free();
}

/*
struct sys_queue* sys_queue_alloc()
{
	return (struct sys_queue*)malloc(sizeof(sizeof(struct sys_queue)));
}

void sys_queue_free(struct sys_queue* q)
{
	free(q);
}

struct sys_queue* sys_queue_new(unsigned int size, unsigned int token_size)
{
	struct sys_queue* q = sys_queue_alloc(1);
	sys_queue_init(q, size, token_size);
	return q;
}

void sys_queue_delete(struct sys_queue* q)
{
	sys_queue_uninit(q);
	sys_queue_free(q);
}
*/
