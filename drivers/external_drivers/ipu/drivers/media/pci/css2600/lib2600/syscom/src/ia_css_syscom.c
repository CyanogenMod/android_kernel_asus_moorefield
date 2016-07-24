#include "ia_css_syscom.h"

#include "ia_css_syscom_context.h"
#include "ia_css_syscom_config_fw.h"

#include "assert_support.h"
#include "queue.h"
#include "send_port.h"
#include "recv_port.h"
#include "regmem_alloc.h"
#include "regmem_access.h"
#include "error_support.h"
#include "cpu_mem_support.h"

#include "queue_struct.h"
#include "send_port_struct.h"
#include "recv_port_struct.h"

#include "syscom_load_program.h"
#include "syscom_sp.h"

#include <vied/shared_memory_map.h>

#ifndef C_RUN
#include PROGMAP
#endif

#define _CAT3(a, b, c)		a ## b ## c
#define _start_pc(f)		_CAT3(HIVE_ADDR_, f, _entry)
#define sp_main_start_pc	_start_pc(SPMAIN)

//////////

struct ia_css_syscom_context*
ia_css_syscom_open(
	struct ia_css_syscom_config  *cfg
)
{
	struct ia_css_syscom_context* ctx;
	struct ia_css_syscom_config_fw fw_cfg;
	unsigned int config_reg;
	unsigned int i;

	const unsigned int id = 0;
	const unsigned int idm = 0;

	/* error handling */
	if (cfg == 0) {
		return 0;
	}
	/* check members of cfg */

	host_virtual_address_t prog_addr = syscom_alloc_program(PROGNAME);
	vied_virtual_address_t cell_prog_addr = shared_memory_map(id, idm, prog_addr);
	syscom_load_program(CELL, PROGNAME, cell_prog_addr, prog_addr);

	// static reg alloc
	config_reg = regmem_alloc();
	assert(config_reg == 0);

	// allocate context
	ctx = (struct ia_css_syscom_context*)ia_css_cpu_mem_alloc(sizeof(struct ia_css_syscom_context));
	if (ctx == 0) {
		regmem_free();
		return 0;
	}

/* fill the host context buffer */
	// ctx->mmio_base_address = (void *)cfg->mmio_base_address;
	// ctx->page_table_base_address = (void *)cfg->page_table_base_address;

	ctx->num_input_queues 		= cfg->num_input_queues;
	ctx->num_output_queues		= cfg->num_output_queues;

	// alloc queues
	ctx->input_queue = (struct sys_queue*) ia_css_cpu_mem_alloc(cfg->num_input_queues * sizeof(struct sys_queue));
	ctx->output_queue = (struct sys_queue*) ia_css_cpu_mem_alloc(cfg->num_output_queues * sizeof(struct sys_queue));
	// initialize the queues
	for (i=0; i<cfg->num_input_queues; i++)
		sys_queue_init(ctx->input_queue + i, cfg->input_queue_size, cfg->input_token_size);
	for (i=0; i<cfg->num_output_queues; i++)
		sys_queue_init(ctx->output_queue + i, cfg->output_queue_size, cfg->output_token_size);

	// alloc space for host ports
	ctx->send_port = ia_css_cpu_mem_alloc(cfg->num_input_queues * sizeof(struct send_port));
	ctx->recv_port = ia_css_cpu_mem_alloc(cfg->num_output_queues * sizeof(struct recv_port));
	// do not yet open the ports

	ctx->initialized = 1;
	ctx->running   =   1;


/* fill the firmware config struct */

	// allocate queue shared buffers
	ctx->input_queue_host_addr = shared_memory_alloc(idm, cfg->num_input_queues * sizeof(struct sys_queue));
	ctx->output_queue_host_addr = shared_memory_alloc(idm, cfg->num_output_queues * sizeof(struct sys_queue));
	// map queues
	ctx->input_queue_vied_addr  = shared_memory_map(id, idm, ctx->input_queue_host_addr);
	ctx->output_queue_vied_addr = shared_memory_map(id, idm, ctx->output_queue_host_addr);
	// fill(copy) fw queue buffers
	shared_memory_store(idm, ctx->input_queue_host_addr, ctx->input_queue, cfg->num_input_queues * sizeof(struct sys_queue));
	shared_memory_store(idm, ctx->output_queue_host_addr, ctx->output_queue, cfg->num_output_queues * sizeof(struct sys_queue));

	// allocate and map firmware specific data
	if (cfg->specific_addr && cfg->specific_size) {
		ctx->specific_host_addr = shared_memory_alloc(idm, cfg->specific_size);
		ctx->specific_vied_addr = shared_memory_map(id, idm, ctx->specific_host_addr);
		shared_memory_store(idm, ctx->specific_host_addr, cfg->specific_addr, cfg->specific_size);
	} else {
		ctx->specific_host_addr = 0;
		ctx->specific_vied_addr = 0;
	}

	fw_cfg.num_input_queues  = cfg->num_input_queues;
	fw_cfg.num_output_queues = cfg->num_output_queues;
	fw_cfg.input_queue       = ctx->input_queue_vied_addr;
	fw_cfg.output_queue      = ctx->output_queue_vied_addr;
	fw_cfg.specific_addr     = ctx->specific_vied_addr;
	fw_cfg.specific_size     = cfg->specific_size;

	// fw config struct alloc, map, and copy
	ctx->config_host_addr = shared_memory_alloc(idm, sizeof(struct ia_css_syscom_config_fw));
	ctx->config_vied_addr = shared_memory_map(id, idm, ctx->config_host_addr);
	shared_memory_store(idm, ctx->config_host_addr, &fw_cfg, sizeof(struct ia_css_syscom_config_fw));
	/* store firmware configuration address in reg 0 */
	regmem_store_32(0, ctx->config_vied_addr);

	// start the SP
	sp_start(sp_main_start_pc);

	return ctx;
}


extern int
ia_css_syscom_close(
	struct ia_css_syscom_context *ctx
) {
	static const int id = 0;
	static const int idm = 0;

	if (!sp_is_ready())
		return 1;

	shared_memory_unmap(id, idm, ctx->config_vied_addr);
	shared_memory_free(idm, ctx->config_host_addr);

	if (ctx->specific_vied_addr) {
		shared_memory_unmap(id, idm, ctx->specific_vied_addr);
		shared_memory_free(idm, ctx->specific_host_addr);
	}

	shared_memory_unmap(id, idm, ctx->input_queue_vied_addr);
	shared_memory_unmap(id, idm, ctx->output_queue_vied_addr);

	shared_memory_free(idm, ctx->input_queue_host_addr);
	shared_memory_free(idm, ctx->output_queue_host_addr);

	ia_css_cpu_mem_free(ctx->send_port);
	ia_css_cpu_mem_free(ctx->recv_port);

	ia_css_cpu_mem_free(ctx->input_queue);
	ia_css_cpu_mem_free(ctx->output_queue);

	ia_css_cpu_mem_free(ctx);

	regmem_free();

	return 0;
}


int ia_css_syscom_send_port_open(
	struct ia_css_syscom_context* ctx,
	unsigned int port
)
{
	/* check parameters */
	verifret(ctx != NULL, EFAULT);
	verifret(port < ctx->num_input_queues, EINVAL);

	/* check state */
	verifret(ctx->initialized, EINVAL);
	// verifret(ctx->send_port[i], EINVAL);

	/* initialize the port */
	send_port_open(ctx->send_port + port, ctx->input_queue + port);

	return 0;
}

int ia_css_syscom_send_port_close(
	struct ia_css_syscom_context* ctx,
	unsigned int port
)
{
	/* check parameters */
	verifret(ctx != NULL, EFAULT);
	verifret(port < ctx->num_input_queues, EINVAL);

	/* check state */
	verifret(ctx->initialized, EINVAL);
	// verifret(ctx->send_port[i], EINVAL);

	/* de-initialize the port */
	//send_port_open(ctx->send_port + port, ctx->input_queue + port);

	return 0;
}

int ia_css_syscom_send_port_available(
	struct ia_css_syscom_context* ctx,
	unsigned int port,
	unsigned int* num_tokens
)
{
	/* check params */
	verifret(ctx != NULL, EFAULT);
	verifret(port < ctx->num_input_queues, EINVAL);

	/* check state */
	verifret(ctx->initialized, EINVAL);
	// verifret(ctx->send_port[i], EINVAL);

	*num_tokens = send_port_available(ctx->send_port + port);

	return 0;
}

int ia_css_syscom_send_port_transfer(
	struct ia_css_syscom_context *ctx,
	unsigned int port,
	const void* token
)
{
	unsigned int num_tokens;

	/* check params */
	verifret(ctx != NULL, EFAULT);
	if (port > ctx->num_input_queues) return EINVAL;

	/* check state */
	verifret(ctx->initialized, EINVAL);
	verifret(port < ctx->num_input_queues, EINVAL);

	num_tokens = send_port_transfer(ctx->send_port + port, token);

	if (!num_tokens) return EWOULDBLOCK;

	return 0;
}

int ia_css_syscom_recv_port_open(
	struct ia_css_syscom_context* ctx,
	unsigned int port
)
{
	/* check parameters */
	verifret(ctx != NULL, EFAULT);
	verifret(port < ctx->num_output_queues, EINVAL);

	/* check state */
	verifret(ctx->initialized, EINVAL);
	// verifret(ctx->recv_port[i], EINVAL);

	/* initialize the port */
	recv_port_open(ctx->recv_port + port, ctx->output_queue + port);

	return 0;
}

int ia_css_syscom_recv_port_close(
	struct ia_css_syscom_context* ctx,
	unsigned int port
)
{
	/* check parameters */
	verifret(ctx != NULL, EFAULT);
	verifret(port < ctx->num_output_queues, EINVAL);

	/* check state */
	verifret(ctx->initialized, EINVAL);
	// verifret(ctx->recv_port[i], EINVAL);

	/* de-initialize the port */
	//recv_port_close(ctx->recv_port + port, ctx->output_queue + port);

	return 0;
}

/*
 * Get the number of responses in the response queue
 */
int
ia_css_syscom_recv_port_available(
	struct ia_css_syscom_context* ctx,
        unsigned int port,
	unsigned int* num_tokens
)
{
	/* check params */
	verifret(ctx != NULL, EFAULT);
	verifret(port < ctx->num_output_queues, EINVAL);

	/* check state */
	verifret(ctx->initialized, EINVAL);
	// verifret(ctx->recv_port[i], EINVAL);

	*num_tokens = recv_port_available(ctx->recv_port + port);

	return 0;
}


/*
 * Dequeue the head of the response queue
 * returns an error when the response queue is empty
 */
int
ia_css_syscom_recv_port_transfer(
	struct ia_css_syscom_context* ctx,
        unsigned int port,
	void* token
)
{
	unsigned int num_tokens;

	/* check params */
	verifret(ctx != NULL, EFAULT);
	if (port > ctx->num_output_queues) return EINVAL;

	/* check state */
	verifret(ctx->initialized, EINVAL);
	verifret(port < ctx->num_output_queues, EINVAL);

	num_tokens = recv_port_transfer(ctx->recv_port + port, token);

	if (!num_tokens) return EWOULDBLOCK;

	return 0;
}

