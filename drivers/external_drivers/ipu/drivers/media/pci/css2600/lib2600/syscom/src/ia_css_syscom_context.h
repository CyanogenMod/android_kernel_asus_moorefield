#ifndef __IA_CSS_SYSCOM_CONTEXT_H__
#define __IA_CSS_SYSCOM_CONTEXT_H__

#include <vied/shared_memory_access.h>

/* host context */
struct ia_css_syscom_context
{
	// vied_subsystem_t		subsystem;
	// void*	base_address; /* why save this? */
	// void*	mmio_base_address;
	// void*	page_table_base_address; /* for all mmu's*/

	unsigned int num_input_queues;
	unsigned int num_output_queues;

	struct sys_queue* input_queue;  /* array of input queues (from host to SP) */
	struct sys_queue* output_queue; /* array of output queues (from SP to host) */

	struct send_port* send_port;
	struct recv_port* recv_port;

	int initialized;
	int	running;

	// state buffer (saved regs)
	// unsigned int regmem[16];

	host_virtual_address_t input_queue_host_addr;
	host_virtual_address_t output_queue_host_addr;
	host_virtual_address_t specific_host_addr;
	host_virtual_address_t config_host_addr;

	vied_virtual_address_t input_queue_vied_addr;
	vied_virtual_address_t output_queue_vied_addr;
	vied_virtual_address_t specific_vied_addr;
	vied_virtual_address_t config_vied_addr;
};


#endif /*__IA_CSS_SYSCOM_CONTEXT_H__*/
