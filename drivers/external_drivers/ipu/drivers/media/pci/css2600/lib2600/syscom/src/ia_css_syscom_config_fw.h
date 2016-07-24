#ifndef __IA_CSS_SYSCOM_CONFIG_FW_H__
#define __IA_CSS_SYSCOM_CONFIG_FW_H__


#include "type_support.h"

/* firmware config: data that sent from the host to SP via DDR */
/* Cell copies data into a context */

struct ia_css_syscom_config_fw
{
	unsigned int num_input_queues;
	unsigned int num_output_queues;
	unsigned int input_queue; /* hmm_ptr / struct queue* */
	unsigned int output_queue; /* hmm_ptr / struct queue* */

	unsigned int specific_addr; /* vied virtual address */
	unsigned int specific_size; 

	//FW_VADDRESS ss_cfg; /* FW_VADDRESS for the subsystem cfg */
};


#endif /*__IA_CSS_SYSCOM_CONFIG_FW_H__*/
