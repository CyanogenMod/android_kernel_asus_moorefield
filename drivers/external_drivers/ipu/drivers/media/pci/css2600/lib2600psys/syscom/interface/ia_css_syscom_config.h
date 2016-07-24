#ifndef __IA_CSS_SYSCOM_CONFIG_H__
#define __IA_CSS_SYSCOM_CONFIG_H__

#include "type_support.h"
#include "ia_css_input_buffer.h"

/**
  * Parameter struct for ia_css_syscom_initialize
  */
struct ia_css_syscom_config
{
	// FW_VADDRESS mmio_base_address; /* FW_VADDRESS */
	// FW_VADDRESS page_table_base_address; /* FW_VADDRESS */
	// FW_VADDRESS firmware_address; /* FW_VADDRESS */

	unsigned int ssid;
	unsigned int mmid;
	unsigned int num_input_queues;
	unsigned int num_output_queues;
	unsigned int input_queue_size; /* max # tokens per queue */
	unsigned int output_queue_size; /* max # tokens per queue */
	unsigned int input_token_size; /* in bytes */
	unsigned int output_token_size; /* in bytes */

	// firmware-specific configuration data
	void* specific_addr;
	unsigned int specific_size;

	// to be reoved:
	int (*ss_cfg_prepare)(ia_css_input_buffer_css_address *, const void *); /* Callback to prepare sybsystem cfg for the FW */
	void * ss_cfg_param; /* param used in the callback for preparing subsystem sfg for the FW */
};

#endif /*__IA_CSS_SYSCOM_CONFIG_H__*/

