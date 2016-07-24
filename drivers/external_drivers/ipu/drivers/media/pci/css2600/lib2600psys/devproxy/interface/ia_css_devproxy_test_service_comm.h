#ifndef IA_CSS_DEVPROXY_TEST_SERVICE_COMM_H_
#define IA_CSS_DEVPROXY_TEST_SERVICE_COMM_H_

#include "type_support.h"  /* uint8_t, uint32_t */
#include "ia_css_devproxy_ctrl.h"

/* test service state structure */
struct test_service_state_type {
	ia_css_devproxy_ctrl_reference_t init_ref;
	uint32_t initialized;
	uint32_t num_configs;
	uint32_t used_configs;
	uint32_t next_config;
	uint32_t ack_addr;
	uint32_t ack_token;
};

void ia_css_devproxy_test_service_init_ref(
	unsigned int service_pid, /* [in] the service to address */
	unsigned int channel_num, /* [in] the channel to use */
	ia_css_devproxy_ctrl_reference_t *init_ref);

void ia_css_devproxy_test_service_route_out(
	unsigned int service_pid, /* [in] the service to address */
	unsigned int channel_num, /* [in] the channel to use */
	uint32_t ack_addr,
	uint32_t ack_token);


#endif /*IA_CSS_DEVPROXY_TEST_SERVICE_COMM_H_*/
