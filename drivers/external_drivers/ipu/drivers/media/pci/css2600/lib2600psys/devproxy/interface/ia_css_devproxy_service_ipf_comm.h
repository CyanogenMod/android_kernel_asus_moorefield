#ifndef IA_CSS_DEVPROXY_SERVICE_IPF_COMM_H_
#define IA_CSS_DEVPROXY_SERVICE_IPF_COMM_H_

#include "type_support.h"  /* uint8_t, uint32_t */
#include "ia_css_devproxy_ctrl.h"
#include "ia_css_tproxy_client.h"

#define IA_CSS_DEVPROXY_SERVICE_IPF_DMA_NOT_USED 0xffffffff

/* IPF service state structure, per channel */
struct ia_css_devproxy_service_ipf_state_type {
	uint32_t initialized;
	uint32_t num_configs;
	uint32_t used_configs;
	ia_css_devproxy_ctrl_reference_t init_ref;
	uint32_t ack_addr;
	uint32_t ack_token;
	/* which sid / stream id to use */
	uint32_t channel_id; /**< channel / stream id assigned to this channel, see ::vied_nci_infeeder_stream_id_t, fixed mapping */
	/* dma stuff to hide large frames from service user */
	ia_css_tproxy_chan_handle_t chan_handle; /**< which channel handle to use, IA_CSS_DEVPROXY_SERVICE_IPF_DMA_NOT_USED for not-used */
	struct ia_css_tproxy_transfer_config transfer_config;
	struct ia_css_tproxy_terminal_config terminal_config;
	/* buffer configuration, circular buffer */
	uint32_t buffer_addr; /**< curr address of buffer to process */
	uint32_t buffer_start_addr; /**< start address of buffer to process */
	uint32_t buffer_end_addr; /**< end address of buffer */
	uint32_t buffer_stride; /**< buffer stride, num blocks in circular buffer is end - start / stride */
	/* configuration / status per next */
	uint32_t buffer_num; /**< number of blocks to transfer per next */
	uint32_t buffer_count; /**< actual nr of blocks transfered for curr next */
};

void ia_css_devproxy_service_ipf_init_ref(
	unsigned int service_pid, /* [in] the service to address */
	unsigned int channel_num, /* [in] the channel to use */
	ia_css_devproxy_ctrl_reference_t *init_ref);

void ia_css_devproxy_service_ipf_dma_config(
	unsigned int service_pid, /* [in] the service to address */
	unsigned int channel_num, /* [in] the channel to use */
	ia_css_tproxy_chan_handle_t *chan_handle,
	struct ia_css_tproxy_transfer_config *transfer_config,
	struct ia_css_tproxy_terminal_config *terminal_config);

void ia_css_devproxy_service_ipf_buffer_config(
	unsigned int service_pid, /* [in] the service to address */
	unsigned int channel_num, /* [in] the channel to use */
	uint32_t buffer_addr,
	uint32_t buffer_start_addr,
	uint32_t buffer_end_addr,
	uint32_t buffer_stride,
	uint32_t buffer_num);

void ia_css_devproxy_service_ipf_route_out(
	unsigned int service_pid, /* [in] the service to address */
	unsigned int channel_num, /* [in] the channel to use */
	uint32_t ack_addr,
	uint32_t ack_token);


#endif /*IA_CSS_DEVPROXY_SERVICE_IPF_COMM_H_*/
