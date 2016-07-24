#include "ia_css_devproxy_service_ipf_comm.h"
#include "ia_css_devproxy_comm.h"

#if __VIED_CELL
# include "hive/cell_support.h"
#pragma warning disable
#pragma message("TODO: HRT_MT_INT on ISP was wrongly defined and not defined for SP")
#pragma warning enable
# undef HRT_MT_INT
# define HRT_MT_INT cmem
# ifndef HRT_MT_INT
#   define HRT_MT_INT cmem
#pragma message("TODO: cell support needs to define HRT_MT_INT")
# endif
#endif

#ifndef HOST_SPP0_DMEM
# include "processing_system_system.h"
# define HOST_SPP0_DMEM 0x28000
# define PSYS_SPP0       processing_system_unps_logic_spp_tile0_sp
#pragma message("TODO: share proper configuration knowing with which cell to communicate")
#endif

#include "vied/vied_subsystem_access.h"

#ifndef VOLATILE
# define VOLATILE
# warning VOLATILE not available on host
#endif

void ia_css_devproxy_service_ipf_init_ref(
	unsigned int service_pid, /* [in] the service to address */
	unsigned int channel_num, /* [in] the channel to use */
	ia_css_devproxy_ctrl_reference_t *init_ref
	)
{
	vied_subsystem_address_t addr = get_service_addr(service_pid, channel_num);
	addr += offsetof(struct ia_css_devproxy_service_ipf_state_type, init_ref);
	store_data_to_proxy(
		addr,
		init_ref,
		sizeof(*init_ref)) VOLATILE;
}

void ia_css_devproxy_service_ipf_route_out(
	unsigned int service_pid, /* [in] the service to address */
	unsigned int channel_num, /* [in] the channel to use */
	uint32_t ack_addr,
	uint32_t ack_token
	)
{
	vied_subsystem_address_t addr = get_service_addr(service_pid, channel_num);
	store_data_to_proxy(
		addr + offsetof(struct ia_css_devproxy_service_ipf_state_type, ack_addr),
		&ack_addr,
		sizeof(ack_addr)) VOLATILE;
	store_data_to_proxy(
		addr + offsetof(struct ia_css_devproxy_service_ipf_state_type, ack_token),
		&ack_token,
		sizeof(ack_token)) VOLATILE;
}

void ia_css_devproxy_service_ipf_dma_config(
	unsigned int service_pid, /* [in] the service to address */
	unsigned int channel_num, /* [in] the channel to use */
	ia_css_tproxy_chan_handle_t *chan_handle,
	struct ia_css_tproxy_transfer_config *transfer_config,
	struct ia_css_tproxy_terminal_config *terminal_config)
{
	vied_subsystem_address_t addr = get_service_addr(service_pid, channel_num);
	store_data_to_proxy(
		addr + offsetof(struct ia_css_devproxy_service_ipf_state_type, chan_handle),
		chan_handle,
		sizeof(*chan_handle)) VOLATILE;
	store_data_to_proxy(
		addr + offsetof(struct ia_css_devproxy_service_ipf_state_type, transfer_config),
		transfer_config,
		sizeof(*transfer_config)) VOLATILE;
	store_data_to_proxy(
		addr + offsetof(struct ia_css_devproxy_service_ipf_state_type, terminal_config),
		terminal_config,
		sizeof(*terminal_config)) VOLATILE;

}

void ia_css_devproxy_service_ipf_buffer_config(
	unsigned int service_pid, /* [in] the service to address */
	unsigned int channel_num, /* [in] the channel to use */
	uint32_t buffer_addr,
	uint32_t buffer_start_addr,
	uint32_t buffer_end_addr,
	uint32_t buffer_stride,
	uint32_t buffer_num)
{
	vied_subsystem_address_t addr = get_service_addr(service_pid, channel_num);
	store_data_to_proxy(
		addr + offsetof(struct ia_css_devproxy_service_ipf_state_type, buffer_addr),
		&buffer_addr,
		sizeof(buffer_addr)) VOLATILE;
	store_data_to_proxy(
		addr + offsetof(struct ia_css_devproxy_service_ipf_state_type, buffer_start_addr),
		&buffer_start_addr,
		sizeof(buffer_start_addr)) VOLATILE;
	store_data_to_proxy(
		addr + offsetof(struct ia_css_devproxy_service_ipf_state_type, buffer_end_addr),
		&buffer_end_addr,
		sizeof(buffer_end_addr)) VOLATILE;
	store_data_to_proxy(
		addr + offsetof(struct ia_css_devproxy_service_ipf_state_type, buffer_stride),
		&buffer_stride,
		sizeof(buffer_stride)) VOLATILE;
	store_data_to_proxy(
		addr + offsetof(struct ia_css_devproxy_service_ipf_state_type, buffer_num),
		&buffer_num,
		sizeof(buffer_num)) VOLATILE;

}


