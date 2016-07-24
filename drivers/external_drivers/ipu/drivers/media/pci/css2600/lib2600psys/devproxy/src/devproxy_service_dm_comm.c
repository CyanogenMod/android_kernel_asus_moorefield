#include "ia_css_devproxy_service_dm_comm.h"
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


void ia_css_devproxy_service_dm_init_ref(
       unsigned int service_pid, /* [in] the service to address */
       unsigned int channel_num, /* [in] the channel to use */
       ia_css_devproxy_ctrl_reference_t *init_ref
       )
{
	vied_subsystem_address_t addr = get_service_addr(service_pid, channel_num);
	addr += offsetof(struct ia_css_devproxy_service_dm_state_type, init_ref);
	store_data_to_proxy(
		addr,
		init_ref,
		sizeof(*init_ref)) VOLATILE;
}

void ia_css_devproxy_service_dm_route_out(
	unsigned int service_pid, /* [in] the service to address */
	unsigned int channel_num, /* [in] the channel to use */
	uint32_t ack_addr,
	uint32_t ack_token
	)
{
	vied_subsystem_address_t addr = get_service_addr(service_pid, channel_num);
	store_data_to_proxy(
		addr + offsetof(struct ia_css_devproxy_service_dm_state_type, ack_addr),
		&ack_addr,
		sizeof(ack_addr)) VOLATILE;
	store_data_to_proxy(
		addr + offsetof(struct ia_css_devproxy_service_dm_state_type, ack_token),
		&ack_token,
		sizeof(ack_token)) VOLATILE;
}

void ia_css_devproxy_service_dm_config(
       unsigned int service_pid, /* [in] the service to address */
       unsigned int channel_num, /* [in] the channel to use */
       uint32_t num_lines)
{
	vied_subsystem_address_t addr = get_service_addr(service_pid, channel_num);
	store_data_to_proxy(
		addr + offsetof(struct ia_css_devproxy_service_dm_state_type, num_lines),
		&num_lines,
		sizeof(num_lines)) VOLATILE;
}

