#ifndef IA_CSS_DEVPROXY_COMM_H_
#define IA_CSS_DEVPROXY_COMM_H_


#define TEST_SERVICE_NUM_CHANNELS 2

#define IA_CSS_DEVPROXY_SERVICE_ANR_NUM_CHANNELS 1
#define IA_CSS_DEVPROXY_SERVICE_IPF_NUM_CHANNELS 5
#define IA_CSS_DEVPROXY_SERVICE_CCM_NUM_CHANNELS 1
#define IA_CSS_DEVPROXY_SERVICE_DM_NUM_CHANNELS 1
#define IA_CSS_DEVPROXY_SERVICE_GTC_NUM_CHANNELS 1
#define IA_CSS_DEVPROXY_SERVICE_WBA_NUM_CHANNELS 1


#include "type_support.h"  /* uint8_t, uint32_t */
#include "ia_css_devproxy_test_service_comm.h"
#include "ia_css_devproxy_service_wba_comm.h"
#include "ia_css_devproxy_service_anr_comm.h"
#include "ia_css_devproxy_service_dm_comm.h"
#include "ia_css_devproxy_service_ccm_comm.h"
#include "ia_css_devproxy_service_gtc_comm.h"
#include "ia_css_devproxy_service_ipf_comm.h"


#define IA_CSS_DEVPROXY_BASE 0x0124

struct ia_css_devproxy_base {
	uint32_t dummy_value;
	// allocation of state for test service, we always presume channels, even if
	// only one... then event pump can take care of indexing to right status
	struct test_service_state_type test_service_0[TEST_SERVICE_NUM_CHANNELS];
	struct test_service_state_type test_service_1[TEST_SERVICE_NUM_CHANNELS];
	struct ia_css_devproxy_service_wba_state_type wba_service[IA_CSS_DEVPROXY_SERVICE_WBA_NUM_CHANNELS];
	struct ia_css_devproxy_service_anr_state_type anr_service[IA_CSS_DEVPROXY_SERVICE_ANR_NUM_CHANNELS];
	struct ia_css_devproxy_service_dm_state_type   dm_service[IA_CSS_DEVPROXY_SERVICE_DM_NUM_CHANNELS];
	struct ia_css_devproxy_service_ccm_state_type ccm_service[IA_CSS_DEVPROXY_SERVICE_CCM_NUM_CHANNELS];
	struct ia_css_devproxy_service_gtc_state_type gtc_service[IA_CSS_DEVPROXY_SERVICE_GTC_NUM_CHANNELS];
	struct ia_css_devproxy_service_ipf_state_type ipf_service[IA_CSS_DEVPROXY_SERVICE_IPF_NUM_CHANNELS];
};

/* WORKAROUND: for getting subsystemaccess to work */
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
/* WORKAROUND: end */

static inline vied_subsystem_address_t get_service_addr(
	unsigned int service_pid, /* [in] the service to address */
	unsigned int channel_num  /* [in] the channel to use */
	)
{
	vied_subsystem_address_t addr = 0;
	switch (service_pid) {
	case IA_CSS_DEVPROXY_CTRL_SERVICE_PID_DUMMY_0:
		addr = (HOST_SPP0_DMEM + IA_CSS_DEVPROXY_BASE) +
			offsetof(struct ia_css_devproxy_base, test_service_0)+
			(channel_num * sizeof(struct test_service_state_type));
		break;
	case IA_CSS_DEVPROXY_CTRL_SERVICE_PID_DUMMY_1:
		addr = (HOST_SPP0_DMEM + IA_CSS_DEVPROXY_BASE) +
			offsetof(struct ia_css_devproxy_base, test_service_1)+
			(channel_num * sizeof(struct test_service_state_type));
		break;
	case IA_CSS_DEVPROXY_CTRL_SERVICE_PID_WBA_BC:
		addr = (HOST_SPP0_DMEM + IA_CSS_DEVPROXY_BASE) +
			offsetof(struct ia_css_devproxy_base, wba_service)+
			(channel_num * sizeof(struct ia_css_devproxy_service_wba_state_type));
		break;
	case IA_CSS_DEVPROXY_CTRL_SERVICE_PID_ANR:
		addr = (HOST_SPP0_DMEM + IA_CSS_DEVPROXY_BASE) +
			offsetof(struct ia_css_devproxy_base, anr_service)+
			(channel_num * sizeof(struct ia_css_devproxy_service_anr_state_type));
		break;
	case IA_CSS_DEVPROXY_CTRL_SERVICE_PID_DM:
		addr = (HOST_SPP0_DMEM + IA_CSS_DEVPROXY_BASE) +
			offsetof(struct ia_css_devproxy_base, dm_service)+
			(channel_num * sizeof(struct ia_css_devproxy_service_dm_state_type));
		break;
	case IA_CSS_DEVPROXY_CTRL_SERVICE_PID_BD_CCM:
		addr = (HOST_SPP0_DMEM + IA_CSS_DEVPROXY_BASE) +
			offsetof(struct ia_css_devproxy_base, ccm_service)+
			(channel_num * sizeof(struct ia_css_devproxy_service_ccm_state_type));
		break;
	case IA_CSS_DEVPROXY_CTRL_SERVICE_PID_GTC:
		addr = (HOST_SPP0_DMEM + IA_CSS_DEVPROXY_BASE) +
			offsetof(struct ia_css_devproxy_base, gtc_service)+
			(channel_num * sizeof(struct ia_css_devproxy_service_gtc_state_type));
		break;
	case IA_CSS_DEVPROXY_CTRL_SERVICE_PID_IPF:
		addr = (HOST_SPP0_DMEM + IA_CSS_DEVPROXY_BASE) +
			offsetof(struct ia_css_devproxy_base, ipf_service)+
			(channel_num * sizeof(struct ia_css_devproxy_service_ipf_state_type));
		break;
	default:
		return addr;
	}
	return addr;
}

static inline void store_data_to_proxy(
	vied_subsystem_address_t addr,    /* [in] method/cmd */
	void        *data,        /* [in] pointer to data */
	unsigned int size         /* [in] how much data to store */
)
{
	unsigned int i;
	// TODO: assert on, or remove, if odd number of bytes can be written
	if ((size % 4 != 0) || (data == NULL)) {
		return;
	}
	for (i = 0; i < size/4; i++) {
		uint32_t *p = (uint32_t *)data;
		vied_subsystem_store_32(PSYS_SPP0,
			addr + (i*4),
			*(p+i));
	}
}

#endif /*IA_CSS_DEVPROXY_COMM_H_*/
