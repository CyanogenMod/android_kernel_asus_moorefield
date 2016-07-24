#include "ia_css_devproxy_ctrl.h"
#include "devproxy_ctrl_eq_config.h"
#include "vied_nci_eq_types.h"
#include "vied_nci_eq_device.h"

//TODO: trial/error for using cell nci, still not usable
//#define CELLNCI

#ifndef __INLINE_DEVPROXY_CTRL__
#include "ia_css_devproxy_ctrl_inline.h"
#endif


/*---------------------------------------------------------------------------*/
void ia_css_devproxy_ctrl_startup(void)
{
	/* make the device proxy ready for receiving events */
    static struct vied_nci_eq_device_config_t dconf = {DEVPROXY_EQ_NUM_QUEUES,
                                                IA_CSS_DEVPROXY_EQ_NUM_PRIORITIES};
    static struct vied_nci_eq_queue_config_t qconf[] = {{DEVPROXY_EQ_PID_END_QUEUE_0},
                                                 {DEVPROXY_EQ_PID_END_QUEUE_1},
                                                 {DEVPROXY_EQ_PID_END_QUEUE_2},
                                                 {DEVPROXY_EQ_PID_END_QUEUE_3},
                                                 {DEVPROXY_EQ_PID_END_QUEUE_4},
                                                 {DEVPROXY_EQ_PID_END_QUEUE_5},
                                                 {DEVPROXY_EQ_PID_END_QUEUE_6},
                                                 {DEVPROXY_EQ_PID_END_QUEUE_7}};
    vied_nci_eq_device_open(EVENT_QUEUE_SPP0_ID, &dconf, qconf);
}

/*
 * Terminate the device proxy
 */
void ia_css_devproxy_ctrl_shutdown(void)
{
	vied_nci_eq_send_port_t p = vied_nci_eq_send_port_open(EVENT_QUEUE_SPP0_ID);

	// reserve a token
	while (vied_nci_eq_reserve(p) == 0)
		;

	vied_nci_eq_send(p, IA_CSS_DEVPROXY_EQ_PRIORITY_SHUTDOWN, IA_CSS_DEVPROXY_CTRL_EVENT_SHUTDOWN);
}

#if 0

#ifdef CELLNCI
#include "vied/vied_nci_cell.h"
#endif

#if __VIED_CELL
# include "hive/cell_support.h"
#pragma warning disable
# warning TODO: HRT_MT_INT on ISP was wrongly defined and not defined for SP
#pragma warning enable
# undef HRT_MT_INT
# define HRT_MT_INT cmem
# ifndef HRT_MT_INT
#   define HRT_MT_INT cmem
#   warning TODO: cell support needs to define HRT_MT_INT
# endif
#endif

#ifndef HOST_SPP0_DMEM
# include "processing_system_system.h"
# define HOST_SPP0_DMEM 0x28000
# define PSYS_SPP0       processing_system_unps_logic_spp_tile0_sp
#pragma warning disable
# warning TODO: share proper configuration knowing with which cell to communicate
#pragma warning enable
#endif

#include "vied/vied_subsystem_access.h"
	// trial code for using different access methods to
	// write reference to memory
	// determine offset into memory
	//#define _hrt_master_to_slave_address_processing_system_unps_logic_spc_tile_sp_cmt_op_to_processing_system_unps_logic_spp_tile0_sp_sl_dmem_ip 0x28000
	// calc offset to address in global state:
	//     pid -> offset to array in global state
	//     channel -> index in the array
	//     function -> which data to update (offset to member in struct)
	// not supported function because vied_subsystem_store_8 is not impl
	// for cells
	vied_subsystem_store(PSYS_SPP0,
		HOST_SPP0_DMEM + IA_CSS_DEVPROXY_BASE + 4,
		ref, sizeof(ia_css_devproxy_ctrl_reference_t));
	for (i = 0; i < (sizeof(ia_css_devproxy_ctrl_reference_t)/4); i++) {
		uint32_t *p = (uint32_t *)ref;
		vied_subsystem_store_32(PSYS_SPP0,
			(HOST_SPP0_DMEM + IA_CSS_DEVPROXY_BASE + 4) + (i*4),
			*(p+i));
	}
#ifdef CELLNCI
	{
	vied_cell_t mycell = vied_cell_open(PSYS_SPP0, VIED_PROCESSING_SYSTEM_UNPS_LOGIC_SPP_TILE0_SP);
	vied_cell_address_t myaddr = vied_cell_address_new(VIED_ISP2600_BASE_DMEM, IA_CSS_DEVPROXY_BASE + 4);
	vied_cell_mem_store(myaddr, ref, sizeof(ia_css_devproxy_ctrl_reference_t));
	vied_cell_close(mycell);
	}
#endif
#endif

