#ifndef _IA_CSS_TPROXY_SERVER_H_
#define _IA_CSS_TPROXY_SERVER_H_

#include <storage_class.h>
#include <vied_nci_dma.h>
#include "ia_css_tproxy_global.h"
#include "ia_css_tproxy_eq.h"

#ifndef DMA_DEV_EXT0_NUM_LOGICAL_CHANNELS
#error  DMA_DEV_EXT0_NUM_LOGICAL_CHANNELS not defined specify number of \
	logical channels to allocate for this device
#endif
#ifndef DMA_DEV_EXT1R_NUM_LOGICAL_CHANNELS
#error  DMA_DEV_EXT1R_NUM_LOGICAL_CHANNELS not defined specify number of \
	logical channels to allocate for this device
#endif
#ifndef DMA_DEV_EXT1W_NUM_LOGICAL_CHANNELS
#error  DMA_DEV_EXT1W_NUM_LOGICAL_CHANNELS not defined specify number of \
	logical channels to allocate for this device
#endif

#ifndef DMA_DEV_INT_NUM_LOGICAL_CHANNELS
#error  DMA_DEV_INT_NUM_LOGICAL_CHANNELS not defined specify number of \
	logical channels to allocate for this device
#endif

#ifndef DMA_DEV_ISA_NUM_PHYSICAL_CHANNELS
#error  DMA_DEV_ISA_NUM_PHYSICAL_CHANNELS not defined specify number of \
	logical channels to allocate for this device
#endif

#ifndef DMA_DEV_IPFD_NUM_PHYSICAL_CHANNELS
#error  DMA_DEV_IPFD_NUM_PHYSICAL_CHANNELS not defined specify number of \
	logical channels to allocate for this device
#endif


#define IA_CSS_N_TPROXY_CHAN	(DMA_DEV_EXT0_NUM_LOGICAL_CHANNELS + \
				DMA_DEV_EXT1R_NUM_LOGICAL_CHANNELS +\
				DMA_DEV_EXT1W_NUM_LOGICAL_CHANNELS +\
				DMA_DEV_INT_NUM_LOGICAL_CHANNELS +\
				DMA_DEV_ISA_NUM_PHYSICAL_CHANNELS +\
				DMA_DEV_IPFD_NUM_PHYSICAL_CHANNELS)

#define IA_CSS_N_TPROXY_DEV		6

typedef void (*ia_css_tproxy_event_handler_t)(
	const struct ia_css_tproxy_eq_msg *msg);


#if 0
enum ia_css_tproxy_chan_state {
	IA_CSS_TPROXY_CHAN_STATE_IDLE,
	IA_CSS_TPROXY_CHAN_STATE_AVALIABLE,
	IA_CSS_TPROXY_CHAN_STATE_READY,
	IA_CSS_TPROXY_CHAN_STATE_RUNNING,
	IA_CSS_N_TPROXY_CHAN_STATE
};

struct ia_css_tproxy_server_chan_handle {
	uint8_t chan_id;
	enum ia_css_tproxy_chan_state state;
	struct vied_nci_dma_chan_t *hndl;
	struct ia_css_tproxy_transfer_config *transfer_config;
	struct ia_css_tproxy_terminal_config *terminal_config;
};
#endif

/**
 * Open a DMA channel
 */
void ia_css_tproxy_service_chan_open(
	const struct ia_css_tproxy_eq_msg *msg);

/**
 * Configure a DMA channel
 */
void ia_css_tproxy_service_chan_configure(
	const struct ia_css_tproxy_eq_msg *msg);

/**
 * Start a DMA channel
 */
void ia_css_tproxy_service_chan_next(
	const struct ia_css_tproxy_eq_msg *msg);

/**
 * Close a DMA channel
 */
void ia_css_tproxy_service_chan_close(
	const struct ia_css_tproxy_eq_msg *msg);

/**
 * Acknowledge a DMA channel completion
 */
void ia_css_tproxy_service_chan_ack(
	const struct ia_css_tproxy_eq_msg *msg);

STORAGE_CLASS_EXTERN void ia_css_tproxy_server_event_handler(void);
STORAGE_CLASS_EXTERN int ia_css_tproxy_server_init(void);
STORAGE_CLASS_EXTERN void ia_css_tproxy_server_exit(void);

#endif /*_IA_CSS_TPROXY_SERVER_H_*/
