//
// Input Feeder Interface
//

#ifndef _VIED_NCI_INPUT_FEEDER_H_
#define _VIED_NCI_INPUT_FEEDER_H_

#include <type_support.h>
#include "vied_nci_input_feeder_defines.h"


#if defined(CFG_VIED_DEVICE_ACCESS_INLINE_IMPL)
#define _VIED_DEVICE_ACCESS_INLINE static inline
#include "vied_device_access_impl.h"
#else
#define _VIED_DEVICE_ACCESS_INLINE
#endif

// The following parameter should eventually be obtained from HSD properties
#define VIED_NCI_INFEEDER_MAX_SIDS  5

// VIED Stream IDs
typedef enum {
  vied_nci_bayer0_id = 0,
  vied_nci_bayer1_id = 1,
  vied_nci_yuv420_id = 2,
  vied_nci_rgb0_id   = 3,
  vied_nci_rgb1_id   = 4,
  vied_nci_n_id      = 5
} vied_nci_infeeder_stream_id_t;

typedef vied_nci_infeeder_stream_id_t vied_nci_infeeder_stream_handle_t;

// Message IDs  ==> To be determined. Error handling is not specified by FW and Tools teams yet
typedef enum {
  vied_nci_infeeder_msg_successful                    = 0,
  vied_nci_infeeder_msg_config_locked                 = 2,
  vied_nci_infeeder_msg_fifo_size_exceeded            = 3,
  vied_nci_infeeder_msg_num_units_per_fetch_exceeded  = 4,
  vied_nci_infeeder_msg_units_per_buffer_exceeded     = 5,
  vied_nci_infeeder_msg_span_width_exceeded           = 6,
  vied_nci_infeeder_msg_span_height_exceeded          = 7,
  vied_nci_infeeder_msg_region_stride_exceeded        = 8,
  vied_nci_infeeder_msg_init_done                     = 9,
  vied_nci_infeeder_msg_ack_received                  = 10,
  vied_nci_infeeder_msg_event_failure                 = 11
} vied_nci_infeeder_msg_t;

typedef int vied_nci_infeeder_dev_id_t;
typedef int vied_nci_infeeder_handle_t;
typedef uint32_t vied_nci_infeeder_event_t;

// An address in the Processing System addressed by the Input Feeder
typedef int32_t vied_nci_infeeder_address_t;

_VIED_DEVICE_ACCESS_INLINE
vied_nci_infeeder_handle_t vied_nci_infeeder_open(vied_nci_infeeder_dev_id_t handle);

_VIED_DEVICE_ACCESS_INLINE
vied_nci_infeeder_stream_handle_t vied_nci_infeeder_stream_open(vied_nci_infeeder_dev_id_t handle, vied_nci_infeeder_stream_id_t sid);

/**
 * @fifo_size            needs to be balanced over streams, minimum is 2 x 'color component vectors'
 * @num_units_per_fetch  units per dma transfer, min = 1 (new version needs always 1... no higher allowed in c0?), num color component vectors
 * @units_per_buffer     units per run to transfer, transfers unit * 'color component vectors'
 * @span_width           dma span width, how many units on a line in memory (related to region stride)
 * @span_height          dma span height, how many units on a line in memory (related to region stride), determines when to wrap
 * @region_stride        stride of line in bytes (32 pix * 2 bytes per) * span widht + any needed stride in bytes  (to be checked)
 */
_VIED_DEVICE_ACCESS_INLINE
vied_nci_infeeder_msg_t vied_nci_infeeder_stream_config(vied_nci_infeeder_stream_handle_t str_handle, unsigned int fifo_size,
                                                                unsigned int num_units_per_fetch, unsigned int units_per_buffer, unsigned int span_width, unsigned int span_height, unsigned int region_stride);
_VIED_DEVICE_ACCESS_INLINE
    vied_nci_infeeder_msg_t vied_nci_infeeder_stream_start(vied_nci_infeeder_stream_handle_t str_handle);
_VIED_DEVICE_ACCESS_INLINE

vied_nci_infeeder_msg_t
vied_nci_infeeder_stream_set_ack_address(vied_nci_infeeder_stream_handle_t str_handle,
					 vied_nci_infeeder_address_t ack_address, unsigned int sidpid);
vied_nci_infeeder_msg_t vied_nci_infeeder_stream_run(vied_nci_infeeder_stream_handle_t str_handle, vied_nci_infeeder_address_t start_address, vied_nci_infeeder_address_t ack_address);

_VIED_DEVICE_ACCESS_INLINE
vied_nci_infeeder_msg_t vied_nci_infeeder_stream_stop(vied_nci_infeeder_stream_handle_t str_handle);

_VIED_DEVICE_ACCESS_INLINE
vied_nci_infeeder_msg_t vied_nci_infeeder_stream_flush(vied_nci_infeeder_stream_handle_t str_handle);

_VIED_DEVICE_ACCESS_INLINE
vied_nci_infeeder_msg_t vied_nci_infeeder_stream_close(vied_nci_infeeder_stream_handle_t str_handle);

_VIED_DEVICE_ACCESS_INLINE
vied_nci_infeeder_msg_t vied_nci_infeeder_close(vied_nci_infeeder_stream_handle_t str_handle);

_VIED_DEVICE_ACCESS_INLINE
vied_nci_infeeder_msg_t vied_nci_infeeder_decode_event(vied_nci_infeeder_event_t event, vied_nci_infeeder_stream_handle_t *str_handle, vied_nci_infeeder_msg_t *decoded_event);


#endif /* _VIED_NCI_INPUT_FEEDER_H_ */

