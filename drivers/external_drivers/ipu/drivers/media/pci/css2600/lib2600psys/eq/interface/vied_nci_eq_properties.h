#ifndef _VIED_NCI_EQ_PROPERTIES_H_
#define _VIED_NCI_EQ_PROPERTIES_H_

#include "vied_nci_eq_device.h"

/* Broxon Configuration
 * This should be gotten from some platform specific configure file */

static const vied_nci_eq_device_properties_t bxt_eq_device_properties = {
    4, //nr_prio
    8, //nr_queues
    32, //queue_size, 32 or 64, SPC = 32, SPP/SPF = 64 FIXME
    6, //sid_size
    6, //pid_size
    0, //gap_size
    20, //msg_size
    16, //tim_size
    0, //nr_block_queues 0 or 8 FIXME
    8, //block_cntr_size
    16, //tr_tim_size
    4, //trace_entry_depth
    16, //trace_depth
};

#define eq_device_properties bxt_eq_device_properties

#endif /* _VIED_NCI_EQ_PROPERTIES_H_ */
