#ifndef _PSYS_SYSTEM_DEFS_H
#define _PSYS_SYSTEM_DEFS_H

/*
 * This file contains all the device address for the Psys
 */

/* EventQ ID for Psys */
typedef enum {
    EVENT_QUEUE_SPC_ID = 0,
    EVENT_QUEUE_SPP0_ID,
    EVENT_QUEUE_SPP1_ID,
    EVENT_QUEUE_SPFP_ID,
    EVENT_QUEUE_ISP0_ID,
    EVENT_QUEUE_ISP1_ID,
    EVENT_QUEUE_ISP2_ID,
    EVENT_QUEUE_ISP3_ID,
    N_EVENT_QUEUE_ID
} event_queue_ID_t;

#endif
