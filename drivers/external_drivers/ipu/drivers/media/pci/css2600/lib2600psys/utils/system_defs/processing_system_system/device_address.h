#ifndef _DEVICE_ADDRESS_
#define _DEVICE_ADDRESS_

#include <eq_device_id.h>
static const unsigned int EVENT_QUEUE_IP_BASE[N_EVENT_QUEUE_ID] = {
    0x00001000,  /*event queue ip SP Control*/
    0x00021000,  /*event queue ip SP proxy 0*/
    0x00031000,  /*event queue ip SP proxy 1*/
    0x00041000,  /*event queue ip SP fp     */
    0x001C1000,  /*event queue ip ISP TILE 0*/
    0x00241000,  /*event queue ip ISP TILE 1*/
    0x002C1000,  /*event queue ip ISP TILE 2*/
    0x00341000 /*event queue ip ISP TILE 3*/
};

#endif /* _DEVICE_ADDRESS_ */
