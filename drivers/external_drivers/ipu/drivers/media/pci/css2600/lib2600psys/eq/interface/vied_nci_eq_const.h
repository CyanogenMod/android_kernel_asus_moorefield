#ifndef _VIED_NCI_EQ_CONST_H_
#define _VIED_NCI_EQ_CONST_H_

#include "vied_nci_eq_device.h"

#define EQ_SID_MAX  ((1 << eq_device_properties.sid_size) -1)
#define EQ_MSG_MAX  ((1 << eq_device_properties.msg_size) -1)
#define EQ_MSG_BITS (eq_device_properties.msg_size)
#define EQ_PID_MAX ((1 << eq_device_properties.pid_size) -1)
#define EQ_PID_BITS (eq_device_properties.pid_size)

/* Input port */
#define EVENT_QUEUE_IP_QUEUE_BASE  0x000
#define EVENT_QUEUE_IP_SDP_BASE    0x100
#define EVENT_QUEUE_IP_PIDMAP_BASE 0x200
#define EVENT_QUEUE_IP_QCFG_BASE   0x300
#define EVENT_QUEUE_IP_QSTAT_BASE  0x400 //status of each queue
#define EVENT_QUEUE_IP_TOT_QCFG    0x500 //flush
#define EVENT_QUEUE_IP_TOT_QSTAT   0x600
#define EVENT_QUEUE_IP_QRESERVE    0x700
#define EVENT_QUEUE_IP_TIMER_INC   0x800


/* Output port Basic */
#define EVENT_QUEUE_0P_QUEUE_BASE  0x000
#define EVENT_QUEUE_OP_SDP_BASE    0x100
#define EVENT_QUEUE_OP_PIDMAP_BASE 0x200
#define EVENT_QUEUE_OP_QCFG_BASE   0x300
#define EVENT_QUEUE_OP_QSTAT_BASE  0x400
#define EVENT_QUEUE_OP_TOT_QCFG    0x500 //flush
#define EVENT_QUEUE_OP_TOT_QSTAT   0x600
#define EVENT_QUEUE_OP_TIMER_INC      0x800

#define EVENT_QUEUE_OP_WAKEUP_STAT_LOW  0x700
#define EVENT_QUEUE_OP_WAKEUP_ENAB_LOW  0x704
#define EVENT_QUEUE_OP_WAKEUP_SET_LOW   0x708
#define EVENT_QUEUE_OP_WAKEUP_CLR_LOW   0x70c
#define EVENT_QUEUE_OP_BQ_BASE          0xF00

/* Input port Tracing */
#define EVENT_QUEUE_IP_TRACE_ADDR_FIRST     0x900
#define EVENT_QUEUE_IP_TRACE_ADDR_MIDDLE    0x904
#define EVENT_QUEUE_IP_TRACE_ADDR_LAST      0x908
#define EVENT_QUEUE_IP_TRACE_ADDR_ALL       0x90c
#define EVENT_QUEUE_IP_TRACE_ENABLE         0x910
#define EVENT_QUEUE_IP_TRACE_PER_PC         0x914
#define EVENT_QUEUE_IP_TRACE_HEADER         0x918
#define EVENT_QUEUE_IP_TRACE_MODE           0x91c
#define EVENT_QUEUE_IP_TRACE_LOST_PACKET    0x920
#define EVENT_QUEUE_IP_TRACE_LP_CLEAR       0x924

/* Output port Tracing */
#define EVENT_QUEUE_OP_TRACE_ADDR_FIRST     0x900
#define EVENT_QUEUE_OP_TRACE_ADDR_MIDDLE    0x904
#define EVENT_QUEUE_OP_TRACE_ADDR_LAST      0x908
#define EVENT_QUEUE_OP_TRACE_ADDR_ALL       0x90c
#define EVENT_QUEUE_OP_TRACE_ENABLE         0x910
#define EVENT_QUEUE_OP_TRACE_PER_PC         0x914
#define EVENT_QUEUE_OP_TRACE_HEADER         0x918
#define EVENT_QUEUE_OP_TRACE_MODE           0x91c
#define EVENT_QUEUE_OP_TRACE_LOST_PACKET    0x920
#define EVENT_QUEUE_OP_TRACE_LP_CLEAR       0x924
#define EVENT_QUEUE_OP_FW_TRACE_ADDR_FIRST     0xa00
#define EVENT_QUEUE_OP_FW_TRACE_ADDR_MIDDLE    0xa04
#define EVENT_QUEUE_OP_FW_TRACE_ADDR_LAST      0xa08

#endif /* _VIED_NCI_EQ_CONST_H_ */
