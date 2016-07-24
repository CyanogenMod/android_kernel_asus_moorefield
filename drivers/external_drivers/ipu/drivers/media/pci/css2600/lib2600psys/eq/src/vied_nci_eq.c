#include "vied_nci_eq_storage_class.h"
#include "vied_nci_eq_device.h"
#include "vied_nci_eq_const.h"
#include "vied_nci_eq_properties.h"
#include "vied_nci_eq_reg_access.h"
#include "assert_support.h"
#include "device_address.h"

#ifndef NOT_USED
#define NOT_USED(a) ((a) = (a))
#endif


#ifndef __INLINE_VIED_NCI_EQ__
#include "vied_nci_eq_send_inline.h"
#include "vied_nci_eq_recv_inline.h"
#endif

/***************************Queue Internal Configure Interface**********************/

/* 0x100
 * Get/Set soft deadline value for queue = queue_nr
 *
 * \param dev      The Queue device address
 * \param queue_nr The soft deadline value for queue_nr
 *
 * return non
 */
unsigned int vied_nci_eq_get_sdp (vied_nci_eq_device_t dev, unsigned int
                                  queue_nr)
{
    unsigned int read_val;

    OP___assert(queue_nr < eq_device_properties.nr_queues);

    read_val =  event_queue_ip_reg_load(dev, EVENT_QUEUE_IP_SDP_BASE + queue_nr * 4);

    return (read_val);
}

void vied_nci_eq_set_sdp (vied_nci_eq_device_t dev, unsigned int
                          queue_nr, unsigned int deadline)
{
    OP___assert(queue_nr < eq_device_properties.nr_queues);

    if (queue_nr == 0) {
        return ; //Unused for priority 0 (highest)
    }

    event_queue_ip_reg_store(dev, EVENT_QUEUE_IP_SDP_BASE + queue_nr*0x4, deadline);
}

/* 0x200
 * Get/Set PID end value for queue = queue_nr */
unsigned int vied_nci_eq_get_pidend (vied_nci_eq_device_t dev, unsigned int queue_nr)
{
    unsigned int read_val;

    OP___assert(queue_nr < eq_device_properties.nr_queues);

    read_val  = event_queue_ip_reg_load(dev, EVENT_QUEUE_IP_PIDMAP_BASE + queue_nr*0x4);

    return (read_val);
}

void vied_nci_eq_set_pidend (vied_nci_eq_device_t dev, unsigned int queue_nr,
                             unsigned int pid)
{
    OP___assert(queue_nr < eq_device_properties.nr_queues);

    event_queue_ip_reg_store(dev, EVENT_QUEUE_IP_PIDMAP_BASE + queue_nr*0x4, pid);
}

/* 0x300
 * Get/Set wakeup priority level for queue = queue_nr*/
unsigned int vied_nci_eq_get_wakeup_prio (vied_nci_eq_device_t dev, unsigned int queue_nr)
{
    unsigned int read_val;

    OP___assert(queue_nr < eq_device_properties.nr_queues);

    read_val  = event_queue_ip_reg_load(dev, EVENT_QUEUE_IP_QCFG_BASE + queue_nr*0x4);

    return (read_val);
}

void vied_nci_eq_set_wake_prio (vied_nci_eq_device_t dev, unsigned int queue_nr,
                                unsigned int wakeup_prio)
{
    OP___assert(queue_nr < eq_device_properties.nr_queues);
    event_queue_ip_reg_store(dev, EVENT_QUEUE_IP_QCFG_BASE + queue_nr*0x4, wakeup_prio);
}

/* 0x800
 * Get/Set timer increment value */
unsigned int vied_nci_eq_get_timer_inc(vied_nci_eq_device_t dev)
{
    return event_queue_ip_reg_load(dev, EVENT_QUEUE_IP_TIMER_INC);
}

void vied_nci_eq_set_timer_inc(vied_nci_eq_device_t dev, unsigned int timer)
{
    event_queue_ip_reg_store(dev, EVENT_QUEUE_IP_TIMER_INC, timer);
}

vied_nci_eq_device_t
vied_nci_eq_device_open(vied_device_id id,
                        const struct vied_nci_eq_device_config_t* dconf,
                        const struct vied_nci_eq_queue_config_t* qconf
                        // const struct vied_nci_eq_priority_config_t* pconf
                       )
{
    vied_nci_eq_device_t dev;
    unsigned int i;

    if (id >= N_EVENT_QUEUE_ID) //KW fix: index out of range
	    return 0;
    // vied open device
    // vied get route -> base address -> dev
    dev = (unsigned int)EVENT_QUEUE_IP_BASE[id];

    // assert(dconf->num_queues > 0);
    // assert(dconf->num_queues < EQ_NUM_QUEUES);

    // configure the pidmap, endpid of last queue is not used
    for (i=0; i<dconf->num_queues; i++)
        vied_nci_eq_set_pidend(dev, i, qconf[i].endpid);

    // assert(dconf->num_priorities > 0);
    // assert(dconf->num_priortoes < EQ_NUM_PRIORITIES);

    // configure deadlines per priority, starting at p=1 (p0 has no deadline)
    // for (i=1; i<dconf->num_priorities; i++)
    // vied_cmem_store_32(dev + EQ_SDP_REG_ADDR + 4*i, pconf[i].deadline);

    return dev;
}

/* Flush all queues, both blocking and non-blocking (WO) */
void vied_nci_eq_device_flush(vied_nci_eq_device_t dev)
{
    event_queue_ip_reg_store(dev, EVENT_QUEUE_IP_TOT_QCFG, 0x1);
}

/****************** Queue Wake Up Interface **********************/
/* Only valid for the output port */
unsigned int vied_nci_eq_get_wakup_stat_low(void)
{
    return event_queue_op_reg_load(EVENT_QUEUE_OP_WAKEUP_STAT_LOW);
}

void vied_nci_eq_enable_wakeup_low(unsigned int wakeup_bit)
{
    event_queue_op_reg_store(EVENT_QUEUE_OP_WAKEUP_ENAB_LOW, wakeup_bit);
}

void vied_nci_eq_set_wakeup_low(unsigned int wakeup_bit)
{
    event_queue_op_reg_store(EVENT_QUEUE_OP_WAKEUP_SET_LOW, wakeup_bit);
}

void vied_nci_eq_clear_wakeup_low(unsigned int wakeup_bit)
{
    event_queue_op_reg_store(EVENT_QUEUE_OP_WAKEUP_CLR_LOW, wakeup_bit);
}

/*************** Tracing Interface *************************/

/* Address of the FW first trace packet */
void vied_nci_eq_set_trace_addr_a(vied_nci_eq_device_t dev, unsigned int addr)
{
    event_queue_ip_reg_store(dev, EVENT_QUEUE_IP_TRACE_ADDR_FIRST, addr);
}

/* Address of the FW middle trace packet */
void vied_nci_eq_set_trace_addr_b(vied_nci_eq_device_t dev, unsigned int addr)
{
    event_queue_ip_reg_store(dev, EVENT_QUEUE_IP_TRACE_ADDR_MIDDLE, addr);
}

/* Address of the FW last trace packet */
void vied_nci_eq_set_trace_addr_c(vied_nci_eq_device_t dev, unsigned int addr)
{
    event_queue_ip_reg_store(dev, EVENT_QUEUE_IP_TRACE_ADDR_LAST, addr);
}

/* Address of all event queue and pc trace packets. */
void vied_nci_eq_set_trace_addr_d(vied_nci_eq_device_t dev, unsigned int addr)
{
    event_queue_ip_reg_store(dev, EVENT_QUEUE_IP_TRACE_ADDR_ALL, addr);
}

/* Individual trace enable.
 * Bit 0 : Enable flush tracing (non-block queue)
 * Bit 1 : Enable entry tracing (non-block queue)
 * Bit 2 : Enable exit tracing (non-block queue)
 * Bit 3 : Enable promotion tracing (non-block queue)
 * Bit 4 : Enable entry tracing (block queue)
 * Bit 5 : Enable exit tracing (block queue)
 * Bit 6 : Enable blocked-exit tracing (block queue)
 * Bit 7 : Enable periodic PC tracing
 * Bit 8 : Enable (I)SP tracing
 */
void vied_nci_eq_enable_trace(vied_nci_eq_device_t dev, unsigned int trace_enable)
{
    event_queue_ip_reg_store(dev, EVENT_QUEUE_IP_TRACE_ENABLE, trace_enable);
}

/* The periodic PC trace timer will count upto timer. It will then reset
 * and send a PC trace packet.
 */
void vied_nci_eq_set_trace_pc_timer(vied_nci_eq_device_t dev, unsigned int timer)
{
    event_queue_ip_reg_store(dev, EVENT_QUEUE_IP_TRACE_PER_PC, timer);
}

/* Bit[3:0] : SVEN header
 */
void vied_nci_eq_set_trace_header(vied_nci_eq_device_t dev, unsigned int header)
{
    event_queue_ip_reg_store(dev, EVENT_QUEUE_IP_TRACE_HEADER, header);
}

/* Mode of tracing :
 * Bit 0 : 0 = lossy tracing
 *         1 = lossless tracing
 */
void vied_nci_eq_set_trace_mode(vied_nci_eq_device_t dev, unsigned int mode)
{
    event_queue_ip_reg_store(dev, EVENT_QUEUE_IP_TRACE_MODE, mode);
}

/* Get the lost packets */
unsigned int vied_nci_eq_get_lost_packets(vied_nci_eq_device_t dev)
{
    return event_queue_ip_reg_load(dev, EVENT_QUEUE_IP_TRACE_LOST_PACKET);
}

/* Clear lost packets */
void vied_nci_eq_clear_lost_packets(vied_nci_eq_device_t dev)
{
    event_queue_ip_reg_store(dev, EVENT_QUEUE_IP_TRACE_LP_CLEAR, 0x1);
}

/* Message of the FIRST FW debug write message */
void vied_nci_eq_set_fwtrace_first(unsigned int msg)
{
    event_queue_op_reg_store(EVENT_QUEUE_OP_FW_TRACE_ADDR_FIRST, msg);
}


/* Message of the any other than FIRST or LAST FW debug write message */
void vied_nci_eq_set_fwtrace_middle(unsigned int msg)
{
    event_queue_op_reg_store(EVENT_QUEUE_OP_FW_TRACE_ADDR_MIDDLE, msg);
}


/* Message of the LAST FW debug write message */
void vied_nci_eq_set_fwtrace_last(unsigned int msg)
{
    event_queue_op_reg_store(EVENT_QUEUE_OP_FW_TRACE_ADDR_FIRST, msg);
}

/* Close
 * Do nothing. Just match FAS
 */
void
vied_nci_eq_device_close(vied_nci_eq_device_t dev)
{
    dev = dev;
}

