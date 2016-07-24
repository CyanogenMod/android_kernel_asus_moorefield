#ifndef _VIED_NCI_EQ_DEVICE_H_
#define _VIED_NCI_EQ_DEVICE_H_

#include "vied_nci_eq_types.h"

#define EQ_INVALID_TOKEN    0xFFFFFFFF
#define SELF_QUEUE          0x0     //The queue port which is attached to the processor

/***********Configure and status Interface ****************/
/*
 * @brief   Struct of the Device static properties got from HAS
 */
typedef struct vied_nci_eq_device_properties_s
{
    //Number of supported priority levels (max 64)
    unsigned int nr_prio;
    //Number of supported non-blocking queues (max 64)
    unsigned int nr_queues;
    //Size of the total event queue (used by nr_prio*nr_queues events)
    unsigned int queue_size;
    //Size of the SID field in the token
    unsigned int sid_size;
    //Size of the PID field in the token
    unsigned int pid_size;
    //Size of the GAP field in the token (unused part of the write data)
    unsigned int gap_size;
    //Size of the MSG field in the token
    unsigned int msg_size;
    //Size of the internal timer
    unsigned int tim_size;
    //Number of supported blocking queues (max 64). In case this value is
    //zero, no blocking queue logic is added.
    unsigned int nr_block_queues;
    //Size (number of bits) of the semaphore counter of the blocking queue
    unsigned int block_cntr_size;
    //Size (number of bits) of the internal trace timer
    unsigned int tr_tim_size;
    //Depth of the trace entry FIFO
    unsigned int trace_entry_depth;
    //Depth of the trace FIFO
    unsigned int trace_depth;
} vied_nci_eq_device_properties_t;


struct vied_nci_eq_device_config_t
{
    unsigned int num_queues;
    unsigned int num_priorities;
};

struct vied_nci_eq_queue_config_t
{
    unsigned int endpid;
};

typedef unsigned int vied_nci_eq_device_t;	// device handle, impl dependent

/*
void
vied_nci_eq_device_get_properties(vied_device_id id
	struct vied_nci_eq_device_properties_t *prop);
*/

vied_nci_eq_device_t
vied_nci_eq_device_open(vied_device_id id,
                        const struct vied_nci_eq_device_config_t* dconf,
                        const struct vied_nci_eq_queue_config_t* qconf
                        // const struct vied_nci_eq_priority_config_t* pconf
                       );

void
vied_nci_eq_device_flush(vied_nci_eq_device_t dev);

void
vied_nci_eq_device_close(vied_nci_eq_device_t dev);

/* TODO : find another place for prototypes ? */
/* Append them here to avoid compiler warnings when -Wmissing-prototypes flag is enabled */

unsigned int vied_nci_eq_get_sdp (vied_nci_eq_device_t dev, unsigned int queue_nr);

void vied_nci_eq_set_sdp (vied_nci_eq_device_t dev,
			unsigned int queue_nr, unsigned int deadline);

unsigned int vied_nci_eq_get_pidend (vied_nci_eq_device_t dev, unsigned int queue_nr);

void vied_nci_eq_set_pidend (vied_nci_eq_device_t dev, unsigned int queue_nr,
                             unsigned int pid);

unsigned int vied_nci_eq_get_wakeup_prio (vied_nci_eq_device_t dev, unsigned int queue_nr);

void vied_nci_eq_set_wake_prio (vied_nci_eq_device_t dev, unsigned int queue_nr,
                                unsigned int wakeup_prio);

unsigned int vied_nci_eq_get_timer_inc(vied_nci_eq_device_t dev);

void vied_nci_eq_set_timer_inc(vied_nci_eq_device_t dev, unsigned int timer);

unsigned int vied_nci_eq_get_wakup_stat_low(void);

void vied_nci_eq_enable_wakeup_low(unsigned int wakeup_bit);

void vied_nci_eq_set_wakeup_low(unsigned int wakeup_bit);

void vied_nci_eq_clear_wakeup_low(unsigned int wakeup_bit);

void vied_nci_eq_set_trace_addr_a(vied_nci_eq_device_t dev, unsigned int addr);

void vied_nci_eq_set_trace_addr_b(vied_nci_eq_device_t dev, unsigned int addr);

void vied_nci_eq_set_trace_addr_c(vied_nci_eq_device_t dev, unsigned int addr);

void vied_nci_eq_set_trace_addr_d(vied_nci_eq_device_t dev, unsigned int addr);

void vied_nci_eq_enable_trace(vied_nci_eq_device_t dev, unsigned int trace_enable);

void vied_nci_eq_set_trace_pc_timer(vied_nci_eq_device_t dev, unsigned int timer);

void vied_nci_eq_set_trace_header(vied_nci_eq_device_t dev, unsigned int header);

void vied_nci_eq_set_trace_mode(vied_nci_eq_device_t dev, unsigned int mode);

unsigned int vied_nci_eq_get_lost_packets(vied_nci_eq_device_t dev);

void vied_nci_eq_clear_lost_packets(vied_nci_eq_device_t dev);

void vied_nci_eq_set_fwtrace_first(unsigned int msg);

void vied_nci_eq_set_fwtrace_middle(unsigned int msg);

void vied_nci_eq_set_fwtrace_last(unsigned int msg);

#endif

