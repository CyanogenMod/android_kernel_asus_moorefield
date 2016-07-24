#ifndef _VIED_NCI_EQ_H_
#define _VIED_NCI_EQ_H_

/*************Basic Types *********************/
typedef unsigned int vied_nci_eq_token_t;
typedef unsigned int vied_nci_eq_sid_t;
typedef unsigned int vied_nci_eq_pid_t;
typedef unsigned int vied_nci_eq_msg_t;

typedef unsigned int vied_nci_eq_device_t;	// device handle, impl dependent

#define EQ_INVALID_TOKEN    0xFFFFFFFF
#define SELF_QUEUE          0x0     //Configure Queue from the OP interface. The queue attached to the CELL

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

/* Get the device properties, it is platform depdendent */
const vied_nci_eq_device_properties_t *vied_nci_eq_get_properties(void);

/* 0x100
 * Get/Set soft deadline value for queue = queue_nr */
unsigned int vied_nci_eq_get_sdp (vied_nci_eq_device_t dev, unsigned int queue_nr);
void vied_nci_eq_set_sdp (vied_nci_eq_device_t dev, unsigned int queue_nr,
                          unsigned int sdp);

/* 0x200
 * Get/Set PID end value for queue = queue_nr */
unsigned int vied_nci_eq_get_pid (vied_nci_eq_device_t dev, unsigned int queue_nr);
void vied_nci_eq_set_pid (vied_nci_eq_device_t dev, unsigned int queue_nr,
                          unsigned int pid);

/* 0x300
 * Get/Set wakeup priority level for queue = queue_nr*/
unsigned int vied_nci_eq_get_wakeup_prio (vied_nci_eq_device_t dev, unsigned int queue_nr);
void vied_nci_eq_set_wake_prio (vied_nci_eq_device_t dev, unsigned int queue_nr,
                                unsigned int wakeup_prio);

/* 0x400
 * Get number of tokens for queue = queue_nr */
unsigned int vied_nci_eq_get_stat (vied_nci_eq_device_t dev, unsigned int queue_nr);

/* 0x600
 * Get number of tokens for all queues */
unsigned int vied_nci_eq_get_tot_stat(vied_nci_eq_device_t dev);

/* 0x800
 * Get/Set timer increment value */
unsigned int vied_nci_eq_get_timer_inc(vied_nci_eq_device_t dev);
void vied_nci_eq_set_timer_inc(vied_nci_eq_device_t dev, unsigned int timer);

/******************* Queue Send/Receive Interface ***************/

/* Send token to priority = queue_nr queue
 * Valid from the Queue input port only
 * */
int
vied_nci_eq_send(vied_nci_eq_device_t dev, unsigned int queue_nr, vied_nci_eq_token_t token);

/* Receive token from priority = queue_nr queue
 * Valid from the Queue output port only
 * */
vied_nci_eq_token_t vied_nci_eq_recv(unsigned int queue_nr);

/* Flush all queues, both blocking and non-blocking (WO) */
void vied_nci_eq_flush(vied_nci_eq_device_t dev);


/* Token help function */
vied_nci_eq_msg_t  vied_nci_eq_token_get_msg(vied_nci_eq_token_t token);
vied_nci_eq_pid_t  vied_nci_eq_token_get_pid(vied_nci_eq_token_t token);
vied_nci_eq_sid_t  vied_nci_eq_token_get_sid(vied_nci_eq_token_t token);
vied_nci_eq_token_t
vied_nci_eq_token_pack(vied_nci_eq_sid_t sid, vied_nci_eq_pid_t pid, vied_nci_eq_msg_t msg);

/****************** Queue Wakeup Interface ******************/

/* Only valid for the output port */
unsigned int vied_nci_eq_get_wakup_stat_low(void);

void vied_nci_eq_enable_wakeup_low(unsigned int wakeup_bit);

void vied_nci_eq_set_wakeup_low(unsigned int wakeup_bit);

void vied_nci_eq_clear_wakeup_low(unsigned int wakup_bit);

/*************** Tracing Interface *************************/

/* Address of the FW first trace packet */
void vied_nci_eq_set_trace_addr_a(vied_nci_eq_device_t dev, unsigned int addr);

/* Address of the FW middle trace packet */
void vied_nci_eq_set_trace_addr_b(vied_nci_eq_device_t dev, unsigned int addr);

/* Address of the FW last trace packet */
void vied_nci_eq_set_trace_addr_c(vied_nci_eq_device_t dev, unsigned int addr);

/* Address of all event queue and pc trace packets. */
void vied_nci_eq_set_trace_addr_d(vied_nci_eq_device_t dev, unsigned int addr);

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
void vied_nci_eq_enable_trace(vied_nci_eq_device_t dev, unsigned int trace_enable);

/* The periodic PC trace timer will count upto timer. It will then reset
 * and send a PC trace packet.
 */
void vied_nci_eq_set_trace_pc_timer(vied_nci_eq_device_t dev, unsigned int timer);

/* Bit[3:0] : SVEN header
 */
void vied_nci_eq_set_trace_header(vied_nci_eq_device_t dev, unsigned int header);

/* Mode of tracing :
 * Bit 0 : 0 = lossy tracing
 *         1 = lossless tracing
 */
void vied_nci_eq_set_trace_mode(vied_nci_eq_device_t dev, unsigned int mode);

/* Get the lost packet */
unsigned int vied_nci_eq_get_lost_packets(vied_nci_eq_device_t dev);

/* Clear lost packets */
void vied_nci_eq_clear_lost_packets(vied_nci_eq_device_t dev);

/* Address of the FIRST FW debug write message */
void vied_nci_eq_set_fwtrace_first(unsigned int msg);

/* Address of the any other than FIRST or LAST FW debug write message */
void vied_nci_eq_set_fwtrace_middle(unsigned int msg);

/* Address of the LAST FW debug write message */
void vied_nci_eq_set_fwtrace_last(unsigned int msg);

#endif //_VIED_NCI_EQ_H_
