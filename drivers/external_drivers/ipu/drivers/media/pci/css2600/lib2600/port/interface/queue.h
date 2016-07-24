#ifndef __SYS_QUEUE_H__
#define __SYS_QUEUE_H__

#include "queue_struct.h"
/*
 * SYS queues are created by the host
 * SYS queues cannot be accessed through the queue interface
 * To send data into a queue a send_port must be opened.
 * To receive data from a queue, a recv_port must be opened.
 */

struct sys_queue;

/*
 * initialize a queue that can hold at least 'size' tokens of 'token_size' bytes.
 */
void
sys_queue_init(struct sys_queue* q, unsigned int size, unsigned int token_size);


void
sys_queue_uninit(struct sys_queue* q);


/*
struct sys_queue* queue_alloc();

void          sys_queue_free(struct queue* q);

struct sys_queue* sys_queue_new(unsigned int size, unsigned int token_size);

void sys_queue_delete(struct queue* q);
*/


#endif /*__QUEUE_H__*/
