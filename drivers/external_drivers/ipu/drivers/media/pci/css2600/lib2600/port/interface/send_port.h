#ifndef __SEND_PORT_H__
#define __SEND_PORT_H__


/*
 * A send port can be used to send tokens into a queue.
 * The interface can be used on any type of processor (host, SP, ...)
 */

struct send_port;
struct sys_queue;

/*
 * Open a send port on a queue. After the port is opened, tokens can be sent
 */
void
send_port_open(struct send_port* p, struct sys_queue* q);

/*
 * Close a send port
 */
void
send_port_close(struct send_port* p);

/*
 * Determine how many tokens can be sent
 */
unsigned int
send_port_available(struct send_port* p);

/*
 * Send a token via a send port. The function returns the number of tokens that have been sent:
 * 1: the token was accepted
 * 0: the token was not accepted (full queue)
 * The size of a token is determined at initialization.
 */
unsigned int
send_port_transfer(struct send_port* p, const void* data);


#endif /*__SEND_PORT_H__*/
