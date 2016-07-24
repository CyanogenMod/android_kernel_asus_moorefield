#ifndef __RECV_PORT_H__
#define __RECV_PORT_H__


struct recv_port;
struct sys_queue;

void
recv_port_open(struct recv_port* p, struct sys_queue* q);

unsigned int
recv_port_available(struct recv_port* p);

unsigned int
recv_port_transfer(struct recv_port* p, void* data);


#endif /*__RECV_PORT_H__*/
