#ifndef _HRT_THREAD_H_
#define _HRT_THREAD_H_

/*
 * Host thread support functions.
 * This includes thread functions, interrupt and semaphore support.
 * The HRT backend is required to implement these functions.
 *
 * Implementations in host_processor.c
 */

typedef void (*hrt_interrupt_handler) (unsigned int irq);
/*
typedef enum {
	hrt_interrupt_pulse,
	hrt_interrupt_level
} hrt_interrupt_type;
*/
typedef void *hrt_semaphore;
typedef void *hrt_thread;

/* Register an interrupt handler that will be passed the user_data variable
   as it's argument when an interrupt is received by the host.
   The host processor model only has one incoming interrupt port, so there
   is no need to specify which interrupt gets handled. The interrupt handler
   will have to find out which interrupt was raised on its own. */
extern void hrt_register_interrupt_handler(hrt_interrupt_handler handler);
extern void hrt_unregister_interrupt_handler(hrt_interrupt_handler handler);
/* Set the shape of the interrupt (level or pulse), default is pulse. */
/* extern void hrt_set_interrupt_type(hrt_interrupt_type type); */

extern hrt_thread hrt_thread_create(
    char *name,
	void (*thread_main) (void *arg), void *arg);
                    
extern hrt_thread hrt_host_thread_create(
    char *name,
    void(*thread_main)(void *arg));

extern void hrt_thread_destroy(
    hrt_thread me);

extern hrt_semaphore hrt_semaphore_create(unsigned int count);
extern void hrt_semaphore_destroy(hrt_semaphore me);
extern void hrt_semaphore_acquire(hrt_semaphore me);
extern void hrt_semaphore_release(hrt_semaphore me);

#endif /* _HRT_THREAD_H_ */
