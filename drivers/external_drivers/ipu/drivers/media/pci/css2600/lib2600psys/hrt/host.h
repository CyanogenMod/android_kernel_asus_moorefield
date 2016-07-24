/* Host support functions. 
   hrt_sleep is included in the hrt because different implementations 
   are needed for different simulator backends.
 */

#ifndef _HRT_HOST_H
#define _HRT_HOST_H

#include "version.h"
#include "hive_types.h"
#include "thread.h"

#if defined(CRUN) || defined(HRT_CSIM)
#include <hrt/csim/csim_interface.h>
#endif

/*
typedef enum {
  hrt_interrupt_pulse,
  hrt_interrupt_level
} hrt_interrupt_type;
*/

#if defined(HRT_KERNEL)

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/time.h>

#define hrt_assert(x) \
  do { if (!(x)) printk("assert failed!\n"); } while (0)

static inline void hrt_usleep(int us)
{
  set_current_state(TASK_INTERRUPTIBLE);
  /* usec_to_jiffies not yet in our kernel */
  if (msecs_to_jiffies(us/1000)) schedule_timeout(msecs_to_jiffies(us));
  else schedule_timeout(1);
}
static inline void hrt_msleep(int ms)
{
  set_current_state(TASK_INTERRUPTIBLE);
  schedule_timeout(msecs_to_jiffies(ms));
}
static inline void hrt_ssleep(int s)
{
  hrt_msleep(1000 * s);
}
static inline void hrt_sleep(void)
{
  hrt_usleep(1);
}

#else /* HRT_KERNEL */

#define hrt_assert(x)          assert(x)
#if defined (HRT_HW)
static inline void hrt_sleep(void)
{
#ifdef __HIVECC
  OP___schedule();
#endif
}
 #else /* HRT_HW */
extern void hrt_sleep(void);              /* host application does a sleep */
#endif /* HRT_HW */
#endif /* HRT_KERNEL */
extern void hrt_system_init(void); /* initialize system, set master interface base addresses for cells */

/* Register an interrupt handler that will be passed the user_data variable as it's argument
   when an interrupt is received by the host.
   The host processor model only has one incoming interrupt port, so there is no need
   to specify which interrupt gets handled. The interrupt handler will have to find out
   which interrupt was raised on its own. */
extern void hrt_register_interrupt_handler(hrt_interrupt_handler handler);
extern void hrt_unregister_interrupt_handler(hrt_interrupt_handler handler);

/* Set the shape of the interrupt (level or pulse), default is pulse. */
/* RJZ: 2010-09-09: Host does not distinguish anymore between level and pulse interrupts */
/* extern void hrt_set_interrupt_type(hrt_interrupt_type type); */

#ifdef HRT_CSIM
#define hrt_host_load_program(h, p) \
  do { \
    _hrt_current_device_set_source_info(__FILE__,__LINE__); \
    _hrt_csim_host_load_program(h,p); \
  } while(0)

#define hrt_host_start_program(h) \
  do { \
    _hrt_current_device_set_source_info(__FILE__,__LINE__); \
    _hrt_csim_host_start_program(h); \
    hrt_sleep(); \
  } while(0)

#endif // HRT_CSIM

#endif /* _HRT_HOST_H */
