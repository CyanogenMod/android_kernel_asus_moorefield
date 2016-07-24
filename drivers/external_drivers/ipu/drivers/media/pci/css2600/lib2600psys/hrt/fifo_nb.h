#ifndef _HRT_FIFO_NB_H
#define _HRT_FIFO_B_H

#include "hive_types.h"
#include "master_port.h"
#include "bits.h"

#include "fifo.h"


#define _hrt_snd_nb_address(port)        HRTCAT(port, _snd_nb_address)
#define _hrt_rcv_nb_address(port)        HRTCAT(port, _rcv_nb_address)
#define _hrt_pre_fetch_address(port)     HRTCAT(port, _pre_fetch_address)
#define _hrt_irq_status_address(device)  HRTCAT(device, _irq_status_address)

#define hrt_fifo_snd_nb(port, val) \
  _hrt_master_port_store_32_volatile(_hrt_snd_nb_address(port), val)

#define hrt_fifo_rcv_nb(port) \
  _hrt_master_port_load_32_volatile(_hrt_rcv_nb_address(port))
  
#define hrt_fifo_pre_fetch(port) \
  _hrt_master_port_load_32_volatile(_hrt_pre_fetch_address(port))

#define hrt_fifo_adapter_nb_irq_status(device) \
  _hrt_master_port_load_32_volatile(_hrt_irq_status_address(device))
  
#endif /* _HRT_FIFO_NB_H */
