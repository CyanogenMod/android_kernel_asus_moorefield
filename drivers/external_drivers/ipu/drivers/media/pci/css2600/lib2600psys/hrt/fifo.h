#ifndef _HRT_FIFO_H
#define _HRT_FIFO_H

#include "hive_types.h"
#include "master_port.h"
#include "bits.h"

#define _hrt_snd_address(port)			HRTCAT(port, _snd_address)
#define _hrt_snd_poll_address(port)		HRTCAT(port, _snd_poll_address)
#define _hrt_snd_poll_bit(port)			HRTCAT(port, _snd_poll_bit)

#define _hrt_rcv_address(port)			HRTCAT(port, _rcv_address)
#define _hrt_rcv_poll_address(port)		HRTCAT(port, _rcv_poll_address)
#define _hrt_rcv_poll_bit(port)			HRTCAT(port, _rcv_poll_bit)


#define hrt_fifo_snd(port, val) \
  _hrt_master_port_store_32_volatile(_hrt_snd_address(port), val)

#define hrt_fifo_snd_poll(port) \
  (!_hrt_get_bit(_hrt_master_port_load_32_volatile(_hrt_snd_poll_address(port)), \
                 _hrt_snd_poll_bit(port)))

#define hrt_fifo_rcv(port) \
  _hrt_master_port_load_32_volatile(_hrt_rcv_address(port))

#define hrt_fifo_rcv_poll(port) \
  (!_hrt_get_bit(_hrt_master_port_load_32_volatile(_hrt_rcv_poll_address(port)), \
                 _hrt_rcv_poll_bit(port)))

#endif /* _HRT_FIFO_H */
