#ifndef _HRT_SLAVE_H
#define _HRT_SLAVE_H

/* This define is necessary to avoid warnings about using NULL in arithmetic
   expressions when compiling with g++ */
#define _hrt_not_null(ptr) ({ void*p = NULL; p != (void*)(ptr); })

#include "master_port.h"
#include "system_api.h"

#define _hrt_direct_store_8_msg(addr, msg, value) \
        _hrt_master_port_store_8_msg(addr, value, msg)
#define _hrt_direct_store_16_msg(addr, msg, value) \
        _hrt_master_port_store_16_msg(addr, value, msg)
#define _hrt_direct_store_32_msg(addr, msg, value) \
        _hrt_master_port_store_32_msg(addr, value, msg)


#define _hrt_direct_load_8_msg(addr, msg) \
        _hrt_master_port_load_8_msg(addr, msg)
#define _hrt_direct_load_16_msg(addr, msg) \
        _hrt_master_port_load_16_msg(addr, msg)
#define _hrt_direct_load_32_msg(addr, msg) \
        _hrt_master_port_load_32_msg(addr, msg)



#define _hrt_slave_port_store_8_msg(slave_port, addr, value, msg) \
        _hrt_master_port_store_8_msg(_hrt_slave_port_master_port_address(slave_port) + (addr), value, _hrt_not_null(msg)?msg:(_hrt_slave_port_error_message(slave_port)))
#define _hrt_slave_port_store_16_msg(slave_port, addr, value, msg) \
        _hrt_master_port_store_16_msg(_hrt_slave_port_master_port_address(slave_port) + (addr), value, _hrt_not_null(msg)?msg:(_hrt_slave_port_error_message(slave_port)))
#define _hrt_slave_port_store_32_msg(slave_port, addr, value, msg) \
        _hrt_master_port_store_32_msg(_hrt_slave_port_master_port_address(slave_port) + (addr), value, _hrt_not_null(msg)?msg:(_hrt_slave_port_error_message(slave_port)))

#define _hrt_slave_port_load_8_msg(slave_port, addr, msg) \
        _hrt_master_port_load_8_msg(_hrt_slave_port_master_port_address(slave_port) + (addr), _hrt_not_null(msg)?msg:(_hrt_slave_port_error_message(slave_port)))
#define _hrt_slave_port_load_16_msg(slave_port, addr, msg) \
        _hrt_master_port_load_16_msg(_hrt_slave_port_master_port_address(slave_port) + (addr), _hrt_not_null(msg)?msg:(_hrt_slave_port_error_message(slave_port)))
#define _hrt_slave_port_load_32_msg(slave_port, addr, msg) \
        _hrt_master_port_load_32_msg(_hrt_slave_port_master_port_address(slave_port) + (addr), _hrt_not_null(msg)?msg:(_hrt_slave_port_error_message(slave_port)))

#define _hrt_slave_port_uload_8_msg(slave_port, addr, msg) \
        _hrt_master_port_uload_8_msg(_hrt_slave_port_master_port_address(slave_port) + (addr), _hrt_not_null(msg)?msg:(_hrt_slave_port_error_message(slave_port)))
#define _hrt_slave_port_uload_16_msg(slave_port, addr, msg) \
        _hrt_master_port_uload_16_msg(_hrt_slave_port_master_port_address(slave_port) + (addr), _hrt_not_null(msg)?msg:(_hrt_slave_port_error_message(slave_port)))
#define _hrt_slave_port_uload_32_msg(slave_port, addr, msg) \
        _hrt_master_port_uload_32_msg(_hrt_slave_port_master_port_address(slave_port) + (addr), _hrt_not_null(msg)?msg:(_hrt_slave_port_error_message(slave_port)))

#define _hrt_slave_port_store_msg(slave_port, addr, data, size, msg) \
        _hrt_master_port_unaligned_store_msg(_hrt_slave_port_master_port_address(slave_port) + (addr), data, size, _hrt_not_null(msg)?msg:(_hrt_slave_port_error_message(slave_port)))
#define _hrt_slave_port_load_msg(slave_port, addr, data, size, msg) \
        _hrt_master_port_unaligned_load_msg(_hrt_slave_port_master_port_address(slave_port) + (addr), data, size, _hrt_not_null(msg)?msg:(_hrt_slave_port_error_message(slave_port)))
#define _hrt_slave_port_set_msg(slave_port, addr, data, size, msg) \
        _hrt_master_port_unaligned_set_msg(_hrt_slave_port_master_port_address(slave_port) + (addr), data, size, _hrt_not_null(msg)?msg:(_hrt_slave_port_error_message(slave_port)))

#define _hrt_slave_port_load_8_volatile_msg(slave_port, addr, msg) \
        _hrt_master_port_load_8_volatile_msg(_hrt_slave_port_master_port_address(slave_port) + (addr), _hrt_not_null(msg)?msg:(_hrt_slave_port_error_message(slave_port)))
#define _hrt_slave_port_uload_8_volatile_msg(slave_port, addr, msg) \
        _hrt_master_port_uload_8_volatile_msg(_hrt_slave_port_master_port_address(slave_port) + (addr), _hrt_not_null(msg)?msg:(_hrt_slave_port_error_message(slave_port)))
#define _hrt_slave_port_store_8_volatile_msg(slave_port, addr, value, msg) \
        _hrt_master_port_store_8_volatile_msg(_hrt_slave_port_master_port_address(slave_port) + (addr), value, _hrt_not_null(msg)?msg:(_hrt_slave_port_error_message(slave_port)))

#define _hrt_slave_port_load_16_volatile_msg(slave_port, addr, msg) \
        _hrt_master_port_load_16_volatile_msg(_hrt_slave_port_master_port_address(slave_port) + (addr), _hrt_not_null(msg)?msg:(_hrt_slave_port_error_message(slave_port)))
#define _hrt_slave_port_uload_16_volatile_msg(slave_port, addr, msg) \
        _hrt_master_port_uload_16_volatile_msg(_hrt_slave_port_master_port_address(slave_port) + (addr), _hrt_not_null(msg)?msg:(_hrt_slave_port_error_message(slave_port)))
#define _hrt_slave_port_store_16_volatile_msg(slave_port, addr, value, msg) \
        _hrt_master_port_store_16_volatile_msg(_hrt_slave_port_master_port_address(slave_port) + (addr), value, _hrt_not_null(msg)?msg:(_hrt_slave_port_error_message(slave_port)))

#define _hrt_slave_port_load_32_volatile_msg(slave_port, addr, msg) \
        _hrt_master_port_load_32_volatile_msg(_hrt_slave_port_master_port_address(slave_port) + (addr), _hrt_not_null(msg)?msg:(_hrt_slave_port_error_message(slave_port)))
#define _hrt_slave_port_uload_32_volatile_msg(slave_port, addr, msg) \
        _hrt_master_port_uload_32_volatile_msg(_hrt_slave_port_master_port_address(slave_port) + (addr), _hrt_not_null(msg)?msg:(_hrt_slave_port_error_message(slave_port)))
#define _hrt_slave_port_store_32_volatile_msg(slave_port, addr, value, msg) \
        _hrt_master_port_store_32_volatile_msg(_hrt_slave_port_master_port_address(slave_port) + (addr), value, _hrt_not_null(msg)?msg:(_hrt_slave_port_error_message(slave_port)))

#define _hrt_slave_port_store_8(p,a,v)           _hrt_slave_port_store_8_msg(p,a,v,NULL)
#define _hrt_slave_port_store_16(p,a,v)          _hrt_slave_port_store_16_msg(p,a,v,NULL)
#define _hrt_slave_port_store_32(p,a,v)          _hrt_slave_port_store_32_msg(p,a,v,NULL)
#define _hrt_slave_port_load_8(p,a)              _hrt_slave_port_load_8_msg(p,a,NULL)
#define _hrt_slave_port_load_16(p,a)             _hrt_slave_port_load_16_msg(p,a,NULL)
#define _hrt_slave_port_load_32(p,a)             _hrt_slave_port_load_32_msg(p,a,NULL)
#define _hrt_slave_port_uload_8(p,a)             _hrt_slave_port_uload_8_msg(p,a,NULL)
#define _hrt_slave_port_uload_16(p,a)            _hrt_slave_port_uload_16_msg(p,a,NULL)
#define _hrt_slave_port_uload_32(p,a)            _hrt_slave_port_uload_32_msg(p,a,NULL)
#define _hrt_slave_port_store(p,a,d,s)           _hrt_slave_port_store_msg(p,a,d,s,NULL)
#define _hrt_slave_port_load(p,a,d,s)            _hrt_slave_port_load_msg(p,a,d,s,NULL)
#define _hrt_slave_port_set(p,a,d,s)             _hrt_slave_port_set_msg(p,a,d,s,NULL)
#define _hrt_slave_port_load_8_volatile(p,a)     _hrt_slave_port_load_8_volatile_msg(p,a,NULL)
#define _hrt_slave_port_uload_8_volatile(p,a)    _hrt_slave_port_uload_8_volatile_msg(p,a,NULL)
#define _hrt_slave_port_store_8_volatile(p,a,v)  _hrt_slave_port_store_8_volatile_msg(p,a,v,NULL)
#define _hrt_slave_port_load_16_volatile(p,a)    _hrt_slave_port_load_16_volatile_msg(p,a,NULL)
#define _hrt_slave_port_uload_16_volatile(p,a)   _hrt_slave_port_uload_16_volatile_msg(p,a,NULL)
#define _hrt_slave_port_store_16_volatile(p,a,v) _hrt_slave_port_store_16_volatile_msg(p,a,v,NULL)
#define _hrt_slave_port_load_32_volatile(p,a)    _hrt_slave_port_load_32_volatile_msg(p,a,NULL)
#define _hrt_slave_port_uload_32_volatile(p,a)   _hrt_slave_port_uload_32_volatile_msg(p,a,NULL)
#define _hrt_slave_port_store_32_volatile(p,a,v) _hrt_slave_port_store_32_volatile_msg(p,a,v,NULL)

#endif /* _HRT_SLAVE_H */
