#ifndef _HRT_BACKEND_HW_VIRS_IMPL_H 
#define _HRT_BACKEND_HW_VIRS_IMPL_H


/* If HRT_USE_VIR_ADDRS is defined, the OS needs to implement the following functions
   for us:
   _hrt_master_port_store_8(addr,data)
   _hrt_master_port_store_16(addr,data)
   _hrt_master_port_store_32(addr,data)
   _hrt_master_port_load_8(addr)
   _hrt_master_port_load_16(addr)
   _hrt_master_port_load_32(addr)
   _hrt_mem_store(addr,data,size)
   _hrt_mem_load(addr,data,size)
   _hrt_mem_set(addr,val,size)
*/
// void _hrt_master_port_store_8(hrt_address address, uint8_t data);
// void _hrt_master_port_store_16(hrt_address address, uint16_t data);
// void _hrt_master_port_store_32(hrt_address address, uint32_t data);
// uint8_t _hrt_master_port_load_8(hrt_address address);
// uint16_t _hrt_master_port_load_16(hrt_address address);
// uint32_t _hrt_master_port_load_32(hrt_address address);
// void _hrt_mem_store(hrt_address address, const void *data, size_t bytes);
// void _hrt_mem_load(hrt_address address, void *data, size_t bytes);
// void _hrt_mem_set(hrt_address address, int data, size_t bytes);


static inline void hrt_master_port_store_8(hrt_address address, uint8_t data)
{
  _hrt_master_port_store_8(address, data);
}
static inline void hrt_master_port_store_16(hrt_address address, uint16_t data)
{
  _hrt_master_port_store_16(address, data);
}
static inline void hrt_master_port_store_32(hrt_address address, uint32_t data)
{
  _hrt_master_port_store_32(address, data);
}
static inline uint8_t hrt_master_port_load_8(hrt_address address)
{
  return _hrt_master_port_load_8(address);
}
static inline uint16_t hrt_master_port_load_16(hrt_address address)
{
  return _hrt_master_port_load_16(address);
}
static inline uint32_t hrt_master_port_load_32(hrt_address address)
{
  return _hrt_master_port_load_32(address);
}

/* We need to support an address translation at the end of hrt on an
   ia32 host. Pointers on that host are 32-bits. The input address is
   more than 32-bits. Therefore we should not cast to pointers,
   otherwise address bits get lost. */
static inline void hrt_master_port_store(hrt_address address, const void *data, size_t bytes)
{
  _hrt_mem_store(address, (const char*)data, bytes);
}
static inline void hrt_master_port_load(hrt_address address, void *data, size_t bytes)
{
  _hrt_mem_load(address, (char*)data, bytes);
}
static inline void hrt_master_port_set(hrt_address address, int data, size_t bytes)
{
  _hrt_mem_set(address, data, bytes);
}
#endif /* _HRT_BACKEND_HW_VIRS_IMPL_H */
