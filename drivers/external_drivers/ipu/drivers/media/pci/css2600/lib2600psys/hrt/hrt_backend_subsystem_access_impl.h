#ifndef _HRT_BACKEND_DEVICE_ACCESS_IMPL_H 
#define _HRT_BACKEND_DEVICE_ACCESS_IMPL_H

#include "hive_types.h"
#include "vied/vied_subsystem_access.h"

/* If HRT_USE_VIR_ADDRS, HRT_HW and HRT_ON_VIED_SUBSYSTEM_ACCESS
   are defined, the OS needs to implement the following functions for us:
   
   void vied_subsystem_store_8 (vied_subsystem_t dev, vied_subsystem_address_t addr, uint8_t  data);
   void vied_subsystem_store_16(vied_subsystem_t dev, vied_subsystem_address_t addr, uint16_t data);
   void vied_subsystem_store_32(vied_subsystem_t dev, vied_subsystem_address_t addr, uint32_t data);
   void vied_subsystem_store(vied_subsystem_t dev, vied_subsystem_address_t addr, const void *data, unsigned int size);

   uint8_t  vied_subsystem_load_8 (vied_subsystem_t dev, vied_subsystem_address_t addr);
   uint16_t vied_subsystem_load_16(vied_subsystem_t dev, vied_subsystem_address_t addr);
   uint32_t vied_subsystem_load_32(vied_subsystem_t dev, vied_subsystem_address_t addr);
   void vied_subsystem_load(vied_subsystem_t dev, vied_subsystem_address_t addr, void *data, unsigned int size);
*/

static inline void hrt_master_port_store_8(hrt_address address, uint8_t data)
{
  vied_subsystem_store_8 (HRT_ON_VIED_SUBSYSTEM_ACCESS, address - _hrt_master_port_subsystem_base_address,
                    data);
}
static inline void hrt_master_port_store_16(hrt_address address, uint16_t data)
{
  vied_subsystem_store_16 (HRT_ON_VIED_SUBSYSTEM_ACCESS, address - _hrt_master_port_subsystem_base_address, data);
}
static inline void hrt_master_port_store_32(hrt_address address, uint32_t data)
{
  vied_subsystem_store_32 (HRT_ON_VIED_SUBSYSTEM_ACCESS, address - _hrt_master_port_subsystem_base_address, data);
}
static inline uint8_t hrt_master_port_load_8(hrt_address address)
{
  return  vied_subsystem_load_8 (HRT_ON_VIED_SUBSYSTEM_ACCESS, address - _hrt_master_port_subsystem_base_address);
}
static inline uint16_t hrt_master_port_load_16(hrt_address address)
{
  return  vied_subsystem_load_16 (HRT_ON_VIED_SUBSYSTEM_ACCESS, address - _hrt_master_port_subsystem_base_address);
}
static inline uint32_t hrt_master_port_load_32(hrt_address address)
{
  return  vied_subsystem_load_32 (HRT_ON_VIED_SUBSYSTEM_ACCESS, address - _hrt_master_port_subsystem_base_address);
}

static inline void hrt_master_port_store(hrt_address address, const void *data, size_t bytes)
{
   vied_subsystem_store(HRT_ON_VIED_SUBSYSTEM_ACCESS, address - _hrt_master_port_subsystem_base_address, data, bytes);
}
static inline void hrt_master_port_load(hrt_address address, void *data, size_t bytes)
{
   vied_subsystem_load(HRT_ON_VIED_SUBSYSTEM_ACCESS, address - _hrt_master_port_subsystem_base_address, data, bytes);
}
  // The following implementation of hrt_master_port_set is not very efficient, 
  // but that doesn't matter because it is only temporarily.
  // as soon as HRT is replaced by the new program API this implementation is not used anymore.
static inline void hrt_master_port_set(hrt_address address, int data, size_t bytes)
{
  while (bytes > 0)  {
    vied_subsystem_store_8 (HRT_ON_VIED_SUBSYSTEM_ACCESS, address - _hrt_master_port_subsystem_base_address, data);
    address++;
    bytes--;
  }
}
#endif /* _HRT_BACKEND_DEVICE_ACCESS_IMPL_H */
