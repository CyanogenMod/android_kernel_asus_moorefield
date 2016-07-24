#ifndef _HRT_BACKEND_H 
#define _HRT_BACKEND_H 

#include "hive_types.h"
#include "type_support.h"

static inline void  hrt_master_port_store_8(hrt_address addr, uint8_t data);
static inline void hrt_master_port_store_16(hrt_address address, uint16_t data);
static inline void hrt_master_port_store_32(hrt_address address, uint32_t data);
static inline uint8_t hrt_master_port_load_8(hrt_address address);
static inline uint16_t hrt_master_port_load_16(hrt_address address);
static inline uint32_t hrt_master_port_load_32(hrt_address address);
static inline void hrt_master_port_store(hrt_address address, const void *data, size_t bytes);
static inline void hrt_master_port_load(hrt_address address, void *data, size_t bytes);
static inline void hrt_master_port_set(hrt_address address, int data, size_t bytes);

#include "hrt_backend_impl.h"

#endif /* _HRT_BACKEND_H */
