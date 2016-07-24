#ifndef _HRT_BACKEND_IMPL_H 
#define _HRT_BACKEND_IMPL_H

 /* subsystem base address as specified in HSD */
extern unsigned long long _hrt_master_port_subsystem_base_address;

 /* subsystem last address of the subsystem as specified in HSD */
extern unsigned long long _hrt_master_port_subsystem_last_address;

 /* system memory base address as specified in HSD */
extern unsigned long long _hrt_master_port_system_memory_base_address;

 /* system memory last address of the subsystem as specified in HSD */
extern unsigned long long _hrt_master_port_system_memory_last_address;


#ifdef HRT_ON_VIED_SUBSYSTEM_ACCESS
  #if !defined(CFG_VIED_SUBSYSTEM_ACCESS_INLINE_IMPL) && \
      !defined(CFG_VIED_SUBSYSTEM_ACCESS_LIB_IMPL)
    #ifdef __HIVECC
      #define CFG_VIED_SUBSYSTEM_ACCESS_INLINE_IMPL
    #else   /* __HIVECC */
      #define CFG_VIED_SUBSYSTEM_ACCESS_LIB_IMPL
    #endif
  #endif
#endif

#ifdef HRT_HW

  #define _hrt_master_port_unaligned_load(addr, data, bytes) hrt_master_port_load(addr, data, bytes)
  #define _hrt_master_port_unaligned_store(addr, data, bytes) hrt_master_port_store(addr, data, bytes)

  #ifdef __HIVECC
    #ifdef HRT_ON_VIED_SUBSYSTEM_ACCESS
      #include "hrt_backend_subsystem_access_cell_impl.h"
    #else
      #ifdef HRT_USE_VIR_ADDRS
        #include "hrt_backend_hw_hivecc_virs_impl.h"
      #else   /* HRT_USE_VIR_ADDRS */
        #include "hrt_backend_hw_hivecc_impl.h"
      #endif  /* HRT_USE_VIR_ADDRS */
    #endif
  #else   /* __HIVECC */
    #ifdef HRT_USE_VIR_ADDRS
      #ifdef HRT_ON_VIED_SUBSYSTEM_ACCESS
        #include "hrt_backend_subsystem_access_impl.h"
      #else   /* HRT_ON_VIED_SUBSYSTEM_ACCESS */
        #include "hrt_backend_hw_virs_impl.h"
      #endif
    #else   /* HRT_USE_VIR_ADDRS */
      #ifdef HRT_ON_VIED_SUBSYSTEM_ACCESS
        #error HRT_ON_VIED_SUBSYSTEM_ACCESS only supported in combination with HRT_USE_VIR_ADDRS
      #endif
      #include "hrt_backend_hw_impl.h"
    #endif  /* HRT_USE_VIR_ADDRS */
  #endif  /* __HIVECC */

#else   /* HRT_HW */

  /* the following 6 macro's are not necessarry, but many testcases contain functions like _hrt_master_port_store_8 */
  #define _hrt_master_port_store_8(addr, data)   hrt_master_port_store_8((hrt_address)(addr),  (uint8_t) (data))
  #define _hrt_master_port_store_16(addr, data)  hrt_master_port_store_16((hrt_address)(addr), (uint16_t)(data))
  #define _hrt_master_port_store_32(addr, data)  hrt_master_port_store_32((hrt_address)(addr), (uint32_t)(data))
  #define _hrt_master_port_load_8(addr)          hrt_master_port_load_8((hrt_address)(addr))
  #define _hrt_master_port_load_16(addr)         hrt_master_port_load_16((hrt_address)(addr))
  #define _hrt_master_port_load_32(addr)         hrt_master_port_load_32((hrt_address)(addr))

  #include "hrt_backend_sim_impl.h"

#endif  /* HRT_HW */


#endif /* _HRT_BACKEND_IMPL_H */
