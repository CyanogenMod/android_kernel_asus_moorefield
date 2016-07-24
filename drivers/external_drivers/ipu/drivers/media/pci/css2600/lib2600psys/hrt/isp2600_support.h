#ifndef _isp2600_support_h
#define _isp2600_support_h

#ifndef ISP2600_VECTOR_TYPES
/* This typedef is to be able to include hive header files
   in the host code which is useful in crun */
typedef char *tmemvectors, *tmemvectoru, *tvector;
#endif

#define hrt_isp_dmem(cell) HRT_PROC_TYPE_PROP(cell, _base_dmem)
#define hrt_isp_vmem(cell) HRT_PROC_TYPE_PROP(cell, _simd_vmem)
#define hrt_isp_bamem(cell) HRT_PROC_TYPE_PROP(cell, _simd_bamem)

#define hrt_isp_dmem_master_port_address(cell) hrt_mem_master_port_address(cell, hrt_isp_dmem(cell))
#define hrt_isp_vmem_master_port_address(cell) hrt_mem_master_port_address(cell, hrt_isp_vmem(cell))
#define hrt_isp_bamem_master_port_address(cell) hrt_mem_master_port_address(cell, hrt_isp_bamem(cell))

#if ISP_HAS_HIST
  #define hrt_isp_hist(cell) HRT_PROC_TYPE_PROP(cell, _simd_histogram)
  #define hrt_isp_hist_master_port_address(cell) hrt_mem_master_port_address(cell, hrt_isp_hist(cell))
#endif

#endif /* _isp2600_support_h */
