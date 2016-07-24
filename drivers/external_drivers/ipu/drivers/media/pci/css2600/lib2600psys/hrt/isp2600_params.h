/* Version */
#define RTL_VERSION

/* Cell name  */
#define ISP_CELL_TYPE                          isp2600
#define ISP_VMEM                               simd_vmem
#define _HRT_ISP_VMEM                          isp2600_simd_vmem

/* data-path */
#define ISP_SCALAR_WIDTH                       32
#define ISP_SLICE_NELEMS                       8
#define ISP_VEC_NELEMS                         32
#define ISP_VEC_ELEMBITS                       16
#define ISP_CLONE_DATAPATH_IS_16               0

/* memories */
#define ISP_VMEM_DEPTH                         2048
#define ISP_VMEM_BSEL_DOWNSAMPLE               8
#define ISP_VMEM_ELEMBITS                      16
#define ISP_VMEM_ELEM_PRECISION                16
#define ISP_VMEM_BIG_ENDIANNESS                0

#define ISP_HAS_BAMEM                          1
#if ISP_HAS_BAMEM
  #define ISP_BAMEM_DEPTH                      2048
  #define ISP_BAMEM_BSEL_DOWNSAMPLE            8
  #define ISP_BAMEM_ELEMBITS                   16
  #define ISP_BAMEM_ELEM_PRECISION             16
  #define ISP_BAMEM_MIN_ELEM_PRECISION         8
  #define ISP_BAMEM_MAX_BOI_HEIGHT             4
  #define ISP_BAMEM_LATENCY                    7
  #define ISP_BAMEM_BANK_NARROWING_FACTOR      2
  #define ISP_BAMEM_NR_DATA_PLANES             8
  #define ISP_BAMEM_NR_CFG_REGISTERS           16
  #define ISP_BAMEM_LININT                     0
  #define ISP_BAMEM_XBM_BITS_LOC               0
  #define ISP_BAMEM_XBM_BITS_EXT               7
  #define ISP_BAMEM_DAP_BITS                   3
  #define ISP_BAMEM_LININT_FRAC_BITS           0
  #define ISP_BAMEM_PID_BITS                   3
  #define ISP_BAMEM_OFFSET_BITS                17
  #define ISP_BAMEM_ADDRESS_BITS               23
  #define ISP_BAMEM_RID_BITS                   4
  #define ISP_BAMEM_TRANSPOSITION              0
  #define ISP_BAMEM_VEC_PLUS_SLICE             1
  #define ISP_BAMEM_BLK_PLUS_SLICE             1
  #define ISP_BAMEM_ARB_SERVICE_CYCLE_BITS     1
  #define ISP_BAMEM_LUT_ELEMS                  8
  #define ISP_BAMEM_LUT_ADDR_WIDTH             13
  #define ISP_BAMEM_HALF_BLOCK_WRITE           0
  #define ISP_BAMEM_SMART_FETCH                0
  #define ISP_BAMEM_BIG_ENDIANNESS             0
  #define ISP_BAMEM_INP_SHIELD                 1
  #define ISP_BAMEM_OUT_SHIELD                 1
  #define ISP_BAMEM_BANK_INP_SHIELD            1
  #define ISP_BAMEM_BANK_OUT_SHIELD            0
  #define ISP_BAMEM_MEMORY_BANK_LATENCY        2
#endif /* ISP_HAS_BAMEM */

#define ISP_HAS_HIST                           0
#if ISP_HAS_HIST
  #define ISP_HIST_ADDRESS_BITS                12
  #define ISP_HIST_ALIGNMENT                   4
  #define ISP_HIST_COMP_IN_PREC                12
  #define ISP_HIST_DEPTH                       1024
  #define ISP_HIST_WIDTH                       24
  #define ISP_HIST_COMPONENTS                  4
#endif /* ISP_HAS_HIST */

/* Derived values */
#define ISP_VEC_WIDTH                          512
#define ISP_SLICE_WIDTH                        128
#define ISP_VMEM_WIDTH                         512
#define ISP_VMEM_ALIGN                         64

#if ISP_HAS_BAMEM
  #define ISP_BAMEM_ALIGN_ELEM                 2
  #define ISP_SIMDBALSU                        1
  #define ISP_BAMEM_WIDTH                      512
  #define ISP_BAMEM_ALIGN                      64
#endif /* ISP_HAS_BAMEM */

/* convenient shortcuts for software*/
#define ISP_NWAY                               ISP_VEC_NELEMS
#define NBITS                                        ISP_VEC_ELEMBITS

#define _isp_ceil_div(a,b)                           (((a)+(b)-1)/(b))

#ifdef C_RUN
#define ISP_VEC_ALIGN                          (_isp_ceil_div(ISP_VEC_WIDTH, 64)*8)
#else
#define ISP_VEC_ALIGN                          ISP_VMEM_ALIGN
#endif

/* HRT specific vector support */
#define isp2600_vector_alignment                ISP_VEC_ALIGN
#define isp2600_vector_elem_bits                ISP_VMEM_ELEMBITS
#define isp2600_vector_elem_precision           ISP_VMEM_ELEM_PRECISION
#define isp2600_vector_num_elems                ISP_VEC_NELEMS

/* HAAR maximum shift value */
#define ISP_HAAR_MAXSHIFT                      0

