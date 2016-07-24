#ifndef _utils_h_
#define _utils_h_

#include <host.h>
#include <hrt/api.h>
#include <hrt/system.h>

#define MEMORY1 memory1
#define MEMORY2 memory2

#define MEMORY1_BASE_ADDR memory1_ip0_master_port_address
#define MEMORY2_BASE_ADDR memory2_ip0_master_port_address

#define DATA_A_BITS 64
#define DATA_B_BITS 32

#define PRECISION_A_BITS 2
#define PRECISION_B_BITS 1

#define SUBSAMPLING_INDEX_A_BITS 2
#define SUBSAMPLING_INDEX_B_BITS 1
#define SUBSAMPLING_OFFSET_A_BITS 3
#define SUBSAMPLING_OFFSET_B_BITS 2

#define SUBSAMPLING_INDEX_A_LSB  0
#define SUBSAMPLING_INDEX_B_LSB  (SUBSAMPLING_INDEX_A_LSB  + SUBSAMPLING_INDEX_A_BITS)
#define SUBSAMPLING_OFFSET_A_LSB (SUBSAMPLING_INDEX_B_LSB  + SUBSAMPLING_INDEX_B_BITS)
#define SUBSAMPLING_OFFSET_B_LSB (SUBSAMPLING_OFFSET_A_LSB + SUBSAMPLING_OFFSET_A_BITS)

#define RIGHT_PADDING_AMOUNT_BITS 5  /* log2(MaxPaddingAmount) */
#define LEFT_PADDING_AMOUNT_BITS  RIGHT_PADDING_AMOUNT_BITS
#define PADDING_MODE_BITS         2

#define PADDING_MODE_LSB         0
#define LEFT_PADDING_AMOUNT_LSB  (PADDING_MODE_LSB        + PADDING_MODE_BITS)
#define RIGHT_PADDING_AMOUNT_LSB (LEFT_PADDING_AMOUNT_LSB + LEFT_PADDING_AMOUNT_BITS)

#define REGSET_ID_LSB  9 /* 12 */
#define BANK_ID_LSB    6 /* 6 */
//#define REGSET_ID_LSB  6 /* 12 */
//#define BANK_ID_LSB    9 /* 6 */

#define GLOBAL_SETS 3

#define CHANNEL_WORDS 5
#define CHANNEL_REGS 13
#define CHANNEL_BANKS 5

#define DIMENSION_WORDS 1

#define LOGICAL_BLOCK_WIDTH_BITS  8
#define LOGICAL_BLOCK_HEIGHT_BITS 6

#define STRIDE_A_BITS          13 /* log2(MaxStride + 1) */
#define STRIDE_B_BITS          13 /* log2(MaxStride + 1) */
#define ELEMENT_SETUP_BITS     (2 + 2 + 1 + 1) /* log2(GlobalSets) + log2(#ElementPrecisionsA) + log2(#ElementPrecisionsB) + 1 */
#define ELEMENT_INIT_DATA_BITS 4
#define SAMPLING_SETUP_BITS    (3 + 2 + 2 + 1) /* log2(MaxSubsamplingFactorA) + log2(MaxSubsamplingFactorB) + log2(#SubsamplingFactorsA) + log2(#SubsamplingFactorsB) */
#define MARGIN_SETUP_BITS      4 /* log2(MaxElementsPerWord) */
#define PADDING_SETUP_BITS     (2*5 + 2) /* 2 * log2(MaxPaddingAmount) + 2 */
#define ACKNOWLEDGE_MODE_BITS  3 /* log2(MaxCompletedCount + 1) */
#define ACKNOWLEDGE_ADDR_BITS  32 /* control master address width */
#define ACKNOWLEDGE_DATA_BITS  32 /* control data address width */

/* command register fields */
#define CHANNEL_ID_BITS          3
#define DIMENSION_ID_BITS        3

#define COMMAND_MODIFIER_LSB     4
#define DIMENSION_INVALIDATE_LSB (4 + COMMAND_MODIFIER_LSB)
#define CHANNEL_INVALIDATE_LSB   (1 + DIMENSION_INVALIDATE_LSB)
#define DIMENSION_ID_LSB         (1 + CHANNEL_INVALIDATE_LSB)
#define CHANNEL_ID_LSB           (DIMENSION_ID_BITS + DIMENSION_ID_LSB)

#define dma_hrt_store_value(base_addr, reg_addr, val) \
  _hrt_master_port_store_32_volatile((base_addr) + addr, &val);

#define dma_hrt_load_value(base_addr, addr) \
  _hrt_slave_port_load_32_volatile((base_addr) +(addr))


/*
#define dma_hrt_store_value(dev_id, addr, val) \
  _hrt_slave_port_store_32_volatile(HRTCAT(dev_id,_s0), (addr), (val))

#define dma_hrt_load_value(dev_id, addr) \
  _hrt_slave_port_load_32(HRTCAT(dev_id,_s0), (addr))

#define _hrt_store_value(dev_id, addr, val) \
  _hrt_slave_port_store_32_volatile(HRTCAT(dev_id,_slv_in), (addr), (val))

#define _hrt_load_value(dev_id, addr) \
  _hrt_slave_port_load_32(HRTCAT(dev_id,_slv_in), (addr))*/

#define BYTES(bits) ((bits + 7) / 8)

//#define MIN(a,b) (a<b?a:b)
#define MAX(a,b) (a>b?a:b)

extern void initialize_memory1_block(int start_addr, int block_width, int block_height, int stride);

extern void initialize_memory2_block(int start_addr, int block_width, int block_height, int stride);


extern void load_block(
  char ***elem,
  int data_bits,
  int start_addr,
  int stride,
  int precision_bits,
  int subsampling_factor,
  int subsampling_offset,
  int logical_block_width,
  int logical_block_height,
  int left_margin,
  int left_padding_amount,
  int right_padding_amount,
  int padding_mode,
  int elem_init_sign_extend,
  int elem_init_data,
  int mem_id,
  int written_block
);

extern int compare_mem1_mem2_block (
  int logical_block_width, int logical_block_height,
  int data_bits1, int data_bits2,
  int start_addr1, int stride1, int precision_type1, int subsampling_factor1, int subsampling_offset1,
  int start_addr2, int stride2, int precision_type2, int subsampling_factor2, int subsampling_offset2,
  int sign_extend,
  int direction,
  int left_margin,
  int left_padding_amount,
  int right_padding_amount,
  int padding_mode,
  int elem_init_data,
  int left_margin_enable,
  int left_padding_enable,
  int right_padding_enable,
  int subsampling_enable
);

extern void write_dimension_regs (
  int dimension_id,
  int logical_block_width,
  int logical_block_height
);

extern void write_channel_regs (
  int channel_id,
  int logical_block_width,
  int logical_block_height,
  int logical_stride_a,
  int logical_stride_b,
  int packer,
  int element_init_data
);

extern void write_channel_subsampling_regs (
  int channel_id,
  int subsampling_index_a,
  int subsampling_index_b,
  int subsampling_offset_a,
  int subsampling_offset_b
);

extern void write_margin_setup (
  int channel_id,
  int left_margin
);
extern void write_padding_setup (
  int channel_id,
  int left_padding_amount,
  int right_padding_amount,
  int padding_mode
);
extern void write_ack_setup (
  int channel_id,
  int acknowledge_mode,
  int acknowledge_addr,
  int acknowledge_data
);

extern void write_virtual_channel_descriptor(
  int virtual_channel_refill_base_addr,
  int channel_id,
  int stride_a,
  int stride_b,
  int precision_bits_a, int precision_bits_b,
  int sign_extend,
  int element_init_data,
  int subsampling_factor_a,
  int subsampling_factor_b,
  int subsampling_offset_a,
  int subsampling_offset_b,
  int left_margin,
  int left_padding_amount,
  int right_padding_amount,
  int padding_mode,
  int acknowledge_mode,
  int acknowledge_addr,
  int acknowledge_data
);

extern void write_virtual_dimension_descriptor(
  int virtual_dimension_refill_base_addr,
  int dimension_id,
  int logical_block_width,
  int logical_block_height
);

extern void read_dimension_regs (
  int dimension_id,
  int *logical_block_width,
  int *logical_block_height
);

extern void read_channel_regs (
  int channel_id,
  int *logical_block_width,
  int *logical_block_height,
  int *logical_stride_a,
  int *logical_stride_b,
  int *packer,
  int *element_init_data
);

extern void read_channel_subsampling_regs (
  int channel_id,
  int *subsampling_index_a,
  int *subsampling_index_b,
  int *subsampling_offset_a,
  int *subsampling_offset_b
);

extern void read_margin_setup (
  int channel_id,
  int *left_margin
);
extern void read_padding_setup (
  int channel_id,
  int *left_padding_amount,
  int *right_padding_amount,
  int *padding_mode
);

extern void write_master_regs (
  int master_id,
  int srmd_support,
  /*int wmask_support,*/
  int burst_support,
  int max_stride
);

extern void read_master_regs (
  int master_id,
  int *srmd_support,
  /*int *wmask_support,*/
  int *burst_support,
  int *max_stride
);

extern void clear_irq ();

extern void write_global_regs (
  //int irq_mask,
  int max_linear_burst_size,
  int max_block_width,
  int max_block_height,
  int set
);

extern void write_virtual_refill_base_addr (
  int virtual_channel_refill_base_addr,
  int virtual_dimension_refill_base_addr
);

extern void read_global_regs (
  //int *irq,
  //int *irq_mask,
  int *max_linear_burst_size,
  int *max_block_width,
  int *max_block_height,
  int set
);

extern void do_command (int requester_id, int command);


extern int compose_command (
  int channel_id,
  int dimension_id,
  int channel_invalidate,
  int dimension_invalidate,
  int margin_enable,
  int left_padding_enable,
  int right_padding_enable,
  int subsampling_enable,
  int command
);

extern void write_request_regs (
  int requester_id,
  int addr_a,
  int addr_b,
  int channel_id,
  int dimension_id,
  int channel_invalidate,
  int dimension_invalidate,
  int margin_enable,
  int left_padding_enable,
  int right_padding_enable,
  int subsamping_enable,
  int command
);

extern void read_request_regs (
  int requester_id,
  int *addr_a,
  int *addr_b,
  int *channel_id,
  int *dimension_id,
  int *command
);

extern int read_completed_counter (
  int channel_id,
  int blocking
);


extern int precision_bits_2_precision_index(
  int master_id,
  int type
);

extern int subsampling_factor_2_subsampling_index(
  int master_id,
  int factor
);

extern void set_channel_bank_as_virtual(
  int channel_bank_id
);
extern void set_channel_bank_as_physical(
  int channel_bank_id
);
extern void set_dimension_bank_as_virtual(
  int dimension_bank_id
);
extern void set_dimension_bank_as_physical(
  int dimension_bank_id
);

extern int execute(
  int channel_id,
  int dimension_id,
  int requester_id,
  int command,
  int addr_a, int addr_b,
  int logical_stride_a, int logical_stride_b,
  int logical_block_width, int logical_block_height,
  int precision_bits_a, int precision_bits_b,
  int sign_extend,
  int subsampling_factor_a, int subsampling_factor_b,
  int subsampling_offset_a, int subsampling_offset_b,
  int left_margin,
  int left_padding_amount,
  int right_padding_amount,
  int padding_mode,
  int elem_init_data,
  int acknowledge_mode,
  int acknowledge_addr,
  int acknowledge_data,
  int left_margin_enable,
  int left_padding_enable,
  int right_padding_enable,
  int subsampling_enable
);

#endif
