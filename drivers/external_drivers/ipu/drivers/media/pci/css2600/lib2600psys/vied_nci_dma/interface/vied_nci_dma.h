/*! \file */

#ifndef _VIED_NCI_DMA_H_
#define _VIED_NCI_DMA_H_

#include "type_support.h"
#include "storage_class.h"
#include "error_support.h"
#include "vied_nci_dma_global.h"

//
// DMA NCI Interfaces
//

// DMA NCI data structures
#ifdef VIED_NCI_DMA_ISYS
enum vied_nci_dma_dev_id {
	VIED_NCI_DMA_FW,
	VIED_NCI_DMA_EXT0,
	VIED_NCI_DMA_EXT1,
	VIED_NCI_ISA_DMA,
	VIED_NCI_N_DMA_DEV
};
#else
enum vied_nci_dma_dev_id {
	VIED_NCI_DMA_FW,
	VIED_NCI_DMA_EXT0,
	VIED_NCI_DMA_EXT1R,
	VIED_NCI_DMA_EXT1W,
	VIED_NCI_DMA_INT,
	VIED_NCI_ISA_DMA,
	VIED_NCI_IPFD_DMA,
	VIED_NCI_N_DMA_DEV
};
#endif

struct vied_nci_dma_chan_t {
	uint8_t dev_id;
	uint8_t chan_id;
};

/**
 * DMA Unit Description
 * The unit is defining the size of a logical block. This is the
 * granularity on which the commands operate.
 */
struct vied_nci_dma_unit_desc_t {
	uint32_t unit_width;   /**< The width of a block expressed in elements @b -1*/
	uint32_t unit_height;  /**< The height of a block, so called lines @b -1*/
};

struct vied_nci_dma_unit_desc_bits_t {
	uint8_t desc_words;
	uint8_t unit_width_bits;
	uint8_t unit_height_bits;
};

struct vied_nci_dma_channel_desc_t {

	uint32_t element_extend_mode;
	uint32_t element_init_data;
	uint32_t padding_mode;
	uint32_t sampling_setup;
	uint32_t global_set_id;
	uint32_t ack_mode;             /**< 0: passive mode, 1: active mode (if channel bank is cached mode) */
	uint32_t ack_addr;
	uint32_t ack_data;
	uint32_t completed_count;
};

struct vied_nci_dma_channel_desc_bits_t {
	uint8_t desc_words;
	uint8_t element_extend_mode_bits;
	uint8_t element_init_data_bits;
	uint8_t padding_mode_bits;
	uint8_t sampling_setup_bits;
	uint8_t global_set_id_bits;
	uint8_t ack_mode_bits;
	uint8_t ack_addr_bits;
	uint8_t ack_data_bits;
	uint8_t completed_count_bits;
};

enum vied_nci_dma_extend_mode_t {
	VIED_NCI_DMA_EXTEND_MODE_ZERO,
	VIED_NCI_DMA_EXTEND_MODE_SIGN
};

enum vied_nci_dma_sampling_setup_t {
	VIED_NCI_DMA_SAMPLING_SETUP_FACTOR_1 = 0,
	VIED_NCI_DMA_SAMPLING_SETUP_FACTOR_2 = 1,
	VIED_NCI_DMA_SAMPLING_SETUP_FACTOR_4 = 2
};

enum vied_nci_dma_padding_mode_t {
	VIED_NCI_DMA_PADDING_MODE_CONSTANT,
	VIED_NCI_DMA_PADDING_MODE_CLONE,
	VIED_NCI_DMA_PADDING_MODE_MIRROR,
	VIED_NCI_DMA_PADDING_MODE_APPEND,
	VIED_NCI_DMA_PADDING_MODE_TRUNCATE
};

enum vied_nci_dma_ack_mode_t {
	VIED_NCI_DMA_ACK_MODE_PASSIVE,
	VIED_NCI_DMA_ACK_MODE_ACTIVE
};
/**
 * Terminal Descriptor
 * Describes either the source or destination terminal, expressed in elements
 * and bytes. It should cover the whole region in memory.
 * So for i.e. DDR 2D image frame, make it cover the whole area in memory.
 */
struct vied_nci_dma_terminal_desc_t {
	uint32_t region_origin;  /**< start address of a region in memory */
	uint32_t region_width;   /**< width of the region in elements @b -1 */
	uint32_t region_stride;  /**< stride of the region in bytes @b ?? -1 ??*/
	uint32_t element_setup;  /**< the type of element, in BXT 0 = 8 bit, 3 = 16 bit*/
	uint32_t cio_info_setup; /**< .... */
	uint32_t port_mode;      /**< port mode: 0 = memory mapped address space, 1 = I/O port (e.g. FIFO) */
};

struct vied_nci_dma_terminal_desc_bits_t {
	uint8_t desc_words;
	uint8_t region_origin_bits;
	uint8_t region_width_bits;
	uint8_t region_stride_bits;
	uint8_t element_setup_bits;
	uint8_t cio_info_setup_bits;
	uint8_t port_mode_bits;
};

enum vied_nci_dma_element_setup_fw_t {
	VIED_NCI_DMA_FW_ELEMENT_PRECISION_32BIT
};

enum vied_nci_dma_element_setup_int_t {
	VIED_NCI_DMA_INT_ELEMENT_PRECISION_8BIT,
	VIED_NCI_DMA_INT_ELEMENT_PRECISION_16BIT,
};

enum vied_nci_dma_element_setup_ext_m0_t {
	VIED_NCI_DMA_EXT_M0_ELEMENT_PRECISION_8BIT,
	VIED_NCI_DMA_EXT_M0_ELEMENT_PRECISION_16BIT
};

enum vied_nci_dma_element_setup_ext_m1_t {
	VIED_NCI_DMA_EXT_M1_ELEMENT_PRECISION_8BIT,
	VIED_NCI_DMA_EXT_M1_ELEMENT_PRECISION_10BIT,
	VIED_NCI_DMA_EXT_M1_ELEMENT_PRECISION_12BIT,
	VIED_NCI_DMA_EXT_M1_ELEMENT_PRECISION_16BIT
};

/**
 * Span Descriptor
 * Describes either the source or destination span, expressed in terms of units.
 * It covers the whole area, just like the terminal
 * See @vied_nci_dma_unit_desc_t for more information on the definition of a
 * unit. It is also known as a logical block.
 */
struct vied_nci_dma_span_desc_t {
	uint32_t unit_location; /**< TBD: not sure how to explain, 0 when no special case for now */
	uint32_t span_row;      /**< TBD: row where to start, 0 when no special case for now */
	uint32_t span_column;   /**< TBD: col where to start, 0 when no special case for now */
	uint32_t span_width;    /**< number of units the area to cover is wide @b -1 */
	uint32_t span_height;   /**< number of units the area is high (i.e. in DDR buffer height / unit_height) @b -1 */
	uint32_t span_mode;     /**< 0x1 means coordinate based addressing */
	int32_t x_coordinate;
	int32_t y_coordinate;
};

struct vied_nci_dma_span_desc_bits_t {
	uint8_t desc_words;
	uint8_t unit_location_bits;
	uint8_t span_column_bits;
	uint8_t span_row_bits;
	uint8_t span_width_bits;
	uint8_t span_height_bits;
	uint8_t span_mode_bits;
	uint8_t x_coordinate_bits;
	uint8_t y_coordinate_bits;
};

enum vied_nci_dma_span_order_t {
	VIED_NCI_DMA_SPAN_ORDER_ROW_FIRST,
	VIED_NCI_DMA_SPAN_ORDER_COLUMN_FIRST
};

enum vied_nci_dma_addressing_mode_t {
	VIED_NCI_DMA_ADDRESSING_MODE_BYTE_ADDRESS_BASED,
	VIED_NCI_DMA_ADDRESSING_MODE_COORDINATE_BASED
};

enum vied_nci_dma_span_mode_t {
	VIED_NCI_DMA_SPAN_MODE_ROW_FIRST_BYTE_ADDRESS_BASED,
	VIED_NCI_DMA_SPAN_MODE_ROW_FIRST_COORDINATE_BASED,
	VIED_NCI_DMA_SPAN_MODE_COLUMN_FIRST_BYTE_ADDRESS_BASED,
	VIED_NCI_DMA_SPAN_MODE_COLUMN_FIRST_COORDINATE_BASED
};

struct vied_nci_dma_request_desc_t {
	uint32_t instruction;
	uint32_t descriptor_id_setup_1;
	uint32_t descriptor_id_setup_2;
};

enum vied_nci_dma_instruction_format_t {
	VIED_NCI_DMA_INSTRUCTION_FORMAT_EXECUTE,
	VIED_NCI_DMA_INSTRUCTION_FORMAT_INVALIDATE
};

enum vied_nci_dma_transfer_direction_t {
	VIED_NCI_DMA_TRANSFER_DIRECTION_AB,
	VIED_NCI_DMA_TRANSFER_DIRECTION_BA
};

enum vied_nci_dma_transfer_kind_t {
	VIED_NCI_DMA_TRANSFER_KIND_INIT,
	VIED_NCI_DMA_TRANSFER_KIND_MOVE
};

enum vied_nci_dma_descriptor_kind_t {
	VIED_NCI_DMA_DESCRIPTOR_KIND_CHANNEL,
	VIED_NCI_DMA_DESCRIPTOR_KIND_TERMINAL,
	VIED_NCI_DMA_DESCRIPTOR_KIND_UNIT,
	VIED_NCI_DMA_DESCRIPTOR_KIND_SPAN
};

enum vied_nci_dma_command_t{
	VIED_NCI_DMA_COMMAND_FILL_B = 0x0,
	VIED_NCI_DMA_COMMAND_FILL_B_ACK = 0x1,
	VIED_NCI_DMA_COMMAND_FILL_B_FLUSH_ACK = 0x3,
	VIED_NCI_DMA_COMMAND_FILL_A = 0x4,
	VIED_NCI_DMA_COMMAND_FILL_A_ACK = 0x5,
	VIED_NCI_DMA_COMMAND_FILL_A_FLUSH_ACK = 0x7,
	VIED_NCI_DMA_COMMAND_MOVE_AB = 0x8,
	VIED_NCI_DMA_COMMAND_MOVE_AB_ACK = 0x9,
	VIED_NCI_DMA_COMMAND_MOVE_AB_FLUSH_ACK = 0xb,
	VIED_NCI_DMA_COMMAND_MOVE_BA = 0xc,
	VIED_NCI_DMA_COMMAND_MOVE_BA_ACK = 0xd,
	VIED_NCI_DMA_COMMAND_MOVE_BA_FLUSH_ACK = 0xf
};

struct vied_nci_dma_request_desc_bits_t {
	uint32_t macro_size_bits;
	uint32_t descriptor_id_bits;
	uint32_t terminal_bits;
	uint32_t span_bits;
	uint32_t channel_bits;
	uint32_t unit_bits;
};

enum vied_nci_dma_bank_mode_t {
	VIED_NCI_DMA_BANK_MODE_NON_CACHED,
	VIED_NCI_DMA_BANK_MODE_CACHED
};

struct vied_nci_dma_global_desc_t {
	uint32_t unit_descriptor_base_addr;
	uint32_t span_descriptor_base_addr;
	uint32_t terminal_descriptor_base_addr;
	uint32_t channel_descriptor_base_addr;
	uint32_t max_block_height;
	uint32_t max_1d_block_width[MAX_GLOBAL_SETS];
	uint32_t max_2d_block_width[MAX_GLOBAL_SETS];
};

struct vied_nci_dma_master_desc_t {
	uint32_t srmd_support;
	uint32_t burst_support;
	uint32_t max_stride;
};

enum vied_nci_dma_requestor_type {
	VIED_NCI_DMA_SHARED_REQUESTOR,
	VIED_NCI_DMA_DEDICATED_REQUESTOR
};

struct vied_nci_dma_chan_res {
	uint16_t chan_id;
	uint16_t unit_id;
	uint16_t terminal_A_id;
	uint16_t terminal_B_id;
	uint16_t span_A_id;
	uint16_t span_B_id;
	uint16_t requestor_id;
	uint32_t chan_addr;
	uint32_t unit_addr;
	uint32_t terminal_A_addr;
	uint32_t terminal_B_addr;
	uint32_t span_A_addr;
	uint32_t span_B_addr;
	uint32_t requestor_addr;
};

struct vied_nci_dma_dev_t {
	uint8_t  dev_id;
	uint8_t  num_of_channels;
	uint8_t  num_of_terminals;
	uint8_t  num_of_spans;
	uint8_t  num_of_units;
	uint8_t  num_of_physical_channels;
	uint8_t  num_of_logic_channels;

	struct vied_nci_dma_chan_t *chans;
	uint8_t  global_sets;
	uint8_t  master_banks;
	uint32_t max_block_height;
	uint32_t max_block_width;
	uint32_t max_linear_burst_size;
	uint8_t  srmd_support[MAX_MASTER_BANKS];
	uint8_t  burst_support[MAX_MASTER_BANKS];

	uint8_t  num_of_terminal_banks;
	uint8_t  num_of_span_banks;
	uint8_t  num_of_channel_banks;
	uint8_t  num_of_unit_banks;
	uint8_t  num_of_requestors;

	uint8_t  bank_mode;
	uint8_t  req_type;

	// DMA dev base address
	uint32_t base_addr;

	// Cache mode desc words
	struct vied_nci_dma_terminal_desc_bits_t terminal_desc_bits;
	struct vied_nci_dma_span_desc_bits_t span_desc_bits;
	struct vied_nci_dma_channel_desc_bits_t channel_desc_bits;
	struct vied_nci_dma_unit_desc_bits_t unit_desc_bits;
	struct vied_nci_dma_request_desc_bits_t request_desc_bits;

	// slave address decomposition
	uint8_t byte_id_idx;
	uint8_t register_id_idx;
	uint8_t bank_id_idx;
	uint8_t group_id_idx;

	uint32_t terminal_desc_base_addr;
	uint32_t unit_desc_base_addr;
	uint32_t channel_desc_base_addr;
	uint32_t span_desc_base_addr;

	uint16_t terminal_desc_size;
	uint16_t unit_desc_size;
	uint16_t channel_desc_size;
	uint16_t span_desc_size;

	// register availability
	uint8_t  bank_mode_reg_available;
	uint8_t  region_stride_reg_available;
};

struct vied_nci_dma_dev_config_t {
	enum vied_nci_dma_bank_mode_t bank_mode;
	enum vied_nci_dma_requestor_type req_type;
	uint32_t num_of_channels;
	struct vied_nci_dma_chan_t *chan_handle_addr;
	uint32_t terminal_desc_base_addr;
	uint32_t unit_desc_base_addr;
	uint32_t channel_desc_base_addr;
	uint32_t span_desc_base_addr;
};

struct vied_nci_dma_transfer_config_t {
	struct vied_nci_dma_unit_desc_t unit_desc;
	struct vied_nci_dma_channel_desc_t channel_desc;
};

struct vied_nci_dma_terminal_config_t {
	struct vied_nci_dma_terminal_desc_t terminal_desc[MAX_MASTER_BANKS];
	struct vied_nci_dma_span_desc_t span_desc[MAX_MASTER_BANKS];
};


/** Open the DMA device with device id \p dev_id.
*/
STORAGE_CLASS_EXTERN struct vied_nci_dma_dev_t* vied_nci_dma_open(
		enum vied_nci_dma_dev_id dev_id);

/** Close the DMA device \p dev.
*/
STORAGE_CLASS_EXTERN int vied_nci_dma_close(
		struct vied_nci_dma_dev_t *dev);

/** Configure the DMA device \p dev.
*/
STORAGE_CLASS_EXTERN int vied_nci_dma_configure(
		struct vied_nci_dma_dev_t *dev,
		const struct vied_nci_dma_dev_config_t *config);

/** Configure the DMA channel \p chan, with configuration \p config.
*/
STORAGE_CLASS_EXTERN int vied_nci_dma_chan_configure(
		const struct vied_nci_dma_chan_t *chan,
		const struct vied_nci_dma_terminal_config_t *config);

/** Open the DMA channel \p chan_id, of DMA device \p dev_id.
 */
STORAGE_CLASS_EXTERN struct vied_nci_dma_chan_t* vied_nci_dma_chan_open(
		enum vied_nci_dma_dev_id dev_id,
		uint8_t chan_id,
		const struct vied_nci_dma_transfer_config_t *config);

/** Start the initial DMA transfer on channel \p chan.
 */
STORAGE_CLASS_EXTERN int vied_nci_dma_chan_start(
		const struct vied_nci_dma_chan_t *chan);

/** Start the subsequent DMA transfers on channel \p chan.
 */
STORAGE_CLASS_EXTERN int vied_nci_dma_chan_next(
		const struct vied_nci_dma_chan_t *chan,
		const enum vied_nci_dma_command_t command,
		const uint32_t n);

/** Stop DMA transfer on DMA channel \p chan.
 */
STORAGE_CLASS_EXTERN int vied_nci_dma_chan_stop(
		const struct vied_nci_dma_chan_t *chan);

/** Close DMA channel \p chan.
 */
STORAGE_CLASS_EXTERN int vied_nci_dma_chan_close(
		struct vied_nci_dma_chan_t *chan);

/** Get resource assigned to a channel \p chan.
 */
STORAGE_CLASS_EXTERN int vied_nci_dma_get_chan_res(
		const struct vied_nci_dma_chan_t *chan,
		struct vied_nci_dma_chan_res *res);

#ifdef VIED_NCI_DMA_DEBUG
/** Wait for DMA transfer acknowledgement on channel \p chan
*/
STORAGE_CLASS_EXTERN int vied_nci_dma_chan_wait_for_ack(
		const struct vied_nci_dma_chan_t *chan);
#endif

#endif				// _VIED_NCI_DMA_H_
