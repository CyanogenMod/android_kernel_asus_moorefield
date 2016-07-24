#ifndef _IA_CSS_TPROXY_GLOBAL_H_
#define _IA_CSS_TPROXY_GLOBAL_H_

#include "vied_nci_dma.h"

#define IA_CSS_N_TPROXY_EVENTS		64

#define IA_CSS_TPROXY_CTRL_DATA_BITS	32
#define IA_CSS_TPROXY_CTRL_DATA_BYTES	4

typedef uint8_t ia_css_tproxy_chan_id_t;

typedef uint32_t ia_css_tproxy_chan_handle_t;

struct ia_css_tproxy_unit_desc {
	uint32_t unit_width;
	uint32_t unit_height;
};

struct ia_css_tproxy_channel_desc {
	uint32_t element_extend_mode;
	uint32_t element_init_data;
	uint32_t padding_mode;
	uint32_t sampling_setup;
	uint32_t global_set_id;
	uint32_t ack_mode;
	uint32_t ack_addr;
	uint32_t ack_data;
	uint32_t completed_count;
};


struct ia_css_tproxy_span_desc {
	uint32_t unit_location;
	uint32_t span_row;
	uint32_t span_column;
	uint32_t span_width;
	uint32_t span_height;
	uint32_t span_mode;
	int32_t x_coordinate;
	int32_t y_coordinate;
};

struct ia_css_tproxy_terminal_desc {
	uint32_t region_origin;
	uint32_t region_width;
	uint32_t region_stride;
	uint32_t element_setup;
	uint32_t cio_info_setup;
	uint32_t port_mode;
};

// TODO: A dummy cricular buffer structure
struct ia_css_tproxy_buf_info {
	uint8_t size;
	uint8_t tail;
	uint32_t tail_addr;
};

struct ia_css_tproxy_transfer_config {
	struct ia_css_tproxy_channel_desc channel;
	struct ia_css_tproxy_unit_desc unit;
	struct ia_css_tproxy_buf_info buf_info;
};

enum ia_css_tproxy_terminal {
	IA_CSS_TPROXY_TERMINAL_A,
	IA_CSS_TPROXY_TERMINAL_B,
	IA_CSS_N_TPROXY_TERMINALS
};

enum ia_css_tproxy_extend_mode {
	IA_CSS_TPROXY_EXTEND_MODE_ZERO = VIED_NCI_DMA_EXTEND_MODE_ZERO,
	IA_CSS_TPROXY_EXTEND_MODE_SIGN = VIED_NCI_DMA_EXTEND_MODE_SIGN
};

enum ia_css_tproxy_padding_mode {
	IA_CSS_TPROXY_PADDING_CONSTANT =VIED_NCI_DMA_PADDING_MODE_CONSTANT,
	IA_CSS_TPROXY_PADDING_CLONE = VIED_NCI_DMA_PADDING_MODE_CLONE,
	IA_CSS_TPROXY_PADDING_MIRROR = VIED_NCI_DMA_PADDING_MODE_MIRROR,
	IA_CSS_TPROXY_PADDING_APPEND = VIED_NCI_DMA_PADDING_MODE_APPEND,
	IA_CSS_TPROXY_PADDING_TRUNCATE = VIED_NCI_DMA_PADDING_MODE_TRUNCATE
};

enum ia_css_tproxy_span_order {
	IA_CSS_TPROXY_SPAN_ORDER_ROW_FIRST = VIED_NCI_DMA_SPAN_ORDER_ROW_FIRST,
	IA_CSS_TPROXY_SPAN_ORDER_COL_FIRST = VIED_NCI_DMA_SPAN_ORDER_COLUMN_FIRST
};

enum ia_css_tproxy_addressing_mode {
	IA_CSS_TPROXY_ADDRESSING_BYTE_ADDRESS_BASED =
		VIED_NCI_DMA_ADDRESSING_MODE_BYTE_ADDRESS_BASED,
	IA_CSS_TPROXY_ADDRESSING_COORDINATE_ADDRESS_BASED =
		VIED_NCI_DMA_ADDRESSING_MODE_COORDINATE_BASED
};

enum ia_css_tproxy_ack_mode {
	IA_CSS_TPROXY_ACK_MODE_PASSIVE = VIED_NCI_DMA_ACK_MODE_PASSIVE,
	IA_CSS_TPROXY_ACK_MODE_ACTIVE = VIED_NCI_DMA_ACK_MODE_ACTIVE
};

enum ia_css_tproxy_transfer_command{
	IA_CSS_TPROXY_COMMAND_FILL_B = VIED_NCI_DMA_COMMAND_FILL_B,
	IA_CSS_TPROXY_COMMAND_FILL_B_ACK = VIED_NCI_DMA_COMMAND_FILL_B_ACK,
	IA_CSS_TPROXY_COMMAND_FILL_B_FLUSH_ACK = VIED_NCI_DMA_COMMAND_FILL_B_FLUSH_ACK,
	IA_CSS_TPROXY_COMMAND_FILL_A = VIED_NCI_DMA_COMMAND_FILL_A,
	IA_CSS_TPROXY_COMMAND_FILL_A_ACK = VIED_NCI_DMA_COMMAND_FILL_A_ACK,
	IA_CSS_TPROXY_COMMAND_FILL_A_FLUSH_ACK = VIED_NCI_DMA_COMMAND_FILL_A_FLUSH_ACK,
	IA_CSS_TPROXY_COMMAND_MOVE_AB = VIED_NCI_DMA_COMMAND_MOVE_AB,
	IA_CSS_TPROXY_COMMAND_MOVE_AB_ACK = VIED_NCI_DMA_COMMAND_MOVE_AB_ACK,
	IA_CSS_TPROXY_COMMAND_MOVE_AB_FLUSH_ACK = VIED_NCI_DMA_COMMAND_MOVE_AB_FLUSH_ACK,
	IA_CSS_TPROXY_COMMAND_MOVE_BA = VIED_NCI_DMA_COMMAND_MOVE_BA,
	IA_CSS_TPROXY_COMMAND_MOVE_BA_ACK = VIED_NCI_DMA_COMMAND_MOVE_BA_ACK,
	IA_CSS_TPROXY_COMMAND_MOVE_BA_FLUSH_ACK = VIED_NCI_DMA_COMMAND_MOVE_BA_FLUSH_ACK
};

struct ia_css_tproxy_terminal_config {
	struct ia_css_tproxy_span_desc span[IA_CSS_N_TPROXY_TERMINALS];
	struct ia_css_tproxy_terminal_desc terminal[IA_CSS_N_TPROXY_TERMINALS];
	uint32_t command;
};


#endif /*_IA_CSS_TPROXY_GLOBAL_H_*/
