#ifndef __IA_CSS_ISYSAPI_TYPES_H__
#define __IA_CSS_ISYSAPI_TYPES_H__

#include "ia_css_isysapi_fw_types.h"

#include "ia_css_return_token.h"
#include "ia_css_output_buffer.h"
#include "ia_css_input_buffer.h"

/**
 * struct ia_css_isys_resolution: Generic resolution structure.
 * @Width
 * @height
 */
struct ia_css_isys_resolution {
	unsigned int width;
	unsigned int height;
};

/**
 * struct ia_css_isys_output_pin_payload
 * @addr: Points to ouput pin buffer
 */
struct ia_css_isys_output_pin_payload {
	ia_css_return_token out_buf_id;
	ia_css_output_buffer_css_address addr;
};

/**
 * struct ia_css_isys_output_pin_info
 * @pt: pin type
 * @ft: frame format type
 * @dt: mipi data type
 * @output_res: pin output resolution
 * @watermark_in_lines: pin watermark level in lines
 * @send_irq: assert if pin event should trigger irq
 *
 */
struct ia_css_isys_output_pin_info {
	struct ia_css_isys_resolution output_res;
	enum ia_css_isys_pin_type pt;
	union {
		enum ia_css_isys_frame_format_type ft;
		enum ia_css_isys_mipi_data_type dt;
	} type_specifics;
	unsigned int watermark_in_lines;
	unsigned int send_irq;
	unsigned int input_pin_index;
};

/**
 * struct ia_css_isys_input_pin
 * @addr: Points to input pin buffer in DDR
 *
 */
struct ia_css_isys_input_pin {
	ia_css_return_token in_buf_id;
	ia_css_input_buffer_css_address addr;
};

/**
 * struct ia_css_isys_input_pin_info
 * @dt
 * @input_res
 * @crop
 */
struct ia_css_isys_input_pin_info {
	enum ia_css_isys_mipi_data_type dt;
	struct ia_css_isys_resolution input_res;
	struct {
		unsigned int top_offset;
		unsigned int left_offset;
		unsigned int bottom_offset;
		unsigned int right_offset;
	} crop;
};

/**
 * struct ia_css_isys_output_pin
 * @payload: payload data
 * @info: info data
 */
struct ia_css_isys_output_pin {
	struct ia_css_isys_output_pin_payload payload;
	struct ia_css_isys_output_pin_info info;
};

/**
 * struct ia_css_isys_isl_cfg. Describes the ISL cfg
 */
struct ia_css_isys_isl_cfg {
	unsigned int isl_input_pin_index0;
	unsigned int isl_input_pin_index1;
};

/**
 * struct ia_css_isys_isa_cfg. Describes the ISA cfg
 */
struct ia_css_isys_isa_cfg {
	struct {
		unsigned int blc_enabled;			/* acc id 0, set if process required */
		unsigned int lsc_enabled;			/* acc id 1, set if process required */
		unsigned int dpc_enabled;			/* acc id 2, set if process required */
		unsigned int downscaler_enabled;	/* acc id 3, set if process required */
		unsigned int awb_enabled;			/* acc id 4, set if process required */
		unsigned int af_enabled;			/* acc id 5, set if process required */
		unsigned int ae_enabled;			/* acc id 6, set if process required */
	} acc_enabled_set;
	unsigned int isa_input_pin_index;
};

 /**
 * struct ia_css_isys_stream_cfg_data
 * ISYS stream configuration data structure
 * @isl_enabled: indicates whether stream requores ISL
 * @src: Stream source index e.g. MIPI_generator_0, CSI2-rx_1
 * @vc: MIPI Virtual Channel (up to 4 virtual per physical channel)
 * @the rest: input/output pin descriptors
 */
struct ia_css_isys_stream_cfg_data {
	struct ia_css_isys_input_pin_info input_pins[MAX_IPINS];
	struct ia_css_isys_output_pin_info output_pins[MAX_OPINS];
	union {
		struct ia_css_isys_isl_cfg isl_cfg;
		struct ia_css_isys_isa_cfg isa_cfg;
	} isl_isa_cfg;
	enum ia_css_isys_isl_use isl_use;
	enum ia_css_isys_stream_source src;
	enum ia_css_isys_mipi_vc vc;
	unsigned int nof_input_pins;
	unsigned int nof_output_pins;
};

/**
 * struct ia_css_isys_frame_buff_set - frame buffer set
 * @the rest: output pin addresses and descriptors
 * @lsc_param:
 * @dpc_param:
 * @blc_param
 * @send_irq_sof: send irq on frame sof event
 * @send_irq_eof: send irq on frame eof event
 */

struct ia_css_isys_frame_buff_set {
	struct ia_css_isys_output_pin output_pins[MAX_OPINS];
	struct {
		struct ia_css_isys_input_pin blc_param;
		struct ia_css_isys_input_pin lsc_param;
		struct ia_css_isys_input_pin dpc_param;
	} input_pin_set;
	unsigned int send_irq_sof;
	unsigned int send_irq_eof;
};


/**
 * struct ia_css_isys_resp_info
 * @type
 * @error
 * @timestamp
 * @stream_handle
 * @pin: this member is only valid for pin event related responses
 */
struct ia_css_isys_resp_info {
	enum ia_css_isys_resp_type type;
	unsigned int timestamp[2];
	unsigned int stream_handle;
	int error;
	union {
		struct ia_css_isys_output_pin pin;
		ia_css_return_token buf_id;
	} resp_data;
};


/**
 * struct ia_css_isys_buffer_partition - buffer partition information
 * @block_size: memory block size for a given stream
 * stream_id0 will make use of block 0
 * stream_id1 will make use of block 1
 * etc..
 * @nof_blocks: number of memory blocks to allocate
 *
 */
struct ia_css_isys_buffer_partition {
	unsigned int block_size[STREAM_ID_MAX];
	unsigned int nof_blocks;
};


#endif /*__IA_CSS_ISYSAPI_TYPES_H__*/
