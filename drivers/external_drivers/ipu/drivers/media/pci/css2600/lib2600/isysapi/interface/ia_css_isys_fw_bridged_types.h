#ifndef __IA_CSS_ISYS_FW_BRIDGED_TYPES_H__
#define __IA_CSS_ISYS_FW_BRIDGED_TYPES_H__

#include "platform_support.h"

#include "ia_css_isysapi_fw_types.h"


/**
 * struct ia_css_isys_resolution: Generic resolution structure.
 * @Width
 * @height
 */
struct ia_css_isys_resolution_comm {
	aligned_uint32(unsigned int, width);
	aligned_uint32(unsigned int, height);
};

/**
 * struct ia_css_isys_output_pin_payload
 * @addr: Points to ouput pin buffer
 */
struct ia_css_isys_output_pin_payload_comm {
	aligned_uint64(ia_css_return_token, out_buf_id);
	aligned_uint32(ia_css_output_buffer_css_address, addr);
	//int ALIGN;
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
struct ia_css_isys_output_pin_info_comm {
	aligned_struct(struct ia_css_isys_resolution_comm, output_res);
	aligned_enum(enum ia_css_isys_pin_type, pt);
	union {
		aligned_enum(enum ia_css_isys_frame_format_type, ft);
		aligned_enum(enum ia_css_isys_mipi_data_type, dt);
	} type_specifics;
	aligned_uint32(unsigned int, watermark_in_lines);
	aligned_uint32(unsigned int, send_irq);
	aligned_uint32(unsigned int, input_pin_index);
	//int ALIGN;
};

/**
 * struct ia_css_isys_input_pin
 * @addr: Points to input pin buffer in DDR
 *
 */
struct ia_css_isys_input_pin_comm {
	aligned_uint64(ia_css_return_token, in_buf_id);
	aligned_uint32(ia_css_input_buffer_css_address, addr);
	//int ALIGN;
};

/**
 * struct ia_css_isys_input_pin_info
 * @dt
 * @input_res
 * @crop
 */
struct ia_css_isys_input_pin_info_comm {
	aligned_struct(struct ia_css_isys_resolution_comm, input_res);
	aligned_enum(enum ia_css_isys_mipi_data_type, dt);
	struct {
		aligned_uint32(unsigned int, top_offset);
		aligned_uint32(unsigned int, left_offset);
		aligned_uint32(unsigned int, bottom_offset);
		aligned_uint32(unsigned int, right_offset);
	} crop;
	//int ALIGN;
};

/**
 * struct ia_css_isys_output_pin
 * @payload: payload data
 * @info: info data
 */
struct ia_css_isys_output_pin_comm {
	aligned_struct(struct ia_css_isys_output_pin_payload_comm, payload);
	aligned_struct(struct ia_css_isys_output_pin_info_comm, info);
};

/**
 * struct ia_css_isys_isl_cfg. Describes the ISL cfg
 */
struct ia_css_isys_isl_cfg_comm {
	aligned_uint32(unsigned int, isl_input_pin_index0);
	aligned_uint32(unsigned int, isl_input_pin_index1);
};

/**
 * struct ia_css_isys_isa_cfg. Describes the ISA cfg
 */
struct ia_css_isys_isa_cfg_comm {
	struct {
		aligned_uint32(unsigned int, blc_enabled);			/* acc id 0, set if process required */
		aligned_uint32(unsigned int, lsc_enabled);			/* acc id 1, set if process required */
		aligned_uint32(unsigned int, dpc_enabled);			/* acc id 2, set if process required */
		aligned_uint32(unsigned int, downscaler_enabled);	/* acc id 3, set if process required */
		aligned_uint32(unsigned int, awb_enabled);			/* acc id 4, set if process required */
		aligned_uint32(unsigned int, af_enabled);			/* acc id 5, set if process required */
		aligned_uint32(unsigned int, ae_enabled);			/* acc id 6, set if process required */
	} acc_enabled_set;
	aligned_uint32(unsigned int, isa_input_pin_index);
};

 /**
 * struct ia_css_isys_stream_cfg_data
 * ISYS stream configuration data structure
 * @isl_enabled: indicates whether stream requores ISL
 * @src: Stream source index e.g. MIPI_generator_0, CSI2-rx_1
 * @vc: MIPI Virtual Channel (up to 4 virtual per physical channel)
 * @the rest: input/output pin descriptors
 */
struct ia_css_isys_stream_cfg_data_comm {
	aligned_struct(struct ia_css_isys_input_pin_info_comm, input_pins[MAX_IPINS]);
	aligned_struct(struct ia_css_isys_output_pin_info_comm, output_pins[MAX_OPINS]);
	union {
		aligned_struct(struct ia_css_isys_isl_cfg_comm, isl_cfg);
		aligned_struct(struct ia_css_isys_isa_cfg_comm, isa_cfg);
	} isl_isa_cfg;
	aligned_enum(enum ia_css_isys_isl_use, isl_use);
	aligned_enum(enum ia_css_isys_stream_source, src);
	aligned_enum(enum ia_css_isys_mipi_vc, vc);
	aligned_uint32(unsigned int, nof_input_pins);
	aligned_uint32(unsigned int, nof_output_pins);
	//int ALIGN;
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

struct ia_css_isys_frame_buff_set_comm {
	aligned_struct(struct ia_css_isys_output_pin_comm, output_pins[MAX_OPINS]);
	struct {
		aligned_struct(struct ia_css_isys_input_pin_comm, blc_param);
		aligned_struct(struct ia_css_isys_input_pin_comm, lsc_param);
		aligned_struct(struct ia_css_isys_input_pin_comm, dpc_param);
	} input_pin_set;
	aligned_uint32(unsigned int, send_irq_sof);
	aligned_uint32(unsigned int, send_irq_eof);
};

/**
 * struct ia_css_isys_resp_info
 * @type
 * @error
 * @timestamp
 * @stream_handle
 * @pin: this member is only valid for pin event related responses
 */
struct ia_css_isys_resp_info_comm {
	aligned_enum(enum ia_css_isys_resp_type, type);
	aligned_uint32(unsigned int, timestamp[2]);
	aligned_uint32(unsigned int, stream_handle);
	aligned_int32(int, error);
	//int ALIGN;
	union {
		aligned_struct(struct ia_css_isys_output_pin_comm, pin);
		aligned_uint64(ia_css_return_token, buf_id);
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
struct ia_css_isys_buffer_partition_comm {
	aligned_uint32(unsigned int, block_size[STREAM_ID_MAX]);
	aligned_uint32(unsigned int, nof_blocks);
	//int ALIGN;
};


/* From here on type defines not coming from the ISYSAPI interface */

/**
 * struct resp_queue_token
 */
struct resp_queue_token {
    aligned_struct(struct ia_css_isys_resp_info_comm, resp_info);
};


/**
 * struct send_queue_token
 */
struct send_queue_token {
   aligned_enum(enum ia_css_isys_send_type, send_type);
   aligned_uint32(ia_css_input_buffer_css_address, payload);
   aligned_uint64(ia_css_return_token, buf_handle);
};


/**
 * struct is_css_isys_fw_config
 */
struct is_css_isys_fw_config {
	aligned_struct(struct ia_css_isys_buffer_partition_comm, mipi);
	aligned_struct(struct ia_css_isys_buffer_partition_comm, pixel);
};


#endif /*__IA_CSS_ISYS_FW_BRIDGED_TYPES_H__*/
