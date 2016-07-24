
#include "ia_css_isys_stream_device.h"
#include "error_support.h"

#ifdef RUN_INTEGRATION

#include "vied_nci_ibufctrl_config.h"
#include "vied_nci_ibufctrl_config_types.h"
#include "vied_nci_ibufctrl_config_types_dev.h"
#include "vied_nci_ibufctrl_command.h"
#include "vied_nci_ibufctrl_command_types.h"


#include "vied_nci_str2mmio.h"
#include "vied_nci_str2mmio_types.h"

#include "vied_nci_dma.h"

#include "isa_ctrl.h"

#define UNIT_ELEM_SIZE_BITS  	8	// const(?)
#define ISL_UNIT_SIZE_ELEMS 	64	// const(?)

#define NOF_FRAMES				199
#define DTYPE_RAW8_BITS         8

#define ISL_UNIT_HEIGHT_ISA_ON		2		/* 2D */
#define ISL_UNIT_HEIGHT_ISA_OFF		1		/* 1D */

#include "vied_nci_isldevice_cio2stream.h"
#include "vied_nci_isldevice_mipi_backend.h"
#include "vied_nci_isldevice_pixel_formatter.h"

#include "system_defs.h"

#define ceil_div(a, b) (((a)+(b)-1)/(b))

#define  STREAM_ID_MAX 6

#define NBITS2(n) (((n)&2)?1:0)
#define NBITS4(n) (((n)&(0xC))?(2+NBITS2((n)>>2)):(NBITS2(n)))
#define NBITS8(n) (((n)&0xF0)?(4+NBITS4((n)>>4)):(NBITS4(n)))
#define NBITS16(n) (((n)&0xFF00)?(8+NBITS8((n)>>8)):(NBITS8(n)))
#define NBITS32(n) (((n)&0xFFFF0000)?(16+NBITS16((n)>>16)):(NBITS16(n)))
#define NBITS(n) ((n)==0?0:NBITS32(n))

#define ALIGN_POW2(n)	(((n)<=2)?(n):(1<<(NBITS((n)-1)+1)))

#define  EXT1_DMA_USED_CHANNELS 2
#define  ISL_DMA_USED_CHANNELS	1

extern int cmds_sent[STREAM_ID_MAX];

/* IBUF_CTRL devices */
static	vied_nci_ibufctrl_device_t * ibufctrl_mg_dev_t = 0;
static	vied_nci_ibufctrl_device_t * ibufctrl_isl_dev_t = 0;

/* STR2MMIO devices */
static vied_nci_str2mmio_device_t * str2mmio_mg_dev_t = 0;
static vied_nci_str2mmio_device_t * str2mmio_isl_dev_t = 0;

#endif

#include <hive/support.h>
extern int AT(0) __dummy;

struct istream_ctx istr_ctx[STREAM_ID_MAX];

/*
 * istream_configure
 */
enum istream_err istream_configure(
	struct istream_ctx **ppistr_ctx,
	const unsigned int stream_id,
	const struct ia_css_isys_stream_cfg_data_comm *pstream_dev_cfg
)
{
	/* TODO:
		From pstream_dev_cfg structure create/trigger the data
		needed for the NCI configuration functions.
		*/
	verifret(ppistr_ctx != NULL, IA_CSS_ISYS_STREAM_CONFIGURE_STREAM_FAIL);
	verifret(pstream_dev_cfg != NULL, IA_CSS_ISYS_STREAM_CONFIGURE_STREAM_FAIL);

	*ppistr_ctx = &istr_ctx[stream_id];						/* export the pointer to the istream_ctx */
	istr_ctx[stream_id].stream_id = stream_id;				/* set the stream_id in the istream_ctx */
	istr_ctx[stream_id].stream_cfg_data = *pstream_dev_cfg;	/* set the pstream_dev_cfg in the istream_ctx */

	{
#ifdef RUN_INTEGRATION

		int return_value = 0;

		unsigned int isl_unit_height = (((*ppistr_ctx)->stream_cfg_data.isl_use == IA_CSS_ISYS_USE_SINGLE_ISA) ? ISL_UNIT_HEIGHT_ISA_ON : ISL_UNIT_HEIGHT_ISA_OFF);

		unsigned int pix_p_line = (*ppistr_ctx)->stream_cfg_data.output_pins[0].output_res.width;
		unsigned int nr_lines = (*ppistr_ctx)->stream_cfg_data.output_pins[0].output_res.height;

		unsigned buf_type = 6; // log2(XMEM_WIDTH)-log2(8)
		unsigned int pixel_bits = DTYPE_RAW8_BITS;
		unsigned int pix_bits_p_line = pix_p_line * pixel_bits;
		unsigned int mb_bits_p_line = pix_p_line * pixel_bits + 64; // plus packet header

		unsigned int MB_UNIT_SIZE_ELEMS = (ALIGN_POW2((mb_bits_p_line/UNIT_ELEM_SIZE_BITS))) / 2;
		/* PB_UNIT_SIZE_ELEMS == MB_UNIT_SIZE_ELEMS previously, it generally does not have any limitations,
		 * it is used only for the final DMA: Pixel SRAM --> DDR. 64 (word size) eliminates all the warnings
		 * about reading from uninitialised memory (DMA EXT1) and does not require any buffer overallocation,
		 * but maybe it is not so good for performance (best is to match the DMA burst).
		 * Unfortunately the HW bug in STR2MMIO (A0) does not allow us this freedom for the MIPI buffer, where
		 * we are forced to use 2 units to cover the width of the image. Maybe there is a way but I could not
		 * find it (the DMA EXT1 seems to continue sending the previous number of units, which means much less
		 * data).
		 */
		unsigned int PB_UNIT_SIZE_ELEMS = 64; //MB_UNIT_SIZE_ELEMS;

		//units and buffers
		unsigned int mb_unit_size_words = ceil_div((MB_UNIT_SIZE_ELEMS * UNIT_ELEM_SIZE_BITS), XMEM_WIDTH);
		unsigned int mb_unit_size_bits  = mb_unit_size_words * XMEM_WIDTH;
		unsigned int isl_unit_size_words  = ceil_div((ISL_UNIT_SIZE_ELEMS * UNIT_ELEM_SIZE_BITS), ISL_INPUT_WORD_SIZE);
		unsigned int isl_unit_size_bits   = isl_unit_size_words * ISL_INPUT_WORD_SIZE;
		unsigned int pb_unit_size_words   = ceil_div((PB_UNIT_SIZE_ELEMS * UNIT_ELEM_SIZE_BITS), XMEM_WIDTH);
		unsigned int pb_unit_size_bits    = pb_unit_size_words * XMEM_WIDTH;

		//image line sizes
		unsigned int pix_byte_p_line   = ceil_div(pix_bits_p_line, 8);
		unsigned int pix_words_p_line  = ceil_div(pix_bits_p_line, XMEM_WIDTH);
		unsigned int pix_units_p_line  = ceil_div(pix_bits_p_line, pb_unit_size_bits);
		//unsigned int mipi_byte_p_line  = ceil_div(mb_bits_p_line, 8);
		unsigned int mipi_words_p_line = ceil_div(mb_bits_p_line, XMEM_WIDTH);
		unsigned int mipi_units_p_line = ceil_div(mb_bits_p_line, mb_unit_size_bits);
		//unsigned int isl_words_p_line  = ceil_div(mb_bits_p_line, ISL_INPUT_WORD_SIZE);
		unsigned int isl_units_p_line  = ceil_div(mb_bits_p_line, isl_unit_size_bits);

		// input buffer size
		unsigned int mipi_buffer_start_addr    = 0;
		unsigned int mipi_buffer_lines         = 2;
		unsigned int mipi_buffer_units_p_line  = mipi_units_p_line; // in this case an entire image line is put in the buffer, shouldn't have to be always the case
		unsigned int mipi_buffer_stride_words  = mipi_buffer_units_p_line * mb_unit_size_words;
		unsigned int mipi_buffer_words         = mipi_buffer_lines * mipi_buffer_stride_words;
		unsigned int mipi_buffer_units         = mipi_buffer_lines * mipi_buffer_units_p_line;

		// pixel buffer size
		unsigned int pix_buffer_start_addr    = 0;
		unsigned int pix_buffer_lines         = 2;
		unsigned int pix_buffer_units_p_line  = pix_units_p_line; // in this case an entire image line is put in the buffer, shouldn't have to be always the case
		unsigned int pix_buffer_stride_words  = pix_buffer_units_p_line * pb_unit_size_words;
		unsigned int pix_buffer_words         = pix_buffer_lines * pix_buffer_stride_words;
		unsigned int pix_buffer_units         = pix_buffer_lines * pix_buffer_units_p_line;

		//struct vied_nci_dma_chan_res chan_res;
		struct vied_nci_dma_chan_res chan_res[2];
		struct vied_nci_dma_chan_res isl_chan_res;

		/* Configure the EXT1 DMA */
		{
			int out_pin;

			struct vied_nci_dma_chan_t dma_chan_memory[EXT1_DMA_USED_CHANNELS] = {0};
			struct vied_nci_dma_transfer_config_t transfer_config = {0};
			struct vied_nci_dma_terminal_config_t terminal_config = {0};
			struct vied_nci_dma_dev_t *dma_dev;
			struct vied_nci_dma_chan_t *dma_chan;

			struct vied_nci_dma_dev_config_t dev_config = {0};

			transfer_config.unit_desc.unit_height = 1 - 1; // 1D

			// Configure Channel descriptor contents
			transfer_config.channel_desc.element_extend_mode = 0;// zero extend if needed
			transfer_config.channel_desc.element_init_data = 0;
			transfer_config.channel_desc.padding_mode =  0; // 0 constant padding
			transfer_config.channel_desc.sampling_setup = 0; // subsampling of 1
			transfer_config.channel_desc.global_set_id = 0; // == dev.group_id_idx
			transfer_config.channel_desc.ack_mode = 0x0; // 0: passive mode, 1: active mode (if channel bank is cached mode)
			transfer_config.channel_desc.ack_data = 0;

			// Configure Span descriptor contents
			terminal_config.span_desc[0].unit_location = 0;
			terminal_config.span_desc[0].span_row = 0;
			terminal_config.span_desc[0].span_column = 0;
			terminal_config.span_desc[0].span_height = ceil_div(mipi_buffer_lines, (transfer_config.unit_desc.unit_height + 1)) - 1;
			terminal_config.span_desc[0].span_mode = 0x1; //coordinate based addressing

			terminal_config.span_desc[1].unit_location = 0;
			terminal_config.span_desc[1].span_row = 0;
			terminal_config.span_desc[1].span_column = 0;
			terminal_config.span_desc[1].span_height = ceil_div(nr_lines, (transfer_config.unit_desc.unit_height + 1)) - 1;
			terminal_config.span_desc[1].span_mode = 0x1; //coordinate based addressing

			terminal_config.terminal_desc[0].element_setup = 0; // 8 bit per pixel
			terminal_config.terminal_desc[0].cio_info_setup = 0x00;  // 0x400 ?
			terminal_config.terminal_desc[0].port_mode = 0x00; // 0: Treats the terminal as a regular memory mapped address space resulting in incrementing addresses while transferring elements contained in a unit.
																//1: Treats the terminal as an I/O port (e.g. FIFO), resulting in a fixed address (corresponding to the unit location) while transferring elements contained in a unit.

			terminal_config.terminal_desc[1].region_origin = 0;

			terminal_config.terminal_desc[1].element_setup = 0; // 8 bit per pixel
			terminal_config.terminal_desc[1].cio_info_setup = 0;
			terminal_config.terminal_desc[1].port_mode = 0x00;

			// Open a DMA device
			dma_dev = vied_nci_dma_open(VIED_NCI_DMA_EXT1);

			// Non-cached mode
			dev_config.bank_mode = VIED_NCI_DMA_BANK_MODE_NON_CACHED;
			//ISYS stream channels require dedicated dma channels/requesters
			dev_config.req_type = VIED_NCI_DMA_DEDICATED_REQUESTOR;
			dev_config.chan_handle_addr = dma_chan_memory;
			dev_config.num_of_channels = EXT1_DMA_USED_CHANNELS;

			// Configure DMA
			vied_nci_dma_configure(dma_dev, &dev_config);

			terminal_config.span_desc[0].x_coordinate = 0;
			terminal_config.span_desc[0].y_coordinate = 0;
			terminal_config.span_desc[1].x_coordinate = 0;
			terminal_config.span_desc[1].y_coordinate = 0;

			for (out_pin = 0; out_pin < (*ppistr_ctx)->stream_cfg_data.nof_output_pins ; out_pin++)
			{
				if ((*ppistr_ctx)->stream_cfg_data.output_pins[out_pin].pt == IA_CSS_ISYS_PIN_TYPE_MIPI)
				{
					transfer_config.unit_desc.unit_width =  MB_UNIT_SIZE_ELEMS - 1;

					terminal_config.span_desc[0].span_width = mipi_buffer_units_p_line - 1;
					terminal_config.span_desc[1].span_width = mipi_units_p_line - 1;

					terminal_config.terminal_desc[0].region_origin = DMA_EXT1_MIPI_BUF_ADDR + mipi_buffer_start_addr; //address of the mipi_buffer seen from the dma
					terminal_config.terminal_desc[0].region_width = (MB_UNIT_SIZE_ELEMS * mipi_buffer_units_p_line) - 1;
					terminal_config.terminal_desc[0].region_stride =  mipi_buffer_stride_words << buf_type;

					terminal_config.terminal_desc[1].region_width = ceil_div(mb_bits_p_line, pixel_bits) - 1; // width in number of elem;
					terminal_config.terminal_desc[1].region_stride = mipi_words_p_line << buf_type;

					// Configure Request register
					transfer_config.channel_desc.ack_addr = vied_nci_ibufctrl_reg_addr_with_base(vied_nci_ibufctrl_open_dev(BXT_IBUFCTRL_MG_DEV)->base_addr, 0, _IBC_2600_GROUP_PROC_CMD, _IBC_2600_PROC_CMD_DEST_ACK(out_pin));
				}
				else if ((*ppistr_ctx)->stream_cfg_data.output_pins[out_pin].pt == IA_CSS_ISYS_PIN_TYPE_RAW_NS)
				{
					transfer_config.unit_desc.unit_width =  PB_UNIT_SIZE_ELEMS - 1;

					terminal_config.span_desc[0].span_width = pix_buffer_units_p_line - 1;
					/* When ISA is in use, 2 lines are merged, so the number of units per merged line is not double, is double data aligned to the unit size divided to the unit size */
					terminal_config.span_desc[1].span_width = ceil_div(isl_unit_height * pix_bits_p_line, pb_unit_size_bits) - 1;

					terminal_config.terminal_desc[0].region_origin = DMA_EXT1_PIX_BUF_ADDR + pix_buffer_start_addr; //address of the mipi_buffer seen from the dma
					terminal_config.terminal_desc[0].region_width = (PB_UNIT_SIZE_ELEMS *  pix_buffer_units_p_line) - 1;
					terminal_config.terminal_desc[0].region_stride =  pix_buffer_stride_words << buf_type;

					/* When ISA is in use, 2 lines are merged, so the amount of data per merged line is double, same with the stride */
					terminal_config.terminal_desc[1].region_width = isl_unit_height * ceil_div(pix_bits_p_line, pixel_bits) - 1; // width in number of elem;
					terminal_config.terminal_desc[1].region_stride = isl_unit_height * (pix_words_p_line << buf_type);

					// Configure Request register
					transfer_config.channel_desc.ack_addr = vied_nci_ibufctrl_reg_addr_with_base(vied_nci_ibufctrl_open_dev(BXT_IBUFCTRL_ISL_DEV)->base_addr, 0, _IBC_2600_GROUP_PROC_CMD, _IBC_2600_PROC_CMD_DEST_ACK(out_pin));
				}
				// Open a DMA channel
				dma_chan = vied_nci_dma_chan_open(VIED_NCI_DMA_EXT1, out_pin, &transfer_config);
				if (dma_chan == NULL) {
					OP___dump(__LINE__, 0xa5);
				}

				// Configure the DMA channel
				vied_nci_dma_chan_configure(dma_chan, &terminal_config);

				vied_nci_dma_get_chan_res(dma_chan, &chan_res[out_pin]);
			}
		}

		/* Configure the ISA DMA */
		if ((*ppistr_ctx)->stream_cfg_data.isl_use != IA_CSS_ISYS_USE_NO_ISL_NO_ISA)
		{
			struct vied_nci_dma_chan_t isl_dma_chan_memory[ISL_DMA_USED_CHANNELS];
			struct vied_nci_dma_transfer_config_t isl_transfer_config;
			struct vied_nci_dma_terminal_config_t isl_terminal_config;
			struct vied_nci_dma_dev_t *isl_dma_dev;
			struct vied_nci_dma_chan_t *isl_dma_chan;

			struct vied_nci_dma_dev_config_t isl_dev_config;

			isl_transfer_config.unit_desc.unit_width =  ISL_UNIT_SIZE_ELEMS - 1;
			isl_transfer_config.unit_desc.unit_height = isl_unit_height - 1; // 1D

			// Configure Channel descriptor contents
			isl_transfer_config.channel_desc.element_extend_mode = 0; // zero extend if needed
			isl_transfer_config.channel_desc.element_init_data = 0;
			isl_transfer_config.channel_desc.padding_mode =  0; // 0 constant padding
			isl_transfer_config.channel_desc.sampling_setup = 0; // subsampling of 1
			isl_transfer_config.channel_desc.global_set_id = 0; // == dev.group_id_idx
			isl_transfer_config.channel_desc.ack_mode = 0x0; // 0: passive mode, 1: active mode (if channel bank is cached mode)
			isl_transfer_config.channel_desc.ack_data = 0;

			// Configure Span descriptor contents
			isl_terminal_config.span_desc[0].unit_location = 0;
			isl_terminal_config.span_desc[0].span_row = 0;
			isl_terminal_config.span_desc[0].span_column = 0;
			isl_terminal_config.span_desc[0].span_width = isl_units_p_line - 1;
			isl_terminal_config.span_desc[0].span_height = ceil_div(mipi_buffer_lines, (isl_transfer_config.unit_desc.unit_height + 1)) - 1;
			isl_terminal_config.span_desc[0].span_mode = 0x1; //coordinate based addressing

			isl_terminal_config.span_desc[1].unit_location = 0;
			isl_terminal_config.span_desc[1].span_row = 0;
			isl_terminal_config.span_desc[1].span_column = 0;
			isl_terminal_config.span_desc[1].span_width = 1 - 1;
			isl_terminal_config.span_desc[1].span_height = 1 - 1;
			isl_terminal_config.span_desc[1].span_mode = 0x1; //coordinate based addressing

			isl_terminal_config.terminal_desc[0].region_origin = DMA_ISL_MIPI_BUF_ADDR + mipi_buffer_start_addr; //address of the mipi_buffer seen from the dma
			isl_terminal_config.terminal_desc[0].region_width = (MB_UNIT_SIZE_ELEMS *  mipi_units_p_line) - 1;
			isl_terminal_config.terminal_desc[0].region_stride =  mipi_buffer_stride_words << buf_type;
			isl_terminal_config.terminal_desc[0].element_setup = 0; // 8 bit per pixel
			isl_terminal_config.terminal_desc[0].cio_info_setup = 0x00;  // 0x400 ? //?
			isl_terminal_config.terminal_desc[0].port_mode = 0x00;  // 0: Treats the terminal as a regular memory mapped address space resulting in incrementing addresses while transferring elements contained in a unit.
																//1: Treats the terminal as an I/O port (e.g. FIFO), resulting in a fixed address (corresponding to the unit location) while transferring elements contained in a unit.

			isl_terminal_config.terminal_desc[1].region_origin = 0;
			isl_terminal_config.terminal_desc[1].region_width = (isl_units_p_line * ISL_UNIT_SIZE_ELEMS) - 1;
			isl_terminal_config.terminal_desc[1].region_stride = 0x10000;
			isl_terminal_config.terminal_desc[1].element_setup = 0; // 8 bit per pixel
			isl_terminal_config.terminal_desc[1].cio_info_setup = 0;
			isl_terminal_config.terminal_desc[1].port_mode = 0x00;

			// Configure Request register
			isl_transfer_config.channel_desc.ack_addr = vied_nci_ibufctrl_reg_addr_with_base(vied_nci_ibufctrl_open_dev(BXT_IBUFCTRL_ISL_DEV)->ibuf_from_str2mmio_dev, 0, _IBC_2600_GROUP_FEEDER, _IBC_2600_FEED_DMA_ACK);

			// Open a DMA device
			isl_dma_dev = vied_nci_dma_open(VIED_NCI_ISA_DMA);

			// Non-cached mode
			isl_dev_config.bank_mode = VIED_NCI_DMA_BANK_MODE_NON_CACHED;
			//ISYS stream channels require dedicated dma channels/requesters
			isl_dev_config.req_type = VIED_NCI_DMA_DEDICATED_REQUESTOR;
			isl_dev_config.chan_handle_addr = isl_dma_chan_memory;
			isl_dev_config.num_of_channels = ISL_DMA_USED_CHANNELS;

			// Configure DMA
			vied_nci_dma_configure(isl_dma_dev, &isl_dev_config);

			isl_terminal_config.span_desc[0].x_coordinate = 0;
			isl_terminal_config.span_desc[0].y_coordinate = 0;
			isl_terminal_config.span_desc[1].x_coordinate = 0;
			isl_terminal_config.span_desc[1].y_coordinate = 0;

			// Open a DMA channel
			isl_dma_chan = vied_nci_dma_chan_open(VIED_NCI_ISA_DMA, 0, &isl_transfer_config);
			if (isl_dma_chan == NULL) {
				OP___dump(__LINE__, 0xa5);
			}

			// Configure the DMA channel
			vied_nci_dma_chan_configure(isl_dma_chan, &isl_terminal_config);

			vied_nci_dma_get_chan_res(isl_dma_chan, &isl_chan_res);
		}

		/* Configure MG_IBC*/
		{
			vied_nci_ibufctrl_ibuf_s ibuf_mg = {0};

			int sid;
			int out_pin;

			ibufctrl_mg_dev_t = vied_nci_ibufctrl_open_dev(BXT_IBUFCTRL_MG_DEV);

			for (sid = 0; sid < ibufctrl_mg_dev_t->nr_sids/*IBC2600_MG_NR_SIDS*/; sid ++)
			{
				ibuf_mg.proc[sid].nr_dests = 2;
			}
			return_value = vied_nci_ibufctrl_setup_ibuf(ibufctrl_mg_dev_t, &ibuf_mg);
			return_value = vied_nci_ibufctrl_config_ibuf(ibufctrl_mg_dev_t, &ibuf_mg);

			ibuf_mg.proc[0].cmd.sidpid = INFO_TO_EVENT_SIDPID(IA_CSS_ISYS_IBUFCTRL_MG,0,0,stream_id);
			ibuf_mg.proc[0].cmd.ack_addr       = IBC2600_MG_SP_QUE_ADDR >> 2;
			ibuf_mg.proc[0].cfg.str2mmio_addr  =
				vied_nci_str2mmio_reg_addr_with_base(
					vied_nci_str2mmio_open_dev(BXT_STR2MMIO_MG_DEV0)->type,
					vied_nci_str2mmio_open_dev(BXT_STR2MMIO_MG_DEV0)->str2mmio_from_ibuf_dev,
					0,
					_STREAM2MMIO_COMMAND_REG_ID) >> 2; // sid 0
			ibuf_mg.proc[0].cfg.store_cmd      = 0; // store words
			ibuf_mg.proc[0].cfg.items_p_unit   = mb_unit_size_words; // amount of words in a unit
			ibuf_mg.proc[0].cfg.units_p_line   = mipi_units_p_line;
			ibuf_mg.proc[0].cfg.lines_p_frame  = (*ppistr_ctx)->stream_cfg_data.output_pins[0].output_res.height;
			ibuf_mg.proc[0].cfg.units_p_ibuf   = mipi_buffer_units;
			ibuf_mg.proc[0].cfg.sync_frame     = 1;
			for (out_pin = 0; out_pin < (*ppistr_ctx)->stream_cfg_data.nof_output_pins ; out_pin++)
			{
				ibuf_mg.proc[0].cfg.dest_en[out_pin] = 1;

				if ((*ppistr_ctx)->stream_cfg_data.output_pins[out_pin].pt == IA_CSS_ISYS_PIN_TYPE_MIPI)
				{
					ibuf_mg.proc[0].cfg.dest_cfg[out_pin]->req_addr =  chan_res[out_pin].requestor_addr >> 2;
					ibuf_mg.proc[0].cfg.dest_cfg[out_pin]->channel_addr = chan_res[out_pin].chan_addr >> 2; //0x00024000; get_dma_channel_addr(&dma4_ibc, 0, 0) >> 2;
					ibuf_mg.proc[0].cfg.dest_cfg[out_pin]->span_a_addr = chan_res[out_pin].span_A_addr >>2; //0x00024800;get_dma_span_addr(&dma4_ibc, 0, 0) >> 2;
					ibuf_mg.proc[0].cfg.dest_cfg[out_pin]->span_b_addr = chan_res[out_pin].span_B_addr >>2; //0x00024810;//get_dma_span_addr(&dma4_ibc, 1, 0) >> 2;
					ibuf_mg.proc[0].cfg.dest_cfg[out_pin]->terminal_b_addr = chan_res[out_pin].terminal_B_addr >>2; // 0x00024c10;//get_dma_terminal_addr(&dma4_ibc, 1, 0) >> 2;
					ibuf_mg.proc[0].cfg.dest_cfg[out_pin]->dest_mode.bits.is_feeder   = 0;
					ibuf_mg.proc[0].cfg.dest_cfg[out_pin]->st_addr          = 0; // not used, online mode only for now
				}
				else if ((*ppistr_ctx)->stream_cfg_data.output_pins[out_pin].pt == IA_CSS_ISYS_PIN_TYPE_RAW_NS)
				{
					ibuf_mg.proc[0].cfg.dest_cfg[out_pin]->feed_addr =  vied_nci_ibufctrl_reg_addr_with_base(vied_nci_ibufctrl_open_dev(BXT_IBUFCTRL_ISL_DEV)->base_addr, 0, _IBC_2600_GROUP_FEEDER, _IBC_2600_FEED_CMD_CMD)>>2;
					ibuf_mg.proc[0].cfg.dest_cfg[out_pin]->req_addr =  isl_chan_res.requestor_addr >> 2; // 0x0002c040  get_dma_request_addr(&dma4_isl_ibc_mg, 0, 0) >> 2;
					ibuf_mg.proc[0].cfg.dest_cfg[out_pin]->channel_addr = isl_chan_res.chan_addr >> 2; // 0x0002c000 get_dma_channel_addr(&dma4_isl_ibc_mg, 0, 0) >> 2;
					ibuf_mg.proc[0].cfg.dest_cfg[out_pin]->span_a_addr = isl_chan_res.span_A_addr >>2; //0x0002c100 get_dma_span_addr(&dma4_isl_ibc_mg, 0, 0) >> 2;
					ibuf_mg.proc[0].cfg.dest_cfg[out_pin]->span_b_addr = isl_chan_res.span_B_addr >>2; //0x0002c110 get_dma_span_addr(&dma4_isl_ibc_mg, 1, 0) >> 2;
					ibuf_mg.proc[0].cfg.dest_cfg[out_pin]->terminal_b_addr = isl_chan_res.terminal_B_addr >>2; // 0x0002c190;//get_dma_terminal_addr(&dma4_isl_ibc_mg, 1, 0) >> 2;
					ibuf_mg.proc[0].cfg.dest_cfg[out_pin]->dest_mode.bits.is_feeder   = 1;
					ibuf_mg.proc[0].cfg.dest_cfg[out_pin]->st_addr          = 0x1cc0000; // not used, online mode only for now
				}

				ibuf_mg.proc[0].cfg.dest_cfg[out_pin]->dest_mode.bits.config_dma  = 1;
				ibuf_mg.proc[0].cfg.dest_cfg[out_pin]->dest_mode.bits.iwake_en    = 0;
				ibuf_mg.proc[0].cfg.dest_cfg[out_pin]->dest_mode.bits.others      = 0;
				ibuf_mg.proc[0].cfg.dest_cfg[out_pin]->num_items        = 0; // not used
				ibuf_mg.proc[0].cfg.dest_cfg[out_pin]->iwake_threshold  = 0; // not used
			}

			return_value = vied_nci_ibufctrl_config_proc(ibufctrl_mg_dev_t, &ibuf_mg, 0);
		}

		/* Configure ISL_IBC*/
		if ((*ppistr_ctx)->stream_cfg_data.isl_use != IA_CSS_ISYS_USE_NO_ISL_NO_ISA)
		{
			unsigned int out_pin;
			vied_nci_ibufctrl_ibuf_s ibuf_isl = {0};
			int sid;
			ibufctrl_isl_dev_t = vied_nci_ibufctrl_open_dev(BXT_IBUFCTRL_ISL_DEV);

			for (sid = 0; sid < ibufctrl_isl_dev_t->nr_sids/*IBC2600_ISL_NR_SIDS*/; sid ++)
			{
				ibuf_isl.proc[sid].nr_dests = 1;
			}
			return_value = vied_nci_ibufctrl_setup_ibuf(ibufctrl_isl_dev_t, &ibuf_isl);
			return_value = vied_nci_ibufctrl_config_ibuf(ibufctrl_isl_dev_t, &ibuf_isl);

			ibuf_isl.proc[0].cmd.sidpid = INFO_TO_EVENT_SIDPID(IA_CSS_ISYS_IBUFCTRL_ISL,0,0,stream_id);
			ibuf_isl.proc[0].cmd.ack_addr       = IBC2600_ISL_SP_QUE_ADDR >> 2;
			ibuf_isl.proc[0].cfg.str2mmio_addr  =
				vied_nci_str2mmio_reg_addr_with_base(
					vied_nci_str2mmio_open_dev(BXT_STR2MMIO_PIXEL_DEV0)->type,
					vied_nci_str2mmio_open_dev(BXT_STR2MMIO_PIXEL_DEV0)->str2mmio_from_ibuf_dev,
					0,
					_STREAM2MMIO_COMMAND_REG_ID) >> 2; // sid 0
			ibuf_isl.proc[0].cfg.store_cmd      = 0; // store words
			ibuf_isl.proc[0].cfg.items_p_unit   = pb_unit_size_words; // amount of words per unit for the pixel buffer
			/* For the ISA case, the number of the units that will come through (based on the rest of the configuration )
			 * is not always the same as without using ISA, because 2 pixel lines are merged into 1 and this is aligned
			 * to the unit size. Currently we use 2 pixel buffer units per line (exception the case width==2^N where it is
			 * 1 for the pixel buffer, 2 for the mipi buffer), and we have nof_lines==image_height. When using ISA
			 * the 2 lines are merged into 1, which means 3 or 4 units per line, and we have nof_merged_lines==image_height/2.
			 * When the merged lines have 4 units per line, then the HW configuration is coincidentally equivallent since the
			 * total number of units for the frame is calculated by the HW as (2)*(image_height) instead of the correct
			 * (4)*(image_height/2). However, when the merged lines have 3 units per line there is a problem since
			 * (2)*(image_height)!=(3)*(image_height/2), meaning that we configure it to expect more data than it should.
			 */
			ibuf_isl.proc[0].cfg.units_p_line   = ceil_div(isl_unit_height * pix_bits_p_line, pb_unit_size_bits);
			ibuf_isl.proc[0].cfg.lines_p_frame  = (*ppistr_ctx)->stream_cfg_data.output_pins[0].output_res.height / isl_unit_height;
			ibuf_isl.proc[0].cfg.units_p_ibuf   = pix_buffer_units;
			ibuf_isl.proc[0].cfg.sync_frame     = 1;
			ibuf_isl.proc[0].cfg.dest_en[0]     = 1;
			ibuf_isl.proc[0].cfg.dest_en[1]     = 0;

			for (out_pin = 0; out_pin < (*ppistr_ctx)->stream_cfg_data.nof_output_pins ; out_pin++)
			{
				if ((*ppistr_ctx)->stream_cfg_data.output_pins[out_pin].pt == IA_CSS_ISYS_PIN_TYPE_RAW_NS)
				{
					break;
				}
			}

			ibuf_isl.dest_cfg[0].req_addr =  (IBC2600_IS_A_EQC_ADDR + chan_res[out_pin].requestor_addr) >> 2; //0x0002c200; get_dma_request_addr(&dma4_ext1_ibc_isl, 0, 0) >> 2;
			ibuf_isl.dest_cfg[0].channel_addr = (IBC2600_IS_A_EQC_ADDR + chan_res[out_pin].chan_addr) >> 2; //0x0002c000; get_dma_channel_addr(&dma4_ext1_ibc_isl, 0, 0) >> 2;
			ibuf_isl.dest_cfg[0].span_a_addr = (IBC2600_IS_A_EQC_ADDR + chan_res[out_pin].span_A_addr) >>2; //0x0002c800;get_dma_span_addr(&dma4_ext1_ibc_isl, 0, 0) >> 2;
			ibuf_isl.dest_cfg[0].span_b_addr = (IBC2600_IS_A_EQC_ADDR + chan_res[out_pin].span_B_addr) >>2; //0x0002c810;//get_dma_span_addr(&dma4_ext1_ibc_isl, 1, 0) >> 2;
			ibuf_isl.dest_cfg[0].terminal_b_addr = IBC2600_IS_A_EQC_ADDR + chan_res[out_pin].terminal_B_addr >>2; // 0x0002cc10;//get_dma_terminal_addr(&dma4_ext1_ibc_isl, 1, 0) >> 2;
			ibuf_isl.proc[0].cfg.dest_cfg[0]->dest_mode.bits.is_feeder   = 0;
			ibuf_isl.proc[0].cfg.dest_cfg[0]->dest_mode.bits.config_dma  = 1;
			ibuf_isl.proc[0].cfg.dest_cfg[0]->dest_mode.bits.iwake_en    = 0;
			ibuf_isl.proc[0].cfg.dest_cfg[0]->dest_mode.bits.others      = 0;
			ibuf_isl.proc[0].cfg.dest_cfg[0]->st_addr          = 0; // not used, online mode only for now
			ibuf_isl.proc[0].cfg.dest_cfg[0]->num_items        = 0; // not used
			ibuf_isl.proc[0].cfg.dest_cfg[0]->iwake_threshold  = 0; // not used*/

			return_value = vied_nci_ibufctrl_config_proc(ibufctrl_isl_dev_t, &ibuf_isl, 0);

		    ibuf_isl.feeder_cfg[0].ack_addr =  vied_nci_ibufctrl_reg_addr_with_base(IBC2600_IS_A_EQC_ADDR + vied_nci_ibufctrl_open_dev(BXT_IBUFCTRL_MG_DEV)->base_addr, 0, _IBC_2600_GROUP_PROC_CMD, _IBC_2600_PROC_CMD_DEST_ACK(0))>>2;//ibc2600_reg_addr_with_base(IBC2600_ISL_IBC2600_MG_ADDR, 0, _IBC_2600_GROUP_PROC_CMD, _IBC_2600_PROC_CMD_DEST_ACK(0))>>2;
			/* TODO: Remove the dependency from the specific path - hardcoded define value */
			ibuf_isl.feeder_cfg[0].req_addr = (isl_chan_res.requestor_addr - SP_AB_IS_A_ADDR) >> 2; //0x00004040 get_dma_request_addr(&dma4_isl_ibc_isl, 0, 0) >> 2;
			ibuf_isl.feeder_cfg[0].channel_addr = (isl_chan_res.chan_addr - SP_AB_IS_A_ADDR) >> 2; //0x00004000 get_dma_channel_addr(&dma4_isl_ibc_isl, 0, 0) >> 2;
			ibuf_isl.feeder_cfg[0].units_out_p_in = ceil_div(mb_unit_size_bits, isl_unit_size_bits);
			if(mipi_buffer_units_p_line==1) // if only one input unit is transferred, only last_units_out is used:
			{
				ibuf_isl.feeder_cfg[0].last_units_out = ceil_div(mb_bits_p_line, isl_unit_size_bits);
			}
			else // calculate how many bits are left and how many isl units are needed:
			{
				ibuf_isl.feeder_cfg[0].last_units_out = ceil_div(mb_bits_p_line - (mb_unit_size_bits * (mipi_buffer_units_p_line - 1)), isl_unit_size_bits);
			}

			ibuf_isl.feeder_cfg[0].height  = isl_unit_height; // block height

			return_value = vied_nci_ibufctrl_config_feeder(ibufctrl_isl_dev_t, &ibuf_isl, 0);
		}

		/* Configure MG_STR2MMIO_0 */
		{
			vied_nci_str2mmio_devs dev;
			vied_nci_str2mmio_sid_regs_s sid = {0};
			unsigned int sid_num = 0;

			dev = BXT_STR2MMIO_MG_DEV0;
			str2mmio_mg_dev_t = vied_nci_str2mmio_open_dev(dev);

			/* Configure a single SID for this device */
			sid.pix_width         = width_32bit;
			sid.st_addr           = ( ((stream_id == 0) ? S2M_MG0_MIPI_BUF_ADDR : S2M_MG1_MIPI_BUF_ADDR) + mipi_buffer_start_addr ) >> buf_type; // start address seen by s2m, word addressable value
			sid.stride            = mb_unit_size_words;
			sid.end_addr          = sid.st_addr + mipi_buffer_words - 1;
			sid.block_when_no_cmd = 0;
			sid.ack_base_addr	  = vied_nci_ibufctrl_reg_addr_with_base(vied_nci_ibufctrl_open_dev(BXT_IBUFCTRL_MG_DEV)->ibuf_from_str2mmio_dev, 0, _IBC_2600_GROUP_PROC_CMD, _IBC_2600_PROC_CMD_STR2MMIO_ACK) >> 2;
			sid.sidpid            = 0;
			sid.lut_entry.as_struct.valid = 1; /* Enable will set this, but put 1 if verify when eabled, 0 otherwise */
			sid.lut_entry.as_struct.dtype = (*ppistr_ctx)->stream_cfg_data.input_pins[0].dt + 0x100; /* What is that +0x100??? */
			sid.lut_entry.as_struct.vc = (*ppistr_ctx)->stream_cfg_data.vc;
			sid.frame_config.as_struct.line_width = pix_byte_p_line;
			sid.frame_config.as_struct.nof_lines = nr_lines;

			vied_nci_str2mmio_cfg_sid(str2mmio_mg_dev_t, sid_num, &sid);
		}

		/* Configure ISL_STR2MMIO_0 */
		if ((*ppistr_ctx)->stream_cfg_data.isl_use != IA_CSS_ISYS_USE_NO_ISL_NO_ISA)
		{
			vied_nci_str2mmio_devs isl_dev;
			vied_nci_str2mmio_sid_regs_s isl_sid = {0};
			unsigned int isl_sid_num = 0;

			isl_dev = BXT_STR2MMIO_PIXEL_DEV0;
			str2mmio_isl_dev_t = vied_nci_str2mmio_open_dev(isl_dev);

			/* Configure a single SID for this device */
			isl_sid.pix_width         = width_8bit;
			isl_sid.st_addr           = ( ((stream_id == 0) ? S2M_ISL0_PIX_BUF_ADDR :  S2M_ISL1_PIX_BUF_ADDR ) + pix_buffer_start_addr) >> buf_type; // start address seen by s2m, word addressable value
			isl_sid.stride            = pb_unit_size_words;
			isl_sid.end_addr          = isl_sid.st_addr + pix_buffer_words - 1;
			isl_sid.block_when_no_cmd = 0;
			isl_sid.ack_base_addr     = vied_nci_ibufctrl_reg_addr_with_base(vied_nci_ibufctrl_open_dev(BXT_IBUFCTRL_ISL_DEV)->ibuf_from_str2mmio_dev, 0, _IBC_2600_GROUP_PROC_CMD, _IBC_2600_PROC_CMD_STR2MMIO_ACK)>>2;
			isl_sid.sidpid            = 0;

			vied_nci_str2mmio_cfg_sid(str2mmio_isl_dev_t, isl_sid_num, &isl_sid);
		}

		/* Configure ISL devices */
		if ((*ppistr_ctx)->stream_cfg_data.isl_use != IA_CSS_ISYS_USE_NO_ISL_NO_ISA)
		{
			unsigned int mipi_words_p_isl_unit = ceil_div(isl_unit_size_bits, 32);
			{
				vied_nci_isldevice_mipi_backend_device_t * isldevice_mipi_backend_dev_t0 = vied_nci_isldevice_mipi_backend_open_dev(BXT_ISLDEVICE_MIPI_BACKEND_ISL_DEV0);
				vied_nci_isldevice_mipi_backend_device_t * isldevice_mipi_backend_dev_t1 = vied_nci_isldevice_mipi_backend_open_dev(BXT_ISLDEVICE_MIPI_BACKEND_ISL_DEV1);

				unsigned int alignment_config = ((NBITS(mipi_words_p_isl_unit))-1);
				vied_nci_isldevice_mipi_backend_set_alignment_config(isldevice_mipi_backend_dev_t0, &alignment_config);
				vied_nci_isldevice_mipi_backend_set_alignment_config(isldevice_mipi_backend_dev_t1, &alignment_config);
			}
			{
				vied_nci_isldevice_cio2stream_device_t * isldevice_cio2stream_dev_t = vied_nci_isldevice_cio2stream_open_dev(BXT_ISLDEVICE_CIO2STREAM_ISL_DEV);
				vied_nci_isldevice_cio2stream_cfg_port cfg_port;
				cfg_port.slv_addr_mask = 0xfff0000;
				cfg_port.slv_addr_cmp_val = 0x1cc0000;
				vied_nci_isldevice_cio2stream_set_port_cfg(isldevice_cio2stream_dev_t, 0, &cfg_port);
				cfg_port.slv_addr_mask = 0xfff0000 | 0x00010000;//dma4_isl_host.terminals[1].stride;
				cfg_port.slv_addr_cmp_val = 0x1cc0000 + 0x00010000;//dma4_isl_host.terminals[1].stride;
				vied_nci_isldevice_cio2stream_set_port_cfg(isldevice_cio2stream_dev_t, 1, &cfg_port);
			}

			{
				vied_nci_isldevice_pixel_formatter_device_t * isldevice_pixel_formatter_dev_t = vied_nci_isldevice_pixel_formatter_open_dev(BXT_ISLDEVICE_PIXEL_FORMATTER_ISL_DEV);
				vied_nci_isldevice_pixel_formatter_cfg_reg	pixel_formatter_cfg_reg;
				pixel_formatter_cfg_reg.as_word = 0;	/* never leave register bits undefined */

				if( pstream_dev_cfg->isl_use != IA_CSS_ISYS_USE_SINGLE_ISA)
				{
					pixel_formatter_cfg_reg.as_bitfield.dual_stream_en = 0; /* single steam */
					{
						vied_nci_isldevice_pixel_formatter_cfg_mode_sel cfg_mode_sel = SINGLE_DUAL_STREAM_MUX_MODE;
						pixel_formatter_cfg_reg.as_bitfield.mode_sel = cfg_mode_sel; /* in mux mode */
					}

					pixel_formatter_cfg_reg.as_bitfield.param_set_01_sel = 0; /* select param bank 0 */
					pixel_formatter_cfg_reg.as_bitfield.param_set_23_sel = 0; /* don't care in single stream */
					pixel_formatter_cfg_reg.as_bitfield.resize_top_stream_en = 0; /* do not resize */
					pixel_formatter_cfg_reg.as_bitfield.resize_bot_stream_en = 0; /* don't care in single stream */
					pixel_formatter_cfg_reg.as_bitfield.bypass_pixel_size_sel = 0; /* 4x 8-bit, not 2x 16-bit, but active only in bypass mode (cfg_mode_sel==SINGLE_STREAM_MERGE_MODE_BYPASS_BE) */
				}
				else
				{
					pixel_formatter_cfg_reg.as_bitfield.dual_stream_en = 0; /* single steam */
					{
						vied_nci_isldevice_pixel_formatter_cfg_mode_sel cfg_mode_sel = SINGLE_STREAM_MERGE_MODE;
						pixel_formatter_cfg_reg.as_bitfield.mode_sel = cfg_mode_sel; /* in merge mode */
					}

					pixel_formatter_cfg_reg.as_bitfield.param_set_01_sel = 0; /* select param bank 0 */
					pixel_formatter_cfg_reg.as_bitfield.param_set_23_sel = 0; /* don't care in single stream */
					pixel_formatter_cfg_reg.as_bitfield.resize_top_stream_en = 0; /* do not resize */
					pixel_formatter_cfg_reg.as_bitfield.resize_bot_stream_en = 0; /* don't care in single stream */
					pixel_formatter_cfg_reg.as_bitfield.bypass_pixel_size_sel = 0; /* 4x 8-bit, not 2x 16-bit, but active only in bypass mode (cfg_mode_sel==SINGLE_STREAM_MERGE_MODE_BYPASS_BE) */
					{
						vied_nci_isldevice_pixel_formatter_param_bank set_bank0 = {0};
						set_bank0.start_param_reg.as_bitfield.horz_start = 0;
						set_bank0.start_param_reg.as_bitfield.vert_start = 0;
						set_bank0.size_param_reg.as_bitfield.horz_size = (*ppistr_ctx)->stream_cfg_data.output_pins[0].output_res.width;
						set_bank0.size_param_reg.as_bitfield.vert_size = (*ppistr_ctx)->stream_cfg_data.output_pins[0].output_res.height;
						set_bank0.out_param_reg.as_word = 0x6;
						set_bank0.pad_param_reg.as_word = 0;
						vied_nci_isldevice_pixel_formatter_set_param_bank(isldevice_pixel_formatter_dev_t, 0 /*bank_id*/,&set_bank0);
					}
				}

				vied_nci_isldevice_pixel_formatter_set_cfg_reg(isldevice_pixel_formatter_dev_t, &pixel_formatter_cfg_reg);
			}
		}

		if( pstream_dev_cfg->isl_use == IA_CSS_ISYS_USE_SINGLE_ISA)
		{
			isa_ctrl_trigger_update(ISA_FSM_TRIGGER_ISTREAM_CONFIG,
									(void *) pstream_dev_cfg);
		}
#endif /* RUN_INTEGRATION */
	}

	return IA_CSS_ISYS_STREAM_NO_ERROR;
}

/*
 * istream_open
 */
enum istream_err istream_open(
	struct istream_ctx *pistr_ctx
){
	verifret(pistr_ctx != NULL, IA_CSS_ISYS_STREAM_OPEN_STREAM_FAIL);

#ifdef RUN_INTEGRATION

	/* Enable MG_STR2MMIO_0 */
	{
		vied_nci_str2mmio_enable_sid(str2mmio_mg_dev_t, 0);	// activate the required SIDs... through stream_cfg_data?
		//vied_nci_str2mmio_enable_sid(str2mmio_isl_dev_t, 0); // pixel str2mmio has no enable bit
	}

	/* Initialize MG_IBUF */
	{
		vied_nci_ibufctrl_send_init(ibufctrl_mg_dev_t, 0);	// which IBUF depends on test, which should depend on the output pin type?
	}

	/* Initialize ISL_IBUF */
	if (pistr_ctx->stream_cfg_data.isl_use != IA_CSS_ISYS_USE_NO_ISL_NO_ISA)
	{
		vied_nci_ibufctrl_send_init(ibufctrl_isl_dev_t, 0);
	}
#endif
	return IA_CSS_ISYS_STREAM_NO_ERROR;
}

/*
 * istream_capture
 */
enum istream_err istream_capture(
	struct istream_ctx *pistr_ctx,
	const struct ia_css_isys_frame_buff_set_comm *frame_buf
){
	verifret(pistr_ctx != NULL, IA_CSS_ISYS_STREAM_TRANSFER_FAIL);
	verifret(frame_buf != NULL, IA_CSS_ISYS_STREAM_TRANSFER_FAIL);

#ifdef RUN_INTEGRATION
	{
		unsigned int mg_ibuf_cmd_dest_idx = 0;
		unsigned int isl_ibuf_cmd_dest_idx = 0;
		unsigned int out_pin;
		vied_nci_ibufctrl_cmd_s mg_ibuf_cmd = {0};
		vied_nci_ibufctrl_cmd_s isl_ibuf_cmd = {0};

		mg_ibuf_cmd.nr_dests = 2;	// Must be nof_output_pins?
		isl_ibuf_cmd.nr_dests = 2;	// Must be nof_output_pins?

		for (out_pin = 0; out_pin < pistr_ctx->stream_cfg_data.nof_output_pins ; out_pin++)
		{
			if (pistr_ctx->stream_cfg_data.output_pins[out_pin].pt == IA_CSS_ISYS_PIN_TYPE_MIPI)
			{
				mg_ibuf_cmd.dest[mg_ibuf_cmd_dest_idx].cmd = addr_next_token;
				mg_ibuf_cmd.dest[mg_ibuf_cmd_dest_idx].address = frame_buf->output_pins[out_pin].payload.addr;
				mg_ibuf_cmd_dest_idx++;
			}
			else if (pistr_ctx->stream_cfg_data.output_pins[out_pin].pt == IA_CSS_ISYS_PIN_TYPE_RAW_NS)
			{
				mg_ibuf_cmd.dest[mg_ibuf_cmd_dest_idx].cmd = addr_register;
				mg_ibuf_cmd.dest[mg_ibuf_cmd_dest_idx].address = 0;
				mg_ibuf_cmd_dest_idx++;

				isl_ibuf_cmd.dest[isl_ibuf_cmd_dest_idx].cmd = addr_next_token;
				isl_ibuf_cmd.dest[isl_ibuf_cmd_dest_idx].address = frame_buf->output_pins[out_pin].payload.addr;
				isl_ibuf_cmd_dest_idx++;
			}
		}

		vied_nci_ibufctrl_send_cmd(ibufctrl_mg_dev_t, &mg_ibuf_cmd, 0);
		if (pistr_ctx->stream_cfg_data.isl_use != IA_CSS_ISYS_USE_NO_ISL_NO_ISA)
		{
			vied_nci_ibufctrl_send_cmd(ibufctrl_isl_dev_t, &isl_ibuf_cmd, 0);
		}

		/*todo: 1/ rename istream_device to stream_adi
		 * 2/ Add stream_adi_close interface call for relasing HW resources and resetting ISA ctrl state*/
		if (pistr_ctx->stream_cfg_data.isl_use == IA_CSS_ISYS_USE_SINGLE_ISA)
		{
			isa_ctrl_trigger_update(	ISA_FSM_TRIGGER_ISTREAM_CAPTURE,
									(void *) frame_buf);
		}
	}
#endif

	return IA_CSS_ISYS_STREAM_NO_ERROR;
}

/*
 * istream_sync
 */
enum istream_err istream_sync(
	struct istream_ctx *pistr_ctx
){
	verifret(pistr_ctx != NULL, IA_CSS_ISYS_STREAM_SYNC_FAIL);

	return IA_CSS_ISYS_STREAM_NO_ERROR;
}

/*
 *  TODO: istream_stop which should free up all the resources related to the stream, upon
 *        a IA_CSS_ISYS_SEND_TYPE_STREAM_STOP message from the driver
 */

