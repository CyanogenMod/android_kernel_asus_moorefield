#ifndef _STREAM2MMMIO_DEFS_H
#define _STREAM2MMMIO_DEFS_H

#include <mipi_backend_defs.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////
/* MIPI_STREAM2MMIO: Incoming MIPI CSI2 stream definition */
#define HRT_CSI2_RX_DATA_LSB						 0
#define HRT_CSI2_RX_DATA_MSB						31
#define HRT_CSI2_RX_SOP 								32
#define HRT_CSI2_RX_EOP 								33
#define HRT_CSI2_RX_ERROR_FLAG					34
#define HRT_CSI2_RX_ERROR_CAUSE_LSB 		35
#define HRT_CSI2_RX_ERROR_CAUSE_MSB 		46
#define HRT_CSI2_RX_FRAME_ID_LSB				47
#define HRT_CSI2_RX_FRAME_ID_MSB				62

/* DO NOT TOUCH THE FOLLOWING */
#define HRT_CSI2_RX_DATA_WIDTH  				(HRT_CSI2_RX_DATA_MSB - HRT_CSI2_RX_DATA_LSB + 1)									
#define HRT_CSI2_RX_SOP_WIDTH 					1
#define HRT_CSI2_RX_EOP_WIDTH 					1
#define HRT_CSI2_RX_ERROR_FLAG_WIDTH	  1
#define HRT_CSI2_RX_ERROR_CAUSE_WIDTH   HRT_CSI2_RX_ERROR_CAUSE_MSB - HRT_CSI2_RX_ERROR_CAUSE_LSB + 1 
#define HRT_CSI2_RX_FRAME_ID_WIDTH			HRT_CSI2_RX_FRAME_ID_MSB - HRT_CSI2_RX_FRAME_ID_LSB + 1 

#define HRT_CSI2_RX_PPI_WIDTH	          HRT_CSI2_RX_DATA_WIDTH + HRT_CSI2_RX_SOP_WIDTH + \
																				HRT_CSI2_RX_EOP_WIDTH + HRT_CSI2_RX_ERROR_FLAG_WIDTH + \
																				HRT_CSI2_RX_ERROR_CAUSE_WIDTH + HRT_CSI2_RX_FRAME_ID_WIDTH 
///////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////
/* MIPI_STREAM2MMIO: Incoming MIPI CSI3 stream definition */
#define HRT_CSI3_RX_DATA_LSB						 0
#define HRT_CSI3_RX_DATA_MSB						63
#define HRT_CSI3_RX_SOP 								64
#define HRT_CSI3_RX_EOP 								65
#define HRT_CSI3_RX_ERROR_FLAG					66
#define HRT_CSI3_RX_ERROR_CAUSE_LSB 		67
#define HRT_CSI3_RX_ERROR_CAUSE_MSB 		82
#define HRT_CSI3_RX_FRAME_ID_LSB				83
#define HRT_CSI3_RX_FRAME_ID_MSB				98
#define HRT_CSI3_RX_LINE_ID_LSB					99
#define HRT_CSI3_RX_LINE_ID_MSB				 114	
#define HRT_CSI3_RX_DATA_VALID_LSB		 115
#define HRT_CSI3_RX_DATA_VALID_MSB		 122

/* DO NOT TOUCH THE FOLLOWING */
#define HRT_CSI3_RX_DATA_WIDTH					HRT_CSI3_RX_DATA_MSB - HRT_CSI3_RX_DATA_LSB + 1
#define HRT_CSI3_RX_SOP_WIDTH 					1
#define HRT_CSI3_RX_EOP_WIDTH 					1
#define HRT_CSI3_RX_ERROR_FLAG_WIDTH		1
#define HRT_CSI3_RX_ERROR_CAUSE_WIDTH 	HRT_CSI3_RX_ERROR_CAUSE_MSB - HRT_CSI3_RX_ERROR_CAUSE_LSB + 1
#define HRT_CSI3_RX_FRAME_ID_WIDTH      HRT_CSI3_RX_FRAME_ID_MSB - HRT_CSI3_RX_FRAME_ID_LSB	+ 1
#define HRT_CSI3_RX_LINE_ID_WIDTH				HRT_CSI3_RX_LINE_ID_MSB - HRT_CSI3_RX_LINE_ID_LSB + 1	
#define HRT_CSI3_RX_DATA_VALID_WIDTH		HRT_CSI3_RX_DATA_VALID_MSB - HRT_CSI3_RX_DATA_VALID_LSB + 1 

#define HRT_CSI3_RX_PPI_WIDTH           HRT_CSI3_RX_DATA_WIDTH + HRT_CSI3_RX_SOP_WIDTH + HRT_CSI3_RX_EOP_WIDTH + \
																				HRT_CSI3_RX_ERROR_FLAG_WIDTH + HRT_CSI3_RX_ERROR_CAUSE_WIDTH + \
																				HRT_CSI3_RX_FRAME_ID_WIDTH + HRT_CSI3_RX_LINE_ID_WIDTH + \
																				HRT_CSI3_RX_DATA_VALID_WIDTH
///////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////
/* MIPI_STREAM2MMIO: Incoming MIPI CPHY stream definition */
#define HRT_CPHY_RX_DATA_LSB						 0
#define HRT_CPHY_RX_DATA_MSB						63
#define HRT_CPHY_RX_SOP 								64
#define HRT_CPHY_RX_EOP 								65
#define HRT_CPHY_RX_ERROR_FLAG					66
#define HRT_CPHY_RX_ERROR_CAUSE_LSB 		67
#define HRT_CPHY_RX_ERROR_CAUSE_MSB 		78
#define HRT_CPHY_RX_FRAME_ID_LSB				79
#define HRT_CPHY_RX_FRAME_ID_MSB				94

/* DO NOT TOUCH THE FOLLOWING */
#define HRT_CPHY_RX_DATA_WIDTH  				(HRT_CPHY_RX_DATA_MSB - HRT_CPHY_RX_DATA_LSB + 1)									
#define HRT_CPHY_RX_SOP_WIDTH 					1
#define HRT_CPHY_RX_EOP_WIDTH 					1
#define HRT_CPHY_RX_ERROR_FLAG_WIDTH	  1
#define HRT_CPHY_RX_ERROR_CAUSE_WIDTH   HRT_CPHY_RX_ERROR_CAUSE_MSB - HRT_CPHY_RX_ERROR_CAUSE_LSB + 1 
#define HRT_CPHY_RX_FRAME_ID_WIDTH			HRT_CPHY_RX_FRAME_ID_MSB - HRT_CPHY_RX_FRAME_ID_LSB + 1 

#define HRT_CPHY_RX_PPI_WIDTH	          HRT_CPHY_RX_DATA_WIDTH + HRT_CPHY_RX_SOP_WIDTH + \
																				HRT_CPHY_RX_EOP_WIDTH + HRT_CPHY_RX_ERROR_FLAG_WIDTH + \
																				HRT_CPHY_RX_ERROR_CAUSE_WIDTH + HRT_CPHY_RX_FRAME_ID_WIDTH 





///////////////////////////////////////////////////////////////////////////////////////////////////////////
/* MIPI_STREAM2MMIO: CSIX_PACKET_HEADER definition */
#define HRT_CSIX_PH_WORDCOUNT_LSB			  0
#define HRT_CSIX_PH_WORDCOUNT_MSB			  15
#define HRT_CSIX_PH_DTYPE_LSB 					16
#define HRT_CSIX_PH_DTYPE_MSB 					28
#define HRT_CSIX_PH_SYNC_LSB			      29
#define HRT_CSIX_PH_SYNC_MSB			      30
#define HRT_CSIX_PH_SENSOR_TYPE 			  31
//#define HRT_CSIX_PH_RESERVED_LSB			  32
//#define HRT_CSIX_PH_RESERVED_MSB			  63
#define HRT_CSIX_PH_SID_LSB 			  		32


/* DO NOT TOUCH THE FOLLOWING */
#define HRT_CSIX_PH_DTYPE_WIDTH 				HRT_CSIX_PH_DTYPE_MSB - HRT_CSIX_PH_DTYPE_LSB + 1
#define HRT_CSIX_PH_SENSOR_TYPE_WIDTH 	1
#define HRT_CSIX_PH_SYNC_WIDTH          HRT_CSIX_PH_SYNC_MSB - HRT_CSIX_PH_SYNC_LSB + 1
#define HRT_CSIX_PH_WORDCOUNT_WIDTH			HRT_CSIX_PH_WORDCOUNT_MSB - HRT_CSIX_PH_WORDCOUNT_LSB + 1	

///////////////////////////////////////////////////////////////////////////////////////////////////////////


#define NOF_IRQS_PER_SID 2

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// NON SID RELATED REGISTERS
///////////////////////////////////////////////////////////////////////////////////////////////////////////
#define _MIPI_STREAM2MMIO_COMMON_REG_OFFSET   					16

#define _MIPI_STREAM2MMIO_CSI2_COMPFMT_VC0_REG_ID 			0
#define _MIPI_STREAM2MMIO_CSI2_COMPFMT_VC1_REG_ID 			1
#define _MIPI_STREAM2MMIO_CSI2_COMPFMT_VC2_REG_ID 			2
#define _MIPI_STREAM2MMIO_CSI2_COMPFMT_VC3_REG_ID 			3
//#define _MIPI_STREAM2MMIO_ERR_MODE_ENABLE_REG_ID        4 
#define _MIPI_STREAM2MMIO_ERR_MODE_FILL_VAL_REG_ID  		4
#define _MIPI_STREAM2MMIO_ERR_MODE_PAUSE_CNTR_REG_ID  	5

#define _MIPI_STREAM2MMIO_NOF_COMMON_REGS         			6

#define _MIPI_STREAM2MMIO_CSI2_COMPFMT_REG_WIDTH  			32
//#define _MIPI_STREAM2MMIO_ERR_MODE_ENABLE_REG_WIDTH 		1
#define _MIPI_STREAM2MMIO_ERR_MODE_FILL_VAL_REG_WIDTH 	1 
#define _MIPI_STREAM2MMIO_ERR_MODE_PAUSE_CNTR_REG_WIDTH 4

///////////////////////////////////////////////////////////////////////////////////////////////////////////


#define _STREAM2MMIO_REG_ALIGN                  4

#define _STREAM2MMIO_COMMAND_REG_ID             0
#define _STREAM2MMIO_PIX_WIDTH_ID_REG_ID        1
#define _STREAM2MMIO_START_ADDR_REG_ID          2      /* master port address,NOT Byte */
#define _STREAM2MMIO_END_ADDR_REG_ID            3      /* master port address,NOT Byte */
#define _STREAM2MMIO_STRIDE_REG_ID              4      /* stride in master port words, increment is per packet for long sids, stride is not used for short sid's*/
#define _STREAM2MMIO_NUM_ITEMS_REG_ID           5      /* number of packets for store packets cmd, number of words for store_words cmd */ 
#define _STREAM2MMIO_BLOCK_WHEN_NO_CMD_REG_ID   6      /* if this register is 1, input will be stalled if there is no pending command for this sid */
#define _STREAM2MMIO_ACK_BASE_ADDR_REG_ID       7
#define _STREAM2MMIO_SIDPID_REG_ID        			8

#define _STREAM2MMIO_REGS_PER_SID               9
#define _STREAM2MMIO_SID_REG_OFFSET             16

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// MIPI STREAM2MMIO SPECIFIC
#define _MIPI_STREAM2MMIO_LUT_ENTRY_REG_ID 		  	 9
#define _MIPI_STREAM2MMIO_FRAME_CONFIG_REG_ID   	10 
#define _MIPI_STREAM2MMIO_ERR_MODE_ENABLE_REG_ID  11

#define _MIPI_STREAM2MMIO_REGS_PER_SID          	12
#define _MIPI_STREAM2MMIO_SID_REG_OFFSET        	16


//#define _STREAM2MMIO_ACK_BASE_ADDR_REG_WIDTH      18         
//#define _STREAM2MMIO_SIDPID_REG_WIDTH           --> defined by parameters 

#define _MIPI_STREAM2MMIO_LUT_ENTRY_REG_WIDTH   	(2+13+1) // VC + DTYPE + VALID
#define _MIPI_STREAM2MMIO_FRAME_CONFIG_REG_WIDTH	32		 // nof lines (16) + line length (16)	
#define _MIPI_STREAM2MMIO_ERR_MODE_ENABLE_REG_WIDTH 		1

#define _MIPI_STREAM2MMIO_LUT_ENTRY_VALID_BIT 				 0
#define _MIPI_STREAM2MMIO_LUT_ENTRY_DTYPE_LSB 				 1
#define _MIPI_STREAM2MMIO_LUT_ENTRY_DTYPE_MSB 				13
#define _MIPI_STREAM2MMIO_LUT_ENTRY_VC_LSB						14
#define _MIPI_STREAM2MMIO_LUT_ENTRY_VC_MSB						15

#define _MIPI_STREAM2MMIO_FRAME_CONFIG_NOFLINES_LSB 	  16
#define _MIPI_STREAM2MMIO_FRAME_CONFIG_NOFLINES_MSB 	  31
#define _MIPI_STREAM2MMIO_FRAME_CONFIG_NOFLINES_WIDTH   16
#define _MIPI_STREAM2MMIO_FRAME_CONFIG_LINELENGTH_LSB 	0
#define _MIPI_STREAM2MMIO_FRAME_CONFIG_LINELENGTH_MSB 	15
#define _MIPI_STREAM2MMIO_FRAME_CONFIG_LINELENGTH_WIDTH 16


//////////////////////////////////////////////////////////////////////////////////////////////////////////

#define _STREAM2MMIO_MAX_NOF_SIDS              64      /* value used in hss model */

/* command token definition     */
#define _STREAM2MMIO_CMD_TOKEN_CMD_LSB          0      /* bits 1-0 is for the command field */
#define _STREAM2MMIO_CMD_TOKEN_CMD_MSB          1

#define _STREAM2MMIO_CMD_TOKEN_WIDTH           (_STREAM2MMIO_CMD_TOKEN_CMD_MSB+1-_STREAM2MMIO_CMD_TOKEN_CMD_LSB)

#define _STREAM2MMIO_CMD_TOKEN_STORE_WORDS              0      /* command for storing a number of output words indicated by reg _STREAM2MMIO_NUM_ITEMS */
#define _STREAM2MMIO_CMD_TOKEN_STORE_PACKETS            1      /* command for storing a number of packets indicated by reg _STREAM2MMIO_NUM_ITEMS      */
#define _STREAM2MMIO_CMD_TOKEN_SYNC_FRAME               2      /* command for waiting for a frame start                                                */

/* acknowledges from packer module */
/* fields: eof   - indicates whether last (short) packet received was an eof packet */
/*         eop   - indicates whether command has ended due to packet end or due to no of words requested has been received */
/*         count - indicates number of words stored */
#define _STREAM2MMIO_PACK_NUM_ITEMS_BITS        16
#define _STREAM2MMIO_PACK_ACK_EOP_BIT           _STREAM2MMIO_PACK_NUM_ITEMS_BITS   // 16
#define _STREAM2MMIO_PACK_ACK_EOF_BIT           (_STREAM2MMIO_PACK_ACK_EOP_BIT+1)  // 17
#define _STREAM2MMIO_PACK_ACK_ERR_BIT           (_STREAM2MMIO_PACK_ACK_EOF_BIT+1)  // 18

/* acknowledge token definition */
#define _STREAM2MMIO_ACK_TOKEN_NUM_ITEMS_LSB    0      /* bits 3-0 is for the command field */
#define _STREAM2MMIO_ACK_TOKEN_NUM_ITEMS_MSB   (_STREAM2MMIO_PACK_NUM_ITEMS_BITS-1)
#define _STREAM2MMIO_ACK_TOKEN_EOP_BIT         _STREAM2MMIO_PACK_ACK_EOP_BIT
#define _STREAM2MMIO_ACK_TOKEN_EOF_BIT         _STREAM2MMIO_PACK_ACK_EOF_BIT
#define _STREAM2MMIO_ACK_TOKEN_VALID_BIT       (_STREAM2MMIO_ACK_TOKEN_EOF_BIT+1)      /* this bit indicates a valid ack    */
                                                                                       /* if there is no valid ack, a read  */
                                                                                       /* on the ack register returns 0     */
#define _STREAM2MMIO_ACK_TOKEN_WIDTH           (_STREAM2MMIO_ACK_TOKEN_VALID_BIT+1)

/* commands for packer module */
#define _STREAM2MMIO_PACK_CMD_STORE_WORDS        0
#define _STREAM2MMIO_PACK_CMD_STORE_LONG_PACKET  1
#define _STREAM2MMIO_PACK_CMD_STORE_SHORT_PACKET 2




#endif /* _STREAM2MMIO_DEFS_H */
