#ifndef _vied_nci_dma_local_h_
#define _vied_nci_dma_local_h_
//#define MAX_DESC_WORDS 8

#define CHANNEL_GROUP_ID  0
#define REQUEST_GROUP_ID  1
#define GLOBAL_GROUP_ID   2
#define MASTER_GROUP_ID   3
#define SPAN_GROUP_ID     4
#define UNIT_GROUP_ID     5
#define TERMINAL_GROUP_ID 6
#define REQUEST_REGS  5
#define UNIT_REGS     16
#define SPAN_REGS     16
#define TERMINAL_REGS 16
#define CHANNEL_REGS  16
#define MASTER_REGS   4

#define MASTER_BANKS   2
#define GLOBAL_BANKS   1

//#define CTRLS_DATA_BITS input_feeder_inst_config_bus_inst_dma_cfg_data_width
#define CTRLS_DATA_BITS	0x20
//#define CTRLS_DATA_BITS       processing_system_unps_logic_configbus_bus_P2_host_mt_dmae0i_data_width
#define CTRLM_DATA_BITS	0x20
// DMA External0:  CTRLM_DATA_BITS = processing_system_dma_logic_dma_external0_m2_data_width
// DMA External1:  CTRLM_DATA_BITS = processing_system_dma_logic_dma_external1_m2_data_width
// DMA Internal:     CTRLM_DATA_BITS = processing_system_dma_logic_dma_internal_m2_data_width
// DMA Firmware:  CTRLM_DATA_BITS = processing_system_dma_logic_dma_firmware_m2_data_width

// PS address width is always 32bits
#define CTRLM_ADDR_BITS	 0x20
#define DATAMB_ADDR_BITS 0x20
#define DATAMA_ADDR_BITS 0x20

#define CIO_INFO_BITS_A            0
#define CIO_INFO_BITS_B            0

#define PORT_MODE_BITS             1
#define DESCRIPTOR_KIND_BITS       2
#define SPAN_MODE_BITS             2
#define ELEMENT_EXTEND_MODE_BITS   1
#define ACK_ADDR_BITS              CTRLM_ADDR_BITS
#define ACK_DATA_BITS              CTRLM_DATA_BITS
#define PADDING_MODE_BITS          3

//#define CTRLS_DATA_BYTES BYTES(CTRLS_DATA_BITS)
//#define CTRLM_DATA_BYTES BYTES(CTRLM_DATA_BITS)
#define CTRLS_DATA_BYTES 4
#define CTRLM_DATA_BYTES 4

#define MAX_DESC_WORDS 4

#define EXECUTION_INVAL_MODIFIER_BITS  0x6
#define EXECUTION_COMMAND_BITS         0x4
#define EXECUTION_FORMAT_BITS          0x1
#define REQUEST_EXECUTION_FORMAT       0x0
#define REQUEST_INVALIDATION_FORMAT    0x1

#ifndef NOT_USED
#define NOT_USED(a) ((void)(a))
#endif

/* Global Register */
#define REG_ERROR              0x0
#define REG_IDLE               0x1
#define REG_UNIT_BASE_ADDR     0x2
#define REG_SPAN_BASE_ADDR     0x3
#define REG_TERMINAL_BASE_ADDR 0x4
#define REG_CHANNEL_BASE_ADDR  0x5
#define REG_MAX_BLOCK_HEIGHT   0x6
#define REG_MAX_BLOCK_WIDTH_1DBURST(SET) (0x7 + 2*(SET))
#define REG_MAX_BLOCK_WIDTH_2DBURST(SET) (0x8 + 2*(SET))

/* Master Register */
#define REG_SRMD_SUPPORT       0x0
#define REG_BURST_SUPPORT      0x2
#define REG_MAX_PHYSICAL_STRIDE 0x3

/* Request Register */
#define REG_INSTRUCTION        0x0
#define REG_DESC_ID_SETUP1     0x1
#define REG_DESC_ID_SETUP2     0x2
#define REG_REQUEST_VALID      0x3
#define REG_REQUEST_RESOURCED  0x4
#define REQUEST_MACRO_SIZE_BIT 0xB
#define REQUEST_FORMAT_BIT 0x0
#define REQUEST_TRANSFER_DIRECTION_BIT 0x3

/* Unit Register */
#define REG_UNIT_WIDTH         0x0
#define REG_UNIT_HEIGHT        0x1

/* Span Register */
#define REG_UNIT_LOCATION      0x0
#define REG_SPAN_COLUMN        0x1
#define REG_SPAN_ROW           0x2
#define REG_SPAN_WIDTH         0x3
#define REG_SPAN_HEIGHT        0x4
#define REG_SPAN_MODE          0x5
#define SPAN_ADDRESSING_MODE_BIT 0x0
#define SPAN_SPAN_ORDER_BIT    0x1

/* Terminal Register */
#define REG_REGION_ORIGIN      0x0
#define REG_REGION_WIDTH       0x1
#define REG_REGION_STRIDE      0x2
#define REG_ELEMENT_SETUP      0x3
#define REG_CIO_INFO           0x4
#define REG_PORT_MODE          0x5

/* Channel Register */
#define REG_ELEMENT_EXTEND_MODE 0x0
#define REG_ELEMENT_INIT_DATA  0x1
#define REG_PADDING_MODE       0x2
#define REG_SAMPLING_SETUP     0x3
#define REG_GLOBAL_SET_ID      0x4
#define REG_ACK_MODE           0x5
#define REG_ACK_ADDRESS        0x6
#define REG_ACK_DATA           0x7


/* Bank Status and Control */
#define REG_COMPLETED_COUNTER  0xB
#define REG_PENDING_COUNTER    0xC
#define REG_LOCK_STATUS        0xD
#define REG_BANK_MODE          0xF

#define REG_REG0               0x0
#endif
