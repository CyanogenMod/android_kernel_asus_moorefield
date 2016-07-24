#ifndef _defines_h_
#define _defines_h_

#define CHANNEL_GROUP_ID  0
#define REQUEST_GROUP_ID  1
#define GLOBAL_GROUP_ID   2
#define MASTER_GROUP_ID   3
#define SPAN_GROUP_ID     4
#define UNIT_GROUP_ID     5
#define TERMINAL_GROUP_ID 6

#define GLOBAL_SETS 1
//_hrt_device_input_feeder_inst_dma4_inst_property_GlobalSets

#define REQUEST_REGS  5
#define UNIT_REGS     16
#define SPAN_REGS     16
#define TERMINAL_REGS 16
#define CHANNEL_REGS  16
#define MASTER_REGS   4
#define GLOBAL_REGS   (7 + 2*GLOBAL_SETS)

#define MASTER_BANKS   2
#define GLOBAL_BANKS   1

// #define REQUEST_BANKS  _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_RequestBanks
// #define UNIT_BANKS     _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_UnitBanks
// #define SPAN_BANKS     _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_SpanBanks
// #define TERMINAL_BANKS _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_TerminalBanks
// #define CHANNEL_BANKS  _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_ChannelBanks

#define REQUEST_BANKS  5
#define UNIT_BANKS     5
#define SPAN_BANKS     10
#define TERMINAL_BANKS 10
#define CHANNEL_BANKS  5

#define DESCRIPTOR_ID_BITS _log2(MAX(UNIT_BANKS, MAX(SPAN_BANKS, MAX(TERMINAL_BANKS, CHANNEL_BANKS))))
#define DESCRIPTOR_KIND_BITS 2

#define CTRLS_ADDR_BITS 32
#define CTRLS_DATA_BITS 32
// #define CTRLS_DATA_BITS processing_system_unps_logic_ipfd_config_bus_inst_dma_cfg_data_width

#define CTRLM_ADDR_BITS 32
#define CTRLM_DATA_BITS 32
// #define CTRLM_DATA_BITS processing_system_unps_logic_ipfd_dma4_inst_mC_data_width

#define DATAMA_ADDR_BITS 32
#define DATAMA_DATA_BITS 512
// #define DATAMA_DATA_BITS processing_system_unps_logic_ipfd_dma4_inst_mA_data_width

#define DATAMB_ADDR_BITS 32
#define DATAMB_DATA_BITS 512
// #define DATAMB_DATA_BITS processing_system_unps_logic_ipfd_dma4_inst_mB_data_width

#define MAX_REGION_STRIDE_A        65536
#define MAX_REGION_STRIDE_B        65536

#define UNIQUE_PRECISIONS          2
#define UNIQUE_PRECISION_LIST     {8, 16}

#define UNIQUE_SUBSAMPLING_FACTORS 1
#define UNIQUE_SUBSAMPLING_FACTOR_LIST {1}

#define CIO_INFO_BITS_A            0
#define CIO_INFO_BITS_B            0

#define PORT_MODE_BITS             1

// #define MAX_PADDING_AMOUNT       _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_MaxPaddingAmount
#define MAX_PADDING_AMOUNT       0

#define TERMINALS                  0x10
#define MAX_REGION_WIDTH           0x4000
#define MAX_REGION_HEIGHT          0x2000
// #define TERMINALS                  _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_Terminals
// #define MAX_REGION_WIDTH           _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_MaxRegionWidth
// #define MAX_REGION_HEIGHT          _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_MaxRegionHeight
#define MAX_REGION_STRIDE          MAX(MAX_REGION_STRIDE_A, MAX_REGION_STRIDE_B)

#define REGION_ORIGIN_BITS         MAX(DATAMA_ADDR_BITS, DATAMB_ADDR_BITS)
#define REGION_WIDTH_BITS          _log2(MAX_REGION_WIDTH)
#define REGION_STRIDE_BITS         _log2(MAX_REGION_STRIDE)

#define ELEMENT_SETUP_BITS         _log2(UNIQUE_PRECISIONS)
#define CIO_INFO_SETUP_BITS        MAX(CIO_INFO_BITS_A, CIO_INFO_BITS_B)

#define MIN_X_COORDINATE           (-MAX_PADDING_AMOUNT)
#define MAX_X_COORDINATE           (MAX_REGION_WIDTH - 1 + MAX_PADDING_AMOUNT)
#define X_COORDINATE_BITS          _log2(-MIN_X_COORDINATE + MAX_X_COORDINATE + 1)

#define MIN_Y_COORDINATE           0
#define MAX_Y_COORDINATE           (MAX_REGION_HEIGHT - 1)
#define Y_COORDINATE_BITS          _log2(-MIN_Y_COORDINATE + MAX_Y_COORDINATE + 1)

// #define UNITS                      _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_Units
// #define MAX_UNIT_WIDTH             _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_MaxUnitWidth
// #define MAX_UNIT_HEIGHT            _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_MaxUnitHeight
// #define SPANS                      _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_Spans
// #define MAX_SPAN_WIDTH             _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_MaxSpanWidth
// #define MAX_SPAN_HEIGHT            _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_MaxSpanHeight

#define UNITS                      0x8
#define MAX_UNIT_WIDTH             0x400
#define MAX_UNIT_HEIGHT            0x20
#define SPANS                      0x10
#define MAX_SPAN_WIDTH             0x200
#define MAX_SPAN_HEIGHT            0x1000

#define UNIT_LOCATION_BITS         MAX(MAX(DATAMA_ADDR_BITS, DATAMB_ADDR_BITS), X_COORDINATE_BITS + Y_COORDINATE_BITS)
#define SPAN_WIDTH_BITS            _log2(MAX_SPAN_WIDTH)
#define SPAN_HEIGHT_BITS           _log2(MAX_SPAN_HEIGHT)
#define SPAN_MODE_BITS              2

#define CHANNELS                   0x8
#define ELEMENT_INIT_DATA_BITS     0x0
// #define CHANNELS                   _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_Channels
// #define ELEMENT_INIT_DATA_BITS     _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_ElementInitDataBits

#define ELEMENT_EXTEND_MODE_BITS   1
#define PADDING_MODE_BITS          3
#define SAMPLING_SETUP_BITS        _log2(UNIQUE_SUBSAMPLING_FACTORS)
#define GLOBAL_SET_ID_BITS         _log2(GLOBAL_SETS)
// #define ACK_MODE_BITS              _log2(_hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_MaxCompletedCount + 1)
#define ACK_MODE_BITS              _log2(0xF + 1)
#define ACK_ADDR_BITS              CTRLM_ADDR_BITS
#define ACK_DATA_BITS              CTRLM_DATA_BITS

#define MASTER_BANKS               2

#define MAX_BLOCK_HEIGHT           0x20
#define MAX_BLOCK_WIDTH            0x400
#define MAX_LINEAR_BURST_SIZE      0x400
// #define MAX_BLOCK_HEIGHT           _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_MaxBlockHeight
// #define MAX_BLOCK_WIDTH            _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_MaxBlockWidth
// #define MAX_LINEAR_BURST_SIZE      _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_MaxLinearBurstSize
#define MAX_STRIDE                 4096

#define BLOCK_HEIGHT_BITS          _log2(MAX_BLOCK_HEIGHT)
#define BLOCK_WIDTH_BITS           _log2(MAX_BLOCK_WIDTH)
#define LINEAR_BURST_SIZE_BITS     _log2(MAX_LINEAR_BURST_SIZE)
#define STRIDE_BITS                _log2(MAX_STRIDE + 1)
#define SRMD_SUPPORT_BITS          1
#define BURST_SUPPORT_BITS         2

// #define MAX_MACRO_SIZE             _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_MaxMacroSize
#define MAX_MACRO_SIZE             0x20
#define MACRO_SIZE_BITS            _log2(MAX_MACRO_SIZE)
#define INSTRUCTION_BITS           (MACRO_SIZE_BITS + 11)
#define DESCRIPTOR_ID_SETUP_1_BITS (_log2(CHANNELS) + 2*_log2(TERMINALS) + _log2(UNITS))
#define DESCRIPTOR_ID_SETUP_2_BITS (2*_log2(SPANS))

// #define COMPLETED_COUNT_BITS       _log2(_hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_MaxOperationsPending + 1)
#define COMPLETED_COUNT_BITS       _log2(0x40 + 1)

#endif
