#include <string.h>
#include <master_to_slave_hrt.h>
#include <hrt/api.h>
#include <hrt/system.h>
#include <host.h>

//#include "math_support.h"
#include "property.h"
#include "vied_bit.h"

//TODO: Use MAX in math_support.h when it is available
#define MAX(a,b) (((a) > (b)) ? (a) : (b))

#ifdef VIED_NCI_DMA_ISYS
// ISys DMA properties
const struct dma_prop_t dev_prop[VIED_NCI_N_DMA_DEV] = {
	{
	 _hrt_device_input_system_unis_logic_fw_dma_dma_property_NativeCIO2DBlock,
	 _hrt_device_input_system_unis_logic_fw_dma_dma_property_NativeCIORAccept,
	 _hrt_device_input_system_unis_logic_fw_dma_dma_property_NativeCIOSRMD,
	 _hrt_device_input_system_unis_logic_fw_dma_dma_property_RequestBanks,
	 _hrt_device_input_system_unis_logic_fw_dma_dma_property_Units,
	 _hrt_device_input_system_unis_logic_fw_dma_dma_property_UnitBanks,
	 _hrt_device_input_system_unis_logic_fw_dma_dma_property_Spans,
	 _hrt_device_input_system_unis_logic_fw_dma_dma_property_SpanBanks,
	 _hrt_device_input_system_unis_logic_fw_dma_dma_property_Terminals,
	 _hrt_device_input_system_unis_logic_fw_dma_dma_property_TerminalBanks,
	 _hrt_device_input_system_unis_logic_fw_dma_dma_property_Channels,
	 _hrt_device_input_system_unis_logic_fw_dma_dma_property_ChannelBanks,
	 _hrt_device_input_system_unis_logic_fw_dma_dma_property_MaxMacroSize,
	 _hrt_device_input_system_unis_logic_fw_dma_dma_property_MaxInstructionsPending,
	 _hrt_device_input_system_unis_logic_fw_dma_dma_property_MaxOperationsPending,
	 _hrt_device_input_system_unis_logic_fw_dma_dma_property_MaxUnitWidth,
	 _hrt_device_input_system_unis_logic_fw_dma_dma_property_MaxUnitHeight,
	 _hrt_device_input_system_unis_logic_fw_dma_dma_property_MaxSpanWidth,
	 _hrt_device_input_system_unis_logic_fw_dma_dma_property_MaxSpanHeight,
	 _hrt_device_input_system_unis_logic_fw_dma_dma_property_MaxRegionWidth,
	 _hrt_device_input_system_unis_logic_fw_dma_dma_property_MaxRegionHeight,
	 _hrt_device_input_system_unis_logic_fw_dma_dma_property_MaxCompletedCount,
	 _hrt_device_input_system_unis_logic_fw_dma_dma_property_ElementInitDataBits,
	 _hrt_device_input_system_unis_logic_fw_dma_dma_property_GlobalSets,
	 _hrt_device_input_system_unis_logic_fw_dma_dma_property_MaxLinearBurstSize,
	 _hrt_device_input_system_unis_logic_fw_dma_dma_property_MaxBlockWidth,
	 _hrt_device_input_system_unis_logic_fw_dma_dma_property_MaxBlockHeight,
	 _hrt_device_input_system_unis_logic_fw_dma_dma_property_MaxPaddingAmount,
	 _hrt_device_input_system_unis_logic_fw_dma_dma_property_InstructionQueueDepth,
	 _hrt_device_input_system_unis_logic_fw_dma_dma_property_CommandQueueDepth,
	 _hrt_device_input_system_unis_logic_fw_dma_dma_property_CompletionQueueDepth,
	 1,
	 {0, 0},
	 {0, 1},
	 {0, 0},
	 {1, 1},
	 {1, 1},
	 {0, 11},
	 _hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_bus0_mt0_to_input_system_unis_logic_ctrl_bus_ab_mt_fw_dma_sl_in,
	 {input_system_unis_logic_fw_dma_dma_m0_data_width,
	 input_system_unis_logic_fw_dma_dma_m1_data_width},
	 input_system_unis_logic_fw_dma_dma_m2_data_width,
	 CTRLS_DATA_BITS,
	 0,
	 0},
	{
	 _hrt_device_input_system_unis_logic_dma_ext0_dma_property_NativeCIO2DBlock,
	 _hrt_device_input_system_unis_logic_dma_ext0_dma_property_NativeCIORAccept,
	 _hrt_device_input_system_unis_logic_dma_ext0_dma_property_NativeCIOSRMD,
	 _hrt_device_input_system_unis_logic_dma_ext0_dma_property_RequestBanks,
	 _hrt_device_input_system_unis_logic_dma_ext0_dma_property_Units,
	 _hrt_device_input_system_unis_logic_dma_ext0_dma_property_UnitBanks,
	 _hrt_device_input_system_unis_logic_dma_ext0_dma_property_Spans,
	 _hrt_device_input_system_unis_logic_dma_ext0_dma_property_SpanBanks,
	 _hrt_device_input_system_unis_logic_dma_ext0_dma_property_Terminals,
	 _hrt_device_input_system_unis_logic_dma_ext0_dma_property_TerminalBanks,
	 _hrt_device_input_system_unis_logic_dma_ext0_dma_property_Channels,
	 _hrt_device_input_system_unis_logic_dma_ext0_dma_property_ChannelBanks,
	 _hrt_device_input_system_unis_logic_dma_ext0_dma_property_MaxMacroSize,
	 _hrt_device_input_system_unis_logic_dma_ext0_dma_property_MaxInstructionsPending,
	 _hrt_device_input_system_unis_logic_dma_ext0_dma_property_MaxOperationsPending,
	 _hrt_device_input_system_unis_logic_dma_ext0_dma_property_MaxUnitWidth,
	 _hrt_device_input_system_unis_logic_dma_ext0_dma_property_MaxUnitHeight,
	 _hrt_device_input_system_unis_logic_dma_ext0_dma_property_MaxSpanWidth,
	 _hrt_device_input_system_unis_logic_dma_ext0_dma_property_MaxSpanHeight,
	 _hrt_device_input_system_unis_logic_dma_ext0_dma_property_MaxRegionWidth,
	 _hrt_device_input_system_unis_logic_dma_ext0_dma_property_MaxRegionHeight,
	 _hrt_device_input_system_unis_logic_dma_ext0_dma_property_MaxCompletedCount,
	 _hrt_device_input_system_unis_logic_dma_ext0_dma_property_ElementInitDataBits,
	 _hrt_device_input_system_unis_logic_dma_ext0_dma_property_GlobalSets,
	 _hrt_device_input_system_unis_logic_dma_ext0_dma_property_MaxLinearBurstSize,
	 _hrt_device_input_system_unis_logic_dma_ext0_dma_property_MaxBlockWidth,
	 _hrt_device_input_system_unis_logic_dma_ext0_dma_property_MaxBlockHeight,
	 _hrt_device_input_system_unis_logic_dma_ext0_dma_property_MaxPaddingAmount,
	 _hrt_device_input_system_unis_logic_dma_ext0_dma_property_InstructionQueueDepth,
	 _hrt_device_input_system_unis_logic_dma_ext0_dma_property_CommandQueueDepth,
	 _hrt_device_input_system_unis_logic_dma_ext0_dma_property_CompletionQueueDepth,
	 1,
	 {0, 0},
	 {0, 1},
	 {65536, 65536},
	 {2, 4},
	 {1, 1},
	 {0, 10},
	 _hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_bus0_mt0_to_input_system_unis_logic_ctrl_bus_ab_mt_dma_ext0_sl_in,
	 {input_system_unis_logic_dma_ext0_dma_m0_data_width,
	 input_system_unis_logic_dma_ext0_dma_m1_data_width},
	 input_system_unis_logic_dma_ext0_dma_m2_data_width,
	 CTRLS_DATA_BITS,
	 1,
	 1},
	{
	 _hrt_device_input_system_unis_logic_dma_ext1_dma_property_NativeCIO2DBlock,
	 _hrt_device_input_system_unis_logic_dma_ext1_dma_property_NativeCIORAccept,
	 _hrt_device_input_system_unis_logic_dma_ext1_dma_property_NativeCIOSRMD,
	 _hrt_device_input_system_unis_logic_dma_ext1_dma_property_RequestBanks,
	 _hrt_device_input_system_unis_logic_dma_ext1_dma_property_Units,
	 _hrt_device_input_system_unis_logic_dma_ext1_dma_property_UnitBanks,
	 _hrt_device_input_system_unis_logic_dma_ext1_dma_property_Spans,
	 _hrt_device_input_system_unis_logic_dma_ext1_dma_property_SpanBanks,
	 _hrt_device_input_system_unis_logic_dma_ext1_dma_property_Terminals,
	 _hrt_device_input_system_unis_logic_dma_ext1_dma_property_TerminalBanks,
	 _hrt_device_input_system_unis_logic_dma_ext1_dma_property_Channels,
	 _hrt_device_input_system_unis_logic_dma_ext1_dma_property_ChannelBanks,
	 _hrt_device_input_system_unis_logic_dma_ext1_dma_property_MaxMacroSize,
	 _hrt_device_input_system_unis_logic_dma_ext1_dma_property_MaxInstructionsPending,
	 _hrt_device_input_system_unis_logic_dma_ext1_dma_property_MaxOperationsPending,
	 _hrt_device_input_system_unis_logic_dma_ext1_dma_property_MaxUnitWidth,
	 _hrt_device_input_system_unis_logic_dma_ext1_dma_property_MaxUnitHeight,
	 _hrt_device_input_system_unis_logic_dma_ext1_dma_property_MaxSpanWidth,
	 _hrt_device_input_system_unis_logic_dma_ext1_dma_property_MaxSpanHeight,
	 _hrt_device_input_system_unis_logic_dma_ext1_dma_property_MaxRegionWidth,
	 _hrt_device_input_system_unis_logic_dma_ext1_dma_property_MaxRegionHeight,
	 _hrt_device_input_system_unis_logic_dma_ext1_dma_property_MaxCompletedCount,
	 _hrt_device_input_system_unis_logic_dma_ext1_dma_property_ElementInitDataBits,
	 _hrt_device_input_system_unis_logic_dma_ext1_dma_property_GlobalSets,
	 _hrt_device_input_system_unis_logic_dma_ext1_dma_property_MaxLinearBurstSize,
	 _hrt_device_input_system_unis_logic_dma_ext1_dma_property_MaxBlockWidth,
	 _hrt_device_input_system_unis_logic_dma_ext1_dma_property_MaxBlockHeight,
	 _hrt_device_input_system_unis_logic_dma_ext1_dma_property_MaxPaddingAmount,
	 _hrt_device_input_system_unis_logic_dma_ext1_dma_property_InstructionQueueDepth,
	 _hrt_device_input_system_unis_logic_dma_ext1_dma_property_CommandQueueDepth,
	 _hrt_device_input_system_unis_logic_dma_ext1_dma_property_CompletionQueueDepth,
	 1,
	 {0, 0},
	 {0, 1},
	 {65536, 65536},
	 {2, 4},
	 {1, 1},
	 {0, 10},
	 _hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_bus0_mt0_to_input_system_unis_logic_ctrl_bus_ab_mt_dma_ext1_sl_in,
	 {input_system_unis_logic_dma_ext1_dma_m0_data_width,
	 input_system_unis_logic_dma_ext1_dma_m1_data_width},
	 input_system_unis_logic_dma_ext1_dma_m2_data_width,
	 CTRLS_DATA_BITS,
	 1,
	 1},
	{
	 _hrt_device_input_system_is_a_logic_dma_property_NativeCIO2DBlock,
	 _hrt_device_input_system_is_a_logic_dma_property_NativeCIORAccept,
	 _hrt_device_input_system_is_a_logic_dma_property_NativeCIOSRMD,
	 _hrt_device_input_system_is_a_logic_dma_property_RequestBanks,
	 _hrt_device_input_system_is_a_logic_dma_property_Units,
	 _hrt_device_input_system_is_a_logic_dma_property_UnitBanks,
	 _hrt_device_input_system_is_a_logic_dma_property_Spans,
	 _hrt_device_input_system_is_a_logic_dma_property_SpanBanks,
	 _hrt_device_input_system_is_a_logic_dma_property_Terminals,
	 _hrt_device_input_system_is_a_logic_dma_property_TerminalBanks,
	 _hrt_device_input_system_is_a_logic_dma_property_Channels,
	 _hrt_device_input_system_is_a_logic_dma_property_ChannelBanks,
	 _hrt_device_input_system_is_a_logic_dma_property_MaxMacroSize,
	 _hrt_device_input_system_is_a_logic_dma_property_MaxInstructionsPending,
	 _hrt_device_input_system_is_a_logic_dma_property_MaxOperationsPending,
	 _hrt_device_input_system_is_a_logic_dma_property_MaxUnitWidth,
	 _hrt_device_input_system_is_a_logic_dma_property_MaxUnitHeight,
	 _hrt_device_input_system_is_a_logic_dma_property_MaxSpanWidth,
	 _hrt_device_input_system_is_a_logic_dma_property_MaxSpanHeight,
	 _hrt_device_input_system_is_a_logic_dma_property_MaxRegionWidth,
	 _hrt_device_input_system_is_a_logic_dma_property_MaxRegionHeight,
	 _hrt_device_input_system_is_a_logic_dma_property_MaxCompletedCount,
	 _hrt_device_input_system_is_a_logic_dma_property_ElementInitDataBits,
	 _hrt_device_input_system_is_a_logic_dma_property_GlobalSets,
	 _hrt_device_input_system_is_a_logic_dma_property_MaxLinearBurstSize,
	 _hrt_device_input_system_is_a_logic_dma_property_MaxBlockWidth,
	 _hrt_device_input_system_is_a_logic_dma_property_MaxBlockHeight,
	 _hrt_device_input_system_is_a_logic_dma_property_MaxPaddingAmount,
	 _hrt_device_input_system_is_a_logic_dma_property_InstructionQueueDepth,
	 _hrt_device_input_system_is_a_logic_dma_property_CommandQueueDepth,
	 _hrt_device_input_system_is_a_logic_dma_property_CompletionQueueDepth,
	 1,
	 {0, 0},
	 {0, 1},
	 {65536, 65536},
	 {2, 2},
	 {1, 1},
	 {0, 0},
	 _hrt_master_to_slave_address_input_system_unis_logic_mmu_at_system_ciopipe_mt2_cio_fifo_info_mt_to_input_system_unis_logic_ctrl_bus_ab_mt_is_a_sl_in
	 + _hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_ab_mt_is_a_mt_out_to_input_system_is_a_logic_dma_s0,
	 {input_system_is_a_logic_dma_m0_data_width,
	 input_system_is_a_logic_dma_m1_data_width},
	 input_system_is_a_logic_dma_m2_data_width,
	 CTRLS_DATA_BITS,
	 0,
	 1}
};

#else
// PSys DMA properties
const struct dma_prop_t dev_prop[VIED_NCI_N_DMA_DEV] = {
	{
	 _hrt_device_processing_system_dma_logic_dma_firmware_property_NativeCIO2DBlock,
	 _hrt_device_processing_system_dma_logic_dma_firmware_property_NativeCIORAccept,
	 _hrt_device_processing_system_dma_logic_dma_firmware_property_NativeCIOSRMD,
	 _hrt_device_processing_system_dma_logic_dma_firmware_property_RequestBanks,
	 _hrt_device_processing_system_dma_logic_dma_firmware_property_Units,
	 _hrt_device_processing_system_dma_logic_dma_firmware_property_UnitBanks,
	 _hrt_device_processing_system_dma_logic_dma_firmware_property_Spans,
	 _hrt_device_processing_system_dma_logic_dma_firmware_property_SpanBanks,
	 _hrt_device_processing_system_dma_logic_dma_firmware_property_Terminals,
	 _hrt_device_processing_system_dma_logic_dma_firmware_property_TerminalBanks,
	 _hrt_device_processing_system_dma_logic_dma_firmware_property_Channels,
	 _hrt_device_processing_system_dma_logic_dma_firmware_property_ChannelBanks,
	 _hrt_device_processing_system_dma_logic_dma_firmware_property_MaxMacroSize,
	 _hrt_device_processing_system_dma_logic_dma_firmware_property_MaxInstructionsPending,
	 _hrt_device_processing_system_dma_logic_dma_firmware_property_MaxOperationsPending,
	 _hrt_device_processing_system_dma_logic_dma_firmware_property_MaxUnitWidth,
	 _hrt_device_processing_system_dma_logic_dma_firmware_property_MaxUnitHeight,
	 _hrt_device_processing_system_dma_logic_dma_firmware_property_MaxSpanWidth,
	 _hrt_device_processing_system_dma_logic_dma_firmware_property_MaxSpanHeight,
	 _hrt_device_processing_system_dma_logic_dma_firmware_property_MaxRegionWidth,
	 _hrt_device_processing_system_dma_logic_dma_firmware_property_MaxRegionHeight,
	 _hrt_device_processing_system_dma_logic_dma_firmware_property_MaxCompletedCount,
	 _hrt_device_processing_system_dma_logic_dma_firmware_property_ElementInitDataBits,
	 _hrt_device_processing_system_dma_logic_dma_firmware_property_GlobalSets,
	 _hrt_device_processing_system_dma_logic_dma_firmware_property_MaxLinearBurstSize,
	 _hrt_device_processing_system_dma_logic_dma_firmware_property_MaxBlockWidth,
	 _hrt_device_processing_system_dma_logic_dma_firmware_property_MaxBlockHeight,
	 _hrt_device_processing_system_dma_logic_dma_firmware_property_MaxPaddingAmount,
	 _hrt_device_processing_system_dma_logic_dma_firmware_property_InstructionQueueDepth,
	 _hrt_device_processing_system_dma_logic_dma_firmware_property_CommandQueueDepth,
	 _hrt_device_processing_system_dma_logic_dma_firmware_property_CompletionQueueDepth,
	 1,
	 {0, 0},
	 {0, 1},
	 {0, 0},
	 {1, 1},
	 {1, 1},
	 {0, 11},
	 _hrt_master_to_slave_address_host_op0_to_processing_system_unps_logic_configbus_ab_mt_dmae1fw_sl_in
	 +
	 _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_dmae1fw_mt_out_to_processing_system_dma_logic_dma_firmware_s0,
	 {processing_system_dma_logic_dma_firmware_m0_data_width,
	  processing_system_dma_logic_dma_firmware_m1_data_width},
	 processing_system_dma_logic_dma_firmware_m2_data_width,
	 CTRLS_DATA_BITS,
	 0,
	 0},
	{
	 _hrt_device_processing_system_dma_logic_dma_external0_property_NativeCIO2DBlock,
	 _hrt_device_processing_system_dma_logic_dma_external0_property_NativeCIORAccept,
	 _hrt_device_processing_system_dma_logic_dma_external0_property_NativeCIOSRMD,
	 _hrt_device_processing_system_dma_logic_dma_external0_property_RequestBanks,
	 _hrt_device_processing_system_dma_logic_dma_external0_property_Units,
	 _hrt_device_processing_system_dma_logic_dma_external0_property_UnitBanks,
	 _hrt_device_processing_system_dma_logic_dma_external0_property_Spans,
	 _hrt_device_processing_system_dma_logic_dma_external0_property_SpanBanks,
	 _hrt_device_processing_system_dma_logic_dma_external0_property_Terminals,
	 _hrt_device_processing_system_dma_logic_dma_external0_property_TerminalBanks,
	 _hrt_device_processing_system_dma_logic_dma_external0_property_Channels,
	 _hrt_device_processing_system_dma_logic_dma_external0_property_ChannelBanks,
	 _hrt_device_processing_system_dma_logic_dma_external0_property_MaxMacroSize,
	 _hrt_device_processing_system_dma_logic_dma_external0_property_MaxInstructionsPending,
	 _hrt_device_processing_system_dma_logic_dma_external0_property_MaxOperationsPending,
	 _hrt_device_processing_system_dma_logic_dma_external0_property_MaxUnitWidth,
	 _hrt_device_processing_system_dma_logic_dma_external0_property_MaxUnitHeight,
	 _hrt_device_processing_system_dma_logic_dma_external0_property_MaxSpanWidth,
	 _hrt_device_processing_system_dma_logic_dma_external0_property_MaxSpanHeight,
	 _hrt_device_processing_system_dma_logic_dma_external0_property_MaxRegionWidth,
	 _hrt_device_processing_system_dma_logic_dma_external0_property_MaxRegionHeight,
	 _hrt_device_processing_system_dma_logic_dma_external0_property_MaxCompletedCount,
	 _hrt_device_processing_system_dma_logic_dma_external0_property_ElementInitDataBits,
	 _hrt_device_processing_system_dma_logic_dma_external0_property_GlobalSets,
	 _hrt_device_processing_system_dma_logic_dma_external0_property_MaxLinearBurstSize,
	 _hrt_device_processing_system_dma_logic_dma_external0_property_MaxBlockWidth,
	 _hrt_device_processing_system_dma_logic_dma_external0_property_MaxBlockHeight,
	 _hrt_device_processing_system_dma_logic_dma_external0_property_MaxPaddingAmount,
	 _hrt_device_processing_system_dma_logic_dma_external0_property_InstructionQueueDepth,
	 _hrt_device_processing_system_dma_logic_dma_external0_property_CommandQueueDepth,
	 _hrt_device_processing_system_dma_logic_dma_external0_property_CompletionQueueDepth,
	 4,
	 {0, 0},
	 {0, 1},
	 {65536, 65536},
	 {2, 4},
	 {2, 3},
	 {0, 10},
	 _hrt_master_to_slave_address_host_op0_to_processing_system_unps_logic_configbus_ab_mt_dmae0i_sl_in
	 +
	 _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_dmae0i_mt_out_to_processing_system_dma_logic_dma_external0_s0,
	 {processing_system_dma_logic_dma_external0_m0_data_width,
	  processing_system_dma_logic_dma_external0_m1_data_width},
	 processing_system_dma_logic_dma_external0_m2_data_width,
	 CTRLS_DATA_BITS,
	 1,
	 1},
	{
	 _hrt_device_processing_system_dma_logic_dma_external1r_property_NativeCIO2DBlock,
	 _hrt_device_processing_system_dma_logic_dma_external1r_property_NativeCIORAccept,
	 _hrt_device_processing_system_dma_logic_dma_external1r_property_NativeCIOSRMD,
	 _hrt_device_processing_system_dma_logic_dma_external1r_property_RequestBanks,
	 _hrt_device_processing_system_dma_logic_dma_external1r_property_Units,
	 _hrt_device_processing_system_dma_logic_dma_external1r_property_UnitBanks,
	 _hrt_device_processing_system_dma_logic_dma_external1r_property_Spans,
	 _hrt_device_processing_system_dma_logic_dma_external1r_property_SpanBanks,
	 _hrt_device_processing_system_dma_logic_dma_external1r_property_Terminals,
	 _hrt_device_processing_system_dma_logic_dma_external1r_property_TerminalBanks,
	 _hrt_device_processing_system_dma_logic_dma_external1r_property_Channels,
	 _hrt_device_processing_system_dma_logic_dma_external1r_property_ChannelBanks,
	 _hrt_device_processing_system_dma_logic_dma_external1r_property_MaxMacroSize,
	 _hrt_device_processing_system_dma_logic_dma_external1r_property_MaxInstructionsPending,
	 _hrt_device_processing_system_dma_logic_dma_external1r_property_MaxOperationsPending,
	 _hrt_device_processing_system_dma_logic_dma_external1r_property_MaxUnitWidth,
	 _hrt_device_processing_system_dma_logic_dma_external1r_property_MaxUnitHeight,
	 _hrt_device_processing_system_dma_logic_dma_external1r_property_MaxSpanWidth,
	 _hrt_device_processing_system_dma_logic_dma_external1r_property_MaxSpanHeight,
	 _hrt_device_processing_system_dma_logic_dma_external1r_property_MaxRegionWidth,
	 _hrt_device_processing_system_dma_logic_dma_external1r_property_MaxRegionHeight,
	 _hrt_device_processing_system_dma_logic_dma_external1r_property_MaxCompletedCount,
	 _hrt_device_processing_system_dma_logic_dma_external1r_property_ElementInitDataBits,
	 _hrt_device_processing_system_dma_logic_dma_external1r_property_GlobalSets,
	 _hrt_device_processing_system_dma_logic_dma_external1r_property_MaxLinearBurstSize,
	 _hrt_device_processing_system_dma_logic_dma_external1r_property_MaxBlockWidth,
	 _hrt_device_processing_system_dma_logic_dma_external1r_property_MaxBlockHeight,
	 _hrt_device_processing_system_dma_logic_dma_external1r_property_MaxPaddingAmount,
	 _hrt_device_processing_system_dma_logic_dma_external1r_property_InstructionQueueDepth,
	 _hrt_device_processing_system_dma_logic_dma_external1r_property_CommandQueueDepth,
	 _hrt_device_processing_system_dma_logic_dma_external1r_property_CompletionQueueDepth,
	 4,
	 {0, 0},
	 {0, 1},
	 {65536, 65536},
	 {2, 4},
	 {2, 3},
	 {0, 10},
	 _hrt_master_to_slave_address_host_op0_to_processing_system_unps_logic_configbus_ab_mt_dmae1fw_sl_in
	 +
	 _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_dmae1fw_mt_out_to_processing_system_dma_logic_dma_external1r_s0,
	 {processing_system_dma_logic_dma_external1r_m0_data_width,
	  processing_system_dma_logic_dma_external1r_m1_data_width},
	 processing_system_dma_logic_dma_external1r_m2_data_width,
	 CTRLS_DATA_BITS,
	 1,
	 1},
	{
	 _hrt_device_processing_system_dma_logic_dma_external1_property_NativeCIO2DBlock,
	 _hrt_device_processing_system_dma_logic_dma_external1_property_NativeCIORAccept,
	 _hrt_device_processing_system_dma_logic_dma_external1_property_NativeCIOSRMD,
	 _hrt_device_processing_system_dma_logic_dma_external1_property_RequestBanks,
	 _hrt_device_processing_system_dma_logic_dma_external1_property_Units,
	 _hrt_device_processing_system_dma_logic_dma_external1_property_UnitBanks,
	 _hrt_device_processing_system_dma_logic_dma_external1_property_Spans,
	 _hrt_device_processing_system_dma_logic_dma_external1_property_SpanBanks,
	 _hrt_device_processing_system_dma_logic_dma_external1_property_Terminals,
	 _hrt_device_processing_system_dma_logic_dma_external1_property_TerminalBanks,
	 _hrt_device_processing_system_dma_logic_dma_external1_property_Channels,
	 _hrt_device_processing_system_dma_logic_dma_external1_property_ChannelBanks,
	 _hrt_device_processing_system_dma_logic_dma_external1_property_MaxMacroSize,
	 _hrt_device_processing_system_dma_logic_dma_external1_property_MaxInstructionsPending,
	 _hrt_device_processing_system_dma_logic_dma_external1_property_MaxOperationsPending,
	 _hrt_device_processing_system_dma_logic_dma_external1_property_MaxUnitWidth,
	 _hrt_device_processing_system_dma_logic_dma_external1_property_MaxUnitHeight,
	 _hrt_device_processing_system_dma_logic_dma_external1_property_MaxSpanWidth,
	 _hrt_device_processing_system_dma_logic_dma_external1_property_MaxSpanHeight,
	 _hrt_device_processing_system_dma_logic_dma_external1_property_MaxRegionWidth,
	 _hrt_device_processing_system_dma_logic_dma_external1_property_MaxRegionHeight,
	 _hrt_device_processing_system_dma_logic_dma_external1_property_MaxCompletedCount,
	 _hrt_device_processing_system_dma_logic_dma_external1_property_ElementInitDataBits,
	 _hrt_device_processing_system_dma_logic_dma_external1_property_GlobalSets,
	 _hrt_device_processing_system_dma_logic_dma_external1_property_MaxLinearBurstSize,
	 _hrt_device_processing_system_dma_logic_dma_external1_property_MaxBlockWidth,
	 _hrt_device_processing_system_dma_logic_dma_external1_property_MaxBlockHeight,
	 _hrt_device_processing_system_dma_logic_dma_external1_property_MaxPaddingAmount,
	 _hrt_device_processing_system_dma_logic_dma_external1_property_InstructionQueueDepth,
	 _hrt_device_processing_system_dma_logic_dma_external1_property_CommandQueueDepth,
	 _hrt_device_processing_system_dma_logic_dma_external1_property_CompletionQueueDepth,
	 4,
	 {0, 0},
	 {0, 1},
	 {65536, 65536},
	 {2, 4},
	 {2, 3},
	 {0, 10},
	 _hrt_master_to_slave_address_host_op0_to_processing_system_unps_logic_configbus_ab_mt_dmae1fw_sl_in
	 +
	 _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_dmae1fw_mt_out_to_processing_system_dma_logic_dma_external1_s0,
	 {processing_system_dma_logic_dma_external1_m0_data_width,
	  processing_system_dma_logic_dma_external1_m1_data_width},
	 processing_system_dma_logic_dma_external1_m2_data_width,
	 CTRLS_DATA_BITS,
	 1,
	 1},
	{
	 _hrt_device_processing_system_dma_logic_dma_internal_property_NativeCIO2DBlock,
	 _hrt_device_processing_system_dma_logic_dma_internal_property_NativeCIORAccept,
	 _hrt_device_processing_system_dma_logic_dma_internal_property_NativeCIOSRMD,
	 _hrt_device_processing_system_dma_logic_dma_internal_property_RequestBanks,
	 _hrt_device_processing_system_dma_logic_dma_internal_property_Units,
	 _hrt_device_processing_system_dma_logic_dma_internal_property_UnitBanks,
	 _hrt_device_processing_system_dma_logic_dma_internal_property_Spans,
	 _hrt_device_processing_system_dma_logic_dma_internal_property_SpanBanks,
	 _hrt_device_processing_system_dma_logic_dma_internal_property_Terminals,
	 _hrt_device_processing_system_dma_logic_dma_internal_property_TerminalBanks,
	 _hrt_device_processing_system_dma_logic_dma_internal_property_Channels,
	 _hrt_device_processing_system_dma_logic_dma_internal_property_ChannelBanks,
	 _hrt_device_processing_system_dma_logic_dma_internal_property_MaxMacroSize,
	 _hrt_device_processing_system_dma_logic_dma_internal_property_MaxInstructionsPending,
	 _hrt_device_processing_system_dma_logic_dma_internal_property_MaxOperationsPending,
	 _hrt_device_processing_system_dma_logic_dma_internal_property_MaxUnitWidth,
	 _hrt_device_processing_system_dma_logic_dma_internal_property_MaxUnitHeight,
	 _hrt_device_processing_system_dma_logic_dma_internal_property_MaxSpanWidth,
	 _hrt_device_processing_system_dma_logic_dma_internal_property_MaxSpanHeight,
	 _hrt_device_processing_system_dma_logic_dma_internal_property_MaxRegionWidth,
	 _hrt_device_processing_system_dma_logic_dma_internal_property_MaxRegionHeight,
	 _hrt_device_processing_system_dma_logic_dma_internal_property_MaxCompletedCount,
	 _hrt_device_processing_system_dma_logic_dma_internal_property_ElementInitDataBits,
	 _hrt_device_processing_system_dma_logic_dma_internal_property_GlobalSets,
	 _hrt_device_processing_system_dma_logic_dma_internal_property_MaxLinearBurstSize,
	 _hrt_device_processing_system_dma_logic_dma_internal_property_MaxBlockWidth,
	 _hrt_device_processing_system_dma_logic_dma_internal_property_MaxBlockHeight,
	 _hrt_device_processing_system_dma_logic_dma_internal_property_MaxPaddingAmount,
	 _hrt_device_processing_system_dma_logic_dma_internal_property_InstructionQueueDepth,
	 _hrt_device_processing_system_dma_logic_dma_internal_property_CommandQueueDepth,
	 _hrt_device_processing_system_dma_logic_dma_internal_property_CompletionQueueDepth,
	 1,
	 {0, 0},
	 {0, 0},
	 {65536, 65536},
	 {2, 2},
	 {1, 1},
	 {0, 0},
	 _hrt_master_to_slave_address_host_op0_to_processing_system_unps_logic_configbus_ab_mt_dmae0i_sl_in
	 +
	 _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_dmae0i_mt_out_to_processing_system_dma_logic_dma_internal_s0,
	 {processing_system_dma_logic_dma_internal_m0_data_width,
	  processing_system_dma_logic_dma_internal_m1_data_width},
	 processing_system_dma_logic_dma_internal_m2_data_width,
	 CTRLS_DATA_BITS,
	 1,
	 1},
	{
	 _hrt_device_processing_system_input_slice_light_logic_dma_property_NativeCIO2DBlock,
	 _hrt_device_processing_system_input_slice_light_logic_dma_property_NativeCIORAccept,
	 _hrt_device_processing_system_input_slice_light_logic_dma_property_NativeCIOSRMD,
	 _hrt_device_processing_system_input_slice_light_logic_dma_property_RequestBanks,
	 _hrt_device_processing_system_input_slice_light_logic_dma_property_Units,
	 _hrt_device_processing_system_input_slice_light_logic_dma_property_UnitBanks,
	 _hrt_device_processing_system_input_slice_light_logic_dma_property_Spans,
	 _hrt_device_processing_system_input_slice_light_logic_dma_property_SpanBanks,
	 _hrt_device_processing_system_input_slice_light_logic_dma_property_Terminals,
	 _hrt_device_processing_system_input_slice_light_logic_dma_property_TerminalBanks,
	 _hrt_device_processing_system_input_slice_light_logic_dma_property_Channels,
	 _hrt_device_processing_system_input_slice_light_logic_dma_property_ChannelBanks,
	 _hrt_device_processing_system_input_slice_light_logic_dma_property_MaxMacroSize,
	 _hrt_device_processing_system_input_slice_light_logic_dma_property_MaxInstructionsPending,
	 _hrt_device_processing_system_input_slice_light_logic_dma_property_MaxOperationsPending,
	 _hrt_device_processing_system_input_slice_light_logic_dma_property_MaxUnitWidth,
	 _hrt_device_processing_system_input_slice_light_logic_dma_property_MaxUnitHeight,
	 _hrt_device_processing_system_input_slice_light_logic_dma_property_MaxSpanWidth,
	 _hrt_device_processing_system_input_slice_light_logic_dma_property_MaxSpanHeight,
	 _hrt_device_processing_system_input_slice_light_logic_dma_property_MaxRegionWidth,
	 _hrt_device_processing_system_input_slice_light_logic_dma_property_MaxRegionHeight,
	 _hrt_device_processing_system_input_slice_light_logic_dma_property_MaxCompletedCount,
	 _hrt_device_processing_system_input_slice_light_logic_dma_property_ElementInitDataBits,
	 _hrt_device_processing_system_input_slice_light_logic_dma_property_GlobalSets,
	 _hrt_device_processing_system_input_slice_light_logic_dma_property_MaxLinearBurstSize,
	 _hrt_device_processing_system_input_slice_light_logic_dma_property_MaxBlockWidth,
	 _hrt_device_processing_system_input_slice_light_logic_dma_property_MaxBlockHeight,
	 _hrt_device_processing_system_input_slice_light_logic_dma_property_MaxPaddingAmount,
	 _hrt_device_processing_system_input_slice_light_logic_dma_property_InstructionQueueDepth,
	 _hrt_device_processing_system_input_slice_light_logic_dma_property_CommandQueueDepth,
	 _hrt_device_processing_system_input_slice_light_logic_dma_property_CompletionQueueDepth,
	 1,
	 {0, 0},
	 {0, 0},
	 {65536, 65536},
	 {2, 2},
	 {1, 1},
	 {0, 0},
	 _hrt_master_to_slave_address_processing_system_unps_logic_configbus_bus_P0_spp0_mt_ps_to_processing_system_input_slice_light_logic_dma_s0,
	 {processing_system_input_slice_light_logic_dma_m0_data_width,
	  processing_system_input_slice_light_logic_dma_m1_data_width},
	 processing_system_input_slice_light_logic_dma_m2_data_width,
	 CTRLS_DATA_BITS,
	 0,
	 1},
	{
	 _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_NativeCIO2DBlock,
	 _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_NativeCIORAccept,
	 _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_NativeCIOSRMD,
	 _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_RequestBanks,
	 _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_Units,
	 _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_UnitBanks,
	 _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_Spans,
	 _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_SpanBanks,
	 _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_Terminals,
	 _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_TerminalBanks,
	 _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_Channels,
	 _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_ChannelBanks,
	 _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_MaxMacroSize,
	 _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_MaxInstructionsPending,
	 _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_MaxOperationsPending,
	 _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_MaxUnitWidth,
	 _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_MaxUnitHeight,
	 _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_MaxSpanWidth,
	 _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_MaxSpanHeight,
	 _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_MaxRegionWidth,
	 _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_MaxRegionHeight,
	 _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_MaxCompletedCount,
	 _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_ElementInitDataBits,
	 _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_GlobalSets,
	 _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_MaxLinearBurstSize,
	 _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_MaxBlockWidth,
	 _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_MaxBlockHeight,
	 _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_MaxPaddingAmount,
	 _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_InstructionQueueDepth,
	 _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_CommandQueueDepth,
	 _hrt_device_processing_system_unps_logic_ipfd_dma4_inst_property_CompletionQueueDepth,
	 1,
	 {0, 0},
	 {0, 0},
	 {65536, 65536},
	 {2, 2},
	 {1, 1},
	 {0, 0},
	 _hrt_master_to_slave_address_processing_system_unps_logic_configbus_bus_P0_spp0_mt_ps_to_processing_system_unps_logic_ipfd_dma4_inst_s0,
	 {processing_system_unps_logic_ipfd_dma4_inst_mA_data_width,
	 processing_system_unps_logic_ipfd_dma4_inst_mB_data_width},
	 processing_system_unps_logic_ipfd_dma4_inst_mC_data_width,
	 CTRLS_DATA_BITS,
	 1,
	 1}
};
#endif


 /**********************************
  * slave address decomposition
  **********************************/
static inline uint32_t byte_index_bits(const uint32_t data_bits)
{
	return vied_minimum_bits(vied_bitpos_to_bytepos(data_bits));
}

static inline uint32_t
register_index_bits(const uint32_t request_regs, const uint32_t unit_regs,
		    const uint32_t span_regs, const uint32_t terminal_regs,
		    const uint32_t channel_regs, const uint32_t master_regs,
		    const uint32_t global_regs)
{
	const uint32_t max_regs = MAX(request_regs,
				      MAX(unit_regs,
					  MAX(span_regs,
					      MAX(terminal_regs,
						  MAX(channel_regs,
						      MAX(master_regs,
							  global_regs))))));
	return vied_minimum_bits(max_regs);
}

static inline uint32_t
bank_index_bits(const uint32_t request_banks, const uint32_t unit_banks,
		const uint32_t span_banks, const uint32_t terminal_banks,
		const uint32_t channel_banks, const uint32_t master_banks,
		const uint32_t global_banks)
{
	const uint32_t max_banks = MAX(request_banks,
				       MAX(unit_banks,
					   MAX(span_banks,
					       MAX(terminal_banks,
						   MAX(channel_banks,
						       MAX(master_banks,
							   global_banks))))));
	return vied_minimum_bits(max_banks);
}

uint32_t prop_get_dma_register_id_idx(const enum vied_nci_dma_dev_id dev_id)
{
	const struct dma_prop_t *prop = NULL;
	if (dev_id >= VIED_NCI_N_DMA_DEV) {
		return 0;
	}

	prop = &dev_prop[dev_id];

	return byte_index_bits(prop->ControlSlaveDataWidth);
}

uint32_t prop_get_dma_bank_id_idx(const enum vied_nci_dma_dev_id dev_id)
{
	const struct dma_prop_t *prop = NULL;
	uint32_t byte_bits = 0;

	if (dev_id >= VIED_NCI_N_DMA_DEV) {
		return 0;
	}

	prop = &dev_prop[dev_id];

	byte_bits = byte_index_bits(prop->ControlSlaveDataWidth);

	return byte_bits + register_index_bits(REQUEST_REGS, UNIT_REGS,
					       SPAN_REGS, TERMINAL_REGS,
					       CHANNEL_REGS, MASTER_REGS,
					       7 + 2 * prop->GlobalSets);
}

uint32_t prop_get_dma_group_id_idx(const enum vied_nci_dma_dev_id dev_id)
{
	const struct dma_prop_t *prop = NULL;
	uint32_t byte_bits = 0;
	uint32_t register_bits = 0;

	if (dev_id >= VIED_NCI_N_DMA_DEV) {
		return 0;
	}

	prop = &dev_prop[dev_id];

	byte_bits = byte_index_bits(prop->ControlSlaveDataWidth);

	register_bits = register_index_bits(REQUEST_REGS, UNIT_REGS, SPAN_REGS,
					    TERMINAL_REGS, CHANNEL_REGS,
					    MASTER_REGS, 7 + 2 * prop->GlobalSets);

	return byte_bits + register_bits + bank_index_bits(prop->RequestBanks,
							   prop->UnitBanks,
							   prop->SpanBanks,
							   prop->TerminalBanks,
							   prop->ChannelBanks,
							   MASTER_BANKS,
							   GLOBAL_BANKS);
}

struct vied_nci_dma_terminal_desc_bits_t
prop_get_terminal_desc_bits(const enum vied_nci_dma_dev_id dev_id)
{
	const struct dma_prop_t *prop = NULL;
	struct vied_nci_dma_terminal_desc_bits_t reg;

	memset(&reg, 0, sizeof(reg));

	if (dev_id >= VIED_NCI_N_DMA_DEV) {
		return reg;
	}

	prop = &dev_prop[dev_id];

	reg.region_origin_bits = MAX(DATAMA_ADDR_BITS, DATAMB_ADDR_BITS);
	reg.region_width_bits = vied_minimum_bits(prop->MaxRegionWidth);
	reg.region_stride_bits =
	    vied_minimum_bits(MAX
			      (prop->MaxRegionStride[0], prop->MaxRegionStride[1] + 1));
	reg.element_setup_bits =
	    vied_minimum_bits(MAX
			      (prop->ElementPrecisions[0],
			       prop->ElementPrecisions[1]));
	reg.cio_info_setup_bits = MAX(prop->InfoWidth[0], prop->InfoWidth[1]);
	reg.port_mode_bits = PORT_MODE_BITS;

	reg.desc_words = (vied_bitpos_to_bytepos(reg.region_origin_bits) +
			  vied_bitpos_to_bytepos(reg.region_width_bits) +
			  vied_bitpos_to_bytepos(reg.region_stride_bits) +
			  vied_bitpos_to_bytepos(reg.element_setup_bits) +
			  vied_bitpos_to_bytepos(reg.cio_info_setup_bits) +
			  vied_bitpos_to_bytepos(reg.port_mode_bits) +
			  CTRLM_DATA_BYTES - 1) / CTRLM_DATA_BYTES;

	return reg;
}

struct vied_nci_dma_span_desc_bits_t
prop_get_span_desc_bits(const enum vied_nci_dma_dev_id dev_id)
{
	const struct dma_prop_t *prop = NULL;
	struct vied_nci_dma_span_desc_bits_t reg;

	memset(&reg, 0, sizeof(reg));

	if (dev_id >= VIED_NCI_N_DMA_DEV) {
		return reg;
	}

	prop = &dev_prop[dev_id];

	reg.x_coordinate_bits = vied_minimum_bits(
			prop->MaxPaddingAmount * prop->MaxSubSamplingFactor
			+ prop->MaxRegionWidth)+1;
	reg.y_coordinate_bits = vied_minimum_bits(prop->MaxRegionHeight);
	reg.unit_location_bits = MAX(MAX(DATAMA_ADDR_BITS, DATAMB_ADDR_BITS),
				     reg.x_coordinate_bits + reg.y_coordinate_bits);
	reg.span_column_bits = vied_minimum_bits(prop->MaxSpanWidth);
	reg.span_row_bits = vied_minimum_bits(prop->MaxSpanHeight);
	reg.span_width_bits = vied_minimum_bits(prop->MaxSpanWidth);
	reg.span_height_bits = vied_minimum_bits(prop->MaxSpanHeight);
	reg.span_mode_bits = SPAN_MODE_BITS;

	reg.desc_words = (vied_bitpos_to_bytepos(reg.unit_location_bits) +
			  vied_bitpos_to_bytepos(reg.span_column_bits) +
			  vied_bitpos_to_bytepos(reg.span_row_bits) +
			  vied_bitpos_to_bytepos(reg.span_width_bits) +
			  vied_bitpos_to_bytepos(reg.span_height_bits) +
			  vied_bitpos_to_bytepos(reg.span_mode_bits) +
			  CTRLM_DATA_BYTES - 1) / CTRLM_DATA_BYTES;

	return reg;
}

struct vied_nci_dma_channel_desc_bits_t
prop_get_channel_desc_bits(const enum vied_nci_dma_dev_id dev_id)
{
	const struct dma_prop_t *prop = NULL;
	struct vied_nci_dma_channel_desc_bits_t reg;

	memset(&reg, 0, sizeof(reg));

	if (dev_id >= VIED_NCI_N_DMA_DEV) {
		return reg;
	}

	prop = &dev_prop[dev_id];
	reg.element_extend_mode_bits = ELEMENT_EXTEND_MODE_BITS;
	reg.element_init_data_bits = prop->ElementInitDataBits;
	reg.padding_mode_bits = PADDING_MODE_BITS;
	reg.sampling_setup_bits =
	    vied_minimum_bits(MAX
			      ((prop->SubSamplingFactors[0]),
			       prop->SubSamplingFactors[1]));
	reg.global_set_id_bits = vied_minimum_bits(prop->GlobalSets);
	reg.ack_mode_bits = vied_minimum_bits((prop->MaxCompletedCount + 1));
	reg.ack_addr_bits = ACK_ADDR_BITS;
	reg.ack_data_bits = ACK_DATA_BITS;
	reg.completed_count_bits =
	    1 + vied_minimum_bits((prop->MaxCompletedCount + 1));

	reg.desc_words = (vied_bitpos_to_bytepos(reg.element_extend_mode_bits) +
			  vied_bitpos_to_bytepos(reg.element_init_data_bits) +
			  vied_bitpos_to_bytepos(reg.padding_mode_bits) +
			  vied_bitpos_to_bytepos(reg.sampling_setup_bits) +
			  vied_bitpos_to_bytepos(reg.global_set_id_bits) +
			  vied_bitpos_to_bytepos(reg.ack_mode_bits) +
			  vied_bitpos_to_bytepos(reg.ack_addr_bits) +
			  vied_bitpos_to_bytepos(reg.ack_data_bits) +
			  vied_bitpos_to_bytepos(reg.completed_count_bits) +
			  CTRLM_DATA_BYTES - 1) / CTRLM_DATA_BYTES;

	return reg;
}

struct vied_nci_dma_unit_desc_bits_t
prop_get_unit_desc_bits(const enum vied_nci_dma_dev_id dev_id)
{
	const struct dma_prop_t *prop = NULL;
	struct vied_nci_dma_unit_desc_bits_t reg;

	memset(&reg, 0, sizeof(reg));

	if (dev_id >= VIED_NCI_N_DMA_DEV) {
		return reg;
	}

	prop = &dev_prop[dev_id];
	reg.unit_width_bits = vied_minimum_bits(prop->MaxUnitWidth);
	reg.unit_height_bits = vied_minimum_bits(prop->MaxUnitHeight);

	reg.desc_words = (vied_bitpos_to_bytepos(reg.unit_width_bits) +
			  vied_bitpos_to_bytepos(reg.unit_height_bits) +
			  CTRLM_DATA_BYTES - 1) / CTRLM_DATA_BYTES;

	return reg;
}

struct vied_nci_dma_request_desc_bits_t
prop_get_request_desc_bits(const enum vied_nci_dma_dev_id dev_id)
{
	const struct dma_prop_t *prop = NULL;
	struct vied_nci_dma_request_desc_bits_t reg;

	memset(&reg, 0, sizeof(reg));

	if (dev_id >= VIED_NCI_N_DMA_DEV) {
		return reg;
	}

	prop = &dev_prop[dev_id];
	reg.macro_size_bits = vied_minimum_bits(prop->MaxMacroSize);
	reg.descriptor_id_bits = vied_minimum_bits(MAX(prop->Units,
					   MAX(prop->Spans,
					       MAX(prop->Terminals,
						   prop->Channels))));

	reg.terminal_bits = vied_minimum_bits(prop->Terminals);
	reg.span_bits = vied_minimum_bits(prop->Spans);
	reg.channel_bits = vied_minimum_bits(prop->Channels);
	reg.unit_bits = vied_minimum_bits(prop->Units);

	return reg;
}
