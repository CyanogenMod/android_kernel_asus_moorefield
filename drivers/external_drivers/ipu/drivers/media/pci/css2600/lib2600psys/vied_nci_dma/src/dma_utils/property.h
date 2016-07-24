#ifndef _PROPERTY_H_
#define _PROPERTY_H_

#include "type_support.h"
#include "vied_nci_dma_local.h"
#include "vied_nci_dma.h"

struct dma_prop_t {
	uint8_t  NativeCIO2DBlock;
	uint8_t  NativeCIORAccept;
	uint8_t  NativeCIOSRMD;
	uint8_t  RequestBanks;
	uint8_t  Units;
	uint8_t  UnitBanks;
	uint8_t  Spans;
	uint8_t  SpanBanks;
	uint8_t  Terminals;
	uint8_t  TerminalBanks;
	uint8_t  Channels;
	uint8_t  ChannelBanks;
	uint32_t MaxMacroSize;
	uint16_t MaxInstructionsPending;
	uint16_t MaxOperationsPending;
	uint32_t MaxUnitWidth;
	uint32_t MaxUnitHeight;
	uint32_t MaxSpanWidth;
	uint32_t MaxSpanHeight;
	uint32_t MaxRegionWidth;
	uint32_t MaxRegionHeight;
	uint8_t  MaxCompletedCount;
	uint8_t  ElementInitDataBits;
	uint8_t  GlobalSets;
	uint32_t MaxLinearBurstSize;
	uint32_t MaxBlockWidth;
	uint32_t MaxBlockHeight;
	uint8_t  MaxPaddingAmount;
	uint8_t  InstructionQueueDepth;
	uint8_t  CommandQueueDepth;
	uint8_t  CompletionQueueDepth;
	uint8_t  MaxSubSamplingFactor;
	uint8_t  SRMDSupport[MAX_MASTER_BANKS];
	uint8_t  BurstSupport[MAX_MASTER_BANKS];
	uint32_t MaxRegionStride[MAX_MASTER_BANKS];
	uint8_t  ElementPrecisions[MAX_MASTER_BANKS];
	uint8_t  SubSamplingFactors[MAX_MASTER_BANKS];
	uint8_t  InfoWidth[MAX_MASTER_BANKS];
	uint32_t BaseAddress;
	uint16_t DataMasterDataWidth[MAX_MASTER_BANKS];
	uint8_t  ControlMasterDataWidth;
	uint8_t  ControlSlaveDataWidth;
	uint8_t  BankModeRegisterAvailable;
	uint8_t  RegionStrideRegisterAvailable;
};

extern const struct dma_prop_t dev_prop[VIED_NCI_N_DMA_DEV];

uint32_t prop_get_dma_register_id_idx(enum vied_nci_dma_dev_id dev_id);

uint32_t prop_get_dma_bank_id_idx(enum vied_nci_dma_dev_id dev_id);

uint32_t prop_get_dma_group_id_idx(enum vied_nci_dma_dev_id dev_id);

struct vied_nci_dma_terminal_desc_bits_t
prop_get_terminal_desc_bits(enum vied_nci_dma_dev_id dev_id);

struct vied_nci_dma_span_desc_bits_t
prop_get_span_desc_bits(enum vied_nci_dma_dev_id dev_id);

struct vied_nci_dma_channel_desc_bits_t
prop_get_channel_desc_bits(enum vied_nci_dma_dev_id dev_id);

struct vied_nci_dma_unit_desc_bits_t
prop_get_unit_desc_bits(enum vied_nci_dma_dev_id dev_id);

struct vied_nci_dma_request_desc_bits_t
prop_get_request_desc_bits(enum vied_nci_dma_dev_id dev_id);

#endif
