#include <errno.h>
#include "vied_nci_dma.h"

#include "channel_descriptor.h"
#include "global_descriptor.h"
#include "master_descriptor.h"
#include "property.h"
#include "request_descriptor.h"
#include "span_descriptor.h"
#include "terminal_descriptor.h"
#include "unit_descriptor.h"
#include "vied_nci_dma_dev_access.h"

static struct vied_nci_dma_dev_t dma_dev[VIED_NCI_N_DMA_DEV];

STORAGE_CLASS_INLINE uint16_t vied_nci_dma_get_span_A_id(
		const uint16_t chan_id)
{
	return chan_id * 2;
}

STORAGE_CLASS_INLINE uint16_t vied_nci_dma_get_span_B_id(
		const uint16_t chan_id)
{
	return chan_id * 2 + 1;
}

STORAGE_CLASS_INLINE uint16_t vied_nci_dma_get_unit_id(
		const uint8_t chan_id)
{
	return chan_id;
}

STORAGE_CLASS_INLINE uint16_t vied_nci_dma_get_terminal_A_id(
		const uint16_t chan_id)
{
	return chan_id * 2;
}

STORAGE_CLASS_INLINE uint16_t vied_nci_dma_get_terminal_B_id(
		uint16_t chan_id)
{
	return chan_id * 2 + 1;
}

STORAGE_CLASS_INLINE uint16_t vied_nci_dma_get_requestor_id(
		const struct vied_nci_dma_dev_t *dev,
		const uint16_t chan_id)
{
	/* In VIED_NCI_DMA_DEDICATED_REQUESTOR, one requestor is dedicated to
	 * one channel. In VIED_NCI_DMA_SHARED_REQUESTOR, requestors are
	 * uniformally distributed and multiple channels can share one requestor
	 */
	return chan_id % dev->num_of_requestors;
}

STORAGE_CLASS_INLINE uint16_t vied_nci_dma_get_num_of_channels(
		uint16_t num_of_channels,
		uint16_t num_of_terminals,
		uint16_t num_of_spans,
		uint16_t num_of_units)
{

	/* The num of channels must be less than the half num of terminals
	 * and spans due to fixed ID mapping
	 */
	if (num_of_channels > (num_of_terminals / 2)) {
		num_of_channels = num_of_terminals / 2;
	}

	if (num_of_channels > (num_of_spans / 2)) {
		num_of_channels = num_of_spans / 2;
	}

	if (num_of_channels > (num_of_units)) {
		num_of_channels = num_of_units;
	}

	return num_of_channels;
}

STORAGE_CLASS_INLINE bool vied_nci_dma_is_cached_mode(
	const struct vied_nci_dma_dev_t *dev)
{
	return (dev->bank_mode == VIED_NCI_DMA_BANK_MODE_CACHED);
}


/** Initialize DMA dev structure.
*/
static int
vied_nci_dma_specify_prop(
		const enum vied_nci_dma_dev_id dev_id)
{
	struct vied_nci_dma_dev_t *dev = &dma_dev[dev_id];
	uint32_t j;
	uint16_t desc_size = 0;
	uint8_t num_of_desc = 0;

	if (dev_id >= VIED_NCI_N_DMA_DEV) {
		return ENODEV;
	}

	// Initialize DMA devivce property
	dev->dev_id = dev_id;
	dev->num_of_requestors = dev_prop[dev_id].RequestBanks;
	dev->num_of_channels = dev_prop[dev_id].Channels;
	dev->num_of_terminals = dev_prop[dev_id].Terminals;
	dev->num_of_spans = dev_prop[dev_id].Spans;
	dev->num_of_units = dev_prop[dev_id].Units;

	dev->num_of_channel_banks = dev_prop[dev_id].ChannelBanks;
	dev->num_of_terminal_banks = dev_prop[dev_id].TerminalBanks;
	dev->num_of_span_banks = dev_prop[dev_id].SpanBanks;
	dev->num_of_unit_banks = dev_prop[dev_id].UnitBanks;

	dev->global_sets = dev_prop[dev_id].GlobalSets;
	dev->master_banks = MASTER_BANKS;
	dev->max_block_height = dev_prop[dev_id].MaxBlockHeight;
	dev->max_block_width = dev_prop[dev_id].MaxBlockWidth;
	dev->max_linear_burst_size = dev_prop[dev_id].MaxLinearBurstSize;

	for (j = 0; j < dev->master_banks; j++) {
		dev->srmd_support[j] = dev_prop[dev_id].SRMDSupport[j];
		// TODO: Burst2DSupport is not considered since no DMA supports
		dev->burst_support[j] = dev_prop[dev_id].BurstSupport[j];
	}

	// DMA dev base address
	dev->base_addr = dev_prop[dev_id].BaseAddress;

	dev->bank_mode_reg_available =
		dev_prop[dev_id].BankModeRegisterAvailable;
	dev->region_stride_reg_available =
		dev_prop[dev_id].RegionStrideRegisterAvailable;

	// Slave address decomposition
	dev->byte_id_idx = 0;
	dev->register_id_idx = prop_get_dma_register_id_idx(dev_id);
	dev->bank_id_idx = prop_get_dma_bank_id_idx(dev_id);
	dev->group_id_idx = prop_get_dma_group_id_idx(dev_id);

	// Descriptor words and register bits for cached mode
	dev->terminal_desc_bits = prop_get_terminal_desc_bits(dev_id);
	dev->span_desc_bits = prop_get_span_desc_bits(dev_id);
	dev->channel_desc_bits = prop_get_channel_desc_bits(dev_id);
	dev->unit_desc_bits = prop_get_unit_desc_bits(dev_id);
	dev->request_desc_bits = prop_get_request_desc_bits(dev_id);

	dev->num_of_physical_channels = vied_nci_dma_get_num_of_channels(
		dev->num_of_channel_banks,
		dev->num_of_terminal_banks,
		dev->num_of_span_banks,
		dev->num_of_unit_banks);

	dev->num_of_logic_channels = vied_nci_dma_get_num_of_channels(
		dev->num_of_channels -
		dev->num_of_channel_banks,
		dev->num_of_terminals -
		dev->num_of_terminal_banks,
		dev->num_of_spans -
		dev->num_of_span_banks,
		dev->num_of_units -
		dev->num_of_unit_banks);

	dev->chans = NULL;

	/* Only logical descriptors are considered for cache memory */
	desc_size = dev->terminal_desc_bits.desc_words * CTRLM_DATA_BYTES;
	num_of_desc = dev->num_of_terminals - dev->num_of_terminal_banks;
	dev->terminal_desc_size = num_of_desc * desc_size;

	desc_size = dev->span_desc_bits.desc_words * CTRLM_DATA_BYTES;
	num_of_desc = dev->num_of_spans - dev->num_of_span_banks;
	dev->span_desc_size = num_of_desc * desc_size;

	desc_size = dev->channel_desc_bits.desc_words * CTRLM_DATA_BYTES;
	num_of_desc = dev->num_of_logic_channels;
	dev->channel_desc_size = num_of_desc * desc_size;

	desc_size = dev->unit_desc_bits.desc_words * CTRLM_DATA_BYTES;
	num_of_desc = dev->num_of_units - dev->num_of_unit_banks;
	dev->unit_desc_size = num_of_desc * desc_size;

	return 0;
}

struct vied_nci_dma_dev_t* vied_nci_dma_open(
		const enum vied_nci_dma_dev_id dev_id)
{
	if (dev_id >= VIED_NCI_N_DMA_DEV) {
		errno = ENODEV;
		return NULL;
	}

	vied_nci_dma_specify_prop(dev_id);

	return &dma_dev[dev_id];
}


int vied_nci_dma_close(
		struct vied_nci_dma_dev_t *const dev)
{
	NOT_USED(dev);
	return 0;
}

int vied_nci_dma_configure(
		struct vied_nci_dma_dev_t *const dev,
		const struct vied_nci_dma_dev_config_t *const config)
{
	struct vied_nci_dma_global_desc_t global_descriptor;
	struct vied_nci_dma_master_desc_t master_descriptor[MAX_MASTER_BANKS];
	uint8_t global_id;
	uint8_t master_id;
	uint8_t i = 0;

	if (config->bank_mode == VIED_NCI_DMA_BANK_MODE_NON_CACHED) {
		// Set bank, 0: Non-cached mode, 1: cached  mode
		dev->bank_mode = VIED_NCI_DMA_BANK_MODE_NON_CACHED;
		dev->terminal_desc_base_addr = 0;
		dev->channel_desc_base_addr = 0;
		dev->span_desc_base_addr = 0;
		dev->unit_desc_base_addr = 0;
		dev->chans =
			(struct vied_nci_dma_chan_t *)config->chan_handle_addr;
		// Don't change the number of channels if the config shows 0
		if (config->num_of_channels != 0) {
			dev->num_of_physical_channels = config->num_of_channels;
		}
	} else {
		// Cached mode
		if (dev->num_of_logic_channels == 0) {
			return EINVAL;
		}
		dev->bank_mode = VIED_NCI_DMA_BANK_MODE_CACHED;
		dev->terminal_desc_base_addr = config->terminal_desc_base_addr;
		dev->channel_desc_base_addr = config->channel_desc_base_addr;
		dev->span_desc_base_addr = config->span_desc_base_addr;
		dev->unit_desc_base_addr = config->unit_desc_base_addr;
		dev->chans =
			(struct vied_nci_dma_chan_t *)config->chan_handle_addr;
		// Don't change the number of channels if the config shows 0
		if (config->num_of_channels != 0) {
			dev->num_of_logic_channels = config->num_of_channels;
		}
	}
	if (dev->chans == NULL) {
		return ENOMEM;
	}

	if (config->req_type == VIED_NCI_DMA_DEDICATED_REQUESTOR) {
		if (vied_nci_dma_is_cached_mode(dev) ||
			(dev->num_of_channel_banks > dev->num_of_requestors)) {
			return EINVAL;
		}
		dev->req_type = VIED_NCI_DMA_DEDICATED_REQUESTOR;
	} else {
		dev->req_type = VIED_NCI_DMA_SHARED_REQUESTOR;
	}

	global_descriptor.unit_descriptor_base_addr = dev->unit_desc_base_addr;
	global_descriptor.span_descriptor_base_addr = dev->span_desc_base_addr;
	global_descriptor.terminal_descriptor_base_addr =
		dev->terminal_desc_base_addr;
	global_descriptor.channel_descriptor_base_addr =
		dev->channel_desc_base_addr;

	global_descriptor.max_block_height = dev->max_block_height - 1;

	//TODO: Global sets are alwalys set to be the same currently
	for (global_id = 0; global_id < dev->global_sets; global_id++) {
		global_descriptor.max_1d_block_width[global_id] =
		    dev->max_linear_burst_size - 1;
		global_descriptor.max_2d_block_width[global_id] =
		    dev->max_block_width - 1;
	}

	for (master_id = 0; master_id < dev->master_banks; master_id++) {
		master_descriptor[master_id].srmd_support =
		    dev->srmd_support[master_id];
		master_descriptor[master_id].burst_support =
		    dev->burst_support[master_id];
	}

	// Upload Global descriptors
	vied_nci_dma_global_desc_reg_store(dev, &global_descriptor);

	// Upload Master descriptors
	for (master_id = 0; master_id < dev->master_banks; master_id++) {
		vied_nci_dma_master_desc_reg_store(dev,
			&master_descriptor[master_id],
			master_id);
	}

	if (dev->bank_mode_reg_available) {
		// Set bank mode
		for (i = 0; i < dev->num_of_terminal_banks; i++) {
			vied_nci_dma_terminal_set_bank_mode(dev,
				i, dev->bank_mode);
		}

		for (i = 0; i < dev->num_of_span_banks; i++) {
			vied_nci_dma_span_set_bank_mode(dev, i, dev->bank_mode);
		}

		for (i = 0; i < dev->num_of_channel_banks; i++) {
			vied_nci_dma_channel_set_bank_mode(dev,
				i, dev->bank_mode);
		}

		for (i = 0; i < dev->num_of_unit_banks; i++) {
			vied_nci_dma_unit_set_bank_mode(dev, i, dev->bank_mode);
		}
	}

	return 0;
}

struct vied_nci_dma_chan_t* vied_nci_dma_chan_open(
		const enum vied_nci_dma_dev_id dev_id,
		const uint8_t chan_id,
		const struct vied_nci_dma_transfer_config_t *const config)
{
	struct vied_nci_dma_dev_t *dev = NULL;
	struct vied_nci_dma_chan_t *chan = NULL;
	uint8_t chan_offset = 0;
	uint8_t num_of_chan = 0;
	uint32_t request_id;
	uint32_t instruction = 0;

	// check if dev_id > DMA_FW & dev_id < NUM_DMA_DEV,
	if (dev_id >= VIED_NCI_N_DMA_DEV) {
		errno = ENODEV;
		goto exit;
	}

	dev = &dma_dev[dev_id];

	// Acquire the channel
	if (vied_nci_dma_is_cached_mode(dev)) {
		// Cached  mode: use only logic channels
		chan_offset = dev->num_of_channel_banks;
		num_of_chan = dev->num_of_logic_channels;
	} else {
		// Non-cached mode: use only physical channels
		chan_offset = 0;
		num_of_chan = dev->num_of_physical_channels;
	}

	if (chan_id >= num_of_chan) {
		errno = ENODEV;
		goto exit;
	}

	chan = &(dev->chans[chan_id]);
	// Logic channel ID doesn't start from 0
	chan->chan_id = chan_id + chan_offset;
	chan->dev_id = dev_id;

	if (vied_nci_dma_is_cached_mode(dev)) {
		request_id = vied_nci_dma_get_requestor_id(dev, chan->chan_id);

		// Invalidate the descriptor cache
		instruction = vied_nci_dma_create_invalidate_instruction(
			dev,
			VIED_NCI_DMA_DESCRIPTOR_KIND_CHANNEL,
			chan->chan_id,
			chan->chan_id);
		vied_nci_dma_request_desc_run(dev, request_id, instruction);

		// Write Channel
		vied_nci_dma_channel_desc_mem_store(dev, chan->chan_id,
				&config->channel_desc);

		// Invalidate the descriptor cache
		instruction = vied_nci_dma_create_invalidate_instruction(
			dev,
			VIED_NCI_DMA_DESCRIPTOR_KIND_UNIT,
			vied_nci_dma_get_unit_id(chan->chan_id),
			vied_nci_dma_get_unit_id(chan->chan_id));
		vied_nci_dma_request_desc_run(dev, request_id, instruction);

		// Write Unit
		vied_nci_dma_unit_desc_mem_store(dev,
			vied_nci_dma_get_unit_id(chan->chan_id),
			&config->unit_desc);
	} else {
		// Upload Channel
		vied_nci_dma_channel_desc_reg_store(dev,
			chan->chan_id,
			&config->channel_desc);

		// Upload Unit
		vied_nci_dma_unit_desc_reg_store(dev,
			vied_nci_dma_get_unit_id(chan->chan_id),
			&config->unit_desc);
	}

	return chan;

exit:
	return NULL;
}

int vied_nci_dma_chan_configure(
		const struct vied_nci_dma_chan_t *const chan,
		const struct vied_nci_dma_terminal_config_t *const config)
{
	const struct vied_nci_dma_dev_t *const dev = &dma_dev[chan->dev_id];
	struct vied_nci_dma_request_desc_t request_descriptor;
	uint32_t request_id;
	uint32_t instruction = 0;

	if (vied_nci_dma_is_cached_mode(dev)) {
		request_id = vied_nci_dma_get_requestor_id(dev, chan->chan_id);

		// Invalidate the descriptor cache
		instruction = vied_nci_dma_create_invalidate_instruction(
			dev,
			VIED_NCI_DMA_DESCRIPTOR_KIND_TERMINAL,
			vied_nci_dma_get_terminal_A_id(chan->chan_id),
			vied_nci_dma_get_terminal_A_id(chan->chan_id));
		vied_nci_dma_request_desc_run(dev, request_id, instruction);

		instruction = vied_nci_dma_create_invalidate_instruction(
			dev,
			VIED_NCI_DMA_DESCRIPTOR_KIND_TERMINAL,
			vied_nci_dma_get_terminal_B_id(chan->chan_id),
			vied_nci_dma_get_terminal_B_id(chan->chan_id));
		vied_nci_dma_request_desc_run(dev, request_id, instruction);

		// Write Terminals
		vied_nci_dma_terminal_desc_mem_store(dev,
			vied_nci_dma_get_terminal_A_id(chan->chan_id),
			&config->terminal_desc[0]);
		vied_nci_dma_terminal_desc_mem_store(dev,
			vied_nci_dma_get_terminal_B_id(chan->chan_id),
			&config->terminal_desc[1]);

		// Invalidate the descriptor cache
		instruction = vied_nci_dma_create_invalidate_instruction(
			dev,
			VIED_NCI_DMA_DESCRIPTOR_KIND_SPAN,
			vied_nci_dma_get_span_A_id(chan->chan_id),
			vied_nci_dma_get_span_A_id(chan->chan_id));
		vied_nci_dma_request_desc_run(dev, request_id, instruction);

		instruction = vied_nci_dma_create_invalidate_instruction(
			dev,
			VIED_NCI_DMA_DESCRIPTOR_KIND_SPAN,
			vied_nci_dma_get_span_B_id(chan->chan_id),
			vied_nci_dma_get_span_B_id(chan->chan_id));
		vied_nci_dma_request_desc_run(dev, request_id, instruction);

		// Write Spans
		vied_nci_dma_span_desc_mem_store(dev,
			vied_nci_dma_get_span_A_id(chan->chan_id),
			&config->span_desc[0]);
		vied_nci_dma_span_desc_mem_store(dev,
			vied_nci_dma_get_span_B_id(chan->chan_id),
			&config->span_desc[1]);
	} else {

		// Upload Terminals
		vied_nci_dma_terminal_desc_reg_store(dev,
			vied_nci_dma_get_terminal_A_id(chan->chan_id),
			&config->terminal_desc[0]);
		vied_nci_dma_terminal_desc_reg_store(dev,
			vied_nci_dma_get_terminal_B_id(chan->chan_id),
			&config->terminal_desc[1]);

		// Upload Spans
		vied_nci_dma_span_desc_reg_store(dev,
			vied_nci_dma_get_span_A_id(chan->chan_id),
			&config->span_desc[0]);
		vied_nci_dma_span_desc_reg_store(dev,
			vied_nci_dma_get_span_B_id(chan->chan_id),
			&config->span_desc[1]);
	}

	if (dev->req_type == VIED_NCI_DMA_DEDICATED_REQUESTOR) {
		request_id = vied_nci_dma_get_requestor_id(dev, chan->chan_id);
		request_descriptor.descriptor_id_setup_1 =
			vied_nci_dma_descriptor_id_setup_1(dev,
				vied_nci_dma_get_unit_id(chan->chan_id),
				vied_nci_dma_get_terminal_A_id(chan->chan_id),
				vied_nci_dma_get_terminal_B_id(chan->chan_id),
				chan->chan_id);
		request_descriptor.descriptor_id_setup_2 =
			vied_nci_dma_descriptor_id_setup_2(dev,
				vied_nci_dma_get_span_A_id(chan->chan_id),
				vied_nci_dma_get_span_B_id(chan->chan_id));

		// Upload Request descriptor
		vied_nci_dma_request_desc_reg_store(dev,
			request_id, &request_descriptor);
	}

	return 0;
}

int vied_nci_dma_chan_start(
		const struct vied_nci_dma_chan_t *const chan)
{
	NOT_USED(chan);
	return 0;
}

// Start next DMA transfer
int vied_nci_dma_chan_next(
		const struct vied_nci_dma_chan_t *chan,
		const enum vied_nci_dma_command_t command,
		const uint32_t n)
{
	const struct vied_nci_dma_dev_t *const dev = &dma_dev[chan->dev_id];

	struct vied_nci_dma_request_desc_t request_descriptor;
	uint32_t request_id;
	uint32_t instruction = 0;

	request_id = vied_nci_dma_get_requestor_id(dev, chan->chan_id);

	if (dev->req_type == VIED_NCI_DMA_SHARED_REQUESTOR) {
		request_descriptor.descriptor_id_setup_1 =
			vied_nci_dma_descriptor_id_setup_1(dev,
				vied_nci_dma_get_unit_id(chan->chan_id),
				vied_nci_dma_get_terminal_A_id(chan->chan_id),
				vied_nci_dma_get_terminal_B_id(chan->chan_id),
				chan->chan_id);
		request_descriptor.descriptor_id_setup_2 =
			vied_nci_dma_descriptor_id_setup_2(dev,
				vied_nci_dma_get_span_A_id(chan->chan_id),
				vied_nci_dma_get_span_B_id(chan->chan_id));

		// Upload Request descriptor
		vied_nci_dma_request_desc_reg_store(dev,
			request_id, &request_descriptor);
	}

	instruction = vied_nci_dma_create_execute_instruction(dev, command, n);
	vied_nci_dma_request_desc_run(dev, request_id, instruction);

	return 0;
}

int vied_nci_dma_chan_stop(
		const struct vied_nci_dma_chan_t *const chan)
{
	NOT_USED(chan);
	return 0;
}

int vied_nci_dma_chan_close(
		struct vied_nci_dma_chan_t *const chan)
{
	chan->chan_id = 0;

	return 0;
}

int vied_nci_dma_get_chan_res(
		const struct vied_nci_dma_chan_t *chan,
		struct vied_nci_dma_chan_res *res)
{
	const struct vied_nci_dma_dev_t *const dev = &dma_dev[chan->dev_id];

	res->chan_id = chan->chan_id;
	res->unit_id = vied_nci_dma_get_unit_id(chan->chan_id);
	res->terminal_A_id = vied_nci_dma_get_terminal_A_id(chan->chan_id);
	res->terminal_B_id = vied_nci_dma_get_terminal_B_id(chan->chan_id);
	res->span_A_id = vied_nci_dma_get_span_A_id(chan->chan_id);
	res->span_B_id = vied_nci_dma_get_span_B_id(chan->chan_id);
	res->requestor_id = vied_nci_dma_get_requestor_id(dev, chan->chan_id);
	if (!vied_nci_dma_is_cached_mode(dev)) {
		// for Non-cached mode only
		res->chan_addr = vied_nci_dma_reg_addr(dev, CHANNEL_GROUP_ID,
				res->chan_id, REG_REG0);
		res->unit_addr = vied_nci_dma_reg_addr(dev, UNIT_GROUP_ID,
				res->unit_id, REG_REG0);
		res->terminal_A_addr = vied_nci_dma_reg_addr(dev, TERMINAL_GROUP_ID,
				res->terminal_A_id, REG_REG0);
		res->terminal_B_addr = vied_nci_dma_reg_addr(dev, TERMINAL_GROUP_ID,
				res->terminal_B_id, REG_REG0);
		res->span_A_addr = vied_nci_dma_reg_addr(dev, SPAN_GROUP_ID,
				res->span_A_id, REG_REG0);
		res->span_B_addr = vied_nci_dma_reg_addr(dev, SPAN_GROUP_ID,
				res->span_B_id, REG_REG0);
		res->requestor_addr = vied_nci_dma_reg_addr(dev, REQUEST_GROUP_ID,
				res->requestor_id, REG_REG0);
	}

	return 0;
}


#ifdef VIED_NCI_DMA_DEBUG
int vied_nci_dma_chan_wait_for_ack(
		const struct vied_nci_dma_chan_t *chan)
{
	const struct vied_nci_dma_dev_t *const dev = &dma_dev[chan->dev_id];
	struct vied_nci_dma_channel_desc_t channel_desc;
	int ret = 0;

	if (vied_nci_dma_is_cached_mode(dev)) {
		// Read Channel
		channel_desc = vied_nci_dma_channel_desc_mem_load(dev,
			chan->chan_id);
	} else {
		// Download Channel
		channel_desc = vied_nci_dma_channel_desc_reg_load(dev,
			chan->chan_id);
	}

	ret = vied_nci_dma_wait_for_acknowledge(dev, chan->chan_id,
		&channel_desc);
	return ret;
}
#endif
