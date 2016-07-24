#include <assert_support.h>
#include "span_descriptor.h"
#include "vied_nci_dma_local.h"
#include "vied_nci_dma_dev_access.h"

/** Creates a unit location field out of (x,y) coordinates.
 */
STORAGE_CLASS_INLINE uint32_t unit_location(
		const struct vied_nci_dma_dev_t *const dev,
		const int x,
		const int y)
{
	const struct vied_nci_dma_span_desc_bits_t *const reg =
	    &(dev->span_desc_bits);

	return (((x & ~((-1) << reg->x_coordinate_bits)) <<
		 (reg->y_coordinate_bits)) | (y & ~((-1) << reg->y_coordinate_bits))
	    );
}

/** Returns the x-coordinate contained in a unit location field.
 */
STORAGE_CLASS_INLINE uint32_t get_x_coordinate(
		const struct vied_nci_dma_dev_t *const dev,
		const uint32_t unit_location)
{
	const struct vied_nci_dma_span_desc_bits_t *const reg =
	    &(dev->span_desc_bits);

	int field, sign, x;

	// extract the x-coordinate field */
	field = (unit_location >> reg->y_coordinate_bits) &
	    ~((-1) << reg->x_coordinate_bits);

	// determine the sign of the encoded x-coordinate */
	sign = (field >> (reg->x_coordinate_bits - 1)) & 0x1;

	// sign-extend the x-coordinate field to obtain the final x-coordinate value as an integer value */
	x = field | ((-sign) << reg->x_coordinate_bits);

	return x;
}

/** Returns the y-coordinate contained in a unit location field.
 */
STORAGE_CLASS_INLINE uint32_t get_y_coordinate(
		const struct vied_nci_dma_dev_t *const dev,
		const uint32_t unit_location)
{
	const struct vied_nci_dma_span_desc_bits_t *const reg =
	    &(dev->span_desc_bits);

	int field, sign, y;

	// extract the y-coordinate field */
	field = (unit_location & ~((-1) << reg->y_coordinate_bits));

	// determine the sign of the encoded x-coordinate */
	sign = (field >> (reg->y_coordinate_bits - 1)) & 0x1;

	// sign-extend the x-coordinate field to obtain the final x-coordinate value as an integer value */
	y = field | ((-sign) << reg->y_coordinate_bits);

	return y;
}

/** Returns the span order contained in the span mode field.
 */
STORAGE_CLASS_INLINE enum vied_nci_dma_span_order_t get_span_order(
		const uint32_t span_mode)
{
	return vied_bit_slice(span_mode, SPAN_SPAN_ORDER_BIT, 1);
}

/** Returns the addressing mode contained in the span mode field.
 */
STORAGE_CLASS_INLINE enum vied_nci_dma_addressing_mode_t
get_addressing_mode(
		const uint32_t span_mode)
{
	return vied_bit_slice(span_mode, SPAN_ADDRESSING_MODE_BIT, 1);
}

STORAGE_CLASS_INLINE bool vied_nci_dma_is_coordinate_based_mode(
		const uint32_t span_mode)
{
	return (get_addressing_mode(span_mode) ==
					VIED_NCI_DMA_ADDRESSING_MODE_COORDINATE_BASED);
}
/******************************************************
 * span descriptor API functions
 ******************************************************/
uint32_t vied_nci_dma_span_desc_mem_store(
		const struct vied_nci_dma_dev_t *const dev,
		const uint32_t span_id,
		const struct vied_nci_dma_span_desc_t *span_descriptor)
{
	uint32_t base_addr;
	uint32_t unit_loc;
	const struct vied_nci_dma_span_desc_bits_t *const reg =
	    &(dev->span_desc_bits);
	uint32_t buf[MAX_DESC_WORDS];
	uint8_t *pbuf = (uint8_t *)buf;

	OP___assert(reg->desc_words <= MAX_DESC_WORDS);
	if ((span_id < dev->num_of_span_banks) || (span_id > dev->num_of_spans)) {
		return EINVAL;
	}

	/* coordinate based addressing */
	if(vied_nci_dma_is_coordinate_based_mode(span_descriptor->span_mode)){
		unit_loc = unit_location(dev, span_descriptor->x_coordinate,
				span_descriptor->y_coordinate);
	} else {
		unit_loc = span_descriptor->unit_location;
	}

	vied_nci_dma_desc_pack(&pbuf, unit_loc,
			       reg->unit_location_bits);
	vied_nci_dma_desc_pack(&pbuf, span_descriptor->span_column,
			       reg->span_column_bits);
	vied_nci_dma_desc_pack(&pbuf, span_descriptor->span_row,
			       reg->span_row_bits);
	vied_nci_dma_desc_pack(&pbuf, span_descriptor->span_width,
			       reg->span_width_bits);
	vied_nci_dma_desc_pack(&pbuf, span_descriptor->span_height,
			       reg->span_height_bits);
	vied_nci_dma_desc_pack(&pbuf, span_descriptor->span_mode,
			       reg->span_mode_bits);

	base_addr = vied_nci_dma_desc_mem_addr(dev->span_desc_base_addr,
			(span_id - dev->num_of_span_banks), reg->desc_words);
	vied_nci_dma_desc_mem_store(base_addr, buf, reg->desc_words);

	return 0;
}

#ifdef VIED_NCI_DMA_DEBUG
struct vied_nci_dma_span_desc_t vied_nci_dma_span_desc_mem_load(
		const struct vied_nci_dma_dev_t *const dev,
		const uint32_t span_id)
{
	uint32_t base_addr;
	struct vied_nci_dma_span_desc_t span_descriptor = {0};

	const struct vied_nci_dma_span_desc_bits_t *const reg =
	    &(dev->span_desc_bits);
	uint32_t buf[MAX_DESC_WORDS];
	uint8_t *pbuf = (uint8_t *)buf;


	if (span_id < dev->num_of_span_banks || span_id > dev->num_of_spans) {
		return span_descriptor;
	}

	base_addr = vied_nci_dma_desc_mem_addr(dev->span_desc_base_addr,
			(span_id - dev->num_of_span_banks), reg->desc_words);
	vied_nci_dma_desc_mem_load(base_addr, buf, reg->desc_words);

	span_descriptor.unit_location =
	    vied_nci_dma_desc_unpack(&pbuf, reg->unit_location_bits);
	span_descriptor.span_column =
	    vied_nci_dma_desc_unpack(&pbuf, reg->span_column_bits);
	span_descriptor.span_row =
	    vied_nci_dma_desc_unpack(&pbuf, reg->span_row_bits);
	span_descriptor.span_width =
	    vied_nci_dma_desc_unpack(&pbuf, reg->span_width_bits);
	span_descriptor.span_height =
	    vied_nci_dma_desc_unpack(&pbuf, reg->span_height_bits);
	span_descriptor.span_mode =
	    vied_nci_dma_desc_unpack(&pbuf, reg->span_mode_bits);

	return span_descriptor;
}
#endif

uint32_t vied_nci_dma_span_desc_reg_store(
		const struct vied_nci_dma_dev_t *const dev,
		const uint32_t span_id,
		const struct vied_nci_dma_span_desc_t *span_descriptor)
{
	uint32_t unit_loc;
	if (span_id >= dev->num_of_span_banks) {
		return EINVAL;
	}

	/* coordinate based addressing */
	if(vied_nci_dma_is_coordinate_based_mode(span_descriptor->span_mode)){
		unit_loc = unit_location(dev, span_descriptor->x_coordinate,
				span_descriptor->y_coordinate);
	} else {
		unit_loc = span_descriptor->unit_location;
	}

	vied_nci_dma_reg_store(dev, SPAN_GROUP_ID, span_id, REG_UNIT_LOCATION,
			    unit_loc);
	vied_nci_dma_reg_store(dev, SPAN_GROUP_ID, span_id, REG_SPAN_COLUMN,
			    span_descriptor->span_column);
	vied_nci_dma_reg_store(dev, SPAN_GROUP_ID, span_id, REG_SPAN_ROW,
			    span_descriptor->span_row);
	vied_nci_dma_reg_store(dev, SPAN_GROUP_ID, span_id, REG_SPAN_WIDTH,
			    span_descriptor->span_width);
	vied_nci_dma_reg_store(dev, SPAN_GROUP_ID, span_id, REG_SPAN_HEIGHT,
			    span_descriptor->span_height);
	vied_nci_dma_reg_store(dev, SPAN_GROUP_ID, span_id, REG_SPAN_MODE,
			    span_descriptor->span_mode);

	return 0;
}

#ifdef VIED_NCI_DMA_DEBUG
struct vied_nci_dma_span_desc_t vied_nci_dma_span_desc_reg_load(
		const struct vied_nci_dma_dev_t *const dev,
		const uint32_t span_id)
{
	struct vied_nci_dma_span_desc_t span_descriptor = {0};

	if (span_id >= dev->num_of_span_banks) {
		return span_descriptor;
	}

	span_descriptor.unit_location =
		vied_nci_dma_reg_load(dev, SPAN_GROUP_ID, span_id, REG_UNIT_LOCATION);
	span_descriptor.span_column =
		vied_nci_dma_reg_load(dev, SPAN_GROUP_ID, span_id, REG_SPAN_COLUMN);
	span_descriptor.span_row =
		vied_nci_dma_reg_load(dev, SPAN_GROUP_ID, span_id, REG_SPAN_ROW);
	span_descriptor.span_width =
		vied_nci_dma_reg_load(dev, SPAN_GROUP_ID, span_id, REG_SPAN_WIDTH);
	span_descriptor.span_height =
		vied_nci_dma_reg_load(dev, SPAN_GROUP_ID, span_id, REG_SPAN_HEIGHT);
	span_descriptor.span_mode =
		vied_nci_dma_reg_load(dev, SPAN_GROUP_ID, span_id, REG_SPAN_MODE);

	return span_descriptor;
}
#endif

uint32_t vied_nci_dma_span_set_bank_mode(
		const struct vied_nci_dma_dev_t *const dev,
		const uint32_t span_bank_id,
		const enum vied_nci_dma_bank_mode_t bank_mode)
{
	if (span_bank_id >= dev->num_of_span_banks) {
		return EINVAL;
	}

	vied_nci_dma_reg_store(dev, SPAN_GROUP_ID, span_bank_id,
			REG_BANK_MODE, bank_mode);

	return 0;
}
