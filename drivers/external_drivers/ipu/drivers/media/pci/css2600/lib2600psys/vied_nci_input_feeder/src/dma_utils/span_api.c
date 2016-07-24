#include "span_api.h"
#include "print_support.h"


unsigned int unit_location(
/* creates a unit location field out of (x,y) coordinates */
  int x,
  int y
)
{
  return (
    ((x & ~((-1) << X_COORDINATE_BITS)) << (Y_COORDINATE_BITS)) |
     (y & ~((-1) << Y_COORDINATE_BITS))
  );
}

int get_x_coordinate (
/* returns the x-coordinate contained in a unit location field */
  unsigned int unit_location
)
{
  int field, sign, x;

  // extract the x-coordinate field */
  field = (unit_location >> Y_COORDINATE_BITS) & ~((-1) << X_COORDINATE_BITS);

  // determine the sign of the encoded x-coordinate */
  sign = (field >> (X_COORDINATE_BITS-1)) & 0x1;

  // sign-extend the x-coordinate field to obtain the final x-coordinate value as an integer value */
  x = field | ((-sign) << X_COORDINATE_BITS);

  return x;
}
int get_y_coordinate (
/* returns the y-coordinate contained in a unit location field */
  unsigned int unit_location
)
{
  int field, sign, y;

  // extract the y-coordinate field */
  field = (unit_location & ~((-1) << Y_COORDINATE_BITS));

  // determine the sign of the encoded x-coordinate */
  sign = (field >> (Y_COORDINATE_BITS-1)) & 0x1;

  // sign-extend the x-coordinate field to obtain the final x-coordinate value as an integer value */
  y = field | ((-sign) << Y_COORDINATE_BITS);

  return y;
}

span_order_t get_span_order(
/* returns the span order contained in the span mode field */
  unsigned int span_mode
)
{
  return (span_order_t) ((span_mode >> 1) & 0x1);
}

addressing_mode_t get_addressing_mode(
/* returns the addressing mode contained in the span mode field */
  unsigned int span_mode
)
{
  return (addressing_mode_t) (span_mode & 0x1);
}

unsigned int span_mode(
/* creates a span mode field out of span order and addressing mode inputs */
  span_order_t span_order,
  addressing_mode_t addressing_mode
)
{
  return (
    ((span_order      & 0x1) << 1) |
     (addressing_mode & 0x1)
  );
}


/******************************************************
 * span descriptor API functions
 ******************************************************/
int span_descriptor_words()
{
  int span_descriptor_words = (
                                BYTES(UNIT_LOCATION_BITS) +
                                BYTES(SPAN_WIDTH_BITS)    +
                                BYTES(SPAN_HEIGHT_BITS)   +
                                BYTES(SPAN_WIDTH_BITS)    +
                                BYTES(SPAN_HEIGHT_BITS)   +
                                BYTES(SPAN_MODE_BITS)     +
                                CTRLM_DATA_BYTES - 1
                              ) / CTRLM_DATA_BYTES;
  return span_descriptor_words;
}

void write_span_descriptor(
  int span_descriptor_base_addr,
  int span_id,
  span_descriptor_t span_descriptor
)
{
  int base_addr = span_descriptor_base_addr + span_id * span_descriptor_words() * CTRLM_DATA_BYTES;

  store_descriptor_field(&base_addr, span_descriptor.unit_location, UNIT_LOCATION_BITS );
  store_descriptor_field(&base_addr, span_descriptor.span_row,      SPAN_WIDTH_BITS );
  store_descriptor_field(&base_addr, span_descriptor.span_column,   SPAN_HEIGHT_BITS);
  store_descriptor_field(&base_addr, span_descriptor.span_width,    SPAN_WIDTH_BITS );
  store_descriptor_field(&base_addr, span_descriptor.span_height,   SPAN_HEIGHT_BITS);
  store_descriptor_field(&base_addr, span_descriptor.span_mode,     SPAN_MODE_BITS);
}

span_descriptor_t read_span_descriptor(
  int span_descriptor_base_addr,
  int span_id
)
{
  span_descriptor_t span_descriptor;

  int base_addr = span_descriptor_base_addr + span_id * span_descriptor_words() * CTRLM_DATA_BYTES;

  span_descriptor.unit_location = load_descriptor_field(&base_addr, UNIT_LOCATION_BITS );
  span_descriptor.span_row      = load_descriptor_field(&base_addr, SPAN_WIDTH_BITS );
  span_descriptor.span_column   = load_descriptor_field(&base_addr, SPAN_HEIGHT_BITS);
  span_descriptor.span_width    = load_descriptor_field(&base_addr, SPAN_WIDTH_BITS );
  span_descriptor.span_height   = load_descriptor_field(&base_addr, SPAN_HEIGHT_BITS);
  span_descriptor.span_mode     = load_descriptor_field(&base_addr, SPAN_MODE_BITS);
  return span_descriptor;
}

bool compare_span_descriptors (
// returns true when descriptors are equal
  span_descriptor_t d1,
  span_descriptor_t d2
)
{
  bool equal = true;
  equal = equal && (d1.unit_location == d2.unit_location);
  equal = equal && (d1.span_row      == d2.span_row );
  equal = equal && (d1.span_column   == d2.span_column);
  equal = equal && (d1.span_width    == d2.span_width );
  equal = equal && (d1.span_height   == d2.span_height);
  equal = equal && (d1.span_mode     == d2.span_mode);
  return equal;
}

void print_span_descriptor(
  span_descriptor_t d
)
{
  PRINT("span descriptor contents:\n");
  PRINT("  unit location : 0x%08x", d.unit_location );
  if (get_addressing_mode(d.span_mode) == addressing_mode_coordinate_based) {
    PRINT(" (reflects coordinates (%d, %d))\n", get_x_coordinate(d.unit_location), get_y_coordinate(d.unit_location));
  }
  else {
    PRINT(" (reflects an absolute byte address)\n");
  }
  PRINT("  span row      : %d\n", d.span_row );
  PRINT("  span column   : %d\n", d.span_column);
  PRINT("  span width    : %d\n", d.span_width );
  PRINT("  span height   : %d\n", d.span_height);
  PRINT("  span mode     : %d\n", d.span_mode);
  return;
}

void upload_span_descriptor(
  span_descriptor_t span_descriptor,
  int span_id
)
{
  int bank_addr;

  if (span_id >= SPAN_BANKS) {
    PERROR("attempting to upload a descriptor with span id %d to a non-existent descriptor register bank (number of span banks is %d)\n", span_id, SPAN_BANKS);
    exit(1);
  }

  bank_addr = (SPAN_GROUP_ID << group_id_lsbidx()) + (span_id << bank_id_lsbidx());

//   dma_hrt_store_value(DMA_ID, bank_addr + 0 * CTRLS_DATA_BYTES, span_descriptor.unit_location);
//   dma_hrt_store_value(DMA_ID, bank_addr + 1 * CTRLS_DATA_BYTES, span_descriptor.span_row);
//   dma_hrt_store_value(DMA_ID, bank_addr + 2 * CTRLS_DATA_BYTES, span_descriptor.span_column);
//   dma_hrt_store_value(DMA_ID, bank_addr + 3 * CTRLS_DATA_BYTES, span_descriptor.span_width);
//   dma_hrt_store_value(DMA_ID, bank_addr + 4 * CTRLS_DATA_BYTES, span_descriptor.span_height);
//   dma_hrt_store_value(DMA_ID, bank_addr + 5 * CTRLS_DATA_BYTES, span_descriptor.span_mode);

  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 0 * CTRLS_DATA_BYTES, span_descriptor.unit_location);
  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 1 * CTRLS_DATA_BYTES, span_descriptor.span_row);
  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 2 * CTRLS_DATA_BYTES, span_descriptor.span_column);
  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 3 * CTRLS_DATA_BYTES, span_descriptor.span_width);
  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 4 * CTRLS_DATA_BYTES, span_descriptor.span_height);
  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 5 * CTRLS_DATA_BYTES, span_descriptor.span_mode);

}

span_descriptor_t download_span_descriptor(
  int span_id
)
{
  int bank_addr;
  span_descriptor_t span_descriptor;

  if (span_id >= SPAN_BANKS) {
    PERROR("attempting to download a descriptor with span id %d from a non-existent descriptor register bank (number of span banks is %d)\n", span_id, SPAN_BANKS);
    exit(1);
  }

  bank_addr = (SPAN_GROUP_ID << group_id_lsbidx()) + (span_id << bank_id_lsbidx());

//   span_descriptor.unit_location = dma_hrt_load_value(DMA_ID, bank_addr + 0 * CTRLS_DATA_BYTES);
//   span_descriptor.span_row      = dma_hrt_load_value(DMA_ID, bank_addr + 1 * CTRLS_DATA_BYTES);
//   span_descriptor.span_column   = dma_hrt_load_value(DMA_ID, bank_addr + 2 * CTRLS_DATA_BYTES);
//   span_descriptor.span_width    = dma_hrt_load_value(DMA_ID, bank_addr + 3 * CTRLS_DATA_BYTES);
//   span_descriptor.span_height   = dma_hrt_load_value(DMA_ID, bank_addr + 4 * CTRLS_DATA_BYTES);
//   span_descriptor.span_mode     = dma_hrt_load_value(DMA_ID, bank_addr + 5 * CTRLS_DATA_BYTES);

  span_descriptor.unit_location = dma_hrt_load_value(DMA_BASE_ADDR, bank_addr + 0 * CTRLS_DATA_BYTES);
  span_descriptor.span_row      = dma_hrt_load_value(DMA_BASE_ADDR, bank_addr + 1 * CTRLS_DATA_BYTES);
  span_descriptor.span_column   = dma_hrt_load_value(DMA_BASE_ADDR, bank_addr + 2 * CTRLS_DATA_BYTES);
  span_descriptor.span_width    = dma_hrt_load_value(DMA_BASE_ADDR, bank_addr + 3 * CTRLS_DATA_BYTES);
  span_descriptor.span_height   = dma_hrt_load_value(DMA_BASE_ADDR, bank_addr + 4 * CTRLS_DATA_BYTES);
  span_descriptor.span_mode     = dma_hrt_load_value(DMA_BASE_ADDR, bank_addr + 5 * CTRLS_DATA_BYTES);

  return span_descriptor;
}

void update_span_descriptor(
/* models the updating of span descriptor contents (unit_location, span_row, span_column) after each operation execution */
  span_descriptor_t *span_descriptor,
  unit_descriptor_t unit_descriptor
)
{
  int x = get_x_coordinate((*span_descriptor).unit_location);
  int y = get_y_coordinate((*span_descriptor).unit_location);
  span_order_t span_order = get_span_order((*span_descriptor).span_mode);

  if (span_order == span_order_row_first) {

    if ((*span_descriptor).span_column == (*span_descriptor).span_width) {
      (*span_descriptor).span_column = 0;
      x -= (*span_descriptor).span_width * (unit_descriptor.unit_width + 1);
      if ((*span_descriptor).span_row == (*span_descriptor).span_height) {
        (*span_descriptor).span_row = 0;
        y -= (*span_descriptor).span_height * (unit_descriptor.unit_height + 1);
      }
      else {
        (*span_descriptor).span_row++;
        y += unit_descriptor.unit_height + 1;
      }
    }
    else {
      (*span_descriptor).span_column++;
      x += unit_descriptor.unit_width + 1;
    }

  }
  else { // span_order == span_order_column_first

    if ((*span_descriptor).span_row == (*span_descriptor).span_height) {
      (*span_descriptor).span_row = 0;
      y -= (*span_descriptor).span_height * (unit_descriptor.unit_height + 1);
      if ((*span_descriptor).span_column == (*span_descriptor).span_width) {
        (*span_descriptor).span_column = 0;
        x -= (*span_descriptor).span_width * (unit_descriptor.unit_width + 1);
      }
      else {
        (*span_descriptor).span_column++;
        x += unit_descriptor.unit_width + 1;
      }
    }
    else {
      (*span_descriptor).span_row++;
      y += unit_descriptor.unit_height + 1;
    }

  }
  (*span_descriptor).unit_location = unit_location(x, y);
}


void set_span_bank_mode(
/* sets the bank mode of bank <span_bank_id> to <bank_mode> */
  int span_bank_id,
  bank_mode_t bank_mode
)
{
  int bank_addr;

    if (span_bank_id >= SPAN_BANKS) {
    PERROR("attempting to download a descriptor with span id %d from a non-existent descriptor register bank (number of span banks is %d)\n", span_bank_id, SPAN_BANKS);
    exit(1);
  }

  bank_addr = (SPAN_GROUP_ID << group_id_lsbidx()) + (span_bank_id << bank_id_lsbidx());

//   dma_hrt_store_value(DMA_ID, bank_addr + 15 * CTRLS_DATA_BYTES, (unsigned int) bank_mode);
  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 15 * CTRLS_DATA_BYTES, (unsigned int) bank_mode);
}
