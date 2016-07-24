#include "model.h"
#include "print_support.h"
#include "cpu_mem_support.h"


unsigned int get_logical_block_width (
  unit_descriptor_t     unit_descriptor,
  span_descriptor_t     span_descriptor,
  terminal_descriptor_t terminal_descriptor,
  channel_descriptor_t  channel_descriptor
)
{
  int x;
  int factor;

  int logical_block_width;

  if (get_addressing_mode(span_descriptor.span_mode) == addressing_mode_coordinate_based) {

    x      = get_x_coordinate(span_descriptor.unit_location);
    factor = (int) get_subsampling_factor(channel_descriptor.sampling_setup);

    if (x < 0) {
      if (channel_descriptor.padding_mode == padding_mode_append) {
        logical_block_width = unit_descriptor.unit_width;
      }
      else {
        logical_block_width = unit_descriptor.unit_width + ((x - (factor-1)) / factor);
      }
    }
    else if ((x >= 0) && (x <= (int) terminal_descriptor.region_width - (int) (unit_descriptor.unit_width * factor))) {
      logical_block_width = unit_descriptor.unit_width;
    }
    else { // x > terminal_descriptor.region_width - unit_descriptor.unit_width * factor
      if (channel_descriptor.padding_mode == padding_mode_append) {
        logical_block_width = unit_descriptor.unit_width;
      }
      else {
        // (RW - x + (factor - 1)) / f
        logical_block_width = (terminal_descriptor.region_width - x + (factor-1)) / factor;
      }
    }
  }
  else {
    logical_block_width = unit_descriptor.unit_width;
  }
  return (unsigned int) logical_block_width;
}

unsigned int get_logical_block_height(
  unit_descriptor_t     unit_descriptor
)
{
  return unit_descriptor.unit_height;
}

unsigned int get_elements_per_word(
  unsigned int data_bits,
  unsigned int precision
)
{
  return data_bits / precision;
}


unsigned int get_read_data_bits(
  transfer_direction_t transfer_direction
)
{
  unsigned int read_data_bits;
  if (transfer_direction == transfer_direction_ab) {
    read_data_bits = DATAMA_DATA_BITS;
  }
  else { // transfer_direction == transfer_direction_ba
    read_data_bits = DATAMB_DATA_BITS;
  }
  return read_data_bits;
}

unsigned int get_write_data_bits(
  transfer_direction_t transfer_direction
)
{
  unsigned int write_data_bits;
  if (transfer_direction == transfer_direction_ab) {
    write_data_bits = DATAMB_DATA_BITS;
  }
  else { // transfer_direction == transfer_direction_ba
    write_data_bits = DATAMA_DATA_BITS;
  }
  return write_data_bits;
}


unsigned int get_left_margin(
  request_descriptor_t  request_descriptor,
  unit_descriptor_t     unit_descriptor,
  span_descriptor_t     span_descriptor,
  terminal_descriptor_t terminal_descriptor,
  channel_descriptor_t  channel_descriptor
)
{
  int x;
  int factor;
  int data_bits;
  int elements_per_word;
  int left_margin;

  if (get_addressing_mode(span_descriptor.span_mode) == addressing_mode_coordinate_based) {
    x                 =       get_x_coordinate(span_descriptor.unit_location);
    factor            = (int) get_subsampling_factor(channel_descriptor.sampling_setup);
    data_bits         = (int) get_read_data_bits(get_transfer_direction(request_descriptor.instruction));
    elements_per_word = (int) get_elements_per_word(data_bits, get_precision(terminal_descriptor.element_setup));
    if (x < 0) {
      if (channel_descriptor.padding_mode == padding_mode_append) {
        left_margin = (x % elements_per_word);
        left_margin += elements_per_word;
        left_margin /= factor;
      }
      else {
        left_margin = 0;
      }
    }
    else { // x >= 0
      left_margin = (x % elements_per_word) / factor;
    }
  }
  else {
    left_margin = 0;
  }
  return (unsigned int) left_margin;
}

unsigned int get_left_padding_amount(
  span_descriptor_t     span_descriptor,
  channel_descriptor_t  channel_descriptor
)
{
  int x;
  int factor;

  int left_padding_amount;

  if (get_addressing_mode(span_descriptor.span_mode) == addressing_mode_coordinate_based) {
    x      =       get_x_coordinate(span_descriptor.unit_location);
    factor = (int) get_subsampling_factor(channel_descriptor.sampling_setup);
    if (x < 0) {
      if ((channel_descriptor.padding_mode == padding_mode_append) || ((channel_descriptor.padding_mode == padding_mode_truncate))) {
        left_padding_amount = 0;
      }
      else {
        left_padding_amount = - ((x - (factor-1)) / factor);
      }
    }
    else { // x >= 0
      left_padding_amount = 0;
    }
  }
  else {
    left_padding_amount = 0;
  }
  return (unsigned int) left_padding_amount;
}

unsigned int get_right_padding_amount(
  unit_descriptor_t     unit_descriptor,
  span_descriptor_t     span_descriptor,
  terminal_descriptor_t terminal_descriptor,
  channel_descriptor_t  channel_descriptor
)
{
  int x;
  int factor;

  int right_padding_amount;
  if (get_addressing_mode(span_descriptor.span_mode) == addressing_mode_coordinate_based) {
    x      =       get_x_coordinate(span_descriptor.unit_location);
    factor = (int) get_subsampling_factor(channel_descriptor.sampling_setup);

    if (x > (int) (terminal_descriptor.region_width - (int) unit_descriptor.unit_width * factor)) {
      if ((channel_descriptor.padding_mode == padding_mode_append) || ((channel_descriptor.padding_mode == padding_mode_truncate))) {
        right_padding_amount = 0;
      }
      else {
        right_padding_amount = (unit_descriptor.unit_width * factor + x - terminal_descriptor.region_width) / factor;
      }
    }
    else { // x <= terminal_descriptor.region_width - unit_descriptor.unit_width * factor
      right_padding_amount = 0;
    }
  }
  else {
    right_padding_amount = 0;
  }
  return (unsigned int) right_padding_amount;
}

unsigned int get_subsampling_offset(
  request_descriptor_t  request_descriptor,
  unit_descriptor_t     unit_descriptor,
  span_descriptor_t     span_descriptor,
  terminal_descriptor_t terminal_descriptor,
  channel_descriptor_t  channel_descriptor
)
{
  int x;
  int factor;
  int data_bits;
  int elements_per_word;
  int offset;
  if (get_addressing_mode(span_descriptor.span_mode) == addressing_mode_coordinate_based) {
    x                 =       get_x_coordinate(span_descriptor.unit_location);
    factor            = (int) get_subsampling_factor(channel_descriptor.sampling_setup);
    data_bits         = (int) get_read_data_bits(get_transfer_direction(request_descriptor.instruction));
    elements_per_word = (int) get_elements_per_word(data_bits, get_precision(terminal_descriptor.element_setup));
    offset            = (unsigned int) (x % elements_per_word) % factor;
  }
  else {
    offset = 0;
  }
  return offset;
}

unsigned int get_read_start_addr(
  request_descriptor_t  request_descriptor,
  unit_descriptor_t     unit_descriptor,
  span_descriptor_t     span_descriptor,
  terminal_descriptor_t terminal_descriptor,
  channel_descriptor_t  channel_descriptor
)
{

  int x;
  int y;
  int data_bits;
  int elements_per_word;

  int addr;

  if (get_addressing_mode(span_descriptor.span_mode) == addressing_mode_coordinate_based) {
    x                 =       get_x_coordinate(span_descriptor.unit_location);
    y                 =       get_y_coordinate(span_descriptor.unit_location);
    data_bits         = (int) get_read_data_bits(get_transfer_direction(request_descriptor.instruction));
    elements_per_word = (int) get_elements_per_word(data_bits, get_precision(terminal_descriptor.element_setup));

    if (x < 0) {
      if (channel_descriptor.padding_mode == padding_mode_append) {
//        addr = (int) terminal_descriptor.region_origin + y * (int) terminal_descriptor.region_stride + (x / elements_per_word) - 1;
        addr = (int) terminal_descriptor.region_origin + y * (int) terminal_descriptor.region_stride + (x / elements_per_word) - BYTES(data_bits);
      }
      else {
        addr = (int) terminal_descriptor.region_origin + y * (int) terminal_descriptor.region_stride;
      }
    }
    else { // x >= 0
      addr = (int) terminal_descriptor.region_origin + y * (int) terminal_descriptor.region_stride + (x / elements_per_word) * BYTES(data_bits);
    }
  }
  else {
    addr = span_descriptor.unit_location;
  }
  return (unsigned int) addr;
}

unsigned int get_write_start_addr(
  request_descriptor_t  request_descriptor,
  span_descriptor_t     span_descriptor,
  terminal_descriptor_t terminal_descriptor,
  channel_descriptor_t  channel_descriptor
)
{
  int x;
  int y;
  int data_bits;
  int precision;

  int bytes_per_element;
  int elements_per_word;

  int addr;

  if (get_addressing_mode(span_descriptor.span_mode) == addressing_mode_coordinate_based) {
    x                 =       get_x_coordinate(span_descriptor.unit_location);
    y                 =       get_y_coordinate(span_descriptor.unit_location);
    data_bits         = (int) get_write_data_bits(get_transfer_direction(request_descriptor.instruction));
    precision         = (int) get_precision(terminal_descriptor.element_setup);

    if (is_strict_precision(precision)) {
      bytes_per_element = (int) get_elements_per_word(precision, 8);
      addr = ((int) terminal_descriptor.region_origin + y * (int) terminal_descriptor.region_stride +  x * bytes_per_element);
    }
    else {
      elements_per_word = (int) get_elements_per_word(data_bits, precision);
      addr = ((int) terminal_descriptor.region_origin + y * (int) terminal_descriptor.region_stride + (x / elements_per_word));
    }
  }
  else {
    addr = span_descriptor.unit_location;
  }

  return (unsigned int) addr;
}

void load_block(
  char ***elem,
  int data_bits,
  int start_addr,
  int stride,
  int precision_bits,
  int subsampling_factor,
  int subsampling_offset,
  int logical_block_width,
  int logical_block_height,
  int left_margin,
  int left_padding_amount,
  int right_padding_amount,
  int padding_mode,
  int elem_init_sign_extend,
  int elem_init_data,
  int mem_id,
  int written_block /* 1, if this block represents a block written by the DMA; 0, if this block represents a block read by the DMA */
)
{
  int row_addr, addr;
  int row_addr_lsbs;
  int elems_per_word;
  int width_in_bytes;
  int word_idx, bit_idx;
  int bits_remaining, prev_bits_remaining, bits_available, bits_required, bits_consumed;
  int i, j, h, w, l, r;
  char byte;
  int incr;

  int _left_margin;

  int transfer_block_width;

  int wl, wu;
  int b;

  /* array of bytes to encode element initialization data converted to the target element precision */
  char *_elem_init_data = ia_css_cpu_mem_alloc(BYTES(precision_bits) * sizeof(char));

  for (b = 0; b < BYTES(precision_bits); b++) {
    if (b < ELEMENT_INIT_DATA_BITS / 8) {
      _elem_init_data[b] = (elem_init_data >> (b * 8)) & 0xFF;
    }
    else if (b == ELEMENT_INIT_DATA_BITS / 8) {
      _elem_init_data[b]  =    (elem_init_data >> (b * 8)) & ~(-1 << (ELEMENT_INIT_DATA_BITS % 8));
      // sign/zero-extend
      _elem_init_data[b] |= (-((_elem_init_data[b] >> ((ELEMENT_INIT_DATA_BITS-1) % 8)) & elem_init_sign_extend)) << (ELEMENT_INIT_DATA_BITS % 8);
    }
    else {
      _elem_init_data[b] = (-((_elem_init_data[ELEMENT_INIT_DATA_BITS / 8 + MIN(1, ELEMENT_INIT_DATA_BITS % 8) - 1] >> ((ELEMENT_INIT_DATA_BITS-1) % 8)) & elem_init_sign_extend));
    }
  }


  transfer_block_width = left_padding_amount + logical_block_width + right_padding_amount;

  if   (written_block == 1) {
    _left_margin = 0;
    wl           = 0;
    wu           = transfer_block_width;
  }
  else {
    _left_margin = left_margin;
    wl           = left_padding_amount;
    wu           = left_padding_amount + logical_block_width;
  }

  i = 0;
  row_addr = start_addr;

  elems_per_word = data_bits / precision_bits;
  width_in_bytes = (logical_block_width / elems_per_word) * BYTES(data_bits) + BYTES((logical_block_width % elems_per_word) * precision_bits);

  for (h = 0; h < logical_block_height; h++) {
    word_idx = 0;

    bit_idx = subsampling_offset * precision_bits + _left_margin * subsampling_factor * precision_bits;

    bits_remaining = 0;
    prev_bits_remaining = 0;
    for (w = wl; w < wu; w++) {

      bits_available = 0;
      bits_required = precision_bits;
      j = 0;
      while (bits_required > 0) {

        addr = row_addr + word_idx * BYTES(data_bits) + (bit_idx / 8);

        if (bits_remaining == 0) {

          bits_remaining = 8 - (bit_idx % 8);
          if      (mem_id == 1) {
//             byte = _hrt_slave_port_load_8(HRTCAT(MEMORY1,_ip0), addr);
          }
          else if (mem_id == 2) {
//             byte = _hrt_slave_port_load_8(HRTCAT(MEMORY2,_ip0), addr);
          }
          byte = byte >> (8 - bits_remaining);

        }

        bits_consumed = MIN(bits_required, bits_remaining - bits_available);

        elem[w][h][j] = (elem[w][h][j] & ~(-1 << bits_available)) | ((byte & ~(-1 << bits_consumed)) << bits_available);

        byte = byte >> bits_consumed;

        bits_available += bits_consumed;

        if (bits_available == 8) {
          j++;
          bits_available = 0;
        };

        bits_required -= bits_consumed;
        prev_bits_remaining = bits_remaining;
        bits_remaining -= bits_consumed;

        bit_idx += bits_consumed;

      }

      /* jump to next bit index based on subsampling factor setting */
      incr = (subsampling_factor - 1) * precision_bits;
      bit_idx += incr;

      if (incr > bits_remaining) {
        prev_bits_remaining = 0;
        bits_remaining = 0;
      }
      else {
        bits_remaining -= incr;
        byte = byte >> incr;
      }

      /* skip over bit-padding at the end of a data word */
      if (bit_idx + precision_bits > data_bits) {
        word_idx++;
        bit_idx += (data_bits % precision_bits); /* add padding at end of the data word to bit index */
        bit_idx = bit_idx % data_bits; /* finally compute bit index modulo the data word width */

        bits_remaining = 0;
      }
      PRINT("read elem[%d][%d]=", w, h);
      for (j = BYTES(precision_bits) - 1; j >= 0; j--) {
        PRINT("%02x ", elem[w][h][j]);
      }
      PRINT("\n");
      i++;
    }
    row_addr += stride;
  }

  // extend a block read by the DMA with optional padding to allow comparison to the block written by the DMA
  if (written_block == 0) {
    for (h = 0; h < logical_block_height; h++) {
      for (l = 0; l < left_padding_amount; l++) {
        for (j = 0; j < BYTES(precision_bits); j++) {
          switch (padding_mode) {
            case 0 : // constant padding
              elem[l][h][j] = _elem_init_data[j];
              break;
            case 1 : // clone padding
              elem[l][h][j] = elem[left_padding_amount][h][j];
              break;
            case 2 : // mirror padding
              elem[l][h][j] = elem[2 * left_padding_amount - l - 1][h][j];
              break;
            default :
              break;
          }
        }
      }
      for (r = 0; r < right_padding_amount; r++) {
        for (j = 0; j < BYTES(precision_bits); j++) {
          switch (padding_mode) {
            case 0 : // constant padding
              elem[left_padding_amount + logical_block_width + r][h][j] = _elem_init_data[j];
              break;
            case 1 : // clone padding
              elem[left_padding_amount + logical_block_width + r][h][j] = elem[left_padding_amount + logical_block_width - 1][h][j];
              break;
            case 2 : // mirror padding
              elem[left_padding_amount + logical_block_width + right_padding_amount - r - 1][h][j] = elem[left_padding_amount + logical_block_width - right_padding_amount + r][h][j];
              break;
            default :
              break;
          }
        }
      }
    }
  }
}




typedef enum {
  zero,
  pass,
  trunc_,
  extend,
  full_extend,
  partial_extend
} convertT;


char convert_byte(
  convertT convert,
  int precision_bits1,
  int precision_bits2,
  char cur_elem,
  char msb_elem,
  int sign_extend
)
{
  char byte;
  int offset1;
  int offset2;
  switch(convert) {
    case zero :
      byte = 0;
      break;
    case pass :
      byte = cur_elem;
      break;
    case trunc_ :
      offset2 = (precision_bits2 % 8);
      byte = cur_elem & ~(-1 << offset2);
      break;
    case extend :
      offset1 = (precision_bits1 % 8);
      if (offset1 == 0) {
        byte = msb_elem;
      }
      else {
//printf("msb_elem = %02x; offset1=%d; sign_extend=%d\n", msb_elem, offset1, sign_extend);
        byte = (-((msb_elem >> (offset1 - 1)) & sign_extend) & (-1 << offset1)) | (msb_elem & ~(-1 << offset1));
//        byte = (msb_elem & ~(-1 << offset1)) | (((msb_elem << (8-offset1)) & (sign_extend << 7)) >> (8-offset1));
      }
      break;
    case full_extend :
      offset1 = (precision_bits1 % 8 == 0) ? 8 : precision_bits1 % 8;
      byte = -((msb_elem >> (offset1 - 1)) & sign_extend);
      break;
    case partial_extend :
      offset1 = (precision_bits1 % 8 == 0) ? 8 : precision_bits1 % 8;
      offset2 = (precision_bits2 % 8 == 0) ? 8 : precision_bits2 % 8;
//      offset2 = (precision_bits2 % 8);
      byte = -((msb_elem >> (offset1 - 1)) & sign_extend) & ~(-1 << offset2);
//printf("offset1=%d; offset2=%d; byte=%02x; msb_elem=%02x, sign_extend=%d\n", offset1, offset2, byte,msb_elem, sign_extend);
//printf("(msb_elem >> (offset1 - 1)=%02x; -((msb_elem >> (offset1 - 1)) & sign_extend)=%02x; ~(-1 << offset2)=%02x\n", (msb_elem >> (offset1 - 1)), -((msb_elem >> (offset1 - 1)) & sign_extend),~(-1 << offset2));
      break;
    default :
      break;
  }
  return byte;
}




int check_operation_transfer (
  request_descriptor_t  request_descriptor,
  unit_descriptor_t     unit_descriptor,
  span_descriptor_t     span_descriptor_a,
  span_descriptor_t     span_descriptor_b,
  terminal_descriptor_t terminal_descriptor_a,
  terminal_descriptor_t terminal_descriptor_b,
  channel_descriptor_t  channel_descriptor
)
{

  int errors;
  int i, b, h, w;

  convertT convert_a, convert_b;

  char byte_a, byte_b;

  int start_addr_a, start_addr_b;
  int stride_a = (int) terminal_descriptor_a.region_stride;
  int stride_b = (int) terminal_descriptor_b.region_stride;
  int data_bits_a, data_bits_b;
  int precision_a = (int) get_precision(terminal_descriptor_a.element_setup);
  int precision_b = (int) get_precision(terminal_descriptor_b.element_setup);
  int elem_init_data        = (int) channel_descriptor.element_init_data;
  int sign_extend = (channel_descriptor.element_extend_mode == extend_mode_sign);
  int left_margin         ;
  int left_padding_amount ;
  int right_padding_amount;
  int padding_mode = (int) channel_descriptor.padding_mode;
  int logical_block_width ;
  int logical_block_height  = (int) get_logical_block_height(                    unit_descriptor);
  int transfer_block_width;

  int subsampling_factor_a, subsampling_factor_b;
  int subsampling_offset_a, subsampling_offset_b;

  transfer_direction_t transfer_direction;

  if (is_execute_instruction(request_descriptor.instruction)) {
    transfer_direction = get_transfer_direction(request_descriptor.instruction);
  }
  else {
    PERROR("instruction is 'invalidate' format.\n");
    exit(1);
  }


  if (transfer_direction == transfer_direction_ab) {
    start_addr_a         = (int) get_read_start_addr (request_descriptor, unit_descriptor, span_descriptor_a, terminal_descriptor_a, channel_descriptor);
    start_addr_b         = (int) get_write_start_addr(request_descriptor,                  span_descriptor_b, terminal_descriptor_b, channel_descriptor);
    data_bits_a          = (int) get_read_data_bits  (get_transfer_direction(request_descriptor.instruction));
    data_bits_b          = (int) get_write_data_bits (get_transfer_direction(request_descriptor.instruction));
    subsampling_factor_a = (int) get_subsampling_factor(channel_descriptor.sampling_setup);
    subsampling_factor_b = 1;
    // note the source defines the left-margin, padding amounts, logical block width, and subsampling offset
    left_margin          = (int) get_left_margin         (request_descriptor, unit_descriptor, span_descriptor_a, terminal_descriptor_a, channel_descriptor);
    left_padding_amount  = (int) get_left_padding_amount (                                     span_descriptor_a,                      channel_descriptor);
    right_padding_amount = (int) get_right_padding_amount(                    unit_descriptor, span_descriptor_a, terminal_descriptor_a, channel_descriptor);
    logical_block_width  = (int) get_logical_block_width (                    unit_descriptor, span_descriptor_a, terminal_descriptor_a, channel_descriptor);
    subsampling_offset_a = (int) get_subsampling_offset(request_descriptor, unit_descriptor, span_descriptor_a, terminal_descriptor_a, channel_descriptor);
    subsampling_offset_b = 0;
  }
  else {
    start_addr_a         = (int) get_write_start_addr(request_descriptor,                  span_descriptor_a, terminal_descriptor_a, channel_descriptor);
    start_addr_b         = (int) get_read_start_addr (request_descriptor, unit_descriptor, span_descriptor_b, terminal_descriptor_b, channel_descriptor);
    data_bits_a          = (int) get_write_data_bits (get_transfer_direction(request_descriptor.instruction));
    data_bits_b          = (int) get_read_data_bits  (get_transfer_direction(request_descriptor.instruction));
    subsampling_factor_a = 1;
    subsampling_factor_b = (int) get_subsampling_factor(channel_descriptor.sampling_setup);
    // note the source defines the left-margin, padding amounts, logical block width, and subsampling offset
    left_margin          = (int) get_left_margin         (request_descriptor, unit_descriptor, span_descriptor_b, terminal_descriptor_b, channel_descriptor);
    left_padding_amount  = (int) get_left_padding_amount (                                     span_descriptor_b,                        channel_descriptor);
    right_padding_amount = (int) get_right_padding_amount(                    unit_descriptor, span_descriptor_b, terminal_descriptor_b, channel_descriptor);
    logical_block_width  = (int) get_logical_block_width (                    unit_descriptor, span_descriptor_b, terminal_descriptor_b, channel_descriptor);
    subsampling_offset_a = 0;
    subsampling_offset_b = (int) get_subsampling_offset(request_descriptor, unit_descriptor, span_descriptor_b, terminal_descriptor_b, channel_descriptor);
  }

/* increase logical block width and logical block width by 1 since encoded values are 1 less */
logical_block_width++;
logical_block_height++;

PRINT("load block with derived parameters:\n");
PRINT("  start addr_a         = 0x%08x\n", start_addr_a);
PRINT("  start addr_b         = 0x%08x\n", start_addr_b);
PRINT("  precision a          = %d\n", precision_a);
PRINT("  precision b          = %d\n", precision_b);
PRINT("  data bits_a          = %d\n", data_bits_a);
PRINT("  data bits_b          = %d\n", data_bits_b);
PRINT("  logical block width  = %d\n", logical_block_width);
PRINT("  logical block height = %d\n", logical_block_height);
PRINT("  left margin          = %d\n", left_margin);
PRINT("  left padding amount  = %d\n", left_padding_amount);
PRINT("  right padding amount = %d\n", right_padding_amount);
PRINT("  subsampling factor_a = %d\n", subsampling_factor_a);
PRINT("  subsampling factor_b = %d\n", subsampling_factor_b);
PRINT("  subsampling offset_a = %d\n", subsampling_offset_a);
PRINT("  subsampling offset_b = %d\n", subsampling_offset_b);


  int direction;

  if (transfer_direction == transfer_direction_ab) {
    direction = 1;
  }
  else {
    direction = 0;
  }

  transfer_block_width = (left_padding_amount + logical_block_width + right_padding_amount);

  /* allocate memory to store elements contained in blocks at source and destination */
  char*** elem_a = (char***) ia_css_cpu_mem_alloc(transfer_block_width * sizeof (char**));
  char*** elem_b = (char***) ia_css_cpu_mem_alloc(transfer_block_width * sizeof (char**));
  for (w = 0; w < transfer_block_width; w++) {
    elem_a[w] = (char**) ia_css_cpu_mem_alloc(logical_block_height * sizeof(char*));
    elem_b[w] = (char**) ia_css_cpu_mem_alloc(logical_block_height * sizeof(char*));
    for (h = 0; h < logical_block_height; h++) {
      elem_a[w][h] = (char*) ia_css_cpu_mem_alloc(BYTES(precision_a) * sizeof(char));
      elem_b[w][h] = (char*) ia_css_cpu_mem_alloc(BYTES(precision_b) * sizeof(char));
    }
  }

  /* load block 1 */
  load_block(
    elem_a,
    data_bits_a,
    start_addr_a,
    stride_a,
    precision_a,
    subsampling_factor_a,
    subsampling_offset_a,
    logical_block_width,
    logical_block_height,
    left_margin,
    left_padding_amount,
    right_padding_amount,
    padding_mode,
    sign_extend,
    elem_init_data,
    1,
    1 - direction
  );

  /* load block 2 */
  load_block(
    elem_b,
    data_bits_b,
    start_addr_b,
    stride_b,
    precision_b,
    subsampling_factor_b,
    subsampling_offset_b,
    logical_block_width,
    logical_block_height,
    left_margin,
    left_padding_amount,
    right_padding_amount,
    padding_mode,
    sign_extend,
    elem_init_data,
    2,
    direction
  );

  /* note comparison assumes data is moved from memory 1 to memory 2 and thus assumes extension/truncation dependent on corresponding precisions accordingly */
  i = 0;
  errors = 0;
  for (h = 0; h < logical_block_height; h++) {
    for (w = 0; w < transfer_block_width; w++) {
      for (b = 0; b < MAX(BYTES(precision_a), BYTES(precision_b)); b++) {

        if      ((b < BYTES(precision_a) - 1) && (b < BYTES(precision_b) - 1)) {
          convert_a = pass;
          convert_b = pass;
        }
        else if ((b < BYTES(precision_a) - 1) && (b == BYTES(precision_b) - 1)) {
          if (direction == 0) {
            convert_a = pass;
            convert_b = extend;
          }
          else {
            convert_a = (precision_b % 8 == 0) ? pass : trunc_;
            convert_b = pass;
          }
        }
        else if ((b < BYTES(precision_a) - 1) && (b > BYTES(precision_b) - 1)) {
          if (direction == 0) {
            convert_a = pass;
            convert_b = full_extend;
          }
          else {
            convert_a = zero;
            convert_b = zero;
          }
        }
        else if ((b == BYTES(precision_a) - 1) && (b < BYTES(precision_b) - 1)) {
          if (direction == 0) {
            convert_a = pass;
            convert_b = (precision_a % 8 == 0) ? pass : trunc_;
          }
          else {
            convert_a = extend;
            convert_b = pass;
          }
        }
        else if ((b == BYTES(precision_a) - 1) && (b == BYTES(precision_b) - 1)) {
          if (precision_a < precision_b) {
            convert_a = extend;
            convert_b = pass;
          }
          else if (precision_a == precision_b) {
            convert_a = pass;
            convert_b = pass;
          }
          else if (precision_a > precision_b) {
            convert_a = trunc_;
            convert_b = pass;
          }
        }
        else if ((b == BYTES(precision_a) - 1) && (b > BYTES(precision_b) - 1)) {
          if (direction == 0) {
            convert_a = pass;
            convert_b = partial_extend;
          }
          else {
            convert_a = zero;
            convert_b = zero;
          }
        }
        else if ((b > BYTES(precision_a) - 1) && (b < BYTES(precision_b) - 1)) {
          if (direction == 0) {
            convert_a = zero;
            convert_b = zero;
          }
          else {
            convert_a = full_extend;
            convert_b = pass;
          }
        }
        else if ((b > BYTES(precision_a) - 1) && (b == BYTES(precision_b) - 1)) {
          if (direction == 0) {
            convert_a = zero;
            convert_b = zero;
          }
          else {
            convert_a = partial_extend;
            convert_b = pass;
          }
        }

        byte_a = convert_byte(convert_a, precision_a, precision_b, elem_a[w][h][b], elem_a[w][h][BYTES(precision_a) - 1], sign_extend);
        byte_b = convert_byte(convert_b, precision_b, precision_a, elem_b[w][h][b], elem_b[w][h][BYTES(precision_b) - 1], sign_extend);

        if (byte_a != byte_b) {
          PRINT("mismatch elem_a[%d][%d][%d]=%02x (relevant bits) <=> elem_b[%d][%d][%d]=%02x (relevant bits)\n", w, h, b, byte_a, w,h, b, byte_b);
          errors++;
        }
      }
      i++;
    }
  }

  // de-allocate memory
  for (w = 0; w < transfer_block_width; w++) {
    for (h = 0; h < logical_block_height; h++) {
      ia_css_cpu_mem_free(elem_a[w][h]);
      ia_css_cpu_mem_free(elem_b[w][h]);
    }
    ia_css_cpu_mem_free(elem_a[w]);
    ia_css_cpu_mem_free(elem_b[w]);
  }
  ia_css_cpu_mem_free(elem_a);
  ia_css_cpu_mem_free(elem_b);

  if (errors == 0) {
    PRINT(" results are correct\n");
  } else {
    PRINT(" results are wrong\n");
  }

  return errors;
}


int check_instruction_transfer (
  request_descriptor_t  request_descriptor,
  unit_descriptor_t     unit_descriptor,
  span_descriptor_t     span_descriptor_a,
  span_descriptor_t     span_descriptor_b,
  terminal_descriptor_t terminal_descriptor_a,
  terminal_descriptor_t terminal_descriptor_b,
  channel_descriptor_t  channel_descriptor
)
{
  int macro_size = get_macro_size(request_descriptor.instruction);

  int o;

  int errors = 0;

  for (o = 0; o <= macro_size; o++) {

    PRINT("checking results of operation %d\n", o);

    errors += check_operation_transfer(request_descriptor, unit_descriptor, span_descriptor_a, span_descriptor_b, terminal_descriptor_a, terminal_descriptor_b, channel_descriptor);

    PRINT("span descriptor A before update:\n");
    print_span_descriptor(span_descriptor_a);
    update_span_descriptor(&span_descriptor_a, unit_descriptor);
    PRINT("span_descriptor A after update:\n");
    print_span_descriptor(span_descriptor_a);
    PRINT("\n");
    PRINT("span descriptor B before update:\n");
    print_span_descriptor(span_descriptor_b);
    update_span_descriptor(&span_descriptor_b, unit_descriptor);
    PRINT("span descriptor B before update:\n");
    print_span_descriptor(span_descriptor_b);
  }
  return errors;
}
