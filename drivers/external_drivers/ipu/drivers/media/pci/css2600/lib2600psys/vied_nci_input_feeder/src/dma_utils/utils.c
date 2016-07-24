#include <hrt/api.h>
#include <hrt/system.h>

#include "utils.h"

#define USE_LEFT_MARGIN 1
//#define dma_s0_error_message dma_dma_ex_s0_error_message
//#define dma_s0_master_port_address dma_dma_ex_s0_master_port_address


int executed = 0;

// void initialize_memory1_block(int start_addr, int block_width, int block_height, int stride)
// {
//   int x, y, addr, row_addr;
//   char data;
//
//   data = 0x0;
//   row_addr = start_addr;
//   for (y = 0; y < block_height; y++) {
//     addr = row_addr;
//     for (x = 0; x < block_width; x++) {
//       _hrt_slave_port_store_8(HRTCAT(MEMORY1,_ip0), addr, data);
//       //printf("store data1[%05x]=%02x\n", addr, data);
//
//       addr += 1;
//       data += 1;
//     }
//     row_addr += stride;
//   }
// }

// void initialize_memory2_block(int start_addr, int block_width, int block_height, int stride)
// {
//   int x, y, addr, row_addr;
//   char data;
//
//   data = 0x0;
//   row_addr = start_addr;
//   for (y = 0; y < block_height; y++) {
//     addr = row_addr;
//     for (x = 0; x < block_width; x++) {
//       _hrt_slave_port_store_8(HRTCAT(MEMORY2,_ip0), addr, data);
// 			 //printf("store data2[%05x]=%02x\n", addr, data);
//       addr += 1;
//       data += 1;
//     }
//     row_addr += stride;
//   }
// }


// typedef enum {
//   zero,
//   pass,
//   trunc,
//   extend,
//   full_extend,
//   partial_extend
// } convertT;
//
//
// char convert_byte(
//   convertT convert,
//   int precision_bits1,
//   int precision_bits2,
//   char cur_elem,
//   char msb_elem,
//   int sign_extend
// )
// {
//   char byte;
//   int offset1;
//   int offset2;
//   switch(convert) {
//     case zero :
//       byte = 0;
//       break;
//     case pass :
//       byte = cur_elem;
//       break;
//     case trunc :
//       offset2 = (precision_bits2 % 8);
//       byte = cur_elem & ~(-1 << offset2);
//       break;
//     case extend :
//       offset1 = (precision_bits1 % 8);
//       if (offset1 == 0) {
//         byte = msb_elem;
//       }
//       else {
// //printf("msb_elem = %02x; offset1=%d; sign_extend=%d\n", msb_elem, offset1, sign_extend);
//         byte = (-((msb_elem >> (offset1 - 1)) & sign_extend) & (-1 << offset1)) | (msb_elem & ~(-1 << offset1));
// //        byte = (msb_elem & ~(-1 << offset1)) | (((msb_elem << (8-offset1)) & (sign_extend << 7)) >> (8-offset1));
//       }
//       break;
//     case full_extend :
//       offset1 = (precision_bits1 % 8 == 0) ? 8 : precision_bits1 % 8;
//       byte = -((msb_elem >> (offset1 - 1)) & sign_extend);
//       break;
//     case partial_extend :
//       offset1 = (precision_bits1 % 8 == 0) ? 8 : precision_bits1 % 8;
//       offset2 = (precision_bits2 % 8 == 0) ? 8 : precision_bits2 % 8;
// //      offset2 = (precision_bits2 % 8);
//       byte = -((msb_elem >> (offset1 - 1)) & sign_extend) & ~(-1 << offset2);
// //printf("offset1=%d; offset2=%d; byte=%02x; msb_elem=%02x, sign_extend=%d\n", offset1, offset2, byte,msb_elem, sign_extend);
// //printf("(msb_elem >> (offset1 - 1)=%02x; -((msb_elem >> (offset1 - 1)) & sign_extend)=%02x; ~(-1 << offset2)=%02x\n", (msb_elem >> (offset1 - 1)), -((msb_elem >> (offset1 - 1)) & sign_extend),~(-1 << offset2));
//       break;
//     default :
//       break;
//   }
//   return byte;
// }
//
//
// void load_block(
//   char ***elem,
//   int data_bits,
//   int start_addr,
//   int stride,
//   int precision_bits,
//   int subsampling_factor,
//   int subsampling_offset,
//   int logical_block_width,
//   int logical_block_height,
//   int left_margin,
//   int left_padding_amount,
//   int right_padding_amount,
//   int padding_mode,
//   int elem_init_sign_extend,
//   int elem_init_data,
//   int mem_id,
//   int written_block /* 1, if this block represents a block written by the DMA; 0, if this block represents a block read by the DMA */
// )
// {
//   int row_addr, addr;
//   int row_addr_lsbs;
//   int elems_per_word;
//   int width_in_bytes;
//   int word_idx, bit_idx;
//   int bits_remaining, prev_bits_remaining, bits_available, bits_required, bits_consumed;
//   int i, j, h, w, l, r;
//   char byte;
//   int incr;
//
//   int _left_margin;
//
//   int transfer_block_width;
//
//   int wl, wu;
//   int b;
//
//   /* array of bytes to encode element initialization data converted to the target element precision */
//   char *_elem_init_data = malloc(BYTES(precision_bits) * sizeof(char));
//
//   for (b = 0; b < BYTES(precision_bits); b++) {
//     if (b < ELEMENT_INIT_DATA_BITS / 8) {
//       _elem_init_data[b] = (elem_init_data >> (b * 8)) & 0xFF;
//     }
//     else if (b == ELEMENT_INIT_DATA_BITS / 8) {
//       _elem_init_data[b]  =    (elem_init_data >> (b * 8)) & ~(-1 << (ELEMENT_INIT_DATA_BITS % 8));
//       // sign/zero-extend
//       _elem_init_data[b] |= (-((_elem_init_data[b] >> ((ELEMENT_INIT_DATA_BITS-1) % 8)) & elem_init_sign_extend)) << (ELEMENT_INIT_DATA_BITS % 8);
//     }
//     else {
//       _elem_init_data[b] = (-((_elem_init_data[ELEMENT_INIT_DATA_BITS / 8 + MIN(1, ELEMENT_INIT_DATA_BITS % 8) - 1] >> ((ELEMENT_INIT_DATA_BITS-1) % 8)) & elem_init_sign_extend));
//     }
//   }
//
//
//   transfer_block_width = left_padding_amount + logical_block_width + right_padding_amount;
//
//   if   (written_block == 1) {
//     _left_margin = 0;
//     wl           = 0;
//     wu           = transfer_block_width;
//   }
//   else {
//     _left_margin = left_margin;
//     wl           = left_padding_amount;
//     wu           = left_padding_amount + logical_block_width;
//   }
//
//   i = 0;
//   row_addr = start_addr;
//
//   elems_per_word = data_bits / precision_bits;
//   width_in_bytes = (logical_block_width / elems_per_word) * BYTES(data_bits) + BYTES((logical_block_width % elems_per_word) * precision_bits);
//
//   for (h = 0; h < logical_block_height; h++) {
//     word_idx = 0;
//
//     bit_idx = subsampling_offset * precision_bits + _left_margin * subsampling_factor * precision_bits;
//
//     bits_remaining = 0;
//     prev_bits_remaining = 0;
//     for (w = wl; w < wu; w++) {
//
//       bits_available = 0;
//       bits_required = precision_bits;
//       j = 0;
//       while (bits_required > 0) {
//
//         addr = row_addr + word_idx * BYTES(data_bits) + (bit_idx / 8);
//
//         if (bits_remaining == 0) {
//
//           bits_remaining = 8 - (bit_idx % 8);
//           if      (mem_id == 1) {
//             byte = _hrt_slave_port_load_8(HRTCAT(MEMORY1,_ip0), addr);
//           }
//           else if (mem_id == 2) {
//             byte = _hrt_slave_port_load_8(HRTCAT(MEMORY2,_ip0), addr);
//           }
//           byte = byte >> (8 - bits_remaining);
//
//         }
//
//         bits_consumed = MIN(bits_required, bits_remaining - bits_available);
//
// #if 1
//         elem[w][h][j] = (elem[w][h][j] & ~(-1 << bits_available)) | ((byte & ~(-1 << bits_consumed)) << bits_available);
// #else
//         elem[i][j] = (elem[i][j] & ~(-1 << bits_available)) | ((byte & ~(-1 << bits_consumed)) << bits_available);
// #endif
//
//         byte = byte >> bits_consumed;
//
//         bits_available += bits_consumed;
//
//         if (bits_available == 8) {
//           j++;
//           bits_available = 0;
//         };
//
//         bits_required -= bits_consumed;
//         prev_bits_remaining = bits_remaining;
//         bits_remaining -= bits_consumed;
//
//         bit_idx += bits_consumed;
//
//       }
//
//       /* jump to next bit index based on subsampling factor setting */
//       incr = (subsampling_factor - 1) * precision_bits;
//       bit_idx += incr;
//
//       if (incr > bits_remaining) {
//         prev_bits_remaining = 0;
//         bits_remaining = 0;
//       }
//       else {
//         bits_remaining -= incr;
//         byte = byte >> incr;
//       }
//
//       /* skip over bit-padding at the end of a data word */
//       if (bit_idx + precision_bits > data_bits) {
//         word_idx++;
//         bit_idx += (data_bits % precision_bits); /* add padding at end of the data word to bit index */
//         bit_idx = bit_idx % data_bits; /* finally compute bit index modulo the data word width */
//
//         bits_remaining = 0;
//       }
//       printf("read elem[%d][%d]=", w, h);
//       for (j = BYTES(precision_bits) - 1; j >= 0; j--) {
// #if 1
//         printf("%02x ", elem[w][h][j]);
// #else
//         printf("%02x ", elem[i][j]);
// #endif
//       }
//       printf("\n");
//       i++;
//     }
//     row_addr += stride;
//   }
//
//   // extend a block read by the DMA with optional padding to allow comparison to the block written by the DMA
//   if (written_block == 0) {
//     for (h = 0; h < logical_block_height; h++) {
//       for (l = 0; l < left_padding_amount; l++) {
//         for (j = 0; j < BYTES(precision_bits); j++) {
//           switch (padding_mode) {
//             case 0 : // constant padding
//               elem[l][h][j] = _elem_init_data[j];
//               break;
//             case 1 : // clone padding
//               elem[l][h][j] = elem[left_padding_amount][h][j];
//               break;
//             case 2 : // mirror padding
//               elem[l][h][j] = elem[2 * left_padding_amount - l - 1][h][j];
//               break;
//             default :
//               break;
//           }
//         }
//       }
//       for (r = 0; r < right_padding_amount; r++) {
//         for (j = 0; j < BYTES(precision_bits); j++) {
//           switch (padding_mode) {
//             case 0 : // constant padding
//               elem[left_padding_amount + logical_block_width + r][h][j] = _elem_init_data[j];
//               break;
//             case 1 : // clone padding
//               elem[left_padding_amount + logical_block_width + r][h][j] = elem[left_padding_amount + logical_block_width - 1][h][j];
//               break;
//             case 2 : // mirror padding
//               elem[left_padding_amount + logical_block_width + right_padding_amount - r - 1][h][j] = elem[left_padding_amount + logical_block_width - right_padding_amount + r][h][j];
//               break;
//             default :
//               break;
//           }
//         }
//       }
//     }
//   }
//
// }
//
//
// int compare_mem1_mem2_block (
//   int logical_block_width, int logical_block_height,
//   int data_bits1, int data_bits2,
//   int start_addr1, int stride1, int precision_bits1, int subsampling_factor1, int subsampling_offset1,
//   int start_addr2, int stride2, int precision_bits2, int subsampling_factor2, int subsampling_offset2,
//   int sign_extend,
//   int direction,
//   int left_margin,
//   int left_padding_amount,
//   int right_padding_amount,
//   int padding_mode,
//   int elem_init_data,
//   int left_margin_enable,
//   int left_padding_enable,
//   int right_padding_enable,
//   int subsampling_enable
// )
// {
//
//   int errors;
//   int i, b, h, w;
//
//   convertT convert1, convert2;
//
//   char /*byte,*/ byte1, byte2;
//
//   printf (" results check...\n");
//
//   // allocate memory for loading blocks
//   int transfer_block_width = (left_padding_enable  ? left_padding_amount  : 0) + logical_block_width + (right_padding_enable ? right_padding_amount : 0);
//
//   char*** elem1 = (char***) malloc(transfer_block_width * sizeof (char**));
//   char*** elem2 = (char***) malloc(transfer_block_width * sizeof (char**));
//   for (w = 0; w < transfer_block_width; w++) {
//     elem1[w] = (char**) malloc(logical_block_height * sizeof(char*));
//     elem2[w] = (char**) malloc(logical_block_height * sizeof(char*));
//     for (h = 0; h < logical_block_height; h++) {
//       elem1[w][h] = (char*) malloc(BYTES(precision_bits1) * sizeof(char));
//       elem2[w][h] = (char*) malloc(BYTES(precision_bits2) * sizeof(char));
//     }
//   }
//
//   printf("precision1 bits %d bytes %d; precision2 bits %d bytes %d\n", precision_bits1, BYTES(precision_bits1), precision_bits2, BYTES(precision_bits2));
//
//   // assuming direction = 0 means moving data from A (memory1) to B (memory2); direction = 1 means moving data from B (memory2) to A (memory1)
//
//   /* load block 1 */
//   load_block(
//     elem1,
//     data_bits1,
//     start_addr1,
//     stride1,
//     precision_bits1,
//     (subsampling_enable ? subsampling_factor1 : 1),
//     (subsampling_enable ? subsampling_offset1 : 0),
//     logical_block_width,
//     logical_block_height,
//     (left_margin_enable   ? left_margin          : 0),
//     (left_padding_enable  ? left_padding_amount  : 0),
//     (right_padding_enable ? right_padding_amount : 0),
//     padding_mode,
//     sign_extend,
//     elem_init_data,
//     1,
//     1 - direction
//   );
//
//   /* load block 2 */
//   load_block(
//     elem2,
//     data_bits2,
//     start_addr2,
//     stride2,
//     precision_bits2,
//     (subsampling_enable ? subsampling_factor2 : 1),
//     (subsampling_enable ? subsampling_offset2 : 0),
//     logical_block_width,
//     logical_block_height,
//     (left_margin_enable   ? left_margin          : 0),
//     (left_padding_enable  ? left_padding_amount  : 0),
//     (right_padding_enable ? right_padding_amount : 0),
//     padding_mode,
//     sign_extend,
//     elem_init_data,
//     2,
//     direction
//   );
//
//   /* note comparison assumes data is moved from memory 1 to memory 2 and thus assumes extension/truncation dependent on corresponding precisions accordingly */
//   i = 0;
//   errors = 0;
//   for (h = 0; h < logical_block_height; h++) {
//     for (w = 0; w < transfer_block_width; w++) {
//       for (b = 0; b < MAX(BYTES(precision_bits1), BYTES(precision_bits2)); b++) {
//
//         if      ((b < BYTES(precision_bits1) - 1) && (b < BYTES(precision_bits2) - 1)) {
//           convert1 = pass;
//           convert2 = pass;
//         }
//         else if ((b < BYTES(precision_bits1) - 1) && (b == BYTES(precision_bits2) - 1)) {
//           if (direction == 0) {
//             convert1 = pass;
//             convert2 = extend;
//           }
//           else {
//             convert1 = (precision_bits2 % 8 == 0) ? pass : trunc;
//             convert2 = pass;
//           }
//         }
//         else if ((b < BYTES(precision_bits1) - 1) && (b > BYTES(precision_bits2) - 1)) {
//           if (direction == 0) {
//             convert1 = pass;
//             convert2 = full_extend;
//           }
//           else {
//             convert1 = zero;
//             convert2 = zero;
//           }
//         }
//         else if ((b == BYTES(precision_bits1) - 1) && (b < BYTES(precision_bits2) - 1)) {
//           if (direction == 0) {
//             convert1 = pass;
//             convert2 = (precision_bits1 % 8 == 0) ? pass : trunc;
//           }
//           else {
//             convert1 = extend;
//             convert2 = pass;
//           }
//         }
//         else if ((b == BYTES(precision_bits1) - 1) && (b == BYTES(precision_bits2) - 1)) {
//           if (precision_bits1 < precision_bits2) {
//             convert1 = extend;
//             convert2 = pass;
//           }
//           else if (precision_bits1 == precision_bits2) {
//             convert1 = pass;
//             convert2 = pass;
//           }
//           else if (precision_bits1 > precision_bits2) {
//             convert1 = trunc;
//             convert2 = pass;
//           }
//         }
//         else if ((b == BYTES(precision_bits1) - 1) && (b > BYTES(precision_bits2) - 1)) {
//           if (direction == 0) {
//             convert1 = pass;
//             convert2 = partial_extend;
//           }
//           else {
//             convert1 = zero;
//             convert2 = zero;
//           }
//         }
//         else if ((b > BYTES(precision_bits1) - 1) && (b < BYTES(precision_bits2) - 1)) {
//           if (direction == 0) {
//             convert1 = zero;
//             convert2 = zero;
//           }
//           else {
//             convert1 = full_extend;
//             convert2 = pass;
//           }
//         }
//         else if ((b > BYTES(precision_bits1) - 1) && (b == BYTES(precision_bits2) - 1)) {
//           if (direction == 0) {
//             convert1 = zero;
//             convert2 = zero;
//           }
//           else {
//             convert1 = partial_extend;
//             convert2 = pass;
//           }
//         }
//
// #if 1
//         byte1 = convert_byte(convert1, precision_bits1, precision_bits2, elem1[w][h][b], elem1[w][h][BYTES(precision_bits1) - 1], sign_extend);
//         byte2 = convert_byte(convert2, precision_bits2, precision_bits1, elem2[w][h][b], elem2[w][h][BYTES(precision_bits2) - 1], sign_extend);
// #else
//         byte1 = convert_byte(convert1, precision_bits1, precision_bits2, elem1[i][b], elem1[i][BYTES(precision_bits1) - 1], sign_extend);
//         byte2 = convert_byte(convert2, precision_bits2, precision_bits1, elem2[i][b], elem2[i][BYTES(precision_bits2) - 1], sign_extend);
// #endif
//
//         if (byte1 != byte2) {
//           printf("mismatch elem1[%d][%d]=%02x (relevant bits) <=> elem2[%d][%d]=%02x (relevant bits)\n", i, b, byte1, i, b, byte2);
//           errors++;
//         }
//       }
//       i++;
//     }
//   }
//
//   // de-allocate memory
// #if 1
//   for (w = 0; w < transfer_block_width; w++) {
//     for (h = 0; h < logical_block_height; h++) {
//       free(elem1[w][h]);
//       free(elem2[w][h]);
//     }
//     free(elem1[w]);
//     free(elem2[w]);
//   }
//   free(elem1);
//   free(elem2);
// #else
//   for (i = 0; i < logical_block_width * logical_block_height; i++) {
//     free(elem1[i]);
//     free(elem2[i]);
//   }
//   free(elem1);
//   free(elem2);
// #endif
//
//   if (errors == 0) {
//     printf (" results are correct\n");
//   } else {
//     printf (" results are wrong\n");
//   }
//
//   return errors;
// }
//
//
//
//
// void write_dimension_regs (
//   int dimension_id,
//   int logical_block_width,
//   int logical_block_height
// )
// {
//   int base_addr;
//   int bank_id = 4;
//
//   base_addr = (bank_id << REGSET_ID_LSB) + (dimension_id << BANK_ID_LSB);
// 	//base_addr = (bank_id << 6) + (dimension_id << BANK_ID_LSB);
//
//   dma_hrt_store_value(DMA_ID, base_addr + 0x0,  logical_block_width);
//   dma_hrt_store_value(DMA_ID, base_addr + 0x4,  logical_block_height);
// }
//
// void write_channel_regs (
//   int channel_id,
//   int logical_block_width,
//   int logical_block_height,
//   int logical_stride_a,
//   int logical_stride_b,
//   int packer,
//   int element_init_data
// )
// {
//   int base_addr;
//   int bank_id = 0;
//
//   base_addr = (bank_id << REGSET_ID_LSB) + (channel_id << BANK_ID_LSB);
//
// #warning in 'write_channel_regs' dimension_id currently hardcoded/derived from channel_id
//   write_dimension_regs(channel_id, logical_block_width, logical_block_height);
//
//   dma_hrt_store_value(DMA_ID, base_addr + 0x0, logical_stride_a);
//   dma_hrt_store_value(DMA_ID, base_addr + 0x4, logical_stride_b);
//   dma_hrt_store_value(DMA_ID, base_addr + 0x8, packer);
//   dma_hrt_store_value(DMA_ID, base_addr + 0xC, element_init_data);
// }
//
// void write_channel_subsampling_regs (
//   int channel_id,
//   int subsampling_index_a,
//   int subsampling_index_b,
//   int subsampling_offset_a,
//   int subsampling_offset_b
// )
// {
//   int base_addr;
//   int bank_id = 0;
//
//   base_addr = (bank_id << REGSET_ID_LSB) + (channel_id << BANK_ID_LSB);
//
//   int subsampling_value = (subsampling_offset_b << SUBSAMPLING_OFFSET_B_LSB) +
//                           (subsampling_offset_a << SUBSAMPLING_OFFSET_A_LSB) +
//                           (subsampling_index_b  << SUBSAMPLING_INDEX_B_LSB)  +
//                           (subsampling_index_a  << SUBSAMPLING_INDEX_A_LSB)  ;
//
//   printf("write subsampling_value = 0x%x\n", subsampling_value);
//
//   dma_hrt_store_value(DMA_ID, base_addr + 0x1c,  subsampling_value);
// }
//
// void write_margin_setup (
//   int channel_id,
//   int left_margin
// )
// {
//   int base_addr;
//   int bank_id = 0;
//
//   base_addr = (bank_id << REGSET_ID_LSB) + (channel_id << BANK_ID_LSB);
//
//   int margin_setup_value = left_margin;
//
//   dma_hrt_store_value(DMA_ID, base_addr + 0x20,  margin_setup_value);
// }
// void write_padding_setup (
//   int channel_id,
//   int left_padding_amount,
//   int right_padding_amount,
//   int padding_mode
// )
// {
//   int base_addr;
//   int bank_id = 0;
//
//   base_addr = (bank_id << REGSET_ID_LSB) + (channel_id << BANK_ID_LSB);
//
//   int padding_setup_value = (right_padding_amount << RIGHT_PADDING_AMOUNT_LSB) +
//                             (left_padding_amount  <<  LEFT_PADDING_AMOUNT_LSB) +
//                             (padding_mode         <<         PADDING_MODE_LSB);
//
//   printf("write padding_setup_value = 0x%x\n", padding_setup_value);
//
//   dma_hrt_store_value(DMA_ID, base_addr + 0x24,  padding_setup_value);
// }
//
// void write_ack_setup (
//   int channel_id,
//   int acknowledge_mode,
//   int acknowledge_addr,
//   int acknowledge_data
// )
// {
//   int base_addr;
//   int bank_id = 0;
//
//   base_addr = (bank_id << REGSET_ID_LSB) + (channel_id << BANK_ID_LSB);
//
//   dma_hrt_store_value(DMA_ID, base_addr + 0x28,  acknowledge_mode);
//   dma_hrt_store_value(DMA_ID, base_addr + 0x2C,  acknowledge_addr);
//   dma_hrt_store_value(DMA_ID, base_addr + 0x30,  acknowledge_data);
// }
//
// void store_descriptor_field(int *addr, int value, int bits)
// {
//   int b;
//   int bytes = (bits + 7) / 8;
//   char byte;
//
//   printf("storing descriptor field with value 0x%08x consisting of %d bytes at address 0x%08x\n", value, bytes, *addr);
//   for (b = 0; b < bytes; b++) {
//     byte = (value >> (b*8)) & 0xFF;
//     printf("  storing byte 0x%02x at address 0x%08x\n", byte, *addr);
//     _hrt_slave_port_store_8(HRTCAT(MEMORY1, _ip0), *addr, byte);
//     (*addr)++;
//   }
// }
//
// void write_virtual_channel_descriptor(
//   int virtual_channel_refill_base_addr,
//   int channel_id,
//   int stride_a,
//   int stride_b,
//   int precision_bits_a, int precision_bits_b,
//   int sign_extend,
//   int element_init_data,
//   int subsampling_factor_a,
//   int subsampling_factor_b,
//   int subsampling_offset_a,
//   int subsampling_offset_b,
//   int left_margin,
//   int left_padding_amount,
//   int right_padding_amount,
//   int padding_mode,
//   int acknowledge_mode,
//   int acknowledge_addr,
//   int acknowledge_data
// )
// {
//   int base_addr = virtual_channel_refill_base_addr + channel_id * CHANNEL_WORDS * 4;
//
//   int global_set_index;
//
//   int element_setup;
//
//   int margin_setup = left_margin;
//   int subsampling_index_a = subsampling_factor_2_subsampling_index(0, subsampling_factor_a);
//   int subsampling_index_b = subsampling_factor_2_subsampling_index(1, subsampling_factor_b);
//   int sampling_setup = (subsampling_offset_b << SUBSAMPLING_OFFSET_B_LSB) +
//                           (subsampling_offset_a << SUBSAMPLING_OFFSET_A_LSB) +
//                           (subsampling_index_b  << SUBSAMPLING_INDEX_B_LSB)  +
//                           (subsampling_index_a  << SUBSAMPLING_INDEX_A_LSB)  ;
//
//   int padding_setup = (right_padding_amount << RIGHT_PADDING_AMOUNT_LSB) +
//                             (left_padding_amount  <<  LEFT_PADDING_AMOUNT_LSB) +
//                             (padding_mode         <<         PADDING_MODE_LSB);
//
//   switch (precision_bits_a) {
//     case 5  : global_set_index = 1; break;
//     case 12 : global_set_index = 0; break;
//     default : global_set_index = 2; break;
//   }
//   element_setup = (global_set_index                                      << (PRECISION_B_BITS + PRECISION_A_BITS + 1)) +
//            (precision_bits_2_precision_index(1, precision_bits_b) <<                    (PRECISION_A_BITS + 1)) +
//            (precision_bits_2_precision_index(0, precision_bits_a) <<                                        1 ) +
//            sign_extend;
//
// #if 1
//   store_descriptor_field(&base_addr, stride_a,          STRIDE_A_BITS);
//   store_descriptor_field(&base_addr, stride_b,          STRIDE_B_BITS);
//   store_descriptor_field(&base_addr, element_setup,     ELEMENT_SETUP_BITS);
//   store_descriptor_field(&base_addr, element_init_data, ELEMENT_INIT_DATA_BITS);
//   store_descriptor_field(&base_addr, sampling_setup,    SAMPLING_SETUP_BITS);
//   store_descriptor_field(&base_addr, margin_setup,      MARGIN_SETUP_BITS);
//   store_descriptor_field(&base_addr, padding_setup,     PADDING_SETUP_BITS);
//   store_descriptor_field(&base_addr, acknowledge_mode,  ACKNOWLEDGE_MODE_BITS);
//   store_descriptor_field(&base_addr, acknowledge_addr,  ACKNOWLEDGE_ADDR_BITS);
//   store_descriptor_field(&base_addr, acknowledge_data,  ACKNOWLEDGE_DATA_BITS);
// #else
//   _hrt_slave_port_store_32(HRTCAT(MEMORY1, _ip0), base_addr + 0x00, logical_stride_a);
//   _hrt_slave_port_store_32(HRTCAT(MEMORY1, _ip0), base_addr + 0x04, logical_stride_b);
//   _hrt_slave_port_store_32(HRTCAT(MEMORY1, _ip0), base_addr + 0x08, packer);
//   _hrt_slave_port_store_32(HRTCAT(MEMORY1, _ip0), base_addr + 0x0C, element_init_data);
//   _hrt_slave_port_store_32(HRTCAT(MEMORY1, _ip0), base_addr + 0x1c, subsampling_value);
//   _hrt_slave_port_store_32(HRTCAT(MEMORY1, _ip0), base_addr + 0x20, margin_setup_value);
//   _hrt_slave_port_store_32(HRTCAT(MEMORY1, _ip0), base_addr + 0x24, padding_setup_value);
//   _hrt_slave_port_store_32(HRTCAT(MEMORY1, _ip0), base_addr + 0x28, acknowledge_mode);
//   _hrt_slave_port_store_32(HRTCAT(MEMORY1, _ip0), base_addr + 0x2C, acknowledge_addr);
//   _hrt_slave_port_store_32(HRTCAT(MEMORY1, _ip0), base_addr + 0x30, acknowledge_data);
// #endif
// }
//
// void write_virtual_dimension_descriptor(
//   int virtual_dimension_refill_base_addr,
//   int dimension_id,
//   int logical_block_width,
//   int logical_block_height
// )
// {
//   int base_addr = virtual_dimension_refill_base_addr + dimension_id * DIMENSION_WORDS * 4;
//
//   store_descriptor_field(&base_addr, logical_block_width,  LOGICAL_BLOCK_WIDTH_BITS );
//   store_descriptor_field(&base_addr, logical_block_height, LOGICAL_BLOCK_HEIGHT_BITS);
// }
//
//
//
// void read_dimension_regs (
//   int dimension_id,
//   int *logical_block_width,
//   int *logical_block_height
// )
// {
//   int base_addr;
//   int bank_id = 4;
//
//   base_addr = (bank_id << REGSET_ID_LSB) + (dimension_id << BANK_ID_LSB);
// 	//base_addr = (bank_id << 6) + (dimension_id << BANK_ID_LSB);
//
//   *logical_block_width  = dma_hrt_load_value(DMA_ID, base_addr + 0x0);
//   *logical_block_height = dma_hrt_load_value(DMA_ID, base_addr + 0x4);
// }
//
// void read_channel_regs (
//   int channel_id,
//   int *logical_block_width,
//   int *logical_block_height,
//   int *logical_stride_a,
//   int *logical_stride_b,
//   int *packer,
//   int *element_init_data
// )
// {
//   int base_addr;
//   int bank_id = 0;
//
//   base_addr = (bank_id << REGSET_ID_LSB) + (channel_id << BANK_ID_LSB);
//
//   read_dimension_regs(channel_id, logical_block_width, logical_block_height);
//
//   *logical_stride_a     = dma_hrt_load_value(DMA_ID, base_addr + 0x0);
//   *logical_stride_b     = dma_hrt_load_value(DMA_ID, base_addr + 0x4);
//   *packer               = dma_hrt_load_value(DMA_ID, base_addr + 0x8);
//   *element_init_data    = dma_hrt_load_value(DMA_ID, base_addr + 0xC);
// }
//
// void read_channel_subsampling_regs (
//   int channel_id,
//   int *subsampling_index_a,
//   int *subsampling_index_b,
//   int *subsampling_offset_a,
//   int *subsampling_offset_b
// )
// {
//   int base_addr;
//   int bank_id = 0;
//
//   base_addr = (bank_id << REGSET_ID_LSB) + (channel_id << BANK_ID_LSB);
//
//   int subsampling_value = dma_hrt_load_value(DMA_ID, base_addr + 0x1c);
//   printf("read subsampling_value = 0x%x\n", subsampling_value);
//
//   *subsampling_offset_b    = (subsampling_value >> SUBSAMPLING_OFFSET_B_LSB) & ~(-1 << SUBSAMPLING_OFFSET_B_BITS);
//   *subsampling_offset_a    = (subsampling_value >> SUBSAMPLING_OFFSET_A_LSB) & ~(-1 << SUBSAMPLING_OFFSET_A_BITS);
//   *subsampling_index_b     = (subsampling_value >>  SUBSAMPLING_INDEX_B_LSB) & ~(-1 << SUBSAMPLING_INDEX_B_BITS);
//   *subsampling_index_a     = (subsampling_value >>  SUBSAMPLING_INDEX_A_LSB) & ~(-1 << SUBSAMPLING_INDEX_A_BITS);
// }
// void read_margin_setup (
//   int channel_id,
//   int *left_margin
// )
// {
//   int base_addr;
//   int bank_id = 0;
//
//   base_addr = (bank_id << REGSET_ID_LSB) + (channel_id << BANK_ID_LSB);
//
//   int margin_setup_value = dma_hrt_load_value(DMA_ID, base_addr + 0x20);
//   printf("read margin_setup_value = 0x%x\n", margin_setup_value);
//
//   *left_margin = margin_setup_value;
// }
// void read_padding_setup (
//   int channel_id,
//   int *right_padding_amount,
//   int *left_padding_amount,
//   int *padding_mode
// )
// {
//   int base_addr;
//   int bank_id = 0;
//
//   base_addr = (bank_id << REGSET_ID_LSB) + (channel_id << BANK_ID_LSB);
//
//   int padding_setup_value = dma_hrt_load_value(DMA_ID, base_addr + 0x24);
//   printf("read padding_setup_value = 0x%x\n", padding_setup_value);
//
//   *right_padding_amount = (padding_setup_value >> RIGHT_PADDING_AMOUNT_LSB) & ~(-1 << RIGHT_PADDING_AMOUNT_BITS);
//   *left_padding_amount  = (padding_setup_value >>  LEFT_PADDING_AMOUNT_LSB) & ~(-1 <<  LEFT_PADDING_AMOUNT_BITS);
//   *padding_mode         = (padding_setup_value >>         PADDING_MODE_LSB) & ~(-1 <<         PADDING_MODE_BITS);
// }
//
// void write_master_regs (
//   int master_id,
//   int srmd_support,
//   /*int wmask_support,*/
//   int burst_support,
//   int max_stride
// )
// {
//   int base_addr;
//   int bank_id = 3;
//
//   base_addr = (bank_id << REGSET_ID_LSB) + (master_id << BANK_ID_LSB);
//
//   dma_hrt_store_value(DMA_ID, base_addr + 0x00, srmd_support);
//   //dma_hrt_store_value(DMA_ID, base_addr + 0x04, wmask_support);
//   dma_hrt_store_value(DMA_ID, base_addr + 0x08, burst_support);
//   dma_hrt_store_value(DMA_ID, base_addr + 0x0C, max_stride);
// }
//
// void read_master_regs (
//   int master_id,
//   int *srmd_support,
//   /*int *wmask_support,*/
//   int *burst_support,
//   int *max_stride
// )
// {
//   int base_addr;
//   int bank_id = 3;
//
//   base_addr = (bank_id << REGSET_ID_LSB) + (master_id << BANK_ID_LSB);
//
//   *srmd_support          = dma_hrt_load_value(DMA_ID, base_addr + 0x00);
//   //*wmask_support         = dma_hrt_load_value(DMA_ID, base_addr + 0x04);
//   *burst_support         = dma_hrt_load_value(DMA_ID, base_addr + 0x08);
//   *max_stride            = dma_hrt_load_value(DMA_ID, base_addr + 0x0C);
// }
//
// void clear_irq ()
// {
//   int base_addr;
//   int bank_id = 2;
//
//   base_addr = (bank_id << REGSET_ID_LSB);
//   dma_hrt_store_value(DMA_ID, base_addr + 0x00, 0);
// }
//
// void write_global_regs (
//   //int irq_mask,
//   int max_linear_burst_size,
//   int max_block_width,
//   int max_block_height,
//   int set
// )
// {
//   int base_addr;
//   int bank_id = 2;
//
//   base_addr = (bank_id << REGSET_ID_LSB);
//
//   // dma_hrt_store_value(DMA_ID, base_addr + 0x04, irq_mask);
//   dma_hrt_store_value(DMA_ID, base_addr + 0x08, max_block_height);
//   dma_hrt_store_value(DMA_ID, base_addr + 0x0C + (2 * set        ) * 4, max_linear_burst_size);
//   dma_hrt_store_value(DMA_ID, base_addr + 0x10 + (2 * set        ) * 4, max_block_width);
// }
//
// void write_virtual_refill_base_addr (
//   int virtual_channel_refill_base_addr,
//   int virtual_dimension_refill_base_addr
// )
// {
//   int base_addr;
//   int bank_id = 2;
//
//   base_addr = (bank_id << REGSET_ID_LSB);
//   dma_hrt_store_value(DMA_ID, base_addr + 0x14 + (2 * (GLOBAL_SETS - 1)) * 4, virtual_channel_refill_base_addr);
//   dma_hrt_store_value(DMA_ID, base_addr + 0x18 + (2 * (GLOBAL_SETS - 1)) * 4, virtual_dimension_refill_base_addr);
// }
//
// void read_global_regs (
//   //int *irq,
//   //int *irq_mask,
//   int *max_linear_burst_size,
//   int *max_block_width,
//   int *max_block_height,
//   int set
// )
// {
//   int base_addr;
//   int bank_id = 2;
//
//   base_addr = (bank_id << REGSET_ID_LSB);
//
//   // *irq                   = dma_hrt_load_value(DMA_ID, base_addr + 0x00);
//   // *irq_mask              = dma_hrt_load_value(DMA_ID, base_addr + 0x04);
//   *max_block_height      = dma_hrt_load_value(DMA_ID, base_addr + 0x08);
//   *max_linear_burst_size = dma_hrt_load_value(DMA_ID, base_addr + 0x0C + set*8);
//   *max_block_width       = dma_hrt_load_value(DMA_ID, base_addr + 0x10 + set*8);
// }
//
// void do_command (int requester_id, int command)
// {
//   int base_addr;
//   int bank_id = 1;
//
//   base_addr = (bank_id << REGSET_ID_LSB) + (requester_id << BANK_ID_LSB);
//
//   dma_hrt_store_value(DMA_ID, base_addr + 0x10, command);
// }
//
// int compose_command (
//   int channel_id,
//   int dimension_id,
//   int channel_invalidate,
//   int dimension_invalidate,
//   int margin_enable,
//   int left_padding_enable,
//   int right_padding_enable,
//   int subsampling_enable,
//   int command
// )
// {
//   int command_modifier = (margin_enable        << 3) +
//                          (left_padding_enable  << 2) +
//                          (right_padding_enable << 1) +
//                          (subsampling_enable   << 0);
//
//   return (channel_id           << CHANNEL_ID_LSB)           +
//          (dimension_id         << DIMENSION_ID_LSB)         +
//          (channel_invalidate   << CHANNEL_INVALIDATE_LSB)   +
//          (dimension_invalidate << DIMENSION_INVALIDATE_LSB) +
//          (command_modifier     << COMMAND_MODIFIER_LSB)     +
//           command;
// }
//
// void write_request_regs (
//   int requester_id,
//   int addr_a,
//   int addr_b,
//   int channel_id,
//   int dimension_id,
//   int channel_invalidate,
//   int dimension_invalidate,
//   int margin_enable,
//   int left_padding_enable,
//   int right_padding_enable,
//   int subsampling_enable,
//   int command
// )
// {
//   int base_addr;
//   int bank_id = 1;
//   int command_value;
//
//   base_addr = (bank_id << REGSET_ID_LSB) + (requester_id << BANK_ID_LSB);
//
//   dma_hrt_store_value(DMA_ID, base_addr + 0x0, addr_a);
//   dma_hrt_store_value(DMA_ID, base_addr + 0x4, addr_b);
// #if 1
//   command_value = compose_command(
//                     channel_id,
//                     dimension_id,
//                     channel_invalidate,
//                     dimension_invalidate,
//                     margin_enable,
//                     left_padding_enable,
//                     right_padding_enable,
//                     subsampling_enable,
//                     command
//                   );
//   dma_hrt_store_value(DMA_ID, base_addr + 0x10, command_value);
// #else
//   dma_hrt_store_value(DMA_ID, base_addr + 0x8, channel_id);
//   dma_hrt_store_value(DMA_ID, base_addr + 0xC, dimension_id);
//   dma_hrt_store_value(DMA_ID, base_addr + 0x10, command);
// #endif
// }
//
// void read_request_regs (
//   int requester_id,
//   int *addr_a,
//   int *addr_b,
//   int *channel_id,
//   int *dimension_id,
//   int *command
// )
// {
//   int base_addr;
//   int bank_id = 1;
//   int command_value;
//
//   base_addr = (bank_id << REGSET_ID_LSB) + (requester_id << BANK_ID_LSB);
//
//   *addr_a       = dma_hrt_load_value(DMA_ID, base_addr + 0x0);
//   *addr_b       = dma_hrt_load_value(DMA_ID, base_addr + 0x4);
//
//   command_value = dma_hrt_load_value(DMA_ID, base_addr + 0x10);
//
//   *command      = command_value & 0xF;
//   *channel_id   = (command_value >> CHANNEL_ID_LSB  ) & ~(-1 << CHANNEL_ID_BITS  );
//   *dimension_id = (command_value >> DIMENSION_ID_LSB) & ~(-1 << DIMENSION_ID_BITS);
// }

int read_completed_counter (
  int channel_id,
  int blocking
)
{
  int count;
  int base_addr;
  int bank_id = 0;

  base_addr = (bank_id << REGSET_ID_LSB) + (channel_id << BANK_ID_LSB);

  if (blocking) {
//     count = dma_hrt_load_value(DMA_ID, base_addr + 0x18);
    count = dma_hrt_load_value(DMA_BASE_ADDR, base_addr + 0x18);
  } else {
//     count = dma_hrt_load_value(DMA_ID, base_addr + 0x14);
    count = dma_hrt_load_value(DMA_BASE_ADDR, base_addr + 0x14);
  }

  return count;
}

/*
int precision_bits_2_precision_index(
  int master_id,
  int type
)
{
  if (master_id == 0) {
    switch(type) {
      case 8  : return 0; break;
      case 12 : return 1; break;
      case 5  : return 2; break;
      default : printf("unsupported precision type %d specified for master %d\n", type, master_id); break;
    }
  }
  else {
    switch(type) {
      case 8  : return 0; break;
      case 16 : return 1; break;
      default : printf("unsupported precision type %d specified for master %d\n", type, master_id); break;
    }
  }
  return 0;
}

int subsampling_factor_2_subsampling_index(
  int master_id,
  int factor
)
{
  if (master_id == 0) {
    switch(factor) {
      case 1 : return 0; break;
      case 2 : return 1; break;
      case 8 : return 2; break;
      case 4 : return 3; break;
      default : printf("unsupported subsampling factor %d specified for master %d\n", factor, master_id); exit(1); break;
    }
  }
  else {
    switch(factor) {
      case 1 : return 0; break;
      case 4 : return 1; break;
      default : printf("unsupported subsampling factor %d specified for master %d\n", factor, master_id); exit(1); break;
    }
  }
  return 0;
}


int command_direction(int command) {
  int direction;
  switch (command) {
    case 4  :
    case 5  :
    case 7  :
    case 12 :
    case 13 :
    case 15 :
      direction = 0;
      break;
    case 0  :
    case 1  :
    case 3  :
    case 8  :
    case 9  :
    case 11 :
      direction = 1;
      break;
    default :
      printf ("error in function 'command_direction': illegal command %d\n", command);
      break;
  }
  return direction;
}


void set_channel_bank_as_virtual(
  int channel_bank_id
)
{
  int base_addr;
  int group_id = 0;

  base_addr = (group_id << REGSET_ID_LSB) + (channel_bank_id << BANK_ID_LSB);

  // set virtual register, i.e. register with index 0xF (= 0x3C as 32-bit aligned address) to 1
  dma_hrt_store_value(DMA_ID, base_addr + 0x3C, 1);
}

void set_channel_bank_as_physical(
  int channel_bank_id
)
{
  int base_addr;
  int group_id = 0;

  base_addr = (group_id << REGSET_ID_LSB) + (channel_bank_id << BANK_ID_LSB);

  // set virtual register, i.e. register with index 0xF (= 0x3C as 32-bit aligned address) to 0
  dma_hrt_store_value(DMA_ID, base_addr + 0x3C, 0);
}

void set_dimension_bank_as_virtual(
  int dimension_bank_id
)
{
  int base_addr;
  int group_id = 4;

  base_addr = (group_id << REGSET_ID_LSB) + (dimension_bank_id << BANK_ID_LSB);

  // set virtual register, i.e. register with index 0xF (= 0x3C as 32-bit aligned address) to 1
  dma_hrt_store_value(DMA_ID, base_addr + 0x3C, 1);
}

void set_dimension_bank_as_physical(
  int dimension_bank_id
)
{
  int base_addr;
  int group_id = 4;

  base_addr = (group_id << REGSET_ID_LSB) + (dimension_bank_id << BANK_ID_LSB);

  // set virtual register, i.e. register with index 0xF (= 0x3C as 32-bit aligned address) to 0
  dma_hrt_store_value(DMA_ID, base_addr + 0x3C, 0);
}

int execute(
  int channel_id,
  int dimension_id,
  int requester_id,
  int command,
  int addr_a, int addr_b,
  int logical_stride_a, int logical_stride_b,
  int logical_block_width, int logical_block_height,
  int precision_bits_a, int precision_bits_b,
  int sign_extend,
  int subsampling_factor_a, int subsampling_factor_b,
  int subsampling_offset_a, int subsampling_offset_b,
  int left_margin,
  int left_padding_amount,
  int right_padding_amount,
  int padding_mode,
  int element_init_data,
  int acknowledge_mode,
  int acknowledge_addr,
  int acknowledge_data,
  int left_margin_enable,
  int left_padding_enable,
  int right_padding_enable,
  int subsampling_enable
)
{
  int packer;
  int master_id;
  int srmd_support, burst_support; //wmask_support,
  int max_linear_burst_size;
  int irq_mask;
  int max_block_width, max_block_height, max_stride;
  int precision_type_a, precision_type_b;
  int completed;

  int data_a_bits, data_b_bits;

  int errors = 0;

  int global_set_index;

  int word;

  printf ("executing command...\n");

  if (channel_id < CHANNEL_BANKS) { // only write channel contents via slave interface for channel_ids within the physical range

    // configure channel parameters
    switch (precision_bits_a) {
      case 5  : global_set_index = 1; break;
      case 12 : global_set_index = 0; break;
      default : global_set_index = 2; break;
    }

    packer = (global_set_index                                      << (PRECISION_B_BITS + PRECISION_A_BITS + 1)) +
             (precision_bits_2_precision_index(1, precision_bits_b) <<                    (PRECISION_A_BITS + 1)) +
             (precision_bits_2_precision_index(0, precision_bits_a) <<                                        1 ) +
             sign_extend;
    write_channel_regs(channel_id, logical_block_width, logical_block_height, logical_stride_a, logical_stride_b, packer, element_init_data);

    write_margin_setup(channel_id, left_margin);
    write_padding_setup(channel_id, left_padding_amount, right_padding_amount, padding_mode);

    // configure subsampling register
    write_channel_subsampling_regs(
      channel_id,
      subsampling_factor_2_subsampling_index(0, subsampling_factor_a),
      subsampling_factor_2_subsampling_index(1, subsampling_factor_b),
      subsampling_offset_a,
      subsampling_offset_b
    );
  }


  // configure master 0
  master_id             = 0;
  srmd_support          = 1;
  //wmask_support         = 1;
  burst_support         = 3;

  irq_mask              = 0;
  max_linear_burst_size = 8 * 5 - 1; // max size for element precision is 12 bits (i.e. 5 elements per 64 bit word)
  max_block_width       = 8 * 5 - 1; // max size for element precision is 12 bits (i.e. 5 elements per 64 bit word)
  max_block_height      = 2;
  max_stride            = 200;
  write_master_regs(
    master_id,
    srmd_support,
    //wmask_support,
    burst_support,
    max_stride
  );
  write_global_regs(
    //irq_mask,
    max_linear_burst_size,
    max_block_width,
    max_block_height,
    0
  );

  max_linear_burst_size = 1 * 12 - 1; // max size for element precision is 5 bits (i.e. 12 elements per 64 bit word)
  max_block_width       = 1 * 12 - 1; // max size for element precision is 5 bits (i.e. 12 elements per 64 bit word)
  write_global_regs(
   // irq_mask,
    max_linear_burst_size,
    max_block_width,
    max_block_height,
    1
  );

  max_linear_burst_size = 10 * 4 - 1; // max size for precise element precision (i.e. 4 elements per 64 bit word)
  max_block_width       = 10 * 4 - 1; // max size for precise element precision (i.e. 4 elements per 64 bit word)
  write_global_regs(
//    //irq_mask,
    max_linear_burst_size,
    max_block_width,
    max_block_height,
    2
  );

  if (acknowledge_mode > 0) { // initialize memory location at acknowledge address for active acknowledge
    word = 0xDEADBEEF;
    _hrt_slave_port_store_32(HRTCAT(MEMORY1,_ip0), acknowledge_addr - MEMORY1_BASE_ADDR, word);
  }

  // configure command for
  write_request_regs(
    requester_id,
    addr_a,
    addr_b,
    channel_id,
    dimension_id,
    0,
    0,
    left_margin_enable,
    left_padding_enable,
    right_padding_enable,
    subsampling_enable,
    command
  );

  executed++;

  if (acknowledge_mode == 0) { // passive acknowledge
    // non-blocking read of completed counter register
    completed = 0;
    while (completed == 0) {
      hrt_sleep();
      completed = read_completed_counter(channel_id, 0);
      printf("polling for completed requests...\n");
    }
    printf("Found that %d requests have completed.\n", completed);
  }
  else if (executed == acknowledge_mode) { // ensure active acknowledge mode == number of executed commands
    executed = 0;
    while (word != acknowledge_data) {
      hrt_sleep();
      hrt_sleep();
      word = _hrt_slave_port_load_32(HRTCAT(MEMORY1,_ip0), acknowledge_addr - MEMORY1_BASE_ADDR);
      printf("polling for active acknowledge data at memory1 address 0x%x: read 0x%x\n", acknowledge_addr, word);
    }
    printf("Found that %d requests have completed.\n", acknowledge_mode);
  }

  //precision_type_a = (int) log2((double)precision_bits_a / (double)8);
  //precision_type_b = (int) log2((double)precision_bits_b / (double)8);
	precision_type_a = (int) (log((double)precision_bits_a / (double)8)/log(2));
  precision_type_b = (int) (log((double)precision_bits_b / (double)8)/log(2));

  data_a_bits = DATA_A_BITS;
  data_b_bits = DATA_B_BITS;

  errors += compare_mem1_mem2_block(
    logical_block_width+1, logical_block_height+1,
    data_a_bits, data_b_bits,
    addr_a, logical_stride_a, precision_bits_a, subsampling_factor_a, subsampling_offset_a,
    addr_b, logical_stride_b, precision_bits_b, subsampling_factor_b, subsampling_offset_b,
    sign_extend,
    command_direction(command),
    left_margin,
    left_padding_amount,
    right_padding_amount,
    padding_mode,
    element_init_data,
    left_margin_enable,
    left_padding_enable,
    right_padding_enable,
    subsampling_enable
  );

  printf ("executing command... done.\n");

  return errors;
}*/

