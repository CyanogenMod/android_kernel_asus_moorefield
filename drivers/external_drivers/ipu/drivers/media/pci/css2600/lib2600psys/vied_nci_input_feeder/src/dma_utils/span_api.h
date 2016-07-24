#ifndef _span_api_h_
#define _span_api_h_

#include "defines.h"
#include "base_api.h"
#include "unit_api.h"

typedef struct {
  unsigned int unit_location;
  unsigned int span_row;
  unsigned int span_column;
  unsigned int span_width;
  unsigned int span_height;
  unsigned int span_mode;
} span_descriptor_t;

typedef enum {
  span_order_row_first,
  span_order_column_first
} span_order_t;

typedef enum {
  addressing_mode_byte_address_based,
  addressing_mode_coordinate_based
} addressing_mode_t;

extern unsigned int unit_location(
/* creates a unit location field out of (x,y) coordinates */
  int x,
  int y
);

extern int get_x_coordinate (
/* returns the x-coordinate contained in a unit location field */
  unsigned int unit_location
);
extern int get_y_coordinate (
/* returns the y-coordinate contained in a unit location field */
  unsigned int unit_location
);

extern span_order_t get_span_order(
/* returns the span order contained in the span mode field */
  unsigned int span_mode
);

extern addressing_mode_t get_addressing_mode(
/* returns the addressing mode contained in the span mode field */
  unsigned int span_mode
);

extern unsigned int span_mode(
/* creates a span mode field out of span order and addressing mode inputs */
  span_order_t span_order,
  addressing_mode_t addressing_mode
);

/* calculates the number of control master data words required to encode a span descriptor */
extern int span_descriptor_words( void );

extern void write_span_descriptor(
/* writes a complete <span descriptor> with <span_id> identifier to memory in a list of descriptors starting at <span_descriptor_base_addr> */
  int span_descriptor_base_addr,
  int span_id,
  span_descriptor_t span_descriptor
);

extern span_descriptor_t read_span_descriptor(
/* reads a complete <span descriptor> with <span_id> identifier from memory from a list of descriptors starting at <span_descriptor_base_addr> */
  int span_descriptor_base_addr,
  int span_id
);

extern bool compare_span_descriptors(
/* compares two span descriptors and returns true when the descriptors are equal, false if they are not equal */
  span_descriptor_t d1,
  span_descriptor_t d2
);

extern void print_span_descriptor(
/* prints the contents of a span descriptor */
  span_descriptor_t d
);

extern void upload_span_descriptor(
/* uploads a span descriptor to DMA span descriptor register bank with id <span_id> */
  span_descriptor_t span_descriptor,
  int span_id
);

extern span_descriptor_t download_span_descriptor(
/* downloads a span descriptor from DMA span descriptor register bank with id <span_id> */
  int span_id
);

extern void update_span_descriptor(
/* models the updating of span descriptor contents (unit_location, span_row, span_column) after each operation execution */
  span_descriptor_t *span_descriptor,
  unit_descriptor_t unit_descriptor
);

extern void set_span_bank_mode(
/* sets the bank mode of bank <span_bank_id> to <bank_mode> */
  int span_bank_id,
  bank_mode_t bank_mode
);

#endif
