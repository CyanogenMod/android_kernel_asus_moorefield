#ifndef _unit_api_h_
#define _unit_api_h_

#include "defines.h"
#include "base_api.h"

#define UNIT_WIDTH_BITS  _log2(MAX_UNIT_WIDTH)
#define UNIT_HEIGHT_BITS _log2(MAX_UNIT_HEIGHT)

typedef struct {
  unsigned int unit_width;
  unsigned int unit_height;
} unit_descriptor_t;

/* calculates the number of control master data words required to encode a unit descriptor */
extern int unit_descriptor_words( void );

extern void write_unit_descriptor(
/* writes a complete <unit descriptor> with <unit_id> identifier to memory in a list of descriptors starting at <unit_descriptor_base_addr> */
  int unit_descriptor_base_addr,
  int unit_id,
  unit_descriptor_t unit_descriptor
);

extern unit_descriptor_t read_unit_descriptor(
/* reads a complete <unit descriptor> with <unit_id> identifier from memory from a list of descriptors starting at <unit_descriptor_base_addr> */
  int unit_descriptor_base_addr,
  int unit_id
);

extern bool compare_unit_descriptors(
/* compares two unit descriptors and returns true when the descriptors are equal, false if they are not equal */
  unit_descriptor_t d1,
  unit_descriptor_t d2
);

extern void print_unit_descriptor(
/* prints the contents of a unit descriptor */
  unit_descriptor_t d
);

extern void upload_unit_descriptor(
/* uploads a unit descriptor to DMA unit descriptor register bank with id <unit_id> */
  unit_descriptor_t unit_descriptor,
  int unit_id
);

extern unit_descriptor_t download_unit_descriptor(
/* downloads a unit descriptor from DMA unit descriptor register bank with id <unit_id> */
  int unit_id
);

extern void set_unit_bank_mode(
/* sets the bank mode of bank <unit_bank_id> to <bank_mode> */
  int unit_bank_id,
  bank_mode_t bank_mode
);
#endif
