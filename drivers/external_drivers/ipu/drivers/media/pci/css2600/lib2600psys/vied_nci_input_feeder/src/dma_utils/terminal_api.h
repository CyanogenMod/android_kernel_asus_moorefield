#ifndef _terminal_api_h_
#define _terminal_api_h_

#include "defines.h"
#include "base_api.h"

typedef struct {
  unsigned int region_origin;
  unsigned int region_width;
  unsigned int region_stride;
  unsigned int element_setup;
  unsigned int cio_info_setup;
  unsigned int port_mode;
} terminal_descriptor_t;


extern unsigned int get_precision(
/* returns the element precision corresponding to the specified element setup (element precision index) value */
  unsigned int element_setup
);

extern bool is_strict_precision(
/* returns true if the <precision> provided as argument is strict; returns false otherwise. */
  unsigned int precision
);

/* calculates the number of control master data words required to encode a terminal descriptor */
extern int terminal_descriptor_words( void );

extern void write_terminal_descriptor(
/* writes a complete <terminal descriptor> with <terminal_id> identifier to memory in a list of descriptors starting at <terminal_descriptor_base_addr> */
  int terminal_descriptor_base_addr,
  int terminal_id,
  terminal_descriptor_t terminal_descriptor
);

extern terminal_descriptor_t read_terminal_descriptor(
/* reads a complete <terminal descriptor> with <terminal_id> identifier from memory from a list of descriptors starting at <terminal_descriptor_base_addr> */
  int terminal_descriptor_base_addr,
  int terminal_id
);

extern bool compare_terminal_descriptors(
/* compares two terminal descriptors and returns true when the descriptors are equal, false if they are not equal */
  terminal_descriptor_t d1,
  terminal_descriptor_t d2
);

extern void print_terminal_descriptor(
/* prints the contents of a terminal descriptor */
  terminal_descriptor_t d
);

extern void upload_terminal_descriptor(
/* uploads a terminal descriptor to DMA terminal descriptor register bank with id <terminal_id> */
  terminal_descriptor_t terminal_descriptor,
  int terminal_id
);

extern terminal_descriptor_t download_terminal_descriptor(
/* downloads a terminal descriptor from DMA terminal descriptor register bank with id <terminal_id> */
  int terminal_id
);

extern void set_terminal_bank_mode(
/* sets the bank mode of bank <terminal_bank_id> to <bank_mode> */
  int terminal_bank_id,
  bank_mode_t bank_mode
);


#endif
