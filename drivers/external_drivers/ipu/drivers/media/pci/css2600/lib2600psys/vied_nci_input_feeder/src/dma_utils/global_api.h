#ifndef _global_api_h_
#define _global_api_h_

#include "defines.h"
#include "base_api.h"

typedef struct {
  unsigned int unit_descriptor_base_addr;
  unsigned int span_descriptor_base_addr;
  unsigned int terminal_descriptor_base_addr;
  unsigned int channel_descriptor_base_addr;
  unsigned int max_block_height;
  unsigned int max_1d_block_width[GLOBAL_SETS];
  unsigned int max_2d_block_width[GLOBAL_SETS];
} global_descriptor_t;

extern bool compare_global_descriptors(
/* compares two global descriptors and returns true when the descriptors are equal, false if they are not equal */
  global_descriptor_t d1,
  global_descriptor_t d2
);

extern void print_global_descriptor(
/* prints the contents of a global descriptor */
  global_descriptor_t d
);

extern void upload_global_descriptor(
/* uploads a global descriptor to DMA global registers */
  global_descriptor_t global_descriptor
);

/* uploads a global descriptor from DMA global registers */
extern global_descriptor_t download_global_descriptor( void );

#endif
