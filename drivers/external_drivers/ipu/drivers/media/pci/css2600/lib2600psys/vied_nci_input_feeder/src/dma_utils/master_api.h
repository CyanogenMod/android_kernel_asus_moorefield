#ifndef _master_api_h_
#define _master_api_h_

#include "defines.h"
#include "base_api.h"

typedef struct {
  unsigned int srmd_support;
  unsigned int burst_support;
  unsigned int max_stride;
} master_descriptor_t;

extern bool compare_master_descriptors(
/* compares two master descriptors and returns true when the descriptors are equal, false if they are not equal */
  master_descriptor_t d1,
  master_descriptor_t d2
);

extern void print_master_descriptor(
/* prints the contents of a master descriptor */
  master_descriptor_t d
);

extern void upload_master_descriptor(
/* uploads a master descriptor to DMA master descriptor register bank with id <master_id> */
  master_descriptor_t master_descriptor,
  int master_id
);

extern master_descriptor_t download_master_descriptor(
/* downloads a master descriptor from DMA master descriptor register bank with id <master_id> */
  int master_id
);


#endif
