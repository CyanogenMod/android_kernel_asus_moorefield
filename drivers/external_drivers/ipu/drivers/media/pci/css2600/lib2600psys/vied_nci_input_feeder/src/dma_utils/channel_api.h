#ifndef _channel_api_h_
#define _channel_api_h_

#include "defines.h"
#include "base_api.h"


typedef struct {
  unsigned int element_extend_mode;
  unsigned int element_init_data;
  unsigned int padding_mode;
  unsigned int sampling_setup;
  unsigned int global_set_id;
  unsigned int ack_mode;
  unsigned int ack_addr;
  unsigned int ack_data;
  unsigned int completed_count;
} channel_descriptor_t;

typedef enum {
  extend_mode_zero,
  extend_mode_sign
} extend_mode_t;

typedef enum {
  padding_mode_constant,
  padding_mode_clone,
  padding_mode_mirror,
  padding_mode_append,
  padding_mode_truncate
} padding_mode_t;

extern char* get_padding_mode_name(
/* returns a string reflecting the name of the <padding_mode> */
  padding_mode_t padding_mode
);

extern unsigned int get_subsampling_factor(
/* returns the subsampling factor corresponding to the specified sampling setup (subsampling index) value */
  unsigned int sampling_setup
);

/* calculates the number of control master data words required to encode a channel descriptor */
extern int channel_descriptor_words( void );

extern void write_channel_descriptor(
/* writes a complete <channel descriptor> with <channel_id> identifier to memory in a list of descriptors starting at <channel_descriptor_base_addr> */
  int channel_descriptor_base_addr,
  int channel_id,
  channel_descriptor_t channel_descriptor
);

extern channel_descriptor_t read_channel_descriptor(
/* reads a complete <channel descriptor> with <channel_id> identifier from memory from a list of descriptors starting at <channel_descriptor_base_addr> */
  int channel_descriptor_base_addr,
  int channel_id
);

extern bool compare_channel_descriptors(
/* compares two channel descriptors and returns true when the descriptors are equal, false if they are not equal */
  channel_descriptor_t d1,
  channel_descriptor_t d2
);

extern void print_channel_descriptor(
/* prints the contents of a channel descriptor */
  channel_descriptor_t d
);

extern void upload_channel_descriptor(
/* uploads a channel descriptor to DMA channel descriptor register bank with id <channel_id> */
  channel_descriptor_t channel_descriptor,
  int channel_id
);

extern channel_descriptor_t download_channel_descriptor(
/* downloads a channel descriptor from DMA channel descriptor register bank with id <channel_id> */
  int channel_id
);

extern unsigned int read_completed_counter (
/* returns the value of the completed counter in channel bank <channel_bank_id> */
  int channel_bank_id
);

extern void set_channel_bank_mode(
/* sets the bank mode of bank <channel_bank_id> to <bank_mode> */
  int channel_bank_id,
  bank_mode_t bank_mode
);

extern int wait_for_acknowledge (
  channel_descriptor_t channel_descriptor,
  int channel_id
);



#endif
