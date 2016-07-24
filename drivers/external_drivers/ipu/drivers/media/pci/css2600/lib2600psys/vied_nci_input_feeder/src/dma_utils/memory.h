#ifndef _memory_h_
#define _memory_h_


typedef struct {
  int memory_id;  /* memory identifier */
  int start_addr; /* byte address offset from memory slave port address */
  int width;      /* width in bytes */
  int height;     /* height in lines */
  int stride;     /* stride in bytes */
} block_t;

extern void initialize_memory_block(
/* initializes a block of memory width default data (a sequence of bytes incrementing in value */
  block_t block
);

extern void initialize_memory_block_32bits(
    /* initializes a block of memory width default data (a sequence of bytes incrementing in value */
    block_t block
                                   );

#endif
