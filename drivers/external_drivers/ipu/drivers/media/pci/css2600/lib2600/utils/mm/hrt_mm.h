#ifndef _hive_mm_hrt_h_
#define _hive_mm_hrt_h_

#include <hmm_64/hmm.h>

/* Allocate memory in main DDR, returns a word-aligned virtual address */
hmm_ptr hrt_mm_alloc(size_t bytes);

/* allocate memory in main DDR and initialize with zeros,
   returns a word-aligned virtual address */
hmm_ptr hrt_mm_calloc(size_t bytes);

hmm_ptr hrt_mm_alloc_contiguous(size_t bytes);
hmm_ptr hrt_mm_calloc_contiguous(size_t bytes);

/* Free memory, given a virtual address */
void hrt_mm_free(hmm_ptr virt_addr);

/* Store data to a virtual address */
void hrt_mm_load(hmm_ptr virt_addr, void *data, size_t bytes);

/* Load data from a virtual address */
void hrt_mm_store(hmm_ptr virt_addr, const void *data, size_t bytes);

/* initialize data at a virtual address */
void hrt_mm_set(hmm_ptr virt_addr, const int data, size_t bytes);

int   hrt_mm_load_int  (hmm_ptr virt_addr);
short hrt_mm_load_short(hmm_ptr virt_addr);
char  hrt_mm_load_char (hmm_ptr virt_addr);

void hrt_mm_store_char (hmm_ptr virt_addr, char data);
void hrt_mm_store_short(hmm_ptr virt_addr, short data);
void hrt_mm_store_int  (hmm_ptr virt_addr, int data);

/* translate a virtual to a physical address, used to program
   the display driver on  the FPGA system */
hmm_ptr hrt_mm_virt_to_phys(hmm_ptr virt_addr);

#endif /* _hive_mm_hrt_h_ */
