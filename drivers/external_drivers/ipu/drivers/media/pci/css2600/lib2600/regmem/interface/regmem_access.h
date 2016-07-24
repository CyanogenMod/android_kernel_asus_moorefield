#ifndef __REGMEM_ACCESS_H__
#define __REGMEM_ACCESS_H__

#include "regmem.h"

unsigned int regmem_load_32(unsigned int reg);
void         regmem_store_32(unsigned int reg, unsigned int value);

#endif /*__REGMEM_ACCESS_H__*/
