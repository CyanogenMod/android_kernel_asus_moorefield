#include "regmem_access.h" // implemented interface

#include "regmem_const.h"

volatile unsigned int regmem[REGMEM_SIZE];

unsigned int regmem_load_32(unsigned int reg)
{
	// assert(reg < REGMEM_SIZE);
	return regmem[reg];
}

void regmem_store_32(unsigned int reg, unsigned int value)
{
	// assert(reg < REGMEM_SIZE);
	regmem[reg] = value;
}

