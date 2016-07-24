#include "regmem_access.h" // implemented interface

#ifdef C_RUN
#error C_RUN not supported
#endif

#include "regmem_const.h"
#include <vied/vied_subsystem_access.h>
#include PROGMAP

#define SPC_DMEM_ADDRESS 0x8000
#define REGMEM_ADDRESS (SPC_DMEM_ADDRESS + HIVE_ADDR_regmem)

unsigned int regmem_load_32(unsigned int reg)
{
	return vied_subsystem_load_32(0, REGMEM_ADDRESS + (4*reg));
}

void regmem_store_32(unsigned int reg, unsigned int value)
{
	vied_subsystem_store_32(0, REGMEM_ADDRESS + (4*reg), value);
}

