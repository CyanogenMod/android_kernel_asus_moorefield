#include "syscom_sp.h" /* implemented interface */

#include <vied/vied_subsystem_access.h>
#include "assert_support.h"

typedef enum {
	cell_stat_ctrl_start_bit		= 1,
	cell_stat_ctrl_run_bit			= 3,
	cell_stat_ctrl_ready_bit		= 5,
	cell_stat_ctrl_icache_invalidate_bit	= 12
} cell_stat_ctrl_bit_t;

static const unsigned int sp_stat_ctrl_reg_address	= 0x0;
static const unsigned int sp_start_pc_reg_address	= 0x4;
static const unsigned int sp_icache_base_reg_address	= 0x10;
static const unsigned int sp_dmem_address		= 0x8000;

void sp_start(unsigned int pc)
{
	unsigned int reg;

	/* set start PC */
	vied_subsystem_store_32(0, sp_start_pc_reg_address, pc);

	/* set run bit and start bit */
	reg = vied_subsystem_load_32(0, sp_stat_ctrl_reg_address);
	reg |= (1 << cell_stat_ctrl_start_bit);
	reg |= (1 << cell_stat_ctrl_run_bit);
	vied_subsystem_store_32(0, sp_stat_ctrl_reg_address, reg);
}

unsigned int sp_is_ready(void)
{
	unsigned int reg;

	reg = vied_subsystem_load_32(0, sp_stat_ctrl_reg_address);
	return (reg >> cell_stat_ctrl_ready_bit) & 1;
}


void
sp_set_icache_base_address(unsigned int value)
{
	vied_subsystem_store_32(0, sp_icache_base_reg_address, value);
}

void
sp_invalidate_icache(void)
{
	unsigned int reg;

	reg = vied_subsystem_load_32(0, sp_stat_ctrl_reg_address);
	reg |= (1 << cell_stat_ctrl_icache_invalidate_bit);
	vied_subsystem_store_32(0, sp_stat_ctrl_reg_address, reg);
}

void
sp_dmem_store(unsigned int dst, const void *src, unsigned int size)
{
	vied_subsystem_store(0, sp_dmem_address + dst, src, size);
}

void
sp_dmem_zero(unsigned int dst, unsigned int size)
{
	unsigned int begin, end;

	assert(size % 4 == 0);
	assert(dst % 4 == 0);

	begin = sp_dmem_address + dst;
	end   = begin + size;

	while (begin != end) {
		vied_subsystem_store_32(0, begin, 0);
		begin += 4;
	}
}

