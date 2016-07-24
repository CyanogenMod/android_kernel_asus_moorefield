#ifndef __SYSCOM_SP_H
#define __SYSCOM_SP_H

void sp_invalidate_icache(void);
void sp_set_icache_base_address(unsigned int value);
void sp_dmem_store(unsigned int dst, const void *src, unsigned int size);
void sp_dmem_zero(unsigned int dst, unsigned int size);

void sp_start(unsigned int pc);
unsigned int sp_is_ready(void);

#endif

