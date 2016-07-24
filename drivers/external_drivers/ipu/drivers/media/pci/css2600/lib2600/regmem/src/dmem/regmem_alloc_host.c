#include "regmem_alloc.h"
#include "regmem_const.h"
#include "assert_support.h"

static unsigned int r = 0;

unsigned int regmem_alloc(void)
{
	assert(r < REGMEM_SIZE);
	unsigned int reg = r++;
	return reg;
}

/* free all registers */
void regmem_free(void)
{
	r = 0;
}

