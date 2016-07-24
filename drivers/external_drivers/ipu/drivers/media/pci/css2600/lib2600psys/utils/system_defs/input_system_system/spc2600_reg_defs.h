#ifndef _SPC2600_REG_DEFS_
#define _SPC2600_REG_DEFS_

////////////SPC STATUS & CONTROL REGISTER OFFSETS//////////
#define SEGMENT_SP_NUM	4
#define SEGMENT_BITS	2

#define SPC_XMEM_BASE0			0x58
#define SPC_XMEM_BASE1			0x64
#define SPC_XMEM_BASE2			0x70
#define SPC_XMEM_BASE3			0x7C

#define SPC_XMEM_INFO0			0x5C
#define SPC_XMEM_INFO1			0x68
#define SPC_XMEM_INFO2			0x74
#define SPC_XMEM_INFO3			0x80

static const unsigned int spc_xmem_base[SEGMENT_SP_NUM] =
{
	SPC_XMEM_BASE0,
	SPC_XMEM_BASE1,
	SPC_XMEM_BASE2,
	SPC_XMEM_BASE3
};
#define SPC_ICACHE_INFO0	0x14


#endif /* _SPC2600_REG_DEFS_ */
