#ifndef __MODADD_H__
#define __MODADD_H__

#if !defined(HAS_std_modadd)
static inline
unsigned int OP_std_modadd(/* unsigned */ int a, /* signed */ int b, /* unsigned */ int c)
{
	return a+b<0 ? a+b+c : a+b>=c ? a+b-c : a+b;
}
#endif

#endif /*__MODADD_H__*/
