#ifndef _syscom_load_program_h
#define _syscom_load_program_h

#if defined(C_RUN) || defined(HRT_UNSCHED) || defined(HRT_CSIM)
#include "syscom_load_program_csim.h"
#else
#include "syscom_load_program_bin.h"
#endif

#endif /* _syscom_load_program_h */
