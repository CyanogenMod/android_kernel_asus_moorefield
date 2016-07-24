#ifndef _hrt_gp_regs_h_
#define _hrt_gp_regs_h_

#include <hrt/api.h>
#include "gp_regs_defs.h"

//#define hrt_gp_regs_slave_port(gp_regs_id) HRTCAT(gp_regs_id,_slv_in)
#define hrt_gp_regs_register_address(reg) (_HRT_GP_REGS_REG_ALIGN * (reg))

#define hrt_gp_regs_set_register(gp_regs_id, reg, val) \
  vied_subsystem_store_32(psys0, gp_regs_id + hrt_gp_regs_register_address(reg), (val))

#define hrt_gp_regs_get_register(gp_regs_id, reg) \
  vied_subsystem_load_32(psys0, gp_regs_id + hrt_gp_regs_register_address(reg))

#endif /* _hrt_gp_regs_h */
