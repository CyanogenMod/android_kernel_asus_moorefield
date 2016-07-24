#ifndef _ibuf_cntrl_2600_h_
#define _ibuf_cntrl_2600_h_

#include <hrt/api.h>
#include "ibuf_cntrl_2600_defs.h"

#define hrt_ibc_2600_slave_port(ibuf_cntrl_2600_id) HRTCAT(ibuf_cntrl_2600_id, _sl)
#define hrt_ibc_2600_register_address(reg) (_IBC_2600_REG_ALIGN * (reg))

#define hrt_ibc_2600_get_register(ibc_2600_id, reg) \
  _hrt_slave_port_load_32_volatile(hrt_ibc_2600_slave_port(ibc_2600_id), hrt_ibc_2600_register_address(reg))

#define hrt_ibc_2600_set_register(ibc_2600_id, reg, val) \
  _hrt_slave_port_store_32_volatile(hrt_ibc_2600_slave_port(ibc_2600_id), hrt_ibc_2600_register_address(reg), (val) )


#define hrt_ibc_2600_reg(bank_id, group_id, reg_id) \
  ((bank_id<<(_IBC_2600_REG_IDX_BITS + _IBC_2600_GROUP_IDX_BITS)) |\
  (group_id<<_IBC_2600_REG_IDX_BITS) |\
  (reg_id))

// hrt to set a register:
#define hrt_ibc_2600_set_shared_register(ibc_2600_id, reg, val) \
  hrt_ibc_2600_set_register(ibc_2600_id, hrt_ibc_2600_reg(0, _IBC_2600_GROUP_SHARED, reg), val)

#define hrt_ibc_2600_set_sid_proc_cfg_register(ibc_2600_id, proc_nr, reg, val) \
  hrt_ibc_2600_set_register(ibc_2600_id, hrt_ibc_2600_reg(proc_nr, _IBC_2600_GROUP_PROC_CFG, reg), val)

#define hrt_ibc_2600_set_sid_proc_cmd_register(ibc_2600_id, proc_nr, reg, val) \
  hrt_ibc_2600_set_register(ibc_2600_id, hrt_ibc_2600_reg(proc_nr, _IBC_2600_GROUP_PROC_CMD, reg), val)

#define hrt_ibc_2600_set_sid_proc_stat_register(ibc_2600_id, proc_nr, reg, val) \
  hrt_ibc_2600_set_register(ibc_2600_id, hrt_ibc_2600_reg(proc_nr, _IBC_2600_GROUP_PROC_STAT, reg), val)

#define hrt_ibc_2600_set_dest_cfg_register(ibc_2600_id, dest_cfg_nr, reg, val) \
  hrt_ibc_2600_set_register(ibc_2600_id, hrt_ibc_2600_reg(proc_nr, _IBC_2600_GROUP_DEST_CFG, reg), val)

#define hrt_ibc_2600_set_irq_check_register(ibc_2600_id, irq_check_nr, reg, val) \
  hrt_ibc_2600_set_register(ibc_2600_id, hrt_ibc_2600_reg(irq_check_nr, _IBC_2600_GROUP_IRQ_CHECK, reg), val)

#define hrt_ibc_2600_set_feeder_register(ibc_2600_id, feeder_nr, reg, val) \
  hrt_ibc_2600_set_register(ibc_2600_id, hrt_ibc_2600_reg(proc_nr, _IBC_2600_GROUP_FEEDER, reg), val)


// hrt to get a register:
#define hrt_ibc_2600_get_shared_register(ibc_2600_id, reg) \
  hrt_ibc_2600_get_register(ibc_2600_id, hrt_ibc_2600_reg(0, _IBC_2600_GROUP_SHARED, reg))

#define hrt_ibc_2600_get_sid_proc_cfg_register(ibc_2600_id, proc_nr, reg) \
  hrt_ibc_2600_get_register(ibc_2600_id, hrt_ibc_2600_reg(proc_nr, _IBC_2600_GROUP_PROC_CFG, reg))

#define hrt_ibc_2600_get_sid_proc_cmd_register(ibc_2600_id, proc_nr, reg) \
  hrt_ibc_2600_get_register(ibc_2600_id, hrt_ibc_2600_reg(proc_nr, _IBC_2600_GROUP_PROC_CMD, reg))

#define hrt_ibc_2600_get_sid_proc_stat_register(ibc_2600_id, proc_nr, reg) \
  hrt_ibc_2600_get_register(ibc_2600_id, hrt_ibc_2600_reg(proc_nr, _IBC_2600_GROUP_PROC_STAT, reg))

#define hrt_ibc_2600_get_dest_cfg_register(ibc_2600_id, dest_cfg_nr, reg) \
  hrt_ibc_2600_get_register(ibc_2600_id, hrt_ibc_2600_reg(proc_nr, _IBC_2600_GROUP_DEST_CFG, reg))

#define hrt_ibc_2600_get_irq_check_register(ibc_2600_id, irq_check_nr, reg) \
  hrt_ibc_2600_get_register(ibc_2600_id, hrt_ibc_2600_reg(irq_check_nr, _IBC_2600_GROUP_IRQ_CHECK, reg))

#define hrt_ibc_2600_get_feeder_register(ibc_2600_id, feeder_nr, reg) \
  hrt_ibc_2600_get_register(ibc_2600_id, hrt_ibc_2600_reg(proc_nr, _IBC_2600_GROUP_FEEDER, reg))


/* Combination of commands: */
#ifndef C_RUN
  #define hrt_str_to_vec_set_crun(ibc_2600_id, var)
#else
  #define hrt_str_to_vec_set_crun(ibc_2600_id, var) \
    hrt_ibc_2600_set_shared_register(ibc_2600_id, _IBC_2600_REG_SET_CRUN, var)
#endif

#endif
