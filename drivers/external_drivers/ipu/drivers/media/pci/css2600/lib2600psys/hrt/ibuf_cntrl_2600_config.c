#include "cpu_mem_support.h"
#include "ibuf_cntrl_2600_config.h"

#include <hrt/api.h>

unsigned int ibc2600_check_value(char *reg_name, unsigned int got, unsigned int expected)
{
  if(got != expected)
  {
    #ifndef __HIVECC
    #endif
    return 1;
  }else{
    return 0;
  }
}

unsigned int ibc2600_reg_addr(ibc2600_ibuf_s *ibuf_dev, unsigned int bank_id, unsigned int group_id, unsigned int reg_nr)
{
  unsigned int reg_addr = 0;
  reg_addr |= (bank_id<<(_IBC_2600_REG_IDX_BITS + _IBC_2600_GROUP_IDX_BITS));
  reg_addr |= (group_id<<_IBC_2600_REG_IDX_BITS);
  reg_addr |= (reg_nr);
  return ibuf_dev->base_address + (reg_addr * _IBC_2600_REG_ALIGN);
}

unsigned int ibc2600_reg_addr_with_base(unsigned int base_address, unsigned int bank_id, unsigned int group_id, unsigned int reg_nr)
{
  unsigned int reg_addr = 0;
  reg_addr |= (bank_id<<(_IBC_2600_REG_IDX_BITS + _IBC_2600_GROUP_IDX_BITS));
  reg_addr |= (group_id<<_IBC_2600_REG_IDX_BITS);
  reg_addr |= (reg_nr);
  return base_address + (reg_addr * _IBC_2600_REG_ALIGN);
}

void ibc2600_set_shared_reg(ibc2600_ibuf_s *ibuf_dev, unsigned int reg_nr, unsigned int val)
{
  unsigned int addr = ibc2600_reg_addr(ibuf_dev, 0, _IBC_2600_GROUP_SHARED, reg_nr);

  _hrt_master_port_store_32_volatile(addr, val);
}

unsigned int ibc2600_get_shared_reg(ibc2600_ibuf_s *ibuf_dev, unsigned int reg_nr)
{
  unsigned int addr = ibc2600_reg_addr(ibuf_dev, 0, _IBC_2600_GROUP_SHARED, reg_nr);

  return _hrt_master_port_load_32_volatile(addr);
}

void ibc2600_set_sid_cfg_reg(ibc2600_ibuf_s *ibuf_dev, unsigned int proc_nr, unsigned int reg_nr, unsigned int val)
{
  unsigned int addr = ibc2600_reg_addr(ibuf_dev, proc_nr, _IBC_2600_GROUP_PROC_CFG, reg_nr);

  _hrt_master_port_store_32_volatile(addr, val);
}

unsigned int ibc2600_get_sid_cfg_reg(ibc2600_ibuf_s *ibuf_dev, unsigned int proc_nr, unsigned int reg_nr)
{
  unsigned int addr = ibc2600_reg_addr(ibuf_dev, proc_nr, _IBC_2600_GROUP_PROC_CFG, reg_nr);

  return _hrt_master_port_load_32_volatile(addr);
}

void ibc2600_set_sid_cmd_reg(ibc2600_ibuf_s *ibuf_dev, unsigned int proc_nr, unsigned int reg_nr, unsigned int val)
{
  unsigned int addr = ibc2600_reg_addr(ibuf_dev, proc_nr, _IBC_2600_GROUP_PROC_CMD, reg_nr);

  _hrt_master_port_store_32_volatile(addr, val);
}

unsigned int ibc2600_get_sid_cmd_reg(ibc2600_ibuf_s *ibuf_dev, unsigned int proc_nr, unsigned int reg_nr)
{
  unsigned int addr = ibc2600_reg_addr(ibuf_dev, proc_nr, _IBC_2600_GROUP_PROC_CMD, reg_nr);

  return _hrt_master_port_load_32_volatile(addr);
}

unsigned int ibc2600_get_sid_stat_reg(ibc2600_ibuf_s *ibuf_dev, unsigned int proc_nr, unsigned int reg_nr)
{
  unsigned int addr = ibc2600_reg_addr(ibuf_dev, proc_nr, _IBC_2600_GROUP_PROC_STAT, reg_nr);

  return _hrt_master_port_load_32_volatile(addr);
}

void ibc2600_set_dest_cfg_reg(ibc2600_ibuf_s *ibuf_dev, unsigned int dest_nr, unsigned int reg_nr, unsigned int val)
{
  unsigned int addr = ibc2600_reg_addr(ibuf_dev, dest_nr, _IBC_2600_GROUP_DEST_CFG, reg_nr);

  _hrt_master_port_store_32_volatile(addr, val);
}

unsigned int ibc2600_get_dest_cfg_reg(ibc2600_ibuf_s *ibuf_dev, unsigned int dest_nr, unsigned int reg_nr)
{
  unsigned int addr = ibc2600_reg_addr(ibuf_dev, dest_nr, _IBC_2600_GROUP_DEST_CFG, reg_nr);

  return _hrt_master_port_load_32_volatile(addr);
}

void ibc2600_set_frame_check_reg(ibc2600_ibuf_s *ibuf_dev, unsigned int frame_check_nr, unsigned int reg_nr, unsigned int val)
{
  unsigned int addr = ibc2600_reg_addr(ibuf_dev, frame_check_nr, _IBC_2600_GROUP_IRQ_CHECK, reg_nr);

  _hrt_master_port_store_32_volatile(addr, val);
}

unsigned int ibc2600_get_frame_check_reg(ibc2600_ibuf_s *ibuf_dev, unsigned int frame_check_nr, unsigned int reg_nr)
{
  unsigned int addr = ibc2600_reg_addr(ibuf_dev, frame_check_nr, _IBC_2600_GROUP_IRQ_CHECK, reg_nr);

  return _hrt_master_port_load_32_volatile(addr);
}

void ibc2600_set_feeder_reg(ibc2600_ibuf_s *ibuf_dev, unsigned int feeder_nr, unsigned int reg_nr, unsigned int val)
{
  unsigned int addr = ibc2600_reg_addr(ibuf_dev, feeder_nr, _IBC_2600_GROUP_FEEDER, reg_nr);

  _hrt_master_port_store_32_volatile(addr, val);
}

unsigned int ibc2600_get_feeder_reg(ibc2600_ibuf_s *ibuf_dev, unsigned int feeder_nr, unsigned int reg_nr)
{
  unsigned int addr = ibc2600_reg_addr(ibuf_dev, feeder_nr, _IBC_2600_GROUP_FEEDER, reg_nr);

  return _hrt_master_port_load_32_volatile(addr);
}

void ibc2600_config_feeder(ibc2600_ibuf_s *ibuf, unsigned int feeder_id)
{
  ibc2600_feeder_cfg_s feeder_cfg = (*ibuf).feeder_cfg[feeder_id];

  ibc2600_set_feeder_reg(ibuf, feeder_id, _IBC_2600_FEED_CFG_ACK_ADDR, feeder_cfg.ack_addr);
  ibc2600_set_feeder_reg(ibuf, feeder_id, _IBC_2600_FEED_CFG_REQUESTER_ADDR, feeder_cfg.req_addr);
  ibc2600_set_feeder_reg(ibuf, feeder_id, _IBC_2600_FEED_CFG_CHANNEL_ADDR, feeder_cfg.channel_addr);
  ibc2600_set_feeder_reg(ibuf, feeder_id, _IBC_2600_FEED_CFG_UNITS_P_LINE_IN, feeder_cfg.units_p_line_in);
  ibc2600_set_feeder_reg(ibuf, feeder_id, _IBC_2600_FEED_CFG_UNITS_OUT_P_IN, feeder_cfg.units_out_p_in);
  ibc2600_set_feeder_reg(ibuf, feeder_id, _IBC_2600_FEED_CFG_LAST_UNITS_OUT, feeder_cfg.last_units_out);
  ibc2600_set_feeder_reg(ibuf, feeder_id, _IBC_2600_FEED_CFG_HEIGHT, feeder_cfg.height);
  ibc2600_set_feeder_reg(ibuf, feeder_id, _IBC_2600_FEED_CFG_WAIT_ON_OTHER_FEEDERS, feeder_cfg.wait_for_other_feeder);

  (*ibuf).feeder_cfg[feeder_id].feeder_id  = feeder_id;
}

unsigned int ibc2600_check_config_feeder(ibc2600_ibuf_s *ibuf, unsigned int feeder_id)
{
  unsigned int val, errors=0;

  ibc2600_feeder_cfg_s feeder_cfg = (*ibuf).feeder_cfg[feeder_id];

  val=ibc2600_get_feeder_reg(ibuf, feeder_id, _IBC_2600_FEED_CFG_ACK_ADDR);
  errors+=ibc2600_check_value("_IBC_2600_FEED_CFG_ACK_ADDR", val, feeder_cfg.ack_addr);
  val=ibc2600_get_feeder_reg(ibuf, feeder_id, _IBC_2600_FEED_CFG_REQUESTER_ADDR);
  errors+=ibc2600_check_value("_IBC_2600_FEED_CFG_REQUESTER_ADDR", val, feeder_cfg.req_addr);
  val=ibc2600_get_feeder_reg(ibuf, feeder_id, _IBC_2600_FEED_CFG_CHANNEL_ADDR);
  errors+=ibc2600_check_value("_IBC_2600_FEED_CFG_CHANNEL_ADDR", val, feeder_cfg.channel_addr);
  val=ibc2600_get_feeder_reg(ibuf, feeder_id, _IBC_2600_FEED_CFG_UNITS_P_LINE_IN);
  errors+=ibc2600_check_value("_IBC_2600_FEED_CFG_UNITS_P_LINE_IN", val, feeder_cfg.units_p_line_in);
  val=ibc2600_get_feeder_reg(ibuf, feeder_id, _IBC_2600_FEED_CFG_UNITS_OUT_P_IN);
  errors+=ibc2600_check_value("_IBC_2600_FEED_CFG_UNITS_OUT_P_IN", val, feeder_cfg.units_out_p_in);
  val=ibc2600_get_feeder_reg(ibuf, feeder_id, _IBC_2600_FEED_CFG_LAST_UNITS_OUT);
  errors+=ibc2600_check_value("_IBC_2600_FEED_CFG_LAST_UNITS_OUT", val, feeder_cfg.last_units_out);
  val=ibc2600_get_feeder_reg(ibuf, feeder_id, _IBC_2600_FEED_CFG_HEIGHT);
  errors+=ibc2600_check_value("_IBC_2600_FEED_CFG_HEIGHT", val, feeder_cfg.height);
  val=ibc2600_get_feeder_reg(ibuf, feeder_id, _IBC_2600_FEED_CFG_WAIT_ON_OTHER_FEEDERS);
  errors+=ibc2600_check_value("_IBC_2600_FEED_CFG_WAIT_ON_OTHER_FEEDERS", val, feeder_cfg.wait_for_other_feeder);

  return errors;
}


void ibc2600_config_frame_check(ibc2600_ibuf_s *ibuf, unsigned int frame_check_id)
{
  ibc2600_frame_check_cfg_s frame_check_cfg = (*ibuf).frame_check_cfg[frame_check_id];

  ibc2600_set_frame_check_reg(ibuf, frame_check_id, _IBC_2600_FR_C_CFG_ENABLE, frame_check_cfg.enable);
  ibc2600_set_frame_check_reg(ibuf, frame_check_id, _IBC_2600_FR_C_CFG_CHECK_MODE, frame_check_cfg.mode.byte);
  ibc2600_set_frame_check_reg(ibuf, frame_check_id, _IBC_2600_FR_C_CFG_SID_PROC_ID, frame_check_cfg.sid_id);
  ibc2600_set_frame_check_reg(ibuf, frame_check_id, _IBC_2600_FR_C_CFG_TRIGGER_OFFSET, frame_check_cfg.trigger_offset);
  ibc2600_set_frame_check_reg(ibuf, frame_check_id, _IBC_2600_FR_C_CFG_TRIGGER_REPEAT_VAL, frame_check_cfg.trigger_repeat);
  ibc2600_set_frame_check_reg(ibuf, frame_check_id, _IBC_2600_FR_C_CFG_ADDR, frame_check_cfg.addr);
  ibc2600_set_frame_check_reg(ibuf, frame_check_id, _IBC_2600_FR_C_CFG_TOKEN, frame_check_cfg.token);

  (*ibuf).frame_check_cfg[frame_check_id].frame_check_id = frame_check_id;
}

unsigned int ibc2600_check_config_frame_check(ibc2600_ibuf_s *ibuf, unsigned int frame_check_id)
{
  unsigned int val, errors=0;

  ibc2600_frame_check_cfg_s frame_check_cfg = (*ibuf).frame_check_cfg[frame_check_id];

  val=ibc2600_get_frame_check_reg(ibuf, frame_check_id, _IBC_2600_FR_C_CFG_ENABLE);
  errors+=ibc2600_check_value("_IBC_2600_FR_C_CFG_ENABLE", val, frame_check_cfg.enable);
  val=ibc2600_get_frame_check_reg(ibuf, frame_check_id, _IBC_2600_FR_C_CFG_CHECK_MODE);
  errors+=ibc2600_check_value("_IBC_2600_FR_C_CFG_CHECK_MODE", val, frame_check_cfg.mode.byte);
  val=ibc2600_get_frame_check_reg(ibuf, frame_check_id, _IBC_2600_FR_C_CFG_SID_PROC_ID);
  errors+=ibc2600_check_value("_IBC_2600_FR_C_CFG_SID_PROC_ID", val, frame_check_cfg.sid_id);
  val=ibc2600_get_frame_check_reg(ibuf, frame_check_id, _IBC_2600_FR_C_CFG_TRIGGER_OFFSET);
  errors+=ibc2600_check_value("_IBC_2600_FR_C_CFG_TRIGGER_OFFSET", val, frame_check_cfg.trigger_offset);
  val=ibc2600_get_frame_check_reg(ibuf, frame_check_id, _IBC_2600_FR_C_CFG_TRIGGER_REPEAT_VAL);
  errors+=ibc2600_check_value("_IBC_2600_FR_C_CFG_TRIGGER_REPEAT_VAL", val, frame_check_cfg.trigger_repeat);
  val=ibc2600_get_frame_check_reg(ibuf, frame_check_id, _IBC_2600_FR_C_CFG_ADDR);
  errors+=ibc2600_check_value("_IBC_2600_FR_C_CFG_ADDR", val, frame_check_cfg.addr);
  val=ibc2600_get_frame_check_reg(ibuf, frame_check_id, _IBC_2600_FR_C_CFG_TOKEN);
  errors+=ibc2600_check_value("_IBC_2600_FR_C_CFG_TOKEN", val, frame_check_cfg.token);

  return errors;
}

void ibc2600_ibc2600_config_proc_cfg(ibc2600_ibuf_s *ibuf, unsigned int proc_id)
{
  unsigned int dest;

  ibc2600_proc_cfg_s proc_cfg = (*ibuf).proc[proc_id].cfg;

  ibc2600_set_sid_cfg_reg(ibuf, proc_id, _IBC_2600_PROC_CFG_STR2MMIO_PROC_ADDR, proc_cfg.str2mmio_addr);
  ibc2600_set_sid_cfg_reg(ibuf, proc_id, _IBC_2600_PROC_CFG_STR2MMIO_STORE_CMD, proc_cfg.store_cmd);
  ibc2600_set_sid_cfg_reg(ibuf, proc_id, _IBC_2600_PROC_CFG_ITEMS_P_UNIT, proc_cfg.items_p_unit);
  ibc2600_set_sid_cfg_reg(ibuf, proc_id, _IBC_2600_PROC_CFG_UNITS_P_LINE, proc_cfg.units_p_line);
  ibc2600_set_sid_cfg_reg(ibuf, proc_id, _IBC_2600_PROC_CFG_LINES_P_FRAME, proc_cfg.lines_p_frame);
  ibc2600_set_sid_cfg_reg(ibuf, proc_id, _IBC_2600_PROC_CFG_UNITS_P_IBUF, proc_cfg.units_p_ibuf);
  ibc2600_set_sid_cfg_reg(ibuf, proc_id, _IBC_2600_PROC_CFG_SYNC_FRAME, proc_cfg.sync_frame);

  for (dest=0; dest<(*ibuf).proc[proc_id].nr_dests; dest++)
  {
    ibc2600_set_sid_cfg_reg(ibuf, proc_id, _IBC_2600_PROC_CFG_DEST_ENABLED(dest), proc_cfg.dest_en[dest]);
  }
}

void ibc2600_ibc2600_config_proc_cmd(ibc2600_ibuf_s *ibuf, unsigned int proc_id)
{
  ibc2600_proc_cmd_s proc_cmd = (*ibuf).proc[proc_id].cmd;

  ibc2600_set_sid_cmd_reg(ibuf, proc_id, _IBC_2600_PROC_CMD_ACK_ADDR, proc_cmd.ack_addr);
  ibc2600_set_sid_cmd_reg(ibuf, proc_id, _IBC_2600_PROC_CMD_EVENTQUE_SIDPID, proc_cmd.sidpid);
  if((*ibuf).has_2nd_buff[proc_id])
    ibc2600_set_sid_cmd_reg(ibuf, proc_id, _IBC_2600_PROC_CMD_2NDBUF_CMD_ADDR, proc_cmd.snd_buf_cmd_addr);
}

unsigned int ibc2600_check_ibc2600_config_proc_cfg(ibc2600_ibuf_s *ibuf_dev, unsigned int proc_id)
{
  unsigned int dest, nr_dests, val, errors=0;

  ibc2600_proc_cfg_s proc_cfg = ibuf_dev->proc[proc_id].cfg;
  nr_dests = ibuf_dev->proc[proc_id].nr_dests;

  val=ibc2600_get_sid_cfg_reg(ibuf_dev, proc_id, _IBC_2600_PROC_CFG_STR2MMIO_PROC_ADDR);
  errors+=ibc2600_check_value("_IBC_2600_PROC_CFG_STR2MMIO_PROC_ADDR", val, proc_cfg.str2mmio_addr);
  val=ibc2600_get_sid_cfg_reg(ibuf_dev, proc_id, _IBC_2600_PROC_CFG_STR2MMIO_STORE_CMD);
  errors+=ibc2600_check_value("_IBC_2600_PROC_CFG_STR2MMIO_STORE_CMD", val, proc_cfg.store_cmd);
  val=ibc2600_get_sid_cfg_reg(ibuf_dev, proc_id, _IBC_2600_PROC_CFG_ITEMS_P_UNIT);
  errors+=ibc2600_check_value("_IBC_2600_PROC_CFG_ITEMS_P_UNIT", val, proc_cfg.items_p_unit);
  val=ibc2600_get_sid_cfg_reg(ibuf_dev, proc_id, _IBC_2600_PROC_CFG_UNITS_P_LINE);
  errors+=ibc2600_check_value("_IBC_2600_PROC_CFG_UNITS_P_LINE", val, proc_cfg.units_p_line);
  val=ibc2600_get_sid_cfg_reg(ibuf_dev, proc_id, _IBC_2600_PROC_CFG_LINES_P_FRAME);
  errors+=ibc2600_check_value("_IBC_2600_PROC_CFG_LINES_P_FRAME", val, proc_cfg.lines_p_frame);
  val=ibc2600_get_sid_cfg_reg(ibuf_dev, proc_id, _IBC_2600_PROC_CFG_UNITS_P_IBUF);
  errors+=ibc2600_check_value("_IBC_2600_PROC_CFG_UNITS_P_IBUF", val, proc_cfg.units_p_ibuf);
  val=ibc2600_get_sid_cfg_reg(ibuf_dev, proc_id, _IBC_2600_PROC_CFG_SYNC_FRAME);
  errors+=ibc2600_check_value("_IBC_2600_PROC_CFG_SYNC_FRAME", val, proc_cfg.sync_frame);

  for (dest=0; dest<nr_dests; dest++)
  {
    val=ibc2600_get_sid_cfg_reg(ibuf_dev, proc_id, _IBC_2600_PROC_CFG_DEST_ENABLED(dest));
    errors+=ibc2600_check_value("_IBC_2600_PROC_CFG_DEST_ENABLED(dest)", val, proc_cfg.dest_en[dest]);
  }
  return errors;
}

unsigned int ibc2600_check_ibc2600_config_proc_cmd(ibc2600_ibuf_s *ibuf_dev, unsigned int proc_id)
{
  unsigned int val, errors=0;

  ibc2600_proc_cmd_s proc_cmd = ibuf_dev->proc[proc_id].cmd;

  val=ibc2600_get_sid_cmd_reg(ibuf_dev, proc_id, _IBC_2600_PROC_CMD_ACK_ADDR);
  errors+=ibc2600_check_value("_IBC_2600_PROC_CMD_ACK_ADDR", val, proc_cmd.ack_addr);
  val=ibc2600_get_sid_cmd_reg(ibuf_dev, proc_id, _IBC_2600_PROC_CMD_EVENTQUE_SIDPID);
  errors+=ibc2600_check_value("_IBC_2600_PROC_CMD_EVENTQUE_SIDPID", val, proc_cmd.sidpid);
  if(ibuf_dev->has_2nd_buff[proc_id])
  {
    val=ibc2600_get_sid_cmd_reg(ibuf_dev, proc_id, _IBC_2600_PROC_CMD_2NDBUF_CMD_ADDR);
    errors+=ibc2600_check_value("_IBC_2600_PROC_CMD_2NDBUF_CMD_ADDR", val, proc_cmd.snd_buf_cmd_addr);
  }

  return errors;
}

void ibc2600_config_proc(ibc2600_ibuf_s *ibuf_dev, unsigned int proc_id)
{
  unsigned int dest;
  ibc2600_ibc2600_config_proc_cfg(ibuf_dev, proc_id);
  ibc2600_ibc2600_config_proc_cmd(ibuf_dev, proc_id);
  for(dest=0; dest < ibuf_dev->proc[proc_id].nr_dests; dest++)
  {
    if(ibuf_dev->proc[proc_id].cfg.dest_en[dest]==1)
      ibc2600_config_dest(ibuf_dev, ibuf_dev->proc[proc_id].cfg.dest_cfg[dest]->dest_id);
  }
}

unsigned int ibc2600_check_config_proc(ibc2600_ibuf_s *ibuf_dev, unsigned int proc_id)
{
  unsigned int errors=0;
  unsigned int dest;

  errors+=ibc2600_check_ibc2600_config_proc_cfg(ibuf_dev, proc_id);
  errors+=ibc2600_check_ibc2600_config_proc_cmd(ibuf_dev, proc_id);
  for(dest=0; dest < ibuf_dev->proc[proc_id].nr_dests; dest++)
  {
    if(ibuf_dev->proc[proc_id].cfg.dest_en[dest]==1)
      errors+=ibc2600_check_config_dest(ibuf_dev, ibuf_dev->proc[proc_id].cfg.dest_cfg[dest]->dest_id);
  }

  return errors;
}

void ibc2600_config_dest(ibc2600_ibuf_s *ibuf_dev, unsigned int dest_id)
{
  ibc2600_dest_cfg_s dest_cfg = ibuf_dev->dest_cfg[dest_id];

  ibc2600_set_dest_cfg_reg(ibuf_dev, dest_id, _IBC_2600_DEST_CFG_FEED_ADDR, dest_cfg.feed_addr);
  ibc2600_set_dest_cfg_reg(ibuf_dev, dest_id, _IBC_2600_DEST_CFG_REQUESTER_ADDR, dest_cfg.req_addr);
  ibc2600_set_dest_cfg_reg(ibuf_dev, dest_id, _IBC_2600_DEST_CFG_CHANNEL_ADDR, dest_cfg.channel_addr);
  ibc2600_set_dest_cfg_reg(ibuf_dev, dest_id, _IBC_2600_DEST_CFG_SPAN_A_ADDR, dest_cfg.span_a_addr);
  ibc2600_set_dest_cfg_reg(ibuf_dev, dest_id, _IBC_2600_DEST_CFG_SPAN_B_ADDR, dest_cfg.span_b_addr);
  ibc2600_set_dest_cfg_reg(ibuf_dev, dest_id, _IBC_2600_DEST_CFG_TERMINAL_B_ADDR, dest_cfg.terminal_b_addr);
  ibc2600_set_dest_cfg_reg(ibuf_dev, dest_id, _IBC_2600_DEST_CFG_DEST_MODE, dest_cfg.dest_mode.byte);
  ibc2600_set_dest_cfg_reg(ibuf_dev, dest_id, _IBC_2600_DEST_CFG_ST_ADDR, dest_cfg.st_addr);
  ibc2600_set_dest_cfg_reg(ibuf_dev, dest_id, _IBC_2600_DEST_CFG_DEST_NUM_UNITS, dest_cfg.num_items);
  ibc2600_set_dest_cfg_reg(ibuf_dev, dest_id, _IBC_2600_DEST_CFG_IWAKE_THRESHOLD, dest_cfg.iwake_threshold);
}

unsigned int ibc2600_check_config_dest(ibc2600_ibuf_s *ibuf_dev, unsigned int dest_id)
{
  unsigned int val, errors=0;

  ibc2600_dest_cfg_s dest_cfg = ibuf_dev->dest_cfg[dest_id];

  val=ibc2600_get_dest_cfg_reg(ibuf_dev, dest_id, _IBC_2600_DEST_CFG_FEED_ADDR);
  errors+=ibc2600_check_value("_IBC_2600_DEST_CFG_FEED_ADDR", val, dest_cfg.feed_addr);
  val=ibc2600_get_dest_cfg_reg(ibuf_dev, dest_id, _IBC_2600_DEST_CFG_REQUESTER_ADDR);
  errors+=ibc2600_check_value("_IBC_2600_DEST_CFG_REQUESTER_ADDR", val, dest_cfg.req_addr);
  val=ibc2600_get_dest_cfg_reg(ibuf_dev, dest_id, _IBC_2600_DEST_CFG_CHANNEL_ADDR);
  errors+=ibc2600_check_value("_IBC_2600_DEST_CFG_CHANNEL_ADDR", val, dest_cfg.channel_addr);
  val=ibc2600_get_dest_cfg_reg(ibuf_dev, dest_id, _IBC_2600_DEST_CFG_SPAN_A_ADDR);
  errors+=ibc2600_check_value("_IBC_2600_DEST_CFG_SPAN_A_ADDR", val, dest_cfg.span_a_addr);
  val=ibc2600_get_dest_cfg_reg(ibuf_dev, dest_id, _IBC_2600_DEST_CFG_SPAN_B_ADDR);
  errors+=ibc2600_check_value("_IBC_2600_DEST_CFG_SPAN_B_ADDR", val, dest_cfg.span_b_addr);
  val=ibc2600_get_dest_cfg_reg(ibuf_dev, dest_id, _IBC_2600_DEST_CFG_TERMINAL_B_ADDR);
  errors+=ibc2600_check_value("_IBC_2600_DEST_CFG_TERMINAL_B_ADDR", val, dest_cfg.terminal_b_addr);
  val=ibc2600_get_dest_cfg_reg(ibuf_dev, dest_id, _IBC_2600_DEST_CFG_DEST_MODE);
  errors+=ibc2600_check_value("_IBC_2600_DEST_CFG_DEST_MODE", val, dest_cfg.dest_mode.byte);
  val=ibc2600_get_dest_cfg_reg(ibuf_dev, dest_id, _IBC_2600_DEST_CFG_ST_ADDR);
  errors+=ibc2600_check_value("_IBC_2600_DEST_CFG_ST_ADDR", val, dest_cfg.st_addr);
  val=ibc2600_get_dest_cfg_reg(ibuf_dev, dest_id, _IBC_2600_DEST_CFG_DEST_NUM_UNITS);
  errors+=ibc2600_check_value("_IBC_2600_DEST_CFG_DEST_NUM_UNITS", val, dest_cfg.num_items);
  val=ibc2600_get_dest_cfg_reg(ibuf_dev, dest_id, _IBC_2600_DEST_CFG_IWAKE_THRESHOLD);
  errors+=ibc2600_check_value("_IBC_2600_DEST_CFG_IWAKE_THRESHOLD", val, dest_cfg.iwake_threshold);

  return errors;
}

void ibc2600_config_ibuf(ibc2600_ibuf_s *ibuf_dev)
{
  unsigned int i;
  for (i=0; i<ibuf_dev->nr_procs; i++)
    ibc2600_config_proc(ibuf_dev, i);
  for (i=0; i<ibuf_dev->nr_feeders; i++)
    ibc2600_config_feeder(ibuf_dev, i);
  for (i=0; i<ibuf_dev->nr_frame_checks; i++)
    ibc2600_config_frame_check(ibuf_dev, i);

  ibc2600_set_shared_reg(ibuf_dev, _IBC_2600_CFG_IWAKE_ADDR, ibuf_dev->iwake_addr);
  ibc2600_set_shared_reg(ibuf_dev, _IBC_2600_CFG_ERROR_IRQ_EN, ibuf_dev->error_irq_en);
  ibc2600_set_shared_reg(ibuf_dev, _IBC_2600_CFG_SECURE_TOUCH_EN, ibuf_dev->secure_touch_en);
  ibc2600_set_shared_reg(ibuf_dev, _IBC_2600_CFG_SECURE_TOUCH_HANDLING, ibuf_dev->secure_touch_handling);
}

unsigned int ibc2600_check_config_ibuf(ibc2600_ibuf_s *ibuf_dev)
{
  unsigned int i, val, check;
  unsigned int errors=0;
  for (i=0; i<ibuf_dev->nr_procs; i++)
    errors+=ibc2600_check_config_proc(ibuf_dev, i);
  for (i=0; i<ibuf_dev->nr_feeders; i++)
    errors+=ibc2600_check_config_feeder(ibuf_dev, i);
  for (i=0; i<ibuf_dev->nr_frame_checks; i++)
    errors+=ibc2600_check_config_frame_check(ibuf_dev, i);

  val = ibc2600_get_shared_reg(ibuf_dev, _IBC_2600_CFG_IWAKE_ADDR);
  check = ibuf_dev->iwake_addr;
  errors+=ibc2600_check_value("_IBC_2600_REG_IWAKE_ADDR", val, check);

  val = ibc2600_get_shared_reg(ibuf_dev, _IBC_2600_CFG_ERROR_IRQ_EN);
  check = ibuf_dev->error_irq_en & ((1<<ibuf_dev->nr_procs) - 1);
  errors+=ibc2600_check_value("_IBC_2600_CFG_ERROR_IRQ_EN", val, check);

  val = ibc2600_get_shared_reg(ibuf_dev, _IBC_2600_CFG_SECURE_TOUCH_EN);
  check = ibuf_dev->secure_touch_en;
  errors+=ibc2600_check_value("_IBC_2600_CFG_SECURE_TOUCH_EN", val, check);

  val = ibc2600_get_shared_reg(ibuf_dev, _IBC_2600_CFG_SECURE_TOUCH_HANDLING);
  check = ibuf_dev->secure_touch_handling;
  errors+=ibc2600_check_value("_IBC_2600_CFG_SECURE_TOUCH_HANDLING", val, check);

  return errors;
}

void ibc2600_setup_ibuf(ibc2600_ibuf_s *ibuf_dev)
{
  unsigned int i,j;
  unsigned int dest_nr=0;

  ibuf_dev->proc=ia_css_cpu_mem_alloc(ibuf_dev->nr_procs *sizeof(ibc2600_proc_s));
  ibuf_dev->feeder_cfg=ia_css_cpu_mem_alloc(ibuf_dev->nr_feeders *sizeof(ibc2600_feeder_cfg_s));
  ibuf_dev->frame_check_cfg=ia_css_cpu_mem_alloc(ibuf_dev->nr_frame_checks *sizeof(ibc2600_frame_check_cfg_s));

  for (i=0; i<ibuf_dev->nr_procs; i++)
  {
    ibuf_dev->proc[i].nr_dests = ibuf_dev->dests_p_proc[i];
    ibuf_dev->proc[i].proc_id = i;

    ibuf_dev->proc[i].cfg.dest_cfg = ia_css_cpu_mem_alloc(ibuf_dev->proc[i].nr_dests * sizeof(ibc2600_dest_cfg_s *));
    ibuf_dev->proc[i].cfg.dest_en = ia_css_cpu_mem_alloc(ibuf_dev->proc[i].nr_dests * sizeof(unsigned char));
    ibuf_dev->proc[i].cmd.dest_ack = ia_css_cpu_mem_alloc(ibuf_dev->proc[i].nr_dests * sizeof(unsigned int));


    ibuf_dev->proc[i].cmd.cmd         = ibc2600_reg_addr(ibuf_dev, i, _IBC_2600_GROUP_PROC_CMD, _IBC_2600_PROC_CMD_CMD);

    for(j=0; j<ibuf_dev->proc[i].nr_dests; j++)
    {
      dest_nr++;
      ibuf_dev->proc[i].cfg.dest_en[j] = 0;
    }
  }

  for (i=0; i<ibuf_dev->nr_feeders; i++)
  {
    ibuf_dev->feeder_cfg[i].feed_cmd   = ibc2600_reg_addr(ibuf_dev, i, _IBC_2600_GROUP_FEEDER, _IBC_2600_FEED_CMD_CMD);
  }

  for (i=0; i<ibuf_dev->nr_frame_checks; i++)
  {
    ibuf_dev->frame_check_cfg[i].mode.byte  = 0;
  }

  ibuf_dev->idrain_rcv   = ibc2600_reg_addr(ibuf_dev, 0, _IBC_2600_GROUP_SHARED, _IBC_2600_CMD_IDRAIN_RECEIVE);
  ibuf_dev->srst_proc    = ibc2600_reg_addr(ibuf_dev, 0, _IBC_2600_GROUP_SHARED, _IBC_2600_CFG_SRST_PROC);
  ibuf_dev->srst_feeder  = ibc2600_reg_addr(ibuf_dev, 0, _IBC_2600_GROUP_SHARED, _IBC_2600_CFG_SRST_FEEDER);

  ibuf_dev->dest_cfg=ia_css_cpu_mem_alloc(dest_nr * sizeof(ibc2600_dest_cfg_s));
  dest_nr=0;
  for (i=0; i<ibuf_dev->nr_procs; i++)
  {
    for(j=0; j<ibuf_dev->proc[i].nr_dests; j++)
    {
      ibuf_dev->proc[i].cfg.dest_cfg[j] = &(ibuf_dev->dest_cfg[dest_nr++]);
      ibuf_dev->proc[i].cfg.dest_cfg[j]->dest_mode.byte = 0;
    }
  }

//   ibuf_dev->iwake_addr = 0;
//   ibuf_dev->error_irq_en = 0;
  ibuf_dev->secure_touch_en = 0;
  ibuf_dev->secure_touch_handling = 0;

  ibuf_dev->nr_dest_cfgs = dest_nr;

  for (i=0; i<ibuf_dev->nr_dest_cfgs; i++)
    ibuf_dev->dest_cfg[i].dest_id = i;
}
