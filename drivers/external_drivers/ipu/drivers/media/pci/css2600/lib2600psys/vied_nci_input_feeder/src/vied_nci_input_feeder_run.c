#include <hive/support.h>
#include "vied_nci_input_feeder_run.h"

static inline void store_cmem(unsigned int address, unsigned int value)
{
  int MEM(cmem) *p;
  p = (int MEM(cmem) *)address;
  *p = value;
}

static unsigned int ibc2600_reg_addr(unsigned int bank_id,
				     unsigned int group_id, unsigned int reg_nr)
{
	unsigned int reg_addr = 0;
	reg_addr |= (bank_id<<(_IBC_2600_REG_IDX_BITS + _IBC_2600_GROUP_IDX_BITS));
	reg_addr |= (group_id<<_IBC_2600_REG_IDX_BITS);
	reg_addr |= (reg_nr);
	return SP_IBC2600_MG_ADDR + (reg_addr * _IBC_2600_REG_ALIGN);
}

static void ibc2600_set_sid_cmd_reg(unsigned int proc_nr, unsigned int reg_nr, unsigned int val)
{
	unsigned int addr = ibc2600_reg_addr(proc_nr, _IBC_2600_GROUP_PROC_CMD, reg_nr);
	store_cmem(addr, val);
}

// Set start address of a stream. Results in an INIT event
static void vied_nci_infeeder_stream_set_start_address(unsigned int stream_id, unsigned int st_addr)
{
  unsigned int addr;

  //set dest configure ibc2600_set_dest_cfg_reg
  addr = ibc2600_reg_addr(stream_id, _IBC_2600_GROUP_DEST_CFG, _IBC_2600_DEST_CFG_ST_ADDR);
  store_cmem(addr, st_addr);

  ibc2600_set_sid_cmd_reg(stream_id, _IBC_2600_PROC_CMD_CMD, _IBC_2600_CMD_INIT_VALUE);
}

// Run stream function ==> does a buffer transfer
void vied_nci_infeeder_stream_run_next(unsigned int stream_id, unsigned int start_address)
{
  unsigned int cmd_token;
#ifdef DEBUG_INPUT_FEEDER
  OP___printstring("Start Set Input Feeder next command with start address\n");
  OP___dump(__LINE__, stream_id);
  OP___dump(__LINE__, start_address);
#endif

  // Set Start Address (results in an Init event)
  vied_nci_infeeder_stream_set_start_address(stream_id, start_address);

  // Run buffer_tranfers (results in an ACK event)
  cmd_token = (_IBC_2600_CMD_STORE_2ND_BUFFER_MODE << _IBC_2600_CMD_DEST_IDX(0));

  ibc2600_set_sid_cmd_reg(stream_id, _IBC_2600_PROC_CMD_CMD, cmd_token);

}

