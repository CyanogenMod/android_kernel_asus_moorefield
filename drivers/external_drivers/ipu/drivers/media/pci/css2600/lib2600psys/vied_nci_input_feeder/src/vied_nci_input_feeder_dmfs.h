#ifndef _DMFS_H_
#define _DMFS_H_

// #include "vied_nci_input_feeder_defines.h"

// Device Config Data and Address width. Can't we generate this from a common location?
#define DMFS_CFG_BUS_DATA_WIDTH    32
#define DMFS_CFG_BUS_ADDR_WIDTH    32
#define DMFS_REG_ALIGNEMENT        ((DMFS_CFG_BUS_ADDR_WIDTH)/(8))

#define SID_CMD_REQUEST_REG        0
#define SID_ACK_RESPONSE_ADDR_REG  1
#define SID_ACK_RESPONSE_REG       2
#define SID_SET_BUFFER_SIZE_REG    3
#define SID_AVAIL_BUFFER_SIZE_REG  4
#define SID_COMPUTE_UNIT_SIZE_REG  5

#define S2M_CMD_REQUEST_REG        0
#define S2M_PIX_WIDTH_ID_REG       1
#define S2M_START_ADDRESS_REG      2
#define S2M_STOP_ADDRESS_REG       3
#define S2M_STRIDE_REG             4
#define S2M_NUM_ITEMS_REG          5
#define S2M_ACK_RESPONSE_ADDR_REG  6


typedef struct s_sid_dmfs {
  unsigned int base_addr;
  unsigned int sid_cmd_request;
  unsigned int sid_ack_response_addr;
  unsigned int sid_ack_response;
  unsigned int sid_set_buffer_size;
  unsigned int sid_avail_buffer_size;
  unsigned int sid_compute_unit_size;
} t_sid_dmfs;

typedef struct s_s2m_dmfs {
  unsigned int base_addr;
  unsigned int s2m_cmd_request;
  unsigned int s2m_num_items;
  unsigned int s2m_ack_response_addr;
} t_s2m_dmfs;


void dmfs_set_reg(unsigned int base_addr, unsigned int reg, unsigned int value);
unsigned int dmfs_get_reg(unsigned int base_addr, unsigned int reg);

void dmfs_static_sid_config(t_sid_dmfs p_sid_dmfs, t_s2m_dmfs p_s2m_dmfs);
void dmfs_set_buffer_size_sid(t_sid_dmfs p_sid_dmfs);


#endif /*_DMFS_H_*/


