#ifndef _DMF_H_
#define _DMF_H_

// #include "vied_nci_input_feeder_defines.h"

#define SID0_IPF_DMF_BUFFER_SIZE 8
#define SID1_IPF_DMF_BUFFER_SIZE 8
#define SID2_IPF_DMF_BUFFER_SIZE 12
#define SID3_IPF_DMF_BUFFER_SIZE 18
#define SID4_IPF_DMF_BUFFER_SIZE 18
#define SID0_DMF_BURST_TIMER     64 - 1
#define SID1_DMF_BURST_TIMER     64 - 1
#define SID2_DMF_BURST_TIMER     32 - 1
#define SID3_DMF_BURST_TIMER     32 - 1
#define SID4_DMF_BURST_TIMER     32 - 1

#define DMF_REG_ALIGNEMENT ((IPF_CFG_BUS_ADDR_WIDTH)/(8))
#define DMF_REG_FILE_SIZE         64

#define SL_CFG_SLV_CNT_REG         0
#define SL_CFG_SLV_VALID0_REG      1
#define SL_CFG_SLV_VALID1_REG      2
#define SL_CFG_MTV_CNT_REG         3
#define SID_BURST_TIMER_REG        4
#define SID_BUFFER_SIZE_REG        5


typedef struct s_sid_dmf {
  unsigned int base_addr;
  unsigned int sl_cfg_slv_cnt;
  unsigned int sl_cfg_slv_valid0;
  unsigned int sl_cfg_slv_valid1;
  unsigned int sl_cfg_mtv_cnt;
  unsigned int sid_burst_timer;
  unsigned int sid_buffer_size;
} t_sid_dmf;

void dmf_set_reg(unsigned int base_addr, unsigned int reg, unsigned int value);
unsigned int dmf_get_reg(unsigned int base_addr, unsigned int reg);

void dmf_static_sid_config(t_sid_dmf p_sid_dmf);
void dmf_set_buffer_size_sid(t_sid_dmf p_sid_dmf);
void dmf_set_burst_timer_sid(t_sid_dmf p_sid_dmf);


#endif /*_DMF_H_*/


