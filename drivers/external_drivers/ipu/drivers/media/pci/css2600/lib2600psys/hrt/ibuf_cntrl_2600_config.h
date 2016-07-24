#ifndef _ibuf_cntrl_2600_config_h_
#define _ibuf_cntrl_2600_config_h_

#include <ibuf_cntrl_2600_defs.h>

typedef union {
  struct {
    unsigned char is_feeder         : 1;
    unsigned char config_dma        : 1;
    unsigned char iwake_en          : 1;
    unsigned char disable_spana_rst : 1;
    unsigned char disable_spanb_rst : 1;
    unsigned char others            : 3;
  } bits;
  unsigned char byte;
} ibc2600_dest_mode_s;

typedef union {
  struct {
    unsigned char check_type: 1;
    unsigned char rep_en    : 1;
    unsigned char others    : 6;
  } bits;
  unsigned char byte;
} ibc2600_check_mode_s;

typedef struct {
  unsigned int feeder_id;
  unsigned int feed_cmd;
  unsigned int dma_ack;
  unsigned int ack_addr;
  unsigned int req_addr;
  unsigned int channel_addr;
  unsigned int units_p_line_in;
  unsigned int units_out_p_in;
  unsigned int last_units_out;
  unsigned int height;
  unsigned char wait_for_other_feeder;
} ibc2600_feeder_cfg_s;

typedef struct {
  unsigned int          frame_check_id;
  unsigned int          enable;
  ibc2600_check_mode_s  mode;
  unsigned int          sid_id;
  unsigned int          trigger_offset;
  unsigned int          trigger_repeat;
  unsigned int          addr;
  unsigned int          token;
} ibc2600_frame_check_cfg_s;

typedef struct {
  unsigned int        dest_id;
  unsigned int        feed_addr;
  unsigned int        req_addr;
  unsigned int        channel_addr;
  unsigned int        span_a_addr;
  unsigned int        span_b_addr;
  unsigned int        terminal_b_addr;
  ibc2600_dest_mode_s dest_mode;
  unsigned int        st_addr;
  unsigned int        num_items;
  unsigned int        iwake_threshold;
} ibc2600_dest_cfg_s;

typedef struct {
  unsigned int        str2mmio_addr;
  unsigned char       store_cmd;
  unsigned int        items_p_unit;
  unsigned int        units_p_line;
  unsigned int        lines_p_frame;
  unsigned int        units_p_ibuf;
  unsigned int        st_addr;
  unsigned char       sync_frame;
  ibc2600_dest_cfg_s  **dest_cfg;
  unsigned char       *dest_en;
} ibc2600_proc_cfg_s;

typedef struct {
  unsigned int        cmd;
  unsigned int        ack_addr;
  unsigned int        s2m_ack;
  unsigned int        snd_buf_ack;
  unsigned int        snd_buf_cmd_addr;
  unsigned int        sidpid;
  unsigned int        *dest_ack;
} ibc2600_proc_cmd_s;

typedef struct {
  unsigned int        proc_id;
  unsigned char       nr_dests;
  ibc2600_proc_cmd_s  cmd;
  ibc2600_proc_cfg_s  cfg;
} ibc2600_proc_s;

typedef struct {
  unsigned int nr_procs;
  unsigned int nr_dest_cfgs;
  unsigned int nr_feeders;
  unsigned int nr_frame_checks;

  unsigned int base_address;

  unsigned char *dests_p_proc;
  unsigned char *has_2nd_buff;

  unsigned int idrain_rcv;
  unsigned int iwake_addr;
  unsigned int error_irq_en;
  unsigned int srst_proc;
  unsigned int srst_feeder;
  unsigned int secure_touch_en;
  unsigned int secure_touch_handling;

  ibc2600_proc_s          *proc;
  ibc2600_dest_cfg_s      *dest_cfg;
  ibc2600_feeder_cfg_s    *feeder_cfg;
  ibc2600_frame_check_cfg_s *frame_check_cfg;
} ibc2600_ibuf_s;

unsigned int ibc2600_check_value(char *reg_name, unsigned int got, unsigned int expected);
unsigned int ibc2600_reg_addr(ibc2600_ibuf_s *ibuf_dev, unsigned int bank_id, unsigned int group_id, unsigned int reg_nr);
unsigned int ibc2600_reg_addr_with_base(unsigned int base_address, unsigned int bank_id, unsigned int group_id, unsigned int reg_nr);

void ibc2600_set_shared_reg(ibc2600_ibuf_s *ibuf_dev, unsigned int reg_nr, unsigned int val);
unsigned int ibc2600_get_shared_reg(ibc2600_ibuf_s *ibuf_dev, unsigned int reg_nr);

void ibc2600_set_sid_cfg_reg(ibc2600_ibuf_s *ibuf_dev, unsigned int proc_nr, unsigned int reg_nr, unsigned int val);
unsigned int ibc2600_get_sid_cfg_reg(ibc2600_ibuf_s *ibuf_dev, unsigned int proc_nr, unsigned int reg_nr);
void ibc2600_set_sid_cmd_reg(ibc2600_ibuf_s *ibuf_dev, unsigned int proc_nr, unsigned int reg_nr, unsigned int val);
unsigned int ibc2600_get_sid_cmd_reg(ibc2600_ibuf_s *ibuf_dev, unsigned int proc_nr, unsigned int reg_nr);
unsigned int ibc2600_get_sid_stat_reg(ibc2600_ibuf_s *ibuf_dev, unsigned int proc_nr, unsigned int reg_nr);

void ibc2600_set_dest_cfg_reg(ibc2600_ibuf_s *ibuf_dev, unsigned int dest_nr, unsigned int reg_nr, unsigned int val);
unsigned int ibc2600_get_dest_cfg_reg(ibc2600_ibuf_s *ibuf_dev, unsigned int dest_nr, unsigned int reg_nr);

void ibc2600_set_frame_check_reg(ibc2600_ibuf_s *ibuf_dev, unsigned int frame_check_nr, unsigned int reg_nr, unsigned int val);
unsigned int ibc2600_get_frame_check_reg(ibc2600_ibuf_s *ibuf_dev, unsigned int frame_check_nr, unsigned int reg_nr);

void ibc2600_set_feeder_reg(ibc2600_ibuf_s *ibuf_dev, unsigned int feeder_nr, unsigned int reg_nr, unsigned int val);
unsigned int ibc2600_get_feeder_reg(ibc2600_ibuf_s *ibuf_dev, unsigned int feeder_nr, unsigned int reg_nr);

void ibc2600_config_feeder(ibc2600_ibuf_s *ibuf, unsigned int feeder_id);
unsigned int ibc2600_check_config_feeder(ibc2600_ibuf_s *ibuf, unsigned int feeder_id);

void ibc2600_config_frame_check(ibc2600_ibuf_s *ibuf, unsigned int frame_check_id);
unsigned int ibc2600_check_config_frame_check(ibc2600_ibuf_s *ibuf, unsigned int frame_check_id);

void ibc2600_ibc2600_config_proc_cfg(ibc2600_ibuf_s *ibuf, unsigned int proc_id);
void ibc2600_ibc2600_config_proc_cmd(ibc2600_ibuf_s *ibuf, unsigned int proc_id);
unsigned int ibc2600_check_ibc2600_config_proc_cfg(ibc2600_ibuf_s *ibuf_dev, unsigned int proc_id);
unsigned int ibc2600_check_ibc2600_config_proc_cmd(ibc2600_ibuf_s *ibuf_dev, unsigned int proc_id);

void ibc2600_config_proc(ibc2600_ibuf_s *ibuf_dev, unsigned int proc_id);
unsigned int ibc2600_check_config_proc(ibc2600_ibuf_s *ibuf_dev, unsigned int proc_id);

void ibc2600_config_dest(ibc2600_ibuf_s *ibuf_dev, unsigned int dest_id);
unsigned int ibc2600_check_config_dest(ibc2600_ibuf_s *ibuf_dev, unsigned int dest_id);

void ibc2600_config_ibuf(ibc2600_ibuf_s *ibuf_dev);
unsigned int ibc2600_check_config_ibuf(ibc2600_ibuf_s *ibuf_dev);

void ibc2600_setup_ibuf(ibc2600_ibuf_s *ibuf_dev);

#endif
