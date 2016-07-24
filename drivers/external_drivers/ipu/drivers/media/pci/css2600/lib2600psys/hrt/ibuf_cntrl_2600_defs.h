#ifndef _ibuf_cntrl_2600_defs_h_
#define _ibuf_cntrl_2600_defs_h_

#include <stream2mmio_defs.h>
// #include <dma4_defs.h>

#define _IBC_2600_REG_ALIGN 4

  /* alignment of register banks: */
#define _IBC_2600_REG_IDX_BITS          4
#define _IBC_2600_GROUP_IDX_BITS        3

#define _IBC_2600_GROUP_SHARED          0
#define _IBC_2600_GROUP_PROC_CFG        1
#define _IBC_2600_GROUP_PROC_CMD        2
#define _IBC_2600_GROUP_PROC_STAT       3
#define _IBC_2600_GROUP_DEST_CFG        4
#define _IBC_2600_GROUP_FR_CHECK        5
#define _IBC_2600_GROUP_IRQ_CHECK       5
#define _IBC_2600_GROUP_FEEDER          6


  /* the actual amount of registers: */
#define _IBC_2600_CONFIG_REGS_SHARED                      9
  /* for the proc reg sets, the total amount is depended on the nr dests. The number given here is WITHOUT the dest depended registers:*/
#define _IBC_2600_CONFIG_REGS_PROC_CFG                    7
#define _IBC_2600_CONFIG_REGS_PROC_CMD                    8
#define _IBC_2600_CONFIG_REGS_PROC_STAT                   8
#define _IBC_2600_CONFIG_REGS_DEST_CFG                    10
#define _IBC_2600_CONFIG_REGS_IRQ_CHECK                   7
#define _IBC_2600_CONFIG_REGS_FEEDER                      15
  /* for the proc reg sets, the total amount is depended on the nr dests:*/
#define _IBC_2600_CONFIG_REGS_PROC_CFG_DEST(nr_dests)     (_IBC_2600_CONFIG_REGS_PROC_CFG + (nr_dests))
#define _IBC_2600_CONFIG_REGS_PROC_CMD_DEST(nr_dests)     (_IBC_2600_CONFIG_REGS_PROC_CMD + (nr_dests))
#define _IBC_2600_CONFIG_REGS_PROC_STAT_DEST(nr_dests)    (_IBC_2600_CONFIG_REGS_PROC_STAT + 2*(nr_dests))


  /* time out bits for a passive str2mmio ack, maximum time out between polls value is 2^_IBC_2600_TIME_OUT_BITS - 1 */
#define _IBC_2600_TIME_OUT_BITS         5

/* Str2MMIO defines */
#define _IBC_2600_STREAM2MMIO_ACK_REG               1
#define _IBC_2600_STREAM2MMIO_CMD_TOKEN_MSB         _STREAM2MMIO_CMD_TOKEN_CMD_MSB
#define _IBC_2600_STREAM2MMIO_CMD_TOKEN_LSB         _STREAM2MMIO_CMD_TOKEN_CMD_LSB
#define _IBC_2600_STREAM2MMIO_NUM_ITEMS_BITS        _STREAM2MMIO_PACK_NUM_ITEMS_BITS
#define _IBC_2600_STREAM2MMIO_CMD_SYNC              _STREAM2MMIO_CMD_TOKEN_SYNC_FRAME
#define _IBC_2600_STREAM2MMIO_CMD_STORE_PACKET      _STREAM2MMIO_CMD_TOKEN_STORE_PACKETS
#define _IBC_2600_STREAM2MMIO_CMD_STORE_WORDS       _STREAM2MMIO_CMD_TOKEN_STORE_WORDS
#define _IBC_2600_STREAM2MMIO_ACK_EOF_BIT           _STREAM2MMIO_PACK_ACK_EOF_BIT
#define _IBC_2600_STREAM2MMIO_ACK_EOP_BIT           _STREAM2MMIO_PACK_ACK_EOP_BIT
#define _IBC_2600_STREAM2MMIO_ACK_ERROR_BIT         _STREAM2MMIO_PACK_ACK_ERR_BIT


/* command tokens */
#define _IBC_2600_CMD_INIT_VALUE              0
#define _IBC_2600_CMD_FIELD_BITS              2
#define _IBC_2600_CMD_DEST_IDX(dest_nr)       ((dest_nr) * _IBC_2600_CMD_FIELD_BITS)
#define _IBC_2600_CMD_DEST_DISABLE            0
#define _IBC_2600_CMD_STORE_ADDR_NEXT_TOKEN   1
#define _IBC_2600_CMD_STORE_ADDR_CONFIG       2
#define _IBC_2600_CMD_STORE_2ND_BUFFER_MODE   3


/* acknowledge token definition */
#define _IBC_2600_ACK_TYPE_BITS               2

#define _IBC_2600_ACK_TYPE_IDX                0
#define _IBC_2600_ACK_SECURE_TOUCH_IDX        (_IBC_2600_ACK_TYPE_IDX + _IBC_2600_ACK_TYPE_BITS)
#define _IBC_2600_ACK_S2M_ERROR_IDX           (_IBC_2600_ACK_SECURE_TOUCH_IDX + 1)
#define _IBC_2600_ACK_LINE_P_FRAME_ERROR_IDX  (_IBC_2600_ACK_S2M_ERROR_IDX + 1)
#define _IBC_2600_ACK_LINE_P_FRAME_RCVD_IDX   (_IBC_2600_ACK_LINE_P_FRAME_ERROR_IDX + 1)

#define _IBC_2600_ACK_TYPE_INIT               0
#define _IBC_2600_ACK_TYPE_STORE              1
#define _IBC_2600_ACK_FALSE_CMD               2

#define _IBC_2600_ACK_ERROR_TYPE_NO_ERROR     0
#define _IBC_2600_ACK_ERROR_TYPE_NR_ITEMS     1
#define _IBC_2600_ACK_ERROR_TYPE_NR_UNITS     2
#define _IBC_2600_ACK_ERROR_TYPE_NR_LINES     3


/*register numbers in the shared register bank:*/
#define _IBC_2600_CMD_IDRAIN_RECEIVE           0
#define _IBC_2600_CFG_IWAKE_ADDR               1
#define _IBC_2600_CFG_SRST_PROC                2
#define _IBC_2600_CFG_SRST_FEEDER              3
#define _IBC_2600_STAT_ARBITERS_STATUS         4
#define _IBC_2600_CFG_ERROR_REG_SET            5
#define _IBC_2600_CFG_ERROR_IRQ_EN             6
#define _IBC_2600_CFG_SECURE_TOUCH_EN          7
#define _IBC_2600_CFG_SECURE_TOUCH_HANDLING    8

#define _IBC_2600_REG_SET_CRUN_ERRORS          9 /* NO PHYSICAL REGISTER!! Only used in HSS model (if needed?)*/


/*register numbers in the proc config bank: */
#define _IBC_2600_PROC_CFG_STR2MMIO_PROC_ADDR         0
#define _IBC_2600_PROC_CFG_STR2MMIO_STORE_CMD         1
#define _IBC_2600_PROC_CFG_ITEMS_P_UNIT               2
#define _IBC_2600_PROC_CFG_UNITS_P_LINE               3
#define _IBC_2600_PROC_CFG_LINES_P_FRAME              4
#define _IBC_2600_PROC_CFG_UNITS_P_IBUF               5
#define _IBC_2600_PROC_CFG_SYNC_FRAME                 6
#define _IBC_2600_PROC_CFG_DEST_ENABLED(dest_nr)      (7 + (dest_nr))

/*register numbers in the proc cmd/ack bank: */
#define _IBC_2600_PROC_CMD_CMD                        0
#define _IBC_2600_PROC_CMD_ACK_ADDR                   1
#define _IBC_2600_PROC_CMD_EVENTQUE_SIDPID            2
#define _IBC_2600_PROC_CMD_2NDBUF_CMD_ADDR            3
#define _IBC_2600_PROC_CMD_2ND_BUFFER_ACK             4
#define _IBC_2600_PROC_CMD_STR2MMIO_ACK               5
#define _IBC_2600_PROC_CMD_ERROR_REG                  6
#define _IBC_2600_PROC_CMD_CLEAR_ERROR_REG            7
#define _IBC_2600_PROC_CMD_DEST_ACK(dest_nr)          (8 + (dest_nr))

/*register numbers in the proc status bank: */
#define _IBC_2600_PROC_STAT_CUR_CMDS                    0
#define _IBC_2600_PROC_STAT_CUR_UNIT_IN_LINE            1
#define _IBC_2600_PROC_STAT_CUR_LINE                    2
#define _IBC_2600_PROC_STAT_MAIN_CNTRL_STATE            3
#define _IBC_2600_PROC_STAT_2NDBUF_CNTRL_STATE          4
#define _IBC_2600_PROC_STAT_DEST0_CUR_ACK_UNIT          5
#define _IBC_2600_PROC_STAT_DEST0_CUR_ACK_UNIT_IN_LINE  6
#define _IBC_2600_PROC_STAT_DEST0_CUR_ACK_LINE          7
#define _IBC_2600_PROC_STAT_DEST_SYNC_STATE(dest_nr)    (8 + ((dest_nr) * 2))
#define _IBC_2600_PROC_STAT_DEST_CUR_CMDS(dest_nr)      (9 + ((dest_nr) * 2))

/* registers per destination configuration set: */
#define _IBC_2600_DEST_CFG_FEED_ADDR            0
#define _IBC_2600_DEST_CFG_REQUESTER_ADDR       1
#define _IBC_2600_DEST_CFG_CHANNEL_ADDR         2
#define _IBC_2600_DEST_CFG_SPAN_A_ADDR          3
#define _IBC_2600_DEST_CFG_SPAN_B_ADDR          4
#define _IBC_2600_DEST_CFG_TERMINAL_B_ADDR      5
#define _IBC_2600_DEST_CFG_DEST_MODE            6
#define _IBC_2600_DEST_CFG_ST_ADDR              7
#define _IBC_2600_DEST_CFG_DEST_NUM_UNITS       8
#define _IBC_2600_DEST_CFG_IWAKE_THRESHOLD      9


/* registers per Frame check: */
#define _IBC_2600_FR_C_CFG_ENABLE               0
#define _IBC_2600_FR_C_CFG_CHECK_MODE           1
#define _IBC_2600_FR_C_CFG_SID_PROC_ID          2
#define _IBC_2600_FR_C_CFG_TRIGGER_OFFSET       3
#define _IBC_2600_FR_C_CFG_TRIGGER_REPEAT_VAL   4
#define _IBC_2600_FR_C_CFG_ADDR                 5
#define _IBC_2600_FR_C_CFG_TOKEN                6


/*register addresses for each feeder: */
#define _IBC_2600_FEED_CMD_CMD                    0
#define _IBC_2600_FEED_DMA_ACK                    1
#define _IBC_2600_FEED_CFG_ACK_ADDR               2
#define _IBC_2600_FEED_CFG_REQUESTER_ADDR         3
#define _IBC_2600_FEED_CFG_CHANNEL_ADDR           4
#define _IBC_2600_FEED_CFG_UNITS_P_LINE_IN        5
#define _IBC_2600_FEED_CFG_UNITS_OUT_P_IN         6
#define _IBC_2600_FEED_CFG_LAST_UNITS_OUT         7
#define _IBC_2600_FEED_CFG_HEIGHT                 8
#define _IBC_2600_FEED_CFG_SIDPID                 9
#define _IBC_2600_FEED_CFG_WAIT_ON_OTHER_FEEDERS  10
#define _IBC_2600_FEED_STAT_CUR_UNIT_IN_LINE_IN   11
#define _IBC_2600_FEED_STAT_CUR_LINE_IN           12
#define _IBC_2600_FEED_STAT_CUR_SND_CMDS          13
#define _IBC_2600_FEED_STAT_CUR_RCVD_ACKS         14

#endif
