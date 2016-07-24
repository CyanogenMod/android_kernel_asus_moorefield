#ifndef _ga_acb_defs_h
#define _ga_acb_defs_h

#include "ga_common/hrt/include/skycam.h"



#define _GA_ACB_BYTES_PER_ELEM           4		

/* --------------------------------------------------*/


/* --------------------------------------------------*/
/* REGISTER INFO */
/* --------------------------------------------------*/

#define ACB_CMD_FIFO_TAIL_REG_OFFSET               (GA_ACB_CMD_FIFO_TAIL_REG_ID * _GA_ACB_BYTES_PER_ELEM) 
#define ACB_REG2_REG_OFFSET                        (GA_ACB_REG2_REG_ID          * _GA_ACB_BYTES_PER_ELEM) 


//------------------------- ACB deifntions --------------------
#define ACB_CMD_FSM_WDTH                            2
#define ACB_CMD_FSM_ACT                             3
#define ACB_CMD_FSM_STATE_IDLE                     (BITVECTOR(TO_UNSIGNED(NATURAL(0),ACB_CMD_FSM_WDTH)))
#define ACB_CMD_FSM_STATE_UPLOAD_IN_PROG           (BITVECTOR(TO_UNSIGNED(NATURAL(1),ACB_CMD_FSM_WDTH))) 
//--      WHEN (TO_BITVECTOR(NATURAL(0),4)) BEGIN

#define LINE_NUM_LSB   16
#define LINE_NUM_RANGE 31 DOWNTO 17
#define LINE_NUM_WIDTH 15
#define CMDID_LSB  0
#define CMDID_RANGE 4 DOWNTO 0
#define INIT_COMMAND B"10000"
#define PROCESS_N_LINES B"00001"
#define CNTRLID_RANGE 10 DOWNTO 8
#define FULL_ACK_WIDTH 8
#define ACK_FIFO_DATA_WIDTH 1

// ACB Upload FSM defines
#define RD_NXT_CMD 0
#define STALL 1
#define ACK_FIFO_WR 2

// Command defs

#define PIX_CMD_CFG_SET_LO_IDX 12

#define GA_ACK_FIFO_CNT_WIDTH 3

#endif /* _ga_acb_defs_h */ 
