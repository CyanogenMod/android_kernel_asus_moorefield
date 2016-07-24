// generated file from <device>_reg_descr.hsd by hive_crgen.pl
#ifndef _strmcioconv_ccbcr_h
#define _strmcioconv_ccbcr_h


#define  ACK_ADDR_ADDR  0x00004
#define  ACK_CMD_ADDR  0x00000






#define STRMCIOCONV_CCBCR_PARAM \
CONSTANT  ACK_CMD_REG_ID    : INTEGER := 0 ; \
CONSTANT  ACK_ADDR_REG_ID    : INTEGER := 1 ; \
\
CONSTANT  ACK_ADDR_REG_WIDTH    : INTEGER := 32 ; \
CONSTANT  ACK_CMD_REG_WIDTH    : INTEGER := 32 ; \
\
CONSTANT  ACK_ADDR_REG_RSTVAL    : NATURAL := 0 ; \
CONSTANT  ACK_CMD_REG_RSTVAL    : NATURAL := 0 ; \
\
CONSTANT  STRMCIOCONV_NOF_REGS : NATURAL := 2 ; \
CONSTANT  STRMCIOCONV_NOF_REGS_BANK : NATURAL := 2 ; \
CONSTANT  c_num_of_regs : NATURAL := 2 ; \
CONSTANT  c_num_of_regs_bank : NATURAL :=  2; \
\
\
CONSTANT p_data_width                             : D1<INTEGER>; \
p_data_width{ACK_ADDR_REG_ID} := ACK_ADDR_REG_WIDTH   ; \
p_data_width{ACK_CMD_REG_ID} := ACK_CMD_REG_WIDTH   ; \
\
FEATURE c_reg_rst_val                             : D1<BITVECTOR>; \
c_reg_rst_val{ACK_ADDR_REG_ID} := BITVECTOR(TO_UNSIGNED(NATURAL(ACK_ADDR_REG_RSTVAL), p_data_width{ACK_ADDR_REG_ID})); \
c_reg_rst_val{ACK_CMD_REG_ID} := BITVECTOR(TO_UNSIGNED(NATURAL(ACK_CMD_REG_RSTVAL), p_data_width{ACK_CMD_REG_ID})); \
\
FEATURE p_reg_wmask                             : D1<BITVECTOR>;  \
 p_reg_wmask{ACK_ADDR_REG_ID}  := X"FFFFFFFF";\
 p_reg_wmask{ACK_CMD_REG_ID}  := X"FFF7FFFF";\
\
FEATURE p_reg_rmask                             : D1<BITVECTOR>;  \
 p_reg_rmask{ACK_ADDR_REG_ID}  := X"FFFFFFFF";\
 p_reg_rmask{ACK_CMD_REG_ID}  := X"FFFFFFFF";\
\
FEATURE p_reg_womask                             : D1<BITVECTOR>;  \
 p_reg_womask{ACK_ADDR_REG_ID}  := X"00000000";\
 p_reg_womask{ACK_CMD_REG_ID}  := X"00000000";\
\
FEATURE p_reg_romask                             : D1<BITVECTOR>;  \
 p_reg_romask{ACK_ADDR_REG_ID}  := X"00000000";\
 p_reg_romask{ACK_CMD_REG_ID}  := X"00080000";\
\
FEATURE p_func_en                             : D1<BOOLEAN>;  \
 p_func_en{ACK_ADDR_REG_ID}  := FALSE; \
 p_func_en{ACK_CMD_REG_ID}  := TRUE; \


#define STRMCIOCONV_CCBCR_SIGNALS \
SIGNAL func_en                                : D1<BIT>(STRMCIOCONV_NOF_REGS_BANK); \
SIGNAL wire_reg_we                            : D1<BIT>(STRMCIOCONV_NOF_REGS_BANK); \
SIGNAL wire_reg_outputs                       : D1<BITVECTOR>(p_data_width); \
SIGNAL func_data                              : D1<BITVECTOR>(p_data_width);

#define ACK_CMD_CMD_RANGE   19
#define ACK_CMD_MESSAGE_RANGE   18 DOWNTO 0
#define ACK_CMD_PID_RANGE   25 DOWNTO 20
#define ACK_CMD_SID_RANGE   31 DOWNTO 26

#define Ack_Cmd_CMD_WIDTH  1
#define Ack_Cmd_MESSAGE_WIDTH  19
#define Ack_Cmd_PID_WIDTH  6
#define Ack_Cmd_SID_WIDTH  6


#endif
