#ifndef _GA_ACB_H
#define _GA_ACB_H

#include <hrt/api.h>
#include "ga_acb_api.h"

typedef struct
{
  	//0 - ACB_CMD_FIFO_TAIL
	unsigned int cmd_fifo_tail		:32;	// Place holder, used for receiving commands.
	
	//1 - ACB_CTRL
	unsigned int AccInSelect		:1;	// 0 = ISP, 1 = Acc
	unsigned int AccOutSelect		:1;	// 0 = ISP, 1 = Acc
	unsigned int IgnoreLineNum		:1; // When set, Ack will be sent only when Eof arrives.
    unsigned int AcbPixRepEn                :1; // when set, ACB pixel replication is enabled
    unsigned int Res0                       :4;
    unsigned int AcbPixPairRepNumber        :8; //number of pixel pairs to pad at the end of each line
    unsigned int ForkAcbOutput              :1; // when set, ACB pixel replication is enabled
	unsigned int Res1			    :15;

	//2 - ACB_FRAME_SIZE
	unsigned int FrameWidth			:16;
	unsigned int FrameHeight		:16;
	
	//3 - ACB_ACC_SCALE
	unsigned int Scale_Mult			:4;
	unsigned int Scale_NF			:4;
	unsigned int Res2			:24;
}ACB_Control_Regs;

typedef union { 
    ACB_Control_Regs	ControlBits; 
    unsigned int	Dword[4] ; 
}ACB_Control_UNION; 


typedef struct
{
	unsigned int CmdID			:5;
	unsigned int Res0			:3;
	unsigned int CtrlID			:3;
	unsigned int Res1			:21;
	
}ACB_Init_Cmd;

typedef struct
{
	unsigned int CmdID			:5;
	unsigned int Res0			:3;
	unsigned int CtrlID			:3;
	unsigned int Res1			:1;
	unsigned int ConfigSet			:4;
	unsigned int NumLines			:16;
}ACB_ProcNLines_Cmd;

typedef union { 
    ACB_Init_Cmd		InitCmdBits;
    ACB_ProcNLines_Cmd		ProcNLinesBits;
    unsigned int		Dword; 
}ACB_Commands_UNION;



typedef struct
{
	unsigned int CmdID			:1;
	unsigned int Res			:31;
	
}ACB_Acknowledge;

typedef union { 
    ACB_Acknowledge	AckBits;
    unsigned int	Dword; 
}ACB_Ack_UNION;







#endif /* _GA_ACB_H */
