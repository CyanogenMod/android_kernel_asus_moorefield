#ifndef _GA_ACB_API_H
#define _GA_ACB_API_H

#include <hrt/api.h>
#include "ga_acb_ccbcr.h"

#define ACC_INIT_CMD_ID			0x0010
#define ACC_PROC_N_LINES_CMD_ID		0x0001


// Standard device API macros
#define HRT_GA_ACB_set_register(ACB_id, addr, val) \
  vied_subsystem_store_32(psys0, ACB_id + addr, (val))

#define HRT_GA_ACB_get_register(ACB_id, addr) \
  vied_subsystem_store_32(psys0, ACB_id + addr)




// More target focused APIs
#define HRT_GA_ACB_Init(ACB_id, Ctrl_Id) \
  HRT_GA_ACB_set_register(ACB_id, GA_ACB_CMD_FIFO_TAIL_ADDR, (Ctrl_Id << 8) | ACC_INIT_CMD_ID);	//sending init command

#define HRT_GA_ACB_Proc_N_Lines(ACB_id, Ctrl_Id, N) \
  HRT_GA_ACB_set_register(ACB_id, GA_ACB_CMD_FIFO_TAIL_ADDR, (((N) << 16) | ((Ctrl_Id) << 8) | ACC_PROC_N_LINES_CMD_ID));

#define HRT_GA_ACB_CfgSet_and_Proc_N_Lines(ACB_id, Ctrl_Id, CfgSet, N) \
  HRT_GA_ACB_set_register(ACB_id, GA_ACB_CMD_FIFO_TAIL_ADDR, (((N) << 16) | ((CfgSet) << 12) |((Ctrl_Id) << 8) | ACC_PROC_N_LINES_CMD_ID));

#define HRT_GA_ACB_Set_Routing(ACB_id, IsInputAcc, IsOutputAcc) \
  HRT_GA_ACB_set_register(ACB_id, GA_ACB_BASE_CTRL_ADDR, ((IsOutputAcc) << 1) | (IsInputAcc));

#define HRT_GA_ACB_Set_Routing_Ack_Upon_Eof(ACB_id, IsInputAcc, IsOutputAcc) \
  HRT_GA_ACB_set_register(ACB_id, GA_ACB_BASE_CTRL_ADDR, (1 << 2) | ((IsOutputAcc) << 1) | (IsInputAcc));

#define HRT_GA_ACB_Set_Routing_and_Replication(ACB_id, IsInputAcc, IsOutputAcc, RepNum) \
  HRT_GA_ACB_set_register(ACB_id, GA_ACB_BASE_CTRL_ADDR, ((RepNum) << 8) | (1 << 3) | ((IsOutputAcc) << 1) | (IsInputAcc));

#define HRT_GA_ACB_Set_Routing_and_Replication_Ack_Upon_Eof(ACB_id, IsInputAcc, IsOutputAcc, RepNum) \
  HRT_GA_ACB_set_register(ACB_id, GA_ACB_BASE_CTRL_ADDR, ((RepNum) << 8) | (1 << 3) | (1 << 2) | ((IsOutputAcc) << 1) | (IsInputAcc));

#define HRT_GA_ACB_Set_Frame_Size(ACB_id, Width, Height) \
  HRT_GA_ACB_set_register(ACB_id, GA_ACB_INPUT_FRAME_SIZE_ADDR, ((Height) << 16) | (Width));

#define HRT_GA_ACB_Set_Scale(ACB_id, Mult, NF) \
  HRT_GA_ACB_set_register(ACB_id, GA_ACB_SCALE_ADDR, (((NF) << 4) | (Mult)));

#define HRT_GA_ACB_Get_Frame_Width(ACB_id) \
  HRT_GA_ACB_get_register(ACB_id, GA_ACB_INPUT_FRAME_SIZE_ADDR);

#define HRT_GA_ACB_Get_Frame_Height(ACB_id) \
  (HRT_GA_ACB_get_register(ACB_id, GA_ACB_INPUT_FRAME_SIZE_ADDR) >> 16);

#define HRT_GA_ACB_Set_Basic_Ctrl(ACB_id, IsInputAcc, IsOutputAcc, IgnoreLineNum, AcbPixRepEn, AcbPixPairRepNumber, ForkAcbOutput) \
  HRT_GA_ACB_set_register(ACB_id, GA_ACB_BASE_CTRL_ADDR, ((ForkAcbOutput) << 16) | ((AcbPixPairRepNumber) << 8) | ((AcbPixRepEn) << 3) | ((IgnoreLineNum) << 2) | ((IsOutputAcc) << 1) | (IsInputAcc));

#endif //_GA_ACB_API_H
