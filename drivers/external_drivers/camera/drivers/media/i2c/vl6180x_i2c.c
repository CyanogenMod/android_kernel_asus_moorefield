
/*******************************************************************************
Copyright © 2014, STMicroelectronics International N.V.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of STMicroelectronics nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED. 
IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/
/*
 * $Date: 2015-01-08 14:30:24 +0100 (Thu, 08 Jan 2015) $
 * $Revision: 2039 $
 */

/**
 * @file vl6180x_i2c.c
 *
 * Copyright (C) 2014 ST MicroElectronics
 *
 * provide variable word size byte/Word/dword VL6180x register access via i2c
 *
 */

#include "vl6180x_i2c.h"
#include "vl6180_lf_api.h"

int VL6180x_WrByte(VL6180xDev_t dev, uint32_t index, uint8_t data){
    int  status=1;
   
    status = ASUS_VL6180x_WrByte(index, data);

    return status;
}

int VL6180x_WrWord(VL6180xDev_t dev, uint32_t index, uint16_t data){
    int  status=1;

    status = ASUS_VL6180x_WrWord(index, data);

    return status;
}

int VL6180x_WrDWord(VL6180xDev_t dev, uint32_t index, uint32_t data){
    int  status=1;

    status = ASUS_VL6180x_WrDWord(index, data);    
	
    return status;
}

int VL6180x_UpdateByte(VL6180xDev_t dev, uint32_t index, uint8_t AndData, uint8_t OrData){
    int  status=1;

    status = ASUS_VL6180x_UpdateByte(index, AndData, OrData);

    return status;
}

int VL6180x_RdByte(VL6180xDev_t dev, uint32_t index, uint8_t *data){
    int  status=1;

    status = ASUS_VL6180x_RdByte(index, data);

    return status;
}

int VL6180x_RdWord(VL6180xDev_t dev, uint32_t index, uint16_t *data){
    int  status=1;

    status = ASUS_VL6180x_RdWord(index, data);

    return status;
}

int  VL6180x_RdDWord(VL6180xDev_t dev, uint32_t index, uint32_t *data){
    int  status=1;

    status = ASUS_VL6180x_RdDWord(index, data, 4);
  
    return status;
}
