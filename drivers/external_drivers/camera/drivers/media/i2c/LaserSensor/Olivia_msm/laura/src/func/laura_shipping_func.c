/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-07
*
*/

#include "laura_shipping_func.h"
#include "show_log.h"
#include <linux/timer.h>
#include <linux/delay.h>

/* Log count */
static int read_range_log_count = 0; // Read range log count

/* Module id */
static bool module_id_flag = false;
uint16_t module_id[34];
uint16_t f0_data[22];
uint16_t f0_data_test[22];
//uint8_t module_id_2[34];


extern int ErrCode;


static uint16_t debug_raw_range = 0;
static uint16_t debug_raw_confidence = 0;

static void init_debug_raw_data(void){
		debug_raw_range = 0;
		debug_raw_confidence = 0;
}

uint16_t get_debug_raw_range(void){
	return debug_raw_range;
}

uint16_t get_debug_raw_confidence(void){
	return debug_raw_confidence;
}


/** @brief Swap high and low of the data (e.g 0x1234 => 0x3412)
*
*	@param register_data the data which will be swap
*
*/
void swap_data(uint16_t* register_data){
	*register_data = ((*register_data >> 8) | ((*register_data & 0xff) << 8)) ;
}


/** @brief Mailbox: create calibration data
*		  This mailbox command is used to retrieve data to be used for the computation of calibration parameters.
*		  This is a singleentry MBX command with MBX message response with Msg_len = 6
*
*	@param dev_t the laser focus controller
*	@param cal_data the calibration ouput data
*
*/
int Mailbox_Command(struct msm_laser_focus_ctrl_t *dev_t, int16_t cal_data[]){
	int status = 0, msg_len = 0, M2H_Msg_Len = 0;
	uint16_t i2c_read_data, i2c_read_data2;
	struct timeval start;//, now;

	start = get_current_time();
	while(1){
		status = CCI_I2C_RdWord(dev_t, ICSR, &i2c_read_data);
		if (status < 0){
			return status;
       	}
		
		if((i2c_read_data&NEW_DATA_IN_MBX) == 0x00){
			break;
		}

		/* Busy pending MCPU Msg */
		status = CCI_I2C_RdWord(dev_t, M2H_MBX, &i2c_read_data2);
		if (status < 0){
			return status;
       	}
		LOG_Handler(LOG_ERR, "%s: register(0x00, 0x10): (0x%x, 0x%x)\n", __func__,  i2c_read_data, i2c_read_data2);
#if 0
		/* Check if time out */
		now = get_current_time();
              if(is_timeout(start,now,TIMEOUT_VAL)){
			LOG_Handler(LOG_ERR, "%s: Verify ICSR(2:1) time out - register(0x10): 0x%x\n", __func__, i2c_read_data);
                    	return -TIMEOUT_VAL;
              }
#endif
		//usleep(DEFAULT_DELAY_TIME);
	}

	status = CCI_I2C_WrWord(dev_t, H2M_MBX, 0x0004);
       if (status < 0){
               return status;
       }

	while(1){
		status = CCI_I2C_RdWord(dev_t, ICSR, &i2c_read_data);
		if (status < 0){
			return status;
       	}
		
		if((i2c_read_data&0x20) == 0x20){
			break;
		}

#if 0
		/* Check if time out */
		now = get_current_time();
              if(is_timeout(start,now,TIMEOUT_VAL)){
			LOG_Handler(LOG_ERR, "%s: Verify ICSR(2:1) time out - register(0x10): 0x%x\n", __func__, i2c_read_data);
                    	return -TIMEOUT_VAL;
              }
#endif
		//usleep(DEFAULT_DELAY_TIME);
	}


	status = CCI_I2C_RdWord(dev_t, M2H_MBX, &i2c_read_data);
	LOG_Handler(LOG_DBG, "%s: Verify M2H_MBX(1)  register(0x12): 0x%x\n", __func__, i2c_read_data);
	if (status < 0){
		return status;
       }
	M2H_Msg_Len = (i2c_read_data & CMD_LEN_MASK)>>8;
	LOG_Handler(LOG_DBG, "%s: Verify M2H_MBX(1) M2H_Msg_Len: %d\n", __func__, M2H_Msg_Len);

	if(((i2c_read_data&0xFF) == 0xCC) && (M2H_Msg_Len == CAL_MSG_LEN)){
		for(msg_len=0; msg_len<M2H_Msg_Len; msg_len++){
			start = get_current_time();
			while(1){
				status = CCI_I2C_RdWord(dev_t, ICSR, &i2c_read_data);
				if (status < 0){
					return status;
       			}
				//LOG_Handler(LOG_ERR, "%s: Verify ICSR(1)  register(0x00): 0x%x\n", __func__, i2c_read_data);
			
				if((i2c_read_data&NEW_DATA_IN_MBX)){
					break;
				}
#if 0
				/* Check if time out */
				now = get_current_time();
              		if(is_timeout(start,now,TIMEOUT_VAL)){
					LOG_Handler(LOG_ERR, "%s: Verify ICSR(1) time out - register(0x00): 0x%x\n", __func__, i2c_read_data);
                    			return -TIMEOUT_VAL;
              		}
#endif
		 		//usleep(DEFAULT_DELAY_TIME);
			}

       		status = CCI_I2C_RdWord(dev_t, M2H_MBX, &i2c_read_data);
			if (status < 0){
              		return status;
       		}

			/* Append to previosly saved data */
			cal_data[msg_len] = i2c_read_data;
			LOG_Handler(LOG_DBG, "%s: Calibration data[%d]: 0x%x\n", __func__, msg_len, cal_data[msg_len]);
		}
	}
	else{
		LOG_Handler(LOG_ERR, "%s: M2H_MBX(7:0): 0x%x, Msg_Len: %d\n", __func__, i2c_read_data&0xFF, M2H_Msg_Len);
		return -1;
	}
	return status;
}


int Check_Ready_via_ICSR(struct msm_laser_focus_ctrl_t *dev_t){
	int status =0;
	uint16_t i2c_read_data, i2c_read_data2;
	
	while(1){
		status = CCI_I2C_RdWord(dev_t, ICSR, &i2c_read_data);
		if (status < 0)
			return status;

		if((i2c_read_data&(NEW_DATA_IN_MBX|MCPU_HAVE_READ_MBX)) == GO_AHEAD)
			return status;

		/* Busy pending MCPU Msg */
		status = CCI_I2C_RdWord(dev_t, M2H_MBX, &i2c_read_data2);
		if (status < 0)
			return status;

		usleep_range(100,100);
		LOG_Handler(LOG_ERR, "%s: register(0x00, 0x10): (0x%x, 0x%x)\n", __func__,  i2c_read_data, i2c_read_data2);

	}
}

int Wait_for_Notification_from_ICSR(struct msm_laser_focus_ctrl_t *dev_t){
	int status =0;
	uint16_t i2c_read_data;

	while(1){
		status = CCI_I2C_RdWord(dev_t, ICSR, &i2c_read_data);
		if (status < 0 || (i2c_read_data&NEW_DATA_IN_MBX))
			return status;
	}
}

int Olivia_Mailbox_Command(struct msm_laser_focus_ctrl_t *dev_t, int16_t cal_data[]){
	int status = 0, msg_index = 0, M2H_Msg_Len = 0;
	uint16_t i2c_read_data;

	status = Check_Ready_via_ICSR(dev_t);
	//if(status !=0)

	//single entry message? -> Cmd_len=0
	status = CCI_I2C_WrWord(dev_t, H2M_MBX, GET_CALIBRATION);
       if (status < 0){
               return status;
       }

	status = Wait_for_Notification_from_ICSR(dev_t);
	//if(status !=0)


	status = CCI_I2C_RdWord(dev_t, M2H_MBX, &i2c_read_data);
	if (status < 0){
		return status;
       }
	
	M2H_Msg_Len = (i2c_read_data & CMD_LEN_MASK)>>8;
	if(M2H_Msg_Len != CAL_MSG_LEN)
		LOG_Handler(LOG_ERR,"Message length is not in expect\n");
	
	if((i2c_read_data&MESSAGE_ID_MASK) == 0xCC) {
		for(msg_index=0; msg_index<CAL_MSG_LEN; msg_index++){
			status = Wait_for_Notification_from_ICSR(dev_t);
			//if(status !=0)
       		status = CCI_I2C_RdWord(dev_t, M2H_MBX, &i2c_read_data);
			if (status < 0){
              		return status;
       		}
			/* Append to previosly saved data */
			cal_data[msg_index] = i2c_read_data;
			LOG_Handler(LOG_DBG, "%s: Calibration data[%d]: 0x%x\n", __func__, msg_index, cal_data[msg_index]);
		}
	}
	else{
		LOG_Handler(LOG_ERR, "%s: M2H_MBX(7:0): 0x%x, Msg_Len: %d\n", __func__, i2c_read_data&0xFF, M2H_Msg_Len);
		return -1;
	}
	return status;
}


/** @brief Load calibration data 
*
*	@param dev_t the laser focus controller
*
*/
int Laura_device_Load_Calibration_Value(struct msm_laser_focus_ctrl_t *dev_t){
	int status = 0;
	uint16_t indirect_addr, data[SIZE_OF_LAURA_CALIBRATION_DATA];
#if DEBUG_LOG_FLAG
	int i = 0;
	uint16_t data_verify;
#endif
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);


	/* Read Calibration data, addr is swapped */
	indirect_addr = 0x10C0;

	status = Larua_Read_Calibration_Data_From_File(data, SIZE_OF_LAURA_CALIBRATION_DATA);
	if(status < 0){
		LOG_Handler(LOG_ERR, "%s: Load calibration fail!!\n", __func__);
		return status;
	}

	Laura_device_indirect_addr_write(dev_t, 0x18, 0x19, indirect_addr, I2C_DATA_PORT, data, 10);

	/* Check patch memory write */
	CCI_I2C_WrByte(dev_t, 0x18, 0x10);
	CCI_I2C_WrByte(dev_t, 0x19, 0xC0);
#if DEBUG_LOG_FLAG
	for(i = 0; i < 20; i++){
		CCI_I2C_RdByte(dev_t, I2C_DATA_PORT, &data_verify);
		LOG_Handler(LOG_DBG, "%s: 0x1A: 0x%x\n", __func__, data_verify);
	}
#endif
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	
	return status;
}


int Olivia_device_Load_Calibration_Value(struct msm_laser_focus_ctrl_t *dev_t){
	int status = 0;
	uint16_t indirect_addr, data[SIZE_OF_OLIVIA_CALIBRATION_DATA];
#if DEBUG_LOG_FLAG
	int i = 0;
	uint16_t data_verify;
#endif
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);


	/* Read Calibration data, addr is swapped */
	indirect_addr = 0x10C0;

	status = Larua_Read_Calibration_Data_From_File(data, SIZE_OF_OLIVIA_CALIBRATION_DATA);
	if(status < 0){
		LOG_Handler(LOG_ERR, "%s: Load calibration fail!!\n", __func__);
		return status;
	}

	Laura_device_indirect_addr_write(dev_t, 0x18, 0x19, indirect_addr, I2C_DATA_PORT, data, SIZE_OF_OLIVIA_CALIBRATION_DATA);

	/* Check patch memory write */
	CCI_I2C_WrByte(dev_t, 0x18, 0x10);
	CCI_I2C_WrByte(dev_t, 0x19, 0xC0);
#if DEBUG_LOG_FLAG
	for(i = 0; i < 2*SIZE_OF_OLIVIA_CALIBRATION_DATA; i++){
		CCI_I2C_RdByte(dev_t, I2C_DATA_PORT, &data_verify);
		LOG_Handler(LOG_DBG, "%s: 0x1A: 0x%x\n", __func__, data_verify);
	}
#endif
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	
	return status;
}

int Laura_Read_Calibration_Value_From_File(struct seq_file *vfile, uint16_t *cal_data){
	int status = 0, i = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	status = Larua_Read_Calibration_Data_From_File(cal_data, SIZE_OF_LAURA_CALIBRATION_DATA);
        if(status < 0){
                LOG_Handler(LOG_ERR, "%s: Load calibration fail!!\n", __func__);
		if(vfile!=NULL){
			seq_printf(vfile,"No calibration data!!\n");
		}
                return status;
        }

	for(i = 0; i < SIZE_OF_LAURA_CALIBRATION_DATA; i++){
                swap_data(cal_data+i);
        }

	LOG_Handler(LOG_CDBG,"Cal data: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x\n",
		cal_data[0], cal_data[1], cal_data[2], cal_data[3], cal_data[4],
		cal_data[5], cal_data[6], cal_data[7], cal_data[8], cal_data[9]);	

	if(vfile!=NULL){
		seq_printf(vfile,"%04x %04x %04x %04x %04x %04x %04x %04x %04x %04x\n",
               		cal_data[0], cal_data[1], cal_data[2], cal_data[3], cal_data[4],
                	cal_data[5], cal_data[6], cal_data[7], cal_data[8], cal_data[9]);

	}

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return status;
}

int Olivia_Read_Calibration_Value_From_File(struct seq_file *vfile, uint16_t *cal_data){
	int status = 0, i = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	status = Larua_Read_Calibration_Data_From_File(cal_data, SIZE_OF_OLIVIA_CALIBRATION_DATA);
        if(status < 0){
                LOG_Handler(LOG_ERR, "%s: Load calibration fail!!\n", __func__);
		if(vfile!=NULL){
			seq_printf(vfile,"No calibration data!!\n");
		}
                return status;
        }

	for(i = 0; i < SIZE_OF_OLIVIA_CALIBRATION_DATA; i++){
                swap_data(cal_data+i);
        }

	LOG_Handler(LOG_CDBG,"Cal data: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x\n",
		cal_data[0], cal_data[1], cal_data[2], cal_data[3], cal_data[4],cal_data[5], cal_data[6], cal_data[7], cal_data[8],
		cal_data[9],cal_data[10], cal_data[11], cal_data[12], cal_data[13], cal_data[14],cal_data[15], cal_data[16]);

	if(vfile!=NULL){
		seq_printf(vfile,"%04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x\n",
			cal_data[0], cal_data[1], cal_data[2], cal_data[3], cal_data[4],cal_data[5], cal_data[6], cal_data[7], cal_data[8],
			cal_data[9],cal_data[10], cal_data[11], cal_data[12], cal_data[13], cal_data[14],cal_data[15], cal_data[16]);

	}

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return status;
}

/** @brief laura read range 
*	
*	@param dev_t the laser focus controller
*
*/
#if 0
uint16_t Laura_device_read_range(struct msm_laser_focus_ctrl_t *dev_t)
{
	uint16_t RawRange = 0, i2c_read_data = 0;
	int status;
	struct timeval start, now;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	start = get_current_time();

	/* read count +1  */
	read_range_log_count++;

	/* Verify status is MCPU on */
	while(1){
		status = CCI_I2C_RdWord(dev_t, DEVICE_STATUS, &i2c_read_data);
		if (status < 0){
       		return status;
       	}

		//include MCPU_ON
		if(i2c_read_data == STATUS_MEASURE_ON){
			break;
		}

		/* Check if time out */
		now = get_current_time();
              if(is_timeout(start,now,5)){
			LOG_Handler(LOG_ERR, "%s: Verify MCPU status time out - register(0x06): 0x%x\n", __func__, i2c_read_data);
                  	return OUT_OF_RANGE;
              }

		LOG_Handler(LOG_DBG, "%s: register(0x06):0x%x!!\n", __func__,i2c_read_data);		
	}
	
#if READ_RETRY_FLAG
	while(1){
#endif
		/* Trigger single measure */
        	status = CCI_I2C_WrWord(dev_t, COMMAND_REGISTER, (SINGLE_MEASURE|VALIDATE_CMD));
        	if (status < 0){
              	return status;
        	}

		/* Wait until data ready */
       	while(1){
			status = CCI_I2C_RdWord(dev_t, ICSR, &i2c_read_data);
			if (status < 0){
       			return status;
       		}

			if(i2c_read_data & NEW_DATA_IN_RESULT_REG){
				break;
			}

			/* Check if time out */
			now = get_current_time();
             		if(is_timeout(start,now,80)){
				LOG_Handler(LOG_ERR, "%s: Wait data ready time out - register(0x00): 0x%x\n", __func__, i2c_read_data);
                    		return OUT_OF_RANGE;
             		}

			/* Delay: waitting laser sensor sample ready */
			usleep(READ_DELAY_TIME);
       	}

		/* Read distance */
       	status = CCI_I2C_RdWord(dev_t, RESULT_REGISTER, &RawRange);
		if (status < 0){
              	return status;
       	}
	
		/* Check if target is out of field of view */
		if((RawRange&ERROR_CODE_MASK)==NO_ERROR && (RawRange&VALID_DATA)){
#if READ_OUTPUT_LIMIT_FLAG == 0
			/* Get real range */
                        LOG_Handler(LOG_DBG, "%s: Non-shift Read range:%d\n", __func__, RawRange);
#endif
			RawRange = (RawRange&DISTANCE_MASK)>>2;
#if READ_OUTPUT_LIMIT_FLAG == 0	
			LOG_Handler(LOG_CDBG, "%s: Read range:%d\n", __func__, RawRange);
#endif
#if READ_OUTPUT_LIMIT_FLAG			
			/* Display distance */
			if(read_range_log_count >= LOG_SAMPLE_RATE){
				read_range_log_count = 0;
				LOG_Handler(LOG_CDBG, "%s: Read range:%d\n", __func__, RawRange);
			}
#endif
#if READ_RETRY_FLAG
			break;
#endif
		}
       	else {
	   	  	if((RawRange&0x2000)==0x2000){
		  		LOG_Handler(LOG_ERR, "%s: The target is near of field of view!!\n", __func__);
	   	  	}else if((RawRange&0x4000)==0x4000){
				LOG_Handler(LOG_ERR, "%s: The target is out of field of view!!\n", __func__);
		  	}else{
				LOG_Handler(LOG_ERR, "%s: Read range fail!!\n", __func__);
		  	}
#if READ_RETRY_FLAG == 0
			return OUT_OF_RANGE;
#endif
       	}
#if READ_RETRY_FLAG
		/* Check if time out */
		now = get_current_time();
             	if(is_timeout(start,now,TIMEOUT_VAL)){
			LOG_Handler(LOG_ERR, "%s: Read range time out!!\n", __func__);
              	return OUT_OF_RANGE;
             	}
	}
#endif

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return RawRange;
}
#endif

#if 0
int16_t Laura_device_read_range(struct msm_laser_focus_ctrl_t *dev_t, int *errStatus)
{
	uint16_t RawRange = 0, i2c_read_data = 0, realRange = 0, RawConfidence = 0, confidence_level = 0;
	int status;
	struct timeval start, now;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	start = get_current_time();

	/* Init debug raw data */
	init_debug_raw_data();

	/* read count +1  */
	read_range_log_count++;

	/* Verify status is MCPU on */
	while(1){
		status = CCI_I2C_RdWord(dev_t, 0x06, &i2c_read_data);
		if (status < 0){
       		return status;
       	}
		i2c_read_data = swap_data(i2c_read_data);

		if(i2c_read_data == STATUS_MEASURE_ON){
			break;
		}

		/* Check if time out */
		now = get_current_time();
              if(is_timeout(start,now,5)){
			LOG_Handler(LOG_ERR, "%s: Verify MCPU status time out - register(0x06): 0x%x\n", __func__, i2c_read_data);
                  	return OUT_OF_RANGE;
              }

		LOG_Handler(LOG_DBG, "%s: register(0x06):0x%x!!\n", __func__,i2c_read_data);
		
	}
	
#if READ_RETRY_FLAG
	while(1){
#endif
		/* Trigger single measure */
        	status = CCI_I2C_WrWord(dev_t, 0x04, swap_data(0x0081));
        	if (status < 0){
              	return status;
        	}

		/* Wait until data ready */
       	while(1){
			status = CCI_I2C_RdWord(dev_t, 0x00, &i2c_read_data);
			if (status < 0){
       			return status;
       		}
			i2c_read_data = swap_data(i2c_read_data);

			if((int)(i2c_read_data&0x10) == 0x10){
				break;
			}

			/* Check if time out */
			now = get_current_time();
             		if(is_timeout(start,now,80)){
				LOG_Handler(LOG_ERR, "%s: Wait data ready time out - register(0x00): 0x%x\n", __func__, i2c_read_data);
                    		return OUT_OF_RANGE;
             		}

			/* Delay: waitting laser sensor sample ready */
			usleep(READ_DELAY_TIME);
       	}

		/* Read distance */
       	status = CCI_I2C_RdWord(dev_t, 0x08, &RawRange);
		if (status < 0){
              	return status;
       	}
		RawRange = swap_data(RawRange);
		debug_raw_range = RawRange;
		LOG_Handler(LOG_DBG, "%s: [SHOW_LOG] Non-shift Read range:%d\n", __func__, RawRange);

		/* Check if target is valid */
		/* RawRange bit 15 is 1 */
		if((RawRange&0x8000)==0x8000){	
			/* RawRange bit 13,14 are 00 */
			if((RawRange&0x6000)==0x00){
				
				/* Get real range success */
				realRange = (RawRange&0x1fff)>>2;
				
				/* [Issue solution] Inf report smale value : 
				 * When the distance measured < 250mm and confidence level < 15, the result is invalid */
				
				/* Read result confidence level */
				status = CCI_I2C_RdWord(dev_t, 0x0A, &RawConfidence);
				if (status < 0){
					return status;
				}
				RawConfidence = swap_data(RawConfidence);
				debug_raw_confidence = RawConfidence;
				confidence_level = (RawConfidence&0x7fff)>>4;
				LOG_Handler(LOG_DBG,"%s: [SHOW_LOG] confidence level is: %d (Raw data:%d)\n", __func__, confidence_level, RawConfidence);
				
				/* distance <= 400 and confidence < 11*400/distance 
				* , distance > 400 and confidence < 11
				* or distance = 0,
				* then ignored this distance measurement.
				* */
                if((realRange <= 400 && confidence_level < (11*400/realRange))
				  || (realRange > 400 && confidence_level < 11)
				  || realRange == 0){
					LOG_Handler(LOG_DBG, "%s: Read range fail (range,confidence level): (%d,%d)\n", __func__,realRange,confidence_level);
					*errStatus = 0;
					realRange = OUT_OF_RANGE;
                }
				else{
					*errStatus = 0;
				}

#if READ_OUTPUT_LIMIT_FLAG
				/* Display distance */
				if(read_range_log_count >= LOG_SAMPLE_RATE){
					read_range_log_count = 0;
					LOG_Handler(LOG_CDBG, "%s: Read range:%d\n", __func__, realRange);
				}
#endif
			}
			/* RawRange bit 13,14 are 11 */
			else if((RawRange&0x6000)==0x6000){
				LOG_Handler(LOG_DBG, "%s: Read range bit 13, 14, 15 are '1'(%d)\n", __func__,RawRange);
				*errStatus = RANGE_ERR_NOT_ADAPT;
				realRange = OUT_OF_RANGE;
			}
			/* RawRange bit 13 is 1, but bit 14 is 0 */
			else if((RawRange&0x2000)==0x2000){
		  		LOG_Handler(LOG_DBG, "%s: The target is near of field of view(%d)!!\n", __func__,RawRange);
				*errStatus = 0;
				realRange = 0;
	   	  	}
	   	  	/* RawRange bit 13 is 0, but bit 14 is 1 */
	   	  	else if((RawRange&0x4000)==0x4000){
				LOG_Handler(LOG_DBG, "%s: The target is out of field of view(%d)!!\n", __func__,RawRange);
				*errStatus = 0;
				realRange = OUT_OF_RANGE;
		  	}
		  	else{
				LOG_Handler(LOG_DBG, "%s: Read range fail(%d)!!\n", __func__,RawRange);
				*errStatus = 0;
				realRange = OUT_OF_RANGE;
		  	}
#if READ_RETRY_FLAG
			break;
#endif
		}
		/* RawRange bit 15 is 0 */
       	else {
			/* RawRange bit 13 is 1, but bit 14 is 0 */
	   	  	if((RawRange&0x2000)==0x2000){
		  		LOG_Handler(LOG_ERR, "%s: The target is near of field of view(%d)!!\n", __func__,RawRange);
				*errStatus = RANGE_ERR;
				realRange = 0;
	   	  	}
	   	  	/* RawRange bit 13 is 0, but bit 14 is 1 */
	   	  	else if((RawRange&0x4000)==0x4000){
				LOG_Handler(LOG_ERR, "%s: The target is out of field of view(%d)!!\n", __func__,RawRange);
				*errStatus = RANGE_ERR;
				realRange = OUT_OF_RANGE;
		  	}
		  	/* RawRange bit 13,14 are 11 or 00 */
		  	else{
				LOG_Handler(LOG_ERR, "%s: Read range fail(%d)!!\n", __func__,RawRange);
				*errStatus = RANGE_ERR;
				realRange = OUT_OF_RANGE;
		  	}
#if READ_RETRY_FLAG == 0
			//return OUT_OF_RANGE;
#endif
       	}
#if READ_RETRY_FLAG
		/* Check if time out */
		now = get_current_time();
        if(is_timeout(start,now,TIMEOUT_VAL)){
			LOG_Handler(LOG_DBG, "%s: Read range time out!!\n", __func__);
        	return OUT_OF_RANGE;
        }
		usleep(1);
	}
#endif

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return (int)realRange;
}
#endif

#define	MCPU_ON_TIME_OUT_ms		5
#define	MEASURE_TIME_OUT_ms		80

int Verify_MCPU_On_by_Time(struct msm_laser_focus_ctrl_t *dev_t){
	struct timeval start,now;
	uint16_t i2c_read_data = 0;
	int status=0;	
	O_get_current_time(&start);

	while(1){
		status = CCI_I2C_RdWord(dev_t, DEVICE_STATUS, &i2c_read_data);
		if (status < 0)
       		return status;
       		
		//include MCPU_ON
		if(i2c_read_data == STATUS_MEASURE_ON){
			LOG_Handler(LOG_DBG, "%s: in MCPU_ON\n", __func__);			
			break;
		}
		 O_get_current_time(&now);
              if(is_timeout(start,now,MCPU_ON_TIME_OUT_ms)){
			LOG_Handler(LOG_DBG, "%s: fail (time out)\n", __func__);						  	
                  	return OUT_OF_RANGE;      
              }
	}	
	return status;
}

int Verify_Range_Data_Ready(struct msm_laser_focus_ctrl_t *dev_t){
	struct timeval start,now;
	uint16_t i2c_read_data = 0;
	int status=0;	
	O_get_current_time(&start);
	
       while(1){
		status = CCI_I2C_RdWord(dev_t, ICSR, &i2c_read_data);
		if (status < 0)
       		return status;
		
		if(i2c_read_data & NEW_DATA_IN_RESULT_REG){
			LOG_Handler(LOG_DBG, "%s: range data ready\n", __func__);	
			break;
		}
		now = get_current_time();
             	if(is_timeout(start,now,MEASURE_TIME_OUT_ms)){
			LOG_Handler(LOG_DBG, "%s: fail (time out)\n", __func__);						  		
                    return OUT_OF_RANGE;
             	}
		/* Delay: waitting laser sensor sample ready */
		usleep_range(READ_DELAY_TIME,READ_DELAY_TIME);
       }
	return status;
	

}

int	Read_Range_Data(struct msm_laser_focus_ctrl_t *dev_t){
	uint16_t RawRange = 0, Range = 0, error_status =0;
	uint16_t RawConfidence = 0, confidence_level =0;
	int status;
	init_debug_raw_data();
	
      	status = CCI_I2C_RdWord(dev_t, RESULT_REGISTER, &RawRange);
	if (status < 0)
             	return status;
       	
	Range = (RawRange&DISTANCE_MASK)>>2;
	LOG_Handler(LOG_DBG, "%s:%d\n",__func__,Range);

	error_status = RawRange&ERROR_CODE_MASK;
	LOG_Handler(LOG_DBG, "%s:Register 0x08 : 0x%x\n",__func__,(RawRange&LASER_BIT_MASK(15,13))>>13);
	if(RawRange&VALID_DATA){
			
		if((error_status==NO_ERROR)){
			Range = (RawRange&DISTANCE_MASK)>>2;

			/* 	2016.01.05 Kun_Wang +++
				CE Gavin ask : if range==2047(mm)
								output range=9999*/
			if(Range==2047)
				Range=OUT_OF_RANGE;
			/*	2016.01.05 Kun_Wang ===*/

			ErrCode = 0;
			CCI_I2C_RdWord(dev_t, 0x0A, &RawConfidence);
			LOG_Handler(LOG_DBG, "%s:Register 0x0A : 0x%x\n",__func__,RawConfidence);
			if (status < 0)
            		 	return status;
       
			debug_raw_confidence = RawConfidence;
			confidence_level = (RawConfidence&0x7ff0)>>4;
			//LOG_Handler(LOG_DBG,"%s: confidence level is: %d (Raw data:%d)\n", __func__, confidence_level, RawConfidence);
		
                	if((Range <= 400 && confidence_level < (11*400/Range))
				  || (Range > 400 && confidence_level < 11)
				  || Range == 0){
				LOG_Handler(LOG_DBG, "%s: Take range as far field (raw range,confidence): (%d,%d)\n", __func__,Range,confidence_level);			
				Range = OUT_OF_RANGE;
               	}
					
		}
		else if(error_status==NEAR_FIELD){
			ErrCode = RANGE_ADAPT;
			Range = 0;
			LOG_Handler(LOG_ERR, "%s: Target is too near\n", __func__);
		}
		else if(error_status==FAR_FIELD){
			ErrCode = RANGE_ADAPT;
			Range =	OUT_OF_RANGE;
			LOG_Handler(LOG_ERR, "%s: Target is too far\n", __func__);
		}
		else{
			ErrCode = RANGE_ERR_NOT_ADAPT;
			Range =	OUT_OF_RANGE;
			LOG_Handler(LOG_ERR, "%s: General error for range reading\n", __func__);
		}
	} 
	else {
		if(error_status==NEAR_FIELD)
			Range = 0;
		else
			Range = OUT_OF_RANGE;
		
		ErrCode = RANGE_ERR_NOT_ADAPT;
	}
	return Range;
	

}

int Olivia_device_read_range(struct msm_laser_focus_ctrl_t *dev_t)
{
	int status, Range=0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	read_range_log_count++;

	status = Verify_MCPU_On_by_Time(dev_t);
	if(status != 0) goto read_err;

	/* Trigger single measure */
       status = CCI_I2C_WrWord(dev_t, COMMAND_REGISTER, (SINGLE_MEASURE|VALIDATE_CMD));
       if (status < 0){
            	return status;
       }
	status = Verify_Range_Data_Ready(dev_t);
	if(status != 0) goto read_err;			

	Range = Read_Range_Data(dev_t);
	if(Range < 0){
		status = Range;
		goto read_err;
	}
	
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return Range;

read_err:
	LOG_Handler(LOG_ERR, "%s: Exit with Error: %d\n", __func__, status);	
	return status;

}



/** @brief MCPU Contorller
*
*	@param dev_t the laser focus controller
*	@param mode the MCPU go to status
*
*/
int Laura_MCPU_Controller(struct msm_laser_focus_ctrl_t *dev_t, int mode){
	int status = 0;
#if DEBUG_LOG_FLAG
	uint16_t i2c_read_data = 0;
#endif


	switch(mode){
		case MCPU_ON:
			/* Enable MCPU to run coming out of standby */
			LOG_Handler(LOG_DBG, "%s: MCPU ON procdure\n", __func__);
			status = CCI_I2C_WrWord(dev_t, PMU_CONFIG, (PATCH_MEM_EN|MCPU_INIT_STATE));
       		if (status < 0){
             			return status;
      			}
			
			/* Wake up MCPU to ON mode */
			status = CCI_I2C_WrWord(dev_t, COMMAND_REGISTER, (MCPU_TO_ON|VALIDATE_CMD));
			if (status < 0){
       			return status;
       		}
			break;
		case MCPU_OFF:
			/* Enable patch memory */
			LOG_Handler(LOG_DBG, "%s: MCPU OFF procdure\n", __func__);
			status = CCI_I2C_WrWord(dev_t, PMU_CONFIG, (PATCH_MEM_EN|PATCH_CODE_LD_EN));
       		if (status < 0){
             			return status;
      			}
			
			/* Go MCPU to OFF status */
			status = CCI_I2C_WrWord(dev_t, COMMAND_REGISTER, (GO_MCPU_OFF|VALIDATE_CMD));
        		if (status < 0){
              		return status;
        		}
			break;
		case MCPU_STANDBY:
			/* Change MCUP to standby mode */
			LOG_Handler(LOG_DBG, "%s: MCPU STANDBY procdure\n", __func__);
			status = CCI_I2C_WrWord(dev_t, COMMAND_REGISTER, (GO_STANDBY|VALIDATE_CMD));
       		if (status < 0){
       			return status;
     			}
			break;
		default:
			LOG_Handler(LOG_ERR, "%s MCPU mode invalid (%d)\n", __func__, mode);
			break;
	}

	/* wait hardware booting(least 500us) */
	usleep_range(MCPU_DELAY_TIME, MCPU_DELAY_TIME);
	
#if DEBUG_LOG_FLAG
	/* Verify MCPU status */
	status = CCI_I2C_RdWord(dev_t, DEVICE_STATUS, &i2c_read_data);
	if (status < 0){
             	return status;
       }
	LOG_Handler(LOG_DBG, "%s: register(0x06):0x%x!!\n", __func__,i2c_read_data);
#endif

	return status;
}

/** @brief Initialize Laura tof configure
*
*	@param dev_t the laser focus controller
*	@param config the configuration param
*
*/
int Laura_device_UpscaleRegInit(struct msm_laser_focus_ctrl_t *dev_t, uint16_t *config)
{
	int status = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

#if DEBUG_LOG_FLAG
	LOG_Handler(LOG_DBG, "%s: config:(0x%x,0x%x,0x%x,0x%x,0x%x,0x%x)\n", __func__,
		config[0], config[1], config[2], config[3], config[4], config[5]);
#endif

	/* Drive INT_PAD high */
	status = CCI_I2C_WrWord(dev_t, ICSR, PAD_INT_MODE);
       if (status < 0){
               return status;
       }

	/* Change the default VCSEL threshold and VCSEL peak */
       status = CCI_I2C_WrWord(dev_t, 0x0C, config[0]);
       if (status < 0){
               return status;
       }
	   
       status = CCI_I2C_WrWord(dev_t, 0x0E, config[1]);
       if (status < 0){
               return status;
       }
	   
       status = CCI_I2C_WrWord(dev_t, 0x20, config[2]);
       if (status < 0){
               return status;
       }
	   
       status = CCI_I2C_WrWord(dev_t, 0x22, config[3]);
       if (status < 0){
               return status;
       }
	   
      status = CCI_I2C_WrWord(dev_t, 0x24, config[4]);
       if (status < 0){
               return status;
       }
	   
       status = CCI_I2C_WrWord(dev_t, 0x26, config[5]);
       if (status < 0){
               return status;
       }

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	
	return status;
} 

/** @brief Wait device go to standby mode
*
*	@param dev_t the laser focus controller
*
*/
int Laura_WaitDeviceStandby(struct msm_laser_focus_ctrl_t *dev_t){
	int status = 0;
	uint16_t i2c_read_data = 0;
#if 1
	struct timeval start, now;
#endif
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
#if 1
	start = get_current_time();
#endif
	/* Wait chip standby */
	while(1){
		/* Go MCPU to standby mode */
		Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
		
		status = CCI_I2C_RdWord(dev_t, DEVICE_STATUS, &i2c_read_data);
		if (status < 0)
              	break;
        	
		i2c_read_data = (i2c_read_data & STATUS_MASK);
		LOG_Handler(LOG_DBG, "%s: %s in STANDBY MODE, reg(0x06): 0x%x\n", __func__
			, (i2c_read_data==STATUS_STANDBY)?"":"Not", i2c_read_data);

		if(i2c_read_data == STATUS_STANDBY){
			break;
		}

#if 1
		/* Check if time out */
		now = get_current_time();
             	if(is_timeout(start,now,TIMEOUT_VAL)){
			LOG_Handler(LOG_ERR, "%s: Wait chip standby time out - register(0x06): 0x%x\n", __func__, i2c_read_data);
              	status = -TIMEOUT_VAL;
			break;
             	}
#endif
		
		//usleep(DEFAULT_DELAY_TIME);
	}

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return status;
}

/** @brief Configure i2c interface
*
*	@param dev_t the laser focus controller
*
*/
int Laura_Config_I2C_Interface(struct msm_laser_focus_ctrl_t *dev_t){
	int status = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/* Configure I2C interface */
	//include enable auto-increment
	status = CCI_I2C_WrWord(dev_t, 0x1C, 0x0065);
       if (status < 0){
               return status;
       }

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return status;
}

/** @brief Power up initialization without applying calibration data
*
*	@param dev_t the laser focus controller
*
*/
int Laura_No_Apply_Calibration(struct msm_laser_focus_ctrl_t *dev_t){
	int status = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/* Wake up MCPU to ON mode */
	Laura_MCPU_Controller(dev_t, MCPU_ON);

#if 0
	/* wait hardware booting(least 500us) */
	//usleep(MCPU_DELAY_TIME);
#endif

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return status;
}

/** @brief Power up initialization which apply calibration data
*
*	@param dev_t the laser focus controller
*
*/
int Laura_Apply_Calibration(struct msm_laser_focus_ctrl_t *dev_t){
	int status = 0;
	uint16_t i2c_read_data = 0;
	
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/* Set then Verify status is MCPU off */
	Laura_MCPU_Controller(dev_t, MCPU_OFF);

#if 0
	/* wait hardware handling (least 500us) */
	usleep(MCPU_DELAY_TIME);
#endif

	while(1){
		
		status = CCI_I2C_RdWord(dev_t, DEVICE_STATUS, &i2c_read_data);
		if (status < 0){
             		return status;
       	}
		i2c_read_data = (i2c_read_data & STATUS_MASK);

		if(i2c_read_data == STATUS_MCPU_OFF){
			break;
		}
		//usleep(DEFAULT_DELAY_TIME);
	}
	
	/* Load calibration data */
	Olivia_device_Load_Calibration_Value(dev_t);

	Laura_MCPU_Controller(dev_t, MCPU_ON);
	
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return status;
}

/** @brief Go to standby mode when do not do measure
*
*	@param dev_t the laser focus controller
*
*/
int Laura_non_measures_go_standby(struct msm_laser_focus_ctrl_t *dev_t)
{
	int status = 0;
	uint16_t i2c_read_data = 0;
	struct timeval start, now;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	start = get_current_time();

	/* Verify status is MCPU on */
	while(1){
		status = CCI_I2C_RdWord(dev_t, DEVICE_STATUS, &i2c_read_data);
		if (status < 0){
             		return status;
       	}

		//include MCPU_ON
		if(i2c_read_data==STATUS_MEASURE_ON){
			break;
		}

		/* Check if time out */
		now = get_current_time();
             	if(is_timeout(start,now,TIMEOUT_VAL)){
			LOG_Handler(LOG_ERR, "%s: Wait MCPU on time out - register(0x06): 0x%x\n", __func__, i2c_read_data);
              	return -TIMEOUT_VAL;
       	}
		LOG_Handler(LOG_DBG, "%s: register(0x06):0x%x!!\n", __func__,i2c_read_data);

		//usleep(DEFAULT_DELAY_TIME);
	}

	/* Go MCPU to standby mode */
	Laura_MCPU_Controller(dev_t, MCPU_STANDBY);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	
	return status;
}


/** @brief Get module id from chip
*
*       @param dev_t the laser focus controller
*
*/
void Laura_Get_Module_ID_From_Chip(struct msm_laser_focus_ctrl_t *dev_t){
	int status = 0, i = 0;
//uint16_t add = 0x04C8;
//uint16_t add2 = 0xc804;
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
Laura_MCPU_Controller(dev_t, MCPU_OFF);
	status = CCI_I2C_WrByte(dev_t, 0x18, 0x04);
        status = CCI_I2C_WrByte(dev_t, 0x19, 0xC8);
/*
	//for(i = 0; i < 34; i++){
               	CCI_I2C_RdByteSeq(dev_t, 0x1A, module_id_2,34);
       // }
module_id[1]=1;
	   module_id[2]=2;
	   module_id_2[0]=5;
	   module_id_2[1]=5;
	LOG_Handler(LOG_DBG,"Module ID:%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c\n",
		module_id_2[0],module_id_2[1],module_id_2[2],module_id_2[3],module_id_2[4],module_id_2[5],module_id_2[6],module_id_2[7],
		module_id_2[8],module_id_2[9],module_id_2[10],module_id_2[11],module_id_2[12],module_id_2[13],module_id_2[14],module_id_2[15],
		module_id_2[16],module_id_2[17],module_id_2[18],module_id_2[19],module_id_2[20],module_id_2[21],module_id_2[22],module_id_2[23],
		module_id_2[24],module_id_2[25],module_id_2[26],module_id_2[27],module_id_2[28],module_id_2[29],module_id_2[30],module_id_2[31],
		module_id_2[32],module_id_2[33]);
Laura_MCPU_Controller(dev_t, MCPU_OFF);
	status = CCI_I2C_WrByte(dev_t, 0x18, 0x04);
        status = CCI_I2C_WrByte(dev_t, 0x19, 0xC8);
*/
	for(i = 0; i < 34; i++){
               	CCI_I2C_RdByte(dev_t, 0x1A, &module_id[i]);
        }

	LOG_Handler(LOG_DBG,"Module ID(Hex):%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x\n",
		module_id[0],module_id[1],module_id[2],module_id[3],module_id[4],module_id[5],module_id[6],module_id[7],
		module_id[8],module_id[9],module_id[10],module_id[11],module_id[12],module_id[13],module_id[14],module_id[15],
		module_id[16],module_id[17],module_id[18],module_id[19],module_id[20],module_id[21],module_id[22],module_id[23],
		module_id[24],module_id[25],module_id[26],module_id[27],module_id[28],module_id[29],module_id[30],module_id[31],
		module_id[32],module_id[33]);

	LOG_Handler(LOG_DBG,"Module ID:%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d\n",
		module_id[0],module_id[1],module_id[2],module_id[3],module_id[4],module_id[5],module_id[6],module_id[7],
		module_id[8],module_id[9],module_id[10],module_id[11],module_id[12],module_id[13],module_id[14],module_id[15],
		module_id[16],module_id[17],module_id[18],module_id[19],module_id[20],module_id[21],module_id[22],module_id[23],
		module_id[24],module_id[25],module_id[26],module_id[27],module_id[28],module_id[29],module_id[30],module_id[31],
		module_id[32],module_id[33]);	

	LOG_Handler(LOG_DBG,"Module ID:%d%d%d%d%d%d\n",
		module_id[0],module_id[1],module_id[2],module_id[3],module_id[4],module_id[5]);
	
	module_id_flag=true;

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

/*
for(i=0;i<6;i++){
	add+=1;
	printk("add: %d\n",add);
	status = CCI_I2C_WrByte(dev_t, 0x18, (add&0xFF00>>8));
        status = CCI_I2C_WrByte(dev_t, 0x19, (add&0x00FF));
		CCI_I2C_RdByte(dev_t, 0x1A, &module_id[i]);
}
	LOG_Handler(LOG_DBG,"Module ID:%c%c%c%c%c%c\n",module_id[0],module_id[1],module_id[2],module_id[3],module_id[4],module_id[5]);

for(i=0;i<6;i++){
	add+=2;
	printk("add: %d\n",add);
	status = CCI_I2C_WrByte(dev_t, 0x18, (add&0xFF00>>8));
        status = CCI_I2C_WrByte(dev_t, 0x19, (add&0x00FF));
		CCI_I2C_RdByte(dev_t, 0x1A, &module_id[i]);
}
	LOG_Handler(LOG_DBG,"Module ID:%c%c%c%c%c%c\n",module_id[0],module_id[1],module_id[2],module_id[3],module_id[4],module_id[5]);


for(i=0;i<6;i++){
	add2+=1;
	printk("add: %c\n",add2);
	status = CCI_I2C_WrByte(dev_t, 0x18,  (add2&0x00FF));
        status = CCI_I2C_WrByte(dev_t, 0x19, (add2&0xFF00>>8));
		CCI_I2C_RdByte(dev_t, 0x1A, &module_id[i]);
}
	LOG_Handler(LOG_DBG,"Module ID:%c%c%c%c%c%c\n",module_id[0],module_id[1],module_id[2],module_id[3],module_id[4],module_id[5]);

for(i=0;i<6;i++){
	add2+=2;
	printk("add: %c\n",add2);
	status = CCI_I2C_WrByte(dev_t, 0x18,  (add2&0x00FF));
        status = CCI_I2C_WrByte(dev_t, 0x19, (add2&0xFF00>>8));
		CCI_I2C_RdByte(dev_t, 0x1A, &module_id[i]);
}
	LOG_Handler(LOG_DBG,"Module ID:%c%c%c%c%c%c\n",module_id[0],module_id[1],module_id[2],module_id[3],module_id[4],module_id[5]);

	*/
}

/** @brief Get Chip from driver
*
*       @param dev_t the laser focus controller
*       @param vfile
*
*/
void Laura_Get_Module_ID(struct msm_laser_focus_ctrl_t *dev_t, struct seq_file *vfile){

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	if(!module_id_flag){
		Laura_MCPU_Controller(dev_t, MCPU_OFF);
		Laura_Get_Module_ID_From_Chip(dev_t);	
	}
	else{
		LOG_Handler(LOG_DBG,"Module ID:%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c\n",
                	module_id[0],module_id[1],module_id[2],module_id[3],module_id[4],module_id[5],module_id[6],module_id[7],
                	module_id[8],module_id[9],module_id[10],module_id[11],module_id[12],module_id[13],module_id[14],module_id[15],
                	module_id[16],module_id[17],module_id[18],module_id[19],module_id[20],module_id[21],module_id[22],module_id[23],
                	module_id[24],module_id[25],module_id[26],module_id[27],module_id[28],module_id[29],module_id[30],module_id[31],
                	module_id[32],module_id[33]);
	}

	if(vfile!=NULL){
                seq_printf(vfile,"%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c\n",
                        module_id[0],module_id[1],module_id[2],module_id[3],module_id[4],module_id[5],module_id[6],module_id[7],
                        module_id[8],module_id[9],module_id[10],module_id[11],module_id[12],module_id[13],module_id[14],module_id[15],
                        module_id[16],module_id[17],module_id[18],module_id[19],module_id[20],module_id[21],module_id[22],module_id[23],
                        module_id[24],module_id[25],module_id[26],module_id[27],module_id[28],module_id[29],module_id[30],module_id[31],
                        module_id[32],module_id[33]);
        }

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
}


/** @brief Verify firmware version
*
*	@param dev_t the laser focus controller
*
*/
bool Laura_FirmWare_Verify(struct msm_laser_focus_ctrl_t *dev_t){
	int status = 0;
	uint16_t fw_major_version, fw_minor_version;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	
	

#if 0
	/* wait hardware handling (least 500us) */
	usleep(MCPU_DELAY_TIME);
#endif

	status = CCI_I2C_WrByte(dev_t, 0x18, 0xC0);
	status = CCI_I2C_WrByte(dev_t, 0x19, 0xFF);

	status = CCI_I2C_RdByte(dev_t, I2C_DATA_PORT, &fw_major_version);
	fw_major_version = fw_major_version & 0x3F;
	status = CCI_I2C_RdByte(dev_t, I2C_DATA_PORT, &fw_minor_version);

	LOG_Handler(LOG_DBG, "%s: LSB: 0x%x ; MSB: 0x%x\n", __func__, fw_major_version, fw_minor_version);

	if( fw_major_version >= 0 && fw_minor_version >= 14 ){
		/* Can do calibraion */
		LOG_Handler(LOG_DBG, "%s: It can do calibration!!\n", __func__);
		return true;
	}
	else{
		/* Can not do calibraion */
		LOG_Handler(LOG_DBG, "%s: The fireware is too old, it can not do calibration!!\n", __func__);
		return false;
	}

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return false;
}

int Olivia_DumpKdata(struct msm_laser_focus_ctrl_t *dev_t, int16_t cal_data[11]){
	int status = 0, i=0;
	uint16_t i2c_read_data;
	Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
	usleep_range(10000, 10000);
	status = CCI_I2C_RdWord(dev_t, DEVICE_STATUS, &i2c_read_data);
	if (status < 0){
             	return status;
       }	
	LOG_Handler(LOG_DBG, "%s: register(0x06):0x%x!!\n", __func__,i2c_read_data);

	
	Laura_MCPU_Controller(dev_t, MCPU_OFF);

	
	usleep_range(10000, 10000);
	status = CCI_I2C_RdWord(dev_t, DEVICE_STATUS, &i2c_read_data);
	if (status < 0){
             	return status;
       }	
	LOG_Handler(LOG_DBG, "%s: register(0x06):0x%x!!\n", __func__,i2c_read_data);

	
	status = CCI_I2C_WrByte(dev_t, 0x18, 0x30);
	status = CCI_I2C_WrByte(dev_t, 0x19, 0xC8);

	
	for(i = 0; i < 22; i++){
               	CCI_I2C_RdByte(dev_t, 0x1A, &f0_data[i]);
       }
	
	for(i=0;i < 11; i++){
		cal_data[i] = (f0_data[2*i] | f0_data[2*i+1]<<8);
	}
	
		LOG_Handler(LOG_DBG,"dump f0_data:%04x  %04x  %04x  %04x  %04x  %04x  %04x  %04x  %04x  %04x  %04x  \n",
                	cal_data[0],cal_data[1],cal_data[2],cal_data[3],cal_data[4],cal_data[5],cal_data[6],cal_data[7],
                	cal_data[8],cal_data[9],cal_data[10]);
/*

	status = CCI_I2C_WrByte(dev_t, 0x18, 0x30);
	status = CCI_I2C_WrByte(dev_t, 0x19, 0xC8);

	for(i = 0; i < 11; i++){
               	CCI_I2C_RdWord(dev_t, 0x1A, &f0_data_test[i]);
       }
		LOG_Handler(LOG_DBG,"dump f0_data_test:%04x  %04x  %04x  %04x  %04x  %04x  %04x  %04x  %04x  %04x  %04x  \n",
                	f0_data_test[0],f0_data_test[1],f0_data_test[2],f0_data_test[3],f0_data_test[4],f0_data_test[5],f0_data_test[6],f0_data_test[7],
                	f0_data_test[8],f0_data_test[9],f0_data_test[10]);
*/

	return status;
}
