/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-07
*
*/

#include "laura_factory_func.h"
#include "show_log.h"
#include <linux/delay.h>
#include <linux/timer.h>

/** @brief laura calibration read
*
*	@param dev_t the laser focus controller
*
*/
uint16_t Laura_device_read_range2(struct msm_laser_focus_ctrl_t *dev_t)
{
	uint16_t RawRange, i2c_read_data = 0;
	int status;
	struct timeval start, now;
	int read_range_log_count = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	start = get_current_time();

	/* read count +1  */
	read_range_log_count++;
#if READ_RETRY_FLAG
	while(1){
#endif
		/* Trigger single measure */
        	status = CCI_I2C_WrWord(dev_t, 0x04, 0x0081);
        	if (status < 0){
              	return status;
        	}

		start = get_current_time();
		/* Wait until data ready */
       	while(1){
			status = CCI_I2C_RdWord(dev_t, 0x00, &i2c_read_data);
			if (status < 0){
       			return status;
       		}

			if((int)(i2c_read_data&0x10) == 0x10){
				break;
			}

			/* Check if time out */
			now = get_current_time();
             		if(is_timeout(start,now,10000)){
				LOG_Handler(LOG_ERR, "%s: Wait data ready time out - register(0x00): 0x%x\n", __func__, i2c_read_data);
                    		return OUT_OF_RANGE;
             		}

			/* Delay: waitting laser sensor sample ready */
			//usleep(DEFAULT_DELAY_TIME);
			
       	}

		/* Read distance */
       	status = CCI_I2C_RdWord(dev_t, 0x08, &RawRange);
		if (status < 0){
              	return status;
       	}
	
		/* Check if target is out of field of view */
		if((RawRange&0x6000)==0x00 && (RawRange&0x8000)==0x8000){
#if READ_OUTPUT_LIMIT_FLAG == 0
			/* Get real range */
			LOG_Handler(LOG_DBG, "%s: Non-shift Read range:%d\n", __func__, RawRange);
			RawRange = (RawRange&0x1fff)>>2;
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
             	if(is_timeout(start,now,TIMEOUT)){
			LOG_Handler(LOG_ERR, "%s: Read range time out!!\n", __func__);
              	return OUT_OF_RANGE;
             	}
	}
#endif

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	
	return RawRange;
}

uint16_t Olivia_device_read_range2(struct msm_laser_focus_ctrl_t *dev_t)
{
	uint16_t RawRange, i2c_read_data = 0;
	int status;
	struct timeval start, now;
	int read_range_log_count = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	start = get_current_time();

	/* read count +1  */
	read_range_log_count++;

		/* Trigger single measure */
        	status = CCI_I2C_WrWord(dev_t, 0x04, 0x0081);
        	if (status < 0){
              	return status;
        	}

		start = get_current_time();
		/* Wait until data ready */
       	while(1){
			status = CCI_I2C_RdWord(dev_t, 0x00, &i2c_read_data);
			if (status < 0){
       			return status;
       		}

			if((int)(i2c_read_data&0x10) == 0x10){
				break;
			}

			/* Check if time out */
			now = get_current_time();
             		if(is_timeout(start,now,10000)){
				LOG_Handler(LOG_ERR, "%s: Wait data ready time out - register(0x00): 0x%x\n", __func__, i2c_read_data);
                    		return OUT_OF_RANGE;
             		}

			/* Delay: waitting laser sensor sample ready */
			//usleep(DEFAULT_DELAY_TIME);
			
       	}

		/* Read distance */
       	status = CCI_I2C_RdWord(dev_t, 0x08, &RawRange);
		if (status < 0){
              	return status;
       	}
	
		/* Check if target is out of field of view */
		if((RawRange&0x6000)==0x00 && (RawRange&0x8000)==0x8000){
#if READ_OUTPUT_LIMIT_FLAG == 0
			/* Get real range */
			LOG_Handler(LOG_DBG, "%s: Non-shift Read range:%d\n", __func__, RawRange);
			RawRange = (RawRange&0x1fff)>>2;
			LOG_Handler(LOG_CDBG, "%s: Read range:%d\n", __func__, RawRange);
#endif
#if READ_OUTPUT_LIMIT_FLAG
			/* Display distance */
			if(read_range_log_count >= LOG_SAMPLE_RATE){
				read_range_log_count = 0;
				LOG_Handler(LOG_CDBG, "%s: Read range:%d\n", __func__, RawRange);
			}
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
			return OUT_OF_RANGE;
       	}
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	
	return RawRange;
}

/** @brief laura calibration
*
*	@param dev_t the laser focus controller
*	@param cal_input_data the calibration input data
*
*/
int Laura_device_calibration(struct msm_laser_focus_ctrl_t *dev_t, int16_t *cal_input_data)
{
	int status = 0, pass_count = 0, fail_count = 0, i = 0, j = 0;
	uint16_t distance = 0;
	int16_t cal_data[CAL_MSG_LEN];
	uint16_t i2c_read_data;
	struct timeval start,now;
	O_get_current_time(&start);

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	if (dev_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF ||
		dev_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI) {
		LOG_Handler(LOG_ERR, "%s: Device without turn on: (%d) \n", __func__, dev_t->device_state);
		return -EBUSY;
	}

	while(1){
		status = CCI_I2C_RdWord(dev_t, DEVICE_STATUS, &i2c_read_data);
		if (status < 0){
              		return status;
        	}

		if(i2c_read_data == STATUS_MEASURE_ON){
			break;
		}

		O_get_current_time(&now);
       	if(is_timeout(start,now,TIMEOUT_VAL)){
			LOG_Handler(LOG_ERR, "%s: Wait MCPU on time out - register(0x06): 0x%x\n", __func__, i2c_read_data);
             		return -TIMEOUT_VAL;
            	}
		usleep_range(DEFAULT_DELAY_TIME, DEFAULT_DELAY_TIME);
	}

	do{
		/* Read distance */
		distance = Laura_device_read_range2(dev_t);

		if(distance==OUT_OF_RANGE || (distance&0x6000)!=0x00){
			fail_count = fail_count + 1;
			LOG_Handler(LOG_ERR, "%s: Read fail, count:%d\n", __func__, fail_count);
			if(fail_count >= 2){
				LOG_Handler(LOG_ERR, "%s: K stop\n", __func__);
				return -EMODULE;
			}
			continue;
		}
		
		pass_count += 1;
		
		/* CMD_MBX to read data */
		status = Olivia_Mailbox_Command(dev_t, cal_data);
		if(status < 0){
			LOG_Handler(LOG_ERR, "%s: MBX Command failed!!\n", __func__);
			return -EMODULE;
		}
		
		/* Append MBX data into a file */
		j=(pass_count==1)?0: CAL_MSG_LEN;
		for(i = 0; i < CAL_MSG_LEN; i++){
			cal_input_data[i+j] = (int16_t)cal_data[i];
			LOG_Handler(LOG_DBG, "%s: Calibration data[%d]: %d\n", __func__, i+j, cal_input_data[i]);
		}
		
	}while(pass_count < 2);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	
	return 0;
}

/** @brief laura do calibration
*
*	@param cal_10 the 10cm calibration input data
*	@param cal_40 the 40cm calibration input data
*	@param cal_inf the infinity calibration input data
*
*/
uint16_t* calc_calib(int16_t *cal_10, int16_t *cal_40, int16_t *cal_inf){
#if 0
	int status = 0;
#endif
	int16_t *output;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	output = kzalloc(sizeof(int16_t)*11, GFP_KERNEL);
	
	if(!output){
		pr_err("%s:%d failed no memory\n", __func__, __LINE__);
		return NULL;

	}
#if DEBUG_LOG_FLAG
	LOG_Handler(LOG_DBG, "%s: cal_10 :(%d, %d, %d, %d, %d, %d, %d, %d , %d, %d, %d, %d)\n", __func__, 
		cal_10[0], cal_10[1], cal_10[2], cal_10[3], cal_10[4], cal_10[5], 
		cal_10[6], cal_10[7], cal_10[8], cal_10[9], cal_10[10], cal_10[11]);

	LOG_Handler(LOG_DBG, "%s: cal_40 :(%d, %d, %d, %d, %d, %d, %d, %d , %d, %d, %d, %d)\n", __func__, 
		cal_40[0], cal_40[1], cal_40[2], cal_40[3], cal_40[4], cal_40[5], 
		cal_40[6], cal_40[7], cal_40[8], cal_40[9], cal_40[10], cal_40[11]);

	LOG_Handler(LOG_DBG, "%s: cal_inf :(%d, %d, %d, %d, %d, %d, %d, %d , %d, %d, %d, %d)\n", __func__, 
		cal_inf[0], cal_inf[1], cal_inf[2], cal_inf[3], cal_inf[4], cal_inf[5], 
		cal_inf[6], cal_inf[7], cal_inf[8], cal_inf[9], cal_inf[10], cal_inf[11]);
#endif

#if 0
	/* Compute calibration output data (call Heptagon calibration library) */
	status = do_calc_calib(cal_10, cal_40, cal_inf, output);
	if(status  != 0){
		LOG_Handler(LOG_ERR, "%s: Calibration fail(0x%x)\n", __func__, status );
		return NULL;
	}
#endif

#if DEBUG_LOG_FLAG
	LOG_Handler(LOG_DBG, "%s: calibration output data: (%d, %d, %d, %d, %d, %d, %d, %d , %d, %d)\n", __func__, 
		output[0], output[1], output[2], output[3], output[4], output[5], output[6], output[7], output[8], output[9]);
#endif

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return output;
}

/** @brief Laura create calibration data
*
*	@param cal_10 the 10cm calibration input data
*	@param cal_40 the 40cm calibration input data
*	@param cal_inf the infinity calibration input data
*
*/
int Larua_Create_Calibration_Data(int16_t *cal_10, int16_t *cal_40, int16_t *cal_inf){
	int status = 0;
	int16_t *cal_data;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	cal_data = kzalloc(sizeof(int16_t)*(SIZE_OF_LAURA_CALIBRATION_DATA+1), GFP_KERNEL);

	cal_data = calc_calib(cal_10, cal_40, cal_inf);

	Larua_Write_Calibration_Data_Into_File(cal_data, SIZE_OF_LAURA_CALIBRATION_DATA);

	kfree(cal_data);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return status;
} 

/** @brief Laura write calibration data to file
*
*	@param cal_data the calibration data
*	@param size the size of calibration data
*
*/
int Larua_Write_Calibration_Data_Into_File(int16_t *cal_data, uint32_t size){
	int status = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

#ifdef CONFIG_ASUS_FACTORY_MODE
	Sysfs_write_word_seq(LAURA_CALIBRATION_FACTORY_FILE, cal_data, size);
#else
	Sysfs_write_word_seq(LAURA_CALIBRATION_FACTORY_FILE, cal_data, size);
#endif

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return status;
}

/** @brief Laura read calibration data from file
*
*	@param cal_data the calibration data
*	@param size the size of calibration data
*
*/
int Larua_Read_Calibration_Data_From_File(int16_t *cal_data, uint32_t size){
	int status = 0, i = 0;
	int buf[size];

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	
	status = Sysfs_read_word_seq(LAURA_CALIBRATION_FACTORY_FILE, buf, size);

	if(status < 0){
		LOG_Handler(LOG_ERR, "%s: Load calibration fail!!\n", __func__);
		for(i = 0; i < size; i++){
			cal_data[i] = 0x0000;
		}
		return status;
	}

	for(i = 0; i < size; i++){
		cal_data[i] = (uint16_t)buf[i];
//		swap_data(cal_data+i);
	}

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return status;
}

/** @brief Laura get calibration input data
*
*	@param buf
*	@param v
*	@param cal_10 the 10cm calibration input data
*	@param cal_40 the 40cm calibration input data
*	@param cal_inf the infinity calibration input data
*
*/
int Laura_get_calibration_input(struct seq_file *buf, void *v, 
	int16_t *cal_data_10, int16_t *cal_data_40, int16_t *cal_data_inf){
	
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	
	seq_printf(buf, "%d %d %d %d %d %d %d %d %d %d %d %d "
		"%d %d %d %d %d %d %d %d %d %d %d %d "
		"%d %d %d %d %d %d %d %d %d %d %d %d\n", 
		cal_data_10[0], cal_data_10[1], cal_data_10[2], cal_data_10[3], cal_data_10[4], cal_data_10[5],
		cal_data_10[6], cal_data_10[7], cal_data_10[8], cal_data_10[9], cal_data_10[10], cal_data_10[11],
		cal_data_40[0], cal_data_40[1], cal_data_40[2], cal_data_40[3], cal_data_40[4], cal_data_40[5],
		cal_data_40[6], cal_data_40[7], cal_data_40[8], cal_data_40[9], cal_data_40[10], cal_data_40[11],
		cal_data_inf[0], cal_data_inf[1], cal_data_inf[2], cal_data_inf[3], cal_data_inf[4], cal_data_inf[5],
		cal_data_inf[6], cal_data_inf[7], cal_data_inf[8], cal_data_inf[9], cal_data_inf[10], cal_data_inf[11]);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return 0;
}

int Olivia_get_calibration_input(struct seq_file *buf, void *v, 
	int16_t *cal_data_10, int16_t *cal_data_40, int16_t *cal_data_inf, int16_t *cal_data_f0){
	
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	
	seq_printf(buf, "%d %d %d %d %d %d %d %d %d %d %d %d "
		"%d %d %d %d %d %d %d %d %d %d %d %d "
		"%d %d %d %d %d %d %d %d %d %d %d %d "
		"%d %d %d %d %d %d %d %d %d %d %d\n", 
		cal_data_10[0], cal_data_10[1], cal_data_10[2], cal_data_10[3], cal_data_10[4], cal_data_10[5],
		cal_data_10[6], cal_data_10[7], cal_data_10[8], cal_data_10[9], cal_data_10[10], cal_data_10[11],
		cal_data_40[0], cal_data_40[1], cal_data_40[2], cal_data_40[3], cal_data_40[4], cal_data_40[5],
		cal_data_40[6], cal_data_40[7], cal_data_40[8], cal_data_40[9], cal_data_40[10], cal_data_40[11],
		cal_data_inf[0], cal_data_inf[1], cal_data_inf[2], cal_data_inf[3], cal_data_inf[4], cal_data_inf[5],
		cal_data_inf[6], cal_data_inf[7], cal_data_inf[8], cal_data_inf[9], cal_data_inf[10], cal_data_inf[11],
		cal_data_f0[0], cal_data_f0[1], cal_data_f0[2], cal_data_f0[3], cal_data_f0[4], cal_data_f0[5],
		cal_data_f0[6], cal_data_f0[7], cal_data_f0[8], cal_data_f0[9], cal_data_f0[10]);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return 0;
}


