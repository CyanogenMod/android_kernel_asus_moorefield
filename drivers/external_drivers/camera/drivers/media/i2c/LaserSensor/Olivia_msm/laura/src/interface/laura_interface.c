/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-08
*
*/

#include "laura_interface.h"
#include "show_log.h"

#define DO_MEASURE true

extern int Laser_Product;

/* Calibration input data */
int16_t cal_data_10[CAL_MSG_LEN*2];	/* Calibration 10cm input data */
int16_t cal_data_40[CAL_MSG_LEN*2];	/* Calibration 40cm input data */
int16_t cal_data_inf[CAL_MSG_LEN*2];	/* Calibration infinity input data */
int16_t cal_data_f0[11];
#if 0
int16_t cal_data_10[12] = {-1231,364,3721,-1029,824,12814,
                            -1222,356,3676,-1000,785,12814};
int16_t cal_data_40[12] =  {-27,-29,6500,-1021,816,12814,
                            -26,-29,6500,-972,764,12814};
int16_t cal_data_inf[12] = {-6,-2,6500,-1025,823,12814,
                            -6,-2,6500,-975,769,12814};
#endif

/** @brief laura calibration interface
*
*	@param dev_t the laser focus controller
*	@param load_cal if load calibration data
*	@param calibration_flag the calibration flag
*	@param ctrl the calibration controller
*
*/

int Laura_device_calibration_interface(struct msm_laser_focus_ctrl_t *dev_t, bool load_cal, bool *calibration_flag, int ctrl){
	int status = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/* Power up initialization */
	status = Laura_device_wake_up_interface(dev_t, load_cal, calibration_flag, DO_MEASURE);

	switch(ctrl){
		/* Do 10cm calibration */
		case MSM_LASER_FOCUS_APPLY_OFFSET_CALIBRATION:
			/* Set TOF configuration */
			status = Laura_device_tof_configuration_interface(dev_t, ctrl);
			if(status < 0){
				/* Go MCPU to standby mode */
				Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
				return status;
			}
			/* Do calibration */
			status = Laura_device_calibration(dev_t, cal_data_10);
			if(status < 0){
				/* Go MCPU to standby mode */
				Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
				return status;
			}
			break;
		/* Do 40cm calibration */
		case MSM_LASER_FOCUS_APPLY_CROSSTALK_CALIBRATION:
			/* Set TOF configuration */
			status = Laura_device_tof_configuration_interface(dev_t, ctrl);
			if(status < 0){
				/* Go MCPU to standby mode */
				Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
				return status;
			}
			/* Do calibration */
			status = Laura_device_calibration(dev_t, cal_data_40);
			if(status < 0){
				/* Go MCPU to standby mode */
				Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
				return status;
			}
			break;
		/* Do infinity calibration */
		case MSM_LASER_FOCUS_APPLY_INFINITY_CALIBRATION:
			/* Set TOF configuration */
			status = Laura_device_tof_configuration_interface(dev_t, ctrl);
			if(status < 0){
				/* Go MCPU to standby mode */
				Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
				return status;
			}
			/* Do calibration */
			status = Laura_device_calibration(dev_t, cal_data_inf);
			if(status < 0){
				/* Go MCPU to standby mode */
				Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
				return status;
			}
#if 0
			status = Larua_Create_Calibration_Data(cal_data_10, cal_data_40, cal_data_inf);
			if(status < 0){
				/* Go MCPU to standby mode */
				Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
				return status;
			}
#endif
			break;
		default:
			LOG_Handler(LOG_ERR, "%s: command fail (%d) !!\n", __func__, ctrl);
			/* Go MCPU to standby mode */
			Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
			return -1;
	}

	/* Go MCPU to standby mode */
	Laura_MCPU_Controller(dev_t, MCPU_STANDBY);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return status;
}



/** @brief laura read calibration input data from calibration file interface
*
*       @param buf
*       @param v
*	@param cal_data the calibration data
*
*/
int Laura_read_calibration_data_interface(struct seq_file *buf, void *v, uint16_t *cal_data){
	int status = 0;
	
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/* read calibration data */
	status = Laura_Read_Calibration_Value_From_File(buf, cal_data);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return status;
}

int Olivia_read_calibration_data_interface(struct seq_file *buf, void *v, uint16_t *cal_data){
	int status = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/* read calibration data */
	status = Olivia_Read_Calibration_Value_From_File(buf, cal_data);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return status;
}

int Laura_get_module_id_interface(struct msm_laser_focus_ctrl_t *dev_t, struct seq_file *buf, void *v){
        int status = 0;

        LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

        /* Get module id */
        Laura_Get_Module_ID(dev_t,buf);

        LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
        return status;
}


int	Olivia_Do_TOF_and_Calibration(struct msm_laser_focus_ctrl_t *dev_t, enum laura_configuration_ctrl  cal_type, int16_t* cal_data){

	int status = 0;
	
	/* Set TOF configuration */
	status = Olivia_device_tof_configuration_interface(dev_t, cal_type);
	if(status < 0)
		return status;
	
	/* Do calibration */
	status = Laura_device_calibration(dev_t, cal_data);
	if(status < 0)
		return status;

	return status;
}



int Olivia_device_calibration_interface(struct msm_laser_focus_ctrl_t *dev_t, bool load_cal, bool *calibration_flag, int ctrl){
	int status = 0;
	static int calib[2] = {0,0};
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/* Power up initialization */
	status = Laura_device_wake_up_interface(dev_t, load_cal, calibration_flag, DO_MEASURE);

	/* Do 10,40,inf cm calibration */
	switch(ctrl){		
		case MSM_LASER_FOCUS_APPLY_OFFSET_CALIBRATION:
			status = Olivia_Do_TOF_and_Calibration(dev_t, ctrl, cal_data_10);
			if(status < 0)
				goto err;
			calib[0] = 1;
			break;
		case MSM_LASER_FOCUS_APPLY_CROSSTALK_CALIBRATION:
			status = Olivia_Do_TOF_and_Calibration(dev_t, ctrl, cal_data_40);
			if(status < 0)
				goto err;
			calib[1] = 1;
			break;
		case MSM_LASER_FOCUS_APPLY_INFINITY_CALIBRATION:
			status = Olivia_Do_TOF_and_Calibration(dev_t, ctrl, cal_data_inf);
			if(status < 0)
				goto err;
#if 0
			status = Larua_Create_Calibration_Data(cal_data_10, cal_data_40, cal_data_inf);
			if(status < 0){
				/* Go MCPU to standby mode */
				Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
				return status;
			}
#endif
			break;
		default:
			LOG_Handler(LOG_ERR, "%s: command fail (%d) !!\n", __func__, ctrl);
			/* Go MCPU to standby mode */
			Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
			return -1;
	}

	if(calib[0]&&calib[1]){
		calib[0] = 0;
		calib[1] = 0;
		Olivia_DumpKdata(dev_t, cal_data_f0);
	}


	/* Go MCPU to standby mode */
	Laura_MCPU_Controller(dev_t, MCPU_STANDBY);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return status;

err:

	LOG_Handler(LOG_ERR, "%s: fail (%d) !!\n", __func__, status);	
	/* Go MCPU to standby mode */
	Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
	return status;	
}


/** @brief laura get calibration input data interface
*
*	@param buf
*	@param v
*
*/
int Laura_get_calibration_input_data_interface(struct seq_file *buf, void *v){
	int status = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/* Get calibration input data */
	status = Laura_get_calibration_input(buf, v, cal_data_10, cal_data_40, cal_data_inf);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return 0;
}


int Olivia_get_calibration_input_data_interface(struct seq_file *buf, void *v){
	int status = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/* Get calibration input data */
	status = Olivia_get_calibration_input(buf, v, cal_data_10, cal_data_40,cal_data_inf, cal_data_f0);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return 0;
}
/** @brief laura read range interface
*
*	@param dev_t the laser focus controller
*	@param load_cal if load calibration data
*	@param calibration_flag the calibration flag
*
*/
int Laura_device_read_range_interface(struct msm_laser_focus_ctrl_t *dev_t, bool load_cal, bool *calibration_flag){
	int status = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/* Power up initialization */
	status = Laura_device_wake_up_interface(dev_t, load_cal, calibration_flag, DO_MEASURE);
	if(status < 0){
		Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
		return status;
	}

	/* Set TOF configuration */
	status = Olivia_device_tof_configuration_interface(dev_t, LAURA_READ_CONFIG);
	if(status < 0){
		Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
		return status;
	}
	
	// Read range 
	//status = Laura_device_read_range(dev_t);
	status = Olivia_device_read_range(dev_t);	
	if(status < 0){
		Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
		return status;
	}

	Laura_MCPU_Controller(dev_t, MCPU_STANDBY);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return status;
}

/** @brief laura power up initialization interface (verify firmware)
*
*	@param dev_t the laser focus controller
*	@param load_cal if load calibration data
*	@param calibration_flag the calibration flag
*	@param do_measure if do measure
*
*/
int Laura_device_power_up_init_interface(struct msm_laser_focus_ctrl_t *dev_t, bool load_cal, bool *cal_flag, bool do_measure){
	int status = 0;

	/* Wait device go to standby */
	status = Laura_WaitDeviceStandby(dev_t);
	if(status < 0){
		/* Go MCPU to standby mode */
		Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
		return status;
	}
	CCI_I2C_WrWord(dev_t, 0x00, 0x0004);//drive INT_PAD High,add by jevian.

	/* Configure I2C interface */
	status = Laura_Config_I2C_Interface(dev_t);
	if(status < 0){
		/* Go MCPU to standby mode */
		Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
		return status;
	}

	Laura_MCPU_Controller(dev_t, MCPU_OFF);

	if(Laser_Product==PRODUCT_LAURA){
		/* Verify firmware version and check if we can do calibration */
		*cal_flag = Laura_FirmWare_Verify(dev_t);
	}

	/* Load default calibration value */
	if(load_cal && *cal_flag){
		status = Laura_Apply_Calibration(dev_t);
		if(status < 0){
			Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
			return status;
		}
	}
	/* Do not load default calibration value */
	else{
		status = Laura_No_Apply_Calibration(dev_t);
		if(status < 0){
			Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
			return status;
		}
	}     

	/* Check if do measure */
	if(!do_measure){
		LOG_Handler(LOG_CDBG, "%s: No measure, go standby \n", __func__);
		status = Laura_non_measures_go_standby(dev_t);
		if(status < 0){
			/* Go MCPU to standby mode */
			Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
			return status;
		}
	}
	
	return status;
}

/** @brief laura power up initialization interface (non-verify firmware)
*
*	@param dev_t the laser focus controller
*	@param load_cal if load calibration data
*	@param calibration_flag the calibration flag
*	@param do_measure if do measure
*
*/
int Laura_device_wake_up_interface(struct msm_laser_focus_ctrl_t *dev_t, bool load_cal, bool *cal_flag, bool do_measure){
	int status = 0;

	/* Wait device go to standby */
	status = Laura_WaitDeviceStandby(dev_t);
	if(status < 0){
		Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
		return status;
	}
	
	/* Load default calibration value */
	if(load_cal && *cal_flag){
		status = Laura_Apply_Calibration(dev_t);
		if(status < 0){
			Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
			return status;
		}
	}
	/* Do not load default calibration value */
	else{
		status = Laura_No_Apply_Calibration(dev_t);
		if(status < 0){
			Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
			return status;
		}
	}

	/* Check if do measure */
	if(!do_measure){
		/* Go MCPU to standby mode */
		status = Laura_non_measures_go_standby(dev_t);
		if(status < 0){
			/* Go MCPU to standby mode */
			Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
			return status;
		}
	}

	return status;
}


extern int Laser_Product;
/** @brief laura configuration interface
*
*	@param dev_t the laser focus controller
*	@param ctrl the tof configuration controller
*
*/
int Laura_device_tof_configuration_interface(struct msm_laser_focus_ctrl_t *dev_t, int ctrl){
	int status = 0;

	/* TOF configuratino default value */
	uint16_t config_normal_laura[LAURA_CONFIG_SIZE] = {0xE100, 0x30FF, 0x07D0, 0xE002, 0xA041, 0x4580};
	uint16_t config_cal_10_laura[LAURA_CONFIG_SIZE] = {0xE100, 0x30FF, 0x07D0, 0xE002, 0xAC81, 0x4580};
	uint16_t config_cal_40_laura[LAURA_CONFIG_SIZE] = {0xE100, 0x30FF, 0x07D0, 0xE002, 0xAC81, 0x4580};
	uint16_t config_cal_inf_laura[LAURA_CONFIG_SIZE] = {0xE100, 0x30FF, 0x07D0, 0xE000, 0xFC81, 0x4500};

		
	switch(ctrl){
		/* These are set TOF configuration */
		case MSM_LASER_FOCUS_APPLY_NORMAL_CALIBRATION:			
			status = Laura_device_UpscaleRegInit(dev_t, config_normal_laura);
			break;
		case MSM_LASER_FOCUS_APPLY_OFFSET_CALIBRATION:
			status = Laura_device_UpscaleRegInit(dev_t, config_cal_10_laura);
			break;
		case MSM_LASER_FOCUS_APPLY_CROSSTALK_CALIBRATION:
			status = Laura_device_UpscaleRegInit(dev_t, config_cal_40_laura);
			break;
		case MSM_LASER_FOCUS_APPLY_INFINITY_CALIBRATION:
			status = Laura_device_UpscaleRegInit(dev_t, config_cal_inf_laura);
			break;
		default:
			LOG_Handler(LOG_ERR, "%s: command fail (%d) !!\n", __func__, ctrl);
			return -1;
	}
	
	return status;
}



int Olivia_device_tof_configuration_interface(struct msm_laser_focus_ctrl_t *dev_t, int ctrl){
	int status = 0;

	/* TOF configuratino default value */
	uint16_t config_normal_olivia[LAURA_CONFIG_SIZE] = {0xE100, 0x30FF, 0x07D0, 0x5001, 0xA041, 0x4590};
	uint16_t config_cal_10_olivia[LAURA_CONFIG_SIZE] = {0xE100, 0x30FF, 0x07D0, 0x5001, 0x2C81, 0x4510};
	uint16_t config_cal_40_olivia[LAURA_CONFIG_SIZE] = {0xE100, 0x30FF, 0x07D0, 0x5000, 0xFC81, 0x4510};
	uint16_t config_cal_inf_olivia[LAURA_CONFIG_SIZE] = {0xE100, 0x30FF, 0x07D0, 0xE000, 0xFC81, 0x4500};
		
	switch(ctrl){
		/* These are set TOF configuration */
		case MSM_LASER_FOCUS_APPLY_NORMAL_CALIBRATION:			
			status = Laura_device_UpscaleRegInit(dev_t, config_normal_olivia);
			break;
		case MSM_LASER_FOCUS_APPLY_OFFSET_CALIBRATION:
			status = Laura_device_UpscaleRegInit(dev_t, config_cal_10_olivia);
			break;
		case MSM_LASER_FOCUS_APPLY_CROSSTALK_CALIBRATION:
			status = Laura_device_UpscaleRegInit(dev_t, config_cal_40_olivia);
			break;
		case MSM_LASER_FOCUS_APPLY_INFINITY_CALIBRATION:
			status = Laura_device_UpscaleRegInit(dev_t, config_cal_inf_olivia);
			break;
		default:
			LOG_Handler(LOG_ERR, "%s: command fail (%d) !!\n", __func__, ctrl);
			return -1;
	}
	
	return status;
}

