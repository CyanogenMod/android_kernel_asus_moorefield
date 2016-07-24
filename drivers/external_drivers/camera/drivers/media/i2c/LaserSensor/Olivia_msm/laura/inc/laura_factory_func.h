/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-07
*
*/

#ifndef __LINUX_SHOW_LAURA_SENSOR_FACTORY_FUNC_H
#define __LINUX_SHOW_LAURA_SENSOR_FACTORY_FUNC_H

#include "msm_laser_focus.h"
#include "laura_shipping_func.h"
#include "laura_cal.h"

#ifndef CALIBRATION_FAIL
/* Calibration fail*/
#define CALIBRATION_FAIL -1
#endif

/* Bad module */
#define EMODULE 1

/* Size of Laura calibration data */
#define SIZE_OF_LAURA_CALIBRATION_DATA 10
#define SIZE_OF_OLIVIA_CALIBRATION_DATA 10

/* Calibration */
int Laura_device_calibration(struct msm_laser_focus_ctrl_t *dev_t, int16_t *cal_input_data);
/* Laura do calibration */
uint16_t* calc_calib(int16_t *cal_10, int16_t *cal_40, int16_t *cal_inf);
/* Laura create calibration data */
int Larua_Create_Calibration_Data(int16_t *cal_10, int16_t *cal_40, int16_t *cal_inf);
/* Laura write calibration data to file */
int Larua_Write_Calibration_Data_Into_File(int16_t *cal_data, uint32_t size);
/* Laura read calibration data from file */
int Larua_Read_Calibration_Data_From_File(int16_t *cal_data, uint32_t size);
/* Laura get calibration input data */
int Laura_get_calibration_input(struct seq_file *buf, void *v, 
	int16_t *cal_data_10, int16_t *cal_data_40, int16_t *cal_data_inf);

int Olivia_get_calibration_input(struct seq_file *buf, void *v, 
	int16_t *cal_data_10, int16_t *cal_data_40, int16_t *cal_data_inf, int16_t *cal_data_f0);

#endif
