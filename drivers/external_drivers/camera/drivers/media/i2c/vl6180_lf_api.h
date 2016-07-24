/* Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef HELLO_H
#define HELLO_H

int ASUS_VL6180x_WrByte(uint32_t register_addr, uint8_t i2c_write_data);
int ASUS_VL6180x_RdByte(uint32_t register_addr, uint8_t *i2c_read_data);
int ASUS_VL6180x_WrWord(uint32_t register_addr, uint16_t i2c_write_data);
int ASUS_VL6180x_RdWord(uint32_t register_addr, uint16_t *i2c_read_data);
int ASUS_VL6180x_WrDWord(uint32_t register_addr, uint32_t i2c_write_data);
int ASUS_VL6180x_RdDWord(uint32_t register_addr, uint32_t *i2c_read_data, uint16_t num_byte);
int ASUS_VL6180x_UpdateByte(uint32_t register_addr, uint8_t AndData, uint8_t OrDat);


//Device Registers
#define VL6180_MODEL_ID_REG			    	0x0000
#define VL6180_MODEL_REV_MAJOR_REG		    0x0001
#define VL6180_MODEL_REV_MINOR_REG		    0x0002
#define VL6180_MODULE_REV_MAJOR_REG		    0x0003
#define VL6180_MODULE_REV_MINOR_REG		    0x0004

#define VL6180_REVISION_ID_REG			    0x0005
#define VL6180_REVISION_ID_REG_BYTES		1
#define VL6180_DATE_HI_REG			    	0x0006
#define VL6180_DATE_HI_REG_BYTES		    1
#define VL6180_DATE_LO_REG			    	0x0007
#define VL6180_DATE_LO_REG_BYTES	   	    1
#define VL6180_TIME_REG			    	    0x0008
#define VL6180_TIME_REG_BYTES			    2
#define VL6180_CODE_REG			    	    0x000a
#define VL6180_CODE_REG_BYTES			    1
#define VL6180_FIRMWARE_REVISION_ID_REG	    	    0x000b
#define VL6180_FIRMWARE_REVISION_ID_REG_BYTES	    1

// Result Registers
#define VL6180_RESULT__RANGE_RAW_REG                                0x0064
#define VL6180_RESULT__RANGE_RAW_REG_BYTES                          1
#define VL6180_RESULT__RANGE_RETURN_RATE_REG                        0x0066
#define VL6180_RESULT__RANGE_RETURN_RATE_REG_BYTES                  2
#define VL6180_RESULT__RANGE_REFERENCE_RATE_REG                     0x0068
#define VL6180_RESULT__RANGE_REFERENCE_RATE_REG_BYTES               2
#define VL6180_RESULT__RANGE_RETURN_VCSEL_COUNT_REG                 0x006c
#define VL6180_RESULT__RANGE_RETURN_VCSEL_COUNT_REG_BYTES           4           
#define VL6180_RESULT__RANGE_REFERENCE_VCSEL_COUNT_REG              0x0070
#define VL6180_RESULT__RANGE_REFERENCE_VCSEL_COUNT_REG_BYTES        4
#define VL6180_RESULT__RANGE_RETURN_AMB_COUNT_REG                   0x0074
#define VL6180_RESULT__RANGE_RETURN_AMB_COUNT_REG_BYTES             4
#define VL6180_RESULT__RANGE_REFERENCE_AMB_COUNT_REG                0x0078
#define VL6180_RESULT__RANGE_REFERENCE_AMB_COUNT_REG_BYTES          4
#define VL6180_RESULT__RANGE_RETURN_CONV_TIME_REG                   0x007c
#define VL6180_RESULT__RANGE_RETURN_CONV_TIME_REG_BYTES             4
#define VL6180_RESULT__RANGE_REFERENCE_CONV_TIME_REG                0x0080
#define VL6180_RESULT__RANGE_REFERENCE_CONV_TIME_REG_BYTES          4



#define DEFINE_MSM_MUTEX(mutexname) \
	static struct mutex mutexname = __MUTEX_INITIALIZER(mutexname)

struct msm_laser_focus_ctrl_t;

enum msm_laser_focus_state_t {
	LASER_FOCUS_POWER_UP,
	LASER_FOCUS_POWER_DOWN,
};

enum msm_laser_focus_data_type {
	MSM_LASER_FOCUS_BYTE_DATA = 1,
	MSM_LASER_FOCUS_WORD_DATA,
};

enum msm_laser_focus_addr_type {
	MSM_LASER_FOCUS_BYTE_ADDR = 1,
	MSM_LASER_FOCUS_WORD_ADDR,
};

enum msm_laser_focus_atd_device_trun_on_type {
	MSM_LASER_FOCUS_DEVICE_OFF = 0,
	MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION,
	MSM_LASER_FOCUS_DEVICE_NO_APPLY_CALIBRATION,
	MSM_LASER_FOCUS_DEVICE_INIT_CCI,
	MSM_LASER_FOCUS_DEVICE_DEINIT_CCI,
};

#define MSM_LASER_FOCUS_APPLY_OFFSET_CALIBRATION		10
#define MSM_LASER_FOCUS_APPLY_CROSSTALK_CALIBRATION	40

// ****************************************************************************
#define VL6180_CROSSTALK_CAL_RANGE		400	/* 400mm */
#define VL6180_OFFSET_CAL_RANGE			100	/* 100mm */
#define STMVL6180_RUNTIMES_OFFSET_CAL		20


struct laser_focus_ctrl_t {
	//struct i2c_driver *i2c_driver;
	//struct platform_driver *pdriver;
	//struct platform_device *pdev;
	//struct msm_camera_i2c_client *i2c_client;
	//enum msm_camera_device_type_t act_device_type;
	//struct msm_sd_subdev msm_sd;
	//struct msm_camera_sensor_board_info *sensordata;
	//enum af_camera_name cam_name;
	//struct mutex *laser_focus_mutex;
	//struct msm_laser_focus_func_tbl *func_tbl;
	enum msm_laser_focus_data_type i2c_data_type;
	//struct v4l2_subdev sdev;
	//struct v4l2_subdev_ops *act_v4l2_subdev_ops;

	int16_t device_state;
	/* For calibration */
	uint16_t laser_focus_offset_value;
	uint16_t laser_focus_cross_talk_offset_value;

	//int16_t curr_step_pos;
	//uint16_t curr_region_index;
	//uint16_t *step_position_table;
	//struct region_params_t region_params[MAX_LASER_FOCUS_REGION];
	uint16_t reg_tbl_size;
	//struct msm_laser_focus_reg_params_t reg_tbl[MAX_LASER_FOCUS_REG_TBL_SIZE];
	//uint16_t region_size;
	void *user_data;
	//uint32_t total_steps;
	//uint16_t pwd_step;
	uint16_t initial_code;
	//struct msm_camera_i2c_reg_array *i2c_reg_tbl;
	//uint16_t i2c_tbl_index;
	//enum cci_i2c_master_t cci_master;
	//uint32_t subdev_id;
	enum msm_laser_focus_state_t laser_focus_state;
	//struct msm_laser_focus_vreg vreg_cfg;
	//struct park_lens_data_t park_lens;
	uint32_t max_code_size;
};
#endif
