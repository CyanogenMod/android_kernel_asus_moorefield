/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-05
*
*/
#include <linux/time.h>
#include "olivia_dev.h"

#include "laser_focus_i2c.h"
#include "show_log.h"

/** @brief Write one byte via CCI i2c
*	
*	@param dev_t the laser focus controller
*	@param register_addr the register address
*	@param i2c_write_data the data which will be written
*
*/

extern void swap_data(uint16_t* register_data);

int CCI_I2C_WrByte(struct msm_laser_focus_ctrl_t *dev_t, uint32_t register_addr, uint16_t i2c_write_data)
{
	#if 0
	int status;
	/* Setting i2c client */
	struct msm_camera_i2c_client *sensor_i2c_client;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	
	sensor_i2c_client = dev_t->i2c_client;
	if (!sensor_i2c_client) {
		LOG_Handler(LOG_ERR, "%s: failed: %p \n", __func__, sensor_i2c_client);
		return -EINVAL;
	}

	status = sensor_i2c_client->i2c_func_tbl->i2c_write(sensor_i2c_client, register_addr,
				i2c_write_data, MSM_CAMERA_I2C_BYTE_DATA);
	if (status < 0) {
		LOG_Handler(LOG_ERR, "%s: wirte register(0x%x) failed\n", __func__, register_addr);
		return status;
	}
	LOG_Handler(LOG_REG, "%s: wirte register(0x%x) : 0x%x \n", __func__, register_addr, i2c_write_data);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	
	return status;
	#else
	olivia_i2c_write_one_byte((u8)register_addr, (u8)i2c_write_data);
	return 0;
	#endif
}

/** @brief Write one word via CCI i2c
*	
*	@param dev_t the laser focus controller
*	@param register_addr the register address
*	@param i2c_write_data the data which will be written
*
*/
int CCI_I2C_WrWord(struct msm_laser_focus_ctrl_t *dev_t, uint32_t register_addr, uint16_t write_data)
{
	#if 0
	int status;
	uint16_t i2c_write_data = write_data;
	/* Setting i2c client */
	struct msm_camera_i2c_client *sensor_i2c_client;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	
	swap_data(&i2c_write_data);
	sensor_i2c_client = dev_t->i2c_client;
	if (!sensor_i2c_client) {
		LOG_Handler(LOG_ERR, "%s: failed: %p \n", __func__, sensor_i2c_client);
		return -EINVAL;
	}

	status = sensor_i2c_client->i2c_func_tbl->i2c_write(sensor_i2c_client, register_addr,
				i2c_write_data, MSM_CAMERA_I2C_WORD_DATA);
	if (status < 0) {
		LOG_Handler(LOG_ERR, "%s: wirte register(0x%x) failed\n", __func__, register_addr);
		return status;
	}
	LOG_Handler(LOG_REG, "%s: wirte register(0x%x) : 0x%x \n", __func__, register_addr, i2c_write_data);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	
	return status;
	#else
	olivia_i2c_write_two_byte((u8)register_addr, (u16)write_data);
	return 0;
	#endif
}

/** @brief Write seqence byte via CCI i2c
*	
*	@param dev_t the laser focus controller
*	@param register_addr the register address
*	@param i2c_write_data the value which will be written
*	@param num_bytes the size of write data
*
*/
int CCI_I2C_WrByteSeq(struct msm_laser_focus_ctrl_t *dev_t, uint32_t register_addr, uint8_t *i2c_write_data, uint32_t num_byte)
{
	#if 0
	int status;
	/* Setting i2c client */
	struct msm_camera_i2c_client *sensor_i2c_client;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	
	sensor_i2c_client = dev_t->i2c_client;
	if (!sensor_i2c_client) {
		LOG_Handler(LOG_ERR, "%s: failed: %p \n", __func__, sensor_i2c_client);
		return -EINVAL;
	}

	status = sensor_i2c_client->i2c_func_tbl->i2c_write_seq(sensor_i2c_client, register_addr, 
		i2c_write_data, num_byte);

	if (status < 0) {
		LOG_Handler(LOG_ERR, "%s: write register(0x%x) failed\n", __func__, register_addr);
		return status;
	}
	//LOG_Handler(LOG_REG, "%s: write register(0x%x) : 0x%x \n", __func__, register_addr, i2c_write_data);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return status;
	#else
	return 0;
	#endif
}


/** @brief Read one byte via CCI i2c
*	
*	@param dev_t the laser focus controller
*	@param register_addr the register address
*	@param i2c_write_data the variable which will be assigned read result
*
*/
int CCI_I2C_RdByte(struct msm_laser_focus_ctrl_t *dev_t, uint32_t register_addr, uint16_t *i2c_read_data)
{
	#if 0
       int status;
        
       /* Setting i2c client */
       struct msm_camera_i2c_client *sensor_i2c_client;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
		
       sensor_i2c_client = dev_t->i2c_client;
       if (!sensor_i2c_client) {
               LOG_Handler(LOG_ERR, "%s: failed: %p \n", __func__, sensor_i2c_client);
               return -EINVAL;
       }

       status = sensor_i2c_client->i2c_func_tbl->i2c_read(sensor_i2c_client, register_addr,
                       i2c_read_data, MSM_CAMERA_I2C_BYTE_DATA);
       if (status < 0) {
               LOG_Handler(LOG_ERR, "%s: read register(0x%x) failed\n", __func__, register_addr);
               return status;
       }
       LOG_Handler(LOG_REG, "%s: read register(0x%x) : 0x%x \n", __func__, register_addr, *i2c_read_data);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	
       return status;
	#else
	olivia_i2c_read_one_byte((u8)register_addr, (u8*)i2c_read_data);
	return 0;
	#endif
}

/** @brief Read one word via CCI i2c
*	
*	@param dev_t the laser focus controller
*	@param register_addr the register address
*	@param i2c_write_data the variable which will be assigned read result
*
*/
int CCI_I2C_RdWord(struct msm_laser_focus_ctrl_t *dev_t, uint32_t register_addr, uint16_t *i2c_read_data)
{
	#if 0
	int status;
	/* Setting i2c client */
	struct msm_camera_i2c_client *sensor_i2c_client;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	
	sensor_i2c_client = dev_t->i2c_client;
	if (!sensor_i2c_client) {
		LOG_Handler(LOG_ERR, "%s: failed: %p \n", __func__, sensor_i2c_client);
		return -EINVAL;
	}

	status = sensor_i2c_client->i2c_func_tbl->i2c_read(sensor_i2c_client, register_addr,
		i2c_read_data, MSM_CAMERA_I2C_WORD_DATA);
	if (status < 0) {
		LOG_Handler(LOG_ERR, "%s: read register(0x%x) failed\n", __func__, register_addr);
		return status;
	}

	swap_data(i2c_read_data);
	LOG_Handler(LOG_REG, "%s: read register(0x%x) : 0x%x \n", __func__, register_addr, *i2c_read_data);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return status;
	#else
	olivia_i2c_read_two_byte((u8)register_addr, (u16*)i2c_read_data);
	return 0;
	#endif
}

/** @brief Read two words via CCI i2c
*	
*	@param dev_t the laser focus controller
*	@param register_addr the register address
*	@param i2c_read_data the variable which will be assigned read result
*	@param num_byte number of the bytes which will be read
*
*/
int CCI_I2C_RdDWord(struct msm_laser_focus_ctrl_t *dev_t, uint32_t register_addr, uint32_t *i2c_read_data)
{
	#if 0
	int status;
	struct msm_camera_i2c_seq_reg_array reg_setting;
	
	/* Setting i2c client */
	struct msm_camera_i2c_client *sensor_i2c_client;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	
	sensor_i2c_client = dev_t->i2c_client;
	if (!sensor_i2c_client) {
		LOG_Handler(LOG_ERR, "%s: failed: %p \n", __func__, sensor_i2c_client);
		return -EINVAL;
	}

	reg_setting.reg_data_size = 4;

	status = (int)sensor_i2c_client->i2c_func_tbl->i2c_read_seq(sensor_i2c_client, register_addr, 
		reg_setting.reg_data, reg_setting.reg_data_size);
	
	if (status < 0) {
		LOG_Handler(LOG_ERR, "%s: read register(0x%x) failed\n", __func__, register_addr);
		return status;
	}
	
	*i2c_read_data=((uint32_t)reg_setting.reg_data[0]<<24)|((uint32_t)reg_setting.reg_data[1]<<16)|((uint32_t)reg_setting.reg_data[2]<<8)|((uint32_t)reg_setting.reg_data[3]);
	LOG_Handler(LOG_REG, "%s: read register(0x%x) : 0x%x \n", __func__, register_addr, *i2c_read_data);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return status;
	#else
	olivia_i2c_read_four_byte((u8)register_addr, (u32*)i2c_read_data);
	return 0;
	#endif
}

/** @brief Read seqence byte via CCI i2c
*	
*	@param dev_t the laser focus controller
*	@param register_addr the register address
*	@param i2c_read_data the variable which will be assigned read result
*	@param num_bytes the size of read data
*
*/
int CCI_I2C_RdByteSeq(struct msm_laser_focus_ctrl_t *dev_t, uint32_t register_addr, uint8_t *i2c_read_data, uint32_t num_byte)
{
	#if 0
	int status;
	uint8_t buf[num_byte];
	/* Setting i2c client */
	struct msm_camera_i2c_client *sensor_i2c_client;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	
	sensor_i2c_client = dev_t->i2c_client;
	if (!sensor_i2c_client) {
		LOG_Handler(LOG_ERR, "%s: failed: %p \n", __func__, sensor_i2c_client);
		return -EINVAL;
	}

	status = sensor_i2c_client->i2c_func_tbl->i2c_read_seq(sensor_i2c_client, register_addr, 
		buf, num_byte);
	if (status < 0) {
		LOG_Handler(LOG_ERR, "%s: read register(0x%x) failed\n", __func__, register_addr);
		return status;
	}
	LOG_Handler(LOG_REG, "%s: read register(0x%x) : 0x%x \n", __func__, register_addr, i2c_read_data);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return status;
	#else
	return 0;
	#endif
}
