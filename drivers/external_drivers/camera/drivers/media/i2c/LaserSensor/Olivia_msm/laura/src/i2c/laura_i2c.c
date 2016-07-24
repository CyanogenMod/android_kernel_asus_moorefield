/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-08
*
*/

#include "laura_i2c.h"
#include "show_log.h"

#if 0
/** @brief laura indirect address read
*
*       @param dev_t the laser focus controller
*
*/
int Laura_device_indirect_addr_read(struct msm_laser_focus_ctrl_t *dev_t, uint16_t *i2c_read_data, uint32_t num_word){
       int status = 0;
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
       return status;
}
#endif

/** @brief laura indirect address write
*
*	@param dev_t the laser focus controller
*	@param i_reg_addr_lsb the lsb register address which store lsb indirect address
*	@param i_reg_addr_msb the msb register address which store msb indirect address
*	@param indirect_addr the indirect address
*	@param register_addr the register address which store data
*	@param i2c_write_data the write data
*	@param num_word the size of write data
*
*/
int Laura_device_indirect_addr_write(struct msm_laser_focus_ctrl_t *dev_t,
										uint16_t i_reg_addr_lsb, uint16_t i_reg_addr_msb, uint16_t indirect_addr, 
										uint32_t register_addr, uint16_t *i2c_write_data, uint32_t num_word){
	int status;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	status = Laura_write_seq(i_reg_addr_lsb, i_reg_addr_msb, indirect_addr, register_addr, i2c_write_data, num_word);
	if (status < 0) {
		LOG_Handler(LOG_ERR, "%s: write register(0x%x) failed\n", __func__, register_addr);
		return status;
	}

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return status;
}

/** @brief laura write sequence bytes for indirect address
*
*     @param client
*	@param i_reg_addr_lsb the lsb register address which store lsb indirect address
*	@param i_reg_addr_msb the msb register address which store msb indirect address
*	@param indirect_addr the indirect address
*	@param addr the register address which store data
*	@param data the write data
*	@param num_word the size of write data
*
*/
int32_t Laura_write_seq(
							uint16_t i_reg_addr_lsb, uint16_t i_reg_addr_msb, uint16_t indirect_addr, 
							uint32_t reg_addr, uint16_t *data, uint32_t num_word)
{
	#if 0
	int32_t rc = -EFAULT;
	uint8_t i = 0, j = 0;
	int reg_conf_tbl_size = (num_word*2)+2; 
	struct msm_camera_cci_ctrl cci_ctrl;
	struct msm_camera_i2c_reg_array reg_conf_tbl[reg_conf_tbl_size];

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	if ((client->addr_type != MSM_CAMERA_I2C_BYTE_ADDR
		&& client->addr_type != MSM_CAMERA_I2C_WORD_ADDR)
		|| num_word == 0)
		return rc;

	LOG_Handler(LOG_DBG, "%s reg addr = 0x%x num bytes: %d\n", __func__, reg_addr, reg_conf_tbl_size);
	
	memset(reg_conf_tbl, 0, (reg_conf_tbl_size) * sizeof(struct msm_camera_i2c_reg_array));

	/* Assing indirect address LSB byte */
	reg_conf_tbl[0].reg_addr = i_reg_addr_lsb;
	reg_conf_tbl[0].reg_data = (indirect_addr&0xFF00)>>8;
	reg_conf_tbl[0].delay = 0;
	/* Assing indirect address MSB byte */
	reg_conf_tbl[1].reg_addr = i_reg_addr_msb;
	reg_conf_tbl[1].reg_data = indirect_addr&0x00FF;
	reg_conf_tbl[1].delay = 0;
	
	if (reg_conf_tbl_size > I2C_SEQ_REG_DATA_MAX) {
		LOG_Handler(LOG_ERR, "%s: num bytes=%d clamped to max supported %d\n", __func__, reg_conf_tbl_size, I2C_SEQ_REG_DATA_MAX);
		reg_conf_tbl_size = I2C_SEQ_REG_DATA_MAX;
	}
	for (i = 2; i < reg_conf_tbl_size; i = i+2) {
		/* Assign data LSB byte */
		reg_conf_tbl[i].reg_addr = reg_addr;
		reg_conf_tbl[i].reg_data = (data[j]&0xFF00)>>8;
		reg_conf_tbl[i].delay = 0;
		/* Assign data MSB byte */
		reg_conf_tbl[i+1].reg_addr = reg_addr;
		reg_conf_tbl[i+1].reg_data = data[j]&0x00FF;
		reg_conf_tbl[i+1].delay = 0;
		j++;
	}
	
	cci_ctrl.cmd = MSM_CCI_I2C_WRITE;
	cci_ctrl.cci_info = client->cci_client;
	cci_ctrl.cfg.cci_i2c_write_cfg.reg_setting = reg_conf_tbl;
	cci_ctrl.cfg.cci_i2c_write_cfg.data_type = MSM_CAMERA_I2C_BYTE_DATA;
	cci_ctrl.cfg.cci_i2c_write_cfg.addr_type = client->addr_type;
	cci_ctrl.cfg.cci_i2c_write_cfg.size = reg_conf_tbl_size;
	rc = v4l2_subdev_call(client->cci_client->cci_subdev, core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl);
	
	rc = cci_ctrl.status;

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	
	return rc;
	#else
	uint8_t i = 0;


	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/* Check patch memory write */
	(void) CCI_I2C_WrByte(0, i_reg_addr_lsb, indirect_addr >> 8);
	(void) CCI_I2C_WrByte(0, i_reg_addr_msb, indirect_addr & 0xFF);

	for (i = 0; i < num_word; i++) {
		CCI_I2C_WrWord(0, I2C_DATA_PORT, data[i]);
	}


	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return 0;
	#endif
}

