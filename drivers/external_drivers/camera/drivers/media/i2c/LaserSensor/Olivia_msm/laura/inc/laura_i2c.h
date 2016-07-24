/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-08
*
*/

#ifndef __LINUX_SHOW_LAURA_SENSOR_I2C_H
#define __LINUX_SHOW_LAURA_SENSOR_I2C_H

#include "msm_laser_focus.h"
#include "laura_shipping_func.h"
#include "laura_factory_func.h"

#if 0
/* Laura indirect address read  */
int Laura_device_indirect_addr_read(struct msm_laser_focus_ctrl_t *dev_t, uint16_t *i2c_read_data, uint32_t num_word);
#endif
/* Laura indirect address write */
int Laura_device_indirect_addr_write(struct msm_laser_focus_ctrl_t *dev_t,
										uint16_t i_reg_addr_lsb, uint16_t i_reg_addr_msb, uint16_t indirect_addr, 
										uint32_t register_addr, uint16_t *i2c_write_data, uint32_t num_word);
/* Laura write sequence bytes for indirect address */
int32_t Laura_write_seq(
										uint16_t i_reg_addr_lsb, uint16_t i_reg_addr_msb, uint16_t indirect_addr, 
										uint32_t reg_addr, uint16_t *data, uint32_t num_word);
#endif
