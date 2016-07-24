/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-05
*
*/

#ifndef __LINUX_LASER_FORCUS_SENSOR_I2C_H
#define __LINUX_LASER_FORCUS_SENSOR_I2C_H

//#include "msm_cci.h" 【platform relationship】
#include "msm_laser_focus.h"

/* Write one byte via CCI i2c */
int CCI_I2C_WrByte(struct msm_laser_focus_ctrl_t *dev_t, uint32_t register_addr, uint16_t i2c_write_data);
/* Write one word via CCI i2c */
int CCI_I2C_WrWord(struct msm_laser_focus_ctrl_t *dev_t, uint32_t register_addr, uint16_t write_data);
/* Write sequence byte via CCI i2c */
int CCI_I2C_WrByteSeq(struct msm_laser_focus_ctrl_t *dev_t, uint32_t register_addr, uint8_t *i2c_write_data, uint32_t num_byte);
/* Read one byte via CCI i2c */
int CCI_I2C_RdByte(struct msm_laser_focus_ctrl_t *dev_t, uint32_t register_addr, uint16_t *i2c_read_data);
/* Read one word via CCI i2c */
int CCI_I2C_RdWord(struct msm_laser_focus_ctrl_t *dev_t, uint32_t register_addr, uint16_t *i2c_read_data);
/* Read two bytes via CCI i2c */
int CCI_I2C_RdDWord(struct msm_laser_focus_ctrl_t *dev_t, uint32_t register_addr, uint32_t *i2c_read_data);
/* Read sequence byte via CCI i2c */
int CCI_I2C_RdByteSeq(struct msm_laser_focus_ctrl_t *dev_t, uint32_t register_addr, uint8_t *i2c_read_data, uint32_t num_byte);

#endif
