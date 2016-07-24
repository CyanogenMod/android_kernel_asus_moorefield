#ifndef __OLIVIA_DEV_H__
#define __OLIVIA_DEV_H__

int olivia_i2c_read_one_byte(u8 reg,u8 *value);
int olivia_i2c_read_two_byte(u8 reg,u16 *value);
int olivia_i2c_read_four_byte(u8 reg,u32 *value);
int olivia_i2c_write_one_byte(u8 reg, u8 value);
int olivia_i2c_write_two_byte(u8 reg, u16 value);
int olivia_i2c_write_four_byte(u8 reg, u32 value);
int init_laser_controller(void);
int uninit_laser_controller(void);
int write_cali_data_to_isp(u16 *cali_data, u8 cali_crc);
int read_cali_data_to_isp(u16 *cali_data, u8 *cali_crc);

#endif
