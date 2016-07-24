#include <linux/time.h>
#include <linux/m12mo_workaround.h>
#include <linux/m12mo.h>
#include "show_timer.h"
#include "show_log.h"
#include "laura_factory_func.h"

#define M12MO_MODE_ADDRESS              0x0b
#define M12MO_MODE_CATGORY              0x00

#define OLIVIA_I2C_DEVICE_ADDRESS       0x4c
#define M12MO_I2C_ROUTER_CATGORY        0x0e

#define M12MO_I2C_ROUTER_DEVICE_ADDR    0x40
#define M12MO_I2C_ROUTER_REG_ADDR       0x42
#define M12MO_I2C_ROUTER_TRIGGER_CMD    0x47
#define M12MO_LS_1BYTE_READ             0x01
#define M12MO_LS_2BYTE_READ             0x02
#define M12MO_LS_4BYTE_READ             0x04
#define M12MO_LS_1BYTE_WRITE            0x11
#define M12MO_LS_2BYTE_WRITE            0x12
#define M12MO_LS_4BYTE_WRITE            0x14

#define M12MO_I2C_ROUTER_DATA1          0x43
#define M12MO_I2C_ROUTER_DATA2          0x44
#define M12MO_I2C_ROUTER_DATA3          0x45
#define M12MO_I2C_ROUTER_DATA4          0x46
#define M12MO_I2C_ROUTER_STATUS         0x48
#define M12MO_I2C_REASON_OK             0x00
#define M12MO_I2C_REASON_NG             0xff
#define M12MO_I2C_REASON_BUSY           0x01
#define M12MO_I2C_REASON_REJECT         0x02

#define M12MO_WRITE_FLASH_CATGORY       0x0d
#define M12MO_FADJ_FLASH_MODE           0xa5
#define M12MO_Read_ISP_Flash            0x01
#define M12MO_Write_ISP_SDRAM           0x03
#define M12MO_Erase_ISP_Flash           0x05

#define M12MO_FADJ_RW_MODE              0xa6
#define M12MO_FADJ_1BYTE_READ           0x01
#define M12MO_FADJ_2BYTE_READ           0x02
#define M12MO_FADJ_4BYTE_READ           0x04
#define M12MO_FADJ_1BYTE_WRITE          0x09
#define M12MO_FADJ_2BYTE_WRITE          0x0a
#define M12MO_FADJ_4BYTE_WRITE          0x0c

#define M12MO_FADJ_RW_DATA              0xa9
#define M12MO_FADJ_RW_OFFSET            0xa7


#define M12MO_SFLASH_SPI_STATUS         0xae
#define M12MO_SFLASH_SPI_STATUS_OK      0x00
#define M12MO_SFLASH_SPI_STATUS_ERROR   0x01
#define M12MO_SFLASH_SPI_STATUS_BUSY    0xff

#define M12MO_FLASH_CALI_DATA_ADDR      0x0022
#define M12MO_FLASH_CALI_CRC_ADDR       0x0068

static int wait_read_or_write_finish(void)
{
	u32 val;
	int ret;
	struct timeval start;
	struct timeval now;

	start = get_current_time();
	while(1)
	{
		if((ret = m12mo_read_fac(1,M12MO_I2C_ROUTER_CATGORY,M12MO_I2C_ROUTER_STATUS,&val)) != 0)
		{
			LOG_Handler(LOG_ERR,"i2c read status fail,ret=%d\n",ret);
			return ret;
		}
		if(val == M12MO_I2C_REASON_OK)
			break;
		else if(val == M12MO_I2C_REASON_BUSY)
		{
			now = get_current_time();
			if(is_timeout(start,now,100) == true)
			{
				LOG_Handler(LOG_ERR,"i2c read read status timeout\n");
				return -1;
			}
		}
		else
		{
			LOG_Handler(LOG_ERR,"i2c read status error\n");
			return -1;
		}
	}
	return 0;
}

int olivia_i2c_read_one_byte(u8 reg,u8 *value)
{
	u32 val;
	int ret;

	//set i2c devices address
	val = OLIVIA_I2C_DEVICE_ADDRESS;
	if((ret = m12mo_write_fac(1 ,M12MO_I2C_ROUTER_CATGORY ,M12MO_I2C_ROUTER_DEVICE_ADDR ,val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[RB]set i2c device address fail,ret=%d\n",ret);
		return ret;
	}

	//set i2c reg address
	val = reg;
	if((ret = m12mo_write_fac(1 ,M12MO_I2C_ROUTER_CATGORY ,M12MO_I2C_ROUTER_REG_ADDR ,val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[RB]set i2c reg address fail,ret=%d\n",ret);
		return ret;
	}

	//trig send cmd
	val = M12MO_LS_1BYTE_READ;
	if((ret = m12mo_write_fac(1 ,M12MO_I2C_ROUTER_CATGORY ,M12MO_I2C_ROUTER_TRIGGER_CMD ,val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[RB]trig i2c read data fail,ret=%d\n",ret);
		return ret;
	}

	if(wait_read_or_write_finish() != 0)
	{
		LOG_Handler(LOG_ERR,"[RB]wait cmd finish error\n");
		return -1;
	}

	//read data
	if((ret = m12mo_read_fac(1,M12MO_I2C_ROUTER_CATGORY,M12MO_I2C_ROUTER_DATA4,&val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[RB]i2c read fail,ret=%d\n",ret);
		return ret;
	}

	*value = val;
	LOG_Handler(LOG_REG,"[RB]read reg:0x%02x, data:0x%02x\n", reg, *value);

	return 0;
}

int olivia_i2c_read_two_byte(u8 reg, u16 *value)
{
	u32 val;
	int ret;

	//set i2c devices address
	val = OLIVIA_I2C_DEVICE_ADDRESS;
	if((ret = m12mo_write_fac(1 ,M12MO_I2C_ROUTER_CATGORY ,M12MO_I2C_ROUTER_DEVICE_ADDR ,val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[RW]set i2c device address fail,ret=%d\n",ret);
		return ret;
	}

	//set i2c reg address
	val = reg;
	if((ret = m12mo_write_fac(1 ,M12MO_I2C_ROUTER_CATGORY ,M12MO_I2C_ROUTER_REG_ADDR ,val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[RW]set i2c reg address fail,ret=%d\n",ret);
		return ret;
	}

	//trig send cmd
	val = M12MO_LS_2BYTE_READ;
	if((ret = m12mo_write_fac(1 ,M12MO_I2C_ROUTER_CATGORY ,M12MO_I2C_ROUTER_TRIGGER_CMD ,val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[RW]trig i2c read data fail,ret=%d\n",ret);
		return ret;
	}

	if(wait_read_or_write_finish() != 0)
	{
		LOG_Handler(LOG_ERR,"[RW]wait cmd finish error\n");
		return -1;
	}

	//read data
	if((ret = m12mo_read_fac(2,M12MO_I2C_ROUTER_CATGORY,M12MO_I2C_ROUTER_DATA3,&val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[RW]i2c read fail,ret=%d\n",ret);
		return ret;
	}

	*value = ((val & 0xff00) >> 8) | ((val & 0xff) << 8);
	LOG_Handler(LOG_REG,"[RW]read reg:0x%02x, data:0x%04x\n", reg, *value);

	return 0;
}

int olivia_i2c_read_four_byte(u8 reg, u32 *value)
{
	u32 val;
	int ret;

	//set i2c devices address
	val = OLIVIA_I2C_DEVICE_ADDRESS;
	if((ret = m12mo_write_fac(1 ,M12MO_I2C_ROUTER_CATGORY ,M12MO_I2C_ROUTER_DEVICE_ADDR ,val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[RDW]set i2c device address fail,ret=%d\n",ret);
		return ret;
	}

	//set i2c reg address
	val = reg;
	if((ret = m12mo_write_fac(1 ,M12MO_I2C_ROUTER_CATGORY ,M12MO_I2C_ROUTER_REG_ADDR ,val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[RDW]set i2c reg address fail,ret=%d\n",ret);
		return ret;
	}

	//trig send cmd
	val = M12MO_LS_4BYTE_READ;
	if((ret = m12mo_write_fac(1 ,M12MO_I2C_ROUTER_CATGORY ,M12MO_I2C_ROUTER_TRIGGER_CMD ,val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[RDW]trig i2c read data fail,ret=%d\n",ret);
		return ret;
	}

	if(wait_read_or_write_finish() != 0)
	{
		LOG_Handler(LOG_ERR,"[RDW]wait cmd finish error\n");
		return -1;
	}

	//read data
	if((ret = m12mo_read_fac(4,M12MO_I2C_ROUTER_CATGORY,M12MO_I2C_ROUTER_DATA1,&val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[RDW]i2c read fail,ret=%d\n",ret);
		return ret;
	}

	*value = ((val & 0xff000000) >> 24) | ((val & 0xff0000) >> 8) | ((val & 0xff00) << 8) | ((val & 0xff) << 24);
	LOG_Handler(LOG_REG,"[RDW]read reg:0x%02x, data:0x%08x\n", reg, *value);

	return 0;
}

int olivia_i2c_write_one_byte(u8 reg,u8 value)
{
	u32 val;
	int ret;

	//set i2c devices address
	val = OLIVIA_I2C_DEVICE_ADDRESS;
	if((ret = m12mo_write_fac(1 ,M12MO_I2C_ROUTER_CATGORY ,M12MO_I2C_ROUTER_DEVICE_ADDR ,val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[WB]set i2c device address fail,ret=%d\n",ret);
		return ret;
	}

	//set i2c reg address
	val = reg;
	if((ret = m12mo_write_fac(1 ,M12MO_I2C_ROUTER_CATGORY ,M12MO_I2C_ROUTER_REG_ADDR ,val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[WB]set i2c reg address fail,ret=%d\n",ret);
		return ret;
	}

	//write data
	val = value;
	if((ret = m12mo_write_fac(1,M12MO_I2C_ROUTER_CATGORY, M12MO_I2C_ROUTER_DATA1 ,val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[WB]set i2c write data fail,ret=%d\n",ret);
		return ret;
	}

	//trig send cmd
	val = M12MO_LS_1BYTE_WRITE;
	if((ret = m12mo_write_fac(1 ,M12MO_I2C_ROUTER_CATGORY ,M12MO_I2C_ROUTER_TRIGGER_CMD ,val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[WB]trig i2c write data fail,ret=%d\n",ret);
		return ret;
	}

	if(wait_read_or_write_finish() != 0)
	{
		LOG_Handler(LOG_ERR,"[WB]wait cmd finish error\n");
		return -1;
	}

	LOG_Handler(LOG_REG,"[WB]write reg:0x%02x, data:0x%02x\n", reg, value);

	return 0;
}


int olivia_i2c_write_two_byte(u8 reg, u16 value)
{
	u32 val;
	int ret;

	//set i2c devices address
	val = OLIVIA_I2C_DEVICE_ADDRESS;
	if((ret = m12mo_write_fac(1 ,M12MO_I2C_ROUTER_CATGORY ,M12MO_I2C_ROUTER_DEVICE_ADDR ,val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[WW]set i2c device address fail,ret=%d\n",ret);
		return ret;
	}

	//set i2c reg address
	val = reg;
	if((ret = m12mo_write_fac(1 ,M12MO_I2C_ROUTER_CATGORY ,M12MO_I2C_ROUTER_REG_ADDR ,val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[WW]set i2c reg address fail,ret=%d\n",ret);
		return ret;
	}

	//write data
	val = ((value & 0xff) << 8) | ((value & 0xff00) >> 8);
	if((ret = m12mo_write_fac(2,M12MO_I2C_ROUTER_CATGORY,M12MO_I2C_ROUTER_DATA1 , val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[WW]set i2c write data fail,ret=%d\n",ret);
		return ret;
	}

	//trig send cmd
	val = M12MO_LS_2BYTE_WRITE;
	if((ret = m12mo_write_fac(1 ,M12MO_I2C_ROUTER_CATGORY ,M12MO_I2C_ROUTER_TRIGGER_CMD ,val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[WW]trig i2c write data fail,ret=%d\n",ret);
		return ret;
	}

	if(wait_read_or_write_finish() != 0)
	{
		LOG_Handler(LOG_ERR,"[WW]wait cmd finish error\n");
		return -1;
	}
	LOG_Handler(LOG_REG,"[WW]write reg:0x%02x, data:0x%04x\n", reg, value);

	return 0;
}

int olivia_i2c_write_four_byte(u8 reg, u32 value)
{
	u32 val;
	int ret;

	//set i2c devices address
	val = OLIVIA_I2C_DEVICE_ADDRESS;
	if((ret = m12mo_write_fac(1 ,M12MO_I2C_ROUTER_CATGORY ,M12MO_I2C_ROUTER_DEVICE_ADDR ,val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[WDW]set i2c device address fail,ret=%d\n",ret);
		return ret;
	}

	//set i2c reg address
	val = reg;
	if((ret = m12mo_write_fac(1 ,M12MO_I2C_ROUTER_CATGORY ,M12MO_I2C_ROUTER_REG_ADDR ,val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[WDW]set i2c reg address fail,ret=%d\n",ret);
		return ret;
	}

	//write data
	val = ((value & 0xff000000) >> 24) | ((value & 0xff0000) >> 8) | ((value & 0xff00) << 8) | ((value & 0xff) << 24);
	if((ret = m12mo_write_fac(4,M12MO_I2C_ROUTER_CATGORY,M12MO_I2C_ROUTER_DATA1 , val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[WDW]set i2c write data fail,ret=%d\n",ret);
		return ret;
	}

	//trig send cmd
	val = M12MO_LS_4BYTE_WRITE;
	if((ret = m12mo_write_fac(1 ,M12MO_I2C_ROUTER_CATGORY ,M12MO_I2C_ROUTER_TRIGGER_CMD ,val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[WDW]trig i2c write data fail,ret=%d\n",ret);
		return ret;
	}

	if(wait_read_or_write_finish() != 0)
	{
		LOG_Handler(LOG_ERR,"[WDW]wait cmd finish error\n");
		return -1;
	}
	LOG_Handler(LOG_REG,"[WDW]write reg:0x%02x, data:0x%08x\n", reg, value);

	return 0;
}

int init_laser_controller(void)
{
	u32 val;
	int ret;
        m12mo_USB_status(1);
	if(m12mo_status_fac() == 0)
	{
		if((ret = m12mo_s_power_fac(1)) != 0)
		{
			LOG_Handler(LOG_ERR,"m12mo power on fail,ret=%d\n",ret);
			return -1;
		}
		LOG_Handler(LOG_REG,"m12mo power on success\n");
	}
	else
	{
		LOG_Handler(LOG_REG,"m12mo power already on,nothing to do\n");
	}
	if((ret = m12mo_read_fac(1,M12MO_MODE_CATGORY,M12MO_MODE_ADDRESS,&val)) != 0)
	{
		LOG_Handler(LOG_ERR,"read m12mo mode fail,ret=%d\n",ret);
		return ret;
	}
	if(val != 0x01)
	{
		val = 0x01;
		if((ret = m12mo_write_fac(1,M12MO_MODE_CATGORY,M12MO_MODE_ADDRESS,val)) != 0)
		{
			LOG_Handler(LOG_ERR,"write m12mo mode register fail,ret=%d\n",ret);
			return ret;
		}
		if((ret = m12mo_read_fac(1,M12MO_MODE_CATGORY,M12MO_MODE_ADDRESS,&val)) != 0)
		{
			LOG_Handler(LOG_ERR,"write m12mo mode register fail,ret=%d\n",ret);
			return ret;
		}
		if(val != 0x01)
		{
			LOG_Handler(LOG_ERR,"change m12mo mode fail,reg=%d\n",val);
			return -1;
		}
	}
	return 0;
}

int uninit_laser_controller(void)
{
	int ret;
	if((ret = m12mo_s_power_fac(0)) != 0)
	{
		LOG_Handler(LOG_ERR,"m12mo power off fail,ret=%d\n",ret);
		return -1;
	}
        m12mo_USB_status(0);
	LOG_Handler(LOG_REG,"m12mo power off fail,ret=%d\n",ret);
	return 0;
}

static int wait_write_or_read_flash(void) {
	u32 val;
	int ret;
	struct timeval start;
	struct timeval now;

	//wait finish
	LOG_Handler(LOG_REG,"start wait ok\n");
	start = get_current_time();
	while(1)
	{
		if((ret = m12mo_read_fac(1,M12MO_WRITE_FLASH_CATGORY,M12MO_SFLASH_SPI_STATUS,&val)) != 0)
		{
			LOG_Handler(LOG_ERR,"write_flash ic status byte fail,ret=%d\n",ret);
			return ret;
		}
		if(val == M12MO_SFLASH_SPI_STATUS_OK)
			break;
		else if(val == M12MO_SFLASH_SPI_STATUS_BUSY)
		{
			now = get_current_time();
			if(is_timeout(start,now,500) == true)
			{
				LOG_Handler(LOG_ERR,"write_flash ic read byte timeout\n");
				return -1;
			}
		}
		else
		{
			LOG_Handler(LOG_ERR,"write_flash ic status error\n");
			return -1;
		}
	}
	return 0;
}

static int write_cali_data(u16 *cali_data)
{
	int i;
	u16 addr;
	int ret;
	u32 val;
	u16 check_data[SIZE_OF_OLIVIA_CALIBRATION_DATA];
	for(i=0;i<SIZE_OF_OLIVIA_CALIBRATION_DATA;i++) {
		check_data[i] = cali_data[i];
	}
	i = 0;
	addr = M12MO_FLASH_CALI_DATA_ADDR;
writeloop:
	val = addr;//maybe this need change.
	if((ret = m12mo_write_fac(2,M12MO_WRITE_FLASH_CATGORY,M12MO_FADJ_RW_OFFSET,val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[write_cali_data_to_isp]write M12MO_FADJ_RW_OFFSET register fail,ret=%d\n",ret);
		return ret;
	}
	val = cali_data[i];//maybe this need change.
	if((ret = m12mo_write_fac(2,M12MO_WRITE_FLASH_CATGORY,M12MO_FADJ_RW_DATA,val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[write_cali_data_to_isp]write M12MO_FADJ_RW_DATA register fail,ret=%d\n",ret);
		return ret;
	}
	val = M12MO_FADJ_2BYTE_WRITE;
	if((ret = m12mo_write_fac(1,M12MO_WRITE_FLASH_CATGORY,M12MO_FADJ_RW_MODE,val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[write_cali_data_to_isp]write M12MO_FADJ_RW_MODE register fail,ret=%d\n",ret);
		return ret;
	}
	val = M12MO_Write_ISP_SDRAM;
	if((ret = m12mo_write_fac(1,M12MO_WRITE_FLASH_CATGORY,M12MO_FADJ_FLASH_MODE,val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[write_cali_data_to_isp]write M12MO_FADJ_FLASH_MODE register fail,ret=%d\n",ret);
		return ret;
	}
	if(wait_write_or_read_flash() != 0)
	{
		LOG_Handler(LOG_ERR,"[write_cali_data_to_isp]wait write flash finish error\n");
		return -1;
	}
	i++;
	addr += 2;
	if(i != SIZE_OF_OLIVIA_CALIBRATION_DATA)
		goto writeloop;
	i = 0;
	addr = M12MO_FLASH_CALI_DATA_ADDR;
checkloop:
	val = addr;//maybe this need change.
	if((ret = m12mo_write_fac(2,M12MO_WRITE_FLASH_CATGORY,M12MO_FADJ_RW_OFFSET,val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[write_cali_data_to_isp]write M12MO_FADJ_RW_OFFSET register fail,ret=%d\n",ret);
		return ret;
	}
	val = M12MO_FADJ_2BYTE_READ;
	if((ret = m12mo_write_fac(1,M12MO_WRITE_FLASH_CATGORY,M12MO_FADJ_RW_MODE,val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[write_cali_data_to_isp]write M12MO_FADJ_RW_MODE register fail,ret=%d\n",ret);
		return ret;
	}
	val = M12MO_Read_ISP_Flash;
	if((ret = m12mo_write_fac(1,M12MO_WRITE_FLASH_CATGORY,M12MO_FADJ_FLASH_MODE,val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[write_cali_data_to_isp]write M12MO_FADJ_FLASH_MODE register fail,ret=%d\n",ret);
		return ret;
	}
	if(wait_write_or_read_flash() != 0)
	{
		LOG_Handler(LOG_ERR,"[write_cali_data_to_isp]wait write flash finish error\n");
		return -1;
	}
	if((ret = m12mo_read_fac(2,M12MO_WRITE_FLASH_CATGORY,M12MO_FADJ_RW_DATA,&val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[write_cali_data_to_isp]write M12MO_FADJ_RW_MODE register fail,ret=%d\n",ret);
		return ret;
	}
	if(val != check_data[i])//maybe this need change
	{
		LOG_Handler(LOG_ERR,"[write_cali_data_to_isp]check fail.,val=%d,check_data[i]=%d\n",val,check_data[i]);
		return -1;
	}
	i++;
	addr += 2;
	if(i != SIZE_OF_OLIVIA_CALIBRATION_DATA)
		goto checkloop;
	return 0;
}

static int write_cali_crc(u8 cali_crc)
{
	u8 check_crc;
	u32 val;
	int ret;
	check_crc = cali_crc;
//writecrc
	val = M12MO_FLASH_CALI_CRC_ADDR;
	if((ret = m12mo_write_fac(2,M12MO_WRITE_FLASH_CATGORY,M12MO_FADJ_RW_OFFSET,val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[write_cali_crc]write M12MO_FADJ_RW_OFFSET register fail,ret=%d\n",ret);
		return ret;
	}
	val = cali_crc;
	if((ret = m12mo_write_fac(1,M12MO_WRITE_FLASH_CATGORY,M12MO_FADJ_RW_DATA,val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[write_cali_crc]write M12MO_FADJ_RW_DATA register fail,ret=%d\n",ret);
		return ret;
	}
	val = M12MO_FADJ_1BYTE_WRITE;
	if((ret = m12mo_write_fac(1,M12MO_WRITE_FLASH_CATGORY,M12MO_FADJ_RW_MODE,val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[write_cali_crc]write M12MO_FADJ_RW_MODE register fail,ret=%d\n",ret);
		return ret;
	}
	val = M12MO_Write_ISP_SDRAM;
	if((ret = m12mo_write_fac(1,M12MO_WRITE_FLASH_CATGORY,M12MO_FADJ_FLASH_MODE,val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[write_cali_crc]write M12MO_FADJ_FLASH_MODE register fail,ret=%d\n",ret);
		return ret;
	}
	if(wait_write_or_read_flash() != 0)
	{
		LOG_Handler(LOG_ERR,"[write_cali_crc]wait write flash finish error\n");
		return -1;
	}
//checkcrc
	val = M12MO_FLASH_CALI_CRC_ADDR;
	if((ret = m12mo_write_fac(2,M12MO_WRITE_FLASH_CATGORY,M12MO_FADJ_RW_OFFSET,val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[write_cali_crc]write M12MO_FADJ_RW_OFFSET register fail,ret=%d\n",ret);
		return ret;
	}
	val = M12MO_FADJ_1BYTE_READ;
	if((ret = m12mo_write_fac(1,M12MO_WRITE_FLASH_CATGORY,M12MO_FADJ_RW_MODE,val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[write_cali_crc]write M12MO_FADJ_RW_MODE register fail,ret=%d\n",ret);
		return ret;
	}
	val = M12MO_Read_ISP_Flash;
	if((ret = m12mo_write_fac(1,M12MO_WRITE_FLASH_CATGORY,M12MO_FADJ_FLASH_MODE,val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[write_cali_crc]write M12MO_FADJ_FLASH_MODE register fail,ret=%d\n",ret);
		return ret;
	}
	if(wait_write_or_read_flash() != 0)
	{
		LOG_Handler(LOG_ERR,"[write_cali_crc]wait write flash finish error\n");
		return -1;
	}
	if((ret = m12mo_read_fac(1,M12MO_WRITE_FLASH_CATGORY,M12MO_FADJ_RW_DATA,&val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[write_cali_crc]write M12MO_FADJ_RW_DATA register fail,ret=%d\n",ret);
		return ret;
	}
	if(val != check_crc)
	{
		LOG_Handler(LOG_ERR,"[write_cali_crc]check fail.,val=%d,check_crc=%d\n",val,check_crc);
		return -1;
	}
	return 0;
}

int write_cali_data_to_isp(u16 *cali_data, u8 cali_crc)
{
	int ret;
	if((ret = write_cali_data(cali_data)) != 0)
	{
		LOG_Handler(LOG_ERR,"[olivia][write_cali_data_to_isp]write_cali_data fail,ret=%d\n",ret);
		return ret;
	}
	if((ret = write_cali_crc(cali_crc)) != 0)
	{
		LOG_Handler(LOG_ERR,"[olivia][write_cali_data_to_isp]write_cali_crc fail,ret=%d\n",ret);
		return ret;
	}
	return 0;
}

static int read_cali_data(u16 *cali_data)
{
	int i;
	u16 addr;
	int ret;
	u32 val;
	i = 0;
	addr = M12MO_FLASH_CALI_DATA_ADDR;
readloop:
	val = addr;//maybe this need change.
	if((ret = m12mo_write_fac(2,M12MO_WRITE_FLASH_CATGORY,M12MO_FADJ_RW_OFFSET,val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[olivia][read_cali_data]write M12MO_FADJ_RW_OFFSET register fail,ret=%d\n",ret);
		return ret;
	}
	val = M12MO_FADJ_2BYTE_READ;
	if((ret = m12mo_write_fac(1,M12MO_WRITE_FLASH_CATGORY,M12MO_FADJ_RW_MODE,val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[olivia][read_cali_data]write M12MO_FADJ_RW_MODE register fail,ret=%d\n",ret);
		return ret;
	}
	val = M12MO_Read_ISP_Flash;
	if((ret = m12mo_write_fac(1,M12MO_WRITE_FLASH_CATGORY,M12MO_FADJ_FLASH_MODE,val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[olivia][read_cali_data]write M12MO_FADJ_FLASH_MODE register fail,ret=%d\n",ret);
		return ret;
	}
	if(wait_write_or_read_flash() != 0)
	{
		LOG_Handler(LOG_ERR,"[olivia][read_cali_data]wait read flash finish error\n");
		return -1;
	}
	if((ret = m12mo_read_fac(2,M12MO_WRITE_FLASH_CATGORY,M12MO_FADJ_RW_DATA,&val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[olivia][read_cali_data]read M12MO_FADJ_RW_MODE register fail,ret=%d\n",ret);
		return ret;
	}
	cali_data[i] = val;
	i++;
	addr += 2;
	if(i != SIZE_OF_OLIVIA_CALIBRATION_DATA)
		goto readloop;
	return 0;
}

static int read_cali_crc(u8 *cali_crc)
{
	u32 val;
	int ret;
	val = M12MO_FLASH_CALI_CRC_ADDR;
	if((ret = m12mo_write_fac(2,M12MO_WRITE_FLASH_CATGORY,M12MO_FADJ_RW_OFFSET,val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[olivia][read_cali_crc]write M12MO_FADJ_RW_OFFSET register fail,ret=%d\n",ret);
		return ret;
	}
	val = M12MO_FADJ_1BYTE_READ;
	if((ret = m12mo_write_fac(1,M12MO_WRITE_FLASH_CATGORY,M12MO_FADJ_RW_MODE,val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[olivia][read_cali_crc]write M12MO_FADJ_RW_MODE register fail,ret=%d\n",ret);
		return ret;
	}
	val = M12MO_Read_ISP_Flash;
	if((ret = m12mo_write_fac(1,M12MO_WRITE_FLASH_CATGORY,M12MO_FADJ_FLASH_MODE,val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[olivia][read_cali_crc]write M12MO_FADJ_FLASH_MODE register fail,ret=%d\n",ret);
		return ret;
	}
	if(wait_write_or_read_flash() != 0)
	{
		LOG_Handler(LOG_ERR,"[olivia][read_cali_crc]wait read flash finish error\n");
		return -1;
	}
	if((ret = m12mo_read_fac(1,M12MO_WRITE_FLASH_CATGORY,M12MO_FADJ_RW_DATA,&val)) != 0)
	{
		LOG_Handler(LOG_ERR,"[olivia][read_cali_crc]read M12MO_FADJ_RW_DATA register fail,ret=%d\n",ret);
		return ret;
	}
	*cali_crc = val;
	return 0;
}

int read_cali_data_to_isp(u16 *cali_data, u8 *cali_crc)
{
	int ret;
	if((ret = read_cali_data(cali_data)) != 0)
	{
		LOG_Handler(LOG_ERR,"[olivia][read_cali_data_to_isp]read_cali_data fail,ret=%d\n",ret);
		return ret;
	}
	if((ret = read_cali_crc(cali_crc)) != 0)
	{
		LOG_Handler(LOG_ERR,"[olivia][read_cali_data_to_isp]read_cali_crc fail,ret=%d\n",ret);
		return ret;
	}
	return 0;
}
