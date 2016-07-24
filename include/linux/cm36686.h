/* include/linux/cm36686.h
 *
 * Copyright (C) 2015 ASUS,
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __LINUX_CM36686_H
#define __LINUX_CM36686_H

#include <linux/bitops.h>
#include <linux/types.h>
#include <linux/ioctl.h>


#define CM36686_I2C_NAME "cm36686"

#define LS_PWR_ON					(1 << 0)
#define PS_PWR_ON					(1 << 1)

#define ASUS_PSENSOR_IOCTL_CLOSE         _IO(CAPELLA_CM36686_IOCTL_MAGIC, 0x12)
#define ASUS_PSENSOR_IOCTL_START         _IO(CAPELLA_CM36686_IOCTL_MAGIC, 0x13)
#define ASUS_PSENSOR_IOCTL_GETDATA       _IOR(CAPELLA_CM36686_IOCTL_MAGIC, 0x14, char[PBUFF_SIZE+1])

#define ASUS_PSENSOR_SETCALI_DATA        _IOW(CAPELLA_CM36686_IOCTL_MAGIC, 0x15, int[2])
#define ASUS_PSENSOR_EN_CALIBRATION      _IOW(CAPELLA_CM36686_IOCTL_MAGIC, 0x16, char)

#define PBUFF_SIZE				16	/* Rx buffer size */

/* Define Slave Address*/
#define	CM36686_slave_addr	0x60

#define ALS_CALIBRATED		0x6E9F
#define PS_CALIBRATED		0x509F

/*Define Command Code*/
#define		ALS_CONF	  0x00
#define		ALS_THDH  	  0x01
#define		ALS_THDL	  0x02
#define		PS_CONF1      0x03
#define		PS_CONF3      0x04
#define		PS_CANC       0x05
#define		PS_THD        0x06
#define		RESERVED      0x07
#define		PS_THDL       0x06
#define		PS_THDH       0x07

#define		PS_DATA       0x08
#define		ALS_DATA      0x09
#define		RESERVED2     0x0A
#define		INT_FLAG      0x0B
#define		ID_REG        0x0C

/*cm36686*/
/*for ALS CONF command*/
#define CM36686_ALS_IT_80ms 	(0 << 6)
#define CM36686_ALS_IT_160ms 	(1 << 6)
#define CM36686_ALS_IT_320ms 	(2 << 6)
#define CM36686_ALS_IT_640ms 	(3 << 6)
#define CM36686_ALS_GAIN_1 		(0 << 2)
#define CM36686_ALS_GAIN_2 		(1 << 2)
#define CM36686_ALS_GAIN_4 		(2 << 2)
#define CM36686_ALS_GAIN_8 		(3 << 2)
#define CM36686_ALS_INT_EN	 	(1 << 1) /*enable/disable Interrupt*/
#define CM36686_ALS_INT_MASK	0xFFFD
#define CM36686_ALS_SD			(1 << 0) /*enable/disable ALS func, 1:disable , 0: enable*/
#define CM36686_ALS_SD_MASK		0xFFFE

/*for PS CONF1 command*/
#define CM36686_PS_ITB_1_2	       (0 << 14)
#define CM36686_PS_ITB_1           (1 << 14)
#define CM36686_PS_ITB_2           (2 << 14)
#define CM36686_PS_ITB_4           (3 << 14)
#define CM36686_PS_INT_OFF	       (0 << 8) /*enable/disable Interrupt*/
#define CM36686_PS_INT_IN          (1 << 8)
#define CM36686_PS_INT_OUT         (2 << 8)
#define CM36686_PS_INT_IN_AND_OUT  (3 << 8)

#define CM36686_PS_INT_MASK   0xFCFF

#define CM36686_PS_DR_1_40   (0 << 6)
#define CM36686_PS_DR_1_80   (1 << 6)
#define CM36686_PS_DR_1_160  (2 << 6)
#define CM36686_PS_DR_1_320  (3 << 6)
#define CM36686_PS_IT_1T 	 (0 << 4)
#define CM36686_PS_IT_1_3T   (1 << 4)
#define CM36686_PS_IT_1_6T 	 (2 << 4)
#define CM36686_PS_IT_2T 	 (3 << 4)
#define CM36686_PS_PERS_1 	 (0 << 2)
#define CM36686_PS_PERS_2 	 (1 << 2)
#define CM36686_PS_PERS_3 	 (2 << 2)
#define CM36686_PS_PERS_4 	 (3 << 2)
#define CM36686_PS_RES_1     (1 << 1)
#define CM36686_PS_SD	     (1 << 0)/*enable/disable PS func, 1:disable , 0: enable*/
#define CM36686_PS_SD_MASK	 0xFFFE

/*for PS CONF3 command*/
#define CM36686_PS_MS_NORMAL          (0 << 14)
#define CM36686_PS_MS_LOGIC_ENABLE    (1 << 14)
#define CM36686_PS_PROL_63 	          (0 << 12)
#define CM36686_PS_PROL_127           (1 << 12)
#define CM36686_PS_PROL_191 	      (2 << 12)
#define CM36686_PS_PROL_255 		  (3 << 12)
#define CM36686_PS_SMART_PERS_ENABLE  (1 << 4)
#define CM36686_PS_ACTIVE_FORCE_MODE  (1 << 3)
#define CM36686_PS_ACTIVE_FORCE_TRIG  (1 << 2)

/*for INT FLAG*/
#define INT_FLAG_PS_SPFLAG           (1<<14)
#define INT_FLAG_ALS_IF_L            (1<<13)
#define INT_FLAG_ALS_IF_H            (1<<12)
#define INT_FLAG_PS_IF_CLOSE         (1<<9)
#define INT_FLAG_PS_IF_AWAY          (1<<8)  

//#define LS_PWR_ON		BIT(0)
//#define PS_PWR_ON		BIT(1)

#define CAPELLA_CM36686_IOCTL_MAGIC 'c'
#define CAPELLA_CM36686_IOCTL_GET_ENABLED \
	_IOR(CAPELLA_CM36686_IOCTL_MAGIC, 1, int *)
#define CAPELLA_CM36686_IOCTL_ENABLE \
	_IOW(CAPELLA_CM36686_IOCTL_MAGIC, 2, int *)

#define LIGHTSENSOR_IOCTL_MAGIC 'l'
#define LIGHTSENSOR_IOCTL_GET_ENABLED _IOR(LIGHTSENSOR_IOCTL_MAGIC, 1, int *)
#define LIGHTSENSOR_IOCTL_ENABLE _IOW(LIGHTSENSOR_IOCTL_MAGIC, 2, int *)

extern unsigned int ps_kparam1;
extern unsigned int ps_kparam2;

#define CM36686_LEVELS_SIZE		10

enum {
	CM36686_ALS_IT0 = 0,
	CM36686_ALS_IT1,
	CM36686_ALS_IT2,
	CM36686_ALS_IT3,
};

struct CM36686_platform_data {
	int intr_pin;
	uint16_t adc_table[10];
	uint16_t golden_adc;
	int (*power)(int, uint8_t); /* power to the chip */
	uint8_t slave_addr;
	uint16_t ps_close_thd_set;
	uint16_t ps_away_thd_set;	
	uint16_t ls_cmd;
	uint16_t ps_conf1_val;
	uint16_t ps_conf3_val;	
	uint16_t als_it;
	bool polling;
};

#endif
