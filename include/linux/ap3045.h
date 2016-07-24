/*
* This file is part of the Dyna-Image AP3045 sensor driver for Nexus7 platform.
* AP3045 is combined proximity, ambient light sensor and IRLED.
*
* Contact: John Huang <john.huang@dyna-image.com>
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
* General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
* 02110-1301 USA
*
*
* Filename: ap3045.h
*
* Summary:
*	AP3045 sensor dirver header file.
*
* Modification History:
* Date     By       Summary
* -------- -------- -------------------------------------------------------
* 07/08/15 Ben		 Original Creation (Test version:1.0)
*
*/

/*
* Definitions for ap3045 als/ps sensor chip.
*/
#ifndef __AP3045_H__
#define __AP3045_H__

#include <linux/ioctl.h>

#define AP3045_SUCCESS					0
#define AP3045_ERR_I2C					-1
#define AP3045_ERR_STATUS				-3
#define AP3045_ERR_SETUP_FAILURE		-4
#define AP3045_ERR_GETGSENSORDATA		-5
#define AP3045_ERR_IDENTIFICATION		-6

#define AP3045_NUM_CACHABLE_REGS	42

#define AP3045_ADATA_CH0_L 	       0x2C
#define AP3045_ADATA_CH0_H         0x2D
#define AP3045_ADATA_CH1_L 	       0x2E
#define AP3045_ADATA_CH1_H         0x2F
#define AP3045_ADATA_L 	           AP3045_ADATA_CH0_L
#define AP3045_ADATA_H 	           AP3045_ADATA_CH0_H
//SYSTEM MODE (AP3045_REG_SYS_CONF)
#define	AP3045_SYS_DEV_DOWN        0x00
#define	AP3045_SYS_ALS_ENABLE      0x01
#define	AP3045_SYS_PS_ENABLE       0x02
#define	AP3045_SYS_ALS_PS_ENABLE   0x03
#define	AP3045_SYS_RST_ENABLE      0x04
#define	AP3045_SYS_HS_ENABLE       0x10
#define	AP3045_SYS_DEV_RESET       			0x04

/* ap3045 control registers */
/*============================================================================*/
#define AP3045_REG_SYS_CONF        0x00
#define AP3045_REG_SYS_CONF_MASK	    0x17
#define AP3045_REG_SYS_CONF_SHIFT	    (0)
#define AP3045_REG_SYS_CONF_HC_MASK	    0x10
#define AP3045_REG_SYS_CONF_HC_SHIFT	(4)
#define AP3045_REG_SYS_CONF_PS_MASK	    0x02
#define AP3045_REG_SYS_CONF_PS_SHIFT	(1)
#define AP3045_REG_SYS_CONF_AL_MASK	    0x01
#define AP3045_REG_SYS_CONF_AL_SHIFT	(0)

/* ap3045 interrupt flag */
#define AP3045_REG_SYS_INTSTATUS   0x01
#define AP3045_REG_SYS_INT_SHIFT    (0)
#define AP3045_REG_SYS_INT_ERR_MASK	 0x20
#define AP3045_REG_SYS_INT_ERR_SHIFT (5)
//#define AP3045_REG_SYS_INT_MASK		 0x07
#define AP3045_REG_SYS_INT_HMASK	 0x04
#define AP3045_REG_SYS_INT_HC_SHIFT	 (2)
#define AP3045_REG_SYS_INT_PS_MASK	 0x02
#define AP3045_REG_SYS_INT_PS_SHIFT	 (1)
#define AP3045_REG_SYS_INT_AL_MASK	 0x01
#define AP3045_REG_SYS_INT_AL_SHIFT	 (0)

#define AP3045_OBJ_COMMAND	0x01
#define AP3045_OBJ_MASK		0x10
#define AP3045_OBJ_SHIFT	(4)

/* ap3045 interrupt flag */
#define AP3045_REG_SYS_INT_STATUS   			0x01
#define AP3045_REG_SYS_INT_MASK				0x23
#define AP3045_REG_SYS_INT_PMASK			0x02
#define AP3045_REG_SYS_INT_AMASK			0x01

/* ap3045 interrupt control */
#define AP3045_REG_SYS_INTCTRL     0x02
#define AP3045_REG_SYS_INTCTRL_PIEN_MASK     0x80
#define AP3045_REG_SYS_INTCTRL_PSPEND_MASK   0x40
#define AP3045_REG_SYS_INTCTRL_PSPEND_SHIFT   (6)
#define AP3045_REG_SYS_INTCTRL_PSMODE_MASK   0x30
#define AP3045_REG_SYS_INTCTRL_PSACC_MASK    0x10
#define AP3045_REG_SYS_INTCTRL_AIEN_MASK     0x08
#define AP3045_REG_SYS_INTCTRL_ALPEND_MASK   0x04

/* ap3045 control registers */
#define AP3045_REG_SYS_INT_CTRL     			0x02

/* ap3045 ALS Control Register */
#define AP3045_REG_ALS_GAIN_CONF			0x07
#define AP3045_ALS_RANGE_MASK				0x07
#define AP3045_ALS_RANGE_SHIFT				(0)
#define AP3045_REG_ALS_AX_GAIN_MASK	 		0x04
#define AP3045_REG_ALS_A_GAIN_MASK	 		0x03

/* ap3045 Waiting Time Register */
#define AP3045_REG_SYS_WAITTIME    0x06
#define AP3045_REG_SYS_WAITTIME_WTIME_MASK  0x7F
#define AP3045_REG_SYS_WAITTIME_WTIME_SHIFT (0)
#define AP3045_REG_SYS_WAITTIME_WUNIT_MASK  0x80
#define AP3045_REG_SYS_WAITTIME_WUNIT_SHIFT (7)

/* ap3045 ALS Control Register */
#define AP3045_REG_ALS_CONF        0x07
#define AP3045_REG_ALS_AXGAIN_CON_MASK   0x04
#define AP3045_REG_ALS_AXGAIN_CON_SHIFT  (2)
#define AP3045_REG_ALS_AGAIN_CON_MASK    0x03
#define AP3045_REG_ALS_AGAIN_CON_SHIFT   (0)

/* ap3045 ALS Persistence Register */
#define AP3045_REG_ALS_PERSIS      0x08
#define AP3045_REG_ALS_PERSIS_MASK       0x3F
#define AP3045_REG_ALS_PERSIS_SHIFT      (0)

/* ap3045 ALS Time Registers */
#define AP3045_REG_ALS_TIME    0x0A
#define AP3045_REG_ALS_TIME_MASK         0xFF
#define AP3045_REG_ALS_TIME_SHIFT        (0)

/* ap3045 PS Control Register */
#define AP3045_REG_PS_GAIN    0x0C
#define AP3045_REG_PS_PLPUC_MASK    0xF0
#define AP3045_REG_PS_PLPUC_SHIFT   (4)
#define AP3045_REG_PS_GAIN_MASK     0x01
#define AP3045_REG_PS_GAIN_SHIFT    (0)

/* ap3045 PS persistence Register */
#define AP3045_REG_PS_PERSIS				0x0D
#define AP3045_REG_PS_PERS    0x0D
#define AP3045_REG_PS_PERS_MASK    0x3F
#define AP3045_REG_PS_PERS_SHIFT  (0)

/* ap3045 PS time Register */
#define AP3045_REG_PS_TIME    0x0F
#define AP3045_REG_PS_TIME_MASK    0x03
#define AP3045_REG_PS_TIME_SHIFT   (0)

/* ap3045 PS LED control Register */
#define AP3045_REG_PS_LED_CONTROL			0x10
#define AP3045_REG_PS_LED_CON    0x10
#define AP3045_REG_PS_LED_CON_PSLDR_MASK    0xC0
#define AP3045_REG_PS_LED_CON_PSLDR_SHIFT   (6)
#define AP3045_REG_PS_LED_CON_PSLPUW_MASK   0x3F
#define AP3045_REG_PS_LED_CON_PSLPUW_SHIFT  (0)

/* ap3045 Mean Time Register */
#define AP3045_REG_PS_MEAN    0x1B
#define AP3045_REG_PS_MEAN_MASK    0x0F
#define AP3045_REG_PS_MEAN_SHIFT   (0)

/* ap3045 PS Data Low Register */
#define AP3045_REG_PS_DATA_LOW    0x26
#define AP3045_REG_PS_DATA_LOW_MASK    0xFF
#define AP3045_REG_PS_DATA_LOW_SHIFT   (0)


/* ap3045 HC IR Data Low Register */
#define AP3045_REG_HC_DATA_LOW    0x2A
#define AP3045_REG_HC_DATA_LOW_MASK    0xFF
#define AP3045_REG_HC_DATA_LOW_SHIFT   (0)

/* ap3045 HC IR Data High Register */
#define AP3045_REG_HC_DATA_HIGH    0x2B
#define AP3045_REG_HC_DATA_HIGH_MASK    0xFF
#define AP3045_REG_HC_DATA_HIGH_SHIFT   (0)

/* ap3045 als data_CH0/CH1/LDATA */
/* ap3045 PS Data High Register */
#define AP3045_REG_PS_DATA_HIGH   	 		0x27
#define AP3045_REG_PS_DATA_HIGH_SHIFT  			(8)
#define	AP3045_REG_PS_DATA_HIGH_MASK			0x0F


/* ap3045 als data_CH0/CH1/LDATA */
#define AP3045_REG_ALS_CH0_DATA_LOW			0x2C
#define AP3045_REG_ALS_CH0_DATA_HIGH			0x2D
#define AP3045_REG_ALS_CH1_DATA_LOW			0x2E
#define AP3045_REG_ALS_CH1_DATA_HIGH			0x2F
#define AP3045_REG_ALS_L_DATA_LOW			0x30
#define AP3045_REG_ALS_L_DATA_HIGH			0x31

/* ap3045 PS Low Threshold Low Register */
#define AP3045_REG_PS_THDL_L       			0x36
#define AP3045_REG_PS_THDL_L_SHIFT			(0)
#define AP3045_REG_PS_THDL_L_MASK			0xFF

/* ap3045 PS Low Threshold High Register */
#define AP3045_REG_PS_THDL_H       			0x37
#define AP3045_REG_PS_THDL_H_SHIFT			(0)	//(2)
#define AP3045_REG_PS_THDL_H_MASK			0x03


/* ap3045 PS High Threshold Low Register */
#define AP3045_REG_PS_THDH_L       			0x38
#define AP3045_REG_PS_THDH_L_SHIFT			(0)
#define AP3045_REG_PS_THDH_L_MASK			0xFF

/* ap3045 PS High Threshold High Register */
#define AP3045_REG_PS_THDH_H       			0x39
#define AP3045_REG_PS_THDH_H_SHIFT			(0)
#define AP3045_REG_PS_THDH_H_MASK			0x03

/* ap3045 ALS Ch0 Data Low Register */
#define AP3045_REG_AL_CH0_DATA_LOW    0x2C
#define AP3045_REG_AL_CH0_DATA_LOW_MASK    0xFF
#define AP3045_REG_AL_CH0_DATA_LOW_SHIFT   (0)

/* ap3045 ALS CH0 Data High Register */
#define AP3045_REG_AL_CH0_DATA_HIGH    0x2D
#define AP3045_REG_AL_CH0_DATA_HIGH_MASK    0xFF
#define AP3045_REG_AL_CH0_DATA_HIGH_SHIFT   (0)

/* ap3045 ALS Ch1 Data Low Register */
#define AP3045_REG_AL_CH1_DATA_LOW    0x2E
#define AP3045_REG_AL_CH1_DATA_LOW_MASK    0xFF
#define AP3045_REG_AL_CH1_DATA_LOW_SHIFT   (0)

/* ap3045 ALS CH1 Data High Register */
#define AP3045_REG_AL_CH1_DATA_HIGH    0x2F
#define AP3045_REG_AL_CH1_DATA_HIGH_MASK    0xFF
#define AP3045_REG_AL_CH1_DATA_HIGH_SHIFT   (0)

/* ap3045 ALS L Data Low Register */
#define AP3045_REG_AL_L_DATA_LOW    0x30
#define AP3045_REG_AL_L_DATA_LOW_MASK    0xFF
#define AP3045_REG_AL_L_DATA_LOW_SHIFT   (0)

/* ap3045 ALS L Data High Register */
#define AP3045_REG_AL_L_DATA_HIGH    0x31
#define AP3045_REG_AL_L_DATA_HIGH_MASK    0xFF
#define AP3045_REG_AL_L_DATA_HIGH_SHIFT   (0)

/* ap3045 ALS Low Threshold Low Register */
#define AP3045_REG_AL_THL_LOW    0x32
#define AP3045_REG_AL_THL_LOW_MASK    0xFF
#define AP3045_REG_AL_THL_LOW_SHIFT   (0)

/* ap3045 ALS Low Threshold High Register */
#define AP3045_REG_AL_THL_HIGH    0x33
#define AP3045_REG_AL_THL_HIGH_MASK    0xFF
#define AP3045_REG_AL_THL_HIGH_SHIFT   (0)

/* ap3045 ALS High Threshold Low Register */
#define AP3045_REG_AL_THH_LOW    0x34
#define AP3045_REG_AL_THH_LOW_MASK    0xFF
#define AP3045_REG_AL_THH_LOW_SHIFT   (0)

/* ap3045 ALS High Threshold High Register */
#define AP3045_REG_AL_THH_HIGH    0x35
#define AP3045_REG_AL_THH_HIGH_MASK    0xFF
#define AP3045_REG_AL_THH_HIGH_SHIFT   (0)

/* ap3045 PS Low Threshold Low Register */
#define AP3045_REG_PS_THL_LOW    0x36
#define AP3045_REG_PS_THL_LOW_MASK    0xFF
#define AP3045_REG_PS_THL_LOW_SHIFT   (0)

/* ap3045 PS Low Threshold High Register */
#define AP3045_REG_PS_THL_HIGH    0x37
#define AP3045_REG_PS_THL_HIGH_MASK    0x03
#define AP3045_REG_PS_THL_HIGH_SHIFT   (0)

/* ap3045 PS High Threshold Low Register */
#define AP3045_REG_PS_THH_LOW    0x38
#define AP3045_REG_PS_THH_LOW_MASK    0xFF
#define AP3045_REG_PS_THH_LOW_SHIFT   (0)

/* ap3045 PS High Threshold High Register */
#define AP3045_REG_PS_THH_HIGH    0x39
#define AP3045_REG_PS_THH_HIGH_MASK    0x03
#define AP3045_REG_PS_THH_HIGH_SHIFT   (0)

/* ap3045 PS Calibration Low Register */
#define AP3045_REG_PS_CAL_LOW    0x3A
#define AP3045_REG_PS_CAL_LOW_MASK    0xFF
#define AP3045_REG_PS_CAL_LOW_SHIFT   (0)

/* ap3045 PS Calibration High Register */
#define AP3045_REG_PS_CAL_HIGH    0x3B
#define AP3045_REG_PS_CAL_HIGH_MASK    0x03
#define AP3045_REG_PS_CAL_HIGH_SHIFT   (0)

/* ap3045 ALS Illuminace Coefficient Register */
#define AP3045_REG_AL_LUMIN_COEF    0x3C
#define AP3045_REG_AL_LUMIN_COEF_MASK    0xFF
#define AP3045_REG_AL_LUMIN_COEF_SHIFT   (0)

/* ap3045 HC Waiting Time Register */
#define AP3045_REG_HC_WAITTIME    0x80
#define AP3045_REG_HC_WAITTIME_WTIME_MASK  0x7F
#define AP3045_REG_HC_WAITTIME_WTIME_SHIFT (0)
#define AP3045_REG_HC_WAITTIME_WUNIT_MASK  0x80
#define AP3045_REG_HC_WAITTIME_WUNIT_SHIFT (7)

/* ap3045 HC Control */
#define AP3045_REG_HC_CTRL     0x81
#define AP3045_REG_HC_CTRL_HIEN_MASK     0x80
#define AP3045_REG_HC_CTRL_HPEND_MASK    0x40
#define AP3045_REG_HC_CTRL_HPEND_SHIFT   (6)
#define AP3045_REG_HC_CTRL_PSMODE_MASK   0x40
#define AP3045_REG_HC_CTRL_IRGAIN_MASK   0x18
#define AP3045_REG_HC_CTRL_IRGAIN_SHIFT  (4)
#define AP3045_REG_HC_CTRL_RGAIN_MASK    0x03
#define AP3045_REG_HC_CTRL_RGAIN_SHIFT   (0)

/* ap3045 HC RLED control Register */
#define AP3045_REG_HC_RLED_CON    0x82
#define AP3045_REG_HC_RLED_CON_LDR_MASK     0x40
#define AP3045_REG_HC_RLED_CON_LDR_SHIFT    (6)
#define AP3045_REG_HC_RLED_CON_LPUW_MASK    0x3F
#define AP3045_REG_HC_RLED_CON_LPUW_SHIFT   (0)

/* ap3045 HC IRLED control Register */
#define AP3045_REG_HC_IRLED_CON    0x83
#define AP3045_REG_HC_IRLED_CON_LDR_MASK     0x40
#define AP3045_REG_HC_IRLED_CON_LDR_SHIFT    (6)
#define AP3045_REG_HC_IRLED_CON_LPUW_MASK    0x3F
#define AP3045_REG_HC_IRLED_CON_LPUW_SHIFT   (0)

/* ap3045 HC FIFO Level Register */
#define AP3045_REG_HC_FIFO_LVL_CON    0x84
#define AP3045_REG_HC_FIFO_LVL_MASK     0x0F
#define AP3045_REG_HC_FIFO_LVL_SHIFT    (0)

/* ap3045 HC FIFO Quantity Register */
#define AP3045_REG_HC_FIFO_QTY_CON    0x85
#define AP3045_REG_HC_FIFO_QTY_MASK     0x1F
#define AP3045_REG_HC_FIFO_QTY_SHIFT    (0)

/* ap3045 HC FIFO Speed Register */
#define AP3045_REG_HC_FIFO_SPEED_CON    0x86
#define AP3045_REG_HC_FIFO_SPEED_MASK     0xFF
#define AP3045_REG_HC_FIFO_SPEED_SHIFT    (0)

/* ap3045 HC FIFO Data Register */
#define AP3045_REG_HC_FIFO_DATA_CON    0xE1
#define AP3045_REG_HC_FIFO_DATA_TAG_MASK     0xC0
#define AP3045_REG_HC_FIFO_DATA_TAG_SHIFT    (6)
#define AP3045_REG_HC_FIFO_DATA_MASK         0x3F
#define AP3045_REG_HC_FIFO_DATA_SHIFT        (0)
/*----------------------------------------------------------------------------*/

#endif

#define PROXIMITYSENSOR_IOCTL_MAGIC 'c'
#define PROXIMITYSENSOR_IOCTL_GET_ENABLED \
	_IOR(PROXIMITYSENSOR_IOCTL_MAGIC, 1, int *)
#define PROXIMITYSENSOR_IOCTL_ENABLE \
	_IOW(PROXIMITYSENSOR_IOCTL_MAGIC, 2, int *)

#define PBUFF_SIZE				16	/* Rx buffer size */

#define ASUS_PSENSOR_IOCTL_CLOSE              _IO(PROXIMITYSENSOR_IOCTL_MAGIC, 0x12)
#define ASUS_PSENSOR_IOCTL_START              _IO(PROXIMITYSENSOR_IOCTL_MAGIC, 0x13)
#define ASUS_PSENSOR_IOCTL_GETDATA            _IOR(PROXIMITYSENSOR_IOCTL_MAGIC, 0x14, char[PBUFF_SIZE+1])

#define ASUS_PSENSOR_SETCALI_DATA             _IOW(PROXIMITYSENSOR_IOCTL_MAGIC, 0x15, int[2])
#define ASUS_PSENSOR_EN_CALIBRATION           _IOW(PROXIMITYSENSOR_IOCTL_MAGIC, 0x16, char)


#define LIGHTSENSOR_IOCTL_MAGIC 'l'

#define LIGHTSENSOR_IOCTL_GET_ENABLED _IOR(LIGHTSENSOR_IOCTL_MAGIC, 1, int *)
#define LIGHTSENSOR_IOCTL_ENABLE _IOW(LIGHTSENSOR_IOCTL_MAGIC, 2, int *)

#define LBUFF_SIZE				16	/* Rx buffer size */

#define ASUS_LIGHTSENSOR_IOCTL_CLOSE          _IO(LIGHTSENSOR_IOCTL_MAGIC, 0x12)
#define ASUS_LIGHTSENSOR_IOCTL_START          _IO(LIGHTSENSOR_IOCTL_MAGIC, 0x13)
#define ASUS_LIGHTSENSOR_IOCTL_GETDATA        _IOR(LIGHTSENSOR_IOCTL_MAGIC, 0x14, char[LBUFF_SIZE+1])

#define ASUS_LIGHTSENSOR_SETCALI_DATA         _IOW(LIGHTSENSOR_IOCTL_MAGIC, 0x15, int[2])
#define ASUS_LIGHTSENSOR_EN_CALIBRATION       _IOW(LIGHTSENSOR_IOCTL_MAGIC, 0x16, char)


#define AP3045_REG_ALS_THDL_L      	 0x32
#define AP3045_REG_ALS_THDL_L_SHIFT	(0)
#define AP3045_REG_ALS_THDL_L_MASK	0xFF

#define AP3045_REG_ALS_THDL_H     	 0x33
#define AP3045_REG_ALS_THDL_H_SHIFT	(8)
#define AP3045_REG_ALS_THDL_H_MASK	0xFF

#define AP3045_REG_ALS_THDH_L      	0x34
#define AP3045_REG_ALS_THDH_L_SHIFT	(0)
#define AP3045_REG_ALS_THDH_L_MASK	0xFF

#define AP3045_REG_ALS_THDH_H     	 0x35
#define AP3045_REG_ALS_THDH_H_SHIFT	(8)
#define AP3045_REG_ALS_THDH_H_MASK	0xFF

struct ap3045_data {
	struct i2c_client *client;
	u8 reg_cache[AP3045_NUM_CACHABLE_REGS];
	u8 power_state_before_suspend;
	int gpio;
	int irq;
	u8 reg;
	int old_mode;
	struct input_dev	*ps_input_dev;
	struct input_dev	*ls_input_dev;
	struct workqueue_struct *plsensor_wq;
	struct work_struct plsensor_work;
	struct timer_list pl_timer;
	int ps_opened;
	int ls_opened;
	int last_initial_report_lux;
};