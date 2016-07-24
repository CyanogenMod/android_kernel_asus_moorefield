/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-07
*
*/

#ifndef __LINUX_SHOW_VL6180x_SENSOR_SHIPPING_FUNC_H
#define __LINUX_SHOW_VL6180x_SENSOR_SHIPPING_FUNC_H

#include "msm_laser_focus.h"
#include "laura_i2c.h"


/*
#define CDBG(fmt, args...) pr_info(fmt, ##args)

#define SPMI_PROP_READ(chip_prop, qpnp_spmi_property, retval)		\
do {									\
	if (retval)							\
		break;							\
	retval = of_property_read_u32(chip->spmi->dev.of_node,		\
				"qcom," qpnp_spmi_property,		\
					&chip->dt.chip_prop);		\
	if (retval) {							\
		pr_err("Error reading " #qpnp_spmi_property		\
					" property %d\n", retval);	\
	}								\
} while (0)

#define SPMI_PROP_READ_OPTIONAL(chip_prop, qpnp_spmi_property, retval)	\
do {									\
	retval = of_property_read_u32(chip->spmi->dev.of_node,		\
				"qcom," qpnp_spmi_property,		\
					&chip->dt.chip_prop);		\
	if (retval)							\
		chip->dt.chip_prop = -EINVAL;				\
} while (0)
*/

#define CHECK_LESS(func, args...)	\
do{								\
	status = func(##args);		\
	if(status < 0)					\
		goto err;					\
}while(0)


#define _LASER_BIT_MASK(BITS, POS) \
	((unsigned short)(((1 << (BITS)) - 1) << (POS)))
	
#define LASER_BIT_MASK(LEFT_BIT_POS, RIGHT_BIT_POS) \
		_LASER_BIT_MASK((LEFT_BIT_POS) - (RIGHT_BIT_POS) + 1, \
					(RIGHT_BIT_POS))

#define BIT(nr)			(1UL << (nr))


#define	ICSR	0x00
	#define	GO_AHEAD		0x00
	#define	PAD_INT_MODE	BIT(2)
	#define	NEW_DATA_IN_RESULT_REG	BIT(4)
	#define 	NEW_DATA_IN_MBX	BIT(5)
	#define	MCPU_HAVE_READ_MBX	BIT(6)

#define	COMMAND_REGISTER		0x04
	#define	SINGLE_MEASURE	0x01
	#define	GO_STANDBY		0x10
	#define	GO_MCPU_OFF		0x11
	#define	MCPU_TO_ON		0x12
	#define	VALIDATE_CMD		BIT(7)
	
#define	DEVICE_STATUS		0x06
	#define	STATUS_MASK	LASER_BIT_MASK(4,0)
	#define	STATUS_STANDBY	0x00
	#define	STATUS_MCPU_OFF	0x10
	#define	STATUS_MCPU_ON	0x18
		#define STATUS_MEASURE_ON		0x01F8
	
#define	RESULT_REGISTER	0x08
	#define	DISTANCE_MASK	LASER_BIT_MASK(12,2)
	#define	ERROR_CODE_MASK	LASER_BIT_MASK(14,13)
		#define	NO_ERROR			0x0000
		#define	GENERAL_ERROR		0x6000
		#define	FAR_FIELD			0x4000
		#define	NEAR_FIELD			0x2000
	#define	VALID_DATA		BIT(15)



#define	H2M_MBX	0x10
	#define	GET_CALIBRATION	0x04

#define	M2H_MBX	0x12
	#define	MESSAGE_ID_MASK	0xFF	
	#define	CMD_LEN_MASK	LASER_BIT_MASK(13,8)
	
#define	PMU_CONFIG	0x14
	#define	PATCH_CODE_LD_EN	BIT(8)
	#define	MCPU_INIT_STATE	BIT(9)
	#define	PATCH_MEM_EN		BIT(10)
	#define	PATHC_CODE_RETAIN	BIT(11)
	
#define	I2C_DATA_PORT	0x1A
#define	I2C_INIT_CONFIG		0x1C
#define	MODULE_CHIP_ID		0x28






#ifndef LOG_SAMPLE_RATE
/* Log sample rate  */
#define LOG_SAMPLE_RATE 50
#endif

#ifndef OUT_OF_RANGE
/* Out of range */
#define OUT_OF_RANGE 9999
#endif

#ifndef TIMEOUT_VAL
/* Time out value: mm */
#define TIMEOUT_VAL 80
#endif

/* Read retry flag */
#define READ_RETRY_FLAG 0

/* Read output limit flag */
#define READ_OUTPUT_LIMIT_FLAG 1

/* MCPU power on fail */
#define MCPU_POWER_ON_FAIL 1

/* Laura calibration message lens */
#define CAL_MSG_LEN 6

/* Laura range error */
#define RANGE_ADAPT 0
#define RANGE_ERR_NOT_ADAPT 11
#define RANGE_ERR_TOO_NEAR 12
#define RANGE_ERR_TOO_FAR 13 
#define RANGE_ERR_OTHER 14
#define RANGE_ERR_OTHER2 15

/* MCPU status */
#define MCPU_ON 0
#define MCPU_OFF 1
#define MCPU_STANDBY 2

/* Swap high and low of the data (e.g 0x1234 => 0x3412) */
void swap_data(uint16_t* register_data);
/* Mailbox: create calibration data */
int Mailbox_Command(struct msm_laser_focus_ctrl_t *dev_t, int16_t cal_data[]);
int Olivia_Mailbox_Command(struct msm_laser_focus_ctrl_t *dev_t, int16_t cal_data[]);
/* Get the laster raw range */
uint16_t get_debug_raw_range(void);
/* Get the laster raw confidence */
uint16_t get_debug_raw_confidence(void);
/* Load calibration data */
int Laura_device_Load_Calibration_Value(struct msm_laser_focus_ctrl_t *dev_t);
int Olivia_device_Load_Calibration_Value(struct msm_laser_focus_ctrl_t *dev_t);

int Laura_Read_Calibration_Value_From_File(struct seq_file *vfile, uint16_t *cal_data);
int Olivia_Read_Calibration_Value_From_File(struct seq_file *vfile, uint16_t *cal_data);
/* Read range */
uint16_t Laura_device_read_range(struct msm_laser_focus_ctrl_t *dev_t);
int Olivia_device_read_range(struct msm_laser_focus_ctrl_t *dev_t);
/* MCPU controller */
int Laura_MCPU_Controller(struct msm_laser_focus_ctrl_t *dev_t, int mode);
/* Initialize Laura tof configure */
int Laura_device_UpscaleRegInit(struct msm_laser_focus_ctrl_t *dev_t, uint16_t *config);
/* Wait device go to standby mode */
int Laura_WaitDeviceStandby(struct msm_laser_focus_ctrl_t *dev_t);
/* Configure i2c interface */
int Laura_Config_I2C_Interface(struct msm_laser_focus_ctrl_t *dev_t);
/* Power up initialization without applying calibration data */
int Laura_No_Apply_Calibration(struct msm_laser_focus_ctrl_t *dev_t);
/* Power up initialization which apply calibration data */
int Laura_Apply_Calibration(struct msm_laser_focus_ctrl_t *dev_t);
/* Go to standby mode when do not do measure */
int Laura_non_measures_go_standby(struct msm_laser_focus_ctrl_t *dev_t);
/* Load module id from chip */
void Laura_Get_Module_ID_From_Chip(struct msm_laser_focus_ctrl_t *dev_t);
/* Get module id from driver */
void Laura_Get_Module_ID(struct msm_laser_focus_ctrl_t *dev_t, struct seq_file *vfile);
/* Verify firmware version */
bool Laura_FirmWare_Verify(struct msm_laser_focus_ctrl_t *dev_t);
//int laura_read_write_test(struct msm_laser_focus_ctrl_t * dev_t);

int Olivia_DumpKdata(struct msm_laser_focus_ctrl_t *dev_t, int16_t cal_data[11]);

#endif
