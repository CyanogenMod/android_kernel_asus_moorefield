#ifndef __LINUX_LASER_FOCUS_SENSOR_HEPTA_H
#define __LINUX_LASER_FOCUS_SENSOR_HEPTA_H

#define	DEVICE_GET_CALIBRATION_INPUT_DATA		"driver/LaserFocus_CalGetInputData" /* Get Calibration input data */

#ifdef CONFIG_ASUS_FACTORY_MODE
#define DEVICE_GET_CALIBRATION_INPUT_DATA_MODE 0776     /* Calibration get input data right */
#else
#define	DEVICE_GET_CALIBRATION_INPUT_DATA_MODE 0664	/* Calibration get input data right */
#endif

struct msm_laser_focus_ctrl_t *get_laura_ctrl(void);

#endif
