/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-06
*
*/

#ifndef __LINUX_SHOW_SENSOR_VERIFY_H
#define __LINUX_SHOW_SENSOR_VERIFY_H

#include <linux/types.h>
#include "show_sysfs.h"

/* Laser Focus spec controller */
#define LASER_FOCUS_OFFSET_SPEC 0	/* Offset spec */
#define LASER_FOCUS_CROSS_TALK_SPEC 1	/* Cross talk spec */

/* Laser Focus ragne information */
#define DEVICE_CROSSTALK_CAL_RANGE		400	/* 400mm */
#define DEVICE_OFFSET_CAL_RANGE			100	/* 100mm */

/* Laser Focus spec path */
#define	OFFSET_SPEC_FILE		"/data/data/LaserFocus_Calibration10_Spec.txt"	/* Calibration offset spec */
#define	CROSS_TALK_SPEC_FILE	"/data/data/LaserFocus_Calibration40_Spec.txt"	/* Calibration cross talk spec */

/* Verify whether or not the range is legal */
bool Violation_Calibration_Spec(int RawRange, int ctrl);

#endif
