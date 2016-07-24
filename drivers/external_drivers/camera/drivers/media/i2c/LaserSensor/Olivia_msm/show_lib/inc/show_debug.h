/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-06
*
*/

#ifndef __LINUX_SHOW_SENSOR_DEBUG_H
#define __LINUX_SHOW_SENSOR_DEBUG_H

#include "msm_laser_focus.h"
#include "laser_focus_i2c.h"

/* Dump register value from 0 to 0x100 (Word) */
int dump_register(struct seq_file *vfile, struct msm_laser_focus_ctrl_t *dev_t, int min, int max);
/* Make laser driver return a fake value */
ssize_t Laser_Focus_enforce(struct msm_laser_focus_ctrl_t *dev_t, const char __user *buff, size_t len, int *LF_enforce_ctrl);

#endif
