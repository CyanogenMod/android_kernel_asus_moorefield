/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-05
*
*/

#ifndef __LINUX_SHOW_SENSOR_LOG_H
#define __LINUX_SHOW_SENSOR_LOG_H

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/uaccess.h>
#include <linux/slab.h>

/* Log tag */
#define TAG "LASER_FOCUS"

/* Debug log flag */
#define DEBUG_LOG_FLAG 0

/* Log contorller */
enum log_ctl{
	/* Log disable */
	ALL_DISABLE = 00,	/* All debug log */
	CDBG_DISABLE = 01,	/* Major debug log */
	DBG_DISABLE = 02,	/* General debug log */
	REG_RW_DBG_DISABLE = 03,	/* I2C register debug log */
	API_DBG_DISABEL = 04,	/* API debug log */
	FUN_DBG_DISABLE = 05,	/* Function debug log */
	ERR_DBG_DISABLE = 06,	/* Error debug log ERR_DBG_ENABLE*/
	
	/* Log enable */
	ALL_ENABLE = 10,	/* All debug log */
	CDBG_ENABLE = 11,	/* Major debug log */
	DBG_ENABLE = 12,	/* General debug log */
	REG_RW_DBG_ENABLE = 13,	/* I2C register debug log */
	API_DBG_ENABLE = 14,	/* API debug log */
	FUN_DBG_ENABLE = 15,	/* Function debug log */
	ERR_DGB_ENABLE = 16,	/* Error debug log */
};

/* Log type */
enum log_t{
	LOG_CDBG,	/* Major debug log */
	LOG_DBG,	/* General debug log */
	LOG_REG,	/* I2C register debug log */
	LOG_API,	/* API debug log */
	LOG_FUN,	/* Function debug log */
	LOG_ERR,	/* Error debug log */
};

/* Display logl */
void display_log(const char* tag, const char* fmt, va_list args);
/* Handle log */
void LOG_Handler(int log_type, const char* fmt, ...);
/* Log contorl */
ssize_t Laser_Focus_log_contorl(const char __user *num, size_t len);

#endif
