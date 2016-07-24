/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-06
*
*/

#include "show_log.h"

#ifdef CONFIG_ASUS_FACTORY_MODE
/* Log status */
static bool DEBUG_CDEB = true;	/* Major debug log */
static bool DEBUG_DBG = true;	/* General debug log */
static bool DEBUG_REG = true;	/* I2C register debug log */
static bool DEBUG_API = true;	/* API debug log */
static bool DEBUG_FUN = true;	/* Function debug log */
static bool DEBUG_ERR = true;	/* Error debug log */
#else
/* Log status */
static bool DEBUG_CDEB = true;	/* Major debug log */
static bool DEBUG_DBG = true;	/* General debug log */
static bool DEBUG_REG = false;	/* I2C register debug log */
static bool DEBUG_API = false;	/* API debug log */
static bool DEBUG_FUN = false;	/* Function debug log */
static bool DEBUG_ERR = true;	/* Error debug log */
#endif

/** @brief Display log
*	
*	@param tag The tag which you want to show before log
*	@param fmt
*	@param args
*
*/
#define MAXDISPLAYSIZE	128
void display_log(const char* tag, const char* fmt, va_list args){

	int len = 0;
	char log[MAXDISPLAYSIZE];

	//printk("%s: Enter\n", __func__);
	
	/* Copy log */
	len = vsnprintf(log, MAXDISPLAYSIZE, fmt, args);
	if(len <= 0)
	{
		pr_err("%s: format log failed %d\n", __func__, __LINE__);
		return;
	}
	else if(len >= MAXDISPLAYSIZE)
		log[MAXDISPLAYSIZE-1] = '\0';
	else
		log[len] = '\0';
	printk("[%s] %s", tag, log);

	//printk("%s: Exit\n", __func__);
	
}	

/** @brief Handle log
*	
*	@param log_type The type of log
*	@param fmt
*	@param ...
*
*/
void LOG_Handler(int log_type, const char* fmt, ...){

	va_list args;
	
	va_start(args, fmt);

	switch(log_type){
		case LOG_CDBG:
			/* Major debug log */
			if(DEBUG_CDEB){
				display_log(TAG, fmt, args);
			}
			break;
		case LOG_DBG:
			/* General debug log */
			if(DEBUG_DBG){
				display_log(TAG, fmt, args);
			}
			break;
		case LOG_REG:
			/* I2C register read/write debug log */
			if(DEBUG_REG){
				display_log(TAG, fmt, args);
			}
			break;
		case LOG_API:
			/* API debug log */
			if(DEBUG_API){
				display_log(TAG, fmt, args);
			}
			break;
		case LOG_FUN:
			/* Function debug log */
			if(DEBUG_FUN){
				display_log(TAG, fmt, args);
			}
			break;
		case LOG_ERR:
			/* Error debug log */
			if(DEBUG_ERR){
				display_log(TAG, fmt, args);
			}
			break;
		default:
			LOG_Handler(LOG_ERR, "%s Type fail(%d) !!\n", __func__, log_type);
			break;
	}

	va_end(args);
	
}

/** @brief Log control
*	
*	@param num the number of log action
*	@param len the size of log number
*
*/
ssize_t Laser_Focus_log_contorl(const char __user *num, size_t len){
	int ctrl = 0;
	char messages[8];

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	if(len > 8){
		len = 8;
	}
	/* Copy fake number to temp buffer */
	if(copy_from_user(messages, num, len)){
		LOG_Handler(LOG_ERR, "%s command fail !!\n", __func__);
		return -EFAULT;
	}

	ctrl = (int)simple_strtol(messages, NULL, 10);

	LOG_Handler(LOG_DBG, "%s command : %d\n", __func__, ctrl);
	switch(ctrl){
		case ALL_ENABLE:
			/* Enable All debug log */
			DEBUG_CDEB = true;
			DEBUG_DBG = true;
			DEBUG_REG = true;
			DEBUG_API = true;
			DEBUG_FUN = true;
			DEBUG_ERR = true;
			break;
		case ALL_DISABLE:
			/* Disable All debug log */
			DEBUG_CDEB = false;
			DEBUG_DBG = false;
			DEBUG_REG = false;
			DEBUG_API = false;
			DEBUG_FUN = false;
			DEBUG_ERR = false;
			break;
		case CDBG_ENABLE:
			/* Enable Major debug log */
			DEBUG_CDEB = true;
			break;
		case CDBG_DISABLE:
			/* Disable Major debug log */
			DEBUG_CDEB = false;
			break;
		case DBG_ENABLE:
			/* Enable General debug log */
			DEBUG_DBG = true;
			break;
		case DBG_DISABLE:
			/* Disable General debug log */
			DEBUG_DBG = false;
			break;
		case REG_RW_DBG_ENABLE:
			/* Enable I2C register read/write debug log */
			DEBUG_REG = true;
			break;
		case REG_RW_DBG_DISABLE:
			/* Disable I2C register read/write debug log */
			DEBUG_REG = false;
			break;
		case API_DBG_ENABLE:
			/* Enable API debug log */
			DEBUG_API = true;
			break;
		case API_DBG_DISABEL:
			/* Disable API debug log */
			DEBUG_API = false;
			break;
		case FUN_DBG_ENABLE:
			/* Enable Function debug log */
			DEBUG_FUN = true;
			break;
		case FUN_DBG_DISABLE:
			/* Disable Function debug log */
			DEBUG_FUN = false;
			break;
		case ERR_DGB_ENABLE:
			/* Enable Error debug log */
			DEBUG_ERR = true;
			break;
		case ERR_DBG_DISABLE:
			/* Disable Error debug log */
			DEBUG_ERR = false;
			break;
		default:
			LOG_Handler(LOG_ERR, "%s command fail(%d) !!\n", __func__, ctrl);
			return -EINVAL;
	}

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return len;
}
