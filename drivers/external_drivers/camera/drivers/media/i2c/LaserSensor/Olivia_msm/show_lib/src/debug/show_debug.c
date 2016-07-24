/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-06
*
*/

#include "show_debug.h"
#include "show_log.h"

/** @brief Dump register value from min to max (Word)
*	
*	@param vfile virtual file which provide information to the user or system administrator
*	@param dev_t the laser focus controller
*	@param min the least number of register
*	@param max the maximum number of register
*
*/
int dump_register(struct seq_file *vfile, struct msm_laser_focus_ctrl_t *dev_t, int min, int max){
	int status, i = 0;
	uint16_t register_value = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	if (dev_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF ||
		dev_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI) {
		LOG_Handler(LOG_ERR, "%s: Device without turn on: (%d) \n", __func__, dev_t->device_state);
		return -EBUSY;
	}

	/* Polling to get register value */
	for (i = min; i < max; i++){
		register_value = 0;
		/* Read register */
		status = CCI_I2C_RdWord(dev_t, i, &register_value);
		LOG_Handler(LOG_DBG, "%s: read register(0x%x): 0x%x for word\n", __func__, i, register_value);
		seq_printf(vfile, "read register(0x%x): 0x%x for word\n", i, register_value);
		if (status < 0) {
			LOG_Handler(LOG_ERR, "%s: read register(0x%x) failed\n", __func__, i);
			return status;
		}
	}

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	
	return 0;
}

/** @brief Make laser driver return a fake value (0: return real range, others: return fake value)
*	
*	@param dev_t the laser focus controller
*	@param num the fake number
*	@param len the size of fake number
*	@param LF_enforce_ctrl the internal enforce controller 
*
*/
ssize_t Laser_Focus_enforce(struct msm_laser_focus_ctrl_t *dev_t, const char __user *num, size_t len, int *LF_enforce_ctrl){
	char messages[8];

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	if (dev_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF ||
		dev_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI) {
		LOG_Handler(LOG_ERR, "%s: Device without turn on: (%d) \n", __func__, dev_t->device_state);
	}

	if(len > 8){
		len = 8;
	}
	/* Copy fake number to temp buffer */
	if(copy_from_user(messages, num, len)){
		LOG_Handler(LOG_ERR, "%s: command fail !!\n", __func__);
		return -EFAULT;
	}
	/* Assign fake number to internal enforce contorller */
	*LF_enforce_ctrl = (int)simple_strtol(messages, NULL, 10);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return len;
}