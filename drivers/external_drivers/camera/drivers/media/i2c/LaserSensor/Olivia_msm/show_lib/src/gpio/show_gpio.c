/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-05
*
*/

#include "show_gpio.h"
#include "show_log.h"

/** @brief Set GPIO to high
*	
*	@param power_info power controller for camera
*	@param gpio_num the number of gpio
*
*/
#if 0
int GPIO_UP(struct msm_camera_power_ctrl_t *power_info, int gpio_num){
	int rc = 0;

	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, GPIO_HIGH);
	if(rc < 0){
		LOG_Handler(LOG_ERR, "%s: request gpio failed\n", __func__);
		return rc;
	}
		
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->gpio_num[gpio_num],
		GPIO_OUT_HIGH
	);

	return rc;
}

/** @brief Set GPIO to low
*
*	@param power_info power controller for camera
*	@param gpio_num the number of gpio
*
*/
int GPIO_DOWN(struct msm_camera_power_ctrl_t *power_info, int gpio_num){
	int rc = 0;

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->gpio_num[gpio_num],
		GPIO_OUT_LOW
	);

	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, GPIO_LOW);
	if(rc < 0){
		LOG_Handler(LOG_ERR, "%s: request gpio failed\n", __func__);
		return rc;
	}

	return rc;
}

/** @brief Handle GPIO
*	
*	@param dev_t the laser focus controller
*	@param gpio_num the number of gpio
*	@param ctrl the action of gpio 
*			GPIO_HIGH	:	set GPIO to high
*			GPIO_LOW	:	set GPIO to ligh
*/
int GPIO_Handler(struct msm_laser_focus_ctrl_t *dev_t, int gpio_num, int ctrl){
	int rc = 0;
	
	struct msm_camera_sensor_board_info *sensordata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	if (!dev_t) {
		LOG_Handler(LOG_ERR, "%s: failed\n", __func__);
		return -EINVAL;
	}
	
	sensordata = dev_t->sensordata;
	power_info = &sensordata->power_info;

	if(power_info->gpio_conf->cam_gpiomux_conf_tbl != NULL){
		LOG_Handler(LOG_ERR, "%s: mux install\n", __func__);
	}

	switch(ctrl){
		case GPIO_HIGH:
			/* SET GPIO HIGH */
			rc = GPIO_UP(power_info, gpio_num);
			if(rc < 0){
				LOG_Handler(LOG_ERR, "%s: request gpio failed\n", __func__);
				return rc;
			}
			break;
		case GPIO_LOW:
			/* SET GPIO LOW */
			rc = GPIO_DOWN(power_info, gpio_num);
			if(rc < 0){
				LOG_Handler(LOG_ERR, "%s: request gpio failed\n", __func__);
				return rc;
			}
			break;
		default:
			LOG_Handler(LOG_ERR, "%s: Invalid argument\n", __func__);
			return -EINVAL;
	}

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	
	return rc;
}
#endif

