/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-05
*
*/

#include "show_parser.h"
#include "show_log.h"

#if 0
/** @brief Parse GPIO information from dtsi
*	
*	@param of_node the device node
*	@param sensordata the sensor board information
*
*/
int32_t dtsi_gpio_parser(struct device_node *of_node, struct msm_camera_sensor_board_info *sensordata)
{
	int i = 0;
	int32_t rc = 0;
	struct msm_camera_gpio_conf *gconf = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	uint16_t *gpio_array = NULL;
	uint16_t gpio_array_size = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	power_info = &sensordata->power_info;
	
	power_info->gpio_conf = kzalloc(sizeof(struct msm_camera_gpio_conf), GFP_KERNEL);
	if(!power_info->gpio_conf){
		LOG_Handler(LOG_ERR, "%s: failed %d\n", __func__, __LINE__);
		rc = -ENOMEM;
		return rc;
	}

	gconf = power_info->gpio_conf;
	
	gpio_array_size = of_gpio_count(of_node);
	LOG_Handler(LOG_DBG, "%s: gpio count %d\n", __func__, gpio_array_size);

	if(gpio_array_size){
		gpio_array = kzalloc(sizeof(uint16_t) * gpio_array_size, GFP_KERNEL);
		if(!gpio_array){
			LOG_Handler(LOG_ERR, "%s: failed %d\n", __func__, __LINE__);
			kfree(gconf);
			rc = -ENOMEM;
			return rc;
		}
		
		for(i=0; i < gpio_array_size; i++){
			gpio_array[i] = of_get_gpio(of_node, i);
			LOG_Handler(LOG_DBG, "%s: gpio_array[%d] = %d\n", __func__, i, gpio_array[i]);
		}

		rc = msm_camera_get_dt_gpio_req_tbl(of_node, gconf, gpio_array, gpio_array_size);
		if(rc < 0){
			LOG_Handler(LOG_ERR, "%s: failed %d\n", __func__, __LINE__);
			kfree(gconf);
			return rc;
		}

		rc = msm_camera_get_dt_gpio_set_tbl(of_node, gconf, gpio_array, gpio_array_size);
		if(rc < 0){
			LOG_Handler(LOG_ERR, "%s: failed %d\n", __func__, __LINE__);
			kfree(gconf->cam_gpio_req_tbl);
			return rc;
		}

		rc = msm_camera_init_gpio_pin_tbl(of_node, gconf, gpio_array, gpio_array_size);
		if(rc < 0){
			LOG_Handler(LOG_ERR, "%s: failed %d\n", __func__, __LINE__);
			kfree(gconf->cam_gpio_set_tbl);
			return rc;
		}
	}
	kfree(gpio_array);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	
	return rc;
}

/** @brief Parse information from dtsi
*	
*	@param of_node the device node
*	@param dev_t the laser focus controller
*
*/
int32_t get_dtsi_data(struct device_node *of_node, struct msm_laser_focus_ctrl_t *dev_t)
{
	int32_t rc = 0;
	struct msm_camera_sensor_board_info *sensordata = NULL;
	uint32_t id_info[3];
	struct msm_laser_focus_vreg *vreg_cfg = NULL;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/* Check device node */
	if (!of_node) {
		LOG_Handler(LOG_ERR, "%s: of_node NULL\n", __func__);
		return -EINVAL;
	}

	dev_t->sensordata = kzalloc(sizeof(struct msm_camera_sensor_board_info), GFP_KERNEL);
	if (!dev_t->sensordata) {
		LOG_Handler(LOG_ERR, "%s: failed %d\n", __func__, __LINE__);
		return -ENOMEM;
	}

	sensordata = dev_t->sensordata;

	/* Get subdev id information */
	rc = of_property_read_u32(of_node, "cell-index", &dev_t->subdev_id);
	if (rc < 0) {
		LOG_Handler(LOG_ERR, "%s: failed\n", __func__);
		return -EINVAL;
	}
	LOG_Handler(LOG_DBG, "%s: subdev id %d\n", __func__, dev_t->subdev_id);

	/* Get label(sensor name) information */
	rc = of_property_read_string(of_node, "label", &sensordata->sensor_name);
	if (rc < 0) {
		LOG_Handler(LOG_ERR, "%s: failed %d\n", __func__, __LINE__);
		goto ERROR;
	}
	LOG_Handler(LOG_DBG, "%s: label %s, rc %d\n", __func__, sensordata->sensor_name, rc);

	/* Get cci master information */
	rc = of_property_read_u32(of_node, "qcom,cci-master", &dev_t->cci_master);
	if (rc < 0) {
		/* Set default master 0 */
		dev_t->cci_master = MASTER_0;
		rc = 0;
	}
	LOG_Handler(LOG_DBG, "%s: qcom,cci-master %d, rc %d\n", __func__, dev_t->cci_master, rc);

	/* Get voltage information */
	if (of_find_property(of_node, "qcom,cam-vreg-name", NULL)) {
		vreg_cfg = &dev_t->vreg_cfg;
		rc = msm_camera_get_dt_vreg_data(of_node, &vreg_cfg->cam_vreg, &vreg_cfg->num_vreg);
		if (rc < 0) {
			kfree(dev_t);
			LOG_Handler(LOG_ERR, "%s: failed rc %d\n", __func__, rc);
			return rc;
		}
	}

	sensordata->slave_info = kzalloc(sizeof(struct msm_camera_slave_info), GFP_KERNEL);
	if (!sensordata->slave_info) {
		LOG_Handler(LOG_ERR, "%s: failed %d\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto ERROR;
	}

	/* Get slave information */
	rc = of_property_read_u32_array(of_node, "qcom,slave-id", id_info, 3);
	if (rc < 0) {
		LOG_Handler(LOG_ERR, "%s: failed %d\n", __func__, __LINE__);
		goto ERROR;
	}
	dev_t->sensordata->slave_info->sensor_slave_addr = id_info[0];
	dev_t->sensordata->slave_info->sensor_id_reg_addr = id_info[1];
	dev_t->sensordata->slave_info->sensor_id = id_info[2];

	LOG_Handler(LOG_DBG, "%s: slave addr 0x%x sensor reg 0x%x id 0x%x\n", __func__,
		dev_t->sensordata->slave_info->sensor_slave_addr,
		dev_t->sensordata->slave_info->sensor_id_reg_addr,
		dev_t->sensordata->slave_info->sensor_id);

	/* Handle GPIO (e.g. CAM_1V2_EN) */
/*	rc = dtsi_gpio_parser(of_node, sensordata);
	if(rc < 0){
		LOG_Handler(LOG_ERR, "%s: failed %d\n", __func__, __LINE__);
		goto ERROR;
	}*/

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return rc;

ERROR:
	kfree(dev_t->sensordata->slave_info);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return rc;
}
#endif

