/*
 * platform_t4k37.c: t4k37 platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/atomisp_platform.h>
#include <asm/intel_scu_ipcutil.h>
#include <asm/intel-mid.h>
#include <media/v4l2-subdev.h>
#include <linux/regulator/consumer.h>
#include "platform_camera.h"
#include "platform_t4k37.h"
#include <linux/lnw_gpio.h>
#include <asm/intel_scu_pmic.h>
#include <linux/HWVersion.h>
#include <asm/intel-mid.h>

#define GP_CAMERA_0_1P2 "INT_CAM_1V2_EN"
#define MSIC_VPROG2_MRFLD_CTRL	0xAD
#define GP_CAMERA_0_2P8 "INT_CAM_2V8_EN"

static int camera_reset = -1;
static int camera_1v2 = -1;
static int camera_vprog1_on;
static int camera_vprog2_on;
static int camera_2v8 = -1;

/*
 * MRFLD VV primary camera sensor - T4K37 platform data
 */

static int t4k37_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;

	printk("%s: ++\n",__func__);

	if (camera_reset < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_0_RESET, GPIOF_DIR_OUT, 0);
		if (ret < 0){
			printk("camera_reset not available.\n");
			return ret;
		}
		camera_reset = ret;
		printk("<< camera_reset:%d, flag:%d\n", camera_reset, flag);
	}

	if (flag) {
		gpio_set_value(camera_reset, 1);
		printk("<<< camera_reset = 1\n");
	} else {
		gpio_set_value(camera_reset, 0);
		printk("<<< camera_reset = 0\n");
	}

	return 0;
}

static int t4k37_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
#ifdef CONFIG_INTEL_SCU_IPC_UTIL
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM0, flag ? clock_khz : 0);
#else
	pr_err("t4k37 clock is not set.\n");
	return 0;
#endif
}

static int t4k37_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int reg_err;
	int ret = 0;

	printk("%s: ++\n",__func__);

	if (camera_reset < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_0_RESET, GPIOF_DIR_OUT, 0);
		if (ret < 0){
			printk("camera_reset not available.\n");
			return ret;
		}
		camera_reset = ret;
		printk("<< camera_reset:%d, flag:%d\n", camera_reset, flag);
	}


	if (camera_1v2 < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_0_1P2, GPIOF_DIR_OUT, 0);
		if (ret < 0){
			printk("camera_reset not available.\n");
			return ret;
		}
		camera_1v2 = ret;
//		lnw_gpio_set_alt(GP_CAMERA_0_1P2, LNW_GPIO);
		printk("<< camera_1v2:%d, flag:%d\n", camera_1v2, flag);
	}

    switch (Read_PROJ_ID()) {

	case PROJ_ID_ZE550ML:
	case PROJ_ID_ZE551ML:
	case PROJ_ID_ZX550ML:
	case PROJ_ID_ZE551ML_CKD:
	    switch (Read_HW_ID()) {
		case HW_ID_EVB:
			pr_info("Hardware VERSION = EVB, t4k37 does not support.\n");
			break;
		case HW_ID_SR1:
		case HW_ID_SR2:
		case HW_ID_ER:
		case HW_ID_ER1_1:
		case HW_ID_ER1_2:
		case HW_ID_pre_PR:
		case HW_ID_PR:
		case HW_ID_MP:
                        pr_info("t4k37 --> HW_ID = 0x%x\n", Read_HW_ID());
			if (camera_2v8 < 0) {
			    ret = camera_sensor_gpio(-1, GP_CAMERA_0_2P8, GPIOF_DIR_OUT, 0);
			    if (ret < 0){
				printk("GP_CAMERA_0_2P8 not available.\n");
				return ret;
			    }
			    camera_2v8 = ret;
			    printk("<< camera_2v8:%d, flag:%d\n", camera_2v8, flag);
			}
		break;
		default:
			pr_info("t4k37 --> HW_ID does not define\n");
		break;
	    }
	break;
	case PROJ_ID_ZE500ML:
	break;
	default:
		pr_info("Project ID is not defined\n");
	break;
    }//end switch

	if (flag){

		if (camera_reset >= 0){
			gpio_set_value(camera_reset, 0);
			printk("<<< camera_reset = 0\n");
		}

		switch (Read_PROJ_ID()) {
	    	    case PROJ_ID_ZE550ML:
	    	    case PROJ_ID_ZE551ML:
		    case PROJ_ID_ZX550ML:
		    case PROJ_ID_ZE551ML_CKD:
			switch (Read_HW_ID()) {
			    case HW_ID_EVB:
				pr_info("Hardware VERSION = EVB, t4k37 does not support.\n");
				break;
			    case HW_ID_SR1:
			    case HW_ID_SR2:
			    case HW_ID_ER:
			    case HW_ID_ER1_1:
			    case HW_ID_ER1_2:
			    case HW_ID_pre_PR:
			    case HW_ID_PR:
			    case HW_ID_MP:
                        	pr_info("t4k37 --> HW_ID = 0x%x\n", Read_HW_ID());
				//turn on power 2.8V
				if (camera_reset >= 0){
			    		gpio_set_value(camera_2v8, 1);
			    		printk(KERN_ALERT "camera_2v8 = 1\n");
		    		}
				//turn on 2.8V
				if (!camera_vprog1_on) {
			    	  	camera_vprog1_on = 1;
			    		ret = intel_scu_ipc_msic_vprog1(1);
			    		if (reg_err) {
					    printk(KERN_ALERT "Failed to enable regulator vprog1\n");
					    return reg_err;
			    		}
			    	printk("<<< 2.8V = 1\n");
				}
			    break;
			    default:
				pr_info("t4k37 --> HW_ID does not define\n");
			    break;
			}
	      	    break;
	    	    case PROJ_ID_ZE500ML:
			pr_info("t4k37 --> HW_ID_SR1\n");
			//turn on 2.8V
			if (!camera_vprog1_on) {
			    camera_vprog1_on = 1;
			    ret = intel_scu_ipc_msic_vprog1(1);
			    if (reg_err) {
				printk(KERN_ALERT "Failed to enable regulator vprog1\n");
				return reg_err;
			    }
			    printk("<<< 2.8V = 1\n");
			}
	    	    break;
	    	    default:
			pr_info("Project ID is not defined\n");
	    	    break;
		}//end switch

		//turn on power 1.8V
		if (!camera_vprog2_on) {
			camera_vprog2_on = 1;
			ret = intel_scu_ipc_iowrite8(MSIC_VPROG2_MRFLD_CTRL, 0x41);
			if (reg_err) {
				printk(KERN_ALERT "Failed to enable regulator vprog1\n");
				return reg_err;
			}
			printk("<<< 1.8V= 1\n");
		}

		//turn on power 1.2V
		gpio_set_value(camera_1v2, 1);
		printk("<<< camera_1v2 = 1\n");

		usleep_range(2000, 2100); //wait vprog1 and vprog2 from enable to 90% (max:2000us)

	}else{

		if (camera_reset >= 0){
			gpio_free(camera_reset);
			camera_reset = -1;
		}

		//turn off power 1.2V
		if (camera_1v2 >= 0){
			gpio_set_value(camera_1v2, 0);
			gpio_free(camera_1v2);
			camera_1v2 = -1;
			printk("<<< camera_1v2 = 0\n");
		}

		//turn off power 1.8V
		if (camera_vprog2_on) {
			camera_vprog2_on = 0;
			reg_err = intel_scu_ipc_iowrite8(MSIC_VPROG2_MRFLD_CTRL, 0);
			if (reg_err) {
				printk(KERN_ALERT "Failed to disable regulator vprog2\n");
				return reg_err;
			}
			printk("<<< 1.8V= 0\n");
		}

		switch (Read_PROJ_ID()) {
		    case PROJ_ID_ZE550ML:
	            case PROJ_ID_ZE551ML:
		    case PROJ_ID_ZX550ML:
		    case PROJ_ID_ZE551ML_CKD:
			switch (Read_HW_ID()) {
			    case HW_ID_EVB:
				pr_info("Hardware VERSION = EVB, t4k37 does not support.\n");
				break;
			    case HW_ID_SR1:
			    case HW_ID_SR2:
			    case HW_ID_ER:
			    case HW_ID_ER1_1:
			    case HW_ID_ER1_2:
			    case HW_ID_pre_PR:
			    case HW_ID_PR:
			    case HW_ID_MP:
                        	pr_info("t4k37 --> HW_ID = 0x%x\n", Read_HW_ID());
				//turn off power 2.8V
		    		if (camera_2v8 >= 0){
			    	    gpio_set_value(camera_2v8, 0);
			    	    gpio_free(camera_2v8);
			    	    camera_2v8 = -1;
			    	    printk("<<< camera_2v8 = 0\n");
				}
		  		//turn off power 2.8V
				if (camera_vprog1_on) {
			    	    camera_vprog1_on = 0;
			   	    reg_err = intel_scu_ipc_msic_vprog1(0);
			    	    if (reg_err) {
				    	printk(KERN_ALERT "Failed to disable regulator vprog1\n");
				    	return reg_err;
			    	    }
				printk("<<< 2.8V = 0\n");
				}
			    break;
			    default:
				pr_info("t4k37 --> HW_ID does not define\n");
			    break;
			}
		    break;
		    case PROJ_ID_ZE500ML:
			pr_info("t4k37 --> HW_ID_SR1\n");
		  	//turn off power 2.8V
			if (camera_vprog1_on) {
			    camera_vprog1_on = 0;
			    reg_err = intel_scu_ipc_msic_vprog1(0);
			    if (reg_err) {
				printk(KERN_ALERT "Failed to disable regulator vprog1\n");
				return reg_err;
			    }
			printk("<<< 2.8V = 0\n");
			}
	    	    break;
	    	    default:
			pr_info("Project ID is not defined\n");
	    	    break;
		}//end switch

	}

	return 0;

}
static int t4k37_platform_init(struct i2c_client *client)
{
	return 0;
}

static int t4k37_platform_deinit(void)
{
	return 0;
}

static int t4k37_csi_configure(struct v4l2_subdev *sd, int flag)
{
	static const int LANES = 4;
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY, LANES,
		//ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_rggb, flag);
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_grbg, flag);
		//ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_bggr, flag);
		//ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_gbrg, flag);
}


static struct camera_sensor_platform_data t4k37_sensor_platform_data = {
	.gpio_ctrl      = t4k37_gpio_ctrl,
	.flisclk_ctrl   = t4k37_flisclk_ctrl,
	.power_ctrl     = t4k37_power_ctrl,
	.csi_cfg        = t4k37_csi_configure,
	.platform_init = t4k37_platform_init,
	.platform_deinit = t4k37_platform_deinit,
};

void *t4k37_platform_data(void *info)
{

	camera_reset = -1;
	camera_1v2 = -1;
	camera_2v8 = -1;
	camera_vprog2_on = 0;
	camera_vprog1_on = 0;

	return &t4k37_sensor_platform_data;
}
