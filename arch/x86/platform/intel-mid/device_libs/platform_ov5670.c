/*
 * platform_ov5670.c: ov5670 platform data initilization file
 *
 * (C) Copyright 2013 Intel Corporation
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
#include <linux/mfd/intel_mid_pmic.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_INTEL_SOC_PMC
#include <asm/intel_soc_pmc.h>
#endif
#include <linux/lnw_gpio.h>
#include "platform_camera.h"
#include "platform_ov5670.h"
#include <asm/intel_scu_pmic.h>
#include <linux/HWVersion.h>
#include <asm/intel-mid.h>

/* workround - pin defined for cht */
#define CAMERA_0_RESET	150
#define CAMERA_1P2_EN	153
#ifdef CONFIG_INTEL_SOC_PMC
#define OSC_CAM0_CLK 0x0
#define CLK_19P2MHz 0x1
/* workaround - use xtal for cht */
#define CLK_19P2MHz_XTAL 0x0
#define CLK_ON	0x1
#define CLK_OFF	0x2
#endif

#define MSIC_VPROG1_MRFLD_CTRL       (0xac)
#define MSIC_VPROG1_ON_2P8           (0xc1)
#define MSIC_VPROG1_ON_1P8           (0x41)
#define MSIC_VPROG1_OFF              (0x0)
#define MSIC_VPROG2_MRFLD_CTRL       (0xad)
#define MSIC_VPROG2_ON_1P8           (0x41)
#define MSIC_VPROG2_OFF              (0x0)
static int camera_vprog2_on = 0;
static int camera_vprog1_on = 0;
static int camera_1p2_en = -1;
static int xshutdown = -1;
static int camera_2v8 = -1;
static int camera_3p3_en2 = -1;

/*
 * OV5670 platform data
 */

static int ov5670_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;
	/*
	 * FIXME: WA using hardcoded GPIO value here.
	 * The GPIO value would be provided by ACPI table, which is
	 * not implemented currently.
	 */
   if (xshutdown < 0) {
        ret = camera_sensor_gpio(-1,"SUB_CAM_PWDN", GPIOF_DIR_OUT, 0);
        if (ret < 0){
            printk("SUB_CAM_PWDN not available.\n");
            return ret;
        }
        xshutdown = ret;
        printk(KERN_INFO "ov5670, gpio number, xshutdown is %d\n", xshutdown);
    }

    if(flag){
	if (xshutdown >= 0)
		gpio_set_value(xshutdown, 1);
    }else{
	if (xshutdown >= 0)
		gpio_set_value(xshutdown, 0);
    }
    return 0;
}

/*
 * WORKAROUND:
 * This func will return 0 since MCLK is enabled by BIOS
 * and will be always on event if set MCLK failed here.
 * TODO: REMOVE WORKAROUND, err should be returned when
 * set MCLK failed.
 */
static int ov5670_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;

#ifdef CONFIG_INTEL_SOC_PMC
	int ret = 0;
	if (flag) {
		ret = pmc_pc_set_freq(OSC_CAM0_CLK, (IS_CHT) ?
				CLK_19P2MHz_XTAL : CLK_19P2MHz);
		if (ret)
			pr_err("ov5670 clock set failed.\n");
	}
	pmc_pc_configure(OSC_CAM0_CLK, flag ? CLK_ON : CLK_OFF);
	return 0;
#elif defined(CONFIG_INTEL_SCU_IPC_UTIL)
	int ret;
    ret = intel_scu_ipc_osc_clk(OSC_CLK_CAM1, flag ? clock_khz : 0);
    msleep(1);
    return ret;

#else
	pr_err("ov5670 clock is not set.\n");
	return 0;
#endif

}

/*
 * The camera_v1p8_en gpio pin is to enable 1.8v power.
 */
static int ov5670_power_ctrl(struct v4l2_subdev *sd, int flag)
{
    int ret = 0;

    printk("@%s PROJECT_ID = 0x%x, HW_ID = 0x%x\n", __func__, Read_PROJ_ID(), Read_HW_ID());

    if (camera_1p2_en < 0) {
        ret = camera_sensor_gpio(-1,"INT_CAM_1V2_EN", GPIOF_DIR_OUT, 0);
        if (ret < 0){
            printk("camera_1p2_en not available.\n");
            return ret;
        }
        camera_1p2_en = ret;
        printk(KERN_INFO "ov5670, gpio number, camera_1p2_en is %d\n", camera_1p2_en);
    }

    if (camera_2v8 < 0) {
        ret = camera_sensor_gpio(-1, "INT_CAM_2V8_EN", GPIOF_DIR_OUT, 0);
        if (ret < 0){
            printk("INT_CAM_2V8_EN not available.\n");
            return ret;
        }
        camera_2v8 = ret;
        printk("<< camera_2v8:%d, flag:%d\n", camera_2v8, flag);
    }


	if (camera_3p3_en2 < 0) {
		gpio_free(58);/////// temp WA.
        	lnw_gpio_set_alt(58, LNW_GPIO);
        	ret = camera_sensor_gpio(58, "3X_I2C_LED", GPIOF_DIR_OUT, 0);
            if (ret < 0){
            	printk("GPIO58 is not available.\n");
            }else{
            	camera_3p3_en2 = ret;
            	printk(KERN_INFO "ov5670, gpio number, camera_3p3_en2 is %d\n", camera_3p3_en2);
            }
    	}

    if (flag) {
        switch (Read_PROJ_ID()) {
            case PROJ_ID_ZX550ML:

        		if(camera_3p3_en2 > 0){
            		    mdelay(1);
            		    printk("@%s %d, project zx550ml pull up GPIO%d\n", __func__, __LINE__, camera_3p3_en2);
            		    gpio_set_value(camera_3p3_en2, 1);
			}

                        //turn on power 1.8V
                        if (!camera_vprog2_on) {
                            camera_vprog2_on = 1;
                            ret = intel_scu_ipc_iowrite8(MSIC_VPROG1_MRFLD_CTRL, MSIC_VPROG1_ON_1P8);
                            if (ret){
                                printk(KERN_INFO "set vprog2 fails\n");
                                return -1;
                            }
                            msleep(1);
                        }
                        pr_info("ov5670 --> HW_ID = 0x%x\n", Read_HW_ID());
                        //turn on power 2.8V
                        if (camera_2v8 >= 0){
                            gpio_set_value(camera_2v8, 1);
                            printk(KERN_ALERT "ov5670 <<< camera_2v8 = 1\n");
                        }



                 break;
            case PROJ_ID_ZE550ML:
            case PROJ_ID_ZE551ML:
            case PROJ_ID_ZE551ML_CKD:
                switch (Read_HW_ID()) {
                    case HW_ID_EVB:
                        pr_info("Hardware VERSION = EVB, ov5670 does not support.\n");
                        break;
                    case HW_ID_SR1:
                    case HW_ID_SR2:
                    case HW_ID_ER:
                    case HW_ID_ER1_1:
                    case HW_ID_ER1_2:
                    case HW_ID_PR:
		    case HW_ID_pre_PR:
                    case HW_ID_MP:
                        pr_info("ov5670 --> HW_ID = 0x%x\n", Read_HW_ID());
                        //turn on power 2.8V
                        if (camera_2v8 >= 0){
                            gpio_set_value(camera_2v8, 1);
                            printk(KERN_ALERT "ov5670 <<< camera_2v8 = 1\n");
                        }

                        //turn on power 1.8V
                        if (!camera_vprog2_on) {
                            camera_vprog2_on = 1;
                            ret = intel_scu_ipc_iowrite8(MSIC_VPROG2_MRFLD_CTRL, MSIC_VPROG2_ON_1P8);
                            if (ret){
                                printk(KERN_INFO "set vprog2 fails\n");
                                return -1;
                            }
                            msleep(1);
                        }
                        break;
                    default:
                        pr_info("ov5670 --> HW_ID 0x%x is not defined\n", Read_HW_ID());
                        break;
                }
                break;

            case PROJ_ID_ZE500ML:
                switch (Read_HW_ID()) {
                    case HW_ID_EVB:
                        pr_info("ov5670 --> HW_ID = 0x%x\n", Read_HW_ID());
                        //turn on power 1.8V
                        if (!camera_vprog2_on) {
                            camera_vprog2_on = 1;
                            ret = intel_scu_ipc_iowrite8(MSIC_VPROG2_MRFLD_CTRL, MSIC_VPROG2_ON_1P8);
                            if (ret){
                                printk(KERN_INFO "set vprog2 fails\n");
                                return -1;
                            }
                            msleep(1);
                        }

                        //turn on power 2.8V
                        if (!camera_vprog1_on) {
                            camera_vprog1_on = 1;
                            intel_scu_ipc_iowrite8(MSIC_VPROG1_MRFLD_CTRL ,MSIC_VPROG1_ON_2P8);
                            if (ret){
                                printk(KERN_INFO "set vprog1 fails\n");
                                return -1;
                            }
                            msleep(1);
                        }
                        break;
                    case HW_ID_SR1:
                    case HW_ID_SR2:
                    case HW_ID_ER:
		    case HW_ID_pre_PR:
                    case HW_ID_PR:
                    case HW_ID_MP:
                        pr_info("HW_ID 0x%x, ov5670 does not support.\n", Read_HW_ID());
                        break;
                    default:
                        pr_info("ov5670 --> HW_ID 0x%x is not defined\n", Read_HW_ID());
                        break;
                }
                break;

            default:
                pr_info("Project ID is not defined\n");
                break;
        }//end switch

        //turn on power 1.2V
        gpio_set_value(camera_1p2_en, 1);
        printk(KERN_INFO "ov5670---camera_1p2_en is %d\n", camera_1p2_en);
        usleep_range(10000, 11000);

        //flag == 0
    } else {

                        if (camera_3p3_en2 >= 0){
			    gpio_set_value(camera_3p3_en2, 0);
			    camera_sensor_gpio_free(camera_3p3_en2);
        		    camera_3p3_en2 = -1;
			}

        //turn OFF power 1.2V
        gpio_set_value(camera_1p2_en, 0);
        gpio_free(camera_1p2_en);
        camera_1p2_en = -1;

        switch (Read_PROJ_ID()) {

            case PROJ_ID_ZX550ML:
        		//turn off power 1.8V
        		if (camera_vprog2_on) {
            		    camera_vprog2_on = 0;
            		    ret = intel_scu_ipc_iowrite8(MSIC_VPROG1_MRFLD_CTRL, MSIC_VPROG1_OFF);
            		    if (ret) {
                		printk(KERN_ALERT "Failed to disable regulator vprog2\n");
                		return ret;
            		    }
            		    printk("<<< 1.8V= 0\n");
        		}
                        //turn off power 2.8V
                        if (camera_2v8 >= 0){
                            gpio_set_value(camera_2v8, 0);
                            gpio_free(camera_2v8);
                            camera_2v8 = -1;
                            printk("<<< camera_2v8 = 0\n");
                        }


                break;
            case PROJ_ID_ZE550ML:
            case PROJ_ID_ZE551ML:
            case PROJ_ID_ZE551ML_CKD:
                switch (Read_HW_ID()) {
                    case HW_ID_EVB:
                        pr_info("Hardware VERSION = EVB, ov5670 does not support.\n");
                        break;
                    case HW_ID_SR1:
                    case HW_ID_SR2:
                    case HW_ID_ER:
                    case HW_ID_ER1_1:
                    case HW_ID_ER1_2:
                    case HW_ID_PR:
		    case HW_ID_pre_PR:
                    case HW_ID_MP:
                        pr_info("ov5670 --> HW_ID = 0x%x\n", Read_HW_ID());
                        //turn off power 2.8V
                        if (camera_2v8 >= 0){
                            gpio_set_value(camera_2v8, 0);
                            gpio_free(camera_2v8);
                            camera_2v8 = -1;
                            printk("<<< camera_2v8 = 0\n");
                        }
        		//turn off power 1.8V
        		if (camera_vprog2_on) {
            		    camera_vprog2_on = 0;
            		    ret = intel_scu_ipc_iowrite8(MSIC_VPROG2_MRFLD_CTRL, MSIC_VPROG2_OFF);
            		    if (ret) {
                		printk(KERN_ALERT "Failed to disable regulator vprog2\n");
                		return ret;
            		    }
            		    printk("<<< 1.8V= 0\n");
        		}
                        break;
                    default:
                        pr_info("ov5670 --> HW_ID is not defined\n");
                        break;
                }
                break;

            case PROJ_ID_ZE500ML:
                switch (Read_HW_ID()) {
                    case HW_ID_EVB:
                        pr_info("ov5670 --> HW_ID = 0x%x\n", Read_HW_ID());
                        //turn off power 2.8V
                        if (camera_vprog1_on) {
                            camera_vprog1_on = 0;
                            ret = intel_scu_ipc_iowrite8(MSIC_VPROG1_MRFLD_CTRL, MSIC_VPROG1_OFF);
                            if (ret) {
                                printk(KERN_ALERT "Failed to disable regulator vprog1\n");
                                return ret;
                            }
                            printk("<<< 2.8V = 0\n");
                        }
        		//turn off power 1.8V
        		if (camera_vprog2_on) {
            			camera_vprog2_on = 0;
            			ret = intel_scu_ipc_iowrite8(MSIC_VPROG2_MRFLD_CTRL, MSIC_VPROG2_OFF);
            			if (ret) {
                			printk(KERN_ALERT "Failed to disable regulator vprog2\n");
                		return ret;
            		}
            		printk("<<< 1.8V= 0\n");
                        break;
		}
                    case HW_ID_SR1:
                    case HW_ID_SR2:
                    case HW_ID_ER:
                    case HW_ID_PR:
		    case HW_ID_pre_PR:
                    case HW_ID_MP:
                        pr_info("HW_ID 0x%x, ov5670 does not support.\n", Read_HW_ID());
                        break;
                    default:
                        pr_info("ov5670 --> HW_ID is not defined\n");
                        break;
                }
                break;

            default:
                pr_info("Project ID is not defined\n");
                break;
        }//end switch


    }//end if

    return 0;
}

static int ov5670_csi_configure(struct v4l2_subdev *sd, int flag)
{
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, 2,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_bggr, flag);
}

static int ov5670_platform_init(struct i2c_client *client)
{
    printk("%s: ++\n", __func__);
/*
    //VPROG2 for 1.8V
    vprog2_reg = regulator_get(&client->dev, "vprog2");
    if (IS_ERR(vprog2_reg)) {
        dev_err(&client->dev, "vprog2 failed\n");
        return PTR_ERR(vprog2_reg);
    }
*/
/*
    ret = regulator_set_voltage(vprog2_reg, VPROG2_VAL, VPROG2_VAL);
    if (ret) {
        dev_err(&client->dev, "vprog2 set failed\n");
        regulator_put(vprog2_reg);
    }
*/
    return 0;
}

static int ov5670_platform_deinit(void)
{
    return 0;
}

static struct camera_sensor_platform_data ov5670_sensor_platform_data = {
	.gpio_ctrl	     = ov5670_gpio_ctrl,
	.flisclk_ctrl	 = ov5670_flisclk_ctrl,
	.power_ctrl      = ov5670_power_ctrl,
	.csi_cfg	     = ov5670_csi_configure,
    .platform_init   = ov5670_platform_init,
    .platform_deinit = ov5670_platform_deinit,
};

void *ov5670_platform_data(void *info)
{
    camera_1p2_en = -1;
    xshutdown     = -1;
    camera_vprog2_on = 0;
    camera_vprog1_on = 0;
    camera_2v8 = -1;
    return &ov5670_sensor_platform_data;
}
