/*
 * board-blackbay.c: Intel Medfield based board (Blackbay)
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/scatterlist.h>
#include <linux/sfi.h>
#include <linux/intel_pmic_gpio.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/mfd/intel_msic.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/i2c-gpio.h>

#include <asm/setup.h>
#include <asm/mpspec_def.h>
#include <asm/hw_irq.h>
#include <asm/apic.h>
#include <asm/io_apic.h>
#include <asm/intel-mid.h>
#include <asm/intel_mid_vrtc.h>
#include <linux/io.h>
#include <asm/i8259.h>
#include <asm/intel_scu_ipc.h>
#include <asm/apb_timer.h>
#include <linux/reboot.h>

/*
 * IPC devices
 */
#include "device_libs/platform_ipc.h"
#include "device_libs/platform_pmic_gpio.h"
#include "device_libs/platform_msic.h"
#include "device_libs/platform_msic_battery.h"
#include "device_libs/platform_msic_gpio.h"
#include "device_libs/platform_msic_audio.h"
#include "device_libs/platform_msic_power_btn.h"
#include "device_libs/platform_msic_ocd.h"
#include "device_libs/platform_msic_vdd.h"
#include "device_libs/platform_mrfl_pmic.h"
#include "device_libs/platform_mrfl_pmic_i2c.h"
#include "device_libs/platform_mrfl_ocd.h"
#include "device_libs/platform_msic_thermal.h"
#include "device_libs/platform_soc_thermal.h"
#include "device_libs/platform_msic_adc.h"
#include "device_libs/platform_bcove_adc.h"
#include "device_libs/platform_scale_adc.h"
#include <asm/platform_mrfld_audio.h>
#include <asm/platform_ctp_audio.h>
#include "device_libs/platform_mrfl_thermal.h"
#include "device_libs/platform_moor_thermal.h"
#include "device_libs/platform_scu_log.h"

/*
 * I2C devices
 */
#include "device_libs/platform_max7315.h"
#include "device_libs/platform_tca6416.h"
#include "device_libs/platform_mpu3050.h"
#include "device_libs/platform_emc1403.h"
#include "device_libs/platform_lis331.h"
#include "device_libs/platform_mpu3050.h"
#include "device_libs/platform_tc35876x.h"
#include "device_libs/platform_rmi4.h"
#include "device_libs/platform_bq24192.h"
#include "device_libs/platform_bq24261.h"
#include "device_libs/platform_r69001.h"
#include "device_libs/platform_pn544.h"
#include "device_libs/platform_fdp.h"
#include "device_libs/platform_l3g4200d.h"
#include "device_libs/platform_lis3dh.h"
#include "device_libs/platform_lsm303.h"
#include "device_libs/platform_apds990x.h"
#include "device_libs/platform_a1026.h"
#include "device_libs/platform_pca9574.h"
#include "device_libs/platform_hx8528_me372cl.h"
#include "device_libs/platform_cyttsp5.h"
#include "device_libs/platform_kl03_hinge.h"
#ifdef CONFIG_TOUCHSCREEN_FTXXXX
#include "device_libs/platform_ftxxxx.h"
#endif
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_I2C_DELUXE
#include "device_libs/platform_synaptics_dsx.h"
#endif
#include "device_libs/platform_bcm2079x.h"

#ifdef CONFIG_SENSORS_CM36686
#include "device_libs/platform_cm36686.h"
#include <linux/cm36686.h>
 #endif
#ifdef CONFIG_SENSOR_AP3045
#include "device_libs/platform_ap3045.h"
#endif
/* SW devices */
#include "device_libs/platform_panel.h"

#include "device_libs/platform_wm8994.h"
#include <asm/platform_cs42l73.h>

#include "device_libs/platform_arizona.h"
#include "device_libs/platform_camera.h"
#include "device_libs/platform_t4k37.h"
#include "device_libs/platform_mt9e013.h"
#include "device_libs/platform_mt9d113.h"
#include "device_libs/platform_mt9m114.h"
#include "device_libs/platform_lm3554.h"
#include "device_libs/platform_lm3642.h"
#include "device_libs/platform_mt9v113.h"
#include "device_libs/platform_ov5640.h"
#include "device_libs/platform_imx208.h"
#include "device_libs/platform_imx175.h"
#include "device_libs/platform_imx135.h"
#include "device_libs/platform_imx219.h"
#include "device_libs/platform_imx134.h"
#include "device_libs/platform_imx132.h"
#include "device_libs/platform_imx227.h"
#include "device_libs/platform_s5k8aay.h"
#include "device_libs/platform_s5k6b2yx.h"
#include "device_libs/platform_ov9724.h"
#include "device_libs/platform_ov2722.h"
#include "device_libs/platform_gc2235.h"
#include "device_libs/platform_leds_lm3642.h"
#include "device_libs/platform_lm3559.h"
#include "device_libs/platform_ov8830.h"
#include "device_libs/platform_ov8858.h"
#include "device_libs/platform_ov5693.h"
#include "device_libs/platform_wm5102.h"
#include "device_libs/platform_ap1302.h"
#include "device_libs/platform_ov680.h"
#include "device_libs/platform_csi_xactor.h"
#include "device_libs/platform_m10mo.h"
#include "device_libs/platform_m12mo.h"
#include "device_libs/platform_pixter.h"
#include "device_libs/platform_ov5670.h"
#include "device_libs/platform_ar0543_raw.h"
#include "device_libs/platform_hm2056_raw.h"

/* LED */
#ifdef CONFIG_LEDS_ASUS
	#include "device_libs/platform_leds_asus.h"
#endif

/*
 * SPI devices
 */
#include "device_libs/platform_max3111.h"
#include "device_libs/platform_max17042.h"
#include "device_libs/platform_xmm2230.h"

/* HSI devices */
#include "device_libs/platform_hsi_modem.h"
#include "device_libs/platform_ffl_modem.h"
#include "device_libs/platform_edlp_modem.h"
#include "device_libs/platform_edlp_fast.h"
#include "device_libs/platform_logical_modem.h"

/* SW devices */
#include "device_libs/platform_modem_ctrl.h"

/* WIFI devices */
#include "device_libs/platform_wl12xx.h"
#include "device_libs/platform_wifi.h"

/* UART devices */
#include "device_libs/platform_gps.h"
#ifdef CONFIG_ME372CG_BATTERY_SMB345
#include "device_libs/platform_me372cg_smb345.h"
#include "device_libs/platform_bq27520.h"
#endif
/* USB devices */
#include "device_libs/pci/platform_usb_otg.h"
/* Flash IC devices*/
#include "device_libs/platform_sky81296.h"
/* Flash IC devices*/
#include "device_libs/platform_flashnode.h"
/* Chih-Hsuan add Charger*/
#ifdef CONFIG_SMB1357_CHARGER
	#include "device_libs/platform_smb1357.h"
#endif

static void __init *no_platform_data(void *info)
{
	return NULL;
}

struct devs_id __initconst device_ids[] = {
	/* UART devices */
	{"bcm4752", SFI_DEV_TYPE_UART, 0, &intel_mid_gps_device_init, NULL},
	{"bcm47521", SFI_DEV_TYPE_UART, 0, &intel_mid_gps_device_init, NULL},
	{"bcm47531", SFI_DEV_TYPE_UART, 0, &intel_mid_gps_device_init, NULL},
	{"bcm4774", SFI_DEV_TYPE_UART, 0, &intel_mid_gps_device_init, NULL},
	{"cg2000", SFI_DEV_TYPE_UART, 0, &intel_mid_gps_device_init, NULL},
	{"csrg05t", SFI_DEV_TYPE_UART, 0, &intel_mid_gps_device_init, NULL},

	/* SD devices */
	{"wl12xx_clk_vmmc", SFI_DEV_TYPE_SD, 0, &wl12xx_platform_data, NULL},
	{"bcm43xx_clk_vmmc", SFI_DEV_TYPE_SD, 0, &wifi_platform_data, NULL},
	{"bcm43xx_vmmc", SFI_DEV_TYPE_SD, 0, &wifi_platform_data, NULL},
	{"iwlwifi_clk_vmmc", SFI_DEV_TYPE_SD, 0, &wifi_platform_data, NULL},
	{"WLAN_FAST_IRQ", SFI_DEV_TYPE_SD, 0, &no_platform_data,
	 &wifi_platform_data_fastirq},

	/* SPI devices */
	{"bma023", SFI_DEV_TYPE_I2C, 1, &no_platform_data, NULL},
	{"pmic_gpio", SFI_DEV_TYPE_SPI, 1, &pmic_gpio_platform_data, NULL},
	{"pmic_gpio", SFI_DEV_TYPE_IPC, 1, &pmic_gpio_platform_data,
					&ipc_device_handler},
	{"spi_max3111", SFI_DEV_TYPE_SPI, 0, &max3111_platform_data, NULL},
	{"spi_xmm2230", SFI_DEV_TYPE_SPI, 0, &xmm2230_platform_data, NULL},
	{"i2c_max7315", SFI_DEV_TYPE_I2C, 1, &max7315_platform_data, NULL},
	{"i2c_max7315_2", SFI_DEV_TYPE_I2C, 1, &max7315_platform_data, NULL},
	{"tca6416", SFI_DEV_TYPE_I2C, 1, &tca6416_platform_data, NULL},
	{"emc1403", SFI_DEV_TYPE_I2C, 1, &emc1403_platform_data, NULL},
	{"i2c_accel", SFI_DEV_TYPE_I2C, 0, &lis331dl_platform_data, NULL},
	{"pmic_audio", SFI_DEV_TYPE_IPC, 1, &no_platform_data,
					&ipc_device_handler},
	{"mpu3050", SFI_DEV_TYPE_I2C, 1, &mpu3050_platform_data, NULL},
	{"i2c_disp_brig", SFI_DEV_TYPE_I2C, 0, &tc35876x_platform_data, NULL},
	{"r69001-ts-i2c", SFI_DEV_TYPE_I2C, 0, &r69001_platform_data, NULL},
	{"synaptics_3202", SFI_DEV_TYPE_I2C, 0, &rmi4_platform_data, NULL},
	{"syn_3400_cgs", SFI_DEV_TYPE_I2C, 0, &rmi4_platform_data, NULL},
	{"syn_3400_igzo", SFI_DEV_TYPE_I2C, 0, &rmi4_platform_data, NULL},
	{"synaptics_3402", SFI_DEV_TYPE_I2C, 0, &rmi4_platform_data, NULL},
	{"cs47l24", SFI_DEV_TYPE_SPI, 0, &arizona_platform_data, NULL},
	{"fingerprint", SFI_DEV_TYPE_SPI, 0, &no_platform_data, NULL},

	/* I2C devices*/
        {"bcm20795", SFI_DEV_TYPE_I2C, 0, &bcm2079x_platform_data, NULL},
	{"max17042", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"max17047", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"max17050", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"bq24192", SFI_DEV_TYPE_I2C, 1, &bq24192_platform_data},
	{"bq24261_charger", SFI_DEV_TYPE_I2C, 1, &bq24261_platform_data, NULL},
	{"pn544", SFI_DEV_TYPE_I2C, 0, &pn544_platform_data, NULL},
	{"fdp", SFI_DEV_TYPE_I2C, 0, &fdp_platform_data, NULL},
	{"l3gd20", SFI_DEV_TYPE_I2C, 0, &l3g4200d_platform_data, NULL},
	{"l3g4200d", SFI_DEV_TYPE_I2C, 0, &l3g4200d_platform_data},
	{"lps331ap", SFI_DEV_TYPE_I2C, 0, &no_platform_data},
	{"accel", SFI_DEV_TYPE_I2C, 0, &lis3dh_platform_data, NULL},
	{"lsm303dl", SFI_DEV_TYPE_I2C, 0, &lsm303dlhc_accel_platform_data},
	{"lsm303cmp", SFI_DEV_TYPE_I2C, 0, &no_platform_data, NULL},
	{"apds990x", SFI_DEV_TYPE_I2C, 0, &apds990x_platform_data},
	{"MNZX8000", SFI_DEV_TYPE_I2C, 0, &no_platform_data, NULL},
	{"pca953x", SFI_DEV_TYPE_I2C, 0, &nxp_pca9574_platform_data, NULL},
	{"hx8528", SFI_DEV_TYPE_I2C, 0, &hx8528_platform_data},
	{"btns_mcu_bl", SFI_DEV_TYPE_I2C, 0, &kl03_hinge_platform_data, NULL},
	{"ak9916", SFI_DEV_TYPE_I2C, 0, &no_platform_data, NULL},
	{"mpu6515", SFI_DEV_TYPE_I2C, 0, &no_platform_data, NULL},
    {"fusb302", SFI_DEV_TYPE_I2C, 0, &no_platform_data, NULL},
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5
	{CYTTSP5_I2C_NAME, SFI_DEV_TYPE_I2C, 0, &cyttsp5_platform_data, NULL},
#endif
	{"leds-lm3642", SFI_DEV_TYPE_I2C, 0, &lm3642_platform_data, NULL},

#ifdef CONFIG_LEDS_ASUS
	{"tca6507", SFI_DEV_TYPE_I2C, 0, &asus_led_platform_data, NULL},
#endif

#ifdef CONFIG_ME372CG_BATTERY_SMB345
#if defined(CONFIG_PF450CL)
	{"smb358", SFI_DEV_TYPE_I2C, 0, &smb345_platform_data},
#else
	{"smb345", SFI_DEV_TYPE_I2C, 0, &smb345_platform_data},
#endif
#endif

#ifdef CONFIG_ME372CG_BATTERY_BQ27520
	{"bq27520", SFI_DEV_TYPE_I2C, 0, &bq27520_platform_data},
#endif


#ifdef CONFIG_PF450CL_BATTERY_BQ27520
	{"bq27520", SFI_DEV_TYPE_I2C, 0, &bq27520_platform_data, NULL},
	{"bq27520f", SFI_DEV_TYPE_I2C, 0, &no_platform_data, NULL},
#endif

#ifdef CONFIG_SMB1357_CHARGER
	{"smb1357_charger", SFI_DEV_TYPE_I2C, 0, &smb1357_platform_data, NULL},
#endif
#ifdef CONFIG_BATTERY_MM8033
	{"mm8033_batt", SFI_DEV_TYPE_I2C, 0, &no_platform_data, NULL},
#endif
#ifdef CONFIG_BATTERY_MAX17058
	{"max17058_batt", SFI_DEV_TYPE_I2C, 0, &no_platform_data, NULL},
#endif
#ifdef CONFIG_CHARGER_SMB1351
	{"smb1351_a", SFI_DEV_TYPE_I2C, 0, &no_platform_data, NULL},
	{"smb1351_b", SFI_DEV_TYPE_I2C, 0, &no_platform_data, NULL},
#endif
#ifdef CONFIG_BATTERY_BQ27742
	{"bq27742", SFI_DEV_TYPE_I2C, 0, &no_platform_data, NULL},
#endif

	/* MSIC subdevices */
	{"msic_adc", SFI_DEV_TYPE_IPC, 1, &msic_adc_platform_data,
						&ipc_device_handler},
	{"bcove_power_btn", SFI_DEV_TYPE_IPC, 1, &msic_power_btn_platform_data,
					&ipc_device_handler},
	{"scove_power_btn", SFI_DEV_TYPE_IPC, 1,
					&msic_power_btn_platform_data,
					&ipc_device_handler},
	{"msic_battery", SFI_DEV_TYPE_IPC, 1, &msic_battery_platform_data,
					&ipc_device_handler},
	{"msic_gpio", SFI_DEV_TYPE_IPC, 1, &msic_gpio_platform_data,
					&ipc_device_handler},
	{"msic_audio", SFI_DEV_TYPE_IPC, 1, &msic_audio_platform_data,
					&ipc_device_handler},
	{"msic_power_btn", SFI_DEV_TYPE_IPC, 1, &msic_power_btn_platform_data,
					&ipc_device_handler},
	{"msic_vdd", SFI_DEV_TYPE_IPC, 1, &msic_vdd_platform_data,
					&ipc_device_handler},
	{"msic_ocd", SFI_DEV_TYPE_IPC, 1, &msic_ocd_platform_data,
					&ipc_device_handler},
	{"bcove_bcu", SFI_DEV_TYPE_IPC, 1, &mrfl_ocd_platform_data,
					&ipc_device_handler},
	{"msic_thermal", SFI_DEV_TYPE_IPC, 1, &msic_thermal_platform_data,
					&ipc_device_handler},
	{"bcove_adc", SFI_DEV_TYPE_IPC, 1, &bcove_adc_platform_data,
					&ipc_device_handler},
	{"scale_adc", SFI_DEV_TYPE_IPC, 1, &scale_adc_platform_data,
					&ipc_device_handler},
	{"bcove_thrm", SFI_DEV_TYPE_IPC, 1, &mrfl_thermal_platform_data,
					&ipc_device_handler},
	{"scove_thrm", SFI_DEV_TYPE_IPC, 1, &moor_thermal_platform_data,
					&ipc_device_handler},
	{"scuLog", SFI_DEV_TYPE_IPC, 1, &scu_log_platform_data,
					&ipc_device_handler},

	/* Panel */
	{"PANEL_CMI_CMD", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PANEL_JDI_VID", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PANEL_JDI_CMD", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
		
	/* Touch */
#ifdef CONFIG_TOUCHSCREEN_FTXXXX
	{"ftxxxx_ts", SFI_DEV_TYPE_I2C, 0, &no_platform_data, NULL},
#endif	
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_I2C_DELUXE
	{"synaptics_dsx_i2c", SFI_DEV_TYPE_I2C, 0, &get_dsx_platformdata, NULL},
#endif
	/*  above 3 items will be removed
	* after firmware changing
	*/
	{"PNC_CMI_7x12", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNV_JDI_7x12", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNC_JDI_7x12", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNC_SHARP_10x19", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNV_SHARP_10x19", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNCD_SHARP_10x19", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNV_SHARP_25x16", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNC_SHARP_25x16", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNV_JDI_25x16", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNC_JDI_25x16", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNC_SDC_16x25", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNC_SDC_25x16", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},

	{"ctp_lt_wm8994", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},

	/* I2C devices for camera image subsystem */
	{"lm3554", SFI_DEV_TYPE_I2C, 0, &lm3554_platform_data_func,
					&intel_register_i2c_camera_device},
	{"lm3642", SFI_DEV_TYPE_I2C, 0, &lm3642_platform_data_func,
					&intel_register_i2c_camera_device},
	{"mt9e013", SFI_DEV_TYPE_I2C, 0, &mt9e013_platform_data,
					&intel_register_i2c_camera_device},
	{"mt9d113", SFI_DEV_TYPE_I2C, 0, &mt9d113_platform_data,
					&intel_register_i2c_camera_device},
	{"mt9m114", SFI_DEV_TYPE_I2C, 0, &mt9m114_platform_data,
					&intel_register_i2c_camera_device},
	{"mt9v113", SFI_DEV_TYPE_I2C, 0, &mt9v113_platform_data,
					&intel_register_i2c_camera_device},
	{"ov8830", SFI_DEV_TYPE_I2C, 0, &ov8830_platform_data,
					&intel_register_i2c_camera_device},
	{"ov8858", SFI_DEV_TYPE_I2C, 0, &ov8858_platform_data,
					&intel_register_i2c_camera_device},
	{"ov5693", SFI_DEV_TYPE_I2C, 0, &ov5693_platform_data,
					&intel_register_i2c_camera_device},
	{"ov5640", SFI_DEV_TYPE_I2C, 0, &ov5640_platform_data,
					&intel_register_i2c_camera_device},
	{"imx208", SFI_DEV_TYPE_I2C, 0, &imx208_platform_data,
					&intel_register_i2c_camera_device},
	{"imx175", SFI_DEV_TYPE_I2C, 0, &imx175_platform_data,
					&intel_register_i2c_camera_device},
	{"imx135", SFI_DEV_TYPE_I2C, 0, &imx135_platform_data,
					&intel_register_i2c_camera_device},
	{"imx219", SFI_DEV_TYPE_I2C, 0, &imx219_platform_data,
					&intel_register_i2c_camera_device},
	{"imx135fuji", SFI_DEV_TYPE_I2C, 0, &imx135_platform_data,
					&intel_register_i2c_camera_device},
	{"imx134", SFI_DEV_TYPE_I2C, 0, &imx134_platform_data,
					&intel_register_i2c_camera_device},
	{"imx132", SFI_DEV_TYPE_I2C, 0, &imx132_platform_data,
					&intel_register_i2c_camera_device},
	{"s5k8aay", SFI_DEV_TYPE_I2C, 0, &s5k8aay_platform_data,
					&intel_register_i2c_camera_device},
	{"s5k6b2yx", SFI_DEV_TYPE_I2C, 0, &s5k6b2yx_platform_data,
					&intel_register_i2c_camera_device},
	{"imx227", SFI_DEV_TYPE_I2C, 0, &imx227_platform_data,
					&intel_register_i2c_camera_device},
	{"ov9724", SFI_DEV_TYPE_I2C, 0, &ov9724_platform_data,
					&intel_register_i2c_camera_device},
	{"ov2722", SFI_DEV_TYPE_I2C, 0, &ov2722_platform_data,
					&intel_register_i2c_camera_device},
	{"gc2235", SFI_DEV_TYPE_I2C, 0, &gc2235_platform_data,
					&intel_register_i2c_camera_device},
	{"lm3559", SFI_DEV_TYPE_I2C, 0, &lm3559_platform_data_func,
					&intel_register_i2c_camera_device},
	{"lm3560", SFI_DEV_TYPE_I2C, 0, &lm3559_platform_data_func,
					&intel_register_i2c_camera_device},
	{"ap1302", SFI_DEV_TYPE_I2C, 0, &ap1302_platform_data,
					&intel_register_i2c_camera_device},
	{"ov680", SFI_DEV_TYPE_I2C, 0, &ov680_platform_data,
					&intel_register_i2c_camera_device},
	{"m10mo", SFI_DEV_TYPE_I2C, 0, &m10mo_platform_data,
					&intel_register_i2c_camera_device},
	{"m12mo", SFI_DEV_TYPE_I2C, 0, &m12mo_platform_data,
					&intel_register_i2c_camera_device},
	{"sky81296", SFI_DEV_TYPE_I2C, 1, &sky81296_platform_data_func,
                                        &intel_register_i2c_camera_device},
	{"flashnode", SFI_DEV_TYPE_I2C, 1, &flashnode_platform_data_func,
					&intel_register_i2c_camera_device},
	{"ov5670", SFI_DEV_TYPE_I2C, 0, &ov5670_platform_data,
                                        &intel_register_i2c_camera_device},
        {"t4k37", SFI_DEV_TYPE_I2C, 0, &t4k37_platform_data,
                    			&intel_register_i2c_camera_device},
	{"ar0543_raw", SFI_DEV_TYPE_I2C, 0, &ar0543_raw_platform_data,
					&intel_register_i2c_camera_device},
	{"hm2056_raw", SFI_DEV_TYPE_I2C, 0, &hm2056_raw_platform_data,
					&intel_register_i2c_camera_device},
#if defined(CONFIG_VIDEO_CSI_XACTOR_MODULE) || defined(CONFIG_VIDEO_CSI_XACTOR)
	{"xactor_a", SFI_DEV_TYPE_I2C, 0, &csi_xactor_a_platform_data,
					&intel_register_i2c_camera_device},
	{"xactor_b", SFI_DEV_TYPE_I2C, 0, &csi_xactor_b_platform_data,
					&intel_register_i2c_camera_device},
	{"xactor_c", SFI_DEV_TYPE_I2C, 0, &csi_xactor_c_platform_data,
					&intel_register_i2c_camera_device},
#endif
	{"pixter_0", SFI_DEV_TYPE_I2C, 0, &pixter_0_platform_data,
					&intel_register_i2c_camera_device},
	{"pixter_1", SFI_DEV_TYPE_I2C, 0, &pixter_1_platform_data,
					&intel_register_i2c_camera_device},
	{"pixter_2", SFI_DEV_TYPE_I2C, 0, &pixter_2_platform_data,
					&intel_register_i2c_camera_device},
	{"audience_es305", SFI_DEV_TYPE_I2C, 0, &audience_platform_data,
						NULL},
	{"wm8994", SFI_DEV_TYPE_I2C, 0, &wm8994_platform_data, NULL},
	{"monzax", SFI_DEV_TYPE_I2C, 0, &no_platform_data, NULL},
	
#ifdef CONFIG_SENSORS_CM36686
    {CM36686_I2C_NAME, SFI_DEV_TYPE_I2C, 0, &cm36686_platform_data, NULL},
 #endif
#ifdef CONFIG_SENSOR_AP3045
	{"ap3045", SFI_DEV_TYPE_I2C, 0, &asus_ap3045_platform_data, NULL},
#endif

	/* IPC devices */
	{"pmic_charger", SFI_DEV_TYPE_IPC, 1, &no_platform_data, NULL},
	{"pmic_ccsm", SFI_DEV_TYPE_IPC, 1, &mrfl_pmic_ccsm_platform_data,
						&ipc_device_handler},
	{"i2c_pmic_adap", SFI_DEV_TYPE_IPC, 1, &mrfl_pmic_i2c_platform_data,
						&ipc_device_handler},
	{"pmic_ipc", SFI_DEV_TYPE_IPC, 1, &no_platform_data, NULL},

	/* IPC devices */
	{"ctp_rt5671", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},
	{"ctp_rhb_cs42l73", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},
	{"ctp_vb_cs42l73", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},
	{"ctp_ht_wm5102", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},
	{"mrfld_lm49453", SFI_DEV_TYPE_IPC, 1, &merfld_audio_platform_data,
						&ipc_device_handler},
	{"mrfld_wm8958", SFI_DEV_TYPE_IPC, 1, &merfld_wm8958_audio_platform_data,
						&ipc_device_handler},
	{"mrfld_rt5647", SFI_DEV_TYPE_IPC, 1, &merfld_rt5647_audio_platform_data,
						&ipc_device_handler},
	{"soc_thrm", SFI_DEV_TYPE_IPC, 1, &no_platform_data,
						&soc_thrm_device_handler},
	/*{"wm8958", SFI_DEV_TYPE_I2C, 0, &wm8994_platform_data, NULL},*/
	{"rt5647", SFI_DEV_TYPE_I2C, 0, &rt5647_platform_data, NULL},
	{"wm5102", SFI_DEV_TYPE_I2C, 0, &wm5102_platform_data, NULL},
	{"lm49453_codec", SFI_DEV_TYPE_I2C, 1, &no_platform_data, NULL},
	{"cs42l73", SFI_DEV_TYPE_I2C, 1, &cs42l73_platform_data, NULL},
	{"rt5671", SFI_DEV_TYPE_I2C, 1, &no_platform_data, NULL},
#ifdef CONFIG_HSI
	{"hsi_ifx_modem", SFI_DEV_TYPE_HSI, 0, &hsi_modem_platform_data, NULL},
	{"hsi_ffl_modem", SFI_DEV_TYPE_HSI, 0, &ffl_modem_platform_data, NULL},
	{"hsi_edlp_modem", SFI_DEV_TYPE_HSI, 0, &edlp_modem_platform_data,
		&sfi_handle_edlp_dev},
	{"hsi_edlp_fast", SFI_DEV_TYPE_HSI, 0, &edlp_fast_platform_data,
		&sfi_handle_edlp_fast_dev},
#endif
	{"XMM_6260", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_6268", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_6360", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV3", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV3_5", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV4", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7260_REV1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"RMC_CYGNUS", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"CYGNUS_FFRD_EU", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"CYGNUS_FFRD_NA", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"RMC_CYGNUS_PCI", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"RMC_PEGASUS", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	/* scalability V2 configurations */
	/* for 6360 modem*/
	{"XMM6360_CONF_1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM6360_CONF_2", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM6360_CONF_3", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	/* for 7160 modem*/
	{"XMM7160_CONF_1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7160_CONF_2", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7160_CONF_3", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7160_CONF_4", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7160_CONF_5", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7160_CONF_6", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	/* for 7260 modem */
	{"XMM7260_CONF_1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_2", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_3", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_4", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_5", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_6", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_7", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_8", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_9", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_10", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_11", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_12", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
#ifdef CONFIG_USB_DWC3_OTG
	/* USB */
	{"ULPICAL_7F", SFI_DEV_TYPE_USB, 0, &no_platform_data,
		&sfi_handle_usb},
	{"ULPICAL_7D", SFI_DEV_TYPE_USB, 0, &no_platform_data,
		&sfi_handle_usb},
	{"UTMICAL_PEDE3TX0", SFI_DEV_TYPE_USB, 0, &no_platform_data,
		&sfi_handle_usb},
	{"UTMICAL_PEDE6TX7", SFI_DEV_TYPE_USB, 0, &no_platform_data,
		&sfi_handle_usb},
#endif
	{},
};
