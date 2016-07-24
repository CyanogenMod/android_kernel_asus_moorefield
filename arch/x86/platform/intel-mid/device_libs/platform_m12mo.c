/*
 * platform_m10m0.c: m10m0 platform data initilization file
 *
 * (C) Copyright 2014 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <asm/intel_scu_ipcutil.h>
#include <asm/intel-mid.h>
#include <asm/intel_scu_flis.h>
#ifdef CONFIG_INTEL_SOC_PMC
#include <asm/intel_soc_pmc.h>
#endif
#ifdef CONFIG_ACPI
#include <linux/acpi_gpio.h>
#endif
#include <linux/atomisp_platform.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <linux/interrupt.h>
#ifdef CONFIG_CRYSTAL_COVE
#include <linux/mfd/intel_mid_pmic.h>
#endif
#include <linux/regulator/consumer.h>
#include <media/v4l2-subdev.h>
#include <media/m12mo_atomisp.h>
#include <linux/spi/spi.h>
//#include <linux/spi/intel_mid_ssp_spi.h>
#include <linux/spi/pxa2xx_spi.h>
#include "platform_camera.h"
#include "platform_m12mo.h"
#include <asm/intel_scu_pmic.h>
#include <linux/HWVersion.h>
#include <asm/intel-mid.h>

#define OSC_CAM_CLK 0x0
#define CLK_19P2MHz 0x1
/* workaround - use xtal for cht */
#define CLK_19P2MHz_XTAL 0x0
#define GPIO4CTLO_REG 0x82 //This REGISTER is for SKY81298 enable pin.
#ifdef CONFIG_CRYSTAL_COVE
#define VPROG_2P8V 0x5d
#define VPROG_1P2V 0x5a
#define VPROG_ENABLE 0x63
#define VPROG_DISABLE 0x62
#else
#define MSIC_VPROG1_MRFLD_CTRL       (0xac)
#define MSIC_VPROG1_ON_2P8           (0xc1)
#define MSIC_VPROG1_ON_1P8           (0x43)
#define MSIC_VPROG1_OFF              (0x0)
#define MSIC_VPROG2_MRFLD_CTRL       (0xad)
#define MSIC_VPROG2_ON_1P8           (0x41)
#define MSIC_VPROG2_OFF              (0x0)
#endif

#define VCM_3V3     8
#define ISP_1V2     50
#define CAM_1V0     55
#define CAM_2V8     56
#define ISP_RST     57
#ifdef CONFIG_ZS550ML
#define CAM_1V2     65
#else
#define CAM_1V2     66
#endif

static int isp_1v2 = -1;
static int isp_rst = -1;
static int cam_2v8 = -1;
static int front_1v2 = -1;
static int rear_1v0 = -1;
static int vcm_3v3 = -1;

static void setup_m12mo_spi(struct m12mo_atomisp_spi_platform_data *spi_pdata,
			    void *data);
int m12mo_platform_identify_fw(void);

static struct atomisp_camera_caps m12mo_camera_caps;

static int intr_gpio = -1;		/* m12mo interrupt pin */
/*
 * Configure these based on the board. Currently we don't get these
 * from BIOS / IAFW
 */
static int cs_chip_select = -1;

static enum atomisp_input_format input_format = ATOMISP_INPUT_FORMAT_YUV422_8;

/*
 * Ext-ISP m12mo platform data
 */
static int m12mo_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;

	if (isp_rst < 0) {
#ifndef CONFIG_ACPI
		ret = camera_sensor_gpio(ISP_RST, "ISP_RST", GPIOF_DIR_OUT, 0);
		if (ret < 0)
			return ret;
		isp_rst = ret;
#else
		isp_rst = acpi_get_gpio("\\_SB.GPO1", 52);
		printk("M12MO isp_rst is %d\n", isp_rst);
		ret = gpio_request(isp_rst, GP_CAMERA_0_RESET);
		if (ret) {
			pr_err("%s: failed to request reset pin\n", __func__);
			return -EINVAL;
		}

		ret = gpio_direction_output(isp_rst, 1);
		if (ret) {
			pr_err("%s: failed to set direction for reset pin\n", __func__);
			gpio_free(isp_rst);
		}
#endif
	}

	if (flag) {
		/* reset = 0 is requied in case of ESD failure */
		gpio_set_value(isp_rst, 0);

		usleep_range(100, 200);
        printk("M12MO pull up GPIO%d\n", isp_rst);
		gpio_set_value(isp_rst, 1);

		usleep_range(1000, 1500);
	} else {
        printk("M12MO pull down GPIO%d\n", isp_rst);
		gpio_set_value(isp_rst, 0);
	}

	return 0;
}

static int m12mo_gpio_intr_ctrl(struct v4l2_subdev *sd)
{
#ifdef CONFIG_INTEL_SCU_FLIS
	/* This should be done in pin cfg XML and not here */
	config_pin_flis(47, PULL, DOWN_50K);
	config_pin_flis(47, MUX, MUX_EN_INPUT_EN | INPUT_EN);
#endif

	if (intr_gpio >= 0)
		return intr_gpio;

	return -EINVAL;
}

static int m12mo_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
#ifdef CONFIG_INTEL_SOC_PMC
	int ret;
	if (flag) {
		ret = pmc_pc_set_freq(OSC_CAM_CLK, (IS_CHT) ?
			CLK_19P2MHz_XTAL : CLK_19P2MHz);
		if (ret)
			return ret;
		ret = pmc_pc_configure(OSC_CAM_CLK, 1);
	} else {
		ret = pmc_pc_configure(OSC_CAM_CLK, 2);
	}
	pr_info("M12MO clock control. flag=%d ret=%d\n", flag, ret);
	return ret;
#elif defined(CONFIG_INTEL_SCU_IPC_UTIL)
	static const unsigned int clock_khz = 19200;
    pr_info("M12MO clock control. flag=%d\n", flag);
	return intel_scu_ipc_osc_clk(OSC_CLK_DISP, flag ? clock_khz : 0);
#else
	pr_err("M12MO clock is not set.\n");
	return 0;
#endif
}

static int m12mo_power_ctrl(struct v4l2_subdev *sd, int flag)
{
    int ret = 0;
    pr_info("M12MO power control. flag = %d\n", flag);
    switch(Read_PROJ_ID()) {
        case PROJ_ID_ZS550ML:
        case PROJ_ID_ZS570ML:
            pr_info("Project ID is ZS550ML or ZS570ML, M12MO enter power_ctrl\n");
            break;
        default:
            pr_info("Project ID is NOT ZS550ML nor ZS570ML, M12MO exit power_ctrl\n");
            return -1;
            break;
    }
#ifdef CONFIG_CRYSTAL_COVE
    if (flag) {
        ret = intel_mid_pmic_writeb(VPROG_2P8V, VPROG_ENABLE);
        if (ret) {
            pr_err("Failed to power on V2P8SX.\n");
            return ret;
        }
        ret = intel_mid_pmic_writeb(VPROG_1P2V, VPROG_ENABLE);
        if (ret) {
            pr_err("Failed to power on V1P2SX.\n");
            /* Turn all powers off if one is failed. */
            intel_mid_pmic_writeb(VPROG_2P8V, VPROG_DISABLE);
            return ret;
        }
        /* Wait for 8ms to make all the power supplies to be stable. */
        usleep_range(8000, 8000);
    } else {
        /* Turn all powers off even when some are failed. */
        if (intel_mid_pmic_writeb(VPROG_2P8V, VPROG_DISABLE))
            pr_err("Failed to power off V2P8SX.\n");
        if (intel_mid_pmic_writeb(VPROG_1P2V, VPROG_DISABLE))
            pr_err("Failed to power off V1P2SX.\n");
    }
#else

    if (isp_1v2 < 0) {
        lnw_gpio_set_alt(ISP_1V2, LNW_GPIO);
        ret = camera_sensor_gpio(ISP_1V2, "ISP_1V2", GPIOF_DIR_OUT, 0);
        if (ret < 0){
            pr_err("M12MO isp_1v2 is not available.\n");
            return ret;
        }
        isp_1v2 = ret;
    }

    if (cam_2v8 < 0) {
        lnw_gpio_set_alt(CAM_2V8, LNW_GPIO);
        ret = camera_sensor_gpio(CAM_2V8, "CAM_2V8", GPIOF_DIR_OUT, 0);
        if (ret < 0){
            pr_err("M12MO cam_2v8 not available.\n");
            return ret;
        }
        cam_2v8 = ret;
    }

    if (vcm_3v3 < 0) {
        lnw_gpio_set_alt(VCM_3V3, LNW_GPIO);
        ret = camera_sensor_gpio(VCM_3V3, "VCM_3V3", GPIOF_DIR_OUT, 0);
        if (ret < 0){
            pr_err("M12MO vcm_3v3 not available.\n");
            return ret;
        }
        vcm_3v3 = ret;
    }

    if (front_1v2 < 0) {
        lnw_gpio_set_alt(CAM_1V2, LNW_GPIO);
        ret = camera_sensor_gpio(CAM_1V2, "CAM_1V2", GPIOF_DIR_OUT, 0);
        if (ret < 0){
            pr_err("M12MO front_1v2 not available.\n");
            return ret;
        }
        front_1v2 = ret;
    }

    if (rear_1v0 < 0) {
        lnw_gpio_set_alt(CAM_1V0, LNW_GPIO);
        ret = camera_sensor_gpio(CAM_1V0, "CAM_1V0", GPIOF_DIR_OUT, 0);
        if (ret < 0){
            pr_err("M12MO rear_1v0 not available.\n");
            return ret;
        }
        rear_1v0 = ret;
    }

    if (flag) {
        if(isp_1v2 > 0){
            printk("M12MO pull up GPIO%d\n", isp_1v2);
            gpio_set_value(isp_1v2, 1);
        }

#ifdef CONFIG_ZS570ML
        ret = intel_scu_ipc_iowrite8(MSIC_VPROG2_MRFLD_CTRL, MSIC_VPROG2_ON_1P8);
        if (ret) {
            pr_err("Failed to power on M12MO MSIC_VPROG2_ON_1P8.\n");
            return ret;
        }else{
            printk("M12MO pull up Vprog2, 1.8V \n");
        }

        ret = intel_scu_ipc_iowrite8(MSIC_VPROG1_MRFLD_CTRL, MSIC_VPROG1_ON_2P8);
        if (ret) {
            pr_err("Failed to power on M12MO MSIC_VPROG1_ON_2P8.\n");
            return ret;
        }else{
            printk("M12MO pull up Vprog1, 2.8V\n");
        }
#endif

        if(cam_2v8 > 0){
            printk("M12MO pull up GPIO%d\n", cam_2v8);
            gpio_set_value(cam_2v8, 1);
        }

        if(vcm_3v3 > 0){
            printk("M12MO pull up GPIO%d\n", vcm_3v3);
            gpio_set_value(vcm_3v3, 1);
        }

#if 1
		ret = intel_scu_ipc_iowrite8(GPIO4CTLO_REG, 0x31);
		if (ret) 
		{
            pr_err("%s\t --> Failed to PMIC GPIO4CTLO_REG(0x%x) pull high(0x%x) ret = %d\n",__func__ , GPIO4CTLO_REG, 0x31, ret);
            return ret;
		}
        else
        	printk("%s\t --> PMIC GPIO4CTLO_REG(0x%x) pull high(0x%x)\n",__func__ , GPIO4CTLO_REG, 0x31);
#endif

//front camera
        if(front_1v2 > 0){
            printk("M12MO pull up GPIO%d\n", front_1v2);
            gpio_set_value(front_1v2, 1);
        }

//rear camera
        if(rear_1v0 > 0){
            printk("M12MO pull up GPIO%d\n", rear_1v0);
            gpio_set_value(rear_1v0, 1);
        }

        /* Wait for 8ms to make all the power supplies to be stable. */
    } else {


        printk("M12MO pull down GPIO%d\n", rear_1v0);
        gpio_set_value(rear_1v0, 0);
        camera_sensor_gpio_free(rear_1v0);
        rear_1v0 = -1;

        printk("M12MO pull down GPIO%d\n", front_1v2);
        gpio_set_value(front_1v2, 0);
        camera_sensor_gpio_free(front_1v2);
        front_1v2 = -1;

        printk("M12MO pull down GPIO%d\n", vcm_3v3);
        gpio_set_value(vcm_3v3, 0);
        camera_sensor_gpio_free(vcm_3v3);
        vcm_3v3 = -1;

        printk("M12MO pull down GPIO%d\n", cam_2v8);
        gpio_set_value(cam_2v8, 0);
        camera_sensor_gpio_free(cam_2v8);
        cam_2v8 = -1;

#ifdef CONFIG_ZS570ML
        printk("M12MO pull down Vprog1, 2.8V \n");
        ret = intel_scu_ipc_iowrite8(MSIC_VPROG1_MRFLD_CTRL, MSIC_VPROG1_OFF);
        if (ret) {
            pr_err("Failed to power off M12MO MSIC_VPROG1_ON_2P8.\n");
            return ret;
        }

        printk("M12MO pull down Vprog2, 1.8V \n");
        ret = intel_scu_ipc_iowrite8(MSIC_VPROG2_MRFLD_CTRL, MSIC_VPROG2_OFF);
        if (ret) {
            pr_err("Failed to power off M12MO MSIC_VPROG2_ON_1P8.\n");
            return ret;
        }
#endif

#if 1
		ret = intel_scu_ipc_iowrite8(GPIO4CTLO_REG, 0x30);
		if (ret)
		{
            pr_err("%s\t --> Failed to PMIC GPIO4CTLO_REG(0x%x) pull low(0x%x) ret = %d\n",__func__ , GPIO4CTLO_REG, 0x30 , ret);
            return ret;
		}
        else
        	printk("%s\t --> PMIC GPIO4CTLO_REG(0x%x) pull low(0x%x)\n",__func__ , GPIO4CTLO_REG, 0x30);
#endif

        printk("M12MO pull down GPIO%d\n", isp_1v2);
        gpio_set_value(isp_1v2, 0);
        camera_sensor_gpio_free(isp_1v2);
        isp_1v2 = -1;

        camera_sensor_gpio_free(isp_rst);
        isp_rst = -1;
    }

#endif
    return ret;
}

static int m12mo_csi_configure(struct v4l2_subdev *sd, int flag)
{
	static const int LANES = 4;
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY, LANES,
		input_format, 0, flag);
}

static struct m12mo_fw_id fw_ids[] = {
	{ "L13F0PAHK01", M12MO_FW_TYPE_0 },
	{ NULL, 0},
};

static u32 clock_rate[] = {
    26000000,
	24000000,
	19200000,
};

static u32 mipi_packet_size[] = {
	2048,
	4096,
};

static void spi_hw_resources_setup(struct m12mo_atomisp_spi_platform_data *pdata)
{
	/* Setup SPI interface */
	if (gpio_request(pdata->spi_cs_gpio, "m12mo_spi_cs")) {
		pr_err("Can't allocate gpio for m12mo spi chip select.\n");
		pr_err("Disabling FW update over the SPI\n");
		pdata->spi_enabled = false;
		return;
	}

	cs_chip_select = pdata->spi_cs_gpio;
	lnw_gpio_set_alt(cs_chip_select, LNW_GPIO);
	gpio_direction_output(cs_chip_select, 0);
	lnw_gpio_set_alt(116, LNW_ALT_1);
	lnw_gpio_set_alt(118, LNW_ALT_1);
	lnw_gpio_set_alt(119, LNW_ALT_1);


#ifdef CONFIG_INTEL_SCU_FLIS
	/* Setup flis configuration if requested to do so */
	if (pdata->spi_clock_flis != -1)
		config_pin_flis(pdata->spi_clock_flis, MUX,
				MUX_EN_OUTPUT_EN | OUTPUT_EN);

	if (pdata->spi_dataout_flis != -1)
		config_pin_flis(pdata->spi_dataout_flis, MUX,
				MUX_EN_OUTPUT_EN | OUTPUT_EN);

	if (pdata->spi_datain_flis != -1)
		config_pin_flis(pdata->spi_datain_flis,
				MUX, MUX_EN_INPUT_EN | INPUT_EN);

	if (pdata->spi_cs_flis != -1)
		config_pin_flis(pdata->spi_cs_flis,
				MUX, MUX_EN_OUTPUT_EN | OUTPUT_EN);
#endif
}

static void spi_cs_control(u32 command)
{
	if (cs_chip_select == -1)
		return;

	/* CS must be set high during transmission */
	if (command == PXA2XX_CS_ASSERT) {
		gpio_set_value(cs_chip_select, 1);
		udelay(10);
	}
	else
		gpio_set_value(cs_chip_select, 0);
};

/*static struct intel_mid_ssp_spi_chip spi_chip = {
	.burst_size	= DFLT_FIFO_BURST_SIZE,
	.timeout	= DFLT_TIMEOUT_VAL,
	.dma_enabled	= false,
	.cs_control	= spi_cs_control,
};*/

static struct pxa2xx_spi_chip spi_chip = {
        .tx_threshold = 8,
        .rx_threshold = 8,
        .dma_burst_size = 8,
        .timeout = 235, /* See Intel documentation */
        .cs_control = spi_cs_control,
        /*
        * gpio_cs is a undocumented struct member that needs to be
        * assigned. (Negative number == don't use GPIO/use SSP)
        */
        .gpio_cs = -1,
};

static int m12mo_platform_init(void)
{
	static const char gpio_name[] = "xenon_ready";
	int ret;

	if (intr_gpio == -1) {
#ifndef CONFIG_ACPI
#if 0
		intr_gpio = get_gpio_by_name(gpio_name);
#else
		intr_gpio = 47;
#endif
		if (intr_gpio == -1) {
			pr_err("Failed to get interrupt gpio\n");
			return -EINVAL;
		}
#else
		intr_gpio = acpi_get_gpio("\\_SB.GPO1", 54);
#endif
		pr_info("camera interrupt gpio: %d\n", intr_gpio);
	}

	ret = gpio_request(intr_gpio, gpio_name);
	if (ret) {
		pr_err("Failed to request interrupt gpio(pin %d)\n", intr_gpio);
		return -EINVAL;
	}

	ret = gpio_direction_input(intr_gpio);
	if (ret) {
		pr_err("failed to set interrupt gpio %d direction\n",
				intr_gpio);
		gpio_free(intr_gpio);
		intr_gpio = -1;
	}

	return ret;
}

static int m12mo_platform_deinit(void)
{
	if (intr_gpio >= 0)
		gpio_free(intr_gpio);
	if (cs_chip_select >= 0)
		gpio_free(cs_chip_select);
	intr_gpio = -1;
	cs_chip_select = -1;

	camera_sensor_gpio_free(isp_rst);
	isp_rst = -1;

	return 0;
}

static struct atomisp_camera_caps *m12mo_get_camera_caps(void)
{
	m12mo_camera_caps.sensor_num = 2;
	m12mo_camera_caps.sensor[0].stream_num = 2;
    m12mo_camera_caps.sensor[1].stream_num = 2;
	return &m12mo_camera_caps;
}

static struct m12mo_platform_data m12mo_sensor_platform_data = {
	/* Common part for all sensors used with atom isp */
	.common.gpio_ctrl	= m12mo_gpio_ctrl,
	.common.gpio_intr_ctrl	= m12mo_gpio_intr_ctrl,
	.common.flisclk_ctrl	= m12mo_flisclk_ctrl,
	.common.power_ctrl	= m12mo_power_ctrl,
	.common.csi_cfg		= m12mo_csi_configure,
	.common.platform_init	= m12mo_platform_init,
	.common.platform_deinit = m12mo_platform_deinit,
	.common.get_camera_caps = m12mo_get_camera_caps,

	/* platform data for spi flashing */
	.spi_pdata.spi_enabled	= true, /* By default SPI is not available */
	.spi_pdata.spi_bus_num	= 6, /* Board specific */
	.spi_pdata.spi_cs_gpio	= 117, /* Board specific */
	.spi_pdata.spi_speed_hz = 10000000, /* Board specific */
	/* Set flis values to -1 if the data is correct in pin cfg xml */
	.spi_pdata.spi_clock_flis	= ann_gp_ssp_6_clk, /* Board specific */
	.spi_pdata.spi_dataout_flis	= ann_gp_ssp_6_txd, /* Board specific */
	.spi_pdata.spi_datain_flis	= ann_gp_ssp_6_rxd, /* Board specific */
	.spi_pdata.spi_cs_flis		= ann_gp_ssp_6_fs, /* Board specific */

	.def_fw_type    = M12MO_FW_TYPE_0,
	.ref_clock_rate = clock_rate, /* Board specific */
	.mipi_packet_size = mipi_packet_size,
	.fw_ids		= fw_ids,
	.spi_setup	= setup_m12mo_spi,
	.identify_fw	= m12mo_platform_identify_fw,
};

int m12mo_platform_identify_fw(void)
{
#ifdef CONFIG_ACPI
	static int fw_type = -1;
	struct device *dev;
	struct i2c_client *client;
    printk("@%s in define CONFIG_ACPI\n", __func__);

	if (fw_type != -1)
		return fw_type;

	dev = bus_find_device_by_name(&i2c_bus_type, NULL, "4-001f");
	if (dev) {
		client = to_i2c_client(dev);
		if (strncmp(client->name, "SOM33FB:00", 10) == 0) {
			m12mo_sensor_platform_data.spi_setup = NULL;
			input_format = ATOMISP_INPUT_FORMAT_YUV420_8_LEGACY;
			m12mo_camera_caps.multi_stream_ctrl = true;
			fw_type = M12MO_FW_TYPE_2;
		}
	}
	return fw_type;
#else
    printk("@%s not in define CONFIG_ACPI\n", __func__);
	return -1;
#endif
}

static struct spi_board_info m12mo_spi_info[] = {
	{
		.modalias		= "m12mo_spi",
		.mode			= SPI_MODE_0,
		.chip_select		= 0,
		.controller_data	= &spi_chip,
		.platform_data		= &m12mo_sensor_platform_data.spi_pdata,
	}
};

static void setup_m12mo_spi(struct m12mo_atomisp_spi_platform_data *spi_pdata,
	void *data)
{
	m12mo_spi_info[0].bus_num	= spi_pdata->spi_bus_num;
	m12mo_spi_info[0].max_speed_hz	= spi_pdata->spi_speed_hz;
	spi_pdata->device_data		= data;

	spi_hw_resources_setup(spi_pdata);

	/* register SPI interface for the FW update */
	spi_register_board_info(m12mo_spi_info,
				ARRAY_SIZE(m12mo_spi_info));
}

void *m12mo_platform_data(void *info)
{
	/*
	 * Atom isp driver assumes to get pointer to struct
	 * "camera_sensor_platform_data". Nasty assumption but must honored
	 */
	return &m12mo_sensor_platform_data.common;
}

#if 0 //def CONFIG_VIDEO_M12MO_FAKE_SFI_TABLE
static struct sfi_device_table_entry m12mo_entry = {
	.type = SFI_DEV_TYPE_I2C,
	.host_num = 4,
	.addr = 0x1F,
	.irq = 0x0,
	.max_freq = 0x0,
	.name = "m12mo",
};

static int __init platform_m12mo_module_init(void)
{
	struct devs_id *dev;
	dev = get_device_id(m12mo_entry.type, m12mo_entry.name);
	if (dev && dev->device_handler)
		dev->device_handler(&m12mo_entry, dev);
	return 0;
}

module_init(platform_m12mo_module_init);
#endif /* CONFIG_M12MO_FAKE_SFI_TABLE */
