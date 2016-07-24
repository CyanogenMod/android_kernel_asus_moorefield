/*
 * Copyright © 2010 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Authors:
 * Xiujun Geng <xiujun.geng@intel.com>
 */

#include "mdfld_dsi_dbi.h"
#include "mdfld_dsi_pkg_sender.h"
#include "mdfld_dsi_esd.h"
#include <linux/version.h>
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0))
#include <asm/intel_scu_pmic.h>
#else
#include <asm/intel_scu_ipc.h>
#endif
#include <linux/gpio.h>

/* HACK to create I2C device while it's not created by platform code */
#define CMI_LCD_I2C_ADAPTER	0
#define CMI_LCD_I2C_ADDR	0x2C

/* Brightness related */
#define MINIMUM_BRIGHTNESS_LEVEL_20KHZ	15
/* MSIC PWM duty cycle goes up to 0x63 = 99% */
#define BACKLIGHT_DUTY_FACTOR	0x63
#define PWM0DUTYCYCLE		0x67

/* Backlight IC(LP8556) Registers */
#define BRIGHTNESS_CONTROL	0x00
#define DEVICE_CONTROL		0x01
#define DEVICE_STATUS		0x02
#define DEVICE_CONTROL_LED	0x04
#define DEVICE_LED_ENABLE	0x05

/* Backlight IC(LP8556) Register 0x01h */
#define BRT_MODE_PWM_INPUT_ONLY	0x00
#define BRT_MODE_PWM_AND_REG_CB	0x02
#define BRT_MODE_PWM_REG_ONLY	0x04
#define BRT_MODE_PWM_AND_REG_CA	0x06

#define BL_CTL_DISABLE		0x00
#define BL_CTL_ENABLE		0x01

/* Panel CABC registers */
#define PANEL_PWM_MAX		0x99

/*GPIO Pins */
#define GPIO_MIPI_BRIDGE_RESET	115

#define GPIO_MIPI_LCD_BL_EN	112
#define GPIO_MIPI_LCD_VADD	110

/* MSIC PWM duty cycle goes up to 0x63 = 99% */
#define BACKLIGHT_DUTY_FACTOR	0x63
#define PWM0DUTYCYCLE		0x67

/* panel info */
#define TC35876X_PANEL_WIDTH	216
#define TC35876X_PANEL_HEIGHT	135

/* Panel CABC registers */
#define PANEL_PWM_CONTROL	0x90
#define PANEL_FREQ_DIVIDER_HI	0x91
#define PANEL_FREQ_DIVIDER_LO	0x92
#define PANEL_DUTY_CONTROL	0x93
#define PANEL_MODIFY_RGB	0x94
#define PANEL_FRAMERATE_CONTROL	0x96
#define PANEL_PWM_MIN		0x97
#define PANEL_PWM_REF		0x98
#define PANEL_PWM_MAX		0x99
#define PANEL_ALLOW_DISTORT	0x9A
#define PANEL_BYPASS_PWMI	0x9B

/* Panel color management registers */
#define PANEL_CM_ENABLE		0x700
#define PANEL_CM_HUE		0x701
#define PANEL_CM_SATURATION	0x702
#define PANEL_CM_INTENSITY	0x703
#define PANEL_CM_BRIGHTNESS	0x704
#define PANEL_CM_CE_ENABLE	0x705
#define PANEL_CM_PEAK_EN	0x710
#define PANEL_CM_GAIN		0x711
#define PANEL_CM_HUETABLE_START	0x730
#define PANEL_CM_HUETABLE_END	0x747 /* inclusive */
#define I2C_PCI_DEVICE_ID	0x082E

/* bridge registers */
/* DSI D-PHY Layer Registers */
#define D0W_DPHYCONTTX		0x0004
#define CLW_DPHYCONTRX		0x0020
#define D0W_DPHYCONTRX		0x0024
#define D1W_DPHYCONTRX		0x0028
#define D2W_DPHYCONTRX		0x002C
#define D3W_DPHYCONTRX		0x0030
#define COM_DPHYCONTRX		0x0038
#define CLW_CNTRL		0x0040
#define D0W_CNTRL		0x0044
#define D1W_CNTRL		0x0048
#define D2W_CNTRL		0x004C
#define D3W_CNTRL		0x0050
#define DFTMODE_CNTRL		0x0054

/* DSI PPI Layer Registers */
#define PPI_STARTPPI		0x0104
#define PPI_BUSYPPI		0x0108
#define PPI_LINEINITCNT		0x0110
#define PPI_LPTXTIMECNT		0x0114
#define PPI_LANEENABLE		0x0134
#define PPI_TX_RX_TA		0x013C
#define PPI_CLS_ATMR		0x0140
#define PPI_D0S_ATMR		0x0144
#define PPI_D1S_ATMR		0x0148
#define PPI_D2S_ATMR		0x014C
#define PPI_D3S_ATMR		0x0150
#define PPI_D0S_CLRSIPOCOUNT	0x0164
#define PPI_D1S_CLRSIPOCOUNT	0x0168
#define PPI_D2S_CLRSIPOCOUNT	0x016C
#define PPI_D3S_CLRSIPOCOUNT	0x0170
#define CLS_PRE			0x0180
#define D0S_PRE			0x0184
#define D1S_PRE			0x0188
#define D2S_PRE			0x018C
#define D3S_PRE			0x0190
#define CLS_PREP		0x01A0
#define D0S_PREP		0x01A4
#define D1S_PREP		0x01A8
#define D2S_PREP		0x01AC
#define D3S_PREP		0x01B0
#define CLS_ZERO		0x01C0
#define D0S_ZERO		0x01C4
#define D1S_ZERO		0x01C8
#define D2S_ZERO		0x01CC
#define D3S_ZERO		0x01D0
#define PPI_CLRFLG		0x01E0
#define PPI_CLRSIPO		0x01E4
#define HSTIMEOUT		0x01F0
#define HSTIMEOUTENABLE		0x01F4

/* DSI Protocol Layer Registers */
#define DSI_STARTDSI		0x0204
#define DSI_BUSYDSI		0x0208
#define DSI_LANEENABLE		0x0210
#define DSI_LANESTATUS0		0x0214
#define DSI_LANESTATUS1		0x0218
#define DSI_INTSTATUS		0x0220
#define DSI_INTMASK		0x0224
#define DSI_INTCLR		0x0228
#define DSI_LPTXTO		0x0230

/* DSI General Registers */
#define DSIERRCNT		0x0300

/* DSI Application Layer Registers */
#define APLCTRL			0x0400
#define RDPKTLN			0x0404

/* Video Path Registers */
#define VPCTRL			0x0450
#define HTIM1			0x0454
#define HTIM2			0x0458
#define VTIM1			0x045C
#define VTIM2			0x0460
#define VFUEN			0x0464

/* LVDS Registers */
#define LVMX0003		0x0480
#define LVMX0407		0x0484
#define LVMX0811		0x0488
#define LVMX1215		0x048C
#define LVMX1619		0x0490
#define LVMX2023		0x0494
#define LVMX2427		0x0498
#define LVCFG			0x049C
#define LVPHY0			0x04A0
#define LVPHY1			0x04A4

/* System Registers */
#define SYSSTAT			0x0500
#define SYSRST			0x0504

/* GPIO Registers */
#define GPIOC			0x0520
#define GPIOO			0x0524
#define GPIOI			0x0528

/* I2C Registers */
#define I2CTIMCTRL		0x0540
#define I2CMADDR		0x0544
#define WDATAQ			0x0548
#define RDATAQ			0x054C

/* Chip/Rev Registers */
#define IDREG			0x0580

/* Input muxing for registers LVMX0003...LVMX2427 */
enum {
	INPUT_R0,	/* 0 */
	INPUT_R1,
	INPUT_R2,
	INPUT_R3,
	INPUT_R4,
	INPUT_R5,
	INPUT_R6,
	INPUT_R7,
	INPUT_G0,	/* 8 */
	INPUT_G1,
	INPUT_G2,
	INPUT_G3,
	INPUT_G4,
	INPUT_G5,
	INPUT_G6,
	INPUT_G7,
	INPUT_B0,	/* 16 */
	INPUT_B1,
	INPUT_B2,
	INPUT_B3,
	INPUT_B4,
	INPUT_B5,
	INPUT_B6,
	INPUT_B7,
	INPUT_HSYNC,	/* 24 */
	INPUT_VSYNC,
	INPUT_DE,
	LOGIC_0,
	/* 28...31 undefined */
};

#define FLD_MASK(start, end)    (((1 << ((start) - (end) + 1)) - 1) << (end))
#define FLD_VAL(val, start, end) (((val) << (end)) & FLD_MASK(start, end))
#define INPUT_MUX(lvmx03, lvmx02, lvmx01, lvmx00)		\
	(FLD_VAL(lvmx03, 29, 24) | FLD_VAL(lvmx02, 20, 16) |	\
	FLD_VAL(lvmx01, 12, 8) | FLD_VAL(lvmx00, 4, 0))

/* local variables */
static struct i2c_client *cmi_lcd_i2c_client;
static struct i2c_client *tc35876x_client;

struct cmi_lcd_data {
	bool enabled;
	struct mutex lock; /* for enabled */
};

static
int tc35876x_bridge_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	PSB_DEBUG_ENTRY("\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		DRM_ERROR("i2c_check_functionality() failed\n");
		return -ENODEV;
	}

	gpio_request(GPIO_MIPI_BRIDGE_RESET, "tc35876x_bridge_reset");

	gpio_request(GPIO_MIPI_LCD_BL_EN, "tc35876x_panel_bl_en");

	gpio_request(GPIO_MIPI_LCD_VADD, "display_dvdd");

	tc35876x_client = client;

	return 0;
}

static int tc35876x_bridge_remove(struct i2c_client *client)
{
	PSB_DEBUG_ENTRY("\n");

	gpio_free(GPIO_MIPI_BRIDGE_RESET);
	gpio_free(GPIO_MIPI_LCD_BL_EN);

	tc35876x_client = NULL;

	return 0;
}

static
const struct i2c_device_id tc35876x_bridge_id[] = {
	{ "i2c_disp_brig", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tc35876x_bridge_id);

static
struct i2c_driver tc35876x_bridge_i2c_driver = {
	.driver = {
		.name = "i2c_disp_brig",
	},
	.id_table = tc35876x_bridge_id,
	.probe = tc35876x_bridge_probe,
	.remove = tc35876x_bridge_remove,
};

static
int cmi_lcd_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct cmi_lcd_data *lcd_data;

	PSB_DEBUG_ENTRY("\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		DRM_ERROR("i2c_check_functionality() failed\n");
		return -ENODEV;
	}

	lcd_data = devm_kzalloc(&client->dev, sizeof(*lcd_data), GFP_KERNEL);
	if (!lcd_data) {
		DRM_ERROR("out of memory\n");
		return -ENOMEM;
	}

	mutex_init(&lcd_data->lock);

	i2c_set_clientdata(client, lcd_data);

	cmi_lcd_i2c_client = client;

	return 0;
}

static const struct i2c_device_id cmi_lcd_i2c_id[] = {
	{ "cmi-lcd", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cmi_lcd_i2c_id);

static int cmi_lcd_i2c_remove(struct i2c_client *client)
{
	dev_dbg(&client->dev, "%s\n", __func__);

	cmi_lcd_i2c_client = NULL;

	return 0;
}

static struct i2c_driver cmi_lcd_i2c_driver = {
	.driver = {
		.name = "cmi-lcd",
	},
	.id_table = cmi_lcd_i2c_id,
	.probe = cmi_lcd_i2c_probe,
	.remove = cmi_lcd_i2c_remove,
};

static
int cmi_lcd_hack_create_device(void)
{
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	struct i2c_board_info info = {
		.type = "cmi-lcd",
		.addr = CMI_LCD_I2C_ADDR,
	};

	PSB_DEBUG_ENTRY("\n");

	adapter = i2c_get_adapter(CMI_LCD_I2C_ADAPTER);
	if (!adapter) {
		DRM_ERROR("i2c_get_adapter(%d) failed\n", CMI_LCD_I2C_ADAPTER);
		return -EINVAL;
	}

	client = i2c_new_device(adapter, &info);
	if (!client) {
		DRM_ERROR("i2c_new_device() failed\n");
		i2c_put_adapter(adapter);
		return -EINVAL;
	}

	return 0;
}

static
int tc35876x_regw(struct i2c_client *client, u16 reg, u32 value)
{
	int r;
	u8 tx_data[] = {
		/* NOTE: Register address big-endian, data little-endian. */
		(reg >> 8) & 0xff,
		reg & 0xff,
		value & 0xff,
		(value >> 8) & 0xff,
		(value >> 16) & 0xff,
		(value >> 24) & 0xff,
	};
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = tx_data,
			.len = ARRAY_SIZE(tx_data),
		},
	};

	r = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (r < 0) {
		dev_err(&client->dev, "%s: reg 0x%04x val 0x%08x error %d\n",
			__func__, reg, value, r);
		return r;
	}

	if (r < ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s: reg 0x%04x val 0x%08x msgs %d\n",
			__func__, reg, value, r);
		return -EAGAIN;
	}

	return 0;
}

static
int tc35876x_regr(struct i2c_client *client, u16 reg, u32 *value)
{
	int r;
	u8 tx_data[] = {
		(reg >> 8) & 0xff,
		reg & 0xff,
	};
	u8 rx_data[4];
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = tx_data,
			.len = ARRAY_SIZE(tx_data),
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = rx_data,
			.len = ARRAY_SIZE(rx_data),
		 },
	};

	r = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (r < 0) {
		dev_err(&client->dev, "%s: reg 0x%04x error %d\n", __func__,
			reg, r);
		return r;
	}

	if (r < ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s: reg 0x%04x msgs %d\n", __func__,
			reg, r);
		return -EAGAIN;
	}

	*value = rx_data[0] << 24 | rx_data[1] << 16 |
		rx_data[2] << 8 | rx_data[3];

	dev_dbg(&client->dev, "%s: reg 0x%04x value 0x%08x\n", __func__,
		reg, *value);

	return 0;
}

static
void tc35876x_set_bridge_reset_state(int state)
{
	PSB_DEBUG_ENTRY("\n");

	if (state) {
		if (gpio_direction_output(GPIO_MIPI_BRIDGE_RESET, 0))
			gpio_set_value_cansleep(GPIO_MIPI_BRIDGE_RESET, 0);
		/* FIXME:
		 * per spec, the min period of reset signal is 50 nano secs,
		 * but no detailed description. Here wait 0.5~1ms for safe.
		 */
		usleep_range(500, 1000);
	} else {

		if (gpio_direction_output(GPIO_MIPI_BRIDGE_RESET, 0))
			/*Pull MIPI Bridge reset pin to Low */
			gpio_set_value_cansleep(GPIO_MIPI_BRIDGE_RESET, 0);
		usleep_range(500, 1000);

		if (gpio_direction_output(GPIO_MIPI_BRIDGE_RESET, 1))
			/*Pull MIPI Bridge reset pin to High */
			gpio_set_value_cansleep(GPIO_MIPI_BRIDGE_RESET, 1);
		usleep_range(500, 1000);
	}
}

static
void tc35876x_toshiba_bridge_panel_on(void)
{
	int ret;

	PSB_DEBUG_ENTRY("\n");

	gpio_direction_output(GPIO_MIPI_LCD_VADD, 1);
	msleep(260);

	if (gpio_direction_output(GPIO_MIPI_LCD_BL_EN, 1))
		gpio_set_value_cansleep(GPIO_MIPI_LCD_BL_EN, 1);

	if (cmi_lcd_i2c_client) {
		/* Set backlight as register control only */
		ret = i2c_smbus_write_byte_data(cmi_lcd_i2c_client,
						DEVICE_CONTROL,
						BRT_MODE_PWM_REG_ONLY |
						BL_CTL_ENABLE);
		if (ret) {
			DRM_ERROR("i2c write brightness control reg failed\n");
			return;
		}
	} else {
		DRM_ERROR("cmi_lcd_i2c_client is NULL!\n");
	}
}

void tc35876x_configure_lvds_bridge(void)
{
	struct i2c_client *i2c = tc35876x_client;
	u32 id = 0;
	u32 ppi_lptxtimecnt;
	u32 txtagocnt;
	u32 txtasurecnt;

	PSB_DEBUG_ENTRY("\n");

	tc35876x_regr(i2c, IDREG, &id);
	DRM_INFO("tc35876x ID 0x%08x\n", id);

	ppi_lptxtimecnt = 4;
	txtagocnt = (5 * ppi_lptxtimecnt - 3) / 4;
	txtasurecnt = 3 * ppi_lptxtimecnt / 2;

	tc35876x_regw(i2c, PPI_TX_RX_TA, FLD_VAL(txtagocnt, 26, 16) |
		FLD_VAL(txtasurecnt, 10, 0));
	tc35876x_regw(i2c, PPI_LPTXTIMECNT, FLD_VAL(ppi_lptxtimecnt, 10, 0));

	tc35876x_regw(i2c, PPI_D0S_CLRSIPOCOUNT, FLD_VAL(1, 5, 0));
	tc35876x_regw(i2c, PPI_D1S_CLRSIPOCOUNT, FLD_VAL(1, 5, 0));
	tc35876x_regw(i2c, PPI_D2S_CLRSIPOCOUNT, FLD_VAL(1, 5, 0));
	tc35876x_regw(i2c, PPI_D3S_CLRSIPOCOUNT, FLD_VAL(1, 5, 0));

	/* Enabling MIPI & PPI lanes, Enable 4 lanes */
	tc35876x_regw(i2c, PPI_LANEENABLE,
		BIT4 | BIT3 | BIT2 | BIT1 | BIT0);
	tc35876x_regw(i2c, DSI_LANEENABLE,
		BIT4 | BIT3 | BIT2 | BIT1 | BIT0);
	tc35876x_regw(i2c, PPI_STARTPPI, BIT0);
	tc35876x_regw(i2c, DSI_STARTDSI, BIT0);

	/* Setting LVDS output frequency */
	tc35876x_regw(i2c, LVPHY0, FLD_VAL(1, 20, 16) |
		FLD_VAL(2, 15, 14) | FLD_VAL(6, 4, 0)); /* 0x00048006 */

	/* Setting video panel control register,0x00000120 VTGen=ON ?!?!? */
	tc35876x_regw(i2c, VPCTRL, BIT(8) | BIT(5));

	/* Horizontal back porch and horizontal pulse width. 0x00280028 */
	tc35876x_regw(i2c, HTIM1, FLD_VAL(40, 24, 16) | FLD_VAL(40, 8, 0));

	/* Horizontal front porch and horizontal active video size. 0x00500500*/
	tc35876x_regw(i2c, HTIM2, FLD_VAL(80, 24, 16) | FLD_VAL(1280, 10, 0));

	/* Vertical back porch and vertical sync pulse width. 0x000e000a */
	tc35876x_regw(i2c, VTIM1, FLD_VAL(14, 23, 16) | FLD_VAL(10, 7, 0));

	/* Vertical front porch and vertical display size. 0x000e0320 */
	tc35876x_regw(i2c, VTIM2, FLD_VAL(14, 23, 16) | FLD_VAL(800, 10, 0));

	/* Set above HTIM1, HTIM2, VTIM1, and VTIM2 at next VSYNC. */
	tc35876x_regw(i2c, VFUEN, BIT0);

	/* Soft reset LCD controller. */
	tc35876x_regw(i2c, SYSRST, BIT2);

	/* LVDS-TX input muxing */
	tc35876x_regw(i2c, LVMX0003,
		INPUT_MUX(INPUT_R5, INPUT_R4, INPUT_R3, INPUT_R2));
	tc35876x_regw(i2c, LVMX0407,
		INPUT_MUX(INPUT_G2, INPUT_R7, INPUT_R1, INPUT_R6));
	tc35876x_regw(i2c, LVMX0811,
		INPUT_MUX(INPUT_G1, INPUT_G0, INPUT_G4, INPUT_G3));
	tc35876x_regw(i2c, LVMX1215,
		INPUT_MUX(INPUT_B2, INPUT_G7, INPUT_G6, INPUT_G5));
	tc35876x_regw(i2c, LVMX1619,
		INPUT_MUX(INPUT_B4, INPUT_B3, INPUT_B1, INPUT_B0));
	tc35876x_regw(i2c, LVMX2023,
		INPUT_MUX(LOGIC_0,  INPUT_B7, INPUT_B6, INPUT_B5));
	tc35876x_regw(i2c, LVMX2427,
		INPUT_MUX(INPUT_R0, INPUT_DE, INPUT_VSYNC, INPUT_HSYNC));

	/* Enable LVDS transmitter. */
	tc35876x_regw(i2c, LVCFG, BIT0);

	/* Clear notifications. Don't write reserved bits. Was write 0xffffffff
	 * to 0x0288, must be in error?! */
	tc35876x_regw(i2c, DSI_INTCLR, FLD_MASK(31, 30) | FLD_MASK(22, 0));

}

static
int tc35876x_vid_power_on(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err = 0;

	PSB_DEBUG_ENTRY("\n");

	tc35876x_configure_lvds_bridge();

	/*send TURN_ON packet*/
	err = mdfld_dsi_send_dpi_spk_pkg_hs(sender,
			MDFLD_DSI_DPI_SPK_TURN_ON);
	if (err) {
		DRM_ERROR("Faild to send turn on packet\n");
		return err;
	}

	return 0;
}

static
void tc35876x_toshiba_bridge_panel_off(void)
{
	PSB_DEBUG_ENTRY("\n");

	if (gpio_direction_output(GPIO_MIPI_LCD_BL_EN, 0))
		gpio_set_value_cansleep(GPIO_MIPI_LCD_BL_EN, 0);
	usleep_range(1000, 2000);

	gpio_direction_output(GPIO_MIPI_LCD_VADD, 0);
}

static
void tc35876x_suspend_lvds_bridge(void)
{
	PSB_DEBUG_ENTRY("\n");

	if (gpio_direction_output(GPIO_MIPI_BRIDGE_RESET, 0))
		gpio_set_value_cansleep(GPIO_MIPI_BRIDGE_RESET, 0);
	usleep_range(500, 1000);
}

static
int tc35876x_vid_power_off(struct mdfld_dsi_config *dsi_config)
{
	PSB_DEBUG_ENTRY("\n");

	tc35876x_toshiba_bridge_panel_off();
	tc35876x_suspend_lvds_bridge();

	return 0;
}

static
int tc35876x_vid_detect(struct mdfld_dsi_config *dsi_config)
{
	int status;
	struct drm_device *dev = dsi_config->dev;
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	int pipe = dsi_config->pipe;
	u32 dpll_val, device_ready_val;

	PSB_DEBUG_ENTRY("\n");

	if (pipe == 0) {
		/*
		 * FIXME: WA to detect the panel connection status, and need to
		 * implement detection feature with get_power_mode DSI command.
		 */
		if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
					OSPM_UHB_FORCE_POWER_ON)) {
			DRM_ERROR("hw begin failed\n");
			return -EAGAIN;
		}

		dpll_val = REG_READ(regs->dpll_reg);
		device_ready_val = REG_READ(regs->device_ready_reg);
		if ((device_ready_val & DSI_DEVICE_READY) &&
		    (dpll_val & DPLL_VCO_ENABLE)) {
			dsi_config->dsi_hw_context.panel_on = true;
			psb_enable_vblank(dev, pipe);
		} else {
			dsi_config->dsi_hw_context.panel_on = false;
			DRM_INFO("%s: panel is not detected!\n", __func__);
		}

		status = MDFLD_DSI_PANEL_CONNECTED;

		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	} else {
		DRM_INFO("%s: do NOT support dual panel\n", __func__);
		status = MDFLD_DSI_PANEL_DISCONNECTED;
	}

	return status;
}

static
int mdfld_dsi_tc35876x_panel_reset(struct mdfld_dsi_config *dsi_config)
{
	tc35876x_set_bridge_reset_state(0);
	tc35876x_toshiba_bridge_panel_on();

	return 0;
}

static
struct drm_display_mode *tc35876x_vid_get_config_mode(void)
{
	struct drm_display_mode *mode;

	PSB_DEBUG_ENTRY("\n");

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	mode->hdisplay = 1280;
	mode->vdisplay = 800;
	mode->hsync_start = mode->hdisplay + 80;
	mode->hsync_end = mode->hsync_start + 40;
	mode->htotal = mode->hsync_end + 40;

	mode->vsync_start = mode->vdisplay + 14;
	mode->vsync_end = mode->vsync_start + 10;
	mode->vtotal = mode->vsync_end + 14;
	mode->vrefresh = 60;
	mode->clock = mode->vrefresh * mode->htotal * mode->vtotal / 1000;

	PSB_DEBUG_ENTRY("hdisplay is %d\n", mode->hdisplay);
	PSB_DEBUG_ENTRY("vdisplay is %d\n", mode->vdisplay);
	PSB_DEBUG_ENTRY("HSS is %d\n", mode->hsync_start);
	PSB_DEBUG_ENTRY("HSE is %d\n", mode->hsync_end);
	PSB_DEBUG_ENTRY("htotal is %d\n", mode->htotal);
	PSB_DEBUG_ENTRY("VSS is %d\n", mode->vsync_start);
	PSB_DEBUG_ENTRY("VSE is %d\n", mode->vsync_end);
	PSB_DEBUG_ENTRY("vtotal is %d\n", mode->vtotal);
	PSB_DEBUG_ENTRY("clock is %d\n", mode->clock);

	mode->type |= DRM_MODE_TYPE_PREFERRED;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	return mode;
}

static
void tc35876x_vid_get_panel_info(int pipe, struct panel_info *pi)
{
	PSB_DEBUG_ENTRY("\n");

	if (pipe == 0) {
		pi->width_mm = TC35876X_PANEL_WIDTH;
		pi->height_mm = TC35876X_PANEL_HEIGHT;
	}
}

static
int tc35876x_vid_set_brightness(struct mdfld_dsi_config *dsi_config, int level)
{
	int panel_duty_val = 0;
	int ret = 0;

	/* When using 20KHz CABC pwm output, the backlight will be unstable if
	 * the brightness level less than 15%, so set the minimum brightness
	 * level to 15% as workround.
	 */
	if (level < MINIMUM_BRIGHTNESS_LEVEL_20KHZ && 0 != level)
		level = MINIMUM_BRIGHTNESS_LEVEL_20KHZ;
	/* I won't pretend to understand this formula. The panel spec is quite
	 * bad engrish.
	 */
	panel_duty_val = (230 * (level - 100) / 100) + 255;

	if (cmi_lcd_i2c_client) {
		PSB_DEBUG_ENTRY("panel_duty_val = %d\n", panel_duty_val);
		ret = i2c_smbus_write_byte_data(cmi_lcd_i2c_client,
						BRIGHTNESS_CONTROL,
						panel_duty_val);

		if (ret < 0) {
			DRM_ERROR("i2c write brightness failed\n");
			return -EINVAL;
		}
	}

	return 0;
}

static
void tc35876x_vid_dsi_controller_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_context *hw_ctx = &dsi_config->dsi_hw_context;

	PSB_DEBUG_ENTRY("\n");

	dsi_config->lane_count = 4;
	dsi_config->bpp = 24;
	dsi_config->lane_config = MDFLD_DSI_DATA_LANE_4_0;
	dsi_config->video_mode = MDFLD_DSI_VIDEO_NON_BURST_MODE_SYNC_EVENTS;

	/* This is for 400 mhz.  Set it to 0 for 800mhz */
	hw_ctx->cck_div = 0;
	hw_ctx->pll_bypass_mode = 0;

	hw_ctx->mipi_control = 0x18;
	hw_ctx->intr_en = 0xffffffff;
	hw_ctx->hs_tx_timeout = 0xdcf50;
	hw_ctx->lp_rx_timeout = 0xffff;
	hw_ctx->turn_around_timeout = 0x14;
	hw_ctx->device_reset_timer = 0xffff;
	hw_ctx->high_low_switch_count = 0x18;
	hw_ctx->init_count = 0x7d0;
	hw_ctx->eot_disable = 0x0;
	hw_ctx->lp_byteclk = 0x3;
	hw_ctx->clk_lane_switch_time_cnt = 0x18000b;
	hw_ctx->video_mode_format = 0x6;
	hw_ctx->dphy_param = 0x160d3610;

	/*set up func_prg*/
	hw_ctx->dsi_func_prg = (0x200 | dsi_config->lane_count);
	hw_ctx->mipi = PASS_FROM_SPHY_TO_AFE | dsi_config->lane_config;
}

void tc35876x_vid_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	int ret = 0;

	PSB_DEBUG_ENTRY("\n");

	p_funcs->get_config_mode = tc35876x_vid_get_config_mode;
	p_funcs->get_panel_info = tc35876x_vid_get_panel_info;
	p_funcs->dsi_controller_init = tc35876x_vid_dsi_controller_init;
	p_funcs->detect = tc35876x_vid_detect;
	p_funcs->reset = mdfld_dsi_tc35876x_panel_reset;
	p_funcs->power_on = tc35876x_vid_power_on;
	p_funcs->power_off = tc35876x_vid_power_off;
	p_funcs->set_brightness = tc35876x_vid_set_brightness;

	ret = cmi_lcd_hack_create_device();
	if (ret) {
		DRM_ERROR("cmi_lcd_hack_create_device() faild\n");
		return;
	}

	ret = i2c_add_driver(&cmi_lcd_i2c_driver);
	if (ret) {
		DRM_ERROR("add LCD I2C driver faild\n");
		return;
	}

	if (cmi_lcd_i2c_client) {
		/* Set backlight as register control only */
		ret = i2c_smbus_write_byte_data(cmi_lcd_i2c_client,
						DEVICE_CONTROL,
						BRT_MODE_PWM_REG_ONLY |
						BL_CTL_ENABLE);
		if (ret) {
			DRM_ERROR("i2c write brightness control reg failed\n");
			return;
		}
	} else {
		DRM_ERROR("cmi_lcd_i2c_client is NULL!\n");
		return;
	}

	ret = i2c_add_driver(&tc35876x_bridge_i2c_driver);
	if (ret) {
		DRM_ERROR("add bridge I2C driver faild\n");
		return;
	}
}

