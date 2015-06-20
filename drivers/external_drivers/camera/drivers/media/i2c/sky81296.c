/*
 * sky81296.c	SKY81296 flash LED driver
 *
 * Copyright 2014 Asus Corporation.
 * Author : Chung-Yi Chou <chung-yi_chou@asus.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2, or (at your option) any
 * later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/gpio.h>
#include <linux/leds.h>
#include <media/sky81296.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <linux/atomisp.h>
#include <linux/proc_fs.h>
#include <asm/intel_scu_pmic.h>
#include <linux/HWVersion.h>
#include <asm/intel-mid.h>
#include <linux/workqueue.h>

//PMIC GPIO4
#define GPIO4CTLO_REG			0x82
#define GPIO4CTLI_REG			0x92


/* Registers */
#define SKY81296_FLASH1_CURRENT		0x00
#define SKY81296_FLASH2_CURRENT		0x01
#define SKY81296_FLASH_TIMER		0x02
#define SKY81296_MOVIE_MODE_CURRENT	0x03
#define SKY81296_CONTROL1		0x04
#define SKY81296_CONTROL2		0x05
#define SKY81296_CONTROL3		0x06
#define SKY81296_BLINKING_MODE1_TIME	0x07
#define SKY81296_BLINKING_MODE2_TIME	0x08
#define SKY81296_FAULT			0x09
#define SKY81296_INPUT_MONITOR1		0x0A
#define SKY81296_INPUT_MONITOR2		0x0B

/* bit mask */
#define SKY81296_IFL1			0x1F
#define SKY81296_IFL2			0x1F
#define SKY81296_FLTM2			0xF0
#define SKY81296_FLTM1			0x0F
#define SKY81296_IMM2			0xF0
#define SKY81296_IMM1			0x0F
#define SKY81296_BLK_EN2		0x40
#define SKY81296_FL_EN2			0x20
#define SKY81296_MM_EN2			0x10
#define SKY81296_BLK_EN1		0x04
#define SKY81296_FL_EN1			0x02
#define SKY81296_MM_EN1			0x01
#define SKY81296_RESET			0x80
#define SKY81296_FLINHM			0x10
#define SKY81296_FLT_INH		0x08
#define SKY81296_ILIM			0x06
#define SKY81296_VINM			0x01
#define SKY81296_VINHYS			0xF0
#define SKY81296_VINTH			0x0F
#define SKY81296_TOFF1			0xF0
#define SKY81296_TON1			0x0F
#define SKY81296_TOFF2			0xF0
#define SKY81296_TON2			0x0F
#define SKY81296_VINMONEX		0x80
#define SKY81296_SC			0x40
#define SKY81296_OC			0x20
#define SKY81296_OTMP			0x10
#define SKY81296_FLED2			0x0C
#define SKY81296_FLED1			0x03
#define SKY81296_IFL_MON1		0x1F
#define SKY81296_IFL_MON2		0x1F

#define CTZ(b) __builtin_ctz(b)

#define SKY81296_NAME			"sky81296"
#define SKY81296_FLASH_MAX_BRIGHTNESS	SKY81296_FLASH_CURRENT_750MA
#define SKY81296_MOVIE_MAX_BRIGHTNESS	10
#define SKY81296_BLINK_MAX_BRIGHTNESS	SKY81296_MOVIE_MAX_BRIGHTNESS
#define SKY81296_FLASH_MAX_TIMER	SKY81296_FLASHTIMEOUT_1425MS
#define SKY81296_FLASH_DEFAULT_TIMER  SKY81296_FLASHTIMEOUT_1425MS

#define sky81296_suspend NULL
#define sky81296_resume  NULL

struct sky81296 {
	struct v4l2_subdev sd;
	struct mutex power_lock;
	int power_count;
	struct regmap *map;
	int enable_by_pin;
	unsigned char fault;
	enum sky81296_flash_current flash_current;
	enum sky81296_torch_current torch_current;
	enum sky81296_flash_timeout timeout;
	enum sky81296_flash_mode mode;
	struct sky81296_platform_data *pdata;
 	int irq;
	int gpio;
};
#define to_sky81296(p_sd)	container_of(p_sd, struct sky81296, sd)

static int light_record;
static struct sky81296 *inner_sky81296;
static int low_disable;
static int high_disable;
static int low_torch_disable;
static int high_torch_disable;
static struct workqueue_struct *flt_wq;
static struct delayed_work flt_sensor_dowork;

struct sky81296_ctrl_id {
	struct v4l2_queryctrl qc;
	int (*s_ctrl) (struct v4l2_subdev *sd, __u32 val);
	int (*g_ctrl) (struct v4l2_subdev *sd, __s32 *val);
};


static int sky81296_set_mode(struct sky81296 *flash, unsigned int new_mode)
{
	int ret;
	int value;

	switch (new_mode) {
		case SKY81296_MODE_SHUTDOWN:
			printk("[AsusFlash] flash shutdown\n");
			ret = regmap_write(flash->map,SKY81296_CONTROL1,0x0);
			break;
		case SKY81296_MODE_FLASH:
			printk("[AsusFlash] flash on\n");
			if( high_disable && low_disable ){
				printk("[AsusFlash] disable dual flash\n");
				ret = regmap_write(flash->map,SKY81296_CONTROL1,0x00);
				ret = regmap_read(flash->map,SKY81296_FLASH1_CURRENT, &value );
				printk("[AsusFlash] high flash set intensity (%x)\n",value);
				ret = regmap_read(flash->map,SKY81296_FLASH2_CURRENT, &value );
				printk("[AsusFlash] low flash set intensity (%x)\n",value);
			}else if( low_disable ){
				printk("[AsusFlash] disable low flash\n");
				ret = regmap_write(flash->map,SKY81296_CONTROL1,0x02);
				ret = regmap_read(flash->map,SKY81296_FLASH1_CURRENT, &value );
				printk("[AsusFlash] high flash set intensity (%x)\n",value);
				ret = regmap_read(flash->map,SKY81296_FLASH2_CURRENT, &value );
				printk("[AsusFlash] low flash set intensity (%x)\n",value);
			}else if( high_disable ){
				printk("[AsusFlash] disable high flash\n");
				ret = regmap_write(flash->map,SKY81296_CONTROL1,0x20);
				ret = regmap_read(flash->map,SKY81296_FLASH1_CURRENT, &value );
				printk("[AsusFlash] high flash set intensity (%x)\n",value);
				ret = regmap_read(flash->map,SKY81296_FLASH2_CURRENT, &value );
				printk("[AsusFlash] low flash set intensity (%x)\n",value);
			}else{
				printk("[AsusFlash] dual flash on\n");
				ret = regmap_write(flash->map,SKY81296_CONTROL1,0x22);
				ret = regmap_read(flash->map,SKY81296_FLASH1_CURRENT, &value );
				printk("[AsusFlash] high flash set intensity (%x)\n",value);
				ret = regmap_read(flash->map,SKY81296_FLASH2_CURRENT, &value );
				printk("[AsusFlash] low flash set intensity (%x)\n",value);
			}
			break;
		case SKY81296_MODE_INDICATOR:
			printk("[AsusFlash] TORCH_INDICATOR on\n");
			if( high_torch_disable  && low_torch_disable ){
				printk("[AsusFlash] disable dual flash torch\n");
				ret = regmap_write(flash->map,SKY81296_CONTROL1,0x00);
				ret = regmap_read(flash->map,SKY81296_MOVIE_MODE_CURRENT, &value );
				printk("[AsusFlash] torch intensity (%x)\n",value);
			}else if( low_torch_disable ){
				printk("[AsusFlash] disable low flash torch\n");
				ret = regmap_write(flash->map,SKY81296_CONTROL1,0x01);
				ret = regmap_read(flash->map,SKY81296_MOVIE_MODE_CURRENT, &value );
				printk("[AsusFlash] torch intensity (%x)\n",value);
			}else if( high_torch_disable ){
				printk("[AsusFlash] disable high flash torch\n");
				ret = regmap_write(flash->map,SKY81296_CONTROL1,0x10);
				ret = regmap_read(flash->map,SKY81296_MOVIE_MODE_CURRENT, &value );
				printk("[AsusFlash] torch intensity (%x)\n",value);
			}else{
				printk("[AsusFlash] dual flash torch on\n");
				ret = regmap_write(flash->map,SKY81296_CONTROL1,0x11);
				ret = regmap_read(flash->map,SKY81296_MOVIE_MODE_CURRENT, &value );
				printk("[AsusFlash] torch intensity (%x)\n",value);
			}
			break;
		case SKY81296_MODE_TORCH:
			printk("[AsusFlash] TORCH on\n");
			if( high_torch_disable  && low_torch_disable ){
				printk("[AsusFlash] disable dual flash torch\n");
				ret = regmap_write(flash->map,SKY81296_CONTROL1,0x00);
				ret = regmap_read(flash->map,SKY81296_MOVIE_MODE_CURRENT, &value );
				printk("[AsusFlash] torch intensity (%x)\n",value);
			}else if( low_torch_disable ){
				printk("[AsusFlash] disable low flash torch\n");
				ret = regmap_write(flash->map,SKY81296_CONTROL1,0x01);
				ret = regmap_read(flash->map,SKY81296_MOVIE_MODE_CURRENT, &value );
				printk("[AsusFlash] torch intensity (%x)\n",value);
			}else if( high_torch_disable ){
				printk("[AsusFlash] disable high flash torch\n");
				ret = regmap_write(flash->map,SKY81296_CONTROL1,0x10);
				ret = regmap_read(flash->map,SKY81296_MOVIE_MODE_CURRENT, &value );
				printk("[AsusFlash] torch intensity (%x)\n",value);
			}else{
				printk("[AsusFlash] dual flash torch on\n");
				ret = regmap_write(flash->map,SKY81296_CONTROL1,0x11);
				ret = regmap_read(flash->map,SKY81296_MOVIE_MODE_CURRENT, &value );
				printk("[AsusFlash] torch intensity (%x)\n",value);
			}
			break;
		default:
			return -EINVAL;
	}
	if (ret == 0)
		flash->mode = new_mode;
	return ret;
}

static int sky81296_set_flash(struct sky81296 *flash)
{
	int val_low,val_high,value;
	int ret;

	val_low = flash->flash_current >> 8;       //Low temp
	val_high = (flash->flash_current ) &  0xff; //High temp

	printk("[AsusFlash] flash intensity = %d (%d,%d) \n", flash->flash_current , val_low , val_high  );

	if(val_low == 0){
		low_disable = true;
	}else{
		low_disable = false;
		val_low = val_low - 1;
	}

	if(val_high == 0){
		high_disable = true;
	}else{
		high_disable = false;
		val_high = val_high - 1;
	}

	if(val_low > SKY81296_FLASH_MAX_BRIGHTNESS || val_low < 0 ){
		val_low = SKY81296_FLASH_MAX_BRIGHTNESS;
	}

	if(val_high > SKY81296_FLASH_MAX_BRIGHTNESS || val_high < 0 ){
		val_high = SKY81296_FLASH_MAX_BRIGHTNESS;
	}

	ret = regmap_write(flash->map,SKY81296_FLASH1_CURRENT,val_high);
	ret = regmap_write(flash->map,SKY81296_FLASH2_CURRENT,val_low);

	ret = regmap_read(flash->map,SKY81296_FLASH1_CURRENT, &value );
	printk("[AsusFlash] high flash set intensity (%x)\n",value);
	ret = regmap_read(flash->map,SKY81296_FLASH2_CURRENT, &value );
	printk("[AsusFlash] low flash set intensity (%x)\n",value);

	flash->mode = SKY81296_MODE_FLASH;
	return ret;
}

static int sky81296_set_torch(struct sky81296 *flash)
{
	int val_low,val_high,value;
	int ret;

	val_low = flash->torch_current >> 8;       //Low temp
	val_high = (flash->torch_current ) &  0xff; //High temp

	printk("[AsusFlash] torch intensity = %d (%d,%d)\n", flash->torch_current , val_low , val_high);

	if(val_low == 0){
		low_torch_disable = true;
	}else{
		low_torch_disable = false;
		val_low = val_low - 1;
	}

	if(val_high == 0){
		high_torch_disable = true;
	}else{
		high_torch_disable = false;
		val_high = val_high - 1;
	}

	ret = regmap_write(flash->map,SKY81296_MOVIE_MODE_CURRENT, (val_low << 4 | val_high) );
	ret = regmap_read(flash->map,SKY81296_MOVIE_MODE_CURRENT, &value );
	printk("[AsusFlash] torch set intensity (%x)\n",value);
	flash->mode = SKY81296_MODE_TORCH;

	return ret;
}

static const struct regmap_config sky81296_config =
{
	.reg_bits = 8,
	.val_bits = 8,
};


/* -----------------------------------------------------------------------------
 * V4L2 controls
 */

static int sky81296_s_flash_timeout(struct v4l2_subdev *sd, u32 val)
{
	struct sky81296 *flash = to_sky81296(sd);
	int ret;
// Protect: Setting this to 1425ms
	if( val > SKY81296_FLASH_MAX_TIMER || val < SKY81296_FLASHTIMEOUT_OFF){
		val = SKY81296_FLASH_MAX_TIMER;
	}
	ret = regmap_write(flash->map,SKY81296_FLASH_TIMER, (val << 4 | val) );
	flash->timeout = val;
	return 0;
}

static int sky81296_g_flash_timeout(struct v4l2_subdev *sd, s32 *val)
{
	struct sky81296 *flash = to_sky81296(sd);
	*val = flash->timeout;
	return 0;
}

static int sky81296_s_flash_intensity(struct v4l2_subdev *sd, u32 intensity)
{

	struct sky81296 *flash = to_sky81296(sd);

	flash->flash_current = intensity;

	return sky81296_set_flash(flash);
}

static int sky81296_g_flash_intensity(struct v4l2_subdev *sd, s32 *val)
{

	struct sky81296 *flash = to_sky81296(sd);
	int value1,value2;
	int ret;
	ret = regmap_read(flash->map,SKY81296_FLASH1_CURRENT,&value1);
	ret = regmap_read(flash->map,SKY81296_FLASH2_CURRENT,&value2);
	*val = value1 * 100 + value2;

	return 0;
}

static int sky81296_s_torch_intensity(struct v4l2_subdev *sd, u32 intensity)
{

	struct sky81296 *flash = to_sky81296(sd);
/*
	Todo : Mapping minimum and maximum current for torch mode.
*/
	flash->torch_current = intensity;

	return sky81296_set_torch(flash);
}

static int sky81296_g_torch_intensity(struct v4l2_subdev *sd, s32 *val)
{

	struct sky81296 *flash = to_sky81296(sd);
	*val = flash->torch_current;
	return 0;
}

static int sky81296_s_indicator_intensity(struct v4l2_subdev *sd, u32 intensity)
{

	struct sky81296 *flash = to_sky81296(sd);
/*
	Todo : Mapping minimum and maximum current for torch mode.
*/
	flash->torch_current = intensity;

	return sky81296_set_torch(flash);
}

static int sky81296_g_indicator_intensity(struct v4l2_subdev *sd, s32 *val)
{
	struct sky81296 *flash = to_sky81296(sd);

	*val = (u32)flash->torch_current;

	return 0;
}

static int sky81296_s_flash_strobe(struct v4l2_subdev *sd, u32 val)
{
	struct sky81296 *flash = to_sky81296(sd);
	int ret;
	ret = regmap_write(flash->map,SKY81296_CONTROL1,0x44);
	return ret;
}

static int sky81296_s_flash_mode(struct v4l2_subdev *sd, u32 new_mode)
{
	struct sky81296 *flash = to_sky81296(sd);
	unsigned int mode;
	switch (new_mode) {
	case ATOMISP_FLASH_MODE_OFF:
		mode = SKY81296_MODE_SHUTDOWN;
		break;
	case ATOMISP_FLASH_MODE_FLASH:
		mode = SKY81296_MODE_FLASH;
		break;
	case ATOMISP_FLASH_MODE_INDICATOR:
		mode = SKY81296_MODE_INDICATOR;
		break;
	case ATOMISP_FLASH_MODE_TORCH:
		mode = SKY81296_MODE_TORCH;
		break;
	default:
		return -EINVAL;
	}

	return sky81296_set_mode(flash, mode);
}

static int sky81296_g_flash_mode(struct v4l2_subdev *sd, s32 * val)
{
	struct sky81296 *flash = to_sky81296(sd);
	*val = flash->mode;
	return 0;
}

static int sky81296_g_flash_status(struct v4l2_subdev *sd, s32 *val)
{
	struct sky81296 *flash = to_sky81296(sd);
	int ret;

	ret = regmap_read(flash->map, SKY81296_CONTROL1,val);
	if (ret < 0)
		return ret;
/*
	It should be mapped to enum atomisp_flash_status
	for
	ATOMISP_FLASH_STATUS_OK,
	ATOMISP_FLASH_STATUS_HW_ERROR,
	ATOMISP_FLASH_STATUS_INTERRUPTED,
	ATOMISP_FLASH_STATUS_TIMEOUT,
*/
	return 0;
}

static int sky81296_g_flash_status_register(struct v4l2_subdev *sd, s32 *val)
{

	struct sky81296 *flash = to_sky81296(sd);
	int ret;

	ret = regmap_read(flash->map, SKY81296_CONTROL1,val);

	if (ret < 0)
		return ret;

	return 0;
}

static const struct sky81296_ctrl_id sky81296_ctrls[] = {
	s_ctrl_id_entry_integer(V4L2_CID_FLASH_TIMEOUT,
				"Flash Timeout",
				0,
				1024,
				1,
				1024,
				0,
				sky81296_s_flash_timeout,
				sky81296_g_flash_timeout),
	s_ctrl_id_entry_integer(V4L2_CID_FLASH_INTENSITY,
				"Flash Intensity",
				0,
				255,
				1,
				255,
				0,
				sky81296_s_flash_intensity,
				sky81296_g_flash_intensity),
	s_ctrl_id_entry_integer(V4L2_CID_FLASH_TORCH_INTENSITY,
				"Torch Intensity",
				0,
				255,
				1,
				255,
				0,
				sky81296_s_torch_intensity,
				sky81296_g_torch_intensity),
	s_ctrl_id_entry_integer(V4L2_CID_FLASH_INDICATOR_INTENSITY,
				"Indicator Intensity",
				0,
				255,
				1,
				100,
				0,
				sky81296_s_indicator_intensity,
				sky81296_g_indicator_intensity),
	s_ctrl_id_entry_boolean(V4L2_CID_FLASH_STROBE,
				"Flash Strobe",
				0,
				0,
				sky81296_s_flash_strobe,
				NULL),
	s_ctrl_id_entry_integer(V4L2_CID_FLASH_MODE,
				"Flash Mode",
				0,   /* don't assume any enum ID is first */
				100, /* enum value, may get extended */
				1,
				ATOMISP_FLASH_MODE_OFF,
				0,
				sky81296_s_flash_mode,
				sky81296_g_flash_mode),
	s_ctrl_id_entry_integer(V4L2_CID_FLASH_STATUS,
				"Flash Status",
				0,   /* don't assume any enum ID is first */
				100, /* enum value, may get extended */
				1,
				ATOMISP_FLASH_STATUS_OK,
				0,
				NULL,
				sky81296_g_flash_status),
	s_ctrl_id_entry_integer(V4L2_CID_FLASH_STATUS_REGISTER,
				"Flash Status Register",
				0,   /* don't assume any enum ID is first */
				100, /* enum value, may get extended */
				1,
				0,
				0,
				NULL,
				sky81296_g_flash_status_register),
};

static const struct sky81296_ctrl_id *find_ctrl_id(unsigned int id)
{
	int i;
	int num;

	num = ARRAY_SIZE(sky81296_ctrls);
	for (i = 0; i < num; i++) {
		if (sky81296_ctrls[i].qc.id == id)
			return &sky81296_ctrls[i];
	}

	return NULL;
}

static int sky81296_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	int num;

	if (!qc)
		return -EINVAL;

	num = ARRAY_SIZE(sky81296_ctrls);
	if (qc->id >= num)
		return -EINVAL;

	*qc = sky81296_ctrls[qc->id].qc;

	return 0;
}

static int sky81296_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	const struct sky81296_ctrl_id *s_ctrl;

	if (!ctrl)
		return -EINVAL;

	s_ctrl = find_ctrl_id(ctrl->id);
	if (!s_ctrl)
		return -EINVAL;

	return s_ctrl->s_ctrl(sd, ctrl->value);
}

static int sky81296_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	const struct sky81296_ctrl_id *s_ctrl;

	if (!ctrl)
		return -EINVAL;

	s_ctrl = find_ctrl_id(ctrl->id);
	if (s_ctrl == NULL)
		return -EINVAL;

	return s_ctrl->g_ctrl(sd, &ctrl->value);
}

static int sky81296_setup(struct sky81296 *flash)
{
	struct i2c_client *client = v4l2_get_subdevdata(&flash->sd);
	struct regmap *map;
	unsigned char value;
	int ret = 0;

	map = devm_regmap_init_i2c(client, &sky81296_config);
	if (IS_ERR(map)){
		return PTR_ERR(map);
	}



	switch(flash->pdata->current_limit) {
		case 2420: /* 2420mA to 3080mA */
			value = 0 << CTZ(SKY81296_ILIM);
			break;
		case 2600: /* 2600mA to 3300mA */
			value = 1 << CTZ(SKY81296_ILIM);
			break;
		case 2800: /* 2800mA to 3600mA */
			value = 2 << CTZ(SKY81296_ILIM);
			break;
		case 3000: /* 3000mA to 3950mA */
			value = 3 << CTZ(SKY81296_ILIM);
			break;
		default:
			return -EINVAL;
	}

	if (flash->pdata->disable_short_led_report)
		value |= SKY81296_FLT_INH;

	if (flash->pdata->shutoff_on_inhibit_mode)
		value |= SKY81296_FLINHM;

	if (flash->pdata->enable_voltage_monitor)
		value |= SKY81296_VINM;

	ret = regmap_write(map, SKY81296_CONTROL2, value);
	if (IS_ERR_VALUE(ret))
		return ret;


	if (!flash->pdata->enable_voltage_monitor){
		flash->map = map;
		return 0;
	}

	if (flash->pdata->input_voltage_threshold < 2800 ||
			flash->pdata->input_voltage_threshold > 3900)
		return -EINVAL;

	value = ((flash->pdata->input_voltage_threshold - 2800) / 100)
			<< CTZ(SKY81296_VINTH);
	if (flash->pdata->input_voltage_hysteresis < 2900 ||
			flash->pdata->input_voltage_hysteresis > 4000)
		return -EINVAL;

	value |= (((flash->pdata->input_voltage_hysteresis - 2900) / 100) + 1 )
			<< CTZ(SKY81296_VINHYS);
	//Voltage Monitor Hysteresis = 3.4V/Input Voltage Monitor Threshold = 3.3V
	ret = regmap_write(map, SKY81296_CONTROL3, value);


	flash->timeout = SKY81296_FLASH_DEFAULT_TIMER;
	ret = regmap_write(map, SKY81296_FLASH_TIMER , (flash->timeout << 4 | flash->timeout) );

	flash->map = map;

	return ret;
}

static int __sky81296_s_power(struct sky81296 *flash, int power)
{
	return 0;
}

static int sky81296_s_power(struct v4l2_subdev *sd, int power)
{
	struct sky81296 *flash = to_sky81296(sd);
	int ret = 0;

	mutex_lock(&flash->power_lock);

	if (flash->power_count == !power) {
		ret = __sky81296_s_power(flash, !!power);
		if (ret < 0)
			goto done;
	}

	flash->power_count += power ? 1 : -1;
	WARN_ON(flash->power_count < 0);

done:
	mutex_unlock(&flash->power_lock);
	return ret;
}

static void sky81296_torch_on_num(struct sky81296 *flash , int intensity , int num){
	int ret;
	if( intensity < SKY81296_TORCH_CURRENT_25MA || intensity > SKY81296_TORCH_CURRENT_250MA ){
		intensity = SKY81296_TORCH_CURRENT_250MA;
	}
	printk("[AsusFlash] Set Torch on %u \n", ( intensity << 4 | intensity ));
	ret = regmap_write(flash->map,SKY81296_MOVIE_MODE_CURRENT,( intensity << 4 | intensity ) );

	if( num == 0){
		ret = regmap_write(flash->map,SKY81296_CONTROL1,0x11);
	}else if( num == 1){
		ret = regmap_write(flash->map,SKY81296_CONTROL1,0x01);
	}else if( num == 2){
		ret = regmap_write(flash->map,SKY81296_CONTROL1,0x10);
	}
}

static void sky81296_torch_on(struct sky81296 *flash , int intensity){
	int ret;
	if( intensity < SKY81296_TORCH_CURRENT_25MA || intensity > SKY81296_TORCH_CURRENT_250MA ){
		intensity = SKY81296_TORCH_CURRENT_250MA;
	}
	printk("[AsusFlash] Set Torch on %u \n", ( intensity << 4 | intensity ));
	ret = regmap_write(flash->map,SKY81296_MOVIE_MODE_CURRENT,( intensity << 4 | intensity ) );
	ret = regmap_write(flash->map,SKY81296_CONTROL1,0x11);
}

static void sky81296_flash_on(struct sky81296 *flash){
	int ret;
	printk("[AsusFlash] Set Flash on \n");
	ret = regmap_write(flash->map,SKY81296_CONTROL1,0x22);
}

static void sky81296_flash_off(struct sky81296 *flash){
	int ret;
	printk("[AsusFlash] Set Flash off \n");
	ret = regmap_write(flash->map,SKY81296_CONTROL1,0x0);
}

static long sky81296_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{

	struct sky81296 *flash = to_sky81296(sd);
	int input_arg = 0;

	switch (cmd) {
	case ATOMISP_TEST_CMD_SET_TORCH:
		input_arg = *(int *)arg;
		if(input_arg == 0){
			sky81296_flash_off(flash);
		}else{
			sky81296_torch_on(flash , 4);
		}
		return 0;
	case ATOMISP_TEST_CMD_SET_FLASH:
		input_arg = *(int *)arg;
		if(input_arg == 0){
			sky81296_flash_off(flash);
		}else{
			sky81296_flash_on(flash);
		}
		return 0;
	default:
		return -EINVAL;
	}

	return 0;
}

static ssize_t sky81296_show(struct file *dev, char *buffer, size_t count, loff_t *ppos)
{

	int len = 0;
	ssize_t ret = 0;
	char *buff;

	printk(KERN_INFO "[AsusFlash] Read Flash %d\n", light_record);
	buff = kmalloc(100,GFP_KERNEL);
	if(!buff)
		return -ENOMEM;

	len += sprintf(buff+len, "%d\n", light_record);
	ret = simple_read_from_buffer(buffer,count,ppos,buff,len);
	kfree(buff);

	return ret;

}

static ssize_t sky81296_store(struct file *dev, const char *buf, size_t count, loff_t *loff)
{
	int set_light = -1;
	// int map_offset = SKY81296_TORCH_CURRENT_125MA;//4 //SKY81296_TORCH_CURRENT_NUM;
	int map_num;

	sscanf(buf, "%d", &set_light);
	printk(KERN_INFO "[AsusFlash] Set light to %d\n", set_light);
	if ( (set_light == light_record)){
		return count;
	}
	if(set_light < 0 || set_light >200){
		return -1;
	}else if (set_light == 0 ){
		sky81296_flash_off(inner_sky81296);
		light_record = set_light;
	}else{
		light_record = set_light;
		sky81296_flash_off(inner_sky81296);
		// map_num = set_light - (set_light % map_offset);
		// map_num = map_num / (map_offset);
		map_num = mapping_torch_intensity(set_light);
		printk(KERN_INFO "[AsusFlash] Real set light to %d\n", map_num);
		//set 1 for high tempature LED.
		sky81296_torch_on_num(inner_sky81296,map_num,1);
	}

	return count;
}

static const struct file_operations flash_proc_fops = {
	.read = sky81296_show,
	.write = sky81296_store,
};

static const struct v4l2_subdev_core_ops sky81296_core_ops = {
	.queryctrl = sky81296_queryctrl,
	.g_ctrl = sky81296_g_ctrl,
	.s_ctrl = sky81296_s_ctrl,
	.s_power = sky81296_s_power,
	.ioctl = sky81296_ioctl,
};

static const struct v4l2_subdev_ops sky81296_ops = {
	.core = &sky81296_core_ops,
};


static int sky81296_detect(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct i2c_adapter *adapter = client->adapter;
	struct sky81296 *flash = to_sky81296(sd);
	int ret;
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "sky81296_detect i2c error\n");
		return -ENODEV;
	}

	/* Power up the flash driver and reset it */
	ret = sky81296_s_power(&flash->sd, 1);
	if (ret < 0)
		return ret;

	/* Setup default values. This makes sure that the chip is in a known
	 * state.
	 */
	ret = sky81296_setup(flash); //Now Not-ready
	if (ret < 0)
		goto fail;

	dev_dbg(&client->dev, "Successfully detected sky81296 LED flash\n");
	sky81296_s_power(&flash->sd, 0);
	return 0;

fail:
	sky81296_s_power(&flash->sd, 0);
	return ret;
}

static int sky81296_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return sky81296_s_power(sd, 1);
}

static int sky81296_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return sky81296_s_power(sd, 0);
}

static const struct v4l2_subdev_internal_ops sky81296_internal_ops = {
	.registered = sky81296_detect,
	.open = sky81296_open,
	.close = sky81296_close,
};

static int sky81296_gpio_init(struct i2c_client *client)
{
#if 0
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sky81296 *flash = to_sky81296(sd);
	struct sky81296_platform_data *pdata = flash->pdata;
	int ret;

	ret = gpio_request(pdata->gpio_enable, "flash enable");
	if (ret < 0)
		return ret;

	ret = gpio_direction_output(pdata->gpio_enable, 1);
	if (ret < 0)
		goto err_gpio_enable;
	return 0;

err_gpio_enable:
	gpio_free(pdata->gpio_enable);
	return ret;
#else

	int ret;

    switch (Read_PROJ_ID()) {

	case PROJ_ID_ZE550ML:
	case PROJ_ID_ZE551ML:
	case PROJ_ID_ZR550ML:
	case PROJ_ID_ZE500ML:
	case PROJ_ID_ZE551ML_CKD:

	switch (Read_HW_ID()) {
		case HW_ID_EVB:
			pr_info("Hardware VERSION = EVB, sky81296 does not support.\n");
			break;
		case HW_ID_SR1:
		case HW_ID_SR2:
		case HW_ID_ER:
		case HW_ID_ER1_1:
		case HW_ID_ER1_2:
		case HW_ID_pre_PR:
		case HW_ID_PR:
		case HW_ID_MP:
                        pr_info("sky81296 --> HW_ID = 0x%x\n", Read_HW_ID());
			pr_info("sky81296 --> PMIC GPIO4CTLO_REG pull high\n");
	    	   	ret = intel_scu_ipc_iowrite8(GPIO4CTLO_REG, 0x31);
			if (ret) {
			    printk(KERN_ALERT "sky81296 --> Failed to output PMIC GPIO4 HIGH\n");
			    return ret;
	                }
		break;
		default:
			pr_info("sky81296 --> HW_ID does not define\n");
		break;
	}
	break;
	case PROJ_ID_ZX550ML:
		pr_info("DISABLE FLASH FOR CES in ZX550ML\n");
		break;
	default:
		pr_info("Project ID is not defined\n");
	break;
    }//end switch

	return 0;
#endif
}

static int sky81296_gpio_uninit(struct i2c_client *client)
{
#if 0
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sky81296 *flash = to_sky81296(sd);
	struct sky81296_platform_data *pdata = flash->pdata;
	int ret;

	ret = gpio_direction_output(pdata->gpio_enable, 0);
	if (ret < 0)
		return ret;

	gpio_free(pdata->gpio_enable);
	return 0;
#else

	int ret;

    switch (Read_PROJ_ID()) {

	case PROJ_ID_ZE550ML:
	case PROJ_ID_ZE551ML:
	case PROJ_ID_ZR550ML:
	case PROJ_ID_ZE500ML:
	case PROJ_ID_ZE551ML_CKD:

	switch (Read_HW_ID()) {
		case HW_ID_EVB:
			pr_info("Hardware VERSION = EVB, sky81296 does not support.\n");
			break;
		case HW_ID_SR1:
		case HW_ID_SR2:
		case HW_ID_ER:
		case HW_ID_ER1_1:
		case HW_ID_ER1_2:
		case HW_ID_pre_PR:
		case HW_ID_PR:
		case HW_ID_MP:
                        pr_info("sky81296 --> HW_ID = 0x%x\n", Read_HW_ID());
			pr_info("sky81296 --> PMIC GPIO4CTLO_REG pull low\n");
			ret = intel_scu_ipc_iowrite8(GPIO4CTLO_REG, 0x30);
			    if (ret) {
				printk(KERN_ALERT "Failed to output PMIC GPIO4 LOW\n");
				return ret;
			    }
		break;
		default:
			pr_info("sky81296 --> HW_ID does not define\n");
		break;
	}
	break;
	case PROJ_ID_ZX550ML:
		pr_info("DISABLE FLASH FOR CES in ZX550ML\n");
		break;
	default:
		pr_info("Project ID is not defined\n");
	break;
    }//end switch

	return 0;
#endif
}


static void flt_do_work_function(struct work_struct *dat)
{
	struct sky81296 *flash = inner_sky81296;
	int value;
	int ret;

	pr_info("[%s] sky81296_interrupt_handler = %d\n", SKY81296_NAME,flash->irq);
	ret = regmap_read(flash->map,SKY81296_FAULT,&value);
/*
#define SKY81296_VINMONEX		0x80
#define SKY81296_SC			0x40
#define SKY81296_OC			0x20
#define SKY81296_OTMP			0x10
#define SKY81296_FLED2			0x0C
#define SKY81296_FLED1			0x03
*/

	if(value & SKY81296_VINMONEX)
	    pr_info("[%s] error sky81296 SKY81296_VINMONEX Fault\n", SKY81296_NAME);
	if(value & SKY81296_SC)
	    pr_info("[%s] error sky81296 SKY81296_SC Fault\n", SKY81296_NAME);
	if(value & SKY81296_OC)
	    pr_info("[%s] error sky81296 SKY81296_OC Fault\n", SKY81296_NAME);
	if(value & SKY81296_OTMP)
	    pr_info("[%s] error sky81296 SKY81296_OTMP Fault\n", SKY81296_NAME);
	if(value & SKY81296_FLED2)
	    pr_info("[%s] error sky81296 SKY81296_FLED2 Fault\n", SKY81296_NAME);
	if(value & SKY81296_FLED1)
	    pr_info("[%s] error sky81296 SKY81296_FLED1 Fault\n", SKY81296_NAME);

}


static irqreturn_t sky81296_interrupt_handler(int irq, void *sd)
{
	queue_delayed_work(flt_wq, &flt_sensor_dowork, msecs_to_jiffies(0));
	return IRQ_HANDLED;
}

static int set_irq(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sky81296 *flash = to_sky81296(sd);
	int rc = 0 ;

	pr_info("[%s] sky81296 FLED_DRIVER_FLT# gpio = %d\n", SKY81296_NAME,flash->gpio);
	flash->irq = gpio_to_irq(flash->gpio);
	pr_info("[%s] sky81296 FLED_DRIVER_FLT# irq = %d\n", SKY81296_NAME,flash->irq);
	rc = request_irq(flash->irq,sky81296_interrupt_handler,
			IRQF_SHARED|IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING,"sky81296_irq",flash);
	if (rc<0) {
		pr_info("[%s] Could not register for sky81296 interrupt, irq = %d, rc = %d\n", SKY81296_NAME,flash->irq,rc);
		rc = -EIO;
		goto err_gpio_request_irq_fail ;
	}

	enable_irq_wake(flash->irq);

	return 0;

err_gpio_request_irq_fail:
	return rc;
}

static int sky81296_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;
	struct sky81296 *flash;
	struct proc_dir_entry* proc_entry_flash;
	void* dummy = NULL;

	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "no platform data\n");
		return -ENODEV;
	}

	flash = kzalloc(sizeof(*flash), GFP_KERNEL);
	if (!flash) {
		dev_err(&client->dev, "out of memory\n");
		return -ENOMEM;
	}

	flash->pdata = client->dev.platform_data;

	v4l2_i2c_subdev_init(&flash->sd, client, &sky81296_ops);//Now
	flash->sd.internal_ops = &sky81296_internal_ops;
	flash->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	flash->mode = SKY81296_MODE_SHUTDOWN;


	err = media_entity_init(&flash->sd.entity, 0, NULL, 0);
	if (err) {
		dev_err(&client->dev, "error initialize a media entity.\n");
		goto fail1;
	}

	flash->sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV_FLASH;

	mutex_init(&flash->power_lock);

	err = sky81296_gpio_init(client);
	if (err) {
		dev_err(&client->dev, "gpio request/direction_output fail");
		goto fail2;
	}

//Add node for flash control
	proc_entry_flash = proc_create_data("driver/asus_flash_brightness", 0666, NULL, &flash_proc_fops, dummy);
	proc_set_user(proc_entry_flash, 1000, 1000);
	inner_sky81296 = flash;


    switch (Read_PROJ_ID()) {

	case PROJ_ID_ZE550ML:
	case PROJ_ID_ZE551ML:
	case PROJ_ID_ZR550ML:
	case PROJ_ID_ZE500ML:
	case PROJ_ID_ZE551ML_CKD:

	switch (Read_HW_ID()) {
		case HW_ID_EVB:
			pr_info("Hardware VERSION = EVB, sky81296 does not support.\n");
			break;
		case HW_ID_SR1:
		case HW_ID_SR2:
		case HW_ID_ER:
		case HW_ID_ER1_1:
		case HW_ID_ER1_2:
		case HW_ID_pre_PR:
		case HW_ID_PR:
		case HW_ID_MP:
                        pr_info("sky81296 --> HW_ID = 0x%x\n", Read_HW_ID());
			flash->gpio = get_gpio_by_name("FLED_DRIVER_FLT#");
			pr_info("sky81296 --> GPIO FLED_DRIVER_FLT# should be 5.\n");
			pr_info("sky81296 --> GPIO FLED_DRIVER_FLT# is %d.\n",flash->gpio);
	 		if (!gpio_is_valid(flash->gpio))
			    {
				pr_info("[%s] GPIO for sky81296 FLED_DRIVER_FLT# does not exist.\n", SKY81296_NAME);
				err= -1;
				goto fail2;
			}
			gpio_request(flash->gpio,"FLED_DRIVER_FLT#");
			gpio_direction_input(flash->gpio);

		break;
		default:
			pr_info("sky81296 --> HW_ID does not define\n");
		break;
	}
	break;
	case PROJ_ID_ZX550ML:
		pr_info("DISABLE FLASH FOR CES in ZX550ML\n");
		break;
	default:
		pr_info("Project ID is not defined\n");
	break;
    }//end switch


	//set irq
	err = set_irq(client);
	if (err < 0)
		goto fail_for_irq;

	flt_wq = create_singlethread_workqueue("flto_wq");
	INIT_DELAYED_WORK(&flt_sensor_dowork, flt_do_work_function);
	return 0;

fail_for_irq:
	gpio_free(flash->gpio);
fail2:
	media_entity_cleanup(&flash->sd.entity);
fail1:
	v4l2_device_unregister_subdev(&flash->sd);
	kfree(flash);

	return err;
}

static int sky81296_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sky81296 *flash = to_sky81296(sd);
	int ret;

	media_entity_cleanup(&flash->sd.entity);
	v4l2_device_unregister_subdev(sd);


	ret = sky81296_gpio_uninit(client);
	if (ret < 0)
		goto fail;

	kfree(flash);

    	disable_irq(flash->irq);
	gpio_free(flash->gpio);
        destroy_workqueue(flt_wq);

	return 0;
fail:
	dev_err(&client->dev, "gpio request/direction_output fail");
	return ret;
}

static const struct i2c_device_id sky81296_ids[] = {
	{SKY81296_NAME, 0},
	{ },
};
MODULE_DEVICE_TABLE(i2c, sky81296_ids);

static const struct dev_pm_ops sky81296_pm_ops = {
	.suspend = sky81296_suspend,
	.resume = sky81296_resume,
};

static struct i2c_driver sky81296_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = SKY81296_NAME,
		.pm   = &sky81296_pm_ops,
	},
	.probe = sky81296_probe,
	.remove = sky81296_remove,
	.id_table = sky81296_ids,
};

static __init int init_sky81296(void)
{
	return i2c_add_driver(&sky81296_driver);
}

static __exit void exit_sky81296(void)
{
	i2c_del_driver(&sky81296_driver);
}

module_init(init_sky81296);
module_exit(exit_sky81296);
MODULE_AUTHOR("Chung-Yi Chou <chung-yi_chou@asus.com>");
MODULE_DESCRIPTION("SKY81296 LED Flash Driver");
MODULE_LICENSE("GPL");

