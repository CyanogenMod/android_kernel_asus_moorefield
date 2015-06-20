/**
 * 2010 - 2013 Goodix Technology.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * Version: 2.0
 * Authors: andrew@goodix.com, meta@goodix.com
 * Release Date: 2013/04/25
 * Revision record:
 *      V1.0:
 *          first Release. By Andrew, 2012/08/31
 *      V1.2:
 *          modify gtp_reset_guitar,slot report,tracking_id & 0x0F.
 *		    By Andrew, 2012/10/15
 *      V1.4:
 *          modify gt9xx_update.c. By Andrew, 2012/12/12
 *      V1.6:
 *          1. new heartbeat/esd_protect mechanism(add external watchdog)
 *          2. doze mode, sliding wakeup
 *          3. 3 more cfg_group(GT9 Sensor_ID: 0~5)
 *          3. config length verification
 *          4. names & comments
 *                  By Meta, 2013/03/11
 *      V1.8:
 *          1. pen/stylus identification
 *          2. read double check & fixed config support
 *          3. new esd & slide wakeup optimization
 *                  By Meta, 2013/06/08
 *      V2.0:
 *          1. compatible with GT9XXF
 *          2. send config after resume
 *                  By Meta, 2013/08/06
 */

#include <linux/irq.h>
#include "gt9xx.h"

#if GTP_ICS_SLOT_REPORT
#include <linux/input/mt.h>
#endif

static const char *goodix_ts_name = "goodix_ts";
static const char *goodix_ts_phys = "input/ts";
static struct workqueue_struct *goodix_wq;
struct i2c_client * i2c_connect_client = NULL;
u8 config[GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH] =
			{GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};

#if GTP_HAVE_TOUCH_KEY
    static const u16 touch_key_array[] = GTP_KEY_TAB;
#define GTP_MAX_KEY_NUM  (sizeof(touch_key_array)/sizeof(touch_key_array[0]))

#if GTP_DEBUG_ON
static const int  key_codes[] = {KEY_HOME, KEY_BACK, KEY_MENU, KEY_SEARCH};
static const char *key_names[] =
		{"Key_Home", "Key_Back", "Key_Menu", "Key_Search"};
#endif
#endif

static int gtp_i2c_test(struct i2c_client *client);
void gtp_reset_guitar(struct i2c_client *client, int ms);
int gtp_send_cfg(struct i2c_client *client);
void gtp_int_sync(int ms);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void goodix_ts_early_suspend(struct early_suspend *h);
static void goodix_ts_late_resume(struct early_suspend *h);
#endif

#if GTP_CREATE_WR_NODE
extern int init_wr_node(struct i2c_client*);
extern void uninit_wr_node(void);
#endif

#if GTP_AUTO_UPDATE
extern u8 gup_init_update_proc(struct goodix_ts_data *);
#endif

#if GTP_ESD_PROTECT
static struct delayed_work gtp_esd_check_work;
static struct workqueue_struct * gtp_esd_check_workqueue = NULL;
static void gtp_esd_check_func(struct work_struct *);
static int gtp_init_ext_watchdog(struct i2c_client *client);
void gtp_esd_switch(struct i2c_client *, int);
#endif

#if GTP_COMPATIBLE_MODE /* For GT9XXF Start */
extern int i2c_read_bytes(struct i2c_client *client, u16 addr, u8 *buf, int len);
extern int i2c_write_bytes(struct i2c_client *client, u16 addr, u8 *buf, int len);
extern int gup_clk_calibration(void);
extern int gup_fw_download_proc(void *dir, u8 dwn_mode);
extern u8 gup_check_fs_mounted(char *path_name);

void gtp_recovery_reset(struct i2c_client *client);
static int gtp_esd_recovery(struct i2c_client *client);
int gtp_fw_startup(struct i2c_client *client);
static int gtp_main_clk_proc(struct goodix_ts_data *ts);
static int gtp_bak_ref_proc(struct goodix_ts_data *ts, u8 mode);
#endif

#if GTP_SLIDE_WAKEUP
typedef enum
{
	DOZE_DISABLED = 0,
	DOZE_ENABLED = 1,
	DOZE_WAKEUP = 2,
}DOZE_T;
static DOZE_T doze_status = DOZE_DISABLED;
static int gtp_enter_doze(struct goodix_ts_data *ts);
#endif

/* true if ic is gt9xxs, like gt915s */
static u8 chip_gt9xxs = 0;

u8 grp_cfg_version = 0;

/**
 * gtp_i2c_read - Read data from the i2c slave device.
 * @client:     i2c device.
 * @buf[0~1]:   read start address.
 * @buf[2~len-1]:   read data buffer.
 * @len:    GTP_ADDR_LENGTH + read bytes count
 */
int gtp_i2c_read(struct i2c_client *client, u8 *buf, int len)
{
	struct i2c_msg msgs[2];
	int ret = -1;
	int retries = 0;

	GTP_DEBUG_FUNC();

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = client->addr;
	msgs[0].len   = GTP_ADDR_LENGTH;
	msgs[0].buf   = &buf[0];
	/* msgs[0].scl_rate = 300 * 1000; */ /* for Rockchip, etc.*/

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = client->addr;
	msgs[1].len   = len - GTP_ADDR_LENGTH;
	msgs[1].buf   = &buf[GTP_ADDR_LENGTH];
	/* msgs[1].scl_rate = 300 * 1000; */

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret == 2)
			break;
		retries++;
	}
	if (retries >= 5) {

#if GTP_COMPATIBLE_MODE
		struct goodix_ts_data *ts = i2c_get_clientdata(client);
#endif

#if GTP_SLIDE_WAKEUP
		/* reset chip would quit doze mode */
		if (DOZE_ENABLED == doze_status)
			return ret;
#endif
		dev_err(&client->dev,
			"I2C Read: 0x%04X, %d bytes failed, errcode: %d! Do reset",
			(((u16)(buf[0] << 8)) | buf[1]), len-2, ret);

#if GTP_COMPATIBLE_MODE
		if (CHIP_TYPE_GT9F == ts->chip_type)
			gtp_recovery_reset(client);
		else
#endif
			gtp_reset_guitar(client, 10);
	}
	return ret;
}

/**
 * gtp_i2c_write - Write data to the i2c slave device.
 * @client:     	i2c device.
 * @buf[0~1]:   	write start address.
 * @buf[2~len-1]:   	data buffer
 * @len:    		GTP_ADDR_LENGTH + write bytes count
 */
int gtp_i2c_write(struct i2c_client *client, u8 *buf, int len)
{
	struct i2c_msg msg;
	int ret = -1;
	int retries = 0;

	GTP_DEBUG_FUNC();

	msg.flags = !I2C_M_RD;
	msg.addr  = client->addr;
	msg.len   = len;
	msg.buf   = buf;
	/*msg.scl_rate = 300 * 1000;*/ /* for Rockchip, etc */

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1)
			break;
		retries++;
	}
	if (retries >= 5) {

#if GTP_COMPATIBLE_MODE
		struct goodix_ts_data *ts = i2c_get_clientdata(client);
#endif

#if GTP_SLIDE_WAKEUP
		if (DOZE_ENABLED == doze_status)
			return ret;
#endif
		dev_err(&client->dev,
			"I2C Write: 0x%04X, %d bytes failed, errcode: %d! Do reset.",
			(((u16)(buf[0] << 8)) | buf[1]), len-2, ret);

#if GTP_COMPATIBLE_MODE
		if (CHIP_TYPE_GT9F == ts->chip_type)
			gtp_recovery_reset(client);
		else
#endif
			gtp_reset_guitar(client, 10);
	}
	return ret;
}

/**
 * gtp_i2c_read_dbl_check - i2c read twice, compare the results
 * @client:  i2c device
 * @addr:    operate address
 * @rxbuf:   read data to store, if compare successful
 * @len:     bytes to read
 */
int gtp_i2c_read_dbl_check(struct i2c_client *client,
					u16 addr, u8 *rxbuf, int len)
{
	u8 buf[16] = {0};
	u8 confirm_buf[16] = {0};
	int retry = 0;

	while (retry++ < 3) {
		memset(buf, 0xAA, 16);
		buf[0] = (u8)(addr >> 8);
		buf[1] = (u8)(addr & 0xFF);
		gtp_i2c_read(client, buf, len + 2);

		memset(confirm_buf, 0xAB, 16);
		confirm_buf[0] = (u8)(addr >> 8);
		confirm_buf[1] = (u8)(addr & 0xFF);
		gtp_i2c_read(client, confirm_buf, len + 2);

		if (!memcmp(buf, confirm_buf, len+2)) {
			memcpy(rxbuf, confirm_buf+2, len);
			return 0;
		}
	}
	dev_err(&client->dev, "I2C read 0x%04X, %d bytes, double check failed!",
			addr, len);
	return -1;
}

int gtp_send_cfg(struct i2c_client *client)
{
	int ret = 0;

#if GTP_DRIVER_SEND_CFG
	int retry = 0;
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	if (ts->fixed_cfg) {
		dev_info(&client->dev, "Ic fixed config, no config sent!");
		return 0;
	}
	if (ts->pnl_init_error) {
		dev_info(&client->dev,
				"Error occured in init_panel, no config sent");
		return 0;
	}

	dev_info(&client->dev, "Driver send config.");
	for (retry = 0; retry < 5; retry++) {
		ret = gtp_i2c_write(client, config,
				GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH);
		if (ret > 0)
			break;
	}
#endif
	return ret;
}

void gtp_irq_disable(struct goodix_ts_data *ts)
{
	unsigned long irqflags;

	GTP_DEBUG_FUNC();

	spin_lock_irqsave(&ts->irq_lock, irqflags);
	if (!ts->irq_is_disable) {
		ts->irq_is_disable = 1;
		disable_irq_nosync(ts->client->irq);
	}
	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

void gtp_irq_enable(struct goodix_ts_data *ts)
{
	unsigned long irqflags = 0;

	GTP_DEBUG_FUNC();

	spin_lock_irqsave(&ts->irq_lock, irqflags);
	if (ts->irq_is_disable) {
		enable_irq(ts->client->irq);
		ts->irq_is_disable = 0;
	}
	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

static void
gtp_touch_down(struct goodix_ts_data* ts, int id, int x, int y, int w)
{
	struct i2c_client *client = ts->client;
#if GTP_CHANGE_X2Y
	GTP_SWAP(x, y);
#endif

#if GTP_ORIENT_INVERSE
	x = ts->abs_x_max - x;
	y = ts->abs_y_max - y;
#endif

#if GTP_ICS_SLOT_REPORT
	input_mt_slot(ts->input_dev, id);
	input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
	input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
#else
	input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
	input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
	input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
	input_mt_sync(ts->input_dev);
#endif
	dev_dbg(&client->dev, "ID:%d, X:%d, Y:%d, W:%d", id, x, y, w);
}

/**
 * gtp_touch_up - Report touch release event
 */
static void gtp_touch_up(struct goodix_ts_data *ts, int id)
{
#if GTP_ICS_SLOT_REPORT
	struct i2c_client *client = ts->client;
	input_mt_slot(ts->input_dev, id);
	input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, -1);
	dev_dbg(&client->dev, "Touch id[%2d] release!", id);
#else
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
	input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
	input_mt_sync(ts->input_dev);
#endif
}

/**
 * goodix_ts_work_func - Goodix touchscreen work function
 */
static void goodix_ts_work_func(struct work_struct *work)
{
	u8 end_cmd[3] = {GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF, 0};
	u8 point_data[2 + 1 + 8 * GTP_MAX_TOUCH + 1] =
		{GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF};
	u8 touch_num = 0;
	u8 finger = 0;
	static u16 pre_touch = 0;
	static u8 pre_key = 0;

#if GTP_WITH_PEN
	static u8 pre_pen = 0;
#endif
	u8  key_value = 0;
	u8* coor_data = NULL;
	int input_x = 0;
	int input_y = 0;
	int input_w = 0;
	int id = 0;
	int i = 0;
	int ret = -1;
	struct goodix_ts_data *ts = NULL;
	struct i2c_client *client;

#if GTP_COMPATIBLE_MODE /* for GT9XXF */
	u8 rqst_buf[3] = {0x80, 0x43};
#endif

#if GTP_SLIDE_WAKEUP
	u8 doze_buf[3] = {0x81, 0x4B};
#endif
	GTP_DEBUG_FUNC();

	ts = container_of(work, struct goodix_ts_data, work);
	client = ts->client;

	if (ts->enter_update) {
		dev_info(&client->dev, "%s: enter update", __func__);
		return;
	}

#if GTP_SLIDE_WAKEUP
	dev_dbg(&client->dev, "%s: GTP_SLIDE_WAKEUP", __func__);
	if (DOZE_ENABLED == doze_status) {
		ret = gtp_i2c_read(i2c_connect_client, doze_buf, 3);
		if (ret < 0) {
			if (ts->use_irq)
				gtp_irq_enable(ts);
			return;
		}
		dev_dbg(&client->dev, "0x814B = 0x%02X", doze_buf[2]);
		if (doze_buf[2] == 0xAA) {
			dev_info(&client->dev,
					"Forward slide to light up the screen!");
			doze_status = DOZE_WAKEUP;
			input_report_key(ts->input_dev, KEY_POWER, 1);
			input_sync(ts->input_dev);
			input_report_key(ts->input_dev, KEY_POWER, 0);
			input_sync(ts->input_dev);
			/* clear 0x814B */
			doze_buf[2] = 0x00;
			gtp_i2c_write(i2c_connect_client, doze_buf, 3);
		} else if (doze_buf[2] == 0xBB) {
			dev_info(&client->dev,
					"Backward slide to light up the screen!");
			doze_status = DOZE_WAKEUP;
			input_report_key(ts->input_dev, KEY_POWER, 1);
			input_sync(ts->input_dev);
			input_report_key(ts->input_dev, KEY_POWER, 0);
			input_sync(ts->input_dev);
			/* clear 0x814B */
			doze_buf[2] = 0x00;
			gtp_i2c_write(i2c_connect_client, doze_buf, 3);
		} else if (0xC0 == (doze_buf[2] & 0xC0)) {
			dev_info(&client->dev,
					"Double click to light up the screen!");
			doze_status = DOZE_WAKEUP;
			input_report_key(ts->input_dev, KEY_POWER, 1);
			input_sync(ts->input_dev);
			input_report_key(ts->input_dev, KEY_POWER, 0);
			input_sync(ts->input_dev);
			/* clear 0x814B */
			doze_buf[2] = 0x00;
			gtp_i2c_write(i2c_connect_client, doze_buf, 3);
		} else {
			gtp_enter_doze(ts);
		}

		if (ts->use_irq)
			gtp_irq_enable(ts);
	}
#endif /* GTP_SLIDE_WAKEUP */

	ret = gtp_i2c_read(ts->client, point_data, 12);
	if (ret < 0) {
		dev_err(&client->dev, "I2C transfer error. errno:%d\n ", ret);
		goto exit_work_func;
	}
	finger = point_data[GTP_ADDR_LENGTH];

#if GTP_COMPATIBLE_MODE
	/* request arrived */
	if ((finger == 0x00) && (CHIP_TYPE_GT9F == ts->chip_type)) {
		ret = gtp_i2c_read(ts->client, rqst_buf, 3);
		if (ret < 0) {
			dev_err(&client->dev, "Read request status error!");
			goto exit_work_func;
		}

		switch (rqst_buf[2] & 0x0F) {
		case GTP_RQST_CONFIG:
			dev_info(&client->dev, "Request for config.");
			ret = gtp_send_cfg(ts->client);
			if (ret < 0) {
				dev_err(&client->dev,
						"Request for config unresponded!");
			} else {
				rqst_buf[2] = GTP_RQST_RESPONDED;
				gtp_i2c_write(ts->client, rqst_buf, 3);
				dev_info(&client->dev,
						"Request for config responded!");
			}
			break;
		case GTP_RQST_BAK_REF:
			dev_info(&client->dev, "Request for backup reference.");
			ret = gtp_bak_ref_proc(ts, GTP_BAK_REF_SEND);
			if (ret < 0) {
				dev_err(&client->dev,
					"Requeset for backup reference unresponed!");
			} else {
				rqst_buf[2] = GTP_RQST_RESPONDED;
				gtp_i2c_write(ts->client, rqst_buf, 3);
				dev_info(&client->dev,
					"Request for backup reference responded!");
			}
			break;
		case GTP_RQST_RESET:
			dev_info(&client->dev, "Request for reset.");
			gtp_recovery_reset(ts->client);
			break;
		case GTP_RQST_MAIN_CLOCK:
			dev_info(&client->dev, "Request for main clock.");
			ts->rqst_processing = 1;
			ret = gtp_main_clk_proc(ts);
			if (ret < 0) {
				dev_err(&client->dev,
					"Request for main clock unresponded!");
			} else {
				dev_info(&client->dev,
					"Request for main clock responded!");
				rqst_buf[2] = GTP_RQST_RESPONDED;
				gtp_i2c_write(ts->client, rqst_buf, 3);
				ts->rqst_processing = 0;
				ts->clk_chk_fs_times = 0;
			}
			break;
		case GTP_RQST_IDLE:
		default:
			break;
		}
	}
#endif /* End of GTP_COMPATIBLE_MODE */

	if ((finger & 0x80) == 0) {
		dev_info(&client->dev, "%s: Buffer status = 0", __func__);
		goto exit_work_func;
	}

	touch_num = finger & 0x0f;
	if (touch_num > GTP_MAX_TOUCH) {
		dev_err(&client->dev,
				"%s: touch number is over max number",
				__func__);
		goto exit_work_func;
	}

	if (touch_num > 1) {
		u8 buf[8 * GTP_MAX_TOUCH] = {(GTP_READ_COOR_ADDR + 10) >> 8,
					(GTP_READ_COOR_ADDR + 10) & 0xff};
		ret = gtp_i2c_read(ts->client, buf, 2 + 8 * (touch_num - 1));
		memcpy(&point_data[12], &buf[2], 8 * (touch_num - 1));
	}

#if GTP_HAVE_TOUCH_KEY
	key_value = point_data[3 + 8 * touch_num];
	if (key_value || pre_key) {
		for (i = 0; i < GTP_MAX_KEY_NUM; i++) {
#if GTP_DEBUG_ON
			for (ret = 0; ret < 4; ++ret) {
				if (key_codes[ret] == touch_key_array[i]) {
					dev_info(&client->dev, "Key: %s %s",
							key_names[ret],
							(key_value & (0x01 << i)) ?
								"Down" : "Up");
					break;
				}
			}
#endif
			input_report_key(ts->input_dev, touch_key_array[i],
							key_value & (0x01<<i));
		}
		touch_num = 0;
		pre_touch = 0;
	}
#endif
	pre_key = key_value;
	dev_dbg(&client->dev, "pre_touch:0x%02x, finger:0x%02x.",
							pre_touch, finger);
#if GTP_ICS_SLOT_REPORT

#if GTP_WITH_PEN
	if (pre_pen && (touch_num == 0)) {
		dev_dbg(&client->dev, "Pen touch UP(Slot)!");
		input_report_key(ts->input_dev, BTN_TOOL_PEN, 0);
		input_mt_slot(ts->input_dev, 5);
		input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, -1);
		pre_pen = 0;
	}
#endif
	if (pre_touch || touch_num) {
		int pos = 0;
		u16 touch_index = 0;
		u8 report_num = 0;
		coor_data = &point_data[3];

		if (touch_num) {
			id = coor_data[pos] & 0x0F;
#if GTP_WITH_PEN
			id = coor_data[pos];
			if ((id & 0x80)) {
				dev_dbg(&client->dev, "Pen touch DOWN(Slot)!");
				input_x  = coor_data[pos + 1] |
						(coor_data[pos + 2] << 8);
				input_y  = coor_data[pos + 3] |
						(coor_data[pos + 4] << 8);
				input_w  = coor_data[pos + 5] |
						(coor_data[pos + 6] << 8);

				input_report_key(ts->input_dev, BTN_TOOL_PEN, 1);
				input_mt_slot(ts->input_dev, 5);
				input_report_abs(ts->input_dev,
						ABS_MT_TRACKING_ID, 5);
				input_report_abs(ts->input_dev,
						ABS_MT_POSITION_X, input_x);
				input_report_abs(ts->input_dev,
						ABS_MT_POSITION_Y, input_y);
				input_report_abs(ts->input_dev,
						ABS_MT_TOUCH_MAJOR, input_w);

				dev_dbg(&client->dev, "Pen/Stylus: (%d, %d)[%d]",
						input_x, input_y, input_w);
				pre_pen = 1;
				pre_touch = 0;
			}
#endif
			touch_index |= (0x01<<id);
		}

		dev_dbg(&client->dev,
			"id = %d,touch_index = 0x%x, pre_touch = 0x%x\n",
						id, touch_index, pre_touch);
		for (i = 0; i < GTP_MAX_TOUCH; i++) {

#if GTP_WITH_PEN
			if (pre_pen == 1)
				break;
#endif
			if ((touch_index & (0x01<<i))) {
				input_x  = coor_data[pos + 1] |
						(coor_data[pos + 2] << 8);
				input_y  = coor_data[pos + 3] |
						(coor_data[pos + 4] << 8);
				input_w  = coor_data[pos + 5] |
						(coor_data[pos + 6] << 8);
				gtp_touch_down(ts, id, input_x, input_y, input_w);
				pre_touch |= 0x01 << i;

				report_num++;
				if (report_num < touch_num) {
					pos += 8;
					id = coor_data[pos] & 0x0F;
					touch_index |= (0x01<<id);
				}
			} else {
				gtp_touch_up(ts, i);
				pre_touch &= ~(0x01 << i);
			}
		}
	}
#else /* No GTP_ICS_SLOT_REPORT */
	input_report_key(ts->input_dev, BTN_TOUCH, (touch_num || key_value));
	if (touch_num) {
		for (i = 0; i < touch_num; i++) {
			coor_data = &point_data[i * 8 + 3];
			id = coor_data[0] & 0x0F;
			input_x  = coor_data[1] | (coor_data[2] << 8);
			input_y  = coor_data[3] | (coor_data[4] << 8);
			input_w  = coor_data[5] | (coor_data[6] << 8);

#if GTP_WITH_PEN
			id = coor_data[0];
			if (id & 0x80) {
				dev_dbg(&client->dev, "Pen touch DOWN!");
				input_report_key(ts->input_dev,
							BTN_TOOL_PEN, 1);
				pre_pen = 1;
				id = 0;
			}
#endif
			gtp_touch_down(ts, id, input_x, input_y, input_w);
		}
	} else if (pre_touch) {

#if GTP_WITH_PEN
		if (pre_pen == 1) {
			dev_dbg(&client->dev, "Pen touch UP!");
			input_report_key(ts->input_dev, BTN_TOOL_PEN, 0);
			pre_pen = 0;
		}
#endif
		dev_dbg(&client->dev, "Touch Release!");
		gtp_touch_up(ts, 0);
	}
	pre_touch = touch_num;
#endif
	input_sync(ts->input_dev);

exit_work_func:
	if(!ts->gtp_rawdiff_mode) {
		ret = gtp_i2c_write(ts->client, end_cmd, 3);
		if (ret < 0)
			dev_info(&client->dev, "I2C write end_cmd error!");
	}
	if (ts->use_irq)
		gtp_irq_enable(ts);
}

/**
 * goodix_ts_timer_handler - Timer interrupt service routine for polling mode.
 */
static enum hrtimer_restart goodix_ts_timer_handler(struct hrtimer *timer)
{
	struct goodix_ts_data *ts =
		container_of(timer, struct goodix_ts_data, timer);

	GTP_DEBUG_FUNC();

	queue_work(goodix_wq, &ts->work);
	hrtimer_start(&ts->timer,
			ktime_set(0, (GTP_POLL_TIME+6)*1000000),
			HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static irqreturn_t goodix_ts_irq_handler(int irq, void *dev_id)
{
	struct goodix_ts_data *ts = dev_id;

	GTP_DEBUG_FUNC();

	gtp_irq_disable(ts);
	queue_work(goodix_wq, &ts->work);

	return IRQ_HANDLED;
}

void gtp_int_sync(int ms)
{
	gpio_direction_output(GTP_INT_PORT, 0);
	msleep(ms);
	gpio_direction_input(GTP_INT_PORT);
}

/**
 * gtp_reset_guitar - Reset chip.
 */
void gtp_reset_guitar(struct i2c_client *client, int ms)
{
#if GTP_COMPATIBLE_MODE
	struct goodix_ts_data *ts = i2c_get_clientdata(client);
#endif

	GTP_DEBUG_FUNC();
	dev_info(&client->dev, "Guitar reset");

	/* begin select I2C slave addr */
	gpio_direction_output(GTP_RST_PORT, 0);
	msleep(ms);                         /* T2: > 10ms */
	/* HIGH: 0x28/0x29, LOW: 0xBA/0xBB */
	gpio_direction_output(GTP_INT_PORT, client->addr == 0x14);

	msleep(2);                          /* T3: > 100us */
	gpio_direction_output(GTP_RST_PORT, 1);

	msleep(6);                          /* T4: > 5ms */

	gpio_direction_input(GTP_RST_PORT); /* end select I2C slave addr */

#if GTP_COMPATIBLE_MODE
	if (CHIP_TYPE_GT9F == ts->chip_type)
		return;
#endif
	gtp_int_sync(50);

#if GTP_ESD_PROTECT
	gtp_init_ext_watchdog(client);
#endif
}

#if GTP_SLIDE_WAKEUP
/**
 * gtp_enter_doze - Enter doze mode for sliding wakeup.
 */
static int gtp_enter_doze(struct goodix_ts_data *ts)
{
	int ret = -1;
	s8 retry = 0;
	u8 i2c_control_buf[3] =
		{(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 8};
	struct i2c_client *client = ts->client;

	GTP_DEBUG_FUNC();

#if GTP_DBL_CLK_WAKEUP
	i2c_control_buf[2] = 0x09;
#endif

	gtp_irq_disable(ts);
	dev_dbg(&client->dev, "Entering doze mode.");
	while (retry++ < 5) {
		i2c_control_buf[0] = 0x80;
		i2c_control_buf[1] = 0x46;
		ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
		if (ret < 0) {
			dev_dbg(&client->dev,
				"failed to set doze flag into 0x8046, %d",
				retry);
			continue;
		}
		i2c_control_buf[0] = 0x80;
		i2c_control_buf[1] = 0x40;
		ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
		if (ret > 0) {
			doze_status = DOZE_ENABLED;
			dev_info(&client->dev, "GTP has been working in doze mode!");
			gtp_irq_enable(ts);
			return ret;
		}
		msleep(10);
	}
	dev_err(&client->dev, "GTP send doze cmd failed");
	gtp_irq_enable(ts);
	return ret;
}
#else  /* No GTP_SLIDE_WAKEUP */

/**
 * gtp_enter_sleep - Enter sleep mode.
 */
static int gtp_enter_sleep(struct goodix_ts_data *ts)
{
	int ret = -1;
	int retry = 0;
	struct i2c_client *client = ts->client;
	u8 i2c_control_buf[3] =
		{(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 5};

#if GTP_COMPATIBLE_MODE
	u8 status_buf[3] = {0x80, 0x44};
#endif

	GTP_DEBUG_FUNC();
#if GTP_COMPATIBLE_MODE
	if (CHIP_TYPE_GT9F == ts->chip_type) {
		/* GT9XXF: host interact with IC */
		ret = gtp_i2c_read(ts->client, status_buf, 3);
		if (ret < 0) {
			dev_err(&client->dev,
				"failed to get backup-reference status");
		}

		if (status_buf[2] & 0x80) {
			ret = gtp_bak_ref_proc(ts, GTP_BAK_REF_STORE);
			if (ret < 0)
				dev_err(&client->dev, "failed to store bak_ref");
		}
	}
#endif

	gpio_direction_output(GTP_INT_PORT, 0);
	msleep(5);

	while (retry++ < 5) {
		ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
		if (ret > 0) {
			dev_info(&client->dev, "GTP enter sleep!");
			return ret;
		}
		msleep(10);
	}

	dev_err(&client->dev, "GTP send sleep cmd failed.");
	return ret;
}
#endif

/**
 * gtp_wakeup_sleep - Wakeup from sleep.
 */
static int gtp_wakeup_sleep(struct goodix_ts_data *ts)
{
	u8 retry = 0;
	s8 ret = -1;
	struct i2c_client *client = ts->client;

	GTP_DEBUG_FUNC();

#if GTP_COMPATIBLE_MODE
	if (CHIP_TYPE_GT9F == ts->chip_type) {
		u8 opr_buf[3] = {0x41, 0x80};

		gpio_direction_output(GTP_INT_PORT, 1);
		msleep(5);

		for (retry = 0; retry < 20; retry++) {
			/* hold ss51 & dsp */
			opr_buf[2] = 0x0C;
			ret = gtp_i2c_write(ts->client, opr_buf, 3);
			if (ret < 0) {
				dev_err(&client->dev, "failed to hold ss51 & dsp!");
				continue;
			}
			opr_buf[2] = 0x00;
			ret = gtp_i2c_read(ts->client, opr_buf, 3);
			if (ret < 0) {
				dev_err(&client->dev,
						"failed to get ss51 & dsp status!");
				continue;
			}
			if (0x0C != opr_buf[2]) {
				dev_dbg(&client->dev, "ss51 & dsp not been hold, %d",
						retry + 1);
				continue;
			}
			dev_dbg(&client->dev, "ss51 & dsp confirmed hold");

			ret = gtp_fw_startup(ts->client);
			if (ret < 0) {
				dev_err(&client->dev,
					"failed to startup GT9XXF, process recovery");
				gtp_esd_recovery(ts->client);
			}
			break;
		}
		if (retry >= 10) {
			dev_err(&client->dev,
				"failed to wakeup, processing esd recovery");
			gtp_esd_recovery(ts->client);
		} else {
			dev_info(&client->dev, "GT9XXF gtp wakeup success");
		}
		return ret;
	}
#endif /* GTP_COMPATIBLE_MODE */

#if GTP_POWER_CTRL_SLEEP
	while (retry++ < 5) {
		gtp_reset_guitar(ts->client, 20);
		dev_info(&client->dev, "GTP wakeup sleep.");
		return 1;
	}
#else
	while (retry++ < 10) {

#if GTP_SLIDE_WAKEUP
		if (DOZE_WAKEUP != doze_status) {
			/* wakeup not by slide */
			dev_dbg(&client->dev, "wakeup by power, reset guitar");
			doze_status = DOZE_DISABLED;
			gtp_irq_disable(ts);
			gtp_reset_guitar(ts->client, 10);
			gtp_irq_enable(ts);
		} else {
			/* wakeup by slide */
			dev_dbg(&client->dev,
				"wakeup by slide/double-click, no reset guitar");
			doze_status = DOZE_DISABLED;

#if GTP_ESD_PROTECT
			gtp_init_ext_watchdog(ts->client);
#endif
		}
#else
		if (chip_gt9xxs == 1) {
			gtp_reset_guitar(ts->client, 10);
		} else {
			gpio_direction_output(GTP_INT_PORT, 1);
			msleep(5);
		}
#endif
		ret = gtp_i2c_test(ts->client);
		if (ret > 0) {
			dev_info(&client->dev, "GTP wakeup sleep.");

#if (!GTP_SLIDE_WAKEUP)

			if (chip_gt9xxs == 0) {
				gtp_int_sync(25);

#if GTP_ESD_PROTECT
				gtp_init_ext_watchdog(ts->client);
#endif
			}
#endif
			return ret;
		}
		gtp_reset_guitar(ts->client, 20);
	}
#endif
	dev_err(&client->dev, "GTP wakeup sleep failed.");
	return ret;
}

#if GTP_DRIVER_SEND_CFG
static int gtp_get_info(struct goodix_ts_data *ts)
{
	u8 opr_buf[6] = {0};
	int ret = 0;
	struct i2c_client *client = ts->client;

	opr_buf[0] = (u8)((GTP_REG_CONFIG_DATA+1) >> 8);
	opr_buf[1] = (u8)((GTP_REG_CONFIG_DATA+1) & 0xFF);

	ret = gtp_i2c_read(ts->client, opr_buf, 6);
	if (ret < 0)
		return ret;

	ts->abs_x_max = (opr_buf[3] << 8) + opr_buf[2];
	ts->abs_y_max = (opr_buf[5] << 8) + opr_buf[4];

	opr_buf[0] = (u8)((GTP_REG_CONFIG_DATA+6) >> 8);
	opr_buf[1] = (u8)((GTP_REG_CONFIG_DATA+6) & 0xFF);

	ret = gtp_i2c_read(ts->client, opr_buf, 3);
	if (ret < 0)
		return ret;
	ts->int_trigger_type = opr_buf[2] & 0x03;

	dev_info(&client->dev, "X_MAX = %d, Y_MAX = %d, TRIGGER = 0x%02x",
			ts->abs_x_max,ts->abs_y_max,ts->int_trigger_type);

	return 0;
}
#endif

static int gtp_init_panel(struct goodix_ts_data *ts)
{
	int ret = -1;
	struct i2c_client *client = ts->client;

#if GTP_DRIVER_SEND_CFG
	int i = 0;
	u8 check_sum = 0;
	u8 opr_buf[16] = {0};
	u8 sensor_id = 0;

	u8 cfg_info_group1[] = CTP_CFG_GROUP1;
	u8 cfg_info_group2[] = CTP_CFG_GROUP2;
	u8 cfg_info_group3[] = CTP_CFG_GROUP3;
	u8 cfg_info_group4[] = CTP_CFG_GROUP4;
	u8 cfg_info_group5[] = CTP_CFG_GROUP5;
	u8 cfg_info_group6[] = CTP_CFG_GROUP6;

	u8 *send_cfg_buf[] = {cfg_info_group1, cfg_info_group2,
				cfg_info_group3, cfg_info_group4,
				cfg_info_group5, cfg_info_group6};
	u8 cfg_info_len[] = { CFG_GROUP_LEN(cfg_info_group1),
				CFG_GROUP_LEN(cfg_info_group2),
				CFG_GROUP_LEN(cfg_info_group3),
				CFG_GROUP_LEN(cfg_info_group4),
				CFG_GROUP_LEN(cfg_info_group5),
				CFG_GROUP_LEN(cfg_info_group6)};

	GTP_DEBUG_FUNC();
	dev_dbg(&client->dev, "Config Groups\' Lengths: %d, %d, %d, %d, %d, %d",
					cfg_info_len[0], cfg_info_len[1],
					cfg_info_len[2], cfg_info_len[3],
					cfg_info_len[4], cfg_info_len[5]);


#if GTP_COMPATIBLE_MODE
	if (CHIP_TYPE_GT9F == ts->chip_type)
		ts->fw_error = 0;
	else
#endif
	{
		ret = gtp_i2c_read_dbl_check(ts->client, 0x41E4, opr_buf, 1);
		if (ret >= 0) {
			if (opr_buf[0] != 0xBE) {
				ts->fw_error = 1;
				dev_err(&client->dev,
						"Firmware error, no config sent!");
				return -1;
			}
		}
	}

	if ((!cfg_info_len[1]) && (!cfg_info_len[2]) &&
			(!cfg_info_len[3]) && (!cfg_info_len[4]) &&
			(!cfg_info_len[5])) {
		sensor_id = 0;
	} else {

#if GTP_COMPATIBLE_MODE
		msleep(50);
#endif
		ret = gtp_i2c_read_dbl_check(ts->client, GTP_REG_SENSOR_ID,
								&sensor_id, 1);
		if (ret < 0) {
			dev_err(&client->dev,
					"Failed to get sensor_id, No config sent!");
			ts->pnl_init_error = 1;
			return -1;
		}
		if (sensor_id >= 0x06) {
			dev_err(&client->dev,
					"Invalid sensor_id(0x%02X), No Config Sent!",
					sensor_id);
			ts->pnl_init_error = 1;

#if GTP_COMPATIBLE_MODE
			if (CHIP_TYPE_GT9F == ts->chip_type)
				return -1;
			else
#endif
			{
				gtp_get_info(ts);
			}
			return 0;
		}
	}
	dev_info(&client->dev, "Sensor_ID: %d", sensor_id);
	ts->gtp_cfg_len = cfg_info_len[sensor_id];
	dev_info(&client->dev, "CTP_CONFIG_GROUP%d used, config length: %d",
					sensor_id + 1, ts->gtp_cfg_len);

	if (ts->gtp_cfg_len < GTP_CONFIG_MIN_LENGTH) {
		dev_err(&client->dev,
				"Config Group%d is INVALID CONFIG GROUP(Len: %d)!",
				sensor_id + 1, ts->gtp_cfg_len);
		dev_err(&client->dev,
			"NO Config Sent! Check the header file CFG_GROUP section!");
		ts->pnl_init_error = 1;
		return -1;
	}

#if GTP_COMPATIBLE_MODE
	if (CHIP_TYPE_GT9F == ts->chip_type)
		ts->fixed_cfg = 0;
	else
#endif
	{
		ret = gtp_i2c_read_dbl_check(ts->client,
				GTP_REG_CONFIG_DATA, &opr_buf[0], 1);
		if (ret < 0) {
			dev_err(&client->dev,
					"Failed to get IC config version!");
			return -1;
		}

		dev_info(&client->dev,
			"CFG_GROUP%d Version:%d,0x%02X; IC Version: %d, 0x%02X",
			sensor_id + 1, send_cfg_buf[sensor_id][0],
			send_cfg_buf[sensor_id][0], opr_buf[0], opr_buf[0]);

		if (opr_buf[0] < 90) {
			/* backup group config version */
			grp_cfg_version = send_cfg_buf[sensor_id][0];
			send_cfg_buf[sensor_id][0] = 0x00;
			ts->fixed_cfg = 0;
		} else {
			/* treated as fixed config, don't send config */
			dev_info(&client->dev,
				"IC fixed config with config version(%d, 0x%02X)",
				opr_buf[0], opr_buf[0]);
			ts->fixed_cfg = 1;
			gtp_get_info(ts);
			return 0;
		}
	}

	memset(&config[GTP_ADDR_LENGTH], 0, GTP_CONFIG_MAX_LENGTH);
	memcpy(&config[GTP_ADDR_LENGTH],
			send_cfg_buf[sensor_id], ts->gtp_cfg_len);

#if GTP_CUSTOM_CFG
	config[RESOLUTION_LOC]     = (u8)GTP_MAX_WIDTH;
	config[RESOLUTION_LOC + 1] = (u8)(GTP_MAX_WIDTH>>8);
	config[RESOLUTION_LOC + 2] = (u8)GTP_MAX_HEIGHT;
	config[RESOLUTION_LOC + 3] = (u8)(GTP_MAX_HEIGHT>>8);

	if (GTP_INT_TRIGGER == 0) {
		/* RISING */
		config[TRIGGER_LOC] &= 0xfe;
	} else if (GTP_INT_TRIGGER == 1) {
		/* FALLING */
		config[TRIGGER_LOC] |= 0x01;
	}
#endif /* GTP_CUSTOM_CFG */

	check_sum = 0;
	for (i = GTP_ADDR_LENGTH; i < ts->gtp_cfg_len; i++)
		check_sum += config[i];
	config[ts->gtp_cfg_len] = (~check_sum) + 1;

#else /* driver doesn't send config */

	ts->gtp_cfg_len = GTP_CONFIG_MAX_LENGTH;
	ret = gtp_i2c_read(ts->client,
			config, ts->gtp_cfg_len + GTP_ADDR_LENGTH);
	if (ret < 0) {
		dev_err(&client->dev,
			"Read Config Failed, use default resolution & INT trigger!");
		ts->abs_x_max = GTP_MAX_WIDTH;
		ts->abs_y_max = GTP_MAX_HEIGHT;
		ts->int_trigger_type = GTP_INT_TRIGGER;
	} else {
		dev_info(&client->dev, "Read Config succeeded!");
		print_hex_dump(KERN_INFO, "Goodix CONFIG:", DUMP_PREFIX_NONE,
				16, 1, config, ts->gtp_cfg_len + GTP_ADDR_LENGTH, false);
	}


#endif /* GTP_DRIVER_SEND_CFG */

	if ((ts->abs_x_max == 0) && (ts->abs_y_max == 0)) {
		ts->abs_x_max = (config[RESOLUTION_LOC + 1] << 8) +
						config[RESOLUTION_LOC];
		ts->abs_y_max = (config[RESOLUTION_LOC + 3] << 8) +
						config[RESOLUTION_LOC + 2];
		ts->int_trigger_type = (config[TRIGGER_LOC]) & 0x03;
	}

#if GTP_COMPATIBLE_MODE
	if (CHIP_TYPE_GT9F == ts->chip_type) {
		u8 sensor_num = 0;
		u8 driver_num = 0;
		u8 have_key = 0;

		have_key = config[GTP_REG_HAVE_KEY -
					GTP_REG_CONFIG_DATA + 2] & 0x01;

		if (1 == ts->is_950) {
			driver_num = config[GTP_REG_MATRIX_DRVNUM -
						GTP_REG_CONFIG_DATA + 2];
			sensor_num = config[GTP_REG_MATRIX_SENNUM -
						GTP_REG_CONFIG_DATA + 2];
			if (have_key)
				driver_num--;
			ts->bak_ref_len =
				(driver_num * (sensor_num - 1) + 2) * 2 * 6;
		} else {
			driver_num = (config[CFG_LOC_DRVA_NUM] & 0x1F) +
						(config[CFG_LOC_DRVB_NUM]&0x1F);
			if (have_key)
				driver_num--;
			sensor_num = (config[CFG_LOC_SENS_NUM] & 0x0F) +
				((config[CFG_LOC_SENS_NUM] >> 4) & 0x0F);
			ts->bak_ref_len =
				(driver_num * (sensor_num - 2) + 2) * 2;
		}

		dev_info(&client->dev,
			"Drv*Sen: %d*%d(key:%d), X_MAX:%d, Y_MAX:%d, TRIGGER:0x%02x",
				driver_num, sensor_num, have_key,
				ts->abs_x_max, ts->abs_y_max,
				ts->int_trigger_type);
		return 0;
	} else
#endif /* GTP_COMPATIBLE_MODE */
	{
#if GTP_DRIVER_SEND_CFG

		ret = gtp_send_cfg(ts->client);
		if (ret < 0)
			dev_err(&client->dev, "Send config error.");
		/* set config version to CTP_CFG_GROUP,
		   for resumimg to send config */
		config[GTP_ADDR_LENGTH] = grp_cfg_version;
		check_sum = 0;
		for (i = GTP_ADDR_LENGTH; i < ts->gtp_cfg_len; i++)
			check_sum += config[i];
		config[ts->gtp_cfg_len] = (~check_sum) + 1;
#endif
		dev_info(&client->dev,
				"X_MAX: %d, Y_MAX: %d, TRIGGER: 0x%02x",
				ts->abs_x_max,
				ts->abs_y_max,
				ts->int_trigger_type);
	}

	msleep(10);
	return 0;
}

int gtp_read_version(struct i2c_client *client, u16 *version)
{
	int ret = -1;
	u8 buf[8] = {GTP_REG_VERSION >> 8, GTP_REG_VERSION & 0xff};

	GTP_DEBUG_FUNC();

	ret = gtp_i2c_read(client, buf, sizeof(buf));
	if (ret < 0) {
		dev_err(&client->dev, "GTP read version failed");
		return ret;
	}

	if (version)
		*version = (buf[7] << 8) | buf[6];

	if (buf[5] == 0x00) {
		dev_info(&client->dev, "IC Version: %c%c%c_%02x%02x",
					buf[2], buf[3], buf[4], buf[7], buf[6]);
	} else {
		if (buf[5] == 'S' || buf[5] == 's')
			chip_gt9xxs = 1;
		dev_info(&client->dev, "IC Version: %c%c%c%c_%02x%02x",
				buf[2], buf[3], buf[4], buf[5], buf[7], buf[6]);
	}
	return ret;
}

static int gtp_i2c_test(struct i2c_client *client)
{
    u8 test[3] = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};
    int retry = 0;
    int ret = -1;

    GTP_DEBUG_FUNC();

    while (retry++ < 5) {
        ret = gtp_i2c_read(client, test, 3);
        if (ret > 0)
            return ret;
        dev_err(&client->dev, "GTP i2c test failed time %d", retry);
        msleep(10);
    }
    return ret;
}

static int gtp_request_io_port(struct goodix_ts_data *ts)
{
	int ret = 0;
	struct i2c_client *client = ts->client;

	GTP_DEBUG_FUNC();

	ret = gpio_request(GTP_INT_PORT, "GTP_INT_IRQ");
	if (ret < 0) {
		dev_err(&client->dev, "Failed to request GPIO:%d, ERRNO:%d",
							GTP_INT_PORT, ret);
		ret = -ENODEV;
	} else {
		gpio_direction_input(GTP_INT_PORT);
		ts->client->irq = gpio_to_irq(GTP_INT_PORT);
		dev_info(&client->dev, "INT gpio %d to irq %d",
					GTP_INT_PORT, ts->client->irq);
	}

	ret = gpio_request(GTP_RST_PORT, "GTP_RST_PORT");
	if (ret < 0) {
		dev_err(&client->dev, "Failed to request GPIO:%d, ERRNO:%d",
							GTP_RST_PORT, ret);
		ret = -ENODEV;
	}

	gpio_direction_input(GTP_RST_PORT);

	gtp_reset_guitar(ts->client, 20);

	if (ret < 0) {
		gpio_free(GTP_RST_PORT);
		gpio_free(GTP_INT_PORT);
	}

	return ret;
}

static int gtp_request_irq(struct goodix_ts_data *ts)
{
	int ret = -1;
	const u8 irq_table[] = GTP_IRQ_TAB;
	struct i2c_client *client = ts->client;

	GTP_DEBUG_FUNC();
	dev_dbg(&client->dev, "INT trigger type:%x", ts->int_trigger_type);

	ret = request_irq(ts->client->irq,
			goodix_ts_irq_handler,
			irq_table[ts->int_trigger_type],
			ts->client->name,
			ts);
	if (ret) {
		dev_err(&client->dev, "Request IRQ failed!ERRNO:%d.", ret);
		gpio_direction_input(GTP_INT_PORT);
		gpio_free(GTP_INT_PORT);

		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = goodix_ts_timer_handler;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		return -1;
	}
	gtp_irq_disable(ts);
	ts->use_irq = 1;

	return 0;
}

static int gtp_request_input_dev(struct goodix_ts_data *ts)
{
	int ret = -1;
	struct i2c_client *client = ts->client;

#if GTP_HAVE_TOUCH_KEY
	u8 index = 0;
#endif
	GTP_DEBUG_FUNC();

	ts->input_dev = input_allocate_device();
	if (!ts->input_dev) {
		dev_err(&client->dev, "Failed to allocate input device");
		return -ENOMEM;
	}

	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) |
					BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;

#if GTP_ICS_SLOT_REPORT
	__set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);
	/* in case of 'out of memory' */
	input_mt_init_slots(ts->input_dev, 16, 0);
#else
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
#endif

#if GTP_HAVE_TOUCH_KEY
	for (index = 0; index < GTP_MAX_KEY_NUM; index++) {
		input_set_capability(ts->input_dev,
				EV_KEY, touch_key_array[index]);
	}
#endif

#if GTP_SLIDE_WAKEUP
	input_set_capability(ts->input_dev, EV_KEY, KEY_POWER);
#endif

#if GTP_WITH_PEN
	/* pen support */
	/*__set_bit(INPUT_PROP_POINTER, ts->input_dev->propbit);*/
	__set_bit(BTN_TOOL_PEN, ts->input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);
#endif

#if GTP_CHANGE_X2Y
	GTP_SWAP(ts->abs_x_max, ts->abs_y_max);
#endif
	input_set_abs_params(ts->input_dev,
				ABS_MT_POSITION_X, 0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev,
				ABS_MT_POSITION_Y, 0, ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);

	ts->input_dev->name = goodix_ts_name;
	ts->input_dev->phys = goodix_ts_phys;
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0xDEAD;
	ts->input_dev->id.product = 0xBEEF;
	ts->input_dev->id.version = 10427;

	ret = input_register_device(ts->input_dev);
	if (ret) {
		dev_err(&client->dev, "Register %s input device failed",
							ts->input_dev->name);
		return -ENODEV;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = goodix_ts_early_suspend;
	ts->early_suspend.resume = goodix_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif
	return 0;
}

#if GTP_COMPATIBLE_MODE /* For GT9XXF Start */
int gtp_fw_startup(struct i2c_client *client)
{
	u8 opr_buf[4];
	int ret = 0;

	/* init sw WDT */
	opr_buf[0] = 0xAA;
	ret = i2c_write_bytes(client, 0x8041, opr_buf, 1);
	if (ret < 0)
		return ret;

	/* release SS51 & DSP */
	opr_buf[0] = 0x00;
	ret = i2c_write_bytes(client, 0x4180, opr_buf, 1);
	if (ret < 0)
		return ret;

	/* int sync */
	gtp_int_sync(25);

	/* check fw run status */
	ret = i2c_read_bytes(client, 0x8041, opr_buf, 1);
	if (ret < 0)
		return ret;
	if (0xAA == opr_buf[0]) {
		dev_err(&client->dev, "IC works abnormally, startup failed");
		return -1;
	}

	dev_info(&client->dev, "IC works normally, Startup success");
	opr_buf[0] = 0xAA;
	i2c_write_bytes(client, 0x8041, opr_buf, 1);

	return 0;
}

static int gtp_esd_recovery(struct i2c_client *client)
{
	int retry = 0;
	int ret = 0;
	struct goodix_ts_data *ts;

	ts = i2c_get_clientdata(client);

	gtp_irq_disable(ts);

	dev_info(&client->dev, "GT9XXF esd recovery mode");
	gtp_reset_guitar(client, 20);	/* reset & select I2C addr */
	for (retry = 0; retry < 5; retry++) {
		ret = gup_fw_download_proc(NULL, GTP_FL_ESD_RECOVERY);
		if (ret < 0) {
			dev_err(&client->dev,
					"esd recovery failed %d", retry + 1);
			continue;
		}
		ret = gtp_fw_startup(ts->client);
		if (ret < 0) {
			dev_err(&client->dev,
					"GT9XXF start up failed %d",retry + 1);
			continue;
		}
		break;
	}
	gtp_irq_enable(ts);

	if (retry >= 5) {
		dev_err(&client->dev, "failed to esd recovery");
		return -1;
	}

	dev_info(&client->dev, "Esd recovery successful");
	return 0;
}

void gtp_recovery_reset(struct i2c_client *client)
{
#if GTP_ESD_PROTECT
	gtp_esd_switch(client, SWITCH_OFF);
#endif
	GTP_DEBUG_FUNC();

	gtp_esd_recovery(client);

#if GTP_ESD_PROTECT
	gtp_esd_switch(client, SWITCH_ON);
#endif
}

static int gtp_bak_ref_proc(struct goodix_ts_data *ts, u8 mode)
{
	int ret = 0;
	int i = 0;
	int j = 0;
	u16 ref_sum = 0;
	u16 learn_cnt = 0;
	u16 chksum = 0;
	int ref_seg_len = 0;
	int ref_grps = 0;
	struct file *ref_filp = NULL;
	u8 *p_bak_ref;
	struct i2c_client *client = ts->client;

	ret = gup_check_fs_mounted("/data");
	if (ret < 0) {
		ts->ref_chk_fs_times++;
		dev_dbg(&client->dev,
				"Ref check /data times/MAX_TIMES: %d / %d",
				ts->ref_chk_fs_times, GTP_CHK_FS_MNT_MAX);
		if (ts->ref_chk_fs_times < GTP_CHK_FS_MNT_MAX) {
			msleep(50);
			dev_info(&client->dev, "/data not mounted.");
			return ret;
		}
		dev_info(&client->dev, "check /data mount timeout...");
	} else {
		dev_info(&client->dev, "/data mounted!!!(%d/%d)",
				ts->ref_chk_fs_times, GTP_CHK_FS_MNT_MAX);
	}

	p_bak_ref = (u8 *)kzalloc(ts->bak_ref_len, GFP_KERNEL);

	if (!p_bak_ref) {
		dev_err(&client->dev, "Allocate memory for p_bak_ref failed!");
		return -1;
	}

	if (ts->is_950) {
		ref_seg_len = ts->bak_ref_len / 6;
		ref_grps = 6;
	} else {
		ref_seg_len = ts->bak_ref_len;
		ref_grps = 1;
	}

	ref_filp = filp_open(GTP_BAK_REF_PATH, O_RDWR | O_CREAT, 0666);
	if (IS_ERR(ref_filp)) {
		dev_info(&client->dev,
				"%s is unavailable, default backup-reference used",
				GTP_BAK_REF_PATH);
		goto bak_ref_default;
	}

	switch (mode) {
	case GTP_BAK_REF_SEND:
		dev_info(&client->dev, "Send backup-reference");
		ref_filp->f_op->llseek(ref_filp, 0, SEEK_SET);
		ret = ref_filp->f_op->read(ref_filp,
				(char*)p_bak_ref, ts->bak_ref_len, &ref_filp->f_pos);
		if (ret < 0) {
			dev_err(&client->dev,
					"failed to read bak_ref info, send default");
			goto bak_ref_default;
		}
		for (j = 0; j < ref_grps; j++) {
			ref_sum = 0;
			for (i = 0; i < (ref_seg_len); i += 2) {
				ref_sum += (p_bak_ref[i + j * ref_seg_len] << 8) +
						p_bak_ref[i+1 + j * ref_seg_len];
			}
			learn_cnt =
				(p_bak_ref[j*ref_seg_len+ref_seg_len-4] << 8) +
				(p_bak_ref[j*ref_seg_len+ref_seg_len-3]);
			chksum = (p_bak_ref[j*ref_seg_len+ref_seg_len-2] << 8) +
				(p_bak_ref[j * ref_seg_len + ref_seg_len -1]);
			dev_dbg(&client->dev, "learn count = %d", learn_cnt);
			dev_dbg(&client->dev, "chksum = %d", chksum);
			dev_dbg(&client->dev, "ref_sum = 0x%04X", ref_sum & 0xFFFF);

			/* Sum(1~ref_seg_len) == 1 */
			if (ref_sum != 1) {
				dev_info(&client->dev,
					"wrong chksum for bak_ref, reset to 0x00");
				memset(&p_bak_ref[j * ref_seg_len], 0, ref_seg_len);
				p_bak_ref[ref_seg_len + j * ref_seg_len - 1] = 0x01;
			} else {
				if (j == (ref_grps - 1)) {
					dev_info(&client->dev,
						"backup-reference data in %s used",
						GTP_BAK_REF_PATH);
				}
			}
		}
		ret = i2c_write_bytes(ts->client,
				GTP_REG_BAK_REF, p_bak_ref, ts->bak_ref_len);
		if (ret < 0) {
			dev_err(&client->dev,
					"Failed to send bak_ref for IIC error");
			filp_close(ref_filp, NULL);
			return ret;
		}
		break;
	case GTP_BAK_REF_STORE:
		dev_info(&client->dev, "Store backup-reference");
		ret = i2c_read_bytes(ts->client,
				GTP_REG_BAK_REF, p_bak_ref, ts->bak_ref_len);
		if (ret < 0) {
			dev_err(&client->dev,
				"Failed to read bak_ref, send default back-reference");
			goto bak_ref_default;
		}
		ref_filp->f_op->llseek(ref_filp, 0, SEEK_SET);
		ref_filp->f_op->write(ref_filp,
				(char*)p_bak_ref,
				ts->bak_ref_len,
				&ref_filp->f_pos);
		break;
	default:
		dev_err(&client->dev, "invalid backup-reference request");
		break;
	}
	filp_close(ref_filp, NULL);
	return 0;

bak_ref_default:
	for (j = 0; j < ref_grps; ++j) {
		memset(&p_bak_ref[j * ref_seg_len], 0, ref_seg_len);
		p_bak_ref[j * ref_seg_len + ref_seg_len - 1] = 0x01;/* checksum = 1*/
	}
	ret = i2c_write_bytes(ts->client, GTP_REG_BAK_REF, p_bak_ref, ts->bak_ref_len);
	if (!IS_ERR(ref_filp)) {
		dev_info(&client->dev, "write backup-reference data into %s",
							GTP_BAK_REF_PATH);
		ref_filp->f_op->llseek(ref_filp, 0, SEEK_SET);
		ref_filp->f_op->write(ref_filp,
					(char*)p_bak_ref,
					ts->bak_ref_len,
					&ref_filp->f_pos);
		filp_close(ref_filp, NULL);
	}
	if (ret < 0) {
		dev_err(&client->dev,
				"Failed to load the default backup reference");
		return ret;
	}
	return 0;
}

static int gtp_verify_main_clk(u8 *p_main_clk)
{
	u8 chksum = 0;
	u8 main_clock = p_main_clk[0];
	int i = 0;

	if (main_clock < 50 || main_clock > 120)
		return -1;

	for (i = 0; i < 5; i++) {
		if (main_clock != p_main_clk[i])
			return -1;
		chksum += p_main_clk[i];
	}
	chksum += p_main_clk[5];
	if (chksum == 0)
		return 0;

	return -1;
}

static int gtp_main_clk_proc(struct goodix_ts_data *ts)
{
	int ret = 0;
	int i = 0;
	int clk_chksum = 0;
	struct file *clk_filp = NULL;
	u8 p_main_clk[6] = {0};
	struct i2c_client *client = ts->client;

	ret = gup_check_fs_mounted("/data");
	if (ret < 0) {
		ts->clk_chk_fs_times++;
		dev_info(&client->dev,
				"Clock check /data times/MAX_TIMES: %d / %d",
				ts->clk_chk_fs_times, GTP_CHK_FS_MNT_MAX);
		if (ts->clk_chk_fs_times < GTP_CHK_FS_MNT_MAX) {
			msleep(50);
			dev_info(&client->dev, "/data not mounted.");
			return ret;
		}
		dev_info(&client->dev, "Check /data mount timeout!");
	} else {
		dev_info(&client->dev, "/data mounted!(%d/%d)",
				ts->clk_chk_fs_times, GTP_CHK_FS_MNT_MAX);
	}

	clk_filp = filp_open(GTP_MAIN_CLK_PATH, O_RDWR | O_CREAT, 0666);
	if (IS_ERR(clk_filp)) {
		dev_err(&client->dev,
				"%s is unavailable, calculate main clock",
				GTP_MAIN_CLK_PATH);
	} else {
		clk_filp->f_op->llseek(clk_filp, 0, SEEK_SET);
		clk_filp->f_op->read(clk_filp,
				(char *)p_main_clk, 6, &clk_filp->f_pos);

		ret = gtp_verify_main_clk(p_main_clk);
		if (ret < 0) {
			/* recalculate main clock & rewrite main clock
			   data to file */
			dev_err(&client->dev, "main clock data in %s is wrong",
				GTP_MAIN_CLK_PATH);
		} else {
			dev_info(&client->dev,
					"main clock data in %s used, freq: %d",
					GTP_MAIN_CLK_PATH, p_main_clk[0]);
			filp_close(clk_filp, NULL);
			goto update_main_clk;
		}
	}

#if GTP_ESD_PROTECT
	gtp_esd_switch(ts->client, SWITCH_OFF);
#endif
	ret = gup_clk_calibration();
	gtp_esd_recovery(ts->client);

#if GTP_ESD_PROTECT
	gtp_esd_switch(ts->client, SWITCH_ON);
#endif

	dev_info(&client->dev, "calibrate main clock: %d", ret);
	if (ret < 50 || ret > 120) {
		dev_err(&client->dev, "wrong main clock: %d", ret);
		goto exit_main_clk;
	}

	/* Sum{0x8020~0x8025} = 0 */
	for (i = 0; i < 5; i++) {
		p_main_clk[i] = ret;
		clk_chksum += p_main_clk[i];
	}
	p_main_clk[5] = 0 - clk_chksum;

	if (!IS_ERR(clk_filp)) {
		dev_dbg(&client->dev, "write main clock data into %s",
							GTP_MAIN_CLK_PATH);
		clk_filp->f_op->llseek(clk_filp, 0, SEEK_SET);
		clk_filp->f_op->write(clk_filp,
				(char *)p_main_clk, 6, &clk_filp->f_pos);
		filp_close(clk_filp, NULL);
	}

update_main_clk:
	ret = i2c_write_bytes(ts->client, GTP_REG_MAIN_CLK, p_main_clk, 6);
	if (ret < 0) {
		dev_err(&client->dev, "update main clock failed!");
		return ret;
	}
	return 0;

exit_main_clk:
	if (!IS_ERR(clk_filp))
		filp_close(clk_filp, NULL);
	return ret;
}

static int gtp_gt9xxf_init(struct i2c_client *client)
{
	int ret = 0;

	ret = gup_fw_download_proc(NULL, GTP_FL_FW_BURN);
	if (ret < 0)
		return ret;

	ret = gtp_fw_startup(client);
	if (ret < 0)
		return ret;

	return 0;
}

static void gtp_get_chip_type(struct goodix_ts_data *ts)
{
	u8 opr_buf[10] = {0x00};
	int ret = 0;
	struct i2c_client *client = ts->client;

	msleep(10);

	ret = gtp_i2c_read_dbl_check(ts->client,
			GTP_REG_CHIP_TYPE, opr_buf, 10);

	if (ret < 0) {
		dev_err(&client->dev, "Fail to get chip type, default: GOODIX_GT9");
		ts->chip_type = CHIP_TYPE_GT9;
		return;
	}

	if (!memcmp(opr_buf, "GOODIX_GT9", 10))
		ts->chip_type = CHIP_TYPE_GT9;
	else
		ts->chip_type = CHIP_TYPE_GT9F;
	dev_info(&client->dev, "Chip Type: %s",
			(ts->chip_type == CHIP_TYPE_GT9) ?
					"GOODIX_GT9" : "GOODIX_GT9F");
}
#endif /* For GT9XXF End */

static int
goodix_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = -1;
	u16 version_info;
	struct goodix_ts_data *ts;

	GTP_DEBUG_FUNC();

	client->addr = 0x14;

	dev_info(&client->dev, "Driver Version: %s", GTP_DRIVER_VERSION);
	dev_info(&client->dev, "GTP I2C Address: 0x%02x", client->addr);

	i2c_connect_client = client;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C check functionality failed.");
		return -ENODEV;
	}
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		dev_err(&client->dev, "Alloc memory failed.");
		return -ENOMEM;
	}

	memset(ts, 0, sizeof(*ts));
	INIT_WORK(&ts->work, goodix_ts_work_func);
	ts->client = client;
	spin_lock_init(&ts->irq_lock);
#if GTP_ESD_PROTECT
	/* HZ: clock ticks in 1 second generated by system */
	ts->clk_tick_cnt = 2 * HZ;
	dev_dbg(&client->dev, "Clock ticks for an esd cycle: %d",
						ts->clk_tick_cnt);
	spin_lock_init(&ts->esd_lock);
#endif
	i2c_set_clientdata(client, ts);

	ts->gtp_rawdiff_mode = 0;

	ret = gtp_request_io_port(ts);
	if (ret < 0) {
		dev_err(&client->dev, "GTP request IO port failed.");
		kfree(ts);
		return ret;
	}

#if GTP_COMPATIBLE_MODE
	gtp_get_chip_type(ts);
	if (CHIP_TYPE_GT9F == ts->chip_type) {
		ret = gtp_gt9xxf_init(ts->client);
		if (ret < 0)
			dev_info(&client->dev, "Failed to init GT9XXF.");
	}
#endif
	ret = gtp_i2c_test(client);
	if (ret < 0)
		dev_err(&client->dev, "I2C communication ERROR!");

	ret = gtp_read_version(client, &version_info);
	if (ret < 0)
		dev_err(&client->dev, "Read version failed.");

	ret = gtp_init_panel(ts);
	if (ret < 0) {
		dev_err(&client->dev, "GTP init panel failed.");
		ts->abs_x_max = GTP_MAX_WIDTH;
		ts->abs_y_max = GTP_MAX_HEIGHT;
		ts->int_trigger_type = GTP_INT_TRIGGER;
	}

#if GTP_AUTO_UPDATE
	ret = gup_init_update_proc(ts);
	if (ret < 0)
		dev_err(&client->dev, "Create update thread error.");
#endif

	ret = gtp_request_input_dev(ts);
	if (ret < 0)
		dev_err(&client->dev, "GTP request input dev failed");

	ret = gtp_request_irq(ts);
	if (ret < 0)
		dev_info(&client->dev, "GTP works in polling mode.");
	else
		dev_info(&client->dev, "GTP works in interrupt mode.");

	if (ts->use_irq)
		gtp_irq_enable(ts);

#if GTP_CREATE_WR_NODE
	init_wr_node(client);
#endif

#if GTP_ESD_PROTECT
	gtp_esd_switch(client, SWITCH_ON);
#endif
	return 0;
}

static int goodix_ts_remove(struct i2c_client *client)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	GTP_DEBUG_FUNC();

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif

#if GTP_CREATE_WR_NODE
	uninit_wr_node();
#endif

#if GTP_ESD_PROTECT
	destroy_workqueue(gtp_esd_check_workqueue);
#endif

	dev_info(&client->dev, "GTP driver removing...");
	i2c_set_clientdata(client, NULL);

	if (ts) {
		if (ts->use_irq) {
			gpio_direction_input(GTP_INT_PORT);
			gpio_free(GTP_INT_PORT);
			free_irq(client->irq, ts);
		} else
			hrtimer_cancel(&ts->timer);

		input_unregister_device(ts->input_dev);
		kfree(ts);
	}

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void goodix_ts_early_suspend(struct early_suspend *h)
{
	int ret = -1;
	struct goodix_ts_data *ts = container_of(h,
			struct goodix_ts_data, early_suspend);
	struct i2c_client *client = ts->client;

	GTP_DEBUG_FUNC();

#if GTP_ESD_PROTECT
	gtp_esd_switch(ts->client, SWITCH_OFF);
#endif
	ts->gtp_is_suspend = 1;

#if GTP_SLIDE_WAKEUP
	ret = gtp_enter_doze(ts);
#else
	if (ts->use_irq)
		gtp_irq_disable(ts);
	else
		hrtimer_cancel(&ts->timer);
	ret = gtp_enter_sleep(ts);
#endif
	if (ret < 0)
		dev_err(&client->dev, "GTP early suspend failed.");
	/* To avoid waking up while is not sleeping,
	   delay 48 + 10ms to ensure reliability
	 */
	msleep(58);
}

static void goodix_ts_late_resume(struct early_suspend *h)
{
	struct goodix_ts_data *ts;
	struct i2c_client *client;
	int ret = -1;

	ts = container_of(h, struct goodix_ts_data, early_suspend);
	client = ts->client;

	GTP_DEBUG_FUNC();

	ret = gtp_wakeup_sleep(ts);

#if GTP_SLIDE_WAKEUP
	doze_status = DOZE_DISABLED;
#endif

	if (ret < 0)
		dev_err(&client->dev, "GTP later resume failed.");
#if GTP_COMPATIBLE_MODE
	if (CHIP_TYPE_GT9F == ts->chip_type)
	{
		/* do nothing */
	}
	else
#endif
		gtp_send_cfg(ts->client);

	if (ts->use_irq)
		gtp_irq_enable(ts);
	else
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);

	ts->gtp_is_suspend = 0;
#if GTP_ESD_PROTECT
	gtp_esd_switch(ts->client, SWITCH_ON);
#endif
}
#endif

#if GTP_ESD_PROTECT
int gtp_i2c_read_no_rst(struct i2c_client *client, u8 *buf, int len)
{
	struct i2c_msg msgs[2];
	int ret=-1;
	int retries = 0;

	GTP_DEBUG_FUNC();

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = client->addr;
	msgs[0].len   = GTP_ADDR_LENGTH;
	msgs[0].buf   = &buf[0];
	/*msgs[0].scl_rate = 300 * 1000; */ /* for Rockchip, etc. */

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = client->addr;
	msgs[1].len   = len - GTP_ADDR_LENGTH;
	msgs[1].buf   = &buf[GTP_ADDR_LENGTH];
	/*msgs[1].scl_rate = 300 * 1000;*/

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret == 2)
			break;
		retries++;
	}
	if (retries >= 5) {
		dev_err(&client->dev,
				"I2C Read: 0x%04X, %d bytes failed, errcode: %d!",
				(((u16)(buf[0] << 8)) | buf[1]), len - 2, ret);
	}
	return ret;
}

static int gtp_i2c_write_no_rst(struct i2c_client *client, u8 *buf, int len)
{
	struct i2c_msg msg;
	int ret = -1;
	int retries = 0;

	GTP_DEBUG_FUNC();

	msg.flags = !I2C_M_RD;
	msg.addr  = client->addr;
	msg.len   = len;
	msg.buf   = buf;

	/* msg.scl_rate = 300 * 1000; */ /* for Rockchip, etc  */

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1)
			break;
		retries++;
	}
	if (retries >= 5) {
		dev_err(&client->dev,
				"I2C Write: 0x%04X, %d bytes failed, errcode: %d!",
				(((u16)(buf[0] << 8)) | buf[1]), len - 2, ret);
	}
	return ret;
}

/**
 *  gtp_esd_switch - switch on or off esd delayed work
 *  @client:  i2c device
 *  @on:      SWITCH_ON / SWITCH_OFF
 */
void gtp_esd_switch(struct i2c_client *client, int on)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	spin_lock(&ts->esd_lock);
	if (SWITCH_ON == on) {
		if (!ts->esd_running) {
			ts->esd_running = 1;
			spin_unlock(&ts->esd_lock);
			dev_info(&client->dev, "Esd started");
			queue_delayed_work(gtp_esd_check_workqueue,
					&gtp_esd_check_work, ts->clk_tick_cnt);
		} else
			spin_unlock(&ts->esd_lock);
	} else {
		if (ts->esd_running) {
			ts->esd_running = 0;
			spin_unlock(&ts->esd_lock);
			dev_info(&client->dev, "Esd cancelled");
			cancel_delayed_work_sync(&gtp_esd_check_work);
		} else
			spin_unlock(&ts->esd_lock);
	}
}

/**
 * gtp_init_ext_watchdog - Initialize external watchdog for esd protect
 */
static int gtp_init_ext_watchdog(struct i2c_client *client)
{
	u8 opr_buffer[3] = {0x80, 0x41, 0xAA};
	dev_dbg(&client->dev, "[Esd]Init external watchdog");
	return gtp_i2c_write_no_rst(client, opr_buffer, 3);
}

/**
 * gtp_esd_check_func - Esd protect function.
 */
static void gtp_esd_check_func(struct work_struct *work)
{
	int i;
	int ret = -1;
	struct i2c_client *client;
	u8 esd_buf[4] = {0x80, 0x40};
	struct goodix_ts_data *ts = NULL;

	GTP_DEBUG_FUNC();

	ts = i2c_get_clientdata(i2c_connect_client);
	client = ts->client;

	if (ts->gtp_is_suspend) {
		dev_info(&client->dev, "Esd suspended!");
		return;
	}

	for (i = 0; i < 3; i++) {
		ret = gtp_i2c_read_no_rst(ts->client, esd_buf, 4);

		dev_dbg(&client->dev, "[Esd]0x8040 = 0x%02X, 0x8041 = 0x%02X",
						esd_buf[2], esd_buf[3]);
		if (ret < 0) {
			/* IIC communication problem */
			continue;
		}

		if (esd_buf[2] == 0xAA || esd_buf[3] != 0xAA) {
			/* IC works abnormally.. */
			u8 chk_buf[4] = {0x80, 0x40};

			gtp_i2c_read_no_rst(ts->client, chk_buf, 4);

			dev_dbg(&client->dev, "0x8040 = 0x%02X, 0x8041 = 0x%02X",
					chk_buf[2], chk_buf[3]);

			if (chk_buf[2] == 0xAA || chk_buf[3] != 0xAA) {
				i = 3;
				break;
			}
			continue;
		}

		/* IC works normally, write 0x8040 0xAA, feed the dog */
		esd_buf[2] = 0xAA;
		gtp_i2c_write_no_rst(ts->client, esd_buf, 3);
		break;
	}

	if (i >= 3) {
#if GTP_COMPATIBLE_MODE
		if (CHIP_TYPE_GT9F == ts->chip_type) {
			if (ts->rqst_processing) {
				dev_info(&client->dev,
						"Request processing, no esd recovery");
			} else {
				dev_err(&client->dev,
						"IC working abnormally! Do esd recovery.");
				gtp_esd_recovery(ts->client);
			}
		} else
#endif
		{
			dev_err(&client->dev,
					"IC working abnormally! Do reset guitar.");
			gtp_reset_guitar(ts->client, 50);
		}
	}

	if (!ts->gtp_is_suspend) {
		queue_delayed_work(gtp_esd_check_workqueue,
				&gtp_esd_check_work, ts->clk_tick_cnt);
	} else
		dev_info(&client->dev, "Esd suspended!");
	return;
}
#endif

static const struct i2c_device_id goodix_ts_id[] = {
	{ GTP_I2C_NAME, 0 },
	{ "GODX0911", 0 },
	{ }
};

static struct acpi_device_id goodix_acpi_match[] = {
#ifdef CONFIG_MRD7
	{ "GODX0911", 0 },
#elif CONFIG_MRD8
	{ "GODX0911", 0 },
#else
	{ "ATML1000", 0 },
#endif
	{ },
};
MODULE_DEVICE_TABLE(acpi, goodix_acpi_match);

static struct i2c_driver goodix_ts_driver = {
	.probe      = goodix_ts_probe,
	.remove     = goodix_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend    = goodix_ts_early_suspend,
	.resume     = goodix_ts_late_resume,
#endif
	.id_table   = goodix_ts_id,
	.driver = {
		.name     = GTP_I2C_NAME,
		.owner    = THIS_MODULE,
		.acpi_match_table = ACPI_PTR(goodix_acpi_match),
	},
};

static int __init goodix_ts_init(void)
{
	int ret;

	GTP_DEBUG_FUNC();

	goodix_wq = create_singlethread_workqueue("goodix_wq");
	if (!goodix_wq) {
		printk("%s: Creat workqueue failed.", __func__);
		return -ENOMEM;
	}

#if GTP_ESD_PROTECT
	INIT_DELAYED_WORK(&gtp_esd_check_work, gtp_esd_check_func);
	gtp_esd_check_workqueue = create_workqueue("gtp_esd_check");
#endif
	ret = i2c_add_driver(&goodix_ts_driver);
	return ret;
}

static void __exit goodix_ts_exit(void)
{
	GTP_DEBUG_FUNC();
	i2c_del_driver(&goodix_ts_driver);
	if (goodix_wq)
		destroy_workqueue(goodix_wq);
}

module_init(goodix_ts_init);
module_exit(goodix_ts_exit);

MODULE_DESCRIPTION("GTP Series Driver");
MODULE_LICENSE("GPL");
