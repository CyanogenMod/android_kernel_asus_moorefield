/*
 * This file is part of the APDS990x sensor driver.
 * Chip is combined proximity and ambient light sensor.
 *
 * Copyright (C) 2010 Nokia Corporation and/or its subsidiary(-ies).
 * Contact: Samu Onkalo <samu.p.onkalo@nokia.com>
 *
 * Copyright (C) 2012 Intel Corporation
 * Contact: Leo Yan <leo.yan@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/poll.h>
#include <linux/earlysuspend.h>
#include <linux/miscdevice.h>
#include <linux/i2c/apds990x.h>
#include <linux/early_suspend_sysfs.h>

/* Register map */
#define APDS990X_ENABLE	 0x00 /* Enable of states and interrupts */
#define APDS990X_ATIME	 0x01 /* ALS ADC time  */
#define APDS990X_PTIME	 0x02 /* Proximity ADC time  */
#define APDS990X_WTIME	 0x03 /* Wait time  */
#define APDS990X_AILTL	 0x04 /* ALS interrupt low threshold low byte */
#define APDS990X_AILTH	 0x05 /* ALS interrupt low threshold hi byte */
#define APDS990X_AIHTL	 0x06 /* ALS interrupt hi threshold low byte */
#define APDS990X_AIHTH	 0x07 /* ALS interrupt hi threshold hi byte */
#define APDS990X_PILTL	 0x08 /* Proximity interrupt low threshold low byte */
#define APDS990X_PILTH	 0x09 /* Proximity interrupt low threshold hi byte */
#define APDS990X_PIHTL	 0x0a /* Proximity interrupt hi threshold low byte */
#define APDS990X_PIHTH	 0x0b /* Proximity interrupt hi threshold hi byte */
#define APDS990X_PERS	 0x0c /* Interrupt persistence filters */
#define APDS990X_CONFIG	 0x0d /* Configuration */
#define APDS990X_PPCOUNT 0x0e /* Proximity pulse count */
#define APDS990X_CONTROL 0x0f /* Gain control register */
#define APDS990X_REV	 0x11 /* Revision Number */
#define APDS990X_ID	 0x12 /* Device ID */
#define APDS990X_STATUS	 0x13 /* Device status */
#define APDS990X_CDATAL	 0x14 /* Clear ADC low data register */
#define APDS990X_CDATAH	 0x15 /* Clear ADC high data register */
#define APDS990X_IRDATAL 0x16 /* IR ADC low data register */
#define APDS990X_IRDATAH 0x17 /* IR ADC high data register */
#define APDS990X_PDATAL	 0x18 /* Proximity ADC low data register */
#define APDS990X_PDATAH	 0x19 /* Proximity ADC high data register */

/* Control */
#define APDS990X_MAX_AGAIN	3

/* Enable register */
#define APDS990X_EN_PIEN	(0x1 << 5)
#define APDS990X_EN_AIEN	(0x1 << 4)
#define APDS990X_EN_WEN		(0x1 << 3)
#define APDS990X_EN_PEN		(0x1 << 2)
#define APDS990X_EN_AEN		(0x1 << 1)
#define APDS990X_EN_PON		(0x1 << 0)
#define APDS990X_EN_DISABLE_ALL 0

/* Status register */
#define APDS990X_ST_PINT	(0x1 << 5)
#define APDS990X_ST_AINT	(0x1 << 4)

/* I2C access types */
#define APDS990x_CMD_TYPE_MASK	(0x03 << 5)
#define APDS990x_CMD_TYPE_RB	(0x00 << 5) /* Repeated byte */
#define APDS990x_CMD_TYPE_INC	(0x01 << 5) /* Auto increment */
#define APDS990x_CMD_TYPE_SPE	(0x03 << 5) /* Special function */

#define APDS990x_ADDR_SHIFT	0
#define APDS990x_CMD		0x80

/* Interrupt ack commands */
#define APDS990X_INT_ACK_ALS	0x6
#define APDS990X_INT_ACK_PS	0x5
#define APDS990X_INT_ACK_BOTH	0x7

/* ptime */
#define APDS990X_PTIME_DEFAULT	0xff /* Recommended conversion time 2.7ms*/

/* wtime */
#define APDS990X_WTIME_DEFAULT	0xee /* ~50ms wait time */

#define APDS990X_TIME_TO_ADC	1024 /* One timetick as ADC count value */

/* Persistence */
#define APDS990X_APERS_SHIFT	0
#define APDS990X_PPERS_SHIFT	4

/* Supported ID:s */
#define APDS990X_ID_0		0x0
#define APDS990X_ID_4		0x4
#define APDS990X_ID_29		0x29
#define APDS993X_ID_30		0x30
#define APDS993X_ID_39		0x39

/* pgain and pdiode settings */
#define APDS_PGAIN_1X	       0x0
#define APDS_PDIODE_IR	       0x2

#define APDS_POWER_DOWN        (0)
#define APDS_POWER_ON          (1)
#define APDS_ALS_ENABLE        (1 << 1)
#define APDS_ALS_DISABLE       (1 << 2)
#define APDS_PS_ENABLE         (1 << 3)
#define APDS_PS_DISABLE        (1 << 4)
#define APDS_GPIO_CHECK_MAX	5

/* alsps_client.status bits */
#define PS_DATA_READY     0
#define PS_IOCTL_ENABLE   1
#define ALS_DATA_READY    2
#define ALS_IOCTL_ENABLE  3

#define APDS_ALS_MAX_LUX	10000
#define APDS_ALS_MIN_ADC	3
#define APDS_ALS_GAIN_MASK	0x3
#define APDS_ALS_WORK_GAIN	0
#define APDS_ALS_INIT_GAIN	2
#define APDS_PS_INIT_DATA	0xffff

/* Reverse chip factors for threshold calculation */
struct reverse_factors {
	u32 afactor;
	int cf1;
	int irf1;
	int cf2;
	int irf2;
};

struct apds990x_chip {
	bool			lux_wait_fresh_res;
	int			ps_cnt;
	int			als_cnt;
	int			gpio;
	unsigned int		alsps_switch;
	struct mutex		mutex; /* avoid parallel access */
	struct list_head	ps_list;
	struct list_head	als_list;
	wait_queue_head_t	ps_workq_head;
	wait_queue_head_t	als_wordq_head;
	struct early_suspend	es;
	struct miscdevice	ps_dev;
	struct miscdevice	als_dev;
	struct i2c_client		*client;
	struct apds990x_platform_data	*pdata;

	/* Chip parameters */
	struct	apds990x_chip_factors	cf;
	struct	reverse_factors		rcf;
	u16	atime;		/* als integration time */
	u16	arate;		/* als reporting rate */
	u16	a_max_result;	/* Max possible ADC value with current atime */
	u8	again_meas;	/* Gain used in last measurement */
	u8	again_next;	/* Next calculated gain */
	u8	pgain;
	u8	pdiode;
	u8	pdrive;
	u8	lux_persistence;
	u8	prox_persistence;

	u32	lux_raw;
	u32	lux;
	u16	lux_clear;
	u16	lux_ir;
	u16	lux_calib;
	u16	lux_thres_hi;
	u16	lux_thres_lo;

	u32	prox_thres;
	u16	prox_data;
	u16	prox_calib;

	char	chipname[10];
	u8      id;
	u8	revision;
};

struct alsps_client {
	unsigned long status;
	struct apds990x_chip *chip;
	struct list_head list;
};

#define APDS_CALIB_SCALER		8192
#define APDS_LUX_NEUTRAL_CALIB_VALUE	(1 * APDS_CALIB_SCALER)
#define APDS_PROX_NEUTRAL_CALIB_VALUE	(1 * APDS_CALIB_SCALER)

#define APDS_PROX_DEF_THRES		600
#define APDS_PROX_HYSTERESIS		30
#define APDS_LUX_DEF_THRES_HI		101
#define APDS_LUX_DEF_THRES_LO		100
#define APDS_DEFAULT_PROX_PERS		0x2

#define APDS_STARTUP_DELAY		25000 /* us */
#define APDS_RANGE			65535
#define APDS_PROX_RANGE			1023
#define APDS_LUX_GAIN_LO_LIMIT		100
#define APDS_LUX_GAIN_LO_LIMIT_STRICT	25

#define TIMESTEP			87 /* 2.7ms is about 87 / 32 */
#define TIME_STEP_SCALER		32

#define APDS_LUX_AVERAGING_TIME		50 /* tolerates 50/60Hz ripple */
#define APDS_LUX_DEFAULT_RATE		5

static const u8 again[]	= {1, 8, 16, 120}; /* ALS gain steps */
static const u8 ir_currents[] = {100, 50, 25, 12}; /* IRled currents in mA */

/* Following two tables must match i.e 10Hz rate means 1 as persistence value */
static const u16 arates_hz[] = {10, 5, 2, 1};
static const u8 apersis[] = {1, 2, 4, 5};

static int apds990x_read_byte(struct apds990x_chip *chip, u8 reg, u8 *data)
{
	struct i2c_client *client = chip->client;
	s32 ret;

	reg &= ~APDS990x_CMD_TYPE_MASK;
	reg |= APDS990x_CMD | APDS990x_CMD_TYPE_RB;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0)
		dev_err(&chip->client->dev, "I2C read 0x%x byte error!",
				reg & ~APDS990x_CMD_TYPE_MASK & ~APDS990x_CMD);
	*data = ret;
	return (int)ret;
}

static int apds990x_read_word(struct apds990x_chip *chip, u8 reg, u16 *data)
{
	struct i2c_client *client = chip->client;
	s32 ret;

	reg &= ~APDS990x_CMD_TYPE_MASK;
	reg |= APDS990x_CMD | APDS990x_CMD_TYPE_INC;

	ret = i2c_smbus_read_word_data(client, reg);
	if (ret < 0)
		dev_err(&chip->client->dev, "I2C read 0x%x word error!",
				reg & ~APDS990x_CMD_TYPE_MASK & ~APDS990x_CMD);
	*data = ret;
	return (int)ret;
}

static int apds990x_write_byte(struct apds990x_chip *chip, u8 reg, u8 data)
{
	struct i2c_client *client = chip->client;
	s32 ret;

	reg &= ~APDS990x_CMD_TYPE_MASK;
	reg |= APDS990x_CMD | APDS990x_CMD_TYPE_RB;

	ret = i2c_smbus_write_byte_data(client, reg, data);
	if (ret < 0)
		dev_err(&chip->client->dev, "I2C write 0x%x byte error!",
				reg & ~APDS990x_CMD_TYPE_MASK & ~APDS990x_CMD);
	return (int)ret;
}

static int apds990x_write_word(struct apds990x_chip *chip, u8 reg, u16 data)
{
	struct i2c_client *client = chip->client;
	s32 ret;

	reg &= ~APDS990x_CMD_TYPE_MASK;
	reg |= APDS990x_CMD | APDS990x_CMD_TYPE_INC;

	ret = i2c_smbus_write_word_data(client, reg, data);
	if (ret < 0)
		dev_err(&chip->client->dev, "I2C write 0x%x word error!",
				reg & ~APDS990x_CMD_TYPE_MASK & ~APDS990x_CMD);
	return (int)ret;
}

static u16 apds990x_lux_to_threshold(struct apds990x_chip *chip, u32 lux)
{
	u32 thres;
	u32 cpl;
	u32 ir;

	if (lux == 0)
		return 0;
	else if (lux == APDS_RANGE)
		return APDS_RANGE;

	/*
	 * Reported LUX value is a combination of the IR and CLEAR channel
	 * values. However, interrupt threshold is only for clear channel.
	 * This function approximates needed HW threshold value for a given
	 * LUX value in the current lightning type.
	 * IR level compared to visible light varies heavily depending on the
	 * source of the light
	 *
	 * Calculate threshold value for the next measurement period.
	 * Math: threshold = lux * cpl where
	 * cpl = atime * again / (glass_attenuation * device_factor)
	 * (count-per-lux)
	 *
	 * First remove calibration. Division by four is to avoid overflow
	 */
	lux = lux * (APDS_CALIB_SCALER / 4) / (chip->lux_calib / 4);

	/* Multiplication by 64 is to increase accuracy */
	cpl = ((u32)chip->atime * (u32)again[chip->again_next] *
		APDS_PARAM_SCALE * 64) / (chip->cf.ga * chip->cf.df);

	thres = lux * cpl / 64;
	/*
	 * Convert IR light from the latest result to match with
	 * new gain step. This helps to adapt with the current
	 * source of light.
	 */
	ir = (u32)chip->lux_ir * (u32)again[chip->again_next] /
		(u32)again[chip->again_meas];

	/*
	 * Compensate count with IR light impact
	 * IAC1 > IAC2 (see apds990x_get_lux for formulas)
	 */
	if (chip->lux_clear * APDS_PARAM_SCALE >=
		chip->rcf.afactor * chip->lux_ir)
		thres = (chip->rcf.cf1 * thres + chip->rcf.irf1 * ir) /
			APDS_PARAM_SCALE;
	else
		thres = (chip->rcf.cf2 * thres + chip->rcf.irf2 * ir) /
			APDS_PARAM_SCALE;

	if (thres >= chip->a_max_result)
		thres = chip->a_max_result - 1;
	return thres;
}

static inline int apds990x_set_atime(struct apds990x_chip *chip, u32 time_ms)
{
	u8 reg_value;

	chip->atime = time_ms;
	/* Formula is specified in the data sheet */
	reg_value = 256 - ((time_ms * TIME_STEP_SCALER) / TIMESTEP);
	/* Calculate max ADC value for given integration time */
	chip->a_max_result = (u16)(256 - reg_value) * APDS990X_TIME_TO_ADC;
	dev_info(&chip->client->dev,
			"max ADC value = %d\n", chip->a_max_result);
	return apds990x_write_byte(chip, APDS990X_ATIME, reg_value);
}

/* Called always with mutex locked */
static int apds990x_refresh_pthres(struct apds990x_chip *chip, int data)
{
	int ret, lo, hi;

	if (data < chip->prox_thres) {
		lo = 0;
		hi = chip->prox_thres;
	} else {
		lo = chip->prox_thres - APDS_PROX_HYSTERESIS;
		hi = APDS_RANGE;
	}

	ret = apds990x_write_word(chip, APDS990X_PILTL, lo);
	ret |= apds990x_write_word(chip, APDS990X_PIHTL, hi);
	return ret;
}

/* Called always with mutex locked */
static void apds990x_clear_to_athres(struct apds990x_chip *chip)
{
	u16 lo, hi;

	/* The data register may float in very bright environment which causes
	 * a lot of meaningless interrupt. To avoid that and reduce power
	 * consumption, set interrupt trigger condition as 2% change of current
	 * ADC value
	 */
	if (chip->lux_clear < APDS_ALS_MIN_ADC) {
		lo = 0;
		hi = APDS_ALS_MIN_ADC;
	} else {
		lo = chip->lux_clear * 98 / 100;
		if (lo >= chip->a_max_result)
			lo = chip->a_max_result * 98 / 100;

		hi = chip->lux_clear * 102 / 100;
		if (hi == chip->lux_clear)
			hi += 1;
		else if (hi >= chip->a_max_result)
			hi = chip->a_max_result - 1;
	}

	chip->lux_thres_hi = hi;
	chip->lux_thres_lo = lo;
}

/* Called always with mutex locked */
static int apds990x_refresh_athres(struct apds990x_chip *chip)
{
	int ret;

	ret = apds990x_write_word(chip, APDS990X_AILTL, chip->lux_thres_lo);
	ret |= apds990x_write_word(chip, APDS990X_AIHTL, chip->lux_thres_hi);

	dev_dbg(&chip->client->dev, "als threshold: %d, %d",
			chip->lux_thres_lo, chip->lux_thres_hi);

	return ret;
}

/* Called always with mutex locked */
static void apds990x_force_a_refresh(struct apds990x_chip *chip)
{
	/* This will force ALS interrupt after the next measurement. */
	apds990x_write_word(chip, APDS990X_AILTL, APDS_LUX_DEF_THRES_LO);
	apds990x_write_word(chip, APDS990X_AIHTL, APDS_LUX_DEF_THRES_HI);
}

/* Called always with mutex locked */
static void apds990x_force_p_refresh(struct apds990x_chip *chip)
{
	/* This will force proximity interrupt after the next measurement. */
	apds990x_write_word(chip, APDS990X_PILTL, APDS_PROX_DEF_THRES - 1);
	apds990x_write_word(chip, APDS990X_PIHTL, APDS_PROX_DEF_THRES);
}

/* Called always with mutex locked */
static int apds990x_calc_again(struct apds990x_chip *chip)
{
	int curr_again = chip->again_meas;
	int next_again = chip->again_meas;
	int ret = 0;

	/* Calculate suitable als gain */
	if (chip->lux_clear == chip->a_max_result)
		next_again -= 2; /* ALS saturated. Decrease gain by 2 steps */
	else if (chip->lux_clear > chip->a_max_result / 2)
		next_again--;
	else if (chip->lux_clear < APDS_LUX_GAIN_LO_LIMIT_STRICT)
		next_again += 2; /* Too dark. Increase gain by 2 steps */
	else if (chip->lux_clear < APDS_LUX_GAIN_LO_LIMIT)
		next_again++;

	/* Limit gain to available range */
	if (next_again < 0)
		next_again = 0;
	else if (next_again > APDS990X_MAX_AGAIN)
		next_again = APDS990X_MAX_AGAIN;

	/* Let's check can we trust the measured result */
	if (chip->lux_clear == chip->a_max_result)
		/* Result can be totally garbage due to saturation */
		ret = -ERANGE;
	else if (next_again != curr_again &&
		chip->lux_clear < APDS_LUX_GAIN_LO_LIMIT_STRICT)
		/*
		 * Gain is changed and measurement result is very small.
		 * Result can be totally garbage due to underflow
		 */
		ret = -ERANGE;

	chip->again_next = next_again;
	apds990x_write_byte(chip, APDS990X_CONTROL,
			(chip->pdrive << 6) |
			(chip->pdiode << 4) |
			(chip->pgain << 2) |
			(chip->again_next << 0));

	/*
	 * Error means bad result -> re-measurement is needed. The forced
	 * refresh uses fastest possible persistence setting to get result
	 * as soon as possible.
	 */
	if (ret < 0)
		apds990x_force_a_refresh(chip);
	else {
		apds990x_clear_to_athres(chip);
		apds990x_refresh_athres(chip);
	}

	return ret;
}

/* Called always with mutex locked */
static int apds990x_get_lux(struct apds990x_chip *chip, int clear, int ir)
{
	int ret;
	u32 lpc; /* Lux per count */
	int ratio, cf, irf, iac;

	if (clear < APDS_ALS_MIN_ADC)
		return 0;

	cf = chip->cf.cf1;
	irf = chip->cf.irf1;

	ratio = (ir * APDS_PARAM_SCALE) / clear;
	if (ratio > chip->cf.incan && ir > chip->cf.min_ir) {
		cf = chip->cf.cf2;
		irf = chip->cf.irf2;
	}

	iac = (cf * clear - irf * ir) / APDS_PARAM_SCALE;
	if (iac < 0) {
		dev_warn(&chip->client->dev, "%s: negative value! clear: %d infrared: %d\n",
			 __func__, clear, ir);
		iac = 0;
	}
	lpc = (chip->cf.df * chip->cf.ga) /
		(u32)(again[chip->again_meas] * (u32)chip->atime);
	ret = (iac * lpc) / APDS_PARAM_SCALE;

	dev_dbg(&chip->client->dev, "clear=%d,ir=%d,iac=%d,lpc=%u,ret=%d\n",
					clear, ir, iac, lpc, ret);
	return min(ret, APDS_ALS_MAX_LUX);
}

static int apds990x_ack_int(struct apds990x_chip *chip, u8 mode)
{
	struct i2c_client *client = chip->client;
	s32 ret;
	u8 reg = APDS990x_CMD | APDS990x_CMD_TYPE_SPE;

	switch (mode & (APDS990X_ST_AINT | APDS990X_ST_PINT)) {
	case APDS990X_ST_AINT:
		reg |= APDS990X_INT_ACK_ALS;
		break;
	case APDS990X_ST_PINT:
		reg |= APDS990X_INT_ACK_PS;
		break;
	default:
		reg |= APDS990X_INT_ACK_BOTH;
		break;
	}

	ret = i2c_smbus_read_byte_data(client, reg);
	return (int)ret;
}

/* mutex must be held when calling this function */
static int als_update_gain(struct apds990x_chip *chip, u8 gain)
{
	u8 ctrl;
	int ret;

	if (chip->again_meas == gain)
		return 0;
	ret = apds990x_read_byte(chip, APDS990X_CONTROL, &ctrl);
	if (ret < 0) {
		dev_err(&chip->client->dev,
			"%s: read control register failed!\n", __func__);
		return ret;
	}
	ctrl &= ~APDS_ALS_GAIN_MASK;
	ctrl |= (gain & APDS_ALS_GAIN_MASK);
	ret = apds990x_write_byte(chip, APDS990X_CONTROL, ctrl);
	if (ret < 0) {
		dev_err(&chip->client->dev,
			"%s: write control register failed!\n", __func__);
		return ret;
	}
	chip->again_meas = gain;

	return 0;
}

/* mutex must be held when calling this function */
static void als_handle_irq(struct apds990x_chip *chip)
{
	struct alsps_client *client;

	apds990x_read_word(chip, APDS990X_CDATAL, &chip->lux_clear);
	apds990x_read_word(chip, APDS990X_IRDATAL, &chip->lux_ir);

	if (chip->again_meas != APDS_ALS_WORK_GAIN) {
		dev_info(&chip->client->dev, "again = %d\n", chip->again_meas);
		chip->lux_wait_fresh_res = true;
		als_update_gain(chip, APDS_ALS_WORK_GAIN);
		return;
	}
	chip->lux_raw = apds990x_get_lux(chip, chip->lux_clear, chip->lux_ir);
	apds990x_clear_to_athres(chip);
	apds990x_refresh_athres(chip);

	if (chip->lux != chip->lux_raw || chip->lux_wait_fresh_res == true) {
		chip->lux_wait_fresh_res = false;
		chip->lux = chip->lux_raw;

		list_for_each_entry(client, &chip->als_list, list)
			set_bit(ALS_DATA_READY, &client->status);

		wake_up(&chip->als_wordq_head);
	}
}

/* mutex must be held when calling this function */
static void ps_handle_irq(struct apds990x_chip *chip)
{
	struct alsps_client *client;
	u16 clr_ch;

	apds990x_read_word(chip, APDS990X_CDATAL, &clr_ch);

	/*
	 * If ALS channel is saturated at min gain,
	 * proximity gives false posivite values.
	 * Just ignore them.
	 */
	if (chip->again_meas == 0 && clr_ch == chip->a_max_result)
		chip->prox_data = 0;
	else
		apds990x_read_word(chip, APDS990X_PDATAL, &chip->prox_data);

	apds990x_refresh_pthres(chip, chip->prox_data);

	dev_info(&chip->client->dev, "clr_ch=%u, proximity=%u, thresh=%u\n",
			clr_ch, chip->prox_data, chip->prox_thres);

	list_for_each_entry(client, &chip->ps_list, list)
		set_bit(PS_DATA_READY, &client->status);

	wake_up(&chip->ps_workq_head);
}

static int apds990x_configure(struct apds990x_chip *chip)
{
	/* It is recommended to use disabled mode during these operations */
	apds990x_write_byte(chip, APDS990X_ENABLE, APDS990X_EN_DISABLE_ALL);

	/* conversion and wait times for different state machince states */
	apds990x_write_byte(chip, APDS990X_PTIME, APDS990X_PTIME_DEFAULT);
	apds990x_write_byte(chip, APDS990X_WTIME, APDS990X_WTIME_DEFAULT);
	apds990x_set_atime(chip, APDS_LUX_AVERAGING_TIME);

	apds990x_write_byte(chip, APDS990X_CONFIG, 0);

	/* Persistence levels */
	apds990x_write_byte(chip, APDS990X_PERS,
			(chip->lux_persistence << APDS990X_APERS_SHIFT) |
			(chip->prox_persistence << APDS990X_PPERS_SHIFT));

	apds990x_write_byte(chip, APDS990X_PPCOUNT, chip->pdata->ppcount);

	/* Start with relatively large gain to improve sensitivity */
	chip->again_meas = APDS_ALS_INIT_GAIN;
	chip->again_next = APDS_ALS_INIT_GAIN;
	apds990x_write_byte(chip, APDS990X_CONTROL,
			(chip->pdrive << 6) |
			(chip->pdiode << 4) |
			(chip->pgain << 2) |
			(chip->again_next << 0));
	return 0;
}

static int apds990x_detect(struct apds990x_chip *chip)
{
	struct i2c_client *client = chip->client;
	int ret;

	ret = apds990x_read_byte(chip, APDS990X_ID, &chip->id);
	if (ret < 0) {
		dev_err(&client->dev, "ID read failed\n");
		return ret;
	}

	ret = apds990x_read_byte(chip, APDS990X_REV, &chip->revision);
	if (ret < 0) {
		dev_err(&client->dev, "REV read failed\n");
		return ret;
	}

	switch (chip->id) {
	case APDS990X_ID_0:
	case APDS990X_ID_4:
	case APDS990X_ID_29:
		snprintf(chip->chipname, sizeof(chip->chipname), "APDS-990x");
		break;
	case APDS993X_ID_30:
	case APDS993X_ID_39:
		snprintf(chip->chipname, sizeof(chip->chipname), "APDS-993x");
		break;
	default:
		ret = -ENODEV;
		break;
	}
	return ret;
}

static ssize_t apds990x_lux_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct apds990x_chip *chip = dev_get_drvdata(dev);
	ssize_t ret;
	u32 result;

	mutex_lock(&chip->mutex);
	result = (chip->lux * chip->lux_calib) / APDS_CALIB_SCALER;
	if (result > APDS_ALS_MAX_LUX)
		result = APDS_ALS_MAX_LUX;

	ret = sprintf(buf, "%d\n", result);
	mutex_unlock(&chip->mutex);
	return ret;
}

static DEVICE_ATTR(lux0_input, S_IRUGO, apds990x_lux_show, NULL);

static ssize_t apds990x_lux_calib_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct apds990x_chip *chip = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", chip->lux_calib);
}

static ssize_t apds990x_lux_calib_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len)
{
	struct apds990x_chip *chip = dev_get_drvdata(dev);
	unsigned long value;

	if (strict_strtoul(buf, 0, &value))
		return -EINVAL;

	if (chip->lux_calib > APDS_RANGE)
		return -EINVAL;

	chip->lux_calib = value;

	return len;
}

static DEVICE_ATTR(lux0_calibscale, S_IRUGO | S_IWUSR, apds990x_lux_calib_show,
		apds990x_lux_calib_store);

static ssize_t apds990x_rate_avail(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int i;
	int pos = 0;
	for (i = 0; i < ARRAY_SIZE(arates_hz); i++)
		pos += sprintf(buf + pos, "%d ", arates_hz[i]);
	sprintf(buf + pos - 1, "\n");
	return pos;
}

static ssize_t apds990x_rate_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct apds990x_chip *chip =  dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", chip->arate);
}

static int apds990x_set_arate(struct apds990x_chip *chip, int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(arates_hz); i++)
		if (rate >= arates_hz[i])
			break;

	if (i == ARRAY_SIZE(arates_hz))
		return -EINVAL;

	/* Pick up corresponding persistence value */
	chip->lux_persistence = apersis[i];
	chip->arate = arates_hz[i];

	/* Persistence levels */
	return apds990x_write_byte(chip, APDS990X_PERS,
			(chip->lux_persistence << APDS990X_APERS_SHIFT) |
			(chip->prox_persistence << APDS990X_PPERS_SHIFT));
}

static ssize_t apds990x_rate_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len)
{
	struct apds990x_chip *chip =  dev_get_drvdata(dev);
	unsigned long value;
	int ret;

	if (strict_strtoul(buf, 0, &value))
		return -EINVAL;

	mutex_lock(&chip->mutex);
	ret = apds990x_set_arate(chip, value);
	mutex_unlock(&chip->mutex);

	if (ret < 0)
		return ret;
	return len;
}

static DEVICE_ATTR(lux0_rate_avail, S_IRUGO, apds990x_rate_avail, NULL);
static DEVICE_ATTR(lux0_rate, S_IRUGO | S_IWUSR, apds990x_rate_show,
						 apds990x_rate_store);

static ssize_t apds990x_prox_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	ssize_t ret;
	struct apds990x_chip *chip =  dev_get_drvdata(dev);

	mutex_lock(&chip->mutex);
	if (!chip->ps_cnt) {
		ret = -EIO;
		goto out;
	}
	/* If the hardware is not ready, report the init data directly*/
	if (chip->prox_data != APDS_PS_INIT_DATA)
		apds990x_read_word(chip, APDS990X_PDATAL, &chip->prox_data);
	ret = sprintf(buf, "%d\n", chip->prox_data);
out:
	mutex_unlock(&chip->mutex);
	return ret;
}

static DEVICE_ATTR(prox0_raw, S_IRUGO, apds990x_prox_show, NULL);

static ssize_t apds990x_lux_thresh_above_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct apds990x_chip *chip =  dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", chip->lux_thres_hi);
}

static ssize_t apds990x_lux_thresh_below_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct apds990x_chip *chip =  dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", chip->lux_thres_lo);
}

static ssize_t apds990x_set_lux_thresh(struct apds990x_chip *chip, u16 *target,
				const char *buf)
{
	int ret = 0;
	unsigned long thresh;

	if (strict_strtoul(buf, 0, &thresh))
		return -EINVAL;

	if (thresh > APDS_RANGE)
		return -EINVAL;

	mutex_lock(&chip->mutex);
	*target = (u16)thresh;
	/*
	 * Don't update values in HW if we are still waiting for
	 * first interrupt to come after device handle open call.
	 */
	if (!chip->lux_wait_fresh_res)
		apds990x_refresh_athres(chip);
	mutex_unlock(&chip->mutex);
	return ret;

}

static ssize_t apds990x_lux_thresh_above_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len)
{
	struct apds990x_chip *chip =  dev_get_drvdata(dev);
	int ret = apds990x_set_lux_thresh(chip, &chip->lux_thres_hi, buf);
	if (ret < 0)
		return ret;
	return len;
}

static ssize_t apds990x_lux_thresh_below_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len)
{
	struct apds990x_chip *chip =  dev_get_drvdata(dev);
	int ret = apds990x_set_lux_thresh(chip, &chip->lux_thres_lo, buf);
	if (ret < 0)
		return ret;
	return len;
}

static DEVICE_ATTR(lux0_thresh_above_value, S_IRUGO | S_IWUSR,
		apds990x_lux_thresh_above_show,
		apds990x_lux_thresh_above_store);

static DEVICE_ATTR(lux0_thresh_below_value, S_IRUGO | S_IWUSR,
		apds990x_lux_thresh_below_show,
		apds990x_lux_thresh_below_store);

static ssize_t apds990x_prox_threshold_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct apds990x_chip *chip =  dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", chip->prox_thres);
}

static ssize_t apds990x_prox_threshold_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len)
{
	struct apds990x_chip *chip =  dev_get_drvdata(dev);
	unsigned long value;

	if (strict_strtoul(buf, 0, &value))
		return -EINVAL;

	if ((value > APDS_RANGE) || (value == 0) ||
		(value < APDS_PROX_HYSTERESIS))
		return -EINVAL;

	mutex_lock(&chip->mutex);
	chip->prox_thres = value;

	apds990x_force_p_refresh(chip);
	mutex_unlock(&chip->mutex);
	return len;
}

static DEVICE_ATTR(prox0_thresh_above_value, S_IRUGO | S_IWUSR,
		apds990x_prox_threshold_show,
		apds990x_prox_threshold_store);

static ssize_t apds990x_chip_id_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct apds990x_chip *chip =  dev_get_drvdata(dev);
	return sprintf(buf, "%s %d\n", chip->chipname, chip->revision);
}

static DEVICE_ATTR(chip_id, S_IRUGO, apds990x_chip_id_show, NULL);

static struct attribute *sysfs_attrs_ctrl[] = {
	&dev_attr_lux0_calibscale.attr,
	&dev_attr_lux0_input.attr,
	&dev_attr_lux0_rate.attr,
	&dev_attr_lux0_rate_avail.attr,
	&dev_attr_lux0_thresh_above_value.attr,
	&dev_attr_lux0_thresh_below_value.attr,
	&dev_attr_prox0_raw.attr,
	&dev_attr_prox0_thresh_above_value.attr,
	&dev_attr_chip_id.attr,
	NULL
};

static struct attribute_group apds990x_attribute_group[] = {
	{.attrs = sysfs_attrs_ctrl },
};

static ssize_t ps_read(struct file *filep,
			char __user *buffer, size_t size, loff_t *offset)
{
	int ret = -ENODEV;
	int value;
	struct alsps_client *client = filep->private_data;
	struct apds990x_chip *chip = client->chip;

	mutex_lock(&chip->mutex);
	if (chip->alsps_switch & APDS_PS_ENABLE) {
		value = chip->prox_data < chip->prox_thres ? 0 : 1;
		ret = sizeof(value);
		clear_bit(PS_DATA_READY, &client->status);
		if (copy_to_user(buffer, &value, sizeof(value)))
			ret = -EFAULT;
	}
	mutex_unlock(&chip->mutex);

	return ret;
}

static unsigned int ps_poll(struct file *filep, struct poll_table_struct *wait)
{
	unsigned int mask = 0;
	struct alsps_client *client = filep->private_data;
	struct apds990x_chip *chip = client->chip;

	poll_wait(filep, &chip->ps_workq_head, wait);

	if (test_bit(PS_DATA_READY, &client->status))
		mask |= (POLLIN | POLLRDNORM);

	return mask;
}

static int ps_open(struct inode *inode, struct file *filep)
{
	struct apds990x_chip *chip =
		container_of(filep->private_data, struct apds990x_chip, ps_dev);
	struct alsps_client *client;

	client = kzalloc(sizeof(struct alsps_client), GFP_KERNEL);
	if (client == NULL) {
		dev_dbg(&chip->client->dev,
				"proximity open kzalloc failed!\n");
		return -ENOMEM;
	}
	client->chip = chip;

	filep->private_data = client;
	mutex_lock(&chip->mutex);
	list_add(&client->list, &chip->ps_list);
	mutex_unlock(&chip->mutex);

	return 0;
}

static ssize_t
als_read(struct file *filep, char __user *buffer, size_t size, loff_t *offset)
{
	u32 lux;
	int ret = -ENODEV;
	struct alsps_client *client = filep->private_data;
	struct apds990x_chip *chip = client->chip;

	mutex_lock(&chip->mutex);
	if (chip->alsps_switch & APDS_ALS_ENABLE) {
		lux = min(chip->lux, APDS_ALS_MAX_LUX);
		clear_bit(ALS_DATA_READY, &client->status);
		if (copy_to_user(buffer, &lux, sizeof(lux)))
			ret = -EFAULT;
		ret = sizeof(lux);
	}
	mutex_unlock(&chip->mutex);

	return ret;
}

static unsigned int
als_poll(struct file *filep, struct poll_table_struct *wait)
{
	unsigned int mask = 0;
	struct alsps_client *client = filep->private_data;
	struct apds990x_chip *chip = client->chip;

	poll_wait(filep, &chip->als_wordq_head, wait);

	if (test_bit(ALS_DATA_READY, &client->status))
		mask |= (POLLIN | POLLRDNORM);

	return mask;
}

static int als_open(struct inode *inode, struct file *filep)
{
	struct apds990x_chip *chip =
		container_of(filep->private_data,
				struct apds990x_chip, als_dev);
	struct alsps_client *client;

	client = kzalloc(sizeof(struct alsps_client), GFP_KERNEL);
	if (client == NULL) {
		dev_dbg(&chip->client->dev, "ALS open kzalloc failed!\n");
		return -ENOMEM;
	}
	client->chip = chip;

	filep->private_data = client;
	mutex_lock(&chip->mutex);
	list_add(&client->list, &chip->als_list);
	mutex_unlock(&chip->mutex);

	return 0;
}

static int apds990x_switch(struct apds990x_chip *chip, int mode)
{
	int ret = 0;
	u8 reg = 0;
	u8 data = APDS990X_EN_PON | APDS990X_EN_WEN;

	switch (mode) {
	case APDS_POWER_ON:
		break;
	case APDS_ALS_ENABLE | APDS_PS_ENABLE:
		data |= APDS990X_EN_AEN | APDS990X_EN_AIEN |
			APDS990X_EN_PEN | APDS990X_EN_PIEN;
		break;
	case APDS_PS_ENABLE:
		data |= APDS990X_EN_PIEN | APDS990X_EN_PEN;
		break;
	case APDS_ALS_ENABLE:
		data |= APDS990X_EN_AIEN | APDS990X_EN_AEN;
		break;
	case APDS_POWER_DOWN:
		data = APDS990X_EN_DISABLE_ALL;
		break;
	default:
		dev_err(&chip->client->dev, "apds990x switch error\n");
		return -1;
	}
	ret = apds990x_read_byte(chip, APDS990X_ENABLE, &reg);
	if (ret < 0) {
		dev_err(&chip->client->dev,
			"%s: APDS990X_ENABLE read failed\n", __func__);
		return ret;
	}
	if (reg == data)
		return 0;
	dev_dbg(&chip->client->dev, "apds990x switch data=0x%x\n", data);
	ret = apds990x_write_byte(chip, APDS990X_ENABLE, data);
	if (ret < 0) {
		dev_err(&chip->client->dev,
			"%s: APDS990X_ENABLE write failed\n", __func__);
		return ret;
	}
	msleep(APDS_STARTUP_DELAY / 1000 +
			((APDS_STARTUP_DELAY % 1000) ? 1 : 0));
	return 0;
}

/* mutex must be held when calling this function */
static void apds990x_mode(struct alsps_client *client, int mode)
{
	struct apds990x_chip *chip = client->chip;

	switch (mode) {
	case APDS_PS_DISABLE:
		if (!test_and_clear_bit(PS_IOCTL_ENABLE, &client->status)) {
			dev_dbg(&chip->client->dev,
				"PS is not enabled for this client\n");
				return;
		}
		if (--chip->ps_cnt <= 0) {
			chip->ps_cnt = 0;
			chip->prox_data = APDS_PS_INIT_DATA;
			chip->alsps_switch &= ~APDS_PS_ENABLE;
		}
		break;
	case APDS_PS_ENABLE:
		if (test_and_set_bit(PS_IOCTL_ENABLE, &client->status))
			return;
		chip->ps_cnt++;

		/* always report first data when ps power on */
		if (chip->ps_cnt == 1)
			apds990x_force_p_refresh(chip);
		chip->alsps_switch |= APDS_PS_ENABLE;
		break;
	case APDS_ALS_DISABLE:
		if (!test_and_clear_bit(ALS_IOCTL_ENABLE, &client->status)) {
			dev_dbg(&chip->client->dev,
				"ALS is not enabled for this client\n");
				return;
		}
		if (--chip->als_cnt <= 0) {
			chip->als_cnt = 0;
			chip->alsps_switch &= ~APDS_ALS_ENABLE;
		}
		break;
	case APDS_ALS_ENABLE:
		if (test_and_set_bit(ALS_IOCTL_ENABLE, &client->status) ||
				chip->als_cnt++ > 0)
			return;
		/* always report first data when als power on */
		als_update_gain(chip, APDS_ALS_INIT_GAIN);
		apds990x_force_a_refresh(chip);
		chip->alsps_switch |= APDS_ALS_ENABLE;
		break;
	default:
		break;
	}
	apds990x_switch(chip, chip->alsps_switch);
}

static irqreturn_t apds990x_irq(int irq, void *data)
{
	u8 status;
	int i, value;
	struct apds990x_chip *chip = data;

	mutex_lock(&chip->mutex);
	for (i = 0; i < APDS_GPIO_CHECK_MAX; i++) {
		apds990x_read_byte(chip, APDS990X_STATUS, &status);
		apds990x_ack_int(chip, status);

		if (status & APDS990X_ST_AINT)
			als_handle_irq(chip);
		if (status & APDS990X_ST_PINT)
			ps_handle_irq(chip);

		/* Since apds990x's interrupt pin is level type and some GPIO
		 * controllers don't support level trigger, we need to check
		 * gpio pin value to see if there is another interupt occurs
		 * in the time window that interrupt status register read and
		 * interrupt ack. If that happens, do irq handle again to
		 * avert interrupt missing.
		 */
		value = gpio_get_value(chip->pdata->gpio_number);
		dev_dbg(&chip->client->dev,
					"%s: try=%d, GPIO value = 0x%x",
					__func__, i, value);
		if (value)
			break;
	}
	if (i == APDS_GPIO_CHECK_MAX) {
		dev_dbg(&chip->client->dev,
				"GPIO check max, reset the sensor\n");
		apds990x_switch(chip, APDS_POWER_DOWN);
		apds990x_switch(chip, chip->alsps_switch);
	}
	mutex_unlock(&chip->mutex);

	return IRQ_HANDLED;
}

static long ps_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct alsps_client *client = file->private_data;
	struct apds990x_chip *chip = client->chip;

	dev_dbg(&chip->client->dev,
		"cmd = %d, arg = %d\n", (int)cmd, (int)arg);
	/* 1 - enable; 0 - disable */

	mutex_lock(&chip->mutex);
	switch (arg) {
	case 0:
		apds990x_mode(client, APDS_PS_DISABLE);
		break;
	case 1:
		apds990x_mode(client, APDS_PS_ENABLE);
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&chip->mutex);
	return ret;
}

static long als_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct alsps_client *client = file->private_data;
	struct apds990x_chip *chip = client->chip;

	dev_dbg(&chip->client->dev,
		"cmd = %d, arg = %d\n", (int)cmd, (int)arg);

	mutex_lock(&chip->mutex);
	/* 1 - enable; 0 - disable */
	switch (arg) {
	case 0:
		apds990x_mode(client, APDS_ALS_DISABLE);
		break;
	case 1:
		apds990x_mode(client, APDS_ALS_ENABLE);
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&chip->mutex);
	return ret;
}

static int als_close(struct inode *inode, struct file *filep)
{
	struct alsps_client *client = filep->private_data;
	struct apds990x_chip *chip = client->chip;

	mutex_lock(&chip->mutex);
	list_del(&client->list);

	if (test_bit(ALS_IOCTL_ENABLE, &client->status))
		apds990x_mode(client, APDS_ALS_DISABLE);
	mutex_unlock(&chip->mutex);
	kfree(client);
	filep->private_data = NULL;

	return 0;
}

static int ps_close(struct inode *inode, struct file *filep)
{
	struct alsps_client *client = filep->private_data;
	struct apds990x_chip *chip = client->chip;

	mutex_lock(&chip->mutex);
	list_del(&client->list);

	if (test_bit(PS_IOCTL_ENABLE, &client->status))
		apds990x_mode(client, APDS_PS_DISABLE);
	mutex_unlock(&chip->mutex);
	kfree(client);
	filep->private_data = NULL;

	return 0;
}

static const struct file_operations ps_fops = {
	.owner = THIS_MODULE,
	.open = ps_open,
	.read = ps_read,
	.poll = ps_poll,
	.release = ps_close,
	.unlocked_ioctl = ps_ioctl,
	.llseek = no_llseek,
};

static const struct file_operations als_fops = {
	.owner = THIS_MODULE,
	.open = als_open,
	.read = als_read,
	.poll = als_poll,
	.release = als_close,
	.unlocked_ioctl = als_ioctl,
	.llseek = no_llseek,
};

static void apds990x_early_suspend_handler(struct apds990x_chip *chip)
{
	dev_dbg(&chip->client->dev, "enter %s\n", __func__);

	mutex_lock(&chip->mutex);
	/* Only proximity is kept actice over the suspend period */
	apds990x_switch(chip, chip->alsps_switch & APDS_PS_ENABLE);
	mutex_unlock(&chip->mutex);
}

static void apds990x_late_resume_handler(struct apds990x_chip *chip)
{
	dev_dbg(&chip->client->dev, "enter %s\n", __func__);

	mutex_lock(&chip->mutex);
	chip->lux_wait_fresh_res = true;
	apds990x_switch(chip, chip->alsps_switch);
	mutex_unlock(&chip->mutex);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void apds990x_early_suspend(struct early_suspend *h)
{
	struct apds990x_chip *chip = container_of(h, struct apds990x_chip, es);

	apds990x_early_suspend_handler(chip);
}

static void apds990x_late_resume(struct early_suspend *h)
{
	struct apds990x_chip *chip = container_of(h, struct apds990x_chip, es);

	apds990x_late_resume_handler(chip);
}
#endif

static struct apds990x_chip *apds990x_alloc_dev(void)
{
	struct apds990x_chip *chip;

	chip = kzalloc(sizeof(struct apds990x_chip), GFP_KERNEL);
	if (!chip)
		return NULL;

	mutex_init(&chip->mutex);
	init_waitqueue_head(&chip->ps_workq_head);
	init_waitqueue_head(&chip->als_wordq_head);

	INIT_LIST_HEAD(&chip->als_list);
	INIT_LIST_HEAD(&chip->ps_list);

	return chip;
}

static int apds990x_setup_irq(struct apds990x_chip *chip)
{
	int ret;
	int gpio = chip->pdata->gpio_number;
	struct i2c_client *client = chip->client;

	dev_dbg(&client->dev, "apds990x setup irq from gpio %d.", gpio);
	ret = gpio_request(gpio, "apds990x");
	if (ret < 0) {
		dev_err(&client->dev, "Request gpio %d failed!\n", gpio);
		goto out;
	}
	ret = gpio_direction_input(gpio);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to configure input\n");
		goto fail_gpio;
	}
	ret = gpio_to_irq(gpio);
	if (ret < 0) {
		dev_err(&client->dev, "Configure gpio to irq failed!\n");
		goto fail_gpio;
	}
	client->irq = ret;
	dev_dbg(&client->dev, "irq = %d.", client->irq);

	ret = request_threaded_irq(client->irq, NULL,
				apds990x_irq,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"apds990x", chip);
	if (ret < 0) {
		dev_err(&client->dev, "Can't allocate irq %d\n", client->irq);
		goto fail_gpio;
	}

	return 0;

fail_gpio:
	gpio_free(gpio);
out:
	return ret;
}

static void apds990x_init_params(struct apds990x_chip *chip)
{
	if (chip->pdata->cf.ga == 0) {
		/* set uncovered sensor default parameters */
		if (chip->id == APDS993X_ID_30 || chip->id == APDS993X_ID_39) {
			chip->cf.ga = 2007; /* 0.49 * APDS_PARAM_SCALE */
			chip->cf.cf1 = 4096; /* 1.00 * APDS_PARAM_SCALE */
			chip->cf.irf1 = 7627; /* 1.862 * APDS_PARAM_SCALE */
			chip->cf.cf2 = 3056; /* 0.746 * APDS_PARAM_SCALE */
			chip->cf.irf2 = 5288; /* 1.291 * APDS_PARAM_SCALE */
			chip->cf.df = 52;
		} else {
			chip->cf.ga = 1966; /* 0.48 * APDS_PARAM_SCALE */
			chip->cf.cf1 = 4096; /* 1.00 * APDS_PARAM_SCALE */
			chip->cf.irf1 = 9134; /* 2.23 * APDS_PARAM_SCALE */
			chip->cf.cf2 = 2867; /* 0.70 * APDS_PARAM_SCALE */
			chip->cf.irf2 = 5816; /* 1.42 * APDS_PARAM_SCALE */
			chip->cf.df = 52;
		}
	} else {
		chip->cf = chip->pdata->cf;
	}

	/* precalculate inverse chip factors for threshold control */
	chip->rcf.afactor =
		(chip->cf.irf1 - chip->cf.irf2) * APDS_PARAM_SCALE /
		(chip->cf.cf1 - chip->cf.cf2);

	chip->rcf.cf1 =  APDS_PARAM_SCALE * APDS_PARAM_SCALE / chip->cf.cf1;
	chip->rcf.irf1 = chip->cf.irf1 * APDS_PARAM_SCALE    / chip->cf.cf1;

	chip->rcf.cf2 =  APDS_PARAM_SCALE * APDS_PARAM_SCALE / chip->cf.cf2;
	chip->rcf.irf2 = chip->cf.irf2 * APDS_PARAM_SCALE    / chip->cf.cf2;

	/* Set something to start with */
	chip->lux_thres_hi = APDS_LUX_DEF_THRES_HI;
	chip->lux_thres_lo = APDS_LUX_DEF_THRES_LO;
	chip->lux_calib = APDS_LUX_NEUTRAL_CALIB_VALUE;
	chip->lux_wait_fresh_res = true;

	chip->prox_thres = APDS_PROX_DEF_THRES;
	chip->pdrive = chip->pdata->pdrive;
	chip->pdiode = APDS_PDIODE_IR;
	chip->pgain = APDS_PGAIN_1X;
	chip->prox_calib = APDS_PROX_NEUTRAL_CALIB_VALUE;
	chip->prox_persistence = APDS_DEFAULT_PROX_PERS;
	chip->prox_data = APDS_PS_INIT_DATA;
}

static ssize_t early_suspend_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct apds990x_chip *chip =  dev_get_drvdata(dev);

	if (!strncmp(buf, EARLY_SUSPEND_ON, EARLY_SUSPEND_STATUS_LEN))
		apds990x_early_suspend_handler(chip);
	else if (!strncmp(buf, EARLY_SUSPEND_OFF, EARLY_SUSPEND_STATUS_LEN))
		apds990x_late_resume_handler(chip);

	return count;
}

static DEVICE_EARLY_SUSPEND_ATTR(early_suspend_store);

static int apds990x_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct apds990x_chip *chip;
	int err;

	dev_dbg(&client->dev, "apds990x driver probe.");
	chip = apds990x_alloc_dev();
	if (!chip)
		return -ENOMEM;

	chip->client = client;
	chip->pdata = client->dev.platform_data;
	i2c_set_clientdata(client, chip);

	if (chip->pdata == NULL) {
		dev_err(&chip->client->dev,
				"apds990x platform data is mandatory\n");
		err = -EINVAL;
		goto fail1;
	}

	if (chip->pdata->setup_resources) {
		err = chip->pdata->setup_resources();
		if (err) {
			dev_err(&chip->client->dev,
					"pdata setup_resources error\n");
			err = -EINVAL;
			goto fail1;
		}
	}

	err = apds990x_detect(chip);
	if (err < 0) {
		dev_err(&client->dev, "APDS990X not found\n");
		goto fail2;
	}

	apds990x_init_params(chip);
	apds990x_configure(chip);
	apds990x_set_arate(chip, APDS_LUX_DEFAULT_RATE);
	apds990x_switch(chip, APDS_POWER_DOWN);

	err = sysfs_create_group(&chip->client->dev.kobj,
				apds990x_attribute_group);
	if (err < 0) {
		dev_err(&chip->client->dev, "Sysfs registration failed\n");
		goto fail2;
	}

	chip->ps_dev.minor = MISC_DYNAMIC_MINOR;
	chip->ps_dev.name = "apds990x_psensor";
	chip->ps_dev.fops = &ps_fops;

	chip->als_dev.minor = MISC_DYNAMIC_MINOR;
	chip->als_dev.name = "apds990x_lsensor";
	chip->als_dev.fops = &als_fops;

	err = misc_register(&chip->ps_dev);
	if (err) {
		dev_err(&client->dev, "proximity miscdev register failed\n");
		goto fail3;
	}

	err = misc_register(&chip->als_dev);
	if (err) {
		dev_err(&client->dev, "ambient miscdev register failed\n");
		goto fail4;
	}

	err = apds990x_setup_irq(chip);
	if (err) {
		dev_err(&client->dev, "Setup IRQ error\n");
		goto fail5;
	}
	enable_irq_wake(client->irq);

	device_create_file(&client->dev, &dev_attr_early_suspend);

#ifdef CONFIG_HAS_EARLYSUSPEND
	chip->es.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 10;
	chip->es.suspend = apds990x_early_suspend;
	chip->es.resume = apds990x_late_resume;
	register_early_suspend(&chip->es);
#endif
	apds990x_force_a_refresh(chip);
	apds990x_force_p_refresh(chip);

	register_early_suspend_device(&client->dev);

	return err;
fail5:
	misc_deregister(&chip->als_dev);
fail4:
	misc_deregister(&chip->ps_dev);
fail3:
	sysfs_remove_group(&chip->client->dev.kobj,
			&apds990x_attribute_group[0]);
fail2:
	if (chip->pdata->release_resources)
		chip->pdata->release_resources();
fail1:
	kfree(chip);
	return err;
}

static int apds990x_remove(struct i2c_client *client)
{
	struct apds990x_chip *chip = i2c_get_clientdata(client);

	disable_irq_wake(client->irq);
	free_irq(client->irq, chip);
	unregister_early_suspend_device(&client->dev);
	device_remove_file(&client->dev, &dev_attr_early_suspend);
	sysfs_remove_group(&chip->client->dev.kobj,
			apds990x_attribute_group);
	misc_deregister(&chip->ps_dev);
	misc_deregister(&chip->als_dev);

	if (chip->pdata && chip->pdata->release_resources)
		chip->pdata->release_resources();

	apds990x_switch(chip, APDS_POWER_DOWN);
	unregister_early_suspend(&chip->es);

	kfree(chip);
	return 0;
}

#ifdef CONFIG_PM
static int apds990x_suspend(struct device *dev)
{
	struct i2c_client *i2c_client = to_i2c_client(dev);
	struct apds990x_chip *chip = i2c_get_clientdata(i2c_client);
	struct alsps_client *client;

	dev_dbg(&i2c_client->dev, "%s: pm suspend", __func__);
	disable_irq(i2c_client->irq);
	if (!mutex_trylock(&chip->mutex)) {
		goto out1;
	}
	list_for_each_entry(client, &chip->ps_list, list) {
		if (test_bit(PS_DATA_READY, &client->status)) {
			goto out2;
		}
	}
	mutex_unlock(&chip->mutex);
	return 0;
out2:
	mutex_unlock(&chip->mutex);
out1:
	enable_irq(i2c_client->irq);
	return -EBUSY;
}

static int apds990x_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	dev_dbg(&client->dev, "%s: pm resume", __func__);
	enable_irq(client->irq);
	return 0;
}
#else
#define apds990x_suspend  NULL
#define apds990x_resume   NULL
#define apds990x_shutdown NULL
#endif

static const struct i2c_device_id apds990x_id[] = {
	{"apds990x", 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, apds990x_id);

static const struct dev_pm_ops apds990x_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(apds990x_suspend, apds990x_resume)
};

static struct i2c_driver apds990x_driver = {
	.driver	 = {
		.name	= "apds990x",
		.owner	= THIS_MODULE,
		.pm	= &apds990x_pm_ops,
	},
	.probe	  = apds990x_probe,
	.remove	  = apds990x_remove,
	.id_table = apds990x_id,
};

static int __init apds990x_init(void)
{
	return i2c_add_driver(&apds990x_driver);
}

static void __exit apds990x_exit(void)
{
	i2c_del_driver(&apds990x_driver);
}

MODULE_DESCRIPTION("APDS990X combined ALS and proximity sensor");
MODULE_AUTHOR("Samu Onkalo, Nokia Corporation");
MODULE_LICENSE("GPL v2");

module_init(apds990x_init);
module_exit(apds990x_exit);
