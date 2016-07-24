/*
 * drivers/misc/kl03-hinge.c
 *
 * Copyright (c) 2014 Google, Inc.
 * Corey Tabaka <eieio@google.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kl03_hinge.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/wakelock.h>

#define WAKELOCK_TIMEOUT_MS	2000

#define HINGE_BL_SLAVE_ADDRESS  0x10
#define HINGE_SLAVE_ADDRESS     0x18

#define HINGE_NAME "btns_mcu_bl"

/* MCU bootloader protocol definitions */

#define MCU_PACKET_START_BYTE         0x5a

#define MCU_PACKET_TYPE_ACK           0xa1
#define MCU_PACKET_TYPE_NACK          0xa2
#define MCU_PACKET_TYPE_ABORT         0xa3
#define MCU_PACKET_TYPE_COMMAND       0xa4
#define MCU_PACKET_TYPE_DATA          0xa5
#define MCU_PACKET_TYPE_PING          0xa6
#define MCU_PACKET_TYPE_PING_RESPONSE 0xa7

/*
 * MCU register definitions.
 * All registers are 32-bit, 4-byte aligned.
 */
#define REG_HALL_LAST_ADC_VALUE   0x00
#define REG_HALL_STATUS           0x04
#define REG_HALL_HINGE_CLOSED     0x08
#define REG_HALL_HINGE_OPENED     0x0c
#define REG_HALL_PREVIEW_RELEASED 0x10
#define REG_HALL_PREVIEW_PRESSED  0x14
#define REG_HALL_PHOTO_RELEASED   0x18
#define REG_HALL_PHOTO_PRESSED    0x1c
#define REG_FIRMWARE_REVISION     0xe8
#define REG_FIRMWARE_DIRTY        0xec
#define REG_TEST_1                0xf4
#define REG_TEST_2                0xf8
#define REG_TEST_3                0xfc

/* REG_HALL_STATUS bits */
#define HALL_STATUS_FILTER_EN    (1<<0)
#define HALL_STATUS_FILTER_MASK  (0x3<<1)
#define HALL_STATUS_FILTER_SHIFT 1
#define HALL_STATUS_HINGE        (1<<8)
#define HALL_STATUS_PREVIEW      (1<<9)
#define HALL_STATUS_PHOTO        (1<<10)

enum mcu_status {
	MCU_STATUS_RESET = 0,
	MCU_STATUS_RUNNING,
	MCU_STATUS_BOOTLOADER,
	MCU_STATUS_ERROR,
};

struct hinge_data {
	struct device *dev;
	struct input_dev *idev;
	struct wake_lock wakelock;

	int irq_hinge;
	int irq_photo;
	int irq_preview;

	int hinge_gpio;
	int photo_gpio;
	int preview_gpio;

	int bootcfg_gpio;
	int reset_gpio;
	int active_polarity;

	enum mcu_status mcu_status;
};

#define CHECK_ERROR(cond, label, fmt, args...) \
	do { \
		if ((cond)) { \
			dev_err(data->dev, fmt, ##args); \
			goto label; \
		} \
	} while (0)

/*
 * Utility routines to assert/deassert reset and bootcfg lines.
 * These routines automatically handle the changes from active low
 * active high that occured in later board revisions.
 */
static void hinge_set_reset(struct hinge_data *data, bool asserted)
{
	int v = (!!asserted) == (!!data->active_polarity);

	gpio_set_value_cansleep(data->reset_gpio, v);
}

static int hinge_get_reset(struct hinge_data *data)
{
	int v = gpio_get_value_cansleep(data->reset_gpio);

	return (!!v) == (!!data->active_polarity);
}

static void hinge_set_bootcfg(struct hinge_data *data, bool asserted)
{
	int v = (!!asserted) == (!!data->active_polarity);

	gpio_set_value_cansleep(data->bootcfg_gpio, v);
}

static int hinge_get_bootcfg(struct hinge_data *data)
{
	int v = gpio_get_value_cansleep(data->bootcfg_gpio);

	return (!!v) == (!!data->active_polarity);
}

static int hinge_read_reg(struct hinge_data *data, uint8_t reg, uint32_t *value)
{
	struct i2c_client *client;
	struct i2c_msg msg[2];
	int ret;

	if (!data || !value)
		return -EINVAL;

	client = to_i2c_client(data->dev);

	msg[0].addr = HINGE_SLAVE_ADDRESS;
	msg[0].flags = 0;
	msg[0].len = sizeof(uint8_t);
	msg[0].buf = (void *) &reg;

	msg[1].addr = HINGE_SLAVE_ADDRESS;
	msg[1].flags = I2C_M_RD;
	msg[1].len = sizeof(uint32_t);
	msg[1].buf = (void *) value;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	CHECK_ERROR(ret < 0, error, "%s: Failed to read register %02x: %d\n",
				__func__, reg, ret);

	return 0;

error:
	return ret;
}

static int hinge_write_reg(struct hinge_data *data, uint8_t reg, uint32_t value)
{
	struct i2c_client *client;
	struct i2c_msg msg[2];
	int ret;

	if (!data)
		return -EINVAL;

	client = to_i2c_client(data->dev);

	msg[0].addr = HINGE_SLAVE_ADDRESS;
	msg[0].flags = 0;
	msg[0].len = sizeof(uint8_t);
	msg[0].buf = (void *) &reg;

	msg[1].addr = HINGE_SLAVE_ADDRESS;
	msg[1].flags = 0;
	msg[1].len = sizeof(uint32_t);
	msg[1].buf = (void *) &value;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	CHECK_ERROR(ret < 0, error, "%s: Failed to write register %02x: %d\n",
				__func__, reg, ret);

	return 0;

error:
	return ret;
}

/* calculate the crc of a ping response packet */
static uint16_t crc16(const uint8_t *src, uint32_t len)
{
	uint32_t crc = 0;
	uint32_t j, i, byte, temp;

	for (j = 0; j < len; j++) {
		byte = src[j];

		crc ^= byte << 8;

		for (i = 0; i < 8; i++) {
			temp = crc << 1;

			if (crc & 0x8000)
				temp ^= 0x1021;

			crc = temp;
		}
	}

	return crc;
}

/* calculate the crc of a packet, skipping the internal crc field */
static uint16_t crc16_packet(uint8_t *src, uint32_t len)
{
	uint32_t crc = 0;
	uint32_t j, i, byte, temp;

	for (j = 0; j < len; j++) {
		if (j == 4 || j == 5) /* skip the crc field */
			continue;

		byte = src[j];

		crc ^= byte << 8;

		for (i = 0; i < 8; i++) {
			temp = crc << 1;

			if (crc & 0x8000)
				temp ^= 0x1021;

			crc = temp;
		}
	}

	return crc;
}

/* calculate a packet crc and update the packet crc field in place */
static void crc16_update(uint8_t *src, uint32_t len)
{
	uint16_t crc;

	crc = crc16_packet(src, len);

	src[4] = crc & 0xff;
	src[5] = crc >> 8;
}

static int hinge_bl_ping(struct hinge_data *data)
{
	struct i2c_client *client;
	struct i2c_msg msg[2];
	uint16_t *crc;
	uint8_t buf[10];
	int ret;

	if (!data)
		return -EINVAL;

	client = to_i2c_client(data->dev);

	buf[0] = MCU_PACKET_START_BYTE;
	buf[1] = MCU_PACKET_TYPE_PING;

	msg[0].addr = HINGE_BL_SLAVE_ADDRESS;
	msg[0].flags = 0;
	msg[0].len = sizeof(uint8_t) * 2;
	msg[0].buf = (void *) buf;

	msg[1].addr = HINGE_BL_SLAVE_ADDRESS;
	msg[1].flags = I2C_M_RD;
	msg[1].len = sizeof(buf);
	msg[1].buf = (void *) buf;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	CHECK_ERROR(ret < 0, error, "%s: Failed to ping MCU bootloader: %d\n",
			__func__, ret);

	crc = (uint16_t *) &buf[8];

	if (buf[0] != 0x5a || buf[1] != 0xa7 || crc16(buf, 8) != *crc) {
		dev_err(data->dev,
		"Invalid MCU ping response: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
			buf[0], buf[1], buf[2], buf[3], buf[4], buf[5],
			buf[6], buf[7], buf[8], buf[9]);
		ret = -EINVAL;
		goto error;
	}

	return 0;

error:
	return ret;
}

static int hinge_bl_send_ack(struct hinge_data *data)
{
	int err;
	struct i2c_client *client;
	struct i2c_msg msg[1];
	uint8_t buf[2];

	if (!data)
		return -EINVAL;

	client = to_i2c_client(data->dev);

	buf[0] = MCU_PACKET_START_BYTE;
	buf[1] = MCU_PACKET_TYPE_ACK;

	msg[0].addr = HINGE_BL_SLAVE_ADDRESS;
	msg[0].flags = 0;
	msg[0].len = sizeof(buf);
	msg[0].buf = (void *) buf;

	err = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	CHECK_ERROR(err < 0, error, "Failed to send ACK: %d\n", err);

	return 0;

error:
	return err;
}

static int hinge_bl_read_generic_response(struct hinge_data *data, uint32_t *resp)
{
	int err, retries;
	struct i2c_client *client;
	struct i2c_msg msg[1];
	uint8_t buf[38]; /* framing and command packet with up to 7 args */
	uint16_t crc, len;
	uint32_t *args;

	if (!data || !resp)
		return -EINVAL;

	client = to_i2c_client(data->dev);

	msg[0].addr = HINGE_BL_SLAVE_ADDRESS;
	msg[0].flags = I2C_M_RD;
	msg[0].len = 1;
	msg[0].buf = (void *) buf;

	retries = 100;
	do {
		err = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
		CHECK_ERROR(err < 0, error, "Failed to read start byte: %d\n", err);
	} while (buf[0] != MCU_PACKET_START_BYTE && retries--);

	if (retries < 0) {
		err = -ETIMEDOUT;
		goto error;
	}

	msg[0].buf = (void *) &buf[1];

	err = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	CHECK_ERROR(err < 0, error, "Failed ready type byte: %d\n", err);

	if (buf[1] != MCU_PACKET_TYPE_COMMAND) {
		dev_err(data->dev, "Received unexpected type byte while reading response: %02x\n",
				buf[1]);
		err = -EIO;
		goto error;
	}

	/* read crc and length fields */
	msg[0].len = 4;
	msg[0].buf = (void *) &buf[2];

	err = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	CHECK_ERROR(err < 0, error, "Failed to read payload: %d\n", err);

	/* NOTE: assumes little endian host */
	len = *((uint16_t *) &buf[2]);
	crc = *((uint16_t *) &buf[4]);

	if (len > 32) {
		dev_err(data->dev, "Slave responded with a payload larger than 32 bytes (%d).\n",
				len);
		err = -EIO;
		goto error;
	}

	/* read the payload */
	msg[0].len = len;
	msg[0].buf = (void *) &buf[6];

	err = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	CHECK_ERROR(err < 0, error, "Failed to read payload: %d\n", err);

	/* validate the response */
	if (crc16_packet(buf, 6 + len) != crc) {
		dev_err(data->dev, "CRC failure in response packet\n");
		err = -EIO;
		goto error;
	}

	dev_info(data->dev, "Response packet: TAG=%02x param count=%02x\n", buf[6], buf[9]);

	if (buf[9] != 2) {
		dev_err(data->dev, "Expected param count to be 2, found %d\n", buf[9]);
		err = -EIO;
		goto error;
	}

	/* read the args */
	args = (uint32_t *) &buf[10];
	*resp = args[0];

	dev_info(data->dev, "Response packet: return code=%u command=%u\n", args[0], args[1]);

	return 0;

error:
	return err;
}

static int hinge_bl_read_ack(struct hinge_data *data)
{
	int err, retries;
	struct i2c_client *client;
	struct i2c_msg msg[1];
	uint8_t value;

	if (!data)
		return -EINVAL;

	client = to_i2c_client(data->dev);

	msg[0].addr = HINGE_BL_SLAVE_ADDRESS;
	msg[0].flags = I2C_M_RD;
	msg[0].len = 1;
	msg[0].buf = (void *) &value;

	retries = 100;
	do {
		value = 0;

		err = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
		CHECK_ERROR(err < 0, error, "Failed to read ACK/NACK: %d\n", err);

		if (value != 0 && value != MCU_PACKET_START_BYTE)
			dev_warn(data->dev, "Unexpected value: %02x\n", value);

		usleep_range(10000, 10500);
	} while (value != MCU_PACKET_START_BYTE && retries--);

	if (retries < 0) {
		dev_err(data->dev, "Timed out waiting for ACK/NACK\n");
		err = -ETIMEDOUT;
		goto error;
	}

	err = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	CHECK_ERROR(err < 0, error, "Failed to read ACK/NACK: %d\n", err);

	return value;

error:
	return err;
}

static int hinge_bl_send_cmd(struct hinge_data *data, uint8_t cmd, uint32_t *args, uint8_t count)
{
	int err;
	struct i2c_client *client;
	struct i2c_msg msg[1];
	uint8_t buf[10 + sizeof(uint32_t) * count];
	uint16_t packet_len;

	/* validate args */
	if (!data || (!args && count > 0) || count > 7) {
		dev_err(data->dev, "%s: invalid args: data=%p cmd=%02x args=%p count=%u\n",
				__func__, data, cmd, args, count);
		return -EINVAL;
	}

	client = to_i2c_client(data->dev);

	/* packet length is size of payload after framing header */
	packet_len = 4 + sizeof(uint32_t) * count;

	/* setup framing packet */
	buf[0] = MCU_PACKET_START_BYTE;
	buf[1] = MCU_PACKET_TYPE_COMMAND;
	buf[2] = packet_len; /* packet len low */
	buf[3] = packet_len >> 8; /* packet len high */
	buf[4] = 0x00; /* crc16 low (updated later) */
	buf[5] = 0x00; /* crc16 high (updated later) */

	/* setup command packet */
	buf[6] = cmd; /* command */
	buf[7] = 0x00; /* flags */
	buf[8] = 0x00; /* reserved */
	buf[9] = count; /* param count */

	/* copy the args into the packet */
	if (args)
		memcpy(&buf[10], args, sizeof(uint32_t) * count);

	crc16_update(buf, sizeof(buf));

	msg[0].addr = HINGE_BL_SLAVE_ADDRESS;
	msg[0].flags = 0;
	msg[0].len = sizeof(buf);
	msg[0].buf = (void *) buf;

	err = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	CHECK_ERROR(err < 0, error, "Failed to send command packet: %d\n", err);

	err = hinge_bl_read_ack(data);

error:
	return err;
}

static int hinge_bl_send_data(struct hinge_data *data, const void *payload, uint16_t len)
{
	int err;
	struct i2c_client *client;
	struct i2c_msg msg[1];
	uint8_t buf[6 + len];

	/* validate args */
	if (!payload || len > 32)
		return -EINVAL;

	client = to_i2c_client(data->dev);

	buf[0] = MCU_PACKET_START_BYTE; /* start byte */
	buf[1] = MCU_PACKET_TYPE_DATA; /* data packet */
	buf[2] = len; /* length low */
	buf[3] = len >> 8; /* length high */
	buf[4] = 0x00; /* crc low */
	buf[5] = 0x00; /* crc high */

	memcpy(&buf[6], payload, len);

	crc16_update(buf, sizeof(buf));

	msg[0].addr = HINGE_BL_SLAVE_ADDRESS;
	msg[0].flags = 0;
	msg[0].len = sizeof(buf);
	msg[0].buf = (void *) buf;

	err = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	CHECK_ERROR(err < 0, error, "Failed to send data packet: %d\n", err);

	err = hinge_bl_read_ack(data);

error:
	return err;
}

static int hinge_bl_send_reset(struct hinge_data *data)
{
	int err;
	uint32_t resp;

	err = hinge_bl_send_cmd(data, 0x0b, NULL, 0);
	if (err < 0)
		goto error;

	switch (err) {
	case MCU_PACKET_TYPE_ACK:
	case MCU_PACKET_TYPE_ABORT:
		break;

	case MCU_PACKET_TYPE_NACK:
	default:
		dev_err(data->dev, "Received negative or error response: %02x\n", err);
		goto error;
	}

	err = hinge_bl_read_generic_response(data, &resp);
	CHECK_ERROR(err, error, "Failed to receive response: %d\n", err);

	err = hinge_bl_send_ack(data);
	CHECK_ERROR(err, error, "Failed to send ack\n");

	if (resp != 0) {
		dev_err(data->dev, "Received error response from slave: %u\n", resp);
		err = -EINVAL;
		goto error;
	}

	return 0;

error:
	return err;
}

static int hinge_bl_execute(struct hinge_data *data, uint32_t address, uint32_t stack)
{
	int err;
	uint32_t resp;
	uint32_t args[3];

	args[0] = address;
	args[1] = 1;
	args[2] = stack;

	err = hinge_bl_send_cmd(data, 0x09, args, 3);
	CHECK_ERROR(err < 0, error, "Failed to send execute command");

	switch (err) {
	case MCU_PACKET_TYPE_ACK:
	case MCU_PACKET_TYPE_ABORT:
		break;

	case MCU_PACKET_TYPE_NACK:
	default:
		dev_err(data->dev, "Received negative or error response: %02x\n", err);
		goto error;
	}

	err = hinge_bl_read_generic_response(data, &resp);
	CHECK_ERROR(err, error, "Failed to receive response: %d\n", err);

	err = hinge_bl_send_ack(data);
	CHECK_ERROR(err, error, "Failed to send ack\n");

	if (resp != 0) {
		dev_err(data->dev, "Received error response from slave: %u\n", resp);
		err = -EINVAL;
		goto error;
	}

	return 0;

error:
	return err;
}

static int hinge_bl_write(struct hinge_data *data, uint32_t address, const void *payload, size_t len)
{
	int err;
	uint32_t resp;
	uint32_t args[2];
	size_t sent = 0, n;
	const uint8_t *out = payload;

	args[0] = address;
	args[1] = len;

	err = hinge_bl_send_cmd(data, 0x04, args, 2);
	CHECK_ERROR(err < 0, error, "Failed to send write command");

	switch (err) {
	case MCU_PACKET_TYPE_ACK:
	case MCU_PACKET_TYPE_ABORT:
		break;

	case MCU_PACKET_TYPE_NACK:
	default:
		dev_err(data->dev, "Received negative or error response: %02x\n", err);
		goto error;
	}

	err = hinge_bl_read_generic_response(data, &resp);
	CHECK_ERROR(err, error, "Failed to receive response: %d\n", err);

	err = hinge_bl_send_ack(data);
	CHECK_ERROR(err, error, "Failed to send ack\n");

	if (resp != 0) {
		dev_err(data->dev, "Received error response from slave: %u\n", resp);
		err = -EINVAL;
		goto error;
	}

	/* send data packets */
	while (sent < len) {
		n = len - sent;
		if (n > 32)
			n = 32;

		dev_info(data->dev, "Sending data packet: sent=%zu n=%zu\n", sent, n);

		err = hinge_bl_send_data(data, out, n);
		CHECK_ERROR(err < 0, error, "Failed to send data");

		switch (err) {
		case MCU_PACKET_TYPE_ACK:
		case MCU_PACKET_TYPE_ABORT:
			break;

		case MCU_PACKET_TYPE_NACK:
		default:
			dev_err(data->dev, "Received negative or error response: %02x\n", err);
			goto error;
		}

		out += n;
		sent += n;
	}

	/* receive generic response and send ack */
	err = hinge_bl_read_generic_response(data, &resp);
	CHECK_ERROR(err, error, "Failed to receive response: %d\n", err);

	err = hinge_bl_send_ack(data);
	CHECK_ERROR(err, error, "Failed to send ack\n");

	if (resp != 0) {
		dev_err(data->dev, "Received error response from slave: %u\n", resp);
		err = -EINVAL;
		goto error;
	}

	return 0;

error:
	return err;
}

/* jumpstart trampoline */
static const uint16_t jumpstart_buf[] = {
	0x4040, /* eor r0, r0, r0 */
	0x6801, /* ldr r1, [r0, #0] */
	0x6842, /* ldr r2, [r0, #4] */
	0x468d, /* mov sp, r1 */
	0x4710, /* bx r2 */
};

static int hinge_bl_jumpstart(struct hinge_data *data)
{
	int err;

	dev_info(data->dev, "Jumpstarting MCU firmware ...\n");

	err = hinge_bl_write(data, 0x200003c0, jumpstart_buf, sizeof(jumpstart_buf));
	CHECK_ERROR(err, error, "Failed to write to MCU");

	/* make sure NMI isn't asserted when SW runs*/
	hinge_set_bootcfg(data, false);

	err = hinge_bl_execute(data, 0x200003c1, 0x200005fc);
	CHECK_ERROR(err, error, "Failed to execute jumpstart program");

	return 0;

error:
	return err;
}

static int hinge_determine_status(struct hinge_data *data)
{
	int err;
	int value;

	if (hinge_get_reset(data)) {
		data->mcu_status = MCU_STATUS_RESET;
		goto done;
	}

	err = hinge_read_reg(data, REG_TEST_1, &value);
	if (err == 0 && value == 0xaa55ff00) {
		data->mcu_status = MCU_STATUS_RUNNING;
		goto done;
	}

	err = hinge_bl_ping(data);
	if (!err) {
		data->mcu_status = MCU_STATUS_BOOTLOADER;
		goto done;
	}

	data->mcu_status = MCU_STATUS_ERROR;

done:
	return 0;
}

static void handle_hinge_event(struct hinge_data *data, bool sync)
{
	int value = !!gpio_get_value_cansleep(data->hinge_gpio);
	dev_info(data->dev, "Hinge event: closed=%d\n", value);

	input_report_switch(data->idev, SW_RFKILL_ALL, value);
	wake_lock_timeout(&data->wakelock,
			  msecs_to_jiffies(WAKELOCK_TIMEOUT_MS));

	if (sync)
		input_sync(data->idev);
}

static void handle_photo_event(struct hinge_data *data, bool sync)
{
	int value = !!gpio_get_value_cansleep(data->photo_gpio);
	dev_info(data->dev, "Photo event: pressed=%d\n", value);

	input_report_key(data->idev, KEY_CAMERA, value);

	if (sync)
		input_sync(data->idev);
}

static void handle_preview_event(struct hinge_data *data, bool sync)
{
	int value = !!gpio_get_value_cansleep(data->preview_gpio);
	dev_info(data->dev, "Preview event: pressed=%d\n", value);

	input_report_key(data->idev, KEY_CAMERA_FOCUS, value);

	if (sync)
		input_sync(data->idev);
}

static irqreturn_t hinge_irq_handler(int irq, void *arg)
{
	struct hinge_data *data = arg;

	if (irq == data->irq_hinge)
		handle_hinge_event(data, true);
	else if (irq == data->irq_photo)
		handle_photo_event(data, true);
	else if (irq == data->irq_preview)
		handle_preview_event(data, true);

	return IRQ_HANDLED;
}

/* device attributes */
static ssize_t show_device_status(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct hinge_data *data = dev_get_drvdata(dev);
	const char *status;

	hinge_determine_status(data);

	switch (data->mcu_status) {
	case MCU_STATUS_RESET:
		status = "reset";
		break;

	case MCU_STATUS_RUNNING:
		status = "running";
		break;

	case MCU_STATUS_BOOTLOADER:
		status = "bootloader";
		break;

	case MCU_STATUS_ERROR:
		status = "error";
		break;

	default:
		status = "unknown";
		break;
	}

	return sprintf(buf, "%s\n", status);
}

static ssize_t set_device_status(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct hinge_data *data = dev_get_drvdata(dev);
	int status = count;

	hinge_determine_status(data);

	if (!strncmp(buf, "reset", 5)) {
		hinge_set_reset(data, true);
		hinge_set_bootcfg(data, false);
	} else if (!strncmp(buf, "running", 7)) {
		switch (data->mcu_status) {
		case MCU_STATUS_RESET:
		case MCU_STATUS_ERROR:
			hinge_set_reset(data, true);
			hinge_set_bootcfg(data, false);
			hinge_set_reset(data, false);
			break;

		case MCU_STATUS_BOOTLOADER:
			hinge_bl_send_reset(data);
			break;

		case MCU_STATUS_RUNNING:
			break;
		}
	} else if (!strncmp(buf, "bootloader", 10)) {
		hinge_set_reset(data, true);
		hinge_set_bootcfg(data, true);
		hinge_set_reset(data, false);
	} else if (!strncmp(buf, "jumpstart", 9)) {
		hinge_bl_jumpstart(data);
	} else {
		status = -EINVAL;
	}

	hinge_determine_status(data);

	return status;
}

static ssize_t show_hinge_status(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct hinge_data *data = dev_get_drvdata(dev);
	const char *status;
	uint32_t value;
	int err;

	hinge_determine_status(data);

	if (data->mcu_status == MCU_STATUS_RUNNING) {
		err = hinge_read_reg(data, REG_HALL_STATUS, &value);
		CHECK_ERROR(err, error, "Failed to get hinge status\n");

		status = value & HALL_STATUS_HINGE ? "closed" : "open";
	} else {
		goto error;
	}

	return sprintf(buf, "%s\n", status);

error:
	return sprintf(buf, "error\n");
}

static ssize_t show_photo_status(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct hinge_data *data = dev_get_drvdata(dev);
	const char *status;
	uint32_t value;
	int err;

	hinge_determine_status(data);

	if (data->mcu_status == MCU_STATUS_RUNNING) {
		err = hinge_read_reg(data, REG_HALL_STATUS, &value);
		CHECK_ERROR(err, error, "Failed to get photo status\n");

		status = value & HALL_STATUS_PHOTO ? "pressed" : "unpressed";
	} else {
		goto error;
	}

	return sprintf(buf, "%s\n", status);

error:
	return sprintf(buf, "error\n");
}

static ssize_t show_preview_status(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct hinge_data *data = dev_get_drvdata(dev);
	const char *status;
	uint32_t value;
	int err;

	hinge_determine_status(data);

	if (data->mcu_status == MCU_STATUS_RUNNING) {
		err = hinge_read_reg(data, REG_HALL_STATUS, &value);
		CHECK_ERROR(err, error, "Failed to get preview status\n");

		status = value & HALL_STATUS_PREVIEW ? "active" : "inactive";
	} else {
		goto error;
	}

	return sprintf(buf, "%s\n", status);

error:
	return sprintf(buf, "error\n");
}

static ssize_t show_active_polarity(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct hinge_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", data->active_polarity);
}

static ssize_t set_reset(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct hinge_data *data = dev_get_drvdata(dev);
	int status = count;
	int value;

	if (!strncmp(buf, "0", 1) || !strncmp(buf, "false", 5)) {
		value = 0;
	} else if (!strncmp(buf, "1", 1) || !strncmp(buf, "true", 4)) {
		value = 1;
	} else {
		status = -EINVAL;
		goto error;
	}

	hinge_set_reset(data, value);

error:
	return status;
}

static ssize_t show_reset(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct hinge_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", hinge_get_reset(data));
}

static ssize_t set_bootcfg(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct hinge_data *data = dev_get_drvdata(dev);
	int status = count;
	int value;

	if (!strncmp(buf, "0", 1) || !strncmp(buf, "false", 5)) {
		value = 0;
	} else if (!strncmp(buf, "1", 1) || !strncmp(buf, "true", 4)) {
		value = 1;
	} else {
		status = -EINVAL;
		goto error;
	}

	hinge_set_bootcfg(data, value);

error:
	return status;
}

static ssize_t show_bootcfg(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct hinge_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", hinge_get_bootcfg(data));
}

static DEVICE_ATTR(device_status, S_IWUSR | S_IRUGO, show_device_status,
		set_device_status);
static DEVICE_ATTR(hinge_status, S_IRUGO, show_hinge_status, NULL);
static DEVICE_ATTR(photo_status, S_IRUGO, show_photo_status, NULL);
static DEVICE_ATTR(preview_status, S_IRUGO, show_preview_status, NULL);
static DEVICE_ATTR(active_polarity, S_IRUGO, show_active_polarity, NULL);
static DEVICE_ATTR(reset, S_IWUSR | S_IRUGO, show_reset, set_reset);
static DEVICE_ATTR(bootcfg, S_IWUSR | S_IRUGO, show_bootcfg, set_bootcfg);

static struct attribute *hinge_attributes[] = {
	&dev_attr_device_status.attr,
	&dev_attr_hinge_status.attr,
	&dev_attr_photo_status.attr,
	&dev_attr_preview_status.attr,
	&dev_attr_active_polarity.attr,
	&dev_attr_reset.attr,
	&dev_attr_bootcfg.attr,
	NULL,
};

static const struct attribute_group hinge_attr_group = {
	.attrs = hinge_attributes,
};

static int hinge_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}

static int hinge_resume(struct i2c_client *client)
{
	struct hinge_data *data = i2c_get_clientdata(client);

	handle_hinge_event(data, true);

	return 0;
}

static int hinge_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct hinge_data *data = NULL;
	struct hinge_platform_data *pdata;
	int err = 0;
	int value;
	int retries;
	int bootcfg_value, reset_value;
	int gpio_init_flags;

	dev_info(&client->dev, "%s: enter\n", __func__);

#if 0
	if (client->addr != HINGE_SLAVE_ADDRESS) {
		dev_err(&client->dev, "Unexpected slave address 0x%02x\n",
				client->addr);
		err = -EINVAL;
		goto error_1;
	}
#endif

	pdata = client->dev.platform_data;
	if (!pdata) {
		dev_err(&client->dev, "Missing platform data.\n");
		err = -EINVAL;
		goto error_1;
	}

	data = kzalloc(sizeof(struct hinge_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto error_1;
	}

	i2c_set_clientdata(client, data);
	data->dev = &client->dev;

	data->idev = input_allocate_device();
	CHECK_ERROR(!data->idev, error_2, "Failed to allocate input device!\n");

	data->idev->name = HINGE_NAME "-keys";
	data->idev->phys = HINGE_NAME "-keys/input0";
	data->idev->id.bustype = BUS_I2C;
	data->idev->dev.parent = &client->dev;

	set_bit(EV_KEY, data->idev->evbit);
	set_bit(KEY_CAMERA, data->idev->keybit);
	set_bit(KEY_CAMERA_FOCUS, data->idev->keybit);

	set_bit(EV_SW, data->idev->evbit);
	set_bit(SW_RFKILL_ALL, data->idev->swbit);

	err = input_register_device(data->idev);
	CHECK_ERROR(err, error_3, "Failed to register input device: %d\n", err);

	wake_lock_init(&data->wakelock, WAKE_LOCK_SUSPEND, "hinge_wakelock");

	dev_info(&client->dev, "Defined GPIOs\n");
	dev_info(&client->dev, "reset_gpio: %d\n", pdata->reset_gpio);
	dev_info(&client->dev, "bootcfg_gpio: %d\n", pdata->bootcfg_gpio);
	dev_info(&client->dev, "preview_gpio: %d\n", pdata->irq_preview_gpio);
	dev_info(&client->dev, "photo_gpio: %d\n", pdata->irq_photo_gpio);
	dev_info(&client->dev, "hinge_gpio: %d\n", pdata->irq_hinge_gpio);

	err = -EINVAL;
	CHECK_ERROR(pdata->irq_hinge_gpio == -1, error_4, "hinge_gpio is not defined\n");
	data->irq_hinge = gpio_to_irq(pdata->irq_hinge_gpio);

	CHECK_ERROR(pdata->irq_photo_gpio == -1, error_4, "photo_gpio is not defined\n");
	data->irq_photo = gpio_to_irq(pdata->irq_photo_gpio);

	CHECK_ERROR(pdata->irq_preview_gpio == -1, error_4, "preview_gpio is not defined\n");
	data->irq_preview = gpio_to_irq(pdata->irq_preview_gpio);

	data->irq_hinge = gpio_to_irq(pdata->irq_hinge_gpio);
	data->irq_photo = gpio_to_irq(pdata->irq_photo_gpio);
	data->irq_preview = gpio_to_irq(pdata->irq_preview_gpio);

	data->hinge_gpio = pdata->irq_hinge_gpio;
	data->photo_gpio = pdata->irq_photo_gpio;
	data->preview_gpio = pdata->irq_preview_gpio;
	data->bootcfg_gpio = pdata->bootcfg_gpio;
	data->reset_gpio = pdata->reset_gpio;
	data->active_polarity = pdata->active_polarity;

	dev_info(&client->dev, "active_polarity=%d\n", data->active_polarity);

	/* reset and bootcfg initial config depends on polarity */
	gpio_init_flags = data->active_polarity ? GPIOF_OUT_INIT_LOW : GPIOF_OUT_INIT_HIGH;

	err = -EINVAL;
	CHECK_ERROR(data->bootcfg_gpio == -1, error_4, "bootcfg_gpio is not defined\n");
	err = devm_gpio_request_one(data->dev, data->bootcfg_gpio, gpio_init_flags, "mcu_bootcfg0");
	CHECK_ERROR(err, error_4, "Failed to request bootcfg gpio (%d): %d\n", data->bootcfg_gpio, err);

	err = -EINVAL;
	CHECK_ERROR(data->reset_gpio == -1, error_5, "reset_gpio is not defined\n");
	err = devm_gpio_request_one(data->dev, data->reset_gpio, gpio_init_flags, "mcu_reset");
	CHECK_ERROR(err, error_5, "Failed to request reset gpio (%d): %d\n", data->reset_gpio, err);

	err = -EINVAL;
	CHECK_ERROR(data->hinge_gpio == -1, error_6, "hinge_gpio is not defined\n");
	err = devm_gpio_request_one(data->dev, data->hinge_gpio, GPIOF_IN, "mcu_hinge");
	CHECK_ERROR(err, error_6, "Failed to request hinge gpio (%d): %d\n", data->hinge_gpio, err);

	err = -EINVAL;
	CHECK_ERROR(data->photo_gpio == -1, error_7, "photo_gpio is not defined\n");
	err = devm_gpio_request_one(data->dev, data->photo_gpio, GPIOF_IN, "mcu_photo");
	CHECK_ERROR(err, error_7, "Failed to request photo gpio (%d): %d\n", data->photo_gpio, err);

	err = -EINVAL;
	CHECK_ERROR(data->preview_gpio == -1, error_8, "preview_gpio is not defined\n");
	err = devm_gpio_request_one(data->dev, data->preview_gpio, GPIOF_IN, "mcu_preview");
	CHECK_ERROR(err, error_8, "Failed to request preview gpio (%d): %d\n", data->preview_gpio, err);

	err = request_threaded_irq(data->irq_hinge, NULL, hinge_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_NO_SUSPEND,
			HINGE_NAME "-hinge", data);
	CHECK_ERROR(err, error_9, "Failed to register hinge irq %d (gpio %d): %d\n",
				data->irq_hinge, pdata->irq_hinge_gpio, err);

	err = request_threaded_irq(data->irq_photo, NULL, hinge_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_NO_SUSPEND,
			HINGE_NAME "-photo", data);
	CHECK_ERROR(err, error_10, "Failed to register photo irq %d (gpio %d): %d\n",
				data->irq_photo, pdata->irq_photo_gpio, err);

	err = request_threaded_irq(data->irq_preview, NULL, hinge_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_NO_SUSPEND,
			HINGE_NAME "-preview", data);
	CHECK_ERROR(err, error_11, "Failed to register preview irq %d (gpio %d): %d\n",
				data->irq_preview, pdata->irq_preview_gpio, err);

	err = sysfs_create_group(&data->dev->kobj, &hinge_attr_group);
	CHECK_ERROR(err, error_12, "Failed to create sysfs nodes: %d\n", err);

	/* read initial values of bootcfg and reset */
	bootcfg_value = hinge_get_bootcfg(data);
	reset_value = hinge_get_reset(data);

	dev_warn(&client->dev, "Initial values: bootcfg=%d reset=%d\n", bootcfg_value, reset_value);

	retries = 10;
	do {
		hinge_determine_status(data);
	} while (data->mcu_status == MCU_STATUS_ERROR && retries--);

	/* if the MCU still can't be reached, try resetting it between attempts */
	hinge_set_bootcfg(data, false);

	retries = 5;
	while (data->mcu_status == MCU_STATUS_ERROR && retries--) {
		hinge_set_reset(data, true);
		usleep_range(10000, 10500);
		hinge_set_reset(data, false);
		usleep_range(10000, 10500);

		hinge_determine_status(data);
	}

	if (data->mcu_status == MCU_STATUS_RESET) {
		dev_info(data->dev, "MCU in reset at boot\n");

		/* make sure the MCU is not left in reset */
		retries = 10;
		do {
			hinge_set_bootcfg(data, false);
			hinge_set_reset(data, false);

			usleep_range(10000, 10500);
			hinge_determine_status(data);
		} while ((data->mcu_status == MCU_STATUS_RESET || data->mcu_status == MCU_STATUS_ERROR)
				&& retries--);

		if (data->mcu_status == MCU_STATUS_RESET)
			dev_err(data->dev, "Failed to take the MCU out of reset\n");
	}

	if (data->mcu_status == MCU_STATUS_BOOTLOADER) {
		dev_info(data->dev, "MCU in bootloader at boot\n");

		hinge_bl_jumpstart(data);

		usleep_range(10000, 10500);
		hinge_determine_status(data);

		if (data->mcu_status != MCU_STATUS_RUNNING)
			dev_err(data->dev, "Failed to jumpstart the MCU\n");
	}

	if (data->mcu_status == MCU_STATUS_RUNNING) {
		err = hinge_read_reg(data, REG_FIRMWARE_REVISION, &value);
		if (err)
			dev_warn(&client->dev, "WARNING: Failed to read MCU firmware revision: %d\n", err);
		else
			dev_info(&client->dev, "Firmware revision is %08x\n", value);

		/* dump programmed thresholds */
		hinge_read_reg(data, REG_HALL_HINGE_CLOSED, &value);
		dev_info(data->dev, "hinge_closed=%08x\n", value);

		hinge_read_reg(data, REG_HALL_HINGE_OPENED, &value);
		dev_info(data->dev, "hinge_opened=%08x\n", value);

		hinge_read_reg(data, REG_HALL_PREVIEW_PRESSED, &value);
		dev_info(data->dev, "preview_pressed=%08x\n", value);

		hinge_read_reg(data, REG_HALL_PREVIEW_RELEASED, &value);
		dev_info(data->dev, "preview_released=%08x\n", value);

		hinge_read_reg(data, REG_HALL_PHOTO_PRESSED, &value);
		dev_info(data->dev, "photo_pressed=%08x\n", value);

		hinge_read_reg(data, REG_HALL_PHOTO_RELEASED, &value);
		dev_info(data->dev, "photo_released=%08x\n", value);

		/* send initial hinge state event */
		handle_hinge_event(data, true);
	} else if (data->mcu_status == MCU_STATUS_ERROR) {
		dev_info(data->dev, "MCU in unknown state at boot, holding it in reset for safety.\n");
		hinge_set_bootcfg(data, false);
		hinge_set_reset(data, true);
	}

	dev_info(&client->dev, "%s: exit\n", __func__);

	return 0;

/*
error_13:
	sysfs_remove_group(&data->dev->kobj, &hinge_attr_group);
*/
error_12:
	free_irq(data->irq_preview, data);
error_11:
	free_irq(data->irq_photo, data);
error_10:
	free_irq(data->irq_hinge, data);
error_9:
	devm_gpio_free(data->dev, data->preview_gpio);
error_8:
	devm_gpio_free(data->dev, data->photo_gpio);
error_7:
	devm_gpio_free(data->dev, data->hinge_gpio);
error_6:
	devm_gpio_free(data->dev, data->reset_gpio);
error_5:
	devm_gpio_free(data->dev, data->bootcfg_gpio);
error_4:
	input_unregister_device(data->idev);
error_3:
	input_free_device(data->idev);
error_2:
	i2c_set_clientdata(client, NULL);
	wake_lock_destroy(&data->wakelock);
	kfree(data);
error_1:
	dev_info(&client->dev, "%s: error: %d\n", __func__, err);

	return err;
}

static int hinge_remove(struct i2c_client *client)
{
	struct hinge_data *data = i2c_get_clientdata(client);

	sysfs_remove_group(&data->dev->kobj, &hinge_attr_group);

	free_irq(data->irq_preview, data);
	free_irq(data->irq_photo, data);
	free_irq(data->irq_hinge, data);

	/* devm gpios are automatically released */

	input_unregister_device(data->idev);
	input_free_device(data->idev);

	i2c_set_clientdata(client, NULL);
	kfree(data);

	return 0;
}

static const struct i2c_device_id hinge_id[] = {
	{ HINGE_NAME, 0 },
};
MODULE_DEVICE_TABLE(i2c, hinge_id);

static struct i2c_driver hinge_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = HINGE_NAME,
	},
	.id_table = hinge_id,
	.probe = hinge_probe,
	.remove = hinge_remove,
	.suspend = hinge_suspend,
	.resume = hinge_resume,
};

module_i2c_driver(hinge_i2c_driver);

MODULE_AUTHOR("Corey Tabaka <eieio@google.com>");
MODULE_DESCRIPTION("Hinge MCU I2C control driver");
MODULE_LICENSE("GPL");

