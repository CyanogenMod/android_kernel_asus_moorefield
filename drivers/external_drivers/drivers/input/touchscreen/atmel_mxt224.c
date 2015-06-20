/*
 *  Atmel maXTouch Touchscreen Controller Driver
 *
 *
 *  Copyright (C) 2010 Atmel Corporation
 *  Copyright (C) 2010 Ulf Samuelsson (ulf@atmel.com)
 *  Copyright (C) 2009 Raphael Derosso Pereira <raphaelpereira@gmail.com>
 *  Contains changes by Wind River Systems, 2010-09-29
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *  Modified by Richard Zhu(jianxin.zhu@borqs.com) 05/19/2011
 */

/*
 *
 * Driver for Atmel maXTouch family of touch controllers.
 *
 */
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/debugfs.h>
#include <linux/cdev.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/atmel_mxt224.h>
#include <linux/early_suspend_sysfs.h>

/* Routines for memory access within a 16 bit address space */

static int mxt_read_block(struct i2c_client *client, u16 addr, u16 length,
			  u8 *value);
static int mxt_write_byte(struct i2c_client *client, u16 addr, u8 value);
static int mxt_write_block(struct i2c_client *client, u16 addr, u16 length,
			   u8 *value);

#define DRIVER_VERSION "0.9a"
#define MXT_CONFIG_VERSION 8

#define MXT_BACKNVM_DELAY 25 /* ms */
#define MXT_RESET_DELAY   65 /* ms */

#define MXT_RECALIB_NEED 0
#define MXT_RECALIB_NG	 1
#define MXT_RECALIB_DONE 2

/*
 * Designware i2c controller can only support up to 32 byte block read/write,
 * max224 uses 16bit address, this leaves 30 bytes for read/write
 */
#define I2C_MAX_BLKSZ 30

static int debug = DEBUG_TRACE;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Activate debugging output");

static struct mxt_data *mxt_es;

/* Device Info descriptor */
/* Parsed from maXTouch "Id information" inside device */
struct mxt_device_info {
	u8   family_id;
	u8   variant_id;
	u8   major;
	u8   minor;
	u8   build;
	u8   num_objs;
	u8   x_size;
	u8   y_size;
	char family_name[16];	 /* Family name */
	char variant_name[16];    /* Variant name */
	u16  num_nodes;           /* Number of sensor nodes */
};

/* object descriptor table, parsed from maXTouch "object table" */
struct mxt_object {
	u16 chip_addr;
	u8  type;
	u8  size;
	u8  instances;
	u8  num_report_ids;
};


/* Mapping from report id to object type and instance */
struct report_id_map {
	u8  object;
	u8  instance;
/*
 * This is the first report ID belonging to object. It enables us to
 * find out easily the touch number: each touch has different report
 * ID (which are assigned to touches in increasing order). By
 * subtracting the first report ID from current, we get the touch
 * number.
 */
	u8  first_rid;
};

struct mxt_finger {
	int status;
	int x;
	int y;
	int area;
};

/* Driver datastructure */
struct mxt_data {
	struct i2c_client    *client;
	struct mutex	     dev_mutex;
	struct input_dev     *touch_input;
	struct input_dev     *key_input;
	char                 touch_phys_name[32];
	char                 key_phys_name[32];
	int                  irq;

	u16                  last_read_addr;
	bool                 new_msgs;

	int                  valid_irq_counter;
	int                  invalid_irq_counter;
	int                  irq_counter;
	int                  message_counter;
	int                  read_fail_counter;


	int                  bytes_to_read;

	u8                   xpos_format;
	u8                   ypos_format;

	/* set if need to report input_sync */
	u8                   need_report_sync;

	u8                   numtouch;
	struct mxt_finger    finger[MXT_MAX_NUM_TOUCHES];

	struct mxt_device_info	device_info;

	u32		     info_block_crc;
	u32                  configuration_crc;
	u16                  report_id_count;
	struct report_id_map *rid_map;
	struct mxt_object    *object_table;

	u16                  msg_proc_addr;
	u8                   message_size;

	u16                  max_x_val;
	u16                  max_y_val;
	u16                  orientation;

	void                 (*init_hw)(void);
	void                 (*exit_hw)(void);
	u8                   (*valid_interrupt)(void);
	u8                   (*read_chg)(void);

	u8                   T7[3];
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend es;
#endif
	bool                 suspended;

#ifdef DEBUG
	u8                   *last_message;
	/* debugfs variables */
	struct dentry        *debug_dir;
	int                  current_debug_datap;

	struct mutex         debug_mutex;
	u16                  *debug_data;

	/* Character device variables */
	struct cdev          cdev;
	struct cdev          cdev_messages;  /* 2nd Char dev for messages */
	dev_t                dev_num;
	struct class         *mxt_class;
	struct mutex         msg_mutex;
	int                  msg_buffer_startp, msg_buffer_endp;
	/* Message buffer & pointers */
	char                 *messages;
	u16                  address_pointer;
	bool                 valid_ap;
#endif

	/* Put only non-touch messages to buffer if this is set */
	char                 nontouch_msg_only;

	int                  prev_key;

	int                  mxt_intr_gpio;
	int                  mxt_reset_gpio;

	unsigned long	    timestamp;
	int		    calibration_confirm;
};

/*
 * This struct is used for i2c transfers.
 */
struct mxt_i2c_byte_transfer {
	__le16 le_addr;
	u8     data;
} __attribute__ ((packed));

#define I2C_RETRY_COUNT 5
#define I2C_PAYLOAD_SIZE 254

/*
 * Check whether we have multi-touch enabled kernel; if not, report just the
 * first touch (on mXT224, the maximum is 10 simultaneous touches).
 * Because just the 1st one is reported, it might seem that the screen is not
 * responding to touch if the first touch is removed while the screen is being
 * touched by another finger, so beware.
 *
 * TODO: investigate if there is any standard set of input events that upper
 * layers are expecting from a touchscreen? These can however be different for
 * different platforms, and customers may have different opinions too about
 * what should be interpreted as right-click, for example.
 *
 */

static const u8	*obj_typ_name[] = {
	[0]  = "Reserved",
	[5]  = "GEN_MESSAGEPROCESSOR_T5",
	[6]  = "GEN_COMMANDPROCESSOR_T6",
	[7]  = "GEN_POWERCONFIG_T7",
	[8]  = "GEN_ACQUIRECONFIG_T8",
	[9]  = "TOUCH_MULTITOUCHSCREEN_T9",
	[15] = "TOUCH_KEYARRAY_T15",
	[17] = "SPT_COMMSCONFIG_T18",
	[19] = "SPT_GPIOPWM_T19",
	[20] = "PROCI_GRIPFACESUPPRESSION_T20",
	[22] = "PROCG_NOISESUPPRESSION_T22",
	[23] = "TOUCH_PROXIMITY_T23",
	[24] = "PROCI_ONETOUCHGESTUREPROCESSOR_T24",
	[25] = "SPT_SELFTEST_T25",
	[27] = "PROCI_TWOTOUCHGESTUREPROCESSOR_T27",
	[28] = "SPT_CTECONFIG_T28",
	[37] = "DEBUG_DIAGNOSTICS_T37",
	[38] = "SPT_USER_DATA_T38",
	[40] = "PROCI_GRIPSUPPRESSION_T40",
	[41] = "PROCI_PALMSUPPRESSION_T41",
	[42] = "PROCI_FACESUPPRESSION_T42",
	[43] = "SPT_DIGITIZER_T43",
	[44] = "SPT_MESSAGECOUNT_T44",
};

void mxt_config_init(struct mxt_data *mxt)
{
	/*
	 * Please refer to "mXT224 Firmware 2.x Protocol Guide" for
	 * the meaning of the config data for each object.
	 */
	u8 v20_T7[]  = { 32, 10, 50 };
	u8 v20_T8[]  = { 6, 0, 5, 5, 0, 0, 5, 50, 5, 192 };
	u8 v20_T9[]  = { 143, 0, 0, 18, 11, 1, 32, 65, 2, 1, 0, 3, 5,
			 0, 2, 10, 35, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			 0, 64, 0, 20, 8 };
	u8 v20_T15[] = { 131, 0, 11, 11, 1, 1, 48, 80, 3, 0, 0 };
	u8 v20_T18[] = { 0, 0 };
	u8 v20_T19[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	u8 v20_T20[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	u8 v20_T22[] = { 5, 0, 0, 0, 0, 0, 0, 0, 30, 0, 1, 12, 17, 22, 255,
			 255, 0 };
	u8 v20_T23[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	u8 v20_T24[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			 0, 0, 0 };
	u8 v20_T25[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
	u8 v20_T28[] = { 0, 0, 2, 16, 32, 30 };

	u8 i = 0, max_objs = 0;
	u16 addr;
	struct mxt_object *obj_index;

	dev_dbg(&mxt->client->dev, "In function %s", __func__);
	max_objs = mxt->device_info.num_objs;
	obj_index = mxt->object_table;

	for (i = 0; i < max_objs; i++) {
		addr = obj_index->chip_addr;
		switch (obj_index->type) {
		case MXT_GEN_POWERCONFIG_T7:
			mxt_write_block(mxt->client, addr,
					sizeof(v20_T7), v20_T7);
			break;
		case MXT_GEN_ACQUIRECONFIG_T8:
			mxt_write_block(mxt->client, addr,
					sizeof(v20_T8), v20_T8);
			break;
		case MXT_TOUCH_MULTITOUCHSCREEN_T9:
			if (v20_T9[MXT_ADR_T9_NUMTOUCH] != mxt->numtouch)
				v20_T9[MXT_ADR_T9_NUMTOUCH] = mxt->numtouch;
			mxt_write_block(mxt->client, addr,
					sizeof(v20_T9), v20_T9);
			dev_dbg(&mxt->client->dev, "init multitouch object");
			break;
		case MXT_TOUCH_KEYARRAY_T15:
			mxt_write_block(mxt->client, addr,
					sizeof(v20_T15), v20_T15);
			break;
		case MXT_SPT_COMCONFIG_T18:
			mxt_write_block(mxt->client, addr,
					sizeof(v20_T18), v20_T18);
			break;
		case MXT_SPT_GPIOPWM_T19:
			mxt_write_block(mxt->client, addr,
					sizeof(v20_T19), v20_T19);
			break;
		case MXT_PROCI_GRIPFACESUPPRESSION_T20:
			mxt_write_block(mxt->client, addr,
					sizeof(v20_T20), v20_T20);
			break;
		case MXT_PROCG_NOISESUPPRESSION_T22:
			mxt_write_block(mxt->client, addr,
					sizeof(v20_T22), v20_T22);
			break;
		case MXT_TOUCH_PROXIMITY_T23:
			mxt_write_block(mxt->client, addr,
					sizeof(v20_T23), v20_T23);
			break;
		case MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24:
			mxt_write_block(mxt->client, addr,
					sizeof(v20_T24), v20_T24);
			break;
		case MXT_SPT_SELFTEST_T25:
			mxt_write_block(mxt->client, addr,
					sizeof(v20_T25), v20_T25);
			break;
		case MXT_SPT_CTECONFIG_T28:
			mxt_write_block(mxt->client, addr,
					sizeof(v20_T28), v20_T28);
			break;
		default:
			break;
		}
		obj_index++;
	}
	dev_dbg(&mxt->client->dev, "config init Done.");
}

/* Returns object address in mXT chip, or zero if object is not found */
static u16 get_object_address(uint8_t object_type,
			      uint8_t instance,
			      struct mxt_object *object_table,
			      int max_objs)
{
	uint8_t object_table_index = 0;
	uint8_t address_found = 0;
	uint16_t address = 0;
	struct mxt_object *obj;

	while ((object_table_index < max_objs) && !address_found) {
		obj = &object_table[object_table_index];
		if (obj->type == object_type) {
			address_found = 1;
			/* Are there enough instances defined in the FW? */
			if (obj->instances >= instance) {
				address = obj->chip_addr +
					  (obj->size + 1) * instance;
			} else {
				break;
			}
		}
		object_table_index++;
	}
	return address;
}

#ifdef DEBUG
/* Writes the address pointer (to set up following reads). */
static int mxt_write_ap(struct mxt_data *mxt, u16 ap)
{
	struct i2c_client *client;
	int err;
	__le16	le_ap = cpu_to_le16(ap);
	client = mxt->client;
	if (mxt != NULL)
		mxt->last_read_addr = -1;
	if (i2c_master_send(client, (u8 *) &le_ap, 2) == 2) {
		dev_dbg(&mxt->client->dev, "Address pointer set to %d\n", ap);
		err = 0;
	} else {
		dev_dbg(&mxt->client->dev, "Error writing address pointer!\n");
		err = -EIO;
	}
	return err;
}

/* Reads a block of bytes from current address from mXT chip. */
static int mxt_read_block_wo_addr(struct i2c_client *client,
			   u16 length,
			   u8 *value)
{
	int err;
	struct mxt_data *mxt;

	mxt = i2c_get_clientdata(client);
	if  (i2c_master_recv(client, value, length) == length) {
		dev_dbg(&client->dev, "I2C block read ok\n");
		err = length;
	} else {
		dev_dbg(&client->dev, "I2C block read failed\n");
		err = -EIO;
	}
	return err;
}

ssize_t debug_data_read(struct mxt_data *mxt, char *buf, size_t count,
			loff_t *ppos, u8 debug_command){
	int i;
	u16 *data;
	u16 diagnostics_reg;
	int offset = 0;
	int size;
	int read_size;
	int error;
	char *buf_start;
	u16 debug_data_addr;
	u16 page_address;
	u8 page;
	u8 debug_command_reg;

	data = mxt->debug_data;
	if (data == NULL)
		return -EIO;

	/* If first read after open, read all data to buffer. */
	if (mxt->current_debug_datap == 0) {

		diagnostics_reg =
			get_object_address(MXT_GEN_COMMANDPROCESSOR_T6,
			0,
			mxt->object_table,
			mxt->device_info.num_objs) + MXT_ADR_T6_DIAGNOSTIC;

		if (count > (mxt->device_info.num_nodes * 2))
			count = mxt->device_info.num_nodes;

		debug_data_addr =
			get_object_address(MXT_DEBUG_DIAGNOSTIC_T37,
			0,
			mxt->object_table,
			mxt->device_info.num_objs) + MXT_ADR_T37_DATA;

		page_address =
			get_object_address(MXT_DEBUG_DIAGNOSTIC_T37,
			0,
			mxt->object_table,
			mxt->device_info.num_objs) + MXT_ADR_T37_PAGE;

		error = mxt_read_block(mxt->client, page_address, 1, &page);
		if (error < 0)
			return error;

		dev_dbg(&mxt->client->dev, "debug data page = %d\n", page);

		while (page != 0) {
			error = mxt_write_byte(mxt->client,
					diagnostics_reg,
					MXT_CMD_T6_PAGE_DOWN);
			if (error < 0)
				return error;
			/* Wait for command to be handled; when it has, the
			   register will be cleared. */
			debug_command_reg = 1;
			while (debug_command_reg != 0) {
				error = mxt_read_block(mxt->client,
						diagnostics_reg, 1,
						&debug_command_reg);
				if (error < 0)
					return error;
				dev_dbg(&mxt->client->dev,
					"Waiting for debug diag command "
					"to propagate...\n");

			}
			error = mxt_read_block(mxt->client, page_address, 1,
					&page);
			if (error < 0)
				return error;

			dev_dbg(&mxt->client->dev,
					"debug data page = %d\n", page);
		}

		/*
		 * Lock mutex to prevent writing some unwanted data to debug
		 * command register. User can still write through the char
		 * device interface though. TODO: fix?
		 */
		mutex_lock(&mxt->debug_mutex);
		/* Configure Debug Diagnostics object to show deltas/refs */
		error = mxt_write_byte(mxt->client, diagnostics_reg,
				debug_command);

		/* Wait for command to be handled; when it has, the
		 * register will be cleared. */
		debug_command_reg = 1;
		while (debug_command_reg != 0) {
			error = mxt_read_block(mxt->client,
					diagnostics_reg, 1,
					&debug_command_reg);
			if (error < 0)
				goto error;
			dev_dbg(&mxt->client->dev,
				"Waiting for debug diag command "
				"to propagate...\n");

		}

		if (error < 0) {
			printk(KERN_WARNING
				"Error writing to maXTouch device!\n");
			goto error;
		}

		size = mxt->device_info.num_nodes * sizeof(u16);

		while (size > 0) {

			read_size = size > 128 ? 128 : size;

			dev_dbg(&mxt->client->dev,
				"Debug data read loop, reading %d bytes...\n",
				read_size);
			error = mxt_read_block(mxt->client,
					       debug_data_addr,
					       read_size,
					       (u8 *) &data[offset]);
			if (error < 0) {
				printk(KERN_WARNING
				       "Error reading debug data\n");
				goto error;
			}
			offset += read_size/2;
			size -= read_size;

			/* Select next page */
			error = mxt_write_byte(mxt->client, diagnostics_reg,
					MXT_CMD_T6_PAGE_UP);
			if (error < 0) {
				printk(KERN_WARNING
					"Error writing to maXTouch device!\n");
				goto error;
			}
		}
		mutex_unlock(&mxt->debug_mutex);
	}

	buf_start = buf;
	i = mxt->current_debug_datap;

	while (((buf - buf_start) < (count - 6)) &&
		(i < mxt->device_info.num_nodes)) {

		mxt->current_debug_datap++;
		if (debug_command == MXT_CMD_T6_REFERENCES_MODE)
			buf += sprintf(buf, "%d: %5d\n", i,
				       (u16) le16_to_cpu(data[i]));
		else if (debug_command == MXT_CMD_T6_DELTAS_MODE)
			buf += sprintf(buf, "%d: %5d\n", i,
				       (s16) le16_to_cpu(data[i]));
		i++;
	}

	return buf - buf_start;
error:
	mutex_unlock(&mxt->debug_mutex);
	return error;
}

ssize_t deltas_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	return debug_data_read(file->private_data, buf, count, ppos,
			       MXT_CMD_T6_DELTAS_MODE);
}

ssize_t refs_read(struct file *file, char *buf, size_t count,
			loff_t *ppos)
{
	return debug_data_read(file->private_data, buf, count, ppos,
			       MXT_CMD_T6_REFERENCES_MODE);
}

int debug_data_open(struct inode *inode, struct file *file)
{
	struct mxt_data *mxt;
	int i;

	mxt = inode->i_private;
	if (mxt == NULL)
		return -EIO;

	mxt->current_debug_datap = 0;
	mxt->debug_data = kmalloc(mxt->device_info.num_nodes * sizeof(u16),
				  GFP_KERNEL);
	if (mxt->debug_data == NULL)
		return -ENOMEM;


	for (i = 0; i < mxt->device_info.num_nodes; i++)
		mxt->debug_data[i] = 7777;


	file->private_data = mxt;
	return 0;
}

int debug_data_release(struct inode *inode, struct file *file)
{
	struct mxt_data *mxt;
	mxt = file->private_data;
	kfree(mxt->debug_data);
	return 0;
}

static const struct file_operations delta_fops = {
	.owner = THIS_MODULE,
	.open = debug_data_open,
	.release = debug_data_release,
	.read = deltas_read,
};

static const struct file_operations refs_fops = {
	.owner = THIS_MODULE,
	.open = debug_data_open,
	.release = debug_data_release,
	.read = refs_read,
};


int mxt_memory_open(struct inode *inode, struct file *file)
{
	struct mxt_data *mxt;
	mxt = container_of(inode->i_cdev, struct mxt_data, cdev);
	if (mxt == NULL)
		return -EIO;
	file->private_data = mxt;
	return 0;
}

int mxt_message_open(struct inode *inode, struct file *file)
{
	struct mxt_data *mxt;
	mxt = container_of(inode->i_cdev, struct mxt_data, cdev_messages);
	if (mxt == NULL)
		return -EIO;
	file->private_data = mxt;
	return 0;
}


ssize_t mxt_memory_read(struct file *file, char *buf, size_t count,
			loff_t *ppos)
{
	int i;
	struct mxt_data *mxt;
	mxt = file->private_data;

	if (mxt == NULL)
		return -EIO;

	if (mxt->valid_ap) {
		dev_dbg(&mxt->client->dev,
			"Reading %d bytes from current ap\n",
			(int) count);
		i = mxt_read_block_wo_addr(mxt->client, count, (u8 *) buf);
	} else {
		dev_dbg(&mxt->client->dev, "Address pointer changed since set;"
			  "writing AP (%d) before reading %d bytes",
			  mxt->address_pointer, (int) count);
		i = mxt_read_block(mxt->client, mxt->address_pointer, count,
				   buf);
	}

	return i;
}

ssize_t mxt_memory_write(struct file *file, const char *buf, size_t count,
			 loff_t *ppos)
{
	int i;
	int whole_blocks;
	int last_block_size;
	struct mxt_data *mxt;
	u16 address;
	mxt = file->private_data;
	address = mxt->address_pointer;

	dev_dbg(&mxt->client->dev, "mxt_memory_write entered\n");
	whole_blocks = count / I2C_PAYLOAD_SIZE;
	last_block_size = count % I2C_PAYLOAD_SIZE;

	for (i = 0; i < whole_blocks; i++) {
		dev_dbg(&mxt->client->dev, "About to write to %d...",
			address);
		mxt_write_block(mxt->client, address, I2C_PAYLOAD_SIZE,
				(u8 *) buf);
		address += I2C_PAYLOAD_SIZE;
		buf += I2C_PAYLOAD_SIZE;
	}

	mxt_write_block(mxt->client, address, last_block_size, (u8 *) buf);

	return count;
}

static int mxt_ioctl(struct inode *inode, struct file *file,
		     unsigned int cmd, unsigned long arg)
{
	int retval;
	struct mxt_data *mxt;
	u8 data[6];
	u16 base_addr;

	retval = 0;
	mxt = file->private_data;

	if (mxt == NULL)
		return -1;

	switch (cmd) {
	case MXT_SET_ADDRESS_IOCTL:
		retval = mxt_write_ap(mxt, (u16) arg);
		if (retval >= 0) {
			mxt->address_pointer = (u16) arg;
			mxt->valid_ap = 1;
		}
		break;
	case MXT_RESET_IOCTL:
		base_addr = get_object_address(MXT_GEN_COMMANDPROCESSOR_T6,
			0,
			mxt->object_table,
			mxt->device_info.num_objs) + MXT_ADR_T6_RESET;

		retval = mxt_write_byte(mxt->client, base_addr, 1);
		break;

	case MXT_CALIBRATE_IOCTL:
		base_addr = get_object_address(MXT_GEN_COMMANDPROCESSOR_T6,
			0,
			mxt->object_table,
			mxt->device_info.num_objs) + MXT_ADR_T6_CALIBRATE;

		retval = mxt_write_byte(mxt->client, base_addr, 1);
		break;

	case MXT_BACKUP_IOCTL:
		base_addr = get_object_address(MXT_GEN_COMMANDPROCESSOR_T6,
			0,
			mxt->object_table,
			mxt->device_info.num_objs) + MXT_ADR_T6_BACKUPNV;

		retval = mxt_write_byte(mxt->client,
				base_addr, MXT_CMD_T6_BACKUP);

		break;

	case MXT_NONTOUCH_MSG_IOCTL:
		mxt->nontouch_msg_only = 1;
		break;
	case MXT_ALL_MSG_IOCTL:
		mxt->nontouch_msg_only = 0;
		break;
	case MXT_SELF_TEST_IOCTL:

		base_addr = get_object_address(MXT_SPT_SELFTEST_T25,
			0,
			mxt->object_table,
			mxt->device_info.num_objs) + MXT_ADR_T25_CTRL;

		retval = mxt_write_byte(mxt->client, base_addr, 0x03);

		if (retval < 0)
			return -1;

		base_addr = get_object_address(MXT_SPT_SELFTEST_T25,
			0,
			mxt->object_table,
			mxt->device_info.num_objs) + MXT_ADR_T25_CMD;

		retval = mxt_write_byte(mxt->client, base_addr,
			MXT_MSGR_T25_RUN_ALL_TESTS);

		if (retval < 0)
			return -1;

		/* Read back to verify update. */
		memset(&data[0], 0x00, sizeof(data));

		base_addr = get_object_address(MXT_SPT_SELFTEST_T25,
			0,
			mxt->object_table,
			mxt->device_info.num_objs);

		retval = mxt_read_block(mxt->client, base_addr,
				sizeof(data),
				(u8 *)&data);
		if (retval < 0)
			return -1;
		else if ((retval > 0) && (data[1] !=
				MXT_MSGR_T25_RUN_ALL_TESTS))
			return -1;
		retval = 0;
		break;
	default:
		return -1;
	}
	return retval;
}

/*
 * Copies messages from buffer to user space.
 *
 * NOTE: if less than (mxt->message_size * 5 + 1) bytes requested,
 * this will return 0!
 *
 */
ssize_t mxt_message_read(struct file *file, char *buf, size_t count,
			 loff_t *ppos)
{
	int i;
	struct mxt_data *mxt;
	char *buf_start;
	mxt = file->private_data;
	if (mxt == NULL)
		return -EIO;
	buf_start = buf;

	mutex_lock(&mxt->msg_mutex);
	/* Copy messages until buffer empty, or 'count' bytes written */
	while ((mxt->msg_buffer_startp != mxt->msg_buffer_endp) &&
	       ((buf - buf_start) < (count - 5 * mxt->message_size - 1))) {

		for (i = 0; i < mxt->message_size; i++) {
			buf += sprintf(buf, "[%2X] ",
				*(mxt->messages + mxt->msg_buffer_endp *
					mxt->message_size + i));
		}
		buf += sprintf(buf, "\n");
		if (mxt->msg_buffer_endp < MXT_MESSAGE_BUFFER_SIZE)
			mxt->msg_buffer_endp++;
		else
			mxt->msg_buffer_endp = 0;
	}
	mutex_unlock(&mxt->msg_mutex);
	return buf - buf_start;
}

static const struct file_operations mxt_message_fops = {
	.owner = THIS_MODULE,
	.open = mxt_message_open,
	.read = mxt_message_read,
};

static const struct file_operations mxt_memory_fops = {
	.owner = THIS_MODULE,
	.open = mxt_memory_open,
	.read = mxt_memory_read,
	.write = mxt_memory_write,
	.ioctl = mxt_ioctl,
};
#endif

/* Calculates the 24-bit CRC sum. */
static u32 CRC_24(u32 crc, u8 byte1, u8 byte2)
{
	static const u32 crcpoly = 0x80001B;
	u32 result;
	u32 data_word;

	data_word = ((((u16) byte2) << 8u) | byte1);
	result = ((crc << 1u) ^ data_word);
	if (result & 0x1000000)
		result ^= crcpoly;
	return result;
}

static void mxt_reset(struct mxt_data *mxt)
{
	u16 base_addr;

	dev_dbg(&mxt->client->dev, "mxt reset\n");

	base_addr = get_object_address(MXT_GEN_COMMANDPROCESSOR_T6,
			0,
			mxt->object_table,
			mxt->device_info.num_objs) + MXT_ADR_T6_RESET;

	mxt_write_byte(mxt->client, base_addr, 1);
	msleep(MXT_RESET_DELAY);
}

static void mxt_gpio_reset(struct mxt_data *mxt)
{
	/*
	 * When RESET pin asserted, it returns the device to its reset state.
	 * The RESET pin must be asserted low for at least 90 ns to cause a
	 * reset. After releasing the RESET pin the device takes 40 ms before
	 * it is ready to start communications.
	 */
	gpio_set_value(mxt->mxt_reset_gpio, 0);
	usleep_range(1000, 2000);
	gpio_set_value(mxt->mxt_reset_gpio, 1);
	msleep(40);
}

/*
 * Reads a block of bytes from given address from mXT chip. If we are
 * reading from message window, and previous read was from message window,
 * there's no need to write the address pointer: the mXT chip will
 * automatically set the address pointer back to message window start.
 */

static int mxt_read_block(struct i2c_client *client,
			  u16 addr, u16 length, u8 *value)
{
	struct mxt_data *mxt;
	int ret = 0;
	int readlen;
	__le16	le_addr;

	mxt = i2c_get_clientdata(client);
	if (mxt &&
	    (mxt->last_read_addr == addr) &&
	    (addr == mxt->msg_proc_addr)) {
		ret = i2c_master_recv(client, value, length);
		if (ret < 0)
			goto out_err;
		return 0;
	}

	dev_dbg(&client->dev, "Writing address pointer 0x%x & reading %d bytes"
		" in on i2c transaction...\n", addr, length);

	while (length > 0) {
		readlen = length > I2C_MAX_BLKSZ ? I2C_MAX_BLKSZ : length;
		/*
		 * block read must be split into two i2c transactions:
		 * 1st: set the 16bit address pointer
		 * 2nd: read data
		 *
		 * need STOP on the i2c bus between write and read:
		 */
		le_addr = cpu_to_le16(addr);
		ret = i2c_master_send(client, (u8 *)&le_addr, sizeof(le_addr));
		if (ret < 0)
			goto out_err;

		ret = i2c_master_recv(client, value, readlen);
		if (ret < 0)
			goto out_err;

		if (mxt)
			mxt->last_read_addr = addr;

		addr += readlen;
		value += readlen;
		length -= readlen;
	}

	return 0;

out_err:
	if (mxt)
		mxt->last_read_addr = -1;
	return ret;
}


/* Writes one byte to given address in mXT chip. */

static int mxt_write_byte(struct i2c_client *client, u16 addr, u8 value)
{

	struct mxt_data *mxt;
	struct mxt_i2c_byte_transfer i2c_byte_transfer;
	int err;

	mxt = i2c_get_clientdata(client);
	if (mxt != NULL)
		mxt->last_read_addr = -1;
	i2c_byte_transfer.le_addr = cpu_to_le16(addr);
	i2c_byte_transfer.data = value;
	if  (i2c_master_send(client, (u8 *) &i2c_byte_transfer, 3) == 3)
		err = 0;
	else
		err = -EIO;
	return err;
}


/* Writes a block of bytes (max 256) to given address in mXT chip. */
static int mxt_write_block(struct i2c_client *client,
		    u16 addr,
		    u16 length,
		    u8 *value)
{
	int ret;
	int wlen;
	struct {
		__le16	le_addr;
		u8	data[I2C_MAX_BLKSZ];

	} i2c_block_transfer;

	struct mxt_data *mxt;
	dev_dbg(&client->dev, "Writing %d bytes to %d...", length, addr);
	if (length > 256)
		return -EINVAL;

	mxt = i2c_get_clientdata(client);
	if (mxt != NULL)
		mxt->last_read_addr = -1;

	while (length > 0) {
		wlen = length > I2C_MAX_BLKSZ ? I2C_MAX_BLKSZ : length;
		memcpy(i2c_block_transfer.data, value, wlen);
		i2c_block_transfer.le_addr = cpu_to_le16(addr);

		ret = i2c_master_send(client, (u8 *)&i2c_block_transfer,
				wlen + sizeof(i2c_block_transfer.le_addr));
		if (ret != wlen + sizeof(i2c_block_transfer.le_addr))
			return -EIO;

		value += wlen;
		addr += wlen;
		length -= wlen;
	}

	return length;
}

/* Calculates the CRC value for mXT infoblock. */
static int calculate_infoblock_crc(u32 *crc_result, u8 *data, int crc_area_size)
{
	u32 crc = 0;
	int i;

	pr_debug("atmel_mxt224 In function %s", __func__);
	for (i = 0; i < (crc_area_size - 1); i = i + 2)
		crc = CRC_24(crc, *(data + i), *(data + i + 1));
	/* If uneven size, pad with zero */
	if (crc_area_size & 0x0001)
		crc = CRC_24(crc, *(data + i), 0);
	/* Return only 24 bits of CRC. */
	*crc_result = (crc & 0x00FFFFFF);

	return 0;
}

static void report_mt(struct mxt_data *mxt)
{
	int i;
	struct mxt_finger *finger = mxt->finger;

	for (i = 0; i < mxt->numtouch; i++) {
		if (!finger[i].status)
			continue;

		input_mt_slot(mxt->touch_input, i);
		input_mt_report_slot_state(mxt->touch_input, MT_TOOL_FINGER,
				finger[i].status != MXT_MSGB_T9_RELEASE);
		if (finger[i].status != MXT_MSGB_T9_RELEASE) {
			input_report_abs(mxt->touch_input,
					ABS_MT_TOUCH_MAJOR, finger[i].area);
			input_report_abs(mxt->touch_input,
					ABS_MT_POSITION_X, finger[i].x);
			input_report_abs(mxt->touch_input,
					ABS_MT_POSITION_Y, finger[i].y);
		} else {
			finger[i].status = 0;
		}
	}
}

static void mxt_enable_autocalib(struct mxt_data *mxt)
{
	u16 addr;
	u8 antitouch_ext[] = { 5, 50, 5, 192 };

	addr = get_object_address(MXT_GEN_ACQUIRECONFIG_T8, 0,
				 mxt->object_table,
				 mxt->device_info.num_objs) + T8_CFG_ATCHCALST;
	mxt_write_block(mxt->client, addr,
			sizeof(antitouch_ext), antitouch_ext);
}

static void mxt_disable_autocalib(struct mxt_data *mxt)
{
	u16 addr;
	u8 antitouch_ext[] = { 255, 1, 0, 0 };


	addr = get_object_address(MXT_GEN_ACQUIRECONFIG_T8, 0,
				 mxt->object_table,
				 mxt->device_info.num_objs) + T8_CFG_ATCHCALST;
	mxt_write_block(mxt->client, addr,
			sizeof(antitouch_ext), antitouch_ext);
	dev_info(&mxt->client->dev, "auto-calibration disabled\n");
}

static void process_T9_message(u8 *message, struct mxt_data *mxt)
{
	struct input_dev *input;
	struct device *dev = &mxt->client->dev;
	struct mxt_finger *finger = mxt->finger;
	u8  status;
	u16 xpos = 0xFFFF;
	u16 ypos = 0xFFFF;
	u8  touch_size = 255;
	u8  touch_number;
	u8  amplitude;
	u8  report_id;
	input = mxt->touch_input;
	status = message[MXT_MSG_T9_STATUS];
	report_id = message[0];

	if (status & MXT_MSGB_T9_SUPPRESS) {
		/* Touch has been suppressed by grip/face */
		/* detection                              */
		dev_dbg(dev, "SUPRESS");
	} else {
		xpos = message[MXT_MSG_T9_XPOSMSB] * 16 +
			((message[MXT_MSG_T9_XYPOSLSB] >> 4) & 0xF);
		ypos = message[MXT_MSG_T9_YPOSMSB] * 16 +
			((message[MXT_MSG_T9_XYPOSLSB] >> 0) & 0xF);

		xpos >>= 2;
		ypos >>= 2;

		touch_number = message[MXT_MSG_REPORTID] -
			mxt->rid_map[report_id].first_rid;

		finger[touch_number].x = xpos;
		finger[touch_number].y = ypos;

		if (status & MXT_MSGB_T9_DETECT) {
			/*
			 * TODO: more precise touch size calculation?
			 * mXT224 reports the number of touched nodes,
			 * so the exact value for touch ellipse major
			 * axis length would be 2*sqrt(touch_size/pi)
			 * (assuming round touch shape).
			 */
			touch_size = message[MXT_MSG_T9_TCHAREA];
			touch_size = touch_size >> 2;
			if (!touch_size)
				touch_size = 1;
			finger[touch_number].area = touch_size;
			finger[touch_number].status = MXT_MSGB_T9_DETECT;
			if (status & MXT_MSGB_T9_AMP)
				/* Amplitude of touch has changed */
				amplitude = message[MXT_MSG_T9_TCHAMPLITUDE];

			dev_dbg(dev, "DETECT:%s%s%s%s",
				((status & MXT_MSGB_T9_PRESS) ? " PRESS" : ""),
				((status & MXT_MSGB_T9_MOVE) ? " MOVE" : ""),
				((status & MXT_MSGB_T9_AMP) ? " AMP" : ""),
				((status & MXT_MSGB_T9_VECTOR) ? " VECT" : ""));
		} else if (status & MXT_MSGB_T9_RELEASE) {
			dev_dbg(dev, "RELEASE");

			/* The previously reported touch has been removed.*/
			finger[touch_number].status = MXT_MSGB_T9_RELEASE;
		}

		dev_dbg(dev, "X=%d, Y=%d, touch number=%d, TOUCHSIZE=%d",
			xpos, ypos, touch_number, touch_size);
	}
}

void process_key_message(u8 *message, struct mxt_data *mxt)
{
	/*key up*/
	if (message[1] == 0 && message[2] == 0 && message[3] == 0 &&
						message[4] == 0) {
		if (mxt->prev_key == 0) {
			dev_dbg(&mxt->client->dev, "No previous key");
		} else {
			input_report_key(mxt->key_input, mxt->prev_key, 0);
			dev_dbg(&mxt->client->dev,
				 "Report key %d up", mxt->prev_key);
			mxt->prev_key = 0;
		}
		input_sync(mxt->key_input);
		return;
	}

	/*key down*/
	if (message[1] == 128 && message[2] == 1 && message[3] == 0 &&
					message[4] == 0) {
		input_report_key(mxt->key_input, KEY_BACK, 1);
		dev_dbg(&mxt->client->dev,
			"Report BACK(%d) key DOWN.", KEY_BACK);
		mxt->prev_key = KEY_BACK;
	} else if (message[1] == 128 && message[2] == 2 && message[3] == 0 &&
					message[4] == 0) {
		input_report_key(mxt->key_input, KEY_MENU, 1);
		dev_dbg(&mxt->client->dev,
			 "Report MENU(%d) key DOWN.", KEY_MENU);
		mxt->prev_key = KEY_MENU;
	} else if (message[1] == 128 && message[2] == 0 && message[3] == 4 &&
					message[4] == 0) {
		input_report_key(mxt->key_input, KEY_HOME, 1);
		dev_dbg(&mxt->client->dev,
			"Report HOME(%d) key DOWN.", KEY_HOME);
		mxt->prev_key = KEY_HOME;
	} else if (message[1] == 128 && message[2] == 0 && message[3] == 2 &&
					message[4] == 0) {
		input_report_key(mxt->key_input, KEY_SEARCH, 1);
		dev_dbg(&mxt->client->dev,
			"Report SEARCH(%d) key DOWN.", KEY_SEARCH);
		mxt->prev_key = KEY_SEARCH;
	}
	input_sync(mxt->key_input);
}

#define T37_TCH_DIAG_SZ 82
#define MXT_XSIZE	18

static void mxt_check_calibration(struct mxt_data *mxt)
{
	u8 data[T37_TCH_DIAG_SZ];
	u16 t6addr, t37addr;
	int i;
	int xlimit = 0;
	int touch_ch = 0, antitouch_ch = 0;
	struct device *dev = &mxt->client->dev;
	int ret;

	/* only select some point to check calibration */
	if (time_before(jiffies, mxt->timestamp + HZ / 25))
		return;

	memset(data, 0xff, sizeof(data));

	t6addr = get_object_address(MXT_GEN_COMMANDPROCESSOR_T6,
			0, mxt->object_table, mxt->device_info.num_objs);
	ret = mxt_write_byte(mxt->client, t6addr + MXT_ADR_T6_DIAGNOSTIC,
			     MXT_CMD_T6_TCH_DIAG);
	if (ret < 0) {
		dev_err(dev, "fail to set touch diagnostic command\n");
		return;
	}

	t37addr = get_object_address(MXT_DEBUG_DIAGNOSTIC_T37,
			0, mxt->object_table, mxt->device_info.num_objs);
	for (i = 0; i < 10; i++) {
		if (data[0] == MXT_CMD_T6_TCH_DIAG && data[1] == T37_PAGE_NUM0)
			break;
		/*
		 * Suggested by Atmel application note QTAN0070
		 * "Recovering from Palm Touches During Calibration"
		 */
		usleep_range(5000, 6000);
		mxt_read_block(mxt->client, t37addr, 2, data);
	}
	if (i == 10)
		dev_err(dev, "fail to get calib diagnostic data\n");

	mxt_read_block(mxt->client, t37addr, sizeof(data), data);
	if (data[0] == MXT_CMD_T6_TCH_DIAG && data[1] == T37_PAGE_NUM0) {
		xlimit = MXT_XSIZE << 1;
		for (i = 0; i < xlimit; i += 2) {
			touch_ch += __sw_hweight16(*(u16 *)&data[2 + i]);
			antitouch_ch += __sw_hweight16(*(u16 *)&data[42 + i]);
		}
	}
	mxt_write_byte(mxt->client,
		t6addr + MXT_ADR_T6_DIAGNOSTIC, MXT_CMD_T6_PAGE_UP);

	dev_dbg(dev, "touch channel:%d, anti-touch channel:%d\n",
		 touch_ch, antitouch_ch);

	if (touch_ch && antitouch_ch == 0) {
		if (mxt->calibration_confirm == 1 &&
		    time_after(jiffies, mxt->timestamp + HZ / 2)) {
			mxt->calibration_confirm = 2;
			dev_info(dev, "calibration confirmed\n");
			mxt_disable_autocalib(mxt);
		}
		if (mxt->calibration_confirm < 2)
			mxt->calibration_confirm = 1;
	} else if ((touch_ch - 25) <= antitouch_ch &&
		   (touch_ch || antitouch_ch)) {
		mxt->calibration_confirm = 0;
		/* since we enable auto calibration in firmware, so
		 * don't need to manually start calibration here
		 */
	}
	mxt->timestamp = jiffies;
}

static int process_message(u8 *message, u8 object, struct mxt_data *mxt)
{
	struct i2c_client *client;
	u8  status;
	u16 xpos = 0xFFFF;
	u16 ypos = 0xFFFF;
	u8  event;
	u8  direction;
	u16 distance;
	u8  length;
	u8  report_id;
	static u8 error_cond;
	client = mxt->client;
	length = mxt->message_size;
	report_id = message[0];

	if ((mxt->nontouch_msg_only == 0) ||
		((object != MXT_TOUCH_MULTITOUCHSCREEN_T9) &&
		(object != MXT_TOUCH_KEYARRAY_T15) &&
		(object != MXT_TOUCH_PROXIMITY_T23) &&
		(object != MXT_TOUCH_SINGLETOUCHSCREEN_T10) &&
		(object != MXT_TOUCH_XSLIDER_T11) &&
		(object != MXT_TOUCH_YSLIDER_T12) &&
		(object != MXT_TOUCH_XWHEEL_T13) &&
		(object != MXT_TOUCH_YWHEEL_T14) &&
		(object != MXT_TOUCH_KEYSET_T31) &&
		(object != MXT_TOUCH_XSLIDERSET_T32))) {

#ifdef DEBUG
		mutex_lock(&mxt->msg_mutex);
		/* Copy the message to buffer */
		if (mxt->msg_buffer_startp < MXT_MESSAGE_BUFFER_SIZE)
			mxt->msg_buffer_startp++;
		else
			mxt->msg_buffer_startp = 0;

		if (mxt->msg_buffer_startp == mxt->msg_buffer_endp) {
			dev_dbg(&mxt->client->dev,
				  "Message buf full, discard last entry.\n");
			if (mxt->msg_buffer_endp < MXT_MESSAGE_BUFFER_SIZE)
				mxt->msg_buffer_endp++;
			else
				mxt->msg_buffer_endp = 0;
		}
		memcpy((mxt->messages + mxt->msg_buffer_startp * length),
		       message,
		       length);
		mutex_unlock(&mxt->msg_mutex);
#endif
	}

	switch (object) {
	case MXT_GEN_COMMANDPROCESSOR_T6:
		status = message[1];

		if (status & MXT_MSGB_T6_COMSERR) {
			if ((!error_cond) & MXT_MSGB_T6_COMSERR) {
				dev_err(&client->dev,
					"maXTouch checksum error\n");
				error_cond |= MXT_MSGB_T6_COMSERR;
			}
		}
		if (status & MXT_MSGB_T6_CFGERR) {
			/*
			 * Configuration error. A proper configuration
			 * needs to be written to chip and backed up. Refer
			 * to protocol document for further info.
			 */
			if ((!error_cond) & MXT_MSGB_T6_CFGERR) {
				dev_err(&client->dev,
					"maXTouch configuration error\n");
				error_cond |= MXT_MSGB_T6_CFGERR;
			}
		}
		if (status & MXT_MSGB_T6_CAL) {
			/* Calibration in action, no need to react */
			dev_info(&client->dev,
				"maXTouch calibration in progress\n");
		}
		if (status & MXT_MSGB_T6_SIGERR) {
			/*
			 * Signal acquisition error, something is seriously
			 * wrong, not much we can in the driver to correct
			 * this
			 */
			if ((!error_cond) & MXT_MSGB_T6_SIGERR) {
				dev_err(&client->dev,
					"maXTouch acquisition error\n");
				error_cond |= MXT_MSGB_T6_SIGERR;
			}
		}
		if (status & MXT_MSGB_T6_OFL) {
			/*
			 * Cycle overflow, the acquisition is too short.
			 * Can happen temporarily when there's a complex
			 * touch shape on the screen requiring lots of
			 * processing.
			 */
			dev_err(&client->dev, "maXTouch cycle overflow\n");
		}
		if (status & MXT_MSGB_T6_RESET) {
			/* Chip has reseted, no need to react. */
			dev_info(&client->dev,
				"maXTouch chip reset\n");
		}
		if (status == 0) {
			/* Chip status back to normal. */
			dev_info(&client->dev,
				"maXTouch status normal\n");
			error_cond = 0;
		}
		break;

	case MXT_TOUCH_KEYARRAY_T15:
		dev_dbg(&client->dev, "key value, message[1]=%d, "
			"message[2]=%d, message[3]=%d, message[4]=%d",
			 message[1], message[2], message[3], message[4]);
		process_key_message(message, mxt);
		break;

	case MXT_TOUCH_MULTITOUCHSCREEN_T9:
		if (mxt->calibration_confirm < 2)
			mxt_check_calibration(mxt);
		mxt->need_report_sync = 1;
		process_T9_message(message, mxt);
		report_mt(mxt);
		break;

	case MXT_SPT_GPIOPWM_T19:
		if (debug >= DEBUG_TRACE)
			dev_info(&client->dev,
				"Receiving GPIO message\n");
		break;

	case MXT_PROCI_GRIPFACESUPPRESSION_T20:
		if (debug >= DEBUG_TRACE)
			dev_info(&client->dev,
				"Receiving face suppression msg\n");
		break;

	case MXT_PROCG_NOISESUPPRESSION_T22:
		if (debug >= DEBUG_TRACE)
			dev_info(&client->dev,
				"Receiving noise suppression msg\n");
		status = message[MXT_MSG_T22_STATUS];
		if (status & MXT_MSGB_T22_FHCHG) {
			if (debug >= DEBUG_TRACE)
				dev_info(&client->dev,
					"maXTouch: Freq changed\n");
		}
		if (status & MXT_MSGB_T22_GCAFERR) {
			if (debug >= DEBUG_TRACE)
				dev_info(&client->dev,
					"maXTouch: High noise "
					"level\n");
		}
		if (status & MXT_MSGB_T22_FHERR) {
			if (debug >= DEBUG_TRACE)
				dev_info(&client->dev,
					"maXTouch: Freq changed - "
					"Noise level too high\n");
		}
		break;

	case MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24:
		if (debug >= DEBUG_TRACE)
			dev_info(&client->dev,
				"Receiving one-touch gesture msg\n");

		event = message[MXT_MSG_T24_STATUS] & 0x0F;
		xpos = message[MXT_MSG_T24_XPOSMSB] * 16 +
			((message[MXT_MSG_T24_XYPOSLSB] >> 4) & 0x0F);
		ypos = message[MXT_MSG_T24_YPOSMSB] * 16 +
			((message[MXT_MSG_T24_XYPOSLSB] >> 0) & 0x0F);
		xpos >>= 2;
		ypos >>= 2;
		direction = message[MXT_MSG_T24_DIR];
		distance = message[MXT_MSG_T24_DIST] +
			   (message[MXT_MSG_T24_DIST + 1] << 16);

		input_event(mxt->touch_input, EV_MSC, MSC_GESTURE,
				(event << 24) | (direction << 16) | distance);
		input_event(mxt->touch_input, EV_MSC, MSC_GESTURE,
				(xpos << 16) | ypos);
		break;

	case MXT_SPT_SELFTEST_T25:
		if (debug >= DEBUG_TRACE)
			dev_info(&client->dev,
				"Receiving Self-Test msg\n");

		if (message[MXT_MSG_T25_STATUS] == MXT_MSGR_T25_RUN_ALL_TESTS) {
			if (debug >= DEBUG_TRACE)
				dev_info(&client->dev,
					"maXTouch: Self-Test OK\n");

		} else  {
			dev_err(&client->dev,
				"maXTouch: Self-Test Failed [%02x]:"
				"{%02x,%02x,%02x,%02x,%02x}\n",
				message[MXT_MSG_T25_STATUS],
				message[MXT_MSG_T25_STATUS + 0],
				message[MXT_MSG_T25_STATUS + 1],
				message[MXT_MSG_T25_STATUS + 2],
				message[MXT_MSG_T25_STATUS + 3],
				message[MXT_MSG_T25_STATUS + 4]
				);
		}
		break;

	case MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27:
		if (debug >= DEBUG_TRACE)
			dev_info(&client->dev,
				"Receiving 2-touch gesture message\n");

		event = message[MXT_MSG_T27_STATUS] & 0xF0;
		xpos = message[MXT_MSG_T27_XPOSMSB] * 16 +
			((message[MXT_MSG_T27_XYPOSLSB] >> 4) & 0x0F);
		ypos = message[MXT_MSG_T27_YPOSMSB] * 16 +
			((message[MXT_MSG_T27_XYPOSLSB] >> 0) & 0x0F);
		xpos >>= 2;
		ypos >>= 2;
		direction = message[MXT_MSG_T27_ANGLE];
		distance = message[MXT_MSG_T27_SEPARATION] +
			   (message[MXT_MSG_T27_SEPARATION + 1] << 16);

		input_event(mxt->touch_input, EV_MSC, MSC_GESTURE,
				(event << 24) | (direction << 16) | distance);
		input_event(mxt->touch_input, EV_MSC, MSC_GESTURE,
				(xpos << 16) | ypos);
		break;

	case MXT_SPT_CTECONFIG_T28:
		if (debug >= DEBUG_TRACE)
			dev_info(&client->dev,
				"Receiving CTE message...\n");
		status = message[MXT_MSG_T28_STATUS];
		if (status & MXT_MSGB_T28_CHKERR)
			dev_err(&client->dev,
				"maXTouch: Power-Up CRC failure\n");

		break;
	default:
		if (debug >= DEBUG_TRACE)
			dev_info(&client->dev,
				"maXTouch: Unknown message!\n");

		break;
	}

	return 0;
}

/*
 * Processes messages when the interrupt line (CHG) is asserted. Keeps
 * reading messages until a message with report ID 0xFF is received,
 * which indicates that there is no more new messages.
 *
 */
static void mxt_worker(struct mxt_data *mxt)
{
	struct i2c_client *client;
	u8	*message;
	u16	message_length;
	u16	message_addr;
	u8	report_id = 0;
	u8	object;
	int	error;
	int	i;

	message = NULL;
	client = mxt->client;
	message_addr = mxt->msg_proc_addr;
	message_length = mxt->message_size;

	if (message_length < 256) {
		message = kmalloc(message_length, GFP_KERNEL);
		if (message == NULL) {
			dev_err(&client->dev, "Error allocating memory\n");
			return;
		}
	} else {
		dev_err(&client->dev,
			"Message length > 256 bytes not supported\n");
		return;
	}

	dev_dbg(&client->dev, "maXTouch worker active:\n");
	do {
		/* Read next message, reread on failure. */
		mxt->message_counter++;
		for (i = 1; i < I2C_RETRY_COUNT; i++) {
			error = mxt_read_block(client, message_addr,
					message_length, message);
			if (error >= 0)
				break;
			mxt->read_fail_counter++;
			dev_err(&client->dev,
					"Failure reading maxTouch device\n");
		}
		if (error < 0) {
			/*
			 * Reset device if we fail to read message
			 * I2C_RETRY_COUNT-1 times, the touch i2c
			 * interface may die.
			 */
			mxt_gpio_reset(mxt);
			continue;
		}

		report_id = message[0];

/* FIXME! this is broken if enabled */
#ifdef DEBUG
		if (mxt->address_pointer != message_addr)
			mxt->valid_ap = 0;

		char *message_string;
		char *message_start;

		if (report_id == 0xFF)
			dev_dbg(&client->dev,
				"Reoport ID is undefined\n");
		else {
			dev_dbg(&client->dev,
				"%s message [msg count: %08x]:",
				obj_typ_name[mxt->rid_map[report_id].object],
				mxt->message_counter);
		}
		/* 5 characters per one byte */
		message_string = kmalloc(message_length * 5,
				GFP_KERNEL);
		if (message_string == NULL) {
			dev_err(&client->dev,
					"Error allocating memory\n");
			kfree(message);
			return;
		}
		message_start = message_string;
		for (i = 0; i < message_length; i++) {
			message_string +=
			sprintf(message_string,
					"0x%02X ", message[i]);
		}
		dev_dbg(&client->dev, "%s", message_start);
		kfree(message_start);
#endif

		if ((report_id != MXT_END_OF_MESSAGES) && (report_id != 0)) {
#ifdef DEBUG
			memcpy(mxt->last_message, message, message_length);
			mxt->new_msgs = 1;
			smp_wmb();
#endif
			/* Get type of object and process the message */
			object = mxt->rid_map[report_id].object;
			process_message(message, object, mxt);
		}
		dev_dbg(&client->dev, "chgline: %d\n", mxt->read_chg());
	} while ((report_id != MXT_END_OF_MESSAGES));

	if (mxt->need_report_sync)
		input_sync(mxt->touch_input);
	kfree(message);
}

/*
 * The maXTouch device will signal the host about a new message by asserting
 * the CHG line. This ISR schedules a worker routine to read the message when
 * that happens.
 */
static irqreturn_t mxt_irq_handler(int irq, void *_mxt)
{
	struct mxt_data *mxt = _mxt;

	mxt->irq_counter++;
	mxt->need_report_sync = 0;
	if (mxt->valid_interrupt()) {
		mxt->valid_irq_counter++;

		mutex_lock(&mxt->dev_mutex);
		mxt_worker(mxt);
		mutex_unlock(&mxt->dev_mutex);
	} else {
		mxt->invalid_irq_counter++;
		return IRQ_NONE;
	}

	return IRQ_HANDLED;
}

/******************************************************************************/
/* Initialization of driver                                                   */
/******************************************************************************/

static int mxt_identify(struct i2c_client *client,
				  struct mxt_data *mxt,
				  u8 *id_block_data)
{
	u8 buf[7];
	int error;
	int identified;

	identified = 0;

	/* Read Device info to check if chip is valid */
	error = mxt_read_block(client, MXT_ADDR_INFO_BLOCK, MXT_ID_BLOCK_SIZE,
			       (u8 *) buf);

	if (error < 0) {
		mxt->read_fail_counter++;
		dev_err(&client->dev, "Failure accessing maXTouch device\n");
		return -EIO;
	}

	memcpy(id_block_data, buf, MXT_ID_BLOCK_SIZE);

	mxt->device_info.family_id  = buf[0];
	mxt->device_info.variant_id = buf[1];
	mxt->device_info.major	    = ((buf[2] >> 4) & 0x0F);
	mxt->device_info.minor      = (buf[2] & 0x0F);
	mxt->device_info.build	    = buf[3];
	mxt->device_info.x_size	    = buf[4];
	mxt->device_info.y_size	    = buf[5];
	mxt->device_info.num_objs   = buf[6];
	mxt->device_info.num_nodes  = mxt->device_info.x_size *
				      mxt->device_info.y_size;

	/*
	 * Check Family & Variant Info; warn if not recognized but
	 * still continue.
	 */

	/* MXT224 */
	if (mxt->device_info.family_id == MXT224_FAMILYID) {
		strcpy(mxt->device_info.family_name, "atmel_mxt224");

		if (mxt->device_info.variant_id == MXT224_CAL_VARIANTID) {
			strcpy(mxt->device_info.variant_name, "Calibrated");
		} else if (mxt->device_info.variant_id ==
			MXT224_UNCAL_VARIANTID) {
			strcpy(mxt->device_info.variant_name, "Uncalibrated");
		} else {
			dev_err(&client->dev,
				"Warning: maXTouch Variant ID [%d] not "
				"supported\n",
				mxt->device_info.variant_id);
			strcpy(mxt->device_info.variant_name, "UNKNOWN");
			/* identified = -ENXIO; */
		}

	/* MXT1386 */
	} else if (mxt->device_info.family_id == MXT1386_FAMILYID) {
		strcpy(mxt->device_info.family_name, "mXT1386");

		if (mxt->device_info.variant_id == MXT1386_CAL_VARIANTID) {
			strcpy(mxt->device_info.variant_name, "Calibrated");
		} else {
			dev_err(&client->dev,
				"Warning: maXTouch Variant ID [%d] not "
				"supported\n",
				mxt->device_info.variant_id);
			strcpy(mxt->device_info.variant_name, "UNKNOWN");
			/* identified = -ENXIO; */
		}
	/* Unknown family ID! */
	} else {
		dev_err(&client->dev,
			"Warning: maXTouch Family ID [%d] not supported\n",
			mxt->device_info.family_id);
		strcpy(mxt->device_info.family_name, "UNKNOWN");
		strcpy(mxt->device_info.variant_name, "UNKNOWN");
		/* identified = -ENXIO; */
	}

	dev_info(
		&client->dev,
		"Atmel maXTouch (Family %s (%X), Variant %s (%X)) Firmware "
		"version [%d.%d] Build %d\n",
		mxt->device_info.family_name,
		mxt->device_info.family_id,
		mxt->device_info.variant_name,
		mxt->device_info.variant_id,
		mxt->device_info.major,
		mxt->device_info.minor,
		mxt->device_info.build
	);
	dev_info(
		&client->dev,
		"Atmel maXTouch Configuration "
		"[X: %d] x [Y: %d]\n",
		mxt->device_info.x_size,
		mxt->device_info.y_size
	);
	return identified;
}

/*
 * Reads the object table from maXTouch chip to get object data like
 * address, size, report id. For Info Block CRC calculation, already read
 * id data is passed to this function too (Info Block consists of the ID
 * block and object table).
 *
 */
static int mxt_read_object_table(struct i2c_client *client,
					   struct mxt_data *mxt,
					   u8 *raw_id_data)
{
	u16	report_id_count;
	u8	buf[MXT_OBJECT_TABLE_ELEMENT_SIZE];
	u8      *raw_ib_data;
	u8	object_type;
	u16	object_address;
	u16	object_size;
	u8	object_instances;
	u8	object_report_ids;
	u16	object_info_address;
	u32	crc;
	u32     calculated_crc;
	int	i;
	int	error;

	u8	object_instance;
	u8	object_report_id;
	u8	report_id;
	int     first_report_id;
	int     ib_pointer;
	struct mxt_object *object_table;
	dev_dbg(&mxt->client->dev, "maXTouch driver reading configuration\n");

	object_table = kzalloc(sizeof(struct mxt_object) *
			       mxt->device_info.num_objs,
			       GFP_KERNEL);
	if (object_table == NULL) {
		printk(KERN_WARNING "maXTouch: Memory allocation failed!\n");
		error = -ENOMEM;
		goto err_object_table_alloc;
	}

	raw_ib_data = kmalloc(MXT_OBJECT_TABLE_ELEMENT_SIZE *
			mxt->device_info.num_objs + MXT_ID_BLOCK_SIZE,
			GFP_KERNEL);
	if (raw_ib_data == NULL) {
		printk(KERN_WARNING "maXTouch: Memory allocation failed!\n");
		error = -ENOMEM;
		goto err_ib_alloc;
	}

	/* Copy the ID data for CRC calculation. */
	memcpy(raw_ib_data, raw_id_data, MXT_ID_BLOCK_SIZE);
	ib_pointer = MXT_ID_BLOCK_SIZE;

	mxt->object_table = object_table;

	dev_dbg(&mxt->client->dev, "maXTouch driver Memory allocated\n");

	object_info_address = MXT_ADDR_OBJECT_TABLE;

	report_id_count = 0;
	for (i = 0; i < mxt->device_info.num_objs; i++) {
		dev_dbg(&mxt->client->dev, "Reading maXTouch at [0x%04x]: ",
			  object_info_address);

		error = mxt_read_block(client, object_info_address,
				       MXT_OBJECT_TABLE_ELEMENT_SIZE, buf);

		if (error < 0) {
			mxt->read_fail_counter++;
			dev_err(&client->dev,
				"maXTouch Object %d could not be read\n", i);
			error = -EIO;
			goto err_object_read;
		}

		memcpy(raw_ib_data + ib_pointer, buf,
		       MXT_OBJECT_TABLE_ELEMENT_SIZE);
		ib_pointer += MXT_OBJECT_TABLE_ELEMENT_SIZE;

		object_type       =  buf[0];
		object_address    = (buf[2] << 8) + buf[1];
		object_size       =  buf[3] + 1;
		object_instances  =  buf[4] + 1;
		object_report_ids =  buf[5];
		dev_dbg(&mxt->client->dev, "Type=%03d, Address=0x%04x, "
			  "Size=0x%02x, %d instances, %d report id's\n",
			  object_type,
			  object_address,
			  object_size,
			  object_instances,
			  object_report_ids
		);

		/* TODO: check whether object is known and supported? */

		/* Save frequently needed info. */
		if (object_type == MXT_GEN_MESSAGEPROCESSOR_T5) {
			mxt->msg_proc_addr = object_address;
			mxt->message_size = object_size;
		}

		object_table[i].type            = object_type;
		object_table[i].chip_addr       = object_address;
		object_table[i].size            = object_size;
		object_table[i].instances       = object_instances;
		object_table[i].num_report_ids  = object_report_ids;
		report_id_count += object_instances * object_report_ids;

		object_info_address += MXT_OBJECT_TABLE_ELEMENT_SIZE;
	}

	mxt->rid_map =
		kzalloc(sizeof(struct report_id_map) * (report_id_count + 1),
			/* allocate for report_id 0, even if not used */
			GFP_KERNEL);
	if (mxt->rid_map == NULL) {
		printk(KERN_WARNING "maXTouch: Can't allocate memory!\n");
		error = -ENOMEM;
		goto err_rid_map_alloc;
	}

#ifdef DEBUG
	mxt->messages = kzalloc(mxt->message_size * MXT_MESSAGE_BUFFER_SIZE,
				GFP_KERNEL);
	if (mxt->messages == NULL) {
		printk(KERN_WARNING "maXTouch: Can't allocate memory!\n");
		error = -ENOMEM;
		goto err_msg_alloc1;
	}

	mxt->last_message = kzalloc(mxt->message_size, GFP_KERNEL);
	if (mxt->last_message == NULL) {
		printk(KERN_WARNING "maXTouch: Can't allocate memory!\n");
		error = -ENOMEM;
		goto err_msg_alloc2;
	}
#endif

	mxt->report_id_count = report_id_count;
	if (report_id_count > 254) {	/* 0 & 255 are reserved */
			dev_err(&client->dev,
				"Too many maXTouch report id's [%d]\n",
				report_id_count);
			error = -ENXIO;
			goto err_max_rid;
	}

	/* Create a mapping from report id to object type */
	report_id = 1; /* Start from 1, 0 is reserved. */

	/* Create table associating report id's with objects & instances */
	for (i = 0; i < mxt->device_info.num_objs; i++) {
		for (object_instance = 0;
		     object_instance < object_table[i].instances;
		     object_instance++){
			first_report_id = report_id;
			for (object_report_id = 0;
			     object_report_id < object_table[i].num_report_ids;
			     object_report_id++) {
				mxt->rid_map[report_id].object =
					object_table[i].type;
				mxt->rid_map[report_id].instance =
					object_instance;
				mxt->rid_map[report_id].first_rid =
					first_report_id;
				report_id++;
			}
		}
	}

	/* Read 3 byte CRC */
	error = mxt_read_block(client, object_info_address, 3, buf);
	if (error < 0) {
		mxt->read_fail_counter++;
		dev_err(&client->dev, "Error reading CRC\n");
	}

	crc = (buf[2] << 16) | (buf[1] << 8) | buf[0];

	if (calculate_infoblock_crc(&calculated_crc, raw_ib_data,
				    ib_pointer)) {
		printk(KERN_WARNING "Error while calculating CRC!\n");
		calculated_crc = 0;
	}
	kfree(raw_ib_data);

	dev_dbg(&mxt->client->dev, "\nReported info block CRC = 0x%6X\n", crc);
	dev_dbg(&mxt->client->dev, "Calculated info block CRC = 0x%6X\n\n",
		       calculated_crc);

	if (crc == calculated_crc) {
		mxt->info_block_crc = crc;
	} else {
		mxt->info_block_crc = 0;
		printk(KERN_ALERT "maXTouch: Info block CRC invalid!\n");
	}

	if (debug >= DEBUG_VERBOSE) {

		dev_info(&client->dev, "maXTouch: %d Objects\n",
				mxt->device_info.num_objs);

		for (i = 0; i < mxt->device_info.num_objs; i++) {
			dev_info(&client->dev, "Type:\t\t\t[%d]: %s\n",
				 object_table[i].type,
				 obj_typ_name[object_table[i].type]);
			dev_info(&client->dev, "\tAddress:\t0x%04X\n",
				object_table[i].chip_addr);
			dev_info(&client->dev, "\tSize:\t\t%d Bytes\n",
				 object_table[i].size);
			dev_info(&client->dev, "\tInstances:\t%d\n",
				 object_table[i].instances);
			dev_info(&client->dev, "\tReport Id's:\t%d\n",
				 object_table[i].num_report_ids);
		}
	}

	return 0;

err_max_rid:
#ifdef DEBUG
	kfree(mxt->last_message);
err_msg_alloc2:
	kfree(mxt->messages);
err_msg_alloc1:
#endif
	kfree(mxt->rid_map);
err_rid_map_alloc:
err_object_read:
	kfree(raw_ib_data);
err_ib_alloc:
	kfree(object_table);
err_object_table_alloc:
	return error;
}

static u8 mxt_valid_interrupt_dummy(void)
{
	return 1;
}

static int mxt_get_config_version(struct mxt_data *mxt)
{
	int ret;
	u32 t38[2] = { 0 };
	u16 addr;

	addr = get_object_address(MXT_USER_INFO_T38,
			0, mxt->object_table,
			mxt->device_info.num_objs);
	ret = mxt_read_block(mxt->client, addr, sizeof(t38), (u8 *)t38);
	if (ret < 0)
		return ret;

	return t38[0];
}

static int mxt_set_config_version(struct mxt_data *mxt, int version)
{
	int ret;
	u32 t38[2] = { 0 };
	u16 addr;

	addr = get_object_address(MXT_USER_INFO_T38,
			0, mxt->object_table,
			mxt->device_info.num_objs);

	t38[0] = version;
	ret = mxt_write_block(mxt->client, addr, sizeof(t38), (u8 *)t38);
	if (ret < 0)
		return ret;

	return 0;
}

static void mxt_initialize(struct mxt_data *mxt)
{
	int config_version = mxt_get_config_version(mxt);
	u16 addr;

	dev_info(&mxt->client->dev, "version nvm:%d, current:%d\n",
		 config_version, MXT_CONFIG_VERSION);

#ifdef MXT_CHECK_VERSION
	if (config_version >= MXT_CONFIG_VERSION)
		goto out_reset;
#endif

	mxt_config_init(mxt);
	mxt_set_config_version(mxt, MXT_CONFIG_VERSION);

	addr = get_object_address(MXT_GEN_COMMANDPROCESSOR_T6,
			0, mxt->object_table,
			mxt->device_info.num_objs) + MXT_ADR_T6_BACKUPNV;
	mxt_write_byte(mxt->client, addr, MXT_CMD_T6_BACKUP);
	msleep(MXT_BACKNVM_DELAY);

#ifdef MXT_CHECK_VERSION
out_reset:
#endif
	mxt_reset(mxt);
}

static void mxt_early_suspend_handler()
{
	u16 addr;
	u8 buf[3] = { 0 };
	u8 err;

	disable_irq(mxt_es->irq);

	mutex_lock(&mxt_es->dev_mutex);

	mxt_enable_autocalib(mxt_es);

	addr = get_object_address(MXT_GEN_POWERCONFIG_T7,
				0,
				mxt_es->object_table,
				mxt_es->device_info.num_objs);
	mxt_read_block(mxt_es->client, addr, 3, mxt_es->T7);
	err = mxt_write_block(mxt_es->client, addr, 3, buf);
	if (err < 0)
		dev_err(&mxt_es->client->dev, "fail to stop scan.\n");

	mxt_es->suspended = TRUE;

	mutex_unlock(&mxt_es->dev_mutex);
}

static void mxt_calibrate(struct mxt_data *mxt)
{
	u16 addr;

	addr = get_object_address(MXT_GEN_COMMANDPROCESSOR_T6,
			0, mxt->object_table,
			mxt->device_info.num_objs) + MXT_ADR_T6_CALIBRATE;
	mxt_write_byte(mxt->client, addr, 0x55);
}


static void mxt_late_resume_handler()
{
	int i;
	int ret;
	u16 addr;

	enable_irq(mxt_es->irq);

	mutex_lock(&mxt_es->dev_mutex);
	mxt_es->calibration_confirm = 0;
	addr = get_object_address(MXT_GEN_POWERCONFIG_T7,
				0,
				mxt_es->object_table,
				mxt_es->device_info.num_objs);
	ret = mxt_write_block(mxt_es->client, addr, 3, mxt_es->T7);
	if (ret < 0) {
		dev_err(&mxt_es->client->dev, "fail to start scan.\n");
		mxt_gpio_reset(mxt_es);
	} else {
		msleep(40);
		mxt_calibrate(mxt_es);
	}

	/* clear touch state when suspending */
	for (i = 0; i < mxt_es->numtouch; i++) {
		if (!mxt_es->finger[i].status)
			continue;
		mxt_es->finger[i].status = MXT_MSGB_T9_RELEASE;
	}
	report_mt(mxt_es);
	input_sync(mxt_es->touch_input);

	mxt_es->suspended = FALSE;
	mxt_es->timestamp = jiffies;
	mutex_unlock(&mxt_es->dev_mutex);
}

static ssize_t early_suspend_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	if (!strncmp(buf, EARLY_SUSPEND_ON, EARLY_SUSPEND_STATUS_LEN))
		mxt_early_suspend_handler();
	else if (!strncmp(buf, EARLY_SUSPEND_OFF, EARLY_SUSPEND_STATUS_LEN))
		mxt_late_resume_handler();

	return count;
}

static DEVICE_EARLY_SUSPEND_ATTR(early_suspend_store);

static int mxt_probe(struct i2c_client *client,
			       const struct i2c_device_id *id)
{
	struct mxt_data          *mxt;
	struct mxt_platform_data *pdata;
	struct input_dev         *touch_input;
	struct input_dev         *key_input;
	u8 *id_data;
	int error;

	printk(KERN_INFO "atmel_mxt224: mxt_probe\n");

	if (client == NULL) {
		pr_debug("maXTouch: client == NULL\n");
		return	-EINVAL;
	} else if (client->adapter == NULL) {
		pr_debug("maXTouch: client->adapter == NULL\n");
		return	-EINVAL;
	} else if (&client->dev == NULL) {
		pr_debug("maXTouch: client->dev == NULL\n");
		return	-EINVAL;
	} else if (&client->adapter->dev == NULL) {
		pr_debug("maXTouch: client->adapter->dev == NULL\n");
		return	-EINVAL;
	} else if (id == NULL) {
		pr_debug("maXTouch: id == NULL\n");
		return	-EINVAL;
	}

	dev_dbg(&client->dev, "maXTouch driver v. %s\n", DRIVER_VERSION);
	dev_dbg(&client->dev, "\t \"%s\"\n", client->name);
	dev_dbg(&client->dev, "\taddr:\t0x%04x\n", client->addr);
	dev_dbg(&client->dev, "\tirq:\t%d\n", client->irq);
	dev_dbg(&client->dev, "\tflags:\t0x%04x\n", client->flags);
	dev_dbg(&client->dev, "\tadapter:\"%s\"\n", client->adapter->name);
	dev_dbg(&client->dev, "\tdevice:\t\"%s\"\n", client->dev.init_name);

	/* Check if the I2C bus supports BYTE transfer */
	error = i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE);
	if (!error) {
		dev_err(&client->dev, "%s adapter not supported\n",
				dev_driver_string(&client->adapter->dev));
		return -ENODEV;
	}
	dev_dbg(&client->dev, "maXTouch driver functionality OK\n");

	/* Allocate structure - we need it to identify device */
	mxt = kzalloc(sizeof(struct mxt_data), GFP_KERNEL);
	if (mxt == NULL) {
		dev_err(&client->dev, "insufficient memory\n");
		error = -ENOMEM;
		goto err_mxt_alloc;
	}

	id_data = kmalloc(MXT_ID_BLOCK_SIZE, GFP_KERNEL);
	if (id_data == NULL) {
		dev_err(&client->dev, "insufficient memory\n");
		error = -ENOMEM;
		goto err_id_alloc;
	}

	touch_input = input_allocate_device();
	if (!touch_input) {
		dev_err(&client->dev, "error allocating touch input device\n");
		error = -ENOMEM;
		goto err_touch_input_dev_alloc;
	}

	key_input = input_allocate_device();
	if (!key_input) {
		dev_err(&client->dev, "error allocating key input device");
		error = -ENOMEM;
		goto err_key_input_dev_alloc;
	}

	/* Initialize Platform data */
	pdata = client->dev.platform_data;
	if (pdata == NULL) {
		dev_err(&client->dev, "platform data is required!\n");
		error = -EINVAL;
		goto err_pdata;
	}
	if (debug >= DEBUG_TRACE)
		printk(KERN_INFO "Platform OK: pdata = 0x%08x\n",
		       (unsigned int) pdata);

	mutex_init(&mxt->dev_mutex);
	mxt->read_fail_counter = 0;
	mxt->message_counter   = 0;
	mxt->max_x_val         = pdata->max_x;
	mxt->max_y_val         = pdata->max_y;
	mxt->orientation       = pdata->orientation;

	mxt->mxt_intr_gpio = pdata->irq;
	mxt->mxt_reset_gpio = pdata->reset;

	/*init INTERRUPT pin*/
	error = gpio_request(mxt->mxt_intr_gpio, 0);
	if (error < 0) {
		printk(KERN_ERR "Failed to request GPIO%d (MaxTouch-interrupt) "
			"error=%d\n",
			mxt->mxt_intr_gpio, error);
	}

	error = gpio_direction_input(mxt->mxt_intr_gpio);
	if (error) {
		printk(KERN_ERR "Failed to set interrupt direction, "
						"error=%d\n", error);
		gpio_free(mxt->mxt_intr_gpio);
	}

	/*init RESET pin*/
	error = gpio_request(mxt->mxt_reset_gpio, "MaxTouch-reset");
	if (error < 0) {
		printk(KERN_ERR "Failed to request GPIO%d (MaxTouch-reset) error=%d\n",
			mxt->mxt_reset_gpio, error);
	}

	error = gpio_direction_output(mxt->mxt_reset_gpio, 1);
	if (error) {
		printk(KERN_ERR "Failed to set reset direction, "
						"error=%d\n", error);
		gpio_free(mxt->mxt_reset_gpio);
	}

	/* maXTouch wants 40mSec minimum after reset to get organized */
	gpio_set_value(mxt->mxt_reset_gpio, 1);
	msleep(40);

	/* Get data that is defined in board specific code. */
	mxt->init_hw = pdata->init_platform_hw;
	mxt->exit_hw = pdata->exit_platform_hw;
	mxt->read_chg = pdata->read_chg;

	if (pdata->valid_interrupt != NULL)
		mxt->valid_interrupt = pdata->valid_interrupt;
	else
		mxt->valid_interrupt = mxt_valid_interrupt_dummy;

	if (mxt->init_hw != NULL)
		mxt->init_hw();

	if (debug >= DEBUG_TRACE)
		printk(KERN_INFO "maXTouch driver identifying chip\n");

	if (mxt_identify(client, mxt, id_data) < 0) {
		dev_err(&client->dev, "Chip could not be identified\n");
		error = -ENODEV;
		goto err_identify;
	}
	/* Chip is valid and active. */
	if (debug >= DEBUG_TRACE)
		printk(KERN_INFO "maXTouch driver allocating input device\n");

	mxt->client = client;
	mxt->touch_input = touch_input;
	mxt->key_input = key_input;

	dev_dbg(&mxt->client->dev, "maXTouch driver creating device name\n");

	snprintf(mxt->touch_phys_name, sizeof(mxt->touch_phys_name),
		 "%s/input0", dev_name(&client->dev));

	snprintf(mxt->key_phys_name, sizeof(mxt->key_phys_name),
		 "%s/input1", dev_name(&client->dev));

	/*touch input parameter*/
	touch_input->name = "mxt224_touchscreen_0";
	touch_input->phys = mxt->touch_phys_name;
	touch_input->id.bustype = BUS_I2C;
	touch_input->dev.parent = &client->dev;

	dev_dbg(&client->dev, "maXTouch name: \"%s\"\n", touch_input->name);
	dev_dbg(&client->dev, "maXTouch phys: \"%s\"\n", touch_input->phys);

	/*key input parameter*/
	key_input->name = "mxt224_key_0";
	key_input->phys = mxt->key_phys_name;

	dev_dbg(&client->dev, "maxTouch name: \"%s\"\n", key_input->name);
	dev_dbg(&client->dev, "maxTouch phys: \"%s\"\n", key_input->phys);
	dev_dbg(&client->dev, "maXTouch driver setting abs parameters\n");

	/* Multitouch */
	input_mt_init_slots(touch_input, MXT_MAX_NUM_TOUCHES);
	input_set_abs_params(touch_input, ABS_MT_POSITION_X,
			     TS_MIN_X, TS_MAX_X, 0, 0);
	input_set_abs_params(touch_input, ABS_MT_POSITION_Y,
			     TS_MIN_Y, TS_MAX_Y, 0, 0);
	input_set_abs_params(touch_input, ABS_MT_TOUCH_MAJOR,
			     0, MXT_MAX_TOUCH_SIZE, 0, 0);

	__set_bit(EV_ABS, touch_input->evbit);
	__set_bit(EV_SYN, touch_input->evbit);

	/* Function key*/
	__set_bit(EV_KEY, key_input->evbit);
	__set_bit(KEY_HOME, key_input->keybit);
	__set_bit(KEY_MENU, key_input->keybit);
	__set_bit(KEY_BACK, key_input->keybit);
	__set_bit(KEY_SEARCH, key_input->keybit);

	dev_dbg(&mxt->client->dev, "maXTouch driver setting client data\n");
	i2c_set_clientdata(client, mxt);
	dev_dbg(&mxt->client->dev, "maXTouch driver setting drv data\n");
	input_set_drvdata(touch_input, mxt);
	input_set_drvdata(key_input, mxt);

	dev_dbg(&mxt->client->dev, "maXTouch driver input register device\n");
	error = input_register_device(mxt->touch_input);
	if (error < 0) {
		dev_err(&client->dev,
			"Failed to register touch input device\n");
		goto err_register_touch_device;
	}
	error = input_register_device(mxt->key_input);
	if (error < 0) {
		dev_err(&client->dev,
			"Failed to register key input device in mxt224\n");
		goto err_register_key_device;
	}

	error = mxt_read_object_table(client, mxt, id_data);
	if (error < 0)
		goto err_read_ot;

#ifdef DEBUG
	mutex_init(&mxt->debug_mutex);
	mutex_init(&mxt->msg_mutex);
	/* Create debugfs entries. */
	mxt->debug_dir = debugfs_create_dir("maXTouch", NULL);
	if ((int)mxt->debug_dir == -ENODEV) {
		/* debugfs is not enabled. */
		printk(KERN_WARNING "debugfs not enabled in kernel\n");
	} else if (mxt->debug_dir == NULL) {
		printk(KERN_WARNING "error creating debugfs dir\n");
	} else {
		dev_dbg(&mxt->client->dev,
			"created \"maXTouch\" debugfs dir\n");

		debugfs_create_file("deltas", S_IRUSR, mxt->debug_dir, mxt,
				    &delta_fops);
		debugfs_create_file("refs", S_IRUSR, mxt->debug_dir, mxt,
				    &refs_fops);
	}

	/* Create character device nodes for reading & writing registers */
	mxt->mxt_class = class_create(THIS_MODULE, "maXTouch_memory");
	/* 2 numbers; one for memory and one for messages */
	error = alloc_chrdev_region(&mxt->dev_num, 0, 2,
				    "maXTouch_memory");
	dev_info(&mxt->client->dev,
		"device number %d allocated!\n", MAJOR(mxt->dev_num));
	if (error)
		printk(KERN_WARNING "Error registering device\n");
	cdev_init(&mxt->cdev, &mxt_memory_fops);
	cdev_init(&mxt->cdev_messages, &mxt_message_fops);

	dev_info(&mxt->client->dev, "cdev initialized\n");
	mxt->cdev.owner = THIS_MODULE;
	mxt->cdev_messages.owner = THIS_MODULE;

	error = cdev_add(&mxt->cdev, mxt->dev_num, 1);
	if (error)
		printk(KERN_WARNING "Bad cdev\n");

	error = cdev_add(&mxt->cdev_messages, mxt->dev_num + 1, 1);
	if (error)
		printk(KERN_WARNING "Bad cdev\n");

	dev_info(&mxt->client->dev, "cdev added\n");

	device_create(mxt->mxt_class, NULL, MKDEV(MAJOR(mxt->dev_num), 0),
			NULL,
			"maXTouch");

	device_create(mxt->mxt_class, NULL, MKDEV(MAJOR(mxt->dev_num), 1),
			NULL,
			"maXTouch_messages");

	mxt->msg_buffer_startp = 0;
	mxt->msg_buffer_endp = 0;
#endif

	device_create_file(&client->dev, &dev_attr_early_suspend);

	mxt->prev_key = 0;

	if (pdata->numtouch)
		mxt->numtouch = pdata->numtouch;

	mxt_initialize(mxt);

	/* Allocate the interrupt */
	dev_dbg(&mxt->client->dev,
			"maXTouch driver allocating interrupt...\n");
	mxt->irq = gpio_to_irq(mxt->mxt_intr_gpio);
	mxt->valid_irq_counter = 0;
	mxt->invalid_irq_counter = 0;
	mxt->irq_counter = 0;

	if (mxt->irq) {
		error = request_threaded_irq(mxt->irq, NULL,
				    mxt_irq_handler,
				    IRQF_TRIGGER_FALLING,
				    client->dev.driver->name,
				    mxt);
		if (error < 0) {
			dev_err(&client->dev,
				"failed to allocate irq %d\n", mxt->irq);
			goto err_irq;
		}
	}

	mxt->timestamp = jiffies;
	mutex_lock(&mxt->dev_mutex);
	mxt_worker(mxt);
	mutex_unlock(&mxt->dev_mutex);

	kfree(id_data);

	mxt->suspended = FALSE;
	mxt->T7[0] = 32;
	mxt->T7[1] = 10;
	mxt->T7[2] = 50;

#ifdef CONFIG_HAS_EARLYSUSPEND
	mxt->es.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	mxt->es.suspend = mxt_early_suspend;
	mxt->es.resume = mxt_late_resume;
	register_early_suspend(&mxt->es);
#endif
	mxt_es = mxt;

	register_early_suspend_device(&client->dev);

	return 0;

err_irq:
	kfree(mxt->rid_map);
	kfree(mxt->object_table);
#ifdef DEBUG
	kfree(mxt->last_message);
#endif
err_read_ot:
err_register_key_device:
err_register_touch_device:
err_identify:
err_pdata:
	input_free_device(key_input);
err_key_input_dev_alloc:
	input_free_device(touch_input);
err_touch_input_dev_alloc:
	kfree(id_data);
err_id_alloc:
	if (mxt->exit_hw != NULL)
		mxt->exit_hw();
	kfree(mxt);
err_mxt_alloc:
	return error;
}

static int mxt_remove(struct i2c_client *client)
{
	struct mxt_data *mxt;

	mxt = i2c_get_clientdata(client);

	if (mxt != NULL) {
		if (mxt->exit_hw != NULL)
			mxt->exit_hw();

		if (mxt->irq)
			free_irq(mxt->irq, mxt);

		device_remove_file(&client->dev, &dev_attr_early_suspend);
#ifdef DEBUG
		/* Remove debug dir entries */
		debugfs_remove_recursive(mxt->debug_dir);
		unregister_chrdev_region(mxt->dev_num, 2);
		device_destroy(mxt->mxt_class, MKDEV(MAJOR(mxt->dev_num), 0));
		device_destroy(mxt->mxt_class, MKDEV(MAJOR(mxt->dev_num), 1));
		cdev_del(&mxt->cdev);
		cdev_del(&mxt->cdev_messages);
		class_destroy(mxt->mxt_class);
		debugfs_remove(mxt->debug_dir);
		kfree(mxt->last_message);
#endif
		unregister_early_suspend_device(&client->dev);
#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&mxt->es);
#endif
		input_unregister_device(mxt->key_input);
		input_unregister_device(mxt->touch_input);
		kfree(mxt->rid_map);
		kfree(mxt->object_table);
	}
	kfree(mxt);

	if (debug >= DEBUG_TRACE)
		dev_info(&client->dev, "Touchscreen unregistered\n");

	return 0;
}

#if defined(CONFIG_PM)
static int mxt_suspend(struct device *dev)
{
	struct mxt_data *mxt = dev_get_drvdata(dev);

	dev_dbg(&mxt->client->dev, "In function %s", __func__);

	if (device_may_wakeup(dev))
		enable_irq_wake(mxt->irq);

	return 0;
}

static int mxt_resume(struct device *dev)
{
	struct mxt_data *mxt = dev_get_drvdata(dev);

	dev_dbg(&mxt->client->dev, "In function %s", __func__);

	if (device_may_wakeup(dev))
		disable_irq_wake(mxt->irq);

	return 0;
}
#else
#define mxt_suspend NULL
#define mxt_resume NULL
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
void mxt_early_suspend(struct early_suspend *h)
{
	mxt_early_suspend_handler();
}

void mxt_late_resume(struct early_suspend *h)
{
	mxt_late_resume_handler();
}
#endif

static const struct dev_pm_ops mxt_pm_ops = {
	.suspend = mxt_suspend,
	.resume = mxt_resume,
};

static const struct i2c_device_id mxt_idtable[] = {
	{TOUCH_DEVICE_NAME, 1,},
	{ }
};

MODULE_DEVICE_TABLE(i2c, mxt_idtable);

static struct i2c_driver mxt_driver = {
	.driver = {
		.name	= TOUCH_DEVICE_NAME,
		.owner  = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &mxt_pm_ops,
#endif
	},

	.id_table	= mxt_idtable,
	.probe		= mxt_probe,
	.remove		= mxt_remove,
};

module_i2c_driver(mxt_driver);

MODULE_AUTHOR("Iiro Valkonen");
MODULE_DESCRIPTION("Driver for Atmel maXTouch Touchscreen Controller");
MODULE_LICENSE("GPL");
