/*
 * Raw data process algos for general sensor driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/errno.h>
#include "sensor_driver_config.h"
#include "sensor_general.h"

/*don't enable this if already put in XML*/
#ifdef CONFIG_SENSOR_RAW_PROC_BMC150

/*Raw data process algos for bmc150 compass
* Output 16LSB/uT with temperature compensation
*/
#define DIG_X1	((s8)data->regbuf[0x5d])
#define DIG_Y1	((s8)data->regbuf[0x5e])
#define DIG_X2	((s8)data->regbuf[0x64])
#define DIG_Y2	((s8)data->regbuf[0x65])
#define DIG_XY1	((u8)data->regbuf[0x71])
#define DIG_XY2	((s8)data->regbuf[0x70])

#define DIG_Z1	((u16)((data->regbuf[0x6b] << 8) | data->regbuf[0x6a]))
#define DIG_Z2	((s16)((data->regbuf[0x69] << 8) | data->regbuf[0x68]))
#define DIG_Z3	((s16)((data->regbuf[0x6f] << 8) | data->regbuf[0x6e]))
#define DIG_Z4	((s16)((data->regbuf[0x63] << 8) | data->regbuf[0x62]))

#define DIG_XYZ1 ((u16)(((data->regbuf[0x6d] & 0x7f) << 8) | data->regbuf[0x6c]))
#define DATA_R ((u16)((data->regbuf[0x49] << 6) | ((data->regbuf[0x48] & 0xfc) >> 2)))

int bsbm0150_proc_x(struct sensor_data *data, int raw)
{
	short mdata_x = (short)raw;
	unsigned short data_R = DATA_R;
	short inter_retval;

	SENSOR_DBG(DBG_LEVEL4, data->dbg_on, "%s raw:0x%x 0x%x",
			data->config->input_name, raw, data_R);

	if (mdata_x == -4096)
		return (int)(-2147483647-1);
	else {
		inter_retval = ((short)(((unsigned short)
				((((int)DIG_XYZ1) << 14) /
				(data_R != 0 ? data_R : DIG_XYZ1))) -
				((unsigned short)0x4000)));

		inter_retval = ((short)((((int)mdata_x) *
				((((((((int)DIG_XY2) *
				((((int)inter_retval) *
				((int)inter_retval)) >> 7)) +
				(((int)inter_retval) *
				((int)(((short)DIG_XY1)
				<< 7)))) >> 9) +
				((int)0x100000)) *
				((int)(((short)DIG_X2) +
				((short)0xA0)))) >> 12)) >> 13)) +
				(((short)DIG_X1) << 3);

		return inter_retval;
	}
}

int bsbm0150_proc_y(struct sensor_data *data, int raw)
{
	short mdata_y = (short)raw;
	unsigned short data_R = DATA_R;
	short inter_retval;

	SENSOR_DBG(DBG_LEVEL4, data->dbg_on, "%s raw:0x%x 0x%x",
			data->config->input_name, raw, data_R);

	if (mdata_y == -4096)
		return (int)(-2147483647-1);
	else {
		inter_retval = ((short)(((unsigned short)(((
				(int)DIG_XYZ1) << 14) /
				(data_R != 0 ?
				data_R : DIG_XYZ1))) -
				((unsigned short)0x4000)));

		inter_retval = ((short)((((int)mdata_y) *
				((((((((int)
				DIG_XY2) *
				((((int) inter_retval) *
				((int)inter_retval)) >> 7)) +
				(((int)inter_retval) *
				((int)(((short)
				DIG_XY1) << 7)))) >> 9) +
				((int)0x100000)) *
				((int)(((short)DIG_Y2)
				+ ((short)0xA0))))
				>> 12)) >> 13)) +
				(((short)DIG_Y1) << 3);

		return inter_retval;
	}
}

int bsbm0150_proc_z(struct sensor_data *data, int raw)
{
	short mdata_z = (short)raw;
	unsigned short data_R = DATA_R;
	int retval;

	SENSOR_DBG(DBG_LEVEL4, data->dbg_on, "%s raw:0x%x 0x%x",
			data->config->input_name, raw, data_R);

	if (mdata_z != -16384) {
		retval = (((((int)(mdata_z - DIG_Z4)) << 15) -
			((((int)DIG_Z3) *
			((int)(((short)data_R) -
			((short)DIG_XYZ1))))>>2)) /
			(DIG_Z2+
			((short)(((((int)DIG_Z1) *
			((((short)data_R) << 1)))+(1<<15))>>16))));
	} else
		retval = ((int)(-2147483647-1));

	return retval;
}
#endif

static struct sensor_rawdata_proc sensor_proc_tbl[] = {
#ifdef CONFIG_SENSOR_RAW_PROC_BMC150
	{"BSBM0150", bsbm0150_proc_x, bsbm0150_proc_y, bsbm0150_proc_z},
#endif
	{NULL}
};

static int __init sensor_rawdata_proc_init(void)
{
	int ret = 0;
	int i;

	for (i = 0; sensor_proc_tbl[i].name; i++) {
		struct sensor_rawdata_proc *proc = &sensor_proc_tbl[i];

		ret = sensor_register_rawdata_proc(proc);
		if (ret < 0)
			printk(KERN_ERR "Fail to register proc %s\n",
				proc->name);
	}

	return ret;
}

static void __exit sensor_rawdata_proc_exit(void)
{
	int ret;
	int i;

	for (i = 0; sensor_proc_tbl[i].name; i++) {
		struct sensor_rawdata_proc *proc = &sensor_proc_tbl[i];

		ret = sensor_unregister_rawdata_proc(proc);
		if (ret < 0)
			printk(KERN_ERR "Fail to unregister proc %s\n",
				proc->name);
	}

	return;
}

module_init(sensor_rawdata_proc_init);
module_exit(sensor_rawdata_proc_exit);

MODULE_AUTHOR("IO&Sensor");
MODULE_DESCRIPTION("Raw Data Process Driver");
MODULE_LICENSE("GPL V2");

