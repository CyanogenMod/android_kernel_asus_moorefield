/*
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
 * Date:	May 2013
 * Authors:	PSI IO & Sensor Team
 *		qipeng.zha@intel.com
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/sysfs.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/acpi_gpio.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <asm/div64.h>
#include "sensor_driver_config.h"
#include "sensor_general.h"
#include <linux/sched.h>
#include <linux/sched/rt.h>

#ifdef CONFIG_GENERAL_SENSOR_DEBUG

/*debug switch*/
unsigned int sensor_general_debug_sensors;
unsigned int sensor_general_debug_level;
EXPORT_SYMBOL(sensor_general_debug_sensors);
EXPORT_SYMBOL(sensor_general_debug_level);
char *action_debug[] = {
	"OP_ACCESS",
	"OP_MIN", "OP_MAX",
	"OP_LOGIC_EQ", "OP_LOGIC_NEQ", "OP_LOGIC_GREATER", "OP_LOGIC_LESS",
	"OP_LOGIC_GE", "OP_LOGIC_LE", "OP_LOGIC_AND", "OP_LOGIC_OR",
	"OP_ARI_ADD", "OP_ARI_SUB", "OP_ARI_MUL", "OP_ARI_DIV", "OP_ARI_MOD",
	"OP_BIT_OR", "OP_BIT_AND", "OP_BIT_LSL", "OP_BIT_LSR", "OP_BIT_NOR",
	"OP_ENDIAN_BE16", "OP_ENDIAN_BE16_UN", "OP_ENDIAN_BE24",
	"OP_ENDIAN_BE32", "OP_ENDIAN_LE16", "OP_ENDIAN_LE16_UN",
	"OP_ENDIAN_LE24", "OP_ENDIAN_LE32",
	"OP_RESERVE",
};

/*performance switch*/
static unsigned int sensor_general_time;

static struct {
	int times;
	ktime_t time;
} sensor_time[MAX_SENSOR_DRIVERS];

static inline void sensor_time_start(struct sensor_data *data, ktime_t *start)
{
	/*preempt_disable();*/
	if (data->dbg_on & sensor_general_time)
		*start = ktime_get();
}

static inline void sensor_time_end(struct sensor_data *data, ktime_t *start)
{
	int sensor_num;
	ktime_t end;
	ktime_t duration;

	sensor_num = data->dbg_on & sensor_general_time;
	if (sensor_num) {
		end = ktime_get();

		duration = sensor_time[ffs(sensor_num) - 1].time;
		duration = ktime_add(duration, ktime_sub(end, *start));
		sensor_time[ffs(sensor_num) - 1].time = duration;

		sensor_time[ffs(sensor_num) - 1].times++;
	}
	/*preempt_enable();*/
}

#else

#define sensor_time_start(a, b)
#define sensor_time_end(a, b)

#endif

static p_extern_c extern_c_array[MAX_EXTERN_C];
static DEFINE_MUTEX(sensor_externc_lock);
static long extern_c_ref_cnt = 0;

static void sensor_launch_work(struct sensor_data *data);
static void unregister_failed_devices(void);

static inline void stack_init(struct sensor_data_stack *stack)
{
	stack->top = -1;
}

static inline int stack_empty(struct sensor_data_stack *stack)
{
	if (stack->top == -1)
		return 1;
	else
		return 0;
}

static inline int pop(struct sensor_data *data)
{
	int ret;
	struct sensor_data_stack *stack = &data->stack;

	if (stack->top < 0 || stack->top > DATA_STACK_MAX_SIZE - 1) {
		dump_stack();
		printk(KERN_ERR "Corrupted stack in sensor drver %d\n",
				stack->top);
		return -EFAULT;
	}

	ret = stack->data_stack[stack->top--];

	SENSOR_DBG(DBG_LEVEL5, data->dbg_on, "%d val:0x%x %s",
				stack->top, ret, data->config->input_name);
	return ret;
}

static inline void push(struct sensor_data *data, int val)
{
	struct sensor_data_stack *stack = &data->stack;

	if (stack->top >= DATA_STACK_MAX_SIZE - 1 || stack->top < -1) {
		dump_stack();
		printk(KERN_ERR "Corrupted stack in sensor drver %d\n",
				stack->top);
		return;
	}

	SENSOR_DBG(DBG_LEVEL5, data->dbg_on, "%d val:0x%x %s",
					stack->top, val,
					data->config->input_name);
	stack->data_stack[++stack->top] = val;
}

static inline int data_operand(struct sensor_data *data,
				struct operand *oper)
{
	int index;
	int len;
	int addr;
	int op;

	switch (oper->type) {
	case OPT_BEFORE:
		op = pop(data);
		break;

	case OPT_IMM:
		op = oper->data.immediate;
		break;

	case OPT_REG_BUF:
		addr = oper->data.reg.addr;
		len = oper->data.reg.len;
		op = 0;
		while (--len >= 0) {
			/*assume little endian, if not,
			use additional endian action*/
			op <<= 8;
			op += data->regbuf[addr + len];
		}
		break;

	case OPT_INDEX:
		index = oper->data.index;
		op = data->private[index];
		break;

	default:
		dev_err(&data->client->dev, "Nonsupport operand1\n");
		return -EINVAL;
	}

	return op;
}

/*
* access of global variable and i2c registers
* i2c reg = reg buf, immediate, before data , global variable
* reg bufs = i2c regs, immediate, before data, global variable
* global variable = reg buf, immeidate, before data
*/
static int data_op_access(struct sensor_data *data, struct data_action *action)
{
	u8 addr;
	u8 flag;
	int len;
	int val;
	int ret = 0;

	SENSOR_DBG(DBG_LEVEL4, data->dbg_on, "%p", action);

	switch (action->operand1.type) {
	case OPT_REG_BUF:
		addr = action->operand1.data.reg.addr;

		switch (action->operand2.type) {
		case OPT_REG:
			/*read i2c register, assume two addr are same*/
			len = action->operand2.data.reg.len;
			flag = action->operand2.data.reg.flag;

			SENSOR_DBG(DBG_LEVEL3, data->dbg_on,
				"readreg:(0x%x %d)=",
				(unsigned int)addr|flag, (unsigned int)len);

			if (len > 1) {
				ret = i2c_smbus_read_i2c_block_data(
					data->client, addr | flag,
					len, &data->regbuf[addr]);
				if (ret < 0) {
					dev_err(&data->client->dev,
						"i2c read error\n");
					ret = -EIO;
				} else {
					int i = 0;

					for (; i < len; i++)
						SENSOR_DBG(DBG_LEVEL3,
							data->dbg_on,
							"0x%x",
						data->regbuf[addr + i]);

					ret = 0;
				}
			} else {
				ret = i2c_smbus_read_byte_data(data->client,
								addr | flag);
				if (ret < 0) {
					dev_err(&data->client->dev,
						"i2c read error\n");
					ret = -EIO;
				} else {
					SENSOR_DBG(DBG_LEVEL3,
						data->dbg_on, "%x", ret);
					data->regbuf[addr] = ret;
					ret = 0;
				}
			}
			break;

		/*only support len=1 currently*/
		case OPT_IMM:
			data->regbuf[addr] = action->operand2.data.immediate;
			break;

		case OPT_BEFORE:
			val = pop(data);
			data->regbuf[addr] = val;
			SENSOR_DBG(DBG_LEVEL3, data->dbg_on,
					"set reg buf:%x=%x", addr, val);
			break;

		case OPT_INDEX:
			val = data->private[action->operand2.data.index];
			data->regbuf[addr] = val;
			SENSOR_DBG(DBG_LEVEL3, data->dbg_on,
					"set reg buf:%x = %x", addr, val);
			break;

		default:
			dev_err(&data->client->dev, "Nonsupport operand2\n");
			return -EINVAL;
		}
		break;

	case OPT_INDEX:
		val = data_operand(data, &action->operand2);
		data->private[action->operand1.data.index] = val;

		SENSOR_DBG(DBG_LEVEL3, data->dbg_on,
				"set privated data:pri[%d]=%x",
				action->operand1.data.index, val);
	     break;

	case OPT_REG:
		addr = action->operand1.data.reg.addr;
		flag = action->operand1.data.reg.flag;
		len = action->operand1.data.reg.len;

		val = data_operand(data, &action->operand2);

		SENSOR_DBG(DBG_LEVEL3, data->dbg_on, "write %s reg: %x=%x",
						len > 1 ? "word" : "byte",
						(unsigned int)addr|flag, val);

		/*only support byte/word write currently*/
		if (len == 2) {
			/*always update reg buf when write*/
			*(u16 *)&data->regbuf[addr] = val;
			ret = i2c_smbus_write_word_data(data->client,
						addr | flag, val);
		} else {
			data->regbuf[addr] = val;
			ret = i2c_smbus_write_byte_data(data->client,
						addr | flag, val);
		}
		break;

	default:
		dev_err(&data->client->dev, "Nonsupport operand1\n");
		return -EINVAL;
	}

	return ret;
}

/*data operations*/
int (*data_op_array[OP_RESERVE])(int, int);

#define __BUILD_DATA_BASIC_OP(op, operation)		\
static inline int data_op_##op(int op1, int op2)	\
{							\
	return operation;				\
}

#define __BUILD_DATA_ENDIAN_OP(op, operator, sign)	\
static inline int data_op_##op(int op1, int op2)	\
{							\
	return (sign)operator(op1);			\
}

__BUILD_DATA_BASIC_OP(logic_eq, op1 == op2)
__BUILD_DATA_BASIC_OP(logic_neq, op1 != op2)
__BUILD_DATA_BASIC_OP(logic_greater, op1 > op2)
__BUILD_DATA_BASIC_OP(logic_less, op1 < op2)
__BUILD_DATA_BASIC_OP(logic_ge, op1 >= op2)
__BUILD_DATA_BASIC_OP(logic_le, op1 <= op2)
__BUILD_DATA_BASIC_OP(logic_and, op1 && op2)
__BUILD_DATA_BASIC_OP(logic_or, op1 || op2)

__BUILD_DATA_BASIC_OP(ari_add, op1 + op2)
__BUILD_DATA_BASIC_OP(ari_sub, op1 - op2)
__BUILD_DATA_BASIC_OP(ari_mul, op1 * op2)
__BUILD_DATA_BASIC_OP(ari_div, op1 / op2)
__BUILD_DATA_BASIC_OP(ari_mod, op1 % op2)

__BUILD_DATA_BASIC_OP(bit_or, op1 | op2)
__BUILD_DATA_BASIC_OP(bit_and, op1 & op2)
__BUILD_DATA_BASIC_OP(bit_lsl, op1 << op2)
__BUILD_DATA_BASIC_OP(bit_lsr, op1 >> op2)

__BUILD_DATA_ENDIAN_OP(be16, be16_to_cpu, s16)
__BUILD_DATA_ENDIAN_OP(be16u, be16_to_cpu, u16)
__BUILD_DATA_ENDIAN_OP(le16, le16_to_cpu, s16)
__BUILD_DATA_ENDIAN_OP(le16u, le16_to_cpu, u16)

__BUILD_DATA_ENDIAN_OP(be32, be32_to_cpu, s32)
__BUILD_DATA_ENDIAN_OP(le32, le32_to_cpu, s32)
__BUILD_DATA_ENDIAN_OP(be24, be24_to_cpu, s32)
__BUILD_DATA_ENDIAN_OP(le24, le24_to_cpu, s32)

static inline int data_op_bit_nor(int op1, int op2)
{
	return ~op1;
}

static inline int data_op_min(int op1, int op2)
{
	return min(op1, op2);
}

static inline int data_op_max(int op1, int op2)
{
	return max(op1, op2);
}

static inline int data_op_comm(int op1, int op2)
{
	return op1 > op2 ? 1 : 0;
}

static int sensor_exec_data_action(struct sensor_data *data,
				struct data_action *action)
{
        int op1, op2 = 0;
        int result;

        SENSOR_DBG(DBG_LEVEL4, data->dbg_on, "actions:%p", action);

        if (action->op == OP_ACCESS)
                return data_op_access(data, action);
        else if (action->op > OP_ACCESS && action->op < OP_RESERVE) {
		/*other data operations*/
	        op1 = data_operand(data, &action->operand1);
		if (action->op < OP_BIT_NOR)
			op2 = data_operand(data, &action->operand2);
		result = data_op_array[action->op](op1, op2);
		push(data, result);

		SENSOR_DBG(DBG_LEVEL3, data->dbg_on, "0x%x %s 0x%x = 0x%x",
                        op1, action_debug[action->op], op2, result);
		return 0;
	} else
		return -EINVAL;
}

#define RET_RETURN	2
static int sensor_exec_actions(struct sensor_data *data,
		int num, struct lowlevel_action *actions)
{
	int ret = 0;
	int num_con;
	int num_if;
	int num_else;
	int inx;
	p_extern_c p = NULL;

	while (num > 0) {
		SENSOR_DBG(DBG_LEVEL4, data->dbg_on,
				"%s %p actions:%p num:%d",
				data->config->input_name, data,
				actions, (int)num);

		switch (actions->type) {
		case DATA:
			ret = sensor_exec_data_action(data,
					&actions->action.data);
			if (ret) {
				dev_err(&data->client->dev,
					"[%d]%s\n", __LINE__, __func__);
				return ret;
			}
			actions++;
			num--;
			break;

		case IFELSE:
			num_con = actions->action.ifelse.num_con;
			num_if = actions->action.ifelse.num_if;
			num_else = actions->action.ifelse.num_else;

			/*skip ifelse action itself*/
			actions++;
			num--;

			ret = sensor_exec_actions(data, num_con, actions);
			if (ret) {
				if (ret != RET_RETURN)
					dev_err(&data->client->dev,
						"[%d]%s\n", __LINE__, __func__);
				return ret;
			}
			actions += num_con;
			num -= num_con;

			if (pop(data)) {
				SENSOR_DBG(DBG_LEVEL3, data->dbg_on,
					"true of if action:%p", actions);

				ret = sensor_exec_actions(data,
							num_if, actions);
				if (ret) {
					if (ret != RET_RETURN)
						dev_err(&data->client->dev,
						"[%d]%s\n", __LINE__, __func__);
					return ret;
				}
			} else {
				SENSOR_DBG(DBG_LEVEL3, data->dbg_on,
						"false of if action:%p",
						actions);

				ret = sensor_exec_actions(data,
						num_else, actions + num_if);
				if (ret) {
					if (ret != RET_RETURN)
						dev_err(&data->client->dev,
						"[%d]%s\n", __LINE__, __func__);
					return ret;
				}
			}
			actions += (num_if + num_else);
			num -= (num_if + num_else);
			break;

		case SLEEP:
			msleep(actions->action.sleep.ms);
			actions++;
			num--;
			break;

		case RETURN:
			return RET_RETURN;

		case EXTERNC:
			SENSOR_DBG(DBG_LEVEL3, data->dbg_on,
					"extern c action:%p", actions);

			inx = actions->action.externc.index;

			/*Don't use sensor_externc_lock directly for sake of performance
			  use a reference count to aovid unregister extern_c here*/
			mutex_lock(&sensor_externc_lock);
			extern_c_ref_cnt++;
			mutex_unlock(&sensor_externc_lock);
			if (inx < MAX_EXTERN_C && (p = extern_c_array[inx])) {
				ret = (*p)(data);
				if (ret) {
					dev_err(&data->client->dev,
						"[%d]%s exec extern c %p\n",
						__LINE__, __func__, p);
				}
			} else {
				dev_err(&data->client->dev,
						"[%d]%s\n", __LINE__, __func__);
				ret = -EINVAL;
			}
			mutex_lock(&sensor_externc_lock);
			extern_c_ref_cnt--;
			mutex_unlock(&sensor_externc_lock);

			if (ret)
				return ret;
			actions++;
			num--;
			break;

		default:
			dev_err(&data->client->dev,
					"[%d]%s\n", __LINE__, __func__);
			return -EINVAL;
		}
	}
	return ret;
}

/*
* num: how many valid actions to exec
* actions: lowlevel actions table
*/
static int sensor_exec_lowlevel_actions(struct sensor_data *data,
		int num, struct lowlevel_action *actions)
{
	int ret;

	ret = sensor_exec_actions(data, num, actions);
	if (ret == RET_RETURN) {
		SENSOR_DBG(DBG_LEVEL3, data->dbg_on, "return action");
		return 0;
	}
	return ret;
}

/*
* exec of user defined sensor action
*/
static int sensor_exec_sensor_action(struct sensor_data *data,
			enum sensor_action type)
{
	int ret = 0;
	int num = data->config->indexs[type].num;

	if (num) {
		int index = data->config->indexs[type].index;
		struct lowlevel_action *action_table =
			(struct lowlevel_action *)&data->config->actions;
		struct lowlevel_action *actions = &action_table[index];

		ret = sensor_exec_lowlevel_actions(data, num, actions);
	}

	return ret;
}

/*
* interval: poll interal in ms
*/
static int sensor_update_odr(struct sensor_data *data, int interval)
{
	int ret = 0;
	int entry;
	int hz_interval;
	int index;
	int num;
	struct lowlevel_action *action_table =
		(struct lowlevel_action *)&data->config->actions;

	SENSOR_DBG(DBG_LEVEL1, data->dbg_on, "%s", data->config->input_name);

	if (!data->config->odr_entries)
		return ret;

	/*search odr table to get the nearest lower than requested interval*/
	/*fix me: follow lis3dhl acc driver*/
	if (!interval)
		interval = data->config->report_interval;

	if (!interval)
		hz_interval = 1000;
	else
		hz_interval = 1000/interval;

	for (entry = 0; entry < data->config->odr_entries; entry++) {
		if (data->config->odr_table[entry].hz >= hz_interval)
			break;
	}

	if (entry >= data->config->odr_entries) {
		dev_err(&data->client->dev, "Nonsupport interval\n");
		return -EINVAL;
	}

	index = data->config->odr_table[entry].index.index;
	num = data->config->odr_table[entry].index.num;
	ret = sensor_exec_lowlevel_actions(data, num, &action_table[index]);

	return ret;
}

static int sensor_update_range(struct sensor_data *data, int range)
{
	int ret = 0;
	int entry;
	int index;
	int num;
	struct lowlevel_action *action_table =
		(struct lowlevel_action *)&data->config->actions;

	SENSOR_DBG(DBG_LEVEL1, data->dbg_on, "%s", data->config->input_name);

	if (!data->config->range_entries)
		return ret;

	/*search range table*/
	for (entry = 0; entry < data->config->range_entries; entry++) {
		if (data->config->range_table[entry].range == range)
			break;
	}

	if (entry >= data->config->range_entries) {
		dev_err(&data->client->dev, "Nonsupport range\n");
		return -EINVAL;
	}

	index = data->config->range_table[entry].index.index;
	num = data->config->range_table[entry].index.num;
	return sensor_exec_lowlevel_actions(data, num, &action_table[index]);
}

/*
* Only be called in probe
*/
static int sensor_init(struct sensor_data *data)
{
	int ret = 0;
	int id;

	SENSOR_DBG(DBG_LEVEL1, data->dbg_on, "%s", data->config->input_name);

	/*common ID check*/
	if (data->config->id_reg_addr != SENSOR_INVALID_REG) {
		ret = i2c_smbus_read_byte_data(data->client,
			data->config->id_reg_addr | data->config->id_reg_flag);
		if (ret < 0) {
			dev_err(&data->client->dev, "Sensor get ID error\n");
			return ret;
		}

		for (id = 0; id < MAX_DEV_IDS; id++) {
			if (data->config->id[id] == ret)
				break;
		}
		if (id >= MAX_DEV_IDS) {
			dev_err(&data->client->dev, "No matched ID %d\n", ret);
			return -ENODEV;
		}
	}

	return sensor_exec_sensor_action(data, INIT);
}

static int sensor_deinit(struct sensor_data *data)
{
	SENSOR_DBG(DBG_LEVEL1, data->dbg_on, "%s", data->config->input_name);

	return sensor_exec_sensor_action(data, DEINIT);
}

static int sensor_enable(struct sensor_data *data)
{
	int ret = 0;

	SENSOR_DBG(DBG_LEVEL1, data->dbg_on, "%s", data->config->input_name);

	if (data->state != STATE_EN) {
		ret = sensor_exec_sensor_action(data, ENABLE);
		data->state = STATE_EN;
		sensor_launch_work(data);
	}

	return ret;
}

static int sensor_disable(struct sensor_data *data)
{
	int ret = 0;

	SENSOR_DBG(DBG_LEVEL1, data->dbg_on, "%s", data->config->input_name);

	if (data->state == STATE_EN) {
		if (INT != data->config->method) {
			data->launched = 0;

			if (data->hrtimer_running) {
				data->hrtimer_running = false;
				hrtimer_cancel(&data->work_timer);
			}
		}

		ret = sensor_exec_sensor_action(data, DISABLE);
		data->state = STATE_DIS;
	}

	return ret;
}

static int sensor_suspend(struct device *dev)
{
	int i;
	struct i2c_client *client = to_i2c_client(dev);
	struct sensor_data *data = i2c_get_clientdata(client);
	struct mutex *lock = data->lock;
	int num = data->config->shared_nums;

	SENSOR_DBG(DBG_LEVEL1, data->dbg_on, "%s", data->config->input_name);

	if (INT == data->config->method) {
		int irq = gpio_to_irq(data->gpio);
		disable_irq(irq);
	}

	mutex_lock(lock);

	for (i = 0; i < num; i++, data++) {
		data->state_suspend = data->state;
		sensor_disable(data);
		data->state = STATE_SUS;
	}

	mutex_unlock(lock);

	return 0;
}

static int sensor_resume(struct device *dev)
{
	int i;
	struct i2c_client *client = to_i2c_client(dev);
	struct sensor_data *data = i2c_get_clientdata(client);
	struct mutex *lock = data->lock;
	int num = data->config->shared_nums;

	SENSOR_DBG(DBG_LEVEL1, data->dbg_on, "%s", data->config->input_name);

	if (INT == data->config->method) {
		int irq = gpio_to_irq(data->gpio);
		enable_irq(irq);
	}

	mutex_lock(lock);

	for (i = 0; i < num; i++, data++) {
		if (data->state_suspend == STATE_EN)
			sensor_enable(data);
	}

	mutex_unlock(lock);

	return 0;
}

/*Fix me, only support up to 32 sensor drivers*/
static int general_sensor_nums;
/*sync access of below*/
static DEFINE_MUTEX(sensor_proc_lock);
static struct sensor_data *sensor_data_tbl[MAX_SENSOR_DRIVERS] = {NULL};
static struct sensor_rawdata_proc *rawdata_proc_tbl[MAX_SENSOR_DRIVERS] = {NULL};

static inline int sensor_add_data(struct sensor_data *data_new)
{
	int i;
	int ret = 0;

	mutex_lock(&sensor_proc_lock);
	for (i = 0; i < MAX_SENSOR_DRIVERS; i++) {
		struct sensor_data *data = sensor_data_tbl[i];

		if (!data) {
			sensor_data_tbl[i] = data_new;
			break;
		}
	}
	if (i >= MAX_SENSOR_DRIVERS) {
		ret = -EINVAL;
	}
	mutex_unlock(&sensor_proc_lock);
	return ret;
}

static inline int sensor_remove_data(struct sensor_data *data_new)
{
	int i;
	int ret = 0;

	mutex_lock(&sensor_proc_lock);
	for (i = 0; i < MAX_SENSOR_DRIVERS; i++) {
		struct sensor_data *data = sensor_data_tbl[i];

		if (data == data_new) {
			sensor_data_tbl[i] = NULL;
			break;
		}
	}
	if (i >= MAX_SENSOR_DRIVERS) {
		ret = -EINVAL;
	}
	mutex_unlock(&sensor_proc_lock);
	return ret;
}

int sensor_register_rawdata_proc(struct sensor_rawdata_proc *proc_new)
{
	int i;
	int ret = 0;

	SENSOR_DBG(DBG_LEVEL3, DBG_ALL_SENSORS, "%s", proc_new->name);

	mutex_lock(&sensor_proc_lock);
	for (i = 0; i < MAX_SENSOR_DRIVERS; i++) {
		struct sensor_rawdata_proc *proc = rawdata_proc_tbl[i];

		if (!proc) {
			rawdata_proc_tbl[i] = proc_new;
			break;
		}
	}
	if (i >= MAX_SENSOR_DRIVERS) {
		ret = -EINVAL;
		goto out;
	}

	for (i = 0; i < MAX_SENSOR_DRIVERS; i++) {
		struct sensor_data *data = sensor_data_tbl[i];

		if (data && !strncmp(proc_new->name,
			data->config->name, strlen(proc_new->name))) {

			if (data->rawdata_proc)
				printk(KERN_WARNING
					"Override new raw data proc for %s\n",
					data->config->input_name);

			data->rawdata_proc = proc_new;
			break;
		}
	}

out:
	mutex_unlock(&sensor_proc_lock);
	return ret;
}
EXPORT_SYMBOL(sensor_register_rawdata_proc);

int sensor_unregister_rawdata_proc(struct sensor_rawdata_proc *proc_new)
{
	int i;
	int ret = 0;

	SENSOR_DBG(DBG_LEVEL3, DBG_ALL_SENSORS, "%s", proc_new->name);

	mutex_lock(&sensor_proc_lock);
	for (i = 0; i < MAX_SENSOR_DRIVERS; i++) {
		struct sensor_rawdata_proc *proc = rawdata_proc_tbl[i];

		if (proc == proc_new) {
			rawdata_proc_tbl[i] = NULL;
			break;
		}
	}
	if (i >= MAX_SENSOR_DRIVERS) {
		ret = -EINVAL;
		goto out;
	}

	for (i = 0; i < MAX_SENSOR_DRIVERS; i++) {
		struct sensor_data *data = sensor_data_tbl[i];

		if (data && data->rawdata_proc == proc_new) {
			data->rawdata_proc = NULL;
			break;
		}
	}

out:
	mutex_unlock(&sensor_proc_lock);
	return ret;
}
EXPORT_SYMBOL(sensor_unregister_rawdata_proc);

int sensor_register_extern_c(p_extern_c p)
{
	int i;
	int ret = 0;

	SENSOR_DBG(DBG_LEVEL3, DBG_ALL_SENSORS, "%p", p);

	mutex_lock(&sensor_externc_lock);
	for (i = 0; i < MAX_EXTERN_C; i++) {
		if (!extern_c_array[i]) {
			extern_c_array[i] = p;
			break;
		}
	}
	if (i >= MAX_EXTERN_C)
		ret = -EINVAL;
	mutex_unlock(&sensor_externc_lock);
	return ret;
}
EXPORT_SYMBOL(sensor_register_extern_c);

int sensor_unregister_extern_c(p_extern_c p)
{
	int i;
	int ret = 0;

	SENSOR_DBG(DBG_LEVEL3, DBG_ALL_SENSORS, "%p", p);

	mutex_lock(&sensor_externc_lock);

	if (extern_c_ref_cnt > 0) {
		printk(KERN_ERR "Err: can't unregister externc %d",
				extern_c_ref_cnt);
		ret = -EBUSY;
		goto out;
	}

	for (i = 0; i < MAX_EXTERN_C; i++) {
		if (extern_c_array[i] == p) {
			extern_c_array[i] = NULL;
			break;
		}
	}
	if (i >= MAX_EXTERN_C)
		ret = -EINVAL;
out:
	mutex_unlock(&sensor_externc_lock);
	return ret;
}
EXPORT_SYMBOL(sensor_unregister_extern_c);

static int sensor_get_report_data(struct sensor_data *data)
{
	int ret = 0;
	int val;
	int (*proc)(struct sensor_data *, int);
#ifdef CONFIG_GENERAL_SENSOR_DEBUG
	ktime_t start;
#endif
	SENSOR_DBG(DBG_LEVEL3, data->dbg_on, "%s", data->config->input_name);
	sensor_time_start(data, &start);

	/*AXIS X*/
	if (data->config->indexs[GET_DATA_X].num) {
		ret = sensor_exec_sensor_action(data, GET_DATA_X);
		if (!ret) {
			if (stack_empty(&data->stack)) {
				SENSOR_DBG(DBG_LEVEL2, data->dbg_on,
					"%s no data", data->config->input_name);
				return ret;	/*no valid data*/
			}

			val = pop(data);

			if (data->rawdata_proc &&
				(proc = data->rawdata_proc->proc_x))
				val = (*proc)(data, val);

			SENSOR_DBG(DBG_LEVEL2, data->dbg_on, "%s X:%d",
					data->config->input_name, val);
			/*Fix me: REL_X/Y/Z == ABS_X/Y/Z*/
			input_event(data->input_dev,
				data->config->event_type, REL_X, val?val:1);
		}
	}

	/*AXIS Y*/
	if (data->config->indexs[GET_DATA_Y].num) {
		ret = sensor_exec_sensor_action(data, GET_DATA_Y);
		if (!ret) {
			if (stack_empty(&data->stack))
				return ret;
			val = pop(data);

			if (data->rawdata_proc &&
				(proc = data->rawdata_proc->proc_y))
				val = (*proc)(data, val);

			SENSOR_DBG(DBG_LEVEL2, data->dbg_on, "%s Y:%d",
					data->config->input_name, val);
			input_event(data->input_dev,
				data->config->event_type, REL_Y, val?val:1);
		}
	}

	/*AXIS Z*/
	if (data->config->indexs[GET_DATA_Z].num) {
		ret = sensor_exec_sensor_action(data, GET_DATA_Z);
		if (!ret) {
			if (stack_empty(&data->stack))
				return ret;
			val = pop(data);

			if (data->rawdata_proc &&
				(proc = data->rawdata_proc->proc_z))
				val = (*proc)(data, val);

			SENSOR_DBG(DBG_LEVEL2, data->dbg_on, "%s Z:%d",
					data->config->input_name, val);
			input_event(data->input_dev,
				data->config->event_type, REL_Z, val?val:1);
		}
	}

	input_sync(data->input_dev);

	sensor_time_end(data, &start);

	return ret;
}

static long unsigned int start, end;
static int report_event(void *data)
{
	int xyz[3] = { 0 };
	struct sensor_data *pdata = data;

	while(1)
	{
		/* wait for report event */
		wait_for_completion(&pdata->report_complete);

		mutex_lock(pdata->lock);

		if (!pdata->launched) {
			mutex_unlock(pdata->lock);
			continue;
		}

		sensor_get_report_data(pdata);

		mutex_unlock(pdata->lock);
       }

       return 0;
}

static ssize_t sensor_range_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sensor_data *data = i2c_get_clientdata(client);
	int range;

	mutex_lock(data->lock);
	range = data->range;
	mutex_unlock(data->lock);

	return sprintf(buf, "%d\n", range);
}

static ssize_t sensor_range_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long range;
	struct i2c_client *client = to_i2c_client(dev);
	struct sensor_data *data = i2c_get_clientdata(client);

	if (kstrtoul(buf, 0, &range))
		return -EINVAL;

	mutex_lock(data->lock);
	ret = sensor_update_range(data, range);
	if (ret < 0)
		goto err;
	data->range = range;
	ret = count;
err:
	mutex_unlock(data->lock);
	return ret;
}

static ssize_t attr_get_selftest(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sensor_data *data = i2c_get_clientdata(client);
	int val;
	int ret;

	SENSOR_DBG(DBG_LEVEL3, data->dbg_on, "%s", data->config->input_name);

	mutex_lock(data->lock);

	ret = sensor_exec_sensor_action(data, GET_SELFTEST);
	if (ret) {
		dev_err(dev, "%s\n", __func__);
		mutex_unlock(data->lock);
		return -EIO;
	}
	val = pop(data);

	mutex_unlock(data->lock);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_selftest(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sensor_data *data = i2c_get_clientdata(client);
	int ret;
	unsigned long val;

	SENSOR_DBG(DBG_LEVEL3, data->dbg_on, "%s", data->config->input_name);

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	mutex_lock(data->lock);

	/*make sure the data action will pop this data*/
	push(data, val);
	ret = sensor_exec_sensor_action(data, SET_SELFTEST);
	if (ret)
		dev_err(dev, "%s\n", __func__);

	mutex_unlock(data->lock);

	return size;
}

/*show routine for data action */
static ssize_t sensor_sysfs_data_acton_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);
	struct sensor_data *data = i2c_get_clientdata(client);
	struct sysfs_entry *entry = data->config->sysfs_table;
	unsigned long val;
	int index;
	int num;
	int i;
	struct lowlevel_action *action_table =
		(struct lowlevel_action *)&data->config->actions;

	SENSOR_DBG(DBG_LEVEL3, data->dbg_on, "%s", data->config->input_name);

	for (i = 0; i < data->config->sysfs_entries; i++, entry++) {
		if (!strcmp(attr->attr.name, entry->name))
			break;
	}
	if (i >= data->config->sysfs_entries) {
		dev_err(&client->dev, "[%d]%s\n", __LINE__, __func__);
		return -EIO;
	}

	mutex_lock(data->lock);

	index = data->config->sysfs_table[i].action.data.index_show.index;
	num = data->config->sysfs_table[i].action.data.index_show.num;
	ret = sensor_exec_lowlevel_actions(data, num, &action_table[index]);
	if (ret) {
		dev_err(&client->dev, "[%d]%s\n", __LINE__, __func__);
		mutex_unlock(data->lock);
		return -EIO;
	}

	val = pop(data);

	mutex_unlock(data->lock);

	SENSOR_DBG(DBG_LEVEL3, data->dbg_on, "%x", (int)val);

	return sprintf(buf, "%d\n", (int)val);
}

/* store routine for data action
*   series data actions, make sure the last action
*   is OP_ACCESS when prepare config
*/
static ssize_t sensor_sysfs_data_acton_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long val;
	struct i2c_client *client = to_i2c_client(dev);
	struct sensor_data *data = i2c_get_clientdata(client);
	struct sysfs_entry *entry = data->config->sysfs_table;
	int index;
	int num;
	int i;
	struct lowlevel_action *action_table =
		(struct lowlevel_action *)&data->config->actions;

	SENSOR_DBG(DBG_LEVEL3, data->dbg_on, "%s", data->config->input_name);

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	for (i = 0; i < data->config->sysfs_entries; i++, entry++) {
		if (!strcmp(attr->attr.name, entry->name))
			break;
	}
	if (i >= data->config->sysfs_entries) {
		dev_err(&client->dev, "[%d]%s\n", __LINE__, __func__);
		return -EIO;
	}

	mutex_lock(data->lock);

	push(data, val);

	SENSOR_DBG(DBG_LEVEL3, data->dbg_on, "%x", (int)val);

	index = data->config->sysfs_table[i].action.data.index_store.index;
	num = data->config->sysfs_table[i].action.data.index_store.num;
	ret = sensor_exec_lowlevel_actions(data, num, &action_table[index]);
	if (ret) {
		dev_err(&client->dev, "[%d]%s\n", __LINE__, __func__);
		mutex_unlock(data->lock);
		return -EIO;
	}

	mutex_unlock(data->lock);

	return count;
}

/*
* for Poll and Mix method, need to lauch delayed work when enable/resume
* while for Int, no sw need to launch except enable hw
*  mutex lock must be held when calling this function
*/
static void sensor_launch_work(struct sensor_data *data)
{
	ktime_t poll_delay;
	SENSOR_DBG(DBG_LEVEL1, data->dbg_on, "%s", data->config->input_name);

	if (INT == data->config->method) {
		SENSOR_DBG(DBG_LEVEL1, data->dbg_on, "%s skip int",
				data->config->input_name);
		return;
	}

	if (data->launched) {
		SENSOR_DBG(DBG_LEVEL1, data->dbg_on, "%s already launch",
					data->config->input_name);
		return;
	}

	if (data->poll_interval > 0) {
		/*Poll method*/
		 poll_delay = ktime_set(0, data->poll_interval * NSEC_PER_MSEC);
	} else {
		/*MIX method*/
		data->report_cnt = data->config->report_cnt;
		poll_delay = ktime_set(0, data->config->report_interval * NSEC_PER_MSEC);
	}
	data->hrtimer_running = true;
        hrtimer_start(&data->work_timer, poll_delay, HRTIMER_MODE_REL);
	data->launched = 1;
}

/*
* general polling func
* create a seperate work queue for all sensor drivers ?
*/
static enum hrtimer_restart sensor_poll_work(struct hrtimer *timer)
{
	ktime_t poll_delay;
        struct sensor_data *data = container_of((struct hrtimer *)timer,
                       struct sensor_data, work_timer);
	SENSOR_DBG(DBG_LEVEL3, data->dbg_on, "%s", data->config->input_name);

	if (data->state != STATE_EN) {
		data->launched = 0;
		return HRTIMER_NORESTART;
	}
	complete(&data->report_complete);

	/*Poll or Mix method*/
	if (data->poll_interval > 0) {
		poll_delay = ktime_set(0, data->poll_interval * NSEC_PER_MSEC);
	} else {
		if (data->report_cnt-- > 0) {
			poll_delay = ktime_set(0, data->config->report_interval * NSEC_PER_MSEC);
		} else {
			data->launched = 0;
		}
	}
	hrtimer_start(&data->work_timer, poll_delay, HRTIMER_MODE_REL);

	return HRTIMER_NORESTART;
}

/*
* this is somewhat hardcode for special sensors
* which ack is not common and performance critical
* although these action can be integrate in get_data action
*/
static int fixup_accel(struct sensor_data *data)
{
	return 0;
}

static int fixup_apds990x(struct sensor_data *data)
{
	return 0;
}

struct sensor_ack_fixup {
	char *name;
	int (*fn)(struct sensor_data *);
};

static struct sensor_ack_fixup fixup_table[] = {
	{"accel", fixup_accel},
	{"apds990x", fixup_apds990x},
	{NULL, NULL}
};

static int sensor_ack_int_fixup(struct sensor_data *data)
		__attribute__((unused));
static int sensor_ack_int_fixup(struct sensor_data *data)
{
	struct sensor_ack_fixup *fixup = fixup_table;

	while (fixup->name) {
		if (!strcmp(fixup->name, data->config->name))
			return fixup->fn(data);

		fixup++;
	}

	return 0;
}

/*
* general irq thread handler
*/
static irqreturn_t sensor_interrupt_handler(int irq, void *pri)
{
	struct sensor_data *data = (struct sensor_data *)pri;
	enum method_get_data method = data->config->method;
	int ret;

retry_int:
	mutex_lock(data->lock);

	if (data->config->irq_serialize) {
		if (data->multi_index != *data->share_irq_seq) {
			mutex_unlock(data->lock);
			cond_resched();
			goto retry_int;
		} else {
			(*data->share_irq_seq)++;
			if (*data->share_irq_seq >= data->config->shared_nums)
				*data->share_irq_seq = 0;
		}
	}

	SENSOR_DBG(DBG_LEVEL3, data->dbg_on, "%s", data->config->input_name);

	ret = sensor_exec_sensor_action(data, INT_ACK);
	if (ret) {
		printk(KERN_ERR "[%d]%s exec INT action error\n",
				 __LINE__, __func__);
		ret = IRQ_HANDLED;
		goto irq_out;
	} else {
		ret = pop(data);
		if (ret == IRQ_NONE) {
			SENSOR_DBG(DBG_LEVEL3, data->dbg_on,
						"unexpected irq");
			goto irq_out;
		}
	}

	/*especially for the first time, similar with work func */
	if (data->state != STATE_EN) {
		ret = IRQ_HANDLED;
		goto irq_out;
	}

	if (method == INT) {
		ret = sensor_get_report_data(data);
		if (ret) {
			printk(KERN_ERR "[%d]%s get data error\n",
					 __LINE__, __func__);
			ret = IRQ_HANDLED;
			goto irq_out;
		}

	} else if (method == MIX)
		sensor_launch_work(data);
	else
		printk(KERN_ERR "Wrong method in sensor irq\n");

	ret = IRQ_HANDLED;
irq_out:
	mutex_unlock(data->lock);

	return ret;
}

/*
* poll: delayed work
* int:  thread irq func
* mix: thread irq func and delayed work, irq triger delayed work
*/
static int sensor_get_data_init(struct sensor_data *data)
{
	int ret = 0;
	enum method_get_data method = data->config->method;
	struct sched_param param = { .sched_priority = MAX_RT_PRIO-1 };

	SENSOR_DBG(DBG_LEVEL3, data->dbg_on, "%s", data->config->input_name);

	/*init delayed work or irq routine*/
	if (method == POLL || method == MIX)
	{
		hrtimer_init(&data->work_timer, CLOCK_MONOTONIC,HRTIMER_MODE_REL);
		data->work_timer.function = sensor_poll_work;
		data->hrtimer_running = false;

		init_completion(&data->report_complete);
		data->thread = kthread_run(report_event, data, "sensor_report_event");
		if (IS_ERR(data->thread)) {
			dev_err(&data->client->dev,
				"unable to create report_event thread\n");
			return ret;
		}
		sched_setscheduler_nocheck(data->thread, SCHED_FIFO, &param);

	}
	/*init irq*/
	if (method == INT || method == MIX) {
		int irq;
		unsigned long flags;

		/*only the fisrt one request gpio*/
		if (data->multi_index == 0) {
			gpio_request(data->gpio, data->config->name);
			gpio_direction_input(data->gpio);
		}

		flags = data->config->irq_flag;
		if (data->config->shared_nums > 1)
			flags |= IRQF_SHARED;

		irq = gpio_to_irq(data->gpio);

		ret = request_threaded_irq(irq, NULL, sensor_interrupt_handler,
					flags, data->config->input_name, data);
		if (ret < 0) {
			if (data->multi_index == 0)
				gpio_free(data->gpio);
			dev_err(&data->client->dev,
				"Fail to request irq:%d ret=%d\n", irq, ret);
			return ret;
		}
	}

	return ret;
}

static int sensor_input_init(struct sensor_data *data)
{
	int ret;
	struct input_dev *input;

	SENSOR_DBG(DBG_LEVEL3, data->dbg_on, "%s", data->config->input_name);

	input = input_allocate_device();
	if (!input) {
		dev_err(&data->client->dev, "input device allocate failed\n");
		return -ENOMEM;
	}

	input->name = data->config->input_name;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &data->client->dev;

	set_bit(data->config->event_type, input->evbit);

	if (data->config->indexs[GET_DATA_X].num) {
		if (data->config->event_type == EV_REL)
			set_bit(REL_X, input->relbit);
		else if (data->config->event_type == EV_ABS)
			set_bit(ABS_X, input->absbit);
	}

	if (data->config->indexs[GET_DATA_Y].num) {
		if (data->config->event_type == EV_REL)
			set_bit(REL_Y, input->relbit);
		else if (data->config->event_type == EV_ABS)
			set_bit(ABS_Y, input->absbit);
	}

	if (data->config->indexs[GET_DATA_Z].num) {
		if (data->config->event_type == EV_REL)
			set_bit(REL_Z, input->relbit);
		else if (data->config->event_type == EV_ABS)
			set_bit(ABS_Z, input->absbit);
	}

	ret = input_register_device(input);
	if (ret) {
		dev_err(&data->client->dev,
			"unable to register input device %s:%d\n",
			input->name, ret);
		goto err;
	}

	data->input_dev = input;
	return 0;
err:
	input_free_device(input);
	return ret;
}

static ssize_t sensor_interval_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sensor_data *data = i2c_get_clientdata(client);
	int interval;

	SENSOR_DBG(DBG_LEVEL3, data->dbg_on, "%s", data->config->input_name);

	mutex_lock(data->lock);
	interval = data->poll_interval;
	mutex_unlock(data->lock);

	return sprintf(buf, "%d\n", interval);
}

static ssize_t sensor_interval_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long interval;
	struct i2c_client *client = to_i2c_client(dev);
	struct sensor_data *data = i2c_get_clientdata(client);

	SENSOR_DBG(DBG_LEVEL1, data->dbg_on, "%s", data->config->input_name);

	if (kstrtoul(buf, 0, &interval))
		return -EINVAL;

	SENSOR_DBG(DBG_LEVEL1, data->dbg_on, "%d-->%d",
			data->poll_interval, (int)interval);

	/*check whether interval is belong to [min, max]*/
	if (data->config->min_poll_interval != SENSOR_INVALID_INTERVAL &&
			interval < data->config->min_poll_interval) {
		interval = data->config->min_poll_interval;
	}

	if (data->config->max_poll_interval != SENSOR_INVALID_INTERVAL &&
			interval > data->config->max_poll_interval) {
		interval = data->config->max_poll_interval;
	}

	mutex_lock(data->lock);

	/*enable, then set poll*/
	if (data->poll_interval == interval)
		goto out;

	sensor_update_odr(data, interval);
	data->poll_interval = interval;
	sensor_launch_work(data);

out:
	mutex_unlock(data->lock);

	return count;
}

static ssize_t sensor_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sensor_data *data = i2c_get_clientdata(client);
	int enabled;

	SENSOR_DBG(DBG_LEVEL1, data->dbg_on, "%s", data->config->input_name);

	mutex_lock(data->lock);

	enabled = (data->state == STATE_EN);

	mutex_unlock(data->lock);

	return sprintf(buf, "%d\n", enabled);
}

#define SENSOR_SYSFS_POWERON		1
#define SENSOR_SYSFS_POWERDOWN		0
static ssize_t sensor_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sensor_data *data = i2c_get_clientdata(client);
	unsigned long val;

	SENSOR_DBG(DBG_LEVEL1, data->dbg_on, "%s", data->config->input_name);

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	SENSOR_DBG(DBG_LEVEL1, data->dbg_on, " %x", (int)val);

	if (val != SENSOR_SYSFS_POWERON && val != SENSOR_SYSFS_POWERDOWN)
		return -EINVAL;

	mutex_lock(data->lock);

	/*both STATE_DIS and SATE_SUS -> STATE_EN*/
	if (val)
		sensor_enable(data);
	else
		sensor_disable(data);

	mutex_unlock(data->lock);

	return count;
}

static DEVICE_ATTR(poll, S_IRUGO|S_IWUSR,
		sensor_interval_show, sensor_interval_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR,
		sensor_enable_show, sensor_enable_store);

static struct attribute *sensor_default_attributes[] = {
	&dev_attr_poll.attr,
	&dev_attr_enable.attr,
	NULL
};

static struct attribute_group sensor_default_attribute_group = {
	.attrs = sensor_default_attributes
};

union sysfsapi_action {
	void (*invalid)(void);
	ssize_t (*show)(struct device *, struct device_attribute *, char *);
	ssize_t (*store)(struct device *,
		struct device_attribute *, const char *, size_t);
};

union sysfsapi_action sysfsapi_table[SENSOR_ACTION_RESERVE] = {
	 /*INIT*/
	 { .invalid = NULL },
	 { .invalid = NULL },
	 /*ENABLE*/
	 { .invalid = NULL },
	 { .invalid = NULL },
	 /*INT_ACK*/
	 { .invalid = NULL },
	 /*GET_DATE_X*/
	 { .invalid = NULL },
	 { .invalid = NULL },
	 { .invalid = NULL },
	 { .show = sensor_range_show },
	 { .store = sensor_range_store },
	 { .show = attr_get_selftest },
	 { .store = attr_set_selftest },
};

static struct device general_sensor_device;
static void sysfs_interfaces_link(struct device *dev)
{
	static int general_sensor_num = 0;
	int ret = 0;
	char name[32] = {0};

	sprintf(name, "sensor%d", general_sensor_num++);
	ret = sysfs_create_link(&general_sensor_device.kobj, &dev->kobj, name);
	if (ret) {
		dev_warn(dev, "Fail to link  %s\n", dev_name(dev));
	}
}
static int create_sysfs_interfaces(struct sensor_data *data)
{
	int ret = 0;
	struct device_attribute *dev_attr;
	int i;
	int sub = 0;

	SENSOR_DBG(DBG_LEVEL3, data->dbg_on, "%s", data->client->name);

	/*default poll and enable sysfs device files*/
	/*fix me: in remove. this will be obsolete after sync with user xml*/
	if (strlen(data->config->attr_name)) {
		sensor_default_attribute_group.name = data->config->attr_name;
		sub = 1;
	} else {
		sensor_default_attribute_group.name = NULL;
	}

	ret = sysfs_create_group(&data->attr_dev->kobj,
			&sensor_default_attribute_group);
	if (ret) {
		dev_err(data->attr_dev, "sysfs can not create group\n");
		return ret;
	}

	sysfs_interfaces_link(data->attr_dev);

	/*developer defined */
	if (!data->config->sysfs_entries)
		return ret;

	dev_attr = kzalloc(data->config->sysfs_entries *
			sizeof(struct device_attribute),
			GFP_KERNEL);
	if (!dev_attr) {
		dev_err(data->attr_dev, "Fail to alloc memory\n");
		return -ENOMEM;
	} else
		data->dev_attr = dev_attr;

	for (i = 0; i < data->config->sysfs_entries; i++, dev_attr++) {
		dev_attr->attr.name = data->config->sysfs_table[i].name;
		dev_attr->attr.mode = S_IRUGO|S_IWUSR;

		if (data->config->sysfs_table[i].type == DATA_ACTION) {
			dev_attr->show = sensor_sysfs_data_acton_show;
			dev_attr->store = sensor_sysfs_data_acton_store;
		} else if (data->config->sysfs_table[i].type == SENSOR_ACTION) {
			enum sensor_action action =
				data->config->sysfs_table[i].action.sensor.show;

			dev_attr->show = sysfsapi_table[action].show;

			action =
			  data->config->sysfs_table[i].action.sensor.store;
			dev_attr->store = sysfsapi_table[action].store;

			if (!dev_attr->store || !dev_attr->show) {
				dev_err(data->attr_dev, "Wrong sys action\n");
				goto err;
			}
		} else {
			dev_err(data->attr_dev, "Nonsupported action\n");
			goto err;
		}

		if (!sub) {
			if (device_create_file(data->attr_dev, dev_attr))
				goto err;
		} else {
			/*fix me. some are subdir*/
			if (device_create_file(data->attr_dev, dev_attr))
				goto err;
		}
	}

	return ret;

err:
	for (i = i - 1, dev_attr--; i >= 0; i--, dev_attr--)
		device_remove_file(data->attr_dev, dev_attr);

	kfree(data->dev_attr);
	data->dev_attr = NULL;

	return ret;
}

static void remove_sysfs_interfaces(struct sensor_data *data)
{
	int i;
	struct device_attribute *dev_attr = data->dev_attr;

	/*remove default */
	sysfs_remove_group(&data->attr_dev->kobj,
			&sensor_default_attribute_group);

	/*remove others*/
	for (i = 0; i < data->config->sysfs_entries; i++, dev_attr++)
		device_remove_file(data->attr_dev, dev_attr);

	kfree(data->dev_attr);
	data->dev_attr = NULL;

	/*release kobj for multi device*/
	if (data->config->shared_nums > 1) {
		device_del(data->attr_dev);
		data->attr_dev = NULL;
	}
}

static int sensor_rawdata_proc_init(struct sensor_data *data)
{
	int i;

	mutex_lock(&sensor_proc_lock);
	for (i = 0; i < MAX_SENSOR_DRIVERS; i++) {
		struct sensor_rawdata_proc *proc = rawdata_proc_tbl[i];

		if (proc && !strncmp(proc->name,
			data->config->name, strlen(proc->name))) {
			data->rawdata_proc = proc;
			goto out;
		}
	}
	data->rawdata_proc = NULL;
out:
	mutex_unlock(&sensor_proc_lock);
	return 0;
}

/*
* sensor private data init
*/
static int sensor_data_init(struct i2c_client *client,
			struct sensor_data *data,
			struct sensor_config *config)
{
	int ret = 0;
	int i;
	int nums = config->shared_nums;
	u8 *regbuf = (u8 *)&data[nums];
	/*share lock with the first one*/
	struct mutex *lock = (struct mutex *)&data->real_lock;

	SENSOR_DBG(DBG_LEVEL3, DBG_ALL_SENSORS, "shared nums:%d", nums);

	/*hook first private data for multi devices*/
	i2c_set_clientdata(client, data);
	mutex_init(lock);

	for (i = 0; i < nums; i++, data++) {
		/*init*/
		data->config = config;
		stack_init(&data->stack);
		data->lock = lock;
		data->regbuf = regbuf;
		data->client = client;
		data->state = STATE_DIS;
		data->state_suspend = STATE_DIS;
		data->launched = 0;
		data->poll_interval = data->config->default_poll_interval;
		data->range = data->config->default_range;

		if (config->method != POLL) {
			data->gpio =
				acpi_get_gpio_by_index(&client->dev, 0, NULL);
			if (data->gpio < 0) {
				dev_warn(&client->dev,
					"Fail to get gpio pin by ACPI\n");
				data->gpio = config->gpio_num;
			}

			if (data->gpio < 0) {
				dev_err(&client->dev, "Need to specify gpio\n");
				goto err;
			}
		}
		sensor_rawdata_proc_init(data);

		data->multi_index = i;
		data->share_irq_seq = regbuf + config->sensor_regs;

		/*init data->attr_dev*/
		if (nums == 1) {
			/*single device*/
			data->attr_dev = &client->dev;
		} else {
			/*multi device*/
			struct device *dev;

			dev = kzalloc(sizeof(struct device), GFP_KERNEL);
			if (!dev) {
				ret = -ENOMEM;
				goto err;
			}
			device_initialize(dev);
			dev->parent = &client->dev;
			dev_set_name(dev, config->input_name);

			ret = device_add(dev);
			if (ret) {
				dev_err(&client->dev,
					"Fail to register device\n");
				kfree(dev);
				goto err;
			}

			data->attr_dev = dev;

			dev_set_drvdata(dev, data);
		}

		config = (struct sensor_config *)((char *)config + config->size);
	}

	return 0;

err:
	for (i = i - 1, data--; i >= 0; i--, data--) {
		device_del(data->attr_dev);
		kfree(data->attr_dev);
		data->attr_dev = NULL;
	}
	return ret;
}

/*
* record registered driver for remove
*/
static int registered_drivers;
static int registered_devices;
static struct i2c_driver *i2c_sensor_drivers[MAX_SENSOR_DRIVERS] = { NULL };
static struct i2c_client *i2c_sensor_devices[MAX_SENSOR_DRIVERS] = { NULL };
static int failed_drivers;
static int failed_devices;
static char *i2c_failed_driver[MAX_SENSOR_DRIVERS] = { NULL };
static struct i2c_client *i2c_failed_device[MAX_SENSOR_DRIVERS] = { NULL };

static int sensor_probe(struct i2c_client *client,
			const struct i2c_device_id *devid)
{
	int ret = 0;
	struct sensor_config *config;
	struct sensor_data *data = NULL;
	int i;

	SENSOR_DBG(DBG_LEVEL3, DBG_ALL_SENSORS, "i2c device:%s", devid->name);

	config = (struct sensor_config *)devid->driver_data;

	if (!i2c_check_functionality(client->adapter,
		I2C_FUNC_I2C | I2C_FUNC_SMBUS_I2C_BLOCK)) {
		dev_err(&client->dev, "client not i2c capable\n");
		return -ENODEV;
	}

	data = kzalloc(config->shared_nums * sizeof(struct sensor_data) +
			config->sensor_regs + 1/*share irq sequence*/,
			GFP_KERNEL);
	if (!data) {
		return -ENOMEM;
		dev_err(&client->dev, "Fail to alloc memory\n");
	}

	ret = sensor_data_init(client, data, config);
	if (ret) {
		dev_err(&client->dev, "sensor_data_init\n");
		kfree(data);
		return ret;
	}

	for (i = 0; i < config->shared_nums; i++) {
		/*don't power on device*/
		ret = sensor_init(data + i);
		if (ret) {
			dev_err(&client->dev, "sensor_init\n");
			i2c_failed_driver[failed_drivers++] = config->name;
			i2c_failed_device[failed_devices++] = client;
			goto err;
		}

		ret = sensor_input_init(data + i);
		if (ret) {
			dev_err(&client->dev, "sensor_input_init\n");
			goto err;
		}

		ret = sensor_get_data_init(data + i);
		if (ret) {
			dev_err(&client->dev, "sensor_get_data_init\n");
			goto err;
		}

		ret = create_sysfs_interfaces(data + i);
		if (ret) {
			dev_err(&client->dev, "create_sysfs_interfaces\n");
			goto err;
		}

		if (general_sensor_nums <= MAX_SENSOR_DRIVERS - 1) {
			(data + i)->dbg_on = (1 << general_sensor_nums);
			general_sensor_nums++;
		} else {
			dev_err(&client->dev, "Too many sensor drivers\n");
			goto err;
		}

		sensor_add_data(data + i);
	}

	return ret;
err:
	if (0 == i)
		kfree(data);
	return ret;
}

/*
* disable sensor device
* remove all info related to this sensor device which add in probe
*/
static int sensor_remove(struct i2c_client *client)
{
	struct sensor_data *data = i2c_get_clientdata(client);
	int num = data->config->shared_nums - 1;

	for (data += num; num >= 0; num--, data--) {
		remove_sysfs_interfaces(data);

		if (data->config->method == INT ||
				data->config->method == MIX) {
			free_irq(gpio_to_irq(data->gpio), data);
			if (num == 0)
				gpio_free(data->gpio);
		}

		input_unregister_device(data->input_dev);
		sensor_disable(data);
		sensor_deinit(data);

		sensor_remove_data(data);
	}

	data++;
	kfree(data);

	return 0;
}

static const struct dev_pm_ops sensor_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(sensor_suspend, sensor_resume)
};

static int register_sensor_driver(struct sensor_config *config)
{
	struct i2c_device_id *i2c_ids;
	struct i2c_driver *sensor_driver;
	int ret;

	sensor_driver = kzalloc(sizeof(struct i2c_driver) +
			MAX_DEV_IDS * sizeof(struct i2c_device_id),
			GFP_KERNEL);
	if (!sensor_driver) {
		ret = -ENOMEM;
		printk(KERN_ERR "Fail to alloc memory in %s\n", __func__);
		return ret;
	}

	sensor_driver->driver.name = config->name;
	sensor_driver->driver.owner = THIS_MODULE;
#ifdef CONFIG_PM_SLEEP
	sensor_driver->driver.pm = &sensor_pm_ops;
#endif
	sensor_driver->probe = sensor_probe;
	sensor_driver->remove = sensor_remove;

	/*set name and private config for this sensor*/
	i2c_ids = (struct i2c_device_id *)&sensor_driver[1];
	sensor_driver->id_table = i2c_ids;

	i2c_ids->driver_data = (unsigned long)config;
	strlcpy(i2c_ids->name, config->name, I2C_NAME_SIZE);
	SENSOR_DBG(DBG_LEVEL3, DBG_ALL_SENSORS,
			"register i2c driver:%s", i2c_ids->name);

	ret = i2c_add_driver(sensor_driver);
	if (ret) {
		kfree(sensor_driver);
		printk(KERN_ERR "Fail to register i2c driver\n");
	} else
		i2c_sensor_drivers[registered_drivers++] = sensor_driver;

	return ret;
}

static int register_sensor_device(struct sensor_config *config)
{
	int ret = 0;
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	struct i2c_board_info info;

	memset(&info, 0, sizeof(info));
	strlcpy(info.type, config->name, I2C_NAME_SIZE);
	adapter = i2c_get_adapter(config->i2c_bus);
	if (!adapter)
		return -ENODEV;
	else {
		int addr;
		int valid_dev = 0;

		for (addr = 0; addr < MAX_I2C_ADDRS &&
			config->i2c_addrs[addr] != INVALID_I2C_ADDR; addr++) {

			info.addr = config->i2c_addrs[addr];

			client = i2c_new_device(adapter, &info);
			if (!client) {
				printk(KERN_ERR "Fail to add i2c device%d:%d\n",
						config->i2c_bus, info.addr);
				continue;
			}

			/*more than one addr, check whether
			this is valid device for current addr*/
			if (config->i2c_addrs[1] != INVALID_I2C_ADDR) {
				ret = i2c_smbus_read_byte_data(client,
						config->test_reg_addr);
				if (ret < 0) {
					i2c_unregister_device(client);
					continue;
				}
			}

			i2c_sensor_devices[registered_devices++] = client;
			valid_dev = 1;
			break;
		}

		if (!valid_dev)
			return -EINVAL;
		else
			return 0;
	}
}

static void unregister_sensor_drivers(void)
{
	int num = registered_drivers - 1;

	while (num >= 0) {
		if (i2c_sensor_drivers[num]) {
			i2c_del_driver(i2c_sensor_drivers[num]);
			kfree(i2c_sensor_drivers[num]);
			i2c_sensor_drivers[num] = NULL;
		}

		num--;
	}

	registered_drivers = 0;
}

static void unregister_failed_driver(char *name) __attribute__((unused));
static void unregister_failed_driver(char *name)
{
	int failed_num = failed_drivers - 1;

	while (failed_num >= 0) {
		char *failed_name = i2c_failed_driver[failed_num];
		if (failed_name && !strcmp(failed_name, name)) {
			int num = registered_drivers - 1;
			while (num >= 0) {
				struct i2c_driver *drv = i2c_sensor_drivers[num];
				if (drv && !strcmp(drv->driver.name, name)) {
					i2c_del_driver(drv);
					kfree(i2c_sensor_drivers[num]);
					i2c_sensor_drivers[num] = NULL;
					i2c_failed_driver[failed_num] = NULL;
					return;
				}
				num--;
			}
		}
		failed_num--;
	}
}

static void unregister_sensor_devices(void)
{
	int num = registered_devices - 1;

	while (num >= 0) {
		if (i2c_sensor_devices[num]) {
			/*fix me*/
			i2c_unregister_device(i2c_sensor_devices[num]);
			i2c_sensor_devices[num] = NULL;
		}

		num--;
	}
}

static void unregister_failed_devices(void)
{
	int failed_num = failed_devices - 1;
	while (failed_num >= 0) {
		struct i2c_client *fail_cli = i2c_failed_device[failed_num];
		int num = registered_devices - 1;
		while (num >= 0) {
			struct i2c_client *reg_cli = i2c_sensor_devices[num];
			if (reg_cli && reg_cli == fail_cli){
				i2c_unregister_device(fail_cli);
				i2c_failed_device[failed_num] = NULL;
				i2c_sensor_devices[num] = NULL;
			}
			num--;
		}
		failed_num--;
	}
	failed_devices = 0;
}

/*
* register one i2c driver and i2c device for each config(maybe multi)
* num: how many sensor config
*/
static int sensor_parse_config(int num, struct sensor_config *configs)
{
	int ret = 0;
	int shared_nums;

	general_sensor_nums = 0;
	while (num > 0) {
		/*two methods to support multi drivers for one i2c slave*/
		unregister_failed_devices();
		//unregister_failed_driver(configs->name);

		ret = register_sensor_driver(configs);
		if (ret) {
			printk(KERN_ERR "Fail to register sensor driver\n");
		}

		if (configs->i2c_bus != INVALID_I2C_BUS) {
			ret = register_sensor_device(configs);
			/*skip this error in case of multi register*/
			if (ret) {
				printk(KERN_WARNING
				"Fail to register sensor device\n");
			}
		}

		shared_nums = configs->shared_nums;
		num -= shared_nums;

		while (shared_nums > 0) {
			shared_nums--;
			configs = (struct sensor_config *)
				((char *)configs + configs->size);
		}
	}

	return 0;
}

static struct sensor_config_image *sensor_image;
static int sensor_general_attached;
static int start_method;
static int fw_finish;
static void sensor_firmware_cb(const struct firmware *fw_entry, void *ctx)
{
	int ret;

	if (!fw_entry) {
		printk(KERN_ERR "Fail to request sensor firmware\n");
		fw_finish = 1;
		return;
	}

	sensor_image = (struct sensor_config_image *)fw_entry->data;
	if (start_method != SG_FORCE_START &&
		(sensor_image->flags & SG_FLAGS_BOOT_DISABLE)) {
		ret = -EINVAL;
		goto out;
	}

	sensor_image = kmalloc(fw_entry->size, GFP_KERNEL);
	if (!sensor_image) {
		printk(KERN_ERR "Fail to alloc mem\n");
		ret = -ENOMEM;
		goto out;
	}
	memcpy((void *)sensor_image, fw_entry->data, fw_entry->size);

#ifdef CONFIG_GENERAL_SENSOR_DEBUG
	/*set default debug switch*/
	sensor_general_debug_sensors = sensor_image->dbg_sensors;
	sensor_general_debug_level = sensor_image->dbg_level;
#endif

	ret = sensor_parse_config(sensor_image->num,
			(struct sensor_config *)&sensor_image->configs);
	if (ret) {
		printk(KERN_ERR "Fail to parse config image\n");
		kfree(sensor_image);
		sensor_image = NULL;
	} else
		sensor_general_attached = 1;
out:
	release_firmware(fw_entry);
	fw_finish = 1;
	return;
}

static int sensor_general_start(int start)
{
	int ret = 0;
	start_method = start;
	fw_finish = 0;
	ret = request_firmware_nowait(THIS_MODULE, true,
			GENERAL_SENSOR_FIRMWARE,
			&general_sensor_device, GFP_KERNEL, NULL,
			sensor_firmware_cb);

	if (ret) {
		SENSOR_DBG(DBG_LEVEL3, DBG_ALL_SENSORS,
                        "Failed to request firmware!\n");
	}
	else {
		while (fw_finish !=1) msleep(10);
	}

	return ret;
}

static ssize_t sensor_general_start_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	if (!sensor_general_attached && val) {
		ret = sensor_general_start(val);
		if (ret)
			printk(KERN_ERR "Fail to start general sensor\n");
	} else if (sensor_general_attached && !val) {
		printk(KERN_DEBUG "[%d]%s, unattach general sensor driver\n",
						__LINE__, __func__);
		unregister_sensor_drivers();
		unregister_sensor_devices();
		kfree(sensor_image);
		sensor_image = NULL;
		sensor_general_attached = 0;
	}

	return count;
}

static DEVICE_ATTR(start, S_IRUGO|S_IWUSR,
			NULL, sensor_general_start_store);

#ifdef CONFIG_GENERAL_SENSOR_DEBUG

static ssize_t sensor_general_debug_sensors_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;

	ret = sprintf(buf, "%08x\n", sensor_general_debug_sensors);
	return ret;
}

static ssize_t sensor_general_debug_sensors_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	sensor_general_debug_sensors = val;

	return count;
}

static DEVICE_ATTR(dbgsensors, S_IRUGO|S_IWUSR,
		sensor_general_debug_sensors_show,
		sensor_general_debug_sensors_store);

static ssize_t sensor_general_debug_level_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret;

	ret = sprintf(buf, "%d\n", sensor_general_debug_level);

	return ret;
}

static ssize_t sensor_general_debug_level_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	sensor_general_debug_level = val;

	return count;
}

static DEVICE_ATTR(dbglevel, S_IRUGO|S_IWUSR,
		sensor_general_debug_level_show,
		sensor_general_debug_level_store);

static ssize_t sensor_general_time_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int i;

	for (i = 0; i < general_sensor_nums; i++) {
		if (sensor_general_time & (1 << i)) {
			s64 ns = ktime_to_ns(sensor_time[i].time);
			int times = sensor_time[i].times;
			s64 onetime = ns;

			if (times)
				do_div(onetime, times);
			else
				onetime = 0;
			ret += sprintf(buf + ret,
			"Sensor %d times:%10d time:%20lld ns %10d ns/times\n",
			i, times, ns, (int)onetime);
		} else {
			sensor_time[i].times = 0;
			sensor_time[i].time.tv64 = 0;
		}
	}

	return ret;
}

static ssize_t sensor_general_time_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	sensor_general_time = val;

	return count;
}

static DEVICE_ATTR(time, S_IRUGO|S_IWUSR,
		sensor_general_time_show,
		sensor_general_time_store);

static struct attribute *sensor_general_attributes[] = {
	&dev_attr_start.attr,
	&dev_attr_dbgsensors.attr,
	&dev_attr_dbglevel.attr,
	&dev_attr_time.attr,
	NULL
};
#else
static struct attribute *sensor_general_attributes[] = {
	&dev_attr_start.attr,
	NULL
};
#endif

static struct attribute_group sensor_general_attribute_group = {
	.attrs = sensor_general_attributes
};

static void sensor_data_op_init(void)
{
	data_op_array[OP_LOGIC_EQ] = data_op_logic_eq;
	data_op_array[OP_LOGIC_NEQ] = data_op_logic_neq;
	data_op_array[OP_LOGIC_GREATER] = data_op_logic_greater;
	data_op_array[OP_LOGIC_LESS] = data_op_logic_less;
	data_op_array[OP_LOGIC_GE] = data_op_logic_ge;
	data_op_array[OP_LOGIC_LE] = data_op_logic_le;
	data_op_array[OP_LOGIC_AND] = data_op_logic_and;
	data_op_array[OP_LOGIC_OR] = data_op_logic_or;

	data_op_array[OP_ARI_ADD] = data_op_ari_add;
	data_op_array[OP_ARI_SUB] = data_op_ari_sub;
	data_op_array[OP_ARI_MUL] = data_op_ari_mul;
	data_op_array[OP_ARI_DIV] = data_op_ari_div;
	data_op_array[OP_ARI_MOD] = data_op_ari_mod;

	data_op_array[OP_BIT_OR] = data_op_bit_or;
	data_op_array[OP_BIT_AND] = data_op_bit_and;
	data_op_array[OP_BIT_LSL] = data_op_bit_lsl;
	data_op_array[OP_BIT_LSR] = data_op_bit_lsr;
	data_op_array[OP_BIT_NOR] = data_op_bit_nor;

	data_op_array[OP_ENDIAN_BE16] = data_op_be16;
	data_op_array[OP_ENDIAN_BE16_UN] = data_op_be16u;
	data_op_array[OP_ENDIAN_BE24] = data_op_be24;
	data_op_array[OP_ENDIAN_BE32] = data_op_be32;

	data_op_array[OP_ENDIAN_LE16] = data_op_le16;
	data_op_array[OP_ENDIAN_LE16_UN] = data_op_le16u;
	data_op_array[OP_ENDIAN_LE24] = data_op_le24;
	data_op_array[OP_ENDIAN_LE32] = data_op_le32;

	data_op_array[OP_MIN] = data_op_min;
	data_op_array[OP_MAX] = data_op_max;
}

static int __init sensor_general_init(void)
{
	int ret;

	device_initialize(&general_sensor_device);
	general_sensor_device.parent = NULL;
	dev_set_name(&general_sensor_device, "generalsensor");
	ret = device_add(&general_sensor_device);
	if (ret) {
		printk(KERN_ERR "Fail to register device\n");
		return ret;
	}

#ifdef CONFIG_GENERAL_SENSOR_DEBUG
	sensor_general_time = 0;
#endif

	sensor_image = NULL;
	sensor_general_attached	= 0;
	registered_drivers = 0;
	registered_devices = 0;
	failed_drivers = 0;
	failed_devices = 0;

	/*init callback of data operations*/
	sensor_data_op_init();

	/*attributes*/
	ret = sysfs_create_group(&general_sensor_device.kobj,
			&sensor_general_attribute_group);
	if (ret) {
		dev_err(&general_sensor_device, "sysfs can not create group\n");
		goto err;
	}

	/*trigger at kernel boot time*/
#ifdef CONFIG_GENERAL_SENSOR_BOOT
	ret = sensor_general_start(SG_START);
	if (ret) {
		dev_err(&general_sensor_device, "error sensor general init\n");
		sysfs_remove_group(&general_sensor_device.kobj,
			&sensor_general_attribute_group);
		goto err;
	}
#endif

	return 0;

err:
	device_del(&general_sensor_device);

	return ret;
}

static void __exit sensor_general_exit(void)
{
	unregister_sensor_drivers();
	unregister_sensor_devices();
	kfree(sensor_image);
	sensor_image = NULL;
	sysfs_remove_group(&general_sensor_device.kobj,
			&sensor_general_attribute_group);
	device_del(&general_sensor_device);
}

module_init(sensor_general_init);
module_exit(sensor_general_exit);

MODULE_DESCRIPTION("General Sensor Driver");
MODULE_AUTHOR("PSI IO&Sensor Team");
MODULE_LICENSE("GPL");
