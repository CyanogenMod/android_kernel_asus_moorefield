
#ifndef _SENSOR_GENERAL_H
#define _SENSOR_GENERAL_H

#include <linux/hrtimer.h>
#include <linux/kthread.h>
#define GENERAL_SENSOR_FIRMWARE "sensor_config.bin"
#define MAX_SENSOR_DRIVERS	32

#define le24_to_cpu(a)		(a)
#define be24_to_cpu(a)		((((a)&0xff)<<16) |	\
				((a)&0xff00) | (((a)&0xff0000>>16)))

/*stack of data operation
* store the input/intermeidate/output data between actions
* all data actions except access will store result in stack
*/
struct sensor_data_stack {
	int top;              /*pointer of top valid data*/
	int data_stack[DATA_STACK_MAX_SIZE];
};

struct sensor_data;
/*
* implement complicated algo on raw data for performance
* name: i2c driver name
* proc_x/y/z: return corrected data relative to raw data
*/
struct sensor_rawdata_proc{
	char *name;
	int (*proc_x)(struct sensor_data *, int);
	int (*proc_y)(struct sensor_data *, int);
	int (*proc_z)(struct sensor_data *, int);
};

/*Private data for each sensor*/
struct sensor_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct device *attr_dev;
	unsigned int dbg_on;

	struct mutex *lock;
	struct mutex real_lock;

	struct hrtimer work_timer;
	struct completion report_complete;
	struct task_struct *thread;
	bool hrtimer_running;
	/*work queue status*/
	int launched;

	/*record sysfs device files for each sensor for remove*/
	struct device_attribute *dev_attr;
	struct sensor_config *config;

	enum sensor_state {
		STATE_DIS,  STATE_EN, STATE_SUS,
	} state;
	enum sensor_state state_suspend; /*state before suspend*/
	int poll_interval;
	int range;
	/*remain report count for poll + interrupt mode*/
	int report_cnt;
	/*gpio number of interrupt input*/
	int gpio;
	struct sensor_rawdata_proc *rawdata_proc;

	/*multi function device*/
	u8 multi_index;		/*index of multi device: [0, nums)*/
	u8 *share_irq_seq;	/*irq sequence of multi device*/

	/*i2c registers buf indexed by i2c reg addr: [0, sensor_regs)*/
	u8 *regbuf;

	struct sensor_data_stack stack;

	/*for extension, such as: sensitivity of accelerometer */
	int private[PRIVATE_MAX_SIZE];
};

typedef int (*p_extern_c)(struct sensor_data *);

int sensor_register_rawdata_proc(struct sensor_rawdata_proc *);
int sensor_unregister_rawdata_proc(struct sensor_rawdata_proc *);
int sensor_register_extern_c(p_extern_c p);
int sensor_unregister_extern_c(p_extern_c p);

#ifdef CONFIG_GENERAL_SENSOR_DEBUG

extern unsigned int sensor_general_debug_sensors;
extern unsigned int sensor_general_debug_level;
extern char *action_debug[];

#define DBG_ALL_SENSORS		0xFFFFFFFF
#define DBG_LEVEL1		1
#define DBG_LEVEL2		2
#define DBG_LEVEL3		3
#define DBG_LEVEL4		4
#define DBG_LEVEL5		5
#define DBG_LEVEL6		6

#define SENSOR_DBG(level, sensors, fmt, ...)			\
do {								\
	if ((level <= sensor_general_debug_level) &&		\
		(sensors & sensor_general_debug_sensors))	\
		printk(KERN_DEBUG "[%d]%s "			\
			fmt "\n",				\
			__LINE__, __func__,			\
			##__VA_ARGS__);				\
} while (0)

static inline void dbg_dump(char *buf, int len) __attribute__((unused));
static inline void dbg_dump(char *buf, int len)
{
	int i;

	printk(KERN_DEBUG "%p 0x%x\n", buf, len);
	for (i = 0; i < len; i++) {
		printk(KERN_DEBUG "%02x ", (unsigned char)buf[i]);
		if ((i + 1) % 32 == 0)
			printk(KERN_DEBUG "\n");
	}
	printk(KERN_DEBUG "\n");
}

#else

#define SENSOR_DBG(level, sensors, fmt, ...)
#define dbg_dump(a, b)

#endif

#endif
