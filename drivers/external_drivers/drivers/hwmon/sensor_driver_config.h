#ifndef SENSOR_DRIVER_CONFIG_H
#define SENSOR_DRIVER_CONFIG_H
/*
* Sensor Driver Configure Interface between HAL and Driver
* If any update in Driver, then update in HAL too and vice versa
*/

/*
* Actions specified by developer
* extend here for new sensor action,new sysfs api or others
*/
enum sensor_action {
	INIT = 0, DEINIT,
	ENABLE, DISABLE,
	INT_ACK,
	GET_DATA_X, GET_DATA_Y, GET_DATA_Z,
	GET_RANGE, SET_RANGE,
	GET_SELFTEST, SET_SELFTEST,
	SENSOR_ACTION_RESERVE,
};

/* operand of data_action
* immediate, i2c register
* global i2c register buf, index of private data
* output of before operation
*/
enum operand_type {
	OPT_IMM = 0, OPT_REG,
	OPT_REG_BUF, OPT_INDEX,
	OPT_BEFORE,
	OPT_RESERVE,
};

struct operand {
	__u8 type;
	union {
		__s32 immediate;
		__s32 index;
		struct operand_register {
			__u8 addr;
			__u8 len;
			__u8 flag;
			__u8 pad;
		} reg;
	} data;
}__attribute__ ((packed));

/* Data Operation
*i2c register and variable& access: =,
*logic: ==,!=,<,>,<=,>=,
*arithmetic: +,-,*,/,%,
*bit: |,&,<<,>>,~
*endian change:	be16_to_cpu, be24_to_cpu, be32_to_cpu, le16_to_cpu,
* le24_to_cpu, le32_to_cpu be24_to_cpu is for 3Bytes of lps331ap X1 data
*min, max:
*comma experession
*/
enum data_op {
	OP_ACCESS = 0,
	OP_MIN, OP_MAX,
	OP_LOGIC_EQ, OP_LOGIC_NEQ, OP_LOGIC_GREATER, OP_LOGIC_LESS,
	OP_LOGIC_GE, OP_LOGIC_LE, OP_LOGIC_AND, OP_LOGIC_OR,
	OP_ARI_ADD, OP_ARI_SUB, OP_ARI_MUL, OP_ARI_DIV, OP_ARI_MOD,
	OP_BIT_OR, OP_BIT_AND, OP_BIT_LSL, OP_BIT_LSR, OP_BIT_NOR,
	OP_ENDIAN_BE16, OP_ENDIAN_BE16_UN, OP_ENDIAN_BE24, OP_ENDIAN_BE32,
	OP_ENDIAN_LE16, OP_ENDIAN_LE16_UN, OP_ENDIAN_LE24, OP_ENDIAN_LE32,
	OP_RESERVE,
};

/*
* data_action of lowlevel_action,  mostly for i2c registers' data
* = operation can descript i2c register read/write
* and global private data setting
* read or write i2c registers:
*	read: the data will be read into global register buf
*	write: the source data is from operand2 and will be
*		firstly written into the global register buf
*
* for other operations, all result will be treat as
*   input of next process or driver API.
*
* operand type:
* immediate; register addr, len; index of private data; data of before operation
* size: 11B
*/
struct data_action {
	 __u8 op;
	 struct operand operand1;
	 struct operand operand2;
}__attribute__ ((packed));

/*
* sleep_action of lowlevel_action
*/
struct sleep_action {
	__s32 ms;
};

/*
* ifelse_action of lowlevel_action
* action nums for condition, if and else
*/
struct ifelse_action {
	__s16 num_con;
	__s16 num_if;
	__s16 num_else;
}__attribute__ ((packed));

/*
* extern_c_action of lowlevel_action
*/
struct externc_action {
	__u32 index;
};

/* General Lowlevel Action
*   used to descript sensor_action by developer
*   extend new type of lowlevel action here
*/
enum action_lowlevel {
	DATA = 0, SLEEP, IFELSE,
	RETURN, SWITCH, EXTERNC, ACTION_RESERVE
};

struct lowlevel_action {
	__u8 type;
	union {
		struct data_action data;
		struct sleep_action sleep;
		struct ifelse_action ifelse;
		struct externc_action externc;
	} action;
}__attribute__ ((packed));

/*
* lowlevel action index info in sensor_config
*/
#define MAX_LL_ACTION_NUM	0xff
struct lowlevel_action_index {
	__u16 index;
	__u16 num;
}__attribute__ ((packed));

/*
* odr table setting provided by developer for each sensor
* @hz:all supported hz of android and sensor, 0 means not support
* @index: index of lowlevel action table
*/
struct odr {
	__s32 hz;
	struct lowlevel_action_index index;
}__attribute__ ((packed));

/*
* range table setting provided by developer for each sensor
* @index: index of lowlevel action table
*/
struct range_setting {
	__s32 range;
	struct lowlevel_action_index index;
}__attribute__ ((packed));

/*
* sysfs file info
* type: sensor_action or data_action
* for extension, don't mix data action and
* sensor actions specified by developer
* @mode: file access mode
*/
#define  MAX_ATTR_NAME_BYTES	15
enum show_store_action {
	DATA_ACTION = 0, SENSOR_ACTION,
};

struct sysfs_entry {
	__u8 name[MAX_ATTR_NAME_BYTES];
	__u8 type;
	__u16 mode;
	/*action detail*/
	union {
		struct {
			__u8 show;
			__u8 store;
		} sensor;

		struct {
			struct lowlevel_action_index index_show;
			struct lowlevel_action_index index_store;
		} data;
	} action;
}__attribute__ ((packed));

/* The whole config format
*  XML parser will generate this formated config image from XML file
*
* @size: config size
* @i2c_bus:bus number of attached adapter
* @test_reg_addr:used to check whether device is valid
*		when there are multi i2c addresss
* @2c_addrs:all supported i2c client address
* @id[MAX_DEV_IDS]: series of device are supported
*		by this driver, 0 means invalid id.
* @name[MAX_DEV_IDS][MAX_DEV_NAME_BYTES]: used to
*		match this driver to relative device
* @input_name[MAX_DEV_NAME_BYTES]: input device name
* @attr_name[MAX_DEV_NAME_BYTES]: name of subdir for
*		attribute files(enable,poll,...)
* @id_reg_addr:addr of ID register
* @id_reg_flag:access flag of ID register
* @sensor_regs: number of registers
* @event_type: input event type
* @method:polling, interrupt, polling+interrupt
* @default_poll_interval: default value, in ms, for MIX method
*             in accelerometer driver, should set poll as 0
* @min_poll_interval: min interval
* @max_poll_interval: max interval
*
*   Additional info for interrupt and interrupt+polling
* @gpio_num:	assume interrupt source are all gpio
* @report_cnt: nonzero, only valid for polling+interrupt mode
* @report_interval: report interval for MIX method
* @irq_flag:flag of request_irq_thread
*
* @shared_nums: how many function of this i2c device
* @irq_serialize:set 1 if need to serialize irq handling
*
* @odr_entries:valid entries in odr_table
* @odr_table[MAX_ODR_SETTING_ENTRIES]:ODR, BW, Resolution setting table
*   range setting table
* @default_range:default, must be set , or will make range_show complicate,
*         zero means not support, so don't init and support relative api
* @range_entries:valid entries in range_table
* @range_setting range_table[MAX_RANGES]:
*
* @indexs: index infos of all specified sensor actions
* @actions: pack all lowlevel actions together to reduce config image size
*/
#define INVALID_I2C_BUS			0xff
#define INVALID_I2C_ADDR		0xff
#define MAX_I2C_ADDRS			4
#define MAX_DEV_IDS			4
#define MAX_DEV_NAME_BYTES		32
#define SENSOR_INVALID_REG		0xff
#define SENSOR_INVALID_INTERVAL		0xffffffff
#define MAX_ODR_SETTING_ENTRIES		8
#define MAX_RANGES			6
#define MAX_SYSFS_ENTRIES		6
enum method_get_data {INT = 0, POLL, MIX,};
struct sensor_config {
	__u16 size;

	/*Basic info of sensor driver*/
	__u8 i2c_bus;
	__u8 test_reg_addr;
	__u8 i2c_addrs[MAX_I2C_ADDRS];
	__u8 id[MAX_DEV_IDS];
	__u8 name[MAX_DEV_NAME_BYTES];
	__u8 input_name[MAX_DEV_NAME_BYTES];
	__u8 attr_name[MAX_DEV_NAME_BYTES];
	__u8 id_reg_addr;
	__u8 id_reg_flag;
	__u8 sensor_regs;
	__u8 event_type;

	/*infos to get data method */
	__u32 method;
	__s32 default_poll_interval;
	__s32 min_poll_interval;
	__s32 max_poll_interval;
	__s32 gpio_num;
	__s32 report_cnt;
	__s32 report_interval;
	__u32 irq_flag;

	/*multi function device*/
	__s32 shared_nums;
	__s32 irq_serialize;

	__s32 odr_entries;
	struct odr odr_table[MAX_ODR_SETTING_ENTRIES];

	__s32 range_entries;
	struct range_setting range_table[MAX_RANGES];

	__s32 sysfs_entries;
	__s32 default_range;
	struct sysfs_entry sysfs_table[MAX_SYSFS_ENTRIES];

	struct lowlevel_action_index indexs[SENSOR_ACTION_RESERVE];
	/*struct lowlevel_action *actions;*/
	__s32 actions;
}__attribute__ ((packed));

/*sensor config image
* @num:how many sensor config in this image
* @configs:sensor config array of all supported sensors
*/
struct sensor_config_image {
	__u32 magic;
	__u32 flags;
	__u32 dbg_sensors;
	__u32 dbg_level;
	__s32 num;
	__u32 configs;
};

#define DATA_STACK_MAX_SIZE	0x20
#define PRIVATE_MAX_SIZE	0x20

/*general flags*/
#define SG_FLAGS_BOOT_DISABLE	0x1
/*the way to start parse*/
#define SG_START		0x1
#define SG_FORCE_START		0x2

/*maxium number of external c functions*/
#define MAX_EXTERN_C            100

#endif
