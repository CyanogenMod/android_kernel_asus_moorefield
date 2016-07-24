/* Himax Android Driver Sample Code Ver 2.5
*
* Copyright (C) 2012 Himax Corporation.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*/

/*==========================================================
 Segment list :
 Include Header file
 Himax Define Options
 Himax Define Variable
 Himax Include Header file / Data Structure
 Himax Variable/Pre Declation Function
 Himax Normal Function
 ASUS debug attr function
 Himax SYS Debug Function
 Himax Touch Work Function
 Himax Linux Driver Probe Function
 Other Function
===========================================================*/

/*=========================================================

	Segment : Include Header file

===========================================================*/
#include <linux/module.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>
#include <linux/debugfs.h>
#include <linux/irq.h>
#include <linux/syscalls.h>
#include <linux/time.h>

/* for linux 2.6.36.3 */
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <asm/ioctl.h>
#include <linux/switch.h>
#include <linux/proc_fs.h>
#include <linux/wakelock.h>
#include <linux/regulator/consumer.h>

#include <linux/fs.h>
#include <asm/segment.h>
#include <asm/uaccess.h>
#include <linux/buffer_head.h>
#include <linux/kthread.h>

#include <linux/HWVersion.h>

#include <linux/hx8528_me372cl.h>

#define DRIVER_VERSION	"0.2.0"	/* Josh modify */

/*=========================================================

	Segment : Himax Define Options

===========================================================
TODO START : Select the function you need!
------------------------------------------
 Support Function Enable : */
#define HX_TP_SYS_DIAG		/* Support Sys : Diag function          ,default is open  */
#define HX_TP_SYS_REGISTER	/* Support Sys : Register function      ,default is open  */
#define HX_TP_SYS_DEBUG_LEVEL	/* Support Sys : Debug Level function   ,default is open  */
#define HX_TP_SYS_FLASH_DUMP	/* Support Sys : Flash dump function    ,default is open  */
#define HX_TP_SYS_SELF_TEST	/* Support Sys : Self Test Function     ,default is open  */
#define HX_TP_SYS_HITOUCH	/* Support Sys : Hi-touch command       ,default is open  */
/*#define HX_EN_SEL_BUTTON	// Support Self Virtual key		,default is close */
/*#define HX_EN_MUT_BUTTON	// Support Mutual Virtual Key		,default is close */
/*#define HX_EN_GESTURE		// Support Gesture , need porting	,default is close */
#define HX_RST_PIN_FUNC		/* Support HW Reset                     ,default is open  */
/*#define HX_LOADIN_CONFIG	// Support Common FW,load in config */
#define HX_PORTING_DEB_MSG	/* Support Driver Porting Message       ,default is close */
/*#define HX_IREF_MODIFY	// Support IREF Modify Function		,default is close */

/* TODO END */

/* ----- Support Different IC. Select one at one time.----- */
#define HX_85XX_A_SERIES_PWON		1
#define HX_85XX_B_SERIES_PWON		2
#define HX_85XX_C_SERIES_PWON		3
#define HX_85XX_D_SERIES_PWON		4

/* ----- Supoort ESD Issue ----- */

#ifdef HX_RST_PIN_FUNC
/*#define HX_RST_BY_POWER		// Support Reset by power pin		,default is close */
/*#define HX_ESD_WORKAROUND		// Support ESD Workaround		,default is close */
#define ENABLE_CHIP_RESET_MACHINE	/* Support Chip Reset Workqueue         ,default is open  */
#endif

#ifdef ENABLE_CHIP_RESET_MACHINE
#define HX_TP_SYS_RESET		/* Support Sys : HW Reset function      ,default is open  */
/*#define ENABLE_CHIP_STATUS_MONITOR	// Support Polling ic status		,default is close */
#endif

/* ----- Support FW Bin checksum method,mapping with Hitouch *.bin ----- */
#define HX_TP_BIN_CHECKSUM_SW	1
#define HX_TP_BIN_CHECKSUM_HW	2
#define HX_TP_BIN_CHECKSUM_CRC	3
/* #define ENABLE_SELF_FIRMWARE_UPGRADE */

/*=========================================================

	Segment : Himax Define Variable

  =========================================================
TODO START : Modify follows deinfe variable */
/* #define HX_ENABLE_EDGE_TRIGGER	// define:Level triggle , un-defined:Level triggle */
#define HX_KEY_MAX_COUNT	3	/* Max virtual keys */
#define DEFAULT_RETRY_CNT	3	/* For I2C Retry count */
/* TODO END */

/* TODO START : Modify follows power gpio / interrupt gpio / reset gpio
//----- power supply , i2c , interrupt gpio ----- */
#define HIMAX_PWR_GPIO		59
#define HIMAX_INT_GPIO		62
#define HIMAX_RST_GPIO		88
/* TODO END */

/* TODO START : Modify the I2C address
//----- I2C ----- */
#define HIMAX_I2C_ADDR	0x48
#define HIMAX_TS_NAME	"hx8528"
/* TODO END */

/* ----- Input Device */
#define INPUT_DEV_NAME	"himax-touchscreen"

/* ----- Flash dump file */
#define FLASH_DUMP_FILE "/data/log/ME372CL_Touch_Flash_Dump.bin"
#define TOUCH_DC_DUMP_FILE "/data/log/touch/ME372CL_touch_dc.txt"
#define TOUCH_IIR_DUMP_FILE "/data/log/touch/ME372CL_touch_diff.txt"
#define TOUCH_BANK_DUMP_FILE "/data/log/touch/ME372CL_touch_bank.txt"

/* ----- Diag Coordinate dump file */
#define DIAG_COORDINATE_FILE "/sdcard/Coordinate_Dump.csv"

/* ----- Virtual key */
#define HX_VKEY_0   KEY_BACK
#define HX_VKEY_1   KEY_HOME
#define HX_VKEY_2   KEY_MENU
/* #define HX_VKEY_3   104 */
#define HX_KEY_ARRAY    {HX_VKEY_0, HX_VKEY_1, HX_VKEY_2}

/* ----- Himax TP COMMANDS -> Do not modify the below definition */
#define HX_CMD_NOP                   0x00	/* no operation */
#define HX_CMD_SETMICROOFF           0x35	/* set micro on */
#define HX_CMD_SETROMRDY             0x36	/* set flash ready */
#define HX_CMD_TSSLPIN               0x80	/* set sleep in */
#define HX_CMD_TSSLPOUT              0x81	/* set sleep out */
#define HX_CMD_TSSOFF                0x82	/* sense off */
#define HX_CMD_TSSON                 0x83	/* sense on */
#define HX_CMD_ROE                   0x85	/* read one event */
#define HX_CMD_RAE                   0x86	/* read all events */
#define HX_CMD_RLE                   0x87	/* read latest event */
#define HX_CMD_CLRES                 0x88	/* clear event stack */
#define HX_CMD_TSSWRESET             0x9E	/* TS software reset */
#define HX_CMD_SETDEEPSTB            0xD7	/* set deep sleep mode */
#define HX_CMD_SET_CACHE_FUN         0xDD	/* set cache function */
#define HX_CMD_SETIDLE               0xF2	/* set idle mode */
#define HX_CMD_SETIDLEDELAY          0xF3	/* set idle delay */
#define HX_CMD_SELFTEST_BUFFER       0x8D	/* Self-test return buffer */
#define HX_CMD_MANUALMODE            0x42
#define HX_CMD_FLASH_ENABLE          0x43
#define HX_CMD_FLASH_SET_ADDRESS     0x44
#define HX_CMD_FLASH_WRITE_REGISTER  0x45
#define HX_CMD_FLASH_SET_COMMAND     0x47
#define HX_CMD_FLASH_WRITE_BUFFER    0x48
#define HX_CMD_FLASH_PAGE_ERASE      0x4D
#define HX_CMD_FLASH_SECTOR_ERASE    0x4E
#define HX_CMD_CB                    0xCB
#define HX_CMD_EA                    0xEA
#define HX_CMD_4A                    0x4A
#define HX_CMD_4F                    0x4F
#define HX_CMD_B9                    0xB9
#define HX_CMD_76                    0x76
#define HX_CMD_E3                    0xE3

/*=========================================================

	Segment : Himax Include Header file / Data Structure

========================================================= */

struct himax_i2c_setup_data {
	unsigned i2c_bus;
	unsigned short i2c_address;
	int irq;
	char name[I2C_NAME_SIZE];
};

struct himax_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct workqueue_struct *himax_wq;
	struct work_struct work;
	int (*power) (int on);
	struct early_suspend early_suspend;
	int intr_gpio;
	/* Firmware Information */
	int fw_ver;
	int fw_id;
	int x_resolution;
	int y_resolution;
	/* For Firmare Update  */
	struct miscdevice firmware;
	struct attribute_group attrs;
	struct switch_dev touch_sdev;
	int abs_x_max;
	int abs_y_max;
	int rst_gpio;
	int init_success;
	struct regulator *vdd;
	int irq;

	/* Wakelock Protect start */
	struct wake_lock wake_lock;
	/* Wakelock Protect end */

	/* Mutexlock Protect Start */
	struct mutex mutex_lock;
	/* Mutexlock Protect End */

	/*----[HX_TP_SYS_FLASH_DUMP]---------start */
#ifdef HX_TP_SYS_FLASH_DUMP
	struct workqueue_struct *flash_wq;
	struct work_struct flash_work;
#endif
	/*----[HX_TP_SYS_FLASH_DUMP]---------end */

	/*----[ENABLE_CHIP_RESET_MACHINE]----start */
	/*      #ifdef ENABLE_CHIP_RESET_MACHINE
	   //   int retry_time; */
	struct delayed_work himax_chip_reset_work;
	/*      #endif */
	/*----[ENABLE_CHIP_RESET_MACHINE]----end */

	/*----[ENABLE_CHIP_STATUS_MONITOR]---start */
#ifdef ENABLE_CHIP_STATUS_MONITOR
	struct delayed_work himax_chip_monitor;
	int running_status;
#endif
	/*----[ENABLE_CHIP_STATUS_MONITOR]---end */

	/* firmware upgrade by Josh +++ */
#ifdef ENABLE_SELF_FIRMWARE_UPGRADE
	struct delayed_work himax_chip_firmware_upgrade_work;
#endif
	/* firmware upgrade by Josh --- */

	/*----[ASUS_BSP Josh_Sh]------------start */
	int tp_status;
	char himax_firmware_version[8];
	int firmware_version_checksum;	/*add by josh for version checksum */
	char tp_lens_version[8];
	int config_version_checksum;
	char config_firmware_version[8];

	int AP_update;
	int AP_progress;
	int tp_firmware_upgrade_proceed;
	int need_upgrade_fw;
	/*----[ASUS_BSP Josh_Sh]------------end	*/

};

static struct himax_ts_data *himax_data;

/*=========================================================
//
//	Segment : Himax Variable/Pre Declation Function
//
//======================================================== */
static uint8_t IC_STATUS_CHECK = 0xAA;	/* for Hand shaking to check IC status */
static int tpd_keys_local[HX_KEY_MAX_COUNT] = HX_KEY_ARRAY;	/* for Virtual key array */

static unsigned char IC_CHECKSUM;
static unsigned char IC_TYPE;

static int HX_TOUCH_INFO_POINT_CNT;

static int HX_RX_NUM;
static int HX_TX_NUM;
static int HX_BT_NUM;
static int HX_X_RES;
static int HX_Y_RES;
static int HX_MAX_PT;
static bool HX_XY_REVERSE;
static bool HX_INT_IS_EDGE;
static unsigned int FW_VER_MAJ_FLASH_ADDR;
static unsigned int FW_VER_MAJ_FLASH_LENG;
static unsigned int FW_VER_MIN_FLASH_ADDR;
static unsigned int FW_VER_MIN_FLASH_LENG;
static unsigned int CFG_VER_MAJ_FLASH_ADDR;
static unsigned int CFG_VER_MAJ_FLASH_LENG;
static unsigned int CFG_VER_MIN_FLASH_ADDR;
static unsigned int CFG_VER_MIN_FLASH_LENG;

static u16 FW_VER_MAJ_buff[1];	/* for Firmware Version */
static u16 FW_VER_MIN_buff[1];
static u16 CFG_VER_MAJ_buff[12];
static u16 CFG_VER_MIN_buff[12];

static bool debug_log;
static bool is_suspend;

static int hx_point_num;	/* for himax_ts_work_func use */
static int p_point_num = 0xFFFF;
static int tpd_key;
static int tpd_key_old = 0xFF;

static int cable_status = -1;	/* josh add for USB time issue (init : -1) */

static struct kobject *android_touch_kobj;	/* Sys kobject variable */

static int i2c_himax_read(struct i2c_client *client, uint8_t command,
			  uint8_t *data, uint8_t length, uint8_t toRetry);
static int i2c_himax_write(struct i2c_client *client, uint8_t command,
			   uint8_t *data, uint8_t length, uint8_t toRetry);
static int i2c_himax_master_write(struct i2c_client *client, uint8_t *data,
				  uint8_t length, uint8_t toRetry);
static int i2c_himax_write_command(struct i2c_client *client, uint8_t command,
				   uint8_t toRetry);

static int himax_lock_flash(void);
static int himax_unlock_flash(void);

static int himax_hang_shaking(void);	/* Hand shaking function */
static int himax_ts_poweron(void);	/* Power on */
static int himax_touch_sysfs_init(void);	/* Sys filesystem initial */
static void himax_touch_sysfs_deinit(void);	/* Sys filesystem de-initial */
static u8 himax_read_fw_ver(bool hw_reset);
static int fts_ctpm_fw_upgrade_with_sys_fs(unsigned char *fw, int len);

static uint8_t rFE96_setting[8] = { 0x02, 0x58, 0x14, 0x14, 0x37, 0x08, 0x37, 0x08 };	/* add by josh for init self_test FE96 */

static int self_test_delay_time = 3;	/* add by josh for init self_test delay time(s) */

/*----[HX_LOADIN_CONFIG]----------start */
#ifdef HX_LOADIN_CONFIG
unsigned char c1[] = { 0x37, 0xFF, 0x08, 0xFF, 0x08 };
unsigned char c2[] = { 0x3F, 0x00 };
unsigned char c3[] = { 0x62, 0x01, 0x00, 0x01, 0x43, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned char c4[] = { 0x63, 0x10, 0x00, 0x10, 0x30, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned char c5[] = { 0x64, 0x01, 0x00, 0x01, 0x43, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned char c6[] = { 0x65, 0x10, 0x00, 0x10, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned char c7[] = { 0x66, 0x01, 0x00, 0x01, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned char c8[] = { 0x67, 0x10, 0x00, 0x10, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned char c9[] = { 0x68, 0x01, 0x00, 0x01, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned char c10[] = { 0x69, 0x10, 0x00, 0x10, 0x30, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned char c11[] = { 0x6A, 0x01, 0x00, 0x01, 0x02, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned char c12[] = { 0x6B, 0x10, 0x00, 0x10, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned char c13[] = { 0x6C, 0x01, 0x00, 0x01, 0x30, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned char c14[] = { 0x6D, 0x10, 0x00, 0x10, 0x03, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned char c15[] = { 0x6E, 0x01, 0x00, 0x01, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned char c16[] = { 0x6F, 0x10, 0x00, 0x10, 0x20, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned char c17[] = { 0x70, 0x01, 0x00, 0x01, 0x03, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned char c18[] = { 0x7B, 0x03 };
unsigned char c19[] = { 0x7C, 0x00, 0xD8, 0x8C };
unsigned char c20[] = { 0x7F, 0x00, 0x04, 0x0A, 0x0A, 0x04, 0x00, 0x00, 0x00 };
unsigned char c21[] = { 0xA4, 0x94, 0x62, 0x94, 0x86 };
unsigned char c22[] = { 0xB4, 0x04, 0x01, 0x01, 0x01, 0x01, 0x03, 0x0F, 0x04, 0x07, 0x04, 0x07,
0x04, 0x07, 0x00 };
unsigned char c23[] = { 0xB9, 0x01, 0x36 };
unsigned char c24[] = { 0xBA, 0x00 };
unsigned char c25[] = { 0xBB, 0x00 };
unsigned char c26[] = { 0xBC, 0x00, 0x00, 0x00, 0x00 };
unsigned char c27[] = { 0xBD, 0x04, 0x0C };
unsigned char c28[] = { 0xC2, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned char c29[] = { 0xC5, 0x0A, 0x1D, 0x00, 0x10, 0x1A, 0x1E, 0x0B, 0x1D, 0x08, 0x16 };
unsigned char c30[] = { 0xC6, 0x1A, 0x10, 0x1F };

unsigned char c31[] = {
0xC9, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0x15, 0x17, 0x17, 0x19, 0x19,
0x1F, 0x1F, 0x1B, 0x1B, 0x1D, 0x1D, 0x21, 0x21, 0x23, 0x23,
	0x25, 0x25, 0x27, 0x27, 0x29, 0x29, 0x2B, 0x2B, 0x2D, 0x2D, 0x2F, 0x2F,
	    0x16, 0x16, 0x18, 0x18, 0x1A, 0x1A, 0x20, 0x20, 0x1C, 0x1C,
	0x1E, 0x1E, 0x22, 0x22, 0x24, 0x24, 0x26, 0x26, 0x28, 0x28, 0x2A, 0x2A,
	    0x2C, 0x2C, 0x2E, 0x2E, 0x30, 0x30, 0x00, 0x00, 0x00
};
unsigned char c32[] = {
0xCB, 0x01, 0xF5, 0xFF, 0xFF, 0x01, 0x00, 0x05, 0x00, 0x9F, 0x00, 0x00,
0x00 };
unsigned char c33[] = { 0xD0, 0x06, 0x01 };
unsigned char c34[] = { 0xD3, 0x06, 0x01 };
unsigned char c35[] = {
0xD5, 0xA5, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00 };

unsigned char c36[] = {
0x40, 0x01, 0x5A, 0x77, 0x02, 0xF0, 0x13, 0x00, 0x00, 0x56, 0x10, 0x14,
0x18, 0x06, 0x10, 0x0C, 0x0F, 0x0F, 0x0F, 0x52, 0x34, 0x80,
	0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};				/* start:0x00 ,size 31 */

unsigned char c37[] = {
0x40, 0xA5, 0x00, 0x80, 0x82, 0x85, 0x00, 0x35, 0x25, 0x0F, 0x0F, 0x83,
0x3C, 0x00, 0x00,
	0x11, 0x11, 0x00, 0x00, 0x01, 0x01, 0x00, 0x0A, 0x00, 0x00, 0x10, 0x02,
	    0x10, 0x64, 0x00, 0x00
};				/* start 0x1E :size 31 */

unsigned char c38[] = { 0x40, 0x40, 0x38, 0x38, 0x02, 0x14, 0x00, 0x00, 0x00, 0x04, 0x03, 0x12, 0x06, 0x06, 0x00, 0x00, 0x00 };	/* start:0x3C ,size 17 */

unsigned char c39[] = {
0x40, 0x18, 0x18, 0x05, 0x00, 0x00, 0xD8, 0x8C, 0x00, 0x00, 0x42, 0x03,
0x00, 0x00, 0x00, 0x00, 0x00,
	0x10, 0x02, 0x80, 0x00, 0x00, 0x00, 0x00, 0x0C
};				/* start 0x4C,size 25 */

unsigned char c40[] = {
0x40, 0x10, 0x12, 0x20, 0x32, 0x01, 0x04, 0x07, 0x09, 0xB4, 0x6E, 0x32,
0x00, 0x0F, 0x1C, 0xA0, 0x16,
	0x00, 0x00, 0x04, 0x38, 0x07, 0x80
};				/*start 0x64,size 23 */

unsigned char c41[] = {
0x40, 0x03, 0x2F, 0x08, 0x5B, 0x56, 0x2D, 0x05, 0x00, 0x69, 0x02, 0x15,
0x4B, 0x6C, 0x05,
	0x03, 0xCE, 0x09, 0xFD, 0x58, 0xCC, 0x00, 0x00, 0x7F, 0x02, 0x85, 0x4C,
	    0xC7, 0x00
};				/*start 0x7A,size 29 */

unsigned char c42[] = { 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };	/*start 0x96,size 9 */

unsigned char c43_1[] = { 0x40, 0x00, 0xFF, 0x15, 0x28, 0x01, 0xFF, 0x16, 0x29, 0x02, 0xFF, 0x1B, 0x2A, 0x03, 0xFF, 0x1C, 0xFF, 0x04, 0xFF, 0x1D, 0xFF, 0x05, 0x0F, 0x1E, 0xFF, 0x06, 0x10, 0x1F, 0xFF, 0x07, 0x11, 0x20 };	/* start 0x9E,size 32 */
unsigned char c43_2[] = { 0x40, 0xFF, 0x08, 0x12, 0x21, 0xFF, 0x09, 0x13, 0x22, 0xFF, 0x0A, 0x14, 0x23, 0xFF, 0x0B, 0x17, 0x24, 0xFF, 0x0C, 0x18, 0x25, 0xFF, 0x0D, 0x19, 0x26, 0xFF, 0x0E, 0x1A, 0x27, 0xFF };	/* start 0xBD,size 29 */

unsigned char c44_1[] = { 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };	/*start 0xDA,size 32 */
unsigned char c44_2[] = { 0x40, 0x00, 0x00, 0x00, 0x00, 0x00 };	/* 0xF9 size 6 */
unsigned char c45[] = { 0x40, 0x1D, 0x00 };	/* start 0xFE,size 3 */
#endif
/*----[HX_LOADIN_CONFIG]------end */

/*----[CONFIG_HAS_EARLYSUSPEND]----start */
#ifdef CONFIG_HAS_EARLYSUSPEND
static void himax_ts_early_suspend(struct early_suspend *h);
static void himax_ts_late_resume(struct early_suspend *h);
#endif
/*----[CONFIG_HAS_EARLYSUSPEND]-------end */

/*----[HX_IREF_MODIFY]----------start */
#ifdef HX_IREF_MODIFY
unsigned char SFR_3u_1[16][2] = {
{0x18, 0x06}, {0x18, 0x16}, {0x18, 0x26}, {0x18, 0x36}, {0x18, 0x46},
{0x18, 0x56}, {0x18, 0x66}, {0x18, 0x76}, {0x18, 0x86}, {0x18, 0x96},
{0x18, 0xA6}, {0x18, 0xB6}, {0x18, 0xC6}, {0x18, 0xD6}, {0x18, 0xE6},
{0x18, 0xF6}
};

unsigned char SFR_6u_1[16][2] = {
{0x98, 0x04}, {0x98, 0x14}, {0x98, 0x24}, {0x98, 0x34}, {0x98, 0x44},
{0x98, 0x54}, {0x98, 0x64}, {0x98, 0x74}, {0x98, 0x84}, {0x98, 0x94},
{0x98, 0xA4}, {0x98, 0xB4}, {0x98, 0xC4}, {0x98, 0xD4}, {0x98, 0xE4},
{0x98, 0xF4}
};
#endif
/*----[HX_IREF_MODIFY]---------end */

/*
//----[HX_EN_GESTURE]-----------------------------------------------------------------------------------start
#ifdef HX_EN_GESTURE
static int 				Dist_Cal_EX = 0xFFFF;
static int 				Dist_Cal_Now = 0xFFFF;
static int 				ZoomInCnt = 0;
static int 				ZoomOutCnt = 0;
#endif
//----[HX_EN_GESTURE]-------------------------------------------------------------------------------------end
 */

/*----[HX_ESD_WORKAROUND]-------start */
static u8 ESD_RESET_ACTIVATE = 1;
#ifdef HX_ESD_WORKAROUND
static u8 ESD_COUNTER;
static int ESD_COUNTER_SETTING = 3;

void ESD_HW_REST(void);
#endif
/*----[HX_ESD_WORKAROUND]------end */

/*----[HX_TP_SYS_SELF_TEST]-----start */
#ifdef HX_TP_SYS_SELF_TEST
static ssize_t himax_chip_self_test_function(struct device *dev,
					     struct device_attribute *attr,
					     char *buf);
static ssize_t himax_self_test_setting(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count);
static int himax_chip_self_test(uint8_t *data);
#endif
/*----[HX_TP_SYS_SELF_TEST]-----end */

/*----[HX_TP_SYS_DEBUG_LEVEL]--------start */
#ifdef HX_TP_SYS_DEBUG_LEVEL
static uint8_t debug_log_level;
static bool fw_update_complete;
static bool irq_enable;
static int handshaking_result;
static unsigned char debug_level_cmd;
static unsigned char upgrade_fw[32 * 1024];

static uint8_t getDebugLevel(void);
#endif
/*----[HX_TP_SYS_DEBUG_LEVEL]----end */

/*----[HX_TP_SYS_REGISTER]-------start */
#ifdef HX_TP_SYS_REGISTER
static uint8_t register_command;
static uint8_t multi_register_command;
static uint8_t multi_register[8] = { 0x00 };
static uint8_t multi_cfg_bank[8] = { 0x00 };
static uint8_t multi_value[1024] = { 0x00 };

static bool config_bank_reg;
#endif
/*----[HX_TP_SYS_REGISTER]--------end */

/*----[HX_TP_SYS_DIAG]------------start */
#ifdef HX_TP_SYS_DIAG
static uint8_t x_channel;
static uint8_t y_channel;
static uint8_t *diag_mutual;
static uint8_t diag_command;
static uint8_t diag_coor[128];	/* = {0xFF}; */

static uint8_t diag_self[100] = { 0 };

static uint8_t *getMutualBuffer(void);
static uint8_t *getSelfBuffer(void);
static uint8_t getDiagCommand(void);
static uint8_t getXChannel(void);
static uint8_t getYChannel(void);

static void setMutualBuffer(void);
static void setXChannel(uint8_t x);
static void setYChannel(uint8_t y);

static uint8_t coordinate_dump_enable;
struct file *coordinate_fn;
#endif
/*----[HX_TP_SYS_DIAG]---------------end */

/*----[HX_TP_SYS_FLASH_DUMP]-------start */
#ifdef HX_TP_SYS_FLASH_DUMP
static uint8_t *flash_buffer;
static uint8_t flash_command;
static uint8_t flash_read_step;
static uint8_t flash_progress;
static uint8_t flash_dump_complete;
static uint8_t flash_dump_fail;
static uint8_t sys_operation;
static uint8_t flash_dump_sector;
static uint8_t flash_dump_page;
static bool flash_dump_going;

static uint8_t getFlashCommand(void);
static uint8_t getFlashDumpComplete(void);
static uint8_t getFlashDumpFail(void);
static uint8_t getFlashDumpProgress(void);
static uint8_t getFlashReadStep(void);
static uint8_t getSysOperation(void);
static uint8_t getFlashDumpSector(void);
static uint8_t getFlashDumpPage(void);
static bool getFlashDumpGoing(void);

static void setFlashBuffer(void);
static void setFlashCommand(uint8_t command);
static void setFlashReadStep(uint8_t step);
static void setFlashDumpComplete(uint8_t complete);
static void setFlashDumpFail(uint8_t fail);
static void setFlashDumpProgress(uint8_t progress);
static void setSysOperation(uint8_t operation);
static void setFlashDumpSector(uint8_t sector);
static void setFlashDumpPage(uint8_t page);
static void setFlashDumpGoing(bool going);
#endif
/*----[HX_TP_SYS_FLASH_DUMP]---------------end */

/*----[HX_TP_SYS_HITOUCH]----------------start */
#ifdef HX_TP_SYS_HITOUCH
static int hitouch_command;
static bool hitouch_is_connect;
#endif
/*---[HX_TP_SYS_HITOUCH]------------------end */

/*----[ASUS_BSP Josh_Sh]----------------start */
static int hx8528_cable_status(int status);
static int himax_firmware_upgrade(int path);
static int update_touch_progress(int update_progress);
static int update_touch_result(int result);

/*----[Proc_function]------start */

#define	HIMAX_ME372CL_PROC_REGISTER	"me372cl_register"
static struct proc_dir_entry *himax_me372cl_proc_register;

#define	HIMAX_ME372CL_PROC_DEBUG_FLAG	"touch_debug_log"
static struct proc_dir_entry *himax_me372cl_proc_debug_flag;

#define	HIMAX_ME372CL_PROC_DIAG_FILE	"me372cl_diag"
static struct proc_dir_entry *himax_me372cl_proc_diag_file;

#define	HIMAX_ME372CL_PROC_DEBUG_LEVEL	"me372cl_debug_level"
static struct proc_dir_entry *himax_me372cl_proc_debug_level;

#define	HIMAX_ME372CL_PROC_FLASH_DUMP	"me372cl_flash_dump"
static struct proc_dir_entry *himax_me372cl_proc_flash_dump;

/*----[Proc_function]------end */

#ifdef HX_TP_SYS_RESET
static ssize_t himax_reset_set(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count);
#endif
#ifdef HX_TP_SYS_REGISTER
static ssize_t himax_register_show(struct device *dev,
				   struct device_attribute *attr, char *buf);
static ssize_t himax_register_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count);
#endif
#ifdef HX_TP_SYS_DEBUG_LEVEL
static ssize_t himax_debug_level_show(struct device *dev,
				      struct device_attribute *attr, char *buf);
static ssize_t himax_debug_level_dump(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count);
#endif
#ifdef HX_TP_SYS_DIAG
static ssize_t himax_diag_show(struct device *dev,
			       struct device_attribute *attr, char *buf);
static ssize_t himax_diag_dump(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count);
static ssize_t himax_chip_raw_data_store(struct device *dev,
					 struct device_attribute *attr,
					 char *buf);
#endif
#ifdef HX_TP_SYS_FLASH_DUMP
static ssize_t himax_flash_show(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t himax_flash_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count);
#endif
#ifdef HX_TP_SYS_HITOUCH
static ssize_t himax_hitouch_show(struct device *dev,
				  struct device_attribute *attr, char *buf);
static ssize_t himax_hitouch_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count);
#endif
static ssize_t himax_get_touch_status(struct device *dev,
				      struct device_attribute *attr, char *buf);
static ssize_t himax_chip_check_running(struct device *dev,
					struct device_attribute *attr,
					char *buf);
static ssize_t himax_chip_enable_irq(struct device *dev,
				     struct device_attribute *attr, char *buf,
				     size_t count);
static ssize_t himax_get_fw_version(struct device *dev,
				    struct device_attribute *attr, char *buf);
static ssize_t himax_debug_log(struct device *dev,
			       struct device_attribute *attr, char *buf,
			       size_t count);
static ssize_t himax_checksum(struct device *dev, struct device_attribute *attr,
			      char *buf);

#ifdef HX_TP_SYS_SELF_TEST
static DEVICE_ATTR(tp_self_test, (S_IWUSR | S_IRUGO),
		   himax_chip_self_test_function, himax_self_test_setting);
#endif
#ifdef HX_TP_SYS_RESET
static DEVICE_ATTR(reset, (S_IWUSR | S_IRUGO), NULL, himax_reset_set);
#endif
#ifdef HX_TP_SYS_REGISTER
static DEVICE_ATTR(register, (S_IWUSR | S_IRUGO), himax_register_show,
		   himax_register_store);
#endif
#ifdef HX_TP_SYS_DEBUG_LEVEL
static DEVICE_ATTR(debug_level, (S_IWUSR | S_IRUGO), himax_debug_level_show,
		   himax_debug_level_dump);
#endif
#ifdef HX_TP_SYS_DIAG
static DEVICE_ATTR(diag, (S_IWUSR | S_IRUGO), himax_diag_show, himax_diag_dump);
static DEVICE_ATTR(tp_output_raw_data, (S_IWUSR | S_IRUGO),
		   himax_chip_raw_data_store, himax_diag_dump);
#endif
#ifdef HX_TP_SYS_FLASH_DUMP
static DEVICE_ATTR(flash_dump, (S_IWUSR | S_IRUGO), himax_flash_show,
		   himax_flash_store);
#endif
#ifdef HX_TP_SYS_HITOUCH
static DEVICE_ATTR(hitouch, (S_IWUSR | S_IRUGO), himax_hitouch_show,
		   himax_hitouch_store);
#endif
static DEVICE_ATTR(touch_status, (S_IWUSR | S_IRUGO), himax_get_touch_status,
		   NULL);
static DEVICE_ATTR(tp_check_running, (S_IWUSR | S_IRUGO),
		   himax_chip_check_running, NULL);
static DEVICE_ATTR(touch_irq, (S_IWUSR | S_IRUGO), NULL, himax_chip_enable_irq);
static DEVICE_ATTR(tp_fw_version, (S_IWUSR | S_IRUGO), himax_get_fw_version,
		   NULL);
static DEVICE_ATTR(debug_log, (S_IWUSR | S_IRUGO), NULL, himax_debug_log);
static DEVICE_ATTR(himax_check_sum, (S_IWUSR | S_IRUGO), himax_checksum, NULL);

static struct attribute *himax_attr[] = {
#ifdef HX_TP_SYS_REGISTER
	&dev_attr_register.attr,
#endif
#ifdef HX_TP_SYS_DIAG
	&dev_attr_diag.attr,
	&dev_attr_tp_output_raw_data.attr,
#endif
#ifdef HX_TP_SYS_SELF_TEST
	&dev_attr_tp_self_test.attr,
#endif
#ifdef HX_TP_SYS_RESET
	&dev_attr_reset.attr,
#endif
	&dev_attr_tp_fw_version.attr,
	&dev_attr_touch_status.attr,
	&dev_attr_touch_irq.attr,
	&dev_attr_tp_check_running.attr,
	&dev_attr_debug_log.attr,
	&dev_attr_himax_check_sum.attr,
	NULL
};

extern int Read_PROJ_ID(void);
/* extern int check_cable_status(void); */
extern int Read_HW_ID(void);

typedef enum {
	SELF,
	AP,
	LOCAL
} TOUCH_FW_PATH;

/*----[ASUS_BSP Josh_Sh]------------------end */

/*===================================================
//
//	Segment : Himax Normal Function
//
//===================================================
//----[ normal function]----------------------start */
void calculate_point_number(void)
{
	HX_TOUCH_INFO_POINT_CNT = HX_MAX_PT * 4;

	if ((HX_MAX_PT % 4) == 0) {
		HX_TOUCH_INFO_POINT_CNT += (HX_MAX_PT / 4) * 4;
	} else {
		HX_TOUCH_INFO_POINT_CNT += ((HX_MAX_PT / 4) + 1) * 4;
	}
}

void himax_ic_package_check(struct himax_ts_data *ts_modify)
{
	uint8_t cmd[3];
	uint8_t data[3];

	/*if( i2c_himax_read(himax_data->client, 0xD1, cmd, 3, DEFAULT_RETRY_CNT) < 0)
	   {
	   return ;
	   }

	   if( i2c_himax_read(himax_data->client, 0x31, data, 3, DEFAULT_RETRY_CNT) < 0)
	   {
	   return ;
	   } */

	/*if((data[0] == 0x85 && data[1] == 0x28) || (cmd[0] == 0x04 && cmd[1] == 0x85 && (cmd[2] == 0x26 || cmd[2] == 0x27 || cmd[2] == 0x28))) */
	if (1) {
		IC_TYPE = HX_85XX_D_SERIES_PWON;
		IC_CHECKSUM = HX_TP_BIN_CHECKSUM_CRC;
		/*Himax: Set FW and CFG Flash Address */
		FW_VER_MAJ_FLASH_ADDR = 133;	/*0x0085 */
		FW_VER_MAJ_FLASH_LENG = 1;;
		FW_VER_MIN_FLASH_ADDR = 134;	/*0x0086 */
		FW_VER_MIN_FLASH_LENG = 1;
		CFG_VER_MAJ_FLASH_ADDR = 160;	/*0x00A0 */
		CFG_VER_MAJ_FLASH_LENG = 12;
		CFG_VER_MIN_FLASH_ADDR = 172;	/*0x00AC */
		CFG_VER_MIN_FLASH_LENG = 12;

		printk("[Himax]: Himax IC package 8528 D\n");
	} else if ((data[0] == 0x85 && data[1] == 0x23)
		   || (cmd[0] == 0x03 && cmd[1] == 0x85
		       && (cmd[2] == 0x26 || cmd[2] == 0x27 || cmd[2] == 0x28
			   || cmd[2] == 0x29))) {
		IC_TYPE = HX_85XX_C_SERIES_PWON;
		IC_CHECKSUM = HX_TP_BIN_CHECKSUM_SW;
		/*Himax: Set FW and CFG Flash Address */
		FW_VER_MAJ_FLASH_ADDR = 133;	/*0x0085 */
		FW_VER_MAJ_FLASH_LENG = 1;
		FW_VER_MIN_FLASH_ADDR = 134;	/*0x0086 */
		FW_VER_MIN_FLASH_LENG = 1;
		CFG_VER_MAJ_FLASH_ADDR = 135;	/*0x0087 */
		CFG_VER_MAJ_FLASH_LENG = 12;
		CFG_VER_MIN_FLASH_ADDR = 147;	/*0x0093 */
		CFG_VER_MIN_FLASH_LENG = 12;

		printk("[Himax]: Himax IC package 8523 C\n");
	} else if ((data[0] == 0x85 && data[1] == 0x26)
		   || (cmd[0] == 0x02 && cmd[1] == 0x85
		       && (cmd[2] == 0x19 || cmd[2] == 0x25
			   || cmd[2] == 0x26))) {
		IC_TYPE = HX_85XX_B_SERIES_PWON;
		IC_CHECKSUM = HX_TP_BIN_CHECKSUM_SW;
		/* Himax: Set FW and CFG Flash Address     */
		FW_VER_MAJ_FLASH_ADDR = 133;	/*0x0085 */
		FW_VER_MAJ_FLASH_LENG = 1;
		FW_VER_MIN_FLASH_ADDR = 728;	/*0x02D8 */
		FW_VER_MIN_FLASH_LENG = 1;
		CFG_VER_MAJ_FLASH_ADDR = 692;	/*0x02B4 */
		CFG_VER_MAJ_FLASH_LENG = 3;
		CFG_VER_MIN_FLASH_ADDR = 704;	/*0x02C0 */
		CFG_VER_MIN_FLASH_LENG = 3;

		printk("[Himax]: Himax IC package 8526 B\n");
	} else if ((data[0] == 0x85 && data[1] == 0x20)
		   || (cmd[0] == 0x01 && cmd[1] == 0x85 && cmd[2] == 0x19)) {
		IC_TYPE = HX_85XX_A_SERIES_PWON;
		IC_CHECKSUM = HX_TP_BIN_CHECKSUM_SW;
		printk("[Himax]: Himax IC package 8520 A\n");
	} else {
		printk("[Himax]: Himax IC package incorrect!!\n");
	}
}

static int himax_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	uint8_t buf[2] = { 0 };
	int ret = 0;

	if (!himax_data->tp_firmware_upgrade_proceed) {
		is_suspend = false;

#ifdef HX_TP_SYS_FLASH_DUMP
		if (getFlashDumpGoing()) {
			printk(KERN_INFO
			       "[Himax] %s: Flash dump is going, reject suspend\n",
			       __func__);
			return 0;
		}
#endif

#ifdef HX_TP_SYS_HITOUCH
		if (hitouch_is_connect) {
			printk(KERN_INFO
			       "[Himax] %s: Hitouch connect, reject suspend\n",
			       __func__);
			return 0;
		}
#endif

		printk(KERN_INFO "[Himax] %s: TS suspend\n", __func__);

		wake_lock(&himax_data->wake_lock);
		mutex_lock(&himax_data->mutex_lock);

		buf[0] = HX_CMD_TSSOFF;
		ret =
		    i2c_himax_master_write(himax_data->client, buf, 1,
					   DEFAULT_RETRY_CNT);
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax] %s: I2C access HX_CMD_TSSOFF failed!\n",
			       __func__);
		}
		msleep(120);

		buf[0] = HX_CMD_TSSLPIN;
		ret =
		    i2c_himax_master_write(himax_data->client, buf, 1,
					   DEFAULT_RETRY_CNT);
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax] %s: I2C access HX_CMD_TSSLPIN failed!\n",
			       __func__);
		}
		msleep(120);

		buf[0] = HX_CMD_SETDEEPSTB;
		buf[1] = 0x01;
		ret =
		    i2c_himax_master_write(himax_data->client, buf, 2,
					   DEFAULT_RETRY_CNT);
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax] %s: I2C access HX_CMD_SETDEEPSTB failed!\n",
			       __func__);
		}
		msleep(120);

		mutex_unlock(&himax_data->mutex_lock);
		wake_unlock(&himax_data->wake_lock);

		/*----[ENABLE_CHIP_STATUS_MONITOR]--------------start */
#ifdef ENABLE_CHIP_STATUS_MONITOR
		himax_data->running_status = 1;
		cancel_delayed_work_sync(&himax_data->himax_chip_monitor);
#endif
		/*----[ENABLE_CHIP_STATUS_MONITOR]----------------end */

		printk(KERN_ERR "[Himax] %s: irq = %d\n", __func__,
		       himax_data->irq);
		disable_irq(himax_data->irq);

		ret = cancel_work_sync(&himax_data->work);
		if (ret) {
			enable_irq(himax_data->irq);
		}

		is_suspend = true;
	}

	return 0;
}

static int himax_ts_resume(struct i2c_client *client)
{
	uint8_t buf[2] = { 0 };
	int ret = 0;

	printk(KERN_INFO "[Himax] %s: TS resume\n", __func__);

	if (!himax_data->tp_firmware_upgrade_proceed) {
		if (!is_suspend) {
			printk(KERN_INFO
			       "[Himax] %s TP never enter suspend , reject the resume action\n",
			       __func__);
			return 0;
		}

		wake_lock(&himax_data->wake_lock);
		mutex_lock(&himax_data->mutex_lock);

		buf[0] = HX_CMD_SETDEEPSTB;
		buf[1] = 0x00;
		ret = i2c_himax_master_write(himax_data->client, buf, 2, DEFAULT_RETRY_CNT);	/*sense on */
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax] %s: HX_CMD_SETDEEPSTB failed addr = 0x%x\n",
			       __func__, himax_data->client->addr);
		}
		udelay(100);

		buf[0] = HX_CMD_TSSON;
		ret = i2c_himax_master_write(himax_data->client, buf, 1, DEFAULT_RETRY_CNT);	/*sense on */
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax] %s: HX_CMD_TSSON failed addr = 0x%x\n",
			       __func__, himax_data->client->addr);
		}
		msleep(30);

		buf[0] = HX_CMD_TSSLPOUT;	/*0x81 */
		ret = i2c_himax_master_write(himax_data->client, buf, 1, DEFAULT_RETRY_CNT);	/*sense on */
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax] %s: HX_CMD_TSSLPOUT failed addr = 0x%x\n",
			       __func__, himax_data->client->addr);
		}

		mutex_unlock(&himax_data->mutex_lock);
		wake_unlock(&himax_data->wake_lock);

		/*              err = himax_ts_poweron(); */

		/*----[HX_ESD_WORKAROUND]-------start   */
#ifdef HX_ESD_WORKAROUND
		ret = himax_hang_shaking();	/*0:Running, 1:Stop, 2:I2C Fail */
		if (ret == 2) {
			queue_delayed_work(himax_data->himax_wq,
					   &himax_data->himax_chip_reset_work,
					   0);
			printk(KERN_INFO "[Himax] %s: I2C Fail \n", __func__);
		}
		if (ret == 1) {
			printk(KERN_INFO "[Himax] %s: MCU Stop \n", __func__);
			/*Do HW_RESET?? */
			ESD_HW_REST();
		} else {
			printk(KERN_INFO "[Himax] %s: MCU Running \n",
			       __func__);
		}
#endif
		/*----[HX_ESD_WORKAROUND]----------------end */

		enable_irq(himax_data->irq);

		/*----[ENABLE_CHIP_STATUS_MONITOR]-----start */
#ifdef ENABLE_CHIP_STATUS_MONITOR
		queue_delayed_work(himax_data->himax_wq, &himax_data->himax_chip_monitor, 10 * HZ);	/*for ESD solution */
#endif
		/*----[ENABLE_CHIP_STATUS_MONITOR]-------------------end */

		/*----[HX_ESD_WORKAROUND]--------------------------start */
#ifdef HX_ESD_WORKAROUND
		ESD_COUNTER = 0;
#endif
		/*----[HX_ESD_WORKAROUND]----------------------------end */
	}
	return 0;
}

/*----[CONFIG_HAS_EARLYSUSPEND]----------------------start */
#ifdef CONFIG_HAS_EARLYSUSPEND
static void himax_ts_early_suspend(struct early_suspend *h)
{
	himax_ts_suspend(himax_data->client, PMSG_SUSPEND);
}

static void himax_ts_late_resume(struct early_suspend *h)
{
	himax_ts_resume(himax_data->client);
}
#endif
/*----[CONFIG_HAS_EARLYSUSPEND]----end	 */
/*----[ normal function]-------------end */

/*----[ i2c read/write function]---start */
static int i2c_himax_read(struct i2c_client *client, uint8_t command,
			  uint8_t *data, uint8_t length, uint8_t toRetry)
{
	int retry;
	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = 1,
		 .buf = &command,
		 },
		{
		 .addr = client->addr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = data,
		 }
	};

	for (retry = 0; retry < toRetry; retry++) {
		if (i2c_transfer(client->adapter, msg, 2) == 2) {
			break;
		}
		msleep(10);
	}
	if (retry == toRetry) {
		printk(KERN_INFO "[Himax] %s: i2c_read_block retry over %d\n",
		       __func__, toRetry);
		return -EIO;
	}
	return 0;
}

static int i2c_himax_write(struct i2c_client *client, uint8_t command,
			   uint8_t *data, uint8_t length, uint8_t toRetry)
{
	int retry, loop_i;
	uint8_t *buf = kzalloc(sizeof(uint8_t) * (length + 1), GFP_KERNEL);

	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = length + 1,
		 .buf = buf,
		 }
	};

	buf[0] = command;
	for (loop_i = 0; loop_i < length; loop_i++) {
		buf[loop_i + 1] = data[loop_i];
	}
	for (retry = 0; retry < toRetry; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1) {
			break;
		}
		msleep(10);
	}

	if (retry == toRetry) {
		printk(KERN_ERR "[Himax] %s: i2c_write_block retry over %d\n",
		       __func__, toRetry);
		kfree(buf);
		return -EIO;
	}
	kfree(buf);
	return 0;
}

static int i2c_himax_write_command(struct i2c_client *client, uint8_t command,
				   uint8_t toRetry)
{
	return i2c_himax_write(himax_data->client, command, NULL, 0, toRetry);
}

int i2c_himax_master_write(struct i2c_client *client, uint8_t *data,
			   uint8_t length, uint8_t toRetry)
{
	int retry, loop_i;
	uint8_t *buf = kzalloc(sizeof(uint8_t) * length, GFP_KERNEL);

	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = length,
		 .buf = buf,
		 }
	};

	for (loop_i = 0; loop_i < length; loop_i++) {
		buf[loop_i] = data[loop_i];
	}
	for (retry = 0; retry < toRetry; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1) {
			break;
		}
		msleep(10);
	}

	if (retry == toRetry) {
		printk(KERN_ERR "[Himax] %s: i2c_write_block retry over %d\n",
		       __func__, toRetry);
		kfree(buf);
		return -EIO;
	}
	kfree(buf);

	return 0;
}

/*----[ i2c read/write function]------end */

#ifdef HX_LOADIN_CONFIG
static int himax_config_flow()
{
	char data[4];
	uint8_t buf0[4];
	int i2c_CheckSum = 0;
	unsigned long i = 0;

	data[0] = HX_CMD_E3;
	data[1] = 0x00;		/*reload disable */
	if (i2c_himax_master_write
	    (himax_data->client, &data[0], 2, DEFAULT_RETRY_CNT) < 0) {
		goto HimaxErr;
	}

	if (i2c_himax_master_write
	    (himax_data->client, &c1[0], sizeof(c1), DEFAULT_RETRY_CNT) < 0) {
		goto HimaxErr;
	}

	if (i2c_himax_master_write
	    (himax_data->client, &c2[0], sizeof(c2), DEFAULT_RETRY_CNT) < 0) {
		goto HimaxErr;
	}

	if (i2c_himax_master_write
	    (himax_data->client, &c3[0], sizeof(c3), DEFAULT_RETRY_CNT) < 0) {
		goto HimaxErr;
	}

	if (i2c_himax_master_write
	    (himax_data->client, &c4[0], sizeof(c4), DEFAULT_RETRY_CNT) < 0) {
		goto HimaxErr;
	}

	if (i2c_himax_master_write
	    (himax_data->client, &c5[0], sizeof(c5), DEFAULT_RETRY_CNT) < 0) {
		goto HimaxErr;
	}

	if (i2c_himax_master_write
	    (himax_data->client, &c6[0], sizeof(c6), DEFAULT_RETRY_CNT) < 0) {
		goto HimaxErr;
	}

	if (i2c_himax_master_write
	    (himax_data->client, &c7[0], sizeof(c7), DEFAULT_RETRY_CNT) < 0) {
		goto HimaxErr;
	}

	if (i2c_himax_master_write
	    (himax_data->client, &c8[0], sizeof(c8), DEFAULT_RETRY_CNT) < 0) {
		goto HimaxErr;
	}

	if (i2c_himax_master_write
	    (himax_data->client, &c9[0], sizeof(c9), DEFAULT_RETRY_CNT) < 0) {
		goto HimaxErr;
	}

	if (i2c_himax_master_write
	    (himax_data->client, &c10[0], sizeof(c10), DEFAULT_RETRY_CNT) < 0) {
		goto HimaxErr;
	}

	if (i2c_himax_master_write
	    (himax_data->client, &c11[0], sizeof(c11), DEFAULT_RETRY_CNT) < 0) {
		goto HimaxErr;
	}

	if (i2c_himax_master_write
	    (himax_data->client, &c12[0], sizeof(c12), DEFAULT_RETRY_CNT) < 0) {
		goto HimaxErr;
	}

	if (i2c_himax_master_write
	    (himax_data->client, &c13[0], sizeof(c13), DEFAULT_RETRY_CNT) < 0) {
		goto HimaxErr;
	}

	if (i2c_himax_master_write
	    (himax_data->client, &c14[0], sizeof(c14), DEFAULT_RETRY_CNT) < 0) {
		goto HimaxErr;
	}

	if (i2c_himax_master_write
	    (himax_data->client, &c15[0], sizeof(c15), DEFAULT_RETRY_CNT) < 0) {
		goto HimaxErr;
	}

	if (i2c_himax_master_write
	    (himax_data->client, &c16[0], sizeof(c16), DEFAULT_RETRY_CNT) < 0) {
		goto HimaxErr;
	}

	if (i2c_himax_master_write
	    (himax_data->client, &c17[0], sizeof(c17), DEFAULT_RETRY_CNT) < 0) {
		goto HimaxErr;
	}

	if (i2c_himax_master_write
	    (himax_data->client, &c18[0], sizeof(c18), DEFAULT_RETRY_CNT) < 0) {
		goto HimaxErr;
	}

	if (i2c_himax_master_write
	    (himax_data->client, &c19[0], sizeof(c19), DEFAULT_RETRY_CNT) < 0) {
		goto HimaxErr;
	}

	if (i2c_himax_master_write
	    (himax_data->client, &c20[0], sizeof(c20), DEFAULT_RETRY_CNT) < 0) {
		goto HimaxErr;
	}

	if (i2c_himax_master_write
	    (himax_data->client, &c21[0], sizeof(c21), DEFAULT_RETRY_CNT) < 0) {
		goto HimaxErr;
	}

	if (i2c_himax_master_write
	    (himax_data->client, &c22[0], sizeof(c22), DEFAULT_RETRY_CNT) < 0) {
		goto HimaxErr;
	}

	if (i2c_himax_master_write
	    (himax_data->client, &c23[0], sizeof(c23), DEFAULT_RETRY_CNT) < 0) {
		goto HimaxErr;
	}

	if (i2c_himax_master_write
	    (himax_data->client, &c24[0], sizeof(c24), DEFAULT_RETRY_CNT) < 0) {
		goto HimaxErr;
	}

	if (i2c_himax_master_write
	    (himax_data->client, &c25[0], sizeof(c25), DEFAULT_RETRY_CNT) < 0) {
		goto HimaxErr;
	}

	if (i2c_himax_master_write
	    (himax_data->client, &c26[0], sizeof(c26), DEFAULT_RETRY_CNT) < 0) {
		goto HimaxErr;
	}

	if (i2c_himax_master_write
	    (himax_data->client, &c27[0], sizeof(c27), DEFAULT_RETRY_CNT) < 0) {
		goto HimaxErr;
	}

	if (i2c_himax_master_write
	    (himax_data->client, &c28[0], sizeof(c28), DEFAULT_RETRY_CNT) < 0) {
		goto HimaxErr;
	}

	if (i2c_himax_master_write
	    (himax_data->client, &c29[0], sizeof(c29), DEFAULT_RETRY_CNT) < 0) {
		goto HimaxErr;
	}

	if (i2c_himax_master_write
	    (himax_data->client, &c30[0], sizeof(c30), DEFAULT_RETRY_CNT) < 0) {
		goto HimaxErr;
	}

	if (i2c_himax_master_write
	    (himax_data->client, &c31[0], sizeof(c31), DEFAULT_RETRY_CNT) < 0) {
		goto HimaxErr;
	}

	if (i2c_himax_master_write
	    (himax_data->client, &c32[0], sizeof(c32), DEFAULT_RETRY_CNT) < 0) {
		goto HimaxErr;
	}

	if (i2c_himax_master_write
	    (himax_data->client, &c33[0], sizeof(c33), DEFAULT_RETRY_CNT) < 0) {
		goto HimaxErr;
	}

	if (i2c_himax_master_write
	    (himax_data->client, &c34[0], sizeof(c34), DEFAULT_RETRY_CNT) < 0) {
		goto HimaxErr;
	}

	if (i2c_himax_master_write
	    (himax_data->client, &c35[0], sizeof(c35), DEFAULT_RETRY_CNT) < 0) {
		goto HimaxErr;
	}

	/* i2c check sum start. */
	data[0] = 0xAB;
	data[1] = 0x00;
	if ((i2c_himax_master_write(himax_data->client, &data[0], 2, 3)) < 0) {
		goto HimaxErr;
	}

	data[0] = 0xAB;
	data[1] = 0x01;
	if ((i2c_himax_master_write(himax_data->client, &data[0], 2, 3)) < 0) {
		goto HimaxErr;
	}

	/*--config bank PART c36 START */
	data[0] = 0xE1;
	data[1] = 0x15;
	if ((i2c_himax_master_write(himax_data->client, &data[0], 2, 3)) < 0) {
		goto HimaxErr;
	}

	data[0] = 0xD8;
	data[1] = 0x00;
	data[2] = 0x00;		/*Start addr */
	if ((i2c_himax_master_write(himax_data->client, &data[0], 3, 3)) < 0) {
		goto HimaxErr;
	}

	if ((i2c_himax_master_write
	     (himax_data->client, &c36[0], sizeof(c36), 3)) < 0) {
		goto HimaxErr;
	}

	data[0] = 0xE1;
	data[1] = 0x00;
	if ((i2c_himax_master_write(himax_data->client, &data[0], 2, 3)) < 0) {
		goto HimaxErr;
	}

	for (i = 0; i < sizeof(c36); i++) {
		i2c_CheckSum += c36[i];
	}
	printk("Himax i2c_checksum_36_size = %d \n", sizeof(c36));
	printk("Himax i2c_checksum_36 = %d \n", i2c_CheckSum);

	i2c_CheckSum += 0x2AF;

	printk("Himax i2c_checksum_36 = %d \n", i2c_CheckSum);
	/*---config bank PART c36 END */

	/*---config bank PART c37 START */
	data[0] = 0xE1;
	data[1] = 0x15;
	if ((i2c_himax_master_write(himax_data->client, &data[0], 2, 3)) < 0) {
		goto HimaxErr;
	}

	data[0] = 0xD8;
	data[1] = 0x00;
	data[2] = 0x1E;		/*Start addr */
	if ((i2c_himax_master_write(himax_data->client, &data[0], 3, 3)) < 0) {
		goto HimaxErr;
	}

	if ((i2c_himax_master_write
	     (himax_data->client, &c37[0], sizeof(c37), 3)) < 0) {
		goto HimaxErr;
	}

	data[0] = 0xE1;
	data[1] = 0x00;
	if ((i2c_himax_master_write(himax_data->client, &data[0], 2, 3)) < 0) {
		goto HimaxErr;
	}

	for (i = 0; i < sizeof(c37); i++) {
		i2c_CheckSum += c37[i];
	}
	printk("Himax i2c_checksum_37_size = %d \n", sizeof(c37));
	printk("Himax i2c_checksum_37 = %d \n", i2c_CheckSum);
	i2c_CheckSum += 0x2CD;
	printk("Himax i2c_checksum_37 = %d \n", i2c_CheckSum);
	/*-----config bank PART c37 END */

	/*-----config bank PART c38 START */
	data[0] = 0xE1;
	data[1] = 0x15;
	if ((i2c_himax_master_write(himax_data->client, &data[0], 2, 3)) < 0) {
		goto HimaxErr;
	}

	data[0] = 0xD8;
	data[1] = 0x00;
	data[2] = 0x3C;		/* Start addr */
	if ((i2c_himax_master_write(himax_data->client, &data[0], 3, 3)) < 0) {
		goto HimaxErr;
	}

	if ((i2c_himax_master_write
	     (himax_data->client, &c38[0], sizeof(c38), 3)) < 0) {
		goto HimaxErr;
	}

	data[0] = 0xE1;
	data[1] = 0x00;
	if ((i2c_himax_master_write(himax_data->client, &data[0], 2, 3)) < 0) {
		goto HimaxErr;
	}

	for (i = 0; i < sizeof(c38); i++) {
		i2c_CheckSum += c38[i];
	}
	printk("Himax i2c_checksum_38_size = %d \n", sizeof(c38));
	printk("Himax i2c_checksum_38 = %d \n", i2c_CheckSum);
	i2c_CheckSum += 0x2EB;
	printk("Himax i2c_checksum_38 = %d \n", i2c_CheckSum);
	/*----config bank PART c38 END */

	/*----config bank PART c39 START */
	data[0] = 0xE1;
	data[1] = 0x15;
	if ((i2c_himax_master_write(himax_data->client, &data[0], 2, 3)) < 0) {
		goto HimaxErr;
	}

	data[0] = 0xD8;
	data[1] = 0x00;
	data[2] = 0x4C;		/* Start addr */
	if ((i2c_himax_master_write(himax_data->client, &data[0], 3, 3)) < 0) {
		goto HimaxErr;
	}

	if ((i2c_himax_master_write
	     (himax_data->client, &c39[0], sizeof(c39), 3)) < 0) {
		goto HimaxErr;
	}

	data[0] = 0xE1;
	data[1] = 0x00;
	if ((i2c_himax_master_write(himax_data->client, &data[0], 2, 3)) < 0) {
		goto HimaxErr;
	}

	for (i = 0; i < sizeof(c39); i++) {
		i2c_CheckSum += c39[i];
	}
	printk("Himax i2c_checksum_39_size = %d \n", sizeof(c39));
	printk("Himax i2c_checksum_39 = %d \n", i2c_CheckSum);
	i2c_CheckSum += 0x2FB;
	printk("Himax i2c_checksum_39 = %d \n", i2c_CheckSum);
	/*------config bank PART c39 END */

	/*------config bank PART c40 START */
	data[0] = 0xE1;
	data[1] = 0x15;
	if ((i2c_himax_master_write(himax_data->client, &data[0], 2, 3)) < 0) {
		goto HimaxErr;
	}

	data[0] = 0xD8;
	data[1] = 0x00;
	data[2] = 0x64;		/* Start addr */
	if ((i2c_himax_master_write(himax_data->client, &data[0], 3, 3)) < 0) {
		goto HimaxErr;
	}

	if ((i2c_himax_master_write
	     (himax_data->client, &c40[0], sizeof(c40), 3)) < 0) {
		goto HimaxErr;
	}

	data[0] = 0xE1;
	data[1] = 0x00;
	if ((i2c_himax_master_write(himax_data->client, &data[0], 2, 3)) < 0) {
		goto HimaxErr;
	}

	for (i = 0; i < sizeof(c40); i++) {
		i2c_CheckSum += c40[i];
	}
	printk("Himax i2c_checksum_40_size = %d \n", sizeof(c40));
	printk("Himax i2c_checksum_40 = %d \n", i2c_CheckSum);
	i2c_CheckSum += 0x313;
	printk("Himax i2c_checksum_40 = %d \n", i2c_CheckSum);
	/*-----config bank PART c40 END */

	/*-----config bank PART c41 START */
	data[0] = 0xE1;
	data[1] = 0x15;
	if ((i2c_himax_master_write(himax_data->client, &data[0], 2, 3)) < 0) {
		goto HimaxErr;
	}

	data[0] = 0xD8;
	data[1] = 0x00;
	data[2] = 0x7A;		/* Start addr */
	if ((i2c_himax_master_write(himax_data->client, &data[0], 3, 3)) < 0) {
		goto HimaxErr;
	}

	if ((i2c_himax_master_write
	     (himax_data->client, &c41[0], sizeof(c41), 3)) < 0) {
		goto HimaxErr;
	}

	data[0] = 0xE1;
	data[1] = 0x00;
	if ((i2c_himax_master_write(himax_data->client, &data[0], 2, 3)) < 0) {
		goto HimaxErr;
	}

	for (i = 0; i < sizeof(c41); i++) {
		i2c_CheckSum += c41[i];
	}
	printk("Himax i2c_checksum_41_size = %d \n", sizeof(c41));
	printk("Himax i2c_checksum_41 = %d \n", i2c_CheckSum);
	i2c_CheckSum += 0x329;
	printk("Himax i2c_checksum_41 = %d \n", i2c_CheckSum);
	/* --------config bank PART c41 END */

	/* --------config bank PART c42 START */
	data[0] = 0xE1;
	data[1] = 0x15;
	if ((i2c_himax_master_write(himax_data->client, &data[0], 2, 3)) < 0) {
		goto HimaxErr;
	}

	data[0] = 0xD8;
	data[1] = 0x00;
	data[2] = 0x96;		/* Start addr */
	if ((i2c_himax_master_write(himax_data->client, &data[0], 3, 3)) < 0) {
		goto HimaxErr;
	}

	if ((i2c_himax_master_write
	     (himax_data->client, &c42[0], sizeof(c42), 3)) < 0) {
		goto HimaxErr;
	}

	data[0] = 0xE1;
	data[1] = 0x00;
	if ((i2c_himax_master_write(himax_data->client, &data[0], 2, 3)) < 0) {
		goto HimaxErr;
	}

	for (i = 0; i < sizeof(c42); i++) {
		i2c_CheckSum += c42[i];
	}
	printk("Himax i2c_checksum_42_size = %d \n", sizeof(c42));
	printk("Himax i2c_checksum_42 = %d \n", i2c_CheckSum);
	i2c_CheckSum += 0x345;
	printk("Himax i2c_checksum_42 = %d \n", i2c_CheckSum);
	/*------config bank PART c42 END */

	/*------config bank PART c43_1 START */
	data[0] = 0xE1;
	data[1] = 0x15;
	if ((i2c_himax_master_write(himax_data->client, &data[0], 2, 3)) < 0) {
		goto HimaxErr;
	}

	data[0] = 0xD8;
	data[1] = 0x00;
	data[2] = 0x9E;		/*Start addr */
	if ((i2c_himax_master_write(himax_data->client, &data[0], 3, 3)) < 0) {
		goto HimaxErr;
	}

	if ((i2c_himax_master_write
	     (himax_data->client, &c43_1[0], sizeof(c43_1), 3)) < 0) {
		goto HimaxErr;
	}

	data[0] = 0xE1;
	data[1] = 0x00;
	if ((i2c_himax_master_write(himax_data->client, &data[0], 2, 3)) < 0) {
		goto HimaxErr;
	}

	for (i = 0; i < sizeof(c43_1); i++) {
		i2c_CheckSum += c43_1[i];
	}
	printk("Himax i2c_checksum_43_1_size = %d \n", sizeof(c43_1));
	printk("Himax i2c_checksum_43_1 = %d \n", i2c_CheckSum);
	i2c_CheckSum += 0x34D;
	printk("Himax i2c_checksum_43_1 = %d \n", i2c_CheckSum);
	/*-------config bank PART c43_1 END */

	/*-------config bank PART c43_2 START */
	data[0] = 0xE1;
	data[1] = 0x15;
	if ((i2c_himax_master_write(himax_data->client, &data[0], 2, 3)) < 0) {
		goto HimaxErr;
	}

	data[0] = 0xD8;
	data[1] = 0x00;
	data[2] = 0xBD;		/* Start addr */
	if ((i2c_himax_master_write(himax_data->client, &data[0], 3, 3)) < 0) {
		goto HimaxErr;
	}

	if ((i2c_himax_master_write
	     (himax_data->client, &c43_2[0], sizeof(c43_2), 3)) < 0) {
		goto HimaxErr;
	}

	data[0] = 0xE1;
	data[1] = 0x00;
	if ((i2c_himax_master_write(himax_data->client, &data[0], 2, 3)) < 0) {
		goto HimaxErr;
	}

	for (i = 0; i < sizeof(c43_2); i++) {
		i2c_CheckSum += c43_2[i];
	}
	printk("Himax i2c_checksum_43_2_size = %d \n", sizeof(c43_2));
	printk("Himax i2c_checksum_43_2 = %d \n", i2c_CheckSum);
	i2c_CheckSum += 0x36C;
	printk("Himax i2c_checksum_43_2 = %d \n", i2c_CheckSum);
	/*------config bank PART c43_2 END */

	/*------config bank PART c44_1 START */
	data[0] = 0xE1;
	data[1] = 0x15;
	if ((i2c_himax_master_write(himax_data->client, &data[0], 2, 3)) < 0) {
		goto HimaxErr;
	}

	data[0] = 0xD8;
	data[1] = 0x00;
	data[2] = 0xDA;		/* Start addr */
	if ((i2c_himax_master_write(himax_data->client, &data[0], 3, 3)) < 0) {
		goto HimaxErr;
	}

	if ((i2c_himax_master_write
	     (himax_data->client, &c44_1[0], sizeof(c44_1), 3)) < 0) {
		goto HimaxErr;
	}

	data[0] = 0xE1;
	data[1] = 0x00;
	if ((i2c_himax_master_write(himax_data->client, &data[0], 2, 3)) < 0) {
		goto HimaxErr;
	}

	for (i = 0; i < sizeof(c44_1); i++) {
		i2c_CheckSum += c44_1[i];
	}
	printk("Himax i2c_checksum_44_1_size = %d \n", sizeof(c44_1));
	printk("Himax i2c_checksum_44_1 = %d \n", i2c_CheckSum);
	i2c_CheckSum += 0x389;
	printk("Himax i2c_checksum_44_1 = %d \n", i2c_CheckSum);
	/*------config bank PART c44_1 END */

	/*------config bank PART c44_2 START */
	data[0] = 0xE1;
	data[1] = 0x15;
	if ((i2c_himax_master_write(himax_data->client, &data[0], 2, 3)) < 0) {
		goto HimaxErr;
	}

	data[0] = 0xD8;
	data[1] = 0x00;
	data[2] = 0xF9;		/*Start addr */
	if ((i2c_himax_master_write(himax_data->client, &data[0], 3, 3)) < 0) {
		goto HimaxErr;
	}

	if ((i2c_himax_master_write
	     (himax_data->client, &c44_2[0], sizeof(c44_2), 3)) < 0) {
		goto HimaxErr;
	}

	data[0] = 0xE1;
	data[1] = 0x00;
	if ((i2c_himax_master_write(himax_data->client, &data[0], 2, 3)) < 0) {
		goto HimaxErr;
	}

	for (i = 0; i < sizeof(c44_2); i++) {
		i2c_CheckSum += c44_2[i];
	}
	printk("Himax i2c_checksum_44_2_size = %d \n", sizeof(c44_2));
	printk("Himax i2c_checksum_44_2 = %d \n", i2c_CheckSum);
	i2c_CheckSum += 0x3A8;
	printk("Himax i2c_checksum_44_2 = %d \n", i2c_CheckSum);
	/*--------config bank PART c44_2 END */

	/*--------config bank PART c45 START */
	data[0] = 0xE1;
	data[1] = 0x15;
	if ((i2c_himax_master_write(himax_data->client, &data[0], 2, 3)) < 0) {
		goto HimaxErr;
	}

	data[0] = 0xD8;
	data[1] = 0x00;
	data[2] = 0xFE;		/* Start addr */
	if ((i2c_himax_master_write(himax_data->client, &data[0], 3, 3)) < 0) {
		goto HimaxErr;
	}

	if ((i2c_himax_master_write
	     (himax_data->client, &c45[0], sizeof(c45), 3)) < 0) {
		goto HimaxErr;
	}

	data[0] = 0xE1;
	data[1] = 0x00;
	if ((i2c_himax_master_write(himax_data->client, &data[0], 2, 3)) < 0) {
		goto HimaxErr;
	}

	for (i = 0; i < sizeof(c45); i++) {
		i2c_CheckSum += c45[i];
	}
	printk("Himax i2c_checksum_45_size = %d \n", sizeof(c45));
	printk("Himax i2c_checksum_45 = %d \n", i2c_CheckSum);
	i2c_CheckSum += 0x3AD;
	printk("Himax i2c_checksum_45 = %d \n", i2c_CheckSum);
	/*-------config bank PART c45 END */

	data[0] = 0xAB;
	data[1] = 0x10;
	if ((i2c_himax_master_write(himax_data->client, &data[0], 2, 3)) < 0) {
		goto HimaxErr;
	}

	i2c_CheckSum += 0xAB;
	i2c_CheckSum += 0x10;

	printk("Himax i2c_checksum_Final = %d \n", i2c_CheckSum);

	data[0] = 0xAC;
	data[1] = i2c_CheckSum & 0xFF;
	data[2] = (i2c_CheckSum >> 8) & 0xFF;
	if ((i2c_himax_master_write(himax_data->client, &data[0], 3, 3)) < 0) {
		goto HimaxErr;
	}

	printk("Himax i2c_checksum_AC = %d , %d \n", data[1], data[2]);

	int ret =
	    i2c_himax_read(himax_data->client, 0xAB, buf0, 2,
			   DEFAULT_RETRY_CNT);
	if (ret < 0) {
		printk(KERN_ERR
		       "[Himax]:i2c_himax_read 0xDA failed line: %d \n",
		       __LINE__);
		goto HimaxErr;
	}

	if (buf0[0] == 0x18) {
		return -1;
	}

	if (buf0[0] == 0x10) {
		return 1;
	}

	return 1;
HimaxErr:
	return -1;
}
#endif

/*----[ register flow function]------start */
static int himax_ts_poweron(void)
{
	uint8_t buf0[11];
	int ret = 0;
	int config_fail_retry = 0;

	mutex_lock(&himax_data->mutex_lock);

	/*----[ HX_85XX_C_SERIES_PWON]-----start */
	if (IC_TYPE == HX_85XX_C_SERIES_PWON) {
		buf0[0] = HX_CMD_MANUALMODE;
		buf0[1] = 0x02;
		ret = i2c_himax_master_write(himax_data->client, buf0, 2, DEFAULT_RETRY_CNT);	/*Reload Disable */
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax] %s: HX_CMD_MANUALMODE failed addr = 0x%x\n",
			       __func__, himax_data->client->addr);
			goto send_i2c_msg_fail;
		}
		udelay(100);

		buf0[0] = HX_CMD_SETMICROOFF;
		buf0[1] = 0x02;
		ret = i2c_himax_master_write(himax_data->client, buf0, 2, DEFAULT_RETRY_CNT);	/*Reload Disable */
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax] %s: HX_CMD_SETMICROOFF failed addr = 0x%x\n",
			       __func__, himax_data->client->addr);
			goto send_i2c_msg_fail;
		}
		udelay(100);

		buf0[0] = HX_CMD_SETROMRDY;
		buf0[1] = 0x0F;
		buf0[2] = 0x53;
		ret = i2c_himax_master_write(himax_data->client, buf0, 3, DEFAULT_RETRY_CNT);	/*enable flash */
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax] %s: HX_CMD_SETROMRDY failed addr = 0x%x\n",
			       __func__, himax_data->client->addr);
			goto send_i2c_msg_fail;
		}
		udelay(100);

		buf0[0] = HX_CMD_SET_CACHE_FUN;
		buf0[1] = 0x05;
		buf0[2] = 0x03;
		ret = i2c_himax_master_write(himax_data->client, buf0, 3, DEFAULT_RETRY_CNT);	/*prefetch */
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax] %s: HX_CMD_SET_CACHE_FUN failed addr = 0x%x\n",
			       __func__, himax_data->client->addr);
			goto send_i2c_msg_fail;
		}
		udelay(100);

		buf0[0] = HX_CMD_B9;
		buf0[1] = 0x01;
		buf0[2] = 0x2D;
		ret = i2c_himax_master_write(himax_data->client, buf0, 3, DEFAULT_RETRY_CNT);	/*prefetch */
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax] %s: HX_CMD_B9 failed addr = 0x%x\n",
			       __func__, himax_data->client->addr);
			goto send_i2c_msg_fail;
		}
		udelay(100);

		buf0[0] = HX_CMD_E3;
		buf0[1] = 0x00;
		ret = i2c_himax_master_write(himax_data->client, buf0, 2, DEFAULT_RETRY_CNT);	/*prefetch */
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax] %s: HX_CMD_E3 failed addr = 0x%x\n",
			       __func__, himax_data->client->addr);
			goto send_i2c_msg_fail;
		}
		udelay(100);

		buf0[0] = HX_CMD_TSSON;
		ret = i2c_himax_master_write(himax_data->client, buf0, 1, DEFAULT_RETRY_CNT);	/*sense on */
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax] %s: HX_CMD_TSSON failed addr = 0x%x\n",
			       __func__, himax_data->client->addr);
			goto send_i2c_msg_fail;
		}
		msleep(120);	/*120ms */

#ifdef HX_LOADIN_CONFIG
		if (himax_config_flow() == -1) {
			printk("[Himax]: send config fail. \n");
			/*goto send_i2c_msg_fail; */
		}
		msleep(100);	/*100ms */
#endif

		buf0[0] = HX_CMD_TSSLPOUT;
		ret = i2c_himax_master_write(himax_data->client, buf0, 1, DEFAULT_RETRY_CNT);	/*sense on */
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax] %s: HX_CMD_TSSLPOUT failed addr = 0x%x\n",
			       __func__, himax_data->client->addr);
			goto send_i2c_msg_fail;
		}
		msleep(120);	/*120ms */
	} else if (IC_TYPE == HX_85XX_A_SERIES_PWON) {
		buf0[0] = HX_CMD_MANUALMODE;
		buf0[1] = 0x02;
		ret = i2c_himax_master_write(himax_data->client, buf0, 2, DEFAULT_RETRY_CNT);	/*Reload Disable */
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax] %s: HX_CMD_MANUALMODE failed addr = 0x%x\n",
			       __func__, himax_data->client->addr);
			goto send_i2c_msg_fail;
		}
		udelay(100);

		buf0[0] = HX_CMD_SETMICROOFF;
		buf0[1] = 0x02;
		ret = i2c_himax_master_write(himax_data->client, buf0, 2, DEFAULT_RETRY_CNT);	/*Reload Disable */
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax] %s: HX_CMD_SETMICROOFF failed addr = 0x%x\n",
			       __func__, himax_data->client->addr);
			goto send_i2c_msg_fail;
		}
		udelay(100);

		buf0[0] = HX_CMD_SETROMRDY;
		buf0[1] = 0x0F;
		buf0[2] = 0x53;
		ret = i2c_himax_master_write(himax_data->client, buf0, 3, DEFAULT_RETRY_CNT);	/*enable flash */
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax] %s: HX_CMD_SETROMRDY failed addr = 0x%x\n",
			       __func__, himax_data->client->addr);
			goto send_i2c_msg_fail;
		}
		udelay(100);

		buf0[0] = HX_CMD_SET_CACHE_FUN;
		buf0[1] = 0x06;
		buf0[2] = 0x02;
		ret = i2c_himax_master_write(himax_data->client, buf0, 3, DEFAULT_RETRY_CNT);	/*prefetch */
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax] %s: HX_CMD_SET_CACHE_FUN failed addr = 0x%x\n",
			       __func__, himax_data->client->addr);
			goto send_i2c_msg_fail;
		}
		udelay(100);

		buf0[0] = HX_CMD_76;
		buf0[1] = 0x01;
		buf0[2] = 0x2D;
		ret = i2c_himax_master_write(himax_data->client, buf0, 3, DEFAULT_RETRY_CNT);	/*prefetch */
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax] %s: HX_CMD_76 failed addr = 0x%x\n",
			       __func__, himax_data->client->addr);
			goto send_i2c_msg_fail;
		}
		udelay(100);

#ifdef HX_LOADIN_CONFIG
		if (himax_config_flow() == -1) {
			printk("[Himax]: send config fail. \n");
		}
		msleep(100);	/*100ms */
#endif

		buf0[0] = HX_CMD_TSSON;
		ret = i2c_himax_master_write(himax_data->client, buf0, 1, DEFAULT_RETRY_CNT);	/*sense on */
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax] %s: HX_CMD_TSSON failed addr = 0x%x\n",
			       __func__, himax_data->client->addr);
			goto send_i2c_msg_fail;
		}
		msleep(120);	/*120ms */

		buf0[0] = HX_CMD_TSSLPOUT;
		ret = i2c_himax_master_write(himax_data->client, buf0, 1, DEFAULT_RETRY_CNT);	/*sense on */
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax] %s: HX_CMD_TSSLPOUT failed addr = 0x%x\n",
			       __func__, himax_data->client->addr);
			goto send_i2c_msg_fail;
		}
		msleep(120);	/*120ms */
	} else if (IC_TYPE == HX_85XX_B_SERIES_PWON) {
		buf0[0] = HX_CMD_MANUALMODE;
		buf0[1] = 0x02;
		ret = i2c_himax_master_write(himax_data->client, buf0, 2, DEFAULT_RETRY_CNT);	/*Reload Disable */
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax] %s: HX_CMD_MANUALMODE failed addr = 0x%x\n",
			       __func__, himax_data->client->addr);
			goto send_i2c_msg_fail;
		}
		udelay(100);

		buf0[0] = HX_CMD_SETMICROOFF;
		buf0[1] = 0x02;
		ret = i2c_himax_master_write(himax_data->client, buf0, 2, DEFAULT_RETRY_CNT);	/*Reload Disable */
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax] %s: HX_CMD_SETMICROOFF failed addr = 0x%x\n",
			       __func__, himax_data->client->addr);
			goto send_i2c_msg_fail;
		}
		udelay(100);

		buf0[0] = HX_CMD_SETROMRDY;
		buf0[1] = 0x0F;
		buf0[2] = 0x53;
		ret = i2c_himax_master_write(himax_data->client, buf0, 3, DEFAULT_RETRY_CNT);	/*enable flash */
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax] %s: HX_CMD_SETROMRDY failed addr = 0x%x\n",
			       __func__, himax_data->client->addr);
			goto send_i2c_msg_fail;
		}
		udelay(100);

		buf0[0] = HX_CMD_SET_CACHE_FUN;
		buf0[1] = 0x06;
		buf0[2] = 0x02;
		ret = i2c_himax_master_write(himax_data->client, buf0, 3, DEFAULT_RETRY_CNT);	/*prefetch */
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax] %s: HX_CMD_SET_CACHE_FUN failed addr = 0x%x\n",
			       __func__, himax_data->client->addr);
			goto send_i2c_msg_fail;
		}
		udelay(100);

		buf0[0] = HX_CMD_76;
		buf0[1] = 0x01;
		buf0[2] = 0x2D;
		ret = i2c_himax_master_write(himax_data->client, buf0, 3, DEFAULT_RETRY_CNT);	/*prefetch */
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax] %s: HX_CMD_76 failed addr = 0x%x\n",
			       __func__, himax_data->client->addr);
			goto send_i2c_msg_fail;
		}
		udelay(100);

#ifdef HX_LOADIN_CONFIG
		if (himax_config_flow() == -1) {
			printk("[Himax]: send config fail\n");
		}
		msleep(100);	/* 100ms */
#endif

		buf0[0] = HX_CMD_TSSON;
		ret = i2c_himax_master_write(himax_data->client, buf0, 1, DEFAULT_RETRY_CNT);	/* sense on */
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax] %s: HX_CMD_TSSON failed addr = 0x%x\n",
			       __func__, himax_data->client->addr);
			goto send_i2c_msg_fail;
		}
		msleep(120);	/*120ms */

		buf0[0] = HX_CMD_TSSLPOUT;
		ret = i2c_himax_master_write(himax_data->client, buf0, 1, DEFAULT_RETRY_CNT);	/*sense on */
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax] %s: HX_CMD_TSSLPOUT failed addr = 0x%x\n",
			       __func__, himax_data->client->addr);
			goto send_i2c_msg_fail;
		}
		msleep(120);	/*120ms */
	} else if (IC_TYPE == HX_85XX_D_SERIES_PWON) {
		buf0[0] = HX_CMD_MANUALMODE;	/*0x42 */
		buf0[1] = 0x02;
		ret = i2c_himax_master_write(himax_data->client, buf0, 2, DEFAULT_RETRY_CNT);	/*Reload Disable */
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax] %s: HX_CMD_MANUALMODE failed addr = 0x%x\n",
			       __func__, himax_data->client->addr);
			goto send_i2c_msg_fail;
		}
		udelay(100);

		buf0[0] = HX_CMD_SETROMRDY;	/*0x36 */
		buf0[1] = 0x0F;
		buf0[2] = 0x53;
		ret = i2c_himax_master_write(himax_data->client, buf0, 3, DEFAULT_RETRY_CNT);	/*enable flash */
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax] %s: HX_CMD_SETROMRDY failed addr = 0x%x\n",
			       __func__, himax_data->client->addr);
			goto send_i2c_msg_fail;
		}
		udelay(100);

		buf0[0] = HX_CMD_SET_CACHE_FUN;	/*0xDD */
		buf0[1] = 0x04;
		buf0[2] = 0x03;
		ret =
		    i2c_himax_master_write(himax_data->client, buf0, 3,
					   DEFAULT_RETRY_CNT);
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax] %s: HX_CMD_SET_CACHE_FUN failed addr = 0x%x\n",
			       __func__, himax_data->client->addr);
			goto send_i2c_msg_fail;
		}
		udelay(100);

		buf0[0] = HX_CMD_B9;	/*setCVDD */
		buf0[1] = 0x01;
		buf0[2] = 0x36;
		ret =
		    i2c_himax_master_write(himax_data->client, buf0, 3,
					   DEFAULT_RETRY_CNT);
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax] %s: HX_CMD_B9 failed addr = 0x%x\n",
			       __func__, himax_data->client->addr);
			goto send_i2c_msg_fail;
		}
		udelay(100);

		buf0[0] = HX_CMD_CB;
		buf0[1] = 0x01;
		buf0[2] = 0xF5;
		ret =
		    i2c_himax_master_write(himax_data->client, buf0, 3,
					   DEFAULT_RETRY_CNT);
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax] %s: HX_CMD_CB failed addr = 0x%x\n",
			       __func__, himax_data->client->addr);
			goto send_i2c_msg_fail;
		}
		udelay(100);

#ifdef HX_LOADIN_CONFIG
		/*load config */
		printk("[Himax]: start load config.\n");
		config_fail_retry = 0;
		while (true) {
			if (himax_config_flow() == -1) {
				config_fail_retry++;
				if (config_fail_retry == 3) {
					printk
					    ("[Himax]: himax_config_flow retry fail.\n");
					goto send_i2c_msg_fail;
				}
				printk("[Himax]: touch config retry = %d \n",
				       config_fail_retry);
			} else {
				break;
			}
		}
		printk("[Himax]: touch end load config.\n");
		msleep(100);	/*100ms */
#endif

		buf0[0] = HX_CMD_TSSON;
		ret = i2c_himax_master_write(himax_data->client, buf0, 1, DEFAULT_RETRY_CNT);	/*sense on */
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax] %s: HX_CMD_TSSON failed addr = 0x%x\n",
			       __func__, himax_data->client->addr);
			goto send_i2c_msg_fail;
		}
		msleep(120);	/*120ms */

		buf0[0] = HX_CMD_TSSLPOUT;	/*0x81 */
		ret = i2c_himax_master_write(himax_data->client, buf0, 1, DEFAULT_RETRY_CNT);	/*sense on */
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax] %s: HX_CMD_TSSLPOUT failed addr = 0x%x\n",
			       __func__, himax_data->client->addr);
			goto send_i2c_msg_fail;
		}
		msleep(120);	/*120ms */
	}

	mutex_unlock(&himax_data->mutex_lock);

	return ret;

send_i2c_msg_fail:

	printk(KERN_ERR "[Himax] %s: power on read/writ i2c msg fail. \n",
	       __func__);

	mutex_unlock(&himax_data->mutex_lock);

	/*----[ENABLE_CHIP_RESET_MACHINE]----start */
#ifdef ENABLE_CHIP_RESET_MACHINE
	if (himax_data->init_success) {
		queue_delayed_work(himax_data->himax_wq,
				   &himax_data->himax_chip_reset_work, 0);
	}
#endif
	/*----[ENABLE_CHIP_RESET_MACHINE]----end */

	return -1;
}

static int himax_ManualMode(int enter)
{
	uint8_t cmd[2];
	cmd[0] = enter;
	if (i2c_himax_write
	    (himax_data->client, 0x42, &cmd[0], 1, DEFAULT_RETRY_CNT) < 0) {
		printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
		return 0;
	}
	return 0;
}

static int himax_FlashMode(int enter)
{
	uint8_t cmd[2];
	cmd[0] = enter;
	if (i2c_himax_write
	    (himax_data->client, 0x43, &cmd[0], 1, DEFAULT_RETRY_CNT) < 0) {
		printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
		return 0;
	}
	return 0;
}

static int himax_lock_flash(void)
{
	uint8_t cmd[5];

	/* lock sequence start */
	cmd[0] = 0x01;
	cmd[1] = 0x00;
	cmd[2] = 0x06;
	if (i2c_himax_write
	    (himax_data->client, 0x43, &cmd[0], 3, DEFAULT_RETRY_CNT) < 0) {
		printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
		return 0;
	}

	cmd[0] = 0x03;
	cmd[1] = 0x00;
	cmd[2] = 0x00;
	if (i2c_himax_write
	    (himax_data->client, 0x44, &cmd[0], 3, DEFAULT_RETRY_CNT) < 0) {
		printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
		return 0;
	}

	cmd[0] = 0x00;
	cmd[1] = 0x00;
	cmd[2] = 0x7D;
	cmd[3] = 0x03;
	if (i2c_himax_write
	    (himax_data->client, 0x45, &cmd[0], 4, DEFAULT_RETRY_CNT) < 0) {
		printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
		return 0;
	}

	if (i2c_himax_write_command(himax_data->client, 0x4A, DEFAULT_RETRY_CNT)
	    < 0) {
		printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
		return 0;
	}
	mdelay(50);
	return 0;
	/* lock sequence stop */
}

static int himax_unlock_flash(void)
{
	uint8_t cmd[5];

	/* unlock sequence start */
	cmd[0] = 0x01;
	cmd[1] = 0x00;
	cmd[2] = 0x06;
	if (i2c_himax_write
	    (himax_data->client, 0x43, &cmd[0], 3, DEFAULT_RETRY_CNT) < 0) {
		printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
		return 0;
	}

	cmd[0] = 0x03;
	cmd[1] = 0x00;
	cmd[2] = 0x00;
	if (i2c_himax_write
	    (himax_data->client, 0x44, &cmd[0], 3, DEFAULT_RETRY_CNT) < 0) {
		printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
		return 0;
	}

	cmd[0] = 0x00;
	cmd[1] = 0x00;
	cmd[2] = 0x3D;
	cmd[3] = 0x03;
	if (i2c_himax_write
	    (himax_data->client, 0x45, &cmd[0], 4, DEFAULT_RETRY_CNT) < 0) {
		printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
		return 0;
	}

	if (i2c_himax_write_command(himax_data->client, 0x4A, DEFAULT_RETRY_CNT)
	    < 0) {
		printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
		return 0;
	}
	mdelay(50);

	return 0;
	/* unlock sequence stop */
}

static uint8_t himax_calculateChecksum(char *ImageBuffer, int fullLength)
{				/*, int address, int RST) */
	printk(KERN_ERR "[Himax] %s: IC_CHECKSUM = %d.\n", __func__,
	       IC_CHECKSUM);

	update_touch_progress(80);

	/*----[ HX_TP_BIN_CHECKSUM_SW]------start */
	if (IC_CHECKSUM == HX_TP_BIN_CHECKSUM_SW) {
		u16 checksum = 0;
		uint8_t cmd[5], last_byte;
		int FileLength, i, readLen, k, lastLength;

		FileLength = fullLength - 2;
		memset(cmd, 0x00, sizeof(cmd));

		/*//himax_hw_reset(RST);

		   //if((i2c_smbus_write_i2c_block_data(i2c_client, 0x81, 0, &cmd[0]))< 0)
		   //return 0;

		   //mdelay(120);
		   //printk("himax_marked, Sleep out: %d\n", __LINE__);
		   //himax_unlock_flash(); */

		himax_FlashMode(1);

		FileLength = (FileLength + 3) / 4;
		for (i = 0; i < FileLength; i++) {
			last_byte = 0;
			readLen = 0;

			cmd[0] = i & 0x1F;
			if (cmd[0] == 0x1F || i == FileLength - 1)
				last_byte = 1;
			cmd[1] = (i >> 5) & 0x1F;
			cmd[2] = (i >> 10) & 0x1F;
			if (i2c_himax_write
			    (himax_data->client, 0x44, &cmd[0], 3,
			     DEFAULT_RETRY_CNT) < 0) {
				printk(KERN_ERR
				       "[Himax] %s: i2c access fail!\n",
				       __func__);
				return 0;
			}

			if (i2c_himax_write_command
			    (himax_data->client, 0x46, DEFAULT_RETRY_CNT) < 0) {
				printk(KERN_ERR
				       "[Himax] %s: i2c access fail!\n",
				       __func__);
				return 0;
			}

			if (i2c_himax_read
			    (himax_data->client, 0x59, cmd, 4,
			     DEFAULT_RETRY_CNT) < 0) {
				printk(KERN_ERR
				       "[Himax] %s: i2c access fail!\n",
				       __func__);
				return -1;
			}

			if (i < (FileLength - 1)) {
				checksum += cmd[0] + cmd[1] + cmd[2] + cmd[3];
				if (i == 0) {
					printk(KERN_ERR
					       "[Himax] %s: himax_marked cmd 0 to 3 (first 4 bytes): %d, %d, %d, %d\n",
					       __func__, cmd[0], cmd[1], cmd[2],
					       cmd[3]);
				}
			} else {
				printk(KERN_ERR
				       "[Himax] %s: himax_marked cmd 0 to 3 (last 4 bytes): %d, %d, %d, %d\n",
				       __func__, cmd[0], cmd[1], cmd[2],
				       cmd[3]);
				printk(KERN_ERR
				       "[Himax] %s: himax_marked, checksum (not last): %d\n",
				       __func__, checksum);

				lastLength =
				    (((fullLength - 2) % 4) >
				     0) ? ((fullLength - 2) % 4) : 4;

				for (k = 0; k < lastLength; k++) {
					checksum += cmd[k];
				}
				printk(KERN_ERR
				       "[Himax] %s: himax_marked, checksum (final): %d\n",
				       __func__, checksum);

				/*Check Success */
				if (upgrade_fw[fullLength - 1] ==
				    (u8) (0xFF & (checksum >> 8))
				    && upgrade_fw[fullLength - 2] ==
				    (u8) (0xFF & checksum)) {
					himax_FlashMode(0);
					printk
					    ("[Himax] %s: HX_TP_BIN_CHECKSUM_SW: Check Success!\n",
					     __func__);
					return 1;
				} else {	/*Check Fail */

					himax_FlashMode(0);
					printk
					    ("[Himax] %s: HX_TP_BIN_CHECKSUM_SW: Check Fail!\n",
					     __func__);
					return 0;
				}
			}
		}
	} else if (IC_CHECKSUM == HX_TP_BIN_CHECKSUM_HW) {
		u32 sw_checksum = 0;
		u32 hw_checksum = 0;
		uint8_t cmd[5], last_byte;
		int FileLength, i, readLen, k, lastLength;

		FileLength = fullLength;
		memset(cmd, 0x00, sizeof(cmd));

		/*//himax_hw_reset(RST);

		   //if((i2c_smbus_write_i2c_block_data(i2c_client, 0x81, 0, &cmd[0]))< 0)
		   //return 0;

		   //mdelay(120);
		   //printk("himax_marked, Sleep out: %d\n", __LINE__);
		   //himax_unlock_flash(); */

		himax_FlashMode(1);

		FileLength = (FileLength + 3) / 4;
		for (i = 0; i < FileLength; i++) {
			last_byte = 0;
			readLen = 0;

			cmd[0] = i & 0x1F;
			if (cmd[0] == 0x1F || i == FileLength - 1) {
				last_byte = 1;
			}
			cmd[1] = (i >> 5) & 0x1F;
			cmd[2] = (i >> 10) & 0x1F;
			if (i2c_himax_write
			    (himax_data->client, 0x44, &cmd[0], 3,
			     DEFAULT_RETRY_CNT) < 0) {
				printk(KERN_ERR
				       "[Himax] %s: i2c access fail!\n",
				       __func__);
				return 0;
			}

			if (i2c_himax_write_command
			    (himax_data->client, 0x46, DEFAULT_RETRY_CNT) < 0) {
				printk(KERN_ERR
				       "[Himax] %s: i2c access fail!\n",
				       __func__);
				return 0;
			}

			if (i2c_himax_read
			    (himax_data->client, 0x59, cmd, 4,
			     DEFAULT_RETRY_CNT) < 0) {
				printk(KERN_ERR
				       "[Himax] %s: i2c access fail!\n",
				       __func__);
				return -1;
			}

			if (i < (FileLength - 1)) {
				sw_checksum +=
				    cmd[0] + cmd[1] + cmd[2] + cmd[3];
				if (i == 0) {
					printk(KERN_ERR
					       "[Himax] %s: himax_marked cmd 0 to 3 (first 4 bytes): %d, %d, %d, %d\n",
					       __func__, cmd[0], cmd[1], cmd[2],
					       cmd[3]);
				}
			} else {
				printk(KERN_ERR
				       "[Himax] %s: himax_marked cmd 0 to 3 (last 4 bytes): %d, %d, %d, %d\n",
				       __func__, cmd[0], cmd[1], cmd[2],
				       cmd[3]);
				printk(KERN_ERR
				       "[Himax] %s: himax_marked, sw_checksum (not last): %d\n",
				       __func__, sw_checksum);

				lastLength =
				    ((fullLength % 4) >
				     0) ? (fullLength % 4) : 4;

				for (k = 0; k < lastLength; k++) {
					sw_checksum += cmd[k];
				}
				printk(KERN_ERR
				       "[Himax] %s: himax_marked, sw_checksum (final): %d\n",
				       __func__, sw_checksum);

				/*Enable HW Checksum function. */
				cmd[0] = 0x01;
				if (i2c_himax_write
				    (himax_data->client, 0xE5, &cmd[0], 1,
				     DEFAULT_RETRY_CNT) < 0) {
					printk(KERN_ERR
					       "[Himax] %s: i2c access fail!\n",
					       __func__);
					return 0;
				}

				/*Must sleep 5 ms. */
				msleep(30);

				/*Get HW Checksum.  */
				if (i2c_himax_read
				    (himax_data->client, 0xAD, cmd, 4,
				     DEFAULT_RETRY_CNT) < 0) {
					printk(KERN_ERR
					       "[Himax] %s: i2c access fail!\n",
					       __func__);
					return -1;
				}

				hw_checksum =
				    cmd[0] + cmd[1] * 0x100 + cmd[2] * 0x10000 +
				    cmd[3] * 1000000;
				printk
				    ("[Himax] %s: himax_marked, sw_checksum (final): %d\n",
				     __func__, sw_checksum);
				printk
				    ("[Himax] %s: himax_marked, hw_checkusm (final): %d\n",
				     __func__, hw_checksum);

				/*Compare the checksum. */
				if (hw_checksum == sw_checksum) {
					himax_FlashMode(0);
					printk
					    ("[Himax] %s: HX_TP_BIN_CHECKSUM_HW: Compare the checksum.\n",
					     __func__);
					return 1;
				} else {
					himax_FlashMode(0);
					printk
					    ("[Himax] %s: HX_TP_BIN_CHECKSUM_HW: Check Fail!\n",
					     __func__);
					return 0;
				}
			}
		}
	} else if (IC_CHECKSUM == HX_TP_BIN_CHECKSUM_CRC) {
		uint8_t cmd[5];

		/*Set Flash Clock Rate */
		if (i2c_himax_read
		    (himax_data->client, 0x7F, cmd, 5, DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c access fail!\n",
			       __func__);
			return -1;
		}
		cmd[3] = 0x02;

		if (i2c_himax_write
		    (himax_data->client, 0x7F, &cmd[0], 5,
		     DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c access fail!\n",
			       __func__);
			return 0;
		}

		/*Enable Flash */
		himax_FlashMode(1);

		/*Select CRC Mode */
		cmd[0] = 0x05;
		cmd[1] = 0x00;
		cmd[2] = 0x00;
		if (i2c_himax_write
		    (himax_data->client, 0xD2, &cmd[0], 3,
		     DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c access fail!\n",
			       __func__);
			return 0;
		}

		/*Enable CRC Function */
		cmd[0] = 0x01;
		if (i2c_himax_write
		    (himax_data->client, 0xE5, &cmd[0], 1,
		     DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c access fail!\n",
			       __func__);
			return 0;
		}

		/*Must delay 30 ms */
		msleep(30);

		/*Read HW CRC */
		if (i2c_himax_read
		    (himax_data->client, 0xAD, cmd, 4, DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c access fail!\n",
			       __func__);
			return -1;
		}

		if (cmd[0] == 0 && cmd[1] == 0 && cmd[2] == 0 && cmd[3] == 0) {
			himax_FlashMode(0);
			printk
			    ("[Himax] %s: HX_TP_BIN_CHECKSUM_CRC: Compare the checksum.\n",
			     __func__);
			return 1;
		} else {
			himax_FlashMode(0);
			printk
			    ("[Himax] %s: HX_TP_BIN_CHECKSUM_CRC: Check Fail!\n",
			     __func__);
			return 0;
		}
	}
	printk("[Himax] %s: IC_CHECKSUM unusual \n", __func__);
	return 0;
}

/*----[HX_RST_PIN_FUNC]---------------------------start */
#ifdef HX_RST_PIN_FUNC
void himax_hw_reset(void)
{
	/*#ifdef HX_ESD_WORKAROUND */
	ESD_RESET_ACTIVATE = 1;
	/*#endif */
	gpio_set_value(himax_data->rst_gpio, 0);
	msleep(100);
	gpio_set_value(himax_data->rst_gpio, 1);
	msleep(100);
}
#endif
/*----[HX_RST_PIN_FUNC]-----------------------------end */
/*----[ register flow function]-----------------------end */

/*----[ ESD function]-------------------------------start */
int himax_hang_shaking(void)
{				/*0:Running, 1:Stop, 2:I2C Fail */
	int ret, result;
	uint8_t hw_reset_check[1];
	uint8_t hw_reset_check_2[1];
	uint8_t buf0[2];

	/*Mutexlock Protect Start */
	mutex_lock(&himax_data->mutex_lock);
	/*Mutexlock Protect End */

	/*Write 0x92 */
	buf0[0] = 0x92;
	if (IC_STATUS_CHECK == 0xAA) {
		buf0[1] = 0xAA;
		IC_STATUS_CHECK = 0x55;
	} else {
		buf0[1] = 0x55;
		IC_STATUS_CHECK = 0xAA;
	}

	ret =
	    i2c_himax_master_write(himax_data->client, buf0, 2,
				   DEFAULT_RETRY_CNT);
	if (ret < 0) {
		printk(KERN_ERR "[Himax]:write 0x92 failed line: %d \n",
		       __LINE__);
		goto work_func_send_i2c_msg_fail;
	}
	msleep(15);		/*Must more than 1 frame */

	buf0[0] = 0x92;
	buf0[1] = 0x00;
	ret =
	    i2c_himax_master_write(himax_data->client, buf0, 2,
				   DEFAULT_RETRY_CNT);
	if (ret < 0) {
		printk(KERN_ERR "[Himax]:write 0x92 failed line: %d \n",
		       __LINE__);
		goto work_func_send_i2c_msg_fail;
	}
	msleep(2);

	ret =
	    i2c_himax_read(himax_data->client, 0xDA, hw_reset_check, 1,
			   DEFAULT_RETRY_CNT);
	if (ret < 0) {
		printk(KERN_ERR
		       "[Himax]:i2c_himax_read 0xDA failed line: %d \n",
		       __LINE__);
		goto work_func_send_i2c_msg_fail;
	}
	/*printk("[Himax]: ESD 0xDA - 0x%x.\n", hw_reset_check[0]);  */

	if ((IC_STATUS_CHECK != hw_reset_check[0])) {
		msleep(2);
		ret =
		    i2c_himax_read(himax_data->client, 0xDA, hw_reset_check_2,
				   1, DEFAULT_RETRY_CNT);
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax]:i2c_himax_read 0xDA failed line: %d \n",
			       __LINE__);
			goto work_func_send_i2c_msg_fail;
		}
		/*printk("[Himax]: ESD check 2 0xDA - 0x%x.\n", hw_reset_check_2[0]); */

		if (hw_reset_check[0] == hw_reset_check_2[0]) {
			result = 1;	/*MCU Stop */
		} else {
			result = 0;	/*MCU Running */
		}
	} else {
		result = 0;	/*MCU Running */
	}

	/*Mutexlock Protect Start */
	mutex_unlock(&himax_data->mutex_lock);
	/*Mutexlock Protect End */
	return result;

work_func_send_i2c_msg_fail:
	/*Mutexlock Protect Start */
	mutex_unlock(&himax_data->mutex_lock);
	/*Mutexlock Protect End */
	return 2;
}

/*----[ENABLE_CHIP_RESET_MACHINE]-------start */
static void himax_chip_reset_function(struct work_struct *dat)
{
	int err = 1;

	printk("[Himax] %s: ++ \n", __func__);

	wake_lock(&himax_data->wake_lock);

	himax_hw_reset();

	err = himax_ts_poweron();
	if (err == 0) {
		printk("[Himax] %s: touch power on. \n", __func__);
	} else {
		printk(KERN_ERR "[Himax] %s: power on error = %d.\n", __func__,
		       err);
	}

	/*//    if(gpio_get_value(himax_data->intr_gpio) == 0)
	   //   {
	   //           printk("[Himax]%s: IRQ = 0, Enable IRQ\n", __func__); */
	enable_irq(himax_data->irq);
	/*      } */

	wake_unlock(&himax_data->wake_lock);

	err = himax_read_fw_ver(true);

	printk("[Himax] %s: -- \n", __func__);
}

/*----[ENABLE_CHIP_RESET_MACHINE]-------end */

/*----[HX_ESD_WORKAROUND]-------------start */
#ifdef HX_ESD_WORKAROUND
void ESD_HW_REST(void)
{
	ESD_RESET_ACTIVATE = 1;
	ESD_COUNTER = 0;

	printk("[Himax]: ESD - Reset\n");

	/*Wakelock Protect start */
	wake_lock(&himax_data->wake_lock);
	/*Wakelock Protect end */

	/*Mutexlock Protect Start */
	mutex_lock(&himax_data->mutex_lock);
	/*Mutexlock Protect End */

#ifdef HX_RST_BY_POWER
	/*power off by different platform's API */
	msleep(100);

	/*power on by different platform's API */
	msleep(100);

#else
	gpio_set_value(himax_data->rst_gpio, 0);
	msleep(30);
	gpio_set_value(himax_data->rst_gpio, 1);
	msleep(30);
#endif

	/*Mutexlock Protect Start */
	mutex_unlock(&himax_data->mutex_lock);
	/*Mutexlock Protec End */

	himax_ts_poweron(himax_data);

	/*Wakelock Protect start */
	wake_unlock(&himax_data->wake_lock);
	/*Wakelock Protect end */

	if (gpio_get_value(himax_data->intr_gpio) == 0) {
		printk("[Himax]%s: IRQ = 0, Enable IRQ\n", __func__);
		enable_irq(himax_data->irq);
	}
}
#endif
/*----[HX_ESD_WORKAROUND]-----end */

/*----[ENABLE_CHIP_STATUS_MONITOR]-----start */
#ifdef ENABLE_CHIP_STATUS_MONITOR
static int himax_chip_monitor_function(struct work_struct *dat)
{				/*for ESD solution */
	int ret;

	if (himax_data->running_status == 0) {	/*&& himax_chip->suspend_state == 0) */
		/*printk(KERN_INFO "[Himax] %s \n", __func__); */
		if (gpio_get_value(himax_data->intr_gpio) == 0) {
			printk("[Himax]%s: IRQ = 0, Enable IRQ\n", __func__);
			enable_irq(himax_data->irq);
		}

		ret = himax_hang_shaking();	/*0:Running, 1:Stop, 2:I2C Fail */
		if (ret == 2) {
			queue_delayed_work(himax_data->himax_wq,
					   &himax_data->himax_chip_reset_work,
					   0);
			printk(KERN_INFO "[Himax] %s: I2C Fail \n", __func__);
		}
		if (ret == 1) {
			printk(KERN_INFO "[Himax] %s: MCU Stop \n", __func__);
			/*himax_data->retry_time = 0; */
			ESD_HW_REST();
		}
		/*else
		   //printk(KERN_INFO "[Himax] %s: MCU Running \n", __func__); */

		queue_delayed_work(himax_data->himax_wq,
				   &himax_data->himax_chip_monitor, 10 * HZ);
	}
	return 0;
}
#endif
/*----[ENABLE_CHIP_STATUS_MONITOR]------end */
/*----[ ESD function]-----------start */

/*----[HX_PORTING_DEB_MSG]------start */
#ifdef HX_PORTING_DEB_MSG
static int himax_i2c_test_function(struct himax_ts_data *ts_modify)
{
	uint8_t buf0[5];
	int ret = 0;

	buf0[0] = 0xE9;
	buf0[1] = 0x01;
	buf0[2] = 0x01;

	while (1) {
		ret = i2c_himax_master_write(himax_data->client, buf0, 3, DEFAULT_RETRY_CNT);	/*sleep out */
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax] %s: i2c_master_send failed addr = 0x%x\n",
			       __func__, himax_data->client->addr);
		} else {
			printk(KERN_ERR "[Himax] %s: OK addr = 0x%x\n",
			       __func__, himax_data->client->addr);
		}
		mdelay(200);
	}
	return ret;
}
#endif
/*----[HX_PORTING_DEB_MSG]-----end */

/*----[HX_IREF_MODIFY]--------start */
#ifdef HX_IREF_MODIFY
int himax_modifyIref(void)
{
	/*int readLen; */
	unsigned char i;
	uint8_t cmd[5];
	uint8_t Iref[2] = { 0x00, 0x00 };

	cmd[0] = 0x01;
	cmd[1] = 0x00;
	cmd[2] = 0x08;
	if (i2c_himax_write
	    (himax_data->client, 0x43, &cmd[0], 3, DEFAULT_RETRY_CNT) < 0) {
		printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
		return 0;
	}

	cmd[0] = 0x00;
	cmd[1] = 0x00;
	cmd[2] = 0x00;
	if (i2c_himax_write
	    (himax_data->client, 0x44, &cmd[0], 3, DEFAULT_RETRY_CNT) < 0) {
		printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
		return 0;
	}

	if (i2c_himax_write_command(himax_data->client, 0x46, DEFAULT_RETRY_CNT)
	    < 0) {
		printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
		return 0;
	}

	if (i2c_himax_read(himax_data->client, 0x59, cmd, 4, DEFAULT_RETRY_CNT)
	    < 0) {
		printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
		return 0;
	}

	mdelay(5);
	for (i = 0; i < 16; i++) {
		if (cmd[1] == SFR_3u_1[i][0] && cmd[2] == SFR_3u_1[i][1]) {
			Iref[0] = SFR_6u_1[i][0];
			Iref[1] = SFR_6u_1[i][1];
		}
	}

	cmd[0] = 0x01;
	cmd[1] = 0x00;
	cmd[2] = 0x06;
	if (i2c_himax_write
	    (himax_data->client, 0x43, &cmd[0], 3, DEFAULT_RETRY_CNT) < 0) {
		printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
		return 0;
	}

	cmd[0] = 0x00;
	cmd[1] = 0x00;
	cmd[2] = 0x00;
	if (i2c_himax_write
	    (himax_data->client, 0x44, &cmd[0], 3, DEFAULT_RETRY_CNT) < 0) {
		printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
		return 0;
	}

	cmd[0] = Iref[0];
	cmd[1] = Iref[1];
	cmd[2] = 0x27;
	cmd[3] = 0x27;
	if (i2c_himax_write
	    (himax_data->client, 0x45, &cmd[0], 4, DEFAULT_RETRY_CNT) < 0) {
		printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
		return 0;
	}

	if (i2c_himax_write_command(himax_data->client, 0x4A, DEFAULT_RETRY_CNT)
	    < 0) {
		printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
		return 0;
	}
	return 1;
}
#endif
/*----[HX_IREF_MODIFY]------end */

static int himax_read_flash(unsigned char *buf, unsigned int addr_start,
			    unsigned int length)
{				/*OK */
	u16 i = 0;
	u16 j = 0;
	u16 k = 0;
	uint8_t cmd[4];
	u16 local_start_addr = addr_start / 4;
	u16 local_length = length;
	u16 local_end_addr = (addr_start + length) / 4 + 1;
	u16 local_addr = addr_start % 4;

	printk
	    ("[Himax] %s addr_start = %d , local_start_addr = %d , local_length = %d , local_end_addr = %d , local_addr = %d \n",
	     __func__, addr_start, local_start_addr, local_length,
	     local_end_addr, local_addr);
	if (i2c_himax_write_command(himax_data->client, 0x81, DEFAULT_RETRY_CNT)
	    < 0) {
		printk(KERN_ERR "[Himax]TOUCH_ERR: %s i2c write 81 fail.\n",
		       __func__);
		return 0;
	}
	msleep(120);
	if (i2c_himax_write_command(himax_data->client, 0x82, DEFAULT_RETRY_CNT)
	    < 0) {
		printk(KERN_ERR "[Himax]TOUCH_ERR: %s i2c write 82 fail.\n",
		       __func__);
		return 0;
	}
	msleep(100);
	cmd[0] = 0x01;
	if (i2c_himax_write
	    (himax_data->client, 0x43, &cmd[0], 1, DEFAULT_RETRY_CNT) < 0) {
		printk(KERN_ERR "[Himax]TOUCH_ERR: %s i2c write 43 fail.\n",
		       __func__);
		return 0;
	}
	msleep(100);
	i = local_start_addr;
	do {
		cmd[0] = i & 0x1F;
		cmd[1] = (i >> 5) & 0x1F;
		cmd[2] = (i >> 10) & 0x1F;
		if (i2c_himax_write
		    (himax_data->client, 0x44, &cmd[0], 3,
		     DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c access fail!\n",
			       __func__);
			return 0;
		}
		if (i2c_himax_write
		    (himax_data->client, 0x46, &cmd[0], 0,
		     DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c access fail!\n",
			       __func__);
			return 0;
		}
		if (i2c_himax_read
		    (himax_data->client, 0x59, cmd, 4, DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c access fail!\n",
			       __func__);
			return 0;
		}
		printk("[Himax]: cmd[0]=%d,cmd[1]=%d,cmd[2]=%d,cmd[3]=%d\n",
		       cmd[0], cmd[1], cmd[2], cmd[3]);
		if (i == local_start_addr) {	/*first page */
			j = 0;
			for (k = local_addr; k < 4 && j < local_length; k++) {
				buf[j++] = cmd[k];
			}
		} else {	/*other page */

			for (k = 0; k < 4 && j < local_length; k++) {
				buf[j++] = cmd[k];
			}
		}
		i++;
	} while (i < local_end_addr);
	cmd[0] = 0;
	if (i2c_himax_write
	    (himax_data->client, 0x43, &cmd[0], 1, DEFAULT_RETRY_CNT) < 0) {
		return 0;
	}
	return 1;
}

/*----[firmware version read]-------start */
static u8 himax_read_fw_ver(bool hw_reset)
{
	u16 fw_ver_maj_start_addr;
	u16 fw_ver_maj_end_addr;
	u16 fw_ver_maj_addr;
	u16 fw_ver_maj_length;

	u16 fw_ver_min_start_addr;
	u16 fw_ver_min_end_addr;
	u16 fw_ver_min_addr;
	u16 fw_ver_min_length;

	u16 cfg_ver_maj_start_addr;
	u16 cfg_ver_maj_end_addr;
	u16 cfg_ver_maj_addr;
	u16 cfg_ver_maj_length;

	u16 cfg_ver_min_start_addr;
	u16 cfg_ver_min_end_addr;
	u16 cfg_ver_min_addr;
	u16 cfg_ver_min_length;

	uint8_t cmd[3];
	u16 i = 0;
	u16 j = 0;
	u16 k = 0;
	int err = 1;

	fw_ver_maj_start_addr = FW_VER_MAJ_FLASH_ADDR / 4;	/* start addr = 133 / 4 = 33  */
	fw_ver_maj_length = FW_VER_MAJ_FLASH_LENG;	/* length = 1 */
	fw_ver_maj_end_addr = (FW_VER_MAJ_FLASH_ADDR + fw_ver_maj_length) / 4 + 1;	/* end addr = 134 / 4 = 33 */
	fw_ver_maj_addr = FW_VER_MAJ_FLASH_ADDR % 4;	/* 133 mod 4 = 1 */

	fw_ver_min_start_addr = FW_VER_MIN_FLASH_ADDR / 4;	/* start addr = 134 / 4 = 33 */
	fw_ver_min_length = FW_VER_MIN_FLASH_LENG;	/* length = 1 */
	fw_ver_min_end_addr = (FW_VER_MIN_FLASH_ADDR + fw_ver_min_length) / 4 + 1;	/* end addr = 135 / 4 = 33 */
	fw_ver_min_addr = FW_VER_MIN_FLASH_ADDR % 4;	/* 134 mod 4 = 2 */

	cfg_ver_maj_start_addr = CFG_VER_MAJ_FLASH_ADDR / 4;	/* start addr = 160 / 4 = 40 */
	cfg_ver_maj_length = CFG_VER_MAJ_FLASH_LENG;	/* length = 12 */
	cfg_ver_maj_end_addr = (CFG_VER_MAJ_FLASH_ADDR + cfg_ver_maj_length) / 4 + 1;	/* end addr = (160 + 12) / 4 = 43 */
	cfg_ver_maj_addr = CFG_VER_MAJ_FLASH_ADDR % 4;	/* 160 mod 4 = 0 */

	cfg_ver_min_start_addr = CFG_VER_MIN_FLASH_ADDR / 4;	/* start addr = 172 / 4 = 43 */
	cfg_ver_min_length = CFG_VER_MIN_FLASH_LENG;	/* length = 12 */
	cfg_ver_min_end_addr = (CFG_VER_MIN_FLASH_ADDR + cfg_ver_min_length) / 4 + 1;	/* end addr = (172 + 12) / 4 = 46 */
	cfg_ver_min_addr = CFG_VER_MIN_FLASH_ADDR % 4;	/* 172 mod 4 = 0 */

	himax_data->firmware_version_checksum = 0;

	disable_irq(himax_data->irq);

#ifdef HX_RST_PIN_FUNC
	if (hw_reset) {
		himax_hw_reset();
	}
#endif

	mutex_lock(&himax_data->mutex_lock);

	/*Sleep out */
	if (i2c_himax_write
	    (himax_data->client, 0x81, &cmd[0], 0, DEFAULT_RETRY_CNT) < 0) {
		printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
		goto err_finish;
	}
	mdelay(120);

	/*Enter flash mode */
	himax_FlashMode(1);

	/*Read Flash Start */
	/*FW Version MAJ */
	i = fw_ver_maj_start_addr;
	do {
		cmd[0] = i & 0x1F;	/*column = 33 mod 32 = 1 */
		cmd[1] = (i >> 5) & 0x1F;	/*page = 33 / 32 = 1 */
		cmd[2] = (i >> 10) & 0x1F;	/*sector = 33 / 1024 = 0 */

		if (i2c_himax_write
		    (himax_data->client, 0x44, &cmd[0], 3,
		     DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c access fail!\n",
			       __func__);
			goto err_finish;
		}
		if (i2c_himax_write
		    (himax_data->client, 0x46, &cmd[0], 0,
		     DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c access fail!\n",
			       __func__);
			goto err_finish;
		}
		if (i2c_himax_read
		    (himax_data->client, 0x59, cmd, 4, DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c access fail!\n",
			       __func__);
			goto err_finish;
		}

		if (i == fw_ver_maj_start_addr) {	/*first page */
			j = 0;
			for (k = fw_ver_maj_addr;
			     k < 4 && j < fw_ver_maj_length; k++) {
				FW_VER_MAJ_buff[j++] = cmd[k];
			}
		} else {	/*other page */

			for (k = 0; k < 4 && j < fw_ver_maj_length; k++) {
				FW_VER_MAJ_buff[j++] = cmd[k];
			}
		}
		i++;
	} while (i < fw_ver_maj_end_addr);

	/*FW Version MIN */
	i = fw_ver_min_start_addr;
	do {
		cmd[0] = i & 0x1F;	/*column = 33 mod 32 = 1 */
		cmd[1] = (i >> 5) & 0x1F;	/*page = 33 / 32 = 1 */
		cmd[2] = (i >> 10) & 0x1F;	/*sector = 33 / 1024 = 0 */

		if (i2c_himax_write
		    (himax_data->client, 0x44, &cmd[0], 3,
		     DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c access fail!\n",
			       __func__);
			goto err_finish;
		}
		if (i2c_himax_write
		    (himax_data->client, 0x46, &cmd[0], 0,
		     DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c access fail!\n",
			       __func__);
			goto err_finish;
		}
		if (i2c_himax_read
		    (himax_data->client, 0x59, cmd, 4, DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c access fail!\n",
			       __func__);
			goto err_finish;
		}

		if (i == fw_ver_min_start_addr) {	/*first page */
			j = 0;
			for (k = fw_ver_min_addr;
			     k < 4 && j < fw_ver_min_length; k++) {
				FW_VER_MIN_buff[j++] = cmd[k];
			}
		} else {	/*other page */

			for (k = 0; k < 4 && j < fw_ver_min_length; k++) {
				FW_VER_MIN_buff[j++] = cmd[k];
			}
		}
		i++;
	} while (i < fw_ver_min_end_addr);

	/*CFG Version MAJ */
	i = cfg_ver_maj_start_addr;
	do {
		cmd[0] = i & 0x1F;	/*column = 40 mod 32 = 8 */
		cmd[1] = (i >> 5) & 0x1F;	/*page = 40 / 32 = 1 */
		cmd[2] = (i >> 10) & 0x1F;	/*sector = 40 / 1024 = 0 */

		if (i2c_himax_write
		    (himax_data->client, 0x44, &cmd[0], 3,
		     DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c access fail!\n",
			       __func__);
			goto err_finish;
		}
		if (i2c_himax_write
		    (himax_data->client, 0x46, &cmd[0], 0,
		     DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c access fail!\n",
			       __func__);
			goto err_finish;
		}
		if (i2c_himax_read
		    (himax_data->client, 0x59, cmd, 4, DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c access fail!\n",
			       __func__);
			goto err_finish;
		}

		if (i == cfg_ver_maj_start_addr) {	/*first page */
			j = 0;
			for (k = cfg_ver_maj_addr;
			     k < 4 && j < cfg_ver_maj_length; k++) {
				CFG_VER_MAJ_buff[j++] = cmd[k];
			}
		} else {	/*other page */

			for (k = 0; k < 4 && j < cfg_ver_maj_length; k++) {
				CFG_VER_MAJ_buff[j++] = cmd[k];
			}
		}
		i++;
	} while (i < cfg_ver_maj_end_addr);

	/*CFG Version MIN */
	i = cfg_ver_min_start_addr;
	do {
		cmd[0] = i & 0x1F;	/*column = 43 mod 32 = 11 */
		cmd[1] = (i >> 5) & 0x1F;	/*page = 43 / 32 = 1 */
		cmd[2] = (i >> 10) & 0x1F;	/*sector = 43 / 1024 = 0 */

		if (i2c_himax_write
		    (himax_data->client, 0x44, &cmd[0], 3,
		     DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c access fail!\n",
			       __func__);
			goto err_finish;
		}
		if (i2c_himax_write
		    (himax_data->client, 0x46, &cmd[0], 0,
		     DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c access fail!\n",
			       __func__);
			goto err_finish;
		}
		if (i2c_himax_read
		    (himax_data->client, 0x59, cmd, 4, DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c access fail!\n",
			       __func__);
			goto err_finish;
		}

		if (i == cfg_ver_min_start_addr) {	/*first page */
			j = 0;
			for (k = cfg_ver_min_addr;
			     k < 4 && j < cfg_ver_min_length; k++) {
				CFG_VER_MIN_buff[j++] = cmd[k];
			}
		} else {	/*other page */

			for (k = 0; k < 4 && j < cfg_ver_min_length; k++) {
				CFG_VER_MIN_buff[j++] = cmd[k];
			}
		}
		i++;
	} while (i < cfg_ver_min_end_addr);

	/*Exit flash mode */
	himax_FlashMode(0);

	/***********************************
	  Check FW Version , TBD
	  FW Major version : FW_VER_MAJ_buff
	  FW Minor version : FW_VER_MIN_buff
	  CFG Major version : CFG_VER_MAJ_buff
	  CFG Minor version : CFG_VER_MIN_buff

	  return 0 :
	  return 1 :
	  return 2 :
	 ***********************************/

	printk("[Himax]: FW_VER_MAJ_buff : %d \n", FW_VER_MAJ_buff[0]);
	printk("[Himax]: FW_VER_MIN_buff : %d \n", FW_VER_MIN_buff[0]);

	printk("[Himax]: CFG_VER_MAJ_buff : ");
	for (i = 0; i < 12; i++)
		printk(" %d ,", CFG_VER_MAJ_buff[i]);
	printk("\n");

	printk("[Himax]: CFG_VER_MIN_buff : ");
	for (i = 0; i < 12; i++)
		printk(" %d ,", CFG_VER_MIN_buff[i]);
	printk("\n");

	sprintf(himax_data->himax_firmware_version, "%2.2X%2.2X",
		FW_VER_MAJ_buff[0], FW_VER_MIN_buff[0]);
	himax_data->firmware_version_checksum =
	    ((himax_data->himax_firmware_version[0] << 24) | (himax_data->
							      himax_firmware_version
							      [1] << 16) |
	     (himax_data->himax_firmware_version[2] << 8) | himax_data->
	     himax_firmware_version[3]);
	printk("[Himax]: Himax firmware version: %s \n",
	       himax_data->himax_firmware_version);
	printk("[Himax]: Himax firmware version checksum: %d \n",
	       himax_data->firmware_version_checksum);

	sprintf(himax_data->tp_lens_version, "%2.2X", CFG_VER_MAJ_buff[0]);
	printk("[Himax]: TP Lens version: %s. \n", himax_data->tp_lens_version);

	sprintf(himax_data->config_firmware_version, "%2.2X",
		CFG_VER_MIN_buff[11]);
	himax_data->config_version_checksum =
	    ((himax_data->config_firmware_version[0] << 8) | himax_data->
	     config_firmware_version[1]);
	printk("[Himax]: Himax config version: %s. \n",
	       himax_data->config_firmware_version);
	printk("[Himax]: Himax config version checksum: %d. \n",
	       himax_data->config_version_checksum);

err_finish:
	mutex_unlock(&himax_data->mutex_lock);

	himax_hw_reset();

	err = himax_ts_poweron();
	if (err == 0) {
		printk("[Himax] %s: touch power on. \n", __func__);
	} else {
		printk(KERN_ERR "[Himax] %s: power on error = %d.\n", __func__,
		       err);
	}

	enable_irq(himax_data->irq);

	return 0;
}

/*----[firmware version read]-----------------------------------------------------------------------------end */

void himax_touch_information(void)
{
	static unsigned char temp_buffer[6];
	if (IC_TYPE == HX_85XX_A_SERIES_PWON) {
		HX_RX_NUM = 0;
		HX_TX_NUM = 0;
		HX_BT_NUM = 0;
		HX_X_RES = 0;
		HX_Y_RES = 0;
		HX_MAX_PT = 0;
		HX_XY_REVERSE = false;
		HX_INT_IS_EDGE = false;
	} else if (IC_TYPE == HX_85XX_B_SERIES_PWON) {
		HX_RX_NUM = 0;
		HX_TX_NUM = 0;
		HX_BT_NUM = 0;
		HX_X_RES = 0;
		HX_Y_RES = 0;
		HX_MAX_PT = 0;
		HX_XY_REVERSE = false;
		HX_INT_IS_EDGE = false;
	} else if (IC_TYPE == HX_85XX_C_SERIES_PWON) {
		/*RX,TX,BT Channel num */
		himax_read_flash(temp_buffer, 0x3D5, 3);
		HX_RX_NUM = temp_buffer[0];
		HX_TX_NUM = temp_buffer[1];
		HX_BT_NUM = (temp_buffer[2]) & 0x1F;

		/*Interrupt is level or edge */
		himax_read_flash(temp_buffer, 0x3EE, 2);

		if ((temp_buffer[0] && 0x04) == 0x04) {
			HX_XY_REVERSE = true;
		} else {
			HX_XY_REVERSE = false;
		}

		if ((temp_buffer[1] && 0x01) == 1) {
			HX_INT_IS_EDGE = true;
		} else {
			HX_INT_IS_EDGE = false;
		}

		/*Resolution */
		himax_read_flash(temp_buffer, 0x345, 4);
		if (HX_XY_REVERSE) {
			HX_X_RES = temp_buffer[2] * 256 + temp_buffer[3];
			HX_Y_RES = temp_buffer[0] * 256 + temp_buffer[1];
		} else {
			HX_X_RES = temp_buffer[0] * 256 + temp_buffer[1];
			HX_Y_RES = temp_buffer[2] * 256 + temp_buffer[3];
		}

		/*Point number */
		himax_read_flash(temp_buffer, 0x3ED, 1);
		HX_MAX_PT = temp_buffer[0] >> 4;
	} else if (IC_TYPE == HX_85XX_D_SERIES_PWON) {
		/*himax_read_flash( temp_buffer, 0x26E, 3);
		   HX_RX_NUM = temp_buffer[0];
		   HX_TX_NUM = temp_buffer[1];
		   HX_MAX_PT = (temp_buffer[2] & 0xF0) >> 4;

		   #ifdef HX_EN_SEL_BUTTON
		   HX_BT_NUM = (temp_buffer[2] & 0x0F);
		   #endif

		   #ifdef HX_EN_MUT_BUTTON
		   himax_read_flash( temp_buffer, 0x262, 1);
		   HX_BT_NUM = (temp_buffer[0] & 0x07);
		   #endif

		   himax_read_flash( temp_buffer, 0x272, 6);
		   if((temp_buffer[0] & 0x04) == 0x04)
		   {
		   HX_XY_REVERSE = true;

		   HX_X_RES = temp_buffer[4]*256 + temp_buffer[5];
		   HX_Y_RES = temp_buffer[2]*256 + temp_buffer[3];
		   }
		   else
		   {
		   HX_XY_REVERSE = false;

		   HX_X_RES = temp_buffer[2]*256 + temp_buffer[3];
		   HX_Y_RES = temp_buffer[4]*256 + temp_buffer[5];
		   } */

#if defined(CONFIG_PF450CL)
		HX_RX_NUM = 13;
		HX_TX_NUM = 24;
		HX_MAX_PT = 10;
		HX_X_RES = 480;
		HX_Y_RES = 854;
#else
		HX_RX_NUM = 30;
		HX_TX_NUM = 18;
		HX_MAX_PT = 10;
		HX_X_RES = 800;
		HX_Y_RES = 1280;
#endif
		himax_read_flash(temp_buffer, 0x200, 6);
		if ((temp_buffer[1] & 0x01) == 1) {
			HX_INT_IS_EDGE = true;
		} else {
			HX_INT_IS_EDGE = false;
		}
	} else {
		HX_RX_NUM = 0;
		HX_TX_NUM = 0;
		HX_BT_NUM = 0;
		HX_X_RES = 0;
		HX_Y_RES = 0;
		HX_MAX_PT = 0;
		HX_XY_REVERSE = false;
		HX_INT_IS_EDGE = false;
	}
}

/*========================================
//
//	Segment : Asus debug attr function
//  add by josh
//======================================== */

static ssize_t himax_get_touch_status(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	printk(KERN_INFO "[Himax]: touch status=%d. \n", himax_data->tp_status);
	return sprintf(buf, "%d \n", himax_data->tp_status);
}

static ssize_t himax_chip_check_running(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int ret;

	printk(KERN_INFO "[Himax]: himax_chip_check_running \n");
	ret = himax_hang_shaking();	/*0:Running, 1:Stop, 2:I2C Fail */
	if (ret == 2) {
		return sprintf(buf, "[Himax]: I2C Fail.\n");
	}
	if (ret == 1) {
		return sprintf(buf, "[Himax]: MCU Stop.\n");
	} else
		return sprintf(buf, "[Himax]: MCU Running\n");

}

static ssize_t himax_chip_enable_irq(struct device *dev,
				     struct device_attribute *attr, char *buf,
				     size_t count)
{
	if (buf[0] == '0') {
		if (irq_enable) {
			printk(KERN_INFO "[Himax]: IRQ is enable\n");
		} else {
			printk(KERN_INFO "[Himax]: IRQ is disable\n");
		}
	} else if (buf[0] == '1') {
		printk(KERN_INFO "[Himax]: himax_chip_enable_irq \n");
		enable_irq(himax_data->irq);
		irq_enable = true;
	} else if (buf[0] == '2') {
		printk(KERN_INFO "[Himax]: himax_chip_disable_irq \n");
		disable_irq(himax_data->irq);
		irq_enable = false;
	}

	return count;
}

static ssize_t himax_get_fw_version(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	printk(KERN_INFO "[Himax]: D%s.%s.%s-%s\n",
	       himax_data->himax_firmware_version, himax_data->tp_lens_version,
	       himax_data->config_firmware_version, DRIVER_VERSION);
	return sprintf(buf, "D%s.%s.%s-%s\n",
		       himax_data->himax_firmware_version,
		       himax_data->tp_lens_version,
		       himax_data->config_firmware_version, DRIVER_VERSION);
}

static ssize_t himax_debug_log(struct device *dev,
			       struct device_attribute *attr, char *buf,
			       size_t count)
{
	if (buf[0] == '0') {
		printk(KERN_INFO "[Himax]: Close debug log\n");
		debug_log = false;
	} else if (buf[0] == '1') {
		printk(KERN_INFO "[Himax]: Open debug log\n");
		debug_log = true;
	}
	return count;
}

static ssize_t himax_checksum(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	uint8_t checksumResult = 0;

	disable_irq(himax_data->irq);
	himax_FlashMode(0);
	himax_ManualMode(0);
	checksumResult = himax_calculateChecksum(0, 32768);	/*, address, RST); */
	/*himax_ManualMode(0); */
	himax_lock_flash();
	enable_irq(himax_data->irq);

	if (checksumResult) {	/*Success */
		printk(KERN_ERR "[Himax] %s: checksum Result success!\n",
		       __func__);
		return sprintf(buf, "[Himax] checksum Result success!\n");
	} else {		/*Fail */

		printk(KERN_ERR "[Himax] %s: checksum Result fail!\n",
		       __func__);
		return sprintf(buf, "[Himax] checksum Result fail!\n");
	}
}

/*--------[proc_function]-----------start */
static int himax_me372cl_chip_proc_chip_check_running(struct seq_file *buf,
						      void *v)
{
	int ret;

	printk(KERN_INFO "[Himax]: himax_chip_check_running \n");
	ret = himax_hang_shaking();	/*0:Running, 1:Stop, 2:I2C Fail */
	if (ret == 2) {
		seq_printf(buf, "[Himax]: I2C Fail.\n");
		return 0;
	}
	if (ret == 1) {

		seq_printf(buf, "[Himax]: MCU Stop.\n");
		/*return 0; */
	} else {

		seq_printf(buf, "[Himax]: MCU Running\n");
		return 0;
	}
}

static int proc_chip_check_running_open(struct inode *inode, struct file *file)
{
	return single_open(file, himax_me372cl_chip_proc_chip_check_running,
			   NULL);
}

static ssize_t himax_me372cl_chip_proc_debug_flag(struct file *filp,
						  const char *buf,
						  unsigned long len, void *data)
{

	if (buf[0] == '0') {
		printk(KERN_INFO "[Himax_ME372CL]: Close debug log\n");
		debug_log = false;
	} else if (buf[0] == '1') {
		printk(KERN_INFO "[Himax_ME372CL]: Open debug log\n");
		debug_log = true;
	}
	return len;
}

static const struct file_operations debug_flag_fops = {
	.owner = THIS_MODULE,
	/*.open = proc_chip_check_running_open, */
	.read = himax_me372cl_chip_proc_chip_check_running,
	.write = himax_me372cl_chip_proc_debug_flag,
};

static void himax_me372cl_chip_create_proc_debug_flag(void)
{
	himax_me372cl_proc_debug_flag =
	    proc_create(HIMAX_ME372CL_PROC_DEBUG_FLAG, 0666, NULL,
			&debug_flag_fops);
	if (himax_me372cl_proc_debug_flag) {
	} else {
		printk(KERN_ERR
		       "[Himax_ME372CL] %s: proc config file create failed!\n",
		       __func__);
	}
}

static void himax_me372cl_chip_remove_proc_debug_flag(void)
{
	extern struct proc_dir_entry proc_root;
	printk(KERN_ERR "[Himax_ME372CL]: %s.\n", __func__);
	remove_proc_entry(HIMAX_ME372CL_PROC_DEBUG_FLAG, &proc_root);
}

static ssize_t himax_me372cl_chip_proc_diag_read(char *buf, char **start,
						 off_t off, int count1,
						 int *eof, void *data)
{
	size_t count = 0;
	uint32_t loop_i;
	uint16_t mutual_num, self_num, width;

	mutual_num = x_channel * y_channel;
	self_num = x_channel + y_channel;	/*don't add KEY_COUNT */
	width = x_channel;
	count +=
	    sprintf(buf + count, "ChannelStart: %4d, %4d\n\n", x_channel,
		    y_channel);

	/* start to show out the raw data in adb shell */
	if (diag_command >= 1 && diag_command <= 6) {
		if (diag_command <= 3) {
			for (loop_i = 0; loop_i < mutual_num; loop_i++) {
				count +=
				    sprintf(buf + count, "%4d",
					    diag_mutual[loop_i]);
				if ((loop_i % width) == (width - 1)) {
					count +=
					    sprintf(buf + count, " %3d\n",
						    diag_self[width +
							      loop_i / width]);
				}
			}
			count += sprintf(buf + count, "\n");
			for (loop_i = 0; loop_i < width; loop_i++) {
				count +=
				    sprintf(buf + count, "%4d",
					    diag_self[loop_i]);
				if (((loop_i) % width) == (width - 1)) {
					count += sprintf(buf + count, "\n");
				}
			}

#ifdef HX_EN_SEL_BUTTON
			count += sprintf(buf + count, "\n");
			for (loop_i = 0; loop_i < HX_BT_NUM; loop_i++) {
				count +=
				    sprintf(buf + count, "%4d",
					    diag_self[HX_RX_NUM + HX_TX_NUM +
						      loop_i]);
			}
#endif
		} else if (diag_command > 4) {
			for (loop_i = 0; loop_i < self_num; loop_i++) {
				count +=
				    sprintf(buf + count, "%4d",
					    diag_self[loop_i]);
				if (((loop_i - mutual_num) % width) ==
				    (width - 1)) {
					count += sprintf(buf + count, "\n");
				}
			}
		} else {
			for (loop_i = 0; loop_i < mutual_num; loop_i++) {
				count +=
				    sprintf(buf + count, "%4d",
					    diag_mutual[loop_i]);
				if ((loop_i % width) == (width - 1)) {
					count += sprintf(buf + count, "\n");
				}
			}
		}
		count += sprintf(buf + count, "ChannelEnd");
		count += sprintf(buf + count, "\n");
	} else if (diag_command == 7) {
		for (loop_i = 0; loop_i < 128; loop_i++) {
			if ((loop_i % 16) == 0) {
				count += sprintf(buf + count, "LineStart:");
			}

			count += sprintf(buf + count, "%4d", diag_coor[loop_i]);
			if ((loop_i % 16) == 15) {
				count += sprintf(buf + count, "\n");
			}
		}
	}

	return count;
}

static ssize_t himax_me372cl_chip_proc_diag_write(struct file *filp,
						  const char __user *buf,
						  unsigned long len, void *data)
{
	const uint8_t command_ec_128_raw_flag = 0x01;
	const uint8_t command_ec_24_normal_flag = 0x00;

	uint8_t command_ec_128_raw_baseline_flag = 0x02;
	uint8_t command_ec_128_raw_bank_flag = 0x03;

	uint8_t command_91h[2] = { 0x91, 0x00 };
	uint8_t command_82h[1] = { 0x82 };
	uint8_t command_F3h[2] = { 0xF3, 0x00 };
	uint8_t command_83h[1] = { 0x83 };
	uint8_t receive[1];

	if (IC_TYPE != HX_85XX_D_SERIES_PWON) {
		command_ec_128_raw_baseline_flag =
		    0x02 | command_ec_128_raw_flag;
	} else {
		command_ec_128_raw_baseline_flag = 0x02;
		command_ec_128_raw_bank_flag = 0x03;
	}

	if (buf[0] == '1') {	/*IIR */
		command_91h[1] = command_ec_128_raw_baseline_flag;	/*A:0x03 , D:0x02 */
		i2c_himax_write(himax_data->client, command_91h[0],
				&command_91h[1], 1, DEFAULT_RETRY_CNT);
		diag_command = buf[0] - '0';
		printk(KERN_ERR "[Himax_ME372CL]: diag_command=0x%x\n",
		       diag_command);
	} else if (buf[0] == '2') {	/*DC */
		command_91h[1] = command_ec_128_raw_flag;	/*0x01 */
		i2c_himax_write(himax_data->client, command_91h[0],
				&command_91h[1], 1, DEFAULT_RETRY_CNT);
		diag_command = buf[0] - '0';
		printk(KERN_ERR "[Himax_ME372CL]: diag_command=0x%x\n",
		       diag_command);
	} else if (buf[0] == '3') {	/*BANK */
		if (IC_TYPE != HX_85XX_D_SERIES_PWON) {
			i2c_himax_write(himax_data->client, command_82h[0],
					&command_82h[0], 0, DEFAULT_RETRY_CNT);
			msleep(50);

			i2c_himax_read(himax_data->client, command_F3h[0],
				       receive, 1, DEFAULT_RETRY_CNT);
			command_F3h[1] = (receive[0] | 0x80);
			i2c_himax_write(himax_data->client, command_F3h[0],
					&command_F3h[1], 1, DEFAULT_RETRY_CNT);

			command_91h[1] = command_ec_128_raw_baseline_flag;
			i2c_himax_write(himax_data->client, command_91h[0],
					&command_91h[1], 1, DEFAULT_RETRY_CNT);

			i2c_himax_write(himax_data->client, command_83h[0],
					&command_83h[0], 0, DEFAULT_RETRY_CNT);
			msleep(50);
		} else {
			command_91h[1] = command_ec_128_raw_bank_flag;	/*0x03 */
			i2c_himax_write(himax_data->client, command_91h[0],
					&command_91h[1], 1, DEFAULT_RETRY_CNT);
		}
		diag_command = buf[0] - '0';
		printk(KERN_ERR "[Himax_ME372CL]: diag_command=0x%x\n",
		       diag_command);
	} else if (buf[0] == '7') {
		diag_command = buf[0] - '0';
	}
	/*coordinate dump start */
	else if (buf[0] == '8') {
		diag_command = buf[0] - '0';

		coordinate_fn =
		    filp_open(DIAG_COORDINATE_FILE,
			      O_CREAT | O_WRONLY | O_APPEND | O_TRUNC, 0666);
		if (IS_ERR(coordinate_fn)) {
			printk(KERN_INFO
			       "[Himax_ME372CL] %s: coordinate_dump_file_create error\n",
			       __func__);
			coordinate_dump_enable = 0;
			filp_close(coordinate_fn, NULL);
		}
		coordinate_dump_enable = 1;
	} else if (buf[0] == '9') {
		coordinate_dump_enable = 0;
		diag_command = buf[0] - '0';

		if (!IS_ERR(coordinate_fn)) {
			filp_close(coordinate_fn, NULL);
		}
	}
	/*coordinate dump end */
	else {
		if (IC_TYPE != HX_85XX_D_SERIES_PWON) {
			i2c_himax_write(himax_data->client, command_82h[0],
					&command_82h[0], 0, DEFAULT_RETRY_CNT);
			msleep(50);
			command_91h[1] = command_ec_24_normal_flag;
			i2c_himax_write(himax_data->client, command_91h[0],
					&command_91h[1], 1, DEFAULT_RETRY_CNT);
			i2c_himax_read(himax_data->client, command_F3h[0],
				       receive, 1, DEFAULT_RETRY_CNT);
			command_F3h[1] = (receive[0] & 0x7F);
			i2c_himax_write(himax_data->client, command_F3h[0],
					&command_F3h[1], 1, DEFAULT_RETRY_CNT);
			i2c_himax_write(himax_data->client, command_83h[0],
					&command_83h[0], 0, DEFAULT_RETRY_CNT);
		} else {
			command_91h[1] = command_ec_24_normal_flag;
			i2c_himax_write(himax_data->client, command_91h[0],
					&command_91h[1], 1, DEFAULT_RETRY_CNT);
		}
		diag_command = 0;
		printk(KERN_ERR "[Himax_ME372CL]: diag_command=0x%x\n",
		       diag_command);
	}

	return len;
}

static const struct file_operations diag_fops = {
	.owner = THIS_MODULE,
	.read = himax_me372cl_chip_proc_diag_read,
	.write = himax_me372cl_chip_proc_diag_write,
};

static void himax_me372cl_chip_create_proc_diag_file(void)
{
	himax_me372cl_proc_diag_file =
	    proc_create(HIMAX_ME372CL_PROC_DIAG_FILE, 0666, NULL, &diag_fops);
	if (himax_me372cl_proc_diag_file) {
	} else {
		printk(KERN_ERR
		       "[Himax_ME372CL] %s: proc diag file create failed!\n",
		       __func__);
	}
}

static void himax_me372cl_chip_remove_proc_diag_file(void)
{
	extern struct proc_dir_entry proc_root;
	printk(KERN_ERR "[Himax_ME372CL]: %s \n", __func__);
	remove_proc_entry(HIMAX_ME372CL_PROC_DIAG_FILE, &proc_root);
}

static ssize_t himax_me372cl_chip_proc_register_read(char *buf, char **start,
						     off_t off, int count,
						     int *eof, void *data)
{
	int ret = 0;
	int base = 0;
	uint16_t loop_i, loop_j;
	uint8_t bufdata[128];
	uint8_t outData[5];

	memset(outData, 0x00, sizeof(outData));
	memset(bufdata, 0x00, sizeof(bufdata));

	printk(KERN_INFO "[Himax_ME372CL]: multi_register_command = %d \n",
	       multi_register_command);

	if (multi_register_command == 1) {
		base = 0;

		for (loop_i = 0; loop_i < 6; loop_i++) {
			if (multi_register[loop_i] != 0x00) {
				if (multi_cfg_bank[loop_i] == 1) {	/*config bank register */
					outData[0] = 0x15;
					i2c_himax_write(himax_data->client,
							0xE1, &outData[0], 1,
							DEFAULT_RETRY_CNT);
					msleep(10);

					outData[0] = 0x00;
					outData[1] = multi_register[loop_i];
					i2c_himax_write(himax_data->client,
							0xD8, &outData[0], 2,
							DEFAULT_RETRY_CNT);
					msleep(10);

					i2c_himax_read(himax_data->client, 0x5A,
						       bufdata, 128,
						       DEFAULT_RETRY_CNT);

					outData[0] = 0x00;
					i2c_himax_write(himax_data->client,
							0xE1, &outData[0], 1,
							DEFAULT_RETRY_CNT);

					for (loop_j = 0; loop_j < 128; loop_j++) {
						multi_value[base++] =
						    bufdata[loop_j];
					}
				} else {	/*normal register */

					i2c_himax_read(himax_data->client,
						       multi_register[loop_i],
						       bufdata, 128,
						       DEFAULT_RETRY_CNT);

					for (loop_j = 0; loop_j < 128; loop_j++) {
						multi_value[base++] =
						    bufdata[loop_j];
					}
				}
			}
		}

		base = 0;
		for (loop_i = 0; loop_i < 6; loop_i++) {
			if (multi_register[loop_i] != 0x00) {
				if (multi_cfg_bank[loop_i] == 1) {
					ret +=
					    sprintf(buf + ret,
						    "Register: FE(%x)\n",
						    multi_register[loop_i]);
				} else {
					ret +=
					    sprintf(buf + ret, "Register: %x\n",
						    multi_register[loop_i]);
				}

				for (loop_j = 0; loop_j < 128; loop_j++) {
					ret +=
					    sprintf(buf + ret, "0x%2.2X ",
						    multi_value[base++]);
					if ((loop_j % 16) == 15) {
						ret += sprintf(buf + ret, "\n");
					}
				}
			}
		}
		return ret;
	}

	if (config_bank_reg) {
		printk(KERN_INFO
		       "[Himax_ME372CL] %s: register_command = FE(%x)\n",
		       __func__, register_command);

		/*Config bank register read flow. */
		outData[0] = 0x15;
		i2c_himax_write(himax_data->client, 0xE1, &outData[0], 1,
				DEFAULT_RETRY_CNT);

		msleep(10);

		outData[0] = 0x00;
		outData[1] = register_command;
		i2c_himax_write(himax_data->client, 0xD8, &outData[0], 2,
				DEFAULT_RETRY_CNT);

		msleep(10);

		i2c_himax_read(himax_data->client, 0x5A, bufdata, 128,
			       DEFAULT_RETRY_CNT);

		msleep(10);

		outData[0] = 0x00;
		i2c_himax_write(himax_data->client, 0xE1, &outData[0], 1,
				DEFAULT_RETRY_CNT);
	} else {
		if (i2c_himax_read
		    (himax_data->client, register_command, bufdata, 128,
		     DEFAULT_RETRY_CNT) < 0) {
			return ret;
		}
	}

	if (config_bank_reg) {
		ret += sprintf(buf, "command: FE(%x)\n", register_command);
	} else {
		ret += sprintf(buf, "command: %x\n", register_command);
	}

	for (loop_i = 0; loop_i < 128; loop_i++) {
		ret += sprintf(buf + ret, "0x%2.2X ", bufdata[loop_i]);
		if ((loop_i % 16) == 15) {
			ret += sprintf(buf + ret, "\n");
		}
	}
	ret += sprintf(buf + ret, "\n");
	return ret;
}

static ssize_t himax_me372cl_chip_proc_register_write(struct file *filp,
						      const char *buf,
						      unsigned long len,
						      void *data)
{
	char buf_tmp[6], length = 0;
	unsigned long result = 0;
	uint8_t loop_i = 0;
	uint16_t base = 5;
	uint8_t write_da[128];
	uint8_t outData[5];

	memset(buf_tmp, 0x0, sizeof(buf_tmp));
	memset(write_da, 0x0, sizeof(write_da));
	memset(outData, 0x0, sizeof(outData));

	printk("[Himax_ME372CL]: %s \n", buf);

	if (buf[0] == 'm' && buf[1] == 'r' && buf[2] == ':') {
		memset(multi_register, 0x00, sizeof(multi_register));
		memset(multi_cfg_bank, 0x00, sizeof(multi_cfg_bank));
		memset(multi_value, 0x00, sizeof(multi_value));

		printk("[Himax_ME372CL]: multi register enter\n");

		multi_register_command = 1;

		base = 2;
		loop_i = 0;

		while (true) {
			if (buf[base] == '\n') {
				break;
			}

			if (loop_i >= 6) {
				break;
			}

			if (buf[base] == ':' && buf[base + 1] == 'x'
			    && buf[base + 2] == 'F' && buf[base + 3] == 'E'
			    && buf[base + 4] != ':') {
				memcpy(buf_tmp, buf + base + 4, 2);
				if (!strict_strtoul(buf_tmp, 16, &result)) {
					multi_register[loop_i] = result;
					multi_cfg_bank[loop_i++] = 1;
				}
				base += 6;
			} else {
				memcpy(buf_tmp, buf + base + 2, 2);
				if (!strict_strtoul(buf_tmp, 16, &result)) {
					multi_register[loop_i] = result;
					multi_cfg_bank[loop_i++] = 0;
				}
				base += 4;
			}
		}

		printk(KERN_INFO "========================== \n");
		for (loop_i = 0; loop_i < 6; loop_i++) {
			printk(KERN_INFO "%d,%d:", multi_register[loop_i],
			       multi_cfg_bank[loop_i]);
		}
		printk(KERN_INFO "\n");
	} else if ((buf[0] == 'r' || buf[0] == 'w') && buf[1] == ':') {
		multi_register_command = 0;

		if (buf[2] == 'x') {
			if (buf[3] == 'F' && buf[4] == 'E') {	/*Config bank register */
				config_bank_reg = true;

				memcpy(buf_tmp, buf + 5, 2);
				if (!strict_strtoul(buf_tmp, 16, &result)) {
					register_command = result;
				}
				base = 7;

				printk(KERN_INFO "CMD: FE(%x)\n",
				       register_command);
			} else {
				config_bank_reg = false;

				memcpy(buf_tmp, buf + 3, 2);
				if (!strict_strtoul(buf_tmp, 16, &result)) {
					register_command = result;
				}
				base = 5;
				printk(KERN_INFO "CMD: %x\n", register_command);
			}

			for (loop_i = 0; loop_i < 128; loop_i++) {
				if (buf[base] == '\n') {
					if (buf[0] == 'w') {
						if (config_bank_reg) {
							outData[0] = 0x15;
							i2c_himax_write
							    (himax_data->client,
							     0xE1, &outData[0],
							     1,
							     DEFAULT_RETRY_CNT);

							msleep(10);

							outData[0] = 0x00;
							outData[1] =
							    register_command;
							i2c_himax_write
							    (himax_data->client,
							     0xD8, &outData[0],
							     2,
							     DEFAULT_RETRY_CNT);

							msleep(10);
							i2c_himax_write
							    (himax_data->client,
							     0x40, &write_da[0],
							     length,
							     DEFAULT_RETRY_CNT);

							msleep(10);

							outData[0] = 0x00;
							i2c_himax_write
							    (himax_data->client,
							     0xE1, &outData[0],
							     1,
							     DEFAULT_RETRY_CNT);

							printk(KERN_INFO
							       "CMD: FE(%x), %x, %d\n",
							       register_command,
							       write_da[0],
							       length);
						} else {
							i2c_himax_write
							    (himax_data->client,
							     register_command,
							     &write_da[0],
							     length,
							     DEFAULT_RETRY_CNT);
							printk(KERN_INFO
							       "CMD: %x, %x, %d\n",
							       register_command,
							       write_da[0],
							       length);
						}
					}

					printk(KERN_INFO "\n");
					return len;
				}
				if (buf[base + 1] == 'x') {
					buf_tmp[4] = '\n';
					buf_tmp[5] = '\0';
					memcpy(buf_tmp, buf + base + 2, 2);
					if (!strict_strtoul
					    (buf_tmp, 16, &result)) {
						write_da[loop_i] = result;
					}
					length++;
				}
				base += 4;
			}
		}
	}
	return len;
}

static const struct file_operations register_fops = {
	.owner = THIS_MODULE,
	.read = himax_me372cl_chip_proc_register_read,
	.write = himax_me372cl_chip_proc_register_write,
};

static void himax_me372cl_chip_create_proc_register(void)
{
	himax_me372cl_proc_register =
	    proc_create(HIMAX_ME372CL_PROC_REGISTER, 0666, NULL,
			&register_fops);
	if (himax_me372cl_proc_register) {
	} else {
		printk(KERN_ERR
		       "[Himax_ME372CL] %s: proc config file create failed!\n",
		       __func__);
	}
}

static void himax_me372cl_chip_remove_proc_register(void)
{
	extern struct proc_dir_entry proc_root;
	printk(KERN_ERR "[Himax_ME372CL]:himax_chip_remove_proc_register\n");
	remove_proc_entry(HIMAX_ME372CL_PROC_REGISTER, &proc_root);
}

static ssize_t himax_me372cl_chip_proc_debug_level_show(char *buf, char **start,
							off_t off, int count1,
							int *eof, void *data)
{
	size_t count = 0;
	int i = 0;

	if (debug_level_cmd == 't') {
		if (fw_update_complete) {
			count += sprintf(buf, "FW Update Complete \n");
		} else {
			count += sprintf(buf, "FW Update Fail \n");
		}
	} else if (debug_level_cmd == 'i') {
		if (irq_enable) {
			count += sprintf(buf, "IRQ is enable\n");
		} else {
			count += sprintf(buf, "IRQ is disable\n");
		}
	} else if (debug_level_cmd == 'h') {
		if (handshaking_result == 0) {
			count +=
			    sprintf(buf,
				    "Handshaking Result = %d (MCU Running)\n",
				    handshaking_result);
		} else if (handshaking_result == 1) {
			count +=
			    sprintf(buf, "Handshaking Result = %d (MCU Stop)\n",
				    handshaking_result);
		} else if (handshaking_result == 2) {
			count +=
			    sprintf(buf,
				    "Handshaking Result = %d (I2C Error)\n",
				    handshaking_result);
		} else {
			count += sprintf(buf, "Handshaking Result = error \n");
		}
	} else if (debug_level_cmd == 'v') {
		count += sprintf(buf + count, "FW_VER_MAJ_buff = ");
		count += sprintf(buf + count, "0x%2.2X \n", FW_VER_MAJ_buff[0]);

		count += sprintf(buf + count, "FW_VER_MIN_buff = ");
		count += sprintf(buf + count, "0x%2.2X \n", FW_VER_MIN_buff[0]);

		count += sprintf(buf + count, "CFG_VER_MAJ_buff = ");
		for (i = 0; i < 12; i++) {
			count +=
			    sprintf(buf + count, "0x%2.2X ",
				    CFG_VER_MAJ_buff[i]);
		}
		count += sprintf(buf + count, "\n");

		count += sprintf(buf + count, "CFG_VER_MIN_buff = ");
		for (i = 0; i < 12; i++) {
			count +=
			    sprintf(buf + count, "0x%2.2X ",
				    CFG_VER_MIN_buff[i]);
		}
		count += sprintf(buf + count, "\n");
	} else if (debug_level_cmd == 'd') {
		count += sprintf(buf + count, "Himax Touch IC Information :\n");
		if (IC_TYPE == HX_85XX_A_SERIES_PWON) {
			count += sprintf(buf + count, "IC Type : A\n");
		} else if (IC_TYPE == HX_85XX_B_SERIES_PWON) {
			count += sprintf(buf + count, "IC Type : B\n");
		} else if (IC_TYPE == HX_85XX_C_SERIES_PWON) {
			count += sprintf(buf + count, "IC Type : C\n");
		} else if (IC_TYPE == HX_85XX_D_SERIES_PWON) {
			count += sprintf(buf + count, "IC Type : D\n");
		} else {
			count += sprintf(buf + count, "IC Type error.\n");
		}

		if (IC_CHECKSUM == HX_TP_BIN_CHECKSUM_SW) {
			count += sprintf(buf + count, "IC Checksum : SW\n");
		} else if (IC_CHECKSUM == HX_TP_BIN_CHECKSUM_HW) {
			count += sprintf(buf + count, "IC Checksum : HW\n");
		} else if (IC_CHECKSUM == HX_TP_BIN_CHECKSUM_CRC) {
			count += sprintf(buf + count, "IC Checksum : CRC\n");
		} else {
			count += sprintf(buf + count, "IC Checksum error.\n");
		}

		if (HX_INT_IS_EDGE) {
			count +=
			    sprintf(buf + count, "Interrupt : EDGE TIRGGER\n");
		} else {
			count +=
			    sprintf(buf + count, "Interrupt : LEVEL TRIGGER\n");
		}

		count += sprintf(buf + count, "RX Num : %d\n", HX_RX_NUM);
		count += sprintf(buf + count, "TX Num : %d\n", HX_TX_NUM);
		count += sprintf(buf + count, "BT Num : %d\n", HX_BT_NUM);
		count += sprintf(buf + count, "X Resolution : %d\n", HX_X_RES);
		count += sprintf(buf + count, "Y Resolution : %d\n", HX_Y_RES);
		count += sprintf(buf + count, "Max Point : %d\n", HX_MAX_PT);
	} else {
		count += sprintf(buf, "%d\n", debug_log_level);
	}
	return count;
}

static ssize_t himax_me372cl_chip_proc_debug_level_dump(struct file *filp1,
							const char *buf,
							unsigned long len,
							void *data)
{
	struct file *filp = NULL;
	mm_segment_t oldfs;
	int result = 0;
	char fileName[128];
	int err = 1;

	printk(KERN_ERR "[Himax] %s ++ \n", __func__);

	if (buf[0] >= '0' && buf[0] <= '9' && buf[1] == '\n') {
		debug_log_level = buf[0] - '0';
		return len;
	}

	if (buf[0] == 'i') {	/*irq */
		debug_level_cmd = buf[0];

		if (buf[2] == '1') {	/*enable irq       */
			enable_irq(himax_data->irq);
			irq_enable = true;
		} else if (buf[2] == '0') {	/*disable irq */
			disable_irq(himax_data->irq);
			irq_enable = false;
		} else {
			printk(KERN_ERR
			       "[Himax] %s: debug_level command = 'i' , parameter error.\n",
			       __func__);
		}
		return len;
	}

	if (buf[0] == 'h') {	/*handshaking */
		debug_level_cmd = buf[0];

		disable_irq(himax_data->irq);

		handshaking_result = himax_hang_shaking();	/*0:Running, 1:Stop, 2:I2C Fail  */

		enable_irq(himax_data->irq);

		return len;
	}

	if (buf[0] == 'v') {	/*firmware version */
		debug_level_cmd = buf[0];
		himax_read_fw_ver(true);
		return len;
	}

	if (buf[0] == 'd') {	/*test */
		debug_level_cmd = buf[0];
		return len;
	}

	if (buf[0] == 't') {

		/*----[ENABLE_CHIP_STATUS_MONITOR]------------------------start	 */
#ifdef ENABLE_CHIP_STATUS_MONITOR
		cancel_delayed_work_sync(&himax_data->himax_chip_monitor);
#endif
		/*----[ENABLE_CHIP_STATUS_MONITOR]--------------------------end	 */

		/*Wakelock Protect start */
		wake_lock(&himax_data->wake_lock);
		/*Wakelock Protect end */

		/*Mutexlock Protect Start */
		mutex_lock(&himax_data->mutex_lock);
		/*Mutexlock Protect End */

		debug_level_cmd = buf[0];
		fw_update_complete = false;

		memset(fileName, 0, 128);
		/* parse the file name */
		snprintf(fileName, len - 2, "%s", &buf[2]);
		printk(KERN_INFO "[Himax] %s: upgrade from file(%s) start!\n",
		       __func__, fileName);
		/* open file */
		filp = filp_open(fileName, O_RDONLY, 0);
		if (IS_ERR(filp)) {
			printk(KERN_ERR
			       "[Himax] %s: open firmware file failed\n",
			       __func__);
			goto firmware_upgrade_done;
			/*return len; */
		}
		oldfs = get_fs();
		set_fs(get_ds());

		/* read the latest firmware binary file */
		result =
		    filp->f_op->read(filp, upgrade_fw, sizeof(upgrade_fw),
				     &filp->f_pos);
		if (result < 0) {
			printk(KERN_ERR
			       "[Himax] %s: read firmware file failed\n",
			       __func__);
			goto firmware_upgrade_done;
			/*return len; */
		}

		set_fs(oldfs);
		filp_close(filp, NULL);

		printk(KERN_INFO
		       "[Himax] %s: upgrade start,len %d: %02X, %02X, %02X, %02X.\n",
		       __func__, result, upgrade_fw[0], upgrade_fw[1],
		       upgrade_fw[2], upgrade_fw[3]);
		printk(KERN_INFO "[Himax] %s: upgrade version: %02X %02X.\n",
		       __func__, upgrade_fw[0x0085], upgrade_fw[0x0086]);
		printk(KERN_INFO
		       "[Himax] %s: last 4 byte, %02X, %02X, %02X, %02X.\n",
		       __func__, upgrade_fw[0x7FFC], upgrade_fw[0x7FFD],
		       upgrade_fw[0x7FFE], upgrade_fw[0x7FFF]);

		if (result > 0) {
			/* start to upgrade */
			disable_irq(himax_data->irq);
			if (fts_ctpm_fw_upgrade_with_sys_fs(upgrade_fw, result)
			    == 0) {
				printk(KERN_INFO
				       "[Himax] %s: TP upgrade error, line: %d\n",
				       __func__, __LINE__);
				fw_update_complete = false;
			} else {
				printk(KERN_INFO
				       "[Himax] %s: TP upgrade OK, line: %d\n",
				       __func__, __LINE__);
				fw_update_complete = true;
			}
			enable_irq(himax_data->irq);
			goto firmware_upgrade_done;
			/*return len;                     */
		}
	}

firmware_upgrade_done:

	mutex_unlock(&himax_data->mutex_lock);

	wake_unlock(&himax_data->wake_lock);

	/*----[ENABLE_CHIP_RESET_MACHINE]------------------start	 */
	queue_delayed_work(himax_data->himax_wq,
			   &himax_data->himax_chip_reset_work, 0);
	/*----[ENABLE_CHIP_RESET_MACHINE]------------------end */

	/*todo himax_chip->tp_firmware_upgrade_proceed = 0; */
	/*todo himax_chip->suspend_state = 0; */
	/*todo enable_irq(himax_chip->irq); */

	/*----[ENABLE_CHIP_STATUS_MONITOR]-----------------start	 */
#ifdef ENABLE_CHIP_STATUS_MONITOR
	queue_delayed_work(himax_data->himax_wq,
			   &himax_data->himax_chip_monitor, 10 * HZ);
#endif
	/*----[ENABLE_CHIP_STATUS_MONITOR]----------------end	 */

	return len;
}

static const struct file_operations debug_level_fops = {
	.owner = THIS_MODULE,
	.read = himax_me372cl_chip_proc_debug_level_show,
	.write = himax_me372cl_chip_proc_debug_level_dump,
};

static void himax_me372cl_chip_create_proc_debug_level(void)
{
	himax_me372cl_proc_debug_level =
	    proc_create(HIMAX_ME372CL_PROC_DEBUG_LEVEL, 0666, NULL,
			&debug_level_fops);
	if (himax_me372cl_proc_debug_level) {
	} else {
		printk(KERN_ERR
		       "[Himax_ME372CL] %s: proc debug level create failed!\n",
		       __func__);
	}
}

static void himax_me372cl_chip_remove_proc_debug_level(void)
{
	extern struct proc_dir_entry proc_root;
	printk(KERN_ERR "[Himax_ME372CL]:himax_chip_remove_proc_debug_level\n");
	remove_proc_entry(HIMAX_ME372CL_PROC_DEBUG_LEVEL, &proc_root);
}

/*--------[proc_function]-----------end */

/*==========================================
//
//	Segment : Himax SYS Debug Function
//
//========================================== */

/*----[HX_TP_SYS_REGISTER]--------------------start */
#ifdef HX_TP_SYS_REGISTER
static ssize_t himax_register_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int base = 0;
	uint16_t loop_i, loop_j;
	uint8_t data[128];
	uint8_t outData[5];

	memset(outData, 0x00, sizeof(outData));
	memset(data, 0x00, sizeof(data));

	printk(KERN_INFO "[Himax]: multi_register_command = %d \n",
	       multi_register_command);

	if (multi_register_command == 1) {
		base = 0;

		for (loop_i = 0; loop_i < 6; loop_i++) {
			if (multi_register[loop_i] != 0x00) {
				if (multi_cfg_bank[loop_i] == 1) {	/*config bank register */
					outData[0] = 0x15;
					i2c_himax_write(himax_data->client,
							0xE1, &outData[0], 1,
							DEFAULT_RETRY_CNT);
					msleep(10);

					outData[0] = 0x00;
					outData[1] = multi_register[loop_i];
					i2c_himax_write(himax_data->client,
							0xD8, &outData[0], 2,
							DEFAULT_RETRY_CNT);
					msleep(10);

					i2c_himax_read(himax_data->client, 0x5A,
						       data, 128,
						       DEFAULT_RETRY_CNT);

					outData[0] = 0x00;
					i2c_himax_write(himax_data->client,
							0xE1, &outData[0], 1,
							DEFAULT_RETRY_CNT);

					for (loop_j = 0; loop_j < 128; loop_j++) {
						multi_value[base++] =
						    data[loop_j];
					}
				} else {	/*normal register */

					i2c_himax_read(himax_data->client,
						       multi_register[loop_i],
						       data, 128,
						       DEFAULT_RETRY_CNT);

					for (loop_j = 0; loop_j < 128; loop_j++) {
						multi_value[base++] =
						    data[loop_j];
					}
				}
			}
		}

		base = 0;
		for (loop_i = 0; loop_i < 6; loop_i++) {
			if (multi_register[loop_i] != 0x00) {
				if (multi_cfg_bank[loop_i] == 1) {
					ret +=
					    sprintf(buf + ret,
						    "Register: FE(%x)\n",
						    multi_register[loop_i]);
				} else {
					ret +=
					    sprintf(buf + ret, "Register: %x\n",
						    multi_register[loop_i]);
				}

				for (loop_j = 0; loop_j < 128; loop_j++) {
					ret +=
					    sprintf(buf + ret, "0x%2.2X ",
						    multi_value[base++]);
					if ((loop_j % 16) == 15) {
						ret += sprintf(buf + ret, "\n");
					}
				}
			}
		}
		return ret;
	}

	if (config_bank_reg) {
		printk(KERN_INFO "[Himax] %s: register_command = FE(%x)\n",
		       __func__, register_command);

		/*Config bank register read flow. */
		outData[0] = 0x15;
		i2c_himax_write(himax_data->client, 0xE1, &outData[0], 1,
				DEFAULT_RETRY_CNT);

		msleep(10);

		outData[0] = 0x00;
		outData[1] = register_command;
		i2c_himax_write(himax_data->client, 0xD8, &outData[0], 2,
				DEFAULT_RETRY_CNT);

		msleep(10);

		i2c_himax_read(himax_data->client, 0x5A, data, 128,
			       DEFAULT_RETRY_CNT);

		msleep(10);

		outData[0] = 0x00;
		i2c_himax_write(himax_data->client, 0xE1, &outData[0], 1,
				DEFAULT_RETRY_CNT);
	} else {
		if (i2c_himax_read
		    (himax_data->client, register_command, data, 128,
		     DEFAULT_RETRY_CNT) < 0) {
			return ret;
		}
	}

	if (config_bank_reg) {
		ret += sprintf(buf, "command: FE(%x)\n", register_command);
	} else {
		ret += sprintf(buf, "command: %x\n", register_command);
	}

	for (loop_i = 0; loop_i < 128; loop_i++) {
		ret += sprintf(buf + ret, "0x%2.2X ", data[loop_i]);
		if ((loop_i % 16) == 15) {
			ret += sprintf(buf + ret, "\n");
		}
	}
	ret += sprintf(buf + ret, "\n");
	return ret;
}

static ssize_t himax_register_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	char buf_tmp[6], length = 0;
	unsigned long result = 0;
	uint8_t loop_i = 0;
	uint16_t base = 5;
	uint8_t write_da[128];
	uint8_t outData[5];

	memset(buf_tmp, 0x0, sizeof(buf_tmp));
	memset(write_da, 0x0, sizeof(write_da));
	memset(outData, 0x0, sizeof(outData));

	printk("[Himax]: %s \n", buf);

	if (buf[0] == 'm' && buf[1] == 'r' && buf[2] == ':') {
		memset(multi_register, 0x00, sizeof(multi_register));
		memset(multi_cfg_bank, 0x00, sizeof(multi_cfg_bank));
		memset(multi_value, 0x00, sizeof(multi_value));

		printk("[Himax]: multi register enter\n");

		multi_register_command = 1;

		base = 2;
		loop_i = 0;

		while (true) {
			if (buf[base] == '\n') {
				break;
			}

			if (loop_i >= 6) {
				break;
			}

			if (buf[base] == ':' && buf[base + 1] == 'x'
			    && buf[base + 2] == 'F' && buf[base + 3] == 'E'
			    && buf[base + 4] != ':') {
				memcpy(buf_tmp, buf + base + 4, 2);
				if (!strict_strtoul(buf_tmp, 16, &result)) {
					multi_register[loop_i] = result;
					multi_cfg_bank[loop_i++] = 1;
				}
				base += 6;
			} else {
				memcpy(buf_tmp, buf + base + 2, 2);
				if (!strict_strtoul(buf_tmp, 16, &result)) {
					multi_register[loop_i] = result;
					multi_cfg_bank[loop_i++] = 0;
				}
				base += 4;
			}
		}

		printk(KERN_INFO "========================== \n");
		for (loop_i = 0; loop_i < 6; loop_i++) {
			printk(KERN_INFO "%d,%d:", multi_register[loop_i],
			       multi_cfg_bank[loop_i]);
		}
		printk(KERN_INFO "\n");
	} else if ((buf[0] == 'r' || buf[0] == 'w') && buf[1] == ':') {
		multi_register_command = 0;

		if (buf[2] == 'x') {
			if (buf[3] == 'F' && buf[4] == 'E') {	/*Config bank register */
				config_bank_reg = true;

				memcpy(buf_tmp, buf + 5, 2);
				if (!strict_strtoul(buf_tmp, 16, &result)) {
					register_command = result;
				}
				base = 7;

				printk(KERN_INFO "CMD: FE(%x)\n",
				       register_command);
			} else {
				config_bank_reg = false;

				memcpy(buf_tmp, buf + 3, 2);
				if (!strict_strtoul(buf_tmp, 16, &result)) {
					register_command = result;
				}
				base = 5;
				printk(KERN_INFO "CMD: %x\n", register_command);
			}

			for (loop_i = 0; loop_i < 128; loop_i++) {
				if (buf[base] == '\n') {
					if (buf[0] == 'w') {
						if (config_bank_reg) {
							outData[0] = 0x15;
							i2c_himax_write
							    (himax_data->client,
							     0xE1, &outData[0],
							     1,
							     DEFAULT_RETRY_CNT);

							msleep(10);

							outData[0] = 0x00;
							outData[1] =
							    register_command;
							i2c_himax_write
							    (himax_data->client,
							     0xD8, &outData[0],
							     2,
							     DEFAULT_RETRY_CNT);

							msleep(10);
							i2c_himax_write
							    (himax_data->client,
							     0x40, &write_da[0],
							     length,
							     DEFAULT_RETRY_CNT);

							msleep(10);

							outData[0] = 0x00;
							i2c_himax_write
							    (himax_data->client,
							     0xE1, &outData[0],
							     1,
							     DEFAULT_RETRY_CNT);

							printk(KERN_INFO
							       "CMD: FE(%x), %x, %d\n",
							       register_command,
							       write_da[0],
							       length);
						} else {
							i2c_himax_write
							    (himax_data->client,
							     register_command,
							     &write_da[0],
							     length,
							     DEFAULT_RETRY_CNT);
							printk(KERN_INFO
							       "CMD: %x, %x, %d\n",
							       register_command,
							       write_da[0],
							       length);
						}
					}

					printk(KERN_INFO "\n");
					return count;
				}
				if (buf[base + 1] == 'x') {
					buf_tmp[4] = '\n';
					buf_tmp[5] = '\0';
					memcpy(buf_tmp, buf + base + 2, 2);
					if (!strict_strtoul
					    (buf_tmp, 16, &result)) {
						write_da[loop_i] = result;
					}
					length++;
				}
				base += 4;
			}
		}
	}
	return count;
}
#endif
/*----[HX_TP_SYS_REGISTER]-------------end */

/*----[HX_TP_SYS_DEBUG_LEVEL]--------start */
#ifdef HX_TP_SYS_DEBUG_LEVEL

static uint8_t getDebugLevel(void)
{
	return debug_log_level;
}

static int fts_ctpm_fw_upgrade_with_sys_fs(unsigned char *fw, int len)
{
	/*      unsigned char* ImageBuffer = fw;//CTPM_FW; */
	int fullFileLength = len;	/*sizeof(CTPM_FW); //Paul Check */
	int i, j;
	uint8_t cmd[5], last_byte, prePage;
	int FileLength;
	uint8_t checksumResult = 0;
	int progress_bar = 35;

	update_touch_progress(progress_bar);

	/*Try 3 Times */
	for (j = 0; j < 3; j++) {
		if (IC_CHECKSUM == HX_TP_BIN_CHECKSUM_CRC) {
			FileLength = fullFileLength;
		} else {
			FileLength = fullFileLength - 2;
		}

#ifdef HX_RST_PIN_FUNC
		himax_hw_reset();
#endif

		if (i2c_himax_write
		    (himax_data->client, 0x81, &cmd[0], 0,
		     DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c access fail!\n",
			       __func__);
			return 0;
		}

		mdelay(120);

		himax_unlock_flash();	/*ok */

		cmd[0] = 0x05;
		cmd[1] = 0x00;
		cmd[2] = 0x02;
		if (i2c_himax_write
		    (himax_data->client, 0x43, &cmd[0], 3,
		     DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c access fail!\n",
			       __func__);
			return 0;
		}

		if (i2c_himax_write
		    (himax_data->client, 0x4F, &cmd[0], 0,
		     DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c access fail!\n",
			       __func__);
			return 0;
		}
		mdelay(50);

		himax_ManualMode(1);
		himax_FlashMode(1);

		FileLength = (FileLength + 3) / 4;

		printk("[Himax] FileLength = %d \n", FileLength);
		for (i = 0, prePage = 0; i < FileLength; i++) {
			if ((i % 1024) == 0) {
				progress_bar = progress_bar + 5;
				update_touch_progress(progress_bar);
			}

			last_byte = 0;
			cmd[0] = i & 0x1F;
			if (cmd[0] == 0x1F || i == FileLength - 1) {
				last_byte = 1;
			}
			cmd[1] = (i >> 5) & 0x1F;
			cmd[2] = (i >> 10) & 0x1F;
			if (i2c_himax_write
			    (himax_data->client, 0x44, &cmd[0], 3,
			     DEFAULT_RETRY_CNT) < 0) {
				printk(KERN_ERR
				       "[Himax] %s: i2c access fail!\n",
				       __func__);
				return 0;
			}

			if (prePage != cmd[1] || i == 0) {
				prePage = cmd[1];
				cmd[0] = 0x01;
				cmd[1] = 0x09;	/*cmd[2] = 0x02; */
				if (i2c_himax_write
				    (himax_data->client, 0x43, &cmd[0], 2,
				     DEFAULT_RETRY_CNT) < 0) {
					printk(KERN_ERR
					       "[Himax] %s: i2c access fail!\n",
					       __func__);
					return 0;
				}

				cmd[0] = 0x01;
				cmd[1] = 0x0D;	/*cmd[2] = 0x02; */
				if (i2c_himax_write
				    (himax_data->client, 0x43, &cmd[0], 2,
				     DEFAULT_RETRY_CNT) < 0) {
					printk(KERN_ERR
					       "[Himax] %s: i2c access fail!\n",
					       __func__);
					return 0;
				}

				cmd[0] = 0x01;
				cmd[1] = 0x09;	/*cmd[2] = 0x02; */
				if (i2c_himax_write
				    (himax_data->client, 0x43, &cmd[0], 2,
				     DEFAULT_RETRY_CNT) < 0) {
					printk(KERN_ERR
					       "[Himax] %s: i2c access fail!\n",
					       __func__);
					return 0;
				}
			}

			memcpy(&cmd[0], &upgrade_fw[4 * i], 4);	/*Paul */
			if (i2c_himax_write
			    (himax_data->client, 0x45, &cmd[0], 4,
			     DEFAULT_RETRY_CNT) < 0) {
				printk(KERN_ERR
				       "[Himax] %s: i2c access fail!\n",
				       __func__);
				return 0;
			}

			cmd[0] = 0x01;
			cmd[1] = 0x0D;	/*cmd[2] = 0x02; */
			if (i2c_himax_write
			    (himax_data->client, 0x43, &cmd[0], 2,
			     DEFAULT_RETRY_CNT) < 0) {
				printk(KERN_ERR
				       "[Himax] %s: i2c access fail!\n",
				       __func__);
				return 0;
			}

			cmd[0] = 0x01;
			cmd[1] = 0x09;	/*cmd[2] = 0x02; */
			if (i2c_himax_write
			    (himax_data->client, 0x43, &cmd[0], 2,
			     DEFAULT_RETRY_CNT) < 0) {
				printk(KERN_ERR
				       "[Himax] %s: i2c access fail!\n",
				       __func__);
				return 0;
			}

			if (last_byte == 1) {
				cmd[0] = 0x01;
				cmd[1] = 0x01;	/*cmd[2] = 0x02; */
				if (i2c_himax_write
				    (himax_data->client, 0x43, &cmd[0], 2,
				     DEFAULT_RETRY_CNT) < 0) {
					printk(KERN_ERR
					       "[Himax] %s: i2c access fail!\n",
					       __func__);
					return 0;
				}

				cmd[0] = 0x01;
				cmd[1] = 0x05;	/*cmd[2] = 0x02; */
				if (i2c_himax_write
				    (himax_data->client, 0x43, &cmd[0], 2,
				     DEFAULT_RETRY_CNT) < 0) {
					printk(KERN_ERR
					       "[Himax] %s: i2c access fail!\n",
					       __func__);
					return 0;
				}

				cmd[0] = 0x01;
				cmd[1] = 0x01;	/*cmd[2] = 0x02; */
				if (i2c_himax_write
				    (himax_data->client, 0x43, &cmd[0], 2,
				     DEFAULT_RETRY_CNT) < 0) {
					printk(KERN_ERR
					       "[Himax] %s: i2c access fail!\n",
					       __func__);
					return 0;
				}

				cmd[0] = 0x01;
				cmd[1] = 0x00;	/*cmd[2] = 0x02; */
				if (i2c_himax_write
				    (himax_data->client, 0x43, &cmd[0], 2,
				     DEFAULT_RETRY_CNT) < 0) {
					printk(KERN_ERR
					       "[Himax] %s: i2c access fail!\n",
					       __func__);
					return 0;
				}

				mdelay(10);
				if (i == (FileLength - 1)) {
					himax_FlashMode(0);
					himax_ManualMode(0);
					checksumResult = himax_calculateChecksum(upgrade_fw, fullFileLength);	/*, address, RST); */
					/*himax_ManualMode(0); */
					himax_lock_flash();

					printk(KERN_ERR
					       "[Himax] %s: checksumResult = %d.\n",
					       __func__, checksumResult);

					if (checksumResult) {	/*Success */
						return 1;
					} else {	/*Fail */

						printk(KERN_ERR
						       "[Himax] %s: checksum Result fail!\n",
						       __func__);
						return 0;
					}
				}
			}
		}
	}
	printk("[Himax] %s: fimware retry fail! \n", __func__);
	return 0;
}

static ssize_t himax_debug_level_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	size_t count = 0;
	int i = 0;

	if (debug_level_cmd == 't') {
		if (fw_update_complete) {
			count += sprintf(buf, "FW Update Complete \n");
		} else {
			count += sprintf(buf, "FW Update Fail \n");
		}
	} else if (debug_level_cmd == 'i') {
		if (irq_enable) {
			count += sprintf(buf, "IRQ is enable\n");
		} else {
			count += sprintf(buf, "IRQ is disable\n");
		}
	} else if (debug_level_cmd == 'h') {
		if (handshaking_result == 0) {
			count +=
			    sprintf(buf,
				    "Handshaking Result = %d (MCU Running)\n",
				    handshaking_result);
		} else if (handshaking_result == 1) {
			count +=
			    sprintf(buf, "Handshaking Result = %d (MCU Stop)\n",
				    handshaking_result);
		} else if (handshaking_result == 2) {
			count +=
			    sprintf(buf,
				    "Handshaking Result = %d (I2C Error)\n",
				    handshaking_result);
		} else {
			count += sprintf(buf, "Handshaking Result = error \n");
		}
	} else if (debug_level_cmd == 'v') {
		count += sprintf(buf + count, "FW_VER_MAJ_buff = ");
		count += sprintf(buf + count, "0x%2.2X \n", FW_VER_MAJ_buff[0]);

		count += sprintf(buf + count, "FW_VER_MIN_buff = ");
		count += sprintf(buf + count, "0x%2.2X \n", FW_VER_MIN_buff[0]);

		count += sprintf(buf + count, "CFG_VER_MAJ_buff = ");
		for (i = 0; i < 12; i++) {
			count +=
			    sprintf(buf + count, "0x%2.2X ",
				    CFG_VER_MAJ_buff[i]);
		}
		count += sprintf(buf + count, "\n");

		count += sprintf(buf + count, "CFG_VER_MIN_buff = ");
		for (i = 0; i < 12; i++) {
			count +=
			    sprintf(buf + count, "0x%2.2X ",
				    CFG_VER_MIN_buff[i]);
		}
		count += sprintf(buf + count, "\n");
	} else if (debug_level_cmd == 'd') {
		count += sprintf(buf + count, "Himax Touch IC Information :\n");
		if (IC_TYPE == HX_85XX_A_SERIES_PWON) {
			count += sprintf(buf + count, "IC Type : A\n");
		} else if (IC_TYPE == HX_85XX_B_SERIES_PWON) {
			count += sprintf(buf + count, "IC Type : B\n");
		} else if (IC_TYPE == HX_85XX_C_SERIES_PWON) {
			count += sprintf(buf + count, "IC Type : C\n");
		} else if (IC_TYPE == HX_85XX_D_SERIES_PWON) {
			count += sprintf(buf + count, "IC Type : D\n");
		} else {
			count += sprintf(buf + count, "IC Type error.\n");
		}

		if (IC_CHECKSUM == HX_TP_BIN_CHECKSUM_SW) {
			count += sprintf(buf + count, "IC Checksum : SW\n");
		} else if (IC_CHECKSUM == HX_TP_BIN_CHECKSUM_HW) {
			count += sprintf(buf + count, "IC Checksum : HW\n");
		} else if (IC_CHECKSUM == HX_TP_BIN_CHECKSUM_CRC) {
			count += sprintf(buf + count, "IC Checksum : CRC\n");
		} else {
			count += sprintf(buf + count, "IC Checksum error.\n");
		}

		if (HX_INT_IS_EDGE) {
			count +=
			    sprintf(buf + count, "Interrupt : EDGE TIRGGER\n");
		} else {
			count +=
			    sprintf(buf + count, "Interrupt : LEVEL TRIGGER\n");
		}

		count += sprintf(buf + count, "RX Num : %d\n", HX_RX_NUM);
		count += sprintf(buf + count, "TX Num : %d\n", HX_TX_NUM);
		count += sprintf(buf + count, "BT Num : %d\n", HX_BT_NUM);
		count += sprintf(buf + count, "X Resolution : %d\n", HX_X_RES);
		count += sprintf(buf + count, "Y Resolution : %d\n", HX_Y_RES);
		count += sprintf(buf + count, "Max Point : %d\n", HX_MAX_PT);
	} else {
		count += sprintf(buf, "%d\n", debug_log_level);
	}
	return count;
}

static ssize_t himax_debug_level_dump(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct file *filp = NULL;
	mm_segment_t oldfs;
	int result = 0;
	char fileName[128];
	int err = 1;

	printk(KERN_ERR "[Himax] %s ++ \n", __func__);

	if (buf[0] >= '0' && buf[0] <= '9' && buf[1] == '\n') {
		debug_log_level = buf[0] - '0';
		return count;
	}

	if (buf[0] == 'i') {	/*irq */
		debug_level_cmd = buf[0];

		if (buf[2] == '1') {	/*enable irq       */
			enable_irq(himax_data->irq);
			irq_enable = true;
		} else if (buf[2] == '0') {	/*disable irq */
			disable_irq(himax_data->irq);
			irq_enable = false;
		} else {
			printk(KERN_ERR
			       "[Himax] %s: debug_level command = 'i' , parameter error.\n",
			       __func__);
		}
		return count;
	}

	if (buf[0] == 'h') {	/*handshaking */
		debug_level_cmd = buf[0];

		disable_irq(himax_data->irq);

		handshaking_result = himax_hang_shaking();	/*0:Running, 1:Stop, 2:I2C Fail  */

		enable_irq(himax_data->irq);

		return count;
	}

	if (buf[0] == 'v') {	/*firmware version */
		debug_level_cmd = buf[0];
		himax_read_fw_ver(true);
		return count;
	}

	if (buf[0] == 'd') {	/*test */
		debug_level_cmd = buf[0];
		return count;
	}

	if (buf[0] == 't') {

		/*----[ENABLE_CHIP_STATUS_MONITOR]-------------------------start	 */
#ifdef ENABLE_CHIP_STATUS_MONITOR
		cancel_delayed_work_sync(&himax_data->himax_chip_monitor);
#endif
		/*----[ENABLE_CHIP_STATUS_MONITOR]-------------------------end	 */

		/*Wakelock Protect start */
		wake_lock(&himax_data->wake_lock);
		/*Wakelock Protect end */

		/*Mutexlock Protect Start */
		mutex_lock(&himax_data->mutex_lock);
		/*Mutexlock Protect End */

		debug_level_cmd = buf[0];
		fw_update_complete = false;

		memset(fileName, 0, 128);
		/* parse the file name */
		snprintf(fileName, count - 2, "%s", &buf[2]);
		printk(KERN_INFO "[Himax] %s: upgrade from file(%s) start!\n",
		       __func__, fileName);
		/* open file */
		filp = filp_open(fileName, O_RDONLY, 0);
		if (IS_ERR(filp)) {
			printk(KERN_ERR
			       "[Himax] %s: open firmware file failed\n",
			       __func__);
			goto firmware_upgrade_done;
			/*return count; */
		}
		oldfs = get_fs();
		set_fs(get_ds());

		/* read the latest firmware binary file */
		result =
		    filp->f_op->read(filp, upgrade_fw, sizeof(upgrade_fw),
				     &filp->f_pos);
		if (result < 0) {
			printk(KERN_ERR
			       "[Himax] %s: read firmware file failed\n",
			       __func__);
			goto firmware_upgrade_done;
			/*return count; */
		}

		set_fs(oldfs);
		filp_close(filp, NULL);

		printk(KERN_INFO
		       "[Himax] %s: upgrade start,len %d: %02X, %02X, %02X, %02X.\n",
		       __func__, result, upgrade_fw[0], upgrade_fw[1],
		       upgrade_fw[2], upgrade_fw[3]);
		printk(KERN_INFO "[Himax] %s: upgrade version: %02X %02X.\n",
		       __func__, upgrade_fw[0x0085], upgrade_fw[0x0086]);
		printk(KERN_INFO
		       "[Himax] %s: last 4 byte, %02X, %02X, %02X, %02X.\n",
		       __func__, upgrade_fw[0x7FFC], upgrade_fw[0x7FFD],
		       upgrade_fw[0x7FFE], upgrade_fw[0x7FFF]);

		if (result > 0) {
			/* start to upgrade */
			disable_irq(himax_data->irq);
			if (fts_ctpm_fw_upgrade_with_sys_fs(upgrade_fw, result)
			    == 0) {
				printk(KERN_INFO
				       "[Himax] %s: TP upgrade error, line: %d\n",
				       __func__, __LINE__);
				fw_update_complete = false;
			} else {
				printk(KERN_INFO
				       "[Himax] %s: TP upgrade OK, line: %d\n",
				       __func__, __LINE__);
				fw_update_complete = true;
			}
			enable_irq(himax_data->irq);
			goto firmware_upgrade_done;
			/*return count;                     */
		}
	}

firmware_upgrade_done:

	mutex_unlock(&himax_data->mutex_lock);

	wake_unlock(&himax_data->wake_lock);

	/*----[ENABLE_CHIP_RESET_MACHINE]-------------start	 */
	queue_delayed_work(himax_data->himax_wq,
			   &himax_data->himax_chip_reset_work, 0);
	/*----[ENABLE_CHIP_RESET_MACHINE]-------------end */

	/*todo himax_chip->tp_firmware_upgrade_proceed = 0; */
	/*todo himax_chip->suspend_state = 0; */
	/*todo enable_irq(himax_chip->irq); */

	/*----[ENABLE_CHIP_STATUS_MONITOR]------------start	 */
#ifdef ENABLE_CHIP_STATUS_MONITOR
	queue_delayed_work(himax_data->himax_wq,
			   &himax_data->himax_chip_monitor, 10 * HZ);
#endif
	/*----[ENABLE_CHIP_STATUS_MONITOR]-----------end	 */

	return count;
}
#endif
/*----[HX_TP_SYS_DEBUG_LEVEL]------------end */

/*----[HX_TP_SYS_DIAG]-------------------start */
#ifdef HX_TP_SYS_DIAG
static uint8_t *getMutualBuffer(void)
{
	return diag_mutual;
}

static uint8_t *getSelfBuffer(void)
{
	return &diag_self[0];
}

static uint8_t getXChannel(void)
{
	return x_channel;
}

static uint8_t getYChannel(void)
{
	return y_channel;
}

static uint8_t getDiagCommand(void)
{
	return diag_command;
}

static void setXChannel(uint8_t x)
{
	x_channel = x;
}

static void setYChannel(uint8_t y)
{
	y_channel = y;
}

static void setMutualBuffer(void)
{
	diag_mutual =
	    kzalloc(x_channel * y_channel * sizeof(uint8_t), GFP_KERNEL);
}

static ssize_t himax_diag_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	size_t count = 0;
	uint32_t loop_i;
	uint16_t mutual_num, self_num, width;

	mutual_num = x_channel * y_channel;
	self_num = x_channel + y_channel;	/*don't add KEY_COUNT */

	width = x_channel;
	count +=
	    sprintf(buf + count, "ChannelStart: %4d, %4d\n\n", x_channel,
		    y_channel);

	/* start to show out the raw data in adb shell */
	if (diag_command >= 1 && diag_command <= 6) {
		if (diag_command <= 3) {
			for (loop_i = 0; loop_i < mutual_num; loop_i++) {
				count +=
				    sprintf(buf + count, "%4d",
					    diag_mutual[loop_i]);
				if ((loop_i % width) == (width - 1)) {
					count +=
					    sprintf(buf + count, " %3d\n",
						    diag_self[width +
							      loop_i / width]);
				}
			}
			count += sprintf(buf + count, "\n");
			for (loop_i = 0; loop_i < width; loop_i++) {
				count +=
				    sprintf(buf + count, "%4d",
					    diag_self[loop_i]);
				if (((loop_i) % width) == (width - 1)) {
					count += sprintf(buf + count, "\n");
				}
			}

#ifdef HX_EN_SEL_BUTTON
			count += sprintf(buf + count, "\n");
			for (loop_i = 0; loop_i < HX_BT_NUM; loop_i++) {
				count +=
				    sprintf(buf + count, "%4d",
					    diag_self[HX_RX_NUM + HX_TX_NUM +
						      loop_i]);
			}
#endif
		} else if (diag_command > 4) {
			for (loop_i = 0; loop_i < self_num; loop_i++) {
				count +=
				    sprintf(buf + count, "%4d",
					    diag_self[loop_i]);
				if (((loop_i - mutual_num) % width) ==
				    (width - 1)) {
					count += sprintf(buf + count, "\n");
				}
			}
		} else {
			for (loop_i = 0; loop_i < mutual_num; loop_i++) {
				count +=
				    sprintf(buf + count, "%4d",
					    diag_mutual[loop_i]);
				if ((loop_i % width) == (width - 1)) {
					count += sprintf(buf + count, "\n");
				}
			}
		}
		count += sprintf(buf + count, "ChannelEnd");
		count += sprintf(buf + count, "\n");
	} else if (diag_command == 7) {
		for (loop_i = 0; loop_i < 128; loop_i++) {
			if ((loop_i % 16) == 0) {
				count += sprintf(buf + count, "LineStart:");
			}

			count += sprintf(buf + count, "%4d", diag_coor[loop_i]);
			if ((loop_i % 16) == 15) {
				count += sprintf(buf + count, "\n");
			}
		}
	}
	return count;
}

static ssize_t himax_diag_dump(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	const uint8_t command_ec_128_raw_flag = 0x01;
	const uint8_t command_ec_24_normal_flag = 0x00;

	uint8_t command_ec_128_raw_baseline_flag = 0x02;
	uint8_t command_ec_128_raw_bank_flag = 0x03;

	uint8_t command_91h[2] = { 0x91, 0x00 };
	uint8_t command_82h[1] = { 0x82 };
	uint8_t command_F3h[2] = { 0xF3, 0x00 };
	uint8_t command_83h[1] = { 0x83 };
	uint8_t receive[1];

	if (IC_TYPE != HX_85XX_D_SERIES_PWON) {
		command_ec_128_raw_baseline_flag =
		    0x02 | command_ec_128_raw_flag;
	} else {
		command_ec_128_raw_baseline_flag = 0x02;
		command_ec_128_raw_bank_flag = 0x03;
	}

	if (buf[0] == '1') {	/*IIR */
		command_91h[1] = command_ec_128_raw_baseline_flag;	/*A:0x03 , D:0x02 */
		i2c_himax_write(himax_data->client, command_91h[0],
				&command_91h[1], 1, DEFAULT_RETRY_CNT);
		diag_command = buf[0] - '0';
		printk(KERN_ERR "[Himax]: diag_command=0x%x\n", diag_command);
	} else if (buf[0] == '2') {	/*DC */
		command_91h[1] = command_ec_128_raw_flag;	/*0x01 */
		i2c_himax_write(himax_data->client, command_91h[0],
				&command_91h[1], 1, DEFAULT_RETRY_CNT);
		diag_command = buf[0] - '0';
		printk(KERN_ERR "[Himax]: diag_command=0x%x\n", diag_command);
	} else if (buf[0] == '3') {	/*BANK */
		if (IC_TYPE != HX_85XX_D_SERIES_PWON) {
			i2c_himax_write(himax_data->client, command_82h[0],
					&command_82h[0], 0, DEFAULT_RETRY_CNT);
			msleep(50);

			i2c_himax_read(himax_data->client, command_F3h[0],
				       receive, 1, DEFAULT_RETRY_CNT);
			command_F3h[1] = (receive[0] | 0x80);
			i2c_himax_write(himax_data->client, command_F3h[0],
					&command_F3h[1], 1, DEFAULT_RETRY_CNT);

			command_91h[1] = command_ec_128_raw_baseline_flag;
			i2c_himax_write(himax_data->client, command_91h[0],
					&command_91h[1], 1, DEFAULT_RETRY_CNT);

			i2c_himax_write(himax_data->client, command_83h[0],
					&command_83h[0], 0, DEFAULT_RETRY_CNT);
			msleep(50);
		} else {
			command_91h[1] = command_ec_128_raw_bank_flag;	/*0x03 */
			i2c_himax_write(himax_data->client, command_91h[0],
					&command_91h[1], 1, DEFAULT_RETRY_CNT);
		}
		diag_command = buf[0] - '0';
		printk(KERN_ERR "[Himax]: diag_command=0x%x\n", diag_command);
	} else if (buf[0] == '7') {
		diag_command = buf[0] - '0';
	}
	/*coordinate dump start */
	else if (buf[0] == '8') {
		diag_command = buf[0] - '0';

		coordinate_fn =
		    filp_open(DIAG_COORDINATE_FILE,
			      O_CREAT | O_WRONLY | O_APPEND | O_TRUNC, 0666);
		if (IS_ERR(coordinate_fn)) {
			printk(KERN_INFO
			       "[Himax] %s: coordinate_dump_file_create error\n",
			       __func__);
			coordinate_dump_enable = 0;
			filp_close(coordinate_fn, NULL);
		}
		coordinate_dump_enable = 1;
	} else if (buf[0] == '9') {
		coordinate_dump_enable = 0;
		diag_command = buf[0] - '0';

		if (!IS_ERR(coordinate_fn)) {
			filp_close(coordinate_fn, NULL);
		}
	}
	/*coordinate dump end */
	else {
		if (IC_TYPE != HX_85XX_D_SERIES_PWON) {
			i2c_himax_write(himax_data->client, command_82h[0],
					&command_82h[0], 0, DEFAULT_RETRY_CNT);
			msleep(50);
			command_91h[1] = command_ec_24_normal_flag;
			i2c_himax_write(himax_data->client, command_91h[0],
					&command_91h[1], 1, DEFAULT_RETRY_CNT);
			i2c_himax_read(himax_data->client, command_F3h[0],
				       receive, 1, DEFAULT_RETRY_CNT);
			command_F3h[1] = (receive[0] & 0x7F);
			i2c_himax_write(himax_data->client, command_F3h[0],
					&command_F3h[1], 1, DEFAULT_RETRY_CNT);
			i2c_himax_write(himax_data->client, command_83h[0],
					&command_83h[0], 0, DEFAULT_RETRY_CNT);
		} else {
			command_91h[1] = command_ec_24_normal_flag;
			i2c_himax_write(himax_data->client, command_91h[0],
					&command_91h[1], 1, DEFAULT_RETRY_CNT);
		}
		diag_command = 0;
		printk(KERN_ERR "[Himax]: diag_command=0x%x\n", diag_command);
	}
	return count;
}

static ssize_t himax_chip_raw_data_store(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	size_t count = 0;
	uint32_t loop_i;
	uint16_t mutual_num, self_num, width;
	struct file *filp = NULL;
	mm_segment_t oldfs;

	mutual_num = x_channel * y_channel;
	self_num = x_channel + y_channel;
	width = x_channel;

	if (diag_command == 1) {
		filp = filp_open(TOUCH_DC_DUMP_FILE, O_RDWR | O_CREAT, S_IRUSR);
		if (IS_ERR(filp)) {
			printk(KERN_ERR
			       "[Himax] %s: open /data/log/touch/ME372CL_touch_dc.txt failed\n",
			       __func__);
			return 0;
		}
		oldfs = get_fs();
		set_fs(get_ds());
	} else if (diag_command == 2) {
		filp =
		    filp_open(TOUCH_IIR_DUMP_FILE, O_RDWR | O_CREAT, S_IRUSR);
		if (IS_ERR(filp)) {
			printk(KERN_ERR
			       "[Himax] %s: open /data/log/touch/ME372CL_touch_diff.txt failed\n",
			       __func__);
			return 0;
		}
		oldfs = get_fs();
		set_fs(get_ds());
	} else if (diag_command == 3) {
		filp =
		    filp_open(TOUCH_BANK_DUMP_FILE, O_RDWR | O_CREAT, S_IRUSR);
		if (IS_ERR(filp)) {
			printk(KERN_ERR
			       "[Himax] %s: open /data/log/touch/ME372CL_touch_bank.txt failed\n",
			       __func__);
			return 0;
		}
		oldfs = get_fs();
		set_fs(get_ds());
	}

	count +=
	    sprintf(buf + count, "Channel: %4d, %4d\n\n", x_channel, y_channel);
	if (diag_command >= 1 && diag_command <= 6) {
		if (diag_command < 4) {
			for (loop_i = 0; loop_i < mutual_num; loop_i++) {
				count +=
				    sprintf(buf + count, "%4d",
					    diag_mutual[loop_i]);
				if ((loop_i % width) == (width - 1)) {
					count +=
					    sprintf(buf + count, " %3d\n",
						    diag_self[width +
							      loop_i / width]);
				}
			}
			count += sprintf(buf + count, "\n");
			for (loop_i = 0; loop_i < width; loop_i++) {
				count +=
				    sprintf(buf + count, "%4d",
					    diag_self[loop_i]);
				if (((loop_i) % width) == (width - 1)) {
					count += sprintf(buf + count, "\n");
				}
			}

#ifdef HX_EN_SEL_BUTTON
			count += sprintf(buf + count, "\n");
			for (loop_i = 0; loop_i < HX_BT_NUM; loop_i++) {
				count +=
				    sprintf(buf + count, "%4d",
					    diag_self[HX_RX_NUM + HX_TX_NUM +
						      loop_i]);
			}
#endif
		} else if (diag_command > 4) {
			for (loop_i = 0; loop_i < self_num; loop_i++) {
				count +=
				    sprintf(buf + count, "%4d",
					    diag_self[loop_i]);
				if (((loop_i - mutual_num) % width) ==
				    (width - 1)) {
					count += sprintf(buf + count, "\n");
				}
			}
		} else {
			for (loop_i = 0; loop_i < mutual_num; loop_i++) {
				count +=
				    sprintf(buf + count, "%4d",
					    diag_mutual[loop_i]);
				if ((loop_i % width) == (width - 1)) {
					count += sprintf(buf + count, "\n");
				}
			}
		}
	}
	if (diag_command >= 1 && diag_command <= 3) {
		filp->f_op->write(filp, buf, count, &filp->f_pos);
		set_fs(oldfs);
		filp_close(filp, NULL);
	}
	return count;
}
#endif
/*----[HX_TP_SYS_DIAG]----------------end */

/*----[HX_TP_SYS_FLASH_DUMP]----------start */
#ifdef HX_TP_SYS_FLASH_DUMP

static uint8_t getFlashCommand(void)
{
	return flash_command;
}

static uint8_t getFlashDumpProgress(void)
{
	return flash_progress;
}

static uint8_t getFlashDumpComplete(void)
{
	return flash_dump_complete;
}

static uint8_t getFlashDumpFail(void)
{
	return flash_dump_fail;
}

static uint8_t getSysOperation(void)
{
	return sys_operation;
}

static uint8_t getFlashReadStep(void)
{
	return flash_read_step;
}

static uint8_t getFlashDumpSector(void)
{
	return flash_dump_sector;
}

static uint8_t getFlashDumpPage(void)
{
	return flash_dump_page;
}

static bool getFlashDumpGoing(void)
{
	return flash_dump_going;
}

static void setFlashBuffer(void)
{
	int i = 0;
	flash_buffer = kzalloc(32768 * sizeof(uint8_t), GFP_KERNEL);
	for (i = 0; i < 32768; i++) {
		flash_buffer[i] = 0x00;
	}
}

static void setSysOperation(uint8_t operation)
{
	sys_operation = operation;
}

static void setFlashDumpProgress(uint8_t progress)
{
	flash_progress = progress;
	printk
	    ("[Himax]: TPPPP setFlashDumpProgress : progress = %d ,flash_progress = %d \n",
	     progress, flash_progress);
}

static void setFlashDumpComplete(uint8_t status)
{
	flash_dump_complete = status;
}

static void setFlashDumpFail(uint8_t fail)
{
	flash_dump_fail = fail;
}

static void setFlashCommand(uint8_t command)
{
	flash_command = command;
}

static void setFlashReadStep(uint8_t step)
{
	flash_read_step = step;
}

static void setFlashDumpSector(uint8_t sector)
{
	flash_dump_sector = sector;
}

static void setFlashDumpPage(uint8_t page)
{
	flash_dump_page = page;
}

static void setFlashDumpGoing(bool going)
{
	flash_dump_going = going;
}

static ssize_t himax_flash_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int loop_i;
	uint8_t local_flash_read_step = 0;
	uint8_t local_flash_complete = 0;
	uint8_t local_flash_progress = 0;
	uint8_t local_flash_command = 0;
	uint8_t local_flash_fail = 0;

	local_flash_complete = getFlashDumpComplete();
	local_flash_progress = getFlashDumpProgress();
	local_flash_command = getFlashCommand();
	local_flash_fail = getFlashDumpFail();

	printk("[Himax]: TPPPP flash_progress = %d \n", local_flash_progress);

	if (local_flash_fail) {
		ret += sprintf(buf + ret, "FlashStart:Fail \n");
		ret += sprintf(buf + ret, "FlashEnd");
		ret += sprintf(buf + ret, "\n");
		return ret;
	}

	if (!local_flash_complete) {
		ret +=
		    sprintf(buf + ret, "FlashStart:Ongoing:0x%2.2x \n",
			    flash_progress);
		ret += sprintf(buf + ret, "FlashEnd");
		ret += sprintf(buf + ret, "\n");
		return ret;
	}

	if (local_flash_command == 1 && local_flash_complete) {
		ret += sprintf(buf + ret, "FlashStart:Complete \n");
		ret += sprintf(buf + ret, "FlashEnd");
		ret += sprintf(buf + ret, "\n");
		return ret;
	}

	if (local_flash_command == 3 && local_flash_complete) {
		ret += sprintf(buf + ret, "FlashStart: \n");
		for (loop_i = 0; loop_i < 128; loop_i++) {
			ret +=
			    sprintf(buf + ret, "x%2.2x", flash_buffer[loop_i]);
			if ((loop_i % 16) == 15) {
				ret += sprintf(buf + ret, "\n");
			}
		}
		ret += sprintf(buf + ret, "FlashEnd");
		ret += sprintf(buf + ret, "\n");
		return ret;
	}

	/*flash command == 0 , report the data */
	local_flash_read_step = getFlashReadStep();

	ret += sprintf(buf + ret, "FlashStart:%2.2x \n", local_flash_read_step);

	for (loop_i = 0; loop_i < 1024; loop_i++) {
		ret +=
		    sprintf(buf + ret, "x%2.2X",
			    flash_buffer[local_flash_read_step * 1024 +
					 loop_i]);

		if ((loop_i % 16) == 15) {
			ret += sprintf(buf + ret, "\n");
		}
	}

	ret += sprintf(buf + ret, "FlashEnd");
	ret += sprintf(buf + ret, "\n");
	return ret;
}

/*-----------------------------------------------------------------------------------
//himax_flash_store
//
//command 0 : Read the page by step number
//command 1 : driver start to dump flash data, save it to mem
//command 2 : driver start to dump flash data, save it to sdcard/Flash_Dump.bin
//
//----------------------------------------------------------------------------------- */
static ssize_t himax_flash_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
	char buf_tmp[6];
	unsigned long result = 0;
	uint8_t loop_i = 0;
	int base = 0;

	memset(buf_tmp, 0x0, sizeof(buf_tmp));

	printk(KERN_INFO "[Himax] %s: buf[0] = %s\n", __func__, buf);

	if (getSysOperation() == 1) {
		printk("[Himax] %s: SYS is busy , return!\n", __func__);
		return count;
	}

	if (buf[0] == '0') {
		setFlashCommand(0);
		if (buf[1] == ':' && buf[2] == 'x') {
			memcpy(buf_tmp, buf + 3, 2);
			printk(KERN_INFO "[Himax] %s: read_Step = %s\n",
			       __func__, buf_tmp);
			if (!strict_strtoul(buf_tmp, 16, &result)) {
				printk("[Himax] %s: read_Step = %lu \n",
				       __func__, result);
				setFlashReadStep(result);
			}
		}
	} else if (buf[0] == '1') {
		setSysOperation(1);
		setFlashCommand(1);
		setFlashDumpProgress(0);
		setFlashDumpComplete(0);
		setFlashDumpFail(0);
		queue_work(himax_data->flash_wq, &himax_data->flash_work);
	} else if (buf[0] == '2') {
		setSysOperation(1);
		setFlashCommand(2);
		setFlashDumpProgress(0);
		setFlashDumpComplete(0);
		setFlashDumpFail(0);

		queue_work(himax_data->flash_wq, &himax_data->flash_work);
	} else if (buf[0] == '3') {
		setSysOperation(1);
		setFlashCommand(3);
		setFlashDumpProgress(0);
		setFlashDumpComplete(0);
		setFlashDumpFail(0);

		memcpy(buf_tmp, buf + 3, 2);
		if (!strict_strtoul(buf_tmp, 16, &result)) {
			setFlashDumpSector(result);
		}

		memcpy(buf_tmp, buf + 7, 2);
		if (!strict_strtoul(buf_tmp, 16, &result)) {
			setFlashDumpPage(result);
		}

		queue_work(himax_data->flash_wq, &himax_data->flash_work);
	} else if (buf[0] == '4') {
		printk(KERN_INFO "[Himax] %s: command 4 enter.\n", __func__);
		setSysOperation(1);
		setFlashCommand(4);
		setFlashDumpProgress(0);
		setFlashDumpComplete(0);
		setFlashDumpFail(0);

		memcpy(buf_tmp, buf + 3, 2);
		if (!strict_strtoul(buf_tmp, 16, &result)) {
			setFlashDumpSector(result);
		} else {
			printk(KERN_INFO
			       "[Himax] %s: command 4 , sector error.\n",
			       __func__);
			return count;
		}

		memcpy(buf_tmp, buf + 7, 2);
		if (!strict_strtoul(buf_tmp, 16, &result)) {
			setFlashDumpPage(result);
		} else {
			printk(KERN_INFO
			       "[Himax] %s: command 4 , page error.\n",
			       __func__);
			return count;
		}

		base = 11;

		printk(KERN_INFO
		       "=========Himax flash page buffer start=========\n");
		for (loop_i = 0; loop_i < 128; loop_i++) {
			memcpy(buf_tmp, buf + base, 2);
			if (!strict_strtoul(buf_tmp, 16, &result)) {
				flash_buffer[loop_i] = result;
				printk(" %d ", flash_buffer[loop_i]);
				if (loop_i % 16 == 15) {
					printk("\n");
				}
			}
			base += 3;
		}
		printk(KERN_INFO
		       "=========Himax flash page buffer end=========\n");

		queue_work(himax_data->flash_wq, &himax_data->flash_work);
	}
	return count;
}

static void himax_ts_flash_work_func(struct work_struct *work)
{
	/*struct himax_ts_data *ts = container_of(work, struct himax_ts_data, flash_work); */

	uint8_t page_tmp[128];
	uint8_t x59_tmp[4] = { 0, 0, 0, 0 };
	int i = 0, j = 0, k = 0, l = 0, /* j_limit = 0, */ buffer_ptr =
	    0, flash_end_count = 0;
	uint8_t local_flash_command = 0;
	uint8_t sector = 0;
	uint8_t page = 0;

	uint8_t x81_command[2] = { 0x81, 0x00 };
	uint8_t x82_command[2] = { 0x82, 0x00 };
	uint8_t x42_command[2] = { 0x42, 0x00 };
	uint8_t x43_command[4] = { 0x43, 0x00, 0x00, 0x00 };
	uint8_t x44_command[4] = { 0x44, 0x00, 0x00, 0x00 };
	uint8_t x45_command[5] = { 0x45, 0x00, 0x00, 0x00, 0x00 };
	uint8_t x46_command[2] = { 0x46, 0x00 };
	uint8_t x4A_command[2] = { 0x4A, 0x00 };
	uint8_t x4D_command[2] = { 0x4D, 0x00 };
	/*uint8_t x59_command[2] = {0x59,0x00}; */

	disable_irq(himax_data->irq);
	setFlashDumpGoing(true);

#ifdef HX_RST_PIN_FUNC
	himax_hw_reset();
#endif

	sector = getFlashDumpSector();
	page = getFlashDumpPage();

	local_flash_command = getFlashCommand();

	if (i2c_himax_master_write(himax_data->client, x81_command, 1, 3) < 0) {	/*sleep out */
		printk(KERN_ERR "[Himax] %s: i2c write 81 fail.\n", __func__);
		goto Flash_Dump_i2c_transfer_error;
	}
	msleep(120);

	if (i2c_himax_master_write(himax_data->client, x82_command, 1, 3) < 0) {
		printk(KERN_ERR "[Himax] %s: i2c write 82 fail.\n", __func__);
		goto Flash_Dump_i2c_transfer_error;
	}
	msleep(100);

	printk(KERN_INFO "[Himax] %s: local_flash_command = %d enter.\n",
	       __func__, local_flash_command);
	printk(KERN_INFO "[Himax] %s: flash buffer start.\n", __func__);
	for (i = 0; i < 128; i++) {
		printk(KERN_INFO " %2.2x ", flash_buffer[i]);
		if ((i % 16) == 15) {
			printk("\n");
		}
	}
	printk(KERN_INFO "[Himax] %s: flash buffer end.\n", __func__);

	if (local_flash_command == 1 || local_flash_command == 2) {
		x43_command[1] = 0x01;
		if (i2c_himax_write
		    (himax_data->client, x43_command[0], &x43_command[1], 1,
		     DEFAULT_RETRY_CNT) < 0) {
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(100);

		for (i = 0; i < 8; i++) {
			for (j = 0; j < 32; j++) {
				printk("[Himax]: TPPPP Step 2 i=%d , j=%d %s\n",
				       i, j, __func__);
				/*read page start */
				for (k = 0; k < 128; k++) {
					page_tmp[k] = 0x00;
				}
				for (k = 0; k < 32; k++) {
					x44_command[1] = k;
					x44_command[2] = j;
					x44_command[3] = i;
					if (i2c_himax_write
					    (himax_data->client, x44_command[0],
					     &x44_command[1], 3,
					     DEFAULT_RETRY_CNT) < 0) {
						printk(KERN_ERR
						       "[Himax] %s: i2c write 44 fail.\n",
						       __func__);
						goto Flash_Dump_i2c_transfer_error;
					}

					if (i2c_himax_write_command
					    (himax_data->client, x46_command[0],
					     DEFAULT_RETRY_CNT) < 0) {
						printk(KERN_ERR
						       "[Himax]%s: i2c write 46 fail.\n",
						       __func__);
						goto Flash_Dump_i2c_transfer_error;
					}
					/*msleep(2); */
					if (i2c_himax_read
					    (himax_data->client, 0x59, x59_tmp,
					     4, DEFAULT_RETRY_CNT) < 0) {
						printk(KERN_ERR
						       "[Himax] %s: i2c write 59 fail.\n",
						       __func__);
						goto Flash_Dump_i2c_transfer_error;
					}
					/*msleep(2); */
					for (l = 0; l < 4; l++) {
						page_tmp[k * 4 + l] =
						    x59_tmp[l];
					}
					/*msleep(10); */
				}
				/*read page end  */

				for (k = 0; k < 128; k++) {
					flash_buffer[buffer_ptr++] =
					    page_tmp[k];

					/*if(page_tmp[k] == 0xFF)
					   //{
					   //   flash_end_count ++;
					   //   if(flash_end_count == 32)
					   //   {
					   //           flash_end_count = 0;
					   //           buffer_ptr = buffer_ptr -32;
					   //           goto FLASH_END;
					   //   }
					   //}
					   //else
					   //{
					   //   flash_end_count = 0;
					   //} */
				}
				setFlashDumpProgress(i * 32 + j);
			}
		}
	} else if (local_flash_command == 3) {
		x43_command[1] = 0x01;
		if (i2c_himax_write
		    (himax_data->client, x43_command[0], &x43_command[1], 1,
		     DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c write 43 fail.\n",
			       __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(100);

		for (i = 0; i < 128; i++) {
			page_tmp[i] = 0x00;
		}

		for (i = 0; i < 32; i++) {
			x44_command[1] = i;
			x44_command[2] = page;
			x44_command[3] = sector;

			if (i2c_himax_write
			    (himax_data->client, x44_command[0],
			     &x44_command[1], 3, DEFAULT_RETRY_CNT) < 0) {
				printk(KERN_ERR
				       "[Himax] %s: i2c write 44 fail.\n",
				       __func__);
				goto Flash_Dump_i2c_transfer_error;
			}

			if (i2c_himax_write_command
			    (himax_data->client, x46_command[0],
			     DEFAULT_RETRY_CNT) < 0) {
				printk(KERN_ERR
				       "[Himax] %s: i2c write 46 fail.\n",
				       __func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			/*msleep(2); */
			if (i2c_himax_read
			    (himax_data->client, 0x59, x59_tmp, 4,
			     DEFAULT_RETRY_CNT) < 0) {
				printk(KERN_ERR
				       "[Himax] %s: i2c write 59 fail.\n",
				       __func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			/*msleep(2); */
			for (j = 0; j < 4; j++) {
				page_tmp[i * 4 + j] = x59_tmp[j];
			}
			/*msleep(10); */
		}
		/*read page end */
		for (i = 0; i < 128; i++) {
			flash_buffer[buffer_ptr++] = page_tmp[i];
		}
	} else if (local_flash_command == 4) {
		/*page write flow. */
		printk(KERN_INFO
		       "[Himax] %s: local_flash_command = 4, enter.\n",
		       __func__);

		/*---------------
		// unlock flash
		//-------------- */
		x43_command[1] = 0x01;
		x43_command[2] = 0x00;
		x43_command[3] = 0x06;
		if (i2c_himax_write
		    (himax_data->client, x43_command[0], &x43_command[1], 3,
		     DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c write 43 fail.\n",
			       __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x44_command[1] = 0x03;
		x44_command[2] = 0x00;
		x44_command[3] = 0x00;
		if (i2c_himax_write
		    (himax_data->client, x44_command[0], &x44_command[1], 3,
		     DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c write 44 fail.\n",
			       __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x45_command[1] = 0x00;
		x45_command[2] = 0x00;
		x45_command[3] = 0x3D;
		x45_command[4] = 0x03;
		if (i2c_himax_write
		    (himax_data->client, x45_command[0], &x45_command[1], 4,
		     DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c write 45 fail.\n",
			       __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		if (i2c_himax_write_command
		    (himax_data->client, x4A_command[0],
		     DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c write 4A fail.\n",
			       __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(50);

		/*---------------
		// page erase
		//-------------- */
		x43_command[1] = 0x01;
		x43_command[2] = 0x00;
		x43_command[3] = 0x02;
		if (i2c_himax_write
		    (himax_data->client, x43_command[0], &x43_command[1], 3,
		     DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c write 43 fail.\n",
			       __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x44_command[1] = 0x00;
		x44_command[2] = page;
		x44_command[3] = sector;
		if (i2c_himax_write
		    (himax_data->client, x44_command[0], &x44_command[1], 3,
		     DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c write 44 fail.\n",
			       __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		if (i2c_himax_write_command
		    (himax_data->client, x4D_command[0],
		     DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c write 4D fail.\n",
			       __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(100);

		/*------------------------
		// enter manual mode
		//------------------------ */
		x42_command[1] = 0x01;
		if (i2c_himax_write
		    (himax_data->client, x42_command[0], &x42_command[1], 1,
		     DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c write 42 fail.\n",
			       __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(100);

		/*---------------------
		// flash enable
		//--------------------- */
		x43_command[1] = 0x01;
		x43_command[2] = 0x00;
		if (i2c_himax_write
		    (himax_data->client, x43_command[0], &x43_command[1], 2,
		     DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c write 43 fail.\n",
			       __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		/*------------------------
		// set flash address
		//------------------------ */
		x44_command[1] = 0x00;
		x44_command[2] = page;
		x44_command[3] = sector;
		if (i2c_himax_write
		    (himax_data->client, x44_command[0], &x44_command[1], 3,
		     DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c write 44 fail.\n",
			       __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		/*-----------------------------------------------------------------------------
		// manual mode command : 47 to latch the flash address when page address change.
		//----------------------------------------------------------------------------- */
		x43_command[1] = 0x01;
		x43_command[2] = 0x09;
		if (i2c_himax_write
		    (himax_data->client, x43_command[0], &x43_command[1], 2,
		     DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c write 43 fail.\n",
			       __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x43_command[1] = 0x01;
		x43_command[2] = 0x0D;
		if (i2c_himax_write
		    (himax_data->client, x43_command[0], &x43_command[1], 2,
		     DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c write 43 fail.\n",
			       __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x43_command[1] = 0x01;
		x43_command[2] = 0x09;
		if (i2c_himax_write
		    (himax_data->client, x43_command[0], &x43_command[1], 2,
		     DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c write 43 fail.\n",
			       __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		for (i = 0; i < 32; i++) {
			printk(KERN_INFO "[Himax]: i=%d \n", i);
			x44_command[1] = i;
			x44_command[2] = page;
			x44_command[3] = sector;
			if (i2c_himax_write
			    (himax_data->client, x44_command[0],
			     &x44_command[1], 3, DEFAULT_RETRY_CNT) < 0) {
				printk(KERN_ERR
				       "[Himax] %s: i2c write 44 fail.\n",
				       __func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			msleep(10);

			x45_command[1] = flash_buffer[i * 4 + 0];
			x45_command[2] = flash_buffer[i * 4 + 1];
			x45_command[3] = flash_buffer[i * 4 + 2];
			x45_command[4] = flash_buffer[i * 4 + 3];
			if (i2c_himax_write
			    (himax_data->client, x45_command[0],
			     &x45_command[1], 4, DEFAULT_RETRY_CNT) < 0) {
				printk(KERN_ERR
				       "[Himax] %s: i2c write 45 fail.\n",
				       __func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			msleep(10);

			/*-----------------------------------------------------------------
			// manual mode command : 48 ,data will be written into flash buffer
			//---------------------------------------------------------------- */
			x43_command[1] = 0x01;
			x43_command[2] = 0x0D;
			if (i2c_himax_write
			    (himax_data->client, x43_command[0],
			     &x43_command[1], 2, DEFAULT_RETRY_CNT) < 0) {
				printk(KERN_ERR
				       "[Himax] %s: i2c write 43 fail.\n",
				       __func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			msleep(10);

			x43_command[1] = 0x01;
			x43_command[2] = 0x09;
			if (i2c_himax_write
			    (himax_data->client, x43_command[0],
			     &x43_command[1], 2, DEFAULT_RETRY_CNT) < 0) {
				printk(KERN_ERR
				       "[Himax] %s: i2c write 43 fail.\n",
				       __func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			msleep(10);
		}

		/*---------------------------------------------------------------------
		// manual mode command : 49 ,program data from flash buffer to this page
		//----------------------------------------------------------------------*/
		x43_command[1] = 0x01;
		x43_command[2] = 0x01;
		if (i2c_himax_write
		    (himax_data->client, x43_command[0], &x43_command[1], 2,
		     DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c write 43 fail.\n",
			       __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x43_command[1] = 0x01;
		x43_command[2] = 0x05;
		if (i2c_himax_write
		    (himax_data->client, x43_command[0], &x43_command[1], 2,
		     DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c write 43 fail.\n",
			       __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x43_command[1] = 0x01;
		x43_command[2] = 0x01;
		if (i2c_himax_write
		    (himax_data->client, x43_command[0], &x43_command[1], 2,
		     DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c write 43 fail.\n",
			       __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x43_command[1] = 0x01;
		x43_command[2] = 0x00;
		if (i2c_himax_write
		    (himax_data->client, x43_command[0], &x43_command[1], 2,
		     DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c write 43 fail.\n",
			       __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		/*-----------------
		// flash disable
		//-----------------*/
		x43_command[1] = 0x00;
		if (i2c_himax_write
		    (himax_data->client, x43_command[0], &x43_command[1], 1,
		     DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c write 43 fail.\n",
			       __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		/*---------------------
		// leave manual mode
		//---------------------*/
		x42_command[1] = 0x00;
		if (i2c_himax_write
		    (himax_data->client, x42_command[0], &x42_command[1], 1,
		     DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c write 43 fail.\n",
			       __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		/*----------------------
		// lock flash
		//---------------------- */
		x43_command[1] = 0x01;
		x43_command[2] = 0x00;
		x43_command[3] = 0x06;
		if (i2c_himax_write
		    (himax_data->client, x43_command[0], &x43_command[1], 3,
		     DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c write 43 fail.\n",
			       __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x44_command[1] = 0x03;
		x44_command[2] = 0x00;
		x44_command[3] = 0x00;
		if (i2c_himax_write
		    (himax_data->client, x44_command[0], &x44_command[1], 3,
		     DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c write 44 fail.\n",
			       __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x45_command[1] = 0x00;
		x45_command[2] = 0x00;
		x45_command[3] = 0x7D;
		x45_command[4] = 0x03;
		if (i2c_himax_write
		    (himax_data->client, x45_command[0], &x45_command[1], 4,
		     DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c write 45 fail.\n",
			       __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		if (i2c_himax_write_command
		    (himax_data->client, x4A_command[0],
		     DEFAULT_RETRY_CNT) < 0) {
			printk(KERN_ERR "[Himax] %s: i2c write 4D fail.\n",
			       __func__);
			goto Flash_Dump_i2c_transfer_error;
		}

		msleep(50);

		buffer_ptr = 128;
		printk(KERN_INFO
		       "[Himax]: Flash page write Complete~~~~~~~~~~~~~~~~~~~~~~~\n");
	}

FLASH_END:

	printk("[Himax]: Complete~~~~~~~~~~~~~~~~~~~~~~~\n");

	printk("[Himax]: buffer_ptr = %d \n", buffer_ptr);

	for (i = 0; i < buffer_ptr; i++) {
		printk("%2.2X ", flash_buffer[i]);

		if ((i % 16) == 15) {
			printk("\n");
		}
	}
	printk("[Himax]: End~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");

	i2c_himax_master_write(himax_data->client, x43_command, 1, 3);
	msleep(50);

	if (local_flash_command == 2) {
		struct file *fn;

		fn = filp_open(FLASH_DUMP_FILE, O_CREAT | O_WRONLY, 0);
		if (!IS_ERR(fn)) {
			fn->f_op->write(fn, flash_buffer,
					buffer_ptr * sizeof(uint8_t),
					&fn->f_pos);
			filp_close(fn, NULL);
		}
	}
#ifdef ENABLE_CHIP_RESET_MACHINE
	if (himax_data->init_success) {
		queue_delayed_work(himax_data->himax_wq,
				   &himax_data->himax_chip_reset_work, 0);
	}
#endif

	enable_irq(himax_data->irq);
	setFlashDumpGoing(false);

	setFlashDumpComplete(1);
	setSysOperation(0);
	return;

Flash_Dump_i2c_transfer_error:

#ifdef ENABLE_CHIP_RESET_MACHINE
	if (himax_data->init_success) {
		queue_delayed_work(himax_data->himax_wq,
				   &himax_data->himax_chip_reset_work, 0);
	}
#endif

	enable_irq(himax_data->irq);
	setFlashDumpGoing(false);
	setFlashDumpComplete(0);
	setFlashDumpFail(1);
	setSysOperation(0);
	return;
}
#endif

static ssize_t himax_me372cl_chip_proc_flash_show(char *buf, char **start,
						  off_t off, int count1,
						  int *eof, void *data)
{
	int ret = 0;
	int loop_i;
	uint8_t local_flash_read_step = 0;
	uint8_t local_flash_complete = 0;
	uint8_t local_flash_progress = 0;
	uint8_t local_flash_command = 0;
	uint8_t local_flash_fail = 0;

	local_flash_complete = getFlashDumpComplete();
	local_flash_progress = getFlashDumpProgress();
	local_flash_command = getFlashCommand();
	local_flash_fail = getFlashDumpFail();

	printk("[Himax]: TPPPP flash_progress = %d \n", local_flash_progress);

	if (local_flash_fail) {
		ret += sprintf(buf + ret, "FlashStart:Fail \n");
		ret += sprintf(buf + ret, "FlashEnd");
		ret += sprintf(buf + ret, "\n");
		return ret;
	}

	if (!local_flash_complete) {
		ret +=
		    sprintf(buf + ret, "FlashStart:Ongoing:0x%2.2x \n",
			    flash_progress);
		ret += sprintf(buf + ret, "FlashEnd");
		ret += sprintf(buf + ret, "\n");
		return ret;
	}

	if (local_flash_command == 1 && local_flash_complete) {
		ret += sprintf(buf + ret, "FlashStart:Complete \n");
		ret += sprintf(buf + ret, "FlashEnd");
		ret += sprintf(buf + ret, "\n");
		return ret;
	}

	if (local_flash_command == 3 && local_flash_complete) {
		ret += sprintf(buf + ret, "FlashStart: \n");
		for (loop_i = 0; loop_i < 128; loop_i++) {
			ret +=
			    sprintf(buf + ret, "x%2.2x", flash_buffer[loop_i]);
			if ((loop_i % 16) == 15) {
				ret += sprintf(buf + ret, "\n");
			}
		}
		ret += sprintf(buf + ret, "FlashEnd");
		ret += sprintf(buf + ret, "\n");
		return ret;
	}

	/*flash command == 0 , report the data */
	local_flash_read_step = getFlashReadStep();

	ret += sprintf(buf + ret, "FlashStart:%2.2x \n", local_flash_read_step);

	for (loop_i = 0; loop_i < 1024; loop_i++) {
		ret +=
		    sprintf(buf + ret, "x%2.2X",
			    flash_buffer[local_flash_read_step * 1024 +
					 loop_i]);

		if ((loop_i % 16) == 15) {
			ret += sprintf(buf + ret, "\n");
		}
	}

	ret += sprintf(buf + ret, "FlashEnd");
	ret += sprintf(buf + ret, "\n");
	return ret;
}

static ssize_t himax_me372cl_chip_proc_flash_store(struct file *filp1,
						   const char *buf,
						   unsigned long len,
						   void *data)
{
	char buf_tmp[6];
	unsigned long result = 0;
	uint8_t loop_i = 0;
	int base = 0;

	memset(buf_tmp, 0x0, sizeof(buf_tmp));

	printk(KERN_INFO "[Himax] %s: buf[0] = %s\n", __func__, buf);

	if (getSysOperation() == 1) {
		printk("[Himax] %s: SYS is busy , return!\n", __func__);
		return len;
	}

	if (buf[0] == '0') {
		setFlashCommand(0);
		if (buf[1] == ':' && buf[2] == 'x') {
			memcpy(buf_tmp, buf + 3, 2);
			printk(KERN_INFO "[Himax] %s: read_Step = %s\n",
			       __func__, buf_tmp);
			if (!strict_strtoul(buf_tmp, 16, &result)) {
				printk("[Himax] %s: read_Step = %lu \n",
				       __func__, result);
				setFlashReadStep(result);
			}
		}
	} else if (buf[0] == '1') {
		setSysOperation(1);
		setFlashCommand(1);
		setFlashDumpProgress(0);
		setFlashDumpComplete(0);
		setFlashDumpFail(0);
		queue_work(himax_data->flash_wq, &himax_data->flash_work);
	} else if (buf[0] == '2') {
		setSysOperation(1);
		setFlashCommand(2);
		setFlashDumpProgress(0);
		setFlashDumpComplete(0);
		setFlashDumpFail(0);

		queue_work(himax_data->flash_wq, &himax_data->flash_work);
	} else if (buf[0] == '3') {
		setSysOperation(1);
		setFlashCommand(3);
		setFlashDumpProgress(0);
		setFlashDumpComplete(0);
		setFlashDumpFail(0);

		memcpy(buf_tmp, buf + 3, 2);
		if (!strict_strtoul(buf_tmp, 16, &result)) {
			setFlashDumpSector(result);
		}

		memcpy(buf_tmp, buf + 7, 2);
		if (!strict_strtoul(buf_tmp, 16, &result)) {
			setFlashDumpPage(result);
		}

		queue_work(himax_data->flash_wq, &himax_data->flash_work);
	} else if (buf[0] == '4') {
		printk(KERN_INFO "[Himax] %s: command 4 enter.\n", __func__);
		setSysOperation(1);
		setFlashCommand(4);
		setFlashDumpProgress(0);
		setFlashDumpComplete(0);
		setFlashDumpFail(0);

		memcpy(buf_tmp, buf + 3, 2);
		if (!strict_strtoul(buf_tmp, 16, &result)) {
			setFlashDumpSector(result);
		} else {
			printk(KERN_INFO
			       "[Himax] %s: command 4 , sector error.\n",
			       __func__);
			return len;
		}

		memcpy(buf_tmp, buf + 7, 2);
		if (!strict_strtoul(buf_tmp, 16, &result)) {
			setFlashDumpPage(result);
		} else {
			printk(KERN_INFO
			       "[Himax] %s: command 4 , page error.\n",
			       __func__);
			return len;
		}

		base = 11;

		printk(KERN_INFO
		       "=========Himax flash page buffer start=========\n");
		for (loop_i = 0; loop_i < 128; loop_i++) {
			memcpy(buf_tmp, buf + base, 2);
			if (!strict_strtoul(buf_tmp, 16, &result)) {
				flash_buffer[loop_i] = result;
				printk(" %d ", flash_buffer[loop_i]);
				if (loop_i % 16 == 15) {
					printk("\n");
				}
			}
			base += 3;
		}
		printk(KERN_INFO
		       "=========Himax flash page buffer end=========\n");

		queue_work(himax_data->flash_wq, &himax_data->flash_work);
	}
	return len;
}

static const struct file_operations flash_fops = {
	.owner = THIS_MODULE,
	.read = himax_me372cl_chip_proc_flash_show,
	.write = himax_me372cl_chip_proc_flash_store,
};

static void himax_me372cl_chip_create_proc_flash_dump(void)
{
	himax_me372cl_proc_flash_dump =
	    proc_create(HIMAX_ME372CL_PROC_FLASH_DUMP, 0666, NULL, &flash_fops);
	if (himax_me372cl_proc_flash_dump) {
	} else {
		printk(KERN_ERR
		       "[Himax_ME372CL] %s: proc debug level create failed!\n",
		       __func__);
	}
}

static void himax_me372cl_chip_remove_proc_flash_dump(void)
{
	extern struct proc_dir_entry proc_root;
	printk(KERN_ERR "[Himax_ME372CL]:himax_chip_remove_proc_flash_dump\n");
	remove_proc_entry(HIMAX_ME372CL_PROC_FLASH_DUMP, &proc_root);
}

/*----[HX_TP_SYS_FLASH_DUMP]--------end */

/*----[HX_TP_SYS_SELF_TEST]---------start */
#ifdef HX_TP_SYS_SELF_TEST

/*add by josh setting self_test config */
/*
 * time : t:x12 (12s)
 * fe96 : w:x96:x03:x02......
 */
static ssize_t himax_self_test_setting(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	char buf_tmp[6], length = 0;
	uint8_t veriLen = 0;
	uint8_t write_da[100];
	unsigned long result = 0;
	static uint8_t himax_command;

	memset(buf_tmp, 0x0, sizeof(buf_tmp));
	memset(write_da, 0x0, sizeof(write_da));
	if (buf[0] == 't' && buf[1] == ':' && buf[2] == 'x') {
		if (buf[3] > 47 && buf[3] < 58 && buf[4] > 47 && buf[4] < 58) {
			self_test_delay_time = (buf[3] - 48) * 10 + buf[4] - 48;
			printk(KERN_INFO "self_test_delay_time: %d",
			       self_test_delay_time);
		}
		return count;
	}

	if (buf[0] == 'w' && buf[1] == ':') {
		if (buf[2] == 'x') {
			uint8_t loop_i;
			uint16_t base = 5;
			memcpy(buf_tmp, buf + 3, 2);
			if (!strict_strtoul(buf_tmp, 16, &result))
				himax_command = result;
			for (loop_i = 0; loop_i < 100; loop_i++) {
				if (buf[base] == '\n') {
					if (buf[0] == 'w')

						printk(KERN_INFO
						       "CMD: %x, %x, %d\n",
						       himax_command,
						       write_da[0], length);
					for (veriLen = 0; veriLen < length;
					     veriLen++) {
						printk(KERN_INFO "%x ",
						       *((&write_da[0]) +
							 veriLen));
						rFE96_setting[veriLen] =
						    *((&write_da[0]) + veriLen);
						printk(KERN_INFO
						       "rFE96_setting[%d] : %x \n",
						       veriLen,
						       rFE96_setting[veriLen]);
					}

					printk(KERN_INFO "\n");
					return count;
				}
				if (buf[base + 1] == 'x') {
					buf_tmp[4] = '\n';
					buf_tmp[5] = '\0';
					memcpy(buf_tmp, buf + base + 2, 2);
					if (!strict_strtoul
					    (buf_tmp, 16, &result))
						write_da[loop_i] = result;
					length++;
				}
				base += 4;
			}
		}
	}

	return count;
}

static ssize_t himax_chip_self_test_function(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	int val = 0x00;
	uint8_t data[16];

	val = himax_chip_self_test(data);
	if (val == 0) {
		return sprintf(buf, "0\n");
	} else {
		return sprintf(buf, "1\n");
	}
}

static int himax_chip_self_test(uint8_t *data)
{
	uint8_t cmdbuf[11];
	int ret = 0;
	uint8_t valuebuf[16];
	int i = 0, pf_value = 0x00, err = 1;

	/*----[HX_RST_PIN_FUNC]-------start */
#ifdef HX_RST_PIN_FUNC
	himax_hw_reset();
#endif
	/*----[HX_RST_PIN_FUNC]-------end */

	err = himax_ts_poweron();
	if (err == 0) {
		printk("[Himax] %s: touch power on. \n", __func__);
	} else {
		printk(KERN_ERR "[Himax] %s: power on error = %d.\n", __func__,
		       err);
	}

	if (IC_TYPE == HX_85XX_C_SERIES_PWON) {
		/*sense off to write self-test parameters */
		cmdbuf[0] = HX_CMD_TSSOFF;
		ret =
		    i2c_himax_master_write(himax_data->client, cmdbuf, 1,
					   DEFAULT_RETRY_CNT);
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax]: write TSSOFF failed line: %d \n",
			       __LINE__);
		}
		mdelay(120);	/*120ms */

		cmdbuf[0] = HX_CMD_SELFTEST_BUFFER;
		cmdbuf[1] = 0xB4;	/*180 */
		cmdbuf[2] = 0x64;	/*100 */
		cmdbuf[3] = 0x36;
		cmdbuf[4] = 0x09;
		cmdbuf[5] = 0x2D;
		cmdbuf[6] = 0x09;
		cmdbuf[7] = 0x32;
		cmdbuf[8] = 0x08;	/*0x19 */
		ret =
		    i2c_himax_master_write(himax_data->client, cmdbuf, 9,
					   DEFAULT_RETRY_CNT);
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax]: write HX_CMD_SELFTEST_BUFFER failed line: %d \n",
			       __LINE__);
		}

		udelay(100);

		ret =
		    i2c_himax_read(himax_data->client, HX_CMD_SELFTEST_BUFFER,
				   valuebuf, 9, DEFAULT_RETRY_CNT);
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax]: read HX_CMD_SELFTEST_BUFFER failed line: %d \n",
			       __LINE__);
		}
		printk("[Himax]: 0x8D[0] = 0x%x\n", valuebuf[0]);
		printk("[Himax]: 0x8D[1] = 0x%x\n", valuebuf[1]);
		printk("[Himax]: 0x8D[2] = 0x%x\n", valuebuf[2]);
		printk("[Himax]: 0x8D[3] = 0x%x\n", valuebuf[3]);
		printk("[Himax]: 0x8D[4] = 0x%x\n", valuebuf[4]);
		printk("[Himax]: 0x8D[5] = 0x%x\n", valuebuf[5]);
		printk("[Himax]: 0x8D[6] = 0x%x\n", valuebuf[6]);
		printk("[Himax]: 0x8D[7] = 0x%x\n", valuebuf[7]);
		printk("[Himax]: 0x8D[8] = 0x%x\n", valuebuf[8]);

		cmdbuf[0] = 0xE9;
		cmdbuf[1] = 0x00;
		cmdbuf[2] = 0x06;
		ret = i2c_himax_master_write(himax_data->client, cmdbuf, 3, DEFAULT_RETRY_CNT);	/*write sernsor to self-test mode */
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax]: write sernsor to self-test mode failed line: %d \n",
			       __LINE__);
		}
		udelay(100);

		ret =
		    i2c_himax_read(himax_data->client, 0xE9, valuebuf, 3,
				   DEFAULT_RETRY_CNT);
		if (ret < 0) {
			printk(KERN_ERR "[Himax]: read 0xE9 failed line: %d \n",
			       __LINE__);
		}
		printk("[Himax]: 0xE9[0] = 0x%x\n", valuebuf[0]);
		printk("[Himax]: 0xE9[1] = 0x%x\n", valuebuf[1]);
		printk("[Himax]: 0xE9[2] = 0x%x\n", valuebuf[2]);

		cmdbuf[0] = HX_CMD_TSSON;
		ret = i2c_himax_master_write(himax_data->client, cmdbuf, 1, DEFAULT_RETRY_CNT);	/*sense on */
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax]: write HX_CMD_TSSON failed line: %d \n",
			       __LINE__);
		}
		mdelay(1500);	/*1500ms */

		cmdbuf[0] = HX_CMD_TSSOFF;
		ret =
		    i2c_himax_master_write(himax_data->client, cmdbuf, 1,
					   DEFAULT_RETRY_CNT);
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax]: write TSSOFF failed line: %d \n",
			       __LINE__);
		}
		mdelay(120);	/*120ms */

		memset(valuebuf, 0x00, 16);
		ret =
		    i2c_himax_read(himax_data->client, HX_CMD_SELFTEST_BUFFER,
				   valuebuf, 16, DEFAULT_RETRY_CNT);

		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax]: read HX_CMD_FW_VERSION_ID failed line: %d \n",
			       __LINE__);
		} else {
			if (valuebuf[0] == 0xAA) {
				printk("[Himax]: self-test pass\n");
				pf_value = 0x0;
			} else {
				printk("[Himax]: self-test fail\n");
				pf_value = 0x1;
				for (i = 0; i < 16; i++) {
					printk
					    ("[Himax]: 0x8D buff[%d] = 0x%x\n",
					     i, valuebuf[i]);
				}
			}
		}
		mdelay(120);	/*120ms */

		cmdbuf[0] = 0xE9;
		cmdbuf[1] = 0x00;
		cmdbuf[2] = 0x00;
		ret = i2c_himax_master_write(himax_data->client, cmdbuf, 3, DEFAULT_RETRY_CNT);	/*write sensor to normal mode */
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax]: write sensor to normal mode failed line: %d \n",
			       __LINE__);
		}
		mdelay(120);	/*120ms */

		cmdbuf[0] = HX_CMD_TSSON;
		ret = i2c_himax_master_write(himax_data->client, cmdbuf, 1, DEFAULT_RETRY_CNT);	/*sense on */
		if (ret < 0) {
			printk(KERN_ERR
			       "[Himax]: write HX_CMD_TSSON failed line: %d \n",
			       __LINE__);
		}
		msleep(120);	/*120ms */
	} else if (IC_TYPE == HX_85XX_D_SERIES_PWON) {
		/*Step 0 : sensor off */
		i2c_himax_write(himax_data->client, 0x82, &cmdbuf[0], 0,
				DEFAULT_RETRY_CNT);
		msleep(120);

		/*Step 1 : Close Re-Calibration FE02 */
		/*-->Read 0xFE02 */
		cmdbuf[0] = 0x15;
		i2c_himax_write(himax_data->client, 0xE1, &cmdbuf[0], 1,
				DEFAULT_RETRY_CNT);
		msleep(10);

		cmdbuf[0] = 0x00;
		cmdbuf[1] = 0x02;	/*FE02  */
		i2c_himax_write(himax_data->client, 0xD8, &cmdbuf[0], 2,
				DEFAULT_RETRY_CNT);
		msleep(10);

		i2c_himax_read(himax_data->client, 0x5A, valuebuf, 2,
			       DEFAULT_RETRY_CNT);
		msleep(10);

		cmdbuf[0] = 0x00;
		i2c_himax_write(himax_data->client, 0xE1, &cmdbuf[0], 1,
				DEFAULT_RETRY_CNT);

		msleep(30);

		printk("[Himax]: 0xFE02_0 = 0x%x\n", valuebuf[0]);
		printk("[Himax]: 0xFE02_1 = 0x%x\n", valuebuf[1]);

		valuebuf[0] = valuebuf[1] & 0xFD;	/* close re-calibration  , shift first byte of config bank register read issue. */

		printk("[Himax]: 0xFE02_valuebuf = 0x%x\n", valuebuf[0]);

		/*-->Write 0xFE02 */
		cmdbuf[0] = 0x15;
		i2c_himax_write(himax_data->client, 0xE1, &cmdbuf[0], 1,
				DEFAULT_RETRY_CNT);
		msleep(10);

		cmdbuf[0] = 0x00;
		cmdbuf[1] = 0x02;	/*FE02  */
		i2c_himax_write(himax_data->client, 0xD8, &cmdbuf[0], 2,
				DEFAULT_RETRY_CNT);
		msleep(10);

		cmdbuf[0] = valuebuf[0];
		i2c_himax_write(himax_data->client, 0x40, &cmdbuf[0], 1,
				DEFAULT_RETRY_CNT);
		msleep(10);

		cmdbuf[0] = 0x00;
		i2c_himax_write(himax_data->client, 0xE1, &cmdbuf[0], 1,
				DEFAULT_RETRY_CNT);

		msleep(30);
		/*0xFE02 Read Back */

		/*-->Read 0xFE02 */
		cmdbuf[0] = 0x15;
		i2c_himax_write(himax_data->client, 0xE1, &cmdbuf[0], 1,
				DEFAULT_RETRY_CNT);
		msleep(10);

		cmdbuf[0] = 0x00;
		cmdbuf[1] = 0x02;	/*FE02  */
		i2c_himax_write(himax_data->client, 0xD8, &cmdbuf[0], 2,
				DEFAULT_RETRY_CNT);
		msleep(10);

		i2c_himax_read(himax_data->client, 0x5A, valuebuf, 2,
			       DEFAULT_RETRY_CNT);
		msleep(10);

		cmdbuf[0] = 0x00;
		i2c_himax_write(himax_data->client, 0xE1, &cmdbuf[0], 1,
				DEFAULT_RETRY_CNT);
		msleep(30);

		printk("[Himax]: 0xFE02_0_back = 0x%x\n", valuebuf[0]);
		printk("[Himax]: 0xFE02_1_back = 0x%x\n", valuebuf[1]);

		/*Step 2 : Close Flash-Reload */
		cmdbuf[0] = 0x00;
		i2c_himax_write(himax_data->client, 0xE3, &cmdbuf[0], 1,
				DEFAULT_RETRY_CNT);

		msleep(30);

		i2c_himax_read(himax_data->client, 0xE3, valuebuf, 1,
			       DEFAULT_RETRY_CNT);

		printk("[Himax]: 0xE3_back = 0x%x\n", valuebuf[0]);

		/*Step 4 : Write self_test parameter to FE96~FE9D */
		/*-->Write FE96~FE9D */

		/*SelfTest Parameter load from config */
		cmdbuf[0] = 0x15;
		i2c_himax_write(himax_data->client, 0xE1, &cmdbuf[0], 1,
				DEFAULT_RETRY_CNT);
		msleep(10);

		cmdbuf[0] = 0x00;
		cmdbuf[1] = 0x96;	/*FE96  */
		i2c_himax_write(himax_data->client, 0xD8, &cmdbuf[0], 2,
				DEFAULT_RETRY_CNT);
		msleep(10);

		/*-->Modify the initial value of self_test. */
		cmdbuf[0] = rFE96_setting[0];
		cmdbuf[1] = rFE96_setting[1];
		cmdbuf[2] = rFE96_setting[2];
		cmdbuf[3] = rFE96_setting[3];
		cmdbuf[4] = rFE96_setting[4];
		cmdbuf[5] = rFE96_setting[5];
		cmdbuf[6] = rFE96_setting[6];
		cmdbuf[7] = rFE96_setting[7];
		i2c_himax_write(himax_data->client, 0x40, &cmdbuf[0], 8,
				DEFAULT_RETRY_CNT);
		msleep(10);

		cmdbuf[0] = 0x00;
		i2c_himax_write(himax_data->client, 0xE1, &cmdbuf[0], 1,
				DEFAULT_RETRY_CNT);

		msleep(30);

		/*Read back */
		cmdbuf[0] = 0x15;
		i2c_himax_write(himax_data->client, 0xE1, &cmdbuf[0], 1,
				DEFAULT_RETRY_CNT);
		msleep(10);

		cmdbuf[0] = 0x00;
		cmdbuf[1] = 0x96;	/*FE96 */
		i2c_himax_write(himax_data->client, 0xD8, &cmdbuf[0], 2,
				DEFAULT_RETRY_CNT);
		msleep(10);

		i2c_himax_read(himax_data->client, 0x5A, valuebuf, 16,
			       DEFAULT_RETRY_CNT);
		msleep(10);

		cmdbuf[0] = 0x00;
		i2c_himax_write(himax_data->client, 0xE1, &cmdbuf[0], 1,
				DEFAULT_RETRY_CNT);

		for (i = 1; i < 16; i++) {
			printk("[Himax]: 0xFE96 buff_back[%d] = 0x%x\n", i,
			       valuebuf[i]);
		}

		msleep(30);

		/*Step 5 : Enter self_test mode */
		cmdbuf[0] = 0x16;
		i2c_himax_write(himax_data->client, 0x91, &cmdbuf[0], 1,
				DEFAULT_RETRY_CNT);

		i2c_himax_read(himax_data->client, 0x91, valuebuf, 1,
			       DEFAULT_RETRY_CNT);

		printk("[Himax]: 0x91_back = 0x%x\n", valuebuf[0]);
		msleep(10);

		/*Step 6 : Sensor On */
		i2c_himax_write(himax_data->client, 0x83, &cmdbuf[0], 0,
				DEFAULT_RETRY_CNT);

		mdelay(self_test_delay_time * 1000);

		/*Step 7 : Sensor Off */
		i2c_himax_write(himax_data->client, 0x82, &cmdbuf[0], 0,
				DEFAULT_RETRY_CNT);

		msleep(30);

		/*Step 8 : Get self_test result */
		cmdbuf[0] = 0x15;
		i2c_himax_write(himax_data->client, 0xE1, &cmdbuf[0], 1,
				DEFAULT_RETRY_CNT);
		msleep(10);

		cmdbuf[0] = 0x00;
		cmdbuf[1] = 0x96;	/*FE96  */
		i2c_himax_write(himax_data->client, 0xD8, &cmdbuf[0], 2,
				DEFAULT_RETRY_CNT);
		msleep(10);

		i2c_himax_read(himax_data->client, 0x5A, valuebuf, 16,
			       DEFAULT_RETRY_CNT);
		msleep(10);

		cmdbuf[0] = 0x00;
		i2c_himax_write(himax_data->client, 0xE1, &cmdbuf[0], 1,
				DEFAULT_RETRY_CNT);

		/*Final : Leave self_test mode */
		cmdbuf[0] = 0x00;
		i2c_himax_write(himax_data->client, 0x91, &cmdbuf[0], 1,
				DEFAULT_RETRY_CNT);

		if (valuebuf[1] == 0xAA) {	/*get the self_test result , shift first byte for config bank read issue. */
			printk("[Himax]: self-test pass\n");
			pf_value = 0x0;

			/*Show pass log */
			for (i = 1; i < 7; i++) {
				printk("[Himax]: 0xFE96 buff[%d] = 0x%x\n", i,
				       valuebuf[i]);
			}
		} else {
			printk("[Himax]: self-test fail\n");
			pf_value = 0x1;
			for (i = 1; i < 16; i++) {
				data[i] = valuebuf[i];
				printk("[Himax]: 0xFE96 buff[%d] = 0x%x\n", i,
				       valuebuf[i]);
			}
		}

		/*HW reset and power on again. */
		/*----[HX_RST_PIN_FUNC]----------start */
#ifdef HX_RST_PIN_FUNC
		himax_hw_reset();
#endif
		/*----[HX_RST_PIN_FUNC]----------end */
		err = himax_ts_poweron();
		if (err == 0) {
			printk("[Himax] %s: touch power on. \n", __func__);
		} else {
			printk(KERN_ERR "[Himax] %s: power on error = %d.\n",
			       __func__, err);
		}
	}
	return pf_value;
}

#endif
/*----[HX_TP_SYS_SELF_TEST]--------end */

/*----[HX_TP_SYS_HITOUCH]----------start */
#ifdef HX_TP_SYS_HITOUCH
static ssize_t himax_hitouch_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret = 0;

	if (hitouch_command == 0) {
		ret += sprintf(buf + ret, "Driver Version:2.0 \n");
	}

	return ret;
}

/*-----------------------------------
//himax_hitouch_store
//command 0 : Get Driver Version
//command 1 : Hitouch Connect
//command 2 : Hitouch Disconnect
//----------------------------------- */
static ssize_t himax_hitouch_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	if (buf[0] == '0') {
		hitouch_command = 0;
	} else if (buf[0] == '1') {
		hitouch_is_connect = true;
		printk("[Himax]: hitouch_is_connect = true\n");
	} else if (buf[0] == '2') {
		hitouch_is_connect = false;
		printk("[Himax]: hitouch_is_connect = false\n");
	}
	return count;
}
#endif
/*----[HX_TP_SYS_HITOUCH]-------end */

/*----[HX_TP_SYS_RESET]---------start */
#ifdef HX_TP_SYS_RESET
static ssize_t himax_reset_set(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	/*----[ENABLE_CHIP_RESET_MACHINE]----------start */
#ifdef ENABLE_CHIP_RESET_MACHINE
	if (himax_data->init_success) {
		queue_delayed_work(himax_data->himax_wq,
				   &himax_data->himax_chip_reset_work, 0);
	}
#endif
	/*----[ENABLE_CHIP_RESET_MACHINE]----------end */

	return count;
}

#endif
/*----[HX_TP_SYS_RESET]-------end	 */

static int himax_touch_sysfs_init(void)
{
	int ret;

	android_touch_kobj = kobject_create_and_add("android_touch", NULL);
	if (android_touch_kobj == NULL) {
		printk(KERN_ERR "[Himax]: subsystem_register failed\n");
		ret = -ENOMEM;
		return ret;
	}
#ifdef HX_TP_SYS_DEBUG_LEVEL
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_debug_level.attr);
	if (ret) {
		printk(KERN_ERR "[Himax]: create_file debug_level failed\n");
		return ret;
	}
#endif

#ifdef HX_TP_SYS_REGISTER
	register_command = 0;
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_register.attr);
	if (ret) {
		printk(KERN_ERR "[Himax]: create_file register failed\n");
		return ret;
	}
#endif

#ifdef HX_TP_SYS_DIAG
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_diag.attr);
	if (ret) {
		printk(KERN_ERR "[Himax]: sysfs_create_file failed\n");
		return ret;
	}
#endif

#ifdef HX_TP_SYS_SELF_TEST
	ret =
	    sysfs_create_file(android_touch_kobj, &dev_attr_tp_self_test.attr);
	if (ret) {
		printk(KERN_ERR
		       "[Himax]: sysfs_create_file dev_attr_tp_self_test failed\n");
		return ret;
	}
#endif

#ifdef HX_TP_SYS_DIAG
	ret =
	    sysfs_create_file(android_touch_kobj,
			      &dev_attr_tp_output_raw_data.attr);
	if (ret) {
		printk(KERN_ERR
		       "[Himax]: sysfs_create_file dev_attr_tp_output_raw_data failed\n");
		return ret;
	}
#endif

#ifdef HX_TP_SYS_FLASH_DUMP
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_flash_dump.attr);
	if (ret) {
		printk(KERN_ERR "[Himax]: sysfs_create_file failed\n");
		return ret;
	}
#endif

#ifdef HX_TP_SYS_HITOUCH
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_hitouch.attr);
	if (ret) {
		printk(KERN_ERR "[Himax]: sysfs_create_file failed\n");
		return ret;
	}
#endif

#ifdef HX_TP_SYS_RESET
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_reset.attr);
	if (ret) {
		printk(KERN_ERR "[Himax]: sysfs_create_file failed\n");
		return ret;
	}
#endif
	/*add by Josh ++++++ */
	ret =
	    sysfs_create_file(android_touch_kobj, &dev_attr_touch_status.attr);
	if (ret) {
		printk(KERN_ERR
		       "[Himax]: sysfs_create_file dev_attr_touch_status failed\n");
		return ret;
	}
	ret =
	    sysfs_create_file(android_touch_kobj,
			      &dev_attr_tp_check_running.attr);
	if (ret) {
		printk(KERN_ERR
		       "[Himax]: sysfs_create_file dev_attr_tp_check_running failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_touch_irq.attr);
	if (ret) {
		printk(KERN_ERR
		       "[Himax]: sysfs_create_file dev_attr_touch_irq failed\n");
		return ret;
	}
	ret =
	    sysfs_create_file(android_touch_kobj, &dev_attr_tp_fw_version.attr);
	if (ret) {
		printk(KERN_ERR
		       "[Himax]: sysfs_create_file dev_attr_tp_fw_version failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_debug_log.attr);
	if (ret) {
		printk(KERN_ERR
		       "[Himax]: sysfs_create_file dev_attr_debug_log failed\n");
		return ret;
	}

	ret =
	    sysfs_create_file(android_touch_kobj,
			      &dev_attr_himax_check_sum.attr);
	if (ret) {
		printk(KERN_ERR
		       "[Himax]: sysfs_create_file dev_attr_himax_check_sum failed\n");
		return ret;
	}
	/*add by Josh ------- */
	return 0;
}

static void himax_touch_sysfs_deinit(void)
{
#ifdef HX_TP_SYS_DIAG
	sysfs_remove_file(android_touch_kobj, &dev_attr_diag.attr);
#endif

#ifdef HX_TP_SYS_DEBUG_LEVEL
	sysfs_remove_file(android_touch_kobj, &dev_attr_debug_level.attr);
#endif

#ifdef HX_TP_SYS_REGISTER
	sysfs_remove_file(android_touch_kobj, &dev_attr_register.attr);
#endif

#ifdef HX_TP_SYS_SELF_TEST
	sysfs_remove_file(android_touch_kobj, &dev_attr_tp_self_test.attr);
#endif

#ifdef HX_TP_SYS_DIAG
	sysfs_remove_file(android_touch_kobj,
			  &dev_attr_tp_output_raw_data.attr);
#endif

#ifdef HX_TP_SYS_RESET
	sysfs_remove_file(android_touch_kobj, &dev_attr_reset.attr);
#endif

	/*add by Josh ++++++ */
	/*sysfs_remove_file(android_touch_kobj, &dev_attr_touch_status.attr); */

	sysfs_remove_file(android_touch_kobj, &dev_attr_tp_check_running.attr);

	sysfs_remove_file(android_touch_kobj, &dev_attr_touch_irq.attr);

	sysfs_remove_file(android_touch_kobj, &dev_attr_tp_fw_version.attr);

	sysfs_remove_file(android_touch_kobj, &dev_attr_debug_log.attr);

	sysfs_remove_file(android_touch_kobj, &dev_attr_himax_check_sum.attr);
	/*add by Josh ------- */

	kobject_del(android_touch_kobj);
}

/*====================================================
//
//	Segment : Upgrade firmware from kernel image
//	By Josh
//==================================================== */
static int update_touch_progress(int update_progress)
{
	if (himax_data->AP_progress < update_progress || update_progress == 0) {
		printk("[Himax] %s: has processed %d.  \n", __func__,
		       update_progress);
		himax_data->AP_progress = update_progress;
		mm_segment_t oldfs;
		char Progress_file_path[] = "/data/touch_update_progress";
		struct file *filePtr = NULL;
		int len = 0;
		loff_t pos = 0;
		char temp_progress[3];

		sprintf(temp_progress, "%d", update_progress);
		/*printk("[Himax] %s: write %d done. \n", __func__, update_progress); */
		filePtr =
		    filp_open(Progress_file_path, O_RDWR | O_CREAT,
			      (S_IWUSR | S_IRUGO));
		if (!IS_ERR_OR_NULL(filePtr)) {
			oldfs = get_fs();
			set_fs(get_ds());
			pos = 0;
			len =
			    filePtr->f_op->write(filePtr, &temp_progress,
						 sizeof(temp_progress), &pos);
			set_fs(oldfs);
			filp_close(filePtr, NULL);
			/*printk("[Himax] %s: write %s done. \n", __func__, Progress_file_path); */
			return 0;
		} else if (PTR_ERR(filePtr) == -ENOENT) {
			printk("[Himax] %s: %s not found\n", __func__,
			       Progress_file_path);
			return 1;
		} else {
			printk("[Himax] %s: %s open error\n", __func__,
			       Progress_file_path);
			return 1;
		}
	}
}

static int update_touch_result(int result)
{
	mm_segment_t oldfs_result;
	char result_state_path[] = "/data/touch_upfw_result";
	struct file *resultfilePtr = NULL;
	loff_t pos = 0;
	int len = 0;

	resultfilePtr =
	    filp_open(result_state_path, O_RDWR | O_CREAT, (S_IWUSR | S_IRUGO));
	if (!IS_ERR_OR_NULL(resultfilePtr)) {
		oldfs_result = get_fs();
		set_fs(get_ds());
		pos = 0;
		len =
		    resultfilePtr->f_op->write(resultfilePtr, result,
					       sizeof(char), &pos);
		set_fs(oldfs_result);
		filp_close(resultfilePtr, NULL);
		/*printk("[Himax] %s: write %s done. \n", __func__, result_state_path); */
		return 0;
	} else if (PTR_ERR(resultfilePtr) == -ENOENT) {
		printk("[Himax] %s: %s not found\n", __func__,
		       result_state_path);
		return 1;
	} else {
		printk("[Himax] %s: %s open error\n", __func__,
		       result_state_path);
		return 1;
	}
}

static int himax_firmware_check(void)
{
	struct file *filp = NULL;
	mm_segment_t oldfs;
	int result = 0;
	char upgrade_fw_ver[8];
	int upgrade_firmware_checksum = 0;
	char upgrade_config_ver[8];
	int upgrade_config_checksum = 0;
	int i = 0, open_fail = 1;

	for (i = 1; i <= 3; i++) {
		printk
		    ("[Himax] %s: file open:/system/etc/firmware/touch_fw_d48.bin. \n",
		     __func__);
		filp =
		    filp_open("/system/etc/firmware/touch_fw_d48.bin", O_RDONLY,
			      0);
		if (!IS_ERR_OR_NULL(filp)) {
			oldfs = get_fs();
			set_fs(get_ds());

			result =
			    filp->f_op->read(filp, upgrade_fw,
					     sizeof(upgrade_fw), &filp->f_pos);
			if (result < 0) {
				printk
				    ("[Himax] %s: read firmware file failed\n",
				     __func__);
			} else {
				open_fail = 0;
				i = 4;
				printk("[Himax] %s: read firmware file done\n",
				       __func__);
			}

			set_fs(oldfs);
			filp_close(filp, NULL);
		} else {
			printk("[Himax] %s: open firmware file failed\n",
			       __func__);
			open_fail++;
			msleep(5000);
		}

		if (open_fail == 3) {
			printk("[Himax] %s: open firmware file retry failed.\n",
			       __func__);
			return 2;
		}
	}

	printk
	    ("[Himax] %s: upgrade bin files start len %d: %02X, %02X, %02X, %02X.\n",
	     __func__, result, upgrade_fw[0], upgrade_fw[1], upgrade_fw[2],
	     upgrade_fw[3]);
	printk("[Himax] %s: upgrade bin files version: %02X %02X.\n", __func__,
	       upgrade_fw[0x0085], upgrade_fw[0x0086]);
	printk("[Himax] %s: upgrade bin files config version: %02X.\n",
	       __func__, upgrade_fw[0x00B7]);
	printk
	    ("[Himax] %s: upgrade bin files last 4 byte: %02X, %02X, %02X, %02X.\n",
	     __func__, upgrade_fw[0x7FFC], upgrade_fw[0x7FFD],
	     upgrade_fw[0x7FFE], upgrade_fw[0x7FFF]);

	sprintf(upgrade_fw_ver, "%2.2X%2.2X", upgrade_fw[0x0085],
		upgrade_fw[0x0086]);
	/*printk("[Himax] %s: upgrade_tp_version: %02X, %02X, %02X, %02X.\n", __func__, upgrade_fw_ver[0], upgrade_fw_ver[1], upgrade_fw_ver[2], upgrade_fw_ver[3]); */
	upgrade_firmware_checksum =
	    ((upgrade_fw_ver[0] << 24) | (upgrade_fw_ver[1] << 16) |
	     (upgrade_fw_ver[2] << 8) | upgrade_fw_ver[3]);

	printk("[Himax]: upgrade firmware verison = %s.\n", upgrade_fw_ver);
	printk("[Himax]: upgrade firmware verison checksum = %d.\n",
	       upgrade_firmware_checksum);

	sprintf(upgrade_config_ver, "%2.2X", upgrade_fw[0x00B7]);
	/*printk("[Himax] %s: upgrade_config_ver: %02X, %02X.\n", __func__, upgrade_config_ver[0], upgrade_config_ver[1]); */
	upgrade_config_checksum =
	    ((upgrade_config_ver[0] << 8) | upgrade_config_ver[1]);
	printk("[Himax]: bin file config verison = %s.\n", upgrade_config_ver);
	printk("[Himax]: bin file config verison checksum = %d.\n",
	       upgrade_config_checksum);

	if (upgrade_firmware_checksum > himax_data->firmware_version_checksum) {
		printk("[Himax]: need upgrade touch firmware.\n");
		return 0;
	} else if (upgrade_firmware_checksum ==
		   himax_data->firmware_version_checksum) {
		if (upgrade_config_checksum >
		    himax_data->config_version_checksum) {
			printk("[Himax]: need upgrade touch config.\n");
			return 0;
		} else {
			printk
			    ("[Himax]: not need upgrade touch firmware and config.\n");
			return 1;
		}
	} else {
		printk
		    ("[Himax]: not need upgrade touch firmware and config.\n");
		return 1;
	}

}

static int himax_firmware_upgrade(int path)
{
	struct file *filp = NULL;
	mm_segment_t oldfs;
	int result = 0;
	char upgrade_fw_ver[8];
	int upgrade_firmware_checksum = 0;
	char upgrade_config_ver[8];
	int upgrade_config_checksum = 0;
	int i = 0, open_fail = 0;
	int upgrade_status = 0;

	update_touch_progress(0);
	/*----[ENABLE_CHIP_STATUS_MONITOR]---------start	 */
#ifdef ENABLE_CHIP_STATUS_MONITOR
	cancel_delayed_work_sync(&himax_data->himax_chip_monitor);
#endif
	/*----[ENABLE_CHIP_STATUS_MONITOR]---------end	 */

	himax_data->tp_firmware_upgrade_proceed = 1;

	wake_lock(&himax_data->wake_lock);
	mutex_lock(&himax_data->mutex_lock);

	update_touch_result(TOUCH_FW_UPGRADE_INIT);
	update_touch_progress(5);

	fw_update_complete = false;
	if ((path == SELF) || (path == AP)) {
		printk
		    ("[Himax] %s: file open:/system/etc/firmware/touch_fw_d48.bin. \n",
		     __func__);
		for (i = 1; i <= 3; i++) {
			filp =
			    filp_open("/system/etc/firmware/touch_fw_d48.bin",
				      O_RDONLY, 0);
			if (!IS_ERR_OR_NULL(filp)) {
				oldfs = get_fs();
				set_fs(get_ds());

				result =
				    filp->f_op->read(filp, upgrade_fw,
						     sizeof(upgrade_fw),
						     &filp->f_pos);
				if (result < 0) {
					printk
					    ("[Himax] %s: read firmware file failed\n",
					     __func__);
				} else {
					open_fail = 0;
					i = 4;
					printk
					    ("[Himax] %s: read firmware file done\n",
					     __func__);
				}

				set_fs(oldfs);
				filp_close(filp, NULL);
			} else {
				printk
				    ("[Himax] %s: open firmware file failed\n",
				     __func__);
				open_fail++;
				msleep(5000);
			}

			if (open_fail == 3) {
				printk
				    ("[Himax] %s: open firmware file retry failed.\n",
				     __func__);
				goto firmware_upgrade_fail;
			}
		}

	} else if (path == LOCAL) {
		printk("[Himax] %s: file open:/data/local/fw.bin. \n",
		       __func__);
		for (i = 1; i <= 3; i++) {
			filp = filp_open("/data/local/fw.bin", O_RDONLY, 0);
			if (!IS_ERR_OR_NULL(filp)) {
				oldfs = get_fs();
				set_fs(get_ds());

				result =
				    filp->f_op->read(filp, upgrade_fw,
						     sizeof(upgrade_fw),
						     &filp->f_pos);
				if (result < 0) {
					printk
					    ("[Himax] %s: read firmware file failed.\n",
					     __func__);
				} else {
					open_fail = 0;
					i = 4;
					printk
					    ("[Himax] %s: read firmware file done.\n",
					     __func__);
				}

				set_fs(oldfs);
				filp_close(filp, NULL);
			} else {
				printk
				    ("[Himax] %s: open firmware file failed.\n",
				     __func__);
				open_fail++;
				msleep(5000);
			}

			if (open_fail == 3) {
				printk
				    ("[Himax] %s: open firmware file retry failed.\n",
				     __func__);
				goto firmware_upgrade_fail;
			}
		}
	} else {
		printk("[Himax] %s: touch firmware path \n", __func__);
		goto firmware_upgrade_fail;
	}

	printk
	    ("[Himax] %s: upgrade bin files start len %d: %02X, %02X, %02X, %02X.\n",
	     __func__, result, upgrade_fw[0], upgrade_fw[1], upgrade_fw[2],
	     upgrade_fw[3]);
	printk("[Himax] %s: upgrade bin files version: %02X %02X.\n", __func__,
	       upgrade_fw[0x0085], upgrade_fw[0x0086]);
	printk("[Himax] %s: upgrade bin files config version: %02X.\n",
	       __func__, upgrade_fw[0x00B7]);
	printk
	    ("[Himax] %s: upgrade bin files last 4 byte: %02X, %02X, %02X, %02X.\n",
	     __func__, upgrade_fw[0x7FFC], upgrade_fw[0x7FFD],
	     upgrade_fw[0x7FFE], upgrade_fw[0x7FFF]);

	sprintf(upgrade_fw_ver, "%2.2X%2.2X", upgrade_fw[0x0085],
		upgrade_fw[0x0086]);
	/*printk("[Himax] %s: upgrade_tp_version: %02X, %02X, %02X, %02X.\n", __func__, upgrade_fw_ver[0], upgrade_fw_ver[1], upgrade_fw_ver[2], upgrade_fw_ver[3]); */
	upgrade_firmware_checksum =
	    ((upgrade_fw_ver[0] << 24) | (upgrade_fw_ver[1] << 16) |
	     (upgrade_fw_ver[2] << 8) | upgrade_fw_ver[3]);

	printk("[Himax]: upgrade firmware verison = %s.\n", upgrade_fw_ver);
	printk("[Himax]: upgrade firmware verison checksum = %d.\n",
	       upgrade_firmware_checksum);

	sprintf(upgrade_config_ver, "%2.2X", upgrade_fw[0x00B7]);
	/*printk("[Himax] %s: upgrade_config_ver: %02X, %02X.\n", __func__, upgrade_config_ver[0], upgrade_config_ver[1]); */
	upgrade_config_checksum =
	    ((upgrade_config_ver[0] << 8) | upgrade_config_ver[1]);
	printk("[Himax]: bin file config verison = %s.\n", upgrade_config_ver);
	printk("[Himax]: bin file config verison checksum = %d.\n",
	       upgrade_config_checksum);

	update_touch_progress(10);
	update_touch_result(TOUCH_FW_UPGRADE_PROCESS);

	if (result > 0) {
		if (path == SELF) {
			if (upgrade_firmware_checksum >
			    himax_data->firmware_version_checksum) {
				printk
				    ("[Himax]: need upgrade touch firmware.\n");
				upgrade_status = 1;
			} else if (upgrade_firmware_checksum ==
				   himax_data->firmware_version_checksum) {
				if (upgrade_config_checksum >
				    himax_data->config_version_checksum) {
					printk
					    ("[Himax]: need upgrade touch config.\n");
					upgrade_status = 1;
				} else {
					printk
					    ("[Himax]: not need upgrade touch firmware and config.\n");
					upgrade_status = 0;
				}
			} else {
				printk
				    ("[Himax]: not need upgrade touch firmware and config.\n");
				upgrade_status = 0;
			}
		} else {
			upgrade_status = 1;
		}

		if (upgrade_status) {
			printk(KERN_INFO "[Himax] start to upgrade.\n");
			/* start to upgrade */
			disable_irq(himax_data->irq);
			if (fts_ctpm_fw_upgrade_with_sys_fs(upgrade_fw, result)
			    == 0) {
				printk(KERN_INFO
				       "[Himax] %s: ME372CL Himax hx8528-D48 chip upgrade error.\n",
				       __func__);
				fw_update_complete = false;
				update_touch_progress(90);
				goto firmware_upgrade_fail;
			} else {
				printk(KERN_INFO
				       "[Himax] %s: ME372CL Himax hx8528-D48 chip upgrade success. \n",
				       __func__);
				fw_update_complete = true;
				update_touch_result(TOUCH_FW_UPGRADE_SUCCESS);
				update_touch_progress(100);
				goto firmware_upgrade_finish;
			}
		} else {
			printk(KERN_INFO
			       "[Himax] %s: ME372CL Current touch firmware is the latest version.\n",
			       __func__);
		}
	} else {
		printk
		    ("[Himax] %s: ME372CL Himax hx8528-D48 chip not upgrade. \n",
		     __func__);
	}

firmware_upgrade_fail:
	update_touch_result(TOUCH_FW_UPGRADE_FAIL);

firmware_upgrade_finish:
	mutex_unlock(&himax_data->mutex_lock);
	wake_unlock(&himax_data->wake_lock);
	himax_data->tp_firmware_upgrade_proceed = 0;
	/*----[ENABLE_CHIP_RESET_MACHINE]--------------start	 */
	queue_delayed_work(himax_data->himax_wq,
			   &himax_data->himax_chip_reset_work, 0);
	/*----[ENABLE_CHIP_RESET_MACHINE]--------------end */

	/*----[ENABLE_CHIP_STATUS_MONITOR]-------------start	 */
#ifdef ENABLE_CHIP_STATUS_MONITOR
	queue_delayed_work(himax_data->himax_wq,
			   &himax_data->himax_chip_monitor, 10 * HZ);
#endif
	/*----[ENABLE_CHIP_STATUS_MONITOR]-------------end	 */

	return 0;
}

static int himax_chip_self_firmware_upgrade(struct work_struct *dat)
{
	printk("[Himax]:Start firmware upgrade.\n");
	/*add by Josh for read chip firmware +++ */

	/*add by Josh for read chip firmware --- */

	himax_firmware_upgrade(SELF);
}

/*==============================================
//
//	Segment : Himax Touch Work Function
//
//============================================== */

static void himax_ts_work_func(struct work_struct *work)
{
	int ret, i, temp1, temp2;
	unsigned int x = 0, y = 0, area = 0, press = 0;
	const unsigned int x_res = HX_X_RES;
	const unsigned int y_res = HX_Y_RES;
	/*      struct himax_ts_data *ts = container_of(work, struct himax_ts_data, work); */
	unsigned char check_sum_cal = 0;
	struct i2c_msg msg[2];
	uint8_t start_reg;
	uint8_t buf[128] = { 0 };
	int RawDataLen = 0;
	unsigned int temp_x[HX_MAX_PT], temp_y[HX_MAX_PT];

	/*----[HX_TP_SYS_DIAG]------------start */
#ifdef HX_TP_SYS_DIAG
	uint8_t *mutual_data;
	uint8_t *self_data;
	uint8_t diag_cmd;
	int mul_num;
	int self_num;
	int index = 0;

	/*coordinate dump start */
	char coordinate_char[15 + (HX_MAX_PT + 5) * 2 * 5 + 2];
	struct timeval t;
	struct tm broken;
	/*coordinate dump end */
#endif
	/*----[HX_TP_SYS_DIAG]------------end */

	/*Calculate the raw data length */
	/*Bizzy added for common RawData */
	int raw_cnt_max = HX_MAX_PT / 4;
	int raw_cnt_rmd = HX_MAX_PT % 4;
	int hx_touch_info_size;
	if (raw_cnt_rmd != 0x00) {	/*more than 4 fingers */
		if (IC_TYPE == HX_85XX_D_SERIES_PWON) {
			RawDataLen =
			    128 - ((HX_MAX_PT + raw_cnt_max + 3) * 4) - 1;
		} else {
			RawDataLen = 128 - ((HX_MAX_PT + raw_cnt_max + 3) * 4);
		}

		hx_touch_info_size = (HX_MAX_PT + raw_cnt_max + 2) * 4;
	} else {		/*less than 4 fingers */

		if (IC_TYPE == HX_85XX_D_SERIES_PWON) {
			RawDataLen =
			    128 - ((HX_MAX_PT + raw_cnt_max + 2) * 4) - 1;
		} else {
			RawDataLen = 128 - ((HX_MAX_PT + raw_cnt_max + 2) * 4);
		}

		hx_touch_info_size = (HX_MAX_PT + raw_cnt_max + 1) * 4;
	}

	/*----[ENABLE_CHIP_STATUS_MONITOR]-------start	 */
#ifdef ENABLE_CHIP_STATUS_MONITOR
	himax_data->running_status = 1;
	cancel_delayed_work_sync(&himax_data->himax_chip_monitor);
#endif
	/*----[ENABLE_CHIP_STATUS_MONITOR]-------end	 */

	start_reg = HX_CMD_RAE;
	msg[0].addr = himax_data->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &start_reg;

	msg[1].addr = himax_data->client->addr;
	msg[1].flags = I2C_M_RD;

#ifdef HX_TP_SYS_DIAG
	if (diag_command)	/*count the i2c read length */
#else
	if (false)
#endif
	{
#ifdef HX_TP_SYS_DIAG
		if (IC_TYPE == HX_85XX_D_SERIES_PWON) {
			msg[1].len = 128;	/*hx_touch_info_size + RawDataLen + 4 + 1;    //4: RawData Header */
		} else {
			msg[1].len = 128;	/*hx_touch_info_size + RawDataLen + 4;        //4: RawData Header */
		}
#else
		msg[1].len = hx_touch_info_size;
#endif
	} else {
		msg[1].len = hx_touch_info_size;
	}
	msg[1].buf = buf;

#ifdef HX_PORTING_DEB_MSG
	if (getDebugLevel() & 0x1) {
		printk("[Himax] %s: Touch Controller Trigger ISR enter.\n",
		       __func__);
	}
#endif

	/*Mutexlock Protect Start */
	mutex_lock(&himax_data->mutex_lock);
	/*Mutexlock Protect End */

	/*read 0x86 all event */
	ret = i2c_transfer(himax_data->client->adapter, msg, 2);
	if (ret < 0) {
		printk(KERN_INFO "[Himax] %s: i2c_transfer fail.\n", __func__);
		memset(buf, 0xff, 128);

		/*----[ENABLE_CHIP_RESET_MACHINE]----start */
#ifdef ENABLE_CHIP_RESET_MACHINE
		/*Mutexlock Protect Start */
		mutex_unlock(&himax_data->mutex_lock);
		/*Mutexlock Protect End */

		enable_irq(himax_data->irq);
		goto work_func_send_i2c_msg_fail;
#endif
		/*----[ENABLE_CHIP_RESET_MACHINE]----end */
	}

	/*----[HX_ESD_WORKAROUND]-----start */
#ifdef HX_ESD_WORKAROUND
	for (i = 0; i < hx_touch_info_size; i++) {
		if (buf[i] == 0x00) {
			check_sum_cal = 1;
		} else if (buf[i] == 0xED) {
			check_sum_cal = 2;
		} else {
			check_sum_cal = 0;
			i = hx_touch_info_size;
		}
	}

	/*IC status is abnormal ,do hand shaking */
	/*----[HX_TP_SYS_DIAG]-----start */
#ifdef HX_TP_SYS_DIAG
	diag_cmd = getDiagCommand();
#ifdef HX_ESD_WORKAROUND
	if (check_sum_cal != 0 && ESD_RESET_ACTIVATE == 0 && diag_cmd == 0)	/*ESD Check */
#else
	if (check_sum_cal != 0 && diag_cmd == 0)
#endif
#else
#ifdef HX_ESD_WORKAROUND
	if (check_sum_cal != 0 && ESD_RESET_ACTIVATE == 0)	/*ESD Check */
#else
	if (check_sum_cal != 0)
#endif
#endif
					/*----[HX_TP_SYS_DIAG]----end */
	{
		/*Mutexlock Protect Start */
		mutex_unlock(&himax_data->mutex_lock);
		/*Mutexlock Protect End */

		ret = himax_hang_shaking();	/*0:Running, 1:Stop, 2:I2C Fail */
		enable_irq(himax_data->irq);

		if (ret == 2) {
			goto work_func_send_i2c_msg_fail;
		}

		if ((ret == 1) && (check_sum_cal == 1)) {
			printk("[Himax]: ESD event checked - ALL Zero.\n");
			ESD_HW_REST();
		} else if (check_sum_cal == 2) {
			printk("[Himax]: ESD event checked - ALL 0xED.\n");
			ESD_HW_REST();
		}
					/*----[ENABLE_CHIP_STATUS_MONITOR]-----start */
#ifdef ENABLE_CHIP_STATUS_MONITOR
		himax_data->running_status = 0;
		queue_delayed_work(himax_data->himax_wq,
				   &himax_data->himax_chip_monitor, 10 * HZ);
#endif
					/*----[ENABLE_CHIP_STATUS_MONITOR]-----end */
		return;
	} else if (ESD_RESET_ACTIVATE)
#else
	if (ESD_RESET_ACTIVATE)
#endif
	{
		ESD_RESET_ACTIVATE = 0;
		printk(KERN_INFO
		       "[Himax] %s: Back from pull reset pin, ready to serve.\n",
		       __func__);
		/*Mutexlock Protect Start */
		mutex_unlock(&himax_data->mutex_lock);
		/*Mutexlock Protect End */
		enable_irq(himax_data->irq);

						/*----[ENABLE_CHIP_STATUS_MONITOR]-----start */
#ifdef ENABLE_CHIP_STATUS_MONITOR
		himax_data->running_status = 0;
		queue_delayed_work(himax_data->himax_wq,
				   &himax_data->himax_chip_monitor, 10 * HZ);
#endif
						/*----[ENABLE_CHIP_STATUS_MONITOR]-----end */
		return;
	}
	/*----[HX_ESD_WORKAROUND]------------end */

	/*calculate the checksum */
	for (i = 0; i < hx_touch_info_size; i++) {
		check_sum_cal += buf[i];
	}

	/*check sum fail */
	if ((check_sum_cal != 0x00)
	    || (buf[HX_TOUCH_INFO_POINT_CNT] & 0xF0) != 0xF0) {
		if (debug_log || getDebugLevel() & 0x1) {
			printk(KERN_INFO
			       "[Himax] checksum fail : check_sum_cal: 0x%02X\n",
			       check_sum_cal);
		}

		/*Mutexlock Protect Start */
		mutex_unlock(&himax_data->mutex_lock);
		/*Mutexlock Protect End */

		enable_irq(himax_data->irq);

		/*----[HX_ESD_WORKAROUND]----------start */
#ifdef HX_ESD_WORKAROUND
		ESD_COUNTER++;
		printk
		    ("[Himax]: ESD event checked - check_sum_cal, ESD_COUNTER = %d.\n",
		     ESD_COUNTER);
		if (ESD_COUNTER > ESD_COUNTER_SETTING) {
			ESD_HW_REST();
		}
#endif
		/*----[HX_ESD_WORKAROUND]----------end */

		/*----[ENABLE_CHIP_STATUS_MONITOR]-------start */
#ifdef ENABLE_CHIP_STATUS_MONITOR
		himax_data->running_status = 0;
		queue_delayed_work(himax_data->himax_wq,
				   &himax_data->himax_chip_monitor, 10 * HZ);
#endif
		/*----[ENABLE_CHIP_STATUS_MONITOR]------end */

		return;
	}

	/*debug , printk the i2c packet data */
	/*----[HX_TP_SYS_DEBUG_LEVEL]-----------start */
#ifdef HX_TP_SYS_DEBUG_LEVEL
	if (getDebugLevel() & 0x8) {
		printk(KERN_INFO "[Himax] %s: raw data:\n", __func__);
		for (i = 0; i < 128; i = i + 8) {
			printk(KERN_INFO
			       "%d: 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X \n",
			       i, buf[i], buf[i + 1], buf[i + 2], buf[i + 3],
			       buf[i + 4], buf[i + 5], buf[i + 6], buf[i + 7]);
		}
	}
#endif
	/*----[HX_TP_SYS_DEBUG_LEVEL]-------------end */

	/*touch monitor raw data fetch */
	/*----[HX_TP_SYS_DIAG]--------------------start */
#ifdef HX_TP_SYS_DIAG
	diag_cmd = getDiagCommand();
	if (diag_cmd >= 1 && diag_cmd <= 6) {

		if (IC_TYPE == HX_85XX_D_SERIES_PWON) {
			/*Check 128th byte CRC */
			for (i = hx_touch_info_size, check_sum_cal = 0; i < 128;
			     i++) {
				check_sum_cal += buf[i];
			}

			if (check_sum_cal % 0x100 != 0) {
				goto bypass_checksum_failed_packet;
			}
		}

		mutual_data = getMutualBuffer();
		self_data = getSelfBuffer();

		/* initiallize the block number of mutual and self */
		mul_num = getXChannel() * getYChannel();

#ifdef HX_EN_SEL_BUTTON
		self_num = getXChannel() + getYChannel() + HX_BT_NUM;
#else
		self_num = getXChannel() + getYChannel();
#endif

		/*Himax: Check Raw-Data Header */
		if (buf[hx_touch_info_size] == buf[hx_touch_info_size + 1]
		    && buf[hx_touch_info_size + 1] ==
		    buf[hx_touch_info_size + 2]
		    && buf[hx_touch_info_size + 2] ==
		    buf[hx_touch_info_size + 3]
		    && buf[hx_touch_info_size] > 0) {
			index = (buf[hx_touch_info_size] - 1) * RawDataLen;
			/*printk("Header[%d]: %x, %x, %x, %x, mutual: %d, self: %d\n", index, buf[56], buf[57], buf[58], buf[59], mul_num, self_num); */
			for (i = 0; i < RawDataLen; i++) {
				if (IC_TYPE == HX_85XX_D_SERIES_PWON) {
					temp1 = index + i;
				} else {
					temp1 = index;
				}

				if (temp1 < mul_num) {	/*mutual */
					mutual_data[index + i] = buf[i + hx_touch_info_size + 4];	/*4: RawData Header */
				} else {	/*self */
					if (IC_TYPE == HX_85XX_D_SERIES_PWON) {
						temp1 = i + index;
						temp2 = self_num + mul_num;
					} else {
						temp1 = i;
						temp2 = self_num;
					}
					if (temp1 >= temp2) {
						break;
					}

					if (IC_TYPE == HX_85XX_D_SERIES_PWON) {
						self_data[i + index - mul_num] = buf[i + hx_touch_info_size + 4];	/*4: RawData Header */
					} else {
						self_data[i] = buf[i + hx_touch_info_size + 4];	/*4: RawData Header */
					}
				}
			}
		} else {
			printk(KERN_INFO
			       "[Himax] %s: header format is wrong!\n",
			       __func__);
		}
	} else if (diag_cmd == 7) {
		memcpy(&(diag_coor[0]), &buf[0], 128);
	}
	/*coordinate dump start    */
	if (coordinate_dump_enable == 1) {
		for (i = 0; i < (15 + (HX_MAX_PT + 5) * 2 * 5); i++) {
			coordinate_char[i] = 0x20;
		}
		coordinate_char[15 + (HX_MAX_PT + 5) * 2 * 5] = 0xD;
		coordinate_char[15 + (HX_MAX_PT + 5) * 2 * 5 + 1] = 0xA;
	}
	/*coordinate dump end    */
#endif
	/*----[HX_TP_SYS_DIAG]------end 	 */

bypass_checksum_failed_packet:

#if defined(HX_EN_SEL_BUTTON) || defined(HX_EN_MUT_BUTTON)
	tpd_key = (buf[HX_TOUCH_INFO_POINT_CNT + 2] >> 4);
	if (tpd_key == 0x0F) {
		tpd_key = 0xFF;
	}
	/*printk("TPD BT:  %x\r\n", tpd_key); */
#else
	tpd_key = 0xFF;
#endif

	p_point_num = hx_point_num;

	if (buf[HX_TOUCH_INFO_POINT_CNT] == 0xff) {
		hx_point_num = 0;
	} else {
		hx_point_num = buf[HX_TOUCH_INFO_POINT_CNT] & 0x0f;
	}

	/* Touch Point information */
	if (hx_point_num != 0 && tpd_key == 0xFF) {
		/* parse the point information */
		for (i = 0; i < HX_MAX_PT; i++) {
			if (buf[4 * i] != 0xFF) {
				/* x and y axis */
				x = buf[4 * i + 1] | (buf[4 * i] << 8);
				y = buf[4 * i + 3] | (buf[4 * i + 2] << 8);

				temp_x[i] = x;
				temp_y[i] = y;

				if ((x <= x_res) && (y <= y_res)) {
					/* caculate the pressure and area */
					press = buf[4 * HX_MAX_PT + i];
					area = press;
					if (area > 31) {
						area = (area >> 3);
					}

					/* kernel call for report point area, pressure and x-y axis */
					input_report_key(himax_data->input_dev, BTN_TOUCH, 1);	/* touch down */
					input_report_abs(himax_data->input_dev, ABS_MT_TRACKING_ID, i);	/*ID of touched point */
					input_report_abs(himax_data->input_dev, ABS_MT_TOUCH_MAJOR, area);	/*Finger Size */
					input_report_abs(himax_data->input_dev, ABS_MT_PRESSURE, press);	/* Pressure */
					input_report_abs(himax_data->input_dev, ABS_MT_POSITION_X, x);	/* X axis */
					input_report_abs(himax_data->input_dev, ABS_MT_POSITION_Y, y);	/* Y axis */

					input_mt_sync(himax_data->input_dev);

#ifdef HX_PORTING_DEB_MSG
					if (debug_log || getDebugLevel() & 0x1) {
						printk
						    ("[Himax] %s: Touch DOWN x = %d, y = %d, area = %d, press = %d.\n",
						     __func__, x, y, area,
						     press);
					}
#endif

					/*----[HX_TP_SYS_DIAG]-----start */
#ifdef HX_TP_SYS_DIAG
					/*coordinate dump start  */
					if (coordinate_dump_enable == 1) {
						do_gettimeofday(&t);
						time_to_tm(t.tv_sec, 0,
							   &broken);

						sprintf(&coordinate_char[0],
							"%2d:%2d:%2d:%3li,",
							broken.tm_hour,
							broken.tm_min,
							broken.tm_sec,
							t.tv_usec / 1000);

						sprintf(&coordinate_char
							[15 + (i * 2) * 5],
							"%4d,", x);
						sprintf(&coordinate_char
							[15 + (i * 2) * 5 + 5],
							"%4d,", y);

						coordinate_fn->f_op->
						    write(coordinate_fn,
							  &coordinate_char[0],
							  15 + (HX_MAX_PT +
								5) * 2 *
							  sizeof(char) * 5 + 2,
							  &coordinate_fn->
							  f_pos);
					}
					/*coordinate dump end   */
#endif
					/*----[HX_TP_SYS_DIAG]----------end                   */
				}
			} else {
				temp_x[i] = 0xFFFF;
				temp_y[i] = 0xFFFF;
				input_mt_sync(himax_data->input_dev);
			}
		}
		input_sync(himax_data->input_dev);

		/*----[HX_ESD_WORKAROUND]--------start */
#ifdef HX_ESD_WORKAROUND
		ESD_COUNTER = 0;
#endif
		/*----[HX_ESD_WORKAROUND]-------end */
	} else if (hx_point_num == 0 && tpd_key != 0xFF) {
		temp_x[0] = 0xFFFF;
		temp_y[0] = 0xFFFF;
		temp_x[1] = 0xFFFF;
		temp_y[1] = 0xFFFF;

		if (tpd_key == 1) {
			input_report_key(himax_data->input_dev,
					 tpd_keys_local[0], 1);
			input_mt_sync(himax_data->input_dev);
			input_sync(himax_data->input_dev);
			printk("[Himax]: Press KEY_BACK\r\n");
		}

		if (tpd_key == 2) {
			input_report_key(himax_data->input_dev,
					 tpd_keys_local[1], 1);
			input_mt_sync(himax_data->input_dev);
			input_sync(himax_data->input_dev);
			printk("[Himax]: Press KEY_HOME*** \r\n");
		}

		if (tpd_key == 3) {
			input_report_key(himax_data->input_dev,
					 tpd_keys_local[2], 1);
			input_mt_sync(himax_data->input_dev);
			input_sync(himax_data->input_dev);
			printk("[Himax]: Press KEY_MENU*** \r\n");
		}

		/*----[HX_ESD_WORKAROUND]-------start */
#ifdef HX_ESD_WORKAROUND
		ESD_COUNTER = 0;
#endif
		/*----[HX_ESD_WORKAROUND]-------end */
	} else if (hx_point_num == 0 && tpd_key == 0xFF) {
		temp_x[0] = 0xFFFF;
		temp_y[0] = 0xFFFF;
		temp_x[1] = 0xFFFF;
		temp_y[1] = 0xFFFF;

		if (tpd_key_old != 0xFF) {
			input_report_key(himax_data->input_dev,
					 tpd_keys_local[tpd_key_old - 1], 0);
			input_mt_sync(himax_data->input_dev);
			input_sync(himax_data->input_dev);
		} else {
			/* leave event */
			input_report_key(himax_data->input_dev, BTN_TOUCH, 0);	/* touch up */
			input_mt_sync(himax_data->input_dev);
			input_sync(himax_data->input_dev);

#ifdef HX_PORTING_DEB_MSG
			if (debug_log || getDebugLevel() & 0x1) {
				printk("[Himax] %s: Touch UP.\n", __func__);
			}
#endif

			/*----[HX_TP_SYS_DIAG]-------start */
#ifdef HX_TP_SYS_DIAG
			/*coordinate dump start  */
			if (coordinate_dump_enable == 1) {
				do_gettimeofday(&t);
				time_to_tm(t.tv_sec, 0, &broken);

				sprintf(&coordinate_char[0], "%2d:%2d:%2d:%lu,",
					broken.tm_hour, broken.tm_min,
					broken.tm_sec, t.tv_usec / 1000);
				sprintf(&coordinate_char[15], "Touch up!");
				coordinate_fn->f_op->write(coordinate_fn,
							   &coordinate_char[0],
							   15 + (HX_MAX_PT +
								 5) * 2 *
							   sizeof(char) * 5 + 2,
							   &coordinate_fn->
							   f_pos);
			}
			/*coordinate dump end */
#endif
			/*----[HX_TP_SYS_DIAG]---end */
		}
		/*----[HX_ESD_WORKAROUND]--------start */
#ifdef HX_ESD_WORKAROUND
		ESD_COUNTER = 0;
#endif
		/*----[HX_ESD_WORKAROUND]------end */
	}
	/*
	   //----[HX_EN_GESTURE]---------------------------------------------------------------------------------start
	   #ifdef HX_EN_GESTURE
	   if (hx_point_num == 2 && p_point_num == 2 && temp_x[0] != 0xFFFF && temp_y[0] != 0xFFFF && temp_x[1] != 0xFFFF && temp_y[1] != 0xFFFF)
	   {
	   if (temp_x[0] > temp_x[1] && temp_y[0] > temp_y[1])
	   {
	   Dist_Cal_Now = (temp_x[0] - temp_x[1]) + (temp_y[0] - temp_y[1]);
	   }
	   else if (temp_x[0] > temp_x[1] && temp_y[1] > temp_y[0])
	   {
	   Dist_Cal_Now = (temp_x[0] - temp_x[1]) + (temp_y[1] - temp_y[0]);
	   }
	   else if (temp_x[1] > temp_x[0] && temp_y[0] > temp_y[1])
	   {
	   Dist_Cal_Now = (temp_x[1] - temp_x[0]) + (temp_y[0] - temp_y[1]);
	   }
	   else if (temp_x[1] > temp_x[0] && temp_y[1] > temp_y[0])
	   {
	   Dist_Cal_Now = (temp_x[1] - temp_x[0]) + (temp_y[1] - temp_y[0]);
	   }

	   if (Dist_Cal_Now - Dist_Cal_EX > 10)
	   {
	   ZoomInCnt++;
	   }
	   else if (Dist_Cal_EX - Dist_Cal_Now > 10)
	   {
	   ZoomOutCnt++;
	   }

	   //EX_x[0]= x[0];
	   //EX_y[0]= y[0];
	   //EX_x[1]= x[1];
	   //EX_y[1]= y[1];

	   if (ZoomInCnt > 2)
	   {
	   ZoomInFlag = 1;
	   ZoomOutFlag = 0;
	   }
	   else if (ZoomOutCnt > 2)
	   {
	   ZoomOutFlag =1;
	   ZoomInFlag = 0;
	   }
	   Dist_Cal_EX = Dist_Cal_Now;
	   }
	   else if (hx_point_num == 2 && p_point_num != 2 && temp_x[0] != 0xFFFF && temp_y[0] != 0xFFFF && temp_x[1] != 0xFFFF && temp_y[1] != 0xFFFF)
	   {
	   Dist_Cal_EX = 0;
	   }
	   else
	   {
	   Dist_Cal_EX = 0xFFFF;
	   Dist_Cal_Now = 0xFFFF;
	   ZoomInCnt = 0;
	   ZoomOutCnt = 0;
	   p_point_num = 0xFFFF;
	   }
	   #endif
	   //----[HX_EN_GESTURE]---------------------------------------------------------------------------------start
	 */

	tpd_key_old = tpd_key;

/*Mutexlock Protect Start */
	mutex_unlock(&himax_data->mutex_lock);
/*Mutexlock Protect End */

	enable_irq(himax_data->irq);

/*----[ENABLE_CHIP_STATUS_MONITOR]-----start	 */
#ifdef ENABLE_CHIP_STATUS_MONITOR
	himax_data->running_status = 0;
	queue_delayed_work(himax_data->himax_wq,
			   &himax_data->himax_chip_monitor, 10 * HZ);
#endif
/*----[ENABLE_CHIP_STATUS_MONITOR]-----end	 */

	return;

work_func_send_i2c_msg_fail:

	printk(KERN_ERR "[Himax]: work_func_send_i2c_msg_fail: %d \n",
	       __LINE__);

/*----[ENABLE_CHIP_RESET_MACHINE]------start */
#ifdef ENABLE_CHIP_RESET_MACHINE
	if (himax_data->init_success) {
		queue_delayed_work(himax_data->himax_wq,
				   &himax_data->himax_chip_reset_work, 0);
	}
#endif
/*----[ENABLE_CHIP_RESET_MACHINE]------end */

/*----[ENABLE_CHIP_STATUS_MONITOR]-----start	 */
#ifdef ENABLE_CHIP_STATUS_MONITOR
	himax_data->running_status = 0;
	queue_delayed_work(himax_data->himax_wq,
			   &himax_data->himax_chip_monitor, 10 * HZ);
#endif
/*----[ENABLE_CHIP_STATUS_MONITOR]-----end	 */

	return;
}

/*========================================================
//
//	Segment : Himax Linux Driver Probe Function
//
//======================================================== */
static int himax_open(struct inode *inode, struct file *file)
{
	printk("[Himax] %s ++ \n", __func__);
	return nonseekable_open(inode, file);
}

static int himax_release(struct inode *inode, struct file *file)
{
	printk("[Himax] %s ++ \n", __func__);
	return 0;
}

static long himax_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0, val = 0;
	int ret = 0;

	printk("[Himax] %s: cmd = 0x%x.\n", __func__, cmd);

	if (_IOC_TYPE(cmd) != TOUCH_IOC_MAGIC) {
		return -ENOTTY;
	}
	if (_IOC_DIR(cmd) & _IOC_READ)
		err =
		    !access_ok(VERIFY_WRITE, (void __user *)arg,
			       _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =
		    !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err) {
		return -EFAULT;
	}

	switch (cmd) {
	case TOUCH_INIT:
	case TOUCH_FW_UPDATE_FLAG:
		if (get_user(val, (unsigned long __user *)arg))
			return -EFAULT;
		if (val) {
			printk
			    ("[Himax]: TOUCH_FW_UPDATE - TOUCH_FW_UPGRADE_ON \n");
			himax_data->AP_update = 1;
			himax_data->AP_progress = 0;
		} else {
			printk
			    ("[Himax]:TOUCH_FW_UPDATE - TOUCH_FW_UPGRADE_OFF \n");
			himax_data->AP_update = 0;
		}
		break;

	case TOUCH_FW_UPDATE_PROCESS:
		printk("[Himax]:%s: TOUCH_FW_UPDATE_PROCESS \n", __func__);
		ret = himax_firmware_upgrade(AP);
		break;
	case TOUCH_TP_FW_check:
		printk("[Himax]:%s: TOUCH_FW_UPDATE_CHECK \n", __func__);
		ret = himax_firmware_check();
		printk("[Himax]:%s: TOUCH_FW_UPDATE_CHECK = % d\n", __func__,
		       ret);
		return ret;
	default:
		printk("[Himax]:%s: incorrect cmd (%d) \n", __FUNCTION__,
		       _IOC_NR(cmd));
		return -EINVAL;
	}

	return 0;
}

static struct file_operations himax_fops = {
	.owner = THIS_MODULE,
	.open = himax_open,
	.release = himax_release,
	.unlocked_ioctl = himax_ioctl
};

static struct miscdevice himax_misc_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "hx8528",
	.fops = &himax_fops,
};

static ssize_t touch_switch_name(struct switch_dev *sdev, char *buf)
{
	printk(KERN_INFO "[Himax]: D%s.%s.%s-%s\n",
	       himax_data->himax_firmware_version, himax_data->tp_lens_version,
	       himax_data->config_firmware_version, DRIVER_VERSION);
	return sprintf(buf, "D%s.%s.%s-%s\n",
		       himax_data->himax_firmware_version,
		       himax_data->tp_lens_version,
		       himax_data->config_firmware_version, DRIVER_VERSION);
}

static ssize_t touch_switch_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%d\n", himax_data->tp_status);
}

/*----[ interrupt ]---------------start */
static irqreturn_t himax_ts_irq_handler(int irq, void *dev_id)
{
	/*      struct himax_ts_data *ts = dev_id; */
	/*      struct i2c_client *client = ts->client; */

	/*      dev_dbg(&client->dev, "[HIMAX TP MSG] %s\n", __func__); */

	/*      printk("[Himax] %s ++ . \n",__func__); */
	disable_irq_nosync(himax_data->irq);
	queue_work(himax_data->himax_wq, &himax_data->work);

	return IRQ_HANDLED;
}

static int himax_ts_register_interrupt(struct i2c_client *client)
{
	/*      struct himax_ts_data *ts = i2c_get_clientdata(client); */
	int err = 0;

	if (HX_INT_IS_EDGE) {	/*edge trigger */
		err =
		    request_irq(himax_data->irq, himax_ts_irq_handler,
				IRQF_TRIGGER_FALLING, himax_data->client->name,
				himax_data->client);
	} else {		/*low level trigger */

		err =
		    request_irq(himax_data->irq, himax_ts_irq_handler,
				IRQF_TRIGGER_LOW, himax_data->client->name,
				himax_data->client);
	}

	if (err) {
		printk("[Himax] %s: request_irq %d failed. \n", __func__,
		       himax_data->irq);
	} else {
		printk("[Himax] %s: request_irq OK.\n", __func__);
	}
	return err;
}

/*----[ interrupt ]---------end */

/*----[ i2c ]---------------start */
static int himax_ts_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct himax_i2c_platform_data *pdata;
	int err = 0, i = 0;

	printk("[Himax] %s ++ \n", __func__);
	/*****************************
	// Check i2c functionality
	//**************************** */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "[Himax] %s: i2c check functionality error\n",
		       __func__);
		goto err_check_functionality_failed;
	}
	printk("[Himax] %s: i2c functionality check OK.\n", __func__);
	/***********************************
	// Allocate the himax_ts_data
	//********************************** */
	himax_data = kzalloc(sizeof(struct himax_ts_data), GFP_KERNEL);
	if (himax_data == NULL) {
		printk(KERN_ERR "[Himax] %s: allocate himax_ts_data failed\n",
		       __func__);
		err = -ENOMEM;
		goto err_alloc_data_failed;
	}

	himax_data->client = client;
	himax_data->init_success = 0;
	himax_data->tp_status = 0;	/* add by josh for check poweron */
	himax_data->firmware_version_checksum = 0;
	himax_data->tp_firmware_upgrade_proceed = 0;
	i2c_set_clientdata(client, himax_data);
	pdata = client->dev.platform_data;
	if (likely(pdata != NULL)) {
		himax_data->intr_gpio = pdata->intr_gpio;
		himax_data->rst_gpio = pdata->rst_gpio;
	}
	printk("[Himax] %s: intr_gpio =%d, rst_gpio=%d \n", __func__,
	       himax_data->intr_gpio, himax_data->rst_gpio);
	printk("[Himax] %s: himax_ts_data allocate OK. \n", __func__);
	/*********************************
	// TODO Interrupt Pin
	//******************************* */
	/*himax_data->intr_gpio = HIMAX_INT_GPIO; */
	if (gpio_request(himax_data->intr_gpio, "HimaxTouch-irq") != 0) {
		printk("[Himax]: interrupt gpio %d request fail.\n",
		       HIMAX_PWR_GPIO);
	}
	err = gpio_direction_input(HIMAX_INT_GPIO);
	if (err) {
		printk(KERN_ERR
		       "[Himax]: Failed to set interrupt direction, error=%d.\n",
		       err);
		goto err_init_intr_gpio_fail;
	} else {
		himax_data->irq = gpio_to_irq(himax_data->intr_gpio);
		printk("[Himax] %s: init interrupt pin OK. \n", __func__);
	}
	/***************************
	// TODO Reset Pin
	//************************** */
	/*himax_data->rst_gpio = HIMAX_RST_GPIO; */
	if (gpio_request(HIMAX_RST_GPIO, "HimaxTouch-reset") != 0) {
		printk("[Himax]: reset gpio %d request fail.\n",
		       HIMAX_PWR_GPIO);
	}
	err = gpio_direction_output(HIMAX_RST_GPIO, 1);	/*reset set high */
	if (err) {
		printk(KERN_ERR
		       "[Himax]: Failed to set interrupt direction, error=%d.\n",
		       err);
		goto err_init_rst_gpio_fail;
	}
	msleep(100);
	printk("[Himax] %s: init reset pin OK. \n", __func__);
	/*********************************
	// Create Work Queue
	//******************************** */
	/*----[ HX_TP_SYS_FLASH_DUMP ]---------start */
#ifdef HX_TP_SYS_FLASH_DUMP
	himax_data->flash_wq = create_singlethread_workqueue("himax_flash_wq");
	if (!himax_data->flash_wq) {
		printk(KERN_ERR
		       "[Himax] %s: create himax_flash_wq workqueue failed\n",
		       __func__);
		goto err_create_flash_wq_failed;
	}
#endif
	/*----[ HX_TP_SYS_FLASH_DUMP ]---------end */

	himax_data->himax_wq = create_singlethread_workqueue("himax_wq");
	if (!himax_data->himax_wq) {
		printk(KERN_ERR
		       "[Himax] %s: create himax_wq workqueue failed\n",
		       __func__);
		goto err_create_wq_failed;
	}

	printk("[Himax] %s: Create Work Queue OK. \n", __func__);
	/************************
	// Init work
	*********************** */
	/*----[ENABLE_CHIP_RESET_MACHINE]--------start */
	INIT_DELAYED_WORK(&himax_data->himax_chip_reset_work,
			  himax_chip_reset_function);
	/*----[ENABLE_CHIP_RESET_MACHINE]--------end */

	/*----[ENABLE_CHIP_STATUS_MONITOR]-------start	 */
#ifdef ENABLE_CHIP_STATUS_MONITOR
	INIT_DELAYED_WORK(&himax_data->himax_chip_monitor, himax_chip_monitor_function);	/*for ESD solution */
#endif
	/*----[ENABLE_CHIP_STATUS_MONITOR]-------start	 */

	/*----[HX_TP_SYS_FLASH_DUMP]-------------start	 */
#ifdef HX_TP_SYS_FLASH_DUMP
	INIT_WORK(&himax_data->flash_work, himax_ts_flash_work_func);
#endif
	/*----[HX_TP_SYS_FLASH_DUMP]-------------end	 */

	INIT_WORK(&himax_data->work, himax_ts_work_func);

	/*firmware upgrade by Josh +++ */
#ifdef ENABLE_SELF_FIRMWARE_UPGRADE
	INIT_DELAYED_WORK(&himax_data->himax_chip_firmware_upgrade_work,
			  himax_chip_self_firmware_upgrade);
#endif
	/*firmware upgrade by Josh --- */

	printk("[Himax] %s:Init work function OK. \n", __func__);
	/**************************************************
	// Himax Information / Initial, I2C must be ready
	//************************************************* */
	himax_ic_package_check(himax_data);

	/**************************************************
	// *.i FW update
	//************************************************* */
	himax_touch_information();
	calculate_point_number();

	/*----[HX_TP_SYS_FLASH_DUMP]-------------start */
#ifdef HX_TP_SYS_FLASH_DUMP
	setSysOperation(0);
	setFlashBuffer();
#endif
	/*----[HX_TP_SYS_FLASH_DUMP]-------------end */

	/*----[HX_TP_SYS_DIAG]-------------------start */
#ifdef HX_TP_SYS_DIAG
	setXChannel(HX_RX_NUM);	/* X channel */
	setYChannel(HX_TX_NUM);	/* Y channel */

	setMutualBuffer();
	if (getMutualBuffer() == NULL) {
		printk(KERN_ERR
		       "[Himax] %s: mutual buffer allocate fail failed\n",
		       __func__);
		return -1;
		goto err_set_mutual_buffer_fail;
	}
#endif
	/*----[HX_TP_SYS_DIAG]------------------end */

	/*----[ENABLE_CHIP_RESET_MACHINE]-----start */
#ifdef ENABLE_CHIP_RESET_MACHINE
	/*himax_data->retry_time = 0; */
#endif
	/*----[ENABLE_CHIP_RESET_MACHINE]-------end */

	/*----[HX_ESD_WORKAROUND]-------------start */
	/*#ifdef HX_ESD_WORKAROUND */
	ESD_RESET_ACTIVATE = 0;
	/*#endif */
	/*----[HX_ESD_WORKAROUND]---------------end */

	/*----[ENABLE_CHIP_STATUS_MONITOR]----start	 */
#ifdef ENABLE_CHIP_STATUS_MONITOR
	himax_data->running_status = 0;
#endif
	/*----[ENABLE_CHIP_STATUS_MONITOR]------end */
	err = himax_touch_sysfs_init();
	if (err) {
		goto err_himax_touch_sysfs_create_group_fail;
	}

	/*add by josh for init  +++ */
	memset(himax_data->himax_firmware_version, 0, 8);
	/*add by josh for init  --- */

	himax_data->attrs.attrs = himax_attr;
	err = sysfs_create_group(&client->dev.kobj, &himax_data->attrs);
	if (err) {
		printk("[Himax] %s: Not able to create the sysfs \n", __func__);
		goto err_sysfs_create_group_fail;
	}
	printk("[Himax] %s: Himax Information Initial OK. \n", __func__);

	/*Mutexlock Protect Start */
	mutex_init(&himax_data->mutex_lock);
	/*Mutexlock Protect End */

	/*Wakelock Protect Start */
	wake_lock_init(&himax_data->wake_lock, WAKE_LOCK_SUSPEND,
		       "himax_touch_wake_lock");
	/*Wakelock Protect End */

	/********************************
	// Allocate Input Device
	//******************************* */
	himax_data->input_dev = input_allocate_device();

	if (himax_data->input_dev == NULL) {
		printk("[Himax]: Failed to allocate input device \n");
		goto err_input_dev_alloc_failed;
	}

	himax_data->input_dev->name = INPUT_DEV_NAME;
	himax_data->abs_x_max = HX_X_RES;
	himax_data->abs_y_max = HX_Y_RES;

	/*----[HX_EN_XXX_BUTTON]--------------------------start	 */
#if defined(HX_EN_SEL_BUTTON) || defined(HX_EN_MUT_BUTTON)
	for (i = 0; i < HX_BT_NUM; i++) {
		set_bit(tpd_keys_local[i], himax_data->input_dev->keybit);
	}
#endif
	/*----[HX_EN_XXX_BUTTON]---------------------------end	 */

	__set_bit(EV_KEY, himax_data->input_dev->evbit);
	__set_bit(EV_ABS, himax_data->input_dev->evbit);
	__set_bit(BTN_TOUCH, himax_data->input_dev->keybit);

	input_set_abs_params(himax_data->input_dev, ABS_MT_TRACKING_ID, 0,
			     HX_MAX_PT, 0, 0);
	input_set_abs_params(himax_data->input_dev, ABS_MT_POSITION_X, 0,
			     HX_X_RES, 0, 0);
	input_set_abs_params(himax_data->input_dev, ABS_MT_POSITION_Y, 0,
			     HX_Y_RES, 0, 0);
	input_set_abs_params(himax_data->input_dev, ABS_MT_TOUCH_MAJOR, 0, 31, 0, 0);	/*Finger Size */
	input_set_abs_params(himax_data->input_dev, ABS_MT_WIDTH_MAJOR, 0, 31, 0, 0);	/*Touch Size */
	input_set_abs_params(himax_data->input_dev, ABS_MT_PRESSURE, 0, 0xFF, 0,
			     0);

	err = input_register_device(himax_data->input_dev);
	if (err) {
		printk("[Himax] %s: unable to register %s input device.\n",
		       __func__, himax_data->input_dev->name);
		goto err_input_register_device_failed;
	}
	printk("[Himax] %s: Input Device Reigster OK. \n", __func__);
	/****************************
	// register switch device
	//*************************** */
	himax_data->touch_sdev.name = TOUCH_SDEV_NAME;
	himax_data->touch_sdev.print_name = touch_switch_name;
	himax_data->touch_sdev.print_state = touch_switch_state;
	if (switch_dev_register(&himax_data->touch_sdev) < 0) {
		printk("[Himax] %s: touch switch device register failed!\n",
		       __func__);
	}
	/**********************************
	// register misc device
	//********************************* */
	err = misc_register(&himax_misc_dev);
	if (err < 0) {
		printk
		    ("[Himax] %s: could not register ME372CL touch misc device.\n",
		     __func__);
	}
	/****************************
	// Himax Touch IC Power on
	****************************/
	/*//HW Reset
	   //----[HX_RST_PIN_FUNC]------------------------start
	   //#ifdef HX_RST_PIN_FUNC

	   //   err = gpio_direction_output(himax_data->rst_gpio, 1);
	   //   if (err)
	   //   {
	   //           printk(KERN_ERR "[Himax]: Failed to set reset direction, error=%d\n", err);
	   //   }
	   //himax_hw_reset();
	   //#endif
	   //----[HX_RST_PIN_FUNC]-------------------------end

	   //PowerOn ++
	   //err = himax_ts_poweron();  */
	err = himax_read_fw_ver(true);
	/*add by josh for checking poweron  */
	if (err == 0) {
		himax_data->tp_status = 1;
	} else {
		printk(KERN_ERR "[Himax]: power on error=%d.\n", err);
		himax_data->tp_status = 0;
		goto err_power_on_failed;
	}
	/*PowerOn --     */
	/*TODO check the interrupt's API function by differrnt platform  */
	err = himax_ts_register_interrupt(himax_data->client);
	if (err) {
		goto err_register_interrupt_failed;
	}

	/*if (gpio_get_value(himax_data->intr_gpio) == 0)
	   {
	   printk(KERN_INFO "[Himax] %s: handle missed interrupt\n", __func__);
	   himax_ts_irq_handler(himax_data->irq, himax_data);
	   } */
	printk("[Himax] %s: Himax Power on OK. \n", __func__);
	/***********************************
	// Remain work of Probe
	********************************** */

	/*----[CONFIG_HAS_EARLYSUSPEND]-----start */
#ifdef CONFIG_HAS_EARLYSUSPEND
	himax_data->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 1;
	himax_data->early_suspend.suspend = himax_ts_early_suspend;
	himax_data->early_suspend.resume = himax_ts_late_resume;
	register_early_suspend(&himax_data->early_suspend);
#endif
	/*----[CONFIG_HAS_EARLYSUSPEND]------------end */

	himax_data->init_success = 1;

	/*----[ENABLE_CHIP_RESET_MACHINE]--------start */
#ifdef ENABLE_CHIP_RESET_MACHINE
	/*himax_data->retry_time = 0; */
#endif
	/*----[ENABLE_CHIP_RESET_MACHINE]----------end */

	/*----[ENABLE_CHIP_STATUS_MONITOR]-------start */
#ifdef ENABLE_CHIP_STATUS_MONITOR
	queue_delayed_work(himax_data->himax_wq, &himax_data->himax_chip_monitor, 60 * HZ);	/*for ESD solution */
#endif
	/*----[ENABLE_CHIP_STATUS_MONITOR]----------end */

	/* add by josh for check USB status +++ */
	/*cable_status = check_cable_status();
	   hx8528_cable_status(cable_status); */
	/* add by josh for check USB status --- */

	/*add by josh for proc +++ */
	himax_me372cl_chip_create_proc_debug_flag();
	himax_me372cl_chip_create_proc_diag_file();
	himax_me372cl_chip_create_proc_register();
	himax_me372cl_chip_create_proc_debug_level();
	himax_me372cl_chip_create_proc_flash_dump();
	/*add by josh for proc --- */

	/*firmware upgrade by Josh ++ */
#ifdef ENABLE_SELF_FIRMWARE_UPGRADE
	queue_delayed_work(himax_data->himax_wq,
			   &himax_data->himax_chip_firmware_upgrade_work,
			   10 * HZ);
#endif

	printk("[Himax]: Himax Touch Driver ver:%s . \n", DRIVER_VERSION);

	return 0;

err_register_interrupt_failed:
err_power_on_failed:
	input_unregister_device(himax_data->input_dev);
err_input_register_device_failed:
	if (himax_data->input_dev) {
		input_free_device(himax_data->input_dev);
	}

err_input_dev_alloc_failed:
	/*Mutexlock Protect Start */
	mutex_destroy(&himax_data->mutex_lock);
	/*Mutexlock Protect End */

	/*Wakelock Protect Start */
	wake_lock_destroy(&himax_data->wake_lock);
	/*Wakelock Protect End */
	sysfs_remove_group(&client->dev.kobj, &himax_data->attrs);
err_sysfs_create_group_fail:
	himax_touch_sysfs_deinit();
err_himax_touch_sysfs_create_group_fail:
#ifdef HX_TP_SYS_DIAG
err_set_mutual_buffer_fail:
#endif

	/*----[ENABLE_CHIP_RESET_MACHINE]-----------start */
	cancel_delayed_work(&himax_data->himax_chip_reset_work);
	/*----[ENABLE_CHIP_RESET_MACHINE]-----------end */
	/*----[ENABLE_CHIP_STATUS_MONITOR]----------start */
#ifdef ENABLE_CHIP_STATUS_MONITOR
	cancel_delayed_work(&himax_data->himax_chip_monitor);
#endif
	/*----[ENABLE_CHIP_STATUS_MONITOR]----------end */
	/*----[HX_TP_SYS_FLASH_DUMP]----------------start	 */
#ifdef HX_TP_SYS_FLASH_DUMP
	cancel_work_sync(&himax_data->flash_work);
#endif
	/*----[HX_TP_SYS_FLASH_DUMP]----------------end	 */
	cancel_delayed_work(&himax_data->work);
	/*firmware upgrade by Josh ++ */
#ifdef ENABLE_SELF_FIRMWARE_UPGRADE
	cancel_delayed_work(&himax_data->himax_chip_firmware_upgrade_work);
#endif

err_create_wq_failed:
	if (himax_data->himax_wq)
		destroy_workqueue(himax_data->himax_wq);
#ifdef HX_TP_SYS_FLASH_DUMP
	if (himax_data->flash_wq)
		destroy_workqueue(himax_data->flash_wq);
err_create_flash_wq_failed:
#endif
err_init_rst_gpio_fail:
	gpio_free(himax_data->rst_gpio);
err_init_intr_gpio_fail:
	gpio_free(himax_data->intr_gpio);

err_alloc_data_failed:
err_check_functionality_failed:
	himax_data->init_success = 0;
	himax_data->tp_status = 0;
	return err;
}

static int himax_ts_remove(struct i2c_client *client)
{
	/*      struct himax_ts_data *ts = i2c_get_clientdata(client); */

	himax_touch_sysfs_deinit();

	unregister_early_suspend(&himax_data->early_suspend);
	free_irq(himax_data->irq, himax_data);

	/*Mutexlock Protect Start */
	mutex_destroy(&himax_data->mutex_lock);
	/*Mutexlock Protect End */

	if (himax_data->himax_wq) {
		destroy_workqueue(himax_data->himax_wq);
	}
	input_unregister_device(himax_data->input_dev);

	/*add by josh for proc +++ */
	himax_me372cl_chip_remove_proc_debug_flag();
	himax_me372cl_chip_remove_proc_diag_file();
	himax_me372cl_chip_remove_proc_register();
	himax_me372cl_chip_remove_proc_debug_level();
	himax_me372cl_chip_remove_proc_flash_dump();
	/*add by josh for proc --- */

	/*Wakelock Protect Start */
	wake_lock_destroy(&himax_data->wake_lock);
	/*Wakelock Protect End */

	kfree(himax_data);

	return 0;
}

static struct himax_i2c_setup_data himax_ts_info = {0, HIMAX_I2C_ADDR, 0, HIMAX_TS_NAME};

static const struct i2c_device_id himax_ts_id[] = {
	{HIMAX_TS_NAME, 0},
	{}
};

static struct i2c_driver himax_ts_driver = {
	.probe = himax_ts_probe,
	.remove = himax_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = himax_ts_suspend,
	.resume = himax_ts_resume,
#endif
	.id_table = himax_ts_id,
	.driver = {
		   .name = HIMAX_TS_NAME,
		   },
};

/*//int himax_add_i2c_device(struct himax_i2c_setup_data *i2c_set_data, struct i2c_driver *driver)
//{
//	struct i2c_board_info info;
//	struct i2c_adapter *adapter;
//	struct i2c_client *client;
//	int ret,err;
//
//	printk("%s : i2c_bus=%d; slave_address=0x%x; i2c_name=%s",__func__,i2c_set_data->i2c_bus,i2c_set_data->i2c_address, i2c_set_data->name);
//
//	memset(&info, 0, sizeof(struct i2c_board_info));
//	info.addr = i2c_set_data->i2c_address;
//	strlcpy(info.type, i2c_set_data->name, I2C_NAME_SIZE);
//
//	if(i2c_set_data->irq > 0)
//	{
//		info.irq = i2c_set_data->irq;
//	}
//
//	adapter = i2c_get_adapter( i2c_set_data->i2c_bus);
//	if (!adapter)
//	{
//		printk("%s: can't get i2c adapter %d\n",__func__,  i2c_set_data->i2c_bus);
//		err = -ENODEV;
//		goto err_driver;
//	}
//
//	client = i2c_new_device(adapter, &info);
//	if (!client)
//	{
//		printk("%s:  can't add i2c device at 0x%x\n",__func__, (unsigned int)info.addr);
//		err = -ENODEV;
//		goto err_driver;
//	}
//
//	i2c_put_adapter(adapter);
//
//	ret = i2c_add_driver(driver);
//	if (ret != 0)
//	{
//		printk("%s: can't add i2c driver\n", __func__);
//		err = -ENODEV;
//		goto err_driver;
//	}
//
//	return 0;
//
//err_driver:
//	return err;
//} */

static int himax_ts_init(void)
{
	printk(KERN_INFO "[himax] hx8528_ME372CL %s\n", __func__);
	i2c_add_driver(&himax_ts_driver);
	printk(KERN_INFO "[Himax] ME372CL init end!!!!\n");

	return 0;
}

static void __exit himax_ts_exit(void)
{
	i2c_del_driver(&himax_ts_driver);
	return;
}

module_init(himax_ts_init);
module_exit(himax_ts_exit);

MODULE_DESCRIPTION("Himax Touchscreen Driver");
MODULE_LICENSE("GPL");
/*----[ i2c ]----------end */

/*==============================
//
//  Segment : Other Function
//
//============================== */

static int hx8528_cable_status(int status)
{
	uint8_t buf0[2] = { 0 };

	if (cable_status == -1) {
		cable_status = status;
		printk("[Himax] %s: cable_status = %d ", __func__, status);
		return 0;
	} else {
		printk
		    ("[Himax] %s: cable_status=%d, tp_status=%d, init_success=%d.\n",
		     __func__, status, himax_data->tp_status,
		     himax_data->init_success);
		if ((himax_data->tp_status == 1)
		    && (himax_data->init_success == 1)) {
			if (status == 0x02) {	/*non usb */
				buf0[0] = 0x00;
				i2c_himax_write(himax_data->client, 0x90,
						&buf0[0], 1, DEFAULT_RETRY_CNT);
			} else if ((status == 0x00) || (status == 0x01)) {	/*usb plug in */
				buf0[0] = 0x01;
				i2c_himax_write(himax_data->client, 0x90,
						&buf0[0], 1, DEFAULT_RETRY_CNT);
			}
		}
		return 0;
	}
}

EXPORT_SYMBOL(hx8528_cable_status);

/*add by Josh +++ */
static int power_status(int status)
{
	printk(KERN_ERR "[Himax_ME372CL] %s: power_status: %d \n", __func__,
	       status);
	return 0;
}

EXPORT_SYMBOL(power_status);
/*add by Josh --- */
