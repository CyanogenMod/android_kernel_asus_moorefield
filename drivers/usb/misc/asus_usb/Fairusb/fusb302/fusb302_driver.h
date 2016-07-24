#ifndef FUSB302_DRIVER_H
#define FUSB302_DRIVER_H

#include "../core/platform.h"	// For typedefs

#include "../core/TypeC.h"	// For exported globals for debug messages
#include "../core/fusb30X.h"	// ^^

extern FSC_BOOL		blnCCPinIsCC1;
extern FSC_BOOL 	blnCCPinIsCC2;
extern DeviceReg_t	Registers;
extern SourceOrSink	sourceOrSink;


/* FUSB300 Register Addresses */
#define regDeviceID     0x01
#define regSwitches0    0x02
#define regSwitches1    0x03
#define regMeasure      0x04
#define regSlice        0x05
#define regControl0     0x06
#define regControl1     0x07
#define regControl2     0x08
#define regControl3     0x09
#define regMask         0x0A
#define regPower        0x0B
#define regReset        0x0C
#define regOCPreg       0x0D
#define regMaska        0x0E
#define regMaskb        0x0F
#define regControl4     0x10
#define regStatus0a     0x3C
#define regStatus1a     0x3D
#define regInterrupta   0x3E
#define regInterruptb   0x3F
#define regStatus0      0x40
#define regStatus1      0x41
#define regInterrupt    0x42
#define regFIFO         0x43

/* for early suspend */
#include <linux/earlysuspend.h>
#include <linux/wakelock.h>

/* FUSB302 STRUCT *************/
struct fusb302_i2c_data 
{
	struct i2c_client       *client;
	struct task_struct      *thread;	
	int	irq_num;
	volatile int int_n;

	int VBUS_5V_EN;
	int VBUS_12V_EN;

    /* for early suspend */
    struct early_suspend es;
	struct wake_lock wake_lock;
};

int fusb302_i2c_write_block(unsigned char regAddr, unsigned char length, unsigned char *data);
int fusb302_i2c_read_block(unsigned char regAddr, unsigned char length, unsigned char *data);
void fusb_gpio_set_vbus_5v(int blnEnable);
int fusb_gpio_get_vbus_5v(void);
void fusb_gpio_set_vbus_other(int blneEnable);
int fusb_gpio_get_vbus_other(void);
void fusb_i2c_data_set_int_n(int value);
int fusb_i2c_data_get_int_n(void);
void fusb_i2c_enable_timers(void);
void fusb_i2c_disable_timers(void);
void fusb_i2c_delay_10us(__u32 delay10us);
void set_usb30_mux(CC_ORIENTATION orientation);


#endif
