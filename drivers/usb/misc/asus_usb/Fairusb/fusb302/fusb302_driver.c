#include <linux/i2c.h>		// For I2C
#include <linux/wait.h>		// For thread sleeping
#include <linux/kthread.h>	// For thread
#include <linux/sched.h>		// For thread scheduling
#include <linux/delay.h>		// For msleep, etc.
#include <linux/slab.h>		// Kzalloc
#include <linux/gpio.h>		// GPIO
#include <linux/interrupt.h>	// IRQ Routines
#include <linux/hrtimer.h>		// High Resolution Timer functions
#include <linux/ktime.h>		// Ktime for timers
#include "fusb302_driver.h"
#include "../core/core.h"       // Core state machine

/* for GPIO 145 , 160 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <asm/intel-mid.h>

/* For charger test: get current informations */
//extern int core_get_advertised_current(void);

#define FUSB302_I2C_NAME "fusb302"
#define FUSB302_I2C_NUM     2
#define FUSB302_I2C_SLAVE_ADDRESS 0x22
#define FUSB302_GPIO_VBUS_5V 140
#define FUSB302_GPIO_INT_N 159

#define USB_RE_GPIO  145
#define USB_MUX_GPIO 160
#define USB_MUX_HIGH 1
#define USB_MUX_LOW 0

#define FUSB302_TIMER_PERIOD_US 1000L

#define US_TO_NS(X)			(X * 1E3L)
#define USB_I2C_INFO(...)	printk("[USB][FUSB_Driver_Check] " __VA_ARGS__);

static const unsigned int MAX_DELAY_10US = (UINT_MAX / 10);
struct fusb302_i2c_data *fusb_i2c_data;	// Create global for our data
static DECLARE_WAIT_QUEUE_HEAD(fusb_thread_wq);
static struct hrtimer fusb302_hr_timer;
static struct gpio fusb_gpios[] = {									// Add GPIOs here
	{ FUSB302_GPIO_VBUS_5V, GPIOF_OUT_INIT_LOW, "VBUS 5V Enable" }, /* default to output OFF*/
	{ FUSB302_GPIO_INT_N,	GPIOF_IN,			"Int_N IRQ"		 },	/* default to input */

	{ USB_RE_GPIO,	       GPIOF_OUT_INIT_HIGH,"USB3_TX_rD_en"	 }, /* default Re-driver to output HIGH */
	{ USB_MUX_GPIO,	       GPIOF_OUT_INIT_LOW,"USB3_MUX_SEL_SOC"}, /* default MUX to output HIGH */

};

static ktime_t ktime_temp;

/******************************************************************************
*                         Core Callback Handlers                              *
******************************************************************************/
void set_usb30_mux(CC_ORIENTATION orientation)
{
	int GPIOVal;
	GPIOVal = gpio_get_value(USB_MUX_GPIO);
	USB_I2C_INFO("[mux][after core machine]GPIO %d state = %d , Pin CC1 state = %d , Pin CC2 state = %d\n", USB_MUX_GPIO, GPIOVal, blnCCPinIsCC1, blnCCPinIsCC2);
	/* MUX driver */
	if (orientation == NONE)
	{
	
	}
	else if (orientation == CC2)
	{
		GPIOVal = gpio_get_value(USB_MUX_GPIO);
		USB_I2C_INFO("[mux][in MUX section , before process]GPIO %d state = %d , Pin CC1 state = %d , Pin CC2 state = %d\n", USB_MUX_GPIO, GPIOVal, blnCCPinIsCC1, blnCCPinIsCC2);

		gpio_set_value(USB_MUX_GPIO, USB_MUX_LOW);

		GPIOVal = gpio_get_value(USB_MUX_GPIO);
		USB_I2C_INFO("[mux][after MUX driver]GPIO %d state = %d , Pin CC1 state = %d , Pin CC2 state = %d\n", USB_MUX_GPIO, GPIOVal, blnCCPinIsCC1, blnCCPinIsCC2);


	}
	else if (orientation == CC1)
	{
		GPIOVal = gpio_get_value(USB_MUX_GPIO);
		USB_I2C_INFO("[mux][in MUX section , before process]GPIO %d state = %d , Pin CC1 state = %d , Pin CC2 state = %d\n", USB_MUX_GPIO, GPIOVal, blnCCPinIsCC1, blnCCPinIsCC2);

		gpio_set_value(USB_MUX_GPIO, USB_MUX_HIGH);

		GPIOVal = gpio_get_value(USB_MUX_GPIO);
		USB_I2C_INFO("[mux][after MUX driver]GPIO %d state = %d , Pin CC1 state = %d , Pin CC2 state = %d\n", USB_MUX_GPIO, GPIOVal, blnCCPinIsCC1, blnCCPinIsCC2);

	}

	USB_I2C_INFO("[mux]dump start.\n", __func__);
    DeviceRead(regDeviceID, 1, &Registers.DeviceID.byte);
    DeviceRead(regSwitches0, 1, &Registers.Switches.byte[0]);
    DeviceRead(regSwitches1, 1, &Registers.Switches.byte[1]);
    DeviceRead(regMeasure, 1, &Registers.Measure.byte);
    DeviceRead(regSlice, 1, &Registers.Slice.byte);
    DeviceRead(regControl0, 1, &Registers.Control.byte[0]);
    DeviceRead(regControl1, 1, &Registers.Control.byte[1]);
    DeviceRead(regControl2, 1, &Registers.Control.byte[2]);
    DeviceRead(regControl3, 1, &Registers.Control.byte[3]);
    DeviceRead(regMask, 1, &Registers.Mask.byte);
    DeviceRead(regPower, 1, &Registers.Power.byte);
    DeviceRead(regReset, 1, &Registers.Reset.byte);
    DeviceRead(regOCPreg, 1, &Registers.OCPreg.byte);
    DeviceRead(regMaska, 1, &Registers.MaskAdv.byte[0]);
    DeviceRead(regMaskb, 1, &Registers.MaskAdv.byte[1]);
    DeviceRead(regStatus0a, 1, &Registers.Status.byte[0]);
    DeviceRead(regStatus1a, 1, &Registers.Status.byte[1]);
    //DeviceRead(regInterrupta, 1, &Registers.Status.byte[2]); //Disabled reading interrupts
    //DeviceRead(regInterruptb, 1, &Registers.Status.byte[3]);
    DeviceRead(regStatus0, 1, &Registers.Status.byte[4]);
    DeviceRead(regStatus1, 1, &Registers.Status.byte[5]);
	USB_I2C_INFO("[mux]dump over.\n", __func__);

}

/******************************************************************************
*                         IRQ Handler                                         *
******************************************************************************/
static irqreturn_t fusb302_irq_handler(int irq, void *dev_id)   // void pointer is chip structure
{
	USB_I2C_INFO("%s +++ intx Triggered! State Machine Begin\n", __func__);

	int GPIOVal;
	GPIOVal = gpio_get_value(USB_MUX_GPIO);
	//USB_I2C_INFO("[irq][before core machine]GPIO %d state = %d , Pin CC1 state = %d , Pin CC2 state = %d\n", USB_MUX_GPIO ,GPIOVal,blnCCPinIsCC1,blnCCPinIsCC2);

	core_state_machine();
	
	USB_I2C_INFO("%s +++ State Machine Exit\n", __func__);

	return IRQ_HANDLED;
}

/******************************************************************************
*                         Timer Routines                                      *
******************************************************************************/
enum hrtimer_restart fusb_hrtimer_callback(struct hrtimer* timer)
{
	core_tick_at_100us();		// TODO: Uncomment when full driver is completed

	//USB_I2C_INFO("%s +++ Timer Timeout Success\n", __func__);
	ktime_temp = ktime_set(0,US_TO_NS(FUSB302_TIMER_PERIOD_US));//1*100*1000);//
	hrtimer_forward(&fusb302_hr_timer, ktime_get(),ktime_temp);

	return HRTIMER_RESTART;         // Do not requeue the timer
}

void fusb_i2c_enable_timers(void)
{
	ktime_t ktime;					// Ktime
	unsigned long delay_in_us = FUSB302_TIMER_PERIOD_US;

	ktime = ktime_set(0,US_TO_NS(delay_in_us));//1*100*1000);
	hrtimer_init(&fusb302_hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);		// Initialise timer
	fusb302_hr_timer.function = fusb_hrtimer_callback;				// Set timer callback
	hrtimer_start(&fusb302_hr_timer, ktime, HRTIMER_MODE_REL);		// Start timer
	USB_I2C_INFO("%s +++ HR Timer Enabled\n", __func__);
}

void fusb_i2c_disable_timers(void)
{
	int ret = hrtimer_cancel(&fusb302_hr_timer);						// Cancel timer
	if(ret<0)
	{
	    USB_I2C_INFO("%s --- HR Timer Cancel Fail: %d\n", __func__, ret);
	}
	USB_I2C_INFO("%s +++ HR Timer Disabled\n", __func__);
}

void fusb_i2c_delay_10us(__u32 delay10us)
{
	unsigned int us = 0;
	if (delay10us > MAX_DELAY_10US)
	{
		printk(KERN_ALERT "%s - Error: Delay of '%u' is too long! Must be less than '%u'.\n", __func__, delay10us, MAX_DELAY_10US);
		return;
	}

	us = delay10us * 10;                                    // Convert to microseconds (us)

    //USB_I2C_INFO("%s ++++ us = %d\n", __func__,us);

	if (us <= 10)                                           // Best practice is to use udelay() for < ~10us times
	{
		udelay(us);                                         // BLOCKING delay for < 10us
	}
	else if (us < 20000)                                    // Best practice is to use usleep_range() for 10us-20ms
	{
		// TODO - optimize this range, probably per-platform
		usleep_range(us, us + (us / 10));                   // Non-blocking sleep for at least the requested time, and up to the requested time + 10%
	}
	else                                                    // Best practice is to use msleep() for > 20ms
	{
		msleep(us / 1000);                                  // Convert to ms. Non-blocking, low-precision sleep
	}
}

/******************************************************************************
*                        FUSB302 I2C Routines                                *
******************************************************************************/
int fusb302_i2c_write_block(unsigned char regAddr, unsigned char length, unsigned char *data)
{
	int i;
	int ret = i2c_smbus_write_i2c_block_data(fusb_i2c_data->client, regAddr,
		length, data);
	if (ret < 0)
	{
		USB_I2C_INFO("%s +++ I2C Write Failure Error: %d\n", __func__, ret);
		return 0;
	}
	else
	{
		for(i=0;i<length;i++)
		{
			//USB_I2C_INFO("%s +++ I2C Write Succeed Reg:0x%2x Val:0x%2x %d\n", __func__, regAddr + i, data[i]);
		}
		return 1;
	}
}


int fusb302_i2c_read_block(unsigned char regAddr, unsigned char length, unsigned char *data)
{
int i;
	int ret = i2c_smbus_read_i2c_block_data(fusb_i2c_data->client, regAddr,
		length, data);
	if (ret < 0)
	{
		USB_I2C_INFO("%s +++ I2C Read Failure Error: %d\n", __func__, ret);
		return 0;
	}
	else
	{
		for(i=0;i<length;i++)
		{
			//USB_I2C_INFO("%s +++ I2C Read Succeed Reg:0x%2x Val:0x%2x\n", __func__, regAddr+i, data[i]);
		}
		return 1;
	}
}




/******************************************************************************
*                        FUSB302 GPIO Routines                                *
******************************************************************************/
void fusb_gpio_set_vbus_5v(int blnEnable)
{
	int ret;
	USB_I2C_INFO("%s +++\n", __func__);
	gpio_set_value(FUSB302_GPIO_VBUS_5V, blnEnable);
	ret = gpio_get_value(FUSB302_GPIO_VBUS_5V);
	USB_I2C_INFO("%s +++ VBUS GPIO Value: %d\n", __func__, ret);
}

int fusb_gpio_get_vbus_5v(void)
{
	USB_I2C_INFO("%s +++\n", __func__);
	int value;
	value = gpio_get_value(FUSB302_GPIO_VBUS_5V);

	if (value)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void fusb_gpio_set_vbus_other(int blneEnable)
{
	// Not Implemented for this platform
}

int fusb_gpio_get_vbus_other(void)
{
	return 0;
}

int int_fusb_gpio_get_int_n(void)
{
	USB_I2C_INFO("%s +++\n", __func__);
	int value;
	value = gpio_get_value(FUSB302_GPIO_INT_N);

	if (value)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

/******************************************************************************
*                        FUSB302 Set/Get Bit Routines                         *
******************************************************************************/
void fusb_i2c_data_set_int_n(int value)
{
	struct fusb302_i2c_data *fusb = fusb_i2c_data;
	fusb->int_n = value;
}

int fusb_i2c_data_get_int_n(void)
{
	struct fusb302_i2c_data *fusb = fusb_i2c_data;
	return fusb->int_n;
}

/******************************************************************************
*                        Create Proc                                          *
******************************************************************************/
ssize_t fusb302_read(struct file *filp, char __user *buffer, size_t count, loff_t *ppos)
{
	USB_I2C_INFO("%s +++\n", __func__);
    int len = 0;
	ssize_t ret=0;
	int RegVal,GPIOVal;
	char *buff;

	buff = kmalloc(300,GFP_KERNEL);
	if(!buff)
		return -ENOMEM;

    /*****  Dump Register Values in Kernel  ****/
	USB_I2C_INFO("%s : dump start.\n", __func__);
    DeviceRead(regDeviceID, 1, &Registers.DeviceID.byte);
    DeviceRead(regSwitches0, 1, &Registers.Switches.byte[0]);
    DeviceRead(regSwitches1, 1, &Registers.Switches.byte[1]);
    DeviceRead(regMeasure, 1, &Registers.Measure.byte);
    DeviceRead(regSlice, 1, &Registers.Slice.byte);
    DeviceRead(regControl0, 1, &Registers.Control.byte[0]);
    DeviceRead(regControl1, 1, &Registers.Control.byte[1]);
    DeviceRead(regControl2, 1, &Registers.Control.byte[2]);
    DeviceRead(regControl3, 1, &Registers.Control.byte[3]);
    DeviceRead(regMask, 1, &Registers.Mask.byte);
    DeviceRead(regPower, 1, &Registers.Power.byte);
    DeviceRead(regReset, 1, &Registers.Reset.byte);
    DeviceRead(regOCPreg, 1, &Registers.OCPreg.byte);
    DeviceRead(regMaska, 1, &Registers.MaskAdv.byte[0]);
    DeviceRead(regMaskb, 1, &Registers.MaskAdv.byte[1]);
    DeviceRead(regStatus0a, 1, &Registers.Status.byte[0]);
    DeviceRead(regStatus1a, 1, &Registers.Status.byte[1]);
    //DeviceRead(regInterrupta, 1, &Registers.Status.byte[2]); //Disabled reading interrupts
    //DeviceRead(regInterruptb, 1, &Registers.Status.byte[3]);
    DeviceRead(regStatus0, 1, &Registers.Status.byte[4]);
    DeviceRead(regStatus1, 1, &Registers.Status.byte[5]);
	USB_I2C_INFO("%s : dump over.\n", __func__);

    /*****  Dump Register Values in adb shell  ****/

    RegVal = DeviceRead(regDeviceID, 1, &Registers.DeviceID.byte);
	len += sprintf(buff + len, "0x0%x  0x%02x\n", regDeviceID ,Registers.DeviceID.byte);
    RegVal = DeviceRead(regSwitches0, 1, &Registers.Switches.byte[0]);
	len += sprintf(buff + len, "0x0%x  0x%02x\n", regSwitches0 ,Registers.Switches.byte[0]);
    RegVal = DeviceRead(regSwitches1, 1, &Registers.Switches.byte[1]);
	len += sprintf(buff + len, "0x0%x  0x%02x\n", regSwitches1 ,Registers.Switches.byte[1]);
    RegVal = DeviceRead(regMeasure, 1, &Registers.Measure.byte);
	len += sprintf(buff + len, "0x0%x  0x%02x\n", regMeasure ,Registers.Measure.byte);
    RegVal = DeviceRead(regSlice, 1, &Registers.Slice.byte);
	len += sprintf(buff + len, "0x0%x  0x%02x\n", regSlice ,Registers.Slice.byte);
    RegVal = DeviceRead(regControl0, 1, &Registers.Control.byte[0]);
	len += sprintf(buff + len, "0x0%x  0x%02x\n", regControl0 ,Registers.Control.byte[0]);
    RegVal = DeviceRead(regControl1, 1, &Registers.Control.byte[1]);
	len += sprintf(buff + len, "0x0%x  0x%02x\n", regControl1 ,Registers.Control.byte[1]);
    RegVal = DeviceRead(regControl2, 1, &Registers.Control.byte[2]);
	len += sprintf(buff + len, "0x0%x  0x%02x\n", regControl2 ,Registers.Control.byte[2]);
    RegVal = DeviceRead(regControl3, 1, &Registers.Control.byte[3]);
	len += sprintf(buff + len, "0x0%x  0x%02x\n", regControl3 ,Registers.Control.byte[3]);
    RegVal = DeviceRead(regMask, 1, &Registers.Mask.byte);
	len += sprintf(buff + len, "0x0%x  0x%02x\n", regMask ,Registers.Mask.byte);
    RegVal = DeviceRead(regPower, 1, &Registers.Power.byte);
	len += sprintf(buff + len, "0x0%x  0x%02x\n", regPower ,Registers.Power.byte);
    RegVal = DeviceRead(regReset, 1, &Registers.Reset.byte);
	len += sprintf(buff + len, "0x0%x  0x%02x\n", regReset ,Registers.Reset.byte);
    RegVal = DeviceRead(regOCPreg, 1, &Registers.OCPreg.byte);
	len += sprintf(buff + len, "0x0%x  0x%02x\n", regOCPreg ,Registers.OCPreg.byte);
    RegVal = DeviceRead(regMaska, 1, &Registers.MaskAdv.byte[0]);
	len += sprintf(buff + len, "0x0%x  0x%02x\n", regMaska ,Registers.MaskAdv.byte[0]);
    RegVal = DeviceRead(regMaskb, 1, &Registers.MaskAdv.byte[1]);
	len += sprintf(buff + len, "0x0%x  0x%02x\n", regMaskb ,Registers.MaskAdv.byte[1]);
    RegVal = DeviceRead(regStatus0a, 1, &Registers.Status.byte[0]);
	len += sprintf(buff + len, "0x0%x  0x%02x\n", regStatus0a ,Registers.Status.byte[0]);
    RegVal = DeviceRead(regStatus1a, 1, &Registers.Status.byte[1]);
	len += sprintf(buff + len, "0x0%x  0x%02x\n", regStatus1a ,Registers.Status.byte[1]);
    //DeviceRead(regInterrupta, 1, &Registers.Status.byte[2]); //Disabled reading interrupts
    //DeviceRead(regInterruptb, 1, &Registers.Status.byte[3]);
    RegVal = DeviceRead(regStatus0, 1, &Registers.Status.byte[4]);
	len += sprintf(buff + len, "0x0%x  0x%02x\n", regStatus0 ,Registers.Status.byte[4]);
    RegVal = DeviceRead(regStatus1, 1, &Registers.Status.byte[5]);
	len += sprintf(buff + len, "0x0%x  0x%02x\n", regStatus1 ,Registers.Status.byte[5]);
    /*****  Dump GPIO 160 Details in adb shell  ****/

    GPIOVal = gpio_get_value(USB_MUX_GPIO);
	len += sprintf(buff + len, "GPIO %d state = %d , Pin CC1 state = %d , Pin CC2 state = %d\n", USB_MUX_GPIO ,GPIOVal,blnCCPinIsCC1,blnCCPinIsCC2);

	ret = simple_read_from_buffer(buffer,count,ppos,buff,len);


    return ret;
}

int proc_reg;
int proc_value;
char  proc_buf[64];

ssize_t fusb302_write(struct file *filp, char __user *buffer, size_t count, loff_t *ppos)
{
    USB_I2C_INFO("%s +++\n", __func__);
	int ret = 0;

    if (copy_from_user(proc_buf, buffer, count)) {
        USB_I2C_INFO("read data from user space error\n");
        return -EFAULT;
    }
    sscanf(proc_buf, "%x %x\n", &proc_reg, &proc_value);
    USB_I2C_INFO("%d,%d\n", proc_reg, proc_value);

	/* turn to number */
	int adb_com = 999;
	char adb_val;

	adb_com = proc_reg;
	adb_val = (char) proc_value;

	USB_I2C_INFO("adb_com = %x , adb_val = %x\n", adb_com, adb_val);

    /* check command details */
    int result = -1;

	USB_I2C_INFO("write register %x with value %x\n", adb_com, adb_val);

	/* start to process command(write register value in adb shell) */

    if( (adb_com == regDeviceID))
	{
		Registers.DeviceID.byte = adb_val;
		result = DeviceWrite(regDeviceID, 1, &Registers.DeviceID.byte);
	}
    if( (adb_com == regSwitches0))
	{
		Registers.Switches.byte[0] = adb_val;
		result = DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);
	}
    if( (adb_com == regSwitches1))
	{
		Registers.Switches.byte[1] = adb_val;
		result = DeviceWrite(regSwitches1, 1, &Registers.Switches.byte[1]);
	}
    if( (adb_com == regMeasure))
	{
		Registers.Measure.byte = adb_val;
		result = DeviceWrite(regMeasure, 1, &Registers.Measure.byte);
	}
    if( (adb_com == regSlice))
	{ 
		Registers.Slice.byte = adb_val;
		result = DeviceWrite(regSlice, 1, &Registers.Slice.byte);
	}
    if( (adb_com == regControl0))
	{
		Registers.Control.byte[0] = adb_val;
		result = DeviceWrite(regControl0, 1, &Registers.Control.byte[0]);
	}
    if( (adb_com == regControl1))
	{
		Registers.Control.byte[1] = adb_val;
		result = DeviceWrite(regControl1, 1, &Registers.Control.byte[1]);
	}
    if( (adb_com == regControl2))
	{
		Registers.Control.byte[2] = adb_val;
		result = DeviceWrite(regControl2, 1, &Registers.Control.byte[2]);
	}
    if( (adb_com == regControl3))
	{
		Registers.Control.byte[3] = adb_val;
		result = DeviceWrite(regControl3, 1, &Registers.Control.byte[3]);
	}
    if( (adb_com == regMask))
	{
		Registers.Mask.byte = adb_val;
		result = DeviceWrite(regMask, 1, &Registers.Mask.byte);
	}
    if( (adb_com == regPower))
	{
		Registers.Power.byte = adb_val;
		result = DeviceWrite(regPower, 1, &Registers.Power.byte);
	}
    if( (adb_com == regReset))
	{
		Registers.Reset.byte = adb_val;
		result = DeviceWrite(regReset, 1, &Registers.Reset.byte);
	}
    if( (adb_com == regOCPreg))
	{
		Registers.OCPreg.byte = adb_val;
		result = DeviceWrite(regOCPreg, 1, &Registers.OCPreg.byte);
	}
    if( (adb_com == regMaska))
	{
		Registers.MaskAdv.byte[0] = adb_val;
		result = DeviceWrite(regMaska, 1, &Registers.MaskAdv.byte[0]);
	}
    if( (adb_com == regMaskb))
	{
		Registers.MaskAdv.byte[1] = adb_val;
		result = DeviceWrite(regMaskb, 1, &Registers.MaskAdv.byte[1]);
	}
    if( (adb_com == regStatus0a))
	{
		Registers.Status.byte[0] = adb_val;
		result = DeviceWrite(regStatus0a, 1, &Registers.Status.byte[0]);
	}
    if( (adb_com == regStatus1a))
	{
		Registers.Status.byte[1] = adb_val;
		result = DeviceWrite(regStatus1a, 1, &Registers.Status.byte[1]);
	}
	//DeviceRead(regInterrupta, 1, &Registers.Status.byte[2]); //Disabled reading interrupts
	//DeviceRead(regInterruptb, 1, &Registers.Status.byte[3]);
    if( (adb_com == regStatus0))
	{
		Registers.Status.byte[4] = adb_val;
		result = DeviceWrite(regStatus0, 1, &Registers.Status.byte[4]);
	}
    if( (adb_com == regStatus1))
	{
		Registers.Status.byte[5] = adb_val;
		result = DeviceWrite(regStatus1, 1, &Registers.Status.byte[5]);
    }
	/* start to process command(write GPIO 160 value in adb shell) */
    if( (adb_com == 0xA0))//USB_MUX_GPIO)) // 0xA0 ---> 160
	{
		gpio_set_value(USB_MUX_GPIO,0);//adb_val);
    }

	return count;
}

int init_asus_for_fusb302(void)
{   USB_I2C_INFO("init_asus_for_fusb302\n");
	struct proc_dir_entry *entry=NULL;

	static struct file_operations fusb302Reg_fop = {
	    .read  = fusb302_read,
		.write = fusb302_write,
	};
	entry = proc_create("fusb302", 0666,NULL, &fusb302Reg_fop);
	if(!entry)
	{
		USB_I2C_INFO("create /proc/fusb302Reg fail\n");
	}

	return 0;
}

/******************************************************************************
*                         Early Suspend                                       *
******************************************************************************/
int fusb302_i2c_early_suspend_read_block(unsigned char regAddr, unsigned char length, unsigned char *data)
{
int i;
	int ret = i2c_smbus_read_i2c_block_data(fusb_i2c_data->client, regAddr,
			length, data);
	if (ret < 0)
	{
		USB_I2C_INFO("%s +++ I2C Read Failure Error: %d\n", __func__, ret);
		return 0;
	}
	else
	{
		for(i=0;i<length;i++)
		{
			USB_I2C_INFO("%s +++ I2C Read Succeed Reg:0x%2x Val:0x%2x\n", __func__, regAddr+i, data[i]);
		}
		return 1;
	}
}
static void fusb302_early_suspend(struct early_suspend *h)
{
	USB_I2C_INFO("%s ++\n", __func__);
	USB_I2C_INFO("[ES]dump start.\n", __func__);

    fusb302_i2c_early_suspend_read_block(regDeviceID, 1, &Registers.DeviceID.byte);
    fusb302_i2c_early_suspend_read_block(regSwitches0, 1, &Registers.Switches.byte[0]);
    fusb302_i2c_early_suspend_read_block(regSwitches1, 1, &Registers.Switches.byte[1]);
    fusb302_i2c_early_suspend_read_block(regMeasure, 1, &Registers.Measure.byte);
    fusb302_i2c_early_suspend_read_block(regSlice, 1, &Registers.Slice.byte);
    fusb302_i2c_early_suspend_read_block(regControl0, 1, &Registers.Control.byte[0]);
    fusb302_i2c_early_suspend_read_block(regControl1, 1, &Registers.Control.byte[1]);
    fusb302_i2c_early_suspend_read_block(regControl2, 1, &Registers.Control.byte[2]);
    fusb302_i2c_early_suspend_read_block(regControl3, 1, &Registers.Control.byte[3]);
    fusb302_i2c_early_suspend_read_block(regMask, 1, &Registers.Mask.byte);
    fusb302_i2c_early_suspend_read_block(regPower, 1, &Registers.Power.byte);
    fusb302_i2c_early_suspend_read_block(regReset, 1, &Registers.Reset.byte);
    fusb302_i2c_early_suspend_read_block(regOCPreg, 1, &Registers.OCPreg.byte);
    fusb302_i2c_early_suspend_read_block(regMaska, 1, &Registers.MaskAdv.byte[0]);
    fusb302_i2c_early_suspend_read_block(regMaskb, 1, &Registers.MaskAdv.byte[1]);
    fusb302_i2c_early_suspend_read_block(regStatus0a, 1, &Registers.Status.byte[0]);
    fusb302_i2c_early_suspend_read_block(regStatus1a, 1, &Registers.Status.byte[1]);
    //fusb302_i2c_early_suspend_read_block(regInterrupta, 1, &Registers.Status.byte[2]); //Disabled reading interrupts
    //fusb302_i2c_early_suspend_read_block(regInterruptb, 1, &Registers.Status.byte[3]);
    fusb302_i2c_early_suspend_read_block(regStatus0, 1, &Registers.Status.byte[4]);
    fusb302_i2c_early_suspend_read_block(regStatus1, 1, &Registers.Status.byte[5]);

	USB_I2C_INFO("[ES]dump over.\n", __func__);
    
}

static void fusb302_config_earlysuspend(struct fusb302_i2c_data *fusb)
{	wake_lock(&fusb->wake_lock);
	fusb->es.level = 4;//EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 2;
	fusb->es.suspend = fusb302_early_suspend;
	fusb->es.resume = NULL;//fusb302_late_resume;
	register_early_suspend(&fusb->es);
}
/******************************************************************************
*                         Probe for fusb302_driver .probe                     *
******************************************************************************/
static int fusb302_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	//return 0;
	struct fusb302_i2c_data *fusb;	// Create fusb302 struct pointer
    int err;
	//int ret = 0;                    // For checking proc
	USB_I2C_INFO("%s +++ Probe start\n", __func__);

	fusb = devm_kzalloc(&i2c->dev, sizeof(struct fusb302_i2c_data), GFP_KERNEL);	// Allocate memory for struct
	if (!fusb) {
		dev_err(&i2c->dev, "private data alloc fail\n");
		return -1;
	}

	fusb_i2c_data = fusb;										// Assign memory to fusb302 struct pointer

	fusb->int_n = 0;											// Clear INT_N by default

	i2c_set_clientdata(i2c, fusb);								// Set up I2C
	fusb->client = i2c;

	err = gpio_request_array(fusb_gpios, ARRAY_SIZE(fusb_gpios));		// Request gpios
	if (err < 0)
	{
		USB_I2C_INFO("%s --- GPIO Request Fail Error: %d\n", __func__, err);
	}

	/* Enable timers */
	fusb_i2c_enable_timers();

	core_initialize();	// Initialize state machine

	core_enable_typec(1);	// Enable type-c

	fusb->irq_num = gpio_to_irq(FUSB302_GPIO_INT_N);						// Get IRQ number
    //return 0;
	/* This will start the state machine */
	err = devm_request_threaded_irq(&fusb->client->dev, fusb->irq_num, NULL, fusb302_irq_handler, IRQF_TRIGGER_LOW | IRQF_ONESHOT, "Int_N", fusb);				// Request IRQ and set to level low, auto-disable after handled
	if (err < 0)
	{
		USB_I2C_INFO("%s --- IRQ Request Fail Error: %d\n", __func__, err);
	}

/* for create proc */
	int ret =0;
	ret = init_asus_for_fusb302();
	if (ret) {
		USB_I2C_INFO("Unable to create proc init_asus_for_fusb302\n");
	}
/* for early suspend */
    fusb302_config_earlysuspend(fusb);

/* only for test : enter */
    FSC_U16 c_test;
    c_test = core_get_advertised_current();    
	return 0;
}

/******************************************************************************
*                         Remove for fusb302_driver .remove                   *
******************************************************************************/
static int fusb302_remove(struct i2c_client *i2c)
{

	struct fusb302_i2c_data *fusb;
	USB_I2C_INFO("%s +++\n", __func__);
	fusb = fusb_i2c_data;

	/* disable timers */
	fusb_i2c_disable_timers();
	
	/* free irq */
	free_irq(fusb->irq_num, NULL);

	/* free gpios */
	gpio_free_array(fusb_gpios, ARRAY_SIZE(fusb_gpios));

	/*i2c_unregister_device(i2c); */
	kfree(i2c_get_clientdata(i2c));
	return 0;
}

/******************************************************************************
*                         Device ID for fusb302_driver .id_table              *
******************************************************************************/
static const struct i2c_device_id fusb302_id[] = {
	{ FUSB302_I2C_NAME, 0 },
	{}
};

/******************************************************************************
*                         Board Info for fusb302_driver init                  *
******************************************************************************/
static struct i2c_board_info __initdata fusb302_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO(FUSB302_I2C_NAME, (FUSB302_I2C_SLAVE_ADDRESS)),
	},
};

/******************************************************************************
*                        Driver Struct                                        *
******************************************************************************/
static struct i2c_driver fusb302_i2c_driver = {
	.driver = {
		   .name = FUSB302_I2C_NAME,
		   .owner = THIS_MODULE,
		   },
	.probe = fusb302_probe,
	.remove = fusb302_remove,
	.id_table = fusb302_id,
};

/******************************************************************************
*                        Init and Exit Routines                               *
******************************************************************************/
int fusb_drv_support = 0;
static int __init fusb302_setup(char *str)
{
	if (strncmp(str, "fusb302", 7) == 0) {
			fusb_drv_support = 1;
	}
	return 1;
}
__setup("asus.dbg.typec=", fusb302_setup);

static int __init fusb302_i2c_init(void)
{
	USB_I2C_INFO("%s +++\n", __func__);

	if (!fusb_drv_support) {
        USB_I2C_INFO("%s skip this driver...\n", __func__);
        return 0;
    }

	i2c_register_board_info(FUSB302_I2C_NUM, fusb302_i2c_boardinfo,
				ARRAY_SIZE(fusb302_i2c_boardinfo));

	return i2c_add_driver(&fusb302_i2c_driver);
}

static void __exit fusb302_i2c_exit(void)
{
	USB_I2C_INFO("%s +++\n", __func__);
	i2c_del_driver(&fusb302_i2c_driver);
}

module_init(fusb302_i2c_init);
module_exit(fusb302_i2c_exit);
