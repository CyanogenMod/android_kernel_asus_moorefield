/*
 * Sweep2Wake driver for Zenfone 2
 * 
 * 
 * Author: andip71 & TheSSJ, 10.12.2014
 * 
 * Version 1.0.1
 *
 * Credits for initial implementation to Dennis Rassmann <showp1984@gmail.com>
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/hrtimer.h>
#include <linux/earlysuspend.h>
#include <linux/cpufreq.h>
#include <linux/HWVersion.h>

#define FTXXXX_NAME	"ftxxxx_ts"

/*****************************************/
/* Module/driver data */
/*****************************************/

#define DRIVER_AUTHOR "andip71 (Lord Boeffla), TheSSJ"
#define DRIVER_DESCRIPTION "Sweep2sleep for Zenfone 2"
#define DRIVER_VERSION "1.0.2"
#define LOGTAG "s2s: "

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPLv2");


//Basically the order of triggering the sleep command is
// BACK HOME MENU

/*****************************************/
/* Variables, structures and pointers */
/*****************************************/

int s2s = 1;
int is_ze550ml = 0;
int y_boundary = 1920; //y value for ZE551ML
int x_boundary = 1080; //x value for ZE551ML
static int debug = 1;
static int pwrkey_dur = 60;
static bool scr_suspended;
static int touch_x = 0;
static int touch_y = 0;
static int key_press = 0;
static bool touch_x_called = false;
static bool touch_y_called = false;
//First index is BACK, second HOME, third MENU, thus we need 2 items in our array as the last keycheck can be done dynamically
#define MAXSIZE 2
static int pressArray[MAXSIZE];

//Size of the area (pixel) where the capacitive buttons are located
#define BLACK_AREA 500

static struct input_dev * sweep2sleep_pwrdev;
static DEFINE_MUTEX(pwrkeyworklock);
static struct workqueue_struct *s2s_input_wq;
static struct work_struct s2s_input_work;


//determination which device is running this kernel
extern int Read_PROJ_ID(void);
extern int Read_HW_ID(void);

/*****************************************/
// Internal functions
/*****************************************/

/* PowerKey work function */
static void sweep2sleep_presspwr(struct work_struct * sweep2sleep_presspwr_work) 
{
	if (!mutex_trylock(&pwrkeyworklock))
		return;
	
	input_event(sweep2sleep_pwrdev, EV_KEY, KEY_POWER, 1);
	input_event(sweep2sleep_pwrdev, EV_SYN, 0, 0);
	msleep(pwrkey_dur);
	
	input_event(sweep2sleep_pwrdev, EV_KEY, KEY_POWER, 0);
	input_event(sweep2sleep_pwrdev, EV_SYN, 0, 0);
	msleep(pwrkey_dur);
    
    mutex_unlock(&pwrkeyworklock);
	return;
}
static DECLARE_WORK(sweep2sleep_presspwr_work, sweep2sleep_presspwr);


/* PowerKey trigger */
static void sweep2sleep_pwrtrigger(void) 
{
	schedule_work(&sweep2sleep_presspwr_work);

    return;
}


/* Reset sweep2sleep */
static void sweep2sleep_reset(void) 
{
	int i;
	for (i =0; i<MAXSIZE; i++)
		pressArray[i] = 0;
}


/* sweep2sleep main function */
static void detect_sweep2sleep(int pressedKey)
{
	if (debug)
		pr_info(LOGTAG"KEY_BACK=%d; KEY_HOME=%d, KEY_MENU=%d\n", (pressedKey == KEY_BACK), (pressedKey == KEY_HOME, (pressedKey == KEY_MENU)));

	if ((scr_suspended == false) && (s2s != 0)) 
	{	
		// Static gestures
		if(pressArray[0] != KEY_BACK)
		{
			if(pressedKey == KEY_BACK)
			{
				//we register KEY_BACK as pressed and break out, there is nothing to do anymore
				if(debug)
					pr_info(LOGTAG"registered KEY_BACK");
				pressArray[0] = KEY_BACK;
				return;
			}
			else
			{
				//reset and break out
				sweep2sleep_reset();
				return;
			}
		}
		//ignore repeated KEY_BACK action
		if(pressArray[0] == KEY_BACK && pressedKey == KEY_BACK)
			return;
		
		//so KEY_BACK was already registered, we can continue
		if(pressArray[1] != KEY_HOME)
		{
			if(pressedKey == KEY_HOME)
			{
				if(debug)
					pr_info(LOGTAG"registered KEY_MENU");
				pressArray[1] = KEY_HOME;
				//register and break out
				return;
			}
			else
			{
				//reset and break out
				sweep2sleep_reset();
				return;
			}
		}
		//again, filter out repeated press
		if(pressArray[1] == KEY_HOME && pressedKey == KEY_HOME)
			return;
		
		//Last key check
		if(pressedKey == KEY_MENU)
		{
			if(debug)
				pr_info(LOGTAG"KEY_MENU pressed, Sweep2sleep activated!\n");
			sweep2sleep_pwrtrigger();
			sweep2sleep_reset();
		}
		else
			sweep2sleep_reset();
	}
	return;
}


/* input callback function */
static void s2s_input_callback(struct work_struct *unused) 
{
	if (s2s)
		detect_sweep2sleep(key_press);

	touch_x = 0;
	touch_y = 0;
	key_press = 0;
	return;
}


/* input event dispatcher */
static void s2s_input_event(struct input_handle *handle, unsigned int type,
				unsigned int code, int value)
{
	if (!s2s)
		return;
	
	if (code == ABS_MT_POSITION_X)
	{
		touch_x = value;
		touch_x_called = true;
	}

	if (code == ABS_MT_POSITION_Y) 
	{
		touch_y = value;
		touch_y_called = true;
	}

	if (touch_x_called && touch_y_called) 
	{
		touch_x_called = false;
		touch_y_called = false;
		
		
		//if sweep2sleep isn't activated, we don't need to evaluate anything. Reset and break out
		if(!s2s)
		{
			touch_x = 0;
			touch_y = 0;
			return;
		}
			
			
		if((touch_y >= y_boundary) && (touch_y <= (y_boundary + BLACK_AREA)))
		{
			if(debug)
				pr_info(LOGTAG"Registered touch, x = %d, y = %d\n", touch_x, touch_y);
			
			if(!is_ze550ml) //ZE551ML
			{
				if(touch_x >= 200 && touch_x <= 300)
					key_press = KEY_BACK;
				if(touch_x >= 500 && touch_x <= 600)
					key_press = KEY_HOME;
				if(touch_x >= 850 && touch_x <= 975)
					key_press = KEY_MENU;
			}
			else //if is_ze550ml //ZE550ML, needs more testing
			{
				if(touch_x >= 130 && touch_x <= 200)
					key_press = KEY_BACK;
				if(touch_x >= 350 && touch_x <= 430)
					key_press = KEY_HOME;
				if(touch_x >= 560 && touch_x <= 650)
					key_press = KEY_MENU;
			}
			
			if(key_press != 0)
			{
				queue_work_on(0, s2s_input_wq, &s2s_input_work);
			}
		}
		else
			sweep2sleep_reset();
	}
	
}

/* input filter function */
static int input_dev_filter(struct input_dev *dev) 
{
	if (strstr(dev->name, "touch") || strstr(dev->name, FTXXXX_NAME)) 
		return 0;

	return 1;
}


/* connect to input stream */
static int s2s_input_connect(struct input_handler *handler,
				struct input_dev *dev, const struct input_device_id *id) 
{
	struct input_handle *handle;
	int error;

	if (input_dev_filter(dev))
		return -ENODEV;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "s2s";

	error = input_register_handle(handle);
	if (error)
		goto err1;

	error = input_open_device(handle);
	if (error)
		goto err2;

	return 0;

err2:
	input_unregister_handle(handle);
err1:
	kfree(handle);
	return error;
}


/* disconnect from input stream */
static void s2s_input_disconnect(struct input_handle *handle) 
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}


static const struct input_device_id s2s_ids[] = 
{
	{ .driver_info = 1 },
	{ },
};


static struct input_handler s2s_input_handler = 
{
	.event		= s2s_input_event,
	.connect	= s2s_input_connect,
	.disconnect	= s2s_input_disconnect,
	.name		= "s2s_inputreq",
	.id_table	= s2s_ids,
};


/* callback functions to detect screen on and off events */
static void early_suspend_screen_off(struct early_suspend *h)
{
	scr_suspended = true;
}

static void late_resume_screen_on(struct early_suspend *h)
{
	scr_suspended = false;
}

static struct early_suspend screen_detect = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
	.suspend = early_suspend_screen_off,
	.resume = late_resume_screen_on,
};

/*****************************************/
// Sysfs definitions 
/*****************************************/

static ssize_t sweep2sleep_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", s2s);
}


static ssize_t sweep2sleep_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int ret = -EINVAL;
	int val;

	// read values from input buffer
	ret = sscanf(buf, "%d", &val);

	if (ret != 1)
		return -EINVAL;
		
	// store if valid data
	if ( val > 0x00 )
		s2s = 1;
	else // val <= 0x00
		s2s = 0;

	return count;
}

static DEVICE_ATTR(sweep2sleep, (S_IWUSR|S_IRUGO),
	sweep2sleep_show, sweep2sleep_store);


static ssize_t debug_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", debug);
}

static ssize_t debug_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int ret = -EINVAL;
	int val;

	// read values from input buffer
	ret = sscanf(buf, "%d", &val);

	if (ret != 1)
		return -EINVAL;
		
	// store if valid data
	if (((val == 0) || (val == 1)))
		debug = val;

	return count;
}

static DEVICE_ATTR(sweep2sleep_debug, (S_IWUSR|S_IRUGO),
	debug_show, debug_store);


static ssize_t version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", DRIVER_VERSION);
}

static DEVICE_ATTR(sweep2sleep_version, (S_IWUSR|S_IRUGO),
	version_show, NULL);


/*****************************************/
// Driver init and exit functions
/*****************************************/

struct kobject *android_touch_kobj;
EXPORT_SYMBOL_GPL(android_touch_kobj);

static int __init sweep2sleep_init(void)
{
	int rc = 0;
	
	sweep2sleep_pwrdev = input_allocate_device();
	if (!sweep2sleep_pwrdev) 
	{
		pr_err(LOGTAG"Can't allocate suspend autotest power button\n");
		return -EFAULT;
	}

	input_set_capability(sweep2sleep_pwrdev, EV_KEY, KEY_POWER);
	input_set_capability(sweep2sleep_pwrdev, EV_KEY, KEY_BACK);
	input_set_capability(sweep2sleep_pwrdev, EV_KEY, KEY_HOME);
	input_set_capability(sweep2sleep_pwrdev, EV_KEY, KEY_MENU);
	
	sweep2sleep_pwrdev->name = "s2s_pwrkey";
	sweep2sleep_pwrdev->phys = "s2s_pwrkey/input0";
	rc = input_register_device(sweep2sleep_pwrdev);
	if (rc) {
		pr_err(LOGTAG"%s: input_register_device err=%d\n", __func__, rc);
		goto err1;
	}

	s2s_input_wq = create_workqueue("s2siwq");
	if (!s2s_input_wq) 
	{
		pr_err(LOGTAG"%s: Failed to create s2siwq workqueue\n", __func__);
		goto err2;
	}
	
	INIT_WORK(&s2s_input_work, s2s_input_callback);
	rc = input_register_handler(&s2s_input_handler);
	if (rc)
	{
		pr_err(LOGTAG"%s: Failed to register s2s_input_handler\n", __func__);
		goto err3;
	}
	
	register_early_suspend(&screen_detect);

	android_touch_kobj = kobject_create_and_add("android_touch", NULL) ;
	if (android_touch_kobj == NULL) 
	{
		pr_err(LOGTAG"%s: android_touch_kobj create_and_add failed\n", __func__);
		goto err4;
	}

	rc = sysfs_create_file(android_touch_kobj, &dev_attr_sweep2sleep.attr);
	if (rc) 
	{
		pr_warn(LOGTAG"%s: sysfs_create_file failed for sweep2sleep\n", __func__);
		goto err4;
	}
	
	rc = sysfs_create_file(android_touch_kobj, &dev_attr_sweep2sleep_debug.attr);
	if (rc) 
	{
		pr_warn(LOGTAG"%s: sysfs_create_file failed for sweep2sleep_debug\n", __func__);
		goto err4;
	}
	
	rc = sysfs_create_file(android_touch_kobj, &dev_attr_sweep2sleep_version.attr);
	if (rc) 
	{
		pr_warn(LOGTAG"%s: sysfs_create_file failed for sweep2sleep_version\n", __func__);
		goto err4;
	}
	
	/*rc = sysfs_create_file(android_touch_kobj, &dev_attr_is_ze550ml.attr);
	if (rc) 
	{
		pr_warn(LOGTAG"%s: sysfs_create_file failed for is_ze550ml\n", __func__);
		goto err4;
	}*/
	
	
	if (Read_HW_ID() == HW_ID_MP)
	{
		if (Read_PROJ_ID() == PROJ_ID_ZE550ML)
			is_ze550ml = 1;
		else
			is_ze550ml = 0;
		
		y_boundary = (!is_ze550ml ? 1920 : 1280);
		x_boundary = (!is_ze550ml ? 1080 : 720);
	}
	
	return 0;

err4:
	unregister_early_suspend(&screen_detect);
	input_unregister_handler(&s2s_input_handler);
err3:
	destroy_workqueue(s2s_input_wq);
err2:
	input_unregister_device(sweep2sleep_pwrdev);
err1:
	input_free_device(sweep2sleep_pwrdev);
	return -EFAULT;
}


static void __exit sweep2sleep_exit(void)
{
	unregister_early_suspend(&screen_detect);
	input_unregister_handler(&s2s_input_handler);
	destroy_workqueue(s2s_input_wq);
	input_unregister_device(sweep2sleep_pwrdev);
	input_free_device(sweep2sleep_pwrdev);
	return;
}


late_initcall(sweep2sleep_init);
module_exit(sweep2sleep_exit);
