/*
 *      Intel_SCU 0.2:  An Intel SCU IOH Based Watchdog Device
 *			for Intel part #(s):
 *				- AF82MP20 PCH
 *
 *      Copyright (C) 2009-2010 Intel Corporation. All rights reserved.
 *
 *      This program is free software; you can redistribute it and/or
 *      modify it under the terms of version 2 of the GNU General
 *      Public License as published by the Free Software Foundation.
 *
 *      This program is distributed in the hope that it will be
 *      useful, but WITHOUT ANY WARRANTY; without even the implied
 *      warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 *      PURPOSE.  See the GNU General Public License for more details.
 *      You should have received a copy of the GNU General Public
 *      License along with this program; if not, write to the Free
 *      Software Foundation, Inc., 59 Temple Place - Suite 330,
 *      Boston, MA  02111-1307, USA.
 *      The full GNU General Public License is included in this
 *      distribution in the file called COPYING.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
/* See Documentation/watchdog/intel-scu-watchdog.txt */

#include <linux/compiler.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/fs.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/kernel_stat.h>
#include <linux/delay.h>
#include <linux/signal.h>
#include <linux/sfi.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/atomic.h>
#include <linux/wakelock.h>
#include <linux/rpmsg.h>
#include <linux/intel_mid_pm.h>
#include <linux/nmi.h>
#include <asm/irq.h>
#include <asm/intel_scu_ipc.h>
#include <asm/intel_scu_ipcutil.h>
#include <asm/apb_timer.h>
#include <asm/intel_mid_rpmsg.h>
#include <asm/intel-mid.h>

#include "intel_scu_watchdog.h"

/* Adjustment flags */
/* from config file */
/* #define CONFIG_DISABLE_SCU_WATCHDOG */
/* local */
#define CONFIG_INTEL_SCU_SOFT_LOCKUP
#define CONFIG_DEBUG_WATCHDOG

/* Defines */
#define IPC_SET_WATCHDOG_TIMER  0xF8
#define IPC_SET_SUB_LOAD_THRES  0x00
#define IPC_SET_SUB_DISABLE     0x01
#define IPC_SET_SUB_KEEPALIVE   0x02
#define IPC_SET_SUB_COLDOFF     0x03
#define IPC_SET_SUB_COLDRESET   0x04
#define IPC_SET_SUB_COLDBOOT    0x05
#define IPC_SET_SUB_DONOTHING   0x06

#define STRING_RESET_TYPE_MAX_LEN 11
#define STRING_COLD_OFF "COLD_OFF"
#define STRING_COLD_RESET "COLD_RESET"
#define STRING_COLD_BOOT "COLD_BOOT"

#ifdef CONFIG_DEBUG_FS
#define SECURITY_WATCHDOG_ADDR  0xFF222230
#define STRING_NONE "NONE"
#endif

#define WDIOC_SETTIMERTIMEOUT     _IOW(WATCHDOG_IOCTL_BASE, 11, int)
#define WDIOC_GETTIMERTIMEOUT     _IOW(WATCHDOG_IOCTL_BASE, 12, int)

/* Statics */
static struct intel_scu_watchdog_dev watchdog_device;
static struct wake_lock watchdog_wake_lock;
static DECLARE_WAIT_QUEUE_HEAD(read_wq);
static unsigned char osnib_reset = OSNIB_WRITE_VALUE;
static int reset_type_to_string(int reset_type, char *string);
static int string_to_reset_type(const char *string, int *reset_type);

/* The read function (intel_scu_read) waits for the warning_flag to */
/* be set by the watchdog interrupt handler. */
/* When warning_flag is set intel_scu_read wakes up the user level */
/* process, which is responsible for refreshing the watchdog timer */
static int warning_flag;

/* Module params */
static bool kicking_active = true;
#ifdef CONFIG_DEBUG_WATCHDOG
module_param(kicking_active, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(kicking_active,
		"Deactivate the kicking will result in a cold reset"
		"after a while");
#endif

static bool disable_kernel_watchdog;
#ifdef CONFIG_DISABLE_SCU_WATCHDOG
/*
 * Please note that we are using a config CONFIG_DISABLE_SCU_WATCHDOG
 * because this boot parameter should only be settable in a developement
 */
module_param(disable_kernel_watchdog, bool, S_IRUGO);
MODULE_PARM_DESC(disable_kernel_watchdog,
		"Disable kernel watchdog"
		"Set to 0, watchdog started at boot"
		"and left running; Set to 1; watchdog"
		"is not started until user space"
		"watchdog daemon is started; also if the"
		"timer is started by the iafw firmware, it"
		"will be disabled upon initialization of this"
		"driver if disable_kernel_watchdog is set");
#endif

static int pre_timeout = DEFAULT_PRETIMEOUT;
module_param(pre_timeout, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(pre_timeout,
		"Watchdog pre timeout"
		"Time between interrupt and resetting the system"
		"The range is from 1 to 160");

static int timeout = DEFAULT_TIMEOUT;
module_param(timeout, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(timeout,
		"Default Watchdog timer setting"
		"Complete cycle time"
		"The range is from 1 to 170"
		"This is the time for all keep alives to arrive");

static int timer_timeout = DEFAULT_TIMER_DURATION;
module_param(timer_timeout, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(timer_timeout,
		"Watchdog timer timeout"
		"Time between timer interrupt and resetting the system");

static bool reset_on_release = true;

#ifdef CONFIG_INTEL_SCU_SOFT_LOCKUP
/*
 * heartbeats: cpu last kstat.system times
 * beattime : jeffies at the sample time of heartbeats.
 * SOFT_LOCK_TIME : some time out in sec after warning interrupt.
 * dump_softloc_debug : called on SOFT_LOCK_TIME time out after scu
 *	interrupt to log data to logbuffer and emmc-panic code,
 *	SOFT_LOCK_TIME needs to be < SCU warn to reset time
 *	which is currently thats 15 sec.
 *
 * The soft lock works be taking a snapshot of kstat_cpu(i).cpustat.system at
 * the time of the warning interrupt for each cpu.  Then at SOFT_LOCK_TIME the
 * amount of time spend in system is computed and if its within 10 ms of the
 * total SOFT_LOCK_TIME on any cpu it will dump the stack on that cpu and then
 * calls panic.
 *
 */
static u64 heartbeats[NR_CPUS];
static u64 beattime;
#define SOFT_LOCK_TIME 10
static void dump_softlock_debug(unsigned long data);
DEFINE_TIMER(softlock_timer, dump_softlock_debug, 0, 0);

static struct rpmsg_instance *watchdog_instance;

/* time is about to run out and the scu will reset soon.  quickly
 * dump debug data to logbuffer and emmc via calling panic before lights
 * go out.
 */
static void smp_dumpstack(void *info)
{
	dump_stack();
}

static void dump_softlock_debug(unsigned long data)
{
	int i, reboot;
	u64 system[NR_CPUS], num_jifs;

	num_jifs = jiffies - beattime;
	for_each_possible_cpu(i) {
		system[i] = kcpustat_cpu(i).cpustat[CPUTIME_SYSTEM] -
				heartbeats[i];
	}

	reboot = 0;

	for_each_possible_cpu(i) {
		if ((num_jifs - cputime_to_jiffies(system[i])) <
						msecs_to_jiffies(10)) {
			WARN(1, "cpu %d wedged\n", i);
			smp_call_function_single(i, smp_dumpstack, NULL, 1);
			reboot = 1;
		}
	}

	if (reboot) {
		panic_timeout = 10;
		trigger_all_cpu_backtrace();
		panic("Soft lock on CPUs\n");
	}
}
#endif /* CONFIG_INTEL_SCU_SOFT_LOCKUP */

/* Check current timeouts */
static int check_timeouts(int pre_time, int timer_time, int timeout_time)
{
	if (timer_time+pre_time < timeout_time)
		return 0;

	return -EINVAL;
}

/* Set the different timeouts needed by the SCU FW */
static int watchdog_set_timeouts(int timer_threshold, int warning_pretimeout,
			    int reset_timeout)
{
	u32	ipc_wbuf[3];
	int	ret = 0;
	u32	freq = watchdog_device.timer7_tbl_ptr->freq_hz;

	ipc_wbuf[0] = timer_threshold * freq;
	ipc_wbuf[1] = warning_pretimeout * freq;
	ipc_wbuf[2] = (reset_timeout - timer_threshold - warning_pretimeout)
			* freq;

	pr_debug(PFX "Watchdog ipc_buff[0]%x\n", ipc_wbuf[0]);
	pr_debug(PFX "Watchdog ipc_buff[1]%x\n", ipc_wbuf[1]);
	pr_debug(PFX "Watchdog ipc_buff[2]%x\n", ipc_wbuf[2]);

	ret = rpmsg_send_command(watchdog_instance,
					IPC_SET_WATCHDOG_TIMER,
					IPC_SET_SUB_LOAD_THRES,
					(u8 *) ipc_wbuf, NULL, 12, 0);
	if (ret)
		pr_crit(PFX "Error Setting SCU Watchdog Timer: %x\n", ret);

	return ret;
}

/* Provisioning function for future enhancement : allow to fine tune timing
   according to watchdog action settings */
static int watchdog_set_appropriate_timeouts(void)
{
	pr_debug(PFX "Setting shutdown timeouts\n");
	return watchdog_set_timeouts(timer_timeout, pre_timeout, timeout);
}

/* Keep alive  */
static int watchdog_keepalive(void)
{
	int ret;

	pr_err(PFX "%s\n", __func__);

	if (unlikely(!kicking_active)) {
			/* Close our eyes */
			pr_err(PFX "Transparent kicking\n");
			return 0;
	}
	/* Really kick it */
	ret = rpmsg_send_command(watchdog_instance,
					IPC_SET_WATCHDOG_TIMER,
					IPC_SET_SUB_KEEPALIVE,
					NULL, NULL, 0, 0);
	if (ret)
		pr_err(PFX "Error sending keepalive ipc: %x\n", ret);

	return ret;
}

/* stops the timer */
static int intel_scu_stop(void)
{
	int ret;

	pr_crit(PFX "%s\n", __func__);

	ret = rpmsg_send_command(watchdog_instance,
					IPC_SET_WATCHDOG_TIMER,
					IPC_SET_SUB_DISABLE,
					NULL, NULL, 0, 0);
	if (ret) {
		pr_crit(PFX "Error sending disable ipc: %x\n", ret);
		goto err;
	}

	watchdog_device.started = false;

err:
	return ret;
}

/* tasklet */
static void watchdog_interrupt_tasklet_body(unsigned long data)
{
	int ret;

	pr_warn(PFX "interrupt tasklet body start\n");

	if (disable_kernel_watchdog) {
		/* disable the timer */
		pr_warn(PFX "interrupt tasklet body disable set\n");
		ret = intel_scu_stop();
		if (ret)
			pr_err(PFX "cannot disable the timer\n");
		return;
	}

	/* wake up read to send data to user (reminder for keep alive */
	warning_flag = 1;

#ifdef CONFIG_INTEL_SCU_SOFT_LOCKUP
	{
		int i;
		/*start timer for softlock detection */
		beattime = jiffies;
		for_each_possible_cpu(i) {
			heartbeats[i] = kcpustat_cpu(i).cpustat[CPUTIME_SYSTEM];
		}
		mod_timer(&softlock_timer, jiffies + SOFT_LOCK_TIME * HZ);
	}
#endif

	/* Wake up the daemon */
	wake_up_interruptible(&read_wq);

	/*
	 * Hold a timeout wakelock so user space watchdogd has a chance
	 * to run after waking up from s3
	 */
	wake_lock_timeout(&watchdog_wake_lock, 5 * HZ);
}

/* timer interrupt handler */
static irqreturn_t watchdog_timer_interrupt(int irq, void *dev_id)
{
	if (watchdog_device.started) {
		pr_warn(PFX "Expected SW WDT warning irq received\n");
	} else {
		/* Unexpected, but we'd better to handle it anyway */
		/* and so try to avoid a ColdReset */
		pr_warn(PFX "Unexpected SW WDT warning irq received\n");
	}

	tasklet_schedule(&watchdog_device.interrupt_tasklet);

	return IRQ_HANDLED;
}

/* warning interrupt handler */
static irqreturn_t watchdog_warning_interrupt(int irq, void *dev_id)
{
	pr_warn("[SHTDWN] %s, WATCHDOG TIMEOUT!\n", __func__);

	/* Let's reset the platform after dumping some data */
	trigger_all_cpu_backtrace();

	/* let the watchdog expire to reset the platform */
	set_reboot_force(REBOOT_FORCE_ON);
	panic("Kernel Watchdog");

	/* This code should not be reached */
	return IRQ_HANDLED;
}

/* Program and starts the timer */
static int watchdog_config_and_start(u32 newtimeout, u32 newpretimeout)
{
int ret;

	timeout = newtimeout;
	pre_timeout = newpretimeout;

	pr_warn(PFX "Configuration: %dkHz, timeout=%ds, pre_timeout=%ds, timer=%ds\n",
		watchdog_device.timer7_tbl_ptr->freq_hz / 1000, timeout,
		pre_timeout, timer_timeout);

	/* Configure the watchdog */
	ret = watchdog_set_timeouts(timer_timeout, pre_timeout, timeout);
	if (ret) {
		pr_err(PFX "%s: Cannot configure the watchdog\n", __func__);

		/* Make sure the watchdog timer is stopped */
		intel_scu_stop();
		return ret;
	}

	watchdog_device.started = true;

	return 0;
}

/* Open */
static int intel_scu_open(struct inode *inode, struct file *file)
{
	/* Set flag to indicate that watchdog device is open */
	if (test_and_set_bit(0, &watchdog_device.driver_open)) {
		pr_err(PFX "watchdog device is busy\n");
		return -EBUSY;
	}

	/* Check for reopen of driver. Reopens are not allowed */
	if (watchdog_device.driver_closed) {
		pr_err(PFX "watchdog device has been closed\n");
		return -EPERM;
	}

	return nonseekable_open(inode, file);
}

/* Release */
static int intel_scu_release(struct inode *inode, struct file *file)
{
	/*
	 * This watchdog should not be closed, after the timer
	 * is started with the WDIPC_SETTIMEOUT ioctl
	 * If reset_on_release is set this  will cause an
	 * immediate reset. If reset_on_release is not set, the watchdog
	 * timer is refreshed for one more interval. At the end
	 * of that interval, the watchdog timer will reset the system.
	 */

	if (!test_bit(0, &watchdog_device.driver_open)) {
		pr_err(PFX "intel_scu_release, without open\n");
		return -ENOTTY;
	}

	if (!watchdog_device.started) {
		/* Just close, since timer has not been started */
		pr_err(PFX "Closed, without starting timer\n");
		return 0;
	}

	pr_crit(PFX "Unexpected close of /dev/watchdog!\n");

	/* Since the timer was started, prevent future reopens */
	watchdog_device.driver_closed = 1;

	/* Refresh the timer for one more interval */
	watchdog_keepalive();

	/* Reboot system if requested */
	if (reset_on_release) {
		pr_crit(PFX "Initiating system reboot.\n");
		emergency_restart();
	}

	pr_crit(PFX "Immediate Reboot Disabled\n");
	pr_crit(PFX "System will reset when watchdog timer expire!\n");

	return 0;
}

/* Write */
static ssize_t intel_scu_write(struct file *file, char const *data, size_t len,
			      loff_t *ppos)
{
	pr_debug(PFX "watchdog %s\n", __func__);

	if (watchdog_device.shutdown_flag == true)
		/* do nothing if we are shutting down */
		return len;

	if (watchdog_device.started) {
		/* Watchdog already started, keep it alive */
		watchdog_keepalive();
		wake_unlock(&watchdog_wake_lock);
	} else {
		/* Start watchdog with timer value set by init */
		watchdog_config_and_start(timeout, pre_timeout);
	}

	return len;
}

/* Read */
static ssize_t intel_scu_read(struct file *file, char __user *user_data,
			     size_t len, loff_t *user_ppos)
{
	int ret;
	const u8 *buf = "0";

	/* we wait for the next interrupt; if more than one */
	/* interrupt has occurred since the last read, we */
	/* dont care. The data is not critical. We will do */
	/* a copy to user each time we get and interrupt */
	/* It is up to the Watchdog daemon to be ready to */
	/* do the read (which signifies that the driver is */
	/* awaiting a keep alive and that a limited time */
	/* is available for the keep alive before the system */
	/* is rebooted by the timer */

	warning_flag = 0;

	/* Please note that the content of the data is irrelevent */
	/* All that matters is that the read is available to the user */
	ret = copy_to_user(user_data, (const void *)buf, 1);

	if (ret)
		return -EFAULT;

	return 1;
}

/* Poll */
static unsigned int intel_scu_poll(struct file *file, poll_table *wait)
{
	unsigned int mask = 0;

	poll_wait(file, &read_wq, wait);

	if (warning_flag == 1)
		mask |= POLLIN | POLLRDNORM;

	return mask;
}

/* ioctl */
static long intel_scu_ioctl(struct file *file, unsigned int cmd,
			    unsigned long arg)
{
void __user *argp = (void __user *)arg;
u32 __user *p = argp;
u32 val;
int options;

	static const struct watchdog_info ident = {
		.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING,
		/* @todo Get from SCU via ipc_get_scu_fw_version()? */
		.firmware_version = 0,
		/* len < 32 */
		.identity = "Intel_SCU IOH Watchdog"
	};

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		return copy_to_user(argp, &ident,
				    sizeof(ident)) ? -EFAULT : 0;
	case WDIOC_GETSTATUS:
	case WDIOC_GETBOOTSTATUS:
		return put_user(0, p);
	case WDIOC_KEEPALIVE:
		pr_warn(PFX "%s: KeepAlive ioctl\n", __func__);
		if (!watchdog_device.started)
			return -EINVAL;

		watchdog_keepalive();
		return 0;
	case WDIOC_SETTIMERTIMEOUT:
		pr_warn(PFX "%s: SetTimerTimeout ioctl\n", __func__);

		if (watchdog_device.started)
			return -EBUSY;

		/* Timeout to start scheduling the daemon */
		if (get_user(val, p))
			return -EFAULT;

		timer_timeout = val;
		return 0;
	case WDIOC_SETPRETIMEOUT:
		pr_warn(PFX "%s: SetPreTimeout ioctl\n", __func__);

		if (watchdog_device.started)
			return -EBUSY;

		/* Timeout to warn */
		if (get_user(val, p))
			return -EFAULT;

		pre_timeout = val;
		return 0;
	case WDIOC_SETTIMEOUT:
		pr_warn(PFX "%s: SetTimeout ioctl\n", __func__);

		if (watchdog_device.started)
			return -EBUSY;

		if (get_user(val, p))
			return -EFAULT;

		timeout = val;
		return 0;
	case WDIOC_GETTIMEOUT:
		return put_user(timeout, p);
	case WDIOC_SETOPTIONS:
		if (get_user(options, p))
			return -EFAULT;

		if (options & WDIOS_DISABLECARD) {
			pr_warn(PFX "%s: Stopping the watchdog\n", __func__);
			intel_scu_stop();
			return 0;
		}

		if (options & WDIOS_ENABLECARD) {
			pr_warn(PFX "%s: Starting the watchdog\n", __func__);

			if (watchdog_device.started)
				return -EBUSY;

			if (check_timeouts(timer_timeout,
					   pre_timeout, timeout)) {
				pr_warn(PFX "%s: Invalid thresholds\n",
					__func__);
				return -EINVAL;
			}
			if (watchdog_config_and_start(timeout, pre_timeout))
				return -EINVAL;
			return 0;
		}
		return 0;
	default:
		return -ENOTTY;
	}
}

static int watchdog_set_reset_type(int reset_type)
{
	int ret;

	ret = rpmsg_send_command(watchdog_instance,
				  IPC_SET_WATCHDOG_TIMER,
				  reset_type,
				  NULL, NULL, 0, 0);

	if (ret) {
		pr_crit(PFX "Error setting watchdog action: %d\n", ret);
		return -EIO;
	}

	watchdog_device.normal_wd_action = reset_type;

	return 0;
}

/* Reboot notifier */
static int reboot_notifier(struct notifier_block *this,
			   unsigned long code,
			   void *another_unused)
{
	int ret;

	if (code == SYS_RESTART || code == SYS_HALT || code == SYS_POWER_OFF) {
		pr_warn(PFX "Reboot notifier\n");

		if (watchdog_set_appropriate_timeouts())
			pr_crit(PFX "reboot notifier cant set time\n");

		switch (code) {
		case SYS_RESTART:
			watchdog_device.reboot_flag = true;
			ret = watchdog_set_reset_type(
				watchdog_device.reboot_wd_action);
			break;

		case SYS_HALT:
		case SYS_POWER_OFF:
			ret = watchdog_set_reset_type(
				watchdog_device.shutdown_wd_action);
			break;
		}
		if (ret)
			pr_err(PFX "%s: could not set reset type\n", __func__);

#ifdef CONFIG_DEBUG_FS
		/* debugfs entry to generate a BUG during
		any shutdown/reboot call */
		if (watchdog_device.panic_reboot_notifier) {
			/* let the watchdog expire to reset the platform */
			set_reboot_force(REBOOT_FORCE_ON);
			BUG();
		}
#endif
		/* Don't do instant reset on close */
		reset_on_release = false;

		/* Kick once again */
		if (disable_kernel_watchdog == false) {
			ret = watchdog_keepalive();
			if (ret)
				pr_warn(PFX "%s: no keep alive\n", __func__);

			/* Don't allow any more keep-alives */
			watchdog_device.shutdown_flag = true;
		}
	}
	return NOTIFY_DONE;
}

#ifdef CONFIG_DEBUG_FS
/* This code triggers a Security Watchdog */
int write_security(struct inode *i, struct file *f)
{
	int ret = 0;
	u64 *ptr;
	u32 value;

	ptr = ioremap_nocache(SECURITY_WATCHDOG_ADDR, sizeof(u32));

	if (!ptr) {
		pr_err(PFX "cannot open secwd's debugfile\n");
		ret = -ENODEV;
		goto error;
	}
	value = readl(ptr); /* trigger */

	pr_err(PFX "%s: This code should never be reached but it got %x\n",
		__func__, (unsigned int)value);

error:
	return ret;
}

static const struct file_operations security_watchdog_fops = {
	.open = nonseekable_open,
	.write = write_security,
	.llseek = no_llseek,
};

static int kwd_trigger_write(struct file *file, const char __user *buff,
			     size_t count, loff_t *ppos)
{
	pr_debug("kwd_trigger_write\n");

	/* let the watchdog expire to reset the platform */
	set_reboot_force(REBOOT_FORCE_ON);
	BUG();
	return 0;
}

static const struct file_operations kwd_trigger_fops = {
	.open = nonseekable_open,
	.write = kwd_trigger_write,
	.llseek = no_llseek,
};

static int kwd_reset_type_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t kwd_reset_type_read(struct file *file, char __user *buff,
				size_t count, loff_t *ppos)
{
	ssize_t len;
	int ret;
	char str[STRING_RESET_TYPE_MAX_LEN + 1];

	pr_debug(PFX "reading reset_type of %x\n",
		 watchdog_device.normal_wd_action);

	if (*ppos > 0)
		return 0;

	ret = reset_type_to_string(watchdog_device.normal_wd_action, str);
	if (ret)
		return -EINVAL;
	else {
		for (len = 0; len < (STRING_RESET_TYPE_MAX_LEN - 1)
			     && str[len] != '\0'; len++)
			;
		str[len++] = '\n';
		ret = copy_to_user(buff, str, len);
	}

	*ppos += len;
	return len;
}

static ssize_t kwd_reset_type_write(struct file *file, const char __user *buff,
		size_t count, loff_t *ppos)
{
	char str[STRING_RESET_TYPE_MAX_LEN];
	unsigned long res;
	int ret, reset_type;

	if (count > STRING_RESET_TYPE_MAX_LEN) {
		pr_err(PFX "Invalid size: count=%d\n", count);
		return -EINVAL;
	}

	memset(str, 0x00, STRING_RESET_TYPE_MAX_LEN);

	res = copy_from_user((void *)str,
		(void __user *)buff,
		(unsigned long)min((unsigned long)(count-1),
		(unsigned long)(STRING_RESET_TYPE_MAX_LEN-1)));

	if (res) {
		pr_err(PFX "%s: copy to user failed\n", __func__);
		return -EINVAL;
	}

	pr_debug(PFX "writing reset_type of %s\n", str);

	ret = string_to_reset_type(str, &reset_type);
	if (ret) {
		pr_err(PFX "Invalid value\n");
		return -EINVAL;
	}

	ret = watchdog_set_reset_type(reset_type);
	if (ret) {
		pr_err(PFX "%s: could not set reset type\n", __func__);
		return -EINVAL;
	}

	return count;
}

static const struct file_operations kwd_reset_type_fops = {
	.open		= nonseekable_open,
	.release	= kwd_reset_type_release,
	.read		= kwd_reset_type_read,
	.write		= kwd_reset_type_write,
	.llseek		= no_llseek,
};

static ssize_t kwd_panic_reboot_read(struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	# define RET_SIZE 3 /* prints only 2 chars : '0' or '1', plus '\n' */
	char str[RET_SIZE];

	int res;

	if (*ppos > 0)
		return 0;

	strcpy(str, watchdog_device.panic_reboot_notifier ? "1\n" : "0\n");

	res = copy_to_user(buff, str, RET_SIZE);
	if (res) {
		pr_err(PFX "%s: copy to user failed\n", __func__);
		return -EINVAL;
	}

	*ppos += RET_SIZE-1;
	return RET_SIZE-1;
}


static ssize_t kwd_panic_reboot_write(struct file *file,
		const char __user *buff, size_t count, loff_t *ppos)
{
	/* whatever is written, simply set flag to TRUE */
	watchdog_device.panic_reboot_notifier = true;

	return count;
}


static const struct file_operations kwd_panic_reboot_fops = {
	.open		= nonseekable_open,
	.read		= kwd_panic_reboot_read,
	.write		= kwd_panic_reboot_write,
	.llseek		= no_llseek,
};

static int remove_debugfs_entries(void)
{
struct intel_scu_watchdog_dev *dev = &watchdog_device;

	/* /sys/kernel/debug/watchdog */
	debugfs_remove_recursive(dev->dfs_wd);

	return 0;
}

static int create_debugfs_entries(void)
{
	struct intel_scu_watchdog_dev *dev = &watchdog_device;

	/* /sys/kernel/debug/watchdog */
	dev->dfs_wd = debugfs_create_dir("watchdog", NULL);
	if (!dev->dfs_wd) {
		pr_err(PFX "%s: Error, cannot create main dir\n", __func__);
		goto error;
	}

	/* /sys/kernel/debug/watchdog/sc_watchdog */
	dev->dfs_secwd = debugfs_create_dir("sc_watchdog", dev->dfs_wd);
	if (!dev->dfs_secwd) {
		pr_err(PFX "%s: Error, cannot create sec dir\n", __func__);
		goto error;
	}

	/* /sys/kernel/debug/watchdog/security_watchdog/trigger */
	dev->dfs_secwd_trigger = debugfs_create_file("trigger",
				    S_IFREG | S_IWUSR | S_IWGRP,
				    dev->dfs_secwd, NULL,
				    &security_watchdog_fops);

	if (!dev->dfs_secwd_trigger) {
		pr_err(PFX "%s: Error, cannot create sec file\n", __func__);
		goto error;
	}

	/* /sys/kernel/debug/watchdog/kernel_watchdog */
	dev->dfs_kwd = debugfs_create_dir("kernel_watchdog", dev->dfs_wd);
	if (!dev->dfs_kwd) {
		pr_err(PFX "%s: Error, cannot create kwd dir\n", __func__);
		goto error;
	}

	/* /sys/kernel/debug/watchdog/kernel_watchdog/trigger */
	dev->dfs_kwd_trigger = debugfs_create_file("trigger",
				    S_IFREG | S_IWUSR | S_IWGRP,
				    dev->dfs_kwd, NULL,
				    &kwd_trigger_fops);

	if (!dev->dfs_kwd_trigger) {
		pr_err(PFX "%s: Error, cannot create kwd trigger file\n",
			__func__);
		goto error;
	}

	/* /sys/kernel/debug/watchdog/kernel_watchdog/reset_type */
	dev->dfs_kwd_reset_type = debugfs_create_file("reset_type",
				    S_IFREG | S_IRUGO | S_IWUSR | S_IWGRP,
				    dev->dfs_kwd, NULL,
				    &kwd_reset_type_fops);

	if (!dev->dfs_kwd_trigger) {
		pr_err(PFX "%s: Error, cannot create kwd trigger file\n",
			__func__);
		goto error;
	}

	/* /sys/kernel/debug/watchdog/kernel_watchdog/panic_reboot_notifier */
	dev->dfs_kwd_panic_reboot = debugfs_create_file("panic_reboot_notifier",
					S_IFREG | S_IRUGO | S_IWUSR | S_IWGRP,
					dev->dfs_kwd, NULL,
					&kwd_panic_reboot_fops);

	if (!dev->dfs_kwd_panic_reboot) {
		pr_err(PFX "%s: Error, cannot create kwd panic_reboot_notifier file\n",
			__func__);
		goto error;
	}


	return 0;
error:
	remove_debugfs_entries();
	return 1;
}
#endif  /* CONFIG_DEBUG_FS*/

/* Kernel Interfaces */
static const struct file_operations intel_scu_fops = {
	.owner          = THIS_MODULE,
	.llseek         = no_llseek,
	.write          = intel_scu_write,
	.read		= intel_scu_read,
	.unlocked_ioctl = intel_scu_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= intel_scu_ioctl,
#endif
	.open           = intel_scu_open,
	.poll		= intel_scu_poll,
	.release        = intel_scu_release,
};

/* sysfs entry to disable watchdog */
#ifdef CONFIG_DISABLE_SCU_WATCHDOG
static ssize_t disable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;

	if (!strtobool(buf, &disable_kernel_watchdog)) {
		if (disable_kernel_watchdog) {
			ret = intel_scu_stop();
			if (ret)
				pr_err(PFX "cannot disable the timer\n");
		} else {
			ret = watchdog_config_and_start(timeout, pre_timeout);
			if (ret)
				return -EINVAL;
		}
	} else {
		pr_err(PFX "got invalid value\n");
		return -EINVAL;
	}

	return size;
}

static ssize_t disable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	pr_debug(PFX "%s\n", __func__);
	if (disable_kernel_watchdog)
		return sprintf(buf, "1\n");

	return sprintf(buf, "0\n");
}

static DEVICE_ATTR(disable, S_IWUSR | S_IRUGO,
	disable_show, disable_store);

#endif

#define OSNIB_WDOG_COUNTER_MASK 0xF0
#define OSNIB_WDOG_COUNTER_SHIFT 4
#define WDOG_COUNTER_MAX_VALUE   3
static ssize_t counter_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;
	pr_debug(PFX "%s\n", __func__);

	ret = sscanf(buf, "%hhu", &osnib_reset);
	if (ret != 1) {
		pr_err(PFX "cannot get counter value\n");
		if (ret == 0)
			ret = -EINVAL;
		return ret;
	}
	if (osnib_reset > WDOG_COUNTER_MAX_VALUE)
		osnib_reset = WDOG_COUNTER_MAX_VALUE;
	osnib_reset = ((osnib_reset << OSNIB_WDOG_COUNTER_SHIFT) &
				OSNIB_WDOG_COUNTER_MASK);
	ret = intel_scu_ipc_write_osnib_wd(&osnib_reset);
	if (ret != 0) {
		pr_err(PFX "cannot write OSNIB\n");
		return -EINVAL;
	}

	return size;
}


static ssize_t counter_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	unsigned char osnib_read = (unsigned char)0;
	int ret;
	pr_debug(PFX "%s\n", __func__);

	ret = intel_scu_ipc_read_osnib_wd(&osnib_read);
	if (ret != 0)
		return -EIO;

	return sprintf(buf, "%d\n", (int)((osnib_read & OSNIB_WDOG_COUNTER_MASK)
						>> OSNIB_WDOG_COUNTER_SHIFT));
}

static int reset_type_to_string(int reset_type, char *string)
{
	switch (reset_type) {
	case IPC_SET_SUB_COLDBOOT:
		strcpy(string, STRING_COLD_BOOT);
		break;
	case IPC_SET_SUB_COLDRESET:
		strcpy(string, STRING_COLD_RESET);
		break;
	case IPC_SET_SUB_COLDOFF:
		strcpy(string, STRING_COLD_OFF);
		break;
#ifdef CONFIG_DEBUG_FS
	case IPC_SET_SUB_DONOTHING:
		/* The IPC command DONOTHING is provided */
		/* for debug purpose only.               */
		strcpy(string, STRING_NONE);
		break;
#endif
	default:
		return 1;
	}

	return 0;
}

static int string_to_reset_type(const char *string, int *reset_type)
{
	if (!reset_type || !string)
		return 1;

	if (strncmp(string, STRING_COLD_RESET,
			sizeof(STRING_COLD_RESET) - 1) == 0) {
		*reset_type = IPC_SET_SUB_COLDRESET;
		return 0;
	}
	if (strncmp(string, STRING_COLD_BOOT,
			sizeof(STRING_COLD_BOOT) - 1) == 0) {
		*reset_type = IPC_SET_SUB_COLDBOOT;
		return 0;
	}
	if (strncmp(string, STRING_COLD_OFF,
			sizeof(STRING_COLD_OFF) - 1) == 0) {
		*reset_type = IPC_SET_SUB_COLDOFF;
		return 0;
	}

	/* We should not be here, this is an error case */
	pr_debug("Invalid reset type value\n");
	return 1;
}

static ssize_t reboot_ongoing_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;

	pr_debug(PFX "%s\n", __func__);
	/* reprogram timeouts. if error : continue */
	ret = watchdog_set_appropriate_timeouts();
	if (ret)
		pr_err(PFX "%s: could not set timeouts\n", __func__);

	/* restore reset type */
	watchdog_set_reset_type(watchdog_device.reboot_wd_action);
	if (ret) {
		pr_err(PFX "%s: could not set reset type\n", __func__);
		return -EINVAL;
	}

	return size;
}

static ssize_t shutdown_ongoing_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;

	pr_debug(PFX "%s\n", __func__);
	/* reprogram timeouts. if error : continue */
	ret = watchdog_set_appropriate_timeouts();
	if (ret)
		pr_err(PFX "%s: could not set timeouts\n", __func__);

	/* restore reset type */
	watchdog_set_reset_type(watchdog_device.shutdown_wd_action);
	if (ret) {
		pr_err(PFX "%s: could not set reset type\n", __func__);
		return -EINVAL;
	}

	return size;
}

static ssize_t normal_config_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	if (reset_type_to_string(watchdog_device.normal_wd_action, buf) != 0)
		return -EINVAL;
	strcat(buf, "\n");
	return strlen(buf);
}

static ssize_t normal_config_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	if (string_to_reset_type(buf, &watchdog_device.normal_wd_action) != 0)
		return -EINVAL;
	if (watchdog_set_reset_type(watchdog_device.normal_wd_action) != 0)
		return -EINVAL;

	return size;
}

static ssize_t reboot_config_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	if (reset_type_to_string(watchdog_device.reboot_wd_action, buf) != 0)
		return -EINVAL;
	strcat(buf, "\n");
	return strlen(buf);
}

static ssize_t reboot_config_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	if (string_to_reset_type(buf, &watchdog_device.reboot_wd_action) != 0)
		return -EINVAL;

	return size;
}

static ssize_t shutdown_config_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	if (reset_type_to_string(watchdog_device.shutdown_wd_action, buf) != 0)
		return -EINVAL;
	strcat(buf, "\n");
	return strlen(buf);
}

static ssize_t shutdown_config_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	if (string_to_reset_type(buf, &watchdog_device.shutdown_wd_action) != 0)
		return -EINVAL;

	return size;
}

/* Watchdog behavior depending on system phase */
static DEVICE_ATTR(normal_config, S_IWUSR | S_IRUGO,
	normal_config_show, normal_config_store);
static DEVICE_ATTR(reboot_config, S_IWUSR | S_IRUGO,
	reboot_config_show, reboot_config_store);
static DEVICE_ATTR(shutdown_config, S_IWUSR | S_IRUGO,
	shutdown_config_show, shutdown_config_store);
static DEVICE_ATTR(reboot_ongoing, S_IWUSR,
	NULL, reboot_ongoing_store);
static DEVICE_ATTR(shutdown_ongoing, S_IWUSR,
	NULL, shutdown_ongoing_store);

/* Reset counter watchdog entry */
static DEVICE_ATTR(counter, S_IWUSR | S_IRUGO,
	counter_show, counter_store);


int create_watchdog_sysfs_files(void)
{
	int ret;

#ifdef CONFIG_DISABLE_SCU_WATCHDOG
	ret = device_create_file(watchdog_device.miscdev.this_device,
		&dev_attr_disable);
	if (ret) {
		pr_warn("cant register dev file for disable\n");
		return ret;
	}
#endif

	ret = device_create_file(watchdog_device.miscdev.this_device,
		&dev_attr_normal_config);
	if (ret) {
		pr_warn("cant register dev file for normal_config\n");
		return ret;
	}

	ret = device_create_file(watchdog_device.miscdev.this_device,
		&dev_attr_reboot_config);
	if (ret) {
		pr_warn("cant register dev file for reboot_config\n");
		return ret;
	}

	ret = device_create_file(watchdog_device.miscdev.this_device,
		&dev_attr_shutdown_config);
	if (ret) {
		pr_warn("cant register dev file for shutdown_config\n");
		return ret;
	}

	ret = device_create_file(watchdog_device.miscdev.this_device,
		&dev_attr_counter);
	if (ret) {
		pr_warn("cant register dev file for counter\n");
		return ret;
	}

	ret = device_create_file(watchdog_device.miscdev.this_device,
		&dev_attr_reboot_ongoing);
	if (ret) {
		pr_warn("cant register dev file for reboot_ongoing\n");
		return ret;
	}

	ret = device_create_file(watchdog_device.miscdev.this_device,
		&dev_attr_shutdown_ongoing);
	if (ret) {
		pr_warn("cant register dev file for shutdown_ongoing\n");
		return ret;
	}
	return 0;
}

int remove_watchdog_sysfs_files(void)
{
#ifdef CONFIG_DISABLE_SCU_WATCHDOG
	device_remove_file(watchdog_device.miscdev.this_device,
		&dev_attr_disable);
#endif
	device_remove_file(watchdog_device.miscdev.this_device,
		&dev_attr_normal_config);

	device_remove_file(watchdog_device.miscdev.this_device,
		&dev_attr_reboot_config);

	device_remove_file(watchdog_device.miscdev.this_device,
		&dev_attr_shutdown_config);

	device_remove_file(watchdog_device.miscdev.this_device,
		&dev_attr_counter);

	device_remove_file(watchdog_device.miscdev.this_device,
		&dev_attr_reboot_ongoing);

	device_remove_file(watchdog_device.miscdev.this_device,
		&dev_attr_shutdown_ongoing);
	return 0;
}

void decode_reboot_force(int watchdog_action)
{
	switch (watchdog_action) {
	case IPC_SET_SUB_COLDRESET:
		set_reboot_force(REBOOT_FORCE_COLD_RESET);
		break;
	case IPC_SET_SUB_COLDBOOT:
		set_reboot_force(REBOOT_FORCE_COLD_BOOT);
		break;
	case IPC_SET_SUB_COLDOFF:
		set_reboot_force(REBOOT_FORCE_OFF);
		break;
	case IPC_SET_SUB_DONOTHING:
		set_reboot_force(REBOOT_FORCE_ON);
		break;
	default:
		set_reboot_force(REBOOT_FORCE_COLD_RESET);
	}
}

/* This is the callback function launched when kernel panic() function */
/* is executed.                                                        */
static int watchdog_panic_handler(struct notifier_block *this,
				  unsigned long         event,
				  void                  *unused)
{
	/* don't allow kicking anymore */
	kicking_active = false;

	if (disable_kernel_watchdog == true) {
		set_reboot_force(REBOOT_FORCE_ON);
		return NOTIFY_OK;
	}

	if (get_reboot_force() == REBOOT_FORCE_ON)
		return NOTIFY_OK;

	if (watchdog_device.reboot_flag == true)
		decode_reboot_force(watchdog_device.reboot_wd_action);
	else if (watchdog_device.shutdown_flag == true)
		decode_reboot_force(watchdog_device.shutdown_wd_action);
	else
		decode_reboot_force(watchdog_device.normal_wd_action);

	return NOTIFY_OK;
}

static struct notifier_block watchdog_panic_notifier = {
	.notifier_call	= watchdog_panic_handler,
	.next		= NULL,
	.priority	= 150	/* priority: INT_MAX >= x >= 0 */
};

/* Init code */
static int intel_scu_watchdog_init(void)
{
	int ret = 0;

	watchdog_device.normal_wd_action = IPC_SET_SUB_COLDRESET;
	watchdog_device.reboot_wd_action = IPC_SET_SUB_COLDRESET;
	watchdog_device.shutdown_wd_action = IPC_SET_SUB_COLDOFF;

#ifdef CONFIG_DEBUG_FS
	watchdog_device.panic_reboot_notifier = false;
#endif /* CONFIG_DEBUG_FS */

	/* Initially, we are not in shutdown mode */
	watchdog_device.shutdown_flag = false;
	watchdog_device.reboot_flag = false;

	/* Check timeouts boot parameter */
	if (check_timeouts(timer_timeout, pre_timeout, timeout)) {
		pr_err(PFX "%s: Invalid timeouts\n", __func__);
		return -EINVAL;
	}

	/* Acquire timer 7 */
	watchdog_device.timer7_tbl_ptr = sfi_get_mtmr(sfi_mtimer_num-1);
	if (watchdog_device.timer7_tbl_ptr == NULL) {
		pr_debug(PFX "Watchdog timer - Intel SCU watchdog: Timer is"
			" not available\n");
		return -ENODEV;
	}
	if (watchdog_device.timer7_tbl_ptr->phys_addr == 0) {
		pr_debug(PFX "Watchdog timer - Intel SCU watchdog - "
			"timer %d does not have valid physical memory\n",
			sfi_mtimer_num);
		return -ENODEV;
	}
	if (watchdog_device.timer7_tbl_ptr->irq == 0) {
		pr_debug(PFX "Watchdog timer: timer %d invalid irq\n",
		  sfi_mtimer_num);
		return -ENODEV;
	}

	/* Acquire timer 6 */
	watchdog_device.timer6_tbl_ptr = sfi_get_mtmr(sfi_mtimer_num-2);
	if (watchdog_device.timer6_tbl_ptr == NULL) {
		pr_debug(PFX "Watchdog timer - Intel SCU watchdog: Timer is"
			" not available\n");
		return -ENODEV;
	}
	if (watchdog_device.timer6_tbl_ptr->irq == 0) {
		pr_debug(PFX "Watchdog timer: timer %d invalid irq\n",
		  sfi_mtimer_num);
		return -ENODEV;
	}

	/* Reboot notifier */
	watchdog_device.reboot_notifier.notifier_call = reboot_notifier;
	watchdog_device.reboot_notifier.priority = 1;
	ret = register_reboot_notifier(&watchdog_device.reboot_notifier);
	if (ret) {
		pr_crit(PFX "cannot register reboot notifier %d\n", ret);
		goto error_stop_timer;
	}

	ret = atomic_notifier_chain_register(&panic_notifier_list,
			&watchdog_panic_notifier);
	if (ret) {
		pr_crit("cannot register panic notifier %d\n", ret);
		goto error_reboot_notifier;
	}

	/* Do not publish the watchdog device when disable (TO BE REMOVED) */
	if (!disable_kernel_watchdog) {
		watchdog_device.miscdev.minor = WATCHDOG_MINOR;
		watchdog_device.miscdev.name = "watchdog";
		watchdog_device.miscdev.fops = &intel_scu_fops;

		ret = misc_register(&watchdog_device.miscdev);
		if (ret) {
			pr_crit(PFX "Cannot register miscdev %d err =%d\n",
				WATCHDOG_MINOR, ret);
			goto error_panic_notifier;
		}
	}

	wake_lock_init(&watchdog_wake_lock, WAKE_LOCK_SUSPEND,
			"intel_scu_watchdog");

	/* MSI #7 handler for timer interrupts */
	ret = request_irq((unsigned int)watchdog_device.timer7_tbl_ptr->irq,
		watchdog_timer_interrupt,
		IRQF_SHARED|IRQF_NO_SUSPEND, "watchdog timer",
		&watchdog_device);
	if (ret) {
		pr_err(PFX "error requesting irq %d\n",
		       watchdog_device.timer7_tbl_ptr->irq);
		pr_err(PFX "error value returned is %d\n", ret);
		goto error_misc_register;
	}

	/* MSI #6 handler to dump registers */
	ret = request_irq((unsigned int)watchdog_device.timer6_tbl_ptr->irq,
		watchdog_warning_interrupt,
		IRQF_SHARED|IRQF_NO_SUSPEND, "watchdog",
		&watchdog_device);
	if (ret) {
		pr_err(PFX "error requesting warning irq %d\n",
		       watchdog_device.timer6_tbl_ptr->irq);
		pr_err(PFX "error value returned is %d\n", ret);
		goto error_request_irq;
	}

	/* set up the tasklet for handling interrupt duties */
	tasklet_init(&watchdog_device.interrupt_tasklet,
		watchdog_interrupt_tasklet_body, (unsigned long)0);

#ifdef CONFIG_INTEL_SCU_SOFT_LOCKUP
	init_timer(&softlock_timer);
#endif

	if (disable_kernel_watchdog) {
		pr_err(PFX "%s: Disable kernel watchdog\n", __func__);

		/* Make sure timer is stopped */
		ret = intel_scu_stop();
		if (ret != 0)
			pr_debug(PFX "cant disable timer\n");
	}

#ifdef CONFIG_DEBUG_FS
	ret = create_debugfs_entries();
	if (ret) {
		pr_err(PFX "%s: Error creating debugfs entries\n", __func__);
		goto error_debugfs_entry;
	}
#endif

	watchdog_device.started = false;

	ret = create_watchdog_sysfs_files();
	if (ret) {
		pr_err(PFX "%s: Error creating debugfs entries\n", __func__);
		goto error_sysfs_entry;
	}

	return ret;

error_sysfs_entry:
	/* Nothing special to do */
#ifdef CONFIG_DEBUG_FS
error_debugfs_entry:
	/* Remove entries done by create function */
#endif
error_request_irq:
	free_irq(watchdog_device.timer7_tbl_ptr->irq, NULL);

error_misc_register:
	misc_deregister(&watchdog_device.miscdev);

error_panic_notifier:
	atomic_notifier_chain_unregister(&panic_notifier_list,
						 &watchdog_panic_notifier);

error_reboot_notifier:
	unregister_reboot_notifier(&watchdog_device.reboot_notifier);

error_stop_timer:
	intel_scu_stop();

	return ret;
}

static void intel_scu_watchdog_exit(void)
{
	int ret = 0;

	remove_watchdog_sysfs_files();
#ifdef CONFIG_DEBUG_FS
	remove_debugfs_entries();
#endif

#ifdef CONFIG_INTEL_SCU_SOFT_LOCKUP
	del_timer_sync(&softlock_timer);
#endif

	ret = intel_scu_stop();
	if (ret != 0)
		pr_err(PFX "cant disable timer\n");

	misc_deregister(&watchdog_device.miscdev);
	unregister_reboot_notifier(&watchdog_device.reboot_notifier);
	atomic_notifier_chain_unregister(&panic_notifier_list,
						 &watchdog_panic_notifier);
}

static int watchdog_rpmsg_probe(struct rpmsg_channel *rpdev)
{
	int ret = 0;

	if (rpdev == NULL) {
		pr_err("rpmsg channel not created\n");
		ret = -ENODEV;
		goto out;
	}

	dev_info(&rpdev->dev, "Probed watchdog rpmsg device\n");

	/* Allocate rpmsg instance for watchdog*/
	ret = alloc_rpmsg_instance(rpdev, &watchdog_instance);
	if (!watchdog_instance) {
		dev_err(&rpdev->dev, "kzalloc watchdog instance failed\n");
		goto out;
	}
	/* Initialize rpmsg instance */
	init_rpmsg_instance(watchdog_instance);
	/* Init scu watchdog */
	ret = intel_scu_watchdog_init();

	if (ret)
		free_rpmsg_instance(rpdev, &watchdog_instance);
out:
	return ret;
}

static void watchdog_rpmsg_remove(struct rpmsg_channel *rpdev)
{
	intel_scu_watchdog_exit();
	free_rpmsg_instance(rpdev, &watchdog_instance);
	dev_info(&rpdev->dev, "Removed watchdog rpmsg device\n");
}

static void watchdog_rpmsg_cb(struct rpmsg_channel *rpdev, void *data,
					int len, void *priv, u32 src)
{
	dev_warn(&rpdev->dev, "unexpected, message\n");

	print_hex_dump(KERN_DEBUG, __func__, DUMP_PREFIX_NONE, 16, 1,
		       data, len,  true);
}

static struct rpmsg_device_id watchdog_rpmsg_id_table[] = {
	{ .name	= "rpmsg_watchdog" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, watchdog_rpmsg_id_table);

static struct rpmsg_driver watchdog_rpmsg = {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= watchdog_rpmsg_id_table,
	.probe		= watchdog_rpmsg_probe,
	.callback	= watchdog_rpmsg_cb,
	.remove		= watchdog_rpmsg_remove,
};

static int __init watchdog_rpmsg_init(void)
{
	if (intel_mid_identify_cpu() == INTEL_MID_CPU_CHIP_PENWELL ||
	    intel_mid_identify_cpu() == INTEL_MID_CPU_CHIP_CLOVERVIEW)
		return register_rpmsg_driver(&watchdog_rpmsg);
	else {
		pr_err(PFX "%s: watchdog driver: bad platform\n", __func__);
		return -ENODEV;
	}
}

#ifdef MODULE
module_init(watchdog_rpmsg_init);
#else
rootfs_initcall(watchdog_rpmsg_init);
#endif

static void __exit watchdog_rpmsg_exit(void)
{
	return unregister_rpmsg_driver(&watchdog_rpmsg);
}
module_exit(watchdog_rpmsg_exit);

MODULE_AUTHOR("Intel Corporation");
MODULE_AUTHOR("mark.a.allyn@intel.com");
MODULE_AUTHOR("yannx.puech@intel.com");
MODULE_DESCRIPTION("Intel SCU Watchdog Device Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
MODULE_VERSION(WDT_VER);
