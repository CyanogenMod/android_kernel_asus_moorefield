/*
 *	intel TCO Watchdog Driver
 *
 *	(c) Copyright 2006-2011 Wim Van Sebroeck <wim@iguana.be>.
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 *
 *	Neither Wim Van Sebroeck nor Iguana vzw. admit liability nor
 *	provide warranty for any of this software. This material is
 *	provided "AS-IS" and at no charge.
 *
 *	The TCO watchdog is implemented in the following I/O controller hubs:
 *	(See the intel documentation on http://developer.intel.com.)
 *	document number 290655-003, 290677-014: 82801AA (ICH), 82801AB (ICHO)
 *	document number 290687-002, 298242-027: 82801BA (ICH2)
 *	document number 290733-003, 290739-013: 82801CA (ICH3-S)
 *	document number 290716-001, 290718-007: 82801CAM (ICH3-M)
 *	document number 290744-001, 290745-025: 82801DB (ICH4)
 *	document number 252337-001, 252663-008: 82801DBM (ICH4-M)
 *	document number 273599-001, 273645-002: 82801E (C-ICH)
 *	document number 252516-001, 252517-028: 82801EB (ICH5), 82801ER (ICH5R)
 *	document number 300641-004, 300884-013: 6300ESB
 *	document number 301473-002, 301474-026: 82801F (ICH6)
 *	document number 313082-001, 313075-006: 631xESB, 632xESB
 *	document number 307013-003, 307014-024: 82801G (ICH7)
 *	document number 322896-001, 322897-001: NM10
 *	document number 313056-003, 313057-017: 82801H (ICH8)
 *	document number 316972-004, 316973-012: 82801I (ICH9)
 *	document number 319973-002, 319974-002: 82801J (ICH10)
 *	document number 322169-001, 322170-003: 5 Series, 3400 Series (PCH)
 *	document number 320066-003, 320257-008: EP80597 (IICH)
 *	document number 324645-001, 324646-001: Cougar Point (CPT)
 *	document number TBD                   : Patsburg (PBG)
 *	document number TBD                   : DH89xxCC
 *	document number TBD                   : Panther Point
 *	document number TBD                   : Lynx Point
 */

/*
 *	Includes, defines, variables, module parameters, ...
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

/* Module and version information */
#define DRV_NAME	"iTCO_wdt"
#define DRV_VERSION	"1.07"

/* Includes */
#include <linux/module.h>		/* For module specific items */
#include <linux/moduleparam.h>		/* For new moduleparam's */
#include <linux/types.h>		/* For standard types (like size_t) */
#include <linux/errno.h>		/* For the -ENODEV/... values */
#include <linux/kernel.h>		/* For printk/panic/... */
#include <linux/miscdevice.h>		/* For MODULE_ALIAS_MISCDEV
							(WATCHDOG_MINOR) */
#include <linux/watchdog.h>		/* For the watchdog specific items */
#include <linux/init.h>			/* For __init/__exit/... */
#include <linux/fs.h>			/* For file operations */
#include <linux/platform_device.h>	/* For platform_driver framework */
#include <linux/pci.h>			/* For pci functions */
#include <linux/ioport.h>		/* For io-port access */
#include <linux/spinlock.h>		/* For spin_lock/spin_unlock/... */
#include <linux/uaccess.h>		/* For copy_to_user/put_user/... */
#include <linux/io.h>			/* For inb/outb/... */
#include <linux/debugfs.h>
#include <linux/reboot.h>
#include <linux/interrupt.h>
#include <linux/acpi.h>
#include <linux/nmi.h>

/* Address definitions for the TCO */
/* TCO base address */
#define TCOBASE		(iTCO_wdt_private.ACPIBASE + 0x60)
/* SMI Control and Enable Register */
#define SMI_EN		(iTCO_wdt_private.ACPIBASE + 0x30)
#define SMI_STS		(iTCO_wdt_private.ACPIBASE + 0x34)

#define TCO_RLD		(TCOBASE + 0x00) /* TCO Timer Reload and Curr. Value */
#define TCOv1_TMR	(TCOBASE + 0x01) /* TCOv1 Timer Initial Value	*/
#define TCO_DAT_IN	(TCOBASE + 0x02) /* TCO Data In Register	*/
#define TCO_DAT_OUT	(TCOBASE + 0x03) /* TCO Data Out Register	*/
#define TCO1_STS	(TCOBASE + 0x04) /* TCO1 Status Register	*/
#define TCO2_STS	(TCOBASE + 0x06) /* TCO2 Status Register	*/
#define TCO1_CNT	(TCOBASE + 0x08) /* TCO1 Control Register	*/
#define TCO2_CNT	(TCOBASE + 0x0a) /* TCO2 Control Register	*/
#define TCOv2_TMR	(TCOBASE + 0x12) /* TCOv2 Timer Initial Value	*/

#define TCO_POLICY_OFFSET	20
#define TCO_POLICY_MASK		0x3
#define TCO_POLICY_NORM		0x0
#define TCO_POLICY_HALT		0x1
#define TCO_POLICY_NO_LOAD	0x2

#define TCO_TIMEOUT_BIT		(1 << 3)
#define SECOND_TO_STS_BIT	(1 << 17)
#define TCO_STS_BIT		(1 << 13)
#define EOS_BIT			(1 << 1)
#define TCO_EN_BIT		(1 << 13)

#define STRING_RESET_TYPE_MAX_LEN 12
#define STRING_COLD_OFF "COLD_OFF"
#define STRING_COLD_RESET "COLD_RESET"

static u32 pmc_base_address;
#define PMC_CFG		(pmc_base_address + 0x8)

static struct dentry *iTCO_debugfs_dir;

/* internal variables */
static unsigned long is_active;
static char expect_release;
static struct {		/* this is private data for the iTCO_wdt device */
	/* TCO watchdog action */
	unsigned int iTCO_wdt_action;
	/* The device's ACPIBASE address (TCOBASE = ACPIBASE+0x60) */
	unsigned long ACPIBASE;
	/* the lock for io operations */
	spinlock_t io_lock;
#ifdef CONFIG_DEBUG_FS
	bool panic_reboot_notifier;
#endif
	/* TCO enable bit */
	bool enable;
} iTCO_wdt_private;

/* the watchdog reboot notifier */
static struct notifier_block reboot_notifier;

/* variable to bypass keep alive after reboot notifier call */
static bool bypass_keepalive;

/* module parameters */
#define WATCHDOG_HEARTBEAT 30	/* 30 sec default heartbeat */
static int heartbeat = WATCHDOG_HEARTBEAT;  /* in seconds */
module_param(heartbeat, int, 0);
MODULE_PARM_DESC(heartbeat, "Watchdog timeout in seconds. "
		 "3..614, default="
		 __MODULE_STRING(WATCHDOG_HEARTBEAT) ")");

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, false);
MODULE_PARM_DESC(nowayout,
		 "Watchdog cannot be stopped once started (default="
		 __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

static void iTCO_wdt_set_NO_REBOOT_bit(void)
{
	u32 val32;
	void __iomem *pmc_cfg;

	pmc_cfg = ioremap(PMC_CFG, 4);
	if (!pmc_cfg) {
		pr_err("cannot ioremap PCM_CFG register (@=0x%x)\n",
		       PMC_CFG);
		return;
	}

	val32 = readl(pmc_cfg);
	val32 |= 0x00000010;
	writel(val32, pmc_cfg);

	iounmap(pmc_cfg);
}

static int iTCO_wdt_unset_NO_REBOOT_bit(void)
{
	int ret = 0;
	u32 val32;
	void __iomem *pmc_cfg;

	pmc_cfg = ioremap(PMC_CFG, 4);
	if (!pmc_cfg) {
		pr_err("cannot ioremap PCM_CFG register (@=0x%x)\n",
				PMC_CFG);
		return -EIO;
	}

	val32 = readl(pmc_cfg);
	val32 &= 0xffffffef;
	writel(val32, pmc_cfg);
	val32 = readl(pmc_cfg);
	if (val32 & 0x00000010)
		ret = -EIO;

	iounmap(pmc_cfg);

	return ret;
}

static int iTCO_wdt_start(void)
{
	unsigned int val;

	if (iTCO_wdt_private.enable == false)
		return 0;

	spin_lock(&iTCO_wdt_private.io_lock);

	/* Force the timer to its reload value by writing to the TCO_RLD
	   register */
	outw(0x01, TCO_RLD);

	/* Bit 11: TCO Timer Halt -> 0 = The TCO timer is enabled to count */
	val = inw(TCO1_CNT);
	val &= 0xf7ff;
	outw(val, TCO1_CNT);
	val = inw(TCO1_CNT);
	spin_unlock(&iTCO_wdt_private.io_lock);

	if (val & 0x0800)
		return -1;
	return 0;
}

static int iTCO_wdt_stop(void)
{
	unsigned int val;

	spin_lock(&iTCO_wdt_private.io_lock);

	/* Bit 11: TCO Timer Halt -> 1 = The TCO timer is disabled */
	val = inw(TCO1_CNT);
	val |= 0x0800;
	outw(val, TCO1_CNT);
	val = inw(TCO1_CNT);

	spin_unlock(&iTCO_wdt_private.io_lock);

	if ((val & 0x0800) == 0)
		return -1;
	return 0;
}

static int iTCO_wdt_keepalive(void)
{
	pr_info("%s\n", __func__);

	if (bypass_keepalive) {
		pr_info("Reboot on going keep alive is bypassed\n");
		return -EBUSY;
	}

	spin_lock(&iTCO_wdt_private.io_lock);

	/* Reload the timer by writing to the TCO Timer Counter register */
	outw(0x01, TCO_RLD);

	spin_unlock(&iTCO_wdt_private.io_lock);
	return 0;
}

static int iTCO_wdt_set_heartbeat(int t)
{
	unsigned int val16;

	/* For TCO v1 the timer counts down twice before rebooting */
	/* from the specs: */
	/* "Values of 0h-1h are ignored and should not be attempted" */
	if (t < 0x02)
		return -EINVAL;

	/* Write new heartbeat to watchdog */
	spin_lock(&iTCO_wdt_private.io_lock);
	val16 = inw(TCOv2_TMR);
	val16 &= 0xfc00;
	val16 |= t;
	outw(val16, TCOv2_TMR);
	val16 = inw(TCOv2_TMR);
	spin_unlock(&iTCO_wdt_private.io_lock);

	if ((val16 & 0x3ff) != t)
		return -EINVAL;
	heartbeat = t;
	return 0;
}

static void iTCO_wdt_last_kick(int last_delay)
{
	/* Set new heart beat giving last_delay to kernel to shutdown cleanly */
	iTCO_wdt_set_heartbeat(last_delay);

	/* Do a last keep alive before bypassing to allow shutdown
	to proceed with new timing */
	iTCO_wdt_keepalive();
	bypass_keepalive = true;
}

static int iTCO_wdt_get_timeleft(int *time_left)
{
	unsigned int val16;

	/* read the TCO Timer */
	spin_lock(&iTCO_wdt_private.io_lock);
	val16 = inw(TCO_RLD);
	val16 &= 0x3ff;
	spin_unlock(&iTCO_wdt_private.io_lock);

	*time_left = val16;

	return 0;
}

/*
 *	/dev/watchdog handling
 */

static int iTCO_wdt_open(struct inode *inode, struct file *file)
{
	/* /dev/watchdog can only be opened once */
	if (test_and_set_bit(0, &is_active))
		return -EBUSY;

	/*
	 *      Reload and activate timer
	 */
	iTCO_wdt_start();
	return nonseekable_open(inode, file);
}

static int iTCO_wdt_release(struct inode *inode, struct file *file)
{
	/*
	 *      Shut off the timer.
	 */
	if (expect_release == 42) {
		iTCO_wdt_stop();
	} else {
		pr_crit("Unexpected close, not stopping watchdog!\n");
		iTCO_wdt_keepalive();
	}
	clear_bit(0, &is_active);
	expect_release = 0;
	return 0;
}

static ssize_t iTCO_wdt_write(struct file *file, const char __user *data,
			      size_t len, loff_t *ppos)
{
	/* See if we got the magic character 'V' and reload the timer */
	if (len) {
		if (!nowayout) {
			size_t i;

			/* note: just in case someone wrote the magic
			   character five months ago... */
			expect_release = 0;

			/* scan to see whether or not we got the
			   magic character */
			for (i = 0; i != len; i++) {
				char c;
				if (get_user(c, data + i))
					return -EFAULT;
				if (c == 'V')
					expect_release = 42;
			}
		}

		/* someone wrote to us, we should reload the timer */
		iTCO_wdt_keepalive();
	}
	return len;
}

static unsigned int iTCO_wdt_get_current_ospolicy(void)
{
	unsigned int val;

	spin_lock(&iTCO_wdt_private.io_lock);

	val = inl(TCO1_CNT);

	val &= (TCO_POLICY_MASK << TCO_POLICY_OFFSET);
	val >>= TCO_POLICY_OFFSET;

	spin_unlock(&iTCO_wdt_private.io_lock);

	return val;
}

static void iTCO_wdt_set_reset_type(int reset_type)
{
	int val;

	spin_lock(&iTCO_wdt_private.io_lock);

	iTCO_wdt_private.iTCO_wdt_action = reset_type;

	val = inl(TCO1_CNT);

	val &= ~(TCO_POLICY_MASK << TCO_POLICY_OFFSET);
	val |= reset_type << TCO_POLICY_OFFSET;

	outl(val, TCO1_CNT);

	spin_unlock(&iTCO_wdt_private.io_lock);
}

#ifdef CONFIG_DEBUG_FS

static ssize_t iTCO_wdt_reset_type_write(struct file *file, const char __user *buff,
		size_t count, loff_t *ppos)
{
	char reset_type_str[STRING_RESET_TYPE_MAX_LEN];
	unsigned long res;

	if (count >= STRING_RESET_TYPE_MAX_LEN) {
		pr_err("Invalid size %s %d/%d\n", reset_type_str, count,
					STRING_RESET_TYPE_MAX_LEN);
		return -EINVAL;
	}

	memset(reset_type_str, 0x00, STRING_RESET_TYPE_MAX_LEN);

	res = copy_from_user((void *)reset_type_str,
		(void __user *)buff,
		(unsigned long)min((unsigned long)(count-1),
		(unsigned long)(STRING_RESET_TYPE_MAX_LEN-1)));

	if (res) {
		pr_err("%s: copy from user failed\n", __func__);
		return -EINVAL;
	}

	if (!strncmp(reset_type_str, STRING_COLD_OFF, STRING_RESET_TYPE_MAX_LEN)) {
		iTCO_wdt_set_reset_type(TCO_POLICY_HALT);
	} else if (!strncmp(reset_type_str, STRING_COLD_RESET,
					STRING_RESET_TYPE_MAX_LEN)) {
		iTCO_wdt_set_reset_type(TCO_POLICY_NORM);
	} else {
		pr_err("Reset type %s is unknown - Invalid value\n", reset_type_str);
		return -EINVAL;
	}

	return count;
}

static ssize_t iTCO_wdt_reset_type_read(struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	ssize_t len = 0;
	unsigned long res;

	if (*ppos > 0)
		return 0;

	switch(iTCO_wdt_private.iTCO_wdt_action) {
	case TCO_POLICY_NORM:
		len = ARRAY_SIZE(STRING_COLD_RESET);
		res = copy_to_user(buff, STRING_COLD_RESET "\n", len + 1);
		break;
	case TCO_POLICY_HALT:
		len = ARRAY_SIZE(STRING_COLD_OFF);
		res = copy_to_user(buff, STRING_COLD_OFF "\n", len + 1);
		break;
	default:
		return -EINVAL;
	}

	if (res) {
		pr_err("%s: copy to user failed\n", __func__);
		return -EINVAL;
	}

	*ppos += len;
	return len;
}

static const struct file_operations iTCO_wdt_reset_type_fops = {
	.read = iTCO_wdt_reset_type_read,
	.write = iTCO_wdt_reset_type_write,
};

#define STR_MAX_LEN 30
static ssize_t tl_read(struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	ssize_t len = 0;
	unsigned long res;
	unsigned char str[STR_MAX_LEN];
	int timeleft;

	if (*ppos > 0)
		return 0;

	if (iTCO_wdt_get_timeleft(&timeleft))
		return -EINVAL;

	sprintf(str, "%d\n", timeleft);
	if (strlen(str) > count)
		return -EFAULT;

	res = copy_to_user(buff, str, strlen(str));
	if (res) {
		pr_err("%s: copy to user failed\n", __func__);
		return -EINVAL;
	}

	len = strlen(str);

	*ppos += len;
	return len;
}

static const struct file_operations tl_fops = {
	.read = tl_read,
};


static ssize_t iTCO_wdt_trigger_write(struct file *file, const char __user *buff,
		size_t count, loff_t *ppos)
{
	iTCO_wdt_last_kick(1);

	return 0;
}

static const struct file_operations iTCO_wdt_trigger_fops = {
	.write = iTCO_wdt_trigger_write,
};

#endif /* CONFIG_DEBUG_FS */

static long iTCO_wdt_ioctl(struct file *file, unsigned int cmd,
							unsigned long arg)
{
	int new_options, retval = -EINVAL;
	int new_heartbeat;
	void __user *argp = (void __user *)arg;
	int __user *p = argp;
	static const struct watchdog_info ident = {
		.options =		WDIOF_SETTIMEOUT |
					WDIOF_KEEPALIVEPING |
					WDIOF_MAGICCLOSE,
		.firmware_version =	0,
		.identity =		DRV_NAME,
	};

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		return copy_to_user(argp, &ident, sizeof(ident)) ? -EFAULT : 0;
	case WDIOC_GETSTATUS:
	case WDIOC_GETBOOTSTATUS:
		return put_user(0, p);

	case WDIOC_SETOPTIONS:
	{
		if (get_user(new_options, p))
			return -EFAULT;

		if (new_options & WDIOS_DISABLECARD) {
			iTCO_wdt_stop();
			retval = 0;
		}
		if (new_options & WDIOS_ENABLECARD) {
			iTCO_wdt_keepalive();
			iTCO_wdt_start();
			retval = 0;
		}
		return retval;
	}
	case WDIOC_KEEPALIVE:
		iTCO_wdt_keepalive();
		return 0;

	case WDIOC_SETTIMEOUT:
	{
		if (get_user(new_heartbeat, p))
			return -EFAULT;
		if (iTCO_wdt_set_heartbeat(new_heartbeat))
			return -EINVAL;
		iTCO_wdt_keepalive();
		/* Fall */
	}
	case WDIOC_GETTIMEOUT:
		return put_user(heartbeat, p);
	case WDIOC_GETTIMELEFT:
	{
		int time_left;
		if (iTCO_wdt_get_timeleft(&time_left))
			return -EINVAL;
		return put_user(time_left, p);
	}
	default:
		return -ENOTTY;
	}
}

/*
 *	Kernel Interfaces
 */

static const struct file_operations iTCO_wdt_fops = {
	.owner =		THIS_MODULE,
	.llseek =		no_llseek,
	.write =		iTCO_wdt_write,
	.unlocked_ioctl =	iTCO_wdt_ioctl,
	.open =			iTCO_wdt_open,
	.release =		iTCO_wdt_release,
};

static struct miscdevice iTCO_wdt_miscdev = {
	.minor =	WATCHDOG_MINOR,
	.name =		"watchdog",
	.fops =		&iTCO_wdt_fops,
};

/*
 *	Reboot notifier
 */

static int TCO_reboot_notifier(struct notifier_block *this,
			   unsigned long code,
			   void *another_unused)
{
	if (code == SYS_HALT || code == SYS_POWER_OFF) {
		iTCO_wdt_set_reset_type(TCO_POLICY_HALT);
	}

	iTCO_wdt_last_kick(5);

#ifdef CONFIG_DEBUG_FS
	if (iTCO_wdt_private.panic_reboot_notifier) {
		BUG();
	}
#endif

	return NOTIFY_DONE;
}

static irqreturn_t tco_irq_handler(int irq, void *arg)
{
	pr_warn("[SHTDWN] %s, WATCHDOG TIMEOUT HANDLER!\n", __func__);

	/* reduce the timeout to the minimum, but sufficient for tracing */
	bypass_keepalive = false;
	iTCO_wdt_last_kick(15);

	trigger_all_cpu_backtrace();

	/* Let the watchdog reset the board */
	panic_timeout = 0;
	panic("Kernel Watchdog");

	/* This code should not be reached */

	return IRQ_HANDLED;
}


static ssize_t shutdown_ongoing_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t size)
{
	iTCO_wdt_set_reset_type(TCO_POLICY_HALT);
	return size;
}


static ssize_t reboot_ongoing_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t size)
{
	iTCO_wdt_set_reset_type(TCO_POLICY_NORM);
	return size;
}

/* Watchdog behavior depending on system phase */
static DEVICE_ATTR(shutdown_ongoing, S_IWUSR, NULL, shutdown_ongoing_store);
static DEVICE_ATTR(reboot_ongoing, S_IWUSR, NULL, reboot_ongoing_store);

static int create_watchdog_sysfs_files(void)
{
	int ret;

	ret = device_create_file(iTCO_wdt_miscdev.this_device,
			&dev_attr_shutdown_ongoing);
	if (ret) {
		pr_warn("cant register dev file for shutdown_ongoing\n");
		return ret;
	}

	ret = device_create_file(iTCO_wdt_miscdev.this_device,
			&dev_attr_reboot_ongoing);
	if (ret) {
		pr_warn("cant register dev file for reboot_ongoing\n");
		return ret;
	}
	return 0;
}

static int remove_watchdog_sysfs_files(void)
{
	device_remove_file(iTCO_wdt_miscdev.this_device,
		&dev_attr_shutdown_ongoing);
	device_remove_file(iTCO_wdt_miscdev.this_device,
		&dev_attr_reboot_ongoing);
	return 0;
}

/*
 *	Init & exit routines
 */

static int iTCO_wdt_init(struct platform_device *pdev)
{
	int ret;
	u32 base_address;
	unsigned long val32;
	struct pci_dev *parent;
	struct resource *irq;

	if (!pdev->dev.parent || !dev_is_pci(pdev->dev.parent)) {
		pr_err("Unqualified parent device.\n");
		return -EINVAL;
	}

	parent = to_pci_dev(pdev->dev.parent);

	/*
	 *      Find the ACPI/PM base I/O address which is the base
	 *      for the TCO registers (TCOBASE=ACPIBASE + 0x60)
	 *      ACPIBASE is bits [15:7] from 0x40-0x43
	 */
	pci_read_config_dword(parent, 0x40, &base_address);
	base_address &= 0x0000ff80;
	if (base_address == 0x00000000) {
		/* Something's wrong here, ACPIBASE has to be set */
		pr_err("failed to get TCOBASE address, device disabled by hardware/BIOS\n");
		return -ENODEV;
	}
	iTCO_wdt_private.ACPIBASE = base_address;

	pci_read_config_dword(parent, 0x44, &pmc_base_address);
	pmc_base_address &= 0xFFFFFE00;

	/*
	 * Disable watchdog on command-line demand
	 */
	if (strstr(saved_command_line, "disable_kernel_watchdog=1")) {
		pr_warn("disable_kernel_watchdog=1 watchdog will not be started\n");
		iTCO_wdt_private.enable = false;
		/* Set the NO_REBOOT bit to prevent later reboots */
		iTCO_wdt_set_NO_REBOOT_bit();
		/* Ensure Wdt is well stopped in case started by IAFW */
		iTCO_wdt_stop();
	} else {
		iTCO_wdt_private.enable = true;
		/* Check chipset's NO_REBOOT bit */
		if (iTCO_wdt_unset_NO_REBOOT_bit()) {
			pr_err("unable to reset NO_REBOOT flag, device disabled by hardware/BIOS\n");
			ret = -ENODEV;	/* Cannot reset NO_REBOOT bit */
			goto out;
		}
	}

	/* The TCO logic uses the TCO_EN bit in the SMI_EN register */
	if (!request_region(SMI_EN, 4, "iTCO_wdt")) {
		pr_err("I/O address 0x%04lx already in use, device disabled\n",
		       SMI_EN);
		ret = -EIO;
		goto out;
	}
	if (!request_region(SMI_STS, 4,	"iTCO_wdt")) {
		pr_err("I/O address 0x%04lx already in use, device disabled\n",
				SMI_STS);
		ret = -EIO;
		goto unreg_smi_sts;
	}

	/* The TCO I/O registers reside in a 32-byte range pointed to
	   by the TCOBASE value */
	if (!request_region(TCOBASE, 0x20, "iTCO_wdt")) {
		pr_err("I/O address 0x%04lx already in use, device disabled\n",
		       TCOBASE);
		ret = -EIO;
		goto unreg_smi_en;
	}

	pr_info("Found a TCO device (TCOBASE=0x%04lx)\n", TCOBASE);

	/* Check that the heartbeat value is within it's range;
	   if not reset to the default */
	if (iTCO_wdt_set_heartbeat(heartbeat)) {
		iTCO_wdt_set_heartbeat(WATCHDOG_HEARTBEAT);
		pr_info("timeout value out of range, using %d\n", heartbeat);
	}

	ret = misc_register(&iTCO_wdt_miscdev);
	if (ret != 0) {
		pr_err("cannot register miscdev on minor=%d (err=%d)\n",
		       WATCHDOG_MINOR, ret);
		goto unreg_region;
	}

	pr_info("initialized. heartbeat=%d sec (nowayout=%d) policy=0x%x\n",
		heartbeat, nowayout, iTCO_wdt_get_current_ospolicy());

	/* Reset OS policy */
	iTCO_wdt_set_reset_type(TCO_POLICY_NORM);

	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq) {
		pr_err("No warning interrupt resource found\n");
		goto misc_unreg;
	}

	ret = acpi_register_gsi(NULL, irq->start,
				irq->flags & (IORESOURCE_IRQ_HIGHEDGE | IORESOURCE_IRQ_LOWEDGE)
				? ACPI_EDGE_SENSITIVE : ACPI_LEVEL_SENSITIVE,
				irq->flags & (IORESOURCE_IRQ_HIGHEDGE | IORESOURCE_IRQ_HIGHLEVEL)
				? ACPI_ACTIVE_HIGH : ACPI_ACTIVE_LOW);
	if (ret < 0) {
		pr_err("failed to configure TCO warning IRQ %d\n", (int)irq->start);
		goto misc_unreg;
	}

	ret = request_irq(irq->start, tco_irq_handler, 0, "tco_watchdog", NULL);
	if (ret < 0) {
		pr_err("failed to request TCO warning IRQ %d\n", (int)irq->start);
		goto gsi_unreg;
	}

	/* Clear old TCO timeout status */
	val32 = TCO_TIMEOUT_BIT | SECOND_TO_STS_BIT;
	outl(val32, TCO1_STS);
	/* Clear the SMI status */
	outl(TCO_STS_BIT, SMI_STS);

	/* Enable SMI for TCO */
	val32 = inl(SMI_EN);
	val32 |= TCO_EN_BIT;
	outl(val32, SMI_EN);
	/* then ensure that PMC is ready to handle next SMI */
	val32 |= EOS_BIT;
	outl(val32, SMI_EN);

	reboot_notifier.notifier_call = TCO_reboot_notifier;
	reboot_notifier.priority = 1;
	ret = register_reboot_notifier(&reboot_notifier);
	if (ret)
		/* We continue as reboot notifier is not critical for
			 * watchdog */
		pr_err("cannot register reboot notifier %d\n", ret);

	return 0;

gsi_unreg:
	acpi_unregister_gsi((int)(irq->start));
misc_unreg:
	misc_deregister(&iTCO_wdt_miscdev);
unreg_region:
	release_region(TCOBASE, 0x20);
unreg_smi_sts:
	release_region(SMI_STS, 4);
unreg_smi_en:
	release_region(SMI_EN, 4);
out:
	iTCO_wdt_private.ACPIBASE = 0;
	return ret;
}

static void iTCO_wdt_cleanup(void)
{
	/* Stop the timer before we leave */
	if (!nowayout)
		iTCO_wdt_stop();

	/* Deregister */
	misc_deregister(&iTCO_wdt_miscdev);
	release_region(TCOBASE, 0x20);
	release_region(SMI_EN, 4);
	release_region(SMI_STS, 4);
	unregister_reboot_notifier(&reboot_notifier);
	iTCO_wdt_private.ACPIBASE = 0;
#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(iTCO_debugfs_dir);
#endif /* CONFIG_DEBUG_FS */
}

static int iTCO_wdt_probe(struct platform_device *pdev)
{
	int ret = 0;

	spin_lock_init(&iTCO_wdt_private.io_lock);

	ret = iTCO_wdt_init(pdev);
	if (ret)
		return ret;

#ifdef CONFIG_DEBUG_FS
	iTCO_debugfs_dir = debugfs_create_dir("iTCO", NULL);
	debugfs_create_file("timeleft", S_IRUSR,
			    iTCO_debugfs_dir, NULL, &tl_fops);
	debugfs_create_file("reset_type", S_IRUSR | S_IWUSR,
			    iTCO_debugfs_dir, NULL, &iTCO_wdt_reset_type_fops);
	debugfs_create_file("trigger", S_IWUSR,
			    iTCO_debugfs_dir, NULL, &iTCO_wdt_trigger_fops);
	debugfs_create_bool("panic_reboot_notifier", S_IRUSR | S_IWUSR,
			    iTCO_debugfs_dir,
			    (u32 *)&iTCO_wdt_private.panic_reboot_notifier);
#endif /* CONFIG_DEBUG_FS */

	create_watchdog_sysfs_files();

	return ret;
}

static int iTCO_wdt_remove(struct platform_device *dev)
{
	if (iTCO_wdt_private.ACPIBASE)
		iTCO_wdt_cleanup();

	remove_watchdog_sysfs_files();

	return 0;
}

static void iTCO_wdt_shutdown(struct platform_device *dev)
{
	pr_info("Shutdown not stopping watchdog");
}

static int iTCO_wdt_suspend(struct platform_device *dev, pm_message_t state)
{
	return iTCO_wdt_stop();
}

static int iTCO_wdt_resume(struct platform_device *dev)
{
	return iTCO_wdt_start();
}

static struct platform_driver iTCO_wdt_driver = {
	.probe          = iTCO_wdt_probe,
	.remove         = (iTCO_wdt_remove),
	.shutdown       = iTCO_wdt_shutdown,
	.driver         = {
		.owner  = THIS_MODULE,
		.name   = DRV_NAME,
	},
	.suspend        = iTCO_wdt_suspend,
	.resume         = iTCO_wdt_resume,
};

static int __init iTCO_wdt_init_module(void)
{
	pr_info("Intel TCO WatchDog Timer Driver v%s\n", DRV_VERSION);
	return platform_driver_register(&iTCO_wdt_driver);
}

static void __exit iTCO_wdt_cleanup_module(void)
{
	platform_driver_unregister(&iTCO_wdt_driver);
	pr_info("Intel TCO WatchDog Timer Driver unloaded\n");
}

module_init(iTCO_wdt_init_module);
module_exit(iTCO_wdt_cleanup_module);

MODULE_AUTHOR("Wim Van Sebroeck <wim@iguana.be>");
MODULE_DESCRIPTION("Intel TCO WatchDog Timer Driver");
MODULE_VERSION(DRV_VERSION);
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
