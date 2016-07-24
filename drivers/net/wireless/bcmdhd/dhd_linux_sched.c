/*
 * Expose some of the kernel scheduler routines
 *
 * $ Copyright Open Broadcom Corporation $
 *
 *
 * <<Broadcom-WL-IPTag/Open:>>
 *
 * $Id: dhd_linux_sched.c 514727 2014-11-12 03:02:48Z $
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <typedefs.h>
#include <linuxver.h>

int setScheduler(struct task_struct *p, int policy, struct sched_param *param)
{
	int rc = 0;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0))
	rc = sched_setscheduler(p, policy, param);
#endif /* LinuxVer */
	return rc;
}

int get_scheduler_policy(struct task_struct *p)
{
	int rc = SCHED_NORMAL;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0))
	rc = p->policy;
#endif /* LinuxVer */
	return rc;
}
