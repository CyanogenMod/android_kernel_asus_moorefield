/* drivers/misc/lowmemorykiller.c
 *
 * The lowmemorykiller driver lets user-space specify a set of memory thresholds
 * where processes with a range of oom_score_adj values will get killed. Specify
 * the minimum oom_score_adj values in
 * /sys/module/lowmemorykiller/parameters/adj and the number of free pages in
 * /sys/module/lowmemorykiller/parameters/minfree. Both files take a comma
 * separated list of numbers in ascending order.
 *
 * For example, write "0,8" to /sys/module/lowmemorykiller/parameters/adj and
 * "1024,4096" to /sys/module/lowmemorykiller/parameters/minfree to kill
 * processes with a oom_score_adj value of 8 or higher when the free memory
 * drops below 4096 pages and kill processes with a oom_score_adj value of 0 or
 * higher when the free memory drops below 1024 pages.
 *
 * The driver considers memory used for caches to be free, but if a large
 * percentage of the cached memory is locked this can be very inaccurate
 * and processes may not get killed until the normal oom killer is triggered.
 *
 * Copyright (C) 2007-2008 Google, Inc.
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

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/oom.h>
#include <linux/sched.h>
#include <linux/swap.h>
#include <linux/rcupdate.h>
#include <linux/notifier.h>
#include <linux/delay.h>
#include <linux/swap.h>
#include <linux/fs.h>
#include <linux/cpuset.h>

static uint32_t lowmem_debug_level = 1;
static short lowmem_adj[6] = {
	0,
	1,
	6,
	12,
};
static int lowmem_adj_size = 4;
static int lowmem_minfree[6] = {
	3 * 512,	/* 6MB */
	2 * 1024,	/* 8MB */
	4 * 1024,	/* 16MB */
	16 * 1024,	/* 64MB */
};
static int lowmem_minfree_size = 4;

static unsigned long lowmem_deathpending_timeout;

#define lowmem_print(level, x...)			\
	do {						\
		if (lowmem_debug_level >= (level))	\
			pr_info(x);			\
	} while (0)

static int is4gDram = 0;
extern int Read_TOTAL_DRAM(void);

//ASUS
#define DTaskMax 4
struct task_node {
     struct list_head node;
     struct task_struct *pTask;
};
enum EListAllocate {
	EListAllocate_Init,
	EListAllocate_Head,
	EListAllocate_Body,
	EListAllocate_Tail
};
static int list_insert(struct task_struct *pTask, struct list_head *pPosition)
{
	int nResult = 0;
	struct task_node *pTaskNode;
	pTaskNode = kmalloc(sizeof(struct task_node), GFP_KERNEL);
	if (pTaskNode) {
		pTaskNode->pTask = pTask;
		list_add(&pTaskNode->node, pPosition);
		nResult = 1;
	}
	return nResult;
}
static void list_reset(struct list_head *pList)
{
	struct task_node *pTaskIterator;
	struct task_node *pTaskNext;
	list_for_each_entry_safe(pTaskIterator, pTaskNext, pList, node) {
		list_del(&pTaskIterator->node);
		kfree(pTaskIterator);
	}
}
//ASUS


#define ASUS_MEMORY_DEBUG_MAXLEN    (128)
#define ASUS_MEMORY_DEBUG_MAXCOUNT  (256)
#define HEAD_LINE "PID       RSS    oom_adj       cmdline\n"
char meminfo_str[ASUS_MEMORY_DEBUG_MAXCOUNT][ASUS_MEMORY_DEBUG_MAXLEN];

static int lowmem_shrink(struct shrinker *s, struct shrink_control *sc)
{
	struct task_struct *tsk;
	struct task_struct *selected = NULL;
	int rem = 0;
	int tasksize;
	int i;
	short min_score_adj = OOM_SCORE_ADJ_MAX + 1;
	int minfree = 0;
	int selected_tasksize = 0;
	short selected_oom_score_adj;
	int array_size = ARRAY_SIZE(lowmem_adj);
	int nr_swap_pages = get_nr_swap_pages();
	int other_free = global_page_state(NR_FREE_PAGES) - totalreserve_pages + nr_swap_pages;
	int other_file = global_page_state(NR_FILE_PAGES) -
						global_page_state(NR_SHMEM);
	//ASUS
	LIST_HEAD(ListHead);
	int nTaskNum = 0;
	int meminfo_str_index = 0;
	struct task_node *pTaskIterator;
	struct task_node *pTaskNext;
	//ASUS

	// BZ7024>>
	int dma32_free = 0, dma32_file = 0;
	struct zone *zone;
	// BZ7024<<
	if (lowmem_adj_size < array_size)
		array_size = lowmem_adj_size;
	if (lowmem_minfree_size < array_size)
		array_size = lowmem_minfree_size;
	for (i = 0; i < array_size; i++) {
		minfree = lowmem_minfree[i];
		if (other_free < minfree && other_file < minfree) {
			min_score_adj = lowmem_adj[i];
			break;
		}
	}
	if (sc->nr_to_scan > 0)
		lowmem_print(3, "lowmem_shrink %lu, %x, ofree %d %d, ma %hd\n",
				sc->nr_to_scan, sc->gfp_mask, other_free,
				other_file, min_score_adj);
	//BZ7024>>
	if(is4gDram == 1) {
		if (min_score_adj == OOM_SCORE_ADJ_MAX + 1) {
			/*
			 * Shrink callback is called but global memory is ok. Maybe
			 * DMA32 zone memory is in short.
			 */
			for_each_populated_zone(zone) {
				if (is_dma32(zone)) {
					dma32_free = zone_page_state(zone, NR_FREE_PAGES);
					dma32_file = zone_page_state(zone, NR_FILE_PAGES)
						- zone_page_state(zone, NR_SHMEM);
				}
			}

			for (i = 0; i < 1; i++) {
				// 6*1024 will cause some fail case
				// EX : TT 634316
				minfree = lowmem_minfree[i];

				if (dma32_free && dma32_free < minfree
						&& dma32_file < minfree) {
					min_score_adj = lowmem_adj[i];
					lowmem_print(1, "dma32 is in short: free:%d, file:%d, min_score_adj:%hu\n",
						dma32_free, dma32_file, min_score_adj);
					break;
				}
			}

			lowmem_print(3, "dma32: free:%d, file:%d, min_score_adj:%hu\n",
					dma32_free, dma32_file, min_score_adj);
		}
	}
	//BZ7024<<
	rem = global_page_state(NR_ACTIVE_ANON) +
		global_page_state(NR_ACTIVE_FILE) +
		global_page_state(NR_INACTIVE_ANON) +
		global_page_state(NR_INACTIVE_FILE);
	if (sc->nr_to_scan <= 0 || min_score_adj == OOM_SCORE_ADJ_MAX + 1) {
		lowmem_print(5, "lowmem_shrink %lu, %x, return %d\n",
			     sc->nr_to_scan, sc->gfp_mask, rem);
		return rem;
	}
	selected_oom_score_adj = min_score_adj;

	rcu_read_lock();
	for_each_process(tsk) {
		struct task_struct *p;
		short oom_score_adj;
		
		//ASUS
		int nError = 0;
		struct list_head *pInsertPos = NULL;
		//ASUS

		if (tsk->flags & PF_KTHREAD)
			continue;

		p = find_lock_task_mm(tsk);
		if (!p)
			continue;

		if (test_tsk_thread_flag(p, TIF_MEMDIE)) {
			if (time_before_eq(jiffies,
				lowmem_deathpending_timeout)) {
				list_reset(&ListHead);	//ASUS
				task_unlock(p);
				rcu_read_unlock();
				return 0;
			} else {
				task_unlock(p);
				continue;
			}
		}
		oom_score_adj = p->signal->oom_score_adj;
		
		//ASUS
		tasksize = get_mm_rss(p->mm);
		if(meminfo_str_index >= ASUS_MEMORY_DEBUG_MAXCOUNT )
			meminfo_str_index = ASUS_MEMORY_DEBUG_MAXCOUNT - 1;
		snprintf(meminfo_str[meminfo_str_index++], ASUS_MEMORY_DEBUG_MAXLEN, "%6d  %8ldkB %8d %s\n", p->pid, tasksize * (long)(PAGE_SIZE / 1024),oom_score_adj, p->comm);
		//ASUS
		
		if (oom_score_adj < min_score_adj) {
			task_unlock(p);
			continue;
		}
		task_unlock(p);
		if (tasksize <= 0)
			continue;
		//ASUS
		lowmem_print(3, "[Candidate] comm: %s, oom_score_adj: %d, tasksize: %d\n", p->comm, oom_score_adj, tasksize);
		if (nTaskNum == 0) {
			if (list_insert(p, &ListHead))
				nTaskNum++;
			else
				lowmem_print(1, "Unable to allocate memory (%d)\n", EListAllocate_Init);
		}
		else	//ASUS
		{
			/* Find the node index that fit for current task */
			struct task_node *pTaskSearchIterator;
			struct task_node *pTaskSearchNext;
			list_for_each_entry_safe(pTaskSearchIterator, pTaskSearchNext, &ListHead, node) {
				int nTargeteAdj = 0;
				int nTargetSize = 0;
				struct task_struct *pTask;
				pTask = pTaskSearchIterator->pTask;
				if (!pTask)
					continue;
				task_lock(pTask);
				if (pTask->signal)
					nTargeteAdj = pTask->signal->oom_score_adj;
				if (pTask->mm)
					nTargetSize = get_mm_rss(pTask->mm);
				task_unlock(pTask);
				if (oom_score_adj > nTargeteAdj) {
				} 
				else if (oom_score_adj < nTargeteAdj) {
					break;
				} 
				else {
					if (tasksize > nTargetSize) {
					} 
					else if (tasksize <= nTargetSize) {
						break;
					}
				}
			}
			/* Determine the insert position */
			if (&pTaskSearchIterator->node == &ListHead) {
				/* Add node to tail */
				pInsertPos = ListHead.prev;
				nError = EListAllocate_Tail;
			} else if (&pTaskSearchIterator->node == ListHead.next) {
				if (nTaskNum < DTaskMax) {
					/* Add node to head */
					pInsertPos = &ListHead;
					nError = EListAllocate_Head;
				}
			} else {
				/* Insert node to the list */
				pInsertPos = pTaskSearchIterator->node.prev;
				nError = EListAllocate_Body;
			}
			/* Perform insertion */
			if (pInsertPos) {
				if (list_insert(p, pInsertPos))
					nTaskNum++;
				else
					lowmem_print(1, "Unable to allocate memory (%d)\n", nError);
			}
			/* Delete node if the kept tasks exceed the limit */
			if (nTaskNum > DTaskMax) {
				struct task_node *pDeleteNode = list_entry((ListHead).next, struct task_node, node);
				list_del((ListHead).next);
				kfree(pDeleteNode);
				nTaskNum--;
			}
		}	//ASUS
		
		/*
		if (selected) {
			if (oom_score_adj < selected_oom_score_adj)
				continue;
			if (oom_score_adj == selected_oom_score_adj &&
			    tasksize <= selected_tasksize)
				continue;
		}
		selected = p;
		selected_tasksize = tasksize;
		selected_oom_score_adj = oom_score_adj;
		lowmem_print(2, "select '%s' (%d), adj %hd, size %d, to kill\n",
			     p->comm, p->pid, oom_score_adj, tasksize);
		*/
	}
	list_for_each_entry_safe_reverse(pTaskIterator, pTaskNext, &ListHead, node)	//ASUS
	{
		selected = pTaskIterator->pTask;
		if (!selected)
			continue;
		task_lock(selected);
		if (test_tsk_thread_flag(selected, TIF_MEMDIE)) {
			task_unlock(selected);
			continue;
		}
		if (selected->signal)
			selected_oom_score_adj = selected->signal->oom_score_adj;
		if (selected->mm)
			selected_tasksize = get_mm_rss(selected->mm);
		task_unlock(selected);
		
		if(selected_oom_score_adj < 1000){
			int count = 0;
			printk(HEAD_LINE);
			while (count < meminfo_str_index ){
				printk(meminfo_str[count]);
				count++;
			}
		}
		//ASUS
		
		lowmem_print(1, "Killing '%s' (%d), adj %hd,\n" \
				"   to free %ldkB on behalf of '%s' (%d) because\n" \
				"   cache %ldkB is below limit %ldkB for oom_score_adj %hd\n" \
				"   Free memory is %ldkB above reserved\n",
			     selected->comm, selected->pid,
			     selected_oom_score_adj,
			     selected_tasksize * (long)(PAGE_SIZE / 1024),
			     current->comm, current->pid,
			     other_file * (long)(PAGE_SIZE / 1024),
			     minfree * (long)(PAGE_SIZE / 1024),
			     min_score_adj,
			     other_free * (long)(PAGE_SIZE / 1024));
		lowmem_deathpending_timeout = jiffies + HZ;
		send_sig(SIGKILL, selected, 0);
		set_tsk_thread_flag(selected, TIF_MEMDIE);
		rem -= selected_tasksize;
		
		/* [ASUS] give the system time to free up the memory */
		msleep_interruptible(20);
	}
	list_reset(&ListHead);	//ASUS
	lowmem_print(4, "lowmem_shrink %lu, %x, return %d\n",
		     sc->nr_to_scan, sc->gfp_mask, rem);
	rcu_read_unlock();
	return rem;
}

static struct shrinker lowmem_shrinker = {
	.shrink = lowmem_shrink,
	.seeks = DEFAULT_SEEKS * 16
};

static int __init lowmem_init(void)
{
	is4gDram = Read_TOTAL_DRAM()>3000?1:0;
	pr_info("DRAM total size = %dM, Is 4G device = %d\n", Read_TOTAL_DRAM(), is4gDram);
	register_shrinker(&lowmem_shrinker);
	return 0;
}

static void __exit lowmem_exit(void)
{
	unregister_shrinker(&lowmem_shrinker);
}

#ifdef CONFIG_ANDROID_LOW_MEMORY_KILLER_AUTODETECT_OOM_ADJ_VALUES
static short lowmem_oom_adj_to_oom_score_adj(short oom_adj)
{
	if (oom_adj == OOM_ADJUST_MAX)
		return OOM_SCORE_ADJ_MAX;
	else
		return (oom_adj * OOM_SCORE_ADJ_MAX) / -OOM_DISABLE;
}

static void lowmem_autodetect_oom_adj_values(void)
{
	int i;
	short oom_adj;
	short oom_score_adj;
	int array_size = ARRAY_SIZE(lowmem_adj);

	if (lowmem_adj_size < array_size)
		array_size = lowmem_adj_size;

	if (array_size <= 0)
		return;

	oom_adj = lowmem_adj[array_size - 1];
	if (oom_adj > OOM_ADJUST_MAX)
		return;

	oom_score_adj = lowmem_oom_adj_to_oom_score_adj(oom_adj);
	if (oom_score_adj <= OOM_ADJUST_MAX)
		return;

	lowmem_print(1, "lowmem_shrink: convert oom_adj to oom_score_adj:\n");
	for (i = 0; i < array_size; i++) {
		oom_adj = lowmem_adj[i];
		oom_score_adj = lowmem_oom_adj_to_oom_score_adj(oom_adj);
		lowmem_adj[i] = oom_score_adj;
		lowmem_print(1, "oom_adj %d => oom_score_adj %d\n",
			     oom_adj, oom_score_adj);
	}
}

static int lowmem_adj_array_set(const char *val, const struct kernel_param *kp)
{
	int ret;

	ret = param_array_ops.set(val, kp);

	/* HACK: Autodetect oom_adj values in lowmem_adj array */
	lowmem_autodetect_oom_adj_values();

	return ret;
}

static int lowmem_adj_array_get(char *buffer, const struct kernel_param *kp)
{
	return param_array_ops.get(buffer, kp);
}

static void lowmem_adj_array_free(void *arg)
{
	param_array_ops.free(arg);
}

static struct kernel_param_ops lowmem_adj_array_ops = {
	.set = lowmem_adj_array_set,
	.get = lowmem_adj_array_get,
	.free = lowmem_adj_array_free,
};

static const struct kparam_array __param_arr_adj = {
	.max = ARRAY_SIZE(lowmem_adj),
	.num = &lowmem_adj_size,
	.ops = &param_ops_short,
	.elemsize = sizeof(lowmem_adj[0]),
	.elem = lowmem_adj,
};
#endif

module_param_named(cost, lowmem_shrinker.seeks, int, S_IRUGO | S_IWUSR);
#ifdef CONFIG_ANDROID_LOW_MEMORY_KILLER_AUTODETECT_OOM_ADJ_VALUES
__module_param_call(MODULE_PARAM_PREFIX, adj,
		    &lowmem_adj_array_ops,
		    .arr = &__param_arr_adj,
		    S_IRUGO | S_IWUSR, -1);
__MODULE_PARM_TYPE(adj, "array of short");
#else
module_param_array_named(adj, lowmem_adj, short, &lowmem_adj_size,
			 S_IRUGO | S_IWUSR);
#endif
module_param_array_named(minfree, lowmem_minfree, uint, &lowmem_minfree_size,
			 S_IRUGO | S_IWUSR);
module_param_named(debug_level, lowmem_debug_level, uint, S_IRUGO | S_IWUSR);

module_init(lowmem_init);
module_exit(lowmem_exit);

MODULE_LICENSE("GPL");

