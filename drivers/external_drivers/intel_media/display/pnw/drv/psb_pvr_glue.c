/*
 * Copyright (c) 2009, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#include "psb_pvr_glue.h"
#include <asm/page.h>

/**
 * FIXME: should NOT use these file under env/linux directly
 */
#include "mm.h"

IMG_UINT32 psb_get_tgid(void)
{
	/* from OSGetCurrentProcessIDKM() */
	if (in_interrupt())
		return KERNEL_ID;

	return (IMG_UINT32)task_tgid_nr(current);
}

int psb_get_meminfo_by_handle(struct drm_psb_private *dev_priv,
		IMG_HANDLE hKernelMemInfo,
		PVRSRV_KERNEL_MEM_INFO **ppsKernelMemInfo)
{
	PVRSRV_KERNEL_MEM_INFO *psKernelMemInfo = IMG_NULL;
	PVRSRV_PER_PROCESS_DATA *psPerProc = IMG_NULL;
	PVRSRV_ERROR eError;

	BUG_ON(!dev_priv->pvr_ops);
	psPerProc = dev_priv->pvr_ops->PVRSRVPerProcessData(
			psb_get_tgid());
	eError = dev_priv->pvr_ops->PVRSRVLookupHandle(
			psPerProc->psHandleBase,
			(IMG_VOID *)&psKernelMemInfo,
			hKernelMemInfo,
			PVRSRV_HANDLE_TYPE_MEM_INFO);
	if (eError != PVRSRV_OK) {
		DRM_ERROR("Cannot find kernel meminfo for handle 0x%x\n",
			  (IMG_UINT32)hKernelMemInfo);
		return -EINVAL;
	}

	*ppsKernelMemInfo = psKernelMemInfo;

	DRM_DEBUG("Got Kernel MemInfo for handle %x\n",
		  (IMG_UINT32)hKernelMemInfo);
	return 0;
}

int psb_get_pages_by_mem_handle(struct drm_psb_private *dev_priv,
				IMG_HANDLE hOSMemHandle,
				u32 **pfn_list,
				int page_count)
{
	LinuxMemArea *psLinuxMemArea = (LinuxMemArea *)hOSMemHandle;
	u32 *pfns = 0;
	IMG_CPU_PHYADDR phys_addr;
	int i;

	if (!pfn_list)
		return -EINVAL;

	/*allocate page list*/
	pfns = kzalloc(page_count * sizeof(u32), GFP_KERNEL);
	if (!pfns) {
		DRM_ERROR("No memory\n");
		return -ENOMEM;
	}

	for (i = 0; i < page_count; i++) {
		phys_addr.uiAddr = 0;

		BUG_ON(!dev_priv->pvr_ops);
		phys_addr = dev_priv->pvr_ops->LinuxMemAreaToCpuPAddr(
				psLinuxMemArea, i * PAGE_SIZE);
		pfns[i] = ((u32)phys_addr.uiAddr) >> PAGE_SHIFT;
	}

	*pfn_list = pfns;
	return 0;
}


int psb_get_vaddr_pages(u32 vaddr, u32 size, u32 **pfn_list, int *page_count)
{
	u32 num_pages;
	struct page **pages = 0;
	struct task_struct *task = current;
	struct mm_struct *mm = task->mm;
	struct vm_area_struct *vma;
	u32 *pfns = 0;
	int ret;
	int i;

	if (unlikely(!pfn_list || !page_count || !vaddr || !size))
		return -EINVAL;

	num_pages = (size + PAGE_SIZE - 1) >> PAGE_SHIFT;

	pages = kzalloc(num_pages * sizeof(struct page *), GFP_KERNEL);
	if (unlikely(!pages)) {
		DRM_ERROR("Failed to allocate page list\n");
		return -ENOMEM;
	}

	down_read(&mm->mmap_sem);
	ret = get_user_pages(task, mm, vaddr, num_pages, 0, 0, pages, NULL);
	up_read(&mm->mmap_sem);

	if (ret <= 0) {
		DRM_DEBUG("failed to get user pages\n");
		kfree(pages);
		pages = 0;
	} else {
		DRM_DEBUG("num_pages %d, ret %d\n", num_pages, ret);
		num_pages = ret;
	}

	/*allocate page list*/
	pfns = kzalloc(num_pages * sizeof(u32), GFP_KERNEL);
	if (!pfns) {
		DRM_ERROR("No memory\n");
		goto get_page_err;
	}

	if (!pages) {
		DRM_ERROR("No pages found, trying to follow pfn\n");
		for (i = 0; i < num_pages; i++) {
			vma = find_vma(mm, vaddr + i * PAGE_SIZE);
			if (!vma) {
				DRM_ERROR("failed to find vma\n");
				goto find_vma_err;
			}

			ret = follow_pfn(vma,
				(unsigned long)(vaddr + i * PAGE_SIZE),
				(unsigned long *)&pfns[i]);
			if (ret) {
				DRM_ERROR("failed to follow pfn\n");
				goto follow_pfn_err;
			}
		}
	} else {
		DRM_ERROR("Found pages\n");
		for (i = 0; i < num_pages; i++)
			pfns[i] = page_to_pfn(pages[i]);
	}

	*pfn_list = pfns;
	*page_count = num_pages;

	kfree(pages);

	return 0;
find_vma_err:
follow_pfn_err:
	kfree(pfns);
get_page_err:
	if (pages) {
		for (i = 0; i < num_pages; i++)
			put_page(pages[i]);
		kfree(pages);
	}
	return -EINVAL;
}

