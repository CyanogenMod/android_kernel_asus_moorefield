#include "drmP.h"
#include "i915_drm.h"
#include "i915_drv.h"
#include "i915_trace.h"
#include "intel_drv.h"
#include <linux/mmu_notifier.h>
#include <linux/swap.h>

static struct
i915_gem_userptr_object *to_userptr_object(struct drm_i915_gem_object *obj)
{
	return container_of(obj, struct i915_gem_userptr_object, gem);
}

int
i915_gem_userptr_obj_pageoffset(struct drm_i915_gem_object *obj) {
	struct i915_gem_userptr_object *vmap = to_userptr_object(obj);
	return offset_in_page(vmap->user_ptr);
}

#if defined(CONFIG_MMU_NOTIFIER)
static void i915_gem_userptr_mn_invalidate_range_start(struct mmu_notifier *mn,
						       struct mm_struct *mm,
						       unsigned long start,
						       unsigned long end)
{
	struct i915_gem_userptr_object *vmap;
	struct drm_device *dev;

	/* XXX race between obj unref and mmu notifier? */
	vmap = container_of(mn, struct i915_gem_userptr_object, mn);
	BUG_ON(vmap->mm != mm);

	if (vmap->user_ptr >= end || vmap->user_ptr + vmap->user_size <= start)
		return;

	if (vmap->gem.pages == NULL) /* opportunistic check */
		return;

	dev = vmap->gem.base.dev;
	mutex_lock(&dev->struct_mutex);
	if (vmap->gem.gtt_space) {
		struct drm_i915_private *dev_priv = dev->dev_private;
		bool was_interruptible;
		int ret;

		was_interruptible = dev_priv->mm.interruptible;
		dev_priv->mm.interruptible = false;

		ret = i915_gem_object_unbind(&vmap->gem);
		BUG_ON(ret && ret != -EIO);

		dev_priv->mm.interruptible = was_interruptible;
	}

	BUG_ON(i915_gem_object_put_pages(&vmap->gem));
	mutex_unlock(&dev->struct_mutex);
}

static void i915_gem_userptr_mn_release(struct mmu_notifier *mn,
					struct mm_struct *mm)
{
	struct i915_gem_userptr_object *vmap;

	vmap = container_of(mn, struct i915_gem_userptr_object, mn);
	BUG_ON(vmap->mm != mm);
	vmap->mm = NULL;

	/* XXX Schedule an eventual unbind? E.g. hook into require request?
	 * However, locking will be complicated.
	 */
}

static const struct mmu_notifier_ops i915_gem_userptr_notifier = {
	.invalidate_range_start = i915_gem_userptr_mn_invalidate_range_start,
	.release = i915_gem_userptr_mn_release, };

static void
i915_gem_userptr_release__mmu_notifier(
			struct i915_gem_userptr_object *vmap) {
	if (vmap->mn.ops && vmap->mm) {
		mmu_notifier_unregister(&vmap->mn, vmap->mm);
		BUG_ON(vmap->mm);
	}
}

static int
i915_gem_userptr_init__mmu_notifier(struct i915_gem_userptr_object *vmap,
				unsigned flags)
{
	if (flags & I915_USERPTR_UNSYNCHRONIZED)
		return capable(CAP_SYS_ADMIN) ? 0 : -EPERM;

	vmap->mn.ops = &i915_gem_userptr_notifier;
	return mmu_notifier_register(&vmap->mn, vmap->mm);
}

#else

static void
i915_gem_userptr_release__mmu_notifier(
			struct i915_gem_userptr_object *vmap)
{
	return;
}

static int
i915_gem_userptr_init__mmu_notifier(struct i915_gem_userptr_object *vmap,
				    unsigned flags)
{
	/* Time being disabling the following checks till MMU Notifier
	 * is not enabled, so as to avoid the change in User Space (UFO)
	 * code, as they assume MMU Notifier is switched on by default,
	 * which is the case with 3.4 kernel code */

	/*
	if ((flags & I915_USERPTR_UNSYNCHRONIZED) == 0)
		return -ENODEV;

	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;
	*/
	return 0;
}
#endif

static int
i915_gem_userptr_get_pages(struct drm_i915_gem_object *obj) {
	struct i915_gem_userptr_object *vmap = to_userptr_object(obj);
	int num_pages = obj->base.size >> PAGE_SHIFT;
	struct sg_table *st;
	struct scatterlist *sg;
	struct page **pvec;
	int n, pinned, ret;

	if (vmap->mm == NULL)
		return -EFAULT;

	if (!access_ok(vmap->read_only ? VERIFY_READ : VERIFY_WRITE,
		       (char __user *)vmap->user_ptr, vmap->user_size))
		return -EFAULT;

	/* If userspace should engineer that these pages are replaced in
	 * the vma between us binding this page into the GTT and completion
	 * of rendering... Their loss. If they change the mapping of their
	 * pages they need to create a new bo to point to the new vma.
	 *
	 * However, that still leaves open the possibility of the vma
	 * being copied upon fork. Which falls under the same userspace
	 * synchronisation issue as a regular bo, except that this time
	 * the process may not be expecting that a particular piece of
	 * memory is tied to the GPU.
	 *
	 * Fortunately, we can hook into the mmu_notifier in order to
	 * discard the page references prior to anything nasty happening
	 * to the vma (discard or cloning) which should prevent the more
	 * egregious cases from causing harm.
	 */

	pvec = kmalloc(num_pages*sizeof(struct page *),
		       GFP_KERNEL | __GFP_NOWARN | __GFP_NORETRY);
	if (pvec == NULL) {
		pvec = drm_malloc_ab(num_pages, sizeof(struct page *));
		if (pvec == NULL)
			return -ENOMEM;
	}

	pinned = 0;
	if (vmap->mm == current->mm)
		pinned = __get_user_pages_fast(vmap->user_ptr, num_pages,
					       !vmap->read_only, pvec);
	if (pinned < num_pages) {
		struct mm_struct *mm = vmap->mm;
		ret = 0;
		mutex_unlock(&obj->base.dev->struct_mutex);
		down_read(&mm->mmap_sem);
		if (vmap->mm != NULL)
			ret = get_user_pages(current, mm,
					vmap->user_ptr + (pinned << PAGE_SHIFT),
					num_pages - pinned,
					!vmap->read_only, 0,
					pvec + pinned,
					NULL);
		up_read(&mm->mmap_sem);
		mutex_lock(&obj->base.dev->struct_mutex);
		if (ret > 0)
			pinned += ret;

		if (obj->pages || pinned < num_pages) {
			ret = obj->pages ? 0 : -EFAULT;
			goto cleanup_pinned;
		}
	}

	st = kmalloc(sizeof(*st), GFP_KERNEL);
	if (st == NULL) {
		ret = -ENOMEM;
		goto cleanup_pinned;
	}

	if (sg_alloc_table(st, num_pages, GFP_KERNEL)) {
		ret = -ENOMEM;
		goto cleanup_st;
	}

	for_each_sg(st->sgl, sg, num_pages, n)
		sg_set_page(sg, pvec[n], PAGE_SIZE, 0);
	drm_free_large(pvec);

	obj->pages = st;
	return 0;

cleanup_st:
	kfree(st);
cleanup_pinned:
	release_pages(pvec, pinned, 0);
	drm_free_large(pvec);
	return ret;
}

static void
i915_gem_userptr_put_pages(struct drm_i915_gem_object *obj) {
	struct scatterlist *sg;
	int i;

	if (obj->madv != I915_MADV_WILLNEED)
		obj->dirty = 0;

	for_each_sg(obj->pages->sgl, sg, obj->pages->nents, i) {
		struct page *page = sg_page(sg);

		if (obj->dirty)
			set_page_dirty(page);

		mark_page_accessed(page);
		page_cache_release(page);
	}
	obj->dirty = 0;

	sg_free_table(obj->pages);
	kfree(obj->pages);
}

static void
i915_gem_userptr_release(struct drm_i915_gem_object *obj) {
	struct i915_gem_userptr_object *vmap = to_userptr_object(obj);

	i915_gem_userptr_release__mmu_notifier(vmap);
}

static bool
i915_gem_userptr_object_fn(void)
{
	return 1;
}

static const struct drm_i915_gem_object_ops i915_gem_userptr_ops = {
	.get_pages      = i915_gem_userptr_get_pages,
	.put_pages      = i915_gem_userptr_put_pages,
	.release        = i915_gem_userptr_release,
	.is_userptr_obj = i915_gem_userptr_object_fn,
};

/**
 * Creates a new mm object that wraps some user memory.
 */
int
i915_gem_userptr_ioctl(struct drm_device *dev,
				void *data, struct drm_file *file) {
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_i915_gem_userptr *args = data;
	struct i915_gem_userptr_object *obj;
	loff_t first_data_page, last_data_page;
	int num_pages;
	int ret;
	u32 handle;

	if (args->flags & ~(I915_USERPTR_READ_ONLY |
						I915_USERPTR_UNSYNCHRONIZED))
		return -EINVAL;

	if (!args->user_ptr)
		return -EINVAL;

	first_data_page = args->user_ptr / PAGE_SIZE;
	last_data_page = (args->user_ptr + args->user_size - 1) / PAGE_SIZE;
	num_pages = last_data_page - first_data_page + 1;
	if (num_pages * PAGE_SIZE > dev_priv->gtt.base.total)
		return -E2BIG;

	/* Allocate the new object */
	obj = i915_gem_object_alloc(dev);
	if (obj == NULL)
		return -ENOMEM;

	if (drm_gem_private_object_init(dev, &obj->gem.base,
					num_pages * PAGE_SIZE)) {
		i915_gem_object_free(&obj->gem);
		return -ENOMEM;
	}

	i915_gem_object_init(&obj->gem, &i915_gem_userptr_ops);

	/* To be checked, what is the optimal Cache policy for vmap objects */
	if (IS_VALLEYVIEW(dev))
		obj->gem.cache_level = I915_CACHE_LLC;
	else
		obj->gem.cache_level = I915_CACHE_L3_LLC;

	obj->user_ptr = args->user_ptr;
	obj->user_size = args->user_size;
	obj->read_only = args->flags & I915_USERPTR_READ_ONLY;

	/* And keep a pointer to the current->mm for resolving the user pages
	 * at binding. This means that we need to hook into the mmu_notifier
	 * in order to detect if the mmu is destroyed.
	 */
	obj->mm = current->mm;
	ret = i915_gem_userptr_init__mmu_notifier(obj, args->flags);
	if (ret) {
		drm_gem_object_release(&obj->gem.base);
		dev_priv->mm.object_count--;
		dev_priv->mm.object_memory -= obj->gem.base.size;
		i915_gem_object_free(&obj->gem);
		return ret;
	}

	ret = drm_gem_handle_create(file, &obj->gem.base, &handle);
	if (ret) {
		i915_gem_userptr_release((struct drm_i915_gem_object *)obj);
		drm_gem_object_release(&obj->gem.base);
		dev_priv->mm.object_count--;
		dev_priv->mm.object_memory -= obj->gem.base.size;
		i915_gem_object_free(&obj->gem);
		return ret;
	}

	/* drop reference from allocate - handle holds it now */
	drm_gem_object_unreference(&obj->gem.base);

	args->handle = handle;
	return 0;
}
