/*
 * Copyright © 2008-2012 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 * Authors:
 *    Eric Anholt <eric@anholt.net>
 *    Chris Wilson <chris@chris-wilson.co.uk>
 *
 */

#include <drm/drmP.h>
#include <drm/i915_drm.h>
#include "i915_drv.h"
#include <linux/shmem_fs.h>

/*
 * The BIOS typically reserves some of the system's memory for the exclusive
 * use of the integrated graphics. This memory is no longer available for
 * use by the OS and so the user finds that his system has less memory
 * available than he put in. We refer to this memory as stolen.
 *
 * The BIOS will allocate its framebuffer from the stolen memory. Our
 * goal is try to reuse that object for our own fbcon which must always
 * be available for panics. Anything else we can reuse the stolen memory
 * for is a boon.
 */

static unsigned long i915_stolen_to_physical(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct resource *r;
	u32 base;

	/* Almost universally we can find the Graphics Base of Stolen Memory
	 * at offset 0x5c in the igfx configuration space. On a few (desktop)
	 * machines this is also mirrored in the bridge device at different
	 * locations, or in the MCHBAR. On gen2, the layout is again slightly
	 * different with the Graphics Segment immediately following Top of
	 * Memory (or Top of Usable DRAM). Note it appears that TOUD is only
	 * reported by 865g, so we just use the top of memory as determined
	 * by the e820 probe.
	 *
	 * XXX However gen2 requires an unavailable symbol.
	 */
	base = 0;
	if (INTEL_INFO(dev)->gen >= 3) {
		/* Read Graphics Base of Stolen Memory directly */
		pci_read_config_dword(dev->pdev, 0x5c, &base);
		base &= ~((1<<20) - 1);
	} else { /* GEN2 */
#if 0
		/* Stolen is immediately above Top of Memory */
		base = max_low_pfn_mapped << PAGE_SHIFT;
#endif
	}

	if (base == 0)
		return 0;

	/* Verify that nothing else uses this physical address. Stolen
	 * memory should be reserved by the BIOS and hidden from the
	 * kernel. So if the region is already marked as busy, something
	 * is seriously wrong.
	 */
	/* requesting region from base + 1, to handle the memory
	 * region conflict with PCI Bus.
	 */
	r = devm_request_mem_region(dev->dev, base + 1,
				dev_priv->gtt.stolen_size - 1,
				    "Graphics Stolen Memory");
	if (r == NULL) {
		DRM_ERROR("conflict detected with stolen region: [0x%08x - 0x%08x]\n",
			  base, base + (uint32_t)dev_priv->gtt.stolen_size);
		base = 0;
	}

	DRM_DEBUG_DRIVER("stolen base = 0x%x, size = 0x%x\n",
		(unsigned int)base, (unsigned int)dev_priv->gtt.stolen_size);
	return base;
}

static int i915_setup_compression(struct drm_device *dev, int size)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_mm_node *compressed_fb, *uninitialized_var(compressed_llb);

	/* Try to over-allocate to reduce reallocations and fragmentation */
	compressed_fb = drm_mm_search_free(&dev_priv->mm.stolen,
					   size <<= 1, 4096, 0);
	if (!compressed_fb)
		compressed_fb = drm_mm_search_free(&dev_priv->mm.stolen,
						   size >>= 1, 4096, 0);
	if (compressed_fb)
		compressed_fb = drm_mm_get_block(compressed_fb, size, 4096);
	if (!compressed_fb)
		goto err;

	if (HAS_PCH_SPLIT(dev))
		I915_WRITE(ILK_DPFC_CB_BASE, compressed_fb->start);
	else if (IS_GM45(dev)) {
		I915_WRITE(DPFC_CB_BASE, compressed_fb->start);
	} else {
		compressed_llb = drm_mm_search_free(&dev_priv->mm.stolen,
						    4096, 4096, 0);
		if (compressed_llb)
			compressed_llb = drm_mm_get_block(compressed_llb,
							  4096, 4096);
		if (!compressed_llb)
			goto err_fb;

		dev_priv->fbc.compressed_llb = compressed_llb;

		I915_WRITE(FBC_CFB_BASE,
			   dev_priv->mm.stolen_base + compressed_fb->start);
		I915_WRITE(FBC_LL_BASE,
			   dev_priv->mm.stolen_base + compressed_llb->start);
	}

	dev_priv->fbc.compressed_fb = compressed_fb;
	dev_priv->fbc.size = size;

	DRM_DEBUG_KMS("reserved %d bytes of contiguous stolen space for FBC\n",
		      size);

	return 0;

err_fb:
	drm_mm_put_block(compressed_fb);
err:
	pr_info_once("drm: not enough stolen space for compressed buffer (need %d more bytes), disabling. Hint: you may be able to increase stolen memory size in the BIOS to avoid this.\n", size);
	return -ENOSPC;
}

int i915_gem_stolen_setup_compression(struct drm_device *dev, int size)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (!drm_mm_initialized(&dev_priv->mm.stolen))
		return -ENODEV;

	if (size < dev_priv->fbc.size)
		return 0;

	/* Release any current block */
	i915_gem_stolen_cleanup_compression(dev);

	return i915_setup_compression(dev, size);
}

void i915_gem_stolen_cleanup_compression(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (dev_priv->fbc.size == 0)
		return;

	if (dev_priv->fbc.compressed_fb)
		drm_mm_put_block(dev_priv->fbc.compressed_fb);

	if (dev_priv->fbc.compressed_llb)
		drm_mm_put_block(dev_priv->fbc.compressed_llb);

	dev_priv->fbc.size = 0;
}

void i915_gem_cleanup_stolen(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (!drm_mm_initialized(&dev_priv->mm.stolen))
		return;

	if (dev_priv->vlv_pctx) {
		mutex_lock(&dev->struct_mutex);
		drm_gem_object_unreference(&dev_priv->vlv_pctx->base);
		mutex_unlock(&dev->struct_mutex);
		dev_priv->vlv_pctx = NULL;
	}

	i915_gem_stolen_cleanup_compression(dev);
	drm_mm_takedown(&dev_priv->mm.stolen);
}

int i915_gem_init_stolen(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int bios_reserved = 0;

	dev_priv->mm.stolen_base = i915_stolen_to_physical(dev);
	if (dev_priv->mm.stolen_base == 0)
		return 0;

	DRM_DEBUG_DRIVER("found %zd bytes of stolen memory at %08lx\n",
		      dev_priv->gtt.stolen_size, dev_priv->mm.stolen_base);

	if (IS_VALLEYVIEW(dev))
		bios_reserved = 1024*1024; /* top 1M on VLV/BYT */

	if (WARN_ON(bios_reserved > dev_priv->gtt.stolen_size))
		return 0;

	/* Basic memrange allocator for stolen space */
	drm_mm_init(&dev_priv->mm.stolen, 0, dev_priv->gtt.stolen_size -
		    bios_reserved);

	return 0;
}

static struct sg_table *
i915_pages_create_for_stolen(struct drm_device *dev,
			     u32 offset, u32 size)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct sg_table *st;
	struct scatterlist *sg;

	DRM_DEBUG_DRIVER("offset=0x%x, size=%d\n", offset, size);
	BUG_ON(offset > dev_priv->gtt.stolen_size - size);

	/* We hide that we have no struct page backing our stolen object
	 * by wrapping the contiguous physical allocation with a fake
	 * dma mapping in a single scatterlist.
	 */

	st = kmalloc(sizeof(*st), GFP_KERNEL);
	if (st == NULL)
		return NULL;

	if (sg_alloc_table(st, 1, GFP_KERNEL)) {
		kfree(st);
		return NULL;
	}

	sg = st->sgl;
	sg->offset = 0;
	sg->length = size;

	sg_dma_address(sg) = (dma_addr_t)dev_priv->mm.stolen_base + offset;
	sg_dma_len(sg) = size;

	return st;
}

static int i915_gem_object_get_pages_stolen(struct drm_i915_gem_object *obj)
{
	BUG();
	return -EINVAL;
}

static void i915_gem_object_put_pages_stolen(struct drm_i915_gem_object *obj)
{
	/* Should only be called during free */
	DRM_DEBUG_DRIVER(
		"Obj removed from stolen, ptr=%p, refcnt=%d, handle_cnt=%d\n",
			obj,
			obj->base.refcount.refcount.counter,
			obj->base.handle_count.counter);

	sg_free_table(obj->pages);
	kfree(obj->pages);
}

static const struct drm_i915_gem_object_ops i915_gem_object_stolen_ops = {
	.get_pages = i915_gem_object_get_pages_stolen,
	.put_pages = i915_gem_object_put_pages_stolen,
};

static struct drm_i915_gem_object *
_i915_gem_object_create_stolen(struct drm_device *dev,
			       struct drm_mm_node *stolen)
{
	struct drm_i915_gem_object *obj;

	obj = i915_gem_object_alloc(dev);
	if (obj == NULL)
		return NULL;

	if (drm_gem_private_object_init(dev, &obj->base, stolen->size))
		goto cleanup;

	i915_gem_object_init(obj, &i915_gem_object_stolen_ops);

	obj->pages = i915_pages_create_for_stolen(dev,
						  stolen->start, stolen->size);
	if (obj->pages == NULL)
		goto cleanup;

	obj->has_dma_mapping = true;
	i915_gem_object_pin_pages(obj);
	obj->stolen = stolen;

	obj->base.read_domains = I915_GEM_DOMAIN_CPU | I915_GEM_DOMAIN_GTT;
	obj->cache_level = HAS_LLC(dev) ? I915_CACHE_LLC : I915_CACHE_NONE;

	DRM_DEBUG_DRIVER("New obj allocated from stolen %p, size = 0x%zx\n",
			 obj, obj->base.size);

	return obj;

cleanup:
	i915_gem_object_free(obj);
	return NULL;
}

struct drm_i915_gem_object *
i915_gem_object_create_stolen(struct drm_device *dev, u32 size)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_i915_gem_object *obj;
	struct drm_mm_node *stolen;

	if (!drm_mm_initialized(&dev_priv->mm.stolen))
		return NULL;

	DRM_DEBUG_KMS("creating stolen object: size=%x\n", size);
	if (size == 0)
		return NULL;

	stolen = drm_mm_search_free(&dev_priv->mm.stolen, size, 4096, 0);
	if (stolen)
		stolen = drm_mm_get_block(stolen, size, 4096);
	if (stolen == NULL)
		return NULL;

	obj = _i915_gem_object_create_stolen(dev, stolen);
	if (obj)
		return obj;

	drm_mm_put_block(stolen);
	return NULL;
}

static int i915_add_clear_obj_cmd(struct drm_i915_gem_object *obj)
{
	struct drm_i915_private *dev_priv = obj->base.dev->dev_private;
	struct intel_ring_buffer *ring = &dev_priv->ring[BCS];
	uint32_t obj_height = (obj->base.size / obj->stride);
	u32 offset = i915_gem_obj_ggtt_offset(obj);
	int ret;

#define	COLOR_BLIT_OP		((0x2 << 29) | (0x40 << 22) | (0x3 << 20) | 0x3)
#define	COLOR_BLIT_OP_DW1	((0x3 << 24) | (0xF0 << 16) | (obj->stride))
#define	COLOR_BLIT_OP_DW2	((obj_height << 16) | (obj->stride))
#define	COLOR_BLIT_OP_DW3	(offset)
#define	COLOR_BLIT_OP_DW4	(0x0)

	ret = intel_ring_begin(ring, 6);
	if (ret)
		return ret;
	intel_ring_emit(ring, COLOR_BLIT_OP);
	intel_ring_emit(ring, COLOR_BLIT_OP_DW1);
	intel_ring_emit(ring, COLOR_BLIT_OP_DW2);
	intel_ring_emit(ring, COLOR_BLIT_OP_DW3);
	intel_ring_emit(ring, COLOR_BLIT_OP_DW4);
	intel_ring_emit(ring, MI_NOOP);
	intel_ring_advance(ring);

	return 0;
}

static int i915_memset_stolen_obj_hw(struct drm_i915_gem_object *obj)
{
	struct drm_i915_private *dev_priv = obj->base.dev->dev_private;
	struct intel_ring_buffer *ring = &dev_priv->ring[BCS];
	unsigned alignment = 0;
	bool map_and_fenceable =  true;
	bool nonblocking = false;
	u32 seqno;
	int ret;

	ret = i915_gem_obj_ggtt_pin(obj, alignment, map_and_fenceable,
		nonblocking);
	if (ret) {
		DRM_ERROR("Mapping of User FB to GTT failed\n");
		return ret;
	}

	/* Adding commands to the blitter ring to
	 * clear out the contents of the buffer object
	 */
	ret = i915_add_clear_obj_cmd(obj);
	if (ret) {
		DRM_ERROR("couldn't add commands in blitter ring\n");
		i915_gem_object_unpin(obj);
		return ret;
	}

	seqno = intel_ring_get_seqno(ring);

	obj->base.read_domains = I915_GEM_DOMAIN_RENDER;
	obj->base.write_domain = I915_GEM_DOMAIN_RENDER;

	i915_gem_object_move_to_active(obj, ring);

	obj->dirty = 1;
	obj->last_write_seqno = seqno;

	/* Unconditionally force add_request to emit a full flush. */
	ring->gpu_caches_dirty = true;

	/* Add a breadcrumb for the completion of the batch buffer */
	(void)i915_add_request(ring, NULL);

	i915_gem_object_unpin(obj);

	return 0;
}

static void i915_memset_stolen_obj_sw(struct drm_i915_gem_object *obj)
{
	if (!i915_gem_obj_ggtt_bound(obj)) {
		int ret;
		char __iomem *base;
		int size = obj->base.size;
		struct drm_i915_private *dev_priv = obj->base.dev->dev_private;
		unsigned alignment = 0;
		bool map_and_fenceable =  true;
		bool nonblocking = false;

		ret = i915_gem_obj_ggtt_pin(obj, alignment, map_and_fenceable,
					nonblocking);
		if (ret) {
			DRM_ERROR("Mapping of User FB to GTT failed\n");
			return;
		}

		/* Get the CPU virtual address of the frame buffer */
		base =
			ioremap_wc(dev_priv->gtt.mappable_base +
				i915_gem_obj_ggtt_offset(obj), size);
		if (base == NULL) {
			DRM_ERROR("Mapping of User FB to CPU failed\n");
			i915_gem_object_unpin(obj);
			return;
		}

		memset_io(base, 0, size);

		iounmap(base);
		i915_gem_object_unpin(obj);

		DRM_DEBUG_DRIVER(
		  "User FB obj ptr=%p cleared using CPU virtual address %p\n",
		  obj, base);
	} else
		BUG_ON(1);
}

void
i915_gem_object_move_to_stolen(struct drm_i915_gem_object *obj)
{
	struct drm_device *dev = obj->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_mm_node *stolen;
	u32 size = obj->base.size;
	int ret = 0;

	if (obj->stolen) {
		BUG_ON(obj->pages == NULL);
		return;
	}

	if (dev_priv->mm.stolen_base == 0)
		return;

	if (size == 0)
		return;

	/* The alloced field stores how many data pages are allocated
	 * to the file. If already shmem space has been allocated for
	 * the frame buffer, then we shall not use the stolen area as
	 * a backing store for the object */
	if (obj->base.filp) {
		struct inode *inode = file_inode(obj->base.filp);
		struct shmem_inode_info *info = SHMEM_I(inode);
		if (!inode)
			return;
		spin_lock(&info->lock);
		ret = info->alloced;
		spin_unlock(&info->lock);
		if (ret > 0) {
			DRM_DEBUG_DRIVER(
				"Already shmem space alloced, %d pges\n", ret);
			return;
		}
	}

	ret = i915_mutex_lock_interruptible(dev);
	if (ret) {
		DRM_ERROR("Mutex lock failed\n");
		return;
	}

	stolen = drm_mm_search_free(&dev_priv->mm.stolen, size, 4096, 0);
	if (stolen)
		stolen = drm_mm_get_block(stolen, size, 4096);

	if (stolen == NULL) {
		DRM_DEBUG_DRIVER("ran out of stolen space on obj=%p,size=%x\n",
				 obj, size);
		goto out;
	}
	/* Set up the object to use the stolen memory,
	 * backing store no longer managed by shmem layer */
	drm_gem_object_release(&(obj->base));
	obj->base.filp = NULL;
	obj->ops = &i915_gem_object_stolen_ops;

	obj->pages = i915_pages_create_for_stolen(dev,
						stolen->start, stolen->size);
	if (obj->pages == NULL) {
		drm_mm_put_block(stolen);
		goto out;
	}
	i915_gem_object_pin_pages(obj);
	list_add_tail(&obj->global_list, &dev_priv->mm.unbound_list);
	obj->has_dma_mapping = true;
	obj->stolen = stolen;

	DRM_DEBUG_DRIVER("Obj moved to stolen, ptr = %p, size = %x\n",
			 obj, size);

	obj->base.read_domains = I915_GEM_DOMAIN_CPU | I915_GEM_DOMAIN_GTT;
	obj->cache_level = HAS_LLC(dev) ? I915_CACHE_LLC : I915_CACHE_NONE;

	ret = i915_memset_stolen_obj_hw(obj);
	if (ret)
		i915_memset_stolen_obj_sw(obj);

out:
	mutex_unlock(&dev->struct_mutex);
	return;
}


struct drm_i915_gem_object *
i915_gem_object_create_stolen_for_preallocated(struct drm_device *dev,
					       u32 stolen_offset,
					       u32 gtt_offset,
					       u32 size)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct i915_address_space *ggtt = &dev_priv->gtt.base;
	struct drm_i915_gem_object *obj;
	struct drm_mm_node *stolen;
	struct i915_vma *vma;
	int ret;

	if (!drm_mm_initialized(&dev_priv->mm.stolen))
		return NULL;

	DRM_DEBUG_DRIVER(
		"creating preallocated stolen obj:stolen_off=%x,gtt_off=%x,size=%x\n",
			stolen_offset, gtt_offset, size);

	/* KISS and expect everything to be page-aligned */
	BUG_ON(stolen_offset & 4095);
	BUG_ON(size & 4095);

	if (WARN_ON(size == 0))
		return NULL;

	stolen = kzalloc(sizeof(*stolen), GFP_KERNEL);
	if (!stolen)
		return NULL;

	stolen->start = stolen_offset;
	stolen->size = size;
	ret = drm_mm_reserve_node(&dev_priv->mm.stolen, stolen);
	if (ret) {
		DRM_DEBUG_KMS("failed to allocate stolen space\n");
		kfree(stolen);
		return NULL;
	}

	obj = _i915_gem_object_create_stolen(dev, stolen);
	if (obj == NULL) {
		DRM_DEBUG_KMS("failed to allocate stolen object\n");
		drm_mm_put_block(stolen);
		return NULL;
	}

	/* Some objects just need physical mem from stolen space */
	if (gtt_offset == I915_GTT_OFFSET_NONE)
		return obj;

	vma = i915_gem_vma_create(obj, ggtt);
	if (IS_ERR(vma)) {
		ret = PTR_ERR(vma);
		goto err_out;
	}

	/* To simplify the initialisation sequence between KMS and GTT,
	 * we allow construction of the stolen object prior to
	 * setting up the GTT space. The actual reservation will occur
	 * later.
	 */
	vma->node.start = gtt_offset;
	vma->node.size = size;
	if (drm_mm_initialized(&ggtt->mm)) {
		ret = drm_mm_reserve_node(&ggtt->mm, &vma->node);
		if (ret) {
			DRM_DEBUG_KMS("failed to allocate stolen GTT space\n");
			goto err_vma;
		}
	}

	obj->has_global_gtt_mapping = 1;

	list_add_tail(&obj->global_list, &dev_priv->mm.bound_list);
	list_add_tail(&vma->mm_list, &ggtt->inactive_list);

	return obj;

err_vma:
	i915_gem_vma_destroy(vma);
err_out:
	drm_mm_put_block(stolen);
	drm_gem_object_unreference(&obj->base);
	return NULL;
}

void
i915_gem_object_release_stolen(struct drm_i915_gem_object *obj)
{
	if (obj->stolen) {
		drm_mm_put_block(obj->stolen);
		obj->stolen = NULL;
	}
}
