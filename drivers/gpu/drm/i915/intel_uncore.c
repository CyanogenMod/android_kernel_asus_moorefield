/*
 * Copyright © 2013 Intel Corporation
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
 */

#include "i915_drv.h"
#include "intel_drv.h"

#define FORCEWAKE_ACK_TIMEOUT_MS 2

#define __raw_i915_read8(dev_priv__, reg__) readb((dev_priv__)->regs + (reg__))
#define __raw_i915_write8(dev_priv__, reg__, val__) writeb(val__, (dev_priv__)->regs + (reg__))

#define __raw_i915_read16(dev_priv__, reg__) readw((dev_priv__)->regs + (reg__))
#define __raw_i915_write16(dev_priv__, reg__, val__) writew(val__, (dev_priv__)->regs + (reg__))

#define __raw_i915_read32(dev_priv__, reg__) readl((dev_priv__)->regs + (reg__))
#define __raw_i915_write32(dev_priv__, reg__, val__) writel(val__, (dev_priv__)->regs + (reg__))

#define __raw_i915_read64(dev_priv__, reg__) readq((dev_priv__)->regs + (reg__))
#define __raw_i915_write64(dev_priv__, reg__, val__) writeq(val__, (dev_priv__)->regs + (reg__))

#define __raw_posting_read(dev_priv__, reg__) (void)__raw_i915_read32(dev_priv__, reg__)


static void __gen6_gt_wait_for_thread_c0(struct drm_i915_private *dev_priv)
{
	u32 gt_thread_status_mask;

	if (IS_HASWELL(dev_priv->dev))
		gt_thread_status_mask = GEN6_GT_THREAD_STATUS_CORE_MASK_HSW;
	else
		gt_thread_status_mask = GEN6_GT_THREAD_STATUS_CORE_MASK;

	/* w/a for a sporadic read returning 0 by waiting for the GT
	 * thread to wake up.
	 */
	if (wait_for_atomic_us((__raw_i915_read32(dev_priv, GEN6_GT_THREAD_STATUS_REG) & gt_thread_status_mask) == 0, 500))
		DRM_ERROR("GT thread status wait timed out\n");
}

static void __gen6_gt_force_wake_reset(struct drm_i915_private *dev_priv)
{
	__raw_i915_write32(dev_priv, FORCEWAKE, 0);
	/* something from same cacheline, but !FORCEWAKE */
	__raw_posting_read(dev_priv, ECOBUS);
}

static void __gen6_gt_force_wake_get(struct drm_i915_private *dev_priv,
								int fw_engine)
{
	if (wait_for_atomic((__raw_i915_read32(dev_priv, FORCEWAKE_ACK) & 1) == 0,
			    FORCEWAKE_ACK_TIMEOUT_MS))
		DRM_ERROR("Timed out waiting for forcewake old ack to clear.\n");

	__raw_i915_write32(dev_priv, FORCEWAKE, 1);
	/* something from same cacheline, but !FORCEWAKE */
	__raw_posting_read(dev_priv, ECOBUS);

	if (wait_for_atomic((__raw_i915_read32(dev_priv, FORCEWAKE_ACK) & 1),
			    FORCEWAKE_ACK_TIMEOUT_MS))
		DRM_ERROR("Timed out waiting for forcewake to ack request.\n");

	/* WaRsForcewakeWaitTC0:snb */
	__gen6_gt_wait_for_thread_c0(dev_priv);
}

void gen6_gt_force_wake_restore(struct drm_i915_private *dev_priv)
{
	/* Restore the current expected force wake state with the
	* hardware. This may be required following a reset.
	*
	* WARNING: Caller *MUST* hold uncore.lock whilst calling this.
	*
	* uncore.lock isn't taken in this function to allow the caller the
	* flexibility to do other work immediately before/after
	* whilst holding the lock*/

	if (IS_VALLEYVIEW(dev_priv->dev))
		return vlv_force_wake_restore(dev_priv, FORCEWAKE_ALL);

	if (dev_priv->uncore.forcewake_count) {
		/* It was enabled, so re-enable it */
		dev_priv->uncore.funcs.force_wake_get(dev_priv, FORCEWAKE_ALL);
	} else {
		/* It was disabled, so disable it */
		dev_priv->uncore.funcs.force_wake_put(dev_priv, FORCEWAKE_ALL);
	}

	dev_priv->uncore.fifo_count =
		__raw_i915_read32(dev_priv, GTFIFOCTL) &
						GT_FIFO_FREE_ENTRIES_MASK;
}

static void __gen6_gt_force_wake_mt_reset(struct drm_i915_private *dev_priv)
{
	__raw_i915_write32(dev_priv, FORCEWAKE_MT, _MASKED_BIT_DISABLE(0xffff));
	/* something from same cacheline, but !FORCEWAKE_MT */
	__raw_posting_read(dev_priv, ECOBUS);
}

static void __gen6_gt_force_wake_mt_get(struct drm_i915_private *dev_priv,
								int fw_engine)
{
	u32 forcewake_ack;

	if (IS_HASWELL(dev_priv->dev))
		forcewake_ack = FORCEWAKE_ACK_HSW;
	else
		forcewake_ack = FORCEWAKE_MT_ACK;

	if (wait_for_atomic((__raw_i915_read32(dev_priv, forcewake_ack) & FORCEWAKE_KERNEL) == 0,
			    FORCEWAKE_ACK_TIMEOUT_MS))
		DRM_ERROR("Timed out waiting for forcewake old ack to clear.\n");

	__raw_i915_write32(dev_priv, FORCEWAKE_MT,
			   _MASKED_BIT_ENABLE(FORCEWAKE_KERNEL));
	/* something from same cacheline, but !FORCEWAKE_MT */
	__raw_posting_read(dev_priv, ECOBUS);

	if (wait_for_atomic((__raw_i915_read32(dev_priv, forcewake_ack) & FORCEWAKE_KERNEL),
			    FORCEWAKE_ACK_TIMEOUT_MS))
		DRM_ERROR("Timed out waiting for forcewake to ack request.\n");

	/* WaRsForcewakeWaitTC0:ivb,hsw */
	__gen6_gt_wait_for_thread_c0(dev_priv);
}

static void gen6_gt_check_fifodbg(struct drm_i915_private *dev_priv)
{
	u32 gtfifodbg;

	gtfifodbg = __raw_i915_read32(dev_priv, GTFIFODBG);
	if (WARN(gtfifodbg & GT_FIFO_CPU_ERROR_MASK,
	     "MMIO read or write has been dropped %x\n", gtfifodbg))
		__raw_i915_write32(dev_priv, GTFIFODBG, GT_FIFO_CPU_ERROR_MASK);
}

static void __gen6_gt_force_wake_put(struct drm_i915_private *dev_priv,
								int fw_engine)
{
	__raw_i915_write32(dev_priv, FORCEWAKE, 0);
	/* something from same cacheline, but !FORCEWAKE */
	__raw_posting_read(dev_priv, ECOBUS);
	gen6_gt_check_fifodbg(dev_priv);
}

static void __gen6_gt_force_wake_mt_put(struct drm_i915_private *dev_priv,
								int fw_engine)
{
	__raw_i915_write32(dev_priv, FORCEWAKE_MT,
			   _MASKED_BIT_DISABLE(FORCEWAKE_KERNEL));
	/* something from same cacheline, but !FORCEWAKE_MT */
	__raw_posting_read(dev_priv, ECOBUS);
	gen6_gt_check_fifodbg(dev_priv);
}

static int __gen6_gt_wait_for_fifo(struct drm_i915_private *dev_priv)
{
	int ret = 0;

	/* On VLV, FIFO will be shared by both SW and HW.
	So, we need to read the	FREE_ENTRIES everytime */
	if (IS_VALLEYVIEW(dev_priv->dev))
		dev_priv->uncore.fifo_count =
			__raw_i915_read32(dev_priv, GTFIFOCTL) &
						GT_FIFO_FREE_ENTRIES_MASK;
	if (dev_priv->uncore.fifo_count < GT_FIFO_NUM_RESERVED_ENTRIES) {
		int loop = 500;
		u32 fifo = __raw_i915_read32(dev_priv, GTFIFOCTL) &
						GT_FIFO_FREE_ENTRIES_MASK;
		while (fifo <= GT_FIFO_NUM_RESERVED_ENTRIES && loop--) {
			udelay(10);
			fifo = __raw_i915_read32(dev_priv, GTFIFOCTL) &
						GT_FIFO_FREE_ENTRIES_MASK;
		}
#ifdef CONFIG_DEBUG_FS
		if (loop < 500)
			atomic_inc(&dev_priv->wfifo_count);
#endif
		if (WARN_ON(loop < 0 && fifo <= GT_FIFO_NUM_RESERVED_ENTRIES))
			++ret;
		dev_priv->uncore.fifo_count = fifo;
	}
	dev_priv->uncore.fifo_count--;

	return ret;
}

static void __vlv_force_wake_get(struct drm_i915_private *dev_priv,
				int fw_engine)
{
	/* Check for Render Engine */
	if (FORCEWAKE_RENDER & fw_engine) {
		__raw_i915_write32(dev_priv, VLV_RENDER_FORCE_WAKE_REG,
					_MASKED_BIT_ENABLE(FORCEWAKE_KERNEL));

		if (wait_for_atomic_us((__raw_i915_read32(dev_priv,
			VLV_RENDER_FORCE_WAKE_STATUS_REG) & FORCEWAKE_KERNEL),
								500))
			DRM_ERROR("Render force wake get wait timed out\n");
	}

	/* Check for Media Engine */
	if (FORCEWAKE_MEDIA & fw_engine) {
		__raw_i915_write32(dev_priv, VLV_MEDIA_FORCE_WAKE_REG,
					_MASKED_BIT_ENABLE(FORCEWAKE_KERNEL));

		if (wait_for_atomic_us((__raw_i915_read32(dev_priv,
			VLV_MEDIA_FORCE_WAKE_STATUS_REG) & FORCEWAKE_KERNEL),
								500))
			DRM_ERROR("Media Force wake get wait timed out\n");
	}

	/* WaRsForcewakeWaitTC0:vlv */
	__gen6_gt_wait_for_thread_c0(dev_priv);
}

static void __vlv_force_wake_put(struct drm_i915_private *dev_priv,
					int fw_engine)
{

	/* Check for Render Engine */
	if (FORCEWAKE_RENDER & fw_engine) {
		__raw_i915_write32(dev_priv, VLV_RENDER_FORCE_WAKE_REG,
					_MASKED_BIT_DISABLE(FORCEWAKE_KERNEL));

		__raw_posting_read(dev_priv, VLV_RENDER_FORCE_WAKE_REG);
	}

	/* Check for Media Engine */
	if (FORCEWAKE_MEDIA & fw_engine) {
		__raw_i915_write32(dev_priv, VLV_MEDIA_FORCE_WAKE_REG,
				_MASKED_BIT_DISABLE(FORCEWAKE_KERNEL));

		__raw_posting_read(dev_priv, VLV_MEDIA_FORCE_WAKE_REG);
	}

	/* The below doubles as a POSTING_READ */
	gen6_gt_check_fifodbg(dev_priv);

}

void vlv_force_wake_get(struct drm_i915_private *dev_priv,
							int fw_engine)
{
	unsigned long irqflags;

	spin_lock_irqsave(&dev_priv->uncore.lock, irqflags);
	if (FORCEWAKE_RENDER & fw_engine) {
		if (dev_priv->uncore.fw_rendercount++ == 0)
			dev_priv->uncore.funcs.force_wake_get(dev_priv,
							FORCEWAKE_RENDER);
	}
	if (FORCEWAKE_MEDIA & fw_engine) {
		if (dev_priv->uncore.fw_mediacount++ == 0)
			dev_priv->uncore.funcs.force_wake_get(dev_priv,
							FORCEWAKE_MEDIA);
	}

	spin_unlock_irqrestore(&dev_priv->uncore.lock, irqflags);
}

void vlv_force_wake_put(struct drm_i915_private *dev_priv,
							int fw_engine)
{
	unsigned long irqflags;

	spin_lock_irqsave(&dev_priv->uncore.lock, irqflags);

	if (FORCEWAKE_RENDER & fw_engine) {
		WARN_ON(dev_priv->uncore.fw_rendercount == 0);
		if (--dev_priv->uncore.fw_rendercount == 0)
			dev_priv->uncore.funcs.force_wake_put(dev_priv,
							FORCEWAKE_RENDER);
	}

	if (FORCEWAKE_MEDIA & fw_engine) {
		WARN_ON(dev_priv->uncore.fw_mediacount == 0);
		if (--dev_priv->uncore.fw_mediacount == 0)
			dev_priv->uncore.funcs.force_wake_put(dev_priv,
							FORCEWAKE_MEDIA);
	}

	spin_unlock_irqrestore(&dev_priv->uncore.lock, irqflags);
}

void vlv_force_wake_restore(struct drm_i915_private *dev_priv,
				int fw_engine)
{
	/* Restore the current force wake state with the hardware
	*  WARNING: Caller *MUST* hold uncore.lock whilst calling this function
	*/

	if (FORCEWAKE_RENDER & fw_engine) {
		if (dev_priv->uncore.fw_rendercount)
			dev_priv->uncore.funcs.force_wake_get(dev_priv,
				FORCEWAKE_RENDER);
		else
			dev_priv->uncore.funcs.force_wake_put(dev_priv,
				FORCEWAKE_RENDER);
	}

	if (FORCEWAKE_MEDIA & fw_engine) {
		if (dev_priv->uncore.fw_mediacount)
			dev_priv->uncore.funcs.force_wake_get(dev_priv,
				FORCEWAKE_MEDIA);
		else
			dev_priv->uncore.funcs.force_wake_put(dev_priv,
				FORCEWAKE_MEDIA);
	}

	dev_priv->uncore.fifo_count =
		__raw_i915_read32(dev_priv, GTFIFOCTL) &
						GT_FIFO_FREE_ENTRIES_MASK;
}


void intel_uncore_early_sanitize(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (HAS_FPGA_DBG_UNCLAIMED(dev))
		__raw_i915_write32(dev_priv, FPGA_DBG, FPGA_DBG_RM_NOCLAIM);
}

void intel_uncore_init(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (IS_VALLEYVIEW(dev)) {
		dev_priv->uncore.funcs.force_wake_get = __vlv_force_wake_get;
		dev_priv->uncore.funcs.force_wake_put = __vlv_force_wake_put;
	} else if (IS_HASWELL(dev)) {
		dev_priv->uncore.funcs.force_wake_get = __gen6_gt_force_wake_mt_get;
		dev_priv->uncore.funcs.force_wake_put = __gen6_gt_force_wake_mt_put;
	} else if (IS_IVYBRIDGE(dev)) {
		u32 ecobus;

		/* IVB configs may use multi-threaded forcewake */

		/* A small trick here - if the bios hasn't configured
		 * MT forcewake, and if the device is in RC6, then
		 * force_wake_mt_get will not wake the device and the
		 * ECOBUS read will return zero. Which will be
		 * (correctly) interpreted by the test below as MT
		 * forcewake being disabled.
		 */
		mutex_lock(&dev->struct_mutex);
		__gen6_gt_force_wake_mt_get(dev_priv, FORCEWAKE_ALL);
		ecobus = __raw_i915_read32(dev_priv, ECOBUS);
		__gen6_gt_force_wake_mt_put(dev_priv, FORCEWAKE_ALL);
		mutex_unlock(&dev->struct_mutex);

		if (ecobus & FORCEWAKE_MT_ENABLE) {
			dev_priv->uncore.funcs.force_wake_get =
				__gen6_gt_force_wake_mt_get;
			dev_priv->uncore.funcs.force_wake_put =
				__gen6_gt_force_wake_mt_put;
		} else {
			DRM_INFO("No MT forcewake available on Ivybridge, this can result in issues\n");
			DRM_INFO("when using vblank-synced partial screen updates.\n");
			dev_priv->uncore.funcs.force_wake_get =
				__gen6_gt_force_wake_get;
			dev_priv->uncore.funcs.force_wake_put =
				__gen6_gt_force_wake_put;
		}
	} else if (IS_GEN6(dev)) {
		dev_priv->uncore.funcs.force_wake_get =
			__gen6_gt_force_wake_get;
		dev_priv->uncore.funcs.force_wake_put =
			__gen6_gt_force_wake_put;
	}
}

void intel_uncore_sanitize(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (IS_VALLEYVIEW(dev)) {
		/* RS state has to be initialized to pull render/media power
		* wells out of sleep. This is required before initializing gem,
		* which touches render/media registers
		*/
		vlv_rs_sleepstateinit(dev, true);
		return;
	} else if (INTEL_INFO(dev)->gen >= 6) {
		__gen6_gt_force_wake_reset(dev_priv);
		if (IS_IVYBRIDGE(dev) || IS_HASWELL(dev))
			__gen6_gt_force_wake_mt_reset(dev_priv);
	}

	/* BIOS often leaves RC6 enabled, but disable it for hw init */
	intel_disable_gt_powersave(dev);
}

/*
 * Generally this is called implicitly by the register read function. However,
 * if some sequence requires the GT to not power down then this function should
 * be called at the beginning of the sequence followed by a call to
 * gen6_gt_force_wake_put() at the end of the sequence.
 */
void gen6_gt_force_wake_get(struct drm_i915_private *dev_priv,
							int fw_engine)
{
	unsigned long irqflags;

	/*Redirect to VLV specific routine*/
	if (IS_VALLEYVIEW(dev_priv->dev))
		return vlv_force_wake_get(dev_priv, fw_engine);

	spin_lock_irqsave(&dev_priv->uncore.lock, irqflags);
	if (dev_priv->uncore.forcewake_count++ == 0)
		dev_priv->uncore.funcs.force_wake_get(dev_priv, fw_engine);
	spin_unlock_irqrestore(&dev_priv->uncore.lock, irqflags);
}

/*
 * see gen6_gt_force_wake_get()
 */
void gen6_gt_force_wake_put(struct drm_i915_private *dev_priv,
							int fw_engine)
{
	unsigned long irqflags;

	/*Redirect to VLV specific routine*/
	if (IS_VALLEYVIEW(dev_priv->dev))
		return vlv_force_wake_put(dev_priv, fw_engine);

	spin_lock_irqsave(&dev_priv->uncore.lock, irqflags);
	if (--dev_priv->uncore.forcewake_count == 0)
		dev_priv->uncore.funcs.force_wake_put(dev_priv, fw_engine);
	spin_unlock_irqrestore(&dev_priv->uncore.lock, irqflags);
}

/* We give fast paths for the really cool registers */
#define NEEDS_FORCE_WAKE(dev_priv, reg) \
	((HAS_FORCE_WAKE((dev_priv)->dev)) && \
	 ((reg) < 0x40000) &&            \
	 ((reg) != FORCEWAKE))

static void
ilk_dummy_write(struct drm_i915_private *dev_priv)
{
	/* WaIssueDummyWriteToWakeupFromRC6:ilk Issue a dummy write to wake up
	 * the chip from rc6 before touching it for real. MI_MODE is masked,
	 * hence harmless to write 0 into. */
	__raw_i915_write32(dev_priv, MI_MODE, 0);
}

static void
hsw_unclaimed_reg_clear(struct drm_i915_private *dev_priv, u32 reg)
{
	if (HAS_FPGA_DBG_UNCLAIMED(dev_priv->dev) &&
	    (__raw_i915_read32(dev_priv, FPGA_DBG) & FPGA_DBG_RM_NOCLAIM)) {
		DRM_ERROR("Unknown unclaimed register before writing to %x\n",
			  reg);
		__raw_i915_write32(dev_priv, FPGA_DBG, FPGA_DBG_RM_NOCLAIM);
	}
}

static void
hsw_unclaimed_reg_check(struct drm_i915_private *dev_priv, u32 reg)
{
	if (HAS_FPGA_DBG_UNCLAIMED(dev_priv->dev) &&
	    (__raw_i915_read32(dev_priv, FPGA_DBG) & FPGA_DBG_RM_NOCLAIM)) {
		DRM_ERROR("Unclaimed write to %x\n", reg);
		__raw_i915_write32(dev_priv, FPGA_DBG, FPGA_DBG_RM_NOCLAIM);
	}
}

#define __i915_read(x) \
u##x i915_read##x(struct drm_i915_private *dev_priv, u32 reg, bool trace) { \
	unsigned long irqflags; \
	int fwengine = FORCEWAKE_ALL; \
	bool forcewake = true; \
	unsigned int *fwcount = &dev_priv->uncore.forcewake_count; \
	u##x val = 0; \
	if (IS_VALLEYVIEW(dev_priv->dev)) {			\
		if (FORCEWAKE_VLV_RENDER_RANGE_OFFSET(reg)) {   \
			fwengine = FORCEWAKE_RENDER;            \
			fwcount = &dev_priv->uncore.fw_rendercount;    \
		}                                               \
		else if (FORCEWAKE_VLV_MEDIA_RANGE_OFFSET(reg)) {       \
			fwengine = FORCEWAKE_MEDIA;             \
			fwcount = &dev_priv->uncore.fw_mediacount;     \
		}                                               \
	else							\
		forcewake = false;				\
	}							\
	spin_lock_irqsave(&dev_priv->uncore.lock, irqflags); \
	if (dev_priv->info->gen == 5) \
		ilk_dummy_write(dev_priv); \
	if (NEEDS_FORCE_WAKE((dev_priv), (reg)) && (trace) && (forcewake)) { \
		if ((*fwcount)++ == 0) \
			dev_priv->uncore.funcs.force_wake_get(dev_priv, \
								fwengine); \
		val = __raw_i915_read##x(dev_priv, reg); \
		if (--(*fwcount) == 0) \
			dev_priv->uncore.funcs.force_wake_put(dev_priv,  \
								fwengine); \
	} else { \
		val = __raw_i915_read##x(dev_priv, reg); \
	} \
	spin_unlock_irqrestore(&dev_priv->uncore.lock, irqflags); \
	trace_i915_reg_rw(false, reg, val, sizeof(val), trace); \
	return val; \
}

__i915_read(8)
__i915_read(16)
__i915_read(32)
__i915_read(64)
#undef __i915_read

#define __i915_write(x) \
void i915_write##x(struct drm_i915_private *dev_priv, u32 reg, u##x val, bool trace) { \
	unsigned long irqflags; \
	u32 __fifo_ret = 0; \
	trace_i915_reg_rw(true, reg, val, sizeof(val), trace); \
	spin_lock_irqsave(&dev_priv->uncore.lock, irqflags); \
	if (NEEDS_FORCE_WAKE((dev_priv), (reg))) { \
		__fifo_ret = __gen6_gt_wait_for_fifo(dev_priv); \
	} \
	if (dev_priv->info->gen == 5) \
		ilk_dummy_write(dev_priv); \
	hsw_unclaimed_reg_clear(dev_priv, reg); \
	__raw_i915_write##x(dev_priv, reg, val); \
	if (unlikely(__fifo_ret)) { \
		gen6_gt_check_fifodbg(dev_priv); \
	} \
	hsw_unclaimed_reg_check(dev_priv, reg); \
	spin_unlock_irqrestore(&dev_priv->uncore.lock, irqflags); \
}
__i915_write(8)
__i915_write(16)
__i915_write(32)
__i915_write(64)
#undef __i915_write

void i915_write_bits32(struct drm_i915_private *dev_priv, u32 reg, u32 val, u32 mask, bool trace) { \
	u32 tmp;
	tmp = I915_READ(reg);
	tmp &= ~mask;
	val &= mask;
	val |= tmp;
	I915_WRITE(reg, val);
}

static const struct register_whitelist {
	uint64_t offset;
	uint32_t size;
	uint32_t gen_bitmask; /* support gens, 0x10 for 4, 0x30 for 4 and 5, etc. */
} whitelist[] = {
	{ RING_TIMESTAMP_LO(RENDER_RING_BASE), 4, 0xF0 },
	{ RING_TIMESTAMP_HI(RENDER_RING_BASE), 4, 0xF0 }
};

int i915_reg_read_ioctl(struct drm_device *dev,
			void *data, struct drm_file *file)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_i915_reg_read *reg = data;
	struct register_whitelist const *entry = whitelist;
	int i;

	for (i = 0; i < ARRAY_SIZE(whitelist); i++, entry++) {
		if (entry->offset == reg->offset &&
		    (1 << INTEL_INFO(dev)->gen & entry->gen_bitmask))
			break;
	}

	if (i == ARRAY_SIZE(whitelist))
		return -EINVAL;

	switch (entry->size) {
	case 8:
		reg->val = I915_READ64(reg->offset);
		break;
	case 4:

		reg->val = I915_READ(reg->offset);
		break;
	case 2:
		reg->val = I915_READ16(reg->offset);
		break;
	case 1:
		reg->val = I915_READ8(reg->offset);
		break;
	default:
		WARN_ON(1);
		return -EINVAL;
	}

	return 0;
}

int i915_get_reset_stats_ioctl(struct drm_device *dev,
			       void *data, struct drm_file *file)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_i915_reset_stats *args = data;
	struct i915_ctx_hang_stats *hs;
	int ret;

	if (args->ctx_id == 0 && !capable(CAP_SYS_ADMIN))
		return -EPERM;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	hs = i915_gem_context_get_hang_stats(dev, file, args->ctx_id);
	if (IS_ERR(hs)) {
		mutex_unlock(&dev->struct_mutex);
		return PTR_ERR(hs);
	}

	if (capable(CAP_SYS_ADMIN))
		args->reset_count = i915_reset_count(&dev_priv->gpu_error);
	else
		args->reset_count = 0;

	args->batch_active = hs->batch_active;
	args->batch_pending = hs->batch_pending;

	mutex_unlock(&dev->struct_mutex);

	return 0;
}

static int i8xx_do_reset(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (IS_I85X(dev))
		return -ENODEV;

	I915_WRITE(D_STATE, I915_READ(D_STATE) | DSTATE_GFX_RESET_I830);
	POSTING_READ(D_STATE);

	if (IS_I830(dev) || IS_845G(dev)) {
		I915_WRITE(DEBUG_RESET_I830,
			   DEBUG_RESET_DISPLAY |
			   DEBUG_RESET_RENDER |
			   DEBUG_RESET_FULL);
		POSTING_READ(DEBUG_RESET_I830);
		msleep(1);

		I915_WRITE(DEBUG_RESET_I830, 0);
		POSTING_READ(DEBUG_RESET_I830);
	}

	msleep(1);

	I915_WRITE(D_STATE, I915_READ(D_STATE) & ~DSTATE_GFX_RESET_I830);
	POSTING_READ(D_STATE);

	return 0;
}

static int i965_reset_complete(struct drm_device *dev)
{
	u8 gdrst = 0;
	pci_read_config_byte(dev->pdev, I965_GDRST, &gdrst);
	return (gdrst & GRDOM_RESET_ENABLE) == 0;
}

static int i965_do_reset(struct drm_device *dev)
{
	int ret;

	/*
	 * Set the domains we want to reset (GRDOM/bits 2 and 3) as
	 * well as the reset bit (GR/bit 0).  Setting the GR bit
	 * triggers the reset; when done, the hardware will clear it.
	 */
	pci_write_config_byte(dev->pdev, I965_GDRST,
			      GRDOM_RENDER | GRDOM_RESET_ENABLE);
	ret =  wait_for(i965_reset_complete(dev), 500);
	if (ret)
		return ret;

	/* We can't reset render&media without also resetting display ... */
	pci_write_config_byte(dev->pdev, I965_GDRST,
			      GRDOM_MEDIA | GRDOM_RESET_ENABLE);

	ret =  wait_for(i965_reset_complete(dev), 500);
	if (ret)
		return ret;

	pci_write_config_byte(dev->pdev, I965_GDRST, 0);

	return 0;
}

static int ironlake_do_reset(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 gdrst;
	int ret;

	gdrst = I915_READ(MCHBAR_MIRROR_BASE + ILK_GDSR);
	gdrst &= ~GRDOM_MASK;
	I915_WRITE(MCHBAR_MIRROR_BASE + ILK_GDSR,
		   gdrst | GRDOM_RENDER | GRDOM_RESET_ENABLE);
	ret = wait_for(I915_READ(MCHBAR_MIRROR_BASE + ILK_GDSR) & 0x1, 500);
	if (ret)
		return ret;

	/* We can't reset render&media without also resetting display ... */
	gdrst = I915_READ(MCHBAR_MIRROR_BASE + ILK_GDSR);
	gdrst &= ~GRDOM_MASK;
	I915_WRITE(MCHBAR_MIRROR_BASE + ILK_GDSR,
		   gdrst | GRDOM_MEDIA | GRDOM_RESET_ENABLE);
	return wait_for(I915_READ(MCHBAR_MIRROR_BASE + ILK_GDSR) & 0x1, 500);
}

static int gen6_do_reset(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int	ret;
	unsigned long irqflags;

	/* Hold uncore.lock across reset to prevent any register access
	 * with forcewake not set correctly
	 */
	spin_lock_irqsave(&dev_priv->uncore.lock, irqflags);

	/* Reset the chip */

	/* GEN6_GDRST is not in the gt power well, no need to check
	 * for fifo space for the write or forcewake the chip for
	 * the read
	 */
	__raw_i915_write32(dev_priv, GEN6_GDRST, GEN6_GRDOM_FULL);

	/* Spin waiting for the device to ack the reset request */
	ret = wait_for((__raw_i915_read32(dev_priv, GEN6_GDRST) & GEN6_GRDOM_FULL) == 0, 500);

	gen6_gt_force_wake_restore(dev_priv);

	spin_unlock_irqrestore(&dev_priv->uncore.lock, irqflags);
	return ret;
}

int intel_gpu_reset(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int ret = -ENODEV;

	switch (INTEL_INFO(dev)->gen) {
	case 7:
	case 6:
			ret = gen6_do_reset(dev);
			break;
	case 5:
			ret = ironlake_do_reset(dev);
			break;
	case 4:
			ret = i965_do_reset(dev);
			break;
	case 2:
			ret = i8xx_do_reset(dev);
			break;

	default:
			ret = -ENODEV;
			break;
	}

	dev_priv->gpu_error.total_resets++;

	DRM_DEBUG_TDR("total_resets %d\n", dev_priv->gpu_error.total_resets);

	return ret;
}


static int gen6_do_engine_reset(struct drm_device *dev,
				enum intel_ring_id engine)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int	ret = -ENODEV;
	unsigned long irqflags;
	char *reset_event[2];
	reset_event[1] = NULL;

	/* Hold uncore.lock across reset to prevent any register access
	 * with forcewake not set correctly
	 */
	spin_lock_irqsave(&dev_priv->uncore.lock, irqflags);

	/* Reset the engine */
	/* GEN6_GDRST is not in the gt power well so no need to check
	* for fifo space for the write or forcewake the chip for
	* the read
	*/
	switch (engine) {
	case RCS:
		__raw_i915_write32(dev_priv, GEN6_GDRST, GEN6_GRDOM_RENDER);

		dev_priv->hangcheck[RCS].total++;

		/* Spin waiting for the device to ack the reset request */
		ret = wait_for_atomic_us((__raw_i915_read32(dev_priv,
					GEN6_GDRST)
					& GEN6_GRDOM_RENDER) == 0, 500);
		DRM_DEBUG_TDR("RCS Reset\n");
		break;


	case BCS:
		__raw_i915_write32(dev_priv, GEN6_GDRST, GEN6_GRDOM_BLT);
		dev_priv->hangcheck[BCS].total++;

		/* Spin waiting for the device to ack the reset request */
		ret = wait_for_atomic_us((__raw_i915_read32(dev_priv,
					GEN6_GDRST)
					& GEN6_GRDOM_BLT) == 0, 500);
		DRM_DEBUG_TDR("BCS Reset\n");
		break;


	case VCS:
		__raw_i915_write32(dev_priv, GEN6_GDRST, GEN6_GRDOM_MEDIA);
		dev_priv->hangcheck[VCS].total++;

		/* Spin waiting for the device to ack the reset request */
		ret = wait_for_atomic_us((__raw_i915_read32(dev_priv,
					GEN6_GDRST)
					& GEN6_GRDOM_MEDIA) == 0, 500);
		DRM_DEBUG_TDR("VCS Reset\n");
		break;


	default:
		DRM_ERROR("Unexpected engine\n");
		break;
	}

	gen6_gt_force_wake_restore(dev_priv);

	spin_unlock_irqrestore(&dev_priv->uncore.lock, irqflags);

	/* Do uevent outside of spinlock as uevent can sleep */
	reset_event[0] = kasprintf(GFP_KERNEL, "RESET RING=%d", engine);
	kobject_uevent_env(&dev->primary->kdev.kobj,
		KOBJ_CHANGE, reset_event);
	kfree(reset_event[0]);

	return ret;
}


int intel_gpu_engine_reset(struct drm_device *dev, enum intel_ring_id engine)
{
	struct drm_i915_private *dev_priv;
	struct intel_ring_buffer *ring;
	/* Reset an individual engine */
	int ret = -ENODEV;

	if (!dev)
		return -EINVAL;

	dev_priv = dev->dev_private;

	switch (INTEL_INFO(dev)->gen) {
	case 7:
	case 6:
			ret = gen6_do_engine_reset(dev, engine);
			/* Driver should invalidate TLB after soft resets */
			if (!ret) {
				ring = &dev_priv->ring[engine];
				if (ring->invalidate_tlb)
					ring->invalidate_tlb(ring);
			}
			break;
	default:
			DRM_ERROR("Engine reset not supported\n");
			ret = -ENODEV;
			break;
	}

	return ret;
}


int i915_handle_hung_ring(struct drm_device *dev, uint32_t ringid)
{
	/* TDR Version 1:
	* Reset the ring that is hung
	*
	* WARNING: Hold dev->struct_mutex before entering
	*          this function
	*/
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct intel_ring_buffer *ring = &dev_priv->ring[ringid];
	struct drm_crtc *crtc;
	struct intel_crtc *intel_crtc;
	int ret = 0;
	int pipe = 0;
	struct intel_unpin_work *unpin_work;
	uint32_t ring_flags = 0;
	uint32_t head;
	struct drm_i915_gem_request *request;
	u32 completed_seqno;
	u32 acthd;

	acthd = intel_ring_get_active_head(ring);
	completed_seqno = ring->get_seqno(ring, false);

	BUG_ON(!mutex_is_locked(&dev->struct_mutex));

	/* Take wake lock to prevent power saving mode */
	if (HAS_FORCE_WAKE(dev))
		gen6_gt_force_wake_get(dev_priv, FORCEWAKE_ALL);

	/* Search the request list to see which batch buffer caused
	* the hang. Only checks requests that haven't yet completed.*/
	list_for_each_entry(request, &ring->request_list, list) {
		if (request && (request->seqno > completed_seqno))
			i915_set_reset_status(ring, request, acthd);
	}

	/* Check if the ring has hung on a MI_DISPLAY_FLIP command.
	* The pipe value will be stored in the HWS page if it has.
	* At the moment this should only happen for the blitter but
	* each ring has its own status page so this should work for
	* all rings*/
	pipe = intel_read_status_page(ring, I915_GEM_PGFLIP_INDEX);
	if (pipe) {
		/* Clear it to avoid responding to it twice*/
		intel_write_status_page(ring, I915_GEM_PGFLIP_INDEX, 0);
	}

	/* Clear any simulated hang flags */
	if (dev_priv->gpu_error.stop_rings) {
		DRM_DEBUG_TDR("Simulated gpu hang, rst stop_rings bits %08x\n",
			(0x1 << ringid));
		dev_priv->gpu_error.stop_rings &= ~(0x1 << ringid);
	}

	DRM_DEBUG_TDR("Resetting ring %d\n", ringid);

	ret = intel_ring_disable(ring);

	if (ret != 0) {
		DRM_ERROR("Failed to disable ring %d\n", ringid);
		goto handle_hung_ring_error;
	}

	/* Sample the current ring head position */
	head = I915_READ(RING_HEAD(ring->mmio_base)) & HEAD_ADDR;
	DRM_DEBUG_TDR("head 0x%08X, last_head 0x%08X\n",
		      head, dev_priv->hangcheck[ringid].last_head);
	if (head == dev_priv->hangcheck[ringid].last_head) {
		/* The ring has not advanced since the last
		* time it hung so force it to advance to the
		* next QWORD. In most cases the ring head
		* pointer will automatically advance to the
		* next instruction as soon as it has read the
		* current instruction, without waiting for it
		* to complete. This seems to be the default
		* behaviour, however an MBOX wait inserted
		* directly to the VCS/BCS rings does not behave
		* in the same way, instead the head pointer
		* will still be pointing at the MBOX instruction
		* until it completes.*/
		ring_flags = FORCE_ADVANCE;
		DRM_DEBUG_TDR("Force ring head to advance\n");
	}
	dev_priv->hangcheck[ringid].last_head = head;

	ret = intel_ring_save(ring, ring_flags);

	if (ret != 0) {
		DRM_ERROR("Failed to save ring state\n");
		goto handle_hung_ring_error;
	}

	ret = intel_gpu_engine_reset(dev, ringid);

	if (ret != 0) {
		DRM_ERROR("Failed to reset ring\n");
		goto handle_hung_ring_error;
	}

	/* Clear last_acthd in hangcheck timer for this ring */
	dev_priv->hangcheck[ringid].last_acthd = 0;

	/* Clear reset flags to allow future hangchecks */
	atomic_set(&dev_priv->hangcheck[ringid].flags, 0);

	ret = intel_ring_restore(ring);

	if (ret != 0) {
		DRM_ERROR("Failed to restore ring state\n");
		goto handle_hung_ring_error;
	}

	/* Correct driver state */
	intel_ring_resample(ring);

	DRM_ERROR("Reset ring %d (GPU Hang)\n", ringid);

	ret = intel_ring_enable(ring);

	if (ret != 0) {
		DRM_ERROR("Failed to enable ring\n");
		goto handle_hung_ring_error;
	}

	/* Wake up anything waiting on this rings queue */
	wake_up_all(&ring->irq_queue);

	/* Note: This will not happen when MMIO based page flipping is used.
	* Page flipping should continue unhindered as it will
	* not be relying on a ring.*/
	if (pipe &&
		((pipe - 1) < ARRAY_SIZE(dev_priv->pipe_to_crtc_mapping))) {
		/* The pipe value in the status page if offset by 1 */
		pipe -= 1;

		/* The ring hung on a page flip command so we
		* must manually release the pending flip queue */
		crtc = dev_priv->pipe_to_crtc_mapping[pipe];
		intel_crtc = to_intel_crtc(crtc);
		unpin_work = intel_crtc->unpin_work;

		if (unpin_work && unpin_work->pending_flip_obj) {
			intel_prepare_page_flip(dev, intel_crtc->pipe);
			intel_finish_page_flip(dev, intel_crtc->pipe);
			DRM_DEBUG_TDR("Released stuck page flip for pipe %d\n",
				pipe);
		}
	}

handle_hung_ring_error:
	/* Release power lock */
	if (HAS_FORCE_WAKE(dev))
		gen6_gt_force_wake_put(dev_priv, FORCEWAKE_ALL);

	return ret;
}

void intel_uncore_clear_errors(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	/* XXX needs spinlock around caller's grouping */
	if (HAS_FPGA_DBG_UNCLAIMED(dev))
		__raw_i915_write32(dev_priv, FPGA_DBG, FPGA_DBG_RM_NOCLAIM);
}

void intel_uncore_check_errors(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (HAS_FPGA_DBG_UNCLAIMED(dev) &&
	    (__raw_i915_read32(dev_priv, FPGA_DBG) & FPGA_DBG_RM_NOCLAIM)) {
		DRM_ERROR("Unclaimed register before interrupt\n");
		__raw_i915_write32(dev_priv, FPGA_DBG, FPGA_DBG_RM_NOCLAIM);
	}
}
