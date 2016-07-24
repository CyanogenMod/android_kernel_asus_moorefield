/*
 * Copyright © 2008 Intel Corporation
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
 *    Keith Packard <keithp@keithp.com>
 *
 */

#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/export.h>
#include <linux/list_sort.h>
#include <linux/string.h>
#include <asm/msr-index.h>
#include <drm/drmP.h>
#include "intel_drv.h"
#include "intel_ringbuffer.h"
#include <drm/i915_drm.h>
#include "i915_drv.h"
#include "i915_debugfs.h"
#include <linux/moduleparam.h>
#include "linux/mfd/intel_mid_pmic.h"
#include <linux/pwm.h>

#define DRM_I915_RING_DEBUG 1

#if defined(CONFIG_DEBUG_FS)

static const char *yesno(int v)
{
	return v ? "yes" : "no";
}

static int i915_capabilities(struct seq_file *m, void *data)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	const struct intel_device_info *info = INTEL_INFO(dev);

	seq_printf(m, "gen: %d\n", info->gen);
	seq_printf(m, "pch: %d\n", INTEL_PCH_TYPE(dev));
#define PRINT_FLAG(x)  seq_printf(m, #x ": %s\n", yesno(info->x))
#define SEP_SEMICOLON ;
	DEV_INFO_FOR_EACH_FLAG(PRINT_FLAG, SEP_SEMICOLON);
#undef PRINT_FLAG
#undef SEP_SEMICOLON

	return 0;
}

static const char *get_pin_flag(struct drm_i915_gem_object *obj)
{
	if (obj->user_pin_count > 0)
		return "P";
	else if (obj->pin_count > 0)
		return "p";
	else
		return " ";
}

static const char *get_tiling_flag(struct drm_i915_gem_object *obj)
{
	switch (obj->tiling_mode) {
	default:
	case I915_TILING_NONE: return " ";
	case I915_TILING_X: return "X";
	case I915_TILING_Y: return "Y";
	}
}

static inline const char *get_global_flag(struct drm_i915_gem_object *obj)
{
	return obj->has_global_gtt_mapping ? "g" : " ";
}

ssize_t i915_gamma_adjust_read(struct file *filp,
		 char __user *ubuf,
		 size_t max,
		 loff_t *ppos)
{
	/* To do: Not implemented yet */
	DRM_ERROR("Gamma adjust: Not implemented\n");
	return -EINVAL;
}

ssize_t i915_gamma_adjust_write(struct file *filp,
		  const char __user *ubuf,
		  size_t count,
		  loff_t *ppos)
{
	int ret = 0;
	int	pipe;
	int crtc_id = -1;
	int bytes_count;
	int bytes_read;
	char *buf = NULL;
	char *temp_buf = NULL;
	struct drm_device *dev = filp->private_data;
	struct drm_crtc *crtc = NULL;
	struct drm_mode_object *obj;

	/* Validate input */
	if (!count) {
		DRM_ERROR("Gamma adjust: insufficient data\n");
		return -EINVAL;
	}

	buf = kzalloc(count, GFP_KERNEL);
	if (!buf) {
		DRM_ERROR("Gamma adjust: insufficient memory\n");
		return -ENOMEM;
	}

	/* Get the data */
	if (copy_from_user(buf, ubuf, count)) {
		DRM_ERROR("Gamma adjust: copy failed\n");
		ret = -EINVAL;
		goto exit;
	}
	bytes_read = 0;
	bytes_count = count;
	if (bytes_count > 0) {
		temp_buf = buf + bytes_read;
		/* Parse data and read the crtc_id */
		ret = parse_clrmgr_input(&crtc_id, temp_buf,
			CRTC_ID_TOKEN_COUNT, &bytes_count);
		if (ret < CRTC_ID_TOKEN_COUNT) {
			DRM_ERROR("CRTC_ID loading failed\n");
			goto exit;
		} else
			DRM_DEBUG("CRTC_ID loading done\n");
	}

	obj = drm_mode_object_find(dev, crtc_id, DRM_MODE_OBJECT_CRTC);
	if (!obj) {
		DRM_DEBUG_KMS("Unknown CRTC ID %d\n", crtc_id);
		return -EINVAL;
	}
	crtc = obj_to_crtc(obj);
	DRM_DEBUG_KMS("[CRTC:%d]\n", crtc->base.id);

	pipe = to_intel_crtc(crtc)->pipe;
	bytes_read += bytes_count;
	bytes_count = count - bytes_read;
	if (bytes_count > 0) {
		temp_buf = buf + bytes_read;
		/* Parse data and load the gamma  table */
		ret = parse_clrmgr_input(gamma_softlut[pipe], temp_buf,
			GAMMA_CORRECT_MAX_COUNT, &bytes_count);
		if (ret < GAMMA_CORRECT_MAX_COUNT) {
			DRM_ERROR("Gamma table loading failed\n");
			goto exit;
		} else
			DRM_DEBUG("Gamma table loading done\n");
	}
	bytes_read += bytes_count;
	bytes_count = count - bytes_read;
	if (bytes_count > 0) {
		temp_buf = buf + bytes_read;

		/* Parse data and load the gcmax table */
		ret = parse_clrmgr_input(gcmax_softlut[pipe], temp_buf,
				GC_MAX_COUNT, &bytes_count);
		if (ret < GC_MAX_COUNT)
			DRM_ERROR("GCMAX table loading failed\n");
		else
			DRM_DEBUG("GCMAX table loading done\n");
	}
exit:
	kfree(buf);
	if (ret < 0)
		return ret;

	return count;
}

ssize_t i915_gamma_enable_read(struct file *filp,
		 char __user *ubuf,
		 size_t max,
		 loff_t *ppos)
{
	int len = 0;
	char buf[40] = {0,};
	struct drm_device *dev = filp->private_data;
	drm_i915_private_t *dev_priv = dev->dev_private;

	len = scnprintf(buf, sizeof(buf), "%s\n%s\n",
		dev_priv->gamma_enabled[0] ? "Pipe 0: Enabled" : "Pipe 0: Disabled",
		dev_priv->gamma_enabled[1] ? "Pipe 1: Enabled" : "Pipe 1: Disabled");

	return simple_read_from_buffer(ubuf, max, ppos,
		(const void *) buf, len);
}
ssize_t i915_gamma_enable_write(struct file *filp,
		  const char __user *ubuf,
		  size_t count,
		  loff_t *ppos)
{
	int ret = 0;
	int	bytes_read;
	int bytes_count;
	int crtc_id = -1;
	int req_state = 0;
	struct drm_crtc *crtc = NULL;
	struct drm_device *dev = filp->private_data;
	char *buf = NULL, *temp_buf = NULL;
	struct drm_mode_object *obj;

	/* Validate input */
	if (!count) {
		DRM_ERROR("Gamma adjust: insufficient data\n");
		return -EINVAL;
	}

	buf = kzalloc(count, GFP_KERNEL);
	if (!buf) {
		DRM_ERROR("Gamma enable: Out of mem\n");
		return  -ENOMEM;
	}

	/* Get the data */
	if (copy_from_user(buf, ubuf, count)) {
		DRM_ERROR("Gamma adjust: copy failed\n");
		ret = -EINVAL;
		goto exit;
	}
	bytes_read = 0;
	bytes_count = count;
	if (bytes_count > 0) {
		temp_buf = buf + bytes_read;
		/* Parse data and load the crtc_id */
		ret = parse_clrmgr_input(&crtc_id, temp_buf,
			CRTC_ID_TOKEN_COUNT, &bytes_count);
		if (ret < CRTC_ID_TOKEN_COUNT) {
			DRM_ERROR("CRTC_ID loading failed\n");
			goto exit;
		} else
			DRM_DEBUG("CRTC_ID loading done\n");
	}

	obj = drm_mode_object_find(dev, crtc_id, DRM_MODE_OBJECT_CRTC);
	if (!obj) {
		DRM_DEBUG_KMS("Unknown CRTC ID %d\n", crtc_id);
		return -EINVAL;
	}
	crtc = obj_to_crtc(obj);
	DRM_DEBUG_KMS("[CRTC:%d]\n", crtc->base.id);

	bytes_read += bytes_count;
	bytes_count = count - bytes_read;
	if (bytes_count > 0) {
		temp_buf = buf + bytes_read;
		/* Parse data and load the gamma  table */
		ret = parse_clrmgr_input(&req_state, temp_buf,
			ENABLE_TOKEN_MAX_COUNT, &bytes_count);
		if (ret < ENABLE_TOKEN_MAX_COUNT) {
			DRM_ERROR("Enable-token loading failed\n");
			goto exit;
		} else
			DRM_DEBUG("Enable-token loading done\n");
	}

	/* if gamma enabled, apply gamma correction on PIPE */
	if (req_state) {
		if (intel_crtc_enable_gamma(crtc,
				to_intel_crtc(crtc)->pipe ? PIPEB : PIPEA)) {
			DRM_ERROR("Apply gamma correction failed\n");
			ret = -EINVAL;
		} else
			ret = count;
	} else {
		/* Disable gamma on this plane */
		intel_crtc_disable_gamma(crtc,
			to_intel_crtc(crtc)->pipe ? PIPEB : PIPEA);
		ret = count;
	}
exit:
	kfree(buf);
	return ret;
}

const struct file_operations i915_gamma_adjust_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = i915_gamma_adjust_read,
	.write = i915_gamma_adjust_write,
	.llseek = default_llseek,
};

const struct file_operations i915_gamma_enable_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = i915_gamma_enable_read,
	.write = i915_gamma_enable_write,
	.llseek = default_llseek,
};

ssize_t i915_cb_adjust_read(struct file *filp,
		 char __user *ubuf,
		 size_t max,
		 loff_t *ppos)
{
	/* To do: Not implemented yet */
	DRM_ERROR("Contrast Brightness adjust: Read Not implemented\n");
	return -EINVAL;
}

ssize_t i915_cb_adjust_write(struct file *filp,
		  const char __user *ubuf,
		  size_t count,
		  loff_t *ppos)
{
	int ret = count;
	int bytes_count = count;
	struct drm_device *dev = filp->private_data;
	struct cont_brightlut *cb_ptr = NULL;
	drm_i915_private_t *dev_priv = dev->dev_private;
	char *buf = NULL;

	/* Validate input */
	if (!count) {
		DRM_ERROR("Contrast Brightness: insufficient data\n");
		return -EINVAL;
	}

	buf = kzalloc(count, GFP_KERNEL);
	if (!buf) {
		DRM_ERROR("Contrast Brightness adjust: insufficient memory\n");
		return -ENOMEM;
	}

	cb_ptr = kzalloc(sizeof(struct cont_brightlut), GFP_KERNEL);
	if (!cb_ptr) {
		DRM_ERROR("Contrast Brightness adjust: insufficient memory\n");
		kfree(buf);
		return -ENOMEM;
	}

	/* Get the data */
	if (copy_from_user(buf, ubuf, count)) {
		DRM_ERROR("Contrast Brightness: copy failed\n");
		ret = -EINVAL;
		goto exit;
	}

	/* Parse input data */
	ret = parse_clrmgr_input((uint *)cb_ptr, buf, CB_MAX_COEFF_COUNT, &bytes_count);
	if (ret < CB_MAX_COEFF_COUNT) {
		DRM_ERROR("Contrast Brightness loading failed\n");
		goto exit;
	}
	else
		DRM_DEBUG("Contrast Brightness loading done\n");

	if (cb_ptr->sprite_no < SPRITEA || cb_ptr->sprite_no > SPRITED ||
			cb_ptr->sprite_no == PLANEB) {
		DRM_ERROR("Sprite value out of range. Enter 2,3, 5 or 6\n");
		goto exit;
	}

	DRM_DEBUG("sprite = %d Val=0x%x,\n", cb_ptr->sprite_no, cb_ptr->val);

	if (intel_sprite_cb_adjust(dev_priv, cb_ptr))
		DRM_ERROR("Contrast Brightness update failed\n");

exit:
	kfree(cb_ptr);
	kfree(buf);
	if (ret < 0)
		return ret;

	return count;
}

ssize_t i915_hs_adjust_read(struct file *filp,
		 char __user *ubuf,
		 size_t max,
		 loff_t *ppos)
{
	/* To do: Not implemented yet */
	DRM_ERROR("Hue Saturation adjust: Read Not implemented\n");
	return -EINVAL;
}
ssize_t i915_hs_adjust_write(struct file *filp,
		  const char __user *ubuf,
		  size_t count,
		  loff_t *ppos)
{
	int ret = count;
	int bytes_count = count;
	struct drm_device *dev = filp->private_data;
	struct hue_saturationlut *hs_ptr = NULL;
	drm_i915_private_t *dev_priv = dev->dev_private;
	char *buf = NULL;

	/* Validate input */
	if (!count) {
		DRM_ERROR("Hue Saturation: insufficient data\n");
		return -EINVAL;
	}

	buf = kzalloc(count, GFP_KERNEL);
	if (!buf) {
		DRM_ERROR("Hue Saturation adjust: insufficient memory\n");
		return -ENOMEM;
	}

	hs_ptr = kzalloc(sizeof(struct hue_saturationlut), GFP_KERNEL);
	if (!hs_ptr) {
		DRM_ERROR("Hue Saturation adjust: insufficient memory\n");
		kfree(buf);
		return -ENOMEM;
	}

	/* Get the data */
	if (copy_from_user(buf, ubuf, count)) {
		DRM_ERROR("Hue Saturation: copy failed\n");
		ret = -EINVAL;
		goto exit;
	}

	/* Parse input data */
	ret = parse_clrmgr_input((uint *)hs_ptr, buf, HS_MAX_COEFF_COUNT, &bytes_count);
	if (ret < HS_MAX_COEFF_COUNT) {
		DRM_ERROR("Hue Saturation loading failed\n");
		goto exit;
	}
	else
		DRM_DEBUG("Hue Saturation loading done\n");

	if (hs_ptr->sprite_no < SPRITEA || hs_ptr->sprite_no > SPRITED ||
			hs_ptr->sprite_no == PLANEB) {
		DRM_ERROR("sprite = %d Val=0x%x,\n", hs_ptr->sprite_no,
					hs_ptr->val);
		goto exit;
	}

	DRM_DEBUG("sprite = %d Val=0x%x,\n", hs_ptr->sprite_no, hs_ptr->val);

	if (intel_sprite_hs_adjust(dev_priv, hs_ptr))
		DRM_ERROR("Hue Saturation update failed\n");

exit:
	kfree(hs_ptr);
	kfree(buf);

	if (ret < 0)
		return ret;

	return count;
}

ssize_t i915_csc_adjust_read(struct file *filp,
		 char __user *ubuf,
		 size_t max,
		 loff_t *ppos)
{
	/* To do: Not implemented yet */
	DRM_ERROR("CSC adjust: Not implemented\n");
	return -EINVAL;
}

ssize_t i915_csc_adjust_write(struct file *filp,
		  const char __user *ubuf,
		  size_t count,
		  loff_t *ppos)
{
	int bytes_count;
	int	bytes_read;
	int ret = 0;
	int pipe;
	int crtc_id = -1;
	char *buf = NULL;
	char *temp_buf = NULL;
	struct drm_device *dev = filp->private_data;
	struct drm_crtc *crtc = NULL;
	struct drm_mode_object *obj;

	/* Validate input */
	if (!count) {
		DRM_ERROR("CSC adjust: insufficient data\n");
		return -EINVAL;
	}

	buf = kzalloc(count, GFP_KERNEL);
	if (!buf) {
		DRM_ERROR("CSC adjust: insufficient memory\n");
		return -ENOMEM;
	}

	/* Get the data */
	if (copy_from_user(buf, ubuf, count)) {
		DRM_ERROR("CSC adjust: copy failed\n");
		ret = -EINVAL;
		goto exit;
	}
	bytes_read = 0;
	bytes_count = count;
	if (bytes_count > 0) {
		temp_buf = buf + bytes_read;
		/* Parse data and load the crtc_id */
		ret = parse_clrmgr_input(&crtc_id, temp_buf,
			CRTC_ID_TOKEN_COUNT, &bytes_count);
		if (ret < CRTC_ID_TOKEN_COUNT) {
			DRM_ERROR("CONNECTOR_TYPE_TOKEN loading failed\n");
			goto exit;
		} else
			DRM_DEBUG("CONNECTOR_TYPE_TOKEN loading done\n");
	}

	obj = drm_mode_object_find(dev, crtc_id, DRM_MODE_OBJECT_CRTC);
	if (!obj) {
		DRM_DEBUG_KMS("Unknown CRTC ID %d\n", crtc_id);
		return -EINVAL;
	}
	crtc = obj_to_crtc(obj);
	DRM_DEBUG_KMS("[CRTC:%d]\n", crtc->base.id);

	pipe = to_intel_crtc(crtc)->pipe;
	bytes_read += bytes_count;
	bytes_count = count - bytes_read;
	if (bytes_count > 0) {
		temp_buf = buf + bytes_read;
		/* Parse data and load the csc  table */
		ret = parse_clrmgr_input(csc_softlut[pipe], temp_buf,
			CSC_MAX_COEFF_COUNT, &bytes_count);
		if (ret < CSC_MAX_COEFF_COUNT)
			DRM_ERROR("CSC table loading failed\n");
		else
			DRM_DEBUG("CSC table loading done\n");
	}
exit:
	kfree(buf);
	if (ret < 0)
		return ret;

	return count;
}

ssize_t i915_csc_enable_read(struct file *filp,
		 char __user *ubuf,
		 size_t max,
		 loff_t *ppos)
{
	int len = 0;
	char buf[40] = {0,};
	struct drm_device *dev = filp->private_data;
	drm_i915_private_t *dev_priv = dev->dev_private;

	len = scnprintf(buf, sizeof(buf), "%s\n%s\n",
		dev_priv->csc_enabled[0] ? "Pipe 0: Enabled" : "Pipe 0: Disabled",
		dev_priv->csc_enabled[1] ? "Pipe 1: Enabled" : "Pipe 1: Disabled");
	return simple_read_from_buffer(ubuf, max, ppos,
		(const void *) buf, len);
}

ssize_t i915_csc_enable_write(struct file *filp,
		  const char __user *ubuf,
		  size_t count,
		  loff_t *ppos)
{
	int ret = 0;
	int pipe;
	int req_state = 0;
	int bytes_read;
	int bytes_count;
	int crtc_id = -1;
	char *buf = NULL;
	char *temp_buf = NULL;
	struct drm_crtc *crtc = NULL;
	struct drm_device *dev = filp->private_data;
	struct drm_mode_object *obj;

	/* Validate input */
	if (!count) {
		DRM_ERROR("CSC enable: insufficient data\n");
		return -EINVAL;
	}

	buf = kzalloc(count, GFP_KERNEL);
	if (!buf) {
		DRM_ERROR("CSC enable: Out of mem\n");
		return -ENOMEM;
	}

	/* Get the data */
	if (copy_from_user(buf, ubuf, count)) {
		DRM_ERROR("CSC enable: copy failed\n");
		ret = -EINVAL;
		goto exit;
	}
	bytes_read = 0;
	bytes_count = count;
	if (bytes_count > 0) {
		temp_buf = buf + bytes_read;
		/* Parse data and load the crtc_id */
		ret = parse_clrmgr_input(&crtc_id, temp_buf,
			CRTC_ID_TOKEN_COUNT, &bytes_count);
		if (ret < CRTC_ID_TOKEN_COUNT) {
			DRM_ERROR("CRTC_ID_TOKEN loading failed\n");
			goto exit;
		} else
			DRM_DEBUG("CRTC_ID_TOKEN loading done\n");
	}

	obj = drm_mode_object_find(dev, crtc_id, DRM_MODE_OBJECT_CRTC);
	if (!obj) {
		DRM_DEBUG_KMS("Unknown CRTC ID %d\n", crtc_id);
		return -EINVAL;
	}
	crtc = obj_to_crtc(obj);
	DRM_DEBUG_KMS("[CRTC:%d]\n", crtc->base.id);

	pipe = to_intel_crtc(crtc)->pipe;
	bytes_read += bytes_count;
	bytes_count = count - bytes_read;
	if (bytes_count > 0) {
		temp_buf = buf + bytes_read;
		/* Parse data and load the gamma  table */
		ret = parse_clrmgr_input(&req_state, temp_buf,
			ENABLE_TOKEN_MAX_COUNT, &bytes_count);
		if (ret < ENABLE_TOKEN_MAX_COUNT) {
			DRM_ERROR("Enable-token loading failed\n");
			goto exit;
		} else
			DRM_DEBUG("Enable-token loading done\n");
	}
	DRM_DEBUG_KMS("req_state:%d\n", req_state);

	/* if CSC enabled, apply CSC correction */
	if (req_state) {
		if (do_intel_enable_csc(dev,
					(void *) csc_softlut[pipe], crtc)) {
			DRM_ERROR("CSC correction failed\n");
			ret = -EINVAL;
		} else
			ret = count;
	} else {
		/* Disable CSC on this CRTC */
		do_intel_disable_csc(dev, crtc);
		ret = count;
	}

exit:
	kfree(buf);
	return ret;
}

static const struct file_operations i915_cb_adjust_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = i915_cb_adjust_read,
	.write = i915_cb_adjust_write,
	.llseek = default_llseek,
};

static const struct file_operations i915_hs_adjust_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = i915_hs_adjust_read,
	.write = i915_hs_adjust_write,
	.llseek = default_llseek,
};

static const struct file_operations i915_csc_adjust_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = i915_csc_adjust_read,
	.write = i915_csc_adjust_write,
	.llseek = default_llseek,
};

static const struct file_operations i915_csc_enable_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = i915_csc_enable_read,
	.write = i915_csc_enable_write,
	.llseek = default_llseek,
};


static void
describe_obj(struct seq_file *m, struct drm_i915_gem_object *obj)
{
	struct i915_vma *vma;
	seq_printf(m, "%pK: %s%s%s %8zdKiB %02x %02x %u %u %u%s%s%s",
		   &obj->base,
		   get_pin_flag(obj),
		   get_tiling_flag(obj),
		   get_global_flag(obj),
		   obj->base.size / 1024,
		   obj->base.read_domains,
		   obj->base.write_domain,
		   obj->last_read_seqno,
		   obj->last_write_seqno,
		   obj->last_fenced_seqno,
		   i915_cache_level_str(obj->cache_level),
		   obj->dirty ? " dirty" : "",
		   obj->madv == I915_MADV_DONTNEED ? " purgeable" : "");
	if (obj->base.name)
		seq_printf(m, " (name: %d)", obj->base.name);
	if (obj->pin_count)
		seq_printf(m, " (pinned x %d)", obj->pin_count);
	if (obj->pin_display)
		seq_printf(m, " (display)");
	if (obj->fence_reg != I915_FENCE_REG_NONE)
		seq_printf(m, " (fence: %d)", obj->fence_reg);
	list_for_each_entry(vma, &obj->vma_list, vma_link) {
		if (!i915_is_ggtt(vma->vm))
			seq_puts(m, " (pp");
		else
			seq_puts(m, " (g");
		seq_printf(m, "gtt offset: %08lx, size: %08lx)",
			   vma->node.start, vma->node.size);
	}
	if (obj->stolen)
		seq_printf(m, " (stolen: %08lx)", obj->stolen->start);
	if (obj->pin_mappable || obj->fault_mappable) {
		char s[3], *t = s;
		if (obj->pin_mappable)
			*t++ = 'p';
		if (obj->fault_mappable)
			*t++ = 'f';
		*t = '\0';
		seq_printf(m, " (%s mappable)", s);
	}
	if (obj->ring != NULL)
		seq_printf(m, " (%s)", obj->ring->name);
}

static int i915_gem_object_list_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	uintptr_t list = (uintptr_t) node->info_ent->data;
	struct list_head *head;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct i915_address_space *vm = &dev_priv->gtt.base;
	struct i915_vma *vma;
	size_t total_obj_size, total_gtt_size;
	int count, ret;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	/* FIXME: the user of this interface might want more than just GGTT */
	switch (list) {
	case ACTIVE_LIST:
		seq_puts(m, "Active:\n");
		head = &vm->active_list;
		break;
	case INACTIVE_LIST:
		seq_puts(m, "Inactive:\n");
		head = &vm->inactive_list;
		break;
	default:
		mutex_unlock(&dev->struct_mutex);
		return -EINVAL;
	}

	total_obj_size = total_gtt_size = count = 0;
	list_for_each_entry(vma, head, mm_list) {
		seq_printf(m, "   ");
		describe_obj(m, vma->obj);
		seq_printf(m, "\n");
		total_obj_size += vma->obj->base.size;
		total_gtt_size += vma->node.size;
		count++;
	}
	mutex_unlock(&dev->struct_mutex);

	seq_printf(m, "Total %d objects, %zu bytes, %zu GTT size\n",
		   count, total_obj_size, total_gtt_size);
	return 0;
}

static int obj_rank_by_stolen(void *priv,
			      struct list_head *A, struct list_head *B)
{
	struct drm_i915_gem_object *a =
		container_of(A, struct drm_i915_gem_object, obj_exec_link);
	struct drm_i915_gem_object *b =
		container_of(B, struct drm_i915_gem_object, obj_exec_link);

	return a->stolen->start - b->stolen->start;
}

static int i915_gem_stolen_list_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_i915_gem_object *obj;
	size_t total_obj_size, total_gtt_size;
	LIST_HEAD(stolen);
	int count, ret;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	total_obj_size = total_gtt_size = count = 0;
	list_for_each_entry(obj, &dev_priv->mm.bound_list, global_list) {
		if (obj->stolen == NULL)
			continue;

		list_add(&obj->obj_exec_link, &stolen);

		total_obj_size += obj->base.size;
		total_gtt_size += i915_gem_obj_ggtt_size(obj);
		count++;
	}
	list_for_each_entry(obj, &dev_priv->mm.unbound_list, global_list) {
		if (obj->stolen == NULL)
			continue;

		list_add(&obj->obj_exec_link, &stolen);

		total_obj_size += obj->base.size;
		count++;
	}
	list_sort(NULL, &stolen, obj_rank_by_stolen);
	seq_puts(m, "Stolen:\n");
	while (!list_empty(&stolen)) {
		obj = list_first_entry(&stolen, typeof(*obj), obj_exec_link);
		seq_puts(m, "   ");
		describe_obj(m, obj);
		seq_putc(m, '\n');
		list_del_init(&obj->obj_exec_link);
	}
	mutex_unlock(&dev->struct_mutex);

	seq_printf(m, "Total %d objects, %zu bytes, %zu GTT size\n",
		   count, total_obj_size, total_gtt_size);
	return 0;
}

#define count_objects(list, member) do { \
	list_for_each_entry(obj, list, member) { \
		size += i915_gem_obj_ggtt_size(obj); \
		++count; \
		if (obj->map_and_fenceable) { \
			mappable_size += i915_gem_obj_ggtt_size(obj); \
			++mappable_count; \
		} \
	} \
} while (0)

struct file_stats {
	int count;
	size_t total, active, inactive, unbound;
};

static int per_file_stats(int id, void *ptr, void *data)
{
	struct drm_i915_gem_object *obj = ptr;
	struct file_stats *stats = data;

	stats->count++;
	stats->total += obj->base.size;

	if (i915_gem_obj_ggtt_bound(obj)) {
		if (!list_empty(&obj->ring_list))
			stats->active += obj->base.size;
		else
			stats->inactive += obj->base.size;
	} else {
		if (!list_empty(&obj->global_list))
			stats->unbound += obj->base.size;
	}

	return 0;
}

#define count_vmas(list, member) do { \
	list_for_each_entry(vma, list, member) { \
		size += i915_gem_obj_ggtt_size(vma->obj); \
		++count; \
		if (vma->obj->map_and_fenceable) { \
			mappable_size += i915_gem_obj_ggtt_size(vma->obj); \
			++mappable_count; \
		} \
	} \
} while (0)

static int i915_gem_object_info(struct seq_file *m, void* data)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 count, mappable_count, purgeable_count;
	size_t size, mappable_size, purgeable_size;
	struct drm_i915_gem_object *obj;
	struct i915_address_space *vm = &dev_priv->gtt.base;
	struct drm_file *file;
	struct i915_vma *vma;
	int ret;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	seq_printf(m, "%u objects, %zu bytes\n",
		   dev_priv->mm.object_count,
		   dev_priv->mm.object_memory);

	size = count = mappable_size = mappable_count = 0;
	count_objects(&dev_priv->mm.bound_list, global_list);
	seq_printf(m, "%u [%u] objects, %zu [%zu] bytes in gtt\n",
		   count, mappable_count, size, mappable_size);

	size = count = mappable_size = mappable_count = 0;
	count_vmas(&vm->active_list, mm_list);
	seq_printf(m, "  %u [%u] active objects, %zu [%zu] bytes\n",
		   count, mappable_count, size, mappable_size);

	size = count = mappable_size = mappable_count = 0;
	count_vmas(&vm->inactive_list, mm_list);
	seq_printf(m, "  %u [%u] inactive objects, %zu [%zu] bytes\n",
		   count, mappable_count, size, mappable_size);

	size = count = purgeable_size = purgeable_count = 0;
	list_for_each_entry(obj, &dev_priv->mm.unbound_list, global_list) {
		size += obj->base.size, ++count;
		if (obj->madv == I915_MADV_DONTNEED)
			purgeable_size += obj->base.size, ++purgeable_count;
	}
	seq_printf(m, "%u unbound objects, %zu bytes\n", count, size);

	size = count = mappable_size = mappable_count = 0;
	list_for_each_entry(obj, &dev_priv->mm.bound_list, global_list) {
		if (obj->fault_mappable) {
			size += i915_gem_obj_ggtt_size(obj);
			++count;
		}
		if (obj->pin_mappable) {
			mappable_size += i915_gem_obj_ggtt_size(obj);
			++mappable_count;
		}
		if (obj->madv == I915_MADV_DONTNEED) {
			purgeable_size += obj->base.size;
			++purgeable_count;
		}
	}
	seq_printf(m, "%u purgeable objects, %zu bytes\n",
		   purgeable_count, purgeable_size);
	seq_printf(m, "%u pinned mappable objects, %zu bytes\n",
		   mappable_count, mappable_size);
	seq_printf(m, "%u fault mappable objects, %zu bytes\n",
		   count, size);

	seq_printf(m, "%zu [%lu] gtt total\n",
		   dev_priv->gtt.base.total,
		   dev_priv->gtt.mappable_end - dev_priv->gtt.base.start);

	seq_putc(m, '\n');
	list_for_each_entry_reverse(file, &dev->filelist, lhead) {
		struct file_stats stats;

		memset(&stats, 0, sizeof(stats));

		spin_lock(&file->table_lock);
		idr_for_each(&file->object_idr, per_file_stats, &stats);
		spin_unlock(&file->table_lock);

		seq_printf(m, "%s: %u objects, %zu bytes (%zu active, %zu inactive, %zu unbound)\n",
			   get_pid_task(file->pid, PIDTYPE_PID)->comm,
			   stats.count,
			   stats.total,
			   stats.active,
			   stats.inactive,
			   stats.unbound);
	}

	mutex_unlock(&dev->struct_mutex);

	return 0;
}

static int i915_gem_gtt_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	uintptr_t list = (uintptr_t) node->info_ent->data;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_i915_gem_object *obj;
	size_t total_obj_size, total_gtt_size;
	int count, ret;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	total_obj_size = total_gtt_size = count = 0;
	list_for_each_entry(obj, &dev_priv->mm.bound_list, global_list) {
		if (list == PINNED_LIST && obj->pin_count == 0)
			continue;

		seq_puts(m, "   ");
		describe_obj(m, obj);
		seq_putc(m, '\n');
		total_obj_size += obj->base.size;
		total_gtt_size += i915_gem_obj_ggtt_size(obj);
		count++;
	}

	mutex_unlock(&dev->struct_mutex);

	seq_printf(m, "Total %d objects, %zu bytes, %zu GTT size\n",
		   count, total_obj_size, total_gtt_size);

	return 0;
}

static int i915_gem_pageflip_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	unsigned long flags;
	struct intel_crtc *crtc;

	list_for_each_entry(crtc, &dev->mode_config.crtc_list, base.head) {
		const char pipe = pipe_name(crtc->pipe);
		const char plane = plane_name(crtc->plane);
		struct intel_unpin_work *work;

		spin_lock_irqsave(&dev->event_lock, flags);
		work = crtc->unpin_work;
		if (work == NULL) {
			seq_printf(m, "No flip due on pipe %c (plane %c)\n",
				   pipe, plane);
		} else {
			if (atomic_read(&work->pending) < INTEL_FLIP_COMPLETE) {
				seq_printf(m, "Flip queued on pipe %c (plane %c)\n",
					   pipe, plane);
			} else {
				seq_printf(m, "Flip pending (waiting for vsync) on pipe %c (plane %c)\n",
					   pipe, plane);
			}
			if (work->enable_stall_check)
				seq_puts(m, "Stall check enabled, ");
			else
				seq_puts(m, "Stall check waiting for page flip ioctl, ");
			seq_printf(m, "%d prepares\n", atomic_read(&work->pending));

			if (work->old_fb_obj) {
				struct drm_i915_gem_object *obj = work->old_fb_obj;
				if (obj)
					seq_printf(m, "Old framebuffer gtt_offset 0x%08lx\n",
						   i915_gem_obj_ggtt_offset(obj));
			}
			if (work->pending_flip_obj) {
				struct drm_i915_gem_object *obj = work->pending_flip_obj;
				if (obj)
					seq_printf(m, "New framebuffer gtt_offset 0x%08lx\n",
						   i915_gem_obj_ggtt_offset(obj));
			}
		}
		spin_unlock_irqrestore(&dev->event_lock, flags);
	}

	return 0;
}

static int i915_gem_request_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct intel_ring_buffer *ring;
	struct drm_i915_gem_request *gem_request;
	int ret, count, i;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	count = 0;
	for_each_ring(ring, dev_priv, i) {
		if (list_empty(&ring->request_list))
			continue;

		seq_printf(m, "%s requests:\n", ring->name);
		list_for_each_entry(gem_request,
				    &ring->request_list,
				    list) {
			seq_printf(m, "    %d @ %d\n",
				   gem_request->seqno,
				   (int) (jiffies - gem_request->emitted_jiffies));
		}
		count++;
	}
	mutex_unlock(&dev->struct_mutex);

	if (count == 0)
		seq_puts(m, "No requests\n");

	return 0;
}

static void i915_ring_seqno_info(struct seq_file *m,
				 struct intel_ring_buffer *ring)
{
	if (ring->get_seqno) {
		seq_printf(m, "Current sequence (%s): %u\n",
			   ring->name, ring->get_seqno(ring, false));
	}
}

static int i915_gem_seqno_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct intel_ring_buffer *ring;
	int ret, i;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	for_each_ring(ring, dev_priv, i)
		i915_ring_seqno_info(m, ring);

	mutex_unlock(&dev->struct_mutex);

	return 0;
}


static int i915_interrupt_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct intel_ring_buffer *ring;
	int ret, i, pipe;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	if (IS_VALLEYVIEW(dev)) {
		seq_printf(m, "Display IER:\t%08x\n",
			   I915_READ(VLV_IER));
		seq_printf(m, "Display IIR:\t%08x\n",
			   I915_READ(VLV_IIR));
		seq_printf(m, "Display IIR_RW:\t%08x\n",
			   I915_READ(VLV_IIR_RW));
		seq_printf(m, "Display IMR:\t%08x\n",
			   I915_READ(VLV_IMR));
		for_each_pipe(pipe)
			seq_printf(m, "Pipe %c stat:\t%08x\n",
				   pipe_name(pipe),
				   I915_READ(PIPESTAT(pipe)));

		seq_printf(m, "Master IER:\t%08x\n",
			   I915_READ(VLV_MASTER_IER));

		seq_printf(m, "Render IER:\t%08x\n",
			   I915_READ(GTIER));
		seq_printf(m, "Render IIR:\t%08x\n",
			   I915_READ(GTIIR));
		seq_printf(m, "Render IMR:\t%08x\n",
			   I915_READ(GTIMR));

		seq_printf(m, "PM IER:\t\t%08x\n",
			   I915_READ(GEN6_PMIER));
		seq_printf(m, "PM IIR:\t\t%08x\n",
			   I915_READ(GEN6_PMIIR));
		seq_printf(m, "PM IMR:\t\t%08x\n",
			   I915_READ(GEN6_PMIMR));

		seq_printf(m, "Port hotplug:\t%08x\n",
			   I915_READ(PORT_HOTPLUG_EN));
		seq_printf(m, "DPFLIPSTAT:\t%08x\n",
			   I915_READ(VLV_DPFLIPSTAT));
		seq_printf(m, "DPINVGTT:\t%08x\n",
			   I915_READ(DPINVGTT));

	} else if (!HAS_PCH_SPLIT(dev)) {
		seq_printf(m, "Interrupt enable:    %08x\n",
			   I915_READ(IER));
		seq_printf(m, "Interrupt identity:  %08x\n",
			   I915_READ(IIR));
		seq_printf(m, "Interrupt mask:      %08x\n",
			   I915_READ(IMR));
		for_each_pipe(pipe)
			seq_printf(m, "Pipe %c stat:         %08x\n",
				   pipe_name(pipe),
				   I915_READ(PIPESTAT(pipe)));
	} else {
		seq_printf(m, "North Display Interrupt enable:		%08x\n",
			   I915_READ(DEIER));
		seq_printf(m, "North Display Interrupt identity:	%08x\n",
			   I915_READ(DEIIR));
		seq_printf(m, "North Display Interrupt mask:		%08x\n",
			   I915_READ(DEIMR));
		seq_printf(m, "South Display Interrupt enable:		%08x\n",
			   I915_READ(SDEIER));
		seq_printf(m, "South Display Interrupt identity:	%08x\n",
			   I915_READ(SDEIIR));
		seq_printf(m, "South Display Interrupt mask:		%08x\n",
			   I915_READ(SDEIMR));
		seq_printf(m, "Graphics Interrupt enable:		%08x\n",
			   I915_READ(GTIER));
		seq_printf(m, "Graphics Interrupt identity:		%08x\n",
			   I915_READ(GTIIR));
		seq_printf(m, "Graphics Interrupt mask:		%08x\n",
			   I915_READ(GTIMR));
	}
	seq_printf(m, "Interrupts received: %d\n",
		   atomic_read(&dev_priv->irq_received));
	for_each_ring(ring, dev_priv, i) {
		if (IS_GEN6(dev) || IS_GEN7(dev)) {
			seq_printf(m,
				   "Graphics Interrupt mask (%s):	%08x\n",
				   ring->name, I915_READ_IMR(ring));
		}
		i915_ring_seqno_info(m, ring);
	}
	mutex_unlock(&dev->struct_mutex);

	return 0;
}

static int i915_gem_fence_regs_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	int i, ret;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	seq_printf(m, "Reserved fences = %d\n", dev_priv->fence_reg_start);
	seq_printf(m, "Total fences = %d\n", dev_priv->num_fence_regs);
	for (i = 0; i < dev_priv->num_fence_regs; i++) {
		struct drm_i915_gem_object *obj = dev_priv->fence_regs[i].obj;

		seq_printf(m, "Fence %d, pin count = %d, object = ",
			   i, dev_priv->fence_regs[i].pin_count);
		if (obj == NULL)
			seq_puts(m, "unused");
		else
			describe_obj(m, obj);
		seq_putc(m, '\n');
	}

	mutex_unlock(&dev->struct_mutex);
	return 0;
}

static int i915_hws_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct intel_ring_buffer *ring;
	const u32 *hws;
	int i;

	ring = &dev_priv->ring[(uintptr_t)node->info_ent->data];
	hws = ring->status_page.page_addr;
	if (hws == NULL)
		return 0;

	for (i = 0; i < 4096 / sizeof(u32) / 4; i += 4) {
		seq_printf(m, "0x%08x: 0x%08x 0x%08x 0x%08x 0x%08x\n",
			   i * 4,
			   hws[i], hws[i + 1], hws[i + 2], hws[i + 3]);
	}
	return 0;
}

static ssize_t
i915_error_state_write(struct file *filp,
		       const char __user *ubuf,
		       size_t cnt,
		       loff_t *ppos)
{
	struct i915_error_state_file_priv *error_priv = filp->private_data;
	struct drm_device *dev = error_priv->dev;
	int ret;

	DRM_DEBUG_DRIVER("Resetting error state\n");

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	i915_destroy_error_state(dev);
	mutex_unlock(&dev->struct_mutex);

	return cnt;
}

static int i915_error_state_open(struct inode *inode, struct file *file)
{
	struct drm_device *dev = inode->i_private;
	struct i915_error_state_file_priv *error_priv;

	error_priv = kzalloc(sizeof(*error_priv), GFP_KERNEL);
	if (!error_priv)
		return -ENOMEM;

	error_priv->dev = dev;

	i915_error_state_get(dev, error_priv);

	file->private_data = error_priv;

	return 0;
}

static int i915_error_state_release(struct inode *inode, struct file *file)
{
	struct i915_error_state_file_priv *error_priv = file->private_data;

	i915_error_state_put(error_priv);
	kfree(error_priv);

	return 0;
}

static ssize_t i915_error_state_read(struct file *file, char __user *userbuf,
				     size_t count, loff_t *pos)
{
	struct i915_error_state_file_priv *error_priv = file->private_data;
	struct drm_i915_error_state_buf error_str;
	loff_t tmp_pos = 0;
	ssize_t ret_count = 0;
	int ret;

	ret = i915_error_state_buf_init(&error_str, count, *pos);
	if (ret)
		return ret;

	ret = i915_error_state_to_str(&error_str, error_priv);
	if (ret)
		goto out;

	ret_count = simple_read_from_buffer(userbuf, count, &tmp_pos,
					    error_str.buf,
					    error_str.bytes);

	if (ret_count < 0)
		ret = ret_count;
	else
		*pos = error_str.start + ret_count;
out:
	i915_error_state_buf_release(&error_str);
	return ret ?: ret_count;
}

static const struct file_operations i915_error_state_fops = {
	.owner = THIS_MODULE,
	.open = i915_error_state_open,
	.read = i915_error_state_read,
	.write = i915_error_state_write,
	.llseek = default_llseek,
	.release = i915_error_state_release,
};

static int
i915_next_seqno_get(void *data, u64 *val)
{
	struct drm_device *dev = data;
	drm_i915_private_t *dev_priv = dev->dev_private;
	int ret;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	*val = dev_priv->next_seqno;
	mutex_unlock(&dev->struct_mutex);

	return 0;
}

static int
i915_next_seqno_set(void *data, u64 val)
{
	struct drm_device *dev = data;
	int ret;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	ret = i915_gem_set_seqno(dev, val);
	mutex_unlock(&dev->struct_mutex);

	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(i915_next_seqno_fops,
			i915_next_seqno_get, i915_next_seqno_set,
			"0x%llx\n");

static int i915_rstdby_delays(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	u16 crstanddelay;
	int ret;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	crstanddelay = I915_READ16(CRSTANDVID);

	mutex_unlock(&dev->struct_mutex);

	seq_printf(m, "w/ctx: %d, w/o ctx: %d\n", (crstanddelay >> 8) & 0x3f, (crstanddelay & 0x3f));

	return 0;
}

static int i915_cur_delayinfo(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	int ret;

	if (IS_GEN5(dev)) {
		u16 rgvswctl = I915_READ16(MEMSWCTL);
		u16 rgvstat = I915_READ16(MEMSTAT_ILK);

		seq_printf(m, "Requested P-state: %d\n", (rgvswctl >> 8) & 0xf);
		seq_printf(m, "Requested VID: %d\n", rgvswctl & 0x3f);
		seq_printf(m, "Current VID: %d\n", (rgvstat & MEMSTAT_VID_MASK) >>
			   MEMSTAT_VID_SHIFT);
		seq_printf(m, "Current P-state: %d\n",
			   (rgvstat & MEMSTAT_PSTATE_MASK) >> MEMSTAT_PSTATE_SHIFT);
	} else if ((IS_GEN6(dev) || IS_GEN7(dev)) && !IS_VALLEYVIEW(dev)) {
		u32 gt_perf_status = I915_READ(GEN6_GT_PERF_STATUS);
		u32 rp_state_limits = I915_READ(GEN6_RP_STATE_LIMITS);
		u32 rp_state_cap = I915_READ(GEN6_RP_STATE_CAP);
		u32 rpstat, cagf;
		u32 rpupei, rpcurup, rpprevup;
		u32 rpdownei, rpcurdown, rpprevdown;
		int max_freq;

		/* RPSTAT1 is in the GT power well */
		ret = mutex_lock_interruptible(&dev->struct_mutex);
		if (ret)
			return ret;

		gen6_gt_force_wake_get(dev_priv, FORCEWAKE_ALL);

		rpstat = I915_READ(GEN6_RPSTAT1);
		rpupei = I915_READ(GEN6_RP_CUR_UP_EI);
		rpcurup = I915_READ(GEN6_RP_CUR_UP);
		rpprevup = I915_READ(GEN6_RP_PREV_UP);
		rpdownei = I915_READ(GEN6_RP_CUR_DOWN_EI);
		rpcurdown = I915_READ(GEN6_RP_CUR_DOWN);
		rpprevdown = I915_READ(GEN6_RP_PREV_DOWN);
		if (IS_HASWELL(dev))
			cagf = (rpstat & HSW_CAGF_MASK) >> HSW_CAGF_SHIFT;
		else
			cagf = (rpstat & GEN6_CAGF_MASK) >> GEN6_CAGF_SHIFT;
		cagf *= GT_FREQUENCY_MULTIPLIER;

		gen6_gt_force_wake_put(dev_priv, FORCEWAKE_ALL);
		mutex_unlock(&dev->struct_mutex);

		seq_printf(m, "GT_PERF_STATUS: 0x%08x\n", gt_perf_status);
		seq_printf(m, "RPSTAT1: 0x%08x\n", rpstat);
		seq_printf(m, "Render p-state ratio: %d\n",
			   (gt_perf_status & 0xff00) >> 8);
		seq_printf(m, "Render p-state VID: %d\n",
			   gt_perf_status & 0xff);
		seq_printf(m, "Render p-state limit: %d\n",
			   rp_state_limits & 0xff);
		seq_printf(m, "CAGF: %dMHz\n", cagf);
		seq_printf(m, "RP CUR UP EI: %dus\n", rpupei &
			   GEN6_CURICONT_MASK);
		seq_printf(m, "RP CUR UP: %dus\n", rpcurup &
			   GEN6_CURBSYTAVG_MASK);
		seq_printf(m, "RP PREV UP: %dus\n", rpprevup &
			   GEN6_CURBSYTAVG_MASK);
		seq_printf(m, "RP CUR DOWN EI: %dus\n", rpdownei &
			   GEN6_CURIAVG_MASK);
		seq_printf(m, "RP CUR DOWN: %dus\n", rpcurdown &
			   GEN6_CURBSYTAVG_MASK);
		seq_printf(m, "RP PREV DOWN: %dus\n", rpprevdown &
			   GEN6_CURBSYTAVG_MASK);

		max_freq = (rp_state_cap & 0xff0000) >> 16;
		seq_printf(m, "Lowest (RPN) frequency: %dMHz\n",
			   max_freq * GT_FREQUENCY_MULTIPLIER);

		max_freq = (rp_state_cap & 0xff00) >> 8;
		seq_printf(m, "Nominal (RP1) frequency: %dMHz\n",
			   max_freq * GT_FREQUENCY_MULTIPLIER);

		max_freq = rp_state_cap & 0xff;
		seq_printf(m, "Max non-overclocked (RP0) frequency: %dMHz\n",
			   max_freq * GT_FREQUENCY_MULTIPLIER);

		seq_printf(m, "Max overclocked frequency: %dMHz\n",
			   dev_priv->rps.hw_max * GT_FREQUENCY_MULTIPLIER);
	} else if (IS_VALLEYVIEW(dev)) {
		u32 freq_sts;

		mutex_lock(&dev_priv->rps.hw_lock);
		freq_sts = vlv_punit_read(dev_priv, PUNIT_REG_GPU_FREQ_STS);
		seq_printf(m, "PUNIT_REG_GPU_FREQ_STS: 0x%08x\n", freq_sts);
		seq_printf(m, "DDR freq: %d MHz\n", dev_priv->mem_freq);

		seq_printf(m, "Max HW Supported GPU freq: %d MHz (%u)\n",
			vlv_gpu_freq(dev_priv,
				dev_priv->rps.hw_max),
				dev_priv->rps.hw_max);
		seq_printf(m, "Min HW Supported GPU freq: %d MHz (%u)\n",
			vlv_gpu_freq(dev_priv,
					dev_priv->rps.hw_min),
					dev_priv->rps.hw_min);

		seq_printf(m, "Max User Selected GPU freq: %d MHz (%u)\n",
			vlv_gpu_freq(dev_priv,
				dev_priv->rps.max_delay),
				dev_priv->rps.max_delay);
		seq_printf(m, "Min User Selected GPU freq: %d MHz (%u)\n",
			vlv_gpu_freq(dev_priv,
					dev_priv->rps.min_delay),
					dev_priv->rps.min_delay);

		seq_printf(m, "Current GPU freq: %d MHz (%u)\n",
			vlv_gpu_freq(dev_priv,
					dev_priv->rps.cur_delay),
					dev_priv->rps.cur_delay);
		seq_printf(m, "Last Requested Gpu freq: %d MHz (%u)\n",
			vlv_gpu_freq(dev_priv,
					dev_priv->rps.requested_delay),
					dev_priv->rps.requested_delay);
		seq_printf(m, "Up Threshold: %d\n",
		atomic_read(&dev_priv->turbodebug.up_threshold));
		seq_printf(m, "Down Threshold: %d\n",
		atomic_read(&dev_priv->turbodebug.down_threshold));
		seq_printf(m, "RP_UP: %d\nRP_DOWN:%d\n",
					dev_priv->rps.rp_up_masked,
					dev_priv->rps.rp_down_masked);
		mutex_unlock(&dev_priv->rps.hw_lock);
	} else {
		seq_puts(m, "no P-state info available\n");
	}

	return 0;
}

static int i915_delayfreq_table(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	u32 delayfreq;
	int ret, i;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	for (i = 0; i < 16; i++) {
		delayfreq = I915_READ(PXVFREQ_BASE + i * 4);
		seq_printf(m, "P%02dVIDFREQ: 0x%08x (VID: %d)\n", i, delayfreq,
			   (delayfreq & PXVFREQ_PX_MASK) >> PXVFREQ_PX_SHIFT);
	}

	mutex_unlock(&dev->struct_mutex);

	return 0;
}

static inline int MAP_TO_MV(int map)
{
	return 1250 - (map * 25);
}

static int i915_inttoext_table(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	u32 inttoext;
	int ret, i;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	for (i = 1; i <= 32; i++) {
		inttoext = I915_READ(INTTOEXT_BASE_ILK + i * 4);
		seq_printf(m, "INTTOEXT%02d: 0x%08x\n", i, inttoext);
	}

	mutex_unlock(&dev->struct_mutex);

	return 0;
}

static int ironlake_drpc_info(struct seq_file *m)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	u32 rgvmodectl, rstdbyctl;
	u16 crstandvid;
	int ret;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	rgvmodectl = I915_READ(MEMMODECTL);
	rstdbyctl = I915_READ(RSTDBYCTL);
	crstandvid = I915_READ16(CRSTANDVID);

	mutex_unlock(&dev->struct_mutex);

	seq_printf(m, "HD boost: %s\n", (rgvmodectl & MEMMODE_BOOST_EN) ?
		   "yes" : "no");
	seq_printf(m, "Boost freq: %d\n",
		   (rgvmodectl & MEMMODE_BOOST_FREQ_MASK) >>
		   MEMMODE_BOOST_FREQ_SHIFT);
	seq_printf(m, "HW control enabled: %s\n",
		   rgvmodectl & MEMMODE_HWIDLE_EN ? "yes" : "no");
	seq_printf(m, "SW control enabled: %s\n",
		   rgvmodectl & MEMMODE_SWMODE_EN ? "yes" : "no");
	seq_printf(m, "Gated voltage change: %s\n",
		   rgvmodectl & MEMMODE_RCLK_GATE ? "yes" : "no");
	seq_printf(m, "Starting frequency: P%d\n",
		   (rgvmodectl & MEMMODE_FSTART_MASK) >> MEMMODE_FSTART_SHIFT);
	seq_printf(m, "Max P-state: P%d\n",
		   (rgvmodectl & MEMMODE_FMAX_MASK) >> MEMMODE_FMAX_SHIFT);
	seq_printf(m, "Min P-state: P%d\n", (rgvmodectl & MEMMODE_FMIN_MASK));
	seq_printf(m, "RS1 VID: %d\n", (crstandvid & 0x3f));
	seq_printf(m, "RS2 VID: %d\n", ((crstandvid >> 8) & 0x3f));
	seq_printf(m, "Render standby enabled: %s\n",
		   (rstdbyctl & RCX_SW_EXIT) ? "no" : "yes");
	seq_puts(m, "Current RS state: ");
	switch (rstdbyctl & RSX_STATUS_MASK) {
	case RSX_STATUS_ON:
		seq_puts(m, "on\n");
		break;
	case RSX_STATUS_RC1:
		seq_puts(m, "RC1\n");
		break;
	case RSX_STATUS_RC1E:
		seq_puts(m, "RC1E\n");
		break;
	case RSX_STATUS_RS1:
		seq_puts(m, "RS1\n");
		break;
	case RSX_STATUS_RS2:
		seq_puts(m, "RS2 (RC6)\n");
		break;
	case RSX_STATUS_RS3:
		seq_puts(m, "RC3 (RC6+)\n");
		break;
	default:
		seq_puts(m, "unknown\n");
		break;
	}

	return 0;
}

static int gen6_drpc_info(struct seq_file *m)
{

	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 rpmodectl1, gt_core_status, rcctl1, rc6vids = 0;
	unsigned forcewake_count;
	int count = 0, ret;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	spin_lock_irq(&dev_priv->uncore.lock);
	forcewake_count = dev_priv->uncore.forcewake_count;
	spin_unlock_irq(&dev_priv->uncore.lock);

	if (forcewake_count) {
		seq_puts(m, "RC information inaccurate because somebody "
			    "holds a forcewake reference \n");
	} else {
		/* NB: we cannot use forcewake, else we read the wrong values */
		while (count++ < 50 && (I915_READ_NOTRACE(FORCEWAKE_ACK) & 1))
			udelay(10);
		seq_printf(m, "RC information accurate: %s\n", yesno(count < 51));
	}

	gt_core_status = readl(dev_priv->regs + GEN6_GT_CORE_STATUS);
	trace_i915_reg_rw(false, GEN6_GT_CORE_STATUS, gt_core_status, 4, true);

	rpmodectl1 = I915_READ(GEN6_RP_CONTROL);
	rcctl1 = I915_READ(GEN6_RC_CONTROL);
	mutex_unlock(&dev->struct_mutex);
	mutex_lock(&dev_priv->rps.hw_lock);
	sandybridge_pcode_read(dev_priv, GEN6_PCODE_READ_RC6VIDS, &rc6vids);
	mutex_unlock(&dev_priv->rps.hw_lock);

	seq_printf(m, "Video Turbo Mode: %s\n",
		   yesno(rpmodectl1 & GEN6_RP_MEDIA_TURBO));
	seq_printf(m, "HW control enabled: %s\n",
		   yesno(rpmodectl1 & GEN6_RP_ENABLE));
	seq_printf(m, "SW control enabled: %s\n",
		   yesno((rpmodectl1 & GEN6_RP_MEDIA_MODE_MASK) ==
			  GEN6_RP_MEDIA_SW_MODE));
	seq_printf(m, "RC1e Enabled: %s\n",
		   yesno(rcctl1 & GEN6_RC_CTL_RC1e_ENABLE));
	seq_printf(m, "RC6 Enabled: %s\n",
		   yesno(rcctl1 & GEN6_RC_CTL_RC6_ENABLE));
	seq_printf(m, "Deep RC6 Enabled: %s\n",
		   yesno(rcctl1 & GEN6_RC_CTL_RC6p_ENABLE));
	seq_printf(m, "Deepest RC6 Enabled: %s\n",
		   yesno(rcctl1 & GEN6_RC_CTL_RC6pp_ENABLE));
	seq_puts(m, "Current RC state: ");
	switch (gt_core_status & GEN6_RCn_MASK) {
	case GEN6_RC0:
		if (gt_core_status & GEN6_CORE_CPD_STATE_MASK)
			seq_puts(m, "Core Power Down\n");
		else
			seq_puts(m, "on\n");
		break;
	case GEN6_RC3:
		seq_puts(m, "RC3\n");
		break;
	case GEN6_RC6:
		seq_puts(m, "RC6\n");
		break;
	case GEN6_RC7:
		seq_puts(m, "RC7\n");
		break;
	default:
		seq_puts(m, "Unknown\n");
		break;
	}

	seq_printf(m, "Core Power Down: %s\n",
		   yesno(gt_core_status & GEN6_CORE_CPD_STATE_MASK));

	/* Not exactly sure what this is */
	seq_printf(m, "RC6 \"Locked to RPn\" residency since boot: %u\n",
		   I915_READ(GEN6_GT_GFX_RC6_LOCKED));
	seq_printf(m, "RC6 residency since boot: %u\n",
		   I915_READ(GEN6_GT_GFX_RC6));
	seq_printf(m, "RC6+ residency since boot: %u\n",
		   I915_READ(GEN6_GT_GFX_RC6p));
	seq_printf(m, "RC6++ residency since boot: %u\n",
		   I915_READ(GEN6_GT_GFX_RC6pp));

	seq_printf(m, "RC6   voltage: %dmV\n",
		   GEN6_DECODE_RC6_VID(((rc6vids >> 0) & 0xff)));
	seq_printf(m, "RC6+  voltage: %dmV\n",
		   GEN6_DECODE_RC6_VID(((rc6vids >> 8) & 0xff)));
	seq_printf(m, "RC6++ voltage: %dmV\n",
		   GEN6_DECODE_RC6_VID(((rc6vids >> 16) & 0xff)));
	return 0;
}

static int i915_drpc_info(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;

	if (IS_GEN6(dev) || IS_GEN7(dev))
		return gen6_drpc_info(m);
	else
		return ironlake_drpc_info(m);
}

static int i915_fbc_status(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;

	if (!I915_HAS_FBC(dev)) {
		seq_puts(m, "FBC unsupported on this chipset\n");
		return 0;
	}

	if (intel_fbc_enabled(dev)) {
		seq_puts(m, "FBC enabled\n");
	} else {
		seq_puts(m, "FBC disabled: ");
		switch (dev_priv->fbc.no_fbc_reason) {
		case FBC_OK:
			seq_puts(m, "FBC actived, but currently disabled in hardware");
			break;
		case FBC_UNSUPPORTED:
			seq_puts(m, "unsupported by this chipset");
			break;
		case FBC_NO_OUTPUT:
			seq_puts(m, "no outputs");
			break;
		case FBC_STOLEN_TOO_SMALL:
			seq_puts(m, "not enough stolen memory");
			break;
		case FBC_UNSUPPORTED_MODE:
			seq_puts(m, "mode not supported");
			break;
		case FBC_MODE_TOO_LARGE:
			seq_puts(m, "mode too large");
			break;
		case FBC_BAD_PLANE:
			seq_puts(m, "FBC unsupported on plane");
			break;
		case FBC_NOT_TILED:
			seq_puts(m, "scanout buffer not tiled");
			break;
		case FBC_MULTIPLE_PIPES:
			seq_puts(m, "multiple pipes are enabled");
			break;
		case FBC_MODULE_PARAM:
			seq_puts(m, "disabled per module param (default off)");
			break;
		case FBC_CHIP_DEFAULT:
			seq_puts(m, "disabled per chip default");
			break;
		default:
			seq_puts(m, "unknown reason");
		}
		seq_putc(m, '\n');
	}
	return 0;
}

static int i915_ips_status(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (!HAS_IPS(dev)) {
		seq_puts(m, "not supported\n");
		return 0;
	}

	if (I915_READ(IPS_CTL) & IPS_ENABLE)
		seq_puts(m, "enabled\n");
	else
		seq_puts(m, "disabled\n");

	return 0;
}

static int i915_sr_status(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	bool sr_enabled = false;

	if (HAS_PCH_SPLIT(dev))
		sr_enabled = I915_READ(WM1_LP_ILK) & WM1_LP_SR_EN;
	else if (IS_CRESTLINE(dev) || IS_I945G(dev) || IS_I945GM(dev))
		sr_enabled = I915_READ(FW_BLC_SELF) & FW_BLC_SELF_EN;
	else if (IS_I915GM(dev))
		sr_enabled = I915_READ(INSTPM) & INSTPM_SELF_EN;
	else if (IS_PINEVIEW(dev))
		sr_enabled = I915_READ(DSPFW3) & PINEVIEW_SELF_REFRESH_EN;

	seq_printf(m, "self-refresh: %s\n",
		   sr_enabled ? "enabled" : "disabled");

	return 0;
}

static int i915_emon_status(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	unsigned long temp, chipset, gfx;
	int ret;

	if (!IS_GEN5(dev))
		return -ENODEV;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	temp = i915_mch_val(dev_priv);
	chipset = i915_chipset_val(dev_priv);
	gfx = i915_gfx_val(dev_priv);
	mutex_unlock(&dev->struct_mutex);

	seq_printf(m, "GMCH temp: %ld\n", temp);
	seq_printf(m, "Chipset power: %ld\n", chipset);
	seq_printf(m, "GFX power: %ld\n", gfx);
	seq_printf(m, "Total power: %ld\n", chipset + gfx);

	return 0;
}

static int i915_ring_freq_table(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	int ret;
	int gpu_freq, ia_freq;

	if (!(IS_GEN6(dev) || IS_GEN7(dev))) {
		seq_puts(m, "unsupported on this chipset\n");
		return 0;
	}

	ret = mutex_lock_interruptible(&dev_priv->rps.hw_lock);
	if (ret)
		return ret;

	seq_puts(m, "GPU freq (MHz)\tEffective CPU freq (MHz)\tEffective Ring freq (MHz)\n");

	for (gpu_freq = dev_priv->rps.min_delay;
	     gpu_freq <= dev_priv->rps.max_delay;
	     gpu_freq++) {
		ia_freq = gpu_freq;
		sandybridge_pcode_read(dev_priv,
				       GEN6_PCODE_READ_MIN_FREQ_TABLE,
				       &ia_freq);
		seq_printf(m, "%d\t\t%d\t\t\t\t%d\n",
			   gpu_freq * GT_FREQUENCY_MULTIPLIER,
			   ((ia_freq >> 0) & 0xff) * 100,
			   ((ia_freq >> 8) & 0xff) * 100);
	}

	mutex_unlock(&dev_priv->rps.hw_lock);

	return 0;
}

static int i915_gfxec(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	int ret;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	seq_printf(m, "GFXEC: %ld\n", (unsigned long)I915_READ(0x112f4));

	mutex_unlock(&dev->struct_mutex);

	return 0;
}

static int i915_opregion(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct intel_opregion *opregion = &dev_priv->opregion;
	void *data = kmalloc(OPREGION_SIZE, GFP_KERNEL);
	int ret;

	if (data == NULL)
		return -ENOMEM;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		goto out;

	if (opregion->header) {
		memcpy_fromio(data, opregion->header, OPREGION_SIZE);
		seq_write(m, data, OPREGION_SIZE);
	}

	mutex_unlock(&dev->struct_mutex);

out:
	kfree(data);
	return 0;
}

static int i915_gem_framebuffer_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct intel_fbdev *ifbdev;
	struct intel_framebuffer *fb;
	int ret;

	ret = mutex_lock_interruptible(&dev->mode_config.mutex);
	if (ret)
		return ret;

	ifbdev = dev_priv->fbdev;
	fb = to_intel_framebuffer(ifbdev->helper.fb);

	seq_printf(m, "fbcon size: %d x %d, depth %d, %d bpp, refcount %d, obj ",
		   fb->base.width,
		   fb->base.height,
		   fb->base.depth,
		   fb->base.bits_per_pixel,
		   atomic_read(&fb->base.refcount.refcount));
	describe_obj(m, fb->obj);
	seq_putc(m, '\n');
	mutex_unlock(&dev->mode_config.mutex);

	mutex_lock(&dev->mode_config.fb_lock);
	list_for_each_entry(fb, &dev->mode_config.fb_list, base.head) {
		if (&fb->base == ifbdev->helper.fb)
			continue;

		seq_printf(m, "user size: %d x %d, depth %d, %d bpp, refcount %d, obj ",
			   fb->base.width,
			   fb->base.height,
			   fb->base.depth,
			   fb->base.bits_per_pixel,
			   atomic_read(&fb->base.refcount.refcount));
		describe_obj(m, fb->obj);
		seq_putc(m, '\n');
	}
	mutex_unlock(&dev->mode_config.fb_lock);

	return 0;
}

static int i915_context_status(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct intel_ring_buffer *ring;
	int ret, i;

	ret = mutex_lock_interruptible(&dev->mode_config.mutex);
	if (ret)
		return ret;

	if (dev_priv->ips.pwrctx) {
		seq_puts(m, "power context ");
		describe_obj(m, dev_priv->ips.pwrctx);
		seq_putc(m, '\n');
	}

	if (dev_priv->ips.renderctx) {
		seq_puts(m, "render context ");
		describe_obj(m, dev_priv->ips.renderctx);
		seq_putc(m, '\n');
	}

	for_each_ring(ring, dev_priv, i) {
		if (ring->default_context) {
			seq_printf(m, "HW default context %s ring ", ring->name);
			describe_obj(m, ring->default_context->obj);
			seq_putc(m, '\n');
		}
	}

	mutex_unlock(&dev->mode_config.mutex);

	return 0;
}

static int i915_gen6_forcewake_count_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	unsigned forcewake_count = 0;
	unsigned fw_rendercount = 0;
	unsigned fw_mediacount = 0;

	spin_lock_irq(&dev_priv->uncore.lock);
	if (IS_VALLEYVIEW(dev)) {
		fw_rendercount = dev_priv->uncore.fw_rendercount;
		fw_mediacount = dev_priv->uncore.fw_mediacount;
	} else
		forcewake_count = dev_priv->uncore.forcewake_count;
	spin_unlock_irq(&dev_priv->uncore.lock);

	if (IS_VALLEYVIEW(dev)) {
		seq_printf(m, "fw_rendercount = %u\n", fw_rendercount);
		seq_printf(m, "fw_mediacount = %u\n", fw_mediacount);
	} else
		seq_printf(m, "forcewake count = %u\n", forcewake_count);

	return 0;
}

static const char *swizzle_string(unsigned swizzle)
{
	switch (swizzle) {
	case I915_BIT_6_SWIZZLE_NONE:
		return "none";
	case I915_BIT_6_SWIZZLE_9:
		return "bit9";
	case I915_BIT_6_SWIZZLE_9_10:
		return "bit9/bit10";
	case I915_BIT_6_SWIZZLE_9_11:
		return "bit9/bit11";
	case I915_BIT_6_SWIZZLE_9_10_11:
		return "bit9/bit10/bit11";
	case I915_BIT_6_SWIZZLE_9_17:
		return "bit9/bit17";
	case I915_BIT_6_SWIZZLE_9_10_17:
		return "bit9/bit10/bit17";
	case I915_BIT_6_SWIZZLE_UNKNOWN:
		return "unknown";
	}

	return "bug";
}

static int i915_swizzle_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int ret;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	seq_printf(m, "bit6 swizzle for X-tiling = %s\n",
		   swizzle_string(dev_priv->mm.bit_6_swizzle_x));
	seq_printf(m, "bit6 swizzle for Y-tiling = %s\n",
		   swizzle_string(dev_priv->mm.bit_6_swizzle_y));

	if (IS_GEN3(dev) || IS_GEN4(dev)) {
		seq_printf(m, "DDC = 0x%08x\n",
			   I915_READ(DCC));
		seq_printf(m, "C0DRB3 = 0x%04x\n",
			   I915_READ16(C0DRB3));
		seq_printf(m, "C1DRB3 = 0x%04x\n",
			   I915_READ16(C1DRB3));
	} else if (IS_GEN6(dev) || IS_GEN7(dev)) {
		seq_printf(m, "MAD_DIMM_C0 = 0x%08x\n",
			   I915_READ(MAD_DIMM_C0));
		seq_printf(m, "MAD_DIMM_C1 = 0x%08x\n",
			   I915_READ(MAD_DIMM_C1));
		seq_printf(m, "MAD_DIMM_C2 = 0x%08x\n",
			   I915_READ(MAD_DIMM_C2));
		seq_printf(m, "TILECTL = 0x%08x\n",
			   I915_READ(TILECTL));
		seq_printf(m, "ARB_MODE = 0x%08x\n",
			   I915_READ(ARB_MODE));
		seq_printf(m, "DISP_ARB_CTL = 0x%08x\n",
			   I915_READ(DISP_ARB_CTL));
	}
	mutex_unlock(&dev->struct_mutex);

	return 0;
}

static int i915_ppgtt_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_ring_buffer *ring;
	int i, ret;


	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;
	if (INTEL_INFO(dev)->gen == 6)
		seq_printf(m, "GFX_MODE: 0x%08x\n", I915_READ(GFX_MODE));

	for_each_ring(ring, dev_priv, i) {
		seq_printf(m, "%s\n", ring->name);
		if (INTEL_INFO(dev)->gen == 7)
			seq_printf(m, "GFX_MODE: 0x%08x\n", I915_READ(RING_MODE_GEN7(ring)));
		seq_printf(m, "PP_DIR_BASE: 0x%08x\n", I915_READ(RING_PP_DIR_BASE(ring)));
		seq_printf(m, "PP_DIR_BASE_READ: 0x%08x\n", I915_READ(RING_PP_DIR_BASE_READ(ring)));
		seq_printf(m, "PP_DIR_DCLV: 0x%08x\n", I915_READ(RING_PP_DIR_DCLV(ring)));
	}
	if (dev_priv->mm.aliasing_ppgtt) {
		struct i915_hw_ppgtt *ppgtt = dev_priv->mm.aliasing_ppgtt;

		seq_puts(m, "aliasing PPGTT:\n");
		seq_printf(m, "pd gtt offset: 0x%08x\n", ppgtt->pd_offset);
	}
	seq_printf(m, "ECOCHK: 0x%08x\n", I915_READ(GAM_ECOCHK));
	mutex_unlock(&dev->struct_mutex);

	return 0;
}

static int i915_dpio_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int ret;


	if (!IS_VALLEYVIEW(dev)) {
		seq_puts(m, "unsupported\n");
		return 0;
	}

	ret = mutex_lock_interruptible(&dev_priv->dpio_lock);
	if (ret)
		return ret;

	seq_printf(m, "DPIO_CTL: 0x%08x\n", I915_READ(DPIO_CTL));

	seq_printf(m, "DPIO_DIV_A: 0x%08x\n",
		   vlv_dpio_read(dev_priv, _DPIO_DIV_A));
	seq_printf(m, "DPIO_DIV_B: 0x%08x\n",
		   vlv_dpio_read(dev_priv, _DPIO_DIV_B));

	seq_printf(m, "DPIO_REFSFR_A: 0x%08x\n",
		   vlv_dpio_read(dev_priv, _DPIO_REFSFR_A));
	seq_printf(m, "DPIO_REFSFR_B: 0x%08x\n",
		   vlv_dpio_read(dev_priv, _DPIO_REFSFR_B));

	seq_printf(m, "DPIO_CORE_CLK_A: 0x%08x\n",
		   vlv_dpio_read(dev_priv, _DPIO_CORE_CLK_A));
	seq_printf(m, "DPIO_CORE_CLK_B: 0x%08x\n",
		   vlv_dpio_read(dev_priv, _DPIO_CORE_CLK_B));

	seq_printf(m, "DPIO_LPF_COEFF_A: 0x%08x\n",
		   vlv_dpio_read(dev_priv, _DPIO_LPF_COEFF_A));
	seq_printf(m, "DPIO_LPF_COEFF_B: 0x%08x\n",
		   vlv_dpio_read(dev_priv, _DPIO_LPF_COEFF_B));

	seq_printf(m, "DPIO_FASTCLK_DISABLE: 0x%08x\n",
		   vlv_dpio_read(dev_priv, DPIO_FASTCLK_DISABLE));

	mutex_unlock(&dev_priv->dpio_lock);

	return 0;
}

static int i915_llc(struct seq_file *m, void *data)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	/* Size calculation for LLC is a bit of a pain. Ignore for now. */
	seq_printf(m, "LLC: %s\n", yesno(HAS_LLC(dev)));
	seq_printf(m, "eLLC: %zuMB\n", dev_priv->ellc_size);

	return 0;
}

static int i915_edp_psr_status(struct seq_file *m, void *data)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 psrstat, psrperf;

	if (!IS_HASWELL(dev)) {
		seq_puts(m, "PSR not supported on this platform\n");
	} else if (IS_HASWELL(dev) && I915_READ(EDP_PSR_CTL) & EDP_PSR_ENABLE) {
		seq_puts(m, "PSR enabled\n");
	} else {
		seq_puts(m, "PSR disabled: ");
		switch (dev_priv->no_psr_reason) {
		case PSR_NO_SOURCE:
			seq_puts(m, "not supported on this platform");
			break;
		case PSR_NO_SINK:
			seq_puts(m, "not supported by panel");
			break;
		case PSR_MODULE_PARAM:
			seq_puts(m, "disabled by flag");
			break;
		case PSR_CRTC_NOT_ACTIVE:
			seq_puts(m, "crtc not active");
			break;
		case PSR_PWR_WELL_ENABLED:
			seq_puts(m, "power well enabled");
			break;
		case PSR_NOT_TILED:
			seq_puts(m, "not tiled");
			break;
		case PSR_SPRITE_ENABLED:
			seq_puts(m, "sprite enabled");
			break;
		case PSR_S3D_ENABLED:
			seq_puts(m, "stereo 3d enabled");
			break;
		case PSR_INTERLACED_ENABLED:
			seq_puts(m, "interlaced enabled");
			break;
		case PSR_HSW_NOT_DDIA:
			seq_puts(m, "HSW ties PSR to DDI A (eDP)");
			break;
		default:
			seq_puts(m, "unknown reason");
		}
		seq_puts(m, "\n");
		return 0;
	}

	psrstat = I915_READ(EDP_PSR_STATUS_CTL);

	seq_puts(m, "PSR Current State: ");
	switch (psrstat & EDP_PSR_STATUS_STATE_MASK) {
	case EDP_PSR_STATUS_STATE_IDLE:
		seq_puts(m, "Reset state\n");
		break;
	case EDP_PSR_STATUS_STATE_SRDONACK:
		seq_puts(m, "Wait for TG/Stream to send on frame of data after SRD conditions are met\n");
		break;
	case EDP_PSR_STATUS_STATE_SRDENT:
		seq_puts(m, "SRD entry\n");
		break;
	case EDP_PSR_STATUS_STATE_BUFOFF:
		seq_puts(m, "Wait for buffer turn off\n");
		break;
	case EDP_PSR_STATUS_STATE_BUFON:
		seq_puts(m, "Wait for buffer turn on\n");
		break;
	case EDP_PSR_STATUS_STATE_AUXACK:
		seq_puts(m, "Wait for AUX to acknowledge on SRD exit\n");
		break;
	case EDP_PSR_STATUS_STATE_SRDOFFACK:
		seq_puts(m, "Wait for TG/Stream to acknowledge the SRD VDM exit\n");
		break;
	default:
		seq_puts(m, "Unknown\n");
		break;
	}

	seq_puts(m, "Link Status: ");
	switch (psrstat & EDP_PSR_STATUS_LINK_MASK) {
	case EDP_PSR_STATUS_LINK_FULL_OFF:
		seq_puts(m, "Link is fully off\n");
		break;
	case EDP_PSR_STATUS_LINK_FULL_ON:
		seq_puts(m, "Link is fully on\n");
		break;
	case EDP_PSR_STATUS_LINK_STANDBY:
		seq_puts(m, "Link is in standby\n");
		break;
	default:
		seq_puts(m, "Unknown\n");
		break;
	}

	seq_printf(m, "PSR Entry Count: %u\n",
		   psrstat >> EDP_PSR_STATUS_COUNT_SHIFT &
		   EDP_PSR_STATUS_COUNT_MASK);

	seq_printf(m, "Max Sleep Timer Counter: %u\n",
		   psrstat >> EDP_PSR_STATUS_MAX_SLEEP_TIMER_SHIFT &
		   EDP_PSR_STATUS_MAX_SLEEP_TIMER_MASK);

	seq_printf(m, "Had AUX error: %s\n",
		   yesno(psrstat & EDP_PSR_STATUS_AUX_ERROR));

	seq_printf(m, "Sending AUX: %s\n",
		   yesno(psrstat & EDP_PSR_STATUS_AUX_SENDING));

	seq_printf(m, "Sending Idle: %s\n",
		   yesno(psrstat & EDP_PSR_STATUS_SENDING_IDLE));

	seq_printf(m, "Sending TP2 TP3: %s\n",
		   yesno(psrstat & EDP_PSR_STATUS_SENDING_TP2_TP3));

	seq_printf(m, "Sending TP1: %s\n",
		   yesno(psrstat & EDP_PSR_STATUS_SENDING_TP1));

	seq_printf(m, "Idle Count: %u\n",
		   psrstat & EDP_PSR_STATUS_IDLE_MASK);

	psrperf = (I915_READ(EDP_PSR_PERF_CNT)) & EDP_PSR_PERF_CNT_MASK;
	seq_printf(m, "Performance Counter: %u\n", psrperf);

	return 0;
}

static int i915_energy_uJ(struct seq_file *m, void *data)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u64 power;
	u32 units;

	if (INTEL_INFO(dev)->gen < 6)
		return -ENODEV;

	rdmsrl(MSR_RAPL_POWER_UNIT, power);
	power = (power & 0x1f00) >> 8;
	units = 1000000 / (1 << power); /* convert to uJ */
	power = I915_READ(MCH_SECP_NRG_STTS);
	power *= units;

	seq_printf(m, "%llu", (long long unsigned)power);

	return 0;
}

static int i915_pc8_status(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (!IS_HASWELL(dev)) {
		seq_puts(m, "not supported\n");
		return 0;
	}

	mutex_lock(&dev_priv->pc8.lock);
	seq_printf(m, "Requirements met: %s\n",
		   yesno(dev_priv->pc8.requirements_met));
	seq_printf(m, "GPU idle: %s\n", yesno(dev_priv->pc8.gpu_idle));
	seq_printf(m, "Disable count: %d\n", dev_priv->pc8.disable_count);
	seq_printf(m, "IRQs disabled: %s\n",
		   yesno(dev_priv->pc8.irqs_disabled));
	seq_printf(m, "Enabled: %s\n", yesno(dev_priv->pc8.enabled));
	mutex_unlock(&dev_priv->pc8.lock);

	return 0;
}

static int
i915_wedged_get(void *data, u64 *val)
{
	struct drm_device *dev = data;
	drm_i915_private_t *dev_priv = dev->dev_private;

	*val = atomic_read(&dev_priv->gpu_error.reset_counter);

	return 0;
}

static int
i915_wedged_set(void *data, u64 val)
{
	struct drm_device *dev = data;
	drm_i915_private_t *dev_priv = dev->dev_private;

	/* This triggers a global reset */
	DRM_INFO("Manually setting wedged to %llu\n", val);
	if (val) {
		if (!i915_reset_in_progress(&dev_priv->gpu_error))
			i915_handle_error(dev, NULL, 0);
	}

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(i915_wedged_fops,
			i915_wedged_get, i915_wedged_set,
			"%llu\n");

static int
i915_ring_stop_get(void *data, u64 *val)
{
	struct drm_device *dev = data;
	drm_i915_private_t *dev_priv = dev->dev_private;

	*val = dev_priv->gpu_error.stop_rings;

	return 0;
}

static int
i915_ring_stop_set(void *data, u64 val)
{
	struct drm_device *dev = data;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int ret;

	DRM_DEBUG_DRIVER("Stopping rings 0x%08llx\n", val);

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	dev_priv->gpu_error.stop_rings = val;
	mutex_unlock(&dev->struct_mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(i915_ring_stop_fops,
			i915_ring_stop_get, i915_ring_stop_set,
			"0x%08llx\n");

#define DROP_UNBOUND 0x1
#define DROP_BOUND 0x2
#define DROP_RETIRE 0x4
#define DROP_ACTIVE 0x8
#define DROP_ALL (DROP_UNBOUND | \
		  DROP_BOUND | \
		  DROP_RETIRE | \
		  DROP_ACTIVE)
static int
i915_drop_caches_get(void *data, u64 *val)
{
	*val = DROP_ALL;

	return 0;
}

static int
i915_drop_caches_set(void *data, u64 val)
{
	struct drm_device *dev = data;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_i915_gem_object *obj, *next;
	struct i915_address_space *vm;
	struct i915_vma *vma, *x;
	int ret;

	DRM_DEBUG_DRIVER("Dropping caches: 0x%08llx\n", val);

	/* No need to check and wait for gpu resets, only libdrm auto-restarts
	 * on ioctls on -EAGAIN. */
	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	if (val & DROP_ACTIVE) {
		ret = i915_gpu_idle(dev);
		if (ret)
			goto unlock;
	}

	if (val & (DROP_RETIRE | DROP_ACTIVE))
		i915_gem_retire_requests(dev);

	if (val & DROP_BOUND) {
		list_for_each_entry(vm, &dev_priv->vm_list, global_link) {
			list_for_each_entry_safe(vma, x, &vm->inactive_list,
						 mm_list) {
				if (vma->obj->pin_count)
					continue;

				ret = i915_vma_unbind(vma);
				if (ret)
					goto unlock;
			}
		}
	}

	if (val & DROP_UNBOUND) {
		list_for_each_entry_safe(obj, next, &dev_priv->mm.unbound_list,
					 global_list)
			if (obj->pages_pin_count == 0) {
				ret = i915_gem_object_put_pages(obj);
				if (ret)
					goto unlock;
			}
	}

unlock:
	mutex_unlock(&dev->struct_mutex);

	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(i915_drop_caches_fops,
			i915_drop_caches_get, i915_drop_caches_set,
			"0x%08llx\n");

/* Helper function to set max freq turbo based on input */
static int
i915_set_max_freq(struct drm_device *dev, int val)
{
	int ret;
	u32 hw_max, hw_min;
	drm_i915_private_t *dev_priv = dev->dev_private;

	DRM_DEBUG_DRIVER("Manually setting max freq to %d\n", val);

	ret = mutex_lock_interruptible(&dev_priv->rps.hw_lock);
	if (ret)
		return ret;

	hw_max = dev_priv->rps.hw_max;
	hw_min = dev_priv->rps.hw_min;

	if (val < hw_min || val > hw_max ||
	    val < dev_priv->rps.min_delay) {
		mutex_unlock(&dev_priv->rps.hw_lock);
		return -1;
	}
	if (IS_VALLEYVIEW(dev))
		dev_priv->rps.max_delay = val;
	else
		dev_priv->rps.max_delay = val / 50;
	/*
	 * Turbo will still be enabled, but won't go above the set value.
	 */
	if (dev_priv->rps.cur_delay > val) {
		if (IS_VALLEYVIEW(dev)) {
			valleyview_set_rps(dev, val);
			/* if rps frequency is changed above we need to take
			* care of bringing it down to rpe through rps timer */
			mod_delayed_work(dev_priv->rpswq,
						&dev_priv->rps.vlv_work,
						msecs_to_jiffies(100));
		} else
			gen6_set_rps(dev, val / 50);
	}

	mutex_unlock(&dev_priv->rps.hw_lock);

	return 0;
}
static ssize_t
i915_ring_hangcheck_read(struct file *filp,
			char __user *ubuf,
			size_t max,
			loff_t *ppos)
{
	/* Returns the total number of times the rings
	 * have hung and been reset since boot */
	struct drm_device *dev = filp->private_data;
	drm_i915_private_t *dev_priv = dev->dev_private;
	char buf[200];
	int len;

	len = scnprintf(buf, sizeof(buf),
			"GPU=0x%08X,RCS=0x%08X,VCS=0x%08X,BCS=0x%08X\n,"
			"RCS_T=0x%08x,VCS_T=0x%08x,BCS_T=0x%08x,"
			"RCS_W=0x%08x,VCS_W=0x%08x,BCS_W=0x%08x\n",
			dev_priv->gpu_error.total_resets,
			dev_priv->hangcheck[RCS].total,
			dev_priv->hangcheck[VCS].total,
			dev_priv->hangcheck[BCS].total,
			dev_priv->hangcheck[RCS].tdr_count,
			dev_priv->hangcheck[VCS].tdr_count,
			dev_priv->hangcheck[BCS].tdr_count,
			dev_priv->hangcheck[RCS].watchdog_count,
			dev_priv->hangcheck[VCS].watchdog_count,
			dev_priv->hangcheck[BCS].watchdog_count);

	return simple_read_from_buffer(ubuf, max, ppos, buf, len);
}

static ssize_t
i915_ring_hangcheck_write(struct file *filp,
			const char __user *ubuf,
			size_t cnt,
			loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int ret;
	uint32_t i;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	for (i = 0; i < I915_NUM_RINGS; i++) {
		/* Reset the hangcheck counters */
		dev_priv->hangcheck[i].total = 0;
		dev_priv->hangcheck[i].tdr_count = 0;
		dev_priv->hangcheck[i].watchdog_count = 0;
	}

	dev_priv->gpu_error.total_resets = 0;

	mutex_unlock(&dev->struct_mutex);

	return cnt;
}

static const struct file_operations i915_ring_hangcheck_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = i915_ring_hangcheck_read,
	.write = i915_ring_hangcheck_write,
	.llseek = default_llseek,
};

/* Helper function to enable and disable dpst based on input */
int
i915_dpst_enable_disable(struct drm_device *dev, unsigned int val)
{
	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	/* 1=> Enable DPST, else disable. */
	if (val == 1)
		i915_dpst_enable_hist_interrupt(dev, true);
	else
		i915_dpst_disable_hist_interrupt(dev, true);

	/* Send a fake signal to start the process */
	i915_dpst_irq_handler(dev);

	return 0;
}


static int
i915_dpst_status(struct drm_device *dev, char *buf, int *len)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	*len = scnprintf(buf, MAX_BUFFER_STR_LEN, "DPST Enabled: %s\n",
			yesno(dev_priv->dpst.enabled ? 1 : 0));

	return 0;
}

static int
i915_dpst_irq_count(struct drm_device *dev, char *buf, int *len)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	*len = scnprintf(buf, MAX_BUFFER_STR_LEN, "DPST Interrupt Count: %d\n",
					dev_priv->dpst.num_interrupt);

	return 0;
}

static int
i915_dpst_dump_reg(struct drm_device *dev, char *buf, int *len)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	u32 btgr_data,  hcr_data, bpcr_data, dpst_set_level;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	btgr_data = I915_READ(BLC_HIST_GUARD);
	hcr_data = I915_READ(BLC_HIST_BIN);
	bpcr_data = I915_READ(BLC_PWM_CTL);
	dpst_set_level = I915_READ(BLC_PWM_CTL) & 0xffff;

	*len = scnprintf(buf, MAX_BUFFER_STR_LEN, "IEBTGR: 0x%x & IEHCR: 0x%x",
				btgr_data, hcr_data);

	*len += scnprintf(&buf[*len], (MAX_BUFFER_STR_LEN - *len),
			" & IEBPCR: 0x%x DPST_SET_LEVEL: 0x%x\n",
			 bpcr_data, dpst_set_level);

	return 0;
}

static int
i915_dpst_get_bin_data(struct drm_device *dev, char *buf, int *len)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	int index;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	*len = scnprintf(buf, MAX_BUFFER_STR_LEN, "Bin Data:\n");

	for (index = 0; index < DPST_BIN_COUNT; index++)
		*len += scnprintf(&buf[*len], (MAX_BUFFER_STR_LEN - *len),
				"%d ", dev_priv->dpst.bin_data[index]);

	*len += scnprintf(&buf[*len], (MAX_BUFFER_STR_LEN - *len), "\n");

	return 0;
}

static int
i915_dpst_get_luma_data(struct drm_device *dev, char *buf, int *len)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	int index;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	*len = scnprintf(buf, MAX_BUFFER_STR_LEN, "LUMA Data:\n");

	for (index = 0; index < DPST_LUMA_COUNT; index++)
		*len += scnprintf(&buf[*len], (MAX_BUFFER_STR_LEN - *len),
				"%d ", dev_priv->dpst.luma_data[index]);

	*len += scnprintf(&buf[*len], (MAX_BUFFER_STR_LEN - *len), "\n");

	return 0;
}


static ssize_t
i915_read_dpst_api(struct file *filp,
			char __user *ubuf,
			size_t max,
			loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	char buf[200], control[10], operation[20], val[20], format[20];
	int len = 0, ret, no_of_tokens;
	u32 dpst_set_level;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	if (i915_debugfs_vars.dpst.dpst_input == 0)
		return len;

	scnprintf(format, sizeof(format), "%%%zus %%%zus %%%zus",
			sizeof(control), sizeof(operation), sizeof(val));

	no_of_tokens = sscanf(i915_debugfs_vars.dpst.dpst_vars,
				format, control, operation, val);
	if (no_of_tokens < 3)
		return len;

	len = sizeof(i915_debugfs_vars.dpst.dpst_vars);

	if (strcmp(operation, STATUS_TOKEN) == 0) {
		ret = i915_dpst_status(dev, buf, &len);
		if (ret)
			return ret;

	} else if (strcmp(operation, ENABLE_TOKEN) == 0) {

		/* 1 => Enable DPST only for video playback scenarios
		 * when source content is 18 bpp, for higher bpp source
		 * content DPST is enabled always */
			ret = i915_dpst_enable_disable(dev, 1);
			if (ret)
				return ret;
		dev_priv->dpst.feature_control = true;
		i915_dpst_status(dev, buf, &len);

	} else if (strcmp(operation, DISABLE_TOKEN) == 0) {

		/* 0 => Disable DPST */
		dev_priv->dpst.feature_control = false;
		ret = i915_dpst_enable_disable(dev, 0);
		if (ret)
			return ret;

		i915_dpst_status(dev, buf, &len);

	} else if (strcmp(operation, DPST_DUMP_REG_TOKEN) == 0) {
		i915_dpst_dump_reg(dev, buf, &len);

	} else if (strcmp(operation, DPST_GET_BIN_DATA_TOKEN) == 0) {
		i915_dpst_get_bin_data(dev, buf, &len);

	} else if (strcmp(operation, DPST_GET_LUMA_DATA_TOKEN) == 0) {
		i915_dpst_get_luma_data(dev, buf, &len);

	} else if (strcmp(operation, DPST_IRQ_COUNT_TOKEN) == 0) {
		i915_dpst_irq_count(dev, buf, &len);

	} else if (strcmp(operation, DPST_FACTOR_TOKEN) == 0) {
		len = scnprintf(buf, sizeof(buf),
				"DPST Backlight Factor: %d\n",
				(dev_priv->dpst.blc_adjustment / 100));

	} else if (strcmp(operation, DPST_LEVEL_TOKEN) == 0) {
		dpst_set_level = I915_READ(BLC_PWM_CTL) & 0xffff;

		len = scnprintf(buf, sizeof(buf),
					"User Applied Backlight Level: 0x%x\n",
					(dev_priv->backlight.level));

		if (len < 0)
			return len;

		if (dev_priv->is_mipi) {

#ifdef CONFIG_CRYSTAL_COVE
			u32 max = intel_panel_get_max_backlight(dev);
			u32 val = 0;
			if (BYT_CR_CONFIG) {
				val = lpio_bl_read(0, LPIO_PWM_CTRL);
				val = (0xff - (val & 0xff)) * max/0xff;
				len += scnprintf(&buf[len], (sizeof(buf) - len),
					"DPST Applied Backlight Level: 0x%x\n", val);
			} else
				len += scnprintf(&buf[len], (sizeof(buf) - len),
					"DPST Applied Backlight Level: 0x%x\n",
					(intel_mid_pmic_readb(0x4E)));
#else
			len += scnprintf(&buf[len], (sizeof(buf) - len),
					"DPST Applied Backlight not supported\n");
#endif
		} else {
			len += scnprintf(&buf[len], (sizeof(buf) - len),
					"DPST Applied Backlight Level: 0x%x\n",
					dpst_set_level);
		}
	} else
		len = scnprintf(buf, sizeof(buf), "NOTSUPPORTED\n");

	i915_debugfs_vars.dpst.dpst_input = 0;
	simple_read_from_buffer(ubuf, max, ppos, buf, len);

	return len;
}


static ssize_t
i915_write_dpst_api(struct file *filp,
			const char __user *ubuf,
			size_t cnt,
			loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	/* Reset the string */
	memset(i915_debugfs_vars.dpst.dpst_vars, 0, MAX_BUFFER_STR_LEN);
	if (cnt > 0) {
		if (cnt > sizeof(i915_debugfs_vars.dpst.dpst_vars) - 1)
			return -EINVAL;
		if (copy_from_user(i915_debugfs_vars.dpst.dpst_vars,
						ubuf, cnt))
			return -EFAULT;
		i915_debugfs_vars.dpst.dpst_vars[cnt] = 0;

		/* Enable read */
		i915_debugfs_vars.dpst.dpst_input = 1;
	}

	return cnt;
}

static const struct file_operations i915_dpst_fops = {
.owner = THIS_MODULE,
.open = simple_open,
.read = i915_read_dpst_api,
.write = i915_write_dpst_api,
.llseek = default_llseek,
};


/* Helper function to set min freq turbo based on input */
static int
i915_set_min_freq(struct drm_device *dev, int val)
{
	int ret;
	u32 hw_max, hw_min;
	drm_i915_private_t *dev_priv = dev->dev_private;

	DRM_DEBUG_DRIVER("Manually setting min freq to %d\n", val);

	ret = mutex_lock_interruptible(&dev_priv->rps.hw_lock);
	if (ret)
		return ret;

	hw_max = dev_priv->rps.hw_max;
	hw_min = dev_priv->rps.hw_min;

	if (val < hw_min || val > hw_max || val > dev_priv->rps.max_delay) {
		mutex_unlock(&dev_priv->rps.hw_lock);
		return -EINVAL;
	}
	if (IS_VALLEYVIEW(dev))
		dev_priv->rps.min_delay = val;
	else
		dev_priv->rps.min_delay = val / 50;
	/*
	 * Turbo will still be enabled, but won't go below the set value.
	 */
	if (dev_priv->rps.cur_delay < val) {
		if (IS_VALLEYVIEW(dev)) {
			valleyview_set_rps(dev, val);
			/* If rps frequency is changed above we need to take
			* care of bringing it down to rpe through rps timer*/
			mod_delayed_work(dev_priv->rpswq,
						&dev_priv->rps.vlv_work,
						msecs_to_jiffies(100));
		} else
			gen6_set_rps(dev, val / 50);
	}

	mutex_unlock(&dev_priv->rps.hw_lock);

	return 0;
}

/* Helper function to enable and disable turbo based on input */
static int
i915_rps_enable_disable(struct drm_device *dev, long unsigned int val)
{
	int ret;
	drm_i915_private_t *dev_priv = dev->dev_private;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	ret = mutex_lock_interruptible(&dev_priv->rps.hw_lock);
	if (ret)
		return ret;

	/* 1=> Enable Turbo, else disable. */

	/* Vlv specific function are not added Yet.*/
	if (val == 1)
		vlv_turbo_initialize(dev);
	else
		vlv_turbo_disable(dev);

	mutex_unlock(&dev_priv->rps.hw_lock);

	return 0;
}

static ssize_t
i915_rps_init_read(struct file *filp, char __user *ubuf, size_t max,
		   loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;
	drm_i915_private_t *dev_priv = dev->dev_private;
	char buf[] = "rps init read is not defined";
	int len;
	u32 rval;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	rval = I915_READ(GEN6_RP_CONTROL);
	len = scnprintf(buf, sizeof(buf),
		       "Turbo Enabled: %s\n", yesno(rval & GEN6_RP_ENABLE));

	return simple_read_from_buffer(ubuf, max, ppos, buf, len);
}


static ssize_t
i915_rps_init_write(struct file *filp, const char __user *ubuf, size_t cnt,
		    loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;
	char buf[20];
	long unsigned int val = 1;
	int ret;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	if (cnt > 0) {
		if (cnt > sizeof(buf) - 1)
			return -EINVAL;

		if (copy_from_user(buf, ubuf, cnt))
			return -EFAULT;

		buf[cnt] = 0;

		ret = kstrtoul(buf, 0, (unsigned long *)&val);
		if (ret)
			return -EINVAL;
	}

	ret = i915_rps_enable_disable(dev, val);
	if (ret)
		return ret;

	return cnt;
}

static const struct file_operations i915_rps_init_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = i915_rps_init_read,
	.write = i915_rps_init_write,
	.llseek = default_llseek,
};

/* Helper function for the rpm related operations */
static int
i915_rpm_enabled(struct drm_device *drm_dev, char *buf, int *len)
{
	struct device *dev = drm_dev->dev;

	if (!(IS_VALLEYVIEW(drm_dev)))
		return -ENODEV;

	*len = scnprintf(buf, MAX_BUFFER_STR_LEN, "RPM Enabled: %s\n",
				yesno(i915_pm_runtime_enabled(dev)));

	return 0;
}

static int
i915_rpm_control(struct drm_device *drm_dev, long unsigned int val)
{
	struct device *dev = drm_dev->dev;

	if (!(IS_VALLEYVIEW(drm_dev)))
		return -ENODEV;

	/* 1=> Enable RPM(D0IX), else disable. */

	if (val == 1)
		i915_rpm_enable(dev);
	else
		i915_rpm_disable(drm_dev);

	return 0;
}

static int
i915_display_pm(struct drm_device *drm_dev, long unsigned int val)
{
	if (!(IS_VALLEYVIEW(drm_dev)))
		return -ENODEV;

	/* 1=> Suspend RPM(D0IX), else Resume. */

	if (val)
		display_runtime_suspend(drm_dev);
	else
		display_runtime_resume(drm_dev);

	return 0;
}


static ssize_t
i915_read_rpm_api(struct file *filp,
		char __user *ubuf,
		size_t max,
		loff_t *ppos)
{
	struct drm_device *drm_dev = filp->private_data;
	char buf[200], control[10], operation[20], val[20], format[20];
	int len = 0, ret, no_of_tokens;

	if (!(IS_VALLEYVIEW(drm_dev)))
		return -ENODEV;

	if (i915_debugfs_vars.rpm.rpm_input == 0)
		return len;

	scnprintf(format, sizeof(format), "%%%zus %%%zus %%%zus",
		sizeof(control), sizeof(operation), sizeof(val));

	no_of_tokens = sscanf(i915_debugfs_vars.rpm.rpm_vars,
				format, control, operation, val);

	if (no_of_tokens < 3)
		return len;

	if (strcmp(operation, STATUS_TOKEN) == 0) {
		ret = i915_rpm_enabled(drm_dev, buf, &len);
		if (ret)
			return ret;

	} else if (strcmp(operation, ENABLE_TOKEN) == 0) {
		/*
		* 1=> Enable RPM, else disable.
		*/
		ret = i915_rpm_control(drm_dev, 1);
		if (ret)
			return ret;

		i915_rpm_enabled(drm_dev, buf, &len);
	} else if (strcmp(operation, DISABLE_TOKEN) == 0) {
		/*
		* 1=> Enable RPM, else disable.
		*/
		ret = i915_rpm_control(drm_dev, 0);
		if (ret)
			return ret;

		i915_rpm_enabled(drm_dev, buf, &len);

	} else if (strcmp(operation, RPM_DISPLAY_RUNTIME_SUSPEND_TOKEN) == 0) {
		/*
		* 1=> Suspend RPM(D0IX), else Resume.
		*/
		ret = i915_display_pm(drm_dev, 1);
		if (ret)
			return ret;

	} else if (strcmp(operation, RPM_DISPLAY_RUNTIME_RESUME_TOKEN) == 0) {
		/*
		* 1=> Suspend RPM(D0IX), else Resume.
		*/
		ret = i915_display_pm(drm_dev, 0);
		if (ret)
			return ret;
	} else
		len = scnprintf(buf, sizeof(buf), "NOTSUPPORTED\n");

	i915_debugfs_vars.rpm.rpm_input = 0;
	simple_read_from_buffer(ubuf, max, ppos, buf, len);

	return len;
}

static ssize_t
i915_write_rpm_api(struct file *filp,
		const char __user *ubuf,
		size_t cnt,
		loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	/* Reset the string */
	memset(i915_debugfs_vars.rpm.rpm_vars, 0, MAX_BUFFER_STR_LEN);

	if (cnt > 0) {
		if (cnt > sizeof(i915_debugfs_vars.rpm.rpm_vars) - 1)
			return -EINVAL;
		if (copy_from_user(i915_debugfs_vars.rpm.rpm_vars, ubuf, cnt))
			return -EFAULT;

		i915_debugfs_vars.rpm.rpm_vars[cnt] = 0;

		/* Enable read */
		i915_debugfs_vars.rpm.rpm_input = 1;
	}

	return cnt;
}

static const struct file_operations i915_rpm_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = i915_read_rpm_api,
	.write = i915_write_rpm_api,
	.llseek = default_llseek,
};

static ssize_t
i915_read_turbo_api(struct file *filp,
		   char __user *ubuf,
		   size_t max,
		   loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;
	drm_i915_private_t *dev_priv = dev->dev_private;
	char buf[200], control[10], operation[20], val[20], format[20];
	int len = 0, ret, no_of_tokens;
	unsigned long pval = 0;
	u32 reg_val = 0;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	if (i915_debugfs_vars.turbo.turbo_input == 0)
		return len;

	snprintf(format, sizeof(format), "%%%zus %%%zus %%%zus",
			sizeof(control), sizeof(operation), sizeof(val));

	no_of_tokens = sscanf(i915_debugfs_vars.turbo.turbo_vars,
				format, control, operation, val);

	if (no_of_tokens < 3)
		return len;

	len = sizeof(i915_debugfs_vars.turbo.turbo_vars);

	if (strcmp(operation, DETAILS_TOKEN) == 0) {
		ret = mutex_lock_interruptible(&dev_priv->rps.hw_lock);
		if (ret)
			return ret;

		reg_val = I915_READ(GEN6_RP_CONTROL);
		len = scnprintf(buf, sizeof(buf),
				"Turbo Enabled: %s\n",
				yesno(reg_val & GEN6_RP_ENABLE));

		if (len < 0)
			return len;

		len += scnprintf(&buf[len], (sizeof(buf) - len),
				"Max Gpu Freq _max_delay_: %d\n",
				dev_priv->rps.max_delay);
		len += scnprintf(&buf[len], (sizeof(buf) - len),
				"Min Gpu Freq _min_delay_: %d\n",
				dev_priv->rps.min_delay);

		reg_val = vlv_punit_read(dev_priv, PUNIT_REG_GPU_FREQ_STS);
		len += scnprintf(&buf[len], (sizeof(buf) - len),
				"Cur Gpu Freq _cur_delay_: %d\n", reg_val >> 8);
		len += scnprintf(&buf[len], (sizeof(buf) - len),
				"Up Threshold: %d\n", atomic_read(
					&dev_priv->turbodebug.up_threshold));
		len += scnprintf(&buf[len], (sizeof(buf) - len),
				"Down Threshold: %d\n",	atomic_read(
					&dev_priv->turbodebug.down_threshold));
		len += scnprintf(&buf[len], (sizeof(buf) - len),
				"RP_UP: %d\nRP_DOWN:%d\n",
				dev_priv->rps.rp_up_masked,
				dev_priv->rps.rp_down_masked);

		mutex_unlock(&dev_priv->rps.hw_lock);

	} else if (strcmp(operation, ENABLE_TOKEN) == 0) {

		/* 1=> Enable Turbo, else disable. */

		ret = i915_rps_enable_disable(dev, 1);
		if (ret)
			return ret;

		len = scnprintf(buf, sizeof(buf),
				"Turbo Enabled: Yes\n");

	} else if (strcmp(operation, DISABLE_TOKEN) == 0) {

		/* 1=> Enable Turbo, else disable. */

		ret = i915_rps_enable_disable(dev, 0);
		if (ret)
			return ret;

		len = scnprintf(buf, sizeof(buf),
				"Turbo Enabled: No\n");

	} else if (strcmp(operation, RP_MAXFREQ_TOKEN) == 0) {

		ret = kstrtoul(val, 0, &pval);
		if (ret)
			return -EINVAL;

		ret = i915_set_max_freq(dev, pval);
		if (ret)
			return ret;

		len = scnprintf(buf, sizeof(buf),
				"OPERATION: SUCCESSFUL\n");
	} else if (strcmp(operation, RP_MINFREQ_TOKEN) == 0) {

		ret = kstrtoul(val, 0, &pval);
		if (ret)
			return -EINVAL;

		ret = i915_set_min_freq(dev, pval);
		if (ret)
			return ret;

		len = scnprintf(buf, sizeof(buf),
				"OPERATION: SUCCESSFUL\n");

	} else
		len = scnprintf(buf, sizeof(buf),
				"NOTSUPPORTED\n");

	i915_debugfs_vars.turbo.turbo_input = 0;
	simple_read_from_buffer(ubuf, max, ppos, buf, len);

	return len;
}

static ssize_t
i915_write_turbo_api(struct file *filp,
		  const char __user *ubuf,
		  size_t cnt,
		  loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	/* Reset the string */
	memset(i915_debugfs_vars.turbo.turbo_vars, 0, MAX_BUFFER_STR_LEN);

	if (cnt > 0) {
		if (cnt > sizeof(i915_debugfs_vars.turbo.turbo_vars) - 1)
			return -EINVAL;

		if (copy_from_user(i915_debugfs_vars.turbo.turbo_vars,
					ubuf, cnt))
			return -EFAULT;

		i915_debugfs_vars.turbo.turbo_vars[cnt] = 0;

		/* Enable read */
		i915_debugfs_vars.turbo.turbo_input = 1;
	}

	return cnt;
}

static const struct file_operations i915_turbo_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = i915_read_turbo_api,
	.write = i915_write_turbo_api,
	.llseek = default_llseek,
};


/* Debugfs rc6 apis implementation */

static inline bool is_rc6_enabled(struct drm_device *dev)
{
	drm_i915_private_t *dev_priv = dev->dev_private;

	return I915_READ(VLV_RENDER_C_STATE_CONTROL_1_REG)
			& (VLV_EVAL_METHOD_ENABLE_BIT
			| VLV_TIMEOUT_METHOD_ENABLE_BIT);
}

static int
rc6_status(struct drm_device *dev, char *buf, int *len)
{
	*len = scnprintf(buf, MAX_BUFFER_STR_LEN,
			"RC6 ENABLED: %s\n",
			yesno(is_rc6_enabled(dev)));
	return 0;
}

static int
rc6_enable_disable(struct drm_device *dev, long unsigned int val)
{
	int ret;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	if (is_rc6_enabled(dev)) {
		if (val > 0)
			return 0;
	} else if (val == 0)
		return 0;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	vlv_rs_setstate(dev, (val > 0 ? true : false));
	mutex_unlock(&dev->struct_mutex);
	DRM_DEBUG_DRIVER("RC6 feature status is %ld\n", val);

	return 0;
}


static ssize_t
i915_read_rc6_api(struct file *filp,
		   char __user *ubuf,
		   size_t max,
		   loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;
	drm_i915_private_t *dev_priv = dev->dev_private;
	char buf[200], control[10], operation[20], format[20];
	int len = 0, ret, no_of_tokens;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	if (i915_debugfs_vars.rc6.rc6_input == 0)
		return len;

	snprintf(format, sizeof(format), "%%%zus %%%zus",
				sizeof(control), sizeof(operation));

	no_of_tokens = sscanf(i915_debugfs_vars.rc6.rc6_vars,
					format, control, operation);

	if (no_of_tokens < 2)
		return len;

	len = sizeof(i915_debugfs_vars.rc6.rc6_vars);

	if (strcmp(operation, STATUS_TOKEN) == 0) {
		rc6_status(dev, buf, &len);

	} else if (strcmp(operation, ENABLE_TOKEN) == 0) {
		/*
		* 1=> Enable RC6, else disable.
		*/
		ret = rc6_enable_disable(dev, 1);
		if (ret)
			return ret;

		rc6_status(dev, buf, &len);

	} else if (strcmp(operation, DISABLE_TOKEN) == 0) {
		/*
		* 1=> Enable RC6, else disable.
		*/
		ret = rc6_enable_disable(dev, 0);
		if (ret)
			return ret;

		rc6_status(dev, buf, &len);

	} else if (strcmp(operation, RC6_POWER_TOKEN) == 0) {
		len = scnprintf(buf, sizeof(buf),
				"RENDER WELL: %s & MEDIA WELL: %s\n",
				(I915_READ(VLV_POWER_WELL_STATUS_REG) &
					VLV_RENDER_WELL_STATUS_MASK)
					? "UP" : "DOWN",
				(I915_READ(VLV_POWER_WELL_STATUS_REG) &
					VLV_MEDIA_WELL_STATUS_MASK)
					? "UP" : "DOWN");

	} else if (strcmp(operation, READ_COUNTER_0_TOKEN) == 0) {
		len = scnprintf(buf, sizeof(buf),
				"RENDER WELL C0 COUNTER: 0x%x & ",
				(unsigned int) I915_READ(GEN6_GT_GFX_RC6));
		len += scnprintf(&buf[len], (sizeof(buf) - len),
				"MEDIA WELL C1 COUNTER: 0x%x\n",
				(unsigned int) I915_READ(GEN6_GT_GFX_RC6p));

	} else if (strcmp(operation, READ_COUNTER_1_TOKEN) == 0) {
		len = scnprintf(buf, sizeof(buf),
				"RENDER WELL C1 COUNTER: 0x%x & ",
				(unsigned int)I915_READ(GEN6_GT_GFX_RC6pp));
		len += scnprintf(&buf[len], (sizeof(buf) - len),
				"MEDIA WELL C1 COUNTER: 0x%x\n",
				(unsigned int)
					I915_READ(VLV_MEDIA_C1_COUNT_REG));

	} else if (strcmp(operation, READ_COUNTER_6_TOKEN) == 0) {
		len = scnprintf(buf, sizeof(buf),
				"RENDER WELL C6 COUNTER: 0x%x & ",
				(unsigned int)
					I915_READ(VLV_RENDER_C0_COUNT_REG));
		len += scnprintf(&buf[len], (sizeof(buf) - len),
				"MEDIA WELL C6 COUNTER: 0x%x\n",
				(unsigned int)
					I915_READ(VLV_MEDIA_C0_COUNT_REG));

	} else if (strcmp(operation, MULTITHREAD_TOKEN) == 0) {
		len = scnprintf(buf, sizeof(buf), "NOTSUPPORTED\n");

	} else if (strcmp(operation, RC6_SINGLETHREAD_TOKEN) == 0) {
		len = scnprintf(buf, sizeof(buf),
			"SINGLE THREAD ENABLED: Yes\n");
	} else
		len = scnprintf(buf, sizeof(buf), "NOTSUPPORTED\n");

	i915_debugfs_vars.rc6.rc6_input = 0;
	simple_read_from_buffer(ubuf, max, ppos, buf, len);

	return len;
}

static ssize_t
i915_write_rc6_api(struct file *filp,
		  const char __user *ubuf,
		  size_t cnt,
		  loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	/* reset the string */
	memset(i915_debugfs_vars.rc6.rc6_vars, 0, MAX_BUFFER_STR_LEN);

	if (cnt > 0) {
		if (cnt > sizeof(i915_debugfs_vars.rc6.rc6_vars) - 1)
			return -EINVAL;

		if (copy_from_user(i915_debugfs_vars.rc6.rc6_vars, ubuf, cnt))
			return -EFAULT;

		i915_debugfs_vars.rc6.rc6_vars[cnt] = 0;

		/* Enable read */
		i915_debugfs_vars.rc6.rc6_input = 1;
	}

	return cnt;
}

static const struct file_operations i915_rc6_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = i915_read_rc6_api,
	.write = i915_write_rc6_api,
	.llseek = default_llseek,
};


static ssize_t
i915_read_rc6_status(struct file *filp,
		   char __user *ubuf,
		   size_t max,
		   loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;
	drm_i915_private_t *dev_priv = dev->dev_private;
	char buf[80];
	int len = 0;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	len = scnprintf(buf, sizeof(buf),
		"RC6 is %s\n",
		(is_rc6_enabled(dev)) ?
				"enabled" : "disabled");

	len += scnprintf(&buf[len], (sizeof(buf) - len),
		"Render well is %s & Media well is %s\n",
		(I915_READ(VLV_POWER_WELL_STATUS_REG) &
			VLV_RENDER_WELL_STATUS_MASK) ? "UP" : "DOWN",
		(I915_READ(VLV_POWER_WELL_STATUS_REG) &
			VLV_MEDIA_WELL_STATUS_MASK) ? "UP" : "DOWN");

	len += scnprintf(&buf[len], (sizeof(buf) - len),
		"Wait fifo count: %d\n",
		atomic_read(&dev_priv->wfifo_count));

	return simple_read_from_buffer(ubuf, max, ppos, buf, len);
}

static ssize_t
i915_write_rc6_status(struct file *filp,
		  const char __user *ubuf,
		  size_t cnt,
		  loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;
	char buf[20];
	int ret = 0;
	long unsigned int val = 0;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	if (cnt > 0) {
		if (cnt > sizeof(buf) - 1)
			return -EINVAL;

		if (copy_from_user(buf, ubuf, cnt))
			return -EFAULT;
		buf[cnt] = 0;

		ret = kstrtoul(buf, 0, (unsigned long *)&val);
		if (ret)
			return -EINVAL;
	}



	/*
	* 1=> Enable RC6, else disable.
	*/
	ret = rc6_enable_disable(dev, val);
	if (ret)
		return ret;

	return cnt;
}

static const struct file_operations i915_rc6_status_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = i915_read_rc6_status,
	.write = i915_write_rc6_status,
	.llseek = default_llseek,
};




static ssize_t
i915_mmio_read_api(struct file *filp,
		   char __user *ubuf,
		   size_t max,
		   loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;
	drm_i915_private_t *dev_priv = dev->dev_private;
	char buf[200], offset[20], operation[10], format[20], val[20];
	int len = 0, ret;
	int no_of_tokens;
	long unsigned int mmio_offset, mmio_to_write;

	if (INTEL_INFO(dev)->gen < 6)
		return -ENODEV;

	if (i915_debugfs_vars.mmio.mmio_input == 0)
		return len;

	snprintf(format, sizeof(format), "%%%zus %%%zus %%%zus",
				sizeof(operation), sizeof(offset), sizeof(val));

	no_of_tokens = sscanf(i915_debugfs_vars.mmio.mmio_vars,
					format, operation, offset, val);

	if (no_of_tokens < 3)
		return len;

	len = sizeof(i915_debugfs_vars.mmio.mmio_vars);

	if (strcmp(operation, READ_TOKEN) == 0) {

		ret = kstrtoul(offset, 16, &mmio_offset);
		if (ret)
			return -EINVAL;

		len = scnprintf(buf, sizeof(buf),
				"0x%x: 0x%x\n",
				(unsigned int) mmio_offset,
				(unsigned int) I915_READ(mmio_offset));
	} else if (strcmp(operation, WRITE_TOKEN) == 0) {

		ret = kstrtoul(offset, 16, &mmio_offset);
		if (ret)
			return -EINVAL;

		ret = kstrtoul(val, 16, &mmio_to_write);
		if (ret)
			return -EINVAL;

		I915_WRITE(mmio_offset, mmio_to_write);
		len = scnprintf(buf, sizeof(buf),
				"0x%x: 0x%x\n",
				(unsigned int) mmio_offset,
				(unsigned int) I915_READ(mmio_offset));
	} else
		len = scnprintf(buf, sizeof(buf), "Operation Not Supported\n");

	i915_debugfs_vars.mmio.mmio_input = 0;

	simple_read_from_buffer(ubuf, max, ppos, buf, len);

	return len;
}

static ssize_t
i915_mmio_write_api(struct file *filp,
		  const char __user *ubuf,
		  size_t cnt,
		  loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;

	if (INTEL_INFO(dev)->gen < 6)
		return -ENODEV;

	/* reset the string */
	memset(i915_debugfs_vars.mmio.mmio_vars, 0, MAX_BUFFER_STR_LEN);

	if (cnt > 0) {
		if (cnt > sizeof(i915_debugfs_vars.mmio.mmio_vars) - 1)
			return -EINVAL;

		if (copy_from_user(i915_debugfs_vars.mmio.mmio_vars, ubuf, cnt))
			return -EFAULT;

		i915_debugfs_vars.mmio.mmio_vars[cnt] = 0;

		/* Enable Read */
		i915_debugfs_vars.mmio.mmio_input = 1;
	}

	return cnt;
}

static const struct file_operations i915_mmio_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = i915_mmio_read_api,
	.write = i915_mmio_write_api,
	.llseek = default_llseek,
};

/* Debugfs iosf apis implementation */

static ssize_t
i915_iosf_read_api(struct file *filp,
		   char __user *ubuf,
		   size_t max,
		   loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;
	drm_i915_private_t *dev_priv = dev->dev_private;
	char buf[200], operation[10], port[10], offset[20], format[20];
	int len = 0, ret, noOfTokens;
	unsigned long iosf_reg;
	u32 iosf_val;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	if (i915_debugfs_vars.iosf.iosf_input == 0)
		return len;

	snprintf(format, sizeof(format), "%%%zus %%%zus %%%zus",
			sizeof(operation), sizeof(port), sizeof(offset));

	noOfTokens = sscanf(i915_debugfs_vars.iosf.iosf_vars,
				format, operation, port, offset);

	if (noOfTokens < 3)
		return len;

	len = sizeof(i915_debugfs_vars.iosf.iosf_vars);

	ret = kstrtoul(offset, 16, &iosf_reg);
	if (ret)
		return -EINVAL;

	if (strcmp(operation, READ_TOKEN) == 0) {
		if (strcmp(port, IOSF_PUNIT_TOKEN) == 0) {
			iosf_val = vlv_punit_read(dev_priv, iosf_reg);
			len = scnprintf(buf, sizeof(buf),
				"0x%x: 0x%x\n", (unsigned int) iosf_reg,
						(unsigned int) iosf_val);
		} else if (strcmp(port, IOSF_FUSE_TOKEN) == 0) {
			iosf_val = vlv_nc_read(dev_priv, iosf_reg);
			len = scnprintf(buf, sizeof(buf),
				"0x%x: 0x%x\n", (unsigned int) iosf_reg,
						(unsigned int) iosf_val);
		} else if (strcmp(port, IOSF_CCU_TOKEN) == 0) {
			iosf_val = vlv_ccu_read(dev_priv, iosf_reg);
			len = scnprintf(buf, sizeof(buf),
				"0x%x: 0x%x\n", (unsigned int) iosf_reg,
						(unsigned int) iosf_val);
		}
	} else {
		len = scnprintf(buf, sizeof(buf),
				"IOSF WRITE not supported\n");
	}

	i915_debugfs_vars.iosf.iosf_input = 0;

	simple_read_from_buffer(ubuf, max, ppos, buf, len);

	return len;
}

static ssize_t
i915_iosf_write_api(struct file *filp,
		  const char __user *ubuf,
		  size_t cnt,
		  loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	/* reset the string */
	memset(i915_debugfs_vars.iosf.iosf_vars, 0, MAX_BUFFER_STR_LEN);

	if (cnt > 0) {
		if (cnt > sizeof(i915_debugfs_vars.iosf.iosf_vars) - 1)
			return -EINVAL;

		if (copy_from_user(i915_debugfs_vars.iosf.iosf_vars, ubuf, cnt))
			return -EFAULT;
		i915_debugfs_vars.iosf.iosf_vars[cnt] = 0;

		/* Enable read */
		i915_debugfs_vars.iosf.iosf_input = 1;
	}

	return cnt;
}

static const struct file_operations i915_iosf_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = i915_iosf_read_api,
	.write = i915_iosf_write_api,
	.llseek = default_llseek,
};

static int
i915_cache_sharing_get(void *data, u64 *val)
{
	struct drm_device *dev = data;
	drm_i915_private_t *dev_priv = dev->dev_private;
	u32 snpcr;
	int ret;

	if (!(IS_GEN6(dev) || IS_GEN7(dev)))
		return -ENODEV;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	snpcr = I915_READ(GEN6_MBCUNIT_SNPCR);
	mutex_unlock(&dev_priv->dev->struct_mutex);

	*val = (snpcr & GEN6_MBC_SNPCR_MASK) >> GEN6_MBC_SNPCR_SHIFT;

	return 0;
}

static int
i915_cache_sharing_set(void *data, u64 val)
{
	struct drm_device *dev = data;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 snpcr;

	if (!(IS_GEN6(dev) || IS_GEN7(dev)))
		return -ENODEV;

	if (val > 3)
		return -EINVAL;

	DRM_DEBUG_DRIVER("Manually setting uncore sharing to %llu\n", val);

	/* Update the cache sharing policy here as well */
	snpcr = I915_READ(GEN6_MBCUNIT_SNPCR);
	snpcr &= ~GEN6_MBC_SNPCR_MASK;
	snpcr |= (val << GEN6_MBC_SNPCR_SHIFT);
	I915_WRITE(GEN6_MBCUNIT_SNPCR, snpcr);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(i915_cache_sharing_fops,
			i915_cache_sharing_get, i915_cache_sharing_set,
			"%llu\n");

/* As the drm_debugfs_init() routines are called before dev->dev_private is
 * allocated we need to hook into the minor for release. */
static int
drm_add_fake_info_node(struct drm_minor *minor,
		       struct dentry *ent,
		       const void *key)
{
	struct drm_info_node *node;

	node = kmalloc(sizeof(struct drm_info_node), GFP_KERNEL);
	if (node == NULL) {
		debugfs_remove(ent);
		return -ENOMEM;
	}

	node->minor = minor;
	node->dent = ent;
	node->info_ent = (void *) key;

	mutex_lock(&minor->debugfs_lock);
	list_add(&node->list, &minor->debugfs_list);
	mutex_unlock(&minor->debugfs_lock);

	return 0;
}

#define TIMESTAMP_BUFFER_LEN  100U

ssize_t i915_timestamp_read(struct file *filp,
		 char __user *ubuf,
		 size_t max,
		 loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int len = 0;
	char buf[TIMESTAMP_BUFFER_LEN] = {0,};
	unsigned long flags;
	u32 gpu_ts_hi, gpu_ts_lo, gpu_ts_hi_new;
	u64 gpu_ts;
	unsigned int cpu;
	u32 sec, nsec;
	u64 ftrace_ts;

	local_irq_save(flags);
	cpu = smp_processor_id();

	gpu_ts_hi = I915_READ(RING_TIMESTAMP_HI(RENDER_RING_BASE));
	gpu_ts_lo = I915_READ(RING_TIMESTAMP_LO(RENDER_RING_BASE));
	gpu_ts_hi_new = I915_READ(RING_TIMESTAMP_HI(RENDER_RING_BASE));
	if (gpu_ts_hi_new != gpu_ts_hi) {
		gpu_ts_lo = I915_READ(RING_TIMESTAMP_LO(RENDER_RING_BASE));
		gpu_ts_hi = gpu_ts_hi_new;
	}
	gpu_ts = (u64)gpu_ts_hi << 32 | gpu_ts_lo;

	ftrace_ts = ftrace_now(cpu);
	local_irq_restore(flags);

	nsec = do_div(ftrace_ts, NSEC_PER_SEC);
	sec = (u32) ftrace_ts;

	len = snprintf(buf, TIMESTAMP_BUFFER_LEN,
		      "CPU%03u %u.%09u s\nGPU %llu ticks\n",
		      cpu, sec, nsec, gpu_ts);

	return simple_read_from_buffer(ubuf, max, ppos,
				       (const void *) buf, sizeof(buf));
}

static const struct file_operations i915_timestamp_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = i915_timestamp_read,
	.write = NULL,
	.llseek = default_llseek,
};

static int i915_forcewake_open(struct inode *inode, struct file *file)
{
	struct drm_device *dev = inode->i_private;
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (INTEL_INFO(dev)->gen < 6)
		return 0;

	gen6_gt_force_wake_get(dev_priv, FORCEWAKE_ALL);

	return 0;
}

static int i915_forcewake_release(struct inode *inode, struct file *file)
{
	struct drm_device *dev = inode->i_private;
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (INTEL_INFO(dev)->gen < 6)
		return 0;

	gen6_gt_force_wake_put(dev_priv, FORCEWAKE_ALL);

	return 0;
}

static const struct file_operations i915_forcewake_fops = {
	.owner = THIS_MODULE,
	.open = i915_forcewake_open,
	.release = i915_forcewake_release,
};

static int i915_forcewake_create(struct dentry *root, struct drm_minor *minor)
{
	struct drm_device *dev = minor->dev;
	struct dentry *ent;

	ent = debugfs_create_file("i915_forcewake_user",
				  S_IRUSR,
				  root, dev,
				  &i915_forcewake_fops);
	if (IS_ERR(ent))
		return PTR_ERR(ent);

	return drm_add_fake_info_node(minor, ent, &i915_forcewake_fops);
}

static int i915_debugfs_create(struct dentry *root,
			       struct drm_minor *minor,
			       const char *name,
			       const struct file_operations *fops)
{
	struct drm_device *dev = minor->dev;
	struct dentry *ent;

	ent = debugfs_create_file(name,
				  S_IRUGO | S_IWUSR,
				  root, dev,
				  fops);
	if (IS_ERR(ent))
		return PTR_ERR(ent);

	return drm_add_fake_info_node(minor, ent, fops);
}

static struct drm_info_list i915_debugfs_list[] = {
	{"i915_capabilities", i915_capabilities, 0},
	{"i915_gem_objects", i915_gem_object_info, 0},
	{"i915_gem_gtt", i915_gem_gtt_info, 0},
	{"i915_gem_pinned", i915_gem_gtt_info, 0, (void *) PINNED_LIST},
	{"i915_gem_active", i915_gem_object_list_info, 0, (void *) ACTIVE_LIST},
	{"i915_gem_inactive", i915_gem_object_list_info, 0, (void *) INACTIVE_LIST},
	{"i915_gem_stolen", i915_gem_stolen_list_info },
	{"i915_gem_pageflip", i915_gem_pageflip_info, 0},
	{"i915_gem_request", i915_gem_request_info, 0},
	{"i915_gem_seqno", i915_gem_seqno_info, 0},
	{"i915_gem_fence_regs", i915_gem_fence_regs_info, 0},
	{"i915_gem_interrupt", i915_interrupt_info, 0},
	{"i915_gem_hws", i915_hws_info, 0, (void *)RCS},
	{"i915_gem_hws_blt", i915_hws_info, 0, (void *)BCS},
	{"i915_gem_hws_bsd", i915_hws_info, 0, (void *)VCS},
	{"i915_gem_hws_vebox", i915_hws_info, 0, (void *)VECS},
	{"i915_rstdby_delays", i915_rstdby_delays, 0},
	{"i915_cur_delayinfo", i915_cur_delayinfo, 0},
	{"i915_delayfreq_table", i915_delayfreq_table, 0},
	{"i915_inttoext_table", i915_inttoext_table, 0},
	{"i915_drpc_info", i915_drpc_info, 0},
	{"i915_emon_status", i915_emon_status, 0},
	{"i915_ring_freq_table", i915_ring_freq_table, 0},
	{"i915_gfxec", i915_gfxec, 0},
	{"i915_fbc_status", i915_fbc_status, 0},
	{"i915_ips_status", i915_ips_status, 0},
	{"i915_sr_status", i915_sr_status, 0},
	{"i915_opregion", i915_opregion, 0},
	{"i915_gem_framebuffer", i915_gem_framebuffer_info, 0},
	{"i915_context_status", i915_context_status, 0},
	{"i915_gen6_forcewake_count", i915_gen6_forcewake_count_info, 0},
	{"i915_swizzle_info", i915_swizzle_info, 0},
	{"i915_ppgtt_info", i915_ppgtt_info, 0},
	{"i915_dpio", i915_dpio_info, 0},
	{"i915_llc", i915_llc, 0},
	{"i915_edp_psr_status", i915_edp_psr_status, 0},
	{"i915_energy_uJ", i915_energy_uJ, 0},
	{"i915_pc8_status", i915_pc8_status, 0},
};
#define I915_DEBUGFS_ENTRIES ARRAY_SIZE(i915_debugfs_list)

static struct i915_debugfs_files {
	const char *name;
	const struct file_operations *fops;
} i915_debugfs_files[] = {
	{"i915_wedged", &i915_wedged_fops},
	{"i915_cache_sharing", &i915_cache_sharing_fops},
	{"i915_ring_stop", &i915_ring_stop_fops},
	{"i915_gem_drop_caches", &i915_drop_caches_fops},
	{"i915_ring_hangcheck", &i915_ring_hangcheck_fops},
	{"i915_error_state", &i915_error_state_fops},
	{"i915_next_seqno", &i915_next_seqno_fops},
	{"i915_mmio_api", &i915_mmio_fops},
	{"i915_iosf_api", &i915_iosf_fops},
	{"i915_rc6_api", &i915_rc6_fops},
	{"i915_rc6_status", &i915_rc6_status_fops},
	{"i915_turbo_api", &i915_turbo_fops},
	{"i915_rps_init", &i915_rps_init_fops},
	{"i915_dpst_api", &i915_dpst_fops},
	{"i915_rpm_api", &i915_rpm_fops},
	{"i915_timestamp", &i915_timestamp_fops},
};

int i915_debugfs_init(struct drm_minor *minor)
{
	int ret, i;

	ret = i915_forcewake_create(minor->debugfs_root, minor);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(i915_debugfs_files); i++) {
		ret = i915_debugfs_create(minor->debugfs_root, minor,
					  i915_debugfs_files[i].name,
					  i915_debugfs_files[i].fops);
		if (ret)
			return ret;
	}

	ret = i915_debugfs_create(minor->debugfs_root, minor,
					"cb_adjust",
					&i915_cb_adjust_fops);
	if (ret)
		return ret;

	ret = i915_debugfs_create(minor->debugfs_root, minor,
					"hs_adjust",
					&i915_hs_adjust_fops);
	if (ret)
		return ret;

	ret = i915_debugfs_create(minor->debugfs_root, minor,
					"csc_adjust",
					&i915_csc_adjust_fops);
	if (ret)
		return ret;

	ret = i915_debugfs_create(minor->debugfs_root, minor,
					"csc_enable",
					&i915_csc_enable_fops);

	if (ret)
		return ret;

	ret = i915_debugfs_create(minor->debugfs_root, minor,
					"gamma_adjust",
					&i915_gamma_adjust_fops);
	if (ret)
		return ret;

	ret = i915_debugfs_create(minor->debugfs_root, minor,
					"gamma_enable",
					&i915_gamma_enable_fops);
	if (ret)
		return ret;

	return drm_debugfs_create_files(i915_debugfs_list,
					I915_DEBUGFS_ENTRIES,
					minor->debugfs_root, minor);
}

void i915_debugfs_cleanup(struct drm_minor *minor)
{
	int i;

	drm_debugfs_remove_files(i915_debugfs_list,
				 I915_DEBUGFS_ENTRIES, minor);
	drm_debugfs_remove_files((struct drm_info_list *) &i915_forcewake_fops,
				 1, minor);
	for (i = 0; i < ARRAY_SIZE(i915_debugfs_files); i++) {
		struct drm_info_list *info_list =
			(struct drm_info_list *) i915_debugfs_files[i].fops;

		drm_debugfs_remove_files(info_list, 1, minor);
	}
}

#endif /* CONFIG_DEBUG_FS */
