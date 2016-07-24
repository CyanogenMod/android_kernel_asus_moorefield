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
 *
 * Authors:
 *    Brad Volkin <bradley.d.volkin@intel.com>
 *
 */
#include "i915_drv.h"
#include "i915_drm.h"

#define LENGTH_BIAS 2

static const struct drm_i915_cmd_descriptor*
find_cmd_in_table(const struct drm_i915_cmd_table *table,
		  unsigned int cmd_header)
{
	int i;

	for (i = 0; i < table->count; i++) {
		const struct drm_i915_cmd_descriptor *desc = &table->table[i];
		unsigned int masked_cmd = desc->cmd.mask & cmd_header;
		unsigned int masked_value = desc->cmd.value & desc->cmd.mask;

		if (masked_cmd == masked_value)
			return desc;
	}

	return NULL;
}

static const struct drm_i915_cmd_descriptor*
find_cmd(struct intel_ring_buffer *ring, unsigned int cmd_header)
{
	static struct drm_i915_cmd_descriptor default_desc = {
		.flags = CMD_DESC_SKIP
	};

	unsigned int mask;
	int i;

	drm_i915_private_t *dev_priv = ring->dev->dev_private;

	for (i = 0; i < ring->cmd_table_count; i++) {
		const struct drm_i915_cmd_descriptor *desc;

		desc = find_cmd_in_table(&ring->cmd_tables[i], cmd_header);
		if (desc)
			return desc;
	}

	if (dev_priv->append_cmd_table[ring->id]) {
		const struct drm_i915_cmd_descriptor *desc;

		desc = find_cmd_in_table(dev_priv->append_cmd_table[ring->id],
					 cmd_header);
		if (desc)
			return desc;
	}

	mask = ring->get_cmd_length_mask(cmd_header);
	if (!mask)
		return NULL;

	default_desc.length.mask = mask;

	return &default_desc;
}

static int valid_reg(const unsigned int *table, int count, unsigned int addr)
{
	if (table && count != 0) {
		int i;

		for (i = 0; i < count; i++) {
			if (table[i] == addr)
				return 1;
		}
	}

	return 0;
}

int i915_parse_cmds(struct intel_ring_buffer *ring,
		    u32 batch_start_offset,
		    u32 *batch_base,
		    u32 batch_obj_size)
{
	int ret = 0;
	unsigned int *cmd, *batch_end;

	/* Needed because find_cmd() currently uses a static variable and
	 * upstream may rework the locking such that multiple execbuffer2
	 * calls may execute in parallel. We would need to rework find_cmd
	 * at that point.
	 */
	WARN_ON(!mutex_is_locked(&ring->dev->struct_mutex));

	/* No command tables currently indicates a platform without parsing */
	if (!ring->cmd_tables)
		return 0;

	if (!batch_base) {
		DRM_ERROR("CMD: Failed to vmap batch\n");
		return -ENOMEM;
	}

	cmd = batch_base + (batch_start_offset / sizeof(*cmd));
	batch_end = cmd + (batch_obj_size / sizeof(*batch_end));

	while (cmd < batch_end) {
		const struct drm_i915_cmd_descriptor *desc;
		unsigned int length;

		if (*cmd == MI_BATCH_BUFFER_END)
			break;

		desc = find_cmd(ring, *cmd);
		if (!desc) {
			DRM_ERROR("CMD: Unrecognized command: 0x%08X\n",
					 *cmd);
			ret = -EINVAL;
			break;
		}

		if (desc->flags & CMD_DESC_FIXED)
			length = desc->length.fixed;
		else
			length = ((*cmd & desc->length.mask) + LENGTH_BIAS);

		if ((batch_end - cmd) < length) {
			DRM_ERROR("CMD: Command length exceeds batch length: 0x%08X length=%d batchlen=%d\n",
					 *cmd,
					 length,
					 (int)(batch_end - cmd));
			ret = -EINVAL;
			break;
		}

		if (desc->flags & CMD_DESC_REJECT) {
			DRM_ERROR("CMD: Rejected command: 0x%08X\n", *cmd);
			ret = -EINVAL;
			break;
		}

		if (desc->flags & CMD_DESC_REGISTER) {
			unsigned int reg_addr =
				cmd[desc->reg.offset] & desc->reg.mask;

			if (!valid_reg(ring->reg_table,
				       ring->reg_count, reg_addr)) {
				drm_i915_private_t *dev_priv =
					ring->dev->dev_private;
				unsigned int *append_table =
					dev_priv->append_reg[ring->id].table;
				int append_count =
					dev_priv->append_reg[ring->id].count;

				if (!append_table ||
				    !valid_reg(append_table,
					       append_count,
					       reg_addr)) {
					DRM_ERROR("CMD: Rejected register 0x%08X in command: 0x%08X (ring=%d)\n",
							 reg_addr, *cmd, ring->id);
					ret = -EINVAL;
					break;
				}
			}
		}

		if (desc->flags & CMD_DESC_BITMASK) {
			int i;

			for (i = 0; i < desc->bits_count; i++) {
				unsigned int dword =
					cmd[desc->bits[i].offset] &
					desc->bits[i].mask;

				if (desc->bits[i].condition_mask != 0) {
					unsigned int condition =
						cmd[desc->bits[i].condition_offset] &
						desc->bits[i].condition_mask;

					if (condition == 0)
						continue;
				}

				if (dword != desc->bits[i].expected) {
					DRM_ERROR("CMD: Rejected command 0x%08X for bitmask 0x%08X (exp=0x%08X act=0x%08X) (ring=%d)\n",
							 *cmd, desc->bits[i].mask,
							 desc->bits[i].expected, dword, ring->id);
					ret = -EINVAL;
					break;
				}
			}

			if (ret)
				break;
		}

		cmd += length;
	}

	if (cmd >= batch_end) {
		DRM_ERROR("CMD: Got to the end of the buffer w/o a BBE cmd!\n");
		ret = -EINVAL;
	}

	return ret;
}

static void cleanup_append_cmd_table(drm_i915_private_t *dev_priv, int ring_id)
{
	if (dev_priv->append_cmd_table[ring_id]) {
		const struct drm_i915_cmd_descriptor *table =
			dev_priv->append_cmd_table[ring_id]->table;

		drm_free_large((void *)table);
		drm_free_large((void *)dev_priv->append_cmd_table[ring_id]);

		dev_priv->append_cmd_table[ring_id] = NULL;
	}
}

static void cleanup_append_reg_table(drm_i915_private_t *dev_priv, int ring_id)
{
	if (dev_priv->append_reg[ring_id].table) {
		drm_free_large((void *)dev_priv->append_reg[ring_id].table);

		dev_priv->append_reg[ring_id].table = NULL;
		dev_priv->append_reg[ring_id].count = 0;
	}
}

void i915_cmd_parser_cleanup(drm_i915_private_t *dev_priv)
{
	int i;

	for (i = 0; i < I915_NUM_RINGS; i++) {
		cleanup_append_cmd_table(dev_priv, i);
		cleanup_append_reg_table(dev_priv, i);
	}
}

static int append_cmds(drm_i915_private_t *dev_priv,
		       int ring_id,
		       struct drm_i915_cmd_parser_append *args)
{
	struct drm_i915_cmd_table *cmd_table;
	int ret;

	cmd_table = drm_malloc_ab(sizeof(*cmd_table), 1);
	if (!cmd_table)
		return -ENOMEM;

	cmd_table->count = args->cmd_count;
	cmd_table->table = drm_malloc_ab(sizeof(*cmd_table->table),
					 args->cmd_count);
	if (!cmd_table->table) {
		drm_free_large(cmd_table);
		return -ENOMEM;
	}

	ret = copy_from_user((void *)cmd_table->table,
			     (struct drm_i915_cmd_descriptor __user *)
			     (uintptr_t)args->cmds,
			     sizeof(*cmd_table->table) * args->cmd_count);
	if (ret) {
		drm_free_large((void *)cmd_table->table);
		drm_free_large(cmd_table);
		return -EFAULT;
	}

	dev_priv->append_cmd_table[ring_id] = cmd_table;

	return 0;
}

static int append_regs(drm_i915_private_t *dev_priv,
		       int ring_id,
		       struct drm_i915_cmd_parser_append *args)
{
	unsigned int *regs;
	int ret;

	regs = drm_malloc_ab(sizeof(*regs), args->reg_count);
	if (!regs)
		return -ENOMEM;

	ret = copy_from_user(regs,
			     (unsigned int __user *)(uintptr_t)args->regs,
			     sizeof(*regs) * args->reg_count);
	if (ret) {
		drm_free_large(regs);
		return -EFAULT;
	}

	dev_priv->append_reg[ring_id].table = regs;
	dev_priv->append_reg[ring_id].count = args->reg_count;

	return 0;
}

int i915_cmd_parser_append_ioctl(struct drm_device *dev, void *data,
				 struct drm_file *file_priv)
{
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct drm_i915_cmd_parser_append *args = data;
	struct intel_ring_buffer *ring = NULL;
	int ret = 0;

	mutex_lock(&dev->struct_mutex);

	/* This ioctl has DRM_ROOT_ONLY set but the code to check that flag in
	 * drm_ioctl is/was removed from the VLV kernel. To be sure, check for
	 * the appropriate permission here.
	 */
	if (!capable(CAP_SYS_ADMIN)) {
		DRM_ERROR("CMD: append from non-root user\n");
		ret = -EACCES;
		goto out;
	}

	if ((args->cmd_count < 1 && args->reg_count < 1) ||
	    (!args->cmds && !args->regs)) {
		DRM_ERROR("CMD: append with invalid lists cmd=(0x%llx, %d) reg=(0x%llx, %d)\n",
			  args->cmds, args->cmd_count,
			  args->regs, args->reg_count);
		ret = -EINVAL;
		goto out;
	}

	switch (args->ring & I915_EXEC_RING_MASK) {
	case I915_EXEC_DEFAULT:
	case I915_EXEC_RENDER:
		ring = &dev_priv->ring[RCS];
		break;
	case I915_EXEC_BSD:
		ring = &dev_priv->ring[VCS];
		break;
	case I915_EXEC_BLT:
		ring = &dev_priv->ring[BCS];
		break;
	case I915_EXEC_VEBOX:
		ring = &dev_priv->ring[VECS];
		break;
	default:
		DRM_ERROR("CMD: append with unknown ring: %d\n",
			  (int)(args->ring & I915_EXEC_RING_MASK));
		ret = -EINVAL;
		goto out;
	}

	if (args->cmds) {
		if (dev_priv->append_cmd_table[ring->id]) {
			DRM_ERROR("CMD: append cmd was already sent\n");
			ret = -EEXIST;
		} else {
			ret = append_cmds(dev_priv, ring->id, args);
		}
		if (ret)
			goto out;
	}

	if (args->regs) {
		if (dev_priv->append_reg[ring->id].table) {
			DRM_ERROR("CMD: append reg was already sent\n");
			ret = -EEXIST;
		} else {
			ret = append_regs(dev_priv, ring->id, args);
			if (ret)
				cleanup_append_cmd_table(dev_priv, ring->id);
		}
		if (ret)
			goto out;
	}

out:
	mutex_unlock(&dev->struct_mutex);
	return ret;
}
