/**************************************************************************
 * Copyright (c) 2014, Intel Corporation.
 * All Rights Reserved.
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
 **************************************************************************/

#include <linux/firmware.h>
#include <drm/drmP.h>
#include <drm/drm.h>
#include "psb_drv.h"
#include "vsp.h"


#ifdef CONFIG_DX_SEP54
extern int sepapp_image_verify(u8 *addr, ssize_t size, u32 key_index, u32 magic_num);
extern int sepapp_key_validity_check(u8 *addr, ssize_t size, u32 flag);
#endif

/*
 * FW load flow:
 * 1. check if verifcation FW is valid for this platform
 * 2. check if product FW is valid
 *
 * To your own FW with non-INTEL key signed, releace verfication FW.
 */

#define FW_NAME_LEN  64
#define IMR_REG_NUMBER(imrl_reg) ((imrl_reg - TNG_IMR_MSG_REGBASE) >> 2)
#define ISLAND_MAGIC_NUMBER(island_str) ((island_str[2] << 24) | (island_str[1] << 16) | (island_str[0] << 8) | '$')
#define VRL_SZ 728

#define prod_suffix		"prod"
#define verf_suffix		"verf"

static int tng_checkfw(struct drm_device *dev, char *fw_name,
	unsigned char *imr_ptr, int key, uint64_t imr_addr)
{
	const struct firmware *raw = NULL;
	const int vrl_header_size = VRL_SZ;
	int ret = -1;

	/* upload fw VRL header */
	ret = request_firmware(&raw, fw_name, &dev->pdev->dev);
	if (raw == NULL || ret < 0) {
		DRM_ERROR("Failed to request firmware %s, ret =  %d\n", fw_name, ret);
		goto out;
	}

	if (vrl_header_size > raw->size) {
		DRM_ERROR("invalid FW: FW size (%d) < VRL header size(%d)\n",
			raw->size, vrl_header_size);
		ret = -1;
		goto out;
	}

	PSB_DEBUG_INIT("Try to copy VRL header into IMR region\n");
	memcpy(imr_ptr, raw->data, vrl_header_size);

	/* validate VRL header */
#ifdef CONFIG_DX_SEP54
	PSB_DEBUG_INIT("Try to validate VRL header for %s\n", fw_name);
	ret = sepapp_key_validity_check((u8 *)imr_addr, vrl_header_size, 0);
	if (ret) {
		DRM_ERROR("firmware %s is invalid, ret %x\n", fw_name, ret);
		goto out;
	}
	PSB_DEBUG_INIT("FW %s is valid\n", fw_name);
#else
	DRM_ERROR("sep module is not compiled in");
#endif

out:
	if (raw)
		release_firmware(raw);

	return ret;
}

static int tng_get_fwinfo(struct drm_device *dev, char *fw_name,
	char *fw_basename, int *sep_key, unsigned char *imr_ptr, uint64_t imr_addr)
{
	int i, ret;
	const int key = 15;

	/* set key */
	if (drm_video_sepkey == -1)
		*sep_key = key;
	else
		*sep_key = drm_video_sepkey;

	PSB_DEBUG_INIT("sep key is %d\n", *sep_key);

	/* check if verification FW is valid */
	snprintf(fw_name, FW_NAME_LEN, "%s.bin.%s", fw_basename, verf_suffix);
	ret = tng_checkfw(dev, fw_name, imr_ptr, *sep_key, imr_addr);
	if (ret == 0)
		return ret;

	/* verfication FW is invalid, check if production FW is valid */
	snprintf(fw_name, FW_NAME_LEN, "%s.bin.%s", fw_basename, prod_suffix);
	ret = tng_checkfw(dev, fw_name, imr_ptr, *sep_key, imr_addr);
	if (ret == 0)
		return ret;

	return -1;
}

static void tng_print_imrinfo(int imrl_reg, uint64_t imr_base, uint64_t imr_end)
{
	unsigned int imr_regnum = IMR_REG_NUMBER(imrl_reg);

	if (imr_base != 0)
		PSB_DEBUG_INIT("IMR%d ranges 0x%12llx - 0x%12llx\n",
			       imr_regnum, imr_base, imr_end);

	PSB_DEBUG_INIT("IMR%d L 0x%2x = 0x%032x\n",
		       imr_regnum, imrl_reg,
		       intel_mid_msgbus_read32(PNW_IMR_MSG_PORT, imrl_reg));
	PSB_DEBUG_INIT("IMR%d H 0x%2x = 0x%032x\n",
		       imr_regnum, imrl_reg + 1,
		       intel_mid_msgbus_read32(PNW_IMR_MSG_PORT, imrl_reg+1));
	PSB_DEBUG_INIT("IMR%d RAC 0x%2x = 0x%032x\n",
		       imr_regnum,  imrl_reg + 2,
		       intel_mid_msgbus_read32(PNW_IMR_MSG_PORT, imrl_reg+2));
	PSB_DEBUG_INIT("IMR%d WAC 0x%2x = 0x%032x\n",
		       imr_regnum, imrl_reg + 3,
		       intel_mid_msgbus_read32(PNW_IMR_MSG_PORT, imrl_reg+3));
}

static void tng_get_imrinfo(int imrl_reg, uint64_t *imr_addr)
{
	uint32_t imrl, imrh;
	uint64_t imr_base, imr_end;

	imrl = intel_mid_msgbus_read32(TNG_IMR_MSG_PORT, imrl_reg);
	imrh = intel_mid_msgbus_read32(TNG_IMR_MSG_PORT, imrl_reg+1);

	imr_base = (imrl & TNG_IMR_ADDRESS_MASK) << TNG_IMR_ADDRESS_SHIFT;
	imr_end = (imrh & TNG_IMR_ADDRESS_MASK) << TNG_IMR_ADDRESS_SHIFT;

	*imr_addr = imr_base;

	tng_print_imrinfo(imrl_reg, imr_base, imr_end);
}

static int tng_securefw_prevsp(struct drm_device *dev, const struct firmware *raw)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct vsp_private *vsp_priv = dev_priv->vsp_private;
	struct vsp_secure_boot_header *boot_header;
	struct vsp_multi_app_blob_data *ma_header;
	unsigned int vrl_header_size = 736;
	unsigned char *ptr, *ma_ptr;

	if (raw->size < sizeof(struct vsp_secure_boot_header)) {
		DRM_ERROR("VSP:firmware is not a correct firmware (size %d)\n", (int)raw->size);
		return 1;
	}

	ptr = (void *)raw->data;
	ma_ptr = (void *) raw->data + vrl_header_size;
	boot_header = (struct vsp_secure_boot_header *)(ptr + vrl_header_size);
	ma_header = (struct vsp_multi_app_blob_data *)(ma_ptr + boot_header->ma_header_offset);

	/* get firmware header */
	memcpy(&vsp_priv->boot_header, boot_header, sizeof(vsp_priv->boot_header));

	if (vsp_priv->boot_header.magic_number != VSP_SECURE_BOOT_MAGIC_NR) {
		DRM_ERROR("VSP: failed to load correct vsp firmware\n"
			  "FW magic number is wrong %x (should be %x)\n",
			  vsp_priv->boot_header.magic_number,
			  VSP_SECURE_BOOT_MAGIC_NR);
		return 1;
	}

	/* read application firmware image data (for state-buffer size, etc) */
	/* load the multi-app blob header */
	memcpy(&vsp_priv->ma_header, ma_header, sizeof(vsp_priv->ma_header));
	if (vsp_priv->ma_header.magic_number != VSP_MULTI_APP_MAGIC_NR) {
		DRM_ERROR("VSP: failed to load correct vsp firmware\n"
			  "FW magic number is wrong %x (should be %x)\n",
			  vsp_priv->ma_header.magic_number,
			  VSP_MULTI_APP_MAGIC_NR);

		return 1;
	}

	VSP_DEBUG("firmware secure header:\n");
	VSP_DEBUG("boot_header magic number %x\n", boot_header->magic_number);
	VSP_DEBUG("boot_text_offset %x\n", boot_header->boot_text_offset);
	VSP_DEBUG("boot_text_reg %x\n", boot_header->boot_text_reg);
	VSP_DEBUG("boot_icache_value %x\n", boot_header->boot_icache_value);
	VSP_DEBUG("boot_icache_reg %x\n", boot_header->boot_icache_reg);
	VSP_DEBUG("boot_pc_value %x\n", boot_header->boot_pc_value);
	VSP_DEBUG("boot_pc_reg %x\n", boot_header->boot_pc_reg);
	VSP_DEBUG("ma_header_offset %x\n", boot_header->ma_header_offset);
	VSP_DEBUG("ma_header_reg %x\n", boot_header->ma_header_reg);
	VSP_DEBUG("boot_start_value %x\n", boot_header->boot_start_value);
	VSP_DEBUG("boot_start_reg %x\n", boot_header->boot_start_reg);
	VSP_DEBUG("firmware ma_blob header:\n");
	VSP_DEBUG("ma_header magic number %x\n", ma_header->magic_number);
	VSP_DEBUG("offset_from_start %x\n", ma_header->offset_from_start);
	VSP_DEBUG("imr_state_buffer_addr %x\n", ma_header->imr_state_buffer_addr);
	VSP_DEBUG("imr_state_buffer_size %x\n", ma_header->imr_state_buffer_size);
	VSP_DEBUG("apps_default_context_buffer_size %x\n",
		  ma_header->apps_default_context_buffer_size);

	return 0;
}

static void tng_securefw_postvsp(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct vsp_private *vsp_priv = dev_priv->vsp_private;

	vsp_priv->ctrl = (struct vsp_ctrl_reg *) (dev_priv->vsp_reg +
						  VSP_CONFIG_REG_SDRAM_BASE +
						  VSP_CONFIG_REG_START);

	vsp_priv->fw_loaded = VSP_FW_LOADED;
	vsp_priv->vsp_state = VSP_STATE_DOWN;

}

int tng_securefw(struct drm_device *dev, char *fw_basename, char *island_name, int imrl_reg)
{
	const struct firmware *raw = NULL;
	char fw_name[FW_NAME_LEN];
	unsigned char *imr_ptr;
	uint64_t imr_addr;
	int ret = 0, sep_key, fw_size;
	const int vrl_header_size = VRL_SZ;

	PSB_DEBUG_INIT("Try to get IMR region information\n");
	tng_get_imrinfo(imrl_reg, &imr_addr);

	PSB_DEBUG_INIT("Try to map IMR region\n");
	imr_ptr = ioremap(imr_addr, VRL_SZ);
	if (!imr_ptr) {
		DRM_ERROR("Failed to map IMR region\n");
		return 1;
	}

	ret = tng_get_fwinfo(dev, fw_name, fw_basename, &sep_key, imr_ptr, imr_addr);
	if (ret) {
		DRM_ERROR("failed to get fwinfo for %s\n", fw_basename);
		goto out;
	}
	PSB_DEBUG_INIT("Use firmware %s for %s, SEP key %d\n", fw_name, sep_key);

	PSB_DEBUG_INIT("Try to unmap IMR region\n");
	if (imr_ptr) {
		iounmap(imr_ptr);
		imr_ptr = NULL;
	}

	/* try to load firmware from storage */
	PSB_DEBUG_INIT("Try to request firmware %s\n", fw_name);
	ret = request_firmware(&raw, fw_name, &dev->pdev->dev);
	if (raw == NULL || ret < 0) {
		DRM_ERROR("Failed to request firmware %s, ret =  %d\n", fw_name, ret);
		goto out;
	}

	PSB_DEBUG_INIT("Try to map IMR region\n");
	imr_ptr = ioremap(imr_addr, raw->size);
	if (!imr_ptr) {
		DRM_ERROR("Failed to map IMR region\n");
		ret = -1;
		goto out;
	}

	if (!strncmp(island_name, "VSP", 3)) {
		ret = tng_securefw_prevsp(dev, raw);
		if (ret) {
			DRM_ERROR("VSP sanity check failed\n");
			goto out;
		}
	}

	fw_size = raw->size;
	PSB_DEBUG_INIT("Try to copy firmware into IMR region\n");
	memcpy(imr_ptr, raw->data, fw_size);

#ifdef CONFIG_DX_SEP54
	PSB_DEBUG_INIT("Try to verify firmware\n");
	ret = sepapp_image_verify((u8 *)imr_addr, fw_size, sep_key,
		ISLAND_MAGIC_NUMBER(island_name));
	if (ret) {
		DRM_ERROR("Failed to verify firmware %x\n", ret);
		goto out;
	}
	PSB_DEBUG_INIT("After verification, IMR region information\n");
	tng_print_imrinfo(imrl_reg, 0, 0);
#endif

	if (!strncmp(island_name, "VSP", 3))
		tng_securefw_postvsp(dev);

out:
	PSB_DEBUG_INIT("Try to release firmware\n");
	if (raw)
		release_firmware(raw);

	PSB_DEBUG_INIT("Try to unmap IMR region\n");
	if (imr_ptr)
		iounmap(imr_ptr);

	return ret;
}

int tng_rawfw(struct drm_device *dev, uint8_t *fw_basename)
{
	DRM_ERROR("Non secure mode never be ran\n");

	return 1;
}

