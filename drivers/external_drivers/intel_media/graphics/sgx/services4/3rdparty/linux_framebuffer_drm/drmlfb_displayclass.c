
/**********************************************************************
 *
 * Copyright (C) Imagination Technologies Ltd. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful but, except
 * as otherwise stated in writing, without any warranty; without even the
 * implied warranty of merchantability or fitness for a particular purpose.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 *
 * Contact Information:
 * Imagination Technologies Ltd. <gpl-support@imgtec.com>
 * Home Park Estate, Kings Langley, Herts, WD4 8LZ, UK
 *
 ******************************************************************************/

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/console.h>
#include <linux/fb.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/notifier.h>
#include <linux/spinlock.h>

#include "img_defs.h"
#include "servicesext.h"
#include "kerneldisplay.h"
#include "drmlfb.h"

#include "psb_drv.h"
#include "psb_fb.h"

#include "mdfld_dsi_dbi_dsr.h"

#if !defined(SUPPORT_DRI_DRM)
#error "SUPPORT_DRI_DRM must be set"
#endif

/*	ASUS_BSP: Louis +++	*/
#ifdef CONFIG_SUPPORT_DDS_MIPI_SWITCH
static bool bootDdsCheck;
#if defined(CONFIG_EEPROM_PADSTATION)
extern int checkPadExist(int);
extern int AX_MicroP_IsP01Connected(void);
#include <linux/microp_notify.h>
#include <linux/microp_pin_def.h>
#include <linux/microp_api.h>
#endif

#endif
/*	ASUS_BSP: Louis ---	*/

#define MAXFLIPCOMMANDS 4

struct flip_command {
	IMG_HANDLE  hCmdCookie;
	IMG_UINT32  ui32DataSize;
	DISPLAYCLASS_FLIP_COMMAND2 vData;
	IMG_BOOL bFlush;
};

static struct display_flip_work {
	struct work_struct flip_work;
	struct flip_command p_flip_command[MAXFLIPCOMMANDS];
	u32 read_index, write_index;
	spinlock_t flip_commands_lock;
} display_flip_work_t;

static void *gpvAnchor;
extern int drm_psb_3D_vblank;

#define MRSTLFB_COMMAND_COUNT		1

#define FLIP_TIMEOUT (HZ/4)

/*if panel refresh rate is 60HZ
* then the max transfer time shoule be smaller
* than 16ms,otherwise the framerate will drop
*/
#define MAX_TRANS_TIME_FOR_ONE_FRAME   16
/*for JB, android use three swap buffer*/
#define SWAP_BUFFER_COUNT              3

static int FirstCleanFlag = 1;
static IMG_BOOL DRMLFBFlipBlackScreen(MRSTLFB_DEVINFO *psDevInfo,
					IMG_BOOL bAlpha);
static MRST_ERROR MRSTLFBAllocBuffer(struct MRSTLFB_DEVINFO_TAG *psDevInfo,
		IMG_UINT32 ui32Size, MRSTLFB_BUFFER **ppBuffer);
static MRST_ERROR MRSTLFBFreeBuffer(struct MRSTLFB_DEVINFO_TAG *psDevInfo,
		MRSTLFB_BUFFER **ppBuffer);

static MRSTLFB_DEVINFO * GetAnchorPtr(void)
{
	return (MRSTLFB_DEVINFO *)gpvAnchor;
}

static void SetAnchorPtr(MRSTLFB_DEVINFO *psDevInfo)
{
	gpvAnchor = (void*)psDevInfo;
}

static IMG_BOOL MRSTLFBFlip(MRSTLFB_DEVINFO *psDevInfo,
				 MRSTLFB_BUFFER *psBuffer)
{
	unsigned long ulAddr = (unsigned long)psBuffer->sDevVAddr.uiAddr;
	struct fb_info *psLINFBInfo;

	if (!psDevInfo->bSuspended && !psDevInfo->bLeaveVT)
	{
		if (MRSTLFBFlipToSurface(psDevInfo, ulAddr) == IMG_FALSE) {
			DRM_INFO("%s: returning false\n", __func__);
			return IMG_FALSE;
		}
	}

	psDevInfo->ulLastFlipAddr = ulAddr;
	psDevInfo->bLastFlipAddrValid = MRST_TRUE;

	psLINFBInfo = psDevInfo->psLINFBInfo;
	psLINFBInfo->screen_base = psBuffer->sCPUVAddr;

	if (FirstCleanFlag == 1) {
		memset(psDevInfo->sSystemBuffer.sCPUVAddr, 0,
				psDevInfo->sSystemBuffer.ui32BufferSize);
		FirstCleanFlag = 0;
		DRMLFBFlipBlackScreen(psDevInfo, IMG_TRUE);
	}
	return IMG_TRUE;
}

static inline void MRSTFBFlipComplete(MRSTLFB_SWAPCHAIN *psSwapChain, MRSTLFB_VSYNC_FLIP_ITEM* psFlipItem, MRST_BOOL bSchedule)
{
	MRSTLFB_VSYNC_FLIP_ITEM *psLastItem;
	MRST_BOOL bMISRScheduled = MRST_FALSE;
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)psSwapChain->psDrmDev->dev_private;

	if (psSwapChain) {
		psLastItem = &(psSwapChain->sLastItem);
		if (psLastItem->bValid && psLastItem->bFlipped && psLastItem->bCmdCompleted == MRST_FALSE)
		{
			psSwapChain->psPVRJTable->pfnPVRSRVCmdComplete((IMG_HANDLE)psLastItem->hCmdComplete, MRST_TRUE);
			psLastItem->bCmdCompleted = MRST_TRUE;
			bMISRScheduled = MRST_TRUE;
		}
		if (psFlipItem)
			psSwapChain->sLastItem = *psFlipItem;
	}
	BUG_ON(!dev_priv->pvr_ops);
	if (bSchedule && !bMISRScheduled)
		dev_priv->pvr_ops->OSScheduleMISR2();
}

static int MRSTLFBCopyOverlayBuf(struct drm_device *dev,
				struct intel_overlay_context *context)
{
	void *addr;
	struct drm_psb_private *dev_priv = psb_priv(dev);
	int index = dev_priv->overlay_buf_index;

	mutex_lock(&dev_priv->ov_ctrl_lock);
	addr = dev_priv->overlay_kmap[index].virtual;
	memcpy(addr, dev_priv->ov_ctrl_blk + context->index,
			sizeof(struct overlay_ctrl_blk));
	mutex_unlock(&dev_priv->ov_ctrl_lock);

	return 0;
}

static u32 MRSTLFBSetupOvadd(struct drm_device *dev,
			struct intel_overlay_context *context)
{
	int ret;
	struct drm_psb_private *dev_priv = psb_priv(dev);
	struct ttm_buffer_object *bo;
	int buf_index = dev_priv->overlay_buf_index;
	u32 ovadd = 0;

	bo = dev_priv->overlay_backbuf[dev_priv->overlay_buf_index];
	ret = MRSTLFBCopyOverlayBuf(dev, context);
	if (ret)
		return 0;

	ovadd |= context->pipe;
	ovadd |= 1;
	ovadd |= bo->offset & 0x0fffffff;

	dev_priv->overlay_buf_index = (buf_index + 1) % OVERLAY_BACKBUF_NUM;

	return ovadd;
}

static int MRSTLFBWaitOverlayFlip(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = psb_priv(dev);
	int retry = 80;

	if (BIT31 & PSB_RVDC32(OV_DOVASTA))
		return 0;

	while (--retry) {
		if (BIT31 & PSB_RVDC32(OV_DOVASTA))
			break;
		usleep_range(500, 600);
	}

	if (retry == 0) {
		DRM_DEBUG("%s: timeout wait for overlay flip\n", __func__);
		return -ETIMEDOUT;
	}

	return 0;
}

static void MRSTLFBFlipOverlay(MRSTLFB_DEVINFO *psDevInfo,
			struct intel_overlay_context *psContext, u32 pipe_mask)
{
	struct drm_device *dev;
	struct drm_psb_private *dev_priv;
	u32 ovadd_reg = OV_OVADD;
	u32 ovadd;
	u32 uDspCntr = 0;
	int overlay_pipe = (psContext->pipe >> 6) & 0x3;

	dev = psDevInfo->psDrmDevice;
	dev_priv = psDevInfo->psDrmDevice->dev_private;

	/* DRM_INFO("%s: flip 0x%x, index %d, pipe 0x%x\n", __func__,
		psContext->ovadd, psContext->index, psContext->pipe);
	*/
	if (psContext->index == 1)
		ovadd_reg = OVC_OVADD;
	else if (psContext->index > 1)
		return;

	ovadd = MRSTLFBSetupOvadd(dev, psContext);
	if (ovadd == 0)
		return;
	if (!(is_cmd_mode_panel(dev) && overlay_pipe == 0))
		MRSTLFBWaitOverlayFlip(dev);
	PSB_WVDC32(ovadd, ovadd_reg);

	/* If overlay enabled while display plane doesn't,
	 * disable display plane explicitly */
	/* A pipe */
	if (overlay_pipe == 0 && !(pipe_mask & (1 << 0))) {
		/* WA: this is workaround to blank sprite instead of
		* disabling sprite plane. As we find that it causes
		* overlay update always to be failure when disable and
		* re-enable overlay on CTP */
		uDspCntr = PSB_RVDC32(DSPACNTR);
		if (dev_priv->init_screen_start != PSB_RVDC32(DSPASURF) ||
			(uDspCntr & DISPPLANE_32BPP) != DISPPLANE_32BPP)
			DRMLFBFlipBlackScreen(psDevInfo, IMG_TRUE);
#if 0
		uDspCntr = PSB_RVDC32(DSPACNTR);
		if (uDspCntr & DISPLAY_PLANE_ENABLE) {
			uDspCntr &= ~DISPLAY_PLANE_ENABLE;
			PSB_WVDC32(uDspCntr, DSPACNTR);
			/* trigger cntr register take effect */
			// FIXME: comment it firstly as it may cause
			// display freeze due to some limitation unknown
			//PSB_WVDC32(0, DSPASURF);
		}
#endif
	} else if (overlay_pipe == 2 && !(pipe_mask & (1 << 1))) {
		uDspCntr = PSB_RVDC32(DSPACNTR + 0x1000);
		if (uDspCntr & DISPLAY_PLANE_ENABLE) {
			uDspCntr &= ~DISPLAY_PLANE_ENABLE;
			PSB_WVDC32(uDspCntr, DSPACNTR + 0x1000);
			/* Set displayB constant alpha to 0 when disable it */
			PSB_WVDC32(1 << 31, DSPBSURF + 0xC);
		}
	} else if (overlay_pipe == 1 && !(pipe_mask & (1 << 2))) {
		uDspCntr = PSB_RVDC32(DSPACNTR + 0x2000);
		if (uDspCntr & DISPLAY_PLANE_ENABLE) {
			uDspCntr &= ~DISPLAY_PLANE_ENABLE;
			PSB_WVDC32(uDspCntr, DSPACNTR + 0x2000);
			/* trigger cntr register take effect */
			// FIXME: comment it firstly as it may cause
			// display freeze due to some limitation unknown
			//PSB_WVDC32(0, DSPCSURF);
		}
	}
}

static void MRSTLFBFlipSprite(MRSTLFB_DEVINFO *psDevInfo,
			struct intel_sprite_context *psContext, u32 *pipe_mask)
{
	struct drm_device *dev;
	struct drm_psb_private *dev_priv;
	struct mdfld_dsi_config *dsi_config = 0;
	struct mdfld_dsi_hw_context *ctx;

/*	ASUS_BSP: Louis +++	*/
#ifdef CONFIG_SUPPORT_DDS_MIPI_SWITCH
#if defined(CONFIG_EEPROM_PADSTATION)
	/*	char *envp_pad_state_1[] = { "PAD_STATE=1", NULL };	*/
	char *envp_pad_state_0[] = { "PAD_STATE=0", NULL };
#endif
#endif
/*	ASUS_BSP: Louis ---	*/

	u32 reg_offset;
	int pipe;

	dev = psDevInfo->psDrmDevice;
	dev_priv =
		(struct drm_psb_private *)psDevInfo->psDrmDevice->dev_private;

	if (psContext->index == 0) {
		reg_offset = 0;
		pipe = 0;
		dsi_config = dev_priv->dsi_configs[0];
	} else if (psContext->index == 1) {
		reg_offset = 0x1000;
		pipe = 1;
	} else if (psContext->index == 2) {
		reg_offset = 0x2000;
		dsi_config = dev_priv->dsi_configs[1];
		pipe = 2;
	} else
		return;

	(*pipe_mask) |= (1 << pipe);

	if (pipe == 1)
		PSB_WVDC32(0, DSPBSURF + 0xC);

	if ((psContext->update_mask & SPRITE_UPDATE_POSITION))
		PSB_WVDC32(psContext->pos, DSPAPOS + reg_offset);
	if ((psContext->update_mask & SPRITE_UPDATE_SIZE)) {
/*	ASUS_BSP: Louis +++	*/
#ifdef CONFIG_SUPPORT_DDS_MIPI_SWITCH
#if defined(CONFIG_EEPROM_PADSTATION)
		if (!bootDdsCheck) {
			checkPadExist(0);
			DRM_INFO("[DISPLAY][DDS] %s: Do checkPadExist(0).\n", __func__);
			bootDdsCheck = true;

			DRM_INFO("[DISPLAY][DDS] %s: panel_turn_on = %d, AX_MicroP_IsP01Connected() =%d, AX_MicroP_getGPIOOutputPinLevel(OUT_uP_LCD_RST)=%d\n", __func__, panel_turn_on, AX_MicroP_IsP01Connected(), AX_MicroP_getGPIOOutputPinLevel(OUT_uP_LCD_RST));

			if ((panel_turn_on == DDS_PAD) && !AX_MicroP_IsP01Connected()) {
				DRM_INFO("[DISPLAY][DDS] hpd = 0\n");
				hpd = 0;
				DRM_INFO("[DISPLAY][DDS] P01_REMOVE.\n");
				kobject_uevent_env(&dsi_config->dev->primary->kdev.kobj, KOBJ_CHANGE, envp_pad_state_0);
			}
			/*	else if((panel_turn_on == DDS_NT35521) /*&& (AX_MicroP_getGPIOOutputPinLevel(OUT_uP_LCD_RST)==0)){
				schedule_work(&dev_priv->reset_panel_work);
			}
			*/
		}

		if (panel_id == 0)
			PSB_WVDC32 ((psContext->size > 0x35501df) ? 0x35501df : psContext->size, DSPASIZE + reg_offset);
		else
			PSB_WVDC32 ((psContext->size > 0x4ff031f) ? 0x4ff031f : psContext->size, DSPASIZE + reg_offset);
#endif
		if (1) {
			if (panel_id == 0)
				PSB_WVDC32 ((psContext->size > 0x35501df) ? 0x35501df : psContext->size, DSPASIZE + reg_offset);
			else
				PSB_WVDC32 ((psContext->size > 0x4ff031f) ? 0x4ff031f : psContext->size, DSPASIZE + reg_offset);
		}
		PSB_WVDC32(psContext->stride, DSPASTRIDE + reg_offset);
#else
		PSB_WVDC32(psContext->size, DSPASIZE + reg_offset);
		PSB_WVDC32(psContext->stride, DSPASTRIDE + reg_offset);
#endif
/*	ASUS_BSP: Louis ---	*/
	}

	if ((psContext->update_mask & SPRITE_UPDATE_CONTROL)) {
		if (!dev_priv->bhdmi_enable && pipe == 1)
			PSB_WVDC32(psContext->cntr | ~DISPLAY_PLANE_ENABLE |
				(PSB_RVDC32(DSPACNTR + reg_offset) &
				DISPPLANE_GAMMA_ENABLE), DSPACNTR + reg_offset);
		else
			PSB_WVDC32(psContext->cntr | DISPLAY_PLANE_ENABLE |
				(PSB_RVDC32(DSPACNTR + reg_offset) &
				DISPPLANE_GAMMA_ENABLE), DSPACNTR + reg_offset);
	}

	if ((psContext->update_mask & SPRITE_UPDATE_SURFACE)) {
		PSB_WVDC32(psContext->linoff, DSPALINOFF + reg_offset);
		PSB_WVDC32(psContext->surf, DSPASURF + reg_offset);
	}

	if (dsi_config) {
		ctx = &dsi_config->dsi_hw_context;
		ctx->dsppos = psContext->pos;
		ctx->dspsize = psContext->size;
		ctx->dspstride = psContext->stride;
		ctx->dspcntr = psContext->cntr | ((PSB_RVDC32(DSPACNTR + reg_offset) & DISPPLANE_GAMMA_ENABLE));
		ctx->dsplinoff = psContext->linoff;
		ctx->dspsurf = psContext->surf;
	}
}

static void MRSTLFBFlipPrimary(MRSTLFB_DEVINFO *psDevInfo,
			struct intel_sprite_context *psContext, u32 *pipe_mask)
{
	struct drm_device *dev;
	struct drm_psb_private *dev_priv;
	struct mdfld_dsi_config *dsi_config = NULL;
	struct mdfld_dsi_hw_context *ctx = 0;
	u32 reg_offset;
	int pipe;

	dev = psDevInfo->psDrmDevice;
	dev_priv =
		(struct drm_psb_private *)psDevInfo->psDrmDevice->dev_private;

	if (psContext->index == 0) {
		reg_offset = 0;
		pipe = 0;
		dsi_config = dev_priv->dsi_configs[0];
	} else if (psContext->index == 1) {
		reg_offset = 0x1000;
		pipe = 1;
	} else if (psContext->index == 2) {
		reg_offset = 0x2000;
		dsi_config = dev_priv->dsi_configs[1];
		pipe = 2;
	} else
		return;

	(*pipe_mask) |= (1 << pipe);

	/*for HDMI only flip the surface address*/
	if (pipe == 1)
		psContext->update_mask &= SPRITE_UPDATE_SURFACE;

	if ((psContext->update_mask & SPRITE_UPDATE_POSITION))
		PSB_WVDC32(psContext->pos, DSPAPOS + reg_offset);
	if ((psContext->update_mask & SPRITE_UPDATE_SIZE)) {
/*	ASUS_BSP: Louis +++	*/
#ifdef CONFIG_SUPPORT_DDS_MIPI_SWITCH
		PSB_WVDC32((panel_id == 0) ? 0x35501df : 0x4ff031f, DSPASIZE + reg_offset);
		PSB_WVDC32(0xc80, DSPASTRIDE + reg_offset);
#else
		PSB_WVDC32(psContext->size, DSPASIZE + reg_offset);
		PSB_WVDC32(psContext->stride, DSPASTRIDE + reg_offset);
#endif
/*	ASUS_BSP: Louis ---	*/
	}

	if ((psContext->update_mask & SPRITE_UPDATE_CONTROL)) {
		if (!dev_priv->bhdmi_enable && pipe == 1)
			PSB_WVDC32(psContext->cntr | ~DISPLAY_PLANE_ENABLE |
				(PSB_RVDC32(DSPACNTR + reg_offset) &
				DISPPLANE_GAMMA_ENABLE), DSPACNTR + reg_offset);
		else
			PSB_WVDC32(psContext->cntr | DISPLAY_PLANE_ENABLE |
				(PSB_RVDC32(DSPACNTR + reg_offset) &
				DISPPLANE_GAMMA_ENABLE), DSPACNTR + reg_offset);
	}

	if ((psContext->update_mask & SPRITE_UPDATE_SURFACE)) {
		PSB_WVDC32(psContext->linoff, DSPALINOFF + reg_offset);
		PSB_WVDC32(psContext->surf, DSPASURF + reg_offset);
	}

	if (dsi_config) {
		ctx = &dsi_config->dsi_hw_context;
		ctx->dsppos = psContext->pos;
		ctx->dspsize = psContext->size;
		ctx->dspstride = psContext->stride;
		ctx->dspcntr = psContext->cntr | ((PSB_RVDC32(DSPACNTR + reg_offset) & DISPPLANE_GAMMA_ENABLE));
		ctx->dsplinoff = psContext->linoff;
		ctx->dspsurf = psContext->surf;
	}

}

static IMG_BOOL MRSTLFBFlipContexts(MRSTLFB_DEVINFO *psDevInfo,
			struct mdfld_plane_contexts *psContexts)
{
	struct intel_sprite_context *psPrimaryContext;
	struct intel_sprite_context *psSpriteContext;
	struct intel_overlay_context *psOverlayContext;
	struct drm_psb_private *dev_priv;
	struct drm_device *dev;
	IMG_BOOL ret = IMG_TRUE;
	int i;
	u32 pipe_mask = 0;

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND, MRST_TRUE)) {
		DRM_ERROR("mdfld_dsi_dsr: failed to hw_begin\n");
		return IMG_FALSE;
	}

	dev = psDevInfo->psDrmDevice;
	dev_priv =
		(struct drm_psb_private *)psDevInfo->psDrmDevice->dev_private;

	/*flip all active primary planes*/
	for (i = 0; i < INTEL_SPRITE_PLANE_NUM; i++) {
		if (psContexts->active_primaries & (1 << i)) {
			psPrimaryContext = &psContexts->primary_contexts[i];
			MRSTLFBFlipPrimary(psDevInfo, psPrimaryContext,
				&pipe_mask);
		}
	}

	/*flip all active sprite planes*/
	for (i = 0; i < INTEL_SPRITE_PLANE_NUM; i++) {
		if (psContexts->active_sprites & (1 << i)) {
			psSpriteContext = &psContexts->sprite_contexts[i];
			MRSTLFBFlipSprite(psDevInfo, psSpriteContext,
				&pipe_mask);
		}
	}

	/*flip all active overlay planes*/
	for (i = 0; i < INTEL_OVERLAY_PLANE_NUM; i++) {
		if (psContexts->active_overlays & (1 << i)) {
			psOverlayContext = &psContexts->overlay_contexts[i];
			MRSTLFBFlipOverlay(psDevInfo, psOverlayContext,
				pipe_mask);
		}
	}

	if (!psDevInfo->bScreenState) {
		if (mdfld_dsi_dsr_update_panel_fb(dev_priv->dsi_configs[0])) {
			DRM_DEBUG("mdfld_dsi_dsr: failed to update panel fb\n");
			ret = IMG_FALSE;
		}
	}

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	return ret;
}

static void MRSTLFBRestoreLastFlip(MRSTLFB_DEVINFO *psDevInfo)
{
	if (!psDevInfo->bSuspended && !psDevInfo->bLeaveVT)
	{
		if (psDevInfo->bLastFlipAddrValid)
		{
			MRSTLFBFlipToSurface(psDevInfo, psDevInfo->ulLastFlipAddr);
		}
	}
}

static void MRSTLFBClearSavedFlip(MRSTLFB_DEVINFO *psDevInfo)
{
	psDevInfo->bLastFlipAddrValid = MRST_FALSE;
}


static IMG_BOOL FlushInternalVSyncQueue(MRSTLFB_SWAPCHAIN *psSwapChain,
			 MRST_BOOL bFlip)
{
	MRSTLFB_VSYNC_FLIP_ITEM *psFlipItem;
	unsigned long            ulMaxIndex;
	unsigned long            i;
	IMG_BOOL                 ret = IMG_TRUE;

	psFlipItem = &psSwapChain->psVSyncFlips[psSwapChain->ulRemoveIndex];
	ulMaxIndex = psSwapChain->ulSwapChainLength - 1;

	MRSTFBFlipComplete(psSwapChain, NULL, MRST_TRUE);
	for(i = 0; i < psSwapChain->ulSwapChainLength; i++)
	{
		if (psFlipItem->bValid == MRST_FALSE)
		{
			continue;
		}

		DEBUG_PRINTK((KERN_INFO DRIVER_PREFIX ": FlushInternalVSyncQueue: Flushing swap buffer (index %lu)\n", psSwapChain->ulRemoveIndex));

		if (psFlipItem->bFlipped == MRST_FALSE && bFlip)
		{
			if (psFlipItem->psBuffer)
				ret = MRSTLFBFlip(
					psSwapChain->psDevInfo,
					psFlipItem->psBuffer);
			else
				ret = MRSTLFBFlipContexts(
					psSwapChain->psDevInfo,
					&psFlipItem->sPlaneContexts);
			if (ret == IMG_FALSE) {
				DRM_INFO("%s: returning %d from DRMLFBFlipBuffer2", __func__, ret);
			}

		}

		if(psFlipItem->bCmdCompleted == MRST_FALSE)
		{
			DEBUG_PRINTK((KERN_INFO DRIVER_PREFIX ": FlushInternalVSyncQueue: Calling command complete for swap buffer (index %lu)\n", psSwapChain->ulRemoveIndex));

			psSwapChain->psPVRJTable->pfnPVRSRVCmdComplete((IMG_HANDLE)psFlipItem->hCmdComplete, MRST_TRUE);
		}


		psSwapChain->ulRemoveIndex++;

		if(psSwapChain->ulRemoveIndex > ulMaxIndex)
		{
			psSwapChain->ulRemoveIndex = 0;
		}


		psFlipItem->bFlipped = MRST_FALSE;
		psFlipItem->bCmdCompleted = MRST_FALSE;
		psFlipItem->bValid = MRST_FALSE;


		psFlipItem = &psSwapChain->psVSyncFlips[psSwapChain->ulRemoveIndex];
	}

	psSwapChain->ulInsertIndex = 0;
	psSwapChain->ulRemoveIndex = 0;

	return IMG_TRUE;
}

static int DRMLFBFifoEmpty(MRSTLFB_DEVINFO *psDevInfo)
{
	struct drm_device *dev = psDevInfo->psDrmDevice;
	struct drm_psb_private *dev_priv =
	(struct drm_psb_private *) psDevInfo->psDrmDevice->dev_private;

	return dev_priv->async_check_fifo_empty(dev);
}

static IMG_BOOL DRMLFBFlipBuffer(MRSTLFB_DEVINFO *psDevInfo,
		 MRSTLFB_SWAPCHAIN *psSwapChain,
		 MRSTLFB_BUFFER *psBuffer)
{
	IMG_BOOL ret = IMG_TRUE;
	if(psSwapChain != NULL)
	{
		if(psDevInfo->psCurrentSwapChain != NULL)
		{
			if(psDevInfo->psCurrentSwapChain != psSwapChain)
				ret = FlushInternalVSyncQueue(
				psDevInfo->psCurrentSwapChain, MRST_FALSE);
			if (ret == IMG_FALSE) {
				DRM_INFO("%s: returning %d from FlushInternalVSyncQueue\n", __func__, ret);
				return ret;
			}
		}
		psDevInfo->psCurrentSwapChain = psSwapChain;
		psDevInfo->psCurrentBuffer = psBuffer;
	}

	ret = MRSTLFBFlip(psDevInfo, psBuffer);
	if (ret != IMG_TRUE)
		DRM_INFO("%s: returning %d from MRSTLFBFlip", __func__, ret);
	return ret;
}

static IMG_BOOL DRMLFBFlipBlackScreen(MRSTLFB_DEVINFO *psDevInfo,
					IMG_BOOL bAlpha)
{
	struct drm_psb_private *dev_priv;
	u32 offset;
	u32 dspcntr;

	if (!psDevInfo) {
		DRM_ERROR("Invalid parameters\n");
		return IMG_FALSE;
	}

	if (psDevInfo->ui32MainPipe == 0)
		offset = 0x0000;
	else if (psDevInfo->ui32MainPipe == 1)
		offset = 0x1000;
	else if (psDevInfo->ui32MainPipe == 2)
		offset = 0x2000;
	else
		return IMG_FALSE;

	dev_priv =
		(struct drm_psb_private *)psDevInfo->psDrmDevice->dev_private;

	if (!dev_priv) {
		DRM_ERROR("Invalid parameters\n");
		return IMG_FALSE;
	}

	dspcntr = PSB_RVDC32(DSPACNTR + offset);
	/* mask alpha and pixel format if bAlpha to be false*/
	if (!bAlpha) {
		dspcntr &= (~DISPPLANE_PIXFORMAT_MASK);
		dspcntr |= DISPPLANE_32BPP_NO_ALPHA;
	} else {
		dspcntr &= (~DISPPLANE_PIXFORMAT_MASK);
		dspcntr |= DISPPLANE_32BPP;
	}

	PSB_WVDC32(0x0, DSPAPOS + offset);
	/* We use small size buffer to avoid unnecessary bandwidth */
	PSB_WVDC32(32 | (32 << 16), DSPASIZE + offset);
	PSB_WVDC32(128, DSPASTRIDE + offset);
	PSB_WVDC32(dev_priv->init_screen_offset, DSPALINOFF + offset);
	PSB_WVDC32(dspcntr, DSPACNTR + offset);
	PSB_WVDC32(dev_priv->init_screen_start, DSPASURF + offset);

	return IMG_TRUE;
}

static IMG_BOOL DRMLFBFlipBlackScreen2(MRSTLFB_DEVINFO *psDevInfo,
					IMG_BOOL bAlpha)
{
	struct drm_psb_private *dev_priv;
	struct mdfld_dsi_config *dsi_config = 0;
	struct mdfld_dsi_hw_context *ctx;
	u32 offset;
	u32 dspcntr;

	if (!psDevInfo) {
		DRM_ERROR("Invalid parameters\n");
		return IMG_FALSE;
	}

	dev_priv =
		(struct drm_psb_private *)psDevInfo->psDrmDevice->dev_private;

	if (!dev_priv) {
		DRM_ERROR("Invalid parameters\n");
		return IMG_FALSE;
	}

	if (psDevInfo->ui32MainPipe == 0) {
		offset = 0x0000;
		dsi_config = dev_priv->dsi_configs[0];
	} else if (psDevInfo->ui32MainPipe == 1) {
		offset = 0x1000;
	} else if (psDevInfo->ui32MainPipe == 2) {
		offset = 0x2000;
		dsi_config = dev_priv->dsi_configs[1];
	} else
		return IMG_FALSE;

	dspcntr = PSB_RVDC32(DSPACNTR + offset);
	/* mask alpha and pixel format if bAlpha to be false*/
	if (!bAlpha) {
		dspcntr &= (~DISPPLANE_PIXFORMAT_MASK);
		dspcntr |= DISPPLANE_32BPP_NO_ALPHA;
	} else {
		dspcntr &= (~DISPPLANE_PIXFORMAT_MASK);
		dspcntr |= DISPPLANE_32BPP;
	}

	PSB_WVDC32(0x0, DSPAPOS + offset);

	/* In CTP platform, Actually need stride mismatch with framebuffer stride,
	this maybe cause garbage display in screen bottom, set and store the stride
	value with init framebuffer stride after destory the swapchain*/
	PSB_WVDC32(dev_priv->init_screen_size, DSPASIZE + offset);
	PSB_WVDC32(psDevInfo->sDisplayDim.ui32ByteStride, DSPASTRIDE + offset);
	PSB_WVDC32(dev_priv->init_screen_offset, DSPALINOFF + offset);
	PSB_WVDC32(dspcntr, DSPACNTR + offset);
	PSB_WVDC32(dev_priv->init_screen_start, DSPASURF + offset);

	if (dsi_config) {
		ctx = &dsi_config->dsi_hw_context;
		ctx->dsppos = 0x0;
		ctx->dspsize = dev_priv->init_screen_size;
		ctx->dspstride = psDevInfo->sDisplayDim.ui32ByteStride;
		ctx->dspcntr = dspcntr;
		ctx->dsplinoff = dev_priv->init_screen_offset;
		ctx->dspsurf = dev_priv->init_screen_start;
	}
	return IMG_TRUE;
}

static IMG_BOOL DRMLFBFlipBuffer2(MRSTLFB_DEVINFO *psDevInfo,
			MRSTLFB_SWAPCHAIN *psSwapChain,
			struct mdfld_plane_contexts *psContexts)
{
	IMG_BOOL ret = IMG_TRUE;

	if (!psSwapChain)
		goto flip_out;

	if (!psDevInfo->psCurrentSwapChain) {
		psDevInfo->psCurrentSwapChain = psSwapChain;
		psDevInfo->psCurrentBuffer = NULL;
		goto flip_out;
	}

	if (psDevInfo->psCurrentSwapChain != psSwapChain) {
		ret = FlushInternalVSyncQueue(psDevInfo->psCurrentSwapChain,
					MRST_FALSE);
		if (ret == IMG_FALSE) {
			DRM_INFO("%s: returning %d from FlushInternalVSyncQueue\n", __func__, ret);
			return ret;
		}
                psDevInfo->psCurrentSwapChain = psSwapChain;
                psDevInfo->psCurrentBuffer = NULL;
	}

flip_out:
	ret = MRSTLFBFlipContexts(psDevInfo, psContexts);
	if (ret != IMG_TRUE)
		DRM_DEBUG("%s: returning %d from MRSTLFBFlipContexts\n", __func__, ret);

	if (FirstCleanFlag == 1) {
		memset(psDevInfo->sSystemBuffer.sCPUVAddr, 0,
				psDevInfo->sSystemBuffer.ui32BufferSize);
		DRMLFBFlipBlackScreen(psDevInfo, IMG_TRUE);
		FirstCleanFlag = 0;
	}

	return ret;
}

static void SetFlushStateNoLock(MRSTLFB_DEVINFO* psDevInfo,
                                        MRST_BOOL bFlushState)
{
	if (bFlushState)
	{
		if (psDevInfo->ulSetFlushStateRefCount == 0)
		{
			psDevInfo->bFlushCommands = MRST_TRUE;
			if (psDevInfo->psCurrentSwapChain != NULL)
			{
				FlushInternalVSyncQueue(psDevInfo->psCurrentSwapChain, MRST_TRUE);
			}
		}
		psDevInfo->ulSetFlushStateRefCount++;
	}
	else
	{
		if (psDevInfo->ulSetFlushStateRefCount != 0)
		{
			psDevInfo->ulSetFlushStateRefCount--;
			if (psDevInfo->ulSetFlushStateRefCount == 0)
			{
				psDevInfo->bFlushCommands = MRST_FALSE;
			}
		}
	}
}

static IMG_VOID SetFlushState(MRSTLFB_DEVINFO* psDevInfo,
                                      MRST_BOOL bFlushState)
{
	mutex_lock(&psDevInfo->sSwapChainMutex);

	SetFlushStateNoLock(psDevInfo, bFlushState);

	mutex_unlock(&psDevInfo->sSwapChainMutex);
}

static IMG_VOID SetDCState(IMG_HANDLE hDevice, IMG_UINT32 ui32State)
{
	MRSTLFB_DEVINFO *psDevInfo = (MRSTLFB_DEVINFO *)hDevice;

	switch (ui32State)
	{
		case DC_STATE_FLUSH_COMMANDS:
			SetFlushState(psDevInfo, MRST_TRUE);
			break;
		case DC_STATE_NO_FLUSH_COMMANDS:
			SetFlushState(psDevInfo, MRST_FALSE);
			break;
		default:
			break;
	}

	return;
}

#if 0
static int FrameBufferEvents(struct notifier_block *psNotif,
                             unsigned long event, void *data)
{
	MRSTLFB_DEVINFO *psDevInfo;
	struct fb_event *psFBEvent = (struct fb_event *)data;
	MRST_BOOL bBlanked;


	if (event != FB_EVENT_BLANK)
	{
		return 0;
	}

	psDevInfo = GetAnchorPtr();

	bBlanked = (*(IMG_INT *)psFBEvent->data != 0) ? MRST_TRUE: MRST_FALSE;

	if (bBlanked != psDevInfo->bBlanked)
	{
		psDevInfo->bBlanked = bBlanked;

		SetFlushState(psDevInfo, bBlanked);
	}

	return 0;
}
#endif

static MRST_ERROR UnblankDisplay(MRSTLFB_DEVINFO *psDevInfo)
{
#if 0
	int res;

	console_lock();
	res = fb_blank(psDevInfo->psLINFBInfo, 0);
	console_unlock();
	if (res != 0)
	{
		printk(KERN_WARNING DRIVER_PREFIX
			": fb_blank failed (%d)", res);
		return (MRST_ERROR_GENERIC);
	}
#endif

	return (MRST_OK);
}

#if 0
static MRST_ERROR EnableLFBEventNotification(MRSTLFB_DEVINFO *psDevInfo)
{
	int                res;
	MRST_ERROR         eError;


	memset(&psDevInfo->sLINNotifBlock, 0, sizeof(psDevInfo->sLINNotifBlock));

	psDevInfo->sLINNotifBlock.notifier_call = FrameBufferEvents;
	psDevInfo->bBlanked = MRST_FALSE;

	res = fb_register_client(&psDevInfo->sLINNotifBlock);
	if (res != 0)
	{
		printk(KERN_WARNING DRIVER_PREFIX
			": fb_register_client failed (%d)", res);

		return (MRST_ERROR_GENERIC);
	}

	eError = UnblankDisplay(psDevInfo);
	if (eError != MRST_OK)
	{
		DEBUG_PRINTK((KERN_WARNING DRIVER_PREFIX
			": UnblankDisplay failed (%d)", eError));
		return eError;
	}

	return (MRST_OK);
}

static MRST_ERROR DisableLFBEventNotification(MRSTLFB_DEVINFO *psDevInfo)
{
	int res;


	res = fb_unregister_client(&psDevInfo->sLINNotifBlock);
	if (res != 0)
	{
		printk(KERN_WARNING DRIVER_PREFIX
			": fb_unregister_client failed (%d)", res);
		return (MRST_ERROR_GENERIC);
	}

	return (MRST_OK);
}
#endif

static PVRSRV_ERROR OpenDCDevice(IMG_UINT32 ui32DeviceID,
                                 IMG_HANDLE *phDevice,
                                 PVRSRV_SYNC_DATA* psSystemBufferSyncData)
{
	MRSTLFB_DEVINFO *psDevInfo;

	UNREFERENCED_PARAMETER(ui32DeviceID);

	psDevInfo = GetAnchorPtr();


	psDevInfo->sSystemBuffer.psSyncData = psSystemBufferSyncData;

	psDevInfo->ulSetFlushStateRefCount = 0;
	psDevInfo->bFlushCommands = MRST_FALSE;

	/* As we don't rely on fb device, instead we use post/post2 to do flip,
	 * so there's no need to register FB event notify call chain.
	 */
#if 0
	eError = EnableLFBEventNotification(psDevInfo);
	if (eError != MRST_OK)
	{
		printk(KERN_WARNING DRIVER_PREFIX ": Couldn't enable framebuffer event notification\n");
		return PVRSRV_ERROR_UNABLE_TO_OPEN_DC_DEVICE;
	}
#endif

	*phDevice = (IMG_HANDLE)psDevInfo;

	return (PVRSRV_OK);
}

static PVRSRV_ERROR CloseDCDevice(IMG_HANDLE hDevice)
{
#if 0
	MRSTLFB_DEVINFO *psDevInfo = (MRSTLFB_DEVINFO *)hDevice;
	MRST_ERROR eError;

	eError = DisableLFBEventNotification(psDevInfo);
	if (eError != MRST_OK)
	{
		printk(KERN_WARNING DRIVER_PREFIX ": Couldn't disable framebuffer event notification\n");
		return PVRSRV_ERROR_UNABLE_TO_REMOVE_DEVICE;
	}
#endif

	return (PVRSRV_OK);
}

static PVRSRV_ERROR EnumDCFormats(IMG_HANDLE hDevice,
                                  IMG_UINT32 *pui32NumFormats,
                                  DISPLAY_FORMAT *psFormat)
{
	MRSTLFB_DEVINFO	*psDevInfo;

	if(!hDevice || !pui32NumFormats)
	{
		return (PVRSRV_ERROR_INVALID_PARAMS);
	}

	psDevInfo = (MRSTLFB_DEVINFO*)hDevice;

	*pui32NumFormats = 1;

	if(psFormat)
	{
		psFormat[0] = psDevInfo->sDisplayFormat;
	}

	return (PVRSRV_OK);
}

static PVRSRV_ERROR EnumDCDims(IMG_HANDLE hDevice,
                               DISPLAY_FORMAT *psFormat,
                               IMG_UINT32 *pui32NumDims,
                               DISPLAY_DIMS *psDim)
{
	MRSTLFB_DEVINFO	*psDevInfo;

	if(!hDevice || !psFormat || !pui32NumDims)
	{
		return (PVRSRV_ERROR_INVALID_PARAMS);
	}

	psDevInfo = (MRSTLFB_DEVINFO*)hDevice;

	*pui32NumDims = 1;


	if(psDim)
	{
		psDim[0] = psDevInfo->sDisplayDim;
	}

	return (PVRSRV_OK);
}


static PVRSRV_ERROR GetDCSystemBuffer(IMG_HANDLE hDevice, IMG_HANDLE *phBuffer)
{
	MRSTLFB_DEVINFO	*psDevInfo;

	if(!hDevice || !phBuffer)
	{
		return (PVRSRV_ERROR_INVALID_PARAMS);
	}

	psDevInfo = (MRSTLFB_DEVINFO*)hDevice;



	*phBuffer = (IMG_HANDLE)&psDevInfo->sSystemBuffer;

	return (PVRSRV_OK);
}


static PVRSRV_ERROR GetDCInfo(IMG_HANDLE hDevice, DISPLAY_INFO *psDCInfo)
{
	MRSTLFB_DEVINFO	*psDevInfo;

	if(!hDevice || !psDCInfo)
	{
		return (PVRSRV_ERROR_INVALID_PARAMS);
	}

	psDevInfo = (MRSTLFB_DEVINFO*)hDevice;

	*psDCInfo = psDevInfo->sDisplayInfo;

	return (PVRSRV_OK);
}

static PVRSRV_ERROR GetDCBufferAddr(IMG_HANDLE        hDevice,
                                    IMG_HANDLE        hBuffer,
                                    IMG_SYS_PHYADDR   **ppsSysAddr,
                                    IMG_SIZE_T        *pui32ByteSize,
                                    IMG_VOID          **ppvCpuVAddr,
                                    IMG_HANDLE        *phOSMapInfo,
                                    IMG_BOOL          *pbIsContiguous,
	                            IMG_UINT32	      *pui32TilingStride)
{
	MRSTLFB_DEVINFO	*psDevInfo;
	MRSTLFB_BUFFER *psSystemBuffer;

	UNREFERENCED_PARAMETER(pui32TilingStride);

	if(!hDevice)
	{
		return (PVRSRV_ERROR_INVALID_PARAMS);
	}
	psDevInfo = (MRSTLFB_DEVINFO*)hDevice;

	if(!hBuffer)
	{
		return (PVRSRV_ERROR_INVALID_PARAMS);
	}
	psSystemBuffer = (MRSTLFB_BUFFER *)hBuffer;

	if (!ppsSysAddr)
	{
		return (PVRSRV_ERROR_INVALID_PARAMS);
	}

	if( psSystemBuffer->bIsContiguous )
		*ppsSysAddr = &psSystemBuffer->uSysAddr.sCont;
	else
		*ppsSysAddr = psSystemBuffer->uSysAddr.psNonCont;

	if (!pui32ByteSize)
	{
		return (PVRSRV_ERROR_INVALID_PARAMS);
	}
	*pui32ByteSize = psSystemBuffer->ui32BufferSize;

	if (ppvCpuVAddr)
	{
		*ppvCpuVAddr = psSystemBuffer->sCPUVAddr;
	}

	if (phOSMapInfo)
	{
		*phOSMapInfo = (IMG_HANDLE)0;
	}

	if (pbIsContiguous)
	{
		*pbIsContiguous = psSystemBuffer->bIsContiguous;
	}

	return (PVRSRV_OK);
}

static PVRSRV_ERROR CreateDCSwapChain(IMG_HANDLE hDevice,
                                      IMG_UINT32 ui32Flags,
                                      DISPLAY_SURF_ATTRIBUTES *psDstSurfAttrib,
                                      DISPLAY_SURF_ATTRIBUTES *psSrcSurfAttrib,
                                      IMG_UINT32 ui32BufferCount,
                                      PVRSRV_SYNC_DATA **ppsSyncData,
                                      IMG_UINT32 ui32OEMFlags,
                                      IMG_HANDLE *phSwapChain,
                                      IMG_UINT32 *pui32SwapChainID)
{
	MRSTLFB_DEVINFO	*psDevInfo;
	MRSTLFB_SWAPCHAIN *psSwapChain;
	MRSTLFB_BUFFER **ppsBuffer;
	MRSTLFB_VSYNC_FLIP_ITEM *psVSyncFlips;
	IMG_UINT32 i;
	IMG_UINT32 iSCId = MAX_SWAPCHAINS;
	PVRSRV_ERROR eError = PVRSRV_ERROR_NOT_SUPPORTED;
	struct drm_device* psDrmDev;
	unsigned long ulSwapChainLength;
	struct drm_psb_private *dev_priv = NULL;
	struct psb_framebuffer *psbfb = NULL;
	struct psb_fbdev *fbdev = NULL;

	UNREFERENCED_PARAMETER(ui32OEMFlags);

	if(!hDevice
	|| !psDstSurfAttrib
	|| !psSrcSurfAttrib
	|| !ppsSyncData
	|| !phSwapChain)
	{
		return (PVRSRV_ERROR_INVALID_PARAMS);
	}

	psDevInfo = (MRSTLFB_DEVINFO*)hDevice;

	if(ui32BufferCount > psDevInfo->sDisplayInfo.ui32MaxSwapChainBuffers)
	{
		return (PVRSRV_ERROR_TOOMANYBUFFERS);
	}

	ulSwapChainLength = ui32BufferCount + 6;

	UNREFERENCED_PARAMETER(ui32Flags);

	psSwapChain = (MRSTLFB_SWAPCHAIN*)MRSTLFBAllocKernelMem(sizeof(MRSTLFB_SWAPCHAIN));
	if(!psSwapChain)
	{
		return (PVRSRV_ERROR_OUT_OF_MEMORY);
	}

	for(iSCId = 0;iSCId < MAX_SWAPCHAINS;++iSCId)
	{
		if( psDevInfo->apsSwapChains[iSCId] == NULL )
		{
			psDevInfo->apsSwapChains[iSCId] = psSwapChain;
			break;
		}
	}

	if(iSCId == MAX_SWAPCHAINS)
	{
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto ErrorFreeSwapChain;
	}
	ppsBuffer = (MRSTLFB_BUFFER**)MRSTLFBAllocKernelMem(sizeof(MRSTLFB_BUFFER*) * ui32BufferCount);
	if(!ppsBuffer)
	{
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto ErrorFreeSwapChain;
	}
	for (i = 0; i < ui32BufferCount; i++) ppsBuffer[i] = NULL;


	psVSyncFlips = (MRSTLFB_VSYNC_FLIP_ITEM *)MRSTLFBAllocKernelMem(sizeof(MRSTLFB_VSYNC_FLIP_ITEM) * ulSwapChainLength);
	if (!psVSyncFlips)
	{
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto ErrorFreeBuffers;
	}

	psSwapChain->ulSwapChainLength = ulSwapChainLength;
	psSwapChain->ulBufferCount = (unsigned long)ui32BufferCount;
	psSwapChain->ppsBuffer = ppsBuffer;
	psSwapChain->psVSyncFlips = psVSyncFlips;
	psSwapChain->ulInsertIndex = 0;
	psSwapChain->ulRemoveIndex = 0;
	psSwapChain->psPVRJTable = &psDevInfo->sPVRJTable;
	psSwapChain->sLastItem.bValid = MRST_FALSE;


	for (i = 0; i < ui32BufferCount; i++)
	{
		unsigned long bufSize = psDevInfo->sDisplayDim.ui32ByteStride * psDevInfo->sDisplayDim.ui32Height;
		if(MRSTLFBAllocBuffer(psDevInfo, bufSize, &ppsBuffer[i] ) != MRST_OK ) {
			eError = PVRSRV_ERROR_OUT_OF_MEMORY;
			goto ErrorFreeAllocatedBuffes;
		}
		ppsBuffer[i]->psSyncData = ppsSyncData[i];
	}


	for (i = 0; i < ulSwapChainLength; i++)
	{
		psVSyncFlips[i].bValid = MRST_FALSE;
		psVSyncFlips[i].bFlipped = MRST_FALSE;
		psVSyncFlips[i].bCmdCompleted = MRST_FALSE;
	}

	psDrmDev = psDevInfo->psDrmDevice;

	psSwapChain->psDevInfo = psDevInfo;
	psSwapChain->psDrmDev = psDrmDev;
	psSwapChain->psDrmDriver = psDrmDev->driver;

	dev_priv = (struct drm_psb_private *)psDrmDev->dev_private;
	fbdev = dev_priv->fbdev;
	if (fbdev != NULL)
		psbfb = fbdev->pfb;

	mutex_lock(&psDevInfo->sSwapChainMutex);

	psSwapChain->ui32SwapChainID = *pui32SwapChainID = iSCId+1;
	psSwapChain->ui32SwapChainPropertyFlag =
		PVRSRV_SWAPCHAIN_ATTACHED_PLANE_A
		| PVRSRV_SWAPCHAIN_ATTACHED_PLANE_B
		| PVRSRV_SWAPCHAIN_ATTACHED_PLANE_C;

	psSwapChain->ulSwapChainGTTOffset =
		(psbfb != NULL) ? psbfb->offset : 0;

	if(psDevInfo->psCurrentSwapChain == NULL)
		psDevInfo->psCurrentSwapChain = psSwapChain;

	psDevInfo->ui32SwapChainNum++;
	if(psDevInfo->ui32SwapChainNum == 1)
	{
		MRSTLFBEnableVSyncInterrupt(psDevInfo);
	}

	mutex_unlock(&psDevInfo->sSwapChainMutex);

	*phSwapChain = (IMG_HANDLE)psSwapChain;

	return (PVRSRV_OK);

ErrorFreeAllocatedBuffes:
	for (i = 0; i < ui32BufferCount; i++)
	{
		if(ppsBuffer[i] != NULL)
			MRSTLFBFreeBuffer( psDevInfo, &ppsBuffer[i] );
	}
	MRSTLFBFreeKernelMem(psVSyncFlips);
ErrorFreeBuffers:
	MRSTLFBFreeKernelMem(ppsBuffer);
ErrorFreeSwapChain:
	if(iSCId != MAX_SWAPCHAINS && psDevInfo->apsSwapChains[iSCId] == psSwapChain )
		psDevInfo->apsSwapChains[iSCId] = NULL;
	MRSTLFBFreeKernelMem(psSwapChain);

	return eError;
}

static PVRSRV_ERROR DestroyDCSwapChain(IMG_HANDLE hDevice,
	IMG_HANDLE hSwapChain)
{
	MRSTLFB_DEVINFO	*psDevInfo;
	MRSTLFB_SWAPCHAIN *psSwapChain;
	int i;
	IMG_UINT32 taskid;
	struct drm_device *dev;

	if(!hDevice || !hSwapChain)
	{
		return (PVRSRV_ERROR_INVALID_PARAMS);
	}
	psDevInfo = (MRSTLFB_DEVINFO*)hDevice;
	psSwapChain = (MRSTLFB_SWAPCHAIN*)hSwapChain;
	dev = psDevInfo->psDrmDevice;

	mutex_lock(&psDevInfo->sSwapChainMutex);

	psDevInfo->ui32SwapChainNum--;

	if(psDevInfo->ui32SwapChainNum == 0)
	{
		MRSTLFBDisableVSyncInterrupt(psDevInfo);
	}

	psDevInfo->apsSwapChains[ psSwapChain->ui32SwapChainID -1] = NULL;

	FlushInternalVSyncQueue(psSwapChain, psDevInfo->ui32SwapChainNum == 0);

	if (psDevInfo->ui32SwapChainNum == 0)
	{
		if (IS_CTP(dev))
			DRMLFBFlipBuffer2(psDevInfo, NULL, &psDevInfo->sSystemBuffer);
		else
			DRMLFBFlipBuffer(psDevInfo, NULL, &psDevInfo->sSystemBuffer);

		MRSTLFBClearSavedFlip(psDevInfo);

		if (IS_CTP(dev))
			DRMLFBFlipBlackScreen2(psDevInfo, IMG_FALSE);
		else
			DRMLFBFlipBlackScreen(psDevInfo, IMG_FALSE);
	}

	if (psDevInfo->psCurrentSwapChain == psSwapChain ||
		psDevInfo->ui32SwapChainNum == 0)
		psDevInfo->psCurrentSwapChain = NULL;

	mutex_unlock(&psDevInfo->sSwapChainMutex);


	if (psSwapChain->ulBufferCount)
		taskid = (psSwapChain->ppsBuffer[0])->ui32OwnerTaskID;

	for (i = 0; i < psSwapChain->ulBufferCount; i++)
	{
		MRSTLFBFreeBuffer(psDevInfo, &psSwapChain->ppsBuffer[i] );
	}

	if (psSwapChain->ulBufferCount)
		psb_gtt_free_ht_for_tgid(psDevInfo->psDrmDevice,
			taskid);

	MRSTLFBFreeKernelMem(psSwapChain->psVSyncFlips);
	MRSTLFBFreeKernelMem(psSwapChain->ppsBuffer);
	MRSTLFBFreeKernelMem(psSwapChain);
	return (PVRSRV_OK);
}

static PVRSRV_ERROR SetDCDstRect(IMG_HANDLE hDevice,
	IMG_HANDLE hSwapChain,
	IMG_RECT *psRect)
{
	UNREFERENCED_PARAMETER(hDevice);
	UNREFERENCED_PARAMETER(hSwapChain);
	UNREFERENCED_PARAMETER(psRect);



	return (PVRSRV_ERROR_NOT_SUPPORTED);
}

static PVRSRV_ERROR SetDCSrcRect(IMG_HANDLE hDevice,
                                 IMG_HANDLE hSwapChain,
                                 IMG_RECT *psRect)
{
	UNREFERENCED_PARAMETER(hDevice);
	UNREFERENCED_PARAMETER(hSwapChain);
	UNREFERENCED_PARAMETER(psRect);



	return (PVRSRV_ERROR_NOT_SUPPORTED);
}

static PVRSRV_ERROR SetDCDstColourKey(IMG_HANDLE hDevice,
                                      IMG_HANDLE hSwapChain,
                                      IMG_UINT32 ui32CKColour)
{
	UNREFERENCED_PARAMETER(hDevice);
	UNREFERENCED_PARAMETER(hSwapChain);
	UNREFERENCED_PARAMETER(ui32CKColour);



	return (PVRSRV_ERROR_NOT_SUPPORTED);
}

static PVRSRV_ERROR SetDCSrcColourKey(IMG_HANDLE hDevice,
                                      IMG_HANDLE hSwapChain,
                                      IMG_UINT32 ui32CKColour)
{
	UNREFERENCED_PARAMETER(hDevice);
	UNREFERENCED_PARAMETER(hSwapChain);
	UNREFERENCED_PARAMETER(ui32CKColour);



	return (PVRSRV_ERROR_NOT_SUPPORTED);
}

static PVRSRV_ERROR GetDCBuffers(IMG_HANDLE hDevice,
                                 IMG_HANDLE hSwapChain,
                                 IMG_UINT32 *pui32BufferCount,
                                 IMG_HANDLE *phBuffer)
{
	MRSTLFB_DEVINFO   *psDevInfo;
	MRSTLFB_SWAPCHAIN *psSwapChain;
	unsigned long      i;


	if(!hDevice
	|| !hSwapChain
	|| !pui32BufferCount
	|| !phBuffer)
	{
		return (PVRSRV_ERROR_INVALID_PARAMS);
	}

	psDevInfo = (MRSTLFB_DEVINFO*)hDevice;
	psSwapChain = (MRSTLFB_SWAPCHAIN*)hSwapChain;


	*pui32BufferCount = (IMG_UINT32)psSwapChain->ulBufferCount;


	for (i = 0; i < psSwapChain->ulBufferCount; i++)
	{
		phBuffer[i] = (IMG_HANDLE)psSwapChain->ppsBuffer[i];
	}

	return (PVRSRV_OK);
}

static PVRSRV_ERROR GetDCFrontBuffer(IMG_HANDLE hDevice,
                                 IMG_UINT32 *pui32SwapChainID,
                                 IMG_HANDLE *phBuffer)
{
	MRSTLFB_DEVINFO   *psDevInfo;

	if(!hDevice
	|| !pui32SwapChainID
	|| !phBuffer)
	{
		return (PVRSRV_ERROR_INVALID_PARAMS);
	}

	psDevInfo = (MRSTLFB_DEVINFO*)hDevice;

	*pui32SwapChainID = psDevInfo->psCurrentSwapChain ?
		psDevInfo->psCurrentSwapChain->ui32SwapChainID : 0;

	*phBuffer = (IMG_HANDLE)psDevInfo->psCurrentBuffer;

	return (PVRSRV_OK);
}

static PVRSRV_ERROR SwapToDCBuffer(IMG_HANDLE hDevice,
                                   IMG_HANDLE hBuffer,
                                   IMG_UINT32 ui32SwapInterval,
                                   IMG_HANDLE hPrivateTag,
                                   IMG_UINT32 ui32ClipRectCount,
                                   IMG_RECT *psClipRect)
{
	MRSTLFB_DEVINFO *psDevInfo;

	UNREFERENCED_PARAMETER(ui32SwapInterval);
	UNREFERENCED_PARAMETER(hPrivateTag);
	UNREFERENCED_PARAMETER(psClipRect);

	if(!hDevice
	|| !hBuffer
	|| (ui32ClipRectCount != 0))
	{
		return (PVRSRV_ERROR_INVALID_PARAMS);
	}

	psDevInfo = (MRSTLFB_DEVINFO*)hDevice;


	return (PVRSRV_OK);
}

static void timer_flip_handler(struct work_struct *work)
{
	MRSTLFB_DEVINFO *psDevInfo;
	MRSTLFB_SWAPCHAIN *psSwapChain;
	MRSTLFB_VSYNC_FLIP_ITEM *psLastItem;
	struct drm_psb_private *dev_priv;

	psDevInfo = container_of(work, MRSTLFB_DEVINFO, flip_complete_work);
	dev_priv =
		(struct drm_psb_private *)psDevInfo->psDrmDevice->dev_private;

	mutex_lock(&psDevInfo->sSwapChainMutex);
	psSwapChain = psDevInfo->psCurrentSwapChain;
	if (psSwapChain == NULL)
	{
		printk(KERN_WARNING "MRSTLFBFlipTimerFn: Swapchain is null\n");
		goto ExitUnlock;
	}
	if (psSwapChain->ulRemoveIndex == psSwapChain->ulInsertIndex)
	{
		psLastItem = &(psSwapChain->sLastItem);
		if (psLastItem->bValid && psLastItem->bFlipped && psLastItem->bCmdCompleted == MRST_FALSE)
		{
			MRSTFBFlipComplete(psSwapChain, NULL, MRST_TRUE);
		}
		goto ExitUnlock;
	}
	dev_priv->vsync_te_trouble_ts = cpu_clock(0);

	printk(KERN_WARNING "MRSTLFBFlipTimerFn: swapchain is not empty, flush queue\n");

	/*
	 * Release the spin lock here, as FlushInternalVSyncQueue() may invoke
	 * mutex_lock when freeing buffer.
	 */

	FlushInternalVSyncQueue(psSwapChain, MRST_FALSE);

	mutex_unlock(&psDevInfo->sSwapChainMutex);

	psb_flip_abnormal_debug_info(psDevInfo->psDrmDevice);

	return;

ExitUnlock:
	mutex_unlock(&psDevInfo->sSwapChainMutex);
	return;

}

static void MRSTLFBFlipTimerFn(unsigned long arg)
{
	MRSTLFB_DEVINFO *psDevInfo = (MRSTLFB_DEVINFO *)arg;

	schedule_work(&psDevInfo->flip_complete_work);
}

static MRST_BOOL MRSTLFBVSyncIHandler(MRSTLFB_DEVINFO *psDevInfo, int iPipe)
{
	MRSTLFB_VSYNC_FLIP_ITEM *psFlipItem;
	MRST_BOOL bStatus = MRST_TRUE;
	unsigned long ulMaxIndex;
	MRSTLFB_SWAPCHAIN *psSwapChain;
        struct drm_psb_private *dev_priv;
        int bhdmiplane_enable = IMG_TRUE;

	mutex_lock(&psDevInfo->sSwapChainMutex);


	psSwapChain = psDevInfo->psCurrentSwapChain;
	if (psSwapChain == NULL)
		goto ExitUnlock;

        //if hdmi connect,and hdmi plane disable,not flush commands
        if(psDevInfo->psDrmDevice){
                dev_priv =
                        (struct drm_psb_private *)psDevInfo->psDrmDevice->dev_private;

                if(dev_priv)
                        bhdmiplane_enable = dev_priv->bhdmi_enable;
        }

        if ((psDevInfo->bFlushCommands && !(hdmi_state && (bhdmiplane_enable == IMG_FALSE)))
                || psDevInfo->bSuspended || psDevInfo->bLeaveVT)
                goto ExitUnlock;


	psFlipItem = &psSwapChain->psVSyncFlips[psSwapChain->ulRemoveIndex];
	ulMaxIndex = psSwapChain->ulSwapChainLength - 1;

	while(psFlipItem->bValid)
	{
		if(psFlipItem->bFlipped)
		{
			if(!psFlipItem->bCmdCompleted)
			{
				MRST_BOOL bScheduleMISR;
				bScheduleMISR = MRST_TRUE;
				MRSTFBFlipComplete(psSwapChain, psFlipItem, MRST_TRUE);

				psFlipItem->bCmdCompleted = MRST_TRUE;
			}

			psFlipItem->ulSwapInterval--;

			if(psFlipItem->ulSwapInterval == 0)
			{
				psSwapChain->ulRemoveIndex++;

				if(psSwapChain->ulRemoveIndex > ulMaxIndex)
					psSwapChain->ulRemoveIndex = 0;

				psFlipItem->bCmdCompleted = MRST_FALSE;
				psFlipItem->bFlipped = MRST_FALSE;

				psFlipItem->bValid = MRST_FALSE;
			}
			else
			{

				break;
			}
		}
		else
		{
			if (psFlipItem->psBuffer)
				DRMLFBFlipBuffer(psDevInfo, psSwapChain,
						psFlipItem->psBuffer);
			else
				DRMLFBFlipBuffer2(psDevInfo, psSwapChain,
						&psFlipItem->sPlaneContexts);
			psFlipItem->bFlipped = MRST_TRUE;

			break;
		}


		psFlipItem = &psSwapChain->psVSyncFlips[psSwapChain->ulRemoveIndex];
	}
	if (psSwapChain->ulRemoveIndex != psSwapChain->ulInsertIndex) {
		mod_timer(&psDevInfo->sFlipTimer, FLIP_TIMEOUT + jiffies);
		bStatus = MRST_FALSE;
	}
ExitUnlock:
	mutex_unlock(&psDevInfo->sSwapChainMutex);

	return bStatus;
}

#if defined(MRST_USING_INTERRUPTS)
static int
MRSTLFBVSyncISR(struct drm_device *psDrmDevice, int iPipe)
{
	MRSTLFB_DEVINFO *psDevInfo = GetAnchorPtr();

	return MRSTLFBVSyncIHandler(psDevInfo, iPipe);
}
#endif

static IMG_BOOL updatePlaneContexts(MRSTLFB_SWAPCHAIN *psSwapChain,
				DISPLAYCLASS_FLIP_COMMAND2 *psFlipCmd,
				struct mdfld_plane_contexts *psPlaneContexts)
{
	int i;
	PDC_MEM_INFO psMemInfo, psCurrentMemInfo;
	MRSTLFB_BUFFER *psBuffer, *psCurrentBuffer;
	struct fb_info *psLINFBInfo;

	if (!psSwapChain || !psFlipCmd || !psPlaneContexts) {
		DRM_ERROR("Invalid parameters\n");
		return IMG_FALSE;
	}

	if (!psPlaneContexts->active_primaries)
		return IMG_TRUE;

	/* if active_primaries, update plane surface address*/
	psCurrentMemInfo = 0;

	for (i = 0; i < psFlipCmd->ui32NumMemInfos; i++) {
		psMemInfo = psFlipCmd->ppsMemInfos[i];

		if (!psMemInfo)
			continue;

		if (psMemInfo->memType == PVRSRV_MEMTYPE_DEVICECLASS) {
			psCurrentMemInfo = psMemInfo;
			break;
		}
	}

	if (!psCurrentMemInfo) {
		DRM_ERROR("Failed to get FB MemInfo\n");
		return IMG_FALSE;
	}

	/*get mrstlfb_buffer*/
	psCurrentBuffer = 0;
	for (i = 0; i < psSwapChain->ulBufferCount; i++) {
		psBuffer = psSwapChain->ppsBuffer[i];

		if (!psBuffer)
			continue;

		if (psBuffer->sCPUVAddr == psMemInfo->pvLinAddrKM) {
			psCurrentBuffer = psBuffer;
			psLINFBInfo = psSwapChain->psDevInfo->psLINFBInfo;
			psLINFBInfo->screen_base = psBuffer->sCPUVAddr;
			break;
		}
	}

	if (!psCurrentBuffer) {
		DRM_ERROR("Failed to get FB Buffer\n");
		return IMG_FALSE;
	}

	/*update primary context with fb surface address*/
	for (i = 0; i < INTEL_SPRITE_PLANE_NUM; i++) {
		if (!(psSwapChain->ui32SwapChainPropertyFlag & (1 << i))) {
			psPlaneContexts->active_primaries &= ~(1 << i);
			continue;
		}

		if (psPlaneContexts->active_primaries & (1 << i)) {
			psPlaneContexts->primary_contexts[i].surf =
				psCurrentBuffer->sDevVAddr.uiAddr;
		}
	}

	return IMG_TRUE;
}

static IMG_BOOL bIllegalFlipContexts(IMG_VOID *pvData)
{
	IMG_BOOL bIllegal = IMG_TRUE;
	DISPLAYCLASS_FLIP_COMMAND2 *psFlipCmd;
	struct drm_device *dev;
	struct drm_psb_private *dev_priv;
	MRSTLFB_DEVINFO *psDevInfo;
	struct mdfld_plane_contexts *psContexts;
	struct intel_sprite_context *psPrimaryContext;
	struct intel_sprite_context *psSpriteContext;
	struct intel_overlay_context *psOverlayContext;
	int i;

	psFlipCmd = (DISPLAYCLASS_FLIP_COMMAND2 *)pvData;
	if (!psFlipCmd) {
		DRM_ERROR("Invalid parameters\n");
		return IMG_TRUE;
	}
	psDevInfo = (MRSTLFB_DEVINFO *)psFlipCmd->hExtDevice;
	if (!psDevInfo) {
		DRM_ERROR("Invalid parameters\n");
		return IMG_TRUE;
	}
	dev = psDevInfo->psDrmDevice;
	dev_priv =
		(struct drm_psb_private *)psDevInfo->psDrmDevice->dev_private;

	psContexts = (struct mdfld_plane_contexts *)psFlipCmd->pvPrivData;

	if (!psContexts || !dev_priv) {
		DRM_ERROR("Invalid parameters\n");
		return IMG_TRUE;
	}

	/*check all active primary planes*/
	for (i = 0; i < INTEL_SPRITE_PLANE_NUM; i++) {
		if (psContexts->active_primaries & (1 << i)) {
			psPrimaryContext = &psContexts->primary_contexts[i];

			if (psPrimaryContext->index == 0 &&
				psDevInfo->bScreenState) {
				/* MIPI A off, should not flush PIPEA */
				psPrimaryContext->index = INVALID_INDEX;
			} else if (psPrimaryContext->index == 1 &&
					hdmi_state &&
					(dev_priv->early_suspended ||
					!dev_priv->bhdmi_enable)) {
				/* HDMI off, should not flush PIPEB */
				psPrimaryContext->index = INVALID_INDEX;
			} else if (psPrimaryContext->index == 2) {
				/* NULL unless PLANE C Enabled */
			} else
				bIllegal = IMG_FALSE;

		}
	}

	/*check all active sprite planes*/
	for (i = 0; i < INTEL_SPRITE_PLANE_NUM; i++) {
		if (psContexts->active_sprites & (1 << i)) {
			psSpriteContext = &psContexts->sprite_contexts[i];

			if (psSpriteContext->index == 0 &&
				psDevInfo->bScreenState) {
				/* MIPI A off, should not flush PIPEA */
				psSpriteContext->index = INVALID_INDEX;
			} else if (psSpriteContext->index == 1 &&
					hdmi_state &&
					(dev_priv->early_suspended ||
					!dev_priv->bhdmi_enable)) {
				/* HDMI off, should not flush PIPEB */
				psSpriteContext->index = INVALID_INDEX;
			} else if (psSpriteContext->index == 2) {
				/* NULL unless PLANE C Enabled */
			} else
				bIllegal = IMG_FALSE;
		}
	}

	/*check all active overlay planes*/
	for (i = 0; i < INTEL_OVERLAY_PLANE_NUM; i++) {
		if (psContexts->active_overlays & (1 << i)) {
			psOverlayContext = &psContexts->overlay_contexts[i];

			/* OVERLAY A/C have same policy */
			if (psOverlayContext->pipe == 0x00 &&
				psDevInfo->bScreenState) {
				psOverlayContext->index = INVALID_INDEX;
			} else if (psOverlayContext->pipe == 0x80 &&
					hdmi_state &&
					dev_priv->early_suspended) {
				psOverlayContext->index = INVALID_INDEX;
			} else
				bIllegal = IMG_FALSE;
		}
	}



	/* if all contexts are illegal, should not do flush */
	return bIllegal;
}


static IMG_BOOL ProcessFlip2(IMG_HANDLE hCmdCookie,
			IMG_UINT32 ui32DataSize,
			IMG_VOID *pvData)
{
	DISPLAYCLASS_FLIP_COMMAND2 *psFlipCmd;
	MRSTLFB_SWAPCHAIN *psSwapChain;
#if defined(MRST_USING_INTERRUPTS)
	MRSTLFB_VSYNC_FLIP_ITEM *psFlipItem;
#endif
	unsigned long irqflags;
	struct drm_device *dev;
	struct drm_psb_private *dev_priv;
	MRSTLFB_DEVINFO *psDevInfo;
	struct mdfld_plane_contexts *psPlaneContexts;
	struct mdfld_dsi_config *dsi_config;
	int contextlocked;
	int retry = MAX_TRANS_TIME_FOR_ONE_FRAME * SWAP_BUFFER_COUNT;

	psFlipCmd = (DISPLAYCLASS_FLIP_COMMAND2 *)pvData;
	psDevInfo = (MRSTLFB_DEVINFO *)psFlipCmd->hExtDevice;
	psSwapChain = (MRSTLFB_SWAPCHAIN *)psFlipCmd->hExtSwapChain;
	dev = psDevInfo->psDrmDevice;
	dev_priv =
		(struct drm_psb_private *)psDevInfo->psDrmDevice->dev_private;

	/*verify private data*/
	if (!psFlipCmd->pvPrivData ||
	psFlipCmd->ui32PrivDataLength != sizeof(struct mdfld_plane_contexts)) {
		DRM_ERROR("%s: Invalid private data\n", __func__);
		return IMG_FALSE;
	}

	dsi_config = dev_priv->dsi_configs[0];

	psPlaneContexts = (struct mdfld_plane_contexts *)psFlipCmd->pvPrivData;

	/* Firstly try to get lock; if failed, check ScreenState.
	 * If screen is off, no need to lock context_lock;otherwise
	 * it will try to get the lock. This is the way to avoid
	 * long time block caused by DPMS.
	 */
	contextlocked = mutex_trylock(&dsi_config->context_lock);
	if (!contextlocked) {
		if (!psDevInfo->bScreenState) {
			mutex_lock(&dsi_config->context_lock);
			contextlocked = 1;
		}
	}

	/*Screen is off, no need to send data*/
	if (bIllegalFlipContexts(pvData)) {
		mutex_lock(&psDevInfo->sSwapChainMutex);

		MRSTFBFlipComplete(psSwapChain, NULL, MRST_FALSE);
		psSwapChain->psPVRJTable->pfnPVRSRVCmdComplete(hCmdCookie,
				IMG_TRUE);

		mutex_unlock(&psDevInfo->sSwapChainMutex);
		mutex_unlock(&dsi_config->context_lock);
		return IMG_TRUE;
	}

	if (contextlocked)
		mdfld_dsi_dsr_forbid_locked(dsi_config);

        /*widi play video always use fake vsync in hwc.
         *video mode panel ,mipi on and have some flip cmd,
         *but mipi vblank interrupt disable,flip cmd can not
         * be completed will caused timeout.
        */
	if (dev_priv->exit_idle && (dsi_config->type == MDFLD_DSI_ENCODER_DPI))
		dev_priv->exit_idle(dev, MDFLD_DSR_2D_3D, NULL, true);

	/* wait for previous frame finished, otherwise
	 * if waiting at sending command, it will occupy CPU resource.
	 * For 60HZ, normaly the max wait will not bigger than
	 * 16ms, if wait time bigger then JB triple buffer, report
	 * fail.
	 */
	if (dsi_config->type == MDFLD_DSI_ENCODER_DBI) {
		if (!DRMLFBFifoEmpty(psDevInfo) && retry) {
			usleep_range(6000, 6500);
			retry--;
			while (!DRMLFBFifoEmpty(psDevInfo) && retry) {
				usleep_range(2000, 2500);
				retry--;
			}
		}
		if (!retry) {
			DRM_ERROR("FIFO never emptied\n");
			MRSTFBFlipComplete(psSwapChain, NULL, MRST_FALSE);
			psSwapChain->psPVRJTable->pfnPVRSRVCmdComplete(hCmdCookie,
					IMG_TRUE);
			if (contextlocked) {
				mdfld_dsi_dsr_allow_locked(dsi_config);
				mutex_unlock(&dsi_config->context_lock);
			}
			return IMG_TRUE;
		}
	}

	mutex_lock(&psDevInfo->sSwapChainMutex);

	/*update context*/
	updatePlaneContexts(psSwapChain, psFlipCmd, psPlaneContexts);

#if defined(MRST_USING_INTERRUPTS)

        /*
        **HDMI plug-in,Play video in OVERLAY_EXTEND mode
        **MIPI will off,bFlushCommands will be set to 1
        **pfnPVRSRVCmdComplete will be called immediately
        **after DRMLFBFlipBuffer2,Video will decode something
        **to the buffer which is displaying,so abnormal.
        **In normal mode,pfnPVRSRVCmdComplete will be called
        **In next vsync if the new buffer is displaying.
        */
	if (!drm_psb_3D_vblank || psFlipCmd->ui32SwapInterval == 0 ||
		(psDevInfo->bFlushCommands && !(hdmi_state && (dev_priv->bhdmi_enable == IMG_FALSE)))) {
#endif
		/* update sprite plane context*/
		if (DRMLFBFlipBuffer2(
			psDevInfo,
			 psSwapChain, psPlaneContexts) == IMG_FALSE) {
			DRM_INFO("%s: DRMLFBFlipBuffer2 failed\n", __func__);
			MRSTFBFlipComplete(psSwapChain, NULL, MRST_FALSE);
			psSwapChain->psPVRJTable->pfnPVRSRVCmdComplete(
					hCmdCookie, IMG_TRUE);
			goto ExitErrorUnlock;
		}
		MRSTFBFlipComplete(psSwapChain, NULL, MRST_FALSE);
		psSwapChain->psPVRJTable->pfnPVRSRVCmdComplete(hCmdCookie,
								IMG_TRUE);
#if defined(MRST_USING_INTERRUPTS)
		goto ExitTrueUnlock;
	}

	psFlipItem = &psSwapChain->psVSyncFlips[psSwapChain->ulInsertIndex];

	if (hdmi_state) {
		/*
		 * Enable HDMI vblank interrupt, otherwise page flip would stuck
		 * if both MIPI A and C are off.
		 */
		spin_lock_irqsave(&dev_priv->irqmask_lock, irqflags);
		mid_enable_pipe_event(dev_priv, 1);
		psb_enable_pipestat(dev_priv, 1, PIPE_VBLANK_INTERRUPT_ENABLE);
		spin_unlock_irqrestore(&dev_priv->irqmask_lock, irqflags);
	}

	if (psFlipItem->bValid == MRST_FALSE) {
		unsigned long ulMaxIndex = psSwapChain->ulSwapChainLength - 1;
		if (psSwapChain->ulInsertIndex == psSwapChain->ulRemoveIndex) {
			/*update sprite plane context*/
			if (DRMLFBFlipBuffer2(
				psDevInfo,
				 psSwapChain, psPlaneContexts) == IMG_FALSE) {
				DRM_DEBUG("%s: DRMLFBFlipBuffer2 failed\n", __func__);
				MRSTFBFlipComplete(psSwapChain, NULL, MRST_FALSE);
				psSwapChain->psPVRJTable->pfnPVRSRVCmdComplete(hCmdCookie,
						IMG_TRUE);
				goto ExitTrueUnlock;
			}
			psFlipItem->bFlipped = MRST_TRUE;
		} else {
			psFlipItem->bFlipped = MRST_FALSE;
		}

		/*start Flip watch dog*/
		mod_timer(&psDevInfo->sFlipTimer, FLIP_TIMEOUT + jiffies);

		psFlipItem->hCmdComplete = (MRST_HANDLE)hCmdCookie;
		psFlipItem->ulSwapInterval =
			(unsigned long)psFlipCmd->ui32SwapInterval;
		psFlipItem->psBuffer = NULL;
		/*copy plane contexts to this flip item*/
		memcpy(&psFlipItem->sPlaneContexts, psPlaneContexts,
			sizeof(struct mdfld_plane_contexts));
		psFlipItem->bValid = MRST_TRUE;

		psSwapChain->ulInsertIndex++;
		if (psSwapChain->ulInsertIndex > ulMaxIndex)
			psSwapChain->ulInsertIndex = 0;

		goto ExitTrueUnlock;
	}
ExitErrorUnlock:
	mutex_unlock(&psDevInfo->sSwapChainMutex);
	if (contextlocked) {
		mdfld_dsi_dsr_allow_locked(dsi_config);
		mutex_unlock(&dsi_config->context_lock);
	}
	return IMG_FALSE;
ExitTrueUnlock:
#endif
	mutex_unlock(&psDevInfo->sSwapChainMutex);
	if (contextlocked) {
		mdfld_dsi_dsr_allow_locked(dsi_config);
		mutex_unlock(&dsi_config->context_lock);
	}
	return IMG_TRUE;
}


static void ProcessFlip(IMG_HANDLE  hCmdCookie,
                            IMG_UINT32  ui32DataSize,
                            IMG_VOID   *pvData, IMG_BOOL bFlush)
{
	DISPLAYCLASS_FLIP_COMMAND *psFlipCmd;
	MRSTLFB_DEVINFO *psDevInfo;
	MRSTLFB_BUFFER *psBuffer;
	MRSTLFB_SWAPCHAIN *psSwapChain;
#if defined(MRST_USING_INTERRUPTS)
	MRSTLFB_VSYNC_FLIP_ITEM* psFlipItem;
#endif
	unsigned long irqflags;
	struct drm_device *dev;
	struct drm_psb_private *dev_priv;
	struct mdfld_dsi_config *dsi_config;
	int contextlocked;
	int retry = MAX_TRANS_TIME_FOR_ONE_FRAME * SWAP_BUFFER_COUNT;

	if(!hCmdCookie || !pvData)
		return ;

	psFlipCmd = (DISPLAYCLASS_FLIP_COMMAND*)pvData;

	if (psFlipCmd == IMG_NULL)
		return ;

	psDevInfo = (MRSTLFB_DEVINFO*)psFlipCmd->hExtDevice;
	dev = psDevInfo->psDrmDevice;
	dev_priv = (struct drm_psb_private *)
		psDevInfo->psDrmDevice->dev_private;
	psBuffer = (MRSTLFB_BUFFER*)psFlipCmd->hExtBuffer;
	psSwapChain = (MRSTLFB_SWAPCHAIN*) psFlipCmd->hExtSwapChain;

	/* bFlush == true means hw recovery */
	if (bFlush || (!psBuffer && bIllegalFlipContexts(pvData)) ||
		(psBuffer && psDevInfo->bScreenState)) {
		mutex_lock(&psDevInfo->sSwapChainMutex);
		MRSTFBFlipComplete(psSwapChain, NULL, MRST_FALSE);
		psSwapChain->psPVRJTable->pfnPVRSRVCmdComplete(hCmdCookie, IMG_TRUE);
		mutex_unlock(&psDevInfo->sSwapChainMutex);
		return ;
	}

	if (!dev_priv->um_start) {
		dev_priv->um_start = true;
		dev_priv->b_async_flip_enable = true;
		if (dev_priv->b_dsr_enable_config)
			dev_priv->b_dsr_enable = true;
	}

	if (!psBuffer) {
		ProcessFlip2(hCmdCookie, ui32DataSize, pvData);
		return ;
	}

	dsi_config = dev_priv->dsi_configs[0];

	contextlocked = mutex_trylock(&dsi_config->context_lock);
	if (!contextlocked) {
		if (!psDevInfo->bScreenState) {
			mutex_lock(&dsi_config->context_lock);
			contextlocked = 1;
		}
	}

	/* double check to make sure we don't send data when screen is off */
	if (psDevInfo->bScreenState) {
		mutex_lock(&psDevInfo->sSwapChainMutex);

		MRSTFBFlipComplete(psSwapChain, NULL, MRST_FALSE);
		psSwapChain->psPVRJTable->pfnPVRSRVCmdComplete(hCmdCookie,
				IMG_TRUE);

		mutex_unlock(&psDevInfo->sSwapChainMutex);
		mutex_unlock(&dsi_config->context_lock);
		return ;
	}

	if (contextlocked)
		mdfld_dsi_dsr_forbid_locked(dsi_config);

	if (dev_priv->exit_idle && (dsi_config->type == MDFLD_DSI_ENCODER_DPI))
		dev_priv->exit_idle(dev, MDFLD_DSR_2D_3D, NULL, true);

	/* Wait for previous frame finished, otherwise
	 * if waiting at sending command, it will occupy CPU resource
	 * For 60HZ, normaly the max wait will not bigger than
	 * 16ms, if wait time bigger then JB triple buffer, report
	 * fail.
	 */
	if (dsi_config->type == MDFLD_DSI_ENCODER_DBI) {
		while (!DRMLFBFifoEmpty(psDevInfo) && retry) {
			usleep_range(500, 1000);
			retry--;
		}
		if (!retry) {
			DRM_ERROR("FIFO never emptied\n");
			MRSTFBFlipComplete(psSwapChain, NULL, MRST_FALSE);
			psSwapChain->psPVRJTable->pfnPVRSRVCmdComplete(hCmdCookie,
					IMG_TRUE);
			if (contextlocked) {
				mdfld_dsi_dsr_allow_locked(dsi_config);
				mutex_unlock(&dsi_config->context_lock);
			}
			return ;
		}
	}

	mutex_lock(&psDevInfo->sSwapChainMutex);

#if defined(MRST_USING_INTERRUPTS)

    if(!drm_psb_3D_vblank || psFlipCmd->ui32SwapInterval == 0 || psDevInfo->bFlushCommands)
	{
#endif
		if (DRMLFBFlipBuffer(
			psDevInfo, psSwapChain, psBuffer) == IMG_FALSE) {
			DRM_INFO("%s: DRMLFBFlipBuffer failed\n", __func__);
			MRSTFBFlipComplete(psSwapChain, NULL, MRST_FALSE);
			psSwapChain->psPVRJTable->pfnPVRSRVCmdComplete(
					hCmdCookie, IMG_TRUE);
			goto Unlock;
		}

		MRSTFBFlipComplete(psSwapChain, NULL, MRST_FALSE);
		psSwapChain->psPVRJTable->pfnPVRSRVCmdComplete(hCmdCookie, IMG_TRUE);

#if defined(MRST_USING_INTERRUPTS)
		goto Unlock;
	}

	psFlipItem = &psSwapChain->psVSyncFlips[psSwapChain->ulInsertIndex];

	if (hdmi_state) {
		/*
		 * Enable HDMI vblank interrupt, otherwise page flip would stuck
		 * if both MIPI A and C are off.
		 */
		spin_lock_irqsave(&dev_priv->irqmask_lock, irqflags);
		mid_enable_pipe_event(dev_priv, 1);
		psb_enable_pipestat(dev_priv, 1, PIPE_VBLANK_INTERRUPT_ENABLE);
		spin_unlock_irqrestore(&dev_priv->irqmask_lock, irqflags);
	}

	if(psFlipItem->bValid == MRST_FALSE)
	{
		unsigned long ulMaxIndex = psSwapChain->ulSwapChainLength - 1;
		if(psSwapChain->ulInsertIndex == psSwapChain->ulRemoveIndex)
		{
			if (DRMLFBFlipBuffer(psDevInfo,
				 psSwapChain, psBuffer) == IMG_FALSE) {
				DRM_DEBUG("%s: DRMLFBFlipBuffer failed\n", __func__);
				MRSTFBFlipComplete(psSwapChain, NULL, MRST_FALSE);
				psSwapChain->psPVRJTable->pfnPVRSRVCmdComplete(hCmdCookie,
					IMG_TRUE);
				goto Unlock;
			}
			psFlipItem->bFlipped = MRST_TRUE;
		}
		else
		{
			psFlipItem->bFlipped = MRST_FALSE;
		}
		/*start Flip watch dog*/
		mod_timer(&psDevInfo->sFlipTimer, FLIP_TIMEOUT + jiffies);

		psFlipItem->hCmdComplete = (MRST_HANDLE)hCmdCookie;
		psFlipItem->ulSwapInterval = (unsigned long)psFlipCmd->ui32SwapInterval;
		psFlipItem->psBuffer = psBuffer;
		psFlipItem->bValid = MRST_TRUE;

		psSwapChain->ulInsertIndex++;
		if(psSwapChain->ulInsertIndex > ulMaxIndex)
		{
			psSwapChain->ulInsertIndex = 0;
		}

	}
Unlock:
#endif
	mutex_unlock(&psDevInfo->sSwapChainMutex);
	if (contextlocked) {
		mdfld_dsi_dsr_allow_locked(dsi_config);
		mutex_unlock(&dsi_config->context_lock);
	}
}


static void DisplayFlipWork(struct work_struct *work)
{
	u32 read_index;
	IMG_HANDLE  hCmdCookie;
	IMG_UINT32  ui32DataSize;
	IMG_BOOL bFlush;
	DISPLAYCLASS_FLIP_COMMAND2 vData;

	spin_lock(&display_flip_work_t.flip_commands_lock);

	while (display_flip_work_t.read_index !=
		display_flip_work_t.write_index) {
		read_index = display_flip_work_t.read_index;

		display_flip_work_t.read_index =
			(display_flip_work_t.read_index + 1) % MAXFLIPCOMMANDS;

		hCmdCookie =
			display_flip_work_t.
			p_flip_command[read_index].hCmdCookie;
		ui32DataSize =
			display_flip_work_t.
			p_flip_command[read_index].ui32DataSize;
		memcpy(&vData,
			&display_flip_work_t.p_flip_command[read_index].vData,
			sizeof(DISPLAYCLASS_FLIP_COMMAND2));
		bFlush =
			display_flip_work_t.p_flip_command[read_index].bFlush;
		spin_unlock(&display_flip_work_t.flip_commands_lock);
		ProcessFlip(hCmdCookie, ui32DataSize, &vData, bFlush);
		spin_lock(&display_flip_work_t.flip_commands_lock);
	}

	spin_unlock(&display_flip_work_t.flip_commands_lock);
}


static IMG_BOOL DisplayFlip(IMG_HANDLE  hCmdCookie,
		IMG_UINT32  ui32DataSize,
		IMG_VOID   *pvData, IMG_BOOL bFlush)
{
	u32 write_index;

	spin_lock(&display_flip_work_t.flip_commands_lock);

	write_index = display_flip_work_t.write_index;
	display_flip_work_t.p_flip_command[write_index].hCmdCookie = hCmdCookie;
	display_flip_work_t.p_flip_command[write_index].ui32DataSize =
		ui32DataSize;
	memcpy(&display_flip_work_t.p_flip_command[write_index].vData, pvData,
		sizeof(DISPLAYCLASS_FLIP_COMMAND2));
	display_flip_work_t.p_flip_command[write_index].bFlush = bFlush;
	display_flip_work_t.write_index =
		(display_flip_work_t.write_index + 1) % MAXFLIPCOMMANDS;

	spin_unlock(&display_flip_work_t.flip_commands_lock);

	if (!queue_work(system_nrt_wq, &display_flip_work_t.flip_work))
		DRM_INFO("Schedule work failed, too heavy system load?\n");

	return IMG_TRUE;
}

#if defined(PVR_MRST_FB_SET_PAR_ON_INIT)
static void MRSTFBSetPar(struct fb_info *psLINFBInfo)
{
	acquire_console_sem();

	if (psLINFBInfo->fbops->fb_set_par != NULL)
	{
		int res;

		res = psLINFBInfo->fbops->fb_set_par(psLINFBInfo);
		if (res != 0)
		{
			printk(KERN_WARNING DRIVER_PREFIX
				": fb_set_par failed: %d\n", res);

		}
	}
	else
	{
		printk(KERN_WARNING DRIVER_PREFIX
			": fb_set_par not set - HW cursor may not work\n");
	}

	release_console_sem();
}
#endif

void MRSTLFBSuspend(void)
{
	MRSTLFB_DEVINFO *psDevInfo = GetAnchorPtr();

	mutex_lock(&psDevInfo->sSwapChainMutex);

	if (!psDevInfo->bSuspended)
	{
#if !defined(PVR_MRST_STYLE_PM)
		if(psDevInfo->ui32SwapChainNum != 0)
		{
			MRSTLFBDisableVSyncInterrupt(psDevInfo);
		}
#endif
		psDevInfo->bSuspended = MRST_TRUE;
	}

	mutex_unlock(&psDevInfo->sSwapChainMutex);
}

void MRSTLFBResume(void)
{
	MRSTLFB_DEVINFO *psDevInfo = GetAnchorPtr();

	mutex_lock(&psDevInfo->sSwapChainMutex);

	if (psDevInfo->bSuspended)
	{
#if !defined(PVR_MRST_STYLE_PM)
		if(psDevInfo->ui32SwapChainNum != 0)
		{
			MRSTLFBEnableVSyncInterrupt(psDevInfo);
		}
#endif
		psDevInfo->bSuspended = MRST_FALSE;

		MRSTLFBRestoreLastFlip(psDevInfo);
	}

	mutex_unlock(&psDevInfo->sSwapChainMutex);

#if !defined(PVR_MRST_STYLE_PM)
	(void) UnblankDisplay(psDevInfo);
#endif
}

#ifdef DRM_PVR_USE_INTEL_FB
#include "mm.h"
static int MRSTLFBHandleChangeFB(struct drm_device* dev, struct psb_framebuffer *psbfb)
{
	MRSTLFB_DEVINFO *psDevInfo = GetAnchorPtr();
	int i;
	struct drm_psb_private * dev_priv;
	struct psb_gtt * pg;

	if( !psDevInfo->sSystemBuffer.bIsContiguous )
		MRSTLFBFreeKernelMem( psDevInfo->sSystemBuffer.uSysAddr.psNonCont );

	dev_priv = (struct drm_psb_private *)dev->dev_private;
	pg = dev_priv->pg;

	psDevInfo->sDisplayDim.ui32ByteStride = psbfb->base.pitches[0];
	psDevInfo->sDisplayDim.ui32Width = psbfb->base.width;
	psDevInfo->sDisplayDim.ui32Height = psbfb->base.height;

	psDevInfo->sSystemBuffer.ui32BufferSize = psbfb->size;

	psDevInfo->sSystemBuffer.sCPUVAddr = pg->vram_addr;

	psDevInfo->sSystemBuffer.sDevVAddr.uiAddr = 0;
	psDevInfo->sSystemBuffer.bIsAllocated = IMG_FALSE;

	if(psbfb->bo )
	{

		psDevInfo->sSystemBuffer.bIsContiguous = IMG_FALSE;
		psDevInfo->sSystemBuffer.uSysAddr.psNonCont = MRSTLFBAllocKernelMem( sizeof( IMG_SYS_PHYADDR ) * psbfb->bo->ttm->num_pages);
		if( psDevInfo->sSystemBuffer.uSysAddr.psNonCont == NULL )
		{
			printk(KERN_ERR "MRSTLFBAllocKernelMem fail\n");
			return 0;
		}

		for(i = 0;i < psbfb->bo->ttm->num_pages;++i)
		{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,3,0))
			struct page *p = ttm_tt_get_page( psbfb->bo->ttm, i);
#else
			struct page *p = psbfb->bo->ttm->pages[i];
#endif
			psDevInfo->sSystemBuffer.uSysAddr.psNonCont[i].uiAddr = page_to_pfn(p) << PAGE_SHIFT;

		}
	}
	else
	{





		psDevInfo->sSystemBuffer.bIsContiguous = IMG_TRUE;
		psDevInfo->sSystemBuffer.uSysAddr.sCont.uiAddr = pg->stolen_base;
	}

	return 0;
}
#else

static int MRSTLFBHandleChangeFB(struct drm_device* dev, struct psb_framebuffer *psbfb)
{
	MRSTLFB_DEVINFO *psDevInfo = GetAnchorPtr();
	int i;


	if( !psDevInfo->sSystemBuffer.bIsContiguous )
		MRSTLFBFreeKernelMem( psDevInfo->sSystemBuffer.uSysAddr.psNonCont );


	psDevInfo->sDisplayDim.ui32ByteStride = psbfb->base.pitches[0];
	psDevInfo->sDisplayDim.ui32Width = psbfb->base.width;
	psDevInfo->sDisplayDim.ui32Height = psbfb->base.height;

	psDevInfo->sSystemBuffer.ui32BufferSize = psbfb->buf.size;
	psDevInfo->sSystemBuffer.sCPUVAddr = psbfb->buf.kMapping;
	psDevInfo->sSystemBuffer.sDevVAddr.uiAddr = psbfb->buf.offsetGTT;
	psDevInfo->sSystemBuffer.bIsAllocated = MRST_FALSE;

	if ( psbfb->buf.type == PSB_BUFFER_VRAM )
	{

		struct drm_device * psDrmDevice = psDevInfo->psDrmDevice;
		struct drm_psb_private * dev_priv = (struct drm_psb_private *)psDrmDevice->dev_private;
		struct psb_gtt * pg = dev_priv->pg;

		psDevInfo->sSystemBuffer.bIsContiguous = MRST_TRUE;
		psDevInfo->sSystemBuffer.uSysAddr.sCont.uiAddr = pg->stolen_base;
	} else {

		psDevInfo->sSystemBuffer.bIsContiguous = MRST_FALSE;
		psDevInfo->sSystemBuffer.uSysAddr.psNonCont = MRSTLFBAllocKernelMem( sizeof( IMG_SYS_PHYADDR ) * (psbfb->buf.pagesNum));
		for (i = 0; i < psbfb->buf.pagesNum; i++)
		{
			psDevInfo->sSystemBuffer.uSysAddr.psNonCont[i].uiAddr = psbfb_get_buffer_pfn( psDevInfo->psDrmDevice, &psbfb->buf, i) << PAGE_SHIFT;
		}
	}

	return 0;
}
#endif

MRST_ERROR MRSTLFBChangeSwapChainProperty(unsigned long *psSwapChainGTTOffset,
		unsigned long ulSwapChainGTTSize, IMG_INT32 i32Pipe)
{
	MRSTLFB_DEVINFO *psDevInfo = GetAnchorPtr();
	struct drm_device *psDrmDevice = NULL;
	struct drm_psb_private *dev_priv = NULL;
	struct psb_gtt *pg = NULL;
	uint32_t tt_pages = 0;
	uint32_t max_gtt_offset = 0;

	int iSwapChainAttachedPlane = PVRSRV_SWAPCHAIN_ATTACHED_PLANE_NONE;
	IMG_UINT32 ui32SwapChainID = 0;
	unsigned long ulCurrentSwapChainGTTOffset = 0;
	MRST_ERROR eError = MRST_ERROR_GENERIC;

	if (psDevInfo == IMG_NULL) {
		DRM_ERROR("MRSTLFB hasn't been initialized, SGX unloaded?\n");
		/* Won't attach/de-attach the plane in case of no swap chain
		 * created. */
		eError = MRST_ERROR_INIT_FAILURE;
		return eError;
	}

	psDrmDevice = psDevInfo->psDrmDevice;
	dev_priv = (struct drm_psb_private *)psDrmDevice->dev_private;
	pg = dev_priv->pg;
	if (pg == NULL) {
		DRM_ERROR("Invalid GTT data.\n");
		return eError;
	}

	tt_pages = (pg->gatt_pages < PSB_TT_PRIV0_PLIMIT) ?
		(pg->gatt_pages) : PSB_TT_PRIV0_PLIMIT;

	/* Another half of GTT is managed by TTM. */
	tt_pages /= 2;
	max_gtt_offset = tt_pages << PAGE_SHIFT;

	if ((psSwapChainGTTOffset == IMG_NULL) ||
			(ulSwapChainGTTSize == 0) ||
			((*psSwapChainGTTOffset + ulSwapChainGTTSize) >
			 max_gtt_offset)) {
		DRM_ERROR("Invalid GTT offset.\n");
		return eError;
	}

	switch (i32Pipe) {
	case 0:
		iSwapChainAttachedPlane = PVRSRV_SWAPCHAIN_ATTACHED_PLANE_A;
		break;
	case 1:
		iSwapChainAttachedPlane = PVRSRV_SWAPCHAIN_ATTACHED_PLANE_B;
		break;
	case 2:
		iSwapChainAttachedPlane = PVRSRV_SWAPCHAIN_ATTACHED_PLANE_C;
		break;
	default:
		DRM_ERROR("Illegal Pipe Number.\n");
		return eError;
	}

	mutex_lock(&psDevInfo->sSwapChainMutex);

	if ((psDevInfo->apsSwapChains == IMG_NULL) ||
		(psDevInfo->apsSwapChains[ui32SwapChainID] == IMG_NULL)) {
		DRM_ERROR("No swap chain.\n");
		mutex_unlock(&psDevInfo->sSwapChainMutex);
		return eError;
	}

	ulCurrentSwapChainGTTOffset =
		psDevInfo->apsSwapChains[ui32SwapChainID]->ulSwapChainGTTOffset;

	for (ui32SwapChainID = 0; ui32SwapChainID < psDevInfo->ui32SwapChainNum;
			ui32SwapChainID++) {
		if (psDevInfo->apsSwapChains[ui32SwapChainID] == IMG_NULL)
			continue;

		if (*psSwapChainGTTOffset == ulCurrentSwapChainGTTOffset) {
			psDevInfo->apsSwapChains[ui32SwapChainID]->ui32SwapChainPropertyFlag |= iSwapChainAttachedPlane;

			/*
			 * Trigger the display plane to flip to the swap
			 * chain's last flip surface, to avoid that it still
			 * displays with original GTT offset after mode setting
			 * and attached to the specific swap chain.
			 */
			if (psDevInfo->bLastFlipAddrValid)
				*psSwapChainGTTOffset =
					psDevInfo->ulLastFlipAddr;
		}
		else
			psDevInfo->apsSwapChains[ui32SwapChainID]->ui32SwapChainPropertyFlag &= ~iSwapChainAttachedPlane;
	}

	mutex_unlock(&psDevInfo->sSwapChainMutex);

	eError = MRST_OK;
	return eError;
}

static int MRSTLFBFindMainPipe(struct drm_device *dev)
{
	struct drm_crtc *crtc;

	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head)
	{
		if ( drm_helper_crtc_in_use(crtc) )
		{
			struct psb_intel_crtc *psb_intel_crtc = to_psb_intel_crtc(crtc);
			return psb_intel_crtc->pipe;
		}
	}

	return 0;
}

#ifndef DRM_PVR_USE_INTEL_FB
static int DRMLFBLeaveVTHandler(struct drm_device *dev)
{
	MRSTLFB_DEVINFO *psDevInfo = GetAnchorPtr();

	mutex_lock(&psDevInfo->sSwapChainMutex);

	if (!psDevInfo->bLeaveVT)
	{
		if(psDevInfo->psCurrentSwapChain != NULL)
		{
			FlushInternalVSyncQueue(psDevInfo->psCurrentSwapChain, MRST_TRUE);

			SetFlushStateNoLock(psDevInfo, MRST_TRUE);
		}

		DRMLFBFlipBuffer(psDevInfo, NULL, &psDevInfo->sSystemBuffer);

		psDevInfo->bLeaveVT = MRST_TRUE;
	}

	mutex_unlock(&psDevInfo->sSwapChainMutex);

	return 0;
}

static int DRMLFBEnterVTHandler(struct drm_device *dev)
{
	MRSTLFB_DEVINFO *psDevInfo = GetAnchorPtr();

	mutex_lock(&psDevInfo->sSwapChainMutex);

	if (psDevInfo->bLeaveVT)
	{
		if(psDevInfo->psCurrentSwapChain != NULL)
		{
			SetFlushStateNoLock(psDevInfo, MRST_FALSE);
		}

		psDevInfo->bLeaveVT = MRST_FALSE;

		MRSTLFBRestoreLastFlip(psDevInfo);
	}

	mutex_unlock(&psDevInfo->sSwapChainMutex);

	return 0;
}
#endif
static
int MRSTLFBScreenEventHandler(struct drm_device* psDrmDevice, int state)
{
	MRSTLFB_DEVINFO *psDevInfo;
	MRST_BOOL bScreenOFF;

	psDevInfo = GetAnchorPtr();

	bScreenOFF = (state == 0) ? MRST_TRUE : MRST_FALSE;

	if (bScreenOFF != psDevInfo->bScreenState) {
		DRM_INFO("Screen event:%d\n", bScreenOFF);
		del_timer(&psDevInfo->sFlipTimer);
		psDevInfo->bScreenState = bScreenOFF;
		SetFlushState(psDevInfo, bScreenOFF);
	}

	return 0;
}

static MRST_ERROR MRSTLFBInstallScreenEvents(MRSTLFB_DEVINFO *psDevInfo, MRSTLFB_SCREEN_EVENT_PFN pScreenEventHandler)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *) psDevInfo->psDrmDevice->dev_private;
	dev_priv->pvr_screen_event_handler = pScreenEventHandler;
	return (MRST_OK);
}

static MRST_ERROR InitDev(MRSTLFB_DEVINFO *psDevInfo)
{
	MRST_ERROR eError = MRST_ERROR_GENERIC;
	struct fb_info *psLINFBInfo;
	struct drm_device * psDrmDevice = psDevInfo->psDrmDevice;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35))
	struct drm_psb_private * psDrmPrivate = (struct drm_psb_private *)psDrmDevice->dev_private;
	struct psb_fbdev * psPsbFBDev = (struct psb_fbdev *)psDrmPrivate->fbdev;
#endif
	struct drm_framebuffer * psDrmFB;
	struct psb_framebuffer *psbfb;


	int hdisplay;
	int vdisplay;
	int i;
	unsigned long FBSize;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35))
	psDrmFB = psPsbFBDev->psb_fb_helper.fb;
#else
	psDrmFB = list_first_entry(&psDrmDevice->mode_config.fb_kernel_list,
				   struct drm_framebuffer,
				   filp_head);
#endif
	if(!psDrmFB) {
		printk(KERN_INFO"%s: Cannot find drm FB",__FUNCTION__);
		return eError;
	}
	psbfb = to_psb_fb(psDrmFB);

	hdisplay = psDrmFB->width;
	vdisplay = psDrmFB->height;
	FBSize = psDrmFB->pitches[0] * psDrmFB->height;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35))
	psLINFBInfo = (struct fb_info*)psPsbFBDev->psb_fb_helper.fbdev;
#else
	psLINFBInfo = (struct fb_info*)psDrmFB->fbdev;
#endif

#if defined(PVR_MRST_FB_SET_PAR_ON_INIT)
	MRSTFBSetPar(psLINFBInfo);
#endif


	psDevInfo->sSystemBuffer.bIsContiguous = MRST_TRUE;
	psDevInfo->sSystemBuffer.bIsAllocated = MRST_FALSE;

	MRSTLFBHandleChangeFB(psDrmDevice, psbfb);

	switch( psDrmFB->depth )
	{
	case 32:
	case 24:
		{
			psDevInfo->sDisplayFormat.pixelformat = PVRSRV_PIXEL_FORMAT_ARGB8888;
			break;
		}
	case 16:
		{
			psDevInfo->sDisplayFormat.pixelformat = PVRSRV_PIXEL_FORMAT_RGB565;
			break;
		}
	default:
		{
			printk(KERN_ERR"%s: Unknown bit depth %d\n",__FUNCTION__,psDrmFB->depth);
		}
	}
	psDevInfo->psLINFBInfo = psLINFBInfo;

	psDevInfo->ui32MainPipe = MRSTLFBFindMainPipe(psDevInfo->psDrmDevice);

	for(i = 0;i < MAX_SWAPCHAINS;++i)
	{
		psDevInfo->apsSwapChains[i] = NULL;
	}




	psDevInfo->pvRegs = psbfb_vdc_reg(psDevInfo->psDrmDevice);

	if (psDevInfo->pvRegs == NULL)
	{
		eError = PVRSRV_ERROR_BAD_MAPPING;
		printk(KERN_WARNING DRIVER_PREFIX ": Couldn't map registers needed for flipping\n");
		return eError;
	}

	return MRST_OK;
}

static IMG_VOID MRSTQuerySwapCommand(IMG_HANDLE hDev, IMG_HANDLE hSwap, IMG_HANDLE hBuffer, IMG_HANDLE hTag, IMG_UINT16* ID, IMG_BOOL* bAddRef)
{
	UNREFERENCED_PARAMETER(hDev);
	UNREFERENCED_PARAMETER(hSwap);
	UNREFERENCED_PARAMETER(hBuffer);
	UNREFERENCED_PARAMETER(hTag);
	UNREFERENCED_PARAMETER(ID);
	*bAddRef = IMG_FALSE;
}


MRST_ERROR MRSTLFBInit(struct drm_device * dev)
{

	MRSTLFB_DEVINFO		*psDevInfo;
	struct drm_psb_private *psDrmPriv = (struct drm_psb_private *)dev->dev_private;

	psDevInfo = GetAnchorPtr();

	if (psDevInfo == NULL)
	{
		PFN_CMD_PROC	 		pfnCmdProcList[MRSTLFB_COMMAND_COUNT];
		IMG_UINT32				aui32SyncCountList[MRSTLFB_COMMAND_COUNT][2];

		psDevInfo = (MRSTLFB_DEVINFO *)MRSTLFBAllocKernelMem(sizeof(MRSTLFB_DEVINFO));

		if(!psDevInfo)
		{
			return (MRST_ERROR_OUT_OF_MEMORY);
		}


		memset(psDevInfo, 0, sizeof(MRSTLFB_DEVINFO));


		SetAnchorPtr((void*)psDevInfo);

		psDevInfo->psDrmDevice = dev;
		psDevInfo->ulRefCount = 0;

		if(InitDev(psDevInfo) != MRST_OK)
		{
			return (MRST_ERROR_INIT_FAILURE);
		}

		if(!psDrmPriv->pvr_ops->PVRGetDisplayClassJTable(
					&psDevInfo->sPVRJTable))
		{
			return (MRST_ERROR_INIT_FAILURE);
		}


		mutex_init(&psDevInfo->sSwapChainMutex);
		INIT_WORK(&psDevInfo->flip_complete_work, timer_flip_handler);

		psDevInfo->psCurrentSwapChain = NULL;
		psDevInfo->bFlushCommands = MRST_FALSE;

		psDevInfo->sDisplayInfo.ui32MaxSwapChainBuffers = 4;
		psDevInfo->sDisplayInfo.ui32MaxSwapChains = MAX_SWAPCHAINS;
		psDevInfo->sDisplayInfo.ui32MaxSwapInterval = 3;
		psDevInfo->sDisplayInfo.ui32MinSwapInterval = 0;

		strncpy(psDevInfo->sDisplayInfo.szDisplayName, DISPLAY_DEVICE_NAME, MAX_DISPLAY_NAME_SIZE);




		DEBUG_PRINTK((KERN_INFO DRIVER_PREFIX
			": Maximum number of swap chain buffers: %u\n",
			psDevInfo->sDisplayInfo.ui32MaxSwapChainBuffers));





		psDevInfo->sDCJTable.ui32TableSize = sizeof(PVRSRV_DC_SRV2DISP_KMJTABLE);
		psDevInfo->sDCJTable.pfnOpenDCDevice = OpenDCDevice;
		psDevInfo->sDCJTable.pfnCloseDCDevice = CloseDCDevice;
		psDevInfo->sDCJTable.pfnEnumDCFormats = EnumDCFormats;
		psDevInfo->sDCJTable.pfnEnumDCDims = EnumDCDims;
		psDevInfo->sDCJTable.pfnGetDCSystemBuffer = GetDCSystemBuffer;
		psDevInfo->sDCJTable.pfnGetDCInfo = GetDCInfo;
		psDevInfo->sDCJTable.pfnGetBufferAddr = GetDCBufferAddr;
		psDevInfo->sDCJTable.pfnCreateDCSwapChain = CreateDCSwapChain;
		psDevInfo->sDCJTable.pfnDestroyDCSwapChain = DestroyDCSwapChain;
		psDevInfo->sDCJTable.pfnSetDCDstRect = SetDCDstRect;
		psDevInfo->sDCJTable.pfnSetDCSrcRect = SetDCSrcRect;
		psDevInfo->sDCJTable.pfnSetDCDstColourKey = SetDCDstColourKey;
		psDevInfo->sDCJTable.pfnSetDCSrcColourKey = SetDCSrcColourKey;
		psDevInfo->sDCJTable.pfnGetDCBuffers = GetDCBuffers;
		psDevInfo->sDCJTable.pfnSwapToDCBuffer = SwapToDCBuffer;
		psDevInfo->sDCJTable.pfnSetDCState = SetDCState;


		if(psDevInfo->sPVRJTable.pfnPVRSRVRegisterDCDevice (
			&psDevInfo->sDCJTable,
			&psDevInfo->uiDeviceID ) != PVRSRV_OK)
		{
			return (MRST_ERROR_DEVICE_REGISTER_FAILED);
		}

		printk("Device ID: %d\n", (int)psDevInfo->uiDeviceID);











#if defined (MRST_USING_INTERRUPTS)

	if(MRSTLFBInstallVSyncISR(psDevInfo,MRSTLFBVSyncISR) != MRST_OK)
	{
		DEBUG_PRINTK((KERN_INFO DRIVER_PREFIX	"ISR Installation failed\n"));
		return (MRST_ERROR_INIT_FAILURE);
	}
#endif


	INIT_WORK(&display_flip_work_t.flip_work, DisplayFlipWork);
	display_flip_work_t.read_index = 0;
	display_flip_work_t.write_index = 0;
	spin_lock_init(&display_flip_work_t.flip_commands_lock);

	pfnCmdProcList[DC_FLIP_COMMAND] = DisplayFlip;


	aui32SyncCountList[DC_FLIP_COMMAND][0] = 0;
	aui32SyncCountList[DC_FLIP_COMMAND][1] = 10;





	if (psDevInfo->sPVRJTable.pfnPVRSRVRegisterCmdProcList (psDevInfo->uiDeviceID,
								&pfnCmdProcList[0],
								aui32SyncCountList,
								MRSTLFB_COMMAND_COUNT) != PVRSRV_OK)
	  {
	    printk(KERN_WARNING DRIVER_PREFIX ": Can't register callback\n");
	    return (MRST_ERROR_CANT_REGISTER_CALLBACK);
	  }


	}


#ifndef DRM_PVR_USE_INTEL_FB
	psDrmPriv->psb_change_fb_handler = MRSTLFBHandleChangeFB;

	psDrmPriv->psb_leave_vt_handler = DRMLFBLeaveVTHandler;
	psDrmPriv->psb_enter_vt_handler = DRMLFBEnterVTHandler;
#endif
    MRSTLFBInstallScreenEvents(psDevInfo, MRSTLFBScreenEventHandler);

	psDevInfo->ulRefCount++;

	psDevInfo->sFlipTimer.data = (unsigned long)psDevInfo;
	psDevInfo->sFlipTimer.function = MRSTLFBFlipTimerFn;
	init_timer(&psDevInfo->sFlipTimer);
	return (MRST_OK);
}

MRST_ERROR MRSTLFBDeinit(void)
{
	MRSTLFB_DEVINFO *psDevInfo, *psDevFirst;

	psDevFirst = GetAnchorPtr();
	psDevInfo = psDevFirst;


	if (psDevInfo == NULL)
	{
		return (MRST_ERROR_GENERIC);
	}


	psDevInfo->ulRefCount--;

	if (psDevInfo->ulRefCount == 0)
	{

		PVRSRV_DC_DISP2SRV_KMJTABLE	*psJTable = &psDevInfo->sPVRJTable;

		if (psDevInfo->sPVRJTable.pfnPVRSRVRemoveCmdProcList (psDevInfo->uiDeviceID, MRSTLFB_COMMAND_COUNT) != PVRSRV_OK)
		{
			return (MRST_ERROR_GENERIC);
		}

#if defined (MRST_USING_INTERRUPTS)

		if(MRSTLFBUninstallVSyncISR(psDevInfo) != MRST_OK)
		{
			return (MRST_ERROR_GENERIC);
		}
#endif


		if (psJTable->pfnPVRSRVRemoveDCDevice(psDevInfo->uiDeviceID) != PVRSRV_OK)
		{
			return (MRST_ERROR_GENERIC);
		}


		MRSTLFBFreeKernelMem(psDevInfo);
	}


	SetAnchorPtr(NULL);


	return (MRST_OK);
}



static MRST_ERROR MRSTLFBAllocBuffer(struct MRSTLFB_DEVINFO_TAG *psDevInfo, IMG_UINT32 ui32Size, MRSTLFB_BUFFER **ppBuffer)
{
	IMG_VOID *pvBuf;
	IMG_UINT32 ulPagesNumber;
	IMG_UINT32 ulCounter;
	int i;

	pvBuf = __vmalloc( ui32Size, GFP_KERNEL | __GFP_HIGHMEM, __pgprot((pgprot_val(PAGE_KERNEL ) & ~_PAGE_CACHE_MASK) | _PAGE_CACHE_WC) );
	if( pvBuf == NULL )
	{
		return MRST_ERROR_OUT_OF_MEMORY;
	}

	ulPagesNumber = (ui32Size + PAGE_SIZE -1) / PAGE_SIZE;

	*ppBuffer = MRSTLFBAllocKernelMem( sizeof( MRSTLFB_BUFFER ) );
	if( *ppBuffer == NULL )
	{
		printk(KERN_ERR "MRSTLFBAllocKernelMem fail\n");
		return MRST_ERROR_GENERIC;
	}

	(*ppBuffer)->sCPUVAddr = pvBuf;
	(*ppBuffer)->ui32BufferSize = ui32Size;
	(*ppBuffer)->uSysAddr.psNonCont = MRSTLFBAllocKernelMem( sizeof( IMG_SYS_PHYADDR ) * ulPagesNumber);
	if( (*ppBuffer)->uSysAddr.psNonCont == NULL )
	{
		printk(KERN_ERR "MRSTLFBAllocKernelMem fail\n");
		return MRST_ERROR_GENERIC;
	}

	(*ppBuffer)->bIsAllocated = MRST_TRUE;
	(*ppBuffer)->bIsContiguous = MRST_FALSE;
	(*ppBuffer)->ui32OwnerTaskID = task_tgid_nr(current);

	i = 0;
	for (ulCounter = 0; ulCounter < ui32Size; ulCounter += PAGE_SIZE)
	{
		(*ppBuffer)->uSysAddr.psNonCont[i++].uiAddr = vmalloc_to_pfn( pvBuf + ulCounter ) << PAGE_SHIFT;
	}

	psb_gtt_map_pvr_memory( psDevInfo->psDrmDevice,
							(unsigned int)*ppBuffer,
							(*ppBuffer)->ui32OwnerTaskID,
							(IMG_CPU_PHYADDR*) (*ppBuffer)->uSysAddr.psNonCont,
							ulPagesNumber,
							&(*ppBuffer)->sDevVAddr.uiAddr );

	(*ppBuffer)->sDevVAddr.uiAddr <<= PAGE_SHIFT;

   	return MRST_OK;
}

static MRST_ERROR MRSTLFBFreeBuffer(struct MRSTLFB_DEVINFO_TAG *psDevInfo, MRSTLFB_BUFFER **ppBuffer)
{
	if( !(*ppBuffer)->bIsAllocated )
		return MRST_ERROR_INVALID_PARAMS;

#ifndef DRM_PVR_USE_INTEL_FB
	psb_gtt_unmap_memory( psDevInfo->psDrmDevice,
#else
	psb_gtt_unmap_pvr_memory( psDevInfo->psDrmDevice,
#endif
							  (unsigned int)*ppBuffer,
							  (*ppBuffer)->ui32OwnerTaskID);

	vfree( (*ppBuffer)->sCPUVAddr );

	MRSTLFBFreeKernelMem( (*ppBuffer)->uSysAddr.psNonCont );

	MRSTLFBFreeKernelMem( *ppBuffer);

	*ppBuffer = NULL;

	return MRST_OK;
}




