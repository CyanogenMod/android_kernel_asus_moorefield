/******************************************************************************
 *
 * Copyright (c) 2011, Intel Corporation.
 * Portions (c), Imagination Technology, Ltd.
 * All rights reserved.
 *
 * Redistribution and Use.  Redistribution and use in binary form, without
 * modification, of the software code provided with this license ("Software"),
 * are permitted provided that the following conditions are met:
 *
 *  1. Redistributions must reproduce the above copyright notice and this
 *     license in the documentation and/or other materials provided with the
 *     Software.
 *  2. Neither the name of Intel Corporation nor the name of Imagination
 *     Technology, Ltd may be used to endorse or promote products derived from
 *     the Software without specific prior written permission.
 *  3. The Software can only be used in connection with the Intel hardware
 *     designed to use the Software as outlined in the documentation. No other
 *     use is authorized.
 *  4. No reverse engineering, decompilation, or disassembly of the Software
 *     is permitted.
 *  5. The Software may not be distributed under terms different than this
 *     license.
 *
 * Limited Patent License.  Intel Corporation grants a world-wide, royalty-free
 * , non-exclusive license under patents it now or hereafter owns or controls
 * to make, have made, use, import, offer to sell and sell ("Utilize") the
 * Software, but solely to the extent that any such patent is necessary to
 * Utilize the Software alone.  The patent license shall not apply to any
 * combinations which include the Software.  No hardware per se is licensed
 * hereunder.
 *
 * Ownership of Software and Copyrights. Title to all copies of the Software
 * remains with the copyright holders. The Software is copyrighted and
 * protected by the laws of the United States and other countries, and
 * international treaty provisions.
 *
 * DISCLAIMER.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <linux/version.h>

#include "psb_drv.h"
#include "psb_intel_reg.h"
#include "psb_dpst.h"
#include "dispmgrnl.h"
#include "psb_dpst_func.h"
#include "mdfld_dsi_dbi_dsr.h"

/*	ASUS_BSP: Louis ++	*/
#include <linux/HWVersion.h>
extern int Read_HW_ID(void);
/*	ASUS_BSP: Louis --	*/

static struct drm_device *g_dev;   /* hack for the queue */
static int blc_adj2;
static u32 lut_adj[256];

#ifdef CONFIG_A500CG
int init_done = 0;
static int current_level = -1;
#endif

void dpst_disable_post_process(struct drm_device *dev);

int send_hist(void)
{
	struct dispmgr_command_hdr dispmgr_cmd;
	struct drm_psb_hist_status_arg mydata;

	/* before we send get the status for run_algorithm */
	dpst_histogram_get_status(g_dev, &mydata);

	dispmgr_cmd.module = DISPMGR_MOD_DPST;
	dispmgr_cmd.cmd = DISPMGR_DPST_HIST_DATA;
	dispmgr_cmd.data_size = sizeof(struct drm_psb_hist_status_arg);
	dispmgr_cmd.data = &mydata;
	dispmgr_nl_send_msg(&dispmgr_cmd);

	return 0;
}

#ifdef CONFIG_A500CG
   /*
    * Implement the workaround for DIET when HDMI plugin.
    * If HDMI not plugin then enable the DIET for dpst ,others disable the dpst
    */
void disable_diet(int disable)
{
	if(init_done) {
		if(disable){ //HDMI plug in
			if(dpst_level > 0)
				current_level = dpst_level;
			dpst_level = 0;
			psb_irq_disable_dpst(g_dev);
		}else {
			if(current_level != -1) {
			dpst_level = current_level; //restore the level last time
				current_level = -1;
			}
			psb_irq_enable_dpst(g_dev);
		}
                psb_diet_enable(g_dev, 0);
	}
}
#endif

/* IOCTL - moved to standard calls for Kernel Integration */

int psb_hist_enable(struct drm_device *dev, void *data)
{
	u32 irqCtrl = 0;
	struct drm_psb_private *dev_priv = psb_priv(dev);
	struct dpst_guardband guardband_reg;
	struct dpst_ie_histogram_control ie_hist_cont_reg;
	struct mdfld_dsi_hw_context *ctx = NULL;
	uint32_t *enable = data;
	struct mdfld_dsi_config *dsi_config = NULL;

	if (!dev_priv)
		return 0;
	dsi_config = dev_priv->dsi_configs[0];
	if (!dsi_config)
		return 0;
	ctx = &dsi_config->dsi_hw_context;

	if (!ospm_power_using_hw_begin
	    (OSPM_DISPLAY_ISLAND, OSPM_UHB_FORCE_POWER_ON)) {
		return 0;
	}
	mdfld_dsi_dsr_forbid(dsi_config);

	if (*enable == 1) {
		ie_hist_cont_reg.data = PSB_RVDC32(HISTOGRAM_LOGIC_CONTROL);
		ie_hist_cont_reg.ie_pipe_assignment = 0;
		ie_hist_cont_reg.histogram_mode_select = DPST_YUV_LUMA_MODE;
		ie_hist_cont_reg.ie_histogram_enable = 1;
		PSB_WVDC32(ie_hist_cont_reg.data, HISTOGRAM_LOGIC_CONTROL);
		ctx->histogram_logic_ctrl = ie_hist_cont_reg.data;

		guardband_reg.data = PSB_RVDC32(HISTOGRAM_INT_CONTROL);
		guardband_reg.interrupt_enable = 1;
		guardband_reg.interrupt_status = 1;
		PSB_WVDC32(guardband_reg.data, HISTOGRAM_INT_CONTROL);
		ctx->histogram_intr_ctrl = guardband_reg.data;

		irqCtrl = PSB_RVDC32(PIPEASTAT);
		ctx->pipestat = (irqCtrl | PIPE_DPST_EVENT_ENABLE);
		PSB_WVDC32(ctx->pipestat, PIPEASTAT);
	} else {
		guardband_reg.data = PSB_RVDC32(HISTOGRAM_INT_CONTROL);
		guardband_reg.interrupt_enable = 0;
		guardband_reg.interrupt_status = 1;
		PSB_WVDC32(guardband_reg.data, HISTOGRAM_INT_CONTROL);
		ctx->histogram_intr_ctrl = guardband_reg.data;

		ie_hist_cont_reg.data = PSB_RVDC32(HISTOGRAM_LOGIC_CONTROL);
		ie_hist_cont_reg.ie_histogram_enable = 0;
		PSB_WVDC32(ie_hist_cont_reg.data, HISTOGRAM_LOGIC_CONTROL);
		ctx->histogram_logic_ctrl = ie_hist_cont_reg.data;

		irqCtrl = PSB_RVDC32(PIPEASTAT);
		irqCtrl &= ~PIPE_DPST_EVENT_ENABLE;
		PSB_WVDC32(irqCtrl, PIPEASTAT);
		ctx->pipestat = irqCtrl;
		dpst_disable_post_process(g_dev);
	}

	mdfld_dsi_dsr_allow(dsi_config);
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);

	return 0;
}

static int psb_hist_status(struct drm_device *dev, void *data)
{
	struct drm_psb_private *dev_priv = psb_priv(dev);
	struct drm_psb_hist_status_arg *hist_status = data;
	uint32_t *arg = hist_status->buf;
	struct mdfld_dsi_config *dsi_config = NULL;
	u32 iedbr_reg_data = 0;
	struct dpst_ie_histogram_control ie_hist_cont_reg;
	u32 i;
	int dpst3_bin_threshold_count = 0;
	uint32_t blm_hist_ctl = HISTOGRAM_LOGIC_CONTROL;
	uint32_t iebdr_reg = HISTOGRAM_BIN_DATA;
	uint32_t segvalue_max_22_bit = 0x3fffff;
	uint32_t iedbr_busy_bit = 0x80000000;
	int dpst3_bin_count = 32;

	if (!dev_priv)
		return 0;
	dsi_config = dev_priv->dsi_configs[0];
	if (!dsi_config)
		return 0;

	if (!ospm_power_using_hw_begin
	    (OSPM_DISPLAY_ISLAND, OSPM_UHB_FORCE_POWER_ON)) {
		return 0;
	}
	mdfld_dsi_dsr_forbid(dsi_config);

	ie_hist_cont_reg.data = PSB_RVDC32(blm_hist_ctl);
	ie_hist_cont_reg.bin_reg_func_select = dpst3_bin_threshold_count;
	ie_hist_cont_reg.bin_reg_index = 0;

	PSB_WVDC32(ie_hist_cont_reg.data, blm_hist_ctl);

	for (i = 0; i < dpst3_bin_count; i++) {
		iedbr_reg_data = PSB_RVDC32(iebdr_reg);

		if (!(iedbr_reg_data & iedbr_busy_bit)) {
			arg[i] = iedbr_reg_data & segvalue_max_22_bit;
		} else {
			i = 0;
			ie_hist_cont_reg.data = PSB_RVDC32(blm_hist_ctl);
			ie_hist_cont_reg.bin_reg_index = 0;
			PSB_WVDC32(ie_hist_cont_reg.data, blm_hist_ctl);
		}
	}
	mdfld_dsi_dsr_allow(dsi_config);

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);

	return 0;
}

/* SH START DIET */
/* SH TODO This doesn't work yet. */
int psb_diet_enable(struct drm_device *dev, void *data)
{
	struct drm_psb_private *dev_priv = psb_priv(dev);
	struct mdfld_dsi_config *dsi_config = NULL;
	struct mdfld_dsi_hw_context *ctx = NULL;
	uint32_t *arg = data;
	struct dpst_ie_histogram_control ie_hist_cont_reg;
	u32 i;
	int dpst3_bin_threshold_count = 1;
	int dpst_hsv_multiplier = 2;
	uint32_t blm_hist_ctl = HISTOGRAM_LOGIC_CONTROL;
	uint32_t iebdr_reg = HISTOGRAM_BIN_DATA;
	int dpst3_bin_count = 32;

	if (!dev_priv)
		return 0;
	dsi_config = dev_priv->dsi_configs[0];
	if (!dsi_config)
		return 0;
	ctx = &dsi_config->dsi_hw_context;

	if (!ospm_power_using_hw_begin
	    (OSPM_DISPLAY_ISLAND, OSPM_UHB_ONLY_IF_ON))
		return 0;

	mdfld_dsi_dsr_forbid(dsi_config);

	if (data) {
		ie_hist_cont_reg.data = PSB_RVDC32(blm_hist_ctl);
		ie_hist_cont_reg.bin_reg_func_select =
		    dpst3_bin_threshold_count;
		ie_hist_cont_reg.bin_reg_index = 0;

		PSB_WVDC32(ie_hist_cont_reg.data, blm_hist_ctl);

		for (i = 0; i < dpst3_bin_count; i++)
			PSB_WVDC32(arg[i], iebdr_reg);
		ctx->aimg_enhance_bin = PSB_RVDC32(iebdr_reg);

		ie_hist_cont_reg.data = PSB_RVDC32(blm_hist_ctl);
		ie_hist_cont_reg.ie_mode_table_enabled = 1;
		ie_hist_cont_reg.alt_enhancement_mode = dpst_hsv_multiplier;

		PSB_WVDC32(ie_hist_cont_reg.data, blm_hist_ctl);
		ctx->histogram_logic_ctrl = ie_hist_cont_reg.data;
	} else {
		ie_hist_cont_reg.data = PSB_RVDC32(blm_hist_ctl);
		ie_hist_cont_reg.ie_mode_table_enabled = 0;

		PSB_WVDC32(ie_hist_cont_reg.data, blm_hist_ctl);
		ctx->histogram_logic_ctrl = ie_hist_cont_reg.data;
	}

	mdfld_dsi_dsr_allow(dsi_config);
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);

	return 0;
}

/* SH END */

int psb_init_comm(struct drm_device *dev, void *data)
{
	struct drm_psb_private *dev_priv = psb_priv(dev);
	struct pci_dev *pdev = NULL;
	struct device *ddev = NULL;
	struct kobject *kobj = NULL;
	uint32_t *arg = data;

	if (*arg == 1) {
		/*find handle to drm kboject */
		pdev = dev->pdev;
		ddev = &pdev->dev;
		kobj = &ddev->kobj;

		if (dev_priv->psb_dpst_state == NULL) {
			/*init dpst kmum comms */
			dev_priv->psb_dpst_state = psb_dpst_init(kobj);
		}

		psb_irq_enable_dpst(dev);
		psb_dpst_notify_change_um(DPST_EVENT_INIT_COMPLETE,
					  dev_priv->psb_dpst_state);
	} else {
		/*hotplug and dpst destroy examples */
		psb_irq_disable_dpst(dev);
		psb_dpst_notify_change_um(DPST_EVENT_TERMINATE,
					  dev_priv->psb_dpst_state);
		psb_dpst_device_pool_destroy(dev_priv->psb_dpst_state);
		dev_priv->psb_dpst_state = NULL;
	}
	return 0;
}

int psb_dpst_mode(struct drm_device *dev, void *data)
{
	struct drm_psb_private *dev_priv = psb_priv(dev);
	struct mdfld_dsi_config *dsi_config = NULL;
	uint32_t *arg = data;
	uint32_t x = 0;
	uint32_t y = 0;
	uint32_t reg;

	if (!dev_priv)
		return 0;

	dsi_config = dev_priv->dsi_configs[0];
	if (!dsi_config)
		return 0;
	mdfld_dsi_dsr_forbid(dsi_config);

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				OSPM_UHB_ONLY_IF_ON))
		return 0;

	reg = PSB_RVDC32(PIPEASRC);

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);

	mdfld_dsi_dsr_allow(dsi_config);
	/* horizontal is the left 16 bits */
	x = reg >> 16;
	/* vertical is the right 16 bits */
	y = reg & 0x0000ffff;

	/* the values are the image size minus one */
	x += 1;
	y += 1;

	*arg = (x << 16) | y;

	return 0;
}

int psb_dpst_get_level_ioctl(struct drm_device *dev, void *data,
		struct drm_file *file_priv)
{
	uint32_t *arg = data;

	if (dpst_level < AGGRESSIVE_LEVEL_MIN ||
			dpst_level > AGGRESSIVE_LEVEL_MAX)
		dpst_level = AGGRESSIVE_LEVEL_DEFAULT;

#ifdef CONFIG_SUPPORT_DDS_MIPI_SWITCH
	if (panel_id == DDS_PAD || (panel_id == DDS_PHONE && Read_HW_ID() >= HW_ID_ER2)) {
		*arg = AGGRESSIVE_LEVEL_MIN;
		PSB_DEBUG_ENTRY("[Display] Turn off DPST\n");
		return AGGRESSIVE_LEVEL_MIN;
	}
#endif

	*arg = dpst_level;

	return dpst_level;
}

int psb_update_guard(struct drm_device *dev, void *data)
{
	struct drm_psb_private *dev_priv = psb_priv(dev);
	struct dpst_guardband *input = (struct dpst_guardband *)data;
	struct mdfld_dsi_config *dsi_config = NULL;
	struct mdfld_dsi_hw_context *ctx = NULL;
	struct dpst_guardband reg_data;

	if (!dev_priv)
		return 0;
	dsi_config = dev_priv->dsi_configs[0];
	if (!dsi_config)
		return 0;
	ctx = &dsi_config->dsi_hw_context;

	if (!ospm_power_using_hw_begin
	    (OSPM_DISPLAY_ISLAND, OSPM_UHB_FORCE_POWER_ON))
		return 0;

	mdfld_dsi_dsr_forbid(dsi_config);

	reg_data.data = PSB_RVDC32(HISTOGRAM_INT_CONTROL);
	reg_data.guardband = input->guardband;
	reg_data.guardband_interrupt_delay = input->guardband_interrupt_delay;

	PSB_WVDC32(reg_data.data, HISTOGRAM_INT_CONTROL);
	ctx->histogram_intr_ctrl = reg_data.data;

	mdfld_dsi_dsr_allow(dsi_config);

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);

	return 0;
}

int dpst_disable(struct drm_device *dev)
{
	uint32_t enable = 0;
	int ret = 0;

	ret = psb_init_comm(dev, &enable);

	return ret;
}

void dpst_process_event(struct umevent_obj *notify_disp_obj, int dst_group_id)
{
	int messageType;
	int do_not_quit = 1;

	messageType = notify_disp_obj->kobj.name[0];
	/* need to debug to figure out which field this is */

	switch (messageType) {
	case 'i':		/* DPST_EVENT_INIT_COMPLETE: */
	case 'h':		/* DPST_EVENT_HIST_INTERRUPT: */
		/* DPST histogram */
		send_hist();
		break;
	case 'p':		/* DPST_EVENT_PHASE_COMPLETE: */
		break;
	case 't':		/* DPST_EVENT_TERMINATE: */
		break;
	default:
		/* disable DPST */
		do_not_quit = 0;
		break;
	}
}

int dpst_histogram_enable(struct drm_device *dev, int enable)
{
	int ret = 0;

	/* enable histogram interrupts */
	ret = psb_hist_enable(dev, &enable);
	return ret;
}

int dpst_histogram_get_status(struct drm_device *dev,
			      struct drm_psb_hist_status_arg *hist_data)
{
	int ret = 0;
	ret = psb_hist_status(dev, hist_data);
	if (ret) {
		pr_err("Histogram get status ioctl returned error\n");
		return 1;
	}
	return 0;
}

int psb_dpst_bl(struct drm_device *dev, void *data)
{
	struct drm_psb_private *dev_priv = psb_priv(dev);
	uint32_t *arg = data;
	struct backlight_device bd;
	dev_priv->blc_adj2 = *arg;

#ifdef CONFIG_BACKLIGHT_CLASS_DEVICE
	bd.props.brightness = psb_get_brightness(&bd);
	psb_set_brightness(&bd);
#endif
	return 0;
}

int psb_gamma_set(struct drm_device *dev, void *data)
{
	uint16_t *lut_arg = data;
	struct drm_crtc *crtc;
	struct drm_connector *connector;
	struct psb_intel_crtc *psb_intel_crtc;
	struct drm_psb_private *dev_priv = dev->dev_private;
	int i = 0;

	connector = dev_priv->dpst_lvds_connector;

	crtc = connector->encoder->crtc;
	psb_intel_crtc = to_psb_intel_crtc(crtc);

	for (i = 0; i < 256; i++)
		psb_intel_crtc->lut_adj[i] = lut_arg[i];

	psb_intel_crtc_load_lut(crtc);

	return 0;
}

static void dpst_save_bl_adj_factor(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;

	if (!dev_priv)
		return;

	blc_adj2 = dev_priv->blc_adj2;
}

static void dpst_restore_bl_adj_factor(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;

	if (!dev_priv)
		return;

	if (blc_adj2 != dev_priv->blc_adj2 && blc_adj2 != 0)
		psb_dpst_bl(dev, &blc_adj2);
}

static void dpst_save_gamma_settings(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct drm_connector *connector;
	struct mdfld_dsi_config *dsi_config;
	struct drm_crtc *crtc;
	struct psb_intel_crtc *psb_intel_crtc;
	int i = 0;

	if (!dev_priv)
		return;

	connector = dev_priv->dpst_lvds_connector;
	dsi_config = dev_priv->dsi_configs[0];

	crtc = connector->encoder->crtc;
	psb_intel_crtc = to_psb_intel_crtc(crtc);

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				OSPM_UHB_FORCE_POWER_ON))
		return;
	mdfld_dsi_dsr_forbid(dsi_config);

	for (i = 0; i < 256; i++)
		lut_adj[i] = REG_READ((PALETTE_A + 4 * i));

	mdfld_dsi_dsr_allow(dsi_config);
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
}

static void dpst_restore_gamma_settings(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct mdfld_dsi_config *dsi_config;
	struct mdfld_dsi_hw_context *ctx;
	struct drm_connector *connector;
	struct drm_crtc *crtc;
	struct psb_intel_crtc *psb_intel_crtc;
	int i = 0;

	if (!dev_priv)
		return;

	connector = dev_priv->dpst_lvds_connector;
	dsi_config = dev_priv->dsi_configs[0];
	ctx = &dsi_config->dsi_hw_context;

	crtc = connector->encoder->crtc;
	psb_intel_crtc = to_psb_intel_crtc(crtc);

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				OSPM_UHB_FORCE_POWER_ON))
		return;
	mdfld_dsi_dsr_forbid(dsi_config);

	for (i = 0; i < 256; i++) {
		ctx->palette[i] = lut_adj[i];
		REG_WRITE((PALETTE_A + 4 * i), lut_adj[i]);
	}

	mdfld_dsi_dsr_allow(dsi_config);
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
}

void dpst_disable_post_process(struct drm_device *dev)
{
	dpst_restore_bl_adj_factor(dev);
	dpst_restore_gamma_settings(dev);
}

void dpst_execute_recv_command(struct dispmgr_command_hdr *cmd_hdr)
{
	switch (cmd_hdr->cmd) {
	case DISPMGR_DPST_GET_MODE:
		{
			struct dispmgr_command_hdr send_cmd_hdr;
			uint32_t xy = 0;

			psb_dpst_mode(g_dev, &xy);
			send_cmd_hdr.data_size = sizeof(xy);
			send_cmd_hdr.data = &xy;
			send_cmd_hdr.module = DISPMGR_MOD_DPST;
			send_cmd_hdr.cmd = DISPMGR_DPST_GET_MODE;
			dispmgr_nl_send_msg(&send_cmd_hdr);
		}
		break;
	case DISPMGR_DPST_INIT_COMM:
		{
			if (cmd_hdr->data_size) {
				unsigned long value =
				    *((unsigned long *)cmd_hdr->data);
				uint32_t enable = value;
				psb_init_comm(g_dev, &enable);
			}
#ifdef CONFIG_A500CG
			init_done = 1;
#endif
		}
		break;
	case DISPMGR_DPST_UPDATE_GUARD:
		{
			if (cmd_hdr->data_size) {
				unsigned long value =
				    *((unsigned long *)cmd_hdr->data);
				uint32_t gb_arg = value;
				psb_update_guard(g_dev, &gb_arg);
			}
		}
		break;
	case DISPMGR_DPST_BL_SET:
		{
			if (cmd_hdr->data_size) {
				unsigned long value =
				    *((unsigned long *)cmd_hdr->data);
				uint32_t data = value;
				psb_dpst_bl(g_dev, (void *)&data);
			}
		}
		break;
	case DISPMGR_DPST_HIST_ENABLE:
		{
			if (cmd_hdr->data_size) {
				unsigned long value =
				    *((unsigned long *)cmd_hdr->data);
				uint32_t enable = value;
				psb_hist_enable(g_dev, &enable);
			}
		}
		break;
	case DISPMGR_DPST_GAMMA_SET:
		{
			if (cmd_hdr->data_size) {
				uint16_t *arg = (uint16_t *) cmd_hdr->data;
				psb_gamma_set(g_dev, (void *)arg);
			}
		}
		break;
	case DISPMGR_DPST_DIET_ENABLE:
		{
			if (cmd_hdr->data_size) {
				uint32_t *arg = (uint32_t *) cmd_hdr->data;
				psb_diet_enable(g_dev, (void *)arg);
			}
		}
		break;
	case DISPMGR_DPST_DIET_DISABLE:
		{
			psb_diet_enable(g_dev, 0);
		}
		break;
	default:
		{
			pr_warn("kdispmgr: received unknown command = %d.\n",
					cmd_hdr->cmd);
		};
	};			/* switch */
}

/* Initialize the dpst data */
int dpst_init(struct drm_device *dev, int level)
{
	g_dev = dev;
	/* hack for now - the work queue does not have the device */
#ifdef CONFIG_A500CG
	if (dpst_level < 1 || dpst_level > 5)
#endif
		dpst_level = level;

	dpst_save_bl_adj_factor(dev);
	dpst_save_gamma_settings(dev);
	dispmgr_start(dev);

	return 0;
}
