/**************************************************************************
 * Copyright (c) 2014, Intel Corporation.
 * All Rights Reserved.
 * Copyright (c) 2008, Tungsten Graphics, Inc. Cedar Park, TX., USA.
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


#include <linux/errno.h>
#include <linux/types.h>

#include "psb_intel_reg.h"
#include "mdfld_dsi_pkg_sender.h"
#include "mdfld_dsi_output.h"


/*
 * display_cmn_set_cabc_mode() - Set CABC hw mode.
 * CABC is Content Adaptive Backlight Control.
 * Caller is responsible for power state, locking, etc.
 * @dsi_config
 * @cabc_mode - the mode to be set.
 * Function return value: < 0 if error, or cabc mode.
 */
int display_cmn_set_cabc_mode(struct mdfld_dsi_config *dsi_config, u8 cabc_mode)
{
	const int len = 1;
	struct mdfld_dsi_pkg_sender *sender;
	int err;

	sender = mdfld_dsi_get_pkg_sender(dsi_config);
	if (!sender) {
		DRM_ERROR("%s:%u: mdfld_dsi_get_pkg_sender failed\n",
		__func__, __LINE__);
		return -EIO;
	}

	err = mdfld_dsi_send_mcs_short_hs(sender,
			write_ctrl_cabc, cabc_mode, len,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s:%u: Write Control CABC\n",
		__func__, __LINE__);
		return -EIO;
	}

	return 0;
}


/*
 * display_cmn_get_cabc_mode() - Return cabc hw mode.
 * CABC is Content Adaptive Backlight Control.
 * Caller is responsible for power state, locking, etc.
 * @dsi_config
 * @cabc_mode - the mode to be set.
 * Function return value: < 0 if error, or cabc mode.
 *
 * Warning: In recent testing, the underlying call from this function to
 * __read_panel_data has been observed to return ETIMEDOUT.
 */
int display_cmn_get_cabc_mode(struct mdfld_dsi_config *dsi_config)
{
	const int len = 1;
	u8 acmode[len];
	struct mdfld_dsi_pkg_sender *sender;
	int ret;

	sender = mdfld_dsi_get_pkg_sender(dsi_config);
	if (!sender) {
		DRM_ERROR("%s:%u: mdfld_dsi_get_pkg_sender failed\n",
		__func__, __LINE__);
		return -EIO;
	}

	if (!sender) {
		DRM_ERROR("%s:%u: Invalid parameter\n", __func__, __LINE__);
		return -EINVAL;
	}

	ret = mdfld_dsi_read_mcs_hs(sender, read_ctrl_cabc, acmode, len);

	if (ret < 0) {
		DRM_ERROR("%s:%u: read CABC failed\n", __func__, __LINE__);
		return ret;
	}

	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL) {
		DRM_ERROR("%s:%u: read CABC abnormal status\n", __func__, __LINE__);
		ret = -EIO;
	}

	return acmode[0];
}
