/*
 * Copyright (c) 2014 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include "libcss2401.h"

EXPORT_SYMBOL_GPL(ia_css_init);
EXPORT_SYMBOL_GPL(ia_css_load_firmware);
EXPORT_SYMBOL_GPL(ia_css_fwctrl_dequeue_event);
EXPORT_SYMBOL_GPL(ia_css_psys_specify);
EXPORT_SYMBOL_GPL(ia_css_isys_device_open);
EXPORT_SYMBOL_GPL(ia_css_isys_device_close);
EXPORT_SYMBOL_GPL(ia_css_isys_stream_open);
EXPORT_SYMBOL_GPL(ia_css_isys_stream_close);
EXPORT_SYMBOL_GPL(ia_css_isys_stream_start);
EXPORT_SYMBOL_GPL(ia_css_isys_stream_stop);
EXPORT_SYMBOL_GPL(ia_css_isys_stream_flush);
EXPORT_SYMBOL_GPL(ia_css_isys_stream_capture_indication);
EXPORT_SYMBOL_GPL(ia_css_isys_stream_handle_response);
EXPORT_SYMBOL_GPL(ia_css_isysapi_rx_set_csi_port_cfg);
EXPORT_SYMBOL_GPL(ia_css_isysapi_rx_set_tpg_cfg);
EXPORT_SYMBOL_GPL(ia_css_isysapi_rx_set_prbs_cfg);

/*
 * FIXME: ia_css_fwctrl_device_open/close are deprecated and replaced by:
 * ia_css_isys_device_open()
 * ia_css_isys_device_close()
 * ia_css_psys_open()
 * ia_css_psys_close()
 */
EXPORT_SYMBOL_GPL(ia_css_fwctrl_device_open);
EXPORT_SYMBOL_GPL(ia_css_fwctrl_device_close);

/* PSYS library symbols */
EXPORT_SYMBOL_GPL(ia_css_psys_open);
EXPORT_SYMBOL_GPL(ia_css_psys_close);
EXPORT_SYMBOL_GPL(ia_css_psys_event_queue_receive);
EXPORT_SYMBOL_GPL(ia_css_process_group_create);
EXPORT_SYMBOL_GPL(ia_css_process_group_submit);
EXPORT_SYMBOL_GPL(ia_css_process_group_start);
EXPORT_SYMBOL_GPL(ia_css_process_group_suspend);
EXPORT_SYMBOL_GPL(ia_css_process_group_resume);
EXPORT_SYMBOL_GPL(ia_css_process_group_reset);
EXPORT_SYMBOL_GPL(ia_css_process_group_abort);
EXPORT_SYMBOL_GPL(ia_css_process_group_destroy);
EXPORT_SYMBOL_GPL(ia_css_process_group_attach_buffer);
EXPORT_SYMBOL_GPL(ia_css_process_group_detach_buffer);
EXPORT_SYMBOL_GPL(ia_css_process_group_set_token);
EXPORT_SYMBOL_GPL(ia_css_process_group_get_terminal);
EXPORT_SYMBOL_GPL(ia_css_process_group_get_terminal_count);
EXPORT_SYMBOL_GPL(ia_css_process_group_get_resource_bitmap);
EXPORT_SYMBOL_GPL(ia_css_process_group_set_resource_bitmap);
EXPORT_SYMBOL_GPL(ia_css_terminal_get_type);

struct ia_css_fw_info *libcss2401_get_sp_fw(void)
{
	return &sh_css_sp_fw;
}
EXPORT_SYMBOL_GPL(libcss2401_get_sp_fw);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Intel css2401 library");
