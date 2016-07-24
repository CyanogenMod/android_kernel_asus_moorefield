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
#include "libcsspsys2600.h"



/* PSYS library symbols */
EXPORT_SYMBOL_GPL(ia_css_psys_open);
EXPORT_SYMBOL_GPL(ia_css_psys_close);
EXPORT_SYMBOL_GPL(ia_css_psys_event_queue_receive);
EXPORT_SYMBOL_GPL(ia_css_sizeof_psys);
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
EXPORT_SYMBOL_GPL(ia_css_terminal_get_type);
EXPORT_SYMBOL_GPL(ia_css_process_group_set_resource_bitmap);
EXPORT_SYMBOL_GPL(ia_css_process_group_get_resource_bitmap);

/*Implement SMIF/Vied subsystem access here */

/* Platform stubs */
/* TODO: remove these (tracked as ICG BZ 3432) */

int platform_load(void)
{
	pr_info("%s not implemented\n", __FUNCTION__);
	return -ENOENT;
}

int platform_start(void)
{
	pr_info("%s not implemented\n", __FUNCTION__);
	return -ENOENT;
}

int platform_stop(void)
{
	pr_info("%s not implemented\n", __FUNCTION__);
	return -ENOENT;
}

unsigned long long _hrt_master_port_subsystem_base_address;

/* end of platform stubs */

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Intel css2600 psys library");
