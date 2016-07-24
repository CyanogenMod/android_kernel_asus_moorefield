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
#include "libcss2600.h"

EXPORT_SYMBOL_GPL(ia_css_isys_device_open);
EXPORT_SYMBOL_GPL(ia_css_isys_device_close);
EXPORT_SYMBOL_GPL(ia_css_isys_stream_open);
EXPORT_SYMBOL_GPL(ia_css_isys_stream_close);
EXPORT_SYMBOL_GPL(ia_css_isys_stream_start);
EXPORT_SYMBOL_GPL(ia_css_isys_stream_stop);
EXPORT_SYMBOL_GPL(ia_css_isys_stream_flush);
EXPORT_SYMBOL_GPL(ia_css_isys_stream_capture_indication);
EXPORT_SYMBOL_GPL(ia_css_isys_stream_handle_response);
EXPORT_SYMBOL_GPL(ia_css_input_buffer_alloc);
EXPORT_SYMBOL_GPL(ia_css_input_buffer_free);
EXPORT_SYMBOL_GPL(ia_css_input_buffer_cpu_map);
EXPORT_SYMBOL_GPL(ia_css_input_buffer_cpu_store);
EXPORT_SYMBOL_GPL(ia_css_input_buffer_css_map);
EXPORT_SYMBOL_GPL(ia_css_input_buffer_css_unmap);
EXPORT_SYMBOL_GPL(ia_css_output_buffer_alloc);
EXPORT_SYMBOL_GPL(ia_css_output_buffer_css_unmap);
EXPORT_SYMBOL_GPL(ia_css_input_buffer_cpu_unmap);
EXPORT_SYMBOL_GPL(ia_css_output_buffer_cpu_load);
EXPORT_SYMBOL_GPL(ia_css_output_buffer_css_map);
EXPORT_SYMBOL_GPL(ia_css_output_buffer_cpu_unmap);
EXPORT_SYMBOL_GPL(ia_css_output_buffer_cpu_map);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Intel css2600 library");

