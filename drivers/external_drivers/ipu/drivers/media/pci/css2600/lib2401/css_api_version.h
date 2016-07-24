/*
 * Support for Intel Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2010 - 2014 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#ifndef __CSS_API_VERSION_H
#define __CSS_API_VERSION_H

/** @file
 * CSS API version file. This file contains the version number of the CSS-API
 */

/**
The last two digits of the CSS_API_VERSION_STRING give the major and minor
versions of the CSS-API. The minor version number will be increased by one when
a backwards-compatible change is made. The major version number will be
increased (and the minor version number reset) when a non-backwards-compatible
change is made.
*/

#define CSS_API_VERSION_STRING	"2.1.3.4"

/*
Change log

V2.0.1.0, initial version:
 - added API versioning

V2.0.1.1, activate CSS-API versioning:
 - added description of major and minor version numbers

v2.0.1.2, modified struct ia_css_frame_info:
 - added new member ia_css_crop_info

V2.0.1.3, added IA_CSS_ERR_NOT_SUPPORTED

V2.1.0.0
- moved version number to 2.1.0.0
- created new files for refactoring the code

v2.1.1.0, modified struct ia_css_pipe_config and struct ia_css_pipe_info and struct ia_css_pipe:
 - use array to handle multiple output ports

V2.1.1.1
- added api to lock/unlock of RAW Buffers to Support HALv3 Feature

v2.1.1.2, modified struct ia_css_stream_config:
 - to support multiple isys streams in one virtual channel, keep the old one for backward compatibility

v2.1.2.0, modify ia_css_stream_config:
 - add isys_config and input_config to support multiple isys stream within one virtual channel

v2.1.2.1, add IA_CSS_STREAM_FORMAT_NUM
 - add IA_CSS_STREAM_FORMAT_NUM definition to reflect the number of ia_css_stream_format enums

v2.1.2.2, modified enum ia_css_stream_format
 - Add 16bit YUV formats to ia_css_stream_format enum:
   IA_CSS_STREAM_FORMAT_YUV420_16 (directly after IA_CSS_STREAM_FORMAT_YUV420_10)
   IA_CSS_STREAM_FORMAT_YUV422_16 (directly after IA_CSS_STREAM_FORMAT_YUV422_10)

v2.1.2.3
- added api to enable/disable digital zoom for capture pipe.

v2.1.2.4, change CSS API to generate the shading table which should be directly sent to ISP:
 - keep the old CSS API (which uses the conversion of the shading table in CSS) for backward compatibility

v2.1.2.5
- Added SP frame time measurement (in ticks) and result is sent on a new member
  in ia_css_buffer.h.

v2.1.2.6 add function ia_css_check_firmware_version()
 - the function ia_css_check_firmware_version() returns true when the firmware version matches
   and returns false otherwise.

v2.1.2.7
- rename dynamic_data_index to dynamic_queue_id in struct ia_css_frame.
- update IA_CSS_PIPE_MODE_NUM

v2.1.2.8
 - added flag for video full range

v2.1.2.9
 - add public parameters for xnr3 kernel

v2.1.2.10
 - add new interface to enable output mirroring

v2.1.2.11 MIPI buffers optimization
 - modified struct ia_css_mipi_buffer_config, added number of MIPI buffers needed for the stream
 - backwards compatible, need another patch to remove legacy function and code

v2.1.2.12
- create consolidated  firmware package for 2400, 2401, csi2p, bxtpoc

v2.1.3.0
 - rename ia_css_output_config.enable_mirror
 - add new interface to enable vertical output flipping

v2.1.3.1
- deprecated ia_css_rx_get_irq_info and ia_css_rx_clear_irq_info because both
  are hardcoded to work on CSI port 1.
- added new functions ia_css_rx_port_get_irq_info and
  ia_css_rx_port_clear_irq_info, both have a port ID as extra argument.

v2.1.3.2
- reverted v2.1.3.0 change

v2.1.3.3
- Added isys event queue.
- Renamed ia_css_dequeue_event to ia_css_dequeue_psys_event
- Made ia_css_dequeue_event deprecated

v2.1.3.4
- added new interface to support ACC extension QoS feature.
- added IA_CSS_EVENT_TYPE_ACC_STAGE_COMPLETE.

*/

#endif /*__CSS_API_VERSION_H*/
