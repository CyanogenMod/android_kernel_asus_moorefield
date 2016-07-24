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

#ifndef __IA_CAMERA_UTILS_CACHED_PARAM_CONFIGS_H__
#define __IA_CAMERA_UTILS_CACHED_PARAM_CONFIGS_H__

#include "ia_css_program_group_data.h"
#include "ia_css_buffer.h"
#include "ia_css_frame_public.h"
#include "ia_css_psys_manifest_types.h" /* ia_css_param_terminal_manifest_t*/

/* For identifying different sections (Stored inside ia_css_parameter_manifest_s->kernel_id)*/
typedef enum {
	IA_CAMERA_PSYS_PARAMS_KERNEL_ID_POC,
	IA_CAMERA_PSYS_PARAMS_KERNEL_ID_S3A,
	IA_CAMERA_PSYS_PARAMS_KERNEL_ID_MAX,

}ia_camera_params_cached_types_t;

typedef struct ia_css_3a_config_ext ia_css_3a_config_ext_t;

#define SIZE_OF_IA_CSS_3A_CONFIG_EXT_IN_BITS\
	((6 * IA_CSS_UINT32_T_BITS) + \
	(7 * IA_CSS_INT32_T_BITS) + \
	(7 * IA_CSS_INT32_T_BITS))

 /* modified version of ia_css_3a_config coming from
	css/isp/kernels/s3a/s3a_1.0/ia_css_s3a_types.h

	reserved bits are added at end to ensure
	safe copy of 64->32 bits
 */
struct ia_css_3a_config_ext {
	/* HOWTO: Alternately we can include ia_css_s3a_types.h &
	 * add ia_css_3a_config as is*/
	uint32_t ae_y_coef_r;	/**< Weight of R for Y.
						u0.16, [0,65535],
						default/ineffective 25559 */
	uint32_t ae_y_coef_g;	/**< Weight of G for Y.
						u0.16, [0,65535],
						default/ineffective 32768 */
	uint32_t ae_y_coef_b;	/**< Weight of B for Y.
						u0.16, [0,65535],
						default/ineffective 7209 */
	uint32_t awb_lg_high_raw;	/**< AWB level gate high for raw.
						u0.16, [0,65535],
						default 65472(=1023*64),
						ineffective 65535 */
	uint32_t awb_lg_low;	/**< AWB level gate low.
						u0.16, [0,65535],
						default 64(=1*64),
						ineffective 0 */
	uint32_t awb_lg_high;	/**< AWB level gate high.
						u0.16, [0,65535],
						default 65535,
						ineffective 65535 */
	int32_t af_fir1_coef[7];	/**< AF FIR coefficients of fir1.
						s0.15, [-32768,32767],
				default/ineffective
				-6689,-12207,-32768,32767,12207,6689,0 */
	int32_t af_fir2_coef[7];	/**< AF FIR coefficients of fir2.
						s0.15, [-32768,32767],
				default/ineffective
				2053,0,-18437,32767,-18437,2053,0 */
};
#endif /* __IA_CAMERA_UTILS_CACHED_PARAM_CONFIGS_H__ */

