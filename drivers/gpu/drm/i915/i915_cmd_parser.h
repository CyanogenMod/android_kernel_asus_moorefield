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

/**
 * This file is a convenience to avoid throwing a bunch of large tables in other
 * files. It should only be included from intel_ringbuffer.c, where we connect
 * the tables to the appropriate rings.
 */

#define STD_MI_OPCODE_MASK  0xFF800000
#define STD_3D_OPCODE_MASK  0xFFFF0000
#define STD_2D_OPCODE_MASK  0xFFC00000
#define STD_MFX_OPCODE_MASK 0xFFFF0000

#define CMD(op, opm, f, lm, fl, ...)		\
	{					\
		.flags = (fl) | (f),		\
		.cmd = { (op), (opm) },		\
		.length = { (lm) },		\
		__VA_ARGS__			\
	}

/* Convenience macros to compress the tables */
#define SMI STD_MI_OPCODE_MASK
#define S3D STD_3D_OPCODE_MASK
#define S2D STD_2D_OPCODE_MASK
#define SMFX STD_MFX_OPCODE_MASK
#define F CMD_DESC_FIXED
#define S CMD_DESC_SKIP
#define R CMD_DESC_REJECT
#define W CMD_DESC_REGISTER
#define B CMD_DESC_BITMASK

/*            Command                          Mask   Fixed Len   Action
	      ---------------------------------------------------------- */
static const struct drm_i915_cmd_descriptor common_cmds[] = {
	CMD(MI_NOOP,                          SMI,    F,  1,      S),
	CMD(MI_USER_INTERRUPT,                SMI,    F,  1,      R),
	CMD(MI_WAIT_FOR_EVENT,                SMI,    F,  1,      S),
	CMD(MI_ARB_CHECK,                     SMI,    F,  1,      S),
	CMD(MI_REPORT_HEAD,                   SMI,    F,  1,      S),
	CMD(MI_SUSPEND_FLUSH,                 SMI,    F,  1,      S),
	CMD(MI_SEMAPHORE_MBOX,                SMI,   !F,  0xFF,   R),
	CMD(MI_STORE_DWORD_INDEX,             SMI,   !F,  0xFF,   R),
	CMD(MI_LOAD_REGISTER_IMM(1),          SMI,   !F,  0xFF,   W,
	      .reg = { .offset = 1, .mask = 0x007FFFFC }),
	CMD(MI_UPDATE_GTT,                    SMI,   !F,  0xFF,   R),
	CMD(MI_STORE_REGISTER_MEM,            SMI,   !F,  0xFF,   W,
	      .reg = { .offset = 1, .mask = 0x007FFFFC },
	      .bits = {{
			.offset = 0,
			.mask = MI_GLOBAL_GTT,
			.expected = 0
	      } },
	      .bits_count = 1),
	CMD(MI_LOAD_REGISTER_MEM,             SMI,   !F,  0xFF,   W,
	      .reg = { .offset = 1, .mask = 0x007FFFFC },
	      .bits = {{
			.offset = 0,
			.mask = MI_GLOBAL_GTT,
			.expected = 0
	      } },
	      .bits_count = 1),
};

static const struct drm_i915_cmd_descriptor render_cmds[] = {
	CMD(MI_FLUSH,                         SMI,    F,  1,      S),
	CMD(MI_ARB_ON_OFF,                    SMI,    F,  1,      R),
	CMD(MI_DISPLAY_FLIP,                  SMI,   !F,  0xFF,   R),
	CMD(MI_PREDICATE,                     SMI,    F,  1,      S),
	CMD(MI_TOPOLOGY_FILTER,               SMI,    F,  1,      S),
	CMD(MI_LOAD_SCAN_LINES_INCL,          SMI,   !F,  0x3F,   R),
	CMD(MI_LOAD_SCAN_LINES_EXCL,          SMI,   !F,  0x3F,   R),
	CMD(MI_STORE_DWORD_IMM,               SMI,   !F,  0x3F,   S,
	      .bits = {{
			.offset = 0,
			.mask = MI_GLOBAL_GTT,
			.expected = 0
	      } },
	      .bits_count = 1),
	CMD(MI_CLFLUSH,                       SMI,   !F,  0x3FF,  S,
	      .bits = {{
			.offset = 0,
			.mask = MI_GLOBAL_GTT,
			.expected = 0
	      } },
	      .bits_count = 1),
	CMD(MI_CONDITIONAL_BATCH_BUFFER_END,  SMI,   !F,  0xFF,   S,
	      .bits = {{
			.offset = 0,
			.mask = MI_GLOBAL_GTT,
			.expected = 0
	      } },
	      .bits_count = 1),
	CMD(GFX_OP_3DSTATE_VF_STATISTICS,     S3D,    F,  1,      S),
	CMD(PIPELINE_SELECT,                  S3D,    F,  1,      S),
	CMD(MEDIA_VFE_STATE,			S3D,   !F,  0xFFFF, B,
	      .bits = {{
			.offset = 2,
			.mask = MEDIA_VFE_STATE_MMIO_ACCESS_MASK,
			.expected = 0
	      } },
	      .bits_count = 1),
	CMD(GPGPU_OBJECT,                     S3D,   !F,  0xFF,   S),
	CMD(GPGPU_WALKER,                     S3D,   !F,  0xFF,   S),
	CMD(GFX_OP_3DSTATE_SO_DECL_LIST,      S3D,   !F,  0x1FF,  S),
	CMD(GFX_OP_PIPE_CONTROL(5),           S3D,   !F,  0xFF,   B,
	      .bits = {{
			.offset = 1,
			.mask = (PIPE_CONTROL_MMIO_WRITE | PIPE_CONTROL_NOTIFY),
			.expected = 0
#if 1
	      } },
	      .bits_count = 1),
#else /*Enable path if aliased PPGTT is on*/
		  },
	      {
			.offset = 1,
			.mask = PIPE_CONTROL_GLOBAL_GTT_IVB,
			.expected = 0,
			.condition_offset = 1,
			.condition_mask = PIPE_CONTROL_POST_SYNC_OP_MASK
	      } },
	      .bits_count = 2),
#endif
};

static struct drm_i915_cmd_descriptor ivb_render_cmds[] = {
	CMD(MI_BATCH_BUFFER_START,            SMI,   !F,  0xFF,   R),
};

static struct drm_i915_cmd_descriptor hsw_render_cmds[] = {
	CMD(MI_SET_PREDICATE,                 SMI,    F,  1,      S),
	CMD(MI_RS_CONTROL,                    SMI,    F,  1,      S),
	CMD(MI_URB_ATOMIC_ALLOC,              SMI,    F,  1,      S),
	CMD(MI_RS_CONTEXT,                    SMI,    F,  1,      S),
	CMD(MI_MATH,                          SMI,   !F,  0x3F,   S),
	CMD(MI_LOAD_REGISTER_REG,             SMI,   !F,  0xFF,   W,
	      .reg = { .offset = 1, .mask = 0x007FFFFC }),
	CMD(MI_LOAD_URB_MEM,                  SMI,   !F,  0xFF,   S),
	CMD(MI_STORE_URB_MEM,                 SMI,   !F,  0xFF,   S),
	CMD(MI_BATCH_BUFFER_START,            SMI,   !F,  0xFF,   B,
	      .bits = {{
			.offset = 0,
			.mask = MI_BATCH_NON_SECURE_HSW,
			.expected = MI_BATCH_NON_SECURE_HSW
	      } },
	      .bits_count = 1),
	CMD(GFX_OP_3DSTATE_DX9_CONSTANTF_VS,  S3D,   !F,  0x7FF,  S),
	CMD(GFX_OP_3DSTATE_DX9_CONSTANTF_PS,  S3D,   !F,  0x7FF,  S),
};

static const struct drm_i915_cmd_descriptor video_cmds[] = {
	CMD(MI_ARB_ON_OFF,                    SMI,    F,  1,      R),
	CMD(MI_STORE_DWORD_IMM,               SMI,   !F,  0xFF,   S,
	      .bits = {{
			.offset = 0,
			.mask = MI_GLOBAL_GTT,
			.expected = 0
	      } },
	      .bits_count = 1),
	CMD(MI_FLUSH_DW,                      SMI,   !F,  0x3F,   B,
	      .bits = {{
			.offset = 0,
			.mask = MI_FLUSH_DW_NOTIFY,
			.expected = 0
	      },
#if 0 /*Enable path if aliased PPGTT is on*/
	      {
			.offset = 1,
			.mask = MI_FLUSH_DW_USE_GTT,
			.expected = 0,
			.condition_offset = 0,
			.condition_mask = MI_FLUSH_DW_OP_MASK
	      },
#endif
	      {
			.offset = 0,
			.mask = MI_FLUSH_DW_STORE_INDEX,
			.expected = 0,
			.condition_offset = 0,
			.condition_mask = MI_FLUSH_DW_OP_MASK
	      } },
#if 0 /*Enable path if aliased PPGTT is on*/
	      .bits_count = 3),
#else
	      .bits_count = 2),
#endif
	CMD(MI_BATCH_BUFFER_START,            SMI,   !F,  0xFF,   S,
	      .bits = {{
			.offset = 0,
			.mask = MI_BATCH_PPGTT_HSW,
			.expected = MI_BATCH_PPGTT_HSW
	      } },
	      .bits_count = 1),
	CMD(MI_CONDITIONAL_BATCH_BUFFER_END,  SMI,   !F,  0xFF,   S,
	      .bits = {{
			.offset = 0,
			.mask = MI_GLOBAL_GTT,
			.expected = 0
	      } },
	      .bits_count = 1),
	CMD(MFX_WAIT,                         SMFX,   F,  1,      S),
};

static const struct drm_i915_cmd_descriptor vecs_cmds[] = {
	CMD(MI_STORE_DWORD_IMM,               SMI,   !F,  0xFF,   B,
	      .bits = {{
			.offset = 0,
			.mask = MI_GLOBAL_GTT,
			.expected = 0
	      } },
	      .bits_count = 1),
	CMD(MI_FLUSH_DW,                      SMI,   !F,  0x3F,   B,
	      .bits = {{
			.offset = 0,
			.mask = MI_FLUSH_DW_NOTIFY,
			.expected = 0
	      },
	      {
			.offset = 1,
			.mask = MI_FLUSH_DW_USE_GTT,
			.expected = 0,
			.condition_offset = 0,
			.condition_mask = MI_FLUSH_DW_OP_MASK
	      },
	      {
			.offset = 0,
			.mask = MI_FLUSH_DW_STORE_INDEX,
			.expected = 0,
			.condition_offset = 0,
			.condition_mask = MI_FLUSH_DW_OP_MASK
	      } },
	      .bits_count = 3),
	CMD(MI_BATCH_BUFFER_START,            SMI,   !F,  0xFF,   R),
	CMD(MI_CONDITIONAL_BATCH_BUFFER_END,  SMI,   !F,  0xFF,   B,
	      .bits = {{
			.offset = 0,
			.mask = MI_GLOBAL_GTT,
			.expected = 0
	      } },
	      .bits_count = 1),
};

static const struct drm_i915_cmd_descriptor blt_cmds[] = {
	CMD(MI_LOAD_SCAN_LINES_INCL,          SMI,   !F,  0x3F,   R),
	CMD(MI_LOAD_SCAN_LINES_EXCL,          SMI,   !F,  0x3F,   R),
	CMD(MI_DISPLAY_FLIP,                  SMI,   !F,  0xFF,   R),
	CMD(MI_STORE_DWORD_IMM,               SMI,   !F,  0x1FF,  S,
	      .bits = {{
			.offset = 0,
			.mask = MI_GLOBAL_GTT,
			.expected = 0
	      } },
	      .bits_count = 1),
	CMD(MI_FLUSH_DW,                      SMI,   !F,  0x3F,   B,
	      .bits = {{
			.offset = 0,
			.mask = MI_FLUSH_DW_NOTIFY,
			.expected = 0
	      },
#if 0 /*Enable path if aliased PPGTT is on*/
	      {
			.offset = 1,
			.mask = MI_FLUSH_DW_USE_GTT,
			.expected = 0,
			.condition_offset = 0,
			.condition_mask = MI_FLUSH_DW_OP_MASK
	      },
#endif
	      {
			.offset = 0,
			.mask = MI_FLUSH_DW_STORE_INDEX,
			.expected = 0,
			.condition_offset = 0,
			.condition_mask = MI_FLUSH_DW_OP_MASK
	      } },
#if 0 /*Enable path if aliased PPGTT is on*/
	      .bits_count = 3),
#else
	      .bits_count = 2),
#endif
	CMD(MI_BATCH_BUFFER_START,            SMI,   !F,  0xFF,   R),
	CMD(COLOR_BLT,                        S2D,   !F,  0x3F,   S),
	CMD(SRC_COPY_BLT,                     S2D,   !F,  0x3F,   S),
};

#undef CMD
#undef SMI
#undef S3D
#undef S2D
#undef SMFX
#undef F
#undef S
#undef R
#undef W
#undef B

static const struct drm_i915_cmd_table gen7_render_cmds[] = {
	{ common_cmds, ARRAY_SIZE(common_cmds) },
	{ render_cmds, ARRAY_SIZE(render_cmds) },
	{ ivb_render_cmds, ARRAY_SIZE(ivb_render_cmds) },
};

static struct drm_i915_cmd_table hsw_render_ring_cmds[] = {
	{ common_cmds, ARRAY_SIZE(common_cmds) },
	{ render_cmds, ARRAY_SIZE(render_cmds) },
	{ hsw_render_cmds, ARRAY_SIZE(hsw_render_cmds) },
};

static const struct drm_i915_cmd_table gen7_video_cmds[] = {
	{ common_cmds, ARRAY_SIZE(common_cmds) },
	{ video_cmds, ARRAY_SIZE(video_cmds) },
};

static struct drm_i915_cmd_table hsw_vebox_cmds[] = {
	{ common_cmds, ARRAY_SIZE(common_cmds) },
	{ vecs_cmds, ARRAY_SIZE(vecs_cmds) },
};

static const struct drm_i915_cmd_table gen7_blt_cmds[] = {
	{ common_cmds, ARRAY_SIZE(common_cmds) },
	{ blt_cmds, ARRAY_SIZE(blt_cmds) },
};
