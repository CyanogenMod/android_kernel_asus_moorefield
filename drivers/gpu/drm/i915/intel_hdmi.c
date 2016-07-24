/*
 * Copyright 2006 Dave Airlie <airlied@linux.ie>
 * Copyright © 2006-2009 Intel Corporation
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
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Authors:
 *	Eric Anholt <eric@anholt.net>
 *	Jesse Barnes <jesse.barnes@intel.com>
 */

#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/hdmi.h>
#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_edid.h>
#include "intel_drv.h"
#include <drm/i915_drm.h>
#include "i915_drv.h"

#define ENABLE_PFIT_HDMI 0

static struct drm_device *intel_hdmi_to_dev(struct intel_hdmi *intel_hdmi)
{
	return hdmi_to_dig_port(intel_hdmi)->base.base.dev;
}

static void
assert_hdmi_port_disabled(struct intel_hdmi *intel_hdmi)
{
	struct drm_device *dev = intel_hdmi_to_dev(intel_hdmi);
	struct drm_i915_private *dev_priv = dev->dev_private;
	uint32_t enabled_bits;

	enabled_bits = HAS_DDI(dev) ? DDI_BUF_CTL_ENABLE : SDVO_ENABLE;

	WARN(I915_READ(intel_hdmi->hdmi_reg) & enabled_bits,
	     "HDMI port enabled, expecting disabled\n");
}

struct intel_hdmi *enc_to_intel_hdmi(struct drm_encoder *encoder)
{
	struct intel_digital_port *intel_dig_port =
		container_of(encoder, struct intel_digital_port, base.base);
	return &intel_dig_port->hdmi;
}

static struct intel_hdmi *intel_attached_hdmi(struct drm_connector *connector)
{
	return enc_to_intel_hdmi(&intel_attached_encoder(connector)->base);
}

static u32 g4x_infoframe_index(enum hdmi_infoframe_type type)
{
	switch (type) {
	case HDMI_INFOFRAME_TYPE_AVI:
		return VIDEO_DIP_SELECT_AVI;
	case HDMI_INFOFRAME_TYPE_SPD:
		return VIDEO_DIP_SELECT_SPD;
	default:
		DRM_DEBUG_DRIVER("unknown info frame type %d\n", type);
		return 0;
	}
}

static u32 g4x_infoframe_enable(enum hdmi_infoframe_type type)
{
	switch (type) {
	case HDMI_INFOFRAME_TYPE_AVI:
		return VIDEO_DIP_ENABLE_AVI;
	case HDMI_INFOFRAME_TYPE_SPD:
		return VIDEO_DIP_ENABLE_SPD;
	default:
		DRM_DEBUG_DRIVER("unknown info frame type %d\n", type);
		return 0;
	}
}

static u32 hsw_infoframe_enable(enum hdmi_infoframe_type type)
{
	switch (type) {
	case HDMI_INFOFRAME_TYPE_AVI:
		return VIDEO_DIP_ENABLE_AVI_HSW;
	case HDMI_INFOFRAME_TYPE_SPD:
		return VIDEO_DIP_ENABLE_SPD_HSW;
	default:
		DRM_DEBUG_DRIVER("unknown info frame type %d\n", type);
		return 0;
	}
}

static u32 hsw_infoframe_data_reg(enum hdmi_infoframe_type type,
				  enum transcoder cpu_transcoder)
{
	switch (type) {
	case HDMI_INFOFRAME_TYPE_AVI:
		return HSW_TVIDEO_DIP_AVI_DATA(cpu_transcoder);
	case HDMI_INFOFRAME_TYPE_SPD:
		return HSW_TVIDEO_DIP_SPD_DATA(cpu_transcoder);
	default:
		DRM_DEBUG_DRIVER("unknown info frame type %d\n", type);
		return 0;
	}
}

static void g4x_write_infoframe(struct drm_encoder *encoder,
				enum hdmi_infoframe_type type,
				const uint8_t *frame, ssize_t len)
{
	uint32_t *data = (uint32_t *)frame;
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 val = I915_READ(VIDEO_DIP_CTL);
	int i;

	WARN(!(val & VIDEO_DIP_ENABLE), "Writing DIP with CTL reg disabled\n");

	val &= ~(VIDEO_DIP_SELECT_MASK | 0xf); /* clear DIP data offset */
	val |= g4x_infoframe_index(type);

	val &= ~g4x_infoframe_enable(type);

	I915_WRITE(VIDEO_DIP_CTL, val);

	mmiowb();
	for (i = 0; i < len; i += 4) {
		I915_WRITE(VIDEO_DIP_DATA, *data);
		data++;
	}
	/* Write every possible data byte to force correct ECC calculation. */
	for (; i < VIDEO_DIP_DATA_SIZE; i += 4)
		I915_WRITE(VIDEO_DIP_DATA, 0);
	mmiowb();

	val |= g4x_infoframe_enable(type);
	val &= ~VIDEO_DIP_FREQ_MASK;
	val |= VIDEO_DIP_FREQ_VSYNC;

	I915_WRITE(VIDEO_DIP_CTL, val);
	POSTING_READ(VIDEO_DIP_CTL);
}

static void ibx_write_infoframe(struct drm_encoder *encoder,
				enum hdmi_infoframe_type type,
				const uint8_t *frame, ssize_t len)
{
	uint32_t *data = (uint32_t *)frame;
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	int i, reg = TVIDEO_DIP_CTL(intel_crtc->pipe);
	u32 val = I915_READ(reg);

	WARN(!(val & VIDEO_DIP_ENABLE), "Writing DIP with CTL reg disabled\n");

	val &= ~(VIDEO_DIP_SELECT_MASK | 0xf); /* clear DIP data offset */
	val |= g4x_infoframe_index(type);

	val &= ~g4x_infoframe_enable(type);

	I915_WRITE(reg, val);

	mmiowb();
	for (i = 0; i < len; i += 4) {
		I915_WRITE(TVIDEO_DIP_DATA(intel_crtc->pipe), *data);
		data++;
	}
	/* Write every possible data byte to force correct ECC calculation. */
	for (; i < VIDEO_DIP_DATA_SIZE; i += 4)
		I915_WRITE(TVIDEO_DIP_DATA(intel_crtc->pipe), 0);
	mmiowb();

	val |= g4x_infoframe_enable(type);
	val &= ~VIDEO_DIP_FREQ_MASK;
	val |= VIDEO_DIP_FREQ_VSYNC;

	I915_WRITE(reg, val);
	POSTING_READ(reg);
}

static void cpt_write_infoframe(struct drm_encoder *encoder,
				enum hdmi_infoframe_type type,
				const uint8_t *frame, ssize_t len)
{
	uint32_t *data = (uint32_t *)frame;
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	int i, reg = TVIDEO_DIP_CTL(intel_crtc->pipe);
	u32 val = I915_READ(reg);

	WARN(!(val & VIDEO_DIP_ENABLE), "Writing DIP with CTL reg disabled\n");

	val &= ~(VIDEO_DIP_SELECT_MASK | 0xf); /* clear DIP data offset */
	val |= g4x_infoframe_index(type);

	/* The DIP control register spec says that we need to update the AVI
	 * infoframe without clearing its enable bit */
	if (type != HDMI_INFOFRAME_TYPE_AVI)
		val &= ~g4x_infoframe_enable(type);

	I915_WRITE(reg, val);

	mmiowb();
	for (i = 0; i < len; i += 4) {
		I915_WRITE(TVIDEO_DIP_DATA(intel_crtc->pipe), *data);
		data++;
	}
	/* Write every possible data byte to force correct ECC calculation. */
	for (; i < VIDEO_DIP_DATA_SIZE; i += 4)
		I915_WRITE(TVIDEO_DIP_DATA(intel_crtc->pipe), 0);
	mmiowb();

	val |= g4x_infoframe_enable(type);
	val &= ~VIDEO_DIP_FREQ_MASK;
	val |= VIDEO_DIP_FREQ_VSYNC;

	I915_WRITE(reg, val);
	POSTING_READ(reg);
}

static void vlv_write_infoframe(struct drm_encoder *encoder,
				enum hdmi_infoframe_type type,
				const uint8_t *frame, ssize_t len)
{
	uint32_t *data = (uint32_t *)frame;
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	int i, reg = VLV_TVIDEO_DIP_CTL(intel_crtc->pipe);
	u32 val = I915_READ(reg);

	WARN(!(val & VIDEO_DIP_ENABLE), "Writing DIP with CTL reg disabled\n");

	val &= ~(VIDEO_DIP_SELECT_MASK | 0xf); /* clear DIP data offset */
	val |= g4x_infoframe_index(type);

	val &= ~g4x_infoframe_enable(type);

	I915_WRITE(reg, val);

	mmiowb();
	for (i = 0; i < len; i += 4) {
		I915_WRITE(VLV_TVIDEO_DIP_DATA(intel_crtc->pipe), *data);
		data++;
	}
	/* Write every possible data byte to force correct ECC calculation. */
	for (; i < VIDEO_DIP_DATA_SIZE; i += 4)
		I915_WRITE(VLV_TVIDEO_DIP_DATA(intel_crtc->pipe), 0);
	mmiowb();

	val |= g4x_infoframe_enable(type);
	val &= ~VIDEO_DIP_FREQ_MASK;
	val |= VIDEO_DIP_FREQ_VSYNC;

	I915_WRITE(reg, val);
	POSTING_READ(reg);
}

static void hsw_write_infoframe(struct drm_encoder *encoder,
				enum hdmi_infoframe_type type,
				const uint8_t *frame, ssize_t len)
{
	uint32_t *data = (uint32_t *)frame;
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	u32 ctl_reg = HSW_TVIDEO_DIP_CTL(intel_crtc->config.cpu_transcoder);
	u32 data_reg;
	int i;
	u32 val = I915_READ(ctl_reg);

	data_reg = hsw_infoframe_data_reg(type,
					  intel_crtc->config.cpu_transcoder);
	if (data_reg == 0)
		return;

	val &= ~hsw_infoframe_enable(type);
	I915_WRITE(ctl_reg, val);

	mmiowb();
	for (i = 0; i < len; i += 4) {
		I915_WRITE(data_reg + i, *data);
		data++;
	}
	/* Write every possible data byte to force correct ECC calculation. */
	for (; i < VIDEO_DIP_DATA_SIZE; i += 4)
		I915_WRITE(data_reg + i, 0);
	mmiowb();

	val |= hsw_infoframe_enable(type);
	I915_WRITE(ctl_reg, val);
	POSTING_READ(ctl_reg);
}

/*
 * The data we write to the DIP data buffer registers is 1 byte bigger than the
 * HDMI infoframe size because of an ECC/reserved byte at position 3 (starting
 * at 0). It's also a byte used by DisplayPort so the same DIP registers can be
 * used for both technologies.
 *
 * DW0: Reserved/ECC/DP | HB2 | HB1 | HB0
 * DW1:       DB3       | DB2 | DB1 | DB0
 * DW2:       DB7       | DB6 | DB5 | DB4
 * DW3: ...
 *
 * (HB is Header Byte, DB is Data Byte)
 *
 * The hdmi pack() functions don't know about that hardware specific hole so we
 * trick them by giving an offset into the buffer and moving back the header
 * bytes by one.
 */
static void intel_write_infoframe(struct drm_encoder *encoder,
				  union hdmi_infoframe *frame)
{
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(encoder);
	uint8_t buffer[VIDEO_DIP_DATA_SIZE];
	ssize_t len;

	/* see comment above for the reason for this offset */
	len = hdmi_infoframe_pack(frame, buffer + 1, sizeof(buffer) - 1);
	if (len < 0)
		return;

	/* Insert the 'hole' (see big comment above) at position 3 */
	buffer[0] = buffer[1];
	buffer[1] = buffer[2];
	buffer[2] = buffer[3];
	buffer[3] = 0;
	len++;

	intel_hdmi->write_infoframe(encoder, frame->any.type, buffer, len);
}

void intel_dip_infoframe_csum(struct dip_infoframe *frame)

{

	uint8_t *data = (uint8_t *)frame;
	uint8_t sum = 0;
	unsigned i;

	frame->checksum = 0;
	frame->ecc = 0;

	for (i = 0; i < frame->len + DIP_HEADER_SIZE; i++)
		sum += data[i];

	frame->checksum = 0x100 - sum;

}

static void intel_set_infoframe(struct drm_encoder *encoder,
					struct dip_infoframe *frame) {

	unsigned len = (DIP_HEADER_SIZE + frame->len);
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(encoder);
	intel_dip_infoframe_csum(frame);
	intel_hdmi->write_infoframe(encoder, frame->type, (const uint8_t *) frame, len);

}

static void intel_hdmi_set_avi_infoframe(struct drm_encoder *encoder,
					 struct drm_display_mode *adjusted_mode)
{
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(encoder);
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);

	enum hdmi_picture_aspect PAR;

	struct dip_infoframe avi_if = {
		.type = DIP_TYPE_AVI,
		.ver = DIP_VERSION_AVI,
		.len = DIP_LEN_AVI,
		.body.avi.Y_A_B_S = 0,
		.body.avi.C_M_R = 8,
		.body.avi.ITC_EC_Q_SC = 0,
		.body.avi.VIC = 0,
		.body.avi.YQ_CN_PR = 0,
		.body.avi.top_bar_end = 0,
		.body.avi.bottom_bar_start = 0,
		.body.avi.left_bar_end = 0,
		.body.avi.right_bar_start = 0,
	};

	/* Bar information */
	avi_if.body.avi.Y_A_B_S |= DIP_AVI_BAR_BOTH;

	avi_if.body.avi.VIC = drm_match_cea_mode(adjusted_mode);

	if (adjusted_mode->flags & DRM_MODE_FLAG_DBLCLK)
		avi_if.body.avi.YQ_CN_PR |= DIP_AVI_PR_2;

	if (intel_hdmi->rgb_quant_range_selectable) {
		if (intel_crtc->config.limited_color_range)
			avi_if.body.avi.ITC_EC_Q_SC |= DIP_AVI_RGB_QUANT_RANGE_LIMITED;
		else
			avi_if.body.avi.ITC_EC_Q_SC |= DIP_AVI_RGB_QUANT_RANGE_FULL;
	} else {
		/* Set full range quantization for non-CEA modes
		and 640x480 */
		if ((avi_if.body.avi.VIC == 0) || (avi_if.body.avi.VIC == 1))
			avi_if.body.avi.ITC_EC_Q_SC |=
				DIP_AVI_RGB_QUANT_RANGE_FULL;
		else
			avi_if.body.avi.ITC_EC_Q_SC |=
				DIP_AVI_RGB_QUANT_RANGE_LIMITED;
	}

	/*If picture aspect ratio (PAR) is set to custom value, then use that,
	else if VIC > 1, then get PAR from CEA mode list, else, calculate
	PAR based on resolution */
	if (adjusted_mode->picture_aspect_ratio == HDMI_PICTURE_ASPECT_4_3 ||
	adjusted_mode->picture_aspect_ratio == HDMI_PICTURE_ASPECT_16_9) {
		avi_if.body.avi.C_M_R |=
			adjusted_mode->picture_aspect_ratio << 4;
		/*PAR is bit 5:4 of data byte 2 of AVI infoframe */
	} else if (avi_if.body.avi.VIC) {
		PAR = drm_get_cea_aspect_ratio(avi_if.body.avi.VIC);
		avi_if.body.avi.C_M_R |= PAR << 4;
	} else {
		if (!(adjusted_mode->vdisplay % 3) &&
			((adjusted_mode->vdisplay * 4 / 3) ==
			adjusted_mode->hdisplay))
			avi_if.body.avi.C_M_R |= HDMI_PICTURE_ASPECT_4_3 << 4;
		else if (!(adjusted_mode->vdisplay % 9) &&
			((adjusted_mode->vdisplay * 16 / 9) ==
			adjusted_mode->hdisplay))
			avi_if.body.avi.C_M_R |= HDMI_PICTURE_ASPECT_16_9 << 4;
	}

	if (avi_if.body.avi.VIC) {
		/* colorimetry: Sections 5.1 and 5.2 of CEA 861-D spec */
		if ((adjusted_mode->vdisplay == 480) ||
			(adjusted_mode->vdisplay == 576) ||
			(adjusted_mode->vdisplay == 240) ||
			(adjusted_mode->vdisplay == 288)) {
			avi_if.body.avi.C_M_R |= DIP_AVI_COLOR_ITU601;
		} else if ((adjusted_mode->vdisplay == 720) ||
			(adjusted_mode->vdisplay == 1080)) {
			avi_if.body.avi.C_M_R |= DIP_AVI_COLOR_ITU709;
		}
	}

	avi_if.body.avi.ITC_EC_Q_SC |= DIP_AVI_IT_CONTENT;
	intel_set_infoframe(encoder, &avi_if);

}

static void intel_hdmi_set_spd_infoframe(struct drm_encoder *encoder)
{
	union hdmi_infoframe frame;
	int ret;

	ret = hdmi_spd_infoframe_init(&frame.spd, "Intel", "Integrated gfx");
	if (ret < 0) {
		DRM_ERROR("couldn't fill SPD infoframe\n");
		return;
	}

	frame.spd.sdi = HDMI_SPD_SDI_PC;

	intel_write_infoframe(encoder, &frame);
}

static void g4x_set_infoframes(struct drm_encoder *encoder,
			       struct drm_display_mode *adjusted_mode)
{
	struct drm_i915_private *dev_priv = encoder->dev->dev_private;
	struct intel_digital_port *intel_dig_port = enc_to_dig_port(encoder);
	struct intel_hdmi *intel_hdmi = &intel_dig_port->hdmi;
	u32 reg = VIDEO_DIP_CTL;
	u32 val = I915_READ(reg);
	u32 port;

	assert_hdmi_port_disabled(intel_hdmi);

	/* If the registers were not initialized yet, they might be zeroes,
	 * which means we're selecting the AVI DIP and we're setting its
	 * frequency to once. This seems to really confuse the HW and make
	 * things stop working (the register spec says the AVI always needs to
	 * be sent every VSync). So here we avoid writing to the register more
	 * than we need and also explicitly select the AVI DIP and explicitly
	 * set its frequency to every VSync. Avoiding to write it twice seems to
	 * be enough to solve the problem, but being defensive shouldn't hurt us
	 * either. */
	val |= VIDEO_DIP_SELECT_AVI | VIDEO_DIP_FREQ_VSYNC;

	if (!intel_hdmi->has_hdmi_sink) {
		if (!(val & VIDEO_DIP_ENABLE))
			return;
		val &= ~VIDEO_DIP_ENABLE;
		I915_WRITE(reg, val);
		POSTING_READ(reg);
		return;
	}

	switch (intel_dig_port->port) {
	case PORT_B:
		port = VIDEO_DIP_PORT_B;
		break;
	case PORT_C:
		port = VIDEO_DIP_PORT_C;
		break;
	default:
		BUG();
		return;
	}

	if (port != (val & VIDEO_DIP_PORT_MASK)) {
		if (val & VIDEO_DIP_ENABLE) {
			val &= ~VIDEO_DIP_ENABLE;
			I915_WRITE(reg, val);
			POSTING_READ(reg);
		}
		val &= ~VIDEO_DIP_PORT_MASK;
		val |= port;
	}

	val |= VIDEO_DIP_ENABLE;
	val &= ~VIDEO_DIP_ENABLE_VENDOR;

	I915_WRITE(reg, val);
	POSTING_READ(reg);

	intel_hdmi_set_avi_infoframe(encoder, adjusted_mode);
	intel_hdmi_set_spd_infoframe(encoder);
}

static void ibx_set_infoframes(struct drm_encoder *encoder,
			       struct drm_display_mode *adjusted_mode)
{
	struct drm_i915_private *dev_priv = encoder->dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	struct intel_digital_port *intel_dig_port = enc_to_dig_port(encoder);
	struct intel_hdmi *intel_hdmi = &intel_dig_port->hdmi;
	u32 reg = TVIDEO_DIP_CTL(intel_crtc->pipe);
	u32 val = I915_READ(reg);
	u32 port;

	assert_hdmi_port_disabled(intel_hdmi);

	/* See the big comment in g4x_set_infoframes() */
	val |= VIDEO_DIP_SELECT_AVI | VIDEO_DIP_FREQ_VSYNC;

	if (!intel_hdmi->has_hdmi_sink) {
		if (!(val & VIDEO_DIP_ENABLE))
			return;
		val &= ~VIDEO_DIP_ENABLE;
		I915_WRITE(reg, val);
		POSTING_READ(reg);
		return;
	}

	switch (intel_dig_port->port) {
	case PORT_B:
		port = VIDEO_DIP_PORT_B;
		break;
	case PORT_C:
		port = VIDEO_DIP_PORT_C;
		break;
	case PORT_D:
		port = VIDEO_DIP_PORT_D;
		break;
	default:
		BUG();
		return;
	}

	if (port != (val & VIDEO_DIP_PORT_MASK)) {
		if (val & VIDEO_DIP_ENABLE) {
			val &= ~VIDEO_DIP_ENABLE;
			I915_WRITE(reg, val);
			POSTING_READ(reg);
		}
		val &= ~VIDEO_DIP_PORT_MASK;
		val |= port;
	}

	val |= VIDEO_DIP_ENABLE;
	val &= ~(VIDEO_DIP_ENABLE_VENDOR | VIDEO_DIP_ENABLE_GAMUT |
		 VIDEO_DIP_ENABLE_GCP);

	I915_WRITE(reg, val);
	POSTING_READ(reg);

	intel_hdmi_set_avi_infoframe(encoder, adjusted_mode);
	intel_hdmi_set_spd_infoframe(encoder);
}

static void cpt_set_infoframes(struct drm_encoder *encoder,
			       struct drm_display_mode *adjusted_mode)
{
	struct drm_i915_private *dev_priv = encoder->dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(encoder);
	u32 reg = TVIDEO_DIP_CTL(intel_crtc->pipe);
	u32 val = I915_READ(reg);

	assert_hdmi_port_disabled(intel_hdmi);

	/* See the big comment in g4x_set_infoframes() */
	val |= VIDEO_DIP_SELECT_AVI | VIDEO_DIP_FREQ_VSYNC;

	if (!intel_hdmi->has_hdmi_sink) {
		if (!(val & VIDEO_DIP_ENABLE))
			return;
		val &= ~(VIDEO_DIP_ENABLE | VIDEO_DIP_ENABLE_AVI);
		I915_WRITE(reg, val);
		POSTING_READ(reg);
		return;
	}

	/* Set both together, unset both together: see the spec. */
	val |= VIDEO_DIP_ENABLE | VIDEO_DIP_ENABLE_AVI;
	val &= ~(VIDEO_DIP_ENABLE_VENDOR | VIDEO_DIP_ENABLE_GAMUT |
		 VIDEO_DIP_ENABLE_GCP);

	I915_WRITE(reg, val);
	POSTING_READ(reg);

	intel_hdmi_set_avi_infoframe(encoder, adjusted_mode);
	intel_hdmi_set_spd_infoframe(encoder);
}

static void vlv_set_infoframes(struct drm_encoder *encoder,
			       struct drm_display_mode *adjusted_mode)
{
	struct drm_i915_private *dev_priv = encoder->dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(encoder);
	u32 reg = VLV_TVIDEO_DIP_CTL(intel_crtc->pipe);
	u32 val = I915_READ(reg);

	assert_hdmi_port_disabled(intel_hdmi);

	/* See the big comment in g4x_set_infoframes() */
	val |= VIDEO_DIP_SELECT_AVI | VIDEO_DIP_FREQ_VSYNC;

	if (!intel_hdmi->has_hdmi_sink) {
		if (!(val & VIDEO_DIP_ENABLE))
			return;
		val &= ~VIDEO_DIP_ENABLE;
		I915_WRITE(reg, val);
		POSTING_READ(reg);
		return;
	}

	val |= VIDEO_DIP_ENABLE;
	val &= ~(VIDEO_DIP_ENABLE_VENDOR | VIDEO_DIP_ENABLE_GAMUT |
		 VIDEO_DIP_ENABLE_GCP);

	I915_WRITE(reg, val);
	POSTING_READ(reg);

	intel_hdmi_set_avi_infoframe(encoder, adjusted_mode);
	intel_hdmi_set_spd_infoframe(encoder);
}

static void hsw_set_infoframes(struct drm_encoder *encoder,
			       struct drm_display_mode *adjusted_mode)
{
	struct drm_i915_private *dev_priv = encoder->dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(encoder);
	u32 reg = HSW_TVIDEO_DIP_CTL(intel_crtc->config.cpu_transcoder);
	u32 val = I915_READ(reg);

	assert_hdmi_port_disabled(intel_hdmi);

	if (!intel_hdmi->has_hdmi_sink) {
		I915_WRITE(reg, 0);
		POSTING_READ(reg);
		return;
	}

	val &= ~(VIDEO_DIP_ENABLE_VSC_HSW | VIDEO_DIP_ENABLE_GCP_HSW |
		 VIDEO_DIP_ENABLE_VS_HSW | VIDEO_DIP_ENABLE_GMP_HSW);

	I915_WRITE(reg, val);
	POSTING_READ(reg);

	intel_hdmi_set_avi_infoframe(encoder, adjusted_mode);
	intel_hdmi_set_spd_infoframe(encoder);
}

static void intel_hdmi_mode_set(struct intel_encoder *encoder)
{
	struct drm_device *dev = encoder->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *crtc = to_intel_crtc(encoder->base.crtc);
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(&encoder->base);
	struct drm_display_mode *adjusted_mode = &crtc->config.adjusted_mode;
	u32 hdmi_val;

	hdmi_val = SDVO_ENCODING_HDMI;
	if (!HAS_PCH_SPLIT(dev))
		hdmi_val |= intel_hdmi->color_range;
	if (adjusted_mode->flags & DRM_MODE_FLAG_PVSYNC)
		hdmi_val |= SDVO_VSYNC_ACTIVE_HIGH;
	if (adjusted_mode->flags & DRM_MODE_FLAG_PHSYNC)
		hdmi_val |= SDVO_HSYNC_ACTIVE_HIGH;

	if (crtc->config.pipe_bpp > 24)
		hdmi_val |= HDMI_COLOR_FORMAT_12bpc;
	else
		hdmi_val |= SDVO_COLOR_FORMAT_8bpc;

	/* Required on CPT */
	if (intel_hdmi->has_hdmi_sink && HAS_PCH_CPT(dev))
		hdmi_val |= HDMI_MODE_SELECT_HDMI;

	if (intel_hdmi->has_audio) {
		DRM_DEBUG_DRIVER("Enabling HDMI audio on pipe %c\n",
				 pipe_name(crtc->pipe));
		hdmi_val |= SDVO_AUDIO_ENABLE;
		hdmi_val |= HDMI_MODE_SELECT_HDMI;
		intel_write_eld(&encoder->base, adjusted_mode);
	}

	if (HAS_PCH_CPT(dev))
		hdmi_val |= SDVO_PIPE_SEL_CPT(crtc->pipe);
	else
		hdmi_val |= SDVO_PIPE_SEL(crtc->pipe);

	I915_WRITE(intel_hdmi->hdmi_reg, hdmi_val);
	POSTING_READ(intel_hdmi->hdmi_reg);

	intel_hdmi->set_infoframes(&encoder->base, adjusted_mode);
	switch_set_state(&intel_hdmi->sdev, 1);
}

static bool intel_hdmi_get_hw_state(struct intel_encoder *encoder,
				    enum pipe *pipe)
{
	struct drm_device *dev = encoder->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(&encoder->base);
	u32 tmp;

	tmp = I915_READ(intel_hdmi->hdmi_reg);

	if (!(tmp & SDVO_ENABLE))
		return false;

	if (HAS_PCH_CPT(dev))
		*pipe = PORT_TO_PIPE_CPT(tmp);
	else
		*pipe = PORT_TO_PIPE(tmp);

	return true;
}

static void intel_hdmi_get_config(struct intel_encoder *encoder,
				  struct intel_crtc_config *pipe_config)
{
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(&encoder->base);
	struct drm_i915_private *dev_priv = encoder->base.dev->dev_private;
	u32 tmp, flags = 0;

	tmp = I915_READ(intel_hdmi->hdmi_reg);

	if (tmp & SDVO_HSYNC_ACTIVE_HIGH)
		flags |= DRM_MODE_FLAG_PHSYNC;
	else
		flags |= DRM_MODE_FLAG_NHSYNC;

	if (tmp & SDVO_VSYNC_ACTIVE_HIGH)
		flags |= DRM_MODE_FLAG_PVSYNC;
	else
		flags |= DRM_MODE_FLAG_NVSYNC;

	pipe_config->adjusted_mode.flags |= flags;
}

static void intel_enable_hdmi(struct intel_encoder *encoder)
{
	struct drm_device *dev = encoder->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->base.crtc);
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(&encoder->base);
	u32 temp;
	u32 enable_bits = SDVO_ENABLE;

	if (intel_hdmi->has_audio)
		enable_bits |= SDVO_AUDIO_ENABLE;

	temp = I915_READ(intel_hdmi->hdmi_reg);

	/* HW workaround for IBX, we need to move the port to transcoder A
	 * before disabling it, so restore the transcoder select bit here. */
	if (HAS_PCH_IBX(dev))
		enable_bits |= SDVO_PIPE_SEL(intel_crtc->pipe);

	/* HW workaround, need to toggle enable bit off and on for 12bpc, but
	 * we do this anyway which shows more stable in testing.
	 */
	if (HAS_PCH_SPLIT(dev)) {
		I915_WRITE(intel_hdmi->hdmi_reg, temp & ~SDVO_ENABLE);
		POSTING_READ(intel_hdmi->hdmi_reg);
	}

	temp |= enable_bits;

	I915_WRITE(intel_hdmi->hdmi_reg, temp);
	POSTING_READ(intel_hdmi->hdmi_reg);

	/* HW workaround, need to write this twice for issue that may result
	 * in first write getting masked.
	 */
	if (HAS_PCH_SPLIT(dev)) {
		I915_WRITE(intel_hdmi->hdmi_reg, temp);
		POSTING_READ(intel_hdmi->hdmi_reg);
	}
}

int intel_hdmi_encoder_status(struct drm_encoder *encoder)
{
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 hdmib_control = I915_READ(HDMIB);

	if ((hdmib_control & SDVO_ENABLE) &&
		(hdmib_control & SDVO_AUDIO_ENABLE) &&
	    dev_priv->late_resume) {
		DRM_ERROR("HDMI encoder inuse!\n");
		return true;
	} else
		return false;
}

static void vlv_enable_hdmi(struct intel_encoder *encoder)
{
}

static void intel_disable_hdmi(struct intel_encoder *encoder)
{
	struct drm_device *dev = encoder->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(&encoder->base);
	u32 temp;
	u32 enable_bits = SDVO_ENABLE | SDVO_AUDIO_ENABLE;

	temp = I915_READ(intel_hdmi->hdmi_reg);

	/* HW workaround for IBX, we need to move the port to transcoder A
	 * before disabling it. */
	if (HAS_PCH_IBX(dev)) {
		struct drm_crtc *crtc = encoder->base.crtc;
		int pipe = crtc ? to_intel_crtc(crtc)->pipe : -1;

		if (temp & SDVO_PIPE_B_SELECT) {
			temp &= ~SDVO_PIPE_B_SELECT;
			I915_WRITE(intel_hdmi->hdmi_reg, temp);
			POSTING_READ(intel_hdmi->hdmi_reg);

			/* Again we need to write this twice. */
			I915_WRITE(intel_hdmi->hdmi_reg, temp);
			POSTING_READ(intel_hdmi->hdmi_reg);

			/* Transcoder selection bits only update
			 * effectively on vblank. */
			if (crtc)
				intel_wait_for_vblank(dev, pipe);
			else
				msleep(50);
		}
	}

	/* HW workaround, need to toggle enable bit off and on for 12bpc, but
	 * we do this anyway which shows more stable in testing.
	 */
	if (HAS_PCH_SPLIT(dev)) {
		I915_WRITE(intel_hdmi->hdmi_reg, temp & ~SDVO_ENABLE);
		POSTING_READ(intel_hdmi->hdmi_reg);
	}

	temp &= ~enable_bits;

	I915_WRITE(intel_hdmi->hdmi_reg, temp);
	POSTING_READ(intel_hdmi->hdmi_reg);

	/* HW workaround, need to write this twice for issue that may result
	 * in first write getting masked.
	 */
	if (HAS_PCH_SPLIT(dev)) {
		I915_WRITE(intel_hdmi->hdmi_reg, temp);
		POSTING_READ(intel_hdmi->hdmi_reg);
	}
}

static int intel_hdmi_mode_valid(struct drm_connector *connector,
				 struct drm_display_mode *mode)
{
	if (mode->clock > 165000)
		return MODE_CLOCK_HIGH;
	if (mode->clock < 20000)
		return MODE_CLOCK_LOW;

	if (mode->flags & DRM_MODE_FLAG_DBLSCAN)
		return MODE_NO_DBLESCAN;

	return MODE_OK;
}

bool intel_hdmi_compute_config(struct intel_encoder *encoder,
			       struct intel_crtc_config *pipe_config)
{
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(&encoder->base);
	struct drm_device *dev = encoder->base.dev;
	struct drm_display_mode *adjusted_mode = &pipe_config->adjusted_mode;
	int clock_12bpc = pipe_config->requested_mode.clock * 3 / 2;
	int desired_bpp;
#if ENABLE_PFIT_HDMI
	struct intel_crtc *intel_crtc = encoder->new_crtc;
	struct intel_connector *intel_connector = intel_hdmi->attached_connector;
#endif

	if (intel_hdmi->color_range_auto) {
		/* See CEA-861-E - 5.1 Default Encoding Parameters */
		if (intel_hdmi->has_hdmi_sink &&
		    drm_match_cea_mode(adjusted_mode) > 1)
			intel_hdmi->color_range = HDMI_COLOR_RANGE_16_235;
		else
			intel_hdmi->color_range = 0;
	}
	/*TODO: Panel fitter is not enabled for HDMI */

#if ENABLE_PFIT_HDMI
	if (IS_VALLEYVIEW(dev)) {
		intel_gmch_panel_fitting(intel_crtc, pipe_config,
				intel_connector->panel.fitting_mode);
	}
#endif
	if (intel_hdmi->color_range)
		pipe_config->limited_color_range = true;

	if (HAS_PCH_SPLIT(dev) && !HAS_DDI(dev))
		pipe_config->has_pch_encoder = true;

	/*
	 * HDMI is either 12 or 8, so if the display lets 10bpc sneak
	 * through, clamp it down. Note that g4x/vlv don't support 12bpc hdmi
	 * outputs. We also need to check that the higher clock still fits
	 * within limits.
	 */
	if (pipe_config->pipe_bpp > 8*3 && clock_12bpc <= 225000
	    && HAS_PCH_SPLIT(dev)) {
		DRM_DEBUG_KMS("picking bpc to 12 for HDMI output\n");
		desired_bpp = 12*3;

		/* Need to adjust the port link by 1.5x for 12bpc. */
		pipe_config->port_clock = clock_12bpc;
	} else {
		DRM_DEBUG_KMS("picking bpc to 8 for HDMI output\n");
		desired_bpp = 8*3;
	}

	if (!pipe_config->bw_constrained) {
		DRM_DEBUG_KMS("forcing pipe bpc to %i for HDMI\n", desired_bpp);
		pipe_config->pipe_bpp = desired_bpp;
	}

	if (adjusted_mode->clock > 225000) {
		DRM_DEBUG_KMS("too high HDMI clock, rejecting mode\n");
		return false;
	}

	return true;
}

static bool vlv_hdmi_live_status(struct drm_device *dev,
			struct intel_hdmi *intel_hdmi)
{
	uint32_t bit;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_digital_port *intel_dig_port =
					hdmi_to_dig_port(intel_hdmi);

	DRM_DEBUG_KMS("Reading Live status");
	switch (intel_dig_port->port) {
	case PORT_B:
		bit = HDMIB_HOTPLUG_LIVE_STATUS;
		break;
	case PORT_C:
		bit = HDMIC_HOTPLUG_LIVE_STATUS;
		break;
	case PORT_D:
		bit = HDMID_HOTPLUG_LIVE_STATUS;
		break;
	default:
		bit = 0;
	}

	/* Return results in trems of connector */
	return I915_READ(PORT_HOTPLUG_STAT) & bit;
}


/*
intel_hdmi_live_status: detect live status of HDMI
if device is gen 6 and above, read the live status reg
else, do not block the detection, return true
*/
static bool intel_hdmi_live_status(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;
	struct intel_hdmi *intel_hdmi = intel_attached_hdmi(connector);

	if (INTEL_INFO(dev)->gen > 6) {
		/* Todo: Implement for other Gen 6+ archs*/
		if (IS_VALLEYVIEW(dev))
			return vlv_hdmi_live_status(dev, intel_hdmi);
	}

	return true;
}

/*
intel_hdmi_send_uevent: inform usespace
about an event
*/
void intel_hdmi_send_uevent(struct drm_device *dev, char *uevent)
{
	char *envp[] = {uevent, NULL};
	/* Notify usp the change */
	kobject_uevent_env(&dev->primary->kdev.kobj, KOBJ_CHANGE, envp);
}

/* Read DDC and get EDID */
struct edid *intel_hdmi_get_edid(struct drm_connector *connector, bool force)
{
	bool current_state = false;
	bool saved_state = false;

	struct edid *new_edid = NULL;
	struct i2c_adapter *adapter = NULL;
	struct drm_device *dev = connector->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_hdmi *intel_hdmi = intel_attached_hdmi(connector);
	u32 hotplug_status = dev_priv->hotplug_status;
	enum port hdmi_port = hdmi_to_dig_port(intel_hdmi)->port;
	unsigned char retry = HDMI_EDID_RETRY_COUNT;

	if (!intel_hdmi) {
		DRM_ERROR("Invalid input to get hdmi\n");
		return NULL;
	}

	/* Get the saved status from top half */
	saved_state = hotplug_status & (1 << (HDMI_LIVE_STATUS_BASE - hdmi_port));

	/* Few monitors are slow to respond on EDID and live status,
	so read live status multiple times within a max delay of 30ms */
	do {
		mdelay(HDMI_LIVE_STATUS_DELAY_STEP);
		current_state = intel_hdmi_live_status(connector);
		if (current_state)
			break;
	} while (retry--);

	/* Compare current status, and saved status in top half */
	if (current_state != saved_state)
		DRM_DEBUG_DRIVER("Warning: Saved HDMI status != current status");

	/* Read EDID if live status or saved status is up, or we are forced */
	if (current_state || saved_state || force) {

		adapter = intel_gmbus_get_adapter(dev_priv,
					intel_hdmi->ddc_bus);
		if (!adapter) {
			DRM_ERROR("Get_hdmi cant get adapter\n");
			return NULL;
		}

		/* Few monitors issue EDID after some delay, so give them
		some chnaces, but within 30ms */
		retry = 3;
READ_EDID:
		new_edid = drm_get_edid(connector, adapter);
		if (!new_edid) {
			if (retry--) {
				mdelay(HDMI_LIVE_STATUS_DELAY_STEP);
				goto READ_EDID;
			}

			DRM_ERROR("Get_hdmi cant read edid\n");
			return NULL;
		}

		DRM_DEBUG_KMS("Live status up, got EDID");
	}

	return new_edid;
}

void intel_hdmi_reset(struct drm_connector *connector)
{
	struct intel_hdmi *intel_hdmi = intel_attached_hdmi(connector);
	struct drm_i915_private *dev_priv = connector->dev->dev_private;

	/* Clean previous detects and modes */
	dev_priv->is_hdmi = false;
	intel_hdmi->edid_mode_count = 0;
	intel_cleanup_modes(connector);
	connector->status = connector_status_disconnected;
}

/* Check if monitor is changed
* Returns 1, if changed
*        -1, if one or both the edids are null. This can happen in hotplug or
*           unplug cases
*         0, if no change is found.
*/
static enum monitor_changed_status
intel_hdmi_monitor_changed(struct edid *old_edid, struct edid *new_edid)
{
	if (!old_edid && !new_edid)
		return MONITOR_INVALID;

	if (!old_edid || !new_edid)
		return MONITOR_PLUG_UNPLUG;

	if (!(new_edid->mfg_id[0] == old_edid->mfg_id[0]
			&& new_edid->mfg_id[1] == old_edid->mfg_id[1]
			&& new_edid->prod_code[0] == old_edid->prod_code[0]
			&& new_edid->prod_code[1] == old_edid->prod_code[1])) {
		DRM_DEBUG_DRIVER("\nMonitor has changed during suspend\n");
		return MONITOR_CHANGED;
	}
	return MONITOR_UNCHANGED;
}
/* Encoder's Hot plug function
  * Caller must hold mode_config mutex
  * Read EDID based on Live status register
  */
void intel_hdmi_hot_plug(struct intel_encoder *intel_encoder)
{
	struct drm_encoder *encoder = &intel_encoder->base;
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(encoder);
	struct drm_device *dev = encoder->dev;
	struct drm_connector *connector = NULL;
	struct edid *edid = NULL;
	struct edid *old_edid = intel_hdmi->edid;
	struct drm_i915_private *dev_priv = dev->dev_private;
	bool need_event = false;
	int pipe = 0;

	connector = &intel_hdmi->attached_connector->base;
	/* We are here, means there is a HDMI hot-plug
	Lets try to get EDID */
	edid = intel_hdmi_get_edid(connector, false);
	if (edid) {
		DRM_DEBUG_DRIVER("Hdmi: Monitor connected\n");
		if (connector->status == connector_status_connected)
			need_event = true;
	} else {
		DRM_DEBUG_DRIVER("Hdmi: Monitor disconnected\n");
		if (intel_encoder->type == INTEL_OUTPUT_HDMI &&
				to_intel_crtc(encoder->crtc)) {
			pipe = to_intel_crtc(encoder->crtc)->pipe;
			if (dev_priv->gamma_enabled[pipe])
				dev_priv->gamma_enabled[pipe] = false;
			if (dev_priv->csc_enabled[pipe])
				dev_priv->csc_enabled[pipe] = false;
		}
		if (connector->status == connector_status_disconnected)
			need_event = true;
	}

	if (dev_priv->late_resume) {
		if (intel_hdmi_monitor_changed(old_edid, edid) == MONITOR_CHANGED) {
			DRM_DEBUG_DRIVER("Hdmi: Monitor changed during suspend\n");
			intel_hdmi_reset(connector);

			/* When 'HDMI-Change' event is sent to user space, it checks the
			* validity of the current mode. The validity of the mode is decided
			* based upon the enabled flag of crtc in drm_crtc. Thats why
			* disabling this flag.
			*/
			intel_encoder->base.crtc->enabled = false;
			intel_hdmi_send_uevent(dev, "HDMI-Change");
		}
	}

	/* need_event
	* This check is required for HDMI compliance when few analyzers are
	* capable of generating on-the-fly EDID, and they just send a couple of connect
	* and disconnect event on EDID change. Consider this case:
	* 1. HDMI connect event with EDID 1 with mode 1
	* 2. Bottom half calls hot_plug() and detect, connector status = connected
	* 3. EDID switch to test EDID, first disconnect event (This is smaller in duration)
	* 4. Bottom half calls hot_plug()
	* 5. By the time hot_plug gets schedules, connect call comes, with new EDID
	* 6. Bottom half calls detect() which reports status = connected again
	* 7. Bottom half checks previous status = current status = connected so no event
		sent to userspace.
	* 8. In this way, the usp is never informed about EDID change, so HDMI tests fail
	* So if we are in HDMI hot_plug, there is some event
	*/
	if (need_event) {
		DRM_DEBUG_DRIVER("Sending self event");
		intel_hdmi_send_uevent(dev, "HOTPLUG=1");
	}
	if (edid == NULL) {
		switch_set_state(&intel_hdmi->sdev, 0);
	}

	/* Update EDID, kfree is NULL protected */
	kfree(intel_hdmi->edid);
	intel_hdmi->edid = edid;
	connector->display_info.raw_edid = (char *)edid;
}


static enum drm_connector_status
intel_hdmi_detect(struct drm_connector *connector, bool force)
{
	struct drm_device *dev = connector->dev;
	struct intel_hdmi *intel_hdmi = intel_attached_hdmi(connector);
	struct intel_digital_port *intel_dig_port =
		hdmi_to_dig_port(intel_hdmi);
	struct intel_encoder *intel_encoder = &intel_dig_port->base;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct edid *edid = NULL;
	enum drm_connector_status status = connector_status_disconnected;
	bool inform_audio = false;

	DRM_DEBUG_KMS("[CONNECTOR:%d:%s]\n",
		      connector->base.id, drm_get_connector_name(connector));

	/* If its force detection, dont read EDID again */
	if (force && dev_priv->is_hdmi) {
		/* Report only known status */
		if (connector->status != connector_status_unknown) {
			DRM_DEBUG_DRIVER("Reporting force status\n");
			return connector->status;
		}
	}

	/*
	* To avoid race condition between get_connector calls and bottom halves
	* of contineous hot-plug/unplug calls, hdmi_detect will always process
	* EDID available in struct intel_hdmi, whereas hdmi_hot_plug is suppose to
	* handle real EDID reads from hot plugs. So if HDMI is available, there
	* must be an EDID in intel_hdmi and viceversa.
	*/
	dev_priv->is_hdmi = false;
	intel_hdmi->rgb_quant_range_selectable = false;
	intel_hdmi->has_hdmi_sink = false;

	/* Need to inform audio about the event */
	if (intel_hdmi->has_audio)
		inform_audio = true;
	intel_hdmi->has_audio = false;

	edid = intel_hdmi->edid;
	if (edid) {
		status = connector_status_connected;
		if (edid->input & DRM_EDID_INPUT_DIGITAL) {
			dev_priv->is_hdmi = true;
			if (intel_hdmi->force_audio != HDMI_AUDIO_OFF_DVI)
				intel_hdmi->has_hdmi_sink =
						drm_detect_hdmi_monitor(edid);
			intel_hdmi->has_audio = drm_detect_monitor_audio(edid);
			intel_hdmi->rgb_quant_range_selectable =
				drm_rgb_quant_range_selectable(edid);
			connector->display_info.raw_edid = (char *)edid;
			dev_priv->unplug = false;
			DRM_DEBUG_DRIVER("Got edid, HDMI connected\n");
		} else {
			DRM_ERROR("EDID not in digital form ?\n");
			return status;
		}
	} else {
		DRM_DEBUG_DRIVER("No edid, HDMI disconnected\n");
		dev_priv->unplug = true;
		connector->display_info.raw_edid = NULL;
		status = connector_status_disconnected;
		intel_cleanup_modes(connector);
		intel_hdmi->edid_mode_count = 0;
	}

	/*
	* If HDMI status is conencted, the event to audio will be
	* sent on basis of current audio status, but if its disconnected, the
	* status will be sent based on previous audio status
	*/
	if ((status != i915_hdmi_state)) {
		if (status == connector_status_connected) {
			if (intel_hdmi->has_audio)
				i915_notify_had = 1;
			if (intel_hdmi->force_audio != HDMI_AUDIO_AUTO)
				intel_hdmi->has_audio =
					(intel_hdmi->force_audio == HDMI_AUDIO_ON);
			intel_encoder->type = INTEL_OUTPUT_HDMI;
		} else {
			/* Send a disconnect event to audio */
			if (inform_audio) {
				DRM_DEBUG_DRIVER("Sending event to audio");
				mid_hdmi_audio_signal_event(dev_priv->dev,
				HAD_EVENT_HOT_UNPLUG);
			}
		}
	}

	i915_hdmi_state = status;
	return status;
}

static int intel_hdmi_get_modes(struct drm_connector *connector)
{
	struct intel_hdmi *intel_hdmi = intel_attached_hdmi(connector);
	struct edid *edid = NULL;
	int ret = 0;

	/* No need to read modes if no connection */
	if (connector->status != connector_status_connected)
		return ret;

	DRM_DEBUG_DRIVER("Reading modes from EDID");
	/* EDID was saved in detect, re-use that if available, avoid
	reading EDID everytime */
	edid =  intel_hdmi->edid;
	if (edid) {
		drm_mode_connector_update_edid_property(connector, edid);
		ret = drm_add_edid_modes(connector, edid);
		drm_edid_to_eld(connector, edid);
		hdmi_get_eld(connector->eld);
	}

	/* Update the mode status */
	intel_hdmi->edid_mode_count = ret;
	return ret;
}

static bool
intel_hdmi_detect_audio(struct drm_connector *connector)
{
	struct intel_hdmi *intel_hdmi = intel_attached_hdmi(connector);
	struct drm_i915_private *dev_priv = connector->dev->dev_private;
	struct edid *edid;
	bool has_audio = false;
	struct i2c_adapter *i2c;

	i2c = intel_gmbus_get_adapter(dev_priv, intel_hdmi->ddc_bus);
	if (i2c == NULL)
		return false;
	edid = drm_get_edid(connector, i2c);
	if (edid) {
		if (edid->input & DRM_EDID_INPUT_DIGITAL)
			has_audio = drm_detect_monitor_audio(edid);
		kfree(edid);
	}

	return has_audio;
}

static int
intel_hdmi_set_property(struct drm_connector *connector,
			struct drm_property *property,
			uint64_t val)
{
	struct intel_hdmi *intel_hdmi = intel_attached_hdmi(connector);
	struct intel_digital_port *intel_dig_port =
		hdmi_to_dig_port(intel_hdmi);
	struct drm_i915_private *dev_priv = connector->dev->dev_private;
	struct intel_connector *intel_connector = to_intel_connector(connector);
	struct intel_encoder *encoder = intel_connector->encoder;
	struct intel_crtc *intel_crtc = encoder->new_crtc;
	int ret;

	ret = drm_object_property_set_value(&connector->base, property, val);
	if (ret)
		return ret;

	if (property == dev_priv->force_audio_property) {
		enum hdmi_force_audio i = val;
		bool has_audio;

		if (i == intel_hdmi->force_audio)
			return 0;

		intel_hdmi->force_audio = i;

		if (i == HDMI_AUDIO_AUTO)
			has_audio = intel_hdmi_detect_audio(connector);
		else
			has_audio = (i == HDMI_AUDIO_ON);

		if (i == HDMI_AUDIO_OFF_DVI)
			intel_hdmi->has_hdmi_sink = 0;

		intel_hdmi->has_audio = has_audio;
		goto done;
	}

	if (property == dev_priv->broadcast_rgb_property) {
		bool old_auto = intel_hdmi->color_range_auto;
		uint32_t old_range = intel_hdmi->color_range;

		switch (val) {
		case INTEL_BROADCAST_RGB_AUTO:
			intel_hdmi->color_range_auto = true;
			break;
		case INTEL_BROADCAST_RGB_FULL:
			intel_hdmi->color_range_auto = false;
			intel_hdmi->color_range = 0;
			break;
		case INTEL_BROADCAST_RGB_LIMITED:
			intel_hdmi->color_range_auto = false;
			intel_hdmi->color_range = HDMI_COLOR_RANGE_16_235;
			break;
		default:
			return -EINVAL;
		}

		if (old_auto == intel_hdmi->color_range_auto &&
		    old_range == intel_hdmi->color_range)
			return 0;

		goto done;
	}
	/* TODO: Panel fitter is not enabled for HDMI */
#if ENABLE_PFIT_HDMI
	if (property == dev_priv->force_pfit_property) {
		if (intel_connector->panel.fitting_mode == val)
			return 0;
		intel_connector->panel.fitting_mode = val;

		if (IS_VALLEYVIEW(dev_priv->dev)) {
			intel_gmch_panel_fitting(intel_crtc, &intel_crtc->config,
					intel_connector->panel.fitting_mode);
			return 0;
		} else
			goto done;
	}
#endif
	if (property == dev_priv->scaling_src_size_property) {
		intel_crtc->scaling_src_size = val;
		DRM_DEBUG_DRIVER("src size = %x", intel_crtc->scaling_src_size);
		return 0;
	}

	return -EINVAL;

done:
	if (intel_dig_port->base.base.crtc)
		intel_crtc_restore_mode(intel_dig_port->base.base.crtc);

	return 0;
}

static void vlv_set_hdmi_level_shifter_settings(struct intel_encoder *encoder)
{
	struct drm_device *dev = encoder->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *crtc = to_intel_crtc(encoder->base.crtc);
	struct drm_display_mode *adjusted_mode = &crtc->config.adjusted_mode;
	struct intel_digital_port *dport = enc_to_dig_port(&encoder->base);
	int port = vlv_dport_to_channel(dport);
	bool disable_deemph_for_hdmi_clk = false;

	u32 de_emp_reg_val = 0;
	u32 transcale_reg_val = 0;
	u32 pre_emp_reg_val = 0;
	u32 clk_de_emp_reg_val = 0;

	/*
	 * Obtaining HDMI pre-emp, vswing settings from VBT.
	 * definitions:
	 * 0 = 1000MV_2DB, 1 = 1000MV_0DB, 2 = 800MV_0DB,
	 * 3 =  600MV_2DB, 4 =  600MV_0DB
	 */
	u8 pre_emp_vswing_setting = dev_priv->vbt.hdmi_level_shifter;
	DRM_DEBUG_KMS("HDMI pre_emp vswing setting from VBT : %d\n",
						pre_emp_vswing_setting);

	/*
	 * As per EV requirement need to set 1000MV_0DB for pixel clock
	 * < 74.250 Mhz
	 */
	if (adjusted_mode->clock < 74250) {
		pre_emp_vswing_setting = HDMI_VSWING_1000MV_0DB;
		DRM_DEBUG_KMS("Forcing HDMI vswing setting to 1V_0DB\n");
	}

	/*FIXME: The Application notes doesn't have pcs_ctrl_reg_val for
	 * settings 1V_0DB, 0.8V_0DB, 0.6V_0DB. The pcs_ctrl_reg_val value
	 * for these is selected from higher demp_vswing settings for which
	 * the data is given.
	 */
	switch (pre_emp_vswing_setting) {
	case HDMI_VSWING_1000MV_2DB:
		de_emp_reg_val = 0x2B245F5F;
		transcale_reg_val = 0x5578B83A;
		clk_de_emp_reg_val = 0x2B247878;
		pre_emp_reg_val = 0x2000;
		disable_deemph_for_hdmi_clk = true;
		break;
	case HDMI_VSWING_1000MV_0DB:
		de_emp_reg_val = 0x2B405555;
		transcale_reg_val = 0x5580A03A;
		clk_de_emp_reg_val = 0x2B405555;
		pre_emp_reg_val = 0x4000;
		break;
	case HDMI_VSWING_800MV_0DB:
		de_emp_reg_val = 0x2B245555;
		transcale_reg_val = 0x5560B83A;
		clk_de_emp_reg_val = 0x2B245555;
		pre_emp_reg_val = 0x4000;
		break;
	case HDMI_VSWING_600MV_2DB:
		de_emp_reg_val = 0x2B406262;
		transcale_reg_val = 0x5560B83A;
		clk_de_emp_reg_val = 0x2B407878;
		pre_emp_reg_val = 0x2000;
		disable_deemph_for_hdmi_clk = true;
		break;
	case HDMI_VSWING_600MV_0DB:
		de_emp_reg_val = 0x2B404040;
		transcale_reg_val = 0x5548B83A;
		clk_de_emp_reg_val = 0x2B404040;
		pre_emp_reg_val = 0x4000;
		break;
	default:	/* HDMI_VSWING_1000MV_2DB*/
		DRM_ERROR("Incorrect pre-emp vswing setting\n");
		de_emp_reg_val = 0x2B245F5F;
		transcale_reg_val = 0x5578B83A;
		clk_de_emp_reg_val = 0x2B247878;
		pre_emp_reg_val = 0x2000;
		disable_deemph_for_hdmi_clk = true;
		break;
	}


	vlv_dpio_write(dev_priv, DPIO_TX_OCALINIT(port), 0x00000000);
	vlv_dpio_write(dev_priv, DPIO_TX_SWING_CTL4(port), de_emp_reg_val);
	vlv_dpio_write(dev_priv, DPIO_TX_SWING_CTL2(port), transcale_reg_val);
	vlv_dpio_write(dev_priv, DPIO_TX_SWING_CTL3(port), 0x0c782040);

	/*
	 * As per the HW Application notes, for 0DB modes we need not disable
	 * the deemph for HDMI clk
	 */
	if (disable_deemph_for_hdmi_clk)
		vlv_dpio_write(dev_priv, DPIO_TX3_SWING_CTL4(port), clk_de_emp_reg_val);

	vlv_dpio_write(dev_priv, DPIO_PCS_STAGGER0(port), 0x00030000);
	vlv_dpio_write(dev_priv, DPIO_PCS_CTL_OVER1(port), pre_emp_reg_val);
	vlv_dpio_write(dev_priv, DPIO_TX_OCALINIT(port), DPIO_TX_OCALINIT_EN);
}

static void intel_hdmi_pre_enable(struct intel_encoder *encoder)
{
	struct intel_digital_port *dport = enc_to_dig_port(&encoder->base);
	struct drm_device *dev = encoder->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc =
		to_intel_crtc(encoder->base.crtc);
	int port = vlv_dport_to_channel(dport);
	int pipe = intel_crtc->pipe;
	u32 val;

	if (!IS_VALLEYVIEW(dev))
		return;

	/* Enable clock channels for this port */
	mutex_lock(&dev_priv->dpio_lock);

	val = 0;
	if (pipe)
		val |= (1<<21);
	else
		val &= ~(1<<21);
	val |= 0x001000c4;
	vlv_dpio_write(dev_priv, DPIO_DATA_CHANNEL(port), val);

	vlv_set_hdmi_level_shifter_settings(encoder);

	/* Program lane clock */
	vlv_dpio_write(dev_priv, DPIO_PCS_CLOCKBUF0(port),
			 0x00760018);
	vlv_dpio_write(dev_priv, DPIO_PCS_CLOCKBUF8(port),
			 0x00400888);
	mutex_unlock(&dev_priv->dpio_lock);

	intel_enable_hdmi(encoder);

	vlv_wait_port_ready(dev_priv, port);
}

static void intel_hdmi_pre_pll_enable(struct intel_encoder *encoder)
{
	struct intel_digital_port *dport = enc_to_dig_port(&encoder->base);
	struct drm_device *dev = encoder->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int port = vlv_dport_to_channel(dport);

	if (!IS_VALLEYVIEW(dev))
		return;

	/* Program Tx lane resets to default */
	mutex_lock(&dev_priv->dpio_lock);
	vlv_dpio_write(dev_priv, DPIO_PCS_TX(port),
			 DPIO_PCS_TX_LANE2_RESET |
			 DPIO_PCS_TX_LANE1_RESET);
	vlv_dpio_write(dev_priv, DPIO_PCS_CLK(port),
			 DPIO_PCS_CLK_CRI_RXEB_EIOS_EN |
			 DPIO_PCS_CLK_CRI_RXDIGFILTSG_EN |
			 (1<<DPIO_PCS_CLK_DATAWIDTH_SHIFT) |
			 DPIO_PCS_CLK_SOFT_RESET);

	/* Fix up inter-pair skew failure */
	vlv_dpio_write(dev_priv, DPIO_PCS_STAGGER1(port), 0x00750f00);
	vlv_dpio_write(dev_priv, DPIO_TX_CTL(port), 0x00001500);
	vlv_dpio_write(dev_priv, DPIO_TX_LANE(port), 0x40400000);

	mutex_unlock(&dev_priv->dpio_lock);
}

static void intel_hdmi_post_disable(struct intel_encoder *encoder)
{
	struct intel_digital_port *dport = enc_to_dig_port(&encoder->base);
	struct drm_i915_private *dev_priv = encoder->base.dev->dev_private;
	int port = vlv_dport_to_channel(dport);

	/* Reset lanes to avoid HDMI flicker (VLV w/a) */
	mutex_lock(&dev_priv->dpio_lock);
	vlv_dpio_write(dev_priv, DPIO_PCS_TX(port), 0x00000000);
	vlv_dpio_write(dev_priv, DPIO_PCS_CLK(port), 0x00e00060);
	mutex_unlock(&dev_priv->dpio_lock);
}

static void intel_hdmi_destroy(struct drm_connector *connector)
{
	drm_sysfs_connector_remove(connector);
	drm_connector_cleanup(connector);
	kfree(connector);
}

static const struct drm_encoder_helper_funcs intel_hdmi_helper_funcs = {
	.inuse = intel_hdmi_encoder_status,
};

static const struct drm_connector_funcs intel_hdmi_connector_funcs = {
	.dpms = intel_connector_dpms,
	.detect = intel_hdmi_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.set_property = intel_hdmi_set_property,
	.destroy = intel_hdmi_destroy,
};

static const struct drm_connector_helper_funcs intel_hdmi_connector_helper_funcs = {
	.get_modes = intel_hdmi_get_modes,
	.mode_valid = intel_hdmi_mode_valid,
	.best_encoder = intel_best_encoder,
};

static const struct drm_encoder_funcs intel_hdmi_enc_funcs = {
	.destroy = intel_encoder_destroy,
};

static void
intel_hdmi_add_properties(struct intel_hdmi *intel_hdmi, struct drm_connector *connector)
{
	intel_attach_force_audio_property(connector);
	intel_attach_broadcast_rgb_property(connector);
	intel_attach_force_pfit_property(connector);
	intel_attach_scaling_src_size_property(connector);
	intel_hdmi->color_range_auto = true;
}

void intel_hdmi_init_connector(struct intel_digital_port *intel_dig_port,
			       struct intel_connector *intel_connector)
{
	struct drm_connector *connector = &intel_connector->base;
	struct intel_hdmi *intel_hdmi = &intel_dig_port->hdmi;
	struct intel_encoder *intel_encoder = &intel_dig_port->base;
	struct drm_device *dev = intel_encoder->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	enum port port = intel_dig_port->port;

	drm_connector_init(dev, connector, &intel_hdmi_connector_funcs,
			   DRM_MODE_CONNECTOR_HDMIA);
	drm_connector_helper_add(connector, &intel_hdmi_connector_helper_funcs);

	connector->interlace_allowed = 0;
	connector->doublescan_allowed = 0;

	switch (port) {
	case PORT_B:
		intel_hdmi->ddc_bus = GMBUS_PORT_DPB;
		intel_encoder->hpd_pin = HPD_PORT_B;
		break;
	case PORT_C:
		intel_hdmi->ddc_bus = GMBUS_PORT_DPC;
		intel_encoder->hpd_pin = HPD_PORT_C;
		break;
	case PORT_D:
		intel_hdmi->ddc_bus = GMBUS_PORT_DPD;
		intel_encoder->hpd_pin = HPD_PORT_D;
		break;
	case PORT_A:
		intel_encoder->hpd_pin = HPD_PORT_A;
		/* Internal port only for eDP. */
	default:
		BUG();
	}

	if (IS_VALLEYVIEW(dev)) {
		intel_hdmi->write_infoframe = vlv_write_infoframe;
		intel_hdmi->set_infoframes = vlv_set_infoframes;
	} else if (!HAS_PCH_SPLIT(dev)) {
		intel_hdmi->write_infoframe = g4x_write_infoframe;
		intel_hdmi->set_infoframes = g4x_set_infoframes;
	} else if (HAS_DDI(dev)) {
		intel_hdmi->write_infoframe = hsw_write_infoframe;
		intel_hdmi->set_infoframes = hsw_set_infoframes;
	} else if (HAS_PCH_IBX(dev)) {
		intel_hdmi->write_infoframe = ibx_write_infoframe;
		intel_hdmi->set_infoframes = ibx_set_infoframes;
	} else {
		intel_hdmi->write_infoframe = cpt_write_infoframe;
		intel_hdmi->set_infoframes = cpt_set_infoframes;
	}

	drm_encoder_helper_add(&intel_encoder->base, &intel_hdmi_helper_funcs);

	if (HAS_DDI(dev))
		intel_connector->get_hw_state = intel_ddi_connector_get_hw_state;
	else
		intel_connector->get_hw_state = intel_connector_get_hw_state;

	intel_hdmi_add_properties(intel_hdmi, connector);

	intel_connector_attach_encoder(intel_connector, intel_encoder);
	drm_sysfs_connector_add(connector);

	/* For G4X desktop chip, PEG_BAND_GAP_DATA 3:0 must first be written
	 * 0xd.  Failure to do so will result in spurious interrupts being
	 * generated on the port when a cable is not attached.
	 */
	if (IS_G4X(dev) && !IS_GM45(dev)) {
		u32 temp = I915_READ(PEG_BAND_GAP_DATA);
		I915_WRITE(PEG_BAND_GAP_DATA, (temp & ~0xf) | 0xd);
	}

	/* Load initialized connector */
	intel_hdmi->attached_connector = intel_connector;

	/*
	* Probe the first state of HDMI forcefully. This is required as
	* EDID read is happening only it hot_plug() functions, but in
	* few configurations kms or fb_console drivers call detect()
	* not the hot_plug(). After init, we can detect plug-in/out in
	* hot-plug functions
	*/
	intel_hdmi->edid = intel_hdmi_get_edid(connector, true);

	/* Update the first status */
	connector->status = intel_hdmi_detect(connector, false);

	dev_priv->port_disabled_on_unplug = false;
}

/* Added for HDMI Audio */
void i915_had_wq(struct work_struct *work)
{
	struct drm_i915_private *dev_priv = container_of(work,
		struct drm_i915_private, hdmi_audio_wq);

	DRM_ERROR("Checking for HDMI connection at boot\n");
	if (i915_hdmi_state == connector_status_connected) {
		DRM_ERROR("hdmi_do_audio_wq: HDMI plugged in\n");
		mid_hdmi_audio_signal_event(dev_priv->dev,
			HAD_EVENT_HOT_PLUG);
	}
}

void intel_hdmi_init(struct drm_device *dev, int hdmi_reg, enum port port)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_digital_port *intel_dig_port;
	struct intel_encoder *intel_encoder;
	struct intel_connector *intel_connector;
	/* Added for HDMI Audio */
	struct hdmi_audio_priv *hdmi_priv;

	intel_dig_port = kzalloc(sizeof(struct intel_digital_port), GFP_KERNEL);
	if (!intel_dig_port)
		return;

	intel_connector = kzalloc(sizeof(struct intel_connector), GFP_KERNEL);
	if (!intel_connector) {
		kfree(intel_dig_port);
		return;
	}

	intel_encoder = &intel_dig_port->base;

	drm_encoder_init(dev, &intel_encoder->base, &intel_hdmi_enc_funcs,
			 DRM_MODE_ENCODER_TMDS);

	intel_encoder->compute_config = intel_hdmi_compute_config;
	intel_encoder->mode_set = intel_hdmi_mode_set;
	intel_encoder->disable = intel_disable_hdmi;
	intel_encoder->get_hw_state = intel_hdmi_get_hw_state;
	intel_encoder->get_config = intel_hdmi_get_config;
	if (IS_VALLEYVIEW(dev)) {
		intel_encoder->pre_pll_enable = intel_hdmi_pre_pll_enable;
		intel_encoder->pre_enable = intel_hdmi_pre_enable;
		intel_encoder->enable = vlv_enable_hdmi;
		intel_encoder->post_disable = intel_hdmi_post_disable;
		intel_encoder->hot_plug = intel_hdmi_hot_plug;
	} else {
		intel_encoder->enable = intel_enable_hdmi;
	}

	intel_encoder->type = INTEL_OUTPUT_HDMI;
	intel_encoder->crtc_mask = (1 << 0) | (1 << 1) | (1 << 2);
	intel_encoder->cloneable = false;

	intel_dig_port->port = port;
	intel_dig_port->hdmi.hdmi_reg = hdmi_reg;
	intel_dig_port->dp.output_reg = 0;

	intel_hdmi_init_connector(intel_dig_port, intel_connector);
	intel_dig_port->hdmi.sdev.name = "hdmi";
	if (switch_dev_register(&intel_dig_port->hdmi.sdev) < 0) {
	    DRM_ERROR("Hdmi switch_dev registration failed\n");
	}
	/* Added for HDMI Audio */
	/* HDMI private data */
	INIT_WORK(&dev_priv->hdmi_audio_wq, i915_had_wq);
	hdmi_priv = kzalloc(sizeof(struct hdmi_audio_priv), GFP_KERNEL);
	if (!hdmi_priv) {
		pr_err("failed to allocate memory");
	} else {
		hdmi_priv->dev = dev;
		hdmi_priv->hdmib_reg = HDMIB;
		hdmi_priv->monitor_type = MONITOR_TYPE_HDMI;
		hdmi_priv->is_hdcp_supported = true;
		i915_hdmi_audio_init(hdmi_priv);
	}
	intel_connector->panel.fitting_mode = 0;
}
