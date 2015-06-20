/**************************************************************************
 * Copyright (c) 2014, Intel Corporation.
 * All Rights Reserved.

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
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Author:
 *   Dale B Stimson <dale.b.stimson@intel.com>
 */

#include <linux/device.h>

#include "psb_drv.h"

#include "dsp_sysfs_attrs.h"

#include <drm/drmP.h>

#include "mdfld_dsi_output.h"

/*
 * Note: As sysfs files are part of the ABI, they should be documented under
 * Documentation/ABI.  See Documentation/ABI/README.
 */

#define DSP_SYSFS_ATTRS_GROUP_NAME "display"


struct cabc_namval_s {
	const char *nvnam;
	int         nvval;
};

static const struct cabc_namval_s cabc_nvtab[] = {
	{ "off", CABC_MODE_OFF, },
	{ "ui_image", CABC_MODE_UI_IMAGE, },
	{ "still_image", CABC_MODE_STILL_IMAGE, },
	{ "moving_image", CABC_MODE_MOVING_IMAGE, },
};
static const int cabc_nvcnt = ARRAY_SIZE(cabc_nvtab);


static const char *_cabc_code_to_text(char *sbuf, size_t slen, int cabc_mode)
{
	const char *pstr;
	int i;

	for (i = 0 ; ; i++) {
		if (i >= cabc_nvcnt) {
			snprintf(sbuf, slen, "unknown=0x%x", cabc_mode);
			pstr = sbuf;
		}
		if (cabc_nvtab[i].nvval == cabc_mode) {
			pstr = cabc_nvtab[i].nvnam;
			break;
		}
	}

	return pstr;
}


/*
 * _sysfs_support_fbc_show() - Return 1 if FBC/FBDC supported, else 0.
 * @kdev - Pointer to struct device
 * @attr - pointer to struct device_attribute
 * @buf - Pointer to output buffer to receive character string.
 * The buffer length is PAGE_SIZE bytes.
 */
static ssize_t _sysfs_support_fbc_show(struct device *kdev,
	struct device_attribute *attr, char *buf)
{
	const int buflen = PAGE_SIZE;
	int support_fbc;

	/* Supported for Anniedale, but not for ANN A0 and ANN B0. */
	if (!IS_ANN() || IS_ANN_A0() || IS_ANN_B0())
		support_fbc = 0;
	else
		support_fbc = 1;

	return scnprintf(buf, buflen, "%d\n", support_fbc);
}


/*
 * _sysfs_panel_mode_show() - Return panel mode as "visual", "command",
 * or "unknown"
 * @kdev - Pointer to struct device
 * @attr - pointer to struct device_attribute
 * @buf - Pointer to output buffer to receive character string.
 * The buffer length is PAGE_SIZE bytes.
 */
static ssize_t _sysfs_panel_mode_show(struct device *kdev,
	struct device_attribute *attr, char *buf)
{
	struct drm_minor *minor;
	struct drm_device *dev;
	const int buflen = PAGE_SIZE;
	const char *pms;

	minor = container_of(kdev, struct drm_minor, kdev);
	dev = minor->dev;
	if (!dev)
		return -ENODEV;

	pms = panel_mode_string(dev);

	return scnprintf(buf, buflen, "%s\n", pms);
}


/*
 * _sysfs_cabc_mode_show() - Show cabc mode.
 * @kdev - Pointer to struct device
 * @attr - pointer to struct device_attribute
 * @buf - Pointer to output buffer to receive character string.
 * The buffer length is PAGE_SIZE bytes.
 */
static ssize_t _sysfs_cabc_mode_show(struct device *kdev,
	struct device_attribute *attr, char *buf)
{
	struct drm_minor *minor;
	struct drm_device *dev;
	struct drm_psb_private *dev_priv;
	struct mdfld_dsi_config *dsi_config;
	const int buflen = PAGE_SIZE;
	const char *pstr;
	char sbuf[32];
	int cabc_mode;

	minor = container_of(kdev, struct drm_minor, kdev);
	dev = minor->dev;
	if (!dev)
		return -ENODEV;

	dev_priv = dev->dev_private;
	if (!dev_priv)
		return -ENODEV;

	dsi_config = dev_priv->dsi_configs[0];
	if (!dsi_config)
		return -ENODEV;

	cabc_mode = dsi_config->cabc_mode;

	pstr = _cabc_code_to_text(sbuf, sizeof(sbuf), cabc_mode);

	return scnprintf(buf, buflen, "%s\n", pstr);
}


/*
 * _sysfs_cabc_mode_store() - set cabc mode
 * @kdev - Pointer to struct device
 * @attr - pointer to struct device_attribute
 * @buf - Pointer to input buffer containing character string.
 * @count - Number of characters in buffer.
 * The buffer length is PAGE_SIZE bytes.
 */
static ssize_t _sysfs_cabc_mode_store(struct device *kdev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct drm_minor *minor;
	struct drm_device *dev;
	struct drm_psb_private *dev_priv;
	struct mdfld_dsi_config *dsi_config;
	int ilen;
	int ret;
	int i;
	u8 cabc_mode;

	minor = container_of(kdev, struct drm_minor, kdev);
	dev = minor->dev;
	if (!dev)
		return -ENODEV;

	dev_priv = dev->dev_private;
	if (!dev_priv)
		return -ENODEV;

	dsi_config = dev_priv->dsi_configs[0];
	if (!dsi_config)
		return -ENODEV;

	ilen = count;
	if (ilen < 3)
		return -EINVAL;

	if (buf[ilen-1] == '\n')
		ilen--;
	if (buf[ilen-1] == '\r')
		ilen--;

	for (i = 0 ; ; i++) {
		if (i >= cabc_nvcnt)
			return -EINVAL;
		if (strncmp(buf, cabc_nvtab[i].nvnam, ilen) == 0) {
			cabc_mode = cabc_nvtab[i].nvval;
			break;
		}
	}

	dsi_config->cabc_mode = cabc_mode;

	ret = mdfld_dsi_set_cabc_mode(dev, dsi_config, cabc_mode);
	if (ret < 0)
		return -EINVAL;

	return count;
}


/*  INCLUDE_CABC_TEST - Non-zero to include test file.  Presently, this
    does not work, with the apparent problem an ETIMEDOUT error return from
    __read_panel_data. */
#define INCLUDE_CABC_TEST 0

#if INCLUDE_CABC_TEST
/*
 * _sysfs_cabc_test_show() - Show cabc hw state.
 * Same as _sysfs_cabc_mode_show, except show hw state instead of sw state.
 * @kdev - Pointer to struct device
 * @attr - pointer to struct device_attribute
 * @buf - Pointer to output buffer to receive character string.
 * The buffer length is PAGE_SIZE bytes.
 */
static ssize_t _sysfs_cabc_test_show(struct device *kdev,
	struct device_attribute *attr, char *buf)
{
	struct drm_minor *minor;
	struct drm_device *dev;
	struct drm_psb_private *dev_priv;
	struct mdfld_dsi_config *dsi_config;
	const int buflen = PAGE_SIZE;
	const char *pstr;
	char sbuf[32];
	int cabc_mode;

	minor = container_of(kdev, struct drm_minor, kdev);
	dev = minor->dev;
	if (!dev)
		return -ENODEV;

	dev_priv = dev->dev_private;
	if (!dev_priv)
		return -ENODEV;

	dsi_config = dev_priv->dsi_configs[0];
	if (!dsi_config)
		return -ENODEV;

	cabc_mode = mdfld_dsi_get_cabc_mode(dev, dsi_config);
	if (cabc_mode < 0)
		pstr = "(unavailable)";
	else
		pstr = _cabc_code_to_text(sbuf, sizeof(sbuf), cabc_mode);

	return scnprintf(buf, buflen, "%s\n", pstr);
}
#endif /* if INCLUDE_CABC_TEST */


static DEVICE_ATTR(panel_mode, S_IRUGO, _sysfs_panel_mode_show, NULL);

static DEVICE_ATTR(support_fbc, S_IRUGO, _sysfs_support_fbc_show, NULL);

static DEVICE_ATTR(cabc_mode, S_IRUGO,
	_sysfs_cabc_mode_show, _sysfs_cabc_mode_store);

#if INCLUDE_CABC_TEST
static DEVICE_ATTR(cabc_test, S_IRUGO, _sysfs_cabc_test_show, NULL);
#endif /* if INCLUDE_CABC_TEST */

static struct attribute *dsp_sysfs_attr_list[] = {
	&dev_attr_panel_mode.attr,
	&dev_attr_support_fbc.attr,
	&dev_attr_cabc_mode.attr,
#if INCLUDE_CABC_TEST
	&dev_attr_cabc_test.attr,
#endif /* if INCLUDE_CABC_TEST */
	NULL
};

static struct attribute_group dsp_sysfs_attr_group = {
	.name = DSP_SYSFS_ATTRS_GROUP_NAME,
	.attrs = dsp_sysfs_attr_list
};


int dsp_sysfs_attr_init(struct drm_device *dev)
{
	int ret;

	/* Initialize the sysfs entries*/
	ret = sysfs_create_group(&dev->primary->kdev.kobj,
		&dsp_sysfs_attr_group);
	if (ret) {
		DRM_ERROR("sysfs attribute group creation failed: %s: %d\n",
			DSP_SYSFS_ATTRS_GROUP_NAME, ret);
	}

	return ret;
}


void dsp_sysfs_attr_uninit(struct drm_device *dev)
{
	sysfs_remove_group(&dev->primary->kdev.kobj, &dsp_sysfs_attr_group);
}
