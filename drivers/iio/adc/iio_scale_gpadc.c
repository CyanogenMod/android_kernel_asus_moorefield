/*
 * iio_scale_gpadc.c - Intel Scaling GPADC Driver
 *
 * Copyright (C) 2014 Intel Corporation
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author: Pavan Kumar S <pavan.kumar.s@intel.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/intel_mid_pm.h>
#include <linux/rpmsg.h>

#include <asm/intel_mid_rpmsg.h>
#include <asm/intel_mid_remoteproc.h>
#include <asm/intel_scu_pmic.h>
#include <asm/intel_scale_gpadc.h>

#include <linux/iio/iio.h>
#include <linux/iio/machine.h>
#include <linux/iio/buffer.h>
#include <linux/iio/driver.h>
#include <linux/iio/types.h>
#include <linux/iio/consumer.h>

#define ADC_TO_TEMP 1
#define TEMP_TO_ADC 0

struct gpadc_info {
	struct device *dev;
	int sample_done;
	void __iomem *intr;
	int channel_num;
	struct iio_map *scale_iio_maps;
	struct iio_channel *iio_chans_adc[SCALE_CH_NUM];
	struct channel_thrms_map *scale_chan_map;
	int (*pmic_adc_temp_conv)(int, int *, int);
};

static int find_key(int key, struct temp_lookup *adc_temp_tbl,
		int tbl_row_cnt, int conv)
{
	int left = 0;
	int right = tbl_row_cnt - 1;
	int mid;

	if (conv == ADC_TO_TEMP) {
		while (left <= right) {
			mid = (left + right)/2;
			if (key == adc_temp_tbl[mid].adc_val ||
					(mid < tbl_row_cnt - 1 &&
					 key > adc_temp_tbl[mid].adc_val &&
					 key < adc_temp_tbl[mid+1].adc_val))
				return mid;
			else if (key < adc_temp_tbl[mid].adc_val)
				right = mid - 1;
			else if (key > adc_temp_tbl[mid].adc_val)
				left = mid + 1;
		}
	} else {
		while (left <= right) {
			mid = (left + right)/2;
			if (key == adc_temp_tbl[mid].temp ||
					(mid > 0 &&
					 key > adc_temp_tbl[mid].temp &&
					 key < adc_temp_tbl[mid-1].temp))
				return mid;
			else if (key > adc_temp_tbl[mid].temp)
				right = mid - 1;
			else if (key < adc_temp_tbl[mid].temp)
				left = mid + 1;
		}
	}

	return -EINVAL;
}

static int scale_adc_temp_conv(struct iio_dev *indio_dev, int thrms, int chan,
		int in_val, int *out_val, int conv)
{
	int tbl_row_cnt = 34;
	struct gpadc_info *info = iio_priv(indio_dev);
	int x0, x1, y0, y1;
	int nr, dr;
	int indx, ret = 0;
	int x = in_val;
	struct temp_lookup *adc_temp_tbl = thrms_lookup[thrms].adc_tbl;

	if (chan == PMIC_DIE) {
		if (!info->pmic_adc_temp_conv) {
			dev_err(info->dev,
					"PMIC adc-temp conv unavailable\n");
			return -EINVAL;
		}

		ret = info->pmic_adc_temp_conv(in_val, out_val,
				conv);
		if (ret)
			dev_err(info->dev,
				"Error converting pmic adc-temp:%d, chan:%s\n",
				in_val, info->scale_chan_map[chan].chan_name);

		return ret;
	}

	if (!adc_temp_tbl || !tbl_row_cnt) {
		dev_err(info->dev, "Lookup table not available for chan:%d\n",
				chan);
		return -EINVAL;
	}

	indx = find_key(in_val, adc_temp_tbl, tbl_row_cnt, conv);
	if (indx < 0) {
		dev_err(info->dev,
			"Given value:%x out of range, chan:%d and conv:%d\n",
				in_val, chan, conv);
		return -EINVAL;
	}

	if (conv == ADC_TO_TEMP) {
		if (adc_temp_tbl[indx].adc_val == in_val) {
			*out_val = adc_temp_tbl[indx].temp * 1000;
			return 0;
		}

		x0 = adc_temp_tbl[indx].adc_val;
		x1 = adc_temp_tbl[indx + 1].adc_val;
		y0 = adc_temp_tbl[indx].temp;
		y1 = adc_temp_tbl[indx + 1].temp;

		nr = (x-x0)*y1 + (x1-x)*y0;
		dr =  x1-x0;

		if (!dr)
			return -EINVAL;

		/* Report temperature in milli-degrees */
		*out_val = (nr * 1000)/dr;
	} else {
		if (adc_temp_tbl[indx].temp == in_val) {
			*out_val = adc_temp_tbl[indx].adc_val;
			return 0;
		}

		x0 = adc_temp_tbl[indx].temp;
		x1 = adc_temp_tbl[indx - 1].temp;
		y0 = adc_temp_tbl[indx].adc_val;
		y1 = adc_temp_tbl[indx - 1].adc_val;

		nr = (x-x0)*y1 + (x1-x)*y0;
		dr =  x1-x0;

		if (!dr)
			return -EINVAL;

		*out_val = nr/dr;
	}

	return 0;
}

static ssize_t intel_scale_gpadc_show_tempvals(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int i, ret;
	int adc_val, temp;
	int used = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct gpadc_info *info = iio_priv(indio_dev);

	for (i = 0; i < info->channel_num; i++) {
		adc_val = 0;
		temp = 0;

		ret = iio_read_channel_raw(info->iio_chans_adc[i],
				&adc_val);
		if (ret) {
			dev_err(info->dev,
				"IIO channel read error:%d for ch:%d\n",
				ret, i);
			continue;
		}

		ret = scale_adc_temp_conv(indio_dev,
				info->scale_chan_map[i].thrms, i,
				adc_val, &temp, ADC_TO_TEMP);
		if (ret) {
			dev_err(info->dev,
				"Error getting temp-val:%d for channel:%d\n",
				ret, i);
			continue;
		}

		used += snprintf(buf + used, PAGE_SIZE - used,
			 "temp_val[%s] = %d (ADC:%x)\n",
			 info->scale_iio_maps[i].consumer_channel,
			 temp, adc_val);
	}

	return used;
}

static DEVICE_ATTR(tempvals, S_IRUGO, intel_scale_gpadc_show_tempvals, NULL);

static struct attribute *intel_scale_gpadc_attrs[] = {
	&dev_attr_tempvals.attr,
	NULL,
};

static struct attribute_group intel_scale_gpadc_attr_group = {
	.name = "scale_gpadc",
	.attrs = intel_scale_gpadc_attrs,
};

static int scale_adc_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int *val, int *val2, long m)
{
	int ret, adc_val, temp;
	int ch = chan->channel;
	struct gpadc_info *gp_info = iio_priv(indio_dev);
	int conv;
	struct iio_channel *cur_chan = gp_info->iio_chans_adc[ch];
	int in_val, *out_val;

	dev_dbg(gp_info->dev, "scale_adc_read_raw - ch:%d, datasheet-name:%s, m:%ld\n",
			ch, chan->datasheet_name, m);

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		conv = ADC_TO_TEMP;

		ret = iio_read_channel_raw(cur_chan, &adc_val);
		if (ret) {
			dev_err(gp_info->dev, "IIO channel read error for ch:%d\n",
					ch);
			return ret;
		}

		in_val = adc_val;
		out_val = &temp;
		break;
	case IIO_CHAN_INFO_SCALE:
		conv = TEMP_TO_ADC;
		in_val = *val2;
		out_val = &adc_val;
		break;
	default:
		return -EINVAL;
	}

	ret = scale_adc_temp_conv(indio_dev,
			gp_info->scale_chan_map[ch].thrms, ch,
			in_val, out_val, conv);
	if (ret) {
		dev_err(gp_info->dev,
			"Error converting adc-temp:%d for chan:%s\n",
			ret, chan->datasheet_name);
		return ret;
	}

	*val = (m == IIO_CHAN_INFO_RAW) ? temp : adc_val;

	return ret;
}

static int scale_adc_read_event_value(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			enum iio_event_type type, enum iio_event_direction dir,
			enum iio_event_info info, int *val, int *val2)
{
	struct gpadc_info *gp_info = iio_priv(indio_dev);
	int ch = chan->channel;
	struct iio_channel *cur_chan = gp_info->iio_chans_adc[ch];
	int ret = 0, temp;

	if (cur_chan->indio_dev->info->read_event_value)
		ret = cur_chan->indio_dev->info->read_event_value(
				cur_chan->indio_dev, cur_chan->channel,
				type, dir, info, val, val2);
	if (ret < 0)
		return ret;

	dev_dbg(gp_info->dev, "read_event_value: %x", *val);
	if (info == IIO_EV_INFO_VALUE) {
		ret = scale_adc_temp_conv(indio_dev,
			gp_info->scale_chan_map[ch].thrms, ch,
			*val, &temp, ADC_TO_TEMP);
		if (ret) {
			dev_err(gp_info->dev,
				"Error converting adc-temp:%d for chan:%s\n",
				ret, gp_info->scale_chan_map[ch].chan_name);
			return ret;
		}

		/* Send back the converted value */
		*val = temp;
	}

	dev_dbg(gp_info->dev, "read_event_sent: %d", *val);

	return IIO_VAL_INT;
}

static int scale_adc_write_event_value(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			enum iio_event_type type, enum iio_event_direction dir,
			enum iio_event_info info, int val, int val2)
{
	struct gpadc_info *gp_info = iio_priv(indio_dev);
	int ch = chan->channel;
	struct iio_channel *cur_chan = gp_info->iio_chans_adc[ch];
	int ret = 0, inter = val;

	dev_dbg(gp_info->dev, "write_event_value: %d", val);
	if (info == IIO_EV_INFO_VALUE) {
		ret = scale_adc_temp_conv(indio_dev,
				gp_info->scale_chan_map[ch].thrms, ch,
				val, &inter, TEMP_TO_ADC);
		if (ret) {
			dev_err(gp_info->dev,
					"Error converting adc-temp:%d for"
					" channel:%s\n",
					ret, chan->datasheet_name);
			return ret;
		}
	}

	dev_dbg(gp_info->dev, "write_event_sent: %x", inter);
	if (cur_chan->indio_dev->info->write_event_value)
		ret = cur_chan->indio_dev->info->write_event_value(
				cur_chan->indio_dev, cur_chan->channel,
				type, dir, info, inter, val2);
	if (ret < 0)
		return ret;

	return IIO_VAL_INT;
}

static const struct iio_info scale_adc_info = {
	.read_raw = &scale_adc_read_raw,
	.read_event_value = &scale_adc_read_event_value,
	.write_event_value = &scale_adc_write_event_value,
	.driver_module = THIS_MODULE,
};

static int scale_gpadc_probe(struct platform_device *pdev)
{
	int err, i;
	struct gpadc_info *info;
	struct iio_dev *indio_dev;
	struct intel_scale_gpadc_platform_data *pdata =
			pdev->dev.platform_data;

	if (!pdata) {
		dev_err(&pdev->dev, "no platform data supplied\n");
		err = -EINVAL;
		goto out;
	}

	indio_dev = iio_device_alloc(sizeof(struct gpadc_info));
	if (indio_dev == NULL) {
		dev_err(&pdev->dev, "allocating iio device failed\n");
		err = -ENOMEM;
		goto out;
	}

	info = iio_priv(indio_dev);

	info->dev = &pdev->dev;
	info->channel_num = pdata->channel_num;
	info->scale_iio_maps = pdata->gpadc_iio_maps;
	info->pmic_adc_temp_conv = pdata->pmic_adc_temp_conv;
	info->scale_chan_map = pdata->scale_chan_map;

	platform_set_drvdata(pdev, indio_dev);

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = pdev->name;

	indio_dev->channels = pdata->gpadc_channels;
	indio_dev->num_channels = pdata->channel_num;
	indio_dev->info = &scale_adc_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	err = iio_map_array_register(indio_dev, pdata->gpadc_iio_maps);
	if (err)
		goto err_free;

	err = iio_device_register(indio_dev);
	if (err < 0)
		goto err_array_unregister;

	err = sysfs_create_group(&pdev->dev.kobj,
			&intel_scale_gpadc_attr_group);
	if (err) {
		dev_err(&pdev->dev,
			"Unable to export sysfs interface, error: %d\n",
			err);
		goto err_iio_device_unregister;
	}

	for (i = 0; i < SCALE_CH_NUM; i++) {
		info->iio_chans_adc[i] = iio_channel_get(NULL,
				info->scale_chan_map[i].chan_name);
		if (info->iio_chans_adc[i] == NULL) {
			dev_err(&pdev->dev, "info->iio_chans_adc null for %s\n",
					info->scale_chan_map[i].chan_name);
			err = -EINVAL;
			goto err_sysfs_remove;
		}
	}

	dev_info(&pdev->dev, "scale_adc probed\n");

	return 0;

err_sysfs_remove:
	sysfs_remove_group(&pdev->dev.kobj,
			&intel_scale_gpadc_attr_group);
err_iio_device_unregister:
	iio_device_unregister(indio_dev);
err_array_unregister:
	iio_map_array_unregister(indio_dev);
err_free:
	iio_device_free(indio_dev);
out:
	return err;
}

static int scale_gpadc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj,
			&intel_scale_gpadc_attr_group);

	iio_device_unregister(indio_dev);
	iio_map_array_unregister(indio_dev);
	iio_device_free(indio_dev);

	return 0;
}

#ifdef CONFIG_PM
static int scale_gpadc_suspend(struct device *dev)
{
	return 0;
}

static int scale_gpadc_resume(struct device *dev)
{
	return 0;
}
#else
#define scale_gpadc_suspend		NULL
#define scale_gpadc_resume		NULL
#endif

static const struct dev_pm_ops scale_gpadc_driver_pm_ops = {
	.suspend	= scale_gpadc_suspend,
	.resume		= scale_gpadc_resume,
};

static struct platform_driver scale_gpadc_driver = {
	.driver = {
		   .name = "scale_adc",
		   .owner = THIS_MODULE,
		   .pm = &scale_gpadc_driver_pm_ops,
		   },
	.probe = scale_gpadc_probe,
	.remove = scale_gpadc_remove,
};

static int __init scale_gpadc_module_init(void)
{
	return platform_driver_register(&scale_gpadc_driver);
}

#ifdef MODULE
module_init(scale_gpadc_module_init);
#else
rootfs_initcall(scale_gpadc_module_init);
#endif

static void __exit scale_gpadc_module_exit(void)
{
	platform_driver_unregister(&scale_gpadc_driver);
}
module_exit(scale_gpadc_module_exit);

MODULE_AUTHOR("Pavan Kumar S <pavan.kumar.s@intel.com>");
MODULE_DESCRIPTION("Intel Scaling GPADC Driver");
MODULE_LICENSE("GPL");
