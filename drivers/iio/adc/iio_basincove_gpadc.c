/*
 * iio_basincove_gpadc.c - Intel Merrifield Basin Cove GPADC Driver
 *
 * Copyright (C) 2012 Intel Corporation
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
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author: Bin Yang <bin.yang@intel.com>
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
#include <asm/intel_basincove_gpadc.h>

#include <linux/iio/iio.h>
#include <linux/iio/machine.h>
#include <linux/iio/buffer.h>
#include <linux/iio/driver.h>
#include <linux/iio/types.h>
#include <linux/iio/consumer.h>

#define OHM_MULTIPLIER 10
#define ADC_TO_TEMP 1
#define TEMP_TO_ADC 0

struct gpadc_info {
	int initialized;
	/* This mutex protects gpadc sample/config from concurrent conflict.
	   Any function, which does the sample or config, needs to
	   hold this lock.
	   If it is locked, it also means the gpadc is in active mode.
	*/
	struct mutex lock;
	struct device *dev;
	int irq;
	u8 irq_status;
	wait_queue_head_t wait;
	int sample_done;
	void __iomem *intr;
	u8 intr_mask;
	int channel_num;
	struct gpadc_regmap_t *gpadc_regmaps;
	struct gpadc_regs_t *gpadc_regs;
	u8 pmic_id;
	bool is_pmic_provisioned;
};

static const char * const iio_ev_type_text[] = {
	[IIO_EV_TYPE_THRESH] = "thresh",
	[IIO_EV_TYPE_MAG] = "mag",
	[IIO_EV_TYPE_ROC] = "roc",
	[IIO_EV_TYPE_THRESH_ADAPTIVE] = "thresh_adaptive",
	[IIO_EV_TYPE_MAG_ADAPTIVE] = "mag_adaptive",
};

static const char * const iio_ev_dir_text[] = {
	[IIO_EV_DIR_EITHER] = "either",
	[IIO_EV_DIR_RISING] = "rising",
	[IIO_EV_DIR_FALLING] = "falling"
};

static const char * const iio_ev_info_text[] = {
	[IIO_EV_INFO_ENABLE] = "en",
	[IIO_EV_INFO_VALUE] = "value",
	[IIO_EV_INFO_HYSTERESIS] = "hysteresis",
};

static inline int gpadc_clear_bits(u16 addr, u8 mask)
{
	return intel_scu_ipc_update_register(addr, 0, mask);
}

static inline int gpadc_set_bits(u16 addr, u8 mask)
{
	return intel_scu_ipc_update_register(addr, 0xff, mask);
}

static inline int gpadc_write(u16 addr, u8 data)
{
	return intel_scu_ipc_iowrite8(addr, data);
}

static inline int gpadc_read(u16 addr, u8 *data)
{
	return intel_scu_ipc_ioread8(addr, data);
}

int shadycove_pmic_adc_temp_conv(int in_val, int *out_val, int conv)
{
	if (conv == ADC_TO_TEMP) {
		if (in_val < PMIC_DIE_ADC_MIN || in_val > PMIC_DIE_ADC_MAX)
			return -EINVAL;

		*out_val = (ADC_COEFFICIENT * (in_val - 2047)) - TEMP_OFFSET;
	} else {
		if (in_val < PMIC_DIE_TEMP_MIN || in_val > PMIC_DIE_TEMP_MAX)
			return -EINVAL;

		/* 'in_val' is in C, convert to mC as TEMP_OFFSET is in mC */
		*out_val = (((in_val * 1000) + TEMP_OFFSET) / ADC_COEFFICIENT) + 2047;
	}

	return 0;
}

static int gpadc_busy_wait(struct gpadc_regs_t *regs)
{
	u8 tmp;
	int timeout = 0;

	gpadc_read(regs->gpadcreq, &tmp);
	while (tmp & regs->gpadcreq_busy && timeout < 500) {
		gpadc_read(regs->gpadcreq, &tmp);
		usleep_range(1800, 2000);
		timeout++;
	}

	if (tmp & regs->gpadcreq_busy)
		return -EBUSY;
	else
		return 0;
}

static void gpadc_dump(struct gpadc_info *info)
{
	u8 tmp;
	struct gpadc_regs_t *regs = info->gpadc_regs;

	dev_err(info->dev, "GPADC registers dump:\n");
	gpadc_read(regs->adcirq, &tmp);
	dev_err(info->dev, "ADCIRQ: 0x%x\n", tmp);
	gpadc_read(regs->madcirq, &tmp);
	dev_err(info->dev, "MADCIRQ: 0x%x\n", tmp);
	gpadc_read(regs->gpadcreq, &tmp);
	dev_err(info->dev, "GPADCREQ: 0x%x\n", tmp);
	gpadc_read(regs->adc1cntl, &tmp);
	dev_err(info->dev, "ADC1CNTL: 0x%x\n", tmp);
}

static irqreturn_t gpadc_isr(int irq, void *data)
{
#ifdef CONFIG_INTEL_SCU_IPC
	struct gpadc_info *info = iio_priv(data);

	info->irq_status = ioread8(info->intr);
	info->sample_done = 1;
	wake_up(&info->wait);
#endif
	return IRQ_WAKE_THREAD;
}

static irqreturn_t gpadc_threaded_isr(int irq, void *data)
{
	struct gpadc_info *info = iio_priv(data);
	struct gpadc_regs_t *regs = info->gpadc_regs;
#ifndef CONFIG_INTEL_SCU_IPC
	gpadc_read(regs->adcirq, &info->irq_status);
	info->sample_done = 1;
	wake_up(&info->wait);
#else
	/* Clear IRQLVL1MASK */
	gpadc_clear_bits(regs->mirqlvl1, regs->mirqlvl1_adc);
#endif

	return IRQ_HANDLED;
}


/**
 * iio_basincove_gpadc_sample - do gpadc sample.
 * @indio_dev: industrial IO GPADC device handle
 * @ch: gpadc bit set of channels to sample, for example, set ch = (1<<0)|(1<<2)
 *	means you are going to sample both channel 0 and 2 at the same time.
 * @res:gpadc sampling result
 *
 * Returns 0 on success or an error code.
 *
 * This function may sleep.
 */

int iio_basincove_gpadc_sample(struct iio_dev *indio_dev,
				int ch, struct gpadc_result *res)
{
	struct gpadc_info *info = iio_priv(indio_dev);
	int i, ret, reg_val;
	u8 tmp, th, tl;
	u8 mask, cursrc;
	unsigned long rlsb;
	unsigned long rlsb_array[] = {
		0, 260420, 130210, 65100, 32550, 16280,
		8140, 4070, 2030, 0, 260420, 130210};

	struct gpadc_regs_t *regs = info->gpadc_regs;
	bool pmic_a0 = false;

	if (!info->initialized)
		return -ENODEV;

	pmic_a0 = ((info->pmic_id & PMIC_MAJOR_REV_MASK) == PMIC_MAJOR_REV_A0)
		&& ((info->pmic_id & PMIC_MINOR_REV_MASK) == PMIC_MINOR_REV_X0);

	mutex_lock(&info->lock);

	mask = info->intr_mask;
	gpadc_clear_bits(regs->madcirq, mask);
	gpadc_clear_bits(regs->mirqlvl1, regs->mirqlvl1_adc);

	tmp = regs->gpadcreq_irqen;

	for (i = 0; i < info->channel_num; i++) {
		if (ch & (1 << i))
			tmp |= (1 << info->gpadc_regmaps[i].cntl);
	}

	info->sample_done = 0;

	ret = gpadc_busy_wait(regs);
	if (ret) {
		dev_err(info->dev, "GPADC is busy\n");
		goto done;
	}

	gpadc_write(regs->gpadcreq, tmp);

	ret = wait_event_timeout(info->wait, info->sample_done, HZ);
	if (ret == 0) {
		gpadc_dump(info);
		ret = -ETIMEDOUT;
		dev_err(info->dev, "sample timeout, return %d\n", ret);
		goto done;
	} else {
		ret = 0;
	}

	for (i = 0; i < info->channel_num; i++) {
		if (ch & (1 << i)) {
			gpadc_read(info->gpadc_regmaps[i].rsltl, &tl);
			gpadc_read(info->gpadc_regmaps[i].rslth, &th);

			reg_val = ((th & 0xF) << 8) + tl;

			if (((info->pmic_id & PMIC_VENDOR_ID_MASK)
					== SHADYCOVE_VENDORID) ||
					is_whiskey_cove()) {
				switch (i) {
				case PMIC_GPADC_CHANNEL_VBUS:
				case PMIC_GPADC_CHANNEL_PMICTEMP:
				case PMIC_GPADC_CHANNEL_PEAK:
				case PMIC_GPADC_CHANNEL_AGND:
				case PMIC_GPADC_CHANNEL_VREF:
					/* Auto mode not applicable */
					res->data[i] = reg_val;
					break;
				case PMIC_GPADC_CHANNEL_BATID:
				case PMIC_GPADC_CHANNEL_BATTEMP0:
				case PMIC_GPADC_CHANNEL_BATTEMP1:
				case PMIC_GPADC_CHANNEL_SYSTEMP0:
				case PMIC_GPADC_CHANNEL_SYSTEMP1:
				case PMIC_GPADC_CHANNEL_SYSTEMP2:
					if (!is_whiskey_cove() && pmic_a0 &&
						!info->is_pmic_provisioned) {
						/* Auto mode with Scaling 4
						 * for non-provisioned A0 */
						rlsb = 32550;
						res->data[i] =
							(reg_val * rlsb)/10000;
						break;
					}
				/* Case fall-through for PMIC-A1 onwards.
				 * For USBID, Auto-mode-without-scaling always
				 */
				case PMIC_GPADC_CHANNEL_USBID:
					/* Auto mode without Scaling */
					cursrc = (th & 0xF0) >> 4;
					rlsb = rlsb_array[cursrc];
					res->data[i] = (reg_val * rlsb)/10000;
					break;
				}
			} else {
				res->data[i] = reg_val;
			}
		}
	}

done:
	gpadc_set_bits(regs->mirqlvl1, regs->mirqlvl1_adc);
	gpadc_set_bits(regs->madcirq, mask);
	mutex_unlock(&info->lock);
	return ret;
}
EXPORT_SYMBOL(iio_basincove_gpadc_sample);

static struct gpadc_result sample_result;
static int chs;

static ssize_t intel_basincove_gpadc_store_channel(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct gpadc_info *info = iio_priv(indio_dev);

	if (sscanf(buf, "%x", &chs) != 1) {
		dev_err(dev, "one channel argument is needed\n");
		return -EINVAL;
	}

	if (chs < (1 << 0) || chs >= (1 << info->channel_num)) {
		dev_err(dev, "invalid channel, should be in [0x1 - 0x1FF]\n");
		return -EINVAL;
	}

	return size;
}

static ssize_t intel_basincove_gpadc_show_channel(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%x\n", chs);
}

static ssize_t intel_basincove_gpadc_store_sample(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	int value, ret;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);

	memset(sample_result.data, 0, sizeof(sample_result.data));

	if (sscanf(buf, "%d", &value) != 1) {
		dev_err(dev, "one argument is needed\n");
		return -EINVAL;
	}

	if (value == 1) {
		ret = iio_basincove_gpadc_sample(indio_dev, chs,
						&sample_result);
		if (ret) {
			dev_err(dev, "sample failed\n");
			return ret;
		}
	} else {
		dev_err(dev, "input '1' to sample\n");
		return -EINVAL;
	}

	return size;
}

static ssize_t intel_basincove_gpadc_show_result(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int i;
	int used = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct gpadc_info *info = iio_priv(indio_dev);

	for (i = 0; i < info->channel_num; i++) {
		used += snprintf(buf + used, PAGE_SIZE - used,
				"sample_result[%s] = %x\n",
				info->gpadc_regmaps[i].name,
				sample_result.data[i]);
	}

	return used;
}


static DEVICE_ATTR(channel, S_IWUSR | S_IRUGO,
		intel_basincove_gpadc_show_channel,
		intel_basincove_gpadc_store_channel);
static DEVICE_ATTR(sample, S_IWUSR, NULL, intel_basincove_gpadc_store_sample);
static DEVICE_ATTR(result, S_IRUGO, intel_basincove_gpadc_show_result, NULL);

static struct attribute *intel_basincove_gpadc_attrs[] = {
	&dev_attr_channel.attr,
	&dev_attr_sample.attr,
	&dev_attr_result.attr,
	NULL,
};
static struct attribute_group intel_basincove_gpadc_attr_group = {
	.name = "basincove_gpadc",
	.attrs = intel_basincove_gpadc_attrs,
};

static u16 get_scove_tempzone_val(u16 resi_val)
{
	u8 cursel = 0, hys = 0;
	u16 trsh = 0, count = 0, bsr_num = 0;
	u16 tempzone_val = 0;

	/* multiply to convert into Ohm*/
	resi_val *= OHM_MULTIPLIER;

	/* CUR = max(floor(log2(round(ADCNORM/2^5)))-7,0)
	 * TRSH = round(ADCNORM/(2^(4+CUR)))
	 * HYS = if(∂ADCNORM>0 then max(round(∂ADCNORM/(2^(7+CUR))),1) else 0
	 */

	/*
	 * while calculating the CUR[2:0], instead of log2
	 * do a BSR (bit scan reverse) since we are dealing with integer values
	 */
	bsr_num = resi_val;
	bsr_num /= (1 << 5);

	while (bsr_num >>= 1)
		count++;

	cursel = max((count - 7), 0);

	/* calculate the TRSH[8:0] to be programmed */
	trsh = ((resi_val) / (1 << (4 + cursel)));

	tempzone_val = (hys << 12) | (cursel << 9) | trsh;

	return tempzone_val;
}

static int basincove_adc_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int *val, int *val2, long m)
{
	int ret;
	int ch = chan->channel;
	struct gpadc_info *info = iio_priv(indio_dev);
	struct gpadc_result res;

	ret = iio_basincove_gpadc_sample(indio_dev, (1 << ch), &res);
	if (ret) {
		dev_err(info->dev, "sample failed\n");
		return -EINVAL;
	}

	*val = res.data[ch];

	return ret;
}

static int basincove_adc_read_all_raw(struct iio_channel *chan,
					int *val)
{
	int ret;
	int i, num = 0;
	int ch = 0;
	int *channels;
	struct gpadc_info *info = iio_priv(chan->indio_dev);
	struct gpadc_result res;

	while (chan[num].indio_dev)
		num++;

	channels = kzalloc(sizeof(int) * num, GFP_KERNEL);
	if (channels == NULL)
		return -ENOMEM;

	for (i = 0; i < num; i++) {
		channels[i] = chan[i].channel->channel;
		ch |= (1 << channels[i]);
	}

	ret = iio_basincove_gpadc_sample(chan->indio_dev, ch, &res);
	if (ret) {
		dev_err(info->dev, "sample failed\n");
		ret = -EINVAL;
		goto end;
	}

	for (i = 0; i < num; i++)
		val[i] = res.data[channels[i]];

end:
	kfree(channels);
	return ret;
}

static int basincove_adc_read_event_value(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			enum iio_event_type type, enum iio_event_direction dir,
			enum iio_event_info info, int *val, int *val2)
{
	int err;
	int ch = chan->channel;
	u16 reg_h, reg_l;
	u8 val_h, val_l;
	struct gpadc_info *gp_info = iio_priv(indio_dev);

	dev_dbg(gp_info->dev, "ch:%d, adc_read_event: %s-%s-%s\n", ch,
			iio_ev_type_text[type], iio_ev_dir_text[dir],
			iio_ev_info_text[info]);

	if (type == IIO_EV_TYPE_THRESH) {
		switch (dir) {
		case IIO_EV_DIR_RISING:
			reg_h = gp_info->gpadc_regmaps[ch].alrt_max_h;
			reg_l = gp_info->gpadc_regmaps[ch].alrt_max_l;
			break;
		case IIO_EV_DIR_FALLING:
			reg_h = gp_info->gpadc_regmaps[ch].alrt_min_h;
			reg_l = gp_info->gpadc_regmaps[ch].alrt_min_l;
			break;
		default:
			dev_err(gp_info->dev,
				"iio_event_direction %d not supported\n",
				dir);
			return -EINVAL;
		}
	} else {
		dev_err(gp_info->dev,
				"iio_event_type %d not supported\n",
				type);
		return -EINVAL;
	}

	dev_dbg(gp_info->dev, "reg_h:%x, reg_l:%x\n", reg_h, reg_l);

	err = gpadc_read(reg_l, &val_l);
	if (err) {
		dev_err(gp_info->dev, "Error reading register:%X\n", reg_l);
		return -EINVAL;
	}
	err = gpadc_read(reg_h, &val_h);
	if (err) {
		dev_err(gp_info->dev, "Error reading register:%X\n", reg_h);
		return -EINVAL;
	}

	switch (info) {
	case IIO_EV_INFO_VALUE:
		*val = ((val_h & 0xF) << 8) + val_l;
		if (((gp_info->pmic_id & PMIC_VENDOR_ID_MASK)
				== SHADYCOVE_VENDORID) ||
			is_whiskey_cove()) {
			if (ch != PMIC_GPADC_CHANNEL_PMICTEMP) {
				int thrsh = *val & 0x01FF;
				int cur = (*val >> 9) & 0x07;

				*val = thrsh * (1 << (4 + cur))
					/ OHM_MULTIPLIER;
			}
		}

		break;
	case IIO_EV_INFO_HYSTERESIS:
		*val = (val_h >> 4) & 0x0F;
		break;
	default:
		dev_err(gp_info->dev,
				"iio_event_info %d not supported\n",
				info);
		return -EINVAL;
	}

	dev_dbg(gp_info->dev, "read_event_sent: %x\n", *val);
	return IIO_VAL_INT;
}

static int basincove_adc_write_event_value(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			enum iio_event_type type, enum iio_event_direction dir,
			enum iio_event_info info, int val, int val2)
{
	int err;
	int ch = chan->channel;
	u16 reg_h, reg_l;
	u8 val_h, val_l, mask;
	struct gpadc_info *gp_info = iio_priv(indio_dev);

	dev_dbg(gp_info->dev, "ch:%d, adc_write_event: %s-%s-%s\n", ch,
			iio_ev_type_text[type], iio_ev_dir_text[dir],
			iio_ev_info_text[info]);

	dev_dbg(gp_info->dev, "write_event_value: %x\n", val);

	if (type == IIO_EV_TYPE_THRESH) {
		switch (dir) {
		case IIO_EV_DIR_RISING:
			reg_h = gp_info->gpadc_regmaps[ch].alrt_max_h;
			reg_l = gp_info->gpadc_regmaps[ch].alrt_max_l;
			break;
		case IIO_EV_DIR_FALLING:
			reg_h = gp_info->gpadc_regmaps[ch].alrt_min_h;
			reg_l = gp_info->gpadc_regmaps[ch].alrt_min_l;
			break;
		default:
			dev_err(gp_info->dev,
				"iio_event_direction %d not supported\n",
				dir);
			return -EINVAL;
		}
	} else {
		dev_err(gp_info->dev,
				"iio_event_type %d not supported\n",
				type);
		return -EINVAL;
	}

	dev_dbg(gp_info->dev, "reg_h:%x, reg_l:%x\n", reg_h, reg_l);

	switch (info) {
	case IIO_EV_INFO_VALUE:
		if (((gp_info->pmic_id & PMIC_VENDOR_ID_MASK)
				== SHADYCOVE_VENDORID) || is_whiskey_cove()) {
			if (ch != PMIC_GPADC_CHANNEL_PMICTEMP)
				val = get_scove_tempzone_val(val);
		}

		val_h = (val >> 8) & 0xF;
		val_l = val & 0xFF;
		mask = 0x0F;

		err = intel_scu_ipc_update_register(reg_l, val_l, 0xFF);
		if (err) {
			dev_err(gp_info->dev, "Error updating register:%X\n", reg_l);
			return -EINVAL;
		}
		break;
	case IIO_EV_INFO_HYSTERESIS:
		val_h = (val << 4) & 0xF0;
		mask = 0xF0;
		break;
	default:
		dev_err(gp_info->dev,
				"iio_event_info %d not supported\n",
				info);
		return -EINVAL;
	}

	err = intel_scu_ipc_update_register(reg_h, val_h, mask);
	if (err) {
		dev_err(gp_info->dev, "Error updating register:%X\n", reg_h);
		return -EINVAL;
	}

	return IIO_VAL_INT;
}

static const struct iio_info basincove_adc_info = {
	.read_raw = &basincove_adc_read_raw,
	.read_all_raw = &basincove_adc_read_all_raw,
	.read_event_value = &basincove_adc_read_event_value,
	.write_event_value = &basincove_adc_write_event_value,
	.driver_module = THIS_MODULE,
};

static int bcove_gpadc_probe(struct platform_device *pdev)
{
	int err;
	u8 pmic_prov;
	struct gpadc_info *info;
	struct iio_dev *indio_dev;
	struct intel_basincove_gpadc_platform_data *pdata =
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

	mutex_init(&info->lock);
	init_waitqueue_head(&info->wait);
	info->dev = &pdev->dev;
	info->irq = platform_get_irq(pdev, 0);
#ifdef CONFIG_INTEL_SCU_IPC
	info->intr = ioremap_nocache(pdata->intr, 1);
	if (!info->intr) {
		dev_err(&pdev->dev, "ioremap of ADCIRQ failed\n");
		err = -ENOMEM;
		goto err_free;
	}
#endif
	info->intr_mask = pdata->intr_mask;
	info->channel_num = pdata->channel_num;
	info->gpadc_regmaps = pdata->gpadc_regmaps;
	info->gpadc_regs = pdata->gpadc_regs;

	err = request_threaded_irq(info->irq, gpadc_isr, gpadc_threaded_isr,
			IRQF_ONESHOT, "adc", indio_dev);
	if (err) {
		gpadc_dump(info);
		dev_err(&pdev->dev, "unable to register irq %d\n", info->irq);
		goto err_iounmap;
	}

	platform_set_drvdata(pdev, indio_dev);

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = pdev->name;

	indio_dev->channels = pdata->gpadc_channels;
	indio_dev->num_channels = pdata->channel_num;
	indio_dev->info = &basincove_adc_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	err = iio_map_array_register(indio_dev, pdata->gpadc_iio_maps);
	if (err)
		goto err_release_irq;

	err = iio_device_register(indio_dev);
	if (err < 0)
		goto err_array_unregister;

	err = gpadc_read(PMIC_ID_ADDR, &info->pmic_id);
	if (err) {
		dev_err(&pdev->dev, "Error reading PMIC ID register\n");
		goto err_iio_device_unregister;
	}

	dev_info(&pdev->dev, "PMIC-ID: %x\n", info->pmic_id);
	if (((info->pmic_id & PMIC_VENDOR_ID_MASK)
			== SHADYCOVE_VENDORID) || is_whiskey_cove()) {
		/* Check if PMIC is provisioned */
		err = gpadc_read(PMIC_SPARE03_ADDR, &pmic_prov);
		if (err) {
			dev_err(&pdev->dev,
					"Error reading PMIC SPARE03 REG\n");
			goto err_iio_device_unregister;
		}

		if ((pmic_prov & PMIC_PROV_MASK) == PMIC_PROVISIONED) {
			dev_info(&pdev->dev, "ShadyCove PMIC provisioned\n");
			info->is_pmic_provisioned = true;
		} else
			dev_info(info->dev,
					"ShadyCove PMIC not provisioned\n");
	}

	err = sysfs_create_group(&pdev->dev.kobj,
			&intel_basincove_gpadc_attr_group);
	if (err) {
		dev_err(&pdev->dev, "Unable to export sysfs interface, error: %d\n",
			err);
		goto err_iio_device_unregister;
	}

	info->initialized = 1;

	dev_info(&pdev->dev, "bcove adc probed\n");

	return 0;

err_iio_device_unregister:
	iio_device_unregister(indio_dev);
err_array_unregister:
	iio_map_array_unregister(indio_dev);
err_release_irq:
	free_irq(info->irq, info);
err_iounmap:
	iounmap(info->intr);
err_free:
	iio_device_free(indio_dev);
out:
	return err;
}

static int bcove_gpadc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct gpadc_info *info = iio_priv(indio_dev);

	sysfs_remove_group(&pdev->dev.kobj,
			&intel_basincove_gpadc_attr_group);

	iio_device_unregister(indio_dev);
	iio_map_array_unregister(indio_dev);
	free_irq(info->irq, info);
	iounmap(info->intr);
	iio_device_free(indio_dev);

	return 0;
}

#ifdef CONFIG_PM
static int bcove_gpadc_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct gpadc_info *info = iio_priv(indio_dev);

	if (!mutex_trylock(&info->lock))
		return -EBUSY;

	return 0;
}

static int bcove_gpadc_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct gpadc_info *info = iio_priv(indio_dev);

	mutex_unlock(&info->lock);
	return 0;
}
#else
#define bcove_gpadc_suspend		NULL
#define bcove_gpadc_resume		NULL
#endif

static const struct dev_pm_ops bcove_gpadc_driver_pm_ops = {
	.suspend	= bcove_gpadc_suspend,
	.resume		= bcove_gpadc_resume,
};

static struct platform_driver bcove_gpadc_driver = {
	.driver = {
		   .name = DRIVERNAME,
		   .owner = THIS_MODULE,
		   .pm = &bcove_gpadc_driver_pm_ops,
		   },
	.probe = bcove_gpadc_probe,
	.remove = bcove_gpadc_remove,
};

static int bcove_gpadc_module_init(void)
{
	return platform_driver_register(&bcove_gpadc_driver);
}

static void bcove_gpadc_module_exit(void)
{
	platform_driver_unregister(&bcove_gpadc_driver);
}

#ifdef CONFIG_INTEL_SCU_IPC
static int bcove_adc_rpmsg_probe(struct rpmsg_channel *rpdev)
{
	int ret = 0;

	if (rpdev == NULL) {
		pr_err("rpmsg channel not created\n");
		ret = -ENODEV;
		goto out;
	}

	dev_info(&rpdev->dev, "Probed bcove_gpadc rpmsg device\n");

	ret = bcove_gpadc_module_init();

out:
	return ret;
}

static void bcove_adc_rpmsg_remove(struct rpmsg_channel *rpdev)
{
	bcove_gpadc_module_exit();
	dev_info(&rpdev->dev, "Removed bcove_gpadc rpmsg device\n");
}

static void bcove_adc_rpmsg_cb(struct rpmsg_channel *rpdev, void *data,
					int len, void *priv, u32 src)
{
	dev_warn(&rpdev->dev, "unexpected, message\n");

	print_hex_dump(KERN_DEBUG, __func__, DUMP_PREFIX_NONE, 16, 1,
		       data, len,  true);
}

static struct rpmsg_device_id bcove_adc_rpmsg_id_table[] = {
	{ .name	= "rpmsg_bcove_adc" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, bcove_adc_rpmsg_id_table);

static struct rpmsg_driver bcove_adc_rpmsg = {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= bcove_adc_rpmsg_id_table,
	.probe		= bcove_adc_rpmsg_probe,
	.callback	= bcove_adc_rpmsg_cb,
	.remove		= bcove_adc_rpmsg_remove,
};

static int __init bcove_adc_rpmsg_init(void)
{
	return register_rpmsg_driver(&bcove_adc_rpmsg);
}

#ifdef MODULE
module_init(bcove_adc_rpmsg_init);
#else
rootfs_initcall(bcove_adc_rpmsg_init);
#endif

static void __exit bcove_adc_rpmsg_exit(void)
{
	return unregister_rpmsg_driver(&bcove_adc_rpmsg);
}
module_exit(bcove_adc_rpmsg_exit);
#else

#ifdef MODULE
module_init(bcove_gpadc_module_init);
#else
rootfs_initcall(bcove_gpadc_module_init);
#endif

module_exit(bcove_gpadc_module_exit);
#endif


MODULE_AUTHOR("Yang Bin<bin.yang@intel.com>");
MODULE_DESCRIPTION("Intel Merrifield Basin Cove GPADC Driver");
MODULE_LICENSE("GPL");
