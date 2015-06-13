#ifndef __INTEL_CRYSTALCOVE_GPADC_H__
#define __INTEL_CRYSTALCOVE_GPADC_H__

#define GPADC_VBAT		(1 << 0)
#define GPADC_BATID		(1 << 1)
#define GPADC_PMICTEMP		(1 << 2)
#define GPADC_BATTEMP0		(1 << 3)
#define GPADC_BATTEMP1		(1 << 4)
#define GPADC_SYSTEMP0		(1 << 5)
#define GPADC_SYSTEMP1		(1 << 6)
#define GPADC_SYSTEMP2		(1 << 7)
#define GPADC_VCCCUR		(1 << 8)
#define GPADC_VNNCUR		(1 << 9)
#define GPADC_V1P0ACUR		(1 << 10)
#define GPADC_V1P05SCUR		(1 << 11)
#define GPADC_VDDQCUR		(1 << 12)
#define GPADC_CH_NUM	13

#define GPADC_RSL(channel, res) (res->data[ffs(channel)-1])

struct iio_dev;

struct gpadc_result {
	int data[GPADC_CH_NUM];
};

int iio_crystalcove_gpadc_sample(struct iio_dev *indio_dev,
				int ch, struct gpadc_result *res);

int intel_crystalcove_gpadc_sample(int ch, struct gpadc_result *res);
#endif
