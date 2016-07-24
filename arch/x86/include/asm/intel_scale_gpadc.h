#ifndef __INTEL_SCALE_GPADC_H__
#define __INTEL_SCALE_GPADC_H__

#define SCALE_CH_NUM	6

struct iio_dev;

enum scale_thermistor_type {
	NTC_10K = 0,
	NTC_47K,
};

struct channel_thrms_map {
	int chan_num;
	char chan_name[24];
	enum scale_thermistor_type thrms;
};

struct temp_lookup {
	int adc_val;
	int temp;
	int temp_err;
};

struct intel_scale_gpadc_platform_data {
	int channel_num;
	struct iio_map *gpadc_iio_maps;
	struct iio_chan_spec *gpadc_channels;
	struct channel_thrms_map *scale_chan_map;
	int (*pmic_adc_temp_conv)(int, int *, int);
};

struct channel_lookup_map {
	enum scale_thermistor_type thrms;
	struct temp_lookup adc_tbl[34];
};

enum scale_adc_channels { PMIC_DIE, BAT0, BAT1, SYS0, SYS1, SYS2, };

static struct channel_lookup_map thrms_lookup[] = {
	{ NTC_10K,
	{ {0x35, 125, 0}, {0x3C, 120, 0},
	{0x43, 115, 0}, {0x4C, 110, 0},
	{0x56, 105, 0}, {0x61, 100, 0},
	{0x6F, 95, 0}, {0x7F, 90, 0},
	{0x91, 85, 0}, {0xA7, 80, 0},
	{0xC0, 75, 0}, {0xDF, 70, 0},
	{0x103, 65, 0}, {0x12D, 60, 0},
	{0x161, 55, 0}, {0x1A0, 50, 0},
	{0x1EC, 45, 0}, {0x247, 40, 0},
	{0x2B7, 35, 0}, {0x33F, 30, 0},
	{0x3E8, 25, 0}, {0x4B8, 20, 0},
	{0x5BB, 15, 0}, {0x700, 10, 0},
	{0x89A, 5, 0}, {0xAA2, 0, 0},
	{0xD3D, -5, 0}, {0x109B, -10, 0},
	{0x14F5, -15, 0}, {0x1AA7, -20, 0},
	{0x2234, -25, 0}, {0x2C47, -30, 0},
	{0x39E4, -35, 0}, {0x4C6D, -40, 0},
	},
	},
};

#endif
