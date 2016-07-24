#ifndef __INTEL_BASINCOVE_GPADC_H__
#define __INTEL_BASINCOVE_GPADC_H__

#define GPADC_VBAT	(1 << 0)
#define GPADC_BATID	(1 << 1)
#define GPADC_IBAT	(1 << 2)
#define GPADC_PMICTEMP	(1 << 3)
#define GPADC_BATTEMP0	(1 << 4)
#define GPADC_BATTEMP1	(1 << 5)
#define GPADC_SYSTEMP0	(1 << 6)
#define GPADC_SYSTEMP1	(1 << 7)
#define GPADC_SYSTEMP2	(1 << 8)
#define GPADC_USBID	(1 << 9)
#define GPADC_PEAK	(1 << 10)
#define GPADC_ADC	(1 << 11)
#define GPADC_VREF	(1 << 12)

#define BCOVE_GPADC_CH_NUM	9
#define SCOVE_GPADC_CH_NUM	12


#ifdef CONFIG_WHISKEY_COVE

#define WCOVE_VENDORID	0x00
#define WCOVE_GPADC_CH_NUM	12
#define MUSBID		(1 << 5)
#define MPEAK		(1 << 6)
#define MBATTEMP	(1 << 3)
#define MSYSTEMP	(1 << 2)
#define MBATT		(1 << 1)
#define MVIBATT		(1 << 0)
#define MGPMEAS		(1 << 4)
#define MCCTICK		(1 << 7)
#define PMIC_SPARE03_ADDR	0x5FC2
#define PMIC_ID_ADDR		0x6E00

#else

#define MUSBID		(1 << 0)
#define MPEAK		(1 << 1)
#define MBATTEMP	(1 << 2)
#define MSYSTEMP	(1 << 3)
#define MBATT		(1 << 4)
#define MVIBATT		(1 << 5)
#define MGPMEAS		(1 << 6)
#define MCCTICK		(1 << 7)

#endif

#define GPADC_RSL(channel, res) (res->data[ffs(channel)-1])

/* Constants defined specific to SC PMIC and WC PMIC*/
#define PMIC_DIE_ADC_MIN	0x35
#define PMIC_DIE_ADC_MAX	0x4C6D
#define PMIC_DIE_TEMP_MIN	-40 /* in C */
#define PMIC_DIE_TEMP_MAX	125
#define ADC_COEFFICIENT		269
#define TEMP_OFFSET		273150 /* in mC */

#define PMIC_VENDOR_ID_MASK     (0x03 << 6)
#define PMIC_MINOR_REV_MASK     0x07
#define PMIC_MAJOR_REV_MASK     (0x07 << 3)
#define BASINCOVE_VENDORID      (0x03 << 6)
#define SHADYCOVE_VENDORID      0x00

#define PMIC_MAJOR_REV_A0       0x00
#define PMIC_MAJOR_REV_B0       (0x01 << 3)
#define PMIC_MINOR_REV_X0       0x00
#define PMIC_MINOR_REV_X1       (0x01 << 0)

#define PMIC_PROVISIONED	(0x01 << 1)
#define PMIC_PROV_MASK		(0x03 << 0)

#ifdef CONFIG_WHISKEY_COVE
#define GPADC_NUM_CHANNELS	WCOVE_GPADC_CH_NUM
#define DRIVERNAME	"whiskey_cove_adc"
#else
#define GPADC_NUM_CHANNELS	SCOVE_GPADC_CH_NUM
#define DRIVERNAME	"bcove_adc"
#define PMIC_ID_ADDR		0x00
/* Register on I2C-dev2-0x6E */
#define PMIC_SPARE03_ADDR	0x010B
#endif

enum gpadc_channel_type {
	PMIC_GPADC_CHANNEL_VBUS = 0,
	PMIC_GPADC_CHANNEL_BATID,
	PMIC_GPADC_CHANNEL_PMICTEMP,
	PMIC_GPADC_CHANNEL_BATTEMP0,
	PMIC_GPADC_CHANNEL_BATTEMP1,
	PMIC_GPADC_CHANNEL_SYSTEMP0,
	PMIC_GPADC_CHANNEL_SYSTEMP1,
	PMIC_GPADC_CHANNEL_SYSTEMP2,
	PMIC_GPADC_CHANNEL_USBID,
	PMIC_GPADC_CHANNEL_PEAK,
	PMIC_GPADC_CHANNEL_AGND,
	PMIC_GPADC_CHANNEL_VREF,
};

struct gpadc_regmap_t {
	char *name;
	int cntl;       /* GPADC Conversion Control Bit indicator */
	int rslth;      /* GPADC Conversion Result Register Addr High */
	int rsltl;      /* GPADC Conversion Result Register Addr Low */
	int alrt_min_h;
	int alrt_min_l;
	int alrt_max_h;
	int alrt_max_l;
};

struct gpadc_regs_t {
	u16 gpadcreq;
	u16 gpadcreq_irqen;
	u16 gpadcreq_busy;
	u16 mirqlvl1;
	u16 mirqlvl1_adc;
	u16 adc1cntl;
	u16 adcirq;
	u16 madcirq;
};

struct iio_dev;

struct intel_basincove_gpadc_platform_data {
	int channel_num;
	unsigned long intr;
	u8 intr_mask;
	struct iio_map *gpadc_iio_maps;
	struct gpadc_regmap_t *gpadc_regmaps;
	struct gpadc_regs_t *gpadc_regs;
	const struct iio_chan_spec *gpadc_channels;
};

struct gpadc_result {
	int data[SCOVE_GPADC_CH_NUM];
};

#ifdef CONFIG_ACPI
static inline bool is_whiskey_cove()
{
	return true;
}
#else
static inline bool is_whiskey_cove()
{
	return false;
}
#endif

extern int shadycove_pmic_adc_temp_conv(int, int *, int);

int iio_basincove_gpadc_sample(struct iio_dev *indio_dev,
				int ch, struct gpadc_result *res);

int intel_basincove_gpadc_sample(int ch, struct gpadc_result *res);
#endif
