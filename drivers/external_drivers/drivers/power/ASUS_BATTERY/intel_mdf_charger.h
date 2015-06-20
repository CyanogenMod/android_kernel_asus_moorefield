/*
 * intel_mdf_charger.h - Intel Medfield MSIC Internal charger Driver header file
 *
 * Copyright (C) 2011 Intel Corporation
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
 * Author: Ananth Krishna <ananth.krishna.r@intel.com>,
 *         Ramakrishna Pallala <ramakrishna.pallala@intel.com>
 *         Jenny TC <jenny.tc@intel.com>
 */

#ifndef __INTEL_MDF_CHARGER_H_
#define __INTEL_MDF_CHARGER_H_


/*********************************************************************
 *		Generic defines
 *********************************************************************/

#define MSIC_BATT_PRESENT		1
#define MSIC_BATT_NOT_PRESENT		0
#define MSIC_USB_CHARGER_PRESENT	1
#define MSIC_USB_CHARGER_NOT_PRESENT	0

/* Interrupt registers*/
#define MSIC_BATT_CHR_PWRSRCINT_ADDR	0x005
#define MSIC_BATT_CHR_BATTDET_MASK	(1 << 0)
#define MSIC_BATT_CHR_USBDET_MASK	(1 << 1)
#define MSIC_BATT_CHR_ADPLVDET_MASK	(1 << 3)
#define MSIC_BATT_CHR_ADPHVDET_MASK	(1 << 4)

#define MSIC_BATT_CHR_PWRSRCINT1_ADDR	0x006
#define MSIC_BATT_CHR_USBDCDET_MASK	(1 << 2)
#define MSIC_BATT_CHR_USBCHPDET_MASK	(1 << 6)

#define MSIC_BATT_CHR_CHRINT_ADDR	0x007
#define MSIC_BATT_CHR_BATTOCP_MASK	(1 << 1)
#define MSIC_BATT_CHR_BATTOTP_MASK	(1 << 2)
#define MSIC_BATT_CHR_LOWBATT_MASK	(1 << 3)
#define MSIC_BATT_CHR_WDTIMEEXP_MASK	(1 << 5)
#define MSIC_BATT_CHR_ADPOVP_MASK	(1 << 6)
#define MSIC_BATT_CHR_TIMEEXP_MASK	(1 << 7)

#define MSIC_BATT_CHR_CHRINT1_ADDR	0x008
#define MSIC_BATT_CHR_WKVINDET_MASK	(1 << 2)
#define MSIC_BATT_CHR_CHROTP_MASK	(1 << 3)
#define MSIC_BATT_CHR_VINREGMINT_MASK	(1 << 4)
#define MSIC_BATT_CHR_BATTOVP_MASK	(1 << 5)
#define MSIC_BATT_CHR_USBOVP_MASK	(1 << 6)
#define MSIC_BATT_CHR_CHRCMPLT_MASK	(1 << 7)
#define MSIC_BATT_CHRINT1_EXCP_MASK	0x68

#define MSIC_BATT_RESETIRQ1_ADDR	0x010
#define MSIC_BATT_RESETIRQ2_ADDR	0x011

/* Interrupts need to be masked */
#define CHRINT1_MASK MSIC_BATT_CHR_WKVINDET_MASK
#define CHRINT_MASK (MSIC_BATT_CHR_BATTOTP_MASK |\
		MSIC_BATT_CHR_BATTOCP_MASK)

/* Interrupt Mask registers */
#define MSIC_BATT_CHR_MPWRSRCINT_ADDR	0x014
#define MSIC_MPWRSRCINT_BATTDET		(1 << 0)
#define MSIC_BATT_CHR_MPWRSRCINT1_ADDR	0x015
#define MSIC_BATT_CHR_MCHRINT_ADDR	0x016
#define MSIC_BATT_CHR_MCHRINT1_ADDR	0x017

#define MSIC_BATT_CHR_VBUSDET_ADDR	0x182
#define MSIC_BATT_CHR_VBUSDET_SET_MIN	0x00

/* Low Battery detect register */
#define MSIC_BATT_CHR_LOWBATTDET_ADDR	0x187
#define MSIC_BATT_CHR_SET_LOWBATTREG	0x06

/* Internal charger control registers */
#define MSIC_BATT_CHR_CHRCTRL_ADDR	0x188
#define CHRCNTL_CHRG_LOW_PWR_ENBL	(1 << 1)
#define CHRCNTL_CHRG_DISABLE		(1 << 2)
#define CHRCNTL_CHRITERMEN		(1 << 3)
#define CHRCNTL_VINLMT_100		0x0
#define CHRCNTL_VINLMT_500		(1 << 6)
#define CHRCNTL_VINLMT_950		(2 << 6)
#define CHRCNTL_VINLMT_NOLMT		(3 << 6)

#define MSIC_BATT_CHR_CHRCVOLTAGE_ADDR	0x189

/* Set Charger Voltage to 4200 mV */
#define CHR_CHRVOLTAGE_SET_DEF		4200

#define MSIC_BATT_CHR_CHRCCURRENT_ADDR	0x18A
#define CHRCC_MIN_CURRENT		500

#define MSIC_BATT_CHR_SPCHARGER_ADDR	0x18B
#define CHR_SPCHRGER_LOWCHR_ENABLE	(1 << 5)
#define WEAKVIN_VOLTAGE_LEVEL		4000
#define CHR_SPCHRGER_WEAKVIN_LVL1	0x00
#define CHR_SPCHRGER_WEAKVIN_LVL2	0x07

#define MSIC_BATT_CHR_CHRTTIME_ADDR	0x18C
#define CHR_CHRTIME_SET_13HRS		0x0F

#define MSIC_BATT_CHR_CHRCTRL1_ADDR	0x18D
#define MSIC_EMRG_CHRG_ENBL		(1 << 3)
#define MSIC_EMRG_CHRG_TEMP		(1 << 4)
#define MSIC_CHRG_EXTCHRDIS		(1 << 5)

/* Temperature limit registers */
#define MSIC_BATT_CHR_PWRSRCLMT_ADDR	0x18E	/* Temperature limits */
#define CHR_PWRSRCLMT_SET_RANGE		0xC0

#define CHR_PWRSRCLMT_TMPH_60		(0x03 << 6)
#define CHR_PWRSRCLMT_TMPH_55		(0x02 << 6)
#define CHR_PWRSRCLMT_TMPH_50		(0x01 << 6)
#define CHR_PWRSRCLMT_TMPH_45		(0x00 << 6)

#define CHR_PWRSRCLMT_TMPL_0		(0x00 << 4)
#define CHR_PWRSRCLMT_TMPL_05		(0x01 << 4)
#define CHR_PWRSRCLMT_TMPL_10		(0x02 << 4)
#define CHR_PWRSRCLMT_TMPL_15		(0x03 << 4)

#define MSIC_BATT_CHR_CHRSTWDT_ADDR	0x18F	/* Watchdog timer */
#define CHR_WDT_DISABLE			0x0
#define CHR_WDT_SET_60SEC		(1 << 4)
#define CHR_WDT_SET_120SEC		(2 << 4)
#define CHR_WDT_SET_180SEC		(3 << 4)

#define MSIC_BATT_CHR_WDTWRITE_ADDR	0x190
#define WDTWRITE_UNLOCK_VALUE		0x01

#define MSIC_BATT_CHR_CHRSAFELMT_ADDR	0x191	/* Maximum safe charging
						   voltage and current */
/* Status registers */
#define MSIC_BATT_CHR_SPWRSRCINT_ADDR	0x192
#define MSIC_BATT_CHR_SPWRSRCINT1_ADDR	0x193
#define MSIC_BATT_CHR_USBLOWBATT_MASK	(1 << 0)


/* ADC Channel Numbers */
#define MSIC_BATT_SENSORS	3
#define MSIC_BATT_PACK_TEMP	0x7
#define MSIC_USB_VOLTAGE	0x5
#define MSIC_BATTID		0x2
#define MSIC_ADC_TEMP_IDX	0
#define MSIC_ADC_USB_VOL_IDX	1
#define MSIC_ADC_BATTID_IDX	2

/*MSIC battery temperature  attributes*/
#define MSIC_BTP_ADC_MIN	107
#define MSIC_BTP_ADC_MAX	977


/* SRAM Addresses for INTR and OCV locations */
#define MSIC_SRAM_INTR_ADDR		0xFFFF7FC3

/* Battery Charger Status Register */
#define CHR_STATUS_FAULT_REG	0x37D
#define CHR_STATUS_STAT_ENBL	(1 << 6)
#define CHR_STATUS_TMR_RST	(1 << 7)
#define CHR_STATUS_VOTG_ENBL	(1 << 7)

#define CHR_STATUS_BIT_MASK	0x30
#define CHR_STATUS_BIT_POS	0x04
#define CHR_STATUS_BIT_READY	0x0
#define CHR_STATUS_BIT_PROGRESS	0x1
#define CHR_STATUS_BIT_CYCLE_DONE	0x2
#define CHR_STATUS_BIT_FAULT	0x03

#define CHR_FAULT_BIT_MASK	0x7
#define CHR_FAULT_BIT_NORMAL	0x0
#define CHR_FAULT_BIT_VBUS_OVP	0x1
#define CHR_FAULT_BIT_SLEEP	0x2
#define CHR_FAULT_BIT_LOW_VBUS	0x3
#define CHR_FAULT_BIT_BATT_OVP	0x4
#define CHR_FAULT_BIT_THRM	0x5
#define CHR_FAULT_BIT_TIMER	0x6
#define CHR_FAULT_BIT_NO_BATT	0x7

/* Battery data Offset range in SMIP */
#define BATT_SMIP_BASE_OFFSET		0x314
#define BATT_SMIP_END_OFFSET		0x3F8

/* Battery data Offset range in UMIP */
#define BATT_UMIP_BASE_OFFSET		0x800
#define BATT_UMIP_END_OFFSET		0xBFF
/* UMIP parameter Offsets from UMIP base */
#define UMIP_REV_MAJ_MIN_NUMBER		0x800
#define UMIP_SIZE_IN_BYTES		0x802

#define UMIP_FG_TBL_SIZE		158
#define UMIP_REF_FG_TBL			0x806	/* 2 bytes */

#define UMIP_BATT_FG_TABLE_SIZE		0x9E
#define UMIP_NO_OF_CFG_TBLS_SIZE	0x01
#define UMIP_BATT_FG_TABLE_OFFSET	0x8A4

#define UMIP_NO_OF_CFG_TBLS		UMIP_BATT_FG_TABLE_OFFSET
#define UMIP_BATT_FG_CFG_TBL1 \
	(UMIP_NO_OF_CFG_TBLS + UMIP_NO_OF_CFG_TBLS)
#define UMIP_BATT_FG_CFG_TBL2 \
	(UMIP_BATT_FG_CFG_TBL1 + UMIP_BATT_FG_TABLE_SIZE)
#define UMIP_BATT_FG_CFG_TBL3 \
	(UMIP_BATT_FG_CFG_TBL2 + UMIP_BATT_FG_TABLE_SIZE)
#define UMIP_BATT_FG_CFG_TBL4 \
	(UMIP_BATT_FG_CFG_TBL3 + UMIP_BATT_FG_TABLE_SIZE)
#define UMIP_BATT_FG_CFG_TBL5 \
	(UMIP_BATT_FG_CFG_TBL4 + UMIP_BATT_FG_TABLE_SIZE)

/* UMIP BATT or FG Table definition */
#define BATT_FG_TBL_REV			0	/* 2 bytes */
#define BATT_FG_TBL_NAME		2	/* 4 bytes */
#define BATT_FG_TBL_BATTID		6	/* 8 bytes */
#define BATT_FG_TBL_SIZE		14	/* 2 bytes */
#define BATT_FG_TBL_CHKSUM		16	/* 2 bytes */
#define BATT_FG_TBL_TYPE		18	/* 1 bytes */
#define BATT_FG_TBL_BODY		14	/* 144 bytes */

#define UMIP_READ	0
#define UMIP_WRITE	1
#define SMIP_READ	2
#define MSIC_IPC_READ	0
#define MSIC_IPC_WRITE	1
#define MAX_IPC_ERROR_COUNT 20

#define CHR_WRITE_RETRY_CNT 3
#define CHR_READ_RETRY_CNT 5

/*
 * Each LSB of Charger LED PWM register
 * contributes to 0.39% of duty cycle
 */
#define MSIC_CHRG_LED_PWM_REG	0x194

#define MSIC_CHRG_LED_CNTL_REG	0x195
#define MSIC_CHRG_LED_ENBL		(1 << 0)

/* LED DC Current settings */
#define MSIC_CHRG_LED_DCCUR1	0x0	/* 0.5 mA */
#define MSIC_CHRG_LED_DCCUR2	0x1	/* 1.0 mA */
#define MSIC_CHRG_LED_DCCUR3	0x2	/* 2.5 mA */
#define MSIC_CHRG_LED_DCCUR4	0x3	/* 5.0 mA */

/* Charger LED Frequency settings */
#define MSIC_CHRG_LED_FREQ1		0x0	/* 0.25 Hz */
#define MSIC_CHRG_LED_FREQ2		0x1	/* 0.50 Hz */
#define MSIC_CHRG_LED_FREQ3		0x2	/* 1.00 Hz */
#define MSIC_CHRG_LED_FREQ4		0x3	/* 2.00 Hz */

/*
 * Convert the voltage form decimal to
 * Register writable format
 */
#define CONV_VOL_DEC_MSICREG(a)	(((a - 3500) / 20) << 2)

#define MSIC_BATT_VMIN_THRESHOLD	3600	/* 3600mV */
#define BATT_OVERVOLTAGE_CUTOFF_VOLT	5040	/* 1.2 times of Max voltage */
#define BATT_CRIT_CUTOFF_VOLT		3700	/* 3700 mV */
#define DEFAULT_MAX_CAPACITY		1500

#define COLMB_TO_MAHRS_CONV_FCTR	3600
#define MSIC_VBUS_LOW_VOLTAGE		4200	/* 4200 mV */
#define MSIC_VBUS_OVER_VOLTAGE		6300	/* 6300 mV */

#define MSIC_BATT_TEMP_MAX		60	/* 60 degrees */
#define MSIC_BATT_TEMP_MIN		0
#define MSIC_TEMP_HYST_ERR		4	/* 4 degrees */

#define VBATT_FULL_DET_MARGIN		50	/* 50mV */

/* internal return values */
#define BIT_SET		1
#define BIT_RESET	0

#define TEMP_CHARGE_DELAY_JIFFIES	(HZ * 30)	/*30 sec */
#define CHARGE_STATUS_DELAY_JIFFIES	(HZ * 60)	/*60 sec */

#define IRQ_FIFO_MAX		16
#define THERM_CURVE_MAX_SAMPLES 23
#define THERM_CURVE_MAX_VALUES	4
#define BATT_STRING_MAX		8
#define HYSTR_SAMPLE_MAX	4

#define USER_SET_CHRG_DISABLE	0
#define USER_SET_CHRG_LMT1	1
#define USER_SET_CHRG_LMT2	2
#define USER_SET_CHRG_LMT3	3
#define USER_SET_CHRG_NOLMT	4

#define BATTID_STR_LEN		8
#define MANFCT_STR_LEN		2
#define MODEL_STR_LEN		4
#define SFI_TEMP_NR_RNG		4

#define DISCHRG_CURVE_MAX_SAMPLES 17
#define DISCHRG_CURVE_MAX_COLUMNS 2
#define CC_TIME_TO_LIVE (HZ/8)	/* 125 ms */
#define ADC_TIME_TO_LIVE (HZ/8)	/* 125 ms */

#define ADC_BATTID_24KOHM	108

#define FULL_CURRENT_AVG_LOW	0
#define FULL_CURRENT_AVG_HIGH	50

/*using 1.2*10 to avoid float operations */
#define OVP_VAL_MULT_FACTOR (12)

/* Convert ADC value to VBUS voltage */
#define MSIC_ADC_TO_VBUS_VOL(adc_val)	((6843 * (adc_val)) / 1000)

#define MSIC_CHRG_REG_DUMP_INT		(1 << 0)
#define MSIC_CHRG_REG_DUMP_BOOT		(1 << 1)
#define MSIC_CHRG_REG_DUMP_EVENT	(1 << 2)

/* SMIP FPO1 options field provides different
 * shutdowns methods which should be enabled
 * by the platfrom.
 */
#define FPO1_CAPACITY_SHUTDOWN		(1 << 0)
#define FPO1_VOLTAGE_SHUTDOWN		(1 << 1)
#define FPO1_LOWBATTINT_SHUTDOWN	(1 << 2)

/* Valid msic exception events */
enum msic_event {
	MSIC_EVENT_BATTOCP_EXCPT,
	MSIC_EVENT_BATTOTP_EXCPT,
	MSIC_EVENT_LOWBATT_EXCPT,
	MSIC_EVENT_BATTOVP_EXCPT,
	MSIC_EVENT_ADPOVP_EXCPT,
	MSIC_EVENT_CHROTP_EXCPT,
	MSIC_EVENT_USBOVP_EXCPT,
	MSIC_EVENT_USB_VINREG_EXCPT,
	MSIC_EVENT_WEAKVIN_EXCPT,
	MSIC_EVENT_TIMEEXP_EXCPT,
	MSIC_EVENT_WDTIMEEXP_EXCPT,
};

/* Valid Charging modes */
enum {
	BATT_CHARGING_MODE_NONE = 0,
	BATT_CHARGING_MODE_NORMAL,
	BATT_CHARGING_MODE_MAINTENANCE,
};

/* Battery Thresholds info which need to get from SMIP area */
struct batt_safety_thresholds {
	u8 smip_rev;
	u8 fpo;		/* fixed implementation options */
	u8 fpo1;	/* fixed implementation options1 */
	u8 rsys;	/* System Resistance for Fuel gauging */

	/* Minimum voltage necessary to
	 * be able to safely shut down */
	short int vbatt_sh_min;

	/* Voltage at which the battery driver
	 * should report the LEVEL as CRITICAL */
	short int vbatt_crit;

	short int itc;		/* Charge termination current */
	short int temp_high;	/* Safe Temp Upper Limit */
	short int temp_low;	/* Safe Temp lower Limit */
	u8 brd_id;		/* Unique Board ID */
} __packed;

/*********************************************************************
 *		SFI table entries Structures
 *********************************************************************/

/* Parameters defining the range */
struct temp_mon_table {
	short int temp_up_lim;
	short int temp_low_lim;
	short int rbatt;
	short int full_chrg_vol;
	short int full_chrg_cur;
	short int maint_chrg_vol_ll;
	short int maint_chrg_vol_ul;
	short int maint_chrg_cur;
} __packed;

/* SFI table entries structure.*/
struct msic_batt_sfi_prop {
	char batt_id[BATTID_STR_LEN];
	unsigned short int voltage_max;
	unsigned int capacity;
	u8 battery_type;
	u8 temp_mon_ranges;
	struct temp_mon_table temp_mon_range[SFI_TEMP_NR_RNG];
} __packed;

/*********************************************************************
 *		Battery properties
 *********************************************************************/
struct charge_params {
	short int cvol;
	short int ccur;
	short int vinilmt;
	short int weakvin;
	enum usb_charger_type chrg_type;
};

struct msic_batt_props {
	unsigned int status;
	unsigned int health;
	unsigned int present;
};

struct msic_charg_props {
	unsigned int charger_present;
	unsigned int charger_health;
	unsigned int vbus_vol;
	char charger_model[BATT_STRING_MAX];
	char charger_vender[BATT_STRING_MAX];
};

/*
 * msic battery info
 */
struct msic_power_module_info {

	struct platform_device *pdev;
	bool is_batt_valid;

	/* msic charger data */
	/* lock to protect usb charger properties
	 * locking is applied wherever read or write
	 * operation is being performed to the msic usb
	 * charger property structure.
	 */
	struct mutex usb_chrg_lock;
	struct msic_charg_props usb_chrg_props;
	struct power_supply usb;

	/* msic battery data */
	/* lock to protect battery  properties
	 * locking is applied wherever read or write
	 * operation is being performed to the msic battery
	 * property structure.
	 */
	struct mutex batt_lock;
	struct msic_batt_props batt_props;

	uint16_t adc_index;	/* ADC Channel Index */
	int irq;		/* GPE_ID or IRQ# */

	struct delayed_work connect_handler;
	struct delayed_work disconn_handler;
	struct charge_params ch_params;	/* holds the charge parameters */

	unsigned long update_time;	/* jiffies when data read */

	void __iomem *msic_intr_iomap;

	/* mutex lock to protect driver event related variables
	 * these events can happen to due OTG as well as charger block
	 */
	struct mutex event_lock;
	int batt_event;
	int charging_mode;
	int usr_chrg_enbl;	/* User Charge Enable or Disable */
	int refresh_charger;	/* Refresh charger parameters */

	/* Worker to monitor status and faults */
	struct delayed_work chr_status_monitor;

	/* Worker to handle otg callback events */
	struct delayed_work chrg_callback_dwrk;

	/* lock to avoid concurrent  access to HW Registers.
	 * As some charger control and parameter registers
	 * can be read or write at same time, ipc_rw_lock lock
	 * is used to synchronize those IPC read or write calls.
	 */
	struct mutex ipc_rw_lock;
	struct wake_lock wakelock;
	/* Handle for gpadc requests */
	void *adc_handle;
	/* Lock for ADC reading */
	struct mutex adc_val_lock;
	/* interrupt masks*/
	uint8_t chrint_mask, chrint1_mask;

	int in_cur_lmt;	/* input current limit level */
};
#endif
