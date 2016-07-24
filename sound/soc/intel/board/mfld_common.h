/*
 *  mfld_common.h - Common routines for the Medfield platform
 *  based on Intel Medfield MID platform
 *
 *  Copyright (C) 2012 Intel Corp
 *  Author: Vinod Koul <vinod.koul@intel.com>
 *  Author: Harsha Priya <priya.harsha@intel.com>
 *  Author: Omair Mohammed Abdullah <omair.m.abdullah@intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#ifndef _MFLD_COMMON_H
#define _MFLD_COMMON_H

#include <linux/gpio.h>
/* ADC channel code values */
#define MFLD_AUDIO_DETECT_CODE 0x06
/*Count of AUD_DETECT ADC Registers*/
#define MFLD_AUDIO_SENSOR 1
#define MFLD_THERM_PROBE_SENSOR 1
#define MFLD_ADC_SAMPLE_COUNT 1
/* multipier to convert to mV */
#define MFLD_ADC_ONE_LSB_MULTIPLIER 2346

#define MFLD_JACK_INSERT_ID	  0x04
#define MFLD_LP_THRESHOLD_VOLTAGE 400 /* mV */


/* min/max volatage at ADCIN11 for accessory to be declared as Therm Probe*/
#define MFLD_TP_VOLT_THLD_LOW             50  /* in mV */
#define MFLD_TP_VOLT_THLD_HI              2008  /* in mV */

enum soc_mic_bias_zones {
	MFLD_MV_START = 0,
	/* mic bias volutage range for Headphones*/
	MFLD_MV_HP = 400,
	/* mic bias volutage range for American Headset*/
	MFLD_MV_AM_HS = 650,
	/* mic bias volutage range for Headset*/
	MFLD_MV_HS = 2000,
	MFLD_MV_UNDEFINED,
};

struct mfld_jack_work {
	unsigned int intr_id;
	struct delayed_work work;
	struct snd_soc_jack *jack;
};

struct mfld_therm_probe_data {
	int tp_en_gpio;
	/* Thermal probe status: connected or not*/
	bool tp_status;
	int tp_adc_ch_id;
};

struct mfld_mc_private {
	struct ipc_device *socdev;
	void __iomem *int_base;
	struct snd_soc_jack mfld_jack;
	u8 jack_interrupt_status;
	u8 oc_interrupt_status;
	spinlock_t lock; /* lock for interrupt status and jack debounce */
	struct mfld_jack_work jack_work;
	void *audio_adc_handle;
	int sn95031_pcm1_master_mode;
	unsigned int mfld_jack_lp_flag;
	unsigned int hs_switch;
	unsigned int sn95031_lo_dac;
	struct msic_audio_platform_data *pdata;
	int jack_gpio;
	int media_usage;
	int voice_usage;
	u8 jack_poll_retry;
	long jack_poll_interval;
	atomic_t hs_det_retry;
	unsigned int jack_status;
#ifdef CONFIG_HAS_WAKELOCK
	struct wake_lock *jack_wake_lock;
#endif
	struct mfld_therm_probe_data tp_data;
};


/* sound card controls */
static const char * const headset_switch_text[] = {"Earpiece", "Headset"};

static const char * const lo_text[] = {"Vibra", "Headset", "IHF", "None"};

static const struct soc_enum mfld_headset_enum =
	SOC_ENUM_SINGLE_EXT(2, headset_switch_text);

static const struct soc_enum sn95031_lo_enum =
	SOC_ENUM_SINGLE_EXT(4, lo_text);

static const char * const sn95031_pcm1_mode_text[] = {"Slave", "Master"};

static const struct soc_enum sn95031_pcm1_mode_config_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sn95031_pcm1_mode_text),
					sn95031_pcm1_mode_text);


static inline int mfld_read_jack_gpio(struct mfld_mc_private *ctx)
{
	/* Return if there is no GPIO support */
	if (ctx->jack_gpio < 0)
		return -EINVAL;
	return gpio_get_value(ctx->jack_gpio);
}

unsigned int mfld_jack_read_voltage(struct snd_soc_jack *jack);
int mfld_vibra_enable_clk(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *k, int event);
int mfld_set_vol_2r(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol);
int mfld_set_pcm1_mode(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol);
int mfld_get_pcm1_mode(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol);
int mfld_headset_get_switch(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol);
int mfld_headset_set_switch(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol);
int mfld_lo_get_switch(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol);
int mfld_lo_set_switch(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol);
void mfld_jack_enable_mic_bias_gen(struct snd_soc_codec *codec, char *mic);
void mfld_jack_disable_mic_bias_gen(struct snd_soc_codec *codec, char *mic);

/*
Prototype declarations related to thermal probe accessory follow
*/
void mfld_therm_probe_init(struct mfld_mc_private *ctx, int adc_ch_id);
void mfld_therm_probe_deinit(struct mfld_mc_private *ctx);
bool mfld_therm_probe_on_connect(struct snd_soc_jack *jack);
bool mfld_therm_probe_on_removal(struct snd_soc_jack *jack);
#endif
