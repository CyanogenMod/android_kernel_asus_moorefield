/*
 *  mfld_common.c - Common routines for the Medfield platform
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

#include <linux/delay.h>
#include <linux/string.h>
#include <asm/intel_mid_gpadc.h>
#include <asm/intel_scu_ipcutil.h>
#include <asm/intel-mid.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include "../../codecs/sn95031.h"
#include "mfld_common.h"

/* reads the voltage level from the ADC Driver*/
unsigned int mfld_jack_read_voltage(struct snd_soc_jack *jack)
{
	unsigned int mic_bias;
	struct mfld_mc_private *ctx =
			snd_soc_card_get_drvdata(jack->codec->card);

	/* Reads the mic bias value */
	if (!ctx->mfld_jack_lp_flag)
		/* GPADC MIC BIAS takes around a 50ms to settle down and
		* get sampled porperly, reading earlier than this causes to
		* read incorrect values */
		msleep(50);
	intel_mid_gpadc_sample(ctx->audio_adc_handle,
			MFLD_ADC_SAMPLE_COUNT, &mic_bias);
	mic_bias = (mic_bias * MFLD_ADC_ONE_LSB_MULTIPLIER) / 1000;
	pr_debug("mic bias = %dmV\n", mic_bias);
	return mic_bias;
}

int mfld_vibra_enable_clk(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *k, int event)
{
	int clk_id = 0;

	if (!strcmp(w->name, "Vibra1Clock"))
		clk_id = CLK0_VIBRA1;
	else if (!strcmp(w->name, "Vibra2Clock"))
		clk_id = CLK0_VIBRA2;

	if (SND_SOC_DAPM_EVENT_ON(event))
		intel_scu_ipc_set_osc_clk0(true, clk_id);
	else if (SND_SOC_DAPM_EVENT_OFF(event))
		intel_scu_ipc_set_osc_clk0(false, clk_id);
	return 0;
}

/* Callback to set volume for *VOLCTRL regs. Needs to be implemented separately
 * since clock and VAUDA need to be on before value can be written to the regs
 */
int mfld_set_vol_2r(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int reg = mc->reg;
	unsigned int reg2 = mc->rreg;
	unsigned int shift = mc->shift;
	int max = mc->max;
	unsigned int mask = (1 << fls(max)) - 1;
	unsigned int invert = mc->invert;
	int err;
	unsigned int val, val2, val_mask;
	int sst_pll_mode_saved;

	val_mask = mask << shift;
	val = (ucontrol->value.integer.value[0] & mask);
	val2 = (ucontrol->value.integer.value[1] & mask);

	if (invert) {
		val = max - val;
		val2 = max - val2;
	}

	val = val << shift;
	val2 = val2 << shift;

	pr_debug("enabling PLL and VAUDA to change volume\n");
	mutex_lock(&codec->mutex);
	sst_pll_mode_saved = intel_scu_ipc_set_osc_clk0(true, CLK0_QUERY);
	intel_scu_ipc_set_osc_clk0(true, CLK0_MSIC);
	snd_soc_dapm_force_enable_pin(&codec->dapm, "VirtBias");
	snd_soc_dapm_sync(&codec->dapm);

	err = snd_soc_update_bits(codec, reg, val_mask, val);
	if (err < 0)
		goto restore_state;

	err = snd_soc_update_bits(codec, reg2, val_mask, val2);
restore_state:
	snd_soc_dapm_disable_pin(&codec->dapm, "VirtBias");
	if (!(sst_pll_mode_saved & CLK0_MSIC))
		intel_scu_ipc_set_osc_clk0(false, CLK0_MSIC);
	mutex_unlock(&codec->mutex);
	return err;
}

int mfld_get_pcm1_mode(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);
	struct mfld_mc_private *ctx = snd_soc_card_get_drvdata(codec->card);

	pr_debug("PCM1 master mode: %d\n", ctx->sn95031_pcm1_master_mode);
	ucontrol->value.integer.value[0] = ctx->sn95031_pcm1_master_mode;
	return 0;
}

int mfld_set_pcm1_mode(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);
	struct mfld_mc_private *ctx = snd_soc_card_get_drvdata(codec->card);

	ctx->sn95031_pcm1_master_mode = ucontrol->value.integer.value[0];
	return 0;
}

int mfld_headset_get_switch(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);
	struct mfld_mc_private *ctx = snd_soc_card_get_drvdata(codec->card);
	ucontrol->value.integer.value[0] = ctx->hs_switch;
	return 0;
}

int mfld_headset_set_switch(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);
	struct mfld_mc_private *ctx = snd_soc_card_get_drvdata(codec->card);

	if (ucontrol->value.integer.value[0] == ctx->hs_switch)
		return 0;

	if (ucontrol->value.integer.value[0]) {
		pr_debug("hs_set HS path\n");
		snd_soc_dapm_enable_pin(&codec->dapm, "Headphones");
		snd_soc_dapm_disable_pin(&codec->dapm, "EPOUT");
	} else {
		pr_debug("hs_set EP path\n");
		snd_soc_dapm_disable_pin(&codec->dapm, "Headphones");
		snd_soc_dapm_enable_pin(&codec->dapm, "EPOUT");
	}
	mutex_lock(&codec->mutex);
	snd_soc_dapm_sync(&codec->dapm);
	mutex_unlock(&codec->mutex);
	ctx->hs_switch = ucontrol->value.integer.value[0];

	return 0;
}

static void mfld_lo_enable_out_pins(struct snd_soc_codec *codec)
{
	struct mfld_mc_private *ctx = snd_soc_card_get_drvdata(codec->card);

	snd_soc_dapm_enable_pin(&codec->dapm, "IHFOUTL");
	snd_soc_dapm_enable_pin(&codec->dapm, "IHFOUTR");
	snd_soc_dapm_enable_pin(&codec->dapm, "LINEOUTL");
	snd_soc_dapm_enable_pin(&codec->dapm, "LINEOUTR");
	snd_soc_dapm_enable_pin(&codec->dapm, "VIB1OUT");
	snd_soc_dapm_enable_pin(&codec->dapm, "VIB2OUT");
	if (ctx->hs_switch) {
		snd_soc_dapm_enable_pin(&codec->dapm, "Headphones");
		snd_soc_dapm_disable_pin(&codec->dapm, "EPOUT");
	} else {
		snd_soc_dapm_disable_pin(&codec->dapm, "Headphones");
		snd_soc_dapm_enable_pin(&codec->dapm, "EPOUT");
	}
	mutex_lock(&codec->mutex);
	snd_soc_dapm_sync(&codec->dapm);
	mutex_unlock(&codec->mutex);
}

int mfld_lo_get_switch(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);
	struct mfld_mc_private *ctx = snd_soc_card_get_drvdata(codec->card);
	ucontrol->value.integer.value[0] = ctx->sn95031_lo_dac;
	return 0;
}

int mfld_lo_set_switch(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);
	struct mfld_mc_private *ctx = snd_soc_card_get_drvdata(codec->card);

	if (ucontrol->value.integer.value[0] == ctx->sn95031_lo_dac)
		return 0;

	/* we dont want to work with last state of lineout so just enable all
	 * pins and then disable pins not required
	 */
	mfld_lo_enable_out_pins(codec);
	switch (ucontrol->value.integer.value[0]) {
	case 0:
		pr_debug("set vibra path\n");
		snd_soc_dapm_disable_pin(&codec->dapm, "VIB1OUT");
		snd_soc_dapm_disable_pin(&codec->dapm, "VIB2OUT");
		snd_soc_update_bits(codec, SN95031_LOCTL, 0x66, 0);
		break;

	case 1:
		pr_debug("set hs  path\n");
		snd_soc_dapm_disable_pin(&codec->dapm, "Headphones");
		snd_soc_dapm_disable_pin(&codec->dapm, "EPOUT");
		snd_soc_update_bits(codec, SN95031_LOCTL, 0x66, 0x22);
		break;

	case 2:
		pr_debug("set spkr path\n");
		snd_soc_dapm_disable_pin(&codec->dapm, "IHFOUTL");
		snd_soc_dapm_disable_pin(&codec->dapm, "IHFOUTR");
		snd_soc_update_bits(codec, SN95031_LOCTL, 0x66, 0x44);
		break;

	case 3:
		pr_debug("set null path\n");
		snd_soc_dapm_disable_pin(&codec->dapm, "LINEOUTL");
		snd_soc_dapm_disable_pin(&codec->dapm, "LINEOUTR");
		snd_soc_update_bits(codec, SN95031_LOCTL, 0x66, 0x66);
		break;
	}
	mutex_lock(&codec->mutex);
	snd_soc_dapm_sync(&codec->dapm);
	mutex_unlock(&codec->mutex);
	ctx->sn95031_lo_dac = ucontrol->value.integer.value[0];
	return 0;
}

void mfld_jack_enable_mic_bias_gen(struct snd_soc_codec *codec, char *mic)
{
	pr_debug("enable mic bias : %s\n", mic);
	mutex_lock(&codec->mutex);
	if (!strncmp(mic, "AMIC1Bias", 9))
		snd_soc_dapm_force_enable_pin(&codec->dapm, "AMIC1Bias");
	else if (!strncmp(mic, "AMIC2Bias", 9))
		snd_soc_dapm_force_enable_pin(&codec->dapm, "AMIC2Bias");
	snd_soc_dapm_sync(&codec->dapm);
	mutex_unlock(&codec->mutex);
}

void mfld_jack_disable_mic_bias_gen(struct snd_soc_codec *codec, char *mic)
{

	pr_debug("disable mic bias : %s\n", mic);
	mutex_lock(&codec->mutex);
	if (!strncmp(mic, "AMIC1Bias", 9))
		snd_soc_dapm_disable_pin(&codec->dapm, "AMIC1Bias");
	else if (!strncmp(mic, "AMIC2Bias", 9))
		snd_soc_dapm_disable_pin(&codec->dapm, "AMIC2Bias");
	snd_soc_dapm_sync(&codec->dapm);
	mutex_unlock(&codec->mutex);

}
/*
Code related to thermal probe accessory follows
*/

/* Initialization function for thermal probe; success or failure does not
   impact the rest of the system (except right channel audio if
   tp_en_gpio is invalid).So return type is void
*/
void mfld_therm_probe_init(struct mfld_mc_private *ctx, int adc_ch_id)
{
	int ret_val;
	struct mfld_therm_probe_data  *tp_data_ptr;

	tp_data_ptr = &ctx->tp_data;

	tp_data_ptr->tp_status = false;
	tp_data_ptr->tp_adc_ch_id = adc_ch_id;
	/* Init therm probe enable GPIO */
	tp_data_ptr->tp_en_gpio = get_gpio_by_name("thermal_probe_en");
	if (tp_data_ptr->tp_en_gpio >= 0) {
		pr_info("GPIO for therm probe %d\n", tp_data_ptr->tp_en_gpio);
	} else {
		/* Workaround if the GPIO not defined in SFI table */
		tp_data_ptr->tp_en_gpio = 114;
	}
	ret_val = gpio_request(tp_data_ptr->tp_en_gpio, "thermal_probe_en");
	if (ret_val) {
		pr_err("Therm probe Enable GPIO alloc fail:%d\n", ret_val);
		/* Set gpio to -1 to indicate that gpio request had failed*/
		tp_data_ptr->tp_en_gpio = -1;
		return;
	}

	/* Default Ground HSL and HSR of MSIC. Grounding HSL  is needed for
	   getting PlugDet interrupt in case thermal probe is connected.
	   This does not impact HS insertion interrupt*/
	snd_soc_update_bits(ctx->mfld_jack.codec, SN95031_BTNCTRL2, BIT(1), BIT(1));
	gpio_direction_output(tp_data_ptr->tp_en_gpio, 0);
}

void mfld_therm_probe_deinit(struct mfld_mc_private *ctx)
{
	struct mfld_therm_probe_data  *tp_data_ptr;

	tp_data_ptr = &ctx->tp_data;
	if (tp_data_ptr->tp_en_gpio >= 0)
		gpio_free(tp_data_ptr->tp_en_gpio);
}

/* This function checks if the inserted accessory is thermal probe.
   If yes, keeps the bias and mux approprate for thermal
   probe so that the sensor driver can just read the adc voltage.
   Informs userspace and sensor driver of the insertion event
   and returns true
   If no, keeps the settings for Audio and returns false
*/
bool mfld_therm_probe_on_connect(struct snd_soc_jack *jack)
{
	int voltage = 0;
	int gpio_state;
	bool tp_status = false;
	void *adc_handle =  NULL;
	struct mfld_mc_private *ctx =
		snd_soc_card_get_drvdata(jack->codec->card);
	struct mfld_therm_probe_data  *tp_data_ptr;

	tp_data_ptr = &ctx->tp_data;

	gpio_state = mfld_read_jack_gpio(ctx);
	if (gpio_state != 0) {
		/* This situation can occur during bootup without any accessory
		   connected. We reach here because we initiate accessory
		   detection manually in the init function */
		pr_debug("In therm probe check; but no acc attached;"
			" returning\n");
		return tp_status;
	}
	/*If audio jack already connected;dont proceed with thermal probe det*/
	if (jack->status & (SND_JACK_HEADSET | SND_JACK_HEADPHONE)) {
		pr_debug("Therm probe det: spurious intr: Audio Jack "
					" already connected\n");
		return false;
	}
	/*If thermal probe already connected;dont proceed with det*/
	if (tp_data_ptr->tp_status) {
		pr_debug("Therm probe det: spurious intr:Thermal probe "
					" already connected\n");
		return true;
	}

	if (tp_data_ptr->tp_en_gpio < 0) {
		pr_err("therm probe gpio not valid; returning\n");
		/* If therm probe en gpio is not valid right channel
		   audio will also not work. But atleast enable left
		   channel audio(unground HSL/HSR) and return */
		snd_soc_update_bits(jack->codec, SN95031_BTNCTRL2, BIT(1), 0);
		return tp_status;
	}

	/* GPADC handle for therm probe detection; ADC channel is allocated
	   and deallocated in this fuction so that sensor driver can allocate
	   it and read the ADC after we report thermal probe connection*/
	adc_handle = intel_mid_gpadc_alloc(MFLD_THERM_PROBE_SENSOR,
			tp_data_ptr->tp_adc_ch_id);
	if (!adc_handle) {
		pr_err("therm probe adc handle not valid; returning\n");
		/* If therm probe adc handle is not valid, enable audio
		   and return. UnGround HSL and HSR of MSIC and Mux right
		   ch. headset pin to HSR of MSIC */
		snd_soc_update_bits(jack->codec, SN95031_BTNCTRL2, BIT(1), 0);
		gpio_direction_output(tp_data_ptr->tp_en_gpio, 1);
		return tp_status;
	}


	/* Ground HSL and HSR of MSIC */
	snd_soc_update_bits(jack->codec, SN95031_BTNCTRL2, BIT(1), BIT(1));
	/* Mux right ch. headset pin to ADC*/
	gpio_direction_output(tp_data_ptr->tp_en_gpio, 0);

	/* Enable mic bias 2 for therm probe check */
	mfld_jack_enable_mic_bias_gen(jack->codec, "AMIC2Bias");
	msleep(50);

	/* Read ADIN11 */
	intel_mid_gpadc_sample(adc_handle, 1, &voltage);
	voltage = (voltage * MFLD_ADC_ONE_LSB_MULTIPLIER) / 1000;

	intel_mid_gpadc_free(adc_handle);

	if (voltage < MFLD_TP_VOLT_THLD_LOW ||
			voltage > MFLD_TP_VOLT_THLD_HI) {

		/* Connected accessory is not thermal probe.
		   Enable audio jack detection and audio */
		/* UnGround HSL and HSR of MSIC */
		snd_soc_update_bits(jack->codec, SN95031_BTNCTRL2, BIT(1), 0);
		/* Mux right ch. headset pin to HSR of MSIC */
		gpio_direction_output(tp_data_ptr->tp_en_gpio, 1);
		mfld_jack_disable_mic_bias_gen(jack->codec, "AMIC2Bias");
		tp_status = false;
	} else {

		tp_status = true;
		if (tp_data_ptr->tp_status != tp_status) {
			tp_data_ptr->tp_status = tp_status;
			pr_debug("thermal probe connected\n");
		}
	}
	pr_debug("%s: therm_probe_state = %d voltage = %d\n", __func__,
			tp_status, voltage);
	return tp_status;
}

/* This function checks if the removed accessory is thermal probe.
   If yes, informuserspace and sensor driver of the removal event
   and returns true.
   If no, returns false. In either case resets the bias and mux
   approprate for thermal probe detection for the next insertion.
*/
bool mfld_therm_probe_on_removal(struct snd_soc_jack *jack)
{
	bool ret = false;
	struct mfld_mc_private *ctx =
		snd_soc_card_get_drvdata(jack->codec->card);
	struct mfld_therm_probe_data  *tp_data_ptr;

	tp_data_ptr = &ctx->tp_data;

	/* Default Ground HSL and HSR of MSIC. Grounding HSL  is needed for
	   getting PlugDet interrupt in case thermal probe is connected. */
	snd_soc_update_bits(jack->codec, SN95031_BTNCTRL2, BIT(1), BIT(1));
	mfld_jack_disable_mic_bias_gen(jack->codec, "AMIC2Bias");

	/* If thermal probe was removed, notify userspace */
	if (tp_data_ptr->tp_status == true) {
		tp_data_ptr->tp_status = false;
		pr_debug("thermal probe removed\n");
		ret = true;
	}
	return ret;
}
