/*
 *  byt_bl_rt5642.c - ASoc Machine driver for Intel Baytrail Baylake MID platform
 *
 *  Copyright (C) 2013 Intel Corp
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

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/acpi.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/reboot.h>
#include <asm/intel_soc_pmc.h>
#include <linux/acpi_gpio.h>
#include <asm/intel-mid.h>
#include <linux/mutex.h>
#include <asm/platform_byt_audio.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <linux/input.h>
#include "../../codecs/rt5640.h"

#ifdef CONFIG_SND_SOC_COMMS_SSP
#include "byt_bl_rt5642.h"
#include "../ssp/mid_ssp.h"
#endif /* CONFIG_SND_SOC_COMMS_SSP */

#define BYT_PLAT_CLK_3_HZ	25000000
#define BYT_CODEC_GPIO_IDX      0
#define BYT_JD_GPIO_IDX         1

#define BYT_JD_INTR_DEBOUNCE            0
#define BYT_CODEC_INTR_DEBOUNCE         0
#define BYT_HS_INSERT_DET_DELAY         500
#define BYT_HS_REMOVE_DET_DELAY         500
#define BYT_BUTTON_DET_DELAY            100
#define BYT_HS_DET_POLL_INTRVL          100
#define BYT_BUTTON_EN_DELAY             1500

#define BYT_HS_DET_RETRY_COUNT          6

static struct platform_device *byt_pdev;

static int byt_bl_reboot_callback(struct notifier_block *nfb, unsigned long event, void *data)
{
	pr_info("%s triggered\n", __func__);
	snd_soc_suspend(&byt_pdev->dev);
	return NOTIFY_OK;
}

static struct notifier_block byt_bl_reboot_notifier_block = {
	.notifier_call = byt_bl_reboot_callback,
	.priority = 1,
};

struct byt_mc_private {
#ifdef CONFIG_SND_SOC_COMMS_SSP
	struct byt_comms_mc_private comms_ctl;
#endif /* CONFIG_SND_SOC_COMMS_SSP */
	struct snd_soc_jack jack;
	struct delayed_work hs_insert_work;
	struct delayed_work hs_remove_work;
	struct delayed_work hs_button_work;
	struct mutex jack_mlock;
	/* To enable button press interrupts after a delay after HS detection.
	   This is to avoid spurious button press events during slow HS insertion */
	struct delayed_work hs_button_en_work;
	int hs_insert_det_delay;
	int hs_remove_det_delay;
	int button_det_delay;
	int button_en_delay;
	int hs_det_poll_intrvl;
	int hs_det_retry;
	bool process_button_events;
	int tristate_buffer_gpio;
	int num_jack_gpios;
	bool use_soc_jd_gpio;
};

#ifdef CONFIG_SND_SOC_COMMS_SSP
static inline struct byt_comms_mc_private *kcontrol2ctl(struct snd_kcontrol *kcontrol)
{
	struct snd_soc_card *card =  snd_kcontrol_chip(kcontrol);
	struct byt_mc_private *ctx = snd_soc_card_get_drvdata(card);
	struct byt_comms_mc_private *ctl = &(ctx->comms_ctl);
	return ctl;
}

int byt_get_ssp_bt_sco_master_mode(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{

	struct byt_comms_mc_private *ctl = kcontrol2ctl(kcontrol);
	ucontrol->value.integer.value[0] = ctl->ssp_bt_sco_master_mode;
	return 0;
}

int byt_set_ssp_bt_sco_master_mode(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct byt_comms_mc_private *ctl = kcontrol2ctl(kcontrol);
	if (ucontrol->value.integer.value[0] != ctl->ssp_bt_sco_master_mode)
		ctl->ssp_bt_sco_master_mode = ucontrol->value.integer.value[0];

	return 0;
}

int byt_get_ssp_modem_master_mode(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct byt_comms_mc_private *ctl = kcontrol2ctl(kcontrol);
	ucontrol->value.integer.value[0] = ctl->ssp_modem_master_mode;
	return 0;
}

int byt_set_ssp_modem_master_mode(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct byt_comms_mc_private *ctl = kcontrol2ctl(kcontrol);
	if (ucontrol->value.integer.value[0] != ctl->ssp_modem_master_mode)
		ctl->ssp_modem_master_mode = ucontrol->value.integer.value[0];

	return 0;
}
#endif /* CONFIG_SND_SOC_COMMS_SSP */

static int byt_jack_codec_gpio_intr(void);
static int byt_jack_soc_gpio_intr(void);
static struct snd_soc_jack_gpio hs_gpio[] = {
	[BYT_CODEC_GPIO_IDX] = {
		.name			= "byt-codec-int",
		.report			= SND_JACK_HEADSET |
					  SND_JACK_HEADPHONE |
					  SND_JACK_BTN_0,
		.debounce_time		= BYT_CODEC_INTR_DEBOUNCE,
		.jack_status_check	= byt_jack_codec_gpio_intr,
	},
	[BYT_JD_GPIO_IDX] = {
		.name			= "byt-jd-int",
		.report			= SND_JACK_HEADSET |
					  SND_JACK_HEADPHONE,
		.debounce_time		= BYT_JD_INTR_DEBOUNCE,
		.jack_status_check	= byt_jack_soc_gpio_intr,
	},

};

static inline void byt_force_enable_pin(struct snd_soc_codec *codec,
			 const char *bias_widget, bool enable)
{
	pr_debug("%s %s\n", enable ? "enable" : "disable", bias_widget);
	if (enable)
		snd_soc_dapm_force_enable_pin(&codec->dapm, bias_widget);
	else
		snd_soc_dapm_disable_pin(&codec->dapm, bias_widget);
}

static inline void byt_set_mic_bias_ldo(struct snd_soc_codec *codec, bool enable)
{
	if (enable) {
		byt_force_enable_pin(codec, "micbias1", true);
		byt_force_enable_pin(codec, "LDO2", true);
	} else {
		byt_force_enable_pin(codec, "micbias1", false);
		byt_force_enable_pin(codec, "LDO2", false);
	}
	snd_soc_dapm_sync(&codec->dapm);
}

/*if Soc Jack det is enabled, use it, otherwise use JD via codec */
static inline int byt_check_jd_status(struct byt_mc_private *ctx)
{
	struct snd_soc_jack_gpio *gpio = &hs_gpio[BYT_JD_GPIO_IDX];
	if (ctx->use_soc_jd_gpio)
		return gpio_get_value(gpio->gpio);
	else
		return rt5640_check_jd_status(ctx->jack.codec);
}
/* Identify the jack type as Headset/Headphone/None */
static int byt_check_jack_type(void)
{
	struct snd_soc_jack_gpio *gpio = &hs_gpio[BYT_CODEC_GPIO_IDX];
	struct snd_soc_jack *jack = gpio->jack;
	struct snd_soc_codec *codec = jack->codec;
	int status, jack_type = 0;
	struct byt_mc_private *ctx = container_of(jack, struct byt_mc_private, jack);

	status = byt_check_jd_status(ctx);
	/* jd status low indicates some accessory has been connected */
	if (!status) {
		pr_debug("Jack insert intr");
		/* Do not process button events until accessory is detected as headset*/
		ctx->process_button_events = false;
		byt_set_mic_bias_ldo(codec, true);
		status = rt5640_detect_hs_type(codec, true);
		if (status == RT5640_HEADPHO_DET)
			jack_type = SND_JACK_HEADPHONE;
		else if (status == RT5640_HEADSET_DET) {
			jack_type = SND_JACK_HEADSET;
			ctx->process_button_events = true;
			/* If headset is detected, enable button interrupts after a delay */
			schedule_delayed_work(&ctx->hs_button_en_work,
					msecs_to_jiffies(ctx->button_en_delay));
		} else /* RT5640_NO_JACK */
			jack_type = 0;

		if (jack_type != SND_JACK_HEADSET)
			byt_set_mic_bias_ldo(codec, false);

	} else
		jack_type = 0;

	pr_debug("Jack type detected:%d", jack_type);

	return jack_type;
}

/* Work function invoked by the Jack Infrastructure. Other delayed works
   for jack detection/removal/button press are scheduled from this function */
static int byt_jack_codec_gpio_intr(void)
{
	struct snd_soc_jack_gpio *gpio = &hs_gpio[BYT_CODEC_GPIO_IDX];
	struct snd_soc_jack *jack = gpio->jack;
	int status, jack_type = 0;
	int ret;
	struct byt_mc_private *ctx = container_of(jack, struct byt_mc_private, jack);

	mutex_lock(&ctx->jack_mlock);
	/* Initialize jack status with previous status. The delayed work will confirm
	   the event and send updated status later */
	jack_type = jack->status;
	pr_debug("Enter:%s", __func__);
	if (ctx->use_soc_jd_gpio) {
		/* Must be button event. Confirm the event in delayed work*/
		if (((jack->status & SND_JACK_HEADSET) == SND_JACK_HEADSET) &&
				ctx->process_button_events) {
			ret = schedule_delayed_work(&ctx->hs_button_work,
					msecs_to_jiffies(ctx->button_det_delay));
			if (!ret)
				pr_debug("byt_check_hs_button_status already queued");
			else
				pr_debug("%s:check BP/BR after %d msec",
						__func__, ctx->button_det_delay);
		}
	} else {
		if (!jack->status) {
			ctx->hs_det_retry = BYT_HS_DET_RETRY_COUNT;
			ret = schedule_delayed_work(&ctx->hs_insert_work,
					msecs_to_jiffies(ctx->hs_insert_det_delay));
			if (!ret)
				pr_debug("byt_check_hs_insert_status already queued");
			else
				pr_debug("%s:Check hs insertion  after %d msec",
						__func__, ctx->hs_insert_det_delay);

		} else {
			/* First check for accessory removal; If not removed,
			   check for button events*/
			status = byt_check_jd_status(ctx);
			/* jd status high indicates accessory has been disconnected.
			   However, confirm the removal in the delayed work */
			if (status) {
				/* Do not process button events while we make sure
				   accessory is disconnected*/
				ctx->process_button_events = false;
				ret = schedule_delayed_work(&ctx->hs_remove_work,
						msecs_to_jiffies(ctx->hs_remove_det_delay));
				if (!ret)
					pr_debug("byt_check_hs_remove_status already queued");
				else
					pr_debug("%s:Check hs removal after %d msec",
							__func__, ctx->hs_remove_det_delay);
			} else { /* Must be button event. Confirm the event in delayed work*/
				if (((jack->status & SND_JACK_HEADSET) == SND_JACK_HEADSET) &&
						ctx->process_button_events) {
					ret = schedule_delayed_work(&ctx->hs_button_work,
							msecs_to_jiffies(ctx->button_det_delay));
					if (!ret)
						pr_debug("byt_check_hs_button_status already queued");
					else
						pr_debug("%s:check BP/BR after %d msec",
								__func__, ctx->button_det_delay);
				}
			}
		}
	}

	pr_debug("Exit:%s", __func__);
	mutex_unlock(&ctx->jack_mlock);
	return jack_type;
}

/*Checks jack insertion and identifies the jack type.
  Retries the detection if necessary */
static void byt_check_hs_insert_status(struct work_struct *work)
{
	struct snd_soc_jack_gpio *gpio = &hs_gpio[BYT_CODEC_GPIO_IDX];
	struct snd_soc_jack *jack = gpio->jack;
	struct byt_mc_private *ctx = container_of(work, struct byt_mc_private, hs_insert_work.work);
	int jack_type = 0;

	mutex_lock(&ctx->jack_mlock);
	pr_debug("Enter:%s", __func__);

	jack_type = byt_check_jack_type();

	/* Report jack immediately only if jack is headset. If headphone or no jack was detected,
	   dont report it until the last HS det try. This is to avoid reporting any temporary
	   jack removal or accessory change(eg, HP to HS) during the detection tries.
	   This provides additional debounce that will help in the case of slow insertion.
	   This also avoids the pause in audio due to accessory change from HP to HS */
	if (ctx->hs_det_retry <= 0) /* end of retries; report the status */
		snd_soc_jack_report(jack, jack_type, gpio->report);
	else {
		/* Schedule another detection try if headphone or no jack is detected.
		   During slow insertion of headset, first a headphone may be detected.
		   Hence retry until headset is detected */
		if (jack_type == SND_JACK_HEADSET) {
			ctx->hs_det_retry = 0; /* HS detected, no more retries needed */
			snd_soc_jack_report(jack, jack_type, gpio->report);
		} else {
			ctx->hs_det_retry--;
			schedule_delayed_work(&ctx->hs_insert_work,
					msecs_to_jiffies(ctx->hs_det_poll_intrvl));
			pr_debug("%s:re-try hs detection after %d msec",
					__func__, ctx->hs_det_poll_intrvl);
		}
	}

	pr_debug("Exit:%s", __func__);
	mutex_unlock(&ctx->jack_mlock);
}
/* Checks jack removal. */
static void byt_check_hs_remove_status(struct work_struct *work)
{
	struct snd_soc_jack_gpio *gpio = &hs_gpio[BYT_CODEC_GPIO_IDX];
	struct snd_soc_jack *jack = gpio->jack;
	struct snd_soc_codec *codec = jack->codec;
	struct byt_mc_private *ctx = container_of(work, struct byt_mc_private, hs_remove_work.work);
	int status = 0, jack_type = 0;

	/* Cancel any pending insertion detection. There could be pending insertion detection in the
	   case of very slow insertion or insertion and immediate removal.*/
	cancel_delayed_work_sync(&ctx->hs_insert_work);

	mutex_lock(&ctx->jack_mlock);
	pr_debug("Enter:%s", __func__);
	/* Initialize jack_type with previous status.
	   If the event was an invalid one, we return the preious state*/
	jack_type = jack->status;

	if (jack->status) { /* jack is in connected state; look for removal event */
		status = byt_check_jd_status(ctx);
		if (status) { /* jd status high implies accessory disconnected */
			pr_debug("Jack remove event");
			ctx->process_button_events = false;
			cancel_delayed_work_sync(&ctx->hs_button_en_work);
			status = rt5640_detect_hs_type(codec, false);
			jack_type = 0;
			byt_set_mic_bias_ldo(codec, false);

		} else if (((jack->status & SND_JACK_HEADSET) == SND_JACK_HEADSET) && !ctx->process_button_events) {
			/* Jack is still connected. We may come here if there was a spurious
			   jack removal event. No state change is done until removal is confirmed
			   by the check_jd_status above.i.e. jack status remains Headset or headphone.
			   But as soon as the interrupt thread(byt_jack/_bp_detection) detected a jack
			   removal, button processing gets disabled. Hence re-enable button processing
			   in the case of headset */
			pr_debug(" spurious Jack remove event for headset; re-enable button events");
			ctx->process_button_events = true;
		}
	}
	snd_soc_jack_report(jack, jack_type, gpio->report);
	pr_debug("Exit:%s", __func__);
	mutex_unlock(&ctx->jack_mlock);
}
/* Check for button press/release */
static void byt_check_hs_button_status(struct work_struct *work)
{
	struct snd_soc_jack_gpio *gpio = &hs_gpio[BYT_CODEC_GPIO_IDX];
	struct snd_soc_jack *jack = gpio->jack;
	struct snd_soc_codec *codec = jack->codec;
	struct byt_mc_private *ctx = container_of(work, struct byt_mc_private, hs_button_work.work);
	int status = 0, jack_type = 0;
	int ret;

	mutex_lock(&ctx->jack_mlock);
	pr_debug("Enter:%s", __func__);
	/* Initialize jack_type with previous status.
	   If the event was an invalid one, we return the preious state*/
	jack_type = jack->status;

	if (((jack->status & SND_JACK_HEADSET) == SND_JACK_HEADSET)
			&& ctx->process_button_events) {

		status = byt_check_jd_status(ctx);
		if (!status) { /* confirm jack is connected */

			status = rt5640_check_bp_status(codec);
			if (jack->status & SND_JACK_BTN_0) { /* if button was previosly in pressed state*/
				if (!status) {
					pr_debug("BR event received");
					jack_type = SND_JACK_HEADSET;
				}
			} else { /* If button was previously in released state */
				if (status) {
					pr_debug("BP event received");
					jack_type = SND_JACK_HEADSET | SND_JACK_BTN_0;
				}
			}
		}
		/* There could be button interrupts during jack removal. There can be
		   situations where a button interrupt is generated first but no jack
		   removal interrupt is generated. This can happen on platforrms where
		   jack detection is aligned to Headset Left pin instead of the ground
		   pin and codec multiplexes (ORs) the jack and button interrupts.
		   So schedule a jack removal detection work */
		if (!ctx->use_soc_jd_gpio) {
			ret = schedule_delayed_work(&ctx->hs_remove_work,
					msecs_to_jiffies(ctx->hs_remove_det_delay));
			if (!ret)
				pr_debug("byt_check_hs_remove_status already queued");
			else
				pr_debug("%s:Check hs removal after %d msec",
						__func__, ctx->hs_remove_det_delay);
		}

	}
	snd_soc_jack_report(jack, jack_type, gpio->report);
	pr_debug("Exit:%s", __func__);
	mutex_unlock(&ctx->jack_mlock);
}

static int byt_jack_soc_gpio_intr(void)
{
	struct snd_soc_jack_gpio *gpio = &hs_gpio[BYT_JD_GPIO_IDX];
	struct snd_soc_jack *jack = gpio->jack;
	struct byt_mc_private *ctx = container_of(jack, struct byt_mc_private, jack);
	int ret;
	int status;

	mutex_lock(&ctx->jack_mlock);

	pr_debug("Enter:%s", __func__);

	if (!jack->status) {
		ctx->hs_det_retry = BYT_HS_DET_RETRY_COUNT;
		ret = schedule_delayed_work(&ctx->hs_insert_work,
				msecs_to_jiffies(ctx->hs_insert_det_delay));
		if (!ret)
			pr_debug("byt_check_hs_insert_status already queued");
		else
			pr_debug("%s:Check hs insertion  after %d msec",
					__func__, ctx->hs_insert_det_delay);

	} else {
		status = byt_check_jd_status(ctx);
		/* jd status high indicates accessory has been disconnected.
		   However, confirm the removal in the delayed work */
		if (status) {
			/* Do not process button events while we make sure
			   accessory is disconnected*/
			ctx->process_button_events = false;
			ret = schedule_delayed_work(&ctx->hs_remove_work,
					msecs_to_jiffies(ctx->hs_remove_det_delay));
			if (!ret)
				pr_debug("byt_check_hs_remove_status already queued");
			else
				pr_debug("%s:Check hs removal after %d msec",
						__func__, ctx->hs_remove_det_delay);
		}
	}
	mutex_unlock(&ctx->jack_mlock);
	pr_debug("Exit:%s", __func__);
	/* return previous status */
	return jack->status;

}
/* Delayed work for enabling the overcurrent detection circuit and interrupt
   for generating button events */
static void byt_enable_hs_button_events(struct work_struct *work)
{
	struct snd_soc_jack_gpio *gpio = &hs_gpio[BYT_CODEC_GPIO_IDX];
	struct snd_soc_jack *jack = gpio->jack;
	struct snd_soc_codec *codec = jack->codec;

	rt5640_enable_ovcd_interrupt(codec, true);
}

static inline struct snd_soc_codec *byt_get_codec(struct snd_soc_card *card)
{
	bool found = false;
	struct snd_soc_codec *codec;

	list_for_each_entry(codec, &card->codec_dev_list, card_list) {
		if (!strstr(codec->name, "rt5640.2-001c")) {
			pr_debug("codec was %s", codec->name);
			continue;
		} else {
			found = true;
			break;
		}
	}
	if (found == false) {
		pr_err("%s: cant find codec", __func__);
		return NULL;
	}
	return codec;
}

#define VLV2_PLAT_CLK_AUDIO	3
#define PLAT_CLK_FORCE_ON	1
#define PLAT_CLK_FORCE_OFF	2
static int platform_clock_control(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *k, int  event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct snd_soc_codec *codec;

	codec = byt_get_codec(card);
	if (!codec) {
		pr_err("Codec not found; Unable to set platform clock\n");
		return -EIO;
	}
	if (SND_SOC_DAPM_EVENT_ON(event)) {
		pmc_pc_configure(VLV2_PLAT_CLK_AUDIO,
				PLAT_CLK_FORCE_ON);
		pr_debug("Platform clk turned ON\n");
		snd_soc_codec_set_sysclk(codec, RT5640_SCLK_S_PLL1,
				0, BYT_PLAT_CLK_3_HZ, SND_SOC_CLOCK_IN);
		snd_soc_write(codec, RT5640_ADDA_CLK1, 0x0014);
	} else {
		/* Set codec clock source to internal clock before
		   turning off the platform clock. Codec needs clock
		   for Jack detection and button press. Also scale down
		   ADC/DAC clock */
		snd_soc_write(codec, RT5640_ADDA_CLK1, 0x7774);
		snd_soc_codec_set_sysclk(codec, RT5640_SCLK_S_RCCLK,
				0, 0, SND_SOC_CLOCK_IN);
		pmc_pc_configure(VLV2_PLAT_CLK_AUDIO,
				PLAT_CLK_FORCE_OFF);
		pr_debug("Platform clk turned OFF\n");
	}

	return 0;
}

static const struct snd_soc_dapm_widget byt_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Int Mic", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
	SND_SOC_DAPM_SUPPLY("Platform Clock", SND_SOC_NOPM, 0, 0,
			platform_clock_control, SND_SOC_DAPM_PRE_PMU|
			SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_route byt_audio_map[] = {
	{"IN2P", NULL, "Headset Mic"},
	{"IN2N", NULL, "Headset Mic"},
	{"DMIC1", NULL, "Int Mic"},
	{"Headphone", NULL, "HPOL"},
	{"Headphone", NULL, "HPOR"},
	{"Ext Spk", NULL, "SPOLP"},
	{"Ext Spk", NULL, "SPOLN"},
	{"Ext Spk", NULL, "SPORP"},
	{"Ext Spk", NULL, "SPORN"},

	{"Headphone", NULL, "Platform Clock"},
	{"Headset Mic", NULL, "Platform Clock"},
	{"Int Mic", NULL, "Platform Clock"},
	{"Ext Spk", NULL, "Platform Clock"},
};

static const struct snd_kcontrol_new byt_mc_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphone"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
	SOC_DAPM_PIN_SWITCH("Int Mic"),
	SOC_DAPM_PIN_SWITCH("Ext Spk"),
};

/* Sets dai format and pll */
static int byt_set_dai_fmt_pll(struct snd_soc_dai *codec_dai, int unsigned fmt,
				int source, unsigned int freq_out)
{
	int ret;
	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0) {
		pr_err("can't set codec DAI configuration %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_pll(codec_dai, 0, source,
				  BYT_PLAT_CLK_3_HZ, freq_out * 512);
	if (ret < 0) {
		pr_err("can't set codec pll: %d\n", ret);
		return ret;
	}
	return 0;
}
static int byt_aif1_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int fmt;

	pr_debug("Enter:%s", __func__);
	/* I2S Slave Mode*/
	fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
	      SND_SOC_DAIFMT_CBS_CFS;
	return byt_set_dai_fmt_pll(codec_dai, fmt, RT5640_PLL1_S_MCLK,
						params_rate(params));

}

static int byt_aif2_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct byt_mc_private *ctx = snd_soc_card_get_drvdata(rtd->card);
	unsigned int fmt;
	int ret;

	pr_debug("Enter:%s", __func__);
	/* I2S  Slave Mode*/
	fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
	      SND_SOC_DAIFMT_CBS_CFS;
	ret = byt_set_dai_fmt_pll(codec_dai, fmt, RT5640_PLL1_S_MCLK,
						params_rate(params));
	if (ret < 0) {
		pr_err("can't set codec dai fmt/pll: %d\n", ret);
		return ret;
	}

	if (ctx->tristate_buffer_gpio >= 0)
		gpio_set_value(ctx->tristate_buffer_gpio, 1);

	return 0;
}

static int byt_aif2_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct byt_mc_private *ctx = snd_soc_card_get_drvdata(rtd->card);

	if (ctx->tristate_buffer_gpio >= 0)
		gpio_set_value(ctx->tristate_buffer_gpio, 0);
	return 0;
}

static int byt_compr_set_params(struct snd_compr_stream *cstream)
{
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int fmt;

	pr_debug("Enter:%s", __func__);
	/* I2S  Slave Mode*/
	fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
	      SND_SOC_DAIFMT_CBS_CFS;
	return byt_set_dai_fmt_pll(codec_dai, fmt, RT5640_PLL1_S_MCLK, 48000);
}

static int byt_set_bias_level(struct snd_soc_card *card,
				struct snd_soc_dapm_context *dapm,
				enum snd_soc_bias_level level)
{
	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
	case SND_SOC_BIAS_OFF:
		break;
	default:
		pr_err("%s: Invalid bias level=%d\n", __func__, level);
		return -EINVAL;
	}
	card->dapm.bias_level = level;
	pr_debug("card(%s)->bias_level %u\n", card->name,
			card->dapm.bias_level);
	return 0;
}

#ifdef CONFIG_SND_SOC_COMMS_SSP
static int byt_comms_dai_link_startup(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *str_runtime;

	str_runtime = substream->runtime;

	WARN(!substream->pcm, "BYT Comms Machine: ERROR NULL substream->pcm\n");

	if (!substream->pcm)
		return -EINVAL;

    /* set the runtime hw parameter with local snd_pcm_hardware struct */
	switch (substream->pcm->device) {
	case BYT_COMMS_BT:
		str_runtime->hw = BYT_COMMS_BT_hw_param;
		break;

	case BYT_COMMS_MODEM:
		str_runtime->hw = BYT_COMMS_MODEM_hw_param;
		break;
	default:
		pr_err("BYT Comms Machine: bad PCM Device = %d\n",
		       substream->pcm->device);
	}
	return snd_pcm_hw_constraint_integer(str_runtime,
					 SNDRV_PCM_HW_PARAM_PERIODS);
}

static int byt_comms_dai_link_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card *soc_card = rtd->card;
	struct byt_mc_private *ctx = snd_soc_card_get_drvdata(soc_card);
	struct byt_comms_mc_private *ctl = &(ctx->comms_ctl);

	int ret = 0;
	unsigned int tx_mask, rx_mask;
	unsigned int nb_slot = 0;
	unsigned int slot_width = 0;
	unsigned int tristate_offset = 0;
	unsigned int device = substream->pcm->device;


	pr_debug("ssp_bt_sco_master_mode %d\n", ctl->ssp_bt_sco_master_mode);
	pr_debug("ssp_modem_master_mode %d\n", ctl->ssp_modem_master_mode);

	switch (device) {
	case BYT_COMMS_BT:
		/*
		 * set cpu DAI configuration
		 * frame_format = PSP_FORMAT
		 * ssp_serial_clk_mode = SSP_CLK_MODE_1
		 * ssp_frmsync_pol_bit = SSP_FRMS_ACTIVE_HIGH
		 */
		ret = snd_soc_dai_set_fmt(cpu_dai,
					  SND_SOC_DAIFMT_I2S |
					  SSP_DAI_SCMODE_1 |
					  SND_SOC_DAIFMT_NB_NF |
					  (ctl->ssp_bt_sco_master_mode ?
					   SND_SOC_DAIFMT_CBM_CFM :
					   SND_SOC_DAIFMT_CBS_CFS));

		if (ret < 0) {
			pr_err("BYT Comms Machine: Set FMT Fails %d\n",
				ret);
			return -EINVAL;
		}

		/*
		 * BT SCO SSP Config
		 * ssp_active_tx_slots_map = 0x01
		 * ssp_active_rx_slots_map = 0x01
		 * frame_rate_divider_control = 1
		 * data_size = 16
		 * tristate = 1
		 * ssp_frmsync_timing_bit = 0
		 * (NEXT_FRMS_ASS_AFTER_END_OF_T4)
		 * ssp_frmsync_timing_bit = 1
		 * (NEXT_FRMS_ASS_WITH_LSB_PREVIOUS_FRM)
		 * ssp_psp_T2 = 1
		 * (Dummy start offset = 1 bit clock period)
		 */
		nb_slot = BYT_SSP_BT_SLOT_NB_SLOT;
		slot_width = BYT_SSP_BT_SLOT_WIDTH;
		tx_mask = BYT_SSP_BT_SLOT_TX_MASK;
		rx_mask = BYT_SSP_BT_SLOT_RX_MASK;

		if (ctl->ssp_bt_sco_master_mode)
			tristate_offset = BIT(TRISTATE_BIT);
		else
			tristate_offset = BIT(FRAME_SYNC_RELATIVE_TIMING_BIT);
		break;

	case BYT_COMMS_MODEM:
		/*
		 * set cpu DAI configuration
		 * frame_format = PSP_FORMAT
		 * ssp_serial_clk_mode = SSP_CLK_MODE_0
		 * ssp_frmsync_pol_bit = SSP_FRMS_ACTIVE_HIGH
		 */
		ret = snd_soc_dai_set_fmt(cpu_dai,
						SND_SOC_DAIFMT_I2S |
						SSP_DAI_SCMODE_0 |
						SND_SOC_DAIFMT_NB_NF |
						(ctl->ssp_modem_master_mode ?
						SND_SOC_DAIFMT_CBM_CFM :
						SND_SOC_DAIFMT_CBS_CFS));
		if (ret < 0) {
			pr_err("BYT Comms Machine:  Set FMT Fails %d\n", ret);
			return -EINVAL;
		}

		/*
		 * Modem Mixing SSP Config
		 * ssp_active_tx_slots_map = 0x01
		 * ssp_active_rx_slots_map = 0x01
		 * frame_rate_divider_control = 1
		 * data_size = 32
		 * Master:
		 *	tristate = 3
		 *	ssp_frmsync_timing_bit = 1, for MASTER
		 *	(NEXT_FRMS_ASS_WITH_LSB_PREVIOUS_FRM)
		 * Slave:
		 *	tristate = 1
		 *	ssp_frmsync_timing_bit = 0, for SLAVE
		 *	(NEXT_FRMS_ASS_AFTER_END_OF_T4)
		 *
		 */
		nb_slot = BYT_SSP_MIXING_SLOT_NB_SLOT;
		slot_width = BYT_SSP_MIXING_SLOT_WIDTH;
		tx_mask = BYT_SSP_MIXING_SLOT_TX_MASK;
		rx_mask = BYT_SSP_MIXING_SLOT_RX_MASK;

		tristate_offset = BIT(TRISTATE_BIT) |
		    BIT(FRAME_SYNC_RELATIVE_TIMING_BIT);

		break;
	default:
		pr_err("BYT Comms Machine: bad PCM Device ID = %d\n", device);
		return -EINVAL;
	}

	ret = snd_soc_dai_set_tdm_slot(cpu_dai, tx_mask,
				   rx_mask, nb_slot, slot_width);

	if (ret < 0) {
		pr_err("BYT Comms Machine:  Set TDM Slot Fails %d\n", ret);
		return -EINVAL;
	}

	ret = snd_soc_dai_set_tristate(cpu_dai, tristate_offset);
	if (ret < 0) {
		pr_err("BYT Comms Machine: Set Tristate Fails %d\n", ret);
		return -EINVAL;
	}

	pr_debug("BYT Comms Machine: slot_width = %d\n",
	     slot_width);
	pr_debug("BYT Comms Machine: tx_mask = %d\n",
	     tx_mask);
	pr_debug("BYT Comms Machine: rx_mask = %d\n",
	     rx_mask);
	pr_debug("BYT Comms Machine: tristate_offset = %d\n",
	     tristate_offset);

	return 0;
}

static int byt_comms_dai_link_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct byt_mc_private *ctx = snd_soc_card_get_drvdata(rtd->card);
	struct byt_comms_mc_private *ctl = &(ctx->comms_ctl);

	unsigned int device = substream->pcm->device;

	pr_debug("%s substream->runtime->rate %d\n",
		__func__,
		substream->runtime->rate);

	/* select clock source (if master) */
	/* BT SCO: CPU DAI is master */
	/* FM: CPU DAI is master */
	/* BT_VOIP: CPU DAI is master */
	if ((device == BYT_COMMS_BT && ctl->ssp_bt_sco_master_mode) ||
	    (device == BYT_COMMS_MODEM && ctl->ssp_modem_master_mode)) {
		snd_soc_dai_set_sysclk(cpu_dai, SSP_CLK_ONCHIP,
				substream->runtime->rate, 0);
	}

	return 0;
}
#endif  /* CONFIG_SND_SOC_COMMS_SSP */

static int byt_init(struct snd_soc_pcm_runtime *runtime)
{
	int ret;
	struct snd_soc_codec *codec = runtime->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct snd_soc_card *card = runtime->card;
	struct byt_mc_private *ctx = snd_soc_card_get_drvdata(runtime->card);

	pr_debug("Enter:%s", __func__);
	/* Set codec bias level */
	byt_set_bias_level(card, dapm, SND_SOC_BIAS_OFF);
	card->dapm.idle_bias_off = true;
	/* Set overcurrent detection threshold base and scale factor
	   for jack type identification and button events. */

	if (INTEL_MID_BOARD(3, TABLET, BYT, BLK, PRO, 8PR1))
		/* The mic bias resistor in BYT FFRD8 PR1 is reduced from
		2.1K to 1.5K. Therefore the correct over current threshold
		for this bias resistance is 1500uA. */
		rt5640_config_ovcd_thld(codec, RT5640_MIC1_OVTH_1500UA,
					RT5640_MIC_OVCD_SF_1P0);
	else
		/* Threshold base = 2000uA; scale factor = 0.5 =>
		effective threshold of 1000uA */
		rt5640_config_ovcd_thld(codec, RT5640_MIC1_OVTH_2000UA,
					RT5640_MIC_OVCD_SF_0P5);

	/* FFRD8 uses codec's JD1 for jack detection */
	if (INTEL_MID_BOARD(3, TABLET, BYT, BLK, PRO, 8PR0) ||
	    INTEL_MID_BOARD(3, TABLET, BYT, BLK, PRO, 8PR1)) {
		/* If SoC jack detect GPIO is used, disable JD functionality
		   via codec. Also disable JD interrupt.*/
		if (ctx->use_soc_jd_gpio) {
			snd_soc_update_bits(codec, RT5640_IRQ_CTRL1,
					RT5640_IRQ_JD_MASK, RT5640_IRQ_JD_BP);
			snd_soc_update_bits(codec, RT5640_JD_CTRL,
					RT5640_JD_MASK, RT5640_JD_DIS);
		} else
			snd_soc_update_bits(codec, RT5640_JD_CTRL,
					RT5640_JD_MASK, RT5640_JD_JD1_IN4P);
	}

	ret = snd_soc_jack_new(codec, "Intel MID Audio Jack",
			       SND_JACK_HEADSET | SND_JACK_HEADPHONE | SND_JACK_BTN_0,
			       &ctx->jack);
	if (ret) {
		pr_err("jack creation failed\n");
		return ret;
	}
	snd_jack_set_key(ctx->jack.jack, SND_JACK_BTN_0, KEY_MEDIA);
	ret = snd_soc_jack_add_gpios(&ctx->jack, ctx->num_jack_gpios, hs_gpio);
	if (ret) {
		pr_err("adding jack GPIO failed\n");
		return ret;
	}

	ret = snd_soc_add_card_controls(card, byt_mc_controls,
					ARRAY_SIZE(byt_mc_controls));
	if (ret) {
		pr_err("unable to add card controls\n");
		return ret;
	}

#ifdef CONFIG_SND_SOC_COMMS_SSP
	/* Add Comms specific controls */
	ctx->comms_ctl.ssp_bt_sco_master_mode = false;
	ctx->comms_ctl.ssp_modem_master_mode = false;

	ret = snd_soc_add_card_controls(card, byt_ssp_comms_controls,
					ARRAY_SIZE(byt_ssp_comms_controls));

	if (ret) {
		pr_err("unable to add COMMS card controls\n");
		return ret;
	}
#endif /* CONFIG_SND_SOC_COMMS_SSP */

	/* Keep the voice call paths active during
	suspend. Mark the end points ignore_suspend */
	snd_soc_dapm_ignore_suspend(dapm, "HPOL");
	snd_soc_dapm_ignore_suspend(dapm, "HPOR");

	snd_soc_dapm_ignore_suspend(dapm, "SPOLP");
	snd_soc_dapm_ignore_suspend(dapm, "SPOLN");
	snd_soc_dapm_ignore_suspend(dapm, "SPORP");
	snd_soc_dapm_ignore_suspend(dapm, "SPORN");
	snd_soc_dapm_ignore_suspend(dapm, "AIF2 Playback");
	snd_soc_dapm_ignore_suspend(dapm, "AIF2 Capture");
	snd_soc_dapm_ignore_suspend(&card->dapm, "Headphone");
	snd_soc_dapm_ignore_suspend(&card->dapm, "Headset Mic");
	snd_soc_dapm_ignore_suspend(&card->dapm, "Ext Spk");
	snd_soc_dapm_ignore_suspend(&card->dapm, "Int Mic");

	snd_soc_dapm_enable_pin(dapm, "Headset Mic");
	snd_soc_dapm_enable_pin(dapm, "Headphone");
	snd_soc_dapm_enable_pin(dapm, "Ext Spk");
	snd_soc_dapm_enable_pin(dapm, "Int Mic");

	snd_soc_dapm_sync(dapm);
	return ret;
}

static unsigned int rates_48000[] = {
	48000,
};

static struct snd_pcm_hw_constraint_list constraints_48000 = {
	.count = ARRAY_SIZE(rates_48000),
	.list  = rates_48000,
};

static int byt_aif1_startup(struct snd_pcm_substream *substream)
{
	return snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE,
			&constraints_48000);
}

static struct snd_soc_ops byt_aif1_ops = {
	.startup = byt_aif1_startup,
	.hw_params = byt_aif1_hw_params,
};
static struct snd_soc_ops byt_aif2_ops = {
	.hw_params = byt_aif2_hw_params,
	.hw_free = byt_aif2_hw_free,
};
static struct snd_soc_compr_ops byt_compr_ops = {
	.set_params = byt_compr_set_params,
};

#ifdef CONFIG_SND_SOC_COMMS_SSP
static struct snd_soc_ops byt_comms_dai_link_ops = {
	.startup = byt_comms_dai_link_startup,
	.hw_params = byt_comms_dai_link_hw_params,
	.prepare = byt_comms_dai_link_prepare,
};
#endif /* CONFIG_SND_SOC_COMMS_SSP */

static struct snd_soc_dai_link byt_dailink[] = {
	[BYT_AUD_AIF1] = {
		.name = "Baytrail Audio",
		.stream_name = "Audio",
		.cpu_dai_name = "Headset-cpu-dai",
		.codec_dai_name = "rt5640-aif1",
		.codec_name = "rt5640.2-001c",
		.platform_name = "sst-platform",
		.init = byt_init,
		.ignore_suspend = 1,
		.ops = &byt_aif1_ops,
		.playback_count = 2,
	},
	[BYT_AUD_AIF2] = {
		.name = "Baytrail Voice",
		.stream_name = "Voice",
		.cpu_dai_name = "Voice-cpu-dai",
		.codec_dai_name = "rt5640-aif2",
		.codec_name = "rt5640.2-001c",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
		.ops = &byt_aif2_ops,
	},
	[BYT_AUD_COMPR_DEV] = {
		.name = "Baytrail Compressed Audio",
		.stream_name = "Compress",
		.cpu_dai_name = "Compress-cpu-dai",
		.codec_dai_name = "rt5640-aif1",
		.codec_name = "rt5640.2-001c",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
		.compr_ops = &byt_compr_ops,
	},
#ifdef CONFIG_SND_SOC_COMMS_SSP
	[BYT_COMMS_BT] = {
		.name = "Baytrail Comms BT SCO",
		.stream_name = "BYT_BTSCO",
		.cpu_dai_name = SSP_BT_DAI_NAME,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.platform_name = "mid-ssp-dai",
		.init = NULL,
		.ops = &byt_comms_dai_link_ops,
	},
	[BYT_COMMS_MODEM] = {
		.name = "Baytrail Comms MODEM",
		.stream_name = "BYT_MODEM_MIXING",
		.cpu_dai_name = SSP_MODEM_DAI_NAME,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.platform_name = "mid-ssp-dai",
		.init = NULL,
		.ops = &byt_comms_dai_link_ops,
	},
#endif /* CONFIG_SND_SOC_COMMS_SSP */
};

#ifdef CONFIG_PM_SLEEP
static int snd_byt_prepare(struct device *dev)
{
	pr_debug("In %s device name\n", __func__);
	return snd_soc_suspend(dev);
}

static void snd_byt_complete(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	snd_soc_resume(dev);
}

static int snd_byt_poweroff(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	return snd_soc_poweroff(dev);
}
#else
#define snd_byt_prepare NULL
#define snd_byt_complete NULL
#define snd_byt_poweroff NULL
#endif

/* SoC card */
static struct snd_soc_card snd_soc_card_byt = {
	.name = "baytrailaudio",
	.dai_link = byt_dailink,
	.num_links = ARRAY_SIZE(byt_dailink),
	.set_bias_level = byt_set_bias_level,
	.dapm_widgets = byt_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(byt_dapm_widgets),
	.dapm_routes = byt_audio_map,
	.num_dapm_routes = ARRAY_SIZE(byt_audio_map),
};

static int snd_byt_mc_probe(struct platform_device *pdev)
{
	int ret_val = 0;
	struct byt_mc_private *drv;
	int codec_gpio;
	int jd_gpio;

	pr_debug("Entry %s\n", __func__);

	drv = devm_kzalloc(&pdev->dev, sizeof(*drv), GFP_ATOMIC);
	if (!drv) {
		pr_err("allocation failed\n");
		return -ENOMEM;
	}
	/* get the codec -> SoC GPIO */
	codec_gpio = acpi_get_gpio_by_index(&pdev->dev, 0, NULL);  /* GPIO_SUS4 */
	pr_debug("%s: GPIOs - codec %d", __func__, codec_gpio);
	hs_gpio[BYT_CODEC_GPIO_IDX].gpio = codec_gpio;


	drv->hs_insert_det_delay = BYT_HS_INSERT_DET_DELAY;
	drv->hs_remove_det_delay = BYT_HS_REMOVE_DET_DELAY;
	drv->button_det_delay = BYT_BUTTON_DET_DELAY;
	drv->hs_det_poll_intrvl = BYT_HS_DET_POLL_INTRVL;
	drv->hs_det_retry = BYT_HS_DET_RETRY_COUNT;
	drv->button_en_delay = BYT_BUTTON_EN_DELAY;
	drv->process_button_events = false;

	INIT_DELAYED_WORK(&drv->hs_insert_work, byt_check_hs_insert_status);
	INIT_DELAYED_WORK(&drv->hs_remove_work, byt_check_hs_remove_status);
	INIT_DELAYED_WORK(&drv->hs_button_work, byt_check_hs_button_status);
	INIT_DELAYED_WORK(&drv->hs_button_en_work, byt_enable_hs_button_events);
	mutex_init(&drv->jack_mlock);
	drv->tristate_buffer_gpio = -1;
	drv->num_jack_gpios = 1;
	drv->use_soc_jd_gpio = false;

	/* FFRD PR1 has 2 SoC gpios for Jack detect/Button press. One GPIO is for
	   codec interrupt(codec-> SoC) and the second GPIO is for jack detection
	   alone (direct jack-> SoC).Since there is a dedicated jack det GPIO on PR1,
	   codec interrupt is used for button press alone. On PR0 and RVP there is
	   only one GPIO(codec->SoC)for Jack/Button detection. Hence both jack
	   detection and button press are handled via codec interrupt. */
	if (INTEL_MID_BOARD(3, TABLET, BYT, BLK, PRO, 8PR1)) {
		/*Get the direct jack detect GPIO (jack -> soc GPIO). */
		jd_gpio = acpi_get_gpio("\\_SB.GPO2", 27);
		if (jd_gpio >= 0) {
			pr_info("%s: GPIOs - Jack detection %d", __func__, jd_gpio);

			hs_gpio[BYT_JD_GPIO_IDX].gpio = jd_gpio;
			/* One GPIO for codec interrupt and other for jack detection */
			drv->num_jack_gpios = 2;
			drv->use_soc_jd_gpio = true;
		} else
			pr_err("%s: Could not get Jack det GPIO. Use codec GPIO instead", __func__);


		/* Configure GPIO_SCORE56 for BT SCO workaround on FFRD8 PR1 */
		drv->tristate_buffer_gpio = acpi_get_gpio("\\_SB.GPO0", 56);
		ret_val = devm_gpio_request_one(&pdev->dev,
					drv->tristate_buffer_gpio,
					GPIOF_OUT_INIT_LOW,
					"byt_ffrd8_tristate_buffer_gpio");
		if (ret_val) {
			pr_err("Tri-state buffer gpio config failed %d\n",
				ret_val);
			return ret_val;
		}
	}

	/* register the soc card */
	byt_pdev = pdev;
	snd_soc_card_byt.dev = &pdev->dev;
	snd_soc_card_set_drvdata(&snd_soc_card_byt, drv);
	ret_val = snd_soc_register_card(&snd_soc_card_byt);
	if (ret_val) {
		pr_err("snd_soc_register_card failed %d\n", ret_val);
		return ret_val;
	}
	platform_set_drvdata(pdev, &snd_soc_card_byt);
	register_reboot_notifier(&byt_bl_reboot_notifier_block);
	pr_info("%s successful\n", __func__);
	return ret_val;
}

static void snd_byt_unregister_jack(struct byt_mc_private *ctx)
{
	/* Set process button events to false so that the button
	   delayed work will not be scheduled.*/
	ctx->process_button_events = false;
	cancel_delayed_work_sync(&ctx->hs_insert_work);
	cancel_delayed_work_sync(&ctx->hs_button_en_work);
	cancel_delayed_work_sync(&ctx->hs_button_work);
	cancel_delayed_work_sync(&ctx->hs_remove_work);
	snd_soc_jack_free_gpios(&ctx->jack, ctx->num_jack_gpios, hs_gpio);
}

static int snd_byt_mc_remove(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);
	struct byt_mc_private *drv = snd_soc_card_get_drvdata(soc_card);

	pr_debug("In %s\n", __func__);
	unregister_reboot_notifier(&byt_bl_reboot_notifier_block);
	snd_byt_unregister_jack(drv);
	snd_soc_card_set_drvdata(soc_card, NULL);
	snd_soc_unregister_card(soc_card);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static void snd_byt_mc_shutdown(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);
	struct byt_mc_private *drv = snd_soc_card_get_drvdata(soc_card);

	pr_debug("In %s\n", __func__);
	snd_byt_unregister_jack(drv);
}

const struct dev_pm_ops snd_byt_mc_pm_ops = {
	.prepare = snd_byt_prepare,
	.complete = snd_byt_complete,
	.poweroff = snd_byt_poweroff,
};

static const struct acpi_device_id byt_mc_acpi_ids[] = {
	{ "AMCR0F28", 0 },
	{},
};
MODULE_DEVICE_TABLE(acpi, byt_mc_acpi_ids);

static struct platform_driver snd_byt_mc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "byt_rt5642",
		.pm = &snd_byt_mc_pm_ops,
		.acpi_match_table = ACPI_PTR(byt_mc_acpi_ids),
	},
	.probe = snd_byt_mc_probe,
	.remove = snd_byt_mc_remove,
	.shutdown = snd_byt_mc_shutdown,
};

static int __init snd_byt_driver_init(void)
{
	pr_info("Baytrail Machine Driver byt_rt5642 registerd\n");
	return platform_driver_register(&snd_byt_mc_driver);
}
late_initcall(snd_byt_driver_init);

static void __exit snd_byt_driver_exit(void)
{
	pr_debug("In %s\n", __func__);
	platform_driver_unregister(&snd_byt_mc_driver);
}
module_exit(snd_byt_driver_exit);

MODULE_DESCRIPTION("ASoC Intel(R) Baytrail Machine driver");
MODULE_AUTHOR("Omair Md Abdullah <omair.m.abdullah@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:bytrt5642-audio");
