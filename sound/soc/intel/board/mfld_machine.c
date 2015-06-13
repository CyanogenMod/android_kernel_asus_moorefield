/*
 *  mfld_machine.c - ASoc Machine driver for Intel Medfield MID platform
 *
 *  Copyright (C) 2010-12 Intel Corp
 *  Author: Vinod Koul <vinod.koul@intel.com>
 *  Author: Harsha Priya <priya.harsha@intel.com>
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
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/async.h>
#include <linux/wakelock.h>
#include <linux/gpio.h>
#include <linux/rpmsg.h>
#include <asm/intel-mid.h>
#include <asm/intel_scu_ipcutil.h>
#include <asm/intel_mid_gpadc.h>
#include <asm/intel_mid_rpmsg.h>
#include <linux/platform_data/intel_mid_remoteproc.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/tlv.h>
#include <sound/msic_audio_platform.h>
#include "../../codecs/sn95031.h"
#include "mfld_common.h"

#define MFLD_JACK_DEBOUNCE_TIME	  250 /* mS */
#define MFLD_GPIO_HEADSET_DET_PIN 77

/* jack detection voltage zones */
static struct snd_soc_jack_zone mfld_zones[] = {
	{MFLD_MV_START, MFLD_MV_AM_HS, SND_JACK_HEADPHONE},
	{MFLD_MV_AM_HS, MFLD_MV_HS, SND_JACK_HEADSET},
};

/* jack detection voltage zones */

static void mfld_jack_enable_mic_bias(struct snd_soc_codec *codec)
{
	pr_debug("enable mic bias\n");
	mutex_lock(&codec->mutex);
	snd_soc_dapm_force_enable_pin(&codec->dapm, "AMIC1Bias");
	snd_soc_dapm_sync(&codec->dapm);
	mutex_unlock(&codec->mutex);
}

static void mfld_jack_disable_mic_bias(struct snd_soc_codec *codec)
{
	pr_debug("disable mic bias\n");
	mutex_lock(&codec->mutex);
	snd_soc_dapm_disable_pin(&codec->dapm, "AMIC1Bias");
	snd_soc_dapm_sync(&codec->dapm);
	mutex_unlock(&codec->mutex);
}

static int mfld_get_headset_state(struct snd_soc_jack *jack)
{
	int micbias, jack_type, gpio_state;
	struct mfld_mc_private *ctx =
			snd_soc_card_get_drvdata(jack->codec->card);

	mfld_jack_enable_mic_bias(jack->codec);
	micbias = mfld_jack_read_voltage(jack);

	jack_type = snd_soc_jack_get_type(jack, micbias);
	pr_debug("jack type detected = %d, micbias = %d\n", jack_type, micbias);

	if (jack_type != SND_JACK_HEADSET && jack_type != SND_JACK_HEADPHONE) {
		gpio_state = mfld_read_jack_gpio(ctx);
		if (gpio_state == 0) {
			jack_type = SND_JACK_HEADPHONE;
			pr_debug("GPIO says there is a headphone, reporting it\n");
		}
	}

	if (jack_type == SND_JACK_HEADSET)
		/* enable btn press detection */
		snd_soc_update_bits(jack->codec, SN95031_BTNCTRL2, BIT(0), 1);
	else
		mfld_jack_disable_mic_bias(jack->codec);

	return jack_type;
}

static void mfld_jack_report(struct snd_soc_jack *jack, unsigned int status)
{
	unsigned int mask = SND_JACK_BTN_0 | SND_JACK_HEADSET;

	pr_debug("jack reported of type: 0x%x\n", status);
	if ((status == SND_JACK_HEADSET) || (status == SND_JACK_HEADPHONE)) {
		/*
		 * if we detected valid headset then disable headset ground.
		 * this is required for jack detection to work well
		 */
		snd_soc_update_bits(jack->codec, SN95031_BTNCTRL2, BIT(1), 0);
	} else if (status == 0) {
		snd_soc_update_bits(jack->codec, SN95031_BTNCTRL2,
						BIT(1), BIT(1));
	}
	snd_soc_jack_report(jack, status, mask);
}

void mfld_jack_wq(struct work_struct *work)
{
	unsigned int mask = SND_JACK_BTN_0 | SND_JACK_HEADSET;
	struct mfld_mc_private *ctx =
		container_of(work, struct mfld_mc_private, jack_work.work.work);
	struct mfld_jack_work *jack_work = &ctx->jack_work;
	struct snd_soc_jack *jack = jack_work->jack;
	unsigned int voltage, status = 0, intr_id = jack_work->intr_id;
	int gpio_state;

	pr_debug("jack status in wq: 0x%x\n", intr_id);
	if (intr_id & SN95031_JACK_INSERTED) {
		status = mfld_get_headset_state(jack);
		/* unmask button press interrupts */
		if (status == SND_JACK_HEADSET)
			snd_soc_update_bits(jack->codec, SN95031_ACCDETMASK,
							BIT(1)|BIT(0), 0);
	} else if (intr_id & SN95031_JACK_REMOVED) {
		gpio_state = mfld_read_jack_gpio(ctx);
		if (gpio_state == 0) {
			pr_debug("remove interrupt, but GPIO says inserted\n");
			return;
		}
		pr_debug("reporting jack as removed\n");
		snd_soc_update_bits(jack->codec, SN95031_BTNCTRL2, BIT(0), 0);
		snd_soc_update_bits(jack->codec, SN95031_ACCDETMASK, BIT(2), 0);
		mfld_jack_disable_mic_bias(jack->codec);
		jack_work->intr_id = 0;
		cancel_delayed_work(&ctx->jack_work.work);
	} else if (intr_id & SN95031_JACK_BTN0) {
		if (ctx->mfld_jack_lp_flag) {
			snd_soc_jack_report(jack, SND_JACK_HEADSET, mask);
			ctx->mfld_jack_lp_flag = 0;
			pr_debug("short press on releasing long press, "
				   "report button release\n");
			return;
		} else {
			status = SND_JACK_HEADSET | SND_JACK_BTN_0;
			pr_debug("short press detected\n");
			snd_soc_jack_report(jack, status, mask);
			/* send explicit button release */
			if (status & SND_JACK_BTN_0)
				snd_soc_jack_report(jack,
						SND_JACK_HEADSET, mask);
			return;
		}
	} else if (intr_id & SN95031_JACK_BTN1) {
		/*
		 * we get spurious interrupts if jack key is held down
		 * so we ignore them until key is released by checking the
		 * voltage level
		 */
		if (ctx->mfld_jack_lp_flag) {
			voltage = mfld_jack_read_voltage(jack);
			if (voltage > MFLD_LP_THRESHOLD_VOLTAGE) {
				snd_soc_jack_report(jack,
						SND_JACK_HEADSET, mask);
				ctx->mfld_jack_lp_flag = 0;
				pr_debug("button released after long press\n");
			}
			return;
		}
		/*
		 * Codec sends separate long press event after button pressed
		 * for a specified time. Need to send separate button pressed
		 * and released events for Android
		 */
		status = SND_JACK_HEADSET | SND_JACK_BTN_0;
		ctx->mfld_jack_lp_flag = 1;
		pr_debug("long press detected\n");
	} else {
		pr_err("Invalid intr_id:0x%x\n", intr_id);
		return;
	}
	mfld_jack_report(jack, status);
}

static int mfld_schedule_jack_wq(struct mfld_jack_work *jack_work)
{
	return schedule_delayed_work(&jack_work->work,
			msecs_to_jiffies(MFLD_JACK_DEBOUNCE_TIME));
}

/*
 * The Medfield jack takes additional time for the interrupts to settle,
 * hence we have an software debounce mechanism so and take the value of
 * the final interrupt reported withing the debounce time to be true
 */
static void mfld_jack_detection(unsigned int intr_id,
				struct mfld_jack_work *jack_work)
{
	int retval;
	struct mfld_mc_private *ctx =
		container_of(jack_work, struct mfld_mc_private, jack_work);

	pr_debug("interrupt id read in sram = 0x%x\n", intr_id);

	if (intr_id & SN95031_JACK_INSERTED ||
				intr_id & SN95031_JACK_REMOVED) {
		ctx->jack_work.intr_id = intr_id;
		retval = mfld_schedule_jack_wq(jack_work);
		if (!retval)
			pr_debug("jack inserted/removed,intr already queued\n");
		/* mask button press interrupts until jack is reported*/
		snd_soc_update_bits(ctx->mfld_jack.codec,
		     SN95031_ACCDETMASK, BIT(1)|BIT(0), BIT(1)|BIT(0));
		return;
	}

	if (intr_id & SN95031_JACK_BTN0 ||
				intr_id & SN95031_JACK_BTN1) {
		if ((ctx->mfld_jack.status & SND_JACK_HEADSET) != 0) {
			ctx->jack_work.intr_id = intr_id;
			retval = mfld_schedule_jack_wq(jack_work);
			if (!retval) {
				pr_debug("spurious btn press, lp_flag:%d\n",
						ctx->mfld_jack_lp_flag);
				return;
			}
			pr_debug("BTN_Press detected\n");
		} else {
			pr_debug("BTN_press received, but jack is removed\n");
		}
	}
}

static const DECLARE_TLV_DB_SCALE(out_tlv, -6200, 100, 0);

static const struct snd_kcontrol_new mfld_snd_controls[] = {
	SOC_ENUM_EXT("Playback Switch", mfld_headset_enum,
			mfld_headset_get_switch, mfld_headset_set_switch),
	SOC_ENUM_EXT("Lineout Mux", sn95031_lo_enum,
			mfld_lo_get_switch, mfld_lo_set_switch),
	SOC_ENUM_EXT("PCM1 Mode", sn95031_pcm1_mode_config_enum,
			mfld_get_pcm1_mode, mfld_set_pcm1_mode),
	/* Add digital volume and mute controls for Headphone/Headset */
	SOC_DOUBLE_R_EXT_TLV("Headphone Playback Volume", SN95031_HSLVOLCTRL,
				SN95031_HSRVOLCTRL, 0, 71, 1,
				snd_soc_get_volsw_2r, mfld_set_vol_2r,
				out_tlv),
	SOC_DOUBLE_R_EXT_TLV("Speaker Playback Volume", SN95031_IHFLVOLCTRL,
				SN95031_IHFRVOLCTRL, 0, 71, 1,
				snd_soc_get_volsw_2r, mfld_set_vol_2r,
				out_tlv),
};

static const struct snd_soc_dapm_widget mfld_widgets[] = {
	SND_SOC_DAPM_HP("Headphones", NULL),
	SND_SOC_DAPM_MIC("Mic", NULL),
	/* Dummy widget to trigger VAUDA on/off */
	SND_SOC_DAPM_SUPPLY("Vibra1Clock", SND_SOC_NOPM, 0, 0,
			mfld_vibra_enable_clk,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("Vibra2Clock", SND_SOC_NOPM, 0, 0,
			mfld_vibra_enable_clk,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_route mfld_map[] = {
	{ "HPOUTL", NULL, "Headset Rail"},
	{ "HPOUTR", NULL, "Headset Rail"},
	{"Headphones", NULL, "HPOUTR"},
	{"Headphones", NULL, "HPOUTL"},
	{"AMIC1", NULL, "Mic"},
	{"VIB1SPI", NULL, "Vibra1Clock"},
	{"VIB2SPI", NULL, "Vibra2Clock"},
};

static int mfld_init(struct snd_soc_pcm_runtime *runtime)
{
	struct snd_soc_codec *codec = runtime->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct mfld_mc_private *ctx = snd_soc_card_get_drvdata(runtime->card);
	int ret_val;

	/* Add jack sense widgets */
	ret_val = snd_soc_add_codec_controls(codec, mfld_snd_controls,
				ARRAY_SIZE(mfld_snd_controls));
	if (ret_val) {
		pr_err("soc_add_controls failed %d", ret_val);
		return ret_val;
	}
	/* default is earpiece pin, userspace sets it explcitly */
	snd_soc_dapm_disable_pin(dapm, "Headphones");
	/* default is lineout NC, userspace sets it explcitly */
	snd_soc_dapm_disable_pin(dapm, "LINEOUTL");
	snd_soc_dapm_disable_pin(dapm, "LINEOUTR");
	ctx->sn95031_lo_dac = 3;
	ctx->hs_switch = 0;
	/* we dont use linein in this so set to NC */
	snd_soc_dapm_disable_pin(dapm, "LINEINL");
	snd_soc_dapm_disable_pin(dapm, "LINEINR");
	snd_soc_dapm_disable_pin(dapm, "DMIC2");
	snd_soc_dapm_disable_pin(dapm, "DMIC3");
	snd_soc_dapm_disable_pin(dapm, "DMIC4");
	snd_soc_dapm_disable_pin(dapm, "DMIC6");
	snd_soc_dapm_disable_pin(dapm, "AMIC2");

	/*
	 * Keep the voice call paths active during
	 * suspend. Mark the end points ignore_suspend
	 */
	snd_soc_dapm_ignore_suspend(dapm, "PCM1_IN");
	snd_soc_dapm_ignore_suspend(dapm, "PCM1_Out");
	snd_soc_dapm_ignore_suspend(dapm, "EPOUT");
	snd_soc_dapm_ignore_suspend(dapm, "IHFOUTL");
	snd_soc_dapm_ignore_suspend(dapm, "IHFOUTR");
	snd_soc_dapm_ignore_suspend(dapm, "DMIC1");
	snd_soc_dapm_ignore_suspend(dapm, "DMIC5");
	snd_soc_dapm_ignore_suspend(dapm, "Headphones");
	snd_soc_dapm_ignore_suspend(dapm, "Mic");
	mutex_lock(&codec->mutex);
	snd_soc_dapm_sync(dapm);
	mutex_unlock(&codec->mutex);
	/* Headset and button jack detection */
	ret_val = snd_soc_jack_new(codec, "Intel(R) MID Audio Jack",
			SND_JACK_HEADSET | SND_JACK_BTN_0,
			&ctx->mfld_jack);
	if (ret_val) {
		pr_err("jack creation failed\n");
		return ret_val;
	}

	ret_val = snd_soc_jack_add_zones(&ctx->mfld_jack,
			ARRAY_SIZE(mfld_zones), mfld_zones);
	if (ret_val) {
		pr_err("adding jack zones failed\n");
		return ret_val;
	}

	ctx->jack_work.jack = &ctx->mfld_jack;
	/*
	 * we want to check if anything is inserted at boot,
	 * so send a fake event to codec and it will read adc
	 * to find if anything is there or not
	 */
	ctx->jack_work.intr_id = MFLD_JACK_INSERT_ID;
	mfld_schedule_jack_wq(&ctx->jack_work);
	return ret_val;
}

#ifdef CONFIG_SND_MFLD_MONO_SPEAKER_SUPPORT
static int mfld_speaker_init(struct snd_soc_pcm_runtime *runtime)
{
	struct snd_soc_dai *cpu_dai = runtime->cpu_dai;
	struct snd_soc_dapm_context *dapm = &runtime->codec->dapm;
	struct snd_soc_codec *codec = runtime->codec;

	snd_soc_dapm_disable_pin(dapm, "IHFOUTR");
	mutex_lock(&codec->mutex);
	snd_soc_dapm_sync(dapm);
	mutex_unlock(&codec->mutex);
	return cpu_dai->driver->ops->set_tdm_slot(cpu_dai, 0, 0, 1, 0);
}
#endif

static int mfld_media_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	int ret;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;

	pr_debug("%s\n", __func__);
	/*
	 * Force the data width to 24 bit in MSIC since post processing
	 * algorithms in DSP enabled with 24 bit precision
	 */
	ret = snd_soc_codec_set_params(codec, SNDRV_PCM_FORMAT_S24_LE);
	if (ret < 0) {
		pr_debug("codec_set_params returned error %d\n", ret);
		return ret;
	}
	snd_soc_codec_set_pll(codec, 0, SN95031_PLLIN, 1, 1);

	/* VAUD needs to be on before configuring PLL */
	snd_soc_dapm_force_enable_pin(&codec->dapm, "VirtBias");
	mutex_lock(&codec->mutex);
	snd_soc_dapm_sync(&codec->dapm);
	mutex_unlock(&codec->mutex);
	usleep_range(5000, 6000);
	sn95031_configure_pll(codec, SN95031_ENABLE_PLL);

	/* enable PCM2 */
	snd_soc_dai_set_tristate(rtd->codec_dai, 0);
	return 0;
}

static int mfld_voice_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *soc_card = rtd->card;
	struct mfld_mc_private *ctx = snd_soc_card_get_drvdata(soc_card);
	pr_debug("%s\n", __func__);

	if (ctx->sn95031_pcm1_master_mode) { /* VOIP call */
		snd_soc_codec_set_pll(codec, 0, SN95031_PLLIN, 1, 1);
		snd_soc_dai_set_fmt(rtd->codec_dai, SND_SOC_DAIFMT_CBM_CFM
						| SND_SOC_DAIFMT_DSP_A);
		/* Sets the PCM1 clock rate */
		snd_soc_update_bits(codec, SN95031_PCM1C1, BIT(0)|BIT(1),
								BIT(0)|BIT(1));
	} else { /* CSV call */
		snd_soc_codec_set_pll(codec, 0, SN95031_PCM1BCLK, 1, 1);
		snd_soc_dai_set_fmt(rtd->codec_dai, SND_SOC_DAIFMT_CBS_CFS
						| SND_SOC_DAIFMT_I2S);
		snd_soc_update_bits(codec, SN95031_PCM1C1, BIT(0)|BIT(1), 0);
	}

	/* VAUD needs to be on before configuring PLL */
	snd_soc_dapm_force_enable_pin(&codec->dapm, "VirtBias");
	mutex_lock(&codec->mutex);
	snd_soc_dapm_sync(&codec->dapm);
	mutex_unlock(&codec->mutex);
	usleep_range(5000, 6000);
	sn95031_configure_pll(codec, SN95031_ENABLE_PLL);
	return 0;
}

static unsigned int rates_44100[] = {
	44100,
};

static struct snd_pcm_hw_constraint_list constraints_44100 = {
	.count	= ARRAY_SIZE(rates_44100),
	.list	= rates_44100,
};


static int mfld_media_startup(struct snd_pcm_substream *substream)
{
	pr_debug("%s - applying rate constraint\n", __func__);
	snd_pcm_hw_constraint_list(substream->runtime, 0,
				   SNDRV_PCM_HW_PARAM_RATE,
				   &constraints_44100);
	intel_scu_ipc_set_osc_clk0(true, CLK0_MSIC);
	return 0;
}

static void mfld_media_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	pr_debug("%s\n", __func__);

	snd_soc_dapm_disable_pin(&rtd->codec->dapm, "VirtBias");
	/* switch off PCM2 port */
	if (!rtd->codec->active)
		snd_soc_dai_set_tristate(codec_dai, 1);
}

static int mfld_voice_startup(struct snd_pcm_substream *substream)
{
	intel_scu_ipc_set_osc_clk0(true, CLK0_MSIC);
	return 0;
}

static void mfld_voice_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	pr_debug("%s\n", __func__);

	snd_soc_dapm_disable_pin(&rtd->codec->dapm, "VirtBias");
}

static struct snd_soc_ops mfld_media_ops = {
	.startup = mfld_media_startup,
	.shutdown = mfld_media_shutdown,
	.hw_params = mfld_media_hw_params,
};

static struct snd_soc_ops mfld_voice_ops = {
	.startup = mfld_voice_startup,
	.shutdown = mfld_voice_shutdown,
	.hw_params = mfld_voice_hw_params,
};

static struct snd_soc_dai_link mfld_msic_dailink[] = {
	{
		.name = "Medfield Headset",
		.stream_name = "Headset",
		.cpu_dai_name = "Headset-cpu-dai",
		.codec_dai_name = "SN95031 Headset",
		.codec_name = "sn95031",
		.platform_name = "sst-platform",
		.init = mfld_init,
		.ignore_suspend = 1,
		.ops = &mfld_media_ops,
	},
	{
		.name = "Medfield Speaker",
		.stream_name = "Speaker",
		.cpu_dai_name = "Speaker-cpu-dai",
		.codec_dai_name = "SN95031 Speaker",
		.codec_name = "sn95031",
		.platform_name = "sst-platform",
#ifdef CONFIG_SND_MFLD_MONO_SPEAKER_SUPPORT
		.init = mfld_speaker_init,
#else
		.init = NULL,
#endif
		.ignore_suspend = 1,
		.ops = &mfld_media_ops,
	},
	{
		.name = "Medfield Voice",
		.stream_name = "Voice",
		.cpu_dai_name = "Voice-cpu-dai",
		.codec_dai_name = "SN95031 Voice",
		.codec_name = "sn95031",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.ops = &mfld_voice_ops,
	},
};

#ifdef CONFIG_PM

static int snd_mfld_mc_suspend(struct device *dev)
{
	pr_debug("In %s device name\n", __func__);
	snd_soc_suspend(dev);
	return 0;
}
static int snd_mfld_mc_resume(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	snd_soc_resume(dev);
	return 0;
}

static int snd_mfld_mc_poweroff(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	snd_soc_poweroff(dev);
	return 0;
}

#else
#define snd_mfld_mc_suspend NULL
#define snd_mfld_mc_resume NULL
#define snd_mfld_mc_poweroff NULL
#endif

static int mfld_card_stream_event(struct snd_soc_dapm_context *dapm, int event)
{
	struct snd_soc_codec *codec;

	if (!dapm) {
		pr_err("%s: Null dapm\n", __func__);
		return -EINVAL;
	}
	/* we have only one codec in this machine */
	codec = list_entry(dapm->card->codec_dev_list.next,
				struct snd_soc_codec, card_list);
	if (!codec) {
		pr_err("%s: Null codec\n", __func__);
		return -EIO;
	}
	pr_debug("machine stream event: %d\n", event);
	if (event == SND_SOC_DAPM_STREAM_STOP) {
		if (!codec->active) {
			sn95031_configure_pll(codec, SN95031_DISABLE_PLL);
			return intel_scu_ipc_set_osc_clk0(false, CLK0_MSIC);
		}
	}
	return 0;
}

/* SoC card */
static struct snd_soc_card snd_soc_card_mfld = {
	.name = "medfield_audio",
	.dai_link = mfld_msic_dailink,
	.num_links = ARRAY_SIZE(mfld_msic_dailink),
	.dapm_widgets = mfld_widgets,
	.num_dapm_widgets = ARRAY_SIZE(mfld_widgets),
	.dapm_routes = mfld_map,
	.num_dapm_routes = ARRAY_SIZE(mfld_map),
};

static irqreturn_t snd_mfld_jack_intr_handler(int irq, void *dev)
{
	struct mfld_mc_private *ctx = (struct mfld_mc_private *) dev;
	u16 intr_status = 0;

	memcpy_fromio(&intr_status, ((void *)(ctx->int_base)), sizeof(u16));
	/* not overwrite status here */
	spin_lock(&ctx->lock);
	/*To retrieve the jack_interrupt_status value (MSB)*/
	ctx->jack_interrupt_status |= 0x0F & (intr_status >> 8);
	/*To retrieve the oc_interrupt_status value (LSB)*/
	ctx->oc_interrupt_status |= 0x1F & intr_status;
	spin_unlock(&ctx->lock);
#ifdef CONFIG_HAS_WAKELOCK
	/*
	 * We don't have any call back from the jack detection completed.
	 * Take wakelock for two seconds to give time for the detection
	 * to finish. Jack detection is happening rarely so this doesn't
	 * have big impact to power consumption.
	 */
	wake_lock_timeout(ctx->jack_wake_lock, 2*HZ);
#endif
	return IRQ_WAKE_THREAD;
}

static irqreturn_t snd_mfld_codec_intr_detection(int irq, void *data)
{
	struct mfld_mc_private *ctx = (struct mfld_mc_private *) data;
	unsigned long flags;
	u8 jack_int_value = 0;

	if (ctx->mfld_jack.codec == NULL) {
		pr_debug("codec NULL returning..");
		spin_lock_irqsave(&ctx->lock, flags);
		ctx->jack_interrupt_status = 0;
		ctx->oc_interrupt_status = 0;
		spin_unlock_irqrestore(&ctx->lock, flags);
		goto ret;
	}
	spin_lock_irqsave(&ctx->lock, flags);
	if (!(ctx->jack_interrupt_status || ctx->oc_interrupt_status)) {
		spin_unlock_irqrestore(&ctx->lock, flags);
		pr_err("OC and Jack Intr with status 0, return....\n");
		goto ret;
	}
	if (ctx->oc_interrupt_status) {
		pr_info("OC int value: %d\n", ctx->oc_interrupt_status);
		ctx->oc_interrupt_status = 0;
	}
	if (ctx->jack_interrupt_status) {
		jack_int_value = ctx->jack_interrupt_status;
		ctx->jack_interrupt_status = 0;
	}
	spin_unlock_irqrestore(&ctx->lock, flags);

	if (jack_int_value)
		mfld_jack_detection(jack_int_value, &ctx->jack_work);

ret:
	return IRQ_HANDLED;
}

static int __devinit snd_mfld_mc_probe(struct platform_device *pdev)
{
	int ret_val = 0, irq;
	struct mfld_mc_private *ctx;
	struct resource *irq_mem;
	struct msic_audio_platform_data *pdata;

	pr_debug("snd_mfld_mc_probe called\n");

	pdata = pdev->dev.platform_data;

	/* retrive the irq number */
	irq = platform_get_irq(pdev, 0);

	/*
	 * audio interrupt base of SRAM location where
	 * interrupts are stored by System FW
	 */
	ctx = kzalloc(sizeof(*ctx), GFP_ATOMIC);
	if (!ctx) {
		pr_err("allocation failed\n");
		return -ENOMEM;
	}
	spin_lock_init(&ctx->lock);
#ifdef CONFIG_HAS_WAKELOCK
	ctx->jack_wake_lock =
		kzalloc(sizeof(*(ctx->jack_wake_lock)), GFP_ATOMIC);
	wake_lock_init(ctx->jack_wake_lock, WAKE_LOCK_SUSPEND, "jack_detect");
#endif

	irq_mem = platform_get_resource_byname(pdev,
				IORESOURCE_MEM, "IRQ_BASE");
	if (!irq_mem) {
		pr_err("no mem resource given\n");
		ret_val = -ENODEV;
		goto unalloc;
	}

	/*GPADC handle for audio_detection*/
	ctx->audio_adc_handle =
		intel_mid_gpadc_alloc(MFLD_AUDIO_SENSOR,
				      MFLD_AUDIO_DETECT_CODE);
	if (!ctx->audio_adc_handle) {
		pr_err("invalid ADC handle\n");
		ret_val = -ENOMEM;
		goto unalloc;
	}
	INIT_DELAYED_WORK(&ctx->jack_work.work, mfld_jack_wq);

	/* Store jack gpio pin number in ctx for future reference */
	ctx->jack_gpio = pdata->jack_gpio;
	if (ctx->jack_gpio >= 0) {
		pr_info("GPIO for jack det is %d\n", ctx->jack_gpio);
		ret_val = gpio_request_one(ctx->jack_gpio,
					   GPIOF_DIR_IN,
					   "headset_detect_gpio");
		if (ret_val) {
			pr_err("Headset detect GPIO alloc fail:%d\n", ret_val);
			goto free_gpadc;
		}
	}

	ctx->int_base = ioremap_nocache(irq_mem->start, resource_size(irq_mem));
	if (!ctx->int_base) {
		pr_err("Mapping of cache failed\n");
		ret_val = -ENOMEM;
		goto free_gpio;
	}
	/* register for interrupt */
	ret_val = request_threaded_irq(irq, snd_mfld_jack_intr_handler,
			snd_mfld_codec_intr_detection,
			IRQF_SHARED | IRQF_NO_SUSPEND,
			pdev->dev.driver->name, ctx);
	if (ret_val) {
		pr_err("cannot register IRQ\n");
		goto free_gpio;
	}
	/* register the soc card */
	snd_soc_card_mfld.dev = &pdev->dev;
	snd_soc_card_mfld.dapm.stream_event = mfld_card_stream_event;
	snd_soc_card_set_drvdata(&snd_soc_card_mfld, ctx);
	ret_val = snd_soc_register_card(&snd_soc_card_mfld);
	if (ret_val) {
		pr_debug("snd_soc_register_card failed %d\n", ret_val);
		goto freeirq;
	}
	platform_set_drvdata(pdev, &snd_soc_card_mfld);
	pr_debug("successfully exited probe\n");
	return ret_val;

freeirq:
	free_irq(irq, ctx);
free_gpio:
	gpio_free(ctx->jack_gpio);
free_gpadc:
	intel_mid_gpadc_free(ctx->audio_adc_handle);
unalloc:
	kfree(ctx);
	return ret_val;
}

static int __devexit snd_mfld_mc_remove(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);
	struct mfld_mc_private *ctx = snd_soc_card_get_drvdata(soc_card);
	pr_debug("snd_mfld_mc_remove called\n");
	free_irq(platform_get_irq(pdev, 0), ctx);
#ifdef CONFIG_HAS_WAKELOCK
	if (wake_lock_active(ctx->jack_wake_lock))
		wake_unlock(ctx->jack_wake_lock);
	wake_lock_destroy(ctx->jack_wake_lock);
	kfree(ctx->jack_wake_lock);
#endif
	cancel_delayed_work(&ctx->jack_work.work);
	intel_mid_gpadc_free(ctx->audio_adc_handle);
	if (ctx->jack_gpio >= 0)
		gpio_free(ctx->jack_gpio);
	kfree(ctx);
	snd_soc_card_set_drvdata(soc_card, NULL);
	snd_soc_unregister_card(soc_card);
	platform_set_drvdata(pdev, NULL);
	return 0;
}
static const struct dev_pm_ops snd_mfld_mc_pm_ops = {
	.suspend = snd_mfld_mc_suspend,
	.resume = snd_mfld_mc_resume,
	.poweroff = snd_mfld_mc_poweroff,
};

static struct platform_driver snd_mfld_mc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "msic_audio",
		.pm   = &snd_mfld_mc_pm_ops,
	},
	.probe = snd_mfld_mc_probe,
	.remove = __devexit_p(snd_mfld_mc_remove),
};

static int snd_mfld_driver_init(void)
{
	pr_info("snd_mfld_driver_init called\n");
	return platform_driver_register(&snd_mfld_mc_driver);
}

static void snd_mfld_driver_exit(void)
{
	pr_debug("snd_mfld_driver_exit called\n");
	platform_driver_unregister(&snd_mfld_mc_driver);
}

static int snd_mfld_rpmsg_probe(struct rpmsg_channel *rpdev)
{
	int ret = 0;

	if (rpdev == NULL) {
		pr_err("rpmsg channel not created\n");
		ret = -ENODEV;
		goto out;
	}

	dev_info(&rpdev->dev, "Probed snd_mfld rpmsg device\n");

	ret = snd_mfld_driver_init();

out:
	return ret;
}

static void snd_mfld_rpmsg_remove(struct rpmsg_channel *rpdev)
{
	snd_mfld_driver_exit();
	dev_info(&rpdev->dev, "Removed snd_mfld rpmsg device\n");
}

static void snd_mfld_rpmsg_cb(struct rpmsg_channel *rpdev, void *data,
				int len, void *priv, u32 src)
{
	dev_warn(&rpdev->dev, "unexpected, message\n");

	print_hex_dump(KERN_DEBUG, __func__, DUMP_PREFIX_NONE, 16, 1,
			data, len,  true);
}

static struct rpmsg_device_id snd_mfld_rpmsg_id_table[] = {
	{ .name = "rpmsg_msic_audio" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, snd_mfld_rpmsg_id_table);

static struct rpmsg_driver snd_mfld_rpmsg = {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= snd_mfld_rpmsg_id_table,
	.probe		= snd_mfld_rpmsg_probe,
	.callback	= snd_mfld_rpmsg_cb,
	.remove		= snd_mfld_rpmsg_remove,
};

static int __init snd_mfld_rpmsg_init(void)
{
	return register_rpmsg_driver(&snd_mfld_rpmsg);
}

late_initcall(snd_mfld_rpmsg_init);

static void __exit snd_mfld_rpmsg_exit(void)
{
	return unregister_rpmsg_driver(&snd_mfld_rpmsg);
}
module_exit(snd_mfld_rpmsg_exit);

MODULE_DESCRIPTION("ASoC Intel(R) MID Machine driver");
MODULE_AUTHOR("Vinod Koul <vinod.koul@intel.com>");
MODULE_AUTHOR("Harsha Priya <priya.harsha@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("ipc:msic-audio");
