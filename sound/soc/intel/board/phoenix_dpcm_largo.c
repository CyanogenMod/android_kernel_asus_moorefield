/*
 *  moor_dpcm_florida.c - ASOC Machine driver for Intel Moorfield platform
 *
 *  Copyright (C) 2014 Wolfson Micro
 *  Copyright (C) 2014 Intel Corp
 *  Author: Nikesh Oswal <Nikesh.Oswal@wolfsonmicro.com>
 *			Praveen Diwakar <praveen.diwakar@intel.com>
 *
 * Based on
 *	merr_dpcm_wm8958.c - ASOC Machine driver for Intel Merrfield MID platform
 *	Copyright (C) 2013 Intel Corp
  *     Author: Vinod Koul <vinod.koul@intel.com>
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
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/async.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <asm/intel_scu_pmic.h>
#include <asm/intel_mid_rpmsg.h>
#include <asm/platform_mrfld_audio.h>
#include <asm/intel_sst_mrfld.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <linux/input.h>
#include <asm/intel-mid.h>

#include <linux/mfd/arizona/registers.h>
#include "../../codecs/largo.h"

/* Codec PLL output clk rate */
#define CODEC_SYSCLK_RATE			49152000
/* Input clock to codec at MCLK1 PIN */
#define CODEC_IN_MCLK1_RATE			19200000
/* Input clock to codec at MCLK2 PIN */
#define CODEC_IN_MCLK2_RATE			32768
/*  define to select between MCLK1 and MCLK2 input to codec as its clock */
#define CODEC_IN_MCLK1				1
#define CODEC_IN_MCLK2				2

/* Register address for OSC Clock */
#define MERR_OSC_CLKOUT_CTRL0_REG_ADDR  0xFF00BC04
/* Size of osc clock register */
#define MERR_OSC_CLKOUT_CTRL0_REG_SIZE  4

struct mrfld_8958_mc_private {
	u8 pmic_id;
	void __iomem    *osc_clk0_reg;
};

/* set_osc_clk0-	enable/disables the osc clock0
 * addr:		address of the register to write to
 * enable:		bool to enable or disable the clock
 */
static inline void set_soc_osc_clk0(void __iomem *addr, bool enable)
{
	u32 osc_clk_ctrl;

	osc_clk_ctrl = readl(addr);
	if (enable)
		osc_clk_ctrl |= BIT(31);
	else
		osc_clk_ctrl &= ~(BIT(31));

	pr_debug("%s: enable:%d val 0x%x\n", __func__, enable, osc_clk_ctrl);

	writel(osc_clk_ctrl, addr);
}

static inline struct snd_soc_codec *moor_florida_get_codec(struct snd_soc_card *card)
{
	bool found = false;
	struct snd_soc_codec *codec;

	list_for_each_entry(codec, &card->codec_dev_list, card_list) {
		if (!strstr(codec->name, "largo-codec")) {
			pr_err("codec was %s", codec->name);
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

static struct snd_soc_dai *moor_florida_get_codec_dai(struct snd_soc_card *card, const char *dai_name)
{
	int i;
	for (i = 0; i < card->num_rtd; i++) {
		if (!strcmp(card->rtd[i].codec_dai->name, dai_name))
			return card->rtd[i].codec_dai;
	}
	pr_err("%s: unable to find codec dai\n", __func__);
	/* this should never occur */
	WARN_ON(1);
	return NULL;
}

/* Function to switch the input clock for codec,  When audio is in
 * progress input clock to codec will be through MCLK1 which is 19.2MHz
 * while in off state input clock to codec will be through 32KHz through
 * MCLK2
 * card	: Sound card structure
 * src	: Input clock source to codec
 */
static int moor_florida_set_codec_clk(struct snd_soc_codec *florida_codec, int src)
{
	int ret;

	/*reset FLL1*/
	snd_soc_codec_set_pll(florida_codec, LARGO_FLL1_REFCLK,
				ARIZONA_FLL_SRC_NONE, 0, 0);
	snd_soc_codec_set_pll(florida_codec, LARGO_FLL1,
				ARIZONA_FLL_SRC_NONE, 0, 0);

	switch (src) {
	case CODEC_IN_MCLK1:
		/* Turn ON the PLL to generate required sysclk rate
		 * from MCLK1 */
		ret = snd_soc_codec_set_pll(florida_codec, LARGO_FLL1,
				ARIZONA_CLK_SRC_MCLK1,  CODEC_IN_MCLK1_RATE,
				CODEC_SYSCLK_RATE);
		if (ret != 0) {
			dev_err(florida_codec->dev, "Failed to enable FLL1 with Ref Clock Loop: %d\n", ret);
			return ret;
		}

		/*Switch to PLL*/
		ret = snd_soc_codec_set_sysclk(florida_codec,
				ARIZONA_CLK_SYSCLK, ARIZONA_CLK_SRC_FLL1,
				CODEC_SYSCLK_RATE, SND_SOC_CLOCK_IN);
		if (ret != 0) {
			dev_err(florida_codec->dev, "Failed to set SYSCLK to FLL1: %d\n", ret);
			return ret;
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int moor_florida_set_clk_fmt(struct snd_soc_codec *codec)
{
	int ret = 0;
	struct snd_soc_card *card = codec->card;
	struct mrfld_8958_mc_private *ctx = snd_soc_card_get_drvdata(card);

	/* Enable the osc clock at start so that it gets settling time */
	set_soc_osc_clk0(ctx->osc_clk0_reg, true);

	/* FIXME: move this to SYS_CLOCK event handler when codec driver
	 * dependency is clean.
	 */
	/* Switch to 19.2MHz MCLK1 input clock for codec */
	ret = moor_florida_set_codec_clk(codec, CODEC_IN_MCLK1);

	return ret;
}

static int moor_florida_hw_params(struct snd_pcm_substream *substream,
			   struct snd_pcm_hw_params *params)
{

	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;

	if (!strcmp(codec_dai->name, "largo-aif1"))
		return moor_florida_set_clk_fmt(rtd->codec);

	return 0;
}

#if 0
static int moor_florida_set_bias_level(struct snd_soc_card *card,
		struct snd_soc_dapm_context *dapm,
		enum snd_soc_bias_level level)
{
	struct snd_soc_dai *florida_dai = moor_florida_get_codec_dai(card, "largo-aif1");
	struct snd_soc_codec *florida_codec = moor_florida_get_codec(card);
	int ret = 0;

	if (!florida_dai || !florida_codec) {
		pr_err("%s: couldn't find the dai or codec pointer!\n", __func__);
		return -ENODEV;
	}

	if (dapm->dev != florida_dai->dev)
		return 0;

	if (level == SND_SOC_BIAS_PREPARE) {
		if (card->dapm.bias_level == SND_SOC_BIAS_STANDBY)
			ret = moor_florida_set_clk_fmt(florida_codec);
	}

	pr_debug("%s card(%s)->bias_level %u\n", __func__, card->name,
			card->dapm.bias_level);
	return ret;
}

static int moor_florida_set_bias_level_post(struct snd_soc_card *card,
		 struct snd_soc_dapm_context *dapm,
		 enum snd_soc_bias_level level)
{
	struct snd_soc_dai *florida_dai = moor_florida_get_codec_dai(card, "largo-aif1");
	struct snd_soc_codec *florida_codec = moor_florida_get_codec(card);
	struct mrfld_8958_mc_private *ctx = snd_soc_card_get_drvdata(card);
	int ret = 0;

	if (!florida_dai || !florida_codec) {
		pr_err("%s: couldn't find the dai or codec pointer!\n", __func__);
		return -ENODEV;
	}

	if (dapm->dev != florida_dai->dev)
		return 0;

	if (level == SND_SOC_BIAS_STANDBY) {
		/* Turn off PLL for MCLK1 */
		ret = snd_soc_codec_set_pll(florida_codec, LARGO_FLL1, 0, 0, 0);
		if (ret != 0) {
			dev_err(florida_codec->dev, "Failed to diasble FLL1 %d\n", ret);
			return ret;
		}
		/* We are in stabdby so turn off 19.2MHz soc osc clock*/
		/*The 32K Clk of codec is sourced from MCLK2 connected to onboard OSC*/
		set_soc_osc_clk0(ctx->osc_clk0_reg, false);
	}
	card->dapm.bias_level = level;
	pr_debug("%s card(%s)->bias_level %u\n", __func__, card->name,
			card->dapm.bias_level);
	return ret;
}
#endif

#define PMIC_ID_ADDR		0x00
#define PMIC_CHIP_ID_A0_VAL	0xC0
#if 0
static int moor_florida_set_vflex_vsel(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *k, int event)
{
#define VFLEXCNT		0xAB
#define VFLEXVSEL_5V		0x01
#define VFLEXVSEL_B0_VSYS_PT	0x80	/* B0: Vsys pass-through */
#define VFLEXVSEL_A0_4P5V	0x41	/* A0: 4.5V */

	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct mrfld_8958_mc_private *ctx = snd_soc_card_get_drvdata(card);

	u8 vflexvsel, pmic_id = ctx->pmic_id;
	int retval = 0;

	pr_debug("%s: ON? %d\n", __func__, SND_SOC_DAPM_EVENT_ON(event));

	vflexvsel = (pmic_id == PMIC_CHIP_ID_A0_VAL) ? VFLEXVSEL_A0_4P5V : VFLEXVSEL_B0_VSYS_PT;
	pr_debug("pmic_id %#x vflexvsel %#x\n", pmic_id,
		SND_SOC_DAPM_EVENT_ON(event) ? VFLEXVSEL_5V : vflexvsel);

	/*FIXME: seems to be issue with bypass mode in MOOR, for now
		force the bias off volate as VFLEXVSEL_5V */
	if ((INTEL_MID_BOARD(1, PHONE, MOFD)) ||
			(INTEL_MID_BOARD(1, TABLET, MOFD)))
		vflexvsel = VFLEXVSEL_5V;

	if (SND_SOC_DAPM_EVENT_ON(event))
		retval = intel_scu_ipc_iowrite8(VFLEXCNT, VFLEXVSEL_5V);
	else if (SND_SOC_DAPM_EVENT_OFF(event))
		retval = intel_scu_ipc_iowrite8(VFLEXCNT, vflexvsel);
	if (retval)
		pr_err("Error writing to VFLEXCNT register\n");

	return retval;
}
#endif
static const struct snd_soc_dapm_widget moor_widgets[] = {
	SND_SOC_DAPM_HP("Headphones", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
	SND_SOC_DAPM_SPK("EP", NULL),
	SND_SOC_DAPM_MIC("AMIC", NULL),
	SND_SOC_DAPM_MIC("DMIC", NULL),
};

static const struct snd_soc_dapm_route moor_map[] = {
	/*Headphones*/
	{ "Headphones", NULL, "HPOUT1L" },
	{ "Headphones", NULL, "HPOUT1R" },

	/*Speakers*/
	{"Ext Spk", NULL, "SPKOUTLP"},
	{"Ext Spk", NULL, "SPKOUTLN"},
	{"Ext Spk", NULL, "SPKOUTRP"},
	{"Ext Spk", NULL, "SPKOUTRN"},
	{"Ext Spk", NULL, "AIF3TX1"},
	{"Ext Spk", NULL, "AIF3TX2"},

	/*Earpiece*/
	{ "EP", NULL, "HPOUT3L" },
	{ "EP", NULL, "HPOUT3R" },

	/*On Board DMIC*/
	{"DMIC", NULL, "MICBIAS1"},
	{"DMIC", NULL, "MICBIAS2"},
	{"IN1L", NULL, "DMIC"},
	{"IN1R", NULL, "DMIC"},
	{"IN2L", NULL, "DMIC"},
	{"IN2R", NULL, "DMIC"},

	/* SWM map link the SWM outs to codec AIF */
	{ "AIF1 Playback", NULL, "ssp2 Tx"},
	{ "ssp2 Tx", NULL, "codec_out0"},
	{ "ssp2 Tx", NULL, "codec_out1"},
	{ "codec_in0", NULL, "ssp2 Rx" },
	{ "codec_in1", NULL, "ssp2 Rx" },
	{ "ssp2 Rx", NULL, "AIF1 Capture"},

	{ "ssp0 Tx", NULL, "modem_out"},
	{ "modem_in", NULL, "ssp0 Rx" },

	{ "ssp1 Tx", NULL, "bt_fm_out"},
	{ "bt_fm_in", NULL, "ssp1 Rx" },

	{ "AIF1 Playback", NULL, "VFLEXCNT" },
	{ "AIF1 Capture", NULL, "VFLEXCNT" },
};

static const struct snd_kcontrol_new moor_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphones"),
	SOC_DAPM_PIN_SWITCH("Ext Spk"),
	SOC_DAPM_PIN_SWITCH("EP"),
	SOC_DAPM_PIN_SWITCH("AMIC"),
	SOC_DAPM_PIN_SWITCH("DMIC"),
};

static int moor_florida_init(struct snd_soc_pcm_runtime *runtime)
{
	int ret;
	unsigned int fmt;
	struct snd_soc_card *card = runtime->card;
	struct snd_soc_dai *florida_dai = moor_florida_get_codec_dai(card, "largo-aif1");
	struct snd_soc_dai *florida_dai3 = moor_florida_get_codec_dai(card, "largo-aif3");



	pr_debug("Entry %s\n", __func__);

	if (!florida_dai) {
		pr_err("%s couldn't find the dai or codec pointer!\n", __func__);
		return -ENODEV;
	}

	if (!florida_dai3) {
		pr_err("%s couldn't find the dai3 or codec pointer!\n", __func__);
		return -ENODEV;
	}

	ret = snd_soc_dai_set_tdm_slot(florida_dai, 0, 0, 4, 24);
	/* slot width is set as 25, SNDRV_PCM_FORMAT_S32_LE */
	if (ret < 0) {
		pr_err("can't set codec pcm format %d\n", ret);
		return ret;
	}

	fmt =   SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_IB_NF
		| SND_SOC_DAIFMT_CBS_CFS;
	ret = snd_soc_dai_set_fmt(florida_dai, fmt);
	if (ret < 0) {
		pr_err("can't set codec DAI configuration %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_tdm_slot(florida_dai3, 0, 0, 2, 24);
	/* slot width is set as 25, SNDRV_PCM_FORMAT_S32_LE */
	if (ret < 0) {
		pr_err("can't set dai3 codec pcm format %d\n", ret);
		return ret;
	}

	fmt =   SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_IB_NF
		| SND_SOC_DAIFMT_CBM_CFM;
	ret = snd_soc_dai_set_fmt(florida_dai3, fmt);
	if (ret < 0) {
		pr_err("can't set codec DAI3 configuration %d\n", ret);
		return ret;
	}

	card->dapm.idle_bias_off = true;

	ret = snd_soc_add_card_controls(card, moor_controls,
					ARRAY_SIZE(moor_controls));
	if (ret) {
		pr_err("unable to add card controls\n");
		return ret;
	}

	return 0;
}

static unsigned int rates_8000_16000[] = {
	8000,
	16000,
};

static struct snd_pcm_hw_constraint_list constraints_8000_16000 = {
	.count = ARRAY_SIZE(rates_8000_16000),
	.list  = rates_8000_16000,
};

static unsigned int rates_48000[] = {
	48000,
};

static struct snd_pcm_hw_constraint_list constraints_48000 = {
	.count = ARRAY_SIZE(rates_48000),
	.list  = rates_48000,
};

static int moor_florida_startup(struct snd_pcm_substream *substream)
{
	return snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE,
			&constraints_48000);
}

static int moor_florida_8k_16k_startup(struct snd_pcm_substream *substream)
{
	return snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE,
			&constraints_8000_16000);
}

static struct snd_soc_ops moor_florida_ops = {
	.startup = moor_florida_startup,
};

static struct snd_soc_ops moor_florida_be_ssp2_ops = {
	.hw_params = moor_florida_hw_params,
};

static struct snd_soc_ops moor_florida_8k_16k_ops = {
	.startup = moor_florida_8k_16k_startup,
	.hw_params = moor_florida_hw_params,
};

static struct snd_soc_compr_ops moor_compr_ops = {
	.set_params = NULL,
};

static int moor_florida_codec_fixup(struct snd_soc_pcm_runtime *rtd,
			    struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_CHANNELS);

	pr_debug("Invoked %s for dailink %s\n", __func__, rtd->dai_link->name);

	/* The DSP will covert the FE rate to 48k, stereo, 24bits */
	rate->min = rate->max = 48000;
	channels->min = channels->max = 4;

	/* set SSP2 to 24-bit */
	snd_mask_set(&params->masks[SNDRV_PCM_HW_PARAM_FORMAT -
				    SNDRV_PCM_HW_PARAM_FIRST_MASK],
				    SNDRV_PCM_FORMAT_S24_LE);
	return 0;
}

static const struct snd_soc_pcm_stream moor_florida_dai_params = {
	.formats = SNDRV_PCM_FMTBIT_S24_LE,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 4,
	.channels_max = 4,
};

struct snd_soc_dai_link moor_florida_msic_dailink[] = {
	[MERR_DPCM_AUDIO] = {
		.name = "Merrifield Audio Port",
		.stream_name = "Saltbay Audio",
		.cpu_dai_name = "Headset-cpu-dai",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "sst-platform",
		.init = moor_florida_init,
		.ignore_suspend = 1,
		.dynamic = 1,
		.ops = &moor_florida_ops,
	},
	[MERR_DPCM_DB] = {
		.name = "Merrifield DB Audio Port",
		.stream_name = "Deep Buffer Audio",
		.cpu_dai_name = "Deepbuffer-cpu-dai",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "sst-platform",
		.ignore_suspend = 1,
		.dynamic = 1,
		.ops = &moor_florida_ops,
	},
	[MERR_DPCM_LL] = {
		.name = "Merrifield LL Audio Port",
		.stream_name = "Low Latency Audio",
		.cpu_dai_name = "Lowlatency-cpu-dai",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "sst-platform",
		.ignore_suspend = 1,
		.dynamic = 1,
		.ops = &moor_florida_ops,
	},
	[MERR_DPCM_COMPR] = {
		.name = "Merrifield Compress Port",
		.stream_name = "Saltbay Compress",
		.platform_name = "sst-platform",
		.cpu_dai_name = "Compress-cpu-dai",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.compr_ops = &moor_compr_ops,
	},
	[MERR_DPCM_VOIP] = {
		.name = "Merrifield VOIP Port",
		.stream_name = "Saltbay Voip",
		.cpu_dai_name = "Voip-cpu-dai",
		.platform_name = "sst-platform",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.init = NULL,
		.ignore_suspend = 1,
		.ops = &moor_florida_8k_16k_ops,
		.dynamic = 1,
	},
	[MERR_DPCM_PROBE] = {
		.name = "Merrifield Probe Port",
		.stream_name = "Saltbay Probe",
		.cpu_dai_name = "Probe-cpu-dai",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.platform_name = "sst-platform",
		.playback_count = 8,
		.capture_count = 8,
	},
	/* CODEC<->CODEC link */
	{
		.name = "Merrifield Codec-Loop Port",
		.stream_name = "Saltbay Codec-Loop",
		.cpu_dai_name = "ssp2-port",
		.platform_name = "sst-platform",
		.codec_dai_name = "largo-aif1",
		.codec_name = "largo-codec",
		.dai_fmt = SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_IB_NF
						| SND_SOC_DAIFMT_CBS_CFS,
		.params = &moor_florida_dai_params,
		.dsp_loopback = true,
	},
	{
		.name = "Merrifield Modem-Loop Port",
		.stream_name = "Saltbay Modem-Loop",
		.cpu_dai_name = "ssp0-port",
		.platform_name = "sst-platform",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.params = &moor_florida_dai_params,
		.dsp_loopback = true,
	},
	{
		.name = "Merrifield BTFM-Loop Port",
		.stream_name = "Saltbay BTFM-Loop",
		.cpu_dai_name = "ssp1-port",
		.platform_name = "sst-platform",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.params = &moor_florida_dai_params,
		.dsp_loopback = true,
	},

	/* back ends */
	{
		.name = "SSP2-Codec",
		.be_id = 1,
		.cpu_dai_name = "ssp2-port",
		.platform_name = "sst-platform",
		.no_pcm = 1,
		.codec_dai_name = "largo-aif1",
		.codec_name = "largo-codec",
		.be_hw_params_fixup = moor_florida_codec_fixup,
		.ignore_suspend = 1,
		.ops = &moor_florida_be_ssp2_ops,
	},
	{
		.name = "SSP1-BTFM",
		.be_id = 2,
		.cpu_dai_name = "snd-soc-dummy-dai",
		.platform_name = "snd-soc-dummy",
		.no_pcm = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.ignore_suspend = 1,
	},
	{
		.name = "SSP0-Modem",
		.be_id = 3,
		.cpu_dai_name = "snd-soc-dummy-dai",
		.platform_name = "snd-soc-dummy",
		.no_pcm = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.ignore_suspend = 1,
	},
	{

		.name = "SSP3-Codec",
		.be_id = 4,
		.cpu_dai_name = "snd-soc-dummy-dai",
		.platform_name = "snd-soc-dummy",
		.no_pcm = 1,
		.codec_name = "largo-codec",
		.codec_dai_name = "largo-aif3",
		.ignore_suspend = 1,
	},
};

#ifdef CONFIG_PM_SLEEP
static int snd_moor_florida_prepare(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	return snd_soc_suspend(dev);
}

static void snd_moor_florida_complete(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	snd_soc_resume(dev);

	return;
}

static int snd_moor_florida_poweroff(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	return snd_soc_poweroff(dev);
}
#else
#define snd_moor_florida_prepare NULL
#define snd_moor_florida_complete NULL
#define snd_moor_florida_poweroff NULL
#endif

static int moor_florida_mc_late_probe(struct snd_soc_card *card)
{
	int ret;
	struct snd_soc_dai *florida_dai = moor_florida_get_codec_dai(card, "largo-aif1");
	struct snd_soc_codec *florida_codec = moor_florida_get_codec(card);

	if (!florida_dai || !florida_codec) {
		pr_err("%s: couldn't find the dai or codec pointer!\n", __func__);
		return -ENODEV;
	}

	ret = snd_soc_dai_set_sysclk(florida_dai,  ARIZONA_CLK_SYSCLK, 0, 0);
	if (ret != 0) {
		dev_err(card->rtd[0].codec->dev, "Failed to set codec dai clk domain: %d\n", ret);
		return ret;
	}

	/*Configure SAMPLE_RATE_1 and ASYNC_SAMPLE_RATE_1 by default to
	48KHz these values can be changed in runtime by corresponding
	DAI hw_params callback */
	snd_soc_update_bits(florida_codec, ARIZONA_SAMPLE_RATE_1,
		ARIZONA_SAMPLE_RATE_1_MASK, 0x03);
	snd_soc_update_bits(florida_codec, ARIZONA_ASYNC_SAMPLE_RATE_1,
		ARIZONA_SAMPLE_RATE_1_MASK, 0x03);

	return 0;
}

/* SoC card */
static struct snd_soc_card snd_soc_card_moor = {
	.name = "phoenix-audio",
	.dai_link = moor_florida_msic_dailink,
	.num_links = ARRAY_SIZE(moor_florida_msic_dailink),
	.late_probe = moor_florida_mc_late_probe,
	.dapm_widgets = moor_widgets,
	.num_dapm_widgets = ARRAY_SIZE(moor_widgets),
	.dapm_routes = moor_map,
	.num_dapm_routes = ARRAY_SIZE(moor_map),
};

static int snd_moor_florida_mc_probe(struct platform_device *pdev)
{
	int ret_val = 0;
	struct mrfld_8958_mc_private *drv;

	pr_err("Phoenix Entry %s\n", __func__);

	drv = devm_kzalloc(&pdev->dev, sizeof(*drv), GFP_ATOMIC);
	if (!drv) {
		pr_err("allocation failed\n");
		return -ENOMEM;
	}

	/* ioremap the register */
	drv->osc_clk0_reg = devm_ioremap_nocache(&pdev->dev,
					MERR_OSC_CLKOUT_CTRL0_REG_ADDR,
					MERR_OSC_CLKOUT_CTRL0_REG_SIZE);
	if (!drv->osc_clk0_reg) {
		pr_err("osc clk0 ctrl ioremap failed\n");
		ret_val = -1;
		goto unalloc;
	}

	/*ret_val = intel_scu_ipc_ioread8(PMIC_ID_ADDR, &drv->pmic_id);
	if (ret_val) {
		pr_err("Error reading PMIC ID register\n");
		goto unalloc;
	}*/

	/* register the soc card */
	snd_soc_card_moor.dev = &pdev->dev;
	snd_soc_card_set_drvdata(&snd_soc_card_moor, drv);
	ret_val = snd_soc_register_card(&snd_soc_card_moor);
	if (ret_val) {
		pr_err("snd_soc_register_card failed %d\n", ret_val);
		goto unalloc;
	}
	platform_set_drvdata(pdev, &snd_soc_card_moor);
	pr_info("%s successful\n", __func__);
	return ret_val;

unalloc:
	devm_kfree(&pdev->dev, drv);
	return ret_val;
}

static int snd_moor_florida_mc_remove(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);
	struct mrfld_8958_mc_private *drv = snd_soc_card_get_drvdata(soc_card);

	pr_debug("In %s\n", __func__);
	devm_kfree(&pdev->dev, drv);
	snd_soc_card_set_drvdata(soc_card, NULL);
	snd_soc_unregister_card(soc_card);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

const struct dev_pm_ops snd_moor_florida_mc_pm_ops = {
	.prepare = snd_moor_florida_prepare,
	.complete = snd_moor_florida_complete,
	.poweroff = snd_moor_florida_poweroff,
};

static struct platform_driver snd_moor_florida_mc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "phoenix_largo",
		.pm = &snd_moor_florida_mc_pm_ops,
	},
	.probe = snd_moor_florida_mc_probe,
	.remove = snd_moor_florida_mc_remove,
};

static int snd_moor_florida_driver_init(void)
{
	pr_err("Phoenix Machine Driver mrfld_wm8280 registerd\n");
	return platform_driver_register(&snd_moor_florida_mc_driver);
}

static void snd_moor_florida_driver_exit(void)
{
	pr_debug("In %s\n", __func__);
	platform_driver_unregister(&snd_moor_florida_mc_driver);
}

static int snd_moor_florida_rpmsg_probe(struct rpmsg_channel *rpdev)
{
	int ret = 0;

	if (rpdev == NULL) {
		pr_err("rpmsg channel not created\n");
		ret = -ENODEV;
		goto out;
	}

	dev_info(&rpdev->dev, "Probed snd_mrfld wm8280 rpmsg device\n");

	ret = snd_moor_florida_driver_init();

out:
	return ret;
}

static void snd_moor_florida_rpmsg_remove(struct rpmsg_channel *rpdev)
{
	snd_moor_florida_driver_exit();
	dev_info(&rpdev->dev, "Removed snd_mrfld wm8280 rpmsg device\n");
}

static void snd_moor_florida_rpmsg_cb(struct rpmsg_channel *rpdev, void *data,
				int len, void *priv, u32 src)
{
	dev_warn(&rpdev->dev, "unexpected, message\n");

	print_hex_dump(KERN_DEBUG, __func__, DUMP_PREFIX_NONE, 16, 1,
			data, len,  true);
}

static struct rpmsg_device_id snd_moor_florida_rpmsg_id_table[] = {
	{ .name = "rpmsg_phoenix_largo_audio" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, snd_moor_florida_rpmsg_id_table);

static struct rpmsg_driver snd_moor_florida_rpmsg = {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= snd_moor_florida_rpmsg_id_table,
	.probe		= snd_moor_florida_rpmsg_probe,
	.callback	= snd_moor_florida_rpmsg_cb,
	.remove		= snd_moor_florida_rpmsg_remove,
};

static int __init snd_moor_florida_rpmsg_init(void)
{
	return register_rpmsg_driver(&snd_moor_florida_rpmsg);
}
late_initcall(snd_moor_florida_rpmsg_init);

static void __exit snd_moor_florida_rpmsg_exit(void)
{
	return unregister_rpmsg_driver(&snd_moor_florida_rpmsg);
}
module_exit(snd_moor_florida_rpmsg_exit);

MODULE_DESCRIPTION("ASoC Moorefield Machine driver");
MODULE_AUTHOR("Nikesh Oswal <Nikesh.Oswal@wolfsonmicro.com>");
MODULE_AUTHOR("Praveen Diwakar <praveen.diwakar@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:phoenix_largo");
