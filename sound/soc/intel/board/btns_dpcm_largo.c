/*
 *  btns_dpcm_largo.c - ASOC Machine driver for Intel BTNS platform
 *
 *  Copyright (C) 2014 Wolfson Micro
 *  Copyright (C) 2014 Intel Corp
 *  Author: Nikesh Oswal <Nikesh.Oswal@wolfsonmicro.com>
 *	    Praveen Diwakar <praveen.diwakar@intel.com>
 *	    Sanyog Kale <sanyog.r.kale@intel.com>
 *
 * Based on
 *	merr_dpcm_wm8958.c - ASOC Machine driver for Intel Merrfield MID platform
 *	Copyright (C) 2013 Intel Corp
 *      Author: Vinod Koul <vinod.koul@intel.com>
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
#include "../platform-libs/controls_v2_dpcm.h"

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

struct moor_slot_info {
	unsigned int tx_mask;
	unsigned int rx_mask;
	int slots;
	int slot_width;
};

static const struct snd_soc_pcm_stream moor_wm8958_dai_params_ssp1_fm = {
	.formats = SNDRV_PCM_FMTBIT_S24_LE,
	.rate_min = SNDRV_BTNS_PCM_RATE_48000,
	.rate_max = SNDRV_BTNS_PCM_RATE_48000,
	.channels_min = 2,
	.channels_max = 2,
};

static const struct snd_soc_pcm_stream moor_wm8958_ssp1_bt_nb = {
	.formats = SNDRV_PCM_FMTBIT_S24_LE,
	.rate_min = SNDRV_BTNS_PCM_RATE_8000,
	.rate_max = SNDRV_BTNS_PCM_RATE_8000,
	.channels_min = 2,
	.channels_max = 2,
};

static const struct snd_soc_pcm_stream moor_wm8958_ssp1_bt_wb = {
	.formats = SNDRV_PCM_FMTBIT_S24_LE,
	.rate_min = SNDRV_BTNS_PCM_RATE_16000,
	.rate_max = SNDRV_BTNS_PCM_RATE_16000,
	.channels_min = 2,
	.channels_max = 2,
};

static const struct snd_soc_pcm_stream moor_wm8958_ssp1_bt_a2dp = {
	.formats = SNDRV_PCM_FMTBIT_S24_LE,
	.rate_min = SNDRV_BTNS_PCM_RATE_48000,
	.rate_max = SNDRV_BTNS_PCM_RATE_48000,
	.channels_min = 2,
	.channels_max = 2,
};

#define MOOR_CONFIG_SLOT(slot_tx_mask, slot_rx_mask, num_slot, width)\
	(struct moor_slot_info){ .tx_mask = slot_tx_mask,                      \
				 .rx_mask = slot_rx_mask,                      \
				 .slots = num_slot,                            \
				 .slot_width = width, }

static int moor_set_slot_and_format(struct snd_soc_dai *dai,
		struct moor_slot_info *slot_info, unsigned int fmt)
{
	int ret;

	ret = snd_soc_dai_set_tdm_slot(dai, slot_info->tx_mask,
		slot_info->rx_mask, slot_info->slots, slot_info->slot_width);
	if (ret < 0) {
		pr_err("can't set codec pcm format %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_fmt(dai, fmt);
	if (ret < 0) {
		pr_err("can't set codec DAI configuration %d\n", ret);
		return ret;
	}
	return ret;
}

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

static inline struct snd_soc_codec *btns_arizona_get_codec(struct snd_soc_card *card)
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

static struct snd_soc_dai *btns_arizona_get_codec_dai(struct snd_soc_card *card, const char *dai_name)
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
static int btns_arizona_set_codec_clk(struct snd_soc_codec *florida_codec, int src)
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

static int btns_arizona_set_clk_fmt(struct snd_soc_codec *codec)
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
	ret = btns_arizona_set_codec_clk(codec, CODEC_IN_MCLK1);

	return ret;
}

static int btns_arizona_hw_params(struct snd_pcm_substream *substream,
			   struct snd_pcm_hw_params *params)
{

	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;

	if (!strcmp(codec_dai->name, "largo-aif1"))
		return btns_arizona_set_clk_fmt(rtd->codec);

	return 0;
}

static int btns_arizona_set_bias_level(struct snd_soc_card *card,
		struct snd_soc_dapm_context *dapm,
		enum snd_soc_bias_level level)
{
	struct snd_soc_dai *arizona_dai = btns_arizona_get_codec_dai(card, "largo-aif1");
	struct snd_soc_codec *arizona_codec = btns_arizona_get_codec(card);
	int ret = 0;

	if (!arizona_dai || !arizona_codec) {
		pr_err("%s: couldn't find the dai or codec pointer!\n", __func__);
		return -ENODEV;
	}

	if (dapm->dev != arizona_dai->dev)
		return 0;

	if (level == SND_SOC_BIAS_PREPARE) {
		if (card->dapm.bias_level == SND_SOC_BIAS_STANDBY)
			ret = btns_arizona_set_clk_fmt(arizona_codec);
	}

	pr_debug("%s card(%s)->bias_level %u\n", __func__, card->name,
			card->dapm.bias_level);
	return ret;
}

static int btns_arizona_set_bias_level_post(struct snd_soc_card *card,
		 struct snd_soc_dapm_context *dapm,
		 enum snd_soc_bias_level level)
{
	struct snd_soc_dai *arizona_dai = btns_arizona_get_codec_dai(card, "largo-aif1");
	struct snd_soc_codec *arizona_codec = btns_arizona_get_codec(card);
	struct mrfld_8958_mc_private *ctx = snd_soc_card_get_drvdata(card);
	int ret = 0;

	if (!arizona_dai || !arizona_codec) {
		pr_err("%s: couldn't find the dai or codec pointer!\n", __func__);
		return -ENODEV;
	}

	if (dapm->dev != arizona_dai->dev)
		return 0;

	if (level == SND_SOC_BIAS_STANDBY) {
		/* Turn off PLL for MCLK1 */
		ret = snd_soc_codec_set_pll(arizona_codec, LARGO_FLL1, 0, 0, 0);
		if (ret != 0) {
			dev_err(arizona_codec->dev, "Failed to diasble FLL1 %d\n", ret);
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


#define PMIC_ID_ADDR		0x00 /* TBD: Need to get correct PMIC address for version number */
#define PMIC_CHIP_ID_A0_VAL	0xC0


static const struct snd_soc_dapm_widget btns_widgets[] = {
	SND_SOC_DAPM_HP("Headphones", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
	SND_SOC_DAPM_SPK("EP", NULL),
	SND_SOC_DAPM_MIC("AMIC", NULL),
	SND_SOC_DAPM_MIC("DMIC", NULL),
};

static const struct snd_soc_dapm_route btns_map[] = {
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

	{"DMIC", NULL, "DSP3"},

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

static const struct snd_kcontrol_new btns_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphones"),
	SOC_DAPM_PIN_SWITCH("Ext Spk"),
	SOC_DAPM_PIN_SWITCH("EP"),
	SOC_DAPM_PIN_SWITCH("AMIC"),
	SOC_DAPM_PIN_SWITCH("DMIC"),
};

static int btns_arizona_init(struct snd_soc_pcm_runtime *runtime)
{
	int ret;
	unsigned int fmt;
	struct snd_soc_card *card = runtime->card;
	struct snd_soc_dai *florida_dai = btns_arizona_get_codec_dai(card, "largo-aif1");
	struct snd_soc_dai *florida_dai3 = btns_arizona_get_codec_dai(card, "largo-aif3");

	struct snd_soc_codec *florida_codec = btns_arizona_get_codec(card);
	struct snd_soc_dapm_context *dapm;

	pr_debug("Entry %s\n", __func__);

	if (!florida_dai || !florida_codec) {
		pr_err("%s couldn't find the dai or codec pointer!\n", __func__);
		return -ENODEV;
	}

	if (!florida_dai) {
		pr_err("%s couldn't find the dai or codec pointer!\n", __func__);
		return -ENODEV;
	}

	if (!florida_dai3) {
		pr_err("%s couldn't find the dai3 or codec pointer!\n", __func__);
		return -ENODEV;
	}

	dapm = &(florida_codec->dapm);

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

	btns_arizona_set_bias_level(card, dapm, SND_SOC_BIAS_OFF);

	card->dapm.idle_bias_off = true;

	ret = snd_soc_add_card_controls(card, btns_controls,
					ARRAY_SIZE(btns_controls));
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

static int btns_arizona_startup(struct snd_pcm_substream *substream)
{
	return snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE,
			&constraints_48000);
}

static int btns_arizona_8k_16k_startup(struct snd_pcm_substream *substream)
{
	return snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE,
			&constraints_8000_16000);
}

static struct snd_soc_ops btns_arizona_ops = {
	.startup = btns_arizona_startup,
};

static struct snd_soc_ops btns_arizona_be_ssp2_ops = {
	.hw_params = btns_arizona_hw_params,
};

static struct snd_soc_ops btns_arizona_8k_16k_ops = {
	.startup = btns_arizona_8k_16k_startup,
	.hw_params = btns_arizona_hw_params,
};

static struct snd_soc_compr_ops btns_compr_ops = {
	.set_params = NULL,
};

static int btns_arizona_codec_fixup(struct snd_soc_pcm_runtime *rtd,
			    struct snd_pcm_hw_params *params)
{
	int ret = 0;
	unsigned int fmt;
	struct moor_slot_info *info;

	struct snd_interval *rate = hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS);

	/* WM8958 slave Mode */
	fmt =   SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_IB_NF |
		SND_SOC_DAIFMT_CBS_CFS;
	info = &MOOR_CONFIG_SLOT(0xF, 0xF, 4, SNDRV_PCM_FORMAT_S24_LE);
	ret = moor_set_slot_and_format(rtd->cpu_dai, info, fmt);




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

#define BT_DOMAIN_NB	0
#define BT_DOMAIN_WB	1
#define BT_DOMAIN_A2DP	2

static int btns_arizona_bt_fm_fixup(struct snd_soc_dai_link *dai_link, struct snd_soc_dai *dai)
{
	unsigned int fmt;
	bool is_bt;
	u16 is_bt_wb;
	unsigned int mask, reg_val;
	int ret;
	struct moor_slot_info *info;

	reg_val = snd_soc_platform_read(dai->platform, SST_MUX_REG);
	mask = (1 << fls(1)) - 1;
	is_bt = (reg_val >> SST_BT_FM_MUX_SHIFT) & mask;
	mask = (1 << fls(2)) - 1;
	is_bt_wb = (reg_val >> SST_BT_MODE_SHIFT) & mask;

	if (is_bt) {
		switch (is_bt_wb) {
		case BT_DOMAIN_WB:
			dai_link->params = &moor_wm8958_ssp1_bt_wb;
			info = &MOOR_CONFIG_SLOT(0x01, 0x01, 1, SNDRV_PCM_FORMAT_S16_LE);
			break;
		case BT_DOMAIN_NB:
			dai_link->params = &moor_wm8958_ssp1_bt_nb;
			info = &MOOR_CONFIG_SLOT(0x01, 0x01, 1, SNDRV_PCM_FORMAT_S16_LE);
			break;
		case BT_DOMAIN_A2DP:
			dai_link->params = &moor_wm8958_ssp1_bt_a2dp;
			info = &MOOR_CONFIG_SLOT(0x03, 0x03, 2, SNDRV_PCM_FORMAT_S16_LE);
			break;
		default:
			return -EINVAL;
		}
		fmt = SND_SOC_DAIFMT_IB_NF | SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_CBS_CFS;
	} else {
		fmt = SND_SOC_DAIFMT_IB_NF | SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_CBS_CFS;
		dai_link->params = &moor_wm8958_dai_params_ssp1_fm;
		info = &MOOR_CONFIG_SLOT(0x00, 0x03, 2, SNDRV_PCM_FORMAT_S16_LE);
	}
	ret = moor_set_slot_and_format(dai, info, fmt);

	return ret;
}

static const struct snd_soc_pcm_stream btns_arizona_dai_params = {
	.formats = SNDRV_PCM_FMTBIT_S24_LE,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 4,
	.channels_max = 4,
};

struct snd_soc_dai_link btns_arizona_msic_dailink[] = {
	[MERR_DPCM_AUDIO] = {
		.name = "Merrifield Audio Port",
		.stream_name = "Saltbay Audio",
		.cpu_dai_name = "Headset-cpu-dai",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "sst-platform",
		.init = btns_arizona_init,
		.ignore_suspend = 1,
		.dynamic = 1,
		.ops = &btns_arizona_ops,
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
		.ops = &btns_arizona_ops,
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
		.ops = &btns_arizona_ops,
	},
	[MERR_DPCM_COMPR] = {
		.name = "Merrifield Compress Port",
		.stream_name = "Saltbay Compress",
		.platform_name = "sst-platform",
		.cpu_dai_name = "Compress-cpu-dai",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.compr_ops = &btns_compr_ops,
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
		.ops = &btns_arizona_8k_16k_ops,
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
	[MERR_DPCM_CAPTURE] = {
		.name = "Merrifield Capture Port",
		.stream_name = "Saltbay Capture",
		.cpu_dai_name = "Capture-cpu-dai",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
		.dynamic = 1,
		.ops = &btns_arizona_ops,
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
		.params = &btns_arizona_dai_params,
		.dsp_loopback = true,
	},
	{
		.name = "Merrifield Modem-Loop Port",
		.stream_name = "Saltbay Modem-Loop",
		.cpu_dai_name = "ssp0-port",
		.platform_name = "sst-platform",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.params = &btns_arizona_dai_params,
		.dsp_loopback = true,
	},
	{
		.name = "Merrifield BTFM-Loop Port",
		.stream_name = "Saltbay BTFM-Loop",
		.cpu_dai_name = "ssp1-port",
		.platform_name = "sst-platform",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.params = &moor_wm8958_ssp1_bt_nb,
		.be_fixup = &btns_arizona_bt_fm_fixup,
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
		.be_hw_params_fixup = btns_arizona_codec_fixup,
		.ignore_suspend = 1,
		.ops = &btns_arizona_be_ssp2_ops,
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
static int snd_btns_arizona_prepare(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	return snd_soc_suspend(dev);
}

static void snd_btns_arizona_complete(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	snd_soc_resume(dev);

	return;
}

static int snd_btns_arizona_poweroff(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	return snd_soc_poweroff(dev);
}
#else
#define snd_btns_arizona_prepare NULL
#define snd_btns_arizona_complete NULL
#define snd_btns_arizona_poweroff NULL
#endif

static int btns_arizona_mc_late_probe(struct snd_soc_card *card)
{
	int ret;
	struct snd_soc_dai *florida_dai = btns_arizona_get_codec_dai(card, "largo-aif1");
	struct snd_soc_codec *florida_codec = btns_arizona_get_codec(card);

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
static struct snd_soc_card snd_soc_card_btns = {
	.name = "phoenix-audio",
	.dai_link = btns_arizona_msic_dailink,
	.num_links = ARRAY_SIZE(btns_arizona_msic_dailink),
	.late_probe = btns_arizona_mc_late_probe,
	.set_bias_level = btns_arizona_set_bias_level,
	.set_bias_level_post = btns_arizona_set_bias_level_post,
	.dapm_widgets = btns_widgets,
	.num_dapm_widgets = ARRAY_SIZE(btns_widgets),
	.dapm_routes = btns_map,
	.num_dapm_routes = ARRAY_SIZE(btns_map),
};

static int snd_btns_arizona_mc_probe(struct platform_device *pdev)
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
	snd_soc_card_btns.dev = &pdev->dev;
	snd_soc_card_set_drvdata(&snd_soc_card_btns, drv);
	ret_val = snd_soc_register_card(&snd_soc_card_btns);
	if (ret_val) {
		pr_err("snd_soc_register_card failed %d\n", ret_val);
		goto unalloc;
	}
	platform_set_drvdata(pdev, &snd_soc_card_btns);
	pr_info("%s successful\n", __func__);
	return ret_val;

unalloc:
	devm_kfree(&pdev->dev, drv);
	return ret_val;
}

static int snd_btns_arizona_mc_remove(struct platform_device *pdev)
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

const struct dev_pm_ops snd_btns_arizona_mc_pm_ops = {
	.prepare = snd_btns_arizona_prepare,
	.complete = snd_btns_arizona_complete,
	.poweroff = snd_btns_arizona_poweroff,
};

static struct platform_driver snd_btns_arizona_mc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "phoenix_largo",
		.pm = &snd_btns_arizona_mc_pm_ops,
	},
	.probe = snd_btns_arizona_mc_probe,
	.remove = snd_btns_arizona_mc_remove,
};

static int snd_btns_arizona_driver_init(void)
{
	pr_err("BTNS Machine Driver mrfld_cs47l24 registerd\n");

	return platform_driver_register(&snd_btns_arizona_mc_driver);
}

static void snd_btns_arizona_driver_exit(void)
{
	pr_debug("In %s\n", __func__);

	platform_driver_unregister(&snd_btns_arizona_mc_driver);
}

static int snd_btns_arizona_rpmsg_probe(struct rpmsg_channel *rpdev)
{
	int ret = 0;

	if (rpdev == NULL) {
		pr_err("rpmsg channel not created\n");
		ret = -ENODEV;
		goto out;
	}

	dev_info(&rpdev->dev, "Probed snd_mrfld cs47l24 rpmsg device\n");

	ret = snd_btns_arizona_driver_init();

out:
	return ret;
}

static void snd_btns_arizona_rpmsg_remove(struct rpmsg_channel *rpdev)
{
	snd_btns_arizona_driver_exit();
	dev_info(&rpdev->dev, "Removed snd_mrfld cs47l24 rpmsg device\n");
}

static void snd_btns_arizona_rpmsg_cb(struct rpmsg_channel *rpdev, void *data,
				int len, void *priv, u32 src)
{
	dev_warn(&rpdev->dev, "unexpected, message\n");

	print_hex_dump(KERN_DEBUG, __func__, DUMP_PREFIX_NONE, 16, 1,
			data, len,  true);
}

static struct rpmsg_device_id snd_btns_arizona_rpmsg_id_table[] = {
	{ .name = "rpmsg_phoenix_largo_audio" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, snd_btns_arizona_rpmsg_id_table);

static struct rpmsg_driver snd_btns_arizona_rpmsg = {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= snd_btns_arizona_rpmsg_id_table,
	.probe		= snd_btns_arizona_rpmsg_probe,
	.callback	= snd_btns_arizona_rpmsg_cb,
	.remove		= snd_btns_arizona_rpmsg_remove,
};

static int __init snd_btns_arizona_rpmsg_init(void)
{
	return register_rpmsg_driver(&snd_btns_arizona_rpmsg);
}
late_initcall(snd_btns_arizona_rpmsg_init);

static void __exit snd_btns_arizona_rpmsg_exit(void)
{
	return unregister_rpmsg_driver(&snd_btns_arizona_rpmsg);
}
module_exit(snd_btns_arizona_rpmsg_exit);

MODULE_DESCRIPTION("ASoC BTNS Machine driver");
MODULE_AUTHOR("Nikesh Oswal <Nikesh.Oswal@wolfsonmicro.com>");
MODULE_AUTHOR("Praveen Diwakar <praveen.diwakar@intel.com>");
MODULE_AUTHOR("Sanyog Kale <sanyog.r.kale@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:phoenix_largo");
