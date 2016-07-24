/*
 *  ctp_lt_wm8994.c - ASoc Machine driver for Intel Cloverview MID platform
 *
 *  Copyright (C) 2012-13 Intel Corp
 *  Author: KP Jeeja<jeeja.kp@intel.com>
 *  Author: Prusty, Subhransu S<subhransu.s.prusty@intel.com>
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


#define pr_fmt(fmt) KBUILD_MODNAME ":%s:%d: " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rpmsg.h>
#include <linux/init.h>
#include <linux/rpmsg.h>
#include <asm/intel_mid_rpmsg.h>
#include <asm/intel_mid_remoteproc.h>
#include <asm/platform_ctp_audio.h>
#include <asm/intel_sst_ctp.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <linux/mfd/arizona/registers.h>
#include "ctp_comms_common.h"
#include "ctp_common.h"
#include "../ssp/mid_ssp.h"
#include "../../codecs/wm8994.h"


#define MICBIAS_NAME "MICBIAS2"

/*Machine  widgets */
static const struct snd_soc_dapm_widget ctp_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", ctp_amp_event),
	SND_SOC_DAPM_MIC("DMIC", NULL),
	SND_SOC_DAPM_MIC("AMIC", NULL),
};

/* Machine Audio Map */
static const struct snd_soc_dapm_route ctp_audio_map[] = {
	{"Headphone", NULL, "HPOUT1R"},
	{"Headphone", NULL, "HPOUT1L"},
	{ "Ext Spk", NULL, "SPKOUTLN" },
	{ "Ext Spk", NULL, "SPKOUTLP" },

	{ "IN1LN", NULL, "AMIC" },
	{ "IN1LP", NULL, "AMIC" },

	{ "DMIC", NULL, "MICBIAS1"},
	{ "AMIC", NULL, "MICBIAS2"},
	{ "DMIC1DAT", NULL, "DMIC" },
	{ "DMIC2DAT", NULL, "DMIC" },
};

static int ctp_wm8994_startup(struct snd_pcm_substream *substream)
{
	unsigned int device = substream->pcm->device;

	pr_debug("applying rate constraint\n");
	switch (device) {
	case CTP_LT_AUD_ASP_DEV:
	case CTP_LT_AUD_PROBE_DEV:
		snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE, &constraints_48000);
			break;
	default:
		pr_err("Invalid device\n");
		return -EINVAL;
	}
	return 0;
}

static int ctp_wm8994_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int device = substream->pcm->device;
	struct ctp_clk_fmt clk_fmt;
	int ret = 0;

	switch (device) {
	case CTP_LT_AUD_ASP_DEV:
	case CTP_LT_AUD_VSP_DEV:
		clk_fmt.clk_id = WM8994_SYSCLK_FLL1;
		clk_fmt.freq = SYSCLK_RATE;
		clk_fmt.dir = SND_SOC_CLOCK_IN;
		/* Slave mode */
		clk_fmt.fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
						SND_SOC_DAIFMT_CBS_CFS;
		ret = ctp_set_clk_fmt(codec_dai, &clk_fmt);
		if (ret) {
			pr_err("clock fmt setting failed\n");
			return ret;
		}

		return snd_soc_dai_set_pll(codec_dai,
				WM8994_FLL1, WM8994_FLL_SRC_MCLK1,
				DEFAULT_MCLK, SYSCLK_RATE);
	default:
		pr_err("Invalid device\n");
		return -EINVAL;
	}
}


static int ctp_wm8994_set_params(struct snd_compr_stream *cstream)
{
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int device = cstream->device->device;
	struct ctp_clk_fmt clk_fmt;

	switch (device) {
	case CTP_LT_AUD_COMP_ASP_DEV:
		clk_fmt.clk_id = WM8994_SYSCLK_FLL1;
		clk_fmt.freq = SYSCLK_RATE;
		clk_fmt.dir = SND_SOC_CLOCK_IN;
		/* Slave mode */
		clk_fmt.fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
						SND_SOC_DAIFMT_CBS_CFS;
		return ctp_set_clk_fmt(codec_dai, &clk_fmt);
	default:
		pr_err("Invalid device\n");
		return -EINVAL;
	}
}

static int ctp_comms_dai_link_startup(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *str_runtime;

	str_runtime = substream->runtime;

	WARN(!substream->pcm, "CTP Comms Machine: ERROR NULL substream->pcm\n");

	if (!substream->pcm)
		return -EINVAL;

	/* set the runtime hw parameter with local snd_pcm_hardware struct */
	switch (substream->pcm->device) {
	case CTP_LT_COMMS_BT_SCO_DEV:
		str_runtime->hw = BT_sco_hw_param;
		break;

	case CTP_LT_COMMS_MSIC_VOIP_DEV:
		str_runtime->hw = VOIP_alsa_hw_param;
		break;

	default:
		pr_err("CTP Comms Machine: bad PCM Device = %d\n",
						substream->pcm->device);
	}
	return snd_pcm_hw_constraint_integer(str_runtime,
						SNDRV_PCM_HW_PARAM_PERIODS);
}

static int ctp_comms_dai_link_hw_params(struct snd_pcm_substream *substream,
				      struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card *soc_card = rtd->card;
	struct ctp_mc_private *ctx = snd_soc_card_get_drvdata(soc_card);
	struct comms_mc_private *ctl = &(ctx->comms_ctl);

	int ret = 0;
	unsigned int tx_mask, rx_mask;
	unsigned int nb_slot = 0;
	unsigned int slot_width = 0;
	unsigned int tristate_offset = 0;
	unsigned int device = substream->pcm->device;


	pr_debug("ssp_bt_sco_master_mode %d\n", ctl->ssp_bt_sco_master_mode);
	pr_debug("ssp_voip_master_mode %d\n", ctl->ssp_voip_master_mode);
	pr_debug("ssp_modem_master_mode %d\n", ctl->ssp_modem_master_mode);

	switch (device) {
	case CTP_LT_COMMS_BT_SCO_DEV:
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
				SND_SOC_DAIFMT_CBM_CFM : SND_SOC_DAIFMT_CBS_CFS));

		if (ret < 0) {
			pr_err("MFLD Comms Machine: Set FMT Fails %d\n",
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
		nb_slot = SSP_BT_SLOT_NB_SLOT;
		slot_width = SSP_BT_SLOT_WIDTH;
		tx_mask = SSP_BT_SLOT_TX_MASK;
		rx_mask = SSP_BT_SLOT_RX_MASK;

		if (ctl->ssp_bt_sco_master_mode)
			tristate_offset = BIT(TRISTATE_BIT);
		else
			tristate_offset = BIT(FRAME_SYNC_RELATIVE_TIMING_BIT);
		break;
	case CTP_LT_COMMS_MSIC_VOIP_DEV:
		/*
		 * set cpu DAI configuration
		 * frame_format = PSP_FORMAT
		 * ssp_serial_clk_mode = SSP_CLK_MODE_0
		 * ssp_frmsync_pol_bit = SSP_FRMS_ACTIVE_LOW
		 */
		ret = snd_soc_dai_set_fmt(cpu_dai,
				SND_SOC_DAIFMT_I2S |
				SSP_DAI_SCMODE_0 |
				SND_SOC_DAIFMT_NB_IF |
				(ctl->ssp_voip_master_mode ?
				SND_SOC_DAIFMT_CBM_CFM : SND_SOC_DAIFMT_CBS_CFS));

		if (ret < 0) {
			pr_err("MFLD Comms Machine: Set FMT Fails %d\n",
							ret);
			return -EINVAL;
		}

		/*
		 * MSIC VOIP SSP Config
		 * ssp_active_tx_slots_map = 0x01
		 * ssp_active_rx_slots_map = 0x01
		 * frame_rate_divider_control = 1
		 * data_size = 32
		 * tristate = 1
		 * ssp_frmsync_timing_bit = 0, for SLAVE
		 * (NEXT_FRMS_ASS_AFTER_END_OF_T4)
		 * ssp_frmsync_timing_bit = 1, for MASTER
		 * (NEXT_FRMS_ASS_WITH_LSB_PREVIOUS_FRM)
		 *
		 *
		 */
		nb_slot = SSP_VOIP_SLOT_NB_SLOT;
		slot_width = SSP_VOIP_SLOT_WIDTH;
		tx_mask = SSP_VOIP_SLOT_TX_MASK;
		rx_mask = SSP_VOIP_SLOT_RX_MASK;

		tristate_offset = BIT(TRISTATE_BIT);
		break;

	default:
		pr_err("CTP Comms Machine: bad PCM Device ID = %d\n",
				device);
		return -EINVAL;
	}

	ret = snd_soc_dai_set_tdm_slot(cpu_dai, tx_mask,
			rx_mask, nb_slot, slot_width);

	if (ret < 0) {
		pr_err("CTP Comms Machine:  Set TDM Slot Fails %d\n",
				ret);
		return -EINVAL;
	}

	ret = snd_soc_dai_set_tristate(cpu_dai, tristate_offset);
	if (ret < 0) {
		pr_err("CTP Comms Machine: Set Tristate Fails %d\n",
				ret);
		return -EINVAL;
	}

	if (device == CTP_LT_COMMS_MSIC_VOIP_DEV) {
		pr_debug("Call ctp_wm8994_hw_params to enable the PLL Codec\n");
		ctp_wm8994_hw_params(substream, params);
	}

	pr_debug("CTP Comms Machine: slot_width = %d\n",
			slot_width);
	pr_debug("CTP Comms Machine: tx_mask = %d\n",
			tx_mask);
	pr_debug("CTP Comms Machine: rx_mask = %d\n",
			rx_mask);
	pr_debug("CTP Comms Machine: tristate_offset = %d\n",
			tristate_offset);

	return 0;

} /* ctp_comms_dai_link_hw_params*/

static int ctp_comms_dai_link_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct ctp_mc_private *ctx = snd_soc_card_get_drvdata(rtd->card);
	struct comms_mc_private *ctl = &(ctx->comms_ctl);

	unsigned int device = substream->pcm->device;

	pr_debug("%s substream->runtime->rate %d\n",
			__func__,
			substream->runtime->rate);

	/* select clock source (if master) */
	/* BT SCO: CPU DAI is master */
	/* FM: CPU DAI is master */
	/* BT_VOIP: CPU DAI is master */
	if (((device == CTP_LT_COMMS_BT_SCO_DEV &&\
		ctl->ssp_bt_sco_master_mode) ||
		((device == CTP_LT_COMMS_MSIC_VOIP_DEV) &&\
		ctl->ssp_voip_master_mode))) {

		snd_soc_dai_set_sysclk(cpu_dai, SSP_CLK_ONCHIP,
				substream->runtime->rate, 0);
	}

	return 0;
} /* ctp_comms_dai_link_prepare */

static int ctp_wm8994_init(struct snd_soc_pcm_runtime *runtime)
{
	int ret;
	struct snd_soc_card *card = runtime->card;
	struct ctp_mc_private *ctx = snd_soc_card_get_drvdata(runtime->card);
	struct snd_soc_codec *codec = ctp_get_codec(card, "wm8994-codec");
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	/* Set codec bias level */
	ctp_set_bias_level(card, dapm, SND_SOC_BIAS_OFF);
	card->dapm.idle_bias_off = true;

	/* Add Jack specific widgets */
	ret = snd_soc_dapm_new_controls(dapm, ctp_dapm_widgets,
					ARRAY_SIZE(ctp_dapm_widgets));
	if (ret)
		return ret;

	/* Set up Jack specific audio path audio_map */
	snd_soc_dapm_add_routes(dapm, ctp_audio_map,
					ARRAY_SIZE(ctp_audio_map));
	/* Add Comms specefic controls */
	ctx->comms_ctl.ssp_bt_sco_master_mode = false;
	ctx->comms_ctl.ssp_voip_master_mode = false;

	ret = snd_soc_add_card_controls(card, ssp_comms_controls,
				ARRAY_SIZE(ssp_comms_controls));
	if (ret) {
		pr_err("Add Comms Controls failed %d",
				ret);
		return ret;
	}
	/* Keep the voice call paths active during
	suspend. Mark the end points ignore_suspend */
	snd_soc_dapm_ignore_suspend(dapm, "HPOUT1R");
	snd_soc_dapm_ignore_suspend(dapm, "HPOUT1L");

	snd_soc_dapm_disable_pin(dapm, "SPKOUTLP");
	snd_soc_dapm_disable_pin(dapm, "SPKOUTLN");
	snd_soc_dapm_ignore_suspend(dapm, "SPKOUTRP");
	snd_soc_dapm_ignore_suspend(dapm, "SPKOUTRN");
	mutex_lock(&codec->mutex);
	snd_soc_dapm_sync(dapm);
	mutex_unlock(&codec->mutex);
	return ret;
}

static struct snd_soc_ops ctp_wm8994_asp_ops = {
	.startup = ctp_wm8994_startup,
	.hw_params = ctp_wm8994_hw_params,
};

static struct snd_soc_compr_ops ctp_wm8994_asp_compr_ops = {
	.set_params = ctp_wm8994_set_params,
};

static struct snd_soc_ops ctp_wm8994_vsp_ops = {
	.hw_params = ctp_wm8994_hw_params,
};

/* dai link ops */
static struct snd_soc_ops ctp_comms_dai_link_ops = {
	.startup = ctp_comms_dai_link_startup,
	.hw_params = ctp_comms_dai_link_hw_params,
	.prepare = ctp_comms_dai_link_prepare,
};
static struct snd_soc_ops ctp_comms_voip_dai_link_ops = {
	.startup = ctp_comms_dai_link_startup,
	.hw_params = ctp_comms_dai_link_hw_params,
	.prepare = ctp_comms_dai_link_prepare,
};


static struct snd_soc_dai_link ctp_lt_wm8994_dailink[] = {
	[CTP_LT_AUD_ASP_DEV] = {
		.name = "Cloverview ASP",
		.stream_name = "Audio",
		.cpu_dai_name = "Headset-cpu-dai",
		.codec_dai_name = "wm8994-aif1",
		.codec_name = "wm8994-codec",
		.platform_name = "sst-platform",
		.init = snd_ctp_init,
		.ignore_suspend = 1,
		.ops = &ctp_wm8994_asp_ops,
	},
	[CTP_LT_AUD_VSP_DEV] = {
		.name = "Cloverview VSP",
		.stream_name = "Voice",
		.cpu_dai_name = "Voice-cpu-dai",
		.codec_dai_name = "wm8994-aif3",
		.codec_name = "wm8994-codec",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
		.ops = &ctp_wm8994_vsp_ops,
	},

	[CTP_LT_AUD_COMP_ASP_DEV] = {
		.name = "Cloverview Comp ASP",
		.stream_name = "Compress-Audio",
		.cpu_dai_name = "Compress-cpu-dai",
		.codec_dai_name = "wm8994-aif1",
		.codec_name = "wm8994-codec",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
		.compr_ops = &ctp_wm8994_asp_compr_ops,
	},
	[CTP_LT_COMMS_BT_SCO_DEV] = {
		.name = "Cloverview Comms BT SCO",
		.stream_name = "BTSCO",
		.cpu_dai_name = SSP_BT_DAI_NAME,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.platform_name = "mid-ssp-dai",
		.init = NULL,
		.ops = &ctp_comms_dai_link_ops,
	},
	[CTP_LT_COMMS_MSIC_VOIP_DEV] = {
		.name = "Cloverview Comms MSIC VOIP",
		.stream_name = "VOIP",
		.cpu_dai_name = SSP_BT_DAI_NAME,
		.codec_dai_name = "wm8994-aif2",
		.codec_name = "wm8994-codec",
		.platform_name = "mid-ssp-dai",
		.init = NULL,
		.ops = &ctp_comms_voip_dai_link_ops,
	},
};

static int ctp_wm8994_dai_link(struct snd_soc_card *card)
{
	pr_err("ctp_wm8994_dai_link....\n");
	card->dai_link = ctp_lt_wm8994_dailink;
	card->num_links = ARRAY_SIZE(ctp_lt_wm8994_dailink);
	return 0;
}

static void ctp_lt_card_name(struct snd_soc_card *card)
{
	card->name = "cloverview_audio";
}

struct snd_soc_machine_ops ctp_lt_wm8994_ops = {
	.card_name = ctp_lt_card_name,
	.ctp_init = ctp_wm8994_init,
	.dai_link = ctp_wm8994_dai_link,
	.jack_support = false,
	.mic_bias = MICBIAS_NAME,
};

MODULE_DESCRIPTION("ASoC Intel(R) Cloverview MID Machine driver");
MODULE_AUTHOR("Jeeja KP<jeeja.kp@intel.com>");
MODULE_AUTHOR("Prusty, Subhransu S<subhransu.s.prusty@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("ipc:ctpwm8994-audio");
