/*
 *  ctp_rhb_rt5671.c - ASoc Machine driver for Intel Cloverview MID platform
 *
 *  Copyright (C) 2011-13 Intel Corp
 *  Author: KP Jeeja<jeeja.kp@intel.com>
 *  Author: Vaibhav Agarwal <vaibhav.agarwal@intel.com>
 *  Author: Dharageswari.R<dharageswari.r@intel.com>
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
#define DEBUG 1

#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/async.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <asm/intel_sst_ctp.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include "../../codecs/rt5671.h"
#include "ctp_rhb_rt5671.h"
#include "../ssp/mid_ssp.h"
#include "ctp_common_rt5671.h"


/* As per the codec spec the mic2_sdet debounce delay is 20ms.
 * But having 20ms delay doesn't work */
#define MIC2SDET_DEBOUNCE_DELAY	50 /* 50 ms */
#define MICBIAS_NAME	"MIC2 Bias"

/* Headset jack detection gpios func(s) */
/* #define HPDETECT_POLL_INTERVAL	msecs_to_jiffies(1000) 1sec */
#define HPDETECT_DEBOUNCE_DELAY	50 /* 50 ms */

struct snd_soc_codec *rt5671_codec;
static unsigned int codec_clk_rate = DEFAULT_MCLK;
#define CTP_PLAT_CLK_3_HZ	19200000

/* ALC5671 widgets */
static const struct snd_soc_dapm_widget ctp_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Int Mic", NULL),
	SND_SOC_DAPM_SPK("Receiver", NULL),
	SND_SOC_DAPM_SPK("Ext Spk L", ctp_amp_event),
	SND_SOC_DAPM_SPK("Ext Spk R", NULL),
};

/* ALC5671 Audio Map */
static const struct snd_soc_dapm_route ctp_audio_map[] = {
	{"micbias1", NULL, "Headset Mic"},
	{"micbias1", NULL, "Headset Mic"},
	{"IN1P", NULL, "micbias1"},
	{"IN1N", NULL, "micbias1"},
	{"DMIC L1", NULL, "Int Mic"},
	{"DMIC R1", NULL, "Int Mic"},
	{"Headphone", NULL, "HPOL"},
	{"Headphone", NULL, "HPOR"},
	{"Ext Spk L", NULL, "LOUTL"},
	{"Ext Spk R", NULL, "LOUTR"},
	{"Receiver", NULL, "MonoP"},
	{"Receiver", NULL, "MonoN"},
};

#define SNDRV_BT_SCO_ENABLE	_IOW('S', 0x01, int)
#define SNDRV_CSV_CALL_ACTIVE  _IOW('S', 0x02, int)
static int CsvCallActive = -1;

static int ctp_startup(struct snd_pcm_substream *substream)
{
	unsigned int device = substream->pcm->device;
	pr_debug("%s - applying rate constraint\n", __func__);
	switch (device) {
	case CTP_RHB_AUD_ASP_DEV:
	case CTP_RHB_AUD_PROBE_DEV:
	case CTP_RHB_AUD_VIRTUAL_ASP_DEV:
		snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE, &constraints_48000);
		break;
	case CTP_RHB_AUD_VSP_DEV:
		ctp_config_voicecall_flag(substream, true);
		break;
	default:
		pr_err("Invalid device\n");
		return -EINVAL;
	}
	return 0;
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
	case CTP_COMMS_BT_SCO_DEV:
		str_runtime->hw = BT_sco_hw_param;
		break;

	case CTP_COMMS_MSIC_VOIP_DEV:
		str_runtime->hw = VOIP_alsa_hw_param;
		break;

	case CTP_COMMS_IFX_MODEM_DEV:
		str_runtime->hw = IFX_modem_alsa_hw_param;
		break;
	default:
		pr_err("CTP Comms Machine: bad PCM Device = %d\n",
						substream->pcm->device);
	}
	return snd_pcm_hw_constraint_integer(str_runtime,
						SNDRV_PCM_HW_PARAM_PERIODS);
}

int ctp_startup_fm_xsp(struct snd_pcm_substream *substream)
{
	pr_debug("%s - applying rate constraint\n", __func__);
	snd_pcm_hw_constraint_list(substream->runtime, 0,
				SNDRV_PCM_HW_PARAM_RATE,
				&constraints_48000);
	return 0;
}

int ctp_set_asp_clk_fmt(struct snd_soc_dai *codec_dai)
{
	unsigned int fmt;
	int ret;

	pr_debug("%s\n", __func__);

	/* ALC5671  Slave Mode`*/
	fmt =   SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF
		| SND_SOC_DAIFMT_CBS_CFS;

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);

	if (ret < 0) {
		pr_err("can't set codec DAI configuration %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_pll(codec_dai, 0, RT5671_PLL1_S_MCLK,
		CTP_PLAT_CLK_3_HZ, codec_clk_rate);

	if (ret < 0) {
		pr_err("can't set codec pll: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, RT5671_SCLK_S_PLL1,
		codec_clk_rate, SND_SOC_CLOCK_IN);

	if (ret < 0) {
		pr_err("can't set codec clock %d\n", ret);
		return ret;
	}
	return 0;
}

static int ctp_asp_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	codec_clk_rate = params_rate(params) * 512;
	return ctp_set_asp_clk_fmt(codec_dai);
}

static int clv_asp_set_params(struct snd_compr_stream *cstream)
{
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;

	return ctp_set_asp_clk_fmt(codec_dai);
}

static int ctp_vsp_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int fmt;
	int ret , clk_source;

	pr_debug("Slave Mode selected\n");
	/* ALC5671 Slave Mode`*/
	fmt =   SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF
		| SND_SOC_DAIFMT_CBS_CFS;
	clk_source = SND_SOC_CLOCK_IN;

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0) {
		pr_err("can't set codec DAI configuration %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_pll(codec_dai, 0, RT5671_PLL1_S_MCLK,
		CTP_PLAT_CLK_3_HZ, params_rate(params) * 512);

	ret = snd_soc_dai_set_sysclk(codec_dai, RT5671_SCLK_S_PLL1,
		params_rate(params) * 512, clk_source);

	if (ret < 0) {
		pr_err("can't set codec clock %d\n", ret);
		return ret;
	}
	return 0;
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

	pr_debug("\n ssp_voip_master_mode %d device %d\n",
		ctl->ssp_voip_master_mode, device);

	switch (device) {
	case CTP_COMMS_BT_SCO_DEV:
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
	case CTP_COMMS_MSIC_VOIP_DEV:
		/*
		 * set cpu DAI configuration
		 * frame_format = PSP_FORMAT
		 * ssp_serial_clk_mode = SSP_CLK_MODE_0
		 * ssp_frmsync_pol_bit = SSP_FRMS_ACTIVE_LOW
		 */

		pr_debug("CTP_COMMS_MSIC_VOIP_:EV ssp_voip_master_mode %d\n",
			ctl->ssp_voip_master_mode);

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

	case CTP_COMMS_IFX_MODEM_DEV:
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
				SND_SOC_DAIFMT_CBM_CFM : SND_SOC_DAIFMT_CBS_CFS));
		if (ret < 0) {
			pr_err("MFLD Comms Machine:  Set FMT Fails %d\n", ret);
			return -EINVAL;
		}

		/*
		 * IFX Modem Mixing SSP Config
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
		nb_slot = SSP_IFX_SLOT_NB_SLOT;
		slot_width = SSP_IFX_SLOT_WIDTH;
		tx_mask = SSP_IFX_SLOT_TX_MASK;
		rx_mask = SSP_IFX_SLOT_RX_MASK;

		tristate_offset = BIT(TRISTATE_BIT) |
				BIT(FRAME_SYNC_RELATIVE_TIMING_BIT);

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

	if (device == CTP_COMMS_MSIC_VOIP_DEV) {
		pr_debug("Call ctp_vsp_hw_params to enable the PLL Codec\n");
		ctp_vsp_hw_params(substream, params);
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
	if (((device == CTP_COMMS_BT_SCO_DEV &&
		ctl->ssp_bt_sco_master_mode) ||
		((device == CTP_COMMS_MSIC_VOIP_DEV) &&
		ctl->ssp_voip_master_mode)) ||
		(device == CTP_COMMS_IFX_MODEM_DEV &&
		ctl->ssp_modem_master_mode)) {

		snd_soc_dai_set_sysclk(cpu_dai, SSP_CLK_ONCHIP,
				substream->runtime->rate, 0);

	}

	return 0;
} /* ctp_comms_dai_link_prepare */

static const struct snd_kcontrol_new ssp_comms_controls[] = {
		SOC_ENUM_EXT("SSP BT Master Mode",
				ssp_bt_sco_master_mode_enum,
				get_ssp_bt_sco_master_mode,
				set_ssp_bt_sco_master_mode),
		SOC_ENUM_EXT("SSP VOIP Master Mode",
				ssp_voip_master_mode_enum,
				get_ssp_voip_master_mode,
				set_ssp_voip_master_mode),
		SOC_ENUM_EXT("SSP Modem Master Mode",
				ssp_modem_master_mode_enum,
				get_ssp_modem_master_mode,
				set_ssp_modem_master_mode),
		SOC_DAPM_PIN_SWITCH("Headphone"),
		SOC_DAPM_PIN_SWITCH("Headset Mic"),
		SOC_DAPM_PIN_SWITCH("Ext Spk L"),
		SOC_DAPM_PIN_SWITCH("Ext Spk R"),
		SOC_DAPM_PIN_SWITCH("Int Mic"),
		SOC_DAPM_PIN_SWITCH("Receiver"),
};

int switch_ctrl_open(struct inode *i_node, struct file *file_ptr)
{
	pr_debug("%s\n", __func__);
	return 0;
}

int switch_ctrl_release(struct inode *i_node, struct file *file_ptr)
{
	pr_debug("%s\n", __func__);
	return 0;
}

long switch_ctrl_ioctl(struct file *file_ptr,
		unsigned int cmd, unsigned long arg) {
	pr_debug("%s\n", __func__);

	switch (_IOC_NR(cmd)) {
	case _IOC_NR(SNDRV_BT_SCO_ENABLE): {
		/* Configure I2S HW switch for audio route */
		int bt_enable;
		if (copy_from_user(&bt_enable, (void __user *)arg,
				   sizeof(bt_enable)))
			return -EFAULT;
		pr_info("%s: BT SCO status %d\n", __func__, bt_enable);

		/* Configure I2S HW switch for audio route */

		break;
	}
	case _IOC_NR(SNDRV_CSV_CALL_ACTIVE): {
		int csv_call_active;

		if (copy_from_user(&csv_call_active,
			(void __user *)arg, sizeof(csv_call_active)))
			return -EFAULT;

		if (csv_call_active)
			CsvCallActive = 1;
		else
			CsvCallActive = 0;

		pr_debug("%s : CSV Call status %d %d\n", __func__,
		csv_call_active, CsvCallActive);
		break;
	}
	default:
		pr_err("%s: command not supported.\n", __func__);
		return -EINVAL;
	}
	return 0;
}

int is_csv_call_active(void)
{
	pr_debug("%s() : CsvCallActive %d\n", __func__, CsvCallActive);
	return CsvCallActive;
}
EXPORT_SYMBOL(is_csv_call_active);

static const struct file_operations switch_ctrl_fops = {
	.owner = THIS_MODULE,
	.open = switch_ctrl_open,
	.release = switch_ctrl_release,
	.unlocked_ioctl = switch_ctrl_ioctl,
};

static struct miscdevice switch_ctrl = {
	.minor = MISC_DYNAMIC_MINOR, /* dynamic allocation */
	.name = "switch_ctrl", /* /dev/bt_switch_ctrl */
	.fops = &switch_ctrl_fops
};

int ctp_init(struct snd_soc_pcm_runtime *runtime)
{
	int ret;
	struct snd_soc_codec *codec = runtime->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct snd_soc_card *card = runtime->card;
	struct ctp_mc_private *ctx = snd_soc_card_get_drvdata(runtime->card);

	pr_debug("%s\n", __func__);

	CsvCallActive = 0;

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

/* Set ssp_voip to master mode by default */
#if 0
	ctx->comms_ctl.ssp_voip_master_mode = false;
#else
	ctx->comms_ctl.ssp_voip_master_mode = true;
	pr_debug("ctp_init ssp_voip_master_mode %d\n",
		ctx->comms_ctl.ssp_voip_master_mode);
#endif

	ctx->comms_ctl.ssp_modem_master_mode = false;

	ret = snd_soc_add_card_controls(card, ssp_comms_controls,
				ARRAY_SIZE(ssp_comms_controls));
	if (ret) {
		pr_err("Add Comms Controls failed %d",
				ret);
		return ret;
	}

	/* Keep the voice call paths active during
	suspend. Mark the end points ignore_suspend */
	snd_soc_dapm_ignore_suspend(dapm, "HPOL");
	snd_soc_dapm_ignore_suspend(dapm, "HPOR");

/*	snd_soc_dapm_disable_pin(dapm, "MIC2");
	snd_soc_dapm_disable_pin(dapm, "SPKLINEOUT"); */
	snd_soc_dapm_ignore_suspend(dapm, "SPOLP");
	snd_soc_dapm_ignore_suspend(dapm, "SPOLN");
	snd_soc_dapm_ignore_suspend(dapm, "SPORP");
	snd_soc_dapm_ignore_suspend(dapm, "SPORN");
	snd_soc_dapm_ignore_suspend(dapm, "MonoP");
	snd_soc_dapm_ignore_suspend(dapm, "MonoN");
	snd_soc_dapm_ignore_suspend(dapm, "IN1P");
	snd_soc_dapm_ignore_suspend(dapm, "IN1N");
	snd_soc_dapm_ignore_suspend(dapm, "DMIC L1");
	snd_soc_dapm_ignore_suspend(dapm, "DMIC R1");
	snd_soc_dapm_ignore_suspend(dapm, "DMIC L2");
	snd_soc_dapm_ignore_suspend(dapm, "DMIC R2");
	snd_soc_dapm_ignore_suspend(dapm, "LOUTL");
	snd_soc_dapm_ignore_suspend(dapm, "LOUTR");
	snd_soc_dapm_ignore_suspend(dapm, "DSPDL");

	snd_soc_dapm_ignore_suspend(dapm, "Headset Mic");
	snd_soc_dapm_ignore_suspend(dapm, "Headphone");
	snd_soc_dapm_ignore_suspend(dapm, "Ext Spk L");
	snd_soc_dapm_ignore_suspend(dapm, "Ext Spk R");
	snd_soc_dapm_ignore_suspend(dapm, "Int Mic");
	snd_soc_dapm_ignore_suspend(dapm, "Receiver");
	snd_soc_dapm_ignore_suspend(dapm, "AIF2 Playback");
	snd_soc_dapm_ignore_suspend(dapm, "AIF2 Capture");
	snd_soc_dapm_ignore_suspend(dapm, "AIF2TX");
	snd_soc_dapm_ignore_suspend(dapm, "AIF2RX");

	snd_soc_dapm_enable_pin(dapm, "Headset Mic");
	snd_soc_dapm_enable_pin(dapm, "Headphone");
	snd_soc_dapm_enable_pin(dapm, "Ext Spk L");
	snd_soc_dapm_enable_pin(dapm, "Ext Spk R");
	snd_soc_dapm_enable_pin(dapm, "Int Mic");
	snd_soc_dapm_enable_pin(dapm, "Receiver");

	mutex_lock(&codec->mutex);
	snd_soc_dapm_sync(dapm);
	mutex_unlock(&codec->mutex);

	/*Register switch Control as misc driver*/
	ret = misc_register(&switch_ctrl);
	if (ret)
		pr_err("%s: couldn't register control device\n",
			__func__);

	rt5671_codec = codec;
	return ret;
}

static struct snd_soc_ops ctp_asp_ops = {
	.startup = ctp_startup_asp,
	.hw_params = ctp_asp_hw_params,
};

static struct snd_soc_compr_ops ctp_asp_compr_ops = {
	.set_params = clv_asp_set_params,
};

static struct snd_soc_ops ctp_vsp_ops = {
	.hw_params = ctp_vsp_hw_params,
};
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

static struct snd_soc_ops ctp_probe_ops = {
	.startup = ctp_startup,
};

static struct snd_soc_dai_link ctp_rhb_dailink[] = {
	[CTP_AUD_ASP_DEV] = {
		.name = "Cloverview ASP",
		.stream_name = "Audio",
		.cpu_dai_name = "Headset-cpu-dai",
		.codec_dai_name = "rt5671-aif1",
		.codec_name = "rt5671.1-001c",
		.platform_name = "sst-platform",
		.init = snd_ctp_init,
		.ignore_suspend = 1,
		.ops = &ctp_asp_ops,
		.playback_count = 2,
	},
	[CTP_AUD_VSP_DEV] = {
		.name = "Cloverview VSP",
		.stream_name = "Voice",
		.cpu_dai_name = "Voice-cpu-dai",
		.codec_dai_name = "rt5671-aif2",
		.codec_name = "rt5671.1-001c",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
		.ops = &ctp_vsp_ops,
	},
	[CTP_AUD_COMP_ASP_DEV] = {
		.name = "Cloverview Comp ASP",
		.stream_name = "Compress-Audio",
		.cpu_dai_name = "Compress-cpu-dai",
		.codec_dai_name = "rt5671-aif1",
		.codec_name = "rt5671.1-001c",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
		.compr_ops = &ctp_asp_compr_ops,
	},
	[CTP_COMMS_BT_SCO_DEV] = {
		.name = "Cloverview Comms BT SCO",
		.stream_name = "BTSCO",
		.cpu_dai_name = SSP_BT_DAI_NAME,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.platform_name = "mid-ssp-dai",
		.init = NULL,
		.ops = &ctp_comms_dai_link_ops,
	},
	[CTP_COMMS_MSIC_VOIP_DEV] = {
		.name = "Cloverview Comms MSIC VOIP",
		.stream_name = "VOIP",
		.cpu_dai_name = SSP_BT_DAI_NAME,
		.codec_dai_name = "rt5671-aif2",
		.codec_name = "rt5671.1-001c",
		.platform_name = "mid-ssp-dai",
		.init = NULL,
		.ops = &ctp_comms_voip_dai_link_ops,
	},
	[CTP_COMMS_IFX_MODEM_DEV] = {
		.name = "Cloverview Comms IFX MODEM",
		.stream_name = "IFX_MODEM_MIXING",
		.cpu_dai_name = SSP_MODEM_DAI_NAME,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.platform_name = "mid-ssp-dai",
		.init = NULL,
		.ops = &ctp_comms_dai_link_ops,
	},
	[CTP_AUD_VIRTUAL_ASP_DEV] = {
		.name = "Cloverview virtual-ASP",
		.stream_name = "virtual-stream",
		.cpu_dai_name = "Virtual-cpu-dai",
		.codec_dai_name = "rt5671-aif1",
		.codec_name = "rt5671.1-001c",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
		.ops = &ctp_asp_ops,
	},
	[CTP_RHB_AUD_PROBE_DEV] = {
		.name = "Cloverview Probe",
		.stream_name = "CTP Probe",
		.cpu_dai_name = "Probe-cpu-dai",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.platform_name = "sst-platform",
		.init = NULL,
		.ops = &ctp_probe_ops,
	},
};
int ctp_hp_detection(struct snd_soc_codec *codec,
			struct snd_soc_jack *jack, int enable)
{
#if defined(CONFIG_PF450CL)
	int plug_status = 0, hookdet_status = 0;
	int gpio_plugdet = 0, gpio_hookdet = 0;
#else
	int status = 0;
#endif

	int gpio_uart = 0;
	int ret = 0;

	pr_debug("%s enable:%d\n", __func__, enable);

/* FIXME 1. Due to UART debug and headset detection
 * are not well-defined in SR build
 * GPIO switch "SOC_DB_SW_INT" initial state is O(L):UART debug ;
 * remark O(H): headset detection temporarily
 * 2.  Due to GPIO name "SOC_DB_SW_INT" isn't ready in FW
 * GPIO number= GP_CORE_077 + 96(Shift)=173
 * remark hard code GPIO number temporarily.
 */
#if 0
	gpio_direction_output(173, 1);
#else
	gpio_uart = get_gpio_by_name("SOC_DB_SW_INT");
	if (gpio_uart > 0) {
		ret = gpio_request_one(gpio_uart,
			GPIOF_DIR_OUT, "SOC_DB_SW_INT");
		if (ret)
			pr_err("gpio_request SOC_DB_SW_INT failed!\n");

		gpio_direction_output(gpio_uart, 1);
		pr_info("SOC_DB_SW_INT value=%x\n", gpio_get_value(gpio_uart));
	} else
		pr_err("get_gpio SOC_DB_SW_INT failed!\n");
#endif

#if defined(CONFIG_PF450CL)
	gpio_plugdet = get_gpio_by_name("gpio_plugdet");
	if (gpio_plugdet > 0)
		plug_status = !gpio_get_value(gpio_plugdet);
	else
		pr_err("get_gpio gpio_plugdet failed!\n");

	pr_debug("%s : plug status: %x\n", __func__, plug_status);

	if (plug_status != 0) {
		snd_soc_update_bits(rt5671_codec, RT5671_PWR_ANLG2, RT5671_PWR_MB1, RT5671_PWR_MB1);
		msleep(50);

		gpio_hookdet = get_gpio_by_name("HOOK_DET");
		if (gpio_hookdet > 0) {
			hookdet_status = !gpio_get_value(gpio_hookdet);
			pr_debug("%s : hook status: %x\n", __func__, hookdet_status);
		} else
			pr_err("get_gpio HOOK_DET failed!\n");

		if (!gpio_get_value(gpio_hookdet)) {
			pr_info("%s headset\n", __func__);
			return SND_JACK_HEADSET;
		} else {
			pr_info("%s headphone\n", __func__);
			return SND_JACK_HEADPHONE;
		}
	} else {
	    snd_soc_update_bits(rt5671_codec, RT5671_PWR_ANLG2, RT5671_PWR_MB1, 0);
	    return 0;
	}
#else
	status = rt5671_headset_detect(codec, enable);
	pr_debug("%s : status: %x\n", __func__, status);

	if (SND_JACK_HEADSET == status)
		msleep(200);

	if (status == RT5671_HEADPHO_DET)
		return SND_JACK_HEADPHONE;
	else if (status == RT5671_HEADSET_DET)
		return SND_JACK_HEADSET;
#endif

	return 0;
}
int ctp_bp_detection(struct snd_soc_codec *codec,
			struct snd_soc_jack *jack, int enable)
{
	int status;
	pr_info("%s\n", __func__);

	if (enable)
		status = jack->status | SND_JACK_BTN_0;
	else
		status = jack->status & ~SND_JACK_BTN_0;
	pr_debug("%s, enable = 0x%x, status = 0x%x\n", __func__, enable, status);
	return status;
}
int ctp_dai_link(struct snd_soc_card *card)
{
	pr_info("%s\n", __func__);
	card->dai_link = ctp_rhb_dailink;
	card->num_links = ARRAY_SIZE(ctp_rhb_dailink);
	return 0;
}

static void ctp_rhb_card_name(struct snd_soc_card *card)
{
	card->name = "cloverview_audio";
}

struct snd_soc_machine_ops ctp_rhb_ops = {
	.card_name = ctp_rhb_card_name,
	.ctp_init = ctp_init,
	.dai_link = ctp_dai_link,
	.bp_detection = ctp_bp_detection,
	.hp_detection = ctp_hp_detection,
	.mclk_switch = NULL,
	.jack_support = true,
	.dmic3_support = false,
	.micsdet_debounce = MIC2SDET_DEBOUNCE_DELAY,
	.mic_bias = MICBIAS_NAME,
};
MODULE_DESCRIPTION("ASoC Intel(R) Cloverview MID Machine driver");
MODULE_AUTHOR("Jeeja KP<jeeja.kp@intel.com>");
MODULE_AUTHOR("Dharageswari R<dharageswari.r@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("ipc:ctprt5671-audio");
