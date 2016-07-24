/*
 *  ctp_comms_common.c - comms driver for Clovertrail MID platform
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

#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <asm/intel_sst_ctp.h>
#include "ctp_comms_common.h"
#include "ctp_common.h"
#include "../ssp/mid_ssp.h"

/* Data path functionalities */
struct snd_pcm_hardware BT_sco_hw_param = {
		.info = (SNDRV_PCM_INFO_INTERLEAVED |
				SNDRV_PCM_INFO_DOUBLE |
				SNDRV_PCM_INFO_PAUSE |
				SNDRV_PCM_INFO_RESUME |
				SNDRV_PCM_INFO_MMAP |
				SNDRV_PCM_INFO_MMAP_VALID |
				SNDRV_PCM_INFO_BATCH |
				SNDRV_PCM_INFO_SYNC_START),
		.formats = (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_U16_LE),
		.rates = (SNDRV_PCM_RATE_8000),
		.rate_min = 8000,
		.rate_max = 8000,
		.channels_min = 1,
		.channels_max = 1,
		.buffer_bytes_max = (320*1024),
		.period_bytes_min = 32,
		.period_bytes_max = (320*1024),
		.periods_min = 2,
		.periods_max = (1024*2),
		.fifo_size = 0,
};

struct snd_pcm_hardware FM_soc_hw_param = {
		.info = (SNDRV_PCM_INFO_INTERLEAVED |
				SNDRV_PCM_INFO_DOUBLE |
				SNDRV_PCM_INFO_PAUSE |
				SNDRV_PCM_INFO_RESUME |
				SNDRV_PCM_INFO_MMAP |
				SNDRV_PCM_INFO_MMAP_VALID |
				SNDRV_PCM_INFO_BATCH |
				SNDRV_PCM_INFO_SYNC_START),
		.formats = (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_U16_LE),
		.rates = (SNDRV_PCM_RATE_48000),
		.rate_min = 48000,
		.rate_max = 48000,
		.channels_min = 1,
		.channels_max = 2,
		.buffer_bytes_max = (640*1024),
		.period_bytes_min = 64,
		.period_bytes_max = (640*1024),
		.periods_min = 2,
		.periods_max = (1024*2),
		.fifo_size = 0,
};

/* Data path functionalities */
struct snd_pcm_hardware IFX_modem_alsa_hw_param = {
		.info = (SNDRV_PCM_INFO_INTERLEAVED |
				SNDRV_PCM_INFO_DOUBLE |
				SNDRV_PCM_INFO_PAUSE |
				SNDRV_PCM_INFO_RESUME |
				SNDRV_PCM_INFO_MMAP |
				SNDRV_PCM_INFO_MMAP_VALID |
				SNDRV_PCM_INFO_BATCH |
				SNDRV_PCM_INFO_SYNC_START),
		.formats = (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_U16_LE),
		.rates = (SNDRV_PCM_RATE_48000),
		.rate_min = 48000,
		.rate_max = 48000,
		.channels_min = 1,
		.channels_max = 2,
		.buffer_bytes_max = (640*1024),
		.period_bytes_min = 64,
		.period_bytes_max = (640*1024),
		.periods_min = 2,
		.periods_max = (1024*2),
		.fifo_size = 0,
};

/* Data path functionalities */
struct snd_pcm_hardware VOIP_alsa_hw_param = {
		.info = (SNDRV_PCM_INFO_INTERLEAVED |
				SNDRV_PCM_INFO_DOUBLE |
				SNDRV_PCM_INFO_PAUSE |
				SNDRV_PCM_INFO_RESUME |
				SNDRV_PCM_INFO_MMAP |
				SNDRV_PCM_INFO_MMAP_VALID |
				SNDRV_PCM_INFO_BATCH |
				SNDRV_PCM_INFO_SYNC_START),
		.formats = (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_U16_LE),
		.rates = (SNDRV_PCM_RATE_48000),
		.rate_min = 48000,
		.rate_max = 48000,
		.channels_min = 1,
		.channels_max = 2,
		.buffer_bytes_max = (640*1024),
		.period_bytes_min = 64,
		.period_bytes_max = (640*1024),
		.periods_min = 2,
		.periods_max = (1024*2),
		.fifo_size = 0,
};

/*
 * MIXER CONTROLS for SSP BT
 */
static const char * const ssp_master_mode_text[] = {"disabled", "enabled"};

static const struct soc_enum ssp_bt_sco_master_mode_enum =
	SOC_ENUM_SINGLE_EXT(2, ssp_master_mode_text);

static const struct soc_enum ssp_voip_master_mode_enum =
	SOC_ENUM_SINGLE_EXT(2, ssp_master_mode_text);

static const struct soc_enum ssp_modem_master_mode_enum =
	SOC_ENUM_SINGLE_EXT(2, ssp_master_mode_text);

int get_ssp_bt_sco_master_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card =  snd_kcontrol_chip(kcontrol);
	struct ctp_mc_private *ctx = snd_soc_card_get_drvdata(card);
	struct comms_mc_private *ctl = &(ctx->comms_ctl);

	ucontrol->value.integer.value[0] = ctl->ssp_bt_sco_master_mode;
	return 0;
}

int set_ssp_bt_sco_master_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card =  snd_kcontrol_chip(kcontrol);
	struct ctp_mc_private *ctx = snd_soc_card_get_drvdata(card);
	struct comms_mc_private *ctl = &(ctx->comms_ctl);

	if (ucontrol->value.integer.value[0] == ctl->ssp_bt_sco_master_mode)
		return 0;

	ctl->ssp_bt_sco_master_mode = ucontrol->value.integer.value[0];

	return 0;
}

int get_ssp_voip_master_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card =  snd_kcontrol_chip(kcontrol);
	struct ctp_mc_private *ctx = snd_soc_card_get_drvdata(card);
	struct comms_mc_private *ctl = &(ctx->comms_ctl);

	ucontrol->value.integer.value[0] = ctl->ssp_voip_master_mode;
	return 0;
}

int set_ssp_voip_master_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card =  snd_kcontrol_chip(kcontrol);
	struct ctp_mc_private *ctx = snd_soc_card_get_drvdata(card);
	struct comms_mc_private *ctl = &(ctx->comms_ctl);

	if (ucontrol->value.integer.value[0] == ctl->ssp_voip_master_mode)
		return 0;

	ctl->ssp_voip_master_mode = ucontrol->value.integer.value[0];

	return 0;
}

int get_ssp_modem_master_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card =  snd_kcontrol_chip(kcontrol);
	struct ctp_mc_private *ctx = snd_soc_card_get_drvdata(card);
	struct comms_mc_private *ctl = &(ctx->comms_ctl);

	ucontrol->value.integer.value[0] = ctl->ssp_modem_master_mode;
	return 0;
}

int set_ssp_modem_master_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card =  snd_kcontrol_chip(kcontrol);
	struct ctp_mc_private *ctx = snd_soc_card_get_drvdata(card);
	struct comms_mc_private *ctl = &(ctx->comms_ctl);

	if (ucontrol->value.integer.value[0] == ctl->ssp_modem_master_mode)
		return 0;

	ctl->ssp_modem_master_mode = ucontrol->value.integer.value[0];

	return 0;
}

const struct snd_kcontrol_new ssp_comms_controls[MAX_CONTROLS] = {
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
};


