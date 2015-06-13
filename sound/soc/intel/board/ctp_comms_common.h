/*
 *  ctp_comms_common.h - Common routines for the BT/voip/voice
 *
 *  Copyright (C) 2011-13 Intel Corp
 *  Author: Selma Bensaid <selma.bensaid@intel.com>
 *  Author: Dharageswari.R <dharageswari.r@intel.com>
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
#ifndef _CTP_COMMS_COMMON_H
#define _CTP_COMMS_COMMON_H

#define MAX_CONTROLS	3
/*
 * For FM 1 slot of 32 bits is used
 * to transfer stereo 16 bits PCM samples
 */
#define SSP_FM_SLOT_NB_SLOT		1
#define SSP_FM_SLOT_WIDTH		32
#define SSP_FM_SLOT_RX_MASK		0x1
#define SSP_FM_SLOT_TX_MASK		0x0
/*
 * For BT SCO 1 slot of 16 bits is used
 * to transfer mono 16 bits PCM samples
 */
#define SSP_BT_SLOT_NB_SLOT		1
#define SSP_BT_SLOT_WIDTH		16
#define SSP_BT_SLOT_RX_MASK		0x1
#define SSP_BT_SLOT_TX_MASK		0x1

/*
 * For VoIP 1 slot of 32 bits is used
 * to transfer stereo 16 bits PCM samples
 */
#define SSP_VOIP_SLOT_NB_SLOT	1
#define SSP_VOIP_SLOT_WIDTH		32
#define SSP_VOIP_SLOT_RX_MASK	0x1
#define SSP_VOIP_SLOT_TX_MASK	0x1

/*
 * For Modem IFX 1 slot of 32 bits is used
 * to transfer stereo 16 bits PCM samples
 */
#define SSP_IFX_SLOT_NB_SLOT	1
#define SSP_IFX_SLOT_WIDTH		32
#define SSP_IFX_SLOT_RX_MASK	0x1
#define SSP_IFX_SLOT_TX_MASK	0x1

extern struct snd_pcm_hardware BT_sco_hw_param;
extern struct snd_pcm_hardware FM_soc_hw_param;
extern struct snd_pcm_hardware IFX_modem_alsa_hw_param;
extern struct snd_pcm_hardware VOIP_alsa_hw_param;
extern const struct snd_kcontrol_new ssp_comms_controls[MAX_CONTROLS];
#endif
