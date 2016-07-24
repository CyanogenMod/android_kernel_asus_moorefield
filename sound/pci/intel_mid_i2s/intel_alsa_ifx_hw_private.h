/*
 *	intel_alsa_ssp_hw_private.h
 *
 *  Copyright (C) 2010 Intel Corp
 *  Authors:	Selma Bensaid <selma.bensaid@intel.com>
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

#ifndef INTEL_ALSA_SSP_HW_PRIVATE_H_
#define INTEL_ALSA_SSP_HW_PRIVATE_H_

#include <sound/core.h>
#include <linux/intel_mid_i2s_if.h>
#include <sound/intel_alsa_ssp_common.h>

const struct intel_mid_i2s_settings
	a_alsa_ifx_stream_settings[INTEL_ALSA_IFX_SND_CARD_MAX_DEVICES] = {
		{
#ifdef CONFIG_SND_INTEL_ALSA_SSP_MODEM_MASTER_MODE
		  .mode = SSP_IN_NETWORK_MODE,
		  .rx_fifo_interrupt = SSP_RX_FIFO_OVER_INT_ENABLE,
		  .tx_fifo_interrupt = SSP_TX_FIFO_UNDER_INT_ENABLE,
		  .frame_format = PSP_FORMAT,
		  .master_mode_clk_selection = SSP_ONCHIP_CLOCK,
		  .frame_rate_divider_control = 1,
		  .master_mode_standard_freq = SSP_FRM_FREQ_48_000,
		  .data_size = 32,
		  .tx_tristate_phase = TXD_TRISTATE_LAST_PHASE_OFF,
		  .tx_tristate_enable = TXD_TRISTATE_ON,
		  .slave_clk_free_running_status =
					SLAVE_SSPCLK_ON_DURING_TRANSFER_ONLY,
		  .sspslclk_direction = SSPSCLK_MASTER_MODE,
		  .sspsfrm_direction = SSPSFRM_MASTER_MODE,
		  .ssp_duplex_mode = RX_WITHOUT_TX_MODE,
		  .ssp_trailing_byte_mode = SSP_TRAILING_BYTE_HDL_BY_IA,
		  .ssp_tx_dma = SSP_TX_DMA_ENABLE,
		  .ssp_rx_dma = SSP_RX_DMA_ENABLE,
		  .ssp_rx_timeout_interrupt_status = SSP_RX_TIMEOUT_INT_DISABLE,
		  .ssp_trailing_byte_interrupt_status =
					SSP_TRAILING_BYTE_INT_DISABLE,
		  .ssp_loopback_mode_status = SSP_LOOPBACK_OFF,
		  .ssp_rx_fifo_threshold = 8,
		  .ssp_tx_fifo_threshold = 7,
		  .ssp_frmsync_timing_bit = NEXT_FRMS_ASS_AFTER_END_OF_T4,
		  .ssp_frmsync_pol_bit = SSP_FRMS_ACTIVE_LOW,
		  .ssp_end_transfer_state = SSP_END_DATA_TRANSFER_STATE_LOW,
		  .ssp_serial_clk_mode = SSP_CLK_MODE_0,
		  .ssp_psp_T1 = 6,
		  .ssp_psp_T2 = 2,
		  .ssp_psp_T4 = 0,
		  .ssp_psp_T5 = 14,
		  .ssp_psp_T6 = 16,
		  .ssp_active_tx_slots_map = 0x01,
		  .ssp_active_rx_slots_map = 0x01,
#else
		  .mode = SSP_IN_NETWORK_MODE,
		  .rx_fifo_interrupt = SSP_RX_FIFO_OVER_INT_ENABLE,
		  .tx_fifo_interrupt = SSP_TX_FIFO_UNDER_INT_ENABLE,
		  .frame_format = PSP_FORMAT,
		  .master_mode_clk_selection = SSP_MASTER_CLOCK_UNDEFINED,
		  .frame_rate_divider_control = 1,
		  .master_mode_standard_freq = SSP_FRM_FREQ_UNDEFINED,
		  .data_size = 32,
		  .tx_tristate_phase = TXD_TRISTATE_LAST_PHASE_OFF,
		  .tx_tristate_enable = TXD_TRISTATE_ON,
		  .slave_clk_free_running_status = SLAVE_SSPCLK_ON_ALWAYS,
		  .sspslclk_direction = SSPSCLK_SLAVE_MODE,
		  .sspsfrm_direction = SSPSFRM_SLAVE_MODE,
		  .ssp_duplex_mode = RX_AND_TX_MODE,
		  .ssp_trailing_byte_mode = SSP_TRAILING_BYTE_HDL_BY_IA,
		  .ssp_tx_dma = SSP_TX_DMA_ENABLE,
		  .ssp_rx_dma = SSP_RX_DMA_ENABLE,
		  .ssp_rx_timeout_interrupt_status = SSP_RX_TIMEOUT_INT_DISABLE,
		  .ssp_trailing_byte_interrupt_status =
					SSP_TRAILING_BYTE_INT_DISABLE,
		  .ssp_loopback_mode_status = SSP_LOOPBACK_OFF,
		  .ssp_rx_fifo_threshold = 8,
		  .ssp_tx_fifo_threshold = 7,
		  .ssp_frmsync_timing_bit = NEXT_FRMS_ASS_WITH_LSB_PREVIOUS_FRM,
		  .ssp_frmsync_pol_bit = SSP_FRMS_ACTIVE_HIGH,
		  .ssp_end_transfer_state = SSP_END_DATA_TRANSFER_STATE_LOW,
		  .ssp_serial_clk_mode = SSP_CLK_MODE_0,
		  .ssp_psp_T1 = 0,
		  .ssp_psp_T2 = 0,
		  .ssp_psp_T4 = 0,
		  .ssp_psp_T5 = 0,
		  .ssp_psp_T6 = 1,
		  .ssp_active_tx_slots_map = 0x01,
		  .ssp_active_rx_slots_map = 0x01
#endif
		}
};

#endif /* INTEL_ALSA_SSP_HW_PRIVATE_H_ */
