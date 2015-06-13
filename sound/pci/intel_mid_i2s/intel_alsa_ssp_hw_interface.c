/*
 *   intel_alsa_ssp_hw_interface.c - Intel Sound card driver for SSP
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
#include <sound/pcm.h>

#include "intel_alsa_ssp_hw_interface.h"
#include "intel_alsa_ssp_hw_private.h"

/*****************************************/
/* Global Variables                      */
/*****************************************/

/**
 * mask to find easily in intel_alsa_ssp_open() if
 * the index correspond to the same device or not
 **/
static unsigned long stream_info_mask[2+1+2] = {
	BIT(0)|BIT(1), /* BT capture is BT: bit 0 & 1 */
	BIT(0)|BIT(1), /* BT Playback is BT: bit 0 & 1 */
	BIT(2), /* FM capture is FM: bit 2 */
	BIT(3)|BIT(4), /* BUILTIN capture is BUILTIN: bit 3 & 4 */
	BIT(3)|BIT(4) /* BUILTIN Playback is BUILTIN: bit 3 & 4 */
};

static struct intel_alsa_stream_status s_stream_status;

void intel_alsa_reset_ssp_status(void)
{
	s_stream_status.ssp_handle = NULL;
	s_stream_status.stream_info = 0;
	spin_lock_init(&s_stream_status.lock);
}

/*
 * intel_alsa_ssp_open - This function opens the requested stream
 * The Intel I2S driver is opened only if all streams has been closed
 *
 * Input parameters
 *		str_info : pointer to stream structure
 * Output parameters
 *		ret_val : status
 */
static int intel_alsa_ssp_open(struct intel_alsa_ssp_stream_info *str_info)
{
	int device_id, status;
	bool open_ssp = false;

	WARN(!str_info, "ALSA_SSP: NULL str_info\n");
	if (!str_info)
		return -EINVAL;

	device_id = str_info->device_id;

	str_info->dma_slot.period_req_index = 0;
	str_info->dma_slot.period_cb_index = 0;

	/*
	 * One Open Callback is called by sub-stream:
	 *  1 for BT Playback
	 *  1 for BT Capture
	 *  1 for FM Capture
	 *  1 for BUILTIN VOIP Playback
	 *  1 for BUILTIN VOIP Capture
	 *  The BUILTIN, FM and BT are exclusive, the 3 devices cannot be opened
	 *  in parallel
	 */

	pr_info("ALSA_SSP: call intel_mid_i2s_open for Device ID = %d\n",
			device_id);

	/*
	 * Algorithm that detects conflicting call of open /close
	 * The Open Call back is called for each device substream
	 * BT Capture, BT Playback and FM Capture
	 * However the Intel MID SSP provide a unique interface
	 * to open/configure and to close the SSP
	 * The aim of this function is to properly handle the concurrent
	 * Open/Close Callbacks
	 * The s_stream_status.stream_info is a bit field that indicated
	 * the stream status open or close
	 * BT Capture substream index = 0
	 * BT Play substream index = 1
	 * FM Capture substream index = 2
	 * BUILTIN Capture substream index = 3
	 * BUILTIN Play substream index = 4
	 */
	spin_lock(&s_stream_status.lock);
	if (s_stream_status.stream_info == 0) {
		/*
		 * No card or device opened
		 */
		open_ssp = true;
	} else if ((s_stream_status.stream_info
			& (~stream_info_mask[str_info->stream_index])) == 0) {
		/*
		 * We check that if we do not have other device opened, then
		 * do nothing because already Open by sibling substream
		 */
		pr_debug("ALSA SSP: Open DO NOTHING\n");

	} else {
		spin_unlock(&s_stream_status.lock);
		WARN(1, "ALSA SSP: Open unsupported Config\n");
		return -EBUSY;
	}
	set_bit(str_info->stream_index, &s_stream_status.stream_info);

	spin_unlock(&s_stream_status.lock);

	/*
	 * the Open SSP is performed out of lock
	 */
	if (open_ssp == true) {

		s_stream_status.ssp_handle =
				intel_mid_i2s_open(SSP_USAGE_BLUETOOTH_FM);

		intel_mid_i2s_command(s_stream_status.ssp_handle,
				SSP_CMD_SET_HW_CONFIG,
				&a_alsa_ssp_stream_settings[device_id]);

		/* Set the Write Callback */
		status = intel_mid_i2s_set_wr_cb(s_stream_status.ssp_handle,
				intel_alsa_ssp_dma_playback_complete);
		if (status)
			return -EINVAL;

		/* Set the Default Read Callback */
		status = intel_mid_i2s_set_rd_cb(s_stream_status.ssp_handle,
				intel_alsa_ssp_dma_capture_complete);
		if (status)
			return -EINVAL;
	}

	switch (str_info->substream->stream) {
	case SNDRV_PCM_STREAM_PLAYBACK:
		if (intel_mid_i2s_command(s_stream_status.ssp_handle,
				SSP_CMD_ALLOC_TX, NULL)) {
			pr_err("ALSA SSP: FCT %s Can not alloc TX DMA Channel\n",
					__func__);

			return -EBUSY;
		}
	break;

	case SNDRV_PCM_STREAM_CAPTURE:
		if (intel_mid_i2s_command(s_stream_status.ssp_handle,
				SSP_CMD_ALLOC_RX, NULL)) {
			pr_err("ALSA SSP: FCT %s Can not alloc RX DMA Channel\n",
					__func__);
			return -EBUSY;
		}
	break;

	default:
		WARN(1, "ALSA SSP: FCT %s Bad stream_dir: %d\n",
				__func__, str_info->substream->stream);
		return -EINVAL;
		break;
	}

	return 0;
} /* intel_alsa_ssp_open */

/*
 * intel_alsa_ssp_close - This function closes the requested stream
 * The Intel I2S driver is closed only if all streams are closed
 *
 * Input parameters
 *		@str_info : pointer to stream structure
 * Output parameters
 *		@ret_val : status
 */
static int intel_alsa_ssp_close(struct intel_alsa_ssp_stream_info *str_info)
{
	unsigned int device_id;
	bool close_ssp = false;

	WARN(!str_info, "ALSA_SSP: NULL str_info\n");
	if (!str_info)
		return -EINVAL;

	WARN(!s_stream_status.ssp_handle, "ALSA_SSP: ERROR, trying to close  "
					"a stream however ssp_handle is NULL\n");
	if (!s_stream_status.ssp_handle)
		return -EINVAL;

	device_id = str_info->device_id;

	/* Algorithm that detects conflicting call of open /close*/
	spin_lock(&s_stream_status.lock);

	if (s_stream_status.stream_info == 0) {
		spin_unlock(&s_stream_status.lock);
		WARN(1, "ALSA SSP: Close before Open\n");
		return -EBUSY;
	}

	clear_bit(str_info->stream_index, &s_stream_status.stream_info);

	if (s_stream_status.stream_info == 0)
		close_ssp = true;

	spin_unlock(&s_stream_status.lock);

	/*
	 * Release the DMA Channels
	 */

	switch (str_info->substream->stream) {

	case SNDRV_PCM_STREAM_PLAYBACK:
		intel_mid_i2s_command(s_stream_status.ssp_handle,
				SSP_CMD_FREE_TX, NULL);

		pr_debug("ALSA SSP: FCT %s TX DMA Channel released\n",
				__func__);
		break;

	case SNDRV_PCM_STREAM_CAPTURE:
		intel_mid_i2s_command(s_stream_status.ssp_handle,
				SSP_CMD_FREE_RX, NULL);
		pr_debug("ALSA SSP: FCT %s RX DMA Channel released\n",
				__func__);
		break;

	default:
		WARN(1, "ALSA SSP: FCT %s Bad stream_dir: %d\n",
				__func__, str_info->substream->stream);
		spin_unlock(&s_stream_status.lock);
		return -EINVAL;
		break;
	}

	/*
	 * the Close SSP is performed out of lock
	 */
	if (close_ssp) {
		intel_mid_i2s_close(s_stream_status.ssp_handle);
		s_stream_status.ssp_handle = NULL;
	}

	return 0;
} /* intel_alsa_ssp_close */

/*
 * intel_alsa_ssp_control - Set Control params
 *
 * Input parameters
 *		@command : command to execute
 *		@value: pointer to a structure
 * Output parameters
 *		@ret_val : status
 */
int intel_alsa_ssp_control(int command,
	struct intel_alsa_ssp_stream_info *str_info)
{
	int retval = 0;

	switch (command) {
	case INTEL_ALSA_SSP_CTRL_SND_OPEN:
		retval = intel_alsa_ssp_open(str_info);
		break;
	/*
	 * SND_PAUSE & SND_RESUME not supported in this version
	 */
	case INTEL_ALSA_SSP_CTRL_SND_PAUSE:
	case INTEL_ALSA_SSP_CTRL_SND_RESUME:
		break;

	case INTEL_ALSA_SSP_CTRL_SND_CLOSE:
		intel_alsa_ssp_close(str_info);
	  break;

	default:
		/* Illegal case */
		WARN(1, "ALSA_SSP: intel_alsa_ssp_control Error: Bad Control ID\n");
		return -EINVAL;
		break;
	}
	return retval;
} /* intel_alsa_ssp_control */

/*
 * intel_alsa_ssp_dma_playback_req - This function programs a write request
 * to the Intel I2S driver
 *
 * Input parameters
 *		@str_info : pointer to stream structure
 * Output parameters
 *		@ret_val : status
 */
static int intel_alsa_ssp_dma_playback_req(
				struct intel_alsa_ssp_stream_info *str_info)
{
	u32 *tx_addr;
	int tx_length;
	struct intel_alsa_ssp_dma_buf *sb_tx;

	WARN(!s_stream_status.ssp_handle, "ALSA_SSP: ERROR, trying to play"
					"a stream however ssp_handle is NULL\n");
	if (!s_stream_status.ssp_handle)
		return -EINVAL;

	sb_tx = &(str_info->dma_slot);
	tx_length = sb_tx->length;
	tx_addr = (u32 *)(sb_tx->addr + tx_length * sb_tx->period_req_index);
	pr_debug("ALSA_SSP: DMA PLAYBACK ADDRESS = 0x%p\n", tx_addr);

	if (intel_mid_i2s_wr_req(s_stream_status.ssp_handle,
			tx_addr, tx_length, str_info) == 0) {
		intel_mid_i2s_command(s_stream_status.ssp_handle,
				SSP_CMD_ENABLE_SSP,
				NULL);

		if (test_and_set_bit(INTEL_ALSA_SSP_STREAM_RUNNING, &str_info->stream_status)) {
			WARN_ON("ALSA SSP: ERROR previous requested "
					"not handled\n");
			return -EBUSY;
		}

		if (++(sb_tx->period_req_index) >= sb_tx->period_index_max)
			sb_tx->period_req_index = 0;
		return 0;
	} else {
		WARN(1, "ALSA_SSP: intel_mid_i2s_wr_req returns ERROR\n");
		return -EINVAL;
	}
} /* intel_alsa_ssp_dma_playback_req */

/*
 * intel_alsa_ssp_dma_capture_req - This function programs a read request
 * to the Intel I2S driver
 *
 * Input parameters
 *		@str_info : pointer to stream structure
 * Output parameters
 *		@ret_val : status
 */
static int intel_alsa_ssp_dma_capture_req(
				struct intel_alsa_ssp_stream_info *str_info)
{
	u32 *rx_addr;
	int rx_length;
	struct intel_alsa_ssp_dma_buf *sb_rx;


	WARN(!s_stream_status.ssp_handle, "ALSA_SSP: ERROR, trying to play "
					"a stream however ssp_handle is NULL\n");
	if (!s_stream_status.ssp_handle)
		return -EINVAL;

	sb_rx = &(str_info->dma_slot);

	rx_length = sb_rx->length;
	rx_addr = (u32 *) (sb_rx->addr + rx_length * sb_rx->period_req_index);

	pr_debug("ALSA_SSP: DMA CAPTURE ADDRESS = 0x%p\n",
			rx_addr);

	if (intel_mid_i2s_rd_req(s_stream_status.ssp_handle,
			rx_addr, rx_length, str_info) == 0) {

		intel_mid_i2s_command(s_stream_status.ssp_handle,
				SSP_CMD_ENABLE_SSP,
				NULL);

		if (test_and_set_bit(INTEL_ALSA_SSP_STREAM_RUNNING, &str_info->stream_status)) {
			WARN_ON("ALSA SSP: ERROR previous requested "
					"not handled\n");
			return -EBUSY;
		}

		if (++(sb_rx->period_req_index) >= sb_rx->period_index_max)
			sb_rx->period_req_index = 0;
		return 0;
	} else {
		WARN(1, "ALSA_SSP: intel_mid_i2s_rd_req returns ERROR\n");
		return -EINVAL;
	}
} /* intel_alsa_ssp_dma_capture_req */

/*
 * intel_alsa_ssp_transfer_data - send data buffers
 *
 * Input parameters
 *		@str_info : pointer to stream structure
 * Output parameters
 *		@ret_val : status
 */
void intel_alsa_ssp_transfer_data(struct work_struct *work)
{
	int status = 0;
	struct intel_alsa_ssp_stream_info *str_info;
	str_info =
		container_of(work, struct intel_alsa_ssp_stream_info, ssp_ws);

	WARN(!str_info, "ALSA SSP: ERROT NULL str_info\n");

	switch (str_info->substream->stream) {
	case SNDRV_PCM_STREAM_PLAYBACK:
		status = intel_alsa_ssp_dma_playback_req(str_info);
		break;

	case SNDRV_PCM_STREAM_CAPTURE:
		status = intel_alsa_ssp_dma_capture_req(str_info);
		break;

	default:
		WARN(1, "ALSA_SSP: FCT %s Bad stream_dir: %d\n",
				__func__, str_info->substream->stream);
		status = -EINVAL;
		break;
	}
} /* intel_alsa_ssp_transfer_data */

/*
 * intel_alsa_ssp_dma_playback_complete - End of playback callback
 * called in DMA Complete Tasklet context
 * This Callback has in charge of re-programming a new write request to
 * Intel MID I2S Driver if the stream has not been Closed.
 * It calls also the snd_pcm_period_elapsed if the stream is not
 * PAUSED or SUSPENDED to inform ALSA Kernel that the Ring Buffer
 * period has been sent properly
 *
 * Input parameters
 *		@param : pointer to a structure
 * Output parameters
 *		@ret_val : status
 */
int intel_alsa_ssp_dma_playback_complete(void *param)
{
	struct intel_alsa_ssp_stream_info *str_info;
	struct intel_alsa_ssp_dma_buf *sb_tx;
	bool call_back = false;
	bool reset_index = false;

	WARN(!param, "ALSA SSP: ERROR param NULL\n");
	if (!param)
		return -EBUSY;
	str_info = param;
	sb_tx = &(str_info->dma_slot);

	if (test_and_clear_bit(INTEL_ALSA_SSP_STREAM_RUNNING, &str_info->stream_status)) {
		if (test_and_clear_bit(INTEL_ALSA_SSP_STREAM_DROPPED, &str_info->stream_status)) {
			if (test_bit(INTEL_ALSA_SSP_STREAM_STARTED, &str_info->stream_status)) {
				/*
				 * the stream has been dropped and restarted
				 * before the callback occurs
				 * in this case the we have to reprogram the
				 * requests to SSP driver and reset
				 * the stream's indexes
				 */
				call_back = true;
				reset_index = true;
			} else
				call_back = false;
		} else {
			if (test_bit(INTEL_ALSA_SSP_STREAM_STARTED, &str_info->stream_status)) {
				/*
				 * the stream is on going
				 */
				call_back = true;
			} else {
				WARN(1, "ALSA_SSP: FCT %s spurious playback"
					"DMA complete 1 ?!\n",
					__func__);
				return -EBUSY;
			}
		}
	} else {
		WARN(1, "ALSA_SSP: FCT %s spurious playback"
			"DMA complete 2 ?!\n",
			__func__);
		return -EBUSY;
	}

	if (call_back == true) {
		pr_debug("ALSA_SSP: playback (REQ=%d,CB=%d): PLAYBACK_DMA_REQ_COMPLETE\n",
				sb_tx->period_req_index,
				sb_tx->period_cb_index);

		if (reset_index) {
			sb_tx->period_cb_index = 0;
			sb_tx->period_req_index = 0;
		} else if (++(sb_tx->period_cb_index) >= sb_tx->period_index_max)
			sb_tx->period_cb_index = 0;

		/*
		 * Launch the next Playback request if no
		 * CLOSE has been requested
		 */
		intel_alsa_ssp_dma_playback_req(str_info);

		/*
		* Call the snd_pcm_period_elapsed to inform ALSA kernel
		* that a ring buffer period has been played
		*/
		snd_pcm_period_elapsed(str_info->substream);

	}
	return 0;
} /* intel_alsa_ssp_dma_playback_complete */

/*
 * intel_alsa_ssp_dma_capture_complete - End of capture callback
 * called in DMA Complete Tasklet context
 * This Callback has in charge of re-programming a new read request to
 * Intel MID I2S Driver if the stream has not been Closed.
 * It calls also the snd_pcm_period_elapsed if the stream is not
 * PAUSED or SUSPENDED to inform ALSA Kernel that the Ring Buffer
 * period has been received properly
 *
 * Input parameters
 *		@param : pointer to a structure
 * Output parameters
 *		@ret_val : status
 */
int intel_alsa_ssp_dma_capture_complete(void *param)
{
	struct intel_alsa_ssp_dma_buf *sb_rx;
	struct intel_alsa_ssp_stream_info *str_info;
	bool call_back = false;
	bool reset_index = false;

	WARN(!param, "ALSA SSP: ERROR param NULL\n");
	if (!param)
		return -EBUSY;

	str_info = param;
	sb_rx = &(str_info->dma_slot);

	if (test_and_clear_bit(INTEL_ALSA_SSP_STREAM_RUNNING, &str_info->stream_status)) {
			if (test_and_clear_bit(INTEL_ALSA_SSP_STREAM_DROPPED, &str_info->stream_status)) {
				if (test_bit(INTEL_ALSA_SSP_STREAM_STARTED, &str_info->stream_status)) {
					/*
					 * the stream has been dropped and
					 * restarted before the callback occurs
					 * in this case the we have to reprogram
					 * the requests to SSP driver and
					 * reset the stream's indexes
					 */
					call_back = true;
					reset_index = true;
				} else
					call_back = false;
			} else {
				if (test_bit(INTEL_ALSA_SSP_STREAM_STARTED, &str_info->stream_status)) {
					/*
					 * the stream is on going
					 */
					call_back = true;
				} else {
					WARN(1, "ALSA_SSP: FCT %s spurious"
						"playback DMA complete 1 ?!\n",
						__func__);
					return -EBUSY;
				}
			}
		} else {
			WARN(1, "ALSA_SSP: FCT %s spurious playback DMA complete 2 ?!\n",
					__func__);
			return -EBUSY;
		}

	if (call_back == true) {
		pr_debug("ALSA_SSP: playback (REQ=%d,CB=%d): PLAYBACK_DMA_REQ_COMPLETE\n",
				sb_rx->period_req_index,
				sb_rx->period_cb_index);

		if (reset_index) {
			sb_rx->period_cb_index = 0;
			sb_rx->period_req_index = 0;
		} else if (++(sb_rx->period_cb_index) >= sb_rx->period_index_max)
			sb_rx->period_cb_index = 0;

		/*
		 * Launch the next Playback request if no
		 * CLOSE has been requested
		 */
		intel_alsa_ssp_dma_capture_req(str_info);
		/*
		 * Call the snd_pcm_period_elapsed to inform ALSA
		 * kernel that a ring buffer period has been played
		 */
		snd_pcm_period_elapsed(str_info->substream);

	}
	return 0;
} /* intel_alsa_ssp_dma_capture_complete */
