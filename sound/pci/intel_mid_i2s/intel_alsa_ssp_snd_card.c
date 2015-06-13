/*
 *   intel_alsa_ssp_snd_card.c - Intel Sound card driver for SSP
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
 * ALSA driver for SSP sound card
 * This Driver provides a sound card with 2 PCM devices:
 * - a BT device which offer a capture and a playback stream
 * - an FM device which offer a capture stream
 * The ALSA SSP Driver interfaces with the Intel MID SSP Driver to
 * send and receive the PCM samples between the Penwell SSP peripheral
 * and the BT/PCM chipset
 *
 */


#include "intel_alsa_ssp_snd_card.h"

MODULE_AUTHOR("Selma Bensaid <selma.bensaid@intel.com>");
MODULE_DESCRIPTION("Intel ALSA SSP Sound Card Driver");
MODULE_LICENSE("GPL v2");
MODULE_SUPPORTED_DEVICE("{Intel,INTEL_ALSA_SSP_SND_CARD");
MODULE_VERSION(INTEL_ALSA_SSP_DRIVER_VERSION);


static struct intel_alsa_ssp_card_info *p_alsa_ssp_snd_card;

/* Index of sound card will be allocated by the kernel */
static int v_intel_alsa_ssp_card_index = SNDRV_DEFAULT_IDX1;
static char *p_intel_alsa_ssp_card_id = SNDRV_DEFAULT_STR1;

module_param(v_intel_alsa_ssp_card_index, int, 0444);
MODULE_PARM_DESC(v_intel_alsa_ssp_card_index,
		"Index value for ALSA SSP PCM soundcard.");
module_param(p_intel_alsa_ssp_card_id, charp, 0444);
MODULE_PARM_DESC(p_intel_alsa_ssp_card_id,
		"ID string ALSA SSP PCM soundcard.");


/*
 * snd_i2s_alsa_open- to set runtime parameters during stream start
 * This function is called by ALSA framework when stream is started
 *
 * Input parameters
 *		@substream:  substream for which the function is called
 *
 * Output parameters
 *		@ret_val : status, 0 ==> OK
 *
 */
int snd_i2s_alsa_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *str_runtime;
	struct intel_alsa_ssp_stream_info *str_info;
	int ret_value, i;
	unsigned int device;

	ret_value = 0;

	WARN(!substream, "ALSA_SSP: ERROR NULL substream\n");
	if (!substream)
		return -EINVAL;

	WARN(!substream->runtime, "ALSA_SSP: ERROR NULL substream->runtime\n");
	if (!substream->runtime)
		return -EINVAL;
	str_runtime = substream->runtime;

	WARN(!substream->pcm, "ALSA_SSP: ERROR NULL substream->pcm\n");
	if (!substream->pcm)
		return -EINVAL;

	device = substream->pcm->device;

	pr_debug("ALSA_SSP: FCT %s snd pcm device %d\n",
			__func__,
			substream->pcm->device);

	/* set the runtime hw parameter with local snd_pcm_hardware struct */
	str_runtime->hw = *(s_dev_info[device].stream_hw_param);

	/*
	 * setup the internal data structure stream pointers based on it being
	 * playback or capture stream
	 */
	str_info = kzalloc(sizeof(*str_info), GFP_KERNEL);

	if (!str_info)
		return -ENOMEM;

	str_info->device_id = device;
	str_info->stream_status = 0;
	str_info->stream_index = 0;

	for (i = 0; i < device; i++)
		str_info->stream_index += s_dev_info[i].nb_capt_stream +
		s_dev_info[i].nb_play_stream;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		str_info->stream_index++;

	INIT_WORK(&str_info->ssp_ws, intel_alsa_ssp_transfer_data);

	/* Initialize SSP1 driver */
	/* Store the Stream information */
	str_runtime->private_data = str_info;

	str_info->substream = substream;

	ret_value = intel_alsa_ssp_control(INTEL_ALSA_SSP_CTRL_SND_OPEN,
			str_info);

	if (ret_value)
		return -EBUSY;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		p_alsa_ssp_snd_card->playback_cnt++;
	else
		p_alsa_ssp_snd_card->capture_cnt++;

	return snd_pcm_hw_constraint_integer(str_runtime,
			SNDRV_PCM_HW_PARAM_PERIODS);
} /* snd_i2s_alsa_open */

/*
 * snd_i2s_alsa_close- to free parameters when stream is stopped
 * This function is called by ALSA framework when stream is stopped
 *
 * Input parameters
 *		@substream:  substream for which the function is called
 *
 * Output parameters
 *		@ret_val : status, 0 ==> OK
 *
 */
int snd_i2s_alsa_close(struct snd_pcm_substream *substream)
{
	struct intel_alsa_ssp_stream_info *str_info;
	int ret_val = 0;

	WARN(!substream, "ALSA_SSP: ERROR NULL substream\n");
	if (!substream)
		return -EINVAL;

	WARN(!substream->runtime, "ALSA_SSP: ERROR NULL substream->runtime\n");
	if (!substream->runtime)
		return -EINVAL;

	str_info = substream->runtime->private_data;

	if (str_info) {
		/* Cancel pending work */
		cancel_work_sync(&str_info->ssp_ws);

		/* SST API to actually stop/free the stream */
		ret_val = intel_alsa_ssp_control(INTEL_ALSA_SSP_CTRL_SND_CLOSE,
				str_info);

		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			p_alsa_ssp_snd_card->playback_cnt--;
		else
			p_alsa_ssp_snd_card->capture_cnt--;
	} else {
		WARN(1, "ALSA_SSP: ERROR p_stream_info is NULL\n");
		return -EINVAL;
	}
	pr_debug("ALSA SSP CLOSE Stream Direction = %d\n",
			substream->stream);

	pr_debug("ALSA SSP CLOSE: Playback cnt = %d Capture cnt = %d\n",
			p_alsa_ssp_snd_card->playback_cnt,
			p_alsa_ssp_snd_card->capture_cnt);

	kfree(substream->runtime->private_data);

	return ret_val;
} /* snd_i2s_alsa_close */

/*
 * snd_i2s_alsa_pcm_trigger- stream activities are handled here
 * This function is called whenever a stream activity is invoked
 * The Trigger function is called in an atomic context
 *
 * Input parameters
 *		@substream:substream for which the stream function is called
 *		@cmd:the stream command thats requested from upper layer
 *
 * Output parameters
 *		@ret_val : status, 0 ==> OK
 *
 */
int snd_i2s_alsa_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	int ret_val = 0;
	struct intel_alsa_ssp_stream_info *str_info;
	struct snd_pcm_runtime *pl_runtime;
	struct intel_alsa_ssp_dma_buf *pl_dma_buf;
	bool trigger_start = true;

	WARN(!substream, "ALSA_SSP: ERROR NULL substream\n");
	if (!substream)
		return -EINVAL;

	WARN(!substream->runtime, "ALSA_SSP: ERROR NULL substream->runtime\n");
	if (!substream->runtime)
		return -EINVAL;
	pl_runtime = substream->runtime;

	WARN(!pl_runtime->private_data,
			"ALSA_SSP: ERROR NULL pl_runtime->private_data\n");
	if (!pl_runtime->private_data)
		return -EINVAL;

	str_info = pl_runtime->private_data;

	pl_dma_buf = &(str_info->dma_slot);
	pr_info("ALSA_SSP:  snd_i2s_alsa_pcm_trigger CMD = 0x%04X\n", cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_RESUME:
		pr_info("ALSA_SSP:  SNDRV_PCM_TRIGGER_START for device %d\n",
				str_info->device_id);
		pr_info("ALSA_SSP: Trigger Start period_size =%ld\n",
				pl_runtime->period_size);

		if (!test_and_set_bit(INTEL_ALSA_SSP_STREAM_STARTED,
					&str_info->stream_status)) {
			if (test_bit(INTEL_ALSA_SSP_STREAM_DROPPED,
					     &str_info->stream_status)) {
				pr_debug("ALSA SSP: Do not restart the trigger, stream running already\n");
				trigger_start = false;
			} else
				trigger_start = true;
		} else {
			WARN(1, "ALSA SSP: ERROR 2 conscutive TRIGGER_START\n");
			return -EBUSY;
		}

		/* Store the substream locally */
		if (trigger_start) {

			pl_dma_buf->length = frames_to_bytes(pl_runtime,
					pl_runtime->period_size);
			pl_dma_buf->addr = pl_runtime->dma_area;
			pl_dma_buf->period_index_max = pl_runtime->periods;

			queue_work(p_alsa_ssp_snd_card->ssp_wq,
					&str_info->ssp_ws);
		}

		str_info->dbg_cum_bytes += frames_to_bytes(substream->runtime,
				substream->runtime->period_size);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		pr_debug("ALSA_SSP:  SNDRV_PCM_TRIGGER_STOP\n");

		if (test_and_clear_bit(INTEL_ALSA_SSP_STREAM_STARTED,
				&str_info->stream_status))
			set_bit(INTEL_ALSA_SSP_STREAM_DROPPED,
					&str_info->stream_status);
		else {
			WARN(1, "ALSA SSP: trigger START/STOP mismatch\n");
			return -EBUSY;
		}
		break;

	default:
		WARN(1, "ALSA_SSP: snd_i2s_alsa_pcm_trigger Bad Command\n");
		return -EINVAL;
		break;
	}
	return ret_val;
}

/**
 * snd_i2s_alsa_hw_params - Allocate memory for Ring Buffer according
 * to hw_params.
 * It's called in a non-atomic context
 *
 * * Input parameters
 *		@substream:substream for which the stream function is called
 *		@hw_params: stream command thats requested from upper layer
 *
 * Output parameters
 *		@ret_val : status, 0 ==> OK
 *
 */
int snd_i2s_alsa_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *hw_params)
{
	int ret_val;

	WARN(!substream, "ALSA_SSP: ERROR NULL substream\n");
	if (!substream)
		return -EINVAL;

	/*
	 * Allocates the DMA buffer for the substream
	 * This callback could be called several time
	 * snd_pcm_lib_malloc_pages allows to avoid memory leak
	 * as it release already allocated memory when already allocated
	 */
	ret_val = snd_pcm_lib_malloc_pages(substream,
			params_buffer_bytes(hw_params));

	memset(substream->runtime->dma_area, 0, params_buffer_bytes(hw_params));

	return ret_val;
}

/**
 * snd_i2s_alsa_hw_free - Called to release ressources allocated by
 * snd_i2s_alsa_hw_params
 * It's called in a non-atomic context
 *
 * * Input parameters
 *		@substream:substream for which the stream function is called
 *
 * Output parameters
 *		@ret_val : status, 0 ==> OK
 *
 */
int snd_i2s_alsa_hw_free(struct snd_pcm_substream *substream)
{
	WARN(!substream, "ALSA_SSP: ERROR NULL substream\n");
	if (!substream)
		return -EINVAL;

	return snd_pcm_lib_free_pages(substream);
}

/*
 * snd_i2s_alsa_pcm_pointer- to send the current buffer pointer
 * processed by HW
 * This function is called by ALSA framework to get the current HW buffer ptr
 * to check the Ring Buffer Status
 *
 * Input parameters
 *		@substream: pointer to the substream for which the function
 *			    is called
 *
 * Output parameters
 *		@pcm_pointer: indicates the number of samples played
 *
 */
snd_pcm_uframes_t snd_i2s_alsa_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct intel_alsa_ssp_stream_info *str_info;
	struct intel_alsa_ssp_dma_buf *dma_info;
	unsigned long pcm_pointer = 0;

	WARN(!substream, "ALSA_SSP: ERROR NULL substream\n");
	if (!substream)
		return -EINVAL;

	WARN(!substream->runtime, "ALSA_SSP: ERROR NULL substream->runtime\n");
	if (!substream->runtime)
		return -EINVAL;
	str_info = substream->runtime->private_data;

	WARN(!str_info, "ALSA_SSP: ERROR NULL str_info\n");
	if (!str_info)
		return -EINVAL;
	dma_info = &(str_info->dma_slot);

	pcm_pointer = (unsigned long) (dma_info->period_cb_index
					* substream->runtime->period_size);

	pr_debug("ALSA_SSP: Frame bits:: %d period_size :: %d periods :: %d\n",
	     (int) substream->runtime->frame_bits,
	     (int) substream->runtime->period_size,
	     (int) substream->runtime->periods);

	pr_debug("ALSA_SSP: snd_i2s_alsa_pcm_pointer returns %ld\n",
			pcm_pointer);

	return pcm_pointer;
}

/*
 * snd_i2s_alsa_pcm_prepare- internal preparation before starting a stream
 * This function is called when a stream is started for internal preparation
 *
 * Input parameters
 *		@substream: substream for which the function is called
 * Output parameters
 *		NA
 */
int snd_i2s_alsa_pcm_prepare(struct snd_pcm_substream *substream)
{
	return 0;
}

/**
 *
 * Card/Device Functions
 *
 */

/*
 * intel_alsa_create_pcm_device : to setup pcm for the card
 *
 * Input parameters
 *		@card : pointer to the sound card structure
 *		@p_alsa_ssp_snd_card : pointer to internal context
 *
 * Output parameters
 *		@ret_val : status, 0 ==> OK
 *
 */
static int intel_alsa_create_pcm_device(struct snd_card *card,
			struct intel_alsa_ssp_card_info *p_alsa_ssp_snd_card)
{
	struct snd_pcm *pl_pcm;
	int i, ret_val = 0;

	WARN(!card, "ALSA_SSP: ERROR NULL card\n");
	if (!card)
		return -EINVAL;

	WARN(!p_alsa_ssp_snd_card,
				"ALSA_SSP: ERROR NULL p_alsa_ssp_snd_card\n");
	if (!p_alsa_ssp_snd_card)
		return -EINVAL;

	/*
	 * The alsa_ssp driver handles provide 2 PCM devices :
	 * device 0 ==> BT with 1 capture sub-stream + 1 play sub-stream
	 * device 1 ==> FM with 1 capture sub-stream + 0 play sub-stream
	 * These 2 devices are exclusive
	 */
	for (i = 0; i < INTEL_ALSA_SSP_SND_CARD_MAX_DEVICES; i++) {

		ret_val = snd_pcm_new(card, s_dev_info[i].dev_name, i,
				s_dev_info[i].nb_play_stream,
				s_dev_info[i].nb_capt_stream,
				&pl_pcm);

		if (ret_val)
			return ret_val;

		/* setup the ops for playback and capture streams */
		snd_pcm_set_ops(pl_pcm, SNDRV_PCM_STREAM_PLAYBACK,
					&s_alsa_ssp_playback_ops);
		snd_pcm_set_ops(pl_pcm, SNDRV_PCM_STREAM_CAPTURE,
					&s_alsa_ssp_capture_ops);

		/* setup private data which can be retrieved when required */
		pl_pcm->private_data = p_alsa_ssp_snd_card;
		pl_pcm->info_flags = 0;

		strncpy(pl_pcm->name, card->shortname, strlen(card->shortname));

		/*
		 * allocate DMA pages for ALSA stream operations pre-allocation
		 * to all substreams of the given pcm for the specified DMA type
		 */
		snd_pcm_lib_preallocate_pages_for_all(pl_pcm,
			SNDRV_DMA_TYPE_CONTINUOUS,
			snd_dma_continuous_data
			(GFP_KERNEL),
			s_dev_info[i].stream_hw_param->buffer_bytes_max,
			s_dev_info[i].stream_hw_param->buffer_bytes_max);
	}

	return ret_val;
}

/**
 * intel_alsa_create_snd_card : function which creates sound card and PCM
 * device
 *
 * Input parameters
 *		NA
 *
 * Output parameters
 *		@ret_val : status, 0 ==> OK
 *
 */
int intel_alsa_create_snd_card(void)
{
	struct snd_card *card;
	int ret_val;

	/*
	 * Allocate the global structure of the Driver p_alsa_ssp_snd_card
	 */
	p_alsa_ssp_snd_card = kzalloc(sizeof(*p_alsa_ssp_snd_card), GFP_KERNEL);

	if (!p_alsa_ssp_snd_card) {
		WARN(1, "ALSA SSP ERROR: malloc fail\n");
		return -ENOMEM;
	}

	/*
	 * create a card instance with ALSA framework
	 */
	ret_val = snd_card_create(v_intel_alsa_ssp_card_index,
			p_intel_alsa_ssp_card_id, THIS_MODULE, 0, &card);

	if (ret_val) {
		pr_debug("ALSA SSP ERROR: snd_card_create fail\n");
		goto free_allocs;
	}

	p_alsa_ssp_snd_card->card = card;
	p_alsa_ssp_snd_card->card_id = p_intel_alsa_ssp_card_id;
	p_alsa_ssp_snd_card->playback_cnt =
			p_alsa_ssp_snd_card->capture_cnt = 0;

	p_alsa_ssp_snd_card->ssp_wq = create_workqueue("ssp_transfer_data");
	if (!p_alsa_ssp_snd_card->ssp_wq) {
		kfree(p_alsa_ssp_snd_card);
		pr_err("ALSA_SSP: Work queue failed\n");
		return -ENOMEM;
	}

	strncpy(card->driver, INTEL_ALSA_SSP_CARD_NAME,
			strlen(INTEL_ALSA_SSP_CARD_NAME));

	strncpy(card->shortname, INTEL_ALSA_SSP_CARD_NAME,
			strlen(INTEL_ALSA_SSP_CARD_NAME));

	ret_val = intel_alsa_create_pcm_device(card, p_alsa_ssp_snd_card);
	if (ret_val) {
		pr_err("ALSA SSP ERROR: failed to allocate p_alsa_ssp_snd_card\n");
		goto free_allocs;
	}

	card->private_data = &p_alsa_ssp_snd_card;

	ret_val = snd_card_register(card);

	if (ret_val) {
		pr_err("ALSA SSP: snd_card_register failed\n");
		goto free_allocs;
	}

	/*
	 * Reset Internal parameters
	 */
	intel_alsa_reset_ssp_status();

	return ret_val;

free_allocs:
	snd_card_free(card);
	kfree(p_alsa_ssp_snd_card);
	WARN(1, "ALSA SSP ERROR: Intel ALSA SSP Card Creation failed\n");
	return ret_val;
} /* intel_alsa_create_snd_card */

/**
 * intel_alsa_remove_snd_card : function which removes the sound card and PCM
 * devices to the SSP driver Interface
 * This function is registered for exit and it's called when the device is
 * uninitialized
 *
 * Input parameters
 *		NA
 * Output parameters
 *		@ret_val : status, 0 ==> OK
 */
int intel_alsa_remove_snd_card(void)
{
	int ret_val;

	if (p_alsa_ssp_snd_card) {
		pr_debug("ALSA_SSP: p_alsa_ssp_snd_card: 0x%08X\n",
				(u32) p_alsa_ssp_snd_card);
		destroy_workqueue(p_alsa_ssp_snd_card->ssp_wq);
		snd_card_free(p_alsa_ssp_snd_card->card);
		kfree(p_alsa_ssp_snd_card);
		ret_val = 0;
	} else {
		WARN(1, "ALSA_SSP: p_alsa_ssp_snd_card is NULL\n");
		ret_val = -EINVAL;
	}
	return ret_val;

} /* intel_alsa_remove_snd_card */

static int __init intel_alsa_ssp_init(void)
{
	int status;
	pr_info("Intel ALSA SSP Driver\n Version %s\n",
			INTEL_ALSA_SSP_DRIVER_VERSION);
	status = intel_alsa_create_snd_card();
	return status;

}

static void __exit intel_alsa_ssp_exit(void)
{
	int status;

	status = intel_alsa_remove_snd_card();
	pr_info("Intel ALSA SSP Driver Remove with status = %d\n", status);

}

module_init(intel_alsa_ssp_init);
module_exit(intel_alsa_ssp_exit);
