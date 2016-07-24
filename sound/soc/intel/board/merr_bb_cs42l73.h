#ifndef _MERR_BB_CS42L73_H_
#define _MERR_BB_CS42L73_H_

/* Data path functionalities */
struct snd_pcm_hardware MERR_BB_BT_sco_hw_param = {
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

struct snd_pcm_hardware MERR_BB_FM_soc_hw_param = {
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
struct snd_pcm_hardware MERR_BB_MIXING_alsa_hw_param = {
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
struct snd_pcm_hardware MERR_BB_VOIP_alsa_hw_param = {
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
 * For VoIP 1 slot of 32 bits is used
 * to transfer stereo 16 bits PCM samples
 */
#define MERR_BB_SSP_VOIP_SLOT_NB_SLOT	1
#define MERR_BB_SSP_VOIP_SLOT_WIDTH		32
#define MERR_BB_SSP_VOIP_SLOT_RX_MASK	0x1
#define MERR_BB_SSP_VOIP_SLOT_TX_MASK	0x1

/*
 * For Modem IFX 1 slot of 32 bits is used
 * to transfer stereo 16 bits PCM samples
 */
#define MERR_BB_SSP_MIXING_SLOT_NB_SLOT	1
#define MERR_BB_SSP_MIXING_SLOT_WIDTH		32
#define MERR_BB_SSP_MIXING_SLOT_RX_MASK	0x1
#define MERR_BB_SSP_MIXING_SLOT_TX_MASK	0x1

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


#endif /* MERR_BB_CS42L73_H */
