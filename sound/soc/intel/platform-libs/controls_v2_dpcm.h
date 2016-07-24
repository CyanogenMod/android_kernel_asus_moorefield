/*
 *  controls_v2_dpcm.h - Intel MID Platform driver header file
 *
 *  Copyright (C) 2013 Intel Corp
 *  Author: Ramesh Babu <ramesh.babu.koul@intel.com>
 *  Author: Omair M Abdullah <omair.m.abdullah@intel.com>
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
 *
 */

#ifndef __SST_CONTROLS_V2_DPCM_H__
#define __SST_CONTROLS_V2_DPCM_H__

#include <uapi/sound/atom_controls.h>

/*These are added specially for BTNS board to check pcm rates*/
#define SNDRV_BTNS_PCM_RATE_8000	8000
#define SNDRV_BTNS_PCM_RATE_16000	16000
#define SNDRV_BTNS_PCM_RATE_44100	44100
#define SNDRV_BTNS_PCM_RATE_48000	48000

/*
 * FBA probe point index id
 */
enum sst_fba_path_index {
	FBA_PROBE_INDEX_1	= 1,
	FBA_PROBE_INDEX_2,
	FBA_PROBE_INDEX_3,
	FBA_PROBE_INDEX_4,
	FBA_PROBE_INDEX_5,
	FBA_PROBE_INDEX_6,
	FBA_PROBE_INDEX_7,
	FBA_PROBE_INDEX_8,
	FBA_PROBE_INDEX_9,
	FBA_PROBE_INDEX_10,
	FBA_PROBE_INDEX_11,
	FBA_PROBE_INDEX_12,
	FBA_PROBE_INDEX_13,
	FBA_PROBE_INDEX_14,
	FBA_PROBE_INDEX_15,
	FBA_PROBE_INDEX_16,
	FBA_PROBE_INDEX_17,
	FBA_PROBE_INDEX_18,
	FBA_PROBE_INDEX_19,
	FBA_PROBE_INDEX_20,
	FBA_PROBE_INDEX_21,
	FBA_PROBE_INDEX_22,
	FBA_PROBE_INDEX_23,
	FBA_PROBE_INDEX_24,
	FBA_PROBE_INDEX_25,
	FBA_PROBE_INDEX_26,
	FBA_PROBE_INDEX_27,
};

enum sst_dsp_switch {
	SST_SWITCH_OFF = 0,
	SST_SWITCH_ON = 3,
};

enum sst_path_switch {
	SST_PATH_OFF = 0,
	SST_PATH_ON = 1,
};

enum sst_swm_state {
	SST_SWM_OFF = 0,
	SST_SWM_ON = 3,
};

#define SST_FILL_LOCATION_IDS(dst, cell_idx, pipe_id)		do {	\
		dst.location_id.p.cell_nbr_idx = (cell_idx);		\
		dst.location_id.p.path_id = (pipe_id);			\
	} while (0)
#define SST_FILL_LOCATION_ID(dst, loc_id)				(\
	dst.location_id.f = (loc_id))
#define SST_FILL_MODULE_ID(dst, mod_id)					(\
	dst.module_id = (mod_id))

#define SST_FILL_DESTINATION1(dst, id)				do {	\
		SST_FILL_LOCATION_ID(dst, (id) & 0xFFFF);		\
		SST_FILL_MODULE_ID(dst, ((id) & 0xFFFF0000) >> 16);	\
	} while (0)
#define SST_FILL_DESTINATION2(dst, loc_id, mod_id)		do {	\
		SST_FILL_LOCATION_ID(dst, loc_id);			\
		SST_FILL_MODULE_ID(dst, mod_id);			\
	} while (0)
#define SST_FILL_DESTINATION3(dst, cell_idx, path_id, mod_id)	do {	\
		SST_FILL_LOCATION_IDS(dst, cell_idx, path_id);		\
		SST_FILL_MODULE_ID(dst, mod_id);			\
	} while (0)

#define SST_FILL_DESTINATION(level, dst, ...)				\
	SST_FILL_DESTINATION##level(dst, __VA_ARGS__)
#define SST_FILL_DEFAULT_DESTINATION(dst)				\
	SST_FILL_DESTINATION(2, dst, SST_DEFAULT_LOCATION_ID, SST_DEFAULT_MODULE_ID)

struct sst_destination_id {
	union sst_location_id {
		struct {
			u8 cell_nbr_idx;	/* module index */
			u8 path_id;		/* pipe_id */
		} __packed	p;		/* part */
		u16		f;		/* full */
	} __packed location_id;
	u16	   module_id;
} __packed;

struct sst_dsp_header {
	struct sst_destination_id dst;
	u16 command_id;
	u16 length;
} __packed;

/*
 *
 * Common Commands
 *
 */
struct sst_cmd_generic {
	struct sst_dsp_header header;
} __packed;

struct swm_input_ids {
	struct sst_destination_id input_id;
} __packed;

struct sst_cmd_set_swm {
	struct sst_dsp_header header;
	struct sst_destination_id output_id;
	u16    switch_state;
	u16    nb_inputs;
	struct swm_input_ids input[SST_CMD_SWM_MAX_INPUTS];
} __packed;

struct sst_cmd_set_media_path {
	struct sst_dsp_header header;
	u16    switch_state;
} __packed;

struct pcm_cfg {
		u8 s_length:2;
		u8 rate:3;
		u8 format:3;
} __packed;

struct pcm_cfg_speech {
		u16 bwx:8;
		u16 s_length:2;
		u16 rate:3;
		u16 format:3;
} __packed;

struct sst_cmd_set_speech_path {
	struct sst_dsp_header header;
	u16    switch_state;
	struct pcm_cfg_speech cfg;
} __packed;

struct gain_cell {
	struct sst_destination_id dest;
	s16 cell_gain_left;
	s16 cell_gain_right;
	u16 gain_time_constant;
} __packed;

#define NUM_GAIN_CELLS 1
struct sst_cmd_set_gain_dual {
	struct sst_dsp_header header;
	u16    gain_cell_num;
	struct gain_cell cell_gains[NUM_GAIN_CELLS];
} __packed;

struct sst_cmd_set_params {
	struct sst_destination_id dst;
	u16 command_id;
	char params[0];
} __packed;

/*
 *
 * Media (MMX) commands
 *
 */

/*
 *
 * SBA commands
 *
 */
struct sst_cmd_sba_vb_start {
	struct sst_dsp_header header;
} __packed;

union sba_media_loop_params {
	struct {
		u16 rsvd:8;
		struct pcm_cfg cfg;
	} part;
	u16 full;
} __packed;

struct sst_cmd_sba_set_media_loop_map {
	struct	sst_dsp_header header;
	u16	switch_state;
	union	sba_media_loop_params param;
	u16	map;
} __packed;

struct sst_cmd_tone_stop {
	struct	sst_dsp_header header;
	u16	switch_state;
} __packed;

enum sst_ssp_mode {
	SSP_MODE_MASTER = 0,
	SSP_MODE_SLAVE = 1,
};

enum sst_ssp_pcm_mode {
	SSP_PCM_MODE_NORMAL = 0,
	SSP_PCM_MODE_NETWORK = 1,
};

enum sst_ssp_duplex {
	SSP_DUPLEX = 0,
	SSP_RX = 1,
	SSP_TX = 2,
};

enum sst_ssp_fs_frequency {
	SSP_FS_8_KHZ = 0,
	SSP_FS_16_KHZ = 1,
	SSP_FS_44_1_KHZ = 2,
	SSP_FS_48_KHZ = 3,
};

enum sst_ssp_fs_polarity {
	SSP_FS_ACTIVE_LOW = 0,
	SSP_FS_ACTIVE_HIGH = 1,
};

enum sst_ssp_protocol {
	SSP_MODE_PCM = 0,
	SSP_MODE_I2S = 1,
};

enum sst_ssp_port_id {
	SSP_MODEM = 0,
	SSP_BT = 1,
	SSP_FM = 2,
	SSP_CODEC = 3,
};

struct sst_cmd_sba_hw_set_ssp {
	struct sst_dsp_header header;
	u16 selection;			/* 0:SSP0(def), 1:SSP1, 2:SSP2 */

	u16 switch_state;

	u16 nb_bits_per_slots:6;        /* 0-32 bits, 24 (def) */
	u16 nb_slots:4;			/* 0-8: slots per frame  */
	u16 mode:3;			/* 0:Master, 1: Slave  */
	u16 duplex:3;

	u16 active_tx_slot_map:8;       /* Bit map, 0:off, 1:on */
	u16 reserved1:8;

	u16 active_rx_slot_map:8;       /* Bit map 0: Off, 1:On */
	u16 reserved2:8;

	u16 frame_sync_frequency;

	u16 frame_sync_polarity:8;
	u16 data_polarity:8;

	u16 frame_sync_width;           /* 1 to N clocks */
	u16 ssp_protocol:8;
	u16 start_delay:8;		/* Start delay in terms of clock ticks */
} __packed;

#define SST_MAX_TDM_SLOTS 8

struct sst_param_sba_ssp_slot_map {
	struct sst_dsp_header header;

	u16 param_id;
	u16 param_len;
	u16 ssp_index;

	u8 rx_slot_map[SST_MAX_TDM_SLOTS];
	u8 tx_slot_map[SST_MAX_TDM_SLOTS];
} __packed;

enum {
	SST_PROBE_EXTRACTOR = 0,
	SST_PROBE_INJECTOR = 1,
};

struct sst_cmd_probe {
	struct sst_dsp_header header;

	u16 switch_state;
	struct sst_destination_id probe_dst;

	u16 shared_mem:1;
	u16 probe_in:1;
	u16 probe_out:1;
	u16 rsvd_1:13;

	u16 rsvd_2:5;
	u16 probe_mode:2;
	u16 rsvd_3:1;
	struct pcm_cfg cfg;

	u16 sm_buf_id;

	u16 gain[6];
	u16 rsvd_4[9];
} __packed;

struct sst_probe_config {
	const char *name;
	u16 loc_id;
	u16 mod_id;
	u8 task_id;
	struct pcm_cfg cfg;
};

int sst_mix_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int sst_mix_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
#endif /* __SST_CONTROLS_V2_DPCM_H__ */
