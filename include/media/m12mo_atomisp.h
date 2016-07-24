/*
 *
 * Copyright (c) 2014 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#ifndef __M12MO_ATOMISP_HEADER__
#define __M12MO_ATOMISP_HEADER__

struct m12mo_atomisp_spi_platform_data {
	void *device_data;
	int spi_enabled;
	int spi_bus_num;
	int spi_cs_gpio;
	int spi_speed_hz;
	int spi_clock_flis;
	int spi_dataout_flis;
	int spi_datain_flis;
	int spi_cs_flis;
};

#define M12MO_MAX_FW_ID_STRING 12

#define M12MO_RESOLUTION_MODE_OFFSET	(0)
#define M12MO_AF_MODE_OFFSET		(4)
#define M12MO_MIPI_FREQ_MODE_OFFSET	(8)
#define M12MO_CLOCK_RATE_MODE_OFFSET	(12)
#define M12MO_MIPI_PACKET_SIZE_OFFSET	(16)
#define M12MO_SHOT_MODE_OFFSET		(31)

#define M12MO_RESOLUTION_MODE_0		(0 << M12MO_RESOLUTION_MODE_OFFSET)
#define M12MO_RESOLUTION_MODE_1		(1 << M12MO_RESOLUTION_MODE_OFFSET)
#define M12MO_RESOLUTION_MODE_2		(2 << M12MO_RESOLUTION_MODE_OFFSET)
#define M12MO_AF_MODE_0			(0 << M12MO_AF_MODE_OFFSET)
#define M12MO_AF_MODE_1			(1 << M12MO_AF_MODE_OFFSET)
#define M12MO_MIPI_FREQ_MODE_0		(0 << M12MO_MIPI_FREQ_MODE_OFFSET)
#define M12MO_MIPI_FREQ_MODE_1		(1 << M12MO_MIPI_FREQ_MODE_OFFSET)
#define M12MO_CLOCK_RATE_MODE_0		(0 << M12MO_CLOCK_RATE_MODE_OFFSET)
#define M12MO_CLOCK_RATE_MODE_1		(1 << M12MO_CLOCK_RATE_MODE_OFFSET)
#define M12MO_MIPI_PACKET_SIZE_2K	(0 << M12MO_MIPI_PACKET_SIZE_OFFSET)
#define M12MO_MIPI_PACKET_SIZE_4K	(1 << M12MO_MIPI_PACKET_SIZE_OFFSET)
#define M12MO_SHOT_MODE_SUPPORT		(1 << M12MO_SHOT_MODE_OFFSET)

#define M12MO_MASK	(0xF)

#define M12MO_FW_TYPE_0	(M12MO_RESOLUTION_MODE_1 | \
			 M12MO_AF_MODE_0 |	   \
			 M12MO_MIPI_FREQ_MODE_0 |  \
			 M12MO_CLOCK_RATE_MODE_0 | \
			 M12MO_MIPI_PACKET_SIZE_2K)

#define M12MO_FW_TYPE_1	(M12MO_RESOLUTION_MODE_1 | \
			 M12MO_AF_MODE_1 |	   \
			 M12MO_MIPI_FREQ_MODE_1 |  \
			 M12MO_CLOCK_RATE_MODE_1 | \
			 M12MO_MIPI_PACKET_SIZE_2K)

#define M12MO_FW_TYPE_2	(M12MO_RESOLUTION_MODE_2 | \
			 M12MO_AF_MODE_1 |	   \
			 M12MO_MIPI_FREQ_MODE_1 |  \
			 M12MO_CLOCK_RATE_MODE_1 | \
			 M12MO_MIPI_PACKET_SIZE_2K)

#define M12MO_FW_TYPE_3	(M12MO_RESOLUTION_MODE_1 | \
			 M12MO_AF_MODE_0 |	   \
			 M12MO_MIPI_FREQ_MODE_0 |  \
			 M12MO_CLOCK_RATE_MODE_1 | \
			 M12MO_SHOT_MODE_SUPPORT | \
			 M12MO_MIPI_PACKET_SIZE_2K)

#define M12MO_FW_TYPE_4	(M12MO_RESOLUTION_MODE_1 | \
			 M12MO_AF_MODE_0 |	   \
			 M12MO_MIPI_FREQ_MODE_0 |  \
			 M12MO_CLOCK_RATE_MODE_1 | \
			 M12MO_SHOT_MODE_SUPPORT | \
			 M12MO_MIPI_PACKET_SIZE_4K)

#define M12MO_FW_TYPE_5	(M12MO_RESOLUTION_MODE_1 | \
			 M12MO_AF_MODE_1 |	   \
			 M12MO_MIPI_FREQ_MODE_1 |  \
			 M12MO_CLOCK_RATE_MODE_1 | \
			 M12MO_MIPI_PACKET_SIZE_4K)

struct m12mo_fw_id {
	char *id_string;
	int fw_type;
};

struct m12mo_platform_data {
	struct camera_sensor_platform_data common;
	struct m12mo_atomisp_spi_platform_data spi_pdata;
	int *ref_clock_rate;
	u32 *mipi_packet_size;
	unsigned int def_fw_type;
	struct m12mo_fw_id *fw_ids;
	void (*spi_setup)(struct m12mo_atomisp_spi_platform_data *spi_pdata,
			  void *data);
	int (*identify_fw)(void);
};

#endif

