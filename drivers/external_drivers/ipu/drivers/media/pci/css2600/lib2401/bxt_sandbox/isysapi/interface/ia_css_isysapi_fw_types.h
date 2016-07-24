/*
 * Support for Intel Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2010 - 2014 Intel Corporation. All Rights Reserved.
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

#ifndef __IA_CSS_ISYSAPI_FW_TYPES_H__
#define __IA_CSS_ISYSAPI_FW_TYPES_H__


/* TODO: REMOVE --> START IF EXTERNALLY INCLUDED/DEFINED */
#define MAX_IPINS 6
#define MAX_OPINS 18
/*       REMOVE --> END   IF EXTERNALLY INCLUDED/DEFINED */

/* Max number of supported virtual streams To be determined*/
#define  STREAM_ID_MAX 6


/**
 * enum ia_css_isys_resp_type
 */
enum ia_css_isys_resp_type {
	IA_CSS_ISYS_RESP_TYPE_STREAM_OPEN_DONE,
	IA_CSS_ISYS_RESP_TYPE_STREAM_START_ACK,
	IA_CSS_ISYS_RESP_TYPE_STREAM_START_AND_CAPTURE_ACK,
	IA_CSS_ISYS_RESP_TYPE_STREAM_CAPTURE_ACK,
	IA_CSS_ISYS_RESP_TYPE_STREAM_STOP_ACK,
	IA_CSS_ISYS_RESP_TYPE_STREAM_FLUSH_ACK,
	IA_CSS_ISYS_RESP_TYPE_STREAM_CLOSE_ACK,
	IA_CSS_ISYS_RESP_TYPE_PIN_DATA_READY,
	IA_CSS_ISYS_RESP_TYPE_PIN_DATA_WATERMARK,
	IA_CSS_ISYS_RESP_TYPE_FRAME_SOF,
	IA_CSS_ISYS_RESP_TYPE_FRAME_EOF,
	IA_CSS_ISYS_RESP_TYPE_STREAM_START_AND_CAPTURE_DONE,
	IA_CSS_ISYS_RESP_TYPE_STREAM_CAPTURE_DONE,
	IA_CSS_ISYS_RESP_TYPE_PIN_DATA_SKIPPED,
	IA_CSS_ISYS_RESP_TYPE_STREAM_CAPTURE_SKIPPED,
	N_IA_CSS_ISYS_RESP_TYPE
};

/**
 * enum ia_css_isys_send_type
 */
enum ia_css_isys_send_type {
	IA_CSS_ISYS_SEND_TYPE_STREAM_OPEN,
	IA_CSS_ISYS_SEND_TYPE_STREAM_START,
	IA_CSS_ISYS_SEND_TYPE_STREAM_START_AND_CAPTURE,
	IA_CSS_ISYS_SEND_TYPE_STREAM_CAPTURE,
	IA_CSS_ISYS_SEND_TYPE_STREAM_STOP,
	IA_CSS_ISYS_SEND_TYPE_STREAM_FLUSH,
	IA_CSS_ISYS_SEND_TYPE_STREAM_CLOSE,
	N_IA_CSS_ISYS_SEND_TYPE
};

/**
 * enum ia_css_isys_stream_source: Specifies a source for a stream
 */
enum ia_css_isys_stream_source {
    IA_CSS_ISYS_STREAM_SRC_CSI2_PORT0,
    IA_CSS_ISYS_STREAM_SRC_CSI2_PORT1,
    IA_CSS_ISYS_STREAM_SRC_CSI2_PORT2,
    IA_CSS_ISYS_STREAM_SRC_CSI2_PORT3,
    IA_CSS_ISYS_STREAM_SRC_MIPIGEN_PORT0,
    IA_CSS_ISYS_STREAM_SRC_MIPIGEN_PORT1,
    N_IA_CSS_ISYS_STREAM_SRC
};

/**
 * enum ia_css_isys_mipi_vc: MIPI csi2 spec
 * supports upto 4 virtual per physical channel
 */
enum ia_css_isys_mipi_vc {
    IA_CSS_ISYS_MIPI_VC_0,
    IA_CSS_ISYS_MIPI_VC_1,
    IA_CSS_ISYS_MIPI_VC_2,
    IA_CSS_ISYS_MIPI_VC_3,
    N_IA_CSS_ISYS_MIPI_VC
};

/**
 *  Supported Pixel Frame formats. TODO: ensure following format list is valid
 */
enum ia_css_isys_frame_format_type {
    IA_CSS_ISYS_FRAME_FORMAT_NV11,          /* 12 bit YUV 411, Y, UV plane */
    IA_CSS_ISYS_FRAME_FORMAT_NV12,          /* 12 bit YUV 420, Y, UV plane */
    IA_CSS_ISYS_FRAME_FORMAT_NV16,          /* 16 bit YUV 422, Y, UV plane */
    IA_CSS_ISYS_FRAME_FORMAT_NV21,          /* 12 bit YUV 420, Y, VU plane */
    IA_CSS_ISYS_FRAME_FORMAT_NV61,          /* 16 bit YUV 422, Y, VU plane */
    IA_CSS_ISYS_FRAME_FORMAT_YV12,          /* 12 bit YUV 420, Y, V, U plane */
    IA_CSS_ISYS_FRAME_FORMAT_YV16,          /* 16 bit YUV 422, Y, V, U plane */
    IA_CSS_ISYS_FRAME_FORMAT_YUV420,        /* 12 bit YUV 420, Y, U, V plane */
    IA_CSS_ISYS_FRAME_FORMAT_YUV420_16,     /* yuv420, 16 bits per subpixel */
    IA_CSS_ISYS_FRAME_FORMAT_YUV422,        /* 16 bit YUV 422, Y, U, V plane */
    IA_CSS_ISYS_FRAME_FORMAT_YUV422_16,     /* yuv422, 16 bits per subpixel */
    IA_CSS_ISYS_FRAME_FORMAT_UYVY,          /* 16 bit YUV 422, UYVY interleaved */
    IA_CSS_ISYS_FRAME_FORMAT_YUYV,          /* 16 bit YUV 422, YUYV interleaved */
    IA_CSS_ISYS_FRAME_FORMAT_YUV444,        /* 24 bit YUV 444, Y, U, V plane */
    IA_CSS_ISYS_FRAME_FORMAT_YUV_LINE,      /* Internal format, 2 y lines followed
					by a uvinterleaved line */
    IA_CSS_ISYS_FRAME_FORMAT_RAW,	        /* RAW, 1 plane */
    IA_CSS_ISYS_FRAME_FORMAT_RGB565,        /* 16 bit RGB, 1 plane. Each 3 sub
					pixels are packed into one 16 bit value, 5 bits for R, 6 bits
					for G and 5 bits for B. */
    IA_CSS_ISYS_FRAME_FORMAT_PLANAR_RGB888, /* 24 bit RGB, 3 planes */
    IA_CSS_ISYS_FRAME_FORMAT_RGBA888,	    /* 32 bit RGBA, 1 plane, A=Alpha
					(alpha is unused) */
	IA_CSS_ISYS_FRAME_FORMAT_QPLANE6,       /* Internal, for advanced ISP */
	IA_CSS_ISYS_FRAME_FORMAT_BINARY_8,	    /* byte stream, used for jpeg. For
					frames of this type, we set the height to 1 and the width to the
					number of allocated bytes. */
	N_IA_CSS_ISYS_FRAME_FORMAT
};

/**
 *  Supported MIPI data type. TODO: ensure following format list is valid
 */
enum ia_css_isys_mipi_data_type {

	/*IA_CSS_ISYS_MIPI_DATA_TYPE_NULL			= 0x10,*/
	/*IA_CSS_ISYS_MIPI_DATA_TYPE_BLANKING_DATA	= 0x11,*/
	IA_CSS_ISYS_MIPI_DATA_TYPE_EMBEDDED			= 0x12,         	/* Embedded 8-bit non Image Data */
	IA_CSS_ISYS_MIPI_DATA_TYPE_YUV420_8			= 0x18,				/* 8 bits per subpixel */
	IA_CSS_ISYS_MIPI_DATA_TYPE_YUV420_10		= 0x19,				/* 10 bits per subpixel */
    IA_CSS_ISYS_MIPI_DATA_TYPE_YUV420_8_LEGACY	= 0x1A,				/* 8 bits per subpixel */
    IA_CSS_ISYS_MIPI_DATA_TYPE_YUV420_8_SHIFT	= 0x1C,				/* YUV420 8-bit Chroma Shifted Pixel Sampling) */
	IA_CSS_ISYS_MIPI_DATA_TYPE_YUV420_10_SHIFT	= 0x1D,				/* YUV420 8-bit (Chroma Shifted Pixel Sampling) */
    IA_CSS_ISYS_MIPI_DATA_TYPE_YUV422_8			= 0x1E,				/* UYVY..UVYV, 8 bits per subpixel */
    IA_CSS_ISYS_MIPI_DATA_TYPE_YUV422_10		= 0x1F,				/* UYVY..UVYV, 10 bits per subpixel */
    IA_CSS_ISYS_MIPI_DATA_TYPE_RGB_444			= 0x20,         	/* BGR..BGR, 4 bits per subpixel */
    IA_CSS_ISYS_MIPI_DATA_TYPE_RGB_555			= 0x21,				/* BGR..BGR, 5 bits per subpixel */
    IA_CSS_ISYS_MIPI_DATA_TYPE_RGB_565			= 0x22,				/* BGR..BGR, 5 bits B and R, 6 bits G */
    IA_CSS_ISYS_MIPI_DATA_TYPE_RGB_666			= 0x23,				/* BGR..BGR, 6 bits per subpixel */
    IA_CSS_ISYS_MIPI_DATA_TYPE_RGB_888			= 0x24,				/* BGR..BGR, 8 bits per subpixel */
    IA_CSS_ISYS_MIPI_DATA_TYPE_RAW_6			= 0x28,           	/* RAW data, 6 bits per pixel */
    IA_CSS_ISYS_MIPI_DATA_TYPE_RAW_7			= 0x29,				/* RAW data, 7 bits per pixel */
    IA_CSS_ISYS_MIPI_DATA_TYPE_RAW_8			= 0x2A,				/* RAW data, 8 bits per pixel */
    IA_CSS_ISYS_MIPI_DATA_TYPE_RAW_10			= 0x2B,				/* RAW data, 10 bits per pixel */
    IA_CSS_ISYS_MIPI_DATA_TYPE_RAW_12			= 0x2C,				/* RAW data, 12 bits per pixel */
    IA_CSS_ISYS_MIPI_DATA_TYPE_RAW_14			= 0x2D,				/* RAW data, 14 bits per pixel */
    IA_CSS_ISYS_MIPI_DATA_TYPE_RAW_16			= 0x2E,				/* RAW data, 16 bits per pixel, not specified in CSI-MIPI standard */
	IA_CSS_ISYS_MIPI_DATA_TYPE_BINARY_8			= 0x2F,        		/* Binary byte stream, which is target at JPEG, not specified in CSI-MIPI standard */
	   /** CSI2-MIPI specific format: User defined byte-based data. For example,
    *  the data transmitter (e.g. the SoC sensor) can keep the JPEG data as
    *  the User Defined Data Type 4 and the MPEG data as the
    *  User Defined Data Type 7.
    */
    IA_CSS_ISYS_MIPI_DATA_TYPE_USER_DEF1		= 0x30,				/* User defined 8-bit data type 1 */
    IA_CSS_ISYS_MIPI_DATA_TYPE_USER_DEF2		= 0x31,				/* User defined 8-bit data type 2 */
    IA_CSS_ISYS_MIPI_DATA_TYPE_USER_DEF3		= 0x32,				/* User defined 8-bit data type 3 */
    IA_CSS_ISYS_MIPI_DATA_TYPE_USER_DEF4		= 0x33,				/* User defined 8-bit data type 4 */
    IA_CSS_ISYS_MIPI_DATA_TYPE_USER_DEF5		= 0x34,				/* User defined 8-bit data type 5 */
    IA_CSS_ISYS_MIPI_DATA_TYPE_USER_DEF6		= 0x35,				/* User defined 8-bit data type 6 */
    IA_CSS_ISYS_MIPI_DATA_TYPE_USER_DEF7		= 0x36,				/* User defined 8-bit data type 7 */
    IA_CSS_ISYS_MIPI_DATA_TYPE_USER_DEF8		= 0x37,				/* User defined 8-bit data type 8 */
    /** CSI2-MIPI specific format: Generic short packet data. It is used to
    *  keep the timing information for the opening/closing of shutters,
    *  triggering of flashes and etc.
    */
    IA_CSS_ISYS_MIPI_DATA_TYPE_GENERIC_SHORT1,  /* Generic Short Packet Code 1 */
    IA_CSS_ISYS_MIPI_DATA_TYPE_GENERIC_SHORT2,  /* Generic Short Packet Code 2 */
    IA_CSS_ISYS_MIPI_DATA_TYPE_GENERIC_SHORT3,  /* Generic Short Packet Code 3 */
    IA_CSS_ISYS_MIPI_DATA_TYPE_GENERIC_SHORT4,  /* Generic Short Packet Code 4 */
    IA_CSS_ISYS_MIPI_DATA_TYPE_GENERIC_SHORT5,  /* Generic Short Packet Code 5 */
    IA_CSS_ISYS_MIPI_DATA_TYPE_GENERIC_SHORT6,  /* Generic Short Packet Code 6 */
    IA_CSS_ISYS_MIPI_DATA_TYPE_GENERIC_SHORT7,  /* Generic Short Packet Code 7 */
    IA_CSS_ISYS_MIPI_DATA_TYPE_GENERIC_SHORT8,  /* Generic Short Packet Code 8 */

    N_IA_CSS_ISYS_MIPI_DATA_TYPE  				= 0x40              /* Keep always last and max value */
};

/** enum ia_css_isys_pin_type: output pin buffer types.
 * Buffers can be queued and de-queued to hand them over between IA and ISYS
 */
enum ia_css_isys_pin_type {
    IA_CSS_ISYS_PIN_TYPE_MIPI,
    IA_CSS_ISYS_PIN_TYPE_RAW_NS,
    IA_CSS_ISYS_PIN_TYPE_RAW_S,
    IA_CSS_ISYS_PIN_TYPE_METADATA_0,
    IA_CSS_ISYS_PIN_TYPE_METADATA_1,
    IA_CSS_ISYS_PIN_TYPE_AF_STATS,
    IA_CSS_ISYS_PIN_TYPE_AWB_STATS,
    IA_CSS_ISYS_PIN_TYPE_HIST_STATS,
    N_IA_CSS_ISYS_PIN_TYPE /*Keep always last and max value */
};

/**
 * enum ia_css_isys_isl_use. Describes the ISL/ISA use
 */
enum ia_css_isys_isl_use {
	IA_CSS_ISYS_USE_NO_ISL_NO_ISA,
	IA_CSS_ISYS_USE_SINGLE_DUAL_ISL,
	IA_CSS_ISYS_USE_SINGLE_ISA,
	N_IA_CSS_ISYS_USE
};


#endif /*__IA_CSS_ISYSAPI_FW_TYPES_H__*/
