#ifndef __IA_CSS_PROGRAM_GROUP_DATA_H_INCLUDED__
#define __IA_CSS_PROGRAM_GROUP_DATA_H_INCLUDED__

/*! \file */

/** @file ia_css_program_group_data.h
 *
 * Define the data objects that are passed to the process groups
 * i.e. frames and matrices with their sub-structures
 *
 * The data objects are separate from the process group terminal,
 * although they are stored by value rather than by reference and
 * make the process group terminal dependendent on its definition
 *
 * This frame definition overloads the current CSS frame definition
 * they are the same object, just a slightly different implementation
 */

#include <vied_nci_psys_system_global.h>	/* vied_vaddress_t */

#include <type_support.h>

/*
 * Dimensions of the data objects. Note that a C-style
 * data order is assumed. Data stored by row.
 */
typedef enum ia_css_dimension {
	IA_CSS_COL_DIMENSION = 0,			/**< The number of columns, i.e. the size of the row */
	IA_CSS_ROW_DIMENSION,				/**< The number of rows, i.e. the size of the column */
	IA_CSS_N_DATA_DIMENSION
} ia_css_dimension_t;

/*
 * Frame buffer state used for sequencing
 * (see FAS 5.5.3)
 *
 * The buffer can be in DDR or a handle to a stream
 */
typedef enum ia_css_buffer_state {
	IA_CSS_BUFFER_NULL = 0,
	IA_CSS_BUFFER_UNDEFINED,
	IA_CSS_BUFFER_EMPTY,
	IA_CSS_BUFFER_NONEMPTY,
	IA_CSS_BUFFER_FULL,
	IA_CSS_N_BUFFER_STATES
} ia_css_buffer_state_t;

/*
 * Pointer state used to signal MMU invalidation
 */
typedef enum ia_css_pointer_state {
	IA_CSS_POINTER_INVALID = 0,
	IA_CSS_POINTER_VALID,
	IA_CSS_N_POINTER_STATES
} ia_css_pointer_state_t;

/*
 * Access direction needed to select the access port
 */
typedef enum ia_css_access_type {
	IA_CSS_ACCESS_LOCKED = 0,
	IA_CSS_ACCESS_READ,
	IA_CSS_ACCESS_WRITE,
	IA_CSS_ACCESS_MODIFY,
	IA_CSS_N_ACCESS_TYPES
} ia_css_access_type_t;

/*
 * Access attribute needed to select the access port
 *	- public : snooped
 *	- private: non-snooped
 * Naming is a bit awkward, lack of inspiration
 */
typedef enum ia_css_access_scope {
	IA_CSS_ACCESS_PRIVATE = 0,
	IA_CSS_ACCESS_PUBLIC,
	IA_CSS_N_ACCESS_SCOPES
} ia_css_access_scopes_t;

/*
 * Pre-defined frame format
 *
 * Those formats have inbuild support of traffic
 * and access functions
 *
 * Note that the formats are for terminals, so there
 * is no distinction between input and output formats
 *	- Custom formats with ot without descriptor
 *	- 4CC formats such as YUV variants
 *	- MIPI (line) formats as produced by CSI receivers
 *	- MIPI (sensor) formats such as Bayer or RGBC
 *	- CSS internal formats (private types)
 *  - CSS parameters (type 1 - 6)
 */
#define IA_CSS_FRAME_FORMAT_TYPE_BITS					32
typedef enum ia_css_frame_format_type {
	IA_CSS_DATA_CUSTOM_NO_DESCRIPTOR = 0,
	IA_CSS_DATA_CUSTOM,

	IA_CSS_DATA_FORMAT_NV11,			/**<  12 bit YUV 411, Y, UV 2-plane  (8 bit per element) */
	IA_CSS_DATA_FORMAT_YUV420,			/**< bpp bit YUV 420, Y, U, V 3-plane (bpp/1.5 bpe) */
	IA_CSS_DATA_FORMAT_YV12,			/**<  12 bit YUV 420, Y, V, U 3-plane (8 bit per element) */
	IA_CSS_DATA_FORMAT_NV12,			/**<  12 bit YUV 420, Y, UV 2-plane (8 bit per element) */
	IA_CSS_DATA_FORMAT_NV21,			/**<  12 bit YUV 420, Y, VU 2-plane  (8 bit per element) */
	IA_CSS_DATA_FORMAT_YUV422,			/**< bpp bit YUV 422, Y, U, V 3-plane (bpp/2 bpe) */
	IA_CSS_DATA_FORMAT_YV16,			/**<  16 bit YUV 422, Y, V, U 3-plane  (8 bit per element) */
	IA_CSS_DATA_FORMAT_NV16,			/**<  16 bit YUV 422, Y, UV 2-plane  (8 bit per element) */
	IA_CSS_DATA_FORMAT_NV61,			/**<  16 bit YUV 422, Y, VU 2-plane  (8 bit per element) */
	IA_CSS_DATA_FORMAT_UYVY,			/**<  16 bit YUV 422, UYVY 1-plane interleaved  (8 bit per element) */
	IA_CSS_DATA_FORMAT_YUYV,			/**<  16 bit YUV 422, YUYV 1-plane interleaved  (8 bit per element) */
	IA_CSS_DATA_FORMAT_YUV444,			/**< bpp bit YUV 444, Y, U, V 3-plane (bpp/3 bpe) */

	IA_CSS_DATA_FORMAT_RGB565,			/**< 5-6-5 bit packed (1-plane) RGB (16bpp, ~5 bpe) */
	IA_CSS_DATA_FORMAT_RGB888,			/**< 24 bit RGB, 3 planes  (8 bit per element) */
	IA_CSS_DATA_FORMAT_RGBA888,		/**< 32 bit RGB-Alpha, 1 plane  (8 bit per element) */

	IA_CSS_DATA_FORMAT_BAYER_GRBG,		/**< bpp bit raw, [[Gr, R];[B, Gb]] 1-plane (bpp == bpe) */
	IA_CSS_DATA_FORMAT_BAYER_RGGB,		/**< bpp bit raw, [[R, Gr];[Gb, B]] 1-plane (bpp == bpe) */
	IA_CSS_DATA_FORMAT_BAYER_BGGR,		/**< bpp bit raw, [[B, Gb];[Gr, R]] 1-plane (bpp == bpe) */
	IA_CSS_DATA_FORMAT_BAYER_GBRG,		/**< bpp bit raw, [[Gb, B];[R, Gr]] 1-plane (bpp == bpe) */

	IA_CSS_DATA_FORMAT_YUV420_LINE,	/**< bpp bit (NV12) YUV 420, Y, UV 2-plane derived 3-line, 2-Y, 1-UV (bpp/1.5 bpe) */
	IA_CSS_DATA_FORMAT_RAW,			/**< Deprecated RAW, 1 plane */
	IA_CSS_DATA_FORMAT_RAW_PACKED,		/**< Deprecated RAW, 1 plane, packed */
	IA_CSS_DATA_FORMAT_QPLANE6,		/**< Internal, for advanced ISP */
	IA_CSS_DATA_FORMAT_BINARY_8,		/**< 1D byte stream, used for jpeg 1-plane */
	IA_CSS_DATA_FORMAT_MIPI,			/**< Deprecated MIPI frame, 1D byte stream 1 plane */
	IA_CSS_DATA_FORMAT_MIPI_YUV420_8,	/**<  12 bit [[YY];[UYVY]]  1-plane interleaved 2-line (8 bit per element) */
	IA_CSS_DATA_FORMAT_MIPI_YUV420_10,	/**<  15 bit [[YY];[UYVY]]  1-plane interleaved 2-line (10 bit per element) */
	IA_CSS_DATA_FORMAT_MIPI_LEGACY_YUV420_8,/**<  12 bit [[UY];[VY]]  1-plane interleaved 2-line (8 bit per element) */

	IA_CSS_DATA_GENERIC_PARAMETER,		/**< Type 1-5 parameter, not fragmentable */
	IA_CSS_DATA_DVS_PARAMETER,			/**< Video stabilisation Type 6 parameter, fragmentable */
	IA_CSS_DATA_DPC_PARAMETER,			/**< Dead Pixel correction Type 6 parameter, fragmentable */
	IA_CSS_DATA_LSC_PARAMETER,			/**< Lens Shading Correction Type 6 parameter, fragmentable */
	IA_CSS_DATA_S3A_STATISTICS_HI,                /**< 3A statistics output HI. */
	IA_CSS_DATA_S3A_STATISTICS_LO,                /**< 3A statistics output LO. */
	IA_CSS_DATA_S3A_HISTOGRAM,		   /**< histogram output */
	IA_CSS_N_FRAME_FORMAT_TYPES
} ia_css_frame_format_type_t;

#define IA_CSS_N_FRAME_PLANES					6

#define IA_CSS_FRAME_FORMAT_BITMAP_BITS			64
typedef uint64_t					ia_css_frame_format_bitmap_t;

typedef struct ia_css_param_frame_descriptor_s	ia_css_param_frame_descriptor_t;
typedef struct ia_css_param_frame_s				ia_css_param_frame_t;

typedef struct ia_css_frame_descriptor_s		ia_css_frame_descriptor_t;
typedef struct ia_css_frame_s					ia_css_frame_t;
typedef struct ia_css_fragment_descriptor_s		ia_css_fragment_descriptor_t;

typedef struct ia_css_kernel_param_descriptor_s	ia_css_kernel_param_descriptor_t;
typedef struct ia_css_kernel_param_s			ia_css_kernel_param_t;

typedef struct ia_css_stream_s				ia_css_stream_t;
/*
 * For parameter transfer, the pass by reference is replaced with copy-on-update
 * into a limited set of parameter buffers. Hence rather than holding multiple
 *
 * The size of the sections is implicit by the section offsets. For most parameters
 * the size is known at compile time. Only for type-5 parameters which are identified
 * by the destination ID in the manifest, the size is variable
 */

struct ia_css_kernel_param_descriptor_s {
	uint16_t						size;										/**< Size of the descriptor */
	uint16_t						section_count;								/**< Number of parameter sections*/
	uint32_t						*section_offsets;							/**< Array[section_count + 1] Plane offsets (in bytes), the last offset equals the size (in bytes) */
};

#define IA_CSS_KERNEL_PARAM_HANDLE_BITS		128

struct ia_css_kernel_param_s {
	vied_vaddress_t					buffer;										/**< Base virtual addresses to parameters in subsystem virtual memory space */
};
struct ia_css_param_frame_descriptor_s {
	uint16_t							size;										/**< Size of the descriptor */
	uint32_t							buffer_count;								/**< Number of parameter buffers */
};

struct ia_css_param_frame_s {
	vied_vaddress_t						*data;										/**< Base virtual addresses to parameters in subsystem virtual memory space */
};

/*
 * Structure defining the frame (size and access) properties for inbuild types only.
 *
 * The inbuild types like FourCC, MIPI and CSS private types are supported by FW
 * all other types are custom types which interpretation must be encoded on the
 * buffer itself or known by the source and sink
 */
struct ia_css_frame_descriptor_s {
	uint16_t							size;										/**< Size of the descriptor */
	ia_css_frame_format_type_t			frame_format_type;							/**< Indicates if this is a generic type or inbuild with variable size descriptor */
	uint32_t							plane_count;								/**< Number of data planes (pointers) */
	uint8_t								bpp;										/**< Bits per pixel */
	uint8_t								bpe;										/**< Bits per element */
/*	uint32_t							*plane_offsets;							*/	/*< Plane offsets accounting for fragments */
	uint32_t							plane_offsets[IA_CSS_N_FRAME_PLANES];		/**< Plane offsets accounting for fragments */
	uint16_t							dimension[IA_CSS_N_DATA_DIMENSION];			/**< Logical dimensions */
	uint32_t							stride[IA_CSS_N_DATA_DIMENSION - 1];		/**< Physical size aspects */
};

/*
 * Main frame structure holding the main store and auxilary access properties
 *
 * the "pointer_state" and "access_scope" should be encoded on the "vied_vaddress_t" type
 */
struct ia_css_frame_s {
	ia_css_buffer_state_t				buffer_state;								/**< State of the frame for purpose of sequencing */
	ia_css_access_type_t				access_type;								/**< Access direction, may change when buffer state changes */
	ia_css_pointer_state_t				pointer_state;								/**< State of the pointer for purpose of embedded MMU coherency */
	ia_css_access_scopes_t				access_scope;								/**< Access to the pointer for purpose of host cache coherency */
	uint32_t							data_bytes;									/**< Total allocation size in bytes */
	vied_vaddress_t						data;										/**< Base virtual address to data in subsystem virtual memory space */
};

/*
 * Structure defining the fragment (size and access) properties.
 *
 * All cropping and padding effects are described by the difference between
 * the frame size and its location and the fragment size(s) and location(s)
 */
struct ia_css_fragment_descriptor_s {
	uint16_t							dimension[IA_CSS_N_DATA_DIMENSION];			/**< Logical dimensions of the fragment */
	uint16_t							index[IA_CSS_N_DATA_DIMENSION];				/**< Logical location of the fragment in the frame */
	uint16_t							offset[IA_CSS_N_DATA_DIMENSION];			/**< Fractional start (phase) of the fragment in the access unit */
};


/*! Print the frame object to file/stream

 @param	frame[in]				frame object
 @param	fid[out]				file/stream handle

 @return < 0 on error
 */
extern int ia_css_frame_print(
	const ia_css_frame_t					*frame,
	void									*fid);

/*! Get the data buffer handle from the frame object

 @param	frame[in]				frame object

 @return buffer pointer, VIED_NULL on error
 */
extern vied_vaddress_t ia_css_frame_get_buffer(
	const ia_css_frame_t					*frame);

/*! Set the data buffer handle on the frame object

 @param	frame[in]				frame object
 @param	buffer[in]				buffer pointer

 @return < 0 on error
 */
extern int ia_css_frame_set_buffer(
	ia_css_frame_t							*frame,
	vied_vaddress_t							buffer);

/*! Set the data buffer size on the frame object

 @param	frame[in]				frame object
 @param	size[in]				number of data bytes

 @return < 0 on error
 */
extern int ia_css_frame_set_data_bytes(
	ia_css_frame_t							*frame,
	unsigned int							size);

/*! Get the data buffer state from the frame object

 @param	frame[in]				frame object

 @return buffer state, limit value on error
 */
extern ia_css_buffer_state_t ia_css_frame_get_buffer_state(
	const ia_css_frame_t					*frame);

/*! Set the data buffer state of the frame object

 @param	frame[in]				frame object
 @param	buffer_state[in]		buffer state

 @return < 0 on error
 */
extern int ia_css_frame_set_buffer_state(
	ia_css_frame_t							*frame,
	const ia_css_buffer_state_t				buffer_state);

/*! Get the data pointer state from the frame object

 @param	frame[in]				frame object

 @return pointer state, limit value on error
 */
extern ia_css_pointer_state_t ia_css_frame_get_pointer_state(
	const ia_css_frame_t					*frame);

/*! Set the data pointer state of the frame object

 @param	frame[in]				frame object
 @param	pointer_state[in]		pointer state

 @return < 0 on error
 */
extern int ia_css_frame_set_pointer_state(
	ia_css_frame_t							*frame,
	const ia_css_pointer_state_t			pointer_state);

/*! Print the frame decriptor object to file/stream

 @param	frame_descriptor[in]	frame descriptor object
 @param	fid[out]				file/stream handle

 @return < 0 on error
 */
extern int ia_css_frame_descriptor_print(
	const ia_css_frame_descriptor_t			*frame_descriptor,
	void									*fid);

/*! Print the fragment decriptor object to file/stream

 @param	fragment_descriptor[in]	fragment descriptor object
 @param	fid[out]				file/stream handle

 @return < 0 on error
 */
extern int ia_css_fragment_descriptor_print(
	const ia_css_fragment_descriptor_t		*fragment_descriptor,
	void									*fid);

/*! Compute the bitmap for the frame format type

 @param	frame_format_type[in]	frame format type

 @return 0 on error
 */
extern ia_css_frame_format_bitmap_t ia_css_frame_format_bit_mask(
	const ia_css_frame_format_type_t		frame_format_type);

/*! clear frame format bitmap

 @return cleared bitmap
 */
extern ia_css_frame_format_bitmap_t ia_css_frame_format_bitmap_clear(void);


/*! Compute the size of storage required for the data descriptor object on a terminal
 *@param»       plane_count[in]		The number of data planes in the buffer
 */
extern size_t ia_css_sizeof_frame_descriptor(
		const uint8_t			plane_count);
/*! Compute the size of storage required for the kernel parameter descriptor object on a terminal

 @param	section_count[in]				The number of parameter sections in the buffer

 @return 0 on error
 */
extern size_t ia_css_sizeof_kernel_param_descriptor(
	const uint16_t							section_count);
/*! Get the kernel parameter buffer handle from the kernel parameter object

 @param	kernel_param[in]				kernel parameter object

 @return buffer pointer, VIED_NULL on error
 */
extern vied_vaddress_t	ia_css_kernel_param_get_buffer(
	const ia_css_kernel_param_t				*kernel_param);

/*! Set the kernel parameter buffer handle on the kernel parameter object

 @param	kernel_param[in]				kernel parameter object
 @param	buffer[in]						buffer pointer

 @return < 0 on error
 */
extern int ia_css_kernel_param_set_buffer(
	ia_css_kernel_param_t					*kernel_param,
	vied_vaddress_t							buffer);

/*! Get the stored size of the kernel parameter descriptor object

 @param	kernel_param_descriptor[in]	kernel parameter descriptor object

 @return size, 0 on error
 */
extern size_t ia_css_kernel_param_descriptor_get_size(
	const ia_css_kernel_param_descriptor_t	*kernel_param_descriptor);

/*! Get the number of parameter sections of the kernel parameter descriptor object

 @param	kernel_param_descriptor[in]	kernel parameter descriptor object

 Note: There must be at least one parameter section

 @return section count, 0 on error
 */
extern uint16_t ia_css_kernel_param_descriptor_get_section_count(
	const ia_css_kernel_param_descriptor_t	*kernel_param_descriptor);

#endif /* __IA_CSS_PROGRAM_GROUP_DATA_H_INCLUDED__  */
