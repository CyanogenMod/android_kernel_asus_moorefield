/*
 * Copyright (c) 2013 Intel Corporation. All Rights Reserved.
 */

#define IA_CSS_VERSION_MAJOR    2
#define IA_CSS_VERSION_MINOR    0
#define IA_CSS_VERSION_REVISION 2

#define IA_CSS_MORPH_TABLE_NUM_PLANES  6

/** Number of DVS coefficient types */
#define IA_CSS_DVS_NUM_COEF_TYPES      6
#define IA_CSS_DVS_COEF_TYPES_ON_DMEM  2
#define IA_CSS_DVS2_NUM_COEF_TYPES     4

/** Unsigned fixed point value, 0 integer bits, 16 fractional bits */
typedef uint32_t ia_css_u0_16;
/** Unsigned fixed point value, 5 integer bits, 11 fractional bits */
typedef uint32_t ia_css_u5_11;
/** Unsigned fixed point value, 8 integer bits, 8 fractional bits */
typedef uint32_t ia_css_u8_8;
/** Signed fixed point value, 0 integer bits, 15 fractional bits */
typedef int32_t ia_css_s0_15;

struct data_offset {
	uint32_t offset; /* offset in bytes */
	uint32_t len;
};

enum ia_css_bayer_order {
	IA_CSS_BAYER_ORDER_GRBG, /**< GRGRGRGRGR .. BGBGBGBGBG */
	IA_CSS_BAYER_ORDER_RGGB, /**< RGRGRGRGRG .. GBGBGBGBGB */
	IA_CSS_BAYER_ORDER_BGGR, /**< BGBGBGBGBG .. GRGRGRGRGR */
	IA_CSS_BAYER_ORDER_GBRG  /**< GBGBGBGBGB .. RGRGRGRGRG */
};

/** Frame formats, some of these come from fourcc.org, others are
   better explained by video4linux2. The NV11 seems to be described only
   on MSDN pages, but even those seem to be gone now.
   Frames can come in many forms, the main categories are RAW, RGB and YUV
   (or YCbCr). The YUV frames come in 4 flavors, determined by how the U and V
   values are subsampled:
   1. YUV420: hor = 2, ver = 2
   2. YUV411: hor = 4, ver = 1
   3. YUV422: hor = 2, ver = 1
   4. YUV444: hor = 1, ver = 1
 */
enum ia_css_frame_format {
	IA_CSS_FRAME_FORMAT_NV11,       /**< 12 bit YUV 411, Y, UV plane */
	IA_CSS_FRAME_FORMAT_NV12,       /**< 12 bit YUV 420, Y, UV plane */
	IA_CSS_FRAME_FORMAT_NV16,       /**< 16 bit YUV 422, Y, UV plane */
	IA_CSS_FRAME_FORMAT_NV21,       /**< 12 bit YUV 420, Y, VU plane */
	IA_CSS_FRAME_FORMAT_NV61,       /**< 16 bit YUV 422, Y, VU plane */
	IA_CSS_FRAME_FORMAT_YV12,       /**< 12 bit YUV 420, Y, V, U plane */
	IA_CSS_FRAME_FORMAT_YV16,       /**< 16 bit YUV 422, Y, V, U plane */
	IA_CSS_FRAME_FORMAT_YUV420,     /**< 12 bit YUV 420, Y, U, V plane */
	IA_CSS_FRAME_FORMAT_YUV420_16,  /**< yuv420, 16 bits per subpixel */
	IA_CSS_FRAME_FORMAT_YUV422,     /**< 16 bit YUV 422, Y, U, V plane */
	IA_CSS_FRAME_FORMAT_YUV422_16,  /**< yuv422, 16 bits per subpixel */
	IA_CSS_FRAME_FORMAT_UYVY,       /**< 16 bit YUV 422, UYVY interleaved */
	IA_CSS_FRAME_FORMAT_YUYV,       /**< 16 bit YUV 422, YUYV interleaved */
	IA_CSS_FRAME_FORMAT_YUV444,     /**< 24 bit YUV 444, Y, U, V plane */
	IA_CSS_FRAME_FORMAT_YUV_LINE,   /**< Internal format, 2 y lines followed
					     by a uvinterleaved line */
	IA_CSS_FRAME_FORMAT_RAW,	/**< RAW, 1 plane */
	IA_CSS_FRAME_FORMAT_RGB565,     /**< 16 bit RGB, 1 plane. Each 3 sub
					     pixels are packed into one 16 bit
					     value, 5 bits for R, 6 bits for G
					     and 5 bits for B. */
	IA_CSS_FRAME_FORMAT_PLANAR_RGB888, /**< 24 bit RGB, 3 planes */
	IA_CSS_FRAME_FORMAT_RGBA888,	/**< 32 bit RGBA, 1 plane, A=Alpha
					     (alpha is unused) */
	IA_CSS_FRAME_FORMAT_QPLANE6, /**< Internal, for advanced ISP */
	IA_CSS_FRAME_FORMAT_BINARY_8,	/**< byte stream, used for jpeg. For
					     frames of this type, we set the
					     height to 1 and the width to the
					     number of allocated bytes. */
	IA_CSS_FRAME_FORMAT_MIPI,	/**< MIPI frame, 1 plane */
	IA_CSS_FRAME_FORMAT_RAW_PACKED  /**< RAW, 1 plane, packed */
};

/** Fractional bits for CTC gain (used only for ISP1).
 *
 *  IA_CSS_CTC_COEF_SHIFT(=13) includes not only the fractional bits
 *  of gain(=8), but also the bits(=5) to convert chroma
 *  from 13bit precision to 8bit precision.
 *
 *    Gain (struct ia_css_ctc_table) : u5.8
 *    Input(Chorma) : s0.12 (13bit precision)
 *    Output(Chorma): s0.7  (8bit precision)
 *    Output = (Input * Gain) >> IA_CSS_CTC_COEF_SHIFT
 */
#define IA_CSS_CTC_COEF_SHIFT          13

/** Number of elements in the CTC table. */
#define IA_CSS_VAMEM_1_CTC_TABLE_SIZE_LOG2      10
#define IA_CSS_VAMEM_1_CTC_TABLE_SIZE           (1U<<IA_CSS_VAMEM_1_CTC_TABLE_SIZE_LOG2)

/** Number of elements in the CTC table. */
#define IA_CSS_VAMEM_2_CTC_TABLE_SIZE_LOG2      8
#define IA_CSS_VAMEM_2_CTC_TABLE_SIZE           ((1U<<IA_CSS_VAMEM_2_CTC_TABLE_SIZE_LOG2) + 1)

enum ia_css_vamem_type {
	IA_CSS_VAMEM_TYPE_1,
	IA_CSS_VAMEM_TYPE_2
};

/** Chroma Tone Control configuration.
 *
 *  ISP block: CTC2 (CTC by polygonal line approximation)
 * (ISP1: CTC1 (CTC by look-up table) is used.)
 *  ISP2: CTC2 is used.
 */
struct ia_css_ctc_config {
	uint16_t y0;	/**< 1st kneepoint gain.
				u[ce_gain_exp].[13-ce_gain_exp], [0,8191],
				default/ineffective 4096(0.5) */
	uint16_t y1;	/**< 2nd kneepoint gain.
				u[ce_gain_exp].[13-ce_gain_exp], [0,8191],
				default/ineffective 4096(0.5) */
	uint16_t y2;	/**< 3rd kneepoint gain.
				u[ce_gain_exp].[13-ce_gain_exp], [0,8191],
				default/ineffective 4096(0.5) */
	uint16_t y3;	/**< 4th kneepoint gain.
				u[ce_gain_exp].[13-ce_gain_exp], [0,8191],
				default/ineffective 4096(0.5) */
	uint16_t y4;	/**< 5th kneepoint gain.
				u[ce_gain_exp].[13-ce_gain_exp], [0,8191],
				default/ineffective 4096(0.5) */
	uint16_t y5;	/**< 6th kneepoint gain.
				u[ce_gain_exp].[13-ce_gain_exp], [0,8191],
				default/ineffective 4096(0.5) */
	uint16_t ce_gain_exp;	/**< Common exponent of y-axis gain.
				u8.0, [0,13],
				default/ineffective 1 */
	uint16_t x1;	/**< 2nd kneepoint luma.
				u0.13, [0,8191], constraints: 0<x1<x2,
				default/ineffective 1024 */
	uint16_t x2;	/**< 3rd kneepoint luma.
				u0.13, [0,8191], constraints: x1<x2<x3,
				default/ineffective 2048 */
	uint16_t x3;	/**< 4th kneepoint luma.
				u0.13, [0,8191], constraints: x2<x3<x4,
				default/ineffective 6144 */
	uint16_t x4;	/**< 5tn kneepoint luma.
				u0.13, [0,8191], constraints: x3<x4<8191,
				default/ineffective 7168 */
};

union ia_css_ctc_data {
	uint16_t vamem_1[IA_CSS_VAMEM_1_CTC_TABLE_SIZE];
	uint16_t vamem_2[IA_CSS_VAMEM_2_CTC_TABLE_SIZE];
};

/** Configuration used by Bayer Noise Reduction (BNR) and
 *  YCC Noise Reduction (YNR,CNR).
 *
 *  ISP block: BNR1, YNR1, CNR1
 *  ISP1: BNR1,YNR1,CNR1 are used.
 *  ISP2: BNR1,YNR1,CNR1 are used for Preview/Video.
 *        BNR1,YNR2,CNR2 are used for Still.
 */
struct ia_css_nr_config {
	ia_css_u0_16 bnr_gain;	   /**< Strength of noise reduction (BNR).
				u0.16, [0,65535],
				default 14336(0.21875), ineffective 0 */
	ia_css_u0_16 ynr_gain;	   /**< Strength of noise reduction (YNR).
				u0.16, [0,65535],
				default 14336(0.21875), ineffective 0 */
	ia_css_u0_16 direction;    /**< Sensitivity of edge (BNR).
				u0.16, [0,65535],
				default 512(0.0078125), ineffective 0 */
	ia_css_u0_16 threshold_cb; /**< Coring threshold for Cb (CNR).
				This is the same as
				de_config.c1_coring_threshold.
				u0.16, [0,65535],
				default 0(0), ineffective 0 */
	ia_css_u0_16 threshold_cr; /**< Coring threshold for Cr (CNR).
				This is the same as
				de_config.c2_coring_threshold.
				u0.16, [0,65535],
				default 0(0), ineffective 0 */
};

/** Edge Enhancement (sharpen) configuration.
 *
 *  ISP block: YEE1
 *  ISP1: YEE1 is used.
 *  ISP2: YEE1 is used for Preview/Video.
 *       (YEE2 is used for Still.)
 */
struct ia_css_ee_config {
	ia_css_u5_11 gain;	  /**< The strength of sharpness.
					u5.11, [0,65535],
					default 8192(4.0), ineffective 0 */
	ia_css_u8_8 threshold;    /**< The threshold that divides noises from
					edge.
					u8.8, [0,65535],
					default 256(1.0), ineffective 65535 */
	ia_css_u5_11 detail_gain; /**< The strength of sharpness in pell-mell
					area.
					u5.11, [0,65535],
					default 2048(1.0), ineffective 0 */
};

/** YNR and YEE (sharpen) configuration.
 */
struct ia_css_yee_config {
	struct ia_css_nr_config nr; /**< The NR configuration. */
	struct ia_css_ee_config ee; /**< The EE configuration. */
};

/** Y(Luma) Noise Reduction configuration.
 *
 *  ISP block: YNR2 & YEE2
 * (ISP1: YNR1 and YEE1 are used.)
 * (ISP2: YNR1 and YEE1 are used for Preview/Video.)
 *  ISP2: YNR2 and YEE2 are used for Still.
 */
struct ia_css_ynr_config {
	uint16_t edge_sense_gain_0;   /**< Sensitivity of edge in dark area.
					u13.0, [0,8191],
					default 1000, ineffective 0 */
	uint16_t edge_sense_gain_1;   /**< Sensitivity of edge in bright area.
					u13.0, [0,8191],
					default 1000, ineffective 0 */
	uint16_t corner_sense_gain_0; /**< Sensitivity of corner in dark area.
					u13.0, [0,8191],
					default 1000, ineffective 0 */
	uint16_t corner_sense_gain_1; /**< Sensitivity of corner in bright area.
					u13.0, [0,8191],
					default 1000, ineffective 0 */
};

/** Fringe Control configuration.
 *
 *  ISP block: FC2 (FC2 is used with YNR2/YEE2.)
 * (ISP1: FC2 is not used.)
 * (ISP2: FC2 is not for Preview/Video.)
 *  ISP2: FC2 is used for Still.
 */
struct ia_css_fc_config {
	uint8_t  gain_exp;   /**< Common exponent of gains.
				u8.0, [0,13],
				default 1, ineffective 0 */
	uint16_t coring_pos_0; /**< Coring threshold for positive edge in dark area.
				u0.13, [0,8191],
				default 0(0), ineffective 0 */
	uint16_t coring_pos_1; /**< Coring threshold for positive edge in bright area.
				u0.13, [0,8191],
				default 0(0), ineffective 0 */
	uint16_t coring_neg_0; /**< Coring threshold for negative edge in dark area.
				u0.13, [0,8191],
				default 0(0), ineffective 0 */
	uint16_t coring_neg_1; /**< Coring threshold for negative edge in bright area.
				u0.13, [0,8191],
				default 0(0), ineffective 0 */
	uint16_t gain_pos_0; /**< Gain for positive edge in dark area.
				u0.13, [0,8191],
				default 4096(0.5), ineffective 0 */
	uint16_t gain_pos_1; /**< Gain for positive edge in bright area.
				u0.13, [0,8191],
				default 4096(0.5), ineffective 0 */
	uint16_t gain_neg_0; /**< Gain for negative edge in dark area.
				u0.13, [0,8191],
				default 4096(0.5), ineffective 0 */
	uint16_t gain_neg_1; /**< Gain for negative edge in bright area.
				u0.13, [0,8191],
				default 4096(0.5), ineffective 0 */
	uint16_t crop_pos_0; /**< Limit for positive edge in dark area.
				u0.13, [0,8191],
				default/ineffective 8191(almost 1.0) */
	uint16_t crop_pos_1; /**< Limit for positive edge in bright area.
				u0.13, [0,8191],
				default/ineffective 8191(almost 1.0) */
	int16_t  crop_neg_0; /**< Limit for negative edge in dark area.
				s0.13, [-8192,0],
				default/ineffective -8192(-1.0) */
	int16_t  crop_neg_1; /**< Limit for negative edge in bright area.
				s0.13, [-8192,0],
				default/ineffective -8192(-1.0) */
};

/** XNR table.
 *
 *  NOTE: The driver does not need to set this table,
 *        because the default values are set inside the css.
 *
 *  This table contains coefficients used for division in XNR.
 *
 *	u0.12, [0,4095],
 *      {4095, 2048, 1365, .........., 65, 64}
 *      ({1/1, 1/2, 1/3, ............., 1/63, 1/64})
 *
 *  ISP block: XNR1
 *  ISP1: XNR1 is used.
 *  ISP2: XNR1 is used.
 *
 */

/** Number of elements in the xnr table. */
#define IA_CSS_VAMEM_1_XNR_TABLE_SIZE_LOG2      6
#define IA_CSS_VAMEM_1_XNR_TABLE_SIZE           (1U<<IA_CSS_VAMEM_1_XNR_TABLE_SIZE_LOG2)

/** Number of elements in the xnr table. */
#define IA_CSS_VAMEM_2_XNR_TABLE_SIZE_LOG2      6
#define IA_CSS_VAMEM_2_XNR_TABLE_SIZE	        (1U<<IA_CSS_VAMEM_2_XNR_TABLE_SIZE_LOG2)

/**< IA_CSS_VAMEM_TYPE_1(ISP2300) or
     IA_CSS_VAMEM_TYPE_2(ISP2400) */
union ia_css_xnr_data {
	uint16_t vamem_1[IA_CSS_VAMEM_1_XNR_TABLE_SIZE];
	/**< Coefficients table on vamem type1. u0.12, [0,4095] */
	uint16_t vamem_2[IA_CSS_VAMEM_2_XNR_TABLE_SIZE];
	/**< Coefficients table on vamem type2. u0.12, [0,4095] */
};

struct ia_css_xnr_table {
	enum ia_css_vamem_type vamem_type;
	union ia_css_xnr_data data;
};

struct ia_css_xnr_config {
	/** XNR threshold.
	 * type:u0.16 valid range:[0,65535]
	 * default: 25 */
	uint16_t threshold;
};

/** White Balance configuration (Gain Adjust).
 *
 *  ISP block: WB1
 *  ISP1: WB1 is used.
 *  ISP2: WB1 is used.
 */
struct ia_css_wb_config {
	uint32_t integer_bits; /**< Common exponent of gains.
				u8.0, [0,3],
				default 1, ineffective 1 */
	uint32_t gr;	/**< Significand of Gr gain.
				u[integer_bits].[16-integer_bits], [0,65535],
				default/ineffective 32768(u1.15, 1.0) */
	uint32_t r;	/**< Significand of R gain.
				u[integer_bits].[16-integer_bits], [0,65535],
				default/ineffective 32768(u1.15, 1.0) */
	uint32_t b;	/**< Significand of B gain.
				u[integer_bits].[16-integer_bits], [0,65535],
				default/ineffective 32768(u1.15, 1.0) */
	uint32_t gb;	/**< Significand of Gb gain.
				u[integer_bits].[16-integer_bits], [0,65535],
				default/ineffective 32768(u1.15, 1.0) */
};

/** Viewfinder decimation
 *
 *  ISP block: vfeven_horizontal_downscale
 */
struct ia_css_vf_configuration {
	uint32_t vf_downscale_bits; /**< Log VF downscale value */
};

/** Temporal Noise Reduction (TNR) configuration.
 *
 *  When difference between current frame and previous frame is less than or
 *  equal to threshold, TNR works and current frame is mixed
 *  with previous frame.
 *  When difference between current frame and previous frame is greater
 *  than threshold, we judge motion is detected. Then, TNR does not work and
 *  current frame is outputted as it is.
 *  Therefore, when threshold_y and threshold_uv are set as 0, TNR can be disabled.
 *
 *  ISP block: TNR1
 *  ISP1: TNR1 is used.
 *  ISP2: TNR1 is used.
 */
struct ia_css_tnr_config {
	ia_css_u0_16 gain; /**< Interpolation ratio of current frame
			        and previous frame.
				gain=0.0 -> previous frame is outputted.
				gain=1.0 -> current frame is outputted.
				u0.16, [0,65535],
			default 32768(0.5), ineffective 65535(almost 1.0) */
	ia_css_u0_16 threshold_y; /**< Threshold to enable interpolation of Y.
				If difference between current frame and
				previous frame is greater than threshold_y,
				TNR for Y is disabled.
				u0.16, [0,65535], default/ineffective 0 */
	ia_css_u0_16 threshold_uv; /**< Threshold to enable interpolation of
				U/V.
				If difference between current frame and
				previous frame is greater than threshold_uv,
				TNR for UV is disabled.
				u0.16, [0,65535], default/ineffective 0 */
};

/** Number of color planes in the shading table. */
#define IA_CSS_SC_NUM_COLORS           4

/** The 4 colors that a shading table consists of.
 *  For each color we store a grid of values.
 */
enum ia_css_sc_color {
	IA_CSS_SC_COLOR_GR, /**< Green on a green-red line */
	IA_CSS_SC_COLOR_R,  /**< Red */
	IA_CSS_SC_COLOR_B,  /**< Blue */
	IA_CSS_SC_COLOR_GB  /**< Green on a green-blue line */
};

/** Lens Shading Correction table.
 *
 *  This describes the color shading artefacts
 *  introduced by lens imperfections. To correct artefacts,
 *  bayer values should be multiplied by gains in this table.
 *
 *  ISP block: SC1
 *  ISP1: SC1 is used.
 *  ISP2: SC1 is used.
 */
struct ia_css_shading_table {
	uint32_t enable; /**< Set to false for no shading correction.
		          The data field can be NULL when enable == true */
	uint32_t sensor_width;  /**< Native sensor width in pixels. */
	uint32_t sensor_height; /**< Native sensor height in lines. */
	uint32_t width;  /**< Number of data points per line per color.
				u8.0, [0,81] */
	uint32_t height; /**< Number of lines of data points per color.
				u8.0, [0,61] */
	uint32_t fraction_bits; /**< Bits of fractional part in the data
				points.
				u8.0, [0,13] */
	/*uint16_t *data[IA_CSS_SC_NUM_COLORS];*/
	struct data_offset data[IA_CSS_SC_NUM_COLORS];
	/**< Table data, one array for each color.
	     Use ia_css_sc_color to index this array.
	     u[13-fraction_bits].[fraction_bits], [0,8191] */
};

/** 3A statistics grid
 *
 *  ISP block: S3A1 (3A Support for 3A ver.1 (Histogram is not used for AE))
 *             S3A2 (3A Support for 3A ver.2 (Histogram is used for AE))
 *  ISP1: S3A1 is used.
 *  ISP2: S3A2 is used.
 */
struct ia_css_3a_grid_info {

	uint32_t enable;            /**< 3A statistics enabled.
					0:disabled, 1:enabled */
	uint32_t use_dmem;          /**< DMEM or VMEM determines layout.
					0:3A statistics are stored to VMEM,
					1:3A statistics are stored to DMEM */
	uint32_t has_histogram;     /**< Statistics include histogram.
					0:no histogram, 1:has histogram */
	uint32_t width;		    /**< Width of 3A grid table.
					(= Horizontal number of grid cells
					in table, which cells have effective
					statistics.) */
	uint32_t height;	    /**< Height of 3A grid table.
					(= Vertical number of grid cells
					in table, which cells have effective
					statistics.) */
	uint32_t aligned_width;     /**< Horizontal stride (for alloc).
					(= Horizontal number of grid cells
					in table, which means
					the allocated width.) */
	uint32_t aligned_height;    /**< Vertical stride (for alloc).
					(= Vertical number of grid cells
					in table, which means
					the allocated height.) */
	uint32_t bqs_per_grid_cell; /**< Grid cell size in BQ(Bayer Quad) unit.
					(1BQ means {Gr,R,B,Gb}(2x2 pixels).)
					Valid values are 8,16,32,64. */
	uint32_t deci_factor_log2;  /**< log2 of bqs_per_grid_cell. */
	uint32_t elem_bit_depth;    /**< Bit depth of element used
					to calculate 3A statistics.
					This is 13, which is the normalized
					bayer bit depth in DSP. */
};

/* This struct should be split into 3, for AE, AWB and AF.
 * However, that will require driver/ 3A lib modifications.
 */

/** 3A configuration. This configures the 3A statistics collection
 *  module.
 *
 *  ae_y_*: Coefficients to calculate luminance from bayer.
 *  awb_lg_*: Thresholds to check the saturated bayer pixels for AWB.
 *    Condition of effective pixel for AWB level gate check:
 *      bayer(sensor) <= awb_lg_high_raw &&
 *      bayer(when AWB statisitcs is calculated) >= awb_lg_low &&
 *      bayer(when AWB statisitcs is calculated) <= awb_lg_high
 *  af_fir*: Coefficients of high pass filter to calculate AF statistics.
 *
 *  ISP block: S3A1(ae_y_* for AE/AF, awb_lg_* for AWB)
 *             S3A2(ae_y_* for AF, awb_lg_* for AWB)
 *             SDVS1(ae_y_*)
 *             SDVS2(ae_y_*)
 *  ISP1: S3A1 and SDVS1 are used.
 *  ISP2: S3A2 and SDVS2 are used.
 */
struct ia_css_3a_config {
	ia_css_u0_16 ae_y_coef_r;	/**< Weight of R for Y.
						u0.16, [0,65535],
						default/ineffective 25559 */
	ia_css_u0_16 ae_y_coef_g;	/**< Weight of G for Y.
						u0.16, [0,65535],
						default/ineffective 32768 */
	ia_css_u0_16 ae_y_coef_b;	/**< Weight of B for Y.
						u0.16, [0,65535],
						default/ineffective 7209 */
	ia_css_u0_16 awb_lg_high_raw;	/**< AWB level gate high for raw.
						u0.16, [0,65535],
						default 65472(=1023*64),
						ineffective 65535 */
	ia_css_u0_16 awb_lg_low;	/**< AWB level gate low.
						u0.16, [0,65535],
						default 64(=1*64),
						ineffective 0 */
	ia_css_u0_16 awb_lg_high;	/**< AWB level gate high.
						u0.16, [0,65535],
						default 65535,
						ineffective 65535 */
	ia_css_s0_15 af_fir1_coef[7];	/**< AF FIR coefficients of fir1.
						s0.15, [-32768,32767],
				default/ineffective
				-6689,-12207,-32768,32767,12207,6689,0 */
	ia_css_s0_15 af_fir2_coef[7];	/**< AF FIR coefficients of fir2.
						s0.15, [-32768,32767],
				default/ineffective
				2053,0,-18437,32767,-18437,2053,0 */
};

/** 3A statistics. This structure describes the data stored
 *  in each 3A grid point.
 *
 *  ISP block: S3A1 (3A Support for 3A ver.1) (Histogram is not used for AE)
 *             S3A2 (3A Support for 3A ver.2) (Histogram is used for AE)
 *             - ae_y is used only for S3A1.
 *             - awb_* and af_* are used both for S3A1 and S3A2.
 *  ISP1: S3A1 is used.
 *  ISP2: S3A2 is used.
 */
struct ia_css_3a_output {
	int32_t ae_y;    /**< Sum of Y in a statistics window, for AE.
				(u19.13) */
	int32_t awb_cnt; /**< Number of effective pixels
				in a statistics window.
				Pixels passed by the AWB level gate check are
				judged as "effective". (u32) */
	int32_t awb_gr;  /**< Sum of Gr in a statistics window, for AWB.
				All Gr pixels (not only for effective pixels)
				are summed. (u19.13) */
	int32_t awb_r;   /**< Sum of R in a statistics window, for AWB.
				All R pixels (not only for effective pixels)
				are summed. (u19.13) */
	int32_t awb_b;   /**< Sum of B in a statistics window, for AWB.
				All B pixels (not only for effective pixels)
				are summed. (u19.13) */
	int32_t awb_gb;  /**< Sum of Gb in a statistics window, for AWB.
				All Gb pixels (not only for effective pixels)
				are summed. (u19.13) */
	int32_t af_hpf1; /**< Sum of |Y| following high pass filter af_fir1
				within a statistics window, for AF. (u19.13) */
	int32_t af_hpf2; /**< Sum of |Y| following high pass filter af_fir2
				within a statistics window, for AF. (u19.13) */
};


/** 3A Statistics. This structure describes the statistics that are generated
 *  using the provided configuration (ia_css_3a_config).
 */
struct ia_css_3a_statistics {
	struct ia_css_3a_grid_info    grid;	/**< grid info contains the dimensions of the 3A grid */
	struct ia_css_3a_output      *data;	/**< the pointer to 3a_output[grid.width * grid.height]
						     containing the 3A statistics */
	struct ia_css_3a_rgby_output *rgby_data;/**< the pointer to 3a_rgby_output[256]
						     containing the histogram */
};

/** Histogram (Statistics for AE).
 *
 *  4 histograms(r,g,b,y),
 *  256 bins for each histogram, unsigned 24bit value for each bin.
 *    struct ia_css_3a_rgby_output data[256];

 *  ISP block: HIST2
 * (ISP1: HIST2 is not used.)
 *  ISP2: HIST2 is used.
 */
struct ia_css_3a_rgby_output {
	uint32_t r;	/**< Number of R of one bin of the histogram R. (u24) */
	uint32_t g;	/**< Number of G of one bin of the histogram G. (u24) */
	uint32_t b;	/**< Number of B of one bin of the histogram B. (u24) */
	uint32_t y;	/**< Number of Y of one bin of the histogram Y. (u24) */
};

/** Optical black mode.
 */
enum ia_css_ob_mode {
	IA_CSS_OB_MODE_NONE,	/**< OB has no effect. */
	IA_CSS_OB_MODE_FIXED,	/**< Fixed OB */
	IA_CSS_OB_MODE_RASTER	/**< Raster OB */
};

/** Optical Black level configuration.
 *
 *  ISP block: OB1
 *  ISP1: OB1 is used.
 *  ISP2: OB1 is used.
 */
struct ia_css_ob_config {
	enum ia_css_ob_mode mode; /**< Mode (None / Fixed / Raster).
					enum, [0,2],
					default 1, ineffective 0 */
	ia_css_u0_16 level_gr;    /**< Black level for GR pixels
					(used for Fixed Mode only).
					u0.16, [0,65535],
					default/ineffective 0 */
	ia_css_u0_16 level_r;     /**< Black level for R pixels
					(used for Fixed Mode only).
					u0.16, [0,65535],
					default/ineffective 0 */
	ia_css_u0_16 level_b;     /**< Black level for B pixels
					(used for Fixed Mode only).
					u0.16, [0,65535],
					default/ineffective 0 */
	ia_css_u0_16 level_gb;    /**< Black level for GB pixels
					(used for Fixed Mode only).
					u0.16, [0,65535],
					default/ineffective 0 */
	uint16_t start_position; /**< Start position of OB area
					(used for Raster Mode only).
					u16.0, [0,63],
					default/ineffective 0 */
	uint16_t end_position;  /**< End position of OB area
					(used for Raster Mode only).
					u16.0, [0,63],
					default/ineffective 0 */
};

/** Number of axes in the MACC table. */
#define IA_CSS_MACC_NUM_AXES           16
/** Number of coefficients per MACC axes. */
#define IA_CSS_MACC_NUM_COEFS          4
/** The number of planes in the morphing table. */

/** Multi-Axes Color Correction (MACC) table.
 *
 *  ISP block: MACC1 (MACC by only matrix)
 *             MACC2 (MACC by matrix and exponent(ia_css_macc_config))
 *  ISP1: MACC1 is used.
 *  ISP2: MACC2 is used.
 *
 *  [MACC1]
 *   OutU = (data00 * InU + data01 * InV) >> 13
 *   OutV = (data10 * InU + data11 * InV) >> 13
 *
 *   default/ineffective:
 *   OutU = (8192 * InU +    0 * InV) >> 13
 *   OutV = (   0 * InU + 8192 * InV) >> 13
 *
 *  [MACC2]
 *   OutU = (data00 * InU + data01 * InV) >> (13 - exp)
 *   OutV = (data10 * InU + data11 * InV) >> (13 - exp)
 *
 *   default/ineffective: (exp=1)
 *   OutU = (4096 * InU +    0 * InV) >> (13 - 1)
 *   OutV = (   0 * InU + 4096 * InV) >> (13 - 1)
 */

struct ia_css_macc_table {
	int16_t data[IA_CSS_MACC_NUM_COEFS * IA_CSS_MACC_NUM_AXES];
	/**< 16 of 2x2 matix
	  MACC1: s2.13, [-65536,65535]
	    default/ineffective:
		16 of "identity 2x2 matix" {8192,0,0,8192}
	  MACC2: s[macc_config.exp].[13-macc_config.exp], [-8192,8191]
	    default/ineffective: (s1.12)
		16 of "identity 2x2 matix" {4096,0,0,4096} */
};
/** sRGB Gamma table, used for sRGB Gamma Correction.
 *
 *  ISP block: GC2 (sRGB Gamma Correction)
 * (ISP1: GC1(YUV Gamma Correction) is used.)
 *  ISP2: GC2 is used.
 */

/** Number of elements in the sRGB gamma table. */
#define IA_CSS_VAMEM_1_RGB_GAMMA_TABLE_SIZE_LOG2 8
#define IA_CSS_VAMEM_1_RGB_GAMMA_TABLE_SIZE      (1U<<IA_CSS_VAMEM_1_RGB_GAMMA_TABLE_SIZE_LOG2)

/** Number of elements in the sRGB gamma table. */
#define IA_CSS_VAMEM_2_RGB_GAMMA_TABLE_SIZE_LOG2    8
#define IA_CSS_VAMEM_2_RGB_GAMMA_TABLE_SIZE     ((1U<<IA_CSS_VAMEM_2_RGB_GAMMA_TABLE_SIZE_LOG2) + 1)

/**< IA_CSS_VAMEM_TYPE_1(ISP2300) or
     IA_CSS_VAMEM_TYPE_2(ISP2400) */
union ia_css_rgb_gamma_data {
	uint16_t vamem_1[IA_CSS_VAMEM_1_RGB_GAMMA_TABLE_SIZE];
	/**< RGB Gamma table on vamem type1. This table is not used,
		because sRGB Gamma Correction is not implemented for ISP2300. */
	uint16_t vamem_2[IA_CSS_VAMEM_2_RGB_GAMMA_TABLE_SIZE];
		/**< RGB Gamma table on vamem type2. u0.12, [0,4095] */
};

struct ia_css_rgb_gamma_table {
	enum ia_css_vamem_type vamem_type;
	union ia_css_rgb_gamma_data data;
};

/** Number of elements in the gamma table. */
#define IA_CSS_VAMEM_1_GAMMA_TABLE_SIZE_LOG2    10
#define IA_CSS_VAMEM_1_GAMMA_TABLE_SIZE         (1U<<IA_CSS_VAMEM_1_GAMMA_TABLE_SIZE_LOG2)

/** Number of elements in the gamma table. */
#define IA_CSS_VAMEM_2_GAMMA_TABLE_SIZE_LOG2    8
#define IA_CSS_VAMEM_2_GAMMA_TABLE_SIZE         ((1U<<IA_CSS_VAMEM_2_GAMMA_TABLE_SIZE_LOG2) + 1)

/** Gamma table, used for Y(Luma) Gamma Correction.
 *
 *  ISP block: GC1 (YUV Gamma Correction)
 *  ISP1: GC1 is used.
 * (ISP2: GC2(sRGB Gamma Correction) is used.)
 */
/**< IA_CSS_VAMEM_TYPE_1(ISP2300) or
     IA_CSS_VAMEM_TYPE_2(ISP2400) */
union ia_css_gc_data {
	uint16_t vamem_1[IA_CSS_VAMEM_1_GAMMA_TABLE_SIZE];
	/**< Y(Luma) Gamma table on vamem type 1. u0.8, [0,255] */
	uint16_t vamem_2[IA_CSS_VAMEM_2_GAMMA_TABLE_SIZE];
	/**< Y(Luma) Gamma table on vamem type 2. u0.8, [0,255] */
};

struct ia_css_gamma_table {
	enum ia_css_vamem_type vamem_type;
	union ia_css_gc_data data;
};

/** Gamma Correction configuration (used only for YUV Gamma Correction).
 *
 *  ISP block: GC1 (YUV Gamma Correction)
 *  ISP1: GC1 is used.
 * (ISP2: GC2 (sRGB Gamma Correction) is used.)
  */
struct ia_css_gc_config {
	uint16_t gain_k1; /**< Gain to adjust U after YUV Gamma Correction.
				u0.16, [0,65535],
				default/ineffective 19000(0.29) */
	uint16_t gain_k2; /**< Gain to adjust V after YUV Gamma Correction.
				u0.16, [0,65535],
				default/ineffective 19000(0.29) */
};

/** Chroma Enhancement configuration.
 *
 *  This parameter specifies range of chroma output level.
 *  The standard range is [0,255] or [16,240].
 *
 *  ISP block: CE1
 *  ISP1: CE1 is used.
 * (ISP2: CE1 is not used.)
 */
struct ia_css_ce_config {
	uint8_t uv_level_min; /**< Minimum of chroma output level.
				u0.8, [0,255], default/ineffective 0 */
	uint8_t uv_level_max; /**< Maximum of chroma output level.
				u0.8, [0,255], default/ineffective 255 */
};

/** Multi-Axes Color Correction (MACC) configuration.
 *
 *  ISP block: MACC2 (MACC by matrix and exponent(ia_css_macc_config))
 * (ISP1: MACC1 (MACC by only matrix) is used.)
 *  ISP2: MACC2 is used.
 */
struct ia_css_macc_config {
	uint8_t exp;	/**< Common exponent of ia_css_macc_table.
				u8.0, [0,13], default 1, ineffective 1 */
};

/** Fixed Pattern Noise table.
 *
 *  This contains the fixed patterns noise values
 *  obtained from a black frame capture.
 *
 *  "shift" should be set as the smallest value
 *  which satisfies the requirement the maximum data is less than 64.
 *
 *  ISP block: FPN1
 *  ISP1: FPN1 is used.
 *  ISP2: FPN1 is used.
 */
struct ia_css_fpn_table {
	/*int16_t *data;*/		/**< Table content (fixed patterns noise).
					u0.[13-shift], [0,63] */
	struct data_offset data;
	uint32_t width;		/**< Table width (in pixels).
					This is the input frame width. */
	uint32_t height;	/**< Table height (in pixels).
					This is the input frame height. */
	uint32_t shift;		/**< Common exponent of table content.
					u8.0, [0,13] */
	uint32_t enabled;	/**< Fpn is enabled.
					bool */
};

struct sh_css_bds_factor {
	unsigned numerator;
	unsigned denominator;
	unsigned int bds_factor;
};

/** Defect Pixel Correction configuration.
 *
 *  ISP block: DPC1 (DPC after WB)
 *             DPC2 (DPC before WB)
 *  ISP1: DPC1 is used.
 *  ISP2: DPC2 is used.
 */
struct ia_css_dp_config {
	ia_css_u0_16 threshold; /**< The threshold of defect pixel correction,
			      representing the permissible difference of
			      intensity between one pixel and its
			      surrounding pixels. Smaller values result
				in more frequent pixel corrections.
				u0.16, [0,65535],
				default 8192, ineffective 65535 */
	ia_css_u8_8 gain;	 /**< The sensitivity of mis-correction. ISP will
			      miss a lot of defects if the value is set
				too large.
				u8.8, [0,65535],
				default 4096, ineffective 65535 */
	uint32_t gr;	/* unsigned <integer_bits>.<16-integer_bits> */
	uint32_t r;	/* unsigned <integer_bits>.<16-integer_bits> */
	uint32_t b;	/* unsigned <integer_bits>.<16-integer_bits> */
	uint32_t gb;	/* unsigned <integer_bits>.<16-integer_bits> */
};


/** Demosaic (bayer-to-YCgCo) configuration.
 *
 *  ISP block: DE1
 *  ISP1: DE1 is used.
 * (ISP2: DE2 is used.)
 */
struct ia_css_de_config {
	ia_css_u0_16 pixelnoise; /**< Pixel noise used in moire elimination.
				u0.16, [0,65535],
				default 0, ineffective 0 */
	ia_css_u0_16 c1_coring_threshold; /**< Coring threshold for C1.
				This is the same as nr_config.threshold_cb.
				u0.16, [0,65535],
				default 128(0.001953125), ineffective 0 */
	ia_css_u0_16 c2_coring_threshold; /**< Coring threshold for C2.
				This is the same as nr_config.threshold_cr.
				u0.16, [0,65535],
				default 128(0.001953125), ineffective 0 */
};

/** Eigen Color Demosaicing configuration.
 *
 *  ISP block: DE2
 * (ISP1: DE1 is used.)
 *  ISP2: DE2 is used.
 */
struct ia_css_ecd_config {
	uint16_t zip_strength;	/**< Strength of zipper reduction.
				u0.13, [0,8191],
				default 5489(0.67), ineffective 0 */
	uint16_t fc_strength;	/**< Strength of false color reduction.
				u0.13, [0,8191],
				default 8191(almost 1.0), ineffective 0 */
	uint16_t fc_debias;	/**< Prevent color change
				     on noise or Gr/Gb imbalance.
				u0.13, [0,8191],
				default 0, ineffective 0 */
};

/** CTC table, used for Chroma Tone Control.
 *
 *  ISP block: CTC1 (CTC by look-up table)
 *  ISP1: CTC1 is used.
 * (ISP2: CTC2 (CTC by polygonal line approximation) is used.)
 */
struct ia_css_ctc_table {
	enum ia_css_vamem_type vamem_type;
	union ia_css_ctc_data data;
};


/** Color Correction configuration.
 *
 *  This structure is used for 3 cases.
 *  ("YCgCo" is the output format of Demosaic.)
 *
 *  1. Color Space Conversion (YCgCo to YUV) for ISP1.
 *     ISP block: CSC1 (Color Space Conversion)
 *     struct ia_css_cc_config   *cc_config
 *
 *  2. Color Correction Matrix (YCgCo to RGB) for ISP2.
 *     ISP block: CCM2 (Color Correction Matrix)
 *     struct ia_css_cc_config   *yuv2rgb_cc_config
 *
 *  3. Color Space Conversion (RGB to YUV) for ISP2.
 *     ISP block: CSC2 (Color Space Conversion)
 *     struct ia_css_cc_config   *rgb2yuv_cc_config
 *
 *  default/ineffective:
 *  1. YCgCo -> YUV
 *	1	0.174		0.185
 *	0	-0.66252	-0.66874
 *	0	-0.83738	0.58131
 *
 *	fraction_bits = 12
 *	4096	713	758
 *	0	-2714	-2739
 *	0	-3430	2381
 *
 *  2. YCgCo -> RGB
 *	1	-1	1
 *	1	1	0
 *	1	-1	-1
 *
 *	fraction_bits = 12
 *	4096	-4096	4096
 *	4096	4096	0
 *	4096	-4096	-4096
 *
 *  3. RGB -> YUV
 *	0.299	   0.587	0.114
 *	-0.16874   -0.33126	0.5
 *	0.5	   -0.41869	-0.08131
 *
 *	fraction_bits = 13
 *	2449	4809	934
 *	-1382	-2714	4096
 *	4096	-3430	-666
 */
struct ia_css_cc_config {
	uint32_t fraction_bits;/**< Fractional bits of matrix.
					u8.0, [0,13] */
	int32_t matrix[3 * 3]; /**< Conversion matrix.
					s[13-fraction_bits].[fraction_bits],
					[-8192,8191] */
};


/** Chroma Noise Reduction configuration.
 *
 *  Small sensitivity of edge means strong smoothness and NR performance.
 *  If you see blurred color on vertical edges,
 *  set higher values on sense_gain_h*.
 *  If you see blurred color on horizontal edges,
 *  set higher values on sense_gain_v*.
 *
 *  ISP block: CNR2
 * (ISP1: CNR1 is used.)
 * (ISP2: CNR1 is used for Preview/Video.)
 *  ISP2: CNR2 is used for Still.
 */
struct ia_css_cnr_config {
	uint16_t coring_u;	/**< Coring level of U.
				u0.13, [0,8191], default/ineffective 0 */
	uint16_t coring_v;	/**< Coring level of V.
				u0.13, [0,8191], default/ineffective 0 */
	uint16_t sense_gain_vy;	/**< Sensitivity of horizontal edge of Y.
				u13.0, [0,8191], default 100, ineffective 0 */
	uint16_t sense_gain_vu;	/**< Sensitivity of horizontal edge of U.
				u13.0, [0,8191], default 100, ineffective 0 */
	uint16_t sense_gain_vv;	/**< Sensitivity of horizontal edge of V.
				u13.0, [0,8191], default 100, ineffective 0 */
	uint16_t sense_gain_hy;	/**< Sensitivity of vertical edge of Y.
				u13.0, [0,8191], default 50, ineffective 0 */
	uint16_t sense_gain_hu;	/**< Sensitivity of vertical edge of U.
				u13.0, [0,8191], default 50, ineffective 0 */
	uint16_t sense_gain_hv;	/**< Sensitivity of vertical edge of V.
				u13.0, [0,8191], default 50, ineffective 0 */
};

#define ISP_HIST_DEPTH                         1024
#define ISP_HIST_WIDTH                         24
#define ISP_HIST_COMPONENTS                    4

/** Number of elements in the BH table.
  * Should be consistent with hmem.h
  */
#define IA_CSS_HMEM_BH_TABLE_SIZE	ISP_HIST_DEPTH
#define IA_CSS_HMEM_BH_UNIT_SIZE	(ISP_HIST_DEPTH/ISP_HIST_COMPONENTS)

#define BH_COLOR_R	(0)
#define BH_COLOR_G	(1)
#define BH_COLOR_B	(2)
#define BH_COLOR_Y	(3)
#define BH_COLOR_NUM	(4)

/** BH table */
struct ia_css_bh_table {
	uint32_t hmem[ISP_HIST_COMPONENTS][IA_CSS_HMEM_BH_UNIT_SIZE];
};

struct ia_css_anr_config {
	int32_t threshold; /**< Threshold */
	int32_t thresholds[4*4*4];
	int32_t factors[3];
};

struct ia_css_anr_thres {
	int16_t data[13*64];
};

struct ia_css_aa_config {
	uint16_t strength;	/**< Strength of the filter.
					u0.13, [0,8191],
					default/ineffective 0 */
};

/* Virtual address within the CSS address space. */
typedef uint32_t ia_css_ptr;

/** Vector with signed values. This is used to indicate motion for
 * Digital Image Stabilization.
 */
struct ia_css_vector {
	int32_t x; /**< horizontal motion (in pixels) */
	int32_t y; /**< vertical motion (in pixels) */
};

/** DVS statistics grid
 *
 *  ISP block: SDVS1 (DIS/DVS Support for DIS/DVS ver.1 (2-axes))
 *             SDVS2 (DVS Support for DVS ver.2 (6-axes))
 *  ISP1: SDVS1 is used.
 *  ISP2: SDVS2 is used.
 */
struct ia_css_dvs_grid_info {
	uint32_t enable;        /**< DVS statistics enabled.
					0:disabled, 1:enabled */
	uint32_t width;		/**< Width of DVS grid table.
					(= Horizontal number of grid cells
					in table, which cells have effective
					statistics.)
					For DVS1, this is equal to
					 the number of vertical statistics. */
	uint32_t aligned_width; /**< Stride of each grid line.
					(= Horizontal number of grid cells
					in table, which means
					the allocated width.) */
	uint32_t height;	/**< Height of DVS grid table.
					(= Vertical number of grid cells
					in table, which cells have effective
					statistics.)
					For DVS1, This is equal to
					the number of horizontal statistics. */
	uint32_t aligned_height;/**< Stride of each grid column.
					(= Vertical number of grid cells
					in table, which means
					the allocated height.) */
	uint32_t bqs_per_grid_cell; /**< Grid cell size in BQ(Bayer Quad) unit.
					(1BQ means {Gr,R,B,Gb}(2x2 pixels).)
					For DVS1, valid value is 64.
					For DVS2, valid value is only 64,
					currently. */
	uint32_t num_hor_coefs;	/**< Number of horizontal coefficients. */
	uint32_t num_ver_coefs;	/**< Number of vertical coefficients. */
};

/** structure that describes the 3A and DIS grids */
struct ia_css_grid_info {
	/** \name ISP input size
	  * that is visible for user
	  * @{
	  */
	uint32_t isp_in_width;
	uint32_t isp_in_height;
	/** @}*/

	struct ia_css_3a_grid_info  s3a_grid; /**< 3A grid info */
	struct ia_css_dvs_grid_info dvs_grid; /**< DVS grid info */

	enum ia_css_vamem_type vamem_type;
};

/** Morphing table, used for geometric distortion and chromatic abberration
 *  correction (GDCAC, also called GDC).
 *  This table describes the imperfections introduced by the lens, the
 *  advanced ISP can correct for these imperfections using this table.
 */
struct ia_css_morph_table {
	uint32_t enable; /**< To disable GDC, set this field to false. The
		          coordinates fields can be set to NULL in this case. */
	uint32_t height; /**< Table height */
	uint32_t width;  /**< Table width */
	/*uint16_t *coordinates_x[IA_CSS_MORPH_TABLE_NUM_PLANES];*/
	struct data_offset coordinates_x[IA_CSS_MORPH_TABLE_NUM_PLANES];
	/**< X coordinates that describe the sensor imperfection */
	struct data_offset coordinates_y[IA_CSS_MORPH_TABLE_NUM_PLANES];
	/*uint16_t *coordinates_y[IA_CSS_MORPH_TABLE_NUM_PLANES];*/
	/**< Y coordinates that describe the sensor imperfection */
};

struct ia_css_dvs_6axis_config {
	unsigned int exp_id;
	uint32_t width_y;
	uint32_t height_y;
	uint32_t width_uv;
	uint32_t height_uv;
	struct data_offset xcoords_y;
	struct data_offset ycoords_y;
	struct data_offset xcoords_uv;
	struct data_offset ycoords_uv;
};

/**
 * Digital zoom:
 * This feature is currently available only for video, but will become
 * available for preview and capture as well.
 * Set the digital zoom factor, this is a logarithmic scale. The actual zoom
 * factor will be 64/x.
 * Setting dx or dy to 0 disables digital zoom for that direction.
 */
struct ia_css_dz_config {
	uint32_t dx;
	uint32_t dy;
};

/** The still capture mode, this can be RAW (simply copy sensor input to DDR),
 *  Primary ISP, the Advanced ISP (GDC) or the low-light ISP (ANR).
 */
enum ia_css_capture_mode {
	IA_CSS_CAPTURE_MODE_RAW,      /**< no processing, copy data only */
	IA_CSS_CAPTURE_MODE_BAYER,    /**< bayer processing, up to demosaic */
	IA_CSS_CAPTURE_MODE_PRIMARY,  /**< primary ISP */
	IA_CSS_CAPTURE_MODE_ADVANCED, /**< advanced ISP (GDC) */
	IA_CSS_CAPTURE_MODE_LOW_LIGHT /**< low light ISP (ANR) */
};

struct ia_css_capture_config {
	enum ia_css_capture_mode mode; /**< Still capture mode */
	uint32_t enable_xnr;	       /**< Enable/disable XNR */
	uint32_t enable_raw_output;
};

/** ISP filter configuration. This is a collection of configurations
 *  for each of the ISP filters (modules).
 *
 *  NOTE! The contents of all pointers is copied when get or set with the
 *  exception of the shading and morph tables. For these we only copy the
 *  pointer, so the caller must make sure the memory contents of these pointers
 *  remain valid as long as they are used by the CSS. This will be fixed in the
 *  future by copying the contents instead of just the pointer.
 *
 *  Comment:
 *    ["ISP block", 1&2]   : ISP block is used both for ISP1 and ISP2.
 *    ["ISP block", 1only] : ISP block is used only for ISP1.
 *    ["ISP block", 2only] : ISP block is used only for ISP2.
 */
struct ia_css_isp_config {
	struct ia_css_wb_config   *wb_config;	/**< White Balance
							[WB1, 1&2] */
	struct ia_css_cc_config   *cc_config;	/**< Color Correction
							[CSC1, 1only] */
	struct ia_css_tnr_config  *tnr_config;	/**< Temporal Noise Reduction
							[TNR1, 1&2] */
	struct ia_css_ecd_config  *ecd_config;	/**< Eigen Color Demosaicing
							[DE2, 2only] */
	struct ia_css_ynr_config  *ynr_config;	/**< Y(Luma) Noise Reduction
							[YNR2&YEE2, 2only] */
	struct ia_css_fc_config   *fc_config;	/**< Fringe Control
							[FC2, 2only] */
	struct ia_css_cnr_config  *cnr_config;	/**< Chroma Noise Reduction
							[CNR2, 2only] */
	struct ia_css_macc_config *macc_config;	/**< MACC
							[MACC2, 2only] */
	struct ia_css_ctc_config  *ctc_config;	/**< Chroma Tone Control
							[CTC2, 2only] */
	struct ia_css_aa_config   *aa_config;	/**< YUV Anti-Aliasing
							[AA2, 2only]
						        (not used currently) */
	struct ia_css_aa_config   *baa_config;	/**< Bayer Anti-Aliasing
							[BAA2, 1&2] */
	struct ia_css_ce_config   *ce_config;	/**< Chroma Enhancement
							[CE1, 1only] */
	struct ia_css_dvs_6axis_config *dvs_6axis_config;
	struct ia_css_ob_config   *ob_config;  /**< Objective Black
							[OB1, 1&2] */
	struct ia_css_dp_config   *dp_config;  /**< Defect Pixel Correction
							[DPC1/DPC2, 1&2] */
	struct ia_css_nr_config   *nr_config;  /**< Noise Reduction
							[BNR1&YNR1&CNR1, 1&2]*/
	struct ia_css_ee_config   *ee_config;  /**< Edge Enhancement
							[YEE1, 1&2] */
	struct ia_css_de_config   *de_config;  /**< Demosaic
							[DE1, 1only] */
	struct ia_css_gc_config   *gc_config;  /**< Gamma Correction (for YUV)
							[GC1, 1only] */
	struct ia_css_anr_config  *anr_config; /**< Advanced Noise Reduction */
	struct ia_css_3a_config   *s3a_config; /**< 3A Statistics config */
	struct ia_css_xnr_config  *xnr_config; /**< eXtra Noise Reduction */
	struct ia_css_dz_config   *dz_config;  /**< Digital Zoom */
	struct ia_css_cc_config *yuv2rgb_cc_config; /**< Color Correction
							[CCM2, 2only] */
	struct ia_css_cc_config *rgb2yuv_cc_config; /**< Color Correction
							[CSC2, 2only] */
	struct ia_css_macc_table  *macc_table;	/**< MACC
							[MACC1/MACC2, 1&2]*/
	struct ia_css_gamma_table *gamma_table;	/**< Gamma Correction (for YUV)
							[GC1, 1only] */
	struct ia_css_ctc_table   *ctc_table;	/**< Chroma Tone Control
							[CTC1, 1only] */

	/** \deprecated */
	struct ia_css_xnr_table   *xnr_table;	/**< eXtra Noise Reduction
							[XNR1, 1&2] */
	struct ia_css_rgb_gamma_table *r_gamma_table;/**< sRGB Gamma Correction
							[GC2, 2only] */
	struct ia_css_rgb_gamma_table *g_gamma_table;/**< sRGB Gamma Correction
							[GC2, 2only] */
	struct ia_css_rgb_gamma_table *b_gamma_table;/**< sRGB Gamma Correction
							[GC2, 2only] */
	struct ia_css_vector      *motion_vector; /**< For 2-axis DVS */
	struct ia_css_shading_table *shading_table;
	struct ia_css_morph_table   *morph_table;
	struct ia_css_dvs_coefficients *dvs_coefs; /**< DVS 1.0 coefficients */
	struct ia_css_dvs2_coefficients *dvs2_coefs; /**< DVS 2.0 coefficients */
	struct ia_css_capture_config   *capture_config;
	struct ia_css_anr_thres   *anr_thres;

	struct ia_css_2500_lin_kernel_config     *lin_2500_config;       /**< Skylake: Linearization config */
	struct ia_css_2500_obgrid_kernel_config  *obgrid_2500_config;    /**< Skylake: OBGRID config */
	struct ia_css_2500_bnr_kernel_config     *bnr_2500_config;       /**< Skylake: bayer denoise config */
	struct ia_css_2500_shd_kernel_config     *shd_2500_config;       /**< Skylake: shading config */
	struct ia_css_2500_dm_kernel_config      *dm_2500_config;        /**< Skylake: demosaic config */
	struct ia_css_2500_rgbpp_kernel_config   *rgbpp_2500_config;     /**< Skylake: RGBPP config */
	struct ia_css_2500_yuvp1_kernel_config   *yuvp1_2500_config;     /**< Skylake: yuvp1 config */
	struct ia_css_2500_yuvp2_kernel_config   *yuvp2_2500_config;     /**< Skylake: yuvp2 config */
	struct ia_css_2500_tnr_kernel_config     *tnr_2500_config;       /**< Skylake: TNR config */
	struct ia_css_2500_dpc_kernel_config     *dpc_2500_config;       /**< Skylake: DPC config */
	struct ia_css_2500_awb_kernel_config     *awb_2500_config;       /**< Skylake: auto white balance config */
	struct ia_css_2500_awb_fr_kernel_config  *awb_fr_2500_config;    /**< Skylake: auto white balance filter response config */
	struct ia_css_2500_anr_kernel_config     *anr_2500_config;       /**< Skylake: ANR config */
	struct ia_css_2500_af_kernel_config      *af_2500_config;        /**< Skylake: auto focus config */
	struct ia_css_2500_ae_kernel_config      *ae_2500_config;        /**< Skylake: auto exposure config */
	struct ia_css_2500_bds_kernel_config     *bds_2500_config;       /**< Skylake: bayer downscaler config */
	struct ia_css_2500_dvs_kernel_config     *dvs_2500_config;       /**< Skylake: digital video stabilization config */
};

/** DVS 1.0 Coefficients.
 *  This structure describes the coefficients that are needed for the dvs statistics.
 */

struct ia_css_dvs_coefficients {
	struct ia_css_dvs_grid_info grid;/**< grid info contains the dimensions of the dvs grid */
	int16_t *hor_coefs;	/**< the pointer to int16_t[grid.num_hor_coefs * IA_CSS_DVS_NUM_COEF_TYPES]
				     containing the horizontal coefficients */
	int16_t *ver_coefs;	/**< the pointer to int16_t[grid.num_ver_coefs * IA_CSS_DVS_NUM_COEF_TYPES]
				     containing the vertical coefficients */
};

/** DVS 1.0 Statistics.
 *  This structure describes the statistics that are generated using the provided coefficients.
 */

struct ia_css_dvs_statistics {
	struct ia_css_dvs_grid_info grid;/**< grid info contains the dimensions of the dvs grid */
	int32_t *hor_proj;	/**< the pointer to int16_t[grid.height * IA_CSS_DVS_NUM_COEF_TYPES]
				     containing the horizontal projections */
	int32_t *ver_proj;	/**< the pointer to int16_t[grid.width * IA_CSS_DVS_NUM_COEF_TYPES]
				     containing the vertical projections */
};

/** DVS 2.0 Coefficient types. This structure contains 4 pointers to
 *  arrays that contain the coeffients for each type.
 */
struct ia_css_dvs2_coef_types {
	/*int16_t *odd_real;*/ /**< real part of the odd coefficients*/
	/*int16_t *odd_imag;*/ /**< imaginary part of the odd coefficients*/
	/*int16_t *even_real;*/ /**< real part of the even coefficients*/
	/*int16_t *even_imag;*/ /**< imaginary part of the even coefficients*/

	struct data_offset odd_real; /**< real part of the odd coefficients*/
	struct data_offset odd_imag; /**< imaginary part of the odd coefficients*/
	struct data_offset even_real;/**< real part of the even coefficients*/
	struct data_offset even_imag;/**< imaginary part of the even coefficients*/
};

/** DVS 2.0 Coefficients. This structure describes the coefficients that are needed for the dvs statistics.
 *  e.g. hor_coefs.odd_real is the pointer to int16_t[grid.num_hor_coefs] containing the horizontal odd real
 *  coefficients.
 */
struct ia_css_dvs2_coefficients {
	struct ia_css_dvs_grid_info grid;        /**< grid info contains the dimensions of the dvs grid */
	struct ia_css_dvs2_coef_types hor_coefs; /**< struct with pointers that contain the horizontal coefficients */
	struct ia_css_dvs2_coef_types ver_coefs; /**< struct with pointers that contain the vertical coefficients */
};

/** DVS 2.0 Statistic types. This structure contains 4 pointers to
 *  arrays that contain the statistics for each type.
 */
struct ia_css_dvs2_stat_types {
	int32_t *odd_real; /**< real part of the odd statistics*/
	int32_t *odd_imag; /**< imaginary part of the odd statistics*/
	int32_t *even_real;/**< real part of the even statistics*/
	int32_t *even_imag;/**< imaginary part of the even statistics*/
};

/** DVS 2.0 Statistics. This structure describes the statistics that are generated using the provided coefficients.
 *  e.g. hor_prod.odd_real is the pointer to int16_t[grid.aligned_height][grid.aligned_width] containing
 *  the horizontal odd real statistics. Valid statistics data area is int16_t[0..grid.height-1][0..grid.width-1]
 */
struct ia_css_dvs2_statistics {
	struct ia_css_dvs_grid_info grid;       /**< grid info contains the dimensions of the dvs grid */
	struct ia_css_dvs2_stat_types hor_prod; /**< struct with pointers that contain the horizontal statistics */
	struct ia_css_dvs2_stat_types ver_prod; /**< struct with pointers that contain the vertical statistics */
};

struct sh_css_uds_info {
	uint16_t curr_dx;
	uint16_t curr_dy;
	uint16_t xc;
	uint16_t yc;
};

struct sh_css_crop_pos {
	uint16_t x;
	uint16_t y;
};

/* uds (Up and Down scaling) */
struct sh_css_sp_uds_params {
	struct sh_css_crop_pos crop_pos;
	struct sh_css_uds_info uds;
};

#define SH_CSS_MAX_STAGES 6 /* copy, preisp, anr, postisp, capture_pp, vf_pp */

/** The pipe id type, distinguishes the kind of pipes that
 *  can be run in parallel.
 */
enum ia_css_pipe_id {
	IA_CSS_PIPE_ID_PREVIEW,
	IA_CSS_PIPE_ID_COPY,
	IA_CSS_PIPE_ID_VIDEO,
	IA_CSS_PIPE_ID_CAPTURE,
	IA_CSS_PIPE_ID_ACC
};

#define IA_CSS_PIPE_ID_NUM (IA_CSS_PIPE_ID_ACC + 1)
#define IA_CSS_NUM_ISP_MEMORIES 7

enum ia_css_parameter_ids {
	IA_CSS_AA_ID,
	IA_CSS_ANR_ID,
	IA_CSS_BH_ID,
	IA_CSS_CNR_ID,
	IA_CSS_CSC_ID,
	IA_CSS_DP_ID,
	IA_CSS_BNR_ID,
	IA_CSS_DE_ID,
	IA_CSS_ECD_ID,
	IA_CSS_FPN_ID,
	IA_CSS_GC_ID,
	IA_CSS_CE_ID,
	IA_CSS_YUV2RGB_ID,
	IA_CSS_RGB2YUV_ID,
	IA_CSS_R_GAMMA_ID,
	IA_CSS_G_GAMMA_ID,
	IA_CSS_B_GAMMA_ID,
	IA_CSS_RAA_ID,
	IA_CSS_S3A_ID,
	IA_CSS_OB_ID,
	IA_CSS_SC_ID,
	IA_CSS_RAW_ID,
	IA_CSS_TNR_ID,
	IA_CSS_MACC_ID,
	IA_CSS_WB_ID,
	IA_CSS_NR_ID,
	IA_CSS_YEE_ID,
	IA_CSS_YNR_ID,
	IA_CSS_FC_ID,
	IA_CSS_CTC_ID,
	IA_CSS_XNR_TABLE_ID,
	IA_CSS_XNR_ID,
	IA_CSS_NUM_PARAMETER_IDS
};

/* Isp parameters per stream */
struct ia_css_isp_parameters {
	/* UDS */
	struct sh_css_sp_uds_params uds[SH_CSS_MAX_STAGES];

	struct ia_css_fpn_table     fpn_config;
	struct ia_css_vector	    motion_config;
	struct ia_css_macc_table    macc_table;
	struct ia_css_gamma_table   gc_table;
	struct ia_css_ctc_table     ctc_table;
	struct ia_css_xnr_table     xnr_table;

	struct ia_css_dz_config     dz_config;
	struct ia_css_3a_config     s3a_config;
	struct ia_css_wb_config     wb_config;
	struct ia_css_cc_config     csc_config;
	struct ia_css_cc_config     yuv2rgb_config;
	struct ia_css_cc_config     rgb2yuv_config;
	struct ia_css_tnr_config    tnr_config;
	struct ia_css_ob_config     ob_config;
	struct ia_css_dp_config     dp_config;
	struct ia_css_nr_config     nr_config;
	struct ia_css_ee_config     ee_config;
	struct ia_css_de_config     de_config;
	struct ia_css_gc_config     gc_config;
	struct ia_css_anr_config    anr_config;
	struct ia_css_ce_config     ce_config;

	struct ia_css_ecd_config    ecd_config;
	struct ia_css_ynr_config    ynr_config;
	struct ia_css_yee_config    yee_config;
	struct ia_css_fc_config     fc_config;
	struct ia_css_cnr_config    cnr_config;
	struct ia_css_macc_config   macc_config;
	struct ia_css_ctc_config    ctc_config;
	struct ia_css_aa_config     aa_config;
	struct ia_css_aa_config     raw_config;
	struct ia_css_aa_config     raa_config;
	struct ia_css_rgb_gamma_table     r_gamma_table;
	struct ia_css_rgb_gamma_table     g_gamma_table;
	struct ia_css_rgb_gamma_table     b_gamma_table;
	struct ia_css_anr_thres     anr_thres;
	struct ia_css_xnr_config    xnr_config;

	struct ia_css_morph_table morph_table;
	struct ia_css_dvs2_coef_types coef_types;
	struct ia_css_dvs2_coefficients coefficients;
	struct ia_css_shading_table sc_table;
	struct ia_css_dvs_6axis_config dvs_config;

	uint32_t fpn_changed;
	uint32_t vector_config_changed;
	uint32_t macc_table_changed;
	uint32_t gc_table_changed;
	uint32_t ctc_table_changed;
	uint32_t xnr_table_changed;

	uint32_t dz_changed;
	uint32_t s3a_changed;
	uint32_t wb_changed;
	uint32_t csc_changed;
	uint32_t yuv2rgb_changed;
	uint32_t rgb2yuv_changed;
	uint32_t tnr_changed;
	uint32_t ob_changed;
	uint32_t dp_changed;
	uint32_t nr_changed;
	uint32_t ee_changed;
	uint32_t de_changed;
	uint32_t gc_changed;
	uint32_t anr_changed;
	uint32_t ce_changed;

	uint32_t ecd_changed;
	uint32_t ynr_changed;
	uint32_t yee_changed;
	uint32_t fc_changed;
	uint32_t cnr_changed;
	uint32_t macc_changed;
	uint32_t ctc_changed;
	uint32_t aa_changed;
	uint32_t raw_changed;
	uint32_t raa_changed;
	uint32_t r_gamma_table_changed;
	uint32_t g_gamma_table_changed;
	uint32_t b_gamma_table_changed;
	uint32_t anr_thres_changed;
	uint32_t xnr_changed;
};

