/*
 * vied_nci_acc_psys_defs.h
 *
 *  Created on: Jan 15, 2014
 *     Authors: mmarkov1
 *              vilic
 */

#ifndef _VIED_NCI_ACC_PSYS_DEFS_
#define _VIED_NCI_ACC_PSYS_DEFS_

#include <type_support.h>

struct vied_nci_acb_conf_data {
	uint32_t    fragment_width;
	uint32_t    fragment_hight;
	uint32_t    ack_address;
	uint32_t    ack_data;

};

enum vied_nci_acc_id {
	/* PSA accelerators */
	VIED_NCI_WBA_ID      = 0,
	VIED_NCI_ANR_ID      ,
	VIED_NCI_DEMOSAIC_ID ,
	VIED_NCI_CCM_ID      ,
	VIED_NCI_GTC_ID      ,
	VIED_NCI_YUV1_ID     ,
	VIED_NCI_DVS_ID      ,
	VIED_NCI_LACE_ID     ,
	/* ISA accelerators */
	VIED_NCI_ICA_ID      ,
	VIED_NCI_LSC_ID      ,
	VIED_NCI_DPC_ID      ,
	VIED_NCI_IDS_ID      ,
	VIED_NCI_AWB_ID      ,
	VIED_NCI_AF_ID       ,
	VIED_NCI_AE_ID       ,
	VIED_NCI_N_ACC_ID
};


enum vied_nci_ff_id {
	/* PSA fixed functions */
	VIED_NCI_WBA_WBA_FF           = 0,
	VIED_NCI_WBA_BC_FF            ,
	VIED_NCI_ANR_SEARCH_FF        ,
	VIED_NCI_ANR_TRANSFORM_FF     ,
	VIED_NCI_ANR_STITCH_FF        ,
	VIED_NCI_ANR_TILE_FF          ,
	VIED_NCI_DEMOSAIC_DEMOSAIC_FF ,
	VIED_NCI_CCM_CCM_FF           ,
	VIED_NCI_CCM_BDC_FF           ,
	VIED_NCI_GTC_CSC_CDS_FF       ,
	VIED_NCI_GTC_GTM_FF           ,
	VIED_NCI_YUV1_YEENR_FF        ,
	VIED_NCI_YUV1_YDS_FF          ,
	VIED_NCI_YUV1_TCC_FF          ,
	VIED_NCI_DVS_YBIN_FF          ,
	VIED_NCI_DVS_DVS_FF           ,
	VIED_NCI_LACE_LACE_FF         ,
	/* ISA fixed functions */
	VIED_NCI_ICA_INL_FF           ,
	VIED_NCI_ICA_GBL_FF           ,
	VIED_NCI_ICA_PCLN_FF          ,
	VIED_NCI_LSC_LSC_FF           ,
	VIED_NCI_DPC_DPC_FF           ,
	VIED_NCI_IDS_SCALER_FF        ,
	VIED_NCI_AWB_AWRG_FF          ,
	VIED_NCI_AF_AF_FF             ,
	VIED_NCI_AE_WGHT_HIST_FF      ,
	VIED_NCI_AE_CCM_FF            ,
	VIED_NCI_N_ACC_FF
};

#define VIED_NCI_N_LACE_OUT_SET 2
#define VIED_NCI_N_DVS_OUT_SET  6

#define VIED_NCI_N_LSC_PARAM_SET 12

/*
 * The StreamToVec device writes each of the color component vectors to a circular buffer in the VMEM/BAMEM
 * which is defined by a start address, and end address, offset and stride.
 * For instance for a Bayer type VecToStr there is a circular buffer defined for Gr, R, B, and Gb.
 * The offset is used to increment the address from one vector write to the following one.
 * The stride is used to increment the address when an end of line is reached.
 */


/* Return type for PSA API functions */
enum vied_nci_err {
	VIED_NCI_SUCCESS           =  0,
	VIED_NCI_OPEN_ACC_FAIL     = -1,
	VIED_NCI_ACC_LOAD_FAIL     = -2,
	VIED_NCI_ACC_STORE_FAIL    = -3,
	VIED_NCI_ACC_ROUTING_FAIL  = -4
};


/*
 * PSA WBA accelerator NCI data types' definitions
 */

#define NUM_OF_BC_LUT_ELEMENTS 256
#define NUM_OF_BC_SEGMENT_ELEMENTS 8

/* PSA WBA-BC ff NCI data types' definitions */
union ff_bc_seg_ctrl {
	struct {
		uint32_t base_level :16;
		uint32_t step_size  :4;
		uint32_t start_bin  :9;
		uint32_t spare0     :3;
	} data_bits;
	uint32_t data;
};

union ff_bc_ctrl {
	struct {
		uint32_t lut_size  :9;
		uint32_t spare0    :7;
		uint32_t bc_enable :1;
		uint32_t spare1    :15;
	} data_bits;
	uint32_t data;
};

struct vied_nci_wba_bc_conf_data {
	uint16_t       lut[NUM_OF_BC_LUT_ELEMENTS];
	union ff_bc_seg_ctrl seg_ctrl[NUM_OF_BC_SEGMENT_ELEMENTS];
	union ff_bc_ctrl     bc_ctrl;
};

/* PSA WBA-WB ff NCI data types' definitions */
struct ff_wba_gain_ctrl {
	uint16_t gch0;
	uint16_t gch1;
	uint16_t gch2;
	uint16_t gch3;
	uint16_t gch4;
	uint16_t gch5;
	uint16_t gch6;
	uint16_t gch7;
};

union ff_wba_sensor_cfg0 {
	struct {
		uint32_t sensor_mode:2;
		uint32_t spare0:30;
	} data_bits;
	uint32_t data;
};

union ff_wba_sensor_cfg1 {
	struct {
		uint32_t pat00:4;
		uint32_t pat01:4;
		uint32_t pat02:4;
		uint32_t pat03:4;
		uint32_t pat10:4;
		uint32_t pat11:4;
		uint32_t pat12:4;
		uint32_t pat13:4;
	} data_bits;
	uint32_t data;
};

union ff_wba_sensor_cfg2 {
	struct {
		uint32_t pat20:4;
		uint32_t pat21:4;
		uint32_t pat22:4;
		uint32_t pat23:4;
		uint32_t pat30:4;
		uint32_t pat31:4;
		uint32_t pat32:4;
		uint32_t pat33:4;
	} data_bits;
	uint32_t data;
};

union ff_wba_gen_ctrl {
	struct {
		uint32_t wba_en:1;
		uint32_t spare0:31;
	} data_bits;
	uint32_t data;
};

struct vied_nci_wba_wb_conf_data {
	struct ff_wba_gain_ctrl  gain_ctrl;   /* 0x00000 */
	union ff_wba_sensor_cfg0 sensor_cfg0; /* 0x00010 */
	union ff_wba_sensor_cfg1 sensor_cfg1; /* 0x00014 */
	union ff_wba_sensor_cfg2 sensor_cfg2; /* 0x00018 */
	union ff_wba_gen_ctrl    gen_ctrl;    /* 0x0001C */
};

struct vied_nci_acc_psa_wba_conf_data {
	struct vied_nci_acb_conf_data acb_conf_data;
	struct vied_nci_wba_wb_conf_data wb_conf_data;
	struct vied_nci_wba_bc_conf_data bc_conf_data;
};


/*
 * PSA ANR accelerator NCI data types' definitions
 */

struct ff_anr_frame_size {
	uint16_t frame_width;
	uint16_t frame_height;
};

/* PSA ANR-search ff NCI data types' definitions */
union ff_anr_search_cfg {
	struct {
		uint32_t search_en :1;
		uint32_t spare0    :31;
	} data_bits;
	uint32_t data;
};

struct vied_nci_anr_search_conf_data {
	union  ff_anr_search_cfg search_cfg;         /* 0x00000 */
	struct ff_anr_frame_size search_frame_size;  /* 0x00004 */
};

/* PSA ANR-transform ff NCI data types' definitions */
union ff_anr_transform_cfg {
	struct {
		uint32_t transform_en :1;
		uint32_t threshold_en :1;
		uint32_t spare0       :30;
	} data_bits;
	uint32_t data;
};

union ff_anr_plane_alpha0 {
	struct {
		uint32_t alpha_gr :9;
		uint32_t spare0   :7;
		uint32_t alphar   :9;
		uint32_t spare1   :7;
	} data_bits;
	uint32_t data;
};

union ff_anr_plane_alpha1 {
	struct {
		uint32_t alpha_b  :9;
		uint32_t spare0   :7;
		uint32_t alpha_gb :9;
		uint32_t spare1   :7;
	} data_bits;
	uint32_t data;
};

union ff_anr_plane_beta0 {
	struct {
		uint32_t beta_gr :11;
		uint32_t spare0  :5;
		uint32_t beta_r  :11;
		uint32_t spare1  :5;
	} data_bits;
	uint32_t data;
};

union ff_anr_plane_beta1 {
	struct {
		uint32_t beta_b  :11;
		uint32_t spare0  :5;
		uint32_t beta_gb :11;
		uint32_t spare1  :5;
	} data_bits;
	uint32_t data;
};

struct ff_anr_transform_plane_alpha {
	union ff_anr_plane_alpha0 plane_alpha_0;
	union ff_anr_plane_alpha1 plane_alpha_1;
	union ff_anr_plane_alpha0 plane_alpha_DC_0;
	union ff_anr_plane_alpha1 plane_alpha_DC_1;
};

struct ff_anr_transform_plane_beta {
	union ff_anr_plane_beta0 plane_beta_0;
	union ff_anr_plane_beta1 plane_beta_1;
};

#define MAX_ANR_WPC_MATRIX_SIZE 16

struct ff_anr_transform_plane_color_reg_w {
	uint16_t gr_reg[MAX_ANR_WPC_MATRIX_SIZE];
	uint16_t r_reg[MAX_ANR_WPC_MATRIX_SIZE];
	uint16_t b_reg[MAX_ANR_WPC_MATRIX_SIZE];
	uint16_t gb_reg[MAX_ANR_WPC_MATRIX_SIZE];
};


union ff_anr_transform_r_calc_1 {
	struct {
		uint32_t xreset :13;
		uint32_t spare0 :3;
		uint32_t yreset :13;
		uint32_t spare1 :3;
	} data_bits;
	uint32_t data;
};

union ff_anr_transform_r_calc_2 {
	struct {
		uint32_t x_sqr_reset  :24;
		uint32_t r_normfactor :5;
		uint32_t spare0       :3;
	} data_bits;
	uint32_t data;
};

union ff_anr_transform_r_calc_3 {
	struct {
		uint32_t y_sqr_reset :24;
		uint32_t a           :8;
	} data_bits;
	uint32_t data;
};

#define MAX_ANR_PLANE_SIZE      3
#define MAX_ANR_SQRT_LUT_SIZE   25

struct vied_nci_anr_transform_conf_data {
	/* 0x00000  */
	union ff_anr_transform_cfg transform_cfg;
	/* 0x00004 */
	char spare1[0x2c];
	/* 0x00030 */
	struct ff_anr_transform_plane_alpha transform_plane_alpha[MAX_ANR_PLANE_SIZE];
	/* 0x00060 */
	struct ff_anr_transform_plane_beta transform_plane_beta[MAX_ANR_PLANE_SIZE];
	/* 0x00078 */
	struct ff_anr_transform_plane_color_reg_w transform_plane_reg_w[MAX_ANR_PLANE_SIZE];
	/* 0x001f8 */
	uint16_t transform_sqrt_lut[MAX_ANR_SQRT_LUT_SIZE];
	uint16_t spare2;
	/* 0x0022c */
	union ff_anr_transform_r_calc_1 transform_r_calc_1;
	/* 0x00230 */
	union ff_anr_transform_r_calc_2 transform_r_calc_2;
	/* 0x00234 */
	union ff_anr_transform_r_calc_3 transform_r_calc_3;
};

/* PSA ANR-stitch ff NCI data types' definitions */
union ff_anr_stitch_cfg {
	struct {
		uint32_t stitch_en :1;
		uint32_t spare0    :31;
	} data_bits;
	uint32_t data;
};

union ff_anr_stitch_reg {
	struct {
		uint32_t entry_0 :6;
		uint32_t entry_1 :6;
		uint32_t entry_2 :6;
		uint32_t spare0  :14;
	} data_bits;
	uint32_t data;
};

#define MAX_ANR_STITCH_PYRAMYD_SIZE 22

struct vied_nci_anr_stitch_conf_data {
	union ff_anr_stitch_cfg stitch_cfg;
	struct ff_anr_frame_size stitch_frame_size;
	char spare1[0x28];
	union ff_anr_stitch_reg stitch_reg[MAX_ANR_STITCH_PYRAMYD_SIZE];
};

/* PSA ANR-stitch ff NCI data types' definitions */
union ff_anr_tile2strm_cfg {
	struct {
		uint32_t tile2strm_en :1;
		uint32_t spare0       :31;
	} data_bits;
	uint32_t data;
};

struct vied_nci_anr_tile2strm_conf_data {
	union ff_anr_tile2strm_cfg tile2strm_cfg;
	struct ff_anr_frame_size tile2strm_frame_size;
};

struct vied_nci_acc_psa_anr_conf_data {
	struct vied_nci_acb_conf_data acb_conf_data;
	struct vied_nci_anr_search_conf_data anr_search_conf_data;
	struct vied_nci_anr_transform_conf_data anr_transform_conf_data;
	struct vied_nci_anr_stitch_conf_data anr_stitch_conf_data;
	struct vied_nci_anr_tile2strm_conf_data anr_tile2stream_conf_data;
};


/*
 * PSA DMS accelerator NCI data types' definitions
 */

union ff_dm_ctrl0 {
	struct {
		uint32_t ff_en      :1;
		uint32_t char_en    :1;
		uint32_t fcc_en     :1;
		uint32_t spare0     :13;
		uint32_t frame_width:16;
	} data_bits;
	uint32_t data;
};

union ff_dm_ctrl1 {
	struct {
		uint32_t gamma_sc  :5;
		uint32_t spare1    :3;
		uint32_t lc_ctrl   :5;
		uint32_t spare2    :3;
		uint32_t cr_param1 :5;
		uint32_t spare3    :3;
		uint32_t cr_param2 :5;
		uint32_t spare4    :3;
	} data_bits;
	uint32_t data;
};

union ff_dm_ctrl2 {
	struct {
		uint32_t coring_param :5;
		uint32_t spare5       :27;
	} data_bits;
	uint32_t data;
};

struct vied_nci_dms_conf_data {
	union ff_dm_ctrl0 dm_ctrl0;
	union ff_dm_ctrl1 dm_ctrl1;
	union ff_dm_ctrl2 dm_ctrl2;
};

struct vied_nci_acc_psa_dms_conf_data {
	struct vied_nci_acb_conf_data acb_conf_data;
	struct vied_nci_dms_conf_data dms_conf_data;
};


/*
 * PSA CCM accelerator NCI data types' definitions
 */

#define MAX_BDC_LUT_TABLE   256
#define MAX_BDC_SEGMENTS    8

/* PSA CCM-BDC ff NCI data types' definitions */
union ff_ccm_bdc_seg_ctrl {
	struct {
		uint32_t base_level :14;
		uint32_t spare0     :2;
		uint32_t step_size  :4;
		uint32_t start_bin  :9;
		uint32_t spare1     :3;
	} data_bits;
	uint32_t data;
};

union ff_ccm_bdc_cfg {
	struct {
		uint32_t lut_size  :8;
		uint32_t spare0    :8;
		uint32_t bdc_en    :1;
		uint32_t spare1    :15;
	} data_bits;
	uint32_t data;
};

struct vied_nci_ccm_bdc_conf_data {
	/* 0x00000 + (0x2*i) */
	uint16_t lut[MAX_BDC_LUT_TABLE];
	/* 0x00200 + (0x4*segId) */
	union ff_ccm_bdc_seg_ctrl seg_ctrl[MAX_BDC_SEGMENTS];
	/* 0x00220 */
	union ff_ccm_bdc_cfg bdc_cfg;
};

/* PSA CCM-CCM ff NCI data types' definitions */
union ff_ccm_ccm_gen_ctrl {
	struct {
		uint32_t ccm_en :1;
		uint32_t spare0 :31;
	} data_bits;
	uint32_t data;
};

struct vied_nci_ccm_ccm_conf_data {
	uint16_t coeff_m11;           /* 0x00000  */
	uint16_t coeff_m12;
	uint16_t coeff_m13;           /* 0x00004  */
	uint16_t coeff_or;
	uint16_t coeff_m21;           /* 0x00008  */
	uint16_t coeff_m22;
	uint16_t coeff_m23;           /* 0x0000C  */
	uint16_t coeff_og;
	uint16_t coeff_m31;           /* 0x00010  */
	uint16_t coeff_m32;
	uint16_t coeff_m33;           /* 0x00014  */
	uint16_t coeff_ob;
	union ff_ccm_ccm_gen_ctrl ccm_cfg;  /* 0x00018  */
};

struct vied_nci_acc_psa_ccm_conf_data {
	struct vied_nci_acb_conf_data  acb_conf_data;
	struct vied_nci_ccm_bdc_conf_data bdc_conf_data;
	struct vied_nci_ccm_ccm_conf_data ccm_conf_data;
};


/*
 * PSA GTC accelerator NCI data types' definitions
 */

#define MAX_GAMMA_LUT_TABLE     385
#define MAX_TM_LUT_TABLE        513
#define MAX_GAMMA_SEG_TABLE     5
#define MAX_TM_SEG_TABLE        1

/* PSA GTC-GTM ff NCI data types' definitions */
union ff_gtm_seg_cfg {
	struct {
		uint32_t base_level :15;
		uint32_t spare0     :1;
		uint32_t step_size  :4;
		uint32_t start_bin  :10;
		uint32_t spare1     :2;
	} data_bits;
	uint32_t data;
};

union ff_gtm_cfg0 {
	struct {
		uint32_t ff_en           :1;
		uint32_t gamma_en        :1;
		uint32_t tm_en           :1;
		uint32_t gamma_before_tm :1;
		uint32_t sr_ctrl_a1      :2;
		uint32_t spare0          :2;
		uint32_t sr_ctrl_a2      :2;
		uint32_t spare1          :2;
		uint32_t sr_ctrl_a3      :2;
		uint32_t spare2          :18;
	} data_bits;
	uint32_t data;
};

union ff_gtm_cfg1 {
	struct {
		uint32_t gamma_lut_size :10;
		uint32_t spare0         :6;
		uint32_t tm_lut_size    :10;
		uint32_t spare1         :6;
	} data_bits;
	uint32_t data[2];
};

struct vied_nci_gtc_gtm_conf_data {
	/* 0x00000 + (0x4*i) */
	uint16_t gamma_lut[MAX_GAMMA_LUT_TABLE];
	/* 0x003020x00320 */
	uint16_t spare0[0xF];
	/* 0x00000 + (0x4*i) */
	uint16_t tm_lut[MAX_TM_LUT_TABLE];
	/* 0x004020x00420 */
	uint16_t spare1[0xF];
	union ff_gtm_seg_cfg gamma_seg_cfg[MAX_GAMMA_SEG_TABLE];
	union ff_gtm_seg_cfg tm_seg_cfg[MAX_TM_SEG_TABLE];
	union ff_gtm_cfg0 gtm_cfg0;
	union ff_gtm_cfg1 gtm_cfg1;
};

/* PSA GTC-CSC ff NCI data types' definitions */
union ff_csc_mat_coeff0 {
	struct {
		uint32_t coeff_cx1 :15;
		uint32_t spare0    :1;
		uint32_t coeff_cx2 :15;
		uint32_t spare1    :1;
	} data_bits;
	uint32_t data;
};

union ff_csc_mat_coeff1 {
	struct {
		uint32_t coeff_cx3  :15;
		uint32_t spare0 :1;
		uint32_t coeff_bx   :14;
		uint32_t spare1 :2;
	} data_bits;
	uint32_t data;
};


union ff_csc_cds_uvds_cfg {
	struct {
		uint32_t ds_c00 :2;
		uint32_t ds_c01 :2;
		uint32_t ds_c02 :2;
		uint32_t ds_c03 :2;
		uint32_t ds_c10 :2;
		uint32_t ds_c11 :2;
		uint32_t ds_c12 :2;
		uint32_t ds_c13 :2;
		uint32_t ds_nf  :3;
		uint32_t spare0 :5;
		uint32_t csc_en :1;
		uint32_t uv_bin_output :1;
		uint32_t spare1 :6;
	} data_bits;
	uint32_t data;
};

struct vied_nci_gtc_csc_conf_data {
	union ff_csc_mat_coeff0 coeff_0;   /* 0x00000  */
	union ff_csc_mat_coeff1 coeff_1;   /* 0x00004  */
	union ff_csc_mat_coeff0 coeff_2;   /* 0x00008  */
	union ff_csc_mat_coeff1 coeff_3;   /* 0x0000C  */
	union ff_csc_mat_coeff0 coeff_4;   /* 0x00010  */
	union ff_csc_mat_coeff1 coeff_5;   /* 0x00014  */
	union ff_csc_cds_uvds_cfg csc_cfg; /* 0x00018  */
};

struct vied_nci_acc_psa_gtc_conf_data {
	struct vied_nci_acb_conf_data  acb_conf_data;
	struct vied_nci_gtc_gtm_conf_data gtm_conf_data;
	struct vied_nci_gtc_csc_conf_data csc_conf_data;
};


/*
 * PSA YUV1 accelerator NCI data types' definitions
 */

/* PSA YUV1-Y_EE ff NCI data types' definitions */
union ff_y_ee_nr_lpf_coeff {
	struct {
		uint32_t a_diag     :5;
		uint32_t spare0     :3;
		uint32_t a_periph   :5;
		uint32_t spare1     :3;
		uint32_t a_cent     :5;
		uint32_t spare2     :9;
		uint32_t y_ee_nr_en :1;
	} data_bits;
	uint32_t data;
};

union ff_y_ee_nr_edge_sense {
	struct {
		uint32_t sence_0     :13;
		uint32_t spare0      :3;
		uint32_t delta_sense :13;
		uint32_t spare1      :3;
	} data_bits;
	uint32_t data;
};

union ff_y_ee_nr_corner_sense {
	struct {
		uint32_t sence_0     :13;
		uint32_t spare0      :3;
		uint32_t delta_sense :13;
		uint32_t spare1      :3;
	} data_bits;
	uint32_t data;
};


union ff_y_ee_nr_frng_ctrl0 {
	struct {
		uint32_t gain_pos_0     :5;
		uint32_t spare0         :3;
		uint32_t delta_gain_pos :5;
		uint32_t spare1         :3;
		uint32_t gain_neg_0     :5;
		uint32_t spare2         :3;
		uint32_t delta_gain_neg :5;
		uint32_t spare3         :3;
	} data_bits;
	uint32_t data;
};

union ff_y_ee_nr_frng_ctrl1 {
	struct {
		uint32_t clip_pos_0     :5;
		uint32_t spare0         :3;
		uint32_t delta_clip_pos :5;
		uint32_t spare1         :3;
		uint32_t clip_neg_0     :5;
		uint32_t spare2         :3;
		uint32_t delta_clip_neg :5;
		uint32_t spare3         :3;
	} data_bits;
	uint32_t data;
};

union ff_y_ee_nr_frng_ctrl2 {
	struct {
		uint32_t gain_exp  :4;
		uint32_t spare0    :4;
		uint32_t mix_range :5;
		uint32_t spare1    :19;
	} data_bits;
	uint32_t data;
};

union ff_y_ee_nr_frng_ctrl3 {
	struct {
		uint32_t min_edge      :13;
		uint32_t spare0        :3;
		uint32_t lin_seg_param :4;
		uint32_t spare1        :4;
		uint32_t t1            :1;
		uint32_t t2            :1;
		uint32_t spare2        :6;
	} data_bits;
	uint32_t data;
};

union ff_y_ee_nr_diag_discg_ctrl {
	struct {
		uint32_t diag_disc_g :4;
		uint32_t spare0      :4;
		uint32_t hvw_hor     :4;
		uint32_t dw_hor      :4;
		uint32_t hvw_diag    :4;
		uint32_t dw_diag     :4;
		uint32_t spare1      :8;
	} data_bits;
	uint32_t data;
};

union ff_y_ee_nr_edge_coring_ctrl0 {
	struct {
		uint32_t fc_coring_pos0       :13;
		uint32_t spare0               :3;
		uint32_t fc_coring_pos_delta  :13;
		uint32_t spare1               :3;
	} data_bits;
	uint32_t data;
};

union ff_y_ee_nr_edge_coring_ctrl1 {
	struct {
		uint32_t fc_coring_nega0      :13;
		uint32_t spare0               :3;
		uint32_t fc_coring_nega_delta :13;
		uint32_t spare1               :3;
	} data_bits;
	uint32_t data;
};

struct vied_nci_yuv1_y_ee_conf_data {
	union ff_y_ee_nr_lpf_coeff lpf_coeff;
	union ff_y_ee_nr_edge_sense edge_sense;
	union ff_y_ee_nr_corner_sense corner_sense;
	union ff_y_ee_nr_frng_ctrl0 frng_ctrl0;
	union ff_y_ee_nr_frng_ctrl1 frng_ctrl1;
	union ff_y_ee_nr_frng_ctrl2 frng_ctrl2;
	union ff_y_ee_nr_frng_ctrl3 frng_ctrl3;
	union ff_y_ee_nr_diag_discg_ctrl diag_discg_ctrl;
	union ff_y_ee_nr_edge_coring_ctrl0 edge_coring_ctrl0;
	union ff_y_ee_nr_edge_coring_ctrl1 edge_coring_ctrl1;
};

/* PSA YUV1-YDS ff NCI data types' definitions */
union vied_nci_yuv1_yds_conf_data {
	struct {
		uint32_t ds_c00 :2;
		uint32_t ds_c01 :2;
		uint32_t ds_c02 :2;
		uint32_t ds_c03 :2;
		uint32_t ds_c10 :2;
		uint32_t ds_c11 :2;
		uint32_t ds_c12 :2;
		uint32_t ds_c13 :2;
		uint32_t ds_nf  :5;
		uint32_t spare0 :4;
		uint32_t y_ds_output :1;
		uint32_t spare1 :6;
	} data_bits;
	uint32_t data;
};

/* PSA YUV1-TCC ff NCI data types' definitions */
#define TCC_NUM_OF_MACC_TABLE_ELEMENTS      16
#define TCC_NUM_OF_INV_Y_LUT_ELEMENTS       14
#define TCC_NUM_OF_GAIN_PCWL_LUT_ELEMENTS   258
#define TCC_NUM_OF_R_SQR_LUT_ELEMENTS       24

union ff_tcc_cfg {
	struct {
		uint32_t tcc_en                   :1;
		uint32_t blend_shift              :3;
		uint32_t gain_according_to_y_only :1;
		uint32_t spare0                   :11;
		uint32_t gamma                    :5;
		uint32_t spare1                   :3;
		uint32_t delta                    :5;
		uint32_t spare2                   :3;
	} data_bits;
	uint32_t data;
};


union ff_tcc_macc_a_b_c_d_sector {
	struct {
		uint32_t a      :12;
		uint32_t spare0 :4;
		uint32_t b      :12;
		uint32_t spare1 :4;
		uint32_t c      :12;
		uint32_t spare2 :4;
		uint32_t d      :12;
		uint32_t spare3 :4;
	} data_bits;
	uint32_t data[2];
};

union ff_tcc_lut {
	struct {
		uint32_t lut_ent_0 :10;
		uint32_t spare0    :6;
		uint32_t lut_ent_1 :10;
		uint32_t spare1    :6;
	} data_bits;
	uint32_t data;
};

struct vied_nci_yuv1_tcc_conf_data {
	/* 0x0000   */
	union ff_tcc_cfg tcc_cfg;
	/* 0x0004   */
	union ff_tcc_macc_a_b_c_d_sector macc_a_b_c_d_sector[TCC_NUM_OF_MACC_TABLE_ELEMENTS];
	/* 0x00084  */
	union ff_tcc_lut inv_y_lut[TCC_NUM_OF_INV_Y_LUT_ELEMENTS/2];
	/* 0x000A0  */
	union ff_tcc_lut gain_pcwl_lut[TCC_NUM_OF_GAIN_PCWL_LUT_ELEMENTS/2];
	/* 0x002A4  */
	union ff_tcc_lut r_sqr_lut[TCC_NUM_OF_R_SQR_LUT_ELEMENTS/2];
};

struct vied_nci_acc_psa_yuv1_conf_data {
	struct vied_nci_acb_conf_data       acb_conf_data;
	struct vied_nci_yuv1_y_ee_conf_data y_ee_conf_data;
	union vied_nci_yuv1_yds_conf_data   yds_conf_data;
	struct vied_nci_yuv1_tcc_conf_data  tcc_conf_data;
};


/*
 * PSA DVS accelerator NCI data types' definitions
 */

/* PSA DVS-Y_BIN ff NCI data types' definitions */
union vied_nci_dvs_y_bin_conf_data {
	struct {
		uint32_t bining_mode :1;
		uint32_t spare0      :31;
	} data_bits;
	uint32_t data;
};

/* PSA DVS-DVS ff NCI data types' definitions */
union ff_dvs_cfg {
	struct {
		uint32_t kappa         :4;
		uint32_t spare0        :4;
		uint32_t match_shift   :4;
		uint32_t ff_y_bin_mode :1;
		uint32_t spare1        :19;
	} data_bits;
	uint32_t data;
};

union ff_dvs_lx_grd_cfg {
	struct {
		uint32_t grid_width   :5;
		uint32_t spare0       :3;
		uint32_t grid_height  :5;
		uint32_t spare1       :3;
		uint32_t block_width  :8;
		uint32_t block_height :8;
	} data_bits;
	uint32_t data;
};

union ff_dvs_lx_grd_start {
	struct {
		uint32_t x_start :12;
		uint32_t spare0  :4;
		uint32_t y_start :12;
		uint32_t spare1  :2;
		uint32_t lx_en   :1;
		uint32_t spare2  :1;
	} data_bits;
	uint32_t data;
};

union ff_dvs_lx_grd_end {
	struct {
		uint32_t x_end  :12;
		uint32_t spare0 :4;
		uint32_t y_end  :12;
		uint32_t spare1 :4;
	} data_bits;
	uint32_t data;
};

union ff_dvs_lx_roi_cfg {
	struct {
		uint32_t x_start :8;
		uint32_t y_start :8;
		uint32_t x_end   :8;
		uint32_t y_end   :8;
	} data_bits;
	uint32_t data;
};


struct vied_nci_dvs_dvs_conf_data {
	union ff_dvs_cfg dvs_cfg;
	union ff_dvs_lx_grd_cfg   l0_grd_cfg;
	union ff_dvs_lx_grd_start l0_grd_start;
	union ff_dvs_lx_grd_end   l0_grd_end;
	union ff_dvs_lx_grd_cfg   l1_grd_cfg;
	union ff_dvs_lx_grd_start l1_grd_start;
	union ff_dvs_lx_grd_end   l1_grd_end;
	union ff_dvs_lx_grd_cfg   l2_grd_cfg;
	union ff_dvs_lx_grd_start l2_grd_start;
	union ff_dvs_lx_grd_end   l2_grd_end;
	union ff_dvs_lx_roi_cfg   l0_roi_cfg;
	union ff_dvs_lx_roi_cfg   l1_roi_cfg;
	union ff_dvs_lx_roi_cfg   l2_roi_cfg;
};

struct vied_nci_acc_psa_dvs_conf_data {
	struct vied_nci_acb_conf_data    acb_conf_data;
	union vied_nci_dvs_y_bin_conf_data y_bin_conf_data;
	struct vied_nci_dvs_dvs_conf_data  dvs_conf_data;
};

/*
 * PSA LACE accelerator NCI data types' definitions
 */

union ff_lace_glb_cfg {
	struct {
		uint32_t lh_mode : 3;
		uint32_t spare0 : 3;
		uint32_t y_ds_mode : 2;
		uint32_t uv_ds_mode : 1;
		uint32_t uv_input : 1;
		uint32_t spare1 : 10;
		uint32_t rst_loc_hist_array : 1;
		uint32_t spare2 : 11;
	} data_bits;
	uint32_t data;
};

union ff_lace_stat_y_grd_hor_cfg {
	struct {
		uint32_t grid_width : 6;
		uint32_t spare0 : 10;
		uint32_t block_width : 4;
		uint32_t spare1 : 12;
	} data_bits;
	uint32_t data;
};

union ff_lace_stat_y_grd_hor_roi {
	struct {
		uint32_t x_start : 12;
		uint32_t spare0 : 4;
		uint32_t x_end : 12;
		uint32_t spare1 : 4;
	} data_bits;
	uint32_t data;
};

union ff_lace_stat_uv_grd_hor_cfg {
	struct {
		uint32_t grid_width : 6;
		uint32_t spare0 : 10;
		uint32_t block_width : 4;
		uint32_t spare1 : 12;
	} data_bits;
	uint32_t data;
};

union ff_lace_stat_uv_grd_hor_roi {
	struct {
		uint32_t x_start : 12;
		uint32_t spare0 : 4;
		uint32_t x_end : 12;
		uint32_t spare1 : 4;
	} data_bits;
	uint32_t data;
};

union ff_lace_stat_uv_grd_vrt_cfg {
	struct {
		uint32_t spare0 : 8;
		uint32_t grid_height : 6;
		uint32_t spare1 : 6;
		uint32_t block_height : 4;
		uint32_t grid_height_per_slice : 7;
		uint32_t spare2 : 1;
	} data_bits;
	uint32_t data;
};

union ff_lace_stat_uv_grd_vrt_roi {
	struct {
		uint32_t y_start : 12;
		uint32_t spare0 : 4;
		uint32_t y_end : 12;
		uint32_t spare1 : 4;
	} data_bits;
	uint32_t data;
};

struct vied_nci_lace_stat_conf_data {
	union ff_lace_glb_cfg glb_cfg;
	union ff_lace_stat_y_grd_hor_cfg  stat_y_grd_hor_cfg;
	union ff_lace_stat_y_grd_hor_roi  stat_y_grd_hor_roi;
	union ff_lace_stat_uv_grd_hor_cfg stat_uv_grd_hor_cfg;
	union ff_lace_stat_uv_grd_hor_roi stat_uv_grd_hor_roi;
	union ff_lace_stat_uv_grd_vrt_cfg stat_uv_grd_vrt_cfg;
	union ff_lace_stat_uv_grd_vrt_roi stat_uv_grd_vrt_roi;
};

struct vied_nci_acc_psa_lace_conf_data {
	struct vied_nci_acb_conf_data    acb_conf_data;
	struct vied_nci_lace_stat_conf_data lace_conf_data;
};


/*-------------------------------------------------*/
/* ISA ICA accelerator NCI data types' definitions */
/*-------------------------------------------------*/

/* ISA ICA-INL ff NCI data types' definitions */

#define NUM_OF_INL_LUT_ELEMENTS 256

union ff_inl_lut_256 {
	struct {
		uint32_t entry_256_range :16;
		uint32_t spare0 :16;
	} data_bits;
	uint32_t data;
};


union ff_inl_ctrl {
	struct {
		uint32_t lin_en :1;
		uint32_t val_bypass :1;
		uint32_t spare0 :2;
		uint32_t shift_depth :4;
		uint32_t spare1 :24;
	} data_bits;
	uint32_t data;
};

struct vied_nci_ica_inl_conf_data {
	uint16_t             lut[NUM_OF_INL_LUT_ELEMENTS];
	union ff_inl_lut_256 inl_lut_256;
	union ff_inl_ctrl    inl_ctrl;
};

/* ISA ICA-PCLN ff NCI data types' definitions */

#define NUM_OF_PCLN_LUT_ELEMENTS 64
#define NUM_OF_PCLN_LUTS 16
#define NUM_OF_PCLN_LUT_IDS 8

union ff_pcln_ctrl {
	struct {
		uint32_t pcln_enable :1;
		uint32_t spare0 :3;
		uint32_t sensor_mode :2;
		uint32_t spare1 :26;
	} data_bits;
	uint32_t data;
};

struct vied_nci_ica_pcln_conf_data {
	uint16_t           lut[NUM_OF_PCLN_LUTS][NUM_OF_PCLN_LUT_ELEMENTS];
	uint32_t           pcln_lut_64[NUM_OF_PCLN_LUT_IDS];
	union ff_pcln_ctrl pcln_ctrl;
};

/* ISA ICA-GBL ff NCI data types' definitions */

#define NUM_OF_GBL_LUT_ELEMENTS 64
#define NUM_OF_GBL_LUTS 16
#define NUM_OF_GBL_BL_BIAS 8

union ff_gbl_grd_cfg0 {
	struct {
		uint32_t grid_width :8;
		uint32_t grid_height :8;
		uint32_t block_width :4;
		uint32_t block_height :4;
		uint32_t grid_height_per_slice :8;
	} data_bits;
	uint32_t data;
};

union ff_gbl_grd_cfg1 {
	struct {
		uint32_t x_start_range :15;
		uint32_t spare0 :1;
		uint32_t y_start_range :15;
		uint32_t spare1 :1;
	} data_bits;
	uint32_t data;
};

union ff_gbl_grd_cfg2 {
	struct {
		uint32_t init_set_vrt_offset :8;
		uint32_t grid_bl_enable :1;
		uint32_t global_bl_enable :1;
		uint32_t ff_enable :1;
		uint32_t spare0 :5;
		uint32_t sensor_mode :2;
		uint32_t spare2 :14;
	} data_bits;
	uint32_t data;
};

union ff_gbl_bl_bias {
	struct {
		uint32_t fixobc0 :16;
		uint32_t fixobc1 :16;
	} data_bits;
	uint32_t data;
};

struct vied_nci_ica_gbl_conf_data {
	uint16_t              lut[NUM_OF_GBL_LUTS][NUM_OF_GBL_LUT_ELEMENTS];
	union ff_gbl_grd_cfg0 grd_cfg0;
	union ff_gbl_grd_cfg1 grd_cfg1;
	union ff_gbl_grd_cfg2 grd_cfg2;
	union ff_gbl_bl_bias  bl_bias[NUM_OF_GBL_BL_BIAS];
};

/* ISA LSC-LSC ff NCI data types' definitions */

#define NUM_OF_LSC_LUT_ELEMENTS 128
#define NUM_OF_LSC_LUTS 12

union ff_lsc_grd_cfg0 {
	struct {
		uint32_t grid_width :8;
		uint32_t grid_height :8;
		uint32_t block_width :4;
		uint32_t block_height :4;
		uint32_t grid_height_per_slice :8;
	} data_bits;
	uint32_t data;
};

union ff_lsc_grd_cfg1 {
	struct {
		uint32_t x_start_range :15;
		uint32_t spare0 :1;
		uint32_t y_start_range :15;
		uint32_t spare1 :1;
	} data_bits;
	uint32_t data;
};

union ff_lsc_grd_cfg2 {
	struct {
		uint32_t init_set_vrt_offset :8;
		uint32_t lsc_enable :1;
		uint32_t spare0 :3;
		uint32_t lsc_exp :3;
		uint32_t spare1 :1;
		uint32_t sensor_mode :2;
		uint32_t spare2 :14;
	} data_bits;
	uint32_t data;
};

struct vied_nci_lsc_lsc_conf_data {
	uint16_t              lut[NUM_OF_LSC_LUTS][NUM_OF_LSC_LUT_ELEMENTS];
	union ff_lsc_grd_cfg0 lsc_grd_cfg0;
	union ff_lsc_grd_cfg1 lsc_grd_cfg1;
	union ff_lsc_grd_cfg2 lsc_grd_cfg2;
};

#endif
