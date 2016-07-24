#ifndef _VIED_NCI_ACC_ISYS_DEFS_
#define _VIED_NCI_ACC_ISYS_DEFS_

#include "type_support.h"

struct vied_nci_acb_conf_data {
	uint32_t    fragment_width;
	uint32_t    fragment_hight;
	uint32_t    ack_address;
	uint32_t    ack_data;

};

enum vied_nci_acc_id {
	VIED_NCI_ICA_ID      = 0,
	VIED_NCI_LSC_ID      ,
	VIED_NCI_DPC_ID      ,
	VIED_NCI_IDS_ID      ,
	VIED_NCI_AWB_ID      ,
	VIED_NCI_AF_ID       ,
	VIED_NCI_AE_ID       ,
	VIED_NCI_N_ACC_ID
};


enum vied_nci_ff_id {
	VIED_NCI_ICA_INL_FF      = 0,
	VIED_NCI_ICA_GBL_FF      ,
	VIED_NCI_ICA_PCLN_FF     ,
	VIED_NCI_LSC_LSC_FF      ,
	VIED_NCI_DPC_DPC_FF      ,
	VIED_NCI_IDS_SCALER_FF   ,
	VIED_NCI_AWB_AWRG_FF     ,
	VIED_NCI_AF_AF_FF        ,
	VIED_NCI_AE_WGHT_HIST_FF ,
	VIED_NCI_AE_CCM_FF       ,
	VIED_NCI_N_ACC_FF
};


struct vied_nci_acc_buf {
	uint32_t start_address;
	uint32_t end_address;
	uint32_t offset;
	uint32_t stride;
};


/* Return type for PSA API functions */
enum vied_nci_err {
	VIED_NCI_SUCCESS           =  0,
	VIED_NCI_OPEN_ACC_FAIL     = -1,
	VIED_NCI_ACC_LOAD_FAIL     = -2,
	VIED_NCI_ACC_STORE_FAIL    = -3,
	VIED_NCI_ACC_ROUTING_FAIL  = -4
};

#define VIED_NCI_N_LSC_PARAM_SET 12

/*
 * ISA ICA accelerator NCI data types' definitions
 */

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
