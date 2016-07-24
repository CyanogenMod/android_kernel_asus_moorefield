#ifndef _MIPI_BACKEND_HRT_H
#define _MIPI_BACKEND_HRT_H

#include <hrt/bits.h>
#include <hrt/api.h>
#include "mipi_backend_defs.h"

#define _hrt_mipi_backend_slave_port(mipi_backend_id) HRTCAT(mipi_backend_id,_ctrl_in)

#define _hrt_mipi_backend_set_register(mipi_backend_id, reg_id, val) \
  _hrt_slave_port_store_32_volatile(_hrt_mipi_backend_slave_port(mipi_backend_id), (reg_id<<2), (val))

#define _hrt_mipi_backend_get_register(mipi_backend_id, reg_id) \
  _hrt_slave_port_load_32_volatile(_hrt_mipi_backend_slave_port(mipi_backend_id), (reg_id<<2))

/* functions to read/write to config registers */

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//#define hrt_mipi_backend_enable(mipi_backend_id)
//  _hrt_mipi_backend_set_register(mipi_backend_id, _HRT_MIPI_BACKEND_ENABLE_REG_IDX, 1)
//
//#define hrt_mipi_backend_disable(mipi_backend_id)
//  _hrt_mipi_backend_set_register(mipi_backend_id, _HRT_MIPI_BACKEND_ENABLE_REG_IDX, 0)
//
//#define hrt_mipi_backend_get_enable_status(mipi_backend_id)
//  _hrt_mipi_backend_get_register(mipi_backend_id, _HRT_MIPI_BACKEND_ENABLE_REG_IDX)
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define hrt_mipi_backend_get_be_status(mipi_backend_id) \
	_hrt_mipi_backend_get_register(mipi_backend_id, _HRT_MIPI_BACKEND_STATUS_REG_IDX)

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// value = memory alignment configuration = 0,1,2,3
// 0 = b00 = 64-bits alignment
// 1 = b01 = 128-bits alignment
// 2 = b10 = 256-bits alignment
// 3 = b11 = 512-bits alignment 
#define hrt_mipi_backend_set_mem_alignment_config(mipi_backend_id, value) \
  _hrt_mipi_backend_set_register(mipi_backend_id, _HRT_MIPI_BACKEND_MEM_ALIGNMENT_CONFIG_REG_IDX, value)

#define hrt_mipi_backend_get_mem_alignment_config(mipi_backend_id) \
  _hrt_mipi_backend_get_register(mipi_backend_id, _HRT_MIPI_BACKEND_MEM_ALIGNMENT_CONFIG_REG_IDX)

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define hrt_mipi_backend_set_comp_scheme_reg(mipi_backend_id, reg_id, value) \
  _hrt_mipi_backend_set_register(mipi_backend_id, (_HRT_MIPI_BACKEND_COMP_FORMAT_REG0_IDX + reg_id), value)

#define _hrt_mipi_backend_set_register(mipi_backend_id, reg_id, val) \
  _hrt_slave_port_store_32_volatile(_hrt_mipi_backend_slave_port(mipi_backend_id), (reg_id<<2), (val))

#define _hrt_mipi_backend_set_bits(mipi_backend_id, reg, lsb, num_bits, val) \
  _hrt_mipi_backend_set_register(mipi_backend_id, reg, \
                                 _hrt_set_bits(_hrt_mipi_backend_get_register(mipi_backend_id, reg), lsb, num_bits, val)\
                                 )
#define _hrt_mipi_backend_get_bits(mipi_backend_id, reg, lsb, num_bits) \
  _hrt_get_bits(_hrt_mipi_backend_get_register(mipi_backend_id, reg), lsb, num_bits)

#define _hrt_mipi_backend_get_comp_scheme_reg_id(vc) \
  HRTCAT(_HRT_MIPI_BACKEND_COMP_FORMAT_REG, HRTCAT(vc, _IDX))

// usd =  user defined data type = 1-8
// comp = compression scheme, 0=no compression, 1=10-6-10, 2=10-7-10, 3=10-8-10, 4=12-6-12, 5=12-7-12, 6=12-8-12
// pred = predictor type: 1=predictor 1, 2=predictor 2
#define hrt_mipi_backend_set_comp_scheme(mipi_backend_id, usd, comp, pred) \
  _hrt_mipi_backend_set_bits(mipi_backend_id, \
  _hrt_mipi_backend_get_comp_scheme_reg_id(0), \
	(_HRT_CSS_RECEIVER_2400_BE_COMP_USD_BITS * ((usd)-1)), \
	_HRT_CSS_RECEIVER_2400_BE_COMP_USD_BITS, \
  ((pred-1) << _HRT_CSS_RECEIVER_2400_BE_COMP_PRED_IDX) + (comp << _HRT_CSS_RECEIVER_2400_BE_COMP_FMT_IDX));

#define hrt_mipi_backend_get_comp_scheme(mipi_backend_id, usd) \
  _hrt_mipi_backend_get_bits(mipi_backend_id, \
  _hrt_mipi_backend_get_comp_scheme_reg_id(0), \
  (_HRT_CSS_RECEIVER_2400_BE_COMP_USD_BITS * ((usd)-1)), \
	_HRT_CSS_RECEIVER_2400_BE_COMP_USD_BITS);

// vc = 0,1,2,3
// usd =  user defined data type = 1-8
// comp = compression scheme, 0=no compression, 1=10-6-10, 2=10-7-10, 3=10-8-10, 4=12-6-12, 5=12-7-12, 6=12-8-12
// pred = predictor type: 1=predictor 1, 2=predictor 2
//#define hrt_mipi_backend_set_comp_scheme(mipi_backend_id, vc, usd, comp, pred)
//  _hrt_mipi_backend_set_bits(mipi_backend_id,
//  _hrt_mipi_backend_get_comp_scheme_reg_id(vc),
//	(_HRT_CSS_RECEIVER_2400_BE_COMP_USD_BITS * ((usd)-1)),
//	_HRT_CSS_RECEIVER_2400_BE_COMP_USD_BITS,
//  ((pred-1) << _HRT_CSS_RECEIVER_2400_BE_COMP_PRED_IDX) + (comp << _HRT_CSS_RECEIVER_2400_BE_COMP_FMT_IDX));

//#define hrt_mipi_backend_get_comp_scheme(mipi_backend_id, vc, usd)
//  _hrt_mipi_backend_get_bits(mipi_backend_id,
//  _hrt_mipi_backend_get_comp_scheme_reg_id(vc),
//  (_HRT_CSS_RECEIVER_2400_BE_COMP_USD_BITS * ((usd)-1)),
//	_HRT_CSS_RECEIVER_2400_BE_COMP_USD_BITS);





/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// raw16_id = format type id to be used for RAW16 (should be any of the reserved long mipi types)
// enable: 0=raw16 processing disabled, 1=raw16 processing is enabled for format typ == raw16_id  
#define hrt_mipi_backend_set_raw16_config_reg(mipi_backend_id, raw16_id, enable) \
  _hrt_mipi_backend_set_register(mipi_backend_id,  \
  _HRT_MIPI_BACKEND_RAW16_CONFIG_REG_IDX, \
  ((raw16_id  << _HRT_CSS_RECEIVER_2400_BE_RAW16_DATAID_IDX) + \
	( enable    << _HRT_CSS_RECEIVER_2400_BE_RAW16_EN_IDX)))

#define hrt_mipi_backend_clear_raw16_config_reg(mipi_backend_id) \
  _hrt_mipi_backend_set_register(mipi_backend_id, \
  _HRT_MIPI_BACKEND_RAW16_CONFIG_REG_IDX, \
  0)

#define hrt_mipi_backend_get_raw16_config_reg(mipi_backend_id) \
  _hrt_mipi_backend_get_register(mipi_backend_id, \
  _HRT_MIPI_BACKEND_RAW16_CONFIG_REG_IDX)

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// raw18_id = format type id to be used for RAW18 (should be any of the reserved long mipi types)
// option: 1,2,3
// enable: 0=raw16 processing disabled, 1=raw16 processing is enabled for format typ == raw16_id 
#define hrt_mipi_backend_set_raw18_config_reg(mipi_backend_id, raw18_id, option, enable) \
  _hrt_mipi_backend_set_register(mipi_backend_id,  \
  _HRT_MIPI_BACKEND_RAW18_CONFIG_REG_IDX, \
  ((raw18_id  << _HRT_CSS_RECEIVER_2400_BE_RAW18_DATAID_IDX) + \
	(option << _HRT_CSS_RECEIVER_2400_BE_RAW18_OPTION_IDX) + \
	(enable      << _HRT_CSS_RECEIVER_2400_BE_RAW18_EN_IDX)))  

#define hrt_mipi_backend_clear_raw18_config_reg(mipi_backend_id) \
  _hrt_mipi_backend_set_register(mipi_backend_id, \
  _HRT_MIPI_BACKEND_RAW18_CONFIG_REG_IDX, \
  0)                                 

#define hrt_mipi_backend_get_raw18_config_reg(mipi_backend_id) \
  _hrt_mipi_backend_get_register(mipi_backend_id, \
  _HRT_MIPI_BACKEND_RAW18_CONFIG_REG_IDX)

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// enable: 0 = no force RAW8, 1 = force RAW8 mode enabled
// option: 0 = all long format types forced to RAW8 encoding (when force raw8 mode is enabled
//         1 = specific long format type (forced_id) forced to RAW8 encoding (when force raw8 mode is enabled  
// forced_id = long format type id to be overruled, according option 1
#define hrt_mipi_backend_set_force_raw8_reg(mipi_backend_id, forced_id, option, enable) \
  _hrt_mipi_backend_set_register(mipi_backend_id, \
	_HRT_MIPI_BACKEND_FORCE_RAW8_REG_IDX, \
	((enable << 0) + \
	 (option << 1) + \
	 (forced_id << 2)))

#define hrt_mipi_backend_clear_force_raw8_reg(mipi_backend_id) \
  _hrt_mipi_backend_set_register(mipi_backend_id, _HRT_MIPI_BACKEND_FORCE_RAW8_REG_IDX, 0) 

#define hrt_mipi_backend_get_force_raw8_reg(mipi_backend_id) \
  _hrt_mipi_backend_get_register(mipi_backend_id, _HRT_MIPI_BACKEND_FORCE_RAW8_REG_IDX)

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// enable   = enable/disable custom mode 
// option   = 0=custom mode for all data_types / 1=custom mode only for data_id specified by data_id field of this register
// data_id  = format_type/data_id for which custom mode is enbabled (option 1)
#define hrt_mipi_backend_set_cust_en_reg(mipi_backend_id, high_prec_en, data_id, option, enable) \
  _hrt_mipi_backend_set_register(mipi_backend_id, \
  _HRT_MIPI_BACKEND_CUST_EN_REG_IDX, \
  ((high_prec_en << BE_CUST_EN_HIGH_PREC_IDX) + \
   (data_id << BE_CUST_EN_DATAID_IDX) + \
   (option << BE_CUST_EN_OPTION_IDX) + \
   (enable << BE_CUST_EN_IDX)))

#define hrt_mipi_backend_clear_cust_en_reg(mipi_backend_id) \
  _hrt_mipi_backend_set_register(mipi_backend_id, _HRT_MIPI_BACKEND_CUST_EN_REG_IDX, 0) 

#define hrt_mipi_backend_get_cust_en_reg(mipi_backend_id) \
  _hrt_mipi_backend_get_register(mipi_backend_id, _HRT_MIPI_BACKEND_CUST_EN_REG_IDX)
  
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
// Sx_valid = enable disable custom data state ( x e {0,1,2}) 
// Sx_getbits = number of bits to fetch  per custom data state  ( x e {0,1,2}) 
#define hrt_mipi_backend_set_cust_data_state_reg(mipi_backend_id, S0_valid, S1_valid, S2_valid, \
                                                 S0_getbits, S1_getbits, S2_getbits) \
  _hrt_mipi_backend_set_register(mipi_backend_id, \
  _HRT_MIPI_BACKEND_CUST_DATA_STATE_REG_IDX, \
  ((S0_valid   << (BE_CUST_DATA_STATE_S0_IDX + BE_CUST_DATA_STATE_VALID_IDX)) + \
   (S1_valid   << (BE_CUST_DATA_STATE_S1_IDX + BE_CUST_DATA_STATE_VALID_IDX)) + \
   (S2_valid   << (BE_CUST_DATA_STATE_S2_IDX + BE_CUST_DATA_STATE_VALID_IDX)) + \
   (S0_getbits << (BE_CUST_DATA_STATE_S0_IDX + BE_CUST_DATA_STATE_GETBITS_IDX)) + \
   (S1_getbits << (BE_CUST_DATA_STATE_S1_IDX + BE_CUST_DATA_STATE_GETBITS_IDX)) + \
   (S2_getbits << (BE_CUST_DATA_STATE_S2_IDX + BE_CUST_DATA_STATE_GETBITS_IDX))))                                                  

#define hrt_mipi_backend_clear_cust_data_state_reg(mipi_backend_id) \
  _hrt_mipi_backend_set_register(mipi_backend_id, _HRT_MIPI_BACKEND_CUST_DATA_STATE_REG_IDX, 0)

#define hrt_mipi_backend_get_cust_data_state_reg(mipi_backend_id) \
  _hrt_mipi_backend_get_register(mipi_backend_id, _HRT_MIPI_BACKEND_CUST_DATA_STATE_REG_IDX)

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define _hrt_mipi_backend_get_cust_pix_reg_id(state_id, pixel_id) \
  HRTCAT(HRTCAT(HRTCAT(_HRT_MIPI_BACKEND_CUST_PIX_EXT_, state_id), pixel_id), _REG_IDX)

// Configure pixel extraction per state
// state_id = S0, S1, S2
// pixel_id = P0, P1, P2, P3 
#define _hrt_mipi_backend_set_pix_ext_reg(mipi_backend_id, state_id, pixel_id, data_align, pix_align, pix_mask, pix_en) \
  _hrt_mipi_backend_set_register(mipi_backend_id, \
  _hrt_mipi_backend_get_cust_pix_reg_id(state_id, pixel_id), \
  ((data_align << BE_CUST_PIX_EXT_DATA_ALIGN_IDX) + \
   (pix_align  << BE_CUST_PIX_EXT_PIX_ALIGN_IDX) + \
   (pix_mask   << BE_CUST_PIX_EXT_PIX_MASK_IDX) + \
   (pix_en     << BE_CUST_PIX_EXT_PIX_EN_IDX)))     

#define _hrt_mipi_backend_clear_pix_ext_reg(mipi_backend_id, state_id, pixel_id) \
  _hrt_mipi_backend_set_register(mipi_backend_id, _hrt_mipi_backend_get_cust_pix_reg_id(state_id, pixel_id), 0)
  
#define _hrt_mipi_backend_get_pix_ext_reg(mipi_backend_id, state_id, pixel_id) \
  _hrt_mipi_backend_get_register(mipi_backend_id, _hrt_mipi_backend_get_cust_pix_reg_id(state_id, pixel_id))  

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Px_valid_eop_config (4 bits) = less_bits_eop(3) & less_bits_valid(2) & normal_eop(1) & normal_valid(0)
#define _hrt_mipi_backend_set_valid_eop_reg(mipi_backend_id, P0_valid_eop_config, P1_valid_eop_config,  \
                                                             P2_valid_eop_config, P3_valid_eop_config) \
  _hrt_mipi_backend_set_register(mipi_backend_id, \
  _HRT_MIPI_BACKEND_CUST_PIX_VALID_EOP_REG_IDX, \
  ((P0_valid_eop_config << BE_CUST_PIX_VALID_EOP_P0_IDX) + \
   (P1_valid_eop_config << BE_CUST_PIX_VALID_EOP_P1_IDX) + \
   (P2_valid_eop_config << BE_CUST_PIX_VALID_EOP_P2_IDX) + \
   (P3_valid_eop_config << BE_CUST_PIX_VALID_EOP_P2_IDX)))

#define _hrt_mipi_backend_clear_valid_eop_reg(mipi_backend_id) \
  _hrt_mipi_backend_set_register(mipi_backend_id, _HRT_MIPI_BACKEND_CUST_PIX_VALID_EOP_REG_IDX, 0)

#define _hrt_mipi_backend_get_valid_eop_reg(mipi_backend_id) \
  _hrt_mipi_backend_get_register(mipi_backend_id, _HRT_MIPI_BACKEND_CUST_PIX_VALID_EOP_REG_IDX)

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define hrt_mipi_backend_set_global_disregard(mipi_backend_id) \
  _hrt_mipi_backend_set_register(mipi_backend_id, _HRT_MIPI_BACKEND_GLOBAL_LUT_DISREGARD_REG_IDX, 1)

#define hrt_mipi_backend_clear_global_disregard(mipi_backend_id) \
  _hrt_mipi_backend_set_register(mipi_backend_id, _HRT_MIPI_BACKEND_GLOBAL_LUT_DISREGARD_REG_IDX, 0)

#define hrt_mipi_backend_get_global_disregard_status(mipi_backend_id) \
  _hrt_mipi_backend_get_register(mipi_backend_id, _HRT_MIPI_BACKEND_GLOBAL_LUT_DISREGARD_REG_IDX)

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
#define hrt_mipi_backend_get_pkt_stall_status(mipi_backend_id) \
	_hrt_mipi_backend_get_register(mipi_backend_id, _HRT_MIPI_BACKEND_PKT_STALL_STATUS_REG_IDX)

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// enable_gsp_through_lp_lut  												
// enable_vsync_insert_for_gsp_lut                    	
#define hrt_mipi_backend_prog_gsp_lp_lut_reg(mipi_backend_id, enable_vsync_insert_for_gsp_lut, enable_gsp_through_lp_lut ) \
  _hrt_mipi_backend_set_register(mipi_backend_id, _HRT_MIPI_BACKEND_PARSE_GSP_THROUGH_LP_LUT_REG_IDX, (enable_vsync_insert_for_gsp_lut << 1 | enable_gsp_through_lp_lut))

#define hrt_mipi_backend_clear_gsp_lp_lut_reg(mipi_backend_id) \
  _hrt_mipi_backend_set_register(mipi_backend_id, _HRT_MIPI_BACKEND_PARSE_GSP_THROUGH_LP_LUT_REG_IDX, 0)

#define hrt_mipi_backend_get_gsp_lp_lut_reg(mipi_backend_id) \
  _hrt_mipi_backend_get_register(mipi_backend_id, _HRT_MIPI_BACKEND_PARSE_GSP_THROUGH_LP_LUT_REG_IDX)

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define hrt_mipi_backend_get_irq_status(mipi_backend_id) \
	_hrt_mipi_backend_get_register(mipi_backend_id, _HRT_MIPI_BACKEND_IRQ_STATUS_REG_IDX)

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define hrt_mipi_backend_clear_irq(mipi_backend_id) \
  _hrt_mipi_backend_set_register(mipi_backend_id, _HRT_MIPI_BACKEND_IRQ_CLEAR_REG_IDX, 0x7)

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define hrt_mipi_backend_sp_lut_enable(mipi_backend_id)

#define hrt_mipi_backend_sp_lut_disable(mipi_backend_id)

#define hrt_mipi_backend_get_sp_lut_enable_status(mipi_backend_id) 1

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// entry_nmbr: 0,1,2,3
// disregard:  0,1
// vc 			:  0,1,2,3	 
//
//#define hrt_mipi_backend_prog_sp_lut(mipi_backend_id, disregard, sid_width, ifc_index, vc) 
//	_hrt_mipi_backend_set_register(mipi_backend_id, (vc + _HRT_MIPI_BACKEND_SP_LUT_ENTRY_0_REG_IDX) , (ifc_index << 1 | disregard))
//
//#define hrt_mipi_backend_read_sp_lut(mipi_backend_id, vc) 
//	_hrt_mipi_backend_get_register(mipi_backend_id, (vc + _HRT_MIPI_BACKEND_SP_LUT_ENTRY_0_REG_IDX) )
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// entry_nmbr: 0,1,2,3
// disregard:  0,1
// vc 			:  0,1,2,3	 
//#define hrt_mipi_backend_prog_lp_lut_entry(mipi_backend_id, entry_nmbr, disregard, sid_width, ifc_index, vc, fmt) 
//	_hrt_mipi_backend_set_register(mipi_backend_id, (entry_nmbr + _HRT_MIPI_BACKEND_NOF_REGISTERS) , ( fmt << (1+sid_width+2) | vc << (1+sid_width) | ifc_index << 1 | disregard))
//
//#define hrt_mipi_backend_read_lp_lut_entry(mipi_backend_id, entry_nmbr) 
//	_hrt_mipi_backend_get_register(mipi_backend_id, (entry_nmbr + _HRT_MIPI_BACKEND_NOF_REGISTERS))
  
  
#endif /* _mipi_backend_HRT_H */


