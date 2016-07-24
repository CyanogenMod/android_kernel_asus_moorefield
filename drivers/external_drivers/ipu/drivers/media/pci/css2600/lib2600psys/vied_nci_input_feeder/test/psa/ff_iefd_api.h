#ifndef _FF_IEFD_API_H
#define _FF_IEFD_API_H
#include "ff_iefd_ccbcr.h"


#define FF_iefd_set_register(FF_IEFD_id, addr, val) \
  vied_subsystem_store_32(psys0, FF_IEFD_id + addr, (val))

#define FF_iefd_get_register(FF_IEFD_id, addr) \
  vied_subsystem_load_32(psys0, FF_IEFD_id + addr)

#define FF_iefd_enable(FF_IEFD_id, denoise_en, m_direct_smooth_en, m_rad_en, m_vssnlm_en)				            \
  FF_iefd_set_register(FF_IEFD_id, IEFD_CONTROL_ADDR, ((m_vssnlm_en) << 4) | ((m_rad_en) << 3) | ((m_direct_smooth_en) << 2) | ((denoise_en) << 1) | 1)

#define FF_iefd_disable(FF_IEFD_id)								                                     \
   FF_iefd_set_register(FF_IEFD_id, IEFD_CONTROL_ADDR, 0x00)

#define FF_iefd_config_params(FF_IEFD_id, m_horver_diag_coeff, m_clamp_stitch, m_direct_metric_update, m_ed_horver_diag_coeff)                  \
   FF_iefd_set_register(FF_IEFD_id, IEFD_CONFIG_ADDR, ((m_ed_horver_diag_coeff) << 24) | ((m_direct_metric_update) << 16) | ((m_clamp_stitch) << 8) | (m_horver_diag_coeff))


#define FF_iefd_shrpn_lmt_params(FF_IEFD_id, m_shrpn_nega_lmt_txt, m_shrpn_posi_lmt_txt, m_shrpn_nega_lmt_dir, m_shrpn_posi_lmt_dir) \
do {                                                                                                                                  \
     FF_iefd_set_register(FF_IEFD_id, M_SHRPN_NEGA_LMT_TXT_ADDR,  (m_shrpn_nega_lmt_txt)); \
     FF_iefd_set_register(FF_IEFD_id, M_SHRPN_POSI_LMT_TXT_ADDR,  (m_shrpn_posi_lmt_txt)); \
     FF_iefd_set_register(FF_IEFD_id, M_SHRPN_NEGA_LMT_DIR_ADDR,  (m_shrpn_nega_lmt_dir)); \
     FF_iefd_set_register(FF_IEFD_id, M_SHRPN_POSI_LMT_DIR_ADDR,  (m_shrpn_posi_lmt_dir)); \
} while (0);

#define FF_iefd_far_params(FF_IEFD_id, m_dir_far_sharp_w, m_dir_far_dns_w, m_ndir_far_dns_power)          \
   FF_iefd_set_register(FF_IEFD_id, M_FAR_W_ADDR, ((m_ndir_far_dns_power) << 16) | ((m_dir_far_dns_w) << 8) | (m_dir_far_sharp_w))

#define FF_iefd_unsharp_params(FF_IEFD_id, m_unsharp_weight, m_unsharp_amount)                             \
  FF_iefd_set_register(FF_IEFD_id, UNSHARPCFG_ADDR, ((m_unsharp_amount) << 8) | (m_unsharp_weight))

#define FF_iefd_unsharp_coeff_params(FF_IEFD_id, c00, c01, c02, c11, c12, c22)                             \
do {                                                                                                       \
     FF_iefd_set_register(FF_IEFD_id, UNSHARPCOEF0_ADDR, ((c02) << 18) | ((c01) << 9) | (c00));            \
     FF_iefd_set_register(FF_IEFD_id, UNSHARPCOEF1_ADDR, ((c22) << 18) | ((c12) << 9) | (c11));            \
} while (0);

#define FF_iefd_rad_reset_params(FF_IEFD_id, x, y, x2, y2)                                                 \
do{                                                                                                        \
     FF_iefd_set_register(FF_IEFD_id, RADIALRESETXY_ADDR, (((y) & 0xFFFF) << 16) | ((x) & 0xFFFF));        \
     FF_iefd_set_register(FF_IEFD_id, RADIALRESETX2_ADDR, (x2));                                           \
     FF_iefd_set_register(FF_IEFD_id, RADIALRESETY2_ADDR, (y2));                                           \
} while (0);

#define FF_iefd_cfg_rad_params(FF_IEFD_id, m_rad_nf, m_rad_inv_r2, m_rad_dir_far_sharp_w, m_rad_dir_far_dns_w, m_rad_ndir_far_dns_power) \
do{                                                                                                                                                               \
     FF_iefd_set_register(FF_IEFD_id, RADIALCFG_ADDR, ((m_rad_inv_r2) << 8) | (m_rad_nf));                                    \
     FF_iefd_set_register(FF_IEFD_id, M_RAD_FAR_W_ADDR, ((m_rad_ndir_far_dns_power) << 16) | ((m_rad_dir_far_dns_w) << 8) | (m_rad_dir_far_sharp_w));             \
} while (0);

#define FF_iefd_cfg_CU_params(FF_IEFD_id, m_CU6_pow, m_CU_Unsharp_pow, m_rad_CU6_pow, m_rad_CU_Unsharp_pow, m_rad_CU6_X1, m_rad_CU_Unsharp_X1)                 \
do{                                                                                                                                                                                                    \
     FF_iefd_set_register(FF_IEFD_id, CUCFG0_ADDR, ((m_rad_CU_Unsharp_pow) << 24) | ((m_rad_CU6_pow) << 16) | ((m_CU_Unsharp_pow) << 8) | (m_CU6_pow)); \
     FF_iefd_set_register(FF_IEFD_id, CUCFG1_ADDR, ((m_rad_CU_Unsharp_X1) << 10) | (m_rad_CU6_X1));                                                                           \
} while (0);

#define FF_iefd_cfg_VS_params(FF_IEFD_id, m_vs_x0, m_vs_x1, m_vs_x2, m_vs_y1, m_vs_y2, m_vs_y3)                       \
do {                                                                                                                  \
      FF_iefd_set_register(FF_IEFD_id, VSS_LUT_X_ADDR,  ((m_vs_x2) << 16) | ((m_vs_x1) << 8) | (m_vs_x0));            \
      FF_iefd_set_register(FF_IEFD_id, VSS_LUT_Y_ADDR,  ((m_vs_y3) << 16) | ((m_vs_y2) << 8) | (m_vs_y1));            \
} while (0);

#define FF_iefd_set_iefd_configUnit2x(FF_IEFD_id, baseAddr, xarr, Aarr)                                               \
do {                                                                                                                  \
   FF_iefd_set_register(FF_IEFD_id, baseAddr       , (Aarr[0] << 18) | (xarr[1] << 9) | xarr[0]);                     \
}                                                                                                                     \
while (0)

#define FF_iefd_set_iefd_configUnit2x_1(FF_IEFD_id, baseAddr, xarr, Aarr, Barr)                                             \
do {                                                                                                                  \
   FF_iefd_set_register(FF_IEFD_id, baseAddr       , (Aarr[0] << 18) | (xarr[1] << 9) | xarr[0]);                     \
   FF_iefd_set_register(FF_IEFD_id, baseAddr + 0x04,                                  (Barr[0]));                     \
}                                                                                                                     \
while (0)

#define FF_iefd_set_iefd_configUnit6x_ED(FF_IEFD_id, baseAddr, xarr, Aarr, Barr)                                                \
do {                                                                                                                            \
    FF_iefd_set_register(FF_IEFD_id, baseAddr       , (xarr[2] << 18) | (xarr[1] << 9) | xarr[0]);                              \
    FF_iefd_set_register(FF_IEFD_id, baseAddr + 0x04, (xarr[5] << 18) | (xarr[4] << 9) | xarr[3]);                              \
    FF_iefd_set_register(FF_IEFD_id, baseAddr + 0x08, ((Aarr[2]& 0x1FF) << 18) | ((Aarr[1]& 0x1FF) << 9) | (Aarr[0] & 0x1FF));  \
    FF_iefd_set_register(FF_IEFD_id, baseAddr + 0x0C, ((Aarr[4]& 0x1FF) << 9) | (Aarr[3] & 0x1FF));                             \
    FF_iefd_set_register(FF_IEFD_id, baseAddr + 0x10, ((Barr[2]& 0x1FF) << 18) | ((Barr[1]& 0x1FF) << 9) | (Barr[0] & 0x1FF));  \
    FF_iefd_set_register(FF_IEFD_id, baseAddr + 0x14, ((Barr[4]& 0x1FF) << 9) | (Barr[3] & 0x1FF));                             \
}                                                                                                                               \
while (0)

#define FF_iefd_set_iefd_configUnit4x(FF_IEFD_id, baseAddr, xarr, Aarr, Barr)                                                     \
do {                                                                                                                              \
       FF_iefd_set_register(FF_IEFD_id, baseAddr       , (xarr[2] << 18) | (xarr[1] << 9) | xarr[0]);                             \
       FF_iefd_set_register(FF_IEFD_id, baseAddr + 0x04, ((Aarr[1] & 0x1FF) << 18) | ((Aarr[0] & 0x1FF) << 9) | xarr[3]);         \
       FF_iefd_set_register(FF_IEFD_id, baseAddr + 0x08, ((Barr[1]& 0xFF) << 17) | ((Barr[0] & 0xFF) << 9) | (Aarr[2] & 0x1FF));  \
       FF_iefd_set_register(FF_IEFD_id, baseAddr + 0x0C,                             (Barr[2] & 0xFF));                           \
       }                                                                                                                          \
while (0)

#define FF_iefd_set_iefd_configUnit6x_RAD(FF_IEFD_id, baseAddr, xarr, Aarr, Barr)                                                     \
do {                                                                                                                                  \
    FF_iefd_set_register(FF_IEFD_id, baseAddr       , (xarr[3] << 24) | (xarr[2] << 16) | (xarr[1] << 8) | xarr[0]);                  \
    FF_iefd_set_register(FF_IEFD_id, baseAddr + 0x04,                                     (xarr[5] << 8) | xarr[4]);                  \
    FF_iefd_set_register(FF_IEFD_id, baseAddr + 0x08, ((Aarr[1]& 0xFFFF) << 16) | (Aarr[0] & 0xFFFF));                                \
    FF_iefd_set_register(FF_IEFD_id, baseAddr + 0x0C, ((Aarr[3]& 0xFFFF) << 16) | (Aarr[2] & 0xFFFF));                                \
    FF_iefd_set_register(FF_IEFD_id, baseAddr + 0x10,                             (Aarr[4] & 0xFFFF));                                \
    FF_iefd_set_register(FF_IEFD_id, baseAddr + 0x14, ((Barr[2]& 0x3FF) << 20) | ((Barr[1]& 0x3FF) << 10) | (Barr[0] & 0x3FF));       \
    FF_iefd_set_register(FF_IEFD_id, baseAddr + 0x18, ((Barr[4]& 0x3FF) << 10) | (Barr[3] & 0x3FF));                                  \
}                                                                                                                                     \
while (0)

#endif // _FF_IEFD_API_H
