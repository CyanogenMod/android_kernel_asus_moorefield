#ifndef _ff_scaler_api_h
#define _ff_scaler_api_h

#include "ff_scaler_ccbcr.h"

#define ff_scaler_set_register(FF_id, addr, val) \
	vied_subsystem_store_32(psys0, FF_id + addr, (val))

#define ff_scaler_get_register(FF_id, addr) \
	vied_subsystem_load_32(psys0, FF_id + addr)

#define ff_scaler_enable(FF_id, dm_en, box_en, input_is_bayer) \
	ff_scaler_set_register(FF_id, FF_SCALER_GEN_CTRL_ADDR, ((input_is_bayer) << 3) | ((box_en) << 2) | ((dm_en) << 1) | 1);

#define ff_scaler_disable(FF_id) \
	ff_scaler_set_register(FF_id, FF_SCALER_GEN_CTRL_ADDR, 0);

#define ff_scaler_set_params(FF_id, SF_numerator_h, SF_denominator_h, SF_numerator_v, SF_denominator_v, Norm_Mul_Output, Norm_shift_Output, Norm_shift_ToLB, OutputFrameHeight, OutputFrameWidth, in_sensor_mode, out_sensor_mode)  \
do {                                                                                                                                                                       \
    ff_scaler_set_register(FF_id, FF_SCALER_FACTOR_CTRL_ADDR, ((SF_denominator_v) << 24) | ((SF_numerator_v) << 16) | ((SF_denominator_h) << 8) | (SF_numerator_h));       \
    ff_scaler_set_register(FF_id, FF_SCALER_NORMALIZATION_ADDR, (((Norm_shift_ToLB) << 16) | (Norm_shift_Output) << 8) | (Norm_Mul_Output));                               \
    ff_scaler_set_register(FF_id, FF_SCALER_OUTPUT_FRAME_SIZE_ADDR, (((OutputFrameHeight) << 16) | (OutputFrameWidth)));                                                    \
    ff_scaler_set_register(FF_id, FF_SCALER_OUTPUT_FORMATTER_SENSOR_CFG_0_ADDR, ((out_sensor_mode) << 4) | (in_sensor_mode));                                                                            \
}                                                                                                                                                                          \
while (0)

#define ff_scaler_set_dm_params(FF_id, IsUlW, FirstW)  \
    ff_scaler_set_register(FF_id, FF_SCALER_DEMOSAIC_CTRL_0_ADDR, ((FirstW) << 4) | (IsUlW))

#define ff_scaler_set_sensor_mode(FF_id, in_sensor_mode, out_sensor_mode)  \
    ff_scaler_set_register(FF_id, FF_SCALER_OUTPUT_FORMATTER_SENSOR_CFG_0_ADDR, ((out_sensor_mode) << 4) | (in_sensor_mode))

#define ff_scaler_set_input_frame_width(FF_id, frame_width)  \
    ff_scaler_set_register(FF_id, FF_SCALER_INPUT_FRAME_SIZE_ADDR, (frame_width))

#define ff_scaler_set_matrix(FF_id, arr)                                     \
do {                                                                         \
    unsigned int data = 0;					             \
    unsigned int reg_addr = FF_SCALER_OUTPUT_FORMATTER_SENSOR_CFG_1_ADDR;    \
    unsigned int arr_idx = 0;                                                \
    while (arr_idx < 8) {                                                    \
        data |= arr[arr_idx++] << (4*arr_idx);                               \
    }                                                                        \
    ff_scaler_set_register(FF_id, reg_addr, data);                           \
    reg_addr = FF_SCALER_OUTPUT_FORMATTER_SENSOR_CFG_2_ADDR;                 \
    data  = 0;                                                               \
    while (arr_idx < 16) {                                                   \
        data |= arr[arr_idx++] << (4*(arr_idx-8));                           \
    }                                                                        \
    ff_scaler_set_register(FF_id, reg_addr, data);    	                     \
}                                                                            \
while (0)

#endif
