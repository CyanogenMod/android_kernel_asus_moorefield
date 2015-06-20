/**
 * Copyright @ 2013 uPI Semiconductor Corp. All right reserved.
 * The information, images, and/or data contained in this material is copyrighted by uPI
 * Semiconductor Corp., and may not be distributed, modified, reproduced in whole or in part
 * without the prior, written consent of uPI Semiconductor Corp.
 */

/**
 * @filename  uG31xx_API_Otp.cpp
 *
 *  Convert OTP registers into readable value
 *
 * @author  AllenTeng <allen_teng@upi-semi.com>
 * @revision  $Revision: 25 $
 */

#include "stdafx.h"     //windows need this??
#include "uG31xx_API.h"

#ifdef  uG31xx_OS_WINDOWS

#define OTP_VERSION      (_T("OTP $Rev: 25 $ "))

#else   ///< else of uG31xx_OS_WINDOWS

#define OTP_VERSION      ("OTP $Rev: 25 $ ")

#endif  ///< end of uG31xx_OS_WINDOWS

/// =============================================
/// [AT-PM] : OTP register definition
/// =============================================

#define OTP1_OFFSET_E0                (0)
#define INDEX_ADC1_200_25_0         (1<<0)
#define INDEX_ADC1_200_25_1         (1<<1)
#define INDEX_ADC1_200_25_2         (1<<2)
#define INDEX_ADC1_200_25_3         (1<<3)
#define DELTA_VREF_0                (1<<4)
#define DELTA_VREF_1                (1<<5)
#define DELTA_VREF_2                (1<<6)
#define DELTA_VREF_3                (1<<7)

#define OTP1_OFFSET_E1                (OTP1_OFFSET_E0 + 1)
#define INDEX_ADC1_100_25_0         (1<<0)
#define INDEX_ADC1_100_25_1         (1<<1)
#define INDEX_ADC1_100_25_2         (1<<2)
#define INDEX_ADC1_100_25_3         (1<<3)
#define FT_IT_3                     (1<<4)
#define FT_IT_4                     (1<<5)
#define FT_IT_5                     (1<<6)
#define FT_IT_6                     (1<<7)

#define OTP1_OFFSET_E2                (OTP1_OFFSET_E1 + 1)
#define INDEX_ADC2_200_25_0         (1<<0)
#define INDEX_ADC2_200_25_1         (1<<1)
#define INDEX_ADC2_200_25_2         (1<<2)
#define INDEX_ADC2_200_25_3         (1<<3)
#define FT_IT_7                     (1<<4)
#define FT_IT_8                     (1<<5)
#define FT_IT_9                     (1<<6)
#define FT_IT_10                    (1<<7)

#define OTP1_OFFSET_E3                (OTP1_OFFSET_E2 + 1)
#define INDEX_ADC2_100_25_0         (1<<0)
#define INDEX_ADC2_100_25_1         (1<<1)
#define INDEX_ADC2_100_25_2         (1<<2)
#define INDEX_ADC2_100_25_3         (1<<3)
#define FT_IT_11                    (1<<4)
#define FT_IT_12                    (1<<5)
#define FT_IT_13                    (1<<6)
#define FT_IT_14                    (1<<7)

#define OTP2_OFFSET_F0                (0)
#define OTP2_OFFSET_F0_RSVD_0       (1<<0)
#define OTP2_OFFSET_F0_RSVD_1       (1<<1)
#define OTP2_OFFSET_F0_RSVD_2       (1<<2)
#define PRODUCT_TYPE_0              (1<<3)
#define PRODUCT_TYPE_1              (1<<4)
#define DELTA_ET_0                  (1<<5)
#define INDEX_ADC2_100_25_4         (1<<6)
#define DELTA_ET_1                  (1<<7)

#define OTP2_OFFSET_F1                (OTP2_OFFSET_F0 + 1)
#define OTP2_OFFSET_F1_RSVD_0       (1<<0)
#define OTP2_OFFSET_F1_RSVD_1       (1<<1)
#define OTP2_OFFSET_F1_RSVD_2       (1<<2)
#define OTP2_OFFSET_F1_RSVD_3       (1<<3)
#define OTP2_OFFSET_F1_RSVD_4       (1<<4)
#define OTP2_OFFSET_F1_RSVD_5       (1<<5)
#define OTP2_OFFSET_F1_RSVD_6       (1<<6)
#define OTP2_OFFSET_F1_RSVD_7       (1<<7)

#define OTP2_OFFSET_F2                (OTP2_OFFSET_F1 + 1)
#define OTP2_OFFSET_F2_RSVD_0       (1<<0)
#define OTP2_OFFSET_F2_RSVD_1       (1<<1)
#define OTP2_OFFSET_F2_RSVD_2       (1<<2)
#define OTP_CELL_EN_0               (1<<3)
#define OTP_CELL_EN_1               (1<<4)
#define OTP_CELL_EN_2               (1<<5)
#define OTP_CELL_EN_3               (1<<6)
#define OTP_CELL_EN_4               (1<<7)

#define OTP2_OFFSET_F3                (OTP2_OFFSET_F2 + 1)
#define OTP2_OFFSET_F3_RSVD_0       (1<<0)
#define OTP2_OFFSET_F3_RSVD_1       (1<<1)
#define OTP2_OFFSET_F3_RSVD_2       (1<<2)
#define OTP2_OFFSET_F3_RSVD_3       (1<<3)
#define OTP2_OFFSET_F3_RSVD_4       (1<<4)
#define OTP2_OFFSET_F3_RSVD_5       (1<<5)
#define OTP2_OFFSET_F3_RSVD_6       (1<<6)
#define OTP2_OFFSET_F3_RSVD_7       (1<<7)

#define OTP2_OFFSET_F4                (OTP2_OFFSET_F3 + 1)
#define ADC1_DELTA_CODE_25_200MV_8  (1<<0)
#define ADC1_DELTA_CODE_25_200MV_9  (1<<1)
#define DEV_ADDR_0                  (1<<2)
#define DEV_ADDR_1                  (1<<3)
#define DEV_ADDR_2                  (1<<4)
#define DEV_ADDR_7                  (1<<5)
#define DEV_ADDR_8                  (1<<6)
#define DEV_ADDR_9                  (1<<7)

#define OTP2_OFFSET_F5                (OTP2_OFFSET_F4 + 1)
#define OTP2_OFFSET_F5_RSVD_0       (1<<0)
#define OTP2_OFFSET_F5_RSVD_1       (1<<1)
#define BGR_TUNE_0                  (1<<2)
#define BGR_TUNE_1                  (1<<3)
#define BGR_TUNE_2                  (1<<4)
#define BGR_TUNE_3                  (1<<5)
#define BGR_TUNE_4                  (1<<6)
#define BGR_TUNE_5                  (1<<7)

#define OTP2_OFFSET_F6                (OTP2_OFFSET_F5 + 1)
#define OSC_DELTA_CODE_25_0         (1<<0)
#define OSC_DELTA_CODE_25_1         (1<<1)
#define OSC_DELTA_CODE_25_2         (1<<2)
#define OSC_DELTA_CODE_25_3         (1<<3)
#define OSC_DELTA_CODE_25_4         (1<<4)
#define OSC_DELTA_CODE_25_5         (1<<5)
#define OSC_DELTA_CODE_25_6         (1<<6)
#define OSC_DELTA_CODE_25_7         (1<<7)

#define OTP2_OFFSET_F7                (OTP2_OFFSET_F6 + 1)
#define OSC_DELTA_CODE_80_0         (1<<0)
#define OSC_DELTA_CODE_80_1         (1<<1)
#define OSC_DELTA_CODE_80_2         (1<<2)
#define OSC_DELTA_CODE_80_3         (1<<3)
#define OSC_DELTA_CODE_80_4         (1<<4)
#define OSC_DELTA_CODE_80_5         (1<<5)
#define OSC_DELTA_CODE_80_6         (1<<6)
#define OSC_DELTA_CODE_80_7         (1<<7)

#define OTP2_OFFSET_F8                (OTP2_OFFSET_F7 + 1)
#define ADC1_DELTA_CODE_25_200MV_0  (1<<0)
#define ADC1_DELTA_CODE_25_200MV_1  (1<<1)
#define ADC1_DELTA_CODE_25_200MV_2  (1<<2)
#define ADC1_DELTA_CODE_25_200MV_3  (1<<3)
#define ADC1_DELTA_CODE_25_200MV_4  (1<<4)
#define ADC1_DELTA_CODE_25_200MV_5  (1<<5)
#define ADC1_DELTA_CODE_25_200MV_6  (1<<6)
#define ADC1_DELTA_CODE_25_200MV_7  (1<<7)

#define OTP2_OFFSET_F9                (OTP2_OFFSET_F8 + 1)
#define OTP2_OFFSET_F9_RSVD_0       (1<<0)
#define OTP2_OFFSET_F9_RSVD_1       (1<<1)
#define OTP2_OFFSET_F9_RSVD_2       (1<<2)
#define OTP2_OFFSET_F9_RSVD_3       (1<<3)
#define OTP2_OFFSET_F9_RSVD_4       (1<<4)
#define OTP2_OFFSET_F9_RSVD_5       (1<<5)
#define OTP2_OFFSET_F9_RSVD_6       (1<<6)
#define OTP2_OFFSET_F9_RSVD_7       (1<<7)

#define OTP2_OFFSET_FA                (OTP2_OFFSET_F9 + 1)
#define ADC1_DELTA_CODE_25_100MV_0  (1<<0)
#define ADC1_DELTA_CODE_25_100MV_1  (1<<1)
#define ADC1_DELTA_CODE_25_100MV_2  (1<<2)
#define ADC1_DELTA_CODE_25_100MV_3  (1<<3)
#define ADC1_DELTA_CODE_25_100MV_4  (1<<4)
#define ADC1_DELTA_CODE_25_100MV_5  (1<<5)
#define ADC1_DELTA_CODE_25_100MV_6  (1<<6)
#define ADC1_DELTA_CODE_25_100MV_7  (1<<7)

#define OTP2_OFFSET_FB                (OTP2_OFFSET_FA + 1)
#define OTP2_OFFSET_FB_RSVD_0       (1<<0)
#define OTP2_OFFSET_FB_RSVD_1       (1<<1)
#define OTP2_OFFSET_FB_RSVD_2       (1<<2)
#define OTP2_OFFSET_FB_RSVD_3       (1<<3)
#define OTP2_OFFSET_FB_RSVD_4       (1<<4)
#define OTP2_OFFSET_FB_RSVD_5       (1<<5)
#define OTP2_OFFSET_FB_RSVD_6       (1<<6)
#define OTP2_OFFSET_FB_RSVD_7       (1<<7)

#define OTP2_OFFSET_FC                (OTP2_OFFSET_FB + 1)
#define ADC1_DELTA_CODE_25_100MV_8  (1<<0)
#define ADC2_DELTA_CODE_25_100MV_0  (1<<1)
#define ADC2_DELTA_CODE_25_100MV_1  (1<<2)
#define ADC2_DELTA_CODE_25_100MV_2  (1<<3)
#define ADC2_DELTA_CODE_25_100MV_3  (1<<4)
#define ADC2_DELTA_CODE_25_100MV_4  (1<<5)
#define ADC2_DELTA_CODE_25_100MV_5  (1<<6)
#define ADC2_DELTA_CODE_25_100MV_6  (1<<7)

#define OTP2_OFFSET_FD                (OTP2_OFFSET_FC + 1)
#define OTP2_OFFSET_FD_RSVD_0       (1<<0)
#define OTP2_OFFSET_FD_RSVD_1       (1<<1)
#define OTP2_OFFSET_FD_RSVD_2       (1<<2)
#define OTP2_OFFSET_FD_RSVD_3       (1<<3)
#define OTP2_OFFSET_FD_RSVD_4       (1<<4)
#define OTP2_OFFSET_FD_RSVD_5       (1<<5)
#define OTP2_OFFSET_FD_RSVD_6       (1<<6)
#define OTP2_OFFSET_FD_RSVD_7       (1<<7)

#define OTP2_OFFSET_FE                (OTP2_OFFSET_FD + 1)
#define ADC2_DELTA_CODE_25_200MV_0  (1<<0)
#define ADC2_DELTA_CODE_25_200MV_1  (1<<1)
#define ADC2_DELTA_CODE_25_200MV_2  (1<<2)
#define ADC2_DELTA_CODE_25_200MV_3  (1<<3)
#define ADC2_DELTA_CODE_25_200MV_4  (1<<4)
#define ADC2_DELTA_CODE_25_200MV_5  (1<<5)
#define ADC2_DELTA_CODE_25_200MV_6  (1<<6)
#define ADC2_DELTA_CODE_25_200MV_7  (1<<7)

#define OTP2_OFFSET_FF                (OTP2_OFFSET_FE + 1)
#define OTP2_OFFSET_FF_RSVD_0       (1<<0)
#define OTP2_OFFSET_FF_RSVD_1       (1<<1)
#define OTP2_OFFSET_FF_RSVD_2       (1<<2)
#define OTP2_OFFSET_FF_RSVD_3       (1<<3)
#define OTP2_OFFSET_FF_RSVD_4       (1<<4)
#define OTP2_OFFSET_FF_RSVD_5       (1<<5)
#define OTP2_OFFSET_FF_RSVD_6       (1<<6)
#define OTP2_OFFSET_FF_RSVD_7       (1<<7)


#define OTP3_OFFSET_70                (0)
#define DELTA_VREF_4                (1<<0)
#define DELTA_ET_2                  (1<<1)
#define DELTA_ET_3                  (1<<2)
#define AVE_IT_25_3                 (1<<3)
#define AVE_IT_25_4                 (1<<4)
#define AVE_IT_25_5                 (1<<5)
#define AVE_IT_25_6                 (1<<6)
#define AVE_IT_25_7                 (1<<7)

#define OTP3_OFFSET_71                (OTP3_OFFSET_70 + 1)
#define AVE_IT_25_8                 (1<<0)
#define AVE_IT_25_9                 (1<<1)
#define AVE_IT_25_10                (1<<2)
#define AVE_IT_25_11                (1<<3)
#define AVE_IT_25_12                (1<<4)
#define AVE_IT_25_13                (1<<5)
#define AVE_IT_25_14                (1<<6)
#define AVE_IT_25_15                (1<<7)

#define OTP3_OFFSET_72                (OTP3_OFFSET_71 + 1)
#define INDEX_ADC2_200_25_4         (1<<0)
#define INDEX_ADC1_100_25_4         (1<<1)
#define INDEX_ADC1_200_25_4         (1<<2)
#define AVE_IT_80_3                 (1<<3)
#define AVE_IT_80_4                 (1<<4)
#define AVE_IT_80_5                 (1<<5)
#define AVE_IT_80_6                 (1<<6)
#define AVE_IT_80_7                 (1<<7)

#define OTP3_OFFSET_73                (OTP3_OFFSET_72 + 1)
#define AVE_IT_80_8                 (1<<0)
#define AVE_IT_80_9                 (1<<1)
#define AVE_IT_80_10                (1<<2)
#define AVE_IT_80_11                (1<<3)
#define AVE_IT_80_12                (1<<4)
#define AVE_IT_80_13                (1<<5)
#define AVE_IT_80_14                (1<<6)
#define AVE_IT_80_15                (1<<7)

/// =============================================
/// [AT-PM] : OTP register conversion routine
/// =============================================

/**
 * @brief ConvOtp1E0
 *
 *  Convert OTP1 0xE0
 *
 * @para  obj address of OtpDataType
 * @return  _UPI_NULL_
 */
void ConvOtp1E0(OtpDataType *obj)
{
        _otp_u8_ value;

        value = obj->otp1[OTP1_OFFSET_E0];
#ifdef	UPI_UBOOT_DEBUG_MSG
        printf("[ConvOtp1E0] Initial value of indexAdc1V200T25 = %d\n");
#endif	///< end of UPI_UBOOT_DEBUG_MSG
        obj->indexAdc1V200T25 = obj->indexAdc1V200T25 + (value & (INDEX_ADC1_200_25_0 |
                                INDEX_ADC1_200_25_1 |
                                INDEX_ADC1_200_25_2 |
                                INDEX_ADC1_200_25_3));

        obj->deltaVref = obj->deltaVref + ((value & (DELTA_VREF_0| DELTA_VREF_1 | DELTA_VREF_2 | DELTA_VREF_3)) >> 4);
}

/**
 * @brief ConvOtp1E1
 *
 *  Convert OTP1 0xE1
 *
 * @para  obj address of OtpDataType
 * @return  _UPI_NULL_
 */
void ConvOtp1E1(OtpDataType *obj)
{
        _otp_u8_ value;

        value = obj->otp1[OTP1_OFFSET_E1];

#ifdef	UPI_UBOOT_DEBUG_MSG
        printf("[ConvOtp1E1] Initial value of indexAdc1V100T25 = %d\n", obj->indexAdc1V100T25);
#endif	///< end of UPI_UBOOT_DEBUG_MSG
        obj->indexAdc1V100T25 = obj->indexAdc1V100T25 + (value & (INDEX_ADC1_100_25_0 |
                                INDEX_ADC1_100_25_1 |
                                INDEX_ADC1_100_25_2 |
                                INDEX_ADC1_100_25_3));

        obj->ftIT = obj->ftIT + ((value & (FT_IT_3 | FT_IT_4 | FT_IT_5 | FT_IT_6)) >> 1);
}

/**
 * @brief ConvOtp1E2
 *
 *  Convert OTP1 0xE2
 *
 * @para  obj address of OtpDataType
 * @return  _UPI_NULL_
 */
void ConvOtp1E2(OtpDataType *obj)
{
        _otp_u8_ value;
        _otp_u16_ tmp;

        value = obj->otp1[OTP1_OFFSET_E2];

#ifdef	UiPI_UBOOT_DEBUG_MSG
        printf("[ConvOtp1E2] Initial value of indexAdc2V200T25 = %d\n", obj->indexAdc2V200T25);
#endif	///< end of UiPI_UBOOT_DEBUG_MSG
        obj->indexAdc2V200T25 = obj->indexAdc2V200T25 + (value & (INDEX_ADC2_200_25_0 |
                                INDEX_ADC2_200_25_1 |
                                INDEX_ADC2_200_25_2 |
                                INDEX_ADC2_200_25_3));

        tmp = (value & (FT_IT_7 | FT_IT_8 | FT_IT_9 | FT_IT_10));
        obj->ftIT = obj->ftIT + (tmp << 3);
}

/**
 * @brief ConvOtp1E3
 *
 *  Convert OTP1 0xE3
 *
 * @para  obj address of OtpDataType
 * @return  _UPI_NULL_
 */
void ConvOtp1E3(OtpDataType *obj)
{
        _otp_u8_ value;
        _otp_u16_ tmp;

        value = obj->otp1[OTP1_OFFSET_E3];

#ifdef	UPI_UBOOT_DEBUG_MSG
        printf("[ConvOtp1E3] Initial value of indexAdc2V100T25 = %d\n", obj->indexAdc2V100T25);
#endif	///< end of UPI_UBOOT_DEBUG_MSG
        obj->indexAdc2V100T25 = obj->indexAdc2V100T25 + (value & (INDEX_ADC2_100_25_0 |
                                INDEX_ADC2_100_25_1 |
                                INDEX_ADC2_100_25_2 |
                                INDEX_ADC2_100_25_3));

        tmp = (value & (FT_IT_11 | FT_IT_12 | FT_IT_13 | FT_IT_14));
        obj->ftIT = obj->ftIT + (tmp << 7);
}

/**
 * @brief ConvOtp2F0
 *
 *  Convert OTP2 0xF0
 *
 * @para  obj address of OtpDataType
 * @return  _UPI_NULL_
 */
void ConvOtp2F0(OtpDataType *obj)
{
        _otp_u8_ value;

        value = obj->otp2[OTP2_OFFSET_F0];

        obj->productType = (value & (PRODUCT_TYPE_0 | PRODUCT_TYPE_1)) >> 3;

        obj->deltaET = obj->deltaET + ((value & DELTA_ET_0) >> 5) + ((value & DELTA_ET_1) >> 6);

#ifdef  UPI_UBOOT_DEBUG_MSG
        printf("[ConvOtp2F0] Initial value of indexAdc2V100T25 = %d\n", obj->indexAdc2V100T25);
#endif  ///< end of UPI_UBOOT_DEBUG_MSG
        obj->indexAdc2V100T25 = obj->indexAdc2V100T25 + ((value & INDEX_ADC2_100_25_4) >> 2);
}

/**
 * @brief ConvOtp2F1
 *
 *  Convert OTP2 0xF1
 *
 * @para  obj address of OtpDataType
 * @return  _UPI_NULL_
 */
void ConvOtp2F1(OtpDataType *obj)
{
}

/**
 * @brief ConvOtp2F2
 *
 *  Convert OTP2 0xF2
 *
 * @para  obj address of OtpDataType
 * @return  _UPI_NULL_
 */
void ConvOtp2F2(OtpDataType *obj)
{
        _otp_u8_ value;

        value = obj->otp2[OTP2_OFFSET_F2];

        obj->otpCellEN = obj->otpCellEN + ((value & (OTP_CELL_EN_0 |
                                            OTP_CELL_EN_1 |
                                            OTP_CELL_EN_2 |
                                            OTP_CELL_EN_3 |
                                            OTP_CELL_EN_4)) >> 3);
}

/**
 * @brief ConvOtp2F3
 *
 *  Convert OTP2 0xF3
 *
 * @para  obj address of OtpDataType
 * @return  _UPI_NULL_
 */
void ConvOtp2F3(OtpDataType *obj)
{
}

/**
 * @brief ConvOtp2F4
 *
 *  Convert OTP2 0xF4
 *
 * @para  obj address of OtpDataType
 * @return  _UPI_NULL_
 */
void ConvOtp2F4(OtpDataType *obj)
{
        _otp_u8_ value;
        _otp_u16_ tmp;

        value = obj->otp2[OTP2_OFFSET_F4];

        tmp = value & (ADC1_DELTA_CODE_25_200MV_8| ADC1_DELTA_CODE_25_200MV_9);
#ifdef	UPI_UBOOT_DEBUG_MSG
        printf("[ConvOtp2F4] Initial value of adc1DeltaCodeT25V200 = %d\n", obj->adc1DeltaCodeT25V200);
#endif	///< end of UPI_UBOOT_DEBUG_MSG
        obj->adc1DeltaCodeT25V200 = obj->adc1DeltaCodeT25V200 + (tmp << 8);

        tmp = value & (DEV_ADDR_7 | DEV_ADDR_8 | DEV_ADDR_9);
        obj->devAddr = (tmp << 2) + ((value & (DEV_ADDR_0 | DEV_ADDR_1 | DEV_ADDR_2)) >> 2);
}

/**
 * @brief ConvOtp2F5
 *
 *  Convert OTP2 0xF5
 *
 * @para  obj address of OtpDataType
 * @return  _UPI_NULL_
 */
void ConvOtp2F5(OtpDataType *obj)
{
        _otp_u8_ value;

        value = obj->otp2[OTP2_OFFSET_F5];

        obj->bgrTune = obj->bgrTune + ((value & (BGR_TUNE_0 |
                                        BGR_TUNE_1 |
                                        BGR_TUNE_2 |
                                        BGR_TUNE_3 |
                                        BGR_TUNE_4 |
                                        BGR_TUNE_5)) >> 2);
}

/**
 * @brief ConvOtp2F6
 *
 *  Convert OTP2 0xF6
 *
 * @para  obj address of OtpDataType
 * @return  _UPI_NULL_
 */
void ConvOtp2F6(OtpDataType *obj)
{
        _otp_s16_ value;

        value = (_otp_s16_)obj->otp2[OTP2_OFFSET_F6];
        if(obj->otp2[OTP2_OFFSET_F6] & OSC_DELTA_CODE_25_7) {
                value = value - 256;
        }
        obj->oscDeltaCode25 = (_otp_s8_)value;
}

/**
 * @brief ConvOtp2F7
 *
 *  Convert OTP2 0xF7
 *
 * @para  obj address of OtpDataType
 * @return  _UPI_NULL_
 */
void ConvOtp2F7(OtpDataType *obj)
{
        _otp_s16_ value;

        value = (_otp_s16_)obj->otp2[OTP2_OFFSET_F7];
        if(obj->otp2[OTP2_OFFSET_F7] & OSC_DELTA_CODE_80_7) {
                value = value - 256;
        }
        obj->oscDeltaCode80 = (_otp_s8_)value;
}

/**
 * @brief ConvOtp2F8
 *
 *  Convert OTP2 0xF8
 *
 * @para  obj address of OtpDataType
 * @return  _UPI_NULL_
 */
void ConvOtp2F8(OtpDataType *obj)
{
        _otp_u8_ value;

        value = obj->otp2[OTP2_OFFSET_F8];

#ifdef  UPI_UBOOT_DEBUG_MSG
        printf("[ConvOtp2F8] Initial value of adc1DeltaCodeT25V200 = %d\n", obj->adc1DeltaCodeT25V200);
#endif  ///< end of UPI_UBOOT_DEBUG_MSG
        obj->adc1DeltaCodeT25V200 = obj->adc1DeltaCodeT25V200 + (value & (ADC1_DELTA_CODE_25_200MV_0 |
                                    ADC1_DELTA_CODE_25_200MV_1 |
                                    ADC1_DELTA_CODE_25_200MV_2 |
                                    ADC1_DELTA_CODE_25_200MV_3 |
                                    ADC1_DELTA_CODE_25_200MV_4 |
                                    ADC1_DELTA_CODE_25_200MV_5 |
                                    ADC1_DELTA_CODE_25_200MV_6 |
                                    ADC1_DELTA_CODE_25_200MV_7));
}

/**
 * @brief ConvOtp2F9
 *
 *  Convert OTP2 0xF9
 *
 * @para  obj address of OtpDataType
 * @return  _UPI_NULL_
 */
void ConvOtp2F9(OtpDataType *obj)
{
}

/**
 * @brief ConvOtp2FA
 *
 *  Convert OTP2 0xFA
 *
 * @para  obj address of OtpDataType
 * @return  _UPI_NULL_
 */
void ConvOtp2FA(OtpDataType *obj)
{
        _otp_u8_ value;

        value = obj->otp2[OTP2_OFFSET_FA];

#ifdef	UPI_UBOOT_DEBUG_MSG
        printf("[ConvOtp2FA] Initial value of adc1DeltaCodeT25V100 = %d\n", obj->adc1DeltaCodeT25V100);
#endif	///< end of UPI_UBOOT_DEBUG_MSG
        obj->adc1DeltaCodeT25V100 = obj->adc1DeltaCodeT25V100 + (value & (ADC1_DELTA_CODE_25_100MV_0 |
                                    ADC1_DELTA_CODE_25_100MV_1 |
                                    ADC1_DELTA_CODE_25_100MV_2 |
                                    ADC1_DELTA_CODE_25_100MV_3 |
                                    ADC1_DELTA_CODE_25_100MV_4 |
                                    ADC1_DELTA_CODE_25_100MV_5 |
                                    ADC1_DELTA_CODE_25_100MV_6 |
                                    ADC1_DELTA_CODE_25_100MV_7));
}

/**
 * @brief ConvOtp2FB
 *
 *  Convert OTP2 0xFB
 *
 * @para  obj address of OtpDataType
 * @return  _UPI_NULL_
 */
void ConvOtp2FB(OtpDataType *obj)
{
}

/**
 * @brief ConvOtp2FC
 *
 *  Convert OTP2 0xFC
 *
 * @para  obj address of OtpDataType
 * @return  _UPI_NULL_
 */
void ConvOtp2FC(OtpDataType *obj)
{
        _otp_u8_ value;
        _otp_u16_ tmp;

        value = obj->otp2[OTP2_OFFSET_FC];

        tmp = value & ADC1_DELTA_CODE_25_100MV_8;
#ifdef	UPI_UBOOT_DEBUG_MSG
        printf("[ConvOtp2FC] Initial value of = %d\n", obj->adc1DeltaCodeT25V100);
#endif	///< end of UPI_UBOOT_DEBUG_MSG
        obj->adc1DeltaCodeT25V100 = obj->adc1DeltaCodeT25V100 + (tmp << 8);

#ifdef  UPI_UBOOT_DEBUG_MSG
        printf("[ConvOtp2FC] Initial value of = %d\n", obj->adc2DeltaCodeT25V100);
#endif  ///< end of UPI_UBOOT_DEBUG_MSG
        obj->adc2DeltaCodeT25V100 = obj->adc2DeltaCodeT25V100 + ((value & (ADC2_DELTA_CODE_25_100MV_0 |
                                    ADC2_DELTA_CODE_25_100MV_1 |
                                    ADC2_DELTA_CODE_25_100MV_2 |
                                    ADC2_DELTA_CODE_25_100MV_3 |
                                    ADC2_DELTA_CODE_25_100MV_4 |
                                    ADC2_DELTA_CODE_25_100MV_5 |
                                    ADC2_DELTA_CODE_25_100MV_6)) >> 1);
}

/**
 * @brief ConvOtp2FD
 *
 *  Convert OTP2 0xFD
 *
 * @para  obj address of OtpDataType
 * @return  _UPI_NULL_
 */
void ConvOtp2FD(OtpDataType *obj)
{
}

/**
 * @brief ConvOtp2FE
 *
 *  Convert OTP2 0xFE
 *
 * @para  obj address of OtpDataType
 * @return  _UPI_NULL_
 */
void ConvOtp2FE(OtpDataType *obj)
{
        _otp_u8_ value;

        value = obj->otp2[OTP2_OFFSET_FE];

#ifdef	UPI_UBOOT_DEBUG_MSG
        printf("[ConvOtp2FE] Initial value of adc2DeltaCodeT25V200 = %d\n", obj->adc2DeltaCodeT25V200);
#endif	///< end of UPI_UBOOT_DEBUG_MSG
        obj->adc2DeltaCodeT25V200 = obj->adc2DeltaCodeT25V200 + (value & (ADC2_DELTA_CODE_25_200MV_0 |
                                    ADC2_DELTA_CODE_25_200MV_1 |
                                    ADC2_DELTA_CODE_25_200MV_2 |
                                    ADC2_DELTA_CODE_25_200MV_3 |
                                    ADC2_DELTA_CODE_25_200MV_4 |
                                    ADC2_DELTA_CODE_25_200MV_5 |
                                    ADC2_DELTA_CODE_25_200MV_6 |
                                    ADC2_DELTA_CODE_25_200MV_7));
}

/**
 * @brief ConvOtp2FF
 *
 *  Convert OTP2 0xFF
 *
 * @para  obj address of OtpDataType
 * @return  _UPI_NULL_
 */
void ConvOtp2FF(OtpDataType *obj)
{
}

/**
 * @brief ConvOtp370
 *
 *  Convert OTP3 0x70
 *
 * @para  obj address of OtpDataType
 * @return  _UPI_NULL_
 */
void ConvOtp370(OtpDataType *obj)
{
        _otp_u8_ value;

        value = obj->otp3[OTP3_OFFSET_70];

        obj->deltaVref = obj->deltaVref + ((value & DELTA_VREF_4) << 4);

        obj->deltaET = obj->deltaET + ((value & (DELTA_ET_2 | DELTA_ET_3)) << 1);

        obj->aveIT25 = obj->aveIT25 + (value & (AVE_IT_25_3 | AVE_IT_25_4 | AVE_IT_25_5 | AVE_IT_25_6 | AVE_IT_25_7));
}

/**
 * @brief ConvOtp371
 *
 *  Convert OTP3 0x71
 *
 * @para  obj address of OtpDataType
 * @return  _UPI_NULL_
 */
void ConvOtp371(OtpDataType *obj)
{
        _otp_u8_ value;
        _otp_u16_ tmp;

        value = obj->otp3[OTP3_OFFSET_71];

        tmp = value & (AVE_IT_25_8 |
                       AVE_IT_25_9 |
                       AVE_IT_25_10 |
                       AVE_IT_25_11 |
                       AVE_IT_25_12 |
                       AVE_IT_25_13 |
                       AVE_IT_25_14 |
                       AVE_IT_25_15);
        obj->aveIT25 = obj->aveIT25 + (tmp << 8);
}

/**
 * @brief ConvOtp372
 *
 *  Convert OTP3 0x72
 *
 * @para  obj address of OtpDataType
 * @return  _UPI_NULL_
 */
void ConvOtp372(OtpDataType *obj)
{
        _otp_u8_ value;

        value = obj->otp3[OTP3_OFFSET_72];

#ifdef  UPI_UBOOT_DEBUG_MSG
        printf("[ConvOtp372] Initial value of indexAdc2V200T25 = %d\n", obj->indexAdc2V200T25);
#endif  ///< end of UPI_UBOOT_DEBUG_MSG
        obj->indexAdc2V200T25 = obj->indexAdc2V200T25 + ((value & INDEX_ADC2_200_25_4) << 4);

#ifdef  UPI_UBOOT_DEBUG_MSG
        printf("[ConvOtp372] Initial value of indexAdc1V100T25 = %d\n", obj->indexAdc1V100T25);
#endif  ///< end of UPI_UBOOT_DEBUG_MSG
        obj->indexAdc1V100T25 = obj->indexAdc1V100T25 + ((value & INDEX_ADC1_100_25_4) << 3);

#ifdef  UPI_UBOOT_DEBUG_MSG
        printf("[ConvOtp372] Initial value of indexAdc1V200T25 = %d\n", obj->indexAdc1V200T25);
#endif  ///< end of UPI_UBOOT_DEBUG_MSG
        obj->indexAdc1V200T25 = obj->indexAdc1V200T25 + ((value & INDEX_ADC1_200_25_4) << 2);

        obj->aveIT80 = obj->aveIT80 + (value & (AVE_IT_80_3 | AVE_IT_80_4 | AVE_IT_80_5 | AVE_IT_80_6 | AVE_IT_80_7));
}

/**
 * @brief ConvOtp373
 *
 *  Convert OTP3 0x73
 *
 * @para  obj address of OtpDataType
 * @return  _UPI_NULL_
 */
void ConvOtp373(OtpDataType *obj)
{
        _otp_u8_ value;
        _otp_u16_ tmp;

        value = obj->otp3[OTP3_OFFSET_73];

        tmp = value & (AVE_IT_80_8 |
                       AVE_IT_80_9 |
                       AVE_IT_80_10 |
                       AVE_IT_80_11 |
                       AVE_IT_80_12 |
                       AVE_IT_80_13 |
                       AVE_IT_80_14 |
                       AVE_IT_80_15);
        obj->aveIT80 = obj->aveIT80 + (tmp << 8);
}

#define CONV_FUNC_PTR_NULL  (0)

typedef void (*ConvFuncPtr)(OtpDataType *obj);

static ConvFuncPtr ConvFuncTable[] = {
        ConvOtp1E0,
        ConvOtp1E1,
        ConvOtp1E2,
        ConvOtp1E3,

        ConvOtp2F0,
        ConvOtp2F1,
        ConvOtp2F2,
        ConvOtp2F3,
        ConvOtp2F4,
        ConvOtp2F5,
        ConvOtp2F6,
        ConvOtp2F7,
        ConvOtp2F8,
        ConvOtp2F9,
        ConvOtp2FA,
        ConvOtp2FB,
        ConvOtp2FC,
        ConvOtp2FD,
        ConvOtp2FE,
        ConvOtp2FF,

        ConvOtp370,
        ConvOtp371,
        ConvOtp372,
        ConvOtp373,

        CONV_FUNC_PTR_NULL,
};

/**
 * @brief CheckOtpISEmpty
 *
 *  Check OTP is empty or not
 *
 * @para  data  address of OtpDataType
 * @return  _UPI_NULL_
 */
void CheckOtpISEmpty(OtpDataType *data)
{
        _otp_u8_ idx;

        /// [AT-PM] : Check OTP1 ; 01/25/2013
        idx = 0;
        while(idx < OTP1_SIZE) {
                if(data->otp1[idx] != 0) {
                        data->empty = OTP_IS_NOT_EMPTY;
                        return;
                }
                idx = idx + 1;
        }

        /// [AT-PM] : Check OTP2 ; 01/25/2013
        idx = 0;
        while(idx < OTP2_SIZE) {
                if(data->otp2[idx] != 0) {
                        data->empty = OTP_IS_NOT_EMPTY;
                        return;
                }
                idx = idx + 1;
        }

        /// [AT-PM] : Check OTP3 ; 01/25/2013
        idx = 0;
        while(idx < OTP3_SIZE) {
                if(data->otp3[idx] != 0) {
                        data->empty = OTP_IS_NOT_EMPTY;
                        return;
                }
                idx = idx + 1;
        }

        /// [AT-PM] : Set OTP is empty ; 01/25/2013
        data->empty = OTP_IS_EMPTY;
}

/// =============================================
/// [AT-PM] : Extern function region
/// =============================================

/**
 * @brief UpiConvertOtp
 *
 *  Convert OTP register value to readable value
 *
 * @para  data  address of OtpDataType
 * @return  _UPI_NULL_
 */
void UpiConvertOtp(OtpDataType *data)
{
        _otp_u8_ idx;

        UG31_LOGI("[%s]: %s : %04x%04x\n", __func__, OTP_VERSION, UG31XX_OTP_VERSION_MAIN, UG31XX_OTP_VERSION_SUB);

        /// [AT-PM] : Set version ; 01/25/2013
        data->versionMain = UG31XX_OTP_VERSION_MAIN;
        data->versionSub = UG31XX_OTP_VERSION_SUB;

        /// [AT-PM] : Conversion ; 01/23/2013
        idx = 0;
        while(1) {
                (*ConvFuncTable[idx])(data);

                idx = idx + 1;
                if(ConvFuncTable[idx] == CONV_FUNC_PTR_NULL) {
                        break;
                }
        }

        /// [AT-PM] : Check OTP is empty ; 01/25/2013
        CheckOtpISEmpty(data);
}


/**
 * Copyright @ 2013 uPI Semiconductor Corp. All right reserved.
 * The information, images, and/or data contained in this material is copyrighted by uPI
 * Semiconductor Corp., and may not be distributed, modified, reproduced in whole or in part
 * without the prior, written consent of uPI Semiconductor Corp.
 */
