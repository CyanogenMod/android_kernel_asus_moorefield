/**
 * Copyright @ 2013 uPI Semiconductor Corp. All right reserved.
 * The information, images, and/or data contained in this material is copyrighted by uPI
 * Semiconductor Corp., and may not be distributed, modified, reproduced in whole or in part
 * without the prior, written consent of uPI Semiconductor Corp.
 */

/**
 * @filename  uG31xx_API_Measurement.cpp
 *
 *  guG31xx measurement API
 *
 * @author  AllenTeng <allen_teng@upi-semi.com>
 * @revision  $Revision: 476 $
 */

#include "stdafx.h"     //windows need this??
#include "uG31xx_API.h"

#ifdef  uG31xx_OS_WINDOWS

#define MEASUREMENT_VERSION      (_T("Measurement $Rev: 476 $"))

#else   ///< else of uG31xx_OS_WINDOWS

#define MEASUREMENT_VERSION      ("Measurement $Rev: 476 $")

#endif  ///< end of uG31xx_OS_WINDOWS

//#define MEAS_FAKE_INT_TEMP

#ifdef  MEAS_FAKE_INT_TEMP

#define MEAS_FAKE_INT_TEMP_OFFSET     (200)

#endif  ///< end of MEAS_FAKE_INT_TEMP

typedef struct MeasDataInternalST {

        MeasDataType *info;

        _meas_s16_ adc1CodeT25V100;
        _meas_s16_ adc1CodeT25V200;
        _meas_s16_ adc1CodeT80V100;
        _meas_s16_ adc1CodeT80V200;

        _meas_s16_ adc2CodeT25V100;
        _meas_s16_ adc2CodeT25V200;
        _meas_s16_ adc2CodeT80V100;
        _meas_s16_ adc2CodeT80V200;

        _meas_u32_ currTime;

        _meas_u16_ codeBat1;
        _meas_s16_ codeCurrent;
        _meas_u16_ codeIntTemperature;
        _meas_u16_ codeExtTemperature;
        _meas_s16_ codeCharge;
        _meas_u16_ codeCounter;
        _meas_s16_ ccOffset;
        _meas_s16_ codeExtTemperatureComp;
} ALIGNED_ATTRIBUTE MeasDataInternalType;

#ifndef UG31XX_SHELL_ALGORITHM

static MeasDataInternalType measData;

#endif  ///< end of UG31XX_SHELL_ALGORITHM

typedef struct BoardFactorST {
        _meas_u16_ voltage_gain;
        _meas_s16_ voltage_offset;
        _meas_u16_ current_gain;
        _meas_s16_ current_offset;
        _meas_s16_ int_temp_offset;
        _meas_s16_ ext_temp_offset;
} ALIGNED_ATTRIBUTE BoardFactorType;

#define BOARD_FACTOR_CONST                  (1000)

static BoardFactorType BoardFactor[] = {
        { 998,  1,  1075, -4, -23,  13  },      ///< uG3105
        { 998,  3,  1066, 10, -23,  13  },      ///< uG310x
        { 1000, 0,  1000, 0,  0,    0   },      ///< default
        { 1000, 0,  1000, 0,  0,    0   },      ///< default
};

typedef struct AdcDeltaCodeMappingST {
        _meas_s32_ Adc1V100;
        _meas_s32_ Adc1V200;
        _meas_s32_ Adc2V100;
        _meas_s32_ Adc2V200;
} ALIGNED_ATTRIBUTE AdcDeltaCodeMappingType;

static AdcDeltaCodeMappingType AdcDeltaCodeMapping[] = {
        { -12800, -25600, 1536,  0    },     ///< Index = 0
        { -12544, -25088, 1600,  128  },     ///< Index = 1
        { -13056, -26112, 1472, -128  },     ///< Index = 2
        { -12288, -24576, 1664,  256  },     ///< Index = 3
        { -13312, -26624, 1408, -256  },     ///< Index = 4
        { -12032, -24064, 1728,  384  },     ///< Index = 5
        { -13568, -27136, 1344, -384  },     ///< Index = 6
        { -11776, -23552, 1792,  512  },     ///< Index = 7
        { -13824, -27648, 1280, -512  },     ///< Index = 8
        { -11520, -23040, 1856,  640  },     ///< Index = 9
        { -14080, -28160, 1216, -640  },     ///< Index = 10
        { -11264, -22528, 1920,  768  },     ///< Index = 11
        { -14336, -28672, 1152, -768  },     ///< Index = 12
        { -11008, -22016, 1984,  896  },     ///< Index = 13
        { -14592, -29184, 1088, -896  },     ///< Index = 14
        { -10752, -21504, 2048,  1024 },     ///< Index = 15
        { -14848, -29696, 1024, -1024 },     ///< Index = 16
        { -10496, -20992, 2112,  1152 },     ///< Index = 17
        { -15104, -30208, 960,  -1152 },     ///< Index = 18
        { -10240, -20480, 2176,  1280 },     ///< Index = 19
        { -15360, -30720, 896,  -1280 },     ///< Index = 20
        { -9984,  -19968, 2240,  1408 },     ///< Index = 21
        { -15616, -31232, 832,  -1408 },     ///< Index = 22
        { -9728,  -19456, 2304,  1536 },     ///< Index = 23
        { -15872, -31744, 768,  -1536 },     ///< Index = 24
        { -9472,  -18944, 2368,  1664 },     ///< Index = 25
        { -16128, -32256, 704,  -1664 },     ///< Index = 26
        { -9216,  -18432, 2432,  1792 },     ///< Index = 27
        { -16384, -32768, 640,  -1792 },     ///< Index = 28
        { -8960,  -17920, 2496,  1920 },     ///< Index = 29
        { -16640, -33280, 576,  -1920 },     ///< Index = 30
        { 0,      0,      0,     0    },     ///< Index = 31
};

#define ADC_TEMPERATURE_GAIN_CONST            (1000)

#define ADC1_CODE_100MV_NEGATIVE              (0xFF00)
#define ADC1_CODE_200MV_NEGATIVE              (0xFE00)
#define ADC1_CP_CODE_25_100MV                 (12288)
#define ADC1_CP_CODE_25_200MV                 (24576)
#define ADC1_DELTA_CODE_25_100MV_SIGN_BIT     (1<<8)
#define ADC1_DELTA_CODE_25_200MV_SIGN_BIT     (1<<9)
#define ADC1_TEMPERATURE_GAIN_100MV           (869600)
#define ADC1_TEMPERATURE_GAIN_200MV           (-695680)

/**
 * @brief ConvertAdc1Data
 *
 *  Convert ADC1 data from OTP
 *
 * @para  obj address of MeasDataInternalType
 * @return  _UPI_NULL_
 */
void ConvertAdc1Data(MeasDataInternalType *obj)
{
        _meas_u16_ tmp16;
        _meas_s32_ tmp32;

        /// [AT-PM] : Get code T25 100mV ; 01/23/2013
        tmp16 = obj->info->otp->adc1DeltaCodeT25V100;
        if(tmp16 & ADC1_DELTA_CODE_25_100MV_SIGN_BIT) {
                tmp16 = tmp16 & (~ADC1_DELTA_CODE_25_100MV_SIGN_BIT);
                if(tmp16 != 0) {
                        tmp16 = tmp16 + ADC1_CODE_100MV_NEGATIVE;
                }
        }
        tmp16 = tmp16 + ADC1_CP_CODE_25_100MV;
        tmp32 = (_meas_s32_)(_meas_s16_)tmp16;
        tmp32 = tmp32 + AdcDeltaCodeMapping[obj->info->otp->indexAdc1V100T25].Adc1V100;
        obj->adc1CodeT25V100 = (_meas_s16_)tmp32;
#ifdef	UPI_UBOOT_DEBUG_MSG
        printf("[ConvertAdc1Data] adc1CodeT25V100 = %d\n", tmp32);
#endif	///< end of UPI_UBOOT_DEBUG_MSG

        /// [AT-PM] : Get code T25 200mV ; 01/23/2013
        tmp16 = obj->info->otp->adc1DeltaCodeT25V200;
        if(tmp16 & ADC1_DELTA_CODE_25_200MV_SIGN_BIT) {
                tmp16 = tmp16 & (~ADC1_DELTA_CODE_25_200MV_SIGN_BIT);
                if(tmp16 != 0) {
                        tmp16 = tmp16 + ADC1_CODE_200MV_NEGATIVE;
                }
        }
        tmp16 = tmp16 + ADC1_CP_CODE_25_200MV;
        tmp32 = (_meas_s32_)(_meas_s16_)tmp16;
        tmp32 = tmp32 + AdcDeltaCodeMapping[obj->info->otp->indexAdc1V200T25].Adc1V200;
        obj->adc1CodeT25V200 = (_meas_s16_)tmp32;
#ifdef  UPI_UBOOT_DEBUG_MSG
        printf("[ConvertAdc1Data] adc1CodeT25V200 = %d\n", tmp32);
#endif  ///< end of UPI_UBOOT_DEBUG_MSG

        tmp32 = (_meas_s32_)obj->info->otp->aveIT80;
        tmp32 = (tmp32 - obj->info->otp->aveIT25)*ADC_TEMPERATURE_GAIN_CONST;

        /// [AT-PM] : Get code T80 100mV ; 01/23/2013
        obj->adc1CodeT80V100 = (_meas_s16_)(tmp32/ADC1_TEMPERATURE_GAIN_100MV + obj->adc1CodeT25V100);
#ifdef  UPI_UBOOT_DEBUG_MSG
        printf("[ConvertAdc1Data] adc1CodeT80V100 = %d\n", obj->adc1CodeT80V100);
#endif  ///< end of UPI_UBOOT_DEBUG_MSG

        /// [AT-PM] : Get code T80 200mV ; 01/23/2013
        obj->adc1CodeT80V200 = (_meas_s16_)(tmp32/ADC1_TEMPERATURE_GAIN_200MV + obj->adc1CodeT25V200);
#ifdef  UPI_UBOOT_DEBUG_MSG
        printf("[ConvertAdc1Data] adc1CodeT80V200 = %d\n", obj->adc1CodeT80V200);
#endif  ///< end of UPI_UBOOT_DEBUG_MSG
}

#define ADC2_CODE_100MV_NEGATIVE              (0xFFC0)
#define ADC2_CODE_200MV_NEGATIVE              (0xFF80)
#define ADC2_CP_CODE_25_100MV                 (3072)
#define ADC2_CP_CODE_25_200MV                 (6144)
#define ADC2_DELTA_CODE_25_100MV_SIGN_BIT     (1<<6)
#define ADC2_DELTA_CODE_25_200MV_SIGN_BIT     (1<<7)
#define ADC2_TEMPERATURE_GAIN_100MV           (-149130)
#define ADC2_TEMPERATURE_GAIN_200MV           (-136937)

/**
 * @brief ConvertAdc2Data
 *
 *  Convert ADC2 data from OTP
 *
 * @para  obj address of MeasDataInternalType
 * @return  _UPI_NULL_
 */
void ConvertAdc2Data(MeasDataInternalType *obj)
{
        _meas_u16_ tmp16;
        _meas_s32_ tmp32;

        /// [AT-PM] : Get code T25 100mV ; 01/23/2013
        tmp16 = obj->info->otp->adc2DeltaCodeT25V100;
        if(tmp16 & ADC2_DELTA_CODE_25_100MV_SIGN_BIT) {
                tmp16 = tmp16 & (~ADC2_DELTA_CODE_25_100MV_SIGN_BIT);
                tmp16 = tmp16 + ADC2_CODE_100MV_NEGATIVE;
        }
        tmp16 = tmp16 + ADC2_CP_CODE_25_100MV;
        tmp32 = (_meas_s32_)(_meas_s16_)tmp16;
        tmp32 = tmp32 + AdcDeltaCodeMapping[obj->info->otp->indexAdc2V100T25].Adc2V100;
        obj->adc2CodeT25V100 = (_meas_s16_)tmp32;
#ifdef  UPI_UBOOT_DEBUG_MSG
        printf("[ConvertAdc2Data] adc2CodeT25V100 = %d\n", tmp32);
#endif  ///< end of UPI_UBOOT_DEBUG_MSG

        /// [AT-PM] : Get code T25 200mV ; 01/23/2013
        tmp16 = obj->info->otp->adc2DeltaCodeT25V200;
        if(tmp16 & ADC2_DELTA_CODE_25_200MV_SIGN_BIT) {
                tmp16 = tmp16 & (~ADC2_DELTA_CODE_25_200MV_SIGN_BIT);
                tmp16 = tmp16 + ADC2_CODE_200MV_NEGATIVE;
        }
        tmp16 = tmp16 + ADC2_CP_CODE_25_200MV;
        tmp32 = (_meas_s32_)(_meas_s16_)tmp16;
        tmp32 = tmp32 + AdcDeltaCodeMapping[obj->info->otp->indexAdc2V200T25].Adc2V200;
        obj->adc2CodeT25V200 = (_meas_s16_)tmp32;
#ifdef  UPI_UBOOT_DEBUG_MSG
        printf("[ConvertAdc2Data] adc2CodeT25V200 = %d\n", tmp32);
#endif  ///< end of UPI_UBOOT_DEBUG_MSG

        tmp32 = (_meas_s32_)obj->info->otp->aveIT80;
        tmp32 = (tmp32 - obj->info->otp->aveIT25)*ADC_TEMPERATURE_GAIN_CONST;

        /// [AT-PM] : Get code T80 100mV ; 01/23/2013
        obj->adc2CodeT80V100 = (_meas_s16_)(tmp32/ADC2_TEMPERATURE_GAIN_100MV + obj->adc2CodeT25V100);
#ifdef  UPI_UBOOT_DEBUG_MSG
        printf("[ConvertAdc2Data] adc2CodeT80V100 = %d\n", obj->adc2CodeT80V100);
#endif  ///< end of UPI_UBOOT_DEBUG_MSG

        /// [AT-PM] : Get code T80 200mV ; 01/23/2013
        obj->adc2CodeT80V200 = (_meas_s16_)(tmp32/ADC2_TEMPERATURE_GAIN_200MV + obj->adc2CodeT25V200);
#ifdef  UPI_UBOOT_DEBUG_MSG
        printf("[ConvertAdc2Data] adc2CodeT80V200 = %d\n", obj->adc2CodeT80V200);
#endif  ///< end of UPI_UBOOT_DEBUG_MSG
}

/**
 * @brief CalAdc1Factors
 *
 *  Calculate ADC1 gain slope and factor B
 *  Calculate ADC1 offset slope and factor O
 *
 * @para  obj address of MeasDataInternalType
 * @return  _UPI_NULL_
 */
void CalAdc1Factors(MeasDataInternalType *obj)
{
        _meas_s32_ delta25;
        _meas_s32_ delta80;
        _meas_s64_ tmp64;

        /// [AT-PM] : Calculate gain slope and factor B ; 01/23/2013
        delta25 = (_meas_s32_)obj->adc1CodeT25V200;
        delta25 = delta25 - obj->adc1CodeT25V100;
        delta80 = (_meas_s32_)obj->adc1CodeT80V200;
        delta80 = delta80 - obj->adc1CodeT80V100;

        obj->info->adc1GainSlope = delta80 - delta25;
        obj->info->adc1GainFactorB = delta25*(obj->info->otp->aveIT80) - delta80*(obj->info->otp->aveIT25);
#ifdef  UPI_UBOOT_DEBUG_MSG
        printf("[CalAdc1Factors] adc1GainSlope / adc1GainFactorB  = %d / %d\n", obj->info->adc1GainSlope, obj->info->adc1GainFactorB);
#endif  ///< end of UPI_UBOOT_DEBUG_MSG

        /// [AT-PM] : Calculate offset slope and factor O ; 01/23/2013
        delta25 = (_meas_s32_)obj->adc1CodeT25V100;
        delta25 = delta25*2 - obj->adc1CodeT25V200;
        delta80 = (_meas_s32_)obj->adc1CodeT80V100;
        delta80 = delta80*2 - obj->adc1CodeT80V200;

        obj->info->adc1OffsetSlope = delta80 - delta25;
        obj->info->adc1OffsetFactorO = delta25*(obj->info->otp->aveIT80) - delta80*(obj->info->otp->aveIT25);
#ifdef  UPI_UBOOT_DEBUG_MSG
        printf("[CalAdc1Factors] adc1OffsetSlope / adc1OffsetFactorO = %d / %d\n", obj->info->adc1OffsetSlope, obj->info->adc1OffsetFactorO);
#endif  ///< end of UPI_UBOOT_DEBUG_MSG

        /// [AT-PM] : Calculate current ADC1 gain ; 01/23/2013
        tmp64 = (_meas_s64_)obj->info->adc1GainSlope;
        tmp64 = tmp64*(obj->codeIntTemperature) + obj->info->adc1GainFactorB;
        obj->info->adc1Gain = (_meas_s32_)tmp64;

        /// [AT-PM] : Calculate current ADC1 offset ; 01/23/2013
        tmp64 = (_meas_s64_)obj->info->adc1OffsetSlope;
        tmp64 = tmp64*(obj->codeIntTemperature) + obj->info->adc1OffsetFactorO;
        obj->info->adc1Offset = (_meas_s32_)tmp64;
#ifdef	UPI_UBOOT_DEBUG_MSG
        printf("[CalAdc1Factors] adc1Gain / adc1Offset = %d / %d\n", obj->info->adc1Gain, obj->info->adc1Offset);
#endif	///< end of UPI_UBOOT_DEBUG_MSG
}

/**
 * @brief CalAdc2Factors
 *
 *  Calculate ADC2 gain slope and factor B
 *  Calculate ADC2 offset slope and factor O
 *
 * @para  obj address of MeasDataInternalType
 * @return  _UPI_NULL_
 */
void CalAdc2Factors(MeasDataInternalType *obj)
{
        _meas_s32_ delta25;
        _meas_s32_ delta80;
        _meas_s64_ tmp64;

        /// [AT-PM] : Calculate gain slope and factor B ; 01/23/2013
        delta25 = (_meas_s32_)obj->adc2CodeT25V200;
        delta25 = delta25 - obj->adc2CodeT25V100;
        delta80 = (_meas_s32_)obj->adc2CodeT80V200;
        delta80 = delta80 - obj->adc2CodeT80V100;

        obj->info->adc2GainSlope = delta80 - delta25;
        obj->info->adc2GainFactorB = delta25*(obj->info->otp->aveIT80) - delta80*(obj->info->otp->aveIT25);
#ifdef	UPI_UBOOT_DEBUG_MSG
        printf("[CalAdc2Factors] adc2GainSlope / adc2GainFactorB = %d / %d\n", obj->info->adc2GainSlope, obj->info->adc2GainFactorB);
#endif	///< end of UPI_UBOOT_DEBUG_MSG

        /// [AT-PM] : Calculate offset slope and factor O ; 01/23/2013
        delta25 = (_meas_s32_)obj->adc2CodeT25V100;
        delta25 = delta25*2 - obj->adc2CodeT25V200;
        delta80 = (_meas_s32_)obj->adc2CodeT80V100;
        delta80 = delta80*2 - obj->adc2CodeT80V200;

        obj->info->adc2OffsetSlope = delta80 - delta25;
        obj->info->adc2OffsetFactorO = delta25*(obj->info->otp->aveIT80) - delta80*(obj->info->otp->aveIT25);
#ifdef  UPI_UBOOT_DEBUG_MSG
        printf("[CalAdc2Factors] adc2OffsetSlope / adc2OffsetFactorO = %d / %d\n", obj->info->adc2OffsetSlope, obj->info->adc2OffsetFactorO);
#endif  ///< end of UPI_UBOOT_DEBUG_MSG

        /// [AT-PM] : Calculate current ADC1 gain ; 01/23/2013
        tmp64 = (_meas_s64_)obj->info->adc2GainSlope;
        tmp64 = tmp64*(obj->codeIntTemperature) + obj->info->adc2GainFactorB;
        obj->info->adc2Gain = (_meas_s32_)tmp64;

        /// [AT-PM] : Calculate current ADC1 offset ; 01/23/2013
        tmp64 = (_meas_s64_)obj->info->adc2OffsetSlope;
        tmp64 = tmp64*(obj->codeIntTemperature) + obj->info->adc2OffsetFactorO;
        obj->info->adc2Offset = (_meas_s32_)tmp64;
#ifdef  UPI_UBOOT_DEBUG_MSG
        printf("[CalAdc2Factors] adc2Gain / adc2Offset = %d / %d\n", obj->info->adc2Gain, obj->info->adc2Offset);
#endif  ///< end of UPI_UBOOT_DEBUG_MSG
}

#define ADC1_IDEAL_CODE_100MV     (614)
#define ADC1_IDEAL_CODE_200MV     (1229)
#define ADC1_IDEAL_CODE_DELTA     (ADC1_IDEAL_CODE_200MV - ADC1_IDEAL_CODE_100MV)

/**
 * @brief CalibrateAdc1Code
 *
 *  Calibrate ADC1 code
 *
 * @para  obj address of MeasDataInternalType
 * @para  code  ADC1 code to be calibrated
 * @return  calibrated code
 */
_meas_s32_ CalibrateAdc1Code(MeasDataInternalType *obj, _meas_s32_ code)
{
        _meas_s64_ tmp64;
        _meas_s32_ tmp32;
        _meas_s32_ deltaIT;
        _meas_s32_ gain;
        _meas_s32_ offset;

        deltaIT = (_meas_s32_)obj->info->otp->aveIT80;
        deltaIT = deltaIT - obj->info->otp->aveIT25;

        /// [AT-PM] : Pre-operation to avoid 64-bit division ; 01/23/2013
        gain = obj->info->adc1Gain;
        offset = obj->info->adc1Offset;
        while(1) {
                tmp64 = (_meas_s64_)code;
                tmp64 = tmp64*deltaIT - offset;
                tmp64 = tmp64*ADC1_IDEAL_CODE_DELTA;
                if((tmp64 < 2147483647) && (tmp64 > -2147483647)) {
                        break;
                }
                code = code/2;
                deltaIT = deltaIT/2;
                gain = gain/4;
                offset = offset/4;
        }

        tmp32 = (_meas_s32_)tmp64;
        tmp32 = tmp32/gain;
#ifdef	UPI_UBOOT_DEBUG_MSG
        printf("[CalibrateAdc1Code] %d -> %d\n", code, tmp32);
#endif	///< end of UPI_UBOOT_DEBUG_MSG
        return (tmp32);
}

#define ADC2_IDEAL_CODE_100MV     (ADC2_CP_CODE_25_100MV)
#define ADC2_IDEAL_CODE_200MV     (ADC2_CP_CODE_25_200MV)
#define ADC2_IDEAL_CODE_DELTA     (ADC2_IDEAL_CODE_200MV - ADC2_IDEAL_CODE_100MV)

/**
 * @brief CalibrateAdc2Code
 *
 *  Calibrate ADC2 code
 *
 * @para  obj address of MeasDataInternalType
 * @para  code  ADC2 code to be calibrated
 * @return  calibrated code
 */
_meas_s32_ CalibrateAdc2Code(MeasDataInternalType *obj, _meas_s32_ code)
{
        _meas_s64_ tmp64;
        _meas_s32_ tmp32;
        _meas_s32_ deltaIT;
        _meas_s32_ gain;
        _meas_s32_ offset;

        deltaIT = (_meas_s32_)obj->info->otp->aveIT80;
        deltaIT = deltaIT - obj->info->otp->aveIT25;

        /// [AT-PM] : Pre-operation to avoid 64-bit division ; 01/23/2013
        gain = obj->info->adc2Gain;
        offset = obj->info->adc2Offset;
        while(1) {
                tmp64 = (_meas_s64_)code;
                tmp64 = tmp64*deltaIT - offset;
                tmp64 = tmp64*ADC2_IDEAL_CODE_DELTA;
                if((tmp64 < 2147483647) && (tmp64 > -2147483647)) {
                        break;
                }
                code = code/2;
                deltaIT = deltaIT/2;
                gain = gain/4;
                offset = offset/4;
        }

        tmp32 = (_meas_s32_)tmp64;
        tmp32 = tmp32/gain;
#ifdef  UPI_UBOOT_DEBUG_MSG
        printf("[CalibrateAdc2Code] %d -> %d\n", code, tmp32);
#endif  ///< end of UPI_UBOOT_DEBUG_MSG
        return (tmp32);
}

#define IT_IDEAL_CODE_25      (24310)
#define IT_IDEAL_CODE_80      (28612)
#define IT_IDEAL_CODE_DELTA   (IT_IDEAL_CODE_80 - IT_IDEAL_CODE_25)

/**
 * @brief CalibrateITCode
 *
 *  Calibrate internal temperature code
 *
 * @para  obj address of MeasDataInternalType
 * @para  itCode  raw IT code
 * @return  calibrated IT code
 */
_meas_u16_ CalibrateITCode(MeasDataInternalType *obj, _meas_u16_ itCode)
{
        _meas_s32_ tmp32;

        tmp32 = (_meas_s32_)itCode;
        tmp32 = tmp32 - obj->info->otp->aveIT25;
        tmp32 = tmp32*IT_IDEAL_CODE_DELTA;
        tmp32 = tmp32/(obj->info->otp->aveIT80 - obj->info->otp->aveIT25);
        tmp32 = tmp32 + IT_IDEAL_CODE_25;
        return ((_meas_u16_)tmp32);
}

#define NORMAL_REGISTER     (NORMAL)

/**
 * @brief CalibrateChargeCode
 *
 *  Calibrate charge code
 *
 * @para  obj address of MeasDataInternalType
 * @return  _UPI_NULL_
 */
void CalibrateChargeCode(MeasDataInternalType *obj)
{
        _meas_s32_ tmp32;
        _meas_s64_ tmp64;
        _meas_s32_ gain;
        _meas_s32_ offset;

        UG31_LOGN("[%s]: Raw Code = %d\n", __func__, obj->codeCharge);
#ifdef	UPI_UBOOT_DEBUG_MSG
        printf("[CalibrateChargeCode]: Raw Code = %d\n", obj->codeCharge);
#endif	///< end of UPI_UBOOT_DEBUG_MSG

        /// [AT-PM] : Calibrate charge code ; 01/23/2013
        obj->info->codeCharge = CalibrateAdc1Code(obj, ((_meas_s32_)obj->codeCharge)*2);

        /// [AT-PM] : Remove the offset in calibrated charge code ; 01/23/2013
        gain = obj->info->adc1Gain;
        offset = obj->info->adc1Offset;
        while(1) {
                tmp64 = (_meas_s64_)offset;
                tmp64 = tmp64*ADC1_IDEAL_CODE_DELTA;
                if((tmp64 < 2147483647) && (tmp64 > -2147483647)) {
                        break;
                }
                gain = gain/2;
                offset = offset/2;
        }
        tmp32 = (_meas_s32_)tmp64;
        tmp32 = tmp32/gain;
        UG31_LOGN("[%s]: Compensation = %d x %d / %d\n", __func__,
                  (int)obj->info->adc1Offset, ADC1_IDEAL_CODE_DELTA, (int)obj->info->adc1Gain);
#ifdef	UPI_UBOOT_DEBUG_MSG
        printf("[CalibrateChargeCode]: Compensation = %d x %d / %d\n", obj->info->adc1Offset, ADC1_IDEAL_CODE_DELTA, obj->info->adc1Gain);
#endif	///< end of UPI_UBOOT_DEBUG_MSG
        obj->info->codeCharge = obj->info->codeCharge + tmp32;
        UG31_LOGI("[%s]: Charge = %d\n", __func__, (int)obj->info->codeCharge);
#ifdef	UPI_UBOOT_DEBUG_MSG
        printf("[CalibrateChargeCode]: Charge = %d\n", obj->info->codeCharge);
#endif	///< end of UPI_UBOOT_DEBUG_MSG
}

#define ADC2_VOLTAGE_100MV    (3000)                                    ///< [AT-PM] : Unit in mV ; 01/25/2013
#define ADC2_VOLTAGE_200MV    (4000)                                    ///< [AT-PM] : Unit in mV ; 01/25/2013
#define ADC2_VOLTAGE_DELTA    (ADC2_VOLTAGE_200MV - ADC2_VOLTAGE_100MV)

/**
 * @brief ConvertBat1
 *
 *  Convert code of BAT1
 *
 * @para  obj address of MeasDataInternalType
 * @return  _UPI_NULL_
 */
void ConvertBat1(MeasDataInternalType *obj)
{
        _meas_s32_ tmp32;

        /// [AT-PM] : Convert from calibrated ADC code ; 01/25/2013
        tmp32 = (_meas_s32_)obj->info->codeBat1;
        tmp32 = tmp32 - ADC2_IDEAL_CODE_100MV;
        tmp32 = tmp32*ADC2_VOLTAGE_DELTA/ADC2_IDEAL_CODE_DELTA;
        tmp32 = tmp32 + ADC2_VOLTAGE_100MV;

        /// [AT-PM] : Apply board factor ; 01/25/2013
        tmp32 = tmp32 - BoardFactor[GET_PRODUCT_TYPE(obj->info->sysData->ggbParameter->NacLmdAdjustCfg)].voltage_offset;
        tmp32 = tmp32*BOARD_FACTOR_CONST/BoardFactor[GET_PRODUCT_TYPE(obj->info->sysData->ggbParameter->NacLmdAdjustCfg)].voltage_gain;

        /// [AT-PM] : Apply calibration parameter ; 01/25/2013
        tmp32 = tmp32 - obj->info->sysData->ggbParameter->adc2_offset;
        tmp32 = tmp32*CALIBRATION_FACTOR_CONST/obj->info->sysData->ggbParameter->adc2_gain;
        obj->info->bat1Voltage = (_meas_u16_)tmp32;
}

#define ADC1_VOLTAGE_100MV    (-5000)                                   ///< [AT-PM] : Unit in uV ; 01/25/2013
#define ADC1_VOLTAGE_200MV    (-10000)                                  ///< [AT-PM] : Unit in uV ; 01/25/2013
#define ADC1_VOLTAGE_DELTA    (ADC1_VOLTAGE_200MV - ADC1_VOLTAGE_100MV)
/**
 * @brief ConvertCurrent
 *
 *  Convert code of Current
 *
 * @para  obj address of MeasDataInternalType
 * @return  _UPI_NULL_
 */
void ConvertCurrent(MeasDataInternalType *obj)
{
        _meas_s32_ tmp32;

        /// [AT-PM] : Convert from calibrated ADC code ; 01/25/2013
        tmp32 = (_meas_s32_)obj->info->codeCurrent;
        tmp32 = tmp32 - ADC1_IDEAL_CODE_100MV;
        tmp32 = tmp32*ADC1_VOLTAGE_DELTA/ADC1_IDEAL_CODE_DELTA;
        tmp32 = tmp32 + ADC1_VOLTAGE_100MV;
        tmp32 = tmp32/obj->info->sysData->ggbParameter->rSense;

        /// [AT-PM] : Apply board factor ; 01/25/2013
        tmp32 = tmp32 - BoardFactor[GET_PRODUCT_TYPE(obj->info->sysData->ggbParameter->NacLmdAdjustCfg)].current_offset;
        tmp32 = tmp32*BOARD_FACTOR_CONST/BoardFactor[GET_PRODUCT_TYPE(obj->info->sysData->ggbParameter->NacLmdAdjustCfg)].current_gain;

        /// [AT-PM] : Apply calibration factor ; 01/25/2013
        tmp32 = tmp32 - obj->info->sysData->ggbParameter->adc1_pos_offset - obj->info->ccOffsetAdj;
        if(tmp32 > 0) {
                tmp32 = tmp32*CALIBRATION_FACTOR_CONST/obj->info->sysData->ggbParameter->adc1_pgain;
        } else {
                tmp32 = tmp32*CALIBRATION_FACTOR_CONST/obj->info->sysData->ggbParameter->adc1_ngain;
        }
        obj->info->curr = (_meas_s16_)tmp32;
        if(MEAS_REVERSE_CURRENT_DIRECTION(obj->info->status) == _UPI_TRUE_) {
                obj->info->curr = obj->info->curr*(-1);
        }

        if((MEAS_CABLE_OUT(obj->info->status) == _UPI_TRUE_) && (obj->info->curr > 0)) {
                obj->info->curr = 0;
        }
}

#define AMBIENT_TEMPERATURE_IN_FT (220)
#define IT_CONST                  (100)
#define IT_GAIN                   (392)
#define IT_OFFSET                 (11172)

static _meas_s16_ FTAmbientMappingTable[] = {
        AMBIENT_TEMPERATURE_IN_FT,            ///< Index = 0
        AMBIENT_TEMPERATURE_IN_FT + 10,       ///< Index = 1
        AMBIENT_TEMPERATURE_IN_FT - 10,       ///< Index = 2
        AMBIENT_TEMPERATURE_IN_FT + 20,       ///< Index = 3
        AMBIENT_TEMPERATURE_IN_FT - 20,       ///< Index = 4
        AMBIENT_TEMPERATURE_IN_FT + 30,       ///< Index = 5
        AMBIENT_TEMPERATURE_IN_FT - 30,       ///< Index = 6
        AMBIENT_TEMPERATURE_IN_FT + 40,       ///< Index = 7
        AMBIENT_TEMPERATURE_IN_FT - 40,       ///< Index = 8
        AMBIENT_TEMPERATURE_IN_FT + 50,       ///< Index = 9
        AMBIENT_TEMPERATURE_IN_FT - 50,       ///< Index = 10
        AMBIENT_TEMPERATURE_IN_FT + 60,       ///< Index = 11
        AMBIENT_TEMPERATURE_IN_FT - 60,       ///< Index = 12
        AMBIENT_TEMPERATURE_IN_FT + 70,       ///< Index = 13
        AMBIENT_TEMPERATURE_IN_FT - 70,       ///< Index = 14
        AMBIENT_TEMPERATURE_IN_FT,            ///< Index = 15
};

/**
 * @brief ConvertIntTemperature
 *
 *  Convert code of internal temperature
 *
 * @para  obj address of MeasDataInternalType
 * @return  _UPI_NULL_
 */
void ConvertIntTemperature(MeasDataInternalType *obj)
{
        _meas_s32_ tmp32;
        _meas_s32_ ftIT;

        /// [AT-PM] : Convert from calibrated ADC code ; 01/25/2013
        tmp32 = (_meas_s32_)obj->info->codeIntTemperature;
        tmp32 = tmp32/2;
        tmp32 = tmp32 - IT_OFFSET;
        tmp32 = tmp32*IT_CONST/IT_GAIN;

        /// [AT-PM] : Apply FT information ; 01/25/2013
        ftIT = (_meas_s32_)CalibrateITCode(obj, obj->info->otp->ftIT);
        ftIT = ftIT/2;
        ftIT = ftIT - IT_OFFSET;
        ftIT = ftIT*IT_CONST/IT_GAIN;
        tmp32 = tmp32 - (ftIT - FTAmbientMappingTable[obj->info->otp->deltaET]);

        /// [AT-PM] : Apply board factor ; 01/25/2013
        tmp32 = tmp32 - BoardFactor[GET_PRODUCT_TYPE(obj->info->sysData->ggbParameter->NacLmdAdjustCfg)].int_temp_offset;

        /// [AT-PM] : Apply calibration factor ; 01/25/2013
        tmp32 = tmp32 - obj->info->sysData->ggbParameter->adc_d5;
        obj->info->intTemperature = (_meas_s16_)tmp32;
}

static _meas_s16_ ExtTemperatureTable[] = {
        -100,       ///< Index = 0
        -50,        ///< Index = 1
        0,          ///< Index = 2
        50,         ///< Index = 3
        100,        ///< Index = 4
        150,        ///< Index = 5
        200,        ///< Index = 6
        250,        ///< Index = 7
        300,        ///< Index = 8
        350,        ///< Index = 9
        400,        ///< Index = 10
        450,        ///< Index = 11
        500,        ///< Index = 12
        550,        ///< Index = 13
        600,        ///< Index = 14
        650,        ///< Index = 15
        700,        ///< Index = 16
        750,        ///< Index = 17
        800,        ///< Index = 18
};

/**
 * @brief ConvertExtTemperature
 *
 *  Convert code of external temperature
 *
 * @para  obj address of MeasDataInternalType
 * @return  _UPI_NULL_
 */
void ConvertExtTemperature(MeasDataInternalType *obj)
{
        _meas_u8_ idx;
        _meas_s32_ tmp32;
        _meas_s16_ tmp16;

        /// [AT-PM] : Get adc code of voltage divider ; 11/01/2013
        tmp16 = (_meas_s16_)obj->info->sysData->ggbParameter->adc_d3;
        tmp16 = tmp16 - obj->codeExtTemperatureComp - obj->info->codeExtTemperature;
        UG31_LOGN("[%s]: Voltage divider = %d\n", __func__, tmp16);

        /// [AT-PM] : Calculate NTC resistance ; 11/01/2013
        tmp32 = (_meas_s32_)obj->info->codeExtTemperature;
        tmp32 = tmp32*(obj->info->sysData->ggbParameter->R)/tmp16;
        UG31_LOGI("[%s]: NTC resistance = %d x %d / %d = %d\n", __func__,
                  obj->info->codeExtTemperature, obj->info->sysData->ggbParameter->R, tmp16, (int)tmp32);

        idx = 0;
        while(idx < ET_NUMS) {
                if(tmp32 >= obj->info->sysData->ggbParameter->rtTable[idx]) {
                        break;
                }
                idx = idx + 1;
        }

        if(idx == 0) {
                /// [AT-PM] : Minimum measurable temperature ; 01/25/2013
                tmp32 = (_meas_s32_)ExtTemperatureTable[0];
        } else if(idx == ET_NUMS) {
                /// [AT-PM] : Maximum measurable temperature ; 01/25/2013
                tmp32 = (_meas_s32_)ExtTemperatureTable[ET_NUMS - 1];
        } else {
                /// [AT-PM] : Calculate external temperature ; 01/25/2013
                tmp32 = tmp32 - obj->info->sysData->ggbParameter->rtTable[idx];
                tmp32 = tmp32*(ExtTemperatureTable[idx - 1] - ExtTemperatureTable[idx]);
                tmp32 = tmp32/(obj->info->sysData->ggbParameter->rtTable[idx - 1] - obj->info->sysData->ggbParameter->rtTable[idx]);
                tmp32 = tmp32 + ExtTemperatureTable[idx];
        }

        /// [AT-PM] : Apply board factor ; 01/25/2013
        tmp32 = tmp32 - BoardFactor[GET_PRODUCT_TYPE(obj->info->sysData->ggbParameter->NacLmdAdjustCfg)].ext_temp_offset;

        /// [AT-PM] : Apply calibration factor ; 01/25/2013
        tmp32 = tmp32 - obj->info->sysData->ggbParameter->adc_d4;

        /// [AT-PM] : Average to avoid unexpected jumping ; 07/04/2013
        tmp32 = tmp32*ET_AVERAGE_NEW + obj->info->extTemperature*ET_AVERAGE_OLD;
        tmp32 = tmp32/ET_AVERAGE_BASE;
        obj->info->extTemperature = (_meas_s16_)tmp32;
}

#define MINIMUM_ADC1_COUNTER_FOR_CONVERT_TIME (10)
#define MAXIMUM_ADC1_CONVERSION_TIME          (0xf8)
#define MINIMUM_ADC1_CONVERSION_TIME          (0x08)

/**
 * @brief CalculateAdc1ConvertTime
 *
 *  Calculate ADC1 conversion time
 *
 * @para  obj address of MeasDataInternalType
 * @return  _UPI_NULL_
 */
void CalculateAdc1ConvertTime(MeasDataInternalType *obj)
{
        _meas_u32_ tmp32;

        UG31_LOGN("[%s]: Initial conversion time = %d\n", __func__, obj->info->adc1ConvertTime);

        /// [AT-PM] : Check internal suspend mode ; 08/01/2013
        if(MEAS_IN_SUSPEND_MODE(obj->info->status) == _UPI_TRUE_) {
                obj->info->lastCounter = obj->codeCounter;
                return;
        }

        /// [AT-PM] : First time to calculate ADC1 conversion time ; 01/25/2013
        if(obj->info->adc1ConvertTime == 0) {
                obj->info->adc1ConvertTime = TIME_DEFAULT_ADC1_CONVERT_TIME;
                obj->info->lastCounter = obj->codeCounter;
                return;
        }

#ifdef  uG31xx_BOOT_LOADER

        /// [AT-PM] : In bootloader, ADC1 converstion time is not calculated ; 02/12/2013
        obj->info->lastCounter = obj->codeCounter;
        return;

#endif  ///< end of uG31xx_BOOT_LOADER

        /// [AT-PM] : Check counter overflow or time overflow; 01/25/2013
        if((obj->codeCounter <= obj->info->lastCounter) || (obj->info->deltaTime == 0)) {
                obj->info->lastCounter = obj->codeCounter;
                return;
        }

        /// [AT-PM] : Limit the minimum counter ; 02/11/2013
        tmp32 = (_meas_u32_)obj->codeCounter;
        tmp32 = tmp32 - obj->info->lastCounter;
        if(tmp32 < MINIMUM_ADC1_COUNTER_FOR_CONVERT_TIME) {
                obj->info->lastCounter = obj->codeCounter;
                return;
        }

        /// [AT-PM] : Calculate new ADC1 conversion time ; 09/25/2013
        tmp32 = obj->info->deltaTime;
        tmp32 = tmp32*TIME_CONVERT_TIME_TO_MSEC/(obj->codeCounter - obj->info->lastCounter);

        /// [AT-PM] : Check conversion time is valid or not ; 02/13/2013
        if((tmp32 > (MAXIMUM_ADC1_CONVERSION_TIME*TIME_CONVERT_TIME_TO_MSEC)) ||
            (tmp32 < (MINIMUM_ADC1_CONVERSION_TIME*TIME_CONVERT_TIME_TO_MSEC))) {
                UG31_LOGI("[%s]: ***************************************************************************************\n", __func__);
                UG31_LOGE("[%s]: ***************************************************************************************\n", __func__);
                UG31_LOGI("[%s]:  ####  #####  ##  ##  ####  #####  ##   ##  ####  ##       ###### ###### ##   ## ######\n", __func__);
                UG31_LOGI("[%s]: ##  ## ##  ## ### ## ##  ## ##  ## ### ### ##  ## ##       ######   ##   ### ### ##\n", __func__);
                UG31_LOGI("[%s]: ###### #####  ###### ##  ## #####  ####### ###### ##         ##     ##   ####### ###\n", __func__);
                UG31_LOGI("[%s]: ##  ## ##  ## ## ### ##  ## ##  ## ##   ## ##  ## ##         ##     ##   ##   ## ##\n", __func__);
                UG31_LOGI("[%s]: ##  ## #####  ##  ##  ####  ##  ## ##   ## ##  ## ######     ##   ###### ##   ## ######\n", __func__);
                UG31_LOGI("[%s]:\n", __func__);
                UG31_LOGI("[%s]: Previous Time Tag    = %d\n", __func__, (int)(obj->info->lastTimeTick - obj->info->deltaTime));
                UG31_LOGI("[%s]: Current Time Tag     = %d\n", __func__, (int)obj->info->lastTimeTick);
                UG31_LOGE("[%s]: Delta Time           = %d\n", __func__, (int)obj->info->deltaTime);
                UG31_LOGE("[%s]: Previous ADC Count   = %d\n", __func__, obj->info->lastCounter);
                UG31_LOGE("[%s]: Current ADC Count    = %d\n", __func__, obj->codeCounter);
                UG31_LOGI("[%s]: Delta ADC Count      = %d\n", __func__, obj->codeCounter - obj->info->lastCounter);
                UG31_LOGI("[%s]: Old ADC Convert Time = %d\n", __func__, obj->info->adc1ConvertTime);
                UG31_LOGI("[%s]: New ADC Convert Time = %d\n", __func__, (int)tmp32);
                UG31_LOGE("[%s]: ***************************************************************************************\n", __func__);
                UG31_LOGI("[%s]: ***************************************************************************************\n", __func__);
                tmp32 = (_meas_u32_)TIME_DEFAULT_ADC1_CONVERT_TIME;
        }

        /// [AT-PM] : ADC1 converstion time filter ; 09/25/2013
        if(tmp32 > obj->info->adc1ConvertTime) {
                if(tmp32 > ((_meas_u32_)(obj->info->adc1ConvertTime + TIME_DEFAULT_ADC1_CONVERT_TIME))) {
                        tmp32 = tmp32 + obj->info->adc1ConvertTime;
                        tmp32 = tmp32/2;
                        obj->info->adc1ConvertTime = (_meas_u16_)tmp32;
                } else {
                        obj->info->adc1ConvertTime = obj->info->adc1ConvertTime + 1;
                }
        } else if(tmp32 < obj->info->adc1ConvertTime) {
                if((tmp32 + TIME_DEFAULT_ADC1_CONVERT_TIME) < ((_meas_u32_)obj->info->adc1ConvertTime)) {
                        tmp32 = tmp32 + obj->info->adc1ConvertTime;
                        tmp32 = tmp32/2;
                        obj->info->adc1ConvertTime = (_meas_u16_)tmp32;
                } else {
                        obj->info->adc1ConvertTime = obj->info->adc1ConvertTime - 1;
                }
        } else {
                obj->info->adc1ConvertTime = (_meas_u16_)tmp32;
        }
        UG31_LOGI("[%s]: Conversion Time = %d ((%d - %d)/%d)\n", __func__,
                  obj->info->adc1ConvertTime, obj->codeCounter, obj->info->lastCounter, (int)obj->info->deltaTime);
        obj->info->lastCounter = obj->codeCounter;
}

#define COULOMB_COUNTER_LSB       (4096)

/**
 * @brief ConvertCharge
 *
 *  Convert code of charge
 *
 * @para  obj address of MeasDataInternalType
 * @return  _UPI_NULL_
 */
void ConvertCharge(MeasDataInternalType *obj)
{
        _meas_s16_ tmp16;
        _meas_s32_ tmp32;
        _meas_s64_ tmp64;

        /// [AT-PM] : Convert from calibrated ADC code ; 01/25/2013
        tmp16 = ADC1_IDEAL_CODE_DELTA;
        tmp32 = (_meas_s32_)obj->info->codeCharge;
        tmp32 = tmp32 - ADC1_IDEAL_CODE_100MV;
        while(1) {
                tmp64 = (_meas_s64_)tmp32;
                tmp64 = tmp64*ADC1_VOLTAGE_DELTA;
                if((tmp64 < 2147483647) && (tmp64 > -2147483647)) {
                        break;
                }
                tmp16 = tmp16/2;
                tmp32 = tmp32/2;
        }
        tmp32 = (_meas_s32_)tmp64;
        tmp32 = tmp32/tmp16;
        tmp32 = tmp32 + ADC1_VOLTAGE_100MV;
        tmp32 = tmp32/obj->info->sysData->ggbParameter->rSense;
        UG31_LOGN("[%s]: ((%d - %d) x %d / %d + %d) / %d = %d\n", __func__,
                  (int)obj->info->codeCharge, ADC1_IDEAL_CODE_100MV, ADC1_VOLTAGE_DELTA,
                  ADC1_IDEAL_CODE_DELTA, ADC1_VOLTAGE_100MV, obj->info->sysData->ggbParameter->rSense, (int)tmp32);
#ifdef	UPI_UBOOT_DEBUG_MSG
        printf("[ConvertCharge]: ((%d - %d) x %d / %d + %d) / %d = %d\n",
               obj->info->codeCharge, ADC1_IDEAL_CODE_100MV, ADC1_VOLTAGE_DELTA,
               ADC1_IDEAL_CODE_DELTA, ADC1_VOLTAGE_100MV, obj->info->sysData->ggbParameter->rSense, tmp32);
#endif	///< end of UPI_UBOOT_DEBUG_MSG
        /// [AT-PM] : Apply board factor ; 01/25/2013
        tmp32 = tmp32*BOARD_FACTOR_CONST/BoardFactor[GET_PRODUCT_TYPE(obj->info->sysData->ggbParameter->NacLmdAdjustCfg)].current_gain;
        UG31_LOGN("[%s]: Board Factor (%d/%d) -> %d\n", __func__,
                  BoardFactor[GET_PRODUCT_TYPE(obj->info->sysData->ggbParameter->NacLmdAdjustCfg)].current_gain,
                  BoardFactor[GET_PRODUCT_TYPE(obj->info->sysData->ggbParameter->NacLmdAdjustCfg)].current_offset, (int)tmp32);

        /// [AT-PM] : Apply calibration factor ; 01/25/2013
        if(tmp32 > 0) {
                tmp32 = tmp32*CALIBRATION_FACTOR_CONST/obj->info->sysData->ggbParameter->adc1_pgain;
        } else {
                tmp32 = tmp32*CALIBRATION_FACTOR_CONST/obj->info->sysData->ggbParameter->adc1_ngain;
        }
        UG31_LOGN("[%s]: Calibration Factor (%d|%d/%d) -> %d\n", __func__,
                  obj->info->sysData->ggbParameter->adc1_pgain, obj->info->sysData->ggbParameter->adc1_ngain,
                  obj->info->sysData->ggbParameter->adc1_pos_offset, (int)tmp32);
#ifdef	UPI_UBOOT_DEBUG_MSG
        printf("[ConvertCharge]: Calibration Factor (%d|%d/%d) -> %d\n",
               obj->info->sysData->ggbParameter->adc1_pgain, obj->info->sysData->ggbParameter->adc1_ngain,
               obj->info->sysData->ggbParameter->adc1_pos_offset, tmp32);
#endif	///< end of UPI_UBOOT_DEBUG_MSG

        /// [AT-PM] : Apply time information ; 01/25/2013
        CalculateAdc1ConvertTime(obj);
        tmp32 = tmp32*(obj->info->adc1ConvertTime)/TIME_MSEC_TO_SEC*COULOMB_COUNTER_LSB/TIME_SEC_TO_HOUR;
        tmp32 = tmp32/TIME_CONVERT_TIME_TO_MSEC;
        if(MEAS_REVERSE_CURRENT_DIRECTION(obj->info->status) == _UPI_TRUE_) {
                tmp32 = tmp32*(-1);
        }

        /// [AT-PM] : Update capacity information ; 01/25/2013
        obj->info->deltaCap = (_meas_s16_)tmp32;
        obj->info->stepCap = obj->info->deltaCap - obj->info->lastDeltaCap;
        obj->info->lastDeltaCap = obj->info->deltaCap;
        UG31_LOGI("[%s]: Capacity = %d (%d)\n", __func__, obj->info->deltaCap, obj->info->stepCap);
#ifdef	UPI_UBOOT_DEBUG_MSG
        printf("[ConvertCharge]: Capacity = %d (%d)\n", obj->info->deltaCap, obj->info->stepCap);
#endif	///< end of UPI_UBOOT_DEBUG_MSG
}

/**
 * @brief TimeTick
 *
 *  Get the time tick and calculate delta time
 *
 * @para  obj address of MeasDataInternalType
 * @return  _UPI_NULL_
 */
void TimeTick(MeasDataInternalType *obj)
{
        if(MEAS_IN_SUSPEND_MODE(obj->info->status) == _UPI_TRUE_) {
                /// [AT-PM] : Prevent adc conversion count overflow ; 06/11/2013
                if(obj->codeCounter < obj->info->lastCounter) {
                        obj->info->deltaTime = (_meas_u32_)obj->codeCounter;
                        UG31_LOGE("[%s]: Counter overflow in internal suspend mode, counter = %d\n", __func__,
                                  (int)obj->codeCounter);
                } else {
                        obj->info->deltaTime = (_meas_u32_)obj->codeCounter;
                        obj->info->deltaTime = obj->info->deltaTime - obj->info->lastCounter;
                }
                /// [AT-PM] : Use conversion count to estimate delta time ; 06/11/2013
                obj->info->deltaTime = obj->info->deltaTime*obj->info->adc1ConvertTime/TIME_CONVERT_TIME_TO_MSEC;
                UG31_LOGE("[%s]: In internal suspend mode, deltaTime = %d\n", __func__, (int)obj->info->deltaTime);
                return;
        }

        obj->currTime = GetTickCount();

        /// [AT-PM] : Prevent time tick overflow ; 01/25/2013
        if(obj->currTime <= obj->info->lastTimeTick) {
                UG31_LOGE("[%s]: OVERFLOW -> %d < %d\n", __func__,
                          (int)obj->currTime, (int)obj->info->lastTimeTick);
                obj->info->deltaTime = obj->currTime;
                obj->info->lastTimeTick = obj->currTime;
                return;
        }

        /// [AT-PM] : Calculate delta time ; 01/25/2013
        obj->info->deltaTime = obj->currTime - obj->info->lastTimeTick;
        UG31_LOGN("[%s]: Delta Time = %d - %d = %d\n", __func__,
                  (int)obj->currTime, (int)obj->info->lastTimeTick, (int)obj->info->deltaTime);
        obj->info->lastTimeTick = obj->currTime;
}

/**
 * @brief ReadRegister
 *
 *  Read measurement registers
 *
 * @para  obj address of MeasDataInternalType
 * @return  _UPI_NULL_
 */
void ReadRegister(MeasDataInternalType *obj)
{
        /// [AT-PM] : Read VBat1Ave ; 01/27/2013
        API_I2C_Read(NORMAL_REGISTER,
                     UG31XX_I2C_HIGH_SPEED_MODE,
                     UG31XX_I2C_TEM_BITS_MODE,
                     REG_AVE_VBAT1_LOW,
                     REG_AVE_VBAT1_HIGH - REG_AVE_VBAT1_LOW + 1,
                     (unsigned char *)&obj->codeBat1);

        /// [AT-PM] : Read CurrentAve ; 01/27/2013
        API_I2C_Read(NORMAL_REGISTER,
                     UG31XX_I2C_HIGH_SPEED_MODE,
                     UG31XX_I2C_TEM_BITS_MODE,
                     REG_AVE_CURRENT_LOW,
                     REG_AVE_CURRENT_HIGH - REG_AVE_CURRENT_LOW + 1,
                     (unsigned char *)&obj->codeCurrent);

        /// [AT-PM] : Read ITAve ; 01/27/2013
        API_I2C_Read(NORMAL_REGISTER,
                     UG31XX_I2C_HIGH_SPEED_MODE,
                     UG31XX_I2C_TEM_BITS_MODE,
                     REG_AVE_IT_LOW,
                     REG_AVE_IT_HIGH - REG_AVE_IT_LOW + 1,
                     (unsigned char *)&obj->codeIntTemperature);

        /// [AT-PM] : Read ETAve ; 01/27/2013
        API_I2C_Read(NORMAL_REGISTER,
                     UG31XX_I2C_HIGH_SPEED_MODE,
                     UG31XX_I2C_TEM_BITS_MODE,
                     REG_AVE_ET_LOW,
                     REG_AVE_ET_HIGH - REG_AVE_ET_LOW + 1,
                     (unsigned char *)&obj->codeExtTemperature);

        /// [AT-PM] : Read Charge ; 01/27/2013
        API_I2C_Read(NORMAL_REGISTER,
                     UG31XX_I2C_HIGH_SPEED_MODE,
                     UG31XX_I2C_TEM_BITS_MODE,
                     REG_CHARGE_LOW,
                     REG_CHARGE_HIGH - REG_CHARGE_LOW + 1,
                     (unsigned char *)&obj->codeCharge);

        /// [AT-PM] : Read Counter ; 01/27/2013
        API_I2C_Read(NORMAL_REGISTER,
                     UG31XX_I2C_HIGH_SPEED_MODE,
                     UG31XX_I2C_TEM_BITS_MODE,
                     REG_COUNTER_LOW,
                     REG_COUNTER_HIGH - REG_COUNTER_LOW + 1,
                     (unsigned char *)&obj->codeCounter);

        /// [AT-PM] : Read Offset ; 01/27/2013
        API_I2C_Read(NORMAL_REGISTER,
                     UG31XX_I2C_HIGH_SPEED_MODE,
                     UG31XX_I2C_TEM_BITS_MODE,
                     REG_ADC1_OFFSET_LOW,
                     REG_ADC1_OFFSET_HIGH - REG_ADC1_OFFSET_LOW + 1,
                     (unsigned char *)&obj->ccOffset);
}

/**
 * @brief ResetCoulombCounter
 *
 *  Reset coulomb counter
 *
 * @para  obj address of MeasDataInternalType
 * @return  _UPI_NULL_
 */
void ResetCoulombCounter(MeasDataInternalType *obj)
{
        _meas_u8_ tmp8;

        tmp8 = 0;
        API_I2C_Read(NORMAL_REGISTER,
                     UG31XX_I2C_HIGH_SPEED_MODE,
                     UG31XX_I2C_TEM_BITS_MODE,
                     REG_CTRL1,
                     1,
                     &tmp8);
        tmp8 = tmp8 | CTRL1_GG_RST;
        API_I2C_Write(NORMAL_REGISTER,
                      UG31XX_I2C_HIGH_SPEED_MODE,
                      UG31XX_I2C_TEM_BITS_MODE,
                      REG_CTRL1,
                      1,
                      &tmp8);
}

/**
 * @brief RevertCalibrateAdc2Code
 *
 *  Revert calibrated ADC2 code
 *
 * @para  data  address of MeasDataType
 * @para  caliCode  calibrated ADC2 code
 * @return  raw ADC2 code
 */
_meas_s32_ RevertCalibrateAdc2Code(MeasDataType *data, _meas_s32_ caliCode)
{
        _meas_s64_ tmp64;
        _meas_s32_ tmp32;
        _meas_s32_ deltaIT;
        _meas_s32_ gain;
        _meas_s32_ offset;
        _meas_s32_ constant;

        /// [AT-PM] : tmp32 = ( caliCode x gain / constant + offset ) / deltaIT ; 04/08/2013
        gain = data->adc2Gain;
        offset = data->adc2Offset;
        deltaIT = (_meas_s32_)data->otp->aveIT80;
        deltaIT = deltaIT - data->otp->aveIT25;
        constant = ADC2_IDEAL_CODE_DELTA;
        while(1) {
                tmp64 = (_meas_s64_)caliCode;
                tmp64 = tmp64*gain;
                if((tmp64 < 2147483647) && (tmp64 > -2147483647)) {
                        break;
                }
                caliCode = caliCode/2;
                gain = gain/2;
                constant = constant/4;
        }
        tmp32 = (_meas_s32_)tmp64;
        tmp32 = tmp32/constant;
        tmp32 = tmp32 + offset;
        tmp32 = tmp32/deltaIT;
        return (tmp32);
}

/**
 * @brief RevertBat1Code
 *
 *  Revert VBat1 code
 *
 * @para  data  address of MeasDataType
 * @para  volt  voltage in mV to be reverted
 * @return  adc2 vbat1 code
 */
_meas_u16_ RevertBat1Code(MeasDataType *data, _upi_s16_ volt)
{
        _meas_s32_ tmp32;

        tmp32 = (_meas_s32_)volt;

        /// [AT-PM] : Revert calibration parameter ; 04/08/2013
        tmp32 = tmp32*data->sysData->ggbParameter->adc2_gain/CALIBRATION_FACTOR_CONST;
        tmp32 = tmp32 + data->sysData->ggbParameter->adc2_offset;

        /// [AT-PM] : Revert board factor ; 04/08/2013
        tmp32 = tmp32*BoardFactor[GET_PRODUCT_TYPE(data->sysData->ggbParameter->NacLmdAdjustCfg)].voltage_gain/BOARD_FACTOR_CONST;
        tmp32 = tmp32 + BoardFactor[GET_PRODUCT_TYPE(data->sysData->ggbParameter->NacLmdAdjustCfg)].voltage_offset;

        /// [AT-PM] : Revert to calibrated ADC code ; 04/08/2013
        tmp32 = tmp32 - ADC2_VOLTAGE_100MV;
        tmp32 = tmp32*ADC2_IDEAL_CODE_DELTA/ADC2_VOLTAGE_DELTA;
        tmp32 = tmp32 + ADC2_IDEAL_CODE_100MV;

        /// [AT-PM] : Revert to raw code ; 04/08/2013
        tmp32 = RevertCalibrateAdc2Code(data, tmp32);
        return ((_meas_u16_)tmp32);
}

/**
 * @brief RevertETCode
 *
 *  Revert ET code
 *
 * @para  data  address of MeasDataType
 * @para  et  external temperature in 0.1oC to be reverted
 * @return  adc1 et code
 */
_meas_u16_ RevertETCode(MeasDataType *data, _upi_s16_ et)
{
        _meas_s32_ tmp32;
        _meas_u8_ idx;

        tmp32 = (_meas_s32_)et;

        /// [AT-PM] : Revert calibration factor ; 04/08/2013
        tmp32 = tmp32 + data->sysData->ggbParameter->adc_d4;

        /// [AT-PM] : Revert board factor ; 04/08/2013
        tmp32 = tmp32 + BoardFactor[GET_PRODUCT_TYPE(data->sysData->ggbParameter->NacLmdAdjustCfg)].ext_temp_offset;

        /// [AT-PM] : Revert external temperature calculation ; 04/08/2013
        idx = 0;
        while(idx < ET_NUMS) {
                if(tmp32 < ExtTemperatureTable[idx]) {
                        break;
                }
                idx = idx + 1;
        }
        if(idx == 0) {
                tmp32 = (_meas_s32_)data->sysData->ggbParameter->rtTable[0];
        } else if(idx >= ET_NUMS) {
                tmp32 = (_meas_s32_)data->sysData->ggbParameter->rtTable[ET_NUMS - 1];
        } else {
                tmp32 = tmp32 - ExtTemperatureTable[idx - 1];
                tmp32 = tmp32*(data->sysData->ggbParameter->rtTable[idx] - data->sysData->ggbParameter->rtTable[idx - 1]);
                tmp32 = tmp32/(ExtTemperatureTable[idx] - ExtTemperatureTable[idx - 1]);
                tmp32 = tmp32 + data->sysData->ggbParameter->rtTable[idx - 1];
        }
        return ((_meas_u16_)tmp32);
}

#define MINIMUM_VBAT1_CODE            (ADC2_IDEAL_CODE_100MV/2)
#define MAXIMUM_CURRENT_CODE          (ADC1_IDEAL_CODE_200MV*6)
#define MINIMUM_CURRENT_CODE          (ADC1_IDEAL_CODE_200MV*(-6))
#define MINIMUM_IT_CODE               (IT_IDEAL_CODE_25/2)
#define MAXIMUM_IT_CODE               (IT_IDEAL_CODE_80*11/10)
#define MINIMUM_ET_CODE               (1000)
#define MAXIMUM_ET_CODE               (28000)

typedef MEAS_RTN_CODE (*CheckAdcCode)(MeasDataInternalType *obj);

/**
 * @brief CheckCurrentMin
 *
 *  Check minimum current ADC code
 *
 * @para  obj address of MeasDataInternalType
 * @return  MEAS_RTN_CODE
 */
MEAS_RTN_CODE CheckCurrentMin(MeasDataInternalType *obj)
{
        /// [FC] : Add variable MPK_active for MPK intial ; 12/10/2013
        if(MPK_active == _UPI_FALSE_) {
                if(obj->codeCurrent < MINIMUM_CURRENT_CODE) {
                        UG31_LOGE("[%s]: Current code %d < %d\n", __func__,
                                  obj->codeCurrent, MINIMUM_CURRENT_CODE);
                        obj->codeCurrent = MINIMUM_CURRENT_CODE;
                }
        }
        return (MEAS_RTN_PASS);
}

/**
 * @brief CheckCurrentMax
 *
 *  Check maximum current ADC code
 *
 * @para  obj address of MeasDataInternalType
 * @return  MEAS_RTN_CODE
 */
MEAS_RTN_CODE CheckCurrentMax(MeasDataInternalType *obj)
{
        /// [FC] : Add variable MPK_active for MPK intial ; 12/10/2013
        if(MPK_active == _UPI_FALSE_) {
                if(obj->codeCurrent > MAXIMUM_CURRENT_CODE) {
                        UG31_LOGE("[%s]: Current code %d > %d\n", __func__,
                                  obj->codeCurrent, MAXIMUM_CURRENT_CODE);
                        obj->codeCurrent = MAXIMUM_CURRENT_CODE;
                }
        }
        return (MEAS_RTN_PASS);
}

/**
 * @brief CheckITMin
 *
 *  Check minimum internal temperature ADC code
 *
 * @para  obj address of MeasDataInternalType
 * @return  MEAS_RTN_CODE
 */
MEAS_RTN_CODE CheckITMin(MeasDataInternalType *obj)
{
        /// [FC] : Add variable MPK_active for MPK intial ; 12/10/2013
        if(MPK_active == _UPI_FALSE_) {
                if(obj->codeIntTemperature < MINIMUM_IT_CODE) {
                        UG31_LOGE("[%s]: Internal Temperature code %d < %d\n", __func__,
                                  obj->codeIntTemperature, MINIMUM_IT_CODE);
                        return (MEAS_RTN_ADC_ABNORMAL);
                }
        }
        return (MEAS_RTN_PASS);
}

/**
 * @brief CheckITMax
 *
 *  Check maximum internal temperature ADC code
 *
 * @para  obj address of MeasDataInternalType
 * @return  MEAS_RTN_CODE
 */
MEAS_RTN_CODE CheckITMax(MeasDataInternalType *obj)
{
        /// [FC] : Add variable MPK_active for MPK intial ; 12/10/2013
        if(MPK_active == _UPI_FALSE_) {
                if(obj->codeIntTemperature > MAXIMUM_IT_CODE) {
                        UG31_LOGE("[%s]: Internal Temperature code %d > %d\n", __func__,
                                  obj->codeIntTemperature, MAXIMUM_IT_CODE);
                        return (MEAS_RTN_ADC_ABNORMAL);
                }
        }
        return (MEAS_RTN_PASS);
}

/**
 * @brief CheckETMin
 *
 *  Check minimum external temperature ADC code
 *
 * @para  obj address of MeasDataInternalType
 * @return  MEAS_RTN_CODE
 */
MEAS_RTN_CODE CheckETMin(MeasDataInternalType *obj)
{
        /// [FC] : Add variable MPK_active for MPK intial ; 12/10/2013
        if(MPK_active == _UPI_FALSE_) {
                if(obj->codeExtTemperature < MINIMUM_ET_CODE) {
                        UG31_LOGE("[%s]: External Temperature code %d < %d\n", __func__,
                                  obj->codeExtTemperature, MINIMUM_ET_CODE);
                        obj->info->status = obj->info->status | MEAS_STATUS_NTC_SHORT;
                } else {
                        obj->info->status = obj->info->status & (~MEAS_STATUS_NTC_SHORT);
                }
        }
        return (MEAS_RTN_PASS);
}

/**
 * @brief CheckETMax
 *
 *  Check maximum external temperature ADC code
 *
 * @para  obj address of MeasDataInternalType
 * @return  MEAS_RTN_CODE
 */
MEAS_RTN_CODE CheckETMax(MeasDataInternalType *obj)
{
        /// [FC] : Add variable MPK_active for MPK intial ; 12/10/2013
        if(MPK_active == _UPI_FALSE_) {
                if(obj->codeExtTemperature > MAXIMUM_ET_CODE) {
                        UG31_LOGE("[%s]: External Temperature code %d > %d\n", __func__,
                                  obj->codeExtTemperature, MAXIMUM_ET_CODE);
                        obj->info->status = obj->info->status | MEAS_STATUS_NTC_OPEN;
                } else {
                        obj->info->status = obj->info->status & (~MEAS_STATUS_NTC_OPEN);
                }
        }
        return (MEAS_RTN_PASS);
}

/**
 * @brief CheckVBat1Min
 *
 *  Check minimum VBat1 ADC code
 *
 * @para  obj address of MeasDataInternalType
 * @return  MEAS_RTN_CODE
 */
MEAS_RTN_CODE CheckVBat1Min(MeasDataInternalType *obj)
{
        /// [FC] : Add variable MPK_active for MPK intial ; 12/10/2013
        if(MPK_active == _UPI_FALSE_) {
                if(obj->codeBat1 < MINIMUM_VBAT1_CODE) {
                        UG31_LOGE("[%s]: Voltage code %d < %d\n", __func__,
                                  obj->codeBat1, MINIMUM_VBAT1_CODE);
                        return (MEAS_RTN_BATTERY_REMOVED);
                }
        }
        return (MEAS_RTN_PASS);
}

static CheckAdcCode CheckAdcCodeRoutine[] = {
        CheckCurrentMin,
        CheckCurrentMax,
        CheckITMin,
        CheckITMax,
#ifdef  ENABLE_NTC_CHECK
        CheckETMin,
        CheckETMax,
#endif  ///< end of ENABLE_NTC_CHECK
        CheckVBat1Min,
        _UPI_NULL_,
};

/**
 * @brief FetchAdcCode
 *
 *  Fetch ADC converted code
 *
 * @para  obj address of MeasDataInternalType
 * @return  MEAS_RTN_CODE
 */
MEAS_RTN_CODE FetchAdcCode(MeasDataInternalType *obj)
{
        _meas_u8_ retry;
        MEAS_RTN_CODE rtn;
        _meas_u8_ idx;

        /// [AT-PM] : Read ADC code ; 01/27/2013
        retry = 0;
        while(retry <= obj->info->fetchRetryCnt) {
                ReadRegister(obj);
                UG31_LOGI("[%s]: Retry = %d (%d)\n", __func__, retry, obj->info->fetchRetryCnt);
                UG31_LOGI("[%s]: Voltage code = %d\n", __func__, obj->codeBat1);
                UG31_LOGI("[%s]: Current code = %d\n", __func__, obj->codeCurrent);
                UG31_LOGI("[%s]: Internal Temperature code = %d\n", __func__, obj->codeIntTemperature);
                UG31_LOGI("[%s]: External Temperature code = %d\n", __func__, obj->codeExtTemperature);
                UG31_LOGI("[%s]: Charge code = %d\n", __func__, obj->codeCharge);
                UG31_LOGI("[%s]: Counter = %d\n", __func__, obj->codeCounter);

                idx = 0;
                while(CheckAdcCodeRoutine[idx] != _UPI_NULL_) {
                        rtn = CheckAdcCodeRoutine[idx](obj);
                        if(rtn != MEAS_RTN_PASS) {
                                SleepMiniSecond(125);
                                break;
                        }
                        idx = idx + 1;
                }

                if(rtn == MEAS_RTN_PASS) {
                        if(retry != 0) {
                                SleepMiniSecond(250);

                                ReadRegister(obj);
                                UG31_LOGI("[%s]: Retry = %d (%d)\n", __func__, retry, obj->info->fetchRetryCnt);
                                UG31_LOGI("[%s]: Voltage code = %d\n", __func__, obj->codeBat1);
                                UG31_LOGI("[%s]: Current code = %d\n", __func__, obj->codeCurrent);
                                UG31_LOGI("[%s]: Internal Temperature code = %d\n", __func__, obj->codeIntTemperature);
                                UG31_LOGI("[%s]: External Temperature code = %d\n", __func__, obj->codeExtTemperature);
                                UG31_LOGI("[%s]: Charge code = %d\n", __func__, obj->codeCharge);
                                UG31_LOGI("[%s]: Counter = %d\n", __func__, obj->codeCounter);
                        }
                        break;
                }

                rtn = MEAS_RTN_ADC_ABNORMAL;
                retry = retry + 1;
        }

        obj->info->codeBat1BeforeCal = obj->codeBat1;
        obj->info->codeCurrentBeforeCal = obj->codeCurrent;
        obj->info->codeIntTemperatureBeforeCal = obj->codeIntTemperature;
        obj->info->codeExtTemperatureBeforeCal = obj->codeExtTemperature;
        obj->info->codeChargeBeforeCal = obj->codeCharge;
        obj->info->codeCCOffset = obj->ccOffset;
        return (rtn);
}

#define MAX_ET_CODE_DIFF  (200)
#define MIN_ET_CODE_DIFF  (-200)

/**
 * @brief CalibrateETCode
 *
 *  Calibrate external temperature code
 *
 * @para  obj address of MeasDataInternalType
 * @return  MEAS_RTN_CODE
 */
_meas_u16_ CalibrateETCode(MeasDataInternalType *obj)
{
        _meas_s16_ tmp16;
        _meas_s32_ tmp32;

        /// [AT-PM] : Get compensation of board and r-sense ; 11/01/2013
        tmp32 = (_meas_s32_)obj->codeCurrent;
        tmp32 = tmp32 + obj->ccOffset;
        tmp32 = tmp32*(obj->info->sysData->ggbParameter->rSense + obj->info->sysData->ggbParameter->offsetR)/obj->info->sysData->ggbParameter->rSense;
        obj->codeExtTemperatureComp = (_meas_s16_)tmp32;
        UG31_LOGN("[%s]: Compensation = %d\n", __func__, (int)tmp32);

        /// [AT-PM] : Compensate temperature code ; 11/01/2013
        tmp16 = (_meas_s16_)obj->codeExtTemperature;
        if(MEAS_REVERSE_CURRENT_DIRECTION(obj->info->status) == _UPI_FALSE_) {
                tmp16 = tmp16 - obj->codeExtTemperatureComp;
        }
        obj->info->codeInstExtTemperature = (_meas_u16_)tmp16;
        UG31_LOGN("[%s]: Calibrated ET Code = %d - %d = %d\n", __func__, obj->codeExtTemperature, obj->codeExtTemperatureComp, tmp16);
        if(obj->info->codeExtTemperature != 0) {
#ifdef	UPI_UBOOT_DEBUG_MSG
                printf("[CalibrateETCode]: Last = %d, Current = %d\n", obj->info->codeExtTemperature, tmp16);
#endif	///< end of UPI_UBOOT_DEBUG_MSG
                tmp16 = tmp16 - obj->info->codeExtTemperature;
                UG31_LOGN("[%s]: Previous ET Code = %d\n", __func__, obj->info->codeExtTemperature);
                if(tmp16 > MAX_ET_CODE_DIFF) {
                        UG31_LOGE("[%s]: Exceed maximum ET code difference (%d > %d)\n", __func__, tmp16, MAX_ET_CODE_DIFF);
                        tmp16 = MAX_ET_CODE_DIFF;
                }
                if(tmp16 < MIN_ET_CODE_DIFF) {
                        UG31_LOGE("[%s]: Exceed minimum ET code difference (%d < %d)\n", __func__, tmp16, MIN_ET_CODE_DIFF);
                        tmp16 = MIN_ET_CODE_DIFF;
                }
                tmp16 = tmp16 + obj->info->codeExtTemperature;
        }
#ifdef	UPI_UBOOT_DEBUG_MSG
        printf("[CalibrateETCode]: ET code = %d\n", tmp16);
#endif	///< end of UPI_UBOOT_DEBUG_MSG
        return ((_meas_u16_)tmp16);
}

/**
 * @brief RevertCurrent
 *
 *  Revert current to adc code
 *
 * @para  obj address of MeasDataInternalType
 * @para  curr  current to be reverted
 * @return  adc code
 */
_meas_s16_ RevertCurrent(MeasDataInternalType *obj, _meas_s32_ curr)
{
        _meas_s32_ tmp32;

        tmp32 = curr*(obj->info->sysData->ggbParameter->rSense) - ADC1_VOLTAGE_100MV;
        tmp32 = tmp32*ADC1_IDEAL_CODE_DELTA/ADC1_VOLTAGE_DELTA;
        tmp32 = tmp32 + ADC1_IDEAL_CODE_100MV;
        return ((_meas_s16_)tmp32);
}

/**
 * @brief CalculateCCOffset
 *
 *  Calculate coulomb counter offset
 *
 * @para  obj address of MeasDataInternalType
 * @return  NULL
 */
void CalculateCCOffset(MeasDataInternalType *obj)
{
        _meas_s32_ offset;
        _meas_s32_ tmp32;

        /// [AT-PM] : Offset from FT ; 09/24/2013
        offset = obj->info->adc1Offset/(obj->info->otp->aveIT80 - obj->info->otp->aveIT25);
        UG31_LOGN("[%s]: Offset from FT = %d\n", __func__, (int)offset);

        /// [AT-PM] : Offset from board factor and calibration factor ; 09/24/2013
        tmp32 = (_meas_s32_)obj->info->sysData->ggbParameter->adc1_pos_offset;
        tmp32 = tmp32 + obj->info->ccOffsetAdj;
        tmp32 = tmp32*BoardFactor[GET_PRODUCT_TYPE(obj->info->sysData->ggbParameter->NacLmdAdjustCfg)].current_gain/BOARD_FACTOR_CONST;
        tmp32 = tmp32 + BoardFactor[GET_PRODUCT_TYPE(obj->info->sysData->ggbParameter->NacLmdAdjustCfg)].current_offset;
        UG31_LOGN("[%s]: Offset from board and calibration factor = %d x %d / %d + %d = %d\n", __func__,
                  obj->info->sysData->ggbParameter->adc1_pos_offset,
                  BoardFactor[GET_PRODUCT_TYPE(obj->info->sysData->ggbParameter->NacLmdAdjustCfg)].current_gain,
                  BOARD_FACTOR_CONST,
                  BoardFactor[GET_PRODUCT_TYPE(obj->info->sysData->ggbParameter->NacLmdAdjustCfg)].current_offset,
                  (int)tmp32);

        /// [AT-PM] : Revert board factor and calibration factor  09/24/2013
        tmp32 = RevertCurrent(obj, tmp32);
        UG31_LOGI("[%s]: Revert offset to adc code from current = %d\n", __func__, (int)tmp32);

        /// [AT-PM] : Update offset ; 09/24/2013
        offset = (offset - tmp32)*(-1);
        obj->info->ccOffset = (_meas_s16_)offset;

        /// [AT-PM] : Set coulomb counter offset ; 01/27/2013
        API_I2C_Write(NORMAL_REGISTER,
                      UG31XX_I2C_HIGH_SPEED_MODE,
                      UG31XX_I2C_TEM_BITS_MODE,
                      REG_ADC1_OFFSET_LOW,
                      REG_ADC1_OFFSET_HIGH - REG_ADC1_OFFSET_LOW + 1,
                      (unsigned char *)&obj->info->ccOffset);
}

/**
 * @brief CountCycleCount
 *
 *  Count cycle count of battery discharged capacity
 *
 * @para  obj address of MeasDataInternalType
 * @return  NULL
 */
void CountCycleCount(MeasDataInternalType *obj)
{
        if(obj->info->curr >= ((obj->info->sysData->ggbParameter->standbyCurrent)*(-1))) {
                UG31_LOGI("[%s]: No cycle count calculation because no discharging capacity measured.\n", __func__);
                return;
        }

        obj->info->cycleCountBuf = obj->info->cycleCountBuf - obj->info->stepCap;

        if(obj->info->sysData->ggbParameter->CycleCountThrd == 0) {
                obj->info->sysData->ggbParameter->CycleCountThrd = obj->info->sysData->ggbParameter->ILMD;
        }
        if(obj->info->cycleCountBuf >= obj->info->sysData->ggbParameter->CycleCountThrd) {
                obj->info->cycleCount = obj->info->cycleCount + 1;
                obj->info->cycleCountBuf = obj->info->cycleCountBuf - obj->info->sysData->ggbParameter->CycleCountThrd;
                UG31_LOGI("[%s]: Update cycle count = %d (%d >= %d)\n", __func__,
                          obj->info->cycleCount, (int)obj->info->cycleCountBuf, obj->info->sysData->ggbParameter->CycleCountThrd);
        }
}

/**
 * @brief CountCumuCap
 *
 *  Count cumulative capacity
 *
 * @para  obj address of MeasDataInternalType
 * @return  NULL
 */
void CountCumuCap(MeasDataInternalType *obj)
{
        obj->info->cumuCap = obj->info->cumuCap + obj->info->stepCap;
        UG31_LOGI("[%s]: Cumulative capacity = %d\n", __func__,
                  obj->info->cumuCap);
}

/// =============================================
/// [AT-PM] : Extern function region
/// =============================================

/**
 * @brief UpiResetCoulombCounter
 *
 *  Reset coulomb counter
 *
 * @para  data  address of MeasDataType
 * @return  _UPI_NULL_
 */
void UpiResetCoulombCounter(MeasDataType *data)
{
        MeasDataInternalType *obj;

#ifdef  UG31XX_SHELL_ALGORITHM
        obj = (MeasDataInternalType *)upi_malloc(sizeof(MeasDataInternalType));
#else   ///< else of UG31XX_SHELL_ALGORITHM
        obj = &measData;
#endif  ///< end of UG31XX_SHELL_ALGORITHM
        upi_memset(obj, 0x00, sizeof(MeasDataInternalType));

        obj->info = data;
        obj->info->fetchRetryCnt = MEAS_MAXIMUM_ROUTINE_RETRY_CNT;

        /// [AT-PM] : Read ADC code ; 01/27/2013
        FetchAdcCode(obj);

        /// [AT-PM] : Get delta time ; 01/25/2013
        TimeTick(obj);
        obj->info->deltaTimeDaemon = obj->info->deltaTimeDaemon + obj->info->deltaTime;

        /// [AT-PM] : Reset coulomb counter ; 01/30/2013
        ResetCoulombCounter(obj);

        /// [AT-PM] : Convert ADC characteristic from OTP ; 01/23/2013
        ConvertAdc1Data(obj);

        /// [AT-PM] : Calculate ADC gain and offset ; 01/23/2013
        CalAdc1Factors(obj);

        /// [AT-PM] : Calibrate ADC code ; 01/23/2013
        CalibrateChargeCode(obj);

        /// [AT-PM] : Convert into physical value ; 01/23/2013
        ConvertCharge(obj);

        /// [AT-PM] : Count cycle count ; 10/08/2013
        CountCycleCount(obj);

        /// [AT-PM] : Count cumulative capacity ; 02/18/2014
        CountCumuCap(obj);

        /// [AT-PM] : Calculate coulomb counter offset ; 09/24/2013
        CalculateCCOffset(obj);

        data->lastDeltaCap = 0;

        /// [AT-PM] : Read ADC code ; 01/27/2013
        FetchAdcCode(obj);
        obj->info->lastCounter = obj->codeCounter;

#ifdef  UG31XX_SHELL_ALGORITHM
        upi_free(obj);
#endif  ///< end of UG31XX_SHELL_ALGORITHM
}

#define RESET_CC_CURRENT_MAGIC_NUMBER               (2)
#define RESET_CC_DELTA_TIME                         (TIME_SEC_TO_HOUR*TIME_MSEC_TO_SEC)
#define COULOMB_COUNTER_RESET_THRESHOLD_CHARGE_CHG  (30000)
#define COULOMB_COUNTER_RESET_THREDHOLD_CHARGE_DSG  (-30000)

/**
 * @brief UpiMeasurement
 *
 *  Measurement routine
 *
 * @para  data  address of MeasDataType
 * @para  select  MEAS_SEL_CODE
 * @return  MEAS_RTN_CODE
 */
MEAS_RTN_CODE UpiMeasurement(MeasDataType *data, MEAS_SEL_CODE select)
{
        MeasDataInternalType *obj;
        MEAS_RTN_CODE rtn;
        _meas_s16_ standbyUpper;
        _meas_s16_ standbyLower;
        _meas_u16_ tmp16;
        _meas_s32_ tmp32;

        UG31_LOGI("[%s]: %s\n", __func__, MEASUREMENT_VERSION);

#ifdef  UG31XX_SHELL_ALGORITHM
        obj = (MeasDataInternalType *)upi_malloc(sizeof(MeasDataInternalType));
#else   ///< else of UG31XX_SHELL_ALGORITHM
        obj = &measData;
#endif  ///< end of UG31XX_SHELL_ALGORITHM
        upi_memset(obj, 0x00, sizeof(MeasDataInternalType));

        obj->info = data;
        obj->info->fetchRetryCnt = MEAS_MAXIMUM_ROUTINE_RETRY_CNT;

        rtn = MEAS_RTN_PASS;

        /// [AT-PM] : Get ADC code ; 06/04/2013
        rtn = FetchAdcCode(obj);
        UG31_LOGE("[%s]: (%d-%d) V=%d, I=%d, IT=%d, ET=%d, CH=%d, CT=%d\n", __func__,
                  select, obj->info->fetchRetryCnt, obj->codeBat1, obj->codeCurrent,
                  obj->codeIntTemperature, obj->codeExtTemperature, obj->codeCharge, obj->codeCounter);
        if(rtn != MEAS_RTN_PASS) {
#ifdef  UG31XX_SHELL_ALGORITHM
                upi_free(obj);
#endif  ///< end of UG31XX_SHELL_ALGORITHM
                return (rtn);
        }

        /// [AT-PM] : Get delta time ; 01/25/2013
        if((select == MEAS_SEL_ALL) || (select == MEAS_SEL_INITIAL)) {
                TimeTick(obj);
                obj->info->deltaTimeDaemon = obj->info->deltaTimeDaemon + obj->info->deltaTime;
        }

        if((select == MEAS_SEL_ALL) || (select == MEAS_SEL_CURRENT) || (select == MEAS_SEL_INITIAL)) {
                /// [AT-PM] : Convert ADC characteristic from OTP ; 01/23/2013
                ConvertAdc1Data(obj);

                /// [FC] : Record ADC code ; 05/15/2013
                data->adc1CodeT25V100 = obj->adc1CodeT25V100;
                data->adc1CodeT25V200 = obj->adc1CodeT25V200;

                /// [AT-PM] : Calculate ADC gain and offset ; 01/23/2013
                CalAdc1Factors(obj);
        }
        if((select == MEAS_SEL_ALL) || (select == MEAS_SEL_VOLTAGE) || (select == MEAS_SEL_INITIAL)) {
                /// [AT-PM] : Convert ADC characteristic from OTP ; 01/23/2013
                ConvertAdc2Data(obj);

                /// [FC] : Record ADC code ; 05/15/2013
                data->adc2CodeT25V100 = obj->adc2CodeT25V100;
                data->adc2CodeT25V200 = obj->adc2CodeT25V200;

                /// [AT-PM] : Calculate ADC gain and offset ; 01/23/2013
                CalAdc2Factors(obj);
        }

        /// [AT-PM] : Calibrate ADC code ; 01/23/2013
        if((select == MEAS_SEL_ALL) || (select == MEAS_SEL_VOLTAGE) || (select == MEAS_SEL_INITIAL)) {
                data->codeBat1 = (_meas_u16_)CalibrateAdc2Code(obj, (_meas_s32_)obj->codeBat1);
                UG31_LOGN("[%s]: VBat1 Code = %d -> %d\n", __func__, obj->codeBat1, data->codeBat1);
        }
        if((select == MEAS_SEL_ALL) || (select == MEAS_SEL_CURRENT) || (select == MEAS_SEL_INITIAL)) {
                data->codeCurrent = (_meas_s16_)CalibrateAdc1Code(obj, (_meas_s32_)obj->codeCurrent);
                UG31_LOGN("[%s]: Current Code = %d -> %d\n", __func__, obj->codeCurrent, data->codeCurrent);
        }
        if((select == MEAS_SEL_ALL) || (select == MEAS_SEL_INITIAL)) {
                CalibrateChargeCode(obj);
        }
        if((select == MEAS_SEL_ALL) || (select == MEAS_SEL_INT_TEMP) || (select == MEAS_SEL_INITIAL)) {
                data->codeIntTemperature = CalibrateITCode(obj, obj->codeIntTemperature);
                UG31_LOGN("[%s]: Internal Temperature Code = %d -> %d\n", __func__,
                          obj->codeIntTemperature, data->codeIntTemperature);
        }
        if((select == MEAS_SEL_ALL) || (select == MEAS_SEL_EXT_TEMP) || (select == MEAS_SEL_INITIAL)) {
                data->codeExtTemperature = CalibrateETCode(obj);
                UG31_LOGN("[%s]: External Temperature Code = %d -> %d\n", __func__, obj->codeExtTemperature, data->codeExtTemperature);
        }

        /// [AT-PM] : Convert into physical value ; 01/23/2013
        if((select == MEAS_SEL_ALL) || (select == MEAS_SEL_VOLTAGE) || (select == MEAS_SEL_INITIAL)) {
                ConvertBat1(obj);

                if(select == MEAS_SEL_INITIAL) {
                        obj->info->bat1VoltageAvg = obj->info->bat1Voltage;
                } else {
                        tmp32 = (_meas_s32_)obj->info->bat1Voltage;
                        tmp32 = tmp32 + obj->info->bat1VoltageAvg;
                        tmp32 = tmp32/2;
                        obj->info->bat1VoltageAvg = (_meas_u16_)tmp32;
                        UG31_LOGN("[%s]: Average voltage = %d (%d)\n", __func__,
                                  obj->info->bat1VoltageAvg,
                                  obj->info->bat1Voltage);
                }
        }
        if((select == MEAS_SEL_ALL) || (select == MEAS_SEL_CURRENT) || (select == MEAS_SEL_INITIAL)) {
                ConvertCurrent(obj);

                if(select == MEAS_SEL_INITIAL) {
                        obj->info->currAvg = obj->info->curr;
                } else {
                        tmp32 = (_meas_s32_)obj->info->curr;
                        tmp32 = tmp32 + obj->info->currAvg;
                        tmp32 = tmp32/2;
                        obj->info->currAvg = (_meas_u16_)tmp32;
                        UG31_LOGN("[%s]: Average current = %d (%d)\n", __func__,
                                  obj->info->currAvg,
                                  obj->info->curr);
                }
        }
        if((select == MEAS_SEL_ALL) || (select == MEAS_SEL_INT_TEMP) || (select == MEAS_SEL_INITIAL)) {
                ConvertIntTemperature(obj);

#ifdef  MEAS_FAKE_INT_TEMP
                data->extTemperature = data->intTemperature;
                data->intTemperature = MEAS_FAKE_INT_TEMP_OFFSET + data->intTemperature%100;
#endif  ///< end of MEAS_FAKE_INT_TEMP
        }
        if((select == MEAS_SEL_ALL) || (select == MEAS_SEL_EXT_TEMP) || (select == MEAS_SEL_INITIAL)) {
                /// [AT-PM] : Convert instant external temperature ; 11/27/2013
                tmp16 = obj->info->codeExtTemperature;
                obj->info->codeExtTemperature = obj->info->codeInstExtTemperature;
                ConvertExtTemperature(obj);
                obj->info->instExtTemperature = obj->info->extTemperature;
                /// [AT-PM] : Convert average external temperature ; 11/27/2013
                obj->info->codeExtTemperature = tmp16;
                ConvertExtTemperature(obj);
        }
        if((select == MEAS_SEL_ALL) || (select == MEAS_SEL_INITIAL)) {
                ConvertCharge(obj);

                /// [AT-PM] : Count cycle count ; 10/08/2013
                CountCycleCount(obj);

                /// [AT-PM] : Count cumulative capacity ; 02/18/2014
                CountCumuCap(obj);

                /// [AT-PM] : Calculate coulomb counter offset ; 09/24/2013
                CalculateCCOffset(obj);

                /// [AT-PM] : Reset coulomb counter if necessary ; 01/27/2013
                standbyUpper = (_meas_s16_)obj->info->sysData->ggbParameter->standbyCurrent;
                standbyUpper = standbyUpper/RESET_CC_CURRENT_MAGIC_NUMBER;
                standbyLower = standbyUpper*(-1);
                if((obj->codeCharge > COULOMB_COUNTER_RESET_THRESHOLD_CHARGE_CHG) ||
                    (obj->codeCharge < COULOMB_COUNTER_RESET_THREDHOLD_CHARGE_DSG) ||
                    ((obj->info->curr < standbyUpper) &&
                     (obj->info->curr > standbyLower) &&
                     (obj->codeCounter > CONST_CONVERSION_COUNT_THRESHOLD*RESET_CC_CURRENT_MAGIC_NUMBER)) ||
                    (obj->info->deltaTime > RESET_CC_DELTA_TIME)) {
                        ResetCoulombCounter(obj);
                        data->lastDeltaCap = 0;

                        /// [AT-PM] : Read ADC code ; 01/27/2013
                        FetchAdcCode(obj);
                        obj->info->lastCounter = obj->codeCounter;
                }
        }

        UG31_LOGI("[%s]: %d mV / %d mA / %d 0.1oC / %d 0.1oC / %d mAh\n", __func__,
                  data->bat1Voltage, data->curr, data->intTemperature, data->extTemperature, data->deltaCap);
#ifdef  UG31XX_SHELL_ALGORITHM
        upi_free(obj);
#endif  ///< end of UG31XX_SHELL_ALGORITHM
        return (rtn);
}

/**
 * @brief UpiMeasAlarmThreshold
 *
 *  Get alarm threshold
 *
 * @para  data  address of MeasDataType
 * @return  MEAS_RTN_CODE
 */
MEAS_RTN_CODE UpiMeasAlarmThreshold(MeasDataType *data)
{
        MEAS_RTN_CODE rtn;

        rtn = MEAS_RTN_PASS;

        /// [AT-PM] : Calculate UV alarm and release threshold ; 04/08/2013
        data->sysData->uvAlarm.alarmThrd = RevertBat1Code(data, data->sysData->ggbParameter->uvAlarm);
        data->sysData->uvAlarm.releaseThrd = RevertBat1Code(data, data->sysData->ggbParameter->uvRelease);
        UG31_LOGN("[%s]: UV Alarm -> %d / %d\n", __func__,
                  data->sysData->uvAlarm.alarmThrd, data->sysData->uvAlarm.releaseThrd);

        /// [AT-PM] : Calculate UET alarm and release threshold ; 04/08/2013
        data->sysData->uetAlarm.alarmThrd = RevertETCode(data, data->sysData->ggbParameter->uetAlarm);
        data->sysData->uetAlarm.releaseThrd = RevertETCode(data, data->sysData->ggbParameter->uetRelease);
        UG31_LOGN("[%s]: UET Alarm -> %d / %d\n", __func__,
                  data->sysData->uetAlarm.alarmThrd, data->sysData->uetAlarm.releaseThrd);

        /// [AT-PM] : Calculate OET alarm and release threshold ; 04/08/2013
        data->sysData->oetAlarm.alarmThrd = RevertETCode(data, data->sysData->ggbParameter->oetAlarm);
        data->sysData->oetAlarm.releaseThrd = RevertETCode(data, data->sysData->ggbParameter->oetRelease);
        UG31_LOGN("[%s]: OET Alarm -> %d / %d\n", __func__,
                  data->sysData->oetAlarm.alarmThrd, data->sysData->oetAlarm.releaseThrd);

        return (rtn);
}

/**
 * @brief UpiMeasReadCode
 *
 *  Read ADC code
 *
 * @para  data  address of MeasDataType
 * @return  MEAS_RTN_CODE
 */
MEAS_RTN_CODE UpiMeasReadCode(MeasDataType *data)
{
        MEAS_RTN_CODE rtn;
        MeasDataInternalType *obj;

#ifdef  UG31XX_SHELL_ALGORITHM
        obj = (MeasDataInternalType *)upi_malloc(sizeof(MeasDataInternalType));
#else   ///< else of UG31XX_SHELL_ALGORITHM
        obj = &measData;
#endif  ///< end of UG31XX_SHELL_ALGORITHM
        upi_memset(obj, 0x00, sizeof(MeasDataInternalType));

        obj->info = data;
        obj->info->fetchRetryCnt = MEAS_MAXIMUM_INITIAL_RETRY_CNT;

        rtn = FetchAdcCode(obj);

#ifdef  UG31XX_SHELL_ALGORITHM
        upi_free(obj);
#endif  ///< end of UG31XX_SHELL_ALGORITHM
        return (rtn);
}

/**
 * @brief UpiGetMeasurementMemorySize
 *
 *  Get memory size used by measurement
 *
 * @return  memory size
 */
_meas_u32_ UpiGetMeasurementMemorySize(void)
{
        _meas_u32_ totalSize;

#ifndef UG31XX_SHELL_ALGORITHM

        totalSize = (_meas_u32_)sizeof(measData);
        UG31_LOGD("[%s]: memory size for measData = %d\n", __func__, (int)totalSize);

#else   ///< else of UG31XX_SHELL_ALGORITHM

        totalSize = 0;

#endif  ///< end of UG31XX_SHELL_ALGORITHM

        return (totalSize);
}


/**
 * Copyright @ 2013 uPI Semiconductor Corp. All right reserved.
 * The information, images, and/or data contained in this material is copyrighted by uPI
 * Semiconductor Corp., and may not be distributed, modified, reproduced in whole or in part
 * without the prior, written consent of uPI Semiconductor Corp.
 */
