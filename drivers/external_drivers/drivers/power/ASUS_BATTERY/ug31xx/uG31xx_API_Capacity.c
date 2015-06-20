/**
 * Copyright @ 2013 uPI Semiconductor Corp. All right reserved.
 * The information, images, and/or data contained in this material is copyrighted by uPI
 * Semiconductor Corp., and may not be distributed, modified, reproduced in whole or in part
 * without the prior, written consent of uPI Semiconductor Corp.
 */

/**
 * @filename  uG31xx_API_Capacity.cpp
 *
 *  Capacity algorithm
 *
 * @author  AllenTeng <allen_teng@upi-semi.com>
 * @revision  $Revision: 29 $
 */

#include "stdafx.h"     //windows need this??
#include "uG31xx_API.h"

#ifdef  uG31xx_OS_ANDROID

#define CAPACITY_VERSION      ("Capacity $Rev: 29 $")
//#define CAP_LOG_UPDATE_TABLE                              ///< [AT-PM] : Log updated table to a file ; 03/25/2013

#ifdef  CAP_LOG_UPDATE_TABLE

#define CAP_LOG_UPDATE_TABLE_PATH ("/sdcard/upi_table")

#endif  ///< end of CAP_LOG_UPDATE_TABLE

#else   ///< else of uG31xx_OS_ANDROID

#if defined(BUILD_UG31XX_LIB)

#define CAPACITY_VERSION      ("Capacity $Rev: 29 $")

#else   ///< else of defined(BUILD_UG31XX_LIB)

#define CAPACITY_VERSION      (_T("Capacity $Rev: 29 $"))

#endif  ///< end of defined(BUILD_UG31XX_LIB)

#endif  ///< end of uG31xx_OS_ANDROID

#define CAP_STS_LAST_STATE          (3<<0)
#define CAP_STS_LAST_STANDBY      (0<<0)
#define CAP_STS_LAST_CHG          (1<<0)
#define CAP_STS_LAST_DSG          (2<<0)
#define CAP_STS_CURR_STATE          (3<<2)
#define CAP_STS_CURR_STANDBY      (0<<2)
#define CAP_STS_CURR_CHG          (1<<2)
#define CAP_STS_CURR_DSG          (2<<2)
#define CAP_STS_FC                  (1<<4)
#define CAP_STS_UPDATE_FCC          (1<<5)
#define CAP_STS_FCC_START_DSG       (1<<6)
#define CAP_STS_REFRESH_FCC         (1<<7)
#define CAP_STS_INIT_TIMER_PASS     (1<<8)
#define CAP_STS_INIT_PROCEDURE      (1<<9)
#define CAP_STS_NAC_UPDATE_DISQ     (1<<10)
#define CAP_STS_CHARGER_FULL        (1<<11)
#define CAP_STS_CHG_CV_MODE         (1<<12)
#define CAP_STS_CHG_FCC_UPDATE      (1<<13)
#define CAP_STS_DSGCHARGE_INITED    (1<<16)
#define CAP_STS_DSG_AFTER_FC        (1<<17)
#define CAP_STS_DSG_REACH_EDVF      (1<<18)
#define CAP_STS_V_OVER_MAX_TABLE    (1<<19)
#define CAP_STS_V_UNDER_MIN_TABLE   (1<<20)
#define CAP_STS_BOARD_OFFSET_KED    (1<<21)
#define CAP_STS_FORCE_STEP_TO_100   (1<<22)
#define CAP_STS_PREV_FC             (1<<23)
#define CAP_STS_FILTER_LOCK_OVER    (1<<24)
#define CAP_STS_NO_STANDBY_CAP_EST  (1<<25)

enum INDEX_BOUNDARY {
        INDEX_BOUNDARY_LOW = 0,
        INDEX_BOUNDARY_HIGH,
        INDEX_BOUNDARY_COUNT,
};

typedef struct CapacityInternalDataST {

        CapacityDataType *info;

        _cap_s16_ stepCap;
        _cap_u8_ cRate;

        _cap_u8_ idxTemperature[INDEX_BOUNDARY_COUNT];
        _cap_u8_ idxOcvVoltage[INDEX_BOUNDARY_COUNT];
        _cap_u8_ idxNacCRate[INDEX_BOUNDARY_COUNT];
        _cap_u8_ idxNacVoltage[INDEX_BOUNDARY_COUNT];

        _cap_s16_ tableNacVoltage[OCV_NUMS];
        _cap_s16_ tableNac[OCV_NUMS];

        _cap_u16_ rm;
        _cap_u16_ fcc;
        _cap_u16_ fccRM;

        _cap_u8_ filterNewRsoc;
        _cap_u8_ filterLastRsoc;

#if defined(uG31xx_OS_ANDROID)
} __attribute__ ((packed)) CapacityInternalDataType;
#else   ///< else of defined(uG31xx_OS_ANDROID)	      
} CapacityInternalDataType;
#endif  ///< end of defined(uG31xx_OS_ANDROID)


typedef void (*VerFuncArguObjRtnNull)(CapacityInternalDataType *obj);
typedef void (*VerFuncArguObjS16RtnNull)(CapacityInternalDataType *obj, _cap_s16_ argu);

#ifndef MEAS_STATUS_REFER_ET

#define MEAS_STATUS_REFER_ET      (1<<4)

#endif  ///< end of MEAS_STATUS_REFER_ET

/**
 * @brief GetBatteryTemperature
 *
 *  Get battery temperature from internal or external temperature
 *
 * @para  obj address of CapacityInternalDataType
 * @return  temperature in 0.1oC
 */
_cap_s16_ GetBatteryTemperature(CapacityInternalDataType *obj)
{
        if(obj->info->measurement->status & MEAS_STATUS_REFER_ET) {
                return ((_cap_s16_)obj->info->measurement->extTemperature);
        } else {
                return ((_cap_s16_)obj->info->measurement->intTemperature);
        }
}

/**
 * @brief GetBatteryState
 *
 *  Get battery state, charging/discharging/standby
 *
 * @para  obj address of CapacityInternalDataType
 * @return  _UPI_NULL_
 */
void GetBatteryState(CapacityInternalDataType *obj)
{
        _cap_u32_ tmp32;

        /// [AT-PM] : Update last state ; 01/25/2013
        tmp32 = (obj->info->status & CAP_STS_CURR_STATE) >> 2;

        if(obj->info->measurement->currAvg < -(obj->info->ggbParameter->standbyCurrent)) {
                /// [AT-PM] : Discharging mode ; 01/25/2013
                tmp32 = tmp32 | CAP_STS_CURR_DSG;
        } else if(obj->info->measurement->currAvg > obj->info->ggbParameter->standbyCurrent) {
                /// [AT-PM] : Charging mode ; 01/25/2013
                tmp32 = tmp32 | CAP_STS_CURR_CHG;
        } else {
                /// [AT-PM] : Standby mode ; 01/25/2013
                tmp32 = tmp32 | CAP_STS_CURR_STANDBY;
        }

        obj->info->status = obj->info->status & (~(CAP_STS_LAST_STATE | CAP_STS_CURR_STATE));
        obj->info->status = obj->info->status | tmp32;
}


/**
 * @brief ResetSelfD
 *
 *  Reset self-discharging capacity
 *
 * @para  obj address of CapacityInternalDataType
 * @return _UPI_NULL_
 */
void ResetSelfD(CapacityInternalDataType *obj)
{
        obj->info->selfDsgMilliSec = 0;
        obj->info->selfDsgSec = 0;
        obj->info->selfDsgMin = 0;
        obj->info->selfDsgHour = 0;
        UG31_LOGD("[%s]: Reset self-discharge calculation\n", __func__);
}

/**
 * @brief CapStatusFCSet
 *
 *  Set CAP_STS_FC
 *
 * @para  info  address of CapacityDataType
 * @return  NULL
 */
void CapStatusFCSet(CapacityDataType *info)
{
        info->status = info->status | CAP_STS_FC;
        info->fcSts = CAP_TRUE;
        info->fcStep100 = CAP_TRUE;
}

/**
 * @brief CapStatusFCClear
 *
 *  Clear CAP_STS_FC
 *
 * @para  info  address of CapacityDataType
 * @return  NULL
 */
void CapStatusFCClear(CapacityDataType *info)
{
        info->status = info->status & (~CAP_STS_FC);
        info->fcSts = CAP_FALSE;
}

/**
 * @brief CapStatusFCGet
 *
 *  Get CAP_STS_FC value
 *
 * @para  info  address of CapacityDataType
 * @return  CAP_TRUE if set
 */
_cap_bool_ CapStatusFCGet(CapacityDataType *info)
{
        return ((info->status & CAP_STS_FC) ? CAP_TRUE : CAP_FALSE);
}

/**
 * @brief FullChargeRelease
 *
 *  Release full charge state
 *
 * @para  obj address of CapacityInternalDataType
 * @return  _UPI_NULL_
 */
void FullChargeRelease(CapacityInternalDataType *obj)
{
        CapStatusFCClear(obj->info);
        obj->info->status = obj->info->status & (~CAP_STS_CHARGER_FULL);
        obj->info->tpTime = 0;
}

#define FULL_CHARGE_RSOC            (100)
#define MAGIC_NUMBER_CHARGE_RSOC    (1)
#define CHG_PREDICT_SOC_RATIO       (100)
#define CHG_PREDICT_SOC_STEP_RATIO  (10)
#define MIN_CHARGE_PREDICT_RSOC     (90)
#define CHG_CV_MODE_CHECK_COUNT     (2)

/**
 * @brief PredictChargeCapacity
 *
 *  Predict charging capacity according to current
 *
 * @para  obj address of CapacityInternalDataType
 * @return  _UPI_NULL_
 */
void PredictChargeCapacity(CapacityInternalDataType *obj)
{
        _cap_s16_ curr;
        _cap_u8_ rsoc;
        _cap_s32_ tmp32;
        _cap_s16_ currStep;
        _cap_s16_ deltaCurrStep;
        _cap_s16_ maxCurrStep;

        /// [AT-PM] : Check current is decreasing ; 08/02/2013
        curr = (_cap_s16_)obj->info->measurement->currAvg;
        curr = curr - obj->info->ggbParameter->TPCurrent;
        UG31_LOGD("[%s]: Delta Charging Current = %d (%d) -> %d\n", __func__, curr, obj->info->lastCVDeltaChgCurr, obj->info->cvCheckCnt);
        if(obj->info->lastCVDeltaChgCurr <= 0) {
                obj->info->lastCVDeltaChgCurr = curr;
                obj->info->cvCheckCnt = 0;
                obj->info->status = obj->info->status & (~CAP_STS_CHG_CV_MODE);
                return;
        }
        if(obj->info->lastCVDeltaChgCurr <= curr) {
                obj->info->status = obj->info->status & (~CAP_STS_CHG_CV_MODE);
                return;
        }
        obj->info->lastCVDeltaChgCurr = curr;
        obj->info->cvCheckCnt = obj->info->cvCheckCnt + 1;
        if(obj->info->cvCheckCnt < CHG_CV_MODE_CHECK_COUNT) {
                return;
        }
        obj->info->cvCheckCnt = 0;
        obj->info->status = obj->info->status | CAP_STS_CHG_CV_MODE;

        /// [AT-PM] : Calculate current step ; 08/02/2013
        maxCurrStep = (_cap_s16_)obj->info->ggbParameter->ILMD;
        maxCurrStep = maxCurrStep/CHG_PREDICT_SOC_RATIO;
        if(maxCurrStep <= 0) {
                maxCurrStep = 1;
        }
        deltaCurrStep = maxCurrStep/CHG_PREDICT_SOC_STEP_RATIO;
        if(deltaCurrStep <= 0) {
                deltaCurrStep = 1;
        }

        UG31_LOGD("[%s]: (0x%08x) %d -> %d (%d) \n", __func__,
                  (unsigned int)obj->info->status, obj->info->measurement->currAvg, obj->info->ggbParameter->TPCurrent, maxCurrStep);

        /// [AT-PM] : Find the target RSOC ; 01/26/2013
        rsoc = FULL_CHARGE_RSOC - 1;
        currStep = obj->info->ggbParameter->standbyCurrent;
        while(rsoc) {
                curr = curr - currStep;
                if(curr < 0) {
                        break;
                }
                currStep = currStep + deltaCurrStep;
                if(currStep > maxCurrStep) {
                        currStep = maxCurrStep;
                }
                rsoc = rsoc - 1;
        }
        UG31_LOGD("[%s]: RSOC = %d (%d)\n", __func__, rsoc, obj->info->rsoc);

        if(obj->info->rsoc == CAP_FC_RELEASE_RSOC) {
                UG31_LOGD("[%s]: Adjust RM at 99%% = %d\n", __func__, obj->rm);
                return;
        }

        /// [AT-PM] : Calculate average charging RM ; 08/01/2013
        tmp32 = (_cap_s32_)rsoc;
        tmp32 = tmp32*obj->fcc/CONST_PERCENTAGE;
        tmp32 = (tmp32 + obj->info->avgRM)/2;
        obj->info->avgRM = (_cap_u16_)tmp32;
        UG31_LOGD("[%s]: Average RM for CV mode = %d\n", __func__, obj->info->avgRM);
}

#define MINIMUM_TABLE_UPDATE_IDX  (1)

/**
 * @brief ResetSelfLearning
 *
 *  Reset self-learning status
 *
 * @para  obj address of CapacityInternalDataType
 * @return  NULL
 */
void ResetSelfLearning(CapacityInternalDataType *obj)
{
        UG31_LOGD("[%s]: Reset self-learning status\n", __func__);
        obj->info->dsgCharge = 0;
        obj->info->dsgChargeStart = 0;
        obj->info->dsgChargeTime = 0;
        obj->info->tableUpdateIdx = MINIMUM_TABLE_UPDATE_IDX;
}

/**
 * @brief FindNacIdxVoltage
 *
 *  Find the NAC table voltage index
 *
 * @para  obj address of CapacityInternalDataType
 * @para  voltage battery voltage
 * @return  _UPI_NULL_
 */
void FindNacIdxVoltage(CapacityInternalDataType *obj, _cap_u16_ voltage)
{
        /// [AT-PM] : Find the index ; 01/25/2013
        obj->idxNacVoltage[INDEX_BOUNDARY_LOW] = 0;
        obj->info->status = obj->info->status & (~(CAP_STS_V_OVER_MAX_TABLE | CAP_STS_V_UNDER_MIN_TABLE));
        while(obj->idxNacVoltage[INDEX_BOUNDARY_LOW] < OCV_NUMS) {
                if(voltage >= obj->tableNacVoltage[obj->idxNacVoltage[INDEX_BOUNDARY_LOW]]) {
                        break;
                }
                obj->idxNacVoltage[INDEX_BOUNDARY_LOW] = obj->idxNacVoltage[INDEX_BOUNDARY_LOW] + 1;
        }

        /// [AT-PM] : Higher than upper bound ; 01/25/2013
        if(obj->idxNacVoltage[INDEX_BOUNDARY_LOW] == 0) {
                obj->idxNacVoltage[INDEX_BOUNDARY_HIGH] = obj->idxNacVoltage[INDEX_BOUNDARY_LOW];
                obj->info->status = obj->info->status | CAP_STS_V_OVER_MAX_TABLE;
                return;
        }

        /// [AT-PM] : Lower than lower bound ; 01/25/2013
        if(obj->idxNacVoltage[INDEX_BOUNDARY_LOW] >= SOV_NUMS) {
                obj->idxNacVoltage[INDEX_BOUNDARY_LOW] = SOV_NUMS - 1;
                obj->idxNacVoltage[INDEX_BOUNDARY_HIGH] = obj->idxNacVoltage[INDEX_BOUNDARY_LOW];
                obj->info->status = obj->info->status | CAP_STS_V_UNDER_MIN_TABLE;
                return;
        }

        obj->idxNacVoltage[INDEX_BOUNDARY_HIGH] = obj->idxNacVoltage[INDEX_BOUNDARY_LOW] - 1;

        if((obj->info->ggbParameter->NacLmdAdjustCfg & NAC_LMD_ADJUST_CFG_ALWAYS_UPDATE_FCC_AT_LAST) &&
            (obj->idxNacVoltage[INDEX_BOUNDARY_LOW] >= (SOV_NUMS - 1)) &&
            (obj->info->tableUpdateIdx < SOV_NUMS)) {
                obj->info->status = obj->info->status | (CAP_STS_UPDATE_FCC | CAP_STS_REFRESH_FCC);
        }
}

/**
 * @brief FindOcvIdxVoltage
 *
 *  Find the OCV tables voltage index
 *
 * @para  obj address of CapacityInternalDataType
 * @para  tableIdx  index of INIT_OCV table
 * @para  voltage battery voltage
 * @return  _UPI_NULL_
 */
void FindOcvIdxVoltage(CapacityInternalDataType *obj, _cap_u8_ tableIdx, _cap_u16_ voltage)
{
        /// [AT-PM] : Find the index ; 01/25/2013
        obj->idxOcvVoltage[INDEX_BOUNDARY_LOW] = 0;
        while(obj->idxOcvVoltage[INDEX_BOUNDARY_LOW] < OCV_NUMS) {
                if(voltage >= obj->info->ggbTable->INIT_OCV[obj->idxTemperature[INDEX_BOUNDARY_LOW]]
                    [tableIdx]
                    [obj->idxOcvVoltage[INDEX_BOUNDARY_LOW]]) {
                        break;
                }
                obj->idxOcvVoltage[INDEX_BOUNDARY_LOW] = obj->idxOcvVoltage[INDEX_BOUNDARY_LOW] + 1;
        }

        /// [AT-PM] : Higher than upper bound ; 01/25/2013
        if(obj->idxOcvVoltage[INDEX_BOUNDARY_LOW] == 0) {
                obj->idxOcvVoltage[INDEX_BOUNDARY_HIGH] = obj->idxOcvVoltage[INDEX_BOUNDARY_LOW];
                return;
        }

        /// [AT-PM] : Lower than lower bound ; 01/25/2013
        if(obj->idxOcvVoltage[INDEX_BOUNDARY_LOW] >= OCV_NUMS) {
                obj->idxOcvVoltage[INDEX_BOUNDARY_LOW] = OCV_NUMS - 1;
                obj->idxOcvVoltage[INDEX_BOUNDARY_HIGH] = obj->idxOcvVoltage[INDEX_BOUNDARY_LOW];
                return;
        }

        obj->idxOcvVoltage[INDEX_BOUNDARY_HIGH] = obj->idxOcvVoltage[INDEX_BOUNDARY_LOW] - 1;
}

/**
 * @brief FindOcvFcc
 *
 *  Find the FCC for OCV table, which is the average from NAC table
 *
 * @para  obj address of CapacityInternalDataType
 * @return  _UPI_NULL_
 */
void FindOcvFcc(CapacityInternalDataType *obj)
{
        _cap_s32_ tmp32;
        _cap_u8_ idx;

        tmp32 = 0;
        idx = 0;
        while(idx < C_RATE_NUMS) {
                tmp32 = tmp32 + obj->info->ggbTable->CELL_NAC_TABLE[obj->idxTemperature[INDEX_BOUNDARY_HIGH]][idx][0];
                idx = idx + 1;
        }
        tmp32 = tmp32/C_RATE_NUMS;
        obj->info->fcc = (_cap_u16_)tmp32;
}

static _cap_u8_ OcvSocTable[] = {
        100,
        95,
        90,
        85,
        80,
        75,
        70,
        65,
        60,
        55,
        50,
        45,
        40,
        35,
        30,
        25,
        20,
        15,
        10,
        5,
        0,
};

static _cap_s16_ TemperatureTable[] = {
        450,
        250,
        150,
        50,
};

/**
 * @brief FindOcvRM
 *
 *  Find the RM for OCV table
 *  -> RM = FCC x RSOC
 *
 * @para  obj address of CapacityInternalDataType
 * @para  tableIdx  OCV table index
 * @para  voltage target voltage
 * @return  _UPI_NULL_
 */
void FindOcvRM(CapacityInternalDataType *obj, _cap_u8_ tableIdx, _cap_u16_ voltage)
{
        _cap_s32_ tmp32;
        _cap_s32_ voltHigh;
        _cap_s32_ voltLow;

        UG31_LOGN("[%s] INIT_OCV[%d][%d][%d] (%d) = %d\n", __func__,
                  obj->idxTemperature[INDEX_BOUNDARY_LOW],
                  tableIdx,
                  obj->idxOcvVoltage[INDEX_BOUNDARY_LOW],
                  OcvSocTable[obj->idxOcvVoltage[INDEX_BOUNDARY_LOW]],
                  obj->info->ggbTable->INIT_OCV[obj->idxTemperature[INDEX_BOUNDARY_LOW]]
                  [tableIdx]
                  [obj->idxOcvVoltage[INDEX_BOUNDARY_LOW]]);
        UG31_LOGN("[%s] INIT_OCV[%d][%d][%d] (%d) = %d\n", __func__,
                  obj->idxTemperature[INDEX_BOUNDARY_LOW],
                  tableIdx,
                  obj->idxOcvVoltage[INDEX_BOUNDARY_HIGH],
                  OcvSocTable[obj->idxOcvVoltage[INDEX_BOUNDARY_HIGH]],
                  obj->info->ggbTable->INIT_OCV[obj->idxTemperature[INDEX_BOUNDARY_LOW]]
                  [tableIdx]
                  [obj->idxOcvVoltage[INDEX_BOUNDARY_HIGH]]);
        UG31_LOGN("[%s] INIT_OCV[%d][%d][%d] (%d) = %d\n", __func__,
                  obj->idxTemperature[INDEX_BOUNDARY_HIGH],
                  tableIdx,
                  obj->idxOcvVoltage[INDEX_BOUNDARY_LOW],
                  OcvSocTable[obj->idxOcvVoltage[INDEX_BOUNDARY_LOW]],
                  obj->info->ggbTable->INIT_OCV[obj->idxTemperature[INDEX_BOUNDARY_HIGH]]
                  [tableIdx]
                  [obj->idxOcvVoltage[INDEX_BOUNDARY_LOW]]);
        UG31_LOGN("[%s] INIT_OCV[%d][%d][%d] (%d) = %d\n", __func__,
                  obj->idxTemperature[INDEX_BOUNDARY_HIGH],
                  tableIdx,
                  obj->idxOcvVoltage[INDEX_BOUNDARY_HIGH],
                  OcvSocTable[obj->idxOcvVoltage[INDEX_BOUNDARY_HIGH]],
                  obj->info->ggbTable->INIT_OCV[obj->idxTemperature[INDEX_BOUNDARY_HIGH]]
                  [tableIdx]
                  [obj->idxOcvVoltage[INDEX_BOUNDARY_HIGH]]);

        /// [AT-PM] : Interpolate voltage ; 02/16/2014
        if(obj->idxTemperature[INDEX_BOUNDARY_HIGH] == obj->idxTemperature[INDEX_BOUNDARY_LOW]) {
                voltHigh = obj->info->ggbTable->INIT_OCV[obj->idxTemperature[INDEX_BOUNDARY_LOW]]
                           [tableIdx]
                           [obj->idxOcvVoltage[INDEX_BOUNDARY_HIGH]];
                voltLow = obj->info->ggbTable->INIT_OCV[obj->idxTemperature[INDEX_BOUNDARY_LOW]]
                          [tableIdx]
                          [obj->idxOcvVoltage[INDEX_BOUNDARY_LOW]];
        } else {
                tmp32 = (_cap_s32_)obj->info->ggbTable->INIT_OCV[obj->idxTemperature[INDEX_BOUNDARY_HIGH]]
                        [tableIdx]
                        [obj->idxOcvVoltage[INDEX_BOUNDARY_HIGH]];
                tmp32 = tmp32 - obj->info->ggbTable->INIT_OCV[obj->idxTemperature[INDEX_BOUNDARY_LOW]]
                        [tableIdx]
                        [obj->idxOcvVoltage[INDEX_BOUNDARY_HIGH]];
                tmp32 = tmp32*(GetBatteryTemperature(obj) -
                               TemperatureTable[obj->idxTemperature[INDEX_BOUNDARY_LOW]]);
                tmp32 = tmp32/(TemperatureTable[obj->idxTemperature[INDEX_BOUNDARY_HIGH]] -
                               TemperatureTable[obj->idxTemperature[INDEX_BOUNDARY_LOW]]);
                tmp32 = tmp32 + obj->info->ggbTable->INIT_OCV[obj->idxTemperature[INDEX_BOUNDARY_LOW]]
                        [tableIdx]
                        [obj->idxOcvVoltage[INDEX_BOUNDARY_HIGH]];
                voltHigh = tmp32;

                tmp32 = (_cap_s32_)obj->info->ggbTable->INIT_OCV[obj->idxTemperature[INDEX_BOUNDARY_HIGH]]
                        [tableIdx]
                        [obj->idxOcvVoltage[INDEX_BOUNDARY_LOW]];
                tmp32 = tmp32 - obj->info->ggbTable->INIT_OCV[obj->idxTemperature[INDEX_BOUNDARY_LOW]]
                        [tableIdx]
                        [obj->idxOcvVoltage[INDEX_BOUNDARY_LOW]];
                tmp32 = tmp32*(GetBatteryTemperature(obj) -
                               TemperatureTable[obj->idxTemperature[INDEX_BOUNDARY_LOW]]);
                tmp32 = tmp32/(TemperatureTable[obj->idxTemperature[INDEX_BOUNDARY_HIGH]] -
                               TemperatureTable[obj->idxTemperature[INDEX_BOUNDARY_LOW]]);
                tmp32 = tmp32 + obj->info->ggbTable->INIT_OCV[obj->idxTemperature[INDEX_BOUNDARY_LOW]]
                        [tableIdx]
                        [obj->idxOcvVoltage[INDEX_BOUNDARY_LOW]];
                voltLow = tmp32;
                UG31_LOGN("[%s]: Voltage High = %d and Low = %d\n", __func__,
                          (int)voltHigh,
                          (int)voltLow);
        }

        /// [AT-PM] : Calculate RSOC ; 01/25/2013
        if(OcvSocTable[obj->idxOcvVoltage[INDEX_BOUNDARY_LOW]] == OcvSocTable[obj->idxOcvVoltage[INDEX_BOUNDARY_HIGH]]) {
                tmp32 = OcvSocTable[obj->idxOcvVoltage[INDEX_BOUNDARY_LOW]];
        } else {
                tmp32 = (_cap_s32_)voltage;
                tmp32 = tmp32 - voltLow;
                tmp32 = tmp32*
                        (OcvSocTable[obj->idxOcvVoltage[INDEX_BOUNDARY_HIGH]] -
                         OcvSocTable[obj->idxOcvVoltage[INDEX_BOUNDARY_LOW]]);
                tmp32 = tmp32/(voltHigh - voltLow);
                tmp32 = tmp32 + OcvSocTable[obj->idxOcvVoltage[INDEX_BOUNDARY_LOW]];
        }
        UG31_LOGD("[%s] RSOC = %d (%d)\n", __func__,
                  obj->info->rsoc,
                  (int)tmp32);

        /// [AT-PM] : Calculate RM ; 01/25/2013
        tmp32 = tmp32*obj->info->fcc/CONST_PERCENTAGE;
        obj->rm = (_cap_u16_)tmp32;
}

/**
 * @brief ChargeSpeedDown
 *
 *  Limit RSOC incresing speed during charging mode
 *
 * @para  obj address of CapacityInternalDataType
 * @return  _UPI_NULL_
 */
void ChargeSpeedDown(CapacityInternalDataType *obj)
{
        _cap_s32_ tmp32 = 0;
        _cap_u8_ rsoc;

        if(obj->info->transferStateToChg > 0) {
                UG31_LOGD("[%s] Start voltage = %d, RSOC = %d\n", __func__, obj->info->startChgVolt, obj->info->startChgRsoc);
                return;
        }

        /// [AT-PM] : Calculate new RSOC ; 02/09/2014
        rsoc = (_cap_u8_)CalculateRsoc((_cap_u32_)obj->rm, obj->info->fcc);

        /// [FC] : Predict charge rsoc before VBAT greater than TP voltage; 07/05/2013
        tmp32 = (_cap_s32_)obj->info->avgVoltage;
        tmp32 = tmp32 - obj->info->startChgVolt;
        tmp32 = tmp32 * (MIN_CHARGE_PREDICT_RSOC - obj->info->startChgRsoc);
        if(obj->info->ggbParameter->TPVoltage > obj->info->startChgVolt) {
                tmp32 = tmp32 / (obj->info->ggbParameter->TPVoltage - obj->info->startChgVolt);
        }
        tmp32 = tmp32 + obj->info->startChgRsoc;
        UG31_LOGD("[%s] Predict RSOC = %d, Current RSOC = %d (%d)\n", __func__,
                  (int)tmp32,
                  rsoc,
                  obj->info->rsoc);
        if(rsoc > tmp32) {
                tmp32 = tmp32 * obj->fcc / CONST_PERCENTAGE;
                UG31_LOGN("[%s]: Limit RM = %d from %d (%d - %d).\n", __func__,
                          tmp32,
                          obj->rm,
                          rsoc,
                          obj->info->fcc);
                obj->rm = (_cap_u16_)tmp32;
        }
}

/**
 * @brief FullChargeSet
 *
 *  Set internal status after full charge
 *
 * @para  obj address of CapacityInternalDataType
 * @return  NULL
 */
void FullChargeSet(CapacityInternalDataType *obj)
{
        ResetSelfD(obj);
        ResetSelfLearning(obj);

        /// [AT-PM] : Set full charge state ; 01/25/2013
        CapStatusFCSet(obj->info);
        obj->info->status = obj->info->status | CAP_STS_DSG_AFTER_FC;
        obj->info->status = obj->info->status & (~CAP_STS_DSGCHARGE_INITED);
        obj->rm = obj->fcc;
}


/**
 * @brief FindIdxTemperatureVer0
 *
 *  Find the temperature index
 *
 * @para  obj address of CapacityInternalDataType
 * @para  argu  battery temperature
 * @return  _UPI_NULL_
 */
void FindIdxTemperatureVer0(CapacityInternalDataType *obj, _cap_s16_ argu)
{
        /// [AT-PM] : Find the temperature index ; 01/25/2013
        obj->idxTemperature[INDEX_BOUNDARY_LOW] = 0;
        while(obj->idxTemperature[INDEX_BOUNDARY_LOW] < TEMPERATURE_NUMS) {
                if(argu >= TemperatureTable[obj->idxTemperature[INDEX_BOUNDARY_LOW]]) {
                        break;
                }
                obj->idxTemperature[INDEX_BOUNDARY_LOW] = obj->idxTemperature[INDEX_BOUNDARY_LOW] + 1;
        }

        /// [AT-PM] : Higher than upper bound ; 01/25/2013
        if(obj->idxTemperature[INDEX_BOUNDARY_LOW] == 0) {
                obj->idxTemperature[INDEX_BOUNDARY_HIGH] = obj->idxTemperature[INDEX_BOUNDARY_LOW];
                return;
        }

        /// [AT-PM] : Smaller than lower bound ; 01/25/2013
        if(obj->idxTemperature[INDEX_BOUNDARY_LOW] >= TEMPERATURE_NUMS) {
                obj->idxTemperature[INDEX_BOUNDARY_LOW] = TEMPERATURE_NUMS - 1;
                obj->idxTemperature[INDEX_BOUNDARY_HIGH] = obj->idxTemperature[INDEX_BOUNDARY_LOW];
                return;
        }

        obj->idxTemperature[INDEX_BOUNDARY_HIGH] = obj->idxTemperature[INDEX_BOUNDARY_LOW] - 1;
}

/**
 * @brief FindIdxTemperatureVer1
 *
 *  Find the temperature index
 *
 * @para  obj address of CapacityInternalDataType
 * @para  argu  battery temperature
 * @return  _UPI_NULL_
 */
void FindIdxTemperatureVer1(CapacityInternalDataType *obj, _cap_s16_ argu)
{
        _cap_s16_ tempThrd;

        /// [AT-PM] : Get temperature index ; 10/25/2013
        obj->idxTemperature[INDEX_BOUNDARY_LOW] = 0;
        while(obj->idxTemperature[INDEX_BOUNDARY_LOW] < (TEMPERATURE_NUMS - 1)) {
                tempThrd = TemperatureTable[obj->idxTemperature[INDEX_BOUNDARY_LOW]] + TemperatureTable[obj->idxTemperature[INDEX_BOUNDARY_LOW] + 1];
                tempThrd = tempThrd/2;

                if(argu > tempThrd) {
                        break;
                }
                obj->idxTemperature[INDEX_BOUNDARY_LOW] = obj->idxTemperature[INDEX_BOUNDARY_LOW] + 1;
        }
        obj->idxTemperature[INDEX_BOUNDARY_HIGH] = obj->idxTemperature[INDEX_BOUNDARY_LOW];
}

#define FindIdxTemperatureVer2      (FindIdxTemperatureVer1)

static VerFuncArguObjS16RtnNull FindIdxTemperature[] = {
        FindIdxTemperatureVer0,
        FindIdxTemperatureVer1,
        FindIdxTemperatureVer2,
        _UPI_NULL_,
};

#define C_RATE_CONVERT_BASE     (-100)

static _cap_u8_ CRateTable[] = {
        C_RATE_TABLE_VALUE_0,
        C_RATE_TABLE_VALUE_1,
        C_RATE_TABLE_VALUE_2,
        C_RATE_TABLE_VALUE_3,
};

/**
 * @brief FindNacIdxCRateVer0
 *
 *  Find the NAC table CRate index
 *
 * @para  obj address of CapacityInternalDataType
 * @return  _UPI_NULL_
 */
void FindNacIdxCRateVer0(CapacityInternalDataType *obj)
{
        /// [AT-PM] : Find CRate index ; 01/25/2013
        obj->idxNacCRate[INDEX_BOUNDARY_LOW] = 0;
        while(obj->idxNacCRate[INDEX_BOUNDARY_LOW] < C_RATE_NUMS) {
                if(obj->cRate >= CRateTable[obj->idxNacCRate[INDEX_BOUNDARY_LOW]]) {
                        break;
                }
                obj->idxNacCRate[INDEX_BOUNDARY_LOW] = obj->idxNacCRate[INDEX_BOUNDARY_LOW] + 1;
        }

        /// [AT-PM] : Higher than upper bound ; 01/25/2013
        if(obj->idxNacCRate[INDEX_BOUNDARY_LOW] == 0) {
                obj->idxNacCRate[INDEX_BOUNDARY_HIGH] = obj->idxNacCRate[INDEX_BOUNDARY_LOW];
                return;
        }

        /// [AT-PM] : Smaller than lower bound ; 01/25/2013
        if(obj->idxNacCRate[INDEX_BOUNDARY_LOW] >= C_RATE_NUMS) {
                obj->idxNacCRate[INDEX_BOUNDARY_LOW] = C_RATE_NUMS - 1;
                obj->idxNacCRate[INDEX_BOUNDARY_HIGH] = obj->idxNacCRate[INDEX_BOUNDARY_LOW];
                return;
        }

        obj->idxNacCRate[INDEX_BOUNDARY_HIGH] = obj->idxNacCRate[INDEX_BOUNDARY_LOW] - 1;
}

/**
 * @brief FindNacIdxCRateVer1
 *
 *  Find the NAC table CRate index
 *
 * @para  obj address of CapacityInternalDataType
 * @return  _UPI_NULL_
 */
void FindNacIdxCRateVer1(CapacityInternalDataType *obj)
{
        _cap_u8_ cRateThrd;

        obj->idxNacCRate[INDEX_BOUNDARY_LOW] = 0;
        while(obj->idxNacCRate[INDEX_BOUNDARY_LOW] < (C_RATE_NUMS - 1)) {
                cRateThrd = CRateTable[obj->idxNacCRate[INDEX_BOUNDARY_LOW]] + CRateTable[obj->idxNacCRate[INDEX_BOUNDARY_LOW] + 1];
                cRateThrd = cRateThrd/2;

                if(obj->cRate > cRateThrd) {
                        break;
                }
                obj->idxNacCRate[INDEX_BOUNDARY_LOW] = obj->idxNacCRate[INDEX_BOUNDARY_LOW] + 1;
        }
        obj->idxNacCRate[INDEX_BOUNDARY_HIGH] = obj->idxNacCRate[INDEX_BOUNDARY_LOW];
}

#define FindNacIdxCRateVer2     (FindNacIdxCRateVer1)

static VerFuncArguObjRtnNull FindNacIdxCRate[] = {
        FindNacIdxCRateVer0,
        FindNacIdxCRateVer1,
        FindNacIdxCRateVer2,
        _UPI_NULL_,
};

/**
 * @brief CreateNacVoltageTableVer0
 *
 *  Create NAC Voltage table
 *
 * @para  obj address of CapacityInternalDataType
 * @return  _UPI_NULL_
 */
void CreateNacVoltageTableVer0(CapacityInternalDataType *obj)
{
        _cap_u8_ idx;
        _cap_s32_ tmp32;
        _cap_s32_ valueInterpolate1;
        _cap_s32_ valueInterpolate2;
        _cap_s16_ temperature;

        idx = 0;
        temperature = GetBatteryTemperature(obj);
        while(idx < OCV_NUMS) {
                /// [AT-PM] : Temperature interpolation ; 01/25/2013
                if(TemperatureTable[obj->idxTemperature[INDEX_BOUNDARY_HIGH]] ==
                    TemperatureTable[obj->idxTemperature[INDEX_BOUNDARY_LOW]]) {
                        valueInterpolate1 = obj->info->ggbTable->CELL_VOLTAGE_TABLE[obj->idxTemperature[INDEX_BOUNDARY_LOW]]
                                            [obj->idxNacCRate[INDEX_BOUNDARY_LOW]]
                                            [idx];
                        valueInterpolate2 = obj->info->ggbTable->CELL_VOLTAGE_TABLE[obj->idxTemperature[INDEX_BOUNDARY_LOW]]
                                            [obj->idxNacCRate[INDEX_BOUNDARY_HIGH]]
                                            [idx];
                } else {
                        tmp32 = (_cap_s32_)obj->info->ggbTable->CELL_VOLTAGE_TABLE[obj->idxTemperature[INDEX_BOUNDARY_HIGH]]
                                [obj->idxNacCRate[INDEX_BOUNDARY_LOW]]
                                [idx];
                        tmp32 = tmp32 - obj->info->ggbTable->CELL_VOLTAGE_TABLE[obj->idxTemperature[INDEX_BOUNDARY_LOW]]
                                [obj->idxNacCRate[INDEX_BOUNDARY_LOW]]
                                [idx];
                        tmp32 = tmp32*(temperature - TemperatureTable[obj->idxTemperature[INDEX_BOUNDARY_LOW]]);
                        tmp32 = tmp32/
                                (TemperatureTable[obj->idxTemperature[INDEX_BOUNDARY_HIGH]] -
                                 TemperatureTable[obj->idxTemperature[INDEX_BOUNDARY_LOW]]);
                        valueInterpolate1 = tmp32 + obj->info->ggbTable->CELL_VOLTAGE_TABLE[obj->idxTemperature[INDEX_BOUNDARY_LOW]]
                                            [obj->idxNacCRate[INDEX_BOUNDARY_LOW]]
                                            [idx];

                        tmp32 = (_cap_s32_)obj->info->ggbTable->CELL_VOLTAGE_TABLE[obj->idxTemperature[INDEX_BOUNDARY_HIGH]]
                                [obj->idxNacCRate[INDEX_BOUNDARY_HIGH]]
                                [idx];
                        tmp32 = tmp32 - obj->info->ggbTable->CELL_VOLTAGE_TABLE[obj->idxTemperature[INDEX_BOUNDARY_LOW]]
                                [obj->idxNacCRate[INDEX_BOUNDARY_HIGH]]
                                [idx];
                        tmp32 = tmp32*(temperature - TemperatureTable[obj->idxTemperature[INDEX_BOUNDARY_LOW]]);
                        tmp32 = tmp32/
                                (TemperatureTable[obj->idxTemperature[INDEX_BOUNDARY_HIGH]] -
                                 TemperatureTable[obj->idxTemperature[INDEX_BOUNDARY_LOW]]);
                        valueInterpolate2 = tmp32 + obj->info->ggbTable->CELL_VOLTAGE_TABLE[obj->idxTemperature[INDEX_BOUNDARY_LOW]]
                                            [obj->idxNacCRate[INDEX_BOUNDARY_HIGH]]
                                            [idx];
                }

                /// [AT-PM] : CRate interpolation ; 01/25/2013
                if(CRateTable[obj->idxNacCRate[INDEX_BOUNDARY_HIGH]] == CRateTable[obj->idxNacCRate[INDEX_BOUNDARY_LOW]]) {
                        tmp32 = valueInterpolate1;
                } else {
                        tmp32 = valueInterpolate2 - valueInterpolate1;
                        tmp32 = tmp32*(obj->cRate - CRateTable[obj->idxNacCRate[INDEX_BOUNDARY_LOW]]);
                        tmp32 = tmp32/
                                (CRateTable[obj->idxNacCRate[INDEX_BOUNDARY_HIGH]] - CRateTable[obj->idxNacCRate[INDEX_BOUNDARY_LOW]]);
                        tmp32 = tmp32 + valueInterpolate1;
                }
                obj->tableNacVoltage[idx] = (_cap_s16_)tmp32;
                UG31_LOGD("[%s]: NAC voltage Table[%d] = %dmV (%d,%d) (%d,%d)\n", __func__,
                          idx, obj->tableNacVoltage[idx],
                          obj->info->ggbTable->CELL_VOLTAGE_TABLE[obj->idxTemperature[INDEX_BOUNDARY_HIGH]][obj->idxNacCRate[INDEX_BOUNDARY_LOW]][idx],
                          obj->info->ggbTable->CELL_VOLTAGE_TABLE[obj->idxTemperature[INDEX_BOUNDARY_LOW]][obj->idxNacCRate[INDEX_BOUNDARY_LOW]][idx],
                          obj->info->ggbTable->CELL_VOLTAGE_TABLE[obj->idxTemperature[INDEX_BOUNDARY_HIGH]][obj->idxNacCRate[INDEX_BOUNDARY_HIGH]][idx],
                          obj->info->ggbTable->CELL_VOLTAGE_TABLE[obj->idxTemperature[INDEX_BOUNDARY_LOW]][obj->idxNacCRate[INDEX_BOUNDARY_HIGH]][idx]);
                idx = idx + 1;
        }
}

/**
 * @brief CreateNacVoltageTableVer2
 *
 *  Create NAC Voltage table for fixed EDV
 *
 * @para  obj address of CapacityInternalDataType
 * @return  _UPI_NULL_
 */
void CreateNacVoltageTableVer2(CapacityInternalDataType *obj)
{
        _cap_u8_ idx;
        _cap_s16_ base;
        _cap_s32_ deltaSoc;

        obj->tableNacVoltage[0] = (_cap_s16_)obj->info->ggbParameter->TPVoltage;
        obj->tableNacVoltage[SOV_NUMS - 1] = (_cap_s16_)obj->info->ggbParameter->edv1Voltage;
        base = obj->tableNacVoltage[0] - obj->tableNacVoltage[SOV_NUMS - 1];
        UG31_LOGD("[%s]: Head = %d, End = %d, Base = %d\n", __func__, obj->tableNacVoltage[0], obj->tableNacVoltage[SOV_NUMS - 1], base);

        idx = 1;
        while(idx < (SOV_NUMS - 1)) {
                deltaSoc = (_cap_s32_)obj->info->ggbParameter->SOV_TABLE[idx - 1];
                deltaSoc = deltaSoc - obj->info->ggbParameter->SOV_TABLE[idx];
                deltaSoc = deltaSoc*base/CONST_PERCENTAGE/10;
                obj->tableNacVoltage[idx] = (_cap_s16_)deltaSoc;
                obj->tableNacVoltage[idx] = obj->tableNacVoltage[idx - 1] + obj->tableNacVoltage[idx];
                UG31_LOGD("[%s]: V[%d] = %d + %d = %d\n", __func__, idx, obj->tableNacVoltage[idx - 1], (int)deltaSoc, obj->tableNacVoltage[idx]);

                idx = idx + 1;
        }

}

#define CreateNacVoltageTableVer1     (CreateNacVoltageTableVer0)

static VerFuncArguObjRtnNull CreateNacVoltageTable[] = {
        CreateNacVoltageTableVer0,
        CreateNacVoltageTableVer1,
        CreateNacVoltageTableVer2,
        _UPI_NULL_,
};

/**
 * @brief CreateNacTableVer0
 *
 *  Create NAC table
 *
 * @para  obj address of CapacityInternalDataType
 * @return  _UPI_NULL_
 */
void CreateNacTableVer0(CapacityInternalDataType *obj)
{
        _cap_u8_ idx;
        _cap_s32_ tmp32;
        _cap_s32_ valueInterpolate1;
        _cap_s32_ valueInterpolate2;
        _cap_s16_ temperature;

        idx = 0;
        temperature = GetBatteryTemperature(obj);
        while(idx < SOV_NUMS) {
                /// [AT-PM] : Temperature interpolation ; 01/25/2013
                if(TemperatureTable[obj->idxTemperature[INDEX_BOUNDARY_HIGH]] ==
                    TemperatureTable[obj->idxTemperature[INDEX_BOUNDARY_LOW]]) {
                        valueInterpolate1 = obj->info->ggbTable->CELL_NAC_TABLE[obj->idxTemperature[INDEX_BOUNDARY_LOW]]
                                            [obj->idxNacCRate[INDEX_BOUNDARY_LOW]]
                                            [idx];

                        valueInterpolate2 = obj->info->ggbTable->CELL_NAC_TABLE[obj->idxTemperature[INDEX_BOUNDARY_LOW]]
                                            [obj->idxNacCRate[INDEX_BOUNDARY_HIGH]]
                                            [idx];
                } else {
                        tmp32 = (_cap_s32_)obj->info->ggbTable->CELL_NAC_TABLE[obj->idxTemperature[INDEX_BOUNDARY_HIGH]]
                                [obj->idxNacCRate[INDEX_BOUNDARY_LOW]]
                                [idx];
                        tmp32 = tmp32 - obj->info->ggbTable->CELL_NAC_TABLE[obj->idxTemperature[INDEX_BOUNDARY_LOW]]
                                [obj->idxNacCRate[INDEX_BOUNDARY_LOW]]
                                [idx];
                        tmp32 = tmp32*(temperature - TemperatureTable[obj->idxTemperature[INDEX_BOUNDARY_LOW]]);
                        tmp32 = tmp32/
                                (TemperatureTable[obj->idxTemperature[INDEX_BOUNDARY_HIGH]] -
                                 TemperatureTable[obj->idxTemperature[INDEX_BOUNDARY_LOW]]);
                        valueInterpolate1 = tmp32 + obj->info->ggbTable->CELL_NAC_TABLE[obj->idxTemperature[INDEX_BOUNDARY_LOW]]
                                            [obj->idxNacCRate[INDEX_BOUNDARY_LOW]]
                                            [idx];

                        tmp32 = (_cap_s32_)obj->info->ggbTable->CELL_NAC_TABLE[obj->idxTemperature[INDEX_BOUNDARY_HIGH]]
                                [obj->idxNacCRate[INDEX_BOUNDARY_HIGH]]
                                [idx];
                        tmp32 = tmp32 - obj->info->ggbTable->CELL_NAC_TABLE[obj->idxTemperature[INDEX_BOUNDARY_LOW]]
                                [obj->idxNacCRate[INDEX_BOUNDARY_HIGH]]
                                [idx];
                        tmp32 = tmp32*(temperature - TemperatureTable[obj->idxTemperature[INDEX_BOUNDARY_LOW]]);
                        tmp32 = tmp32/
                                (TemperatureTable[obj->idxTemperature[INDEX_BOUNDARY_HIGH]] -
                                 TemperatureTable[obj->idxTemperature[INDEX_BOUNDARY_LOW]]);
                        valueInterpolate2 = tmp32 + obj->info->ggbTable->CELL_NAC_TABLE[obj->idxTemperature[INDEX_BOUNDARY_LOW]]
                                            [obj->idxNacCRate[INDEX_BOUNDARY_HIGH]]
                                            [idx];
                }

                /// [AT-PM] : CRate interpolation ; 01/25/2013
                if(CRateTable[obj->idxNacCRate[INDEX_BOUNDARY_HIGH]] == CRateTable[obj->idxNacCRate[INDEX_BOUNDARY_LOW]]) {
                        tmp32 = valueInterpolate1;
                } else {
                        tmp32 = valueInterpolate2 - valueInterpolate1;
                        tmp32 = tmp32*(obj->cRate - CRateTable[obj->idxNacCRate[INDEX_BOUNDARY_LOW]]);
                        tmp32 = tmp32/
                                (CRateTable[obj->idxNacCRate[INDEX_BOUNDARY_HIGH]] - CRateTable[obj->idxNacCRate[INDEX_BOUNDARY_LOW]]);
                        tmp32 = tmp32 + valueInterpolate1;
                }
                obj->tableNac[idx] = (_cap_s16_)tmp32;
                UG31_LOGD("[%s]: NAC Table Index = (%d-%d, %d-%d)\n", __func__,
                          obj->idxTemperature[INDEX_BOUNDARY_HIGH], obj->idxTemperature[INDEX_BOUNDARY_LOW],
                          obj->idxNacCRate[INDEX_BOUNDARY_HIGH], obj->idxNacCRate[INDEX_BOUNDARY_LOW]);
                UG31_LOGD("[%s]: NAC Table[%d] = %dmAh (%d,%d) (%d,%d)\n", __func__,
                          idx, obj->tableNac[idx],
                          obj->info->ggbTable->CELL_NAC_TABLE[obj->idxTemperature[INDEX_BOUNDARY_HIGH]][obj->idxNacCRate[INDEX_BOUNDARY_LOW]][idx],
                          obj->info->ggbTable->CELL_NAC_TABLE[obj->idxTemperature[INDEX_BOUNDARY_LOW]][obj->idxNacCRate[INDEX_BOUNDARY_LOW]][idx],
                          obj->info->ggbTable->CELL_NAC_TABLE[obj->idxTemperature[INDEX_BOUNDARY_HIGH]][obj->idxNacCRate[INDEX_BOUNDARY_HIGH]][idx],
                          obj->info->ggbTable->CELL_NAC_TABLE[obj->idxTemperature[INDEX_BOUNDARY_LOW]][obj->idxNacCRate[INDEX_BOUNDARY_HIGH]][idx]);
                idx = idx + 1;
        }
}

/**
 * @brief CreateNacTableVer0
 *
 *  Create NAC table
 *
 * @para  obj address of CapacityInternalDataType
 * @return  _UPI_NULL_
 */
void CreateNacTableVer1(CapacityInternalDataType *obj)
{
        _cap_u8_ idx;
        _cap_s32_ tmp32;
        _cap_u16_ base;

        /// [AT-PM] : Copy nac table from global variable ; 10/25/2013
        upi_memcpy((_cap_u8_ *)obj->tableNac, (_cap_u8_ *)obj->info->tableNac, sizeof(_cap_s16_)*SOV_NUMS);

        /// [AT-PM] : Get ratio ; 10/25/2013
        while(1) {
                idx = 1;
                while(idx < SOV_NUMS) {
                        if(obj->tableNac[idx] > 255) {
                                break;
                        }
                        idx = idx + 1;
                }

                if(idx >= SOV_NUMS) {
                        break;
                }

                idx = 1;
                while(idx < SOV_NUMS) {
                        obj->tableNac[idx] = obj->tableNac[idx]/2;
                        if(obj->tableNac[idx] <= 0) {
                                obj->tableNac[idx] = 1;
                        }
                        idx = idx + 1;
                }
        }

        /// [AT-PM] : Get base ; 10/25/2013
        base = 0;
        idx = 1;
        while(idx < SOV_NUMS) {
                base = base + obj->tableNac[idx];
                UG31_LOGD("[%s]: Ratio[%d] = %d - %d\n", __func__, idx, obj->tableNac[idx], base);
                idx = idx + 1;
        }

        /// [AT-PM] : Get capacity at each region ; 10/25/2013
        obj->tableNac[0] = 0;
        idx = 1;
        while(idx < SOV_NUMS) {
                tmp32 = (_cap_s32_)obj->info->ggbTable->CELL_NAC_TABLE[obj->idxTemperature[INDEX_BOUNDARY_LOW]][obj->idxNacCRate[INDEX_BOUNDARY_LOW]][0];
                tmp32 = tmp32*obj->tableNac[idx]/base;
                if(tmp32 <= 0) {
                        tmp32 = 1;
                }
                obj->tableNac[idx] = (_cap_s16_)tmp32;
                obj->tableNac[0] = obj->tableNac[0] + obj->tableNac[idx];
                UG31_LOGD("[%s]: Table[%d] = %d - %d\n", __func__, idx, obj->tableNac[idx], obj->tableNac[0]);
                idx = idx + 1;
        }
}

#define CreateNacTableVer2      (CreateNacTableVer1)

static VerFuncArguObjRtnNull CreateNacTable[] = {
        CreateNacTableVer0,
        CreateNacTableVer1,
        CreateNacTableVer2,
        _UPI_NULL_,
};

#define MAX_DELTA_PREDICT_RSOC    (3)
#define MIN_DELTA_PREDICT_RSOC    (-3)
#define MAX_DISQUALIFY_DELTA_RSOC (20)

/**
 * @brief FindNacRM
 *
 *  Find the RM from NAC table
 *
 * @para  obj address of CapacityInternalDataType
 * @return  _UPI_NULL_
 */
void FindNacRM(CapacityInternalDataType *obj, _cap_u16_ voltage)
{
        _cap_s32_ tmp32;
        _cap_s32_ weightCC;
        _cap_s32_ weightVolt;
        _cap_s32_ min;

        /// [AT-PM] : Calculate RM below current voltage region ; 03/13/2013
        tmp32 = SOV_NUMS - 1;
        min = 0;
        while(tmp32 > obj->idxNacVoltage[INDEX_BOUNDARY_LOW]) {
                min = min + obj->tableNac[tmp32];
                tmp32 = tmp32 - 1;
        }
        UG31_LOGD("[%s]: RM below voltage region %d = %d\n", __func__, obj->idxNacVoltage[INDEX_BOUNDARY_LOW], (int)min);

        /// [AT-PM] : Calculate RM for capacity parser ; 02/09/2013
        if(obj->tableNacVoltage[obj->idxNacVoltage[INDEX_BOUNDARY_LOW]] ==
            obj->tableNacVoltage[obj->idxNacVoltage[INDEX_BOUNDARY_HIGH]]) {
                tmp32 = 0;
        } else {
                tmp32 = (_cap_s32_)voltage;
                tmp32 = tmp32 - obj->tableNacVoltage[obj->idxNacVoltage[INDEX_BOUNDARY_LOW]];
                tmp32 = tmp32*
                        obj->tableNac[obj->idxNacVoltage[INDEX_BOUNDARY_LOW]]/
                        (obj->tableNacVoltage[obj->idxNacVoltage[INDEX_BOUNDARY_HIGH]] -
                         obj->tableNacVoltage[obj->idxNacVoltage[INDEX_BOUNDARY_LOW]]);
        }
        tmp32 = tmp32 + min;
        obj->rm = (_cap_u16_)tmp32;
        UG31_LOGD("[%s]: Predicted RM = %d\n", __func__, obj->rm);

        /// [AT-PM] : Calculate RM for FCC update ; 01/25/2013
        if(obj->info->status & CAP_STS_DSGCHARGE_INITED) {
                obj->fccRM = obj->rm;
                UG31_LOGD("[%s]: Predicted RM for FCC Update = %d\n", __func__, obj->fccRM);
        } else {
                tmp32 = obj->tableNac[0];
                tmp32 = tmp32 - (obj->info->preDsgCharge + obj->info->dsgCharge);
                if(tmp32 < 0) {
                        tmp32 = 0;
                }
                obj->fccRM = (_cap_u16_)tmp32;
                UG31_LOGD("[%s]: Predicted RM for FCC Update = %d - ( %d + %d ) = %d\n", __func__,
                          obj->tableNac[0], (int)obj->info->preDsgCharge, (int)obj->info->dsgCharge, obj->fccRM);
        }

        if(obj->info->status & CAP_STS_INIT_PROCEDURE) {
                UG31_LOGD("[%s]: Initial procedure no voltage weighting\n", __func__);
                return;
        }

        /// [AT-PM] : If predicted RSOC is 20% higher than current RSOC, no coulomb counter wetighting ; 01/06/2014
        weightCC = 0;
        weightVolt = 0;
        tmp32 = (_cap_s32_)CalculateRsoc(obj->rm, (_cap_u16_)obj->tableNac[0]);
        tmp32 = tmp32 - obj->info->rsoc;
        if(tmp32 >= MAX_DISQUALIFY_DELTA_RSOC) {
                UG31_LOGD("[%s]: Delta RSOC = %d(%d) out of range, no coulomb counter weighting\n", __func__, (int)tmp32, obj->info->rsoc);
        } else {
                /// [FC] : Add voltage weighting ; 06/27/2013
                weightVolt = obj->idxNacVoltage[INDEX_BOUNDARY_LOW];
                weightCC = (SOV_NUMS - 1) - obj->idxNacVoltage[INDEX_BOUNDARY_LOW];
                tmp32 = SOV_NUMS - 1;
                switch(obj->info->ggbParameter->NacLmdAdjustCfg & NAC_LMD_ADJUST_CFG_VOLTAGE_CC_WEIGHT) {
                case  NAC_LMD_ADJUST_CFG_VOLTAGE_CC_WEIGHT_1:
                        tmp32 = tmp32*tmp32;
                        weightCC = weightCC*weightCC;
                        break;
                case  NAC_LMD_ADJUST_CFG_VOLTAGE_CC_WEIGHT_2:
                        tmp32 = tmp32*tmp32*tmp32;
                        weightCC = weightCC*weightCC*weightCC;
                        break;
                case  NAC_LMD_ADJUST_CFG_VOLTAGE_CC_WEIGHT_3:
                        tmp32 = tmp32*tmp32*tmp32*tmp32;
                        weightCC = weightCC*weightCC*weightCC*weightCC;
                        break;
                case  NAC_LMD_ADJUST_CFG_VOLTAGE_CC_WEIGHT_4:
                        tmp32 = tmp32*tmp32*tmp32*tmp32*tmp32;
                        weightCC = weightCC*weightCC*weightCC*weightCC*weightCC;
                        break;
                default:
                        tmp32 = tmp32*tmp32;
                        weightCC = weightCC*weightCC;
                        break;
                }
                weightVolt = tmp32 - weightCC;
                UG31_LOGD("[%s]: CC Weight = %d, Voltage Weight = %d, Base = %d\n", __func__, (int)weightCC, (int)weightVolt, (int)tmp32);
                weightCC = weightCC*obj->fccRM;
                weightVolt = weightVolt*obj->rm;
                tmp32 = (weightCC + weightVolt)/tmp32;
                obj->rm = (_cap_u16_)tmp32;
        }
        UG31_LOGN("[%s]: Target Predicted RM = %d (%d,%d)\n", __func__, obj->rm, (int)weightCC, (int)weightVolt);

        /// [AT-PM] : Check predicted RSOC and current RSOC ; 04/02/2013
        tmp32 = (_cap_s32_)CalculateRsoc(obj->rm, (_cap_u16_)obj->tableNac[0]);
        tmp32 = tmp32 - obj->info->rsoc;
        if((tmp32 <= MAX_DELTA_PREDICT_RSOC) && (tmp32 >= MIN_DELTA_PREDICT_RSOC)) {
                tmp32 = (_cap_s32_)CalculateRsoc(obj->rm, (_cap_u16_)obj->tableNac[0]);
                tmp32 = tmp32*obj->info->fcc/CONST_PERCENTAGE;
                obj->rm = (_cap_u16_)tmp32;
                UG31_LOGN("[%s]: Target Predicted RM with RSOC = %d\n", __func__, obj->rm);
        }
}

#define AVG_CRATE_MINIMUM_VALUE     (1)

/**
 * @brief CalculateCRate
 *
 *  Calculate C-Rate
 *
 * @para  obj address of CapacityInternalDataType
 * @return  _UPI_NULL_
 */
void CalculateCRate(CapacityInternalDataType *obj)
{
        _cap_s32_ cRate;

        /// [AT-PM] : Calculate CRate ; 01/25/2013
        cRate = (_cap_s32_)obj->info->measurement->currAvg;
        cRate = cRate*C_RATE_CONVERT_BASE/obj->info->ggbParameter->ILMD;
        obj->cRate = (_cap_u8_)cRate;
        cRate = (cRate + obj->info->avgCRate)/2;
        if(cRate < AVG_CRATE_MINIMUM_VALUE) {
                cRate = AVG_CRATE_MINIMUM_VALUE;
        }
        obj->info->avgCRate = (_cap_u8_)cRate;
        UG31_LOGD("[%s]: C-Rate = %d (%d)\n", __func__, obj->cRate, obj->info->avgCRate);
}

#define INIT_CAP_PARSER_FULL_SOC      (100)

static _cap_u8_ const InitCapMap[] = {
        0,  0,  1,  1,  2,  2,  3,  3,  4,  4,      ///< 0% ~ 9%
        5,  5,  6,  6,  7,  7,  8,  8,  9,  9,      ///< 10% ~ 19%
        10, 10, 11, 11, 12, 12, 13, 13, 14, 14,     ///< 20% ~ 29%
        15, 15, 16, 16, 17, 17, 18, 18, 19, 19,     ///< 30% ~ 39%
        20, 20, 20, 20, 20, 20, 20, 20, 20, 20,     ///< 40% ~ 49%
        20, 20, 20, 20, 20, 19, 19, 18, 18, 17,     ///< 50% ~ 59%
        17, 16, 16, 15, 15, 14, 14, 13, 13, 12,     ///< 60% ~ 69%
        12, 11, 11, 10, 10, 9,  9,  8,  8,  7,      ///< 70% ~ 79%
        7,  6,  6,  5,  5,  4,  4,  3,  3,  2,      ///< 80% ~ 89%
        2,  2,  1,  1,  1,  0,  0,  0,  0,  0,      ///< 90% ~ 99%
        0,                                          ///< 100%
};

/**
 * @brief InitCapacityParser
 *
 *  Reduce the initial capacity value
 *
 * @para  obj address of CapacityInternalDataType
 * @return  _UPI_NULL_
 */
void InitCapacityParser(CapacityInternalDataType *obj)
{
        _cap_u8_ newRsoc;
        _cap_u32_ tmp32;

        if(obj->info->ggbParameter->NacLmdAdjustCfg & NAC_LMD_ADJUST_CFG_REMOVE_INIT_PARSER) {
                return;
        }

        newRsoc = obj->info->rsoc - InitCapMap[obj->info->rsoc];
        if(newRsoc == obj->info->rsoc) {
                return;
        }

        tmp32 = (_cap_u32_)newRsoc;
        tmp32 = tmp32*obj->info->fcc/CONST_PERCENTAGE;
        obj->info->rm = (_cap_u16_)tmp32;
        obj->info->rsoc = (_cap_u8_)CalculateRsoc(obj->info->rm, obj->info->fcc);
}

/**
 * @brief InitCharge
 *
 *  Initialize charge status according to temperature, voltage, and current
 *
 * @para  obj address of CapacityInternalDataType
 * @return  _UPI_NULL_
 */
void InitCharge(CapacityInternalDataType *obj)
{
        _cap_u16_ tmp16;

        /// [AT-PM] : Find temperature index ; 01/25/2013
        if(FindIdxTemperature[GET_CAP_ALGORITHM_VER(obj->info->ggbParameter->NacLmdAdjustCfg)] != _UPI_NULL_) {
                (*FindIdxTemperature[GET_CAP_ALGORITHM_VER(obj->info->ggbParameter->NacLmdAdjustCfg)])(obj, GetBatteryTemperature(obj));
        }

        switch(obj->info->status & CAP_STS_CURR_STATE) {
        case CAP_STS_CURR_STANDBY:
                UG31_LOGN("[%s]: (0x%08x) CAP_STS_CURR_STANDBY\n", __func__, (unsigned int)obj->info->status);
                /// [AT-PM] : Loop up OCV table ; 01/25/2013
                FindOcvIdxVoltage(obj, OCV_TABLE_IDX_STAND_ALONE, obj->info->measurement->bat1VoltageAvg);
                FindOcvFcc(obj);
                FindOcvRM(obj, OCV_TABLE_IDX_STAND_ALONE, obj->info->measurement->bat1VoltageAvg);
                obj->info->rm = obj->rm;
                break;
        case CAP_STS_CURR_CHG:
                UG31_LOGN("[%s]: (0x%08x) CAP_STS_CURR_CHG\n", __func__, (unsigned int)obj->info->status);
                /// [AT-PM] : Loop up charging table ; 01/25/2013
                tmp16 = obj->info->measurement->bat1VoltageAvg;
                tmp16 = tmp16 - obj->info->measurement->currAvg/CHARGE_VOLTAGE_CONST;
                FindOcvIdxVoltage(obj, OCV_TABLE_IDX_STAND_ALONE, tmp16);
                FindOcvFcc(obj);
                FindOcvRM(obj, OCV_TABLE_IDX_STAND_ALONE, tmp16);
                obj->info->rm = obj->rm;
                break;
        case CAP_STS_CURR_DSG:
                UG31_LOGN("[%s]: (0x%08x) CAP_STS_CURR_DSG\n", __func__, (unsigned int)obj->info->status);
                /// [AT-PM] : Loop up NAC table ; 01/25/2013
                CalculateCRate(obj);
                if(FindNacIdxCRate[GET_CAP_ALGORITHM_VER(obj->info->ggbParameter->NacLmdAdjustCfg)] != _UPI_NULL_) {
                        (*FindNacIdxCRate[GET_CAP_ALGORITHM_VER(obj->info->ggbParameter->NacLmdAdjustCfg)])(obj);
                }
                if(CreateNacVoltageTable[GET_CAP_ALGORITHM_VER(obj->info->ggbParameter->NacLmdAdjustCfg)] != _UPI_NULL_) {
                        (*CreateNacVoltageTable[GET_CAP_ALGORITHM_VER(obj->info->ggbParameter->NacLmdAdjustCfg)])(obj);
                }
                FindNacIdxVoltage(obj, (_cap_u16_)obj->info->measurement->bat1VoltageAvg);
                if(CreateNacTable[GET_CAP_ALGORITHM_VER(obj->info->ggbParameter->NacLmdAdjustCfg)] != _UPI_NULL_) {
                        (*CreateNacTable[GET_CAP_ALGORITHM_VER(obj->info->ggbParameter->NacLmdAdjustCfg)])(obj);
                }
                obj->fcc = obj->tableNac[0];
                obj->info->fcc = obj->tableNac[0];
                FindNacRM(obj, (_cap_u16_)obj->info->measurement->bat1VoltageAvg);
                obj->info->rm = obj->rm;
                break;
        default:
                UG31_LOGE("[%s]: (0x%08x) CAP_STS_UNKNOWN\n", __func__, (unsigned int)obj->info->status);
                /// [AT-PM] : Loop up OCV table ; 01/25/2013
                FindOcvIdxVoltage(obj, OCV_TABLE_IDX_STAND_ALONE, obj->info->measurement->bat1VoltageAvg);
                FindOcvFcc(obj);
                FindOcvRM(obj, OCV_TABLE_IDX_STAND_ALONE, obj->info->measurement->bat1VoltageAvg);
                obj->info->rm = obj->rm;
                break;
        }

        if(obj->info->rm > obj->info->fcc) {
                obj->info->rm = obj->info->fcc;
        }
        obj->info->rsoc = (_cap_u8_)CalculateRsoc(obj->info->rm, obj->info->fcc);
        InitCapacityParser(obj);
        obj->info->rsoc = (_cap_u8_)CalculateRsoc(obj->info->rm, obj->info->fcc);
        obj->info->fccBackup = obj->info->fcc;
        obj->info->fccBeforeChg = obj->info->fcc;
        obj->info->lastRsoc = obj->info->rsoc;
        obj->info->predictRsoc = obj->info->rsoc;
        UG31_LOGN("[%s]: Initial capacity %d / %d = %d\n", __func__,
                  obj->info->rm, obj->info->fcc, obj->info->rsoc);
}

/**
 * @brief CheckTableAvailable
 *
 *  Check table is available or not
 *
 * @para  data  address of CapacityInternalDataType
 * @return  _UPI_TRUE_ if data is available
 */
_cap_bool_ CheckTableAvailable(CapacityInternalDataType *data)
{
        _cap_u8_ idx;

        idx = 0;
        while(idx < (SOV_NUMS - 1)) {
                if(data->info->encriptTable[idx] == 0) {
                        return (_UPI_FALSE_);
                }
                idx = idx + 1;
        }
        return (_UPI_TRUE_);
}

#define CELL_TABLE_ENCRIPTION_RATIO     (1000)

/**
 * @brief EncriptTable
 *
 *  Encript table from data->info->ggbTable->CELL_NAC_TABLE to data->info->encriptTable
 *
 * @para  data  address of CapacityInternalDataType
 * @return  _UPI_NULL_
 */
void EncriptTable(CapacityInternalDataType *data)
{
        _cap_u8_ idxTemp;
        _cap_u8_ idxCRate;
        _cap_u8_ idxSov;
        _cap_u8_ idx;
        _cap_u16_ tmp;
        _cap_u16_ percentage[TEMPERATURE_NUMS][C_RATE_NUMS][SOV_NUMS-1];
        _cap_u32_ tmp32;

        idxTemp = 0;
        idxCRate = 0;
        idxSov = 0;
        while(idxTemp < TEMPERATURE_NUMS) {
                while(idxCRate < C_RATE_NUMS) {
                        while(idxSov < (SOV_NUMS - 1)) {
                                percentage[idxTemp][idxCRate][idxSov] = 0;
                                idxSov = idxSov + 1;
                        }
                        idxCRate = idxCRate + 1;
                }
                idxTemp = idxTemp + 1;
        }

        /// [FC] : Encript table by percentage ; 06/14/2013
        if(data->tableNac[0] != 0) {
                idx = 0;
                idxSov = 1;
                while(idxSov < SOV_NUMS) {
                        tmp32 = (_cap_u32_)data->tableNac[idxSov];
                        tmp32 = tmp32*CELL_TABLE_ENCRIPTION_RATIO*CONST_ROUNDING/data->tableNac[0];
                        tmp32 = (tmp32 + CONST_ROUNDING_5)/CONST_ROUNDING;
                        if(tmp32 < 1) {
                                tmp32 = 1;
                        }
                        data->info->encriptTable[idx] = (_cap_u8_)tmp32;
                        UG31_LOGD("[%s]: Encripted Value [%d] = %d\n", __func__,
                                  idx, data->info->encriptTable[idx]);
                        idxSov = idxSov + 1;
                        idx = idx + 1;
                }
                idxTemp = 0;
                while(idxTemp < TEMPERATURE_NUMS) {
                        idxCRate = 0;
                        while(idxCRate < C_RATE_NUMS) {
                                if(data->info->ggbTable->CELL_NAC_TABLE[idxTemp][idxCRate][0] < SOV_NUMS) {
                                        data->info->ggbTable->CELL_NAC_TABLE[idxTemp][idxCRate][0] = SOV_NUMS - 1;
                                }
                                data->info->encriptTable[idx] = data->info->ggbTable->CELL_NAC_TABLE[idxTemp][idxCRate][0] / 256;
                                UG31_LOGD("[%s]: Encripted Value [%d] = %d\n", __func__,
                                          idx, data->info->encriptTable[idx]);
                                data->info->encriptTable[idx+1] = data->info->ggbTable->CELL_NAC_TABLE[idxTemp][idxCRate][0] % 256;
                                UG31_LOGD("[%s]: Encripted Value [%d] = %d\n", __func__,
                                          idx+1, data->info->encriptTable[idx+1]);
                                idx = idx + 2;
                                idxCRate = idxCRate + 1;
                        }
                        idxTemp = idxTemp + 1;
                }
        } else {
                idx = SOV_NUMS - 1;
                idxTemp = 0;
                while(idxTemp < TEMPERATURE_NUMS) {
                        idxCRate = 0;
                        while(idxCRate < C_RATE_NUMS) {
                                idxSov = 1;
                                while(idxSov < SOV_NUMS) {
                                        tmp32 = (_cap_u32_)data->info->ggbTable->CELL_NAC_TABLE[idxTemp][idxCRate][idxSov];
                                        tmp32 = tmp32*1000*CONST_ROUNDING/data->info->ggbTable->CELL_NAC_TABLE[idxTemp][idxCRate][0];
                                        tmp32 = (tmp32 + CONST_ROUNDING_5)/CONST_ROUNDING;
                                        percentage[idxTemp][idxCRate][idxSov - 1] = (_cap_u16_)tmp32;
                                        UG31_LOGD("[%s]: Percentage =  Table[%d][%d][%d] * %d / %d = %d  * %d / %d = %d\n", __func__,
                                                  idxTemp, idxCRate, idxSov,
                                                  CELL_TABLE_ENCRIPTION_RATIO,
                                                  data->info->ggbTable->CELL_NAC_TABLE[idxTemp][idxCRate][0],
                                                  data->info->ggbTable->CELL_NAC_TABLE[idxTemp][idxCRate][idxSov],
                                                  CELL_TABLE_ENCRIPTION_RATIO,
                                                  data->info->ggbTable->CELL_NAC_TABLE[idxTemp][idxCRate][0],
                                                  percentage[idxTemp][idxCRate][idxSov - 1]);
                                        idxSov = idxSov + 1;
                                }
                                if(data->info->ggbTable->CELL_NAC_TABLE[idxTemp][idxCRate][0] < SOV_NUMS) {
                                        data->info->ggbTable->CELL_NAC_TABLE[idxTemp][idxCRate][0] = SOV_NUMS - 1;
                                }
                                data->info->encriptTable[idx] = data->info->ggbTable->CELL_NAC_TABLE[idxTemp][idxCRate][0] / 256;
                                UG31_LOGD("[%s]: Encripted Value [%d] = %d\n", __func__,
                                          idx, data->info->encriptTable[idx]);
                                data->info->encriptTable[idx+1] = data->info->ggbTable->CELL_NAC_TABLE[idxTemp][idxCRate][0] % 256;
                                UG31_LOGD("[%s]: Encripted Value [%d] = %d\n", __func__,
                                          idx+1, data->info->encriptTable[idx+1]);
                                idx = idx + 2;
                                idxCRate = idxCRate + 1;
                        }
                        idxTemp = idxTemp + 1;
                }
                idxSov = 1;
                while(idxSov < SOV_NUMS) {
                        idxTemp = 0;
                        tmp = 0;
                        while(idxTemp < TEMPERATURE_NUMS) {
                                idxCRate = 0;
                                while(idxCRate < C_RATE_NUMS) {
                                        tmp = tmp + percentage[idxTemp][idxCRate][idxSov-1];
                                        idxCRate = idxCRate + 1;
                                }
                                idxTemp = idxTemp + 1;
                        }
                        data->info->encriptTable[idxSov-1] = tmp / (TEMPERATURE_NUMS * C_RATE_NUMS);
                        if(data->info->encriptTable[idxSov-1] < 1) {
                                data->info->encriptTable[idxSov-1] = 1;
                        }
                        UG31_LOGD("[%s]: Encripted Value [%d] = %d\n", __func__,
                                  idxSov-1, data->info->encriptTable[idxSov-1]);
                        idxSov = idxSov + 1;
                }
        }
}

/**
 * @brief DecriptTable
 *
 *  Decript table from data->info->encriptTable to data->info->ggbTable->CELL_NAC_TABLE
 *
 * @para  data  address of CapacityInternalDataType
 * @return  _UPI_NULL_
 */
void DecriptTable(CapacityInternalDataType *data)
{
        _cap_u8_ idxTemp;
        _cap_u8_ idxCRate;
        _cap_u8_ idxSov;
        _cap_u8_ idx;
        _cap_s16_ tmp;
        _cap_u32_ tmp32;

        /// [AT-PM] : Decript table ; 12/17/2012
        idx = SOV_NUMS - 1;
        idxTemp = 0;
        while(idxTemp < TEMPERATURE_NUMS) {
                idxCRate = 0;
                while(idxCRate < C_RATE_NUMS) {
                        idxSov = 0;
                        data->info->ggbTable->CELL_NAC_TABLE[idxTemp][idxCRate][idxSov] = 0;
                        data->info->ggbTable->CELL_NAC_TABLE[idxTemp][idxCRate][0] =
                                data->info->encriptTable[idx] * 256 +  data->info->encriptTable[idx+1];
                        if(data->info->ggbTable->CELL_NAC_TABLE[idxTemp][idxCRate][0] < SOV_NUMS) {
                                data->info->ggbTable->CELL_NAC_TABLE[idxTemp][idxCRate][0] = SOV_NUMS - 1;
                        }
                        UG31_LOGD("[%s]: Table[%d][%d][%d] = %d\n", __func__,
                                  idxTemp, idxCRate, 0,
                                  data->info->ggbTable->CELL_NAC_TABLE[idxTemp][idxCRate][0]);
                        idxSov = 1;
                        tmp = 0;
                        while(idxSov < SOV_NUMS) {
                                tmp32 = (_cap_u32_)data->info->encriptTable[idxSov - 1];
                                tmp32 = tmp32*data->info->ggbTable->CELL_NAC_TABLE[idxTemp][idxCRate][0]*CONST_ROUNDING/CELL_TABLE_ENCRIPTION_RATIO;
                                tmp32 = (tmp32 + CONST_ROUNDING_5)/CONST_ROUNDING;
                                data->info->ggbTable->CELL_NAC_TABLE[idxTemp][idxCRate][idxSov] = (_upi_s16_)tmp32;
                                /// [FC] : Limit the minimum capacity to 1 ; 06/14/2013
                                if(data->info->ggbTable->CELL_NAC_TABLE[idxTemp][idxCRate][idxSov] < 1) {
                                        data->info->ggbTable->CELL_NAC_TABLE[idxTemp][idxCRate][idxSov] = 1;
                                }
                                tmp = tmp + data->info->ggbTable->CELL_NAC_TABLE[idxTemp][idxCRate][idxSov];
                                idxSov = idxSov + 1;
                        }
                        /// [FC] : Distribute insufficient or superfluous capacity ; 06/14/2013
                        tmp = data->info->ggbTable->CELL_NAC_TABLE[idxTemp][idxCRate][0] - tmp;
                        while(tmp) {
                                idxSov = 1;
                                while(idxSov < SOV_NUMS) {
                                        if(tmp > 0) {
                                                data->info->ggbTable->CELL_NAC_TABLE[idxTemp][idxCRate][idxSov] =
                                                        data->info->ggbTable->CELL_NAC_TABLE[idxTemp][idxCRate][idxSov] + 1;
                                                tmp = tmp - 1;
                                        } else {
                                                if(data->info->ggbTable->CELL_NAC_TABLE[idxTemp][idxCRate][idxSov] > 1) {
                                                        data->info->ggbTable->CELL_NAC_TABLE[idxTemp][idxCRate][idxSov] =
                                                                data->info->ggbTable->CELL_NAC_TABLE[idxTemp][idxCRate][idxSov] - 1;
                                                        tmp = tmp + 1;
                                                }
                                        }
                                        if(tmp == 0) {
                                                break;
                                        }
                                        idxSov = idxSov + 1;
                                }
                        }
                        idxSov = 1;
                        while(idxSov < SOV_NUMS) {
                                UG31_LOGD("[%s]: Table[%d][%d][%d] = ( Encript Value [%d] = %d ) x %d / %d = %d\n", __func__,
                                          idxTemp, idxCRate, idxSov, idxSov-1,
                                          data->info->encriptTable[idxSov-1],
                                          data->info->ggbTable->CELL_NAC_TABLE[idxTemp][idxCRate][0],
                                          CELL_TABLE_ENCRIPTION_RATIO,
                                          data->info->ggbTable->CELL_NAC_TABLE[idxTemp][idxCRate][idxSov]);
                                idxSov = idxSov + 1;
                        }
                        idx = idx + 2;
                        idxCRate = idxCRate + 1;
                }
                idxTemp = idxTemp + 1;
        }
}

#define PREPARE_NAC_TABLE_VER1_CONST      (10000)

/**
 * @brief PrepareNacTableVer1
 *
 *  Prepare NAC table for no learning feature
 *
 * @para  obj address of CapacityInternalDataType
 * @return  NULL
 */
void PrepareNacTableVer1(CapacityInternalDataType *obj)
{
        _cap_s32_ tmp32;
        _cap_u8_ idxSov;
        _cap_u8_ idxTemp;
        _cap_u8_ idxCRate;
        _cap_u8_ dataCnt;
        _cap_s32_ ratio;

        /// [AT-PM] : Find a suitable FCC for each curve ; 10/28/2013
        tmp32 = 0;
        dataCnt = 0;
        idxTemp = 0;
        while(idxTemp < TEMPERATURE_NUMS) {
                idxCRate = 0;
                while(idxCRate < C_RATE_NUMS) {
                        tmp32 = tmp32 + obj->info->ggbTable->CELL_NAC_TABLE[idxTemp][idxCRate][0];
                        dataCnt = dataCnt + 1;
                        idxCRate = idxCRate + 1;
                }
                idxTemp = idxTemp + 1;
        }
        tmp32 = tmp32/dataCnt;

        ratio = (_cap_s32_)obj->info->ggbParameter->ILMD;
        ratio = ratio*PREPARE_NAC_TABLE_VER1_CONST;
        ratio = ratio/tmp32;

        /// [AT-PM] : Prepare NAC table ; 10/28/2013
        obj->info->tableNac[0] = 0;

        idxSov = 1;
        while(idxSov < SOV_NUMS) {
                dataCnt = 0;
                tmp32 = 0;

                idxTemp = 0;
                while(idxTemp < TEMPERATURE_NUMS) {
                        idxCRate = 0;
                        while(idxCRate < C_RATE_NUMS) {
                                tmp32 = tmp32 + obj->info->ggbTable->CELL_NAC_TABLE[idxTemp][idxCRate][idxSov];
                                dataCnt = dataCnt + 1;
                                idxCRate = idxCRate + 1;
                        }

                        idxTemp = idxTemp + 1;
                }

                tmp32 = tmp32*ratio/PREPARE_NAC_TABLE_VER1_CONST/dataCnt;
                obj->info->tableNac[idxSov] = (_cap_s16_)tmp32;
                obj->info->tableNac[0] = obj->info->tableNac[0] + obj->info->tableNac[idxSov];
                UG31_LOGD("[%s]: tableNac[%d] = %d (%d)\n", __func__, idxSov, obj->info->tableNac[idxSov], obj->info->tableNac[0]);

                idxSov = idxSov + 1;
        }
}

#define PrepareNacTableVer0     (_UPI_NULL_)
#define PrepareNacTableVer2     (PrepareNacTableVer1)

static VerFuncArguObjRtnNull PrepareNacTable[] = {
        PrepareNacTableVer0,
        PrepareNacTableVer1,
        PrepareNacTableVer2,
        _UPI_NULL_,
};

/**
 * @brief SaveUpdateNacTable
 *
 *  Save updated NAC table to IC
 *
 * @para  data  address of CapacityInternalDataType
 * @return  _UPI_NULL_
 */
void SaveUpdateNacTable(CapacityInternalDataType *data)
{
        _cap_u8_ tmpTable[MAX_ENCRIPT_TABLE_SIZE];
        _sys_u8_ *ptr;

        ptr = (_sys_u8_ *)&tmpTable[0];
        UpiAllocateTableBuf((_sys_u8_ **)&ptr, (_sys_u8_ *)&data->info->tableSize);
        upi_memcpy(tmpTable, data->info->encriptTable, data->info->tableSize);

        /// [AT-PM] : Refresh NAC table ; 02/26/2013
        if(CreateNacTable[GET_CAP_ALGORITHM_VER(data->info->ggbParameter->NacLmdAdjustCfg)] != _UPI_NULL_) {
                (*CreateNacTable[GET_CAP_ALGORITHM_VER(data->info->ggbParameter->NacLmdAdjustCfg)])(data);
        }

        /// [AT-PM] : Encript table ; 01/31/2013
        EncriptTable(data);

        /// [AT-PM] : Decript table ; 01/31/2013
        DecriptTable(data);
        UpiFreeTableBuf((_sys_u8_ **)&tmpTable);

        /// [AT-PM] : Prepare NAC table ; 10/25/2013
        if(PrepareNacTable[GET_CAP_ALGORITHM_VER(data->info->ggbParameter->NacLmdAdjustCfg)] != _UPI_NULL_) {
                (*PrepareNacTable[GET_CAP_ALGORITHM_VER(data->info->ggbParameter->NacLmdAdjustCfg)])(data);
        }

        /// [AT-PM] : Refresh NAC table ; 02/26/2013
        if(CreateNacTable[GET_CAP_ALGORITHM_VER(data->info->ggbParameter->NacLmdAdjustCfg)] != _UPI_NULL_) {
                (*CreateNacTable[GET_CAP_ALGORITHM_VER(data->info->ggbParameter->NacLmdAdjustCfg)])(data);
        }
}

/**
 * @brief UpdateCCRecord
 *
 *  Update CC record array from coulomb counter
 *
 * @para  obj address of CapacityInternalDataType
 * @return  _UPI_NULL_
 */
void UpdateCCRecord(CapacityInternalDataType *obj)
{
        _cap_s32_ tmp32;
        _cap_u8_ idx;

        tmp32 = obj->info->dsgCharge - obj->info->dsgChargeStart;
        if((tmp32 >= 0) && (obj->info->tableUpdateIdx < SOV_NUMS)) {
                obj->info->ccRecord[obj->info->tableUpdateIdx] = (_cap_s16_)tmp32;
        }

        obj->info->ccRecord[0] = 0;
        idx = 1;
        while(idx < SOV_NUMS) {
                obj->info->ccRecord[0] = obj->info->ccRecord[0] + obj->info->ccRecord[idx];
                UG31_LOGD("[%s]: New CC Record[%d] = %d (%d)\n", __func__,
                          idx, obj->info->ccRecord[idx], obj->info->ccRecord[0]);

                idx = idx + 1;
        }
}

#ifdef  CAP_LOG_UPDATE_TABLE

/**
 * @brief DumpUpdateTable
 *
 *  Dump updated NAC table to file
 *
 * @para  obj address of Capacity InternalDataType
 * @return  NULL
 */
void DumpUpdateTable(CapacityInternalDataType *obj)
{
        struct file *fp;
        mm_segment_t oldFS;
        loff_t pos;
        size_t size;

        fp = _UPI_NULL_;
        fp = filp_open(CAP_LOG_UPDATE_TABLE_PATH, O_CREAT | O_APPEND | O_WRONLY, 0);
        if(IS_ERR(fp)) {
                UG31_LOGE("[%s]: Create table dump file fail\n", __func__);
                return;
        }

        /// [AT-PM] : Write data to file ; 03/25/2013
        oldFS = get_fs();
        set_fs(get_ds());
        pos = 0;
        size = vfs_write(fp, (char *)&obj->info->tableUpdateIdx, sizeof(obj->info->tableUpdateIdx), &pos);
        size = vfs_write(fp, (char *)obj->info->encriptTable, sizeof(obj->info->encriptTable), &pos);
        set_fs(oldFS);

        filp_close(fp, _UPI_NULL_);
}

#endif  ///< end of CAP_LOG_UPDATE_TABLE

/**
 * @brief UpdateProcedureVer0
 *
 *  Procedure for updating NAC table
 *
 * @para  obj address of CapacityInternalDataType
 * @return  NULL
 */
void UpdateProcedureVer0(CapacityInternalDataType *obj)
{
        _cap_s32_ offset;
        _cap_s32_ tmp32;
        _cap_u8_ idxTemperature;
        _cap_u8_ idxCRate;

        /// [AT-PM] : Update table using offset ; 02/26/2013
        offset = obj->info->dsgCharge - obj->info->dsgChargeStart - obj->tableNac[obj->info->tableUpdateIdx];
        UG31_LOGD("[%s]: Update Offset = (%d - %d) - %d = %d\n", __func__,
                  (int)obj->info->dsgCharge, (int)obj->info->dsgChargeStart, obj->tableNac[obj->info->tableUpdateIdx], (int)offset);

        /// [AT-PM] : Update table ; 02/26/2013
        idxTemperature = obj->idxTemperature[INDEX_BOUNDARY_HIGH];
        while(idxTemperature <= obj->idxTemperature[INDEX_BOUNDARY_LOW]) {
                idxCRate = obj->idxNacCRate[INDEX_BOUNDARY_HIGH];
                while(idxCRate <= obj->idxNacCRate[INDEX_BOUNDARY_LOW]) {
                        tmp32 = (_cap_s32_)obj->info->ggbTable->CELL_NAC_TABLE[idxTemperature][idxCRate][obj->info->tableUpdateIdx];
                        tmp32 = tmp32 + offset;
                        if(tmp32 < 1) {
                                tmp32 = 1;
                        }
                        UG31_LOGD("[%s]: Update NAC Table[%d][%d][%d] = %d -> %d\n", __func__,
                                  idxTemperature, idxCRate, obj->info->tableUpdateIdx,
                                  obj->info->ggbTable->CELL_NAC_TABLE[idxTemperature][idxCRate][obj->info->tableUpdateIdx], (int)tmp32);
                        obj->info->ggbTable->CELL_NAC_TABLE[idxTemperature][idxCRate][obj->info->tableUpdateIdx] = (_upi_s16_)tmp32;

                        tmp32 = offset + (_cap_s32_)obj->info->ggbTable->CELL_NAC_TABLE[idxTemperature][idxCRate][0];
                        if(tmp32 <= SOV_NUMS - 1) {
                                tmp32 = SOV_NUMS - 1;
                        }
                        obj->info->ggbTable->CELL_NAC_TABLE[idxTemperature][idxCRate][0] = (_upi_s16_)tmp32;

                        idxCRate = idxCRate + 1;
                }
                idxTemperature = idxTemperature + 1;
        }
}

/**
 * @brief UpdateProcedureVer1
 *
 *  Procedure for updating NAC table
 *
 * @para  obj address of CapacityInternalDataType
 * @return  NULL
 */
void UpdateProcedureVer1(CapacityInternalDataType *obj)
{
        _cap_s32_ offset;
        _cap_s32_ tmp32;

        /// [AT-PM] : Get offset to be updated ; 10/25/2013
        offset = obj->info->dsgCharge - obj->info->dsgChargeStart - obj->tableNac[obj->info->tableUpdateIdx];
        UG31_LOGD("[%s]: Update Offset = (%d - %d) - %d = %d\n", __func__,
                  (int)obj->info->dsgCharge, (int)obj->info->dsgChargeStart, obj->tableNac[obj->info->tableUpdateIdx], (int)offset);

        /// [AT-PM] : Update FCC table ; 10/25/2013
        tmp32 = (_cap_s32_)obj->info->ggbTable->CELL_NAC_TABLE[obj->idxTemperature[INDEX_BOUNDARY_LOW]][obj->idxNacCRate[INDEX_BOUNDARY_LOW]][0];
        tmp32 = tmp32 + offset;
        if(tmp32 < SOV_NUMS) {
                tmp32 = SOV_NUMS;
        }
        UG31_LOGD("[%s]: Update table FCC[%d][%d][%d] = %d (%d)\n", __func__, obj->idxTemperature[INDEX_BOUNDARY_LOW],
                  obj->idxNacCRate[INDEX_BOUNDARY_LOW], 0, (int)tmp32,
                  obj->info->ggbTable->CELL_NAC_TABLE[obj->idxTemperature[INDEX_BOUNDARY_LOW]][obj->idxNacCRate[INDEX_BOUNDARY_LOW]][0]);
        obj->info->ggbTable->CELL_NAC_TABLE[obj->idxTemperature[INDEX_BOUNDARY_LOW]][obj->idxNacCRate[INDEX_BOUNDARY_LOW]][0] = (_upi_s16_)tmp32;

        /// [AT-PM] : Update capacity in the region ; 10/25/2013
        tmp32 = (_cap_s32_)obj->info->tableNac[obj->info->tableUpdateIdx];
        tmp32 = tmp32 + offset;
        if(tmp32 < 1) {
                tmp32 = 1;
        }
        UG31_LOGD("[%s]: Update table NAC[%d] = %d (%d)\n", __func__, obj->info->tableUpdateIdx, (int)tmp32,
                  obj->info->tableNac[obj->info->tableUpdateIdx]);
        obj->info->tableNac[obj->info->tableUpdateIdx] = (_cap_s16_)tmp32;

        /// [AT-PM] : Update FCC of the region ; 10/25/2013
        tmp32 = (_cap_s32_)obj->info->tableNac[0];
        tmp32 = tmp32 + offset;
        if(tmp32 < SOV_NUMS) {
                tmp32 = SOV_NUMS;
        }
        UG31_LOGD("[%s]: Update table FCC = %d (%d)\n", __func__, (int)tmp32, obj->info->tableNac[0]);
        obj->info->tableNac[0] = (_cap_s16_)tmp32;
}

#define UpdateProcedureVer2     (_UPI_NULL_)

static VerFuncArguObjRtnNull UpdateProcedure[] = {
        UpdateProcedureVer0,
        UpdateProcedureVer1,
        UpdateProcedureVer2,
        _UPI_NULL_,
};

/**
 * @brief UpdateTable
 *
 *  Update table procedure
 *
 * @para  obj address of CapacityInternalDataType
 * @return  NULL
 */
void UpdateTable(CapacityInternalDataType *obj)
{
        _cap_s32_ tmp32;

        /// [AT-PM] : Check delta capacity ; 01/31/2013
        tmp32 = obj->info->dsgCharge - obj->info->dsgChargeStart;
        if(tmp32 < 0) {
                UG31_LOGE("[%s]: Check delta capacity fail (%d - %d < 0)\n",
                          __func__, (int)obj->info->dsgCharge, (int)obj->info->dsgChargeStart);
                return;
        }

        /// [AT-PM] : Check discharge time ; 07/18/2013
        if(obj->info->dsgChargeTime == 0) {
                UG31_LOGE("[%s]: Discharge time = 0\n", __func__);
                return;
        }

        /// [AT-PM] : Calculate average c-rate ; 01/31/2013
        tmp32 = tmp32*TIME_SEC_TO_HOUR/obj->info->dsgChargeTime;
        UG31_LOGD("[%s]: Average C-Rate tmp = %d x %d / %d\n", __func__, (int)tmp32, TIME_SEC_TO_HOUR, (int)obj->info->dsgChargeTime);
        tmp32 = tmp32*C_RATE_CONVERT_BASE*(-1)/obj->info->ggbParameter->ILMD;
        UG31_LOGD("[%s]: Average C-Rate = %d x %d x (-1) / %d\n", __func__, (int)tmp32, C_RATE_CONVERT_BASE, obj->info->ggbParameter->ILMD);
        obj->cRate = (_cap_u8_)tmp32;
        UG31_LOGD("[%s]: Average C-Rate = %d\n", __func__, obj->cRate);

        /// [AT-PM] : Refresh NAC and voltage table ; 01/31/2013
        if(FindNacIdxCRate[GET_CAP_ALGORITHM_VER(obj->info->ggbParameter->NacLmdAdjustCfg)] != _UPI_NULL_) {
                (*FindNacIdxCRate[GET_CAP_ALGORITHM_VER(obj->info->ggbParameter->NacLmdAdjustCfg)])(obj);
        }
        if(CreateNacVoltageTable[GET_CAP_ALGORITHM_VER(obj->info->ggbParameter->NacLmdAdjustCfg)] != _UPI_NULL_) {
                (*CreateNacVoltageTable[GET_CAP_ALGORITHM_VER(obj->info->ggbParameter->NacLmdAdjustCfg)])(obj);
        }
        FindNacIdxVoltage(obj, obj->info->avgVoltage);
        if(CreateNacTable[GET_CAP_ALGORITHM_VER(obj->info->ggbParameter->NacLmdAdjustCfg)] != _UPI_NULL_) {
                (*CreateNacTable[GET_CAP_ALGORITHM_VER(obj->info->ggbParameter->NacLmdAdjustCfg)])(obj);
        }

        /// [AT-PM] : Update NAC table ; 10/25/2013
        if(UpdateProcedure[GET_CAP_ALGORITHM_VER(obj->info->ggbParameter->NacLmdAdjustCfg)] != _UPI_NULL_) {
                (*UpdateProcedure[GET_CAP_ALGORITHM_VER(obj->info->ggbParameter->NacLmdAdjustCfg)])(obj);
        }

        /// [AT-PM] : Save updated NAC table ; 01/31/2013
        SaveUpdateNacTable(obj);
        return;
}

/**
 * @brief RsocFilterReset
 *
 *  Reset timer
 *
 * @para  obj address of CapacityInternalDataType
 * @return  NULL
 */
void RsocFilterReset(CapacityInternalDataType *obj)
{
        obj->info->status = obj->info->status & (~CAP_STS_FILTER_LOCK_OVER);
        obj->info->socTimeStep = 0;
        obj->info->socTimeStepOverCnt = 0;
        obj->info->socTimeFull = 0;
        obj->info->socTimeFullOverCnt = 0;
        obj->info->socTimeLock = 0;
}


/**
 * @brief GetCCRecordFromTable
 *
 *  Get CC record array element from NAC table
 *
 * @para  obj address of CapacityInternalDataType
 * @para  idxSov  SOV index
 * @para  idxCRate C-Rate index
 * @return  CC Record value
 */
_cap_s32_ GetCCRecordFromTable(CapacityInternalDataType *obj, _cap_u8_ idxSov, _cap_u8_ idxCRate)
{
        _cap_s32_ valueInterpolate;
        _cap_s16_ temperature;

        temperature = GetBatteryTemperature(obj);

        if(TemperatureTable[obj->idxTemperature[INDEX_BOUNDARY_HIGH]] ==
            TemperatureTable[obj->idxTemperature[INDEX_BOUNDARY_LOW]]) {
                valueInterpolate = (_cap_s32_)obj->info->ggbTable->CELL_NAC_TABLE[obj->idxTemperature[INDEX_BOUNDARY_LOW]]
                                   [idxCRate]
                                   [idxSov];
        } else {
                valueInterpolate = (_cap_s32_)obj->info->ggbTable->CELL_NAC_TABLE[obj->idxTemperature[INDEX_BOUNDARY_HIGH]]
                                   [idxCRate]
                                   [idxSov];
                valueInterpolate = valueInterpolate - obj->info->ggbTable->CELL_NAC_TABLE[obj->idxTemperature[INDEX_BOUNDARY_LOW]]
                                   [idxCRate]
                                   [idxSov];
                valueInterpolate = valueInterpolate*(temperature - TemperatureTable[obj->idxTemperature[INDEX_BOUNDARY_LOW]]);
                valueInterpolate = valueInterpolate/
                                   (TemperatureTable[obj->idxTemperature[INDEX_BOUNDARY_HIGH]] -
                                    TemperatureTable[obj->idxTemperature[INDEX_BOUNDARY_LOW]]);
                valueInterpolate = valueInterpolate +
                                   obj->info->ggbTable->CELL_NAC_TABLE[obj->idxTemperature[INDEX_BOUNDARY_LOW]]
                                   [idxCRate]
                                   [idxSov];
        }
        return (valueInterpolate);
}

/**
 * @brief InitCCRecord
 *
 *  Initialize coulomb counter record from NAC table
 *
 * @para  obj address of CapacityInternalDataType
 * @return  _UPI_NULL_
 */
void InitCCRecord(CapacityInternalDataType *obj)
{
        _cap_u8_ idxSov;
        _cap_u8_ idxCRate;
        _cap_s32_ tmp32;
        _cap_s32_ valueInterpolate;

        obj->info->ccRecord[0] = 0;

        idxSov = 1;
        while(idxSov < SOV_NUMS) {
                tmp32 = 0;

                idxCRate = 0;
                while(idxCRate < C_RATE_NUMS) {
                        valueInterpolate = GetCCRecordFromTable(obj, idxSov, idxCRate);
                        tmp32 = tmp32 + valueInterpolate;

                        idxCRate = idxCRate + 1;
                }

                tmp32 = tmp32/idxCRate;
                obj->info->ccRecord[idxSov] = (_cap_s16_)tmp32;
                obj->info->ccRecord[0] = obj->info->ccRecord[0] + obj->info->ccRecord[idxSov];
                UG31_LOGD("[%s]: CC Record[%d] = %d (%d)\n", __func__,
                          idxSov, obj->info->ccRecord[idxSov], obj->info->ccRecord[0]);

                idxSov = idxSov + 1;
        }
}


/**
 * @brief ResetUpdateNacTableVer2
 *
 *  Reset UpdateNacTable buffer for fixed EDV version
 *
 * @para  obj address of CapacityInternalDataType
 * @return  NULL
 */
void ResetUpdateNacTableVer2(CapacityInternalDataType *obj)
{
        upi_memset((_cap_u8_ *)&obj->info->tableNacUpdate[0], 0, (sizeof(_cap_s16_)*SOV_NUMS));
}

#define ResetUpdateNacTableVer0     (_UPI_NULL_)
#define ResetUpdateNacTableVer1     (_UPI_NULL_)

static VerFuncArguObjRtnNull ResetUpdateNacTable[] = {
        ResetUpdateNacTableVer0,
        ResetUpdateNacTableVer1,
        ResetUpdateNacTableVer2,
        _UPI_NULL_,
};


/// =============================================
/// Extern region
/// =============================================

/**
 * @brief UpiInitCapacity
 *
 *  Initial capacity algorithm
 *
 * @para  data  address of CapacityDataType
 * @return  _UPI_NULL_
 */
void UpiInitCapacity(CapacityDataType *data)
{
        CapacityInternalDataType *obj;
        _cap_u16_ idx;
        _cap_u8_ *ptr;

        UG31_LOGI("[%s]: %s\n", __func__, CAPACITY_VERSION);

        /// [AT-PM] : Initialize variables ; 01/25/2013
        data->status = CAP_STS_LAST_STANDBY | CAP_STS_CURR_STANDBY | CAP_STS_INIT_PROCEDURE;
        data->status = data->status & (~(CAP_STS_INIT_TIMER_PASS));
        data->tableUpdateIdx = SOV_NUMS;
        data->dsgChargeStart = (_cap_s32_)data->ggbParameter->ILMD*2;
        data->tableUpdateDisqTime = 0;
        data->standbyDsgRatio = 0;
        data->standbyMilliSec = 0;
        data->standbyHour = 0;

#ifdef  UG31XX_SHELL_ALGORITHM
        obj = (CapacityInternalDataType *)upi_malloc(sizeof(CapacityInternalDataType));
#else   ///< else of UG31XX_SHELL_ALGORITHM
        obj = &capData;
#endif  ///< end of UG31XX_SHELL_ALGORITHM

        idx = 0;
        ptr = (_cap_u8_ *)obj;
        while(idx < sizeof(CapacityInternalDataType)) {
                *(ptr + idx) = 0;
                idx = idx + 1;
        }

        obj->info = data;
        obj->rm = obj->info->rm;
        obj->fcc = obj->info->fcc;

        UG31_LOGN("[%s]: Temperature Table: %d / %d / %d / %d\n", __func__,
                  TemperatureTable[0], TemperatureTable[1], TemperatureTable[2], TemperatureTable[3]);
        UG31_LOGN("[%s]: CRate Table: %d / %d / %d / %d\n", __func__,
                  CRateTable[0], CRateTable[1], CRateTable[2], CRateTable[3]);
        UG31_LOGN("[%s]: Current status: %d mV / %d mA / %d 0.1oC / %d 0.1oC\n", __func__,
                  data->measurement->bat1VoltageAvg, data->measurement->currAvg, data->measurement->intTemperature, data->measurement->extTemperature);

        /// [AT-PM] : Get battery state ; 01/25/2013
        data->avgVoltage = (_cap_u16_)data->measurement->bat1VoltageAvg;
        data->avgTemperature = GetBatteryTemperature(obj);
        GetBatteryState(obj);
        /// [AT-PM] : Release full charge condition ; 01/25/2013
        FullChargeRelease(obj);
        /// [AT-PM] : Find out initial capacity ; 01/28/2013
        InitCharge(obj);
        /// [AT-PM] : Initialize record CC array ; 02/19/2013
        InitCCRecord(obj);
        /// [AT-PM] : Reset RSOC filter ; 02/07/2014
        RsocFilterReset(obj);

        data->status = data->status & (~CAP_STS_INIT_PROCEDURE);
#ifdef  UG31XX_SHELL_ALGORITHM
        upi_free(obj);
#endif  ///< end of UG31XX_SHELL_ALGORITHM
}


/**
 * @brief UpiTableCapacity
 *
 *  Look up capacity from table
 *
 * @para  data  address of CapacityDataType
 * @return  _UPI_NULL_
 */
void UpiTableCapacity(CapacityDataType *data)
{
        CapacityInternalDataType *obj;
        _cap_u16_ idx;
        _cap_u8_ *ptr;

#ifdef  UG31XX_SHELL_ALGORITHM
        obj = (CapacityInternalDataType *)upi_malloc(sizeof(CapacityInternalDataType));
#else   ///< else of UG31XX_SHELL_ALGORITHM
        obj = &capData;
#endif  ///< end of UG31XX_SHELL_ALGORITHM

        idx = 0;
        ptr = (_cap_u8_ *)obj;
        while(idx < sizeof(CapacityInternalDataType)) {
                *(ptr + idx) = 0;
                idx = idx + 1;
        }

        obj->info = data;
        obj->rm = obj->info->rm;
        obj->fcc = obj->info->fcc;

        /// [AT-PM] : Find out initial capacity ; 01/28/2013
        obj->info->status = obj->info->status | CAP_STS_INIT_PROCEDURE;
        InitCharge(obj);
        obj->info->status = obj->info->status & (~CAP_STS_INIT_PROCEDURE);
#ifdef  UG31XX_SHELL_ALGORITHM
        upi_free(obj);
#endif  ///< end of UG31XX_SHELL_ALGORITHM
}

/**
 * @brief UpiInitNacTable
 *
 *  Initialize NAC table
 *
 * @para  data  address of CapacityDataType
 * @return  _UPI_NULL_
 */
void UpiInitNacTable(CapacityDataType *data)
{
        CapacityInternalDataType *obj;
        _cap_bool_ rtn;
        _cap_u16_ idx;
        _cap_u8_ *ptr;

#ifdef  UG31XX_SHELL_ALGORITHM
        obj = (CapacityInternalDataType *)upi_malloc(sizeof(CapacityInternalDataType));
#else   ///< else of UG31XX_SHELL_ALGORITHM
        obj = &capData;
#endif  ///< end of UG31XX_SHELL_ALGORITHM

        idx = 0;
        ptr = (_cap_u8_ *)obj;
        while(idx < sizeof(CapacityInternalDataType)) {
                *(ptr + idx) = 0;
                idx = idx + 1;
        }

        obj->info = data;
        obj->rm = obj->info->rm;
        obj->fcc = obj->info->fcc;

        idx = 0;
        while(idx < ENCRIPT_TABLE_SIZE) {
                data->encriptBuf[idx] = data->encriptTable[idx];
                idx = idx + 1;
        }

        /// [AT-PM] : Check data from IC ; 01/31/2013
#ifdef  UG31XX_RESET_DATABASE
        rtn = _UPI_FALSE_;
#else   ///< else of UG31XX_RESET_DATABASE
        rtn = CheckTableAvailable(obj);
#endif  ///< end of UG31XX_RESET_DATABASE
        if(rtn == _UPI_FALSE_) {
                /// [AT-PM] : Encript table from GGB file ; 01/31/2013
                EncriptTable(obj);
        } else {
                /// [AT-PM] : Decript table ; 01/31/2013
                DecriptTable(obj);
        }

        /// [AT-PM] : Prepare NAC table ; 10/25/2013
        if(PrepareNacTable[GET_CAP_ALGORITHM_VER(obj->info->ggbParameter->NacLmdAdjustCfg)] != _UPI_NULL_) {
                (*PrepareNacTable[GET_CAP_ALGORITHM_VER(obj->info->ggbParameter->NacLmdAdjustCfg)])(obj);
        }
        /// [AT-PM] : Reset update NAC table buffer ; 11/05/2013
        if(ResetUpdateNacTable[GET_CAP_ALGORITHM_VER(obj->info->ggbParameter->NacLmdAdjustCfg)] != _UPI_NULL_) {
                (*ResetUpdateNacTable[GET_CAP_ALGORITHM_VER(obj->info->ggbParameter->NacLmdAdjustCfg)])(obj);
        }

#ifdef  UG31XX_SHELL_ALGORITHM
        upi_free(obj);
#endif  ///< end of UG31XX_SHELL_ALGORITHM
}

/**
 * @brief UpiSetChargerFull
 *
 *  Set charger full condition for capacity algorithm
 *
 * @para  data  address of CapacityDataType
 * @para  isFull  set _UPI_TRUE_ for full charge condition
 * @return  NULL
 */
void UpiSetChargerFull(CapacityDataType *data, _cap_bool_ isFull)
{
        CapacityInternalDataType *obj;

#ifdef  UG31XX_SHELL_ALGORITHM
        obj = (CapacityInternalDataType *)upi_malloc(sizeof(CapacityInternalDataType));
#else   ///< else of UG31XX_SHELL_ALGORITHM
        obj = &capData;
#endif  ///< end of UG31XX_SHELL_ALGORITHM
        obj->info = data;
        obj->rm = obj->info->rm;
        obj->fcc = obj->info->fcc;

        if(isFull == _UPI_FALSE_) {
                obj->info->status = obj->info->status & (~CAP_STS_CHARGER_FULL);
        } else {
                /// [AT-PM] : Set full charge state ; 01/25/2013
                FullChargeSet(obj);

                obj->info->status = obj->info->status | CAP_STS_CHARGER_FULL;
                obj->info->rm = obj->rm;
                obj->info->fcc = obj->fcc;
                obj->info->rsoc = FULL_CHARGE_RSOC;
                UG31_LOGD("[%s]: (0x%08x) %d mAh / %d mAh = %d %%\n", __func__,
                          (unsigned int)obj->info->status, obj->info->rm, obj->info->fcc, obj->info->rsoc);
        }

#ifdef  UG31XX_SHELL_ALGORITHM
        upi_free(obj);
#endif  ///< end of UG31XX_SHELL_ALGORITHM
}

/**
 * @brief UpiSetChargerFullStep
 *
 *  Set charger full with stepping RSOC
 *
 * @para  data  address of CapacityDataType
 * @para  orgData original capacity data
 * @return  NULL
 */
void UpiSetChargerFullStep(CapacityDataType *data, GG_BATTERY_INFO *orgData)
{
        data->rm = (_cap_u16_)orgData->NAC;
        data->fcc = (_cap_u16_)orgData->LMD;
        data->rsoc = (_cap_u8_)orgData->RSOC;
        data->status = data->status | CAP_STS_FORCE_STEP_TO_100;
}

/**
 * @brief UpiInitDsgCharge
 *
 *  Initialize data->dsgCharge value
 *
 * @para  data  address of CapacityDataType
 * @return  _UPI_NULL_
 */
void UpiInitDsgCharge(CapacityDataType *data)
{
        data->dsgCharge = 0;
        data->status = data->status | CAP_STS_DSGCHARGE_INITED;
        UG31_LOGD("[%s]: Initial discharge charge counter = %d\n", __func__, (int)data->dsgCharge);
}


/**
 * @brief UpiAdjustCCRecord
 *
 *  Adjust CCRecord according to FCC
 *
 * @para  data  address of CapacityDataType
 * @return  NULL
 */
void UpiAdjustCCRecord(CapacityDataType *data)
{
        _cap_u8_ idxSov;
        _cap_s16_ residual;
        _cap_s32_ tmpNewData;
        _cap_s32_ tmpBuf;

        idxSov = 1;
        residual = 0;
        while(idxSov < SOV_NUMS) {
                tmpBuf = (_cap_s32_)data->ccRecord[idxSov];
                tmpBuf = tmpBuf*data->fcc*CONST_ROUNDING/data->ccRecord[0] + residual;
                tmpNewData = tmpBuf/CONST_ROUNDING;
                UG31_LOGD("[%s]: CCRecord[%d] = %d (%d)\n", __func__, idxSov, (int)tmpNewData, data->ccRecord[idxSov]);
                data->ccRecord[idxSov] = (_cap_s16_)tmpNewData;
                residual = (_cap_s16_)(tmpBuf - tmpNewData*CONST_ROUNDING);
                idxSov = idxSov + 1;
        }
        data->ccRecord[0] = data->fcc;
        UG31_LOGD("[%s]: CCRecord[0] = %d\n", __func__, data->ccRecord[0]);
}

/**
 * @brief CalculateRsoc
 *
 *  RSOC = RM x 100 / FCC
 *
 * @para  rm  remaining capacity
 * @para  fcc full charged capacity
 * @return  relative state of charge
 */
_cap_u16_ CalculateRsoc(_cap_u32_ rm, _cap_u16_ fcc)
{
        _sys_u32_ tmp32;

        tmp32 = rm*CONST_PERCENTAGE*CONST_ROUNDING/fcc;
        tmp32 = tmp32 + CONST_ROUNDING_5;
        tmp32 = tmp32/CONST_ROUNDING;
        return ((_cap_u16_)tmp32);
}

/**
 * @brief UpiSetBoardOffsetKed
 *
 *  Set board offset has been calibrated
 *
 * @para  data  address of CapacityDataType
 * @return  NULL
 */
void UpiSetBoardOffsetKed(CapacityDataType *data)
{
        data->status = data->status | CAP_STS_BOARD_OFFSET_KED;
}

/**
 * @brief UpiSetFactoryBoardOffset
 *
 *  Set board offset is loaded from factory
 *
 * @para  data  address of CapacityDataType
 * @return  NULL
 */
void UpiSetFactoryBoardOffset(CapacityDataType *data)
{
        data->status = data->status | CAP_STS_NO_STANDBY_CAP_EST;
}


/**
 * Copyright @ 2013 uPI Semiconductor Corp. All right reserved.
 * The information, images, and/or data contained in this material is copyrighted by uPI
 * Semiconductor Corp., and may not be distributed, modified, reproduced in whole or in part
 * without the prior, written consent of uPI Semiconductor Corp.
 */
