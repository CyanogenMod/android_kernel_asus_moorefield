/**
 * Copyright @ 2013 uPI Semiconductor Corp. All right reserved.
 * The information, images, and/or data contained in this material is copyrighted by uPI
 * Semiconductor Corp., and may not be distributed, modified, reproduced in whole or in part
 * without the prior, written consent of uPI Semiconductor Corp.
 */

/// ===========================================
/// uG31xx_API.cpp
/// ===========================================

#include "stdafx.h"     //windows need this??
#include "uG31xx_API.h"

static _upi_bool_ Ug31BackupFileEnable;
static _upi_bool_ Ug31SaveDataEnable = _UPI_TRUE_;
/// [FC] : Add variable MPK_active for MPK intial ; 12/10/2013
_upi_bool_ MPK_active = _UPI_FALSE_;

#ifdef  uG31xx_BOOT_LOADER

static _upi_u8_ ug31_uboot_sts;

#endif  ///< end of uG31xx_BOOT_LOADER

#if defined (uG31xx_OS_WINDOWS)

#define UG31XX_API_VERSION      (_T("UG31XX API $Rev: 480 $"))

#else

#define UG31XX_API_VERSION      ("UG31XX API $Rev: 480 $")

#endif

/// ===========================================
/// uG31xx_API.cpp (VAR)
/// ===========================================

/* uPI ug31xx hardware control interface */
struct ug31xx_data {

        /// [AT-PM] : Following variables are used for uG31xx operation ; 11/01/2012
        _upi_u8_  totalCellNums;
        _upi_bool_ bFirstData;

        // Global variable
        CELL_TABLE      cellTable;     // data from .GGB file
        CELL_PARAMETER  cellParameter;  // data from .GGB file
        GG_BATTERY_INFO batteryInfo;
        GG_DEVICE_INFO  deviceInfo;
        GG_USER_REG     userReg;			//user register 0x00 ~0x10
        GG_USER2_REG	  user2Reg;		//user register 0x40 ~0x4f
        GG_TI_BQ27520   bq27520Cmd;

        OtpDataType       otpData;
        MeasDataType      measData;
        CapacityDataType  capData;
        SystemDataType    sysData;
        BackupDataType    backupData;

        _upi_u8_ EncriptTableStatus;
        _upi_u16_ PreviousITAve;
        _upi_u8_ Options;
};

/// ===========================================
/// End of uG31xx_API.cpp (VAR)
/// ===========================================

#ifndef uG31xx_BOOT_LOADER

/**
 * @brief upiGG_GetAlarmStatus
 *
 *  Get alarm status
 *
 * @para  pAlarmStatus  address of alarm status
 * @return  UG_READ_DEVICE_ALARM_SUCCESS if success
 */
GGSTATUS upiGG_GetAlarmStatus(char *pObj, _upi_u8_ *pAlarmStatus)
{
        struct ug31xx_data *pUg31xx;

        pUg31xx = (struct ug31xx_data *)pObj;

        UpiMeasAlarmThreshold(&pUg31xx->measData);
        *pAlarmStatus = UpiAlarmStatus(&pUg31xx->sysData);

        pUg31xx->userReg.regAlarm1Status = (_upi_u8_)(pUg31xx->sysData.alarmSts & 0x00ff);
        pUg31xx->userReg.regAlarm2Status = (_upi_u8_)(pUg31xx->sysData.alarmSts >> 8);

        return (UG_READ_DEVICE_ALARM_SUCCESS);
}

// Read GG_USER_REG from device to global variable and output
GGSTATUS upiGG_ReadAllRegister(char *pObj, GG_USER_REG* pUserReg, GG_USER2_REG* pUser2Reg)
{
        struct ug31xx_data *pUg31xx;

        pUg31xx = (struct ug31xx_data *)pObj;

        if(!API_I2C_Read(NORMAL,
                         UG31XX_I2C_HIGH_SPEED_MODE,
                         UG31XX_I2C_TEM_BITS_MODE,
                         REG_MODE,
                         sizeof(GG_USER_REG),
                         &pUg31xx->userReg.regMode)) {
                return (UG_READ_REG_FAIL);
        }
        if(!API_I2C_Read(NORMAL,
                         UG31XX_I2C_HIGH_SPEED_MODE,
                         UG31XX_I2C_TEM_BITS_MODE,
                         REG_VBAT2_LOW,
                         sizeof(GG_USER2_REG),
                         (_upi_u8_* )&pUg31xx->user2Reg.regVbat2)) {  //read
                return (UG_READ_REG_FAIL);
        }

        return (UG_READ_REG_SUCCESS);
}

#endif  ///< end of uG31xx_BOOT_LOADER

// 07/04/1022/Jacky
_upi_u16_ CalculateVoltageFromUserReg(struct ug31xx_data *pUg31xx, _upi_s16_ voltageAdcCode, _upi_s16_ curr, _upi_u16_ offsetR, _upi_u16_ deltaR)
{
        _upi_u16_ voltage_return;

        voltage_return = (_upi_u16_)voltageAdcCode;
        if(curr < 0) {
                voltage_return = voltage_return + offsetR*abs(curr)/1000 + deltaR;
        } else {
                voltage_return = voltage_return - offsetR*abs(curr)/1000 + deltaR;
        }
        return (voltage_return);
}

#ifdef  uG31xx_BOOT_LOADER

/**
 * @brief uboot_reset_full_charge
 *
 *  Reset full charge function
 *
 * @para  address of struct ug31xx_data
 * @return  NULL
 */
void uboot_reset_full_charge(struct ug31xx_data *obj)
{
        ug31_uboot_sts = ug31_uboot_sts & (~UPI_BOOT_STATUS_FC);

        obj->capData.tpTime = 0;
}

/**
 * @brief uboot_check_full_charge
 *
 *  Check full charge condition for uBoot
 *
 * @para  obj address of struct ug31xx_data
 * @return  NULL
 */
void uboot_check_full_charge(struct ug31xx_data *obj)
{
        _upi_u32_ tmp32;

        if(ug31_uboot_sts & UPI_BOOT_STATUS_FC) {
                return;
        }

        /// [AT-PM] : Check taper voltage ; 09/03/2013
        if(obj->measData.bat1Voltage < obj->cellParameter.TPVoltage) {
                uboot_reset_full_charge(obj);
                return;
        }

        /// [AT-PM] : Check taper current ; 09/03/2013
        if((obj->measData.curr < obj->cellParameter.standbyCurrent) ||
            (obj->measData.curr > obj->cellParameter.TPCurrent)) {
                uboot_reset_full_charge(obj);
                return;
        }

        /// [AT-PM] : Check taper time ; 09/03/2013
        obj->capData.tpTime = obj->capData.tpTime + obj->measData.deltaTime;
        tmp32 = (_upi_u32_)obj->cellParameter.TPTime;
        tmp32 = tmp32*TIME_MSEC_TO_SEC;
        if(obj->capData.tpTime < tmp32) {
                return;
        }

        /// [AT-PM] : Set full charge status ; 09/03/2013
        ug31_uboot_sts = ug31_uboot_sts | UPI_BOOT_STATUS_FC;
        obj->batteryInfo.NAC = obj->batteryInfo.LMD;
        obj->batteryInfo.RSOC = CONST_PERCENTAGE;
        obj->capData.tpTime = obj->cellParameter.TPTime;
}

/**
 * @brief uboot_check_capacity
 *
 *  Check capacity data
 *
 * @para  obj address of struct ug31xx_data
 * @return  NULL
 */
void uboot_check_capacity(struct ug31xx_data *obj)
{
        _upi_u32_ tmp32;

        /// [AT-PM] : Check full charge release condition ; 09/03/2013
        if(obj->batteryInfo.RSOC < CONST_PERCENTAGE) {
                ug31_uboot_sts = ug31_uboot_sts & (~UPI_BOOT_STATUS_FC);
        }

        /// [AT-PM] : Check RSOC < 100 before FC ; 09/03/2013
        if((obj->batteryInfo.RSOC >= CONST_PERCENTAGE) &&
            (!(ug31_uboot_sts & UPI_BOOT_STATUS_FC))) {
                obj->batteryInfo.RSOC = CONST_PERCENTAGE - 1;

                /// [AT-PM] : Recalculate RM ; 09/03/2013
                tmp32 = (_upi_u32_)obj->batteryInfo.LMD;
                tmp32 = tmp32*(obj->batteryInfo.RSOC)/CONST_PERCENTAGE;
                obj->batteryInfo.NAC = (_upi_u16_)tmp32;
        }

        /// [AT-PM] : Check RM <= FCC ; 09/03/2013
        if(obj->batteryInfo.NAC > obj->batteryInfo.LMD) {
                obj->batteryInfo.NAC = obj->batteryInfo.LMD;
        }
}

#endif  ///< end of uG31xx_BOOT_LOADER

#define MAX_DIFF_IN_IT_AVE_CODE     (1000)
#define MIN_DIFF_IN_IT_AVE_CODE     (-1000)

/**
 * @brief ChkITAveCode
 *
 *  Check IT AVE code, which should be continuous
 *
 * @para  pObj  address of struct ug31xx_data
 * @return  NULL
 */
void ChkITAveCode(struct ug31xx_data *pObj)
{
        _upi_s32_ tmp;

        if(pObj->PreviousITAve != 0) {
                tmp = (_upi_s32_)pObj->PreviousITAve;
                tmp = tmp - pObj->userReg.regITAve;
                if((tmp > MAX_DIFF_IN_IT_AVE_CODE) || (tmp < MIN_DIFF_IN_IT_AVE_CODE)) {
                        UG31_LOGE("[%s]: IT AVE Code abnormal -> %d/%d\n", __func__, pObj->userReg.regITAve, pObj->PreviousITAve);
                        pObj->userReg.regITAve = pObj->PreviousITAve;
                }
        }
        pObj->PreviousITAve = pObj->userReg.regITAve;
}

/**
 * @brief CheckOtpData
 *
 *  Check OTP data
 *  1. Check OTP is empty or not
 *  2. Check product type
 *
 * @para  pObj  address of OtpDataType
 * @return  GGSTATUS
 */
GGSTATUS CheckOtpData(OtpDataType *pObj)
{
        /// [AT-PM] : Check OTP is empty or not ; 01/25/2013
        if(pObj->empty == OTP_IS_EMPTY) {
                return (UG_OTP_ISEMPTY);
        }

        /// [AT-PM] : Check product type ; 01/25/2013
        if(pObj->productType != UG31XX_PRODUCT_TYPE_0) {
                return (UG_OTP_PRODUCT_DISMATCH);
        }
        return (UG_READ_DEVICE_INFO_SUCCESS);
}

// Read GG_USER_REG from device and calculate GG_DEVICE_INFO, then write to global variable and output
// TODO: offsetR and deltaR will input from .GGB in the future modify
GGSTATUS upiGG_ReadDeviceInfo(char *pObj, GG_DEVICE_INFO* pExtDeviceInfo)
{
        // Get current user register data
        GGSTATUS status = UG_READ_DEVICE_INFO_SUCCESS;
        struct ug31xx_data *pUg31xx;
        MEAS_RTN_CODE rtn;
#ifdef  uG31xx_BOOT_LOADER
        _upi_u8_ tmp;
#endif  ///< end of uG31xx_BOOT_LOADER

        pUg31xx = (struct ug31xx_data *)pObj;

#ifndef uG31xx_BOOT_LOADER

        if(!API_I2C_Read(NORMAL,
                         UG31XX_I2C_HIGH_SPEED_MODE,
                         UG31XX_I2C_TEM_BITS_MODE,
                         REG_MODE,
                         REG_AVE_RID_HIGH - REG_MODE + 1,
                         &pUg31xx->userReg.regMode)) {
                status = UG_READ_ADC_FAIL;
        } else {
                if(!API_I2C_Read(NORMAL,
                                 UG31XX_I2C_HIGH_SPEED_MODE,
                                 UG31XX_I2C_TEM_BITS_MODE,
                                 REG_INTR_STATUS,
                                 REG_CTRL2 - REG_INTR_STATUS + 1,
                                 &pUg31xx->userReg.regIntrStatus)) {
                        status = UG_READ_ADC_FAIL;
                } else {
                        if(!API_I2C_Read(NORMAL,
                                         UG31XX_I2C_HIGH_SPEED_MODE,
                                         UG31XX_I2C_TEM_BITS_MODE,
                                         REG_VBAT2_LOW,
                                         sizeof(GG_USER2_REG),
                                         (_upi_u8_* )&pUg31xx->user2Reg.regVbat2)) {  //read
                                status = UG_READ_ADC_FAIL;
                        }
                }
        }

        /// [AT-PM] : Check IT AVE code, which should be continuous ; 12/28/2012
        ChkITAveCode(pUg31xx);

#endif  ///< end of uG31xx_BOOT_LOADER

        status = CheckOtpData(&pUg31xx->otpData);
        if(status != UG_READ_DEVICE_INFO_SUCCESS) {
                return (status);
        }

        pUg31xx->measData.sysData = &pUg31xx->sysData;
        pUg31xx->measData.otp = &pUg31xx->otpData;
#ifdef  uG31xx_BOOT_LOADER

        pUg31xx->measData.lastTimeTick = DEFAULT_TIME_TICK;

#endif  ///< end of uG31xx_BOOT_LOADER
        rtn = UpiMeasurement(&pUg31xx->measData, MEAS_SEL_ALL);
        if(rtn != MEAS_RTN_PASS) {
                return ((GGSTATUS)(rtn + UG_MEAS_FAIL));
        }
        pUg31xx->sysData.cycleCount = (_sys_u16_)pUg31xx->measData.cycleCount;

        pUg31xx->deviceInfo.chargeRegister = pUg31xx->userReg.regCharge;					//coulomb counter
        pUg31xx->deviceInfo.AdcCounter = pUg31xx->userReg.regCounter;						//adc1 convert counter
        pUg31xx->deviceInfo.aveCurrentRegister = pUg31xx->userReg.regCurrentAve;    //2012/07/11
        pUg31xx->deviceInfo.current_mA = pUg31xx->measData.curr;
        pUg31xx->deviceInfo.AveCurrent_mA = pUg31xx->measData.curr;
        pUg31xx->deviceInfo.IT = pUg31xx->measData.intTemperature;
        pUg31xx->deviceInfo.ET = pUg31xx->measData.extTemperature;
        pUg31xx->deviceInfo.v1_mV = pUg31xx->measData.bat1Voltage;
        pUg31xx->deviceInfo.vCell1_mV = pUg31xx->measData.bat1Voltage;
        pUg31xx->deviceInfo.vBat1Average_mV = CalculateVoltageFromUserReg(pUg31xx,
                                              pUg31xx->measData.bat1Voltage,
                                              pUg31xx->measData.curr,
                                              pUg31xx->cellParameter.offsetR,
                                              0);
        pUg31xx->deviceInfo.voltage_mV = pUg31xx->deviceInfo.vBat1Average_mV;
        pUg31xx->deviceInfo.chargeData_mAh = pUg31xx->measData.deltaCap;

        pUg31xx->sysData.otpData = &pUg31xx->otpData;
        UpiCalculateOscFreq(&pUg31xx->sysData);

        UpiAdcStatus(&pUg31xx->sysData);

        upi_memcpy(pExtDeviceInfo, &pUg31xx->deviceInfo, sizeof(GG_DEVICE_INFO));

#ifdef  uG31xx_BOOT_LOADER

        /// [AT-PM] : Update capacity ; 09/03/2013
        tmp = (_upi_s32_)pUg31xx->measData.curr;
        tmp = tmp*(pUg31xx->measData.stepCap);
        if(tmp < 0) {
                pUg31xx->measData.stepCap = 0;
        }
        pUg31xx->sysData.voltage = (_sys_u16_)pUg31xx->measData.bat1Voltage;
        pUg31xx->sysData.curr = (_sys_s16_)pUg31xx->measData.curr;
        UpiUpdateBatInfoFromIC(&pUg31xx->sysData, (_sys_s16_)pUg31xx->measData.stepCap);

        pUg31xx->batteryInfo.NAC = pUg31xx->sysData.rmFromIC;
        pUg31xx->batteryInfo.LMD = pUg31xx->sysData.fccFromIC;
        pUg31xx->batteryInfo.RSOC = pUg31xx->sysData.rsocFromIC;

        /// [AT-PM] : Reset coulomb counter if necessary ; 09/03/2013
        tmp = (_upi_s32_)pUg31xx->measData.deltaCap;
        if(tmp < 0) {
                tmp = tmp*(-1);
        }
        if(tmp > RESET_COULOMB_COUNTER_DELTA_CAP) {
                UpiResetCoulombCounter(&pUg31xx->measData);
                pUg31xx->sysData.cycleCount = (_sys_u16_)pUg31xx->measData.cycleCount;
        }

        /// [AT-PM] : Check full charge termination ; 09/03/2013
        uboot_check_full_charge(pUg31xx);

        /// [AT-PM] : Check capacity data ; 09/03/2013
        uboot_check_capacity(pUg31xx);

        /// [AT-PM] : Save data back to IC ; 09/03/2013
        if(ug31_uboot_sts & (UPI_BOOT_STATUS_FCC_IS_0 | UPI_BOOT_STATUS_IC_IS_NOT_ACTIVE)) {
                pUg31xx->sysData.fccFromIC = 0;
        }
        UpiSaveBatInfoTOIC(&pUg31xx->sysData);
        pUg31xx->sysData.fccFromIC = pUg31xx->batteryInfo.LMD;

#endif  ///< end of uG31xx_BOOT_LOADER

        return (status);
}

#ifndef uG31xx_BOOT_LOADER

void dumpInfo(struct ug31xx_data *pUg31xx)
{
        int i=0;
        int j;
        int k;

/// dump parameter setting
        UG31_LOGD("/// 2012/12/16/1611====================================\n");
        UG31_LOGD("/// CELL_PARAMETER\n");
        UG31_LOGD("/// ====================================2012/12/16/1611\n");
        UG31_LOGD("Total struct size: %d\n", pUg31xx->cellParameter.totalSize);
        UG31_LOGD("firmware version: 0x%02X\n", pUg31xx->cellParameter.fw_ver);
        UG31_LOGI("customer: %s\n", pUg31xx->cellParameter.customer);
        UG31_LOGI("project: %s\n", pUg31xx->cellParameter.project);
        UG31_LOGD("ggb version: 0x%02X\n", pUg31xx->cellParameter.ggb_version);
        UG31_LOGE("customer self-define: %s\n", pUg31xx->cellParameter.customerSelfDef);
        UG31_LOGE("project self-define: %s\n", pUg31xx->cellParameter.projectSelfDef);
        UG31_LOGD("cell type : 0x%04X\n", pUg31xx->cellParameter.cell_type_code);
        UG31_LOGD("ICType: 0x%02X\n", pUg31xx->cellParameter.ICType);
        UG31_LOGD("gpio1: 0x%02X\n", pUg31xx->cellParameter.gpio1);
        UG31_LOGD("gpio2: 0x%02X\n", pUg31xx->cellParameter.gpio2);
        UG31_LOGD("gpio34: 0x%02X\n", pUg31xx->cellParameter.gpio34);
        UG31_LOGD("Chop control ?? : 0x%02X\n", pUg31xx->cellParameter.chopCtrl);
        UG31_LOGD("ADC1 offset ?? : %d\n", pUg31xx->cellParameter.adc1Offset);
        UG31_LOGD("Cell number ?? : %d\n", pUg31xx->cellParameter.cellNumber);
        UG31_LOGD("Assign cell one to: %d\n", pUg31xx->cellParameter.assignCellOneTo);
        UG31_LOGD("Assign cell two to: %d\n", pUg31xx->cellParameter.assignCellTwoTo);
        UG31_LOGD("Assign cell three to: %d\n", pUg31xx->cellParameter.assignCellThreeTo);
        UG31_LOGD("I2C Address: : 0x%02X\n", pUg31xx->cellParameter.i2cAddress);
        UG31_LOGD("I2C 10bit address: : 0x%02X\n", pUg31xx->cellParameter.tenBitAddressMode);
        UG31_LOGD("I2C high speed: 0x%02X\n", pUg31xx->cellParameter.highSpeedMode);
        UG31_LOGD("clock(kHz): %d\n", pUg31xx->cellParameter.clock);
        UG31_LOGI("RSense(m ohm): %d\n", pUg31xx->cellParameter.rSense);
        UG31_LOGI("ILMD(mAH) ?? : %d\n", pUg31xx->cellParameter.ILMD);
        UG31_LOGI("EDV1 Voltage(mV): %d\n", pUg31xx->cellParameter.edv1Voltage);
        UG31_LOGI("Standby current ?? : %d\n", pUg31xx->cellParameter.standbyCurrent);
        UG31_LOGI("TP Current(mA)?? : %d\n", pUg31xx->cellParameter.TPCurrent);
        UG31_LOGI("TP Voltage(mV)?? : %d\n", pUg31xx->cellParameter.TPVoltage);
        UG31_LOGI("TP Time ?? : %d\n", pUg31xx->cellParameter.TPTime);
        UG31_LOGI("Offset R ?? : %d\n", pUg31xx->cellParameter.offsetR);
        UG31_LOGD("Delta R ?? : %d\n", pUg31xx->cellParameter.deltaR);
        UG31_LOGD("max delta Q(%%)  ?? : %d\n", pUg31xx->cellParameter.maxDeltaQ);
        UG31_LOGD("TP Bypass Current ?? : %d\n", pUg31xx->cellParameter.TpBypassCurrent);    //20121029/Jacky
        UG31_LOGD("time interval (s) : %d\n", pUg31xx->cellParameter.timeInterval);
        UG31_LOGI("ADC1 pgain: %d\n", pUg31xx->cellParameter.adc1_pgain);
        UG31_LOGI("ADC1 ngain: %d\n", pUg31xx->cellParameter.adc1_ngain);
        UG31_LOGI("ADC1 pos. offset: %d\n", pUg31xx->cellParameter.adc1_pos_offset);
        UG31_LOGI("ADC2 gain: %d\n", pUg31xx->cellParameter.adc2_gain);
        UG31_LOGI("ADC2 offset: %d\n", pUg31xx->cellParameter.adc2_offset);
        UG31_LOGI("R ?? : %d\n", pUg31xx->cellParameter.R);
        for (i=0; i<(int)(sizeof(pUg31xx->cellParameter.rtTable)/sizeof(_upi_u16_)); i++) {
                UG31_LOGI("RTTable[%02d]: %d\n", i, pUg31xx->cellParameter.rtTable[i]);
        }
        for (i=0; i<(int)(sizeof(pUg31xx->cellParameter.SOV_TABLE)/sizeof(_upi_u16_)); i++) {
                UG31_LOGD("SOV Table[%02d]: %d\n", i, pUg31xx->cellParameter.SOV_TABLE[i]/10);
        }
        UG31_LOGI("ADC d1: %d\n", pUg31xx->cellParameter.adc_d1);
        UG31_LOGI("ADC d2: %d\n", pUg31xx->cellParameter.adc_d2);
        UG31_LOGI("ADC d3: %d\n", pUg31xx->cellParameter.adc_d3);
        UG31_LOGI("ADC d4: %d\n", pUg31xx->cellParameter.adc_d4);
        UG31_LOGI("ADC d5: %d\n", pUg31xx->cellParameter.adc_d5);
        UG31_LOGE("NacLmdAdjustCfg: %d\n", pUg31xx->cellParameter.NacLmdAdjustCfg);    //20121124

        /// [AT-PM] : Dump NAC table ; 01/27/2013
        i = 0;
        while(i < TEMPERATURE_NUMS) {
                j = 0;
                while(j < C_RATE_NUMS) {
                        k = 0;
                        while(k < SOV_NUMS) {
                                UG31_LOGD("NAC Table [%d][%d][%d] = %d\n", i, j, k, pUg31xx->cellTable.CELL_NAC_TABLE[i][j][k]);
                                k = k + 1;
                        }
                        j = j + 1;
                }
                i = i + 1;
        }
}

/// count Time Elapsed in suspend/power Off
_upi_u32_ CountTotalTime(_upi_u32_ savedTimeTag)
{
        _upi_u32_ totalTime;
        _upi_u32_ currentTime;

        totalTime = 0;
#if defined(uG31xx_OS_ANDROID)
        currentTime = GetSysTickCount();
#else   ///< else of defined(uG31xx_OS_ANDROID)
#if defined(BUILD_UG31XX_LIB)
        currentTime = GetSysTickCount();
#else   ///< else of defined(BUILD_UG31XX_LIB)
        currentTime = GetTickCount();
#endif  ///< end of defined(BUILD_UG31XX_LIB)
#endif  ///< end of defined(uG31xx_OS_ANDROID)
        if(currentTime > savedTimeTag) {
                totalTime = currentTime - savedTimeTag;				//count the delta Time
        } else {
                totalTime = currentTime;
        }
        UG31_LOGI("[%s]current time/save Time/totalTime = %d/%d/%d \n",
                  __func__,
                  currentTime,
                  savedTimeTag,
                  totalTime
                 );
        return(totalTime);
}

#define MS_IN_A_DAY                             (86400000)
#define INIT_CAP_FROM_CC_FACTOR                 (10)

/**
 * @brief CheckInitCapacityFromCC
 *
 *  Check the initial capacity from coulomb counter with time interval
 *  The delta RSOC should be less than n days x 0.1%
 *
 * @para  pUg31xx address of struct ug31xx_data
 * @return  _UPI_NULL_
 */
void CheckInitCapacityFromCC(struct ug31xx_data *pUg31xx)
{
        _upi_s32_ tmp32;

        tmp32 = (_upi_s16_)pUg31xx->sysData.rsocFromIC;
        tmp32 = tmp32 - pUg31xx->sysData.rsocFromICBackup;
        if(tmp32 < 0) {
                tmp32 = (_upi_s32_)CountTotalTime(pUg31xx->sysData.timeTagFromIC)/MS_IN_A_DAY/INIT_CAP_FROM_CC_FACTOR;
                tmp32 = tmp32*(-1) + pUg31xx->sysData.rsocFromICBackup;
                if(tmp32 < 0) {
                        tmp32 = 1;
                }
                UG31_LOGN("[%s]: RSOC should be limited to %d (%d <-> %d)\n", __func__,
                          tmp32, pUg31xx->sysData.rsocFromICBackup, pUg31xx->sysData.rsocFromIC);
                pUg31xx->sysData.rsocFromIC = (_sys_u8_)tmp32;
                tmp32 = tmp32*pUg31xx->sysData.fccFromIC/CONST_PERCENTAGE;
                pUg31xx->sysData.rmFromIC = (_sys_u16_)tmp32;
        }
}

#define MAX_DELTA_RSOC_THRESHOLD_FOR_WAKEUP     (10)
#define MIN_DELTA_RSOC_THRESHOLD_FOR_WAKEUP     (-10)
#define MAX_DELTA_TIME_THRESHOLD_FOR_WAKEUP     (MS_IN_A_DAY*1)
#define MAX_DELTA_RSOC_THRESHOLD_FOR_TABLE      (10)
#define MIN_DELTA_RSOC_THRESHOLD_FOR_TABLE      (-10)

/**
 * @brief CmpCapData
 *
 *  Compare capacity data from coulomb counter and table
 *
 * @para  pUg31xx address of struct ug31xx_data
 * @para  initial   set _UPI_TRUE_ for upiGG_Initial procedure
 * @return  NULL
 */
void CmpCapData(struct ug31xx_data *pUg31xx, _upi_bool_ initial)
{
        _upi_s16_ deltaQC;
        _upi_s32_ tmp32;

        if(CountTotalTime(pUg31xx->sysData.timeTagFromIC) > MAX_DELTA_TIME_THRESHOLD_FOR_WAKEUP) {
                /// [AT-PM] : Check the data accuracy ; 01/27/2013
                deltaQC = (_upi_s16_)pUg31xx->sysData.rsocFromIC;
                deltaQC = deltaQC - pUg31xx->capData.rsoc;
                if((deltaQC > MAX_DELTA_RSOC_THRESHOLD_FOR_WAKEUP) || (deltaQC < MIN_DELTA_RSOC_THRESHOLD_FOR_WAKEUP)) {
                        /// [AT-PM] : Use data from table ; 07/17/2013
                        deltaQC = (_upi_s16_)pUg31xx->capData.rsoc;
                        deltaQC = deltaQC - pUg31xx->sysData.rsocFromICBackup;
                        if((deltaQC <= MAX_DELTA_RSOC_THRESHOLD_FOR_TABLE) && (deltaQC >= MIN_DELTA_RSOC_THRESHOLD_FOR_TABLE)) {
                                pUg31xx->capData.rsoc = (_cap_u8_)pUg31xx->sysData.rsocFromICBackup;

                        }
                        pUg31xx->capData.fcc = pUg31xx->sysData.fccFromIC;
                        tmp32 = (_upi_s32_)pUg31xx->capData.fcc;
                        tmp32 = tmp32*pUg31xx->capData.rsoc/CONST_PERCENTAGE;
                        pUg31xx->capData.rm = (_cap_u16_)tmp32;
                        UG31_LOGI("[%s]: Coulomb counter is not available -> Use data from table (%d/%d = %d)\n", __func__,
                                  pUg31xx->capData.rm, pUg31xx->capData.fcc, pUg31xx->capData.rsoc);
                } else {
                        if(initial == _UPI_TRUE_) {
                                CheckInitCapacityFromCC(pUg31xx);
                        }
                        pUg31xx->capData.rm = (_cap_u16_)pUg31xx->sysData.rmFromIC;
                        pUg31xx->capData.fcc = (_cap_u16_)pUg31xx->sysData.fccFromIC;
                        pUg31xx->capData.rsoc = (_cap_u8_)pUg31xx->sysData.rsocFromIC;
                        UG31_LOGI("[%s]: Use data from coulomb counter (%d/%d = %d)\n", __func__,
                                  pUg31xx->capData.rm, pUg31xx->capData.fcc, pUg31xx->capData.rsoc);
                }
        } else {
                if(initial == _UPI_TRUE_) {
                        CheckInitCapacityFromCC(pUg31xx);
                }
                pUg31xx->capData.rm = (_cap_u16_)pUg31xx->sysData.rmFromIC;
                pUg31xx->capData.fcc = (_cap_u16_)pUg31xx->sysData.fccFromIC;
                pUg31xx->capData.rsoc = (_cap_u8_)pUg31xx->sysData.rsocFromIC;
                UG31_LOGI("[%s]: Use data from coulomb counter (%d/%d = %d)\n", __func__,
                          pUg31xx->capData.rm, pUg31xx->capData.fcc, pUg31xx->capData.rsoc);
        }
}

#endif  ///< end of uG31xx_BOOT_LOADER

#ifdef  uG31xx_BOOT_LOADER

static _upi_s16_ temperature_table[] = {
        350,
        200,
        100,
        -1,
};

static _upi_u8_ ocv_soc_table[] = {
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

/**
 * @brief uboot_init_charge
 *
 *  Initialize capacity for uBoot
 *
 * @para  obj address of struct ug31xx_data
 * @return  NULL
 */
void uboot_init_charge(struct ug31xx_data *obj)
{
        _upi_s32_ volt;
        _upi_s32_ tmp32;
        _upi_u8_ idx_soc;
        _upi_u8_ idx_temp;

        /// [AT-PM] : FCC = ILMD ; 09/03/2013
        obj->batteryInfo.LMD = obj->cellParameter.ILMD;

        /// [AT-PM] : Find the real battery voltage ; 09/03/2013
        volt = (_upi_s32_)obj->measData.bat1Voltage;
        tmp32 = (_upi_s32_)obj->measData.curr;
        tmp32 = tmp32/CHARGE_VOLTAGE_CONST;
        if(tmp32 > 0) {
                volt = volt - tmp32;
        }

        /// [AT-PM] : Find the temperature region ; 09/03/2013
        idx_temp = 0;
        while(temperature_table[idx_temp] >= 0) {
                if(obj->measData.intTemperature > temperature_table[idx_temp]) {
                        break;
                }
                idx_temp = idx_temp + 1;
        }

        /// [AT-PM] : Look up OCV table ; 09/03/2013
        idx_soc = 0;
        while(ocv_soc_table[idx_soc] > 0) {
                if(volt >= obj->cellTable.INIT_OCV[idx_temp][OCV_TABLE_IDX_STAND_ALONE][idx_soc]) {
                        break;
                }
                idx_soc = idx_soc + 1;
        }

        /// [AT-PM] : Set RSOC ; 09/03/2013
        obj->batteryInfo.RSOC = (_upi_u16_)ocv_soc_table[idx_soc];

        /// [AT-PM] : Calculate RM ; 09/03/2013
        tmp32 = (_upi_s32_)obj->batteryInfo.LMD;
        tmp32 = tmp32*(obj->batteryInfo.RSOC)/CONST_PERCENTAGE;
        obj->batteryInfo.NAC = (_upi_u16_)tmp32;
}

#endif  ///< end of uG31xx_BOOT_LOADER

/**
 * @brief ChkResumeData
 *
 *  Check capacity data after resume if no dc in during suspend
 *
 * @para  pUg31xx address of struct ug31xx_data
 * @para  pOldBatInfo address of GG_BATTERY_INFO
 * @para  totalTime total suspend time in mSec
 * @para  deltaQC cumulative capacity from coulomb counter
 * @return  NULL
 */
void ChkResumeData(struct ug31xx_data *pUg31xx, GG_BATTERY_INFO *pOldBatInfo, _upi_u32_ totalTime, _upi_s16_ deltaQC)
{
        _upi_s32_ tmp32;

        tmp32 = (_upi_s32_)(totalTime/TIME_MSEC_TO_SEC);
        tmp32 = tmp32*(pUg31xx->cellParameter.standbyCurrent)/2/TIME_SEC_TO_HOUR*(-1);
        UG31_LOGN("[%s]: Estimated capacity = %d\n", __func__, tmp32);

        /// [AT-PM] : Check capacity has to be decreased ; 10/25/2013
        if(deltaQC > tmp32) {
                tmp32 = (tmp32 + deltaQC)/2;
                if(tmp32 > 0) {
                        tmp32 = 0;
                }
                tmp32 = tmp32 + pOldBatInfo->NAC;
                if(tmp32 < 0) {
                        tmp32 = 0;
                }
                UG31_LOGN("[%s]: Adjust capacity = %d -> %d\n", __func__, pOldBatInfo->NAC, tmp32);
                pOldBatInfo->NAC = (_upi_u16_)tmp32;

                pUg31xx->capData.rm = (_cap_u16_)pOldBatInfo->NAC;
                pUg31xx->capData.fcc = (_cap_u16_)pOldBatInfo->LMD;
                pUg31xx->capData.rsoc = (_cap_u8_)CalculateRsoc((_cap_u32_)pUg31xx->capData.rm, pUg31xx->capData.fcc);
                UG31_LOGN("[%s]: Capacity after resume = %d / %d = %d\n", __func__, pUg31xx->capData.rm, pUg31xx->capData.fcc, pUg31xx->capData.rsoc);
        }

        /// [AT-PM] : Check coulomb counter offset ; 10/25/2013
        if(pUg31xx->measData.codeChargeBeforeCal > 0) {
                UG31_LOGN("[%s]: Coulomb counter offset has to be adjusted (%d).\n", __func__, pUg31xx->measData.codeChargeBeforeCal);

                if(pUg31xx->measData.ccOffsetAdj <= pUg31xx->cellParameter.standbyCurrent) {
                        pUg31xx->measData.ccOffsetAdj = pUg31xx->measData.ccOffsetAdj + 1;
                        UG31_LOGN("[%s]: New offset = %d (%d).\n", __func__, pUg31xx->cellParameter.adc1_pos_offset, pUg31xx->measData.ccOffsetAdj);
                }
        }
}

// Read GGB file and initial
#ifdef uG31xx_OS_WINDOWS
GGSTATUS upiGG_Initial(char **pObj,const wchar_t* GGBFilename,const wchar_t* OtpFileName, unsigned char ForceReset)
#else
GGSTATUS upiGG_Initial(char **pObj, GGBX_FILE_HEADER *pGGBXBuf, unsigned char ForceReset)
#endif
{
        _upi_bool_ firstPowerOn;
        struct ug31xx_data *pUg31xx;
        SYSTEM_RTN_CODE rtn;
        _upi_s16_ deltaQC = 0;
        _upi_s32_ tmp32;
        MEAS_RTN_CODE rtnMeas;
        _sys_u8_ lastRsocFromIC;
        _upi_u32_ totalTime;
        GG_BATTERY_INFO batInfoBefore;
        _sys_u8_ *ptr;
#ifdef  __TEST_CHARGER_STOP_CHARGING__
        _upi_u16_ tmp16;
#endif  ///< end of __TEST_CHARGER_STOP_CHARGING__

        UG31_LOGE("[%s]: %s (%d-%s)\n", __func__, UG31XX_API_VERSION, UG31XX_DRIVER_VERSION, UG31XX_DRIVER_RELEASE_DATE);

        firstPowerOn = _UPI_FALSE_;
#ifdef  uG31xx_BOOT_LOADER

        ug31_uboot_sts = 0;

#endif  ///< end of uG31xx_BOOT_LOADER
        *pObj = (char *)upi_malloc(sizeof(struct ug31xx_data));
        pUg31xx = (struct ug31xx_data *)(*pObj);

        upi_memset(pUg31xx, 0, sizeof(struct ug31xx_data));

#ifdef uG31xx_OS_WINDOWS
        pUg31xx->sysData.ggbFilename = GGBFilename;
        pUg31xx->sysData.otpFileName = OtpFileName;
#else
        pUg31xx->sysData.ggbXBuf = pGGBXBuf;
#endif
        pUg31xx->sysData.ggbParameter = &pUg31xx->cellParameter;
        pUg31xx->sysData.ggbCellTable = &pUg31xx->cellTable;
        rtn = UpiInitSystemData(&pUg31xx->sysData);
        if(rtn != SYSTEM_RTN_PASS) {
                if(rtn == SYSTEM_RTN_READ_GGB_FAIL) {
                        return (UG_READ_GGB_FAIL);
                }
                return (UG_NOT_DEF);
        }

        // Initial I2C and Open HID
#if defined(uG31xx_OS_WINDOWS)

        if(!API_I2C_Init(pUg31xx->cellParameter.clock, pUg31xx->cellParameter.i2cAddress)) {
                return UG_I2C_INIT_FAIL;
        }

#endif  ///< end of defined(uG31xx_OS_WINDOWS)

        UpiLoadBatInfoFromIC(&pUg31xx->sysData);
        pUg31xx->measData.cycleCount = (ForceReset == 0) ? (_meas_u16_)pUg31xx->sysData.cycleCount : 0;
        pUg31xx->measData.ccOffsetAdj = (ForceReset == 0) ? (_meas_s8_)pUg31xx->sysData.ccOffset : 0;
        pUg31xx->measData.cumuCap = 0;
        pUg31xx->capData.standbyDsgRatio = (ForceReset == 0) ? (_cap_u8_)pUg31xx->sysData.standbyDsgRatio : 0;
        /// Count total Time
        totalTime = CountTotalTime(pUg31xx->sysData.timeTagFromIC);
        batInfoBefore.NAC = (_upi_u16_)pUg31xx->sysData.rmFromIC;
        batInfoBefore.LMD = (_upi_u16_)pUg31xx->sysData.fccFromIC;
        batInfoBefore.RSOC = (_upi_u16_)pUg31xx->sysData.rsocFromIC;

#ifdef  uG31xx_BOOT_LOADER

        if(pUg31xx->sysData.fccFromIC == 0) {
                ug31_uboot_sts = ug31_uboot_sts | UPI_BOOT_STATUS_FCC_IS_0;
        }

#else   ///< else of uG31xx_BOOT_LOADER

        // [FC] : Load table from IC ; 05/30/2013
        ptr = (_sys_u8_ *)&pUg31xx->capData.encriptTable[0];
        UpiAllocateTableBuf((_sys_u8_ **)&ptr, &pUg31xx->capData.tableSize);
        ptr = (_sys_u8_ *)&pUg31xx->capData.encriptBuf[0];
        UpiAllocateTableBuf((_sys_u8_ **)&ptr, &pUg31xx->capData.tableSize);
        if((Ug31SaveDataEnable == _UPI_TRUE_) && (ForceReset == 0)) {
                UpiLoadTableFromIC((_sys_u8_ *)pUg31xx->capData.encriptTable);
        } else {
                upi_memset(pUg31xx->capData.encriptTable, 0, pUg31xx->capData.tableSize);
        }
        pUg31xx->capData.ggbTable = &pUg31xx->cellTable;
        pUg31xx->capData.ggbParameter = &pUg31xx->cellParameter;
        pUg31xx->capData.measurement = &pUg31xx->measData;
        UpiInitNacTable(&pUg31xx->capData);

#endif  ///< end of uG31xx_BOOT_LOADER

        /// [AT-PM] : Check IC is active or not ; 01/28/2013
#ifdef  UG31XX_RESET_DATABASE

        firstPowerOn = _UPI_TRUE_;

#else   ///< else of UG31XX_RESET_DATABASE

        if(ForceReset == 0) {
                firstPowerOn = UpiCheckICActive();
        } else {
                firstPowerOn = _UPI_TRUE_;
        }

#endif  ///< end of UG31XX_RESET_DATABASE
        if(firstPowerOn == _UPI_TRUE_) {
#ifdef  uG31xx_BOOT_LOADER

                ug31_uboot_sts = ug31_uboot_sts | UPI_BOOT_STATUS_IC_IS_NOT_ACTIVE;

#endif  ///< end of uG31xx_BOOT_LOADER

                UG31_LOGE("[%s]#####firstPowerOn= %d \n",__func__,firstPowerOn);
                rtn = UpiActiveUg31xx();
                if(rtn != SYSTEM_RTN_PASS) {
                        return (UG_ACTIVE_FAIL);
                }

                UpiSetupAdc(&pUg31xx->sysData);
                UpiSetupSystem(&pUg31xx->sysData);

#ifdef  UG31XX_RESET_DATABASE
                pUg31xx->backupData.icDataAvailable = BACKUP_BOOL_TRUE;
                pUg31xx->backupData.backupFileSts = BACKUP_FILE_STS_NOT_EXIST;
#else   ///< else of UG31XX_RESET_DATABASE
#ifdef  UG31XX_CELL_REPLACE_TEST
                ForceReset = 0;
#endif  ///< end of UG31XX_CELL_REPLACE_TEST
                pUg31xx->backupData.icDataAvailable = (ForceReset == 0) ? BACKUP_BOOL_FALSE : BACKUP_BOOL_TRUE;
                pUg31xx->backupData.backupFileSts = (ForceReset == 0) ? BACKUP_FILE_STS_CHECKING : BACKUP_FILE_STS_NOT_EXIST;
#endif  ///< end of UG31XX_RESET_DATABASE
        } else {
#ifndef uG31xx_BOOT_LOADER

                UG31_LOGE("[%s]#####Last time tag = %d, NAC = %d, LMD = %d\n", __func__,
                          (int)pUg31xx->sysData.timeTagFromIC,
                          pUg31xx->sysData.rmFromIC,
                          pUg31xx->sysData.fccFromIC);
                pUg31xx->measData.lastTimeTick = GetTickCount();
                pUg31xx->measData.lastDeltaCap = pUg31xx->sysData.deltaCapFromIC;
                pUg31xx->measData.adc1ConvertTime = pUg31xx->sysData.adc1ConvTime;

#endif  ///< end of uG31xx_BOOT_LOADER

                pUg31xx->backupData.icDataAvailable = BACKUP_BOOL_TRUE;
        }

#ifndef uG31xx_BOOT_LOADER

        /// [FC] : Save table to IC ; 05/30/2013
        if(Ug31SaveDataEnable == _UPI_TRUE_) {
                UpiSaveTableToIC((_sys_u8_ *)pUg31xx->capData.encriptTable, (_sys_u8_ *)pUg31xx->capData.encriptBuf, (_sys_u8_)pUg31xx->capData.tableSize);
        }
#endif  ///< end of uG31xx_BOOT_LOADER

        /// [AT-PM] : Fetch ADC code for system stable ; 06/04/2013
#ifdef  __TEST_CHARGER_STOP_CHARGING__
        tmp16 = (_upi_u16_)Ug31DebugEnable;
        Ug31DebugEnable = 3;
        tmp32 = 100;
        while(tmp32) {
                UpiMeasReadCode(&pUg31xx->measData);
                SleepMiniSecond(125);
                tmp32 = tmp32 - 1;
        }
        Ug31DebugEnable = (_upi_u8_)tmp16;
#else   ///< else of __TEST_CHARGER_STOP_CHARGING__
        UpiMeasReadCode(&pUg31xx->measData);
#endif  ///< end of __TEST_CHARGER_STOP_CHARGING__

        /// [AT-PM] : Load OTP data ; 01/31/2013
        API_I2C_Read(SECURITY, UG31XX_I2C_HIGH_SPEED_MODE, UG31XX_I2C_TEM_BITS_MODE, OTP1_BYTE1, OTP1_SIZE, pUg31xx->otpData.otp1);
        API_I2C_Read(SECURITY, UG31XX_I2C_HIGH_SPEED_MODE, UG31XX_I2C_TEM_BITS_MODE, OTP2_BYTE1, OTP2_SIZE, pUg31xx->otpData.otp2);
        API_I2C_Read(NORMAL, UG31XX_I2C_HIGH_SPEED_MODE, UG31XX_I2C_TEM_BITS_MODE, OTP6_BYTE1, OTP3_SIZE, pUg31xx->otpData.otp3);
        UpiConvertOtp(&pUg31xx->otpData);

        /// [AT-PM] : Check product type ; 01/25/2013
        if(pUg31xx->otpData.productType != UG31XX_PRODUCT_TYPE_0) {
#ifdef  uG31xx_BOOT_LOADER

                ug31_uboot_sts = ug31_uboot_sts | UPI_BOOT_STATUS_WRONG_PRODUCT_TYPE;

#endif  ///< end of uG31xx_BOOT_LOADER
                return (UG_OTP_PRODUCT_DISMATCH);
        }

        UG31_LOGN("[%s]: Do measurement\n", __func__);

        pUg31xx->measData.sysData = &pUg31xx->sysData;
        pUg31xx->measData.otp = &pUg31xx->otpData;
        pUg31xx->measData.status = 0;
#ifdef  uG31xx_BOOT_LOADER

        if(ug31_uboot_sts & (UPI_BOOT_STATUS_IC_IS_NOT_ACTIVE | UPI_BOOT_STATUS_FCC_IS_0)) {
                pUg31xx->measData.lastDeltaCap = 0;
                pUg31xx->measData.adc1ConvertTime = TIME_DEFAULT_ADC1_CONVERT_TIME;
        } else {
                pUg31xx->measData.lastDeltaCap = pUg31xx->sysData.deltaCapFromIC;
                pUg31xx->measData.adc1ConvertTime = pUg31xx->sysData.adc1ConvTime;
        }
        pUg31xx->measData.lastTimeTick = DEFAULT_TIME_TICK;

#endif  ///< end of uG31xx_BOOT_LOADER
        rtnMeas = UpiMeasurement(&pUg31xx->measData, MEAS_SEL_INITIAL);
        if(rtnMeas != MEAS_RTN_PASS) {
                return ((GGSTATUS)(rtnMeas + UG_MEAS_FAIL));
        }
        pUg31xx->sysData.cycleCount = (_sys_u16_)pUg31xx->measData.cycleCount;

        /// [AT-PM] : No external temperature average ; 07/04/2013
        tmp32 = (_upi_s32_)pUg31xx->measData.extTemperature;
        tmp32 = tmp32*ET_AVERAGE_BASE/ET_AVERAGE_NEW;
        pUg31xx->measData.extTemperature = (_meas_s16_)tmp32;

        if(firstPowerOn == _UPI_TRUE_) {
                /// [AT-PM] : Recover ADC1 conversion queue ; 06/04/2013
                UpiSetupAdc1Queue(&pUg31xx->sysData);

                /// [AT-PM] : Initialize alarm function ; 04/08/2013
                UpiMeasAlarmThreshold(&pUg31xx->measData);
                UpiInitAlarm(&pUg31xx->sysData);
        }
        UG31_LOGI("[%s]: Current Status = %d mV / %d mA / %d-%d-%d 0.1oC\n", __func__,
                  pUg31xx->measData.bat1Voltage, pUg31xx->measData.curr, pUg31xx->measData.intTemperature, pUg31xx->measData.extTemperature, pUg31xx->measData.instExtTemperature);
        pUg31xx->measData.extTemperature = pUg31xx->measData.intTemperature;
        pUg31xx->measData.instExtTemperature = pUg31xx->measData.intTemperature;

#ifdef  uG31xx_BOOT_LOADER

        if(ug31_uboot_sts & (UPI_BOOT_STATUS_IC_IS_NOT_ACTIVE | UPI_BOOT_STATUS_FCC_IS_0)) {
                uboot_init_charge(pUg31xx);

                pUg31xx->sysData.rmFromIC = (_sys_u16_)pUg31xx->batteryInfo.NAC;
                pUg31xx->sysData.fccFromIC = (_sys_u16_)pUg31xx->batteryInfo.LMD;
                pUg31xx->sysData.rsocFromIC = (_sys_u16_)pUg31xx->batteryInfo.RSOC;
        } else {
                pUg31xx->batteryInfo.NAC = (_upi_u16_)pUg31xx->sysData.rmFromIC;
                pUg31xx->batteryInfo.LMD = (_upi_u16_)pUg31xx->sysData.fccFromIC;
                pUg31xx->batteryInfo.RSOC = (_upi_u8_)pUg31xx->sysData.rsocFromIC;
        }
        return (UG_INIT_SUCCESS);

#endif  ///< end of uG31xx_BOOT_LOADER

        UpiInitCapacity(&pUg31xx->capData);
        if((firstPowerOn == _UPI_TRUE_) || (pUg31xx->sysData.fccFromIC == 0)) {
                pUg31xx->batteryInfo.NAC = (_upi_u16_)pUg31xx->capData.rm;
                pUg31xx->batteryInfo.LMD = (_upi_u16_)pUg31xx->capData.fcc;
                pUg31xx->batteryInfo.RSOC = (_upi_u16_)pUg31xx->capData.rsoc;
                UG31_LOGE("[%s]: Init data from table -> %d/%d = %d\n", __func__,
                          pUg31xx->batteryInfo.NAC, pUg31xx->batteryInfo.LMD, pUg31xx->batteryInfo.RSOC);
        } else {
                pUg31xx->capData.tableUpdateIdx = pUg31xx->sysData.tableUpdateIdxFromIC;

                /// [AT-PM] : Calculate the RSOC/NAC/LMD from coulomb counter ; 01/27/2013
                deltaQC = (_upi_s16_)pUg31xx->measData.stepCap;
                pUg31xx->sysData.voltage = pUg31xx->measData.bat1Voltage;
                pUg31xx->sysData.curr = (_sys_s16_)pUg31xx->measData.curr;
                lastRsocFromIC = pUg31xx->sysData.rsocFromIC;
                UpiUpdateBatInfoFromIC(&pUg31xx->sysData, deltaQC);
                CmpCapData(pUg31xx, _UPI_TRUE_);
                pUg31xx->batteryInfo.NAC = (_upi_u16_)pUg31xx->capData.rm;
                pUg31xx->batteryInfo.LMD = (_upi_u16_)pUg31xx->capData.fcc;
                pUg31xx->batteryInfo.RSOC = (_upi_u16_)pUg31xx->capData.rsoc;
                UpiResetCoulombCounter(&pUg31xx->measData);
                pUg31xx->sysData.cycleCount = (_sys_u16_)pUg31xx->measData.cycleCount;
        }
        UpiInitDsgCharge(&pUg31xx->capData);
        UpiAdjustCCRecord(&pUg31xx->capData);

        /// [AT-PM] : Save battery information to IC ; 01/31/2013
        pUg31xx->sysData.rmFromIC = pUg31xx->batteryInfo.NAC;
        pUg31xx->sysData.fccFromIC = pUg31xx->batteryInfo.LMD;
        pUg31xx->sysData.rsocFromIC = (_sys_u8_)pUg31xx->batteryInfo.RSOC;
        pUg31xx->sysData.tableUpdateIdxFromIC = pUg31xx->capData.tableUpdateIdx;
        pUg31xx->sysData.deltaCapFromIC = pUg31xx->measData.lastDeltaCap;
        pUg31xx->sysData.adc1ConvTime = pUg31xx->measData.adc1ConvertTime;
        pUg31xx->sysData.ccOffset = (_sys_s8_)pUg31xx->measData.ccOffsetAdj;
        pUg31xx->sysData.standbyDsgRatio = (_sys_u8_)pUg31xx->capData.standbyDsgRatio;
        UpiSaveBatInfoTOIC(&pUg31xx->sysData);

        /// [AT-PM] : Initialize buffer for suspend / resume data ; 11/07/2013
        pUg31xx->backupData.capData = &pUg31xx->capData;
        pUg31xx->backupData.sysData = &pUg31xx->sysData;
        pUg31xx->backupData.measData = &pUg31xx->measData;
        UpiInitBackupData(&pUg31xx->backupData);

        dumpInfo(pUg31xx);
        return (UG_INIT_SUCCESS);
}

#if defined(uG31xx_OS_WINDOWS)

GGSTATUS upiGG_MpkActiveGG(char **pObj,const wchar_t* GGBFilename,const wchar_t* OtpFileName, _upi_u16_ i2cAddress)
{
        struct ug31xx_data *pUg31xx;
        SYSTEM_RTN_CODE rtn;
        MEAS_RTN_CODE rtnMeas;
        _upi_bool_ first_poweron;
        *pObj = (char *)upi_malloc(sizeof(struct ug31xx_data));
        pUg31xx = (struct ug31xx_data *)(*pObj);
        MPK_active = _UPI_TRUE_;

        upi_memset(pUg31xx, 0, sizeof(struct ug31xx_data));
        pUg31xx->sysData.ggbFilename = GGBFilename;
        pUg31xx->sysData.otpFileName = OtpFileName;
        pUg31xx->sysData.ggbParameter = &pUg31xx->cellParameter;
        pUg31xx->sysData.ggbCellTable = &pUg31xx->cellTable;
        rtn = UpiInitSystemData(&pUg31xx->sysData);
        if(rtn != SYSTEM_RTN_PASS) {
                if(rtn == SYSTEM_RTN_READ_GGB_FAIL) {
                        return (UG_READ_GGB_FAIL);
                }
                return (UG_NOT_DEF);
        }

        // Initial I2C and Open HID
        if(!API_I2C_Init(pUg31xx->cellParameter.clock, pUg31xx->cellParameter.i2cAddress)) {
                return UG_I2C_INIT_FAIL;
        }

        first_poweron = UpiCheckICActive();
        if(first_poweron == _UPI_TRUE_) {
                // Initial I2C and Open HID
                if(!API_I2C_Init(pUg31xx->cellParameter.clock, i2cAddress)) {
                        return UG_I2C_INIT_FAIL;
                }

                rtn = UpiActiveUg31xx();
                if(rtn != SYSTEM_RTN_PASS) {
                        return (UG_ACTIVE_FAIL);
                }
                // Initial I2C and Open HID
                if(!API_I2C_Init(pUg31xx->cellParameter.clock, pUg31xx->cellParameter.i2cAddress)) {
                        return UG_I2C_INIT_FAIL;
                }
        }
        UpiSetupAdc(&pUg31xx->sysData);
        UpiSetupSystem(&pUg31xx->sysData);

        UpiMeasReadCode(&pUg31xx->measData);
        /// [AT-PM] : Load OTP data ; 01/31/2013
        API_I2C_Read(SECURITY, UG31XX_I2C_HIGH_SPEED_MODE, UG31XX_I2C_TEM_BITS_MODE, OTP1_BYTE1, OTP1_SIZE, pUg31xx->otpData.otp1);
        API_I2C_Read(SECURITY, UG31XX_I2C_HIGH_SPEED_MODE, UG31XX_I2C_TEM_BITS_MODE, OTP2_BYTE1, OTP2_SIZE, pUg31xx->otpData.otp2);
        API_I2C_Read(NORMAL, UG31XX_I2C_HIGH_SPEED_MODE, UG31XX_I2C_TEM_BITS_MODE, OTP6_BYTE1, OTP3_SIZE, pUg31xx->otpData.otp3);
        UpiConvertOtp(&pUg31xx->otpData);

        /// [AT-PM] : Check product type ; 01/25/2013
        if(pUg31xx->otpData.productType != UG31XX_PRODUCT_TYPE_0) {
                return (UG_OTP_PRODUCT_DISMATCH);
        }

        UG31_LOGN("[%s]: Do measurement\n", __func__);
        pUg31xx->measData.sysData = &pUg31xx->sysData;
        pUg31xx->measData.otp = &pUg31xx->otpData;
        pUg31xx->measData.status = 0;
        rtnMeas = UpiMeasurement(&pUg31xx->measData, MEAS_SEL_ALL);
        if(rtnMeas != MEAS_RTN_PASS) {
                return ((GGSTATUS)(rtnMeas + UG_MEAS_FAIL));
        }
        /// [AT-PM] : Recover ADC1 conversion queue ; 06/04/2013
        if(first_poweron == _UPI_TRUE_) {
                UpiSetupAdc1Queue(&pUg31xx->sysData);
        }
        upiGG_BackupFileSwitch(_UPI_FALSE_);

        return UG_INIT_SUCCESS;
}

#endif  ///< end of defined(uG31xx_OS_WINDOWS)

#ifndef uG31xx_BOOT_LOADER

GGSTATUS upiGG_PreSuspend(char *pObj)
{
        GGSTATUS Status = UG_READ_DEVICE_INFO_SUCCESS;
        struct ug31xx_data *pUg31xx;

        UG31_LOGI("[%s]:*****upiGG_PreSuspend *****\n",  __func__);
        pUg31xx = (struct ug31xx_data *)pObj;

        UG31_LOGN("[%s]: Reset Coulomb Counter.\n", __func__);
        UpiResetCoulombCounter(&pUg31xx->measData);
        pUg31xx->sysData.cycleCount = (_sys_u16_)pUg31xx->measData.cycleCount;

        /// [AT-PM] : Save battery information to IC ; 01/31/2013
        pUg31xx->sysData.rmFromIC = pUg31xx->batteryInfo.NAC;
        pUg31xx->sysData.fccFromIC = pUg31xx->batteryInfo.LMD;
        pUg31xx->sysData.rsocFromIC = (_sys_u8_)pUg31xx->batteryInfo.RSOC;
        pUg31xx->sysData.tableUpdateIdxFromIC = pUg31xx->capData.tableUpdateIdx;
        pUg31xx->sysData.deltaCapFromIC = pUg31xx->measData.lastDeltaCap;
        pUg31xx->sysData.adc1ConvTime = pUg31xx->measData.adc1ConvertTime;
        pUg31xx->sysData.voltage = pUg31xx->measData.bat1Voltage;
        pUg31xx->sysData.curr = (_sys_s16_)pUg31xx->measData.curr;
        UpiUpdateBatInfoFromIC(&pUg31xx->sysData, (_sys_s16_) pUg31xx->measData.stepCap);
        pUg31xx->sysData.ccOffset = (_sys_s8_)pUg31xx->measData.ccOffsetAdj;
        pUg31xx->sysData.standbyDsgRatio = (_sys_u8_)pUg31xx->capData.standbyDsgRatio;
        UpiSaveBatInfoTOIC(&pUg31xx->sysData);
        return(Status);
}

/**
 * @brief upiGG_ShellUpdateCapacity
 *
 *  Run capacity algorithm to update capacity
 *
 * @para  pObj  address of memory buffer
 * @return  NULL
 */
void upiGG_ShellUpdateCapacity(char *pObj)
{
        struct ug31xx_data *pUg31xx;
        _upi_u32_ tmpDeltaTime;

        pUg31xx = (struct ug31xx_data *)pObj;

        tmpDeltaTime = pUg31xx->measData.deltaTime;

        pUg31xx->capData.ggbTable = &pUg31xx->cellTable;
        pUg31xx->capData.ggbParameter = &pUg31xx->cellParameter;
        pUg31xx->capData.measurement = &pUg31xx->measData;
        pUg31xx->measData.deltaTime = pUg31xx->measData.deltaTimeDaemon;

#ifndef UG31XX_SHELL_ALGORITHM

        UpiReadCapacity(&pUg31xx->capData);

#endif  ///< end of UG31XX_SHELL_ALGORITHM

        pUg31xx->measData.deltaTime = tmpDeltaTime;
        pUg31xx->measData.deltaTimeDaemon = 0;

        UG31_LOGD("[%s]: %d / %d = %d\n", __func__, pUg31xx->capData.rm, pUg31xx->capData.fcc, pUg31xx->capData.rsoc);
}

/**
 * @brief upiGG_ShellUpdateCC
 *
 *  Update capacity with coulomb counter information
 *
 * @para  pObj  address of memory buffer
 * @return  NULL
 */
void upiGG_ShellUpdateCC(char *pObj)
{
        struct ug31xx_data *pUg31xx;
        _upi_s16_ tmp16;

        pUg31xx = (struct ug31xx_data *)pObj;

        if(((MEAS_CABLE_OUT(pUg31xx->measData.status) == _UPI_TRUE_) && (pUg31xx->measData.stepCap > 0)) ||
            ((MEAS_CABLE_OUT(pUg31xx->measData.status) == _UPI_FALSE_) && (pUg31xx->measData.stepCap < 0)) ||
            ((pUg31xx->measData.curr > 0) && (pUg31xx->measData.stepCap < 0)) ||
            ((pUg31xx->measData.curr < 0) && (pUg31xx->measData.stepCap > 0))) {
                tmp16 = 0;
                UG31_LOGI("[%s]: Filter stepCap = 0 (%d)\n", __func__,
                          pUg31xx->measData.stepCap);
        } else {
                tmp16 = pUg31xx->measData.stepCap;
        }
        pUg31xx->sysData.voltage = (_sys_u16_)pUg31xx->measData.bat1Voltage;
        pUg31xx->sysData.curr = (_sys_s16_)pUg31xx->measData.curr;
        UpiUpdateBatInfoFromIC(&pUg31xx->sysData, (_sys_s16_)tmp16);

        pUg31xx->batteryInfo.NAC = pUg31xx->sysData.rmFromIC;
        pUg31xx->batteryInfo.LMD = pUg31xx->sysData.fccFromIC;
        pUg31xx->batteryInfo.RSOC = pUg31xx->sysData.rsocFromIC;
        pUg31xx->capData.rm = (_cap_u16_)pUg31xx->batteryInfo.NAC;
        pUg31xx->capData.fcc = (_cap_u16_)pUg31xx->batteryInfo.LMD;
        pUg31xx->capData.rsoc = (_cap_u8_)pUg31xx->batteryInfo.RSOC;
        UG31_LOGI("[%s]: %d / %d = %d\n", __func__,
                  pUg31xx->batteryInfo.NAC,
                  pUg31xx->batteryInfo.LMD,
                  pUg31xx->batteryInfo.RSOC);

        pUg31xx->sysData.rmFromIC = pUg31xx->batteryInfo.NAC;
        pUg31xx->sysData.fccFromIC = pUg31xx->batteryInfo.LMD;
        pUg31xx->sysData.rsocFromIC = (_sys_u8_)pUg31xx->batteryInfo.RSOC;
        pUg31xx->sysData.deltaCapFromIC = pUg31xx->measData.lastDeltaCap;
        pUg31xx->sysData.adc1ConvTime = pUg31xx->measData.adc1ConvertTime;
        pUg31xx->sysData.ccOffset = (_sys_s8_)pUg31xx->measData.ccOffsetAdj;
        UpiSaveBatInfoTOIC(&pUg31xx->sysData);
}

//====================================================
//API Call to get the Battery Capacity
// charge full condition:
//	if((Iav <TP current) && (Voltage >= TP Voltage))
//====================================================
void upiGG_ReadCapacity(char *pObj, GG_CAPACITY *pExtCapacity)
{
        struct ug31xx_data *pUg31xx;
        _upi_u8_ prevCapStsFC;
        _upi_u8_ nowCapStsFC;

        pUg31xx = (struct ug31xx_data *)pObj;

        pUg31xx->capData.ggbTable = &pUg31xx->cellTable;
        pUg31xx->capData.ggbParameter = &pUg31xx->cellParameter;
        pUg31xx->capData.measurement = &pUg31xx->measData;

#ifndef UG31XX_SHELL_ALGORITHM

        prevCapStsFC = ((pUg31xx->capData.fcSts == CAP_TRUE) ? _UPI_TRUE_ : _UPI_FALSE_);
        UpiReadCapacity(&pUg31xx->capData);
        nowCapStsFC = ((pUg31xx->capData.fcSts == CAP_TRUE) ? _UPI_TRUE_ : _UPI_FALSE_);
        UG31_LOGD("[%s]: %d / %d = %d\n", __func__, pUg31xx->capData.rm, pUg31xx->capData.fcc, pUg31xx->capData.rsoc);

#endif  ///< end of UG31XX_SHELL_ALGORITHM

        pUg31xx->batteryInfo.NAC = (_upi_u16_)pUg31xx->capData.rm;
        pUg31xx->batteryInfo.LMD = (_upi_u16_)pUg31xx->capData.fcc;
        pUg31xx->batteryInfo.RSOC = (_upi_u16_)pUg31xx->capData.rsoc;
        UG31_LOGI("[%s]: %d / %d = %d\n", __func__, pUg31xx->batteryInfo.NAC, pUg31xx->batteryInfo.LMD, pUg31xx->batteryInfo.RSOC);

#ifdef  uG31xx_OS_WINDOWS
        if((nowCapStsFC == CAP_TRUE) && (prevCapStsFC == CAP_FALSE)) {
                pUg31xx->batteryInfo.NAC = (_upi_u16_)pUg31xx->capData.rm;
                pUg31xx->batteryInfo.LMD = (_upi_u16_)pUg31xx->capData.fcc;
                pUg31xx->batteryInfo.RSOC = (_upi_u16_)pUg31xx->capData.rsoc;
                UpiSetChargerFull(&pUg31xx->capData, CAP_TRUE);
                pUg31xx->capData.rm = (_cap_u16_)pUg31xx->batteryInfo.NAC;
                pUg31xx->capData.fcc = (_cap_u16_)pUg31xx->batteryInfo.LMD;
                pUg31xx->capData.rsoc = (_cap_u8_)pUg31xx->batteryInfo.RSOC;
        }
#endif  ///< end of uG31xx_OS_WINDOWS

        // Output result by assign value from global variable
        pExtCapacity->LMD = pUg31xx->batteryInfo.LMD;
        pExtCapacity->NAC = pUg31xx->batteryInfo.NAC;
        pExtCapacity->RSOC = pUg31xx->batteryInfo.RSOC;
        pExtCapacity->Ready = pUg31xx->batteryInfo.Ready;
#ifdef  uG31xx_OS_WINDOWS
        pExtCapacity->DsgCharge = (_upi_s32_)pUg31xx->capData.dsgCharge;
#endif  ///< end of uG31xx_OS_WINDOWS

        /// [AT-PM] : If fully charged and keeps charging, reset coulomb counter ; 02/11/2013
        if((pUg31xx->batteryInfo.RSOC == 100) && (pUg31xx->measData.curr >= pUg31xx->cellParameter.standbyCurrent)) {
                pUg31xx->measData.sysData = &pUg31xx->sysData;
                pUg31xx->measData.otp = &pUg31xx->otpData;
                UpiResetCoulombCounter(&pUg31xx->measData);
                pUg31xx->sysData.cycleCount = (_sys_u16_)pUg31xx->measData.cycleCount;
        }

        /// [AT-PM] : Save battery information to IC ; 01/31/2013
        pUg31xx->sysData.rmFromIC = pUg31xx->batteryInfo.NAC;
        pUg31xx->sysData.fccFromIC = pUg31xx->batteryInfo.LMD;
        pUg31xx->sysData.rsocFromIC = (_sys_u8_)pUg31xx->batteryInfo.RSOC;
        pUg31xx->sysData.tableUpdateIdxFromIC = pUg31xx->capData.tableUpdateIdx;
        pUg31xx->sysData.deltaCapFromIC = pUg31xx->measData.lastDeltaCap;
        pUg31xx->sysData.adc1ConvTime = pUg31xx->measData.adc1ConvertTime;
        pUg31xx->sysData.ccOffset = (_sys_s8_)pUg31xx->measData.ccOffsetAdj;
        pUg31xx->sysData.standbyDsgRatio = (_sys_u8_)pUg31xx->capData.standbyDsgRatio;
        UpiSaveBatInfoTOIC(&pUg31xx->sysData);
        /// [FC] : Save table to IC ; 05/30/2013
        if(Ug31SaveDataEnable == _UPI_TRUE_) {
                UpiSaveTableToIC((_sys_u8_ *)pUg31xx->capData.encriptTable, (_sys_u8_ *)pUg31xx->capData.encriptBuf, (_sys_u8_)pUg31xx->capData.tableSize);
        }
}

//system wakeup
// to read back the preSuspend information from uG31xx RAM area
// re-calculate the deltaQmax( the charge/discharge) during the suspend time
GGSTATUS upiGG_Wakeup(char *pObj, _upi_bool_ dc_in_before)
{
        GGSTATUS Status = UG_READ_DEVICE_INFO_SUCCESS;
        _upi_s16_ deltaQC = 0;							//coulomb counter's deltaQ
        _upi_u32_ totalTime;
        MEAS_RTN_CODE rtn;
        GG_BATTERY_INFO batInfoBeforeSuspend;

        struct ug31xx_data *pUg31xx;
        pUg31xx = (struct ug31xx_data *)pObj;
        batInfoBeforeSuspend = pUg31xx->batteryInfo;

        ///Load the Saved time tag NAC LMD
        UpiLoadBatInfoFromIC(&pUg31xx->sysData);
        pUg31xx->measData.cycleCount = (_meas_u16_)pUg31xx->sysData.cycleCount;
        pUg31xx->measData.ccOffsetAdj = (_meas_s8_)pUg31xx->sysData.ccOffset;
        pUg31xx->capData.standbyDsgRatio = (_cap_u8_)pUg31xx->sysData.standbyDsgRatio;
        /// Count total Time
        totalTime = CountTotalTime(pUg31xx->sysData.timeTagFromIC);
        /// count the deltaQ during suspend
        pUg31xx->measData.sysData = &pUg31xx->sysData;
        pUg31xx->measData.otp = &pUg31xx->otpData;
        pUg31xx->measData.lastDeltaCap = pUg31xx->sysData.deltaCapFromIC;
        pUg31xx->measData.adc1ConvertTime = pUg31xx->sysData.adc1ConvTime;
        rtn = UpiMeasurement(&pUg31xx->measData, MEAS_SEL_ALL);
        if(rtn != MEAS_RTN_PASS) {
                return ((GGSTATUS)(rtn + UG_MEAS_FAIL));
        }
        pUg31xx->sysData.cycleCount = (_sys_u16_)pUg31xx->measData.cycleCount;
        deltaQC = (_upi_s16_)pUg31xx->measData.stepCap;
        /// [AT-PM] : Calculate the RSOC/NAC/LMD from coulomb counter ; 01/27/2013
        pUg31xx->sysData.voltage = pUg31xx->measData.bat1Voltage;
        pUg31xx->sysData.curr = (_sys_s16_)pUg31xx->measData.curr;
        UpiUpdateBatInfoFromIC(&pUg31xx->sysData, deltaQC);
        UG31_LOGI("[%s]: suspend time = %d ms,deltaQ = %d mAh, RSOC =%d, LMD = %d mAh, NAC=%d mAh\n",
                  __func__,
                  totalTime,
                  deltaQC,
                  pUg31xx->sysData.rsocFromIC,
                  pUg31xx->sysData.fccFromIC,
                  pUg31xx->sysData.rmFromIC);

        /// [AT-PM] : Calculate the RSOC/NAC/LMD from table ; 01/28/2013
        pUg31xx->capData.ggbTable = &pUg31xx->cellTable;
        pUg31xx->capData.ggbParameter = &pUg31xx->cellParameter;
        pUg31xx->capData.measurement = &pUg31xx->measData;
        pUg31xx->capData.tableUpdateIdx = pUg31xx->sysData.tableUpdateIdxFromIC;
        UpiTableCapacity(&pUg31xx->capData);
        /// [AT-PM] : Check the data accuracy ; 01/27/2013
        CmpCapData(pUg31xx, _UPI_FALSE_);
        pUg31xx->capData.dsgCharge = pUg31xx->capData.dsgCharge - (pUg31xx->capData.rm - pUg31xx->batteryInfo.NAC);

        /// [AT-PM] : Check capacity can not be increased if no dc in during suspend ; 10/25/2013
        if(dc_in_before == _UPI_FALSE_) {
                UG31_LOGI("[%s]: No dc in during suspend.\n", __func__);
                ChkResumeData(pUg31xx, &batInfoBeforeSuspend, totalTime, deltaQC);
        }

        /// [AT-PM] : Update capacity information ; 10/25/2013
        pUg31xx->batteryInfo.NAC = (_upi_u16_)pUg31xx->capData.rm;
        pUg31xx->batteryInfo.LMD = (_upi_u16_)pUg31xx->capData.fcc;
        pUg31xx->batteryInfo.RSOC = (_upi_u16_)pUg31xx->capData.rsoc;

        /// [AT-PM] : Reset coulomb counter ; 10/25/2013
        UpiResetCoulombCounter(&pUg31xx->measData);
        pUg31xx->sysData.cycleCount = (_sys_u16_)pUg31xx->measData.cycleCount;

        /// [AT-PM] : Save battery information to IC ; 01/31/2013
        pUg31xx->sysData.rmFromIC = pUg31xx->batteryInfo.NAC;
        pUg31xx->sysData.fccFromIC = pUg31xx->batteryInfo.LMD;
        pUg31xx->sysData.rsocFromIC = (_sys_u8_)pUg31xx->batteryInfo.RSOC;
        pUg31xx->sysData.tableUpdateIdxFromIC = pUg31xx->capData.tableUpdateIdx;
        pUg31xx->sysData.deltaCapFromIC = pUg31xx->measData.lastDeltaCap;
        pUg31xx->sysData.adc1ConvTime = pUg31xx->measData.adc1ConvertTime;
        pUg31xx->sysData.ccOffset = (_sys_s8_)pUg31xx->measData.ccOffsetAdj;
        pUg31xx->sysData.standbyDsgRatio = (_sys_u8_)pUg31xx->capData.standbyDsgRatio;
        UpiSaveBatInfoTOIC(&pUg31xx->sysData);

        return (Status);
}

/**
 * @brief upiGG_AccessMeasurementParameter
 *
 *  Access measurement parameter
 *
 * @para  read  set _UPI_TRUE_ to read data from API
 * @para  pMeasPara pointer of GG_MEAS_PARA_TYPE
 * @return  GGSTATUS
 */
GGSTATUS upiGG_AccessMeasurementParameter(char *pObj, _upi_bool_ read, GG_MEAS_PARA_TYPE *pMeasPara)
{
        struct ug31xx_data *pUg31xx;

        pUg31xx = (struct ug31xx_data *)pObj;

        /// [AT-PM] : Read data ; 08/29/2012
        if(read == _UPI_TRUE_) {
                pMeasPara->Adc1Gain = pUg31xx->cellParameter.adc1_ngain;
                pMeasPara->Adc1Offset = pUg31xx->cellParameter.adc1_pos_offset;
                pMeasPara->Adc2Gain = pUg31xx->cellParameter.adc2_gain;
                pMeasPara->Adc2Offset = pUg31xx->cellParameter.adc2_offset;
                pMeasPara->ITOffset = pUg31xx->cellParameter.adc_d5;
                pMeasPara->ETOffset = pUg31xx->cellParameter.adc_d4;
                pMeasPara->ProductType = pUg31xx->otpData.productType;
                return (UG_SUCCESS);
        }

        /// [AT-PM] : Write data ; 08/29/2012
        pUg31xx->cellParameter.adc1_ngain = pMeasPara->Adc1Gain;
        pUg31xx->cellParameter.adc1_pos_offset = pMeasPara->Adc1Offset;
        pUg31xx->cellParameter.adc2_gain = pMeasPara->Adc2Gain;
        pUg31xx->cellParameter.adc2_offset = pMeasPara->Adc2Offset;
        pUg31xx->cellParameter.adc_d5 = pMeasPara->ITOffset;
        pUg31xx->cellParameter.adc_d4 = pMeasPara->ETOffset;
        return (UG_SUCCESS);
}

#endif  ///< end of uG31xx_BOOT_LOADER

#ifdef  ENABLE_BQ27520_SW_CMD

/**
 * @brief TI_Cntl
 *
 *  Control() command
 *
 * @para  pData address of data
 * @return  GGSTATUS
 */
GGSTATUS TI_Cntl(struct ug31xx_data *pUg31xx, _upi_u16_ *pData)
{
        _upi_u16_ CntlData;

        CntlData = *pData;
        switch(CntlData) {
        case UG_STD_CMD_CNTL_CONTROL_STATUS:
                *pData = pUg31xx->bq27520Cmd.CntlControlStatus;
                break;
        case UG_STD_CMD_CNTL_DEVICE_TYPE:
                *pData = 0x3103;
                break;
        case UG_STD_CMD_CNTL_FW_VERSION:
                *pData = 0x0001;
                break;
        case UG_STD_CMD_CNTL_PREV_MACWRITE:
                *pData = pUg31xx->bq27520Cmd.CntlPrevMacWrite;
                break;
        case UG_STD_CMD_CNTL_CHEM_ID:
                *pData = 0x0001;
                break;
        case UG_STD_CMD_CNTL_OCV_CMD:
                break;
        case UG_STD_CMD_CNTL_BAT_INSERT:
                if(!(pUg31xx->bq27520Cmd.Opcfg & UG_STD_CMD_OPCFG_BIE)) {
                        pUg31xx->bq27520Cmd.Flags = pUg31xx->bq27520Cmd.Flags | UG_STD_CMD_FLAGS_BAT_DET;
                }
                break;
        case UG_STD_CMD_CNTL_BAT_REMOVE:
                if(!(pUg31xx->bq27520Cmd.Opcfg & UG_STD_CMD_OPCFG_BIE)) {
                        pUg31xx->bq27520Cmd.Flags = pUg31xx->bq27520Cmd.Flags & (~UG_STD_CMD_FLAGS_BAT_DET);
                }
                break;
        case UG_STD_CMD_CNTL_SET_HIBERNATE:
                pUg31xx->bq27520Cmd.CntlControlStatus = pUg31xx->bq27520Cmd.CntlControlStatus | UG_STD_CMD_CNTL_CONTROL_STATUS_HIBERNATE;
                break;
        case UG_STD_CMD_CNTL_CLEAR_HIBERNATE:
                pUg31xx->bq27520Cmd.CntlControlStatus = pUg31xx->bq27520Cmd.CntlControlStatus & (~UG_STD_CMD_CNTL_CONTROL_STATUS_HIBERNATE);
                break;
        case UG_STD_CMD_CNTL_SET_SLEEP_PLUS:
                pUg31xx->bq27520Cmd.CntlControlStatus = pUg31xx->bq27520Cmd.CntlControlStatus | UG_STD_CMD_CNTL_CONTROL_STATUS_SNOOZE;
                break;
        case UG_STD_CMD_CNTL_CLEAR_SLEEP_PLUS:
                pUg31xx->bq27520Cmd.CntlControlStatus = pUg31xx->bq27520Cmd.CntlControlStatus & (~UG_STD_CMD_CNTL_CONTROL_STATUS_SNOOZE);
                break;
        case UG_STD_CMD_CNTL_FACTORY_RESTORE:
                break;
        case UG_STD_CMD_CNTL_ENABLE_DLOG:
                pUg31xx->bq27520Cmd.CntlControlStatus = pUg31xx->bq27520Cmd.CntlControlStatus | UG_STD_CMD_CNTL_CONTROL_STATUS_DLOGEN;
                break;
        case UG_STD_CMD_CNTL_DISABLE_DLOG:
                pUg31xx->bq27520Cmd.CntlControlStatus = pUg31xx->bq27520Cmd.CntlControlStatus & (~UG_STD_CMD_CNTL_CONTROL_STATUS_DLOGEN);
                break;
        case UG_STD_CMD_CNTL_DF_VERSION:
                *pData = 0x0000;
                break;
        case UG_STD_CMD_CNTL_SEALED:
                pUg31xx->bq27520Cmd.CntlControlStatus = pUg31xx->bq27520Cmd.CntlControlStatus | UG_STD_CMD_CNTL_CONTROL_STATUS_SS;
                break;
        case UG_STD_CMD_CNTL_RESET:
                if(!(pUg31xx->bq27520Cmd.CntlControlStatus & UG_STD_CMD_CNTL_CONTROL_STATUS_SS)) {
                }
                break;
        default:
                *pData = 0x0000;
                break;
        }

        pUg31xx->bq27520Cmd.CntlPrevMacWrite = ((CntlData) > UG_STD_CMD_CNTL_PREV_MACWRITE) ? UG_STD_CMD_CNTL_PREV_MACWRITE : CntlData;
        return (UG_SUCCESS);
}

/**
 * @brief TI_AR
 *
 *  AtRate() command
 *
 * @para  pData address of data
 * @return  GGSTATUS
 */
GGSTATUS TI_AR(struct ug31xx_data *pUg31xx, _upi_u16_ *pData)
{
        _upi_s16_ AR;

        AR = (_upi_s16_)(*pData);
        if(AR != pUg31xx->bq27520Cmd.AR) {
                pUg31xx->bq27520Cmd.AR = AR;
        }
        return (UG_SUCCESS);
}

/**
 * @brief TI_Artte
 *
 *  AtRateTimeToEmpty() command
 *
 * @para  pData address of data
 * @return  GGSTATUS
 */
GGSTATUS TI_Artte(struct ug31xx_data *pUg31xx, _upi_u16_ *pData)
{
        _upi_s32_ Artte;

        if(pUg31xx->bq27520Cmd.AR >= 0) {
                *pData = 65535;
        }

        Artte = (_upi_s32_)pUg31xx->batteryInfo.NAC;
        Artte = Artte*60*(-1)/pUg31xx->bq27520Cmd.AR;
        *pData = (_upi_u16_)Artte;
        return (UG_SUCCESS);
}

/**
 * @brief TI_Temp
 *
 *  Temperature() command
 *
 * @para  pData address of data
 * @return  GGSTATUS
 */
GGSTATUS TI_Temp(struct ug31xx_data *pUg31xx, _upi_u16_ *pData)
{
        if(pUg31xx->bq27520Cmd.Opcfg & UG_STD_CMD_OPCFG_WRTEMP) {
                /// [AT-PM] : Temperature is from host ; 10/11/2012
                pUg31xx->bq27520Cmd.Temp = *pData;
        } else {
                /// [AT-PM] : Temperature is measured by uG31xx ; 10/11/2012
                if(pUg31xx->bq27520Cmd.Opcfg & UG_STD_CMD_OPCFG_TEMPS) {
                        /// [AT-PM] : Report external temperature ; 10/11/2012
                        *pData = pUg31xx->deviceInfo.ET;
                } else {
                        /// [AT-PM] : Report internal temperature ; 10/11/2012
                        *pData = pUg31xx->deviceInfo.IT;
                }
        }
        return (UG_SUCCESS);
}

/**
 * @brief TI_Volt
 *
 *  Voltage() command
 *
 * @para  pData address of data
 * @return  GGSTATUS
 */
GGSTATUS TI_Volt(struct ug31xx_data *pUg31xx, _upi_u16_ *pData)
{
        *pData = pUg31xx->deviceInfo.voltage_mV;
        return (UG_SUCCESS);
}

/**
 * @brief TI_Flags
 *
 *  Flags() command
 *
 * @para  pData address of data
 * @return  GGSTATUS
 */
GGSTATUS TI_Flags(struct ug31xx_data *pUg31xx, _upi_u16_ *pData)
{
        /// [AT-PM] : OTC - Overtemperature in charge ; 10/11/2012

        /// [AT-PM] : OTD - Overtemperature in discharge ; 10/11/2012

        /// [AT-PM] : CHG_INH - Charge inhibit ; 10/11/2012

        /// [AT-PM] : XCHG - Charge suspend alert ; 10/11/2012

        /// [AT-PM] : FC - Full-charged ; 10/11/2012
        if(pUg31xx->batteryInfo.RSOC < pUg31xx->bq27520Cmd.FCClear) {
                pUg31xx->bq27520Cmd.Flags = pUg31xx->bq27520Cmd.Flags & (~UG_STD_CMD_FLAGS_FC);
        }
        if(pUg31xx->bq27520Cmd.FCSet < 0) {
                if(pUg31xx->batteryInfo.RSOC == 100) {
                        pUg31xx->bq27520Cmd.Flags = pUg31xx->bq27520Cmd.Flags | UG_STD_CMD_FLAGS_FC;
                }
        } else {
                if(pUg31xx->batteryInfo.RSOC > pUg31xx->bq27520Cmd.FCSet) {
                        pUg31xx->bq27520Cmd.Flags = pUg31xx->bq27520Cmd.Flags | UG_STD_CMD_FLAGS_FC;
                }
        }

        /// [AT-PM] : CHG - (Fast) charging allowed ; 10/11/2012
        if(pUg31xx->bq27520Cmd.Flags & UG_STD_CMD_FLAGS_FC) {
                pUg31xx->bq27520Cmd.Flags = pUg31xx->bq27520Cmd.Flags & (~UG_STD_CMD_FLAGS_CHG);
        } else {
                pUg31xx->bq27520Cmd.Flags = pUg31xx->bq27520Cmd.Flags | UG_STD_CMD_FLAGS_CHG;
        }

        /// [AT-PM] : OCV_GD - Good OCV measurement taken ; 10/11/2012

        /// [AT-PM] : WAIT_ID - Waiting to identify inserted battery ; 10/11/2012

        /// [AT-PM] : BAT_DET - Battery detected ; 10/11/2012
        if(pUg31xx->userReg.regAlarm2Status & ALARM2_STATUS_OV1_ALARM) {
                pUg31xx->bq27520Cmd.Flags = pUg31xx->bq27520Cmd.Flags | UG_STD_CMD_FLAGS_BAT_DET;
        } else {
                pUg31xx->bq27520Cmd.Flags = pUg31xx->bq27520Cmd.Flags & (~UG_STD_CMD_FLAGS_BAT_DET);
        }

        /// [AT-PM] : SOC1 - State-of-charge threshold 1 (SOC1 Set) reached ; 10/11/2012
        if(pUg31xx->batteryInfo.NAC > pUg31xx->bq27520Cmd.Soc1Clear) {
                pUg31xx->bq27520Cmd.Flags = pUg31xx->bq27520Cmd.Flags & (~UG_STD_CMD_FLAGS_SOC1);
        }
        if(pUg31xx->batteryInfo.NAC < pUg31xx->bq27520Cmd.Soc1Set) {
                pUg31xx->bq27520Cmd.Flags = pUg31xx->bq27520Cmd.Flags | UG_STD_CMD_FLAGS_SOC1;
        }

        /// [AT-PM] : SYSDOWN - System should shut down ; 10/11/2012

        /// [AT-PM] : DSG - Discharging detected ; 10/11/2012
        if(pUg31xx->deviceInfo.AveCurrent_mA <= 0) {
                pUg31xx->bq27520Cmd.Flags = pUg31xx->bq27520Cmd.Flags | UG_STD_CMD_FLAGS_DSG;
        } else {
                pUg31xx->bq27520Cmd.Flags = pUg31xx->bq27520Cmd.Flags & (~UG_STD_CMD_FLAGS_DSG);
        }

        *pData = pUg31xx->bq27520Cmd.Flags;
        return (UG_SUCCESS);
}

/**
 * @brief TI_Nac
 *
 *  NominalAvailableCapacity() command
 *
 * @para  pData address of data
 * @return  GGSTATUS
 */
GGSTATUS TI_Nac(struct ug31xx_data *pUg31xx, _upi_u16_ *pData)
{
        *pData = pUg31xx->batteryInfo.NAC;
        return (UG_SUCCESS);
}

/**
 * @brief TI_Fac
 *
 *  FullAvailableCapacity() command
 *
 * @para  pData address of data
 * @return  GGSTATUS
 */
GGSTATUS TI_Fac(struct ug31xx_data *pUg31xx, _upi_u16_ *pData)
{
        *pData = pUg31xx->batteryInfo.LMD;
        return (UG_SUCCESS);
}

/**
 * @brief TI_RM
 *
 *  RemainingCapacity() command
 *
 * @para  pData address of data
 * @return  GGSTATUS
 */
GGSTATUS TI_RM(struct ug31xx_data *pUg31xx, _upi_u16_ *pData)
{
        *pData = pUg31xx->batteryInfo.NAC;
        return (UG_SUCCESS);
}

/**
 * @brief TI_Fcc
 *
 *  FullChargeCapacity() command
 *
 * @para  pData address of data
 * @return  GGSTATUS
 */
GGSTATUS TI_Fcc(struct ug31xx_data *pUg31xx, _upi_u16_ *pData)
{
        *pData = pUg31xx->batteryInfo.LMD;
        return (UG_SUCCESS);
}

/**
 * @brief TI_AI
 *
 *  AverageCurrent() command
 *
 * @para  pData address of data
 * @return  GGSTATUS
 */
GGSTATUS TI_AI(struct ug31xx_data *pUg31xx, _upi_u16_ *pData)
{
        *pData = pUg31xx->deviceInfo.AveCurrent_mA;
        return (UG_SUCCESS);
}

/**
 * @brief TI_Tte
 *
 *  TimeToEmpty() command
 *
 * @para  pData address of data
 * @return  GGSTATUS
 */
GGSTATUS TI_Tte(struct ug31xx_data *pUg31xx, _upi_u16_ *pData)
{
        _upi_s32_ Tte;

        if(pUg31xx->deviceInfo.AveCurrent_mA >= 0) {
                *pData = 65535;
                return (UG_SUCCESS);
        }

        Tte = (_upi_s32_)pUg31xx->batteryInfo.NAC;
        Tte = Tte*60*(-1)/pUg31xx->deviceInfo.AveCurrent_mA;
        *pData = (_upi_u16_)Tte;
        return (UG_SUCCESS);
}

/**
 * @brief TI_Ttf
 *
 *  TimeToFull() command
 *
 * @para  pData address of data
 * @return  GGSTATUS
 */
GGSTATUS TI_Ttf(struct ug31xx_data *pUg31xx, _upi_u16_ *pData)
{
        _upi_s32_ Ttf;

        if(pUg31xx->deviceInfo.AveCurrent_mA <= 0) {
                *pData = 65535;
                return (UG_SUCCESS);
        }

        Ttf = (_upi_s32_)pUg31xx->batteryInfo.LMD;
        Ttf = Ttf - pUg31xx->batteryInfo.NAC;
        Ttf = Ttf*90/pUg31xx->deviceInfo.AveCurrent_mA;
        *pData = (_upi_u16_)Ttf;
        return (UG_SUCCESS);
}

/**
 * @brief TI_SI
 *
 *  StandbyCurrent() command
 *
 * @para  pData address of data
 * @return  GGSTATUS
 */
GGSTATUS TI_SI(struct ug31xx_data *pUg31xx, _upi_u16_ *pData)
{
        _upi_s16_ LowerBound;
        _upi_s32_ NewSI;

        /// [AT-PM] : Set initial SI ; 10/11/2012
        if(pUg31xx->bq27520Cmd.SINow == 0) {
                pUg31xx->bq27520Cmd.SINow = pUg31xx->bq27520Cmd.InitSI;
        }

        LowerBound = pUg31xx->bq27520Cmd.InitSI*2;
        if(LowerBound > 0) {
                LowerBound = LowerBound*(-1);
        }

        /// [AT-PM] : SI criteria - 2 x InitSI < Current < 0 ; 10/11/2012
        if((pUg31xx->deviceInfo.AveCurrent_mA < 0) && (pUg31xx->deviceInfo.AveCurrent_mA > LowerBound)) {
                /// [AT-PM] : Update SI every 1 minute ; 10/11/2012
                if((pUg31xx->bq27520Cmd.SIWindow >= 60) && (pUg31xx->bq27520Cmd.SISample > 0)) {
                        NewSI = pUg31xx->bq27520Cmd.SIBuf/pUg31xx->bq27520Cmd.SISample;
                        NewSI = NewSI*7 + pUg31xx->bq27520Cmd.SINow*93;
                        NewSI = NewSI/100;
                        pUg31xx->bq27520Cmd.SINow = (_upi_s16_)NewSI;
                        pUg31xx->bq27520Cmd.SISample = -1;
                        pUg31xx->bq27520Cmd.SIBuf = 0;
                        pUg31xx->bq27520Cmd.SIWindow = 0;
                } else {
                        pUg31xx->bq27520Cmd.SISample = pUg31xx->bq27520Cmd.SISample + 1;
                        pUg31xx->bq27520Cmd.SIWindow = pUg31xx->bq27520Cmd.SIWindow + pUg31xx->bq27520Cmd.DeltaSec;

                        /// [AT-PM] : Ignore the first sample ; 10/11/2012
                        if(pUg31xx->bq27520Cmd.SISample > 0) {
                                pUg31xx->bq27520Cmd.SIBuf = pUg31xx->bq27520Cmd.SIBuf + pUg31xx->deviceInfo.AveCurrent_mA;
                        }
                }
        } else {
                /// [AT-PM] : Ignore the last sample ; 10/11/2012
                if(pUg31xx->bq27520Cmd.SISample > 0) {
                        NewSI = pUg31xx->bq27520Cmd.SIBuf/pUg31xx->bq27520Cmd.SISample;
                        NewSI = NewSI*7 + pUg31xx->bq27520Cmd.SINow*93;
                        NewSI = NewSI/100;
                        pUg31xx->bq27520Cmd.SINow = (_upi_s16_)NewSI;
                }
                pUg31xx->bq27520Cmd.SISample = -1;
                pUg31xx->bq27520Cmd.SIBuf = 0;
                pUg31xx->bq27520Cmd.SIWindow = 0;
        }

        *pData = pUg31xx->bq27520Cmd.SINow;
        return (UG_SUCCESS);
}

/**
 * @brief TI_Stte
 *
 *  StandbyTimeToEmpty() command
 *
 * @para  pData address of data
 * @return  GGSTATUS
 */
GGSTATUS TI_Stte(struct ug31xx_data *pUg31xx, _upi_u16_ *pData)
{
        _upi_s32_ Stte;

        if(pUg31xx->bq27520Cmd.SINow >= 0) {
                *pData = 65535;
                return (UG_SUCCESS);
        }

        Stte = (_upi_s32_)pUg31xx->batteryInfo.NAC;
        Stte = Stte*60*(-1)/pUg31xx->bq27520Cmd.SINow;
        *pData = (_upi_u16_)Stte;
        return (UG_SUCCESS);
}

/**
 * @brief TI_Mli
 *
 *  MaxLoadCurrent() command
 *
 * @para  pData address of data
 * @return  GGSTATUS
 */
GGSTATUS TI_Mli(struct ug31xx_data *pUg31xx, _upi_u16_ *pData)
{
        _upi_s32_ NewMli;

        /// [AT-PM] : Set initial MLI ; 10/11/2012
        if(pUg31xx->bq27520Cmd.Mli == 0) {
                pUg31xx->bq27520Cmd.Mli = pUg31xx->bq27520Cmd.InitMaxLoadCurrent;
        }

        /// [AT-PM] : Get the start charging SOC ; 10/11/2012
        if(pUg31xx->bq27520Cmd.Flags & UG_STD_CMD_FLAGS_DSG) {
                pUg31xx->bq27520Cmd.MliDsgSoc = (_upi_u8_)pUg31xx->batteryInfo.RSOC;
        }

        /// [AT-PM] : MLI criteria - Current < MLI ; 10/11/2012
        if(pUg31xx->deviceInfo.AveCurrent_mA < pUg31xx->bq27520Cmd.Mli) {
                pUg31xx->bq27520Cmd.Mli = pUg31xx->deviceInfo.AveCurrent_mA;
        }

        /// [AT-PM] : Reduce MLI at FC ; 10/11/2012
        if((pUg31xx->bq27520Cmd.Flags & UG_STD_CMD_FLAGS_FC) && (pUg31xx->bq27520Cmd.MliDsgSoc < 50)) {
                NewMli = (_upi_s32_)pUg31xx->bq27520Cmd.InitMaxLoadCurrent;
                NewMli = NewMli + pUg31xx->bq27520Cmd.Mli;
                NewMli = NewMli/2;
                pUg31xx->bq27520Cmd.Mli = (_upi_s16_)NewMli;
        }

        *pData = (_upi_u16_)pUg31xx->bq27520Cmd.Mli;
        return (UG_SUCCESS);
}

/**
 * @brief TI_Stte
 *
 *  MaxLoadTimeToEmpty() command
 *
 * @para  pData address of data
 * @return  GGSTATUS
 */
GGSTATUS TI_Mltte(struct ug31xx_data *pUg31xx, _upi_u16_ *pData)
{
        _upi_s32_ Mltte;

        if(pUg31xx->deviceInfo.AveCurrent_mA >= 0) {
                *pData = 65535;
                return (UG_SUCCESS);
        }

        Mltte = (_upi_s32_)pUg31xx->batteryInfo.NAC;
        Mltte = Mltte*60*(-1)/pUg31xx->bq27520Cmd.Mli;
        *pData = (_upi_u16_)Mltte;
        return (UG_SUCCESS);
}

/**
 * @brief TI_AE
 *
 *  AvailableEnergy() command
 *
 * @para  pData address of data
 * @return  GGSTATUS
 */
GGSTATUS TI_AE(struct ug31xx_data *pUg31xx, _upi_u16_ *pData)
{
        _upi_u32_ AE;

        AE = (_upi_u32_)pUg31xx->batteryInfo.NAC;
        AE = AE*pUg31xx->deviceInfo.voltage_mV/1000;
        pUg31xx->bq27520Cmd.AE = (_upi_u16_)AE;
        *pData = pUg31xx->bq27520Cmd.AE;
        return (UG_SUCCESS);
}

/**
 * @brief TI_AP
 *
 *  AveragePower() command
 *
 * @para  pData address of data
 * @return  GGSTATUS
 */
GGSTATUS TI_AP(struct ug31xx_data *pUg31xx, _upi_u16_ *pData)
{
        _upi_s32_ AP;

        if((pUg31xx->deviceInfo.AveCurrent_mA == 0) || (pUg31xx->bq27520Cmd.APDsgTime == 0)) {
                pUg31xx->bq27520Cmd.AP = 0;
                *pData = 0;
                return (UG_SUCCESS);
        }

        AP = (_upi_s32_)pUg31xx->batteryInfo.NAC;
        AP = AP*pUg31xx->deviceInfo.voltage_mV/1000;

        /// [AT-PM] : Average discharging power ; 10/11/2012
        if(pUg31xx->deviceInfo.AveCurrent_mA > 0) {
                pUg31xx->bq27520Cmd.APStartDsgE = AP;
                pUg31xx->bq27520Cmd.APDsgTime = 0;

                pUg31xx->bq27520Cmd.APChgTime = pUg31xx->bq27520Cmd.APChgTime + pUg31xx->bq27520Cmd.DeltaSec;
                AP = AP - pUg31xx->bq27520Cmd.APStartChgE;
                AP = AP*3600/pUg31xx->bq27520Cmd.APChgTime;
                pUg31xx->bq27520Cmd.AP = (_upi_s16_)AP;
        }

        /// [AT-PM] : Average charging power ; 10/11/2012
        if(pUg31xx->deviceInfo.AveCurrent_mA < 0) {
                pUg31xx->bq27520Cmd.APStartChgE = AP;
                pUg31xx->bq27520Cmd.APChgTime = 0;

                pUg31xx->bq27520Cmd.APDsgTime = pUg31xx->bq27520Cmd.APDsgTime + pUg31xx->bq27520Cmd.DeltaSec;
                AP = AP - pUg31xx->bq27520Cmd.APStartDsgE;
                AP = AP*3600/pUg31xx->bq27520Cmd.APDsgTime;
                pUg31xx->bq27520Cmd.AP = (_upi_s16_)AP;
        }

        *pData = (_upi_u16_)pUg31xx->bq27520Cmd.AP;
        return (UG_SUCCESS);
}

/**
 * @brief TI_Ttecp
 *
 *  TimeToEmptyAtConstantPower() command
 *
 * @para  pData address of data
 * @return  GGSTATUS
 */
GGSTATUS TI_Ttecp(struct ug31xx_data *pUg31xx, _upi_u16_ *pData)
{
        _upi_s32_ Ttecp;

        if(pUg31xx->bq27520Cmd.AP >= 0) {
                *pData = 65535;
                return (UG_SUCCESS);
        }

        Ttecp = (_upi_s32_)pUg31xx->bq27520Cmd.AE;
        Ttecp = Ttecp*60*(-1)/pUg31xx->bq27520Cmd.AP;
        *pData = (_upi_u16_)Ttecp;
        return (UG_SUCCESS);
}

/**
 * @brief TI_Soh
 *
 *  StateOfHealth() command
 *
 * @para  pData address of data
 * @return  GGSTATUS
 */
GGSTATUS TI_Soh(struct ug31xx_data *pUg31xx, _upi_u16_ *pData)
{
        _upi_u32_ Soh;

        Soh = (_upi_u32_)pUg31xx->batteryInfo.LMD;
        Soh = Soh*100/pUg31xx->bq27520Cmd.Dcap;

        Soh = Soh & UG_STD_CMD_SOH_VALUE_MASK;
        Soh = Soh | UG_STD_CMD_SOH_STATUS_READY;
        *pData = (_upi_u16_)Soh;
        return (UG_SUCCESS);
}

/**
 * @brief TI_CC
 *
 *  CycleCount() command
 *
 * @para  pData address of data
 * @return  GGSTATUS
 */
GGSTATUS TI_CC(struct ug31xx_data *pUg31xx, _upi_u16_ *pData)
{
        if(pUg31xx->deviceInfo.AveCurrent_mA < 0) {
                pUg31xx->bq27520Cmd.CCBuf = pUg31xx->bq27520Cmd.CCBuf + pUg31xx->bq27520Cmd.CCLastNac - pUg31xx->batteryInfo.NAC;
        }
        pUg31xx->bq27520Cmd.CCLastNac = pUg31xx->batteryInfo.NAC;

        if(pUg31xx->bq27520Cmd.CCBuf >= pUg31xx->bq27520Cmd.CCThreshold) {
                pUg31xx->bq27520Cmd.CC = pUg31xx->bq27520Cmd.CC + 1;
                pUg31xx->bq27520Cmd.CCBuf = pUg31xx->bq27520Cmd.CCBuf - pUg31xx->bq27520Cmd.CCThreshold;
        }

        *pData = pUg31xx->bq27520Cmd.CC;
        return (UG_SUCCESS);
}

/**
 * @brief TI_Soc
 *
 *  StateOfCharge() command
 *
 * @para  pData address of data
 * @return  GGSTATUS
 */
GGSTATUS TI_Soc(struct ug31xx_data *pUg31xx, _upi_u16_ *pData)
{
        *pData = pUg31xx->batteryInfo.RSOC;
        return (UG_SUCCESS);
}

/**
 * @brief TI_Nic
 *
 *  NormalizedImpedanceCal() command
 *
 * @para  pData address of data
 * @return  GGSTATUS
 */
GGSTATUS TI_Nic(struct ug31xx_data *pUg31xx, _upi_u16_ *pData)
{
        *pData = 0;
        return (UG_SUCCESS);
}

/**
 * @brief TI_Icr
 *
 *  InstantaneousCurrentReading() command
 *
 * @para  pData address of data
 * @return  GGSTATUS
 */
GGSTATUS TI_Icr(struct ug31xx_data *pUg31xx, _upi_u16_ *pData)
{
        *pData = pUg31xx->deviceInfo.current_mA;
        return (UG_SUCCESS);
}

/**
 * @brief TI_Dli
 *
 *  DataLogIndex() command
 *
 * @para  pData address of data
 * @return  GGSTATUS
 */
GGSTATUS TI_Dli(struct ug31xx_data *pUg31xx, _upi_u16_ *pData)
{
        *pData = pUg31xx->bq27520Cmd.Dli;
        return (UG_SUCCESS);
}

/**
 * @brief TI_Dlb
 *
 *  DataLogBuffer() command
 *
 * @para  pData address of data
 * @return  GGSTATUS
 */
GGSTATUS TI_Dlb(struct ug31xx_data *pUg31xx, _upi_u16_ *pData)
{
        *pData = pUg31xx->bq27520Cmd.Dlb;
        return (UG_SUCCESS);
}

/**
 * @brief TI_Itemp
 *
 *  InternalTemperature() command
 *
 * @para  pData address of data
 * @return  GGSTATUS
 */
GGSTATUS TI_Itemp(struct ug31xx_data *pUg31xx, _upi_u16_ *pData)
{
        *pData = pUg31xx->deviceInfo.IT;
        return (UG_SUCCESS);
}

/**
 * @brief TI_Opcfg
 *
 *  OperationConfiguration() command
 *
 * @para  pData address of data
 * @return  GGSTATUS
 */
GGSTATUS TI_Opcfg(struct ug31xx_data *pUg31xx, _upi_u16_ *pData)
{
        *pData = (_upi_u16_)pUg31xx->bq27520Cmd.Opcfg;
        return (UG_SUCCESS);
}

/**
 * @brief TI_Dcap
 *
 *  DesignCapacity() command
 *
 * @para  pData address of data
 * @return  GGSTATUS
 */
GGSTATUS TI_Dcap(struct ug31xx_data *pUg31xx, _upi_u16_ *pData)
{
        *pData = pUg31xx->bq27520Cmd.Dcap;
        return (UG_SUCCESS);
}

typedef GGSTATUS (*TIBq27520FuncPtr)(struct ug31xx_data *pUg31xx, _upi_u16_ *pData);
typedef struct TIBq27520FuncTableST {
        TIBq27520FuncPtr pFunc;
        _upi_u8_ CmdCode;
} TIBq27520FuncTableType;


TIBq27520FuncTableType TI_Command[] = {
        { TI_Cntl,  UG_STD_CMD_CNTL,  },
        { TI_AR,    UG_STD_CMD_AR,    },
        { TI_Artte, UG_STD_CMD_ARTTE, },
        { TI_Temp,  UG_STD_CMD_TEMP,  },
        { TI_Volt,  UG_STD_CMD_VOLT,  },
        { TI_Flags, UG_STD_CMD_FLAGS, },
        { TI_Nac,   UG_STD_CMD_NAC,   },
        { TI_Fac,   UG_STD_CMD_FAC,   },
        { TI_RM,    UG_STD_CMD_RM,    },
        { TI_Fcc,   UG_STD_CMD_FCC,   },
        { TI_AI,    UG_STD_CMD_AI,    },
        { TI_Tte,   UG_STD_CMD_TTE,   },
        { TI_Ttf,   UG_STD_CMD_TTF,   },
        { TI_SI,    UG_STD_CMD_SI,    },
        { TI_Stte,  UG_STD_CMD_STTE,  },
        { TI_Mli,   UG_STD_CMD_MLI,   },
        { TI_Mltte, UG_STD_CMD_MLTTE, },
        { TI_AE,    UG_STD_CMD_AE,    },
        { TI_AP,    UG_STD_CMD_AP,    },
        { TI_Ttecp, UG_STD_CMD_TTECP, },
        { TI_Soh,   UG_STD_CMD_SOH,   },
        { TI_CC,    UG_STD_CMD_CC,    },
        { TI_Soc,   UG_STD_CMD_SOC,   },
        { TI_Nic,   UG_STD_CMD_NIC,   },
        { TI_Icr,   UG_STD_CMD_ICR,   },
        { TI_Dli,   UG_STD_CMD_DLI,   },
        { TI_Dlb,   UG_STD_CMD_DLB,   },
        { TI_Itemp, UG_STD_CMD_ITEMP, },
        { TI_Opcfg, UG_STD_CMD_OPCFG, },
        { TI_Dcap,  UG_EXT_CMD_DCAP,  },
};

/**
 * @brief upiGG_FetchDataCommand
 *
 *  Read the gas gauge status following TI bq27520's interface
 *
 * @para  CommandCode command code
 * @para  pData address of returned data
 * @return  GGSTATUS
 */
GGSTATUS upiGG_FetchDataCommand(char *pObj, _upi_u8_ CommandCode, _upi_u16_ *pData)
{
        GGSTATUS Rtn;
        int TotalCmd;
        GG_DEVICE_INFO DevInfo;
        GG_CAPACITY CapData;
        _upi_u32_ DeltaT;
        int CmdIdx;
        struct ug31xx_data *pUg31xx;

        pUg31xx = (struct ug31xx_data *)pObj;

        Rtn = upiGG_ReadDeviceInfo(pObj, &DevInfo);
        if(Rtn != UG_READ_DEVICE_INFO_SUCCESS) {
                return (Rtn);
        }

        DeltaT = pUg31xx->measData.deltaTime;
        if(DeltaT >= 5000000) {
                upiGG_ReadCapacity(pObj, &CapData);
        }

        DeltaT = GetTickCount();
        DeltaT = DeltaT - pUg31xx->bq27520Cmd.LastTime;
        pUg31xx->bq27520Cmd.LastTime = GetTickCount();
        pUg31xx->bq27520Cmd.DeltaSec = (_upi_u16_)(DeltaT/1000);

        Rtn = UG_SUCCESS;
        TotalCmd = sizeof(TI_Command)/sizeof(TIBq27520FuncTableType);
        CmdIdx = 0;
        while(1) {
                if(TI_Command[CmdIdx].CmdCode == CommandCode) {
                        Rtn = (*TI_Command[CmdIdx].pFunc)(pUg31xx, pData);
                        break;
                }

                CmdIdx = CmdIdx + 1;
                if(CmdIdx >= TotalCmd) {
                        Rtn = UG_TI_CMD_OVERFLOW;
                }
        }

        return (Rtn);
}

/**
 * @brief upiGG_FetchDataParameter
 *
 *  Set the parameter for bq27520 like command
 *
 * @para  data  parameters of GG_FETCH_DATA_PARA_TYPE
 * @return  GGSTATUS
 */
GGSTATUS upiGG_FetchDataParameter(char *pObj, GG_FETCH_DATA_PARA_TYPE data)
{
        GGSTATUS Rtn;
        struct ug31xx_data *pUg31xx;

        pUg31xx = (struct ug31xx_data *)pObj;

        Rtn = UG_SUCCESS;

        pUg31xx->bq27520Cmd.FCSet = data.FCSet;
        pUg31xx->bq27520Cmd.FCClear = data.FCClear;
        pUg31xx->bq27520Cmd.Soc1Set = data.Soc1Set;
        pUg31xx->bq27520Cmd.Soc1Clear = data.Soc1Clear;
        pUg31xx->bq27520Cmd.InitSI = data.InitSI;
        pUg31xx->bq27520Cmd.InitMaxLoadCurrent = data.InitMaxLoadCurrent;
        pUg31xx->bq27520Cmd.CCThreshold = data.CCThreshold;
        pUg31xx->bq27520Cmd.Opcfg = data.Opcfg;
        pUg31xx->bq27520Cmd.Dcap = data.Dcap;
        return (Rtn);
}

#endif  ///< end of ENABLE_BQ27520_SW_CMD

#ifndef uG31xx_BOOT_LOADER

/**
 * @brief upiGG_DumpRegister
 *
 *  Dump whole register value
 *
 * @para  pBuf  address of register value buffer
 * @return  data size
 */
_upi_u16_ upiGG_DumpRegister(char *pObj, _upi_u8_ * pBuf)
{
        _upi_u16_ idx;
        struct ug31xx_data *pUg31xx;

        pUg31xx = (struct ug31xx_data *)pObj;

        idx = 0;
        upi_memcpy(pBuf + idx, pUg31xx->otpData.otp1, OTP1_SIZE);
        idx = idx + OTP1_SIZE;
        upi_memcpy(pBuf + idx, pUg31xx->otpData.otp2, OTP2_SIZE);
        idx = idx + OTP2_SIZE;
        upi_memcpy(pBuf + idx, pUg31xx->otpData.otp3, OTP3_SIZE);
        idx = idx + OTP3_SIZE;
        upi_memcpy(pBuf + idx, &pUg31xx->userReg, sizeof(GG_USER_REG));
        idx = idx + sizeof(GG_USER_REG);
        upi_memcpy(pBuf + idx, &pUg31xx->user2Reg, sizeof(GG_USER2_REG));
        idx = idx + sizeof(GG_USER2_REG);
        return (idx);
}

/**
 * @brief upiGG_DumpCellTable
 *
 *  Dump cell NAC table
 *
 * @para  pTable address of cell table
 * @return  _UPI_NULL_
 */
void upiGG_DumpCellTable(char *pObj, CELL_TABLE *pTable)
{
        struct ug31xx_data *pUg31xx;

        pUg31xx = (struct ug31xx_data *)pObj;
        upi_memcpy(pTable, &pUg31xx->cellTable, sizeof(CELL_TABLE));
}

#endif  ///< end of uG31xx_BOOT_LOADER

/**
 * @brief upiGG_UnInitial
 *
 *  Un-initialize uG31xx
 *
 * @para pObj address of memory buffer allocated for uG31xx
 * @return GGSTATUS
 */
GGSTATUS upiGG_UnInitial(char **pObj)
{
        struct ug31xx_data *pUg31xx;

        UG31_LOGE("[%s]***** upiGG_UnInitial() to free memory ***** \n",   __func__);

        if((*pObj) == _UPI_NULL_) {
                return (UG_SUCCESS);
        }

        pUg31xx = (struct ug31xx_data *)(*pObj);

#ifndef uG31xx_BOOT_LOADER

        // [FC] : Free table buffer ; 07/08/2013
        UpiFreeTableBuf((_sys_u8_ **)&pUg31xx->capData.encriptTable);
        UpiFreeTableBuf((_sys_u8_ **)&pUg31xx->capData.encriptBuf);

        /// [AT-PM] : Free suspend /resume data buffer ; 11/07/2013
        UpiFreeBackupData(&pUg31xx->backupData);

#endif  ///< end of uG31xx_BOOT_LOADER

        upi_free(*pObj);
        return (UG_SUCCESS);
}

#ifndef uG31xx_BOOT_LOADER

/**
 * @brief upiGG_DumpParameter
 *
 *  Dump all parameter setting
 *
 * @para  pTable address of cell parameter
 * @return  _UPI_NULL_
 */
void upiGG_DumpParameter(char *pObj, CELL_PARAMETER *pTable)
{
        struct ug31xx_data *pUg31xx;

        pUg31xx = (struct ug31xx_data *)pObj;
        upi_memcpy(pTable, &pUg31xx->cellParameter, sizeof(CELL_PARAMETER));
}

/**
 * @brief upiGG_FetchDebugData
 *
 *  Fetch debug information data
 *
 * @para  pObj  address of memory buffer
 * @para  data  address of GG_FETCH_CAP_DATA_TYPE
 * @return  _UPI_NULL_
 */
void upiGG_FetchDebugData(char *pObj, GG_FETCH_DEBUG_DATA_TYPE *data)
{
        struct ug31xx_data *pUg31xx;

        pUg31xx = (struct ug31xx_data *)pObj;

        data->capDelta = pUg31xx->measData.stepCap;
        data->capDsgCharge = pUg31xx->capData.dsgCharge;
        data->capDsgChargeStart = pUg31xx->capData.dsgChargeStart;
        data->capDsgChargeTime = pUg31xx->capData.dsgChargeTime;
        data->capPreDsgCharge = pUg31xx->capData.preDsgCharge;
        data->capSelfHour = pUg31xx->capData.selfDsgHour;
        data->capSelfMilliSec = pUg31xx->capData.selfDsgMilliSec;
        data->capSelfMin = pUg31xx->capData.selfDsgMin;
        data->capSelfSec = pUg31xx->capData.selfDsgSec;
        data->capStatus = pUg31xx->capData.status;
        data->capTableUpdateIdx = pUg31xx->capData.tableUpdateIdx;
        data->capTPTime = pUg31xx->capData.tpTime;

        data->measAdc1ConvertTime = pUg31xx->measData.adc1ConvertTime;
        data->measAdc1Gain = pUg31xx->measData.adc1Gain;
        data->measAdc1Offset = pUg31xx->measData.adc1Offset;
        data->measAdc2Gain = pUg31xx->measData.adc2Gain;
        data->measAdc2Offset = pUg31xx->measData.adc2Offset;
        data->measCCOffset = pUg31xx->measData.ccOffset;
        data->measCharge = pUg31xx->measData.codeCharge;
        data->measCodeBat1 = pUg31xx->measData.codeBat1;
        data->measCodeCurrent = pUg31xx->measData.codeCurrent;
        data->measCodeET = pUg31xx->measData.codeExtTemperature;
        data->measCodeIT = pUg31xx->measData.codeIntTemperature;
        data->measLastCounter = pUg31xx->measData.lastCounter;
        data->measLastDeltaQ = pUg31xx->measData.lastDeltaCap;
        data->measLastTimeTick = pUg31xx->measData.lastTimeTick;
        data->measAdc1CodeT25V100 = pUg31xx->measData.adc1CodeT25V100;
        data->measAdc1CodeT25V200 = pUg31xx->measData.adc1CodeT25V200;
        data->measAdc2CodeT25V100 = pUg31xx->measData.adc2CodeT25V100;
        data->measAdc2CodeT25V200 = pUg31xx->measData.adc2CodeT25V200;
        data->measCodeBat1BeforeCal = pUg31xx->measData.codeBat1BeforeCal;
        data->measCodeCurrentBeforeCal = pUg31xx->measData.codeCurrentBeforeCal;
        data->measCodeITBeforeCal = pUg31xx->measData.codeIntTemperatureBeforeCal;
        data->measCodeETBeforeCal = pUg31xx->measData.codeExtTemperatureBeforeCal;

        data->sysOscFrequency = pUg31xx->sysData.oscFreq;
}

/**
 * @brief upiGG_DebugSwitch
 *
 *  Enable/disable debug information to UART
 *
 * @para  Enable  set _UPI_TRUE_ to enable it
 * @return  NULL
 */
void upiGG_DebugSwitch(_upi_u8_ enable)
{
        Ug31DebugEnable = enable;
}

/**
 * @brief upiGG_BackupFileSwitch
 *
 *  Enable/disable backup file operation
 *
 * @para  Enable  set _UPI_TRUE_ to enable it
 * @return  NULL
 */
void upiGG_BackupFileSwitch(_upi_bool_ enable)
{
        if(Ug31BackupFileEnable != enable) {
                UG31_LOGN("[%s]: Ug31BackupFileEnable = %d -> %d\n", __func__, Ug31BackupFileEnable, enable);
        }
        Ug31BackupFileEnable = enable;
}

typedef struct BackupFileReloadRsocThrdST {
        _upi_s16_ rsoc;
        _upi_s8_ max;
        _upi_s8_ min;
} ALIGNED_ATTRIBUTE BackupFileReloadRsocThrdType;

static BackupFileReloadRsocThrdType BackupFileReloadRsocThrdTable[] = {
        { 80, 10, -10,  },
        { 60, 10, -20,  },
        { 20, 10, -30,  },
        { 10, 10, -20,  },
        { 0,  10, -10,  },
};

/**
 * @brief upiGG_BackupFileCheck
 *
 *  Backup file check procedure
 *
 * @para  pObj  address of memory buffer
 * @return  UG_CAP_DATA_STATUS
 */
#if defined (uG31xx_OS_WINDOWS)

_upi_u8_ upiGG_BackupFileCheck(char *pObj, const wchar_t* BackupFileName, const wchar_t* SuspendFileName)

#else   ///< else of defined (uG31xx_OS_WINDOWS)

_upi_u8_ upiGG_BackupFileCheck(char *pObj, char *BackupFileName, char *SuspendFileName)

#endif  ///< end of defined (uG31xx_OS_WINDOWS)
{
        struct ug31xx_data *pUg31xx;
        _upi_bool_ rtn;
        _upi_s16_ tmp16;
        _upi_u8_ idx;

        pUg31xx = (struct ug31xx_data *)pObj;

        if(Ug31BackupFileEnable != _UPI_TRUE_) {
                return (UG_CAP_DATA_READY);
        }

        /// [AT-PM] : Backup data to file routine ; 02/21/2013
        pUg31xx->sysData.predictRsoc = (_sys_u8_)pUg31xx->capData.predictRsoc;
        pUg31xx->sysData.standbyDsgRatio = (_sys_u8_)pUg31xx->capData.standbyDsgRatio;
        pUg31xx->sysData.ggbParameter = &pUg31xx->cellParameter;
        pUg31xx->backupData.capData = &pUg31xx->capData;
        pUg31xx->backupData.sysData = &pUg31xx->sysData;
        pUg31xx->backupData.measData = &pUg31xx->measData;
        pUg31xx->backupData.backupFileName = BackupFileName;
        pUg31xx->backupData.suspendFileName = SuspendFileName;
        UG31_LOGN("[%s]: Backup file routine START (%d)\n", __func__, pUg31xx->backupData.backupFileSts);
        UpiBackupVoltage(&pUg31xx->backupData);
        UpiBackupData(&pUg31xx->backupData);
        UG31_LOGN("[%s]: Backup file routine END (%d)\n", __func__, pUg31xx->backupData.backupFileSts);
        if(pUg31xx->Options & LKM_OPTIONS_ENABLE_SUSPEND_DATA_LOG) {
                UG31_LOGN("[%s]: Backup suspend / resume file routine START\n", __func__);
                UpiWriteSuspendResumeData(&pUg31xx->backupData);
                UG31_LOGN("[%s]: Backup suspend / resume file routine END\n", __func__);
        }

        if((pUg31xx->backupData.backupFileSts >= BACKUP_FILE_STS_EXIST) &&
            (pUg31xx->backupData.icDataAvailable == BACKUP_BOOL_FALSE)) {
                /// [AT-PM] : Restore data from file ; 09/16/2013
                pUg31xx->backupData.icDataAvailable = BACKUP_BOOL_TRUE;
                UG31_LOGI("[%s]: Restore file routine START\n", __func__);
                rtn = UpiRestoreData(&pUg31xx->backupData);
                if((rtn == _UPI_TRUE_) &&
                    (pUg31xx->cellParameter.NacLmdAdjustCfg & NAC_LMD_ADJUST_CFG_BATTERY_REINSERT_DETECT_EN)) {
                        pUg31xx->sysData.timeTagFromIC = pUg31xx->measData.lastTimeTick;
                        pUg31xx->sysData.deltaCapFromIC = pUg31xx->measData.lastDeltaCap;
                        /// [FC] : Compare rsoc difference between backup file and initial capacity ; 09/26/2013
                        tmp16 = (_upi_s16_)pUg31xx->capData.predictRsoc;
                        tmp16 = tmp16 - pUg31xx->sysData.predictRsoc;
                        UG31_LOGI("[%s]: Check RSOC difference -> %d - %d = %d\n", __func__, pUg31xx->capData.predictRsoc, pUg31xx->sysData.predictRsoc, tmp16);

                        idx = 0;
                        while(1) {
                                if(BackupFileReloadRsocThrdTable[idx].rsoc < pUg31xx->capData.predictRsoc) {
                                        break;
                                }
                                if(BackupFileReloadRsocThrdTable[idx].rsoc == 0) {
                                        break;
                                }
                                idx = idx + 1;
                        }
                        UG31_LOGN("[%s]: RSOC threshold (%d) %d -> %d\n", __func__, BackupFileReloadRsocThrdTable[idx].rsoc, BackupFileReloadRsocThrdTable[idx].max, BackupFileReloadRsocThrdTable[idx].min);

                        if((tmp16 < BackupFileReloadRsocThrdTable[idx].max) &&
                            (tmp16 > BackupFileReloadRsocThrdTable[idx].min)) {
                                UpiSaveBatInfoTOIC(&pUg31xx->sysData);
                                pUg31xx->capData.rm = (_cap_u16_)pUg31xx->sysData.rmFromIC;
                                pUg31xx->capData.fcc = (_cap_u16_)pUg31xx->sysData.fccFromIC;
                                pUg31xx->capData.rsoc = (_cap_u8_)pUg31xx->sysData.rsocFromIC;
                                pUg31xx->capData.tableUpdateIdx = pUg31xx->sysData.tableUpdateIdxFromIC;
                                UG31_LOGE("[%s]: Refresh driver information from file (%d/%d=%d)\n", __func__, pUg31xx->capData.rm, pUg31xx->capData.fcc, pUg31xx->capData.rsoc);
                        }
                        /// [FC] : Save table to IC ; 05/30/2013
                        if(Ug31SaveDataEnable == _UPI_TRUE_) {
                                UpiSaveTableToIC((_sys_u8_ *)pUg31xx->capData.encriptTable, (_sys_u8_ *)pUg31xx->capData.encriptBuf, (_sys_u8_)pUg31xx->capData.tableSize);
                        }
                        pUg31xx->sysData.rmFromIC = (_sys_u16_)pUg31xx->capData.rm;
                        pUg31xx->sysData.fccFromIC = (_sys_u16_)pUg31xx->capData.fcc;
                        pUg31xx->sysData.rsocFromIC = (_sys_u8_)pUg31xx->capData.rsoc;
                        pUg31xx->batteryInfo.NAC = (_upi_u16_)pUg31xx->capData.rm;
                        pUg31xx->batteryInfo.LMD = (_upi_u16_)pUg31xx->capData.fcc;
                        pUg31xx->batteryInfo.RSOC = (_upi_u16_)pUg31xx->capData.rsoc;
                        pUg31xx->measData.adc1ConvertTime = pUg31xx->sysData.adc1ConvTime;
                }
                UG31_LOGI("[%s]: Restore file routine END\n", __func__);
        }

        if(pUg31xx->backupData.backupFileSts == BACKUP_FILE_STS_VERSION_MISMATCH) {
                pUg31xx->batteryInfo.Ready = UG_CAP_DATA_VERSION_MISMATCH;
        } else if(pUg31xx->backupData.backupFileSts == BACKUP_FILE_STS_COMPARE) {
                pUg31xx->batteryInfo.Ready = UG_CAP_DATA_READY;
        } else {
                pUg31xx->batteryInfo.Ready = UG_CAP_DATA_NOT_READY;
        }
        return (pUg31xx->batteryInfo.Ready);
}

#endif  ///< end of uG31xx_BOOT_LOADER

#if defined (uG31xx_OS_WINDOWS)

//====================================================
//API Call to simulate capacity algorithm
//====================================================
void upiGG_AlgorithmSimulatorInit(char **pObj, const wchar_t* GGBFilename,
                                  MeasDataType *pMeasure, GG_CAP_LOG_TYPE *pCap,
                                  _upi_u8_ *NacTable)
{
        SYSTEM_RTN_CODE rtn;
        struct ug31xx_data *pUg31xx;
        _upi_s16_ deltaQC = 0;
        _sys_u8_ *ptr;

        *pObj = (char *)malloc(sizeof(struct ug31xx_data));
        pUg31xx = (struct ug31xx_data *)(*pObj);
        upi_memset(pUg31xx, 0, sizeof(struct ug31xx_data));

        pUg31xx->sysData.ggbFilename = GGBFilename;
        pUg31xx->sysData.ggbParameter = &pUg31xx->cellParameter;
        pUg31xx->sysData.ggbCellTable = &pUg31xx->cellTable;
        rtn = UpiInitSystemData(&pUg31xx->sysData);
        if(rtn != SYSTEM_RTN_PASS) {
                if(rtn == SYSTEM_RTN_READ_GGB_FAIL) {
                        return;
                }
                return;
        }
        // [FC] : Allocate buffer ; 07/08/2013
        ptr = (_sys_u8_ *)&pUg31xx->capData.encriptTable[0];
        UpiAllocateTableBuf((_sys_u8_ **)&ptr, &pUg31xx->capData.tableSize);
        ptr = (_sys_u8_ *)&pUg31xx->capData.encriptBuf[0];
        UpiAllocateTableBuf((_sys_u8_ **)&ptr, &pUg31xx->capData.tableSize);

        pUg31xx->capData.ggbTable = &pUg31xx->cellTable;
        pUg31xx->capData.ggbParameter = &pUg31xx->cellParameter;
        pUg31xx->capData.measurement = pMeasure;
        if(NacTable != NULL) {
                pUg31xx->sysData.rmFromIC = pCap->rm;
                pUg31xx->sysData.fccFromIC = pCap->fcc;
                pUg31xx->sysData.rsocFromIC = pCap->rsoc;
                pUg31xx->sysData.timeTagFromIC = pCap->timeTagFromIC;
                pUg31xx->sysData.tableUpdateIdxFromIC = pCap->tableUpdateIdxFromIC;
                pUg31xx->sysData.deltaCapFromIC = pCap->deltaCapFromIC;
                pUg31xx->sysData.adc1ConvTime = pCap->adc1ConvTime;
                pUg31xx->sysData.rmFromICBackup = pUg31xx->sysData.rmFromIC;
                pUg31xx->sysData.fccFromICBackup = pUg31xx->sysData.fccFromIC;
                pUg31xx->sysData.rsocFromICBackup = pUg31xx->sysData.rsocFromIC;
                upi_memcpy(pUg31xx->capData.encriptTable, NacTable, pUg31xx->capData.tableSize);
        }
        UpiInitNacTable(&pUg31xx->capData);
        UpiInitCapacity(&pUg31xx->capData);
        UpiInitDsgCharge(&pUg31xx->capData);
        if(NacTable != NULL) {
                pUg31xx->capData.tableUpdateIdx = pUg31xx->sysData.tableUpdateIdxFromIC;
                deltaQC = (_upi_s16_)pMeasure->stepCap;
                pUg31xx->sysData.voltage = pMeasure->bat1Voltage;
                pUg31xx->sysData.curr = (_sys_s16_)pUg31xx->measData.curr;
                UpiUpdateBatInfoFromIC(&pUg31xx->sysData, deltaQC);
                pUg31xx->capData.rm = (_cap_u16_)pUg31xx->sysData.rmFromIC;
                pUg31xx->capData.fcc = (_cap_u16_)pUg31xx->sysData.fccFromIC;
                pUg31xx->capData.rsoc = (_cap_u8_)pUg31xx->sysData.rsocFromIC;
                UG31_LOGI("[%s]: Use data from coulomb counter (%d/%d = %d)\n", __func__,
                          pUg31xx->capData.rm, pUg31xx->capData.fcc, pUg31xx->capData.rsoc);
                pUg31xx->capData.dsgCharge = pUg31xx->capData.dsgCharge - (pUg31xx->capData.rm - pUg31xx->batteryInfo.NAC);
                pUg31xx->batteryInfo.NAC = (_upi_u16_)pUg31xx->capData.rm;
                pUg31xx->batteryInfo.LMD = (_upi_u16_)pUg31xx->capData.fcc;
                pUg31xx->batteryInfo.RSOC = (_upi_u16_)pUg31xx->capData.rsoc;
                UpiAdjustCCRecord(&pUg31xx->capData);
                /// [FC] : Save battery information to AP ; 06/08/2013
                pCap->rm = (_upi_u16_)pUg31xx->capData.rm;
                pCap->fcc = (_upi_u16_)pUg31xx->capData.fcc;
                pCap->rsoc = (_upi_u16_)pUg31xx->capData.rsoc;
                pCap->timeTagFromIC = 0;
                pCap->tableUpdateIdxFromIC = pUg31xx->capData.tableUpdateIdx;
                pCap->deltaCapFromIC = pUg31xx->measData.lastDeltaCap;
                pCap->adc1ConvTime = pUg31xx->measData.adc1ConvertTime;
        }
}

void upiGG_AlgorithmSimulatorRead(char *pObj, MeasDataType *pMeasure,
                                  GG_CAP_LOG_TYPE *pCap, _upi_u8_ *NacTable)
{
        struct ug31xx_data *pUg31xx;

        pUg31xx = (struct ug31xx_data *)pObj;

        pUg31xx->capData.ggbTable = &pUg31xx->cellTable;
        pUg31xx->capData.ggbParameter = &pUg31xx->cellParameter;
        pUg31xx->capData.measurement = pMeasure;
        UG31_LOGI("[%s]: %d - %d - %d - %d\n", __func__, pMeasure->bat1Voltage, pMeasure->curr, pMeasure->intTemperature, pMeasure->stepCap);
        UpiReadCapacity(&pUg31xx->capData);
        pCap->rm = (_upi_u16_)pUg31xx->capData.rm;
        pCap->fcc = (_upi_u16_)pUg31xx->capData.fcc;
        pCap->rsoc = (_upi_u16_)pUg31xx->capData.rsoc;
        pCap->timeTagFromIC = -1;
        pCap->tableUpdateIdxFromIC = pUg31xx->capData.tableUpdateIdx;
        pCap->deltaCapFromIC = -1;
        pCap->adc1ConvTime = -1;
        UG31_LOGI("[%s]: RM = %d, FCC = %d, RSOC = %d\n", __func__, pCap->rm, pCap->fcc, pCap->rsoc);
        /// [FC] : Save table to file ; 05/30/2013
        upi_memcpy(NacTable, pUg31xx->capData.encriptTable, pUg31xx->capData.tableSize);
}

void upiGG_AlgorithmSimulatorClose(char **pObj)
{
        struct ug31xx_data *pUg31xx;

        pUg31xx = (struct ug31xx_data *)(*pObj);

        // [FC] : Allocate buffer ; 07/08/2013
        UpiFreeTableBuf((_sys_u8_ **)&pUg31xx->capData.encriptTable);
        UpiFreeTableBuf((_sys_u8_ **)&pUg31xx->capData.encriptBuf);

        upi_free(*pObj);
        *pObj = NULL;
}

#endif  ///< end of defined (uG31xx_OS_WINDOWS)

#ifndef uG31xx_BOOT_LOADER

/**
 * @brief upiGG_InternalSuspendMode
 *
 *  Set internal suspend mode
 *  In internal suspend mode, the adc1 conversion time will not be updated, and the delta time is estimated from adc conversion count
 *
 * @para  pObj  address of memory buffer
 * @para  inSuspend set _UPI_TRUE_ for enable internal suspend mode
 * @return  NULL
 */
void upiGG_InternalSuspendMode(char *pObj, _upi_bool_ inSuspend)
{
        struct ug31xx_data *pUg31xx;

        pUg31xx = (struct ug31xx_data *)pObj;

        /// [AT-PM] : Set for measurement module ; 12/10/2013
        if(inSuspend == _UPI_TRUE_) {
                pUg31xx->measData.status = pUg31xx->measData.status | MEAS_STATUS_IN_SUSPEND_MODE;
        } else {
                pUg31xx->measData.status = pUg31xx->measData.status & (~MEAS_STATUS_IN_SUSPEND_MODE);
        }
        return;
}

/**
 * @brief upiGG_Reset
 *
 *  Reset driver
 *
 * @para  pObj  address of buffer
 * @para  GGBFilename ggb filename
 * @para  OtpFileName otp data filename
 * @para  pGGBXBuf  address of ggb data
 * @return  GGSTATUS
 */
#if defined (uG31xx_OS_WINDOWS)

GGSTATUS upiGG_Reset(char *pObj, const wchar_t* GGBFilename, const wchar_t* OtpFileName)

#else

GGSTATUS upiGG_Reset(char *pObj, GGBX_FILE_HEADER *pGGBXBuf)

#endif
{
        struct ug31xx_data *pUg31xx;
        SYSTEM_RTN_CODE rtn;
        _upi_s32_ tmp32;
        MEAS_RTN_CODE rtnMeas;

        UG31_LOGI("[%s]: %s\n", __func__, UG31XX_API_VERSION);

        Ug31DebugEnable = LOG_LEVEL_ERROR;
        pUg31xx = (struct ug31xx_data *)pObj;

#ifdef uG31xx_OS_WINDOWS
        pUg31xx->sysData.ggbFilename = GGBFilename;
        pUg31xx->sysData.otpFileName = OtpFileName;
#else
        pUg31xx->sysData.ggbXBuf = pGGBXBuf;
#endif
        pUg31xx->sysData.ggbParameter = &pUg31xx->cellParameter;
        pUg31xx->sysData.ggbCellTable = &pUg31xx->cellTable;
        rtn = UpiInitSystemData(&pUg31xx->sysData);
        if(rtn != SYSTEM_RTN_PASS) {
                if(rtn == SYSTEM_RTN_READ_GGB_FAIL) {
                        return (UG_READ_GGB_FAIL);
                }
                return (UG_NOT_DEF);
        }

        // [FC] : Load table from IC ; 05/30/2013

        pUg31xx->capData.ggbTable = &pUg31xx->cellTable;
        pUg31xx->capData.ggbParameter = &pUg31xx->cellParameter;
        pUg31xx->capData.measurement = &pUg31xx->measData;
        UpiInitNacTable(&pUg31xx->capData);
        /// [FC] : Save table to IC ; 05/30/2013
        if(Ug31SaveDataEnable == _UPI_TRUE_) {
                UpiSaveTableToIC((_sys_u8_ *)&pUg31xx->capData.encriptTable, (_sys_u8_ *)&pUg31xx->capData.encriptBuf, (_sys_u8_)pUg31xx->capData.tableSize);
        }
        rtn = UpiActiveUg31xx();
        if(rtn != SYSTEM_RTN_PASS) {
                return (UG_ACTIVE_FAIL);
        }

        UpiSetupAdc(&pUg31xx->sysData);
        UpiSetupSystem(&pUg31xx->sysData);

        pUg31xx->backupData.icDataAvailable = BACKUP_BOOL_TRUE;

        /// [AT-PM] : Fetch ADC code for system stable ; 06/04/2013
        UpiMeasReadCode(&pUg31xx->measData);

        /// [AT-PM] : Load OTP data ; 01/31/2013
        API_I2C_Read(SECURITY, UG31XX_I2C_HIGH_SPEED_MODE, UG31XX_I2C_TEM_BITS_MODE, OTP1_BYTE1, OTP1_SIZE, pUg31xx->otpData.otp1);
        API_I2C_Read(SECURITY, UG31XX_I2C_HIGH_SPEED_MODE, UG31XX_I2C_TEM_BITS_MODE, OTP2_BYTE1, OTP2_SIZE, pUg31xx->otpData.otp2);
        API_I2C_Read(NORMAL, UG31XX_I2C_HIGH_SPEED_MODE, UG31XX_I2C_TEM_BITS_MODE, OTP6_BYTE1, OTP3_SIZE, pUg31xx->otpData.otp3);
        UpiConvertOtp(&pUg31xx->otpData);

        /// [AT-PM] : Check product type ; 01/25/2013
        if(pUg31xx->otpData.productType != UG31XX_PRODUCT_TYPE_0) {
                return (UG_OTP_PRODUCT_DISMATCH);
        }

        UG31_LOGN("[%s]: Do measurement\n", __func__);

        pUg31xx->measData.sysData = &pUg31xx->sysData;
        pUg31xx->measData.otp = &pUg31xx->otpData;
        pUg31xx->measData.status = 0;
        rtnMeas = UpiMeasurement(&pUg31xx->measData, MEAS_SEL_ALL);
        if(rtnMeas != MEAS_RTN_PASS) {
                return ((GGSTATUS)(rtnMeas + UG_MEAS_FAIL));
        }
        pUg31xx->sysData.cycleCount = (_sys_u16_)pUg31xx->measData.cycleCount;

        /// [AT-PM] : No external temperature average ; 07/04/2013
        tmp32 = (_upi_s32_)pUg31xx->measData.extTemperature;
        tmp32 = tmp32*ET_AVERAGE_BASE/ET_AVERAGE_NEW;
        pUg31xx->measData.extTemperature = (_meas_s16_)tmp32;

        /// [AT-PM] : Recover ADC1 conversion queue ; 06/04/2013
        UpiSetupAdc1Queue(&pUg31xx->sysData);

        /// [AT-PM] : Initialize alarm function ; 04/08/2013
        UpiMeasAlarmThreshold(&pUg31xx->measData);
        UpiInitAlarm(&pUg31xx->sysData);

        UG31_LOGI("[%s]: Current Status = %d mV / %d mA / %d 0.1oC\n", __func__,
                  pUg31xx->measData.bat1Voltage, pUg31xx->measData.curr, pUg31xx->measData.intTemperature);

        UpiInitCapacity(&pUg31xx->capData);
        pUg31xx->batteryInfo.NAC = (_upi_u16_)pUg31xx->capData.rm;
        pUg31xx->batteryInfo.LMD = (_upi_u16_)pUg31xx->capData.fcc;
        pUg31xx->batteryInfo.RSOC = (_upi_u16_)pUg31xx->capData.rsoc;
        UG31_LOGN("[%s]: Init data from table -> %d/%d = %d\n", __func__,
                  pUg31xx->batteryInfo.NAC, pUg31xx->batteryInfo.LMD, pUg31xx->batteryInfo.RSOC);
        UpiInitDsgCharge(&pUg31xx->capData);
        UpiAdjustCCRecord(&pUg31xx->capData);

        /// [AT-PM] : Save battery information to IC ; 01/31/2013
        pUg31xx->sysData.rmFromIC = pUg31xx->batteryInfo.NAC;
        pUg31xx->sysData.fccFromIC = pUg31xx->batteryInfo.LMD;
        pUg31xx->sysData.rsocFromIC = (_sys_u8_)pUg31xx->batteryInfo.RSOC;
        pUg31xx->sysData.tableUpdateIdxFromIC = pUg31xx->capData.tableUpdateIdx;
        pUg31xx->sysData.deltaCapFromIC = pUg31xx->measData.lastDeltaCap;
        pUg31xx->sysData.adc1ConvTime = pUg31xx->measData.adc1ConvertTime;
        pUg31xx->sysData.ccOffset = (_sys_s8_)pUg31xx->measData.ccOffsetAdj;
        pUg31xx->sysData.standbyDsgRatio = (_sys_u8_)pUg31xx->capData.standbyDsgRatio;
        UpiSaveBatInfoTOIC(&pUg31xx->sysData);

        dumpInfo(pUg31xx);
        return (UG_INIT_SUCCESS);
}

#endif  ///< end of uG31xx_BOOT_LOADER

#if defined (uG31xx_OS_WINDOWS)

/**
 * @brief upiGG_BackupMemory
 *
 *  Backup memory in Windows AP
 *
 * @para  pObj  address of memory buffer
 * @para  BackupFileName  filename of backup file
 * @return  NULL
 */
void upiGG_BackupMemory(char *pObj, const wchar_t* BackupFileName)
{
        CFile fileObj;

        if(fileObj.Open(BackupFileName, CFile::modeCreate | CFile::modeWrite, NULL) == FALSE) {
                return;
        }

        fileObj.Write(pObj, sizeof(struct ug31xx_data));
        fileObj.Close();
}

/**
 * @brief upiGG_RecoveryMemory
 *
 *  Recovery memory in Windows AP
 *
 * @para  pObj  address of memory buffer
 * @para  BackupFileName  filename of backup file
 * @return  NULL
 */
void upiGG_RecoveryMemory(char *pObj, const wchar_t* BackupFileName)
{
        CFile fileObj;
        struct ug31xx_data *pUg31xx;

        pUg31xx = (struct ug31xx_data *)pObj;

        if(fileObj.Open(BackupFileName, CFile::modeRead, NULL) == FALSE) {
                return;
        }

        fileObj.Read(pObj, sizeof(struct ug31xx_data));
        fileObj.Close();

        pUg31xx->sysData.ggbParameter = &pUg31xx->cellParameter;
        pUg31xx->sysData.ggbCellTable = &pUg31xx->cellTable;
        pUg31xx->capData.ggbTable = &pUg31xx->cellTable;
        pUg31xx->capData.ggbParameter = &pUg31xx->cellParameter;
        pUg31xx->capData.measurement = &pUg31xx->measData;
        pUg31xx->measData.sysData = &pUg31xx->sysData;
        pUg31xx->measData.otp = &pUg31xx->otpData;
}

#endif  ///< end of defined (uG31xx_OS_WINDOWS)

#ifndef uG31xx_BOOT_LOADER

/**
 * @brief upiGG_SetBatteryET
 *
 *  Set battery temperature from ET
 *
 * @para  pObj  address of memory buffer
 * @return  NULL
 */
void upiGG_SetBatteryET(char *pObj)
{
        struct ug31xx_data *pUg31xx;

        pUg31xx = (struct ug31xx_data *)pObj;
        pUg31xx->measData.status = pUg31xx->measData.status | MEAS_STATUS_REFER_ET;
}

/**
 * @brief upiGG_SetBatteryIT
 *
 *  Set battery temperature from IT
 *
 * @para  pObj  address of memory buffer
 * @return  NULL
 */
void upiGG_SetBatteryIT(char *pObj)
{
        struct ug31xx_data *pUg31xx;

        pUg31xx = (struct ug31xx_data *)pObj;
        pUg31xx->measData.status = pUg31xx->measData.status & (~MEAS_STATUS_REFER_ET);
}

#endif  ///< end of uG31xx_BOOT_LOADER

/**
 * @brief upiGG_GetCycleCount
 *
 *  Get cycle count information
 *
 * @para  pObj  address of memory buffer
 * @return  cycle count
 */
int upiGG_GetCycleCount(char *pObj)
{
        struct ug31xx_data *pUg31xx;

        if(pObj == _UPI_NULL_) {
                return (-1);
        }

        pUg31xx = (struct ug31xx_data *)pObj;
        return ((int)pUg31xx->measData.cycleCount);
}

/**
 * @brief upiGG_SetCycleCount
 *
 *  Set cycle count information
 *
 * @para  pObj  address of memory buffer
 * @para  value initial value of cycle count
 * @return  0 if success
 */
int upiGG_SetCycleCount(char *pObj, _upi_u16_ value)
{
        struct ug31xx_data *pUg31xx;

        if(pObj == _UPI_NULL_) {
                return (-1);
        }

        pUg31xx = (struct ug31xx_data *)pObj;
        pUg31xx->sysData.cycleCount = (_sys_u16_)value;
        pUg31xx->measData.cycleCount = (_meas_u16_)value;
        pUg31xx->measData.cycleCountBuf = 0;
        return (0);
}

/**
 * @brief upiGG_ReverseCurrent
 *
 *  Enable reverse current feature
 *
 * @para  pObj  address of memory buffer
 * @para  reverse set to _UPI_TRUE_ to enable reverse current feature
 * @return  NULL
 */
void upiGG_ReverseCurrent(char *pObj, _upi_bool_ reverse)
{
        struct ug31xx_data *pUg31xx;

        pUg31xx = (struct ug31xx_data *)pObj;
        if(reverse == _UPI_TRUE_) {
                pUg31xx->measData.status = pUg31xx->measData.status | MEAS_STATUS_REVERSE_CURRENT_DIRECTION;
        } else {
                pUg31xx->measData.status = pUg31xx->measData.status & (~MEAS_STATUS_REVERSE_CURRENT_DIRECTION);
        }
}

/**
 * @brief upiGG_AdjustCellTable
 *
 *  Adjust current cell table according to design capacity
 *
 * @para  pObj  address of memory buffer
 * @para  designCap target design capacity
 * @return  NULL
 */
void upiGG_AdjustCellTable(char *pObj, _upi_u16_ designCap)
{
        struct ug31xx_data *pUg31xx;
        _upi_u32_ tmp32;
        _upi_u16_ ratio;
        _upi_u8_ idxTemp;
        _upi_u8_ idxCRate;
        _upi_u8_ idxSov;

        pUg31xx = (struct ug31xx_data *)pObj;

        /// [AT-PM] : Get ratio between target and current design capacity ; 11/21/2013
        tmp32 = (_upi_u32_)designCap;
        tmp32 = tmp32*CONST_PERCENTAGE*CONST_ROUNDING/(pUg31xx->cellParameter.ILMD);
        ratio = (_upi_u16_)tmp32;
        UG31_LOGI("[%s]: Ratio of design capacity = %d / %d = %d\n", __func__, designCap, pUg31xx->cellParameter.ILMD, ratio);

        /// [AT-PM] : Refresh cell table ; 11/21/2013
        idxTemp = 0;
        while(idxTemp < TEMPERATURE_NUMS) {
                idxCRate = 0;
                while(idxCRate < C_RATE_NUMS) {
                        pUg31xx->cellTable.CELL_NAC_TABLE[idxTemp][idxCRate][0] = 0;
                        idxSov = 1;
                        while(idxSov < SOV_NUMS) {
                                tmp32 = (_upi_u32_)pUg31xx->cellTable.CELL_NAC_TABLE[idxTemp][idxCRate][idxSov];
                                tmp32 = tmp32*ratio/CONST_PERCENTAGE/CONST_ROUNDING;
                                if(tmp32 == 0) {
                                        tmp32 = 1;
                                }
                                pUg31xx->cellTable.CELL_NAC_TABLE[idxTemp][idxCRate][0] = pUg31xx->cellTable.CELL_NAC_TABLE[idxTemp][idxCRate][0] + tmp32;
                                UG31_LOGI("[%s]: Table[%d][%d][%d] = %d -> %d (%d)\n", __func__, idxTemp, idxCRate, idxSov,
                                          pUg31xx->cellTable.CELL_NAC_TABLE[idxTemp][idxCRate][idxSov], tmp32, pUg31xx->cellTable.CELL_NAC_TABLE[idxTemp][idxCRate][0]);
                                pUg31xx->cellTable.CELL_NAC_TABLE[idxTemp][idxCRate][idxSov] = (_upi_s16_)tmp32;
                                idxSov = idxSov + 1;
                        }
                        idxCRate = idxCRate + 1;
                }
                idxTemp = idxTemp + 1;
        }
}

/**
 * @brief upiGG_GetNtcStatus
 *
 *  Report NTC status
 *
 * @para  pObj  address of memory buffer
 * @return  GGSTATUS
 */
GGSTATUS upiGG_GetNtcStatus(char *pObj)
{
        struct ug31xx_data *pUg31xx;

        pUg31xx = (struct ug31xx_data *)pObj;

        if(MEAS_NTC_OPEN(pUg31xx->measData.status) == _UPI_TRUE_) {
                return (UG_MEAS_FAIL_NTC_OPEN);
        }

        if(MEAS_NTC_SHORT(pUg31xx->measData.status) == _UPI_TRUE_) {
                return (UG_MEAS_FAIL_NTC_SHORT);
        }

        return (UG_SUCCESS);
}

/**
 * @brief upiGG_AccessBackupBuffer
 *
 *  Access backup data buffer
 *
 * @para  pObj  address of memory buffer
 * @para  size  address of backup buffer size
 * @return  address of backup buffer
 */
_upi_u8_ * upiGG_AccessBackupBuffer(char *pObj, _upi_u8_ *size)
{
        struct ug31xx_data *pUg31xx;

        pUg31xx = (struct ug31xx_data *)pObj;

        *size = (_upi_u8_)pUg31xx->backupData.backupBufferSize;
        return ((_upi_u8_ *)(pUg31xx->backupData.backupBuffer));
}

/**
 * @brief upiGG_FetchCurrent
 *
 *  Fetch current only
 *
 * @para  pObj  address of memory buffer
 * @return  GGSTATUS
 */
GGSTATUS upiGG_FetchCurrent(char *pObj)
{
        struct ug31xx_data *pUg31xx;
        MEAS_RTN_CODE rtn;
        GGSTATUS status;

        pUg31xx = (struct ug31xx_data *)pObj;

        /// [AT-PM] : Check IT AVE code, which should be continuous ; 12/28/2012
        ChkITAveCode(pUg31xx);

        status = CheckOtpData(&pUg31xx->otpData);
        if(status != UG_READ_DEVICE_INFO_SUCCESS) {
                return (status);
        }

        pUg31xx->measData.sysData = &pUg31xx->sysData;
        pUg31xx->measData.otp = &pUg31xx->otpData;
        rtn = UpiMeasurement(&pUg31xx->measData, MEAS_SEL_CURRENT);
        if(rtn != MEAS_RTN_PASS) {
                return ((GGSTATUS)(rtn + UG_MEAS_FAIL));
        }

        pUg31xx->deviceInfo.aveCurrentRegister = pUg31xx->userReg.regCurrentAve;    //2012/07/11
        pUg31xx->deviceInfo.current_mA = pUg31xx->measData.curr;
        pUg31xx->deviceInfo.AveCurrent_mA = pUg31xx->measData.curr;
        return (UG_READ_DEVICE_INFO_SUCCESS);
}

/**
 * @brief upiGG_FetchVoltage
 *
 *  Fetch voltage only
 *
 * @para  pObj  address of memory buffer
 * @return  GGSTATUS
 */
GGSTATUS upiGG_FetchVoltage(char *pObj)
{
        struct ug31xx_data *pUg31xx;
        MEAS_RTN_CODE rtn;
        GGSTATUS status;

        pUg31xx = (struct ug31xx_data *)pObj;

        /// [AT-PM] : Check IT AVE code, which should be continuous ; 12/28/2012
        ChkITAveCode(pUg31xx);

        status = CheckOtpData(&pUg31xx->otpData);
        if(status != UG_READ_DEVICE_INFO_SUCCESS) {
                return (status);
        }

        pUg31xx->measData.sysData = &pUg31xx->sysData;
        pUg31xx->measData.otp = &pUg31xx->otpData;
        rtn = UpiMeasurement(&pUg31xx->measData, MEAS_SEL_VOLTAGE);
        if(rtn != MEAS_RTN_PASS) {
                return ((GGSTATUS)(rtn + UG_MEAS_FAIL));
        }

        pUg31xx->deviceInfo.v1_mV = pUg31xx->measData.bat1Voltage;
        pUg31xx->deviceInfo.vCell1_mV = pUg31xx->measData.bat1Voltage;
        pUg31xx->deviceInfo.vBat1Average_mV = CalculateVoltageFromUserReg(pUg31xx,
                                              pUg31xx->measData.bat1Voltage,
                                              pUg31xx->measData.curr,
                                              pUg31xx->cellParameter.offsetR,
                                              0);
        pUg31xx->deviceInfo.voltage_mV = pUg31xx->deviceInfo.vBat1Average_mV;
        return (UG_READ_DEVICE_INFO_SUCCESS);
}

/**
 * @brief upiGG_FetchInternalTemperature
 *
 *  Fetch internal temperature only
 *
 * @para  pObj  address of memory buffer
 * @return  GGSTATUS
 */
GGSTATUS upiGG_FetchInternalTemperature(char *pObj)
{
        struct ug31xx_data *pUg31xx;
        MEAS_RTN_CODE rtn;
        GGSTATUS status;

        pUg31xx = (struct ug31xx_data *)pObj;

        /// [AT-PM] : Check IT AVE code, which should be continuous ; 12/28/2012
        ChkITAveCode(pUg31xx);

        status = CheckOtpData(&pUg31xx->otpData);
        if(status != UG_READ_DEVICE_INFO_SUCCESS) {
                return (status);
        }

        pUg31xx->measData.sysData = &pUg31xx->sysData;
        pUg31xx->measData.otp = &pUg31xx->otpData;
        rtn = UpiMeasurement(&pUg31xx->measData, MEAS_SEL_INT_TEMP);
        if(rtn != MEAS_RTN_PASS) {
                return ((GGSTATUS)(rtn + UG_MEAS_FAIL));
        }

        pUg31xx->deviceInfo.IT = pUg31xx->measData.intTemperature;
        return (UG_READ_DEVICE_INFO_SUCCESS);
}

/**
 * @brief upiGG_FetchExternalTemperature
 *
 *  Fetch external temperature only
 *
 * @para  pObj  address of memory buffer
 * @return  GGSTATUS
 */
GGSTATUS upiGG_FetchExternalTemperature(char *pObj)
{
        struct ug31xx_data *pUg31xx;
        MEAS_RTN_CODE rtn;
        GGSTATUS status;

        pUg31xx = (struct ug31xx_data *)pObj;

        /// [AT-PM] : Check IT AVE code, which should be continuous ; 12/28/2012
        ChkITAveCode(pUg31xx);

        status = CheckOtpData(&pUg31xx->otpData);
        if(status != UG_READ_DEVICE_INFO_SUCCESS) {
                return (status);
        }

        pUg31xx->measData.sysData = &pUg31xx->sysData;
        pUg31xx->measData.otp = &pUg31xx->otpData;
        rtn = UpiMeasurement(&pUg31xx->measData, MEAS_SEL_EXT_TEMP);
        if(rtn != MEAS_RTN_PASS) {
                return ((GGSTATUS)(rtn + UG_MEAS_FAIL));
        }

        pUg31xx->deviceInfo.ET = pUg31xx->measData.extTemperature;
        return (UG_READ_DEVICE_INFO_SUCCESS);
}

#define MAXIMUM_BOARD_OFFSET      (127)
#define MINIMUM_BOARD_OFFSET      (-127)

/**
 * @brief upiGG_GetBoardOffset
 *
 *  Record measured current as board offset
 *
 * @para  pObj  address of memory buffer
 * @para  fullStep  set _UPI_TRUE_ to full calibration
 * @para  upper upper bound for calibration
 * @para  lower lower bound for calibration
 * @return  GGSTATUS
 */
void upiGG_GetBoardOffset(char *pObj, _upi_s16_ fullStep, _upi_s16_ upper, _upi_s16_ lower)
{
        struct ug31xx_data *pUg31xx;
        _upi_s16_ rawCurr;
        _upi_s8_ oldCCOffsetAdj;
        _upi_s16_ avgCCOffset;

        pUg31xx = (struct ug31xx_data *)pObj;
        oldCCOffsetAdj = (_upi_s8_)pUg31xx->measData.ccOffsetAdj;

        UG31_LOGE("[%s]: Board offset information -> Step=%d, I=%d, BO_Org=%d, BO_Adj=%d, UB=%d, LB=%d.\n", __func__,
                  fullStep,
                  pUg31xx->measData.curr,
                  pUg31xx->cellParameter.adc1_pos_offset,
                  pUg31xx->measData.ccOffsetAdj,
                  upper,
                  lower);

        rawCurr = (_upi_s16_)pUg31xx->measData.curr;
        rawCurr = rawCurr + pUg31xx->measData.ccOffsetAdj;

        if((pUg31xx->measData.curr < upper) && (pUg31xx->measData.curr > lower)) {
                pUg31xx->measData.ccOffsetAdj = (_meas_s8_)rawCurr;
        } else {
                upper = upper*2;
                lower = lower*2;

                if((pUg31xx->measData.curr < upper) && (pUg31xx->measData.curr > lower)) {
                        pUg31xx->measData.ccOffsetAdj = (_meas_s8_)rawCurr;
                        pUg31xx->measData.ccOffsetAdj = pUg31xx->measData.ccOffsetAdj/2;
                }
        }

        if(fullStep == GET_BOARD_OFFSET_STEP) {
                if(oldCCOffsetAdj < pUg31xx->measData.ccOffsetAdj) {
                        pUg31xx->measData.ccOffsetAdj = (_meas_s8_)oldCCOffsetAdj;
                        pUg31xx->measData.ccOffsetAdj = pUg31xx->measData.ccOffsetAdj + 1;
                }
                if(oldCCOffsetAdj > pUg31xx->measData.ccOffsetAdj) {
                        pUg31xx->measData.ccOffsetAdj = (_meas_s8_)oldCCOffsetAdj;
                        pUg31xx->measData.ccOffsetAdj = pUg31xx->measData.ccOffsetAdj - 1;
                }
        } else {
                UpiSetBoardOffsetKed(&pUg31xx->capData);
        }

        if(fullStep == GET_BOARD_OFFSET_AVG) {
                avgCCOffset = (_upi_s16_)pUg31xx->measData.ccOffsetAdj;
                avgCCOffset = avgCCOffset + oldCCOffsetAdj;
                avgCCOffset = avgCCOffset/2;
                pUg31xx->measData.ccOffsetAdj = (_meas_s8_)avgCCOffset;
        }

        UG31_LOGE("[%s]: (%d) Board offset = %d (%d)\n", __func__, fullStep, pUg31xx->measData.ccOffsetAdj, rawCurr);
}

/**
 * @brief upiGG_SetCapacitySuspendMode
 *
 *  Set capacity module in suspend mode
 *
 * @para  pObj  address of memory buffer
 * @para  inSuspend set _UPI_TRUE_ for entering suspend mode
 * @return  NULL
 */
void upiGG_SetCapacitySuspendMode(char *pObj, _upi_bool_ inSuspend)
{
        struct ug31xx_data *pUg31xx;

        pUg31xx = (struct ug31xx_data *)pObj;
        pUg31xx->capData.inSuspend = (inSuspend == _UPI_TRUE_) ? CAP_TRUE : CAP_FALSE;
}

/**
 * @brief upiGG_SetCapacity
 *
 *  Force set capacity to target %
 *
 * @para  pObj  address of memory buffer
 * @para  target  target RSOC
 * @return  NULL
 */
void upiGG_SetCapacity(char *pObj, _upi_u8_ target)
{
        struct ug31xx_data *pUg31xx;
        _upi_u32_ tmp32;

        pUg31xx = (struct ug31xx_data *)pObj;

        tmp32 = pUg31xx->capData.fcc;
        tmp32 = tmp32*target/CONST_PERCENTAGE;

        pUg31xx->capData.rm = (_cap_u16_)tmp32;
        pUg31xx->capData.rsoc = (_cap_u8_)target;

        pUg31xx->batteryInfo.NAC = (_upi_u16_)pUg31xx->capData.rm;
        pUg31xx->batteryInfo.LMD = (_upi_u16_)pUg31xx->capData.fcc;
        pUg31xx->batteryInfo.RSOC = (_upi_u16_)pUg31xx->capData.rsoc;
        UG31_LOGI("[%s]: %d / %d = %d\n", __func__, pUg31xx->batteryInfo.NAC, pUg31xx->batteryInfo.LMD, pUg31xx->batteryInfo.RSOC, target);
}

#ifndef uG31xx_OS_WINDOWS

#ifdef  uG31xx_BOOT_LOADER

static char *uboot_gauge = _UPI_NULL_;

/**
 * @brief uboot_initial
 *
 *  initial procedure in uBoot
 *
 * @para  pGGB  address of GGB data
 * @para  force_reset set 1 to force uG31xx driver reset as first power on
 * @return  0 if success
 */
int uboot_initial(char *pGGB, unsigned char force_reset)
{
        GGSTATUS rtn;

        if(uboot_gauge != _UPI_NULL_) {
                rtn = upiGG_UnInitial(&uboot_gauge);
                if(rtn != UG_SUCCESS) {
                        return (rtn);
                }
        }

        rtn = upiGG_Initial(&uboot_gauge, (GGBX_FILE_HEADER *)pGGB, force_reset);
        if(rtn != UG_INIT_SUCCESS) {
                return ((int)rtn);
        }
        return (0);
}

/**
 * @brief uboot_uninitial
 *
 *  uninitialize procedure in uBoot
 *
 * @return  0 if success
 */
int uboot_uninitial(void)
{
        GGSTATUS rtn;

        rtn = upiGG_UnInitial(&uboot_gauge);
        if(rtn != UG_SUCCESS) {
                return (rtn);
        }
        return (0);
}

/**
 * @brief uboot_update
 *
 *  Update battery information in uBoot
 *
 * @return  0 if success
 */
int uboot_update(void)
{
        GGSTATUS rtn;
        GG_DEVICE_INFO *obj;

        obj = _UPI_NULL_;
        obj = upi_malloc(sizeof(GG_DEVICE_INFO));
        if(obj == _UPI_NULL_) {
                return (-1);
        }

        rtn = upiGG_ReadDeviceInfo(uboot_gauge, obj);
        if(rtn != UG_READ_DEVICE_INFO_SUCCESS) {
                return (rtn);
        }
        return (0);
}

/**
 * @brief uboot_get_voltage
 *
 *  Read voltage
 *
 * @return  battery voltage in mV
 */
int uboot_get_voltage(void)
{
        struct ug31xx_data *pUg31xx;

        pUg31xx = (struct ug31xx_data *)uboot_gauge;

        return ((int)pUg31xx->measData.bat1Voltage);
}

/**
 * @brief uboot_get_current
 *
 *  Read current
 *
 * @return  current in mA
 */
int uboot_get_current(void)
{
        struct ug31xx_data *pUg31xx;

        pUg31xx = (struct ug31xx_data *)uboot_gauge;

        return ((int)pUg31xx->measData.curr);
}

/**
 * @brief uboot_get_internal_temperature
 *
 *  Read internal temperature
 *
 * @return  internal temperature in 0.1oC
 */
int uboot_get_internal_temperature(void)
{
        struct ug31xx_data *pUg31xx;

        pUg31xx = (struct ug31xx_data *)uboot_gauge;

        return ((int)pUg31xx->measData.intTemperature);
}

/**
 * @brief uboot_get_external_temperature
 *
 *  Read external temperature
 *
 * @return  external temperature in 0.1oC
 */
int uboot_get_external_temperature(void)
{
        struct ug31xx_data *pUg31xx;

        pUg31xx = (struct ug31xx_data *)uboot_gauge;

        return ((int)pUg31xx->measData.extTemperature);
}

/**
 * @brief uboot_get_remaining_capacity
 *
 *  Read remaining capacity
 *
 * @return  remaining capacity in mAh
 */
int uboot_get_remaining_capacity(void)
{
        struct ug31xx_data *pUg31xx;

        pUg31xx = (struct ug31xx_data *)uboot_gauge;

        return ((int)pUg31xx->batteryInfo.NAC);
}

/**
 * @brief uboot_get_full_charge_capacity
 *
 *  Read full charge capacity
 *
 * @return  full charge capacity in mAh
 */
int uboot_get_full_charge_capacity(void)
{
        struct ug31xx_data *pUg31xx;

        pUg31xx = (struct ug31xx_data *)uboot_gauge;

        return ((int)pUg31xx->batteryInfo.LMD);
}

/**
 * @brief uboot_get_relative_state_of_charge
 *
 *  Read relative state of charge
 *
 * @return  relative state of charge in %
 */
int uboot_get_relative_state_of_charge(void)
{
        struct ug31xx_data *pUg31xx;

        pUg31xx = (struct ug31xx_data *)uboot_gauge;

        return ((int)pUg31xx->batteryInfo.RSOC);
}

/**
 * @brief uboot_get_full_charge_status
 *
 *  Read full charge status
 *
 * @return  1 if full charge reached
 */
int uboot_get_full_charge_status(void)
{
        if(ug31_uboot_sts & UPI_BOOT_STATUS_FC) {
                return (1);
        }
        return (0);
}

/**
 * @brief uboot_get_rsense
 *
 *  Get R-Sense value
 *
 * @return  R-Sense in mOhm
 */
int uboot_get_rsense(void)
{
        struct ug31xx_data *pUg31xx;

        pUg31xx = (struct ug31xx_data *)uboot_gauge;

        return ((int)pUg31xx->cellParameter.rSense);
}

/**
 * @brief uboot_set_rsense
 *
 *  Set R-Sense value
 *
 * @para  rsense  R-Sense in mOhm
 * @return  0 if success
 */
int uboot_set_rsense(int rsense)
{
        struct ug31xx_data *pUg31xx;

        pUg31xx = (struct ug31xx_data *)uboot_gauge;
        pUg31xx->cellParameter.rSense = (_upi_u8_)rsense;
        return (0);
}

struct ug31xx_uboot_interface ug31xx_uboot_module = {
        .initial                      = uboot_initial,
        .uninitial                    = uboot_uninitial,
        .update                       = uboot_update,

        .get_voltage                  = uboot_get_voltage,
        .get_current                  = uboot_get_current,
        .get_internal_temperature     = uboot_get_internal_temperature,
        .get_external_temperature     = uboot_get_external_temperature,
        .get_remaining_capacity       = uboot_get_remaining_capacity,
        .get_full_charge_capacity     = uboot_get_full_charge_capacity,
        .get_relative_state_of_charge = uboot_get_relative_state_of_charge,
        .get_full_charge_status       = uboot_get_full_charge_status,
        .get_rsense                   = uboot_get_rsense,

        .set_rsense                   = uboot_set_rsense,
};

#else   ///< else of uG31xx_BOOT_LOADER

#ifdef  ANDROID_SHELL_ALGORITHM

/**
 * @brief upi_lib_debug_switch
 *
 *  Debug switch API for user space program
 *
 * @para  op_options  operation from kernel
 * @return  NULL
 */
void upi_lib_debug_switch(unsigned char op_option)
{
        upiGG_DebugSwitch(LKM_OPTIONS_DEBUG_LEVEL(op_option));
}

static bool upi_lib_first_update_capacity = true;

/**
 * @brief upi_lib_update_capacity
 *
 *  Capacity algorithm API for user space program
 *
 * @para  obj address of memory buffer from kernel
 * @return  NULL
 */
void upi_lib_update_capacity(char *obj)
{
        struct ug31xx_data *pUg31xx;

        pUg31xx = (struct ug31xx_data *)obj;
        UG31_LOGI("[%s]: Time Tick = %d (%d)\n", __func__, pUg31xx->measData.lastTimeTick, pUg31xx->measData.lastCounter);

        if(upi_lib_first_update_capacity == true) {
                upi_lib_first_update_capacity = false;
                UG31_LOGE("[%s]: First update capacity (%d -> %d)\n", __func__,
                          pUg31xx->measData.deltaTimeDaemon,
                          pUg31xx->measData.deltaTime);
                pUg31xx->measData.deltaTimeDaemon = pUg31xx->measData.deltaTime;
        }
        upiGG_ShellUpdateCapacity(obj);
}

/**
 * @brief upi_lib_backup_data
 *
 *  Backup data operation for user space program
 *
 * @para  obj address of memory buffer from kernel
 * @para  backup_file address of backup filename
 * @para  suspend_file  address of suspend filename
 * @return  0 if success, 1 if driver version mismatch, -1 if not ready
 */
int upi_lib_backup_data(char *obj, char *backup_file, char *suspend_file)
{
        int rtn;
        struct ug31xx_data *pUg31xx;

        pUg31xx = (struct ug31xx_data *)obj;
        UG31_LOGI("[%s]: Time Tick = %d (%d)\n", __func__, pUg31xx->measData.lastTimeTick, pUg31xx->measData.lastCounter);

        upiGG_BackupFileSwitch(_UPI_TRUE_);
        rtn = upiGG_BackupFileCheck(obj, backup_file, suspend_file);
        upiGG_BackupFileSwitch(_UPI_FALSE_);

        if(rtn == UG_CAP_DATA_VERSION_MISMATCH) {
                UG31_LOGE("[%s]: Driver version mismatched.\n", __func__);
                return (1);
        }
        if(rtn == UG_CAP_DATA_NOT_READY) {
                UG31_LOGE("[%s]: Driver backup data not ready.\n", __func__);
                return (-1);
        }
        return (0);
}

/**
 * @brief upi_lib_malloc_memory
 *
 *  Malloc memory buffer for userspace program
 *
 * @para  obj address of memory buffer pointer
 * @return  size of memory buffer
 */
int upi_lib_malloc_memory(char **obj)
{
        if((*obj) != NULL) {
                upi_free(*obj);
        }

        *obj = (char *)upi_malloc(sizeof(struct ug31xx_data));
        return (((*obj) == NULL) ? 0 : sizeof(struct ug31xx_data));
}

/**
 * @brief upi_lib_get_memory_size
 *
 *  Get memory size used by userspace program
 *
 * @return  size of memory
 */
int upi_lib_get_memory_size(void)
{
        int total_size;
        int rtn;

        total_size = 0;

        rtn = (int)UpiGetBackupMemorySize();
        total_size = total_size + rtn;

        rtn = (int)UpiGetCapacityMemorySize();
        total_size = total_size + rtn;

        rtn = (int)UpiGetMeasurementMemorySize();
        total_size = total_size + rtn;

        UG31_LOGD("[%s]: Total memory size = %d\n", __func__, total_size);
        return (total_size);
}

#else   ///< else of ANDROID_SHELL_ALGORITHM

enum  LKM_OPERATION_MODE {
        LKM_OPERATION_MODE_NORMAL = 0,
        LKM_OPERATION_MODE_ENTER_SUSPEND,
        LKM_OPERATION_MODE_SUSPEND,
        LKM_OPERATION_MODE_FORCE_EXIT_SUSPEND = 20,
};

#define LKM_CHECK_BACKUP_FILE_INTERVAL    (20)
#define LKM_SUSPEND_UPDATE_INTERVAL       (1)
#define LKM_AVG_TEMPERATURE_COUNT         (4)
#define LKM_AVG_TEMPERATURE_INIT_WITH_25  (LKM_AVG_TEMPERATURE_COUNT/2)
#define LKM_MAX_OPERATION_FAIL_CNT        (3)

static _upi_u8_ lkm_alarm_status;
static _upi_u8_ lkm_battery_removed;
static _upi_u8_ lkm_chk_backup_file_interval;
static char *lkm_version_string = _UPI_NULL_;
static _upi_u8_ lkm_operation_mode;
static _upi_u8_ lkm_suspend_update_delay;
static char *lkm_gauge = _UPI_NULL_;
static char *lkm_backup_filename = _UPI_NULL_;
static char *lkm_suspend_filename = _UPI_NULL_;
static unsigned char lkm_options = 0;
static _upi_bool_ lkm_dc_in_before_suspend = _UPI_FALSE_;
static _upi_s16_ lkm_avg_ext_temperature_buf[LKM_AVG_TEMPERATURE_COUNT];
static char *lkm_shell_ap_name = _UPI_NULL_;
static _upi_u8_ lkm_operation_fail_cnt = 0;
static _upi_u32_ lkm_last_update_time_tag = 0;
static _upi_u8_ lkm_polling_init_cnt = 0;

#define GGB_VERSION_LENGTH      (4)

static char lkm_string_table[] = {
        '0',
        '1',
        '2',
        '3',
        '4',
        '5',
        '6',
        '7',
        '8',
        '9',
        'a',
        'b',
        'c',
        'd',
        'e',
        'f',
};

/**
 * @brief lkm_set_version
 *
 *  Set version string
 *
 * @para  ug31xx  address of struct ug31xx_data
 * @return  NULL
 */
void lkm_set_version(struct ug31xx_data *ug31xx)
{
        _upi_u32_ size;
        _upi_u32_ idxVerNote;
        _upi_u32_ idxGgb;
        _upi_u32_ idxCell;
        _upi_u8_ idx;
        _upi_u16_ value;
        _upi_u8_ fwOffset;
        _upi_u8_ bkOffset;

        idxVerNote = upi_strlen(UG31XX_DRIVER_VERSION_STR) + 1;
        idxGgb = idxVerNote + upi_strlen(UG31XX_DRIVER_RELEASE_NOTE) + 1;
        idxCell = idxGgb + GGB_VERSION_LENGTH + 1;
        size = idxCell + CELL_PARAMETER_STRING_LENGTH + CELL_PARAMETER_STRING_LENGTH + 1;

        if(lkm_version_string != _UPI_NULL_) {
                upi_free(lkm_version_string);
        }
        lkm_version_string = (char *)upi_malloc(size);

        /// [AT-PM] : Set driver version ; 11/13/2013
        upi_memcpy(lkm_version_string, UG31XX_DRIVER_VERSION_STR, upi_strlen(UG31XX_DRIVER_VERSION_STR));
        lkm_version_string[upi_strlen(UG31XX_DRIVER_VERSION_STR)] = ':';
        upi_memcpy(&lkm_version_string[idxVerNote], UG31XX_DRIVER_RELEASE_NOTE, upi_strlen(UG31XX_DRIVER_RELEASE_NOTE));
        lkm_version_string[idxVerNote + upi_strlen(UG31XX_DRIVER_RELEASE_NOTE)] = '-';


        /// [AT-PM] : Set ggb version ; 11/15/2013
        idx = 0;
        fwOffset = 0;
        bkOffset = 12;
        while(idx < GGB_VERSION_LENGTH) {
                value = ug31xx->cellParameter.ggb_version;

                value = value << fwOffset;
                value = value >> bkOffset;

                lkm_version_string[idxGgb + idx] = lkm_string_table[value];

                idx = idx + 1;
                fwOffset = fwOffset + 4;
        }
        lkm_version_string[idxGgb + idx] = '-';

        /// [AT-PM] : Set cell manufacturer ; 11/13/2013
        idx = 0;
        while(idx < CELL_PARAMETER_STRING_LENGTH) {
                lkm_version_string[idxCell] = ug31xx->cellParameter.customerSelfDef[idx];
                idxCell = idxCell + 1;
                idx = idx + 1;

                if(ug31xx->cellParameter.customerSelfDef[idx] == 0) {
                        break;
                }
        }

        lkm_version_string[idxCell] = ':';
        idxCell = idxCell + 1;

        /// [AT-PM] : Set cell type number ; 11/13/2013
        idx = 0;
        while(idx < CELL_PARAMETER_STRING_LENGTH) {
                lkm_version_string[idxCell] = ug31xx->cellParameter.projectSelfDef[idx];
                idxCell = idxCell + 1;
                idx = idx + 1;

                if(ug31xx->cellParameter.projectSelfDef[idx] == 0) {
                        break;
                }
        }

        lkm_version_string[idxCell] = '\0';
        UG31_LOGE("[%s]: Version = %s\n", __func__, lkm_version_string);
}

/**
 * @brief lkm_initial
 *
 *  Initialization procedure for linux kernel module
 *
 * @para  ggb address of ggb data array
 * @para  cable cable status
 * @return  0 if success
 */
int lkm_initial(char *ggb, unsigned char cable)
{
        GGSTATUS rtn;
        struct ug31xx_data *ug31xx;
        int idx;

        upiGG_DebugSwitch(LKM_OPTIONS_DEBUG_LEVEL(lkm_options));

        lkm_last_update_time_tag = GetSysTickCount();

        if(lkm_gauge != _UPI_NULL_) {
                rtn = upiGG_UnInitial(&lkm_gauge);
        }

#ifdef  UG31XX_CELL_REPLACE_TEST

        rtn = upiGG_Initial(&lkm_gauge, (GGBX_FILE_HEADER *)ggb, LKM_OPTIONS_FORCE_RESET);

#else   ///< else of UG31XX_CELL_REPLACE_TEST

        rtn = upiGG_Initial(&lkm_gauge, (GGBX_FILE_HEADER *)ggb, (lkm_options & LKM_OPTIONS_FORCE_RESET));

#endif  ///< end of UG31XX_CELL_REPLACE_TEST

        ug31xx = (struct ug31xx_data *)lkm_gauge;
        ug31xx->Options = (_upi_u8_)lkm_options;

        /// [AT-PM] : Check 0% RSOC ; 01/14/2014
        if((ug31xx->measData.bat1Voltage > ug31xx->cellParameter.edv1Voltage) &&
            (cable == UG31XX_CABLE_IN) &&
            (ug31xx->batteryInfo.RSOC == 0)) {
                upiGG_SetCapacity(lkm_gauge, 1);
        }

        lkm_set_version(ug31xx);

        lkm_alarm_status = 0;
        lkm_battery_removed = UPI_UG31XX_BATTERY_INSERTED;
        lkm_chk_backup_file_interval = LKM_CHECK_BACKUP_FILE_INTERVAL;
        lkm_operation_mode = LKM_OPERATION_MODE_NORMAL;
        lkm_suspend_update_delay = 0;
        lkm_options = lkm_options & (~LKM_OPTIONS_FORCE_RESET);
        lkm_polling_init_cnt = 0;

        /// [AT-PM] : Initialize average external temperature buffer ; 11/27/2013
        idx = 0;
        while(idx < LKM_AVG_TEMPERATURE_COUNT) {
                if(idx < LKM_AVG_TEMPERATURE_INIT_WITH_25) {
                        lkm_avg_ext_temperature_buf[idx] = (_upi_s16_)ug31xx->measData.extTemperature;
                } else {
                        lkm_avg_ext_temperature_buf[idx] = 250;
                }
                idx = idx + 1;
        }
        return ((rtn == UG_INIT_SUCCESS) ? 0 : -1);
}

/**
 * @brief lkm_uninitial
 *
 *  Uninitialization procedure for linux kernel module
 *
 * @return  0 if success
 */
int lkm_uninitial(void)
{
        GGSTATUS rtn;

        rtn = UG_SUCCESS;
        if(lkm_gauge != _UPI_NULL_) {
                rtn = upiGG_UnInitial(&lkm_gauge);
        }
        if(lkm_version_string != _UPI_NULL_) {
                upi_free(lkm_version_string);
        }
        return ((rtn == UG_SUCCESS) ? 0 : -1);
}

/**
 * @brief lkm_check_fail_cnt
 *
 *  Check operation fail count
 *
 * @para  reset set _UPI_TRUE_ to reset fail count
 * @return  0 if not exceed MAX count
 */
int lkm_check_fail_cnt(_upi_bool_ reset)
{
        if(reset == _UPI_TRUE_) {
                lkm_operation_fail_cnt = 0;
                return (0);
        }

        lkm_operation_fail_cnt = lkm_operation_fail_cnt + 1;
        if(lkm_operation_fail_cnt < LKM_MAX_OPERATION_FAIL_CNT) {
                return (0);
        }
        return (1);
}

/**
 * @brief lkm_suspend
 *
 *  Suspend procedure for linux kernel module
 *
 * @para  dc_in set 1 if dc is in
 * @return  0 if success
 */
int lkm_suspend(char dc_in)
{
        GGSTATUS rtn;

        upiGG_BackupFileSwitch(_UPI_FALSE_);

        lkm_last_update_time_tag = GetSysTickCount();

        lkm_dc_in_before_suspend = (dc_in == 1) ? _UPI_TRUE_ : _UPI_FALSE_;

        if((lkm_operation_mode == LKM_OPERATION_MODE_NORMAL) ||
            (lkm_operation_mode == LKM_OPERATION_MODE_FORCE_EXIT_SUSPEND)) {
                lkm_operation_mode = LKM_OPERATION_MODE_ENTER_SUSPEND;
                lkm_suspend_update_delay = 0;
                upiGG_InternalSuspendMode(lkm_gauge, _UPI_TRUE_);
                rtn = upiGG_PreSuspend(lkm_gauge);
                rtn = lkm_check_fail_cnt((rtn == UG_READ_DEVICE_INFO_SUCCESS) ? _UPI_TRUE_ : _UPI_FALSE_);
                return (rtn);
        }
        return (0);
}

/**
 * @brief lkm_update_avg_external_temperature
 *
 *  Update average external temperature buffer
 *
 * @para  extTemp current measured external temperature
 * @return  NULL
 */
void lkm_update_avg_external_temperature(_upi_s16_ extTemp)
{
        _upi_u8_ tmp;

        tmp = LKM_AVG_TEMPERATURE_COUNT - 1;
        while(tmp) {
                lkm_avg_ext_temperature_buf[tmp] = lkm_avg_ext_temperature_buf[tmp - 1];
                tmp = tmp - 1;
        }
        lkm_avg_ext_temperature_buf[tmp] = extTemp;
}

/**
 * @brief lkm_update_procedure
 *
 *  Update battery information procedure
 *
 * @para  user_space_response true if user space algorithm has responsed
 * @return  GGSTATUS
 */
GGSTATUS lkm_update_procedure(_upi_bool_ user_space_response)
{
        GGSTATUS rtn;
        GG_DEVICE_INFO *devInfo;
        GG_CAPACITY *devCapacity;
        struct ug31xx_data *ug31xx;

        ug31xx = (struct ug31xx_data *)lkm_gauge;

        devInfo = (GG_DEVICE_INFO *)upi_malloc(sizeof(GG_DEVICE_INFO));
        if(devInfo == _UPI_NULL_) {
                return (-1);
        }
        devCapacity = (GG_CAPACITY *)upi_malloc(sizeof(GG_CAPACITY));
        if(devCapacity == _UPI_NULL_) {
                upi_free(devInfo);
                return (-1);
        }

        rtn = upiGG_ReadDeviceInfo(lkm_gauge, devInfo);
        if(rtn == UG_READ_DEVICE_INFO_SUCCESS) {
#ifndef UG31XX_SHELL_ALGORITHM

                upiGG_ReadCapacity(lkm_gauge, devCapacity);

#endif  ///< end of UG31XX_SHELL_ALGORITHM

                if(user_space_response == _UPI_FALSE_) {
                        upiGG_ShellUpdateCC(lkm_gauge);
                        UG31_LOGE("[%s]: User space daemon no response.\n", __func__);
                }

                rtn = upiGG_GetAlarmStatus(lkm_gauge, &lkm_alarm_status);
        }

#ifndef UG31XX_SHELL_ALGORITHM

        UG31_LOGI("[%s]: %d / %d = %d\n", __func__, devCapacity->NAC, devCapacity->LMD, devCapacity->RSOC);

#endif  ///< end of UG31XX_SHELL_ALGORITHM

        if(rtn == UG_MEAS_FAIL_BATTERY_REMOVED) {
                lkm_battery_removed = UPI_UG31XX_BATTERY_REMOVED;
        } else {
                lkm_battery_removed = UPI_UG31XX_BATTERY_INSERTED;
        }

        /// [AT-PM] : Update average external temperature buffer ; 11/27/2013
        lkm_update_avg_external_temperature((_upi_s16_)ug31xx->measData.instExtTemperature);

        upi_free(devInfo);
        upi_free(devCapacity);
        return (rtn);
}

/**
 * @brief lkm_resume
 *
 *  Resume procedure for linux kernel module
 *
 * @para  user_space_response true if user space algorithm has responsed
 * @return  0 if success
 */
int lkm_resume(char user_space_response)
{
        GGSTATUS rtn;
        GG_DEVICE_INFO *devInfo;

        lkm_last_update_time_tag = GetSysTickCount();

        if(lkm_operation_mode == LKM_OPERATION_MODE_ENTER_SUSPEND) {
                lkm_operation_mode = LKM_OPERATION_MODE_SUSPEND;
                lkm_suspend_update_delay = 0;
                lkm_chk_backup_file_interval = 0;

                devInfo = (GG_DEVICE_INFO *)upi_malloc(sizeof(GG_DEVICE_INFO));
                if(devInfo == _UPI_NULL_) {
                        return (-1);
                }

                rtn = upiGG_Wakeup(lkm_gauge, lkm_dc_in_before_suspend);
                if(rtn == UG_READ_DEVICE_INFO_SUCCESS) {
                        rtn = upiGG_GetAlarmStatus(lkm_gauge, &lkm_alarm_status);
                }

                upiGG_InternalSuspendMode(lkm_gauge, _UPI_FALSE_);

                if(rtn == UG_MEAS_FAIL_BATTERY_REMOVED) {
                        lkm_battery_removed = UPI_UG31XX_BATTERY_REMOVED;
                } else {
                        lkm_battery_removed = UPI_UG31XX_BATTERY_INSERTED;
                }

                upi_free(devInfo);
                rtn = lkm_check_fail_cnt((rtn == UG_READ_DEVICE_ALARM_SUCCESS) ? _UPI_TRUE_ : _UPI_FALSE_);
                return (rtn);
        }

        if(lkm_operation_mode >= LKM_OPERATION_MODE_SUSPEND) {
                UG31_LOGI("[%s]: RESUME -> %d / %d\n", __func__, lkm_suspend_update_delay, lkm_operation_mode);

                lkm_suspend_update_delay = lkm_suspend_update_delay + 1;
                if(lkm_suspend_update_delay >= LKM_SUSPEND_UPDATE_INTERVAL) {
                        lkm_suspend_update_delay = 0;
                        lkm_operation_mode = lkm_operation_mode + 1;

                        rtn = lkm_update_procedure((user_space_response == UG31XX_USER_SPACE_RESPONSE) ? _UPI_TRUE_ : _UPI_FALSE_);
                        rtn = lkm_check_fail_cnt((rtn == UG_READ_DEVICE_ALARM_SUCCESS) ? _UPI_TRUE_ : _UPI_FALSE_);
                        return (rtn);
                }
        }
        return (0);
}

/**
 * @brief lkm_shutdown
 *
 *  Shutdown procedure for linux kernel module
 *
 * @returen 0 if success
 */
int lkm_shutdown(void)
{
        GGSTATUS rtn;

        rtn = upiGG_PrePowerOff(lkm_gauge);
        return ((rtn == UG_READ_DEVICE_INFO_SUCCESS) ? 0 : -1);
}

/**
 * @brief lkm_update
 *
 *  Update battery information procedure for linux kernel module
 *
 * @para  user_space_response true if user space algorithm has responsed
 * @return  0 if success
 */
int lkm_update(char user_space_response)
{
        GGSTATUS rtn;
        int oldRsoc;
        struct ug31xx_data *ug31xx;

        lkm_last_update_time_tag = GetSysTickCount();

        if(lkm_operation_mode >= LKM_OPERATION_MODE_SUSPEND) {
                lkm_operation_mode = LKM_OPERATION_MODE_NORMAL;
        }

        upiGG_DebugSwitch(LKM_OPTIONS_DEBUG_LEVEL(lkm_options));
        upiGG_ReverseCurrent(lkm_gauge, ((lkm_options & LKM_OPTIONS_ENABLE_REVERSE_CURRENT) ? _UPI_TRUE_ : _UPI_FALSE_));

        rtn = lkm_update_procedure((user_space_response == UG31XX_USER_SPACE_RESPONSE) ? _UPI_TRUE_ : _UPI_FALSE_);

        /// [AT-PM] : Check backup file if RSOC is changed ; 05/16/2013
        ug31xx = (struct ug31xx_data *)lkm_gauge;
        oldRsoc = (int)ug31xx->batteryInfo.RSOC;
        if(oldRsoc != ((int)ug31xx->batteryInfo.RSOC)) {
                lkm_chk_backup_file_interval = LKM_CHECK_BACKUP_FILE_INTERVAL;
        }
        rtn = lkm_check_fail_cnt((rtn == UG_READ_DEVICE_ALARM_SUCCESS) ? _UPI_TRUE_ : _UPI_FALSE_);
        return (rtn);
}

/**
 * @brief lkm_reset
 *
 *  Reset linux kernel module
 *
 * @para  ggb address of ggb data
 * @return  0 if success
 */
int lkm_reset(char *ggb)
{
        GGSTATUS rtn;
        struct ug31xx_data *ug31xx;

        upiGG_DebugSwitch(LKM_OPTIONS_DEBUG_LEVEL(lkm_options));

        lkm_last_update_time_tag = GetSysTickCount();

        upiGG_UnInitial(&lkm_gauge);
        rtn = upiGG_Initial(&lkm_gauge, (GGBX_FILE_HEADER *)ggb, _UPI_TRUE_);

        ug31xx = (struct ug31xx_data *)lkm_gauge;
        ug31xx->Options = (_upi_u8_)lkm_options;

        lkm_set_version(ug31xx);

        lkm_alarm_status = 0;
        lkm_battery_removed = UPI_UG31XX_BATTERY_INSERTED;
        lkm_chk_backup_file_interval = LKM_CHECK_BACKUP_FILE_INTERVAL;
        lkm_operation_mode = LKM_OPERATION_MODE_NORMAL;
        lkm_suspend_update_delay = LKM_CHECK_BACKUP_FILE_INTERVAL;
        return ((rtn == UG_INIT_SUCCESS) ? 0 : -1);
}

/**
 * @brief lkm_get_voltage
 *
 *  Get voltage for linux kernel module
 *
 * @return  voltage
 */
int lkm_get_voltage(void)
{
        struct ug31xx_data *ug31xx;

        ug31xx = (struct ug31xx_data *)lkm_gauge;

        return ((int)ug31xx->measData.bat1Voltage);
}

/**
 * @brief lkm_get_current
 *
 *  Get current for linux kernel module
 *
 * @return  current
 */
int lkm_get_current(void)
{
        struct ug31xx_data *ug31xx;

        ug31xx = (struct ug31xx_data *)lkm_gauge;

        return ((int)ug31xx->measData.curr);
}

/**
 * @brief lkm_get_external_temperature
 *
 *  Get external temperature for linux kernel module
 *
 * @return  external temperature
 */
int lkm_get_external_temperature(void)
{
        struct ug31xx_data *ug31xx;

        ug31xx = (struct ug31xx_data *)lkm_gauge;

        return ((int)ug31xx->measData.extTemperature);
}

/**
 * @brief lkm_get_internal_temperature
 *
 *  Get internal temperature for linux kernel module
 *
 * @return  internal temperature
 */
int lkm_get_internal_temperature(void)
{
        struct ug31xx_data *ug31xx;

        ug31xx = (struct ug31xx_data *)lkm_gauge;

        return ((int)ug31xx->measData.intTemperature);
}

/**
 * @brief lkm_get_remaining_capacity
 *
 *  Get remaining capacity for linux kernel module
 *
 * @return  remaining capacity
 */
int lkm_get_remaining_capacity(void)
{
        struct ug31xx_data *ug31xx;

        ug31xx = (struct ug31xx_data *)lkm_gauge;

        return ((int)ug31xx->batteryInfo.NAC);
}

/**
 * @brief lkm_get_full_charge_capacity
 *
 *  Get full charge capacity for linux kernel module
 *
 * @return  full charge capacity
 */
int lkm_get_full_charge_capacity(void)
{
        struct ug31xx_data *ug31xx;

        ug31xx = (struct ug31xx_data *)lkm_gauge;

        return ((int)ug31xx->batteryInfo.LMD);
}

#define LKM_RSOC_REMAP_TO_100     (99)
#define LKM_RSOC_REMAP_TO_0       (5)

/**
 * @brief lkm_get_relative_state_of_charge
 *
 *  Get relative state of charge for linux kernel module
 *
 * @return  relative state of charge
 */
int lkm_get_relative_state_of_charge(void)
{
        struct ug31xx_data *ug31xx;
        int rsoc;

        ug31xx = (struct ug31xx_data *)lkm_gauge;

        rsoc = (int)ug31xx->batteryInfo.RSOC;
        if(!(lkm_options & LKM_OPTIONS_RSOC_REMAP)) {
                return (rsoc);
        }

        /// [AT-PM] : RSOC >= LKM_RSOC_REMAP_TO_100 -> report to 100% ; 02/20/2014
        if(rsoc >= LKM_RSOC_REMAP_TO_100) {
                UG31_LOGI("[%s]: Remap to 100 from %d\n", __func__,
                          rsoc);
                return (100);
        }

        /// [AT-PM] : RSOC <= LKM_RSOC_REMAP_TO_0 -> report to 0% ; 02/20/2014
        if(rsoc <= LKM_RSOC_REMAP_TO_0) {
                UG31_LOGI("[%s]: Remap to 0 from %d\n", __func__,
                          rsoc);
                return (0);
        }

        rsoc = rsoc - LKM_RSOC_REMAP_TO_0;
        rsoc = rsoc*CONST_PERCENTAGE;
        rsoc = rsoc/(LKM_RSOC_REMAP_TO_100 - LKM_RSOC_REMAP_TO_0);
        UG31_LOGI("[%s]: Remap to %d from %d\n", __func__,
                  rsoc,
                  ug31xx->batteryInfo.RSOC);
        return (rsoc);
}

/**
 * @brief lkm_get_version
 *
 *  Get version string for linux kernel module
 *
 * @return  version address
 */
char* lkm_get_version(void)
{
        return (lkm_version_string);
}

#define UPI_POLLING_TIME_CONST          (60)
#define UPI_POLLING_TIME_NEAR_FULL      (10)
#define UPI_POLLING_TIME_NEAR_FULL_SOC  (90)
#define UPI_POLLING_TIME_MINIMUM        (5)
#define UPI_POLLING_TIME_MAXIMUM        (30)
#define UPI_POLLING_TIME_NOT_READY      (UPI_POLLING_TIME_MINIMUM - 1)
#define UPI_POLLING_TIME_VER_MISMATCH   (UPI_POLLING_TIME_NOT_READY - 1)
#define UPI_POLLING_TIME_FAIL_RETRY     (UPI_POLLING_TIME_VER_MISMATCH - 1)
#define UPI_POLLING_TIME_NEAR_OT        (10)
#define UPI_POLLING_TIME_NEAR_UT        (10)
#define UPI_POLLING_TIME_OT             (5)
#define UPI_POLLING_TIME_UT             (5)
#define UPI_POLLING_TIME_NEAR_OT_THRD   (450)
#define UPI_POLLING_TIME_NEAR_UT_THRD   (50)
#define UPI_POLLING_TIME_OT_THRD        (500)
#define UPI_POLLING_TIME_UT_THRD        (0)
#define UPI_POLLING_TIME_INIT_THRD      (15)

_upi_s32_ polling_time_algorithm(void)
{
        struct ug31xx_data *ug31xx;
        _upi_s32_ polling_time;

        ug31xx = (struct ug31xx_data *)lkm_gauge;

        if(lkm_operation_fail_cnt != 0) {
                polling_time = UPI_POLLING_TIME_FAIL_RETRY;
        } else if(ug31xx->backupData.backupFileSts == BACKUP_FILE_STS_VERSION_MISMATCH) {
                polling_time = UPI_POLLING_TIME_VER_MISMATCH;
        } else if(ug31xx->backupData.backupFileSts <= BACKUP_FILE_STS_EXIST) {
                polling_time = UPI_POLLING_TIME_NOT_READY;
        } else {
                if(ug31xx->capData.fcSts == CAP_TRUE) {
                        polling_time = 60;
                } else if((ug31xx->measData.curr > 0) &&
                          ((ug31xx->batteryInfo.RSOC >= UPI_POLLING_TIME_NEAR_FULL_SOC) ||
                           (ug31xx->capData.tpTime > 0))) {
                        polling_time = UPI_POLLING_TIME_NEAR_FULL;
                } else {
                        if(ug31xx->capData.predictRsoc >= 50) {
                                polling_time = 60;
                        } else if(ug31xx->capData.predictRsoc >= 20) {
                                polling_time = 30;
                        } else if(ug31xx->capData.predictRsoc >= 5) {
                                polling_time = 10;
                        } else {
                                polling_time = 5;
                        }
                }

                if(polling_time < UPI_POLLING_TIME_MINIMUM) {
                        polling_time = UPI_POLLING_TIME_MINIMUM;
                }
                if(polling_time > UPI_POLLING_TIME_MAXIMUM) {
                        polling_time = UPI_POLLING_TIME_MAXIMUM;
                }
        }

        /// [AT-PM] : Check temperature range ; 11/28/2013
        if((ug31xx->measData.extTemperature >= UPI_POLLING_TIME_OT_THRD) &&
            (polling_time > UPI_POLLING_TIME_OT)) {
                polling_time = UPI_POLLING_TIME_OT;
        }
        if((ug31xx->measData.extTemperature >= UPI_POLLING_TIME_NEAR_OT_THRD) &&
            (polling_time > UPI_POLLING_TIME_NEAR_OT)) {
                polling_time = UPI_POLLING_TIME_NEAR_OT;
        }
        if((ug31xx->measData.extTemperature <= UPI_POLLING_TIME_UT_THRD) &&
            (polling_time > UPI_POLLING_TIME_UT)) {
                polling_time = UPI_POLLING_TIME_UT;
        }
        if((ug31xx->measData.extTemperature <= UPI_POLLING_TIME_NEAR_UT_THRD) &&
            (polling_time > UPI_POLLING_TIME_NEAR_UT)) {
                polling_time = UPI_POLLING_TIME_NEAR_UT;
        }

        lkm_polling_init_cnt = lkm_polling_init_cnt + 1;
        if(lkm_polling_init_cnt < UPI_POLLING_TIME_INIT_THRD) {
                polling_time = UPI_POLLING_TIME_NOT_READY;
                UG31_LOGI("[%s]: Force polling time to %d seconds. (%d)\n", __func__,
                          polling_time,
                          lkm_polling_init_cnt);
        } else {
                lkm_polling_init_cnt = UPI_POLLING_TIME_INIT_THRD;
        }
        UG31_LOGN("[%s]: Time interval = %d seconds (%d)\n", __func__,
                  polling_time,
                  lkm_polling_init_cnt);
        return (polling_time);
}

/**
 * @brief lkm_get_polling_time
 *
 *  Get polling time interval for linux kernel module
 *
 * @return  polling time interval
 */
int lkm_get_polling_time(void)
{
        struct ug31xx_data *ug31xx;
        _upi_s32_ polling_time;

        ug31xx = (struct ug31xx_data *)lkm_gauge;

        /// [AT-PM] : Check SOC range ; 12/10/2013
        if(ug31xx->capData.predictRsoc >= 50) {
                polling_time = 60;
        } else if(ug31xx->capData.predictRsoc >= 20) {
                polling_time = 30;
        } else if(ug31xx->capData.predictRsoc >= 5) {
                polling_time = 10;
        } else {
                polling_time = 5;
        }

        /// [AT-PM] : Check temperature range ; 11/28/2013
        if((ug31xx->measData.extTemperature >= UPI_POLLING_TIME_OT_THRD) &&
            (polling_time > UPI_POLLING_TIME_OT)) {
                polling_time = UPI_POLLING_TIME_OT;
        }
        if((ug31xx->measData.extTemperature >= UPI_POLLING_TIME_NEAR_OT_THRD) &&
            (polling_time > UPI_POLLING_TIME_NEAR_OT)) {
                polling_time = UPI_POLLING_TIME_NEAR_OT;
        }
        if((ug31xx->measData.extTemperature <= UPI_POLLING_TIME_UT_THRD) &&
            (polling_time > UPI_POLLING_TIME_UT)) {
                polling_time = UPI_POLLING_TIME_UT;
        }
        if((ug31xx->measData.extTemperature <= UPI_POLLING_TIME_NEAR_UT_THRD) &&
            (polling_time > UPI_POLLING_TIME_NEAR_UT)) {
                polling_time = UPI_POLLING_TIME_NEAR_UT;
        }
        UG31_LOGN("[%s]: Polling time = %d seconds\n", __func__, polling_time);
        return (polling_time);
}

/**
 * @brief lkm_get_module_ready
 *
 *  Get module is ready or not for linux kernel module
 *
 * @return  UPI_UG31XX_MODULE_READY or UPI_UG31XX_MODULE_NOT_READY
 */
int lkm_get_module_ready(void)
{
        struct ug31xx_data *ug31xx;

        ug31xx = (struct ug31xx_data *)lkm_gauge;

        return ((ug31xx->batteryInfo.Ready == UG_CAP_DATA_READY) ? UPI_UG31XX_MODULE_READY : UPI_UG31XX_MODULE_NOT_READY);
}

/**
 * @brief lkm_get_battery_removed
 *
 *  Get battery is removed or not for linux kernel module
 *
 * @return  UPI_UG31XX_BATTERY_REMOVED or UPI_UG31XX_BATTERY_INSERTED
 */
int lkm_get_battery_removed(void)
{
        return (lkm_battery_removed);
}

/**
 * @brief lkm_get_alarm_status
 *
 *  Get alarm status for linux kernel module
 *
 * @return  alarm status
 */
int lkm_get_alarm_status(void)
{
        GGSTATUS rtn;
        _upi_u8_ alarmSts;

        lkm_alarm_status = 0;
        alarmSts = 0;

        rtn = upiGG_GetAlarmStatus(lkm_gauge, &alarmSts);
        if(rtn == UG_READ_DEVICE_ALARM_SUCCESS) {
                if(alarmSts & ALARM_STATUS_UV) {
                        lkm_alarm_status = lkm_alarm_status | UPI_UG31XX_ALARM_STATUS_UV;
                }
                if(alarmSts & ALARM_STATUS_UET) {
                        lkm_alarm_status = lkm_alarm_status | UPI_UG31XX_ALARM_STATUS_UET;
                }
                if(alarmSts & ALARM_STATUS_OET) {
                        lkm_alarm_status = lkm_alarm_status | UPI_UG31XX_ALARM_STATUS_OET;
                }
        }
        return (lkm_alarm_status);
}

/**
 * @brief lkm_set_backup_file
 *
 *  Enable/disable backup file operation
 *
 * @para  enable  set 0 to disable backup file operation
 * @return  0 if success
 */
int lkm_set_backup_file(char enable)
{
        _upi_bool_ backupEN;

        backupEN = (enable == 0) ? _UPI_FALSE_ : _UPI_TRUE_;
        upiGG_BackupFileSwitch(backupEN);
        return (0);
}

/**
 * @brief lkm_chk_backup_file
 *
 *  Check backup file procedure
 *
 * @return  0 if success, 1 if version mismatched, -1 if fail
 */
int lkm_chk_backup_file(void)
{
        _upi_u8_ rtn;
        struct ug31xx_data *ug31xx;

        ug31xx = (struct ug31xx_data *)lkm_gauge;

        if(ug31xx->batteryInfo.Ready != UG_CAP_DATA_READY) {
                lkm_chk_backup_file_interval = LKM_CHECK_BACKUP_FILE_INTERVAL;
        }

        lkm_chk_backup_file_interval = lkm_chk_backup_file_interval + 1;
        UG31_LOGN("[%s]: lkm_chk_backup_file_interval = %d (%d)\n", __func__,
                  lkm_chk_backup_file_interval, LKM_CHECK_BACKUP_FILE_INTERVAL);
        if(lkm_chk_backup_file_interval > LKM_CHECK_BACKUP_FILE_INTERVAL) {
                rtn = upiGG_BackupFileCheck(lkm_gauge, lkm_backup_filename, lkm_suspend_filename);
                if(rtn == UG_CAP_DATA_VERSION_MISMATCH) {
                        UG31_LOGE("[%s]: Driver version mismatched.\n", __func__);
                        return (1);
                }
                if(rtn == UG_CAP_DATA_READY) {
                        lkm_chk_backup_file_interval = 0;
                }
        }
        return (0);
}

#define LKM_SET_CHARGER_FULL_STEP_THRESHOLD (95)
#define LKM_SET_CHARGER_FULL_STEP_VOLTAGE   (99)

/**
 * @brief lkm_set_charger_full
 *
 *  Set charger full signal
 *
 * @para  is_full set 0 if charger does not detect full
 * @return  0 if success
 */
int lkm_set_charger_full(char is_full)
{
        struct ug31xx_data *ug31xx;
        _upi_u32_ tmp32;

        ug31xx = (struct ug31xx_data *)lkm_gauge;

        if(is_full == UG31XX_CHARGER_NO_DETECTS_FULL) {
                /// [AT-PM] : No full charge detects ; 09/03/2013
                UpiSetChargerFull(&ug31xx->capData, _UPI_FALSE_);
                return (0);
        }

        if((is_full != UG31XX_CHARGER_DETECTS_FULL) &&
            (is_full != UG31XX_CHARGER_DETECTS_FULL_STEP) &&
            (is_full != UG31XX_CHARGER_DETECTS_FULL_CHECK) &&
            (is_full != UG31XX_TAPER_REACHED)) {
                return (0);
        }

        /// [AT-PM] : Full charge detects ; 09/03/2013
        ug31xx->batteryInfo.NAC = (_upi_u16_)ug31xx->capData.rm;
        ug31xx->batteryInfo.LMD = (_upi_u16_)ug31xx->capData.fcc;
        ug31xx->batteryInfo.RSOC = (_upi_u16_)ug31xx->capData.rsoc;

        /// [AT-PM] : Minimum voltage for charger detect full is 99% x TP Voltage ; 01/28/2014
        tmp32 = (_upi_u32_)ug31xx->cellParameter.TPVoltage;
        tmp32 = tmp32*LKM_SET_CHARGER_FULL_STEP_VOLTAGE/CONST_PERCENTAGE;

        if((is_full == UG31XX_CHARGER_DETECTS_FULL_STEP) &&
            (ug31xx->capData.predictRsoc < LKM_SET_CHARGER_FULL_STEP_THRESHOLD) &&
            (ug31xx->measData.bat1Voltage < tmp32)) {
                if(ug31xx->cellParameter.alarmEnable & CELL_PARAMETER_ALARM_EN_UET) {
                        if(ug31xx->measData.extTemperature <= ug31xx->cellParameter.uetAlarm) {
                                UG31_LOGE("[%s]: (%d) %d < %d -> No full status with UET and keep %d (T:%d)\n", __func__,
                                          is_full, ug31xx->capData.predictRsoc, LKM_SET_CHARGER_FULL_STEP_THRESHOLD,
                                          ug31xx->batteryInfo.RSOC, ug31xx->measData.extTemperature);
                                return (0);
                        }
                } else if(ug31xx->cellParameter.alarmEnable & CELL_PARAMETER_ALARM_EN_OET) {
                        if(ug31xx->measData.extTemperature >= ug31xx->cellParameter.oetAlarm) {
                                UG31_LOGE("[%s]: (%d) %d < %d -> No full status with OET and keep %d (T:%d)\n", __func__,
                                          is_full, ug31xx->capData.predictRsoc, LKM_SET_CHARGER_FULL_STEP_THRESHOLD,
                                          ug31xx->batteryInfo.RSOC, ug31xx->measData.extTemperature);
                                return (0);
                        }
                } else {
                        UG31_LOGE("[%s]: (%d) %d < %d -> No full status and keep %d (T:%d)\n", __func__,
                                  is_full, ug31xx->capData.predictRsoc, LKM_SET_CHARGER_FULL_STEP_THRESHOLD,
                                  ug31xx->batteryInfo.RSOC, ug31xx->measData.extTemperature);
                        return (0);
                }
        }

        if((is_full == UG31XX_CHARGER_DETECTS_FULL_CHECK) &&
            (ug31xx->capData.predictRsoc < LKM_SET_CHARGER_FULL_STEP_THRESHOLD) &&
            (ug31xx->measData.bat1Voltage < tmp32)) {
                if(ug31xx->cellParameter.alarmEnable & CELL_PARAMETER_ALARM_EN_UET) {
                        if(ug31xx->measData.extTemperature <= ug31xx->cellParameter.uetAlarm) {
                                UG31_LOGE("[%s]: (%d) %d < %d -> No full status with UET and keep %d (T:%d)\n", __func__,
                                          is_full, ug31xx->capData.predictRsoc, LKM_SET_CHARGER_FULL_STEP_THRESHOLD,
                                          ug31xx->batteryInfo.RSOC, ug31xx->measData.extTemperature);
                                return (0);
                        }
                } else if(ug31xx->cellParameter.alarmEnable & CELL_PARAMETER_ALARM_EN_OET) {
                        if(ug31xx->measData.extTemperature >= ug31xx->cellParameter.oetAlarm) {
                                UG31_LOGE("[%s]: (%d) %d < %d -> No full status with OET and keep %d (T:%d)\n", __func__,
                                          is_full, ug31xx->capData.predictRsoc, LKM_SET_CHARGER_FULL_STEP_THRESHOLD,
                                          ug31xx->batteryInfo.RSOC, ug31xx->measData.extTemperature);
                                return (0);
                        }
                } else {
                        UG31_LOGE("[%s]: (%d) %d < %d -> No full status and keep %d (T:%d)\n", __func__,
                                  is_full, ug31xx->capData.predictRsoc, LKM_SET_CHARGER_FULL_STEP_THRESHOLD,
                                  ug31xx->batteryInfo.RSOC, ug31xx->measData.extTemperature);
                        return (0);
                }
        }

        UpiSetChargerFull(&ug31xx->capData, _UPI_TRUE_);

        if((is_full == UG31XX_TAPER_REACHED) ||
            (is_full == UG31XX_CHARGER_DETECTS_FULL_STEP)) {
                /// [AT-PM] : Avoid RSOC jumping ; 09/03/2013
                ug31xx->capData.ggbTable = &ug31xx->cellTable;
                ug31xx->capData.ggbParameter = &ug31xx->cellParameter;
                ug31xx->capData.measurement = &ug31xx->measData;
                UpiSetChargerFullStep(&ug31xx->capData, &ug31xx->batteryInfo);
        }

        /// [AT-PM] : Reset coulomb counter ; 07/18/2013^M
        ug31xx->measData.sysData = &ug31xx->sysData;
        ug31xx->measData.otp = &ug31xx->otpData;
        UpiResetCoulombCounter(&ug31xx->measData);
        ug31xx->sysData.cycleCount = (_sys_u16_)ug31xx->measData.cycleCount;

        /// [AT-PM] : Save battery information to IC ; 01/31/2013
        ug31xx->sysData.rmFromIC = (_sys_u16_)ug31xx->capData.rm;
        ug31xx->sysData.fccFromIC = (_sys_u16_)ug31xx->capData.fcc;
        ug31xx->sysData.rsocFromIC = (_sys_u8_)ug31xx->capData.rsoc;
        ug31xx->sysData.tableUpdateIdxFromIC = ug31xx->capData.tableUpdateIdx;
        ug31xx->sysData.deltaCapFromIC = ug31xx->measData.lastDeltaCap;
        ug31xx->sysData.adc1ConvTime = ug31xx->measData.adc1ConvertTime;
        ug31xx->sysData.ccOffset = (_sys_s8_)ug31xx->measData.ccOffsetAdj;
        ug31xx->sysData.standbyDsgRatio = (_sys_u8_)ug31xx->capData.standbyDsgRatio;
        UpiSaveBatInfoTOIC(&ug31xx->sysData);

        ug31xx->batteryInfo.NAC = (_upi_u16_)ug31xx->capData.rm;
        ug31xx->batteryInfo.LMD = (_upi_u16_)ug31xx->capData.fcc;
        ug31xx->batteryInfo.RSOC = (_upi_u16_)ug31xx->capData.rsoc;
        UG31_LOGI("[%s]: %d / %d = %d (%d)\n", __func__, ug31xx->batteryInfo.NAC, ug31xx->batteryInfo.LMD, ug31xx->batteryInfo.RSOC, is_full);
        return (0);
}

/**
 * @brief lkm_set_taper_current
 *
 *  Set taper current
 *
 * @para  curr  new taper current
 * @return  0 if success
 */
int lkm_set_taper_current(int curr)
{
        struct ug31xx_data *ug31xx;

        ug31xx = (struct ug31xx_data *)lkm_gauge;

        ug31xx->cellParameter.TPCurrent = (_upi_u16_)curr;
        UG31_LOGE("[%s]: TPCurrent = %d\n", __func__,
                  curr);
        return (0);
}

/**
 * @brief lkm_get_taper_current
 *
 *  Get taper current setting
 *
 * @return  taper current
 */
int lkm_get_taper_current(void)
{
        struct ug31xx_data *ug31xx;

        ug31xx = (struct ug31xx_data *)lkm_gauge;

        return ((int)ug31xx->cellParameter.TPCurrent);
}

/**
 * @brief lkm_get_full_charge_status
 *
 *  Get full charge status is reached or not
 *
 * @return  1 if full charge status is reached
 */
int lkm_get_full_charge_status(void)
{
        struct ug31xx_data *ug31xx;
        _upi_bool_ fc_sts;

        ug31xx = (struct ug31xx_data *)lkm_gauge;

        if(ug31xx->capData.fcSts == CAP_TRUE) {
                return (1);
        }

        if(ug31xx->capData.fcStep100 == CAP_TRUE) {
                return (-1);
        }

        return (0);
}

/**
 * @brief lkm_get_design_capacity
 *
 *  Get design capacity
 *
 * @return  design capacity in mAh
 */
int lkm_get_design_capacity(void)
{
        struct ug31xx_data *ug31xx;

        ug31xx = (struct ug31xx_data *)lkm_gauge;
        return ((int)ug31xx->cellParameter.ILMD);
}

/**
 * @brief lkm_set_battery_temp_external
 *
 *  Set battery temperature from external
 *
 * @return  0 if success
 */
int lkm_set_battery_temp_external(void)
{
        upiGG_SetBatteryET(lkm_gauge);
        return (0);
}

/**
 * @brief lkm_set_battery_temp_internal
 *
 *  Set battery temperature from internal
 *
 * @return  0 if success
 */
int lkm_set_battery_temp_internal(void)
{
        upiGG_SetBatteryIT(lkm_gauge);
        return (0);
}

/**
 * @brief lkm_get_rsense
 *
 *  Get R-Sense value
 *
 * @return  R-Sense value in mOhm
 */
int lkm_get_rsense(void)
{
        struct ug31xx_data *ug31xx;

        ug31xx = (struct ug31xx_data *)lkm_gauge;
        return ((int)ug31xx->cellParameter.rSense);
}

/**
 * @brief lkm_set_rsense
 *
 *  Set R-Sense value
 *
 * @para  rsense  R-Sense value in mOhm
 * @return  0 if success
 */
int lkm_set_rsense(int rsense)
{
        struct ug31xx_data *ug31xx;

        ug31xx = (struct ug31xx_data *)lkm_gauge;

        ug31xx->cellParameter.rSense = (_upi_u8_)rsense;
        UG31_LOGE("[%s]: rSense = %d\n", __func__,
                  rsense);
        return (0);
}

/**
 * @brief lkm_get_predict_rsoc
 *
 *  Get predicted RSOC
 *
 * @return  predicted RSOC in %
 */
int lkm_get_predict_rsoc(void)
{
        struct ug31xx_data *ug31xx;

        ug31xx = (struct ug31xx_data *)lkm_gauge;

        return ((int)(ug31xx->capData.predictRsoc));
}

/**
 * @brief lkm_saveDataToIC_switch
 *
 *  Enable/Disable the function of save table to IC
 *
 * @return  0 if success
 */
int lkm_saveDataToIC_switch(_upi_bool_ enable)
{
        Ug31SaveDataEnable = enable;
        return (0);
}

/**
 * @brief lkm_change_to_pri_batt
 *
 *  Change current cell table to primary battery
 *
 * @return  0 if success
 */
int lkm_change_to_pri_batt(char *ggb, char pri_batt)
{
        struct ug31xx_data *ug31xx;
        SYSTEM_RTN_CODE rtn;

        ug31xx = (struct ug31xx_data *)lkm_gauge;
        ug31xx->sysData.ggbXBuf = (GGBX_FILE_HEADER *)ggb;
        ug31xx->sysData.ggbParameter = &ug31xx->cellParameter;
        ug31xx->sysData.ggbCellTable = &ug31xx->cellTable;
        rtn = UpiInitSystemData(&ug31xx->sysData);
        if(rtn != SYSTEM_RTN_PASS) {
                if(rtn == SYSTEM_RTN_READ_GGB_FAIL) {
                        return (UG_READ_GGB_FAIL);
                }
                return (UG_NOT_DEF);
        }
        /// Reset coulomb counter
        ug31xx->measData.sysData = &ug31xx->sysData;
        ug31xx->measData.otp = &ug31xx->otpData;
        UpiResetCoulombCounter(&ug31xx->measData);
        ug31xx->sysData.cycleCount = (_sys_u16_)ug31xx->measData.cycleCount;
        /// Inital capacity
        UpiInitCapacity(&ug31xx->capData);
        ug31xx->batteryInfo.NAC = (_upi_u16_)ug31xx->capData.rm;
        ug31xx->batteryInfo.LMD = (_upi_u16_)ug31xx->capData.fcc;
        ug31xx->batteryInfo.RSOC = (_upi_u16_)ug31xx->capData.rsoc;
        UG31_LOGN("[%s]: Init data from table -> %d/%d = %d\n", __func__,
                  ug31xx->batteryInfo.NAC, ug31xx->batteryInfo.LMD, ug31xx->batteryInfo.RSOC);
        UpiInitDsgCharge(&ug31xx->capData);
        UpiAdjustCCRecord(&ug31xx->capData);

        if(pri_batt == _UPI_TRUE_) {
                ug31xx->sysData.rmFromIC = ug31xx->batteryInfo.NAC;
                ug31xx->sysData.fccFromIC = ug31xx->batteryInfo.LMD;
                ug31xx->sysData.rsocFromIC = (_sys_u8_)ug31xx->batteryInfo.RSOC;
                ug31xx->sysData.tableUpdateIdxFromIC = SOV_NUMS;
                ug31xx->sysData.deltaCapFromIC = 0;
                ug31xx->sysData.adc1ConvTime = ug31xx->measData.adc1ConvertTime;
                ug31xx->sysData.ccOffset = (_sys_s8_)ug31xx->measData.ccOffsetAdj;
                ug31xx->sysData.standbyDsgRatio = (_sys_u8_)ug31xx->capData.standbyDsgRatio;
                UpiSaveBatInfoTOIC(&ug31xx->sysData);
                /// Load table
                UpiLoadTableFromIC((_sys_u8_ *)ug31xx->capData.encriptTable);
                ug31xx->capData.ggbTable = &ug31xx->cellTable;
                ug31xx->capData.ggbParameter = &ug31xx->cellParameter;
                ug31xx->capData.measurement = &ug31xx->measData;
                UpiInitNacTable(&ug31xx->capData);
        }
        return (0);
}

/**
 * @brief lkm_set_backup_file_name
 *
 *  Set backup filename
 *
 * @para  filename  filename of backup file
 * @para  length  length of filename
 * @return  0 if success
 */
int lkm_set_backup_file_name(char *filename, int length)
{
        if(lkm_backup_filename != _UPI_NULL_) {
                upi_free(lkm_backup_filename);
        }
        lkm_backup_filename = (char *)upi_malloc((_upi_u32_)length);
        upi_memcpy(lkm_backup_filename, filename, (_upi_u32_)length);
        return (0);
}

/**
 * @brief lkm_set_suspend_file_name
 *
 *  Set suspend data backup filename
 *
 * @para  filename  filename of suspend backup file
 * @para  length  length of filename
 * @return  0 if success
 */
int lkm_set_suspend_file_name(char *filename, int length)
{
        if(lkm_suspend_filename != _UPI_NULL_) {
                upi_free(lkm_suspend_filename);
        }
        lkm_suspend_filename = (char *)upi_malloc((_upi_u32_)length);
        upi_memcpy(lkm_suspend_filename, filename, (_upi_u32_)length);
        return (0);
}

/**
 * @brief lkm_set_options
 *
 *  Set kernel module operation options
 *
 * @para  options options from insmod
 * @return  0 if success
 */
int lkm_set_options(unsigned char options)
{
        lkm_options = options;
        return (0);
}

/**
 * @brief lkm_get_gpio
 *
 *  Get GPIO status
 *
 * @para  gpio  uG31xx gpio index
 * @return  ug31xx_gpio_status_t
 */
int lkm_get_gpio(ug31xx_gpio_idx_t gpio)
{
        _upi_bool_ rtn;
        _upi_u16_ addr;
        _upi_u8_ mask;
        _upi_u8_ value;

        switch(gpio) {
        case  UG31XX_GPIO_1:
                addr = REG_CTRL1;
                mask = CTRL1_IO1DATA;
                break;
        case  UG31XX_GPIO_2:
                addr = REG_CTRL2;
                mask = CTRL2_IO2DATA;
                break;
        case  UG31XX_GPIO_3:
                addr = REG_CTRL2;
                mask = CTRL2_IO3DATA;
                break;
        case  UG31XX_GPIO_4:
                addr = REG_CTRL2;
                mask = CTRL2_IO4DATA;
                break;
        default:
                return (UG31XX_GPIO_STS_UNKNOWN);
        }

        rtn = API_I2C_Read(_UPI_FALSE_, _UPI_FALSE_, _UPI_FALSE_, addr, 1, &value);
        if(rtn != _UPI_TRUE_) {
                return (UG31XX_GPIO_STS_UNKNOWN);
        }

        return ((value & mask) ? UG31XX_GPIO_STS_HIGH : UG31XX_GPIO_STS_LOW);
}

/**
 * @brief lkm_set_gpio
 *
 *  Set GPIO status
 *
 * @para  gpio  uG31xx gpio index
 * @para  status  uG31xx gpio status
 * @return  0 if success
 */
int lkm_set_gpio(ug31xx_gpio_idx_t gpio, int status)
{
        _upi_bool_ rtn;
        _upi_u16_ addr;
        _upi_u8_ mask;
        _upi_u8_ value;

        switch (gpio) {
        case  UG31XX_GPIO_1:
                addr = REG_CTRL1;
                mask = CTRL1_IO1DATA;
                break;
        case  UG31XX_GPIO_2:
                addr = REG_CTRL2;
                mask = CTRL2_IO2DATA;
                break;
        case  UG31XX_GPIO_3:
                addr = REG_CTRL2;
                mask = CTRL2_IO3DATA;
                break;
        case  UG31XX_GPIO_4:
                addr = REG_CTRL2;
                mask = CTRL2_IO4DATA;
                break;
        default:
                return (-1);
        }

        rtn = API_I2C_Read(_UPI_FALSE_, _UPI_FALSE_, _UPI_FALSE_, addr, 1, &value);
        if(rtn != _UPI_TRUE_) {
                return (-1);
        }

        if(status == UG31XX_GPIO_STS_LOW) {
                value = value & (~mask);
        } else if(status == UG31XX_GPIO_STS_HIGH) {
                value = value | mask;
        } else {
                return (-1);
        }

        rtn = API_I2C_Write(_UPI_FALSE_, _UPI_FALSE_, _UPI_FALSE_, addr, 1, &value);
        return ((rtn == _UPI_TRUE_) ? 0 : -1);
}

/**
 * @brief lkm_get_cycle_count
 *
 *  Get cycle count information
 *
 * @return  cycle count
 */
int lkm_get_cycle_count(void)
{
        int cycleCount;

        cycleCount = upiGG_GetCycleCount(lkm_gauge);
        return (cycleCount);
}

/**
 * @brief lkm_reset_cycle_count
 *
 *  Reset cycle count information
 *
 * @return  0 if success
 */
int lkm_reset_cycle_count(void)
{
        int rtn;

        rtn = upiGG_SetCycleCount(lkm_gauge, 0);
        return (rtn);
}

int lkm_i2c_read(unsigned short addr, unsigned char *data)
{
        int rtn;
        rtn = API_I2C_Read((addr < 0x80)? NORMAL: SECURITY,
                           UG31XX_I2C_HIGH_SPEED_MODE,
                           UG31XX_I2C_TEM_BITS_MODE,
                           addr,
                           1,
                           data);
        return ((rtn == _UPI_TRUE_) ? 0 : -1);
}

int lkm_i2c_write(unsigned short addr, unsigned char *data)
{
        int rtn;
        rtn = API_I2C_Write((addr < 0x80)? NORMAL: SECURITY,
                            UG31XX_I2C_HIGH_SPEED_MODE,
                            UG31XX_I2C_TEM_BITS_MODE,
                            addr,
                            1,
                            data);
        return ((rtn == _UPI_TRUE_) ? 0 : -1);
}

/**
 * @brief lkm_adjust_cell_table
 *
 *  Adjust current cell table according to design capacity
 *
 * @para  design_capacity target design capacity
 * @return  0 if success
 */
int lkm_adjust_cell_table(unsigned short design_capacity)
{
        upiGG_AdjustCellTable(lkm_gauge, design_capacity);
        return (0);
}

/**
 * @brief lkm_get_avg_external_temperature
 *
 *  Get average instant external temperature
 *
 * @return  average instant external temperature
 */
int lkm_get_avg_external_temperature(void)
{
        _upi_u8_ idx;
        _upi_s32_ avgTemp;

        /// [AT-PM] : Calculate average external temperature from buffer ; 11/27/2013
        avgTemp = 0;
        idx = 0;
        while(idx < LKM_AVG_TEMPERATURE_COUNT) {
                avgTemp = avgTemp + lkm_avg_ext_temperature_buf[idx];
                idx = idx + 1;
        }
        avgTemp = avgTemp/LKM_AVG_TEMPERATURE_COUNT;
        return ((int)avgTemp);
}

/**
 * @brief lkm_get_ntc_status
 *
 *  Get NTC status
 *
 * @return  0 if normal
 */
int lkm_get_ntc_status(void)
{
        if(upiGG_GetNtcStatus(lkm_gauge) == UG_MEAS_FAIL_NTC_SHORT) {
                return (UPI_UG31XX_NTC_SHORT);
        }

        if(upiGG_GetNtcStatus(lkm_gauge) == UG_MEAS_FAIL_NTC_OPEN) {
                return (UPI_UG31XX_NTC_OPEN);
        }

        return (UPI_UG31XX_NTC_NORMAL);
}

/**
 * @brief lkm_get_backup_buffer
 *
 *  Get backup buffer address and size
 *
 * @para  size  address of backup buffer size
 * @return  address of backup buffer
 */
unsigned char * lkm_get_backup_buffer(int *size)
{
        unsigned char *bufAddr;
        unsigned char bufSize;

        bufAddr = upiGG_AccessBackupBuffer(lkm_gauge, (_upi_u8_ *)&bufSize);
        *size = (int)bufSize;
        return (bufAddr);
}

/**
 * @brief lkm_set_shell_ap
 *
 *  Set shell AP name
 *
 * @para  apname  AP name
 * @para  length  length of filename
 * @return  0 if success
 */
int lkm_set_shell_ap(char *apname, int length)
{
        if(lkm_shell_ap_name != _UPI_NULL_) {
                upi_free(lkm_shell_ap_name);
        }
        lkm_shell_ap_name = (char *)upi_malloc((_upi_u32_)length);
        upi_memcpy(lkm_shell_ap_name, apname, (_upi_u32_)length);
        set_shell_ap_name(lkm_shell_ap_name);
        return (0);
}

/**
 * @brief lkm_get_backup_daemon_cntl
 *
 *  Get control for backup daemon
 *
 * @return  control
 */
unsigned char lkm_get_backup_daemon_cntl(void)
{
        unsigned char cntl;

        cntl = (unsigned char)get_file_op_status();
        return (cntl);
}

/**
 * @brief lkm_set_backup_daemon_cntl
 *
 *  Set control for backup daemon
 *
 * @para  cntl  control to be set
 * @return  0 if success
 */
int lkm_set_backup_daemon_cntl(unsigned char cntl)
{
        cntl = cntl & (~(UG31XX_KERNEL_FILE_EXIST | UG31XX_KERNEL_FILE_READ | UG31XX_KERNEL_FILE_WRITE));

        clear_file_op_status_bit(UG31XX_KERNEL_FILE_FINISH);
        clear_file_op_status_bit(UG31XX_USER_FILE_EXIST);
        clear_file_op_status_bit(UG31XX_USER_FILE_READ);
        clear_file_op_status_bit(UG31XX_USER_FILE_WRITE);

        set_file_op_status_bit(cntl);
        return (0);
}

/**
 * @brief lkm_get_backup_daemon_period
 *
 *  Get period of backup daemon
 *
 * @return  period time in second
 */
unsigned char lkm_get_backup_daemon_period(void)
{
        unsigned char period;
        _upi_s32_ tmp32;

        period = (unsigned char)polling_time_algorithm();

        tmp32 = LKM_CHECK_BACKUP_FILE_INTERVAL;
        tmp32 = tmp32 - lkm_chk_backup_file_interval;
        if(tmp32 > 1) {
                tmp32 = tmp32*period;
                if(tmp32 > 255) {
                        tmp32 = 255;
                }
                period = (unsigned char)tmp32;
        }
        return (period);
}

/**
 * @brief lkm_get_current_now
 *
 *  Get current now
 *
 * @return  current in mA
 */
int lkm_get_current_now(void)
{
        struct ug31xx_data *ug31xx;

        ug31xx = (struct ug31xx_data *)lkm_gauge;

        upiGG_FetchCurrent(lkm_gauge);
        return (ug31xx->measData.curr);
}

/**
 * @brief lkm_get_voltage_now
 *
 *  Get voltage now
 *
 * @return  voltage in mA
 */
int lkm_get_voltage_now(void)
{
        struct ug31xx_data *ug31xx;

        ug31xx = (struct ug31xx_data *)lkm_gauge;

        upiGG_FetchVoltage(lkm_gauge);
        return (ug31xx->measData.bat1Voltage);
}

/**
 * @brief lkm_get_external_temperature_now
 *
 *  Get external temperature now
 *
 * @return  voltage in mA
 */
int lkm_get_external_temperature_now(void)
{
        struct ug31xx_data *ug31xx;

        ug31xx = (struct ug31xx_data *)lkm_gauge;

        upiGG_FetchExternalTemperature(lkm_gauge);
        lkm_update_avg_external_temperature((_upi_s16_)ug31xx->measData.instExtTemperature);
        return(ug31xx->measData.extTemperature);
}

/**
 * @brief lkm_get_internal_temperature_now
 *
 *  Get internal temperature now
 *
 * @return  voltage in mA
 */
int lkm_get_internal_temperature_now(void)
{
        struct ug31xx_data *ug31xx;

        ug31xx = (struct ug31xx_data *)lkm_gauge;

        upiGG_FetchInternalTemperature(lkm_gauge);
        return (ug31xx->measData.intTemperature);
}

/**
 * @brief lkm_get_update_interval
 *
 *  Get update time interval
 *
 * @return  time interval in second
 */
int lkm_get_update_interval(void)
{
        _upi_u32_ curr_time_tag;

        curr_time_tag = GetSysTickCount();
        curr_time_tag = curr_time_tag - lkm_last_update_time_tag;
        curr_time_tag = curr_time_tag/1000;
        UG31_LOGN("[%s]: LKM update interval = %d seconds\n", __func__, curr_time_tag);

        if(!(curr_time_tag % UPI_POLLING_TIME_CONST)) {
                UG31_LOGI("[%s]: LKM update interval = %d minutes\n", __func__, curr_time_tag/UPI_POLLING_TIME_CONST);
        }
        return (curr_time_tag);
}

/**
 * @brief lkm_get_update_time
 *
 *  Get battery information update time
 *
 * @return  time interval in second
 */
int lkm_get_update_time(void)
{
        int update_time;

        update_time = (int)polling_time_algorithm();
        return (update_time);
}

#define CALIBRATE_CURRENT_UPPER_BOUND     (20)
#define CALIBRATE_CURRENT_LOWER_BOUND     (-20)
#define CALIBRATE_CURRENT_NO_UPPER_BOUND  (10000)

/**
 * @brief lkm_calibrate_offset
 *
 *  Calibrate board current offset
 *
 * @para  options UG31XX_BOARD_OFFSET_CALI_STEP or UG31XX_BOARD_OFFSET_CALI_FULL
 * @return  0 if success
 */
int lkm_calibrate_offset(unsigned char options)
{
        _upi_s16_ upper;
        _upi_s16_ lower;
        _upi_s8_ fullStep;

        upiGG_FetchCurrent(lkm_gauge);

        upper = CALIBRATE_CURRENT_UPPER_BOUND;
        lower = CALIBRATE_CURRENT_LOWER_BOUND;

        if(options == UG31XX_BOARD_OFFSET_CALI_FULL_NO_UPPER) {
                upper = CALIBRATE_CURRENT_NO_UPPER_BOUND;
                options = UG31XX_BOARD_OFFSET_CALI_FULL;
        }

        if(options == UG31XX_BOARD_OFFSET_CALI_FULL) {
                fullStep = GET_BOARD_OFFSET_FULL;
        } else if(options == UG31XX_BOARD_OFFSET_CALI_AVG) {
                fullStep = GET_BOARD_OFFSET_AVG;
        } else {
                fullStep = GET_BOARD_OFFSET_STEP;
        }
        upiGG_GetBoardOffset(lkm_gauge, fullStep, upper, lower);
        return (0);
}

/**
 * @brief lkm_get_board_offset
 *
 *  Get current board offset
 *
 * @return  effective board offset
 */
int lkm_get_board_offset(void)
{
        struct ug31xx_data *ug31xx;
        int offset;

        ug31xx = (struct ug31xx_data *)lkm_gauge;

        offset = (int)ug31xx->cellParameter.adc1_pos_offset;
        offset = offset + ug31xx->measData.ccOffsetAdj;
        return (offset);
}

/**
 * @brief lkm_get_ggb_board_offset
 *
 *  Get GGB board offset
 *
 * @return  GGB board offset
 */
int lkm_get_ggb_board_offset(void)
{
        struct ug31xx_data *ug31xx;

        ug31xx = (struct ug31xx_data *)lkm_gauge;
        return ((int)ug31xx->cellParameter.adc1_pos_offset);
}

/**
 * @brief lkm_set_ggb_board_offset
 *
 *  Set GGB board offset
 *
 * @para  offset  target GGB board offset
 * @return  0 if success
 */
int lkm_set_ggb_board_offset(int offset)
{
        struct ug31xx_data *ug31xx;

        ug31xx = (struct ug31xx_data *)lkm_gauge;
        ug31xx->cellParameter.adc1_pos_offset = (_upi_s16_)offset;
        UG31_LOGE("[%s]: adc1_pos_offset = %d\n", __func__,
                  offset);
        return (0);
}

#define LKM_MAX_CC_OFFSET_ADJ     (100)
#define LKM_MIN_CC_OFFSET_ADJ     (-100)

/**
 * @brief lkm_set_board_offset
 *
 *  Set effective board offset
 *
 * @para  offset  target effective offset
 * @para  from_upi_bo set UG31XX_BOARD_OFFSET_FROM_UPI_BO for board offset from upi_bo file
 * @return  0 if success
 */
int lkm_set_board_offset(int offset, char from_upi_bo)
{
        struct ug31xx_data *ug31xx;

        ug31xx = (struct ug31xx_data *)lkm_gauge;

        offset = offset - ug31xx->cellParameter.adc1_pos_offset;
        if(offset > LKM_MAX_CC_OFFSET_ADJ) {
                offset = LKM_MAX_CC_OFFSET_ADJ;
        }
        if(offset < LKM_MIN_CC_OFFSET_ADJ) {
                offset = LKM_MIN_CC_OFFSET_ADJ;
        }
        ug31xx->measData.ccOffsetAdj = (_meas_s8_)offset;
        UG31_LOGI("[%s]: Set board offset to %d + %d\n", __func__,
                  ug31xx->cellParameter.adc1_pos_offset,
                  ug31xx->measData.ccOffsetAdj);

        if(from_upi_bo == UG31XX_BOARD_OFFSET_FROM_UPI_BO) {
                UpiSetFactoryBoardOffset(&ug31xx->capData);
        }
        return (0);
}

/**
 * @brief lkm_set_ntc_offset
 *
 *  Set NTC offset to GGB file
 *
 * @para  offset  target NTC offset
 * @return  0 if success
 */
int lkm_set_ntc_offset(int offset)
{
        struct ug31xx_data *ug31xx;

        ug31xx = (struct ug31xx_data *)lkm_gauge;
        ug31xx->cellParameter.offsetR = (_upi_u16_)offset;
        UG31_LOGE("[%s]: offsetR = %d\n", __func__,
                  offset);
        return (0);
}

/**
 * @brief lkm_get_ntc_offset
 *
 *  Get NTC offset from GGB file
 *
 * @return  NTC offset in mOhm
 */
int lkm_get_ntc_offset(void)
{
        struct ug31xx_data *ug31xx;

        ug31xx = (struct ug31xx_data *)lkm_gauge;
        return ((int)ug31xx->cellParameter.offsetR);
}

/**
 * @brief lkm_set_standby_current
 *
 *  Set standby current for suspend
 *
 * @para  curr  standby current in mA
 * @return  0 if success
 */
int lkm_set_standby_current(int curr)
{
        struct ug31xx_data *ug31xx;

        ug31xx = (struct ug31xx_data *)lkm_gauge;
        ug31xx->cellParameter.deltaR = (_upi_u16_)curr;
        UG31_LOGE("[%s]: deltaR = %d\n", __func__,
                  curr);
        return (0);
}

/**
 * @brief lkm_get_standby_current
 *
 *  Get standby current in suspend
 *
 * @return  standby current in mA
 */
int lkm_get_standby_current(void)
{
        struct ug31xx_data *ug31xx;

        ug31xx = (struct ug31xx_data *)lkm_gauge;
        return ((int)ug31xx->cellParameter.deltaR);
}

/**
 * @brief lkm_get_ggb_board_gain
 *
 *  Get board gain in GGB
 *
 * @return  board gain in GGB
 */
int lkm_get_ggb_board_gain(void)
{
        struct ug31xx_data *ug31xx;

        ug31xx = (struct ug31xx_data *)lkm_gauge;
        return ((int)ug31xx->cellParameter.adc1_ngain);
}

/**
 * @brief lkm_set_ggb_board_gain
 *
 *  Set board gain in GGB
 *
 * @para  gain  board gain
 * @return   0 if success
 */
int lkm_set_ggb_board_gain(int gain)
{
        struct ug31xx_data *ug31xx;

        ug31xx = (struct ug31xx_data *)lkm_gauge;
        ug31xx->cellParameter.adc1_ngain = (_upi_s16_)gain;
        UG31_LOGE("[%s]: adc1_ngain = %d\n", __func__,
                  gain);
        return (0);
}

/**
 * @brief lkm_get_cumulative_capacity
 *
 *  Get cumulative capacity from coulomb counter
 *
 * @return  cumulative capacity in mAh
 */
int lkm_get_cumulative_capacity(void)
{
        struct ug31xx_data *ug31xx;

        ug31xx = (struct ug31xx_data *)lkm_gauge;
        return ((int)ug31xx->measData.cumuCap);
}

/**
 * @brief lkm_get_delta_q
 *
 *  Get delta capacity from coulomb counter
 *
 * @return  delta capacity in mAh
 */
int lkm_get_delta_q(void)
{
        struct ug31xx_data *ug31xx;

        ug31xx = (struct ug31xx_data *)lkm_gauge;
        return ((int)ug31xx->measData.stepCap);
}

/**
 * @brief lkm_set_capacity_suspend_mode
 *
 *  Set capacity module in suspend mode
 *
 * @para  in_suspend  set 0 to disable suspend mode
 * @return  0 if success
 */
int lkm_set_capacity_suspend_mode(char in_suspend)
{
        _upi_bool_ enable;

        enable = (in_suspend == 0) ? _UPI_FALSE_ : _UPI_TRUE_;
        upiGG_SetCapacitySuspendMode(lkm_gauge, enable);
        return (0);
}

/**
 * @brief lkm_shell_update
 *
 *  Interaction with shell algorithm after lkm_update_procedure
 *
 * @return  0 if success
 */
int lkm_shell_update(void)
{
#ifdef  UG31XX_SHELL_ALGORITHM

        GG_CAPACITY *devCapacity;
        struct ug31xx_data *ug31xx;

        ug31xx = (struct ug31xx_data *)lkm_gauge;

        devCapacity = (GG_CAPACITY *)upi_malloc(sizeof(GG_CAPACITY));
        if(devCapacity == _UPI_NULL_) {
                return (-1);
        }

        upiGG_ReadCapacity(lkm_gauge, devCapacity);
        UG31_LOGI("[%s]: %d / %d = %d\n", __func__, devCapacity->NAC, devCapacity->LMD, devCapacity->RSOC);
        upi_free(devCapacity);

#endif  ///< end of UG31XX_SHELL_ALGORITHM

        return (0);
}

/**
 * @brief lkm_set_cable_out
 *
 *  Set current cable status
 *
 * @para  cntl  UG31XX_CABLE_OUT or UG31XX_CABLE_IN
 * @return  0 if success
 */
int lkm_set_cable_out(unsigned char cntl)
{
        struct ug31xx_data *ug31xx;

        ug31xx = (struct ug31xx_data *)lkm_gauge;

        if(cntl == UG31XX_CABLE_OUT) {
                ug31xx->measData.status = ug31xx->measData.status | MEAS_STATUS_CABLE_OUT;
        } else {
                ug31xx->measData.status = ug31xx->measData.status & (~MEAS_STATUS_CABLE_OUT);
        }
        return (0);
}

/**
 * @brief lkm_shell_backup
 *
 *  Backup file operation
 *
 * @return  backup file status
 */
int lkm_shell_backup(void)
{
        struct ug31xx_data *ug31xx;

        ug31xx = (struct ug31xx_data *)lkm_gauge;

        if(ug31xx->batteryInfo.Ready == UG_CAP_DATA_VERSION_MISMATCH) {
                return (1);
        }

        if(ug31xx->batteryInfo.Ready == UG_CAP_DATA_NOT_READY) {
                return (-1);
        }
        return (0);
}

/**
 * @brief lkm_shell_memory
 *
 *  Get memory buffer address and size
 *
 * @para  mem_size  address of memory size
 * @return  memory buffer address
 */
unsigned char * lkm_shell_memory(int *mem_size)
{
        *mem_size = (int)sizeof(struct ug31xx_data);
        UG31_LOGN("[%s]: memory[0] = %02x (%x-%d)\n", __func__, lkm_gauge[0], lkm_gauge, (*mem_size));
        return (lkm_gauge);
}

/**
 * @brief lkm_shell_backup_memory
 *
 *  Get backup memory buffer and size
 *
 * @para  mem_size  address of memory size
 * @return  memory buffer address
 */
unsigned char * lkm_shell_backup_memory(int *mem_size)
{
        struct ug31xx_data *ug31xx;

        ug31xx = (struct ug31xx_data *)lkm_gauge;

        *mem_size = ug31xx->backupData.backupBufferSize;
        UG31_LOGN("[%s]: memory[0] = %02x (%x-%d)\n", __func__, ug31xx->backupData.backupBuffer[0], ug31xx->backupData.backupBuffer, (*mem_size));
        return (ug31xx->backupData.backupBuffer);
}

/**
 * @brief lkm_shell_table_memory
 *
 *  Get capacity table memory buffer and size
 *
 * @para  mem_size  address of memory size
 * @return  memory buffer address
 */
unsigned char * lkm_shell_table_memory(int *mem_size)
{
        struct ug31xx_data *ug31xx;

        ug31xx = (struct ug31xx_data *)lkm_gauge;

        *mem_size = ug31xx->capData.tableSize;
        UG31_LOGN("[%s]: memory[0] = %02x (%x-%d)\n", __func__, ug31xx->capData.encriptBuf[0], ug31xx->capData.encriptBuf, (*mem_size));
        return (ug31xx->capData.encriptBuf);
}

/**
 * @brief lkm_shell_table_buf_memory
 *
 *  Get capacity table buffer memory buffer and size
 *
 * @para  mem_size  address of memory size
 * @return  memory buffer address
 */
unsigned char * lkm_shell_table_buf_memory(int *mem_size)
{
        struct ug31xx_data *ug31xx;

        ug31xx = (struct ug31xx_data*)lkm_gauge;

        *mem_size = ug31xx->capData.tableSize;
        UG31_LOGN("[%s]: memory[0] = %02x (%x-%d)\n", __func__, ug31xx->capData.encriptBuf[0], ug31xx->capData.encriptBuf, (*mem_size));
        return (ug31xx->capData.encriptBuf);
}

static SystemDataType *meas_sysData;
static OtpDataType *meas_otp;
static CELL_PARAMETER *cap_ggbParameter;
static CELL_TABLE *cap_ggbTable;
static MeasDataType *cap_measurement;
static GGBX_FILE_HEADER *sys_ggbXBuf;
static CELL_PARAMETER *sys_ggbParameter;
static CELL_TABLE *sys_ggbCellTable;
static OtpDataType *sys_otpData;
static CapacityDataType *backup_capData;
static SystemDataType *backup_sysData;
static MeasDataType *backup_measData;

/**
 * @brief lkm_backup_pointer
 *
 *  Backup pointer in memory buffer
 *
 * @return  NULL
 */
int lkm_backup_pointer(void)
{
        struct ug31xx_data *ug31xx;

        ug31xx = (struct ug31xx_data *)lkm_gauge;

        meas_sysData = ug31xx->measData.sysData;
        meas_otp = ug31xx->measData.otp;
        cap_ggbParameter = ug31xx->capData.ggbParameter;
        cap_ggbTable = ug31xx->capData.ggbTable;
        cap_measurement = ug31xx->capData.measurement;
        sys_ggbXBuf = ug31xx->sysData.ggbXBuf;
        sys_ggbParameter = ug31xx->sysData.ggbParameter;
        sys_ggbCellTable = ug31xx->sysData.ggbCellTable;
        sys_otpData = ug31xx->sysData.otpData;
        backup_sysData = ug31xx->backupData.sysData;
        backup_measData = ug31xx->backupData.measData;
        UG31_LOGN("[%s]: cap_ggbParameter, cap_ggbTable, cap_measurement = %d, %d, %d\n", __func__,
                  cap_ggbParameter, cap_ggbTable, cap_measurement);
}

/**
 * @brief lkm_restore_pointer
 *
 *  Restore pointer in memory buffer
 *
 * @return  NULL
 */
int lkm_restore_pointer(void)
{
        struct ug31xx_data *ug31xx;

        ug31xx = (struct ug31xx_data *)lkm_gauge;

        ug31xx->measData.sysData = meas_sysData;
        ug31xx->measData.otp = meas_otp;
        ug31xx->capData.ggbParameter = cap_ggbParameter;
        ug31xx->capData.ggbTable = cap_ggbTable;
        ug31xx->capData.measurement = cap_measurement;
        ug31xx->sysData.ggbXBuf = sys_ggbXBuf;
        ug31xx->sysData.ggbParameter = sys_ggbParameter;
        ug31xx->sysData.ggbCellTable = sys_ggbCellTable;
        ug31xx->sysData.otpData = sys_otpData;
        ug31xx->backupData.sysData = backup_sysData;
        ug31xx->backupData.measData = backup_measData;
        UG31_LOGN("[%s]: cap_ggbParameter, cap_ggbTable, cap_measurement = %d, %d, %d\n", __func__,
                  ug31xx->capData.ggbParameter, ug31xx->capData.ggbTable, ug31xx->capData.measurement);
}

struct ug31xx_module_interface ug31_module = {
        .initial    = lkm_initial,
        .uninitial  = lkm_uninitial,
        .suspend    = lkm_suspend,
        .resume     = lkm_resume,
        .shutdown   = lkm_shutdown,
        .update     = lkm_update,
        .reset      = lkm_reset,

        .shell_update           = lkm_shell_update,
        .shell_memory           = lkm_shell_memory,
        .shell_backup           = lkm_shell_backup,
        .shell_backup_memory    = lkm_shell_backup_memory,
        .shell_table_memory     = lkm_shell_table_memory,
        .shell_table_buf_memory = lkm_shell_table_buf_memory,

        .get_voltage                    = lkm_get_voltage,
        .get_voltage_now                = lkm_get_voltage_now,
        .get_current                    = lkm_get_current,
        .get_current_now                = lkm_get_current_now,
        .get_external_temperature       = lkm_get_external_temperature,
        .get_external_temperature_now   = lkm_get_external_temperature_now,
        .get_internal_temperature       = lkm_get_internal_temperature,
        .get_internal_temperature_now   = lkm_get_internal_temperature_now,
        .get_remaining_capacity         = lkm_get_remaining_capacity,
        .get_full_charge_capacity       = lkm_get_full_charge_capacity,
        .get_relative_state_of_charge   = lkm_get_relative_state_of_charge,
        .get_version                    = lkm_get_version,
        .get_polling_time               = lkm_get_polling_time,
        .get_module_ready               = lkm_get_module_ready,
        .get_battery_removed            = lkm_get_battery_removed,
        .get_alarm_status               = lkm_get_alarm_status,
        .get_charge_termination_current = lkm_get_taper_current,
        .get_full_charge_status         = lkm_get_full_charge_status,
        .get_design_capacity            = lkm_get_design_capacity,
        .get_rsense                     = lkm_get_rsense,
        .get_predict_rsoc               = lkm_get_predict_rsoc,
        .get_gpio                       = lkm_get_gpio,
        .get_cycle_count                = lkm_get_cycle_count,
        .get_avg_external_temperature   = lkm_get_avg_external_temperature,
        .get_ntc_status                 = lkm_get_ntc_status,
        .get_backup_buffer              = lkm_get_backup_buffer,
        .get_backup_daemon_cntl         = lkm_get_backup_daemon_cntl,
        .get_backup_daemon_period       = lkm_get_backup_daemon_period,
        .get_update_interval            = lkm_get_update_interval,
        .get_update_time                = lkm_get_update_time,
        .get_board_offset               = lkm_get_board_offset,
        .get_delta_q                    = lkm_get_delta_q,
        .get_ggb_board_offset           = lkm_get_ggb_board_offset,
        .get_ntc_offset                 = lkm_get_ntc_offset,
        .get_cumulative_capacity        = lkm_get_cumulative_capacity,
        .get_standby_current            = lkm_get_standby_current,
        .get_ggb_board_gain             = lkm_get_ggb_board_gain,

        .set_backup_file                = lkm_set_backup_file,
        .set_charger_full               = lkm_set_charger_full,
        .set_charge_termination_current = lkm_set_taper_current,
        .set_battery_temp_external      = lkm_set_battery_temp_external,
        .set_battery_temp_internal      = lkm_set_battery_temp_internal,
        .set_rsense                     = lkm_set_rsense,
        .set_backup_file_name           = lkm_set_backup_file_name,
        .set_suspend_file_name          = lkm_set_suspend_file_name,
        .set_options                    = lkm_set_options,
        .set_gpio                       = lkm_set_gpio,
        .set_shell_ap                   = lkm_set_shell_ap,
        .set_backup_daemon_cntl         = lkm_set_backup_daemon_cntl,
        .set_capacity_suspend_mode      = lkm_set_capacity_suspend_mode,
        .set_cable_out                  = lkm_set_cable_out,
        .set_ggb_board_offset           = lkm_set_ggb_board_offset,
        .set_board_offset               = lkm_set_board_offset,
        .set_ntc_offset                 = lkm_set_ntc_offset,
        .set_standby_current            = lkm_set_standby_current,
        .set_ggb_board_gain             = lkm_set_ggb_board_gain,

        .chk_backup_file                = lkm_chk_backup_file,
        .enable_save_data               = lkm_saveDataToIC_switch,
        .change_to_pri_batt             = lkm_change_to_pri_batt,
        .ug31xx_i2c_read                = lkm_i2c_read,
        .ug31xx_i2c_write               = lkm_i2c_write,
        .reset_cycle_count              = lkm_reset_cycle_count,
        .adjust_cell_table              = lkm_adjust_cell_table,
        .calibrate_offset               = lkm_calibrate_offset,
        .backup_pointer                 = lkm_backup_pointer,
        .restore_pointer                = lkm_restore_pointer,
};

#endif  ///< end of ANDROID_SHELL_ALGORITHM

#endif  ///< end of uG31xx_BOOT_LOADER

#endif  ///< end of uG31xx_OS_WINDOWS

/// ===========================================
/// End of uG31xx_API.cpp
/// ===========================================


/**
 * Copyright @ 2013 uPI Semiconductor Corp. All right reserved.
 * The information, images, and/or data contained in this material is copyrighted by uPI
 * Semiconductor Corp., and may not be distributed, modified, reproduced in whole or in part
 * without the prior, written consent of uPI Semiconductor Corp.
 */
