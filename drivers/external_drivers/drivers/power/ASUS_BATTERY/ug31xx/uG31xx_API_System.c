/**
 * Copyright @ 2013 uPI Semiconductor Corp. All right reserved.
 * The information, images, and/or data contained in this material is copyrighted by uPI
 * Semiconductor Corp., and may not be distributed, modified, reproduced in whole or in part
 * without the prior, written consent of uPI Semiconductor Corp.
 */

/**
 * @filename  uG31xx_API_System.cpp
 *
 *  uG31xx system control
 *
 * @author  AllenTeng <allen_teng@upi-semi.com>
 * @revision  $Revision: 461 $
 */

#include "stdafx.h"     //windows need this??
#include "uG31xx_API.h"

#if defined(uG31xx_OS_WINDOWS)

#define SYSTEM_VERSION      (_T("System $Rev: 461 $"))

_upi_bool_ ReadGGBFileToCellDataAndInitSetting(SystemDataType *obj)
{
        FILE* stream;

        UG31_LOGI("[%s]: %s\n", __func__, SYSTEM_VERSION);

        _wfopen_s(&stream, obj->ggbFilename, _T("rb, ccs=UTF-8"));

        upi_memset(obj->ggbCellTable, 0x00, sizeof(CELL_TABLE));
        upi_memset(obj->ggbParameter, 0x00, sizeof(CELL_PARAMETER));

        if(!stream) {
                return (_UPI_FALSE_);
        }
        if(fread(obj->ggbParameter, sizeof(char), sizeof(CELL_PARAMETER), stream) != sizeof(CELL_PARAMETER)) {
                fclose(stream);
                return (_UPI_FALSE_);
        }
        if(fread(obj->ggbCellTable, sizeof(char), sizeof(CELL_TABLE), stream) != sizeof(CELL_TABLE)) {
                fclose(stream);
                return (_UPI_FALSE_);
        }

        fclose(stream);

        return (_UPI_TRUE_);
}

#else   ///< else of defined(uG31xx_OS_WINDOWS)

#define SYSTEM_VERSION      ("System $Rev: 461 $")

_upi_bool_ ReadGGBXFileToCellDataAndInitSetting(SystemDataType *obj)
{
        _sys_u8_ *p_start = _UPI_NULL_;
        _sys_u8_ *p_end = _UPI_NULL_;
        _sys_u16_ sum16=0;
        _sys_u32_ i=0;

        UG31_LOGI("[%s]: %s\n", __func__, SYSTEM_VERSION);

        /*
           * check GGBX_FILE tag
           */
        if(obj->ggbXBuf->ggb_tag != GGBX_FILE_TAG) {
                UG31_LOGE("[%s] GGBX file tag not correct. tag: %08X\n", __func__, obj->ggbXBuf->ggb_tag);
                return (_UPI_FALSE_);
        }

        /*
           * check GGBX_FILE checksum
           */
        p_start = (_sys_u8_ *)obj->ggbXBuf + sizeof(GGBX_FILE_HEADER);
        p_end = p_start + obj->ggbXBuf->length - 1;
        for (; p_start <= p_end; p_start++) {
                sum16 += *p_start;
        }

        /* check done. prepare copy data */
        upi_memset(obj->ggbCellTable, 0x00, sizeof(CELL_TABLE));
        upi_memset(obj->ggbParameter, 0x00, sizeof(CELL_PARAMETER));

        p_start = (_sys_u8_ *)obj->ggbXBuf + sizeof(GGBX_FILE_HEADER);
        for (i=0; i<obj->ggbXBuf->num_ggb; i++) {
                /* TODO: boundary checking */
                /* TODO: select right ggb content by sku */
                upi_memcpy(obj->ggbParameter, p_start, sizeof(CELL_PARAMETER));
                upi_memcpy(obj->ggbCellTable, p_start + obj->ggbXBuf->parameter_size, sizeof(CELL_TABLE));
                p_start += (obj->ggbXBuf->parameter_size + obj->ggbXBuf->cell_table_size);
        }
        return (_UPI_TRUE_);
}

#endif  ///< end of defined(uG31xx_OS_WINDOWS)

/**
 * @brief GetCellNum
 *
 *  Get cell number from ggbParameter->ICType
 *
 * @para  data  address of SystemDataType
 * @return  _UPI_NULL_
 */
void GetCellNum(SystemDataType *data)
{
        switch(data->ggbParameter->ICType) {
        case  0:
        case  1:
                data->cellNum = 1;
                break;
        case  2:
        case  4:
                data->cellNum = 2;
                break;
        case  5:
                data->cellNum = 3;
                break;
        case  3:
        default:
                data->cellNum = 0;
                break;
        }
}

/**
 * @brief SetupAdcChopFunction
 *
 *  Setup ADC chop function
 *
 * @para  data  address of SystemDataType
 * @return  _UPI_NULL_
 */
void SetupAdcChopFunction(SystemDataType *data)
{
        API_I2C_Write(SECURITY,
                      UG31XX_I2C_HIGH_SPEED_MODE,
                      UG31XX_I2C_TEM_BITS_MODE,
                      REG_FW_CTRL,
                      1,
                      &data->ggbParameter->chopCtrl);
}

/**
 * @brief SetupAdc1Queue
 *
 *  Setup ADC1 conversion queue
 *
 * @para  data  address of SystemDataType
 * @return  _UPI_NULL_
 */
void SetupAdc1Queue(SystemDataType *data)
{
        _sys_u8_ adcQueue[4];

        adcQueue[0] = SET_A_CURRENT | SET_B_CURRENT | SET_C_CURRENT | SET_D_ET;
        adcQueue[1] = SET_E_ET | SET_F_ET | SET_G_IT | SET_H_IT;
        adcQueue[2] = SET_I_IT | SET_J_CURRENT | SET_K_CURRENT | SET_L_CURRENT;
        adcQueue[3] = SET_M_CURRENT | SET_N_CURRENT | SET_O_CURRENT | SET_P_CURRENT;

        API_I2C_Write(SECURITY,
                      UG31XX_I2C_HIGH_SPEED_MODE,
                      UG31XX_I2C_TEM_BITS_MODE,
                      REG_ADC_CTR_A,
                      4,
                      &adcQueue[0]);
}

/**
 * @brief SetupAdc2Queue
 *
 *  Set ADC2 conversion queue
 *
 * @para  data  address of SystemDataType
 * @return  _UPI_NULL_
 */
void SetupAdc2Quene(SystemDataType *data)
{
        _sys_u8_ adc2Queue[3];

        /// [AT-PM] : Set sell type ; 01/31/2013
        if(data->cellNum == 1) {
                adc2Queue[0] = SET_V1_VBAT1 | SET_V2_VBAT1 | SET_V3_VBAT1 | SET_V4_VBAT1;
                adc2Queue[1] = SET_V5_VBAT1 | SET_V6_VBAT1 | SET_V7_VBAT1 | SET_V8_VBAT1;
                adc2Queue[2] = SET_V9_VBAT1 | SET_V10_VBAT1 | SET_V11_VBAT1 | SET_V12_VBAT1;
        } else if(data->cellNum == 2) {
                adc2Queue[0] = SET_V1_VBAT1 | SET_V2_VBAT1 | SET_V3_VBAT2 | SET_V4_VBAT2;
                adc2Queue[1] = SET_V5_VBAT1 | SET_V6_VBAT1 | SET_V7_VBAT2 | SET_V8_VBAT2;
                adc2Queue[2] = SET_V9_VBAT1 | SET_V10_VBAT1 | SET_V11_VBAT2 | SET_V12_VBAT2;
        } else if(data->cellNum == 3) {
                adc2Queue[0] = SET_V1_VBAT1 | SET_V2_VBAT1 | SET_V3_VBAT2 | SET_V4_VBAT2;
                adc2Queue[1] = SET_V5_VBAT3 | SET_V6_VBAT3 | SET_V7_VBAT1 | SET_V8_VBAT1;
                adc2Queue[2] = SET_V9_VBAT2 | SET_V10_VBAT2 | SET_V11_VBAT3 | SET_V12_VBAT3;
        } else {
                /// [AT-PM] : 1-cell ; 01/31/2013
                adc2Queue[0] = SET_V1_VBAT1 | SET_V2_VBAT1 | SET_V3_VBAT1 | SET_V4_VBAT1;
                adc2Queue[1] = SET_V5_VBAT1 | SET_V6_VBAT1 | SET_V7_VBAT1 | SET_V8_VBAT1;
                adc2Queue[2] = SET_V9_VBAT1 | SET_V10_VBAT1 | SET_V11_VBAT1 | SET_V12_VBAT1;
        }
        API_I2C_Write(SECURITY, UG31XX_I2C_HIGH_SPEED_MODE, UG31XX_I2C_TEM_BITS_MODE, REG_ADC_V1, 3, &adc2Queue[0]);
}

/**
 * @brief EnableCbc
 *
 *  Enable CBC function
 *
 * @para  data  address of SystemDataType
 * @return  _UPI_NULL_
 */
void EnableCbc(SystemDataType *data)
{
        _sys_u8_ tmp8;

        tmp8 = 0;
        API_I2C_Read(SECURITY,
                     UG31XX_I2C_HIGH_SPEED_MODE,
                     UG31XX_I2C_TEM_BITS_MODE,
                     REG_INTR_CTRL_B,
                     1,
                     &tmp8);
        tmp8 = tmp8 & (~(INTR_CTRL_B_CBC_32_EN | INTR_CTRL_B_CBC_21_EN));
        tmp8 = tmp8 | (INTR_CTRL_B_ET_EN | INTR_CTRL_B_IT_EN | INTR_CTRL_B_RID_EN);
        tmp8 = tmp8 | (data->ggbParameter->cbcEnable << 4);
        API_I2C_Write(SECURITY,
                      UG31XX_I2C_HIGH_SPEED_MODE,
                      UG31XX_I2C_TEM_BITS_MODE,
                      REG_INTR_CTRL_B,
                      1,
                      &tmp8);
}

/**
 * @brief EnableICType
 *
 *  Enable IC type
 *
 * @para  data  address of SystemDataType
 * @return  _UPI_NULL_
 */
void EnableICType(SystemDataType *data)
{
        _sys_u8_ tmp8;

        tmp8 = 0;
        API_I2C_Read(SECURITY,
                     UG31XX_I2C_HIGH_SPEED_MODE,
                     UG31XX_I2C_TEM_BITS_MODE,
                     REG_CELL_EN,
                     1,
                     &tmp8);
        tmp8 = tmp8 & (~CELL_EN_APPLICATION);
        tmp8 = tmp8 | (data->ggbParameter->ICType << 2);
        API_I2C_Write(SECURITY,
                      UG31XX_I2C_HIGH_SPEED_MODE,
                      UG31XX_I2C_TEM_BITS_MODE,
                      REG_CELL_EN,
                      1,
                      &tmp8);
}

/**
 * @brief ConfigGpioFunction
 *
 *  Configure GPIO1/2 function
 *
 * @para  setting GPIO1/2 setting
 * @return  register value
 */
_sys_u8_ ConfigGpioFunction(_sys_u8_ setting)
{
        _sys_u8_ gpioSelData = 0;

        if(setting & FUN_GPIO) {
                gpioSelData = 0;
        }
        if(setting & FUN_ALARM) { //select Alarm function
                gpioSelData = 1;
        }
        if(setting & FUN_CBC_EN21) {	//cbc21 enable
                gpioSelData = 2;
        }
        if(setting & FUN_CBC_EN32) {	//cbc32 Enable
                gpioSelData = 3;
        }
        if(setting & FUN_PWM) { //PWM function, set PWM cycle
                gpioSelData = 4;
        }
        return (gpioSelData);
}

/**
 * @brief ConfigureGpio
 *
 *  Configure GPIO function
 *
 * @para  data  SystemDataType
 * @return  _UPI_NULL_
 */
void ConfigureGpio(SystemDataType *data)
{
        _sys_u8_ tmp8;

        tmp8 = 0;
        API_I2C_Read(SECURITY,
                     UG31XX_I2C_HIGH_SPEED_MODE,
                     UG31XX_I2C_TEM_BITS_MODE,
                     REG_INTR_CTRL_A,
                     1,
                     &tmp8);
        tmp8 = tmp8 | (ConfigGpioFunction(data->ggbParameter->gpio1) << 2);
        tmp8 = tmp8 | (ConfigGpioFunction(data->ggbParameter->gpio2) << 5);
        API_I2C_Write(SECURITY,
                      UG31XX_I2C_HIGH_SPEED_MODE,
                      UG31XX_I2C_TEM_BITS_MODE,
                      REG_INTR_CTRL_A,
                      1,
                      &tmp8);
        API_I2C_Write(SECURITY,
                      UG31XX_I2C_HIGH_SPEED_MODE,
                      UG31XX_I2C_TEM_BITS_MODE,
                      REG_INTR_CTRL_D,
                      1,
                      &data->ggbParameter->gpio34);
}

#define ADC_FAIL_CRITERIA     (10)

/**
 * @brief CheckAdcStatusFail
 *
 *  Check ADC status is fail or not
 *
 * @para  pUg31xx address of SystemDataType
 * @return  _UPI_TRUE_ if fail
 */
_upi_bool_ CheckAdcStatusFail(SystemDataType *data)
{
        API_I2C_Read(NORMAL,
                     UG31XX_I2C_HIGH_SPEED_MODE,
                     UG31XX_I2C_TEM_BITS_MODE,
                     REG_COUNTER_LOW,
                     REG_COUNTER_HIGH - REG_COUNTER_LOW + 1,
                     (unsigned char *)&data->adcCheckData.regCounter);

        API_I2C_Read(NORMAL,
                     UG31XX_I2C_HIGH_SPEED_MODE,
                     UG31XX_I2C_TEM_BITS_MODE,
                     REG_AVE_VBAT1_LOW,
                     REG_AVE_VBAT1_HIGH - REG_AVE_VBAT1_LOW + 1,
                     (unsigned char *)&data->adcCheckData.regVbat1Ave);

        /// [AT-PM] : Compare counter register ; 01/27/2013
        if(data->adcCheckData.regCounter == data->adcCheckData.lastCounter) {
                data->adcCheckData.failCounterCurrent = data->adcCheckData.failCounterCurrent + 1;
                UG31_LOGN("[%s]: Counter fixed (%d) ... %d\n", __func__,
                          data->adcCheckData.regCounter, data->adcCheckData.failCounterCurrent);
        } else {
                data->adcCheckData.failCounterCurrent = 0;
        }
        data->adcCheckData.lastCounter = data->adcCheckData.regCounter;

        /// [AT-PM] : Compre VBat1 register ; 01/27/2013
        if(data->adcCheckData.regVbat1Ave == data->adcCheckData.lastVBat1Ave) {
                data->adcCheckData.failCounterVoltage = data->adcCheckData.failCounterVoltage + 1;
                UG31_LOGN("[%s]: VBat1 fixed (%d) ... %d\n", __func__,
                          data->adcCheckData.regVbat1Ave, data->adcCheckData.failCounterVoltage);
        } else {
                data->adcCheckData.failCounterVoltage = 0;
        }
        data->adcCheckData.lastVBat1Ave = data->adcCheckData.regVbat1Ave;

        /// [AT-PM] : Check ADC fail criteria ; 01/27/2013
        if(data->adcCheckData.failCounterCurrent > ADC_FAIL_CRITERIA) {
                data->adcCheckData.failCounterCurrent = 0;
                return (_UPI_TRUE_);
        }
        if(data->adcCheckData.failCounterVoltage > ADC_FAIL_CRITERIA) {
                data->adcCheckData.failCounterVoltage = 0;
                return (_UPI_TRUE_);
        }
        return (_UPI_FALSE_);
}

/**
 * @brief DecimateRst
 *
 *  Decimate reset filter of ADC
 *
 * @return  _UPI_NULL_
 */
void DecimateRst(void)
{
        _sys_u8_ tmp8;

        tmp8 = 0x00;
        API_I2C_Read(NORMAL, UG31XX_I2C_HIGH_SPEED_MODE, UG31XX_I2C_TEM_BITS_MODE, REG_ALARM_EN, 1, &tmp8);
        tmp8 = tmp8 & (~ALARM_EN_DECIMATE_RST);
        API_I2C_Write(NORMAL, UG31XX_I2C_HIGH_SPEED_MODE, UG31XX_I2C_TEM_BITS_MODE, REG_ALARM_EN, 1, &tmp8);
        tmp8 = tmp8 | ALARM_EN_DECIMATE_RST;
        API_I2C_Write(NORMAL, UG31XX_I2C_HIGH_SPEED_MODE, UG31XX_I2C_TEM_BITS_MODE, REG_ALARM_EN, 1,  &tmp8);
        UG31_LOGN("[%s]: DECIMATE_RST\n", __func__);
}

/**
 * @brief AlarmEnable
 *
 *  Enable alarm
 *
 * @para  alarm REG_ALARM_EN bits
 * @return  NULL
 */
void AlarmEnable(_sys_u8_ alarm)
{
        _sys_u8_ tmp8;

        tmp8 = 0;
        API_I2C_Read(NORMAL, UG31XX_I2C_HIGH_SPEED_MODE, UG31XX_I2C_TEM_BITS_MODE, REG_ALARM_EN, 1, &tmp8);
        tmp8 = tmp8 | alarm;
        API_I2C_Write(NORMAL, UG31XX_I2C_HIGH_SPEED_MODE, UG31XX_I2C_TEM_BITS_MODE, REG_ALARM_EN, 1, &tmp8);
}

/**
 * @brief AlarmDisable
 *
 *  Disable alarm
 *
 * @para  alarm REG_ALARM_EN bits
 * @return  NULL
 */
void AlarmDisable(_sys_u8_ alarm)
{
        _sys_u8_ tmp8;

        tmp8 = 0;
        API_I2C_Read(NORMAL, UG31XX_I2C_HIGH_SPEED_MODE, UG31XX_I2C_TEM_BITS_MODE, REG_ALARM_EN, 1, &tmp8);
        tmp8 = tmp8 & (~alarm);
        API_I2C_Write(NORMAL, UG31XX_I2C_HIGH_SPEED_MODE, UG31XX_I2C_TEM_BITS_MODE, REG_ALARM_EN, 1, &tmp8);
}

#define SYS_ALARM_STS_UV1     (ALARM2_STATUS_UV1_ALARM)
#define SYS_ALARM_STS_OV1     (ALARM2_STATUS_OV1_ALARM)
#define SYS_ALARM_STS_UV2     (ALARM2_STATUS_UV2_ALARM)
#define SYS_ALARM_STS_OV2     (ALARM2_STATUS_OV2_ALARM)
#define SYS_ALARM_STS_UV3     (ALARM2_STATUS_UV3_ALARM)
#define SYS_ALARM_STS_OV3     (ALARM2_STATUS_OV3_ALARM)
#define SYS_ALARM_STS_UET     (ALARM1_STATUS_UET_ALARM<<8)
#define SYS_ALARM_STS_OET     (ALARM1_STATUS_OET_ALARM<<8)
#define SYS_ALARM_STS_UIT     (ALARM1_STATUS_UIT_ALARM<<8)
#define SYS_ALARM_STS_OIT     (ALARM1_STATUS_OIT_ALARM<<8)
#define SYS_ALARM_STS_DOC     (ALARM1_STATUS_DOC_ALARM<<8)
#define SYS_ALARM_STS_COC     (ALARM1_STATUS_COC_ALARM<<8)

/**
 * @brief ProcUVAlarm
 *
 *  UV alarm function
 *
 * @para  data  address of SystemDataType
 * @return  NULL
 */
void ProcUVAlarm(SystemDataType *data)
{
        _sys_u8_ tmp8[4];

        /// [AT-PM] : Check alarm is enable or not ; 04/08/2013
        if(!(data->ggbParameter->alarmEnable & CELL_PARAMETER_ALARM_EN_UV)) {
                /// [AT-PM] : Disable UV and OV alarm ; 04/08/2013
                AlarmDisable(ALARM_EN_V1_ALARM_EN);
                return;
        }

        if(data->uvAlarm.state == _UPI_TRUE_) {
                /// [AT-PM] : UV alarm has been set -> Wait for OV alarm ; 04/08/2013
                if(data->alarmSts & SYS_ALARM_STS_OV1) {
                        data->uvAlarm.state = _UPI_FALSE_;

                        /// [AT-PM] : Release UV alarm by disable ; 04/08/2013
                        AlarmDisable(ALARM_EN_V1_ALARM_EN);

                        /// [AT-PM] : UV release threshold reached -> set alarm threshold ; 04/08/2013
                        tmp8[0] = 0xff;
                        tmp8[1] = 0x7f;
                        tmp8[2] = (_sys_u8_)(data->uvAlarm.alarmThrd & 0x00ff);
                        tmp8[3] = (_sys_u8_)(data->uvAlarm.alarmThrd >> 8);
                } else {
                        /// [AT-PM] : UV state -> set release threshold ; 04/08/2013
                        tmp8[0] = (_sys_u8_)(data->uvAlarm.releaseThrd & 0x00ff);
                        tmp8[1] = (_sys_u8_)(data->uvAlarm.releaseThrd >> 8);
                        tmp8[2] = 0x00;
                        tmp8[3] = 0x00;
                }
        } else {
                /// [AT-PM] : Normal state ; 04/08/2013
                if(data->alarmSts & SYS_ALARM_STS_UV1) {
                        data->uvAlarm.state = _UPI_TRUE_;

                        /// [AT-PM] : Release UV alarm by disable ; 04/08/2013
                        AlarmDisable(ALARM_EN_V1_ALARM_EN);

                        /// [AT-PM] : UV alarm reached -> set release threshold ; 04/08/2013
                        tmp8[0] = (_sys_u8_)(data->uvAlarm.releaseThrd & 0x00ff);
                        tmp8[1] = (_sys_u8_)(data->uvAlarm.releaseThrd >> 8);
                        tmp8[2] = 0x00;
                        tmp8[3] = 0x00;
                } else {
                        /// [AT-PM] : Normal state -> set alarm threshold ; 04/08/2013
                        tmp8[0] = 0xff;
                        tmp8[1] = 0x7f;
                        tmp8[2] = (_sys_u8_)(data->uvAlarm.alarmThrd & 0x00ff);
                        tmp8[3] = (_sys_u8_)(data->uvAlarm.alarmThrd >> 8);
                }
        }

        /// [AT-PM] : Set alarm threshold ; 04/08/2013
        API_I2C_Write(SECURITY, UG31XX_I2C_HIGH_SPEED_MODE, UG31XX_I2C_TEM_BITS_MODE, REG_OV1_LOW, 4, &tmp8[0]);

        /// [AT-PM] : Enable UV and OV alarm ; 04/08/2013
        AlarmEnable(ALARM_EN_V1_ALARM_EN);
}

/**
 * @brief ProcETAlarm
 *
 *  UET and OET alarm function
 *
 * @para  data  address of SystemDataType
 * @return  NULL
 */
void ProcETAlarm(SystemDataType *data)
{
        _sys_u8_ tmp8[4];

        /// [AT-PM] : Check alarm is enable or not ; 04/08/2013
        if(!(data->ggbParameter->alarmEnable & (CELL_PARAMETER_ALARM_EN_UET | CELL_PARAMETER_ALARM_EN_OET))) {
                /// [AT-PM] : Disable UV and OV alarm ; 04/08/2013
                AlarmDisable(ALARM_EN_ET_ALARM_EN);
                return;
        }

        if(data->uetAlarm.state == _UPI_TRUE_) {
                /// [AT-PM] : UET alarm state -> wait for OET alarm ; 04/08/2013
                if(data->alarmSts & SYS_ALARM_STS_OET) {
                        data->uetAlarm.state = _UPI_FALSE_;

                        /// [AT-PM] : Release by disable ; 04/08/2013
                        AlarmDisable(ALARM_EN_ET_ALARM_EN);

                        /// [AT-PM] : UET release met -> set UET and OET alarm ; 04/08/2013
                        tmp8[0] = (_sys_u8_)(data->oetAlarm.alarmThrd & 0x00ff);
                        tmp8[1] = (_sys_u8_)(data->oetAlarm.alarmThrd >> 8);
                        tmp8[2] = (_sys_u8_)(data->uetAlarm.alarmThrd & 0x00ff);
                        tmp8[3] = (_sys_u8_)(data->uetAlarm.alarmThrd >> 8);
                } else {
                        /// [AT-PM] : Wait OET alarm ; 04/08/2013
                        tmp8[0] = (_sys_u8_)(data->uetAlarm.releaseThrd & 0x00ff);
                        tmp8[1] = (_sys_u8_)(data->uetAlarm.releaseThrd >> 8);
                        tmp8[2] = 0x00;
                        tmp8[3] = 0x00;
                }
        } else if(data->oetAlarm.state == _UPI_TRUE_) {
                /// [AT-PM] : OET alarm state -> wait for UET alarm ; 04/08/2013
                if(data->alarmSts & SYS_ALARM_STS_UET) {
                        data->oetAlarm.state = _UPI_FALSE_;

                        /// [AT-PM] : Release by disable ; 04/08/2013
                        AlarmDisable(ALARM_EN_ET_ALARM_EN);

                        /// [AT-PM] : OET release met -> set UET and OET alarm ; 04/08/2013
                        tmp8[0] = (_sys_u8_)(data->oetAlarm.alarmThrd & 0x00ff);
                        tmp8[1] = (_sys_u8_)(data->oetAlarm.alarmThrd >> 8);
                        tmp8[2] = (_sys_u8_)(data->uetAlarm.alarmThrd & 0x00ff);
                        tmp8[3] = (_sys_u8_)(data->uetAlarm.alarmThrd >> 8);
                } else {
                        /// [AT-PM] : Wait UET alarm ; 04/08/2013
                        tmp8[0] = 0xff;
                        tmp8[1] = 0x7f;
                        tmp8[2] = (_sys_u8_)(data->oetAlarm.releaseThrd & 0x00ff);
                        tmp8[3] = (_sys_u8_)(data->oetAlarm.releaseThrd >> 8);
                }
        } else {
                /// [AT-PM] : Normal state ; 04/08/2013
                if((data->alarmSts & SYS_ALARM_STS_UET) &&
                    (data->ggbParameter->alarmEnable & CELL_PARAMETER_ALARM_EN_UET)) {
                        data->uetAlarm.state = _UPI_TRUE_;

                        /// [AT-PM] : Release by disable ; 04/08/2013
                        AlarmDisable(ALARM_EN_ET_ALARM_EN);

                        /// [AT-PM] : UET is set -> set UET release threshold ; 04/08/2013
                        tmp8[0] = (_sys_u8_)(data->uetAlarm.releaseThrd & 0x00ff);
                        tmp8[1] = (_sys_u8_)(data->uetAlarm.releaseThrd >> 8);
                        tmp8[2] = 0x00;
                        tmp8[3] = 0x00;
                } else if((data->alarmSts & SYS_ALARM_STS_OET) &&
                          (data->ggbParameter->alarmEnable & CELL_PARAMETER_ALARM_EN_OET)) {
                        data->oetAlarm.state = _UPI_TRUE_;

                        /// [AT-PM] : Release by disable ; 04/08/2013
                        AlarmDisable(ALARM_EN_ET_ALARM_EN);

                        /// [AT-PM] : OET is set -> set OET release threshold ; 04/08/2013
                        tmp8[0] = 0xff;
                        tmp8[1] = 0x7f;
                        tmp8[2] = (_sys_u8_)(data->oetAlarm.releaseThrd & 0x00ff);
                        tmp8[3] = (_sys_u8_)(data->oetAlarm.releaseThrd >> 8);
                } else {
                        /// [AT-PM] : Set OET alarm threshold ; 04/08/2013
                        if(data->ggbParameter->alarmEnable & CELL_PARAMETER_ALARM_EN_OET) {
                                tmp8[0] = (_sys_u8_)(data->oetAlarm.alarmThrd & 0x00ff);
                                tmp8[1] = (_sys_u8_)(data->oetAlarm.alarmThrd >> 8);
                        } else {
                                tmp8[0] = 0xff;
                                tmp8[1] = 0x7f;
                        }
                        /// [AT-PM] : Set UET alarm threshold ; 04/11/2013
                        if(data->ggbParameter->alarmEnable & CELL_PARAMETER_ALARM_EN_UET) {
                                tmp8[2] = (_sys_u8_)(data->uetAlarm.alarmThrd & 0x00ff);
                                tmp8[3] = (_sys_u8_)(data->uetAlarm.alarmThrd >> 8);
                        } else {
                                tmp8[2] = 0x00;
                                tmp8[3] = 0x00;
                        }
                }
        }

        /// [AT-PM] : Set alarm threshold ; 04/08/2013
        API_I2C_Write(SECURITY, UG31XX_I2C_HIGH_SPEED_MODE, UG31XX_I2C_TEM_BITS_MODE, REG_EXTR_OVER_TEMP_LOW, 4, &tmp8[0]);

        /// [AT-PM] : Enable UV and OV alarm ; 04/08/2013
        AlarmEnable(ALARM_EN_ET_ALARM_EN);
}

/**
 * @brief EnableAlarm
 *
 *  Set UV, UET, and OET alarm functions
 *
 * @para  data  address of SystemDataType
 * @return  NULL
 */
void EnableAlarm(SystemDataType *data)
{
        /// [AT-PM] : UV alarm ; 04/08/2013
        ProcUVAlarm(data);

        /// [AT-PM] : UET and OET alarm ; 04/08/2013
        ProcETAlarm(data);
}

/// =============================================
/// [AT-PM] : Extern function region
/// =============================================

/**
 * @brief UpiInitSystemData
 *
 *  Initialize system variables
 *
 * @para  data  address of SystemDataType
 * @return  SYSTEM_RTN_CODE
 */
SYSTEM_RTN_CODE UpiInitSystemData(SystemDataType *data)
{
        /// [AT-PM] : Initialize variables ; 01/30/2013
        data->preITAve = 0;
        data->cellNum = 0;

        /// [AT-PM] : Load GGB file ; 01/30/2013
        UG31_LOGN("[%s]: Read GGB\n", __func__);
#if defined(uG31xx_OS_WINDOWS)

        if(!ReadGGBFileToCellDataAndInitSetting(data))

#else   ///< else of defined(uG31xx_OS_WINDOWS)

        if(!ReadGGBXFileToCellDataAndInitSetting(data))

#endif  ///< end of defined(uG31xx_OS_WINDOWS)
        {
                return (SYSTEM_RTN_READ_GGB_FAIL);
        }

        /// [AT-PM] : Set cell number ; 01/31/2013
        GetCellNum(data);
        return (SYSTEM_RTN_PASS);
}

/**
 * @brief UpiCheckICActive
 *
 *  Check IC is actived or not
 *
 * @return  _UPI_TRUE_ if uG31xx is not actived
 */
_upi_bool_ UpiCheckICActive(void)
{
        _upi_u8_ tmp = 0;

        if(!API_I2C_Read(NORMAL, UG31XX_I2C_HIGH_SPEED_MODE, UG31XX_I2C_TEM_BITS_MODE, REG_MODE, 1, &tmp)) {
                UG31_LOGE("[%s]: Get GG_RUN fail.\n", __func__);
                return (_UPI_TRUE_);
        }

        if(((tmp & MODE_GG_RUN) == GG_RUN_OPERATION_MODE) && (tmp != 0xFF)) {
                UG31_LOGN("[%s]: uG31xx is actived.\n", __func__);
                return (_UPI_FALSE_);
        }
        UG31_LOGE("[%s]: uG31xx is NOT actived.\n", __func__);
        return (_UPI_TRUE_);
}

/**
 * @brief UpiActiveUg31xx
 *
 *  Active uG31xx
 *
 * @return  SYSTEM_RTN_CODE
 */
SYSTEM_RTN_CODE UpiActiveUg31xx(void)
{
        _sys_u8_ tmp8;

        /// [AT-PM] : Reset uG31xx ; 01/31/2013
        tmp8 = PORDET_W_SOFTRESET | IO1DATA_W_HIGH;
        if(!API_I2C_Write(NORMAL, UG31XX_I2C_HIGH_SPEED_MODE, UG31XX_I2C_TEM_BITS_MODE, REG_CTRL1, 1, &tmp8)) {
                return (SYSTEM_RTN_I2C_FAIL);
        }
        tmp8 = IO1DATA_W_HIGH;
        if(!API_I2C_Write(NORMAL, UG31XX_I2C_HIGH_SPEED_MODE, UG31XX_I2C_TEM_BITS_MODE, REG_CTRL1, 1, &tmp8)) {
                return (SYSTEM_RTN_I2C_FAIL);
        }

        /// [AT-PM] : Active uG31xx ; 01/31/2013
        tmp8 = CTRL1_GG_RST | IO1DATA_W_HIGH;
        if(!API_I2C_Write(NORMAL, UG31XX_I2C_HIGH_SPEED_MODE, UG31XX_I2C_TEM_BITS_MODE, REG_CTRL1, 1, &tmp8)) {
                return (SYSTEM_RTN_I2C_FAIL);
        }
        tmp8 = GG_RUN_OPERATION_MODE;
        if(!API_I2C_Write(NORMAL, UG31XX_I2C_HIGH_SPEED_MODE, UG31XX_I2C_TEM_BITS_MODE, REG_MODE, 1, &tmp8)) {
                return (SYSTEM_RTN_I2C_FAIL);
        }

#ifdef	uG31xx_BOOT_LOADER
        /// [AT-PM] : Delay 255mS for system stable ; 01/31/2013
        SleepMiniSecond(255);     //2012/08/29/Jacky, need wait 255 ms
#endif	///< end of uG31xx_BOOT_LOADER
        return (SYSTEM_RTN_PASS);
}

/**
 * @brief UpiSetupAdc
 *
 *  Setup ADC configurations
 *
 * @para  data  address of SystemDataType
 * @return  _UPI_NULL_
 */
void UpiSetupAdc(SystemDataType *data)
{
        _sys_u8_ tmp8;

        tmp8 = 0;
        /// [AT-PM] : Set ADC chop function ; 01/31/2013
        SetupAdcChopFunction(data);

        /// [AT-PM] : Set ADC1 queue ; 01/31/2013
        SetupAdc1Queue(data);

        /// [AT-PM] : Set ADC2 queue ; 01/31/2013
        SetupAdc2Quene(data);

        /// [AT-PM] : Enable ADC ; 01/31/2013
        API_I2C_Read(SECURITY,
                     UG31XX_I2C_HIGH_SPEED_MODE,
                     UG31XX_I2C_TEM_BITS_MODE,
                     REG_INTR_CTRL_A,
                     1,
                     &tmp8);
        tmp8 = tmp8 | (INTR_CTRL_A_ADC2_EN | INTR_CTRL_A_ADC1_EN);
        API_I2C_Write(SECURITY,
                      UG31XX_I2C_HIGH_SPEED_MODE,
                      UG31XX_I2C_TEM_BITS_MODE,
                      REG_INTR_CTRL_A,
                      1,
                      &tmp8);

        /// [AT-PM] : Decimate reset ; 01/31/2013
        DecimateRst();

        /// [AT-PM] : Enable CBC function ; 01/31/2013
        EnableCbc(data);
}

/**
 * @brief UpiSetupSystem
 *
 *  Setup uG31xx system
 *
 * @para  data  address of SystemDataType
 * @return  _UPI_NULL_
 */
void UpiSetupSystem(SystemDataType *data)
{
        _sys_u8_ tmp8;

        tmp8 = 0;
        /// [AT-PM] : Enable IC type ; 01/31/2013
        EnableICType(data);

        /// [AT-PM] : Configure GPIO ; 01/31/2013
        ConfigureGpio(data);

        /// [AT-PM] : Enable cell ; 01/31/2013
        API_I2C_Read(SECURITY,
                     UG31XX_I2C_HIGH_SPEED_MODE,
                     UG31XX_I2C_TEM_BITS_MODE,
                     REG_CELL_EN,
                     1,
                     &tmp8);
        tmp8 = tmp8 | (CELL_EN1 | CELL_EN0);
        API_I2C_Write(SECURITY,
                      UG31XX_I2C_HIGH_SPEED_MODE,
                      UG31XX_I2C_TEM_BITS_MODE,
                      REG_CELL_EN,
                      1,
                      &tmp8);
}

#define OSC_CNT_TARG          (512)
#define OSC_CP_SCL_L_TIME     (15739)     ///< [AT-PM] : 15.73876mS ; 05/20/2013
#define OSC_KHZ_TO_HZ         (1000)
#define OSC_FREQ_CONVERT(x)   (x*OSC_KHZ_TO_HZ*OSC_KHZ_TO_HZ/OSC_CP_SCL_L_TIME)

/**
 * @brief UpiCalculateOscFreq
 *
 *  OSC calibration
 *    oscCnt25[9:0] = oscCntTarg[9:0] + oscDeltaCode25[7:0]
 *    oscCnt80[9:0] = oscCntTarg[9:0] + oscDeltaCode80[7:0]
 *    oscCnt[9:0] = m*ITcode[15:8] + C[9:0]
 *    m = (oscCnt80[9:0]-oscCnt25[9:0])/(iTcode80[7:0]-iTcode25[7:0])
 *    c = oscCnt25[9:0] - m*ITcode25[7:0]
 *    write oscCnt[9:0] to register 0x97-98
 *
 * @para  data  address of SystemDataType
 * @return  _UPI_NULL_
 */
void UpiCalculateOscFreq(SystemDataType *data)
{
        _sys_u32_ oscFreq25;
        _sys_u32_ oscFreq80;
        _sys_u16_ aveIT;
        _sys_s32_ tmp32;
        _sys_s32_ deltaT;

        /// [AT-PM] : Convert OSC frequency from OTP ; 05/17/2013
        oscFreq25 = OSC_CNT_TARG + data->otpData->oscDeltaCode25;
        oscFreq25 = oscFreq25*OSC_KHZ_TO_HZ*OSC_KHZ_TO_HZ/OSC_CP_SCL_L_TIME;
        UG31_LOGN("[%s]: oscFreq25 = %d (%d)\n", __func__, (int)oscFreq25, data->otpData->oscDeltaCode25);
        oscFreq80 = OSC_CNT_TARG + data->otpData->oscDeltaCode80;
        oscFreq80 = oscFreq80*OSC_KHZ_TO_HZ*OSC_KHZ_TO_HZ/OSC_CP_SCL_L_TIME;
        UG31_LOGN("[%s]: oscFreq80 = %d (%d)\n", __func__, (int)oscFreq80, data->otpData->oscDeltaCode80);

        /// [AT-PM] : Read ITAve ; 01/27/2013
        aveIT = 0;
        API_I2C_Read(NORMAL,
                     UG31XX_I2C_HIGH_SPEED_MODE,
                     UG31XX_I2C_TEM_BITS_MODE,
                     REG_AVE_IT_LOW,
                     REG_AVE_IT_HIGH - REG_AVE_IT_LOW + 1,
                     (_sys_u8_ *)&aveIT);
        UG31_LOGN("[%s]: aveIT = %d (%d <-> %d)\n", __func__, aveIT, data->otpData->aveIT80, data->otpData->aveIT25);

        tmp32 = (_sys_s32_)oscFreq80;
        tmp32 = tmp32 - oscFreq25;
        deltaT = (_sys_s32_)aveIT;
        deltaT = deltaT - data->otpData->aveIT25;
        tmp32 = tmp32*deltaT;
        deltaT = (_sys_s32_)data->otpData->aveIT80;
        deltaT = deltaT - data->otpData->aveIT25;
        tmp32 = tmp32/deltaT + oscFreq25;
        data->oscFreq = (_sys_u32_)tmp32;
        UG31_LOGN("[%s]: OSC frequency = %dHz\n", __func__, (int)data->oscFreq);
}

/**
 * @brief UpiAdcStatus
 *
 *  Check ADC status
 *
 * @para  data  address of SystemDataType
 * @return  _UPI_NULL_
 */
void UpiAdcStatus(SystemDataType *data)
{
        if(CheckAdcStatusFail(data) == _UPI_TRUE_) {    //check ADC Code frozen
                DecimateRst();
        }
}

#define BACKUP_TIME_BYTE3           (REG_COC_LOW)
#define BACKUP_TIME_BYTE2           (REG_COC_HIGH)
#define BACKUP_NAC_HIGH             (REG_CBC21_LOW)
#define BACKUP_NAC_LOW              (REG_CBC21_HIGH)
#define BACKUP_LMD_HIGH             (REG_CBC32_LOW)
#define BACKUP_LMD_LOW              (REG_CBC32_HIGH)
#define BACKUP_TABLE_UPDATE_IDX     (REG_UV3_HIGH)
#define BACKUP_DELTA_CAP_HIGH       (REG_DOC_LOW)
#define BACKUP_DELTA_CAP_LOW        (REG_DOC_HIGH)
#define BACKUP_ADC1_CONV_TIME_HIGH  (REG_OVP_HIGH)
#define BACKUP_ADC1_CONV_TIME_LOW   (REG_OVP_LOW)
#define BACKUP_CYCLE_COUNT_LOW      (REG_UVP_LOW)
#define BACKUP_CYCLE_COUNT_HIGH     (REG_UVP_HIGH)
#define BACKUP_CC_OFFSET            (REG_INTR_OVER_TEMP_LOW)
#define BACKUP_STANDBY_RATIO        (REG_INTR_OVER_TEMP_HIGH)

/**
 * @brief UpiLoadBatInfoFromIC
 *
 *  Load battery information from uG31xx
 *
 * @para  data  address of SystemDataType
 * @return  _UPI_NULL_
 */
void UpiLoadBatInfoFromIC(SystemDataType *data)
{
        _sys_u8_ *u8Ptr;
        _sys_u8_ u8Temp;
        _sys_u8_ u8TempHigh;
#ifdef	uG31xx_BOOT_LOADER
        _sys_u32_ u32Temp;
        _sys_u16_ u16Temp;
#endif	///< end of uG31xx_BOOT_LOADER

        u8Temp = 0;
        u8TempHigh = 0;

        //Load the time tag
        u8Ptr = (_sys_u8_ *)&data->timeTagFromIC;
        *u8Ptr = 0;
        *(u8Ptr + 1) = 0;
        API_I2C_Read(SECURITY,
                     UG31XX_I2C_HIGH_SPEED_MODE,
                     UG31XX_I2C_TEM_BITS_MODE,
                     BACKUP_TIME_BYTE2,
                     1,
                     &u8Temp);
        *(u8Ptr + 2) = u8Temp;
        API_I2C_Read(SECURITY,
                     UG31XX_I2C_HIGH_SPEED_MODE,
                     UG31XX_I2C_TEM_BITS_MODE,
                     BACKUP_TIME_BYTE3,
                     1,
                     &u8Temp);
        *(u8Ptr + 3) = u8Temp;

        //Load the NAC
        API_I2C_Read(SECURITY,
                     UG31XX_I2C_HIGH_SPEED_MODE,
                     UG31XX_I2C_TEM_BITS_MODE,
                     BACKUP_NAC_HIGH,
                     1,
                     &u8TempHigh);
        API_I2C_Read(SECURITY,
                     UG31XX_I2C_HIGH_SPEED_MODE,
                     UG31XX_I2C_TEM_BITS_MODE,
                     BACKUP_NAC_LOW,
                     1,
                     &u8Temp);
        data->rmFromIC = (_sys_u16_)u8TempHigh;
        data->rmFromIC = data->rmFromIC*256 + u8Temp;

        // Load LMD
        API_I2C_Read(SECURITY,
                     UG31XX_I2C_HIGH_SPEED_MODE,
                     UG31XX_I2C_TEM_BITS_MODE,
                     BACKUP_LMD_HIGH,
                     1,
                     &u8TempHigh);
        API_I2C_Read(SECURITY,
                     UG31XX_I2C_HIGH_SPEED_MODE,
                     UG31XX_I2C_TEM_BITS_MODE,
                     BACKUP_LMD_LOW,
                     1,
                     &u8Temp);
        data->fccFromIC = (_sys_u16_)u8TempHigh;
        data->fccFromIC = data->fccFromIC*256 + u8Temp;
        UG31_LOGI("[%s]:timeTag =%d/%x ms,NAC = %d mAh,LMD = %dmAh\n",
                  __func__,
                  (int)data->timeTagFromIC,
                  (int)data->timeTagFromIC,
                  data->rmFromIC,
                  data->fccFromIC);

        /// [AT-PM] : Load table update index ; 02/10/2013
        API_I2C_Read(SECURITY,
                     UG31XX_I2C_HIGH_SPEED_MODE,
                     UG31XX_I2C_TEM_BITS_MODE,
                     BACKUP_TABLE_UPDATE_IDX,
                     1,
                     &u8Temp);
        data->tableUpdateIdxFromIC = u8Temp;
        UG31_LOGN("[%s]: Table Update Index From IC = %d\n", __func__, data->tableUpdateIdxFromIC);

        /// [AT-PM] : Load delta capacity ; 02/10/2013
        API_I2C_Read(SECURITY,
                     UG31XX_I2C_HIGH_SPEED_MODE,
                     UG31XX_I2C_TEM_BITS_MODE,
                     BACKUP_DELTA_CAP_HIGH,
                     1,
                     &u8TempHigh);
        API_I2C_Read(SECURITY,
                     UG31XX_I2C_HIGH_SPEED_MODE,
                     UG31XX_I2C_TEM_BITS_MODE,
                     BACKUP_DELTA_CAP_LOW,
                     1,
                     &u8Temp);
        data->deltaCapFromIC = (_sys_u16_)u8TempHigh;
        data->deltaCapFromIC = data->deltaCapFromIC*256 + u8Temp;
        UG31_LOGN("[%s]: Delta Capacity From IC = %d (0x%02x%02x)\n", __func__, data->deltaCapFromIC, u8TempHigh, u8Temp);

        /// [AT-PM] : Load ADC1 conversion time ; 02/10/2013
        API_I2C_Read(SECURITY,
                     UG31XX_I2C_HIGH_SPEED_MODE,
                     UG31XX_I2C_TEM_BITS_MODE,
                     BACKUP_ADC1_CONV_TIME_HIGH,
                     1,
                     &u8TempHigh);
        API_I2C_Read(SECURITY,
                     UG31XX_I2C_HIGH_SPEED_MODE,
                     UG31XX_I2C_TEM_BITS_MODE,
                     BACKUP_ADC1_CONV_TIME_LOW,
                     1,
                     &u8Temp);
        data->adc1ConvTime = u8TempHigh*256 + u8Temp;
        UG31_LOGN("[%s]: ADC1 Conversion Time From IC = %d\n", __func__, data->adc1ConvTime);

        /// [AT-PM] : Load cycle count ; 10/14/2013
        API_I2C_Read(SECURITY,
                     UG31XX_I2C_HIGH_SPEED_MODE,
                     UG31XX_I2C_TEM_BITS_MODE,
                     BACKUP_CYCLE_COUNT_LOW,
                     1,
                     &u8Temp);
        API_I2C_Read(SECURITY,
                     UG31XX_I2C_HIGH_SPEED_MODE,
                     UG31XX_I2C_TEM_BITS_MODE,
                     BACKUP_CYCLE_COUNT_HIGH,
                     1,
                     &u8TempHigh);
        data->cycleCount = u8TempHigh;
        data->cycleCount = (data->cycleCount)*256 + u8Temp;
        UG31_LOGN("[%s]: Cycle Count From IC = %d\n", __func__, data->cycleCount);

        /// [AT-PM] : Load cc offset ; 12/10/2013
        u8Ptr = (_sys_u8_ *)&data->ccOffset;
        API_I2C_Read(SECURITY,
                     UG31XX_I2C_HIGH_SPEED_MODE,
                     UG31XX_I2C_TEM_BITS_MODE,
                     BACKUP_CC_OFFSET,
                     1,
                     u8Ptr);
        UG31_LOGN("[%s]: CC Offset From IC = %d\n", __func__, data->ccOffset);

        /// [AT-PM] : Load standby ratio ; 01/03/2014
        u8Ptr = (_sys_u8_ *)&data->standbyDsgRatio;
        API_I2C_Read(SECURITY,
                     UG31XX_I2C_HIGH_SPEED_MODE,
                     UG31XX_I2C_TEM_BITS_MODE,
                     BACKUP_STANDBY_RATIO,
                     1,
                     u8Ptr);
        UG31_LOGN("[%s]: Standby Ratio From IC = %d\n", __func__, data->standbyDsgRatio);

        /// [AT-PM] : Get RSOC ; 01/31/2013
        if(data->fccFromIC == 0) {
                data->rsocFromIC = 0;
        } else {
#ifdef	uG31xx_BOOT_LOADER
                u32Temp = (_sys_u32_)data->rmFromIC;
                u32Temp = u32Temp*100 + 50;
                u32Temp = u32Temp/data->fccFromIC;
                data->rsocFromIC = (_sys_u8_)u32Temp;
#else	///< else of uG31xx_BOOT_LOADER
                data->rsocFromIC = (_sys_u8_)CalculateRsoc(data->rmFromIC, data->fccFromIC);
#endif	///< end of uG31xx_BOOT_LOADER
        }

        data->rmFromICBackup = data->rmFromIC;
        data->fccFromICBackup = data->fccFromIC;
        data->rsocFromICBackup = data->rsocFromIC;
}

/**
 * @brief UpiUpdateBatInfoFromIC
 *
 *  Update battery information from uG31xx
 *
 * @para  data  address of SystemDataType
 * @para  deltaQ  delta capacity from coulomb counter
 * @return  _UPI_NULL_
 */
void UpiUpdateBatInfoFromIC(SystemDataType *data, _sys_s16_ deltaQ)
{
        _sys_s32_ tmp32;
        _sys_u8_ oldRsoc;

        oldRsoc = data->rsocFromIC;

        tmp32 = (_sys_s32_)data->rmFromIC;
        tmp32 = tmp32 + deltaQ;
#ifdef  UPI_UBOOT_DEBUG_MSG
        printf("[UpiUpdateBatInfoFromIC] RM = %d + %d = %d\n", data->rmFromIC, deltaQ, tmp32);
#endif  ///< end of UPI_UBOOT_DEBUG_MSG
        if(tmp32 < 0) {
                tmp32 = 0;
        }
        if(tmp32 > data->fccFromIC) {
                tmp32 = (_sys_s32_)data->fccFromIC;
        }
        UG31_LOGN("[%s]: RM = %d + %d = %d\n", __func__,
                  data->rmFromIC, deltaQ, (int)tmp32);
        data->rmFromIC = (_sys_u16_)tmp32;

#ifdef  UPI_UBOOT_DEBUG_MSG
        printf("[UpiUpdateBatInfoFromIC] fccFromIC = %d, rsocFromIC = %d\n", data->fccFromIC, data->rsocFromIC);
#endif  ///< end of UPI_UBOOT_DEBUG_MSG
        if(data->fccFromIC == 0) {
                data->rsocFromIC = 0;
                data->tpCount = 0;
                return;
        }

        /// [AT-PM] : Check RM should not larger than FCC ; 12/24/2013
        if(data->rmFromIC > data->fccFromIC) {
                data->rmFromIC = data->fccFromIC;
        }

        /// [AT-PM] : Calculate new RSOC ; 12/24/2013
#ifdef	uG31xx_BOOT_LOADER
        tmp32 = tmp32*100 + 50;
        tmp32 = tmp32/data->fccFromIC;
        data->rsocFromIC = (_sys_u8_)tmp32;
#else	///< else of uG31xx_BOOT_LOADER
        data->rsocFromIC = (_sys_u8_)CalculateRsoc(data->rmFromIC, data->fccFromIC);
#endif	///< end of uG31xx_BOOT_LOADER

        /// [AT-PM] : Check taper condition reached ; 12/26/2013
        if((data->voltage > data->ggbParameter->TPVoltage) &&
            (data->curr < data->ggbParameter->TPCurrent) &&
            (data->curr >= data->ggbParameter->standbyCurrent)) {
                data->tpCount = data->tpCount + 1;
                if(data->tpCount > 3) {
                        data->tpCount = 3;
                        data->rsocFromIC = 100;
                        data->rmFromIC = data->fccFromIC;
                        return;
                }
        } else {
                data->tpCount = 0;
        }

        /// [AT-PM] : Check RSOC not 100 before taper condition ; 12/26/2013
        if((data->curr >= data->ggbParameter->standbyCurrent) &&
            (data->rsocFromIC >= 100) &&
            (oldRsoc != 100)) {
                data->rsocFromIC = 99;
                tmp32 = (_sys_s32_)data->rsocFromIC;
                tmp32 = tmp32*data->fccFromIC/CONST_PERCENTAGE;
                data->rmFromIC = (_sys_u16_)tmp32;
                return;
        }

        /// [AT-PM] : EDVF reached and RSOC = 0 not in charging -> do nothing ; 12/26/2013
        if((data->rsocFromIC == 0) &&
            (data->voltage < data->ggbParameter->edv1Voltage) &&
            (data->curr < data->ggbParameter->standbyCurrent)) {
                return;
        }

        /// [AT-PM] : EDVF reached not in charging -> set RSOC to 0 ; 12/26/2013
        if((data->voltage < data->ggbParameter->edv1Voltage) &&
            (data->curr < data->ggbParameter->standbyCurrent)) {
                data->rsocFromIC = 0;
                data->rmFromIC = 0;
                return;
        }
}

/**
 * @brief UpiSaveBatInfoTOIC
 *
 *  Save battery information from uG31xx
 *
 * @para  data  address of SystemDataType
 * @return  _UPI_NULL_
 */
void UpiSaveBatInfoTOIC(SystemDataType *data)
{
        _sys_u8_ *u8Ptr;
        _sys_u8_ u8Temp;
        _sys_u8_ u8Temp1;
        _sys_u16_ u16Temp;

#if defined(uG31xx_OS_ANDROID)

        data->timeTagFromIC = GetSysTickCount();

#else   ///< else of defined(uG31xx_OS_ANDROID)

#if defined(BUILD_UG31XX_LIB)

        data->timeTagFromIC = GetSysTickCount();

#else   ///< else of defined(BUILD_UG31XX_LIB)

        data->timeTagFromIC = GetTickCount();

#endif  ///< end of defined(BUILD_UG31XX_LIB)

#endif  ///< end of defined(uG31xx_OS_ANDROID)
        UG31_LOGI("[%s]:timeTag =%d/%x ms,NAC = %d maH,LMD = %d maH\n",
                  __func__,
                  (int)data->timeTagFromIC,
                  (int)data->timeTagFromIC,
                  data->rmFromIC,
                  data->fccFromIC);

        //save the time tag
        u8Ptr = (_sys_u8_ *)&data->timeTagFromIC;
        API_I2C_Write(SECURITY,
                      UG31XX_I2C_HIGH_SPEED_MODE,
                      UG31XX_I2C_TEM_BITS_MODE,
                      BACKUP_TIME_BYTE2,
                      1,
                      (u8Ptr + 2));
        API_I2C_Write(SECURITY,
                      UG31XX_I2C_HIGH_SPEED_MODE,
                      UG31XX_I2C_TEM_BITS_MODE,
                      BACKUP_TIME_BYTE3,
                      1,
                      (u8Ptr + 3));

        //save the NAC
        u8Temp = (_sys_u8_)((data->rmFromIC & 0xff00)/256);
        API_I2C_Write(SECURITY,
                      UG31XX_I2C_HIGH_SPEED_MODE,
                      UG31XX_I2C_TEM_BITS_MODE,
                      BACKUP_NAC_HIGH,
                      1,
                      &u8Temp);
        u8Temp = (_sys_u8_)(data->rmFromIC & 0x00ff);
        API_I2C_Write(SECURITY,
                      UG31XX_I2C_HIGH_SPEED_MODE,
                      UG31XX_I2C_TEM_BITS_MODE,
                      BACKUP_NAC_LOW,
                      1,
                      &u8Temp);

        // save LMD
        u8Temp = (_sys_u8_)((data->fccFromIC & 0xff00)/256);
        API_I2C_Write(SECURITY,
                      UG31XX_I2C_HIGH_SPEED_MODE,
                      UG31XX_I2C_TEM_BITS_MODE,
                      BACKUP_LMD_HIGH,
                      1,
                      &u8Temp);
        u8Temp = (_sys_u8_)(data->fccFromIC & 0x00ff);
        API_I2C_Write(SECURITY,
                      UG31XX_I2C_HIGH_SPEED_MODE,
                      UG31XX_I2C_TEM_BITS_MODE,
                      BACKUP_LMD_LOW,
                      1,
                      &u8Temp);

        /// [AT-PM] : Save table update index ; 02/10/2013
        u8Temp = data->tableUpdateIdxFromIC;
        API_I2C_Write(SECURITY,
                      UG31XX_I2C_HIGH_SPEED_MODE,
                      UG31XX_I2C_TEM_BITS_MODE,
                      BACKUP_TABLE_UPDATE_IDX,
                      1,
                      &u8Temp);
        UG31_LOGN("[%s]: Save Table Update Index = %d\n", __func__, data->tableUpdateIdxFromIC);
        /// [AT-PM] : Save delta capacity ; 02/10/2013
        u16Temp = (_sys_u16_)data->deltaCapFromIC;
        u8Temp = (_sys_u8_)(u16Temp >> 8);
        API_I2C_Write(SECURITY,
                      UG31XX_I2C_HIGH_SPEED_MODE,
                      UG31XX_I2C_TEM_BITS_MODE,
                      BACKUP_DELTA_CAP_HIGH,
                      1,
                      &u8Temp);
        u8Temp1 = (_sys_u8_)(u16Temp & 0x00ff);
        API_I2C_Write(SECURITY,
                      UG31XX_I2C_HIGH_SPEED_MODE,
                      UG31XX_I2C_TEM_BITS_MODE,
                      BACKUP_DELTA_CAP_LOW,
                      1,
                      &u8Temp1);
        UG31_LOGN("[%s]: Save Delta Capacity = %d - 0x%02x%02x\n", __func__, data->deltaCapFromIC, u8Temp, u8Temp1);

        /// [AT-PM] : Save adc1 conversion time ; 02/10/2013
        u8Temp = data->adc1ConvTime >> 8;
        API_I2C_Write(SECURITY,
                      UG31XX_I2C_HIGH_SPEED_MODE,
                      UG31XX_I2C_TEM_BITS_MODE,
                      BACKUP_ADC1_CONV_TIME_HIGH,
                      1,
                      &u8Temp);
        u8Temp = data->adc1ConvTime & 0x00ff;
        API_I2C_Write(SECURITY,
                      UG31XX_I2C_HIGH_SPEED_MODE,
                      UG31XX_I2C_TEM_BITS_MODE,
                      BACKUP_ADC1_CONV_TIME_LOW,
                      1,
                      &u8Temp);
        UG31_LOGN("[%s]: Save ADC1 Conversion Time = %d\n", __func__, data->adc1ConvTime);

        /// [AT-PM] : Save cycle count ; 10/14/2013
        u8Temp = data->cycleCount >> 8;
        API_I2C_Write(SECURITY,
                      UG31XX_I2C_HIGH_SPEED_MODE,
                      UG31XX_I2C_TEM_BITS_MODE,
                      BACKUP_CYCLE_COUNT_HIGH,
                      1,
                      &u8Temp);
        u8Temp = data->cycleCount & 0x00ff;
        API_I2C_Write(SECURITY,
                      UG31XX_I2C_HIGH_SPEED_MODE,
                      UG31XX_I2C_TEM_BITS_MODE,
                      BACKUP_CYCLE_COUNT_LOW,
                      1,
                      &u8Temp);
        UG31_LOGN("[%s]: Save Cycle Count = %d\n", __func__, data->cycleCount);

        /// [AT-PM] : Save coulomb counter offset ; 12/10/2013
        u8Ptr = (_sys_u8_ *)&data->ccOffset;
        API_I2C_Write(SECURITY,
                      UG31XX_I2C_HIGH_SPEED_MODE,
                      UG31XX_I2C_TEM_BITS_MODE,
                      BACKUP_CC_OFFSET,
                      1,
                      u8Ptr);
        UG31_LOGN("[%s]: Save coulomb counter offset = %d\n", __func__, data->ccOffset);

        /// [AT-PM] : Save standby ratio ; 01/03/2014
        u8Ptr = (_sys_u8_ *)&data->standbyDsgRatio;
        API_I2C_Write(SECURITY,
                      UG31XX_I2C_HIGH_SPEED_MODE,
                      UG31XX_I2C_TEM_BITS_MODE,
                      BACKUP_STANDBY_RATIO,
                      1,
                      u8Ptr);
        UG31_LOGN("[%s]: Save standby ratio = %d\n", __func__, data->standbyDsgRatio);
}

/**
 * @brief UpiInitAlarm
 *
 *  Initialize alarm function of uG3105
 *
 * @para  data  address of SystemDataType
 * @return  NULL
 */
void UpiInitAlarm(SystemDataType *data)
{
        /// [AT-PM] : Set GPIO as alarm pin ; 04/08/2013
        ConfigureGpio(data);

        /// [AT-PM] : Set delay time ; 04/08/2013
        API_I2C_Write(SECURITY,
                      UG31XX_I2C_HIGH_SPEED_MODE,
                      UG31XX_I2C_TEM_BITS_MODE,
                      REG_TIMER,
                      1,
                      &data->ggbParameter->alarm_timer);
        API_I2C_Write(SECURITY,
                      UG31XX_I2C_HIGH_SPEED_MODE,
                      UG31XX_I2C_TEM_BITS_MODE,
                      REG_CLK_DIVA,
                      1,
                      &data->ggbParameter->clkDivA);
        API_I2C_Write(SECURITY,
                      UG31XX_I2C_HIGH_SPEED_MODE,
                      UG31XX_I2C_TEM_BITS_MODE,
                      REG_CLK_DIVB,
                      1,
                      &data->ggbParameter->clkDivB);

        /// [AT-PM] : Enable alarm ; 04/08/2013
        data->alarmSts = 0;
        data->uvAlarm.state = _UPI_FALSE_;
        data->uetAlarm.state = _UPI_FALSE_;
        data->oetAlarm.state = _UPI_FALSE_;
        EnableAlarm(data);
}

/**
 * @brief UpiAlarmStatus
 *
 *  Get alarm status
 *
 * @para  data  address of SystemDataType
 * @return  NULL
 */
_sys_u8_ UpiAlarmStatus(SystemDataType *data)
{
        _sys_u8_ sts;
        _sys_u8_ tmp8[2];

        sts = 0;
        tmp8[0] = 0;
        tmp8[1] = 0;

        /// [AT-PM] : Read alarm status from uG3105 ; 04/08/2013
        API_I2C_Read(NORMAL, UG31XX_I2C_HIGH_SPEED_MODE, UG31XX_I2C_TEM_BITS_MODE, REG_ALARM1_STATUS, 2, &tmp8[0]);
        data->alarmSts = (_sys_u16_)tmp8[0];
        data->alarmSts = data->alarmSts*256 + tmp8[1];

        /// [AT-PM] : Enable alarm ; 04/08/2013
        EnableAlarm(data);

        /// [AT-PM] : Update current alarm status ; 04/08/2013
        tmp8[0] = data->uvAlarm.state == _UPI_TRUE_ ? ALARM_STATUS_UV : 0;
        sts = sts | tmp8[0];
        tmp8[0] = data->uetAlarm.state == _UPI_TRUE_ ? ALARM_STATUS_UET : 0;
        sts = sts | tmp8[0];
        tmp8[0] = data->oetAlarm.state == _UPI_TRUE_ ? ALARM_STATUS_OET : 0;
        sts = sts | tmp8[0];
        return (sts);
}

typedef struct TableStorageST {
        _cap_u8_ addr;
        _cap_u8_ size;
} ALIGNED_ATTRIBUTE TableStorageType;

TableStorageType TableStorage[] = {
        { REG_RAM0,     32, },
        { REG_OV2_LOW,  7,  },
        { 0,            0,  },
};

/**
 * @brief UpiLoadTableFromIC
 *
 *  Load table form IC
 *
 * @para  data point of _sys_u8_ type
 * @return  NULL
 */
void UpiLoadTableFromIC(_sys_u8_ *data)
{
        _sys_u8_ idx;
        _sys_u8_ offset;

        idx = 0;
        offset = 0;
        while(TableStorage[idx].size) {
                API_I2C_Read((TableStorage[idx].addr < 0x80) ? NORMAL : SECURITY,
                             UG31XX_I2C_HIGH_SPEED_MODE,
                             UG31XX_I2C_TEM_BITS_MODE,
                             TableStorage[idx].addr,
                             TableStorage[idx].size,
                             &data[offset]);
                offset = offset + TableStorage[idx].size;
                idx = idx + 1;
        }

        idx = 0;
        while(idx < offset) {
                UG31_LOGD("[%s]: Load data[%d] = 0x%02x\n", __func__, idx, data[idx]);
                idx = idx + 1;
        }
}

/**
 * @brief UpiLoadTableFromIC
 *
 *  Save table to IC
 *
 * @para  data point of _sys_u8_ type
 * @return  NULL
 */
void UpiSaveTableToIC(_sys_u8_ *data, _sys_u8_ *buf, _sys_u8_ size)
{
        _sys_u8_ idx;
        _sys_u8_ addr;
        _sys_u8_ cnt;
        _sys_u8_ offset;

        idx = 0;
        offset = 0;
        while(TableStorage[idx].size) {
                addr = TableStorage[idx].addr;
                cnt = 0;
                while(cnt < TableStorage[idx].size) {
                        if(buf[offset] != data[offset]) {
                                API_I2C_Write((TableStorage[idx].addr < 0x80) ? NORMAL : SECURITY,
                                              UG31XX_I2C_HIGH_SPEED_MODE,
                                              UG31XX_I2C_TEM_BITS_MODE,
                                              addr,
                                              1,
                                              &data[offset]);
                                UG31_LOGD("[%s]: Save data[%d] = 0x%02x to 0x%02x\n", __func__, offset, data[offset], addr);
                        }
                        addr = addr + 1;
                        cnt = cnt + 1;
                        offset = offset + 1;
                }
                idx = idx + 1;
        }

        upi_memcpy(buf, data, size);
}

/**
 * @brief UpiSetupAdc1Queue
 *
 *  Setup ADC1 queue for normal operation
 *
 * @para  data  address of SystemDataType structure
 * @return  NULL
 */
void UpiSetupAdc1Queue(SystemDataType *data)
{
        _sys_u8_ adcQueue[4];
        _sys_u8_ tmp8;

        tmp8 = 0;
        /// [AT-PM] : Disable ADC1 ; 06/04/2013
        API_I2C_Read(SECURITY,
                     UG31XX_I2C_HIGH_SPEED_MODE,
                     UG31XX_I2C_TEM_BITS_MODE,
                     REG_INTR_CTRL_A,
                     1,
                     &tmp8);
        tmp8 = tmp8 & (~INTR_CTRL_A_ADC1_EN);
        API_I2C_Write(SECURITY,
                      UG31XX_I2C_HIGH_SPEED_MODE,
                      UG31XX_I2C_TEM_BITS_MODE,
                      REG_INTR_CTRL_A,
                      1,
                      &tmp8);

        /// [AT-PM] : Set ADC1 queue ; 06/04/2013
        adcQueue[0] = SET_A_IT | SET_B_IT | SET_C_ET | SET_D_ET;
        adcQueue[1] = SET_E_CURRENT | SET_F_CURRENT | SET_G_CURRENT | SET_H_CURRENT;
        adcQueue[2] = SET_I_CURRENT | SET_J_CURRENT | SET_K_CURRENT | SET_L_CURRENT;
        adcQueue[3] = SET_M_CURRENT | SET_N_CURRENT | SET_O_CURRENT | SET_P_CURRENT;

        API_I2C_Write(SECURITY,
                      UG31XX_I2C_HIGH_SPEED_MODE,
                      UG31XX_I2C_TEM_BITS_MODE,
                      REG_ADC_CTR_A,
                      4,
                      &adcQueue[0]);

        /// [AT-PM] : Enable ADC ; 01/31/2013
        API_I2C_Read(SECURITY,
                     UG31XX_I2C_HIGH_SPEED_MODE,
                     UG31XX_I2C_TEM_BITS_MODE,
                     REG_INTR_CTRL_A,
                     1,
                     &tmp8);
        tmp8 = tmp8 | (INTR_CTRL_A_ADC2_EN | INTR_CTRL_A_ADC1_EN);
        API_I2C_Write(SECURITY,
                      UG31XX_I2C_HIGH_SPEED_MODE,
                      UG31XX_I2C_TEM_BITS_MODE,
                      REG_INTR_CTRL_A,
                      1,
                      &tmp8);

        /// [AT-PM] : Decimate reset ; 01/31/2013
        DecimateRst();
}

/**
 * @brief UpiAllocateTableBuf
 *
 *  Allocate table buffer
 *
 * @para  data  address of SystemDataType structure
 * @return  NULL
 */
void UpiAllocateTableBuf(_sys_u8_ **data, _sys_u8_ *size)
{
        _sys_u8_ idx = 0;
        _sys_u8_ lSize = 0;

        while(TableStorage[idx].size) {
                lSize = lSize + TableStorage[idx].size;
                idx = idx + 1;
        }
        *size = lSize;

        upi_memset(*data, 0, lSize);

        UG31_LOGD("[%s]: Table buffer address = %d (%d)\n", __func__, (int)(*data), lSize);
}

/**
 * @brief UpiFreeTableBuf
 *
 *  Free table buffer
 *
 * @para  data  address of SystemDataType structure
 * @return  NULL
 */
void UpiFreeTableBuf(_sys_u8_ **data)
{
        UG31_LOGD("[%s]: Free address 0x%02x\n", __func__, (unsigned int)(*data));
}


/**
 * Copyright @ 2013 uPI Semiconductor Corp. All right reserved.
 * The information, images, and/or data contained in this material is copyrighted by uPI
 * Semiconductor Corp., and may not be distributed, modified, reproduced in whole or in part
 * without the prior, written consent of uPI Semiconductor Corp.
 */
