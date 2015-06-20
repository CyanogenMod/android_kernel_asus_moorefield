/**
 * Copyright @ 2013 uPI Semiconductor Corp. All right reserved.
 * The information, images, and/or data contained in this material is copyrighted by uPI
 * Semiconductor Corp., and may not be distributed, modified, reproduced in whole or in part
 * without the prior, written consent of uPI Semiconductor Corp.
 */

/**
 * @filename  uG31xx_API_Backup.cpp
 *
 *  Backup data on uG31xx to a file in system
 *
 * @author  AllenTeng <allen_teng@upi-semi.com>
 * @revision  $Revision: 16 $
 */

#include "stdafx.h"     //windows need this??
#include "uG31xx_API.h"

#define UG31XX_BACKUP_FILE_ENABLE

#ifndef UG31XX_SHELL_ALGORITHM

static CapacityDataType CheckBackupFile_orgCapData;
static SystemDataType CheckBackupFile_orgSysData;
static BackupSuspendDataType UpiWriteSuspendResumeData_buf[BACKUP_MAX_LOG_SUSPEND_DATA];

#endif  ///< end of UG31XX_SHELL_ALGORITHM

/**
 * @brief CreateBackupBuffer
 *
 *  Create buffer for backup file operation
 *
 * @para  data  address of BackupDataType
 * @return  NULL
 */
void CreateBackupBuffer(BackupDataType *data)
{
        /// [AT-PM] : Count total size to be written to file ; 07/12/2013
        data->backupBufferSize = data->capData->tableSize;
        data->backupBufferSize = data->backupBufferSize + sizeof(data->sysData->rmFromIC);
        data->backupBufferSize = data->backupBufferSize + sizeof(data->sysData->fccFromIC);
        data->backupBufferSize = data->backupBufferSize + sizeof(data->sysData->timeTagFromIC);
        data->backupBufferSize = data->backupBufferSize + sizeof(data->sysData->tableUpdateIdxFromIC);
        data->backupBufferSize = data->backupBufferSize + sizeof(data->sysData->deltaCapFromIC);
        data->backupBufferSize = data->backupBufferSize + sizeof(data->sysData->adc1ConvTime);
        data->backupBufferSize = data->backupBufferSize + sizeof(data->sysData->rsocFromIC);
        data->backupBufferSize = data->backupBufferSize + sizeof(data->sysData->cycleCount);
        data->backupBufferSize = data->backupBufferSize + sizeof(data->sysData->ccOffset);
        data->backupBufferSize = data->backupBufferSize + sizeof(data->sysData->predictRsoc);
        data->backupBufferSize = data->backupBufferSize + sizeof(data->sysData->standbyDsgRatio);
        data->backupBufferSize = data->backupBufferSize + sizeof(data->backupVolt1);
        data->backupBufferSize = data->backupBufferSize + sizeof(data->backupVolt2);
        data->backupBufferSize = data->backupBufferSize + sizeof(data->capData->preDsgCharge);
        data->backupBufferSize = data->backupBufferSize + sizeof(_backup_u32_);     ///< [AT-PM] : Used for driver version ; 11/07/2013
        UG31_LOGN("[%s]: Total %d bytes need to be created for written to file.\n", __func__, data->backupBufferSize);

        /// [AT-PM] : Set memory buffer ; 07/12/2013
        if(data->backupBufferSize > MAX_BACKUP_BUFFER_SIZE) {
                UG31_LOGE("[%s]: Exceed maximum backup buffer size\n", __func__);
                data->backupBufferSize = MAX_BACKUP_BUFFER_SIZE;
        }
        upi_memset(data->backupBuffer, 0, (_upi_u32_)data->backupBufferSize);
}

/**
 * @brief PrepareData
 *
 *  Prepare data to be written to file
 *
 * @para  data  address of BackupDataType
 * @return  NULL
 */
void PrepareData(BackupDataType *data)
{
        _backup_u8_ *ptr;
        _backup_u32_ driverVer;

        driverVer = (_backup_u32_)UG31XX_DRIVER_VERSION;
        UG31_LOGN("[%s]: Driver version = %d (%d)\n", __func__, (int)driverVer, UG31XX_DRIVER_VERSION);

        ptr = data->backupBuffer;
        upi_memcpy(ptr, (_backup_u8_ *)data->capData->encriptTable, (_upi_u32_)data->capData->tableSize);
        ptr = ptr + data->capData->tableSize;
        upi_memcpy(ptr, (_backup_u8_ *)&data->sysData->rmFromIC, sizeof(data->sysData->rmFromIC)/sizeof(_backup_u8_));
        ptr = ptr + sizeof(data->sysData->rmFromIC)/sizeof(_backup_u8_);
        upi_memcpy(ptr, (_backup_u8_ *)&data->sysData->fccFromIC, sizeof(data->sysData->fccFromIC)/sizeof(_backup_u8_));
        ptr = ptr + sizeof(data->sysData->fccFromIC)/sizeof(_backup_u8_);
        upi_memcpy(ptr, (_backup_u8_ *)&data->sysData->timeTagFromIC, sizeof(data->sysData->timeTagFromIC)/sizeof(_backup_u8_));
        ptr = ptr + sizeof(data->sysData->timeTagFromIC)/sizeof(_backup_u8_);
        upi_memcpy(ptr, (_backup_u8_ *)&data->sysData->tableUpdateIdxFromIC, sizeof(data->sysData->tableUpdateIdxFromIC)/sizeof(_backup_u8_));
        ptr = ptr + sizeof(data->sysData->tableUpdateIdxFromIC)/sizeof(_backup_u8_);
        upi_memcpy(ptr, (_backup_u8_ *)&data->sysData->deltaCapFromIC, sizeof(data->sysData->deltaCapFromIC)/sizeof(_backup_u8_));
        ptr = ptr + sizeof(data->sysData->deltaCapFromIC)/sizeof(_backup_u8_);
        upi_memcpy(ptr, (_backup_u8_ *)&data->sysData->adc1ConvTime, sizeof(data->sysData->adc1ConvTime)/sizeof(_backup_u8_));
        ptr = ptr + sizeof(data->sysData->adc1ConvTime)/sizeof(_backup_u8_);
        upi_memcpy(ptr, (_backup_u8_ *)&data->sysData->rsocFromIC, sizeof(data->sysData->rsocFromIC)/sizeof(_backup_u8_));
        ptr = ptr + sizeof(data->sysData->rsocFromIC)/sizeof(_backup_u8_);
        upi_memcpy(ptr, (_backup_u8_ *)&data->sysData->cycleCount, sizeof(data->sysData->cycleCount)/sizeof(_backup_u8_));
        ptr = ptr + sizeof(data->sysData->cycleCount)/sizeof(_backup_u8_);
        upi_memcpy(ptr, (_backup_u8_ *)&data->sysData->ccOffset, sizeof(data->sysData->ccOffset)/sizeof(_backup_u8_));
        ptr = ptr + sizeof(data->sysData->ccOffset)/sizeof(_backup_u8_);
        upi_memcpy(ptr, (_backup_u8_ *)&data->sysData->predictRsoc, sizeof(data->sysData->predictRsoc)/sizeof(_backup_u8_));
        ptr = ptr + sizeof(data->sysData->predictRsoc)/sizeof(_backup_u8_);
        upi_memcpy(ptr, (_backup_u8_ *)&data->sysData->standbyDsgRatio, sizeof(data->sysData->standbyDsgRatio)/sizeof(_backup_u8_));
        ptr = ptr + sizeof(data->sysData->standbyDsgRatio)/sizeof(_backup_u8_);
        upi_memcpy(ptr, (_backup_u8_ *)&data->backupVolt1, sizeof(data->backupVolt1)/sizeof(_backup_u8_));
        ptr = ptr + sizeof(data->backupVolt1)/sizeof(_backup_u8_);
        upi_memcpy(ptr, (_backup_u8_ *)&data->backupVolt2, sizeof(data->backupVolt2)/sizeof(_backup_u8_));
        ptr = ptr + sizeof(data->backupVolt2)/sizeof(_backup_u8_);
        upi_memcpy(ptr, (_backup_u8_ *)&data->capData->preDsgCharge, sizeof(data->capData->preDsgCharge)/sizeof(_backup_u8_));
        ptr = ptr + sizeof(data->capData->preDsgCharge)/sizeof(_backup_u8_);
        upi_memcpy(ptr, (_backup_u8_ *)&driverVer, sizeof(driverVer)/sizeof(_backup_u8_));
}

/**
 * @brief ConvertData
 *
 *  Convert data from buffer
 *
 * @para  data  address of BackupDataType structure
 * @return  driver version
 */
_backup_u32_ ConvertData(BackupDataType *data)
{
        _backup_u8_ *ptr;
        _backup_u32_ driverVer;

        ptr = data->backupBuffer;
        upi_memcpy((_backup_u8_ *)data->capData->encriptTable, ptr, (_upi_u32_)data->capData->tableSize);
        ptr = ptr + data->capData->tableSize;
        upi_memcpy((_backup_u8_ *)&data->sysData->rmFromIC, ptr, sizeof(data->sysData->rmFromIC)/sizeof(_backup_u8_));
        ptr = ptr + sizeof(data->sysData->rmFromIC)/sizeof(_backup_u8_);
        upi_memcpy((_backup_u8_ *)&data->sysData->fccFromIC, ptr, sizeof(data->sysData->fccFromIC)/sizeof(_backup_u8_));
        ptr = ptr + sizeof(data->sysData->fccFromIC)/sizeof(_backup_u8_);
        upi_memcpy((_backup_u8_ *)&data->sysData->timeTagFromIC, ptr, sizeof(data->sysData->timeTagFromIC)/sizeof(_backup_u8_));
        ptr = ptr + sizeof(data->sysData->timeTagFromIC)/sizeof(_backup_u8_);
        upi_memcpy((_backup_u8_ *)&data->sysData->tableUpdateIdxFromIC, ptr, sizeof(data->sysData->tableUpdateIdxFromIC)/sizeof(_backup_u8_));
        ptr = ptr + sizeof(data->sysData->tableUpdateIdxFromIC)/sizeof(_backup_u8_);
        upi_memcpy((_backup_u8_ *)&data->sysData->deltaCapFromIC, ptr, sizeof(data->sysData->deltaCapFromIC)/sizeof(_backup_u8_));
        ptr = ptr + sizeof(data->sysData->deltaCapFromIC)/sizeof(_backup_u8_);
        upi_memcpy((_backup_u8_ *)&data->sysData->adc1ConvTime, ptr, sizeof(data->sysData->adc1ConvTime)/sizeof(_backup_u8_));
        ptr = ptr + sizeof(data->sysData->adc1ConvTime)/sizeof(_backup_u8_);
        upi_memcpy((_backup_u8_ *)&data->sysData->rsocFromIC, ptr, sizeof(data->sysData->rsocFromIC)/sizeof(_backup_u8_));
        ptr = ptr + sizeof(data->sysData->rsocFromIC)/sizeof(_backup_u8_);
        upi_memcpy((_backup_u8_ *)&data->sysData->cycleCount, ptr, sizeof(data->sysData->cycleCount)/sizeof(_backup_u8_));
        ptr = ptr + sizeof(data->sysData->cycleCount)/sizeof(_backup_u8_);
        upi_memcpy((_backup_u8_ *)&data->sysData->ccOffset, ptr, sizeof(data->sysData->ccOffset)/sizeof(_backup_u8_));
        ptr = ptr + sizeof(data->sysData->ccOffset)/sizeof(_backup_u8_);
        upi_memcpy((_backup_u8_ *)&data->sysData->predictRsoc, ptr, sizeof(data->sysData->predictRsoc)/sizeof(_backup_u8_));
        ptr = ptr + sizeof(data->sysData->predictRsoc)/sizeof(_backup_u8_);
        upi_memcpy((_backup_u8_ *)&data->sysData->standbyDsgRatio, ptr, sizeof(data->sysData->standbyDsgRatio)/sizeof(_backup_u8_));
        ptr = ptr + sizeof(data->sysData->standbyDsgRatio)/sizeof(_backup_u8_);
        upi_memcpy((_backup_u8_ *)&data->backupVolt1, ptr, sizeof(data->backupVolt1)/sizeof(_backup_u8_));
        ptr = ptr + sizeof(data->backupVolt1)/sizeof(_backup_u8_);
        upi_memcpy((_backup_u8_ *)&data->backupVolt2, ptr, sizeof(data->backupVolt2)/sizeof(_backup_u8_));
        ptr = ptr + sizeof(data->backupVolt2)/sizeof(_backup_u8_);
        upi_memcpy((_backup_u8_ *)&data->capData->preDsgCharge, ptr, sizeof(data->capData->preDsgCharge)/sizeof(_backup_u8_));
        ptr = ptr + sizeof(data->capData->preDsgCharge)/sizeof(_backup_u8_);
        upi_memcpy((_backup_u8_ *)&driverVer, ptr, sizeof(driverVer)/sizeof(_backup_u8_));
        UG31_LOGI("[%s]: Read driver version = %d (%d)\n", __func__, (int)driverVer, UG31XX_DRIVER_VERSION);
        return (driverVer);
}

enum CHECK_BACKUP_FILE_STS {
        CHECK_BACKUP_FILE_STS_PASS = 0,
        CHECK_BACKUP_FILE_STS_VERSION_MISMATCH,
        CHECK_BACKUP_FILE_STS_READ_FAIL,
        CHECK_BACKUP_FILE_STS_NEED_UPDATE,
        CHECK_BACKUP_FILE_STS_WRITE_FAIL,
};

/**
 * @brief CheckBackupFile
 *
 *  Check backup file is consisted with IC or not
 *
 * @para  data  address of BackupDataType
 * @return  CHECK_BACKUP_FILE_STS
 */
_backup_u8_ CheckBackupFile(BackupDataType *data)
{
        CapacityDataType *orgCapData;
        SystemDataType *orgSysData;
        _upi_bool_ rtn;
        _backup_u32_ driverVer;
        _backup_u8_ rtnU8;
        _backup_u16_ volt1;
        _backup_u16_ volt2;

        /// [AT-PM] : Create buffer ; 02/21/2013
#ifndef UG31XX_SHELL_ALGORITHM
        orgCapData = &CheckBackupFile_orgCapData;
        orgSysData = &CheckBackupFile_orgSysData;
#else   ///< else of UG31XX_SHELL_ALGORITHM
        orgCapData = (CapacityDataType *)upi_malloc(sizeof(CapacityDataType));
        orgSysData = (SystemDataType *)upi_malloc(sizeof(SystemDataType));
#endif  ///< end of UG31XX_SHELL_ALGORITHM
        upi_memcpy(orgCapData, data->capData, sizeof(CapacityDataType));
        upi_memcpy(orgSysData, data->sysData, sizeof(SystemDataType));
        volt1 = data->backupVolt1;
        volt2 = data->backupVolt2;

        /// [AT-PM] : Get data from file ; 02/21/2013
        rtn = read_backup_file(data->backupFileName, data->backupBuffer, data->backupBufferSize);
        if(rtn == _UPI_FALSE_) {
                UG31_LOGD("[%s]: Read data from backup file fail.\n", __func__);
#ifdef  UG31XX_SHELL_ALGORITHM
                upi_free(orgCapData);
                upi_free(orgSysData);
#endif  ///< end of UG31XX_SHELL_ALGORITHM
                return (CHECK_BACKUP_FILE_STS_READ_FAIL);
        }
        driverVer = ConvertData(data);

        /// [AT-PM] : Following information is not checked ; 02/21/2013
        data->sysData->rmFromIC = orgSysData->rmFromIC;
        data->sysData->timeTagFromIC = orgSysData->timeTagFromIC;
        data->sysData->deltaCapFromIC = orgSysData->deltaCapFromIC;
        data->sysData->adc1ConvTime = orgSysData->adc1ConvTime;
        data->capData->preDsgCharge = orgCapData->preDsgCharge;

        /// [AT-PM] : Check data ; 02/21/2013
        if(driverVer != UG31XX_DRIVER_VERSION) {
                UG31_LOGN("[%s]: Backup file version mismatched.\n", __func__);
                rtnU8 = CHECK_BACKUP_FILE_STS_VERSION_MISMATCH;
        } else if((upi_memcmp(orgCapData, data->capData, sizeof(CapacityDataType)) != 0) ||
                  (upi_memcmp(orgSysData, data->sysData, sizeof(SystemDataType)) != 0) ||
                  (volt1 != data->backupVolt1) ||
                  (volt2 != data->backupVolt2)) {
                UG31_LOGN("[%s]: Backup file needs to be updated\n", __func__);
                rtnU8 = CHECK_BACKUP_FILE_STS_NEED_UPDATE;
        } else {
                rtnU8 = CHECK_BACKUP_FILE_STS_PASS;
        }
#ifdef  UG31XX_SHELL_ALGORITHM
        upi_free(orgCapData);
        upi_free(orgSysData);
#endif  ///< end of UG31XX_SHELL_ALGORITHM
        return (rtnU8);
}

/**
 * @brief UpdateBackupFile
 *
 *  Update backup file
 *
 * @para  data  address of BackupDataType
 * @return  CHECK_BACKUP_FILE_STS
 */
_backup_u8_ UpdateBackupFile(BackupDataType *data)
{
        _upi_bool_ rtn;

        PrepareData(data);
        rtn = write_backup_file(data->backupFileName, data->backupBuffer, data->backupBufferSize);
        if(rtn == _UPI_FALSE_) {
                UG31_LOGD("[%s]: Write data to backup file fail.\n", __func__);
                return (CHECK_BACKUP_FILE_STS_WRITE_FAIL);
        }
        return (CHECK_BACKUP_FILE_STS_PASS);
}

/// =============================================
/// Extern Function Region
/// =============================================

#ifdef  UG31XX_SHELL_ALGORITHM

#define RETRY_CHECKING_THRESHOLD      (20)

#else   ///< else of UG31XX_SHELL_ALGORITHM

#define RETRY_CHECKING_THRESHOLD      (5)

#endif  ///< end of UG31XX_SHELL_ALGORITHM

/**
 * @brief UpiBackupData
 *
 *  Backup data from IC to system routine
 *
 * @para  data  address of BackupDataType
 * @return  _UPI_NULL_
 */
void UpiBackupData(BackupDataType *data)
{
        _backup_bool_ rtnBool;
        _backup_u8_ rtnU8;

#ifndef UG31XX_BACKUP_FILE_ENABLE
        rtnBool = _UPI_TRUE_;
        data->backupFileSts = BACKUP_FILE_STS_COMPARE;
#endif  ///< end of UG31XX_BACKUP_FILE_ENABLE

        switch(data->backupFileSts) {
        case  BACKUP_FILE_STS_CHECKING:
                /// [AT-PM] : Check backup file existed or not ; 02/21/2013
                rtnBool = is_file_exist(data->backupFileName);
                UG31_LOGN("[%s]: is_file_exist() = %d.\n", __func__, rtnBool);
                if(rtnBool == BACKUP_BOOL_TRUE) {
                        data->backupFileSts = BACKUP_FILE_STS_EXIST;
                        data->backupFileRetryCnt = 0;
                } else {
                        data->backupFileRetryCnt = data->backupFileRetryCnt + 1;
                        UG31_LOGN("[%s]: Check backup file retry count = %d\n", __func__, data->backupFileRetryCnt);
                        if(data->backupFileRetryCnt > RETRY_CHECKING_THRESHOLD) {
                                data->backupFileSts = BACKUP_FILE_STS_NOT_EXIST;
                                UG31_LOGE("[%s]: Backup file is not exist.\n", __func__);
                        }
                }
                break;
        case  BACKUP_FILE_STS_NOT_EXIST:
                /// [AT-PM] : Create backup file ; 02/21/2013
                PrepareData(data);
                rtnBool = create_backup_file(data->backupFileName, data->backupBuffer, data->backupBufferSize);
                UG31_LOGN("[%s]: create_backup_file() = %d.\n", __func__, rtnBool);
                if(rtnBool == BACKUP_BOOL_TRUE) {
                        data->backupFileSts = BACKUP_FILE_STS_EXIST;
                        data->icDataAvailable = BACKUP_BOOL_TRUE;
                } else {
                        UG31_LOGE("[%s]: Create backup file fail.\n", __func__);
                }
                break;
        case  BACKUP_FILE_STS_EXIST:
                data->backupFileSts = BACKUP_FILE_STS_COMPARE;
        case  BACKUP_FILE_STS_COMPARE:
                if(data->icDataAvailable == BACKUP_BOOL_TRUE) {
                        /// [AT-PM] : Check content of file is consist with IC or not ; 02/21/2013
                        rtnU8 = CheckBackupFile(data);
                        UG31_LOGN("[%s]: CheckBackupFile() = %d.\n", __func__, rtnU8);
                        if(rtnU8 == CHECK_BACKUP_FILE_STS_VERSION_MISMATCH) {
                                data->backupFileSts = BACKUP_FILE_STS_UPDATE_BY_VERSION;
                        } else if(rtnU8 == CHECK_BACKUP_FILE_STS_NEED_UPDATE) {
                                data->backupFileSts = BACKUP_FILE_STS_UPDATE;
                        } else if(rtnU8 == CHECK_BACKUP_FILE_STS_PASS) {
                                data->backupFileSts = BACKUP_FILE_STS_COMPARE;
                        } else {
                                data->backupFileSts = BACKUP_FILE_STS_UPDATE;
                        }
                } else {
                        data->backupFileSts = BACKUP_FILE_STS_CHECKING;
                }
                break;
        case  BACKUP_FILE_STS_UPDATE:
        case  BACKUP_FILE_STS_UPDATE_BY_VERSION:
                if(data->icDataAvailable == BACKUP_BOOL_TRUE) {
                        rtnU8 = UpdateBackupFile(data);
                        UG31_LOGN("[%s]: UpdateBackupFile() = %d.\n", __func__, rtnU8);
                        if(rtnU8 == CHECK_BACKUP_FILE_STS_PASS) {
                                if(data->backupFileSts == BACKUP_FILE_STS_UPDATE_BY_VERSION) {
                                        data->backupFileSts = BACKUP_FILE_STS_VERSION_MISMATCH;
                                } else {
                                        data->backupFileSts = BACKUP_FILE_STS_COMPARE;
                                }
                        }
                } else {
                        data->backupFileSts = BACKUP_FILE_STS_CHECKING;
                }
                break;
        default:
                /// [AT-PM] : Un-known state ; 02/21/2013
                data->backupFileSts = BACKUP_FILE_STS_NOT_EXIST;
                break;
        }
}

#define RESTORE_ABNORMAL_VOLT_RANGE_UPBND (100)
#define RESTORE_ABNORMAL_VOLT_RANGE_LWBND (-100)

/**
 * @brief UpiRestoreData
 *
 *  Restore data from system to IC routine
 *
 * @para  data  address of BackupDataType
 * @return  BACKUP_BOOL_TRUE if success
 */
_backup_bool_ UpiRestoreData(BackupDataType *data)
{
        _backup_bool_ rtn;
        _backup_u32_ driverVer;
        SystemDataType *orgSysData;
        _backup_s32_ tmp32;

        /// [AT-PM] : Create buffer ; 02/21/2013
#ifndef UG31XX_SHELL_ALGORITHM
        orgSysData = &CheckBackupFile_orgSysData;
#else   ///< else of UG31XX_SHELL_ALGORITHM
        orgSysData = (SystemDataType *)upi_malloc(sizeof(SystemDataType));
#endif  ///< end of UG31XX_SHELL_ALGORITHM
        upi_memcpy(orgSysData, data->sysData, sizeof(SystemDataType));

        /// [AT-PM] : Get data from file ; 02/21/2013
        rtn = read_backup_file(data->backupFileName, data->backupBuffer, data->backupBufferSize);
        if(rtn == _UPI_FALSE_) {
                UG31_LOGE("[%s]: Read data from backup file fail.\n", __func__);
                return (BACKUP_BOOL_FALSE);
        }
        driverVer = ConvertData(data);
        UG31_LOGI("[%s]: Driver version = %d\n", __func__, (int)driverVer);

        /// [AT-PM] : Keep following information ; 01/22/2014
        data->sysData->timeTagFromIC = orgSysData->timeTagFromIC;
        data->sysData->tableUpdateIdxFromIC = orgSysData->tableUpdateIdxFromIC;

        /// [AT-PM] : Check abnormal condition ; 01/22/2014
        if((data->backupVolt1 != 0) && (data->backupVolt2 != 0)) {
                tmp32 = (_backup_s32_)data->backupVolt1;
                tmp32 = tmp32 - data->backupVolt2;
                if((tmp32 < RESTORE_ABNORMAL_VOLT_RANGE_UPBND) && (tmp32 > RESTORE_ABNORMAL_VOLT_RANGE_LWBND)) {
                        data->sysData->rmFromIC = orgSysData->rmFromIC;
                        data->sysData->fccFromIC = orgSysData->fccFromIC;
                        data->sysData->rsocFromIC = orgSysData->rsocFromIC;
                }
        }
        return (BACKUP_BOOL_TRUE);
}

/**
 * @brief UpiInitBackupData
 *
 *  Initialize memory buffer of BackupDataType structure
 *
 * @para  data  address of BackupDataType
 * @return  NULL
 */
void UpiInitBackupData(BackupDataType *data)
{
        /// [AT-PM] : Backup data in GGB file ; 12/10/2013
        data->backupNacLmdAdjustCfg = (_backup_u32_)data->sysData->ggbParameter->NacLmdAdjustCfg;

        /// [AT-PM] : Memory buffer for backup file ; 12/03/2013
        data->backupFileRetryCnt = 0;
        CreateBackupBuffer(data);

        /// [AT-PM] : Memory buffer for suspend/resume data ; 12/03/2013
        data->backupSuspendIdx = BACKUP_MAX_LOG_SUSPEND_DATA;
        while(data->backupSuspendIdx) {
                data->backupSuspendIdx = data->backupSuspendIdx - 1;
                upi_memset(&data->backupSuspendData[data->backupSuspendIdx], 0, sizeof(BackupSuspendDataType));
                UG31_LOGN("[%s]: data->backupSuspendData[%d] = %d\n", __func__,
                          data->backupSuspendIdx, (int)(&data->backupSuspendData[data->backupSuspendIdx]));
        }

        /// [AT-PM] : Initialize backup voltage variables ; 01/24/2014
        data->backupVolt1 = 0;
        data->backupVolt2 = 0;
        data->backupDeltaQ = 0;
}

/**
 * @brief UpiFreeBackupData
 *
 *  Free memory for BackupDataType
 *
 * @para  data  address of BackupDataType
 * @return  NULL
 */
void UpiFreeBackupData(BackupDataType *data)
{
        /// [AT-PM] : Memory buffer for backup file ; 12/03/2013

        /// [AT-PM] : Memory buffer for suspend/resume data ; 12/03/2013
        data->backupSuspendIdx = BACKUP_MAX_LOG_SUSPEND_DATA;
        while(data->backupSuspendIdx) {
                data->backupSuspendIdx = data->backupSuspendIdx - 1;
                UG31_LOGN("[%s]: data->backupSuspendData[%d] = %d\n", __func__,
                          data->backupSuspendIdx, (int)&data->backupSuspendData[data->backupSuspendIdx]);
        }
}

/**
 * @brief UpiSaveSuspendData
 *
 *  Save suspend data for backup
 *
 * @para  data  address of BackupDataType
 * @return  NULL
 */
void UpiSaveSuspendData(BackupDataType *data)
{
        data->backupSuspendData[data->backupSuspendIdx].beforeCapData = *(data->capData);
        data->backupSuspendData[data->backupSuspendIdx].beforeMeasData = *(data->measData);
        UG31_LOGN("[%s]: Save suspend data to buffer %d\n", __func__, data->backupSuspendIdx);
}

/**
 * @brief UpiSaveResumeData
 *
 *  Save resume data for backup
 *
 * @para  data  address of BackupDataType
 * @return  NULL
 */
void UpiSaveResumeData(BackupDataType *data)
{
        data->backupSuspendData[data->backupSuspendIdx].afterCapData = *(data->capData);
        data->backupSuspendData[data->backupSuspendIdx].afterMeasData = *(data->measData);
        UG31_LOGN("[%s]: Save resume data to buffer %d\n", __func__, data->backupSuspendIdx);

        data->backupSuspendIdx = data->backupSuspendIdx + 1;
        if(data->backupSuspendIdx >= BACKUP_MAX_LOG_SUSPEND_DATA) {
                data->backupSuspendIdx = 0;
                while(data->backupSuspendIdx < (BACKUP_MAX_LOG_SUSPEND_DATA - 1)) {
                        upi_memcpy(&data->backupSuspendData[data->backupSuspendIdx],
                                   &data->backupSuspendData[data->backupSuspendIdx + 1],
                                   sizeof(BackupSuspendDataType));
                        data->backupSuspendIdx = data->backupSuspendIdx + 1;
                }

                upi_memset(&data->backupSuspendData[data->backupSuspendIdx], 0, sizeof(BackupSuspendDataType));
                UG31_LOGN("[%s]: Next suspend / resume data buffer is %d\n", __func__, data->backupSuspendIdx);
        }
}

/**
 * @brief UpiWriteSuspendResumeData
 *
 *  Write suspend / resume data to file
 *
 * @para  data  address of BackupDataType
 * @return  NULL
 */
void UpiWriteSuspendResumeData(BackupDataType *data)
{
        _backup_bool_ rtn;
        _backup_u8_ *buf;
        _backup_u32_ size;
        _backup_u32_ cmpResult;

        size = sizeof(BackupSuspendDataType)*BACKUP_MAX_LOG_SUSPEND_DATA;
#ifndef UG31XX_SHELL_ALGORITHM
        buf = (_backup_u8_ *)&UpiWriteSuspendResumeData_buf[0];
#else   ///< else of UG31XX_SHELL_ALGORITHM
        buf = (_backup_u8_ *)upi_malloc(size);
#endif  ///< end of UG31XX_SHELL_ALGORITHM

        rtn = read_backup_file(data->suspendFileName, (_upi_u8_ *)buf, size);
        UG31_LOGN("[%s]: Read suspend / resume data from file -> %d\n", __func__, rtn);

        if(rtn != _UPI_TRUE_) {
                rtn = write_backup_file(data->suspendFileName, (_upi_u8_ *)(&data->backupSuspendData[0]), size);
                UG31_LOGN("[%s]: Write suspend / resume data to file -> %d\n", __func__, rtn);
#ifdef  UG31XX_SHELL_ALGORITHM
                upi_free(buf);
#endif  ///< end of UG31XX_SHELL_ALGORITHM
                return;
        }

        cmpResult = upi_memcmp(buf, (_backup_u8_ *)(&data->backupSuspendData[0]), size);
        UG31_LOGN("[%s]: Compare suspend / resume data with file -> %d\n", __func__, (int)cmpResult);

        if(cmpResult != 0) {
                rtn = write_backup_file(data->suspendFileName, (_upi_u8_ *)(&data->backupSuspendData[0]), size);
                UG31_LOGN("[%s]: Write suspend / resume data to file -> %d\n", __func__, rtn);
        }
#ifdef  UG31XX_SHELL_ALGORITHM
        upi_free(buf);
#endif  ///< end of UG31XX_SHELL_ALGORITHM
}

/**
 * @brief UpiGetBackupMemorySize
 *
 *  Get memory size used in backup module
 *
 * @return  memory size
 */
_backup_u32_ UpiGetBackupMemorySize(void)
{
        _backup_u32_ totalSize;
        _backup_u32_ tmp;

        totalSize = 0;

#ifndef UG31XX_SHELL_ALGORITHM

        tmp = (_backup_u32_)sizeof(CheckBackupFile_orgCapData);
        totalSize = totalSize + tmp;
        UG31_LOGD("[%s]: memory size for CheckBackupFile_orgCapData = %d (%d)\n", __func__, (int)tmp, (int)totalSize);

        tmp = (_backup_u32_)sizeof(CheckBackupFile_orgSysData);
        totalSize = totalSize + tmp;
        UG31_LOGD("[%s]: memory size for CheckBackupFile_orgSysData = %d (%d)\n", __func__, (int)tmp, (int)totalSize);

        tmp = (_backup_u32_)sizeof(UpiWriteSuspendResumeData_buf[0]);
        totalSize = totalSize + tmp*BACKUP_MAX_LOG_SUSPEND_DATA;
        UG31_LOGD("[%s]: memory size for UpiWriteSuspendResumeData_buf = %d (%d)\n", __func__, (int)tmp, (int)totalSize);

#endif  ///< end of UG31XX_SHELL_ALGORITHM

        return (totalSize);
}

#define BACKUP_VOLTAGE_DELTA_Q_THRESHOLD  (10)

/**
 * @brief UpiBackupVoltage
 *
 *  Backup voltage points for abnormal battery checking
 *
 * @para  data  address of BackupDataType
 * @return  NULL
 */
void UpiBackupVoltage(BackupDataType *data)
{
        _backup_s32_ tmp32;

        /// [AT-PM] : Cumulate delta Q ; 01/22/2014
        tmp32 = (_backup_s32_)data->measData->stepCap;
        tmp32 = tmp32*data->measData->curr;
        if(tmp32 < 0) {
                tmp32 = 0;
        } else {
                tmp32 = (_backup_s32_)data->measData->stepCap;
        }
        tmp32 = tmp32 + data->backupDeltaQ;
        data->backupDeltaQ = (_backup_u16_)tmp32;

        /// [AT-PM] : Update voltage point 1 ; 01/22/2014
        data->backupVolt1 = (_backup_u16_)data->measData->bat1Voltage;
        UG31_LOGN("[%s]: Update backup voltage point 1 = %d (%d)\n", __func__, data->backupVolt1, data->backupDeltaQ);

        /// [AT-PM] : Check delta Q ; 01/22/2014
        tmp32 = data->sysData->ggbParameter->ILMD;
        tmp32 = tmp32*BACKUP_VOLTAGE_DELTA_Q_THRESHOLD/CONST_PERCENTAGE;
        if(data->backupDeltaQ < 0) {
                tmp32 = tmp32*(-1);
                if(data->backupDeltaQ > tmp32) {
                        UG31_LOGN("[%s]: data->backupDeltaQ not reach threshold %d\n", __func__, tmp32);
                        return;
                }
        } else {
                if(data->backupDeltaQ < tmp32) {
                        UG31_LOGN("[%s]: data->backupDeltaQ not reach threshold %d\n", __func__, tmp32);
                        return;
                }
        }

        /// [AT-PM] : Update voltage point 2 ; 01/22/2014
        data->backupVolt2 = data->backupVolt1;
        data->backupDeltaQ = 0;
        UG31_LOGI("[%s]: Update backup voltage point 2 = %d (%d)\n", __func__, data->backupVolt1, data->backupDeltaQ);
}


/**
 * Copyright @ 2013 uPI Semiconductor Corp. All right reserved.
 * The information, images, and/or data contained in this material is copyrighted by uPI
 * Semiconductor Corp., and may not be distributed, modified, reproduced in whole or in part
 * without the prior, written consent of uPI Semiconductor Corp.
 */
