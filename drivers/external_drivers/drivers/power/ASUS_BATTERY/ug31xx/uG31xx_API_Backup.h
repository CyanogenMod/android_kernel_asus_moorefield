/**
 * @filename  uG31xx_API_Backup.h
 *
 *  Header of uG31xx_API_Backup.cpp
 *
 * @author  AllenTeng <allen_teng@upi-semi.com>
 * @revision  $Revision: 14 $
 */

#define BACKUP_BOOL_TRUE      (1)
#define BACKUP_BOOL_FALSE     (0)

enum BACKUP_FILE_STS {
  BACKUP_FILE_STS_CHECKING = 0,         ///< [AT-PM] : Initial state and check file exist at next time ; 11/18/2013
  BACKUP_FILE_STS_NOT_EXIST,            ///< [AT-PM] : Create backup file at next time ; 11/18/2013
  BACKUP_FILE_STS_EXIST,                ///< [AT-PM] : Check backup file at next time ; 11/18/2013
  BACKUP_FILE_STS_COMPARE,              ///< [AT-PM] : Check backup file after a period of time ; 11/18/2013
  BACKUP_FILE_STS_UPDATE,               ///< [AT-PM] : Update backup file ; 12/04/2013
  BACKUP_FILE_STS_UPDATE_BY_VERSION,    ///< [AT-PM] : Update backup file by version mistached ; 12/04/2013
  BACKUP_FILE_STS_VERSION_MISMATCH,     ///< [AT-PM] : Do full reset ; 11/18/2013
};

typedef unsigned char _backup_bool_;
typedef unsigned char _backup_u8_;
typedef unsigned short _backup_u16_;
typedef signed short _backup_s16_;
typedef unsigned long _backup_u32_;
typedef signed long _backup_s32_;

#define BACKUP_MAX_LOG_SUSPEND_DATA     (8)

typedef struct BackupSuspendDataST {
  CapacityDataType beforeCapData;
  MeasDataType beforeMeasData;

  CapacityDataType afterCapData;
  MeasDataType afterMeasData;
} ALIGNED_ATTRIBUTE BackupSuspendDataType;

#define MAX_BACKUP_BUFFER_SIZE      (128)

typedef struct BackupDataST {

  CapacityDataType *capData;
  SystemDataType *sysData;
  MeasDataType *measData;

  _backup_bool_ icDataAvailable;
  _backup_u8_ backupFileSts;
  _backup_u8_ backupBuffer[MAX_BACKUP_BUFFER_SIZE];
  _backup_u8_ backupBufferSize;
  _backup_u8_ backupFileRetryCnt;
  _backup_u32_ backupNacLmdAdjustCfg;

  _backup_u8_ backupSuspendIdx;
  BackupSuspendDataType backupSuspendData[BACKUP_MAX_LOG_SUSPEND_DATA];

  _backup_u16_ backupVolt1;
  _backup_u16_ backupVolt2;
  _backup_s16_ backupDeltaQ;
  
  #if defined (uG31xx_OS_WINDOWS)
    const wchar_t* backupFileName;
    const wchar_t* suspendFileName;
  #else
    char *backupFileName;
    char *suspendFileName;
  #endif

} ALIGNED_ATTRIBUTE BackupDataType;

/**
 * @brief UpiBackupData
 *
 *  Backup data from IC to system routine
 *
 * @para  data  address of BackupDataType
 * @return  _UPI_NULL_
 */
extern void UpiBackupData(BackupDataType *data);

/**
 * @brief UpiRestoreData
 *
 *  Restore data from system to IC routine
 *
 * @para  data  address of BackupDataType
 * @return  BACKUP_BOOL_TRUE if success
 */
extern _backup_bool_ UpiRestoreData(BackupDataType *data);

/**
 * @brief UpiInitBackupData
 *
 *  Initialize memory buffer of BackupDataType structure
 *
 * @para  data  address of BackupDataType
 * @return  NULL
 */
extern void UpiInitBackupData(BackupDataType *data);

/**
 * @brief UpiFreeBackupData
 *
 *  Free memory for BackupDataType
 *
 * @para  data  address of BackupDataType
 * @return  NULL
 */
extern void UpiFreeBackupData(BackupDataType *data);

/**
 * @brief UpiSaveSuspendData
 *
 *  Save suspend data for backup
 *
 * @para  data  address of BackupDataType
 * @return  NULL
 */
extern void UpiSaveSuspendData(BackupDataType *data);

/**
 * @brief UpiSaveResumeData
 *
 *  Save resume data for backup
 *
 * @para  data  address of BackupDataType
 * @return  NULL
 */
extern void UpiSaveResumeData(BackupDataType *data);

/**
 * @brief UpiWriteSuspendResumeData
 *
 *  Write suspend / resume data to file
 *
 * @para  data  address of BackupDataType
 * @return  NULL
 */
extern void UpiWriteSuspendResumeData(BackupDataType *data);

/**
 * @brief UpiGetBackupMemorySize
 *
 *  Get memory size used in backup module
 *
 * @return  memory size
 */
extern _backup_u32_ UpiGetBackupMemorySize(void);

/**
 * @brief UpiBackupVoltage
 *
 *  Backup voltage points for abnormal battery checking
 *
 * @para  data  address of BackupDataType
 * @return  NULL
 */
extern void UpiBackupVoltage(BackupDataType *data);

