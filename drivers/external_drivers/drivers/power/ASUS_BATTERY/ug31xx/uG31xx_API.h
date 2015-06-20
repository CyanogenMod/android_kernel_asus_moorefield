/// ===========================================
/// uG31xx_API.h
/// ===========================================

#ifndef _UG31XXAPI_H_
#define _UG31XXAPI_H_

#include "uG31xx_Platform.h"
#include "global.h"
#include "uG31xx.h"
#include "uG31xx_API_Platform.h"
#include "typeDefine.h"
#include "uG31xx_Reg_Def.h"
#include "uG31xx_API_Otp.h"
#include "uG31xx_API_System.h"
#include "uG31xx_API_Measurement.h"
#include "uG31xx_API_Capacity.h"
#include "uG31xx_API_Backup.h"

#ifndef uG31xx_OS_WINDOWS

  #ifdef  uG31xx_BOOT_LOADER

    #include "ug31xx_boot.h"
    
  #else   ///< else of uG31xx_BOOT_LOADER

    #ifdef  ANDROID_SHELL_ALGORITHM

      #include "upi_algorithm.h"
      
    #else   ///< else of ANDROID_SHELL_ALGORITHM
    
      #include "ug31xx_gauge.h"
    
    #endif  ///< end of ANDROID_SHELL_ALGORITHM

  #endif	///< end of uG31xx_BOOT_LOADER

  #include "ug31xx_version.h"

#else   ///< else of uG31xx_OS_WINDOWS

  #define UG31XX_DRIVER_VERSION       (_UPI_NULL_)
  #define UG31XX_DRIVER_RELEASE_DATE  (_T("Windows No Release Date"))
  
#endif  ///< end of uG31xx_OS_WINDOWS

#define UG31XX_I2C_HIGH_SPEED_MODE    (_UPI_FALSE_)
#define UG31XX_I2C_TEM_BITS_MODE      (_UPI_FALSE_)

/* data struct */
typedef enum _GGSTATUS{
  UG_SUCCESS                    = 0x00,
  UG_FAIL		                    = 0x01,
  UG_NOT_DEF                    = 0x02,
  UG_INIT_OCV_FAIL	            = 0x03,
  UG_READ_GGB_FAIL              = 0x04,
  UG_ACTIVE_FAIL                = 0x05,
  UG_INIT_SUCCESS               = 0x06,
  UG_OTP_ISEMPTY                = 0x07,
  UG_OTP_PRODUCT_DISMATCH       = 0x08,

  UG_I2C_INIT_FAIL              = 0x10,
  UG_I2C_READ_SUCCESS           = 0x11,
  UG_I2C_READ_FAIL              = 0x12,
  UG_I2C_WRITE_SUCCESS          = 0x13,
  UG_I2C_WRITE_FAIL             = 0x14,

  UG_READ_REG_SUCCESS           = 0x20,
  UG_READ_REG_FAIL              = 0x21,

  UG_READ_DEVICE_INFO_SUCCESS   = 0x22,
  UG_READ_DEVICE_INFO_FAIL      = 0x23,
  UG_READ_DEVICE_ALARM_SUCCESS  = 0x24,
  UG_READ_DEVICE_ALARM_FAIL     = 0x25,
  UG_READ_DEVICE_RID_SUCCESS	  = 0x26,
  UG_READ_DEVICE_RID_FAIL		    = 0x27,
  UG_READ_ADC_FAIL						  = 0x28,			//new add for filter ADC Error Code

  UG_TI_CMD_OVERFLOW            = 0x30,

  UG_MEAS_FAIL                  = 0x40,
  UG_MEAS_FAIL_BATTERY_REMOVED  = 0x41,
  UG_MEAS_FAIL_ADC_ABNORMAL     = 0x42,
  UG_MEAS_FAIL_NTC_SHORT        = 0x43,
  UG_MEAS_FAIL_NTC_OPEN         = 0x44,

  UG_CAP_DATA_READY             = 0x50,
  UG_CAP_DATA_NOT_READY         = 0x51,
  UG_CAP_DATA_VERSION_MISMATCH  = 0x52,
}GGSTATUS;

/*
    GGSTATUS upiGG_Initial
    Description: Initial and active uG31xx function
    Input: .GGB(gas gauge battery) setting filename, need include complete path
	Output: UG_INIT_SUCCESS -> initial uG31xx success
	        UG_READ_GGB_FAIL -> read GGB file fail
			UG_INIT_I2C_FAIL -> initial I2C to open HID fail
			UG_ACTIVE_FAIL -> active uG31xx fail
*/
#if defined (uG31xx_OS_WINDOWS)

  EXPORTS GGSTATUS upiGG_Initial(char **pObj, const wchar_t* GGBFilename, const wchar_t* OtpFileName, unsigned char ForceReset = 0);

#else

  GGSTATUS upiGG_Initial(char **pObj, GGBX_FILE_HEADER *pGGBXBuf, unsigned char ForceReset);

#endif

/*
    GGSTATUS upiGG_CountInitQmax
    Description:
    Input: None
	Output: None
*/
//EXPORTS void upiGG_CountInitQmax(void);

/*
    GGSTATUS upiGG_ReadDevieRegister
    Description: Read GG_USER_REG from device to global variable and output
    Input: Pointer of sturct GG_USER_REG 
	Output: UG_READ_REG_SUCCESS -> read success
	        UG_READ_REG_FAIL -> read fail
*/
EXPORTS GGSTATUS upiGG_ReadAllRegister(char *pObj,GG_USER_REG* pExtUserReg, GG_USER2_REG* pExtUserReg2);

/*
    GGSTATUS upiGG_ReadDeviceInfo
    Description: Read GG_USER_REG from device and calculate GG_DEVICE_INFO, then write to global variable and output
    Input: Pointer of struct GG_DEVICE_INFO
	Output: UG_READ_DEVICE_INFO_SUCCESS -> calculate derive information sucess
	        UG_READ_DEVICE_INFO_FAIL -> calculate derive information fail
*/
EXPORTS GGSTATUS upiGG_ReadDeviceInfo(char *pObj,GG_DEVICE_INFO* pExtDeviceInfo);

/* GGSTATUS upiGG_ReadCapacity
    Description:
    Input:
	Output: None
*/
EXPORTS void upiGG_ReadCapacity(char *pObj,GG_CAPACITY *pExtCapacity);

#if defined (uG31xx_OS_WINDOWS)

EXPORTS void upiGG_AlgorithmSimulatorInit(char **pObj, const wchar_t* GGBFilename, 
                                                   MeasDataType *pMeasure, GG_CAP_LOG_TYPE *pCap,
                                                   _upi_u8_ *NacTable);
EXPORTS void upiGG_AlgorithmSimulatorRead(char *pObj, MeasDataType *pMeasure, 
                                                      GG_CAP_LOG_TYPE *pCap, _upi_u8_ *NacTable);
EXPORTS void upiGG_AlgorithmSimulatorClose(char **pObj);

#endif  ///< end of defined (uG31xx_OS_WINDOWS)

/**
 * @brief upiGG_GetAlarmStatus
 *
 *  Get alarm status
 *
 * @para  pAlarmStatus  address of alarm status
 * @return  UG_READ_DEVICE_ALARM_SUCCESS if success
 */
EXPORTS GGSTATUS upiGG_GetAlarmStatus(char *pObj, _upi_u8_ *pAlarmStatus);

/*
new add function for System suspend & wakeup

*/
EXPORTS GGSTATUS upiGG_PreSuspend(char *pObj);
EXPORTS GGSTATUS upiGG_Wakeup(char *pObj, _upi_bool_ dc_in_before);

/**
 * @brief upiGG_DumpRegister
 *
 *  Dump whole register value
 *
 * @para  pBuf  address of register value buffer
 * @return  data size
 */
EXPORTS _upi_u16_ upiGG_DumpRegister(char *pObj, _upi_u8_ *pBuf);

/**
 * @brief upiGG_DumpCellTable
 *
 *  Dump cell NAC table
 *
 * @para  pTable address of cell table
 * @return  _UPI_NULL_
 */
EXPORTS void upiGG_DumpCellTable(char *pObj, CELL_TABLE *pTable);
EXPORTS GGSTATUS upiGG_UnInitial(char **pObj);
EXPORTS void upiGG_DumpParameter(char *pObj, CELL_PARAMETER *pTable);

#define upiGG_PrePowerOff (upiGG_PreSuspend)

#ifdef ENABLE_BQ27520_SW_CMD

/**
 * @brief upiGG_AccessMeasurementParameter
 *
 *  Access measurement parameter
 *
 * @para  read  set _UPI_TRUE_ to read data from API
 * @para  pMeasPara pointer of GG_MEAS_PARA_TYPE
 * @return  GGSTATUS
 */
EXPORTS GGSTATUS upiGG_AccessMeasurementParameter(char *pObj, _upi_bool_ read, GG_MEAS_PARA_TYPE *pMeasPara);

#define UG_STD_CMD_CNTL     (0x00)
  #define UG_STD_CMD_CNTL_CONTROL_STATUS    (0x0000)
    #define UG_STD_CMD_CNTL_CONTROL_STATUS_DLOGEN     (1<<15)
    #define UG_STD_CMD_CNTL_CONTROL_STATUS_FAS        (1<<14)
    #define UG_STD_CMD_CNTL_CONTROL_STATUS_SS         (1<<13)
    #define UG_STD_CMD_CNTL_CONTROL_STATUS_CSV        (1<<12)
    #define UG_STD_CMD_CNTL_CONTROL_STATUS_CCA        (1<<11)
    #define UG_STD_CMD_CNTL_CONTROL_STATUS_BCA        (1<<10)
    #define UG_STD_CMD_CNTL_CONTROL_STATUS_OCVCMDCOMP (1<<9)
    #define UG_STD_CMD_CNTL_CONTROL_STATUS_OCVFAIL    (1<<8)
    #define UG_STD_CMD_CNTL_CONTROL_STATUS_INITCOMP   (1<<7)
    #define UG_STD_CMD_CNTL_CONTROL_STATUS_HIBERNATE  (1<<6)
    #define UG_STD_CMD_CNTL_CONTROL_STATUS_SNOOZE     (1<<5)
    #define UG_STD_CMD_CNTL_CONTROL_STATUS_SLEEP      (1<<4)
    #define UG_STD_CMD_CNTL_CONTROL_STATUS_LDMD       (1<<3)
    #define UG_STD_CMD_CNTL_CONTROL_STATUS_RUP_DIS    (1<<2)
    #define UG_STD_CMD_CNTL_CONTROL_STATUS_VOK        (1<<1)
    #define UG_STD_CMD_CNTL_CONTROL_STATUS_QEN        (1<<0)
  #define UG_STD_CMD_CNTL_DEVICE_TYPE       (0x0001)
  #define UG_STD_CMD_CNTL_FW_VERSION        (0x0002)
  #define UG_STD_CMD_CNTL_PREV_MACWRITE     (0x0007)
  #define UG_STD_CMD_CNTL_CHEM_ID           (0x0008)
  #define UG_STD_CMD_CNTL_OCV_CMD           (0x000C)      ///< [AT-PM] : Not implemented ; 10/11/2012
  #define UG_STD_CMD_CNTL_BAT_INSERT        (0x000D)
  #define UG_STD_CMD_CNTL_BAT_REMOVE        (0x000E)
  #define UG_STD_CMD_CNTL_SET_HIBERNATE     (0x0011)
  #define UG_STD_CMD_CNTL_CLEAR_HIBERNATE   (0x0012)
  #define UG_STD_CMD_CNTL_SET_SLEEP_PLUS    (0x0013)
  #define UG_STD_CMD_CNTL_CLEAR_SLEEP_PLUS  (0x0014)
  #define UG_STD_CMD_CNTL_FACTORY_RESTORE   (0x0015)
  #define UG_STD_CMD_CNTL_ENABLE_DLOG       (0x0018)
  #define UG_STD_CMD_CNTL_DISABLE_DLOG      (0x0019)
  #define UG_STD_CMD_CNTL_DF_VERSION        (0x001F)
  #define UG_STD_CMD_CNTL_SEALED            (0x0020)
  #define UG_STD_CMD_CNTL_RESET             (0x0041)
#define UG_STD_CMD_AR       (0x02)
#define UG_STD_CMD_ARTTE    (0x04)
#define UG_STD_CMD_TEMP     (0x06)
#define UG_STD_CMD_VOLT     (0x08)
#define UG_STD_CMD_FLAGS    (0x0A)
  #define UG_STD_CMD_FLAGS_OTC            (1<<15)
  #define UG_STD_CMD_FLAGS_OTD            (1<<14)
  #define UG_STD_CMD_FLAGS_RSVD13         (1<<13)
  #define UG_STD_CMD_FLAGS_RSVD12         (1<<12)
  #define UG_STD_CMD_FLAGS_CHG_INH        (1<<11)
  #define UG_STD_CMD_FLAGS_XCHG           (1<<10)
  #define UG_STD_CMD_FLAGS_FC             (1<<9)
  #define UG_STD_CMD_FLAGS_CHG            (1<<8)
  #define UG_STD_CMD_FLAGS_RSVD7          (1<<7)
  #define UG_STD_CMD_FLAGS_RSVD6          (1<<6)
  #define UG_STD_CMD_FLAGS_OCV_GD         (1<<5)
  #define UG_STD_CMD_FLAGS_WAIT_ID        (1<<4)
  #define UG_STD_CMD_FLAGS_BAT_DET        (1<<3)
  #define UG_STD_CMD_FLAGS_SOC1           (1<<2)
  #define UG_STD_CMD_FLAGS_SYSDOWN        (1<<1)
  #define UG_STD_CMD_FLAGS_DSG            (1<<0)
#define UG_STD_CMD_NAC      (0x0C)
#define UG_STD_CMD_FAC      (0x0E)
#define UG_STD_CMD_RM       (0x10)
#define UG_STD_CMD_FCC      (0x12)
#define UG_STD_CMD_AI       (0x14)
#define UG_STD_CMD_TTE      (0x16)
#define UG_STD_CMD_TTF      (0x18)
#define UG_STD_CMD_SI       (0x1A)
#define UG_STD_CMD_STTE     (0x1C)
#define UG_STD_CMD_MLI      (0x1E)
#define UG_STD_CMD_MLTTE    (0x20)
#define UG_STD_CMD_AE       (0x22)
#define UG_STD_CMD_AP       (0x24)
#define UG_STD_CMD_TTECP    (0x26)
#define UG_STD_CMD_SOH      (0x28)
  #define UG_STD_CMD_SOH_VALUE_MASK   (0x00FF)
  #define UG_STD_CMD_SOH_STATUS_MASK  (0xFF00)
    #define UG_STD_CMD_SOH_STATUS_NOT_VALID     (0x0000)
    #define UG_STD_CMD_SOH_STATUS_INSTANT_READY (0x0100)
    #define UG_STD_CMD_SOH_STATUS_INITIAL_READY (0x0200)
    #define UG_STD_CMD_SOH_STATUS_READY         (0x0300)
#define UG_STD_CMD_CC       (0x2A)
#define UG_STD_CMD_SOC      (0x2C)
#define UG_STD_CMD_NIC      (0x2E)      ///< [AT-PM] : Not implemented ; 10/11/2012
#define UG_STD_CMD_ICR      (0x30)
#define UG_STD_CMD_DLI      (0x32)
#define UG_STD_CMD_DLB      (0x34)
#define UG_STD_CMD_ITEMP    (0x36)
#define UG_STD_CMD_OPCFG    (0x3A)
  #define UG_STD_CMD_OPCFG_RESCAP     (1<<31)
  #define UG_STD_CMD_OPCFG_BATG_OVR   (1<<30)
  #define UG_STD_CMD_OPCFG_INT_BERM   (1<<29)
  #define UG_STD_CMD_OPCFG_PFC_CFG1   (1<<28)
  #define UG_STD_CMD_OPCFG_PFC_CFG0   (1<<27)
  #define UG_STD_CMD_OPCFG_IWAKE      (1<<26)
  #define UG_STD_CMD_OPCFG_RSNS1      (1<<25)
  #define UG_STD_CMD_OPCFG_RSNS0      (1<<24)
  #define UG_STD_CMD_OPCFG_INT_FOCV   (1<<23)
  #define UG_STD_CMD_OPCFG_IDSELEN    (1<<22)
  #define UG_STD_CMD_OPCFG_SLEEP      (1<<21)
  #define UG_STD_CMD_OPCFG_RMFCC      (1<<20)
  #define UG_STD_CMD_OPCFG_SOCI_POL   (1<<19)
  #define UG_STD_CMD_OPCFG_BATG_POL   (1<<18)
  #define UG_STD_CMD_OPCFG_BATL_POL   (1<<17)
  #define UG_STD_CMD_OPCFG_TEMPS      (1<<16)
  #define UG_STD_CMD_OPCFG_WRTEMP     (1<<15)
  #define UG_STD_CMD_OPCFG_BIE        (1<<14)
  #define UG_STD_CMD_OPCFG_BL_INT     (1<<13)
  #define UG_STD_CMD_OPCFG_GNDSEL     (1<<12)
  #define UG_STD_CMD_OPCFG_FCE        (1<<11)
  #define UG_STD_CMD_OPCFG_DFWRINDBL  (1<<10)
  #define UG_STD_CMD_OPCFG_RFACTSTEP  (1<<9)
  #define UG_STD_CMD_OPCFG_INDFACRES  (1<<8)
  #define UG_STD_CMD_OPCFG_BATGSPUEN  (1<<7)
  #define UG_STD_CMD_OPCFG_BATGWPUEN  (1<<6)
  #define UG_STD_CMD_OPCFG_BATLSPUEN  (1<<5)
  #define UG_STD_CMD_OPCFG_BATLWSPUEN (1<<4)
  #define UG_STD_CMD_OPCFG_RSVD3      (1<<3)
  #define UG_STD_CMD_OPCFG_SLPWKCHG   (1<<2)
  #define UG_STD_CMD_OPCFG_DELTAVOPT1 (1<<1)
  #define UG_STD_CMD_OPCFG_DELTAVOPT0 (1<<0)
#define UG_EXT_CMD_DCAP     (0x3C)

/**
 * @brief upiGG_FetchDataCommand
 *
 *  Fetch bq27520 like command
 *
 * @para  read  set _UPI_TRUE_ to read data from API
 * @para  pMeasPara pointer of GG_MEAS_PARA_TYPE
 * @return  GGSTATUS
 */
EXPORTS GGSTATUS upiGG_FetchDataCommand(char *pObj, _upi_u8_ CommandCode, _upi_u16_ *pData);

typedef struct GG_FETCH_DATA_PARA_ST {
  _upi_s8_ FCSet;
  _upi_s8_ FCClear;
  _upi_u8_ Soc1Set;
  _upi_u8_ Soc1Clear;
  _upi_s8_ InitSI;
  _upi_s16_ InitMaxLoadCurrent;
  _upi_u16_ CCThreshold;
  _upi_u32_ Opcfg;
  _upi_u16_ Dcap;
} ALIGNED_ATTRIBUTE GG_FETCH_DATA_PARA_TYPE;

/**
 * @brief upiGG_FetchDataParameter
 *
 *  Set the parameter for bq27520 like command
 *
 * @para  data  parameters of GG_FETCH_DATA_PARA_TYPE
 * @return  GGSTATUS
 */
EXPORTS GGSTATUS upiGG_FetchDataParameter(char *pObj, GG_FETCH_DATA_PARA_TYPE data);

#endif     //endif ENABLE_BQ27520_SW_CMD

typedef struct GG_FETCH_DEBUG_DATA_ST {
  /// [AT-PM] : Capacity related ; 01/30/2013
  int capStatus;
  int capSelfHour;
  int capSelfMin;
  int capSelfSec;
  int capSelfMilliSec;
  int capTPTime;
  int capDelta;
  int capDsgCharge;
  int capDsgChargeStart;
  int capDsgChargeTime;
  int capPreDsgCharge;
  int capTableUpdateIdx;

  /// [AT-PM] : Measurement related ; 01/30/2013
  int measCodeBat1;
  int measCodeCurrent;
  int measCodeIT;
  int measCodeET;
  int measCharge;
  int measCCOffset;
  int measAdc1ConvertTime;
  int measAdc1Gain;
  int measAdc1Offset;
  int measAdc2Gain;
  int measAdc2Offset;
  int measLastCounter;
  int measLastTimeTick;
  int measLastDeltaQ;
  int measAdc1CodeT25V100;
  int measAdc1CodeT25V200;
  int measAdc2CodeT25V100;
  int measAdc2CodeT25V200;
  int measCodeBat1BeforeCal;
  int measCodeCurrentBeforeCal;
  int measCodeITBeforeCal;
  int measCodeETBeforeCal;
  
  int sysOscFrequency;
  
} ALIGNED_ATTRIBUTE GG_FETCH_DEBUG_DATA_TYPE;

/**
 * @brief upiGG_FetchDebugData
 *
 *  Fetch debug information data
 *
 * @para  pObj  address of memory buffer
 * @para  data  address of GG_FETCH_CAP_DATA_TYPE
 * @return  _UPI_NULL_
 */
EXPORTS void upiGG_FetchDebugData(char *pObj, GG_FETCH_DEBUG_DATA_TYPE *data);

/**
 * @brief upiGG_DebugSwitch
 *
 *  Enable/disable debug information to UART
 *
 * @para  Enable  set debug level
 * @return  NULL
 */
EXPORTS void upiGG_DebugSwitch(_upi_u8_ enable);

/**
 * @brief upiGG_BackupFileSwitch
 *
 *  Enable/disable backup file operation
 *
 * @para  Enable  set _UPI_TRUE_ to enable it
 * @return  NULL
 */
EXPORTS void upiGG_BackupFileSwitch(_upi_bool_ enable);

/**
 * @brief upiGG_BackupFileCheck
 *
 *  Backup file check procedure
 *
 * @para  pObj  address of memory buffer
 * @para  BackupFileName  backup filename
 * @para  SuspendFileName suspend filename
 * @return  UG_CAP_DATA_STATUS
 */
#if defined (uG31xx_OS_WINDOWS)
  
    EXPORTS _upi_u8_ upiGG_BackupFileCheck(char *pObj, const wchar_t* BackupFileName, const wchar_t *SuspendFileName);
  
#else
  
    _upi_u8_ upiGG_BackupFileCheck(char *pObj, char *BackupFileName, char *SuspendFileName);
  
#endif

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
EXPORTS void upiGG_InternalSuspendMode(char *pObj, _upi_bool_ inSuspend);

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

  EXPORTS GGSTATUS upiGG_Reset(char *pObj, const wchar_t* GGBFilename, const wchar_t* OtpFileName);

#else

  GGSTATUS upiGG_Reset(char *pObj, GGBX_FILE_HEADER *pGGBXBuf);

#endif

#if defined (uG31xx_OS_WINDOWS)

  EXPORTS void upiGG_BackupMemory(char *pObj, const wchar_t* BackupFileName);
  EXPORTS void upiGG_RecoveryMemory(char *pObj, const wchar_t* BackupFileName);
  
#endif  ///< end of defined (uG31xx_OS_WINDOWS)

#ifndef uG31xx_BOOT_LOADER

EXPORTS void upiGG_SetBatteryET(char *pObj);
EXPORTS void upiGG_SetBatteryIT(char *pObj);

#endif  ///< end of uG31xx_BOOT_LOADER

EXPORTS int upiGG_GetCycleCount(char *pObj);
EXPORTS int upiGG_SetCycleCount(char *pObj, _upi_u16_ value);

EXPORTS void upiGG_ReverseCurrent(char *pObj, _upi_bool_ reverse);

EXPORTS void upiGG_AdjustCellTable(char *pObj, _upi_u16_ designCap);

EXPORTS GGSTATUS upiGG_GetNtcStatus(char *pObj);

EXPORTS _upi_u8_ * upiGG_AccessBackupBuffer(char *pObj, _upi_u8_ *size);

EXPORTS GGSTATUS upiGG_FetchCurrent(char *pObj);
EXPORTS GGSTATUS upiGG_FetchVoltage(char *pObj);
EXPORTS GGSTATUS upiGG_FetchInternalTemperature(char *pObj);
EXPORTS GGSTATUS upiGG_FetchExternalTemperature(char *pObj);

#define GET_BOARD_OFFSET_FULL     (1)
#define GET_BOARD_OFFSET_AVG      (-1)
#define GET_BOARD_OFFSET_STEP     (0)

EXPORTS void upiGG_GetBoardOffset(char *pObj, _upi_s16_ fullStep, _upi_s16_ upper, _upi_s16_ lower);

EXPORTS void upiGG_SetCapacitySuspendMode(char *pObj, _upi_bool_ inSuspend);
EXPORTS void upiGG_SetCapacity(char *pObj, _upi_u8_ target);

#if defined (uG31xx_OS_WINDOWS)
EXPORTS GGSTATUS upiGG_MpkActiveGG(char **pObj,const wchar_t* GGBFilename,const wchar_t* OtpFileName, _upi_u16_ i2cAddress);
#endif  ///< end of uG31xx_OS_WINDOWS

EXPORTS void upiGG_ShellUpdateCapacity(char *pObj);
EXPORTS void upiGG_ShellUpdateCC(char *pObj);

#endif  ///< end of _UG31XXAPI_H_

/// ===========================================
/// End of uG31xx_API.h
/// ===========================================

