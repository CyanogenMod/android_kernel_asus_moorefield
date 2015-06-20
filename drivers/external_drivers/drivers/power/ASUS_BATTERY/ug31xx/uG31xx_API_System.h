/**
 * @filename  uG31xx_API_System.h
 *
 *  Interface of ug31xx system control
 *
 * @author  AllenTeng <allen_teng@upi-semi.com>
 * @revision  $Revision: 19 $
 */

#define UG31XX_SYSTEM_VERSION     (7)

typedef unsigned char   _sys_u8_;
typedef signed char     _sys_s8_;
typedef unsigned short  _sys_u16_;
typedef signed short    _sys_s16_;
typedef unsigned long   _sys_u32_;
typedef signed long     _sys_s32_;
typedef char            _sys_bool_;

typedef enum _SYSTEM_RTN_CODE {
  SYSTEM_RTN_PASS = 0,
  SYSTEM_RTN_READ_GGB_FAIL,
  SYSTEM_RTN_I2C_FAIL,
} SYSTEM_RTN_CODE;

typedef struct AlarmDataST {
  _sys_u16_ alarmThrd;
  _sys_u16_ releaseThrd;
  _sys_bool_ state;
} ALIGNED_ATTRIBUTE AlarmDataType;

typedef struct SystemDataST {

  #if defined (uG31xx_OS_WINDOWS)
    const wchar_t* ggbFilename;
    const wchar_t* otpFileName;
  #else   ///< else of defined (uG31xx_OS_WINDOWS)
    GGBX_FILE_HEADER *ggbXBuf;
  #endif  ///< end of defined (uG31xx_OS_WINDOWS)
  
  CELL_PARAMETER *ggbParameter;
  CELL_TABLE *ggbCellTable;
  OtpDataType *otpData;
  
  ADC_CHECK adcCheckData;			//add for adc error check 20121025/jacky

  _sys_u16_ voltage;
  _sys_s16_ curr;
  _sys_u8_ tpCount;
  
  _sys_u16_ preITAve;
  _sys_u8_ cellNum;

  _sys_u16_ rmFromIC;
  _sys_u16_ fccFromIC;
  _sys_u8_ rsocFromIC;
  _sys_u32_ timeTagFromIC;
  _sys_u8_ tableUpdateIdxFromIC;
  _sys_s16_ deltaCapFromIC;
  _sys_u16_ adc1ConvTime;
  _sys_u32_ oscFreq;
  _sys_u16_ cycleCount;
  _sys_s8_ ccOffset;
  _sys_u8_ predictRsoc;
  _sys_u8_ standbyDsgRatio;
  
  _sys_u16_ rmFromICBackup;
  _sys_u16_ fccFromICBackup;
  _sys_u8_ rsocFromICBackup;

  AlarmDataType uvAlarm;
  AlarmDataType oetAlarm;
  AlarmDataType uetAlarm;
  _sys_u16_ alarmSts;
} ALIGNED_ATTRIBUTE SystemDataType;

/**
 * @brief UpiInitSystemData
 *
 *  Initialize system data
 *
 * @para  data  address of BootDataType
 * @return  _UPI_NULL_
 */
extern SYSTEM_RTN_CODE UpiInitSystemData(SystemDataType *data);

/**
 * @brief UpiCheckICActive
 *
 *  Check IC is actived or not
 *
 * @return  _UPI_TRUE_ if uG31xx is not actived
 */
extern _upi_bool_ UpiCheckICActive(void);

/**
 * @brief UpiActiveUg31xx
 *
 *  Active uG31xx
 *
 * @return  SYSTEM_RTN_CODE
 */
extern SYSTEM_RTN_CODE UpiActiveUg31xx(void);

/**
 * @brief UpiSetupAdc
 *
 *  Setup ADC configurations
 *
 * @para  data  address of SystemDataType
 * @return  _UPI_NULL_
 */
extern void UpiSetupAdc(SystemDataType *data);

/**
 * @brief UpiDecimateRst
 *
 *  Decimate reset filter of ADC
 *
 * @return  _UPI_NULL_
 */
extern void UpiDecimateRst(void);

/**
 * @brief UpiSetupSystem
 *
 *  Setup uG31xx system
 *
 * @para  data  address of SystemDataType
 * @return  _UPI_NULL_
 */
extern void UpiSetupSystem(SystemDataType *data);

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
extern void UpiCalculateOscFreq(SystemDataType *data);

/**
 * @brief UpiAdcStatus
 *
 *  Check ADC status
 *
 * @para  data  address of SystemDataType
 * @return  _UPI_NULL_
 */
extern void UpiAdcStatus(SystemDataType *data);

/**
 * @brief UpiLoadBatInfoFromIC
 *
 *  Load battery information from uG31xx
 *
 * @para  data  address of SystemDataType
 * @return  _UPI_NULL_
 */
extern void UpiLoadBatInfoFromIC(SystemDataType *data);

/**
 * @brief UpiUpdateBatInfoFromIC
 *
 *  Update battery information from uG31xx
 *
 * @para  data  address of SystemDataType
 * @para  deltaQ  delta capacity from coulomb counter
 * @return  _UPI_NULL_
 */
extern void UpiUpdateBatInfoFromIC(SystemDataType *data, _sys_s16_ deltaQ);

/**
 * @brief UpiSaveBatInfoTOIC
 *
 *  Save battery information from uG31xx
 *
 * @para  data  address of SystemDataType
 * @return  _UPI_NULL_
 */
extern void UpiSaveBatInfoTOIC(SystemDataType *data);

/**
 * @brief UpiInitAlarm
 *
 *  Initialize alarm function of uG3105
 *
 * @para  data  address of SystemDataType
 * @return  NULL
 */
extern void UpiInitAlarm(SystemDataType *data);

/**
 * @brief UpiAlarmStatus
 *
 *  Get alarm status
 *
 * @para  data  address of SystemDataType
 * @return  NULL
 */
extern _sys_u8_ UpiAlarmStatus(SystemDataType *data);

/**
 * @brief UpiLoadTableFromIC
 *
 *  Load table form IC
 *
 * @para  data point of _sys_u8_ type
 * @return  NULL
 */
extern void UpiLoadTableFromIC(_sys_u8_ *data);

/**
 * @brief UpiLoadTableFromIC
 *
 *  Save table to IC
 *
 * @para  data point of _sys_u8_ type
 * @return  NULL
 */
extern void UpiSaveTableToIC(_sys_u8_ *data, _sys_u8_ *buf, _sys_u8_ size);

/**
 * @brief UpiSetupAdc1Queue
 *
 *  Setup ADC1 queue for normal operation
 *
 * @para  data  address of SystemDataType structure
 * @return  NULL
 */
extern void UpiSetupAdc1Queue(SystemDataType *data);

/**
 * @brief UpiAllocateTableBuf
 *
 *  Allocate table buffer
 *
 * @para  data  address of SystemDataType structure
 * @return  NULL
 */
extern void UpiAllocateTableBuf(_sys_u8_ **data, _sys_u8_ *size);

/**
 * @brief UpiFreeTableBuf
 *
 *  Free table buffer
 *
 * @para  data  address of SystemDataType structure
 * @return  NULL
 */
extern void UpiFreeTableBuf(_sys_u8_ **data);

