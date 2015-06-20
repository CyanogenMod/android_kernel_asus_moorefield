/**
 * @filename  uG31xx_API_Measurement.h
 *
 *  Header for uG31xx measurement API
 *
 * @author  AllenTeng <allen_teng@upi-semi.com>
 * @revision  $Revision: 476 $
 */

typedef signed char       _meas_s8_;
typedef unsigned char     _meas_u8_;
typedef signed short      _meas_s16_;
typedef unsigned short    _meas_u16_;
typedef signed long       _meas_s32_;
typedef unsigned long     _meas_u32_;
typedef signed long long  _meas_s64_;
typedef char              _meas_bool_;

#define CALIBRATION_FACTOR_CONST    (1000)

#define TIME_DEFAULT_ADC1_CONVERT_TIME        (1253)

#define MEAS_STATUS_IN_SUSPEND_MODE           (1<<0)
#define MEAS_STATUS_REVERSE_CURRENT_DIRECTION (1<<1)
#define MEAS_STATUS_NTC_OPEN                  (1<<2)
#define MEAS_STATUS_NTC_SHORT                 (1<<3)
#define MEAS_STATUS_REFER_ET                  (1<<4)
#define MEAS_STATUS_CABLE_OUT                 (1<<5)

#define MEAS_IN_SUSPEND_MODE(x)           ((x & MEAS_STATUS_IN_SUSPEND_MODE) ? _UPI_TRUE_ : _UPI_FALSE_)
#define MEAS_REVERSE_CURRENT_DIRECTION(x) ((x & MEAS_STATUS_REVERSE_CURRENT_DIRECTION) ? _UPI_TRUE_ : _UPI_FALSE_)
#define MEAS_NTC_OPEN(x)                  ((x & MEAS_STATUS_NTC_OPEN) ? _UPI_TRUE_ : _UPI_FALSE_)
#define MEAS_NTC_SHORT(x)                 ((x & MEAS_STATUS_NTC_SHORT) ? _UPI_TRUE_ : _UPI_FALSE_)
#define MEAS_CABLE_OUT(x)                 ((x & MEAS_STATUS_CABLE_OUT) ? _UPI_TRUE_ : _UPI_FALSE_)

#define MEAS_MAXIMUM_INITIAL_RETRY_CNT      (40)
#define MEAS_MAXIMUM_ROUTINE_RETRY_CNT      (3)
/// [FC] : Add variable MPK_active for MPK intial ; 12/10/2013
extern _upi_bool_ MPK_active;

typedef enum _MEAS_RTN_CODE {
  MEAS_RTN_PASS = 0,
  MEAS_RTN_BATTERY_REMOVED,
  MEAS_RTN_ADC_ABNORMAL,
  MEAS_RTN_NTC_SHORT,
} MEAS_RTN_CODE;

typedef enum _MEAS_SEL_CODE {
  MEAS_SEL_ALL = 0,
  MEAS_SEL_VOLTAGE,
  MEAS_SEL_CURRENT,
  MEAS_SEL_EXT_TEMP,
  MEAS_SEL_INT_TEMP,
  MEAS_SEL_INITIAL,
} MEAS_SEL_CODE;

typedef struct MeasDataST {

  /// [AT-PM] : System data ; 04/08/2013
  SystemDataType *sysData;
  
  /// [AT-PM] : OTP data ; 01/23/2013
  OtpDataType *otp;
  
  /// [AT-PM] : Physical value ; 01/23/2013
  _meas_u16_ bat1Voltage;
  _meas_u16_ bat1VoltageAvg;
  _meas_s16_ curr;
  _meas_s16_ currAvg;
  _meas_s16_ intTemperature;
  _meas_s16_ extTemperature;
  _meas_s16_ instExtTemperature;
  _meas_s16_ deltaCap;
  _meas_s16_ stepCap;
  _meas_s32_ cumuCap;
  _meas_u32_ deltaTime;
  _meas_u32_ deltaTimeDaemon;

  /// [AT-PM] : ADC code ; 01/23/2013
  _meas_u16_ codeBat1;
  _meas_s16_ codeCurrent;
  _meas_u16_ codeIntTemperature;
  _meas_u16_ codeExtTemperature;
  _meas_u16_ codeInstExtTemperature;
  _meas_s32_ codeCharge;

  /// [AT-PM] : Coulomb counter offset ; 01/23/2013
  _meas_s16_ ccOffset;
  _meas_s8_ ccOffsetAdj;
  
  /// [AT-PM] : ADC1 characteristic ; 01/23/2013
  _meas_u16_ adc1ConvertTime;
  _meas_s32_ adc1Gain;
  _meas_s32_ adc1GainSlope;
  _meas_s32_ adc1GainFactorB;
  _meas_s32_ adc1Offset;
  _meas_s32_ adc1OffsetSlope;
  _meas_s32_ adc1OffsetFactorO;

  /// [AT-PM] : ADC2 characteristic ; 01/23/2013
  _meas_s32_ adc2Gain;
  _meas_s32_ adc2GainSlope;
  _meas_s32_ adc2GainFactorB;
  _meas_s32_ adc2Offset;
  _meas_s32_ adc2OffsetSlope;
  _meas_s32_ adc2OffsetFactorO;

  /// [AT-PM] : Previous information ; 01/25/2013
  _meas_u16_ lastCounter;
  _meas_u32_ lastTimeTick;
  _meas_s16_ lastDeltaCap;

  /// [FC] : Record ADC code ; 05/15/2013
  _meas_s16_ adc1CodeT25V100;
  _meas_s16_ adc1CodeT25V200;
  _meas_s16_ adc2CodeT25V100;
  _meas_s16_ adc2CodeT25V200;
  _meas_u16_ codeBat1BeforeCal;
  _meas_s16_ codeCurrentBeforeCal;
  _meas_u16_ codeIntTemperatureBeforeCal;
  _meas_u16_ codeExtTemperatureBeforeCal;
  _meas_s16_ codeChargeBeforeCal;
  _meas_s16_ codeCCOffset;

  /// [AT-PM] : Operation control ; 11/12/2013
  _meas_u32_ status;
  _meas_u8_ fetchRetryCnt;

  /// [AT-PM] : Cycle count operation ; 10/08/2013
  _meas_s32_ cycleCountBuf;
  _meas_u16_ cycleCount;
} ALIGNED_ATTRIBUTE MeasDataType;

/**
 * @brief UpiResetCoulombCounter
 *
 *  Reset coulomb counter
 *
 * @para  data  address of MeasDataType
 * @return  _UPI_NULL_
 */
extern void UpiResetCoulombCounter(MeasDataType *data);

/**
 * @brief UpiMeasurement
 *
 *  Measurement routine
 *
 * @para  data  address of MeasDataType
 * @para  select  MEAS_SEL_CODE
 * @return  MEAS_RTN_CODE
 */
extern MEAS_RTN_CODE UpiMeasurement(MeasDataType *data, MEAS_SEL_CODE select);

/**
 * @brief UpiMeasAlarmThreshold
 *
 *  Get alarm threshold
 *
 * @para  data  address of MeasDataType
 * @return  MEAS_RTN_CODE
 */
extern MEAS_RTN_CODE UpiMeasAlarmThreshold(MeasDataType *data);

/**
 * @brief UpiMeasReadCode
 *
 *  Read ADC code
 *
 * @para  data  address of MeasDataType
 * @return  MEAS_RTN_CODE
 */
extern MEAS_RTN_CODE UpiMeasReadCode(MeasDataType *data);

/**
 * @brief UpiGetMeasurementMemorySize
 *
 *  Get memory size used by measurement
 *
 * @return  memory size
 */
extern _meas_u32_ UpiGetMeasurementMemorySize(void);

