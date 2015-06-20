/**
 * @filename  uG31xx_API_Otp.h
 *
 *  Header of OTP conversion module
 *
 * @author  AllenTeng <allen_teng@upi-semi.com>
 * @revision  $Revision: 2 $
 */

/// [AT-PM] : Product Type definition in OTP ; 01/23/2013
enum UG31XX_PRODUCT_TYPE {
  UG31XX_PRODUCT_TYPE_0 = 0,
  UG31XX_PRODUCT_TYPE_1 = 1,
  UG31XX_PRODUCT_TYPE_2 = 2,
  UG31XX_PRODUCT_TYPE_3 = 3,
};

#define UG31XX_OTP_VERSION_MAIN (0x2013)
#define UG31XX_OTP_VERSION_SUB  (0x0110)

#define OTP_IS_EMPTY      (1)
#define OTP_IS_NOT_EMPTY  (0)

#define OTP1_SIZE         (4)
#define OTP2_SIZE         (16)
#define OTP3_SIZE         (4)

typedef unsigned char   _otp_u8_;
typedef signed char     _otp_s8_;
typedef unsigned short  _otp_u16_;
typedef signed short    _otp_s16_;

typedef struct OtpDataST {

  /// [AT-PM] : Version ; 01/23/2013
  _otp_u16_ versionMain;
  _otp_u16_ versionSub;
  _otp_u8_ empty;
  
  /// [AT-PM] : Raw data ; 01/23/2013
  _otp_u8_ otp1[OTP1_SIZE];
  _otp_u8_ otp2[OTP2_SIZE];
  _otp_u8_ otp3[OTP3_SIZE];

  /// [AT-PM] : Converted value ; 01/23/2013
  _otp_u16_ adc1DeltaCodeT25V100;
  _otp_u16_ adc1DeltaCodeT25V200;
  _otp_u16_ adc2DeltaCodeT25V100;
  _otp_u16_ adc2DeltaCodeT25V200;
  _otp_u16_ aveIT25;
  _otp_u16_ aveIT80;

  _otp_u8_ bgrTune;
  
  _otp_u8_ deltaET;
  _otp_u8_ deltaVref;
  _otp_u16_ devAddr;

  _otp_u16_ ftIT;
  
  _otp_u8_ indexAdc1V100T25;
  _otp_u8_ indexAdc1V200T25;
  _otp_u8_ indexAdc2V100T25;
  _otp_u8_ indexAdc2V200T25;

  _otp_s8_ oscDeltaCode25;
  _otp_s8_ oscDeltaCode80;
  _otp_u8_ otpCellEN;
  
  _otp_u8_ productType;
  
} ALIGNED_ATTRIBUTE OtpDataType;

/**
 * @brief UpiConvertOtp
 *
 *  Convert OTP register value to readable value
 *
 * @para  data  address of OtpDataType
 * @return  _UPI_NULL_
 */
extern void UpiConvertOtp(OtpDataType *data);

