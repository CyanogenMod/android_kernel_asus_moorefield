/// ===========================================
/// uG31xx.h
/// ===========================================

#ifndef _UG31XX_H_
#define _UG31XX_H_

#define  SECURITY			1		//Security Mode enable
#define  NORMAL				0		//Security Mode OFF

#define  HIGH_SPEED			1		//HS Mode
#define  FULL_SPEED			0		//FIL speed

#define  TEN_BIT_ADDR		1		//10-bit address Mode
#define	 SEVEN_BIT_ADDR		0		//7-bit address Mode

#define  I2C_SUCESS			1		//
#define  I2C_FAIL			0

/// ===========================================================================
/// Constant for Calibration
/// ===========================================================================

#define IT_TARGET_CODE25	  (12155)
#define IT_TARGET_CODE80	  (14306)

#define IT_CODE_25		(23171)
#define IT_CODE_80		(27341)

//constant
//define IC type
#define uG3100		0
#define uG3101		1
#define uG3102		2
#define uG3103_2	4
#define uG3103_3	5

//constant
//GPIO1/2 define
#define FUN_GPIO				0x01
#define FUN_ALARM				0x02
#define FUN_PWM					0x04
#define FUN_CBC_EN21			0x08
#define FUN_CBC_EN32			0x10

#define BIT_MACRO(x)		((_upi_u8_)1 << (x))

#define MAX_CRATE_AVAILABLE     (20)
					
#define I2C_ADDRESS    0x70
#define I2C_CLOCK      0x100

//const for CELL_TABLE table
#define TEMPERATURE_NUMS  (4)
#define C_RATE_NUMS				(3)     ///< [AT-PM] : 0.5, 0.2, 0.1, 0.02 ; 12/17/2013
#define OCV_NUMS				  (21)			//include the 0% & 100%
#define SOV_NUMS				  (16)     ///< [FC] : 100, 95, 90, 80, 70, 60, 50, 40, 35, 30, 25, 20, 15, 10, 5, 0 ; 06/14/2013
#define ET_NUMS           (19)      ///< [AT-PM] : -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80 ; 01/25/2013

/// [AT-PM] : Suggested value of NacLmdAdjustCfg is 0x2001 for uG3105 ; 08/07/2013
/// [AT-PM] : Suggested value of NacLmdAdjustCfg is 0x2041 for uG310x ; 08/07/2013
#define NAC_LMD_ADJUST_CFG_NO_LMD_UPDATE_LIMIT                (3<<0)
  #define NAC_LMD_ADJUST_CFG_NO_LMD_UPDATE_NO_LIMIT           (0<<0)
  #define NAC_LMD_ADJUST_CFG_NO_LMD_UPDATE_BETWEEN_10_90_EN   (1<<0)
  #define NAC_LMD_ADJUST_CFG_NO_LMD_UPDATE_ABOVE_10_EN        (2<<0)
#define NAC_LMD_ADJUST_CFG_PARSER_FORCE_0_BELOW_EDVF_DISABLE  (1<<2)
#define NAC_LMD_ADJUST_CFG_PARSER_LOCK_1_BEFORE_EDVF_DISABLE  (1<<3)
#define NAC_LMD_ADJUST_CFG_PARSER_FORCE_RSOC_STEP_1_DISABLE   (1<<4)
#define NAC_LMD_ADJUST_CFG_DISPLAY_CC_AS_FCC                  (1<<5)
#define NAC_LMD_ADJUST_CFG_PRODUCT_TYPE                       (3<<6)
  #define NAC_LMD_ADJUST_CFG_PRODUCT_TYPE_UG3105              (0<<6)
  #define NAC_LMD_ADJUST_CFG_PRODUCT_TYPE_UG310X              (1<<6)
  #define NAC_LMD_ADJUST_CFG_PRODUCT_TYPE_RSVD2               (2<<6)
  #define NAC_LMD_ADJUST_CFG_PRODUCT_TYPE_RSVD3               (3<<6)
#define NAC_LMD_ADJUST_CFG_PARSER_FORCE_100_REACH_TP_DISABLE  (1<<8)
#define NAC_LMD_ADJUST_CFG_PARSER_FORCE_99_BEFORE_TP_DISABLE  (1<<9)
#define NAC_LMD_ADJUST_CFG_PARSER_RM_NOT_EXCEED_FCC_DISABLE   (1<<10)
#define NAC_LMD_ADJUST_CFG_CHG_FCC_SELECTION                  (3<<11)
  #define NAC_LMD_ADJUST_CFG_CHG_FCC_WITH_CURR_FCC            (0<<11)
  #define NAC_LMD_ADJUST_CFG_CHG_FCC_WITH_CC_RECORD           (1<<11)
  #define NAC_LMD_ADJUST_CFG_CHG_FCC_WITH_NAC_TABLE           (2<<11)   
  #define NAC_LMD_ADJUST_CFG_CHG_FCC_WITH_DSG_CHARGE          (3<<11)
#define NAC_LMD_ADJUST_CFG_ALWAYS_UPDATE_FCC_AT_LAST          (1<<13)
#define NAC_LMD_ADJUST_CFG_PARSER_RATIO_DISABLE               (1<<14)
#define NAC_LMD_ADJUST_CFG_CAP_ALGORITHM_VER                  (3<<15)
  #define NAC_LMD_ADJUST_CFG_CAP_ALGORITHM_ORIGINAL           (0<<15)
  #define NAC_LMD_ADJUST_CFG_CAP_ALGORITHM_NO_LEARNING        (1<<15)
  #define NAC_LMD_ADJUST_CFG_CAP_ALGORITHM_FIX_EDV            (2<<15)
#define NAC_LMD_ADJUST_CFG_NO_MAX_FCC_LIMIT_WITH_TABLE        (1<<17)
#define NAC_LMD_ADJUST_CFG_COUNT_STANDBY_CURRENT              (1<<18)
#define NAC_LMD_ADJUST_CFG_REMOVE_INIT_PARSER                 (1<<19)
#define NAC_LMD_ADJUST_CFG_LIMIT_FCC_CHANGE_RANGE_10          (1<<20)
#define NAC_LMD_ADJUST_CFG_VOLTAGE_CC_WEIGHT                  (3<<21)
  #define NAC_LMD_ADJUST_CFG_VOLTAGE_CC_WEIGHT_1              (0<<21)
  #define NAC_LMD_ADJUST_CFG_VOLTAGE_CC_WEIGHT_2              (1<<21)
  #define NAC_LMD_ADJUST_CFG_VOLTAGE_CC_WEIGHT_3              (2<<21)
  #define NAC_LMD_ADJUST_CFG_VOLTAGE_CC_WEIGHT_4              (3<<21)
#define NAC_LMD_ADJUST_CFG_OCV_REFER_NAC_TABLE_EN             (1<<23)
#define NAC_LMD_ADJUST_CFG_STANDBY_DYNAMIC_OCV_WEIGHT_EN      (1<<24)
#define NAC_LMD_ADJUST_CFG_NO_STANDBY_CAP_EST_EN              (1<<25)
#define NAC_LMD_ADJUST_CFG_RSOC_FILTER_RSOC_TABLE             (1<<26)
#define NAC_LMD_ADJUST_CFG_RSOC_FILTER_LOCK_TABLE             (1<<27)
#define NAC_LMD_ADJUST_CFG_BATTERY_REINSERT_DETECT_EN         (1<<28)

#define GET_PRODUCT_TYPE(x)       ((x & NAC_LMD_ADJUST_CFG_PRODUCT_TYPE) >> 6)
#define GET_CAP_ALGORITHM_VER(x)  ((x & NAC_LMD_ADJUST_CFG_CAP_ALGORITHM_VER) >> 15)

enum C_RATE_TABLE_VALUES {
  C_RATE_TABLE_VALUE_0 = 50,
  C_RATE_TABLE_VALUE_1 = 20,
  C_RATE_TABLE_VALUE_2 = 10,
  C_RATE_TABLE_VALUE_3 = 2
};

enum OCV_TABLE_IDX {
  OCV_TABLE_IDX_CHARGE = 0,
  OCV_TABLE_IDX_STAND_ALONE,
  OCV_TABLE_IDX_100MA,
  OCV_TABLE_IDX_COUNT,
};

#define CONST_PERCENTAGE                  (100)
#define CONST_ROUNDING                    (10)
#define CONST_ROUNDING_5                  (5)
#define TIME_CONVERT_TIME_TO_MSEC         (10)
#define CONST_CONVERSION_COUNT_THRESHOLD  (300)
#define TIME_SEC_TO_HOUR                  (3600)
#define TIME_MSEC_TO_SEC                  (1000)

#define ET_AVERAGE_NEW      (1)
#define ET_AVERAGE_OLD      (7)
#define ET_AVERAGE_BASE     (ET_AVERAGE_NEW + ET_AVERAGE_OLD)

#endif

#define ENCRIPT_TABLE_SIZE (39)

#define _LKM_OPTIONS_
#define LKM_OPTIONS_FORCE_RESET             (1<<0)
#define LKM_OPTIONS_ENABLE_SUSPEND_DATA_LOG (1<<1)
#define LKM_OPTIONS_ENABLE_DEBUG_LOG        (3<<2)
  #define LKM_OPTIONS_DEBUG_ERROR           (0<<2)
  #define LKM_OPTIONS_DEBUG_INFO            (1<<2)
  #define LKM_OPTIONS_DEBUG_NOTICE          (2<<2)
  #define LKM_OPTIONS_DEBUG_DEBUG           (3<<2)
#define LKM_OPTIONS_ENABLE_REVERSE_CURRENT  (1<<4)
#define LKM_OPTIONS_ADJUST_DESIGN_CAPACITY  (1<<5)
#define LKM_OPTIONS_DISABLE_BACHUP_FILE     (1<<6)
#define LKM_OPTIONS_RSOC_REMAP              (1<<7)

#define LKM_OPTIONS_DEBUG_LEVEL(x)          ((x & LKM_OPTIONS_ENABLE_DEBUG_LOG) >> 2)

/// ===========================================
/// End of uG31xx.h
/// ===========================================

