/**
 * @filename  uG31xx_API_Platform.h
 *
 *  Header for function required by uG31xx driver
 *
 * @author  AllenTeng <allen_teng@upi-semi.com>
 * @note
 */

typedef unsigned char       _upi_u8_;
typedef unsigned short      _upi_u16_;
typedef unsigned int        _upi_u32_;
typedef unsigned long long  _upi_u64_;
typedef char                _upi_s8_;
typedef short               _upi_s16_;
typedef int                 _upi_s32_;
typedef long long           _upi_s64_;
typedef char                _upi_bool_;

#define _UPI_TRUE_      (1)
#define _UPI_FALSE_     (0)
#define _UPI_NULL_      (0)

#define UG31XX_LIB_DEBUG_MSG      ///< [AT-PM] : Enable to enable debug message in linux kernel ; 07/17/2013

#if defined (uG31xx_OS_WINDOWS)

  #pragma pack(push)
  #pragma pack(1)

  #include <windows.h>  
  #include "../../uG31xx_I2C_DLL/uG3100Dll/uG31xx_I2C.h"
  #include <assert.h>
  #include "wDebug.h"

  #define EXPORTS _declspec(dllexport)

#else   ///< else of defined (uG31xx_OS_WINDOWS)

  #ifdef  uG31xx_BOOT_LOADER

    #include "ug31xx_boot_i2c.h"

//    #define UPI_UBOOT_DEBUG_MSG

    #define UPI_BOOT_STATUS_FCC_IS_0            (1<<0)
    #define UPI_BOOT_STATUS_IC_IS_NOT_ACTIVE    (1<<1)
    #define UPI_BOOT_STATUS_WRONG_PRODUCT_TYPE  (1<<2)
    #define UPI_BOOT_STATUS_FC                  (1<<3)
    
    #define DEFAULT_TIME_TICK               (0xffffffff)
    #define RESET_COULOMB_COUNTER_DELTA_CAP (10)

  #else   ///< else of uG31xx_BOOT_LOADER

    #ifdef  ANDROID_SHELL_ALGORITHM

      #include <stdio.h>
      #include <stdlib.h>
      #include <sys/types.h>
      #include <time.h>
      #include <stdarg.h>
      #include <cutils/klog.h>
      
      #include <cutils/logd.h>
      #include <cutils/sockets.h>
      #include <ctype.h>
      
    #else   ///< else of ANDROID_SHELL_ALGORITHM
    
      #ifndef BUILD_UG31XX_LIB

        #include <linux/module.h>
        #include <linux/delay.h>
        #include <linux/fs.h>
        #include <linux/i2c.h>
        #include <linux/slab.h>
        #include <linux/jiffies.h>
        #include <linux/err.h>
        #include <linux/kernel.h>
        #include <asm/uaccess.h>
        #include <asm/unaligned.h>
      
      #endif  ///< end of BUILD_UG31XX_LIB
    
    #endif  ///< end of ANDROID_SHELL_ALGORITHM
    
  #endif  ///< end of uG31xx_BOOT_LOADER

  #define EXPORTS 
  
  #define GGBX_FILE_TAG         (0x5F47475F) // _GG_
  #define GGBX_FACTORY_FILE_TAG (0x5F67675F) // _gg_
  
#endif  ///< end of defined (uG31xx_OS_WINDOWS)

/// ===================================
/// [AT-PM] : For read / write file operation ; 07/12/2013
/// ===================================

#ifdef  uG31xx_OS_WINDOWS

  extern _upi_bool_ is_file_exist(const wchar_t *filename);
  extern _upi_bool_ create_backup_file(const wchar_t *filename, _upi_u8_ *data, _upi_u8_ size);
  extern _upi_bool_ read_backup_file(const wchar_t *filename, _upi_u8_ *data, _upi_u32_ size);
  extern _upi_bool_ write_backup_file(const wchar_t *filename, _upi_u8_ *data, _upi_u32_ size);
  
#else   ///< else of uG31xx_OS_WINDOWS

  #define UG31XX_KERNEL_FILE_EXIST      (1<<0)
  #define UG31XX_KERNEL_FILE_READ       (1<<1)
  #define UG31XX_KERNEL_FILE_WRITE      (1<<2)
  #define UG31XX_KERNEL_FILE_VERSION    (1<<3)
  #define UG31XX_KERNEL_FILE_FINISH     (1<<4)
  #define UG31XX_USER_FILE_EXIST        (1<<5)
  #define UG31XX_USER_FILE_READ         (1<<6)
  #define UG31XX_USER_FILE_WRITE        (1<<7)

  #ifndef  uG31xx_BOOT_LOADER
  
    extern _upi_bool_ is_file_exist(char *filename);
    extern _upi_bool_ create_backup_file(char *filename, _upi_u8_ *data, _upi_u8_ size);
    extern _upi_bool_ read_backup_file(char *filename, _upi_u8_ *data, _upi_u32_ size);
    extern _upi_bool_ write_backup_file(char *filename, _upi_u8_ *data, _upi_u32_ size);
    
    extern void set_shell_ap_name(char *apname);
    extern _upi_u8_ get_file_op_status(void);
    extern _upi_u8_ set_file_op_status_bit(_upi_u8_ bit_sts);
    extern _upi_u8_ clear_file_op_status_bit(_upi_u8_ bit_sts);

  #endif  ///< end of uG31xx_BOOT_LOADER
  
#endif  ///< end of uG31xx_OS_WINDOWS

/// ===================================
/// [AT-PM] : For memory operation ; 07/12/2013
/// ===================================

#ifdef  uG31xx_BOOT_LOADER

  #ifdef  BUILD_UG31XX_LIB

    #define memset
    #define memcpy
    #define malloc
    #define free
    
  #endif  ///< end of BUILD_UG31XX_LIB

#endif  ///< end of uG31xx_BOOT_LOADER

extern void upi_free(void *obj);
extern void *upi_malloc(_upi_u32_ size);
extern void upi_memcpy(void *dest, void *src, _upi_u32_ size);
extern _upi_u32_ upi_memcmp(void *s1, void *s2, _upi_u32_ size);
extern void upi_memset(void *ptr, _upi_u8_ value, _upi_u32_ size);

/// ===================================
/// [AT-PM] : For system time operation ; 07/12/2013
/// ===================================

extern void SleepMiniSecond(_upi_u32_ msec);

#ifndef uG31xx_OS_WINDOWS

  extern _upi_u32_ GetTickCount(void);
  extern _upi_u32_ GetSysTickCount(void);

#endif  ///< end of uG31xx_OS_WINDOWS

/// ===================================
/// [AT-PM] : For debug message operation ; 07/12/2013
/// ===================================

#define LOG_LEVEL_ERROR       (0)
#define LOG_LEVEL_INFO        (1)
#define LOG_LEVEL_NOTICE      (2)
#define LOG_LEVEL_DEBUG       (3)

extern _upi_u8_ Ug31DebugEnable;

#if defined (uG31xx_OS_WINDOWS)

  #define  _L(X) __L(X)
  #define __L(X) L##X

  #define DEBUG_FILE      (_T("uG3105"))
  #define __func__        (_T(__FUNCTION__))

  #define UG31_LOGE(...)  wDebug::LOGE(DEBUG_FILE, 0, _T(__VA_ARGS__));
  #define UG31_LOGI(...)  wDebug::LOGE(DEBUG_FILE, 0, _T(__VA_ARGS__));
  #define UG31_LOGN(...)  wDebug::LOGE(DEBUG_FILE, 0, _T(__VA_ARGS__));
  #define UG31_LOGD(...)  wDebug::LOGE(DEBUG_FILE, 0, _T(__VA_ARGS__));

  #define ug31_printk(...)

#else   ///< else of defined (uG31xx_OS_WINDOWS)

  #ifdef  uG31xx_BOOT_LOADER

    #ifdef  BUILD_UG31XX_LIB

      #define printf

    #endif  ///< end of BUILD_UG31XX_LIB
   
    #define UG31_LOGE(...)  printf(__VA_ARGS__)
    #define UG31_LOGI(...)  printf(__VA_ARGS__)
    #define UG31_LOGN(...)  printf(__VA_ARGS__)
    #define UG31_LOGD(...)  printf(__VA_ARGS__)

  #else   ///< else of uG31xx_BOOT_LOADER
    
    #define UG31_LOGE(...)  ug31_printk(LOG_LEVEL_ERROR, "<UG31/E>" __VA_ARGS__)
    #define UG31_LOGI(...)  ug31_printk(LOG_LEVEL_INFO, "<UG31/I>" __VA_ARGS__)
    #define UG31_LOGN(...)  ug31_printk(LOG_LEVEL_NOTICE, "<UG31/N>" __VA_ARGS__)
    #define UG31_LOGD(...)  ug31_printk(LOG_LEVEL_DEBUG, "<UG31/D>" __VA_ARGS__)
    
    extern int ug31_printk(int level, const char *fmt, ...);
    
  #endif  ///< end of uG31xx_BOOT_LOADER
  
#endif  ///< end of defined (uG31xx_OS_WINDOWS)

/// ===================================
/// [AT-PM] : For string operation ; 07/12/2013
/// ===================================

#ifndef uG31xx_OS_WINDOWS

  extern _upi_u32_ upi_strlen(char *stream);

#endif  ///< end of uG31xx_OS_WINDOWS

/// ===================================
/// [AT-PM] : For I2C operation ; 07/13/2013
/// ===================================

#ifndef uG31xx_OS_WINDOWS

#ifndef uG31xx_BOOT_LOADER

  extern void API_I2C_Init(void *client);
  extern _upi_bool_ API_I2C_Read(_upi_bool_ bSecurityMode, _upi_bool_ bHighSpeedMode, _upi_bool_ bTenBitMode, _upi_u16_ readAddress, _upi_u8_ readLength, _upi_u8_ *pReadDataBuffer);
  extern _upi_bool_ API_I2C_Write(_upi_bool_ bSecurityMode, _upi_bool_ bHighSpeedMode, _upi_bool_ bTenBitMode, _upi_u16_ writeAddress, _upi_u8_ writeLength, _upi_u8_ *pWriteData);

#endif  ///< end of uG31xx_BOOT_LOADER

#endif  ///< end of uG31xx_OS_WINDOWS

