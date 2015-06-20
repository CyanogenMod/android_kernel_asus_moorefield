#ifndef _GLOBAL_H_
#define _GLOBAL_H_

//#define UG31XX_RESET_DATABASE     ///< [AT-PM] : DEFAULT off ; 04/13/2013

#ifdef  uG31xx_OS_WINDOWS

#define ALIGNED_ATTRIBUTE

#define ENABLE_BQ27520_SW_CMD

#else   ///< else of uG31xx_OS_WINDOWS

#ifdef  uG31xx_ALIGNED_4

#define ALIGNED_ATTRIBUTE __attribute__((aligned(4)))

#else   ///< else of uG31xx_ALIGNED_4

#define ALIGNED_ATTRIBUTE __attribute__((packed))

#endif  ///< end of uG31xx_ALIGNED_4

#endif  ///< end of uG31xx_OS_WINDOWS

//#define ENABLE_NTC_CHECK

#endif
