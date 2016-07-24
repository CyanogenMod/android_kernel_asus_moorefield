/*! @file vfsSpiDrv.h
*******************************************************************************
**  SPI Driver Interface Functions
**
**  This file contains the SPI driver interface functions.
**
**  Copyright 2013-2014 Synaptics, Inc. All Rights Reserved.
**
**
*/

#ifndef VFSSPIDRV_H_
#define VFSSPIDRV_H_

#define PR_ERR(fmt, args...)   pr_err("synafpspi:"fmt, ## args)
//#if DEBUG
#define PR_DEBUG(fmt, args...) pr_debug("synafpspi:"fmt, ## args)
#define PR_INFO(fmt, args...)  pr_info("synafpspi:"fmt, ## args)
//#else /* DEBUG */
//#define PR_DEBUG(fmt, args...)
//#define PR_INFO(fmt, args...)
//#endif /* DEBUG */

//#define SYNA_PART_NAME "syna_fingerprint"
#define SYNA_PART_NAME "fingerprint"

#define SYNA_DEV_NAME  "vfsspi"

/* DRDY GPIO pin number */
#define VFSSPI_DRDY_PIN		105 //133
/* Sleep GPIO pin number */
#define VFSSPI_SLEEP_PIN	106 //132

#define DRDY_ACTIVE_STATUS		1
#define BITS_PER_WORD			8
#define DRDY_IRQ_FLAG			IRQF_TRIGGER_RISING

/* Max baud rate supported by Validity sensor. */
#define MAX_BAUD_RATE   12000000

/* The coefficient which is multiplying with value retrieved from the
 * VFSSPI_IOCTL_SET_CLK IOCTL command for getting the final baud rate. */
#define BAUD_RATE_COEF  1000

/* Maximum transfer size */
#define DEFAULT_BUFFER_SIZE  (8 * 4096)

/* Indicates DRDY IRQ enabled or disabled */
#define DRDY_IRQ_ENABLE			1
#define DRDY_IRQ_DISABLE		0

#endif /* VFSSPIDRV_H__ */
