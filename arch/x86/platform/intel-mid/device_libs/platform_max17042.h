/*
 * platform_max17042.h: max17042 platform data header file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _PLATFORM_MAX17042_H_
#define _PLATFORM_MAX17042_H_

#define	NTC_47K_TGAIN			0xE4E4
#define	NTC_47K_TOFF			0x2F1D
#define	NTC_47K_TH05_TGAIN		0xDA1F
#define	NTC_47K_TH05_TOFF		0x38C7
#define	NTC_10K_B3435K_TDK_TGAIN	0xE4E4
#define	NTC_10K_B3435K_TDK_TOFF		0x2218
#define	NTC_10K_NCP15X_TGAIN		0xE254
#define	NTC_10K_NCP15X_TOFF		0x2ACF
#define NTC_10K_MURATA_TGAIN		0xE39C
#define NTC_10K_MURATA_TOFF		0x2673

extern void *max17042_platform_data(void *info) __attribute__((weak));
/*
*Safe memcpy
*
*This function validates the source/destination pointers along with
*respective sizes before performing a memcpy, but doesn't check for
*overlapping and maximum destination-size
*
Returns 0 if successfull, else appropriate errors.
*/

static inline int memcpy_safe(void *dest, size_t dmax, const void *src, size_t smax)
{
	char *dp = dest;
	const char *sp = src;
	size_t count = smax;
	if ((dp == NULL) ||  (sp == NULL))
		return -EFAULT;

	if ((dmax == 0) || (smax == 0))
		return -EINVAL;

	if (smax > dmax)
		return -EOVERFLOW;

	while (count--)
		*dp++ = *sp++;
	return 0;
}
#endif
