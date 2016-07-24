/*
 * ISH platform-specific definitions
 *
 * Copyright (c) 2012-2014, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#ifndef PLATFORM_CONFIG__H
#define PLATFORM_CONFIG__H

/* Build ID string */
#define	BUILD_ID	"0038-sony-noprintk-sensorhal"

#if 0
/* Define if running on VirtualBox - may solve imprecise timer emulation problems */
#define	HOST_VIRTUALBOX	1
#endif

#if 0
/* Timer-polling workaround for DUTs with non-functional interrupts reporting */
#define	TIMER_POLLING	1
#endif

#define	REVISION_ID_CHT_A0	0x6
#define	REVISION_ID_CHT_A0_SI	0x0
#define	REVISION_ID_CHT_B0	0xB0

#define	REVISION_ID_SI_MASK	0x70

/* For buggy (pre-)silicon, select model rather than retrieve it */
#if 0
/* If defined, will support A0 only, will not check revision ID */
#define	SUPPORT_A0_ONLY	1

#else

#if  0
/* If defined, will support B0 only, will not check revision ID */
#define	SUPPORT_B0_ONLY	1
#endif
#endif

#if defined(SUPPORT_A0_ONLY) && defined(SUPPORT_B0_ONLY)
#error Only one of SUPPORT_A0_ONLY and SUPPORT_B0_ONLY may be defined
#endif

/* D3 RCR */
#define	D3_RCR	1

/* Define in order to force FW-initated reset */
#define	FORCE_FW_INIT_RESET	1

#endif /* PLATFORM_CONFIG__H*/
