/*
 * Dongle WL Header definitions
 *
 * $ Copyright Open Broadcom Corporation $
 *
 *
 * <<Broadcom-WL-IPTag/Open:>>
 *
 * $Id: dngl_wlhdr.h 514727 2014-11-12 03:02:48Z $
 */

#ifndef _dngl_wlhdr_h_
#define _dngl_wlhdr_h_

typedef struct wl_header {
    uint8   type;           /* Header type */
    uint8   version;        /* Header version */
	int8	rssi;			/* RSSI */
	uint8	pad;			/* Unused */
} wl_header_t;

#define WL_HEADER_LEN   sizeof(wl_header_t)
#define WL_HEADER_TYPE  0
#define WL_HEADER_VER   1
#endif /* _dngl_wlhdr_h_ */
